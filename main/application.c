/**
 * @file application.c
 * @brief Echo 应用主入口 — 启动序列编排
 *
 * 职责：按依赖顺序初始化各子系统，启动后将运行时控制权交给 session 模块。
 * 设计哲学：application 仅负责"启动"，所有运行时状态机和事件处理在各自模块。
 *
 * 启动序列（严格顺序，不可调换）：
 *   BSP → NVS → 音频 → 唤醒词 → WiFi → MQTT → Session
 *   → Servo → InteractionManager → TouchScanTask → TouchDispatchTask
 *
 * 触摸事件路由（application 层策略，不放在 BSP 层）：
 *   bsp_touch.c 只负责检测事件并入队 → touch_dispatch_task 读取并映射到情绪
 *   映射表集中在此文件，便于后续按产品需求调整（不需改 BSP 或情绪引擎）
 */

#include "application.h"
#include "session/session.h"
#include "audio/audio_processor.h"
#include "bsp/servo_manager.h"
#include "ui/interaction.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define TAG "Application"

/** @brief 打印当前内部 SRAM 剩余空间（追踪初始化内存消耗） */
#define PRINT_INTERNAL_HEAP \
    ESP_LOGI(TAG, "[heap] internal free: %lu B", esp_get_free_internal_heap_size())

// ═══════════════════════════════════════════════════════════════════════════════
// 1. 触摸事件 → 情绪映射表（application 层策略，可随时调整）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 触摸事件到情绪 ID 的映射表
 *
 * 映射关系集中在 application 层，优点：
 *   - BSP 层（bsp_touch.c）不依赖任何 UI 模块，保持纯硬件抽象
 *   - 调整触摸→情绪对应关系时只改这张表，无需改底层驱动
 *   - 未来游戏模式可切换到另一张表（game_touch_map），无需修改 BSP
 */
static const struct
{
    touch_event_t event;
    robot_emotion_t emotion;
} s_touch_emotion_map[] = {
    /* 单点短按 — 基础情绪 */
    {TOUCH_EVENT_SHORT_HEAD, EMO_HAPPY},          ///< 头部轻触 → 开心
    {TOUCH_EVENT_SHORT_ABDOMEN, EMO_COMFORTABLE}, ///< 腹部轻触 → 舒服
    {TOUCH_EVENT_SHORT_BACK, EMO_TICKLISH},       ///< 背部轻触 → 怕痒

    /* 单点长按 — 进阶情绪 */
    {TOUCH_EVENT_LONG_HEAD, EMO_ACT_CUTE},   ///< 头部长按 → 撒娇
    {TOUCH_EVENT_LONG_ABDOMEN, EMO_HEALING}, ///< 腹部长按 → 治愈
    {TOUCH_EVENT_LONG_BACK, EMO_SURPRISED},  ///< 背部长按 → 惊喜

    /* 双点组合 — 特殊情绪 */
    {TOUCH_EVENT_COMBO_HEAD_ABDOMEN, EMO_SHY_RUB},          ///< 头+腹 → 害羞蹭蹭
    {TOUCH_EVENT_COMBO_HEAD_BACK, EMO_EXCITED},             ///< 头+背 → 兴奋
    {TOUCH_EVENT_COMBO_ABDOMEN_BACK, EMO_COMFORTABLE_ROLL}, ///< 腹+背 → 舒服到打滚
};
#define TOUCH_EMOTION_MAP_SIZE (sizeof(s_touch_emotion_map) / sizeof(s_touch_emotion_map[0]))

// ═══════════════════════════════════════════════════════════════════════════════
// 2. 唤醒词回调
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 唤醒词引擎识别命中后的回调
 *
 * 仅做转发，真正的会话开启逻辑在 session_on_wake_word 内部。
 */
static void wake_word_callback(const char *wake_word_display)
{
    ESP_LOGW("WAKE_UP", "唤醒词触发: [%s]", wake_word_display);
    session_on_wake_word(wake_word_display);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. 触摸任务（BSP 扫描 + 应用层 dispatch）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief touch_scan_task 的 FreeRTOS 兼容包装
 *
 * touch_scan_task 签名为 void(bsp_board_t*)，不符合 TaskFunction_t(void*)，
 * 此包装解决类型问题，同时保留 bsp_board 参数传递。
 */
static void touch_scan_wrapper(void *arg)
{
    touch_scan_task((bsp_board_t *)arg);
}

/**
 * @brief 触摸事件分发任务（application 层）
 *
 * 每 10ms 轮询一次触摸事件队列，将事件查表映射为情绪 ID，
 * 调用 ui_interaction_play() 触发完整情绪表现（舵机+震动+音效+表情）。
 *
 * 为何在 application 层而不在 bsp_touch.c：
 *   - 保持 BSP 层与 UI 层解耦
 *   - 映射策略（哪个触摸→哪个情绪）是产品决策，属于 app 层职责
 *   - 未来切换游戏模式时，只需替换分发逻辑，无需改 BSP
 *
 * @param arg 未使用
 */
static void touch_dispatch_task(void *arg)
{
    ESP_LOGI(TAG, "触摸事件分发任务启动");
    touch_event_t event;

    while (1)
    {
        if (bsp_touch_get_event(&event))
        {
            /* 查映射表，找到对应情绪后触发 */
            for (int i = 0; i < (int)TOUCH_EMOTION_MAP_SIZE; i++)
            {
                if (s_touch_emotion_map[i].event == event)
                {
                    ESP_LOGI(TAG, "触摸事件 %d → 情绪 %d", (int)event,
                             (int)s_touch_emotion_map[i].emotion);
                    ui_interaction_play(s_touch_emotion_map[i].emotion);
                    break;
                }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 4. 应用主初始化序列
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 应用程序主初始化入口（由 main.c 的 app_main() 调用）
 *
 * 函数返回后系统进入事件驱动模式，所有逻辑由各任务/回调推进。
 *
 * 初始化顺序约束：
 *   1. bsp_board_get_instance  — 必须最先（创建 EventGroup）
 *   2. bsp_board_nvs_init      — 早于所有使用 NVS 的模块
 *   3. audio_init              — 早于唤醒词引擎（feed task 立即投喂）
 *   4. wake_word_init/start    — 引擎就绪后再启动监听
 *   5. bsp_board_wifi_main     — 阻塞等待联网
 *   6. protocol_mqtt_start     — 必须在联网后
 *   7. session_init            — WebSocket 预连接
 *   8. bsp_board_servo_init    — 舵机 LEDC 硬件初始化
 *   9. servo_manager_init      — 队列化舵机管理器（依赖步骤 8）
 *  10. interaction_manager_init — 情绪矩阵 worker（依赖步骤 9）
 *  11. xTaskCreate touch_scan   — 触摸硬件扫描（依赖步骤 10 已初始化）
 *  12. xTaskCreate touch_dispatch — 触摸→情绪路由（依赖步骤 10 + 11）
 */
void application_init(void)
{
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 1: BSP 单例 ──────────────────────────────────────────────────── */
    bsp_board_t *bsp_board = bsp_board_get_instance();

    /* ── 步骤 2: NVS Flash ─────────────────────────────────────────────────── */
    bsp_board_nvs_init(bsp_board);
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 3: 音频硬件 + 采集任务 ──────────────────────────────────────── */
    audio_init(bsp_board);
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 4: 唤醒词引擎 ────────────────────────────────────────────────── */
    wake_word_init(wake_word_callback);
    wake_word_start(); /* BUG-024 修复：显式启动，不依赖隐式启动 */
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 5: WiFi（阻塞直至获取 IP 或彻底失败后重启）─────────────────── */
    bsp_board_wifi_main(bsp_board);
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 6: MQTT 客户端 ───────────────────────────────────────────────── */
    protocol_mqtt_start();
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 8: 舵机硬件初始化（LEDC/PWM）──────────────────────────────── */
    bsp_board_servo_init(bsp_board);
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 9: 舵机管理器（队列 + worker task，栈在 SPIRAM）─────────────── */
    esp_err_t ret = servo_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "servo_manager_init 失败: %s", esp_err_to_name(ret));
    }
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 10: 情绪交互管理器（情绪矩阵 + worker task，栈在 SPIRAM）────── */
    ret = interaction_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "interaction_manager_init 失败: %s", esp_err_to_name(ret));
    }
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 11: 触摸扫描任务（BSP 层，仅入队，不含 UI 逻辑）─────────────── */
    BaseType_t task_ret = xTaskCreatePinnedToCoreWithCaps(
        touch_scan_wrapper, /* 包装函数（解决类型兼容） */
        "touch_scan",       /* 任务名 */
        4096,               /* 栈大小（SPIRAM） */
        bsp_board,          /* 参数：bsp 实例 */
        4,                  /* 优先级：低于音频(7)和 session(5)，略低于 interaction(5) */
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "touch_scan_task 创建失败");
    }
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 12: 触摸事件分发任务（application 层策略，映射触摸→情绪）───── */
    task_ret = xTaskCreatePinnedToCoreWithCaps(
        touch_dispatch_task,
        "touch_dispatch",
        2048, /* 栈小（只做查表+入队，无深调用链） */
        NULL,
        4, /* 与 touch_scan 同优先级 */
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (task_ret != pdPASS)
    {
        ESP_LOGE(TAG, "touch_dispatch_task 创建失败");
    }
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 7: 会话模块（WebSocket 预连接）─────────────────────────────── */
    session_init("ws://122.224.191.2:4888/ws/voice");
    PRINT_INTERNAL_HEAP;

    ESP_LOGI(TAG, "application_init 完成，系统就绪");
    ESP_LOGI(TAG, "触摸链路: 铜箔 → touch_scan → queue → touch_dispatch → interaction_worker → 舵机/震动/表情/音效");
}
