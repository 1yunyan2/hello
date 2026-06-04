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
/**
 * @file application.h
 * @brief 应用层初始化入口头文件
 * 负责整合 BSP、协议、唤醒词等模块的统一启动
 */

#include "bsp/bsp_board.h"
#include "protocol/mqtt_protocol.h"
#include "wake_word/custom_wake_word.h"
#include "application.h"
#include "session/session.h"
#include "audio/audio_processor.h"
#include "bsp/servo_manager.h"
#include "bsp/bsp_ota.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui/ui_port.h"
#include "ui/interaction.h"
#include "ui/reminder.h"
#include "ui/standby.h"
#define TAG "Application"

/** @brief 打印当前内部 SRAM 剩余空间（追踪初始化内存消耗） */
#define PRINT_INTERNAL_HEAP \
    ESP_LOGI(TAG, "[heap] internal free: %lu B", esp_get_free_internal_heap_size())

// ─── CPU 占用诊断 ───────────────────────────────────────────────────────────
// 依赖 sdkconfig：
//   CONFIG_FREERTOS_USE_TRACE_FACILITY=y
//   CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
//   CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
// 调试完毕可注释掉 application_init() 末尾的任务创建语句。
#define CPU_STATS_PERIOD_MS 10000 // 打印周期：10 秒
#define CPU_STATS_BUF_SIZE 2048   // ~18 任务 × 70 字节/行 富余

/**
 * @brief 周期性打印每个任务的运行时间百分比
 *
 * 输出格式（vTaskGetRunTimeStats）：
 *   任务名         绝对运行时间    占总时间 %
 *   IDLE0          xxxxx           45%      ← CPU0 空闲率，100%-此值 = CPU0 负载
 *   IDLE1          xxxxx           30%      ← CPU1 空闲率，100%-此值 = CPU1 负载
 *   encoder_task   xxxxx           12%
 *   ...
 *
 * 注意：%CPU 是"占总 runtime 计数"的百分比；双核累计可超过 100%。
 *       看单核负载：IDLE0/IDLE1 反向推算更直观。
 */
static void cpu_stats_task(void *arg)
{
    char *buf = (char *)heap_caps_malloc(CPU_STATS_BUF_SIZE,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL)
    {
        ESP_LOGE("CPU_STATS", "诊断缓冲分配失败，任务退出");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(CPU_STATS_PERIOD_MS));
        memset(buf, 0, CPU_STATS_BUF_SIZE);
        vTaskGetRunTimeStats(buf);
        // 用 printf 直输，避免 ESP_LOGI 多行截断
        printf("\n=== CPU Runtime Stats ===\n%s===========================\n", buf);
    }
}

#include <dirent.h>  // 必须包含这个，才能使用 DIR 和 readdir
#include <string.h>  // memset 用于 cpu_stats_task
#include <stdio.h>   // printf 用于 CPU 占用诊断输出
#include "esp_log.h" // 确保能使用 ESP_LOGI 等日志宏
// 一个函数扫描所有资源
void scan_production_assets(const char *root_path)
{
    ESP_LOGI("FS", "========================================");
    ESP_LOGI("FS", "🔍 开始资源完整性校验: %s", root_path);

    const char *sub_folders[] = {"/gif", "/audio"}; // 你关心的子目录

    for (int i = 0; i < 2; i++)
    {
        char full_path[128];
        snprintf(full_path, sizeof(full_path), "%s%s", root_path, sub_folders[i]);

        DIR *dir = opendir(full_path);
        if (dir == NULL)
        {
            ESP_LOGW("FS", "⚠️ 未发现目录: %s (请检查 storage.bin 打包结构)", full_path);
            continue;
        }

        struct dirent *de;
        while ((de = readdir(dir)) != NULL)
        {
            // 忽略系统隐藏文件
            if (de->d_name[0] == '.')
                continue;
            ESP_LOGI("FS", "[%s] 🚀 发现资源: %s", sub_folders[i], de->d_name);
        }
        closedir(dir);
    }
    ESP_LOGI("FS", "========================================");
}
void debug_root_files(void)
{
    DIR *dir = opendir("/S");
    if (!dir)
    {
        ESP_LOGE("DEBUG", "连 /S 都打不开！");
        return;
    }
    struct dirent *de;
    int count = 0;
    printf("--- 物理搜索开始 ---\n");
    while ((de = readdir(dir)) != NULL)
    {
        printf("[%d] 找到条目: %s (类型: %d)\n", count++, de->d_name, de->d_type);
    }
    if (count == 0)
    {
        printf("🚨 警告：外挂 Flash 根目录下空无一物！\n");
    }
    closedir(dir);
}
// ═══════════════════════════════════════════════════════════════════════════════
// 2. 唤醒提示音
// ═══════════════════════════════════════════════════════════════════════════════

// ─── 唤醒提示音 ───────────────────────────────────────────────────────────
// 880Hz 方波，持续 300ms，通过 ES8311 DAC 输出到扬声器
// 方波生成无需浮点运算，在 ESP32-S3 上 CPU 占用极低
static void play_wake_tone(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    // Codec 设备必须已初始化（audio_init 完成后才调用此函数，正常不会为 NULL）
    if (!board || !board->codec_dev)
        return;

    // 音调参数
    const int sample_rate = 16000;  // 16kHz采样率
    const int freq_hz = 880;        // 880Hz = 音乐 A5，清脆易辨
    const int duration_ms = 300;    // 持续 300ms，简短提示
    const int16_t amplitude = 8000; // 幅度（0~32767，8000 约为 24% 满幅，适中音量）

    // 计算方波半周期采样点数：half_period = 采样率 / 频率 / 2
    // 880Hz → 半周期 = 16000 / 880 / 2 ≈ 9 个采样点
    const int half_period = sample_rate / freq_hz / 2;
    const int total_samples = sample_rate * duration_ms / 1000; // = 4800 个采样点

// 使用栈上小缓冲区分块写入，避免 heap 分配大块内存
#define TONE_CHUNK 256
    int16_t buf[TONE_CHUNK];
    int written = 0; // 已生成的采样点计数
    int phase = 0;   // 方波相位计数（0~2×half_period 循环）

    while (written < total_samples)
    {
        // 本次写入的采样点数（最后一块可能不满 TONE_CHUNK）
        int n = total_samples - written;
        if (n > TONE_CHUNK)
            n = TONE_CHUNK;

        // 生成方波：前半周期为正幅度，后半周期为负幅度
        for (int i = 0; i < n; i++)
        {
            buf[i] = (phase < half_period) ? amplitude : -amplitude;
            // 相位推进并循环归零
            if (++phase >= half_period * 2)
                phase = 0;
        }

        // 将 PCM 数据写入 Codec TX 通道（阻塞直到 DMA 接收完本块数据）
        esp_codec_dev_write(board->codec_dev, buf, n * sizeof(int16_t));
        written += n;
    }
}

// 舵机循环测试任务（独立运行，不阻塞 LVGL）
static void servo_test_task(void *arg)
{
    while (1)
    {
        bsp_servo_move_all_parallel(60.0f, 60.0f, 120.0f, SERVO_SPEED_MID); // 同步运动示例
        vTaskDelay(pdMS_TO_TICKS(500));
        bsp_servo_move_all_parallel(120.0f, 120.0f, 60.0f, SERVO_SPEED_MID); // 同步运动示例
        vTaskDelay(pdMS_TO_TICKS(500));
        bsp_servo_move_all_parallel(90.0f, 90.0f, 90.0f, SERVO_SPEED_MID); // 同步运动示例
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. 唤醒词回调
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 唤醒词引擎识别命中后的回调
 *
 * 仅做转发，真正的会话开启逻辑在 session_on_wake_word 内部。
 */
static void wake_word_callback(const char *wake_word_display)
{
    ESP_LOGW("WAKE_UP", "唤醒词触发: [%s]", wake_word_display);

    standby_notify_activity(); // 唤醒命中视为活动，刷新待机倒计时

    // 播放 880Hz 提示音给用户听觉反馈
    // play_wake_tone();

    session_on_wake_word(wake_word_display);
}

//
// ═══════════════════════════════════════════════════════════════════════════════
// 5. 应用主初始化序列
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

    /* ⚠️⚠️⚠️ 关键启动顺序：此段必须保持在 application_init 最开头，
     *    早于 bsp_flash_init / audio_init / bsp_lcd_init 等所有 I2C/SPI 外设初始化。
     *
     * 背景：GPIO 14 是 ESP32-S3 的 FSPIWP/SUBSPIWP 复用脚（官方手册 I1 字段表示
     *       上电默认配置为「输入 + 弱上拉到 1」，约 45kΩ → 3.3V）。
     *       本项目 PCB 焊死把左臂舵机信号线接到 GPIO 14（无法改换引脚）。
     *
     * 现象：若不压低 → 上电瞬间舵机信号被弱上拉拉高 → 舵机识别为短脉冲反复抖动
     *       → 拉走大电流 → 共用 3.3V 电源轨压降 → ES8311/触摸 IC 报 NACK，
     *       同时 LEDC TIMER_0 共用的 GPIO 4/9 舵机连锁失灵。
     *
     * 修复：用 gpio_config 显式禁用上下拉/中断并主动输出 0V，舵机识别为「无脉冲」
     *       保持静止。后续 bsp_board_servo_init 启用 LEDC 时会通过 GPIO Matrix
     *       重新路由 PWM 信号，覆盖此处输出状态，无冲突。
     *
     * 详见 memory/bugs/BUG-015.md */
    gpio_config_t io_conf_g14 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, // 显式关上拉，覆盖 FSPIWP 默认 I1=1
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE, // 显式关中断，避免 SUBSPIWP 残留中断
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_g14));
    gpio_set_level(GPIO_NUM_14, 0); // 主动输出 0V，停止舵机误抖动
    bsp_flash_init();
    PRINT_INTERNAL_HEAP;
    debug_root_files();
    scan_production_assets("/S"); // 扫描 /S 目录下的所有资源
    if (access("/S/assets/gif/one.gif", F_OK) == 0)
        printf("路径 A 物理存在！\n");
    if (access("/S/gif/one.gif", F_OK) == 0)
        printf("路径 B 物理存在！\n");
    /* ── 步骤 1: BSP 单例 ──────────────────────────────────────────────────── */
    bsp_board_t *bsp_board = bsp_board_get_instance();

    /* ── 步骤 2: NVS Flash ─────────────────────────────────────────────────── */
    bsp_board_nvs_init(bsp_board);
    PRINT_INTERNAL_HEAP;
    /* ── 步骤 5: WiFi（阻塞直至获取 IP 或彻底失败后重启）─────────────────── */
    bsp_board_wifi_main(bsp_board);
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 9.5: 电池电压监控（VBAT_ADC）── */
    // 若 BSP_BAT_ADC_PIN 未配置（仍为占位 -1），bsp_battery_init 会返回错误，
    // 仅打印警告，不影响其他流程。后续硬件确认引脚后即可自动启用。
    esp_err_t bat_ret = bsp_battery_init();
    if (bat_ret == ESP_OK)
    {
        bsp_battery_start_task(NULL); // 暂不接低电回调，UI 自身已带变色提示
        // bsp_battery_start_log_task(); // 新增：每 5s 打印一次电池电压/电量，便于调试
        xEventGroupSetBits(bsp_board->board_status, BATTERY_BIT);
        ESP_LOGI(TAG, "电池监控已启动");
    }
    else
    {
        ESP_LOGW(TAG, "电池监控未启用 (%s)，UI 电量将显示 --%%", esp_err_to_name(bat_ret));
    }
    PRINT_INTERNAL_HEAP;
    bsp_board_lcd_init(bsp_board); // LCD 初始化（当前未自动置位 LCD_BIT，后续可根据需求调整）
    PRINT_INTERNAL_HEAP;
    ui_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    PRINT_INTERNAL_HEAP;
    bsp_board_lcd_on(bsp_board); // 临时上移到 ui_init 之前
    // 提醒系统初始化（含 MOCK_TIME 模式下的系统时间设置）
    reminder_init(NULL); // NULL = 暂无 TTS 回调，后续接入 session 层时替换
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 3: 音频硬件 + 采集任务 ──────────────────────────────────────── */
    audio_init(bsp_board);
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 4: 唤醒词引擎 ────────────────────────────────────────────────── */
    wake_word_init(wake_word_callback);
    wake_word_start(); /* BUG-024 修复：显式启动，不依赖隐式启动 */
    PRINT_INTERNAL_HEAP;
    printf("唤醒词引擎已启动，等待触发...\n");
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
    // 舵机测试任务（独立跑，不影响 LVGL 刷新）
    // xTaskCreatePinnedToCoreWithCaps(
    //     servo_test_task,
    //     "servo_test",
    //     4096,
    //     NULL,
    //     5,
    //     NULL,
    //     tskNO_AFFINITY,
    //     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // 6. 创建触摸扫描任务（栈分配在PSRAM，节省内部SRAM）
    ret = xTaskCreatePinnedToCoreWithCaps(
        touch_scan_task,
        "touch_scan",
        8192,
        NULL,
        4, // 优先级略低于舵机和音频
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "创建触摸扫描任务失败！");
    }
    else
    {
        ESP_LOGI(TAG, "触摸扫描任务创建完成");
    }

    /* ── 步骤 10.5: 无活动待机模块（依赖 LCD/唤醒词/舵机管理器均已就绪）──── */
    standby_init();
    PRINT_INTERNAL_HEAP;

    /* ── 步骤 7: 会话模块（WebSocket 预连接）─────────────────────────────── */

    session_init("ws://122.224.191.2:4888/ws/omni");
    // session_init(" ws://122.224.191.2:4888/ws/voice");

    // PRINT_INTERNAL_HEAP;

    /* ── 步骤 8: CPU 占用诊断任务（调试用，可注释掉）─────────────────────── */
    // 低优先级、tskNO_AFFINITY、栈在 SPIRAM，对业务无干扰
    // xTaskCreatePinnedToCoreWithCaps(
    //     cpu_stats_task,
    //     "cpu_stats",
    //     4096,
    //     NULL,
    //     1,                  // 最低优先级（仅次于 IDLE）
    //     NULL,
    //     tskNO_AFFINITY,
    //     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // ESP_LOGI(TAG, "CPU 占用诊断任务已启动，每 10 秒打印一次");

    /* ── 步骤 9: OTA 验证（必须在所有初始化完成后调用）──────────────────── */
    // 若当前是刚 OTA 升级完首次启动，会进入 PENDING_VERIFY 状态：
    //   - 调用 esp_ota_mark_app_valid_cancel_rollback() 防止 Bootloader 回滚
    //   - 把 NVS 中的 pending_ver 提升为 committed_ver
    // 若启动前期崩溃（未到这里），Bootloader 下次启动会自动回滚到旧固件
    ESP_LOGI(TAG, "当前固件版本: %s", bsp_ota_get_current_version());
    bsp_ota_mark_valid();

    ESP_LOGI(TAG, "application_init 1.0.4.1 完成，系统就绪");
}
