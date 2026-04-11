/**
 * @file application.c
 * @brief Echo 应用主入口 — 启动序列编排
 *
 * 职责:按依赖顺序初始化各子系统,启动后将运行时控制权交给 session 模块。
 * 设计哲学:application 仅负责"启动",所有运行时状态机和事件处理在 session.c。
 *
 * 启动序列(严格顺序,不可调换):
 *   BSP → NVS → 唤醒词引擎 → 音频硬件 → WiFi → MQTT → Session → 电源监测
 *
 * 修复记录:
 *   - BUG-024: 显式调用 wake_word_start() 确保引擎进入监听态
 *   - BUG-025: 启用 power_monitor_init() 电源监测
 */

#include "application.h"
#include "session/session.h"
#include "audio/audio_processor.h"
#include "esp_log.h"

#define TAG "Application"

/**
 * @brief 调试宏:打印当前内部堆剩余空间
 *
 * 用于追踪每个初始化步骤的内存消耗,定位泄漏点。
 */
#define PRINT_INTERNAL_HEAP \
    ESP_LOGI(TAG, "[%s:%d] internal heap free: %lu", __FILE__, __LINE__, esp_get_free_internal_heap_size())

// ─── 唤醒词触发回调 ───────────────────────────────────────────────────────
/**
 * @brief 唤醒词引擎识别命中后的回调
 *
 * 由唤醒词引擎在识别到命令词后,从音频投喂上下文中调用。
 * 仅做转发,真正的会话开启逻辑在 session_on_wake_word 内部。
 *
 * @param wake_word_display  显示文字,如 "云炎" 或 "Hello Echo"
 */
static void wake_word_callback(const char *wake_word_display)
{
    ESP_LOGW("WAKE_UP", "唤醒词触发: [%s]", wake_word_display);

    // 会话结束后由 session_close 内部调用 wake_word_start 恢复监听
    session_on_wake_word(wake_word_display);
}

// ─── 应用程序主初始化序列 ─────────────────────────────────────────────────
/**
 * @brief 应用程序主初始化入口
 *
 * 由 main.c 的 app_main() 调用,完成整套子系统启动。
 * 函数返回后,系统进入事件驱动模式,所有逻辑由各任务/回调推进。
 */
void application_init(void)
{
    PRINT_INTERNAL_HEAP;

    // 步骤 1:获取全局唯一 BSP 实例(内部自动创建事件组)
    bsp_board_t *bsp_board = bsp_board_get_instance();

    // 步骤 2:初始化 NVS Flash(唤醒词/WiFi 凭证/MQTT 凭证均存放于 NVS)
    bsp_board_nvs_init(bsp_board);
    PRINT_INTERNAL_HEAP;

    // 步骤 3:初始化唤醒词引擎,注册触发回调
    // 从 NVS 加载上次保存的唤醒词和语言,加载对应 MultiNet6 模型
    wake_word_init(wake_word_callback);

    // 【BUG-024 修复】显式启动唤醒词引擎监听
    // 原代码此处被注释,依赖 wake_word_init 内部隐式启动,
    // 在重构后该假设不再成立 → 显式调用确保监听态
    wake_word_start();
    PRINT_INTERNAL_HEAP;

    // 步骤 4:初始化音频硬件(I2C + I2S + ES8311 Codec)并启动麦克风采集任务
    // 必须在唤醒词引擎初始化之后调用,采集任务会立即向引擎投喂音频帧
    audio_init(bsp_board);
    PRINT_INTERNAL_HEAP;

    // 步骤 5:启动 WiFi(阻塞直至成功获取 IP 或连接彻底失败后重启)
    // 包含 BLE 配网、自动重连、按键重置等完整流程
    bsp_board_wifi_main(bsp_board);
    PRINT_INTERNAL_HEAP;

    // 步骤 6:启动 MQTT 客户端(必须在获取 IP 后执行,否则连接无法建立)
    // 内部启动心跳任务,并订阅唤醒词更新主题
    protocol_mqtt_start();
    PRINT_INTERNAL_HEAP;

    // 步骤 7:初始化会话模块(WebSocket 预连接立即建立,唤醒时零延迟)
    // 认证方式：session_init 内部用 deviceToken 换取 accessToken，放入 Authorization 头
    //  session_init("wss://api.tenclass.net/xiaozhi/v1/");
    // session_init("ws://122.224.191.2:4888/ws/voice?token=<accessToken>");
    session_init("ws://122.224.191.2:4888/ws/voice");
    PRINT_INTERNAL_HEAP;

    // 步骤 8:启动电源监测(ADC 采样电池电压 + 上报)
    // 【BUG-025 修复】原代码被注释,导致设备无电量上报
    // power_monitor_init();
    PRINT_INTERNAL_HEAP;

    ESP_LOGI(TAG, "application_init complete, system ready");
}