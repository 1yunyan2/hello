#include "application.h"
#include "session/session.h"
#define PRINT_INTERNAL_HEAP \
    ESP_LOGE(TAG, "[%s:%d] heap size: %lu", __FILE__, __LINE__, esp_get_free_internal_heap_size())

// ─── 唤醒词触发回调 ───────────────────────────────────────────────────────
// 由唤醒词引擎在识别到命令词后，从音频投喂上下文中调用
static void wake_word_callback(const char *wake_word_display)
{
    ESP_LOGW("WAKE_UP", "唤醒词触发: [%s]", wake_word_display);

    // 会话结束后由 session_close 内部调用 bsp_wake_word_start 恢复监听
    session_on_wake_word(wake_word_display);
}

// ─── 应用程序主初始化序列 ─────────────────────────────────────────────────
void application_init(void)
{
    // 获取全局唯一 BSP 实例（内部自动创建事件组）
    bsp_board_t *bsp_board = bsp_board_get_instance();

    // 步骤 1：初始化 NVS Flash（唤醒词/WiFi 凭证/MQTT 凭证均存放于 NVS）
    bsp_board_nvs_init(bsp_board);

    // 步骤 2：初始化唤醒词引擎，注册触发回调
    // 从 NVS 加载上次保存的唤醒词和语言，加载对应 MultiNet6 模型
    bsp_wake_word_init(wake_word_callback);

    // 步骤 3：初始化音频硬件（I2C + I2S + ES8311 Codec）并启动麦克风采集任务
    // 必须在唤醒词引擎初始化之后调用，采集任务会立即向引擎投喂音频帧
    audio_init(bsp_board);

    // 步骤 4：启动 WiFi（阻塞直至成功获取 IP 或连接彻底失败后重启）
    // 包含 BLE 配网、自动重连、按键重置等完整流程
    bsp_board_wifi_main(bsp_board);

    // 步骤 5：启动 MQTT 客户端（必须在获取 IP 后执行，否则连接无法建立）
    // 内部启动心跳任务，并订阅唤醒词更新主题
    protocol_mqtt_start();

    // 步骤 6：初始化会话模块（在 WiFi 连接后初始化，URI 从 NVS 读取或用默认值）
    // 如需自定义 WebSocket 地址，将 NULL 改为 "ws://your-server:8080/audio"
    session_init("wss://api.tenclass.net/xiaozhi/v1/");

    // 步骤 7：初始化电源监测功能
    // power_monitor_init();
}
