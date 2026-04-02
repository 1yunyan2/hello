#include "application.h"
#include "session/session.h"

// ─── 唤醒提示音 ───────────────────────────────────────────────────────────
// 880Hz 方波，持续 1 秒，通过 ES8311 DAC 输出到扬声器
// 方波生成无需浮点运算，在 ESP32-S3 上 CPU 占用极低
static void play_wake_tone(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    // Codec 设备必须已初始化（audio_init 完成后才调用此函数，正常不会为 NULL）
    if (!board || !board->codec_dev)
        return;

    // 音调参数
    const int sample_rate = BSP_CODEC_SAMPLE_RATE; // 16000 Hz
    const int freq_hz = 880;                       // 880Hz = 音乐 A5，清脆易辨
    const int duration_ms = 1000;                  // 持续 1 秒
    const int16_t amplitude = 6000;                // 幅度（0~32767，6000 约为 18% 满幅，适中音量）

    // 计算方波半周期采样点数：half_period = 采样率 / 频率 / 2
    // 880Hz → 半周期 = 16000 / 880 / 2 ≈ 9 个采样点
    const int half_period = sample_rate / freq_hz / 2;
    const int total_samples = sample_rate * duration_ms / 1000; // = 16000 个采样点

// 使用栈上小缓冲区分块写入，避免 heap 分配 32KB 大块
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

// ─── 唤醒词触发回调 ───────────────────────────────────────────────────────
// 由唤醒词引擎在识别到命令词后，从音频投喂上下文中调用
static void wake_word_callback(const char *wake_word_display)
{
    ESP_LOGW("WAKE_UP", "唤醒词触发: [%s]", wake_word_display);

    // 播放 880Hz 提示音给用户听觉反馈（提示音结束前麦克风已停止向唤醒引擎投喂）
    play_wake_tone();
    // bsp_wake_word_start();
    // 将控制权交给会话模块：建立 WebSocket、启动编解码管道
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
    session_init(NULL);

    // 步骤 7：初始化电源监测功能
    // power_monitor_init();
}
