/**
 * @file session.c
 * @brief 会话状态机
 *
 * 完整数据流：
 *   麦克风(I2S) → audio_feed_task → AFE(降噪+VAD)
 *               → afe_fetch_task → enhanced_pcm_hook → enc_input(ring)
 *               → audio_encoder_task (OPUS) → enc_output(ring)
 *               → ws_sender_task → WebSocket → 云端大模型
 *
 *   云端大模型 → WebSocket → on_ws_receive → dec_input(ring)
 *             → audio_decoder_task → dec_output(ring)
 *             → play_task → codec_dev(I2S) → 扬声器
 *
 * VAD 流程：
 *   AFE 检测到 VAD_SPEECH → on_vad_change 记录 speech_detected
 *   AFE 检测到 VAD_SILENCE（说话后）→ 启动 EOS 定时器（800ms）
 *   EOS 定时器触发 → 停止发送，等待云端回复
 */

#include "session.h"
#include "audio/audio_processor.h"
#include "protocol/websocket_client.h"
#include "wake_word/custom_wake_word.h"
#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

#define TAG "Session"

// ─── 可调参数 ─────────────────────────────────────────────────────────────────

#define DEFAULT_WS_URI "ws://192.168.1.100:8080/audio"
#define NVS_NAMESPACE_NET "net_config"
#define NVS_KEY_WS_URI "ws_uri"

// 会话整体超时（ms）：云端长时间无回复则关闭
#define SESSION_TIMEOUT_MS 10000

// 语音结束定时器（ms）：VAD 检测到静音后等待此时长再停发
// 与 AFE vad_min_noise_ms 保持一致，避免重复触发
#define EOS_SILENCE_MS 900

// WS 发送缓冲区（OPUS 帧一般 < 300 字节，512 足够）
#define OPUS_SEND_BUF 512

// ─── 模块私有状态 ─────────────────────────────────────────────────────────────

static volatile session_state_t s_state = SESSION_IDLE;
static audio_processor_t *s_processor = NULL;
static TimerHandle_t s_session_timer = NULL;    // 整体超时
static TimerHandle_t s_eos_timer = NULL;        // 语音结束
static volatile bool s_speech_detected = false; // VAD 检测到过语音
static char s_ws_uri[128] = DEFAULT_WS_URI;
static volatile TaskHandle_t s_sender_handle = NULL;

// ─── 前向声明 ─────────────────────────────────────────────────────────────────

static void session_close(void);

// ─── 超时回调 ─────────────────────────────────────────────────────────────────

static void on_session_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "会话整体超时，强制关闭");
    session_close();
}

// 语音结束计时结束：用户停止说话，停止向云端发送
static void on_eos_timeout(TimerHandle_t t)
{
    if (s_state != SESSION_LISTENING)
        return;

    ESP_LOGI(TAG, "检测到说话结束（VAD 静音），停止发送，等待云端回复");
    s_state = SESSION_PLAYING;

    // 注销增强 PCM 钩子：不再录音
    bsp_wake_word_set_enhanced_pcm_hook(NULL);

    // 重置整体超时定时器，给云端足够时间生成回复
    xTimerReset(s_session_timer, 0);
}

// ─── WebSocket 接收回调 ───────────────────────────────────────────────────────

static void on_ws_receive(const void *data, size_t len)
{
    if (s_state == SESSION_IDLE || s_processor == NULL)
        return;

    // 重置整体超时（收到数据说明云端还活着）
    xTimerReset(s_session_timer, 0);

    // 写入解码器输入缓冲
    audio_processor_write(s_processor, (void *)data, len);
}

// ─── VAD 状态变化回调（来自 AFE fetch 任务）─────────────────────────────────

static void on_vad_change(vad_state_t state)
{
    if (s_state != SESSION_LISTENING)
        return;

    if (state == VAD_SPEECH)
    {
        // 用户开始说话：记录标志，停止 EOS 定时器
        s_speech_detected = true;
        xTimerStop(s_eos_timer, 0);
    }
    else
    {
        // VAD_SILENCE：如果之前检测到过语音，启动 EOS 计时
        if (s_speech_detected)
            xTimerStart(s_eos_timer, 0);
    }
}

// ─── 增强 PCM 钩子（来自 AFE fetch 任务，降噪后的麦克风数据）────────────────

static void on_enhanced_pcm(const int16_t *data, size_t samples)
{
    if (s_state != SESSION_LISTENING || s_processor == NULL)
        return;
    audio_processor_write_pcm(s_processor, (void *)data, samples * sizeof(int16_t));
}

// ─── WS 发送任务 ─────────────────────────────────────────────────────────────

static void ws_sender_task(void *arg)
{
    uint8_t buf[OPUS_SEND_BUF];

    while (s_state == SESSION_LISTENING || s_state == SESSION_PLAYING)
    {
        // 带 100ms 超时，确保状态变化时能退出
        size_t len = audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);
        if (len > 0 && ws_client_is_connected())
            ws_client_send_binary(buf, len);
    }

    s_sender_handle = NULL;
    vTaskDelete(NULL);
}

// ─── NVS 加载 WS URI ─────────────────────────────────────────────────────────

static void load_ws_uri_from_nvs(char *uri, size_t max_len)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE_NET, NVS_READONLY, &h) == ESP_OK)
    {
        size_t sz = max_len;
        if (nvs_get_str(h, NVS_KEY_WS_URI, uri, &sz) == ESP_OK)
        {
            nvs_close(h);
            ESP_LOGI(TAG, "WS URI 从 NVS 加载: %s", uri);
            return;
        }
        nvs_close(h);
    }
    // 使用默认值（已在初始化时填入 s_ws_uri）
    ESP_LOGW(TAG, "NVS 未找到 ws_uri，使用默认: %s", uri);
}

// ─── 关闭会话 ─────────────────────────────────────────────────────────────────

static void session_close(void)
{
    if (s_state == SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "关闭会话");
    s_state = SESSION_IDLE;
    s_speech_detected = false;

    // 1. 注销 AFE 钩子（先关，防止 PCM 继续流入已停的 processor）
    bsp_wake_word_set_enhanced_pcm_hook(NULL);
    bsp_wake_word_set_vad_callback(NULL);

    // 2. 断开 WebSocket
    ws_client_stop();

    // 3. 停止并销毁音频处理器（ring buffer 被删，sender_task 读到 NULL 后退出）
    if (s_processor != NULL)
    {
        audio_processor_stop(s_processor);
        // 等待 sender_task 自然退出（最多 500ms）
        for (int i = 0; i < 5 && s_sender_handle != NULL; i++)
            vTaskDelay(pdMS_TO_TICKS(100));
        audio_processor_destroy(s_processor);
        s_processor = NULL;
    }

    // 4. 停止所有定时器
    xTimerStop(s_session_timer, 0);
    xTimerStop(s_eos_timer, 0);

    // 5. 恢复唤醒词监听（MultiNet 重新开始检测）
    bsp_wake_word_start();
    ESP_LOGI(TAG, "会话已关闭，重新监听唤醒词...");
}

// ─── 公开 API ─────────────────────────────────────────────────────────────────

void session_init(const char *ws_uri)
{
    // 优先使用传入的 URI
    if (ws_uri != NULL)
    {
        strncpy(s_ws_uri, ws_uri, sizeof(s_ws_uri) - 1);
        s_ws_uri[sizeof(s_ws_uri) - 1] = '\0';
    }
    else
    {
        // 从 NVS 加载，若无则保留 DEFAULT_WS_URI
        load_ws_uri_from_nvs(s_ws_uri, sizeof(s_ws_uri));
    }

    // 整体超时定时器（单次触发）
    s_session_timer = xTimerCreate("session_to",
                                   pdMS_TO_TICKS(SESSION_TIMEOUT_MS),
                                   pdFALSE, NULL, on_session_timeout);

    // EOS 定时器（单次触发，由 VAD 静音事件启动）
    s_eos_timer = xTimerCreate("eos_to",
                               pdMS_TO_TICKS(EOS_SILENCE_MS),
                               pdFALSE, NULL, on_eos_timeout);

    ESP_LOGI(TAG, "会话模块已初始化，WS URI: %s", s_ws_uri);
}

void session_on_wake_word(const char *display)
{
    if (s_state != SESSION_IDLE)
    {
        ESP_LOGW(TAG, "已有活跃会话，忽略本次唤醒");
        return;
    }

    ESP_LOGI(TAG, "会话开始 [%s]", display);
    s_state = SESSION_LISTENING;
    s_speech_detected = false;

    // 步骤 1：创建并启动音频处理器
    s_processor = audio_processor_create();
    if (s_processor == NULL)
    {
        ESP_LOGE(TAG, "audio_processor_create 失败");
        s_state = SESSION_IDLE;
        bsp_wake_word_start();
        return;
    }
    audio_processor_start(s_processor);

    // 步骤 2：连接 WebSocket
    esp_err_t err = ws_client_start(s_ws_uri, on_ws_receive);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "WebSocket 连接失败，关闭会话");
        audio_processor_stop(s_processor);
        audio_processor_destroy(s_processor);
        s_processor = NULL;
        s_state = SESSION_IDLE;
        bsp_wake_word_start();
        return;
    }

    // 步骤 3：注册 AFE 钩子
    // - 增强 PCM 钩子：将降噪后的麦克风音频送给 OPUS 编码器
    // - VAD 回调：检测用户说话结束，自动触发 EOS
    bsp_wake_word_set_enhanced_pcm_hook(on_enhanced_pcm);
    bsp_wake_word_set_vad_callback(on_vad_change);

    // 步骤 4：启动 WS 发送任务
    xTaskCreatePinnedToCoreWithCaps(
        ws_sender_task, "ws_sender",
        4096, NULL, 5,
        (TaskHandle_t *)&s_sender_handle,
        0, MALLOC_CAP_SPIRAM);

    // 步骤 5：启动整体超时定时器
    xTimerStart(s_session_timer, 0);

    ESP_LOGI(TAG, "会话就绪：AFE-PCM → OPUS → WS 管道已建立");
}

session_state_t session_get_state(void)
{
    return s_state;
}
