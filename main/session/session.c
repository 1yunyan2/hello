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

#include "protocol/websocket_client.h"
#include "session.h"
#include "audio/audio_processor.h"
#include "protocol/protocol.h" // 🌟 引入全新的协议层
#include "wake_word/custom_wake_word.h"
#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "nvs.h"
#include <string.h>

#define TAG "Session"

#define DEFAULT_WS_URI "ws://192.168.1.100:8080/audio"
#define NVS_NAMESPACE_NET "net_config"
#define SESSION_TIMEOUT_MS 10000
#define EOS_SILENCE_MS 900
#define OPUS_SEND_BUF 512

// 事件组：用于同步 WS 握手流程
static EventGroupHandle_t s_session_eg = NULL;
#define SESSION_SERVER_READY_BIT BIT0

static volatile session_state_t s_state = SESSION_IDLE;
static audio_processor_t *s_processor = NULL;
static protocol_t *s_protocol = NULL; // 🌟 协议对象实例

static TimerHandle_t s_session_timer = NULL;
static TimerHandle_t s_eos_timer = NULL;
static volatile bool s_speech_detected = false;
static char s_ws_uri[128] = DEFAULT_WS_URI;
static char s_ws_token[256] = {0}; // 保存鉴权 Token
static volatile TaskHandle_t s_sender_handle = NULL;
static SemaphoreHandle_t s_wake_word_mutex = NULL;   // 保护 s_current_wake_word 的跨任务并发访问
static char s_current_wake_word[64] = {0};            // 保存当前唤醒词，等待握手后发送

static void session_close(void);

static void on_session_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "会话整体超时，强制关闭");
    session_close();
}

static void on_eos_timeout(TimerHandle_t t)
{
    if (s_state != SESSION_LISTENING)
        return;
    ESP_LOGI(TAG, "检测到说话结束（VAD 静音），停止发送");
    s_state = SESSION_PLAYING;
    bsp_wake_word_set_enhanced_pcm_hook(NULL);
    xTimerReset(s_session_timer, 0);

    // 🌟 说话结束，告诉服务器停止监听，大模型可以开始生成了
    if (s_protocol && protocol_is_connected(s_protocol))
    {
        protocol_send_stop_listening(s_protocol);
    }
}

static void on_vad_change(vad_state_t state)
{
    if (s_state != SESSION_LISTENING)
        return;
    if (state == VAD_SPEECH)
    {
        s_speech_detected = true;
        xTimerStop(s_eos_timer, 0);
    }
    else
    {
        if (s_speech_detected)
            xTimerStart(s_eos_timer, 0);
    }
}

static void on_enhanced_pcm(const int16_t *data, size_t samples)
{
    if (s_state != SESSION_LISTENING || s_processor == NULL)
        return;
    audio_processor_write_pcm(s_processor, (void *)data, samples * sizeof(int16_t));
}

// 🌟 核心：协议层事件分发器
static void protocol_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    case PROTOCOL_EVENT_CONNECTED:
        ESP_LOGI(TAG, "👉 WebSocket 已连接，发送 Hello 握手");
        protocol_send_hello(s_protocol);
        break;

    case PROTOCOL_EVENT_HELLO:
        ESP_LOGI(TAG, "收到服务器 Hello 响应，请求开启自动监听");
        // 服务器就绪并分配了 session_id 后，再发送唤醒词和启动监听指令
        xSemaphoreTake(s_wake_word_mutex, portMAX_DELAY);
        protocol_send_wake_word(s_protocol, s_current_wake_word);
        xSemaphoreGive(s_wake_word_mutex);
        protocol_send_start_listening(s_protocol, PROTOCOL_LISTEN_TYPE_AUTO);
        xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT); // 唤醒发包任务
        break;

    case PROTOCOL_EVENT_STT:
        ESP_LOGI(TAG, "📝 语音转文字(STT): %s", (char *)event_data);
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_LLM:
        ESP_LOGI(TAG, "🤖 大模型状态: %s", (char *)event_data);
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_TTS_START:
        ESP_LOGI(TAG, "🔊 服务器 TTS 开始播放");
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_TTS_SENTENCE_START:
        ESP_LOGI(TAG, "🎵 TTS 正在朗读: %s", (char *)event_data);
        break;

    case PROTOCOL_EVENT_AUDIO:
        // 收到云端下发的二进制音频，送入解码器
        if (s_state != SESSION_IDLE && s_processor != NULL)
        {
            binary_data_t *bin = (binary_data_t *)event_data;
            audio_processor_write(s_processor, bin->ptr, bin->size);
            xTimerReset(s_session_timer, 0);
        }
        break;

    case PROTOCOL_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "❌ WebSocket 已断开连接");
        break;

    default:
        break;
    }
}

static void ws_sender_task(void *arg)
{
    uint8_t buf[OPUS_SEND_BUF];

    // 🌟 必须等待服务器发送了 Hello 并完成握手，才开始发送麦克风数据
    ESP_LOGI(TAG, "发送任务启动，等待服务器就绪...");
    xEventGroupWaitBits(s_session_eg, SESSION_SERVER_READY_BIT, pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "服务器已就绪，开始推送 Opus 音频流");

    while (s_state == SESSION_LISTENING || s_state == SESSION_PLAYING)
    {
        size_t len = audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);
        if (len > 0 && s_protocol && protocol_is_connected(s_protocol))
        {
            binary_data_t bin = {.ptr = buf, .size = len};
            protocol_send_audio_data(s_protocol, &bin); // 🌟 使用新接口
        }
    }
    s_sender_handle = NULL;
    vTaskDelete(NULL);
}

static void session_close(void)
{
    if (s_state == SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "关闭会话");
    s_state = SESSION_IDLE;
    s_speech_detected = false;

    // 🌟 释放可能因为等待握手而被永远阻塞的 ws_sender_task，防止僵尸任务和内存越界
    xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);

    bsp_wake_word_set_enhanced_pcm_hook(NULL);
    bsp_wake_word_set_vad_callback(NULL);

    // 🌟 销毁 Protocol 实例
    if (s_protocol)
    {
        protocol_disconnect(s_protocol);
        protocol_destroy(s_protocol);
        s_protocol = NULL;
    }

    if (s_processor != NULL)
    {
        audio_processor_stop(s_processor);
        for (int i = 0; i < 5 && s_sender_handle != NULL; i++)
            vTaskDelay(pdMS_TO_TICKS(100));
        audio_processor_destroy(s_processor);
        s_processor = NULL;
    }

    xTimerStop(s_session_timer, 0);
    xTimerStop(s_eos_timer, 0);
    bsp_wake_word_start();
    ESP_LOGI(TAG, "会话已关闭，重新监听唤醒词...");
}

void session_init(const char *ws_uri)
{
    s_session_eg = xEventGroupCreate();
    s_wake_word_mutex = xSemaphoreCreateMutex();

    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE_NET, NVS_READONLY, &h) == ESP_OK)
    {
        if (ws_uri == NULL)
        {
            size_t sz = sizeof(s_ws_uri);
            nvs_get_str(h, "ws_uri", s_ws_uri, &sz);
        }
        // 🌟 读取前端 App 传过来的鉴权 Token
        size_t token_sz = sizeof(s_ws_token);
        nvs_get_str(h, "ws_token", s_ws_token, &token_sz);
        nvs_close(h);
    }
    if (ws_uri != NULL)
        strncpy(s_ws_uri, ws_uri, sizeof(s_ws_uri) - 1);

    s_session_timer = xTimerCreate("session_to", pdMS_TO_TICKS(SESSION_TIMEOUT_MS), pdFALSE, NULL, on_session_timeout);
    s_eos_timer = xTimerCreate("eos_to", pdMS_TO_TICKS(EOS_SILENCE_MS), pdFALSE, NULL, on_eos_timeout);

    ESP_LOGI(TAG, "会话模块已初始化，WS URI: %s", s_ws_uri);
    if (strlen(s_ws_token) > 0)
        ESP_LOGI(TAG, "已加载 Token: ***");
}

void session_on_wake_word(const char *display)
{
    if (s_state != SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "会话开始 [%s]", display);
    s_state = SESSION_LISTENING;
    s_speech_detected = false;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT); // 重置同步位

    s_processor = audio_processor_create();
    if (!s_processor)
        goto error;
    audio_processor_start(s_processor);

    // 🌟 创建并启动新的 Protocol 客户端（带鉴权）
    s_protocol = protocol_create(s_ws_uri, s_ws_token);
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);

    // 暂存唤醒词（加锁保护，ws_sender_task 可能同时读取）
    xSemaphoreTake(s_wake_word_mutex, portMAX_DELAY);
    strncpy(s_current_wake_word, display, sizeof(s_current_wake_word) - 1);
    s_current_wake_word[sizeof(s_current_wake_word) - 1] = '\0';
    xSemaphoreGive(s_wake_word_mutex);

    bsp_wake_word_set_enhanced_pcm_hook(on_enhanced_pcm);
    bsp_wake_word_set_vad_callback(on_vad_change);

    xTaskCreatePinnedToCoreWithCaps(ws_sender_task, "ws_sender", 4096, NULL, 5, (TaskHandle_t *)&s_sender_handle, 0, MALLOC_CAP_SPIRAM);
    xTimerStart(s_session_timer, 0);
    return;

error:
    ESP_LOGE(TAG, "会话启动失败");
    s_state = SESSION_IDLE;
    bsp_wake_word_start();
}

session_state_t session_get_state(void)
{
    return s_state;
}