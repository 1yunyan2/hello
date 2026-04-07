/**
 * @file session.c
 * @brief 会话状态机 — WebSocket 预连接 + 多轮对话 + 语音打断
 *
 * 连接策略：
 *   WiFi 就绪后立即建立 WebSocket 连接（含 TLS 握手），
 *   唤醒词触发时直接发送 Hello，无需等待连接建立。
 *   会话结束后保持连接，下次唤醒零延迟。
 *
 * 完整数据流：
 *   麦克风(I2S) → audio_feed_task → PCM Hook → enc_input(ring)
 *               → audio_encoder_task (OPUS) → enc_output(ring)
 *               → ws_sender_task → WebSocket → 云端大模型
 *
 *   云端大模型 → WebSocket → on_ws_receive → dec_input(ring)
 *             → audio_decoder_task → dec_output(ring)
 *             → play_task → codec_dev(I2S) → 扬声器
 *
 * 多轮对话流程：
 *   唤醒词触发 → 发送 Hello → LISTENING
 *   TTS_START  → PLAYING（停止编码，唤醒词引擎监听打断）
 *   TTS_STOP   → LISTENING（恢复编码，继续对话）
 *   唤醒词(PLAYING中) → abort + LISTENING
 *   超时无活动 → 关闭会话（保持 WebSocket 连接）
 */

#include "protocol/websocket_client.h"
#include "session.h"
#include "audio/audio_processor.h"
#include "protocol/mqtt_protocol.h"
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
#include "protocol/auth.h"

#define TAG "Session"

#define DEFAULT_WS_URI "ws://192.168.1.100:8080/audio"
#define NVS_NAMESPACE_NET "net_config" // NVS 存储
#define SESSION_TIMEOUT_MS 60000       // 整体会话超时：60 秒
#define EOS_SILENCE_MS 800             // 说话结束静音检测：800ms
#define OPUS_SEND_BUF 512

// ─── 事件组位定义 ────────────────────────────────────────────────────────────
static EventGroupHandle_t s_session_eg = NULL;
#define SESSION_SERVER_READY_BIT BIT0 // Hello 握手完成，可以发送音频
#define SESSION_WS_CONNECTED_BIT BIT1 // WebSocket 底层已连接

// ─── 模块级状态变量 ──────────────────────────────────────────────────────────
static volatile session_state_t s_state = SESSION_IDLE;
static audio_processor_t *s_processor = NULL;

// 【关键】Protocol 是持久对象，在 session_init 中创建，整个生命周期不销毁
static protocol_t *s_protocol = NULL;

static TimerHandle_t s_session_timer = NULL;
static TimerHandle_t s_eos_timer = NULL;
static volatile bool s_speech_detected = false;
static char s_ws_uri[128] = DEFAULT_WS_URI;
static char s_ws_token[256] = {0};     // deviceToken（App 绑定时下发的长期凭证）
static char s_access_token[512] = {0}; // accessToken（通过 device-login 换取的短效令牌）
static volatile TaskHandle_t s_sender_handle = NULL;
static SemaphoreHandle_t s_wake_word_mutex = NULL;
static char s_current_wake_word[64] = {0};

static void session_close(void);
static void on_enhanced_pcm(const int16_t *data, size_t samples);

// ─── 定时器回调 ──────────────────────────────────────────────────────────────

static void on_session_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "会话超时（%d 秒无活动），关闭会话", SESSION_TIMEOUT_MS / 1000);
    session_close();
}

static void on_eos_timeout(TimerHandle_t t)
{
    if (s_state != SESSION_LISTENING)
        return;
    ESP_LOGI(TAG, "检测到说话结束（VAD 静音 %dms），通知服务器", EOS_SILENCE_MS);
    if (s_protocol && protocol_is_connected(s_protocol))
        protocol_send_stop_listening(s_protocol);
}

// ─── 麦克风 PCM 数据回调 ────────────────────────────────────────────────────

static void on_enhanced_pcm(const int16_t *data, size_t samples)
{
    if (s_processor == NULL)
        return;

    // PLAYING 状态不送编码器（防回声）
    // 唤醒词引擎由 audio_feed_task → custom_wake_word_feed 独立运行
    if (s_state != SESSION_LISTENING)
        return;

    // 帧能量 VAD 辅助
    int64_t energy = 0;
    for (size_t i = 0; i < samples; i++)
        energy += (int64_t)data[i] * data[i];
    energy /= (samples > 0 ? samples : 1);

    if (energy > 200000)
    {
        if (!s_speech_detected)
        {
            s_speech_detected = true;
            ESP_LOGD(TAG, "检测到语音活动 (energy=%lld)", energy);
        }
        xTimerStop(s_eos_timer, 0);
    }
    else if (s_speech_detected)
    {
        xTimerStart(s_eos_timer, 0);
    }

    audio_processor_write_pcm(s_processor, (void *)data, samples * sizeof(int16_t));
}

// ─── 核心：协议层事件分发器 ─────────────────────────────────────────────────

static void protocol_event_handler(void *handler_args, esp_event_base_t base,
                                   int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    // ── WebSocket 底层连接成功（启动时预连接，或断线重连后）──────────────
    case PROTOCOL_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket 已连接（预连接就绪）");
        xEventGroupSetBits(s_session_eg, SESSION_WS_CONNECTED_BIT);

        // 如果有会话正在等待连接（极少数情况：唤醒时恰好断线重连中）
        if (s_state == SESSION_LISTENING)
        {
            ESP_LOGI(TAG, "会话等待中，立即发送 Hello");
            protocol_send_hello(s_protocol);
        }
        break;

    // ── 收到服务器 Hello 响应 → 握手完成 ────────────────────────────────
    case PROTOCOL_EVENT_HELLO:
        if (s_state == SESSION_IDLE)
            break; // 无活跃会话，忽略
        ESP_LOGI(TAG, "收到服务器 Hello 响应，会话已建立");

        xSemaphoreTake(s_wake_word_mutex, portMAX_DELAY);
        protocol_send_wake_word(s_protocol, s_current_wake_word);
        xSemaphoreGive(s_wake_word_mutex);
        protocol_send_start_listening(s_protocol, PROTOCOL_LISTEN_TYPE_AUTO);

        // 唤醒发送任务，开始消费 enc_output 中积压的 OPUS 帧
        xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);
        xTimerReset(s_session_timer, 0);
        ESP_LOGI(TAG, "服务器就绪，开始推送 Opus 音频流");
        break;

    // ── TTS 开始 → PLAYING，重启唤醒词引擎支持打断 ──────────────────────
    case PROTOCOL_EVENT_TTS_START:
        ESP_LOGI(TAG, "🔊 服务器 TTS 开始播放");
        s_state = SESSION_PLAYING;
        s_speech_detected = false;
        xTimerStop(s_eos_timer, 0);
        xTimerReset(s_session_timer, 0);
        // 重启唤醒词引擎，TTS 期间可以检测打断唤醒词
        bsp_wake_word_start();
        break;

    case PROTOCOL_EVENT_TTS_SENTENCE_START:
        ESP_LOGI(TAG, "🎵 TTS: %s", (char *)event_data);
        break;

    // ── TTS 结束 → 恢复 LISTENING，继续多轮对话 ────────────────────────
    case PROTOCOL_EVENT_TTS_STOP:
        ESP_LOGI(TAG, "🔇 TTS 播放结束，恢复对话");
        s_state = SESSION_LISTENING;
        s_speech_detected = false;

        // 不 flush 编码器！用户可能在 TTS 尾声已开始说话
        if (s_protocol && protocol_is_connected(s_protocol))
            protocol_send_start_listening(s_protocol, PROTOCOL_LISTEN_TYPE_AUTO);

        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_STT:
        ESP_LOGI(TAG, "📝 语音转文字(STT): %s", (char *)event_data);
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_LLM:
        ESP_LOGI(TAG, "🤖 大模型状态: %s", (char *)event_data);
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_AUDIO:
        if (s_state != SESSION_IDLE && s_processor != NULL)
        {
            binary_data_t *bin = (binary_data_t *)event_data;
            audio_processor_write(s_processor, bin->ptr, bin->size);
            xTimerReset(s_session_timer, 0);
        }
        break;

    // ── WebSocket 断开 → 标记断开，esp_websocket_client 自动重连 ────────
    case PROTOCOL_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "❌ WebSocket 已断开连接");
        xEventGroupClearBits(s_session_eg, SESSION_WS_CONNECTED_BIT | SESSION_SERVER_READY_BIT);

        // 如果有活跃会话，给 15 秒等待重连
        if (s_state != SESSION_IDLE)
        {
            ESP_LOGW(TAG, "会话中断线，等待自动重连...");
            xTimerChangePeriod(s_session_timer, pdMS_TO_TICKS(15000), 0);
        }
        // IDLE 状态断线不需要处理，esp_websocket_client 的 reconnect_timeout_ms=5000 会自动重连
        break;

    default:
        break;
    }
}

// ─── WebSocket 发送任务 ─────────────────────────────────────────────────────

static void ws_sender_task(void *arg)
{
    uint8_t buf[OPUS_SEND_BUF];

    ESP_LOGI(TAG, "发送任务启动，等待服务器就绪...");
    xEventGroupWaitBits(s_session_eg, SESSION_SERVER_READY_BIT,
                        pdFALSE, pdFALSE, portMAX_DELAY);
    ESP_LOGI(TAG, "服务器已就绪，发送任务运行中");

    while (s_state == SESSION_LISTENING || s_state == SESSION_PLAYING)
    {
        // 断线期间暂停
        if (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
        {
            vTaskDelay(pdMS_TO_TICKS(100));
            continue;
        }

        size_t len = audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);
        if (len > 0 && s_protocol && protocol_is_connected(s_protocol))
        {
            binary_data_t bin = {.ptr = buf, .size = len};
            protocol_send_audio_data(s_protocol, &bin);
        }
    }

    s_sender_handle = NULL;
    vTaskDelete(NULL);
}

// ─── 关闭会话（保持 WebSocket 连接）────────────────────────────────────────

static void session_close(void)
{
    if (s_state == SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "关闭会话");
    s_state = SESSION_IDLE;
    s_speech_detected = false;

    // 释放发送任务的阻塞
    xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
    bsp_wake_word_set_enhanced_pcm_hook(NULL);

    xTimerStop(s_session_timer, 0);
    xTimerStop(s_eos_timer, 0);

    // 【关键】不销毁 Protocol！WebSocket 连接保持活跃，下次唤醒零延迟
    // s_protocol 是持久对象

    if (s_processor != NULL)
    {
        audio_processor_stop(s_processor);
        for (int i = 0; i < 5 && s_sender_handle != NULL; i++)
            vTaskDelay(pdMS_TO_TICKS(100));
        audio_processor_destroy(s_processor);
        s_processor = NULL;
    }

    bsp_wake_word_start();
    ESP_LOGI(TAG, "会话已关闭，WebSocket 保持连接，重新监听唤醒词...");
}

// ─── 公开 API：初始化 + 预连接 ──────────────────────────────────────────────

void session_init(const char *ws_uri)
{
    s_session_eg = xEventGroupCreate();
    s_wake_word_mutex = xSemaphoreCreateMutex();

    // 从 NVS 读取配置
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE_NET, NVS_READONLY, &h) == ESP_OK)
    {
        if (ws_uri == NULL)
        {
            size_t sz = sizeof(s_ws_uri);
            nvs_get_str(h, "ws_uri", s_ws_uri, &sz);
        }
        size_t token_sz = sizeof(s_ws_token);
        nvs_get_str(h, "device_token", s_ws_token, &token_sz);
        nvs_close(h);
    }
    if (ws_uri != NULL)
        strncpy(s_ws_uri, ws_uri, sizeof(s_ws_uri) - 1);

    // ── 步骤 2/3：用 deviceToken 调用 /api/auth/device-login 换取 accessToken ──
    const char *ws_bearer_token = "";
    if (strlen(s_ws_token) > 0)
    {
        ESP_LOGI(TAG, "检测到 deviceToken，正在换取 accessToken...");
        auth_t *auth = auth_create();
        auth_perform(auth, s_ws_token); // 调用 /api/auth/device-login

        if (auth->access_token != NULL)
        {
            strncpy(s_access_token, auth->access_token, sizeof(s_access_token) - 1);
            ws_bearer_token = s_access_token;
            ESP_LOGI(TAG, "✅ accessToken 获取成功");
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ accessToken 获取失败，尝试用 NVS 缓存的旧 token 连接");
            // 尝试从 NVS 读取上次缓存的 accessToken（auth_perform 成功时会存入）
            nvs_handle_t nh;
            if (nvs_open(NVS_NAMESPACE_NET, NVS_READONLY, &nh) == ESP_OK)
            {
                size_t at_sz = sizeof(s_access_token);
                if (nvs_get_str(nh, "access_token", s_access_token, &at_sz) == ESP_OK)
                    ws_bearer_token = s_access_token;
                nvs_close(nh);
            }
        }
        auth_destroy(auth);
    }
    else
    {
        ESP_LOGW(TAG, "⚠️ 无 deviceToken，WebSocket 将无认证连接");
    }

    // 创建定时器
    s_session_timer = xTimerCreate("session_to",
                                   pdMS_TO_TICKS(SESSION_TIMEOUT_MS),
                                   pdFALSE, NULL, on_session_timeout);
    s_eos_timer = xTimerCreate("eos_to",
                               pdMS_TO_TICKS(EOS_SILENCE_MS),
                               pdFALSE, NULL, on_eos_timeout);

    // 【关键】步骤 1：用 accessToken 预创建并连接 WebSocket
    s_protocol = protocol_create(s_ws_uri, ws_bearer_token);
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);

    ESP_LOGI(TAG, "会话模块已初始化，WebSocket 预连接中... URI: %s", s_ws_uri);
    if (strlen(s_access_token) > 0)
        ESP_LOGI(TAG, "已使用 accessToken 认证连接");
}

// ─── 公开 API：唤醒词触发 ──────────────────────────────────────────────────

void session_on_wake_word(const char *display)
{
    // ── 场景 1：PLAYING 中打断 ──────────────────────────────────────────
    if (s_state == SESSION_PLAYING)
    {
        ESP_LOGW(TAG, "⚠️ 唤醒词打断 TTS: [%s]", display);
        if (s_protocol && protocol_is_connected(s_protocol))
        {
            protocol_send_abort_speaking(s_protocol);
            protocol_send_wake_word(s_protocol, display);
            protocol_send_start_listening(s_protocol, PROTOCOL_LISTEN_TYPE_AUTO);
        }
        // 清空解码器缓冲区（停止 TTS 播放）
        audio_processor_flush_output(s_processor);
        s_state = SESSION_LISTENING;
        s_speech_detected = false;
        xTimerReset(s_session_timer, 0);
        return;
    }

    // ── 场景 2：正常唤醒 ────────────────────────────────────────────────
    if (s_state != SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "会话开始 [%s]", display);
    s_state = SESSION_LISTENING;
    s_speech_detected = false;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);

    // 暂存唤醒词
    xSemaphoreTake(s_wake_word_mutex, portMAX_DELAY);
    strncpy(s_current_wake_word, display, sizeof(s_current_wake_word) - 1);
    s_current_wake_word[sizeof(s_current_wake_word) - 1] = '\0';
    xSemaphoreGive(s_wake_word_mutex);

    // 创建音频管道
    s_processor = audio_processor_create();
    if (!s_processor)
        goto error;
    audio_processor_start(s_processor);

    // 立即开启 PCM Hook，音频积压在编码器缓冲区
    bsp_wake_word_set_enhanced_pcm_hook(on_enhanced_pcm);

    // 创建发送任务
    xTaskCreatePinnedToCoreWithCaps(ws_sender_task, "ws_sender",
                                    4096, NULL, 5,
                                    (TaskHandle_t *)&s_sender_handle,
                                    0, MALLOC_CAP_SPIRAM);

    // 【关键】WebSocket 已经预连接好，直接发 Hello，无需等待 TLS
    if (xEventGroupGetBits(s_session_eg) & SESSION_WS_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WebSocket 已就绪，立即发送 Hello");
        protocol_send_hello(s_protocol);
    }
    else
    {
        // 极少数情况：WebSocket 恰好断开，等待自动重连后在 CONNECTED 事件中发送 Hello
        ESP_LOGW(TAG, "WebSocket 未就绪，等待重连后自动发送 Hello...");
    }

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
