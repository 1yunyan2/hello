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
#define NVS_NAMESPACE_NET "net_config"     // NVS 存储
#define SESSION_TIMEOUT_MS 60000           // 整体会话超时：60 秒
#define EOS_SILENCE_MS 800                 // 说话结束静音检测：800ms
#define TOKEN_REFRESH_MS (110 * 60 * 1000) // Token 主动刷新：110 分钟（过期时间 2h，提前 10 分钟）
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
static TimerHandle_t s_token_refresh_timer = NULL; // accessToken 主动刷新定时器（2h 过期，提前 10min 刷新）
static volatile bool s_speech_detected = false;
static char s_ws_uri[128] = DEFAULT_WS_URI;
static char s_ws_token[256] = {0};     // deviceToken（App 绑定时下发的长期凭证）
static char s_access_token[512] = {0}; // accessToken（通过 device-login 换取的短效令牌）
static volatile TaskHandle_t s_sender_handle = NULL;
static volatile TaskHandle_t s_reconnect_handle = NULL; // 重连任务句柄，防止重复创建
static int s_reconnect_attempts = 0;                    // 连续重连次数，用于指数退避
#define RECONNECT_MAX_ATTEMPTS 5                        // 最大重连次数，超过后停止重连
#define RECONNECT_BASE_DELAY_MS 5000                    // 基础退避延迟 5 秒
static SemaphoreHandle_t s_wake_word_mutex = NULL;
static char s_current_wake_word[64] = {0};

static void session_close(void);
static void on_enhanced_pcm(const int16_t *data, size_t samples);
static void session_reconnect_task(void *arg);

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

/**
 * @brief Token 主动刷新定时器回调
 * accessToken 有效期 2 小时，提前 10 分钟触发刷新。
 * 只在 IDLE 状态执行（会话中不打断，等会话结束后自然触发下一次）。
 */
static void on_token_refresh_timeout(TimerHandle_t t)
{
    if (s_state != SESSION_IDLE)
    {
        // 会话进行中，延迟 1 分钟后重试
        ESP_LOGI(TAG, "会话中，Token 刷新延迟 1 分钟");
        xTimerChangePeriod(s_token_refresh_timer, pdMS_TO_TICKS(60000), 0);
        return;
    }

    if (strlen(s_ws_token) == 0)
        return; // 无 deviceToken，无需刷新

    ESP_LOGI(TAG, "Token 即将过期，主动刷新...");
    if (s_reconnect_handle == NULL)
    {
        xTaskCreatePinnedToCoreWithCaps(session_reconnect_task, "ws_reconn",
                                        6144, (void *)(intptr_t)0, 3,
                                        (TaskHandle_t *)&s_reconnect_handle,
                                        1, MALLOC_CAP_SPIRAM);
    }

    // 重置定时器为标准周期（如果上面是延迟重试进来的）
    xTimerChangePeriod(s_token_refresh_timer, pdMS_TO_TICKS(TOKEN_REFRESH_MS), 0);
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
        s_reconnect_attempts = 0; // 连接成功，重置退避计数
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

    // ── WebSocket 断开 → 退避重连 ─────────────────────────────────────────
    case PROTOCOL_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "❌ WebSocket 已断开连接");
        xEventGroupClearBits(s_session_eg, SESSION_WS_CONNECTED_BIT | SESSION_SERVER_READY_BIT);

        if (s_state != SESSION_IDLE)
        {
            ESP_LOGW(TAG, "会话中断线，等待重连...");
            xTimerChangePeriod(s_session_timer, pdMS_TO_TICKS(15000), 0);
        }

        // 指数退避重连：第 1 次 5 秒，第 2 次 10 秒，第 3 次 20 秒...
        // 超过最大次数后停止，防止 429 限流导致无限重连风暴
        if (s_reconnect_handle != NULL)
        {
            ESP_LOGW(TAG, "重连任务已在运行中，跳过");
        }
        else if (s_reconnect_attempts >= RECONNECT_MAX_ATTEMPTS)
        {
            ESP_LOGE(TAG, "已连续重连 %d 次均失败，停止重连。等待下次唤醒或 Token 刷新时重试",
                     s_reconnect_attempts);
        }
        else
        {
            s_reconnect_attempts++;
            int delay_ms = RECONNECT_BASE_DELAY_MS * (1 << (s_reconnect_attempts - 1)); // 指数退避
            if (delay_ms > 60000)
                delay_ms = 60000; // 上限 60 秒
            ESP_LOGW(TAG, "第 %d 次重连，%d 秒后执行...", s_reconnect_attempts, delay_ms / 1000);
            // 延迟在重连任务内部执行，避免阻塞事件回调
            xTaskCreatePinnedToCoreWithCaps(session_reconnect_task, "ws_reconn",
                                            6144, (void *)(intptr_t)delay_ms, 3,
                                            (TaskHandle_t *)&s_reconnect_handle,
                                            1, MALLOC_CAP_SPIRAM);
        }
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

    // 【关键修复】等待期间持续排空编码器输出缓冲区，防止 enc_output 满溢
    // 之前用 portMAX_DELAY 死等，导致编码器输出无人消费 → 缓冲区满 → 疯狂丢帧报警
    while (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
    {
        if (s_state != SESSION_LISTENING && s_state != SESSION_PLAYING)
            goto exit; // 会话已关闭，直接退出
        // 读出并丢弃：服务器尚未就绪，这些帧无法发送，但必须消费以保持管道畅通
        audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);
    }

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

exit:
    s_sender_handle = NULL;
    vTaskDelete(NULL);
}

// ─── Token 刷新 + WebSocket 重连任务 ────────────────────────────────────────
// 断线时在独立任务中执行：重新认证 → 销毁旧连接 → 创建新连接
// 不能在 WebSocket 回调中直接 destroy（会死锁），必须在独立任务中执行

static void session_reconnect_task(void *arg)
{
    int delay_ms = (int)(intptr_t)arg;

    // 指数退避延迟：在重连前等待，避免 429 限流风暴
    if (delay_ms > 0)
    {
        ESP_LOGI(TAG, "重连退避等待 %d 秒...", delay_ms / 1000);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    ESP_LOGI(TAG, "开始 Token 刷新 + 重连流程...");

    // 第一步：用 deviceToken 重新换取 accessToken
    const char *new_token = "";
    if (strlen(s_ws_token) > 0)
    {
        auth_t *auth = auth_create();
        auth_perform(auth, s_ws_token);

        if (auth->access_token != NULL)
        {
            strncpy(s_access_token, auth->access_token, sizeof(s_access_token) - 1);
            new_token = s_access_token;
            ESP_LOGI(TAG, "✅ Token 刷新成功");
        }
        else
        {
            ESP_LOGW(TAG, "⚠️ Token 刷新失败，使用 NVS 缓存的旧 token");
            nvs_handle_t nh;
            if (nvs_open(NVS_NAMESPACE_NET, NVS_READONLY, &nh) == ESP_OK)
            {
                size_t at_sz = sizeof(s_access_token);
                if (nvs_get_str(nh, "access_token", s_access_token, &at_sz) == ESP_OK)
                    new_token = s_access_token;
                nvs_close(nh);
            }
        }
        auth_destroy(auth);
    }

    // 第二步：销毁旧 WebSocket 连接，用新 token 重建
    if (s_protocol != NULL)
    {
        protocol_disconnect(s_protocol);
        protocol_destroy(s_protocol);
        s_protocol = NULL;
    }

    s_protocol = protocol_create(s_ws_uri, new_token);
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);

    ESP_LOGI(TAG, "重连完成，WebSocket 正在建立连接... URI: %s", s_ws_uri);

    s_reconnect_handle = NULL;
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

    // Token 主动刷新定时器：2 小时过期，提前 10 分钟（110 分钟）自动刷新
    // pdTRUE = 自动重载，周期性执行
    s_token_refresh_timer = xTimerCreate("token_ref",
                                         pdMS_TO_TICKS(TOKEN_REFRESH_MS),
                                         pdTRUE, NULL, on_token_refresh_timeout);
    if (strlen(s_ws_token) > 0)
        xTimerStart(s_token_refresh_timer, 0);

    // 【关键】步骤 1：动态拼接 ?token=<accessToken> 到 URL 后面
    char full_ws_uri[1024] = {0}; // 必须要足够大，因为 accessToken 很长

    if (strlen(ws_bearer_token) > 0)
    {
        // 如果有 Token，按后端的格式拼接到网址末尾
        snprintf(full_ws_uri, sizeof(full_ws_uri), "%s?token=%s", s_ws_uri, ws_bearer_token);
    }
    else
    {
        // 如果没有，就用原网址
        strncpy(full_ws_uri, s_ws_uri, sizeof(full_ws_uri) - 1);
    }

    ESP_LOGW(TAG, "最终请求的完整 WebSocket URI: %s", full_ws_uri);

    // 【关键】步骤 1：用 accessToken 预创建并连接 WebSocket
    // 使用拼接好的完整 URL 去建立连接
    s_protocol = protocol_create(full_ws_uri, ws_bearer_token);
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);

    ESP_LOGI(TAG, "会话模块已初始化，WebSocket 预连接中... URI: %s", s_ws_uri);
    if (strlen(s_access_token) > 0)
        ESP_LOGI(TAG, "已使用 accessToken 认证连接（%d 分钟后自动刷新）", TOKEN_REFRESH_MS / 60000);
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
