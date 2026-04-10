/**
 * @file session.c
 * @brief 会话状态机 — WebSocket 预连接 + 多轮对话 + 语音打断
 *
 * 连接策略：
 * WiFi 就绪后立即建立 WebSocket 连接（含 TLS 握手），
 * 唤醒词触发时直接发送 Hello，无需等待连接建立。
 * 会话结束后保持连接，下次唤醒零延迟。
 *
 * 完整数据流：
 * 麦克风(I2S) → audio_feed_task → PCM Hook → enc_input(ring)
 * → audio_encoder_task (OPUS) → enc_output(ring)
 * → ws_sender_task → WebSocket → 云端大模型
 *
 * 云端大模型 → WebSocket → on_ws_receive → dec_input(ring)
 * → audio_decoder_task → dec_output(ring)
 * → play_task → codec_dev(I2S) → 扬声器
 *
 * 多轮对话流程：
 * 唤醒词触发 → 发送 Hello → LISTENING
 * TTS_START  → PLAYING（停止编码，唤醒词引擎监听打断）
 * TTS_STOP   → LISTENING（恢复编码，继续对话）
 * 唤醒词(PLAYING中) → abort + LISTENING
 * 超时无活动 → 关闭会话（保持 WebSocket 连接）
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
#include "freertos/queue.h" // 【新增】引入队列支持
#include "esp_log.h"
#include "nvs.h"
#include <string.h>
#include "protocol/auth.h"
#include "wake_word/custom_wake_word.h"
#include "object.h"
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

// ─── 【新增】事件队列定义（用于将网络操作从定时器中剥离） ──────────────────────
typedef enum
{
    SESSION_EVT_VAD_STOP,
    SESSION_EVT_CLOSE, // 异步关闭信号
    // 未来可扩展其他耗时事件...
} session_evt_t;

static QueueHandle_t s_session_evt_queue = NULL;
static TaskHandle_t s_session_evt_task = NULL;

// ─── 模块级状态变量 ──────────────────────────────────────────────────────────
static volatile session_state_t s_state = SESSION_IDLE;
static audio_processor_t *s_processor = NULL;

// 【关键】Protocol 是持久对象，在 session_init 中创建，整个生命周期不销毁
static protocol_t *s_protocol = NULL;

static TimerHandle_t s_session_timer = NULL;
static TimerHandle_t s_eos_timer = NULL;
static TimerHandle_t s_token_refresh_timer = NULL; // accessToken 主动刷新定时器（2h 过期，提前 10min 刷新）
static volatile bool s_speech_detected = false;
static volatile bool s_stop_sent = false; // stop 只发一次
static bool s_is_continuous_turn = false; // 连续会话
static TickType_t s_vad_ready_tick = 0;   // 服务器就绪时刻（VAD 延迟启动基准）
#define VAD_GRACE_MS 500                  // 唤醒词尾音消退期：500ms
static char s_ws_uri[128] = DEFAULT_WS_URI;
static char s_ws_token[256] = {0};     // deviceToken（App 绑定时下发的长期凭证）
static char s_access_token[512] = {0}; // accessToken（通过 device-login 换取的短效令牌）
static volatile TaskHandle_t s_sender_handle = NULL;
static volatile TaskHandle_t s_reconnect_handle = NULL; // 重连任务句柄，防止重复创建
static int s_reconnect_attempts = 0;                    // 连续重连次数，用于指数退避
#define RECONNECT_MAX_ATTEMPTS 5                        // 最大重连次数，超过后停止重连
#define RECONNECT_BASE_DELAY_MS 5000                    // 基础退避延迟 5 秒
static SemaphoreHandle_t s_wake_word_mutex = NULL;      // 唤醒词锁，防止多线程操作
static char s_current_wake_word[64] = {0};

static void session_close(void);
static void on_enhanced_pcm(const int16_t *data, size_t samples);
static void session_reconnect_task(void *arg);

static void async_session_close_task(void *arg)
{
    session_close();
    vTaskDelete(NULL);
}

// ─── 【新增】会话专职网络事件任务 ─────────────────────────────────────────────
static void session_event_task(void *arg)
{
    session_evt_t evt;
    ESP_LOGI(TAG, "Session Event Task 启动，专属栈空间护航，等待队列消息...");

    while (1)
    {
        // 阻塞等待队列消息，不消耗 CPU
        if (xQueueReceive(s_session_evt_queue, &evt, portMAX_DELAY) == pdTRUE)
        {
            switch (evt)
            {
            case SESSION_EVT_VAD_STOP:
                ESP_LOGI(TAG, "检测到说话结束（VAD 静音 %dms），通知服务器", EOS_SILENCE_MS);
                if (s_protocol && protocol_is_connected(s_protocol))
                {
                    // 在这里执行耗时的 WebSocket 发送操作，绝对不会导致定时器栈溢出！
                    protocol_send_stop_listening(s_protocol);
                }
                break;
            case SESSION_EVT_CLOSE: // 【在这里安全地执行关闭】
                ESP_LOGI(TAG, "接收到异步关闭信号，安全关闭会话...");
                session_close();
                break;
            default:
                break;
            }
        }
    }
}

// ─── 定时器回调 ──────────────────────────────────────────────────────────────

static void on_session_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "会话超时（%d 秒无活动），关闭会话", SESSION_TIMEOUT_MS / 1000);
    session_close();
}

static void on_eos_timeout(TimerHandle_t t)
{
    if (s_state != SESSION_LISTENING || s_stop_sent)
        return;

    s_stop_sent = true; // 防止重复发送

    // 【修改点】不再直接发网络请求，而是飞速将信号推入队列，0 阻塞立即返回
    if (s_session_evt_queue != NULL)
    {
        session_evt_t evt = SESSION_EVT_VAD_STOP;
        xQueueSend(s_session_evt_queue, &evt, 0);
    }
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

    // ── VAD 检测（仅在服务器就绪 + 消退期后启用）──────────────────
    // 条件：服务器已就绪、消退期已过、stop 尚未发送
    if (s_vad_ready_tick != 0 && !s_stop_sent &&
        (xTaskGetTickCount() - s_vad_ready_tick) >= pdMS_TO_TICKS(VAD_GRACE_MS))
    {
        vad_state_t vad = bsp_wake_word_get_vad_state();

        if (vad == VAD_SPEECH)
        {
            if (!s_speech_detected)
            {
                s_speech_detected = true;
                ESP_LOGI(TAG, "🎤 AFE VAD 检测到语音活动");
                if (s_is_continuous_turn)
                {
                    if (s_protocol && protocol_is_connected(s_protocol))
                    {
                        protocol_send_hello(s_protocol);
                        ESP_LOGI(TAG, "连续对话：已发送 Start 握手");
                    }
                    s_is_continuous_turn = false; // 发完就重置，防止同一句话里重复发送
                }
            }
            xTimerStop(s_eos_timer, 0);
        }
        else if (s_speech_detected)
        {
            if (xTimerIsTimerActive(s_eos_timer) == pdFALSE)
                xTimerStart(s_eos_timer, 0);
        }
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
        ESP_LOGI(TAG, "收到服务器 started 响应，会话已建立");

        // 重置 VAD 状态，忽略唤醒词尾音；500ms 消退期后才开始检测
        s_speech_detected = false;
        s_stop_sent = false;
        s_vad_ready_tick = xTaskGetTickCount();

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
        s_stop_sent = false;
        s_is_continuous_turn = true;

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
            xTaskCreatePinnedToCore(session_reconnect_task, "ws_reconn",
                                    6144, (void *)(intptr_t)delay_ms, 3,
                                    (TaskHandle_t *)&s_reconnect_handle,
                                    1);
        }
        break;

    // ── 收到错误 → 不退出，直接重新进入录音状态 ────────────────────────────────
    case PROTOCOL_EVENT_ERROR:
        ESP_LOGE(TAG, "❌ 收到服务器报错: %s", event_data ? (char *)event_data : "未知");

        // if (s_state == SESSION_LISTENING || s_state == SESSION_PLAYING)
        // {
        //     ESP_LOGI(TAG, "刷新状态，重新开始录音...");
        //     // 重置所有语音检测标志
        //     s_speech_detected = false;
        //     s_stop_sent = false;
        //     s_vad_ready_tick = xTaskGetTickCount();
        //     xTimerReset(s_session_timer, 0); // 重置 60 秒超时定时器

        //     xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
        //     // 告诉服务器：我们重新开始新一轮的对话
        //     s_is_continuous_turn = true;

        //     s_state = SESSION_LISTENING;
        // }
        // break;

        // 【修改：报错后不再重连，直接关闭会话】
        // session_close();
        if (s_session_evt_queue != NULL)
        {
            session_evt_t evt = SESSION_EVT_CLOSE;
            xQueueSend(s_session_evt_queue, &evt, 0);
        }
        // 显式重置标志位，确保不会因为环境音自动触发
        s_is_continuous_turn = false;
        break;
    // ── 收到完成信号 → 如果没有TTS播放，则直接开始下一轮听 ────────────────────
    case PROTOCOL_EVENT_COMPLETE:
        // ESP_LOGI(TAG, "✅ 收到服务器 Complete 消息，交互回合结束");

        // // 如果目前状态还是 LISTENING（说明服务器只发了文本，没有下发TTS语音）
        // // 我们就不关闭会话，而是直接刷新状态，进入连续对话
        // if (s_state == SESSION_LISTENING)
        // {
        //     ESP_LOGI(TAG, "未收到语音回复，保持唤醒，进入连续对话...");
        //     s_speech_detected = false;
        //     s_stop_sent = false;
        //     s_vad_ready_tick = xTaskGetTickCount();
        //     xTimerReset(s_session_timer, 0);

        //     xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
        //     s_is_continuous_turn = true;
        // }
        // break;
        // 如果你希望只有在正常说完话后才进入连续对话，
        // 则在这里保留 s_is_continuous_turn = true;
        // 如果想彻底手动，也直接 session_close();
        ESP_LOGI(TAG, "✅ 会话完成");
        session_close();
        s_is_continuous_turn = false;
        break;
    default:
        break;
    }
}

// ─── WebSocket 发送任务 ─────────────────────────────────────────────────────

// ─── WebSocket 发送任务 ─────────────────────────────────────────────────────

static void ws_sender_task(void *arg)
{
    uint8_t buf[OPUS_SEND_BUF];

    ESP_LOGI(TAG, "发送任务启动，等待服务器就绪...");

    // 1. 开局等待阶段：在第一次收到 started 之前，只读不发，防止管道堵塞
    while (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
    {
        if (s_state != SESSION_LISTENING && s_state != SESSION_PLAYING)
            goto exit; // 会话已关闭，直接退出
        // 读出并丢弃：服务器尚未就绪，这些帧无法发送，但必须消费以保持管道畅通
        audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);
    }

    ESP_LOGI(TAG, "服务器已就绪，发送任务运行中");

    int sent_frames = 0;

    // 2. 主循环阶段：只要处于会话大周期内，任务就一直运行
    while (s_state == SESSION_LISTENING || s_state == SESSION_PLAYING)
    {
        // 核心原则：不管网络状态如何，必须先把编码器的数据读出来（抽水），防止内存爆掉
        size_t len = audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);

        if (len > 0)
        {
            // 【核心阀门机制】
            // 只有当前拥有 SERVER_READY 权限（且连接正常），才往外发包
            if ((xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT) &&
                s_protocol && protocol_is_connected(s_protocol) && !s_stop_sent)
            {
                binary_data_t bin = {.ptr = buf, .size = len};
                protocol_send_audio_data(s_protocol, &bin);

                sent_frames++;
                if (sent_frames % 50 == 1) // 每 50 帧（约 3 秒）打印一次
                    ESP_LOGI(TAG, "OPUS 发送中: frame#%d size=%d", sent_frames, (int)len);
            }
            // 💡 如果没进上面的 if，说明此时处于静音等待期（或刚报完错）。
            // 读出来的音频数据会直接被静默丢弃，绝对不会发给服务器惹祸。
        }
    }

    ESP_LOGI(TAG, "发送任务结束，共发送 %d 帧 OPUS", sent_frames);

exit:
    s_sender_handle = NULL;
    vTaskDelete(NULL);
}
// ─── Token 刷新 + WebSocket 重连任务 ────────────────────────────────────────

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
        ESP_LOGI(TAG, "关闭旧 WebSocket 连接（释放服务端连接计数）...");
        protocol_disconnect(s_protocol); // 发送 close frame，关闭 TCP
        protocol_destroy(s_protocol);
        s_protocol = NULL;
        vTaskDelay(pdMS_TO_TICKS(500));
    }

    char *full_ws_uri = (char *)malloc_zeroed(1024);
    if (full_ws_uri == NULL)
    {
        ESP_LOGE(TAG, "full_ws_uri 分配失败，放弃重连");
        s_reconnect_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    if (strlen(new_token) > 0)
        snprintf(full_ws_uri, 1024, "%s?token=%s", s_ws_uri, new_token);
    else
        strncpy(full_ws_uri, s_ws_uri, 1024 - 1);

    s_protocol = protocol_create(full_ws_uri, new_token);
    if (s_protocol == NULL)
    {
        ESP_LOGE(TAG, "protocol_create 失败，内存不足，放弃重连");
        free(full_ws_uri);
        s_reconnect_handle = NULL;
        vTaskDelete(NULL);
        return;
    }
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);

    ESP_LOGI(TAG, "重连完成，WebSocket 正在建立连接... URI: %s", s_ws_uri);

    free(full_ws_uri);
    s_reconnect_handle = NULL;
    vTaskDelete(NULL);
}

// ─── 关闭会话（保持 WebSocket 连接）────────────────────────────────────────

static void session_close(void)
{
    if (s_state == SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "关闭会话");
    s_state = SESSION_IDLE; // 先置状态，sender 任务循环条件会检测到退出
    s_speech_detected = false;

    // 先停止 PCM Hook，防止新数据继续写入已停止的编码器
    bsp_wake_word_set_enhanced_pcm_hook(NULL);

    // 释放发送任务的阻塞
    xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);

    xTimerStop(s_session_timer, 0);
    xTimerStop(s_eos_timer, 0);

    if (s_processor != NULL)
    {
        audio_processor_stop(s_processor);
        for (int i = 0; i < 10 && s_sender_handle != NULL; i++)
            vTaskDelay(pdMS_TO_TICKS(100));
        if (s_sender_handle != NULL)
            ESP_LOGW(TAG, "sender 任务未能在超时内退出");
        audio_processor_destroy(s_processor);
        s_processor = NULL;
    }

    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);

    bsp_wake_word_start();
    ESP_LOGI(TAG, "会话已关闭，WebSocket 保持连接，重新监听唤醒词...");
    PRINT_MEM_INFO(TAG, "对话会话结束清理后");
}

// ─── 公开 API：初始化 + 预连接 ──────────────────────────────────────────────

void session_init(const char *ws_uri)
{
    s_session_eg = xEventGroupCreate();
    s_wake_word_mutex = xSemaphoreCreateMutex();

    // ── 【新增】初始化事件队列与任务 ──
    if (s_session_evt_queue == NULL)
    {
        s_session_evt_queue = xQueueCreate(5, sizeof(session_evt_t));
    }
    if (s_session_evt_task == NULL)
    {
        // 分配 8KB 栈在外部 PSRAM，保护内部 RAM 并彻底告别栈溢出
        xTaskCreatePinnedToCoreWithCaps(session_event_task, "session_evt_tsk",
                                        8192, NULL, 5,
                                        &s_session_evt_task,
                                        1, MALLOC_CAP_SPIRAM);
    }

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

    s_token_refresh_timer = xTimerCreate("token_ref",
                                         pdMS_TO_TICKS(TOKEN_REFRESH_MS),
                                         pdTRUE, NULL, on_token_refresh_timeout);
    if (strlen(s_ws_token) > 0)
        xTimerStart(s_token_refresh_timer, 0);

    char full_ws_uri[1024] = {0};

    if (strlen(ws_bearer_token) > 0)
    {
        snprintf(full_ws_uri, sizeof(full_ws_uri), "%s?token=%s", s_ws_uri, ws_bearer_token);
        for (int i = strlen(full_ws_uri) - 1; i >= 0 && (full_ws_uri[i] == ' ' || full_ws_uri[i] == '\n' || full_ws_uri[i] == '\r'); i--)
        {
            full_ws_uri[i] = '\0';
        }
    }
    else
    {
        strncpy(full_ws_uri, s_ws_uri, sizeof(full_ws_uri) - 1);
    }

    ESP_LOGW(TAG, "最终请求的完整 WebSocket URI: %s", full_ws_uri);

    s_protocol = protocol_create(full_ws_uri, ws_bearer_token);
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);

    if (strlen(s_access_token) > 0)
        ESP_LOGI(TAG, "已使用 accessToken 认证连接（%d 分钟后自动刷新）", TOKEN_REFRESH_MS / 60000);
}

// ─── 公开 API：唤醒词触发 ──────────────────────────────────────────────────

void session_on_wake_word(const char *display)
{
    PRINT_MEM_INFO(TAG, "对话会话开始");
    if (s_state == SESSION_PLAYING)
    {
        ESP_LOGW(TAG, "⚠️ 唤醒词打断 TTS: [%s]", display);
        if (s_protocol && protocol_is_connected(s_protocol))
        {
            protocol_send_abort_speaking(s_protocol);
        }
        audio_processor_flush_output(s_processor);
        s_state = SESSION_LISTENING;
        s_speech_detected = false;
        xTimerReset(s_session_timer, 0);
        return;
    }

    if (s_state != SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "会话开始 [%s]", display);
    s_state = SESSION_LISTENING;
    s_speech_detected = false;
    s_stop_sent = false;
    s_vad_ready_tick = 0;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);

    xSemaphoreTake(s_wake_word_mutex, portMAX_DELAY);
    strncpy(s_current_wake_word, display, sizeof(s_current_wake_word) - 1);
    s_current_wake_word[sizeof(s_current_wake_word) - 1] = '\0';
    xSemaphoreGive(s_wake_word_mutex);

    s_processor = audio_processor_create();
    if (!s_processor)
    {
        ESP_LOGE(TAG, "音频处理器创建失败，内存不足");
        goto error;
    }
    audio_processor_start(s_processor);

    bsp_wake_word_set_enhanced_pcm_hook(on_enhanced_pcm);

    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(ws_sender_task, "ws_sender",
                                                     4096, NULL, 5,
                                                     (TaskHandle_t *)&s_sender_handle,
                                                     0, MALLOC_CAP_SPIRAM);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "发送任务创建失败");
        bsp_wake_word_set_enhanced_pcm_hook(NULL);
        audio_processor_stop(s_processor);
        audio_processor_destroy(s_processor);
        s_processor = NULL;
        goto error;
    }

    if (xEventGroupGetBits(s_session_eg) & SESSION_WS_CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "WebSocket 已就绪，立即发送 Hello");
        protocol_send_hello(s_protocol);
    }
    else
    {
        ESP_LOGW(TAG, "WebSocket 未就绪，等待重连后自动发送 Hello...");
    }

    xTimerStart(s_session_timer, 0);
    return;

error:
    ESP_LOGE(TAG, "会话启动失败，回滚所有资源");
    s_state = SESSION_IDLE;
    s_speech_detected = false;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
    bsp_wake_word_start();
}

session_state_t session_get_state(void)
{
    return s_state;
}