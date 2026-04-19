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

// WebSocket服务器默认URI地址
#define DEFAULT_WS_URI "ws://192.168.1.100:8080/audio"
// NVS存储命名空间，用于存储网络配置
#define NVS_NAMESPACE_NET "net_config"
// 整体会话超时时间：30秒，超过此时间无活动则关闭会话
#define SESSION_TIMEOUT_MS 30000
// 说话结束静音检测时间：500ms，用于判断用户是否说完话
#define EOS_SILENCE_MS 800
// Token主动刷新时间：110分钟（accessToken过期时间为2小时，提前10分钟刷新）
#define TOKEN_REFRESH_MS (110 * 60 * 1000)
// OPUS音频帧发送缓冲区大小
#define OPUS_SEND_BUF 512

// ─── 事件组位定义 ────────────────────────────────────────────────────────────
// 会话事件组句柄，用于任务间同步
static EventGroupHandle_t s_session_eg = NULL;
// 位定义：Hello握手完成，可以发送音频数据
#define SESSION_SERVER_READY_BIT BIT0
// 位定义：WebSocket底层已连接成功
#define SESSION_WS_CONNECTED_BIT BIT1

// ─── 【新增】事件队列定义（用于将网络操作从定时器中剥离） ──────────────────────
/**
 * @brief 会话事件类型枚举
 * 定义会话模块需要处理的异步事件类型
 */
typedef enum
{
    SESSION_EVT_VAD_STOP, // VAD检测到说话结束事件
    SESSION_EVT_CLOSE,    // 异步关闭会话信号
    // 未来可扩展其他耗时事件...
} session_evt_t;

// 会话事件队列句柄，用于在不同任务间传递事件
static QueueHandle_t s_session_evt_queue = NULL;
// 会话事件处理任务句柄
static TaskHandle_t s_session_evt_task = NULL;

// ─── 模块级状态变量 ──────────────────────────────────────────────────────────
// 当前会话状态（SESSION_IDLE, SESSION_LISTENING, SESSION_PLAYING）
static volatile session_state_t s_state = SESSION_IDLE;
// 音频处理器实例指针
static audio_processor_t *s_processor = NULL;

// 【关键】Protocol是持久对象，在session_init中创建，整个生命周期不销毁
static protocol_t *s_protocol = NULL;

// 会话超时定时器句柄
static TimerHandle_t s_session_timer = NULL;
// 说话结束检测定时器句柄
static TimerHandle_t s_eos_timer = NULL;
// accessToken主动刷新定时器句柄
static TimerHandle_t s_token_refresh_timer = NULL;
// 标记是否检测到语音活动
static volatile bool s_speech_detected = false;
// 标记stop消息是否已发送（防止重复发送）
static volatile bool s_stop_sent = false;
// 标记是否为连续会话模式
static bool s_is_continuous_turn = false;
// 服务器就绪时刻（用于VAD延迟启动基准）
static TickType_t s_vad_ready_tick = 0;
// static TickType_t s_tts_stop_tick = 0;
static bool s_waiting_for_silence = false;
static int s_vad_silence_count = 0;
static TickType_t s_wait_silence_start = 0;
// 唤醒词尾音消退期：1500ms，避免唤醒词尾音被误判为用户语音
// 实测唤醒词尾音 + 硬件排空延迟约 1.2s，保护期需覆盖此窗口
#define VAD_GRACE_MS 1000
// static bool s_waiting_for_silence = false; // 是否正在等待喇叭排空
// static int s_vad_silence_count = 0;        // 连续静音帧计数器
// VAD 连续帧防抖：连续确认帧数 & 当前计数
// AFE fetch 约 32ms/帧，3 帧 ≈ 96ms 持续语音才确认为"真实人声"
// 防止环境音/短促噪声单帧触发 s_speech_detected，导致误报给服务器
// 演化：8 帧(256ms 吞首字) → 6 帧(192ms 仍丢"和"/"今"擦音字) → 3 帧(96ms 配合 1 帧静音容忍)
#define VAD_SPEECH_CONFIRM_FRAMES 3
// 静音容忍：连续 ≤1 帧 SILENCE 不清零 speech_count，防止 VAD 单帧抖动吃掉首字
// 第 2 帧静音才真清零（≈ 64ms 静音 = 真停顿）
#define VAD_SILENCE_TOLERANCE_FRAMES 1
static int s_vad_speech_count = 0;   // 连续 VAD_SPEECH 帧计数
static int s_vad_silence_streak = 0; // 当前连续 VAD_SILENCE 帧计数（用于 1 帧容忍判定）
// WebSocket服务器URI地址
static char s_ws_uri[128] = DEFAULT_WS_URI;
// deviceToken（App绑定时下发的长期凭证）
static char s_ws_token[256] = {0};
// accessToken（通过device-login换取的短效令牌）
static char s_access_token[512] = {0};
// WebSocket发送任务句柄
static volatile TaskHandle_t s_sender_handle = NULL;
// WebSocket重连任务句柄，防止重复创建
static volatile TaskHandle_t s_reconnect_handle = NULL;
// 连续重连次数，用于指数退避算法
static int s_reconnect_attempts = 0;
// 最大重连次数，超过后停止重连
#define RECONNECT_MAX_ATTEMPTS 5
// 基础退避延迟时间：5秒
#define RECONNECT_BASE_DELAY_MS 500
// 唤醒词操作互斥锁，防止多线程操作冲突
static SemaphoreHandle_t s_wake_word_mutex = NULL;
// 当前触发的唤醒词字符串
static char s_current_wake_word[64] = {0};
static bool s_reset_frame_counter = false; // 是否重置帧计数器（用于连续会话时区分第一轮和后续轮次）
// 函数声明
static void session_close(void);
static void on_enhanced_pcm(const int16_t *data, size_t samples);
static void session_reconnect_task(void *arg);

// ─── AEC 参考信号提供者 ───────────────────────────────────────────────────
/**
 * @brief AEC（回声消除）参考信号提供回调函数
 *
 * 说明：AFE（Audio Front End）每帧调用此回调获取参考PCM信号（即当前扬声器播放的音频），
 *       用于回声消除算法。当无会话或无播放时返回零值，AFE AEC会安全退化为纯降噪模式。
 * API：audio_processor_read_ref_pcm, memset
 * 数据：向buf参数填充参考PCM数据
 */
static void aec_ref_provider(int16_t *buf, size_t samples)
{
    if (s_processor != NULL)
        audio_processor_read_ref_pcm(s_processor, buf, samples);
    else
        memset(buf, 0, samples * sizeof(int16_t));
}

/**
 * @brief 异步会话关闭任务
 * 创建一个独立任务来执行会话关闭操作，避免在回调中直接关闭导致的问题
 *
 * @param arg 任务参数（未使用）
 *
 * 调用者：protocol_event_handler中的PROTOCOL_EVENT_ERROR事件处理
 */
static void async_session_close_task(void *arg)
{
    session_close();
    vTaskDelete(NULL);
}

// ─── 【新增】会话专职网络事件任务 ─────────────────────────────────────────────
/**
 * @brief 会话事件处理任务
 * 专门处理会话相关的网络事件，从队列中接收事件并执行相应的网络操作
 * 这样可以避免在网络回调或定时器回调中直接执行耗时的网络操作
 *
 * @param arg 任务参数（未使用）
 *
 * 调用者：session_init函数中创建
 */
static void session_event_task(void *arg)
{
    session_evt_t evt;
    ESP_LOGI(TAG, "session_event_task启动，...");

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
                    int wait_count = 0;
                    while (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT) && wait_count < 10)
                    {
                        vTaskDelay(pdMS_TO_TICKS(50)); // 50ms 间隔，总上限 2.5s，减少 stop_listening 发出延迟
                        wait_count++;
                    }
                    // 在这里执行耗时的 WebSocket 发送操作，绝对不会导致定时器栈溢出！
                    protocol_send_stop_listening(s_protocol);
                }
                else
                {
                    // 兜底：WS 已断但 stop 未发出，避免会话卡死等到 60s 主超时
                    // 主动触发 CLOSE，让 session 层走正常的清理+重连流程
                    ESP_LOGW(TAG, "[ERR] 发送 stop 时 WS 已断，直接关闭会话避免卡死");
                    session_evt_t close_evt = SESSION_EVT_CLOSE;
                    // 含义：发送事件到队列,如果队列满了事件会丢失，
                    xQueueSend(s_session_evt_queue, &close_evt, 0);
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

/**
 * @brief 会话超时定时器回调
 * 当会话在指定时间内无任何活动时触发，自动关闭会话以释放资源
 *
 * @param t 定时器句柄
 *
 * 调用者：FreeRTOS定时器系统
 */
static void on_session_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "会话超时（%d 秒无活动），关闭会话", SESSION_TIMEOUT_MS / 1000);
    // session_close();  // ❌ 必须删掉！定时器里直接关机会导致栈溢出死机！

    // ✅ 修复：通过专用的会话事件队列发送关闭信号，安全可靠
    if (s_session_evt_queue != NULL)
    {
        session_evt_t evt = SESSION_EVT_CLOSE;
        xQueueSend(s_session_evt_queue, &evt, 0);
    }
}

/**
 * @brief 说话结束检测定时器回调
 * 当检测到静音持续指定时间后，认为用户已说完话，发送stop消息给服务器
 *
 * @param t 定时器句柄
 *
 * 调用者：FreeRTOS定时器系统
 */
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
 *
 * @param t 定时器句柄
 *
 * 调用者：FreeRTOS定时器系统
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
        // ★ 必须用内部 SRAM：auth_perform 调用 nvs_set_str → SPI Flash 写入 →
        //   关闭 Cache，SPIRAM 栈在 Cache 关闭后不可访问，会触发 assert 崩溃
        xTaskCreatePinnedToCoreWithCaps(session_reconnect_task, "ws_reconn",
                                        6144, (void *)(intptr_t)0, 3,
                                        (TaskHandle_t *)&s_reconnect_handle,
                                        1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    }

    // 重置定时器为标准周期（如果上面是延迟重试进来的）
    xTimerChangePeriod(s_token_refresh_timer, pdMS_TO_TICKS(TOKEN_REFRESH_MS), 0);
}

// ─── 麦克风 PCM 数据回调 ────────────────────────────────────────────────────

/**
 * @brief 增强PCM数据回调函数
 * 处理从麦克风采集到的PCM音频数据，进行VAD检测和音频处理
 *
 * @param data PCM音频数据指针
 * @param samples 采样点数量
 *
 * 调用者：wake_word_set_enhanced_pcm_hook设置的回调
 */
static void on_enhanced_pcm(const int16_t *data, size_t samples)
{
    if (s_processor == NULL)
        return;

    // PLAYING 状态不送编码器（防回声）
    // 唤醒词引擎由 audio_feed_task → custom_wake_word_feed 独立运行
    // 【修复】s_stop_sent 后也不再写入 enc_input：
    //   stop_listening 已发送，继续编码纯属浪费 CPU0 资源，
    //   编码器/解码器/播放任务全在 CPU0 优先级 5 竞争，
    //   编码无用 PCM 会饿死解码和播放任务 → TTS 只听到几个字
    if (s_state != SESSION_LISTENING || s_stop_sent)
        return;

    // ── VAD 检测（仅在服务器就绪 + 消退期后启用）──────────────────
    // 条件：服务器已就绪、消退期已过、stop 尚未发送
    // vad_state_t vad = wake_word_get_vad_state();

    // ── 防抖：连续 N 帧 VAD_SPEECH 才确认为真实人声 ──────────
    // AFE 约 32ms/帧，VAD_SPEECH_CONFIRM_FRAMES=5 → 需要 160ms 持续语音
    // 环境音/短促噪声通常只触发 1~2 帧，不足 N 帧则直接忽略
    // ★ 阶段 2：正式监听期（喇叭已闭嘴，开始抓用户声音）

    if (!s_stop_sent)
    {
        vad_state_t vad = wake_word_get_vad_state();

        // 🟢 阶段 1：扬声器动态排空期 (等待物理播放彻底结束)
        if (s_waiting_for_silence)
        {
            uint32_t wait_ms = (xTaskGetTickCount() - s_wait_silence_start) * portTICK_PERIOD_MS;

            // 1. 强制物理静音期：前 400ms 内，不管 VAD 说啥，强制认为还在响
            // 喇叭物理震动的余波最危险
            if (wait_ms < 400)
            {
                s_vad_silence_count = 0;
                return; // 丢弃音频
            }

            if (vad == VAD_SILENCE)
            {
                s_vad_silence_count++;
                // 2. 将连续静音帧数提高到 15 帧（约 480ms），确保空气中真的没有余音了
                if (s_vad_silence_count > 15)
                {
                    s_waiting_for_silence = false;
                    ESP_LOGI(TAG, ">>> 扬声器排空完毕 (耗时 %d ms)，开启监听 <<<", (int)wait_ms);
                    s_vad_ready_tick = xTaskGetTickCount();
                }
            }
            else
            {
                s_vad_silence_count = 0; // 只要有声音，就重置静音计数器继续死等
            }

            // 防卡死兜底：如果等了 8 秒喇叭还没播完（或者环境太吵），强行切入监听
            if (wait_ms > 8000)
            {
                s_waiting_for_silence = false;
                ESP_LOGW(TAG, ">>> 排空等待超时 (8s)，强制开启监听 <<<");
                s_vad_ready_tick = xTaskGetTickCount();
            }
            return; // ⚠️ 排空期间，丢弃所有音频数据，绝对不触发人声判定！
        }
        // ==========================================
        // 🟢 新增防御：软件噪音门限 (Noise Gate)
        // 专门对付下雨、风扇、空调等持续型环境白噪音
        // ==========================================
        uint32_t sum_amp = 0;
        for (size_t i = 0; i < samples; i++)
        {
            int16_t val = data[i];
            sum_amp += (val < 0 ? -val : val); // 取绝对值
        }
        uint32_t avg_amp = sum_amp / samples; // 算出当前帧的平均振幅音量

        // 如果底层 VAD 误判为有声音，但实际音量非常小（背景白噪音）
        // 我们强行推翻底层的判定，将其压制为静音！
        // 💡 这里的 200 是阈值。如果雨下得大依然误触发，可以改成 250 或 300。
        // 代价是：阈值越高，你离它太远小声说话时，它可能也会听不见。
        if (vad == VAD_SPEECH && avg_amp < 150)
        {
            vad = VAD_SILENCE;
        }
        // 🟢 阶段 2：正常监听用户讲话 (确保此处没有重复定义 in_grace_period)
        bool in_grace_period = (s_vad_ready_tick != 0) &&
                               ((xTaskGetTickCount() - s_vad_ready_tick) < pdMS_TO_TICKS(VAD_GRACE_MS));

        int current_threshold = in_grace_period ? 14 : VAD_SPEECH_CONFIRM_FRAMES;

        bool is_active_audio = false;
        if (vad == VAD_SPEECH)
        {
            is_active_audio = true; // 只要有声音，先存下来再说！(这就是保住"今天"这俩字的关键)
            s_vad_speech_count++;
            s_vad_silence_streak = 0; // 收到语音帧，清零静音连击计数

            // 使用动态阈值 current_threshold 来判断
            if (!s_speech_detected && s_vad_speech_count >= current_threshold)
            {
                s_speech_detected = true;
                ESP_LOGI(TAG, "[MIC] VAD 确认人声（连续 %d 帧, 保护期内:%d），有效输入！",
                         s_vad_speech_count, in_grace_period);

                if (s_is_continuous_turn)
                {
                    if (s_protocol && protocol_is_connected(s_protocol))
                    {
                        protocol_send_hello(s_protocol);
                        ESP_LOGI(TAG, "连续对话：已向服务器补发 Start 握手");
                    }
                    s_is_continuous_turn = false;
                }
            }
            if (s_eos_timer)
                xTimerStop(s_eos_timer, 0);
        }
        else // VAD_SILENCE (静音)
        {
            // ★ 关键：单帧静音容忍机制（防 VAD 抖动吃首字）
            //   旧逻辑：只要中间断开 1 帧 silence 就立刻清零 speech_count → 累积一半夭折
            //   现逻辑：连续 ≤1 帧 silence 时不清零（≈ 32ms 抖动），第 2 帧才真清零（≈ 64ms 真停顿）
            //   同时把这 1~2 帧静音的 PCM 也录进去（is_active_audio = true），
            //   防止凑帧期间出现"语音 - 静音 - 语音"被切成两半
            s_vad_silence_streak++;
            if (s_vad_silence_streak > VAD_SILENCE_TOLERANCE_FRAMES)
            {
                // 超出容忍帧数，确认是真停顿，重置 speech_count
                s_vad_speech_count = 0;
            }
            else if (!s_speech_detected && s_vad_speech_count > 0)
            {
                // 容忍期内未确认状态：把这帧静音算作"准语音"录入，避免凑帧出现断点
                is_active_audio = true;
            }

            if (s_speech_detected)
            {
                is_active_audio = true; // 已确认为句子，呼吸/停顿也要录，保持句子连贯
                if (s_eos_timer && xTimerIsTimerActive(s_eos_timer) == pdFALSE)
                {
                    xTimerStart(s_eos_timer, 0);
                }
            }
        }
        if (is_active_audio)
        {
            audio_processor_write_pcm(s_processor, (void *)data, samples * sizeof(int16_t));
        }
    }
}

// ─── 核心：协议层事件分发器 ─────────────────────────────────────────────────

/**
 * @brief 协议层事件处理函数
 * 处理WebSocket协议层的各种事件，包括连接、断开、消息接收等
 * 根据不同的事件ID执行相应的状态转换和操作
 *
 * @param handler_args 用户自定义参数（此处为protocol_t指针）
 * @param base 事件基类（PROTOCOL_EVENT）
 * @param event_id 具体事件ID
 * @param event_data 事件相关数据
 *
 * 调用者：websocket_client模块通过esp_event框架调用
 */
static void protocol_event_handler(void *handler_args, esp_event_base_t base,
                                   int32_t event_id, void *event_data)
{
    switch (event_id)
    {
    // ── WebSocket 底层连接成功（启动时预连接，或断线重连后）──────────────
    /**
     * @brief WebSocket连接成功事件处理
     *
     * 说明：处理WebSocket底层连接成功的事件，重置重连计数并设置连接状态位。
     *       如果有会话正在等待连接，则立即发送Hello握手消息。
     * API：xEventGroupSetBits
     * 数据：修改s_reconnect_attempts和s_session_eg
     */
    case PROTOCOL_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket 已连接（预连接就绪）");
        s_reconnect_attempts = 0; // 连接成功，重置退避计数
        xEventGroupSetBits(s_session_eg, SESSION_WS_CONNECTED_BIT);
        xTimerChangePeriod(s_session_timer, pdMS_TO_TICKS(SESSION_TIMEOUT_MS), 0);
        // 如果有会话正在等待连接（极少数情况：唤醒时恰好断线重连中）
        if (s_state == SESSION_LISTENING)
        {
            ESP_LOGI(TAG, "会话等待中，立即发送 Hello");
            protocol_send_hello(s_protocol);
        }
        break;

    // ── 收到服务器 Hello 响应 → 握手完成 ────────────────────────────────
    /**
     * @brief 服务器Hello响应事件处理
     *
     * 说明：处理收到服务器Hello响应的事件，表示握手完成，会话已建立。
     *       重置VAD状态，设置服务器就绪标志位，并启动会话超时定时器。
     * API：xEventGroupSetBits, xTimerReset
     * 数据：修改s_vad_ready_tick, s_speech_detected, s_stop_sent和s_session_eg
     */
    case PROTOCOL_EVENT_HELLO:
        if (s_state == SESSION_IDLE)
            break; // 无活跃会话，忽略
        ESP_LOGI(TAG, "收到服务器 started 响应，会话已建立");

        s_stop_sent = false;

        xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);
        xTimerReset(s_session_timer, 0);
        ESP_LOGI(TAG, "服务器就绪，开始推送 Opus 音频流");
        break;

    // ── TTS 开始 → PLAYING，重启唤醒词引擎支持打断 ──────────────────────
    /**
     * @brief TTS开始事件处理
     *
     * 说明：处理TTS开始播放的事件，切换到PLAYING状态，停止EOS定时器，
     *       重启唤醒词引擎以支持在播放过程中检测打断唤醒词。
     * API：wake_word_start, xTimerStop, xTimerReset
     * 数据：修改s_state, s_speech_detected
     */
    case PROTOCOL_EVENT_TTS_START:
        ESP_LOGI(TAG, "[TTS] 服务器 TTS 开始播放");
        s_state = SESSION_PLAYING;
        s_speech_detected = false;
        s_vad_speech_count = 0; // 重置防抖计数，TTS 播放中不再累计语音帧
        xTimerStop(s_eos_timer, 0);
        xTimerReset(s_session_timer, 0);
        // 重启唤醒词引擎，TTS 期间可以检测打断唤醒词
        wake_word_start();
        break;

    case PROTOCOL_EVENT_TTS_SENTENCE_START:
        ESP_LOGI(TAG, "[TTS] TTS: %s", (char *)event_data);
        break;

    // ── TTS 结束 → 恢复 LISTENING，继续多轮对话 ────────────────────────
    /**
     * @brief TTS结束事件处理
     *
     * 说明：处理TTS播放结束的事件，恢复到LISTENING状态，准备继续多轮对话。
     *       重置相关状态标志，并启动连续对话模式。
     * API：xTimerReset
     * 数据：修改s_state, s_speech_detected, s_stop_sent, s_is_continuous_turn
     *
     * 方案 B（多轮，当前启用）：TTS 结束后保持连接，等用户下一句话。
     * 方案 A（单轮，已禁用）：TTS 结束后直接关闭会话，每次都要重新唤醒。
     */
    case PROTOCOL_EVENT_TTS_STOP:
#if 1 /* ✅ 方案 B：多轮对话（当前启用） */
        ESP_LOGI(TAG, "[TTS] TTS 播放结束，切换到多轮监听，等待用户下一句话");
        s_state = SESSION_LISTENING;
        s_speech_detected = false;
        s_stop_sent = false;
        s_is_continuous_turn = true;
        s_reset_frame_counter = true; // ★ 每次 AI 闭嘴，准备新一轮记录
        // // ★ 核心修复：不再依赖写死的消退时间，改为动态检测静音
        // s_waiting_for_silence = true;
        // s_vad_silence_count = 0;
        s_waiting_for_silence = true;
        s_vad_silence_count = 0;
        s_wait_silence_start = xTaskGetTickCount();
        s_vad_speech_count = 0;
        s_vad_ready_tick = 0; // ★ 关键修复：重置服务器就绪时刻，确保 VAD 保护期正确计算
        // s_vad_ready_tick = xTaskGetTickCount();

        xTimerReset(s_session_timer, 0);
        xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
        wake_word_stop();
#else /* ❌ 方案 A：单轮对话（已禁用，仅保留作参考） */
        ESP_LOGI(TAG, "[TTS] TTS 播放结束，单轮模式关闭会话");
        s_is_continuous_turn = false;
        if (s_session_evt_queue != NULL)
        {
            session_evt_t evt = SESSION_EVT_CLOSE;
            xQueueSend(s_session_evt_queue, &evt, 0);
        }
#endif
        break;

    case PROTOCOL_EVENT_STT:
        ESP_LOGI(TAG, "[STT] 语音转文字(STT): %s", (char *)event_data);
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_LLM:
        ESP_LOGI(TAG, "[LLM] 大模型状态: %s", (char *)event_data);
        xTimerReset(s_session_timer, 0);
        break;

    case PROTOCOL_EVENT_AUDIO:
        if (s_state != SESSION_IDLE && s_processor != NULL)
        {
            binary_data_t *bin = (binary_data_t *)event_data;

            // ★ Opus 防爆音装甲 ★
            // 标准的 Opus 语音帧极少小于 15 字节。
            // 如果收到极小的碎片，极大概率是网络抖动导致的 TCP 粘包/断包残骸。
            // 直接丢弃这些垃圾碎片，防止它们污染解码器导致 error:-4 卡磁破音。
            if (bin->size > 15)
            {
                audio_processor_write(s_processor, bin->ptr, bin->size);
            }
            else
            {
                ESP_LOGW(TAG, "丢弃异常音频碎片 (size=%d)，防止 Opus 解码错位爆音", bin->size);
            }
            xTimerReset(s_session_timer, 0);
        }
        break;

        // ── WebSocket 断开 → 退避重连 ─────────────────────────────────────────
        /**
         * @brief WebSocket断开连接事件处理
         *
         * 说明：处理WebSocket断开连接的事件，清除连接状态位，根据当前会话状态
         *       决定是否需要重连，并实现指数退避重连策略。
         * API：xEventGroupClearBits, xTimerChangePeriod, xTaskCreatePinnedToCore
         * 数据：修改s_reconnect_attempts, s_reconnect_handle
         */
    case PROTOCOL_EVENT_DISCONNECTED:
    { // ★ 注意这里的大括号，用于限定局部变量作用域
        ESP_LOGW(TAG, "[ERR] WebSocket 已断开连接");
        xEventGroupClearBits(s_session_eg, SESSION_WS_CONNECTED_BIT | SESSION_SERVER_READY_BIT);

        // ★ 核心修复 2：防抖机制，拦截底层网络碎片化导致的疯狂重复断开
        static TickType_t s_last_disconnect_tick = 0;
        if (xTaskGetTickCount() - s_last_disconnect_tick < pdMS_TO_TICKS(1000))
        {
            ESP_LOGW(TAG, "网络轻微抖动，已拦截重复的断开事件");
            break;
        }
        s_last_disconnect_tick = xTaskGetTickCount();

        if (s_state != SESSION_IDLE)
        {
            // ★ 核心修复 3：会话中途断网，直接终止当前对话释放内存！防死锁！
            ESP_LOGW(TAG, "会话中网络断开，直接终止当前对话释放内存，待机再重连！");
            s_is_continuous_turn = false;
            if (s_session_evt_queue != NULL)
            {
                session_evt_t evt = SESSION_EVT_CLOSE;
                xQueueSend(s_session_evt_queue, &evt, 0);
            }
        }

        // 闲置待机状态，才允许发起正常的重连
        if (s_reconnect_handle != NULL)
        {
            ESP_LOGW(TAG, "重连任务已在运行中，跳过");
        }
        else if (s_reconnect_attempts >= RECONNECT_MAX_ATTEMPTS)
        {
            ESP_LOGE(TAG, "已连续重连 %d 次均失败，停止重连。", s_reconnect_attempts);
        }
        else
        {
            s_reconnect_attempts++;
            int delay_ms = 5000 * (1 << (s_reconnect_attempts - 1));
            if (delay_ms > 60000)
                delay_ms = 60000;
            ESP_LOGW(TAG, "第 %d 次重连，%d 秒后执行...", s_reconnect_attempts, delay_ms / 1000);
            xTaskCreatePinnedToCoreWithCaps(session_reconnect_task, "ws_reconn",
                                            6144, (void *)(intptr_t)delay_ms, 3,
                                            (TaskHandle_t *)&s_reconnect_handle,
                                            1, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        }
        break;
    } // ★ 对应开头的大括号
    // ── 收到错误 → 不退出，直接重新进入录音状态 ────────────────────────────────
    case PROTOCOL_EVENT_ERROR:
        ESP_LOGE(TAG, "[ERR] 收到服务器报错: %s", event_data ? (char *)event_data : "未知");

        if (event_data != NULL && strstr((char *)event_data, "Error committing input audio buffer") != NULL)
        {
            ESP_LOGW(TAG, "检测到云端并发 Bug，已启动免疫机制，保持会话存活！");
            break; // 直接跳出，绝对不发送 CLOSE 信号
        }
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
    // ── 收到完成信号 → 多轮模式下忽略，单轮模式下关闭会话 ──────────────────
    /**
     * @brief COMPLETE 事件处理
     *
     * 多轮模式（方案 B，当前启用）：
     *   TTS_STOP 已经把状态切回 LISTENING 并等待用户下一句话。
     *   COMPLETE 是本轮（上一轮）AI 回复结束的信号，此时应继续监听。
     *
     * 竞态保护：
     *   s_is_continuous_turn 在 on_enhanced_pcm 发出下一轮 Hello 时被清为 false。
     *   因此不能依赖它来判断是否该忽略 COMPLETE——改用 s_state 判断：
     *   只要处于 SESSION_LISTENING，说明当前会话仍在进行（等待用户或已进入第 N 轮），
     *   均不关闭；由 session_timer 超时（60s 无活动）统一负责自然结束。
     */
    case PROTOCOL_EVENT_COMPLETE:
        if (s_state == SESSION_LISTENING)
        {
            // 多轮模式：处于监听状态，忽略 COMPLETE，继续等待用户输入
            // 若用户不再说话，session_timer 60s 超时后自动关闭
            ESP_LOGI(TAG, "多轮模式：收到 Complete，保持监听等待下一轮");
            s_speech_detected = false;
            s_stop_sent = false;
            s_is_continuous_turn = true;
            s_reset_frame_counter = true;
            s_vad_speech_count = 0;
            s_vad_silence_streak = 0; // 多轮模式：进入下一轮前清零静音连击，避免容忍机制残留
            s_vad_ready_tick = xTaskGetTickCount();
            xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
            xTimerReset(s_session_timer, 0);
        }
        else if (s_state != SESSION_IDLE)
        {
            // PLAYING 或其他活跃状态收到 COMPLETE（极少见的异常路径），安全关闭
            ESP_LOGI(TAG, "收到 Complete，关闭会话");
            s_is_continuous_turn = false;
            if (s_session_evt_queue != NULL)
            {
                session_evt_t evt = SESSION_EVT_CLOSE;
                xQueueSend(s_session_evt_queue, &evt, 0);
            }
        }
        break;
    default:
        break;
    }
}

// ─── WebSocket 发送任务 ─────────────────────────────────────────────────────

/**
 * @brief WebSocket音频发送任务
 * 负责从音频处理器读取编码后的OPUS数据并通过WebSocket发送到服务器
 * 包含两个阶段：等待服务器就绪阶段和主循环发送阶段
 *
 * @param arg 任务参数（未使用）
 *
 * 调用者：session_on_wake_word函数中创建
 */
static void ws_sender_task(void *arg)
{
    uint8_t buf[OPUS_SEND_BUF];

    ESP_LOGI(TAG, "发送任务启动，等待服务器就绪...");

    // 1. 开局等待阶段：等待服务器 started 响应，不排空 enc_output
    //    enc_output（8KB ≈ 2.7s OPUS）足以缓冲 hello/started RTT 期间的帧，
    //    服务器就绪后阶段2直接发送，唤醒词后立即说话的语音不再丢失。
    while (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
    {
        if (s_state != SESSION_LISTENING && s_state != SESSION_PLAYING)
            goto exit;                 // 会话已关闭，直接退出
        vTaskDelay(pdMS_TO_TICKS(20)); // 等待，不消耗 enc_output
    }

    ESP_LOGI(TAG, "服务器已就绪，发送任务运行中");

    int sent_frames = 0;

    // 2. 主循环阶段：只要处于会话大周期内，任务就一直运行
    while (s_state == SESSION_LISTENING || s_state == SESSION_PLAYING)
    {
        if (s_reset_frame_counter)
        {
            sent_frames = 0;
            s_reset_frame_counter = false;
        }
        if (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
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
                // 让步 1 tick（≈ 1ms）即可：encoder_task 已经在每帧编码后让 2ms，
                // 这里再让 2ms 会拖慢发送（50 帧累计 100ms），且发送频率本就 ≤ 50fps，
                // CPU0 压力主要在 encoder 而非 sender，1 tick 足够给 IDLE0 喂狗。
                vTaskDelay(1);
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

/**
 * @brief 会话重连任务
 * 负责刷新accessToken并重建WebSocket连接
 * 实现指数退避重连策略，避免频繁重连导致服务器压力过大
 *
 * @param arg 重连延迟时间（毫秒）
 *
 * 调用者：on_token_refresh_timeout和PROTOCOL_EVENT_DISCONNECTED事件处理中创建
 */
static void session_reconnect_task(void *arg)
{
    int delay_ms = (int)(intptr_t)arg;

    // 指数退避延迟
    if (delay_ms > 0)
    {
        ESP_LOGI(TAG, "重连退避等待 %d 秒...", delay_ms / 1000);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    ESP_LOGI(TAG, "开始 Token 刷新 + 重连流程...");

    const char *new_token = s_access_token;

    // 极端异常兜底：请求新 Token
    if (strlen(new_token) == 0 && strlen(s_ws_token) > 0)
    {
        ESP_LOGW(TAG, "内存无可用 Token，发起 HTTP 认证...");
        auth_t *auth = auth_create();
        auth_perform(auth, s_ws_token);

        if (auth->access_token != NULL)
        {
            strncpy(s_access_token, auth->access_token, sizeof(s_access_token) - 1);
            new_token = s_access_token;
        }
        auth_destroy(auth);
    }

    // 销毁旧连接
    if (s_protocol != NULL)
    {
        ESP_LOGI(TAG, "关闭旧 WebSocket 连接（释放服务端连接计数）...");
        protocol_disconnect(s_protocol);
        protocol_destroy(s_protocol);
        s_protocol = NULL;
    }

    char *full_ws_uri = (char *)malloc_zeroed(1024);
    if (full_ws_uri == NULL)
    {
        ESP_LOGE(TAG, "full_ws_uri 分配失败，放弃重连");
        goto exit_task;
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
        goto exit_task;
    }

    protocol_register_callback(s_protocol, protocol_event_handler, NULL);
    protocol_connect(s_protocol);
    ESP_LOGI(TAG, "重连完成，WebSocket 正在建立连接... URI: %s", s_ws_uri);
    free(full_ws_uri);

exit_task:
    // ★ 核心修复 4：无论成功失败，必须清空句柄，防止系统永远以为正在重连从而陷入死锁！
    s_reconnect_handle = NULL;
    vTaskDelete(NULL);
}
// ─── 关闭会话（保持 WebSocket 连接）────────────────────────────────────────

/**
 * @brief 关闭当前会话
 * 释放会话相关的所有资源，但保持WebSocket连接以便下次快速启动
 *
 * 调用者：on_session_timeout、session_on_wake_word（打断场景）、PROTOCOL_EVENT_COMPLETE等
 */
static void session_close(void)
{
    if (s_state == SESSION_IDLE)
        return;

    ESP_LOGI(TAG, "关闭会话");
    s_state = SESSION_IDLE;       // 先置状态，sender 任务循环条件会检测到退出
    s_speech_detected = false;    // 重置语音检测状态
    s_is_continuous_turn = false; // 关闭会话后不再认为是连续对话，下一次唤醒即新会话
    s_vad_speech_count = 0;       // 重置 VAD 相关状态，确保下次唤醒后从干净状态开始
    s_vad_silence_streak = 0;     // 同步清零静音连击计数（防容忍机制残留状态污染下一轮）
    s_waiting_for_silence = false;
    s_vad_silence_count = 0;
    // s_waiting_for_silence = false;
    // s_vad_silence_count = 0;
    // 先停止 PCM Hook，防止新数据继续写入已停止的编码器
    wake_word_set_enhanced_pcm_hook(NULL);
    // 注销 AEC 参考回调：之后 feed 使用零参考，AEC 退化为纯 NS，不影响唤醒词检测
    custom_wake_word_set_aec_ref(NULL);

    // 释放发送任务的阻塞
    xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);

    xTimerStop(s_session_timer, 0);
    xTimerStop(s_eos_timer, 0);

    // ─── 音频处理器和发送任务清理逻辑 ────────────────────────────────────────
    // 说明：安全停止并销毁音频处理管道，确保所有相关任务正常退出
    // API：audio_processor_stop/destroy, vTaskDelay
    // 数据：修改 s_processor 和 s_sender_handle 全局变量
    if (s_processor != NULL)
    {
        // 步骤1: 停止音频处理器（设置内部运行标志为false，让编码任务自然退出）
        audio_processor_stop(s_processor);

        // 步骤2: 等待发送任务完全退出（最多等待1秒，每次检查间隔100ms）
        // 发送任务在检测到s_state变为SESSION_IDLE后会自动退出
        for (int i = 0; i < 10 && s_sender_handle != NULL; i++)
            vTaskDelay(pdMS_TO_TICKS(100));

        // 步骤3: 如果超时仍未退出，记录警告日志（可能任务卡在阻塞状态）
        if (s_sender_handle != NULL)
            ESP_LOGW(TAG, "sender 任务未能在超时内退出");

        // 步骤4: 销毁音频处理器实例，释放所有相关资源（缓冲区、编码器等）
        audio_processor_destroy(s_processor);
        s_processor = NULL;
    }

    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);

    wake_word_start();
    ESP_LOGI(TAG, "会话已关闭，WebSocket 保持连接，重新监听唤醒词...");
    PRINT_MEM_INFO(TAG, "对话会话结束清理后");
}

// ─── 公开 API：初始化 + 预连接 ──────────────────────────────────────────────

/**
 * @brief 会话模块初始化函数
 * 初始化会话状态机，建立WebSocket预连接，准备接收唤醒词触发
 *
 * @param ws_uri WebSocket服务器地址，如果为NULL则从NVS读取
 *
 * 调用者：application.c中的应用初始化逻辑
 */
void session_init(const char *ws_uri)
{
    s_session_eg = xEventGroupCreate();
    s_wake_word_mutex = xSemaphoreCreateMutex();

    // ── 【新增】初始化事件队列与任务 ──
    if (s_session_evt_queue == NULL)
    {
        s_session_evt_queue = xQueueCreate(5, sizeof(session_evt_t)); // 创建事件队列
    }
    // 事件处理任务：专门处理会话相关事件，避免在协议回调中直接操作会话状态导致竞态
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
        nvs_get_str(h, "ws_token", s_ws_token, &token_sz);
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
            ESP_LOGI(TAG, "[OK] accessToken 获取成功");
        }
        else
        {
            ESP_LOGW(TAG, "[WARN] accessToken 获取失败，尝试用 NVS 缓存的旧 token 连接");
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
        ESP_LOGW(TAG, "[WARN] 无 deviceToken，WebSocket 将无认证连接");
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

/**
 * @brief 唤醒词触发处理函数
 * 当检测到唤醒词时调用，启动新的会话或处理打断逻辑
 *
 * @param display 触发的唤醒词显示名称
 *
 * 调用者：wake_word模块检测到唤醒词后调用
 */
void session_on_wake_word(const char *display)
{
    PRINT_MEM_INFO(TAG, "对话会话开始");
    if (s_state == SESSION_PLAYING)
    {
        ESP_LOGW(TAG, "[WARN] 唤醒词打断 TTS: [%s]", display);
        if (s_protocol && protocol_is_connected(s_protocol))
        {
            protocol_send_abort_speaking(s_protocol);
        }
        audio_processor_flush_output(s_processor);
        s_state = SESSION_LISTENING;
        s_speech_detected = false;
        s_stop_sent = false;                    // 重置：允许本轮新语音触发 EOS
        s_is_continuous_turn = false;           // 打断后 session 仍活跃，不需要重发 Hello
        s_vad_speech_count = 0;                 // 重置防抖计数，避免打断时累积帧数误触
        s_vad_ready_tick = xTaskGetTickCount(); // ★ 必须有这行！保证打断后也有 800ms 的尾音保护
        wake_word_stop();                       // ★ 同样挂起引擎
        // s_waiting_for_silence = true;
        // s_vad_silence_count = 0;
        // wake_word_stop(); // 同样在打断后挂起唤醒引擎释放算力
        xTimerReset(s_session_timer, 0);
        return;
    }
    // ★ 核心修复 2：如果是在连麦等待期，用户却强行喊了唤醒词，主动发起新一轮对话
    if (s_state == SESSION_LISTENING && s_is_continuous_turn)
    {
        ESP_LOGI(TAG, "连麦等待期听到唤醒词，主动发起新一轮对话");
        if (s_protocol && protocol_is_connected(s_protocol))
        {
            protocol_send_hello(s_protocol); // 强行发握手包
        }
        s_is_continuous_turn = false;
        s_speech_detected = true; // 假装 VAD 已经触发，直接准备录音
        s_vad_ready_tick = xTaskGetTickCount();
        xTimerReset(s_session_timer, 0);
        return;
    }
    if (s_state != SESSION_IDLE)
        return;

    // drain 阶段（session_close 正在等待 decoder 排水）：
    // s_state 已是 IDLE 但 s_processor 尚未销毁，直接 flush dec_input
    // 让排水立即结束，session_close() 快速完成后 wake_word_start() 会重新使能唤醒。
    // 用户需再说一次唤醒词，但系统状态安全，不会双重创建 processor。
    if (s_processor != NULL)
    {
        ESP_LOGW(TAG, "会话正在收尾（drain），flush 加速退出，请再说一次唤醒词");
        audio_processor_flush_output(s_processor);
        return;
    }

    ESP_LOGI(TAG, "会话开始 [%s]", display);
    s_state = SESSION_LISTENING;
    s_speech_detected = false;
    s_stop_sent = false;
    s_vad_speech_count = 0;                 // 新会话从零开始，不带上次会话残留帧数
    s_vad_ready_tick = xTaskGetTickCount(); // 从当前时刻开始计算保护期
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

    // 注册 AEC 参考回调：play_task 向 aec_ref_buf 推副本，feed 时提供给 AFE AEC
    custom_wake_word_set_aec_ref(aec_ref_provider);

    wake_word_set_enhanced_pcm_hook(on_enhanced_pcm);

    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(ws_sender_task, "ws_sender",
                                                     4096, NULL, 5,
                                                     (TaskHandle_t *)&s_sender_handle,
                                                     0, MALLOC_CAP_SPIRAM);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "发送任务创建失败");
        wake_word_set_enhanced_pcm_hook(NULL);
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
    // ★ 新增：系统进入第一次录音状态，暂停后台唤醒词检测，防卡顿
    wake_word_stop();
    xTimerStart(s_session_timer, 0);
    return;

error:
    ESP_LOGE(TAG, "会话启动失败，回滚所有资源");
    s_state = SESSION_IDLE;
    s_speech_detected = false;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
    wake_word_start();
}

/**
 * @brief 获取当前会话状态
 * 返回当前会话模块的状态
 *
 * @return session_state_t 当前会话状态
 *
 * 调用者：应用层或其他模块需要查询会话状态时
 */
session_state_t session_get_state(void)
{
    return s_state;
}