/**
 * @file session.c
 * @brief 会话状态机 — WebSocket 预连接 + 多轮对话 + 语音打断
 *
 * 连接策略：
 * WiFi 就绪后立即建立 WebSocket 连接（含 TLS 握手），
 * 唤醒词触发时直接发送 start，无需等待连接建立。
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
 * 多轮对话流程（与"应有流程"对照）：
 *  ① 开机自动建 WS（session_init → protocol_connect）
 *  ② 唤醒词触发 → 发 start → LISTENING
 *  ③ started 到来 → SERVER_READY_BIT 置位，PCM 上行（云端做 VAD，
 *     客户端 on_enhanced_pcm 含静音过滤 VAD 闸门，仅为匹配 encoder 35fps 速率上限）
 *  ④ TTS_START → PLAYING（停 PCM 上行，唤醒词引擎仍在监听打断）
 *  ⑤ TTS_END + 扬声器播完 → 500ms 排空保护 → 切回 LISTENING
 *  ⑥ 唤醒词(PLAYING中) → flush dec+enc 缓冲 + 隐式打断（直接 send_start，不发 cancel）
 *  ⑦ 双 20s 静默 → 发 cancel + 关闭会话（保持 WS 连接）
 *     - wait_user_timer：started 后启动；首个 STT/AUDIO 时停止
 *     - wait_next_turn_timer：tts_end 切回 LISTENING 时启动；云端下行(STT/TTS)kick 刷新，
 *                             上行人声也可刷新但受硬上限约束（折中策略，防回声顶满）
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
#define DEFAULT_WS_URI "ws://122.224.191.2:4888/ws/omni"
// NVS存储命名空间，用于存储网络配置
#define NVS_NAMESPACE_NET "net_config"
// 整体会话超时时间：20秒，超过此时间无活动则关闭会话
#define SESSION_TIMEOUT_MS 20000
// wait_next_turn 折中策略上限：tts_end 切回 LISTENING 后，允许用户上行人声
// （on_enhanced_pcm 检测到能量）刷新该定时器，但累计最多撑到本上限即强制 cancel。
// 目的：兼顾「用户说话续命」与「防 TTS 回声/底噪把定时器顶满导致永不超时」（旧坑）。
// 一旦从切回 LISTENING 起经过 SESSION_NEXT_TURN_HARD_LIMIT_MS 仍无云端下行业务事件，
// 即便上行人声持续刷新也强制结束会话。
#define SESSION_NEXT_TURN_HARD_LIMIT_MS 40000
// Token主动刷新时间：110分钟（accessToken过期时间为2小时，提前10分钟刷新）
#define TOKEN_REFRESH_MS (110 * 60 * 1000)
// OPUS音频帧发送缓冲区大小
#define OPUS_SEND_BUF 512

// ─── 事件组位定义 ────────────────────────────────────────────────────────────
// 会话事件组句柄，用于任务间同步
static EventGroupHandle_t s_session_eg = NULL;
// 位定义：start握手完成，可以发送音频数据
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
    SESSION_EVT_ABORT,     // 异步发送 CANCEL 信号（在 session_event_task 里执行 WebSocket 发送，避免 Tmr Svc 栈溢出）
    SESSION_EVT_CLOSE,     // 异步关闭会话信号
    SESSION_EVT_RECONNECT, // 异步重连信号：在常驻 session_event_task 里执行完整重连，
                           // 不在 WS 回调上下文创建临时任务，也不绑核，根治"重连任务不被调度"卡死
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
// 保护 s_protocol 句柄本身的 create/destroy/stop，杜绝 session_close 与
// session_reconnect_task 并发销毁同一句柄导致的卡死/野指针/泄漏。
// 注意：只保护"换句柄"这类生命周期操作；高频的 send/is_connected 不加锁
//       （它们由 esp_websocket_client 内部自带锁保证线程安全）。
static SemaphoreHandle_t s_protocol_mutex = NULL;

// ─── 流程 #7 拆分的两个 20s 静默超时定时器 ────────────────────────────────────
// wait_user_timer：started 后启动 → 等用户开始说话；收到首个 STT/AUDIO 时停止
// wait_next_turn_timer：tts_end 后切回 LISTENING 时启动 → 等用户开启下一轮；
//                       云端下行(STT/TTS)kick 刷新；上行人声 kick_by_voice 刷新但受
//                       SESSION_NEXT_TURN_HARD_LIMIT_MS 硬上限约束（折中，防回声顶满）
// 任一定时器到期 → 发 cancel + close（与旧 s_session_timer 行为一致）
static TimerHandle_t s_wait_user_timer = NULL;
static TimerHandle_t s_wait_next_turn_timer = NULL;
// 标记 wait_user_timer 已被首个 STT/AUDIO 停止：用于让后续重复信号不再 stop
static volatile bool s_wait_user_armed = false;
// 标记 wait_next_turn_timer 已启动：on_enhanced_pcm 检测到上行人声时 kick 刷新（折中策略）
static volatile bool s_wait_next_turn_armed = false;
// wait_next_turn 折中策略：记录切回 LISTENING 的 arm 起始 tick，作为 SESSION_NEXT_TURN_HARD_LIMIT_MS 硬上限基准。
// 上行人声只在「距 arm 起点未超硬上限」时才允许刷新定时器，超过后不再续命，让其到点 cancel。
static volatile TickType_t s_wait_next_turn_start_tick = 0;
// accessToken主动刷新定时器句柄
static TimerHandle_t s_token_refresh_timer = NULL;
// 标记是否为连续会话模式
static bool s_is_continuous_turn = false;
// 扬声器排空保护期标志：TTS播完后屏蔽PCM写入，防止尾音上传
static bool s_waiting_for_silence = false;
// 排空保护期计时起点
static TickType_t s_wait_silence_start = 0;
// 延后停止唤醒词标志：TTS 播完后立即调 wake_word_stop() 会被 MultiNet 持有的
// buffer_mutex 阻塞数秒（portMAX_DELAY），导致 ws_sender_task 卡在 702 行，
// 排空保护期倒计时检查 (706 行) 跑不到，500ms 实际变 8 秒+。
// 改为：进保护期时只置位本标志，保护期 500ms 到期后再调 wake_word_stop()，
// 此时 wait_ms 已正确计完，即使再卡也不影响保护期长度。
static bool s_pending_wake_word_stop = false;
// WebSocket服务器URI地址
static char s_ws_uri[128] = DEFAULT_WS_URI;
// deviceToken（App绑定时下发的长期凭证）
static char s_ws_token[256] = {0};
// accessToken（通过device-login换取的短效令牌）
static char s_access_token[512] = {0};
// TTS云端数据接收完毕标志，等待扬声器实际播放完成后再切换监听状态
static volatile bool s_tts_data_done = false;
// WebSocket发送任务句柄
static volatile TaskHandle_t s_sender_handle = NULL;
// WebSocket重连任务句柄，防止重复创建（保留兼容，重连已改为常驻任务事件驱动，不再用作守卫）
static volatile TaskHandle_t s_reconnect_handle = NULL;
// 重连进行中守卫：重连主体已搬入常驻 session_event_task，靠此标志防重入
static volatile bool s_reconnecting = false;
// 连续重连次数，用于指数退避算法
static int s_reconnect_attempts = 0;
// 重连中 auth 重试计数器：服务器不可达时避免反复创建 HTTP 客户端浪费内部 SRAM
static int s_auth_retry_in_reconnect = 0;
// 基础退避延迟时间：2秒，指数退避上限为 60s（无最大次数限制，服务端重启场景需要无限重试）
#define RECONNECT_BASE_DELAY_MS 2000
// 唤醒词操作互斥锁，防止多线程操作冲突
static SemaphoreHandle_t s_wake_word_mutex = NULL;
// 当前触发的唤醒词字符串
static char s_current_wake_word[64] = {0};
static volatile bool s_reset_frame_counter = false; // session_event_task(CPU0) 与 ws_sender_task(CPU1) 共享，需 volatile
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
            case SESSION_EVT_ABORT:
                if (s_protocol && protocol_is_connected(s_protocol))
                {
                    protocol_send_abort_speaking(s_protocol);
                    ESP_LOGI(TAG, "已发送CANCEL消息结束会话");
                }
                break;
            case SESSION_EVT_CLOSE: // 【在这里安全地执行关闭】
                ESP_LOGI(TAG, "接收到异步关闭信号，安全关闭会话...");
                session_close();
                break;
            case SESSION_EVT_RECONNECT:
                // 在常驻任务（普通上下文、不绑核）里派生重连任务，规避两个坑：
                //   ① 不在 WS 回调上下文创建（旧实现的卡死根源之一）
                //   ② 重连任务用 tskNO_AFFINITY，不再绑 CPU1，避免被唤醒词推理饿死不调度
                // 重连任务栈必须用 INTERNAL：内部 auth_perform→nvs_set_str 写 Flash 关 Cache，
                //   SPIRAM 栈（本任务自身就是 SPIRAM 栈）此时不可访问，故重连主体必须独立 INTERNAL 栈。
                if (s_reconnecting || s_reconnect_handle != NULL)
                {
                    ESP_LOGW(TAG, "重连已在进行中，忽略重复 RECONNECT 事件");
                    break;
                }
                {
                    // 退避延迟在此自算：2s→4s→8s→16s→32s→60s（上限），无最大次数限制
                    s_reconnect_attempts++;
                    int shift = s_reconnect_attempts - 1;
                    if (shift > 5)
                        shift = 5;
                    int delay_ms = RECONNECT_BASE_DELAY_MS * (1 << shift);
                    if (delay_ms > 60000)
                        delay_ms = 60000;
                    ESP_LOGW(TAG, "第 %d 次重连，%d 秒后执行...", s_reconnect_attempts, delay_ms / 1000);
                    BaseType_t r = xTaskCreatePinnedToCoreWithCaps(
                        session_reconnect_task, "ws_reconn",
                        6144, (void *)(intptr_t)delay_ms, 4,
                        (TaskHandle_t *)&s_reconnect_handle,
                        tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                    if (r != pdPASS)
                    {
                        ESP_LOGE(TAG, "[MEM] 重连任务创建失败，回退计数，下次断开重试");
                        s_reconnect_handle = NULL;
                        s_reconnect_attempts--;
                    }
                }
                break;
            default:
                break;
            }
        }
    }
}

// ─── 定时器回调 ──────────────────────────────────────────────────────────────

/**
 * @brief 流程 #7 - 等用户开口超时回调（started 后 20s 仍无云端任何业务事件）
 * 触发条件：started 已到，但 20s 内未收到首个 STT 文本或 AUDIO 二进制帧
 * 行为：发送 cancel + 关闭会话
 */
static void on_wait_user_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "[流程#7-A] started 后 %d 秒云端无业务事件，发送 cancel 结束会话",
             SESSION_TIMEOUT_MS / 1000);
    s_wait_user_armed = false;
    if (s_session_evt_queue != NULL)
    {
        session_evt_t evt = SESSION_EVT_ABORT;
        xQueueSend(s_session_evt_queue, &evt, 0);
        evt = SESSION_EVT_CLOSE;
        xQueueSend(s_session_evt_queue, &evt, 0);
    }
}

/**
 * @brief 流程 #7 - 等下一轮开口超时回调（tts_end 切回 LISTENING 后 20s 无刷新）
 * 触发条件：切回 LISTENING 后连续 20s 既无云端下行(STT/TTS)、也无上行人声刷新；
 *           或上行人声虽持续但已达 SESSION_NEXT_TURN_HARD_LIMIT_MS 硬上限不再续命
 * 行为：发送 cancel + 关闭会话
 */
static void on_wait_next_turn_timeout(TimerHandle_t t)
{
    ESP_LOGW(TAG, "[流程#7-B] tts_end 后 %d 秒用户无新输入，发送 cancel 结束会话",
             SESSION_TIMEOUT_MS / 1000);
    s_wait_next_turn_armed = false;
    if (s_session_evt_queue != NULL)
    {
        session_evt_t evt = SESSION_EVT_ABORT;
        xQueueSend(s_session_evt_queue, &evt, 0);
        evt = SESSION_EVT_CLOSE;
        xQueueSend(s_session_evt_queue, &evt, 0);
    }
}

/**
 * @brief 启动 wait_user 定时器（在 PROTOCOL_EVENT_start 中调用）
 */
static inline void wait_user_timer_start(void)
{
    if (s_wait_user_timer == NULL)
        return;
    s_wait_user_armed = true;
    xTimerChangePeriod(s_wait_user_timer, pdMS_TO_TICKS(SESSION_TIMEOUT_MS), 0);
}

/**
 * @brief 停止 wait_user 定时器（首个 STT/AUDIO 到达时调用，幂等）
 */
static inline void wait_user_timer_stop(void)
{
    if (s_wait_user_timer == NULL || !s_wait_user_armed)
        return;
    s_wait_user_armed = false;
    xTimerStop(s_wait_user_timer, 0);
}

/**
 * @brief 启动 wait_next_turn 定时器（在切回 LISTENING 时调用）
 * 同时记录 arm 起始 tick，作为 SESSION_NEXT_TURN_HARD_LIMIT_MS 硬上限的基准时间。
 */
static inline void wait_next_turn_timer_start(void)
{
    if (s_wait_next_turn_timer == NULL)
        return;
    s_wait_next_turn_armed = true;
    s_wait_next_turn_start_tick = xTaskGetTickCount(); // 记录折中策略硬上限基准
    xTimerChangePeriod(s_wait_next_turn_timer, pdMS_TO_TICKS(SESSION_TIMEOUT_MS), 0);
}

/**
 * @brief 停止 wait_next_turn 定时器（on_enhanced_pcm 检测到有效语音上传时调用，幂等）
 */
static inline void wait_next_turn_timer_stop(void)
{
    if (s_wait_next_turn_timer == NULL || !s_wait_next_turn_armed)
        return;
    s_wait_next_turn_armed = false;
    xTimerStop(s_wait_next_turn_timer, 0);
}

/**
 * @brief 重置 wait_next_turn 定时器为新一轮 20s（收到云端大模型下行时调用，幂等）
 * 与 stop 的区别：stop 是停摆（永不超时），kick 是把 20s 倒计时重新拉满。
 * 用途：只要云端还在持续回复（STT/TTS），就刷新超时；一旦连续 20s 收不到
 *       任何云端下行，定时器照常到点 ABORT+CLOSE。
 * 注意：本函数为「云端下行」专用，不受硬上限约束（云端在回话说明会话健康）。
 */
static inline void wait_next_turn_timer_kick(void)
{
    if (s_wait_next_turn_timer == NULL || !s_wait_next_turn_armed)
        return;
    xTimerReset(s_wait_next_turn_timer, 0);
    s_wait_next_turn_start_tick = xTaskGetTickCount(); // 云端下行视为新一轮，重置硬上限基准
}

/**
 * @brief 折中策略：上行人声刷新 wait_next_turn 定时器（on_enhanced_pcm 检测到能量时调用）
 * 与 wait_next_turn_timer_kick 的区别：本函数受 SESSION_NEXT_TURN_HARD_LIMIT_MS 硬上限约束。
 * 行为：
 *   - 距 arm 起点未超硬上限 → 把 20s 倒计时重新拉满（用户说话续命）。
 *   - 已超硬上限 → 不刷新，让定时器到点 cancel（防 TTS 回声/底噪把定时器顶到永不超时）。
 * 幂等且无锁：仅在 armed 时生效，被高频 on_enhanced_pcm 调用，开销极低。
 */
static inline void wait_next_turn_timer_kick_by_voice(void)
{
    if (s_wait_next_turn_timer == NULL || !s_wait_next_turn_armed)
        return;
    TickType_t elapsed = xTaskGetTickCount() - s_wait_next_turn_start_tick;
    if (elapsed >= pdMS_TO_TICKS(SESSION_NEXT_TURN_HARD_LIMIT_MS))
        return; // 已达硬上限，不再续命，让定时器到点结束会话
    xTimerReset(s_wait_next_turn_timer, 0);
}

/**
 * @brief 停止两个定时器（会话关闭 / 状态切换时统一调用）
 */
static inline void session_timers_stop_all(void)
{
    wait_user_timer_stop();
    wait_next_turn_timer_stop();
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
    // 走与断线重连同一条常驻任务路径：丢 RECONNECT 事件，由 session_event_task 执行
    // 完整 Auth 刷新 + 重建连接（s_reconnecting 守卫防与正在进行的重连重入）。
    if (!s_reconnecting && s_session_evt_queue != NULL)
    {
        session_evt_t evt = SESSION_EVT_RECONNECT;
        xQueueSend(s_session_evt_queue, &evt, 0);
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

    // 要求④：PLAYING时云端TTS正在下发，停止向enc_input写PCM，防回声/自激
    if (s_state != SESSION_LISTENING)
        return;

    // 要求②：只有收到云端started（SERVER_READY_BIT置位）后才开始写PCM
    if (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
        return;

    // 排空保护期：TTS播完后固定500ms，让I2S DMA尾帧和扬声器余振彻底消退，防止尾音上传
    // 倒计时由ws_sender_task在检测到audio_processor_is_playing()==false后负责清除
    if (s_waiting_for_silence)
        return;

    // ── 客户端 VAD 闸门（精简版）────────────────────────────────────────────
    // 职责：仅决定"这一帧静音段要不要喂给 encoder"，不做 EOS、不发协议命令
    //       云端继续负责 VAD/EOS（vad_state 与 stop_listening 仍在云端）
    // 必要性：encoder OPUS 编码物理速度 ~35 fps，AFE 生产 50 fps，
    //         若全量上传 enc_input 9 秒填满丢 PCM → ASR 拿到残缺音频识别失败。
    //         过滤静音段后 enc_input 实际生产 < 35 fps 消费速率，永不溢出。
    // 阈值：复用历史调试值 200000（commit 29653be 经验值，安静办公环境足够灵敏）
    // hangover：能量低于阈值后再续传 10 帧（200ms）防词尾被切
    //           人说话词与词间存在 50~100ms 自然短停顿，hangover 必须 > 100ms
#define VAD_ENERGY_THRESHOLD 200000 // 每采样点平方平均能量阈值
#define VAD_HANGOVER_FRAMES 15      // 静音段尾部仍上传 50 帧（@20ms/帧=1000ms）
                                    // 覆盖换气/思考停顿（300~600ms），防云端误切句
    static int silence_streak = 0;

    int64_t energy_sum = 0;
    for (size_t i = 0; i < samples; i++)
        energy_sum += (int64_t)data[i] * data[i];
    int64_t energy_avg = samples > 0 ? (energy_sum / (int64_t)samples) : 0;

    if (energy_avg >= VAD_ENERGY_THRESHOLD)
    {
        silence_streak = 0; // 检测到人声，清零静音计数
        // 折中策略：检测到上行人声 → 刷新 wait_next_turn 定时器（让用户说话续命）。
        //   但受 SESSION_NEXT_TURN_HARD_LIMIT_MS 硬上限约束：距切回 LISTENING 起
        //   超过硬上限后不再刷新，避免被 TTS 回声/底噪能量顶满导致永不超时（旧坑）。
        //   云端下行事件（STT/TTS_START）走 wait_next_turn_timer_kick，不受硬上限约束。
        wait_next_turn_timer_kick_by_voice();
    }
    else
        silence_streak++; // 静音帧累计

    if (silence_streak > VAD_HANGOVER_FRAMES)
        return; // 持续静音 > 200ms：丢帧（不上传到 encoder）

    audio_processor_write_pcm(s_processor, (void *)data, samples * sizeof(int16_t));
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
    // ─── IDLE 闸门：忽略会话关闭后云端在途的迟到业务事件 ───────────────────────
    // 会话因超时/打断/错误被 session_close() 关闭后，s_state 已置 IDLE、s_processor 已销毁，
    // 但云端在 cancel 到达前已发出的 transcript/tts_start 等"在途"事件仍会陆续到达设备。
    // 这些"鬼魂消息"绝不能改写 s_state，更不能触碰已销毁的 s_processor，
    // 否则会把已关闭会话错误复活成 PLAYING：紧接着的服务端 FIN 断链会被误判为
    // "会话中网络断开" → 抢发 CLOSE + 创建重连任务，与 close 流程、WS 回调线程
    // 三方无锁并发销毁同一 s_protocol → 重连卡死 + mbedTLS/rx_buf 内存泄漏。
    // 仅放行连接生命周期事件(CONNECTED/DISCONNECTED/ERROR)，其余业务事件直接忽略。
    if (s_state == SESSION_IDLE &&
        event_id != PROTOCOL_EVENT_CONNECTED &&
        event_id != PROTOCOL_EVENT_DISCONNECTED &&
        event_id != PROTOCOL_EVENT_ERROR)
    {
        ESP_LOGW(TAG, "会话已关闭，忽略迟到的云端事件 id=%d（在途数据）", (int)event_id);
        return;
    }

    switch (event_id)
    {
    // ── WebSocket 底层连接成功（启动时预连接，或断线重连后）──────────────
    /**
     * @brief WebSocket连接成功事件处理
     *
     * 说明：处理WebSocket底层连接成功的事件，重置重连计数并设置连接状态位。
     *       如果有会话正在等待连接，则立即发送start握手消息。
     * API：xEventGroupSetBits
     * 数据：修改s_reconnect_attempts和s_session_eg
     */
    case PROTOCOL_EVENT_CONNECTED:
        ESP_LOGI(TAG, "WebSocket 已连接（预连接就绪）");
        s_reconnect_attempts = 0;      // 连接成功，重置退避计数
        s_auth_retry_in_reconnect = 0; // 连接成功，重置 auth 重试计数
        xEventGroupSetBits(s_session_eg, SESSION_WS_CONNECTED_BIT);
        // 如果有会话正在等待连接（极少数情况：唤醒时恰好断线重连中）
        if (s_state == SESSION_LISTENING)
        {
            ESP_LOGI(TAG, "会话等待中，立即发送 start");
            protocol_send_start(s_protocol);
        }
        break;

    // ── 收到服务器 start 响应 → 握手完成 ────────────────────────────────
    /**
     * @brief 服务器start响应事件处理
     *
     * 说明：处理收到服务器start响应的事件，表示握手完成，会话已建立。
     *       重置VAD状态，设置服务器就绪标志位，并启动会话超时定时器。
     * API：xEventGroupSetBits, xTimerReset
     * 数据：修改s_vad_ready_tick, s_speech_detected, s_stop_sent和s_session_eg
     */
    case PROTOCOL_EVENT_start:
        if (s_state == SESSION_IDLE)
            break; // 无活跃会话，忽略
        ESP_LOGI(TAG, "收到服务器 started 响应，会话已建立");

        // 打断场景：cancel 后 TTS_STOP/COMPLETE 会把 s_is_continuous_turn 置为 true，
        // 但此时我们已经发了新的 start，started 到来意味着新 session 已建立，
        // 必须清除该标志，防止 VAD 再触发一次多余的 start（双重握手）
        s_is_continuous_turn = false;

        xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);
        // 流程 #7-A：started 后启动 wait_user 定时器，等待云端首个业务事件（STT/AUDIO）
        wait_user_timer_start();
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
        s_tts_data_done = false;   // 新一轮 TTS 开始，重置播放完成标志
        s_state = SESSION_PLAYING; // 暂停麦克风上传，防止回声/自激
        // TTS_START 隐含云端已开始干活，确保 wait_user 已停止（兜底，正常 STT/AUDIO 已先停）
        wait_user_timer_stop();
        // 新一轮 TTS 开始，解除打断时置位的静音标志，允许 play_task 正常写 I2S
        audio_processor_unmute_output(s_processor);
        // 重启唤醒词引擎，TTS 期间可以检测打断唤醒词
        // 当前阈值 0.18f：与 LISTENING 一致；如出现 AEC 残留误触可上调至 0.40~0.55
        wake_word_set_det_threshold(0.3f);
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
    case PROTOCOL_EVENT_TTS_END:
        // 云端 opus 数据已全部下发，但扬声器可能还在播放缓冲中的数据。
        // 不在此处切换状态，由 ws_sender_task 轮询 audio_processor_is_playing() 为 false 后再开启监听。
        // wait_next_turn 定时器也由 ws_sender_task 在切回 LISTENING 时启动，这里不动。
        ESP_LOGI(TAG, "云端 TTS 数据下发完毕，等待扬声器播放完成后开启监听");
        s_tts_data_done = true;
        break;

    case PROTOCOL_EVENT_STT:
        ESP_LOGI(TAG, "[STT] 语音转文字(STT): %s", (char *)event_data);
        // 流程 #7-A：首个 STT 到达 → 云端开始干活，停止 wait_user 定时器
        wait_user_timer_stop();
        // 流程 #7-B：收到云端大模型下行 → 刷新下一轮超时（20s 无任何下行才关）
        wait_next_turn_timer_kick();
        break;

    case PROTOCOL_EVENT_LLM:
        ESP_LOGI(TAG, "[LLM] 大模型状态: %s", (char *)event_data);
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
            // 流程 #7-A：首个 AUDIO 帧到达 → 云端 TTS 流开始，停止 wait_user 定时器
            wait_user_timer_stop();
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
        if (xTaskGetTickCount() - s_last_disconnect_tick < pdMS_TO_TICKS(200))
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

        // ★ 根治卡死：不再在 WS 回调上下文创建临时任务（曾因绑核 CPU1 调度不到而卡死），
        //   只往常驻 session_event_task 的队列丢一个 RECONNECT 事件，由它串行执行完整重连。
        //   重入由 s_reconnecting 守卫（在 RECONNECT 分支内判断），此处无脑丢即可。
        if (s_session_evt_queue != NULL)
        {
            session_evt_t evt = SESSION_EVT_RECONNECT;
            xQueueSend(s_session_evt_queue, &evt, 0);
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
    // 多轮模式下 COMPLETE 仅记录日志：状态切换由 TTS_END + 扬声器排空联合驱动，
    // 静默超时由 wait_next_turn_timer 接管
    case PROTOCOL_EVENT_COMPLETE:
        ESP_LOGI(TAG, "收到 Complete");
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

    // 1. 开局等待阶段：等待服务器 started 响应，最多等待 8s
    //    enc_output（8KB ≈ 2.7s OPUS）足以缓冲 start/started RTT 期间的帧，
    //    服务器就绪后阶段2直接发送，唤醒词后立即说话的语音不再丢失。
    //    超时兜底：8s 未收到 started（网络不稳或服务端无响应），主动关闭避免任务挂死到30s超时
    {
        int wait_count = 0;
        const int max_wait = 8000 / 20; // 8s / 20ms per tick
        while (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
        {
            if (s_state != SESSION_LISTENING && s_state != SESSION_PLAYING)
                goto exit;
            if (++wait_count > max_wait)
            {
                ESP_LOGW(TAG, "等待服务器 started 超时（8s），主动关闭会话");
                if (s_session_evt_queue != NULL)
                {
                    session_evt_t evt = SESSION_EVT_CLOSE;
                    xQueueSend(s_session_evt_queue, &evt, 0);
                }
                goto exit;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }
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

        // 云端TTS数据已全部接收，且dec_output缓冲audio_processor_start已排空（扬声器最后一批PCM已交给I2S）
        // 切换到LISTENING，开启500ms固定排空保护期，等待I2S DMA尾帧和扬声器余振消退
        if (s_tts_data_done && s_state == SESSION_PLAYING &&
            s_processor != NULL && !audio_processor_is_playing(s_processor))
        {
            ESP_LOGI(TAG, "TTS 播放完成，切换到监听状态，开始500ms排空保护期");
            s_tts_data_done = false;
            s_state = SESSION_LISTENING;
            s_is_continuous_turn = false;
            s_reset_frame_counter = true;
            s_waiting_for_silence = true;
            s_wait_silence_start = xTaskGetTickCount();
            // 清除 SERVER_READY_BIT：阻止 ws_sender_task 在服务端发 COMPLETE 前提前发音频。
            // 旧代码靠 wake_word_stop 意外阻塞 12s 充当等待缓冲区，现在显式等待 COMPLETE。
            // COMPLETE 事件处理器（SESSION_LISTENING 分支）会重新 set 该 bit。
            // xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
            // 流程 #7-B：切回 LISTENING 启动 wait_next_turn 定时器，
            // 由 on_enhanced_pcm 在检测到首个有效语音帧时停止
            wait_next_turn_timer_start();
            wake_word_set_det_threshold(0.18f); // 切回 LISTENING 标准阈值
            // ⚠️ 不能在此处同步调 wake_word_stop()：会被 MultiNet 持有的 buffer_mutex
            //    阻塞数秒（portMAX_DELAY），导致下方 706 行倒计时检查跑不到。
            //    改为置位标志，由保护期解除分支异步调用，wait_ms 才能准确反映 500ms。
            s_pending_wake_word_stop = true;
        }

        // 排空保护期倒计时：500ms后解除PCM写入阻塞，恢复正常录音上行
        if (s_waiting_for_silence)
        {
            uint32_t wait_ms = (xTaskGetTickCount() - s_wait_silence_start) * portTICK_PERIOD_MS;
            if (wait_ms >= 500)
            {
                s_waiting_for_silence = false;
                ESP_LOGI(TAG, "排空保护期结束（%d ms），恢复PCM写入，继续监听", (int)wait_ms);
                // 保护期已解除，此时再调 wake_word_stop() 即使被锁阻塞，
                // 也不会影响 wait_ms 计数（已经准确打印过）；
                // 副作用是 wake_word_stop() 后到下次 wake_word_start() 之间
                // MultiNet 推理仍占 CPU1 一会儿，可接受。
                //! 已修改，在sseion中调用wake_word_stop()：1030
                if (s_pending_wake_word_stop) // 如果在排空保护期结束时，唤醒词引擎仍未停止（正常情况是被 TTS_END 置位后就停止了）。
                {
                    s_pending_wake_word_stop = false;
                    wake_word_stop();
                }
            }
        }

        if (!(xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT))
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }
        if (s_state == SESSION_IDLE)
            break;
        // 核心原则：不管网络状态如何，必须先把编码器的数据读出来（抽水），防止内存爆掉
        size_t len = audio_processor_read_timeout(s_processor, buf, sizeof(buf), 100);

        if (len > 0)
        {
            // 【核心阀门机制】
            // 双重条件：① LISTENING 状态（PLAYING 时只排水不发，防止与 TTS 下行数据
            //           争抢 WS 内部互斥锁导致 "Could not lock ws-client" 超时）
            //           ② 拥有 SERVER_READY 权限且连接正常
            if (s_state == SESSION_LISTENING &&
                (xEventGroupGetBits(s_session_eg) & SESSION_SERVER_READY_BIT) &&
                s_protocol && protocol_is_connected(s_protocol))
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
            // 💡 PLAYING 状态或 SERVER_READY 未就绪：数据静默丢弃，绝不发给服务器。
        }
    }

    ESP_LOGI(TAG, "发送任务结束，共发送 %d 帧 OPUS", sent_frames);

exit:
    s_sender_handle = NULL;
    // ★ 本任务由 xTaskCreatePinnedToCoreWithCaps 创建（栈在 SPIRAM 单独 heap_caps 分配），
    //   自删必须用 vTaskDeleteWithCaps，否则栈内存不会被 idle 回收 → 每轮会话泄漏 ~4KB SPIRAM。
    vTaskDeleteWithCaps(NULL);
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

    // 重入守卫：与 s_reconnect_handle 一起防止重复重连
    s_reconnecting = true;

    // 首次进入的指数退避延迟
    if (delay_ms > 0)
    {
        ESP_LOGI(TAG, "重连退避等待 %d 秒...", delay_ms / 1000);
        vTaskDelay(pdMS_TO_TICKS(delay_ms));
    }

    // 每轮失败后的重试间隔：5s 起，指数增长，最大 60s
    int retry_interval_ms = 5000;

    // 无限重试循环：直到 WebSocket 真正连上才退出
    while (true)
    {
        // ── 内部 SRAM 检查：不足时等待释放，不放弃重连 ──────────────────
        size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (internal_free < 8192)
        {
            ESP_LOGW(TAG, "[MEM] 内部 SRAM 仅剩 %d B，等 3s 释放后再试...", (int)internal_free);
            vTaskDelay(pdMS_TO_TICKS(3000));
            continue;
        }

        // ── Auth HTTP 探测：刷新 accessToken（同时检测服务器是否在线）────
        ESP_LOGI(TAG, "探测服务器（Auth HTTP）...");
        auth_t *probe_auth = auth_create();
        auth_perform(probe_auth, s_ws_token);
        bool auth_ok = (probe_auth->access_token != NULL);
        if (auth_ok)
            strncpy(s_access_token, probe_auth->access_token, sizeof(s_access_token) - 1);
        auth_destroy(probe_auth);

        if (!auth_ok)
        {
            ESP_LOGW(TAG, "Auth 失败（服务器未响应），%d 秒后重试...", retry_interval_ms / 1000);
            vTaskDelay(pdMS_TO_TICKS(retry_interval_ms));
            if (retry_interval_ms < 15000)
                retry_interval_ms = retry_interval_ms * 2 > 60000 ? 60000 : retry_interval_ms * 2;
            continue;
        }

        // ── 销毁旧 protocol，重建 WebSocket 连接 ─────────────────────────
        // 每次都完整 destroy+create，释放 mbedTLS 上下文防止内部 SRAM 泄漏。
        // ★ 加锁串行化"换句柄"临界区：杜绝与 session_close 等并发销毁同一 s_protocol，
        //   也避免他人在 destroy 与 create 之间用到 NULL/野指针。
        //   注意：锁只覆盖句柄切换，不覆盖下方 8s 等待（避免长时间占锁）。
        ESP_LOGI(TAG, "Auth 成功，尝试建立 WebSocket 连接...");
        const char *new_token = s_access_token;
        char *full_ws_uri = (char *)malloc_zeroed(1024);
        if (full_ws_uri == NULL)
        {
            ESP_LOGE(TAG, "full_ws_uri 分配失败，5s 后重试");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }
        if (strlen(new_token) > 0)
            snprintf(full_ws_uri, 1024, "%s?token=%s", s_ws_uri, new_token);
        else
            strncpy(full_ws_uri, s_ws_uri, 1024 - 1);

        if (s_protocol_mutex != NULL)
            xSemaphoreTake(s_protocol_mutex, portMAX_DELAY);
        if (s_protocol != NULL)
        {
            // 带 3s 超时的断开：避免 stop 在 FIN/TLS 半关闭态无限阻塞重连任务
            protocol_disconnect_timeout(s_protocol, 3000);
            protocol_destroy(s_protocol);
            s_protocol = NULL;
        }
        s_protocol = protocol_create(full_ws_uri, new_token);
        if (s_protocol != NULL)
            protocol_register_callback(s_protocol, protocol_event_handler, NULL);
        if (s_protocol_mutex != NULL)
            xSemaphoreGive(s_protocol_mutex);
        free(full_ws_uri);

        if (s_protocol == NULL)
        {
            ESP_LOGE(TAG, "protocol_create 失败（内存不足），5s 后重试");
            vTaskDelay(pdMS_TO_TICKS(5000));
            continue;
        }

        protocol_connect(s_protocol);
        ESP_LOGI(TAG, "WebSocket 连接请求已发出，等待结果（最多 8s）...");

        // ── 等待 CONNECTED 事件：通过 SESSION_WS_CONNECTED_BIT 判断 ──────
        EventBits_t bits = xEventGroupWaitBits(
            s_session_eg,
            SESSION_WS_CONNECTED_BIT,
            pdFALSE, // 不自动清位（由 DISCONNECTED 处理清）
            pdFALSE, // 只等任意一位
            pdMS_TO_TICKS(8000));

        if (bits & SESSION_WS_CONNECTED_BIT)
        {
            ESP_LOGI(TAG, "WebSocket 重连成功！");
            PRINT_MEM_INFO(TAG, "WS 重连成功");
            break; // 真正连上，退出重试循环
        }

        // 8s 超时仍未连上（WS 服务可能还没起来），销毁本次 protocol，下轮重试
        ESP_LOGW(TAG, "WS 连接超时（8s），%d 秒后重试...", retry_interval_ms / 1000);
        if (s_protocol_mutex != NULL)
            xSemaphoreTake(s_protocol_mutex, portMAX_DELAY);
        if (s_protocol != NULL)
        {
            protocol_disconnect_timeout(s_protocol, 3000);
            protocol_destroy(s_protocol);
            s_protocol = NULL;
        }
        if (s_protocol_mutex != NULL)
            xSemaphoreGive(s_protocol_mutex);
        vTaskDelay(pdMS_TO_TICKS(retry_interval_ms));
        if (retry_interval_ms < 60000)
            retry_interval_ms = retry_interval_ms * 2 > 60000 ? 60000 : retry_interval_ms * 2;
    }

    // 连接成功后清理状态，下次断开从 2s 退避重新开始
    s_reconnect_attempts = 0;
    s_auth_retry_in_reconnect = 0;
    s_reconnect_handle = NULL;
    s_reconnecting = false; // 重连完成，释放重入守卫
    // ★ 本任务由 xTaskCreatePinnedToCoreWithCaps 创建（6KB INTERNAL 栈，单独 heap_caps 分配），
    //   自删必须用 vTaskDeleteWithCaps，否则栈内存不会被 idle 回收 → 每轮重连泄漏 ~6KB 内部 SRAM。
    vTaskDeleteWithCaps(NULL);
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
    s_state = SESSION_IDLE; // 先置状态，sender 任务循环条件会检测到退出
    s_tts_data_done = false;
    s_is_continuous_turn = false;
    s_waiting_for_silence = false;
    s_pending_wake_word_stop = false; // 会话关闭：复位延后停标志，避免下次会话误调
    // 先停止 PCM Hook，防止新数据继续写入已停止的编码器
    wake_word_set_enhanced_pcm_hook(NULL);
    // 注销 AEC 参考回调：之后 feed 使用零参考，AEC 退化为纯 NS，不影响唤醒词检测
    custom_wake_word_set_aec_ref(NULL);

    // 释放发送任务的阻塞
    xEventGroupSetBits(s_session_eg, SESSION_SERVER_READY_BIT);

    session_timers_stop_all();

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

        // while (s_sender_handle != NULL)
        //     vTaskDelay(pdMS_TO_TICKS(50));

        // // ★ sender 已安全退出，现在可以安全调 wake_word_stop()
        // wake_word_stop();

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
    s_protocol_mutex = xSemaphoreCreateMutex(); // 串行化 s_protocol 句柄的销毁/重建

    // ── 【新增】初始化事件队列与任务 ──
    if (s_session_evt_queue == NULL)
    {
        s_session_evt_queue = xQueueCreate(10, sizeof(session_evt_t)); // 队列深度 10，防 CLOSE+VAD_STOP+CLOSE 连发丢事件
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

    // 创建定时器（流程 #7 拆分为两个独立的 20s 静默超时定时器）
    s_wait_user_timer = xTimerCreate("wait_user",
                                     pdMS_TO_TICKS(SESSION_TIMEOUT_MS),
                                     pdFALSE, NULL, on_wait_user_timeout);
    s_wait_next_turn_timer = xTimerCreate("wait_next",
                                          pdMS_TO_TICKS(SESSION_TIMEOUT_MS),
                                          pdFALSE, NULL, on_wait_next_turn_timeout);
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

    // 始终创建 protocol 对象（但不一定连接），避免 s_protocol 为 NULL 导致后续崩溃
    s_protocol = protocol_create(full_ws_uri, ws_bearer_token);
    protocol_register_callback(s_protocol, protocol_event_handler, NULL);

    // ★ P1 优化：Auth 失败且服务器不可达时，跳过 WebSocket 连接，避免白白消耗内部 SRAM
    if (!auth_is_server_reachable())
    {
        ESP_LOGW(TAG, "[WARN] 服务器不可达，跳过 WebSocket 连接，触发后台重连任务...");
        // 投递 RECONNECT 事件：由 session_event_task 派生 session_reconnect_task，
        // 以指数退避（2s→4s→…→60s）无限重试 Auth + WebSocket 连接，不阻塞主线程。
        if (s_session_evt_queue != NULL)
        {
            session_evt_t evt = SESSION_EVT_RECONNECT;
            xQueueSend(s_session_evt_queue, &evt, 0);
        }
    }
    else
    {
        protocol_connect(s_protocol);
    }

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
wake_result_t session_on_wake_word(const char *display)
{
    // LISTENING 期间（用户说话中）完全忽略唤醒词触发。
    // wake_word_stop() 已在进入 LISTENING 时调用，此处兜底防竞态窗口误打断。
    // 连麦等待期（s_is_continuous_turn=true）属于 LISTENING 中的特殊子状态，仍需响应。
    if (s_state == SESSION_LISTENING && !s_is_continuous_turn)
        return WAKE_IGNORED;

    PRINT_MEM_INFO(TAG, "对话会话开始");
    if (s_state == SESSION_PLAYING)
    {
        // 区分两种 PLAYING（判据：扬声器缓冲是否还有 PCM 要播，而非 tts_end 文本标志）：
        //   - is_playing==true ：dec_output 仍有数据 → 大模型实际还在出声 → 真打断
        //   - is_playing==false：缓冲已排空，仅状态未切走 → 仅残余排空 → 开新一轮
        // ★ 为什么不用 s_tts_data_done：云端一发声就下发 tts_end，该标志几乎全程为 true，
        //   会把"大模型正在说话时的打断"误判成"开新一轮"。改用 is_playing 与排空逻辑同源
        //   （ws_sender_task 切回 LISTENING 也用它），真实反映扬声器当前是否还在出声。
        bool still_speaking = audio_processor_is_playing(s_processor);
        if (still_speaking)
            ESP_LOGW(TAG, "[WARN] 唤醒词打断 TTS（扬声器播放中）: [%s]", display);
        else
            ESP_LOGI(TAG, "上一轮已答完(缓冲排空)，唤醒开启新一轮: [%s]", display);

        // ① 立刻清空 TTS 解码缓冲，让扬声器尽快停声
        //    云端协议：PLAYING 期间收到新 start = 隐式打断旧轮 + 开新一轮，
        //    无需先发 cancel/abort（cancel 会让云端结束多轮，违背流程 #6）
        audio_processor_flush_output(s_processor);

        // ② 清空上行编码缓冲（enc_input + enc_output），
        //    防止打断瞬间残留的旧 PCM/OPUS 在新 started 到来后混入新轮 ASR
        audio_processor_flush_input(s_processor);

        // ③ [P0] 清除服务器就绪位：阻止 ws_sender_task 在新 started 到来前发送旧会话的音频
        xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);

        // 重置会话状态
        s_tts_data_done = false; // 打断 TTS，清除播放完成等待标志
        s_state = SESSION_LISTENING;
        s_is_continuous_turn = false; // 打断后直接发 start，不走延迟补发路径

        // ④ [P1] 开启排空保护期，防止flush后残余余振被上传
        // SERVER_READY_BIT已清除，on_enhanced_pcm自然不会写PCM，此期间额外兜底
        s_waiting_for_silence = true;
        s_wait_silence_start = xTaskGetTickCount();

        // ⑤ [P0] 立即发送新 start，通知服务端本次打断后开启新一轮对话
        //    ws_sender_task 此时因 SERVER_READY_BIT 被清除而阻塞，
        //    等服务端返回 started → PROTOCOL_EVENT_start 重新 set 该位后才恢复发送
        if (s_protocol && protocol_is_connected(s_protocol))
        {
            protocol_send_start(s_protocol);
            ESP_LOGI(TAG, "打断后发送 start，等待服务端 started 确认新一轮对话");
        }

        wake_word_stop(); // 挂起唤醒词引擎，释放 CPU1 算力
        // 打断后等服务端 started → PROTOCOL_EVENT_start 会重新启动 wait_user_timer，
        // 这里只需停止已存在的两个静默定时器以免误触发
        session_timers_stop_all();
        // ★ still_speaking 决定本次唤醒的语义：
        //   扬声器还在出声 → 真打断（不播提示音，避免打断用户插话的连贯性）；
        //   缓冲已排空 → 上一轮其实已说完，本次等同开启新一轮（播提示音）。
        return still_speaking ? WAKE_INTERRUPT : WAKE_NEW_SESSION;
    }
    // ★ 核心修复 2：如果是在连麦等待期，用户却强行喊了唤醒词，主动发起新一轮对话
    if (s_state == SESSION_LISTENING && s_is_continuous_turn)
    {
        ESP_LOGI(TAG, "连麦等待期听到唤醒词，主动发起新一轮对话");
        if (s_protocol && protocol_is_connected(s_protocol))
        {
            protocol_send_start(s_protocol); // 强行发握手包
        }
        s_is_continuous_turn = false;
        // 等 started → wait_user_timer 会重启；同时停掉 wait_next_turn
        session_timers_stop_all();
        return WAKE_NEW_SESSION; // 连麦等待期主动发起新一轮 → 播提示音
    }
    if (s_state != SESSION_IDLE)
        return WAKE_IGNORED;

    // drain 阶段（session_close 正在等待 decoder 排水）：
    // s_state 已是 IDLE 但 s_processor 尚未销毁，直接 flush dec_input
    // 让排水立即结束，session_close() 快速完成后 wake_word_start() 会重新使能唤醒。
    // 用户需再说一次唤醒词，但系统状态安全，不会双重创建 processor。
    if (s_processor != NULL)
    {
        ESP_LOGW(TAG, "会话正在收尾（drain），flush 加速退出，请再说一次唤醒词");
        audio_processor_flush_output(s_processor);
        return WAKE_IGNORED; // 需用户再说一次，本次不算成功唤醒，不播提示音
    }

    ESP_LOGI(TAG, "会话开始 [%s]", display);
    s_state = SESSION_LISTENING;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);

    // TODO: 播放唤醒提示音 - 需要实现具体的音频播放接口
    // 示例：play_wakeup_beep(); 或 play_audio_file("wakeup_prompt.opus");

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

    // 优先级 4（低于 LVGL 的 5）：ws_sender 与 LVGL 同在 CPU0，发 opus 上行流时
    // 二者同优先级会时间片轮转，抢走 GIF 渲染的 CPU → 监听时 GIF 掉帧。
    // 降到 4 让 LVGL(5) 优先渲染；ws_sender 主体是网络 IO（send_bin 阻塞等 TCP 时
    // 会主动让出 CPU），降一级不影响上行实时性，且编码器输出有 8KB(~2.7s) 缓冲吸收抖动。
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(ws_sender_task, "ws_sender",
                                                     4096, NULL, 4,
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
        ESP_LOGI(TAG, "WebSocket 已就绪，立即发送 start");
        protocol_send_start(s_protocol);
    }
    else
    {
        ESP_LOGW(TAG, "WebSocket 未就绪，等待重连后自动发送 start...");
    }
    // ★ 新增：系统进入第一次录音状态，暂停后台唤醒词检测，防卡顿
    wake_word_stop();
    // wait_user_timer 不在此启动，由 PROTOCOL_EVENT_start（收到 started）触发。
    // 若 8s 内未收到 started，ws_sender_task 会主动 CLOSE 会话兜底（见 [main/session/session.c#L587]）。
    return WAKE_NEW_SESSION; // 从 IDLE 正常开启新会话 → 播提示音

error:
    ESP_LOGE(TAG, "会话启动失败，回滚所有资源");
    s_state = SESSION_IDLE;
    xEventGroupClearBits(s_session_eg, SESSION_SERVER_READY_BIT);
    wake_word_start();
    return WAKE_IGNORED; // 启动失败，不播提示音
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

// ─── 【调试】主动断连测试 ───────────────────────────────────────────────────

/**
 * @brief 调试断连任务：主动 close WebSocket，模拟服务端 FIN 断链
 *
 * 加锁串行化对 s_protocol 的访问，与 session_reconnect_task 互斥，
 * 只 close 不 destroy（destroy 交给后续重连任务），close 会触发底层
 * WEBSOCKET_EVENT_DISCONNECTED → PROTOCOL_EVENT_DISCONNECTED → 退避重连。
 *
 * @param arg 未使用
 */
static void session_debug_kill_ws_task(void *arg)
{
    ESP_LOGW(TAG, "[调试] 收到 ws_kill 指令，主动断开 WebSocket 模拟服务端 FIN...");
    if (s_protocol_mutex != NULL)
        xSemaphoreTake(s_protocol_mutex, portMAX_DELAY);
    if (s_protocol != NULL)
        protocol_disconnect_timeout(s_protocol, 3000);
    if (s_protocol_mutex != NULL)
        xSemaphoreGive(s_protocol_mutex);
    ESP_LOGW(TAG, "[调试] WebSocket 已主动断开，等待退避重连流程接管");
    // ★ 本任务由 xTaskCreatePinnedToCoreWithCaps 创建（4KB INTERNAL 栈，单独 heap_caps 分配），
    //   自删必须用 vTaskDeleteWithCaps，否则栈内存不会被 idle 回收 → 每次 ws_kill 泄漏 ~4KB 内部 SRAM。
    vTaskDeleteWithCaps(NULL);
}

/**
 * @brief 【调试】主动断开 WebSocket 连接（见 .h 说明）
 *
 * 创建独立任务异步执行，避免在 MQTT 事件回调上下文直接 close 阻塞事件循环。
 * 栈用内部 SRAM：close 路径可能触及 TLS 上下文释放，SPIRAM 栈在 Cache 关闭时不可访问。
 */
void session_debug_kill_ws(void)
{
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        session_debug_kill_ws_task, "dbg_ws_kill",
        4096, NULL, 4, NULL,
        tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ret != pdPASS)
        ESP_LOGE(TAG, "[调试] 创建 ws_kill 任务失败（内存不足）");
}