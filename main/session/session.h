#pragma once

/**
 * @file session.h
 * @brief 会话状态机接口 — 语音对话生命周期管理
 *
 * 本模块是整个系统的业务逻辑中枢，管理从唤醒词触发到多轮对话结束的完整流程：
 *
 * 状态机示意：
 *
 *   SESSION_IDLE
 *       │ session_on_wake_word()（唤醒词触发）
 *       ↓
 *   SESSION_LISTENING  ← 麦克风→OPUS→WebSocket→云端
 *       │ PROTOCOL_EVENT_TTS_START（TTS 开始）
 *       ↓
 *   SESSION_PLAYING    ← OPUS 解码→扬声器，唤醒词引擎监听打断
 *       │ PROTOCOL_EVENT_TTS_END（TTS 结束）→ 连续对话
 *       │ PROTOCOL_EVENT_COMPLETE 或 60s 超时
 *       ↓
 *   SESSION_IDLE
 *
 * 连接策略：
 *   WiFi 就绪后立即预建立 WebSocket 连接（含 TLS 握手），
 *   唤醒词触发时直接发送 start，无需等待连接，实现零等待唤醒响应。
 *
 * 关键定时参数：
 *   SESSION_TIMEOUT_MS  = 60000 ms  ← 会话整体超时（60 秒无活动自动关闭）
 *   EOS_SILENCE_MS      = 800 ms    ← VAD 静音 800ms 判定说话结束
 *   VAD_GRACE_MS        = 500 ms    ← 唤醒词尾音消退保护期
 *   TOKEN_REFRESH_MS    = 110min    ← accessToken 主动刷新周期（2h 过期前 10min 刷新）
 *   RECONNECT_MAX       = 5 次      ← 最大重连次数
 *   RECONNECT_BASE_DELAY= 5000 ms   ← 指数退避基础延迟（5s/10s/20s/40s/60s）
 *
 * @note 调用前置条件：
 *   1. NVS 已初始化（bsp_board_nvs_init）
 *   2. 音频已初始化（audio_init）
 *   3. WiFi 已连接（bsp_board_wifi_main 完成并置 WIFI_BIT）
 *   4. 唤醒词引擎已初始化（wake_word_init）
 */

// ─── 会话状态枚举 ─────────────────────────────────────────────────────────────

/**
 * @brief 会话状态枚举
 *
 * 整个系统在同一时刻只处于三种状态之一：
 *   IDLE      → 无活跃会话，唤醒词引擎正在监听，WebSocket 保持预连接
 *   LISTENING → 用户说话中，音频正在推送至云端，等待 VAD 静音触发结束
 *   PLAYING   → 正在播放云端 TTS 合成语音，同时监听唤醒词以支持打断
 */
typedef enum
{
    SESSION_IDLE,      ///< 待机状态：唤醒词监听中，WebSocket 预连接保持，无音频处理
    SESSION_LISTENING, ///< 监听状态：麦克风 PCM → OPUS 编码 → WebSocket → 云端大模型
    SESSION_PLAYING,   ///< 播放状态：云端 TTS → OPUS 解码 → 扬声器，唤醒词引擎同时监听打断
} session_state_t;

// ─── 公开 API ─────────────────────────────────────────────────────────────────

/**
 * @brief 初始化会话模块并建立 WebSocket 预连接
 *
 * 完整初始化流程：
 *   1. 创建 EventGroup（SESSION_SERVER_READY_BIT / SESSION_WS_CONNECTED_BIT）
 *   2. 创建会话事件队列（5 条消息容量，防止定时器栈溢出）
 *   3. 启动会话专职事件任务（session_event_task，8KB 栈于 SPIRAM）
 *   4. 从 NVS "net_config" 读取 ws_uri 和 device_token
 *   5. 若有 deviceToken，调用 auth_perform() 换取 accessToken（HTTP POST）
 *   6. 创建会话超时、EOS 静音检测、Token 刷新三个软件定时器
 *   7. 创建 protocol_t 实例，注册 protocol_event_handler 回调
 *   8. 调用 protocol_connect() 发起 WebSocket 预连接（含 TLS 握手）
 *
 * @param ws_uri  WebSocket 服务器地址（如 "ws://192.168.1.100:8080/audio"）；
 *                传 NULL 则从 NVS 读取，NVS 无则使用编译期默认地址
 *
 * @note 调用者：application.c → app_main()，在 WiFi 连接完成后调用
 * @note 此函数会阻塞执行 HTTP 认证请求（最多几秒），建议不在时间敏感任务中调用
 */
void session_init(const char *ws_uri);

/**
 * @brief 唤醒词触发入口（MultiNet 检测到命令词后调用）
 *
 * 根据当前状态执行不同逻辑：
 *
 * 情形 A — 从 IDLE 启动新会话（正常流程）：
 *   1. 状态 → SESSION_LISTENING
 *   2. 创建 audio_processor_t（编解码管道）
 *   3. 注册 enhanced_pcm 钩子（麦克风降噪 PCM 进入编码器）
 *   4. 创建 ws_sender_task（负责读取 OPUS 并发送）
 *   5. 若 WebSocket 已连接，立即发送 start 握手
 *   6. 启动 60 秒会话超时定时器
 *
 * 情形 B — PLAYING 状态中打断 TTS：
 *   1. 发送 abort 指令（通知服务端停止推送音频）
 *   2. 清空解码输出缓冲区（立即停止扬声器）
 *   3. 状态 → SESSION_LISTENING（继续录音）
 *   4. 重置会话超时定时器
 *
 * @param display  唤醒词显示文字，如 "你好伙伴" 或 "start Echo"
 *                 （从 custom_wake_word 的触发回调中透传而来）
 *
 * @note 调用者：custom_wake_word.c → afe_fetch_task → user_callback
 * @note 线程安全：可从 AFE fetch 任务调用（CPU1），内部使用互斥锁保护
 */

/**
 * @brief 唤醒词触发的处理结果
 *
 * 供调用方（wake_word_callback）区分本次唤醒命中实际走了哪条逻辑，
 * 据此决定是否播放唤醒提示音：
 *   - 只有 WAKE_NEW_SESSION（真·开启新会话）才播提示音；
 *   - WAKE_INTERRUPT（打断 TTS）不播——打断是插话，播提示音会打断用户表达；
 *   - WAKE_IGNORED（被忽略：LISTENING 中 / drain 收尾 / 启动失败）不播。
 */
typedef enum
{
    WAKE_IGNORED = 0, ///< 本次唤醒被忽略，未开启会话也未打断
    WAKE_NEW_SESSION, ///< 开启了一轮新会话（含从 IDLE 启动、连麦等待期主动发起）→ 应播提示音
    WAKE_INTERRUPT,   ///< 打断了正在播放的 TTS → 不播提示音
} wake_result_t;

wake_result_t session_on_wake_word(const char *display);

/**
 * @brief 查询当前会话状态（线程安全只读访问）
 *
 * @return session_state_t 当前状态（IDLE / LISTENING / PLAYING）
 *
 * @note 调用者：任意模块（只读查询），主要供调试或 UI 刷新使用
 */
session_state_t session_get_state(void);

/**
 * @brief 【调试】主动断开 WebSocket 连接，用于测试断连重连逻辑
 *
 * 内部创建独立任务调用带超时的 protocol close 主动关闭 WS，
 * 等效于服务端 FIN 断链，会触发 PROTOCOL_EVENT_DISCONNECTED → 退避重连。
 * 必须异步执行：调用方常在 MQTT 事件回调上下文，直接 close 会阻塞事件循环。
 *
 * @note 调用者：mqtt_protocol.c 收到 {"type":"ws_kill"} 指令时；仅供联调测试。
 */
void session_debug_kill_ws(void);

/**
 * @brief 为 OTA 升级彻底停止会话，释放内部 SRAM/CPU 给固件下载
 *
 * 置内部 OTA 锁定标志（此后唤醒词不再拉起会话），并异步投递关闭事件：
 * 停止 PCM Hook、停止并销毁 audio_processor、停所有定时器、会话回到 IDLE。
 * 关闭为异步执行（投递事件后立即返回），调用方（bsp_ota）应在下载前调用并
 * 短暂等待（如 500ms）让关闭完成后再开始下载。
 *
 * @note 锁定标志不会清除——OTA 无论成功或失败都会 esp_restart()，重启后自然复位。
 * @note 线程安全：可从任意任务（如 OTA 下载任务）调用。
 */
void session_stop_for_ota(void);
