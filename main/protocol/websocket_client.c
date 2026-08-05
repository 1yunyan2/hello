/**
 * @file websocket_client.c
 * @brief WebSocket 协议层实现 — 消息收发与事件分发
 *
 * 本模块实现了设备与云端大模型之间的完整 WebSocket 通信协议：
 *
 * 上行（设备→云端）：
 *   - start 握手：协商音频参数，建立会话
 *   - 唤醒词通知：告知服务端触发的唤醒词
 *   - 监听控制：start / stop / detect
 *   - 音频帧：OPUS 编码的麦克风数据（Binary Frame）
 *   - 打断指令：用户说唤醒词打断 TTS
 *   - IoT 消息：设备能力和状态上报
 *
 * 下行（云端→设备）：
 *   - start 响应：返回 session_id
 *   - STT 结果：语音识别文本
 *   - LLM 状态：大模型情感标签
 *   - TTS 控制：start / stop / sentence_start
 *   - 音频帧：OPUS 编码的 TTS 合成语音（Binary Frame）
 *   - IoT 指令：远程控制命令
 */

#include "websocket_client.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_timer.h" // 【诊断】[WS接收] 回调持锁耗时打点用 esp_timer_get_time()
#include "esp_crt_bundle.h"
#include "esp_mac.h"
#include "esp_heap_caps.h"
#include "cJSON.h"
#include <string.h>
#include "object.h"

/* 定义协议事件基类（用于 esp_event 框架） */
ESP_EVENT_DEFINE_BASE(PROTOCOL_EVENT);

#define TAG "Protocol"

/**
 * @brief 发送 JSON 文本消息的便捷宏
 *
 * 自动检查连接状态 → 格式化 JSON 字符串 → 发送 → 释放内存。
 * 若未连接则直接 return（宏展开在调用函数内，会退出调用函数）。
 *
 * @param protocol 协议实例指针
 * @param fmtstr   printf 格式字符串
 * @param ...      格式化参数
 */
#define protocol_send_text(protocol, fmtstr, ...)                                                                    \
    do                                                                                                               \
    {                                                                                                                \
        if (!esp_websocket_client_is_connected(protocol->websocket_client))                                          \
            return;                                                                                                  \
        char *_message = NULL;                                                                                       \
        asprintf(&_message, fmtstr, ##__VA_ARGS__);                                                                  \
        ESP_LOGD(TAG, "Send JSON: %s", _message);                                                                    \
        esp_websocket_client_send_text(protocol->websocket_client, _message, strlen(_message), pdMS_TO_TICKS(5000)); \
        free(_message);                                                                                              \
    } while (0)

/**
 * @brief 协议实例内部结构体
 *
 * 封装底层 WebSocket 客户端句柄和会话状态。
 * 对外通过 protocol_t 前向声明隐藏实现细节。
 */
struct protocol
{
    esp_websocket_client_handle_t websocket_client; ///< 底层 WebSocket 客户端句柄
    char *session_id;                               ///< 服务端分配的会话 ID（start 响应中获取）
    esp_event_handler_t callback;                   ///< 上层注册的事件回调函数
    void *handler_args;                             ///< 回调函数的用户自定义参数
};

/* ★ 音频接收 buffer：放在 SPIRAM 节省 8KB 内部 SRAM（仅一个 protocol 实例，文件级共享） */
#define AUDIO_RX_BUF_SIZE 8192
static uint8_t *s_audio_rx_buf = NULL;

/* ★ 文本消息接收缓冲区：用于 JSON 分片重组，断连时需要清理 */
static char *s_text_rx_buf = NULL;
static int s_text_rx_offset = 0;
static int s_text_rx_total_len = 0;

// ─── 下行消息处理器 ────────────────────────────────────────────────────────
// 每个 handler 负责解析一种 type 的 JSON 消息，提取数据后通过回调通知上层

/**
 * @brief 处理服务端 start 响应
 * 提取 session_id 并保存，通知上层握手完成
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_start_handler(protocol_t *protocol, cJSON *root)
{
    /* 释放旧的 session_id（重连场景） */
    if (protocol->session_id)
    {
        free(protocol->session_id);
        protocol->session_id = NULL;
    }

    /* 提取新的 session_id */
    cJSON *session_id = cJSON_GetObjectItem(root, "session_id");
    if (cJSON_IsString(session_id))
    {
        protocol->session_id = strdup // 含义：复制字符串并返回新指针，此处保存服务端分配的 session_id
            (session_id->valuestring);
    }

    /* 通知上层：start 握手完成 */
    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_start, NULL);
}

/**
 * @brief 处理大模型情感状态消息
 * 提取 emotion 字段（如 "happy" / "thinking"）通知上层
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_llm_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *emotion = cJSON_GetObjectItem(root, "emotion");
    if (cJSON_IsString(emotion))
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_LLM, emotion->valuestring);
    }
}

/**
 * @brief 处理语音识别（STT）结果消息
 * 提取 text 字段（识别出的文本）通知上层
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_stt_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *text = cJSON_GetObjectItem(root, "text");
    if (cJSON_IsString(text))
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_STT, text->valuestring);
    }
}

/**
 * @brief 处理 TTS 状态消息
 * 根据 state 字段分发三种事件：
 *   "start"          → TTS_START（开始播放）
 *   "stop"           → TTS_STOP（播放结束）
 *   "sentence_start" → TTS_SENTENCE_START（新句子开始，附带文本）
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_tts_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *state = cJSON_GetObjectItem(root, "state");
    if (!cJSON_IsString(state))
        return;

    if (strcmp(state->valuestring, "start") == 0)
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_START, NULL);
    }
    else if (strcmp(state->valuestring, "stop") == 0)
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_END, NULL);
    }
    else if (strcmp(state->valuestring, "sentence_start") == 0)
    {
        cJSON *text = cJSON_GetObjectItem(root, "text");
        if (cJSON_IsString(text))
            protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_SENTENCE_START, text->valuestring);
    }
}

/**
 * @brief 处理错误消息
 * 提取错误信息并通知上层
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_error_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *msg = cJSON_GetObjectItem(root, "message");
    if (cJSON_IsString(msg))
    {
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_ERROR, msg->valuestring);
    }
}

/**
 * @brief 处理会话完成消息
 * 通知上层会话已完成
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_complete_handler(protocol_t *protocol, cJSON *root)
{
    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_COMPLETE, NULL);
}

/**
 * @brief 处理 IoT 控制指令消息
 * 提取 commands 数组通知上层执行设备控制操作
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_iot_handler(protocol_t *protocol, cJSON *root)
{
    cJSON *commands = cJSON_GetObjectItem(root, "commands");
    if (commands)
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_IOT, commands);
}

// ─── WebSocket 底层事件处理 ────────────────────────────────────────────────

/**
 * @brief WebSocket 事件总入口 — 接收底层事件并分发到对应处理器
 *
 * 处理三种底层事件：
 *   CONNECTED    → 通知上层连接成功
 *   DATA         → 区分 Binary（音频）和 Text（JSON），分别处理
 *   DISCONNECTED → 通知上层连接断开
 *
 * Binary Frame（opcode 0x02）：直接封装为 binary_data_t 传递音频数据
 * Text Frame（opcode 0x01）：解析 JSON，根据 type 字段路由到对应 handler
 *
 * @param handler_args 用户自定义参数（此处为protocol_t指针）
 * @param base 事件基类（WEBSOCKET_EVENT）
 * @param event_id 具体事件ID
 * @param event_data 事件相关数据
 *
 * 调用者：ESP-IDF WebSocket客户端底层事件系统
 */
static void protocol_websocket_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    protocol_t *protocol = (protocol_t *)handler_args;
    if (!protocol->callback)
        return;
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id)
    {
    // ── WebSocket错误事件处理 ────────────────────────────────────────────────
    /**
     * @brief WebSocket错误事件处理
     *
     * 说明：处理WebSocket底层错误事件，记录错误日志。
     * API：ESP_LOGE
     * 数据：无状态修改
     */
    case WEBSOCKET_EVENT_ERROR:
        ESP_LOGE(TAG, "Websocket Error");
        break;

    // ── WebSocket连接成功事件处理 ────────────────────────────────────────────
    /**
     * @brief WebSocket连接成功事件处理
     *
     * 说明：处理WebSocket连接成功的事件，通知上层连接已建立，
     *       并打印内存使用情况快照用于调试。
     * API：protocol->callback, PRINT_MEM_INFO
     * 数据：通过回调通知PROTOCOL_EVENT_CONNECTED事件
     */
    case WEBSOCKET_EVENT_CONNECTED:
        ESP_LOGI(TAG, "Websocket Connected");
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_CONNECTED, NULL);
        PRINT_MEM_INFO(TAG, "握手后");
        break;

        // ── WebSocket数据接收事件处理 ────────────────────────────────────────────
        /**
         * @brief WebSocket数据接收事件处理
         *
         * 说明：处理接收到的数据事件，区分Binary Frame（音频数据）和Text Frame（JSON控制消息），
         *       对Text Frame进行JSON解析并根据type字段路由到对应的处理器。
         * API：cJSON_ParseWithLength, cJSON_GetObjectItem, strcmp, cJSON_Delete
         * 数据：根据消息类型调用不同的处理器函数
         */
    case WEBSOCKET_EVENT_DATA:
        /* opcode 0x02 = Binary Frame -> 云端下发的 OPUS 音频帧 */
        if (data->op_code == 0x02)
        {
            // ★ 使用 SPIRAM 分配的 buffer，节省 8KB 内部 SRAM
            if (s_audio_rx_buf == NULL)
            {
                ESP_LOGE(TAG, "audio_rx_buf 未初始化，丢弃音频帧");
                return;
            }
            static int s_audio_rx_offset = 0;

            // 🟢 救命装甲 1：新帧强制复位！
            // WebSocket 协议规定，如果是一帧的开头，payload_offset 必定为 0。
            // 无论上一次重组进行到哪里（有没有卡死），只要收到新帧开头，立刻清零重来！
            if (data->payload_offset == 0)
            {
                s_audio_rx_offset = 0;
            }

            // 🟢 救命装甲 2：溢出保护，防止内存踩踏
            if (s_audio_rx_offset + data->data_len > AUDIO_RX_BUF_SIZE)

            {
                ESP_LOGW(TAG, "音频帧分片异常或过大(len=%d)，丢弃防爆内存", data->payload_len);
                s_audio_rx_offset = 0;
                return;
            }

            memcpy(s_audio_rx_buf + s_audio_rx_offset, data->data_ptr, data->data_len);
            s_audio_rx_offset += data->data_len;

            // 拼齐了一整帧
            if (s_audio_rx_offset >= data->payload_len && data->payload_len > 0)
            {
                // 🟢 救命装甲 3：微小碎片过滤（丢弃网络残留垃圾，防 error:-4）
                if (data->payload_len > 15)
                {
                    binary_data_t bin = {.ptr = s_audio_rx_buf, .size = (size_t)data->payload_len};

                    // ── 【诊断打点，纯统计不影响逻辑】接收回调阻塞时长 ──────────────
                    // 本回调运行在 esp_websocket_client 的接收任务里，且**持有
                    // client->lock**。下游 audio_processor_write() 内部最多重试 30×100ms
                    // = 3 秒才放弃（dec_input 满时），这 3 秒内锁一直不放。
                    // 与此同时 ws_sender 若调 send_bin，会在
                    // esp_websocket_client.c:711 用 portMAX_DELAY 抢同一把锁 → 永久阻塞。
                    //
                    // 判读：本条耗时若经常 >100ms，即证实接收侧长期持锁，
                    //       与 [WS发送] 只见"进入"不见"返回"互为印证。
                    static int64_t diag_rx_last_us = 0;
                    int64_t diag_rx_t0 = esp_timer_get_time();

                    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_AUDIO, &bin);

                    int64_t diag_rx_dt = esp_timer_get_time() - diag_rx_t0;
                    // 限频 2 秒；但只要单次超过 50ms 就立即打印（异常优先于限频）
                    if (diag_rx_dt > 50000 || (diag_rx_t0 - diag_rx_last_us) >= 2000000)
                    {
                        diag_rx_last_us = diag_rx_t0;
                        ESP_LOGW(TAG, "[WS接收] 回调持锁 耗时=%d.%02dms size=%d",
                                 (int)(diag_rx_dt / 1000), (int)((diag_rx_dt % 1000) / 10),
                                 (int)data->payload_len);
                    }
                }

                // 送完后，立刻清零，准备迎接下一帧
                s_audio_rx_offset = 0;
            }
            return;
        }
        /* opcode 0x01 = Text Frame → JSON 控制消息 */
        if (data->op_code == 0x01)
        {
            // 如果是新消息的开始（payload_offset == 0），重置缓冲区
            if (data->payload_offset == 0)
            {
                // 释放之前的缓冲区（如果有）
                if (s_text_rx_buf)
                {
                    free(s_text_rx_buf);
                    s_text_rx_buf = NULL;
                }
                s_text_rx_offset = 0;
                s_text_rx_total_len = data->payload_len;

                // 分配足够的缓冲区来存储完整的消息
                if (s_text_rx_total_len > 0)
                {
                    s_text_rx_buf = (char *)malloc(s_text_rx_total_len + 1); // +1 for null terminator
                    if (!s_text_rx_buf)
                    {
                        ESP_LOGE(TAG, "内存分配失败，无法处理文本消息 (len=%d)", s_text_rx_total_len);
                        return;
                    }
                }
            }

            // 检查缓冲区是否已分配且不会溢出
            if (s_text_rx_buf && s_text_rx_offset + data->data_len <= s_text_rx_total_len)
            {
                memcpy(s_text_rx_buf + s_text_rx_offset, data->data_ptr, data->data_len);
                s_text_rx_offset += data->data_len;
            }
            else
            {
                ESP_LOGW(TAG, "文本消息分片异常或缓冲区不足，丢弃消息");
                if (s_text_rx_buf)
                {
                    free(s_text_rx_buf);
                    s_text_rx_buf = NULL;
                }
                s_text_rx_offset = 0;
                s_text_rx_total_len = 0;
                return;
            }

            // 如果收到了完整的消息
            if (s_text_rx_offset >= s_text_rx_total_len && s_text_rx_total_len > 0)
            {
                /* 【堆损坏排查·2026-07-29 已排除，暂停用】
                 * 下一行写 '\0' 曾因"最像事故指纹"（Bad tail 被写成 0x00000000）被列为嫌疑。
                 * 实测排除：日志中从未出现「收到文本帧分片」，即本工程实际未走分片路径，
                 * offset==total_len 恒落在 malloc(total_len+1) 的最后一个合法字节内，不越界。
                 * 真凶是 Tmr Svc 栈溢出（详见 ui_port.c main_gif_apply_index 处注释）。 */
                // HEAP_PROBE("HEAPCHK", "文本帧写\\0前");
                // 添加null终止符以便打印和解析
                s_text_rx_buf[s_text_rx_offset] = '\0';
                // HEAP_PROBE("HEAPCHK", "文本帧写\\0后");

                ESP_LOGI(TAG, "收到文本帧: %s", s_text_rx_buf);
                cJSON *root = cJSON_Parse(s_text_rx_buf);
                if (!root)
                {
                    ESP_LOGW(TAG, "JSON 解析失败");
                }
                else
                {
                    /* 根据 type 字段路由到对应的消息处理器 */
                    cJSON *type = cJSON_GetObjectItem(root, "type");
                    if (cJSON_IsString(type))
                    {
                        if (strcmp(type->valuestring, "started") == 0) // 当收到服务端的 start 响应时，type 字段是 "started"
                            protocol_start_handler(protocol, root);
                        else if (strcmp(type->valuestring, "llm") == 0) // 假设服务端发情感状态的 type 字段是 "llm"
                            protocol_llm_handler(protocol, root);
                        // else if (strcmp(type->valuestring, "stt") == 0)
                        //     protocol_stt_handler(protocol, root);
                        else if (strcmp(type->valuestring, "tts") == 0) // 假设服务端发 TTS 状态的 type 字段是 "tts"
                            protocol_tts_handler(protocol, root);
                        //  兼容你服务端的 tts_start
                        else if (strcmp(type->valuestring, "tts_start") == 0)
                            protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_START, NULL);
                        //  兼容你服务端的 tts_end，
                        else if (strcmp(type->valuestring, "tts_end") == 0)
                            protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_END, NULL);
                        else if (strcmp(type->valuestring, "iot") == 0)
                            protocol_iot_handler(protocol, root);
                        else if (strcmp(type->valuestring, "error") == 0)
                            protocol_error_handler(protocol, root);
                        else if (strcmp(type->valuestring, "complete") == 0)
                            protocol_complete_handler(protocol, root);
                        else if (strcmp(type->valuestring, "transcript") == 0)
                            protocol_stt_handler(protocol, root); // 假设你的 stt_handler 是处理文字结果的
                    }
                    cJSON_Delete(root);
                }

                // 清理缓冲区，准备接收下一条消息
                free(s_text_rx_buf);
                s_text_rx_buf = NULL;
                s_text_rx_offset = 0;
                s_text_rx_total_len = 0;
            }
            else
            {
                // 还未收到完整消息，只打印当前分片用于调试
                ESP_LOGI(TAG, "收到文本帧分片: %.*s", data->data_len, data->data_ptr);
            }
            return;
        }
        break;

    // ── WebSocket断开连接事件处理 ────────────────────────────────────────────
    /**
     * @brief WebSocket断开连接事件处理
     *
     * 说明：处理WebSocket断开连接或结束的事件，通知上层连接已断开。
     * API：protocol->callback
     * 数据：通过回调通知PROTOCOL_EVENT_DISCONNECTED事件
     */
    case WEBSOCKET_EVENT_DISCONNECTED:
    case WEBSOCKET_EVENT_FINISH:
        // 清理文本消息缓冲区，防止断连时正在重组的 JSON 消息泄漏
        if (s_text_rx_buf)
        {
            free(s_text_rx_buf);
            s_text_rx_buf = NULL;
        }
        s_text_rx_offset = 0;
        s_text_rx_total_len = 0;
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_DISCONNECTED, NULL);
        break;
    }
}

// ─── 公开 API：生命周期管理 ────────────────────────────────────────────────

/**
 * @brief 创建并初始化 WebSocket 客户端实例
 *
 * 分配内存创建 protocol_t 结构体，根据传入的 URL 和 Token 配置 WebSocket 连接参数，
 * 包括自定义 HTTP 头（Device-Id, Authorization 等）。该函数不立即建立连接。
 *
 * 自定义 HTTP 头格式：
 *   Device-Id: <MAC 地址>
 *   Client-Id: <MAC 地址>
 *   Authorization: Bearer <accessToken>
 *   Protocol-Version: 1
 *
 * @param url   WebSocket 服务器的完整地址（如 "wss://example.com/audio"）
 * @param token 用于身份验证的 Bearer Token（accessToken），可为 NULL
 * @return protocol_t* 成功时返回指向新创建实例的指针，失败时返回 NULL
 *
 * 调用者：session_init函数
 */
protocol_t *protocol_create(const char *url, const char *token)
{
    protocol_t *protocol = (protocol_t *)calloc(1, sizeof(protocol_t));
    if (protocol == NULL)
    {
        ESP_LOGE(TAG, "protocol calloc 失败，内存不足");
        return NULL;
    }

    /* 读取设备 WiFi MAC 地址作为 Device-Id 和 Client-Id */
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char mac_str[18];
    snprintf // 含义：将格式化的数据写入字符串中，此处将MAC地址格式化为常见的冒号分隔形式
        (mac_str, sizeof(mac_str), "%02x:%02x:%02x:%02x:%02x:%02x", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    /* 构造自定义 HTTP 头（WebSocket 握手时携带） */
    char *headers = NULL;
    asprintf // 含义：将参数列表中的内容格式化成字符串，并返回该字符串的指针。
        (&headers, "Device-Id: %s\r\nClient-Id: %s\r\nAuthorization: Bearer %s\r\nProtocol-Version: 1\r\n",
         mac_str, mac_str, token ? token : "");

    /* 配置 WebSocket 客户端参数 */
    esp_websocket_client_config_t websocket_cfg = {
        .uri = url,
        .headers = headers,
        .crt_bundle_attach = esp_crt_bundle_attach, // wss:// 根证书校验
        .network_timeout_ms = 3000,     // 网络超时 3 秒（缩短让连接失败更快触发重试）
        .disable_auto_reconnect = true, // 禁用自动重连，由 session 层控制退避策略
        // ★ Ping 保活说明：
        // 设备端 ping 要求服务端必须回 pong（RFC 6455）。
        // 若服务端未实现 pong 响应，pingpong_timeout_sec 到期后 ESP-IDF 会主动断开，
        // 表现为 "transport_poll_write returned 0, errno=Success"（不是真网络断）。
        //
        // 当前策略：关闭设备端主动 ping（服务端有自己的心跳机制）。
        // 若需要开启，必须先确认服务端已实现 pong 响应，再取消下面两行注释。
        // .ping_interval_sec = 20,
        // .pingpong_timeout_sec = 10,
        .buffer_size = 8192, //! 增加了缓存，防止接收数据过大 默认接收缓冲区大小是 1024 字节
    };

    /* 初始化底层 WebSocket 客户端并注册事件回调 */
    protocol->websocket_client = esp_websocket_client_init(&websocket_cfg);
    // 注册事件回调，监听所有 WebSocket 事件（WEBSOCKET_EVENT_ANY），传递协议实例指针作为参数
    esp_websocket_register_events(protocol->websocket_client, WEBSOCKET_EVENT_ANY, protocol_websocket_event_handler, protocol);

    /* ★ 分配音频接收 buffer 到 SPIRAM（节省 8KB 内部 SRAM） */
    if (s_audio_rx_buf == NULL)
    {
        s_audio_rx_buf = (uint8_t *)heap_caps_malloc(AUDIO_RX_BUF_SIZE, MALLOC_CAP_SPIRAM);
        if (s_audio_rx_buf == NULL)
            ESP_LOGE(TAG, "audio_rx_buf SPIRAM 分配失败，音频接收将不可用");
    }

    free(headers); // headers 已被底层拷贝，可安全释放
    return protocol;
}

/**
 * @brief 销毁 WebSocket 客户端实例并释放所有资源
 *
 * @param protocol 协议实例指针，调用后不应再使用
 *
 * 调用者：session_reconnect_task、session_close等需要清理资源的地方
 */
void protocol_destroy(protocol_t *protocol)
{
    if (protocol->session_id)
        free(protocol->session_id);
    esp_websocket_client_destroy(protocol->websocket_client);
    free(protocol);

    /* ★ 释放音频接收 buffer */
    if (s_audio_rx_buf)
    {
        free(s_audio_rx_buf);
        s_audio_rx_buf = NULL;
    }
}

// ─── 公开 API：连接控制 ────────────────────────────────────────────────────

/**
 * @brief 建立 WebSocket 连接
 * 如果已经连接则不执行任何操作
 *
 * @param protocol 协议实例指针
 *
 * 调用者：session_init、session_reconnect_task
 */
void protocol_connect(protocol_t *protocol)
{
    if (!esp_websocket_client_is_connected(protocol->websocket_client))
        esp_websocket_client_start(protocol->websocket_client);
    PRINT_MEM_INFO(TAG, "握手前");
}

/**
 * @brief 断开 WebSocket 连接并确保底层资源完全释放
 *
 * 【关键】无论当前连接状态如何都调用 stop，确保：
 *   1. 向服务端发送 WebSocket close frame（正常关闭握手）
 *   2. 底层 TCP 连接被正确关闭，服务端释放连接计数
 *   3. 避免服务端残留"幽灵连接"导致 429 连接超限
 *
 * 之前的 bug：只在 is_connected 为 true 时才 stop，
 * 但断线事件触发时 is_connected 已经为 false，导致 stop 从未被调用，
 * 服务端无法感知连接关闭，连接数持续累积。
 *
 * @param protocol 协议实例指针
 *
 * 调用者：session_reconnect_task、session_close
 */
void protocol_disconnect(protocol_t *protocol)
{
    /* 无论是否 connected 都调用 stop，确保底层 TCP 彻底关闭 */
    esp_websocket_client_stop(protocol->websocket_client);
}

/**
 * @brief 带超时地断开 WebSocket 连接（见 .h Doxygen 说明）
 *
 * esp_websocket_client_close 与 stop 的区别：close 会发送 WebSocket close 帧并
 * 接受 timeout 参数，在 timeout 内未完成干净关闭也会返回，从而避免 stop 在
 * FIN/TLS 半关闭态下无限阻塞 ws_reconn 任务。超时返回后调用方应继续 destroy 兜底。
 *
 * 调用者：session_reconnect_task（换句柄前的安全断开）
 */
void protocol_disconnect_timeout(protocol_t *protocol, int timeout_ms)
{
    esp_websocket_client_close(protocol->websocket_client, pdMS_TO_TICKS(timeout_ms));
}

/**
 * @brief 查询 WebSocket 连接状态
 *
 * @param protocol 协议实例指针
 * @return bool 连接状态，true表示已连接，false表示未连接
 *
 * 调用者：ws_sender_task、session_on_wake_word等需要检查连接状态的地方
 */
bool protocol_is_connected(protocol_t *protocol)
{
    return esp_websocket_client_is_connected(protocol->websocket_client);
}

// ─── 公开 API：消息发送 ────────────────────────────────────────────────────

/**
 * @brief 发送 start 握手消息
 * 协商音频参数：单声道 / OPUS 编码 / 60ms 帧时长 / 16kHz 采样率
 *
 * @param protocol 协议实例指针
 *
 * 调用者：session_on_wake_word、PROTOCOL_EVENT_CONNECTED事件处理
 */
void protocol_send_start(protocol_t *protocol)
{
    ESP_LOGI(TAG, "发送 Start 握手消息...");

    // 获取 MAC 地址后三字节生成 toyId (与 MQTT 一致)
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);
    char toy_id[16];
    snprintf(toy_id, sizeof(toy_id), "%02X%02X%02X", mac[3], mac[4], mac[5]);

    protocol_send_text(protocol,
                       "{\"type\":\"start\",\"format\":\"opus\",\"sampleRate\":16000,\"deviceId\":\"%s\"}",
                       toy_id);
}

/**
 * @brief 发送唤醒词通知（type=listen, state=detect）
 *
 * @param protocol 协议实例指针
 * @param wake_word 唤醒词字符串
 *
 * 调用者：目前未使用，仅打印日志
 */
void protocol_send_wake_word(protocol_t *protocol, const char *wake_word)
{
    // protocol_send_text(protocol, "{\"session_id\":\"%s\",\"state\":\"detect\",\"text\":\"%s\",\"type\":\"listen\"}", protocol->session_id ? protocol->session_id : "", wake_word);
    ESP_LOGI("Protocol", "本地已唤醒: %s", wake_word);
}

/**
 * @brief 发送开始监听指令（type=listen, state=start）
 *
 * @param protocol 协议实例指针
 * @param type 监听类型（未使用）
 *
 * 调用者：目前未使用
 */
void protocol_send_start_listening(protocol_t *protocol, protocol_listen_type_t type)
{
    // static const char *mode_str[] = {"auto", "manual", "realtime"};
    // protocol_send_text(protocol, "{\"mode\":\"%s\",\"session_id\":\"%s\",\"state\":\"start\",\"type\":\"listen\"}", mode_str[type], protocol->session_id ? protocol->session_id : "");
}

/**
 * @brief 发送停止监听指令（type=listen, state=stop）
 *
 * @param protocol 协议实例指针
 *
 * 调用者：on_eos_timeout定时器回调
 */
void protocol_send_stop_listening(protocol_t *protocol)
{
    // protocol_send_text(protocol, "{\"session_id\":\"%s\",\"state\":\"stop\",\"type\":\"listen\"}", protocol->session_id ? protocol->session_id : "");
    // 这里保留了 session_id（通常后端追踪会话都需要）
    protocol_send_text(protocol, "{\"type\":\"stop\"}");
    // protocol_send_text(protocol, "{\"session_id\":\"%s\",\"type\":\"stop\"}", protocol->session_id ? protocol->session_id : "");
}

/**
 * @brief 发送 OPUS 音频二进制帧（Binary Frame）
 *
 * @param protocol 协议实例指针
 * @param data 二进制数据结构体指针，包含数据指针和大小
 *
 * 调用者：ws_sender_task
 */
void protocol_send_audio_data(protocol_t *protocol, binary_data_t *data)
{
    if (esp_websocket_client_is_connected(protocol->websocket_client))
    {
        // 实时音频发送：300ms 超时给 TCP 窗口抖动留出恢复时间，
        // 同时远短于会话 60s 超时，不会让 sender 长期阻塞。
        // （原 100ms 过激进，网络稍拥就触发 transport_poll_write returned 0）
        int ret = esp_websocket_client_send_bin(protocol->websocket_client, data->ptr, data->size,
                                                pdMS_TO_TICKS(1000));
        if (ret < 0)
        {
            // 发送失败通常由网络拥堵或底层缓冲区暂满引起；
            // 不在此强制断连，保活 ping 或接收端事件会触发重连逻辑
            ESP_LOGW(TAG, "WS 音频发送失败 (size=%d, ret=%d)，网络拥堵", data->size, ret);
        }
    }
}

/**
 * @brief 发送打断 TTS 指令（type=abort, reason=wake_word_detected）
 *
 * @param protocol 协议实例指针
 *
 * 调用者：session_on_wake_word（打断场景）
 */
void protocol_send_abort_speaking(protocol_t *protocol)
{
    // protocol_send_text(protocol, "{\"reason\":\"wake_word_detected\",\"session_id\":\"%s\",\"type\":\"abort\"}", protocol->session_id ? protocol->session_id : "");
    protocol_send_text(protocol, "{\"type\":\"cancel\"}");
}

/**
 * @brief 发送 IoT 消息（设备能力描述 / 状态上报）
 *
 * @param protocol 协议实例指针
 * @param type 消息类型（descriptors / states）
 * @param json cJSON 对象，函数内部会接管所有权并添加到消息中
 *
 * 调用者：应用层需要上报设备状态或能力时
 */
void protocol_send_iot(protocol_t *protocol, protocol_iot_message_type_t type, cJSON *json)
{
    /* 防止在断开连接时调用底层 API 导致报错 */
    if (!esp_websocket_client_is_connected(protocol->websocket_client))
    {
        return;
    }

    const char *type_str[] = {"descriptors", "states"};

    /* 构建完整 IoT JSON 消息 */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "session_id", protocol->session_id ? protocol->session_id : "");
    cJSON_AddStringToObject(root, "type", "iot");
    cJSON_AddBoolToObject(root, "update", cJSON_True);
    cJSON_AddItemToObject(root, type_str[type], json); // json 所有权转移给 root

    char *json_str = cJSON_PrintUnformatted(root);
    cJSON_Delete(root); // 同时释放 json

    esp_websocket_client_send_text(protocol->websocket_client, json_str, strlen(json_str), pdMS_TO_TICKS(10000));
    free(json_str);
}

// ─── 公开 API：回调注册 ────────────────────────────────────────────────────

/**
 * @brief 注册协议事件回调
 * 所有协议事件通过此回调分发给上层（session 模块）
 *
 * @param protocol 协议实例指针
 * @param callback 事件回调函数指针
 * @param handler_args 回调函数的用户自定义参数
 *
 * 调用者：session_init
 */
void protocol_register_callback(protocol_t *protocol, esp_event_handler_t callback, void *handler_args)
{
    protocol->callback = callback;
    protocol->handler_args = handler_args;
}