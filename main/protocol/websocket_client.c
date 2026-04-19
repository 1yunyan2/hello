/**
 * @file websocket_client.c
 * @brief WebSocket 协议层实现 — 消息收发与事件分发
 *
 * 本模块实现了设备与云端大模型之间的完整 WebSocket 通信协议：
 *
 * 上行（设备→云端）：
 *   - Hello 握手：协商音频参数，建立会话
 *   - 唤醒词通知：告知服务端触发的唤醒词
 *   - 监听控制：start / stop / detect
 *   - 音频帧：OPUS 编码的麦克风数据（Binary Frame）
 *   - 打断指令：用户说唤醒词打断 TTS
 *   - IoT 消息：设备能力和状态上报
 *
 * 下行（云端→设备）：
 *   - Hello 响应：返回 session_id
 *   - STT 结果：语音识别文本
 *   - LLM 状态：大模型情感标签
 *   - TTS 控制：start / stop / sentence_start
 *   - 音频帧：OPUS 编码的 TTS 合成语音（Binary Frame）
 *   - IoT 指令：远程控制命令
 */

#include "websocket_client.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "esp_crt_bundle.h"
#include "esp_mac.h"
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
    char *session_id;                               ///< 服务端分配的会话 ID（Hello 响应中获取）
    esp_event_handler_t callback;                   ///< 上层注册的事件回调函数
    void *handler_args;                             ///< 回调函数的用户自定义参数
};

// ─── 下行消息处理器 ────────────────────────────────────────────────────────
// 每个 handler 负责解析一种 type 的 JSON 消息，提取数据后通过回调通知上层

/**
 * @brief 处理服务端 Hello 响应
 * 提取 session_id 并保存，通知上层握手完成
 *
 * @param protocol 协议实例指针
 * @param root JSON根对象指针
 *
 * 调用者：protocol_websocket_event_handler中的WEBSOCKET_EVENT_DATA事件处理
 */
static void protocol_hello_handler(protocol_t *protocol, cJSON *root)
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

    /* 通知上层：Hello 握手完成 */
    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_HELLO, NULL);
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
        protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_STOP, NULL);
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
        /* opcode 0x02 = Binary Frame → 云端下发的 OPUS 音频帧 */
        if (data->op_code == 0x02)
        {
            binary_data_t bin = {.ptr = (void *)data->data_ptr, .size = data->data_len};
            protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_AUDIO, &bin);
            return;
        }

        /* opcode 0x01 = Text Frame → JSON 控制消息 */
        if (data->op_code == 0x01)
        {
            ESP_LOGI(TAG, "收到文本帧: %.*s", data->data_len, data->data_ptr);
            cJSON *root = cJSON_ParseWithLength(data->data_ptr, data->data_len);
            if (!root)
            {
                ESP_LOGW(TAG, "JSON 解析失败");
                return;
            }

            /* 根据 type 字段路由到对应的消息处理器 */
            cJSON *type = cJSON_GetObjectItem(root, "type");
            if (cJSON_IsString(type))
            {
                if (strcmp(type->valuestring, "started") == 0)
                    protocol_hello_handler(protocol, root);
                else if (strcmp(type->valuestring, "llm") == 0)
                    protocol_llm_handler(protocol, root);
                // else if (strcmp(type->valuestring, "stt") == 0)
                //     protocol_stt_handler(protocol, root);
                else if (strcmp(type->valuestring, "tts") == 0)
                    protocol_tts_handler(protocol, root);
                //  兼容你服务端的 tts_start
                else if (strcmp(type->valuestring, "tts_start") == 0)
                    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_START, NULL);
                //  兼容你服务端的 tts_end，这会触发上面我们刚写的方案A关机！
                else if (strcmp(type->valuestring, "tts_end") == 0)
                    protocol->callback(protocol->handler_args, PROTOCOL_EVENT, PROTOCOL_EVENT_TTS_STOP, NULL);
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
        // .crt_bundle_attach = esp_crt_bundle_attach, // HTTPS 根证书校验（wss:// 需要）
        .network_timeout_ms = 5000,     // 网络超时 5 秒
        .disable_auto_reconnect = true, // 禁用自动重连，由 session 层控制退避策略
        // 启用 Ping 保活：防止 NAT/路由 idle 断链产生"幽灵连接"，
        // 也让客户端能及时感知服务端/网络断开并触发重连（修复 transport_poll_write returned 0）
        .ping_interval_sec = 20,    // 20s 发一次 ping，维持 NAT 连接活跃
        .pingpong_timeout_sec = 10, // ping 后 10s 未收到 pong 则判定断开
        .buffer_size = 8192, //! 增加了缓存，防止接收数据过大 默认接收缓冲区大小是 1024 字节
    };

    /* 初始化底层 WebSocket 客户端并注册事件回调 */
    protocol->websocket_client = esp_websocket_client_init(&websocket_cfg);
    // 注册事件回调，监听所有 WebSocket 事件（WEBSOCKET_EVENT_ANY），传递协议实例指针作为参数
    esp_websocket_register_events(protocol->websocket_client, WEBSOCKET_EVENT_ANY, protocol_websocket_event_handler, protocol);

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
 * @brief 发送 Hello 握手消息
 * 协商音频参数：单声道 / OPUS 编码 / 60ms 帧时长 / 16kHz 采样率
 *
 * @param protocol 协议实例指针
 *
 * 调用者：session_on_wake_word、PROTOCOL_EVENT_CONNECTED事件处理
 */
void protocol_send_hello(protocol_t *protocol)
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
                                                pdMS_TO_TICKS(300));
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
    protocol_send_text(protocol, "{\"reason\":\"wake_word_detected\",\"session_id\":\"%s\",\"type\":\"abort\"}", protocol->session_id ? protocol->session_id : "");
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