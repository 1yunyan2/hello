#pragma once

/**
 * @file websocket_client.h
 * @brief WebSocket 协议层头文件 — 设备与云端 AI 的通信协议定义
 *
 * 本模块封装了设备与云端大模型之间的 WebSocket 双向通信协议，包括：
 *   - 文本消息（JSON）：Hello 握手、唤醒词通知、监听控制、TTS/STT/LLM 事件
 *   - 二进制消息：OPUS 音频帧的上行（麦克风→云端）和下行（云端→扬声器）
 *
 * 协议消息格式（JSON text frame）：
 *   上行: {"type": "hello/listen/abort/iot", ...}
 *   下行: {"type": "hello/stt/llm/tts/iot", ...}
 *
 * 协议消息格式（Binary frame）：
 *   上行: OPUS 编码音频帧（设备麦克风采集）
 *   下行: OPUS 编码音频帧（云端 TTS 合成）
 */

#include <stddef.h>
#include <stdint.h>
#include "esp_event.h"
#include "cJSON.h"

// ─── 协议事件类型枚举 ──────────────────────────────────────────────────────
// 由 WebSocket 消息解析后分发给 session 层的事件回调

typedef enum
{
    PROTOCOL_EVENT_CONNECTED,          // WebSocket 底层连接成功（TCP + TLS 握手完成）
    PROTOCOL_EVENT_DISCONNECTED,       // WebSocket 连接断开（网络异常/服务端关闭）
    PROTOCOL_EVENT_HELLO,              // 收到服务器 Hello 响应（会话握手成功，分配了 session_id）
    PROTOCOL_EVENT_STT,                // 收到语音识别结果 (event_data: char* 识别文本)
    PROTOCOL_EVENT_LLM,                // 收到大模型情感状态 (event_data: char* 情感标签)
    PROTOCOL_EVENT_TTS_START,          // TTS 开始播放（服务端开始推送音频）
    PROTOCOL_EVENT_TTS_SENTENCE_START, // TTS 句子开始 (event_data: char* 当前句子文本)
    PROTOCOL_EVENT_TTS_STOP,           // TTS 播放结束（服务端推送完毕）
    PROTOCOL_EVENT_AUDIO,              // 收到音频二进制数据 (event_data: binary_data_t*)
    PROTOCOL_EVENT_IOT,                // 收到 IoT 控制指令 (event_data: cJSON* 命令列表)
    PROTOCOL_EVENT_ERROR,              // <-- 新增：错误事件
    PROTOCOL_EVENT_COMPLETE            // <-- 新增：完成事件
} protocol_event_t;

// ─── 监听模式类型 ──────────────────────────────────────────────────────────

typedef enum
{
    PROTOCOL_LISTEN_TYPE_AUTO,     // 自动模式：服务端根据 VAD 判断说话结束
    PROTOCOL_LISTEN_TYPE_MANUAL,   // 手动模式：客户端显式发送 stop 结束录音
    PROTOCOL_LISTEN_TYPE_REALTIME, // 实时模式：持续推送，不等说话结束
} protocol_listen_type_t;

// ─── IoT 消息类型 ──────────────────────────────────────────────────────────

typedef enum
{
    MESSAGE_TYPE_DESCRIPTOR, // 设备能力描述符（告诉服务端设备支持哪些 IoT 操作）
    MESSAGE_TYPE_STATE,      // 设备状态上报（如灯光开关、音量等当前状态）
} protocol_iot_message_type_t;

// ─── 二进制数据封装 ────────────────────────────────────────────────────────

/**
 * @brief 二进制数据结构体（用于音频帧传递）
 * 不持有内存所有权，ptr 指向的数据由调用方管理生命周期
 */
typedef struct
{
    void *ptr;   ///< 数据指针（OPUS 音频帧首地址）
    size_t size; ///< 数据大小（字节）
} binary_data_t;

// ─── 协议实例（前向声明，内部结构在 .c 中定义）────────────────────────────
typedef struct protocol protocol_t;

// ─── 生命周期管理 ──────────────────────────────────────────────────────────

/**
 * @brief 创建 WebSocket 协议实例
 * 分配内存，配置连接参数（URL / Bearer Token / Device-Id），但不立即连接
 * @param url   WebSocket 服务器地址（如 "wss://api.example.com/v1/"）
 * @param token Bearer Token（accessToken），用于 Authorization 头认证，可为 NULL
 * @return protocol_t* 协议实例指针
 */
protocol_t *protocol_create(const char *url, const char *token);

/**
 * @brief 销毁协议实例，释放所有资源
 * @param protocol 协议实例指针
 */
void protocol_destroy(protocol_t *protocol);

// ─── 连接控制 ──────────────────────────────────────────────────────────────

/**
 * @brief 建立 WebSocket 连接（含 TLS 握手）
 * 若已连接则无操作。连接成功后触发 PROTOCOL_EVENT_CONNECTED 事件。
 * @param protocol 协议实例指针
 */
void protocol_connect(protocol_t *protocol);

/**
 * @brief 断开 WebSocket 连接
 * @param protocol 协议实例指针
 */
void protocol_disconnect(protocol_t *protocol);

/**
 * @brief 查询当前连接状态
 * @param protocol 协议实例指针
 * @return true 已连接，false 未连接
 */
bool protocol_is_connected(protocol_t *protocol);

// ─── 消息发送 ──────────────────────────────────────────────────────────────

/**
 * @brief 发送 Hello 握手消息
 * 协商音频参数（16kHz / OPUS / 60ms帧），服务端返回 session_id
 * @param protocol 协议实例指针
 */
void protocol_send_hello(protocol_t *protocol);

/**
 * @brief 发送唤醒词通知
 * 告诉服务端用户说了哪个唤醒词（用于多唤醒词场景的个性化响应）
 * @param protocol   协议实例指针
 * @param wake_word  唤醒词显示文本（如"云炎"）
 */
void protocol_send_wake_word(protocol_t *protocol, const char *wake_word);

/**
 * @brief 发送开始监听指令
 * 通知服务端设备开始推送音频，指定监听模式
 * @param protocol 协议实例指针
 * @param type     监听模式（auto / manual / realtime）
 */
void protocol_send_start_listening(protocol_t *protocol, protocol_listen_type_t type);

/**
 * @brief 发送停止监听指令
 * 通知服务端用户已说完，可以开始 ASR + LLM 处理
 * @param protocol 协议实例指针
 */
void protocol_send_stop_listening(protocol_t *protocol);

/**
 * @brief 发送音频二进制数据帧
 * 将编码后的 OPUS 帧通过 WebSocket Binary Frame 推送到云端
 * @param protocol 协议实例指针
 * @param data     二进制数据封装（ptr + size）
 */
void protocol_send_audio_data(protocol_t *protocol, binary_data_t *data);

/**
 * @brief 发送打断 TTS 指令
 * 用户唤醒词打断 AI 说话时调用，服务端收到后停止推送 TTS 音频
 * @param protocol 协议实例指针
 */
void protocol_send_abort_speaking(protocol_t *protocol);

/**
 * @brief 发送 IoT 控制消息
 * 上报设备能力描述或当前状态
 * @param protocol 协议实例指针
 * @param type     消息类型（描述符 / 状态）
 * @param json     cJSON 对象（函数内部会接管所有权，调用方无需释放）
 */
void protocol_send_iot(protocol_t *protocol, protocol_iot_message_type_t type, cJSON *json);

// ─── 回调注册 ──────────────────────────────────────────────────────────────

/**
 * @brief 注册协议事件回调
 * 所有协议事件（连接/断开/Hello/STT/LLM/TTS/Audio/IoT）通过此回调分发
 * @param protocol     协议实例指针
 * @param callback     事件处理函数
 * @param handler_args 传递给回调的用户自定义参数（通常为 NULL）
 */
void protocol_register_callback(protocol_t *protocol, esp_event_handler_t callback, void *handler_args);
