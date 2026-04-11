#pragma once

/**
 * @file websocket_client.h
 * @brief WebSocket 协议层头文件 — 设备与云端 AI 的通信协议定义
 *
 * 职责：封装基于 JSON 与 Binary 的 WebSocket 双向通信机制。
 */

#include <stddef.h>
#include <stdint.h>
#include "esp_event.h"
#include "cJSON.h"

/**
 * @brief 协议事件类型枚举
 * 用于在 WS 数据解析后将具体事件回调给会话管理器。
 */
typedef enum
{
    PROTOCOL_EVENT_CONNECTED,          ///< WebSocket 底层连接成功
    PROTOCOL_EVENT_DISCONNECTED,       ///< WebSocket 连接断开
    PROTOCOL_EVENT_HELLO,              ///< 收到服务器 Hello(started) 响应
    PROTOCOL_EVENT_STT,                ///< 收到 STT 文字结果 (数据为 char*)
    PROTOCOL_EVENT_LLM,                ///< 收到 LLM 状态 (数据为 char*)
    PROTOCOL_EVENT_TTS_START,          ///< TTS 音频开始播放事件
    PROTOCOL_EVENT_TTS_SENTENCE_START, ///< TTS 句子开始播报 (数据为 char*)
    PROTOCOL_EVENT_TTS_STOP,           ///< TTS 播放结束事件
    PROTOCOL_EVENT_AUDIO,              ///< 收到下行音频数据 (数据为 binary_data_t*)
    PROTOCOL_EVENT_IOT,                ///< 收到 IoT 遥测控制指令 (数据为 cJSON*)
    PROTOCOL_EVENT_ERROR,              ///< 收到服务端错误报警事件
    PROTOCOL_EVENT_COMPLETE            ///< 会话完成关闭事件
} protocol_event_t;

/**
 * @brief 云端控制的录音监听模式枚举
 */
typedef enum
{
    PROTOCOL_LISTEN_TYPE_AUTO,     ///< 自动模式（VAD 由云端辅助）
    PROTOCOL_LISTEN_TYPE_MANUAL,   ///< 手动模式
    PROTOCOL_LISTEN_TYPE_REALTIME, ///< 实时推理模式
} protocol_listen_type_t;

/**
 * @brief IoT 消息的类别标识枚举
 */
typedef enum
{
    MESSAGE_TYPE_DESCRIPTOR, ///< 描述符类型
    MESSAGE_TYPE_STATE,      ///< 状态类型
} protocol_iot_message_type_t;

/**
 * @brief 音频二进制数据包封装
 * 用于给上层携带不具有所有权的指针。
 */
typedef struct
{
    void *ptr;   ///< 指向 Opus 帧起始地址
    size_t size; ///< 帧长度（字节数）
} binary_data_t;

/** @brief 协议实例（不透明指针） */
typedef struct protocol protocol_t;

/**
 * @brief 创建 WebSocket 协议实例
 * @param[in] url   WebSocket 服务器地址
 * @param[in] token HTTP 头携带的 Auth Token 认证参数
 * @return protocol_t* 实例指针
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
protocol_t *protocol_create(const char *url, const char *token);

/**
 * @brief 销毁协议实例
 * @param[in] protocol 实例句柄
 * @return 无
 * @note 调用者：session.c -> session_reconnect_task()
 */
void protocol_destroy(protocol_t *protocol);

/**
 * @brief 主动连接 WebSocket 服务器
 * @param[in] protocol 实例句柄
 * @return 无
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
void protocol_connect(protocol_t *protocol);

/**
 * @brief 断开 WebSocket 服务器连接
 * @param[in] protocol 实例句柄
 * @return 无
 * @note 调用者：session.c -> session_reconnect_task()
 */
void protocol_disconnect(protocol_t *protocol);

/**
 * @brief 查询 WebSocket 当前是否处于连接状态
 * @param[in] protocol 实例句柄
 * @return bool true表示在线，false表示离线
 * @note 调用者：session.c
 */
bool protocol_is_connected(protocol_t *protocol);

/**
 * @brief 发送设备的协议握手消息
 * @param[in] protocol 实例句柄
 * @return 无
 * @note 调用者：session.c
 */
void protocol_send_hello(protocol_t *protocol);

/**
 * @brief 发送打断 TTS 指令
 * @param[in] protocol 实例句柄
 * @return 无
 * @note 调用者：session.c -> session_on_wake_word()
 */
void protocol_send_abort_speaking(protocol_t *protocol);

/**
 * @brief 发送停止监听/发音完毕信令
 * @param[in] protocol 实例句柄
 * @return 无
 * @note 调用者：session.c -> session_event_task()
 */
void protocol_send_stop_listening(protocol_t *protocol);

/**
 * @brief 发送上行音频 Binary 帧
 * @param[in] protocol 实例句柄
 * @param[in] data     包含 PCM 编码后的二进制指针及长度
 * @return 无
 * @note 调用者：session.c -> ws_sender_task()
 */
void protocol_send_audio_data(protocol_t *protocol, binary_data_t *data);

/**
 * @brief 注册协议底层解析回调，将事件路由给上层状态机
 * @param[in] protocol     实例句柄
 * @param[in] callback     上层提供的 esp_event_handler_t 事件回调
 * @param[in] handler_args 自定义附加参数
 * @return 无
 * @note 调用者：session.c -> session_init()
 */
void protocol_register_callback(protocol_t *protocol, esp_event_handler_t callback, void *handler_args);