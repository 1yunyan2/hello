#pragma once

#include <stddef.h>
#include <stdint.h>
#include "esp_event.h"
#include "cJSON.h"

// 协议事件类型枚举
typedef enum
{
    PROTOCOL_EVENT_CONNECTED,          // 连接成功
    PROTOCOL_EVENT_DISCONNECTED,       // 连接断开
    PROTOCOL_EVENT_HELLO,              // 收到服务器 Hello 响应
    PROTOCOL_EVENT_STT,                // 收到语音识别结果 (event_data: char*)
    PROTOCOL_EVENT_LLM,                // 收到大模型回复 (event_data: char*)
    PROTOCOL_EVENT_TTS_START,          // TTS 开始播放
    PROTOCOL_EVENT_TTS_SENTENCE_START, // TTS 句子开始 (event_data: char*)
    PROTOCOL_EVENT_TTS_STOP,           // TTS 播放停止
    PROTOCOL_EVENT_AUDIO,              // 收到音频数据 (event_data: binary_data_t*)
    PROTOCOL_EVENT_IOT,                // 收到 IoT 控制指令 (event_data: cJSON*)
} protocol_event_t;

// 监听模式类型
typedef enum
{
    PROTOCOL_LISTEN_TYPE_AUTO,     // 自动模式
    PROTOCOL_LISTEN_TYPE_MANUAL,   // 手动模式
    PROTOCOL_LISTEN_TYPE_REALTIME, // 实时模式
} protocol_listen_type_t;

// IoT 消息类型
typedef enum
{
    MESSAGE_TYPE_DESCRIPTOR, // 描述符
    MESSAGE_TYPE_STATE,      // 状态
} protocol_iot_message_type_t;

// 二进制数据结构体
typedef struct
{
    void *ptr;
    size_t size;
} binary_data_t;

typedef struct protocol protocol_t;

protocol_t *protocol_create(const char *url, const char *token);
void protocol_destroy(protocol_t *protocol);

void protocol_connect(protocol_t *protocol);
void protocol_disconnect(protocol_t *protocol);
bool protocol_is_connected(protocol_t *protocol);

void protocol_send_hello(protocol_t *protocol);
void protocol_send_wake_word(protocol_t *protocol, const char *wake_word);
void protocol_send_start_listening(protocol_t *protocol, protocol_listen_type_t type);
void protocol_send_stop_listening(protocol_t *protocol);
void protocol_send_audio_data(protocol_t *protocol, binary_data_t *data);
void protocol_send_abort_speaking(protocol_t *protocol);
void protocol_send_iot(protocol_t *protocol, protocol_iot_message_type_t type, cJSON *json);

void protocol_register_callback(protocol_t *protocol, esp_event_handler_t callback, void *handler_args);