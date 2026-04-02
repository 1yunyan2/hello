#pragma once

/**
 * @brief 会话状态
 */
typedef enum
{
    SESSION_IDLE,      ///< 待机，唤醒词监听中
    SESSION_LISTENING, ///< 录音中，PCM→OPUS→WebSocket→云端
    SESSION_PLAYING,   ///< 播放云端返回的音频
} session_state_t;

/**
 * @brief 初始化会话模块（在 audio_init 之后、WiFi 连接之后调用）
 *
 * @param ws_uri  WebSocket 服务器地址，如 "ws://192.168.1.100:8080/audio"
 *                若为 NULL 则使用编译期默认地址
 */
void session_init(const char *ws_uri);

/**
 * @brief 唤醒词触发时调用（替换原 wake_word_callback 中的逻辑）
 * @param display  显示词，如 "云炎"
 */
void session_on_wake_word(const char *display);

/**
 * @brief 查询当前会话状态
 */
session_state_t session_get_state(void);
