#pragma once

#include <stddef.h>
#include "esp_err.h"
#include <stdbool.h>

/**
 * @brief WebSocket 接收数据回调（在 WebSocket 事件任务中调用，勿阻塞）
 * @param data  收到的二进制数据指针（OPUS 帧）
 * @param len   数据字节数
 */
typedef void (*ws_receive_cb_t)(const void *data, size_t len);

/**
 * @brief 初始化并连接 WebSocket
 * @param uri        服务器地址，如 "ws://192.168.1.100:8080/audio"
 * @param receive_cb 收到数据时调用的回调，传 NULL 则不处理接收
 * @return ESP_OK 成功，ESP_FAIL 连接失败
 */
esp_err_t ws_client_start(const char *uri, ws_receive_cb_t receive_cb);

/**
 * @brief 断开 WebSocket 并释放资源
 */
void ws_client_stop(void);

/**
 * @brief 发送二进制帧（OPUS 数据）
 * @param data 数据指针
 * @param len  字节数
 * @return ESP_OK 成功，ESP_FAIL 未连接或发送失败
 */
esp_err_t ws_client_send_binary(const void *data, size_t len);

/**
 * @brief 查询当前是否已连接
 */
bool ws_client_is_connected(void);