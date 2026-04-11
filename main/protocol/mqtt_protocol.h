#pragma once

/**
 * @file mqtt_protocol.h
 * @brief MQTT 设备管理协议接口（心跳上报、唤醒词热更新、重置通知）
 *
 * 本模块通过 MQTT 实现设备与云端的管理通道（非语音交互通道），功能包括：
 *   - 定期向 echopal/device/{id}/heartbeat 上报电量和 Wi-Fi 信号强度
 *   - 订阅 echopal/device/{id}/wake-word，接收云端下发的唤醒词更新指令
 *   - 在出厂重置后向 echopal/device/{id}/reset 发送通知
 *
 * MQTT 凭证优先从 NVS "mqtt_creds" 命名空间读取，未配置时回退到编译期默认值。
 * 设备 ID 由 Wi-Fi STA MAC 地址后三字节生成（6位十六进制）。
 *
 * @note 调用者：application.c → application_init()（Wi-Fi 连接成功后调用）
 */

#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/adc.h"
#include "wake_word/custom_wake_word.h"
#include "esp_timer.h"

/**
 * @brief 初始化并启动 MQTT 客户端
 *
 * 步骤：
 *   1. 从 NVS "mqtt_creds" 加载 Broker 地址、用户名、密码（失败则用编译期默认值）
 *   2. 创建并配置 esp_mqtt_client（URI、凭证）
 *   3. 注册 mqtt_event_handler（处理连接、数据、断开事件）
 *   4. 启动 MQTT 客户端（开始连接 Broker）
 *   5. 创建 heartbeat_task（独立任务，每 50s 上报一次设备状态）
 *
 * @return void
 *
 * @note 调用者：application.c → application_init()（Wi-Fi 就绪后调用）
 * @note MQTT 连接成功后，在 mqtt_event_handler 中自动订阅唤醒词更新主题
 */
void protocol_mqtt_start(void);

/**
 * @brief 初始化电源监测功能（预留接口，当前版本暂未实现）
 *
 * @return void
 *
 * @note 调用者：application.c（可选，按需调用）
 */
void power_monitor_init(void);

/**
 * @brief 向 MQTT 服务器发送设备重置通知
 *
 * 构建 JSON 消息 {"event":"factory_reset"}，以 QoS 1 发布到
 * echopal/device/{id}/reset 主题，确认云端感知到设备被重置。
 * 若 MQTT 客户端未就绪（未连接）则直接返回，不阻塞调用方。
 *
 * @return void
 *
 * @note 调用者：出厂重置逻辑（如长按按钮触发 NVS 擦除后调用）
 * @note 要求 MQTT 已连接（s_mqtt_connected == true）
 */
void send_reset_notification(void);
