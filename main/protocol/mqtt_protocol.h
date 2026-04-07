#pragma once

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
 * @brief 初始化并启动 MQTT 客户端，从 NVS 加载凭证，启动心跳任务
 */
void protocol_mqtt_start(void);

/**
 * @brief 初始化电源监测功能
 */
void power_monitor_init(void);

/**
 * @brief 发送设备重置通知到MQTT服务器
 */
void send_reset_notification(void);
