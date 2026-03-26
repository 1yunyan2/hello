#pragma once

#include "bsp_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include "esp_http_client.h"
#include "esp_crt_bundle.h" // 用于 HTTPS 的根证书校验
#include "cJSON.h"
#include <string.h> // 🌟 新增头文件，因为后面用到了 strdup 和 strlen
#include "driver/gpio.h"
// #include "esp_bt.h"
#include "freertos/event_groups.h"

#include "esp_codec_dev.h" // 音频设备

// /// 设备状态位定义（用于 EventGroup）
// #define LED_BIT BIT0    ///< LED 初始化完成标志
// #define BUTTON_BIT BIT1 ///< 按钮初始化完成标志
// #define WIFI_BIT BIT2   ///< WiFi 连接成功标志
// #define NVS_BIT BIT3    ///< NVS 初始化完成标志
// #define CODEC_BIT BIT4  ///< 编解码器初始化完成标志
// #define LCD_BIT BIT5    ///< LCD 初始化完成标志

// 单实例
typedef struct
{
    esp_codec_dev_handle_t codec_dev; ///< 音频编解码器句柄

} bsp_board_t;

// bsp_board_t *bsp_board_get_instance(void);

void bsp_board_wifi_main(void);

// void bsp_board_codec_init(bsp_board_t *bsp_board);