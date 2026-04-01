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
#include "driver/i2s_std.h"

#include "nvs.h"
#include "esp_random.h"
// 设备状态位定义（统一使用 board_status EventGroup）
#define LED_BIT BIT0       ///< LED 初始化完成标志
#define BUTTON_BIT BIT1    ///< 按钮初始化完成标志
#define WIFI_BIT BIT2      ///< WiFi 连接成功标志
#define NVS_BIT BIT3       ///< NVS 初始化完成标志
#define CODEC_BIT BIT4     ///< 编解码器初始化完成标志
#define LCD_BIT BIT5       ///< LCD 初始化完成标志
#define WIFI_FAIL_BIT BIT6 ///< WiFi 连接最终失败标志
#define PROV_DONE_BIT BIT7 ///< BLE 配网流程结束标志

// 单实例
typedef struct
{
    EventGroupHandle_t board_status;  ///< 设备状态事件组句柄
    esp_codec_dev_handle_t codec_dev; ///< 音频编解码器句柄

} bsp_board_t;

bsp_board_t *bsp_board_get_instance(void);

void bsp_board_nvs_init(bsp_board_t *bsp_board);

void bsp_board_wifi_main(bsp_board_t *bsp_board);

void bsp_board_codec_init(bsp_board_t *bsp_board);

bool bsp_board_check_status(bsp_board_t *bsp_board, EventBits_t bits_to_check, TickType_t wait_ticks);

// 音频初始化与采集任务
void audio_init(bsp_board_t *bsp_board);
void audio_feed_task(void *arg);
