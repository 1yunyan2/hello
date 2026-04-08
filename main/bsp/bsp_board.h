#pragma once

/**
 * @file bsp_board.h
 * @brief 板级支持包（BSP）— 硬件抽象与全局状态管理
 *
 * 本模块是整个硬件层的入口，提供：
 *   1. 全局唯一的 BSP 单例实例（bsp_board_t）
 *   2. 各子系统初始化函数（NVS / WiFi / Codec / 音频采集）
 *   3. 基于 FreeRTOS EventGroup 的跨模块状态同步机制
 *
 * 启动顺序约束（由 application.c 保证）：
 *   bsp_board_get_instance → nvs_init → wake_word_init → audio_init → wifi_main → mqtt → session
 */

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
#include <string.h>
#include "driver/gpio.h"
// #include "esp_bt.h"
#include "freertos/event_groups.h"

#include "esp_codec_dev.h" // 音频设备
#include "driver/i2s_std.h"

#include "nvs.h"
#include "esp_random.h"

// ─── 设备状态位定义（统一使用 board_status EventGroup）─────────────────────
// 各模块完成初始化或达到特定状态时置位对应 BIT，其他模块通过 WaitBits 同步等待
#define LED_BIT BIT0       ///< LED 初始化完成标志
#define BUTTON_BIT BIT1    ///< 按钮初始化完成标志
#define WIFI_BIT BIT2      ///< WiFi 连接成功（已获取 IP）
#define NVS_BIT BIT3       ///< NVS 初始化完成标志
#define CODEC_BIT BIT4     ///< 音频编解码器（ES8311）初始化完成
#define LCD_BIT BIT5       ///< LCD 初始化完成标志
#define WIFI_FAIL_BIT BIT6 ///< WiFi 连接最终失败（超过最大重试次数）
#define PROV_DONE_BIT BIT7 ///< BLE 配网流程结束（无论成功或超时）

// ─── BSP 单例结构体 ────────────────────────────────────────────────────────

/**
 * @brief 板级支持包全局实例
 *
 * 全局唯一，通过 bsp_board_get_instance() 获取。
 * 各子系统将自己的句柄挂载到此结构体上，实现跨模块共享。
 */
typedef struct
{
    EventGroupHandle_t board_status;  ///< 设备状态事件组（各模块通过此同步就绪状态）
    esp_codec_dev_handle_t codec_dev; ///< 音频编解码器设备句柄（ES8311，全双工录放）
} bsp_board_t;

// ─── 公开 API ──────────────────────────────────────────────────────────────

/**
 * @brief 获取全局唯一 BSP 单例实例
 * 首次调用时自动创建 EventGroup，后续调用返回同一指针
 * @return bsp_board_t* 全局 BSP 实例指针
 */
bsp_board_t *bsp_board_get_instance(void);

/**
 * @brief 初始化 NVS Flash（非易失存储）
 * NVS 存储 WiFi 凭证、MQTT 凭证、唤醒词等配置。
 * 若分区表损坏则自动擦除重建。完成后置位 NVS_BIT。
 * @param bsp_board BSP 实例指针
 */
void bsp_board_nvs_init(bsp_board_t *bsp_board);

/**
 * @brief WiFi 主初始化入口（阻塞直到网络就绪或失败）
 * 包含完整流程：BLE 配网 / 自动重连 / 按键重置 / 超时重启
 * 返回时 WIFI_BIT 或 WIFI_FAIL_BIT 必有一个被置位
 * @param bsp_board BSP 实例指针
 */
void bsp_board_wifi_main(bsp_board_t *bsp_board);

/**
 * @brief 初始化音频编解码器硬件（I2C + I2S + ES8311）
 * 完成后置位 CODEC_BIT，codec_dev 句柄挂载到 bsp_board
 * @param bsp_board BSP 实例指针
 */
void bsp_board_codec_init(bsp_board_t *bsp_board);

/**
 * @brief 检查指定状态位是否全部就绪
 * @param bsp_board      BSP 实例指针
 * @param bits_to_check  需要检查的位掩码（如 NVS_BIT | WIFI_BIT）
 * @param wait_ticks     等待超时（0 = 立即返回，portMAX_DELAY = 永久等待）
 * @return true 所有指定位均已置位，false 超时未满足
 */
bool bsp_board_check_status(bsp_board_t *bsp_board, EventBits_t bits_to_check, TickType_t wait_ticks);

// ─── 音频采集 ──────────────────────────────────────────────────────────────

/**
 * @brief 完整音频初始化：硬件初始化 + 打开设备 + 创建采集任务
 * 必须在唤醒词引擎初始化之后调用（采集任务会立即向引擎投喂音频帧）
 * @param bsp_board BSP 实例指针
 */
void audio_init(bsp_board_t *bsp_board);

/**
 * @brief 麦克风采集任务入口（由 audio_init 内部创建，无需手动调用）
 * 持续从 I2S 读取 PCM 数据，投喂给唤醒词引擎 + PCM 钩子
 * @param arg bsp_board_t* 实例指针
 */
void audio_feed_task(void *arg);

// audio_set_pcm_hook 已移除：PCM 数据改由 AFE 降噪后通过
// bsp_wake_word_set_enhanced_pcm_hook → custom_wake_word_feed 内部 fetch 输出
