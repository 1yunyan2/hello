#pragma once

/**
 * @file bsp_board.h（精简版）
 * @brief 板级支持包 — audio_test_standalone 专用最小子集
 *
 * 与主工程 bsp_board.h 的关系：
 *   主工程版本一次性声明了 LCD/舵机/触摸/电池/WiFi 等全部硬件 API；
 *   本版本只保留音频测试需要的部分：BSP 单例 + NVS + Codec + 状态位。
 *
 * 提取自：main/bsp/bsp_board.h（原文件 ~430 行 → 本版 ~80 行）
 */

#include "bsp_config.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/event_groups.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_event.h>
#include <string.h>
#include "driver/gpio.h"
#include "esp_codec_dev.h"
#include "driver/i2s_std.h"
#include "nvs.h"
#include "esp_heap_caps.h"

// ─── 设备状态位定义（仅保留音频相关）────────────────────────────────────────
#define NVS_BIT BIT3   ///< NVS Flash 初始化完成
#define CODEC_BIT BIT4 ///< ES8311 音频编解码器初始化完成

// ─── BSP 全局单例结构体（精简版）────────────────────────────────────────────
/**
 * @brief 板级支持包全局实例结构体（音频测试专用最小集）
 *
 * 仅保留音频链路用到的字段，去掉 LCD/舵机/wifi 等字段，避免拖入额外驱动依赖。
 */
typedef struct
{
    EventGroupHandle_t board_status;  ///< 设备状态事件组（FreeRTOS）
    esp_codec_dev_handle_t codec_dev; ///< ES8311 音频编解码器设备句柄
    i2s_chan_handle_t i2s_tx_handle;  ///< I2S TX 通道句柄（播放）
    i2s_chan_handle_t i2s_rx_handle;  ///< I2S RX 通道句柄（录音诊断用）
} bsp_board_t;

// ─── 公开 API ─────────────────────────────────────────────────────────────────

/** @brief 获取全局唯一 BSP 单例实例 */
bsp_board_t *bsp_board_get_instance(void);

/** @brief 初始化 NVS Flash，完成后置位 NVS_BIT */
void bsp_board_nvs_init(bsp_board_t *bsp_board);

/** @brief 检查指定状态位是否全部就绪（AND 等待） */
bool bsp_board_check_status(bsp_board_t *bsp_board, EventBits_t bits_to_check, TickType_t wait_ticks);

/** @brief ES8311 硬件初始化（I2C + I2S + Codec 驱动），audio_init 内部调用 */
void bsp_board_codec_init(bsp_board_t *bsp_board);

/** @brief 完整音频初始化：硬件 + 打开设备 + 设置增益。采集任务由 main.c 负责创建 */
void audio_init(bsp_board_t *bsp_board);
