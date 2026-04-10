#pragma once

/**
 * @file bsp_board.h
 * @brief 板级支持包（BSP）— 硬件抽象层核心头文件
 *
 * 本模块是整个硬件层的统一入口，提供三大能力：
 *   1. 全局唯一的 BSP 单例（bsp_board_t），各模块通过它共享硬件句柄
 *   2. 各子系统初始化函数（NVS / WiFi / Codec / LCD）
 *   3. 基于 FreeRTOS EventGroup 的跨模块状态同步机制
 *
 * 启动顺序约束（由 application.c 严格保证，不可调换）：
 *   bsp_board_get_instance()
 *     → bsp_board_nvs_init()        [NVS_BIT]
 *     → bsp_wake_word_init()        [引擎就绪]
 *     → audio_init()                [CODEC_BIT + 采集任务]
 *     → bsp_board_wifi_main()       [WIFI_BIT]
 *     → protocol_mqtt_start()       [MQTT连接]
 *     → session_init()              [WebSocket预连接]
 *
 * 跨模块状态同步模型：
 *   各模块完成初始化时通过 xEventGroupSetBits() 置位对应 BIT；
 *   依赖某模块就绪的代码使用 bsp_board_check_status() 或
 *   xEventGroupWaitBits() 阻塞等待，确保无竞争启动。
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
#include "esp_crt_bundle.h"
#include "cJSON.h"
#include <string.h>
#include "driver/gpio.h"
#include "freertos/event_groups.h"
#include "esp_codec_dev.h"
#include "driver/i2s_std.h"
#include "nvs.h"
#include "esp_random.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"

// ─── 设备状态位定义（统一使用 board_status EventGroup）─────────────────────
// 各模块完成初始化或达到特定状态时置位对应 BIT，其他模块通过 WaitBits 同步等待。
// 使用规范：
//   置位 → xEventGroupSetBits(bsp->board_status, XXX_BIT)
//   等待 → bsp_board_check_status(bsp, XXX_BIT, timeout) 或直接 xEventGroupWaitBits

#define LED_BIT       BIT0 ///< LED 初始化完成（当前未使用）
#define BUTTON_BIT    BIT1 ///< 按钮初始化完成（当前未使用）
#define WIFI_BIT      BIT2 ///< WiFi 连接成功且已获取有效 IP 地址
#define NVS_BIT       BIT3 ///< NVS Flash 初始化完成，可读写非易失配置
#define CODEC_BIT     BIT4 ///< ES8311 音频编解码器初始化完成，可开始录音/播放
#define LCD_BIT       BIT5 ///< LCD 显示屏初始化完成（当前未自动置位）
#define WIFI_FAIL_BIT BIT6 ///< WiFi 连接彻底失败（超过最大重试次数），系统将重启
#define PROV_DONE_BIT BIT7 ///< BLE 配网流程结束（无论成功/超时），解除配网阻塞

// ─── BSP 全局单例结构体 ───────────────────────────────────────────────────────

/**
 * @brief 板级支持包全局实例结构体
 *
 * 全局唯一，通过 bsp_board_get_instance() 获取。
 * 各子系统将自己的句柄挂载到此结构体中，实现跨模块安全共享。
 *
 * 字段访问规范：
 *   - board_status : 仅通过 xEventGroupSetBits / xEventGroupClearBits / xEventGroupWaitBits 操作
 *   - codec_dev    : 初始化后只读，通过 esp_codec_dev_read/write 操作音频数据
 *   - lcd_io/panel : 初始化后只读，通过 esp_lcd_panel_* API 操作显示
 */
typedef struct
{
    EventGroupHandle_t board_status;   ///< 设备状态事件组（FreeRTOS），各模块通过此实现就绪同步
    esp_codec_dev_handle_t codec_dev;  ///< ES8311 音频编解码器设备句柄，由 audio_init() 填充
    esp_lcd_panel_io_handle_t lcd_io;  ///< LCD SPI 传输接口句柄，由 bsp_board_lcd_init() 填充
    esp_lcd_panel_handle_t lcd_panel;  ///< LCD ST7789 面板驱动句柄，由 bsp_board_lcd_init() 填充
} bsp_board_t;

// ─── 公开 API：生命周期管理 ───────────────────────────────────────────────────

/**
 * @brief 获取全局唯一 BSP 单例实例
 *
 * 首次调用时自动创建 FreeRTOS EventGroup（board_status），后续调用返回同一指针。
 * 线程安全：因首次调用在 app_main 单线程阶段，无竞争风险。
 *
 * @return bsp_board_t* 全局 BSP 实例指针，永远不为 NULL
 *
 * @note 调用者：application.c → application_init()（第一步调用）
 */
bsp_board_t *bsp_board_get_instance(void);

/**
 * @brief 初始化 NVS Flash（非易失存储）
 *
 * NVS 存储 WiFi 凭证、MQTT 凭证、唤醒词、accessToken 等配置。
 * 若分区表损坏或版本不兼容则自动擦除重建（丢失已存储配置）。
 * 完成后置位 NVS_BIT，通知依赖 NVS 的模块（如 WiFi）可以安全读写。
 *
 * @param bsp_board BSP 实例指针（必须先调用 bsp_board_get_instance()）
 *
 * @note 调用者：application.c → application_init()（步骤 2）
 * @note 必须在所有使用 NVS 的模块之前调用
 */
void bsp_board_nvs_init(bsp_board_t *bsp_board);

/**
 * @brief WiFi 主初始化入口（阻塞直至网络就绪或彻底失败）
 *
 * 内部完整流程：
 *   1. 初始化 TCP/IP 协议栈和事件循环
 *   2. 检查 NVS 中是否存有 WiFi 凭证
 *   3a. 无凭证 → 启动 BLE 配网广播，等待 App 推送 SSID + 密码（120s 超时）
 *   3b. 有凭证 → 直接 STA 模式连接，自动重连最多 5 次
 *   4. 阻塞等待 WIFI_BIT 或 WIFI_FAIL_BIT 任一置位后返回
 *   5. WIFI_FAIL_BIT 置位时等待 30s 后自动重启（给用户看日志）
 *
 * 函数返回时保证：WIFI_BIT 或 WIFI_FAIL_BIT 必有一个已置位。
 *
 * @param bsp_board BSP 实例指针
 *
 * @note 调用者：application.c → application_init()（步骤 5）
 * @note 前置条件：NVS_BIT 必须已置位（WiFi 凭证存在 NVS）
 * @note 此函数是阻塞的，可能耗时数秒至数分钟
 */
void bsp_board_wifi_main(bsp_board_t *bsp_board);

/**
 * @brief 初始化 ES8311 音频编解码器硬件（I2C + I2S + Codec 驱动）
 *
 * 内部步骤：
 *   1. 创建 I2C 主机总线（ES8311 控制接口，SDA=8/SCL=15）
 *   2. 注册 ES8311 I2C 控制接口
 *   3. 创建 GPIO 控制接口（PA 使能等）
 *   4. 初始化 ES8311 Codec 驱动（全双工：同时支持录音和播放）
 *   5. 创建 I2S 双向通道（TX/RX，Philips 标准模式，16kHz/16bit/单声道）
 *   6. 创建顶层音频设备句柄并挂载到 bsp_board->codec_dev
 *   7. 置位 CODEC_BIT
 *
 * @param bsp_board BSP 实例指针，codec_dev 字段由此函数填充
 *
 * @note 调用者：bsp_codec.c → audio_init()（内部调用，外部勿直接使用）
 * @note 此函数在 audio_init() 内部自动调用，外部直接调用 audio_init() 即可
 */
void bsp_board_codec_init(bsp_board_t *bsp_board);

/**
 * @brief 检查指定状态位是否全部就绪（AND 等待）
 *
 * 封装 xEventGroupWaitBits() 的 AND 模式（所有位都满足才返回）。
 * 可指定超时时间：0 = 立即检查不等待，portMAX_DELAY = 永久等待。
 *
 * @param bsp_board      BSP 实例指针
 * @param bits_to_check  需要检查的位掩码（多个位用 | 组合，如 NVS_BIT | WIFI_BIT）
 * @param wait_ticks     等待超时（FreeRTOS tick 数），0 立即返回，portMAX_DELAY 永久
 * @return true  所有指定位均已置位
 * @return false 超时，部分位尚未置位
 *
 * @note 调用者：bsp_wifi.c（检查 NVS_BIT 前置条件）
 */
bool bsp_board_check_status(bsp_board_t *bsp_board, EventBits_t bits_to_check, TickType_t wait_ticks);

// ─── 公开 API：音频初始化 ─────────────────────────────────────────────────────

/**
 * @brief 完整音频初始化：硬件初始化 + 打开设备 + 创建麦克风采集任务
 *
 * 内部步骤：
 *   1. 调用 bsp_board_codec_init()：初始化 I2C+I2S+ES8311 硬件
 *   2. esp_codec_dev_open()：打开音频设备，配置 16kHz/16bit/单声道
 *   3. 设置麦克风增益（40 ≈ 20dB）和扬声器音量（60/100）
 *   4. xTaskCreatePinnedToCore(audio_feed_task, CPU1)：启动麦克风采集任务
 *
 * 必须在 bsp_wake_word_init() 之后调用，因为采集任务启动后立即向引擎投喂音频。
 *
 * @param bsp_board BSP 实例指针
 *
 * @note 调用者：application.c → application_init()（步骤 4）
 * @note 前置条件：bsp_wake_word_init() 已完成（AFE/MultiNet 引擎就绪）
 */
void audio_init(bsp_board_t *bsp_board);

/**
 * @brief 麦克风采集任务（由 audio_init() 内部创建，不可手动调用）
 *
 * 任务主循环：
 *   while(1) {
 *     esp_codec_dev_read(codec_dev, buffer, chunk_size)  // 从 I2S DMA 读 PCM
 *     custom_wake_word_feed(buffer, chunk_size)           // 投喂给 AFE
 *   }
 *
 * AFE 内部处理：降噪(NS) → VAD → enhanced_pcm_hook → MultiNet 命令词检测
 *
 * @param arg bsp_board_t* 实例指针
 *
 * @note 运行核心：CPU1（与 WiFi 栈隔离，避免优先级竞争）
 * @note 栈大小：8192 字节，优先级：5
 * @note 调用者：audio_init() 通过 xTaskCreatePinnedToCore() 自动创建
 */
void audio_feed_task(void *arg);

// ─── 公开 API：LCD 显示控制 ───────────────────────────────────────────────────

/**
 * @brief 初始化 LCD 显示屏（SPI 总线 + ST7789 驱动）
 *
 * 内部步骤：
 *   1. 配置背光 GPIO（初始关闭）
 *   2. 初始化 SPI2 总线（80MHz，DMA 自动）
 *   3. 创建 SPI LCD 通信接口（DC/CS/80MHz）
 *   4. 初始化 ST7789 面板驱动（240×320，RGB565，颜色反转）
 *   5. 复位并初始化面板（关闭显示，等待上层主动开启）
 *
 * @param bsp_board BSP 实例指针，lcd_io/lcd_panel 字段由此函数填充
 *
 * @note 调用者：application.c（当前注释掉，LCD 功能预留）
 */
void bsp_board_lcd_init(bsp_board_t *bsp_board);

/**
 * @brief 打开 LCD 背光和显示
 *
 * 调用 esp_lcd_panel_disp_on_off(true) 启用显示，然后 GPIO 拉高背光。
 *
 * @param bsp_board BSP 实例指针
 *
 * @note 前置条件：bsp_board_lcd_init() 已调用
 */
void bsp_board_lcd_on(bsp_board_t *bsp_board);

/**
 * @brief 关闭 LCD 背光和显示
 *
 * GPIO 拉低背光，然后调用 esp_lcd_panel_disp_on_off(false) 关闭显示。
 *
 * @param bsp_board BSP 实例指针
 *
 * @note 前置条件：bsp_board_lcd_init() 已调用
 */
void bsp_board_lcd_off(bsp_board_t *bsp_board);
