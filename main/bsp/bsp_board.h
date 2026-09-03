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
 *     → wake_word_init()        [引擎就绪]
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
#include "driver/i2c_master.h"
#include "nvs.h"
#include "esp_random.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_dev.h"
#include "iot_servo.h"
#include "esp_heap_caps.h"

// ─── 设备状态位定义（统一使用 board_status EventGroup）─────────────────────
// 各模块完成初始化或达到特定状态时置位对应 BIT，其他模块通过 WaitBits 同步等待。
// 使用规范：
//   置位 → xEventGroupSetBits(bsp->board_status, XXX_BIT)
//   等待 → bsp_board_check_status(bsp, XXX_BIT, timeout) 或直接 xEventGroupWaitBits

#define WIFI_BIT BIT2      ///< WiFi 连接成功且已获取有效 IP 地址
#define NVS_BIT BIT3       ///< NVS Flash 初始化完成，可读写非易失配置
#define CODEC_BIT BIT4     ///< ES8311 音频编解码器初始化完成，可开始录音/播放
#define LCD_BIT BIT5       ///< LCD 显示屏初始化完成（当前未自动置位）
#define WIFI_FAIL_BIT BIT6 ///< WiFi 连接彻底失败（超过最大重试次数），系统将重启
#define PROV_DONE_BIT BIT7 ///< BLE 配网流程结束（无论成功/超时），解除配网阻塞
#define BATTERY_BIT BIT8   ///< 电池监控模块初始化完成（ADC 就绪，可读取 VBAT）

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
    EventGroupHandle_t board_status;  ///< 设备状态事件组（FreeRTOS），各模块通过此实现就绪同步
    esp_codec_dev_handle_t codec_dev; ///< ES8311 音频编解码器设备句柄，由 audio_init() 填充
    i2s_chan_handle_t i2s_tx_handle;  ///< I2S TX 通道句柄（播放专用），由 bsp_board_codec_init() 填充
                                      ///< play_task 直接调用 i2s_channel_write 绕过 codec_dev mutex，
                                      ///< 使 audio_feed_task 的 read 与播放真正并发，消除 AFE FEED 溢出
    i2s_chan_handle_t i2s_rx_handle;  ///< [诊断用] I2S RX 通道句柄，用于绕过 codec_dev 直接读取验证
    esp_lcd_panel_io_handle_t lcd_io; ///< LCD SPI 传输接口句柄，由 bsp_board_lcd_init() 填充
    esp_lcd_panel_handle_t lcd_panel; ///< LCD ST7789 面板驱动句柄，由 bsp_board_lcd_init() 填充
    bool servo_initialized;           ///< 记录舵机是否成功初始化
    i2c_master_bus_handle_t i2c_bus;  ///< I2C 主机总线句柄（I2C_NUM_0），由 bsp_board_codec_init() 填充
                                      ///< 供 ES8311 及其它 I2C 从机（如 PCA9536 IO 扩展器）共享复用
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
 * @brief 清除 NVS 中存储的 WiFi 凭证和认证 Token，并软件重启
 *
 * 用于解绑设备：擦除 ws_token / access_token 和 WiFi 配网凭证，
 * 重启后设备重新进入 BLE 配网模式。重启前会通过 MQTT 发送重置通知。
 *
 * @note 调用者：button_monitor_task()（长按触发）或 MQTT command 消息解绑指令
 * @note 此函数调用 esp_restart()，不会返回
 */
void clear_wifi_and_restart(void);

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
 * 必须在 wake_word_init() 之后调用，因为采集任务启动后立即向引擎投喂音频。
 *
 * @param bsp_board BSP 实例指针
 *
 * @note 调用者：application.c → application_init()（步骤 4）
 * @note 前置条件：wake_word_init() 已完成（AFE/MultiNet 引擎就绪）
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

/**
 * @brief 设置扬声器输出音量（运行期可调，自动持久化到 NVS）
 *
 * 封装 esp_codec_dev_set_out_vol()，供运行期动态调节扬声器音量使用
 * （典型场景：MQTT 云端下发 {"type":"volume","value":N} 指令）。
 * 设置成功后将音量写入 NVS "audio_cfg" 命名空间的 "out_vol" 键，
 * 设备重启后由 audio_init() 读回作为初始音量。
 *
 * @param volume 目标音量（0~100）。超出范围会被自动钳位：<0 取 0，>100 取 100。
 *               音量过大可能导致 ES8311 内部 DAC 饱和产生爆音，故上限 100。
 * @return ESP_OK             设置成功
 *         ESP_ERR_INVALID_STATE  codec_dev 尚未初始化（音频硬件未就绪）
 *         其他 esp_err_t      底层 codec 写寄存器失败
 *
 * @note 通过 bsp_board_get_instance() 获取 codec_dev，无需传入 bsp_board
 * @note 线程安全：esp_codec_dev 内部有 mutex，可在任意任务/回调中调用
 * @note 调用者：mqtt_protocol.c（volume 指令）、bsp_codec.c → audio_init()（初始化）
 */
esp_err_t bsp_board_codec_set_volume(int volume);

/**
 * @brief 进入低功耗：把扬声器音量临时压到固定低音量（不影响 NVS 持久化值）
 *
 * 与 bsp_board_codec_set_volume() 的区别：本函数【不写 NVS】，只是临时调低运行期
 * 音量，避免低功耗期间的提示音/离线音频音量过大打扰。用户此前通过 MQTT/本地设置的
 * 音量（NVS "audio_cfg/out_vol"）保持不变，供 bsp_board_codec_exit_lowpower() 读回恢复。
 *
 * @note 幂等：codec_dev 未就绪时静默返回，不报错。
 * @note 调用者：standby.c enter_standby()（进一级低功耗时）
 */
void bsp_board_codec_enter_lowpower(void);

/**
 * @brief 退出低功耗：把扬声器音量恢复为用户此前设置的值（读 NVS，不重新写 NVS）
 *
 * 阻塞版：分步线性渐变直到完成再返回，仅供能接受阻塞的调用方使用
 * （standby_wake() 可能被触摸/MQTT 任务同步调用，不能用此版本——见下方非阻塞两件套）。
 *
 * @note 幂等：codec_dev 未就绪时静默返回。
 */
void bsp_board_codec_exit_lowpower(void);

/**
 * @brief 读取「退出低功耗应恢复到的音量」（NVS 保存值，无记录则默认音量），纯读取不设置
 * @return 目标音量 0~100
 * @note 调用者：standby.c standby_wake()（退出一级低功耗时取目标值，交给下面的 fade_step 推进）
 */
int bsp_board_codec_get_restore_volume(void);

/**
 * @brief 音量渐变——非阻塞单步版（按已过时间设置一次目标音量，不写 NVS）
 *
 * 供不能阻塞的调用方使用：调用方记录渐变起始时间戳，在自己的轮询循环里每 tick
 * 传入已过时间调用本函数，真正的分步设置在内部完成，不产生 vTaskDelay。
 *
 * @param from_vol   起始音量 0~100
 * @param to_vol     目标音量 0~100
 * @param elapsed_ms 距渐变开始已过的毫秒数
 * @param total_ms   渐变总耗时（毫秒）
 * @return true=已到达终点（本次已设为 to_vol），false=仍在渐变中
 * @note 非阻塞，可在任意任务的轮询循环中调用；不写 NVS。
 * @note 调用者：standby.c standby_task（一级待机退出时的非阻塞音量渐变）
 */
bool bsp_board_codec_fade_step(int from_vol, int to_vol, uint32_t elapsed_ms, uint32_t total_ms);

// ─── 公开 API：音频初始化 ─────────────────────────────────────────────────────

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

/**
 * @brief 仅关闭显示控制器输出（DISPOFF），不碰背光 PWM
 *
 * 配合 bsp_board_lcd_fade_brightness() 实现二级待机「背光先线性渐暗到 0 →
 * 再关显示控制器」，避免 bsp_board_lcd_off() 背光直接跳变到 0 的生硬观感。
 *
 * @param bsp_board BSP 实例指针
 * @note 调用者：standby.c enter_deep_standby()（二级待机线性关屏）
 */
void bsp_board_lcd_disp_off(bsp_board_t *bsp_board);

/**
 * @brief 仅打开显示控制器输出（DISPON），不碰背光 PWM
 *
 * 配合 bsp_board_lcd_fade_brightness() 实现二级待机退出「先开显示控制器
 * （此时背光仍是0，不可见）→ 再背光线性渐亮到 100%」，避免 bsp_board_lcd_on()
 * 背光直接跳变到 100% 的生硬观感。
 *
 * @param bsp_board BSP 实例指针
 * @note 调用者：standby.c standby_wake()（二级待机线性开屏）
 */
void bsp_board_lcd_disp_on(bsp_board_t *bsp_board);

/**
 * @brief 设置 LCD 背光亮度（LEDC PWM 调光，0~100%）
 *
 * 背光由 LEDC PWM 驱动，可在运行时无级调节亮度。0=熄灭，100=最亮。
 *
 * @param percent 亮度百分比（0~100，超 100 自动钳到 100）
 * @return void
 * @note 调用者：standby 待机模块（进入待机降至 BSP_LCD_BK_STANDBY_PCT，退出恢复 100%）
 * @note 前置条件：bsp_board_lcd_init() 已完成 LEDC 配置
 */
void bsp_board_lcd_set_brightness(uint8_t percent);
uint8_t bsp_board_lcd_get_brightness(void);

/**
 * @brief 背光亮度线性渐变（阻塞，从 from_pct 渐变到 to_pct）
 *
 * 分步线性过渡，避免瞬间跳变的生硬观感。总耗时由 bsp_lcd.c 内部宏
 * BSP_LCD_BK_FADE_MS 控制（当前 800ms）。
 *
 * @param from_pct 起始亮度百分比 0~100
 * @param to_pct   目标亮度百分比 0~100
 * @note 阻塞：调用期间会 vTaskDelay 约 BSP_LCD_BK_FADE_MS，仅供能接受阻塞的
 *       任务调用（当前设计为 standby_task 自身低优先级任务）。
 * @note 调用者：standby.c enter_standby()/standby_wake()（一级待机降/恢复亮度）
 */
void bsp_board_lcd_fade_brightness(uint8_t from_pct, uint8_t to_pct);

/**
 * @brief 背光亮度渐变——非阻塞单步版（按已过时间设置一次目标亮度）
 *
 * 供不能阻塞的调用方使用（如 standby_wake() 可能被触摸/MQTT任务同步调用）：
 * 调用方只需记录渐变起始时间戳，在自己的轮询循环里每 tick 传入已过时间调用本函数，
 * 真正的分步设置在本函数内部完成，不产生 vTaskDelay。
 *
 * @param from_pct   起始亮度百分比 0~100
 * @param to_pct     目标亮度百分比 0~100
 * @param elapsed_ms 距渐变开始已过的毫秒数
 * @param total_ms   渐变总耗时（毫秒）
 * @return true=已到达终点（本次已设为 to_pct），false=仍在渐变中
 * @note 非阻塞，可在任意任务的轮询循环中调用。
 * @note 调用者：standby.c standby_task（一级待机退出时的非阻塞亮度渐变）
 */
bool bsp_board_lcd_fade_step(uint8_t from_pct, uint8_t to_pct, uint32_t elapsed_ms, uint32_t total_ms);

/**
 * @brief 背光渐变——非阻塞单步版【伽马校正 + 全 10 位精度】，感知亮度线性
 *
 * 与上面 bsp_board_lcd_fade_step() 参数/返回值完全一致，可直接替换调用；区别在于
 * 内部不经过 101 级百分比、且按人眼感知（t^2.2）分配亮度，消除"暗端一顿一顿、
 * 亮端没变化"的可见跳变。详细推导见 bsp_lcd.c 中的函数注释。
 *
 * @param from_pct   起始亮度百分比 0~100
 * @param to_pct     目标亮度百分比 0~100
 * @param elapsed_ms 距渐变开始已过的毫秒数
 * @param total_ms   渐变总耗时（毫秒）
 * @return true=已到达终点（本次已设为 to_pct），false=仍在渐变中
 * @note 非阻塞。调用者：ui_port.c ui_boot_fade_timer_cb（开机 logo 渐亮/渐暗/渐亮三段）
 */
bool bsp_board_lcd_fade_step_fine(uint8_t from_pct, uint8_t to_pct, uint32_t elapsed_ms, uint32_t total_ms);

/**
 * @brief 感知亮度线性的背光渐变——【阻塞版，时长可传】（2026-08-20 新增）
 *
 * 把上面那个非阻塞单步版包成阻塞循环（每 10ms 推进一步），并支持传入总时长。
 * 相比旧的 bsp_board_lcd_fade_brightness()：步数从固定 16 步（每 125ms 才跳一档，
 * 肉眼可见"断层"）加密到每 10ms 一步，且带伽马校正，暗端不再跳。
 * 旧函数保持原样不动，其余调用方行为不受影响。
 *
 * @param from_pct 起始亮度百分比 0~100
 * @param to_pct   目标亮度百分比 0~100
 * @param total_ms 本段渐变总耗时（毫秒），传 0 则直接跳到 to_pct
 * @note 阻塞。调用者：standby.c 进/退低功耗的三段式转场（渐暗→暗中换画面→渐亮）
 */
void bsp_board_lcd_fade_brightness_fine(uint8_t from_pct, uint8_t to_pct, uint32_t total_ms);

// ========== 3. 在 API 声明区添加 ==========
/**
 * @brief 初始化躯体三轴舵机
 * @param bsp_board BSP 实例指针
 */
void bsp_board_servo_init(bsp_board_t *bsp_board);

/**
 * @brief 安全平滑地移动指定舵机
 * @param channel   舵机通道 (CH_HEAD, CH_L_ARM, CH_R_ARM)
 * @param target    目标角度 (0.0 ~ 180.0，自动受限于内部软限位)
 * @param step_ms   步进延时，数值越大动作越慢 (推荐使用 SERVO_SPEED_xxx 宏)
 */
void bsp_servo_move_smooth(uint8_t channel, float target, uint32_t step_ms);

/**
 * @brief 抢占式平滑运动：拿到通道锁【之后】才清打断标志，随即插值
 *
 * 与 bsp_servo_move_smooth 的唯一差别是加锁成功后多清一次打断标志，消除
 * 「清标志 → (窗口) → 拿锁」之间被其它任务插入的竞态。详细缘由见 bsp_servo.c
 * 中本函数的实现注释。
 *
 * @param channel   舵机通道 (CH_HEAD, CH_L_ARM, CH_R_ARM)
 * @param target    目标角度 (0.0 ~ 180.0，自动受限于内部软限位)
 * @param step_ms   步进延时，数值越大动作越慢 (推荐使用 SERVO_SPEED_xxx 宏)
 * @note 仅供「抢占后立即接管」的场景，普通动作一律用 bsp_servo_move_smooth。
 */
void bsp_servo_move_smooth_preempt(uint8_t channel, float target, uint32_t step_ms);

/**
 * @brief 读取指定舵机通道的当前角度（LEDC 寄存器推算值，非物理传感器反馈）
 *
 * @param channel  舵机通道 (CH_HEAD, CH_L_ARM, CH_R_ARM)
 * @param out_angle 输出参数：当前角度（度）
 * @return true 读取成功，false 未就绪/通道无效/读取失败（out_angle 不会被修改）
 * @note 调用者：standby.c enter_standby()（按当前角度动态计算归中速度，与熄屏时长对齐）
 */
bool bsp_servo_read_angle(uint8_t channel, float *out_angle);

/**
 * @brief 三轴舵机同时平滑运动到各自目标（并行插值，不割裂）
 * @param head_target  头部目标角度
 * @param larm_target  左臂目标角度
 * @param rarm_target  右臂目标角度
 * @param step_ms      最长轴每步延时，对应 SERVO_SPEED_xxx
 */
void bsp_servo_move_all_parallel(float head_target, float larm_target, float rarm_target, uint32_t step_ms);

/**
 * @brief 【★抖动排查临时接口，定位完即删】运行时覆盖插值帧长
 *
 * 用于在【同一次烧录】里对比不同帧长的抖动表现——SERVO_FRAME_MS 是编译期宏，
 * 要对比只能烧两次板、靠记忆比较，极不可靠。
 *
 * ★关键：改帧长【不改变动作快慢】。总耗时 = 行程 × step_ms，与帧长无关；
 *   帧长只决定这段时间被切成多少帧，即 每帧位移 = 帧长 ÷ step_ms。
 *   所以对比时唯一变量就是每帧位移，不会混入"变快了所以看不出抖"的干扰。
 *
 * @param frame_ms 帧长（毫秒）。传 0 = 恢复使用编译期的 SERVO_FRAME_MS。
 *                 建议只用 10 的整数倍：FreeRTOS tick=100Hz，vTaskDelay 粒度 10ms，
 *                 非整数倍会被截断，导致实际帧长与设定值不符。
 * @note 线程安全（atomic）。对正在进行的运动不生效，下一次运动才采用新值。
 */
void bsp_servo_debug_set_frame_ms(uint32_t frame_ms);

/**
 * @brief 请求中止正在进行的舵机插值运动（立即停在当前角度）
 *
 * 置打断标志，bsp_servo_move_all_parallel / bsp_servo_move_smooth 的插值步循环
 * 每步检查，为真则立即停止（不走完整个行程）。供 servo_manager_flush 调用，
 * 使进功能盘/强制回主时舵机最坏只滞后一个插值步（几十 ms）即停。
 * @note 打断后须由上层调 bsp_servo_clear_abort() 清标志，否则后续运动会被立即中止。
 */
void bsp_servo_request_abort(void);

/** @brief 清除舵机打断标志（上层在「打断后、开始新动作前」调用） */
void bsp_servo_clear_abort(void);

/** @brief 查询当前是否有舵机打断请求（true=已请求中止） */
bool bsp_servo_abort_requested(void);

/**
 * @brief 舵机进入低功耗休眠：停止三路 PWM 输出（供二级待机省电）
 *
 * 对 CH_HEAD/CH_L_ARM/CH_R_ARM 三路 LEDC 通道 ledc_stop，舵机失去保持力矩，
 * 静态电流下降。只动通道不动共享 timer，不影响背光/马达；不删 worker 任务，
 * 恢复为毫秒级。与 bsp_servo_resume() 配对，幂等。
 */
void bsp_servo_idle(void);

/**
 * @brief 从低功耗休眠恢复舵机并慢速归中（与 bsp_servo_idle 配对）
 *
 * 重新点亮三路 PWM 输出（写 90°）并缓慢归中，避免从失力位置猛跳抽搐。幂等。
 */
void bsp_servo_resume(void);

// ─── 7. 触摸事件与接口 (整合自 bsp_touch.h) ───────────────────────────────

/**
 * @brief 触摸事件类型枚举（对应物理铜箔位置）
 */
typedef enum
{
    TOUCH_EVENT_NONE = 0,
    // 单位置触摸
    TOUCH_EVENT_SHORT_HEAD,    // 1 头部短按（主界面情绪/游戏确认）
    TOUCH_EVENT_LONG_HEAD,     // 2 头部长按（≥800ms）→ 各层通用退出/返回
    TOUCH_EVENT_SHORT_ABDOMEN, // 3 腹部短按（仅主界面情绪，不参与应用/游戏）
    TOUCH_EVENT_SHORT_BACK,    // 4 背部短按（仅主界面情绪，不参与应用/游戏）
    // 双位置组合触摸（仅主界面）
    TOUCH_EVENT_COMBO_HEAD_ABDOMEN, // 5 头部+腹部同时按
    TOUCH_EVENT_COMBO_HEAD_BACK,    // 6 头部+背部同时按
    TOUCH_EVENT_COMBO_ABDOMEN_BACK, // 7 腹部+背部同时按
    // 翻页控制触摸
    TOUCH_EVENT_SHORT_PREV_PAGE, // 8  前一页：短按
    TOUCH_EVENT_SHORT_NEXT_PAGE, // 9  后一页：短按
    TOUCH_EVENT_LONG_PREV_PAGE,  // 10 前一页：长按 → 进入功能菜单
    TOUCH_EVENT_LONG_NEXT_PAGE   // 11 后一页：长按 → 进入功能菜单
} touch_event_t;

/**
 * @brief 触摸扫描任务入口（由 app_main 创建）
 * @param pvParameters BSP 实例指针（可选）
 */
void touch_scan_task(void *pvParameters);

/**
 * @brief OTA 升级前停止触摸扫描任务（不可逆，重启前调用）
 *
 * 置停止标志，touch_scan_task 在循环开头检测到后 vTaskDelete(NULL) 自删，
 * 彻底停止触摸扫描、让出 CPU 给固件下载。OTA 结束必重启，无需恢复。
 */
void bsp_touch_stop_for_ota(void);

/**
 * @brief 非阻塞获取触摸事件（立即返回）
 * @param out_event 输出触摸事件
 * @return true 有事件，false 无事件
 */
bool bsp_touch_get_event(touch_event_t *out_event);

/**
 * @brief 获取最近一次翻页键（左/右耳）从按下到松手的按压时长（毫秒）
 *
 * 用途：跳一跳游戏的「长按蓄力」——按住越久跳得越远。
 * 触摸层在松手判定那一刻已算出 held 时长（见 update_page_btn），
 * 原本仅用于区分短按/长按后丢弃；此处把它保留导出，供游戏读取。
 *
 * 时序：每收到一个 SHORT_PREV/NEXT_PAGE 或 LONG_PREV/NEXT_PAGE 事件后，
 *       立即调用本函数即可拿到该次按压的真实时长。值会被下一次按压覆盖。
 *
 * @return 最近一次翻页键按压时长（毫秒）；从未按过返回 0
 */
uint32_t bsp_touch_last_page_hold_ms(void);

/**
 * @brief 查询翻页键当前实时按压时长（毫秒）
 *
 * 供跳一跳 engine_cb 每帧驱动「按住期间小人/台子压扁 + 蓄力条实时增长」。
 * 未按住任何翻页键时返回 0。
 */
uint32_t bsp_touch_page_held_ms(void);

/**
 * @brief 清除全部触摸按键的「持续按压闩锁」（consumed 标志）
 *
 * 用途：退出深度待机的转场结束后调用一次。
 *
 * 【为什么需要】唤醒待机的那一次触摸，在触摸层是正常发出了事件的（并因此置了
 * consumed 闩锁），但上层 ui_dispatch_touch_event 判定「本次仅用于唤醒」后直接
 * 丢弃了它。触摸层无从得知事件被丢弃，闩锁就此残留。叠加退待机转场约 2.6 秒
 * 全程阻塞（扫描任务停摆、离手确认帧数累不够），表现为「用哪个位置唤醒，
 * 哪个位置随后就按不动，其余位置正常」。
 *
 * @note 纯内存赋值，不阻塞、不读 NVS、不取锁，可在任意任务上下文安全调用
 *       （包括栈在 PSRAM 的任务）。
 * @note 幂等；非待机路径不会调用到，正常态 / 游戏 / 功能盘的闩锁行为不受影响。
 * @note 调用者：standby.c standby_wake()（转场收尾，释放 s_waking 门闩之前）
 */
void bsp_touch_clear_latch(void);

/**
 * @brief 请求触摸任务在本轮末尾补一次震动（延后到 LVGL 锁已释放后执行）
 *
 * 供上层在「操作确认真正生效」时调用（如功能盘 home_jelly 真正推进了下标），
 * 实现「震动跟随真实切换，而不是跟随触摸」。
 *
 * @note 只置标志、不阻塞：真正的 bsp_motor_pulse()（内含 30ms vTaskDelay）由
 *       touch_scan_task 在事件分发完成、LVGL 锁已释放之后统一执行，避免持锁阻塞
 *       LVGL 线程（同 BUG-038 的坑）。
 * @note 与按下沿震动共用同一标志，同一轮内重复请求只震一次，天然去重。
 * @note 仅可在任务上下文调用，不可在中断里用。
 */
void bsp_touch_request_vibrate(void);

/**
 * @brief 震动马达单次脉冲（触觉反馈，默认强度，约 30ms）
 * @note 内部以 LEDC PWM 输出，强度由 BSP_MOTOR_DEFAULT_STRENGTH 决定
 */
void bsp_motor_pulse(void);

/**
 * @brief 设置震动马达持续输出强度
 * @param strength 震动强度百分比 0~100：0=停止，100=最强
 * @note 通过 LEDC 占空比实现，数值越大震感越强；
 *       调用后马达保持该强度持续震动，需自行调用 bsp_motor_set(0) 停止。
 */
void bsp_motor_set(uint8_t strength);

/**
 * @brief 震动马达带强度的单次脉冲
 * @param strength 震动强度百分比 0~100
 * @param ms       持续时长（毫秒），结束后自动停止
 * @note 内部含 vTaskDelay 阻塞，仅可在任务上下文调用
 */
void bsp_motor_pulse_level(uint8_t strength, uint32_t ms);

/**
 * @brief 单独初始化震动马达的 LEDC PWM 通道（不含触摸硬件）
 *
 * 【为什么要单独暴露】开机震动反馈需要在 application_init() 最前面就能震，
 * 而马达 LEDC 原本只在 bsp_touch_init() 里初始化，那是 touch_scan_task 创建时
 * 才跑的（LCD 点亮之后），太晚。故拆出本函数供开机早期先行调用。
 *
 * 幂等：与 bsp_touch_init() 内部那次重复调用同参数，LEDC 重复配置安全无副作用。
 */
void bsp_motor_ledc_init(void);

/**
 * @brief 震动 PWM 方波测试任务（仅调试用）
 *
 * 循环把占空比设为 0→25→50→75→100，每档持续 3 秒，串口打印当前档位。
 * 用 bsp_motor_set() 持续输出，便于示波器观测 5kHz PWM 方波（时基 ~50µs/格）。
 * 在 application.c 里 xTaskCreate 启动；测试完成后注释掉创建代码即可。
 */
void motor_pwm_test_task(void *pvParameters);

/**
 * @brief 初始化触摸控制器和震动马达（内部调用，无需手动执行）
 * @note 由 touch_scan_task() 内部自动调用
 */
void bsp_touch_init(void);

void bsp_flash_init(void);

// ─── 8. 电池电压监控 (VBAT_ADC) ─────────────────────────────────────────────
//
// 硬件原理：
//   VBAT ──[R23 200kΩ]──┬──[R24 200kΩ]── GND
//                       └── 分压点 → ADC 引脚（BSP_BAT_ADC_PIN）
//   真实电池电压 = ADC 采样电压 × (R23 + R24) / R24 = ADC × 2
//
// 软件设计：
//   - ESP-IDF 新 ADC oneshot API + curve fitting 校准
//   - 多次平均 + IIR 低通滤波，抑制噪声与抖动
//   - 锂电池放电曲线分段插值（4.2V→100% ... 3.0V→0%）
//   - 后台任务周期采样，低电触发回调（带 100mV 滞回）

/**
 * @brief 低电量告警回调函数原型
 *
 * 后台采样发现电池电压低于 BSP_BAT_VOLTAGE_LOW_MV 时调用（只触发一次，
 * 直到电压回升超过阈值 + 100mV 滞回区间后才会再次允许触发）。
 *
 * @param voltage_mv 当前电池电压（毫伏）
 * @param percent    当前电量百分比（0~100）
 */
typedef void (*bsp_battery_low_cb_t)(uint32_t voltage_mv, uint8_t percent);

/**
 * @brief 初始化电池监控模块（仅初始化 ADC，不启动后台任务）
 *
 * 内部完成：配置 ADC1 oneshot 单元，为通道选择 ADC_ATTEN_DB_12（0~3.1V 量程），
 * 创建 curve fitting 校准句柄。
 *
 * @return ESP_OK 成功；ESP_ERR_INVALID_STATE 表示 BSP_BAT_ADC_PIN 未配置；
 *         其他错误为 ADC 初始化失败。
 */
esp_err_t bsp_battery_init(void);

/**
 * @brief 反初始化（释放 ADC 与校准资源，停止后台任务）
 */
esp_err_t bsp_battery_deinit(void);

/**
 * @brief 同步读取当前电池电压（毫伏，已经过分压还原 ×2 和多次平均）
 * @return 电池电压（毫伏）；返回 0 表示尚未初始化或读取失败
 */
uint32_t bsp_battery_read_voltage_mv(void);

/**
 * @brief 获取后台任务最近一次采样并滤波后的电池电压（毫伏）
 *
 * 若后台任务未启动，则会现场调用一次 ADC 采样。
 */
uint32_t bsp_battery_get_voltage_mv(void);

/**
 * @brief 获取电量百分比（基于锂电池放电曲线，0~100）
 *
 * 映射节点（典型 LiPo 放电曲线）：
 *   4.20V → 100%   4.05V → 90%    3.95V → 80%    3.85V → 70%
 *   3.80V → 60%    3.75V → 50%    3.70V → 40%    3.65V → 30%
 *   3.60V → 20%    3.45V → 10%    3.30V → 5%     3.00V → 0%
 */
uint8_t bsp_battery_get_percent(void);

/**
 * @brief 查询当前是否判定为正在充电（USB/适配器供电）
 *
 * 无独立 VBUS 检测脚，靠 OCV 连续上升趋势推断（见 bsp_battery.c 防回弹逻辑），
 * 与低电自动关机共用同一个判定结果，保证两处认知一致。
 *
 * @return true=正在充电，false=未充电（或尚未判定出趋势）
 */
bool bsp_battery_is_charging(void);

/**
 * @brief 立即软关机（GPIO18/OPT 高→低下降沿命令 HK015T 切断主电源），不返回
 *
 * 屏幕探针实测：单纯拉低不断电、单纯拉高只瞬断自恢复；真正断电靠 OPT 的"高→低"
 * 下降沿（先拉高 BSP_PWR_OFF_PULSE_MS 建立干净高，再拉低保持）。供低电自动关机与
 * standby 三级关机共用。调用前如需存档电量请由调用方自行处理。
 */
void bsp_battery_power_off(void);

/**
 * @brief 获取当前 WiFi 信号强度 RSSI
 *
 * @return RSSI 值（dBm，负数，越大越好）；未连接或读取失败返回 0
 *
 * @note 调用者：UI 状态栏（顶部 WiFi 图标刷新）
 */
int bsp_wifi_get_rssi(void);

/**
 * @brief 查询设备是否已进入"运行态离线模式"
 *
 * 运行态（已配网、已 GOT_IP 过之后）WiFi 断线重连达 MAX_RETRY_COUNT 次仍失败时，
 * bsp_wifi 内部置位本标志并彻底关停 WiFi（disconnect + stop 关射频），不再重连、
 * 也不再软复位重启。标志一旦置位，**本次开机周期内永不清除**——恢复联网需用户
 * 手动关机再开机（冷启动后静态变量自然复位）。
 *
 * 所有依赖网络的功能（WebSocket 会话 / MQTT / 天气 / OTA / UDP 日志）必须在发起
 * 请求前调用本函数拦截，离线时直接返回，避免任务空转与 SRAM 反复申请释放。
 *
 * @return true = 已进入离线模式（网络功能必须跳过）；false = 正常
 *
 * @note 调用者：session.c / mqtt_protocol.c / reminder.c / bsp_ota.c
 * @note 区别于瞬时的 WIFI_BIT（随抖动变化），本标志是"判定彻底掉网"后的持久状态
 */
bool bsp_wifi_is_offline_mode(void);

/**
 * @brief 启动后台采样任务（周期 BSP_BAT_TASK_INTERVAL_MS）
 *
 * 任务循环：采样 → IIR 滤波 → 判断是否触发低电回调 → 等待下一周期。
 *
 * @param low_cb 低电量告警回调，传 NULL 表示不需要回调
 * @return ESP_OK；若任务已存在则返回 ESP_ERR_INVALID_STATE
 */
esp_err_t bsp_battery_start_task(bsp_battery_low_cb_t low_cb);

/**
 * @brief 停止后台采样任务
 */
esp_err_t bsp_battery_stop_task(void);

/**
 * @brief 启动独立的电池电压日志任务（固定每 5 秒打印一次）
 *
 * 该任务不参与采样/滤波逻辑，仅周期性读取当前电压与电量百分比并以
 * INFO 级别打印，方便调试观察。需在 bsp_battery_init() 之后调用。
 *
 * @return ESP_OK；若任务已存在则返回 ESP_ERR_INVALID_STATE
 */
esp_err_t bsp_battery_start_log_task(void);

/**
 * @brief 停止电池电压日志任务（与 bsp_battery_start_log_task 成对）
 *
 * 请求日志任务退出并等待其真正结束。用于 OTA 升级等需要关闭无关后台任务的场景。
 *
 * @return ESP_OK；若任务不存在则返回 ESP_OK（幂等）
 */
esp_err_t bsp_battery_stop_log_task(void);