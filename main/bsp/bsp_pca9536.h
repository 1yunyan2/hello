#pragma once

/**
 * @file bsp_pca9536.h
 * @brief PCA9536 —— 4 位 I2C IO 扩展器驱动（TI PCA9536DGKR）
 *
 * PCA9536 通过 I2C 提供 4 路可独立配置输入/输出的 GPIO（P0~P3）。
 * 复用 ES8311 所在的 I2C_NUM_0 总线（SDA/SCL 见 bsp_config.h），
 * 从机地址固定 0x41，与 ES8311(0x18) 不冲突，可直接并挂在同一总线。
 *
 * 数据手册要点（TI PCA9536, Table 1~5）：
 *   - 7-bit 从机地址：0x41（无地址选择引脚，固定）
 *   - 4 个寄存器（命令字节）：
 *       0x00 Input Port          读引脚电平（只读）
 *       0x01 Output Port         写输出电平（默认 0xFF）
 *       0x02 Polarity Inversion  输入极性反转（默认 0x00）
 *       0x03 Configuration       方向：位=1 输入 / 位=0 输出（默认 0xFF 全输入）
 *   - 仅低 4 位有效（bit0~bit3 → P0~P3），高 4 位读回为 0、写入忽略
 *
 * 使用顺序：先 bsp_pca9536_init() → bsp_pca9536_set_direction() 配方向
 *          → 输出用 bsp_pca9536_write_pin()，输入用 bsp_pca9536_read_pin()
 *
 * @note 依赖 bsp_board_codec_init() 先执行（它创建并保存 i2c_bus 句柄）
 */

#include "bsp_board.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

// ─── PCA9536 引脚编号 ─────────────────────────────────────────────────────────
#define BSP_PCA9536_P0 0 ///< 扩展 IO P0
#define BSP_PCA9536_P1 1 ///< 扩展 IO P1
#define BSP_PCA9536_P2 2 ///< 扩展 IO P2
#define BSP_PCA9536_P3 3 ///< 扩展 IO P3

// ─── 引脚方向 ─────────────────────────────────────────────────────────────────
typedef enum
{
    BSP_PCA9536_DIR_OUTPUT = 0, ///< 输出（Configuration 位 = 0）
    BSP_PCA9536_DIR_INPUT = 1,  ///< 输入（Configuration 位 = 1，上电默认）
} bsp_pca9536_dir_t;

/**
 * @brief 初始化 PCA9536（把设备挂到已存在的 I2C 总线上）
 *
 * 从 bsp_board 单例取出 codec_init 保存的 i2c_bus 句柄，
 * 用 i2c_master_bus_add_device 把 PCA9536(0x41) 加为从设备，
 * 并回读一次输入寄存器验证通信。
 *
 * @param bsp_board BSP 全局单例指针（其 i2c_bus 字段必须已由 codec_init 填充）
 * @return ESP_OK 成功；ESP_ERR_INVALID_STATE 总线未就绪；其它为 I2C 通信失败
 *
 * @note 必须在 bsp_board_codec_init() 之后调用
 */
esp_err_t bsp_pca9536_init(bsp_board_t *bsp_board);

/**
 * @brief 设置单个引脚方向（读改写 Configuration 寄存器 0x03）
 *
 * @param pin 引脚编号 BSP_PCA9536_P0 ~ P3
 * @param dir BSP_PCA9536_DIR_OUTPUT 或 BSP_PCA9536_DIR_INPUT
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG 参数越界；其它为 I2C 失败
 */
esp_err_t bsp_pca9536_set_direction(uint8_t pin, bsp_pca9536_dir_t dir);

/**
 * @brief 一次性设置全部 4 路方向（直接写 Configuration 寄存器 0x03）
 *
 * @param dir_mask 低 4 位方向掩码：位=1 输入 / 位=0 输出
 *                 例：0b0001 → P0 输入，P1~P3 输出
 * @return ESP_OK 成功；其它为 I2C 失败
 */
esp_err_t bsp_pca9536_set_direction_mask(uint8_t dir_mask);

/**
 * @brief 设置输出引脚电平（读改写 Output 寄存器 0x01）
 *
 * @param pin   引脚编号 BSP_PCA9536_P0 ~ P3（须已配置为输出）
 * @param level true=高电平，false=低电平
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG 参数越界；其它为 I2C 失败
 */
esp_err_t bsp_pca9536_write_pin(uint8_t pin, bool level);

/**
 * @brief 读取单个引脚电平（读 Input 寄存器 0x00）
 *
 * 无论引脚配置为输入还是输出，Input 寄存器都反映管脚真实电平。
 *
 * @param pin       引脚编号 BSP_PCA9536_P0 ~ P3
 * @param out_level 输出：true=高电平，false=低电平
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG 参数越界/空指针；其它为 I2C 失败
 */
esp_err_t bsp_pca9536_read_pin(uint8_t pin, bool *out_level);

/**
 * @brief 读取全部 4 路输入电平（读 Input 寄存器 0x00）
 *
 * @param out_mask 输出：低 4 位为 P0~P3 电平（位=1 高，位=0 低）
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG 空指针；其它为 I2C 失败
 */
esp_err_t bsp_pca9536_read_all(uint8_t *out_mask);

// ═══════════════════════════════════════════════════════════════════════════════
// 功放（NS4150）开关封装 —— 本板 P0 的唯一用途
//   硬件：PCA9536.P0 → PA_EN → NS4150.CTRL（高=出声，低=静音）
//   使用原则（防爆音）：★功放最后一个开、第一个关★
//     开机：bsp_pa_disable() → ES8311上电 → 等VMID稳定 → bsp_pa_enable()
//     关机：bsp_pa_disable() → 等50ms → 关ES8311/断电
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 打开功放（PA_EN=1，NS4150 开始工作）
 * @note 必须在 ES8311 上电并等 VMID 稳定之后调用，否则上电阶跃会被放大成爆音
 * @return ESP_OK 成功；其它为 I2C 失败
 */
esp_err_t bsp_pa_enable(void);

/**
 * @brief 关闭功放（PA_EN=0，NS4150 关断静音）
 * @note 用于开机上电前静音保护、运行期静音、关机断电前 pop 抑制
 * @return ESP_OK 成功；其它为 I2C 失败
 */
esp_err_t bsp_pa_disable(void);
