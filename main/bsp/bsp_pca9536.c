/**
 * @file bsp_pca9536.c
 * @brief PCA9536 4 位 I2C IO 扩展器驱动实现
 *
 * 复用 ES8311 所在的 I2C_NUM_0 总线（句柄由 bsp_board_codec_init() 保存到
 * bsp_board->i2c_bus）。所有寄存器访问为「读改写」，避免误改其它引脚位。
 */

#include "bsp_pca9536.h"
#include "esp_log.h"

static const char *TAG = "bsp_pca9536";

// ─── PCA9536 硬件常量（TI 数据手册 Table 1~5）────────────────────────────────
#define PCA9536_I2C_ADDR 0x41 ///< 7-bit 从机地址（固定，无地址引脚）

#define PCA9536_REG_INPUT 0x00    ///< 输入端口寄存器（只读，读管脚电平）
#define PCA9536_REG_OUTPUT 0x01   ///< 输出端口寄存器（写输出电平，默认 0xFF）
#define PCA9536_REG_POLARITY 0x02 ///< 极性反转寄存器（默认 0x00）
#define PCA9536_REG_CONFIG 0x03   ///< 方向配置寄存器（位=1 输入/位=0 输出，默认 0xFF）

#define PCA9536_PIN_MAX 3          ///< 最大引脚号（P0~P3）
#define PCA9536_I2C_TIMEOUT_MS 100 ///< 单次 I2C 事务超时（ms）

// ─── 模块内部状态 ─────────────────────────────────────────────────────────────
static i2c_master_dev_handle_t s_pca9536_dev = NULL; ///< I2C 从设备句柄

// ═══════════════════════════════════════════════════════════════════════════════
// 内部工具：单寄存器读 / 写
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 读 PCA9536 单个寄存器（先写命令字节，再读 1 字节）
 */
static esp_err_t pca9536_read_reg(uint8_t reg, uint8_t *out_val)
{
    if (s_pca9536_dev == NULL)
    {
        ESP_LOGE(TAG, "设备未初始化，请先调用 bsp_pca9536_init()");
        return ESP_ERR_INVALID_STATE;
    }
    // I2C 时序：START → 从机地址(W) → 命令字节 reg → RESTART → 从机地址(R) → 读 1 字节 → STOP
    return i2c_master_transmit_receive(s_pca9536_dev, &reg, 1, out_val, 1,
                                       PCA9536_I2C_TIMEOUT_MS);
}

/**
 * @brief 写 PCA9536 单个寄存器（写命令字节 + 数据字节，共 2 字节）
 */
static esp_err_t pca9536_write_reg(uint8_t reg, uint8_t val)
{
    if (s_pca9536_dev == NULL)
    {
        ESP_LOGE(TAG, "设备未初始化，请先调用 bsp_pca9536_init()");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2] = {reg, val};
    // I2C 时序：START → 从机地址(W) → reg → val → STOP
    return i2c_master_transmit(s_pca9536_dev, buf, sizeof(buf), PCA9536_I2C_TIMEOUT_MS);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 公开 API
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t bsp_pca9536_init(bsp_board_t *bsp_board)
{
    if (bsp_board == NULL || bsp_board->i2c_bus == NULL)
    {
        ESP_LOGE(TAG, "I2C 总线未就绪，请确认已先执行 bsp_board_codec_init()");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_pca9536_dev != NULL)
    {
        ESP_LOGW(TAG, "PCA9536 已初始化，跳过重复初始化");
        return ESP_OK;
    }

    // ── 把 PCA9536 挂到已存在的 I2C 总线上 ──────────────────────────────────
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7, // 7-bit 地址模式
        .device_address = PCA9536_I2C_ADDR,    // 0x41（新驱动用原始 7-bit，无需右移）
        .scl_speed_hz = 100000,                // 100kHz 标准速率（PCA9536 最高支持 400kHz）
    };
    esp_err_t err = i2c_master_bus_add_device(bsp_board->i2c_bus, &dev_cfg, &s_pca9536_dev);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "add_device 失败: %s", esp_err_to_name(err));
        s_pca9536_dev = NULL;
        return err;
    }

    // ── 回读输入寄存器验证通信 ──────────────────────────────────────────────
    uint8_t in_val = 0;
    err = pca9536_read_reg(PCA9536_REG_INPUT, &in_val);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "初始化回读失败（检查接线/地址/上拉）: %s", esp_err_to_name(err));
        i2c_master_bus_rm_device(s_pca9536_dev);
        s_pca9536_dev = NULL;
        return err;
    }

    ESP_LOGI(TAG, "PCA9536 初始化成功 @0x%02X，输入端口=0x%X", PCA9536_I2C_ADDR, in_val & 0x0F);
    return ESP_OK;
}

esp_err_t bsp_pca9536_set_direction(uint8_t pin, bsp_pca9536_dir_t dir)
{
    if (pin > PCA9536_PIN_MAX)
    {
        ESP_LOGE(TAG, "引脚号越界: %u（有效 0~3）", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // 读改写：只改目标位，保留其它引脚方向
    uint8_t cfg = 0;
    esp_err_t err = pca9536_read_reg(PCA9536_REG_CONFIG, &cfg);
    if (err != ESP_OK)
    {
        return err;
    }

    if (dir == BSP_PCA9536_DIR_INPUT)
    {
        cfg |= (1u << pin); // 位=1 输入
    }
    else
    {
        cfg &= ~(1u << pin); // 位=0 输出
    }

    return pca9536_write_reg(PCA9536_REG_CONFIG, cfg);
}

esp_err_t bsp_pca9536_set_direction_mask(uint8_t dir_mask)
{
    // 仅低 4 位有效
    return pca9536_write_reg(PCA9536_REG_CONFIG, dir_mask & 0x0F);
}

esp_err_t bsp_pca9536_write_pin(uint8_t pin, bool level)
{
    if (pin > PCA9536_PIN_MAX)
    {
        ESP_LOGE(TAG, "引脚号越界: %u（有效 0~3）", pin);
        return ESP_ERR_INVALID_ARG;
    }

    // 读改写：只改目标位，保留其它引脚输出状态
    uint8_t out = 0;
    esp_err_t err = pca9536_read_reg(PCA9536_REG_OUTPUT, &out);
    if (err != ESP_OK)
    {
        return err;
    }

    if (level)
    {
        out |= (1u << pin); // 高电平
    }
    else
    {
        out &= ~(1u << pin); // 低电平
    }

    return pca9536_write_reg(PCA9536_REG_OUTPUT, out);
}

esp_err_t bsp_pca9536_read_pin(uint8_t pin, bool *out_level)
{
    if (pin > PCA9536_PIN_MAX || out_level == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t in_val = 0;
    esp_err_t err = pca9536_read_reg(PCA9536_REG_INPUT, &in_val);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_level = (in_val & (1u << pin)) != 0;
    return ESP_OK;
}

esp_err_t bsp_pca9536_read_all(uint8_t *out_mask)
{
    if (out_mask == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t in_val = 0;
    esp_err_t err = pca9536_read_reg(PCA9536_REG_INPUT, &in_val);
    if (err != ESP_OK)
    {
        return err;
    }

    *out_mask = in_val & 0x0F; // 仅低 4 位有效
    return ESP_OK;
}

// ═══════════════════════════════════════════════════════════════════════════════
// 功放（NS4150）开关封装 —— P0 → PA_EN → NS4150.CTRL（本板 P0 的唯一用途）
//   高 = 功放工作（出声）    低 = 功放关断（静音）
//   使用原则（防爆音）：★功放最后一个开、第一个关★
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t bsp_pa_enable(void)
{
    esp_err_t err = bsp_pca9536_write_pin(BSP_PCA9536_P0, true); // 高 = 功放工作
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "开功放失败: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t bsp_pa_disable(void)
{
    esp_err_t err = bsp_pca9536_write_pin(BSP_PCA9536_P0, false); // 低 = 功放关断
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "关功放失败: %s", esp_err_to_name(err));
    }
    return err;
}
