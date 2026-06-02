#include "bsp_board.h"
#include "driver/gpio.h"
#include "driver/ledc.h" // 背光 PWM 调光（LEDC 外设）
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"

/**
 * @brief 设置 LCD 背光亮度（内部 helper，LEDC PWM 占空比方式）
 *
 * 将 0~100 的百分比线性映射到 10 位 LEDC 占空比（0~1023），写入并立即生效。
 * 占空比越大背光越亮（对应硬件「高电平开启背光」）。
 *
 * @param pct 亮度百分比，0=熄灭，100=最亮（超过 100 自动钳到 100）
 * @return void
 *
 * @note 调用者：bsp_board_lcd_init/on/off、bsp_board_lcd_set_brightness（待机模块）
 * @note 前置条件：bsp_board_lcd_init() 已完成 LEDC timer/channel 配置
 */
static void bsp_lcd_bk_set_percent(uint8_t pct)
{
    if (pct > 100)
        pct = 100; // 钳位，防止占空比越界
    // ★ 实测结论：本板背光为「高电平点亮」——GPIO42 高电平占比越大越亮。
    //   （两次实测确认：duty=DUTY_MAX/高电平→全亮；duty=0/低电平→全灭，故不取反。）
    //   duty 直接正比于亮度：pct=100→duty=DUTY_MAX(最亮)；pct=0→duty=0(灭)。
    uint32_t duty = (uint32_t)pct * BSP_LCD_BK_DUTY_MAX / 100;
    esp_err_t e1 = ledc_set_duty(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_CHANNEL, duty);
    esp_err_t e2 = ledc_update_duty(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_CHANNEL);
    // 【诊断】打印实际写入的占空比与返回值，确认软件链路生效（验证 OK 后可删）
    ESP_LOGW("BSP_LCD_BK", "亮度=%u%% → duty=%lu/%d (高电平点亮, set=%d upd=%d)",
             pct, (unsigned long)duty, BSP_LCD_BK_DUTY_MAX, e1, e2);
}

/**
 * @brief 设置 LCD 背光亮度（对外 API，供待机/省电模块调用）
 *
 * @param percent 亮度百分比 0~100
 * @return void
 * @note 调用者：standby 待机模块（进入待机降至 50%，退出恢复 100%）
 */
void bsp_board_lcd_set_brightness(uint8_t percent)
{
    bsp_lcd_bk_set_percent(percent);
}

/**
 * @brief 初始化 LCD 显示屏（ST7789，240×320，RGB565）
 *
 * 完整 LCD 初始化流程：
 *   1. 配置背光 GPIO（初始关闭，等上层主动开启）
 *   2. 初始化 SPI2 总线（80MHz，自动 DMA 分配）
 *   3. 创建 SPI LCD 通信接口（含 DC/CS/时钟配置）
 *   4. 初始化 ST7789 面板驱动（含颜色反转，IPS 屏必需）
 *   5. 硬件复位 → 软件初始化 → 关闭显示（等待上层显式开启）
 *
 * @param bsp_board BSP 实例指针
 *                  - 输出：lcd_io 字段填充 SPI 接口句柄
 *                  - 输出：lcd_panel 字段填充面板驱动句柄
 * @return void（失败时 ESP_ERROR_CHECK 触发系统重启）
 *
 * @note 调用者：application.c（当前已预留，LCD 功能启用后取消注释）
 * @note 引脚：CS=10, MOSI=11, SCLK=12, DC=13, RST=14, BK=48（bsp_config.h）
 * @note 颜色格式：RGB565（每像素 2 字节），小端字节序
 */
void bsp_board_lcd_init(bsp_board_t *bsp_board)
{
    gpio_reset_pin(BSP_LCD_SCLK_PIN); // 释放 时钟线（SCLK）
    gpio_reset_pin(BSP_LCD_MOSI_PIN); // 释放 数据线（MOSI）
    gpio_reset_pin(BSP_LCD_DC_PIN);   // 释放数据/命令选择（D/C）
    gpio_reset_pin(BSP_LCD_CS_PIN);   // 释放 片选（CS/NSS）
    gpio_reset_pin(BSP_LCD_BK_PIN);   // 释放 背光控制（BK）
    gpio_reset_pin(BSP_LCD_RST_PIN);  // 释放 硬件复位（RST）
    // ── 步骤 1：配置背光 LEDC PWM（调光）────────────────────────────────────
    // 背光引脚（GPIO42）由 LEDC PWM 驱动，支持 0~100% 亮度调节（待机模式需 50%）。
    // LEDC 输出经 GPIO Matrix 自动路由到 GPIO42（ESP32-S3 LEDC 无 IO_MUX 直连），无需手动映射。
    // ★ 使用独立 TIMER_1 + CHANNEL_3，与舵机的 TIMER_0/CH0-2 隔离（见 BUG-015）。
    // 初始占空比 0（背光关闭），避免屏幕在初始化过程中显示乱码。
    ledc_timer_config_t bk_timer_config = {
        .speed_mode = BSP_LCD_BK_LEDC_MODE,       // 低速模式
        .timer_num = BSP_LCD_BK_LEDC_TIMER,       // 独立定时器 TIMER_1
        .duty_resolution = BSP_LCD_BK_LEDC_RES,   // 10 位分辨率（0~1023）
        .freq_hz = BSP_LCD_BK_LEDC_FREQ_HZ,       // 5kHz
        .clk_cfg = BSP_LCD_BK_LEDC_CLK,           // RC_FAST：与舵机时钟源统一，避免冲突
    };
    ESP_ERROR_CHECK(ledc_timer_config(&bk_timer_config));

    ledc_channel_config_t bk_channel_config = {
        .gpio_num = BSP_LCD_BK_PIN,               // 背光引脚 GPIO42
        .speed_mode = BSP_LCD_BK_LEDC_MODE,
        .channel = BSP_LCD_BK_LEDC_CHANNEL,       // 独立通道 CHANNEL_3
        .timer_sel = BSP_LCD_BK_LEDC_TIMER,       // 绑定到 TIMER_1
        .duty = 0,                                // 初始占空比 0 = 背光关闭
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bk_channel_config));

    // ── 步骤 2：初始化 SPI2 总线 ─────────────────────────────────────────────
    // SPI2_HOST（HSPI）：ESP32-S3 第二个 SPI 控制器，支持 DMA 加速传输
    // max_transfer_sz 设为整屏大小，保证一次 flush_fb 不会截断
    spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SCLK_PIN, // 时钟线（GPIO12），最高 80MHz
        .mosi_io_num = BSP_LCD_MOSI_PIN, // 数据线（GPIO11），主发从收，LCD 单向写
        .miso_io_num = -1,               // 无 MISO（ST7789 不支持读回，只写）
        .quadwp_io_num = -1,             // 不使用四线 SPI（QSPI）
        .quadhd_io_num = -1,             // 不使用四线 SPI
        // 最大 DMA 传输字节数 = 整屏像素数 × 每像素字节数 + 余量
        // 240×320×2 = 153600 字节 ≈ 150KB，确保整帧刷新不溢出
        .max_transfer_sz = BSP_LCD_WIDTH * BSP_LCD_HEIGHT * 2 + 8,
    };
    // SPI_DMA_CH_AUTO：自动分配 DMA 通道，使用 DMA 可大幅降低 CPU 占用
    // 注意：LCD 引脚（CS=41,MOSI=39,SCLK=38）对应 SPI2_HOST（HSPI），不是 SPI3
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // ── 步骤 3：创建 SPI LCD 通信接口 ────────────────────────────────────────
    // 此接口封装了 SPI 事务的时序细节，上层只需调用 esp_lcd_panel_* API
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC_PIN, // DC 引脚（GPIO13）：高=数据，低=命令
        .cs_gpio_num = BSP_LCD_CS_PIN, // CS 引脚（GPIO10）：低电平选中 LCD
        // SPI 时钟 20MHz：PCBA 板信号完整性比开发板差（更长走线 + 共地不理想 + 寄生电容），
        // 在 40MHz 下偶发 "上电只有背光 / 白屏 / 卡 GIF 第一帧 / 重影" —— 日志正常但屏幕无显示，
        // 是典型 SPI 命令丢包征兆（ST7789 没读到完整 init/cmd 序列）。
        // 20MHz 在开发板已大量验证可流畅刷 GIF，先把时序余量留足，稳定后再尝试 30/40MHz。
        .pclk_hz = 40 * 1000 * 1000,
        .lcd_cmd_bits = 8,       // 命令字段位宽（ST7789 固定 8-bit）
        .lcd_param_bits = 8,     // 参数字段位宽（ST7789 固定 8-bit）
        .spi_mode = 0,           // SPI 模式 0（CPOL=0，CPHA=0）
        .trans_queue_depth = 10, // 事务队列深度（最多 10 个异步事务排队）
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &bsp_board->lcd_io));

    // ── 步骤 4：初始化 ST7789 LCD 面板驱动 ───────────────────────────────────
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST_PIN,          // 复位引脚（GPIO14），低电平复位
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // RGB 像素排列顺序（R高位，B低位）
        .bits_per_pixel = 16,                       // 每像素 16-bit（RGB565 格式）
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,  // 小端字节序（ESP32 原生字节序）
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(
        bsp_board->lcd_io, &panel_config, &bsp_board->lcd_panel));

    // ── 步骤 5：关闭背光（等待初始化完成后再开启，避免花屏）────────────────
    bsp_lcd_bk_set_percent(0); // 背光关闭（PWM 占空比 0）

    // ── 步骤 6：硬件复位显示屏 ────────────────────────────────────────────────
    // RST 引脚拉低一段时间，复位 ST7789 内部寄存器到出厂默认值
    ESP_ERROR_CHECK(esp_lcd_panel_reset(bsp_board->lcd_panel));

    // ── 步骤 7：初始化 ST7789 面板驱动（写入初始化寄存器序列）──────────────
    // 内部向 ST7789 发送约 20 条初始化命令，设置显示方向、颜色格式等
    ESP_ERROR_CHECK(esp_lcd_panel_init(bsp_board->lcd_panel));

    // ── 步骤 8：颜色反转（IPS 屏必须开启，否则颜色像底片负片）──────────────
    // 普通 TN 屏不需要反转，IPS 屏（本设备使用）必须开启此选项
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(bsp_board->lcd_panel, true));

    // 可选配置（根据屏幕安装方向决定是否开启镜像/轴交换）:
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(bsp_board->lcd_panel, true, false));
    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(bsp_board->lcd_panel, true));

    // ── 步骤 9：关闭显示（等待上层主动调用 bsp_board_lcd_on() 开启）────────
    // 初始化完成但不立即显示，让上层决定何时打开（可以先准备好画面再开背光）
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, false));
    xEventGroupSetBits(bsp_board->board_status, LCD_BIT); // 置位 LCD_BIT，通知其他模块 LCD 已就绪
}

/**
 * @brief 打开 LCD 背光和显示
 *
 * 先启用显示控制器输出，再点亮背光 LED，避免背光先亮时看到初始化过渡帧。
 *
 * @param bsp_board BSP 实例指针（访问 lcd_panel 句柄和背光 GPIO）
 * @return void
 *
 * @note 调用者：application.c 或业务层（需要显示时调用）
 * @note 前置条件：bsp_board_lcd_init() 已成功调用
 */
void bsp_board_lcd_on(bsp_board_t *bsp_board)
{
    // 先启用 ST7789 显示输出（DISPON 命令），再点亮背光
    // 顺序：控制器输出 → 背光点亮，避免背光亮时显示未就绪的画面
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, true)); // 启用显示
    bsp_lcd_bk_set_percent(BSP_LCD_BK_DEFAULT_PCT);                        // 点亮背光（默认 100%）
}

/**
 *
 * @brief 关闭 LCD 背光和显示
 *
 * 先关闭背光 LED，再关闭显示控制器，省电效果最佳。
 *
 * @param bsp_board BSP 实例指针（访问 lcd_panel 句柄和背光 GPIO）
 * @return void
 *
 * @note 调用者：业务层（休眠/省电时调用）
 * @note 前置条件：bsp_board_lcd_init() 已成功调用
 */
void bsp_board_lcd_off(bsp_board_t *bsp_board)
{
    // 先关背光（用户立即看不到画面），再关显示控制器
    bsp_lcd_bk_set_percent(0);                                              // 关闭背光（PWM 占空比 0）
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, false)); // 关闭显示
}
