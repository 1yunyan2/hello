/**
 * @file lcd_demo.c
 * @brief ST7789 LCD 独立示例实现（无 LVGL，无业务框架依赖）
 *
 * 引脚（ESP32-S3）：
 *   CS=41, MOSI=39, SCLK=40, DC=38, RST=45, BK=42
 * SPI 时钟 40 MHz，色深 RGB565，小端字节序。
 */

#include "lcd_demo.h"

#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_err.h"
#include "esp_log.h"

// ─── LCD 引脚定义（如接收方板子引脚不同，改这里即可）────────────────────────
#define LCD_DEMO_CS_PIN 41
#define LCD_DEMO_MOSI_PIN 39
#define LCD_DEMO_SCLK_PIN 40
#define LCD_DEMO_DC_PIN 38
#define LCD_DEMO_RST_PIN 45
#define LCD_DEMO_BK_PIN 42

#define LCD_DEMO_SPI_HOST SPI2_HOST
#define LCD_DEMO_SPI_CLK_HZ (40 * 1000 * 1000) // 40 MHz

static const char *TAG = "lcd_demo";

// ════════════════════════════════════════════════════════════════════════════
// 初始化
// ════════════════════════════════════════════════════════════════════════════
void lcd_demo_init(lcd_demo_handle_t *h)
{
    // 步骤 1：释放可能被其它驱动占用的 GPIO
    gpio_reset_pin(LCD_DEMO_SCLK_PIN);
    gpio_reset_pin(LCD_DEMO_MOSI_PIN);
    gpio_reset_pin(LCD_DEMO_DC_PIN);
    gpio_reset_pin(LCD_DEMO_CS_PIN);
    gpio_reset_pin(LCD_DEMO_BK_PIN);
    gpio_reset_pin(LCD_DEMO_RST_PIN);

    // 步骤 2：背光 GPIO 推挽输出，初始关闭（避免初始化过程花屏）
    gpio_config_t bk_cfg = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << LCD_DEMO_BK_PIN,
    };
    ESP_ERROR_CHECK(gpio_config(&bk_cfg));
    gpio_set_level(LCD_DEMO_BK_PIN, 0);

    // 步骤 3：初始化 SPI2 总线（DMA 自动分配）
    spi_bus_config_t buscfg = {
        .sclk_io_num = LCD_DEMO_SCLK_PIN,
        .mosi_io_num = LCD_DEMO_MOSI_PIN,
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_DEMO_WIDTH * LCD_DEMO_HEIGHT * 2 + 8,
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_DEMO_SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
    ESP_LOGI(TAG, "SPI bus init OK");

    // 步骤 4：创建 SPI LCD IO 接口
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = LCD_DEMO_DC_PIN,
        .cs_gpio_num = LCD_DEMO_CS_PIN,
        .pclk_hz = LCD_DEMO_SPI_CLK_HZ,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)LCD_DEMO_SPI_HOST, &io_config, &h->io));

    // 步骤 5：创建 ST7789 面板驱动
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_DEMO_RST_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(h->io, &panel_config, &h->panel));

    // 步骤 6：复位 → 初始化
    ESP_ERROR_CHECK(esp_lcd_panel_reset(h->panel));
    ESP_ERROR_CHECK(esp_lcd_panel_init(h->panel));

    // 步骤 6.1：ST7789 原生 240×320 竖屏，swap_xy + mirror 转为 320×240 横屏
    // 如果屏幕方向不对（颠倒/镜像），调整下面两行的 true/false
    ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(h->panel, true));
    ESP_ERROR_CHECK(esp_lcd_panel_mirror(h->panel, true, false));
    // ESP_ERROR_CHECK(esp_lcd_panel_set_gap(h->panel, 0, 0));  // 部分模组需偏移

    // 步骤 6.2：颜色反转 —— IPS 屏一般 true，TN 屏 false
    // 如果颜色完全颠倒（红绿蓝反 / 黑白对调），改成 false
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(h->panel, true));

    // 步骤 6.3：开显示
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(h->panel, true));
    ESP_LOGI(TAG, "ST7789 panel init OK");

    // 步骤 7：打开背光
    gpio_set_level(LCD_DEMO_BK_PIN, 1);
    ESP_LOGI(TAG, "backlight on, display ready");
}

// ════════════════════════════════════════════════════════════════════════════
// 全屏填充：逐行写，避免一次性申请整帧 DMA 缓冲（150KB 太大）
// ════════════════════════════════════════════════════════════════════════════
void lcd_demo_fill(lcd_demo_handle_t *h, uint16_t color)
{
    static uint16_t line_buf[LCD_DEMO_WIDTH]; // 320×2 = 640 B 静态行缓冲
    for (int i = 0; i < LCD_DEMO_WIDTH; i++)
    {
        line_buf[i] = color;
    }
    for (int y = 0; y < LCD_DEMO_HEIGHT; y++)
    {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
            h->panel, 0, y, LCD_DEMO_WIDTH, y + 1, line_buf));
    }
}

// ════════════════════════════════════════════════════════════════════════════
// 画实心矩形（逐行写，静态行缓冲，避免 DMA 异步释放风险）
// ════════════════════════════════════════════════════════════════════════════
static void draw_rect(lcd_demo_handle_t *h,
                      int x, int y, int w, int hgt, uint16_t color)
{
    static uint16_t row_buf[LCD_DEMO_WIDTH];
    if (w > LCD_DEMO_WIDTH)
        w = LCD_DEMO_WIDTH;

    for (int i = 0; i < w; i++)
        row_buf[i] = color;

    for (int j = 0; j < hgt; j++)
    {
        ESP_ERROR_CHECK(esp_lcd_panel_draw_bitmap(
            h->panel, x, y + j, x + w, y + j + 1, row_buf));
    }
}

// ════════════════════════════════════════════════════════════════════════════
// 示例画面：黑底 + 四象限色块
// ════════════════════════════════════════════════════════════════════════════
void lcd_demo_show_blocks(lcd_demo_handle_t *h)
{
    lcd_demo_fill(h, LCD_COLOR_BLACK);

    // 画布 320×240，每块 100×80，离边 20px
    draw_rect(h, 20, 20, 100, 80, LCD_COLOR_RED);      // 左上
    draw_rect(h, 200, 20, 100, 80, LCD_COLOR_GREEN);   // 右上
    draw_rect(h, 20, 140, 100, 80, LCD_COLOR_BLUE);    // 左下
    draw_rect(h, 200, 140, 100, 80, LCD_COLOR_YELLOW); // 右下

    ESP_LOGI(TAG, "show_blocks done");
}
