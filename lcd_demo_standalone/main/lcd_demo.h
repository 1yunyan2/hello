#pragma once

/**
 * @file lcd_demo.h
 * @brief ST7789 LCD 独立示例（320×240 横屏，SPI，RGB565，无 LVGL）
 */

#include <stdint.h>
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"

// ST7789 屏幕参数（横屏）
#define LCD_DEMO_WIDTH 320
#define LCD_DEMO_HEIGHT 240

// RGB565 常用颜色（小端，与 panel data_endian=LITTLE 一致）
#define LCD_COLOR_BLACK 0x0000
#define LCD_COLOR_WHITE 0xFFFF
#define LCD_COLOR_RED 0xF800
#define LCD_COLOR_GREEN 0x07E0
#define LCD_COLOR_BLUE 0x001F
#define LCD_COLOR_YELLOW 0xFFE0
#define LCD_COLOR_CYAN 0x07FF
#define LCD_COLOR_MAGENTA 0xF81F

// LCD 句柄聚合体（SPI IO + 面板驱动）
typedef struct
{
    esp_lcd_panel_io_handle_t io;
    esp_lcd_panel_handle_t panel;
} lcd_demo_handle_t;

// 初始化 SPI 总线 + ST7789 面板，并打开显示与背光
void lcd_demo_init(lcd_demo_handle_t *h);

// 用单一颜色全屏填充
void lcd_demo_fill(lcd_demo_handle_t *h, uint16_t color);

// 示例画面：黑底 + 4 个不同颜色的矩形色块（左上红/右上绿/左下蓝/右下黄）
void lcd_demo_show_blocks(lcd_demo_handle_t *h);
