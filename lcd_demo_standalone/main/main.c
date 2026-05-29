/**
 * @file main.c
 * @brief LCD demo 入口
 *
 * 预期现象：屏幕黑底，4 个 100×80 矩形——
 *   左上红、右上绿、左下蓝、右下黄。
 */

#include "lcd_demo.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "main";

void app_main(void)
{
    ESP_LOGI(TAG, "LCD demo start");

    static lcd_demo_handle_t lcd;

    lcd_demo_init(&lcd);        // 初始化 SPI + ST7789 + 背光
    lcd_demo_show_blocks(&lcd); // 绘制示例画面

    ESP_LOGI(TAG, "LCD demo done, idle forever");

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
