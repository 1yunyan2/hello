// test_wifi_time.c — 最小测试：WiFi + SNTP时间 + 闹钟 + LCD
#include "bsp/bsp_board.h"
#include "ui/reminder.h"
#include "ui/ui_port.h"

void apppp_main(void)
{
    // 1. NVS 初始化（WiFi凭证存在这里）
    bsp_board_t *board = bsp_board_get_instance();
    bsp_board_nvs_init(board); // 置位 NVS_BIT

    // 2. WiFi 连接（阻塞直到连上或失败重启）
    bsp_board_wifi_main(board); // 置位 WIFI_BIT

    // 3. 提醒系统（SNTP自动同步 + 闹钟引擎）
    reminder_init(NULL);

    bsp_board_lcd_init(board); // LCD 初始化（当前未自动置位 LCD_BIT，后续可根据需求调整）
    ui_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    bsp_board_lcd_on(board);
    // 6. 创建触摸扫描任务（栈分配在PSRAM，节省内部SRAM）
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        touch_scan_task,
        "touch_scan",
        8192,
        NULL,
        4, // 优先级略低于舵机和音频
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
    {
        printf("创建触摸扫描任务失败\n");
    }
    else
    {
        printf("触摸扫描任务创建完成\n");
    }
    ret = interaction_manager_init();
    if (ret != ESP_OK)
    {
        printf("创建交互管理器失败\n");
    }
}