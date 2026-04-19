/**
 * @file main.c
 * @brief 程序入口
 * ESP-IDF 框架要求的 app_main() 入口函数，仅负责调用应用层初始化
 */

#include "application.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"

/**
 * @brief ESP-IDF 程序入口
 * 系统启动后由 FreeRTOS 主任务调用，所有业务逻辑在 application_init() 中展开
 */
void app_main(void)
{
    // ESP32-S3 首次烧录后 PSRAM 驱动有时未完全初始化（bootloader 需一次复位才能稳定）。
    // AFE 模型和音频缓冲区大量依赖 PSRAM，PSRAM 异常时 AFE 静默失败导致无法唤醒。
    // 检测可用 PSRAM：正常 ≥ 2MB，异常（< 1MB）时主动重启，让 bootloader 重新初始化。
    size_t psram_free = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    if (psram_free < 1 * 1024 * 1024)
    {
        ESP_LOGE("BOOT", "PSRAM异常(仅%u字节可用)，自动重启以完成初始化", (unsigned)psram_free);
        esp_restart();
    }

    application_init();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}