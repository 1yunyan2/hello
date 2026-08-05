/**
 * @file main.c
 * @brief 程序入口
 * ESP-IDF 框架要求的 app_main() 入口函数，仅负责调用应用层初始化
 */

#include "application.h"
#include "heap_forensic.h" // 【堆损坏排查·2026-07-29】纯诊断，不参与业务
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
extern void apppp_main(void);

/**
 * @brief ESP-IDF 程序入口
 * 系统启动后由 FreeRTOS 主任务调用，所有业务逻辑在 application_init() 中展开
 *
 * 注：三级关机不再"借重启断电"。实测 GPIO18(OPT) 高→低下降沿即可让 HK015T 在运行态
 *     直接断电（见 standby.c enter_shutdown → bsp_battery_power_off），故此处不再需要
 *     开机最早期消费 RTC 魔数拉脚的逻辑。
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
    /* 【堆损坏排查·2026-07-29】曾在此启动 heap tracing 记录分配调用栈，已停用：
     * CONFIG_HEAP_TRACING_STANDALONE 会给每次 malloc/free 加记录开销，把 MultiNet
     * FST 编译从约 5s 拖到 46s（与 BUG-027 记录的堆毒化拖慢同源），代价过大。
     * sdkconfig 已还原为 CONFIG_HEAP_TRACING_OFF；heap_forensic.c/h 保留备用，
     * 需要时重开配置并取消下面一行注释即可。 */
    // heap_forensic_start(2000);

    application_init();
    // apppp_main();
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}