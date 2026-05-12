// #include <stdio.h>
// #include <string.h>
// #include "esp_log.h"
// #include "esp_flash.h"
// #include "esp_flash_spi_init.h"
// #include "esp_vfs_fat.h"
// #include "driver/spi_common.h"
// #include "esp_partition.h"
// #include "esp_timer.h" // 用 esp_timer_get_time() 算擦除耗时
// // ★ 改用官方 USJ 驱动:中断 + 软件环形缓冲,吞吐 ~500KB/s 起。
// //   旧的 LL 直戳 FIFO 在 FREERTOS_HZ=100 下被 vTaskDelay(1)=10ms 拖死,
// //   叠加 USJ RX FIFO 仅 64B,host→device 实测 ~6.4KB/s,
// //   PC 端 ser.write(128KB) 必然超过 15s write_timeout 而 Write timeout。
// #include "driver/usb_serial_jtag.h"
// #include "driver/usb_serial_jtag_vfs.h" // 把 printf/ESP_LOG 也接到驱动上,避免 TX 抢 FIFO
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "bsp/bsp_config.h"

// static const char *TAG = "PROD_BURNER";
// esp_flash_t *ext_flash = NULL;
// static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

// #define BURNER_CHUNK (128 * 1024)           // 128KB 缓冲区，适配 2MB 空闲 PSRAM
// #define TOTAL_FLASH_SIZE (32 * 1024 * 1024) // 32MB 全量大小
// // USJ 驱动软件环形缓冲;RX 给大点是因为 host 一次能塞 128KB,
// // 缓冲越大,esp_flash_write 那段时间的 USB 反压发生越晚,吞吐越平稳。
// #define USJ_RX_BUF_SIZE (16 * 1024)
// #define USJ_TX_BUF_SIZE (4 * 1024)

// // 擦除进度上报:esp_flash_erase_region 是单次原子操作(芯片擦除指令 0xC7),
// // 无法实时拿真实进度,只能按经验值线性估算百分比 —— 真擦完时会强制跳到 100%。
// #define ERASE_ESTIMATE_SEC 81 // 32MB W25Q 整片擦除典型耗时,按你板子实测调
// static volatile bool s_erase_in_progress = false;
// static int64_t s_erase_start_us = 0;

// static void erase_progress_task(void *arg)
// {
//     while (s_erase_in_progress)
//     {
//         vTaskDelay(pdMS_TO_TICKS(10000));
//         if (!s_erase_in_progress)
//             break;
//         int elapsed_s = (int)((esp_timer_get_time() - s_erase_start_us) / 1000000);
//         int pct = (elapsed_s * 100) / ERASE_ESTIMATE_SEC;
//         if (pct > 99)
//             pct = 99; // 别先冲到 100,留给真擦完时打
//         ESP_LOGW(TAG, "  ⏳ 擦除中... %3d%% (已用 %2ds / 预计 %ds)",
//                  pct, elapsed_s, ERASE_ESTIMATE_SEC);
//     }
//     vTaskDelete(NULL);
// }

// /**
//  * @brief 核心烧录函数：流式接收并写入外部 Flash
//  */
// void start_production_burning(void)
// {
//     uint8_t *buffer = (uint8_t *)heap_caps_malloc(BURNER_CHUNK, MALLOC_CAP_SPIRAM);
//     if (!buffer)
//     {
//         ESP_LOGE(TAG, "PSRAM 内存不足！");
//         return;
//     }
//     // 🚀 核心改动：先告诉电脑"我开始擦除了"，让 Python 脚本别慌
//     printf("STARTING_ERASE\n");
//     fflush(stdout);
//     vTaskDelay(10); // 确保数据发出
//     // 1. 物理擦除：32MB 全盘擦除（必须在挂载前执行）
//     ESP_LOGW(TAG, "正在全盘擦除 (约 %ds)，请勿断电...", ERASE_ESTIMATE_SEC);
//     s_erase_start_us = esp_timer_get_time();
//     s_erase_in_progress = true;
//     xTaskCreate(erase_progress_task, "erase_prog", 3072, NULL, 1, NULL);

//     esp_flash_erase_region(ext_flash, 0, TOTAL_FLASH_SIZE);

//     s_erase_in_progress = false;
//     int64_t erase_total_ms = (esp_timer_get_time() - s_erase_start_us) / 1000;
//     ESP_LOGW(TAG, "✅ 擦除完成 100%% — 实际耗时 %lld.%03lld 秒",
//              erase_total_ms / 1000, erase_total_ms % 1000);

//     // 2. 握手信号
//     printf("READY_FOR_DATA\n");
//     fflush(stdout);

//     uint32_t offset = 0;
//     while (offset < TOTAL_FLASH_SIZE)
//     {
//         int received = 0;
//         while (received < BURNER_CHUNK)
//         {
//             // 走驱动:ISR 持续把 USJ RX FIFO 排空到 16KB 软件环形缓冲,
//             // 这里 read_bytes 直接从环形缓冲取,不再受 64B FIFO + 10ms tick 拖累。
//             int len = usb_serial_jtag_read_bytes(buffer + received,
//                                                  BURNER_CHUNK - received,
//                                                  pdMS_TO_TICKS(5000));
//             if (len > 0)
//                 received += len;
//         }

//         // 直接进行物理扇区写入，不经过文件系统
//         esp_flash_write(ext_flash, buffer, offset, BURNER_CHUNK);
//         offset += BURNER_CHUNK;

//         // 给 Python 脚本反馈进度
//         printf("ACK:%lu\n", offset);
//         fflush(stdout);
//     }

//     ESP_LOGI(TAG, "✅ 32MB 资源同步成功！设备即将重启...");
//     heap_caps_free(buffer);
//     vTaskDelay(pdMS_TO_TICKS(1000));
//     esp_restart(); // 重启后进入正常挂载逻辑
// }

// // 1. 把这个提取函数加在 start_production_burning() 的下面
// void start_production_dump(void)
// {
//     // 申请 PSRAM 缓冲区，减轻内部 SRAM 压力
//     uint8_t *buffer = (uint8_t *)heap_caps_malloc(128 * 1024, MALLOC_CAP_SPIRAM);
//     if (!buffer)
//     {
//         ESP_LOGE(TAG, "PSRAM 内存不足！");
//         return;
//     }

//     printf("READY_TO_DUMP\n");
//     fflush(stdout);
//     vTaskDelay(pdMS_TO_TICKS(200)); // 给电脑预留准备时间

//     // 关掉所有 ESP_LOG，防止后台任务的日志混进二进制流污染镜像
//     esp_log_level_set("*", ESP_LOG_NONE);

//     uint32_t offset = 0;
//     while (offset < TOTAL_FLASH_SIZE)
//     {
//         // 1. 从 Flash 读取一块数据
//         esp_flash_read(ext_flash, buffer, offset, 128 * 1024);

//         // 2. 走驱动写出。注意:LF→CRLF 翻译只发生在 VFS 层(printf 那条路径),
//         //    write_bytes 是直通驱动的二进制接口,不会被改写。
//         usb_serial_jtag_write_bytes(buffer, 128 * 1024, portMAX_DELAY);

//         offset += 128 * 1024;
//     }

//     ESP_LOGI(TAG, "✅ 黄金镜像已完整吐出，正在重启...");
//     heap_caps_free(buffer);
//     vTaskDelay(pdMS_TO_TICKS(1000));
//     esp_restart();
// }

// /**
//  * @brief 统一初始化入口
//  */
// void bsp_ext_flash_init(void)
// {
//     // --- 步骤 0: 安装 USB-Serial-JTAG 驱动(必须最先做,且只装一次)---
//     // 旧实现刻意不装,是因为作者担心 driver TX 和 secondary console (printf)
//     // 同时戳 HW FIFO 出现交错。正确做法是装完之后调用 vfs_use_driver(),
//     // 让 printf/ESP_LOG 也排队走驱动 TX,从根上消除冲突。
//     usb_serial_jtag_driver_config_t usj_cfg = {
//         .rx_buffer_size = USJ_RX_BUF_SIZE,
//         .tx_buffer_size = USJ_TX_BUF_SIZE,
//     };
//     ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));
//     usb_serial_jtag_vfs_use_driver();

//     // --- 步骤 1: 硬件底层初始化 (SPI3) ---
//     const spi_bus_config_t bus_config = {
//         .mosi_io_num = EXT_FLASH_MOSI,
//         .miso_io_num = EXT_FLASH_MISO,
//         .sclk_io_num = EXT_FLASH_SCLK,
//         .quadwp_io_num = -1,
//         .quadhd_io_num = -1,
//     };
//     ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));

//     const esp_flash_spi_device_config_t device_config = {
//         .host_id = SPI3_HOST,
//         .cs_id = 0,
//         .cs_io_num = EXT_FLASH_CS,
//         .io_mode = SPI_FLASH_SLOWRD,
//         .freq_mhz = 20,
//     };
//     ESP_ERROR_CHECK(spi_bus_add_flash_device(&ext_flash, &device_config));
//     ESP_ERROR_CHECK(esp_flash_init(ext_flash));

//     // --- 步骤 2: 产线双模监听 (10秒) ---
//     ESP_LOGI(TAG, "📢 产线模式开启！发送 'START' 烧录，发送 'DUMP' 提取...");

//     char cmd[32] = {0};
//     uint32_t wait_ms = 0;
//     int mode = 0; // 0: 正常模式, 1: 烧录模式, 2: 提取模式

//     while (wait_ms < 10000)
//     {
//         // 每 2s 喊话一次（10s 内 5 次），够 Python 同步且不刷屏
//         if (wait_ms % 2000 == 0)
//         {
//             printf("\nESP32_READY_CMD_WAIT\n"); // Python 握手锚点，勿删
//             fflush(stdout);                     // 确保数据发出
//         }

//         memset(cmd, 0, sizeof(cmd));
//         int len = usb_serial_jtag_read_bytes((uint8_t *)cmd, sizeof(cmd) - 1,
//                                              pdMS_TO_TICKS(10));
//         if (len > 0)
//         {
//             if (strstr(cmd, "START"))
//             {
//                 mode = 1; // 触发你的原始烧录逻辑
//                 break;
//             }
//             else if (strstr(cmd, "DUMP"))
//             {
//                 mode = 2; // 触发现在的提取逻辑
//                 break;
//             }
//         }
//         wait_ms += 10;
//     }

//     if (mode == 1)
//     {
//         printf("\n>>> 进入烧录模式 (PC -> Flash) <<<\n");
//         start_production_burning(); // 调用你原有的烧录函数
//     }
//     else if (mode == 2)
//     {
//         printf("\n>>> 进入提取模式 (Flash -> PC) <<<\n");
//         start_production_dump(); // 调用我们新写的吐数据函数
//     }

//     else
//     {
//         // --- 步骤 3: 正常启动流程 (挂载文件系统) ---
//         uint32_t flash_size;
//         esp_flash_get_size(ext_flash, &flash_size);
//         const esp_partition_t *fat_partition;

//         ESP_ERROR_CHECK(esp_partition_register_external(ext_flash, 0, flash_size, "ext_storage",
//                                                         ESP_PARTITION_TYPE_DATA,
//                                                         ESP_PARTITION_SUBTYPE_DATA_FAT,
//                                                         &fat_partition));

//         const esp_vfs_fat_mount_config_t mount_config = {
//             // ⚠️ 必须为 false：true 会在任何挂载错误时静默格式化整盘，
//             //    用户上传的 GIF/audio 资源会无声丢失。失败时改走下面的
//             //    NEED_BURN 分支，等待主动 START 烧录。
//             .format_if_mount_failed = false,
//             .max_files = 5,
//             .allocation_unit_size = 4096};
//         esp_err_t mount_ret = esp_vfs_fat_spiflash_mount_rw_wl("/S", "ext_storage", &mount_config, &s_wl_handle);
//         if (mount_ret == ESP_OK)
//         {
//             ESP_LOGI(TAG, "外部 Flash 正常挂载到 /S");
//         }
//         else
//         {
//             // 没有有效 FAT 文件系统（首次上电或镜像无效）
//             // 不要 abort，否则会陷入重启循环导致 burner.py 无法接入
//             ESP_LOGW(TAG, "挂载失败 (0x%x)，检查镜像格式或手动发送 START 进入烧录模式。...", mount_ret);
//             printf("NEED_BURN\n"); // 让 burner.py 知道可以直接发数据
//                                    // start_production_burning(); // 内部含 esp_restart，不会返回
//                                    // 建议在这里进入一个死循环或者等待指令的任务，而不是直接重启
//             while (1)
//             {
//                 vTaskDelay(pdMS_TO_TICKS(1000));
//             }
//         }
//     }
// }
