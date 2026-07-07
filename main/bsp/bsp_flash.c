#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "esp_flash.h"
#include "esp_flash_spi_init.h"
#include "esp_vfs_fat.h"
#include "driver/spi_common.h"
#include "esp_partition.h"
#include "esp_timer.h" // 用 esp_timer_get_time() 算擦除耗时
// ★ 改用官方 USJ 驱动:中断 + 软件环形缓冲,吞吐 ~500KB/s 起。
//   旧的 LL 直戳 FIFO 在 FREERTOS_HZ=100 下被 vTaskDelay(1)=10ms 拖死,
//   叠加 USJ RX FIFO 仅 64B,host→device 实测 ~6.4KB/s,
//   PC 端 ser.write(128KB) 必然超过 15s write_timeout 而 Write timeout。
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h" // 把 printf/ESP_LOG 也接到驱动上,避免 TX 抢 FIFO
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "bsp/bsp_config.h"
#include "bsp/bsp_board.h"
static const char *TAG = "PROD_BURNER";
esp_flash_t *ext_flash = NULL;
static wl_handle_t s_wl_handle = WL_INVALID_HANDLE;

#define BURNER_CHUNK (128 * 1024)           // 128KB 缓冲区，适配 2MB 空闲 PSRAM
#define TOTAL_FLASH_SIZE (32 * 1024 * 1024) // 32MB 全量大小
// USJ 驱动软件环形缓冲;RX 给大点是因为 host 一次能塞 128KB,
// 缓冲越大,esp_flash_write 那段时间的 USB 反压发生越晚,吞吐越平稳。
#define USJ_RX_BUF_SIZE (16 * 1024)
#define USJ_TX_BUF_SIZE (4 * 1024)

// ── 强制烧录窗口 ──
// 已烧录成品板挂载必成功、会零延迟跳过产线，导致无法再重烧外挂资源。
// 为此在「挂载之前」开一个短窗口监听 PC 命令：窗口内 3.py 发来 START/DUMP
// 就强制进产线重烧/提取；没命令则窗口结束、继续正常挂载启动。
// 用法：电脑先跑 3.py（它会等设备），再按板子 RST，窗口内即可握手重烧。
// 设为 0 可彻底关闭本窗口、恢复纯零延迟启动（届时已烧板将无法再重烧）。
// 取 3s 是为给 PC 端 3.py（连接后约 1.5s 才开始读）留足握手余量。
#define BURN_FORCE_WINDOW_MS 10 //! 烧录模式需要，运行模式不需要

// 擦除进度上报:esp_flash_erase_region 是单次原子操作(芯片擦除指令 0xC7),
// 无法实时拿真实进度,只能按经验值线性估算百分比 —— 真擦完时会强制跳到 100%。
#define ERASE_ESTIMATE_SEC 35 // 32MB W25Q 整片擦除典型耗时,按你板子实测调
static volatile bool s_erase_in_progress = false;
static int64_t s_erase_start_us = 0;

static void erase_progress_task(void *arg)
{
    while (s_erase_in_progress)
    {
        vTaskDelay(pdMS_TO_TICKS(10000));
        if (!s_erase_in_progress)
            break;
        int elapsed_s = (int)((esp_timer_get_time() - s_erase_start_us) / 1000000);
        int pct = (elapsed_s * 100) / ERASE_ESTIMATE_SEC;
        if (pct > 99)
            pct = 99; // 别先冲到 100,留给真擦完时打
        ESP_LOGW(TAG, "  ⏳ 擦除中... %3d%% (已用 %2ds / 预计 %ds)",
                 pct, elapsed_s, ERASE_ESTIMATE_SEC);
    }
    vTaskDelete(NULL);
}

/**
 * @brief 核心烧录函数：流式接收并写入外部 Flash
 */
void start_production_burning(void)
{
    uint8_t *buffer = (uint8_t *)heap_caps_malloc(BURNER_CHUNK, MALLOC_CAP_SPIRAM);
    if (!buffer)
    {
        ESP_LOGE(TAG, "PSRAM 内存不足！");
        return;
    }
    // 🚀 核心改动：先告诉电脑"我开始擦除了"，让 Python 脚本别慌
    printf("STARTING_ERASE\n");
    fflush(stdout);
    vTaskDelay(10); // 确保数据发出
    // 1. 物理擦除：32MB 全盘擦除（必须在挂载前执行）
    ESP_LOGW(TAG, "正在全盘擦除 (约 %ds)，请勿断电...", ERASE_ESTIMATE_SEC);
    s_erase_start_us = esp_timer_get_time();
    s_erase_in_progress = true;
    xTaskCreate(erase_progress_task, "erase_prog", 3072, NULL, 1, NULL);

    esp_flash_erase_region(ext_flash, 0, TOTAL_FLASH_SIZE);

    s_erase_in_progress = false;
    int64_t erase_total_ms = (esp_timer_get_time() - s_erase_start_us) / 1000;
    ESP_LOGW(TAG, "✅ 擦除完成 100%% — 实际耗时 %lld.%03lld 秒",
             erase_total_ms / 1000, erase_total_ms % 1000);

    // 2. 握手信号
    printf("READY_FOR_DATA\n");
    fflush(stdout);

    uint32_t offset = 0;
    while (offset < TOTAL_FLASH_SIZE)
    {
        int received = 0;
        while (received < BURNER_CHUNK)
        {
            // 走驱动:ISR 持续把 USJ RX FIFO 排空到 16KB 软件环形缓冲,
            // 这里 read_bytes 直接从环形缓冲取,不再受 64B FIFO + 10ms tick 拖累。
            int len = usb_serial_jtag_read_bytes(buffer + received,
                                                 BURNER_CHUNK - received,
                                                 pdMS_TO_TICKS(5000));
            if (len > 0)
                received += len;
        }

        // 直接进行物理扇区写入，不经过文件系统
        esp_flash_write(ext_flash, buffer, offset, BURNER_CHUNK);
        offset += BURNER_CHUNK;

        // 给 Python 脚本反馈进度
        printf("ACK:%lu\n", offset);
        fflush(stdout);
    }

    ESP_LOGI(TAG, "✅ 32MB 资源同步成功！设备即将重启...");
    heap_caps_free(buffer);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart(); // 重启后进入正常挂载逻辑
}

// 1. 把这个提取函数加在 start_production_burning() 的下面
void start_production_dump(void)
{
    // 申请 PSRAM 缓冲区，减轻内部 SRAM 压力
    uint8_t *buffer = (uint8_t *)heap_caps_malloc(128 * 1024, MALLOC_CAP_SPIRAM);
    if (!buffer)
    {
        ESP_LOGE(TAG, "PSRAM 内存不足！");
        return;
    }

    printf("READY_TO_DUMP\n");
    fflush(stdout);
    vTaskDelay(pdMS_TO_TICKS(200)); // 给电脑预留准备时间

    // 关掉所有 ESP_LOG，防止后台任务的日志混进二进制流污染镜像
    esp_log_level_set("*", ESP_LOG_NONE);

    uint32_t offset = 0;
    while (offset < TOTAL_FLASH_SIZE)
    {
        // 1. 从 Flash 读取一块数据
        esp_flash_read(ext_flash, buffer, offset, 128 * 1024);

        // 2. 走驱动写出。注意:LF→CRLF 翻译只发生在 VFS 层(printf 那条路径),
        //    write_bytes 是直通驱动的二进制接口,不会被改写。
        usb_serial_jtag_write_bytes(buffer, 128 * 1024, portMAX_DELAY);

        offset += 128 * 1024;
    }

    ESP_LOGI(TAG, "✅ 黄金镜像已完整吐出，正在重启...");
    heap_caps_free(buffer);
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

/**
 * @brief 统一初始化入口
 */
void bsp_flash_init(void)
{
    // --- 步骤 1: 硬件底层初始化 (SPI3) ---
    const spi_bus_config_t bus_config = {
        .mosi_io_num = EXT_FLASH_MOSI,
        .miso_io_num = EXT_FLASH_MISO,
        .sclk_io_num = EXT_FLASH_SCLK,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
    };
    // ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO));
    esp_err_t ret = spi_bus_initialize(SPI3_HOST, &bus_config, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "SPI 总线初始化失败，跳过外挂 flash (ret=%d)", ret);
        return;
    }
    const esp_flash_spi_device_config_t device_config = {
        .host_id = SPI3_HOST,
        .cs_id = 0,
        .cs_io_num = EXT_FLASH_CS,
        .io_mode = SPI_FLASH_SLOWRD,
        .freq_mhz = 40,
    };
    // ESP_ERROR_CHECK(spi_bus_add_flash_device(&ext_flash, &device_config));
    // ESP_ERROR_CHECK(esp_flash_init(ext_flash));
    ret = spi_bus_add_flash_device(&ext_flash, &device_config);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "未检测到外挂 flash，跳过产线模式");
        return;
    }
    ret = esp_flash_init(ext_flash);
    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "外挂 flash init 失败 (ret=0x%x)，跳过产线模式", ret);

        /* ── JEDEC ID 诊断 ──────────────────────────────────────────────────
         * init 失败时，芯片句柄（spi_bus_add_flash_device 已挂上）仍可用，
         * 这里直接读一次厂商/容量 ID，用原始值区分故障类型，避免反复盲猜硬件：
         *   - 0xFFFFFF  → MISO 恒高：芯片没焊上 / DO 断线 / 没供电（总线被弱上拉拉满）
         *   - 0x000000  → MISO 恒低：芯片被 HOLD#/RESET# 拉低挂起 / DO 对地短路
         *   - 其它正常值 → 总线通了但握手没过，多半是 freq/io_mode 或电源纹波问题
         * ID 字节序：bit23..16=厂商, bit15..8=存储类型, bit7..0=容量
         * （W25Q 厂商=0xEF，如 0xEF4019=W25Q256 32MB）
         * 详见 memory/bugs 中 BUG-016 同类虚焊指纹。 */
        uint32_t jedec_id = 0;
        esp_err_t id_ret = esp_flash_read_id(ext_flash, &jedec_id);
        if (id_ret != ESP_OK)
        {
            ESP_LOGE(TAG, "[FLASH诊断] 连 JEDEC ID 都读不出 (ret=0x%x)，总线层就没通", id_ret);
        }
        else if (jedec_id == 0x000000 || jedec_id == 0xFFFFFF)
        {
            ESP_LOGE(TAG, "[FLASH诊断] JEDEC ID=0x%06lX (恒%s) → 芯片无响应："
                          "查 HOLD#/WP# 上拉、CS/CLK/DI/DO 焊接、VCC 供电",
                     (unsigned long)jedec_id, jedec_id ? "高" : "低");
        }
        else
        {
            ESP_LOGW(TAG, "[FLASH诊断] JEDEC ID=0x%06lX (厂商=0x%02lX 容量=0x%02lX) → "
                          "总线已通但握手未过，查 freq/io_mode/电源纹波",
                     (unsigned long)jedec_id,
                     (unsigned long)((jedec_id >> 16) & 0xFF),
                     (unsigned long)(jedec_id & 0xFF));
        }

        ext_flash = NULL;
        return;
    }
    // --- 步骤 2: 先尝试挂载文件系统（成品板零延迟启动） ---
    // 设计要点：只有"挂载失败"才意味着这块板没有有效镜像（新板/烧坏的板），
    // 才需要进入产线监听等 burner。已经烧录成功的成品板挂载必然成功，
    // 这里直接 return，不再像旧逻辑那样无条件硬等 10 秒产线窗口。
    uint32_t flash_size;
    esp_flash_get_size(ext_flash, &flash_size);
    const esp_partition_t *fat_partition;

    ESP_ERROR_CHECK(esp_partition_register_external(ext_flash, 0, flash_size, "ext_storage",
                                                    ESP_PARTITION_TYPE_DATA,
                                                    ESP_PARTITION_SUBTYPE_DATA_FAT,
                                                    &fat_partition));

    // --- 步骤 2.5: 强制烧录窗口（挂载前先开短窗口监听 PC 命令）---
    // 见顶部 BURN_FORCE_WINDOW_MS 说明。窗口内收到 START/DUMP 即强制进产线，
    // 跳过挂载（产线本就要整盘擦写，无需先挂载）。无命令则结束、继续正常挂载。
    // 此阶段不安装 USJ 驱动，用默认 secondary console 读 stdin（与下方产线一致）。
    int force_mode = 0; // 0:不强制 1:烧录 2:提取
#if BURN_FORCE_WINDOW_MS > 0
    {
        char fcmd[32];
        int fwait = 0;
        while (fwait < BURN_FORCE_WINDOW_MS)
        {
            if (fwait % 500 == 0)
            {
                printf("\nESP32_READY_CMD_WAIT\n"); // 3.py 握手锚点，勿删
                printf("NEED_BURN\n");
                fflush(stdout);
            }
            memset(fcmd, 0, sizeof(fcmd));
            int len = 0;
            while (len < (int)(sizeof(fcmd) - 1))
            {
                int c = fgetc(stdin);
                if (c == EOF)
                    break;
                fcmd[len++] = (char)c;
            }
            if (len > 0)
            {
                if (strstr(fcmd, "START"))
                {
                    force_mode = 1;
                    break;
                }
                else if (strstr(fcmd, "DUMP"))
                {
                    force_mode = 2;
                    break;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            fwait += 10;
        }
        if (force_mode)
            ESP_LOGW(TAG, "收到 PC 命令，强制进入产线模式 (mode=%d)", force_mode);
    }
#endif

    const esp_vfs_fat_mount_config_t mount_config = {
        // ⚠️ 必须为 false：true 会在任何挂载错误时静默格式化整盘，
        //    用户上传的 GIF/audio 资源会无声丢失。失败时改走下面的
        //    产线监听分支，等待 burner 发 START 烧录。
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 4096};
    esp_err_t mount_ret = ESP_FAIL;
    if (force_mode == 0) // 未被强制烧录时才尝试正常挂载
    {
        mount_ret = esp_vfs_fat_spiflash_mount_rw_wl("/S", "ext_storage", &mount_config, &s_wl_handle);
        if (mount_ret == ESP_OK)
        {
            // 成品板：挂载成功，直接返回继续正常启动，不进产线
            ESP_LOGI(TAG, "外部 Flash 正常挂载到 /S");
            return;
        }
    }

    // --- 步骤 3: 强制烧录 或 挂载失败 → 进入产线监听 ---
    // 此阶段不安装 USJ 驱动，用默认 secondary console 读写，
    // 避免 USJ ISR 干扰后续 I2C 总线时序（BUG: ES8311 NACK）。
    // 不 abort、不重启：保持监听，让 burner.py 在任意时刻插入都能握手接管。
    if (force_mode == 0)
        ESP_LOGW(TAG, "挂载失败 (0x%x)，无有效镜像，进入产线模式等待烧录...", mount_ret);
    ESP_LOGI(TAG, "📢 产线模式开启！发送 'START' 烧录，发送 'DUMP' 提取...");

    char cmd[32] = {0};
    uint32_t wait_ms = 0;
    int mode = force_mode; // 0:继续监听 1:烧录 2:提取（强制时直接带入，跳过监听循环）

    // 旧逻辑这里是 10 秒超时窗口；现在挂载已确认失败、没有可启动的文件系统，
    // 故改为持续监听（死循环里轮询命令字），等 burner 随时接管。
    while (mode == 0)
    {
        // 每 2s 喊话一次，够 Python 同步且不刷屏
        if (wait_ms % 2000 == 0)
        {
            printf("\nESP32_READY_CMD_WAIT\n"); // Python 握手锚点，勿删
            printf("NEED_BURN\n");              // 兼容旧约定：告知 burner.py 可发数据
            fflush(stdout);
        }

        // 不依赖 USJ 驱动，直接读 HW RX FIFO（最多 64 字节，足够检测命令字）
        memset(cmd, 0, sizeof(cmd));
        int len = 0;
        while (len < (int)(sizeof(cmd) - 1))
        {
            int c = fgetc(stdin);
            if (c == EOF)
                break;
            cmd[len++] = (char)c;
        }

        if (len > 0)
        {
            if (strstr(cmd, "START"))
            {
                mode = 1;
                break;
            }
            else if (strstr(cmd, "DUMP"))
            {
                mode = 2;
                break;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
        wait_ms += 10;
    }

    // 进入产线模式时才安装 USJ 驱动（大吞吐量需要环形缓冲）
    usb_serial_jtag_driver_config_t usj_cfg = {
        .rx_buffer_size = USJ_RX_BUF_SIZE,
        .tx_buffer_size = USJ_TX_BUF_SIZE,
    };
    ESP_ERROR_CHECK(usb_serial_jtag_driver_install(&usj_cfg));
    usb_serial_jtag_vfs_use_driver();

    if (mode == 1)
    {
        printf("\n>>> 进入烧录模式 (PC -> Flash) <<<\n");
        start_production_burning(); // 内部全盘擦除+烧录，含 esp_restart，不返回
    }
    else if (mode == 2)
    {
        printf("\n>>> 进入提取模式 (Flash -> PC) <<<\n");
        start_production_dump(); // 内部含 esp_restart，不返回
    }
}
