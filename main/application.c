/**
 * @file application.c
 * @brief Echo 应用主入口 — 启动序列编排
 *
 * 职责：按依赖顺序初始化各子系统，启动后将运行时控制权交给 session 模块。
 * 设计哲学：application 仅负责"启动"，所有运行时状态机和事件处理在各自模块。
 *
 * 启动序列（严格顺序，不可调换）：
 *   BSP → NVS → 音频 → 唤醒词 → WiFi → MQTT → Session
 *   → Servo → InteractionManager → TouchScanTask → TouchDispatchTask
 *
 * 触摸事件路由（application 层策略，不放在 BSP 层）：
 *   bsp_touch.c 只负责检测事件并入队 → touch_dispatch_task 读取并映射到情绪
 *   映射表集中在此文件，便于后续按产品需求调整（不需改 BSP 或情绪引擎）
 */
/**
 * @file application.h
 * @brief 应用层初始化入口头文件
 * 负责整合 BSP、协议、唤醒词等模块的统一启动
 */

#include "bsp/bsp_board.h"
#include "object.h" // PRINT_TASK_CREATED：任务创建后打印栈+堆
#include "protocol/mqtt_protocol.h"
#include "wake_word/custom_wake_word.h"
#include "application.h"
#include "session/session.h"
#include "audio/audio_processor.h"
#include "audio/offline_audio.h" // 离线音频播放（外挂 flash /S/voice/）

// ── 【调试】离线音频开机自测总开关（详见 app_main 内调用处注释）─────────────
// 2026-07-15 已实测通过：MP3/P3 双分支出声（三连音铃声）、零泄漏、时长吻合。
// 复测方法：置 1 重烧即可，测试音频已留存在外挂 flash /S/voice/test.{mp3,p3}
#define OFFLINE_AUDIO_BOOT_TEST 0

// ── 【排查 pp.c:4384 / ppProcTxSecFrame 看门狗】上行发包隔离开关 ──────────────
// 置 1：跳过所有会主动上行发包的模块（MQTT TLS + Auth HTTP + WebSocket），
//       只保留 WiFi 连接本身空转，用来判定崩溃是否由"上行发包→TX 硬件加密"触发：
//         · 置 1 后仍崩 → 与上行发包无关，是纯 RF/电源/beacon 接收侧问题
//         · 置 1 后不崩 → 确认一发数据就卡加密引擎，指向 DMA buffer 落 PSRAM 脏数据
//       排查完成后务必置回 0。
#define WIFI_TXHANG_ISOLATE 0
#include "bsp/servo_manager.h"
#include "bsp/bsp_ota.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui/ui_port.h"
#include "ui/interaction.h"
#include "ui/reminder.h"
#include "ui/standby.h"
#include "esp_lvgl_port.h"
#define TAG "Application"

/**
 * @brief 打印当前内部 SRAM 剩余空间，并自动算出"相对上一个打点消耗了多少"
 *
 * 用法：PRINT_INTERNAL_HEAP_STEP("步骤名")，紧跟在每个 init 调用之后。
 * 靠 static 变量记住上一次的剩余值，第一次调用没有基准，只打印当前值。
 * 谁消耗大，打印里会直接标红（Wxxx 级别日志）以便一眼看出。
 */
#define PRINT_INTERNAL_HEAP_STEP(step_name)                                            \
    do                                                                                 \
    {                                                                                  \
        static uint32_t s_last_internal_free = 0;                                      \
        static bool s_has_last = false;                                                \
        uint32_t now_free = (uint32_t)esp_get_free_internal_heap_size();               \
        if (!s_has_last)                                                               \
        {                                                                              \
            ESP_LOGI(TAG, "[heap] %-24s internal free: %lu B", (step_name), now_free); \
        }                                                                              \
        else                                                                           \
        {                                                                              \
            int32_t used = (int32_t)s_last_internal_free - (int32_t)now_free;          \
            if (used >= 2048)                                                          \
                ESP_LOGW(TAG, "[heap] %-24s internal free: %lu B（本步消耗 %ld B）★",  \
                         (step_name), now_free, (long)used);                           \
            else                                                                       \
                ESP_LOGI(TAG, "[heap] %-24s internal free: %lu B（本步消耗 %ld B）",   \
                         (step_name), now_free, (long)used);                           \
        }                                                                              \
        s_last_internal_free = now_free;                                               \
        s_has_last = true;                                                             \
    } while (0)

/** @brief 打印当前内部 SRAM 剩余空间（追踪初始化内存消耗） */
#define PRINT_INTERNAL_HEAP \
    ESP_LOGI(TAG, "[heap] internal free: %lu B", esp_get_free_internal_heap_size())

// ─── CPU 占用诊断 ───────────────────────────────────────────────────────────
// 依赖 sdkconfig：
//   CONFIG_FREERTOS_USE_TRACE_FACILITY=y
//   CONFIG_FREERTOS_USE_STATS_FORMATTING_FUNCTIONS=y
//   CONFIG_FREERTOS_GENERATE_RUN_TIME_STATS=y
// 调试完毕可注释掉 application_init() 末尾的任务创建语句。
#define CPU_STATS_PERIOD_MS 10000 // 打印周期：10 秒
#define CPU_STATS_BUF_SIZE 2048   // ~18 任务 × 70 字节/行 富余

/**
 * @brief 周期性打印每个任务的运行时间百分比
 *
 * 输出格式（vTaskGetRunTimeStats）：
 *   任务名         绝对运行时间    占总时间 %
 *   IDLE0          xxxxx           45%      ← CPU0 空闲率，100%-此值 = CPU0 负载
 *   IDLE1          xxxxx           30%      ← CPU1 空闲率，100%-此值 = CPU1 负载
 *   encoder_task   xxxxx           12%
 *   ...
 *
 * 注意：%CPU 是"占总 runtime 计数"的百分比；双核累计可超过 100%。
 *       看单核负载：IDLE0/IDLE1 反向推算更直观。
 */
static void cpu_stats_task(void *arg)
{
    char *buf = (char *)heap_caps_malloc(CPU_STATS_BUF_SIZE,
                                         MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buf == NULL)
    {
        ESP_LOGE("CPU_STATS", "诊断缓冲分配失败，任务退出");
        vTaskDelete(NULL);
        return;
    }

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(CPU_STATS_PERIOD_MS));
        memset(buf, 0, CPU_STATS_BUF_SIZE);
        vTaskGetRunTimeStats(buf);
        // 用 printf 直输，避免 ESP_LOGI 多行截断
        printf("\n=== CPU Runtime Stats ===\n%s===========================\n", buf);
    }
}

#include <dirent.h>  // 必须包含这个，才能使用 DIR 和 readdir
#include <string.h>  // memset 用于 cpu_stats_task；strcmp 用于任务清单内部/外部栈判定
#include <stdbool.h> // 任务清单 is_internal 标志
#include <stdio.h>   // printf 用于 CPU 占用诊断输出
#include "esp_log.h" // 确保能使用 ESP_LOGI 等日志宏
// 一个函数扫描所有资源
void scan_production_assets(const char *root_path)
{
    ESP_LOGI("FS", "========================================");
    ESP_LOGI("FS", "🔍 开始资源完整性校验: %s", root_path);

    const char *sub_folders[] = {"/gif", "/audio"}; // 你关心的子目录

    for (int i = 0; i < 2; i++)
    {
        char full_path[128];
        snprintf(full_path, sizeof(full_path), "%s%s", root_path, sub_folders[i]);

        DIR *dir = opendir(full_path);
        if (dir == NULL)
        {
            ESP_LOGW("FS", "⚠️ 未发现目录: %s (请检查 storage.bin 打包结构)", full_path);
            continue;
        }

        struct dirent *de;
        while ((de = readdir(dir)) != NULL)
        {
            // 忽略系统隐藏文件
            if (de->d_name[0] == '.')
                continue;
            ESP_LOGI("FS", "[%s] 🚀 发现资源: %s", sub_folders[i], de->d_name);
        }
        closedir(dir);
    }
    ESP_LOGI("FS", "========================================");
}
void debug_root_files(void)
{
    DIR *dir = opendir("/S");
    if (!dir)
    {
        ESP_LOGE("DEBUG", "连 /S 都打不开！");
        return;
    }
    struct dirent *de;
    int count = 0;
    printf("--- 物理搜索开始 ---\n");
    while ((de = readdir(dir)) != NULL)
    {
        printf("[%d] 找到条目: %s (类型: %d)\n", count++, de->d_name, de->d_type);
    }
    if (count == 0)
    {
        printf("🚨 警告：外挂 Flash 根目录下空无一物！\n");
    }
    closedir(dir);
}
// ═══════════════════════════════════════════════════════════════════════════════
// 2. 唤醒提示音
// ═══════════════════════════════════════════════════════════════════════════════

// ─── 唤醒提示音 ───────────────────────────────────────────────────────────
// 880Hz 方波，持续 300ms，通过 ES8311 DAC 输出到扬声器
// 方波生成无需浮点运算，在 ESP32-S3 上 CPU 占用极低
static void play_wake_tone(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    // Codec 设备必须已初始化（audio_init 完成后才调用此函数，正常不会为 NULL）
    if (!board || !board->codec_dev)
        return;

    // 音调参数
    const int sample_rate = 16000;  // 16kHz采样率
    const int freq_hz = 880;        // 880Hz = 音乐 A5，清脆易辨
    const int duration_ms = 300;    // 持续 300ms，简短提示
    const int16_t amplitude = 8000; // 幅度（0~32767，8000 约为 24% 满幅，适中音量）

    // 计算方波半周期采样点数：half_period = 采样率 / 频率 / 2
    // 880Hz → 半周期 = 16000 / 880 / 2 ≈ 9 个采样点
    const int half_period = sample_rate / freq_hz / 2;
    const int total_samples = sample_rate * duration_ms / 1000; // = 4800 个采样点

// 使用栈上小缓冲区分块写入，避免 heap 分配大块内存
#define TONE_CHUNK 256
    int16_t buf[TONE_CHUNK];
    int written = 0; // 已生成的采样点计数
    int phase = 0;   // 方波相位计数（0~2×half_period 循环）

    while (written < total_samples)
    {
        // 本次写入的采样点数（最后一块可能不满 TONE_CHUNK）
        int n = total_samples - written;
        if (n > TONE_CHUNK)
            n = TONE_CHUNK;

        // 生成方波：前半周期为正幅度，后半周期为负幅度
        for (int i = 0; i < n; i++)
        {
            buf[i] = (phase < half_period) ? amplitude : -amplitude;
            // 相位推进并循环归零
            if (++phase >= half_period * 2)
                phase = 0;
        }

        // 将 PCM 数据写入 Codec TX 通道（阻塞直到 DMA 接收完本块数据）
        esp_codec_dev_write(board->codec_dev, buf, n * sizeof(int16_t));
        written += n;
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 2.5 【调试】离线音频开机自测（OFFLINE_AUDIO_BOOT_TEST=1 时编译）
//
// 闭环验证方案：测试音频（2 秒三连音铃声）以 EMBED_FILES 内嵌在固件中，
// 开机后由本任务落盘到外挂 flash /S/voice/（免产线全盘重烧），
// 随后依次播放 test.mp3（MP3 分支）和 test.p3（P3/OPUS 分支），
// 并做 heap 双口径对账（内部 SRAM + PSRAM）验证零泄漏。
// ═══════════════════════════════════════════════════════════════════════════════
#if OFFLINE_AUDIO_BOOT_TEST
#include <sys/stat.h> // mkdir / stat

#define TAG_OFFLINE_TEST "OFFLINE_TEST"

// 固件内嵌测试音频（main/audio/test_assets/，由 CMakeLists EMBED_FILES 注入）
extern const uint8_t test_mp3_start[] asm("_binary_test_mp3_start");
extern const uint8_t test_mp3_end[] asm("_binary_test_mp3_end");
extern const uint8_t test_p3_start[] asm("_binary_test_p3_start");
extern const uint8_t test_p3_end[] asm("_binary_test_p3_end");

/** 把一段内嵌数据落盘到外挂 flash（已存在且大小一致则跳过，省 flash 写寿命） */
static void boot_test_seed_file(const char *path, const uint8_t *start, const uint8_t *end)
{
    size_t want = (size_t)(end - start);
    struct stat st;
    if (stat(path, &st) == 0 && (size_t)st.st_size == want)
    {
        ESP_LOGI(TAG_OFFLINE_TEST, "已存在 %s (%u B)，跳过落盘", path, (unsigned)want);
        return;
    }
    FILE *f = fopen(path, "wb");
    if (!f)
    {
        ESP_LOGE(TAG_OFFLINE_TEST, "创建 %s 失败（/S 未挂载？）", path);
        return;
    }
    size_t written = fwrite(start, 1, want, f);
    fclose(f);
    ESP_LOGI(TAG_OFFLINE_TEST, "落盘 %s: %u/%u B", path, (unsigned)written, (unsigned)want);
}

/** 等待当前离线播放结束（带超时，防解码分支卡死拖住自测流程） */
static void boot_test_wait_done(int timeout_ms)
{
    while (offline_audio_is_playing() && timeout_ms > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
        timeout_ms -= 100;
    }
}

/** 自测任务：落盘 → 连播 MP3/P3 → heap 对账 → 自删 */
static void offline_audio_boot_test_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG_OFFLINE_TEST);
    vTaskDelay(pdMS_TO_TICKS(3000)); // 等系统初始化收敛，避免日志混杂/开机爆音重叠

    mkdir("/S/voice", 0777); // 已存在会返回 EEXIST，无害
    boot_test_seed_file("/S/voice/test.p3", test_p3_start, test_p3_end);

    // ── 只测 P3/OPUS 分支：播 3 遍，每遍间隔 1.5s，便于耳朵反复确认 ──────
    // 目的：单独确认 OPUS→P3 解码链路能出声（不掺 MP3，声音来源无歧义）。
    for (int i = 1; i <= 3; i++)
    {
        ESP_LOGW(TAG_OFFLINE_TEST, "════ P3/OPUS 播放 第 %d/3 遍 ════", i);
        offline_audio_play("/S/voice/test.p3");
        boot_test_wait_done(10000);
        vTaskDelay(pdMS_TO_TICKS(1500)); // 遍间静默，声音分得清
    }
    ESP_LOGW(TAG_OFFLINE_TEST, "════ P3 自测完成（共 3 遍，全部走 OPUS 解码）════");
    vTaskDelete(NULL); // 普通内部 SRAM 栈任务，普通自删即可
}

/** app_main 调用入口：拉起自测任务后立即返回，不阻塞初始化流程 */
static void offline_audio_boot_test_start(void)
{
    // 内部 SRAM 6KB 栈：任务里有 FAT 写外挂 flash 操作，调试任务稳妥优先
    xTaskCreatePinnedToCore(offline_audio_boot_test_task, "offline_test",
                            6144, NULL, 3, NULL, 0);
    PRINT_TASK_CREATED(TAG_OFFLINE_TEST, "offline_test", 6144, 1);
}
#endif // OFFLINE_AUDIO_BOOT_TEST

// 舵机循环测试任务（独立运行，不阻塞 LVGL）
// static void servo_test_task(void *arg)
// {
//     while (1)
//     {
//         bsp_servo_move_all_parallel(60.0f, 60.0f, 120.0f, SERVO_SPEED_MID); // 同步运动示例
//         vTaskDelay(pdMS_TO_TICKS(500));
//         bsp_servo_move_all_parallel(120.0f, 120.0f, 60.0f, SERVO_SPEED_MID); // 同步运动示例
//         vTaskDelay(pdMS_TO_TICKS(500));
//         bsp_servo_move_all_parallel(90.0f, 90.0f, 90.0f, SERVO_SPEED_MID); // 同步运动示例
//         vTaskDelay(pdMS_TO_TICKS(1000));
//     }
// }

// ═══════════════════════════════════════════════════════════════════════════════
// 3. 唤醒词回调
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 唤醒词引擎识别命中后的回调
 *
 * 仅做转发，真正的会话开启逻辑在 session_on_wake_word 内部。
 */
static void wake_word_callback(const char *wake_word_display)
{
    standby_notify_activity(); // 唤醒命中视为活动，刷新待机倒计时（无论后续是否成会话都算用户活动）

    // 先交给 session 处理，由其返回值判定本次唤醒的真实语义。
    // 注意：提示音必须在 session 判定之后才播——否则打断 TTS 时也会先播提示音，
    //       造成"唤醒提示音 + 打断"两套逻辑同时触发（见 wake_result_t 说明）。
    wake_result_t result = session_on_wake_word(wake_word_display);

    switch (result)
    {
    case WAKE_NEW_SESSION:
        // 真·开启新会话：给用户听觉反馈
        ESP_LOGW("WAKE_UP", "唤醒词触发（开启新会话）: [%s]", wake_word_display);
        // play_wake_tone(); // 播放 880Hz 提示音
        break;
    case WAKE_INTERRUPT:
        // 打断 TTS：不播提示音，避免打断用户插话的连贯性
        ESP_LOGW("WAKE_UP", "唤醒词触发（打断 TTS，不播提示音）: [%s]", wake_word_display);
        break;
    case WAKE_IGNORED:
    default:
        // 被忽略（LISTENING 中 / drain 收尾 / 启动失败）：不打扰用户
        ESP_LOGW("WAKE_UP", "唤醒词触发（已忽略）: [%s]", wake_word_display);
        break;
    }
}

//
// ═══════════════════════════════════════════════════════════════════════════════
// 5. 应用主初始化序列
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 应用程序主初始化入口（由 main.c 的 app_main() 调用）
 *
 * 函数返回后系统进入事件驱动模式，所有逻辑由各任务/回调推进。
 *
 * 初始化顺序约束：
 *   1. bsp_board_get_instance  — 必须最先（创建 EventGroup）
 *   2. bsp_board_nvs_init      — 早于所有使用 NVS 的模块
 *   3. audio_init              — 早于唤醒词引擎（feed task 立即投喂）
 *   4. wake_word_init/start    — 引擎就绪后再启动监听
 *   5. bsp_board_wifi_main     — 阻塞等待联网
 *   6. protocol_mqtt_start     — 必须在联网后
 *   7. session_init            — WebSocket 预连接
 *   8. bsp_board_servo_init    — 舵机 LEDC 硬件初始化
 *   9. servo_manager_init      — 队列化舵机管理器（依赖步骤 8）
 *  10. interaction_manager_init — 情绪矩阵 worker（依赖步骤 9）
 *  11. xTaskCreate touch_scan   — 触摸硬件扫描（依赖步骤 10 已初始化）
 *  12. xTaskCreate touch_dispatch — 触摸→情绪路由（依赖步骤 10 + 11）
 */
void application_init(void)
{

    /* ⚠️⚠️⚠️ 关键启动顺序：此段必须保持在 application_init 最开头，
     *    早于 bsp_flash_init / audio_init / bsp_lcd_init 等所有 I2C/SPI 外设初始化。
     *
     * 背景：GPIO 14 是 ESP32-S3 的 FSPIWP/SUBSPIWP 复用脚（官方手册 I1 字段表示
     *       上电默认配置为「输入 + 弱上拉到 1」，约 45kΩ → 3.3V）。
     *       本项目 PCB 焊死把左臂舵机信号线接到 GPIO 14（无法改换引脚）。
     *
     * 现象：若不压低 → 上电瞬间舵机信号被弱上拉拉高 → 舵机识别为短脉冲反复抖动
     *       → 拉走大电流 → 共用 3.3V 电源轨压降 → ES8311/触摸 IC 报 NACK，
     *       同时 LEDC TIMER_0 共用的 GPIO 4/9 舵机连锁失灵。
     *
     * 修复：用 gpio_config 显式禁用上下拉/中断并主动输出 0V，舵机识别为「无脉冲」
     *       保持静止。后续 bsp_board_servo_init 启用 LEDC 时会通过 GPIO Matrix
     *       重新路由 PWM 信号，覆盖此处输出状态，无冲突。
     *
     * 详见 memory/bugs/BUG-015.md */
    gpio_config_t io_conf_g14 = {
        .pin_bit_mask = (1ULL << GPIO_NUM_14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE, // 显式关上拉，覆盖 FSPIWP 默认 I1=1
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE, // 显式关中断，避免 SUBSPIWP 残留中断
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_g14));
    gpio_set_level(GPIO_NUM_14, 0); // 主动输出 0V，停止舵机误抖动
    bsp_flash_init();
    PRINT_INTERNAL_HEAP_STEP("bsp_flash_init");
    debug_root_files();
    scan_production_assets("/S"); // 扫描 /S 目录下的所有资源
    if (access("/S/assets/gif/one.gif", F_OK) == 0)
        printf("路径 A 物理存在！\n");
    if (access("/S/gif/one.gif", F_OK) == 0)
        printf("路径 B 物理存在！\n");
    /* ── 步骤 1: BSP 单例 ──────────────────────────────────────────────────── */
    bsp_board_t *bsp_board = bsp_board_get_instance();

    /* ── 步骤 2: NVS Flash ─────────────────────────────────────────────────── */
    bsp_board_nvs_init(bsp_board);
    PRINT_INTERNAL_HEAP_STEP("bsp_board_nvs_init");

    /* ── 步骤 5: WiFi / BluFi 配网（阻塞直至获取 IP 或彻底失败后重启）───────
     * ★LCD/UI 初始化必须放在此步【之后】：BLE controller 使能窗口内若 LVGL 正在
     *   并发解码 GIF / SPI DMA 刷屏 / 投递舵机，会与 BT 抢内部资源，导致 BLE 初始化
     *   随机崩溃（LoadProhibited 野 handle 或 ble_svc_gap_init 断言，见 BUG-026）。
     *   配网不再显示二维码（已删除该逻辑），因此 UI 无需早于 WiFi，放回配网之后即可。*/
    bsp_board_wifi_main(bsp_board);
    PRINT_INTERNAL_HEAP_STEP("bsp_board_wifi_main");
    /* ── 步骤 2.5: LCD + UI 初始化（WiFi/配网完成后再起，避开 BLE 初始化窗口）── */
    bsp_board_lcd_init(bsp_board);
    PRINT_INTERNAL_HEAP_STEP("bsp_board_lcd_init");
    ui_init();
    vTaskDelay(pdMS_TO_TICKS(100));
    PRINT_INTERNAL_HEAP_STEP("ui_init");
    if (lvgl_port_lock(1000))
    {
        bsp_board_lcd_on(bsp_board);
        lvgl_port_unlock();
    }

    /* ── 步骤 3: 音频硬件 + 采集任务（裸板无 ES8311，注释）──────────────── */
    audio_init(bsp_board);
    PRINT_INTERNAL_HEAP_STEP("audio_init");

// ── 【调试】离线音频开机自测开关 ─────────────────────────────────────────────
// 置 1：开机后把固件内嵌的测试音频落盘到外挂 flash /S/voice/，然后依次自动
//       播放 test.mp3 和 test.p3，验证双分支解码链路 + heap 泄漏对账。
//       验证完成后置回 0（内嵌音频与 CMakeLists 的 EMBED_FILES 可一并移除）。
// 前置条件：bsp_flash_init（/S 已挂载）+ audio_init（codec 就绪）均已完成。
#if OFFLINE_AUDIO_BOOT_TEST
    offline_audio_boot_test_start();
#endif

    /* ── 步骤 4: 唤醒词引擎（裸板无麦克风，注释）──────────────────────────── */
    wake_word_init(wake_word_callback);
    wake_word_start();
    PRINT_INTERNAL_HEAP_STEP("wake_word_init+start");

#if WIFI_TXHANG_ISOLATE
    /* ── 【排查】上行发包隔离：跳过 MQTT + session，仅 WiFi 空转 ───────────── */
    ESP_LOGW(TAG, "[排查] WIFI_TXHANG_ISOLATE=1：跳过 MQTT/Auth/WebSocket，仅 WiFi 空转");
#else
    /* ── 步骤 6: MQTT 客户端 ───────────────────────────────────────────────── */
    protocol_mqtt_start();
    PRINT_INTERNAL_HEAP_STEP("protocol_mqtt_start");

    /* ── 步骤 7: 会话模块（WebSocket 预连接）─────────────────────────────── */
    // session_init("ws://122.224.191.2:4888/ws/omni");
    session_init("wss://ai.strailine-space.com/ws/omni");

    PRINT_INTERNAL_HEAP_STEP("session_init");
#endif // WIFI_TXHANG_ISOLATE

    // /* ── 步骤 8: 舵机硬件初始化（LEDC/PWM）──────────────────────────────── */
    bsp_board_servo_init(bsp_board);
    PRINT_INTERNAL_HEAP_STEP("bsp_board_servo_init");

    // /* ── 步骤 9: 舵机管理器（队列 + worker task，栈在 SPIRAM）─────────────── */
    esp_err_t ret = servo_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "servo_manager_init 失败: %s", esp_err_to_name(ret));
    }
    PRINT_INTERNAL_HEAP_STEP("servo_manager_init");

    /* ── 步骤 10: 情绪交互管理器（情绪矩阵 + worker task，栈在 SPIRAM）────── */
    ret = interaction_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "interaction_manager_init 失败: %s", esp_err_to_name(ret));
    }
    PRINT_INTERNAL_HEAP_STEP("interaction_manager_init");
    // // 舵机测试任务（独立跑，不影响 LVGL 刷新）
    // xTaskCreatePinnedToCoreWithCaps(
    //     servo_test_task,
    //     "servo_test",
    //     4096,
    //     NULL,
    //     5,
    //     NULL,
    //     tskNO_AFFINITY,
    //     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    // 6. 创建触摸扫描任务
    // ⚠ 栈必须在内部 SRAM，不能放 SPIRAM！
    //    本任务承载整个 UI 跳转链（含游戏初始化），其中 game_whack 读写 NVS 高分会
    //    触发 spi_flash_disable_interrupts_caches_and_other_cpu()，期间 cache 被禁用，
    //    SPIRAM 栈不可访问 → esp_task_stack_is_sane_cache_disabled() 断言 panic（BUG-010 家族）。
    //    实测进游戏路径栈高水位剩 5888B，即峰值用量仅 2304B，故 4096 足够（留 ~1.7× 余量）。
    //    内部 SRAM 净增 4KB，换来彻底消除「触摸任务里碰 flash 必崩」隐患。
    ret = xTaskCreatePinnedToCoreWithCaps(
        touch_scan_task,
        "touch_scan",
        4096,
        NULL,
        4, // 优先级略低于舵机和音频
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "创建触摸扫描任务失败！");
    }
    else
    {
        ESP_LOGI(TAG, "触摸扫描任务创建完成");
        PRINT_TASK_CREATED(TAG, "touch_scan", 4096, 1); // 栈在内部SRAM
    }

    /* ═══ 【震动 PWM 方波测试】临时调试任务 ═══════════════════════════════════
     * 循环 0→25→50→75→100% 占空比，每档 3 秒，串口打印档位。
     * 用示波器测 GPIO3（BSP_MOTOR_VIB_PIN），时基 ~50µs/格 可见 5kHz 方波。
     * 测试完成后，删除/注释本段即可恢复正常逻辑。 */
    // xTaskCreate(motor_pwm_test_task, "motor_pwm_test", 4096, NULL, 3, NULL);

    /* ── 步骤 10.5: 无活动待机模块（依赖 LCD/唤醒词/舵机管理器均已就绪）──── */
    standby_init();
    PRINT_INTERNAL_HEAP_STEP("standby_init");

    /* ── 步骤 11: 电池 + 提醒系统（无感/次要功能，后置以让核心链路尽早就绪）──
     * 挪动理由：电池监控纯 ADC 无依赖；reminder_init 内部会同步做一次 IP 定位
     * （阻塞 HTTPS，实测 ~850ms）+ 天气拉取。这两块对用户「看到 GIF、能对话、能
     * 触摸互动」毫无感知贡献，故整体后移到所有可见模块之后，不再拖慢舵机/触摸/
     * 待机的就绪。二者都依赖 WiFi 已连（此处 WiFi 早已就绪，无依赖风险）。 */
    esp_err_t bat_ret = bsp_battery_init();
    if (bat_ret == ESP_OK)
    {
        bsp_battery_start_task(NULL); // 暂不接低电回调，UI 自身已带变色提示
        bsp_battery_start_log_task(); // 新增：每 5s 打印一次电池电压/电量，便于调试
        xEventGroupSetBits(bsp_board->board_status, BATTERY_BIT);
        ESP_LOGI(TAG, "电池监控已启动");
    }
    else
    {
        ESP_LOGW(TAG, "电池监控未启用 (%s)，UI 电量将显示 --%%", esp_err_to_name(bat_ret));
    }
    PRINT_INTERNAL_HEAP_STEP("bsp_battery_init");
    reminder_init(NULL);
    PRINT_INTERNAL_HEAP_STEP("reminder_init");

    /* ── 诊断：全部初始化跑完后的内存全量快照 + 逐任务栈占用清单 ──────────────
     * 用于定位"初始化结束后内部SRAM仅剩极少"的问题：
     *   - PRINT_MEM_INFO 给出内部SRAM/PSRAM当前剩余 + 历史最低值（历史最低更能反映
     *     启动过程中的峰值消耗，比"当前剩余"更接近问题根因）
     *   - uxTaskGetSystemState 遍历当前所有任务，打印每个任务的栈总深度和历史最小
     *     剩余水位（栈总深度可反推"这个任务在内部/外部SRAM分别占了多少"，需要人工
     *     核对该任务创建时用的是 MALLOC_CAP_INTERNAL 还是 MALLOC_CAP_SPIRAM）
     */
    PRINT_MEM_INFO(TAG, "★全部初始化完成★");
    {
        UBaseType_t task_count = uxTaskGetNumberOfTasks();
        TaskStatus_t *task_list = heap_caps_malloc(task_count * sizeof(TaskStatus_t),
                                                   MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (task_list != NULL)
        {
            uint32_t total_runtime;
            UBaseType_t got = uxTaskGetSystemState(task_list, task_count, &total_runtime);
            ESP_LOGW(TAG, "[任务清单] 共 %u 个任务（栈深度单位：字，×4=字节）：", (unsigned)got);
            ESP_LOGW(TAG, "  说明：[INTERNAL]=栈占内部SRAM（缩它才省SRAM）；[PSRAM]=栈在外部PSRAM，缩了不省内部SRAM，勿被剩余大字数误导");
            for (UBaseType_t i = 0; i < got; i++)
            {
                /* FreeRTOS 的 TaskStatus_t 不记录栈所在内存类型，只能按任务名白名单判断。
                 * 白名单来源：逐个核对各 xTaskCreate* 创建时用的 MALLOC_CAP_INTERNAL / 普通
                 * xTaskCreate（栈默认内部SRAM）；未列入者一律视为 PSRAM 栈。
                 * ★ 新增/改动任务的栈内存类型时，务必同步更新此白名单，否则标签会误导。 */
                static const char *const k_internal_stack_tasks[] = {
                    // ── 业务任务：显式 MALLOC_CAP_INTERNAL 或普通 xTaskCreate（栈在内部SRAM）──
                    "audio_feed",   // bsp_codec.c   MALLOC_CAP_INTERNAL
                    "btn_task",     // bsp_wifi.c    MALLOC_CAP_INTERNAL
                    "offline_test", // application.c 普通 xTaskCreate（自测任务，可关宏省 6KB）
                    "bat_mon",      // bsp_battery.c 普通 xTaskCreate
                    "nvs_save",     // reminder.c    MALLOC_CAP_INTERNAL
                    // ── IDF 系统任务：栈默认分配在内部SRAM ──
                    "main",
                    "IDLE0",
                    "IDLE1",
                    "ipc0",
                    "ipc1",
                    "wifi",
                    "tiT",
                    "esp_timer",
                    "Tmr Svc",
                    "sys_evt",
                };
                bool is_internal = false;
                for (size_t k = 0; k < sizeof(k_internal_stack_tasks) / sizeof(k_internal_stack_tasks[0]); k++)
                {
                    if (strcmp(task_list[i].pcTaskName, k_internal_stack_tasks[k]) == 0)
                    {
                        is_internal = true;
                        break;
                    }
                }
                ESP_LOGW(TAG, "  %-18s #%2u %-16s 优先级%2u 栈历史最小剩余 %5u 字(≈%5u B)",
                         is_internal ? "[INTERNAL]" : "[PSRAM ·不占SRAM]",
                         (unsigned)i, task_list[i].pcTaskName,
                         (unsigned)task_list[i].uxCurrentPriority,
                         (unsigned)task_list[i].usStackHighWaterMark,
                         (unsigned)(task_list[i].usStackHighWaterMark * sizeof(StackType_t)));
            }
            free(task_list);
        }
        else
        {
            ESP_LOGE(TAG, "[任务清单] SPIRAM分配失败，无法打印任务清单（任务数=%u）", (unsigned)task_count);
        }
    }

    /* ── 步骤 8: CPU 占用诊断任务（调试用，可注释掉）─────────────────────── */
    // 低优先级、tskNO_AFFINITY、栈在 SPIRAM，对业务无干扰
    // xTaskCreatePinnedToCoreWithCaps(
    //     cpu_stats_task,
    //     "cpu_stats",
    //     4096,
    //     NULL,
    //     1,                  // 最低优先级（仅次于 IDLE）
    //     NULL,
    //     tskNO_AFFINITY,
    //     MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    // ESP_LOGI(TAG, "CPU 占用诊断任务已启动，每 10 秒打印一次");

    /* ── 步骤 9: OTA 验证（必须在所有初始化完成后调用）──────────────────── */
    // 若当前是刚 OTA 升级完首次启动，会进入 PENDING_VERIFY 状态：
    //   - 调用 esp_ota_mark_app_valid_cancel_rollback() 防止 Bootloader 回滚
    //   - 把 NVS 中的 pending_ver 提升为 committed_ver
    // 若启动前期崩溃（未到这里），Bootloader 下次启动会自动回滚到旧固件
    ESP_LOGI(TAG, "当前固件版本: %s", bsp_ota_get_current_version());
    bsp_ota_mark_valid();

    ESP_LOGI(TAG, "后续版本使用变量");
}
