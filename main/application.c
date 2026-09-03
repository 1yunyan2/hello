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
// 2026-07-22 改为「响度对照测试」：真实MP3(cs.mp3) vs 占位音(ticklish.mp3) 交替播，
//            验证"声音小"是素材响度问题而非设备音量问题。素材全部读自外挂 flash。
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
#include "esp_cpu.h" // 【临时调试】esp_cpu_set_watchpoint 抓越界写
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "ui/ui_port.h"
#include "ui/interaction.h"
#include "ui/reminder.h"
#include "ui/standby.h"
#include "ui/remote_control.h" // remote_control_init/cancel()：app 远程手动控制（舵机/GIF + 30s 冻结）
#include "esp_lvgl_port.h"
#define TAG "Application"

/* 开机诊断段总开关（内存快照 + 区域①验证 + 逐任务栈清单）。
 *
 * 【为何默认关闭】这段诊断跑在 main（prio 1、CPU0）上，而 CPU0 同时住着
 *   taskLVGL（prio 5，见 ui_port.c 的 lvgl_port_cfg.task_affinity=0）。
 *   GIF 轮播解码期间 main 几乎抢不到 CPU，实测 26 行任务清单要打 13 秒，
 *   把开机时间从 ~14s 拖到 ~30s；而且 main 全程不让权还会饿死 IDLE0，
 *   触发 "task_wdt: IDLE0 (CPU 0)" 报警（详见 BUG-027 同类问题）。
 *   循环里的 vTaskDelay 只解决看门狗，解决不了耗时。
 *
 * 【何时打开】排查内部SRAM占用/碎片/各任务栈水位时改成 1，查完改回 0。
 *   打开后开机会明显变慢属预期，不是 bug。 */
#define APP_BOOT_DIAG_ENABLE 0

/**
 * @brief 打印当前内部 SRAM 剩余空间，并自动算出"相对上一个打点消耗了多少"
 *
 * 用法：PRINT_INTERNAL_HEAP_STEP("步骤名")，紧跟在每个 init 调用之后。
 * 靠 static 变量记住上一次的剩余值，第一次调用没有基准，只打印当前值。
 * 谁消耗大，打印里会直接标红（Wxxx 级别日志）以便一眼看出。
 *
 * ★ 本宏与 HEAP_CHECK_STEP 跟随 object.h 的 MEM_DEBUG_LOG_ENABLE 总开关，
 *   关闭时编译成空语句（HEAP_CHECK_STEP 内的 heap_caps_check_integrity_all
 *   要遍历全部堆块，是启动路径上单条最贵的调试调用，必须一起关）。
 */
#if MEM_DEBUG_LOG_ENABLE
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

/** @brief 【临时调试】查堆完整性，坏了就大声报是哪一步之后坏的 —— 定位越界元凶 */
#define HEAP_CHECK_STEP(step_name)                                               \
    do                                                                           \
    {                                                                            \
        if (!heap_caps_check_integrity_all(false))                               \
            ESP_LOGE("HEAPSTEP", "☠☠☠ 堆在【%s】之后已损坏！凶手就在此步或之前", \
                     (step_name));                                               \
        else                                                                     \
            ESP_LOGW("HEAPSTEP", "✓ [%s] 之后堆完好", (step_name));              \
    } while (0)

#else /* MEM_DEBUG_LOG_ENABLE == 0 */

#define PRINT_INTERNAL_HEAP_STEP(step_name) \
    do                                      \
    {                                       \
    } while (0)
#define HEAP_CHECK_STEP(step_name) \
    do                             \
    {                              \
    } while (0)

#endif /* MEM_DEBUG_LOG_ENABLE */

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

    const char *sub_folders[] = {"/gif", "/audio", "/voice"}; // 你关心的子目录

    for (int i = 0; i < 3; i++)
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
// 网络下载的真实 MP3（已转 16k 单声道 / 峰值 0dB），4.5KB
extern const uint8_t cs_mp3_start[] asm("_binary_cs_mp3_start");
extern const uint8_t cs_mp3_end[] asm("_binary_cs_mp3_end");

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

    // ── 真实 MP3 播放测试（素材内嵌固件，无需跑 2.py/3.py 烧外挂 flash）──────
    // cs.mp3：网络下载的真实 MP3，已 ffmpeg 转 16k 单声道（峰值 0dB 满响度）。
    // 由 CMakeLists EMBED_FILES 内嵌进固件 → 开机落盘到 /S/voice/ → 播放。
    // （offline_audio 引擎按文件路径工作，故内嵌数据须先落盘再播。）
    mkdir("/S/voice", 0777); // 已存在返回 EEXIST，无害
    boot_test_seed_file("/S/voice/cs.mp3", cs_mp3_start, cs_mp3_end);

    // ── 只测 P3/OPUS 分支：播 3 遍，每遍间隔 1.5s，便于耳朵反复确认 ──────
    // 目的：单独确认 OPUS→P3 解码链路能出声（不掺 MP3，声音来源无歧义）。
    for (int i = 1; i <= 3; i++)
    {
        ESP_LOGW(TAG_OFFLINE_TEST, "════ P3/OPUS 播放 第 %d/3 遍 ════", i);
        offline_audio_play("/S/voice/cs.mp3"); // 内嵌素材已 ffmpeg 转 P3/OPUS，峰值 0dB 满响度
        boot_test_wait_done(10000);
        vTaskDelay(pdMS_TO_TICKS(1500)); // 遍间静默，声音分得清
    }

    vTaskDelete(NULL); // FreeRTOS任务函数禁止return，必须自删，否则栈上返回地址非法触发IllegalInstruction
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

/* ══ 舵机抖动 A/B 对照任务（★定位完连同 bsp_servo_debug_set_frame_ms 一并删除）══
 *
 * 【要回答的唯一问题】运动时的轻微抖动，到底是不是"每帧位移小于舵机死区"造成的？
 *
 * 【已知】厂家规格书：塑胶齿(臂)信号虚位 4~12μs、金属齿(头)≤6μs；本项目脉宽
 *   500~2400μs 覆盖 180°（10.56μs/度），换算过来电气死区约 0.4~1.1°。
 *   单向标定实测门槛 0.8°（=8.4μs），与规格吻合。
 *
 * 【2026-09-03 A/B 对照结论：死区模型已被推翻】
 *   曾用 A组每帧0.80°（压在门槛上）对 B组每帧0.20°（远低于门槛）做对照，
 *   实测【两组抖动无可分辨差异】，且全程空载。结合"负载已排除、PWM 分频精确"，
 *   ⇒ "每帧位移小于电气死区"【不是】运动抖动的原因，该方向就此关闭，勿再重走。
 *
 * 【真凶与修复】改查写入定时，在 bsp_servo.c 插值循环找到两个缺陷：
 *   ① vTaskDelay 是相对延时，被 LVGL/音频/唤醒词抢占后误差【逐帧累积】，
 *      帧间隔漂移 → 写入时刻相对 PWM 周期漂移 → 某些周期收到两次更新（该帧位移
 *      被覆盖丢失）、某些周期一次没有（位移重复）→ 脉宽序列不匀 → 抖动。
 *   ② 三轴分三次写入，中间可被抢占，三路 duty 落到不同 PWM 周期 → 三轴错拍。
 *   已分别改为 vTaskDelayUntil 绝对定时 + vTaskSuspendAll 包住三轴写入。
 *   ★这两条都与每帧位移大小无关，正好解释了 A/B 为何无差别。
 *
 * 【本任务现在的用途】单纯的连续慢速大行程运动，用来验证上述定时修复的效果。
 *   帧长不再覆盖（传 0 = 沿用 SERVO_FRAME_MS），保持与业务一致。
 */
#define SERVO_AB_STEP_MS SERVO_SPEED_VERY_SLOW ///< 慢速档(50ms/度)，抖动最明显的速度
#define SERVO_AB_ANGLE_LO 40.0f                ///< 行程下限（避开端点，防堵转干扰判断）
#define SERVO_AB_ANGLE_HI 140.0f               ///< 行程上限（100°行程，慢档约5秒走完，够看清）
#define SERVO_AB_CH CH_HEAD                    ///< 单轴测试用哪一路（三轴同动会互相干扰观察）

static void servo_test_task(void *arg)
{
    (void)arg;
    bsp_servo_debug_set_frame_ms(0); // 沿用 SERVO_FRAME_MS，不覆盖

    ESP_LOGW("SERVO_AB", "════ 三阶段抖动定位（不用信号发生器，只用本主控）════");
    ESP_LOGW("SERVO_AB", "★阶段1【静止】写一次角度后【完全不再写】，保持5秒。");
    ESP_LOGW("SERVO_AB", "★  LEDC硬件自己在输出恒定脉宽，全程零软件参与。");
    ESP_LOGW("SERVO_AB", "★  这5秒里抖 ⇒ 舵机在恒定信号下自己抖，与代码彻底无关。");
    ESP_LOGW("SERVO_AB", "★阶段2【粗步进】每500ms直接跳2°，一步到位不插值。");
    ESP_LOGW("SERVO_AB", "★  跳完那一下之后的静止期抖不抖？抖 ⇒ 是舵机的整定振荡。");
    ESP_LOGW("SERVO_AB", "★阶段3【正常插值】走业务同款慢速运动，作为对照基准。");

    while (1)
    {
        // ══ 阶段 1：绝对静止——写一次就撒手，验证"恒定信号下抖不抖" ══════════
        // ★这是本次测试的核心。等价于"用信号发生器送一路干净不变的PWM"：
        //   写完这一笔后，软件再也不碰 LEDC，硬件持续输出同一个脉宽。
        //   若此时仍抖，说明是舵机内部电位器反馈+模拟比较器的整定振荡，
        //   属于廉价模拟舵机的固有特性，任何软件手段都消不掉。
        // ⚠️ 先移到别处再回来：bsp_servo_move_smooth 有「差值<1° 直接返回」的死区过滤，
        //    上一轮阶段3 结束时已停在 ANGLE_LO，这里若再写 ANGLE_LO 会被过滤掉、
        //    根本不产生运动，第二轮起阶段1 就失去意义。故先去 90° 再回来。
        bsp_servo_move_smooth(SERVO_AB_CH, 90.0f, SERVO_SPEED_MID);
        vTaskDelay(pdMS_TO_TICKS(300));

        ESP_LOGW("SERVO_AB", "【阶段1·静止】已写 %.0f°，接下来5秒【零写入】——现在抖吗？",
                 SERVO_AB_ANGLE_LO);
        bsp_servo_move_smooth(SERVO_AB_CH, SERVO_AB_ANGLE_LO, SERVO_SPEED_MID);
        vTaskDelay(pdMS_TO_TICKS(5000)); // 全程不写 LEDC，纯硬件恒定输出

        // ══ 阶段 2：粗步进——每步远大于死区，且步间完全静止 ══════════════════
        // 若阶段1不抖、这里每跳一下之后的静止期也不抖，但连续插值时抖，
        // 才说明问题真在"连续写入"上；否则就是舵机自身特性。
        ESP_LOGW("SERVO_AB", "【阶段2·粗步进】每500ms跳2°，共10步——每跳之后静止时抖吗？");
        for (int i = 1; i <= 10; i++)
        {
            // 直接用 INSTANT（瞬间模式）：内部只写一次 duty，不做任何插值
            bsp_servo_move_smooth(SERVO_AB_CH, SERVO_AB_ANGLE_LO + (float)i * 2.0f, SERVO_SPEED_INSTANT);
            vTaskDelay(pdMS_TO_TICKS(500));
        }
        vTaskDelay(pdMS_TO_TICKS(1500));

        // ══ 阶段 3：正常插值运动——业务同款，作为对照 ════════════════════════
        ESP_LOGW("SERVO_AB", "【阶段3·插值】业务同款慢速运动——和前两阶段比，抖得更明显吗？");
        bsp_servo_move_smooth(SERVO_AB_CH, SERVO_AB_ANGLE_HI, SERVO_AB_STEP_MS);
        bsp_servo_move_smooth(SERVO_AB_CH, SERVO_AB_ANGLE_LO, SERVO_AB_STEP_MS);
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ═══════════════════════════════════════════════════════════════════════════════
// 3. 唤醒词回调
// ═══════════════════════════════════════════════════════════════════════════════

/* ── 唤醒词震动反馈参数 ──
 * 唤醒是高频动作，只需一下轻促的确认感，故比闹钟（ALARM_RING_VIBRATE_MS=200）短。
 * 实现走 bsp_motor_pulse_level()（内含 vTaskDelay 阻塞），调用点在唤醒引擎任务
 * 上下文（非 LVGL 线程），阻塞该时长不影响 UI 刷新与音频链路。
 *
 * 触发范围：只有 WAKE_NEW_SESSION（真开会话）与 WAKE_INTERRUPT（真打断 TTS）才震。
 * WAKE_IGNORED 不震——那 6 种情形（OTA 中 / 离线模式 / LISTENING 中 / 非 IDLE 态 /
 * drain 收尾 / 启动失败，见 session.c）系统并不会有任何响应，震了反而误导用户
 * 以为已经唤醒成功。 */
#define WAKE_WORD_VIBRATE_MS 100    ///< 唤醒词命中震动时长（毫秒）
#define WAKE_WORD_VIBRATE_LEVEL 100 ///< 唤醒词命中震动强度（0~100）

/* ── 开机震动反馈参数（2026-08-31 新增）──
 * 长按 3 秒开机后立刻震一下，让用户确认设备已通电启动，随后才开始各项初始化。
 * 调用点在 application_init() 最开头（GPIO14 拉低之后、bsp_flash_init 之前）。
 *
 * 时长取 200ms：开机是低频动作且要求"明确可感"，比唤醒（100ms，高频轻促）更实，
 * 与闹钟提醒（ALARM_RING_VIBRATE_MS=200）同级。嫌长/短直接改本宏即可，
 * 该值同时也是开机流程被推迟的时长（bsp_motor_pulse_level 内含等长 vTaskDelay）。 */
#define BOOT_VIBRATE_MS 200    ///< 开机震动时长（毫秒）
#define BOOT_VIBRATE_LEVEL 100 ///< 开机震动强度（0~100）

/**
 * @brief 唤醒词引擎识别命中后的回调
 *
 * 仅做转发，真正的会话开启逻辑在 session_on_wake_word 内部。
 */
static void wake_word_callback(const char *wake_word_display)
{
    standby_notify_activity(); // 唤醒命中视为活动，刷新待机倒计时（无论后续是否成会话都算用户活动）

    // 唤醒是「真实交互」，立刻打断 app 远程控制的 30s 冻结窗口。
    // center_servo=false（不归中）：唤醒后紧接着就是对话状态 GIF + 唤醒动作接管，
    // 舵机从当前角度直接平滑过渡即可，先归中反而多一次复位抽动。幂等，可无脑调用。
    remote_control_cancel(/*center_servo=*/false);

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
        bsp_motor_pulse_level(WAKE_WORD_VIBRATE_LEVEL, WAKE_WORD_VIBRATE_MS);
        break;
    case WAKE_INTERRUPT:
        // 打断 TTS：不播提示音，避免打断用户插话的连贯性
        ESP_LOGW("WAKE_UP", "唤醒词触发（打断 TTS，不播提示音）: [%s]", wake_word_display);
        bsp_motor_pulse_level(WAKE_WORD_VIBRATE_LEVEL, WAKE_WORD_VIBRATE_MS);
        break;
    case WAKE_IGNORED:
    default:
        // 被忽略（OTA 中 / 离线模式 / 功能层 / 提醒进行中 / LISTENING 中 / drain 收尾）：不打扰用户
        ESP_LOGW("WAKE_UP", "唤醒词触发（已忽略）: [%s]", wake_word_display);

        /* ★★【2026-08-25 修复：被忽略后引擎永久失聪】★★
         *
         * 【问题】custom_wake_word.c 的 detect 命中后会立即 `is_running = false`
         *   自停引擎（见该文件 wake_triggered 分支），把重启的责任交给 session：
         *   正常路径下 session_on_wake_word() 会开会话，流程里的 wake_word_start()
         *   负责把引擎拉回来。
         *   但 WAKE_IGNORED 的六条路径（OTA / 离线 / 功能层 / 提醒中 / LISTENING /
         *   drain）全部是 `return WAKE_IGNORED` 直接早退，【不经过任何 wake_word_start()】
         *   → 引擎停了没人重启 → 设备从此彻底失聪，直到下一次会话或退待机才偶然恢复。
         *
         * 【实测铁证】在功能盘喊唤醒词，命中一次 prob=0.720338 之后：
         *   detect=0/156、门控挡下/开闸/超时关闸/丢帧 五个计数器【全部冻结 30 秒以上】，
         *   而同期 GIF 正常切图、PCM 诊断正常打印 —— 系统活着，只有唤醒引擎死了。
         *   用户体感即「喊十几次只成功一次」：那一次是引擎死前的最后一次识别。
         *
         * 【为什么修在这里而不是 custom_wake_word.c】那边的自停是给正常会话流程用的
         *   （避免会话建立期间重复触发），逻辑正确，不该动。真正缺的是「被忽略时
         *   谁来负责恢复」——回调是唯一同时知道「命中了」和「被忽略了」的地方。
         *
         * 【与 session.c:1586 设计意图的呼应】那段注释明确写过：选逻辑守卫而非
         *   进功能层就停引擎，正是为了避免「永久失聪」。本修复让该意图真正成立。
         *
         * 【幂等】wake_word_start() 内部持锁后只做 clean + 置位 + 清队列，
         *   重复调用无害；引擎本就在运行时调用它也不会出问题。 */
        wake_word_start();
        break;
    }
}

/**
 * @brief 提醒系统触发回调 — 目前只接震动
 *
 * 闹钟开始响铃、以及响铃期间每次 tick（ALARM_RING_INTERVAL_MS 一次）都会回调到这里，
 * is_alarm_start=true 时震一下提示用户。倒计时/日历/天气不震动，只有闹钟。
 * reminder 模块调用本回调时不在 LVGL 线程上（在 reminder 自己的任务里），
 * 故可直接用 bsp_motor_pulse()（内含 30ms 阻塞），不会卡 UI。
 */
/* 本轮闹钟是否已切过页（2026-08-10 死锁修复配套，见 on_reminder_trigger 内注释）。
 * 置位：第一次响铃（alarm_ring_start，不持 s_ctx.mutex）时。
 * 复位：闹钟关闭回调（is_alarm_start=false）时，以及 reminder 已回到非响铃态时——
 *       后者是必需的兜底，因为响铃次数用尽自动关闭那条路径（reminder.c:970
 *       alarm_ring_stop）**不发任何回调**，只靠关闭回调复位会让下一次闹钟切不了页。 */
static bool s_alarm_ring_page_shown = false;

static void on_reminder_trigger(reminder_type_t type, const char *message, bool is_alarm_start)
{
    (void)message;

    /* 兜底复位：只要 reminder 不在响铃态，本轮标志一律清掉（幂等、无副作用）。
     * 覆盖「响铃次数用尽自动关闭」这条不发回调的路径。 */
    if (reminder_get_state() != REMINDER_STATE_RINGING)
        s_alarm_ring_page_shown = false;

    /* ★【2026-08-25】改用 reminder 提供的专用标志判断"是否本轮第一次响铃回调"。
     * 原先借 `reminder_get_state() != RINGING` 做这个判断，迫使 reminder 把 state
     * 的置位推迟到回调之后，结果留下"响铃画面已出、state 却还没置 RINGING"的空窗，
     * 使 ui_port 的「响铃中任意触摸关闭闹钟」分支进不去（实测：闹钟界面头部触摸无反应）。
     * 现在两个语义各用各的标志，state 可以第一时间置位，本判断也依然准确。 */
    if (reminder_is_first_ring_callback())
        s_alarm_ring_page_shown = false;

    if (type == REMINDER_TYPE_ALARM && is_alarm_start)
    {
        /* 2026-08-10 新增：闹钟响起时强制切到闹钟页显示「闹钟响铃」画面。
         *
         * ⚠️【只在第一次响铃时切页 —— 这是死锁修复，不是优化】
         * 本回调有两个来源，加锁情况完全不同：
         *   · alarm_ring_start()          reminder.c:541  **不持锁**  ← 只在这里切页
         *   · REM_EVT_ALARM_RING_TICK     reminder.c:980  **持有 s_ctx.mutex**
         * 若在后者里调 ui_show_alarm_ringing()，reminder_task 会带着 s_ctx.mutex
         * 阻塞在 lvgl_port_lock(100) 上；而 alarm_ring_stop() 同样要 s_ctx.mutex，
         * 触摸关闭闹钟走的 REM_EVT_ALARM_DISMISS 又排在同一个队列里等这次事件处理完
         * → 实测现象＝闹钟结束后卡在闹钟页，**任何触摸无反应、无法退出**
         * （触摸被 ui_port.c:6049「响铃中任意触摸关闭闹钟」分支吃掉，
         *   而那次 dismiss 永远等不到 reminder_task 来处理）。
         *
         * 用 s_ring_page_shown 保证只有第一次（不持锁那次）真正进 UI，
         * 后续重复响铃只震动、不碰 UI，reminder_task 绝不会带着互斥锁去抢 LVGL 锁。
         *
         * 【顺序：先切页、再震动】与倒计时到期同一原则——震动是阻塞调用，
         * 放前面会把界面切换推迟。 */
        if (!s_alarm_ring_page_shown)
        {
            s_alarm_ring_page_shown = true;
            ui_show_alarm_ringing();
        }
        /* 2026-08-11：由 bsp_motor_pulse()（固定 30ms 触摸级轻反馈，作为闹钟
         * 提醒太短促）改为按 ALARM_RING_VIBRATE_MS 时长震动，默认 200ms。 */
        bsp_motor_pulse_level(ALARM_RING_VIBRATE_LEVEL, ALARM_RING_VIBRATE_MS);
    }

    /* 2026-08-10 新增：倒计时到期时强制把界面切到番茄时钟页显示到期画面。
     * 原先到期只震动不碰屏幕，而倒计时启动 2 秒后就退出了功能页，用户到期时
     * 多半不在该页上 → 渲染全打在隐藏对象上 → 「日志到期了但屏幕没变化」。
     * ui_show_countdown_expired() 内部自行 lvgl_port_lock（本回调在 reminder_task
     * 上下文，非 LVGL 线程），拿不到锁会自己放弃，不会阻塞本任务。 */
    if (type == REMINDER_TYPE_TIMER)
    {
        ui_show_countdown_expired();
    }
}

//
// ═══════════════════════════════════════════════════════════════════════════════
// 5. 应用主初始化序列
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 唤醒词初始化专用临时任务（钉在 CPU1，跑完自删）
 *
 * 【为什么要单起一个任务】唤醒词模型加载（AFE create + MultiNet create + FST 编译）
 *   是约 2 秒的纯计算，原本跑在 main 任务里（main 钉 CPU0）。而 taskLVGL 也在 CPU0，
 *   两者抢同一个核：
 *     · main 用默认优先级 1 → taskLVGL(5) 压着它 → 2 秒的活拖成 26 秒，IDLE0 饿死报 task_wdt
 *     · main 提到 5 与 taskLVGL 同级 → 时间片轮转各占一半 → 加载恢复 2 秒，但 GIF 掉到
 *       半速，这 2 秒画面肉眼可见卡顿
 *   本质是 CPU0 容量守恒，在同一个核上腾挪只能二选一。
 *
 * 【改法】把这段一次性的初始化整体挪到 CPU1 执行，main 阻塞等待。
 *   main 阻塞期间不占 CPU0 任何时间片 → taskLVGL 独占 CPU0 → 开机 GIF 全速播放。
 *   加载在 CPU1 上与 AFE 分时（本任务优先级 4 < afe_fetch/audio_feed 的 5，绝不抢
 *   实时音频），耗时会比 2 秒略长，换来的是开机动画全程流畅。
 *
 * 【不变的部分】wake_word_init 内部创建的 afe_fetch / mn_detect 仍由它自己
 *   xTaskCreatePinnedToCore(..., 1, ...) 显式钉在 CPU1，绑核与优先级一个字未改，
 *   音频链路运行时行为完全不受影响。custom_wake_word.c 未做任何修改。
 *
 * 【栈必须在内部 SRAM】模型加载要读 flash，flash 操作期间 cache 被禁用，此时若任务栈
 *   在 PSRAM 会触发 esp_task_stack_is_sane_cache_disabled() 断言 panic（BUG-010 家族，
 *   同 touch_scan 任务的处理）。故用普通 xTaskCreatePinnedToCore（栈默认内部 SRAM），
 *   不能用 ...WithCaps(MALLOC_CAP_SPIRAM)。
 *   栈深 8192：main 实测栈历史最小剩余 5720B / 8192B，即含本段在内峰值仅用 2472B，余量充足。
 *
 * @param arg 调用方（main）的任务句柄，用于跑完后 xTaskNotifyGive 唤醒它
 */
static void ww_init_task(void *arg)
{
    /* MultiNet create/build_fsts 是纯计算，本任务默认优先级 4 < audio_feed/afe_fetch
     * 的 5，会被这两个周期性音频任务反复抢占，实测把约 1.6s 的计算拖成 14s（2026-07-28
     * 关闭堆毒化后实测复现，backtrace 落在 multinet6_quantized.c build_fsts 路径）。
     * 临时提到 5 与它们同级，时间片轮转各占一半，计算期间不再被完全饿住；
     * 跑完本任务立即自删，不需要手动恢复优先级。 */
    vTaskPrioritySet(NULL, 6);
    wake_word_init(wake_word_callback);
    wake_word_start();
    xTaskNotifyGive((TaskHandle_t)arg); // 通知 main：唤醒词已就绪，可以继续后续初始化
    vTaskDelete(NULL);                  // 一次性任务，跑完即销毁，不占常驻资源
}

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

    //     /* GPIO14 上电默认弱上拉，左臂舵机线焊死在此脚会被误触发抖动进而拉低电源轨，
    //      * 引发 ES8311/触摸 NACK 及其他舵机连锁失灵。必须在最开头先拉低占住该脚。
    //      */
    //     gpio_config_t io_conf_g14 = {
    //         .pin_bit_mask = (1ULL << GPIO_NUM_14),
    //         .mode = GPIO_MODE_OUTPUT,
    //         .pull_up_en = GPIO_PULLUP_DISABLE,
    //         .pull_down_en = GPIO_PULLDOWN_DISABLE,
    //         .intr_type = GPIO_INTR_DISABLE,
    //     };
    //     ESP_ERROR_CHECK(gpio_config(&io_conf_g14));
    //     gpio_set_level(GPIO_NUM_14, 0); // 打开震动
    //     bsp_motor_ledc_init();          /* 幂等：bsp_touch_init() 里还会再调一次，同参数重复配置安全 */
    //     bsp_motor_pulse_level(BOOT_VIBRATE_LEVEL, BOOT_VIBRATE_MS);

    //     bsp_flash_init();
    //     PRINT_INTERNAL_HEAP_STEP("bsp_flash_init");
    //     HEAP_CHECK_STEP("bsp_flash_init");
    //     debug_root_files();
    //     scan_production_assets("/S"); // 扫描 /S 目录下的所有资源
    //     if (access("/S/assets/gif/one.gif", F_OK) == 0)
    //         printf("路径 A 物理存在！\n");
    //     if (access("/S/gif/one.gif", F_OK) == 0)
    //         printf("路径 B 物理存在！\n");
    //     /* ── 步骤 1: BSP 单例 ──────────────────────────────────────────────────── */
    bsp_board_t *bsp_board = bsp_board_get_instance();

    //     /* ── 步骤 2: NVS Flash ─────────────────────────────────────────────────── */
    //     bsp_board_nvs_init(bsp_board);
    //     PRINT_INTERNAL_HEAP_STEP("bsp_board_nvs_init");

    //     /* ── 步骤 5: WiFi / BluFi 配网（阻塞直至获取 IP 或彻底失败后重启）───────
    //      * ★LCD/UI 初始化必须放在此步【之后】：BLE controller 使能窗口内若 LVGL 正在
    //      *   并发解码 GIF / SPI DMA 刷屏 / 投递舵机，会与 BT 抢内部资源，导致 BLE 初始化
    //      *   随机崩溃（LoadProhibited 野 handle 或 ble_svc_gap_init 断言，见 BUG-026）。
    //      *   配网不再显示二维码（已删除该逻辑），因此 UI 无需早于 WiFi，放回配网之后即可。*/
    //     bsp_board_wifi_main(bsp_board);
    //     PRINT_INTERNAL_HEAP_STEP("bsp_board_wifi_main");
    //     /* ── 步骤 2.5: LCD + UI 初始化（WiFi/配网完成后再起，避开 BLE 初始化窗口）── */
    //     bsp_board_lcd_init(bsp_board);
    //     PRINT_INTERNAL_HEAP_STEP("bsp_board_lcd_init");
    //     ui_init();
    //     vTaskDelay(pdMS_TO_TICKS(100));
    //     PRINT_INTERNAL_HEAP_STEP("ui_init");
    //     if (lvgl_port_lock(1000))
    //     {
    //         /* ★背光渐变避让：ui_init() 内部可能已启动开机 logo 渐亮（UI_BOOT_FADE_IN）。
    //          * 此时若走 bsp_board_lcd_on()，它会把背光直接拍到 100%，而渐变 timer 下一拍
    //          * （4ms 后）按自身时间进度算出较低亮度又写回去 —— 表现为渐亮途中"突然亮一下
    //          * 再暗回来"的回弹（仅首次配网路径可见，已配网设备时序不同不触发）。
    //          * 故渐变进行中只开显示控制器、把背光完全交给渐变状态机推进。 */
    //         if (ui_is_boot_fading())
    //             bsp_board_lcd_disp_on(bsp_board); // 只开显示，不碰背光
    //         else
    //             bsp_board_lcd_on(bsp_board); // 无渐变：照旧开显示 + 点背光
    //         lvgl_port_unlock();
    //     }

    //     /* ── 步骤 3: 音频硬件 + 采集任务（裸板无 ES8311，注释）──────────────── */
    //     audio_init(bsp_board);
    //     PRINT_INTERNAL_HEAP_STEP("audio_init");

    // // ── 【调试】离线音频开机自测开关 ─────────────────────────────────────────────
    // // 置 1：开机后把固件内嵌的测试音频落盘到外挂 flash /S/voice/，然后依次自动
    // //       播放 test.mp3 和 test.p3，验证双分支解码链路 + heap 泄漏对账。
    // //       验证完成后置回 0（内嵌音频与 CMakeLists 的 EMBED_FILES 可一并移除）。
    // // 前置条件：bsp_flash_init（/S 已挂载）+ audio_init（codec 就绪）均已完成。
    // #if OFFLINE_AUDIO_BOOT_TEST
    //     offline_audio_boot_test_start();
    // #endif

    //     /* ── 步骤 4: 唤醒词引擎（裸板无麦克风，注释）──────────────────────────── */
    //     /* ★ 唤醒词初始化挪到 CPU1 执行，main 在 CPU0 上阻塞等待（详见 ww_init_task 注释）。
    //      *   目的：main 阻塞期间不占 CPU0，taskLVGL 独占 CPU0 → 开机 logo GIF 全速不卡顿；
    //      *   同时模型加载在 CPU1 上正常推进，不再被 GIF 挤成 26 秒。
    //      *   优先级 4：低于 afe_fetch / audio_feed 的 5，保证实时音频链路不被抢。 */
    //     ESP_LOGW(TAG, "[开机加速] 唤醒词初始化交给 CPU1（优先级4），main 在 CPU0 阻塞等待");
    //     BaseType_t ww_ok = xTaskCreatePinnedToCore(
    //         ww_init_task,                // 任务函数
    //         "ww_init",                   // 任务名
    //         8192,                        // 栈深（内部 SRAM，实测峰值仅需 2472B）
    //         xTaskGetCurrentTaskHandle(), // 传入 main 句柄，完成后通知它
    //         4,                           // 优先级 4 < AFE 的 5，不抢实时音频
    //         NULL,                        // 不保留句柄（任务自删）
    //         1);                          // ★ 钉在 CPU1
    //     if (ww_ok != pdPASS)
    //     {
    //         // 兜底：任务建不起来（内存不足）时退回原地同步初始化，功能优先于流畅度
    //         ESP_LOGE(TAG, "ww_init 任务创建失败，退回 main 内同步初始化（开机会卡顿）");
    //         wake_word_init(wake_word_callback);
    //         wake_word_start();
    //     }
    //     else
    //     {
    //         ulTaskNotifyTake(pdTRUE, portMAX_DELAY); // 阻塞等唤醒词就绪，期间完全让出 CPU0
    //         ESP_LOGW(TAG, "[开机加速] 唤醒词就绪，main 继续后续初始化");
    //     }
    //     PRINT_INTERNAL_HEAP_STEP("wake_word_init+start");

    //     /* ── 步骤 6: MQTT 客户端 ───────────────────────────────────────────────── */
    //     protocol_mqtt_start();
    //     PRINT_INTERNAL_HEAP_STEP("protocol_mqtt_start");

    //     /* ── 步骤 7: 会话模块（WebSocket 预连接）─────────────────────────────── */
    //     //// session_init("ws://122.224.191.2:4888/ws/omni");
    //     session_init("wss://ai.strailine-space.com/ws/omni");
    //     PRINT_INTERNAL_HEAP_STEP("session_init");

    //     // 6. 创建触摸扫描任务
    //     // ⚠ 栈必须在内部 SRAM，不能放 SPIRAM！
    //     //    本任务承载整个 UI 跳转链（含游戏初始化），其中 game_whack 读写 NVS 高分会
    //     //    触发 spi_flash_disable_interrupts_caches_and_other_cpu()，期间 cache 被禁用，
    //     //    SPIRAM 栈不可访问 → esp_task_stack_is_sane_cache_disabled() 断言 panic（BUG-010 家族）。
    //     //    实测进游戏路径栈高水位剩 5888B，即峰值用量仅 2304B，故 4096 足够（留 ~1.7× 余量）。
    //     //    内部 SRAM 净增 4KB，换来彻底消除「触摸任务里碰 flash 必崩」隐患。
    //     esp_err_t ret = xTaskCreatePinnedToCoreWithCaps(
    //         touch_scan_task,
    //         "touch_scan",
    //         4096,
    //         NULL,
    //         4, // 优先级略低于舵机和音频
    //         NULL,
    //         tskNO_AFFINITY,
    //         MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    //     if (ret != pdPASS)
    //     {
    //         ESP_LOGE(TAG, "创建触摸扫描任务失败！");
    //     }
    //     else
    //     {
    //         ESP_LOGI(TAG, "触摸扫描任务创建完成");
    //         PRINT_TASK_CREATED(TAG, "touch_scan", 4096, 1); // 栈在内部SRAM
    //     }
    // /* ── 步骤 8: 舵机硬件初始化（LEDC/PWM）──────────────────────────────── */
    bsp_board_servo_init(bsp_board);
    PRINT_INTERNAL_HEAP_STEP("bsp_board_servo_init");

    // /* ── 步骤 9: 舵机管理器（队列 + worker task，栈在 SPIRAM）─────────────── */
    esp_err_t ret = servo_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "servo_manager_init 失败: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGE(TAG, "interaction_manager_init 成功");
    }
    PRINT_INTERNAL_HEAP_STEP("servo_manager_init");

    /* ── 步骤 10: 情绪交互管理器（情绪矩阵 + worker task，栈在 SPIRAM）────── */
    ret = interaction_manager_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "interaction_manager_init 失败: %s", esp_err_to_name(ret));
    }
    else
    {
        ESP_LOGE(TAG, "interaction_manager_init 成功:");
    }
    PRINT_INTERNAL_HEAP_STEP("interaction_manager_init");

    /* ══════════════════════════════════════════════════════════════════════
     * 【2026-08-31 提前】开机 logo → GIF 的切换信号挪到这里（原在步骤 11 之后）
     *
     * 【为什么提前】GIF 出现＝给用户"初始化完成、可以用了"的肉眼信号，故它应该在
     * 用户真正能感知的那几项就绪后立刻发，而不是等所有后台模块跑完。
     * 按感知优先级排序：对话 > LCD > 触摸 > 舵机 > 其他，到这一行为止：
     *   · 对话：session_init（步骤 7）已完成  ✔
     *   · LCD ：LCD/UI（步骤 2.5）已完成      ✔
     *   · 触摸：touch_scan 任务已创建（上方） ✔
     *   · 舵机：servo_init/manager（步骤 8/9）已完成 ✔
     * 四项全部就绪，此刻发信号名副其实。
     *
     * 【后面还剩什么】standby / remote_control / 电池 / reminder(天气)。这些都是
     * "其他"档：无可见产出，且 reminder_init 内含同步 IP 定位（阻塞 HTTPS 实测
     * ~850ms）+ 天气拉取，原先卡在信号前面纯属白等。它们在本行之后继续跑，
     * 用户看到 GIF 时它们仍在后台完成，不影响任何已就绪的交互。
     *
     * 【安全性】ui_notify_boot_ready() 只写标志 + 启 esp_timer，不依赖下方任何模块；
     * 反过来下方模块也不读 boot 渐变状态，两者无耦合，提前调用无副作用。
     * ══════════════════════════════════════════════════════════════════════ */
    // ui_notify_boot_ready();
    // printf("log结束标志\n");

    // 舵机测试任务（独立跑，不影响 LVGL 刷新）
    xTaskCreatePinnedToCoreWithCaps(
        servo_test_task,
        "servo_test",
        4096,
        NULL,
        5,
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    /* ── 步骤 10.5: 无活动待机模块（依赖 LCD/唤醒词/舵机管理器均已就绪）──── */
    standby_init();
    PRINT_INTERNAL_HEAP_STEP("standby_init");

    // /* ── 步骤 10.6: app 远程手动控制模块（舵机角度 / GIF 显示 + 30s 冻结窗口）──
    //  * 依赖：舵机（步骤 9）、LCD/UI（步骤 2.5）均已就绪 —— worker 任务一旦跑起来
    //  * 就可能立刻调用 bsp_servo_move_smooth / ui_request_state_gif。
    //  * 与 standby 模块无强依赖（当前 standby_init 停用也不影响本模块工作）。
    //  * 初始化失败只是远程控制不可用（后续 submit 静默丢弃），不影响其他功能，故不 abort。 */
    // if (!remote_control_init())
    // {
    //     ESP_LOGE(TAG, "remote_control_init 失败，app 远程舵机/GIF 控制将不可用");
    // }
    // PRINT_INTERNAL_HEAP_STEP("remote_control_init");

    // /* ── 步骤 11: 电池 + 提醒系统（无感/次要功能，后置以让核心链路尽早就绪）──
    //  * 挪动理由：电池监控纯 ADC 无依赖；reminder_init 内部会同步做一次 IP 定位
    //  * （阻塞 HTTPS，实测 ~850ms）+ 天气拉取。这两块对用户「看到 GIF、能对话、能
    //  * 触摸互动」毫无感知贡献，故整体后移到所有可见模块之后，不再拖慢舵机/触摸/
    //  * 待机的就绪。二者都依赖 WiFi 已连（此处 WiFi 早已就绪，无依赖风险）。 */
    // esp_err_t bat_ret = bsp_battery_init();
    // if (bat_ret == ESP_OK)
    // {
    //     bsp_battery_start_task(NULL); // 暂不接低电回调，UI 自身已带变色提示
    //     // bsp_battery_start_log_task(); // 新增：每 5s 打印一次电池电压/电量，便于调试
    //     xEventGroupSetBits(bsp_board->board_status, BATTERY_BIT);
    //     ESP_LOGI(TAG, "电池监控已启动");
    // }
    // else
    // {
    //     ESP_LOGW(TAG, "电池监控未启用 (%s)，UI 电量将显示 --%%", esp_err_to_name(bat_ret));
    // }
    // PRINT_INTERNAL_HEAP_STEP("bsp_battery_init");
    // reminder_init(on_reminder_trigger);
    // PRINT_INTERNAL_HEAP_STEP("reminder_init");
    // /* 注：ui_notify_boot_ready() 原本在这里，已提前到 touch_scan 任务创建之后
    //  * （见上方"【2026-08-31 提前】"注释块）——GIF 不再等电池/天气这些无感知模块。 */

    // /* ── 步骤 8: OTA 验证（必须在所有初始化完成后调用）──────────────────── */
    // // 若当前是刚 OTA 升级完首次启动，会进入 PENDING_VERIFY 状态：
    // //   - 调用 esp_ota_mark_app_valid_cancel_rollback() 防止 Bootloader 回滚
    // //   - 把 NVS 中的 pending_ver 提升为 committed_ver
    // // 若启动前期崩溃（未到这里），Bootloader 下次启动会自动回滚到旧固件
    // //
    // // ★ 位置说明：本段必须排在下面的诊断段【之前】。诊断段开启时要打十几秒日志，
    // //   放在它后面会让"防回滚标记"被无谓推迟同样长的时间（升级后这段时间内断电
    // //   就会被误判为启动失败而回滚），与诊断无因果关系，不该受它拖累。
    // ESP_LOGI(TAG, "当前固件版本: %s", bsp_ota_get_current_version());
    // bsp_ota_mark_valid();
    // ESP_LOGI(TAG, "后续版本使用变量");

#if APP_BOOT_DIAG_ENABLE
    /* ── 诊断：全部初始化跑完后的内存全量快照 + 逐任务栈占用清单 ──────────────
     * 用于定位"初始化结束后内部SRAM仅剩极少"的问题：
     *   - PRINT_MEM_INFO 给出内部SRAM/PSRAM当前剩余 + 历史最低值（历史最低更能反映
     *     启动过程中的峰值消耗，比"当前剩余"更接近问题根因）
     *   - uxTaskGetSystemState 遍历当前所有任务，打印每个任务的栈总深度和历史最小
     *     剩余水位（栈总深度可反推"这个任务在内部/外部SRAM分别占了多少"，需要人工
     *     核对该任务创建时用的是 MALLOC_CAP_INTERNAL 还是 MALLOC_CAP_SPIRAM）
     */
    PRINT_MEM_INFO(TAG, "★全部初始化完成★");
    PRINT_MEM_FRAG_INFO(TAG, "★全部初始化完成★");
    heap_caps_print_heap_info(MALLOC_CAP_INTERNAL); // 按尺寸区间打印空闲块分布，定位碎片具体大小

    // ── 一次性验证：区域① (0x3fcb7a98附近) 是否真的能被普通分配请求批到 ──
    // 故意申请一块接近该区域大小(32767B)的内存，打印实际拿到的地址，
    // 落在 0x3fcb7xxx 说明确实从区域①批出，落在 0x3fcb0/3fce9 说明走了区域②③。
    // 验证完毕后应删除本段（仅用于确认区域①是否被分配器实际使用）。
    {
        void *probe = heap_caps_malloc(30000, MALLOC_CAP_INTERNAL);
        ESP_LOGW(TAG, "[区域①验证] malloc(30000, INTERNAL) 实际地址 = %p", probe);
        PRINT_MEM_FRAG_INFO(TAG, "[区域①验证] 占用中（释放前）"); // 此时应比全部初始化完成时少约30000B
        if (probe)
            heap_caps_free(probe);
        PRINT_MEM_FRAG_INFO(TAG, "[区域①验证] 已释放（释放后）"); // 此时应恢复到接近释放前的水平
    }

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

                vTaskDelay(pdMS_TO_TICKS(20));
            }
            free(task_list);
        }
        else
        {
            ESP_LOGE(TAG, "[任务清单] SPIRAM分配失败，无法打印任务清单（任务数=%u）", (unsigned)task_count);
        }
    }
#endif /* APP_BOOT_DIAG_ENABLE */

    /* ── 步骤 9: CPU 占用诊断任务（调试用，可注释掉）─────────────────────── */
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
}
