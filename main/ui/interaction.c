/**
 * @file interaction.c
 * @brief 情绪矩阵与动作解析引擎（非阻塞 worker 架构）
 *
 * 架构说明：
 *   - ui_interaction_play() 仅入队，立即返回，不阻塞调用方（session/UI 任务）。
 *   - 内部 interaction_worker_task 串行消费队列，依次执行：
 *       屏幕动画 → 音效 → 震动马达 → 三轴舵机动作序列 → 舵机归中
 *   - 所有舵机调用经 bsp_servo_move_smooth()，内部持 per-channel mutex，线程安全。
 *
 * 角度约定：中心点 = 90°
 *   头部：左+ 右−  (左转30° → angle=120, 右转30° → angle=60)
 *   手臂：前+ 后−  (前摆20° → angle=110, 后摆20° → angle=70)
 */

#include "interaction.h"
#include "bsp/servo_manager.h" // SERVO_SPEED_* 常量
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "freertos/semphr.h" // 完成通知信号量
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include <stdatomic.h> // atomic_bool 播放中标志（跨核安全）
#include "bsp/bsp_board.h"
#include "ui/ui_port.h"
#include "ui/remote_control.h"   // remote_control_is_active：收尾 flush 不得打断远程接管运动
#include "object.h"              // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM
#include "audio/offline_audio.h" // 离线音频播放（情绪音效 → 外挂 flash /S/voice/）
#include <stdio.h>               // snprintf 拼音频绝对路径

static const char *TAG = "INTERACTION";

/* ═══════════════════════════════════════════════════════════════════════════
 * 【2026-08-31 问题3】情绪音效（离线提示音）总开关
 *
 * 需求：暂时去除情绪的声音。置 0 后，触摸触发情绪时【不再播放】/S/voice/ 下的
 * 离线音效，GIF、震动、舵机三项全部不受影响，照常执行。
 *
 * 【为什么用宏而不是删代码】用户明确说是"暂时"。情绪矩阵里 20 条 .audio_file
 * 字段全部保留原样（见下方 g_emotion_matrix），要恢复只需把本宏置回 1，
 * 一行开关即可，不必再去逐条补回文件名。
 *
 * 【影响范围】只管情绪音效这一条链路。以下不受本开关影响：
 *   · 唤醒提示音、闹钟/倒计时的震动提醒 —— 走的是别的模块，与此无关；
 *   · application.c 的 OFFLINE_AUDIO_BOOT_TEST 开机自测 —— 已是 0，本就不播。
 * 即：全项目 offline_audio_play() 的调用点只有两处，另一处已关，故本宏置 0 后
 * 离线提示音链路整体静默。
 * ═══════════════════════════════════════════════════════════════════════════ */
#define IA_EMOTION_AUDIO_ENABLE 0

// ─── 情绪播放状态（供 ui_port 自动循环/触摸路由读取，跨核 atomic）──────────────
// true = 正在执行某动作（情绪/对话状态/空闲）。置位早于 GIF/震动，清位在动作完成后。
static atomic_bool s_ia_playing = ATOMIC_VAR_INIT(false);
// true = 当前正在播放的是“空闲动作”（低优先级，可被触摸情绪打断）。
// 区分：情绪动作 / 对话状态动作（keep_screen=true）置 false；空闲动作（keep_screen=false）置 true。
static atomic_bool s_ia_is_idle = ATOMIC_VAR_INIT(false);
// true = 低功耗（一级待机）头部专属模式：情绪播放时手臂钉 90°（count 清零）+ 跳过震动，
// 头部动作/GIF 切图不受影响。由 standby.c 在 enter_standby/standby_wake 时置位/清位。
static atomic_bool s_ia_lowpower = ATOMIC_VAR_INIT(false);
// 情绪舵机动作完成信号量：worker 入队舵机请求带此 sem，servo_manager 执行完 give。
static SemaphoreHandle_t s_ia_d1_2_sem = NULL;

bool interaction_is_playing(void)
{
    return atomic_load(&s_ia_playing);
}

bool interaction_is_idle_action(void)
{
    return atomic_load(&s_ia_is_idle);
}

void interaction_set_lowpower(bool enable)
{
    atomic_store(&s_ia_lowpower, enable);
}

// interaction_flush_queue 定义在文件末尾（需在 s_ia_queue/s_ia_inited 定义之后）。

// ─── Worker task 配置 ────────────────────────────────────────────────────────
#define INTERACTION_QUEUE_LEN 8     ///< 最多缓存 4 个待执行情绪（超出时丢弃新请求）
#define INTERACTION_TASK_STACK 8192 ///< worker 栈大小（含音频/舵机调用链）
#define INTERACTION_TASK_PRIO 5     ///< 优先级与 session 相当，略低于音频（7）

static QueueHandle_t s_ia_queue = NULL; ///< 动作请求队列（情绪 ID / 自定义动作）
static TaskHandle_t s_ia_worker = NULL; ///< worker 任务句柄

/// OTA 停止标志：置 true 后 worker 丢弃一切请求、不再切 GIF/驱动舵机/震动。
/// 用于 OTA 升级期间彻底静默 interaction（GIF+舵机+震动）。置位后不清除（OTA 必重启）。
static volatile bool s_ia_stopped_for_ota = false;
static bool s_ia_inited = false; // 初始化完成

// ==========================================
// 1. 数据结构：严格对齐情绪矩阵表头
// ==========================================

/* ActionStep_t / VibStep_t 已移到 interaction.h 暴露（供 ui_port 构造状态动作表）。 */

/* ── 命名震动序列（对应情绪队列表 D 列的 10 种震动类型）────────────────────
 * 强度用中间占空比（真 PWM 方波，0/100 为直流端点）。这里是初版手感值，
 * 烧录后用示波器/体感逐项微调 strength/on_ms/off_ms 即可，不必改结构。
 * 多个情绪可复用同一序列（同震动类型）。 */
// static const VibStep_t vib_short2[] = {{70, 50, 50}, {70, 50, 0}}; // 短促震动 2 次
static const VibStep_t vib_light1[] = {{50, 60, 0}}; // 轻微震动 1 次
// static const VibStep_t vib_light2[] = {{50, 60, 60}, {50, 60, 0}};                  // 轻微震动 2 次
// static const VibStep_t vib_short_strong2[] = {{85, 50, 50}, {85, 50, 0}};           // 短促强震动 2 次
// static const VibStep_t vib_strong2[] = {{100, 90, 70}, {100, 90, 0}};               // 强震动 2 次
// static const VibStep_t vib_fast_cont[] = {{80, 30, 25}, {80, 30, 25}, {80, 30, 25}, // 连续/快速连续震动
//                                           {80, 30, 25},
//                                           {80, 30, 0}};
// static const VibStep_t vib_slow_long1[] = {{45, 400, 0}};                  // 缓慢长震动 1 次
// static const VibStep_t vib_soft_cont[] = {{35, 500, 0}};                   // 持续轻柔震动
// static const VibStep_t vib_intermittent[] = {{45, 40, 120}, {45, 40, 120}, // 轻微间断震动
//                                              {45, 40, 120},
//                                              {45, 40, 0}};
/* 「无震动」用 vib_seq = NULL 表示，无需定义序列。 */

/**
 * @brief 情绪矩阵行：情绪 ID → 全套硬件动作映射
 */
typedef struct
{
    robot_emotion_t emotion_id; ///< 情绪枚举 ID（查表键）
    const char *screen_anim;    ///< 屏幕动画标识（人类可读描述，仅日志用）
    const char *gif_path;       ///< 该情绪的 GIF 路径（占位符，后续接外挂flash）；NULL=不切图
    const VibStep_t *vib_seq;   ///< 震动序列（NULL=无震动）；按段顺序播放，与 GIF 大致同期
    uint8_t vib_seq_len;        ///< 震动序列长度（段数）
    const char *audio_file;     ///< 音效文件名（传给音频层，预留）
    /* ★2026-09-04：三轴由 ActionStep_t（A↔B 往返 N 次）改为 ActionSeq_t（终点+动作序列）。
     * 新流程：90°(起始) → target(终点) → seq[](动作序列) → 归中90°。
     * 三轴各走各的步数与速度，互不同步；详见 interaction.h 的 ActionSeq_t 注释。
     * 空闲/状态动作走的 ia_custom_action_t 仍用旧 ActionStep_t，本次不动。 */
    ActionSeq_t head;      ///< 头部舵机（CH_HEAD）
    ActionSeq_t left_arm;  ///< 左臂舵机（CH_L_ARM）
    ActionSeq_t right_arm; ///< 右臂舵机（CH_R_ARM）
} InteractionMatrix_t;

// ══════════════════════════════════════════════════════════════════════════════
//   ⚠️【幅度硬约束】相邻两个角度点的差值须 ≥ 3~5°：舵机死区约 1°，再加齿隙，
//      ±1~2° 的微动在本硬件上肉眼看不出来（这正是 BUG-033 的成因）。
//
// ══════════════════════════════════════════════════════════════════════════════
/* ★序列上限三处一致性护栏（2026-09-11 新增）
 *
 * 序列上限分散在三个头文件里，改一处忘了另两处的后果是【静默截断】——
 * 动作少走几步，不报错也不打日志，只能靠肉眼发现"这个情绪好像没做完"。
 * 用编译期断言把它变成编译错误，改漏了当场就知道。
 *
 *   ACTION_SEQ_MAX_STEPS     (interaction.h)   配置侧：表里能填几步
 *   SERVO_SEQ_MAX_STEPS      (servo_manager.h) 中间层：队列消息能装几步
 *   BSP_SERVO_SEQ_MAX_POINTS (bsp_board.h)     底层：一串角度点的容量
 *
 * 底层要 ≥ 配置侧 +2，因为 servo_exec_seq 把"走向终点"和"归中"这两步也拼进
 * 同一串点（归中自 2026-09-17 起由 worker 下沉为每轴最后一段）。 */
_Static_assert(ACTION_SEQ_MAX_STEPS == SERVO_SEQ_MAX_STEPS,
               "配置侧与执行侧的序列上限必须相等，否则长序列会被静默截断");
_Static_assert(BSP_SERVO_SEQ_MAX_POINTS >= ACTION_SEQ_MAX_STEPS + 2,
               "底层点数容量须 ≥ 序列上限+2（多的 2 个给'走向终点'和'归中'两步）");

static const InteractionMatrix_t g_emotion_matrix[] = {
    // // ── 1 ：1_1:中性1 ──────────────────────────────────────────────────────────────
    // {
    //     .emotion_id = EMO_NEUTRAL1,
    //     .screen_anim = "中性1.gif",
    //     .gif_path = "S:/gif/1_1.gif", // 中性1
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 2,
    //     // 由于没有设计师，audio_file音频暂时放弃。全部删除改动太大，暂时占位
    //     .audio_file = "laugh_short.mp3", // 暂弃
    //     .head = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {70.0f},
    //             {110.0f},
    //             {70.0f},
    //             {110.0f},
    //             {70.0f},
    //             {110.0f},
    //             {70.0f},
    //             {110.0f},
    //             {90.0f},
    //         },
    //         .seq_len = 9,
    //         .seq_speed = SERVO_SPEED_SLOWER,
    //     },
    //     .left_arm = {
    //         .target = 170.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {140.0f},
    //             {170.0f},
    //             {140.0f},
    //             {170.0f},
    //             {140.0f},
    //             {170.0f},
    //             {140.0f},
    //             {170.0f},
    //         },
    //         .seq_len = 8,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .right_arm = {
    //         .target = 170.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {140.0f},
    //             {170.0f},
    //             {140.0f},
    //             {170.0f},
    //             {140.0f},
    //             {170.0f},
    //             {140.0f},
    //             {170.0f},
    //         },
    //         .seq_len = 8,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    // },

    // // ── 1: 中性1_2 ──────────────────────────────────────────────────────────────
    // {
    //     .emotion_id = EMO_NEUTRAL2,
    //     .screen_anim = "中性2.gif",
    //     .gif_path = "S:/gif/1_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "doubt.mp3",
    //     .head = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {75.0f},
    //             {105.0f},
    //             {75.0f},
    //             {105.0f},
    //             {75.0f},
    //             {105.0f},
    //             {75.0f},
    //             {105.0f},
    //             {75.0f},
    //             {105.0f},
    //             {90.0f},
    //         },
    //         .seq_len = 11,
    //         .seq_speed = SERVO_SPEED_SLOWER,
    //     },
    //     .left_arm = {
    //         .target = 50.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {15.0f},
    //         },
    //         .seq_len = 11,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .right_arm = {
    //         .target = 50.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {20.0f},
    //             {50.0f},
    //             {15.0f},
    //         },
    //         .seq_len = 11,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    // },

    // // ── 1: 中性1_3 ──────────────────────────────────────────────────────────────
    // {
    //     .emotion_id = EMO_NEUTRAL3,
    //     .screen_anim = "中性3.gif",
    //     .gif_path = "S:/gif/1_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "doubt.mp3",
    //     .head = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {60.0f, 0, 750},
    //             {120.0f, 0, 750},
    //             {90.0f, 0, 750},
    //             {60.0f, 0, 750},
    //             {120.0f, 0, 500},
    //             {90.0f},
    //         },
    //         .seq_len = 6,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 20.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {75.0f, 0, 100},
    //             {10.0f},
    //             {75.0f, 0, 100},
    //             {10.0f},
    //             {75.0f, 0, 100},
    //             {15.0f},
    //         },
    //         .seq_len = 6,
    //         .seq_speed = SERVO_SPEED_SLOWER,
    //     },
    //     .right_arm = {
    //         .target = 20.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {75.0f, 0, 100},
    //             {10.0f},
    //             {75.0f, 0, 100},
    //             {10.0f},
    //             {75.0f, 0, 100},
    //             {15.0f},
    //         },
    //         .seq_len = 6,
    //         .seq_speed = SERVO_SPEED_SLOWER,
    //     },
    // },

    // // ── 1: 中性1_4 ──────────────────────────────────────────────────────────────
    // {
    //     .emotion_id = EMO_NEUTRAL4,
    //     .screen_anim = "中性4.gif",
    //     .gif_path = "S:/gif/1_4.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "doubt.mp3",
    //     .head = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {90.0f, 0, 1000},
    //             {100.0f},
    //             {80.0f},
    //             {100.0f},
    //             {80.0f},
    //             {90.0f, 0, 1000},
    //             {100.0f},
    //             {80.0f},
    //             {100.0f},
    //             {90.0f},
    //         },
    //         .seq_len = 10,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 130.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {130.0f, 0, 450},
    //             {140.0f},
    //             {120.0f},
    //             {140.0f},
    //             {120.0f},
    //             {140.0f, 0, 800},
    //             {120.0f},
    //             {140.0f},
    //             {120.0f},
    //             {140.0f},
    //             {0.0f},

    //         },
    //         .seq_len = 11,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .right_arm = {
    //         .target = 15.0f,
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {

    //         },
    //         .seq_len = 0,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    // },
    // // ── 1: 中性1_5 ──────────────────────────────────────────────────────────────
    // {
    //     .emotion_id = EMO_NEUTRAL5,
    //     .screen_anim = "中性5.gif",
    //     .gif_path = "S:/gif/1_5.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "doubt.mp3",
    //     .head = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {90.0f, 0, 200},
    //             {60.0f, 0, 700},
    //             {120.0f, 0, 300},
    //             {90.0f, 0, 300},
    //             {85.0f},
    //             {95.0f},
    //             {85.0f},
    //             {95.0f},
    //             {90.0f},
    //         },
    //         .seq_len = 8,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 15.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {15.0f, 0, 1500},
    //             {90.0f},
    //             {15.0f},
    //         },
    //         .seq_len = 3,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .right_arm = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {15.0f},
    //         },
    //         .seq_len = 1,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    // },

    {
        // ── 1_6: 中性①「中性表情3」── 主语：头（两次偏右的打量）
        //   【素材真身，按 duration 累加的真实时间轴量的】周期 3170ms，里面是四段：
        //     0~1000   眼睛形状疾速互换 8 下（每 ~120ms 一次，diff 310↔463 交替）
        //     1000~1380 【静止 380ms】
        //     1380~2500 由弱渐强，+1580 处出现全片最大变化（628）=「双眼同时变方」
        //     2500~2880 【静止 380ms】
        //     2880~3170 中等变化收尾
        //   ⇒ 身体语言照抄这个骨架 = 两段大行程 + 两处停顿，且【停顿落在素材真静止处】。
        //
        //   ★为什么不做素材那 8 下快拍：40ms 节拍下最快 1.6°/帧，一个 8° 的小动作
        //     也要 200ms —— 125ms 一拍的节奏在舵机上物理不可达。故压成两次大行程，
        //     这是【硬件做不到】的取舍，不是没看素材。
        //   ★幅度：单步 12°~22°（13_x 红线要求 16~25° 才看得见）。
        //     上一版只给 5~11°，且 79~86° 全都贴着中位 ⇒ 上板观感是"头被卡住了"。
        //   ★全程 66~90°，一次都不越过 90° ⇒ 偏置往复，读作"打量"不是"摇头"（6_x 判据）。
        //   ★最长静止段 360ms（上一版 960ms 才是"卡住"的真凶）+ 总时长 3160ms = 0.997 周期。
        //   ★右臂【刻意不参与】（speed=0 & seq_len=0 ⇒ 该轴 n=0，钉在基线 15° 不动），
        //     与左臂"抬到半空举着"形成一前一后的不对称 —— 这是本张的性格来源。
        .emotion_id = EMO_NEUTRAL6,
        .screen_anim = "中性表情3.gif",
        .gif_path = "S:/gif/1_6.gif",
        .vib_seq = vib_light1,
        .vib_seq_len = 1,
        .audio_file = NULL,
        // 头：90→68(22°) 打量到底 → 停 320ms → 80 → 66 → 88(22°) 后仰 → 停 320ms → 74 → 90
        //     行程 22/12/14/22/14/16°，两处停顿正好压在素材 1000~1380 与 2500~2880 的静止段上
        .head = {.target = 68.0f, .speed = SERVO_SPEED_SLOW, .seq = {
                                                                 {68.0f, 0, 320},                // ★停 320ms（素材静止段①）
                                                                 {80.0f, SERVO_SPEED_SLOW, 0},   // 收回 12°
                                                                 {66.0f, SERVO_SPEED_SLOW, 0},   // 再下去 14°
                                                                 {88.0f, SERVO_SPEED_SLOW, 320}, // ★22° 最大行程（对素材 +1580 的高潮）
                                                                 {74.0f, SERVO_SPEED_SLOW, 0},   // 收回 14°
                                                                 {90.0f, SERVO_SPEED_SLOW, 0},   // 16° 收在中位，归中零位移
                                                             },
                 .seq_len = 6,
                 .seq_speed = SERVO_SPEED_SLOW},
        // 左臂（配角）：15→34 抬起来，中途两次小停顿把它"举在半空别急着放"
        .left_arm = {.target = 34.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                       {34.0f, 0, 680},
                                                                       {30.0f, SERVO_SPEED_SLOWER, 400},
                                                                       {22.0f, SERVO_SPEED_SLOWER, 0},
                                                                       {36.0f, SERVO_SPEED_SLOWER, 600},
                                                                       {15.0f, SERVO_SPEED_SLOWER, 0},
                                                                   },
                     .seq_len = 5,
                     .seq_speed = SERVO_SPEED_SLOWER},
        // 右臂：不参与（保持基线 15°）
        .right_arm = {.target = 0.0f, .speed = 0, .seq_len = 0, .seq_speed = 0},
    },
    {
        // ── 1_7: 中性②「中性聆听1-1-1」── 主语：两臂（全程不停的匀速往复）
        //   【素材真身】5 秒里 diff 恒在 490~520、【一次停顿都没有】—— 这是"匀速三角波"
        //     的指纹（若是正弦，峰值转折处 diff 会趋近 0）。周期实测 1000ms ×5。
        //   ⇒ 身体语言必须【全程在动】：上一版给头两段 1200ms 静止（占 88% 时间），
        //     和素材"永不停"的性格正好相反 —— 这就是"与 GIF 不匹配"的根因。
        //   ★头改成 8 个连续小摆、最长静止只有 160ms，全程 70~90° 偏置往复不跨中位。
        //   ★两臂仍同幅(23°/17°)同速(@SLOWER)，但【停顿时长故意不同】(360/400 vs 720/400/520)
        //     ⇒ 两条臂会自然错开相位，读作"呼吸"而不是"复制粘贴的复制品"。
        //   ★右臂末步收在基线 15° ⇒ 归中是零位移，只花 1 帧。
        //   ★总时长 3000ms = 3.000 × 素材周期，头/左臂/右臂三轴全部精确 3000ms。
        .emotion_id = EMO_NEUTRAL7,
        .screen_anim = "中性聆听1.gif",
        .gif_path = "S:/gif/1_7.gif",
        .vib_seq = vib_light1,
        .vib_seq_len = 1,
        .audio_file = NULL,
        // 头：8 个连续小摆（16/14/16/14/16/16/10/14°），只有一处 160ms 的呼吸停顿。
        //     ⚠️ 全部 ≤ 90° ⇒ 偏在一侧往复，不是摇头（6_x 判据）。
        .head = {.target = 74.0f, .speed = SERVO_SPEED_SLOW, .seq = {
                                                                 {74.0f, 0, 160},                // 唯一的停顿（160ms）
                                                                 {88.0f, SERVO_SPEED_SLOW, 0},   // 回 14°
                                                                 {72.0f, SERVO_SPEED_SLOW, 0},   // 出 16°
                                                                 {86.0f, SERVO_SPEED_SLOW, 0},   // 回 14°
                                                                 {70.0f, SERVO_SPEED_SLOW, 0},   // 出 16°
                                                                 {86.0f, SERVO_SPEED_SLOW, 0},   // 回 16°
                                                                 {78.0f, SERVO_SPEED_SLOW, 0},   // 出 8°
                                                                 {90.0f, SERVO_SPEED_SLOW, 0},   // 回 12°，收在中位
                                                             },
                 .seq_len = 8,
                 .seq_speed = SERVO_SPEED_SLOW},
        // 左臂：15↔38（23°）往返两次，末步收在基线
        .left_arm = {.target = 38.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                       {38.0f, 0, 320},
                                                                       {15.0f, SERVO_SPEED_SLOWER, 360},
                                                                       {38.0f, SERVO_SPEED_SLOWER, 360},
                                                                       {15.0f, SERVO_SPEED_SLOWER, 0},
                                                                   },
                     .seq_len = 4,
                     .seq_speed = SERVO_SPEED_SLOWER},
        // 右臂：15↔32（17°）往返两次。⚠️ target 必须填 15（= 手臂基线）而不是 32：
        //   填 32 会让点链变成 15→32→15→32，多出一次多余的上下摆（本组第一版踩过）。
        .right_arm = {.target = 32.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                        {32.0f, 0, 680},                  // 起手先停 680ms ⇒ 与左臂错开相位
                                                                        {15.0f, SERVO_SPEED_SLOWER, 360},
                                                                        {32.0f, SERVO_SPEED_SLOWER, 480},
                                                                        {15.0f, SERVO_SPEED_SLOWER, 0},
                                                                    },
                      .seq_len = 4,
                      .seq_speed = SERVO_SPEED_SLOWER},
    },
    {
        // ── 1_8: 中性③「中性聆听2」── 主语：头（4 次眨眼同步的小"嗯"）
        //   素材 = 每 2500ms 里眨 4 次，起点 +0/+380/+1040/+1380ms，
        //   深度【浅浅深深】。翻译成身体 = 头跟着做 4 次小动，
        //   前两次小(6°/4°)、后两次大(9°/7°)，与眨眼深浅同构。
        //   ★落点全部 ≤ 88°（不跨 90°），读作"一下一下地应声"。
        .emotion_id = EMO_NEUTRAL8,
        .screen_anim = "中性聆听2.gif",
        .gif_path = "S:/gif/1_8.gif",
        .vib_seq = vib_light1,
        .vib_seq_len = 1,
        .audio_file = NULL,
        // 头：4 拍小动。起点都压在素材眨眼点上（+0/+360/+960/+1360 vs +0/+380/+1040/+1380）
        .head = {.target = 84.0f, .speed = SERVO_SPEED_SLOW, .seq = {
                                                                 {84.0f, 0, 180},                // 眨眼1(浅)：落 6°，微停
                                                                 {88.0f, SERVO_SPEED_SLOW, 500}, // 眨眼2(浅)：回 4°，停 500ms
                                                                 {79.0f, SERVO_SPEED_SLOW, 200}, // 眨眼3(深)：出去 9°，只停 200ms
                                                                 {86.0f, SERVO_SPEED_SLOW, 840}, // 眨眼4(深)：回 7°，长停收尾
                                                             },
                 .seq_len = 4,
                 .seq_speed = SERVO_SPEED_SLOW},
        // 左臂（慢参考）：只抬 13° 就举着，不做任何花活 —— 主角是头，臂一忙就抢戏
        .left_arm = {.target = 28.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                       {28.0f, 0, 1400},
                                                                   },
                     .seq_len = 1,
                     .seq_speed = SERVO_SPEED_SLOWER},
        // 右臂：★与左臂【错开 600ms 起手】—— 先原地停 500ms 再抬。
        //   ⚠️ target 必须填 15（基线），理由同 1_7 右臂。
        .right_arm = {.target = 15.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                        {15.0f, 0, 500}, // 等左臂先动
                                                                        {28.0f, SERVO_SPEED_SLOWER, 900},
                                                                    },
                      .seq_len = 2,
                      .seq_speed = SERVO_SPEED_SLOWER},
    },
    {
        // ── 1_9: 中性④「中性聆听3」── 主语：头（每 750ms 一拍，共 4 拍）
        //   🔴【上一版把素材周期量错了】：旧记录写"周期 3000ms = A段2000 + B段1000"，
        //     实际逐帧量 diff 后是【每 750ms 一次强事件】：高能平台出现在
        //     t = 540/1290/2040/2790/3540/4290，间隔恒为 750ms（5000ms ÷ 750 = 6.67 个周期）。
        //     旧结论是"按等间隔抽帧看montage"产生的走样，不是真实节拍。
        //   ⇒ 身体语言 = 【每 750ms 一次清楚的"点亮"】，4 拍正好 3000ms = 4.000 周期。
        //   ★素材里眼睛是"一开一闭互换"（天生对称），若照抄成 90° 两侧对称摆就是【摇头】；
        //     故整段压在 70~90° 一侧往复（偏置往复 = 打量/轻晃，6_x 判据允许）。
        //   ★单步 18~20°（13_x 红线 16~25°）；上一版只有 4~8°，肉眼几乎看不出。
        //   ★最长静止段 280ms（上一版 620ms）；发屏节拍 480/440/440/440 对素材 750ms 一拍。
        .emotion_id = EMO_NEUTRAL9,
        .screen_anim = "中性聆听3.gif",
        .gif_path = "S:/gif/1_9.gif",
        .vib_seq = vib_light1,
        .vib_seq_len = 1,
        .audio_file = NULL,
        // 头：4 拍，每拍 = 一次 18~20° 的移动 + 200~240ms 停顿
        .head = {.target = 70.0f, .speed = SERVO_SPEED_SLOW, .seq = {
                                                                 {70.0f, 0, 200},                // 拍1：20° 到位，停 200ms
                                                                 {88.0f, SERVO_SPEED_SLOW, 240}, // 拍2：18° 回摆，停 240ms
                                                                 {70.0f, SERVO_SPEED_SLOW, 240}, // 拍3：18° 出去，停 240ms
                                                                 {88.0f, SERVO_SPEED_SLOW, 240}, // 拍4：18° 回摆，停 240ms
                                                                 {90.0f, SERVO_SPEED_SLOW, 0},   // 收在中位
                                                             },
                 .seq_len = 5,
                 .seq_speed = SERVO_SPEED_SLOW},
        // 左臂：15↔40（25°）两次开合，中途有停顿；末步收在基线 15°
        .left_arm = {.target = 38.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                       {38.0f, 0, 200},
                                                                       {20.0f, SERVO_SPEED_SLOWER, 0},
                                                                       {40.0f, SERVO_SPEED_SLOWER, 240},
                                                                       {26.0f, SERVO_SPEED_SLOWER, 0},
                                                                       {38.0f, SERVO_SPEED_SLOWER, 200},
                                                                       {15.0f, SERVO_SPEED_SLOWER, 0},
                                                                   },
                     .seq_len = 6,
                     .seq_speed = SERVO_SPEED_SLOWER},
        // 右臂：与左臂【同相但幅度更小(19° vs 25°)】，且停顿长短不同 ⇒ 不会读成复制品
        .right_arm = {.target = 34.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
                                                                        {34.0f, 0, 320},
                                                                        {18.0f, SERVO_SPEED_SLOWER, 0},
                                                                        {36.0f, SERVO_SPEED_SLOWER, 240},
                                                                        {20.0f, SERVO_SPEED_SLOWER, 0},
                                                                        {34.0f, SERVO_SPEED_SLOWER, 240},
                                                                        {15.0f, SERVO_SPEED_SLOWER, 0},
                                                                    },
                      .seq_len = 6,
                      .seq_speed = SERVO_SPEED_SLOWER},
    },
    //  // ══════════════════════════════════════════════════════════════════════════
    //     // ── 10- 2_1傲娇① 别过脸·哼（配 assets/gif/2_1.gif = 傲娇1.gif：斜眼+波浪嘴）──────
    //     {
    //         .emotion_id = EMO_TSUNDERE_BASE,
    //         .screen_anim = "傲娇1.gif",
    //         .gif_path = "S:/gif/2_1.gif",
    //         .vib_seq = vib_light1,
    //         .vib_seq_len = 1,
    //         .audio_file = "doubt.mp3",
    //         .head = {
    //             .target = 142.0f,
    //             .speed = SERVO_SPEED_FAST,
    //             .seq = {
    //                 {142.0f, 0, 800},
    //                 {130.0f, SERVO_SPEED_MID, 1200},
    //                 {145.0f, SERVO_SPEED_FAST, 800},

    //             },
    //             .seq_len = 3,
    //             .seq_speed = SERVO_SPEED_MID,
    //         },
    //         .left_arm = {
    //             .target = 104.0f,
    //             .speed = SERVO_SPEED_VERY_FAST,
    //             .seq = {
    //                 {104.0f, 0, 3105},
    //                 {15.0f, SERVO_SPEED_MID},
    //             },
    //             .seq_len = 2,
    //             .seq_speed = SERVO_SPEED_MID,
    //         },
    //         .right_arm = {
    //             .target = 100.0f,
    //             .speed = SERVO_SPEED_VERY_FAST,
    //             .seq = {
    //                 {100.0f, 0, 3105},
    //                 {15.0f, SERVO_SPEED_MID},
    //             },
    //             .seq_len = 2,
    //             .seq_speed = SERVO_SPEED_MID,
    //         },
    //     },

    //     // ── 11- 2_2傲娇────────
    //     {
    //         .emotion_id = EMO_TSUNDERE_ROLL,
    //         .screen_anim = "傲娇2.gif",
    //         .gif_path = "S:/gif/2_2.gif",
    //         .vib_seq = vib_light1,
    //         .vib_seq_len = 1,
    //         .audio_file = "doubt.mp3",
    //         .head = {
    //             .target = 70.0f,           // 小=右：先甩到右侧
    //             .speed = SERVO_SPEED_FAST, // 20°×10 ≈ 200ms
    //             .seq = {
    //                 {70.0f, 0, 250},
    //                 {140.0f, SERVO_SPEED_VERY_FAST, 1500},
    //                 {110.0f, 0, 350},
    //                 {96.0f, SERVO_SPEED_MID, 500},
    //                 {130.0f, SERVO_SPEED_VERY_FAST},
    //                 {130.0f, 0, 500},
    //             },
    //             .seq_len = 6,
    //             .seq_speed = SERVO_SPEED_MID,
    //         },
    //         .left_arm = {
    //             .target = 62.0f,
    //             .speed = SERVO_SPEED_MID,
    //             .seq = {
    //                 {62.0f, 0, 3105},
    //                 {15.0f, SERVO_SPEED_FAST},
    //             },
    //             .seq_len = 2,
    //             .seq_speed = SERVO_SPEED_MID,
    //         },
    //         .right_arm = {
    //             .target = 58.0f,
    //             .speed = SERVO_SPEED_MID,
    //             .seq = {
    //                 {58.0f, 0, 3105},
    //                 {15.0f, SERVO_SPEED_FAST},
    //             },
    //             .seq_len = 2,
    //             .seq_speed = SERVO_SPEED_MID,
    //         },
    //     },
    // ── 12 -2_3傲娇③ 绷住·抬臂挡（配 assets/gif/2_3.gif = 傲娇3.gif）★主角是手 ────────
    // {
    //     .emotion_id = EMO_TSUNDERE_PEEK,
    //     .screen_anim = "傲娇3.gif",
    //     .gif_path = "S:/gif/2_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "doubt.mp3",
    //     .head = {
    //         .target = 110.0f,          // 先扭到 110°
    //         .speed = SERVO_SPEED_FAST, // 34°×10 ≈ 340ms
    //         .seq = {
    //             {110.0f, 0, 300},              // 先顿一下
    //             {144.0f, SERVO_SPEED_FAST},    // ★再补 10° 过去 —— 分两次到位
    //             {144.0f, 0, 900},              // 长僵住
    //             {98.0f, SERVO_SPEED_MID, 400}, // 松一点，不回满
    //         },
    //         .seq_len = 4,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 100.0f,          // 第一段：先抬到 100°，一次到不了位
    //         .speed = SERVO_SPEED_FAST, // 85°×10 ≈ 850ms
    //         .seq = {
    //             {100.0f, 0, 300},                // 顿一下（与头部 110° 那 300ms 停顿同拍）
    //             {124.0f, SERVO_SPEED_VERY_FAST}, // ★第二段猛地补上去：24°×5 ≈ 120ms
    //             {124.0f, 0, 700},                // 抱在胸前
    //             {117.0f, SERVO_SPEED_MID, 600},  // ★回弹 7°（overshoot）后停住
    //             {15.0f, SERVO_SPEED_MID},        // 慢慢放下：102°×15 ≈ 1530ms
    //         },
    //         .seq_len = 5,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .right_arm = {
    //         .target = 94.0f, // 与左臂刻意差 6°（124/118 同源的差量）
    //         .speed = SERVO_SPEED_FAST,
    //         .seq = {
    //             {94.0f, 0, 300},
    //             {118.0f, SERVO_SPEED_VERY_FAST},
    //             {118.0f, 0, 700},
    //             {111.0f, SERVO_SPEED_MID, 600}, // 回弹 7°
    //             {15.0f, SERVO_SPEED_MID},
    //         },
    //         .seq_len = 5,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    // },

    // // ── 傲娇④ 求摸（配 assets/gif/2_4.gif = 傲娇求摸.gif）──────────────────────
    // {
    //     .emotion_id = EMO_TSUNDERE_PET,
    //     .screen_anim = "傲娇4.gif",
    //     .gif_path = "S:/gif/2_4.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "doubt.mp3",
    //     .head = {
    //         .seq = {
    //             {60.0f},
    //             {110.0f},
    //             {60.0f},
    //             {110.0f},
    //         },
    //         .seq_len = 4,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 45.0f,           // 第一段：先抬到 100°，一次到不了位
    //         .speed = SERVO_SPEED_FAST, // 85°×10 ≈ 850ms
    //         .seq = {
    //             {45.0f, 0, 300},
    //             {55.0f},
    //             {35.0f},
    //             {55.0f},
    //             {35.0f},
    //             {55.0f},

    //         },
    //         .seq_len = 6,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .right_arm = {
    //         .target = 45.0f, // 与左臂刻意差 6°（124/118 同源的差量）
    //         .speed = SERVO_SPEED_FAST,
    //         .seq = {
    //             {45.0f, 0, 300},
    //             {35.0f},
    //             {55.0f},
    //             {35.0f},
    //             {55.0f},
    //             {35.0f},

    //         },
    //         .seq_len = 6,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    // },

    // ── 3: 兴奋3_1 ──────────────────────────────────────────────────────────────

    // ══════════════════════════════════════════════════════════════════════════
    // 兴奋四连（2026-09-20 新增，配 3_1~3_4.gif）
    //
    // 【第一性原理：兴奋在运动学上到底是什么】
    //   兴奋 = 高唤醒 + 正效价。落到身体上是三件事：①频率高 ②起停快（有回弹）
    //   ③重心往上提。反过来，"慢、匀速、大范围"无论幅度多大都不像兴奋，
    //   那读起来是"晃"或者"得意"。
    //
    // 【本机的三个硬事实，决定了它只能这么做】
    //   ① 没有竖直自由度 —— 三轴只有 head 左右、L/R_arm 前后，做不出真正的"跳"。
    //      于是拿【双臂前抬】当"跳"的替身：手臂每抬起一次，视觉上身体就提一下。
    //      ★所以本情绪的【主角是双臂，不是头】。
    //   ② 头大幅左右摆 = 摇头 = 否定，语义与兴奋正好相反 —— 头只能做 ±6° 的
    //      小幅跟随，且用比手臂慢一档的速度，充当"被身体带动的余震"。
    //   ③ 相邻点差 <3~5° 肉眼不可见（死区+齿隙，BUG-033 的成因）—— 想"抖"就得
    //      每步实实在在地走出去，不能靠微动堆出来。
    //
    // 【节拍怎么定的】
    //   单步耗时 = |Δ角| × 速度档，FAST=10ms/°，MID=15ms/°。
    //   手臂取 20°/步 @FAST = 200ms/步：起身 + 弹 8 次 + 收尾 ≈ 3.0s，与 GIF 的
    //   5s 量级对得上；头取 12°/步 @MID = 180ms/步，比手臂慢半拍、对不齐，
    //   反而像"余震"而不是"第二个人在摇头"。
    //   四条变体的差别只在【幅度、起手速度、左右对称性】三处，不另起结构。
    //
    //   ⚠️ 以下角度为【首版设计值】，付了推导但没上过板。烧录后按手感微调即可，
    //      调的时候只动角度、不要动"手臂为主/头为辅"这个分工，改了就不是兴奋了。
    // ══════════════════════════════════════════════════════════════════════════

    // // ── 3_1: 兴奋① 星星眼·最亢奋 ★四条的基准款 ────────────────────────────────
    // //   手臂 VERY_FAST 猛地起身（87°×5 ≈ 435ms，就是"噌"那一下），随后 105↔125
    // //   均匀弹 8 次，最后放慢速度放下（"余兴未消"）——起手快、收手慢，这个不对称
    // //   正是"活的"和"机械的"分界（同傲娇③末尾 overshoot 的道理）。
    // {
    //     .emotion_id = EMO_EXCITED1,
    //     .screen_anim = "兴奋3_1.gif",
    //     .gif_path = "S:/gif/3_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "laugh_short.mp3",
    //     .head = {
    //         .target = 96.0f,          // 只偏 6°：是"表态"，不是"摇头"
    //         .speed = SERVO_SPEED_MID, // 6°×15 = 90ms
    //         .seq = {
    //             {96.0f,0,500},
    //             {84.0f}, // 12°×15 = 180ms
    //             {96.0f},
    //             {84.0f},
    //             {96.0f},
    //             {84.0f},
    //             {96.0f},
    //             {90.0f, SERVO_SPEED_FAST}, // 先收回中位，把收尾让给手臂单独演
    //         },
    //         .seq_len = 8,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 120.0f,
    //         .speed = SERVO_SPEED_VERY_FAST, // ★87°×5 ≈ 435ms：起身必须"噌"地一下
    //         .seq = {
    //             {105.0f, SERVO_SPEED_FAST}, // 15°×10 = 150ms
    //             {125.0f},                   // 20°×10 = 200ms 弹起
    //             {105.0f},
    //             {125.0f},
    //             {105.0f},
    //             {125.0f},
    //             {105.0f},
    //             {125.0f},
    //             {105.0f},
    //             {33.0f, SERVO_SPEED_FAST}, // 92°×10 = 920ms 慢慢放下
    //         },
    //         .seq_len = 10,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    //     .right_arm = {
    //         .target = 116.0f, // 与左臂刻意差 4°：完全对称会显得很机械（同傲娇③手法）
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {101.0f, SERVO_SPEED_FAST},
    //             {121.0f},
    //             {101.0f},
    //             {121.0f},
    //             {101.0f},
    //             {121.0f},
    //             {101.0f},
    //             {121.0f},
    //             {101.0f},
    //             {33.0f, SERVO_SPEED_FAST},
    //         },
    //         .seq_len = 10,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    // },

    // // ── 3_2: 兴奋② 圆眼笑·憨 ─────────────────────────────────────────────────
    // //   与①的差别只在【起手速度】：这条不"噌"，是用 FAST 一点点抬上来（1.2s），
    // //   抬到位后再弹。慢起手 = 憨；那条快起手 = 机灵。幅度反而比①大 6°，
    // //   "心大、动作开"是憨的另一半。
    // {
    //     .emotion_id = EMO_EXCITED2,
    //     .screen_anim = "兴奋3_2.gif",
    //     .gif_path = "S:/gif/3_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "laugh_short.mp3",
    //     .head = {
    //         .target = 98.0f,
    //         .speed = SERVO_SPEED_MID,
    //         .seq = {
    //             {82.0f}, // 16°×15 = 240ms：头跟着晃得比①多一点
    //             {98.0f},
    //             {82.0f},
    //             {98.0f},
    //             {90.0f, SERVO_SPEED_FAST},
    //         },
    //         .seq_len = 5,
    //         .seq_speed = SERVO_SPEED_MID,
    //     },
    //     .left_arm = {
    //         .target = 124.0f,
    //         .speed = SERVO_SPEED_FAST, // ★不"噌"，91°×10 ≈ 910ms 一点点抬上来
    //         .seq = {
    //             {104.0f, SERVO_SPEED_FAST}, // 20°×10 = 200ms
    //             {130.0f},                   // 26°×10 = 260ms 甩得开
    //             {104.0f},
    //             {130.0f},
    //             {104.0f},
    //             {130.0f},
    //             {33.0f, SERVO_SPEED_MID}, // 97°×15 ≈ 1455ms 憨憨地放下
    //         },
    //         .seq_len = 7,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    //     .right_arm = {
    //         .target = 120.0f,
    //         .speed = SERVO_SPEED_FAST,
    //         .seq = {
    //             {100.0f, SERVO_SPEED_FAST},
    //             {126.0f},
    //             {100.0f},
    //             {126.0f},
    //             {100.0f},
    //             {126.0f},
    //             {33.0f, SERVO_SPEED_MID},
    //         },
    //         .seq_len = 7,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    // },

    // // ── 3_3: 兴奋③ 眨眼笑·俏皮 ★唯一一条【双臂反相】的 ────────────────────────
    // //   俏皮的本质是【不对齐】：两臂同拍弹跳是"蹦"（①②④都是这个），这里让左右臂
    // //   【同拍反相】——左臂在前时右臂在后、左臂在后时右臂在前。这个节奏读起来是
    // //   【欢呼时挥舞手臂】，正是人类表达兴奋最原型的姿势；四条里只有它这么走。
    // //
    // //   ⚠️ 首版曾让右臂弹两下后 hold 1.1s "举着乐"，观感是【卡住】而不是【乐】：
    // //      舵机保持某个角度时是彻底静止的，没有微动去维持"活着"的感觉，静止超过
    // //      约 0.5s 就会被看成故障而非表演。【通则：高唤醒情绪里不要出现 >0.5s 的
    // //      静止段】——想要"持续感"，要么继续走位，要么换另一根轴接着动。
    // //   ⚠️ 幅度必须守住 20°（103↔123）：再放大就变成"走路/竞走"，不是欢呼了。
    // {
    //     .emotion_id = EMO_EXCITED3,
    //     .screen_anim = "兴奋3_3.gif",
    //     .gif_path = "S:/gif/3_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "laugh_short.mp3",
    //     .head = {
    //         .target = 90.0f, // 先歪向一侧（俏皮的"歪头"）
    //         .speed = SERVO_SPEED_FAST,
    //         .seq = {
    //             {84.0f, 0, 600},
    //             {96.0f},
    //             {86.0f}, // 回来但不回满，留一点"歪"
    //             {96.0f},
    //             {86.0f},
    //             {92.0f, SERVO_SPEED_MID},
    //         },
    //         .seq_len = 5,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    //     .left_arm = {
    //         .target = 103.0f,               // 起点先在"后"位——此刻右臂正在"前"位
    //         .speed = SERVO_SPEED_VERY_FAST, // ★70°×5 = 350ms 猛地弹起
    //         .seq = {
    //             {123.0f, SERVO_SPEED_FAST}, // 20°×10 = 200ms 前
    //             {103.0f},                   // 200ms 后（右臂此刻正好在前）
    //             {123.0f},
    //             {103.0f},
    //             {123.0f},
    //             {103.0f},
    //             {123.0f},
    //             {103.0f},
    //             {123.0f},
    //             {33.0f, SERVO_SPEED_FAST}, // 90°×10 = 900ms 放下
    //         },
    //         .seq_len = 10,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    //     .right_arm = {
    //         .target = 123.0f,               // ★与左臂【同拍反相】：它前我后、它后我前
    //         .speed = SERVO_SPEED_VERY_FAST, // 90°×5 = 450ms
    //         .seq = {
    //             {103.0f, SERVO_SPEED_FAST}, // 200ms 后（左臂此刻在前）
    //             {123.0f},                   // 200ms 前
    //             {103.0f},
    //             {123.0f},
    //             {103.0f},
    //             {123.0f},
    //             {103.0f},
    //             {123.0f},
    //             {103.0f},
    //             {33.0f, SERVO_SPEED_FAST}, // 90°×10 = 900ms
    //         },
    //         .seq_len = 10,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    // },

    // ── 3_4: 兴奋④ 星星眼────────────────────────────────────────
    // {
    //     .emotion_id = EMO_EXCITED4,
    //     .screen_anim = "兴奋3_4.gif",
    //     .gif_path = "S:/gif/3_4.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = "laugh_short.mp3",
    //     .head = {
    //         .target = 90.0f,
    //         .speed = SERVO_SPEED_FAST, // 6°×10 = 60ms：头先动、且比①快
    //         .seq = {
    //             {90.0f, 0, 500},
    //             {84.0f},
    //             {96.0f}, // 12°×10 = 120ms
    //             {84.0f},
    //             {96.0f},
    //             {84.0f},
    //             {96.0f},
    //             {84.0f},
    //             {96.0f},
    //             {90.0f, SERVO_SPEED_MID}, // 慢收，把余震演出来
    //         },
    //         .seq_len = 10,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    //     .left_arm = {
    //         .target = 116.0f,
    //         .speed = SERVO_SPEED_VERY_FAST, // 83°×5 ≈ 415ms
    //         .seq = {
    //             {101.0f, SERVO_SPEED_FAST}, // 15°×10 = 150ms
    //             {121.0f},                   // 20°×10 = 200ms
    //             {101.0f},
    //             {121.0f},
    //             {101.0f},
    //             {33.0f, SERVO_SPEED_FAST}, // 88°×10 = 880ms
    //         },
    //         .seq_len = 8,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    //     .right_arm = {
    //         .target = 112.0f, // 与左臂差 4°
    //         .speed = SERVO_SPEED_VERY_FAST,
    //         .seq = {
    //             {117.0f, SERVO_SPEED_SLOW}, // 抢到前位（此刻左臂在后位 101）
    //             {97.0f},                    // 20°×10 = 200ms 落到后位（左臂此刻 121 在前）
    //             {117.0f},
    //             {97.0f},
    //             {117.0f},
    //             {33.0f, SERVO_SPEED_FAST}, // 84°×10 = 840ms 与左臂同拍落地
    //         },
    //         .seq_len = 8,
    //         .seq_speed = SERVO_SPEED_FAST,
    //     },
    // },

    // /* ══════════════════════════════════════════════════════════════════════════
    /* ── 4_x：好奇（3 张）───────────────────────────────────────────────────────
     *
     * ★2026-09-21 整组重做。旧版（源文档《21 好奇歪头》）的"歪头"前提已被推翻，
     *   逐帧看素材后按第一性原理重新推导，结论如下：
     *
     * 【推翻的旧前提：好奇 = 歪头】
     *   三张素材的脸【朝向恒定】，画面里零倾斜、零转向——好奇全靠"眼睛大小 +
     *   眉毛 + 冒问号"表达，压根没有歪头。而本机芯头部只有【偏航一轴】
     *   (10~170°，中位 90°)，横滚物理上不存在，"歪头"表达不了。
     *   旧版把"歪头"翻译成左右扫视(70↔120 来回 50°)，在只有偏航轴的机芯上
     *   读出来是【摇头 = 否定】，语义正好反过来——这就是"别扭"的根。
     *
     * 【重推导出的新原则】身体只做「定向 → 停留 → 回神」三件事：
     *   ① 定向：头朝一个【具体方向】转过去（不是来回扫）
     *   ② 停留：在该方向【保持住】，时长 = 注意时长 ← "在思考"的唯一载体
     *   ③ 回神：解除定向，由 worker 归中
     *   硬性排除项：绝不出现【连续来回摆动】——那读作摇头。
     *   停顿是把"打量"与"摇头"区分开的【唯一手段】。
     *
     * 【三张素材的实测节拍（逐帧数出，42ms/帧）】
     *   4_1「好奇」   6.0s/144帧：眼睛大小交替 + 眉毛挑压 + "?"号闪现 3 次
     *                             (约 1.1s / 3.1s / 5.0s)，且 "?" 出现在画面左侧
     *   4_2「好奇2」  5.0s/120帧：纯【眯眼呼吸】，无 "?"，节拍约 1.7s
     *   4_3「好奇 2」 5.0s/120帧：纯【眨眼】，无 "?"，节拍约 0.9s
     *   ⇒ 三张素材内容明显不同，身体动作也必须各给一种"性格"，
     *     不能再复刻同一套（旧版三行逐字相同是错的）。
     *
     * 【三组各自的性格（本次设计的核心区分）】
     *   4_1 长停打量：头偏左 24°，四拍、长停(700~1300ms)，逐拍对齐 3 个 "?"；
     *                 左臂【抬到半空停住】= 人类"等一下，你说什么"的经典手势。
     *   4_2 侧耳慢呼吸：头只定向【一次后彻底冻结】，节奏全交给两臂，
     *                 两臂在 t≈2.3s 【反相】一抬一落，对上素材的呼吸节拍。
     *   4_3 凑近端详：头【只朝右、绝不回头】(30°→12°→8° 递减)，
     *                 越看越凑近然后定住；两臂【完全同步】小幅前倾
     *                 （素材是"眨眼"，两侧同时，故同步是对的）。
     *
     * ⚠️【手臂基线不是 90°，是 15°】ARM_CENTER_DEG(bsp_config.h:332) = 15.0f，
     *   手臂静止/归位在 15°，不是中位 90°。手臂角度【越大越向前抬】
     *   (L/R_ARM_MAX_ANGLE=170 向前极限，注释写"防止撞头")。
     *   所以"抬起"= 往大于 15 的方向走；旧版把手臂摆在 30~45 附近当"中位"用是错的。
     *   另：左臂硬件反装已在 bsp_servo.c 的 servo_write_angle/servo_read_angle
     *   收口（逻辑角 = 物理角镜像），故本表左右臂【填相同角度即物理对称】。
     *
     * ⚠️【"保持不动"必须用带 hold_ms 的 seq 步，不能用 seq_len=0】
     *   servo_manager.c 在目标点之后【总会】追加归中点，seq_len=0 的手臂会被
     *   立刻拉回去，根本停不住。要让某轴停留在一个姿势上，必须给一个
     *   {角度, 0, 大hold} 的 seq 步（本组 4_1 右臂、4_2 头都靠这招）。
     *
     * ⚠️【头角方向】头 大角=左 小角=右（interaction.c:12 的既有约定）。
     *   物理上"屏幕左"对应头往哪边，上板后需用一次 90→130 单步确认。 */
    // {
    //     // ── 4_1: 好奇①「好奇」── 主语：头（长停打量）
    //     //    节拍对齐：走到位停 700ms → 落点 t≈1.06s，正好卡住素材第 1 个 "?"
    //     //      ① 90→114 (偏左24°)  360ms + 停 700 → t 1.06s   [卡 ? @1.1s]
    //     //      ② 114→104 (回撤10°) 150ms + 停 900 → t 2.11s
    //     //      ③ 104→122 (再偏18°) 270ms + 停1300 → t 3.68s   [卡 ? @3.1s]
    //     //      ④ 122→112 (轻颔10°) 150ms + 停1300 → t 5.13s   [卡 ? @5.0s]
    //     //      归中 112→90 330ms → t≈5.46s（素材 6.0s）
    //     .emotion_id = EMO_CURIOUS1,
    //     .screen_anim = "好奇.gif",
    //     .gif_path = "S:/gif/4_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：四拍打量，方向一直偏左、两次小幅回撤（有停顿 ⇒ 读作"打量"而非"摇头"）
    //     .head = {.target = 114.0f,
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {114.0f, 0, 700},  // 停在偏左 24° 打量，卡第 1 个 "?"
    //                  {104.0f, 0, 900},  // 回撤 10°，像"再想一下"
    //                  {122.0f, 0, 1300}, // 再偏过去 18°，卡第 2 个 "?"
    //                  {112.0f, 0, 1300}, // 轻颔 10°，卡第 3 个 "?"
    //              },
    //              .seq_len = 4,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：抬到半空【停住】—— "等一下，你说什么"的手势。
    //     //   (15° 静止位 → 96° 抬 81°，FAST=10ms/° → 810ms)
    //     //   到 3.61s 时轻放 18° 到 78°，再停到 4.59s 归中
    //     .left_arm = {.target = 96.0f,
    //                  .speed = SERVO_SPEED_FAST,
    //                  .seq = {
    //                      {96.0f, 0, 2800}, // ★停住：这一句就是"举手"这个手势本身
    //                      {78.0f, 0, 800},  // 轻轻放下一点
    //                  },
    //                  .seq_len = 2,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：全程基本不动，只小幅抬 29° 后【同一姿势停到底】，
    //     //       与左臂形成"一前一后"的不对称，且全程无振荡。
    //     //       ★seq 只有一个 {44,0,4600}：靠长 hold 把姿势钉住（不能用 seq_len=0）
    //     .right_arm = {.target = 44.0f,
    //                   .speed = SERVO_SPEED_MID,
    //                   .seq = {
    //                       {44.0f, 0, 4600}, // ★停住到左臂也走完
    //                   },
    //                   .seq_len = 1,
    //                   .seq_speed = SERVO_SPEED_MID},
    // },
    // {
    //     // ── 4_2: 好奇②「好奇2」── 主语：两臂（侧耳慢呼吸），头一次定向后冻结
    //     //    素材是【纯慢呼吸、无脉冲】，所以身体也必须"慢、无节拍"：
    //     //    头转过去就【不动了】（一个 {110,0,4600} 的长 hold），
    //     //    节奏全交给两臂——t≈2.3s 时【反相】一抬一落，对上素材约 1.7s 的呼吸。
    //     //    两臂时间线：都是 到终点 + hold → t=2.30s 反相换位 → 再停住
    //     //      左臂 15→88 (MID 1095ms) + hold 1200 = 2.30s → 落到 56°(MID 480ms) + 停 1500
    //     //      右臂 15→40 (MID  375ms) + hold 1925 = 2.30s → 抬到 80°(MID 600ms) + 停 1500
    //     .emotion_id = EMO_CURIOUS2,
    //     .screen_anim = "好奇2.gif",
    //     .gif_path = "S:/gif/4_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：只定向一次（偏左 20°）然后【全程冻结】，绝不来回
    //     .head = {.target = 110.0f,
    //              .speed = SERVO_SPEED_FAST, // 20°×10 = 200ms 转过去
    //              .seq = {
    //                  {110.0f, 0, 4600}, // ★停住：这 4.6s 的"不动"才是"侧耳听"的表情
    //              },
    //              .seq_len = 1,
    //              .seq_speed = SERVO_SPEED_FAST},
    //     // 左臂：先抬起停在半空，到呼吸换气点再【反相落下】
    //     .left_arm = {.target = 88.0f,
    //                  .speed = SERVO_SPEED_MID,
    //                  .seq = {
    //                      {88.0f, 0, 1200},               // 停到换气点 t≈2.30s
    //                      {56.0f, SERVO_SPEED_MID, 1500}, // ★反相落下 32°（慢），再停住
    //                  },
    //                  .seq_len = 2,
    //                  .seq_speed = SERVO_SPEED_MID},
    //     // 右臂：与左臂【反相】——先在低位等，换气点才抬起，
    //     //       两臂同刻反向 ⇒ 观感是"身体在轻轻起伏"，即成"呼吸"
    //     .right_arm = {.target = 40.0f,
    //                   .speed = SERVO_SPEED_MID,
    //                   .seq = {
    //                       {40.0f, 0, 1925},                // 低位等左臂（MID 375ms + 停 1925 = 2.30s）
    //                       {80.0f, SERVO_SPEED_MID, 1500},  // ★反相抬起 40°（慢），再停住
    //                   },
    //                   .seq_len = 2,
    //                   .seq_speed = SERVO_SPEED_MID},
    // },
    // {
    //     // ── 4_3: 好奇③「好奇 2」── 主语：头（凑近端详）+ 两臂同步前倾
    //     //    与 4_1 必须区分开，故刻意做三点不同：
    //     //      ① 方向相反：一直偏【右】，且【绝不回头】（不出现左右交替 ⇒ 绝不会读成摇头）
    //     //      ② 幅度递减：30° → 12° → 8°，"越看越凑近"然后定住
    //     //      ③ 两臂【完全同步】小幅前倾（素材是"眨眼"，两侧同时，同步才对）
    //     //    时间线：60°(450ms)+停1000 → 48°(180ms)+停1200 → 56°(120ms)+停1500
    //     //            → 归中 510ms，合计 ≈4.96s（素材 5.0s）
    //     .emotion_id = EMO_CURIOUS3,
    //     .screen_anim = "好奇 2.gif",
    //     .gif_path = "S:/gif/4_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：只朝右、绝不回头。大角=左，故偏右取小角。
    //     .head = {.target = 60.0f,
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {60.0f, 0, 1000}, // 先定住看清（偏右 30°）
    //                  {48.0f, 0, 1200}, // 再凑近 12°，停更久
    //                  {56.0f, 0, 1500}, // 略回 8°，长静默收尾
    //              },
    //              .seq_len = 3,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 两臂【同步】小幅前倾抬 49°（15→64, FAST=10ms/° → 490ms），
    //     // 到 4.39s 一起轻放回 48° 后归中。左右填同值 ⇒ 物理对称（左臂镜像已收口）
    //     .left_arm = {.target = 64.0f,
    //                  .speed = SERVO_SPEED_FAST,
    //                  .seq = {
    //                      {64.0f, 0, 3900}, // ★停住：与右臂完全同拍
    //                      {48.0f},
    //                  },
    //                  .seq_len = 2,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     .right_arm = {.target = 64.0f,
    //                   .speed = SERVO_SPEED_FAST,
    //                   .seq = {
    //                       {64.0f, 0, 3900}, // ★与左臂逐字相同 = 刻意同步
    //                       {48.0f},
    //                   },
    //                   .seq_len = 2,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },

    /* ── 5_x：委屈（2 张）───────────────────────────────────────────────────────
     *
     * ★2026-09-21 整组重做。同 4_x，先逐帧拆素材再反推身体，不抄源文档。
     *
     * 【素材实测（逐帧 + 差异图）】
     *   5_1「委屈」 5.5s/132帧：**含泪 + 眼神躲闪**。眼角有静态泪光符号；
     *       眼睛（含瞳孔）全程持续小幅无规律游移（=眼神飘、不敢直视），
     *       前 ~4.5s 幅度很小，**t≈4.55~5.05s 有一次明显加大的晃动**，然后归位。
     *       全程【无抽泣】、脸朝向不变。
     *   5_2「委屈2」6.0s/144帧：**抽泣 + 掉泪**。动画周期实为 3.0s（内容重复两遍），
     *       一个周期内约 3 次泪珠滚落（t≈0.79/1.88/2.54s，间隔约 1.1s 和 0.66s）；
     *       眼睛和嘴持续抽动，脸朝向不变。
     *   ⇒ 两张素材【语义完全不同】：一个是"含泪不敢看你"，一个是"抽泣掉泪"，
     *     不能像旧版那样共用同一套。
     *
     * 【旧版的两处硬 bug（本次修掉）】
     *   ① 5_1 两臂都填 18°，但手臂基线是 **15°**(ARM_CENTER_DEG)——实际只位移 3°，
     *      远低于死区+齿隙，上板【完全看不见】（同 BUG-033 的成因）。
     *   ② 5_2 的"抽泣"写成 105↔98 = **7°**，同样看不见 ⇒ 旧版根本抽不起来。
     *   两处都是"幅度没到可见阈值"的同一类错误，不是手感问题。
     *
     * 【设计原则（沿用 4_x 验证过的那套）】
     *   委屈的身体语言 = **收缩 / 回避**，不是来回摆：
     *     5_1 主语是头：**偏开一侧躲住不动**（不敢直视），到素材末尾那次大晃动时
     *         全身一起【瑟缩一下】再停住。两臂慢慢抬到身前错开停住 = "无措地绞手"。
     *     5_2 主语是两臂：**双手抬到脸前**（捂脸）后做出**抽泣**——
     *         每拍 = 快速回落 20° + 短停 + 抽回来 + 短停，共 4 拍。
     *         这是全库唯一【故意用重复动作】的一组：素材"抽泣"的语义本身就是重复，
     *         靠"快抽 + 明显停顿"和"匀速挥手"区分开。
     *   ⇒ 两组的差异维度：主语（头 vs 两臂）、头方向（偏右躲 vs 偏左别开）、
     *     手臂高度（88° 身前 vs 140° 到脸前）、有无重复（无 vs 4 拍抽泣）。
     *
     * ⚠️ 同 4_x 的三条硬件事实继续适用：手臂基线 15°（角度越大越前抬）、
     *   要停住必须用长 hold（seq_len=0 会被立刻归中）、三轴各自归中。
     * ⚠️ 手臂 140° 仍在安全区内：向前极限 L/R_ARM_MAX_ANGLE=170（注释写"防止撞头"）。 */
    // {
    //     // ── 5_1: 委屈①「委屈」── 主语：头（含泪躲闪 + 末尾一次瑟缩）
    //     //    时间线（三轴的第二拍刻意【都压在 t≈4.53s】= 素材那次大晃动）：
    //     //      头     90→68(偏右22°) 330ms + 停4200 → t 4.53 → 58° + 停400 → 归中 → 5.56s
    //     //      左臂   15→88(抬73°)  1095ms + 停3430 → t 4.53 → 72° + 停500 → 归中 → 5.84s
    //     //      右臂   15→66(抬51°)   765ms + 停3765 → t 4.53 → 54° + 停500 → 归中 → 5.60s
    //     .emotion_id = EMO_GRIEVED1,
    //     .screen_anim = "委屈.gif",
    //     .gif_path = "S:/gif/5_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：偏右躲开【并停住】——"不敢直视"。绝不来回 ⇒ 不会读成摇头。
    //     //     到素材末尾那次大晃动时再躲 10°，是全身瑟缩的一部分。
    //     .head = {.target = 68.0f, // 偏右 22°（大角=左，故偏右取小角）
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {68.0f, 0, 2200}, // ★停住：这 4.2s 的"偏着头不动"就是"躲"
    //                  {58.0f, 0, 400},  // ★瑟缩：再躲开 10°，顿住
    //              },
    //              .seq_len = 2,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：慢慢抬到身前【停住】（无措地绞手），末尾随瑟缩再收 16°
    //     .left_arm = {.target = 88.0f, // 抬 73°
    //                  .speed = SERVO_SPEED_MID,
    //                  .seq = {
    //                      {88.0f, 0, 2430}, // ★停住（到位 1.10s + 停 3.43s = t 4.53s）
    //                      {72.0f, 0, 500},  // ★随瑟缩收 16°
    //                  },
    //                  .seq_len = 2,
    //                  .seq_speed = SERVO_SPEED_MID},
    //     // 右臂：与左臂【错开竖直高度】（66° vs 88°）+ 到位时刻也不同，
    //     //       刻意不对称 ⇒ 像真的"手足无措"，而非操练式齐动
    //     .right_arm = {.target = 66.0f, // 抬 51°，比左臂低 22°
    //                   .speed = SERVO_SPEED_MID,
    //                   .seq = {
    //                       {66.0f, 0, 2765}, // ★停住（到位 0.77s + 停 3.77s = t 4.53s）
    //                       {54.0f, 0, 500},  // ★随瑟缩收 12°
    //                   },
    //                   .seq_len = 2,
    //                   .seq_speed = SERVO_SPEED_MID},
    // },
    // {
    //     // ── 5_2: 委屈②「委屈2」── 主语：两臂（双手抬到脸前 + 抽泣）
    //     //    全库唯一【故意用重复动作】的一组：素材的"抽泣"本身就是一抽一抽，
    //     //    靠"快抽 20° + 明显停顿"把抽泣与"匀速挥手"区分开。
    //     //    幅度取 20°（旧版 7° 在齿隙里，是看不见的，这是本次要修的核心）。
    //     //    4 拍抽泣，每拍 0.8s（抽 200ms + 停 150 + 回 200 + 停 250）。
    //     //    两臂【稳定错开 0.54s】——右臂首拍前多停 550ms，之后节奏完全相同，
    //     //    于是四拍的谷值依次落在 1.62/2.42/3.22/4.02s，对左臂的
    //     //    1.08/1.88/2.68/3.48s 恒定滞后 0.54s。不齐，也不同时收尾（就是要这效果）。
    //     //    时间线（左臂）：15→140(抬125°) 625ms + 停250 → 抽120° → 回140° ×4
    //     //                    → 定600 → 120° → 归中 → 5.68s
    //     //    时间线（右臂）：同节奏、整体滞后 0.54s ⇒ 归中 → 6.00s
    //     .emotion_id = EMO_GRIEVED2,
    //     .screen_anim = "委屈2.gif",
    //     .gif_path = "S:/gif/5_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：素材脸朝向不变 ⇒ 头【先定住不动】（target 就是中位 90°，只靠 hold
    //     //     把它钉住 3.8s），直到抽泣快收尾时才轻轻【别开】14° —— 抽泣的人会
    //     //     别过脸去。方向取【左】，与 5_1 的"偏右躲"相反，两组才区分得开。
    //     .head = {.target = 90.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                             {90.0f, 0, 3800}, // ★停住：起点即中位，钉住 3.8s 不动
    //                                                             {104.0f, 0, 900}  // ★别开脸 14°（偏左），停到收尾
    //                                                         },
    //              .seq_len = 2,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：抬到脸前捂脸(140°)，然后 4 拍抽泣（140↔120，每拍 20° 位移，
    //     //       用 FAST=10ms/° → 单程 200ms，够快才像"抽"）
    //     .left_arm = {.target = 140.0f,               // 抬 125°，到脸前
    //                  .speed = SERVO_SPEED_VERY_FAST, // 5ms/° → 625ms 快速捂住
    //                  .seq = {
    //                      {140.0f, 0, 250}, // 捂住，停一拍
    //                      {120.0f, 0, 150}, // ★抽泣第 1 抽（回落 20° + 停）
    //                      {140.0f, 0, 250}, // 抽回来
    //                      {120.0f, 0, 150}, // ★第 2 抽
    //                      {140.0f, 0, 250},
    //                      {120.0f, 0, 150}, // ★第 3 抽
    //                      {140.0f, 0, 250},
    //                      {120.0f, 0, 150}, // ★第 4 抽
    //                      {140.0f, 0, 600}, // 抽完定住
    //                      {120.0f},         // 开始放下
    //                  },
    //                  .seq_len = 10,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：与左臂【同节奏但首拍前多停 550ms】⇒ 全程恒定滞后 0.54s。
    //     //       两臂错开是刻意的——真实抽泣时两肩不齐，齐动像机械。
    //     //       ⚠️ 该 550ms 偏移量 = 800 - 250，改任一 hold 都会连带改错位相位。
    //     .right_arm = {.target = 138.0f, // 抬 123°，比左臂低 2°
    //                   .speed = SERVO_SPEED_VERY_FAST,
    //                   .seq = {
    //                       {138.0f, 0, 800}, // ★首拍前多停 550ms（=整体滞后半拍）
    //                       {118.0f, 0, 250}, // ★抽泣第 1 抽
    //                       {138.0f, 0, 150},
    //                       {118.0f, 0, 250}, // ★第 2 抽
    //                       {138.0f, 0, 150},
    //                       {118.0f, 0, 250}, // ★第 3 抽
    //                       {138.0f, 0, 150},
    //                       {118.0f, 0, 250}, // ★第 4 抽
    //                       {138.0f, 0, 300}, // 抽完定住（比左臂的 600 短，收尾仍差半拍）
    //                       {118.0f},
    //                   },
    //                   .seq_len = 10,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },

    /* ── 6_x：害羞（2 张）───────────────────────────────────────────────────────
     *
     * ★2026-09-21 整组重做。同 4_x/5_x，先逐帧拆素材再反推身体，不抄源文档。
     *
     * 【素材实测（逐帧 + 真实帧时长，不是帧序号）】
     *   6_1「害羞」 5.0s：灰色牌/双手从下方【举起来遮住脸】，举到位后【完全静止 0.92s】，
     *       再放下、歇 0.5s，然后【一模一样再走一遍】。两轮的逐帧差异曲线逐值相同
     *       ⇒ 真实节拍 = 抬起 0～0.58s │ 停住 0.58～1.50s │ 放下 1.50～2.00s │ 歇 2.00～2.50s。
     *       同时眼睛从"^^ 笑眼"变成瞪圆、脸颊爆红 ⇒ 语义 = 捂脸两下：对称、有长停顿。
     *   6_2「害羞蹭蹭」 5.0s：⚠️【画面几何完全不动】（眼睛质心全程只漂约 2px），
     *       唯一的动静是整张脸的颜色以【严格 0.5s 周期闪烁】共 10 次（灰→亮→暗红→中灰）。
     *       颜色闪烁无法用舵机复现 ⇒ 本组必须【自己发明动作】，依据只有名字里的"蹭"。
     *   ⇒ 两张素材毫无共通点：一个是"举起来停住"，一个是"完全没有位移"。
     *
     * 【旧版的问题】
     *   ① 6_2 逐字复制 6_1（注释自己写着"复用 6_1"）⇒ 把两张素材抹成同一个动作。
     *   ② 6_1 左臂填 30°/25°，减掉基线 15° 后只剩 15°/10° 位移；右臂 90→100→85→75
     *      的摆幅也只有 ±10~15° —— 全在死区+齿隙边缘（同 BUG-033 的成因）。
     *   ③ 头 115→110 只有 5° ⇒ 肉眼看不见。
     *   ④ "头 115 + 右臂 90"的不对称是照抄文档的，而素材 6_1 其实是【完全对称】的。
     *
     * 【设计原则（沿用 4_x/5_x 验证过的那套）】
     *   6_1 主语 = 【两臂】：素材本身就是【两次】"举起—停住—放下"，照做。
     *       两臂【完全对称】同时抬到脸前 132°，每次停住 0.92s；
     *       头全程只在 90↔112 之间【偏开一侧再回正】两次，且整条头时间线
     *       【滞后两臂 150ms】—— 手先捂上、脸才别过去，才像真的。
     *   6_2 主语 = 【头】：把"蹭"翻译成【偏置的小幅快速摩擦】——
     *       头先偏左躲到 108°（不回来），然后【只在 96°~108° 之间】来回蹭 10 个来回
     *       （每半拍 250ms，正好对上素材 0.5s 的颜色周期）。
     *       ⚠️ 这是全库【第二组故意用重复动作】的（第一组是 5_2 的抽泣），
     *          "蹭"的语义本身就是反复摩擦。它与"摇头=否定"靠三点拉开距离：
     *           ① 幅度小（12°，摇头是 40°+）  ② 节奏快而均匀（1Hz，摇头是 2~3 下）
     *           ③ 【整体偏置在中位左侧，全程不跨越 90°】
     *       两臂在这一组【故意慢】：三级台阶式慢慢往上缩肩膀（52→68→84），
     *       与头的快蹭形成两层节奏，避免全身同一个频率。
     *   ⇒ 两组的差异维度：主语（两臂 vs 头）、重复次数（2 次 vs 10 次）、
     *     手臂高度（132° 遮脸 vs 84° 缩肩）、头部（偏开又回正 vs 偏开后一直在一侧蹭）。
     *
     * ⚠️ 同 4_x/5_x 的三条硬件事实继续适用：手臂基线 15°（角度越大越前抬）、
     *   要停住必须用长 hold（seq_len=0 会被立刻归中）、三轴各自归中。 */

    // {
    //     // ── 6_1: 害羞①「害羞」── 主语：两臂（对称，两次"举起捂脸—停住—放下"）
    //     //    素材节拍：抬起0.58s │ 停0.92s │ 放下0.50s │ 歇0.50s，×2 = 5.0s
    //     //    臂 15→132 抬 117°，VERY_FAST=5ms/° ⇒ 单程 585ms（够猛才像"猛地捂住"）
    //     //    时间线：① t0→585 抬起 + 停920 → t1505
    //     //            ② t1505→2090 放下 + 歇500 → t2590
    //     //            ③ t2590→3175 再抬 + 停920 → t4095
    //     //            ④ t4095→4680 放下 → 归中
    //     .emotion_id = EMO_SHY1,
    //     .screen_anim = "害羞.gif",
    //     .gif_path = "S:/gif/6_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：90↔112 偏开左侧 22° 两次。MID=15ms/° ⇒ 单程 330ms。
    //     //     整条时间线比两臂【晚 150ms】（seq[0] 先空停 150ms）：
    //     //     两臂 t0 起抬、头 t170 才动；两臂 t1505 起落、头 t1675 才回。
    //     //     ⚠️ .target 填 90（= 当前中位）只是占位，实际位移全部由 seq 完成。
    //     .head = {.target = 90.0f,
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {90.0f, 0, 150},                 // ★滞后：先让两臂动起来
    //                  {112.0f, SERVO_SPEED_MID, 1175}, // 偏开330 + 停到 t1675（对齐"捂着"那一段）
    //                  {90.0f, SERVO_SPEED_MID, 755},   // 回正330 + 停到 t2760（对齐"歇"那一段）
    //                  {112.0f, SERVO_SPEED_MID, 1175}, // 第二次偏开
    //                  {90.0f, SERVO_SPEED_MID, 85},    // 回正，与两臂同刻收尾（t4680）
    //              },
    //              .seq_len = 5,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 两臂：左右填同值 ⇒ 物理对称（左臂镜像已在 BSP 层收口）。
    //     //       ⚠️ 不许改成不对称：素材 6_1 的牌子是整体抬起的，对称才对。
    //     .left_arm = {.target = 132.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        {132.0f, 0, 920},                     // ① 捂住脸，停住（素材那 0.92s 静止）
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 500},  // ② 放下585 + 歇500
    //                                                                        {132.0f, SERVO_SPEED_VERY_FAST, 920}, // ③ 再捂一次，停住
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 0},    // ④ 放下
    //                                                                    },
    //                  .seq_len = 4,
    //                  .seq_speed = SERVO_SPEED_VERY_FAST},
    //     .right_arm = {.target = 132.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                         {132.0f, 0, 920}, // ★与左臂逐字相同 = 刻意同步
    //                                                                         {15.0f, SERVO_SPEED_VERY_FAST, 500},
    //                                                                         {132.0f, SERVO_SPEED_VERY_FAST, 920},
    //                                                                         {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                     },
    //                   .seq_len = 4,
    //                   .seq_speed = SERVO_SPEED_VERY_FAST},
    // },

    // {
    //     // ── 6_2: 害羞②「害羞蹭蹭」── 主语：头（偏置的快速「蹭」）+ 两臂三级慢缩肩
    //     //    头: t0→270 偏左躲到 108°（MID 18°）；之后在 96↔108 之间来回蹭，
    //     //         每半拍 = 12°@FAST(120ms) + 停130 = 250ms，共 20 半拍 = 10 个来回 = 5000ms
    //     //         → t5270，最后归中 270ms。
    //     //    臂（故意慢，三级台阶，两臂同刻换级、高度错开）：
    //     //         左 15→52(555) 停900 → 68(240) 停900 → 84(240) 停2435 → 总 5270
    //     //         右 15→40(375) 停1080 → 54(210) 停930 → 68(210) 停2465 → 总 5270
    //     .emotion_id = EMO_SHY2,
    //     .screen_anim = "害羞蹭蹭.gif",
    //     .gif_path = "S:/gif/6_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：★核心。全程只在 96°~108°（中位 90° 的【左侧】）活动，绝不跨越中位
    //     //     ⇒ 物理上不可能被读成"摇头"。起点中位 90 → 108 是"先躲开"，之后才是"蹭"。
    //     //     ⚠️ 20 个半拍逐拍显式写死，刻意与素材的 10 次颜色周期一一对应；
    //     //        想改节奏就改这里的 hold，不要去动 seq_speed。
    //     .head = {.target = 108.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                              {96.0f, SERVO_SPEED_FAST, 130},
    //                                                              {108.0f, SERVO_SPEED_FAST, 130},
    //                                                              {96.0f, SERVO_SPEED_FAST, 130},
    //                                                              {108.0f, SERVO_SPEED_FAST, 130},
    //                                                              {96.0f, SERVO_SPEED_FAST, 130},
    //                                                              {108.0f, SERVO_SPEED_FAST, 130},
    //                                                              {96.0f, SERVO_SPEED_FAST, 130},
    //                                                              {108.0f, SERVO_SPEED_FAST, 130},
    //                                                          },
    //              .seq_len = 8,
    //              .seq_speed = SERVO_SPEED_FAST},
    //     // 左臂：每级 16° 位移 @MID=240ms，级间长停 ⇒ 读作"一点点越缩越紧"。
    //     .left_arm = {.target = 52.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                                 {52.0f, 0, 900},               // 第 1 级到位，停
    //                                                                 {68.0f, SERVO_SPEED_MID, 900}, // 第 2 级
    //                                                                 {84.0f, SERVO_SPEED_MID, 435}, // 第 3 级，一直停到与头同刻收尾
    //                                                             },
    //                  .seq_len = 3,
    //                  .seq_speed = SERVO_SPEED_MID},
    //     // 右臂：与左臂同刻换级，但每级高度都低 12~16° ⇒ 不对称（"蹭"本就该歪）。
    //     .right_arm = {.target = 40.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                                  {40.0f, 0, 1080},
    //                                                                  {54.0f, SERVO_SPEED_MID, 930},
    //                                                                  {68.0f, SERVO_SPEED_MID, 465},
    //                                                              },
    //                   .seq_len = 3,
    //                   .seq_speed = SERVO_SPEED_MID},
    // },

    /* ── 7_x：怕痒（4 张）── 源文档《9 憨笑晃动》─────────────────────────────
     * ★2026-09-21 重做。旧版四张【逐字复用同一条】（注释自己写"复用 7_1"），且
     *   手臂值填 60/40/30（基线 ARM_CENTER_DEG=15°，实际只动 45/25/15°）——与
     *   5_x / 6_x 同属"幅度没减基线"那一类毛病。本次按逐帧实测的节拍，四张各一套。
     *
     * 【素材实测：四张都是"高频等周期抖动"，但频率与幅度差得很开】
     *   7_1 怕痒.gif        5.00s  基础周期 420ms  约 11.9 次  幅度小
     *       两态：眉/眼合上又分开，脸部位置始终不动。
     *   7_2 怕痒2.gif       5.00s  基础周期 500ms  约  9.9 次  幅度中
     *       两态：透明底 + 高扬 ^^ 眉  ↔  灰牌 + 低眉。
     *   7_3 怕痒 2.gif      5.33s  基础周期 290ms  约 18.1 次  幅度最小
     *       两态：高弧眼 + 波浪嘴  ↔  平眼 + 平嘴。四张里最碎最快。
     *   7_4 怕痒到扭动.gif  5.33s  基础周期 670ms  约  7.9 次  幅度最大
     *       ★四张里唯一"脸真的在动"的一张。实测四相位：左右眼高差
     *       -5.0 / -0.4 / +4.7 / -0.5 px，双眼间距 97.5 / 97.2 / 92.2 / 97.2 px
     *       ⇒ 整张脸既左右倾（roll）又时而横向压扁（等价于偏航），即"扭动"。
     *
     * ⇒ 结论：7_1 / 7_2 / 7_3 三张【画面几何零位移】（动的只有眉、眼、嘴），
     *   舵机复现不出来，这三条动作只能自己发明；只有 7_4 有真实位移可照做。
     *
     * 【设计：主语不同 + 节奏不同 + 快慢结构不同，四张互不复用】
     *   7_1 主语=两臂，头【全程钉死】。两臂同相"锯齿"抽动：慢抬 29°@FAST(290ms)
     *       + 极快弹回 29°@VERY_FAST(145ms) = 435ms/拍 × 11 拍，对齐素材 420ms。
     *       头一动不动的用意：只有肩膀在一抽一抽、脸僵着忍——这是本条的性格。
     *   7_2 主语=两层节奏。两臂 500ms 对称快摆（15↔38），头【慢一倍】1000ms
     *       才偏开一次（90↔110）。快慢两层叠在一起，对应素材里"眉与灰牌交替"
     *       的那份忙乱。头只在 90~110 之间偏，不跨越中位 90°。
     *   7_3 主语=两臂【反相】。左右臂交替快抽（左起右落），每半拍 25°@FAST(250ms)
     *       + 停 40ms = 290ms，正对素材的 290ms 周期；右臂整条时间线滞后一拍
     *       (290ms) 来制造反相。四张里最快最碎的一张，头仍钉死作对比。
     *   7_4 主语=全身扭。头围绕中位对称摆 90±16°（74↔106）@FAST 连续不停，
     *       32°全幅×10ms = 320ms/半拍 ⇒ 640ms/周期，对齐素材 670ms；两臂
     *       15↔50（35°全幅，四张里幅度最大）与头【反相】拧动。靠"头在摆 +
     *       两臂在拧"读成扭身，而不是摇头；且全程无停顿 = 不庄重，故不会被
     *       读成"否定"。
     *   ⚠️ 算节拍时注意：【半拍行程 = 全幅】，不是半幅。7_4 头若按 ±16° 当
     *      22°×… 去算会差一倍（初版就用 MID 算成 1320ms/周期，是素材的两倍）。
     *
     * 【幅度红线】手臂基线是 ARM_CENTER_DEG = 15°，本次四张臂行程 29°/23°/25°/35°，
     *   全部远超死区+齿隙（约 15°），上板可见。
     * 【速度档】VERY_FAST=5ms/度、FAST=10ms/度、MID=15ms/度（MID 为实测最慢不抖档）。
     * 【停顿】所有"定住"一律写显式 hold_ms，绝不靠重复角度凑（零位移只花 1 帧≈20ms）。
     * ⚠️ 四张均为连续往复摆动，是本组素材的固有语义（怕痒=抖）；7_4 头摆围绕中位
     *   对称，理论上有"摇头"风险，靠 640ms 快速周期 + 两臂反相拧动拉开距离，待实测。
     */
    // {
    //     // ── 7_1: 怕痒①「怕痒」—— 主语=两臂，头钉死；锯齿 435ms × 11 拍
    //     .emotion_id = EMO_TICKLISH1,
    //     .screen_anim = "怕痒.gif",
    //     .gif_path = "S:/gif/7_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：钉在中位全程不动（一个 5075ms 的长 hold），与两臂的频繁抽动形成对比
    //     .head = {.target = 100.0f, .speed = SERVO_SPEED_MID, .seq = {{100.0f, 0, 2575}}, .seq_len = 1, .seq_speed = SERVO_SPEED_MID},
    //     // 两臂：完全对称。抬 29°@FAST=290ms → 弹回 29°@VERY_FAST=145ms，一拍 435ms
    //     .left_arm = {.target = 44.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                  {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                  {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                  {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                  {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                  {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                  {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                  {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  //  {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  //  {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  //  {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  //  {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                  //  {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                              },
    //                  .seq_len = 12,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     .right_arm = {.target = 44.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                   {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                   {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                   {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                   {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                   {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                   {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                   {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   //   {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   //   {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   //   {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   //   {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                                   //   {15.0f, SERVO_SPEED_VERY_FAST, 0}, {44.0f, SERVO_SPEED_FAST, 0},
    //                                                               },
    //                   .seq_len = 12,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 7_2: 怕痒②「怕痒2」—— 两层节奏：两臂 500ms 快摆 + 头 1000ms 慢缩
    //     .emotion_id = EMO_TICKLISH2,
    //     .screen_anim = "怕痒2.gif",
    //     .gif_path = "S:/gif/7_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：慢一倍。20°@MID=300ms + 停 200ms = 500ms/半步，两步 = 1000ms 一个往复 × 5
    //     //     只在 90~110 之间偏（不跨越中位 90°），读作"缩一下"而非摇头
    //     .head = {.target = 110.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                              {110.0f, SERVO_SPEED_MID, 200},
    //                                                              {90.0f, SERVO_SPEED_MID, 200},
    //                                                              {110.0f, SERVO_SPEED_MID, 200},
    //                                                              {90.0f, SERVO_SPEED_MID, 200},
    //                                                              {110.0f, SERVO_SPEED_MID, 200},
    //                                                              {90.0f, SERVO_SPEED_MID, 200},
    //                                                              //  {110.0f, SERVO_SPEED_MID, 200},
    //                                                              //  {90.0f, SERVO_SPEED_MID, 200},
    //                                                              //  {110.0f, SERVO_SPEED_MID, 200}, {90.0f, SERVO_SPEED_MID, 200},
    //                                                          },
    //              .seq_len = 6,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 两臂：对称快摆。23°@FAST=230ms + 停 20ms = 250ms/半步，两步 = 500ms 一拍 × 10
    //     .left_arm = {.target = 38.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                  //  {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  //  {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  //  {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  //  {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  //  {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                  {38.0f, SERVO_SPEED_FAST, 20},
    //                                                              },
    //                  .seq_len = 10,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     .right_arm = {.target = 38.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                   {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 20},
    //                                                                   {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   //   {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   //   {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   //   {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   //   {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                                   //   {15.0f, SERVO_SPEED_FAST, 20}, {38.0f, SERVO_SPEED_FAST, 20},
    //                                                               },
    //                   .seq_len = 10,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 7_3: 怕痒③「怕痒 2」—— 主语=两臂【反相】；290ms × 18 拍，最快最碎
    //     .emotion_id = EMO_TICKLISH3,
    //     .screen_anim = "怕痒 2.gif",
    //     .gif_path = "S:/gif/7_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：钉在中位（与两臂的反相快抽形成对比）
    //     .head = {.target = 90.0f, .speed = SERVO_SPEED_MID, .seq = {{90.0f, 0, 3530}}, .seq_len = 1, .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：25°@FAST=250ms + 停 40ms = 290ms/步，两步 580ms 一个往复 × 9 = 18 步
    //     .left_arm = {.target = 40.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                  {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  //  {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  //  {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  //  {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                  //  {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                              },
    //                  .seq_len = 10,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：同样的节奏，但整条滞后一拍(290ms) ⇒ 左起右落，两臂反相
    //     //       .target 填 15°（= 当前位置）零位移，只占 1 帧，是为下面那步延迟做落点
    //     .right_arm = {.target = 15.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                   {15.0f, 0, 290}, // ← 相位延迟：左臂先动
    //                                                                                    //   {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                                    //   {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                                    //   {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                                    //   {40.0f, SERVO_SPEED_FAST, 40}, {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {40.0f, SERVO_SPEED_FAST, 40},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 40},
    //                                                               },
    //                   .seq_len = 11,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 7_4: 怕痒④「怕痒到扭动」—— 主语=全身扭；头 640ms / 臂 700ms × 8 拍，幅度最大
    //     .emotion_id = EMO_TICKLISH4,
    //     .screen_anim = "怕痒到扭动.gif",
    //     .gif_path = "S:/gif/7_4.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：围绕中位对称摆 90±16°（74↔106，全幅 32°）。⚠️ 半拍行程是【全幅 32°】
    //     //     而不是 ±16°，所以必须用 FAST(10ms/度)：32°×10 = 320ms/半拍 ⇒ 640ms/周期
    //     //     × 8 拍 ≈ 素材的 670ms。全程【不停顿】——不庄重正是本条与"摇头"的分界
    //     .head = {.target = 106.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                               {74.0f, SERVO_SPEED_FAST, 0},
    //                                                               {106.0f, SERVO_SPEED_FAST, 0},
    //                                                               {74.0f, SERVO_SPEED_FAST, 0},
    //                                                               {106.0f, SERVO_SPEED_FAST, 0},
    //                                                               {74.0f, SERVO_SPEED_FAST, 0},
    //                                                               {106.0f, SERVO_SPEED_FAST, 0},
    //                                                               {74.0f, SERVO_SPEED_FAST, 0},
    //                                                               {106.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {74.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {106.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {74.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {106.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {74.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {106.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {74.0f, SERVO_SPEED_FAST, 0},
    //                                                             //   {106.0f, SERVO_SPEED_FAST, 0},
    //                                                           },
    //              .seq_len =8,
    //              .seq_speed = SERVO_SPEED_FAST},
    //     // 左臂：35°全幅@FAST=350ms/半拍 ⇒ 700ms/周期，与头【同相】起步（0.64s vs
    //     //       0.70s 的微差是故意的：8 拍下来会缓慢错开一点，像真的在拧，不是机械锁相）
    //     .left_arm = {.target = 50.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //  {50.0f, SERVO_SPEED_FAST, 0},
    //                                                              },
    //                  .seq_len =8,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：与左臂【反相】（滞后半周期 350ms）⇒ 一臂抬一臂落，全身拧起来
    //     .right_arm = {.target = 15.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                   {15.0f, 0, 350}, // ← 相位延迟：反相
    //                                                                   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {50.0f, SERVO_SPEED_FAST, 0},
    //                                                                 //   {15.0f, SERVO_SPEED_FAST, 0},
    //                                                               },
    //                   .seq_len =9,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },

    /* ── 8_x：惊喜（3 张）─────────────────────────────────────────────────────
     * ★2026-09-21 重做。旧版 8_1 头填 `{0}`、8_2 与 8_3 【逐字完全相同】（注释自写"复用 8_2"），
     *   且两臂幅度只有 120↔130（10°）、头 85↔95（10°）—— 与 [[grieved_5x_redesign]] 同属
     *   "幅度没到可见阈值"那一类毛病。
     *
     * 【素材实测：三张画面几何全部零位移，只能自己发明动作】
     *   8_1 惊喜.gif      5.00s  周期 500ms ×约10次，★全程【无停顿】（墨量 846→657→846 三角波）
     *       动的只有：两颗五角星星眼同步张缩（无质心漂移，≤2.6px）。
     *   8_2 惊喜2.gif     5.00s  周期 500ms ×约10次，★谷底【连停 3 帧 ≈125ms】（1341,1341,1341）
     *       动的有：星星眼/圆形眼、圆心高光（蓝通道 244.9→214.5 变亮）、笑弧、两只手弧。
     *   8_3 惊喜抱抱.gif  5.33s  周期 667ms ×约8次，无停顿（墨量 695→625→695）
     *       ★【双眼全程纹丝不动】—— 像素差涂红只见眉弧和手弧在动。
     *
     * ⇒ 三条的偏航/前抬都无法从画面里"抄"，依据 = 名字语义 + 实测节拍。
     *
     * 【★★2026-09-21 用户上板前复核，三条头部全部推翻重做】
     *   ❌ 首版我曾写一条"素材里眼睛动不动 ⇒ 头该不该有戏"的映射，并据此把 8_3 的头【整体钉死】。
     *      用户指出后复核：**这条映射是我自己编的，不是从任何已验证原则推出来的**——它的真实
     *      来源是"要给 8_3 找一个和 8_1/8_2 不同的差异点"，而"头钉死"是最省事的那个（先有结论
     *      再包装成"忠实素材"）。逻辑上也站不住：素材里"眼睛不动"只说明**没有眨眼/星星闪烁**，
     *      说明不了**头和身体冻住**；真人抱住东西时头照样会靠过去、会呼吸。**该映射已删除。**
     *   ✅ 复核定下的硬原则（后面 9_x~14_x 照此执行）：
     *      ① **头是脸上唯一的关节，不许出现"整条钉死几秒"的设计**——那读起来是"坏掉"不是"情绪"。
     *      ② **头的动作必须比四肢慢一档、软一档**。给手臂用 VERY_FAST 是对的（抽泣/惊跳需要），
     *         同样 60ms 走 12° 搬到头上就不是"屏息"而是"舵机打嗝"。
     *      ③ 头可以和四肢**同拍**（全身一起使劲时是对的），也可以**差一倍拍**（分层），
     *         但绝不能和手臂**同速同幅**——那等于把头和胳膊焊在一起。
     *   ❌ 首版三处具体毛病：8_1 头后段 hold 4300ms 钉死（素材 5s 里死了 86%）；
     *      8_2 头用 VERY_FAST 60ms 猛收 + 200ms 硬憋（生硬）；8_3 头 `{0}` 完全不参与。
     *
     * 【设计：三条互不复用，头的幅度/速度档/节拍全不同】
     *   8_1 = 【惊 → 喜 两段结构】头 90→126 猛偏左 36°(180ms) 后★僵住 300ms —— 那"一顿"就是
     *       受惊本身；僵住之后头**不再钉死**，接着做 8 拍【102↔126，24° 全幅 @FAST = 240ms/半拍
     *       ⇒ 480ms 周期】（素材 500ms）。两臂弹到 140° 后做 6 拍【快降 35°@VERY_FAST(175ms)
     *       + 慢升 35°@FAST(350ms) = 525ms/拍】，★全程无停顿，升降不等速 = "抽"不是"匀速摆"。
     *       ⇒ 8_1 的头是三条里**最活跃、幅度最大**的（对应素材里星星眼最剧烈的张缩）。
     *   8_2 = 【憋-放】那个"憋"只给手臂：每拍【收 24°@VERY_FAST(120ms) + ★憋住 130ms
     *       + 放 24°@FAST(240ms) = 490ms/拍】×8，对上素材谷底连停 3 帧。
     *       ★头【同拍但不照搬】：改用 **MID** 走同样的 12°（180ms 而不是 60ms，**软 3 倍**），
     *       节奏是【93→105 收 180ms + ★柔停 130ms + 105→93 放 180ms】= 490ms。
     *       同拍是对的（8_2 本来就是"全身一起憋"），关键是**不能同速同幅**。
     *   8_3 = 【抱】三条里最慢最柔：两臂周期 660ms（前两条的 1.33 倍）且★没有顿挫，
     *       一路合抱到 163° 再柔和收放 6 个来回（130↔163，33° 全幅 @FAST = 330ms/半拍）。
     *       ★头 = 【靠过去 + 呼吸】：90→106 偏左 16°(MID 240ms) → 停 200ms（"靠上了"）→
     *       再 102↔114 **慢摆 3 拍**，一拍 180ms + **停 960ms = 1320ms**（★是两臂的 2 倍拍）。
     *       ⇒ 两臂在"抱紧-放松"、头在"慢慢点头"，两层叠起来是"窝在怀里呼吸"。
     *
     * 【幅度红线】手臂基线是 ARM_CENTER_DEG = 15°，本组最低的造型位是 105 / 104 / 130°，
     *   离基线足够远，不存在"幅度被齿隙吃掉"的问题（对比 [[grieved_5x_redesign]] 的 18° 事故）。
     * 【速度档】VERY_FAST=5ms/度、FAST=10ms/度、MID=15ms/度（MID 为实测最慢不抖档）。
     * 【停顿】所有"定住"一律写显式 hold_ms，绝不靠重复角度凑（零位移只花 1 帧≈20ms）。
     * ⚠️ 左右臂刻意错开 4°（140/136、128/124、163/159），避免完全同步的机械感。
     * ⚠️ 待办：8_1/8_2 这种高频组其实更适合 vib_intermittent（interaction.c:115 已写好但
     *   仍注释着），启用需要改公共注释区，等指令。
     * ⚠️ 三条的头都是连续往复摆动，全部靠【整体偏置在中位左侧、绝不跨越 90°】来规避"摇头"语义
     *   （同 [[shy_6x_redesign]] 的判据）：8_1 头 102~126、8_2 头 93~105、8_3 头 102~114，
     *   **区间下界全部 ≥90°**。其中 **8_1 头的 24° 全幅是本次唯一有"摇头"风险的地方**（幅度最大、
     *   节奏最快），上板务必重点看这条；若观感不对，第一优先是把 24° 收小到 12~16°，而不是改结构。
     */
    // {
    //     // ── 8_1: 惊喜①「惊喜」—— 惊(头定格) + 喜(两臂连续锯齿)，500ms 无停顿
    //     .emotion_id = EMO_SURPRISED1,
    //     .screen_anim = "惊喜.gif",
    //     .gif_path = "S:/gif/8_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：★"惊"由头承担，但★僵住之后【不许钉死】——素材 5s 里星星眼一直在张缩，头就该一直在动。
    //     //     90→126 猛偏左 36°(36×5=180ms) → 僵住 300ms（被吓到的那一顿，就是"受惊"本身）
    //     //     → 接着 8 拍【102↔126，24° 全幅 @FAST = 240ms/半拍 ⇒ 480ms 周期】（素材 500ms）。
    //     //     ★整段偏在左侧 102~126、**绝不跨越 90°** ⇒ 按 6_x 判据读作"激动得发颤"而非"摇头"。
    //     //     ⚠️ 本组唯一有摇头风险的地方就是这里（幅度最大 + 节奏最快），上板重点看。
    //     .head = {.target = 126.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                    {126.0f, 0, 300}, // ★僵住：这 300ms 就是"受惊"本身
    //                                                                                      // ── 僵住之后跟着素材的 500ms 节拍继续颤 8 拍，不再钉死 ──────────
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {102.0f, SERVO_SPEED_FAST, 0},
    //                                                                    {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                },
    //              .seq_len = 17,
    //              .seq_speed = SERVO_SPEED_FAST},
    //     // 左臂：★"喜"由两臂承担。弹到高位 140°(125×5=625ms) → 停 100ms →
    //     //       6 拍【35°@VERY_FAST=175ms 快降 + 35°@FAST=350ms 慢升】= 525ms/拍（素材 500ms）
    //     //       升降不等速 ⇒ 读作"一抽一抽"，而不是 3_x 兴奋那种对称匀速摆。
    //     //       全程【无停顿】，正对素材那条连续三角波。
    //     .left_arm = {.target = 140.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        {140.0f, 0, 100}, // 弹到高位，停 100ms 再做第一个"惊"的定格
    //                                                                                          // ── 以下 6 拍锯齿，无停顿 ──────────────────────────
    //                                                                        {105.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                        {140.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {105.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                        {140.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {105.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                        {140.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {105.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                        {140.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {105.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                        {140.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {105.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                        {140.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {30.0f, SERVO_SPEED_VERY_FAST, 0}, // 110°×5=550ms 快速收手
    //                                                                    },
    //                  .seq_len = 14,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：与左臂同构，整体刻意低 4°（136 / 101）避免机械同步。两侧时间线刻意错开约 20ms。
    //     .right_arm = {.target = 136.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                         {136.0f, 0, 100},
    //                                                                         // ── 6 拍锯齿，与左臂同拍 ────────────────────────────
    //                                                                         {101.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                         {136.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {101.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                         {136.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {101.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                         {136.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {101.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                         {136.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {101.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                         {136.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {101.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                         {136.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {30.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                     },
    //                   .seq_len = 14,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 8_2: 惊喜②「惊喜2」—— 「憋-放」一抽一停，头同拍屏息（全程偏右侧不跨中位）
    //     .emotion_id = EMO_SURPRISED2,
    //     .screen_anim = "惊喜2.gif",
    //     .gif_path = "S:/gif/8_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：★★2026-09-21 修正。首版用 VERY_FAST 60ms 猛收 12° + 硬憋 200ms，用户上板前看代码
    //     //     就指出"太生硬"——那个"抽一下定住"给手臂是对的（抽泣感），搬到头上就不是"屏息"
    //     //     而是"舵机打嗝"（头是脸上唯一的关节，动作必须比四肢慢一档、软一档）。
    //     //     ✅ 改法：★同拍但【不同速不同幅】——改用 MID 走同样的 12°（180ms 而不是 60ms，软 3 倍）：
    //     //     93→105 收 180ms + ★柔停 130ms + 105→93 放 180ms = 490ms/拍（与两臂同拍，全身一起憋）。
    //     //     ★区间 93~105 全程 ≥90°，绝不跨中位，读作屏息而非摇头。
    //     //     首步 {105,0,320} 把头的起拍对齐到两臂的 t≈565ms（两臂起手 113°×5=565ms）。
    //     .head = {.target = 105.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                              {105.0f, 0, 320},              // ★等两臂起手到位（15°×15=225ms + 20ms + 320 = 565ms）
    //                                                                                             // ── 8 拍"屏息"，与两臂同拍但慢 3 倍 ────────────────
    //                                                              {93.0f, SERVO_SPEED_MID, 130}, // 收 180ms + ★柔停 130ms
    //                                                              {105.0f, SERVO_SPEED_MID, 0},  // 放 180ms
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                              {93.0f, SERVO_SPEED_MID, 130},
    //                                                              {105.0f, SERVO_SPEED_MID, 0},
    //                                                          },
    //              .seq_len = 17,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：抬到 128°(113×5=565ms) → 8 拍【收 24°@VERY_FAST=120ms + ★憋 130ms
    //     //       + 放 24°@FAST=240ms】= 490ms/拍，对上素材 500ms 且谷底连停 3 帧。
    //     //       ★与 8_1 的唯一区别就是这个"憋"——8_1 是连续不断，8_2 是一抽一停。
    //     .left_arm = {.target = 128.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        // ── 8 拍"憋-放" ────────────────────────────────────
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130}, // 收 120ms + ★憋住 130ms
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},        // 放 240ms
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {104.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                        {128.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {30.0f, SERVO_SPEED_VERY_FAST, 0}, // 98°×5=490ms 快速收手
    //                                                                    },
    //                  .seq_len = 17,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：同构、刻意低 4°（124 / 100）。两臂同相（素材里左右是对称一起动的），错 4° 免机械。
    //     .right_arm = {.target = 124.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                         // ── 8 拍"憋-放"，与左臂同拍 ────────────────────────
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {100.0f, SERVO_SPEED_VERY_FAST, 130},
    //                                                                         {124.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {30.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                     },
    //                   .seq_len = 17,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 8_3: 惊喜③「惊喜抱抱」—— 头【靠过去 + 呼吸】+ 两臂柔和合抱收放，660ms 周期
    //     .emotion_id = EMO_SURPRISED3,
    //     .screen_anim = "惊喜抱抱.gif",
    //     .gif_path = "S:/gif/8_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：★★2026-09-21 修正。首版这里是 `{0}`（整体不参与），依据是我自己编的一条
    //     //     "素材里眼睛不动 ⇒ 头也不该动"的映射。用户指出后复核：**这条推理站不住**——
    //     //     眼睛不动只说明没有眨眼/星星闪烁，说明不了头和身体冻住；真人抱住东西时头照样会
    //     //     靠过去、会呼吸。而且头是脸上唯一的关节，5.3s 一动不动读起来是"坏掉"不是"情绪"。
    //     //     ✅ 改法 = 【靠过去 + 呼吸】两段：
    //     //       ① 90→106 偏左 16°(MID 240ms) → 停 200ms（"把脸靠进怀里"）
    //     //       ② 102↔114 慢摆 3 拍，每拍【180ms + ★停 960ms = 1320ms】——★节拍是两臂的 2 倍
    //     //     ⇒ 两臂在"抱紧-放松"（660ms）、头在"慢慢点头"（1320ms），两层叠起来 = 窝在怀里呼吸。
    //     //     ★区间 102~114 全程 ≥90°，绝不跨中位。
    //     .head = {.target = 106.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                              {106.0f, 0, 200},               // ★"靠上了"，停 200ms
    //                                                                                              // ── 3 拍慢呼吸，一拍 1320ms（两臂的 2 倍）──────────
    //                                                              {102.0f, SERVO_SPEED_MID, 960}, // 收 180ms + ★歇 960ms
    //                                                              {114.0f, SERVO_SPEED_MID, 0},   // 放 180ms
    //                                                              {102.0f, SERVO_SPEED_MID, 960},
    //                                                              {114.0f, SERVO_SPEED_MID, 0},
    //                                                              {102.0f, SERVO_SPEED_MID, 960},
    //                                                              {114.0f, SERVO_SPEED_MID, 0},
    //                                                          },
    //              .seq_len = 7,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：三条里最慢最柔的一条 —— 没有顿挫，就是"抱着轻轻晃"。
    //     //       抬到 163°(148×5=740ms 合抱位) → 6 个来回【130↔163，33° 全幅 @FAST
    //     //       = 330ms/半拍 ⇒ 660ms/周期】，正对素材 667ms。整条臂时间线 5565ms（素材 5330ms）。
    //     .left_arm = {.target = 163.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        // ── 6 个柔和来回，全程无停顿 ────────────────────────
    //                                                                        {130.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {163.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {130.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {163.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {130.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {163.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {130.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {163.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {130.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {163.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {130.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {163.0f, SERVO_SPEED_FAST, 0},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 0}, // 123°×5=615ms 松手放下
    //                                                                    },
    //                  .seq_len = 13,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：同构、刻意低 4°（159 / 126）。"抱抱"本就该对称，只靠这 4° 免掉机械感。
    //     .right_arm = {.target = 159.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                         // ── 6 个柔和来回，与左臂同拍 ────────────────────────
    //                                                                         {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {159.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {159.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {159.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {159.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {159.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {126.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {159.0f, SERVO_SPEED_FAST, 0},
    //                                                                         {40.0f, SERVO_SPEED_VERY_FAST, 0},
    //                                                                     },
    //                   .seq_len = 13,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },

    /* ── 9_x：慵懒（1 张）── 源文档《19 失望叹气》──────────────────────────────
     *   双手抬起又【无力地垂落】，抬起快、落下慢 —— 这个不对称就是"叹气"的核心节奏。 */
    // {
    //     // ── 9_1: 慵懒①「慵懒2」
    //     .emotion_id = EMO_SLUGGISH1,
    //     .screen_anim = "慵懒2.gif",
    //     .gif_path = "S:/gif/9_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1, // 慵懒气质，不配震动
    //     .audio_file = NULL,
    //     .head = {.target = 90.0f, .speed = SERVO_SPEED_FAST, .seq = {{90.0f, 0, 500}, {100.0f}}, .seq_len = 2, .seq_speed = SERVO_SPEED_MID},
    //     .left_arm = {.target = 75.0f, .speed = SERVO_SPEED_FAST, .seq = {{75.0f, 0, 500}, {20.0f}}, .seq_len = 2, .seq_speed = SERVO_SPEED_MID},
    //     .right_arm = {.target = 75.0f, .speed = SERVO_SPEED_FAST, .seq = {{75.0f, 0, 500}, {20.0f}}, .seq_len = 2, .seq_speed = SERVO_SPEED_MID},
    // },

    /* ── 10_x：撒娇（1 张）── 素材 撒娇.gif（全组仅此一张）─────────────────────
     *
     * ★2026-09-21 重做。旧版（源文档《8 俏皮眨眼》"右手举到耳侧、头同侧歪"）被推翻，
     *   旧代码四条硬伤：
     *   ① 头 target=75 而 seq={70},{75},{90}：在 70↔90 之间【围绕中位 90° 对称往返】，
     *      而机芯只有偏航轴 ⇒ 这是一条【零可见量的摇头】（±10° 全在齿隙里，同 BUG-033），
     *      语义还正好反过来（"撒娇"变成"不要不要"）。
     *   ② 左臂 target=30：手臂真基线是 ARM_CENTER_DEG=15°（不是文档以为的 90°），
     *      30 只抬了 15°；后面 {25} 更只动 5° ⇒ 整条停在齿隙里，上板看不见。
     *   ③ 右臂 target=125 起步合理，但 seq={135},{125},{25,MID} 里 130→25 那一下
     *      105°×5ms = 525ms 直接砸下来 —— 那是"打"不是"撒娇"。
     *   ④ 全条都是 VERY_FAST(5ms/°) 且【没有一处 hold 是用来定住姿势的】，
     *      读出来是【抽搐】，与"扭捏黏人"的气质相反。
     *
     * 【素材实测（逐帧 120 帧 / 5000ms / 320×240，按 duration 累加的真实时间轴）】
     *   · 周期 = 【恰好 500ms】：第 12/24/36…/108 帧与首帧【逐像素完全相同（差异=0）】，
     *     即 10 个完整周期，第 11 次回到原点收尾。
     *   · 波形 = 【对称三角波】：升 5 帧(≈208ms) + 降 5 帧(≈208ms)，★【全程无停顿】。
     *   · 动的部位（变化像素按行/列聚类，y113~122 完全无变化 ⇒ 上下两带互相独立）：
     *       右眼/wink 带 (x131-240, y77-112)：墨量 607 → 545，【净减 62px】= 弧变细（眯眼）
     *       嘴    带 (x131-244, y123-160)：墨量 778 → 839，【净增 61px】= 嘴张开
     *       左眼区   (x87-127,  y68-140)：墨量极差仅 9 / 1357 = 【0.7%，完全不动】
     *   · 画面几何【零位移】：所有帧的差异包围盒恒为 (75,244,77,160)，质心漂移仅 1.4px。
     *   ⇒ 素材 = 【只动右半张脸的「右眼一眯 + 嘴一张」】，舵机复现不了，
     *     这一组的身体动作【只能按名字语义 + 实测节拍自己发明】（同 6_x/7_x/8_x/11_x 那批）。
     *
     * 【由素材反推出的身体语言】
     *   素材的三个特征直接翻译成身体：① 500ms 一个来回 ⇒ 节奏是【不快不慢的匀速絮叨】，
     *   不是抽搐也不是懒散；② 【无停顿】⇒ 撒娇是"黏着你不放"，不设计"停住盯你"那种好奇式留白；
     *   ③ 【只有单侧在动】⇒ 身体取【两只手各按各的节奏晃】而非整齐划一，
     *   读作"扭捏着往你这边凑"，而不是"举手比划"（那是 3_x 兴奋那组的主语）。
     *
     * 【设计：两臂同幅、不同速、不同拍数】
     *   · 右臂（主）15↔48（33° 全幅）@FAST(10ms/°) ⇒ 半拍 330ms、整拍 660ms、【7 拍】
     *   · 左臂（配）15↔48（33° 全幅）@MID (15ms/°) ⇒ 半拍 495ms、整拍 990ms、【5 拍】
     *   ★ 同幅但【不同速度档、不同拍数(7:5 不整除)】⇒ 两臂相位差一路在变，
     *     眼睛看到的是"两只手各有各的节奏在晃"，比严格同相/反相都更"活"。
     *   （8_x 那条"绝不能同速同幅"禁的是【同速**且**同幅】，同幅不同速是允许的。）
     *   ⚠️ 不用 SLOWER/SLOW 档：bsp_config.h:298-299 实测那两档【有抖动】，
     *      本组 33° 幅度不算"小幅度"，故只用 FAST / MID 两个确定不抖的档。
     *
     * ⚠️【头的安全判据（见 [[shy_6x_redesign]] 对"不许连续来回摆"的修正）】
     *   判据不是"能不能往复"，而是【往复是否围绕中位 90° 对称】。
     *   本组头全程停在 102~114°，【一次都不跨越 90°】⇒ 是"偏置"不是"对称往复"
     *   ⇒ 读作"把头歪过去靠着你"，不会被读成摇头。若日后调节幅度，
     *   ★务必守住"不跨 90°"这条线，跨过去立刻变摇头。
     *
     * ⚠️【头必须比四肢慢一档、且不能与四肢同幅】【头不许整条钉死几秒】
     *   （两条均来自 [[surprised_8x_redesign]] 的教训）
     *   本组：头 MID(15ms/°) 走 12~24°，两臂 FAST/MID 走 33° —— 速度档与幅度都不同档；
     *   且头【不是一条长 hold】，而是"靠过去 → 定住 400ms → 断续蹭 9 下（走180ms+停300ms）"
     *   ⇒ 同拍但【结构不同】：两臂是连续的，头是断续的。
     *
     * 【时间线核算（MID=15ms/°、FAST=10ms/°、归中头 15ms/° 臂 10ms/°）】
     *   头：90→114 (24°×15=360) + 停 400 → 0.76s
     *       114↔102 (12°×15=180) + 停 300 = 480ms/段 × 9 段 → 5.08s
     *       归中 102→90 (12°×15=180) → 5.26s
     *   右臂：90→48 (42°×10=420) → 0.42s
     *         48↔15 (33°×10=330) × 14 步 → 5.04s
     *         归中 48→15 (33°×10=330) → 5.37s
     *   左臂：90→48 (42°×10=420) → 0.42s
     *         48↔15 (33°×15=495) × 10 步 → 5.37s
     *         归中 48→15 (33°×10=330) → 5.70s
     *   ⇒ 三轴都在 ≈5.3~5.7s 收尾（素材 5.0s），节拍与素材同为"匀速连续、无停顿"。
     *   ⇒ 容量：头 点数 12 / seq_len 10；两臂 点数 16 / seq_len 14 —— 全在 34 / 32 以内。 */
    // {
    //     // ── 10_1: 撒娇①「撒娇」── 主语：两臂（同幅不同速的"各自晃"）
    //     .emotion_id = EMO_ACT_CUTE1,
    //     .screen_anim = "撒娇.gif",
    //     .gif_path = "S:/gif/10_1.gif",
    //     .vib_seq = vib_light1, // 轻微 1 次（强度 50/60ms）—— 撒娇只给"轻轻一下"
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：偏左 24° 靠过去 → 定住 400ms"贴上了" → 在 102~114 之间断续蹭 9 下
    //     //     ★全程 ≥102° > 90° 中位，一次都不跨越 ⇒ 是"偏置往复"不是摇头
    //     .head = {.target = 114.0f, // 偏左 24°（大角=左）
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {114.0f, 0, 400},               // ★定住：长保持 = "靠住了"
    //                  {102.0f, SERVO_SPEED_MID, 300}, // 回撤 12°(180ms) + 停 300ms = 480ms/段
    //                  {114.0f, SERVO_SPEED_MID, 300}, // 靠回去 12°(180ms) + 停 300ms
    //                  {102.0f, SERVO_SPEED_MID, 300},
    //                  {114.0f, SERVO_SPEED_MID, 300},
    //                  {102.0f, SERVO_SPEED_MID, 300},
    //                  {114.0f, SERVO_SPEED_MID, 300},
    //                  {102.0f, SERVO_SPEED_MID, 300},
    //                  {114.0f, SERVO_SPEED_MID, 300},
    //                  {102.0f, SERVO_SPEED_MID, 300}, // 末位 102°
    //              },
    //              .seq_len = 10,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂（配角）：33° 全幅 @MID 慢档 ⇒ 半拍 495ms、整拍 990ms，共 5 拍
    //     //               与右臂同幅但慢 1.5 倍、少 2 拍 ⇒ 相位差一路在变，读作"另一只手慢慢跟着晃"
    //     .left_arm = {.target = 48.0f,
    //                  .speed = SERVO_SPEED_FAST, // 起步 42°×10 = 420ms
    //                  .seq = {
    //                      {15.0f, SERVO_SPEED_MID},
    //                      {48.0f, SERVO_SPEED_MID},
    //                      {15.0f, SERVO_SPEED_MID},
    //                      {48.0f, SERVO_SPEED_MID},
    //                      {15.0f, SERVO_SPEED_MID},
    //                      {48.0f, SERVO_SPEED_MID},
    //                      {15.0f, SERVO_SPEED_MID},
    //                      {48.0f, SERVO_SPEED_MID},
    //                      {15.0f, SERVO_SPEED_MID},
    //                      {48.0f, SERVO_SPEED_MID}, // 末位 48°
    //                  },
    //                  .seq_len = 10,
    //                  .seq_speed = SERVO_SPEED_MID},
    //     // 右臂（主角）：同 33° 全幅但 @FAST 快档 ⇒ 半拍 330ms、整拍 660ms，共 7 拍
    //     //               ★整拍 660ms 与素材 500ms 同量级；7 拍与左臂 5 拍不整除 = 刻意错位
    //     .right_arm = {.target = 48.0f,
    //                   .speed = SERVO_SPEED_FAST, // 起步 42°×10 = 420ms
    //                   .seq = {
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST},
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST},
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST},
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST},
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST},
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST},
    //                       {15.0f, SERVO_SPEED_FAST},
    //                       {48.0f, SERVO_SPEED_FAST}, // 末位 48°
    //                   },
    //                   .seq_len = 14,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },

    /* ── 11_x：治愈（1 张）── 素材 治愈2.gif（全组仅此一张）─────────────────────
     *
     * ★2026-09-21 重做。旧版（源文档《5 温柔微笑》）已被推翻，旧代码有三个硬伤：
     *   ① 头 target=80 而 seq={90} ⇒ 净位移只有 10°，落在齿隙里上板看不见（同 BUG-033）
     *   ② 两臂 seq_len=0 ⇒ servo_manager 在目标点后【总会】追加归中点，
     *      手臂抬到 45° 会被【立刻】拉回 15° 基线，等于完全没停住（见 4_x 表头那条警告）
     *   ③ 头 80° 是"往右偏"，与"温柔微笑"的气质无关，纯粹是把文档的"轻歪 10°"照抄了
     *
     * 【素材实测（逐帧 144 帧 / 6000ms / 320×240，按 duration 累加的真实时间轴）】
     *   · 周期 = 48 帧 = 【恰好 2.000 秒】，144 帧 = 3 个完整循环
     *   · 每周期四拍（逐帧数右眼区非透明像素，0 → 1727 → 0）：
     *       展开 290ms  →  峰停 750ms  →  收回 130ms  →  基停 830ms
     *     （展开 7 帧 / 峰停 18 帧 / 收回 4 帧 / 基停 20 帧）
     *   · ★动的部位【左右极不对称】，这才是本组的性格所在：
     *       右半 (x≥160)：942 px → 4070 px，【×4.32】——
     *                     一条细弧【爆发式撑成一个粗大的「>」折线】（笑眼撑开）
     *                     展开不是匀速：f3→f4 一帧之内 95→1228，【一帧吃掉 66% 行程】
     *       左半 (x<160)：945 px → 1490 px，【×1.58】——
     *                     还是那条弧，只是【上抬 + 加粗】，并没有变成「>」
     *       嘴：唯一【每帧匀速】变化的部位，从头到尾一直在慢慢涨
     *       腮红点：往外、往下移，同时变大
     *   · ★时间上也不对称：左眼【先到顶】(170ms vs 290ms)、【先收完】(1130 vs 1170ms)，
     *     右眼是后动后收的"压轴主角"。
     *   · 整幅画 bbox 171×75 → 222×127（宽 ×1.30 / 高 ×1.69），但【零平移、零旋转】
     *     ⇒ 舵机动作【无从照抄，只能按气质自己发明】（同 6_x/7_x/8_x 那批）。
     *
     * 【由素材反推出的身体语言：两臂 = 呼吸，且必须与「>」【逐拍锁相】】
     *   素材骨架「快展开 → 长保持 → 快收回 → 长保持」就是一个【呼吸】。
     *   但"配合"只能落成【时间锁相】，不能落成"形状对应"——
     *   手臂只有一个前抬自由度，没有缩放语义，"眼睛放大"翻译过来只能是"抬更高"，
     *   那是把形状当语义（8_x 就是编了一条这种映射把三条头全做错了）。
     *   ★所以本组两臂做【整整 3 个循环】，逐拍咬住素材：
     *       素材：展开 0→290 | 峰停 290→1040 | 收回 1040→1170 | 基停 1170→2000
     *       手臂：抬 0→255   | 停   255→1005 | 落   1005→1260 | 停   1260→2120
     *   （手臂每拍比素材名义值【多留 δ≈120ms 的余量】，为什么见下面三次修正那条）
     *   3 次张合 / 6.3 秒 = 【约 19~20 次/分 = 人的静息呼吸频率】，不是机械重复。
     *
     * 🔴【2026-09-21 二次修正：相位漂移】
     *   上一版只做了【2 个周期】、周期是 2700ms（510+1300+420+470），
     *   并在注释里给自己找了理由"3 次显机械感、治愈贵在静，少即是多"。
     *   ★那个理由是【事后找的】：素材明明循环 3 次，我却按自己的 2700ms 节奏数了 2 次，
     *   再把"少一次"包装成设计取舍——与 8_x「先有结论再包装成忠实素材」同一个毛病。
     *   实测后果：每循环比素材多 700ms，3 个循环累计漂 ~2.1s，正好漂掉一整拍。
     *   量化（眼睛睁着时的手臂平均高度）：
     *       第 1 个「>」 眼开时 66° / 眼闭时 65° —— 算搭上（其实全程高位没起伏）
     *       第 2 个「>」 眼开时 30° / 眼闭时 63° —— ★【完全反相】
     *       第 3 个「>」 眼开时 47° / 眼闭时 24° —— 错位约 500ms
     *   铁证：t=2290ms 眼睛睁满(1.00) 的瞬间，手臂正落在 24° 最低点；
     *         t=3040ms 眼睛要收了，手臂才抬到 56°。⇒ 与"配合"恰好相反。
     *   ★教训：「素材循环 N 次」就老老实实做 N 次、周期严格对齐，
     *     自己的节奏与素材不一致时，累计漂移迟早会把相位翻过来。
     *
     * ⚠️【头的安全判据（见 [[shy_6x_redesign]] 对"不许连续来回摆"的修正）】
     *   判据不是"能不能往复"，而是【往复是否围绕中位 90° 对称】。
     *   本组头全程停在 100~116°，【一次都不跨越 90°】⇒ 是"偏置"不是"对称往复"
     *   ⇒ 读作"轻轻侧过头靠着"，不会被读成摇头。若日后调节幅度，
     *   ★务必守住"不跨 90°"这条线，跨过去立刻变摇头。
     *
     * ⚠️【头必须比四肢慢一档、且不能与四肢同幅】（[[surprised_8x_redesign]] 三条硬原则）
     *   本组：头 MID(15ms/°) 走 12~22°，两臂 VERY_FAST(5ms/°) 走 51° —— 速度与幅度都不同档。
     *   ★注意：头【故意不跟着手臂一起锁相】，否则头和胳膊就焊在一起了（8_x 硬原则③）。
     *   另：头【不许整条钉死几秒】（同样来自 8_x 的教训），故头拆成三次慢微调，不是一次长 hold。
     *
     * 🔴🔴【2026-09-21 三次修正：★真机制 = 舵机结束的那一刻会【硬切】GIF】
     *   第三次用户反馈：「现在是三遍半？第三遍归中之后，又播放了 GIF 变成 `<`，然后突然截断。
     *                     手臂就跟不上节奏了，GIF 和手臂就错位了。」
     *   ★ 这次去读了执行器，才找到【确定的事实】（读之前两轮都在猜，猜错了一轮）：
     *     · interaction.c:2626 `xSemaphoreTake(s_ia_d1_2_sem, 10s)` —— 本函数【阻塞等舵机序列跑完】
     *     · interaction.c:2650 `ui_resume_main_gif_loop()` —— 舵机一完，【立刻】恢复主界面 GIF
     *     ⇒ **这次情绪的时长 == 舵机序列的时长**，GIF 只是被"到点硬切"。
     *       GIF 自己以 loop_count=0 无限循环，所以切在哪一拍【全凭时长碰巧】。
     *
     *   ⇒ 真正的约束不是"两套时钟会走散"，而是：
     *        **舵机总时长 ≈ GIF 循环周期 × 整数倍**
     *     否则多出来的零头会露出下一个循环的开头，并被硬切 —— 正是用户看到的
     *     "又播了一遍变成 `<` 然后突然截断"。
     *
     *   ⚠️⚠️ 上一轮（第二版注释，已删）我把它误判成"GIF 比舵机慢，每循环漂 δ≈125ms"，
     *     并且据此把谷底停 740→840（总时长 6.04s → 6.34s）。
     *     **6.34s 恰好越过 3 个循环（3×2.000=6.00s）多出 340ms** —— 而素材一个循环里
     *     "展开"只用 290ms ⇒ 340ms 足够让第 4 个 `<` 【完全成形】，然后被一刀切掉。
     *     **那个新症状是我改出来的**，不是素材或硬件的锅。教训：
     *     ★ 拿"用户描述"反推数值（δ=125ms）而不去读决定时长的那几行代码，会造出新的 bug。
     *
     *   ✅ 本次对策：把总时长压回 3 个循环【以内】，让"切点"落在第 3 个循环的【基停】里
     *     （画面最不敏感：眼已闭、手臂已在 15° 基线）——即使时长有 ±100ms 误差也切得好看。
     *     谷底停 840→740（回到对齐值），★末拍 740→600 再收 140ms ⇒ 总时长 5.90s。
     *
     * 【时间线核算（MID=15ms/°、VERY_FAST=5ms/°；★零位移的 seq 步也要吃 1 帧 ≈20ms）】
     *   头：90→112 (22°×15=330) +停1600 → 1.93s
     *       112→100 (12°×15=180) +停1500 → 3.61s
     *       100→116 (16°×15=240) +停1500 → 5.35s
     *       归中 116→90 (26°×15=390) → 5.74s
     *   臂：15→66 (51°×5=255) + 首步零位移 1 帧(20) +停 750 → 1.025s  ┐ 第 1 个呼吸
     *       66→15 (255) +停 740                              → 2.020s  ┘
     *       15→66 (255) +停 750                              → 3.025s  ┐ 第 2 个呼吸
     *       66→15 (255) +停 740                              → 4.020s  ┘
     *       15→66 (255) +停 750                              → 5.025s  ┐ 第 3 个呼吸
     *       66→15 (255) +停 600                              → 5.880s  ┘
     *       归中：已在 15° 基线 ⇒ 1 帧                              → 5.90s
     *   ⇒ 三拍抬到 66° = 255 / 2275 / 4275ms，对上素材睁眼 290 / 2290 / 4290ms
     *     ⇒ 差 −35/−15/−15ms。【均匀地略早一点点】才是关键，肉眼看不出来。
     *   ⇒ 切点 5.90s 落在第 3 个循环的 5.17~6.00s【基停】内 ⇒ 眼闭手落，切得干净。
     *   ⚠️ 头 5.74s 是名义值；头先静下来属正常（头本来就要比四肢慢一档）。
     *
     *   🔧【上板后若还要微调，旋钮是"总时长"，不是"相位"】
     *     · 看到第 4 个 `<` 成形后被切 ⇒ 总时长【超了】3 个循环 ⇒ 再减谷底停
     *     · 第 3 个 `<` 还没播完就被切   ⇒ 总时长【不够】 ⇒ 再加谷底停
     *     · 同步测量法（不用改代码）：串口日志自带毫秒时间戳，读
     *       ">>> 开始执行情绪动画: 48 <<<" 与 ">>> 情绪动作执行完毕: 48 <<<" 两行的时间差，
     *       就是舵机的【真实】总时长。名义值 5.90s；若实测明显偏离，说明舵机的名义时间
     *       跟真实时间有系统偏差，那才是需要按实测比例整体缩放的时候。 */
    // {
    //     // ── 11_1: 治愈①「治愈2」── 主语：两臂（三次缓慢张合 = 呼吸，与眼睛的 `<` 逐拍锁相）
    //     .emotion_id = EMO_HEALING1,
    //     .screen_anim = "治愈2.gif",
    //     .gif_path = "S:/gif/11_1.gif",
    //     .vib_seq = vib_light1, // 轻微 1 次（强度 50/60ms）—— 治愈只给"轻轻一下"
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     .head = {.target = 112.0f, // 偏左 22°（大角=左）
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {112.0f, 0, 1600}, // 定住 —— 温柔靠过去
    //                  {100.0f, 0, 1500}, // 回撤 12°，但【仍在 90 以左】
    //                  {116.0f, 0, 1500}, // 再靠过去 16°；末拍拉长到 1500 使头在 5.74s 收尾
    //              },
    //              .seq_len = 3,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：15° ⇄ 66° 完整张合 3 次，周期严格 2000ms 咬素材。
    //     //       ★用 VERY_FAST(5ms/°) 才能把 51° 压进素材的 290ms 展开窗口（255ms）
    //     .left_arm = {.target = 66.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                       {66.0f, 0, 750},                     // 峰停 750ms（素材 290→1040）
    //                                                                       {15.0f, SERVO_SPEED_VERY_FAST, 740}, // 收回 255ms 落到基线 15°
    //                                                                       {66.0f, SERVO_SPEED_VERY_FAST, 750}, // 第 2 个呼吸
    //                                                                       {15.0f, SERVO_SPEED_VERY_FAST, 1000},
    //                                                                       {66.0f, SERVO_SPEED_VERY_FAST, 750}, // 第 3 个呼吸
    //                                                                       {15.0f, SERVO_SPEED_VERY_FAST, 310}, // ★只压缩【末尾静止】，不碰前面任何一拍
    //                                                                   },
    //                  .seq_len = 6,
    //                  .seq_speed = SERVO_SPEED_VERY_FAST},
    //     .right_arm = {.target = 66.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        {66.0f, 0, 750}, // ★与左臂逐字相同 = 刻意同步
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 740},
    //                                                                        {66.0f, SERVO_SPEED_VERY_FAST, 750},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 1000},
    //                                                                        {66.0f, SERVO_SPEED_VERY_FAST, 750},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 310}, // ★与左臂逐字相同
    //                                                                    },
    //                   .seq_len = 6,
    //                   .seq_speed = SERVO_SPEED_VERY_FAST},
    // },

    /* ── 12_x：犯困（2 张）────────────────────────────────────────────────────
     *
     * 【★2026-09-21 逐帧拆素材后重做。旧版（源文档《23 打哈欠》《24 入睡》）整组作废——
     *   文档给的两个动作跟素材没关系：12_1 填"两臂 130→135→20"（135 已过向前极限的
     *   大半、20 贴着 15° 基线几乎没动），12_2 三轴全是 {0} + seq_len=0
     *   （servo_manager.c:252 总会追加归中点 ⇒ 会被立刻拉回，等于**完全没动作**）。】
     *
     * 【★★ 幅度红线：犯困是低能量情绪，本组位移一律压到 20° 以内 ★★】
     *   我自己的第一版是按素材的"画面变化面积"配的幅——12_1 两臂给了 15↔81
     *   （66° 全幅）连摆 3 个周期、12_2 两臂 60°/头 36°，用户一眼指出
     *   「这两个是犯困，你的动作幅度很大」。
     *   错在**判断依据选错了**：素材那对睡眼从"全睁开"到"全闭"在画面上面积变化
     *   1.9 倍（墨量 1941↔3714），看着占比很大；可那是**眼皮这种极小器官的满行程**，
     *   而且画面里本来也没别的东西可动。
     *   ⇒ **幅度要看"这个器官自己的满行程"，不是"画面上变化的面积占比"。**
     *   犯困该是**小幅度 + 长停顿**（没力气才动得少）；大幅度是挥手/兴奋。
     *   本组最终：两臂全幅 17~18°、头全幅 14~18°，时间全靠 hold 撑。
     *
     * 【素材实测（Pillow 逐帧 + 像素差 + 分区质心，42ms/帧）】
     *   12_1「犯困」   320×240 / 144 帧 / 6000ms
     *     · 构图 = 一对**闭着的睡眼弯月** + 下方一个圆下巴。
     *     · 几何：下巴圆(y≥130)质心漂 x0.7 / y1.7px ⇒ **零平移零旋转**；
     *       眼区墨量在 1941~3714 之间变 ⇒ 唯一在动的是**弯月的粗细（眯紧↔松开）**。
     *     · ★节拍：**周期 48 帧 = 2000ms 的对称三角波，正好 3 个完整循环，
     *       全程一个停顿都没有**（墨量 max@f0 / min@f24，即 t=0 睁满 → t=1000ms 眯到底）。
     *   12_2「犯困 2」 320×240 / 144 帧 / 6000ms
     *     · 构图 = 同样的睡眼 + 圆下巴，**右上角周期性飘出一个 "Zzz"**。
     *     · 几何：下巴圆质心漂 **x0.0 / y0.0px（分毫不动）**；眼区质心看着漂了 16px，
     *       实为 Zzz 飘进该区把质心拽偏（同 [[healing_11x_redesign]] 那个坑，这次靠
     *       "下巴锚点"识破）⇒ **零平移**，动的只有 Zzz 的出现/消失。
     *     · ★节拍：**周期 36 帧 = 1500ms，正好 4 个完整循环**；形态是
     *       基线 t=0 → 缓升到峰 **t=790ms（Zzz 最大）** → **t≈1130ms 一帧骤降** → 回基线。
     *       ⇒ 语义 = **慢慢沉入睡 → 沉到最沉 → 一个激灵"惊醒"**。
     *
     * ⇒ 两张素材和 6_2 / 7_1~7_3 / 8_1~8_3 / 11_1 一样是**几何零位移**（动的只是画出来的
     *   表情），舵机复现不了，只能按"名字语义 + 实测节拍"自己发明动作。
     *
     * 【三条硬约束（做本组时逐条核对过）】
     *   ① 总时长 ≈ GIF 周期 × 整数倍。interaction.c:2626 阻塞等舵机、:2650 舵机一完就
     *      ui_resume_main_gif_loop() 硬切 GIF（GIF 自己 loop_count=0 无限循环），
     *      所以切点必须落在循环边界或不敏感的姿态上。本次 12_1 = 2000×3、12_2 = 1500×4。
     *   ② 头**绝不跨中位 90°**。本组两条头都在 **[90,112] 区间内往复**（偏置往复 = 打盹；
     *      围绕中位对称的往复才是摇头/否定，那才是要禁的，见 6_x 的判据修正）。
     *   ③ **纯 hold 步会先花 1 帧（约 20ms）走到同角度再开始计时**，所以要在拍点上卡准
     *      "停 T 毫秒"，`hold_ms` 得写 **T − 20**。本组所有 `{角度, 0, N}` 都已扣过 20ms；
     *      漏算会让每个纯 hold 步多 20ms，几步累计就能把总时长顶出循环边界。
     *
     * 【★★ 第四轮：用户仍嫌"速度快" ⇒ 全组降档到 SLOWER + 把拍拉长 ★★】
     *   第三版把幅度压到 17~18° 后，用户仍反馈
     *   「犯困的第二张的手臂速度快，头的转速也快」。
     *   根因不是幅度，是**两个速度来源都没降**：
     *     (a) 单次滑动的档位还停在 MID(15ms/°)，且 12_2 的"惊醒/抬臂"还用了 FAST(10ms/°)；
     *     (b) 事件密度照抄了素材的循环数——12_2 每 1500ms 就来一次"沉+惊"，
     *         6 秒里 16 次运动，观感就是"忙 = 快"。
     *   ⇒ 两条一起治：
     *     (a) **全组降到 SERVO_SPEED_SLOWER(20ms/°)，彻底删掉 FAST。**
     *         ★为什么 SLOWER 是安全的：[bsp_servo.c:176] 的 servo_auto_frame_ms() 会
     *         **按速度自动加长帧长**（need = step_ms×0.8×1.5，向上取整到 20ms 的倍数，
     *         再钳在 [20,60]），把每帧位移顶回死区之上。实算四档：
     *           MID(15)      帧长 20ms → 每帧 1.33° → 死区(0.8°)余量 1.67×
     *           SLOWER(20)   帧长 40ms → 每帧 2.00° → 余量 **2.50×（比 MID 还安全）**
     *           SLOW(25)     帧长 40ms → 每帧 1.60° → 余量 2.00×
     *           VERY_SLOW(50)帧长 60ms → 每帧 1.20° → 余量 1.50×
     *         ⇒ bsp_config.h:296-300 那句"SLOWER/SLOW 实测【有抖动】"是
     *         **auto_frame 之前**的旧结论，现在已不成立（SLOWER 余量 2.5× 比 MID 的 1.67×
     *         还大）。本组保守只动一档(SLOWER)；上板后若还想更慢，**SLOW / VERY_SLOW
     *         就是现成的备用旋钮**（余量 2.0×/1.5× 仍高于死区）。
     *     (b) **拍数不再照抄素材循环数**。11_x 那轮已定过：硬约束是"总时长 ≈ GIF 周期
     *         × 整数倍"，**不是**"身体必须走素材那么多拍"。素材的 3 循环 / 4 循环只是
     *         眼皮和 Zzz 符号的节拍，照搬到肩和头上就变成"忙"。本组主角一律 3 拍
     *         × 2000ms，配角比主角慢 1.5 倍拍(3000ms)
     *         （12_2 的运动事件数因此从 16 次/6s 降到 12 次）。
     *
     * 【三轴分工（本组两条刻意做成不同性格，禁止复用同一动作体）】
     *   12_1 **主语是两臂**：15↔33（18° 全幅）@**SLOWER**，**对称**——浮起 360ms → 停 580ms
     *        → 落下 360ms → 停 700ms = 2000ms/周期，连走 3 个周期 ⇒ "睡得很稳"。
     *        ⇐ 打盹时肩膀的一起一伏。素材写的是"无停顿三角波"，但执行层做不出慢速
     *        连续运动（MID 下要凑满 2000ms 周期就得走 133° 行程，这恰是前几版依然太快
     *        的根源），所以改成"走一小段 + 停一大段"——观感同样是缓，幅度却只有 18°。
     *        头（配角）比两臂慢 1.5 倍拍：**3000ms/周期 × 2**，先偏左 22° 到 112°，
     *        再只在 **98↔112（14° 全幅）**里缓吸两次，末尾慢慢"醒"回 90°。
     *        全程 ≥98° ⇒ 不跨中位；最长静止段 1.32s，没有"整条钉死几秒"。
     *   12_2 **主语是头**：3 拍「慢慢沉下去 → 惊醒弹回」，每拍 2000ms ⇒ "睡不稳，一惊一惊"。
     *        头 90→106 沉 16°@SLOWER(320ms)，停在"睡得最沉"处 0.76s，再 106→92
     *        用 SLOWER(280ms) 弹回——★惊醒起点落在素材 t≈1130ms 的骤降点上。
     *        末拍不再回 92 而是**直接回到 90**（16°@SLOWER=320ms），让末尾归中变零位移。
     *        全程 [90,106] ⇒ 不跨中位。
     *        ※ 惊醒**不再用 FAST**：素材那下骤降只有 1 帧(42ms)，舵机复现不了；
     *          改靠**对比**——停 0.76s vs 走 0.28s（2.7:1），一样读得出"惊"。
     *        两臂（配角）比头慢 1.5 倍拍：**3000ms/周期 × 2**，15↔33（18°）@SLOWER
     *        ——垂 360ms + 停 1.12s + 抬 360ms + 停 1.14s。
     *        与头不同拍不同幅（8_x 原则③：头 16°/320ms、臂 18°/360ms）。
     */
    // {
    //     // ── 12_1: 犯困①「犯困」── 主语：两臂（18° 小幅度对称起伏，3 个周期）
    //     //    名义时间线：两臂 360 + (580+360+700) + 2×(360+580+360+700) + 归中20      = 6020ms
    //     //                头   440 + 1200+280+1320 + 280+1320+280+700 + 160 + 归中20    = 6000ms
    //     //                （素材 6000ms = 2000ms 周期 × 3；头按 3000ms/周期 × 2 走，慢 1.5 倍拍）
    //     .emotion_id = EMO_SLEEPY1,
    //     .screen_anim = "犯困.gif",
    //     .gif_path = "S:/gif/12_1.gif",
    //     .vib_seq = NULL,
    //     .vib_seq_len = 0, // 犯困气质，不配震动（见 interaction.c:118 约定）
    //     .audio_file = NULL,
    //     // 头：比两臂慢 1.5 倍拍（3000ms vs 2000ms），只在 98~112 的 14° 里缓吸 2 次
    //     //     98 > 90 ⇒ 全程偏置在中位一侧，不会读成摇头
    //     //     @SLOWER(20ms/°)：22° 偏过去 440ms、每次缓吸 14° 走 280ms
    //     .head = {.target = 112.0f, // 偏左 22°（大角=左）
    //              .speed = SERVO_SPEED_SLOWER,
    //              .seq = {
    //                  {112.0f, 0, 1180},               // 停住（= 想停 1200ms，已扣 1 帧）
    //                  {98.0f, SERVO_SPEED_SLOWER, 0},  // 缓沉 14°（280ms）
    //                  {98.0f, 0, 1300},                // 停住（= 想停 1320ms）
    //                  {112.0f, SERVO_SPEED_SLOWER, 0}, // 缓抬 14°（280ms）
    //                  {112.0f, 0, 1300},
    //                  {98.0f, SERVO_SPEED_SLOWER, 0},
    //                  {98.0f, 0, 680},                 // 停（= 想停 700ms）
    //                  {90.0f, SERVO_SPEED_SLOWER, 0},  // ★慢慢"醒"回中位（8°→160ms）
    //              },
    //              .seq_len = 8,
    //              .seq_speed = SERVO_SPEED_SLOWER},
    //     // 两臂：15↔33（18° 全幅）@SLOWER，**对称** 360 走 / 580·700 停，2000ms 一周期 ×3
    //     .left_arm = {.target = 33.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
    //                                                                    {33.0f, 0, 560},                // 停（= 想停 580ms）
    //                                                                    {15.0f, SERVO_SPEED_SLOWER, 0}, // 慢慢落下 18°（360ms）
    //                                                                    {15.0f, 0, 680},                // 停（= 想停 700ms）
    //                                                                    {33.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                    {33.0f, 0, 560},
    //                                                                    {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                    {15.0f, 0, 680},
    //                                                                    {33.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                    {33.0f, 0, 560},
    //                                                                    {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                    {15.0f, 0, 680},
    //                                                                },
    //                  .seq_len = 11,
    //                  .seq_speed = SERVO_SPEED_SLOWER},
    //     // 右臂：★与左臂逐字相同 = 刻意**同相**（犯困是瘫软，两臂一起沉浮才对）
    //     .right_arm = {.target = 33.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
    //                                                                     {33.0f, 0, 560},
    //                                                                     {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {15.0f, 0, 680},
    //                                                                     {33.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {33.0f, 0, 560},
    //                                                                     {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {15.0f, 0, 680},
    //                                                                     {33.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {33.0f, 0, 560},
    //                                                                     {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {15.0f, 0, 680},
    //                                                                 },
    //                   .seq_len = 11,
    //                   .seq_speed = SERVO_SPEED_SLOWER},
    // },
    // {
    //     // ── 12_2: 犯困②「犯困 2」── 主语：头（沉下去 → 惊醒，3 拍）
    //     //    名义时间线：头 3 拍、每拍正好 2000ms = 6000ms，+ 归中20             = 6020ms
    //     //                两臂 360 + (1120+360+1160) + (360+1120+360+1160) + 归中20 = 6020ms
    //     //                （素材 6000ms = 1500ms 周期 × 4；拍数不照抄循环数，见块头 (b)）
    //     .emotion_id = EMO_SLEEPY2,
    //     .screen_anim = "犯困 2.gif",
    //     .gif_path = "S:/gif/12_2.gif",
    //     .vib_seq = NULL,
    //     .vib_seq_len = 0,
    //     .audio_file = NULL,
    //     // 头：3 拍「沉 → 停 → 惊」，区间 90~106 全程 ≥90° ⇒ 偏置往复，不会读成摇头
    //     //     @SLOWER(20ms/°)：沉 16° 走 320ms、惊 14° 走 280ms；停 0.76s vs 走 0.28s
    //     //     的对比(2.7:1)就是"惊醒"，不再用 FAST 硬甩。
    //     .head = {.target = 106.0f, // 偏左 16°（大角=左），"沉下去"
    //              .speed = SERVO_SPEED_SLOWER,
    //              .seq = {
    //                  {106.0f, 0, 740},                // 停在"睡得最沉"（= 想停 760ms）
    //                  {92.0f, SERVO_SPEED_SLOWER, 0},  // ★惊醒！280ms 弹回
    //                  {92.0f, 0, 620},                 // t=2000ms，拍 1 收
    //                  {106.0f, SERVO_SPEED_SLOWER, 0},
    //                  {106.0f, 0, 760}, // t=3060ms
    //                  {92.0f, SERVO_SPEED_SLOWER, 0},
    //                  {92.0f, 0, 640}, // t=4000ms，拍 2 收
    //                  {106.0f, SERVO_SPEED_SLOWER, 0},
    //                  {106.0f, 0, 720}, // t=5020ms
    //                  {90.0f, SERVO_SPEED_SLOWER, 0}, // ★末拍直接回到中位（320ms）
    //                  {90.0f, 0, 640},                // t=6000ms
    //              },
    //              .seq_len = 11,
    //              .seq_speed = SERVO_SPEED_SLOWER},
    //     // 两臂：比头慢 1.5 倍拍（3000ms vs 2000ms），与头不同幅不同速（8_x 原则③）
    //     //       15↔33（18° 全幅）@SLOWER，**对称**：垂 360 走 / 停 1.12s / 抬 360 走 / 停 1.14s
    //     .left_arm = {.target = 33.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
    //                                                                    {33.0f, 0, 1100},               // 停（= 想停 1120ms）
    //                                                                    {15.0f, SERVO_SPEED_SLOWER, 0}, // 慢慢垂下 18°（360ms）
    //                                                                    {15.0f, 0, 1140},               // 停（= 想停 1160ms）⇒ 一拍 3000ms
    //                                                                    {33.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                    {33.0f, 0, 1100},
    //                                                                    {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                    {15.0f, 0, 1140},
    //                                                                },
    //                  .seq_len = 7,
    //                  .seq_speed = SERVO_SPEED_SLOWER},
    //     // 右臂：与左臂逐字相同 = 同相（困得两臂一起软）
    //     .right_arm = {.target = 33.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
    //                                                                     {33.0f, 0, 1100},
    //                                                                     {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {15.0f, 0, 1140},
    //                                                                     {33.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {33.0f, 0, 1100},
    //                                                                     {15.0f, SERVO_SPEED_SLOWER, 0},
    //                                                                     {15.0f, 0, 1140},
    //                                                                 },
    //                   .seq_len = 7,
    //                   .seq_speed = SERVO_SPEED_SLOWER},
    // },
    /* ── 13_x：生气（2 张）──────────────────────────────────────────────────────
     *
     * 【★2026-09-21 逐帧拆素材后重做。旧版（源文档《11 愤怒挥拳》《12 气到发抖》）
     *   整组作废——文档给的两个动作和三张素材没关系，且 13_2 只填了 ±4°/±8°
     *   的微抖（见下"死区红线"），上板必然是"看不见 + 舵机啸叫干磨"。】
     *
     * 【素材实测（用 Pillow 逐帧 + 像素差涂红 + 前景 bbox/质心，42ms/帧）】
     *   13_1「生气」   320×240 / 144 帧 / 6000ms
     *     · 几何：**极度特写的怒脸**（前景 bbox 164×112，几乎占满画面）。
     *       逐帧质心仅漂 ±0.7px ⇒ **零平移、零旋转**。
     *     · 唯一在动的是：两只斜怒眼的**下半部红色渐变（充血）在搏动**，
     *       眼形随之微涨（bbox 高 114→131）。
     *     · ★节拍：与首帧的差异呈 **18 个等间隔脉冲，周期 333ms**
     *       （变化段起点 t = 80/420/750/1080 … 步长恒 333ms）。
     *       ⇒ 语义 = **"气得眼睛一鼓一鼓"，超高频、强搏动**。
     *   13_2「生气 2」 320×240 / 120 帧 / 5000ms
     *     · 几何：只有**眉 / 眼 / 嘴**（bbox 130×66），是另一张更远的特写。
     *     · 唯一在动的是：**两条倒八怒眉整体下压**（顶部 y 84→97 下移 13px，
     *       前景像素 1953→1693 减少 13%，底部不动）⇒"眉压眼"。
     *     · ★节拍：**10 个等间隔脉冲，周期 500ms**（起点 80/580/1080/1580…）。
     *       ⇒ 语义 = **"持续用力地压眉"**。⚠️ 500ms = **2Hz，客观一点不慢**，
     *         是"压下去、松开、再压下去"的**反复使劲**，不是"没力气的缓慢"。
     *         （★2026-09-21 修正：初版把它读成"闷气/慢"，属于我加戏——
     *          素材只有"规律压眉"这一个事实，推不出"闷"。见下方设计段。）
     *
     * 【结论：两张都是纯脸部特写，舵机的偏航/前抬都"抄"不到】
     *   依据只有"名字语义 + 实测节拍"，身体动作必须自己发明
     *   （同 5_x/6_x/7_x/8_x/11_x 那批）。
     *
     * 【设计：两条的气质必须相反，且都不许读成"摇头"】
     *   13_1 = 【爆发式怒气】主语是**两臂**。
     *      两臂 15→72° 抬起绷住(VERY_FAST 285ms)，然后做 **8 个来回的"抖"**：
     *        {72 ↔ 48，24° 全幅 @VERY_FAST 单程 120ms + 停 223ms = **686ms/拍**}
     *      ——素材 333ms 的 2.06 倍（★不照抄 333ms：见下方"死区红线"）。
     *      左右臂**逐字相同 = 同相**，一起抖 = "身子在颤"。
     *      ★头**偏左 24° 别过去**（≠素材里头的朝向，是补出来的），全程停在 106~114
     *        **从不跨 90°**（承 [[shy_6x_redesign]] 的判据：偏置往复=赌气，对称往复=摇头），
     *        且只做 3 次**慢**调（变化起点间隔 2400/1020/980ms，**是两臂 686ms 的 1.4~3.5 倍**），
     *        用 MID 而非 VERY_FAST ⇒ 满足 8_x 复核定下的原则②③（头慢一档软一档、
     *        绝不与手臂同速同幅）。幅度也刻意不同：头 8°/6°，臂 24°。
     *   13_2 = 【全身一起使劲】主语是**头与臂同拍**，但反相。
     *      ⚠️ 初版这里写成"头偏右僵住 2 秒 + 只做 8°/6° 微调"，
     *        **同时踩了两条自己的规矩**：违反 8_x 原则①（头不许整条钉死几秒，
     *        静止 2.1s 读起来是"坏掉"）＋ 违反本组刚立的"死区红线"（8° 用 MID 走
     *        每帧仅约 1.3°，只比死区 0.8° 高一点，余量极薄）。用户上板前看代码指出
     *        "节奏很慢，体现不出愤怒" —— 根因就是头这 3.5 秒的零运动。
     *      修正后：
     *        头 **偏右 30° 别住**（60°）后**自己也抖**：{60 ↔ 76，16° 全幅 @FAST 单程 160ms
     *          + 停 120ms = **560ms/拍** —— **比 13_1 的 686ms 还快**，这是对本条
     *          "节奏很慢"最直接的回应}。范围 60~76 **离中位 14~30°** ⇒ 明确的偏置
     *          往复，不会读成围绕中位的摇头（承 [[shy_6x_redesign]] 判据）。
     *        两臂**低位砸**：{15 ↔ 40，25° 全幅 @VERY_FAST 单程 120ms + 停 160ms
     *          = **560ms/拍**}= **与头同拍**，但**不同速**（VERY_FAST vs FAST）
     *          **不同幅**（25° vs 16°）⇒ 满足 8_x 原则③（可同拍，绝不能同速同幅），
     *          也满足原则②（头比四肢慢一档）。
     *        两臂**反相**：右臂整条晚 280ms（= 半拍 560/2，首步 hold 160→440ms）
     *          ⇒ 左臂落时右臂抬，全身一起使劲。
     *      ⇒ 与 13_1 的区分（四条）：①主语 臂 vs 头 ②臂位 **48~72 高位挥** vs **15~40 低位砸**
     *        ③臂相位 同相 vs 反相 ④头 偏左 24° **慢调** vs 偏右 30° **快抖**。
     *        ⑤节奏 686 vs 560ms（13_2 更快）。
     *
     * 【★死区红线：为什么不敢照抄素材的 333ms】
     *   素材 13_1 是 333ms/拍的高频搏动，但**脸部搏动不需要舵机出力**，
     *   舵机不能这么干：周期越短 ⇒ 每个来回的 hold 越小 ⇒ 越接近"纯位移机械链"，
     *   而且小幅高频往复正是 [[project_servo_motion_curve]] 里"掉进死区"的成因
     *   （每帧位移 < 死区 0.8° ⇒ 舵机收不到指令却持续通电 ⇒ 啸叫 + 磨塑料齿轮，
     *   旧版 13_2 注释里"幅度必须保持 ±4°否则 MG90S 啸叫并迅速磨损"说的就是这件事）。
     *   ⇒ 本组一律取素材节拍的**整倍数**，幅度保持 16~25° 的大幅，并给**每一步**
     *      都写显式 hold（13_1 是 223ms，13_2 因为要更密所以压到 120~160ms——
     *      这是本条红线允许的下限，**再往下就要缩小幅度了，那就掉回死区，不许再压**）。
     *   ⚠️ 旧版 13_2 填的 86↔94（**8°**）与 46↔54（**8°**）、旧版 13_1 的 100↔40 虽大
     *      但 seq_len=2 无 hold（零停顿），都属于上面这条红线里的错误。
     *
     * 【时长对齐】13_1 三轴 max = 6000ms ≈ 素材 6000ms（**1 倍周期**）；
     *   13_2 三轴 max = 头 4980ms ≈ 素材 5000ms（500ms×**10 拍**）。
     *   ⇒ 两条都落在素材的**拍边界**上（承 [[healing_11x_redesign]]：舵机一完
     *   就硬切 GIF，总时长必须是素材周期的整数倍，否则切在第几拍全凭碰巧）。
     * 【速度档】VERY_FAST=5ms/度、FAST=10、MID=15（MID 为实测最慢不抖档）。 */
    // {
    //     // ── 13_1: 生气①「生气」── 爆发式怒气（主语：两臂同相发抖）
    //     //    时间线（两臂逐字相同）：
    //     //      抬起 15→72 (VERY_FAST 57°×5 = 285ms)
    //     //      → 15 次位移「72↔48，24° @VERY_FAST 单程 120ms + 停 223ms」= 686ms/拍
    //     //        （8 次下落 + 7 次抬起，最后停在 48 再归中——不是整 8 个对称来回，
    //     //          刻意让它"落在低处"，读起来是被压住而不是弹回来）
    //     //      名义总 = 285 + (20+223) + 15×(120+223) + 归中(33°×10=330) ≈ 6.00s
    //     //      ★按 bsp_servo 帧量化(20ms/帧)重放实测：头 6000 / 左臂 5988 / 右臂 5988ms
    //     //        ⇒ 三轴 max = 6.00s = 素材 6000ms 的**正好 1 倍周期**，切图落在拍边界上
    //     //        （验收线：上板日志两行时间差应≈6000ms，见 [[healing_11x_redesign]]）
    //     .emotion_id = EMO_ANGRY1,
    //     .screen_anim = "生气.gif",
    //     .gif_path = "S:/gif/13_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：偏左 24° 别过去后只做 3 次慢调（变化起点间隔 2400/1020/980ms，是两臂 686ms 的 1.4~3.5 倍）
    //     //     幅度 8°/6°、档位 MID ⇒ 与两臂（24°/VERY_FAST）**不同速不同幅**（8_x 原则③）
    //     //     全程 106~114 全部 > 90 ⇒ 偏置往复，不会读成摇头
    //     .head = {.target = 114.0f, // 偏左 24°（大角=左），"别过脸生气"
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {114.0f, 0, 2400}, // ★停住：整段里"不理人"的主体时间
    //                  {106.0f, SERVO_SPEED_MID, 900},
    //                  {114.0f, SERVO_SPEED_MID, 900},
    //                  {108.0f, SERVO_SPEED_MID, 820},
    //              },
    //              .seq_len = 4,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：抬起绷住 → 8 个来回的高频抖（24° 全幅，远超齿隙）
    //     .left_arm = {.target = 72.0f, // 15→72，抬 57°
    //                  .speed = SERVO_SPEED_VERY_FAST,
    //                  .seq = {
    //                      {72.0f, 0, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                      {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                  },
    //                  .seq_len = 16,
    //                  .seq_speed = SERVO_SPEED_VERY_FAST},
    //     // 右臂：★与左臂逐字相同 = 刻意**同相**（对比 13_2 的反相）
    //     .right_arm = {.target = 72.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        {72.0f, 0, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {72.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                        {48.0f, SERVO_SPEED_VERY_FAST, 223},
    //                                                                    },
    //                   .seq_len = 16,
    //                   .seq_speed = SERVO_SPEED_VERY_FAST},
    // },
    // {
    //     // ── 13_2: 生气②「生气 2」── 全身一起使劲（主语：头也抖，两臂反相砸）
    //     //    ⚠️ 初版写成"头偏右僵住 2 秒 + 只做 8°/6° 微调"，被用户指出
    //     //       "节奏很慢，体现不出愤怒"。根因 = 头在 5.5s 里有 3.5s 零运动
    //     //       （违反 8_x 原则①），且 8° 用 MID 走每帧仅 1.3°，几乎贴死区。
    //     //    本版：**头自己也抖，而且比 13_1 更快（560ms/拍 < 686ms/拍）**；
    //     //          两臂从 13_1 的高位挥改成**低位砸**，反相错半拍。
    //     //    时间线（头 16 步 / 两臂各 16 步，头与臂同拍 560ms，但不同速不同幅）：
    //     //      头  到位 440(MID) + (20+120) + 15×(160+120) + 归中200 = 4980ms
    //     //      左臂 到位 120(VF) + (20+160) + 15×(120+160) + 归中20  = 4520ms
    //     //      右臂 同左臂，★首步 hold 160+280 = 440（整半拍）       = 4800ms
    //     //      ⇒ 整体取 max = 头 4980ms ≈ 素材 5000ms（500ms×10 拍，落在拍边界）
    //     //      （验收线：上板日志两行时间差应≈5000ms，见 [[healing_11x_redesign]]）
    //     .emotion_id = EMO_ANGRY2,
    //     .screen_anim = "生气 2.gif",
    //     .gif_path = "S:/gif/13_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：偏右 30°(60°) 别住之后**自己也抖** —— {60 ↔ 76，16° 全幅 @FAST 单程 160ms
    //     //     + 停 120ms = **560ms/拍**（比 13_1 的 686ms 更快）}。
    //     //     范围 60~76 **全部远低于 90°**（离中位 14~30°）⇒ 明确的偏置往复，
    //     //     不会读成围绕中位的摇头（承 [[shy_6x_redesign]] 判据）。
    //     //     ⚠️ 头绝不用 VERY_FAST（8_x 原则②：搬到头上就是"舵机打嗝"）；
    //     //        FAST 已是头能用最快的一档，16°@FAST 每帧 2° 远高于死区 0.8°。
    //     .head = {.target = 60.0f, // 偏右 30°（小角=右）
    //              .speed = SERVO_SPEED_MID,
    //              .seq = {
    //                  {60.0f, 0, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //                  {60.0f, SERVO_SPEED_FAST, 120},
    //                  {76.0f, SERVO_SPEED_FAST, 120},
    //              },
    //              .seq_len = 16,
    //              .seq_speed = SERVO_SPEED_FAST},
    //     // 左臂：**低位砸**（15 ↔ 40，25° 全幅）——与 13_1 的高位 48~72 挥形成对比。
    //     //     15° 就是手臂基线（完全放下），所以这是"在身前一下一下地砸/拍"。
    //     .left_arm = {.target = 40.0f, // 15→40
    //                  .speed = SERVO_SPEED_VERY_FAST,
    //                  .seq = {
    //                      {40.0f, 0, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                      {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                  },
    //                  .seq_len = 16,
    //                  .seq_speed = SERVO_SPEED_VERY_FAST},
    //     // 右臂：与左臂逐字相同，只把**首步 hold 加 291ms（= 半拍 582/2）**
    //     //       ⇒ 左臂落时右臂抬 = 反相，全身一起使劲
    //     .right_arm = {.target = 40.0f, .speed = SERVO_SPEED_VERY_FAST, .seq = {
    //                                                                        {40.0f, 0, 440}, // ★160 + 280 = 整半拍错开
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {40.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                        {15.0f, SERVO_SPEED_VERY_FAST, 160},
    //                                                                    },
    //                   .seq_len = 16,
    //                   .seq_speed = SERVO_SPEED_VERY_FAST},
    // },

    /* ── 14_x：舒服（3 张）── 2026-09-21 重做，取代源文档《7 满足舒展》─────────
     *
     * 【逐帧拆素材得到的真实数据】（★用 duration 累加的真实时间轴，不数帧序号）
     *   14_1「舒服.gif」      144 帧 / 6000ms。**周期恰好 24 帧 = 1000ms，整 6 个循环**
     *     整幅画的 bbox 宽**恒定 191px 完全不变**；只有质心在竖直方向 114↔118 浮动
     *     （**4px**，横坐标恒为 159 零位移）。画面内容 = 一张弯月笑眼 + 两个腮红点 +
     *     微笑嘴的笑脸，**整张脸一起上下轻微起伏**，没有任何局部单独动。
     *     ⇒ 语义 = **最均匀、最平缓的一次"舒服地呼吸"**（素材本身就是等幅等周期的）。
     *   14_2「舒服到打滚.gif」 128 帧 / 5330ms。周期 16 帧 = **667ms，整 8 拍**。
     *     上沿 y 78→71（升 7px）、质心 y 122→118、宽 138↔128（**收 10px**）——
     *     是**以脸为中心的上下缩放（一缩一舒）**，零旋转零平移。
     *     画面 = 眯起眼（^^ 带波浪眉）+ 大微笑。
     *     ⇒ 语义 = **舒服得眯眼、一缩一舒**；素材名里的"打滚"在画面上**不存在旋转**，
     *       舵机抄不到，只能按"缩放"这个实测特征发明动作。
     *   14_3「舒服(1).gif」   144 帧 / 6000ms。**周期 48 帧 = 2000ms，整 3 个循环**
     *     画面 = 同一张笑脸 + **红心一颗颗从下往上冒**。逐帧数红心：
     *       t=130~330ms 左眼位（x≈110）一颗从 y132 升到 y66；t=420~580 右眼位
     *       （x≈165）一颗 y113→71；t=670~880 右侧（x≈207）一颗 y117→64；
     *       t≈1170~1670ms **一次同时冒出 4~5 颗**（红心像素从 ~350 暴涨到 1666）。
     *     ⇒ 语义 = **心一颗颗顶上来、后半段爆发**。
     *
     * 【设计：三组气质必须互不相同（承 [[feedback_pause_plus_variation]]）】
     *   三张素材**画面几何完全不同**（浮动 / 缩放 / 冒心），所以身体也绝不能复用一套。
     *   共同点：主语都是**两臂**（手臂才有能做"舒展/缩/顶"的物理自由度；
     *     头只有偏航一轴，上下浮动它做不出来），头退居**注脚**，只慢调 2~4 次。
     *   ┌ 14_1 = 【同相·轻·快】两臂同相 30°（15↔45°）起伏，FAST，**6 个整起伏**
     *   │         上下沿幅度最小、速度档最快 ⇒ 读作"轻轻快快地喘气"
     *   │         头 116→130→118，**单向偏左不来回**（承 [[curious_4x_redesign]] 的
     *   │         "定向→停留"，读作"舒服得靠过去"）
     *   ├ 14_2 = 【同相·重·慢】两臂同相 40°（15↔55°）缩舒，FAST 但**只有 4 个整起伏**
     *   │         幅度比 14_1 大 1.33 倍、次数只有一半 ⇒ "深深地把身子缩一下再摊开"
     *   │         头 110→124→112，同样单向偏左
     *   └ 14_3 = 【交错·波浪】两臂同为 30°（15↔45°）上顶，但右臂**整条晚 675ms**
     *             （约 1/3 个 2000ms 周期）⇒ 左臂到顶时右臂正在下落，
     *             两臂形成**一波一波的涟漪**，对上"红心一颗颗往上冒"
     *             头 100↔122 做 **4 次偏置往复**（全程 >90° **绝不跨中位** ——
     *             按 [[shy_6x_redesign]] 的判据：偏置往复 = "舒服得蹭"，不是摇头）
     *             ⇒ 与 14_1/14_2 的"头只单向偏"形成第三种头部语言
     *
     * 【⚠️ 头不许"整条钉死"（承 [[surprised_8x_redesign]] 复核定下的硬原则①）】
     *   14_1 的素材里脸几乎不动，但**不能因此把头冻结 6 秒**——那样读起来是"坏掉"。
     *   三组头部都是 2~4 次变化，最短的停 1400ms，绝无长钉死。
     *
     * 【⚠️ 速度档的硬约束（本次查 bsp_config.h:294-300 才注意到）】
     *   SLOWER(20)/SLOW(25)/VERY_SLOW(50) 三个宏的注释都写着**实测有抖动、
     *   仅限小幅度动作**；而 memory 里的翻译规则是"位移 ≥15° 时最慢只钳到 MID"。
     *   ⇒ 本次头部的**大位移（20~26°）一律用 MID**，
     *     只有 seq 内部 **<15° 的收尾小调整**才用 SLOWER（14° / 12° / 10°）。
     *   ⚠️ 头位移 22° 配 MID：每帧 20ms 走 0.8° = 恰好压住死区红线 0.8°，
     *      是"能看见"的最低要求（承 [[angry_13x_redesign]] 的死区红线），不要再往下调。
     *
     * 【时长对齐】（承 [[healing_11x_redesign]]：舵机一完就硬切 GIF，
     *   总时长必须是素材拍长的整数倍，否则切在第几拍全凭碰巧）
     *   14_1 三轴均 **6000ms** = 素材 6×(1000ms)  ★正好 6 个整循环
     *   14_2 三轴均 **5336ms** = 素材 8×(667ms)   ★正好 1 个整循环（≈5330ms）
     *   14_3 三轴均 **6000ms** = 素材 3×(2000ms)  ★正好 3 个整循环
     *   🔧 上板微调旋钮 = 总时长（见第 4 颗心被切掉 = 长了 / 第 3 颗没冒完 = 短了），
     *      实测法 = 日志里"舵机开始 / 舵机结束"两行的时间差（自带毫秒戳，不用改码）。
     * 【速度档速查】VERY_FAST=5ms/度、FAST=10、MID=15（实测最慢不抖档）、
     *   SLOWER=20、SLOW=25、VERY_SLOW=50；手臂基线 ARM_CENTER_DEG=15°（不是 90°）。
     *
     * 【🔴 本组新增的坑：算步长时必须按底层**帧量化**算，不能按 位移×毫秒/度 算】
     *   bsp_servo 每步按 `SERVO_FRAME_MS=20ms` 的整数帧推进，且**向上取整**
     *   （`frames = floor(位移×速度/20) + 1`）。所以
     *     30° @MID(15) 不是 450ms 而是 **460ms**（23 帧）；
     *     45° @MID 不是 675ms 而是 **680ms**（34 帧）；
     *     45° @FAST(10) 不是 450ms 而是 **460ms**；40° @FAST 是 **420ms**（21 帧）；
     *     26° @MID 是 **400ms**；22° @MID 是 **340ms**；14° @SLOWER 是 **300ms**；
     *     12° @SLOWER 是 **260ms**；10° @SLOWER 是 **220ms**。
     *   ⚠️ 我第一版就是漏了这一步：把 30°@MID 当 450ms，于是 12 步 ×(450+550)
     *      算成 "6000ms"，实际是 **12160ms**（正好 2 倍）——**差点又交一个时长错一倍的组**。
     *   ⇒ 本文件里每个轴的注释都直接写**按帧量化后的实测值**，并给出逐项相加的算式；
     *     改任何一个角度/速度/hold 后，请照算式重加一遍（或用 Python 按本节语义重放）。
     *   ★另外三条同样是这次踩到的：
     *     ① 零位移的一步**只要有 hold_ms>0 就真的停那么久**（只花 1 帧约 20ms 的是
     *        hold_ms=0 的零位移）—— 14_3 右臂就是靠 `{15.0f, 0, 675}` 这步做"晚半拍"的。
     *     ② 手臂的目标值 45° **不等于 45° 的行程**：基线是 15°，所以 15↔45 只有 **30°**。
     *     ③ 目标点与归中点的耗时也要算进去，且**归中的速度不是动作速度**：
     *        头归中用 SERVO_SPEED_CENTER=MID，**臂归中用 SERVO_SPEED_CENTER_ARM=FAST**。
     *        末步若已停在归位角（如 15°），归中段是**零位移 20ms** 而不是一整段行程。
     *   ⚠️ 这三条 + 帧量化，正是我这一组前后算错两次（第一次差一倍、第二次差 140ms）
     *      的全部原因。**改完务必用 Python 重放一遍**，不要只看注释里的算式。*/
    // {
    //     // ── 14_1: 舒服①「舒服」── 同相轻快呼吸（主语：两臂同相，头单向偏左）
    //     //    两臂逐字相同：从基线 15° 抬到 45°（30° @FAST = 320ms）+ 停 152ms
    //     //      ⇒ **半步 472ms，一次完整起伏 ≈944ms**（素材是 1000ms/起伏 ⇒ 基本 1:1）
    //     //      共 6 次起伏 = 12 步，末步 hold 压到 148ms 把那 4ms 找平
    //     //    ★为什么幅度只有 30°、档位却是 FAST：素材 14_1 的起伏本来就**又轻又快**
    //     //      （6 个循环塞进 6 秒），要凑出 6 次就必须把单个行程压短；
    //     //      MID 下一趟要 460ms，12 步光行程就 5.5s，塞不进 6 次
    //     //    ★与 14_2 的区分：本组**幅度小、次数多**（30°×6 次），
    //     //      14_2 是**幅度大、次数少**（40°×4 次）——不是"同一招换参数"
    //     .emotion_id = EMO_COMFORTABLE1,
    //     .screen_anim = "舒服.gif",
    //     .gif_path = "S:/gif/14_1.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：90→116 (26° @MID = 20 帧 400ms) → 停 1500 → 130 (14° @SLOWER = 15 帧 300ms)
    //     //     → 停 1500 → 118 (12° @SLOWER = 13 帧 260ms) → 停 1580 → 归中 118→90 (28°@MID 440ms)
    //     //     总 = 400 + (20+1500) + (300+1500) + (260+1580) + 440 = **6000ms** ✓
    //     //     最长的一停 1580ms，且此时两臂仍在起伏 ⇒ 不构成"钉死"
    //     .head = {.target = 116.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                              {116.0f, 0, 1500},                  // ★定住：靠过去以后"舒服地待着"
    //                                                              {130.0f, SERVO_SPEED_SLOWER, 1500}, // 再慢慢偏深 14°（小位移才敢用慢档）
    //                                                              {118.0f, SERVO_SPEED_SLOWER, 1580}, // 略收回 12° 长静默收尾
    //                                                          },
    //              .seq_len = 3,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：15↔45°（**从基线起算只有 30° 行程**）@FAST = 16 帧 320ms，
    //     //      加停 177ms ⇒ **半步 497ms，一次完整起伏 994ms ≈ 素材的 1000ms/起伏** ✓
    //     //      总 = 20(起点 15→15) + 8×497 + 4×496 + 20(归中 15→15 零位移) = **6000ms** ✓
    //     //      ⚠️ 注意末步收在 15° 就是中位，归中段是【零位移 20ms】不是整段行程——
    //     //         我第一版把它当成 30° 的归中(320ms)多算了，整轴时长才对不上
    //     .left_arm = {.target = 15.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                  {45.0f, 0, 177},
    //                                                                  {15.0f, 0, 177}, // 第 1 次起伏
    //                                                                  {45.0f, 0, 177},
    //                                                                  {15.0f, 0, 177}, // 第 2 次
    //                                                                  {45.0f, 0, 177},
    //                                                                  {15.0f, 0, 177}, // 第 3 次
    //                                                                  {45.0f, 0, 177},
    //                                                                  {15.0f, 0, 177}, // 第 4 次
    //                                                                  {45.0f, 0, 176},
    //                                                                  {15.0f, 0, 176}, // 第 5 次
    //                                                                  {45.0f, 0, 176},
    //                                                                  {15.0f, 0, 176}, // 第 6 次（收在基线）
    //                                                              },
    //                  .seq_len = 12,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     // 右臂：与左臂逐字相同 = 同相（14_3 才做交错）
    //     .right_arm = {.target = 15.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                   {45.0f, 0, 177},
    //                                                                   {15.0f, 0, 177},
    //                                                                   {45.0f, 0, 177},
    //                                                                   {15.0f, 0, 177},
    //                                                                   {45.0f, 0, 177},
    //                                                                   {15.0f, 0, 177},
    //                                                                   {45.0f, 0, 177},
    //                                                                   {15.0f, 0, 177},
    //                                                                   {45.0f, 0, 176},
    //                                                                   {15.0f, 0, 176},
    //                                                                   {45.0f, 0, 176},
    //                                                                   {15.0f, 0, 176},
    //                                                               },
    //                   .seq_len = 12,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 14_2: 舒服②「舒服到打滚」── 同相深缩舒（主语：两臂同相，幅度更大次数更少）
    //     //    两臂逐字相同：从基线 15° 抬到 55°（40° 行程 @FAST = 21 帧 420ms）+ 停 242ms
    //     //      ⇒ 半步 662ms，**一次完整缩舒 1324ms，只做 4 次**（不是 8 次）
    //     //      总 = 20 + 8×662 + 20(归中零位移) = **5336ms** ✓
    //     //    ★这里刻意**不追素材的 8 拍**：40° 在 FAST 下光行程就 420ms，
    //     //      凑 8 次要 16 步（≈9.8s）或把 hold 压到 ~30ms——
    //     //      前者太长、后者等于"纯位移机械链"（正是用户最早说"别扭"的成因）。
    //     //      取舍：**保住幅度和停顿，把次数减半，让总时长仍落在素材的拍边界上**
    //     //      （5336 = 667×8，GIF 正好走完一个完整循环再切）
    //     //    ★与 14_1 的三点区分：幅度 40° vs 30°、次数 4 vs 6、单步 612ms vs 472ms
    //     //    头：90→110 (20° @MID = 16 帧 320ms) → 停 1400 → 124 (14°@SLOWER 300ms)
    //     //        → 停 1400 → 112 (12°@SLOWER 260ms) → 停 1296 → 归中 112→90 (22°@MID 340ms)
    //     //        总 = 320 + (20+1400) + (300+1400) + (260+1296) + 340 = **5336ms** ✓
    //     //      ⚠️ 头只做 2 次调整，两臂做 8 步 ⇒ 头明显慢一档（承 8_x 原则②③）
    //     .emotion_id = EMO_COMFORTABLE2,
    //     .screen_anim = "舒服到打滚.gif",
    //     .gif_path = "S:/gif/14_2.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     .head = {.target = 110.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                              {110.0f, 0, 1400},                  // ★定住：眯起眼舒服地待着
    //                                                              {124.0f, SERVO_SPEED_SLOWER, 1400}, // 微微陷进去 14°
    //                                                              {112.0f, SERVO_SPEED_SLOWER, 1296}, // 松回来 12°，收尾
    //                                                          },
    //              .seq_len = 3,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：15↔55°（**从基线起算 40° 行程**）@FAST = 21 帧 420ms，
    //     //      加停 242ms ⇒ **半步 662ms，一次完整缩舒 1324ms，共 4 次**
    //     //      总 = 20 + 8×662 + 20(归中 15→15 零位移) = **5336ms** ✓
    //     .left_arm = {.target = 15.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                  {55.0f, 0, 242},
    //                                                                  {15.0f, 0, 242}, // 第 1 次缩舒
    //                                                                  {55.0f, 0, 242},
    //                                                                  {15.0f, 0, 242}, // 第 2 次
    //                                                                  {55.0f, 0, 242},
    //                                                                  {15.0f, 0, 242}, // 第 3 次
    //                                                                  {55.0f, 0, 242},
    //                                                                  {15.0f, 0, 242}, // 第 4 次（收在基线）
    //                                                              },
    //                  .seq_len = 8,
    //                  .seq_speed = SERVO_SPEED_FAST},
    //     .right_arm = {.target = 15.0f, .speed = SERVO_SPEED_FAST, .seq = {
    //                                                                   {55.0f, 0, 242},
    //                                                                   {15.0f, 0, 242},
    //                                                                   {55.0f, 0, 242},
    //                                                                   {15.0f, 0, 242},
    //                                                                   {55.0f, 0, 242},
    //                                                                   {15.0f, 0, 242},
    //                                                                   {55.0f, 0, 242},
    //                                                                   {15.0f, 0, 242},
    //                                                               },
    //                   .seq_len = 8,
    //                   .seq_speed = SERVO_SPEED_FAST},
    // },
    // {
    //     // ── 14_3: 舒服③「舒服(1)」── 错开起手的涟漪顶（主语：两臂 + 头偏置往复）
    //     //    两臂同幅度同节奏：15↔45°（**45° 全幅 @MID = 34 帧 680ms**）+ 停 313ms
    //     //      ⇒ **半步 993ms，一次完整上顶 1986ms ≈ 素材 2000ms 周期**（3 次 = 6 秒）✓
    //     //    左臂 t=0 起手走 6 步；右臂**原地等 315ms** 再走 5 步、末步停在最高位 45°
    //     //      ⇒ 左臂到顶时右臂刚起步、左臂下落时右臂在上抬 ⇒ 两臂之间有一道**涟漪**
    //     //        对上了素材里"红心一颗颗往上冒、后半段一起涌出来"的观感
    //     //    ★两臂的收尾刻意不同：左臂收在 15°（基线，安静落地）、
    //     //      右臂收在 45°（高处，靠归中把它放下来）⇒ 不"整齐收工"
    //     //    头：100↔122 做 **4 次偏置往复**（22° @MID = 17 帧 340ms + 停 848ms = 1188ms/拍）
    //     //      **全程 100~122 一路在 90° 的同一侧、绝不跨中位**
    //     //      ⇒ 按 [[shy_6x_redesign]] 的判据这是"舒服得蹭"，不是摇头否定
    //     //      （★若上板读成"一直点头/摇头"，把幅度从 22° 缩到 14° 并改用 SLOWER）
    //     .emotion_id = EMO_COMFORTABLE3,
    //     .screen_anim = "舒服(1).gif",
    //     .gif_path = "S:/gif/14_3.gif",
    //     .vib_seq = vib_light1,
    //     .vib_seq_len = 1,
    //     .audio_file = NULL,
    //     // 头：90→100 (10° @SLOWER = 11 帧 220ms) → 停 848 → 100↔122 各 340ms+848ms ×4
    //     //     → 归中 100→90 (10°@MID 160ms)
    //     //     总 = 220 + (20+848) + 4×(340+848) + 160 = **6000ms** ✓
    //     //     头一拍 1188ms vs 两臂一拍 1060ms ⇒ 不同速（承 8_x 原则③）
    //     .head = {.target = 100.0f, .speed = SERVO_SPEED_SLOWER, .seq = {
    //                                                                 {100.0f, 0, 848},               // ★先定住，再开始"蹭"
    //                                                                 {122.0f, SERVO_SPEED_MID, 848}, // 往里蹭 22°
    //                                                                 {100.0f, SERVO_SPEED_MID, 848}, // 出来
    //                                                                 {122.0f, SERVO_SPEED_MID, 848}, // 再蹭
    //                                                                 {100.0f, SERVO_SPEED_MID, 848}, // 再出来（收在这一侧）
    //                                                             },
    //              .seq_len = 5,
    //              .seq_speed = SERVO_SPEED_MID},
    //     // 左臂：15↔45°（**从基线起算 30° 行程**）@MID = 23 帧 460ms，
    //     //      加停 533ms ⇒ **半步 993ms，一次完整上顶 1986ms ≈ 素材 2000ms 周期** ✓
    //     //      6 步 = 3 次完整上顶
    //     //      总 = 20(起点 15→15) + 4×993 + 2×994 + 20(归中 15→15 零位移) = **6000ms** ✓
    //     //      （末两步 hold 比前四步多 1ms，纯粹是把 6×993.33 的小数找平）
    //     .left_arm = {.target = 15.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                                 {45.0f, 0, 533},
    //                                                                 {15.0f, 0, 533}, // 第 1 次上顶
    //                                                                 {45.0f, 0, 533},
    //                                                                 {15.0f, 0, 533}, // 第 2 次
    //                                                                 {45.0f, 0, 534},
    //                                                                 {15.0f, 0, 534}, // 第 3 次（收在基线）
    //                                                             },
    //                  .seq_len = 6,
    //                  .seq_speed = SERVO_SPEED_MID},
    //     // 右臂：**首步原地等 675ms** 再起手，只走 5 步，末步停在最高位 45° 让归中放下。
    //     //   ⚠️ 零位移 + hold_ms>0 是合法的：servo_manager.c:226 把 hold_ms 原样透传，
    //     //      底层按 hold 停住（"零位移只花 1 帧 20ms"只在 hold_ms=0 时成立）
    //     //   ⚠️ 归中用的是 **SERVO_SPEED_CENTER_ARM = FAST(10)**，不是 MID！
    //     //      45→15° = 30° ×10 = 16 帧 **320ms**（我第一版按 MID 算成 460ms，差 140ms）。
    //     //   总 = 20(起点) + (20+675)(等待) + 5×993 + 320(归中 45→15°@FAST) = **6000ms** ✓
    //     //   ★675ms 这个滞后不是"挑"出来的，是**解方程解出来的**：左臂 6 步恰好装满
    //     //     6000ms，右臂少一步、且末位在 45° 要比左臂多走一段 320ms 的归中，
    //     //     倒推滞后只能是 675ms（约 1/3 个素材周期）⇒ **明显的涟漪**。
    //     //     ⚠️ 上板若觉得"两臂错开太生硬"，把 675 调小即可（每减 332ms 少错开半步）。
    //     .right_arm = {.target = 15.0f, .speed = SERVO_SPEED_MID, .seq = {
    //                                                                  {15.0f, 0, 675}, // ★原地等，让左臂先顶
    //                                                                  {45.0f, 0, 533},
    //                                                                  {15.0f, 0, 533}, // 第 1 次上顶
    //                                                                  {45.0f, 0, 533},
    //                                                                  {15.0f, 0, 533}, // 第 2 次
    //                                                                  {45.0f, 0, 533}, // 第 3 次只顶上来，不落（归中代劳）
    //                                                              },
    //                   .seq_len = 6,
    //                   .seq_speed = SERVO_SPEED_MID},
    // },

};

// ==========================================
// 3a. 方波 beep（I2S 占位音效，真实音频接入后删除）
// ==========================================

// 880Hz 方波，半周期采样数（16kHz 采样率：16000/880/2 ≈ 9）
#define BEEP_HALF 9
#define BEEP_FULL (BEEP_HALF * 2)

/**
 * @brief 通过 I2S TX 输出 300ms 880Hz 方波（立体声 16-bit PCM，16kHz）
 */
static void play_square_wave_beep(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL || board->i2s_tx_handle == NULL)
        return;

    static const int16_t AMP = 0x1800; // ~37.5% 满幅
    int16_t one_period[BEEP_FULL * 2]; // 一个完整周期，L+R 各 int16_t
    for (int i = 0; i < BEEP_FULL; i++)
    {
        int16_t v = (i < BEEP_HALF) ? AMP : -AMP;
        one_period[i * 2] = v;
        one_period[i * 2 + 1] = v;
    }

    // 300ms × 16000 frames/s = 4800 frames
    int total_frames = 16000 * 300 / 1000;
    size_t bytes_written;
    int sent = 0;
    while (sent < total_frames)
    {
        int chunk = BEEP_FULL;
        if (sent + chunk > total_frames)
            chunk = total_frames - sent;
        i2s_channel_write(board->i2s_tx_handle,
                          one_period,
                          chunk * 2 * sizeof(int16_t),
                          &bytes_written,
                          pdMS_TO_TICKS(200));
        sent += chunk;
    }
    ESP_LOGI(TAG, "🔊 方波 beep 播放完毕");
}

// ==========================================
// 3. 震动马达控制（私有）
//    使用 BSP_MOTOR_VIB_PIN 宏，不硬编码 GPIO 号
// ==========================================

/**
 * @brief 播放一段震动序列（按段顺序：震 on_ms → 停 off_ms）
 *
 * 实现「一个情绪/GIF 可挂多个强度、时长各不相同的震动」。逐段执行，
 * 不与 GIF 帧精确对齐，触发时一口气播完即可（与 GIF 大致同期）。
 * 马达已升级 LEDC PWM（见 bsp_touch.c），强度即占空比；中间值才是真方波，
 * 0/100 为直流端点。强度统一走 bsp_motor_pulse_level()，不直接操作 GPIO。
 *
 * @param seq  震动序列数组（NULL 表示无震动，直接返回）
 * @param len  序列段数
 */
static void trigger_vibration_motor(const VibStep_t *seq, uint8_t len)
{
    if (seq == NULL || len == 0)
        return;

    for (uint8_t i = 0; i < len; i++)
    {
        // 本段震动：strength 强度持续 on_ms（on_ms=0 则跳过震动，只保留停顿）
        if (seq[i].on_ms > 0)
            bsp_motor_pulse_level(seq[i].strength, seq[i].on_ms);

        // 段间静默间隔（最后一段若 off_ms=0 则不停顿，立即结束）
        if (seq[i].off_ms > 0)
            vTaskDelay(pdMS_TO_TICKS(seq[i].off_ms));
    }
}

// ==========================================
// 4. 私有：同步执行单个情绪动作（在 worker 任务中调用）
//    调用方必须在 FreeRTOS 任务上下文，不可在中断中使用
// ==========================================

/**
 * @brief 阻塞执行指定情绪的完整动作序列（worker 内部调用）
 *
 * 执行顺序：
 *   1. 查表
 *   2. 触发屏幕动画（预留日志占位）
 *   3. 触发音频播放（预留日志占位）
 *   4. 触发震动马达
 *   5. 三轴舵机同步动作（angle_1 → angle_2，循环 count 次）
 *   6. 全轴归中至待机姿态（90°）
 */
static void interaction_play_blocking(robot_emotion_t target_emotion)
{
    const InteractionMatrix_t *cmd = NULL;
    int table_size = (int)(sizeof(g_emotion_matrix) / sizeof(g_emotion_matrix[0])); // 获取矩阵大小

    // ── 1. 查表 ───────────────────────────────────────────────────────────────
    for (int i = 0; i < table_size; i++)
    {
        if (g_emotion_matrix[i].emotion_id == target_emotion)
        {
            cmd = &g_emotion_matrix[i];
            break;
        }
    }

    if (cmd == NULL)
    {
        ESP_LOGE(TAG, "未在矩阵中找到情绪 ID: %d，请在 g_emotion_matrix 中补充！", (int)target_emotion);
        return;
    }

    ESP_LOGI(TAG, ">>> 开始执行情绪动画: %d (%s) <<<", (int)target_emotion, cmd->screen_anim);

    // ── 0. 置「情绪播放中」标志 + 清队（必须早于 GIF/震动/舵机）──────────────────
    // 置位后：主界面自动 GIF 循环立刻让位（不切图、不入舵机请求），触摸路由屏蔽新情绪。
    // ★ 必须 flush：从「触摸」到「worker 置此标志」有 50~150ms 延迟，期间自动循环已往
    //   servo_manager 队列塞了 2~3 条旧舵机请求。标志只能拦「之后」的新入队，拦不掉
    //   已堆积的旧请求。不清的话情绪舵机要排在旧请求后，表现为「GIF 已切、舵机还在做
    //   上一个动作」（现象1）。flush 清队 + 打断当前 + 归中，让情绪舵机从干净状态起步。
    atomic_store(&s_ia_playing, true);
    servo_manager_flush();

    // ── 2. 屏幕 GIF 切换（跨线程安全）─────────────────────────────────────────
    // 本任务（ia_worker）栈在 SPIRAM，禁 cache 时不能直接调 lv_gif_set_src（BUG-010）。
    // ui_request_emotion_gif 只设 pending 标记 + 唤醒 LVGL 线程的延迟 timer，由其
    // 在 LVGL 线程真正切图，安全。gif_path 为 NULL 的情绪（如无 GIF）会被内部忽略。
    ui_request_emotion_gif(cmd->gif_path);

    // ── 3. 音频播放（情绪音效 → 外挂 flash /S/voice/）──────────────────────────
    // audio_file 是情绪矩阵里的文件名（如 "doubt.mp3"），拼成绝对路径后交给
    // offline_audio_play 异步播放：它内部另起独立解码任务（栈/缓冲全在 SPIRAM），
    // 不阻塞本 ia_worker，也不碰云端对话解码实例。
    // 仲裁由 offline_audio 内部处理：对话中（非 IDLE）会被拒绝、播放中会被顶掉，
    // 与情绪触摸在 play_random_emotion 处的屏蔽策略一致，双重保险。
#if IA_EMOTION_AUDIO_ENABLE
    if (cmd->audio_file && cmd->audio_file[0] != '\0')
    {
        char audio_path[64];
        snprintf(audio_path, sizeof(audio_path), "/S/voice/%s", cmd->audio_file);
        esp_err_t aret = offline_audio_play(audio_path);
        if (aret != ESP_OK)
            ESP_LOGW(TAG, "🔊 情绪音效播放未启动: %s (ret=%d)", audio_path, aret);
        else
            ESP_LOGI(TAG, "🔊 情绪音效: %s", audio_path);
    }
#else
    /* 【2026-08-31 问题3】情绪音效已由 IA_EMOTION_AUDIO_ENABLE 关闭（暂时去除）。
     * 矩阵里的 .audio_file 字段保留不动，置回 1 即恢复。 */
    (void)cmd->audio_file;
#endif

    // ── 4. 震动马达 ──────────────────────────────────────────────────────────
    // 按情绪表配置的震动序列逐段播放（强度/时长/间隔每段独立；NULL=无震动）。
    // 低功耗（一级待机）头部专属模式下跳过震动，避免打扰。
    bool lowpower = atomic_load(&s_ia_lowpower);
    if (!lowpower)
        trigger_vibration_motor(cmd->vib_seq, cmd->vib_seq_len);

    // ── 5. 舵机三轴动作序列（入队 servo_manager，统一执行体）───────────────────
    // 把整个情绪压成【一条】序列请求，交给 servo_manager worker 执行：
    //   - 每轴走 归位角→target→seq[]→归位角，三轴【各走各的】互不等待，
    //     各轴走完自己的序列就【立即】回自己的归位角（头 90°/臂 ARM_CENTER_DEG），
    //     不再等其余两轴（★2026-09-17：此前由 worker 在三轴全走完后统一归中，
    //     导致双臂做完后干等头部摇完 8 步才一起回中，用户反馈这是错的）；
    //     （★2026-09-11 更正：此处原注释写的"angle_1↔angle_2 往返
    //     count 次"是 2026-09-04 之前旧结构的语义，改序列结构时没跟着改）；
    //   - 带 s_ia_d1_2_sem，执行完（正常 或 被 servo_manager_flush 打断）后 give；
    //   - 统一执行体保证情绪/自动循环/待机互斥，且进功能盘 flush 能打断本动作。
    // 情绪表 ActionSeq_t 是绝对角度，1:1 填入请求，无换算误差（plan R5）。
    //
    // 低功耗模式下手臂【seq_len 与 speed 一起清零】：seq_len=0 只是不走序列，
    // 若 speed 还留着，手臂仍会走"到终点"那一步（★2026-09-17 起还会再走一步
    // 归中，见 servo_exec_seq 的 ③）；两者都清才真正钉住不动
    // （见下方 if (lowpower) 块）。头部不受影响，情绪头部动作照常播放。
    // ★seq 位填 {{0}}（不是 {0}）：seq 现在是结构体数组，多一层花括号才与类型
    //   形状匹配，否则 -Wmissing-braces 会报警告。实际内容由下面的循环填入。
    servo_seq_request_t preq = {
        .head = {cmd->head.target, cmd->head.speed, {{0}}, cmd->head.seq_len, cmd->head.seq_speed},
        .l_arm = {cmd->left_arm.target, cmd->left_arm.speed, {{0}}, (uint8_t)(lowpower ? 0 : cmd->left_arm.seq_len), cmd->left_arm.seq_speed},
        .r_arm = {cmd->right_arm.target, cmd->right_arm.speed, {{0}}, (uint8_t)(lowpower ? 0 : cmd->right_arm.seq_len), cmd->right_arm.seq_speed},
    };
    // 序列逐步拷贝：配置侧 ActionSeqStep_t → 执行侧 servo_seq_step_t。
    //
    // ★2026-09-11 由 memcpy 改为逐字段拷贝：两者是【不同类型】的结构体，
    //   memcpy 依赖"两边内存布局恰好一致"这个隐含前提，哪天有人只给其中一边
    //   加了字段就会静默错位（角度读成速度），且编译器不会报错。逐字段赋值
    //   让编译器替我们保证正确性，代价只是几行代码。
    // ★2026-09-18 hold_ms 也在此逐字段透传（新增字段时正是靠这条约定避免了
    //   "只给一边加字段导致静默错位"，见上方注释）。
    for (int s = 0; s < ACTION_SEQ_MAX_STEPS; s++)
    {
        preq.head.seq[s].angle = cmd->head.seq[s].angle;
        preq.head.seq[s].speed = cmd->head.seq[s].speed;
        preq.head.seq[s].hold_ms = cmd->head.seq[s].hold_ms;
        preq.l_arm.seq[s].angle = cmd->left_arm.seq[s].angle;
        preq.l_arm.seq[s].speed = cmd->left_arm.seq[s].speed;
        preq.l_arm.seq[s].hold_ms = cmd->left_arm.seq[s].hold_ms;
        preq.r_arm.seq[s].angle = cmd->right_arm.seq[s].angle;
        preq.r_arm.seq[s].speed = cmd->right_arm.seq[s].speed;
        preq.r_arm.seq[s].hold_ms = cmd->right_arm.seq[s].hold_ms;
    }
    // 低功耗：手臂不参与（speed 也清零，否则仍会走"到终点"那一步）
    if (lowpower)
    {
        preq.l_arm.speed = 0;
        preq.r_arm.speed = 0;
    }

    // 清掉可能残留的旧 d1_2 信号（防上一轮超时遗留导致本轮 take 立即返回）
    xSemaphoreTake(s_ia_d1_2_sem, 0);

    if (servo_manager_submit_seq_notify(&preq, s_ia_d1_2_sem) == ESP_OK)
    {
        // 阻塞等待舵机动作执行完毕（含归中）。10s 超时兜底：万一被 flush 清队
        // 丢弃了请求导致 give 不发生，也不会永久卡死（plan R4）。
        if (xSemaphoreTake(s_ia_d1_2_sem, pdMS_TO_TICKS(10000)) != pdTRUE)
            ESP_LOGW(TAG, "情绪舵机完成等待超时（可能被 flush 打断），继续");
    }
    else
    {
        ESP_LOGW(TAG, "情绪舵机请求入队失败（队列满？），跳过本次动作");
    }

    ESP_LOGI(TAG, ">>> 情绪动作执行完毕: %d <<<", (int)target_emotion);

    // ── 6. 清「情绪播放中」标志 + 恢复主界面自动 GIF 循环 ──────────────────────
    // 先 flush 再清标志再恢复：清掉情绪期间可能残留的旧舵机请求（理论上已空，
    // 但兜底防现象2「恢复后 GIF 切好几遍舵机才动」）。flush 必须在 ui_resume 之前——
    // 否则会把 resume 刚入队的新舵机请求一起清掉。
    //
    // ★远程控制活跃时【必须跳过】(2026-08-05)：本情绪动作往往正是被远程指令打断的那条，
    //   打断后 rc_worker 已经拿到通道锁开始把舵机送往用户指定角度。此处 flush 内部会调
    //   bsp_servo_request_abort()，把 rc 刚清干净的打断标志重新置上 → rc 的插值走两步就
    //   break，手臂停在半路（实测 steps=2 aborted=1 停在 35.0°，目标 0°）。
    //   而正常收尾时 resume_loop 早于 rc 运动完成，两者不重叠，故表现为「时灵时不灵」。
    //   远程控制期间也不需要这个归中兜底：rc 马上要把舵机放到用户要的位置，归中纯属多余。
    if (!remote_control_is_active())
        servo_manager_flush();
    atomic_store(&s_ia_playing, false);
    ui_resume_main_gif_loop();
}

/**
 * @brief 阻塞执行一个自定义动作（GIF + 三轴舵机 + 震动），不查情绪矩阵（worker 内部调用）
 *
 * 与 interaction_play_blocking 同样的执行骨架（置标志+flush → 切图 → 震动 → 舵机 → 等
 * 完成 → flush 归中 → 清标志），但：
 *   - 切图按 act->is_state_gif 选高优先级状态切图 / 普通情绪切图（见 D.1）；
 *   - act->keep_screen=true 时【跳过】结尾的 ui_resume_main_gif_loop，屏幕停在该 GIF
 *     （对话中状态动作播完不应跳回待机循环）。
 */
static void interaction_play_custom_blocking(const ia_custom_action_t *act)
{
    if (act == NULL)
        return;

    // 置「播放中」+ 清舵机队列（与情绪同样的起步姿态，防旧请求残留）
    // is_idle=true 即「空闲动作」（低优先级，可被触摸情绪 flush 打断）；
    // 对话状态动作 is_idle=false（不可被打断，与情绪同级）。is_idle 与 keep_screen 解耦。
    atomic_store(&s_ia_is_idle, act->is_idle);
    atomic_store(&s_ia_playing, true);
    // ESP_LOGI("GIFDBG", "custom动作 开始 idle=%d gif=%s keep=%d",
    //          act->is_idle, act->gif_path ? act->gif_path : "(NULL)", act->keep_screen);
    /* 【堆探针·2026-07-29 已验证无问题，暂停用】
     * 实测这四个探针（进入前/flush后/切图后/震动后）在事故日志里【全部同时报同一地址】，
     * 且「进入前」就已经报 → 说明本函数只是撞上别处留下的损坏，自身不是凶手。
     * 整条 GIF+舵机+震动链路据此排除嫌疑。需再查时取消注释即可。 */
    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@custom进入前(上一动作遗留)");
    servo_manager_flush();
    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@servo_manager_flush()之后");

    // 切图（跨线程安全）：状态动作走高优先级 ui_request_state_gif（is_state=true），
    // 否则会被对话中 main_gif_switch_timer_cb 的「丢弃情绪切图」兜底误伤（D.1）。
    if (act->gif_path != NULL && act->gif_path[0] != '\0')
    {
        if (act->is_state_gif)
            ui_request_state_gif(act->gif_path);
        else
            ui_request_emotion_gif(act->gif_path);
    }
    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@切图之后");   // 【已验证无问题，暂停用】

    // 震动（逐段阻塞播放；NULL/0 直接返回）
    trigger_vibration_motor(act->vib_seq, act->vib_seq_len);
    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@震动之后");   // 【已验证无问题，暂停用】

    // 三轴舵机绝对角度并行动作（与情绪同一执行体，执行完 worker 统一归中 90°）
    servo_abs_parallel_request_t preq = {
        .head = {act->head.angle_1, act->head.angle_2, act->head.speed, act->head.count},
        .l_arm = {act->left_arm.angle_1, act->left_arm.angle_2, act->left_arm.speed, act->left_arm.count},
        .r_arm = {act->right_arm.angle_1, act->right_arm.angle_2, act->right_arm.speed, act->right_arm.count},
    };
    xSemaphoreTake(s_ia_d1_2_sem, 0); // 清残留 d1_2 信号
    if (servo_manager_submit_abs_parallel_notify(&preq, s_ia_d1_2_sem) == ESP_OK)
    {
        if (xSemaphoreTake(s_ia_d1_2_sem, pdMS_TO_TICKS(10000)) != pdTRUE)
            ESP_LOGW(TAG, "状态动作舵机完成等待超时（可能被 flush 打断），继续");
    }
    else
    {
        ESP_LOGW(TAG, "状态动作舵机请求入队失败（队列满？），跳过本次动作");
    }

    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@舵机动作之后");  // 【已验证无问题，暂停用】
    /* ia_worker 栈在 PSRAM（见 interaction_manager_init 的 MALLOC_CAP_SPIRAM），
     * 溢出踩不到内部 SRAM 堆；实测水位剩 7676B/8192B，极宽裕 → 排除。 */
    // WARN_TASK_STACK_LOW("HEAPCHK", NULL, 1024);
    // ★远程控制活跃时跳过：理由同 interaction_play_blocking 收尾处的详细注释——
    //   本 flush 会重新置上打断标志，打断 rc_worker 正在进行的接管运动。
    //   空闲动作走的正是本函数，故「空闲时下发指令也会停住」同源于此。
    if (!remote_control_is_active())
        servo_manager_flush(); // 归中（含打断兜底）
    atomic_store(&s_ia_is_idle, false);
    atomic_store(&s_ia_playing, false);
    // ESP_LOGI("GIFDBG", "custom动作 结束 keep=%d → %s", act->keep_screen,
    //          act->keep_screen ? "停在GIF" : "恢复循环");
    if (!act->keep_screen)
        ui_resume_main_gif_loop(); // 仅非 keep_screen 才恢复待机循环；对话中 keep_screen=true 停在状态 GIF（D.3）
}

// ==========================================
// 5. Worker 任务：串行消费动作队列（情绪 / 自定义动作）
// ==========================================

/**
 * @brief interaction worker 主循环
 *
 * 永久阻塞等待队列，每次取出一个 robot_emotion_t 并执行完整动作序列。
 * 串行执行保证动作不重叠，无需外部加锁。
 */
static void interaction_worker_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    ia_request_t req;
    for (;;)
    {
        // 无限等待，收到请求后按类型分发执行（阻塞期间不占 CPU）
        if (xQueueReceive(s_ia_queue, &req, portMAX_DELAY) == pdTRUE)
        {
            // OTA 升级期间：丢弃一切请求，不切 GIF / 不驱动舵机 / 不震动
            if (s_ia_stopped_for_ota)
                continue;
            if (req.type == IA_REQ_EMOTION)
                interaction_play_blocking(req.u.emo);
            else /* IA_REQ_CUSTOM */
                interaction_play_custom_blocking(&req.u.custom);
        }
    }
}

// ==========================================
// 6. 公开 API
// ==========================================

/**
 * @brief 初始化 interaction_manager（创建队列 + worker 任务）
 *
 * 必须在 bsp_board_servo_init() 之后、首次调用 ui_interaction_play() 之前调用。
 */
esp_err_t interaction_manager_init(void)
{
    if (s_ia_inited)
        return ESP_OK;

    // ── 创建情绪请求队列 ──────────────────────────────────────────────────────
    s_ia_queue = xQueueCreate(INTERACTION_QUEUE_LEN, sizeof(ia_request_t));
    if (s_ia_queue == NULL)
    {
        ESP_LOGE(TAG, "创建 interaction 队列失败，内存不足!");
        return ESP_ERR_NO_MEM;
    }

    // ── 创建舵机动作完成信号量（worker 等待 servo_manager 执行完毕用）──────────
    s_ia_d1_2_sem = xSemaphoreCreateBinary();
    if (s_ia_d1_2_sem == NULL)
    {
        ESP_LOGE(TAG, "创建 interaction 完成信号量失败，内存不足!");
        vQueueDelete(s_ia_queue);
        s_ia_queue = NULL;
        return ESP_ERR_NO_MEM;
    }

    // ── 创建 worker 任务（栈分配到 SPIRAM，节省内部 SRAM）────────────────────
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        interaction_worker_task,
        "ia_worker",
        INTERACTION_TASK_STACK,
        NULL,
        INTERACTION_TASK_PRIO,
        &s_ia_worker,
        tskNO_AFFINITY,                       // 不绑定 CPU 核心，调度器自由分配
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); // 栈分配在 SPIRAM

    if (ret != pdPASS)
    {
        vQueueDelete(s_ia_queue);
        s_ia_queue = NULL;
        ESP_LOGE(TAG, "创建 interaction worker 任务失败，内存不足!");
        return ESP_ERR_NO_MEM;
    }
    PRINT_TASK_CREATED(TAG, "ia_worker", INTERACTION_TASK_STACK, 0); // 栈在PSRAM

    s_ia_inited = true;
    ESP_LOGI(TAG, "interaction_manager 初始化完成（队列深度=%d）", INTERACTION_QUEUE_LEN);
    return ESP_OK;
}

/**
 * @brief 非阻塞触发情绪动作（将 emotion_id 入队后立即返回）
 *
 * 线程安全，可从任意任务调用。
 * 若队列已满（INTERACTION_QUEUE_LEN），新请求被丢弃并打印警告。
 */
void ui_interaction_play(robot_emotion_t target_emotion)
{
    if (!s_ia_inited || s_ia_queue == NULL)
    {
        ESP_LOGE(TAG, "interaction_manager 未初始化，调用 interaction_manager_init() 后再使用!");
        return;
    }

    // 触摸情绪优先级高于空闲动作：若当前正在播空闲动作，先 flush 打断它的舵机，
    // 让 worker 的舵机等待（s_ia_d1_2_sem）立即返回、空闲动作迅速收尾，紧接着出队本情绪。
    // （空闲动作震动很短，flush 主要打断耗时的舵机摆动；对话状态动作不置 idle，不受影响。）
    if (interaction_is_idle_action())
    {
        ESP_LOGI(TAG, "触摸情绪打断当前空闲动作");
        servo_manager_flush();
    }

    // 非阻塞入队（timeout=0），队满则丢弃
    ia_request_t req = {.type = IA_REQ_EMOTION, .u.emo = target_emotion};
    if (xQueueSend(s_ia_queue, &req, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "interaction 队列已满，丢弃情绪 %d", (int)target_emotion);
    }
}

/**
 * @brief 非阻塞触发一个自定义动作（GIF + 三轴舵机 + 震动），不查情绪矩阵
 *
 * 将动作整体值拷贝入队后立即返回，由 worker 串行执行（见 interaction_play_custom_blocking）。
 * 线程安全，可从任意任务调用；队列满时丢弃并打印警告。
 */
void ui_interaction_play_custom(const ia_custom_action_t *act)
{
    if (act == NULL)
    {
        ESP_LOGE(TAG, "custom 动作为空，忽略请求!");
        return;
    }
    if (!s_ia_inited || s_ia_queue == NULL)
    {
        // 开机早期 GIF 自动轮播已启动，但 interaction_manager_init() 要等舵机硬件/
        // servo_manager 就绪后才执行（见 application_init 步骤 8~10），中间这段时间
        // 的 custom 请求属预期内丢弃，不应按 ERROR 刷屏。
        ESP_LOGD(TAG, "interaction_manager 尚未初始化，忽略 custom 请求");
        return;
    }

    // 整体值拷贝入队（custom 内 gif_path/vib_seq 是指针，须指向 static const，由调用方保证）
    ia_request_t req = {.type = IA_REQ_CUSTOM, .u.custom = *act};
    if (xQueueSend(s_ia_queue, &req, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "interaction 队列已满，丢弃 custom 动作");
    }
}

void interaction_flush_queue(void)
{
    // 只清「未执行」的存量请求；正在执行的那条由 servo_manager_flush 打断（二者配合）。
    if (s_ia_inited && s_ia_queue != NULL)
        xQueueReset(s_ia_queue);
}

void interaction_stop_for_ota(void)
{
    // ① 置停止标志：worker 此后取到任何请求都直接丢弃，不再切 GIF/舵机/震动
    s_ia_stopped_for_ota = true;
    // ② 清空队列里未执行的存量请求
    if (s_ia_inited && s_ia_queue != NULL)
        xQueueReset(s_ia_queue);
    // ③ 打断【正在执行】的那条舵机动作（含归中兜底），让当前动作立即收尾
    servo_manager_flush();
    ESP_LOGW(TAG, "interaction 已为 OTA 停止（GIF/舵机/震动全部静默）");
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 调试用：GIF + 舵机适配循环播放任务（2026-09-16 新增）
 *
 * 【用途】逐个情绪调试「GIF + 三轴舵机」的适配性。调试时只需在 g_emotion_matrix
 *   里注释/增删条目，数组里留几条就轮着播几条（留一条就循环播那一条），
 *   无需改任何枚举/宏，也无需碰 MQTT / session 等上层链路。
 *
 * 【为什么直接调 interaction_play_blocking 而非走队列】本任务与 ia_worker 同角色，
 *   串行阻塞执行每一条情绪（切 GIF → 震动 → 舵机序列 → 归中），节奏由本任务控制；
 *   栈在 SPIRAM，与 ia_worker 一致，blocking 内不直接擦写 flash，无 BUG-039 风险。
 * ═══════════════════════════════════════════════════════════════════════════ */
#define INTERACTION_DEMO_GAP_MS 000         ///< 每轮情绪之间停顿（ms），方便肉眼观察适配效果
#define INTERACTION_DEMO_INIT_DELAY_MS 3000 ///< 启动后先等 UI/主循环就绪再开播

/**
 * @brief demo 任务主体：遍历 g_emotion_matrix，循环播放数组里现有的每一条情绪
 */
static void interaction_demo_task(void *arg)
{
    (void)arg;
    vTaskDelay(pdMS_TO_TICKS(INTERACTION_DEMO_INIT_DELAY_MS)); // 等开机 logo/UI 就绪

    /* ★demo 独占（2026-09-17 根治空闲态抢占）：置 OTA 停止标志，让 worker 丢弃一切
     * 队列请求（尤其主界面空闲轮播投递的空闲动作）。否则 worker 与 demo 并发执行，
     * 且在空闲动作收尾 atomic_store(s_ia_playing,false) 清掉 demo 的锁 → "偶尔空闲态抢占"。
     * demo task 直接调 interaction_play_blocking（不走队列），不受此标志影响。 */
    s_ia_stopped_for_ota = true;

    for (;;)
    {
        int n = (int)(sizeof(g_emotion_matrix) / sizeof(g_emotion_matrix[0]));
        for (int i = 0; i < n; i++)
        {
            /* ★锁住主界面「空闲 GIF 自动轮播」（2026-09-17 修复抢占）：
             * 不锁的话，主循环 main_gif_switch_timer_cb 会在轮间停顿(GAP_MS)期间
             * 切它自己的空闲 GIF + 投递空闲舵机动作，与 demo 情绪抢占屏幕和舵机
             * （实测"好几个抢占"）。
             *   - interaction_flush_queue()：清掉 worker 队列里已积压的空闲动作；
             *   - atomic_store(&s_ia_playing, true)：主循环 timer_cb 的「自动随机循环」
             *     分支判 interaction_is_playing()==true 即 return，不再切空闲图/投空闲动作。
             *   ★不拦 demo 自己的情绪 GIF：情绪切图走 timer_cb 的 pending_path 分支
             *     （在 playing 判断之前），照常切。 */
            interaction_flush_queue();
            atomic_store(&s_ia_playing, true);

            ESP_LOGI(TAG, "[DEMO] 播放情绪 %d: %s (GIF=%s)",
                     (int)g_emotion_matrix[i].emotion_id,
                     g_emotion_matrix[i].screen_anim,
                     g_emotion_matrix[i].gif_path ? g_emotion_matrix[i].gif_path : "(无)");
            interaction_play_blocking(g_emotion_matrix[i].emotion_id);

            /* interaction_play_blocking 收尾会清 s_ia_playing=false 并 ui_resume_main_gif_loop，
             * 这里立即重新锁 true，堵住轮间停顿期间主循环空闲轮播的插队。 */
            atomic_store(&s_ia_playing, true);
            vTaskDelay(pdMS_TO_TICKS(INTERACTION_DEMO_GAP_MS));
        }
    }
}

void interaction_demo_start(void)
{
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        interaction_demo_task,
        "ia_demo",
        INTERACTION_TASK_STACK,
        NULL,
        INTERACTION_TASK_PRIO,
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
        ESP_LOGE(TAG, "创建 interaction demo 任务失败，内存不足!");
    else
        PRINT_TASK_CREATED(TAG, "ia_demo", INTERACTION_TASK_STACK, 0);
}
