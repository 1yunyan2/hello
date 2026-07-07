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

static const char *TAG = "INTERACTION";

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
static bool s_ia_inited = false;        // 初始化完成

// ==========================================
// 1. 数据结构：严格对齐情绪矩阵表头
// ==========================================

/* ActionStep_t / VibStep_t 已移到 interaction.h 暴露（供 ui_port 构造状态动作表）。 */

/* ── 命名震动序列（对应情绪队列表 D 列的 10 种震动类型）────────────────────
 * 强度用中间占空比（真 PWM 方波，0/100 为直流端点）。这里是初版手感值，
 * 烧录后用示波器/体感逐项微调 strength/on_ms/off_ms 即可，不必改结构。
 * 多个情绪可复用同一序列（同震动类型）。 */
static const VibStep_t vib_short2[] = {{70, 50, 50}, {70, 50, 0}};                  // 短促震动 2 次
static const VibStep_t vib_light1[] = {{50, 60, 0}};                                // 轻微震动 1 次
static const VibStep_t vib_light2[] = {{50, 60, 60}, {50, 60, 0}};                  // 轻微震动 2 次
static const VibStep_t vib_short_strong2[] = {{85, 50, 50}, {85, 50, 0}};           // 短促强震动 2 次
static const VibStep_t vib_strong2[] = {{100, 90, 70}, {100, 90, 0}};               // 强震动 2 次
static const VibStep_t vib_fast_cont[] = {{80, 30, 25}, {80, 30, 25}, {80, 30, 25}, // 连续/快速连续震动
                                          {80, 30, 25},
                                          {80, 30, 0}};
static const VibStep_t vib_slow_long1[] = {{45, 400, 0}};                  // 缓慢长震动 1 次
static const VibStep_t vib_soft_cont[] = {{35, 500, 0}};                   // 持续轻柔震动
static const VibStep_t vib_intermittent[] = {{45, 40, 120}, {45, 40, 120}, // 轻微间断震动
                                             {45, 40, 120},
                                             {45, 40, 0}};
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
    ActionStep_t head;          ///< 头部舵机（CH_HEAD）
    ActionStep_t left_arm;      ///< 左臂舵机（CH_L_ARM）
    ActionStep_t right_arm;     ///< 右臂舵机（CH_R_ARM）
} InteractionMatrix_t;

// ==========================================
// 2. 情绪动作矩阵表（14 种全覆盖）
//
// 角度速查（实际软限位见 bsp_servo.c: HEAD/L_ARM/R_ARM _MIN/MAX_ANGLE，均为 [0, 180]）：
//   中位 90°，本表数值为绝对角度
// ==========================================
static const InteractionMatrix_t g_emotion_matrix[] = {
    //! 后续将screen_anim和audio_file字段传给UI层和音频层，目前先占位空字符串，避免未初始化的垃圾值导致不可预期的行为。
    // ── 0: 开心 ──────────────────────────────────────────────────────────────
    // 快速左右摇头3次 + 双臂前后摆2次 + 2次短促震动
    {
        .emotion_id = EMO_HAPPY,
        .screen_anim = "anim_happy_stars",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(活泼)；真GIF就绪后改为 happy.gif
        .vib_seq = vib_short2,        // 短促震动 2 次
        .vib_seq_len = 2,
        .audio_file = "laugh_short.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_FAST, 3},    // 【调试】头部改最大值 左170→右10，快速×3
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 2}, // 前20→后20，中速×2
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 2},
    },

    // ── 1: 好奇 ──────────────────────────────────────────────────────────────
    // 缓慢左右侧头2次 + 双臂前举1次 + 1次轻震动
    {
        .emotion_id = EMO_CURIOUS,
        .screen_anim = "anim_curious_q",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(好奇)；真GIF就绪后改为 curious.gif
        .vib_seq = vib_light1,
        .vib_seq_len = 1,
        .audio_file = "doubt.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_SLOW, 2},     // 【调试】头部改最大值 左170→右10，慢×2
        .left_arm = {120.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 前30→归中，慢×1
        .right_arm = {120.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 2: 傲娇 ──────────────────────────────────────────────────────────────
    // 头部轻偏左1次（不回来，等归中）+ 双臂快速后收1次 + 无震动
    {
        .emotion_id = EMO_TSUNDERE,
        .screen_anim = "anim_tsundere",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(好奇系)；真GIF就绪后改为 tsundere.gif
        // 无震动：vib_seq 留空（NULL）
        .audio_file = "hmph.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_SLOW, 1}, // 【调试】头部改最大值 左170→右10，极慢×1
        .left_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},   // 后30→归中，快×1
        .right_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 3: 怕痒 ──────────────────────────────────────────────────────────────
    // 极速抖头5次 + 双臂快速前后摆3次 + 连续震动
    {
        .emotion_id = EMO_TICKLISH,
        .screen_anim = "anim_ticklish",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(活泼)；真GIF就绪后改为 ticklish.gif
        .vib_seq = vib_fast_cont,     // 连续快速震动
        .vib_seq_len = 5,
        .audio_file = "ticklish.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_FAST, 5}, // 【调试】头部改最大值 左170→右10，极快×5
        .left_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 3},  // 前25→后25，快×3
        .right_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 3},
    },

    // ── 4: 犯困 ──────────────────────────────────────────────────────────────
    // 极慢大幅侧头1次 + 双臂缓缓下垂1次 + 无震动
    {
        .emotion_id = EMO_SLEEPY,
        .screen_anim = "anim_sleepy",
        .gif_path = "S:/gif/1_1.gif", // 占位→1_1(犯困)；真GIF就绪后改为 sleepy.gif
        .vib_seq = vib_slow_long1,    // 缓慢长震动 1 次
        .vib_seq_len = 1,
        .audio_file = "yawn.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_SLOW, 1},    // 【调试】头部改最大值 左170→右10，极慢×1
        .left_arm = {70.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1}, // 后20（下垂）→归中
        .right_arm = {70.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
    },

    // ── 5: 委屈 ──────────────────────────────────────────────────────────────
    // 头部缓慢轻偏1次 + 双臂小幅后收（内缩）1次 + 无震动
    {
        .emotion_id = EMO_GRIEVED,
        .screen_anim = "anim_grieved",
        .gif_path = "S:/gif/1_1.gif", // 占位→1_1(低落)；真GIF就绪后改为 grieved.gif
        .vib_seq = vib_intermittent,  // 轻微间断震动
        .vib_seq_len = 4,
        .audio_file = "sob.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_SLOW, 1},    // 【调试】头部改最大值 左170→右10，慢×1
        .left_arm = {65.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 后25→归中，慢×1
        .right_arm = {65.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 6: 舒服 ──────────────────────────────────────────────────────────────
    // 慢速轻摇头2次 + 双臂微微展开1次 + 1次轻震动
    {
        .emotion_id = EMO_COMFORTABLE,
        .screen_anim = "anim_comfortable",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(舒缓)；真GIF就绪后改为 comfortable.gif
        .vib_seq = vib_soft_cont,     // 持续轻柔震动
        .vib_seq_len = 1,
        .audio_file = "sigh_happy.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_SLOW, 2},     // 【调试】头部改最大值 左170→右10，慢×2
        .left_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 前10→归中，慢×1
        .right_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 7: 撒娇 ──────────────────────────────────────────────────────────────
    // 头部中速倾斜往返2次 + 双臂中速上举2次 + 2次短促震动
    {
        .emotion_id = EMO_ACT_CUTE,
        .screen_anim = "anim_act_cute",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(舒缓)；真GIF就绪后改为 act_cute.gif
        .vib_seq = vib_light2,        // 轻微震动 2 次
        .vib_seq_len = 2,
        .audio_file = "cute.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_MID, 2},     // 【调试】头部改最大值 左170→右10，中速×2
        .left_arm = {120.0f, 90.0f, SERVO_SPEED_MID, 2}, // 前30→归中，中速×2
        .right_arm = {120.0f, 90.0f, SERVO_SPEED_MID, 2},
    },

    // ── 8: 生气 ──────────────────────────────────────────────────────────────
    // 快速大幅摇头3次 + 双臂用力前摆2次 + 2次短促震动
    {
        .emotion_id = EMO_ANGRY,
        .screen_anim = "anim_angry",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(强烈)；真GIF就绪后改为 angry.gif
        .vib_seq = vib_strong2,       // 强震动 2 次
        .vib_seq_len = 2,
        .audio_file = "angry.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_FAST, 3},     // 【调试】头部改最大值 左170→右10，快×3
        .left_arm = {125.0f, 70.0f, SERVO_SPEED_FAST, 2}, // 前35→后20，快×2
        .right_arm = {125.0f, 70.0f, SERVO_SPEED_FAST, 2},
    },

    // ── 9: 害羞 ──────────────────────────────────────────────────────────────
    // 头部缓缓轻低垂1次 + 双臂小幅前举（遮脸感）1次 + 1次轻震动
    {
        .emotion_id = EMO_SHY,
        .screen_anim = "anim_shy",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(舒缓)；真GIF就绪后改为 shy.gif
        .vib_seq = vib_light1,        // 轻微震动 1 次
        .vib_seq_len = 1,
        .audio_file = "shy.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_SLOW, 1},     // 【调试】头部改最大值 左170→右10，慢×1
        .left_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 前15→归中，慢×1
        .right_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 10: 惊喜 ─────────────────────────────────────────────────────────────
    // 头部快速左右摆1次 + 双臂快速上扬1次 + 1次轻震动
    {
        .emotion_id = EMO_SURPRISED,
        .screen_anim = "anim_surprised",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(强烈)；真GIF就绪后改为 surprised.gif
        .vib_seq = vib_short_strong2, // 短促强震动 2 次
        .vib_seq_len = 2,
        .audio_file = "surprise.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_FAST, 1},     // 【调试】头部改最大值 左170→右10，快×1
        .left_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1}, // 前35→归中，快×1
        .right_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 11: 慵懒 ─────────────────────────────────────────────────────────────
    // 极慢小幅侧头1次 + 双臂极慢微垂1次 + 无震动
    {
        .emotion_id = EMO_SLUGGISH,
        .screen_anim = "anim_sluggish",
        .gif_path = "S:/gif/1_1.gif", // 占位→1_1(困倦)；真GIF就绪后改为 sluggish.gif
        .vib_seq = vib_slow_long1,    // 缓慢长震动 1 次
        .vib_seq_len = 1,
        .audio_file = "lazy.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_SLOW, 1},    // 【调试】头部改最大值 左170→右10，极慢×1
        .left_arm = {75.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1}, // 后15→归中，极慢×1
        .right_arm = {75.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
    },

    // ── 12: 治愈 ─────────────────────────────────────────────────────────────
    // 缓慢温柔摇头2次 + 双臂轻柔微展2次 + 1次轻震动
    {
        .emotion_id = EMO_HEALING,
        .screen_anim = "anim_healing",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(舒缓)；真GIF就绪后改为 healing.gif
        .vib_seq = vib_soft_cont,     // 持续轻柔震动
        .vib_seq_len = 1,
        .audio_file = "healing.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_SLOW, 2},     // 【调试】头部改最大值 左170→右10，慢×2
        .left_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 2}, // 前10→归中，慢×2
        .right_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 2},
    },

    // ── 13: 兴奋 ─────────────────────────────────────────────────────────────
    // 快速大幅摇头4次 + 双臂大幅前后摆3次 + 2次短促震动
    {
        .emotion_id = EMO_EXCITED,
        .screen_anim = "anim_excited",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(活泼)；真GIF就绪后改为 excited.gif
        .vib_seq = vib_fast_cont,     // 快速连续震动
        .vib_seq_len = 5,
        .audio_file = "excited.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_FAST, 4},     // 【调试】头部改最大值 左170→右10，快×4
        .left_arm = {125.0f, 60.0f, SERVO_SPEED_FAST, 3}, // 前35→后30，快×3
        .right_arm = {125.0f, 60.0f, SERVO_SPEED_FAST, 3},
    },
    // ── 14: 害羞蹭蹭 ─────────────────────────────────────────────────────────
    // 表格要求: 轻微震动2次 | 头:极慢左15右15(4次) | 左/右臂:缓慢前15(1次)
    {
        .emotion_id = EMO_SHY_RUB,
        .screen_anim = "anim_shy_rub",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(舒缓)；真GIF就绪后改为 shy_rub.gif
        .vib_seq = vib_light2,        // 轻微震动 2 次
        .vib_seq_len = 2,
        .audio_file = "shy_rub.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_SLOW, 4}, // 【调试】头部改最大值 左170→右10
        .left_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 15: 舒服到打滚 ───────────────────────────────────────────────────────
    // 表格要求: 持续轻柔震动 | 头:缓慢左30右30(2次) | 左/右臂:极慢后20前10(2次)
    {
        .emotion_id = EMO_COMFORTABLE_ROLL,
        .screen_anim = "anim_comfortable_roll",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(舒缓)；真GIF就绪后改为 comfortable_roll.gif
        .vib_seq = vib_soft_cont,     // 持续轻柔震动
        .vib_seq_len = 1,
        .audio_file = "purr.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_SLOW, 2}, // 【调试】头部改最大值 左170→右10
        .left_arm = {70.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 2},
        .right_arm = {70.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 2},
    },

    // ── 16: 傲娇求摸 ─────────────────────────────────────────────────────────
    // 表格要求: 无震动 | 头:极慢左15右15(3次) | 左/右臂:快速后30(1次)
    {
        .emotion_id = EMO_TSUNDERE_PET,
        .screen_anim = "anim_tsundere_pet",
        .gif_path = "S:/gif/1_3.gif", // 占位→1_3(好奇系)；真GIF就绪后改为 tsundere_pet.gif
        // 无震动：vib_seq 留空（NULL）
        .audio_file = "hmph_pet.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_SLOW, 3}, // 【调试】头部改最大值 左170→右10
        .left_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
        .right_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 17: 慵懒瘫坐 ─────────────────────────────────────────────────────────
    // 表格要求: 缓慢长震动1次 | 头:极慢左10右10(1次) | 左/右臂:极慢前10(1次)
    {
        .emotion_id = EMO_SLUGGISH_SIT,
        .screen_anim = "anim_sluggish_sit",
        .gif_path = "S:/gif/1_1.gif", // 占位→1_1(困倦)；真GIF就绪后改为 sluggish_sit.gif
        .vib_seq = vib_slow_long1,    // 缓慢长震动 1 次
        .vib_seq_len = 1,
        .audio_file = "lazy_sit.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_SLOW, 1}, // 【调试】头部改最大值 左170→右10
        .left_arm = {100.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
        .right_arm = {100.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
    },

    // ── 18: 惊喜抱抱 ─────────────────────────────────────────────────────────
    // 表格要求: 短促强震动1次 | 头:快速左15(1次) | 左/右臂:快速前35(1次)
    {
        .emotion_id = EMO_SURPRISED_HUG,
        .screen_anim = "anim_surprised_hug",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(强烈)；真GIF就绪后改为 surprised_hug.gif
        .vib_seq = vib_short_strong2, // 短促强震动 2 次（惊喜系）
        .vib_seq_len = 2,
        .audio_file = "hug_me.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_FAST, 1}, // 【调试】头部改最大值 左170→右10
        .left_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1},
        .right_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 19: 怕痒到扭动 ───────────────────────────────────────────────────────
    // 表格要求: 连续快速震动 | 头:极快左20右20(5次) | 左/右臂:快速前25后25(4次)
    {
        .emotion_id = EMO_TICKLISH_WIGGLE,
        .screen_anim = "anim_ticklish_wiggle",
        .gif_path = "S:/gif/1_2.gif", // 占位→1_2(活泼)；真GIF就绪后改为 ticklish_wiggle.gif
        .vib_seq = vib_fast_cont,     // 连续快速震动
        .vib_seq_len = 5,
        .audio_file = "wiggle_laugh.mp3",
        .head = {170.0f, 10.0f, SERVO_SPEED_VERY_FAST, 5}, // 【调试】头部改最大值 左170→右10
        .left_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 4},
        .right_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 4},
    }};

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

    // ── 3. 音频播放（方波占位，真实文件接入后f替换）──────────────────────────
    // 注：lv_gif_set_src 会访问 SPIFFS（SPI flash），ia_worker 栈在 SPIRAM，
    //     禁用 cache 时 SPIRAM 不可访问，因此不能在此任务中调用屏幕动画。
    // TODO: audio_player_play_file(cmd->audio_file);
    ESP_LOGI(TAG, "🔊 音效槽: %s（当前使用方波占位）", cmd->audio_file);
    // play_square_wave_beep(); // 播放方波占位音频

    // ── 4. 震动马达 ──────────────────────────────────────────────────────────
    // 按情绪表配置的震动序列逐段播放（强度/时长/间隔每段独立；NULL=无震动）。
    // 低功耗（一级待机）头部专属模式下跳过震动，避免打扰。
    bool lowpower = atomic_load(&s_ia_lowpower);
    if (!lowpower)
        trigger_vibration_motor(cmd->vib_seq, cmd->vib_seq_len);

    // ── 5. 舵机三轴动作序列（改为入队 servo_manager，统一执行体）───────────────
    // 把整个情绪压成【一条】绝对角度并行请求，交给 servo_manager worker 执行：
    //   - 三轴各 angle_1↔angle_2 往返 count 次，执行完由 worker 统一归中 90°；
    //   - 带 s_ia_d1_2_sem，执行完（正常 或 被 servo_manager_flush 打断）后 give；
    //   - 统一执行体保证情绪/自动循环/待机互斥，且进功能盘 flush 能打断本动作。
    // 情绪表 ActionStep_t 是绝对角度，1:1 填入绝对角度请求，无换算误差（plan R5）。
    // 低功耗模式下手臂 count 强制清零：三轴并行插值每步仍会写手臂，但因
    // count=0 恒以 90° 为目标（见 servo_exec_abs_parallel），即"钉住不摆、不断电"；
    // 头部 count/角度不受影响，情绪头部动作照常播放。
    servo_abs_parallel_request_t preq = {
        .head = {cmd->head.angle_1, cmd->head.angle_2, cmd->head.speed, cmd->head.count},
        .l_arm = {cmd->left_arm.angle_1, cmd->left_arm.angle_2, cmd->left_arm.speed,
                  (uint8_t)(lowpower ? 0 : cmd->left_arm.count)},
        .r_arm = {cmd->right_arm.angle_1, cmd->right_arm.angle_2, cmd->right_arm.speed,
                  (uint8_t)(lowpower ? 0 : cmd->right_arm.count)},
    };

    // 清掉可能残留的旧 d1_2 信号（防上一轮超时遗留导致本轮 take 立即返回）
    xSemaphoreTake(s_ia_d1_2_sem, 0);

    if (servo_manager_submit_abs_parallel_notify(&preq, s_ia_d1_2_sem) == ESP_OK)
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
    ESP_LOGI("GIFDBG", "custom动作 开始 idle=%d gif=%s keep=%d",
             act->is_idle, act->gif_path ? act->gif_path : "(NULL)", act->keep_screen);
    servo_manager_flush();

    // 切图（跨线程安全）：状态动作走高优先级 ui_request_state_gif（is_state=true），
    // 否则会被对话中 main_gif_switch_timer_cb 的「丢弃情绪切图」兜底误伤（D.1）。
    if (act->gif_path != NULL && act->gif_path[0] != '\0')
    {
        if (act->is_state_gif)
            ui_request_state_gif(act->gif_path);
        else
            ui_request_emotion_gif(act->gif_path);
    }

    // 震动（逐段阻塞播放；NULL/0 直接返回）
    trigger_vibration_motor(act->vib_seq, act->vib_seq_len);

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

    servo_manager_flush(); // 归中（含打断兜底）
    atomic_store(&s_ia_is_idle, false);
    atomic_store(&s_ia_playing, false);
    ESP_LOGI("GIFDBG", "custom动作 结束 keep=%d → %s", act->keep_screen,
             act->keep_screen ? "停在GIF" : "恢复循环");
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
    ia_request_t req;
    for (;;)
    {
        // 无限等待，收到请求后按类型分发执行（阻塞期间不占 CPU）
        if (xQueueReceive(s_ia_queue, &req, portMAX_DELAY) == pdTRUE)
        {
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
