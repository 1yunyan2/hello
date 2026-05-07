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
#include "driver/gpio.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "bsp/bsp_board.h"
#include "ui/ui_port.h"

static const char *TAG = "INTERACTION";

// ─── Worker task 配置 ────────────────────────────────────────────────────────
#define INTERACTION_QUEUE_LEN 8     ///< 最多缓存 4 个待执行情绪（超出时丢弃新请求）
#define INTERACTION_TASK_STACK 8192 ///< worker 栈大小（含音频/舵机调用链）
#define INTERACTION_TASK_PRIO 5     ///< 优先级与 session 相当，略低于音频（7）

static QueueHandle_t s_ia_queue = NULL; ///< 情绪 ID 队列
static TaskHandle_t s_ia_worker = NULL; ///< worker 任务句柄
static bool s_ia_inited = false;        // 初始化完成

// ==========================================
// 1. 数据结构：严格对齐情绪矩阵表头
// ==========================================

/**
 * @brief 单轴舵机动作参数
 *
 * 描述一个情绪下某轴舵机的完整动作：
 *   angle_1 → angle_2 为一次循环，共执行 count 次，最后自动归中。
 *
 * 角度说明：中心 = 90°。
 *   head:  左+(大) 右-(小)  | L/R_arm: 前+(大) 后-(小)
 */
typedef struct
{
    float angle_1;  ///< 动作前半段目标角度（度）
    float angle_2;  ///< 动作后半段目标角度（度）；单向动作填 90.0f（归中）
    uint32_t speed; ///< step_ms，对应 SERVO_SPEED_xxx（值越大越慢）
    uint8_t count;  ///< 循环次数（0 = 该轴不参与本情绪动作）
} ActionStep_t;

/**
 * @brief 情绪矩阵行：情绪 ID → 全套硬件动作映射
 */
typedef struct
{
    robot_emotion_t emotion_id; ///< 情绪枚举 ID（查表键）
    const char *screen_anim;    ///< 屏幕动画标识（传给 UI 层，预留）
    uint8_t motor_mode;         ///< 震动马达模式 (0=无 1=轻1次 2=短促2次 3=连续 4=长1次)
    const char *audio_file;     ///< 音效文件名（传给音频层，预留）
    ActionStep_t head;          ///< 头部舵机（CH_HEAD）
    ActionStep_t left_arm;      ///< 左臂舵机（CH_L_ARM）
    ActionStep_t right_arm;     ///< 右臂舵机（CH_R_ARM）
} InteractionMatrix_t;

// ==========================================
// 2. 情绪动作矩阵表（14 种全覆盖）
//
// 角度速查：
//   左/前 45° → 135°  |  右/后 45° → 45°
//   头部软限位: [45, 135]   手臂软限位: [10, 160]
// ==========================================
static const InteractionMatrix_t g_emotion_matrix[] = {
    //! 后续将screen_anim和audio_file字段传给UI层和音频层，目前先占位空字符串，避免未初始化的垃圾值导致不可预期的行为。
    // ── 0: 开心 ──────────────────────────────────────────────────────────────
    // 快速左右摇头3次 + 双臂前后摆2次 + 2次短促震动
    {
        .emotion_id = EMO_HAPPY,
        .screen_anim = "anim_happy_stars",
        .motor_mode = 2,
        .audio_file = "laugh_short.mp3",
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 3},    // 左30→右30，快速×3
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 2}, // 前20→后20，中速×2
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 2},
    },

    // ── 1: 好奇 ──────────────────────────────────────────────────────────────
    // 缓慢左右侧头2次 + 双臂前举1次 + 1次轻震动
    {
        .emotion_id = EMO_CURIOUS,
        .screen_anim = "anim_curious_q",
        .motor_mode = 1,
        .audio_file = "doubt.mp3",
        .head = {115.0f, 65.0f, SERVO_SPEED_SLOW, 2},     // 左25→右25，慢×2
        .left_arm = {120.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 前30→归中，慢×1
        .right_arm = {120.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 2: 傲娇 ──────────────────────────────────────────────────────────────
    // 头部轻偏左1次（不回来，等归中）+ 双臂快速后收1次 + 无震动
    {
        .emotion_id = EMO_TSUNDERE,
        .screen_anim = "anim_tsundere",
        .motor_mode = 0,
        .audio_file = "hmph.mp3",
        .head = {105.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1}, // 左15→归中，极慢×1
        .left_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},   // 后30→归中，快×1
        .right_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 3: 怕痒 ──────────────────────────────────────────────────────────────
    // 极速抖头5次 + 双臂快速前后摆3次 + 连续震动
    {
        .emotion_id = EMO_TICKLISH,
        .screen_anim = "anim_ticklish",
        .motor_mode = 3,
        .audio_file = "ticklish.mp3",
        .head = {105.0f, 75.0f, SERVO_SPEED_VERY_FAST, 5}, // 左15→右15，极快×5
        .left_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 3},  // 前25→后25，快×3
        .right_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 3},
    },

    // ── 4: 犯困 ──────────────────────────────────────────────────────────────
    // 极慢大幅侧头1次 + 双臂缓缓下垂1次 + 无震动
    {
        .emotion_id = EMO_SLEEPY,
        .screen_anim = "anim_sleepy",
        .motor_mode = 0,
        .audio_file = "yawn.mp3",
        .head = {115.0f, 65.0f, SERVO_SPEED_VERY_SLOW, 1},    // 左25→右25，极慢×1
        .left_arm = {70.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1}, // 后20（下垂）→归中
        .right_arm = {70.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
    },

    // ── 5: 委屈 ──────────────────────────────────────────────────────────────
    // 头部缓慢轻偏1次 + 双臂小幅后收（内缩）1次 + 无震动
    {
        .emotion_id = EMO_GRIEVED,
        .screen_anim = "anim_grieved",
        .motor_mode = 0,
        .audio_file = "sob.mp3",
        .head = {100.0f, 90.0f, SERVO_SPEED_SLOW, 1},    // 左10→归中，慢×1
        .left_arm = {65.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 后25→归中，慢×1
        .right_arm = {65.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 6: 舒服 ──────────────────────────────────────────────────────────────
    // 慢速轻摇头2次 + 双臂微微展开1次 + 1次轻震动
    {
        .emotion_id = EMO_COMFORTABLE,
        .screen_anim = "anim_comfortable",
        .motor_mode = 1,
        .audio_file = "sigh_happy.mp3",
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},     // 左15→右15，慢×2
        .left_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 前10→归中，慢×1
        .right_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 7: 撒娇 ──────────────────────────────────────────────────────────────
    // 头部中速倾斜往返2次 + 双臂中速上举2次 + 2次短促震动
    {
        .emotion_id = EMO_ACT_CUTE,
        .screen_anim = "anim_act_cute",
        .motor_mode = 2,
        .audio_file = "cute.mp3",
        .head = {110.0f, 90.0f, SERVO_SPEED_MID, 2},     // 左20→归中，中速×2
        .left_arm = {120.0f, 90.0f, SERVO_SPEED_MID, 2}, // 前30→归中，中速×2
        .right_arm = {120.0f, 90.0f, SERVO_SPEED_MID, 2},
    },

    // ── 8: 生气 ──────────────────────────────────────────────────────────────
    // 快速大幅摇头3次 + 双臂用力前摆2次 + 2次短促震动
    {
        .emotion_id = EMO_ANGRY,
        .screen_anim = "anim_angry",
        .motor_mode = 2,
        .audio_file = "angry.mp3",
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 3},     // 左30→右30，快×3
        .left_arm = {125.0f, 70.0f, SERVO_SPEED_FAST, 2}, // 前35→后20，快×2
        .right_arm = {125.0f, 70.0f, SERVO_SPEED_FAST, 2},
    },

    // ── 9: 害羞 ──────────────────────────────────────────────────────────────
    // 头部缓缓轻低垂1次 + 双臂小幅前举（遮脸感）1次 + 1次轻震动
    {
        .emotion_id = EMO_SHY,
        .screen_anim = "anim_shy",
        .motor_mode = 1,
        .audio_file = "shy.mp3",
        .head = {80.0f, 90.0f, SERVO_SPEED_SLOW, 1},      // 右10→归中，慢×1
        .left_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1}, // 前15→归中，慢×1
        .right_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 10: 惊喜 ─────────────────────────────────────────────────────────────
    // 头部快速左右摆1次 + 双臂快速上扬1次 + 1次轻震动
    {
        .emotion_id = EMO_SURPRISED,
        .screen_anim = "anim_surprised",
        .motor_mode = 1,
        .audio_file = "surprise.mp3",
        .head = {115.0f, 65.0f, SERVO_SPEED_FAST, 1},     // 左25→右25，快×1
        .left_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1}, // 前35→归中，快×1
        .right_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 11: 慵懒 ─────────────────────────────────────────────────────────────
    // 极慢小幅侧头1次 + 双臂极慢微垂1次 + 无震动
    {
        .emotion_id = EMO_SLUGGISH,
        .screen_anim = "anim_sluggish",
        .motor_mode = 0,
        .audio_file = "lazy.mp3",
        .head = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 1},    // 左10→右10，极慢×1
        .left_arm = {75.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1}, // 后15→归中，极慢×1
        .right_arm = {75.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
    },

    // ── 12: 治愈 ─────────────────────────────────────────────────────────────
    // 缓慢温柔摇头2次 + 双臂轻柔微展2次 + 1次轻震动
    {
        .emotion_id = EMO_HEALING,
        .screen_anim = "anim_healing",
        .motor_mode = 1,
        .audio_file = "healing.mp3",
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},     // 左15→右15，慢×2
        .left_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 2}, // 前10→归中，慢×2
        .right_arm = {100.0f, 90.0f, SERVO_SPEED_SLOW, 2},
    },

    // ── 13: 兴奋 ─────────────────────────────────────────────────────────────
    // 快速大幅摇头4次 + 双臂大幅前后摆3次 + 2次短促震动
    {
        .emotion_id = EMO_EXCITED,
        .screen_anim = "anim_excited",
        .motor_mode = 2,
        .audio_file = "excited.mp3",
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 4},     // 左30→右30，快×4
        .left_arm = {125.0f, 60.0f, SERVO_SPEED_FAST, 3}, // 前35→后30，快×3
        .right_arm = {125.0f, 60.0f, SERVO_SPEED_FAST, 3},
    },
    // ── 14: 害羞蹭蹭 ─────────────────────────────────────────────────────────
    // 表格要求: 轻微震动2次 | 头:极慢左15右15(4次) | 左/右臂:缓慢前15(1次)
    {
        .emotion_id = EMO_SHY_RUB,
        .screen_anim = "anim_shy_rub",
        .motor_mode = 2,
        .audio_file = "shy_rub.mp3",
        .head = {105.0f, 75.0f, SERVO_SPEED_VERY_SLOW, 4},
        .left_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {105.0f, 90.0f, SERVO_SPEED_SLOW, 1},
    },

    // ── 15: 舒服到打滚 ───────────────────────────────────────────────────────
    // 表格要求: 持续轻柔震动 | 头:缓慢左30右30(2次) | 左/右臂:极慢后20前10(2次)
    {
        .emotion_id = EMO_COMFORTABLE_ROLL,
        .screen_anim = "anim_comfortable_roll",
        .motor_mode = 1, // 当前代码无“持续轻柔”，暂以1代
        .audio_file = "purr.mp3",
        .head = {120.0f, 60.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {70.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 2},
        .right_arm = {70.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 2},
    },

    // ── 16: 傲娇求摸 ─────────────────────────────────────────────────────────
    // 表格要求: 无震动 | 头:极慢左15右15(3次) | 左/右臂:快速后30(1次)
    {
        .emotion_id = EMO_TSUNDERE_PET,
        .screen_anim = "anim_tsundere_pet",
        .motor_mode = 0,
        .audio_file = "hmph_pet.mp3",
        .head = {105.0f, 75.0f, SERVO_SPEED_VERY_SLOW, 3},
        .left_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
        .right_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 17: 慵懒瘫坐 ─────────────────────────────────────────────────────────
    // 表格要求: 缓慢长震动1次 | 头:极慢左10右10(1次) | 左/右臂:极慢前10(1次)
    {
        .emotion_id = EMO_SLUGGISH_SIT,
        .screen_anim = "anim_sluggish_sit",
        .motor_mode = 4,
        .audio_file = "lazy_sit.mp3",
        .head = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 1},
        .left_arm = {100.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
        .right_arm = {100.0f, 90.0f, SERVO_SPEED_VERY_SLOW, 1},
    },

    // ── 18: 惊喜抱抱 ─────────────────────────────────────────────────────────
    // 表格要求: 短促强震动1次 | 头:快速左15(1次) | 左/右臂:快速前35(1次)
    {
        .emotion_id = EMO_SURPRISED_HUG,
        .screen_anim = "anim_surprised_hug",
        .motor_mode = 4, // 当前代码无“短促强震”，暂以4(长震)代
        .audio_file = "hug_me.mp3",
        .head = {105.0f, 90.0f, SERVO_SPEED_FAST, 1},
        .left_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1},
        .right_arm = {125.0f, 90.0f, SERVO_SPEED_FAST, 1},
    },

    // ── 19: 怕痒到扭动 ───────────────────────────────────────────────────────
    // 表格要求: 连续快速震动 | 头:极快左20右20(5次) | 左/右臂:快速前25后25(4次)
    {
        .emotion_id = EMO_TICKLISH_WIGGLE,
        .screen_anim = "anim_ticklish_wiggle",
        .motor_mode = 3,
        .audio_file = "wiggle_laugh.mp3",
        .head = {110.0f, 70.0f, SERVO_SPEED_VERY_FAST, 5},
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
 * @brief 按模式触发震动马达脉冲
 *
 * @param mode  0=无  1=轻1次50ms  2=短促2次  3=连续3次  4=长震1次200ms
 */
static void trigger_vibration_motor(uint8_t mode)
{
    if (mode == 0)
        return;

    if (mode == 1)
    {
        // 轻微 1 次（50ms）
        gpio_set_level(BSP_MOTOR_VIB_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
    }
    else if (mode == 2)
    {
        // 短促 2 次（50ms × 2，间隔 50ms）
        for (int i = 0; i < 2; i++)
        {
            gpio_set_level(BSP_MOTOR_VIB_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(50));
            gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
            if (i < 1)
                vTaskDelay(pdMS_TO_TICKS(50)); // 两次之间间隔
        }
    }
    else if (mode == 3)
    {
        // 连续 3 次（30ms × 3，间隔 30ms）
        for (int i = 0; i < 3; i++)
        {
            gpio_set_level(BSP_MOTOR_VIB_PIN, 1);
            vTaskDelay(pdMS_TO_TICKS(30));
            gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
            if (i < 2)
                vTaskDelay(pdMS_TO_TICKS(30));
        }
    }
    else if (mode == 4)
    {
        // 长震 1 次（200ms）
        gpio_set_level(BSP_MOTOR_VIB_PIN, 1);
        vTaskDelay(pdMS_TO_TICKS(200));
        gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
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

    // ── 3. 音频播放（方波占位，真实文件接入后替换）──────────────────────────
    // 注：lv_gif_set_src 会访问 SPIFFS（SPI flash），ia_worker 栈在 SPIRAM，
    //     禁用 cache 时 SPIRAM 不可访问，因此不能在此任务中调用屏幕动画。
    // TODO: audio_player_play_file(cmd->audio_file);
    ESP_LOGI(TAG, "🔊 音效槽: %s（当前使用方波占位）", cmd->audio_file);
    play_square_wave_beep(); // 播放方波占位音频

    // ── 4. 震动马达 ──────────────────────────────────────────────────────────
    trigger_vibration_motor(cmd->motor_mode);

    // ── 5. 舵机三轴动作序列 ─────────────────────────────────────────────────
    // 以三轴中循环次数最多的为外层循环次数，保证全轴都完整执行
    uint8_t max_loop = cmd->head.count;
    if (cmd->left_arm.count > max_loop)
        max_loop = cmd->left_arm.count;
    if (cmd->right_arm.count > max_loop)
        max_loop = cmd->right_arm.count;

    for (uint8_t loop = 0; loop < max_loop; loop++)
    {
        // 前半段：三轴同时运动到 angle_1（未参与的轴保持 90°）
        bsp_servo_move_all_parallel(
            (loop < cmd->head.count) ? cmd->head.angle_1 : 90.0f,
            (loop < cmd->left_arm.count) ? cmd->left_arm.angle_1 : 90.0f,
            (loop < cmd->right_arm.count) ? cmd->right_arm.angle_1 : 90.0f,
            cmd->head.speed);

        // 后半段：三轴同时运动到 angle_2
        bsp_servo_move_all_parallel(
            (loop < cmd->head.count) ? cmd->head.angle_2 : 90.0f,
            (loop < cmd->left_arm.count) ? cmd->left_arm.angle_2 : 90.0f,
            (loop < cmd->right_arm.count) ? cmd->right_arm.angle_2 : 90.0f,
            cmd->head.speed);
    }

    // ── 6. 全轴同时归中（恢复待机姿态）──────────────────────────────────────
    bsp_servo_move_all_parallel(90.0f, 90.0f, 90.0f, SERVO_SPEED_MID);

    ESP_LOGI(TAG, ">>> 情绪动作执行完毕: %d <<<", (int)target_emotion);
}

// ==========================================
// 5. Worker 任务：串行消费情绪队列
// ==========================================

/**
 * @brief interaction worker 主循环
 *
 * 永久阻塞等待队列，每次取出一个 robot_emotion_t 并执行完整动作序列。
 * 串行执行保证动作不重叠，无需外部加锁。
 */
static void interaction_worker_task(void *arg)
{
    robot_emotion_t emo;
    for (;;)
    {
        // 无限等待，收到情绪请求后执行（阻塞期间不占 CPU）
        if (xQueueReceive(s_ia_queue, &emo, portMAX_DELAY) == pdTRUE)
        {
            interaction_play_blocking(emo);
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
    s_ia_queue = xQueueCreate(INTERACTION_QUEUE_LEN, sizeof(robot_emotion_t));
    if (s_ia_queue == NULL)
    {
        ESP_LOGE(TAG, "创建 interaction 队列失败，内存不足!");
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

    // 非阻塞入队（timeout=0），队满则丢弃
    if (xQueueSend(s_ia_queue, &target_emotion, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "interaction 队列已满，丢弃情绪 %d", (int)target_emotion);
    }
}
