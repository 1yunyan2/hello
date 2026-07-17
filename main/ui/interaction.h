
#pragma once

/**
 * @file interaction.h
 * @brief 机器人情绪与动作交互总控接口
 *
 * 本模块对外提供情绪枚举和触发接口，内部通过情绪矩阵表（InteractionMatrix_t）
 * 将情绪 ID 映射到屏幕动画、音效文件、震动模式和三轴舵机动作参数。
 *
 * 调用方只需传入情绪枚举值，模块自动完成：
 *   1. 查表找到对应 InteractionMatrix_t 条目
 *   2. 触发屏幕动画（预留接口）
 *   3. 播放音效（预留接口）
 *   4. 控制震动马达
 *   5. 三轴舵机同步执行动作序列（前半段→后半段，循环指定次数）
 *   6. 动作完成后所有舵机归中到 90°待机姿态
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bsp/bsp_board.h" // 引用底层管家（包含舵机 bsp_servo_move_smooth 和通道宏）

/**
 * @brief 机器人情绪/动作指令枚举（对应情绪矩阵表第一列索引）
 *
 * 每个枚举值对应 g_emotion_matrix[] 中的一行数据，
 * 包含屏幕动画、音效、震动模式和三轴舵机动作参数。
 * 扩展新情绪时，在此追加枚举值并在 interaction.c 中补充对应矩阵行。
 */
typedef enum
{
    EMO_HAPPY = 0,        ///< 0: 开心（基础）
    EMO_CURIOUS,          ///< 1: 好奇（基础）
    EMO_TSUNDERE,         ///< 2: 傲娇（基础）
    EMO_TICKLISH,         ///< 3: 怕痒（基础）
    EMO_SLEEPY,           ///< 4: 犯困（基础）
    EMO_GRIEVED,          ///< 5: 委屈（基础）
    EMO_COMFORTABLE,      ///< 6: 舒服（基础）
    EMO_ACT_CUTE,         ///< 7: 撒娇（基础）
    EMO_ANGRY,            ///< 8: 生气（基础）
    EMO_SHY,              ///< 9: 害羞（基础）
    EMO_SURPRISED,        ///< 10: 惊喜（基础）
    EMO_SLUGGISH,         ///< 11: 慵懒（通用）
    EMO_HEALING,          ///< 12: 治愈（基础）
    EMO_EXCITED,          ///< 13: 兴奋（基础）
    EMO_SHY_RUB,          ///< 14: 害羞蹭蹭（害羞进阶，头部左右轻蹭）
    EMO_COMFORTABLE_ROLL, ///< 15: 舒服到打滚（舒服进阶，头部大幅左右摇摆）
    EMO_TSUNDERE_PET,     ///< 16: 傲娇求摸（傲娇进阶，头部微抬+手臂轻抬）
    EMO_SLUGGISH_SIT,     ///< 17: 慵懒瘫坐（慵懒进阶，双臂完全下垂+头部低垂）
    EMO_SURPRISED_HUG,    ///< 18: 惊喜抱抱（惊喜进阶，双臂快速上举张开）
    EMO_TICKLISH_WIGGLE   ///< 19: 怕痒到扭动（怕痒进阶，头部+双臂极速抖动）
} robot_emotion_t;

/** 情绪矩阵总数（EMO_TICKLISH_WIGGLE 是最后一项，值为 count-1）；随机抽情绪用 */
#define EMOTION_COUNT ((int)EMO_TICKLISH_WIGGLE + 1)

/**
 * @brief 单轴舵机动作参数（绝对角度，中心 = 90°）
 *
 * angle_1 → angle_2 为一次循环，共执行 count 次，最后自动归中。
 *   head: 左+(大) 右-(小)  | L/R_arm: 前+(大) 后-(小)
 *
 * @note 原定义在 interaction.c 内部，现移到此处暴露，供 ui_port 构造状态动作表。
 */
typedef struct
{
    float angle_1;  ///< 动作前半段目标角度（度）
    float angle_2;  ///< 动作后半段目标角度（度）；单向动作填 90.0f（归中）
    uint32_t speed; ///< step_ms，对应 SERVO_SPEED_xxx（值越大越慢）
    uint8_t count;  ///< 循环次数（0 = 该轴不参与本动作）
} ActionStep_t;

/**
 * @brief 单段震动步骤
 *
 * 一串 VibStep_t 按顺序逐段播放：每段先以 strength 强度震 on_ms，再静默 off_ms。
 *   strength 0~100（LEDC 占空比，需中间值才是真 PWM 方波；0/100 为直流端点）
 *
 * @note 原定义在 interaction.c 内部，现移到此处暴露，供 ui_port 构造状态动作表。
 */
typedef struct
{
    uint8_t strength;  ///< 该段震动强度 0~100
    uint16_t on_ms;    ///< 该段震动时长（ms）
    uint16_t off_ms;   ///< 该段结束后的静默间隔（ms），最后一段可填 0
} VibStep_t;

/* ═══════════════════════════════════════════════════════════════
 * 自定义动作（状态动作）—— GIF + 三轴舵机 + 震动，三者独立配置
 *
 * 与情绪矩阵解耦：不查 g_emotion_matrix，由调用方（如 ui_port 的状态动作表）
 * 直接给全套参数。用于"对话状态切换"播放 GIF + 舵机 + 震动，且可独立指定
 * 是否高优先级状态切图、播完是否保持屏幕（不恢复待机循环）。
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    IA_REQ_EMOTION = 0, ///< 旧路径：查情绪矩阵
    IA_REQ_CUSTOM       ///< 新路径：调用方直接给全套参数
} ia_req_type_t;

typedef struct
{
    const char *gif_path;     ///< GIF 路径（NULL=不切图，仅舵机+震动；空闲动作用 NULL，GIF 由主循环按 READY 节奏切）
    bool is_state_gif;        ///< true=高优先级状态切图（走 ui_request_state_gif，绕过情绪兜底丢弃）
    bool keep_screen;         ///< true=播完不恢复待机循环（对话状态/空闲均 true，避免反复触发失控循环）
    bool is_idle;             ///< true=空闲动作（低优先级，可被触摸情绪 flush 打断）；与 keep_screen 解耦
    const VibStep_t *vib_seq; ///< 震动序列（NULL=无震动）；须指向 static const
    uint8_t vib_seq_len;      ///< 震动序列段数
    ActionStep_t head;        ///< 头部舵机动作（绝对角度）
    ActionStep_t left_arm;    ///< 左臂舵机动作
    ActionStep_t right_arm;   ///< 右臂舵机动作
} ia_custom_action_t;

/** worker 队列消息：情绪 ID 或自定义动作（tagged union） */
typedef struct
{
    ia_req_type_t type;
    union
    {
        robot_emotion_t emo;       ///< type==IA_REQ_EMOTION
        ia_custom_action_t custom; ///< type==IA_REQ_CUSTOM（整体值拷贝入队）
    } u;
} ia_request_t;
/**
 * @brief 初始化 interaction_manager（创建 worker 任务 + 动作队列）
 *
 * 必须在 bsp_board_servo_init() 之后调用，在首次调用 ui_interaction_play() 之前完成。
 *
 * @return ESP_OK 成功，ESP_ERR_NO_MEM 内存不足
 *
 * @note 调用者：application.c 初始化序列
 */
esp_err_t interaction_manager_init(void);

/**
 * @brief 非阻塞触发机器人的全套情绪表现（屏幕动画 + 音效 + 震动 + 三轴舵机动作）
 *
 * 将目标情绪入队后立即返回，实际动作由内部 worker 任务串行执行：
 *   屏幕动画 → 音效 → 震动马达 → 三轴舵机动作序列 → 舵机归中
 *
 * @param target_emotion 目标情绪 ID（robot_emotion_t 枚举值）
 * @return void（未初始化或队列已满时仅打印日志，不崩溃）
 *
 * @note 调用者：session.c 或上层逻辑模块，在 AI 返回情绪指令时触发
 * @note 此函数现为非阻塞：入队后立即返回，不阻塞调用任务
 * @note 若队列已满（连续触发超过 INTERACTION_QUEUE_LEN），新情绪将被丢弃并打印警告
 */
void ui_interaction_play(robot_emotion_t target_emotion);

/**
 * @brief 非阻塞触发一个自定义动作（GIF + 三轴舵机 + 震动），不查情绪矩阵
 *
 * 将动作整体值拷贝入队后立即返回，由情绪 worker 串行执行：
 *   切图（按 is_state_gif 选高/低优先级）→ 震动 → 三轴舵机绝对角度 → 归中，
 *   keep_screen=false 时播完恢复待机自动循环，true 时屏幕停在该 GIF。
 *
 * @param act 动作描述（指针字段须指向 static const，调用方保证生命周期）
 * @note 线程安全，可从任意任务调用；队列满时丢弃并打印警告。
 */
void ui_interaction_play_custom(const ia_custom_action_t *act);

/**
 * @brief 查询当前是否正在播放某情绪动作（线程安全，跨核 atomic）
 *
 * 供主界面自动 GIF 循环（ui_port.c）与触摸路由读取：
 *   - 自动循环：播放中暂停切图 + 暂停舵机入队，让位给情绪；
 *   - 触摸路由：播放中屏蔽新的情绪触摸（丢弃，不排队不打断）。
 * 标志在情绪动作开始（早于 GIF/震动）置位，动作完成（含归中）后清位。
 *
 * @return true=正在播放情绪，false=空闲
 */
bool interaction_is_playing(void);

/**
 * @brief 查询当前播放的是否为「空闲动作」（低优先级，可被触摸情绪打断）
 *
 * 空闲动作 = 开机后随机 GIF 循环每张伴随的 GIF+舵机+震动（keep_screen=false）。
 * 触摸路由据此放行触摸（空闲动作播放中不屏蔽情绪触摸）；ui_interaction_play
 * 据此在情绪入队前 flush 打断当前空闲动作。
 *
 * @return true=正在播空闲动作；false=空闲/正在播情绪或对话状态动作
 */
bool interaction_is_idle_action(void);

/**
 * @brief 设置/清除「低功耗头部专属」模式（供 standby 一级待机使用）
 *
 * 置 true 后，interaction_play_blocking()（情绪播放）会：
 *   - 手臂（左/右）动作次数强制清零（count=0），舵机并行插值时手臂目标恒为 90°，
 *     即"钉住不摆、但不断电失力"（PWM 仍保持，见 servo_exec_abs_parallel 语义）；
 *   - 跳过震动马达（trigger_vibration_motor 不调用）；
 *   - 头部动作、GIF 切图完全不受影响，情绪矩阵照常随机播放。
 * 置 false 恢复正常（手臂/震动照常参与）。
 *
 * @note 仅影响【情绪路径】(IA_REQ_EMOTION)，不影响自定义动作(IA_REQ_CUSTOM，
 *       如对话状态/普通空闲动作)——那些路径不在低功耗一级触发。
 * @note 线程安全（atomic），可在 standby_task/standby_wake 等任意任务调用。
 * @note 调用者：standby.c enter_standby()/standby_wake()
 */
void interaction_set_lowpower(bool enable);

/**
 * @brief 清空 interaction 情绪/动作请求队列中【未执行】的存量请求
 *
 * 进低功耗（一级/二级）时调用：一级待机期间 main_gif_switch_timer_cb 会持续往
 * s_ia_queue 塞随机情绪；若不清，进二级瞬间队列里的存量情绪会被 worker 继续执行，
 * 用 iot_servo_write_angle 把刚 ledc_stop 的舵机 PWM 重新点亮（"二级了头部还在动"）。
 * 本接口只 xQueueReset 清未执行请求，不影响【正在执行】的那一条（由 servo_manager_flush
 * 打断），二者需配合使用。
 *
 * @note 线程安全（QueueReset 内部临界区），可在 standby_task 等任意任务调用。
 * @note 调用者：standby.c enter_standby()/enter_deep_standby()
 */
void interaction_flush_queue(void);

/**
 * @brief 为 OTA 升级彻底停止 interaction（GIF 切图 + 舵机 + 震动全部静默）
 *
 * 置内部停止标志（worker 此后丢弃一切请求）、清空未执行队列、打断正在执行的舵机动作。
 * 用于 OTA 升级期间释放 CPU/避免 GIF 逐帧读 flash 与下载抢资源。
 *
 * @note 停止标志不清除——OTA 无论成功失败都会 esp_restart()，重启后自然复位。
 * @note 线程安全，可从任意任务（如 OTA 下载任务）调用。
 */
void interaction_stop_for_ota(void);
