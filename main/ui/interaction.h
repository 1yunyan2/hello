
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
    EMO_TSUNDERE_BASE,    ///< 2: 傲娇1（傲娇基础态·别过脸哼，配 2_1.gif）
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
    EMO_TSUNDERE_PET,     ///< 16: 傲娇求摸（配 2_4.gif，双臂抬到胸前绷住+头扭开让位）
    EMO_TSUNDERE_PEEK,    ///< 17: 傲娇3（配 2_3.gif，偷瞄）
    EMO_TSUNDERE_ROLL,    ///< 18: 傲娇2（配 2_2.gif，甩头→偷瞄→被发现，三要素最全）
    EMO_SLUGGISH_SIT,     ///< 19: 慵懒瘫坐（慵懒进阶，双臂完全下垂+头部低垂）
    EMO_SURPRISED_HUG,    ///< 20: 惊喜抱抱（惊喜进阶，双臂快速上举张开）
    EMO_TICKLISH_WIGGLE,  ///< 21: 怕痒到扭动（怕痒进阶，头部+双臂极速抖动）
    EMO_NEUTRAL1,         ///< 22: 中性1
    EMO_NEUTRAL2,         ///< 23: 中性2
    EMO_NEUTRAL3,         ///< 24: 中性3
    EMO_NEUTRAL4,         ///< 25: 中性4
    EMO_NEUTRAL5,         ///< 26: 中性5
    EMO_NEUTRAL6,         ///< 27: 中性6
    EMO_EXCITED1,         ///< 28: 兴奋3_1（星星眼·最亢奋，主角是双臂）
    EMO_EXCITED2,         ///< 29: 兴奋3_2（圆眼笑·憨，起手慢半拍）
    EMO_EXCITED3,         ///< 30: 兴奋3_3（眨眼笑·俏皮，左右臂刻意不对称）
    EMO_EXCITED4,         ///< 31: 兴奋3_4（星星眼，头先动臂后动，相位相反）
    EMO_CURIOUS1,         ///< 32: 好奇①  4_1.gif（好奇）
    EMO_CURIOUS2,         ///< 33: 好奇②  4_2.gif（好奇2）
    EMO_CURIOUS3,         ///< 34: 好奇③  4_3.gif（好奇 2）
    EMO_GRIEVED1,         ///< 35: 委屈①  5_1.gif（委屈）
    EMO_GRIEVED2,         ///< 36: 委屈②  5_2.gif（委屈2）
    EMO_SHY1,             ///< 37: 害羞①  6_1.gif（害羞）
    EMO_SHY2,             ///< 38: 害羞②  6_2.gif（害羞蹭蹭）
    EMO_TICKLISH1,        ///< 39: 怕痒①  7_1.gif（怕痒）
    EMO_TICKLISH2,        ///< 40: 怕痒②  7_2.gif（怕痒2）
    EMO_TICKLISH3,        ///< 41: 怕痒③  7_3.gif（怕痒 2）
    EMO_TICKLISH4,        ///< 42: 怕痒④  7_4.gif（怕痒到扭动）
    EMO_SURPRISED1,       ///< 43: 惊喜①  8_1.gif（惊喜）
    EMO_SURPRISED2,       ///< 44: 惊喜②  8_2.gif（惊喜2）
    EMO_SURPRISED3,       ///< 45: 惊喜③  8_3.gif（惊喜抱抱）
    EMO_SLUGGISH1,        ///< 46: 慵懒①  9_1.gif（慵懒2，全组仅此一张）
    EMO_ACT_CUTE1,        ///< 47: 撒娇① 10_1.gif（撒娇，全组仅此一张）
    EMO_HEALING1,         ///< 48: 治愈① 11_1.gif（治愈2，全组仅此一张）
    EMO_SLEEPY1,          ///< 49: 犯困① 12_1.gif（犯困）
    EMO_SLEEPY2,          ///< 50: 犯困② 12_2.gif（犯困 2）
    EMO_ANGRY1,           ///< 51: 生气① 13_1.gif（生气）
    EMO_ANGRY2,           ///< 52: 生气② 13_2.gif（生气 2）
    EMO_COMFORTABLE1,     ///< 53: 舒服① 14_1.gif（舒服）
    EMO_COMFORTABLE2,     ///< 54: 舒服② 14_2.gif（舒服到打滚）
    EMO_COMFORTABLE3,     ///< 55: 舒服③ 14_3.gif（舒服(1)）

    /* ── 2026-09-21 中性四张补齐 ───────────────────────────────────────────
     * ★这里【不是】它们"应该"待的位置：EMO_NEUTRAL1~6 在 22~27（本枚举头部），
     *   而 7/8/9 排在 56~58 的末尾。原因是硬规矩（见 [[emotion_4x_14x_sets]]）：
     *   【新枚举一律末尾追加，绝不插队】——插在中间会让其后所有值整体 +1，
     *   而 MQTT 下发 / NVS 持久化都是按【数值】认情绪的，一插队就整体错位。
     *   （反例：EMO_TSUNDERE_ROLL 当年插在 18，使 18~30 全部 +1，已在记忆里留档。）
     *   故编号"难看"是刻意的代价，功能上等价。 */
    EMO_NEUTRAL7,         ///< 56: 中性7  1_7.gif（中性聆听1-1-1）
    EMO_NEUTRAL8,         ///< 57: 中性8  1_8.gif（中性聆听2）
    EMO_NEUTRAL9,         ///< 58: 中性9  1_9.gif（中性聆听3）
} robot_emotion_t;

// typedef enum
// {
//     EMO_NEUTRAL1,      ///< 1: 中性1
//     EMO_NEUTRAL2,      ///< 2: 中性2
//     EMO_NEUTRAL3,      ///< 3: 中性3
//     EMO_NEUTRAL4,      ///< 4: 中性4
//     EMO_NEUTRAL5,      ///< 5: 中性5
//     EMO_NEUTRAL6,      ///< 6: 中性6
//     EMO_NEUTRAL7,      ///< 7: 中性7聆听
//     EMO_NEUTRAL8,      ///< 8: 中性8聆听
//     EMO_NEUTRAL9,      ///< 9: 中性9聆听
//     EMO_TSUNDERE_BASE, ///< 10: 傲娇1（傲娇基础态·别过脸哼，配 2_1.gif）
//     EMO_TSUNDERE_ROLL, ///< 11: 傲娇2（配 2_2.gif，甩头→偷瞄→被发现，三要素最全）
//     EMO_TSUNDERE_PEEK, ///< 12: 傲娇3（配 2_3.gif，偷瞄）
//     EMO_TSUNDERE_PET,  ///< 13: 傲娇4求摸（配 2_4.gif，双臂抬到胸前绷住+头扭开让位）
//     EMO_EXCITED1,      ///< 14: 兴奋3_1（星星眼·最亢奋，主角是双臂）
//     EMO_EXCITED2,      ///< 15: 兴奋3_2（圆眼笑·憨，起手慢半拍）
//     EMO_EXCITED3,      ///< 16: 兴奋3_3（眨眼笑·俏皮，左右臂刻意不对称）
//     EMO_EXCITED4,      ///< 17: 兴奋3_4（星星眼，头先动臂后动，相位相反）
//     EMO_CURIOUS1,      ///< 18: 好奇①  4_1.gif（好奇）
//     EMO_CURIOUS2,     ///< 19: 好奇②  4_2.gif（好奇2）
//     EMO_CURIOUS3,     ///< 20: 好奇③  4_3.gif（好奇 2）
//     EMO_GRIEVED1,     ///< 21: 委屈①  5_1.gif（委屈）
//     EMO_GRIEVED2,     ///< 22: 委屈②  5_2.gif（委屈2）
//     EMO_SHY1,         ///< 23: 害羞①  6_1.gif（害羞）
//     EMO_SHY2,         ///< 24: 害羞②  6_2.gif（害羞蹭蹭）
//     EMO_TICKLISH1,    ///< 25: 怕痒①  7_1.gif（怕痒）
//     EMO_TICKLISH2,    ///< 26: 怕痒②  7_2.gif（怕痒2）
//     EMO_TICKLISH3,    ///< 27: 怕痒③  7_3.gif（怕痒 2）
//     EMO_TICKLISH4,    ///< 28: 怕痒④  7_4.gif（怕痒到扭动）
//     EMO_SURPRISED1,   ///< 29: 惊喜①  8_1.gif（惊喜）
//     EMO_SURPRISED2,   ///< 30: 惊喜②  8_2.gif（惊喜2）
//     EMO_SURPRISED3,   ///< 31: 惊喜③  8_3.gif（惊喜抱抱）
//     EMO_SLUGGISH1,    ///< 32: 慵懒①  9_1.gif（慵懒2，全组仅此一张）
//     EMO_ACT_CUTE1,    ///< 33: 撒娇① 10_1.gif（撒娇，全组仅此一张）
//     EMO_HEALING1,     ///< 34: 治愈① 11_1.gif（治愈2，全组仅此一张）
//     EMO_SLEEPY1,      ///< 35: 犯困① 12_1.gif（犯困）
//     EMO_SLEEPY2,      ///< 36: 犯困② 12_2.gif（犯困 2）
//     EMO_ANGRY1,       ///< 37: 生气① 13_1.gif（生气）
//     EMO_ANGRY2,       ///< 38: 生气② 13_2.gif（生气 2）
//     EMO_COMFORTABLE1, ///< 39: 舒服① 14_1.gif（舒服）
//     EMO_COMFORTABLE2, ///< 40: 舒服② 14_2.gif（舒服到打滚）
//     EMO_COMFORTABLE3, ///< 41: 舒服③ 14_3.gif（舒服(1)）
// } robot_emotion_t;

/** 情绪矩阵总数（最后一个枚举值 +1）；随机抽情绪用
 *  ⚠️ 新增枚举后必须同步挪到这里 → 现在锚在 EMO_NEUTRAL9(58)，故 = 59。
 *     旧值锚在 EMO_EXCITED4 上，4_x~14_x 加进来后若不改，随机抽情绪会抽不到新情绪；
 *     2026-09-21 补 1_7/1_8/1_9 时序末尾追加了 EMO_NEUTRAL7~9，故再挪一次。
 *  ⚠️ 锚点必须永远指向【枚举里最大的那个值】（即末尾那个），不能指向语义上"更该在这"的值。 */
#define EMOTION_COUNT ((int)EMO_NEUTRAL9 + 1)

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

/** 动作序列每轴最大步数（超过则表里放不下；不够用就改大这个数）
 *
 * ★2026-09-11 由 8 提到 32。原因：情绪 19「怕痒扭动」头部已用满 8 步，
 *   上限先于表达力成为瓶颈。代价仅 flash（表是 static const 放 flash，
 *   不占内部 SRAM），每加 1 步约 240 字节。
 * ⚠️ 改这个数必须同步另外两处，否则长序列会被静默截断：
 *   - servo_manager.h  SERVO_SEQ_MAX_STEPS   （须相等）
 *   - bsp_board.h      BSP_SERVO_SEQ_MAX_POINTS（须 ≥ 本值 +2，多的 2 个
 *     位置留给"走向终点"和"归中"那两步，它们被拼进同一串点） */
#define ACTION_SEQ_MAX_STEPS 32

/**
 * @brief 动作序列中的一步：目标角度 + 该步专属速度（2026-09-11 新增）
 *
 * 【为什么每步要带速度】真实生物很少匀速做完一整套动作，一个动作里往往有
 *   两种节奏。例如"惊吓"应当是【猛地甩过去】+【余悸未消地慢慢晃回来】，
 *   "害羞"是【慢慢低头】+【突然快速缩回】。旧结构整条序列共用一个速度，
 *   这类对比做不出来，动作观感偏机械。
 *
 * 【怎么填】speed 留空（或填 0）= 沿用本轴的 seq_speed，第三位 hold_ms 留空
 *   （或填 0）= 到位后不停留。所以不想变速、不想停顿的动作写法和以前一样，
 *   一个额外的数字都不用填：
 *
 *       .seq = {
 *           { 65.0f, SERVO_SPEED_VERY_FAST },  // 这一步单独用极快 —— 猛地甩出去
 *           { 65.0f, 0, 400 },                 // ★原地停 400ms（到位后不动）
 *           {115.0f },                         // 速度位留空 = 用下面的 seq_speed
 *           { 70.0f },                         // 同上
 *       },
 *       .seq_len   = 4,
 *       .seq_speed = SERVO_SPEED_MID,          // 未单独指定的步都用它
 *
 * ★2026-09-18 新增 hold_ms —— 这一位补上了执行层缺失的【时间】维度。
 *
 * 【为什么必须要它，堆重复点为什么不行】执行层每段的耗时 = |角度差| × speed，
 *   时间的唯一来源是【位移】。所以"原地不动"耗时恒为 0，会被 frames<1 的钳位
 *   压成 1 帧（约 20ms）就换下一点：
 *     · 连写 6 个 {60.0f} 想表示"停在 60° 一会儿" ⇒ 实际只停约 6×20=120ms，看不出；
 *     · 改成 {89.0f},{90.0f} 交替 ⇒ 1° 折算出 0 帧，照样钳成 1 帧，
 *       与纯重复角度【耗时完全相同】，只是多了一次 1° 的方波抖动。
 *   ⇒ 停顿时长写不出来的问题，只能用显式字段解决，不能靠数组凑。
 *
 * 【hold_ms 语义】走到本步角度后，原地保持 hold_ms 毫秒，再前往下一步。
 *   与轴速【无关】：按毫秒计，快轴慢轴的 400ms 都是同样的墙上时间。
 *   ⚠️ 停留会占用本步的步数（不额外占位），故加了停顿后要注意 seq_len 与
 *      容量 ACTION_SEQ_MAX_STEPS 的余量。
 */
typedef struct
{
    float angle;      ///< 该步目标角度（度，0~180；超出由 bsp 软限位裁剪）
    uint32_t speed;   ///< 该步速度（SERVO_SPEED_xxx）；★留空/0 = 沿用 seq_speed
    uint32_t hold_ms; ///< ★到位后原地停留的毫秒数；★留空/0 = 不停留
} ActionSeqStep_t;

/**
 * @brief 单轴动作序列（2026-09-04 新结构，取代 ActionStep_t 的"A↔B 往返 N 次"）
 *
 * 【一个轴的完整生命周期，四段】
 *
 *      90°(起始)  →  终点(target)  →  动作序列(seq)  →  归中 90°
 *      固定           每情绪每轴各配      真正的动作内容      收尾
 *
 *   ① 起始永远是 90°（上电归中位）。
 *   ② 终点 target：从 90° 走到这里，每个情绪、每个轴都可以不一样。
 *   ③ 动作序列 seq：到了终点之后，依次走过 seq[0]、seq[1]... 这一串角度点。
 *      这才是"这个情绪长什么样"的主体，步数不固定（seq_len 指定）。
 *   ④ 归中：序列走完才归中，由 servo_manager worker 统一负责。
 *
 * 【为什么推翻 ActionStep_t】旧结构只能表达"A↔B 来回 N 次"，两个角度打死，
 *   做不出"到位后先向左 30°、再向右 20°、再回来一点"这类多步动作；且旧执行
 *   路径把三轴强行同步（速度取三轴最慢的那个，见 servo_manager.c），头和手臂
 *   的幅度、快慢本来就该不同，被统一后全走样。新结构下三轴各走各的。
 *
 * 【三轴的时间关系】三轴独立推进，各走各的步数与速度，互不等待；
 *   全部走完后才一起归中（归中本身仍是三轴同步的一次动作）。
 *
 * 【怎么配】
 *      .head = {
 *          .target    = 60.0f,           // 先从 90° 走到 60°
 *          .speed     = SERVO_SPEED_MID, // 走到终点用中速
 *          .seq       = {                // 到位后依次走这 4 个角度
 *              {120.0f, SERVO_SPEED_VERY_FAST}, // 这一步单独变速（猛地甩过去）
 *              {120.0f, 0, 300},                // ★在 120° 原地停 300ms
 *              { 60.0f },                       // 速度留空 = 用下面的 seq_speed
 *              {100.0f },
 *              { 80.0f },
 *          },
 *          .seq_len   = 5,
 *          .seq_speed = SERVO_SPEED_FAST, // 未单独指定的步都用它
 *      },
 *
 *   某轴不参与本情绪：整个填 0（seq_len=0 且 target=0 视为不动，见执行函数）。
 *   只走终点不做序列：填 target/speed，seq_len 留 0。
 *
 * ⚠️【幅度硬约束】相邻两个角度点的差值建议 ≥ 3~5°。舵机死区约 1°，
 *   加上齿隙，±1~2° 的微动在本硬件上肉眼看不出来（BUG-033 的成因）。
 *
 * ⚠️【想要停顿，必须用第三步 hold_ms，不能靠重复角度凑】（2026-09-18）
 *   重复同一个角度 = 零位移段，执行层只花 1 帧（约 20ms）就过，完全看不出停；
 *   ±1° 交替同理（同样钳成 1 帧）。正确写法是在要停的那个角度上填 hold_ms。
 *
 * @note 角度方向：head 左+(大) 右-(小) | L/R_arm 前+(大) 后-(小)
 */
typedef struct
{
    float target;                              ///< 终点角度（度）：从 90° 先走到这里
    uint32_t speed;                            ///< 走向终点的速度（SERVO_SPEED_xxx，值越大越慢）
    ActionSeqStep_t seq[ACTION_SEQ_MAX_STEPS]; ///< 动作序列：每步 = {角度, 该步速度, 到位后停留ms}
    uint8_t seq_len;                           ///< 序列实际步数（0 = 到终点就停，不做序列）
    uint32_t seq_speed;                        ///< 序列【默认】速度（每步 speed 留空时用它）
} ActionSeq_t;

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
    uint8_t strength; ///< 该段震动强度 0~100
    uint16_t on_ms;   ///< 该段震动时长（ms）
    uint16_t off_ms;  ///< 该段结束后的静默间隔（ms），最后一段可填 0
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

/**
 * @brief 调试用：启动「GIF + 舵机适配」循环播放任务
 *
 * 新建一个独立任务，遍历 g_emotion_matrix 循环播放数组里现有的每一条情绪
 * （切 GIF + 震动 + 三轴舵机序列 + 归中）。调试时只需在 interaction.c 的
 * g_emotion_matrix 里注释/增删条目，数组里留几条就轮着播几条，无需改枚举。
 *
 * @note 调用者：application.c（舵机测试模式），须在 interaction_manager_init 之后调用。
 */
void interaction_demo_start(void);
