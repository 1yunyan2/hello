
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
    EMO_HAPPY = 0,   ///< 0: 开心（摇头+挥臂，2次短促震动）
    EMO_CURIOUS,     ///< 1: 好奇（缓慢歪头，1次轻震动）
    EMO_TSUNDERE,    ///< 2: 傲娇（轻微侧头，无震动）
    EMO_TICKLISH,    ///< 3: 怕痒（快速抖动，连续震动）
    EMO_SLEEPY,      ///< 4: 犯困
    EMO_GRIEVED,     ///< 5: 委屈
    EMO_COMFORTABLE, ///< 6: 舒服
    EMO_ACT_CUTE,    ///< 7: 撒娇
    EMO_ANGRY,       ///< 8: 生气
    EMO_SHY,         ///< 9: 害羞
    EMO_SURPRISED,   ///< 10: 惊喜
    EMO_SLUGGISH,    ///< 11: 慵懒
    EMO_HEALING,     ///< 12: 治愈
    EMO_EXCITED      ///< 13: 兴奋
    // ... 在这里继续添加表格里剩下的情绪
} robot_emotion_t;

/**
 * @brief 触发机器人的全套情绪表现（屏幕动画 + 音效 + 震动 + 三轴舵机动作）
 *
 * 在 g_emotion_matrix[] 中查找 target_emotion 对应的行，按顺序执行：
 * 屏幕动画 → 音效 → 震动马达 → 三轴舵机同步动作序列 → 舵机归中
 *
 * @param target_emotion 目标情绪 ID（robot_emotion_t 枚举值）
 * @return void（若未找到对应情绪，仅打印 LOGE 并返回）
 *
 * @note 调用者：session.c 或上层逻辑模块，在 AI 返回情绪指令时触发
 * @note 此函数为同步阻塞：舵机动作和震动期间会阻塞调用任务
 */
void bsp_interaction_play(robot_emotion_t target_emotion);
