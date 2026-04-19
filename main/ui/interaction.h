
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
