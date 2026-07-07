#pragma once

#include <stdbool.h>

/**
 * @file standby.h
 * @brief 无活动待机（省电）模块
 *
 * 设备在 STANDBY_TIMEOUT_MS（默认 60s）内无任何活动（无对话、无触摸、无唤醒）后，
 * 自动进入待机：
 *   1. LCD 背光降到 BSP_LCD_BK_STANDBY_PCT（50%）；
 *   2. 关闭唤醒词监听（wake_word_stop，仅切 is_running 标志，不删任务）；
 *   3. 头部舵机做慢速左右摆动（呼吸/睡眠感）；
 * 退出待机仅靠「触摸头部铜箔」：恢复 100% 亮度 + 重开监听 + 头部归中。
 *
 * 设计要点（见 plan）：
 *   - 唯一判据是「距上次活动的时间」，绝不读背光亮度反推状态（避免循环依赖）。
 *   - 单一周期任务（1s）统一 poll 会话状态 + 判超时 + 待机摆头，活动源只更新时间戳，
 *     避免活动打点分散到多处导致遗漏。
 *
 * 模块依赖：bsp_board.h（背光/舵机）、custom_wake_word.h（监听开关）、session.h（会话状态）
 */

/**
 * @brief 初始化待机模块（创建周期监测任务）
 *
 * 必须在 LCD、唤醒词引擎、舵机管理器均初始化完成后调用。
 *
 * @return void
 * @note 调用者：application.c → application_init()（所有外设初始化之后）
 */
void standby_init(void);

/**
 * @brief 上报一次「活动」，刷新空闲计时（重置 60s 倒计时）
 *
 * 任何用户交互/系统活动都应调用此函数。若当前处于待机，本函数不直接退出待机
 * （退出由 standby_wake 显式触发），仅刷新时间戳。
 *
 * @return void
 * @note 调用者：wake_word_callback（唤醒命中）、ui_dispatch_touch_event（任意触摸）
 */
void standby_notify_activity(void);

/**
 * @brief 主动退出待机并恢复正常状态（恢复亮度 + 重开监听 + 头部归中）
 *
 * 幂等：非待机状态下调用无副作用。同时刷新活动计时。
 *
 * @return void
 * @note 调用者：ui_port.c → 摸头事件（TOUCH_EVENT_SHORT_HEAD）
 */
void standby_wake(void);

/**
 * @brief 查询当前是否处于待机状态
 *
 * @return true 待机中，false 正常
 * @note 调用者：ui_port.c（摸头时判断是否需要先唤醒而非触发情绪）
 */
bool standby_is_active(void);

/**
 * @brief 查询当前是否已进入二级（深度）待机
 *
 * 二级期间屏已关、舵机 PWM 已停（enter_deep_standby），不应再有任何代码路径
 * 主动驱动舵机/切图——否则会出现"关了又被点亮"（PWM 被重新 set_duty）。
 * standby_is_active() 在一级、二级期间均返回 true，无法区分二者，故新增本接口。
 *
 * @return true 已进入二级，false 未到二级（含未待机、仅一级）
 * @note 调用者：ui_port.c（一级/二级待机分支需分别处理，二级不再触发随机情绪动作）
 */
bool standby_is_deep_active(void);
