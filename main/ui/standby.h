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
 * ⚠️⚠️【调用者的栈必须在内部 SRAM，禁止从 PSRAM 栈的任务调用】⚠️⚠️
 *   本函数内部会调 bsp_board_codec_exit_lowpower() 读 NVS 恢复音量，最终走到
 *   spi_flash_disable_interrupts_caches_and_other_cpu()：关主 flash cache 的
 *   同时 PSRAM 也不可访问，此时若当前任务栈在 PSRAM，返回地址与局部变量全部
 *   读不到，IDF 会断言 abort（esp_task_stack_is_sane_cache_disabled，
 *   cache_utils.c:152）。同 reminder.c 里 nvs_save_weather_data 那条铁律。
 *   已知安全调用者：standby_task、触摸任务（两者栈均在内部 SRAM）。
 *   ★ reminder_task 的栈在 PSRAM，绝不可直接调用本函数或 standby_notify_activity()，
 *     请改用下方的 standby_request_wake()。
 *
 * @return void
 * @note 调用者：ui_port.c → 摸头事件（TOUCH_EVENT_SHORT_HEAD）
 */
void standby_wake(void);

/**
 * @brief 唤醒来源：决定 standby_task 唤醒完成后紧接着要把哪一屏画上去
 *
 * 【为什么要带来源】提醒类唤醒必须"醒了就直接显示提醒页"。实测若不带来源、
 * 让 reminder 自己在下一拍再去切页，中间会隔着整个唤醒转场（约 2.6 秒）+ 一拍
 * 轮询，而这段时间里 standby_wake() 已经恢复了 GIF 轮播并开始切图，随后
 * ui_show_alarm_ringing() 去抢 LVGL 锁只等 100ms 抢不过，直接放弃切页 ——
 * 实测现象＝「闹钟在响、屏幕却停在待机 GIF 界面」。
 */
typedef enum
{
    STANDBY_WAKE_SRC_GENERIC = 0,     ///< 通用唤醒（转场挂起补偿等），醒了就好，不额外画
    STANDBY_WAKE_SRC_COUNTDOWN_EXPIRE, ///< 倒计时到期，醒后直接显示番茄钟到期画面
    STANDBY_WAKE_SRC_ALARM_RING,       ///< 闹钟响铃，醒后直接显示闹钟响铃画面
} standby_wake_src_t;

/**
 * @brief 请求退出待机（异步，任何任务都可安全调用）（2026-08-25 新增）
 *
 * 只做一次原子置位，不碰 flash、不碰 NVS、不碰 LVGL、不阻塞，因此
 * 【栈在 PSRAM 的任务也可以安全调用】——这正是它存在的唯一理由。
 * 真正的唤醒动作由 standby_task（栈在内部 SRAM）在下一拍（≤1 秒）执行。
 *
 * 典型场景：reminder_task 检测到倒计时/闹钟即将触发，但设备正处于深度待机，
 * 需要先亮屏再显示提醒——它自己不能调 standby_wake()（会踩 PSRAM 栈 + 关 cache
 * 的断言），于是发一个请求，等 standby_task 醒完之后再照常投递到期事件。
 *
 * 幂等：重复调用只是把同一个标志重复置 true。非待机状态下调用也无副作用
 * （standby_wake 自身有 !s_deep_standby 早退）。
 *
 * @param src 唤醒来源。提醒类（COUNTDOWN_EXPIRE / ALARM_RING）会让 standby_task
 *            在唤醒完成后【立刻】把对应提醒页画上，不给 GIF 轮播抢屏的机会。
 *
 * @note 调用者：reminder.c poll_timer_callback（待机中到期，先请求亮屏）
 */
void standby_request_wake(standby_wake_src_t src);

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
