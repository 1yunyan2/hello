#pragma once

/**
 * @file game_jump.h
 * @brief 跳一跳（Jump）游戏实现接口
 *
 * 架构与打地鼠（game_whack.c）/赛车（game_race.c）同源，复用同一套四子状态机
 * 与生命周期约定，ui_port 只认一个 UI_VIEW_GAME。
 *
 * 玩法（320×240 横屏，纯 LVGL 绘制，零新增图片）：
 *   小人站在当前台上，前方随机距离有下一个台子。
 *   长按左/右耳「蓄力」——按住越久蓄得越满（读 bsp_touch_last_page_hold_ms()），
 *   松手起跳，小人沿抛物线飞出一段距离：
 *     · 落在下一台 → +1 分（落中心额外加分），整体左移，生成更前方的台子
 *     · 没够着 / 跳过头 → 掉落 → 结算（无限闯关，跳失败即结束，不限时）
 *   难度递增：台子宽度随分数线性缩小（落点越来越小）。
 *
 * 四子状态机：
 *   ① JS_SELECT    难度选择（左耳上移/右耳下移/头部确认）
 *   ② JS_COUNTDOWN 3 秒倒计时（大数字 3→2→1→GO，期间忽略输入）
 *   ③ JS_PLAYING   游戏进行（含「待蓄力 / 飞行中 / 掉落中」三个内部子相）
 *   ④ JS_RESULT    结算（头部短按=重玩回难度选择，长按头部退出由 ui_port 拦截）
 *
 * 玩法参数全部宏化，见 jump_sprites.h。
 * 高分按难度分别持久化到 NVS（命名空间 "jump"，刷新才写，低分不动）。
 *
 * 输入（由 games.c → ui_port.c 的 UI_VIEW_GAME 分支转发）：
 *   - TOUCH_EVENT_SHORT_PREV_PAGE / SHORT_NEXT_PAGE（左/右铜箔）：
 *       难度页 上移/下移；游戏中 = 一次「蓄力起跳」（时长由触摸层 held 给出）
 *   - TOUCH_EVENT_SHORT_HEAD（头部）：难度页确认 / 结算页重玩
 *   （长按头部=退出回功能盘，由 ui_port.c 统一拦截，不到这里）
 *
 * 生命周期：jump_start() 建面板+启引擎，落到难度选择；
 *   jump_stop() 删全部定时器+面板，幂等，由 games_stop() 在所有退出路径调用。
 *   三个接口均自行加锁（LVGL 端口为递归锁，任意上下文调用安全）。
 */

#include "bsp/bsp_board.h" /* touch_event_t */

/** @brief 进入跳一跳：创建面板、复位状态、落到难度选择界面 */
void jump_start(void);

/** @brief 游戏内触摸输入（左/右耳蓄力起跳），其余事件按子状态处理 */
void jump_touch(touch_event_t event);

/** @brief 退出跳一跳：删除定时器与面板（幂等，未运行时为空操作） */
void jump_stop(void);
