#pragma once

/**
 * @file game_race.h
 * @brief 赛车（Racing）游戏实现接口
 *
 * 架构与打地鼠（game_whack.c）同源，复用同一套四子状态机与生命周期约定，
 * ui_port 只认一个 UI_VIEW_GAME。
 *
 * 横向赛道（320×240 横屏）：上下两条车道横向排开，玩家车贴左侧，
 * 敌车从右侧屏外进入向左移动；玩家左右耳切换上/下车道躲避，撞车即结束。
 *
 * 四子状态机：
 *   ① 难度选择  简单/一般/困难（左耳上移/右耳下移/头部确认）
 *   ② 3 秒倒计时 大数字 3→2→1→GO（期间忽略输入）
 *   ③ 游戏进行  单局 30 秒，敌车随机车道/随机间隔/随机素材自右向左
 *               · 每 10 秒提速（速度+、生成间隔×80%，带上/下限）
 *               · 躲过一辆（移出左屏）+1 分；撞车（矩形重叠）立即结束
 *               · 撞车：红屏闪烁 200ms 后进结算
 *   ④ 结算      得分 + 高分刷新提示；头部=重玩回难度选择
 *
 * 玩法参数全部宏化，见 race_sprites.h。
 * 高分按难度分别持久化到 NVS（命名空间 "race"，刷新才写，低分不动）。
 *
 * 输入（由 games.c → ui_port.c 的 UI_VIEW_GAME 分支转发）：
 *   - TOUCH_EVENT_SHORT_PREV_PAGE（左铜箔）：难度页上移 / 游戏中切上车道
 *   - TOUCH_EVENT_SHORT_NEXT_PAGE（右铜箔）：难度页下移 / 游戏中切下车道
 *   - TOUCH_EVENT_SHORT_HEAD（头部）：难度页确认 / 结算页重玩
 *   （长按头部=退出回功能盘，由 ui_port.c 统一拦截，不到这里）
 *
 * 生命周期：race_start() 建面板+启引擎，落到难度选择；
 *   race_stop() 删全部定时器+面板，幂等，由 games_stop() 在所有退出路径调用。
 *   三个接口均自行加锁（LVGL 端口为递归锁，任意上下文调用安全）。
 */

#include "bsp/bsp_board.h" /* touch_event_t */

/** @brief 进入赛车：创建面板、复位状态、落到难度选择界面 */
void race_start(void);

/** @brief 游戏内触摸输入（左/右耳切上/下车道），其余事件按子状态处理 */
void race_touch(touch_event_t event);

/** @brief 退出赛车：删除定时器与面板（幂等，未运行时为空操作） */
void race_stop(void);
