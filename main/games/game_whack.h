#pragma once

/**
 * @file game_whack.h
 * @brief 打地鼠（Whac-A-Mole）游戏实现接口 v3
 *
 * v3 设计（四子状态机，全部在本模块内部，ui_port 只认一个 UI_VIEW_GAME）：
 *   ① 难度选择  简单/一般/困难（左耳上移/右耳下移/头部确认）
 *   ② 3 秒倒计时 大数字 3→2→1→GO（期间忽略输入，防提前点击）
 *   ③ 游戏进行  单局 30 秒，2 洞同时只 1 只地鼠；左耳打左洞/右耳打右洞
 *               · 每 10 秒提速（停留时间、出现间隔各乘 80%，带下限）
 *               · 空敲（敲到空洞）扣分 + 记失误；同一地鼠只计一次分
 *               · 命中：眩晕星星 + 直接消失 + 震动（bsp_motor_pulse）
 *   ④ 结算      得分/击中/漏/空敲/准确率 + 高分刷新提示；头部=重玩回难度选择
 *
 * 难度与玩法参数全部宏化，见 whack_sprites.h。
 * 高分按难度分别持久化到 NVS（命名空间 "whack"，刷新才写，低分不动）。
 *
 * 输入（由 games.c → ui_port.c 的 UI_VIEW_GAME 分支转发）：
 *   - TOUCH_EVENT_SHORT_PREV_PAGE（左铜箔）：难度页上移 / 游戏中打左洞
 *   - TOUCH_EVENT_SHORT_NEXT_PAGE（右铜箔）：难度页下移 / 游戏中打右洞
 *   - TOUCH_EVENT_SHORT_HEAD（头部）：难度页确认 / 结算页重玩
 *   （长按头部=退出回功能盘，由 ui_port.c 统一拦截，不到这里）
 *
 * 渲染：本模块自持一组 LVGL 对象（面板/地洞/地鼠/锤子/星星/HUD/居中文字/提示），
 *   四个子界面复用同一组对象，靠显隐与文字切换，避免反复建删。
 *
 * 生命周期：whack_start() 建面板+启引擎，落到难度选择界面（不直接开打）；
 *   whack_stop() 删全部定时器+面板，幂等，由 games_stop() 在所有退出路径调用。
 *   三个接口均自行加锁（LVGL 端口为递归锁，任意上下文调用安全）。
 */

#include "bsp/bsp_board.h" /* touch_event_t */

/** @brief 进入打地鼠：创建面板、复位状态、启动出洞/引擎/倒计时定时器 */
void whack_start(void);

/** @brief 游戏内触摸输入（左/右铜箔直击对应洞的地鼠），其余事件忽略 */
void whack_touch(touch_event_t event);

/** @brief 退出打地鼠：删除定时器与面板（幂等，未运行时为空操作） */
void whack_stop(void);
