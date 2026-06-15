#pragma once

/**
 * @file games.h
 * @brief 小游戏模块框架接口
 *
 * 设计目标：先搭好「游戏列表 → 进入具体游戏 → 游戏内输入处理」的完整框架，
 * 三个游戏（打地鼠/跳一跳/赛车）首版仅占位（显示「敬请期待」），
 * 以后只需把游戏表中对应条目的 init/on_touch/render 函数指针换成真实实现，
 * 调用方（ui_port.c 状态机）无需改动。
 *
 * 输入约定（与全局统一三键模型一致）：
 *   - 左/右耳（PREV/NEXT_PAGE）：游戏内选择方向/选项
 *   - 头部（SHORT_HEAD）：确认/出招
 *   - 腹/背（SHORT_ABDOMEN/BACK）：退出游戏，返回游戏列表
 *   - 长按耳（LONG_*）：直接返回主界面
 */

#include "bsp/bsp_board.h" /* touch_event_t */

/**
 * @brief 游戏 ID 枚举（对应游戏表 g_games[] 的行索引）
 *
 * 扩展新游戏：在此追加枚举，并在 games.c 的 g_games[] 补充对应行。
 */
typedef enum
{
    GAME_WHACK = 0, ///< 0: 打地鼠
    GAME_JUMP,      ///< 1: 跳一跳
    GAME_RACE,      ///< 2: 赛车
    GAME_COUNT      ///< 游戏总数（同时作为数组长度）
} game_id_t;

/**
 * @brief 获取指定游戏的显示名称（供游戏列表渲染用）
 * @param id 游戏 ID
 * @return 游戏中文名；id 越界时返回空字符串
 */
const char *games_get_name(game_id_t id);

/**
 * @brief 启动指定游戏（进入 UI_VIEW_GAME 视图）
 *
 * 由 ui_port.c 在「游戏列表」中头部确认时调用。
 * 首版：调用对应游戏的 init()，显示占位画面「<游戏名> 敬请期待」。
 *
 * @param id 目标游戏 ID
 */
void games_start(game_id_t id);

/**
 * @brief 游戏运行视图（UI_VIEW_GAME）的触摸输入总入口
 *
 * 由 ui_port.c 在 UI_VIEW_GAME 分支统一转发触摸事件。
 * 首版占位逻辑：腹/背 或 长按耳 → 退出游戏；其余事件忽略。
 *
 * @param event 触摸事件
 */
void games_handle_touch(touch_event_t event);

/**
 * @brief 停止当前游戏并释放其资源（面板、定时器等）
 *
 * 由 ui_port.c 在所有「离开 UI_VIEW_GAME」的路径调用：
 *   腹/背退出、长按耳回主界面、功能层空闲超时。
 * 幂等：未在游戏中或当前游戏无清理需求时为空操作。
 * 占位游戏无资源，真实游戏（如打地鼠）借此删除 LVGL 对象与定时器，避免泄漏。
 */
void games_stop(void);
