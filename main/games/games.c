/**
 * @file games.c
 * @brief 小游戏模块框架实现（首版：三个游戏占位）
 *
 * 当前阶段只搭框架：游戏表 + 启动 + 输入入口，三个游戏均显示「敬请期待」占位。
 * 以后实现具体玩法时，把对应 game_entry_t 的 init/on_touch 换成真实函数即可，
 * ui_port.c 的状态机分发逻辑无需改动。
 *
 * 渲染说明：游戏画面复用功能菜单的文字面板，通过 ui_port.c 暴露的
 * ui_menu_show_text() 显示标题+正文，games.c 不直接持有任何 LVGL 对象，
 * 保持与 UI 端口的低耦合。
 */
#include "games.h"
#include "game_whack.h"
#include "game_race.h"
#include "game_jump.h"
#include "ui/ui_port.h"
#include "esp_log.h"
#include <stddef.h>
#include <stdio.h>

static const char *TAG = "GAMES";

/* ═══════════════════════════════════════════════════════════════
 * 游戏表：每个游戏一行 {名称, 启动回调, 触摸回调}
 *   - on_start : 进入游戏时渲染初始画面（首版：占位提示）
 *   - on_touch : 游戏内触摸输入处理（首版：占位为空）
 * 扩展真实游戏时，替换对应行的函数指针即可。
 * ═══════════════════════════════════════════════════════════════ */
typedef struct
{
    const char *name;                       ///< 游戏中文名
    void (*on_start)(void);                 ///< 进入时渲染 / 初始化
    void (*on_touch)(touch_event_t event);  ///< 游戏内输入（不含通用退出，退出由 ui_port 统一处理）
    void (*on_stop)(void);                  ///< 退出时清理（删 LVGL 对象/定时器）；占位游戏为 NULL
} game_entry_t;

/* ── 占位实现：扩展新游戏时的「敬请期待」模板（三个游戏均已实现，暂未引用）──
 * 保留备用：新增游戏首版可先填 placeholder_start/touch 占位，做好后再换真实函数。
 * 加 __attribute__((unused)) 避免「全部游戏已实现、占位函数未被引用」时的编译警告。 */
static const char *s_placeholder_name = NULL; /* 当前占位游戏名，供占位渲染读取 */

__attribute__((unused)) static void placeholder_start(void)
{
    char buf[48];
    /* 正文提示：游戏未完成 + 退出方式 */
    snprintf(buf, sizeof(buf), "敬请期待\n\n摸肚子返回");
    ui_menu_show_text(s_placeholder_name ? s_placeholder_name : "游戏", buf);
}

__attribute__((unused)) static void placeholder_touch(touch_event_t event)
{
    (void)event; /* 占位阶段游戏内无任何交互，退出由 ui_port 统一处理 */
}

static const game_entry_t g_games[GAME_COUNT] = {
    /* 打地鼠：已接入真实实现（game_whack.c） */
    [GAME_WHACK] = {"打地鼠", whack_start, whack_touch, whack_stop},
    /* 跳一跳：已接入真实实现（game_jump.c） */
    [GAME_JUMP]  = {"跳一跳", jump_start, jump_touch, jump_stop},
    /* 赛车：已接入真实实现（game_race.c） */
    [GAME_RACE]  = {"赛车",   race_start, race_touch, race_stop},
};

/* 当前正在运行的游戏 ID（-1 表示未在游戏中） */
static int s_current_game = -1;

/* ═══════════════════════════════════════════════════════════════
 * 对外接口
 * ═══════════════════════════════════════════════════════════════ */
const char *games_get_name(game_id_t id)
{
    if (id < 0 || id >= GAME_COUNT)
        return "";
    return g_games[id].name;
}

void games_start(game_id_t id)
{
    if (id < 0 || id >= GAME_COUNT)
    {
        ESP_LOGW(TAG, "非法游戏 ID: %d", (int)id);
        return;
    }
    s_current_game = (int)id;
    s_placeholder_name = g_games[id].name;
    ESP_LOGI(TAG, "进入游戏: %s", g_games[id].name);
    if (g_games[id].on_start)
        g_games[id].on_start();
}

void games_handle_touch(touch_event_t event)
{
    /* 通用退出（长按头部）由 ui_port.c 的 UI_VIEW_GAME 分支统一拦截，不到这里。
     * 此处只转发给当前游戏的专属输入逻辑。 */
    if (s_current_game < 0 || s_current_game >= GAME_COUNT)
        return;
    if (g_games[s_current_game].on_touch)
        g_games[s_current_game].on_touch(event);
}

int games_get_current(void)
{
    return s_current_game;
}

void games_stop(void)
{
    if (s_current_game < 0 || s_current_game >= GAME_COUNT)
        return; /* 未在游戏中，幂等 */
    if (g_games[s_current_game].on_stop)
        g_games[s_current_game].on_stop();
    ESP_LOGI(TAG, "退出游戏: %s", g_games[s_current_game].name);
    s_current_game = -1;
    s_placeholder_name = NULL;
}
