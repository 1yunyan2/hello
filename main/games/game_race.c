/**
 * @file game_race.c
 * @brief 赛车（Racing）游戏实现
 *
 * 全部使用图片素材（s1 背景 + s2 玩家车 + s3/s4/s5 敌车），结构与打地鼠
 * （game_whack.c）同源，复用四子状态机与生命周期约定。
 *
 * 竖向赛道（320×240 横屏，敌车自上而下落）：
 *   - 左车道 x=RACE_LANE_LEFT_X，右车道 x=RACE_LANE_RIGHT_X，中间画白色竖向滚动虚线
 *   - 玩家车 s2 固定贴底部 y=RACE_PLAYER_Y，可在左/右车道间瞬间切换
 *   - 敌车从顶部屏外 y=-30 进入，向下移动；移出底部（y≥240）算躲过 +1 分
 *   - 撞车（玩家与敌车矩形重叠）→ 红屏闪烁 → 立即结算
 *
 * 子状态机（全部在本模块内部，ui_port 只认一个 UI_VIEW_GAME）：
 *   ① RS_SELECT    难度选择（左耳上移/右耳下移/头部确认）
 *   ② RS_COUNTDOWN 3 秒倒计时（大数字 3→2→1→GO，期间忽略输入）
 *   ③ RS_PLAYING   游戏进行 30 秒（左耳切左道/右耳切右道）
 *   ④ RS_RESULT    结算（头部=重玩回难度选择，腹/背退出由 ui_port 拦截）
 *
 * 难度与玩法参数全部宏化，见 race_sprites.h。
 * 高分按难度分别持久化到 NVS（命名空间 "race"，刷新才写，低分不动）。
 */
#include "game_race.h"
#include "race_sprites.h"
#include "ui/ui_port.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bsp/bsp_config.h"
#include "bsp/bsp_board.h"

LV_FONT_DECLARE(font_cn_16);
LV_FONT_DECLARE(font_cn_32); /* 32px 中文大字库（仅含游戏用字），用于居中大文字 */

static const char *TAG = "RACE";

/* ── 引擎帧周期（动画/物理），与打地鼠一致 20fps ── */
#define ENGINE_MS 50

/* ── 车道编号（竖向赛道：左道/右道，按 x 区分）── */
#define LANE_LEFT 0  /* 左车道 */
#define LANE_RIGHT 1 /* 右车道 */
#define LANE_COUNT 2

/* ── 难度档（数组下标，与 NVS key 一一对应）── */
typedef enum
{
    DIFF_EASY = 0,
    DIFF_NORMAL,
    DIFF_HARD,
    DIFF_COUNT,
} race_diff_t;

/* 难度参数表：{敌车基准速度px/帧, 生成间隔基准ms, 中文名, NVS key} */
typedef struct
{
    int speed_px;
    int spawn_ms;
    const char *name;
    const char *nvs_key;
} diff_param_t;

static const diff_param_t s_diff[DIFF_COUNT] = {
    [DIFF_EASY] = {RACE_SPEED_EASY, RACE_SPAWN_MS_EASY, "简单", "hi_easy"},
    [DIFF_NORMAL] = {RACE_SPEED_NORMAL, RACE_SPAWN_MS_NORMAL, "一般", "hi_normal"},
    [DIFF_HARD] = {RACE_SPEED_HARD, RACE_SPAWN_MS_HARD, "困难", "hi_hard"},
};

#define RACE_NVS_NS "race" /* NVS 命名空间 */

/* ── 子状态机 ── */
typedef enum
{
    RS_SELECT = 0, /* 难度选择 */
    RS_COUNTDOWN,  /* 3 秒倒计时 */
    RS_PLAYING,    /* 游戏进行 */
    RS_RESULT,     /* 结算 */
} race_screen_t;

/* ── 敌车对象池单元 ── */
typedef struct
{
    lv_obj_t *obj; /* 对应的 LVGL image，常驻，靠显隐复用 */
    int16_t x, y;  /* 左上角坐标 */
    int8_t lane;   /* 所在车道 LANE_LEFT / LANE_RIGHT */
    bool active;   /* 是否在场 */
} enemy_t;

/* ── 全局状态 ── */
static struct
{
    race_screen_t screen; /* 当前子界面 */
    race_diff_t diff;     /* 当前选中难度 */

    /* 本局动态参数（= 难度基准，运行中随 10 秒加速变化）*/
    int cur_speed_px;
    int cur_spawn_ms;
    int high_score; /* 当前难度历史最高分（进游戏时读 NVS）*/

    /* 倒计时 */
    int cd_value; /* 3 → 2 → 1 → 0(GO) */

    /* 对局数据 */
    bool over;
    int score;     /* 得分（每躲过一辆 +1）*/
    int elapsed_s; /* 已进行秒数，用于 10 秒加速 */

    /* 玩家 */
    int8_t player_lane; /* LANE_LEFT / LANE_RIGHT */

    /* 敌车对象池 */
    enemy_t enemies[RACE_MAX_ENEMIES];

    /* 分隔虚线滚动偏移（向左滚动）*/
    int dash_offset;

    /* 碰撞闪烁 */
    bool flash_active;
    uint32_t flash_start;
} g;

/* ── LVGL 对象 ── */
static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_bg = NULL;      /* 背景路面 s1 */
static lv_obj_t *s_player = NULL;  /* 玩家车 s2 */
static lv_obj_t *s_hud = NULL;     /* 游戏中顶部一行 HUD */
static lv_obj_t *s_center = NULL;  /* 居中大文字（难度/倒计时/结算共用）*/
static lv_obj_t *s_hint = NULL;    /* 底部操作提示 */
static lv_obj_t *s_divider = NULL; /* 中间车道分隔虚线 */
static lv_obj_t *s_flash = NULL;   /* 碰撞红屏闪烁遮罩 */

static lv_timer_t *s_engine_tmr = NULL; /* 动画/物理引擎 */
static lv_timer_t *s_spawn_tmr = NULL;  /* 敌车生成节拍 */
static lv_timer_t *s_clock_tmr = NULL;  /* 1Hz 倒计时 + 10 秒加速 */
static lv_timer_t *s_cd_tmr = NULL;     /* 开局 3 秒倒计时 */

/* 前置声明 */
static void enter_select(void);
static void enter_countdown(void);
static void enter_playing(void);
static void enter_result(void);
static void game_timers_stop(void);

/* ═══════════════════════════════════════════════════════════════
 * NVS 高分读写（按难度独立 key）
 *
 * 与打地鼠同样的前置依赖：触摸任务栈须在内部 SRAM，否则在该任务上下文
 * 读写 flash 会触发 cache disable 断言（BUG-010 家族）。
 * ═══════════════════════════════════════════════════════════════ */
#define RACE_NVS_ENABLED 1

static int highscore_load(race_diff_t d)
{
#if RACE_NVS_ENABLED
    nvs_handle_t h;
    int32_t v = 0;
    if (nvs_open(RACE_NVS_NS, NVS_READONLY, &h) == ESP_OK)
    {
        nvs_get_i32(h, s_diff[d].nvs_key, &v);
        nvs_close(h);
    }
    return (int)v;
#else
    (void)d;
    return 0;
#endif
}

/* 仅在刷新（score > 旧值）时写入，返回是否刷新 */
static bool highscore_save_if_better(race_diff_t d, int score)
{
    if (score <= g.high_score)
        return false;
#if RACE_NVS_ENABLED
    nvs_handle_t h;
    if (nvs_open(RACE_NVS_NS, NVS_READWRITE, &h) == ESP_OK)
    {
        nvs_set_i32(h, s_diff[d].nvs_key, (int32_t)score);
        nvs_commit(h);
        nvs_close(h);
    }
#endif
    g.high_score = score; /* 内存中仍更新，本局结算「新纪录」判定正常 */
    return true;
}

/* ═══════════════════════════════════════════════════════════════
 * 视觉辅助
 * ═══════════════════════════════════════════════════════════════ */

/* 取某车道对应的车左上角 x（竖向赛道：左道/右道按 x 区分）*/
static inline int lane_x(int8_t lane)
{
    return (lane == LANE_LEFT) ? RACE_LANE_LEFT_X : RACE_LANE_RIGHT_X;
}

/* 为敌车随机选一张素材（s3/s4/s5，颜色不同更好看）*/
static const lv_image_dsc_t *enemy_random_src(void)
{
    switch (esp_random() % 3)
    {
    case 0:
        return &s3;
    case 1:
        return &s4;
    default:
        return &s5;
    }
}

/* 把所有敌车藏起来并标记空闲（切界面清场）*/
static void enemies_hide_all(void)
{
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        g.enemies[i].active = false;
        if (g.enemies[i].obj)
            lv_obj_add_flag(g.enemies[i].obj, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 刷新玩家车位置（车道切换后调用）*/
static void player_refresh(void)
{
    if (s_player == NULL)
        return;
    lv_obj_set_pos(s_player, lane_x(g.player_lane), RACE_PLAYER_Y);
}

static void hud_refresh(void)
{
    if (s_hud == NULL)
        return;
    char buf[48];
    /* 高分显示取 max(当前分, 历史高分)：当前局超纪录时实时涨，
     * 但不修改 g.high_score 本身（保留给结算时的 NVS 刷新判定）。 */
    int show_high = (g.score > g.high_score) ? g.score : g.high_score;
    snprintf(buf, sizeof(buf), "分数%d  高分%d",
             g.score, show_high);
    lv_label_set_text(s_hud, buf);
}

/* ── 重画分隔虚线（竖直线，按 dash_offset 向下滚动）── */
static void divider_refresh(void)
{
    if (s_divider == NULL)
        return;

    /* 用一组 lv_point_precise_t 描述若干虚线段（竖直线，x 固定=0，
     * 对象本身定位在 RACE_DIVIDER_X，所以这里 x 都用 0 相对坐标）。
     * 滚动靠整体平移 dash_offset 实现，循环周期 = 段长+间隔。 */
    static lv_point_precise_t pts[2 * ((BSP_LCD_HEIGHT / (RACE_DASH_LEN + RACE_DASH_GAP)) + 2)];
    int n = 0;
    int period = RACE_DASH_LEN + RACE_DASH_GAP;
    /* 起点从 -period 开始，保证下滚时顶部不留缝 */
    for (int y = -period + g.dash_offset; y < BSP_LCD_HEIGHT; y += period)
    {
        if (n + 2 > (int)(sizeof(pts) / sizeof(pts[0])))
            break;
        pts[n].x = 0;
        pts[n].y = y;
        n++;
        pts[n].x = 0;
        pts[n].y = y + RACE_DASH_LEN;
        n++;
    }
    lv_line_set_points(s_divider, pts, n);
}

/* ═══════════════════════════════════════════════════════════════
 * 引擎：动画 + 物理（所有子界面共用，只在 PLAYING 推进对局逻辑）
 * ═══════════════════════════════════════════════════════════════ */
static void engine_cb(lv_timer_t *t)
{
    (void)t;

    /* ── 碰撞闪烁：任何子界面都让其自然结束 ── */
    if (g.flash_active)
    {
        if (s_flash && (lv_tick_get() - g.flash_start) < RACE_FLASH_MS)
        {
            /* 闪烁期间保持红屏显示 */
            lv_obj_clear_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            g.flash_active = false;
            if (s_flash)
                lv_obj_add_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
            /* 闪烁结束才真正切结算（撞车时已置 over，此处展示结算文字）*/
            if (g.over && g.screen == RS_PLAYING)
                enter_result();
        }
    }

    if (g.screen != RS_PLAYING || g.over)
        return;

    bool dirty = false;

    /* ── 分隔虚线向下滚动（速度与敌车速度挂钩，营造车辆向下的相对运动）── */
    g.dash_offset += g.cur_speed_px / RACE_DASH_SPEED_DIV;
    if (g.cur_speed_px / RACE_DASH_SPEED_DIV == 0)
        g.dash_offset += 1; /* 至少滚 1px，避免静止 */
    int period = RACE_DASH_LEN + RACE_DASH_GAP;
    while (g.dash_offset >= period)
        g.dash_offset -= period;
    divider_refresh();

    /* ── 敌车移动 + 碰撞/计分 ── */
    int player_x = lane_x(g.player_lane);
    int player_y = RACE_PLAYER_Y;

    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        enemy_t *e = &g.enemies[i];
        if (!e->active)
            continue;

        e->y += g.cur_speed_px; /* 向下移动 */

        /* 移出底部：躲过，加分回收 */
        if (e->y >= RACE_ENEMY_GONE_Y)
        {
            e->active = false;
            lv_obj_add_flag(e->obj, LV_OBJ_FLAG_HIDDEN);
            g.score++;
            /* 注意：这里不可提前把 g.high_score 顶成 g.score，否则结算时
             * highscore_save_if_better 的 (score <= high_score) 判定恒成立，
             * 永远写不进 NVS。HUD 显示的「高分」改用 max(score,high) 实时算。 */
            dirty = true;
            continue;
        }

        /* 碰撞检测（矩形重叠）：玩家与敌车同在屏上 */
        if (e->x < player_x + RACE_CAR_W && e->x + RACE_CAR_W > player_x &&
            e->y < player_y + RACE_CAR_H && e->y + RACE_CAR_H > player_y)
        {
            /* 撞车：立即结束本局，红屏闪烁后进结算。
             * 高分写入统一放到 enter_result()，此处不写——否则会提前覆盖
             * g.high_score，导致结算无法判断「是否破纪录」与显示旧纪录。*/
            g.over = true;
            g.flash_active = true;
            g.flash_start = lv_tick_get();
            if (s_flash)
                lv_obj_clear_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
            ESP_LOGI(TAG, "撞车! 得分=%d", g.score);
            return; /* 立即停止本帧后续更新 */
        }

        lv_obj_set_pos(e->obj, e->x, e->y);
    }

    if (dirty)
        hud_refresh();
}

/* ── 敌车生成节拍：找空闲槽位，随机车道/素材生成一辆 ── */
static void spawn_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != RS_PLAYING || g.over)
        return;

    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        enemy_t *e = &g.enemies[i];
        if (e->active)
            continue;

        e->lane = (esp_random() % LANE_COUNT) == 0 ? LANE_LEFT : LANE_RIGHT;
        e->x = lane_x(e->lane);        /* 车道决定 x */
        e->y = RACE_ENEMY_SPAWN_Y;     /* 从顶部屏外进入 */
        e->active = true;

        lv_image_set_src(e->obj, enemy_random_src()); /* 随机换一张敌车图 */
        lv_obj_set_pos(e->obj, e->x, e->y);
        lv_obj_clear_flag(e->obj, LV_OBJ_FLAG_HIDDEN);
        break; /* 一拍只生成一辆 */
    }
}

/* ── 1Hz 时钟：计时 + 每 10 秒持续加速（无时间上限，撞车才结束）── */
static void clock_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != RS_PLAYING || g.over)
        return;

    g.elapsed_s++;

    /* 每 10 秒提速：速度 +量（带上限），生成间隔 ×80%（带下限）。
     * 不设时间上限，速度会一路逼近 RACE_SPEED_MAX 越来越快。*/
    if (g.elapsed_s % RACE_ACCEL_EVERY_S == 0)
    {
        g.cur_speed_px += RACE_ACCEL_SPEED_ADD;
        if (g.cur_speed_px > RACE_SPEED_MAX)
            g.cur_speed_px = RACE_SPEED_MAX;
        g.cur_spawn_ms = g.cur_spawn_ms * RACE_ACCEL_SPAWN_PCT / 100;
        if (g.cur_spawn_ms < RACE_SPAWN_MS_MIN)
            g.cur_spawn_ms = RACE_SPAWN_MS_MIN;
        if (s_spawn_tmr)
            lv_timer_set_period(s_spawn_tmr, g.cur_spawn_ms);
        ESP_LOGI(TAG, "提速@%ds: speed=%d spawn=%d", g.elapsed_s, g.cur_speed_px, g.cur_spawn_ms);
    }

    hud_refresh();
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：① 难度选择
 * ═══════════════════════════════════════════════════════════════ */
static void select_render(void)
{
    if (s_center == NULL)
        return;
    char buf[96];
    /* 竖排三选项：当前项前伸（无缩进），其余项缩进两格，靠位置区分选中态。 */
    snprintf(buf, sizeof(buf),
             "选择难度\n%s简单\n%s一般\n%s困难",
             g.diff == DIFF_EASY ? "" : "    ",
             g.diff == DIFF_NORMAL ? "" : "    ",
             g.diff == DIFF_HARD ? "" : "    ");
    lv_label_set_text(s_center, buf);
    lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
}

static void enter_select(void)
{
    g.screen = RS_SELECT;
    game_timers_stop(); /* 选择期间不跑对局定时器 */
    enemies_hide_all();
    if (s_player)
        lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);
    if (s_divider)
        lv_obj_add_flag(s_divider, LV_OBJ_FLAG_HIDDEN);
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    if (s_hint)
        lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
    select_render();
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：② 倒计时（3 → 2 → 1 → GO!）
 * ═══════════════════════════════════════════════════════════════ */
static void countdown_render(void)
{
    if (s_center == NULL)
        return;
    char buf[16];
    if (g.cd_value > 0)
        snprintf(buf, sizeof(buf), "%d", g.cd_value);
    else
        snprintf(buf, sizeof(buf), "GO!");
    lv_label_set_text(s_center, buf);
    lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
}

static void countdown_cb(lv_timer_t *t)
{
    (void)t;
    g.cd_value--;
    if (g.cd_value >= 0)
    {
        countdown_render(); /* 2、1、GO! */
    }
    else
    {
        /* GO! 显示一拍后真正开打 */
        if (s_cd_tmr)
        {
            lv_timer_del(s_cd_tmr);
            s_cd_tmr = NULL;
        }
        enter_playing();
    }
}

static void enter_countdown(void)
{
    g.screen = RS_COUNTDOWN;
    if (s_hint)
        lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
    /* 提前露出赛道氛围：分隔虚线 + 玩家车 */
    if (s_divider)
        lv_obj_clear_flag(s_divider, LV_OBJ_FLAG_HIDDEN);
    g.player_lane = LANE_LEFT;
    player_refresh();
    if (s_player)
        lv_obj_clear_flag(s_player, LV_OBJ_FLAG_HIDDEN);
    g.dash_offset = 0;
    divider_refresh();

    g.cd_value = RACE_COUNTDOWN_FROM;
    countdown_render(); /* 先显示 3 */
    if (s_cd_tmr)
    {
        lv_timer_del(s_cd_tmr);
        s_cd_tmr = NULL;
    }
    s_cd_tmr = lv_timer_create(countdown_cb, RACE_COUNTDOWN_STEP_MS, NULL);
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：③ 游戏进行
 * ═══════════════════════════════════════════════════════════════ */
static void enter_playing(void)
{
    g.screen = RS_PLAYING;
    g.over = false;

    /* 复位对局数据 */
    g.score = 0;
    g.elapsed_s = 0;
    g.player_lane = LANE_LEFT;
    g.dash_offset = 0;
    g.flash_active = false;

    /* 本局动态参数 = 当前难度基准 */
    g.cur_speed_px = s_diff[g.diff].speed_px;
    g.cur_spawn_ms = s_diff[g.diff].spawn_ms;
    g.high_score = highscore_load(g.diff);

    enemies_hide_all();
    player_refresh();
    if (s_player)
        lv_obj_clear_flag(s_player, LV_OBJ_FLAG_HIDDEN);
    if (s_divider)
        lv_obj_clear_flag(s_divider, LV_OBJ_FLAG_HIDDEN);
    if (s_center)
        lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    if (s_hud)
        lv_obj_clear_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    hud_refresh();

    /* 启动对局定时器 */
    if (s_spawn_tmr == NULL)
        s_spawn_tmr = lv_timer_create(spawn_cb, g.cur_spawn_ms, NULL);
    else
    {
        lv_timer_set_period(s_spawn_tmr, g.cur_spawn_ms);
        lv_timer_resume(s_spawn_tmr);
    }

    if (s_clock_tmr == NULL)
        s_clock_tmr = lv_timer_create(clock_cb, 1000, NULL);
    else
        lv_timer_resume(s_clock_tmr);

    ESP_LOGI(TAG, "开打: 难度=%s speed=%d spawn=%d",
             s_diff[g.diff].name, g.cur_speed_px, g.cur_spawn_ms);
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：④ 结算
 * ═══════════════════════════════════════════════════════════════ */
static void enter_result(void)
{
    g.over = true;
    g.screen = RS_RESULT;

    /* 暂停对局定时器，立即清场禁点 */
    if (s_spawn_tmr)
        lv_timer_pause(s_spawn_tmr);
    if (s_clock_tmr)
        lv_timer_pause(s_clock_tmr);
    enemies_hide_all();
    if (s_player)
        lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);
    if (s_divider)
        lv_obj_add_flag(s_divider, LV_OBJ_FLAG_HIDDEN);

    /* 先记下写入前的历史最高（本局开始时从 NVS load 的旧值），
     * 用于结算展示「历史最高」与判断是否破纪录；之后再尝试写入 NVS。*/
    int old_high = g.high_score;
    bool refreshed = highscore_save_if_better(g.diff, g.score);

    if (s_center)
    {
        char buf[120];
        if (refreshed)
        {
            /* 破纪录：历史最高显示为本局新分（已 = g.high_score）*/
            snprintf(buf, sizeof(buf),
                     "撞车了!\n得分 %d\n历史最高 %d\n突破记录!",
                     g.score, g.high_score);
        }
        else
        {
            snprintf(buf, sizeof(buf),
                     "撞车了!\n得分 %d\n历史最高 %d",
                     g.score, old_high);
        }
        lv_label_set_text(s_center, buf);
        lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    if (s_hint)
        lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "结算: 难度=%s 得分=%d 历史最高=%d %s",
             s_diff[g.diff].name, g.score, g.high_score,
             refreshed ? "[突破记录]" : "");
}

/* ═══════════════════════════════════════════════════════════════
 * 面板构建（一次性，所有子界面共用同一组 LVGL 对象）
 * ═══════════════════════════════════════════════════════════════ */
static void build_panel(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL)
    {
        ESP_LOGE(TAG, "lv_screen_active 返回 NULL");
        return;
    }

    /* ── 面板 ── */
    s_panel = lv_obj_create(scr);
    lv_obj_set_size(s_panel, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_align(s_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(0x202020), 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_panel, 0, 0);
    lv_obj_set_style_pad_all(s_panel, 0, 0);
    lv_obj_set_style_radius(s_panel, 0, 0);
    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(s_panel, LV_SCROLLBAR_MODE_OFF);

    /* ── 背景路面（全屏铺底，垫在最底层）── */
#if RACE_BG_USE_IMG
    /* 图片模式：从外挂 flash 读背景图 "S:/img/s1.bin"（不再编进 app 分区）。
     * 图片由 PNG 经 1.py 转 RGB565A8 .bin，2.py 打包、3.py 烧进 /S（=S: 盘符）。
     * 注意：此 lv_image_set_src 必须在 LVGL 线程调用（本函数由 UI 流程触发，
     * 已在 LVGL 线程，安全）；SPIRAM 栈任务直接调会在禁 cache 期崩（BUG-010）。*/
    s_bg = lv_image_create(s_panel);
    lv_image_set_src(s_bg, "S:/img/s1.bin");
    lv_obj_set_pos(s_bg, 0, 0);
#else
    /* 色块模式：深灰路面 + 上下两侧绿化带，纯 LVGL 绘制 */
    s_bg = lv_obj_create(s_panel);
    lv_obj_set_size(s_bg, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_set_pos(s_bg, 0, 0);
    lv_obj_set_style_bg_color(s_bg, lv_color_hex(0x3A3A3A), 0); /* 沥青灰路面 */
    lv_obj_set_style_bg_opa(s_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_bg, 0, 0);
    lv_obj_set_style_radius(s_bg, 0, 0);
    lv_obj_clear_flag(s_bg, LV_OBJ_FLAG_SCROLLABLE);
    /* 上下两条绿化带（路肩）：各 30px 高的绿色矩形 */
    lv_obj_t *grass_top = lv_obj_create(s_bg);
    lv_obj_set_size(grass_top, BSP_LCD_WIDTH, 30);
    lv_obj_set_pos(grass_top, 0, 0);
    lv_obj_set_style_bg_color(grass_top, lv_color_hex(0x2D7D32), 0);
    lv_obj_set_style_border_width(grass_top, 0, 0);
    lv_obj_set_style_radius(grass_top, 0, 0);
    lv_obj_clear_flag(grass_top, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_t *grass_bot = lv_obj_create(s_bg);
    lv_obj_set_size(grass_bot, BSP_LCD_WIDTH, 30);
    lv_obj_set_pos(grass_bot, 0, BSP_LCD_HEIGHT - 30);
    lv_obj_set_style_bg_color(grass_bot, lv_color_hex(0x2D7D32), 0);
    lv_obj_set_style_border_width(grass_bot, 0, 0);
    lv_obj_set_style_radius(grass_bot, 0, 0);
    lv_obj_clear_flag(grass_bot, LV_OBJ_FLAG_SCROLLABLE);
#endif

    /* ── 中间分隔虚线（白色竖直虚线，靠 lv_line 多段实现）──
     * 对象定位在 x=RACE_DIVIDER_X，虚线点用相对 x=0；y 随滚动变化。*/
    s_divider = lv_line_create(s_panel);
    lv_obj_set_pos(s_divider, RACE_DIVIDER_X, 0);
    lv_obj_set_style_line_color(s_divider, lv_color_white(), 0);
    lv_obj_set_style_line_width(s_divider, 3, 0);
    lv_obj_set_style_line_rounded(s_divider, false, 0);
    lv_obj_add_flag(s_divider, LV_OBJ_FLAG_HIDDEN);

    /* ── 玩家车 s2（贴底部，x 随车道切换，原始尺寸不放大）── */
    s_player = lv_image_create(s_panel);
    lv_image_set_src(s_player, &s2);
    lv_obj_set_pos(s_player, RACE_LANE_LEFT_X, RACE_PLAYER_Y);
    lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);

    /* ── 敌车对象池（常驻 image，靠显隐 + 换图复用，原始尺寸）── */
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        g.enemies[i].obj = lv_image_create(s_panel);
        lv_image_set_src(g.enemies[i].obj, &s3);
        lv_obj_add_flag(g.enemies[i].obj, LV_OBJ_FLAG_HIDDEN);
        g.enemies[i].active = false;
    }

    /* ── HUD（顶部一行）── */
    s_hud = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_hud, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_hud, lv_color_white(), 0);
    lv_obj_align(s_hud, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);

    /* ── 居中大文字（难度/倒计时/结算共用）── 用 32px 中文大字库 */
    s_center = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_center, &font_cn_32, 0);
    lv_obj_set_style_text_color(s_center, lv_color_hex(0xFFD60A), 0);
    lv_obj_set_style_text_align(s_center, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_center, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);

    /* ── 底部操作提示 ── */
    s_hint = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_hint, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_hint, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_hint, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);

    /* ── 碰撞红屏闪烁遮罩（盖住全屏，平时隐藏）── */
    s_flash = lv_obj_create(s_panel);
    lv_obj_set_size(s_flash, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_set_pos(s_flash, 0, 0);
    lv_obj_set_style_bg_color(s_flash, lv_color_hex(0xFF0000), 0);
    lv_obj_set_style_bg_opa(s_flash, LV_OPA_70, 0);
    lv_obj_set_style_border_width(s_flash, 0, 0);
    lv_obj_set_style_radius(s_flash, 0, 0);
    lv_obj_clear_flag(s_flash, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_flash, LV_OBJ_FLAG_HIDDEN);
}

/* 停掉对局相关定时器（spawn/clock/cd），保留 engine（让闪烁收尾）*/
static void game_timers_stop(void)
{
    if (s_spawn_tmr)
        lv_timer_pause(s_spawn_tmr);
    if (s_clock_tmr)
        lv_timer_pause(s_clock_tmr);
    if (s_cd_tmr)
    {
        lv_timer_del(s_cd_tmr);
        s_cd_tmr = NULL;
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 对外接口
 * ═══════════════════════════════════════════════════════════════ */
void race_start(void)
{
    if (!lvgl_port_lock(200))
    {
        ESP_LOGW(TAG, "race_start 取锁失败");
        return;
    }

    if (s_panel == NULL)
        build_panel();
    else
        lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);

    /* engine 全程运行（动画/虚线滚动/闪烁），对局节拍由 spawn/clock 控制 */
    if (s_engine_tmr == NULL)
        s_engine_tmr = lv_timer_create(engine_cb, ENGINE_MS, NULL);
    else
        lv_timer_resume(s_engine_tmr);

    g.diff = DIFF_EASY; /* 默认停在简单档 */
    enter_select();     /* 入口落到难度选择，而非直接开打 */

    lvgl_port_unlock();
    ESP_LOGI(TAG, "赛车进入：难度选择界面");
}

void race_touch(touch_event_t event)
{
    if (!lvgl_port_lock(100))
        return;

    static uint32_t last_touch = 0; /* 换道防误触时间戳 */

    switch (g.screen)
    {
    /* ── ① 难度选择：左耳上移 / 右耳下移 / 头部确认 ── */
    case RS_SELECT:
        if (event == TOUCH_EVENT_SHORT_PREV_PAGE)
        {
            g.diff = (g.diff == DIFF_EASY) ? DIFF_HARD : (race_diff_t)(g.diff - 1);
            select_render();
        }
        else if (event == TOUCH_EVENT_SHORT_NEXT_PAGE)
        {
            g.diff = (g.diff == DIFF_HARD) ? DIFF_EASY : (race_diff_t)(g.diff + 1);
            select_render();
        }
        else if (event == TOUCH_EVENT_SHORT_HEAD)
        {
            enter_countdown();
        }
        break;

    /* ── ② 倒计时：忽略一切输入 ── */
    case RS_COUNTDOWN:
        break;

    /* ── ③ 游戏进行：左耳切左道 / 右耳切右道（带防误触）── */
    case RS_PLAYING:
        if (g.over)
            break;
        if (event == TOUCH_EVENT_SHORT_PREV_PAGE || event == TOUCH_EVENT_SHORT_NEXT_PAGE)
        {
            uint32_t now = lv_tick_get();
            if (now - last_touch < RACE_TOUCH_COOLDOWN_MS)
                break; /* 防误触：冷却期内忽略 */
            int8_t want = (event == TOUCH_EVENT_SHORT_PREV_PAGE) ? LANE_LEFT : LANE_RIGHT;
            if (want != g.player_lane)
            {
                g.player_lane = want;
                player_refresh();
                last_touch = now;
            }
        }
        break;

    /* ── ④ 结算：头部重玩（回难度选择），其余忽略 ── */
    case RS_RESULT:
        if (event == TOUCH_EVENT_SHORT_HEAD)
            enter_select();
        break;

    default:
        break;
    }

    lvgl_port_unlock();
}

void race_stop(void)
{
    if (!lvgl_port_lock(200))
        return;
    if (s_panel == NULL)
    {
        lvgl_port_unlock();
        return;
    }

    if (s_engine_tmr)
    {
        lv_timer_del(s_engine_tmr);
        s_engine_tmr = NULL;
    }
    if (s_spawn_tmr)
    {
        lv_timer_del(s_spawn_tmr);
        s_spawn_tmr = NULL;
    }
    if (s_clock_tmr)
    {
        lv_timer_del(s_clock_tmr);
        s_clock_tmr = NULL;
    }
    if (s_cd_tmr)
    {
        lv_timer_del(s_cd_tmr);
        s_cd_tmr = NULL;
    }

    lv_obj_del(s_panel); /* 子对象（背景/玩家/敌车/虚线/文字/闪烁）随父一并删除 */
    s_panel = NULL;
    s_bg = NULL;
    s_player = NULL;
    s_hud = NULL;
    s_center = NULL;
    s_hint = NULL;
    s_divider = NULL;
    s_flash = NULL;
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        g.enemies[i].obj = NULL;
        g.enemies[i].active = false;
    }

    lvgl_port_unlock();
    ESP_LOGI(TAG, "赛车已退出，资源已释放");
}
