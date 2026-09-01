/**
 * @file game_whack.c
 * @brief 打地鼠（Whac-A-Mole）游戏实现 v3
 *
 * 全部使用图片素材（p2~p6），不使用 LVGL 绘图。
 *
 * 素材映射：
 *   p2 = 地鼠正常（118×58, RGB565A8）
 *   p3 = 地鼠被打（103×41, RGB565A8）
 *   p4 = 地洞（118×58, RGB565A8）
 *   ds3 = 锤子（80×135, RGB565A8）
 *   p6 = 星星（118×58, RGB565A8）
 *
 * v3 子状态机（全部在本模块内部，ui_port 只认一个 UI_VIEW_GAME）：
 *   ① WS_SELECT    难度选择（左耳上移/右耳下移/头部确认）
 *   ② WS_COUNTDOWN 3 秒倒计时（大数字 3→2→1→GO，期间忽略输入）
 *   ③ WS_PLAYING   游戏进行 30 秒（左耳打左洞/右耳打右洞）
 *   ④ WS_RESULT    结算（头部=重玩回难度选择，腹/背退出由 ui_port 拦截）
 *
 * 输入（由 ui_port.c UI_VIEW_GAME 分支转发，腹/背与长按耳已被 ui_port 拦截）：
 *   - TOUCH_EVENT_SHORT_PREV_PAGE（左铜箔）
 *   - TOUCH_EVENT_SHORT_NEXT_PAGE（右铜箔）
 *   - TOUCH_EVENT_SHORT_HEAD（头部）
 *   含义随当前子状态而变，见 whack_touch()。
 *
 * 难度与玩法参数全部宏化，见 whack_sprites.h。
 * 高分按难度分别持久化到 NVS（刷新才写，低分不动）。
 */
#include "game_whack.h"
#include "whack_sprites.h"
#include "ui/ui_port.h"
#include "ui/font_loader.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_random.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "bsp/bsp_config.h"
#include "bsp/bsp_board.h"

LV_FONT_DECLARE(font_cn_16);

static const char *TAG = "WHACK";

/* ── 布局 ── */
#define HOLE_COUNT 2
#define ENGINE_MS 50 /* 引擎帧周期（动画/物理）*/
/* ds.png 像素分析：
 * 洞口椭圆顶部 y≈181，底部 y≈218，高约37px
 * 左洞中心 x=85，右洞中心 x=240（非等分，由像素实测）*/
#define HOLE_TOP_Y 181                         /* 洞口椭圆顶部 y */
#define HOLE_BOTTOM_Y 218                      /* ★ 洞口椭圆底部 y = 地面线 = 裁剪容器底边（按背景图微调）*/
static const int16_t s_hole_cx[2] = {85, 240}; /* 两洞口中心 x，像素实测 */
/* 地鼠在各洞容器内的 x 微调（负=往左，正=往右），用于对齐背景图洞口中心。
 * ★左洞地鼠想再偏左，把 [0] 调更小（更负）；右洞改 [1] */
static const int16_t s_mole_dx[2] = {-8, 0};

/* ── 难度档（数组下标，与 NVS key 一一对应）── */
typedef enum
{
    DIFF_EASY = 0,
    DIFF_NORMAL,
    DIFF_HARD,
    DIFF_COUNT,
} whack_diff_t;

/* 难度参数表：{停留基准ms, 间隔基准ms, 出洞px/帧, 中文名, NVS key} */
typedef struct
{
    int life_ms;
    int spawn_ms;
    int rise_px;
    const char *name;
    const char *nvs_key;
} diff_param_t;

static const diff_param_t s_diff[DIFF_COUNT] = {
    [DIFF_EASY] = {WHACK_LIFE_MS_EASY, WHACK_SPAWN_MS_EASY, WHACK_RISE_PX_EASY, "简单", "hi_easy"},
    [DIFF_NORMAL] = {WHACK_LIFE_MS_NORMAL, WHACK_SPAWN_MS_NORMAL, WHACK_RISE_PX_NORMAL, "一般", "hi_normal"},
    [DIFF_HARD] = {WHACK_LIFE_MS_HARD, WHACK_SPAWN_MS_HARD, WHACK_RISE_PX_HARD, "困难", "hi_hard"},
};

#define WHACK_NVS_NS "whack" /* NVS 命名空间 */

/* ── 子状态机 ── */
typedef enum
{
    WS_SELECT = 0, /* 难度选择 */
    WS_COUNTDOWN,  /* 3 秒倒计时 */
    WS_PLAYING,    /* 游戏进行 */
    WS_RESULT,     /* 结算 */
} whack_screen_t;

/* ── 单洞状态机 ── */
typedef enum
{
    HOLE_EMPTY = 0,
    HOLE_RISING,
    HOLE_UP,
    HOLE_HIT,
    HOLE_FALLING,
} hole_state_t;

#define STAR_COUNT 0 //! 暂时关闭星星特效
#define STAR_LIFE_MS 500

typedef struct
{
    lv_obj_t *obj;
    int16_t x, y;
    int16_t vx, vy;
    uint32_t born;
    bool active;
} star_effect_t;

/* ── 全局状态 ── */
static struct
{
    whack_screen_t screen; /* 当前子界面 */
    whack_diff_t diff;     /* 当前选中难度 */

    /* 本局动态参数（= 难度基准，运行中随 10 秒加速衰减）*/
    int cur_life_ms;
    int cur_spawn_ms;
    int cur_rise_px;
    int high_score; /* 当前难度历史最高分（进游戏时读 NVS）*/

    /* 倒计时 */
    int cd_value; /* 3 → 2 → 1 → 0(GO)（旧数字倒计时，已弃用）*/

    /* 时间条 */
    uint32_t intro_t0; /* 开场动画起始 tick（时间条 3 秒耗空用）*/
    uint32_t play_t0;  /* 正式开局起始 tick（时间条平滑倒计时用）*/

    /* 对局数据 */
    bool over;
    int score;     /* 得分（可被空敲扣减，最低 0）*/
    int hits;      /* 命中地鼠数 */
    int escaped;   /* 未中掉（逃跑）数 */
    int misses;    /* 空敲次数 */
    int taps;      /* 总有效敲击数（命中+空敲），用于准确率 */
    int elapsed_s;   /* 已进行秒数，用于 10 秒加速 */
    int time_left;   /* 剩余秒（旧，仅 HUD 兼容用，结算判定已改用 time_left_ms）*/
    int time_left_ms; /* 剩余毫秒：时间条/结算唯一真相，砸中 +1000，封顶 30000 */

    /* 单洞 */
    hole_state_t st[HOLE_COUNT];
    uint32_t t0[HOLE_COUNT];
    uint32_t hammer_t0[HOLE_COUNT]; /* 锤子出现时刻（用于到时自动收回）*/
    /* 命中震动起始时刻（0=未在震动）。非阻塞震动：try_hit 里只开，引擎按时关，
     * 避免 bsp_motor_pulse() 的 30ms vTaskDelay 卡住 LVGL 线程拖垮锤子动画。*/
    uint32_t motor_t0;
    int16_t mole_y[HOLE_COUNT];
    int16_t mole_hide_y;
    int16_t mole_show_y;
    star_effect_t stars[HOLE_COUNT][STAR_COUNT];
} g;

/* ── LVGL 对象 ── */
static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_hud = NULL;          /* 游戏中顶部一行 HUD */
static lv_obj_t *s_center = NULL;       /* 居中大文字（难度选择/倒计时/结算共用）*/
static lv_obj_t *s_hint = NULL;         /* 底部操作提示 */
static lv_obj_t *s_dim = NULL;          /* 结算半透明遮罩（暗化背景）*/
static lv_obj_t *s_timebar_bg = NULL;   /* 时间条轨道（底）*/
static lv_obj_t *s_timebar_fill = NULL; /* 时间条填充（随剩余时间收缩）*/
static lv_obj_t *s_hole[HOLE_COUNT];
static lv_obj_t *s_mole[HOLE_COUNT];
static lv_obj_t *s_hammer[HOLE_COUNT];

static lv_timer_t *s_spawn_tmr = NULL;  /* 出洞节拍 */
static lv_timer_t *s_engine_tmr = NULL; /* 动画/物理引擎 */
static lv_timer_t *s_clock_tmr = NULL;  /* 1Hz 倒计时 + 10 秒加速 */
static lv_timer_t *s_cd_tmr = NULL;     /* 开局 3 秒倒计时 */

/* ── 结算后自动重开 ──
 * 需求：时间到的结算界面只留背景图，停 WHACK_RESTART_DELAY_MS 后自动重来一局，
 * 等价于以前在结算界面「触摸头部继续」。头部触摸仍保留（想立刻重来也行）。*/
#define WHACK_RESTART_DELAY_MS 1000      /* 结算停留时长（ms）*/
static lv_timer_t *s_restart_tmr = NULL; /* one-shot 自动重开定时器 */

/* 前置声明 */
static void enter_select(void);
static void enter_countdown(void);
static void enter_playing(void);
static void enter_result(void);
static void game_timers_stop(void);

/* 取消尚未触发的自动重开定时器。
 * 手动重开 / 退出游戏时必须调用，否则面板已删而定时器仍在，回调里会碰野指针。*/
static void restart_timer_cancel(void)
{
    if (s_restart_tmr)
    {
        lv_timer_del(s_restart_tmr);
        s_restart_tmr = NULL;
    }
}

/* one-shot 回调：结算停留结束，自动开下一局（等价于在结算界面按一次头部）*/
static void restart_cb(lv_timer_t *t)
{
    (void)t;
    s_restart_tmr = NULL; /* repeat_count=1：LVGL 在本回调返回后自动删除该定时器 */
    enter_countdown();
}

/* ═══════════════════════════════════════════════════════════════
 * NVS 高分读写（按难度独立 key）
 * ═══════════════════════════════════════════════════════════════ */
/* NVS 高分持久化已恢复（值 1）。
 * 前置依赖：触摸任务栈已迁回内部 SRAM（见 application.c 创建 touch_scan_task 处），
 *           否则在该任务上下文读写 NVS 会触发 flash cache disable 断言崩溃（BUG-010 家族）。
 * 若日后把 NVS 读写挪到其它栈在 SPIRAM 的任务，必须重新评估此风险。 */
#define WHACK_NVS_ENABLED 1

static int highscore_load(whack_diff_t d)
{
    (void)d;
#if WHACK_NVS_ENABLED
    nvs_handle_t h;
    int32_t v = 0;
    if (nvs_open(WHACK_NVS_NS, NVS_READONLY, &h) == ESP_OK)
    {
        nvs_get_i32(h, s_diff[d].nvs_key, &v);
        nvs_close(h);
    }
    return (int)v;
#else
    return 0;
#endif
}

/* 仅在刷新（score > 旧值）时写入，返回是否刷新 */
static bool highscore_save_if_better(whack_diff_t d, int score)
{
    (void)d;
    if (score <= g.high_score)
        return false;
#if WHACK_NVS_ENABLED
    nvs_handle_t h;
    if (nvs_open(WHACK_NVS_NS, NVS_READWRITE, &h) == ESP_OK)
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
 * 视觉渲染：地鼠 / 锤子 / 星星
 * ═══════════════════════════════════════════════════════════════ */
static void mole_set_visual(int i, hole_state_t state)
{
    if (i < 0 || i >= HOLE_COUNT || s_mole[i] == NULL)
        return;

    if (state == HOLE_EMPTY)
    {
        lv_obj_add_flag(s_mole[i], LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_obj_set_y(s_mole[i], g.mole_y[i]);
    lv_obj_clear_flag(s_mole[i], LV_OBJ_FLAG_HIDDEN);

    if (state == HOLE_HIT)
        lv_image_set_src(s_mole[i], &ds1); /* 被打扁（ds1，与正常态等比同尺寸）*/
    else
        lv_image_set_src(s_mole[i], &p2); /* 正常 */

    /* 命中后稳定显示打扁态 ds1，停顿 WHACK_HIT_SHOW_MS(0.5s) 后由引擎切回 EMPTY 下洞。
     * （原命中闪烁逻辑已移除：停留拉长到 0.5s 后闪烁会过于杂乱）*/
}

static void hammer_set_visual(int i, bool show)
{
    if (i < 0 || i >= HOLE_COUNT || s_hammer[i] == NULL)
        return;

    if (!show)
    {
        lv_obj_add_flag(s_hammer[i], LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(s_hammer[i], LV_OBJ_FLAG_HIDDEN);

    /* 锤头（ds3 图上半部）正好砸到地鼠头顶：
     *   地鼠头顶绝对 y = 容器顶(HOLE_BOTTOM_Y - WHACK_MOLE_H) + 地鼠露出微调(mole_show_y)
     *   锤头中心绝对 y = 锤子左上 y + WHACK_HAMMER_HEAD_OFFSET
     *   令 锤头中心 = 地鼠头顶 + HIT_DY → 锤子左上 y = 地鼠头顶 + HIT_DY - HEAD_OFFSET
     *   （之前用图底边对齐导致"锤柄"砸头，故整体下移 = 改用锤头偏移对齐）
     *   x 以地鼠中心(洞中心 + 每洞微调 s_mole_dx)为基准，再加水平偏移 HIT_DX */
    int16_t mole_top_y = HOLE_BOTTOM_Y - WHACK_MOLE_H + g.mole_show_y;
    int16_t mole_cx = s_hole_cx[i] + s_mole_dx[i];
    lv_obj_set_pos(s_hammer[i],
                   mole_cx - WHACK_HAMMER_W / 2 + WHACK_HAMMER_HIT_DX,
                   mole_top_y + WHACK_HAMMER_HIT_DY - WHACK_HAMMER_HEAD_OFFSET);
}

static void stars_set_visual(int i, bool activate)
{
    if (i < 0 || i >= HOLE_COUNT)
        return;

    for (int s = 0; s < STAR_COUNT; s++)
    {
        star_effect_t *star = &g.stars[i][s];
        if (star->obj == NULL)
            continue;

        if (!activate)
        {
            lv_obj_add_flag(star->obj, LV_OBJ_FLAG_HIDDEN);
            star->active = false;
            continue;
        }

        star->x = s_hole_cx[i];
        star->y = HOLE_TOP_Y - 20;
        star->vx = (int16_t)((esp_random() % 7) - 3);
        star->vy = (int16_t)(-(esp_random() % 4 + 2));
        star->born = lv_tick_get();
        star->active = true;

        lv_obj_set_pos(star->obj, star->x, star->y);
        lv_obj_clear_flag(star->obj, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 一次性把所有地洞/地鼠/锤子/星星藏起来（切界面时清场）*/
static void board_hide_all(void)
{
    for (int i = 0; i < HOLE_COUNT; i++)
    {
        g.st[i] = HOLE_EMPTY;
        if (s_hole[i])
            lv_obj_add_flag(s_hole[i], LV_OBJ_FLAG_HIDDEN);
        mole_set_visual(i, HOLE_EMPTY);
        hammer_set_visual(i, false);
        g.hammer_t0[i] = 0; /* 复位锤子计时，避免切界面残留 */
        stars_set_visual(i, false);
    }
}

static void board_show_holes(void)
{
    for (int i = 0; i < HOLE_COUNT; i++)
        if (s_hole[i])
            lv_obj_clear_flag(s_hole[i], LV_OBJ_FLAG_HIDDEN);
}
/**
 * 刷新 HUD（分数/高分）。每帧调用（引擎回调）以保持分数更新及时。
 * HUD 设计在时间条下方，避免与时间条重叠（尤其是时间条快没了时）。
 */
static void hud_refresh(void)
{
    /* 打地鼠所有文字已按需求注释掉，HUD 不再显示分数/高分文字
    if (s_hud == NULL)
        return;
    char buf[64];
    snprintf(buf, sizeof(buf), "分数%d  高分%d", g.score, g.high_score);
    lv_label_set_text(s_hud, buf);
    */
}

/* 设置时间条填充比例 ratio∈[0,1]：1=满，0=空。
 * 同时按剩余比例切换颜色（正常绿 / 警告橙 / 危险红）。
 * show=false 时整条隐藏（如难度/结算界面）。 */
static void timebar_set(float ratio, bool show)
{
    if (s_timebar_bg == NULL || s_timebar_fill == NULL)
        return;

    if (!show)
    {
        lv_obj_add_flag(s_timebar_bg, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_timebar_fill, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    lv_obj_clear_flag(s_timebar_bg, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_timebar_fill, LV_OBJ_FLAG_HIDDEN);

    if (ratio < 0.0f)
        ratio = 0.0f;
    if (ratio > 1.0f)
        ratio = 1.0f;

    /* 轨道总宽 = 屏宽 - 两侧边距；填充宽 = 轨道宽 × ratio */
    int track_w = BSP_LCD_WIDTH - WHACK_TIMEBAR_MARGIN * 2;
    int fill_w = (int)(track_w * ratio + 0.5f);
    lv_obj_set_width(s_timebar_fill, fill_w);

    /* 按剩余比例切换填充颜色 */
    uint32_t color;
    if (ratio <= 1.0f / 6.0f)
        color = WHACK_TIMEBAR_DANGER_COLOR;
    else if (ratio <= 1.0f / 3.0f)
        color = WHACK_TIMEBAR_WARN_COLOR;
    else
        color = WHACK_TIMEBAR_OK_COLOR;
    lv_obj_set_style_bg_color(s_timebar_fill, lv_color_hex(color), 0);
}

/* ═══════════════════════════════════════════════════════════════
 * 引擎：动画 + 物理（所有子界面共用，只在 PLAYING 推进对局逻辑）
 * ═══════════════════════════════════════════════════════════════ */
static void engine_cb(lv_timer_t *t)
{
    (void)t;
    uint32_t now = lv_tick_get();
    bool dirty = false;

    /* ── 时间条驱动（平滑，按 tick 而非整秒）── */
    if (g.screen == WS_COUNTDOWN)
    {
        /* 开场动画：时间条 WHACK_INTRO_MS 内从满(1)线性耗到空(0)，
         * 耗空瞬间回满并真正开打（见 enter_playing）。*/
        uint32_t e = now - g.intro_t0;
        if (e >= WHACK_INTRO_MS)
        {
            timebar_set(1.0f, true); /* 回满 */
            enter_playing();         /* 回满即开打 */
        }
        else
        {
            float ratio = 1.0f - (float)e / (float)WHACK_INTRO_MS;
            timebar_set(ratio, true);
        }
    }
    else if (g.screen == WS_PLAYING && !g.over)
    {
        /* 正式倒计时：剩余毫秒 time_left_ms 为唯一真相，每帧递减 ENGINE_MS。
         * 砸中地鼠会给 time_left_ms 加时间（见 try_hit），故时间条会回涨。*/
        if (g.time_left_ms > ENGINE_MS)
            g.time_left_ms -= ENGINE_MS;
        else
            g.time_left_ms = 0;

        int total_ms = WHACK_GAME_SECONDS * 1000;
        float ratio = (float)g.time_left_ms / (float)total_ms;
        timebar_set(ratio, true);

        if (g.time_left_ms == 0)
            enter_result(); /* 时间耗尽，立即结算 */
    }

    for (int i = 0; i < HOLE_COUNT; i++)
    {
        if (g.screen == WS_PLAYING && !g.over)
        {
            switch (g.st[i])
            {
            case HOLE_RISING:
                g.mole_y[i] -= g.cur_rise_px;
                if (g.mole_y[i] <= g.mole_show_y)
                {
                    g.mole_y[i] = g.mole_show_y;
                    g.st[i] = HOLE_UP;
                    g.t0[i] = now;
                }
                mole_set_visual(i, g.st[i]);
                break;

            case HOLE_UP:
                if ((now - g.t0[i]) >= (uint32_t)g.cur_life_ms)
                {
                    g.st[i] = HOLE_FALLING;
                    g.t0[i] = now;
                    g.escaped++;
                    dirty = true;
                }
                break;

            case HOLE_HIT:
                if ((now - g.t0[i]) >= WHACK_HIT_SHOW_MS)
                {
                    /* 被打中后直接消失（不缩回）。锤子收回由下方独立逻辑处理 */
                    g.st[i] = HOLE_EMPTY;
                    g.mole_y[i] = g.mole_hide_y;
                    mole_set_visual(i, HOLE_EMPTY);
                }
                else
                {
                    mole_set_visual(i, g.st[i]);
                }
                break;

            case HOLE_FALLING:
                g.mole_y[i] += WHACK_FALL_PX;
                if (g.mole_y[i] >= g.mole_hide_y)
                {
                    g.mole_y[i] = g.mole_hide_y;
                    g.st[i] = HOLE_EMPTY;
                    mole_set_visual(i, HOLE_EMPTY);
                }
                else
                {
                    mole_set_visual(i, HOLE_FALLING);
                }
                break;

            default:
                break;
            }
        }

        /* 锤子收回：显示后过 WHACK_HAMMER_SHOW_MS 自动隐藏
         * （命中/空敲统一处理，hammer_t0!=0 表示锤子正显示）*/
        if (g.hammer_t0[i] != 0 && (now - g.hammer_t0[i]) >= WHACK_HAMMER_SHOW_MS)
        {
            hammer_set_visual(i, false);
            g.hammer_t0[i] = 0;
        }

        /* 星星特效（任何子界面都让残留星星飞完）*/
        for (int s = 0; s < STAR_COUNT; s++)
        {
            star_effect_t *star = &g.stars[i][s];
            if (!star->active)
                continue;
            star->x += star->vx;
            star->y += star->vy;
            star->vy += 1;
            lv_obj_set_pos(star->obj, star->x, star->y);
            if ((now - star->born) >= STAR_LIFE_MS)
            {
                lv_obj_add_flag(star->obj, LV_OBJ_FLAG_HIDDEN);
                star->active = false;
            }
        }
    }

    /* 命中震动关闭（非阻塞）：到时长就停，替代 bsp_motor_pulse() 里的 vTaskDelay。
     * 放在洞循环外，因为震动是全局一个马达，与具体哪个洞无关。*/
    if (g.motor_t0 != 0 && (now - g.motor_t0) >= WHACK_MOTOR_MS)
    {
        bsp_motor_set(0);
        g.motor_t0 = 0;
    }

    if (dirty)
        hud_refresh();
}

/* ── 出洞节拍：单只地鼠，找到空洞就冒一只 ── */
static void spawn_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != WS_PLAYING || g.over)
        return;

    /* 同时只 1 只：已有非空洞则不再生成 */
    for (int i = 0; i < HOLE_COUNT; i++)
        if (g.st[i] != HOLE_EMPTY)
            return;

    int hole = (int)(esp_random() % HOLE_COUNT);
    g.st[hole] = HOLE_RISING;
    g.t0[hole] = lv_tick_get();
    g.mole_y[hole] = g.mole_hide_y;
    mole_set_visual(hole, HOLE_RISING);
}

/* ── 1Hz 时钟：倒计时 + 每 10 秒加速 ── */
static void clock_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != WS_PLAYING || g.over)
        return;

    /* 剩余秒由 time_left_ms 换算（仅供日志/兼容，结算判定在引擎按 time_left_ms 走）*/
    g.time_left = (g.time_left_ms + 999) / 1000;
    g.elapsed_s++;

    /* 每 10 秒提速：停留时间、出现间隔各乘 80%，带下限 */
    if (g.elapsed_s % WHACK_ACCEL_EVERY_S == 0)
    {
        g.cur_life_ms = g.cur_life_ms * WHACK_ACCEL_PERCENT / 100;
        g.cur_spawn_ms = g.cur_spawn_ms * WHACK_ACCEL_PERCENT / 100;
        if (g.cur_life_ms < WHACK_LIFE_MS_MIN)
            g.cur_life_ms = WHACK_LIFE_MS_MIN;
        if (g.cur_spawn_ms < WHACK_SPAWN_MS_MIN)
            g.cur_spawn_ms = WHACK_SPAWN_MS_MIN;
        if (s_spawn_tmr)
            lv_timer_set_period(s_spawn_tmr, g.cur_spawn_ms);
        ESP_LOGI(TAG, "提速@%ds: life=%d spawn=%d", g.elapsed_s, g.cur_life_ms, g.cur_spawn_ms);
    }

    hud_refresh();
    /* 倒计时结束判定已移到 engine_cb（按 time_left_ms），此处不再触发结算 */
}

/* ═══════════════════════════════════════════════════════════════
 * 打击逻辑（仅 PLAYING 子界面调用）
 * ═══════════════════════════════════════════════════════════════ */
static void try_hit(int hole)
{
    if (hole < 0 || hole >= HOLE_COUNT)
        return;
    if (g.over)
        return;

    g.taps++; /* 计入总敲击（用于准确率）*/

    /* 只要敲击就立即显示锤子并做击打动作（不管洞里有没有地鼠）。
     * 锤子会在引擎里 WHACK_HIT_SHOW_MS 后自动收回（命中时），
     * 空敲时由本函数末尾的延时收回逻辑处理。 */
    hammer_set_visual(hole, true);
    g.hammer_t0[hole] = lv_tick_get(); /* 记录锤子出现时刻，供引擎收回 */

    /* 仅 UP（完全出洞到达最高点）可击中；RISING/HIT/FALLING 均视为空敲 */
    if (g.st[hole] == HOLE_UP)
    {
        g.score++;
        g.hits++;
        g.st[hole] = HOLE_HIT; /* 进 HIT 态即锁定，同一地鼠只计一次 */
        g.t0[hole] = lv_tick_get();

        /* 砸中奖励时间：+1 秒，封顶满格（30s），时间条随之回涨 */
        g.time_left_ms += WHACK_HIT_BONUS_MS;
        if (g.time_left_ms > WHACK_GAME_SECONDS * 1000)
            g.time_left_ms = WHACK_GAME_SECONDS * 1000;

        mole_set_visual(hole, HOLE_HIT);
        stars_set_visual(hole, true); /* 眩晕星星 */
        hud_refresh();
        ESP_LOGI(TAG, "命中洞%d! 得分=%d", hole, g.score);

        /* ★ 命中震动必须放在最后，且不能阻塞：
         * bsp_motor_pulse() 内部是 set(100) + vTaskDelay(30) + set(0)，会把
         * 本函数所在的 LVGL 线程卡住 30ms。原先它排在 mole/hammer 渲染之前，
         * 导致锤子刚显示就被这 30ms 吃掉寿命（WHACK_HAMMER_SHOW_MS=30 时锤子
         * 还没渲染出来就到期被收回）→ 表现为「空敲看得见锤子、命中却看不见」。
         * 改用 bsp_motor_set() 只开震动不等待，关闭交给引擎按时间片处理。*/
        bsp_motor_set(BSP_MOTOR_DEFAULT_STRENGTH);
        g.motor_t0 = lv_tick_get();
    }
    else
    {
        /* 空敲惩罚：扣分（最低 0）+ 记失误（锤子仍已显示）*/
        g.misses++;
        g.score -= WHACK_MISS_PENALTY;
        if (g.score < 0)
            g.score = 0;
        hud_refresh();
        ESP_LOGI(TAG, "空敲洞%d! 扣分 得分=%d 失误=%d", hole, g.score, g.misses);
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：① 难度选择
 * ═══════════════════════════════════════════════════════════════ */
static void select_render(void)
{
    if (s_center == NULL)
        return;
    char buf[96];
    /* 竖排三选项：当前项前伸（无缩进），其余项缩进两格，靠位置区分选中态（不用箭头）。
     * 当前难度的历史高分附在选中行后面。 */
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
    g.screen = WS_SELECT;
    game_timers_stop(); /* 选择期间不跑对局定时器 */
    board_hide_all();
    timebar_set(0.0f, false); /* 隐藏时间条 */
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    if (s_hint)
        lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN); /* 清掉结算页残留提示 */
    /* 隐藏结算遮罩 */
    if (s_dim)
        lv_obj_add_flag(s_dim, LV_OBJ_FLAG_HIDDEN);
    /* 还原居中文字属性（结算时扩了宽度/换行模式）*/
    if (s_center)
    {
        lv_obj_set_style_text_font(s_center, font_cn_32_get(), 0);
        lv_obj_set_width(s_center, LV_SIZE_CONTENT);
        lv_label_set_long_mode(s_center, LV_LABEL_LONG_CLIP);
        lv_obj_align(s_center, LV_ALIGN_CENTER, 0, -10);
    }
    select_render();
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：② 开场动画（时间条 3 秒耗空 → 回满即开打）
 *   原 3-2-1-GO 数字倒计时已移除，改用顶部时间条动画作为开场提示。
 *   实际进度由 engine_cb 的 WS_COUNTDOWN 分支驱动（按 intro_t0 计算）。
 * ═══════════════════════════════════════════════════════════════ */
static void enter_countdown(void)
{
    restart_timer_cancel(); /* 手动/自动重开都从这里进，先撤掉挂起的自动重开 */
    g.screen = WS_COUNTDOWN;
    if (s_hint)
        lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
    if (s_center)
        lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN); /* 不再显示数字 */
    board_show_holes();                                /* 提前露出地洞，营造氛围 */
    g.intro_t0 = lv_tick_get();                        /* 开场动画计时起点 */
    timebar_set(1.0f, true);                           /* 时间条从满开始耗 */
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：③ 游戏进行
 * ═══════════════════════════════════════════════════════════════ */
static void enter_playing(void)
{
    g.screen = WS_PLAYING;
    g.over = false;

    /* 复位对局数据 */
    g.score = 0;
    g.hits = 0;
    g.escaped = 0;
    g.misses = 0;
    g.taps = 0;
    g.elapsed_s = 0;
    g.time_left = WHACK_GAME_SECONDS;
    g.time_left_ms = WHACK_GAME_SECONDS * 1000; /* 时间条/结算唯一真相 */

    /* 本局动态参数 = 当前难度基准（本版本不分难度，g.diff 固定一般档）*/
    g.cur_life_ms = s_diff[g.diff].life_ms;
    g.cur_spawn_ms = s_diff[g.diff].spawn_ms;
    g.cur_rise_px = s_diff[g.diff].rise_px;
    g.high_score = highscore_load(g.diff);

    board_hide_all();
    board_show_holes();
    if (s_center)
        lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN); /* HUD 文字已注释，始终隐藏 */
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

    ESP_LOGI(TAG, "开打: 难度=%s life=%d spawn=%d rise=%d",
             s_diff[g.diff].name, g.cur_life_ms, g.cur_spawn_ms, g.cur_rise_px);
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：④ 结算
 * ═══════════════════════════════════════════════════════════════ */
static void enter_result(void)
{
    g.over = true;
    g.screen = WS_RESULT;

    /* 暂停对局定时器，立即清场禁点 */
    if (s_spawn_tmr)
        lv_timer_pause(s_spawn_tmr);
    if (s_clock_tmr)
        lv_timer_pause(s_clock_tmr);
    board_hide_all();
    timebar_set(0.0f, false); /* 结算隐藏时间条 */

    /* 高分刷新判定（刷新才写 NVS）*/
    bool refreshed = highscore_save_if_better(g.diff, g.score);

    /* 准确率 = 命中 / 总敲击 */
    int acc = (g.taps > 0) ? (g.hits * 100 / g.taps) : 0;

    /* 结算变暗遮罩已按需求注释掉（不再压暗背景）
    if (s_dim)
    {
        lv_obj_clear_flag(s_dim, LV_OBJ_FLAG_HIDDEN);
        lv_obj_move_to_index(s_dim, -2);
    }
    */
    if (s_dim)
        lv_obj_add_flag(s_dim, LV_OBJ_FLAG_HIDDEN); /* 保证遮罩隐藏 */

    /* 结算文字已按需求注释掉（不显示任何文字）
    if (s_center)
    {
        lv_obj_set_style_text_font(s_center, font_cn_32_get(), 0);
        lv_obj_set_width(s_center, BSP_LCD_WIDTH - 20);
        lv_label_set_long_mode(s_center, LV_LABEL_LONG_WRAP);
        char buf[160];
        snprintf(buf, sizeof(buf),
                 "时间到!\n得分 %d%s\n击中 %d  未中 %d\n空敲 %d  准确 %d%%",
                 g.score, refreshed ? " (新纪录!)" : "",
                 g.hits, g.escaped, g.misses, acc);
        lv_label_set_text(s_center, buf);
        lv_obj_move_foreground(s_center);
        lv_obj_align(s_center, LV_ALIGN_CENTER, 0, 0);
        lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    }
    */
    if (s_center)
        lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN); /* 结算不显示文字 */
    (void)refreshed;
    (void)acc;
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    if (s_hint)
        lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN); /* 结算页不显示操作提示 */

    /* 停留 1 秒后自动重开一局（one-shot，触发一次后 LVGL 自动删除）*/
    restart_timer_cancel();
    s_restart_tmr = lv_timer_create(restart_cb, WHACK_RESTART_DELAY_MS, NULL);
    if (s_restart_tmr)
        lv_timer_set_repeat_count(s_restart_tmr, 1);

    ESP_LOGI(TAG, "结算: 难度=%s 得分=%d 击中=%d 未中=%d 空敲=%d 准确=%d%% %s",
             s_diff[g.diff].name, g.score, g.hits, g.escaped, g.misses, acc,
             refreshed ? "[新纪录]" : "");
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

    /* 地鼠动画 y 值 = 地鼠图【顶部】相对容器顶部的 y（由 mole_set_visual 的 lv_obj_set_y 应用）
     *
     * 实现原理（容器裁剪，等价 HTML 的 overflow:hidden）：
     *   容器 s_hole 关闭 OVERFLOW_VISIBLE，超出容器边界的地鼠部分被裁掉。
     *   容器底部对齐地面线(HOLE_BOTTOM_Y)，容器高=地鼠高(WHACK_MOLE_H)，
     *   故容器顶 = HOLE_BOTTOM_Y - WHACK_MOLE_H。
     *   地鼠从容器底以下往上滑：
     *     - 沉到容器底以下 → 整只被裁，看不见（藏在洞里）
     *     - 升到容器内      → 露出；超出容器顶的部分被裁
     *
     * mole_hide_y：地鼠顶部 = 容器底(y=容器高) → 整只地鼠在容器外下方，被裁
     * mole_show_y：地鼠顶部 = 容器顶(y=0) → 地鼠完整钻出，底部落在地面线
     *   ★想让地鼠下半身留在洞里(只露上半身)，把 show_y 调大(正值，往下沉)即可 */
    g.mole_hide_y = WHACK_MOLE_H; /* 藏：地鼠整体沉到容器底以下，被裁掉不可见 */
    g.mole_show_y = 3;            /* 露：地鼠完整钻出后再下潜 3px（底部低于地面线 3px，上半身少露一点）*/

    /* ── 面板 ── */
    s_panel = lv_obj_create(scr);
    lv_obj_set_size(s_panel, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_align(s_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_panel, 0, 0);
    lv_obj_set_style_pad_all(s_panel, 0, 0);
    lv_obj_set_style_radius(s_panel, 0, 0);
    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(s_panel, LV_SCROLLBAR_MODE_OFF);

    /* ── 背景：按 whack_sprites.h 的 WHACK_BG_MODE 二选一 ── */
#if WHACK_BG_MODE == WHACK_BG_MODE_IMAGE
    /* 素材模式：铺一张全屏背景图（最底层，作为 s_panel 的第一个子对象）*/
    {
        lv_obj_t *bg = lv_image_create(s_panel);
        lv_image_set_src(bg, &WHACK_BG_IMAGE);
        lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
        lv_obj_move_background(bg); /* 压到最底，保证地洞/地鼠/文字都在它之上 */
    }
#else
    /* 纯色模式：LVGL 纯色填充整屏（无需素材）*/
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(WHACK_BG_COLOR), 0);
#endif

    /* ── HUD / 居中大文字 / 底部提示 ──
     * 三者全代码库只有 add_flag(HIDDEN)，无任何 clear_flag 使其显示：
     *   - s_hud：744行注释明写"HUD 文字已注释，始终隐藏"
     *   - s_center：whack_start() 固定 DIFF_NORMAL 直接 enter_countdown()，
     *     enter_select()/select_render() 是死路径不会触发
     *   - s_hint：同样全程隐藏
     * 用户从未看到过这三个标签的文字。注释掉创建，避免 s_center 首建时触发
     * font_cn_32_get() 同步读盘阻塞（实测卡顿130ms+）。三变量保持 NULL，
     * 其余引用处 if(s_hud)/if(s_center)/if(s_hint) 判断会自动跳过，不受影响。
    s_hud = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_hud, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_hud, lv_color_hex(0x00C800), 0);
    lv_obj_align(s_hud, LV_ALIGN_TOP_MID, 0, WHACK_TIMEBAR_MARGIN + WHACK_TIMEBAR_H + 4);
    lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);

    s_center = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_center, font_cn_32_get(), 0);
    lv_obj_set_style_text_color(s_center, lv_color_hex(0x00C800), 0);
    lv_obj_set_style_text_align(s_center, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_center, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);

    s_hint = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_hint, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_hint, lv_color_hex(0x00C800), 0);
    lv_obj_set_style_text_align(s_hint, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_hint, LV_ALIGN_BOTTOM_MID, 0, -8);
    lv_obj_add_flag(s_hint, LV_OBJ_FLAG_HIDDEN);
    */

    /* ── 结算半透明遮罩（默认隐藏，结算时显示暗化背景）── */
    s_dim = lv_obj_create(s_panel);
    lv_obj_set_size(s_dim, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_set_pos(s_dim, 0, 0);
    lv_obj_set_style_bg_color(s_dim, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_dim, LV_OPA_50, 0);
    lv_obj_set_style_border_width(s_dim, 0, 0);
    lv_obj_clear_flag(s_dim, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_dim, LV_OBJ_FLAG_HIDDEN);

    /* ── 时间条（顶部缓慢消失，剩余时间可视化）── */
    int tb_w = BSP_LCD_WIDTH - WHACK_TIMEBAR_MARGIN * 2;
    /* 轨道（底色）*/
    s_timebar_bg = lv_obj_create(s_panel);
    lv_obj_set_size(s_timebar_bg, tb_w, WHACK_TIMEBAR_H);
    lv_obj_set_pos(s_timebar_bg, WHACK_TIMEBAR_MARGIN, WHACK_TIMEBAR_MARGIN);
    lv_obj_set_style_bg_color(s_timebar_bg, lv_color_hex(WHACK_TIMEBAR_BG_COLOR), 0);
    lv_obj_set_style_bg_opa(s_timebar_bg, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_timebar_bg, 0, 0);
    lv_obj_set_style_radius(s_timebar_bg, WHACK_TIMEBAR_RADIUS, 0);
    lv_obj_set_style_pad_all(s_timebar_bg, 0, 0);
    lv_obj_clear_flag(s_timebar_bg, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_timebar_bg, LV_OBJ_FLAG_HIDDEN);
    /* 填充（左对齐，宽度随剩余时间收缩）— 作为轨道的子对象，左端对齐 */
    s_timebar_fill = lv_obj_create(s_timebar_bg);
    lv_obj_set_size(s_timebar_fill, tb_w, WHACK_TIMEBAR_H);
    lv_obj_align(s_timebar_fill, LV_ALIGN_LEFT_MID, 0, 0);
    lv_obj_set_style_bg_color(s_timebar_fill, lv_color_hex(WHACK_TIMEBAR_OK_COLOR), 0);
    lv_obj_set_style_bg_opa(s_timebar_fill, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_timebar_fill, 0, 0);
    lv_obj_set_style_radius(s_timebar_fill, WHACK_TIMEBAR_RADIUS, 0);
    lv_obj_set_style_pad_all(s_timebar_fill, 0, 0);
    lv_obj_clear_flag(s_timebar_fill, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_timebar_fill, LV_OBJ_FLAG_HIDDEN);

    /* ── 两个地洞 + 地鼠 + 锤子 + 星星 ── */
    for (int i = 0; i < HOLE_COUNT; i++)
    {
        int cx = s_hole_cx[i]; /* 实测洞口中心 x：左洞85，右洞240 */

        /* 地洞已合成在 ds_bg 背景图里，此处是地鼠的【裁剪容器】（透明）。
         * 关键：容器【不开】OVERFLOW_VISIBLE → 默认裁剪，超出容器的地鼠被裁掉，
         *       这就是地鼠"钻进/钻出洞"的视觉来源（等价 HTML overflow:hidden）。
         * 容器底部对齐地面线 HOLE_BOTTOM_Y，高度=WHACK_MOLE_H，
         * 故容器顶 = HOLE_BOTTOM_Y - WHACK_MOLE_H。 */
        s_hole[i] = lv_obj_create(s_panel);
        lv_obj_set_size(s_hole[i], WHACK_HOLE_W, WHACK_MOLE_H);
        lv_obj_set_style_bg_opa(s_hole[i], LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_hole[i], 0, 0);
        lv_obj_set_style_pad_all(s_hole[i], 0, 0);
        lv_obj_clear_flag(s_hole[i], LV_OBJ_FLAG_SCROLLABLE);
        /* 不加 OVERFLOW_VISIBLE：保持默认裁剪 */
        lv_obj_set_pos(s_hole[i],
                       cx - WHACK_HOLE_W / 2,
                       HOLE_BOTTOM_Y - WHACK_MOLE_H); /* 容器底=地面线 */
        lv_obj_add_flag(s_hole[i], LV_OBJ_FLAG_HIDDEN);

        s_mole[i] = lv_image_create(s_hole[i]);
        lv_image_set_src(s_mole[i], &p2);
        /* x = 容器内居中 + 每洞微调(s_mole_dx)，y 由 mole_set_visual 动态更新 */
        lv_obj_set_pos(s_mole[i],
                       (WHACK_HOLE_W - WHACK_MOLE_W) / 2 + s_mole_dx[i],
                       g.mole_hide_y);
        lv_obj_add_flag(s_mole[i], LV_OBJ_FLAG_HIDDEN);

        s_hammer[i] = lv_image_create(s_panel);
        lv_image_set_src(s_hammer[i], &ds3);
        lv_obj_add_flag(s_hammer[i], LV_OBJ_FLAG_HIDDEN);

        for (int s = 0; s < STAR_COUNT; s++)
        {
            g.stars[i][s].obj = lv_image_create(s_panel);
            lv_image_set_src(g.stars[i][s].obj, &p6);
            lv_obj_add_flag(g.stars[i][s].obj, LV_OBJ_FLAG_HIDDEN);
            g.stars[i][s].active = false;
        }
    }
}

/* 停掉对局相关定时器（spawn/clock/cd），保留 engine（让星星飞完）*/
static void game_timers_stop(void)
{
    if (s_spawn_tmr)
    {
        lv_timer_pause(s_spawn_tmr);
    }
    if (s_clock_tmr)
    {
        lv_timer_pause(s_clock_tmr);
    }
    if (s_cd_tmr)
    {
        lv_timer_del(s_cd_tmr);
        s_cd_tmr = NULL;
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 对外接口
 * ═══════════════════════════════════════════════════════════════ */
void whack_start(void)
{
    if (!lvgl_port_lock(200))
    {
        ESP_LOGW(TAG, "whack_start 取锁失败");
        return;
    }

    if (s_panel == NULL)
        build_panel();
    else
        lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);

    /* engine 全程运行（动画/星星），对局节拍由 spawn/clock 控制 */
    if (s_engine_tmr == NULL)
        s_engine_tmr = lv_timer_create(engine_cb, ENGINE_MS, NULL);
    else
        lv_timer_resume(s_engine_tmr);

    /* 本版本不分难度：固定一般档，跳过难度选择界面，进入即播放开场时间条动画 */
    g.diff = DIFF_NORMAL;
    enter_countdown(); /* 直接进开场动画（时间条耗空→回满→开打）*/

    lvgl_port_unlock();
    ESP_LOGI(TAG, "打地鼠进入：开场时间条动画");
}

void whack_touch(touch_event_t event)
{
    if (!lvgl_port_lock(100))
        return;

    switch (g.screen)
    {
    /* ── ① 难度选择：左耳上移 / 右耳下移 / 头部确认 ── */
    case WS_SELECT:
        if (event == TOUCH_EVENT_SHORT_PREV_PAGE)
        { /* 左耳：上移（循环）*/
            g.diff = (g.diff == DIFF_EASY) ? DIFF_HARD : (whack_diff_t)(g.diff - 1);
            select_render();
        }
        else if (event == TOUCH_EVENT_SHORT_NEXT_PAGE)
        { /* 右耳：下移（循环）*/
            g.diff = (g.diff == DIFF_HARD) ? DIFF_EASY : (whack_diff_t)(g.diff + 1);
            select_render();
        }
        else if (event == TOUCH_EVENT_SHORT_HEAD)
        { /* 头部：确认 → 倒计时 */
            enter_countdown();
        }
        break;

    /* ── ② 倒计时：忽略一切输入（防提前点击）── */
    case WS_COUNTDOWN:
        break;

    /* ── ③ 游戏进行：左右耳打洞，头部忽略 ── */
    case WS_PLAYING:
        if (event == TOUCH_EVENT_SHORT_PREV_PAGE)
            try_hit(0);
        else if (event == TOUCH_EVENT_SHORT_NEXT_PAGE)
            try_hit(1);
        break;

    /* ── ④ 结算：头部重玩（本版本不分难度，直接回开场动画重开），其余忽略 ── */
    case WS_RESULT:
        if (event == TOUCH_EVENT_SHORT_HEAD)
            enter_countdown();
        break;

    default:
        break;
    }

    lvgl_port_unlock();
}

void whack_stop(void)
{
    if (!lvgl_port_lock(200))
        return;
    if (s_panel == NULL)
    {
        lvgl_port_unlock();
        return;
    }

    if (s_spawn_tmr)
    {
        lv_timer_del(s_spawn_tmr);
        s_spawn_tmr = NULL;
    }
    if (s_engine_tmr)
    {
        lv_timer_del(s_engine_tmr);
        s_engine_tmr = NULL;
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
    restart_timer_cancel(); /* 退出时撤掉自动重开，防面板删后回调野指针 */

    /* 退出时若正在震动，引擎已停不会再有人关它，这里兜底关闭，防马达一直震 */
    if (g.motor_t0 != 0)
    {
        bsp_motor_set(0);
        g.motor_t0 = 0;
    }

    lv_obj_del(s_panel);
    s_panel = NULL;
    s_hud = NULL;
    s_center = NULL;
    s_hint = NULL;
    s_dim = NULL;
    s_timebar_bg = NULL;
    s_timebar_fill = NULL;
    for (int i = 0; i < HOLE_COUNT; i++)
    {
        s_hole[i] = NULL;
        s_mole[i] = NULL;
        s_hammer[i] = NULL;
        for (int s = 0; s < STAR_COUNT; s++)
        {
            g.stars[i][s].obj = NULL;
            g.stars[i][s].active = false;
        }
    }

    lvgl_port_unlock();
    ESP_LOGI(TAG, "打地鼠已退出，资源已释放");
}
