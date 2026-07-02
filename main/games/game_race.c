/**
 * @file game_race.c
 * @brief 赛车（Racing）游戏实现（三车道·透视版）
 *
 * 全部使用图片素材（c8 背景 + c7 玩家 + c1~c6 障碍），结构与打地鼠
 * （game_whack.c）同源，复用四子状态机与生命周期约定。
 *
 * ── 透视赛道（320×240，障碍自远向近冲来）──
 *   背景 c8 是一点透视的三车道（近宽远窄）。障碍物用「进度 t」驱动：
 *     t = 0    → 远端(消失点附近, 屏幕 y=RACE_FAR_Y)，最小
 *     t = 1000 → 近端(屏幕底缘 y=RACE_NEAR_Y)，最大
 *   每帧 t += 当前速度，渲染时三件事同时按 t 线性插值：
 *     ① 纵向 y   = lerp(FAR_Y, NEAR_Y)
 *     ② 车道横向 x = lerp(车道远端中心, 车道近端中心)（左右道越近越往两边张开）
 *     ③ 缩放 scale = near_scale × lerp(FAR_RATIO, 1.0)（近大远小）
 *   玩家 c7 固定贴底，可在左/中/右三车道间瞬间切换；下 1/5 落到屏外不显示。
 *   障碍中心 y 越过 RACE_COLLIDE_Y 时与玩家比对车道：同道=撞车，异道=放过 +1 分。
 *
 * ── 子状态机（全部在本模块内部，ui_port 只认一个 UI_VIEW_GAME）──
 *   ① RS_COUNTDOWN 3 秒倒计时（大数字 3→2→1→GO，期间忽略输入）
 *   ② RS_PLAYING   游戏进行（左耳左移一道 / 右耳右移一道）
 *   ③ RS_RESULT    结算（居中显示「游戏结束 + 得分」，头部=重玩）
 *
 * 入口直接进倒计时开打（固定简单档，无难度选择）。游戏过程中不显示分数；
 * 撞车进结算界面，居中文字显示「游戏结束\n得分 X 分」。无高分、无红屏闪烁。
 * 玩法与布局参数全部宏化，见 race_sprites.h。
 */
#include "game_race.h"
#include "race_sprites.h"
#include "ui/ui_port.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_random.h"
#include "bsp/bsp_config.h"
#include "bsp/bsp_board.h"
#include "esp_heap_caps.h"
#include <stdio.h>

LV_FONT_DECLARE(font_cn_16);
LV_FONT_DECLARE(font_cn_32); /* 32px 中文大字库（仅含游戏用字），用于居中大文字 */

static const char *TAG = "RACE";

/* ── 引擎帧周期（动画/物理），与打地鼠一致 20fps ── */
#define ENGINE_MS 50

/* ── 难度档（数组下标，与 NVS key 一一对应）── */
typedef enum
{
    DIFF_EASY = 0,
    DIFF_NORMAL,
    DIFF_HARD,
    DIFF_COUNT,
} race_diff_t;

/* 难度参数表：{障碍基准速度(t千分比/帧), 生成间隔基准ms, 中文名, NVS key} */
typedef struct
{
    int speed_t;
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
    RS_COUNTDOWN = 0, /* 3 秒倒计时 */
    RS_PLAYING,       /* 游戏进行 */
    RS_RESULT,        /* 结算（游戏结束+得分）*/
} race_screen_t;

/* ── 障碍物对象池单元（透视：用进度 t 驱动，而非绝对像素）── */
typedef struct
{
    lv_obj_t *obj; /* 对应的 LVGL image，常驻，靠显隐 + 换图 + 缩放复用 */
    int t;         /* 透视进度 0..1000（0=远端最小，1000=近端最大）*/
    int8_t lane;   /* 所在车道 LANE_LEFT / LANE_MID / LANE_RIGHT */
    uint8_t idx;   /* 障碍种类 0..RACE_OBST_COUNT-1（→ c1~c6）*/
    bool active;   /* 是否在场 */
    bool checked;  /* 是否已过碰撞判定线（避免重复判定）*/
} enemy_t;

/* ── 全局状态 ── */
static struct
{
    race_screen_t screen; /* 当前子界面 */
    race_diff_t diff;     /* 当前选中难度 */

    /* 本局动态参数（= 难度基准，运行中随 10 秒加速变化）*/
    int cur_speed_t;
    int cur_spawn_ms;

    /* 倒计时 */
    int cd_value; /* 3 → 2 → 1 → 0(GO) */

    /* 对局数据 */
    bool over;
    int score;     /* 得分（每放过一个障碍 +1）*/
    int elapsed_s; /* 已进行秒数，用于 10 秒加速 */

    /* 玩家所在车道 LANE_LEFT / LANE_MID / LANE_RIGHT */
    int8_t player_lane;

    /* 障碍物对象池 */
    enemy_t enemies[RACE_MAX_ENEMIES];
} g;

/* ── LVGL 对象 ── */
static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_bg = NULL;     /* 背景 c8（最底层）*/
static uint8_t *s_bg_buf = NULL;  /* c8 缓存到 PSRAM 的整文件缓冲（开机读一次）*/
static lv_image_dsc_t s_bg_dsc;   /* 指向 PSRAM 缓冲的背景描述符（内存驻留，免每帧回外挂解码）*/
static lv_obj_t *s_player = NULL; /* 玩家车 c7 */
static lv_obj_t *s_center = NULL; /* 居中大文字（难度/倒计时/结算共用）*/
static lv_obj_t *s_hint = NULL;   /* 底部操作提示 */

static lv_timer_t *s_engine_tmr = NULL; /* 动画/物理引擎 */
static lv_timer_t *s_spawn_tmr = NULL;  /* 障碍生成节拍 */
static lv_timer_t *s_clock_tmr = NULL;  /* 1Hz 倒计时 + 10 秒加速 */
static lv_timer_t *s_cd_tmr = NULL;     /* 开局 3 秒倒计时 */

/* ── 素材/车道查表（const，编译期定死）── */
/* 障碍素材（C 数组，内存驻留→可实时缩放），随机选一张换图 */
static const lv_image_dsc_t *const s_obst[RACE_OBST_COUNT] = {&c1, &c2, &c3, &c4, &c5, &c6};
/* 各障碍原始尺寸（与 c1~c6.png 一致，用于居中定位）*/
static const int s_obst_w[RACE_OBST_COUNT] = {48, 60, 88, 41, 134, 44};
static const int s_obst_h[RACE_OBST_COUNT] = {35, 36, 45, 20, 63, 39};
/* 各障碍「近端(t=1000)」缩放(256=原尺寸)：= 近端目标宽 × 256 / 原始宽。
 * 近端目标宽均 ≤ 底部单车道宽(≈156px) 的约 78%，留出躲避缝隙：
 * 目标宽 {84,84,96,84,110,78}，故 near_scale 如下。*/
static const int s_obst_near_scale[RACE_OBST_COUNT] = {448, 358, 279, 524, 210, 454};

/* 玩家三车道固定中心 x（玩家不随 t 变，贴底固定）*/
static const int s_player_lane_x[LANE_COUNT] = {RACE_PLAYER_LANE_L, RACE_PLAYER_LANE_M, RACE_PLAYER_LANE_R};

/* ── 一点透视·直线车道（按屏幕 y 取三车道中心 x）──
 * 走廊是中心对称一点透视：三条平行车道投影成收束于中心(x≈160)的直线。
 * 中道垂直恒 160，左右对称斜线。每条道用「远端/近端」两点线性插值（=直线），
 * cx 与 cy 同由 tv 驱动 → (cx,cy) 必落在直线上，严格沿车道、绝不拐弧。
 * 坐标全部来自 race_sprites.h 的宏，要微调通道位置就改那 6 个 _X 宏。*/
static void track_lane_x_at(int cy, int *lx, int *mx, int *rx)
{
    if (cy <= RACE_FAR_Y)
    {
        *lx = RACE_FAR_L_X;
        *mx = RACE_FAR_M_X;
        *rx = RACE_FAR_R_X;
        return;
    }
    if (cy >= RACE_NEAR_Y)
    {
        *lx = RACE_NEAR_L_X;
        *mx = RACE_NEAR_M_X;
        *rx = RACE_NEAR_R_X;
        return;
    }
    int f = (cy - RACE_FAR_Y) * 1000 / (RACE_NEAR_Y - RACE_FAR_Y); /* 0..1000 深度进度 */
    *lx = RACE_FAR_L_X + (RACE_NEAR_L_X - RACE_FAR_L_X) * f / 1000;
    *mx = RACE_FAR_M_X + (RACE_NEAR_M_X - RACE_FAR_M_X) * f / 1000;
    *rx = RACE_FAR_R_X + (RACE_NEAR_R_X - RACE_FAR_R_X) * f / 1000;
}

/* 进度 t(0..1000 线性=时间) → 视觉进度 tv：二次缓动，远慢近冲（模拟 1/Z 加速）。*/
static inline int enemy_tv(int t) { return t * t / 1000; }

/* 进度 t → 障碍中心屏幕 y（供碰撞判定与渲染共用）。*/
static inline int enemy_cy_from_t(int t)
{
    return RACE_FAR_Y + (RACE_NEAR_Y - RACE_FAR_Y) * enemy_tv(t) / 1000;
}

/* 前置声明 */
static void enter_countdown(void);
static void enter_playing(void);
static void enter_result(void);

/* ═══════════════════════════════════════════════════════════════
 * 视觉辅助
 * ═══════════════════════════════════════════════════════════════ */

/* 随机选一种障碍素材索引（c1~c6）*/
static inline uint8_t obst_random_idx(void)
{
    return (uint8_t)(esp_random() % RACE_OBST_COUNT);
}

/* 把所有障碍藏起来并标记空闲（切界面清场）*/
static void enemies_hide_all(void)
{
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        g.enemies[i].active = false;
        if (g.enemies[i].obj)
            lv_obj_add_flag(g.enemies[i].obj, LV_OBJ_FLAG_HIDDEN);
    }
}

/* 刷新玩家车位置（车道切换后调用）。玩家贴底固定，仅 x 随车道变。
 * 顶边固定 RACE_PLAYER_TOP_Y，使下 1/5 落到屏外被父对象裁掉而不显示。*/
static void player_refresh(void)
{
    if (s_player == NULL)
        return;
    int cx = s_player_lane_x[g.player_lane];
    lv_obj_set_pos(s_player, cx - RACE_PLAYER_W / 2, RACE_PLAYER_TOP_Y);
}

/* 按进度 t 渲染一个障碍：插值 {纵向 y、车道横向 x、缩放}，居中定位。
 * LVGL set_scale 绕图片中心缩放，set_pos 设的是「缩放前」左上角，
 * 故只需把原始尺寸框中心对齐到 (cx, cy)，缩放会自动绕该中心扩缩。*/
static void enemy_render(enemy_t *e)
{
    int t = e->t; /* 0..1000 线性（=时间，匀速推进）*/
    int idx = e->idx;
    int lane = e->lane;

    int tv = enemy_tv(t); /* 视觉进度（二次缓动，远慢近冲）*/

    /* ① 纵向 y：从远端门洞 → 近端底缘 */
    int cy = RACE_FAR_Y + (RACE_NEAR_Y - RACE_FAR_Y) * tv / 1000;

    /* ② 横向 x：按当前 y 直线插值取三车道中心（中道也用插值，对称一点透视）*/
    int lx = RACE_FAR_L_X, mx = RACE_FAR_M_X, rx = RACE_FAR_R_X;
    track_lane_x_at(cy, &lx, &mx, &rx);
    int cx = (lane == LANE_LEFT) ? lx : (lane == LANE_RIGHT) ? rx
                                                             : mx;

    /* ③ 缩放（由小变大）：远端 = near_scale × RACE_FAR_SCALE/1000，近端 = near_scale。
     * factor 从 RACE_FAR_SCALE(远) 线性升到 1000(近)。这是「近大远小」的核心。*/
    int factor = RACE_FAR_SCALE + (1000 - RACE_FAR_SCALE) * tv / 1000;
    int scale = s_obst_near_scale[idx] * factor / 1000;
    if (scale < 16)
        scale = 16; /* LVGL 最小缩放兜底，防渲染异常 */

    /* ④ 门洞淡入：t<RACE_FADE_T 时透明度 0→255，消除门洞处凭空蹦出 */
    lv_opa_t opa = (t < RACE_FADE_T) ? (lv_opa_t)(255 * t / RACE_FADE_T) : LV_OPA_COVER;
    lv_obj_set_style_image_opa(e->obj, opa, 0);

    /* ⑤ 居中定位：LVGL set_scale 绕图片中心缩放，set_pos 设缩放前左上角，
     * 故把原始尺寸框中心对齐 (cx, cy) 即可，缩放自动绕中心扩缩。*/
    int w = s_obst_w[idx], hh = s_obst_h[idx];
    lv_image_set_scale(e->obj, scale);
    lv_obj_set_pos(e->obj, cx - w / 2, cy - hh / 2);
}

/* ═══════════════════════════════════════════════════════════════
 * 引擎：动画 + 物理（所有子界面共用，只在 PLAYING 推进对局逻辑）
 * ═══════════════════════════════════════════════════════════════ */
static void engine_cb(lv_timer_t *t)
{
    (void)t;

    if (g.screen != RS_PLAYING || g.over)
        return;

    bool dirty = false;

    /* ── 障碍推进 + 碰撞/计分 ── */
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        enemy_t *e = &g.enemies[i];
        if (!e->active)
            continue;

        e->t += g.cur_speed_t; /* 沿透视轨道向近端推进（线性时间）*/

        /* 越过碰撞判定线：障碍中心屏幕 y 到达玩家深度时与玩家比对车道（只判一次）。
         * 用屏幕 y 而非线性 t，配合二次曲线才准（玩家固定在某屏幕深度）。*/
        if (!e->checked && enemy_cy_from_t(e->t) >= RACE_COLLIDE_Y)
        {
            e->checked = true;
            if (e->lane == g.player_lane)
            {
                /* 撞车：进结算界面显示「游戏结束 + 得分」。*/
                ESP_LOGI(TAG, "得分=%d", g.score);
                enter_result();
                return; /* 立即停止本帧后续更新 */
            }
        }

        /* 冲到近端外：放过，加分回收 */
        if (e->t >= 1000)
        {
            e->active = false;
            lv_obj_add_flag(e->obj, LV_OBJ_FLAG_HIDDEN);
            g.score++;
            dirty = true;
            continue;
        }

        enemy_render(e);
    }

    (void)dirty; /* 游戏中不显示分数，得分仅在结算时展示 */
}

/* ── 障碍生成节拍：选一条「安全」车道（不会刚生成就贴脸、且至少留一条逃生道）── */
static void spawn_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != RS_PLAYING || g.over)
        return;

    /* 找一个空闲对象池槽位 */
    int slot = -1;
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
        if (!g.enemies[i].active)
        {
            slot = i;
            break;
        }
    if (slot < 0)
        return; /* 满了，本拍不生成 */

    /* 候选车道：该车道当前没有「刚出生(t<SAFE_T)」的障碍。
     * 同时统计已被占用的车道数，保证生成后不会三道全堵（至少留一条逃生道）。*/
    int cand[LANE_COUNT];
    int nc = 0;
    int busy_lanes = 0;
    for (int lane = 0; lane < LANE_COUNT; lane++)
    {
        bool fresh = false;    /* 该道有刚出生的障碍 */
        bool occupied = false; /* 该道有任意在场障碍 */
        for (int i = 0; i < RACE_MAX_ENEMIES; i++)
        {
            enemy_t *e = &g.enemies[i];
            if (!e->active || e->lane != lane)
                continue;
            occupied = true;
            if (e->t < RACE_SPAWN_SAFE_T)
                fresh = true;
        }
        if (occupied)
            busy_lanes++;
        if (!fresh)
            cand[nc++] = lane;
    }

    /* 已有两条道被占用：再生成第三条会堵死，跳过本拍（强制留逃生道）*/
    if (busy_lanes >= LANE_COUNT - 1)
        return;
    if (nc == 0)
        return;

    int lane = cand[esp_random() % nc];
    enemy_t *e = &g.enemies[slot];
    e->lane = (int8_t)lane;
    e->idx = obst_random_idx();
    e->t = 0;
    e->checked = false;
    e->active = true;

    lv_image_set_src(e->obj, s_obst[e->idx]);
    /* 缩放锚点 = 本图中心：图片绕自身中心缩放，中心点恒钉在车道点(cx,cy)，
     * 任何大小都不漂移（消除「远处飘出车道、近处才正」的根因）。
     * 锚点用源图坐标，故按该障碍原始尺寸的一半设；换图后须重设，所以放这里。*/
    lv_image_set_pivot(e->obj, s_obst_w[e->idx] / 2, s_obst_h[e->idx] / 2);
    enemy_render(e);
    lv_obj_clear_flag(e->obj, LV_OBJ_FLAG_HIDDEN);
}

/* ── 1Hz 时钟：计时 + 每 10 秒持续加速（无时间上限，撞车才结束）── */
static void clock_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != RS_PLAYING || g.over)
        return;

    g.elapsed_s++;

    if (g.elapsed_s % RACE_ACCEL_EVERY_S == 0)
    {
        g.cur_speed_t += RACE_ACCEL_SPEED_ADD;
        if (g.cur_speed_t > RACE_SPEED_MAX)
            g.cur_speed_t = RACE_SPEED_MAX;
        g.cur_spawn_ms = g.cur_spawn_ms * RACE_ACCEL_SPAWN_PCT / 100;
        if (g.cur_spawn_ms < RACE_SPAWN_MS_MIN)
            g.cur_spawn_ms = RACE_SPAWN_MS_MIN;
        if (s_spawn_tmr)
            lv_timer_set_period(s_spawn_tmr, g.cur_spawn_ms);
        ESP_LOGI(TAG, "提速@%ds: speed=%d spawn=%d", g.elapsed_s, g.cur_speed_t, g.cur_spawn_ms);
    }
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
    /* 提前露出赛道氛围：玩家车贴底（默认中间道）*/
    g.player_lane = LANE_MID;
    player_refresh();
    if (s_player)
        lv_obj_clear_flag(s_player, LV_OBJ_FLAG_HIDDEN);

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
    g.player_lane = LANE_MID;

    /* 本局动态参数 = 当前难度基准 */
    g.cur_speed_t = s_diff[g.diff].speed_t;
    g.cur_spawn_ms = s_diff[g.diff].spawn_ms;

    enemies_hide_all();
    player_refresh();
    if (s_player)
        lv_obj_clear_flag(s_player, LV_OBJ_FLAG_HIDDEN);
    if (s_center)
        lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    /* 游戏中不显示分数，得分仅在结算界面展示 */

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
             s_diff[g.diff].name, g.cur_speed_t, g.cur_spawn_ms);
}

/* ═══════════════════════════════════════════════════════════════
 * 子界面：③ 结算（游戏结束 + 得分）
 * 仅显示「游戏结束」与本局得分，无高分、无红色；头部按键重玩。
 * ═══════════════════════════════════════════════════════════════ */
static void enter_result(void)
{
    g.over = true;
    g.screen = RS_RESULT;

    if (s_spawn_tmr)
        lv_timer_pause(s_spawn_tmr);
    if (s_clock_tmr)
        lv_timer_pause(s_clock_tmr);
    enemies_hide_all();
    if (s_player)
        lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);

    if (s_center)
    {
        char buf[48];
        snprintf(buf, sizeof(buf), "得分 %d 分", g.score);
        lv_label_set_text(s_center, buf);
        lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    }
    ESP_LOGI(TAG, "结算: 得分=%d", g.score);
}

/* ═══════════════════════════════════════════════════════════════
 * 背景缓存：把外挂 c8.bin 开机读一次解到 PSRAM，构造内存驻留 dsc。
 *
 * 为什么：c8 留在外挂 Flash，若直接 set_src(文件路径)，障碍每帧移动盖过背景 →
 * LVGL 局部重绘那块背景 → 每次都回外挂重新解码 c8 那一片，外挂读很慢，肉眼可见卡。
 * 解决：开机一次性把整张 c8.bin 读进 PSRAM（仍存外挂，只是读一次进 RAM），
 * 之后背景就是内存图，局部重绘直接读 RAM，不再碰外挂。
 *
 * .bin 文件 = lv_image_header_t(12B) + 像素数据，直接拆成内存 dsc 即可（无需二次解码）。
 * 成功返回 &s_bg_dsc；失败返回 NULL（调用方回退为直接用文件路径）。
 * ═══════════════════════════════════════════════════════════════ */
static const lv_image_dsc_t *bg_cache_to_psram(void)
{
    if (s_bg_buf)
        return &s_bg_dsc; /* 已缓存，直接复用 */

    FILE *f = fopen("/S/img/c8.bin", "rb");
    if (!f)
    {
        ESP_LOGW(TAG, "打不开 /S/img/c8.bin，背景回退为文件直读");
        return NULL;
    }
    fseek(f, 0, SEEK_END);
    long sz = ftell(f);
    fseek(f, 0, SEEK_SET);
    if (sz <= (long)sizeof(lv_image_header_t))
    {
        fclose(f);
        ESP_LOGW(TAG, "c8.bin 异常(%ld B)，背景回退为文件直读", sz);
        return NULL;
    }

    s_bg_buf = heap_caps_malloc(sz, MALLOC_CAP_SPIRAM);
    if (!s_bg_buf)
    {
        fclose(f);
        ESP_LOGW(TAG, "PSRAM 不足(%ld B)，背景回退为文件直读", sz);
        return NULL;
    }
    size_t rd = fread(s_bg_buf, 1, (size_t)sz, f);
    fclose(f);
    if (rd != (size_t)sz)
    {
        heap_caps_free(s_bg_buf);
        s_bg_buf = NULL;
        ESP_LOGW(TAG, "c8.bin 读取不完整(%u/%ld)，背景回退为文件直读", (unsigned)rd, sz);
        return NULL;
    }

    /* 前 12B 是 lv_image_header_t（含 magic/cf/w/h/stride），其后是像素数据 */
    memcpy(&s_bg_dsc.header, s_bg_buf, sizeof(lv_image_header_t));
    s_bg_dsc.data = s_bg_buf + sizeof(lv_image_header_t);
    s_bg_dsc.data_size = (uint32_t)(sz - (long)sizeof(lv_image_header_t));
    s_bg_dsc.reserved = NULL;
    ESP_LOGI(TAG, "背景 c8 已缓存到 PSRAM: %ux%u cf=%u (%ld B)",
             (unsigned)s_bg_dsc.header.w, (unsigned)s_bg_dsc.header.h,
             (unsigned)s_bg_dsc.header.cf, sz);
    return &s_bg_dsc;
}

/* ═══════════════════════════════════════════════════════════════
 * 面板构建（一次性，所有子界面共用同一组 LVGL 对象）
 *
 * 层级（创建顺序 = z 序，先创建在底）：
 *   背景 c8 → 障碍池 c1~c6 → 玩家 c7 → HUD/大文字/闪烁遮罩
 * 这样玩家恒在障碍之上、HUD/闪烁在最上。
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

    /* ── 背景 c8（320×240 全屏铺底，外挂读一次缓存到 PSRAM；失败回退文件直读）── */
    s_bg = lv_image_create(s_panel);
    const lv_image_dsc_t *bg = bg_cache_to_psram();
    if (bg)
        lv_image_set_src(s_bg, bg);
    else
        lv_image_set_src(s_bg, RACE_IMG_C8);
    lv_obj_set_pos(s_bg, 0, 0);

    /* ── 障碍物对象池（常驻 image，靠显隐 + 换图 + 缩放复用）── */
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        g.enemies[i].obj = lv_image_create(s_panel);
        lv_image_set_src(g.enemies[i].obj, &c1);
        lv_obj_add_flag(g.enemies[i].obj, LV_OBJ_FLAG_HIDDEN);
        g.enemies[i].active = false;
    }

    /* ── 玩家车 c7（贴底，x 随车道切换；下 1/5 落屏外被裁）── */
    s_player = lv_image_create(s_panel);
    lv_image_set_src(s_player, &c7);
    lv_image_set_scale(s_player, RACE_PLAYER_SCALE); /* 缩小到约 0.78× */
    lv_obj_set_pos(s_player, RACE_PLAYER_LANE_M - RACE_PLAYER_W / 2, RACE_PLAYER_TOP_Y);
    lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);

    /* ── 居中大文字（倒计时/结算共用）── 用 32px 中文大字库 */
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

    /* engine 全程运行（动画/闪烁收尾），对局节拍由 spawn/clock 控制 */
    if (s_engine_tmr == NULL)
        s_engine_tmr = lv_timer_create(engine_cb, ENGINE_MS, NULL);
    else
        lv_timer_resume(s_engine_tmr);

    g.diff = DIFF_EASY; /* 固定简单档（难度选择已去除）*/
    enter_countdown();  /* 入口直接进倒计时开打，跳过难度选择 */

    lvgl_port_unlock();
    ESP_LOGI(TAG, "赛车进入：直接开打");
}

void race_touch(touch_event_t event)
{
    if (!lvgl_port_lock(100))
        return;

    static uint32_t last_touch = 0; /* 换道防误触时间戳 */

    switch (g.screen)
    {
    /* ── ② 倒计时：忽略一切输入 ── */
    case RS_COUNTDOWN:
        break;

    /* ── ③ 游戏进行：左耳左移一道 / 右耳右移一道（三道间，带防误触）── */
    case RS_PLAYING:
        if (g.over)
            break;
        if (event == TOUCH_EVENT_SHORT_PREV_PAGE || event == TOUCH_EVENT_SHORT_NEXT_PAGE)
        {
            uint32_t now = lv_tick_get();
            if (now - last_touch < RACE_TOUCH_COOLDOWN_MS)
                break; /* 防误触：冷却期内忽略 */
            int want = g.player_lane +
                       (event == TOUCH_EVENT_SHORT_NEXT_PAGE ? 1 : -1);
            if (want < LANE_LEFT)
                want = LANE_LEFT;
            if (want > LANE_RIGHT)
                want = LANE_RIGHT;
            if (want != g.player_lane)
            {
                g.player_lane = (int8_t)want;
                player_refresh();
                last_touch = now;
            }
        }
        break;

    /* ── ③ 结算：头部重玩（重新倒计时开打），其余忽略 ── */
    case RS_RESULT:
        if (event == TOUCH_EVENT_SHORT_HEAD)
            enter_countdown();
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

    lv_obj_del(s_panel); /* 子对象（背景/玩家/障碍/文字/闪烁）随父一并删除 */
    s_panel = NULL;
    s_bg = NULL;
    /* 释放背景 PSRAM 缓存（s_bg 已随 panel 删除，不再引用 s_bg_buf，可安全释放）*/
    if (s_bg_buf)
    {
        heap_caps_free(s_bg_buf);
        s_bg_buf = NULL;
    }
    s_player = NULL;
    s_center = NULL;
    s_hint = NULL;
    for (int i = 0; i < RACE_MAX_ENEMIES; i++)
    {
        g.enemies[i].obj = NULL;
        g.enemies[i].active = false;
    }

    lvgl_port_unlock();
    ESP_LOGI(TAG, "赛车已退出，资源已释放");
}
