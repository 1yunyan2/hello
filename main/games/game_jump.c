/**
 * @file game_jump.c
 * @brief 跳一跳游戏实现
 *
 * 坐标系：
 *   - 台子使用「世界坐标」(world_cx, world_cy)，不受摄像机影响
 *   - 摄像机 (cam_x, cam_y)，屏幕坐标 = 世界坐标 - 摄像机偏移
 *   - 屏幕中心锚点：小人始终在 (JUMP_START_X, JUMP_START_Y) 附近
 *
 * 台子池：3个槽位 [0]=cur [1]=next [2]=prenext
 *   每次落台后：[0]←[1]←[2]，[2]生成新台，旧[0]的obj复用给新[2]
 *
 * 摄像机：落台成功后平滑滑动到新位置（JUMP_CAM_SMOOTH_FRAMES帧）
 *
 * 飞行：二维抛物线，X+Y方向同时运动
 *   参数化 t=0~1，dx=fly_total_x*t, dy=fly_total_y*t
 *   高度 h = 4*apex*t*(1-t)（弧顶在t=0.5处）
 *
 * 子相：PH_IDLE → PH_FLY → PH_CAM → PH_IDLE（成功；压扁只在 PH_IDLE 蓄力时）
 *                         → PH_FALL → enter_result()（失败）
 *   注：原 PH_LAND「着陆压扁」已去掉——压扁只在蓄力，落台不压。
 */
#include "game_jump.h"
#include "jump_sprites.h"
#include "cube3d.h"
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
LV_FONT_DECLARE(font_cn_32);

static const char *TAG = "JUMP";

/* ── 难度 ── */
typedef enum
{
    DIFF_EASY = 0,
    DIFF_NORMAL,
    DIFF_HARD,
    DIFF_COUNT
} jump_diff_t;

typedef struct
{
    int pw, gap_min, gap_max;
    const char *name, *nvs_key;
} diff_param_t;

static const diff_param_t s_diff[DIFF_COUNT] = {
    [DIFF_EASY] = {JUMP_PW_EASY, JUMP_GAP_MIN_EASY, JUMP_GAP_MAX_EASY, "简单", "hi_easy"},
    [DIFF_NORMAL] = {JUMP_PW_NORMAL, JUMP_GAP_MIN_NORMAL, JUMP_GAP_MAX_NORMAL, "一般", "hi_normal"},
    [DIFF_HARD] = {JUMP_PW_HARD, JUMP_GAP_MIN_HARD, JUMP_GAP_MAX_HARD, "困难", "hi_hard"},
};

#define JUMP_NVS_NS "jump"

/* ── 状态机 ── */
typedef enum
{
    JS_SELECT = 0,
    JS_PLAYING,
    JS_RESULT
} jump_screen_t;
typedef enum
{
    PH_IDLE = 0,
    PH_FLY,
    PH_LAND,
    PH_CAM,
    PH_FALL
} jump_phase_t;

/* ── 台子（世界坐标，双轴）── */
#define PLAT_COUNT 3 /* cur(0) + next(1) + prenext(2) */

typedef struct
{
    lv_obj_t *cube;  /* 等轴测立方体（cube3d 控件）*/
    int world_cx;    /* 世界坐标中心 x */
    int world_cy;    /* 世界坐标中心 y（台面顶边）*/
    int w;           /* 台子宽度（= 立方体边长基准）*/
    int style;       /* 样式索引（指向 s_plat_styles[]，决定顶面色）*/
    int depth;       /* 立方体竖直厚度（随机，做扁/高台非对称外形）*/
    uint8_t stripes; /* 侧面亮条带掩码（随机）*/
    uint8_t pattern; /* 顶面/正面几何花纹类型（随机，0=无）*/
    bool active;
    /* ── 从天而降入场动画 ── */
    int drop;      /* 当前竖直入场偏移（>0=在目标上方，渲染时 sy 减去它）*/
    int drop_vy;   /* 掉落速度（每帧px，向下加速）*/
    int bounce;    /* 触底回弹压扁余量（0~100，渲染时加到 squash）*/
    bool dropping; /* 入场动画进行中 */
} platform_t;

/* ── 立体台子样式表（顶面色；正面/右侧面由 cube3d 自动调暗生成）──
 * 后期“五六种不同大小/颜色”的台子，往这里加条目即可，渲染逻辑无需改动。
 * 大小由 cur_platform_w() 给宽度，这里只管颜色风格。*/
static const uint32_t s_plat_styles[JUMP_PLAT_STYLE_COUNT] = {
    0x8FBF8F, /* 灰绿（参考素材色）*/
    0x8FAFD4, /* 蓝灰 */
    0xD4B98F, /* 暖棕 */
    0xB98FD4, /* 紫 */
    0xD48F9F, /* 暖红 */
};

/* ── 全局状态 ── */
static struct
{
    jump_screen_t screen;
    jump_diff_t diff;
    jump_phase_t phase;

    int high_score, score;

    /* 台子池：[0]=cur [1]=next [2]=prenext */
    platform_t plats[PLAT_COUNT];

    /* 摄像机世界坐标偏移：screen_pos = world_pos - cam */
    int cam_x, cam_y;

    /* 摄像机平滑：目标值 + 剩余帧数 */
    int cam_target_x, cam_target_y;
    int cam_smooth_frames;

    /* 小人世界坐标 */
    int player_wx;
    int player_wy; /* 脚底世界 y（等于当前台 world_cy）*/

    /* 飞行（参数化，t从0→FLY_STEPS） */
    int fly_total_x; /* 目标台 world_cx - 起跳 world_cx */
    int fly_total_y; /* 目标台 world_cy - 起跳 world_cy */
    int fly_dist_px; /* 直线距离，用于算弧顶 */
    int fly_apex;    /* 弧顶高度（屏幕px，向上为正）*/
    int fly_step;    /* 当前步 0~fly_steps */
    int fly_steps;   /* 总步数 = fly_dist_px / FLY_STEP_PX */
    int fly_start_wx, fly_start_wy;

    /* 掉落（落空时继承飞行末态，做连续抛物线，而非垂直撞墙）*/
    int fall_vy; /* 垂直掉落速度（每帧px，向下加速）*/
    int fall_vx; /* 水平速度（继承飞行末尾的每帧水平位移，继续往前飞）*/
    int fall_h;  /* 当前离地弧高余量（从飞行末尾 fly_h 继承，被重力逐帧吃掉）*/
} g;

/* ── 蓄力光斑粒子 ──
 * 蓄力时人物四周生成彩色小点，逐帧向人物中心移动并缩小，到中心后换随机角度/颜色重生，
 * 模拟“聚气”光斑。只在蓄力时显示，松手/起跳全部隐藏。*/
#define JUMP_SPARK_COUNT 8
typedef struct
{
    int ang;        /* 角度（0~359 度，整数）*/
    int radius;     /* 距人物中心半径（px）*/
    uint32_t color; /* 颜色 */
} spark_t;

/* ── LVGL 对象 ── */
static lv_obj_t *s_panel = NULL;
static lv_obj_t *s_player = NULL;                  /* 人物图片 j1 */
static lv_obj_t *s_player_shadow = NULL;           /* 人物脚下椭圆影子 */
static lv_obj_t *s_sparks[JUMP_SPARK_COUNT] = {0}; /* 光斑粒子对象池 */
static spark_t s_spark_st[JUMP_SPARK_COUNT];
static lv_obj_t *s_hud = NULL;
static lv_obj_t *s_center = NULL;

static lv_timer_t *s_engine_tmr = NULL;

/* 前置 */
static void enter_select(void);
static void enter_playing(void);
static void enter_result(void);

/* ══════════════════════════════════════════════════
 * NVS
 * ══════════════════════════════════════════════════ */
/* 高分三难度共享：统一存 NVS key "hi_all"（不再按难度分开）*/
#define JUMP_NVS_KEY_HISCORE "hi_all"

static int highscore_load(void)
{
    nvs_handle_t h;
    int32_t v = 0;
    if (nvs_open(JUMP_NVS_NS, NVS_READONLY, &h) == ESP_OK)
    {
        nvs_get_i32(h, JUMP_NVS_KEY_HISCORE, &v);
        nvs_close(h);
    }
    return (int)v;
}

static bool highscore_save_if_better(int score)
{
    if (score <= g.high_score)
        return false;
    nvs_handle_t h;
    if (nvs_open(JUMP_NVS_NS, NVS_READWRITE, &h) == ESP_OK)
    {
        nvs_set_i32(h, JUMP_NVS_KEY_HISCORE, (int32_t)score);
        nvs_commit(h);
        nvs_close(h);
    }
    g.high_score = score;
    return true;
}

/* ══════════════════════════════════════════════════
 * 辅助
 * ══════════════════════════════════════════════════ */
static void hud_refresh(void)
{
    if (!s_hud)
        return;
    char buf[48];
    /* 「高分」始终显示 NVS 里的历史最高分（不被当前分顶替），作为玩家追赶的目标纪录。
     * 当前分破纪录后，结束结算时才把新纪录写回 NVS（见 enter_result）。*/
    snprintf(buf, sizeof(buf), "分数%d  最高分%d", g.score, g.high_score);
    lv_label_set_text(s_hud, buf);
}

static int cur_platform_w(void)
{
    int w = s_diff[g.diff].pw - g.score * JUMP_PW_SHRINK_PER_SCORE;
    return w < JUMP_PW_MIN ? JUMP_PW_MIN : w;
}

static int rand_gap_x(void)
{
    int lo = s_diff[g.diff].gap_min, hi = s_diff[g.diff].gap_max;
    if (hi <= lo)
        return lo;
    return lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
}

/* Y轴偏移：随机 -JUMP_GAP_Y_MAX ~ +JUMP_GAP_Y_MAX，但不让台子超出可视区 */
static int rand_gap_y(int base_cy)
{
    int max = JUMP_GAP_Y_MAX;
    int offset = (int)(esp_random() % (uint32_t)(max * 2 + 1)) - max;
    /* 限制台面顶边在 [40, LCD_HEIGHT-60] 内 */
    int new_cy = base_cy + offset;
    if (new_cy < 40)
        new_cy = 40;
    if (new_cy > BSP_LCD_HEIGHT - 60)
        new_cy = BSP_LCD_HEIGHT - 60;
    return new_cy - base_cy;
}

static int hold_to_dist(uint32_t ms)
{
    if (ms <= JUMP_HOLD_MIN_MS)
        return JUMP_DIST_MIN_PX;
    if (ms >= JUMP_HOLD_MAX_MS)
        return JUMP_DIST_MAX_PX;
    return JUMP_DIST_MIN_PX +
           (int)((ms - JUMP_HOLD_MIN_MS) * (uint32_t)(JUMP_DIST_MAX_PX - JUMP_DIST_MIN_PX) / (uint32_t)(JUMP_HOLD_MAX_MS - JUMP_HOLD_MIN_MS));
}

/* 整数平方根（用于算直线距离）*/
static int isqrt(int n)
{
    if (n <= 0)
        return 0;
    int x = n, y = (x + 1) / 2;
    while (y < x)
    {
        x = y;
        y = (x + n / x) / 2;
    }
    return x;
}

/* 世界→屏幕坐标 */
static inline int wx_to_sx(int wx) { return wx - g.cam_x; }
static inline int wy_to_sy(int wy) { return wy - g.cam_y; }

/* 台子宽度 → 立方体边长：立方体投影宽 = 2*edge*cos30 ≈ 1.73*edge，
 * 故 edge = w / 1.73 ≈ w * 148/256。*/
static inline int plat_w_to_edge(int w)
{
    int e = w * 148 / 256;
    return e < 8 ? 8 : e;
}

/* ══════════════════════════════════════════════════
 * 台子渲染（squash_pct 0~100：蓄力时立方体竖直方向压扁）
 *   立方体「顶面中心」对齐世界坐标 (world_cx, world_cy)（小人脚踩顶面）。
 * ══════════════════════════════════════════════════ */
static void platform_render_one(int idx, int squash_pct)
{
    platform_t *p = &g.plats[idx];
    if (!p->cube || !p->active)
    {
        if (p->cube)
            lv_obj_add_flag(p->cube, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    int edge = plat_w_to_edge(p->w);
    cube3d_set_geometry(p->cube, edge);
    cube3d_set_depth(p->cube, p->depth); /* 非对称厚度 */
    cube3d_set_top_color(p->cube, s_plat_styles[p->style]);
    cube3d_set_stripes(p->cube, p->stripes); /* 随机亮条带 */
    cube3d_set_pattern(p->cube, p->pattern); /* 随机几何花纹 */
    cube3d_set_shadow(p->cube, false);       /* 不要影子（用户反馈不好看）*/
    /* 压扁 = 蓄力压扁 + 入场触底回弹压扁（取大，封顶100）*/
    int sq = squash_pct + p->bounce;
    if (sq > 100)
        sq = 100;
    cube3d_set_squash(p->cube, sq);

    /* 顶面中心对齐屏幕 (sx, sy)（小人脚踩顶面）。入场掉落时整体上移 drop。*/
    int sx = wx_to_sx(p->world_cx);
    int sy = wy_to_sy(p->world_cy) - p->drop; /* drop>0=在目标上方，往下落 */
    cube3d_place(p->cube, sx, sy);

    lv_obj_clear_flag(p->cube, LV_OBJ_FLAG_HIDDEN);
}

static void platforms_render_all(void)
{
    for (int i = 0; i < PLAT_COUNT; i++)
        platform_render_one(i, 0);
}

/* 每帧推进「从天而降」入场动画：掉落→触底→回弹压扁消退。
 * 返回是否有任一台子在动画中（用于决定是否需要重渲染）。*/
static bool platforms_drop_update(void)
{
    bool any = false;
    for (int i = 0; i < PLAT_COUNT; i++)
    {
        platform_t *p = &g.plats[i];
        if (!p->active || !p->dropping)
            continue;
        any = true;

        if (p->drop > 0)
        {
            /* 下落：重力加速 */
            p->drop_vy += JUMP_DROP_GRAVITY;
            p->drop -= p->drop_vy;
            if (p->drop <= 0)
            { /* 触底：归位 + 触发回弹压扁 */
                p->drop = 0;
                p->bounce = JUMP_DROP_BOUNCE;
            }
        }
        else if (p->bounce > 0)
        {
            /* 回弹：压扁量逐帧衰减到 0 */
            p->bounce -= JUMP_DROP_BOUNCE_DECAY;
            if (p->bounce <= 0)
            {
                p->bounce = 0;
                p->dropping = false;
            }
        }
        else
        {
            p->dropping = false;
        }
    }
    return any;
}

/* 按「当前所站台宽的 1/3」算人物等比缩放，下限 JUMP_PLAYER_MIN_W px。
 * 返回 LVGL scale（256=原尺寸）。台子大棋子大、台子小棋子保底不至于看不清。*/
static int player_base_scale(void)
{
    int target_w = g.plats[0].w / 3; /* 目标棋子宽 = 台宽 1/3 */
    if (target_w < JUMP_PLAYER_MIN_W)
        target_w = JUMP_PLAYER_MIN_W;               /* 下限 */
    int scale = target_w * 256 / JUMP_PLAYER_IMG_W; /* 反算 scale */
    if (scale < 16)
        scale = 16;
    return scale;
}

/* ══════════════════════════════════════════════════
 * 小人渲染：人物是图片 j4，用 LVGL scale 缩放。
 *   缩放基准 = 当前台宽 1/3（动态，见 player_base_scale）。
 *   squash_pct 0~100（蓄力/着陆压扁百分比）：X 恒等比、Y 压扁时进一步缩小(scale_y)。
 *   fly_h：抛物线离地高度（向上）。
 *   foot_sink：脚底额外下沉量（蓄力时跟随台子顶面下沉，保持人台相连，不脱离）。
 *   锚点 = 脚底（图片底边中点）对齐 (player_wx, player_wy - fly_h + foot_sink)。
 * ══════════════════════════════════════════════════ */
static void player_render(int squash_pct, int fly_h, int foot_sink)
{
    if (!s_player)
        return;
    if (squash_pct < 0)
        squash_pct = 0;
    if (squash_pct > 100)
        squash_pct = 100;

    /* 动态等比基准（台宽1/3）。满压时纵向压到 base 的 SQUASH_RATIO%，保持原压扁手感。*/
    int base = player_base_scale();
    int scale_x = base;
    /* scale_y = base × (1 - (1-SQUASH_RATIO/100) × pct/100)，线性插值 */
    int scale_y = base - base * (100 - JUMP_PLAYER_SQUASH_RATIO) * squash_pct / 10000;

    /* 缩放后实际高度（用于把脚底锚到地面）*/
    int img_h = JUMP_PLAYER_IMG_H * scale_y / 256;

    int sx = wx_to_sx(g.player_wx);
    int foot_sy = wy_to_sy(g.player_wy) - fly_h + foot_sink; /* 飞行上移、蓄力随台下沉 */
    int img_top = foot_sy - img_h;

    /* LVGL 缩放绕图片中心进行：set_pos 用的是缩放前的左上角，
     * 缩放后图片相对中心扩缩，故左上角需按 (原尺寸-缩放尺寸)/2 反向补偿。*/
    int raw_w = JUMP_PLAYER_IMG_W, raw_h = JUMP_PLAYER_IMG_H;
    int pos_x = sx - raw_w / 2;                /* 中心对齐 player x */
    int pos_y = img_top - (raw_h - img_h) / 2; /* 让缩放后底边落在 img_top+img_h */

    lv_image_set_scale_x(s_player, scale_x);
    lv_image_set_scale_y(s_player, scale_y);
    lv_obj_set_pos(s_player, pos_x, pos_y);
    lv_obj_clear_flag(s_player, LV_OBJ_FLAG_HIDDEN);
}

static void sparks_hide(void); /* 前置声明 */

static void player_hide(void)
{
    if (s_player)
        lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);
    if (s_player_shadow)
        lv_obj_add_flag(s_player_shadow, LV_OBJ_FLAG_HIDDEN);
    sparks_hide();
}

/* ── 光斑粒子 ── */
/* 光斑候选色（明亮）*/
static const uint32_t s_spark_colors[] = {
    0xFFE066,
    0xFF6B6B,
    0x6BE6FF,
    0xB06BFF,
    0x6BFF95,
    0xFFFFFF,
};

static void spark_respawn(int i)
{
    s_spark_st[i].ang = (int)(esp_random() % 360);
    s_spark_st[i].radius = 28 + (int)(esp_random() % 16); /* 外圈 28~43px */
    s_spark_st[i].color =
        s_spark_colors[esp_random() % (sizeof(s_spark_colors) / sizeof(s_spark_colors[0]))];
}

static void sparks_hide(void)
{
    for (int i = 0; i < JUMP_SPARK_COUNT; i++)
        if (s_sparks[i])
            lv_obj_add_flag(s_sparks[i], LV_OBJ_FLAG_HIDDEN);
}

/* 每帧推进光斑（蓄力时调用）。pct=蓄力百分比，sink=脚底下沉量（与 player_render 一致）。
 * 光斑中心跟随棋子「当前真实身体中部」：含动态缩放后的实际高度 + 蓄力下沉，
 * 不再锚在固定空间，故压扁/下沉时光斑跟着棋子走。*/
static void sparks_update(int pct, int sink)
{
    /* 复刻 player_render 的尺寸算法，求棋子当前实际高度与脚底屏幕 y */
    int base = player_base_scale();
    int scale_y = base - base * (100 - JUMP_PLAYER_SQUASH_RATIO) * pct / 10000;
    int img_h = JUMP_PLAYER_IMG_H * scale_y / 256; /* 棋子当前实际高度 */

    int cx = wx_to_sx(g.player_wx);
    int foot_sy = wy_to_sy(g.player_wy) + sink; /* 脚底（含蓄力下沉）*/
    int cy = foot_sy - img_h / 2;               /* 身体中部 = 脚底上移半个身高 */
    int speed = 2 + pct / 25;                   /* 蓄力越满收拢越快（2~6 px/帧）*/

    for (int i = 0; i < JUMP_SPARK_COUNT; i++)
    {
        if (!s_sparks[i])
            continue;
        s_spark_st[i].radius -= speed;
        if (s_spark_st[i].radius <= 3)
            spark_respawn(i); /* 到中心→重生外圈 */

        int r = s_spark_st[i].radius;
        int ang = s_spark_st[i].ang;
        int sx = cx + (lv_trigo_cos(ang) * r >> 15);
        int sy = cy + (lv_trigo_sin(ang) * r >> 15);

        /* 越靠近中心点越小（3→1px）*/
        int sz = r > 24 ? 4 : (r > 12 ? 3 : 2);
        lv_obj_set_size(s_sparks[i], sz, sz);
        lv_obj_set_pos(s_sparks[i], sx - sz / 2, sy - sz / 2);
        lv_obj_set_style_bg_color(s_sparks[i], lv_color_hex(s_spark_st[i].color), 0);
        lv_obj_clear_flag(s_sparks[i], LV_OBJ_FLAG_HIDDEN);
    }
}

/* ══════════════════════════════════════════════════
 * 台子初始化 / 推进
 *
 * 世界坐标：
 *   plats[0] world_cx = JUMP_START_X（初始，cam_x=0 时屏幕坐标相同）
 *   plats[0] world_cy = JUMP_TOP_Y
 *   plats[1] = plats[0] + (rand_gap_x, rand_gap_y)
 *   plats[2] = plats[1] + (rand_gap_x, rand_gap_y)
 *
 * 落台成功后 platforms_advance()：
 *   旧plats[0]的obj复用→挪给新prenext(plats[2])
 *   数据滚动：[0]←[1]←[2]←新生成
 *   cam_target 更新为让新cur台对齐 JUMP_START_X/Y
 * ══════════════════════════════════════════════════ */
/* 让台子进入「从天而降」入场状态（从目标上方 JUMP_DROP_HEIGHT 处掉落）*/
static void plat_start_drop(platform_t *p)
{
    p->drop = JUMP_DROP_HEIGHT;
    p->drop_vy = 0;
    p->bounce = 0;
    p->dropping = true;
}

/* 台子无入场动画（直接就位，如起始台）*/
static void plat_no_drop(platform_t *p)
{
    p->drop = 0;
    p->drop_vy = 0;
    p->bounce = 0;
    p->dropping = false;
}

static void gen_platform(platform_t *dst, const platform_t *ref)
{
    dst->w = cur_platform_w();
    /* 水平间距：随机 gap，但钳到「安全最小中心距」防止台子重叠。
     * 立方体投影宽≈台宽，两台不重叠要求 中心距 ≥ (ref->w + dst->w)/2 + 缝隙。*/
    int gap = rand_gap_x();
    int min_gap = (ref->w + dst->w) / 2 + JUMP_PLAT_MIN_SPACING;
    if (gap < min_gap)
        gap = min_gap;
    dst->world_cx = ref->world_cx + gap;
    dst->world_cy = ref->world_cy + rand_gap_y(ref->world_cy);
    /* 随机一种立体台子样式（避开与参考台同色，区分更明显）*/
    int s = (int)(esp_random() % JUMP_PLAT_STYLE_COUNT);
    if (s == ref->style)
        s = (s + 1) % JUMP_PLAT_STYLE_COUNT;
    dst->style = s;
    /* 非对称外形：厚度在边长的 60%~120% 间随机（出现扁台/高台）*/
    {
        int edge = plat_w_to_edge(dst->w);
        int lo = edge * 60 / 100, hi = edge * 120 / 100;
        dst->depth = lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
    }
    /* 随机亮条带：~50% 概率出现，掩码取低 4 位随机（4 条带各自亮/灭）*/
    dst->stripes = (esp_random() & 1) ? (uint8_t)(esp_random() & 0x0F) : 0;
    /* 随机几何花纹：~60% 概率出现一种（1~CUBE3D_PATTERN_MAX），其余无花纹 */
    dst->pattern = (esp_random() % 10 < 6)
                       ? (uint8_t)(1 + esp_random() % CUBE3D_PATTERN_MAX)
                       : 0;
    dst->active = true;
    plat_start_drop(dst); /* 新台子从天而降入场 */
}

static void platforms_init(void)
{
    g.plats[0].world_cx = JUMP_START_X;
    g.plats[0].world_cy = JUMP_TOP_Y;
    g.plats[0].w = cur_platform_w();
    g.plats[0].style = 0;                            /* 起始台固定第0种样式 */
    g.plats[0].depth = plat_w_to_edge(g.plats[0].w); /* 起始台正常厚度 */
    g.plats[0].stripes = 0;                          /* 起始台无条带，干净 */
    g.plats[0].pattern = 0;                          /* 起始台无花纹，干净 */
    g.plats[0].active = true;
    plat_no_drop(&g.plats[0]); /* 起始台直接就位，不掉落 */

    gen_platform(&g.plats[1], &g.plats[0]);
    gen_platform(&g.plats[2], &g.plats[1]);

    /* 摄像机归零：cur台在 (JUMP_START_X, JUMP_TOP_Y) */
    g.cam_x = g.cam_y = 0;
    g.cam_target_x = g.cam_target_y = 0;
    g.cam_smooth_frames = 0;

    platforms_render_all();
}

static void platforms_advance(void)
{
    /* 旧cur(plats[0])的obj/side将复用为新prenext */
    lv_obj_t *reuse_cube = g.plats[0].cube;

    /* 数据滚动：[0]←[1]←[2] */
    g.plats[0] = g.plats[1];
    g.plats[1] = g.plats[2];

    /* 生成新prenext，复用旧cur的立方体对象 */
    gen_platform(&g.plats[2], &g.plats[1]);
    g.plats[2].cube = reuse_cube;

    /* 摄像机目标：让新cur台对齐屏幕锚点 */
    g.cam_target_x = g.plats[0].world_cx - JUMP_START_X;
    g.cam_target_y = g.plats[0].world_cy - JUMP_TOP_Y;
    g.cam_smooth_frames = JUMP_CAM_SMOOTH_FRAMES;

    platforms_render_all();

    ESP_LOGI(TAG, "advance: cur(%d,%d) next(%d,%d) pre(%d,%d)",
             g.plats[0].world_cx, g.plats[0].world_cy,
             g.plats[1].world_cx, g.plats[1].world_cy,
             g.plats[2].world_cx, g.plats[2].world_cy);
}

/* ══════════════════════════════════════════════════
 * 起跳
 * ══════════════════════════════════════════════════ */
static void do_jump(uint32_t held_ms)
{
    if (g.phase != PH_IDLE)
        return;

    int dist = hold_to_dist(held_ms);

    /* 目标台子世界坐标 */
    int tx = g.plats[1].world_cx;
    int ty = g.plats[1].world_cy;

    /* 方向向量：从当前小人指向下一台中心（决定飞行方向）*/
    int dir_x = tx - g.player_wx;
    int dir_y = ty - g.player_wy;
    int dir_len = isqrt(dir_x * dir_x + dir_y * dir_y);
    if (dir_len < 1)
        dir_len = 1;

    /* 关键修复：飞行的实际位移 = 蓄力距离 dist 沿方向向量投影。
     * 之前 fly_total_x/y 直接用方向向量（=台间距），导致无论蓄力多少，
     * 视觉上都飞到台子中心，再用另一套 actual_wx 判定，二者不一致：
     * 视觉落台→判定落空→小人被瞬移到空白处掉落。
     * 现在视觉位移与判定统一，飞多远取决于蓄力。*/
    g.fly_total_x = dir_x * dist / dir_len;
    g.fly_total_y = dir_y * dist / dir_len;
    g.fly_dist_px = dist;
    g.fly_apex = dist * JUMP_APEX_RATIO / 100;
    g.fly_step = 0;
    g.fly_steps = dist / JUMP_FLY_STEP_PX;
    if (g.fly_steps < 1)
        g.fly_steps = 1;
    g.fly_start_wx = g.player_wx;
    g.fly_start_wy = g.player_wy;
    g.phase = PH_FLY;

    platform_render_one(0, 0); /* 恢复台子压扁 */
    sparks_hide();             /* 起跳：收起蓄力光斑 */
    bsp_motor_pulse();
    ESP_LOGI(TAG, "起跳 dist=%d target(%d,%d)", dist, tx, ty);
}

/* ══════════════════════════════════════════════════
 * 物理引擎（30fps）
 * ══════════════════════════════════════════════════ */
static void engine_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != JS_PLAYING)
        return;

    /* 每帧推进台子「从天而降」入场动画。只重渲染正在掉落的台子，
     * 不碰 slot0（避免覆盖蓄力时的当前台压扁）。独立于 phase。*/
    if (platforms_drop_update())
    {
        for (int i = 0; i < PLAT_COUNT; i++)
            if (g.plats[i].dropping || g.plats[i].drop > 0 || g.plats[i].bounce > 0)
                platform_render_one(i, 0);
    }

    switch (g.phase)
    {
    /* 待蓄力 */
    case PH_IDLE:
    {
        uint32_t held = bsp_touch_page_held_ms();
        if (held == 0)
        {
            player_render(0, 0, 0); /* 不压扁 */
            platform_render_one(0, 0);
            sparks_hide(); /* 未蓄力：无光斑 */
        }
        else
        {
            uint32_t c = held < JUMP_HOLD_MIN_MS ? JUMP_HOLD_MIN_MS : (held > JUMP_HOLD_MAX_MS ? JUMP_HOLD_MAX_MS : held);
            int pct = (int)((c - JUMP_HOLD_MIN_MS) * 100u /
                            (JUMP_HOLD_MAX_MS - JUMP_HOLD_MIN_MS));
            /* 台子顶面下沉量 = 当前台厚度 × 压扁百分比；人物脚底跟着下沉，保持相连 */
            int sink = g.plats[0].depth * pct / 100;
            player_render(pct, 0, sink); /* 蓄力越满压得越扁，且随台下沉 */
            platform_render_one(0, pct); /* 立方体台子同步压扁 */
            sparks_update(pct, sink);    /* 蓄力光斑跟随棋子聚拢 */
        }
        break;
    }

    /* 飞行（二维抛物线，参数化步进）*/
    case PH_FLY:
    {
        g.fly_step++;
        if (g.fly_step > g.fly_steps)
            g.fly_step = g.fly_steps;

        /* 参数 t = 0~1 */
        int num = g.fly_step * 256 / g.fly_steps; /* t*256 定点数 */
        int t256 = num;

        /* 位置：线性插值 */
        g.player_wx = g.fly_start_wx + g.fly_total_x * g.fly_step / g.fly_steps;
        g.player_wy = g.fly_start_wy + g.fly_total_y * g.fly_step / g.fly_steps;

        /* 弧高：h = apex * 4 * t * (1-t) */
        int fly_h = g.fly_apex * 4 * t256 * (256 - t256) / (256 * 256);

        player_render(0, fly_h, 0);

        if (g.fly_step < g.fly_steps)
            break; /* 仍在飞 */

        /* ── 到达终点，判定落点 ── */
        /* 落点判定：小人最终世界x是否落在next台宽度内 */
        int nl = g.plats[1].world_cx - g.plats[1].w / 2;
        int nr = g.plats[1].world_cx + g.plats[1].w / 2;
        /* 视觉飞行终点即判定点（二者已统一，不再单独重算）*/
        int actual_wx = g.player_wx;

        ESP_LOGI(TAG, "判定 actual_wx=%d next[%d,%d]", actual_wx, nl, nr);

        if (actual_wx >= nl && actual_wx <= nr)
        {
            /* 落台成功：小人世界坐标对齐next台 */
            g.player_wx = actual_wx;
            g.player_wy = g.plats[1].world_cy;

            g.score++; /* 每次落台统一 +1，不再有完美居中额外加分 */

            platforms_advance(); /* cam_target 在此更新 */
            /* 落台后小人 x = 实际落点 actual_wx（落哪是哪，不再瞬移到台心）。
             * advance 只滚动台子数据，落点台 world_cx 不变（原plats[1]→现plats[0]），
             * 故 actual_wx 仍在新 plats[0] 台面范围内。y 对齐新当前台顶面。*/
            g.player_wx = actual_wx;
            g.player_wy = g.plats[0].world_cy;
            /* 保险：玩家落上的台子强制结束入场动画，确保脚下台子绝不压扁/回弹
             * （即使该台进场动画恰好没播完，落上去也立即归位）*/
            plat_no_drop(&g.plats[0]);

            hud_refresh();
            /* 落台后不做着陆压扁（压扁只在蓄力时）：直接进摄像机平滑阶段，人物保持正常。*/
            g.phase = PH_CAM;
            player_render(0, 0, 0);
            ESP_LOGI(TAG, "落台成功 score=%d", g.score);
        }
        else
        {
            /* 落空：从飞行末态平滑接力掉落（连续抛物线，不再瞬间垂直撞墙）。
             * - 水平速度 fall_vx = 飞行每帧的水平位移（继续往前飞）
             * - 弧高余量 fall_h = 飞行最后一帧的离地高度（被重力逐帧吃掉）
             * - 垂直速度 fall_vy 从 0 起重力加速 */
            g.player_wx = actual_wx;
            g.fall_vx = g.fly_total_x / g.fly_steps;                       /* 飞行平均每帧水平位移 */
            g.fall_h = g.fly_apex * 4 * t256 * (256 - t256) / (256 * 256); /* 末帧弧高 */
            g.fall_vy = 0;
            g.phase = PH_FALL;
            highscore_save_if_better(g.score);
            ESP_LOGI(TAG, "落空 wx=%d vx=%d h=%d", g.player_wx, g.fall_vx, g.fall_h);
        }
        break;
    }

    /* 摄像机平滑滑动 */
    case PH_CAM:
        if (g.cam_smooth_frames > 0)
        {
            /* 每帧向目标前进 1/remaining 步（匀减速感）*/
            g.cam_x += (g.cam_target_x - g.cam_x + g.cam_smooth_frames - 1) / g.cam_smooth_frames;
            g.cam_y += (g.cam_target_y - g.cam_y + g.cam_smooth_frames - 1) / g.cam_smooth_frames;
            g.cam_smooth_frames--;
            platforms_render_all();
            player_render(0, 0, 0);
        }
        if (g.cam_smooth_frames <= 0)
        {
            /* 对齐到精确目标，消除累积误差 */
            g.cam_x = g.cam_target_x;
            g.cam_y = g.cam_target_y;
            platforms_render_all();
            player_render(0, 0, 0);
            g.phase = PH_IDLE;
        }
        break;

    /* 掉落出屏（连续抛物线：水平继续飞 + 弧高消退 + 重力加速下坠）*/
    case PH_FALL:
    {
        /* 水平继续往前（衰减一点，模拟空气阻力，避免飞太远）*/
        g.player_wx += g.fall_vx;
        /* 弧高先把残余顶起的高度吃掉，吃完才开始真正下坠 */
        int h;
        if (g.fall_h > 0)
        {
            g.fall_h -= 3; /* 弧高逐帧消退 */
            if (g.fall_h < 0)
                g.fall_h = 0;
            h = g.fall_h; /* 仍被弧高顶着，player_wy 不动 */
        }
        else
        {
            g.fall_vy += 3;           /* 重力加速 */
            g.player_wy += g.fall_vy; /* 真正下坠 */
            h = 0;
        }
        player_render(0, h, 0); /* h 作为离地高度，保持弧线连续 */
        if (wy_to_sy(g.player_wy) >= BSP_LCD_HEIGHT + JUMP_PLAYER_H * 2)
            enter_result();
        break;
    }

    default:
        break;
    }
}

/* ══════════════════════════════════════════════════
 * 子界面
 * ══════════════════════════════════════════════════ */
static void select_render(void)
{
    if (!s_center)
        return;
    char buf[96];
    snprintf(buf, sizeof(buf), "选择难度\n%s简单\n%s一般\n%s困难",
             g.diff == DIFF_EASY ? ">" : " ",
             g.diff == DIFF_NORMAL ? ">" : " ",
             g.diff == DIFF_HARD ? ">" : " ");
    lv_label_set_text(s_center, buf);
    lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
}

static void enter_select(void)
{
    g.screen = JS_SELECT;
    if (s_engine_tmr)
        lv_timer_pause(s_engine_tmr);
    player_hide();
    for (int i = 0; i < PLAT_COUNT; i++)
    {
        g.plats[i].active = false;
        if (g.plats[i].cube)
            lv_obj_add_flag(g.plats[i].cube, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    select_render();
}

static void enter_playing(void)
{
    g.screen = JS_PLAYING;
    g.phase = PH_IDLE;
    g.score = 0;
    g.high_score = highscore_load(); /* 高分三难度共享 */

    platforms_init();
    g.player_wx = g.plats[0].world_cx;
    g.player_wy = g.plats[0].world_cy;

    if (s_center)
        lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    if (s_hud)
        lv_obj_clear_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
    hud_refresh();
    player_render(0, 0, 0);
    if (s_engine_tmr)
        lv_timer_resume(s_engine_tmr);
    ESP_LOGI(TAG, "开跳 难度=%s", s_diff[g.diff].name);
}

static void enter_result(void)
{
    g.screen = JS_RESULT;
    if (s_engine_tmr)
        lv_timer_pause(s_engine_tmr);
    player_hide();
    for (int i = 0; i < PLAT_COUNT; i++)
    {
        g.plats[i].active = false;
        if (g.plats[i].cube)
            lv_obj_add_flag(g.plats[i].cube, LV_OBJ_FLAG_HIDDEN);
    }
    /* 先记下「打破前」的历史最高分（save_if_better 会就地更新 g.high_score）*/
    int prev_high = g.high_score;
    bool refreshed = highscore_save_if_better(g.score);
    int shown_high = refreshed ? g.score : prev_high; /* 当前应展示的最高分 */
    if (s_center)
    {
        char buf[96];
        /* 注意：font_cn_32 是裁剪字库，「游戏结束/打」等字不在其中（会显示豆腐块）。
         * 只用字库确含的字：得 分 最 高 破 纪 录 新 数 等。*/
        if (refreshed)
        {
            /* 超过 NVS 记录：得分 + 破纪录 + 新最高分 */
            snprintf(buf, sizeof(buf),
                     "得分 %d\n破纪录\n历史最高 %d",
                     g.score, shown_high);
            ESP_LOGI(TAG, "破纪录！得分=%d（旧纪录 %d）", g.score, prev_high);
        }
        else
        {
            /* 未破纪录：得分 + 历史最高分 */
            snprintf(buf, sizeof(buf),
                     "得分 %d\n历史最高 %d",
                     g.score, shown_high);
        }
        lv_label_set_text(s_center, buf);
        lv_obj_clear_flag(s_center, LV_OBJ_FLAG_HIDDEN);
    }
    if (s_hud)
        lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);
}

/* ══════════════════════════════════════════════════
 * 面板构建（一次性）
 * ══════════════════════════════════════════════════ */
static void build_panel(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (!scr)
    {
        ESP_LOGE(TAG, "screen NULL");
        return;
    }

    s_panel = lv_obj_create(scr);
    lv_obj_set_size(s_panel, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_align(s_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_opa(s_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_bg_color(s_panel, lv_color_hex(JUMP_BG_COLOR), 0);
    lv_obj_set_style_border_width(s_panel, 0, 0);
    lv_obj_set_style_pad_all(s_panel, 0, 0);
    lv_obj_set_style_radius(s_panel, 0, 0);
    lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scrollbar_mode(s_panel, LV_SCROLLBAR_MODE_OFF);

#if JUMP_BG_USE_IMG
    {
        lv_obj_t *bg = lv_image_create(s_panel);
        lv_image_set_src(bg, &JUMP_BG_IMAGE);
        lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
        lv_obj_move_background(bg);
    }
#endif

    /* 三个等轴测立方体台子（cube3d 控件，先创建=在小人之下）。*/
    for (int i = 0; i < PLAT_COUNT; i++)
    {
        lv_obj_t *cube = cube3d_create(s_panel);
        cube3d_set_geometry(cube, plat_w_to_edge(s_diff[DIFF_EASY].pw));
        cube3d_set_top_color(cube, s_plat_styles[0]);
        lv_obj_add_flag(cube, LV_OBJ_FLAG_HIDDEN);

        g.plats[i].cube = cube;
        g.plats[i].style = 0;
        g.plats[i].active = false;
    }

    /* （人物影子已按用户反馈去掉，s_player_shadow 保持 NULL，相关代码均 NULL 守卫）*/

    /* 小人：图片 j1（带 alpha），最后创建=在最上层。等比缩放在 player_render 里按需设置。*/
    s_player = lv_image_create(s_panel);
    lv_image_set_src(s_player, &JUMP_PLAYER_IMG);
    lv_image_set_antialias(s_player, true);
    lv_obj_set_style_image_recolor(s_player, lv_color_hex(0x000000), 0);
    lv_obj_set_style_image_recolor_opa(s_player, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_player, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_player, LV_OBJ_FLAG_HIDDEN);

    /* 蓄力光斑粒子池（最上层，覆盖人物）*/
    for (int i = 0; i < JUMP_SPARK_COUNT; i++)
    {
        lv_obj_t *sp = lv_obj_create(s_panel);
        lv_obj_remove_style_all(sp);
        lv_obj_set_style_radius(sp, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_opa(sp, LV_OPA_COVER, 0);
        lv_obj_clear_flag(sp, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(sp, LV_OBJ_FLAG_HIDDEN);
        s_sparks[i] = sp;
        spark_respawn(i);
    }

    /* HUD */
    s_hud = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_hud, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_hud, lv_color_hex(0x2C3E50), 0);
    lv_obj_align(s_hud, LV_ALIGN_TOP_MID, 0, 8);
    lv_obj_add_flag(s_hud, LV_OBJ_FLAG_HIDDEN);

    /* 居中大文字 */
    s_center = lv_label_create(s_panel);
    lv_obj_set_style_text_font(s_center, &font_cn_32, 0);
    lv_obj_set_style_text_color(s_center, lv_color_hex(0x2C3E50), 0);
    lv_obj_set_style_text_align(s_center, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_center, LV_ALIGN_CENTER, 0, -10);
    lv_obj_add_flag(s_center, LV_OBJ_FLAG_HIDDEN);
}

/* ══════════════════════════════════════════════════
 * 对外接口
 * ══════════════════════════════════════════════════ */
void jump_start(void)
{
    if (!lvgl_port_lock(200))
    {
        ESP_LOGW(TAG, "取锁失败");
        return;
    }
    if (!s_panel)
        build_panel();
    else
        lv_obj_clear_flag(s_panel, LV_OBJ_FLAG_HIDDEN);
    if (!s_engine_tmr)
        s_engine_tmr = lv_timer_create(engine_cb, JUMP_ENGINE_MS, NULL);
    g.diff = DIFF_EASY;
    enter_select();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "跳一跳启动");
}

void jump_touch(touch_event_t event)
{
    if (!lvgl_port_lock(100))
        return;
    switch (g.screen)
    {
    case JS_SELECT:
        if (event == TOUCH_EVENT_SHORT_PREV_PAGE)
            g.diff = g.diff == DIFF_EASY ? DIFF_HARD : (jump_diff_t)(g.diff - 1);
        else if (event == TOUCH_EVENT_SHORT_NEXT_PAGE)
            g.diff = g.diff == DIFF_HARD ? DIFF_EASY : (jump_diff_t)(g.diff + 1);
        else if (event == TOUCH_EVENT_SHORT_HEAD)
            enter_playing();
        if (g.screen == JS_SELECT)
            select_render();
        break;
    case JS_PLAYING:
        if (g.phase == PH_IDLE &&
            (event == TOUCH_EVENT_SHORT_PREV_PAGE ||
             event == TOUCH_EVENT_SHORT_NEXT_PAGE))
            do_jump(bsp_touch_last_page_hold_ms());
        break;
    case JS_RESULT:
        if (event == TOUCH_EVENT_SHORT_HEAD)
            enter_select();
        break;
    default:
        break;
    }
    lvgl_port_unlock();
}

void jump_stop(void)
{
    if (!lvgl_port_lock(200))
        return;
    if (!s_panel)
    {
        lvgl_port_unlock();
        return;
    }
    if (s_engine_tmr)
    {
        lv_timer_del(s_engine_tmr);
        s_engine_tmr = NULL;
    }
    lv_obj_del(s_panel);
    s_panel = s_player = s_player_shadow = s_hud = s_center = NULL;
    for (int i = 0; i < JUMP_SPARK_COUNT; i++)
        s_sparks[i] = NULL;
    for (int i = 0; i < PLAT_COUNT; i++)
    {
        g.plats[i].cube = NULL;
        g.plats[i].active = false;
    }
    lvgl_port_unlock();
    ESP_LOGI(TAG, "跳一跳退出");
}
