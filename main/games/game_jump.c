/**
 * @file game_jump.c
 * @brief 跳一跳游戏实现
 *
 * 坐标系：
 *   - 台子使用「世界坐标」(world_cx, world_cy)，不受摄像机影响
 *   - 摄像机 (cam_x, cam_y)，屏幕坐标 = 世界坐标 - 摄像机偏移
 *   - 屏幕中心锚点：小人始终在 (JUMP_START_X, JUMP_START_Y) 附近
 *
 * 台子池：5个槽位 [0]=cur [1]=next [2]=prenext [3]=prev [4]=prevprev
 *   每次落台后：[4]←[3]←旧[0]，[0]←[1]←[2]，[2]生成新台
 *   旧cur不立即回收：先入prev槽随相机滑出屏幕，滚到prevprev出屏后才把obj复用给新[2]
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
    PH_FALL,
    PH_TOPPLE /* 踩到台子边缘但落脚面不够 → 先站稳一瞬再朝台外倾倒掉落（失败）*/
} jump_phase_t;

/* ── 台子（世界坐标，双轴）── */
/* 5 槽位：游戏槽 cur(0)+next(1)+prenext(2)，离场槽 prev(3)+prevprev(4)。
 * 离场槽保留刚跳离的台子对象，使其随相机平移自然滑出屏幕（不再瞬间消失）；
 * 待其滚到 prevprev(4) 已彻底出屏后，下一次 advance 再回收该对象给新 prenext。*/
#define PLAT_COUNT 5 /* cur(0) next(1) prenext(2) prev(3) prevprev(4) */

typedef struct
{
    lv_obj_t *cube;  /* JUMP_PLAT_USE_IMG=0 时是 cube3d 控件；=1 时复用为台子图片(lv_image)句柄 */
    int world_cx;    /* 世界坐标中心 x */
    int world_cy;    /* 世界坐标中心 y（台面顶边）*/
    int w;           /* 台子宽度（图片模式=缩放目标宽；cube 模式=立方体边长基准）*/
    int style;       /* 样式索引（指向 s_plat_styles[]，决定顶面色，仅 cube 模式）*/
    int depth;       /* 立方体竖直厚度（随机，做扁/高台非对称外形，仅 cube 模式）*/
    uint8_t stripes; /* 侧面亮条带掩码（随机，仅 cube 模式）*/
    uint8_t pattern; /* 顶面/正面几何花纹类型（随机，0=无，仅 cube 模式）*/
    uint8_t img_idx; /* 台子图片索引 0~JUMP_PLAT_IMG_COUNT-1（仅 JUMP_PLAT_USE_IMG=1）*/
    bool active;
    /* ── 从天而降入场动画 ── */
    int drop;          /* 当前竖直入场偏移（>0=在目标上方，渲染时 sy 减去它）*/
    int drop_vy;       /* 掉落速度（每帧px，向下加速）*/
    int bounce;        /* 触底回弹压扁余量（0~100，渲染时加到 squash）*/
    bool dropping;     /* 入场动画进行中 */
    bool drop_pending; /* 已生成但尚未起跌：台子多生成在屏幕右侧外，先挂起，
                        * 等它随相机滚入视野右缘再触发 plat_start_drop，
                        * 玩家才能亲眼看到从天而降，而非屏外掉完才滑进来 */
    /* ── 棋子落台「压扁→回弹原高度」动画（JUMP_LAND_USE_SQUASH=1 时启用）──
     * 落台时台子顶面压扁下沉（底部锁死），棋子脚底跟随，随后回弹到 land_sq=0（台面原高度）。
     * 与整体下坠 bounce（整体下移 px）并存，由宏 JUMP_LAND_USE_SQUASH 二选一，旧逻辑完整保留。*/
    int land_sq;         /* 当前落台压扁百分比 0~JUMP_LAND_SQUASH_PCT */
    uint8_t land_phase;  /* 0=压扁上升阶段 1=回弹下降阶段 */
    bool land_squashing; /* 落台压扁回弹动画进行中 */
    /* ── 蓄力压扁台面下沉量（由 platform_render_one 写入，供棋子 sink 读取）── */
    int squash_face_sink; /* 台面因压扁下沉的像素数（正值=下沉），图片模式专用 */
} platform_t;

#if !JUMP_PLAT_USE_IMG
/* ── 立体台子样式表（顶面色；正面/右侧面由 cube3d 自动调暗生成）──
 * 后期“五六种不同大小/颜色”的台子，往这里加条目即可，渲染逻辑无需改动。
 * 大小由 cur_platform_w() 给宽度，这里只管颜色风格。
 * 仅 cube3d 自画模式用；图片台子模式(JUMP_PLAT_USE_IMG=1)不编译此表，避免 unused 报错。*/
static const uint32_t s_plat_styles[JUMP_PLAT_STYLE_COUNT] = {
    0x8FBF8F, /* 灰绿（参考素材色）*/
    0x8FAFD4, /* 蓝灰 */
    0xD4B98F, /* 暖棕 */
    0xB98FD4, /* 紫 */
    0xD48F9F, /* 暖红 */
};
#endif

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
    /* 飞行位置定点累积（×256），避免每帧重新整除导致 ±1px 截断抖动 */
    int fly_wx256;     /* player_wx 的定点值（实际 wx = fly_wx256 >> 8）*/
    int fly_wy256;     /* player_wy 的定点值 */
    int fly_step256_x; /* 每帧 wx 步进量（×256）*/
    int fly_step256_y; /* 每帧 wy 步进量（×256）*/

    /* 掉落（落空时继承飞行末态，做连续抛物线，而非垂直撞墙）*/
    int fall_vy; /* 垂直掉落速度（每帧px，向下加速）*/
    int fall_vx; /* 水平速度（继承飞行末尾的每帧水平位移，继续往前飞）*/
    int fall_h;  /* 当前离地弧高余量（从飞行末尾 fly_h 继承，被重力逐帧吃掉）*/

    /* ── 摔倒（PH_TOPPLE）：踩到台缘但落脚面不够，先站稳一瞬再朝台外倾倒掉落 ── */
    int topple_dir;      /* 倾倒方向：+1=向右倒（落点偏台右），-1=向左倒 */
    int topple_ang;      /* 当前倾倒角度（0.1°为单位×10，即 0~JUMP_TOPPLE_MAX_ANG）*/
    int topple_hold;     /* 站稳停顿剩余帧数（>0 时人物不动，仅展示「站上去了」）*/
    int topple_pivot_wx; /* 倾倒支点（脚底）世界 x，固定在台缘处，旋转绕此点 */

    /* 摄像机延迟：落台后先等 N 帧，避免视线在棋子落台瞬间就移走 */
    int cam_delay_frames;

    /* ── 随机台子变体调度（锁死出现频率，见 jump_sprites.h JUMP_VARIANT_*）── */
    int plat_gen_seq;       /* 已生成台子计数（含初始预生成），用于热身判定 */
    int variant_gap_normal; /* 距上一张变体台已经过的「正常台」数量（冷却计数）*/
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
static void trigger_next_drop(void); /* 触发下一块目标台从天而降（开局/镜头停稳后调用）*/
static int player_base_scale(void);  /* 棋子等比缩放基准，plat_foot_half_w 提前用到 */

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
    /* 台宽固定为当前难度基础宽，不再随分数缩小。
     * 「小台子」变化由 gen_platform 的随机变体逻辑负责，且受冷却频率锁死。*/
    return s_diff[g.diff].pw;
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

/* 台子「可站立顶面」的半宽（世界坐标）——落台判定用此，而非整图半宽。
 * p->w 是整张台子图缩放后的宽度；真正能落脚的只有顶面那块菱形/方块，
 * 占整图宽的 JUMP_PLAT_IMG_FACE_W_PCT_TABLE[img_idx]%。
 * 图片模式按该表收窄；cube 模式顶面≈整宽，直接用半宽。*/
static int plat_face_half_w(const platform_t *p)
{
#if JUMP_PLAT_USE_IMG
    static const int pct_tbl[JUMP_PLAT_IMG_COUNT] = JUMP_PLAT_IMG_FACE_W_PCT_TABLE;
    int idx = p->img_idx;
    if (idx < 0 || idx >= JUMP_PLAT_IMG_COUNT)
        idx = 0;
    int face_w = p->w * pct_tbl[idx] / 100;
    return face_w / 2;
#else
    return p->w / 2;
#endif
}

/* 棋子「有效落脚半宽」（世界坐标）——摔倒判定用。
 * = 当前棋子缩放后图片半宽 × JUMP_FOOT_W_PCT%。棋子脚不是整张图那么宽，
 * 故按落脚比例收窄，作为「重心是否还压在台面上」的判定半径。*/
static int plat_foot_half_w(void)
{
    int scale = player_base_scale();                /* 256=原尺寸 */
    int player_w = JUMP_PLAYER_IMG_W * scale / 256; /* 缩放后图片宽 */
    int foot_w = player_w * JUMP_FOOT_W_PCT / 100;  /* 有效落脚宽 */
    return foot_w / 2;
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
    /* 「挂起入场」台子（drop_pending=true）尚未被触发从天而降：保持隐藏、完全不可见。
     * 否则其 drop=0 会被渲染成「落定态」在屏内闪现，并与落台动画同帧浮动（见修复计划）。
     * 待 trigger_next_drop 把它转为 dropping=true 后，drop_pending 清零，本判断自然放行。*/
    if (p->drop_pending)
    {
        if (p->cube)
            lv_obj_add_flag(p->cube, LV_OBJ_FLAG_HIDDEN);
        return;
    }
    /* 顶面中心对齐屏幕 (sx, sy)（小人脚踩顶面）。入场掉落时整体上移 drop。*/
    int sx = wx_to_sx(p->world_cx);
    int sy = wy_to_sy(p->world_cy) - p->drop; /* drop>0=在目标上方，往下落 */

#if JUMP_PLAT_USE_IMG
    /* ── 图片台子：等比缩放 x + 蓄力压扁 y + 底部固定 + 各图独立台面偏移 ──
     *
     * 蓄力压扁：scale_y < scale_x，台子纵向压扁，底部固定（顶面向下移）。
     * LVGL scale 以图片中心为锚，压扁后底边会上移 (sh_normal-sh)/2，
     * 通过 bottom_fix 向下补偿，实现底部固定效果。
     * 台面随顶面下沉 face_sink px，棋子脚底跟随同量下沉（squeeze = 从上往下压）。
     *
     * 各图台面偏移：每张台子图形状不同，台面 Y 在 face_y_table[] 独立配置。
     * LVGL scale 以图片中心为锚，set_pos 是缩放前左上角：
     *   target_face_sy = pos_y + raw_h/2 - sh/2 + face_in_scaled
     *   → pos_y = target_face_sy - raw_h/2 + sh/2 - face_in_scaled
     */
    static const lv_image_dsc_t *const imgs[JUMP_PLAT_IMG_COUNT] = {
        &jt1, &jt3, &jt4, &jt5, &jt6, &jt7};
    /* 每张台子图台面顶边在原图中的 Y 偏移（各图形状高度不同，须独立配置）*/
    static const int face_y_table[JUMP_PLAT_IMG_COUNT] = JUMP_PLAT_IMG_FACE_Y_TABLE;
    int pic = p->img_idx;
    if (pic < 0 || pic >= JUMP_PLAT_IMG_COUNT)
        pic = 0;
    lv_image_set_src(p->cube, imgs[pic]);

    /* 等比缩放：宽度对齐台宽，高度同比 */
    int sx_scale = p->w * 256 / JUMP_PLAT_IMG_W;
    if (sx_scale < 16)
        sx_scale = 16;

    /* 蓄力时台子竖向压扁，与棋子同比映射（squash_pct 0~100）*/
    int sy_scale = sx_scale;
    if (squash_pct > 0)
    {
        int sq_ratio = 100 - (int)((100 - JUMP_PLAYER_SQUASH_RATIO) * squash_pct / 100);
        sy_scale = sx_scale * sq_ratio / 100;
        if (sy_scale < 16)
            sy_scale = 16;
    }
    lv_image_set_scale_x(p->cube, sx_scale);
    lv_image_set_scale_y(p->cube, sy_scale);

    int raw_w = JUMP_PLAT_IMG_W, raw_h = JUMP_PLAT_IMG_H;
    int sh_full = raw_h * sx_scale / 256; /* 无压扁时缩放高度 */
    int sh = raw_h * sy_scale / 256;      /* 压扁后缩放高度 */

    /* 各图台面偏移（按当前压扁缩放算）*/
    int face_in = face_y_table[pic] * sy_scale / 256;
    int face_in_full = face_y_table[pic] * sx_scale / 256;
    int bounce_offset = p->bounce;

    /* 底部固定定位 + 台面顶边对齐世界 y(=棋子脚底)：
     *
     * 关键修复：world_cy 是「台面顶边」的世界 y，棋子脚底也渲染在 wy_to_sy(world_cy)=sy。
     * LVGL scale 绕图片中心进行，缩放后台面顶边（距图片中心 sh_full/2 - face_in_full 处）
     * 落在 pos_y + raw_h/2 - sh_full/2 + face_in_full。
     * 旧公式 pos_y = sy - face_in_full 让台面顶边落在 sy + (raw_h - sh_full)/2，
     * 即台面比脚底低 (raw_h - sh_full)/2 px —— 台子越缩小该值越大，棋子整体悬空在台面上方。
     * 现在把基准 pos_y 补偿到「未压扁时台面顶边 == sy」：
     *   pos_y0 = sy - raw_h/2 + sh_full/2 - face_in_full
     * 再叠加底部固定的压扁补偿 (sh_full - sh)/2（顶面随蓄力下压、底边不动）。
     * 验证(未压扁 sh=sh_full)：顶边 = pos_y + raw_h/2 - sh_full/2 + face_in_full = sy ✓ */
    int pos_x = sx - raw_w / 2;
    int pos_y = (sy + bounce_offset) - raw_h / 2 + sh_full / 2 - face_in_full + (sh_full - sh) / 2;
    lv_obj_set_pos(p->cube, pos_x, pos_y);

    /* 台面实际下沉量 = (sh_full-sh) + (face_in - face_in_full)，棋子跟随同量下沉 */
    p->squash_face_sink = (sh_full - sh) + (face_in - face_in_full);
#else
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
    cube3d_place(p->cube, sx, sy);
#endif

    lv_obj_clear_flag(p->cube, LV_OBJ_FLAG_HIDDEN);
}

static void platforms_render_all(void)
{
    for (int i = 0; i < PLAT_COUNT; i++)
        platform_render_one(i, 0);
}

#if 1 /* ── 启用：图片模式向上弹起版本 ── */
/* 每帧推进「从天而降」入场动画。
 * 图片模式：下落→触底→向上弹起一次→落定（物理弹跳，无下坠/压扁）。
 *   drop>0=台子在目标上方；触底时 drop_vy 反向×弹力系数使台子弹起；
 *   drop 回升到0时落定（弹力太小则直接落定，避免无限微颤）。
 * cube模式：下落→触底→压扁回弹消退（原有逻辑）。
 * 返回是否有任一台子在动画中。*/
static bool platforms_drop_update(void)
{
    bool any = false;
    for (int i = 0; i < PLAT_COUNT; i++)
    {
        platform_t *p = &g.plats[i];
        if (!p->active || !p->dropping)
            continue;
        any = true;

#if JUMP_PLAT_USE_IMG
        /* 重力始终向下：vy 每帧增大，drop 每帧减去 vy（drop 减小=台子向下落）*/
        p->drop_vy += JUMP_DROP_GRAVITY;
        p->drop -= p->drop_vy;

        if (p->drop <= 0 && p->drop_vy > 0)
        {
            /* 触底（drop穿过0且速度向下）：反弹，速度反向并衰减 */
            p->drop = 0;
            p->drop_vy = -(p->drop_vy * JUMP_DROP_BOUNCE_COEF / 100);
            /* 弹力不足以弹起（<3px/帧）则直接落定，避免无限微颤 */
            if (p->drop_vy > -3)
            {
                p->drop_vy = 0;
                p->dropping = false;
            }
        }
        else if (p->drop <= 0 && p->drop_vy <= 0)
        {
            /* 弹起后回落到0：落定 */
            p->drop = 0;
            p->drop_vy = 0;
            p->dropping = false;
        }
#else
        if (p->drop > 0)
        {
            /* cube模式下落：重力加速 */
            p->drop_vy += JUMP_DROP_GRAVITY;
            p->drop -= p->drop_vy;
            if (p->drop <= 0)
            {
                p->drop = 0;
                p->bounce = JUMP_DROP_BOUNCE;
            }
        }
        else if (p->bounce > 0)
        {
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
#endif
    }
    return any;
}
#endif /* ── 注释段结束 ── */

#if 0 /* ── 已注释：更改之前的原始版本（掉落→触底→回弹压扁消退），暂时停用 ── */
/* 返回是否有任一台子在动画中（用于决定是否需要重渲染）。*/
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
#if JUMP_PLAT_USE_IMG
                /* 图片模式 bounce = 下移 px；入场触底抖动封顶 JUMP_LAND_BOUNCE（轻弹）*/
                if (p->bounce > JUMP_LAND_BOUNCE)
                    p->bounce = JUMP_LAND_BOUNCE;
#endif
            }
        }
        else if (p->bounce > 0)
        {
            /* 回弹：压扁量逐帧衰减到 0 */
#if JUMP_PLAT_USE_IMG
            p->bounce -= JUMP_LAND_BOUNCE_DECAY; /* 图片台子用更柔的衰减（落台/入场统一）*/
#else
            p->bounce -= JUMP_DROP_BOUNCE_DECAY;
#endif
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
#endif /* ── 原始版本注释段结束 ── */

/* 按「难度基础台宽的 2/5」算人物等比缩放，下限 JUMP_PLAYER_MIN_W px。
 * 返回 LVGL scale（256=原尺寸）。基准用固定基础台宽 s_diff[].pw（而非当前台实际宽
 * g.plats[0].w），故棋子尺寸恒定，不再随小台子/变体台一起缩小放大。*/
static int player_base_scale(void)
{
    int target_w = s_diff[g.diff].pw * 2 / 5; /* 目标棋子宽 = 基础台宽 2/5（恒定）*/
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

    lv_image_set_rotation(s_player, 0);                 /* 清除摔倒残留旋转，正常态恒为竖直 */
    lv_image_set_pivot(s_player, raw_w / 2, raw_h / 2); /* 复位支点到图片中心：正常缩放公式按绕中心算，
                                                          摔倒后 pivot 残留在脚底会导致缩放后整体偏位 */
    lv_image_set_scale_x(s_player, scale_x);
    lv_image_set_scale_y(s_player, scale_y);
    lv_obj_set_pos(s_player, pos_x, pos_y);
    lv_obj_clear_flag(s_player, LV_OBJ_FLAG_HIDDEN);
}

/* 摔倒专用渲染：棋子绕「脚底」旋转 ang_deci（0.1°单位），向 dir（+1右/-1左）倾倒，
 * 同时整体随 fall_h 下坠。与 player_render 区别仅在：设置 pivot 到脚底 + set_rotation。
 * 旋转方向：dir>0 顺时针（图像向右倒），LVGL rotation 正值=顺时针（0.1°单位）。*/
static void player_render_topple(int ang_deci, int dir, int fall_h)
{
    if (!s_player)
        return;
    int base = player_base_scale();
    int img_h = JUMP_PLAYER_IMG_H * base / 256;

    int sx = wx_to_sx(g.topple_pivot_wx);         /* 支点=脚底所在台缘 x（固定不动）*/
    int foot_sy = wy_to_sy(g.player_wy) - fall_h; /* 脚底屏幕 y，下坠时随 player_wy 走 */
    int img_top = foot_sy - img_h;

    int raw_w = JUMP_PLAYER_IMG_W, raw_h = JUMP_PLAYER_IMG_H;
    int pos_x = sx - raw_w / 2;
    int pos_y = img_top - (raw_h - img_h) / 2;

    /* pivot 用「缩放前原图坐标」：脚底中点 = (raw_w/2, raw_h)。
     * LVGL 绕此点做缩放+旋转，故脚底锚定不动，头部向外倒。*/
    lv_image_set_pivot(s_player, raw_w / 2, raw_h);
    lv_image_set_scale_x(s_player, base);
    lv_image_set_scale_y(s_player, base);
    lv_image_set_rotation(s_player, dir > 0 ? ang_deci : -ang_deci);
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
    p->drop_vy = 0; /* 给初速让台子一开始就快速下落 */
    p->bounce = 0;
    p->dropping = true;
    p->drop_pending = false;
}

/* 台子无入场动画（直接就位，如起始台）*/
static void plat_no_drop(platform_t *p)
{
    p->drop = 0;
    p->drop_vy = 0;
    p->bounce = 0;
    p->dropping = false;
    p->drop_pending = false;
}

/* 台子「挂起入场」：生成后先不掉落，标记 drop_pending=true。
 * 待它滚动成「下一块目标台」(plats[1]) 且镜头停稳后，由 trigger_next_drop 触发从天而降。
 * 挂起期间 drop_pending=true：platform_render_one 强制隐藏，完全不可见（不再屏内闪现）。*/
static void plat_pend_drop(platform_t *p)
{
    p->drop = 0;
    p->drop_vy = JUMP_DROP_INIT_VY; // 掉落初速度
    p->bounce = 0;
    p->dropping = false;
    p->drop_pending = true;
}

#if JUMP_PLAT_USE_IMG
#if JUMP_LAND_USE_SQUASH
/* 棋子落台（压扁回弹版）：触发台子「顶面压扁→回弹到原高度」动画（底部锁死）。
 * 由 PH_CAM 每帧 land_squash_step / land_squash_render_slot0 推进，
 * 压扁到峰值后回弹到 land_sq=0，台子恢复台面原高度。*/
static void plat_land_squash(platform_t *p)
{
    p->land_sq = 0;
    p->land_phase = 0;
    p->land_squashing = true;
}

/* 推进 slot0 落台压扁回弹状态机一帧（仅更新状态，不渲染）。
 * 阶段0：land_sq 升到 JUMP_LAND_SQUASH_PCT（压扁）；阶段1：降回 0（回弹到原高度）。*/
static void land_squash_step(platform_t *p)
{
    if (!p->land_squashing)
        return;
    if (p->land_phase == 0)
    {
        p->land_sq += JUMP_LAND_SQUASH_RISE;
        if (p->land_sq >= JUMP_LAND_SQUASH_PCT)
        {
            p->land_sq = JUMP_LAND_SQUASH_PCT;
            p->land_phase = 1;
        }
    }
    else
    {
        p->land_sq -= JUMP_LAND_SQUASH_FALL;
        if (p->land_sq <= 0)
        {
            p->land_sq = 0;
            p->land_squashing = false;
        }
    }
}

/* 按当前 land_sq 渲染 slot0 台子（顶面压扁、底部锁死），
 * 返回棋子脚底应跟随的顶面下沉量 px（land_sq=0 时返回 0，台面原高度）。*/
static int land_squash_render_slot0(void)
{
    platform_render_one(0, g.plats[0].land_sq); /* 写入 squash_face_sink */
    return g.plats[0].land_sq > 0 ? g.plats[0].squash_face_sink : 0;
}
#else
/* 棋子落台（整体下坠版，旧逻辑）：给这张台注入一次「被踩下压」，
 * 随后由 platforms_drop_update 逐帧回弹。复用 bounce 通道（drop=0 直接进回弹阶段），
 * 衰减用 JUMP_LAND_BOUNCE_DECAY。*/
static void plat_land_bounce(platform_t *p)
{
    p->drop = 0;
    p->drop_vy = 0;
    p->bounce = JUMP_LAND_BOUNCE; /* 图片模式：下移 px 数；棋子同步偏移 */
    p->dropping = true;           /* 标记动画中，使 platforms_drop_update 处理其回弹 */
}
#endif
#endif

static void gen_platform(platform_t *dst, const platform_t *ref)
{
    dst->w = cur_platform_w();
    int gap = rand_gap_x();

    /* ── 随机台子变体调度（小台子 / 远台子，频率被严格锁死）──
     * 1) 热身：前 JUMP_VARIANT_WARMUP 张一律正常台。
     * 2) 冷却：距上一张变体台不足 JUMP_VARIANT_COOLDOWN 张正常台时绝不出变体。
     * 3) 触发：冷却满足后按 JUMP_VARIANT_TRIGGER_PCT 概率出变体。
     * 4) 类型：出变体时小台/远台各 50%。*/
    g.plat_gen_seq++;
    bool make_variant = false;
    if (g.plat_gen_seq > JUMP_VARIANT_WARMUP &&
        g.variant_gap_normal >= JUMP_VARIANT_COOLDOWN &&
        (int)(esp_random() % 100) < JUMP_VARIANT_TRIGGER_PCT)
    {
        make_variant = true;
    }

    if (make_variant)
    {
        g.variant_gap_normal = 0; /* 重置冷却：本张是变体台 */
        if (esp_random() & 1)
        {
            /* 小台子：宽缩到基础宽的 SMALL_MIN%~SMALL_MAX%，落脚面更窄 */
            int lo = JUMP_VARIANT_SMALL_MIN_PCT, hi = JUMP_VARIANT_SMALL_MAX_PCT;
            int pct = lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
            dst->w = dst->w * pct / 100;
            if (dst->w < JUMP_PW_MIN)
                dst->w = JUMP_PW_MIN;
        }
        else
        {
            /* 远台子：在常规间距上额外拉远 FAR_EXTRA_MIN~MAX px（下方再钳到可达上限）*/
            int lo = JUMP_VARIANT_FAR_EXTRA_MIN, hi = JUMP_VARIANT_FAR_EXTRA_MAX;
            gap += lo + (int)(esp_random() % (uint32_t)(hi - lo + 1));
        }
    }
    else
    {
        g.variant_gap_normal++; /* 本张是正常台，冷却计数 +1 */
    }

    /* 水平间距钳到「安全最小中心距」防止台子重叠。
     * 立方体投影宽≈台宽，两台不重叠要求 中心距 ≥ (ref->w + dst->w)/2 + 缝隙。*/
    int min_gap = (ref->w + dst->w) / 2 + JUMP_PLAT_MIN_SPACING;
    if (gap < min_gap)
        gap = min_gap;
    /* 水平间距钳到「可达上限」：即便下方 Y 偏移取到 ±JUMP_GAP_Y_MAX，
     * 直线距离也不超过蓄力能跳的最大值 JUMP_DIST_MAX_PX（留 4px 余量），
     * 保证每张台子（含远台子）都一定跳得到，不会出现必死局。*/
    int reach_max = isqrt(JUMP_DIST_MAX_PX * JUMP_DIST_MAX_PX -
                          JUMP_GAP_Y_MAX * JUMP_GAP_Y_MAX) -
                    4;
    if (gap > reach_max)
        gap = reach_max;

    dst->world_cx = ref->world_cx + gap;
    dst->world_cy = ref->world_cy + rand_gap_y(ref->world_cy);
#if JUMP_PLAT_USE_IMG
    /* 图片台子：从 jt1~jt7 随机选一张，避开与上一台相同（连续不重复）*/
    {
        int idx = (int)(esp_random() % JUMP_PLAT_IMG_COUNT);
        if (idx == ref->img_idx)
            idx = (idx + 1) % JUMP_PLAT_IMG_COUNT;
        dst->img_idx = (uint8_t)idx;
    }
#else
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
#endif
    dst->active = true;
    plat_pend_drop(dst); /* 新台子先挂起隐藏，待滚成下一块目标台、镜头停稳后才从天而降（见 trigger_next_drop）*/
}

static void platforms_init(void)
{
    /* 变体调度计数清零：新一局从热身期重新开始 */
    g.plat_gen_seq = 0;
    g.variant_gap_normal = 0;

    g.plats[0].world_cx = JUMP_START_X;
    g.plats[0].world_cy = JUMP_TOP_Y;
    g.plats[0].w = cur_platform_w();
    g.plats[0].style = 0;                            /* 起始台固定第0种样式（cube 模式）*/
    g.plats[0].depth = plat_w_to_edge(g.plats[0].w); /* 起始台正常厚度（cube 模式）*/
    g.plats[0].stripes = 0;                          /* 起始台无条带，干净（cube 模式）*/
    g.plats[0].pattern = 0;                          /* 起始台无花纹，干净（cube 模式）*/
    g.plats[0].img_idx = 0;                          /* 起始台固定第0张图（图片模式）*/
    g.plats[0].active = true;
    plat_no_drop(&g.plats[0]); /* 起始台直接就位，不掉落 */

    gen_platform(&g.plats[1], &g.plats[0]);
    gen_platform(&g.plats[2], &g.plats[1]);

    /* 离场槽 prev(3)/prevprev(4) 开局无内容：置非激活并清掉残留动画状态。
     * 其 cube 仍保留 build_panel 创建的对象备用；不重置 active 会让上一局残留的
     * 离场台子在新局左侧诡异显示（platform_render_one 只对 active 台子绘制）。*/
    for (int i = 3; i < PLAT_COUNT; i++)
    {
        g.plats[i].active = false;
        g.plats[i].dropping = false;
        g.plats[i].drop_pending = false;
    }

    /* 摄像机归零：cur台在 (JUMP_START_X, JUMP_TOP_Y) */
    g.cam_x = g.cam_y = 0;
    g.cam_target_x = g.cam_target_y = 0;
    g.cam_smooth_frames = 0;

    platforms_render_all();

    /* 开局：让第二块(plats[1])从天而降落定；第三块(plats[2])保持挂起隐藏，
     * 初始界面只见两块。第二块落定后玩家方可起跳（见 do_jump 的 dropping 保护）。
     * 若想开局第二块直接静止落定，把下面改成 plat_no_drop(&g.plats[1]) 即可。*/
    trigger_next_drop();
}

static void platforms_advance(void)
{
    /* 离场台子回收时机：只回收已滚到 prevprev(4)、彻底出屏的那张台子的对象。
     * 旧cur(plats[0])刚跳离，不能立刻回收——否则它会瞬间消失。改为：
     *   prevprev(4) ← prev(3) ← 旧cur(0)，被挤出 prevprev 的旧对象才复用给新 prenext。
     * 这样旧cur保留对象，随相机平移自然向左滑出屏幕（"跟随视觉移动，逐渐离场"）。*/
    lv_obj_t *reuse_cube = g.plats[4].cube; /* prevprev 已出屏，其对象可安全复用 */

    /* 离场槽滚动：旧 prev → prevprev，旧 cur → prev（均保留各自 cube 继续渲染）*/
    g.plats[4] = g.plats[3];
    g.plats[3] = g.plats[0];

    /* 数据滚动：[0]←[1]←[2] */
    g.plats[0] = g.plats[1];
    g.plats[1] = g.plats[2];

    /* 生成新prenext，复用「已出屏的旧 prevprev」立方体对象 */
    gen_platform(&g.plats[2], &g.plats[1]);
    g.plats[2].cube = reuse_cube;

    /* 摄像机目标：只水平跟随，垂直方向锁定（cam_target_y 恒为 0）。
     * 原因：台子世界 Y 已被 rand_gap_y() 钳在 [40, LCD_HEIGHT-60] 屏内范围，
     * 不会飞出屏幕，本不需要相机 Y 跟随。若 cam_target_y 跟随落点台高度，
     * 每跳镜头竖直方向大幅起伏，会把「其它台子（含落点台后一张）」整体上下拖动，
     * 视觉上就是「落地时后方台子也跟着上浮/下沉」的 bug。锁定 Y 后，
     * 落地只剩落点台自身的压扁动画在动，其余台子竖直完全静止。*/
    g.cam_target_x = g.plats[0].world_cx - JUMP_START_X;
    g.cam_target_y = 0;
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

    /* 手感保护：下一块目标台(plats[1])还在从天而降未落定时，不允许起跳，
     * 否则会对着尚在空中的台子做落点判定。等它落定(dropping=false)再跳。*/
    if (g.plats[1].dropping)
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
    /* 定点累积初始值：从起跳位置开始，每帧匀速步进，消除整除截断抖动 */
    g.fly_wx256 = g.player_wx * 256;
    g.fly_wy256 = g.player_wy * 256;
    g.fly_step256_x = g.fly_total_x * 256 / g.fly_steps;
    g.fly_step256_y = g.fly_total_y * 256 / g.fly_steps;
    g.phase = PH_FLY;

    platform_render_one(0, 0); /* 恢复台子压扁 */
    sparks_hide();             /* 起跳：收起蓄力光斑 */
    bsp_motor_pulse();
    ESP_LOGI(TAG, "起跳 dist=%d target(%d,%d)", dist, tx, ty);
}

/* 触发「下一块目标台」(plats[1]) 从天而降。
 * 只针对 plats[1]、一次一块，且仅在「开局」与「镜头平移结束」两个时点调用：
 *   - 旧做法是每帧扫描「进屏即触发」，但落台后镜头还没动时第三块就已在屏内，
 *     会被立刻触发从天而降，和落台压扁同帧 → 后方台子同步浮动（已修复，见计划）。
 *   - 现在改为只在镜头停稳后给下一块目标台触发，保证落台/平移全程后方零浮动。
 * plats[2]（再下一块）保持挂起隐藏，待它滚动成新 plats[1] 后才会被触发。*/
static void trigger_next_drop(void)
{
    if (g.plats[1].active && g.plats[1].drop_pending)
        plat_start_drop(&g.plats[1]);
}

/* ══════════════════════════════════════════════════
 * 物理引擎（30fps）
 * ══════════════════════════════════════════════════ */
static void engine_cb(lv_timer_t *t)
{
    (void)t;
    if (g.screen != JS_PLAYING)
        return;

    /* 注意：不再每帧扫描触发挂起台子（旧 platforms_drop_trigger 会在落台瞬间误触发
     * 第三块从天而降，与落台同帧浮动）。改由 trigger_next_drop 在开局/镜头停稳后调用。*/

    /* 每帧推进台子入场/回弹动画，独立于 phase。
     * 图片模式下 slot0 有落台 bounce 时，棋子也跟随台面下移（视觉同步）。*/
    if (platforms_drop_update())
    {
        for (int i = 0; i < PLAT_COUNT; i++)
        {
            if (g.plats[i].dropping || g.plats[i].drop > 0 || g.plats[i].bounce > 0)
            {
                platform_render_one(i, 0);
#if JUMP_PLAT_USE_IMG && !JUMP_LAND_USE_SQUASH
                /* slot0 有落台回弹时，棋子脚底跟随台面下移（bounce px）。
                 * 仅整体下坠版需要；压扁回弹版在 PH_CAM 内统一驱动棋子跟随。*/
                if (i == 0 && g.phase == PH_CAM && g.plats[0].bounce > 0)
                    player_render(0, 0, g.plats[0].bounce);
#endif
            }
        }
    }

    switch (g.phase)
    {
    /* 待蓄力 */
    case PH_IDLE:
    {
        uint32_t held = bsp_touch_page_held_ms();
        if (held == 0)
        {
#if JUMP_PLAT_USE_IMG
            player_render(0, 0, g.plats[0].bounce); /* 有残余落台回弹时棋子跟随 */
#else
            player_render(0, 0, 0);
#endif
            platform_render_one(0, 0);
            sparks_hide(); /* 未蓄力：无光斑 */
        }
        else
        {
            /* 蓄力百分比从按下瞬间就开始计算（0~100%对应 0~JUMP_HOLD_MAX_MS），
             * 不再等到 JUMP_HOLD_MIN_MS 才开始，棋子/台子立即响应压扁。*/
            uint32_t clamped = held > JUMP_HOLD_MAX_MS ? JUMP_HOLD_MAX_MS : held;
            int pct = (int)(clamped * 100u / JUMP_HOLD_MAX_MS);
#if JUMP_PLAT_USE_IMG
            /* 图片台子：先渲染台子（写入 squash_face_sink），再让棋子 sink 跟随台面下沉。
             * 台子底部固定、顶面向下压；台面下沉多少，棋子脚底就跟着沉多少。*/
            platform_render_one(0, pct);
            int sink = g.plats[0].squash_face_sink;
            player_render(pct, 0, sink);
            sparks_update(pct, sink);
#else
            int sink = g.plats[0].depth * pct / 100;
            player_render(pct, 0, sink);
            platform_render_one(0, pct);
            sparks_update(pct, sink);
#endif
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

        /* 位置：定点累积步进，避免每帧重新整除导致 ±1px 截断抖动 */
        g.fly_wx256 += g.fly_step256_x;
        g.fly_wy256 += g.fly_step256_y;
        g.player_wx = g.fly_wx256 >> 8;
        g.player_wy = g.fly_wy256 >> 8;
        /* 最后一步强制对齐终点，消除定点累积的尾部误差 */
        if (g.fly_step >= g.fly_steps)
        {
            g.player_wx = g.fly_start_wx + g.fly_total_x;
            g.player_wy = g.fly_start_wy + g.fly_total_y;
        }

        /* 弧高：h = apex * 4 * t * (1-t) */
        int fly_h = g.fly_apex * 4 * t256 * (256 - t256) / (256 * 256);

        player_render(0, fly_h, 0);

        if (g.fly_step < g.fly_steps)
            break; /* 仍在飞 */

        /* ── 到达终点，判定落点 ── */
        /* 落点判定：小人最终世界 x 是否落在 next 台「可站立顶面」内。
         * 关键修复：用 plat_face_half_w（顶面半宽）而非整图半宽 w/2，
         * 否则棋子落在台子透明边/侧壁斜面上也算成功（视觉上悬空在台面外）。*/
        int half = plat_face_half_w(&g.plats[1]);
        int nl = g.plats[1].world_cx - half;
        int nr = g.plats[1].world_cx + half;
        /* 视觉飞行终点即判定点（二者已统一，不再单独重算）*/
        int actual_wx = g.player_wx;

        /* 摔倒判定：棋子中心超出台缘多少（overshoot>0 才有出台风险）。
         * overshoot<=0          → 重心在台面内，稳稳站住（成功）
         * 0<overshoot<脚底半宽×倍率 → 踩到台缘但落脚面撑不住 → 摔倒（失败）
         * overshoot>=该上界      → 重心彻底出台缘 → 完全没站住，直接坠落（失败）*/
        int overshoot = 0;
        int topple_dir = 0;
        if (actual_wx > nr)
        {
            overshoot = actual_wx - nr;
            topple_dir = +1; /* 落点偏台右，向右倒 */
        }
        else if (actual_wx < nl)
        {
            overshoot = nl - actual_wx;
            topple_dir = -1; /* 落点偏台左，向左倒 */
        }
        int foot_half = plat_foot_half_w();
        int topple_limit = foot_half * JUMP_TOPPLE_RANGE_PCT / 100; /* 摔倒区上界 */

        ESP_LOGI(TAG, "判定 actual_wx=%d next[%d,%d] overshoot=%d foot_half=%d",
                 actual_wx, nl, nr, overshoot, foot_half);

        if (overshoot <= 0)
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
            /* 保险：玩家落上的台子强制结束入场动画（清掉残留掉落状态），随后再注入落台回弹 */
            plat_no_drop(&g.plats[0]);
#if JUMP_PLAT_USE_IMG
#if JUMP_LAND_USE_SQUASH
            /* 棋子落上台子：台子顶面压扁→回弹到原高度（底部锁死）。
             * PH_CAM 每帧 land_squash_step/render 推进，回弹结束后才回 PH_IDLE。*/
            plat_land_squash(&g.plats[0]);
#else
            /* 棋子落上台子：台子整体被踩一下下压再回弹（旧逻辑，图片模式专属手感）。
             * platforms_drop_update 每帧独立推进其回弹，PH_CAM 期间自然播完。*/
            plat_land_bounce(&g.plats[0]);
#endif
#endif

            hud_refresh();
            /* 落台后先等 JUMP_CAM_DELAY_FRAMES 帧，让玩家看清落点，再开始视线平移 */
            g.cam_delay_frames = JUMP_CAM_DELAY_FRAMES;
            g.phase = PH_CAM;
            player_render(0, 0, 0);
            platform_render_one(0, 0); /* 立即渲染一帧体现下压起始 */
            ESP_LOGI(TAG, "落台成功 score=%d", g.score);
        }
        else if (overshoot < topple_limit)
        {
            /* ── 摔倒：棋子踩到台缘，重心仍在台面内但落脚面撑不住 ──
             * 先在落点站稳一瞬（topple_hold 帧），再绕脚底朝台外侧倾倒掉落。
             * 关键：绝不调 platforms_advance()，台子数据保持不变，倾倒支点才对得上台缘。
             * 支点固定在「靠近台心一侧的台缘」：向右倒→脚踩右缘(nr)，向左倒→脚踩左缘(nl)。*/
            g.player_wx = actual_wx;
            g.player_wy = g.plats[1].world_cy; /* 脚底高度=台面，先站上去 */
            g.topple_dir = topple_dir;
            g.topple_pivot_wx = (topple_dir > 0) ? nr : nl; /* 脚底支点=所踩台缘 */
            g.topple_ang = 0;
            g.topple_hold = JUMP_TOPPLE_HOLD_FRAMES;
            g.fall_vy = 0;
            g.fall_h = 0;
            g.phase = PH_TOPPLE;
            highscore_save_if_better(g.score);
            /* 站稳第一帧：竖直渲染在落点（尚未倾倒）*/
            player_render(0, 0, 0);
            ESP_LOGI(TAG, "摔倒 wx=%d dir=%d pivot=%d", actual_wx, topple_dir, g.topple_pivot_wx);
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
    {
#if JUMP_PLAT_USE_IMG && JUMP_LAND_USE_SQUASH
        /* 压扁回弹版：先推进 slot0 压扁回弹状态机一帧（仅状态，渲染在各分支内）。
         * 棋子脚底跟随台面下沉量 cam_sink（由 land_squash_render_slot0 返回）。*/
        land_squash_step(&g.plats[0]);
        int cam_sink = 0;
        /* 延迟阶段：棋子在落点，台子压扁回弹动画播放，玩家看清落点。
         * 必须先 platforms_render_all() 把所有台子按 squash=0 重渲一遍，
         * 否则其它台子会卡在上一帧 land_squash_render_slot0 留下的 scale_y 压扁状态，
         * 出现「其它台子也跟着压扁」的 bug（slot0 之后再叠加压扁覆盖）。*/
        if (g.cam_delay_frames > 0)
        {
            g.cam_delay_frames--;
            platforms_render_all();
            cam_sink = land_squash_render_slot0();
            player_render(0, 0, cam_sink);
            break;
        }
        if (g.cam_smooth_frames > 0)
        {
            /* 每帧向目标前进 1/remaining 步（匀减速感）*/
            g.cam_x += (g.cam_target_x - g.cam_x + g.cam_smooth_frames - 1) / g.cam_smooth_frames;
            g.cam_y += (g.cam_target_y - g.cam_y + g.cam_smooth_frames - 1) / g.cam_smooth_frames;
            g.cam_smooth_frames--;
            platforms_render_all();                /* 先按相机渲染全部（slot0 squash=0）*/
            cam_sink = land_squash_render_slot0(); /* 再叠加 slot0 压扁，覆盖其渲染 */
            player_render(0, 0, cam_sink);
            //! 诊断[平移帧]：cam_y 是否真锁住=0？台子3(slot1)屏幕y 是否随帧动？*/
            // ESP_LOGW(TAG, "PAN camX=%d camY=%d tgtY=%d | s1_py=%d s1_wy=%d",
            //          g.cam_x, g.cam_y, g.cam_target_y,
            //          (int)lv_obj_get_y(g.plats[1].cube), g.plats[1].world_cy);
        }
        if (g.cam_smooth_frames <= 0)
        {
            /* 对齐到精确目标，消除累积误差 */
            g.cam_x = g.cam_target_x;
            g.cam_y = g.cam_target_y;
            platforms_render_all();
            cam_sink = land_squash_render_slot0();
            player_render(0, 0, cam_sink);
            /* 相机到位后，仅当压扁回弹也播完才回 IDLE（否则停在 CAM 继续播完回弹）*/
            if (!g.plats[0].land_squashing)
            {
                g.phase = PH_IDLE;
                /* 镜头停稳：此刻才让下一块目标台(plats[1])从天而降，避免与落台同帧浮动 */
                trigger_next_drop();
            }
        }
        break;
#else
        /* 整体下坠版（旧逻辑）：棋子跟随 slot0 落台回弹偏移（bounce = 下移px，随台子逐帧归零）*/
#if JUMP_PLAT_USE_IMG
        int cam_sink = g.plats[0].bounce;
#else
        int cam_sink = 0;
#endif
        /* 延迟阶段：棋子在落点静止，台子回弹动画播放，玩家看清落点 */
        if (g.cam_delay_frames > 0)
        {
            g.cam_delay_frames--;
            player_render(0, 0, cam_sink);
            break;
        }
        if (g.cam_smooth_frames > 0)
        {
            /* 每帧向目标前进 1/remaining 步（匀减速感）*/
            g.cam_x += (g.cam_target_x - g.cam_x + g.cam_smooth_frames - 1) / g.cam_smooth_frames;
            g.cam_y += (g.cam_target_y - g.cam_y + g.cam_smooth_frames - 1) / g.cam_smooth_frames;
            g.cam_smooth_frames--;
            platforms_render_all();
            player_render(0, 0, cam_sink);
        }
        if (g.cam_smooth_frames <= 0)
        {
            /* 对齐到精确目标，消除累积误差 */
            g.cam_x = g.cam_target_x;
            g.cam_y = g.cam_target_y;
            platforms_render_all();
            player_render(0, 0, cam_sink);
            g.phase = PH_IDLE;
            /* 镜头停稳：此刻才让下一块目标台(plats[1])从天而降，避免与落台同帧浮动 */
            trigger_next_drop();
        }
        break;
#endif
    }

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

    /* 摔倒：踩到台缘但落脚面不够 → 先站稳一瞬，再绕脚底朝台外倾倒并下坠（失败）*/
    case PH_TOPPLE:
    {
        /* 阶段A：站稳停顿——人物竖直站在落点，仅展示「站上去了」 */
        if (g.topple_hold > 0)
        {
            g.topple_hold--;
            player_render(0, 0, 0);
            break;
        }

        /* 阶段B：绕脚底倾倒 + 整体下坠 */
        if (g.topple_ang < JUMP_TOPPLE_MAX_ANG)
        {
            /* 仍在倾倒：角度逐帧增大；倒过一半后开始重力下坠（失去支撑滑落台缘）*/
            g.topple_ang += JUMP_TOPPLE_ANG_STEP;
            if (g.topple_ang > JUMP_TOPPLE_MAX_ANG)
                g.topple_ang = JUMP_TOPPLE_MAX_ANG;
            if (g.topple_ang > JUMP_TOPPLE_MAX_ANG / 2)
            {
                g.fall_vy += 2;
                g.player_wy += g.fall_vy;
            }
        }
        else
        {
            /* 倾倒到位（已趴下）：纯重力坠落，沿倾倒方向略微外移 */
            g.fall_vy += 3;
            g.player_wy += g.fall_vy;
            g.topple_pivot_wx += g.topple_dir * 2; /* 脱离台缘后沿倾倒方向外滑 */
        }
        player_render_topple(g.topple_ang, g.topple_dir, 0);
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
    g.cam_delay_frames = 0;

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

    /* 三个台子对象（先创建=在小人之下）。cube 字段两模式复用。*/
    for (int i = 0; i < PLAT_COUNT; i++)
    {
#if JUMP_PLAT_USE_IMG
        /* 图片台子：lv_image，src/scale/pos 在 platform_render_one 里按帧设置 */
        lv_obj_t *plat = lv_image_create(s_panel);
        lv_image_set_antialias(plat, true);
        lv_obj_clear_flag(plat, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_flag(plat, LV_OBJ_FLAG_HIDDEN);
        g.plats[i].cube = plat;
        g.plats[i].img_idx = 0;
#else
        lv_obj_t *cube = cube3d_create(s_panel);
        cube3d_set_geometry(cube, plat_w_to_edge(s_diff[DIFF_EASY].pw));
        cube3d_set_top_color(cube, s_plat_styles[0]);
        lv_obj_add_flag(cube, LV_OBJ_FLAG_HIDDEN);
        g.plats[i].cube = cube;
        g.plats[i].style = 0;
#endif
        g.plats[i].active = false;
    }

    /* （人物影子已按用户反馈去掉，s_player_shadow 保持 NULL，相关代码均 NULL 守卫）*/

    /* 小人：图片 jp（8.png，彩色带 alpha），最后创建=在最上层。等比缩放在 player_render 里按需设置。
     * 注意：8.png 本身有色，不再做黑色 recolor（旧 j4 才需涂黑），否则棋子会变黑块。*/
    s_player = lv_image_create(s_panel);
    lv_image_set_src(s_player, &JUMP_PLAYER_IMG);
    lv_image_set_antialias(s_player, true);
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
    enter_playing();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "跳一跳启动");
}

void jump_touch(touch_event_t event)
{
    if (!lvgl_port_lock(100))
        return;
    switch (g.screen)
    {
    case JS_PLAYING:
        if (g.phase == PH_IDLE &&
            (event == TOUCH_EVENT_SHORT_PREV_PAGE ||
             event == TOUCH_EVENT_SHORT_NEXT_PAGE))
            do_jump(bsp_touch_last_page_hold_ms());
        break;
    case JS_RESULT:
        if (event == TOUCH_EVENT_SHORT_HEAD)
            enter_playing();
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
