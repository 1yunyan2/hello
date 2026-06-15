/**
 * @file cube3d.c
 * @brief 等轴测立方体控件实现（LVGL 9.5.0 纯 C）
 *
 * 几何（对象局部坐标，原点=对象左上角）：
 *   设边长 a，cos30≈0.866，dx=a*cos30，dy=a/2，竖直高度 vh=a*(100-squash)/100。
 *   顶面是一个菱形（rhombus），其四角：
 *     T(顶) (dx, 0)   R(右) (2dx, dy)   B(底中) (dx, 2dy)   L(左) (0, dy)
 *   正面（左下平行四边形）：L(0,dy) → B(dx,2dy) → B+vh(dx,2dy+vh) → L+vh(0,dy+vh)
 *   右侧面（右下平行四边形）：B(dx,2dy) → R(2dx,dy) → R+vh(2dx,dy+vh) → B+vh(dx,2dy+vh)
 *   对象包围盒：宽 = 2dx，高 = 2dy + vh（squash=0 时 = a + a = 2a 的近似，dy=a/2 故 2dy=a）。
 *
 *   实际高度 = 2dy + vh = a + vh。为简单起见对象高度按最大（squash=0）预留 a + a = 2a，
 *   压扁时只是内部画矮，包围盒不缩（多出的透明区不影响）。
 *
 * 每个面是平行四边形，用 2 个 lv_draw_triangle 拼成。
 * 颜色绘制顺序：正面/右侧面（下层）先画，顶面（上层）后画——本就无重叠，顺序无所谓，
 * 但保持「先侧后顶」习惯。
 */
#include "cube3d.h"
#include "esp_log.h"

/* cos30 定点：0.8660254 * 256 ≈ 222 */
#define CUBE_COS30_FP 222
#define CUBE_FP_SHIFT 8

typedef struct {
    int      edge;        /* 边长 a（决定顶面菱形大小）*/
    int      depth;       /* 竖直厚度（squash=0 时的“高”），独立于 edge，可做非对称扁/高台 */
    int      squash_pct;  /* 0~100 压扁（自上而下：底面固定、顶面下沉）*/
    uint32_t c_top, c_front, c_right;  /* 三面 RGB888 */
    uint8_t  stripe_mask; /* 侧面亮色条带的位掩码（bit i = 第 i 条带亮），0=无条带 */
    uint8_t  pattern;     /* 顶面/正面几何花纹类型（见 cube3d_pattern_t），0=无 */
    bool     shadow;      /* 是否在底部画椭圆影子 */
} cube_data_t;

/* 把 RGB888 调暗到 ratio/256（生成阴影面色）*/
static uint32_t cube_darken(uint32_t rgb, int ratio)
{
    uint32_t r = (rgb >> 16) & 0xFF, g = (rgb >> 8) & 0xFF, b = rgb & 0xFF;
    r = r * ratio / 256; g = g * ratio / 256; b = b * ratio / 256;
    return (r << 16) | (g << 8) | b;
}

static inline int cube_dx(int a) { return a * CUBE_COS30_FP >> CUBE_FP_SHIFT; }
static inline int cube_dy(int a) { return a / 2; }
/* 当前竖直厚度（压扁后）：从 depth 线性压到 (100-squash)% */
static inline int cube_vh(int depth, int squash) { return depth * (100 - squash) / 100; }

/* 画一个平行四边形（4 顶点）= 2 个三角形 */
static void draw_quad(lv_layer_t *layer, lv_draw_triangle_dsc_t *dsc,
                      int x0, int y0, int x1, int y1,
                      int x2, int y2, int x3, int y3)
{
    /* 三角形 1：0-1-2 */
    dsc->p[0].x = x0; dsc->p[0].y = y0;
    dsc->p[1].x = x1; dsc->p[1].y = y1;
    dsc->p[2].x = x2; dsc->p[2].y = y2;
    lv_draw_triangle(layer, dsc);
    /* 三角形 2：0-2-3 */
    dsc->p[0].x = x0; dsc->p[0].y = y0;
    dsc->p[1].x = x2; dsc->p[1].y = y2;
    dsc->p[2].x = x3; dsc->p[2].y = y3;
    lv_draw_triangle(layer, dsc);
}

/* 在某个侧面平行四边形上画几条水平亮色条带（按 stripe_mask）。
 * 平行四边形由「上沿两点 (ax,ay)-(bx,by)」+ 竖直高度 vh 定义（向下拉 vh）。
 * 条带数固定 CUBE_STRIPE_N，bit i 亮则在第 i 段画一条更亮的细带。*/
#define CUBE_STRIPE_N 4
static void draw_side_stripes(lv_layer_t *layer, lv_draw_triangle_dsc_t *dsc,
                              uint8_t mask, uint32_t base_color,
                              int ax, int ay, int bx, int by, int vh)
{
    if (mask == 0 || vh < CUBE_STRIPE_N * 2) return;
    /* 亮带色 = 基色每通道 +60 提亮（封顶 255）*/
    uint32_t br = (base_color >> 16) & 0xFF, bg = (base_color >> 8) & 0xFF, bb = base_color & 0xFF;
    br = br + 60 > 255 ? 255 : br + 60;
    bg = bg + 60 > 255 ? 255 : bg + 60;
    bb = bb + 60 > 255 ? 255 : bb + 60;
    uint32_t bright = (br << 16) | (bg << 8) | bb;
    int seg = vh / CUBE_STRIPE_N;           /* 每段高度 */
    int band = seg / 2; if (band < 2) band = 2;  /* 亮带厚度 */
    dsc->color = lv_color_hex(bright);
    for (int i = 0; i < CUBE_STRIPE_N; i++) {
        if (!(mask & (1 << i))) continue;
        int y0 = i * seg + (seg - band) / 2;  /* 该段内居中的一条亮带 */
        int y1 = y0 + band;
        /* 平行四边形在深度 y0..y1 的横切（上下沿平移 y0/y1）*/
        draw_quad(layer, dsc,
                  ax, ay + y0, bx, by + y0,
                  bx, by + y1, ax, ay + y1);
    }
}

/* 画一个「缩小的顶面菱形」（同心菱形）：四角向中心收缩 shrink/256。
 * cx,cy=顶面中心；hx=半宽(dx)、hy=半高(dy)。scale=外形相对完整菱形的比例(0~256)。*/
static void draw_top_diamond(lv_layer_t *layer, lv_draw_triangle_dsc_t *dsc,
                             int cx, int cy, int hx, int hy, int scale)
{
    int x = hx * scale / 256, y = hy * scale / 256;
    draw_quad(layer, dsc,
              cx,     cy - y,   /* 上 */
              cx + x, cy,       /* 右 */
              cx,     cy + y,   /* 下 */
              cx - x, cy);      /* 左 */
}

/* 在顶面/正面叠画几何花纹（pattern 见 cube3d.h 注释）。
 * 顶面中心 (tcx,tcy)、半宽 dx、半高 dy；正面参考点用 L/B 角 + 高度 vh。
 * 花纹色基于顶/正面色提亮或调暗，保证在各种底色上可见。*/
/* 在一个侧面平行四边形上画一条「横带」：上沿 (ax,ay)→(bx,by)，
 * 在深度 [y0,y1] 区间填色（两端各自向下平移）。正面/右侧面通用。*/
static void draw_face_band(lv_layer_t *layer, lv_draw_triangle_dsc_t *pd,
                           int ax, int ay, int bx, int by, int y0, int y1)
{
    draw_quad(layer, pd, ax, ay + y0, bx, by + y0, bx, by + y1, ax, ay + y1);
}

/* 在一个侧面平行四边形上画一条「竖带」：沿上沿斜线取 [t0,t1]% 段，向下拉 vh。*/
static void draw_face_vstripe(lv_layer_t *layer, lv_draw_triangle_dsc_t *pd,
                              int ax, int ay, int bx, int by, int vh, int t0, int t1)
{
    int x0 = ax + (bx - ax) * t0 / 100, yy0 = ay + (by - ay) * t0 / 100;
    int x1 = ax + (bx - ax) * t1 / 100, yy1 = ay + (by - ay) * t1 / 100;
    draw_quad(layer, pd, x0, yy0, x1, yy1, x1, yy1 + vh, x0, yy0 + vh);
}

/* 在一个侧面平行四边形上画一个「圆」：圆心在该面中部偏下。*/
static void draw_face_circle(lv_layer_t *layer, lv_draw_rect_dsc_t *rd,
                             int ax, int ay, int bx, int by, int vh, int r)
{
    int ccx = (ax + bx) / 2, ccy = (ay + by) / 2 + vh / 2;
    rd->bg_opa = LV_OPA_COVER; rd->radius = LV_RADIUS_CIRCLE;
    lv_area_t ca = { ccx - r, ccy - r, ccx + r, ccy + r };
    lv_draw_rect(layer, rd, &ca);
}

static void draw_cube_pattern(lv_layer_t *layer, uint8_t pattern,
                              int tcx, int tcy, int dx, int dy,
                              uint32_t top_color, uint32_t front_color,
                              int Lx, int Ly, int Bx, int By, int Rx, int Ry, int vh)
{
    if (pattern == 0) return;

    /* 花纹色：顶面用调暗色（深色花纹），高亮用提亮色 */
    uint32_t dark  = cube_darken(top_color, 150);
    uint32_t darkr = cube_darken(top_color, 190);
    uint32_t light;
    {
        uint32_t r = (top_color >> 16) & 0xFF, g = (top_color >> 8) & 0xFF, b = top_color & 0xFF;
        r = r + 50 > 255 ? 255 : r + 50; g = g + 50 > 255 ? 255 : g + 50; b = b + 50 > 255 ? 255 : b + 50;
        light = (r << 16) | (g << 8) | b;
    }

    lv_draw_triangle_dsc_t pd;
    lv_draw_triangle_dsc_init(&pd);
    pd.opa = LV_OPA_COVER;

    lv_draw_rect_dsc_t rd;
    lv_draw_rect_dsc_init(&rd);

    switch (pattern) {
    case 1: /* 井盖：2 层同心菱形（深-浅交替）*/
        pd.color = lv_color_hex(darkr);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 180);
        pd.color = lv_color_hex(top_color);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 110);
        break;

    case 2: /* 中心方块：顶面正中一个小菱形深块 */
        pd.color = lv_color_hex(dark);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 90);
        break;

    case 3: /* 十字：顶面沿两对角的细带（T-B 与 L-R 方向各一条）*/
        pd.color = lv_color_hex(dark);
        /* 竖轴细菱（窄横宽） */
        draw_quad(layer, &pd, tcx, tcy - dy, tcx + dx/6, tcy, tcx, tcy + dy, tcx - dx/6, tcy);
        /* 横轴细菱 */
        draw_quad(layer, &pd, tcx - dx, tcy, tcx, tcy - dy/6, tcx + dx, tcy, tcx, tcy + dy/6);
        break;

    case 4: /* 招牌横条：正面+右侧面各一条亮色粗横带（便利店招牌感，两面都有）*/
    {
        int y0 = vh / 6, y1 = vh / 6 + vh / 4; if (y1 - y0 < 2) y1 = y0 + 2;
        pd.color = lv_color_hex(light);
        draw_face_band(layer, &pd, Lx, Ly, Bx, By, y0, y1);   /* 正面 */
        draw_face_band(layer, &pd, Bx, By, Rx, Ry, y0, y1);   /* 右侧面 */
        break;
    }

    case 5: /* logo 圆：正面+右侧面各一个亮色圆（音箱/标志感，两面都有）*/
    {
        int r = vh / 3; if (r < 3) r = 3;
        rd.bg_color = lv_color_hex(light);
        draw_face_circle(layer, &rd, Lx, Ly, Bx, By, vh, r);  /* 正面 */
        draw_face_circle(layer, &rd, Bx, By, Rx, Ry, vh, r);  /* 右侧面 */
        break;
    }

    case 6: /* 边框：顶面描一圈深色内边框（画大菱形再叠原色小菱形）*/
        pd.color = lv_color_hex(darkr);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 230);
        pd.color = lv_color_hex(top_color);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 180);
        break;

    case 7: /* 双横线：正面+右侧面各两条亮横带（加粗，两面都有）*/
    {
        int band = vh / 7; if (band < 3) band = 3;   /* 横带厚度，至少 3px */
        pd.color = lv_color_hex(light);
        for (int k = 1; k <= 2; k++) {
            int y0 = vh * k / 3 - band / 2, y1 = y0 + band;
            draw_face_band(layer, &pd, Lx, Ly, Bx, By, y0, y1);   /* 正面 */
            draw_face_band(layer, &pd, Bx, By, Rx, Ry, y0, y1);   /* 右侧面 */
        }
        break;
    }

    case 8: /* 三同心菱形（井盖加强版）*/
        pd.color = lv_color_hex(darkr);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 200);
        pd.color = lv_color_hex(light);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 140);
        pd.color = lv_color_hex(dark);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 70);
        break;

    case 9: /* 顶面圆点：正中一个深色圆 */
    {
        int r = dy / 2; if (r < 3) r = 3;
        rd.bg_color = lv_color_hex(dark); rd.bg_opa = LV_OPA_COVER; rd.radius = LV_RADIUS_CIRCLE;
        lv_area_t ca = { tcx - r, tcy - r, tcx + r, tcy + r };
        lv_draw_rect(layer, &rd, &ca);
        break;
    }

    case 10: /* 半分菱形：顶面下半深色（对角分两色）*/
        pd.color = lv_color_hex(darkr);
        draw_quad(layer, &pd, tcx - dx, tcy, tcx, tcy + dy, tcx + dx, tcy, tcx, tcy + dy/4);
        break;

    case 11: /* 竖亮带（门/书脊感）：正面+右侧面各一条，沿上沿斜线取中段拉 vh */
        pd.color = lv_color_hex(light);
        draw_face_vstripe(layer, &pd, Lx, Ly, Bx, By, vh, 40, 60);   /* 正面 */
        draw_face_vstripe(layer, &pd, Bx, By, Rx, Ry, vh, 40, 60);   /* 右侧面 */
        break;

    case 12: /* 顶面四点（角落各一小圆，糖果感）*/
    {
        int r = dy / 5; if (r < 2) r = 2;
        rd.bg_color = lv_color_hex(light); rd.bg_opa = LV_OPA_COVER; rd.radius = LV_RADIUS_CIRCLE;
        int px[4] = { tcx, tcx + dx/2, tcx, tcx - dx/2 };
        int py[4] = { tcy - dy/2, tcy, tcy + dy/2, tcy };
        for (int k = 0; k < 4; k++) {
            lv_area_t ca = { px[k]-r, py[k]-r, px[k]+r, py[k]+r };
            lv_draw_rect(layer, &rd, &ca);
        }
        break;
    }

    case 13: /* 顶面双圆环（深圆里套亮圆，靶心感）*/
    {
        int r1 = dy * 3 / 4, r2 = dy * 2 / 5;
        if (r1 < 3) r1 = 3;
        if (r2 < 2) r2 = 2;
        rd.bg_opa = LV_OPA_COVER; rd.radius = LV_RADIUS_CIRCLE;
        rd.bg_color = lv_color_hex(dark);
        lv_area_t c1 = { tcx-r1, tcy-r1, tcx+r1, tcy+r1 }; lv_draw_rect(layer, &rd, &c1);
        rd.bg_color = lv_color_hex(light);
        lv_area_t c2 = { tcx-r2, tcy-r2, tcx+r2, tcy+r2 }; lv_draw_rect(layer, &rd, &c2);
        break;
    }

    case 14: /* 顶面斜分双色（沿一条对角线分两色，半边调暗）*/
        pd.color = lv_color_hex(darkr);
        /* 取 T-R-B 半边三角（右半）*/
        draw_quad(layer, &pd, tcx, tcy - dy, tcx + dx, tcy, tcx, tcy + dy, tcx, tcy - dy);
        break;

    case 15: /* 三竖带（栅栏/百叶感）：正面+右侧面各三条 */
        pd.color = lv_color_hex(light);
        for (int k = 1; k <= 3; k++) {
            int t0 = k * 25 - 5, t1 = k * 25 + 5;   /* 25/50/75% 各一窄带 */
            draw_face_vstripe(layer, &pd, Lx, Ly, Bx, By, vh, t0, t1);  /* 正面 */
            draw_face_vstripe(layer, &pd, Bx, By, Rx, Ry, vh, t0, t1);  /* 右侧面 */
        }
        break;

    case 16: /* 顶面棋盘四格（左右深色、上下原色，方格感）*/
        pd.color = lv_color_hex(darkr);
        /* 左半三角 */
        draw_quad(layer, &pd, tcx, tcy - dy, tcx, tcy + dy, tcx - dx, tcy, tcx - dx, tcy);
        break;

    case 17: /* 顶面中心亮圆 + 深边框（按钮感）*/
    {
        pd.color = lv_color_hex(dark);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 160);
        int r = dy / 2; if (r < 3) r = 3;
        rd.bg_color = lv_color_hex(light); rd.bg_opa = LV_OPA_COVER; rd.radius = LV_RADIUS_CIRCLE;
        lv_area_t ca = { tcx-r, tcy-r, tcx+r, tcy+r }; lv_draw_rect(layer, &rd, &ca);
        break;
    }

    case 18: /* 正面圆 + 顶面圆（双面圆点，骰子感）*/
    {
        int rt = dy / 3, rf = vh / 3;
        if (rt < 2) rt = 2;
        if (rf < 2) rf = 2;
        rd.bg_opa = LV_OPA_COVER; rd.radius = LV_RADIUS_CIRCLE;
        rd.bg_color = lv_color_hex(dark);
        lv_area_t cat = { tcx-rt, tcy-rt, tcx+rt, tcy+rt }; lv_draw_rect(layer, &rd, &cat);
        /* 正面 + 右侧面各一个亮圆（顶面 + 两侧面都有点）*/
        rd.bg_color = lv_color_hex(light);
        draw_face_circle(layer, &rd, Lx, Ly, Bx, By, vh, rf);  /* 正面 */
        draw_face_circle(layer, &rd, Bx, By, Rx, Ry, vh, rf);  /* 右侧面 */
        break;
    }

    case 19: /* 上下双粗带（顶亮带 + 底深带，礼盒感）：正面+右侧面都有 */
    {
        int band = vh / 4; if (band < 3) band = 3;
        pd.color = lv_color_hex(light);
        draw_face_band(layer, &pd, Lx, Ly, Bx, By, 0, band);            /* 正面顶带 */
        draw_face_band(layer, &pd, Bx, By, Rx, Ry, 0, band);           /* 右侧面顶带 */
        pd.color = lv_color_hex(darkr);
        draw_face_band(layer, &pd, Lx, Ly, Bx, By, vh - band, vh);     /* 正面底带 */
        draw_face_band(layer, &pd, Bx, By, Rx, Ry, vh - band, vh);     /* 右侧面底带 */
        break;
    }

    case 20: /* 顶面小菱形点阵（中心 + 四角五个小菱，花砖感）*/
    {
        pd.color = lv_color_hex(dark);
        draw_top_diamond(layer, &pd, tcx, tcy, dx, dy, 45);            /* 中心 */
        draw_top_diamond(layer, &pd, tcx, tcy - dy/2, dx, dy, 28);     /* 上 */
        draw_top_diamond(layer, &pd, tcx, tcy + dy/2, dx, dy, 28);     /* 下 */
        draw_top_diamond(layer, &pd, tcx + dx/2, tcy, dx, dy, 28);     /* 右 */
        draw_top_diamond(layer, &pd, tcx - dx/2, tcy, dx, dy, 28);     /* 左 */
        break;
    }

    default: break;
    }
}

/* DRAW_MAIN 事件：影子 → 三面 → 侧面亮条带 → 几何花纹 */
static void cube_draw_cb(lv_event_t *e)
{
    lv_obj_t *cube = lv_event_get_target_obj(e);
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d || d->edge <= 0) return;

    lv_layer_t *layer = lv_event_get_layer(e);

    /* 对象绝对坐标左上角 */
    lv_area_t area;
    lv_obj_get_coords(cube, &area);
    int ox = area.x1, oy = area.y1;

    int a    = d->edge;
    int dx   = cube_dx(a);
    int dy   = cube_dy(a);
    int full = d->depth;                     /* squash=0 时的竖直厚度 */
    int vh   = cube_vh(d->depth, d->squash_pct);
    int sink = full - vh;                    /* 顶面相对“不压扁”下沉的量（自上而下压扁）*/

    /* 影子：画在“底面”下方一点的扁椭圆（半透明深色）。
     * 底面中点固定在 (ox+dx, oy + 2dy + full)。压扁不动底面，影子也不动。*/
    if (d->shadow) {
        int sh_cx = ox + dx;
        int sh_cy = oy + 2 * dy + full + 2;
        int sh_w  = 2 * dx;                  /* 与立方体同宽 */
        int sh_h  = dy;                      /* 扁 */
        lv_draw_rect_dsc_t sd;
        lv_draw_rect_dsc_init(&sd);
        sd.bg_color = lv_color_hex(0x000000);
        sd.bg_opa   = LV_OPA_30;
        sd.radius   = LV_RADIUS_CIRCLE;
        lv_area_t sa = { sh_cx - sh_w / 2, sh_cy - sh_h / 2,
                         sh_cx + sh_w / 2, sh_cy + sh_h / 2 };
        lv_draw_rect(layer, &sd, &sa);
    }

    /* 顶面菱形四角（局部）。自上而下压扁：顶面整体下沉 sink。*/
    int Tx = ox + dx,     Ty = oy + 0 + sink;        /* 顶 */
    int Rx = ox + 2 * dx, Ry = oy + dy + sink;       /* 右 */
    int Bx = ox + dx,     By = oy + 2 * dy + sink;   /* 底中（顶面下角）*/
    int Lx = ox + 0,      Ly = oy + dy + sink;       /* 左 */

    lv_draw_triangle_dsc_t dsc;
    lv_draw_triangle_dsc_init(&dsc);
    dsc.opa = LV_OPA_COVER;

    /* 正面（左下平行四边形）：L → B → B↓ → L↓ */
    dsc.color = lv_color_hex(d->c_front);
    draw_quad(layer, &dsc, Lx, Ly, Bx, By, Bx, By + vh, Lx, Ly + vh);
    draw_side_stripes(layer, &dsc, d->stripe_mask, d->c_front, Lx, Ly, Bx, By, vh);

    /* 右侧面（右下平行四边形）：B → R → R↓ → B↓ */
    dsc.color = lv_color_hex(d->c_right);
    draw_quad(layer, &dsc, Bx, By, Rx, Ry, Rx, Ry + vh, Bx, By + vh);
    draw_side_stripes(layer, &dsc, d->stripe_mask, d->c_right, Bx, By, Rx, Ry, vh);

    /* 顶面菱形：T → R → B → L（最后画=盖在最上）*/
    dsc.color = lv_color_hex(d->c_top);
    draw_quad(layer, &dsc, Tx, Ty, Rx, Ry, Bx, By, Lx, Ly);

    /* 几何花纹：在顶面/正面叠画（顶面中心 = T 与 B 的中点）*/
    if (d->pattern) {
        int tcx = (Tx + Bx) / 2, tcy = (Ty + By) / 2;
        draw_cube_pattern(layer, d->pattern, tcx, tcy, dx, dy,
                          d->c_top, d->c_front, Lx, Ly, Bx, By, Rx, Ry, vh);
    }
}

/* 对象删除时释放堆上的参数结构体（防泄漏）*/
static void cube_delete_cb(lv_event_t *e)
{
    lv_obj_t *cube = lv_event_get_target_obj(e);
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (d) { lv_free(d); lv_obj_set_user_data(cube, NULL); }
}

lv_obj_t *cube3d_create(lv_obj_t *parent)
{
    lv_obj_t *cube = lv_obj_create(parent);
    lv_obj_remove_style_all(cube);                 /* 透明、无边框、无背景 */
    lv_obj_clear_flag(cube, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_clear_flag(cube, LV_OBJ_FLAG_CLICKABLE);

    cube_data_t *d = (cube_data_t *)lv_malloc(sizeof(cube_data_t));
    if (d) {
        d->edge = 0; d->depth = 0; d->squash_pct = 0;
        d->c_top = 0xE8E8E8; d->c_front = 0xA0A0A0; d->c_right = 0xC0C0C0;
        d->stripe_mask = 0; d->pattern = 0; d->shadow = false;
        lv_obj_set_user_data(cube, d);
    }

    lv_obj_add_event_cb(cube, cube_draw_cb,   LV_EVENT_DRAW_MAIN, NULL);
    lv_obj_add_event_cb(cube, cube_delete_cb, LV_EVENT_DELETE,    NULL);
    return cube;
}

/* 重算包围盒：宽=2dx，高=2dy + depth + 影子余量。压扁只在盒内下沉，不缩盒。*/
static void cube_resize_box(lv_obj_t *cube, cube_data_t *d)
{
    int dx = cube_dx(d->edge);
    int dy = cube_dy(d->edge);
    int pad = d->shadow ? dy + 4 : 0;        /* 影子在底面下方，留余量 */
    lv_obj_set_size(cube, 2 * dx, 2 * dy + d->depth + pad);
}

void cube3d_set_geometry(lv_obj_t *cube, int edge_len)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->edge = edge_len;
    if (d->depth <= 0) d->depth = edge_len;  /* 默认厚度=边长（正立方体感）*/
    cube_resize_box(cube, d);
    lv_obj_invalidate(cube);
}

void cube3d_set_depth(lv_obj_t *cube, int depth)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->depth = depth < 2 ? 2 : depth;
    cube_resize_box(cube, d);
    lv_obj_invalidate(cube);
}

void cube3d_set_stripes(lv_obj_t *cube, uint8_t mask)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->stripe_mask = mask;
    lv_obj_invalidate(cube);
}

void cube3d_set_pattern(lv_obj_t *cube, uint8_t pattern)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->pattern = pattern;
    lv_obj_invalidate(cube);
}

void cube3d_set_shadow(lv_obj_t *cube, bool on)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->shadow = on;
    cube_resize_box(cube, d);
    lv_obj_invalidate(cube);
}

void cube3d_set_colors(lv_obj_t *cube, uint32_t top, uint32_t front, uint32_t right)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->c_top = top; d->c_front = front; d->c_right = right;
    lv_obj_invalidate(cube);
}

void cube3d_set_top_color(lv_obj_t *cube, uint32_t top_rgb)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    d->c_top   = top_rgb;
    d->c_right = cube_darken(top_rgb, 205);  /* 右侧面≈80% 亮度 */
    d->c_front = cube_darken(top_rgb, 150);  /* 正面≈59% 亮度（最暗）*/
    lv_obj_invalidate(cube);
}

void cube3d_set_squash(lv_obj_t *cube, int squash_pct)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d) return;
    if (squash_pct < 0)   squash_pct = 0;
    if (squash_pct > 100) squash_pct = 100;
    d->squash_pct = squash_pct;
    lv_obj_invalidate(cube);
}

void cube3d_place(lv_obj_t *cube, int cx, int top_y)
{
    cube_data_t *d = (cube_data_t *)lv_obj_get_user_data(cube);
    if (!d || d->edge <= 0) return;
    int dx = cube_dx(d->edge);
    int dy = cube_dy(d->edge);
    /* 锚点 = 「不压扁时的顶面中心」对齐屏幕 (cx, top_y)（小人脚踩这条线）。
     * 顶面中心在对象局部 (dx, dy)（sink=0 时 T 与 B 的中点）。
     * 故对象左上角 = (cx - dx, top_y - dy)。
     * 压扁(自上而下)只在对象盒内让顶面下沉，底面固定，top_y 不随 squash 漂移。*/
    lv_obj_set_pos(cube, cx - dx, top_y - dy);
    lv_obj_invalidate(cube);
}
