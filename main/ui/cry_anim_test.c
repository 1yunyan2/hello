/**
 * @file cry_anim_test.c
 * @brief 哭泣表情手绘循环动画（纯 LVGL canvas，无外部图片）
 */

#include "cry_anim_test.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "lvgl.h"

#define TAG "CryAnim"

/* ── 屏幕 ─────────────────────────────────────────── */
#define SCREEN_W 320
#define SCREEN_H 240
#define CX (SCREEN_W / 2)

/* ── 眼睛 ─────────────────────────────────────────── */
#define EYE_BASE_Y 80
#define EYE_HALF 38
#define EYE_V 20
#define EYE_GAP 62
#define LINE_W 3
#define LE_X (CX - EYE_GAP)
#define RE_X (CX + EYE_GAP)

static const int s_jitter[] = {0, 4, 5, 3, 0, -3, -5, -4};

/* ── 嘴巴 ^ ───────────────────────────────────────── */
#define MOUTH_Y (EYE_BASE_Y + 45)
#define MOUTH_HALF 22
#define MOUTH_V 12

/* ── 眼泪 ─────────────────────────────────────────── */
/* 水滴总高 = DROP_TRI_H + DROP_R*2 = 10+12 = 22px
 * TEAR_GAP 必须 > 22，这里取 45，间距 = 45-22 = 23px，清晰可见 */
#define DROP_R 6
#define DROP_TRI_H 10
#define DROP_H (DROP_TRI_H + DROP_R * 2) /* 22px */

#define TEAR_COUNT 4 /* 同时显示4滴，充满屏幕高度 */
#define TEAR_GAP 45  /* 相邻泪滴顶点间距（固定不变） */
#define TEAR_SPEED 7 /* 每帧下移 px */
#define TEAR_ORIGIN (EYE_BASE_Y + 26)

#define L_TEAR_X LE_X
#define R_TEAR_X RE_X

/* ── 帧率 ─────────────────────────────────────────── */
#define FRAME_MS 33 /* ~30fps */

/* ── 状态 ─────────────────────────────────────────── */
static lv_obj_t *s_canvas = NULL;
static uint16_t *s_cbuf = NULL;
static lv_timer_t *s_timer = NULL;
static int s_frame = 0;

/* 单一偏移驱动，所有泪滴间距永远固定
 * 第 i 滴的顶点 y = TEAR_ORIGIN + offset + i*TEAR_GAP
 * offset 每帧 +TEAR_SPEED，超过 TEAR_GAP 则取模，无限循环无断点 */
static int s_offset_l = 0;
static int s_offset_r = 0; /* 右眼初始错开半个间距 */

/* ── 画粗线 ───────────────────────────────────────── */
static void draw_line(lv_layer_t *layer, int x1, int y1, int x2, int y2, int w)
{
    lv_draw_line_dsc_t dsc;
    lv_draw_line_dsc_init(&dsc);
    dsc.color = lv_color_white();
    dsc.width = w;
    dsc.opa = LV_OPA_COVER;
    dsc.round_start = 1;
    dsc.round_end = 1;
    dsc.p1.x = x1;
    dsc.p1.y = y1;
    dsc.p2.x = x2;
    dsc.p2.y = y2;
    lv_draw_line(layer, &dsc);
}

/* ── 画实心圆 ─────────────────────────────────────── */
static void draw_circle(lv_layer_t *layer, int cx, int cy, int r)
{
    lv_draw_arc_dsc_t dsc;
    lv_draw_arc_dsc_init(&dsc);
    dsc.color = lv_color_white();
    dsc.opa = LV_OPA_COVER;
    dsc.width = r;
    dsc.radius = r;
    dsc.start_angle = 0;
    dsc.end_angle = 360;
    dsc.center.x = cx;
    dsc.center.y = cy;
    lv_draw_arc(layer, &dsc);
}

/* ── 画水滴（倒三角尖朝上 + 圆球无缝贴合）────────── */
static void draw_teardrop(lv_layer_t *layer, int tx, int ty)
{
    int bot_y = ty + DROP_TRI_H;

    lv_draw_triangle_dsc_t tri;
    lv_draw_triangle_dsc_init(&tri);
    tri.color = lv_color_white();
    tri.opa = LV_OPA_COVER;
    tri.p[0].x = tx;
    tri.p[0].y = ty;
    tri.p[1].x = tx - DROP_R;
    tri.p[1].y = bot_y;
    tri.p[2].x = tx + DROP_R;
    tri.p[2].y = bot_y;
    lv_draw_triangle(layer, &tri);

        draw_circle(layer, tx, bot_y + DROP_R - 4, DROP_R);
}

/* ── 画 > 左眼 ────────────────────────────────────── */
static void draw_left_eye(lv_layer_t *layer, int tip_y)
{
    draw_line(layer, LE_X - EYE_HALF, tip_y - EYE_V, LE_X, tip_y, LINE_W);
    draw_line(layer, LE_X, tip_y, LE_X - EYE_HALF, tip_y + EYE_V, LINE_W);
}

/* ── 画 < 右眼 ────────────────────────────────────── */
static void draw_right_eye(lv_layer_t *layer, int tip_y)
{
    draw_line(layer, RE_X + EYE_HALF, tip_y - EYE_V, RE_X, tip_y, LINE_W);
    draw_line(layer, RE_X, tip_y, RE_X + EYE_HALF, tip_y + EYE_V, LINE_W);
}

/* ── 画 ^ 嘴巴 ────────────────────────────────────── */
static void draw_mouth(lv_layer_t *layer)
{
    draw_line(layer, CX - MOUTH_HALF, MOUTH_Y + MOUTH_V, CX, MOUTH_Y, LINE_W);
    draw_line(layer, CX, MOUTH_Y, CX + MOUTH_HALF, MOUTH_Y + MOUTH_V, LINE_W);
}

/* ── 每帧回调 ─────────────────────────────────────── */
static void cry_anim_cb(lv_timer_t *t)
{
    if (!lvgl_port_lock(10))
        return;

    lv_canvas_fill_bg(s_canvas, lv_color_black(), LV_OPA_COVER);

    lv_layer_t layer;
    lv_canvas_init_layer(s_canvas, &layer);

    /* 眼睛抖动 */
    int eye_y = EYE_BASE_Y + s_jitter[s_frame % 8];
    draw_left_eye(&layer, eye_y);
    draw_right_eye(&layer, eye_y);
    draw_mouth(&layer);

    /* 画左侧泪滴：i 滴顶点 y = TEAR_ORIGIN + s_offset_l + i*TEAR_GAP */
    for (int i = 0; i < TEAR_COUNT; i++)
    {
        int ty = TEAR_ORIGIN + s_offset_l + i * TEAR_GAP;
        /* 只画在屏幕内的（顶点还没出底部）*/
        if (ty < SCREEN_H)
            draw_teardrop(&layer, L_TEAR_X, ty);
    }

    /* 画右侧泪滴 */
    for (int i = 0; i < TEAR_COUNT; i++)
    {
        int ty = TEAR_ORIGIN + s_offset_r + i * TEAR_GAP;
        if (ty < SCREEN_H)
            draw_teardrop(&layer, R_TEAR_X, ty);
    }

    lv_canvas_finish_layer(s_canvas, &layer);

    /* 偏移推进，取模保持在 [0, TEAR_GAP) 内，实现无缝循环 */
    s_offset_l = (s_offset_l + TEAR_SPEED) % TEAR_GAP;
    s_offset_r = (s_offset_r + TEAR_SPEED) % TEAR_GAP;

    s_frame++;
    lvgl_port_unlock();
}

/* ── 公开入口 ─────────────────────────────────────── */
void cry_anim_test_start(void)
{
    s_offset_l = 0;
    s_offset_r = TEAR_GAP / 2; /* 右眼错开半个间距，不与左眼同步 */

    size_t buf_size = SCREEN_W * SCREEN_H * sizeof(uint16_t);
    s_cbuf = (uint16_t *)heap_caps_malloc(buf_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_cbuf)
    {
        ESP_LOGE(TAG, "canvas buffer 分配失败");
        return;
    }

    if (!lvgl_port_lock(1000))
    {
        ESP_LOGE(TAG, "LVGL lock 超时");
        return;
    }

    lv_obj_t *scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);

    s_canvas = lv_canvas_create(scr);
    lv_canvas_set_buffer(s_canvas, s_cbuf, SCREEN_W, SCREEN_H, LV_COLOR_FORMAT_RGB565);
    lv_canvas_fill_bg(s_canvas, lv_color_black(), LV_OPA_COVER);
    lv_obj_align(s_canvas, LV_ALIGN_CENTER, 0, 0);

    lvgl_port_unlock();

    s_timer = lv_timer_create(cry_anim_cb, FRAME_MS, NULL);
    ESP_LOGI(TAG, "哭泣动画启动");
}
