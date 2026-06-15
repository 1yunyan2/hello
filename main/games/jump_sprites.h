#pragma once

/**
 * @file jump_sprites.h
 * @brief 跳一跳游戏参数
 */

#include "lvgl.h"

/* ── 背景 ── */
#define JUMP_BG_USE_IMG 0
#define JUMP_BG_COLOR 0xC8CDD8 /* 浅蓝灰 */

#if JUMP_BG_USE_IMG
extern const lv_image_dsc_t picture1;
#define JUMP_BG_IMAGE picture1
#endif

/* ── 引擎帧率 ── */
#define JUMP_ENGINE_MS 33 /* ≈30fps */

/* ── 小人（图片 j4，动态等比缩放 + 蓄力纵向压扁）──
 * j4 原图 62×47，RGB565A8 带 alpha。用 lv_image 显示。
 * 缩放为「动态」：棋子宽 = 当前所站台宽的 1/3（台大棋大、台小棋保底），
 *   见 game_jump.c player_base_scale()。蓄力时只压 scale_y（纵向压扁）。
 * 换素材只需替换 j4.c 并改下面 IMG_W/H 尺寸。*/
extern const lv_image_dsc_t j4;
#define JUMP_PLAYER_IMG j4
#define JUMP_PLAYER_IMG_W 62 /* 原图宽 */
#define JUMP_PLAYER_IMG_H 47 /* 原图高 */

/* 动态缩放参数 */
#define JUMP_PLAYER_MIN_W 20        /* 棋子宽下限 px（小台子上不至于太小看不清）*/
#define JUMP_PLAYER_SQUASH_RATIO 40 /* 满蓄力时 scale_y 压到基准的百分比（40%=压得更扁，蓄力更明显）*/

/* 近似逻辑尺寸（仅用于光斑中心/出屏阈值等粗略计算，非精确像素）。
 * 取一个代表性高度：约最小棋子宽对应的高（j4 比例 47/62）。*/
#define JUMP_PLAYER_H (JUMP_PLAYER_MIN_W * JUMP_PLAYER_IMG_H / JUMP_PLAYER_IMG_W) /* ≈15 */

/* ── 立体台子（LVGL 纯代码画：顶面亮 + 侧面暗，2.5D）──
 * 顶面是一个圆角矩形（亮色），其正下方叠一层稍暗的侧面矩形当“厚度”，产生立体感。
 * 后期 5~6 种不同大小/颜色只需扩 s_plat_styles[] 表，无需改渲染逻辑。*/
#define JUMP_TOP_Y 145          /* 顶面顶边 y（小人脚踩这条线）*/
#define JUMP_PLATFORM_H 22      /* 顶面厚度（圆角矩形高）*/
#define JUMP_PLATFORM_SIDE_H 14 /* 侧面（立体厚度）高度 */
#define JUMP_PLATFORM_RADIUS 6  /* 顶面圆角 */
/* 顶面色随分数/随机从样式表取，侧面色 = 顶面色调暗，下面是默认 fallback */
#define JUMP_PLATFORM_COLOR 0xE8E8E8  /* 当前台默认顶面色（浅灰白）*/
#define JUMP_PLATFORM_COLOR2 0xD4EAF7 /* 下一台默认顶面色（浅蓝白）*/
#define JUMP_MAX_PLATFORMS 2          /* 场上台子总数：cur(0) + next(1) */

/* 台子样式表条目数（顶面色，侧面色由代码按比例调暗自动生成）。
 * 后期要更多“五六种大小/颜色”就往 game_jump.c 的 s_plat_styles[] 里加。*/
#define JUMP_PLAT_STYLE_COUNT 5

/* 起跳基准：当前台中心固定在此 x */
#define JUMP_START_X 80

/* ── 难度 ──（初始台宽放大约 0.3 倍：64→83 / 50→65 / 38→50）*/
#define JUMP_PW_EASY 83
#define JUMP_GAP_MIN_EASY 70
#define JUMP_GAP_MAX_EASY 120

#define JUMP_PW_NORMAL 65
#define JUMP_GAP_MIN_NORMAL 80
#define JUMP_GAP_MAX_NORMAL 150

#define JUMP_PW_HARD 50
#define JUMP_GAP_MIN_HARD 90
#define JUMP_GAP_MAX_HARD 180

/* 越往后台子越小：每得 1 分台宽缩 2px（原 1px，放大初始值后加快递减），下限 26 */
#define JUMP_PW_SHRINK_PER_SCORE 2
#define JUMP_PW_MIN 26

/* 相邻台子之间的最小可见缝隙 px（中心距 ≥ 两台半宽和 + 此值，防止台子重叠）*/
#define JUMP_PLAT_MIN_SPACING 18

/* ── 蓄力→跳跃距离 ── */
#define JUMP_HOLD_MIN_MS 150
#define JUMP_HOLD_MAX_MS 1400
#define JUMP_DIST_MIN_PX 55
#define JUMP_DIST_MAX_PX 200

/* ── 台子 Y 轴随机偏移（双轴散落效果）── */
#define JUMP_GAP_Y_MAX 50 /* 下一台相对当前台 Y 轴最大偏移 px（上下各±50）*/

/* ── 抛物线（二维）── */
#define JUMP_APEX_RATIO 40 /* 弧顶高 = 直线距离 × 40% */
#define JUMP_FLY_STEP_PX 9 /* 每帧沿飞行方向推进 px（值越大跳越快；6→8 比半速快约1/3）*/

/* ── 摄像机平滑（落台后滑动）── */
#define JUMP_CAM_SMOOTH_FRAMES 12 /* 平滑帧数，约0.4s @30fps */

/* ── 台子从天而降入场动画 ── */
#define JUMP_DROP_HEIGHT 90      /* 入场起始离目标的高度 px（从这么高掉下来）*/
#define JUMP_DROP_GRAVITY 3      /* 掉落重力加速度（每帧 drop_vy += 此值）*/
#define JUMP_DROP_BOUNCE 45      /* 触底回弹初始压扁量（0~100，越大弹得越狠）*/
#define JUMP_DROP_BOUNCE_DECAY 8 /* 回弹压扁每帧衰减量（回弹消退速度）*/

/* ── 掉落 ── */
#define JUMP_FALL_STEP_PX 10

/* ── 蓄力压扁（台子）── */
#define JUMP_PLATFORM_SQUASH_MAX_PX 18
