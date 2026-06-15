#pragma once

/**
 * @file race_sprites.h
 * @brief 赛车（Racing）游戏素材接口 + 玩法参数
 *
 * 横向赛道：320×240 横屏，上下两条车道，敌车从右向左跑。
 *
 * 素材映射（全部 RGB565A8 格式，定义在 s1.c~s5.c）：
 *   s1 = 背景路面（320×240 全屏）
 *   s2 = 玩家车（40×30）
 *   s3/s4/s5 = 敌方车（40×30 / 40×30 / 40×27，随机选用，颜色不同更好看）
 *
 * 后续替换真实素材时，只需重转 s1.c~s5.c（保持变量名 s1~s5 不变），
 * 并据实更新下方尺寸常量即可。
 *
 * LVGL 图片转换器: https://lvgl.io/tools/imageconverter
 * 设置: Color format = LV_COLOR_FORMAT_RGB565A8, Output = C array
 */

#include "lvgl.h"

/* ═══════════════════════════════════════════════════════════════
 * 背景来源开关（仅针对全屏背景 s1，编译期二选一）
 *
 * s1 是 320×240 RGB565A8（≈225KB），编进 app 会撑爆分区，所以单独做开关；
 * 车辆 s2~s5 很小（40×30，几 KB），始终用图片，正常编进 app。
 *
 *   RACE_BG_USE_IMG = 0（默认）：
 *     背景用 LVGL 画色块（深灰路面 + 上下两侧绿化带），零 flash 占用。
 *     当前用于验证游戏逻辑——不必准备背景大图，固件不会超分区。
 *
 *   RACE_BG_USE_IMG = 1：
 *     背景用图片 s1（编进 app）。注意此时需保证分区放得下，
 *     或把 s1 改放外挂 flash 用路径加载（见下方 RACE_IMG_BG）。
 * ═══════════════════════════════════════════════════════════════ */
/* 背景 s1 已改为从外挂 flash 读 "S:/img/s1.bin"（见 game_race.c），
 * 不再编进 app、不再需要 s1 符号，故这里不再 extern s1。
 * 默认开启图片背景（=1）：背景从 flash 加载，不占 app 分区。 */
#ifndef RACE_BG_USE_IMG
#define RACE_BG_USE_IMG 1
#endif

/* 车辆素材：始终用图片，体积小，正常编进 app（定义在 s2.c~s5.c）。
 * 当前为临时占位尺寸（40×30 / 40×27），后续替换真实素材时保持变量名不变。 */
extern const lv_image_dsc_t s2; /* 玩家车 40×30 */
extern const lv_image_dsc_t s3; /* 敌车 A 40×30 */
extern const lv_image_dsc_t s4; /* 敌车 B 40×30 */
extern const lv_image_dsc_t s5; /* 敌车 C 40×27 */

/* ── 素材尺寸（与实际图片一致，用于布局/碰撞计算，不放大）── */
#define RACE_CAR_W 40 /* 车宽（玩家/敌车统一按 40 计算碰撞框）*/
#define RACE_CAR_H 30 /* 车高（按 s2 的 30；s5 略矮 27，不影响矩形碰撞）*/

/* ═══════════════════════════════════════════════════════════════
 * 布局参数（320×240 横屏，竖向赛道 —— 敌车从上往下落）
 *   屏幕宽 320、高 240；左右两条车道竖向排开。
 *   车原始尺寸 40×30；左车道 x=80、右车道 x=200，分隔线居中 x=160。
 *   玩家车固定贴底部，敌车从顶部屏外进入向下移动。
 * ═══════════════════════════════════════════════════════════════ */
#define RACE_LANE_LEFT_X   80  /* 左车道：车左上角 x（车中心 ≈100）*/
#define RACE_LANE_RIGHT_X  200 /* 右车道：车左上角 x（车中心 ≈220）*/
#define RACE_DIVIDER_X     160 /* 中间白色竖向分隔虚线的 x（两车道正中）*/

#define RACE_PLAYER_Y      200 /* 玩家车固定 y（贴底部，车高30→底缘230）*/
#define RACE_ENEMY_SPAWN_Y (-RACE_CAR_H) /* 敌车进入 y（顶部屏外起点）*/
#define RACE_ENEMY_GONE_Y  BSP_LCD_HEIGHT /* 敌车移到此 y(=240)算躲过 +1 分 */

/* 注：赛车无时间限制，撞车才结束；速度随时间持续加快，无 RACE_GAME_SECONDS。*/

/* ═══════════════════════════════════════════════════════════════
 * 玩法参数（全部宏化，方便统一调参）
 *
 * 三档难度（简单 / 一般 / 困难）决定两件事：
 *   - 敌车基准速度 SPEED_*：每帧（ENGINE_MS=50ms）左移像素，越大越快
 *   - 敌车生成间隔 SPAWN_MS_*：两辆敌车之间的等待，越短越密集
 * 数值为「档位基准」，运行中再叠加「10 秒加速」（同打地鼠）。
 *
 * 速度换算：起点 x=320 → 终点 x=-40，总行程 360px。
 *   speed=6px/帧 × 20fps = 120px/s → 360/120 ≈ 3.0s 跑完一趟。
 *   speed=16px/帧 → 360/320 ≈ 1.1s 跑完一趟（最难时）。
 * ═══════════════════════════════════════════════════════════════ */

/* ── 难度档：简单 ── */
#define RACE_SPEED_EASY      6   /* 敌车 6px/帧（约 3s 过屏）*/
#define RACE_SPAWN_MS_EASY   1300 /* 生成间隔 1.3s */

/* ── 难度档：一般 ── */
#define RACE_SPEED_NORMAL    9   /* 敌车 9px/帧（约 2s 过屏）*/
#define RACE_SPAWN_MS_NORMAL 1000 /* 生成间隔 1.0s */

/* ── 难度档：困难 ── */
#define RACE_SPEED_HARD      12  /* 敌车 12px/帧（约 1.5s 过屏）*/
#define RACE_SPAWN_MS_HARD   700  /* 生成间隔 0.7s */

/* ── 动态加速：每 10 秒，速度 +量、生成间隔 ×百分比（同打地鼠思路）── */
#define RACE_ACCEL_EVERY_S   10 /* 每 10 秒提速一次 */
#define RACE_ACCEL_SPEED_ADD 2  /* 每次速度 +2px/帧 */
#define RACE_ACCEL_SPAWN_PCT 80 /* 每次生成间隔取原值 80% */
#define RACE_SPEED_MAX       16 /* 敌车速度上限（约 1.1s 过屏）*/
#define RACE_SPAWN_MS_MIN    450 /* 生成间隔下限，防止挤成一团 */

/* ── 敌车对象池 ── */
#define RACE_MAX_ENEMIES     3  /* 同屏最多 3 辆敌车（预分配复用）*/

/* ── 触摸防误触 ── */
#define RACE_TOUCH_COOLDOWN_MS 200 /* 换道后 200ms 内不再响应，防连跳 */

/* ── 碰撞闪烁 ── */
#define RACE_FLASH_MS        200 /* 碰撞后红屏闪烁时长（ms）*/

/* ── 分隔虚线滚动 ── */
#define RACE_DASH_LEN        24 /* 虚线段长（px）*/
#define RACE_DASH_GAP        20 /* 虚线段间隔（px）*/
#define RACE_DASH_SPEED_DIV  2  /* 虚线滚动速度 = 敌车速度 / 该值（越快越带感）*/

/* ── 倒计时 ── */
#define RACE_COUNTDOWN_FROM    3   /* 3 → 2 → 1 → GO! */
#define RACE_COUNTDOWN_STEP_MS 700 /* 每个数字停留毫秒 */
