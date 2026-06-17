#pragma once

/**
 * @file whack_sprites.h
 * @brief 打地鼠游戏素材接口
 *
 * 当前使用 p2~p6 作为占位素材（全部 RGB565A8 格式）：
 *   p2 = 地鼠正常（118×58）
 *   p3 = 地鼠被打（103×41）
 *   p4 = 地洞（118×58）
 *   p5 = 锤子（118×58）
 *   p6 = 星星（118×58）
 *
 * 后续替换真实素材时，只需：
 *   1. 用 LVGL 图片转换器把 PNG 转为 RGB565A8 C 数组
 *   2. 替换 p2.c~p6.c 的内容（保持变量名不变）
 *   3. 更新下方尺寸常量
 *
 * LVGL 图片转换器: https://lvgl.io/tools/imageconverter
 * 设置: Color format = LV_COLOR_FORMAT_RGB565A8, Output = C array
 */

#include "lvgl.h"

/* ── 素材尺寸（与实际图片一致，用于布局计算）── */
#define WHACK_MOLE_W 100     /* ds2_mole 缩放后宽 */
#define WHACK_MOLE_H 117     /* ds2_mole 缩放后高 */
#define WHACK_MOLE_HIT_W 100 // 被打（ds1，与正常态等比同尺寸）
#define WHACK_MOLE_HIT_H 117 // 被打（ds1，与正常态等比同尺寸）
#define WHACK_HOLE_W 118     // 洞口宽（背景图内洞口实际宽度，非等分，由像素实测）
#define WHACK_HOLE_H 58      // 洞口高（背景图内洞口实际高度，由像素实测）
#define WHACK_HAMMER_W 80    // 锤子宽（ds3 实际宽度）
#define WHACK_HAMMER_H 135   // 锤子高（ds3 实际高度）
/* ds3 锤子图布局：【锤头在图上半部，锤柄在图下半部】（像素实测：上半不透明像素更密）。
 * 锤头中心距【图顶】的纵向距离（px）。用于把"锤头"而非"锤柄"对齐到地鼠头。
 * ★锤头实际落点偏低（锤柄方向）就调大，偏高就调小 */
#define WHACK_HAMMER_HEAD_OFFSET 30
/* 锤头落点相对【地鼠头顶】的纵向微调（正=往下砸进去一点，负=悬在上方）*/
#define WHACK_HAMMER_HIT_DY 8
/* 锤头相对地鼠中心的水平偏移（正=往右，让锤子斜砸更自然）*/
#define WHACK_HAMMER_HIT_DX 10
#define WHACK_STAR_W 118 // 星星宽（p6 实际宽度）
#define WHACK_STAR_H 58  // 星星高（p6 实际高度）

/* ── 素材声明 ── */
extern const lv_image_dsc_t ds2_mole; /* 地鼠正常（100×117, RGB565A8）*/
#define p2 ds2_mole                   /* 保持 game_whack.c 内部引用名不变 */
extern const lv_image_dsc_t ds1;      /* 地鼠被打/打扁（100×117, RGB565A8，与正常态等比同尺寸）*/
extern const lv_image_dsc_t p3;       /* 地鼠被打（旧占位，已由 ds1 取代）*/
extern const lv_image_dsc_t p4;       /* 地洞（仅作布局参考，背景已含地洞图形）*/
extern const lv_image_dsc_t ds3;      /* 锤子（80×135, RGB565A8）*/
extern const lv_image_dsc_t p6;       /* 星星 */

/* ═══════════════════════════════════════════════════════════════
 * 背景绘制方式开关（二选一）
 *
 *   WHACK_BG_MODE_COLOR  = 0  用 LVGL 纯色填充（无需素材，省 flash/RAM）
 *   WHACK_BG_MODE_IMAGE  = 1  铺一张全屏背景素材图（更好看，占 flash）
 *
 * 把 WHACK_BG_MODE 设为其中之一即可切换，game_whack.c 的 build_panel() 据此分支。
 * 选 IMAGE 时：
 *   - 背景图变量见下方 WHACK_BG_IMAGE（默认复用 picture1，可换成自己的全屏图）
 *   - 建议尺寸 = 屏幕分辨率(BSP_LCD_WIDTH × BSP_LCD_HEIGHT)，RGB565 即可（背景无需 A8）
 * ═══════════════════════════════════════════════════════════════ */
#define WHACK_BG_MODE_COLOR 0 // 0 = 纯色模式（默认），1 = 素材模式
#define WHACK_BG_MODE_IMAGE 1 // 1 = 素材模式（默认）

/* ★ 在此切换背景绘制方式（默认纯色）★ */
#define WHACK_BG_MODE WHACK_BG_MODE_IMAGE

/* 纯色模式下的背景颜色（草地绿）*/
#define WHACK_BG_COLOR 0x2D5A27

#if WHACK_BG_MODE == WHACK_BG_MODE_IMAGE
/* ds_bg = ds.png 转换而来（320×240 RGB565A8），地洞已合成在背景图内 */
extern const lv_image_dsc_t ds_bg;
#define WHACK_BG_IMAGE ds_bg
#endif

/* ═══════════════════════════════════════════════════════════════
 * 玩法参数（全部宏化，方便统一调参）
 *
 * 三档难度（简单 / 一般 / 困难）只改三个变量：
 *   - 地鼠停留时间 MOLE_LIFE_MS_*：冒出后多久没被打就缩回（越短越难）
 *   - 出现间隔     SPAWN_MS_*    ：两只地鼠之间的等待（越短越密集）
 *   - 出洞速度     RISE_PX_*     ：每帧上移像素（越大冒得越快）
 * 数值为「档位基准」，运行中再叠加「10 秒加速」。
 * ═══════════════════════════════════════════════════════════════ */

/* ── 一局时长 ── */
#define WHACK_GAME_SECONDS 30 /* 单局 30 秒 */

/* 说明：引擎帧周期 ENGINE_MS=50（20fps），地鼠从洞里钻出需爬升约 WHACK_HOLE_H(58) 像素。
 * 「完全冒出耗时 ≈ 58 / RISE_PX × 50ms」。RISE_PX 越大冒得越快、越干脆。
 *   12px → ≈0.25s   16px → ≈0.18s   20px → ≈0.15s（近乎弹出）。 */

/* ── 难度档：简单 ── */
#define WHACK_LIFE_MS_EASY 533  /* 停留 ~0.53s（原1.6s×1/3） */
#define WHACK_SPAWN_MS_EASY 333 /* 间隔 ~0.33s（原1.0s×1/3） */
#define WHACK_RISE_PX_EASY 36   /* 出洞 ≈0.08s（原12px×3，近乎弹出） */

/* ── 难度档：一般 ── */
#define WHACK_LIFE_MS_NORMAL 400  /* 停留 ~0.4s（原1.2s×1/3） */
#define WHACK_SPAWN_MS_NORMAL 267 /* 间隔 ~0.27s（原0.8s×1/3） */
#define WHACK_RISE_PX_NORMAL 48   /* 出洞 ≈0.06s（原16px×3） */

/* ── 难度档：困难 ── */
#define WHACK_LIFE_MS_HARD 267  /* 停留 ~0.27s（原0.8s×1/3） */
#define WHACK_SPAWN_MS_HARD 200 /* 间隔 0.2s（原0.6s×1/3，取下限） */
#define WHACK_RISE_PX_HARD 60   /* 出洞 ≈0.04s（原20px×3，瞬现） */

/* ── 动态加速：每 10 秒，停留时间与出现间隔各乘以该因子（百分比）── */
#define WHACK_ACCEL_EVERY_S 10 /* 每 10 秒提速一次 */
#define WHACK_ACCEL_PERCENT 80 /* 提速后取原值的 80% */
#define WHACK_LIFE_MS_MIN 67   /* 停留时间下限（原200ms×1/3） */
#define WHACK_SPAWN_MS_MIN 67  /* 出现间隔下限（原200ms×1/3） */

/* ── 入洞速度（缩回，与档位无关，固定手感）── */
#define WHACK_FALL_PX 30 /* 缩回 30 px/帧（原10px×3，约0.1s缩回） */

/* ── 命中表现 ── */
#define WHACK_HIT_SHOW_MS 500    /* 被打后扁掉(ds1)停留时长：砸中→打扁停顿 0.5s→再下洞消失 */
#define WHACK_HIT_BONUS_MS 1000  /* 砸中地鼠奖励时间：时间条 +1 秒（封顶满格 30s）*/
#define WHACK_HAMMER_SHOW_MS 120 /* 锤子击打动作停留时长（每次敲击都显示，到时收回）*/

/* ── 空敲惩罚 ── */
#define WHACK_MISS_PENALTY 1 /* 敲到空洞扣 1 分（最低不低于 0）*/

/* ── 开场动画（时间条耗完→回满，替代旧 3-2-1-GO 数字倒计时）── */
#define WHACK_INTRO_MS 3000 /* 开场：时间条从满 3 秒耗到空，随后回满即开打 */

/* ── 时间条（顶部缓慢消失的剩余时间可视化）── */
#define WHACK_TIMEBAR_H 16             /* 时间条高度 px */
#define WHACK_TIMEBAR_MARGIN 8         /* 时间条距屏幕左右/顶部的边距 px */
#define WHACK_TIMEBAR_RADIUS 8         /* 圆角半径 */
#define WHACK_TIMEBAR_BG_COLOR 0x16213E   /* 轨道底色（深蓝）*/
#define WHACK_TIMEBAR_OK_COLOR 0x00D466   /* 正常（绿）*/
#define WHACK_TIMEBAR_WARN_COLOR 0xFFA502 /* 警告：剩余≤1/3（橙）*/
#define WHACK_TIMEBAR_DANGER_COLOR 0xFF3B30 /* 危险：剩余≤1/6（红）*/

/* 旧难度数字倒计时（已由时间条开场动画取代，保留宏以防引用）*/
#define WHACK_COUNTDOWN_FROM 3       /* 3 → 2 → 1 → GO! */
#define WHACK_COUNTDOWN_STEP_MS 1000 /* 每个数字停留毫秒 */
