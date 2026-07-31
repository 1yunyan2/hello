#pragma once

/**
 * @file race_sprites.h
 * @brief 赛车（Racing）游戏素材接口 + 玩法参数（三车道·透视版）
 *
 * 素材分两处放（按大小分流，和原来 s1/s2~s5 同套路）：
 *   c1~c6 障碍 + c7 玩家 = C 数组编进 app（c1.c~c7.c）——小图，且要实时缩放，
 *     必须放内存：LVGL 缩放需要完整内存位图，外挂文件图边读边解码无法缩放
 *     （会渲染成彩色条纹），所以障碍/玩家一律进固件。
 *   c8 背景（320×240，大）= 留外挂 Flash "S:/img/c8.bin"，不缩放、只整屏铺底，
 *     文件图不缩放可正常显示；放外挂避免撑爆 app 分区。
 *
 * 透视模型（详见 game_race.c 顶部注释）：
 *   障碍物用进度 t(0=远端消失点附近 → 1000=近端屏幕底缘) 驱动，
 *   每帧 t 增加，同时插值 {纵向 y、车道横向 x、缩放}，模拟近大远小。
 *   关键常量取自 c8 背景实测（屏幕坐标，320×240）。
 *
 * 改图后：c1~c7 改了重新 idf.py build（C 数组随固件烧）；c8 改了才需重打包烧外挂。
 * 注意：读 c8 需 LVGL 线程上下文（race_* 接口已加锁，安全）。
 */

#include "lvgl.h"

/* ── 障碍 + 玩家素材（C 数组，编进 app，定义在 c1.c~c7.c）── */
extern const lv_image_dsc_t c1; /* 48×35  红黑✕方块 */
extern const lv_image_dsc_t c2; /* 60×36  金属滚轴 */
extern const lv_image_dsc_t c3; /* 88×45  ⚠警告地板 */
extern const lv_image_dsc_t c4; /* 41×20  黑色矮栏 */
extern const lv_image_dsc_t c5; /* 134×63 双塔激光门 */
extern const lv_image_dsc_t c6; /* 44×39  红黑刺球 */
extern const lv_image_dsc_t c7; /* 78×115 玩家（机器人）*/

/* ── 背景素材路径（外挂 Flash，挂载点 S:，不缩放只铺底）── */
#define RACE_IMG_C8 "S:/img/c8.bin" /* 320×240 背景（三车道透视）*/

#define RACE_OBST_COUNT 6 /* 障碍物种类数 c1~c6 */

/* ═══════════════════════════════════════════════════════════════
 * 三车道（透视）
 * ═══════════════════════════════════════════════════════════════ */
#define LANE_LEFT 0
#define LANE_MID 1
#define LANE_RIGHT 2
#define LANE_COUNT 3

/* ── 一点透视·直线车道（中心对称，消失点在屏幕中心 x≈160）──
 * 障碍进度 t：0=远端(门洞 y=RACE_FAR_Y) → 1000=近端(底缘 y=RACE_NEAR_Y)。
 * 三条平行车道投影到屏幕 = 收束于中心消失点的直线：中道垂直恒 160，左右对称斜线。
 * 每条道「远端/近端」两点线性插值（直线，绝不拐弧），配合二次速度+缩放+淡入。
 * 全部宏定义，改这里即可微调；坐标提取自正拍原图 c8（320×240 基准）。*/
#define RACE_FAR_Y 70   /* 远端 y：三道在门洞处仍可分辨的高度 */
#define RACE_NEAR_Y 240 /* 近端 y：屏幕底缘 */

/* 三条车道中心 x：远端(门洞处) / 近端(底缘处)。中道恒 160；左右对称。
 * 远端越靠近 160=越收拢；近端 L 越小/R 越大=底部越张开。改这 6 个数即可贴合白线。*/
#define RACE_FAR_L_X 145
#define RACE_NEAR_L_X 38
#define RACE_FAR_R_X 177
#define RACE_NEAR_R_X 288
#define RACE_FAR_M_X 160
#define RACE_NEAR_M_X 160

/* 缩放：远端 = 近端尺寸 × RACE_FAR_SCALE/1000，近端 = 1.0×（各障碍 s_obst_near_scale）。
 * ★「由小变大」的关键★ 数值越小，障碍远处越小、变大越夸张（当前 0.12×→1.0×）。*/
#define RACE_FAR_SCALE 80

/* 门洞淡入：t<RACE_FADE_T 时透明度 0→255（消除门洞处凭空蹦出）。*/
#define RACE_FADE_T 150

/* 碰撞判定线：障碍中心屏幕 y 越过此值时与玩家比对车道（≈玩家身体所在深度）。*/
#define RACE_COLLIDE_Y 200

/* ═══════════════════════════════════════════════════════════════
 * 玩家（固定贴底，3 车道横切）
 * ═══════════════════════════════════════════════════════════════ */
/* 玩家三车道中心 x：与碰撞判定线 RACE_COLLIDE_Y 处的车道中心对齐，
 * 改为 {80,160,240} 让棋子在各车道内居中，不贴屏幕边缘。*/
#define RACE_PLAYER_LANE_L 40
#define RACE_PLAYER_LANE_M 160
#define RACE_PLAYER_LANE_R 285
#define RACE_PLAYER_W 78  /* c7 原始宽 */
#define RACE_PLAYER_H 115 /* c7 原始高 */
/* 玩家缩放（LVGL scale 单位：256=1.0×）; 200 ≈ 0.78× → 显示宽≈61px 高≈90px */
#define RACE_PLAYER_SCALE 230
/* 缩放后高 ≈ 90px，顶边 y 使下 1/5(≈18px) 落屏外：240-90+18=168 */
#define RACE_PLAYER_TOP_Y 157

/* ═══════════════════════════════════════════════════════════════
 * 玩法参数（全部宏化）
 *
 * 速度单位 = 每帧(ENGINE_MS=50ms, 20fps) 的 t 增量(千分比)。
 *   1000 / speed ≈ 障碍物从远跑到近所需帧数；÷20 ≈ 秒数。
 *   speed=18 → 约 56 帧 ≈ 2.8s 过屏；speed=34 → 约 30 帧 ≈ 1.5s。
 * 运行中再叠加「每 10 秒加速」。
 * ═══════════════════════════════════════════════════════════════ */
#define RACE_SPEED_EASY 18
#define RACE_SPAWN_MS_EASY 1300

#define RACE_SPEED_NORMAL 26
#define RACE_SPAWN_MS_NORMAL 1000

#define RACE_SPEED_HARD 34
#define RACE_SPAWN_MS_HARD 750

/* 动态加速（同打地鼠思路）：每 10 秒，速度 +量(带上限)、生成间隔 ×百分比(带下限)。*/
#define RACE_ACCEL_EVERY_S 10
#define RACE_ACCEL_SPEED_ADD 4
#define RACE_ACCEL_SPAWN_PCT 80
#define RACE_SPEED_MAX 60
#define RACE_SPAWN_MS_MIN 450

/* 障碍物对象池：同屏最多 3 个（预分配复用）。*/
#define RACE_MAX_ENEMIES 3
/* 防叠：某车道若已有 t<此值(刚出生)的障碍，则本拍不在该车道再生成，
 * 同时保证不会三道同时被堵死（至少留一条逃生道）。*/
#define RACE_SPAWN_SAFE_T 220

/* ── 换道灵敏度 ──
 * 两次换道之间的强制冷却：这段时间内的耳朵事件直接丢弃。
 * 越小 = 挪移越跟手、可连续快速换道；越大 = 越不容易误触但手感发钝。
 * 参考：180ms 偏保守（一秒最多换 5 次）；90ms 手感明显更跟手；
 *       0 = 不限速，完全跟随触摸事件（受 bsp_touch.c 扫描/消抖限制，约 75ms 一次）。*/
#define RACE_TOUCH_COOLDOWN_MS 10 /* 换道防误触冷却（ms），调这一个即可 */
#define RACE_FLASH_MS 200         /* 撞车红屏闪烁时长 */

#define RACE_COUNTDOWN_FROM 3
#define RACE_COUNTDOWN_STEP_MS 700

/* ═══════════════════════════════════════════════════════════════
 * 开场进度条（复用打地鼠 game_whack.c 的时间条做法）
 *
 * 替代原「3→2→1→GO」大数字倒计时：进入游戏后顶部一条进度条在
 * RACE_INTRO_MS 内从满线性耗到空，耗空即开打。
 * ★ 与打地鼠不同：赛车【只在开场用】这条进度条，正式开打后整条隐藏
 *   （赛车没有限时，撞车才结束，游戏中没有"剩余时间"可显示）。
 * 由 engine_cb 的 RS_COUNTDOWN 分支按 intro_t0 每帧平滑驱动。
 * ═══════════════════════════════════════════════════════════════ */
#define RACE_INTRO_MS 3000 /* 开场进度条耗空时长（ms）= 入场三秒倒计时 */

#define RACE_TIMEBAR_H 16                  /* 进度条高度 px */
#define RACE_TIMEBAR_MARGIN 8              /* 距屏幕左右/顶部的边距 px */
#define RACE_TIMEBAR_RADIUS 8              /* 圆角半径 */
#define RACE_TIMEBAR_BG_COLOR 0x16213E     /* 轨道底色（深蓝）*/
#define RACE_TIMEBAR_OK_COLOR 0x00D466     /* 正常（绿）*/
#define RACE_TIMEBAR_WARN_COLOR 0xFFA502   /* 警告：剩余≤1/3（橙）*/
#define RACE_TIMEBAR_DANGER_COLOR 0xFF3B30 /* 危险：剩余≤1/6（红）*/
