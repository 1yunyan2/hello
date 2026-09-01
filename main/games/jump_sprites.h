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
/* 棋子换成 8.png（转得 jp，79×112，高瘦型，RGB565A8 带 alpha）。
 * 缩放逻辑沿用现有 player_base_scale()「棋子宽=台宽1/3」，台宽随分数缩小→棋子等比缩小。
 * 只换素材与 IMG_W/H 尺寸，缩放代码零改动。*/
extern const lv_image_dsc_t jp;
#define JUMP_PLAYER_IMG jp
#define JUMP_PLAYER_IMG_W 79  /* 8.png 原图宽 */
#define JUMP_PLAYER_IMG_H 112 /* 8.png 原图高 */

/* 动态缩放参数 */
#define JUMP_PLAYER_MIN_W 20        /* 棋子宽下限 px（小台子上不至于太小看不清）*/
#define JUMP_PLAYER_SQUASH_RATIO 75 /* 满蓄力时 scale_y 压到基准的百分比（75%=只压扁约1/4，比例小幅压扁，仍清晰可辨）*/

/* 近似逻辑尺寸（仅用于光斑中心/出屏阈值等粗略计算，非精确像素）。
 * 取一个代表性高度：约最小棋子宽对应的高（j4 比例 47/62）。*/
#define JUMP_PLAYER_H (JUMP_PLAYER_MIN_W * JUMP_PLAYER_IMG_H / JUMP_PLAYER_IMG_W) /* ≈15 */

/* ── 台子渲染方式开关 ──
 * 1 = 图片台子（1-7.png 转得 jt1~jt7，用 lv_image + scale_x/scale_y 缩放压扁）
 * 0 = cube3d 自画台子（顶/侧面三角形 + 随机条带 + 20 种几何花纹），旧逻辑保留，可一键回退 */
#define JUMP_PLAT_USE_IMG 1

#if JUMP_PLAT_USE_IMG
/* 7 张台子图（均 95×97），每个新台随机选一张（避开上一台），台宽随分数缩小→图等比缩小 */
extern const lv_image_dsc_t jt1, jt3, jt4, jt5, jt6, jt7;
#define JUMP_PLAT_IMG_COUNT 6
#define JUMP_PLAT_IMG_W 95 /* 台子原图宽 */
#define JUMP_PLAT_IMG_H 97 /* 台子原图高 */
/* 各台子图片台面顶边在原图中的 Y 偏移（从图片顶边，像素）。
 * 每张图形状不同，台面高度不一样，必须独立配置；否则棋子会悬空或陷入台面。
 * 索引 0~6 对应 jt1~jt7（img_idx 0~6）。*/
#define JUMP_PLAT_IMG_FACE_Y_TABLE {24, 52, 52, 22, 18, 28, 52}
/* 各台子图「可站立顶面宽度」占整图宽(95px)的百分比。
 * 台子是 2.5D 立方体，整图含侧壁/透明边，真正能落脚的只有顶面那块菱形/方块。
 * 落台判定必须用「顶面宽」而非整图宽，否则棋子站在侧壁斜面/透明边也算落台（悬空感）。
 * 比例按每张原图顶面实测标定（索引 0~6 对应 jt1~jt7）：
 *   1 弹簧顶小方块~53 / 2 横躺~79 / 3 红格~74 / 4 黄箭头~65 / 5 星星~74 / 6 箭头~65 / 7 圆盘~82 */
#define JUMP_PLAT_IMG_FACE_W_PCT_TABLE {53, 79, 74, 65, 74, 65, 82}
/* 摄像机延迟启动：落台后先等 N 帧（让玩家看清落点），再开始平滑滑动 */
#define JUMP_CAM_DELAY_FRAMES 5
/* 棋子落台：台子整体下移 N px 再回弹，棋子跟随同步下移（不做 scale 变形）。
 * bounce 字段在图片模式下直接存下移像素数（0~JUMP_LAND_BOUNCE px）。
 * 蓄力时台子同步下压，棋子随台面下沉（sink = bounce_pct * max_sink）。*/
#define JUMP_LAND_BOUNCE 12      /* 落台初始下移量 px（轻微压感）*/
#define JUMP_LAND_BOUNCE_DECAY 1 /* 每帧衰减 px（约 12 帧弹回）*/
/* ── 落台手感二选一开关 ──
 * 0 = 整体下坠：台子+棋子整体下移 JUMP_LAND_BOUNCE px 再线性弹回（旧逻辑，bounce 字段）。
 * 1 = 压扁回弹：复用蓄力 squash（顶面下沉、底部锁死），棋子脚底跟随，压一下再回弹到台面原高度。
 * 两套代码均保留，改这一个宏即可切换。*/
#define JUMP_LAND_USE_SQUASH 1
/* 压扁回弹参数（仅 JUMP_LAND_USE_SQUASH=1 生效）：
 * 落台瞬间台子顶面压扁到 PCT%（对应顶面最大下沉 PCT% × JUMP_PLAT_IMG_SQUASH_MAX_PX），
 * 再以 FALL/帧 回弹到 0（台面原高度）。RISE 越大压得越快，FALL 越小弹回越慢、越 Q 弹。*/
#define JUMP_LAND_SQUASH_PCT 30  /* 落台压扁峰值百分比 0~100（想压更狠就调大）*/
#define JUMP_LAND_SQUASH_RISE 12 /* 压扁阶段每帧上升百分比 */
#define JUMP_LAND_SQUASH_FALL 4  /* 回弹阶段每帧下降百分比 */
/* 图片台子蓄力最大下沉量 px（台子随棋子蓄力同步下压，0~100%对应0~此值）*/
#define JUMP_PLAT_IMG_SQUASH_MAX_PX 8
#endif

/* ── 立体台子（LVGL 纯代码画：顶面亮 + 侧面暗，2.5D）── 仅 JUMP_PLAT_USE_IMG=0 时生效
 * 顶面是一个圆角矩形（亮色），其正下方叠一层稍暗的侧面矩形当“厚度”，产生立体感。
 * 后期 5~6 种不同大小/颜色只需扩 s_plat_styles[] 表，无需改渲染逻辑。*/
#define JUMP_TOP_Y 145          /* 顶面顶边 y（小人脚踩这条线）*/
#define JUMP_PLATFORM_H 22      /* 顶面厚度（圆角矩形高）*/
#define JUMP_PLATFORM_SIDE_H 14 /* 侧面（立体厚度）高度 */
#define JUMP_PLATFORM_RADIUS 0  /* 顶面圆角 */
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

/* 台宽下限（小台子变体也不会窄于此值，保证还能落脚）*/
#define JUMP_PW_MIN 26

/* ── 随机台子变体（不随分数递增，且锁死出现频率）──
 * 台宽不再「越往后越小」；改为：绝大多数台子是「正常台」，
 * 少数台子变成「小台子」或「远台子」，且出现频率被严格锁死：
 *   1) 开局前 WARMUP 张一律正常台（热身，让玩家先上手）；
 *   2) 热身后，距上一张变体台「不足 COOLDOWN 张正常台」时绝不出变体
 *      （保证两张变体台之间「最少」间隔 COOLDOWN 张正常台，绝不连续）；
 *   3) 冷却满足后，每张台按 TRIGGER_PCT 概率决定是否变体
 *      （所以实际间隔 ≥ COOLDOWN，且平均更稀疏）；
 *   4) 触发变体时在「小台子 / 远台子」之间各 50% 随机选。*/
#define JUMP_VARIANT_WARMUP 10        /* 开局前 N 张强制正常台 */
#define JUMP_VARIANT_COOLDOWN 3       /* 两张变体台之间「最少」间隔 N 张正常台 */
#define JUMP_VARIANT_TRIGGER_PCT 50   /* 冷却满足后，每张台出现变体的百分比概率 */
#define JUMP_VARIANT_SMALL_MIN_PCT 72 /* 小台子宽 = 基础宽的 72%~（约缩小 1/4）*/
#define JUMP_VARIANT_SMALL_MAX_PCT 80 /* ~80% */
#define JUMP_VARIANT_FAR_EXTRA_MIN 25 /* 远台子在常规间距上额外拉远 25~ px */
#define JUMP_VARIANT_FAR_EXTRA_MAX 45 /* ~45 px（最终仍被「可达上限」钳制，保证跳得到）*/

/* 相邻台子之间的最小可见缝隙 px（中心距 ≥ 两台半宽和 + 此值，防止台子重叠）*/
#define JUMP_PLAT_MIN_SPACING 18

/* ── 蓄力→跳跃距离 ── */
#define JUMP_HOLD_MIN_MS 150
#define JUMP_HOLD_MAX_MS 1400
#define JUMP_DIST_MIN_PX 55
#define JUMP_DIST_MAX_PX 200

/* ── 蓄力震动（替代原「按下震一下」）──
 * 需求：跳一跳不再在按下瞬间震一下，改为【按住蓄力全程持续震动】，
 * 强度随蓄力百分比线性上升，松手起跳/作废立刻停。
 *   · MIN_LEVEL：刚按下时的强度（%），太低马达转不起来，建议 ≥30
 *   · MAX_LEVEL：蓄满（JUMP_HOLD_MAX_MS）时的强度（%）
 *   · STEP     ：强度量化步长（%），只有跨过一档才真正写 LEDC，
 *                避免 30fps 每帧都写寄存器（听感上也更像"逐级加力"）*/
#define JUMP_CHARGE_VIB_MIN_LEVEL 30
#define JUMP_CHARGE_VIB_MAX_LEVEL 100
#define JUMP_CHARGE_VIB_STEP 10

/* ── 台子 Y 轴随机偏移（双轴散落效果）── */
#define JUMP_GAP_Y_MAX 50 /* 下一台相对当前台 Y 轴最大偏移 px（上下各±50）*/

/* ── 抛物线（二维）── */
#define JUMP_APEX_RATIO 40 /* 弧顶高 = 直线距离 × 40% */
#define JUMP_FLY_STEP_PX 9 /* 每帧沿飞行方向推进 px（值越大跳越快；6→8 比半速快约1/3）*/

/* ── 落空坠落（PH_FALL）── */
/* 坠落重力：每帧竖直速度增量 px。注意坠落「初速」不用这个值起跳，而是接力飞行末帧的
 * 真实下落速度（≈4*apex*(n-1)/n²，恒约12~14px/帧，见 game_jump.c 落空判定分支）；
 * 若从 0 起加速，落空瞬间竖直会由 14→3 骤降而水平仍 9px/帧，肉眼呈现
 * 「贴着台面高度横向平移一下」的假象（像踩到透明台子）。值越大坠落越急。*/
#define JUMP_FALL_GRAVITY 3

/* ── 摔倒判定（PH_TOPPLE）：踩到台缘但落脚面不够，先站稳再朝台外倾倒 ── */
/* 棋子「有效落脚半宽」= 缩放后图片半宽 × 此百分比。占比越大越易判摔倒（更难）。*/
#define JUMP_FOOT_W_PCT 50 /* 落脚面占棋子图宽的百分比（50%）*/
/* 摔倒区间边界：棋子中心超出台缘的量 overshoot 落在 (0, 脚底半宽×此倍率/100) 算摔倒；
 * 超过则重心彻底出台缘，走完全坠落 PH_FALL。倍率>100 可放大「摔倒」出现概率。*/
#define JUMP_TOPPLE_RANGE_PCT 100 /* 摔倒区上界 = 脚底半宽 × 100% */
#define JUMP_TOPPLE_HOLD_FRAMES 6 /* 倾倒前「站稳一瞬」停顿帧数（约0.2s @30fps）*/
#define JUMP_TOPPLE_MAX_ANG 800   /* 最大倾倒角（0.1°单位，800=80°，接近趴下）*/
#define JUMP_TOPPLE_ANG_STEP 80   /* 每帧倾倒角增量（0.1°单位，80=8°/帧）*/

/* ── 摄像机平滑（落台后滑动）── */
#define JUMP_CAM_SMOOTH_FRAMES 12 /* 平滑帧数，约0.4s @30fps */

/* ── 台子从天而降入场动画 ── */
/* 【2026-08-19 减轻掉落斜纹】台子入场/回弹改用独立高频 timer（game_jump.c 的
 * drop_tick_cb），不再跟随 JUMP_ENGINE_MS。
 *
 * 原理：撕裂一直在发生（屏无 TE 引脚，SPI 写入与液晶扫描不同步，见 [BUG-041]），
 * 肉眼看不看得见取决于【撕裂线两侧差多少】=【每帧位移量】：
 *   · 落台压扁每帧只变 ~1px  → 只剩边缘锯齿（可接受）
 *   · 台子下落每帧移 11~24px → 一道明显斜切
 * 故把掉落帧率翻倍、重力同步减小：总距离/总时长基本不变，每帧只走一半。
 * 只拆这一段跑高频，主引擎仍是 JUMP_ENGINE_MS，飞行/相机/蓄力手感一律不变。*/
/* 【必须与游戏内 LVGL 刷新周期(game_jump.c 的 JUMP_REFR_PERIOD_MS)取同一个值】
 * 两个定时器都由同一个 lv_timer_handler 驱动，同周期即同一轮里依次触发，是确定的
 * 1:1，不会相位漂移。只提刷新率 → 多刷的是重复帧；只提步进率 → 值变了也要等下次
 * 刷新才上屏。两者缺一都白搭（同 ui_port.c:7361 头部压扁已验证的结论）。*/
#define JUMP_DROP_TICK_MS 10 /* 台子入场/回弹专用帧间隔 ms */
/* 入场高度 90→60→40：撕裂阈值锁死在 JUMP_DROP_SPEED_PX=4，速度不能再提，
 * 想让掉落更快只能缩短总距离（总时长 = 距离 ÷ 速度 × 帧间隔）：
 *   60px → 15帧 × 20ms = 300ms
 *   40px → 10帧 × 20ms = 200ms ← 当前取值
 *   30px →  8帧 × 20ms = 160ms（再短「从天而降」感开始不足）*/
#define JUMP_DROP_HEIGHT 70 /* 入场起始离目标的高度 px */

/* ── 匀速下落速度：撕裂消不掉，就把每帧错位压到看不见的量级 ──────────────
 * 【判据】撕裂可见度 = 每帧位移量（ui_port.c:7314 头部压扁实测结论）：
 *   · 跳一跳蓄力压扁   1px/帧   → 看不出
 *   · 头部压扁修复后   4.3px/帧 → 边缘毛刺，可接受
 *   · 本处旧版重力加速 峰值24px/帧 → 一眼可见大斜切
 *
 * 【为什么去掉重力加速】看着有多明显取决于【最坏那一帧】而非平均值。重力下 vy
 * 递增、最后几帧最快，峰值是平均的 3 倍，前面帧数的努力全被最后一帧作废。
 * 匀速后峰值 = 平均，零成本砍掉全部峰值（同 ui_port.c:7381 改 linear 的理由）。
 *
 * 【调参阶梯】帧间隔约 20ms（16ms 周期 + lvgl_port timer_period_ms=10 的调度粒度）：
 *   6 → 60px/6 = 10帧 × 20ms = 200ms   快，锯齿仍可见
 *   4 → 60px/4 = 15帧 × 20ms = 300ms ← 当前取值，已优于头部压扁的 4.3px
 *   3 → 60px/3 = 20帧 × 20ms = 400ms   更淡，掉落开始显肉
 *   2 → 60px/2 = 30帧 × 20ms = 600ms   接近蓄力压扁那档(看不出)，但太慢
 * 实拍后直接调本宏即可，逻辑不用动。*/
#define JUMP_DROP_SPEED_PX 6 /* 匀速下落/弹起速度 px/帧（峰值=平均）*/

/* 触底后向上弹起的高度 px（0=不弹，落到底直接落定）。
 * 弹起同样走 JUMP_DROP_SPEED_PX 匀速，12px ≈ 上下各 3 帧，共 6 帧约 120ms。*/
#define JUMP_DROP_BOUNCE_UP_PX 5

#define JUMP_DROP_GRAVITY 1      /* 【匀速版不再使用】仅 cube3d 模式(JUMP_PLAT_USE_IMG=0)分支引用 */
#define JUMP_DROP_INIT_VY 10     /* 挂起态占位速度（plat_pend_drop 用，实际由 plat_start_drop 覆盖）*/
#define JUMP_DROP_BOUNCE 45      /* 触底回弹初始压扁量（仅 cube3d 模式）*/
#define JUMP_DROP_BOUNCE_DECAY 8 /* 回弹压扁每帧衰减量（仅 cube3d 模式）*/
#define JUMP_DROP_BOUNCE_COEF 35 /* 【匀速版不再使用】旧「速度反弹系数」，保留防其它引用编译失败 */

/* ── 掉落 ── */
#define JUMP_FALL_STEP_PX 10

/* ── 蓄力压扁（台子）── */
#define JUMP_PLATFORM_SQUASH_MAX_PX 18
