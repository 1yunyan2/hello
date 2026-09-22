/**
 * @file ui_port.c
 * @brief UI界面端口实现文件（极简版：单闹钟 + 单倒计时）
 *
 * 核心功能：基于LVGL实现智能玩偶的全量UI界面
 * 架构特点：
 *   1. 视图状态机：主界面 → 功能菜单(4页) → 闹钟编辑(2步：时→分)
 *   2. 异步渲染：所有LVGL操作均加锁保护
 *   3. 低耦合：与底层触摸、提醒系统通过接口解耦
 *
 * 本次改动：
 *   - 闹钟：单闹钟直接展示 + 2步编辑(时/分)，重复固定「只响一次」、保存即开启
 *   - 倒计时：保持单倒计时，分钟调节逻辑完整保留
 *   - 清理：移除所有列表相关死代码
 */
#include "ui_port.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_random.h"
#include "esp_timer.h" /* 初次联网避让窗口计时：esp_timer_get_time() */
#include "bsp/bsp_config.h"
#include "bsp/bsp_board.h"
#include "bsp/servo_manager.h" /* GIF 切换时只驱动舵机:非阻塞入队 + 独立 worker 执行 */
#include "esp_spiffs.h"
#include "ui/reminder.h"
#include "ui/standby.h"
#include "games/games.h"
#include "ui/interaction.h"    /* interaction_is_playing()：情绪播放中暂停自动循环 */
#include "session/session.h"   /* session_get_state()：对话中屏蔽情绪触摸 */
#include "ui/remote_control.h" /* remote_control_is_active/cancel()：app 远程控制 30s 冻结窗口 */
#include "ui/weather_icons.h"  /* weather_icon_get()：中文天气现象 → 128×128 图标查表 */
#include "font_loader.h"       /* 外挂 .bin 字体懒加载（font_cn_12/32、font_num_140 等） */
#include "object.h"
#include <string.h>
#include <limits.h>
#include <stdio.h>
#include "driver/adc.h"

/* ── 外部自定义中文字体声明 ──
 * 2026-08-14 大改：font_cn_12/24/32、font_num_140、font_time_144/cal_74/wx_50
 * 已迁到外挂 Flash（assets/font/ 下的 .bin，经 font_loader.h 懒加载），不再编译进固件。
 * 此处只保留尚未迁移的：
 *   - font_cn_16（16px 大字符集，暂留固件，后续再迁）
 *   - font_alarm_59（闹钟，暂不放外挂） */
LV_FONT_DECLARE(font_cn_16);
LV_FONT_DECLARE(font_alarm_59);

/* 时间页大号时间专用（215px 4bpp，MiSans-Regular，仅 "0123456789:"）。
 * 编译进固件的 numbig.c 体积约 500KB，把 app 分区挤爆（overflow），故改回
 * 外挂 Flash 运行时加载，源文件 assets/font/numbig.bin，加载策略同下方
 * wx_deg_font_get()：懒加载 + 失败回退，不占 app 分区。 */

/* ── 功能页面顺序 ── */
typedef enum
{
    /* 2026-08-10 拆分：原 FN_PAGE_TIME 一页同时画「右上角小时钟 + 本周日历条」，
     * 现拆成两个独立功能页，各自对应功能盘上的一个图标：
     *   FN_PAGE_TIME     → 纯时间页（居中大号 HH:MM + 右上角 周几/日 + 温度）
     *   FN_PAGE_CALENDAR → 纯日历页（左侧大号日号 + 英文星期 + 右侧整月点阵）
     * 时间页排在最前：它是全机最高频需求，且拆分前时间只藏在日历页角落。 */
    FN_PAGE_TIME = 0,  // 纯时间（大号 HH:MM）
    FN_PAGE_CALENDAR,  // 纯日历（大号日号 + 整月点阵）
    FN_PAGE_ALARM,     // 闹钟
    FN_PAGE_WEATHER,   // 天气
    FN_PAGE_COUNTDOWN, // 倒计时
    FN_PAGE_COUNT
} fn_page_t;

static const char *const s_fn_page_titles[FN_PAGE_COUNT] = {
    [FN_PAGE_TIME] = "时间",
    [FN_PAGE_CALENDAR] = "日历",
    [FN_PAGE_ALARM] = "闹钟",
    [FN_PAGE_WEATHER] = "天气",
    [FN_PAGE_COUNTDOWN] = "番茄时钟", /* 2026-08-10：倒计时页改造为番茄时钟（6 预设 + 自定义） */
};

static const char *TAG = "UI_PORT";

/* ── 函数前向声明 ── */
static void main_clock_refresh(void);
static void main_clock_tick_cb(lv_timer_t *t);
/* ── 主界面 GIF 自动循环 + 舵机联动 前向声明 ── */
static int main_gif_pick_next_index(int cur);               // 选下一张 GIF 表索引
static void main_gif_ready_cb(lv_event_t *e);               // GIF 播完一轮事件(只置 flag)
static void main_gif_switch_timer_cb(lv_timer_t *t);        // 延迟切换:真正 set_src + 入队舵机
static void main_gif_apply_index(int idx, bool with_servo); // 应用一张 GIF(切图 + 可选触发舵机)
static void main_gif_kick_resume(void);                     // 退回主界面后重启 GIF 循环
/* 移除 on_refr_start / on_refr_ready：
 *   PARTIAL 模式 + W*H/5 单缓冲下，LV_EVENT_RENDER_START 每个 invalid area 触发一次（多次），
 *   而 LV_EVENT_RENDER_READY 整帧只触发一次，pause/resume 不对称 → GIF 帧推进 timer 被永久 pause，
 *   表现为"卡死第一帧 + UI 切换无效"。在 PCBA 信号不稳的板上，这层多余防护会放大 GIF 不刷新现象。
 *   BUG-010（栈在 SPIRAM + Flash cache disable）已通过 task_stack_caps = MALLOC_CAP_INTERNAL 解决，
 *   不再需要这层"渲染期间暂停 GIF"的补丁。
 */
__attribute__((unused)) static const char *s_alarm_repeat_cn(alarm_repeat_t r);
static void countdown_tick_cb(lv_timer_t *t);
static void alarm_edit_render(void);
static void countdown_page_render(void);
/* 2026-08-10 到期自动复位（方案 A）：定义在文件靠后处，此处前置声明供
 * countdown_page_show() / countdown_cancel() / countdown_page_hide() 提前使用。 */
static void countdown_expire_reset_cb(lv_timer_t *t);
static void countdown_expire_reset_cancel(void);
static void countdown_start_and_exit(void); // 头部长按：启动倒计时+退出功能页（震动由触摸层给）
static void render_fn_page(fn_page_t page);
static void time_page_render_text(void);
/* 2026-08-20：低功耗常亮时钟（ui_standby_clock_show，位于本文件更靠前处）需要它——
 * s_menu_panel 是懒创建的（只有进过功能盘才 lv_obj_create），而时间页是它的子对象，
 * 面板为 NULL 时 time_page_render() 首行直接 return，表现为「时钟画不出来」。 */
static void ensure_menu_panel(void);
static void alarm_page_create(void);
static void alarm_page_rebuild(void);
static void alarm_page_show(void);
static void alarm_page_hide(void);
static void alarm_edit_create(void);
static void alarm_edit_enter(void);
static void alarm_edit_enter_ex(bool reset_to_zero); // true=编辑值从 00:00 起（响铃结束入口）
static void alarm_edit_exit(bool save);
static void alarm_edit_confirm_with_feedback(void); // 头部长按：闪烁后保存退出（震动由触摸层给）
static void alarm_edit_value_next(void);
static void alarm_edit_value_prev(void);
static void alarm_edit_advance(void);
static void alarm_edit_zoom_cancel(void); // 取消「1秒后转场」定时器（幂等）
__attribute__((unused)) static void alarm_edit_back(void);

/* ── 单层横向功能盘：渲染与导航 前向声明 ── */
static void menu_enter_fn_page(fn_page_t page);   // 从功能盘进入指定功能页（复用功能菜单）
static void app_enter_time(void);                 // 功能盘项：时间 → FN_PAGE_TIME
static void app_enter_calendar(void);             // 功能盘项：日历 → FN_PAGE_CALENDAR
static void app_enter_alarm(void);                // 功能盘项：闹钟 → 直接进闹钟编辑
static void app_enter_countdown(void);            // 功能盘项：定时器 → FN_PAGE_COUNTDOWN
static void app_enter_weather(void);              // 功能盘项：天气 → FN_PAGE_WEATHER
static void enter_whack(void);                    // 功能盘项：打地鼠 → UI_VIEW_GAME + games_start
static void enter_race(void);                     // 功能盘项：赛车 → UI_VIEW_GAME + games_start
static void enter_jump(void);                     // 功能盘项：跳一跳 → UI_VIEW_GAME + games_start
static void home_render(void);                    // 渲染功能盘（居中大图标 + 名称）
static void home_icon_cache_init(void);           // 功能盘图标预读进 PSRAM（幂等）
static void ui_home_fade_timer_cb(lv_timer_t *t); // 进功能盘背光渐变 timer 回调
static void home_jelly(int dir);                  // 果冻弹动切换（dir=+1 下一个 / -1 上一个）
static void back_to_home(void);                   // 功能页/游戏 摸腹背 → 返回功能盘
/* 【2026-09-01】功能层渐变退出启动器（不震动）。实现在文件后段
 * （紧邻 ui_func_layer_exit_to_main）；这里声明是因为空闲超时回调
 * menu_idle_timeout_cb 位置更靠前，要调用它。 */
static void func_layer_exit_fade_start(void);
static void menu_clear_func_pages(void);                 // 隐藏所有功能页专属对象，仅留 title+body
static void home_icon_fade_cancel(void);                 // 取消挂起的图标渐暗渐亮（幂等，退出功能盘时调用）
static void home_icon_fade_timer_cb(lv_timer_t *t);      // 图标渐暗→换图→渐亮推进回调
static void home_press_refr_boost(bool on);              // 头部压扁期临时提高 LVGL 帧率（幂等）
static void countdown_exit_fade_timer_cb(lv_timer_t *t); // 倒计时退出渐暗→切页→渐亮推进回调
static void countdown_exit_tmr_cb(lv_timer_t *t);        // 倒计时停留计时到点 → 触发退出渐变
static uint32_t countdown_pick_duration_sec(void);       // 取当前选中项的倒计时秒数
static bool countdown_enter_prepare_ui(void);            // 渐变进入第一步：暗态只切版式不计时
static void countdown_enter_start_timing(void);          // 渐变进入第二步：渐亮后才真正计时

/* ── 【2026-09-01】提醒到期渐变（闹钟响铃 / 番茄钟结束）前向声明 ──
 * 启动器实现在 countdown_exit_fade_timer_cb 附近（与它共用状态机）；
 * 五个暗态动作各自就近实现在原硬切代码的位置上。 */
static bool expire_fade_start(void (*dark_cb)(void), bool quiet_main_screen, const char *what);
static void countdown_expired_dark_action(void);      // 主界面 → 番茄钟到期画面
static void alarm_ringing_dark_action(void);          // 主界面 → 闹钟响铃画面
static void countdown_tick_expired_dark_action(void); // 番茄钟运行界面 → 到期画面（人在页上）
static void countdown_reset_dark_action(void);        // 番茄钟到期画面 → 主界面空闲 GIF
static void alarm_reset_dark_action(void);            // 闹钟响铃画面 → 主界面空闲 GIF
/* 【2026-09-01】退出功能层回主界面的暗态动作本体（藏 menu_panel + 显 gif_obj +
 * s_view=MAIN + ui_resume_main_gif_loop 拉回空闲轮播与舵机动作）。实现在文件后段
 * （紧邻 ui_home_fade_timer_cb）；这里声明是因为上面两个到期暗态动作位置更靠前，
 * 它们都要调用它（结束后直接回空闲 GIF，不再停在设定/编辑页）。 */
static void ui_home_exit_apply(void);

/* 【临时诊断 2026-08-19】内部 SRAM 占用探针（定义在 render_fn_page 上方）。
 * 前向声明放这里：闹钟编辑入口 alarm_edit_enter_ex() 位于本文件更靠前处，
 * 不声明会隐式声明报警告/报错。定案后与实现一并删除。 */
#define UI_MEM_PROBE 1
#if UI_MEM_PROBE
static void ui_mem_probe(const char *stage, const char *page_name);
#endif

/* ── 配置宏 ── */
#define UI_MAIN_GIF_RANDOM 1 // 主界面 GIF 是否随机（1=随机，0=顺序）。空闲 GIF 随机切换。
/* 开机 Logo：开机固定显示这一张【静态图】（LVGL bin，外挂 flash），一直显示到
 * ui_notify_boot_ready() 到达，随后渐暗→揭图→渐亮，进入正常主界面轮播。
 *
 * ★ 为什么用独立的 lv_image 而不是复用 gif_obj：
 *   静态图不会触发 LV_EVENT_READY，而"logo 循环重播"正是靠该事件驱动的
 *   （旧实现见 main_gif_ready_cb 的 s_boot_logo_playing 分支）。若把静态图塞进
 *   gif_obj，那条链路失效的同时 gif_obj 也会停在非 GIF 源的坏状态。故改为：
 *   gif_obj 开机就预载好主轮播首张（被 logo 图整个盖住看不见），logo 用一个叠在
 *   上层的 lv_image 承载；就绪后在"渐暗到底"那一刻删掉它即揭开下面的 GIF，
 *   零切图延迟，且天然不重复播放。
 *
 * 源文件 assets/img/log.bin，需随 storage.bin 打包烧录到外挂 flash 的 /S/img/
 * 目录（同 pw.bin / c8.bin）。若文件不存在，自动回退为直接进正常轮播（不黑屏）。 */
#define UI_BOOT_LOGO_GIF 1                 // 1=开机先显示 logo，0=直接进正常轮播
#define UI_BOOT_LOGO_PATH "S:/img/log.bin" // 开机 logo 静态图路径（320×240 LVGL bin）

/// 首次配网提示图路径（320×240 LVGL bin，外挂 flash）。源文件 assets/img/pw.bin，
/// 需随 storage.bin 一起打包烧录到外挂 flash 的 /S/img/ 目录（同 c8.bin）。
#define UI_PROVISION_IMG_PATH "S:/img/pw.bin"

/* 开机 logo 背光渐变：开始渐亮一次(0%→100%) + boot_ready 后渐暗→切图→渐亮一次(100%→0%→100%)。
 * 只用于这两次过渡，logo 循环重播、空闲态自动轮播、情绪/状态切图均保持硬切不受影响。
 * 独立 lv_timer 推进，与 s_gif_switch_tmr（切图 timer）完全解耦。 */
/* 两个宏都只影响开机这三段过渡，进功能盘的 UI_HOME_FADE_OUT_MS / UI_HOME_FADE_IN_MS
 * 独立，不受影响。
 *
 * 【2026-08-31 调参】1200/1600 → 400/400，为缩短开机总时长（这 2.8s 是串行追加在
 * 初始化之后的，纯等待）。原先渐亮拉长到 1600 是为了平衡"渐亮显快、渐暗显慢"的
 * 观感不对称，缩到 400 后单段已很短，该差异不再明显，故两段取同值。
 *
 * ⚠【实测勿改为 0/1】曾试把两者改成 1（等效去除渐变，需求："瞬间出 logo、瞬间出
 * GIF"），烧录后 logo→GIF 切换处立刻出现撕裂，已回退。
 * 【原因】UI_BOOT_FADE_OUT 的作用不只是好看：它渐暗到全黑后【在黑屏里切图】，把
 * 刷屏过程整个藏起来。这块屏没有 TE 引脚，flush 与面板扫描永远不同步，亮屏状态下
 * 直接切图必然可见斜切/两帧同框（同 BUG-041 / BUG-040，属硬件天花板、十余轮软件
 * 方案已证实无解）。
 *
 * ★【2026-08-31 关键修正】"亮起来才看到 logo 切 GIF"不是时间不够，是【顺序错了】：
 *   渐暗到底时并没有真的切图，只是置 s_boot_logo_del_pending 标志，真正的 lv_obj_del
 *   由 LVGL 线程在下一拍执行（必须回 LVGL 线程，见 BUG-029/030）。而原代码置完标志
 *   【立刻】转入渐亮，不等 LVGL 干完 —— 开机时 taskLVGL 正忙于 GIF 解码（BUG-027 实测
 *   可抢 93.7% CPU），这一拍能被拖几十上百毫秒，于是背光先亮、logo 后删。
 *   把渐变时长从 400 调到 1000 依然复现，正是"靠时间余量赌顺序"必然失败的证据。
 *   已改为新增 UI_BOOT_FADE_WAIT_SWAP 相位：全黑卡住直到揭图真正完成才渐亮，
 *   顺序由因果保证，与这两个时长值再无关系（想设多短都安全）。 */
#define UI_BOOT_FADE_MS 400    // 渐暗（UI_BOOT_FADE_OUT）单段耗时
#define UI_BOOT_FADE_IN_MS 400 // 渐亮（UI_BOOT_FADE_IN / IN2）单段耗时
#define UI_BOOT_FADE_STEP_MS 2 // 推进间隔：对齐 ~60Hz，避免 20ms(50Hz) 与刷屏拍频
/* 等待 LVGL 线程揭图的最长时间（ms）。超时后强制渐亮，退化成改动前的行为
 * （可能看到切换），但绝不会永久黑屏 —— 万一 s_gif_switch_tmr 因早退分支未被消费，
 * 没有这道兜底就是死黑屏，比撕裂严重得多。正常情况几十毫秒内即完成。 */
#define UI_BOOT_SWAP_WAIT_MAX_MS 1000
/* 揭图后的【暗态窗口】：等 GIF 首帧真正 flush 到屏上 + 液晶响应，之后才渐亮。
 *
 * 【为什么光等 s_boot_logo_del_pending 清零不够】那个标志在 main_gif_switch_timer_cb
 * 里是【lv_obj_del 调用之前】就被清掉的（见该函数），而 lv_obj_del 本身也只是把对象
 * 从对象树摘掉并标脏 —— 真正把 GIF 画面推上屏的 flush 由 lv_timer_handler 异步完成。
 * 于是"标志清零"≠"画面已就位"：若此刻立刻渐亮，flush 会在背光已经爬升的过程中落地，
 * 屏上就出现一条水平分界（界线上方是已刷出的部分、下方还是旧内容），且随 CPU 忙闲
 * 时有时无 —— 即实测到的"渐亮期间偶发亮带、400~800ms 都有而 3000ms 没有"
 * （3000ms 时前段背光仍极暗，flush 早在暗期完成，故看不见）。
 *
 * 【同源先例】进功能盘那条路早就踩过同一个坑并用同样手法解决，见下方
 * UI_HOME_FADE_DARK_MS 的注释（"图标那一帧 flush 会在背光已爬升时落地→图标闪一下"）。
 * 本宏就是它的开机版，取值同量级。 */
#define UI_BOOT_SWAP_DARK_MS 60
typedef enum
{
    UI_BOOT_FADE_IDLE = 0,
    UI_BOOT_FADE_IN,        // 开机首次渐亮 0→100
    UI_BOOT_FADE_OUT,       // boot_ready 后渐暗 100→0（暗到底 → 转 WAIT_SWAP）
    UI_BOOT_FADE_WAIT_SWAP, // 全黑保持，等 LVGL 线程真正删掉 logo 揭图后才渐亮
    UI_BOOT_FADE_IN2,       // 揭图完成后渐亮 0→100
} ui_boot_fade_phase_t;
static ui_boot_fade_phase_t s_boot_fade_phase = UI_BOOT_FADE_IDLE;
static int64_t s_boot_fade_start_us = 0;
/* 揭图完成（pending 被 LVGL 线程清零）的时刻，用于计 UI_BOOT_SWAP_DARK_MS 暗态窗口。
 * 不能复用 s_boot_fade_start_us：那个在 WAIT_SWAP 期间正兼作"等揭图超时"的计时基准，
 * 一旦覆盖就把超时判据也一起改掉了。0 = 尚未揭图。 */
static int64_t s_boot_swap_done_us = 0;
static lv_timer_t *s_boot_fade_tmr = NULL;

/* 进功能盘背光渐变：主界面 GIF → 功能盘图标的过渡，复用开机 logo 渐变同一套推进模式
 * （渐暗到底→切图→渐亮），独立 lv_timer，不与 s_boot_fade_tmr / s_gif_switch_tmr 混用。
 * 【DARK 窗口】切图(隐藏 GIF + 显示图标)后 LVGL 异步 flush + 液晶响应需要时间，
 * 若立即渐亮，图标那一帧 flush 会在背光已爬升时落地，表现为「进入时图标闪一下」，
 * 故插入 UI_HOME_FADE_DARK_MS 等待，与左右耳切换的三阶段 fade 对称。 */
/* 【2026-08-31 问题5】渐变时长缩短：原 800/50/1000 = 1850ms，进功能盘等太久。
 * 现 350/50/450 = 850ms，约为原来的一半以内。
 * 【调参依据】
 *   · OUT/IN 是纯背光爬升，无几何形变，缩短不会引入撕裂（与 BUG-041 那类无关）；
 *   · 仍保持 IN > OUT：渐亮比渐暗慢一点，观感更柔和，是原设计意图，予以保留；
 *   · DARK 50ms 【不要动】——它是等切图 flush 落地 + 液晶响应的窗口，砍掉会让
 *     图标那一帧在背光已爬升时才落地，表现为「进入时图标闪一下」（见上方注释）。
 * 【嫌快/嫌慢怎么调】只动 OUT/IN 两个数，成比例增减即可，DARK 保持 50。 */
#define UI_HOME_FADE_OUT_MS 450  // 渐暗耗时
#define UI_HOME_FADE_DARK_MS 100 // 全黑：隐藏 GIF + 显示图标 + 等 flush + 等液晶响应
#define UI_HOME_FADE_IN_MS 550   // 渐亮耗时（比渐暗略慢，观感更柔和）
typedef enum
{
    UI_HOME_FADE_IDLE = 0,
    UI_HOME_FADE_OUT,  // 渐暗 100→0
    UI_HOME_FADE_DARK, // 全黑：切图 + 等 flush + 等液晶响应
    UI_HOME_FADE_IN,   // 切图后渐亮 0→100
} ui_home_fade_phase_t;
static ui_home_fade_phase_t s_home_fade_phase = UI_HOME_FADE_IDLE;
static int64_t s_home_fade_start_us = 0;
static lv_timer_t *s_home_fade_tmr = NULL;
/* 【2026-08-31 问题4】本轮渐变的方向：true=进功能盘（暗态执行 ui_home_enter_apply），
 * false=退出功能盘回主界面（暗态执行 ui_func_layer_exit_to_main）。
 * 进/退共用同一套三段状态机与同一个 timer，只在「暗到底」那一步分流，
 * 手法与倒计时页的 s_cd_fade_is_enter 一致。 */
static bool s_home_fade_is_enter = true;

/* ★★【2026-09-01 新增：进功能盘渐变期的「主界面空闲链路」闸门】★★
 *
 * 【修的实测故障】「画面已经渐变到功能盘了，偶发舵机还在做上一个动作，做完才快速归中」。
 *
 * 【根因】ui_home_enter() 第一行就 servo_manager_flush() 把舵机打断归中，这一步没问题；
 *   问题出在【它把 s_view = UI_VIEW_HOME 推迟到了暗态】（2026-08-31 改走渐变时搬进了
 *   ui_home_enter_apply）。于是渐暗那 450ms 里 s_view 仍是 UI_VIEW_MAIN，整条主界面
 *   空闲链路的三道守卫【同时失效】：
 *       main_gif_ready_cb / ui_resume_main_gif_loop / ui_request_emotion_gif
 *   实测时序：
 *     t=0    flush → 舵机打断，worker 归中并 give 信号量
 *     t≈10   interaction.c 收尾解阻塞 → 清 is_playing → ui_resume_main_gif_loop()
 *            → s_view 还是 MAIN，守卫放行 → 排下一张图并 resume 轮播 timer
 *     t≈20   main_gif_switch_timer_cb → ui_interaction_play_custom(空闲动作)
 *            → 【新的舵机动作入队并开始跑】←── 多出来的那一下
 *     t=450  暗态才 s_view=HOME + pause 轮播，但新动作已经在跑，拦不住了
 *     t=1100 渐亮结束，功能盘完整显示，舵机仍在动 → 跑完才由 worker 归中
 *   偶发的原因：只有「进功能盘那一刻恰好有动作卡在信号量上」才会触发这条解阻塞链。
 *   ui_home_enter 上那句「下面已把 s_view 设为 HOME」的老注释即由此过期（已更正）。
 *
 * 【为何不直接把 s_view 提前】game_enter_fade 的注释记着这个坑：渐变期间 s_view 与
 *   实际画面不一致，期间来一次触摸会被 ui_dispatch_touch_event 按【新视图】分发，
 *   而屏幕还是旧画面。故另立标志，只关闸「主界面空闲 GIF/舵机」这一条链路，不动分发。
 *
 * 【生命周期】ui_home_enter() 置 true → ui_home_enter_apply() 置 false（两条路径都
 *   必经该函数：渐变暗态、无 timer 兜底）；ui_force_back_to_main() 撤销渐变时一并清，
 *   否则残留会让主界面 GIF 轮播永久停摆。 */
static bool s_home_enter_pending = false;
/* 判据函数 main_idle_loop_active() 定义在 s_view 声明之后（本文件靠后处），
 * 不能放这里：s_view 尚未声明，引用会编译不过。 */

/* 【2026-08-16 新增】倒计时启动后退出功能页的背光渐变状态：与 ui_home_fade_*
 * 同一套「渐暗→暗到底切页→渐亮」三段模式，独立状态机/timer，不与其它渐变复用。
 * 需求：倒计时启动后先停留 3 次 1s tick（3 秒，让用户看清数字在走），随后不再
 * 硬切退出，而是渐暗到全黑时才真正切回功能盘，避免像之前那样在亮屏时刷屏。 */
#define UI_CD_EXIT_FADE_OUT_MS 300 // 倒计时退出：渐暗耗时
#define UI_CD_EXIT_FADE_DARK_MS 50 // 全黑：切页 + 等 flush + 等液晶响应
#define UI_CD_EXIT_FADE_IN_MS 500  // 切页后渐亮耗时

/* 【2026-08-17 新增】功能页「手动退出回功能盘」专用时长档。
 * 与倒计时那档共用同一套状态机，只是分段时长不同：倒计时是自动退出（用户在等），
 * 850ms 的从容节奏合适；手动长按返回若也等 850ms 会明显发钝，故单列一档更短的。
 * DARK 必须盖住整屏重绘：退出要重画 320x240，实测被拆成 6 条 320x40 逐条上屏
 * 约 60~70ms（见 UI_FLUSH_TRACE 实测），故留 100ms 余量，否则渐亮会提前开始、
 * 让用户看见擦除的尾巴。 */
/* 【2026-08-18 实测调档】进时间页的诊断读数：
 *   OUT elapsed=155(设定150) 准时 → esp_timer 改动已生效；
 *   dark_cb 仅 8ms、time_page_render 自身 1ms → 暗态动作不是瓶颈；
 *   DARK elapsed=256(上限200) refr=1 → 全黑 256ms，是全程最大的单段，
 *     且已超过 200 上限 56ms，说明整屏重绘期间连 esp_timer 回调都被拖慢。
 * 用户反馈「变暗后等一点点长」即这 256ms。三个杠杆一起调：
 *   ① OUT 150→100：OUT 段回调准时，这 50ms 是确定省下来的；
 *   ② DARK 200→120：DARK 前期 LVGL 尚未开画、回调还准，能在被拖住前触发上限；
 *   ③ IN 200→280：关键一步。fade_step_fine 带伽马校正(t^2.2)，渐亮前 40%
 *      时间亮度仅约 13%，等于用「很暗但在变亮」替换掉「纯黑干等」，
 *      剩余重绘藏在低亮度段里。总时长 611→500ms，反而更短。 */
/* 【2026-08-18 退出减速：与进入对齐】
 *
 * 【为什么要单列】改动前退出档与进入档是同一组宏（进入档 #define 成退出档），
 * 设定值完全相同，但【实测耗时】差一大截 —— 诊断日志：
 *     进入：DARK 结束 elapsed=256~279   ← 上限 120，被整屏重绘拖到 256+
 *     退出：DARK 结束 elapsed=43        ← 43ms 就从自适应出口走了
 * 差别不在设定，而在暗态要画什么：进入要画时间页（215px 大字整屏重绘），
 * 退出只画回功能盘（一张小图标），后者几十毫秒就 flush 完、REFR_READY 立刻到，
 * 于是 DARK 一到 40ms 地板就放行。结果就是用户感受到的「进入慢、退出快」。
 *
 * 【为什么调 DARK 而不调 OUT/IN】OUT/IN 两段是纯背光曲线，两边本就等长且准时
 * （实测 OUT elapsed≈设定值）；唯一的差额就在 DARK 这一段的 43 vs 256。
 * 故只把退出的 DARK 上限抬上去，让它也黑够同样久，两条路径观感即对齐。
 *
 * 【为什么抬上限就能生效】DARK 有两个出口（见 UI_FADE_DARK_MIN_MS 那段注释）：
 * 自适应出口要求 refr_done && elapsed>=40，兜底出口是 elapsed>=上限。
 * 退出时重绘早就完成，走的是【自适应出口】—— 它只看 40ms 地板，抬上限没用！
 * 所以真正起作用的是下面这条 UI_FN_EXIT_DARK_MIN_MS：给退出单独抬高地板。
 *
 * 【取值】进入实测全黑约 256ms，退出取 240ms 与之基本齐平（略快一点点，
 * 符合用户「稍微慢一点或和进入同速就行」的要求）。想再调只改这一个数。 */
#define UI_FN_EXIT_FADE_OUT_MS 100  // 手动退出：渐暗耗时（与进入同值）
#define UI_FN_EXIT_FADE_DARK_MS 280 // 全黑上限：抬到 280 给下面的地板留出空间
#define UI_FN_EXIT_FADE_IN_MS 280   // 切页后渐亮耗时（与进入同值）
/* 退出专用的最小暗场：进入因整屏重绘天然黑 256ms，退出画得快只黑 43ms，
 * 用这个地板把退出也压住，消除「退出太快」的不对称。0 表示不特殊处理。 */
#define UI_FN_EXIT_DARK_MIN_MS 240

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-18】自适应 DARK：画完就亮，不再固定干等
 *
 * 【原来的毛病】DARK 是一个固定等待值，必须按【最坏情况】取（整屏 6 条 flush
 * 约 60~70ms，时间/日历页首次还要读 f140.bin 字体约 100ms），于是简单页面
 * 明明 70ms 就画完了，也要陪着黑到 100ms —— 实测就是「灭了之后干等一下」。
 *
 * 【改法】给显示器注册 LV_EVENT_REFR_READY，暗态动作执行完就清标志，
 * 收到刷新完成事件即置位；DARK 分支改为「刷新已完成 且 已过最小暗场」就结束。
 * 于是：普通页面约 70ms 就开始渐亮，首次带字体加载那次自然等满，两头都最优。
 *
 * 【为什么一次 REFR_READY 就代表整屏画完】LVGL 的刷新是单线程串行的：
 * lv_display_refr_timer 在一次调用里遍历完所有失效区（本工程整屏被拆成 6 条，
 * 6 次 flush 都在这一次刷新周期内做完）才触发 REFR_READY。而暗态动作跑在
 * 同一个线程的定时器回调里，不可能与刷新周期交叠，故动作之后的第一次
 * REFR_READY 必然是「包含了本次全部失效区」的那一次，不会提前误判。
 *
 * 【为什么还要保留最小暗场】屏是液晶，像素灰阶响应有 20~40ms 惯性。
 * flush 完成 ≠ 画面已稳定；若此刻立刻起渐亮，上一张的残影会在背光爬升时被
 * 照出来（这正是原注释里「等液晶响应」那一笔）。故设 40ms 地板。
 *
 * 【为什么还要保留上限】REFR_READY 万一没来（暗态动作没产生任何失效区，
 * 例如页面内容恰好没变），不能永远黑着。s_cd_fade_dark_ms 退化为上限兜底，
 * 行为与改动前完全一致，故这条改动对四个既有调用方都是只快不慢、无回归风险。
 * ══════════════════════════════════════════════════════════════════════════ */
#define UI_FADE_DARK_MIN_MS 40 // 最小暗场：盖住液晶灰阶响应(20~40ms)，防残影被照出来

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-18 新增】功能盘 →「功能页」进入专用时长档
 *
 * 【解决什么】进入这条路径此前是全机四条转场里唯一没有任何遮挡的：
 * menu_enter_fn_page() 拿到锁就在满亮度下 render_fn_page()，整屏 320x240 脏，
 * 被拆成 6 条 320x40 逐条上屏，用户直接看见刷屏过程。
 * 另三条（主界面→功能盘 / 功能页→功能盘 / 游戏→功能盘）早就用背光渐变盖住了，
 * 这一条只是历史遗留没跟上，本档即为它补齐。
 *
 * 【为什么方向时而自上而下、时而斜着，且斜向左右都有 —— 不必再排查】
 * 那是 flush 节奏与面板扫描的拍频（混叠），非软件缺陷：
 *   · LVGL 按逻辑行拆块（lv_refr.c:794），6 条带在你视角里自上而下贴；
 *   · swap_xy+mirror_x 把物理扫描线转成你视角里的横向扫掠，60Hz / 16.7ms 一遍；
 *   · 每条带间隔 Δ = 渲染耗时 + 2.56ms 传输。
 *     Δ ≫ 16.7ms（时间/日历页，大字体 font_num_140 渲染慢）→ 每条独立现形
 *       → 看到整齐的自上而下；
 *     Δ < 16.7ms（内容简单的页）→ 6 条在同一次扫掠内贴完，竖向的贴与横向的扫
 *       叠加 → 斜线；差值落在零的哪一侧决定斜向左右，故两个方向都会出现。
 * 屏无 TE 引脚，这个相位锁不住，方向调不出来 —— 唯一的解是让它发生在全黑里。
 *
 * 【时长直接复用退出档 UI_FN_EXIT_FADE_*，不单列】
 * 曾短暂单列过一档 120/150/250，已撤销，原因有二：
 *   ① 实测「灭了之后的停顿」偏长 —— DARK 150ms 明显能感觉到干等一下。
 *      依据是上面记的整屏 6 条约 60~70ms，100ms 已有三成余量，150 属过量补偿。
 *   ② OUT/IN 单列成 120/250 纯属随手取值、无依据，反而造出「进去 120/250、
 *      出来 150/200」的不对称。进出同节奏观感更整。
 *
 * 【已知残留：首次进时间/日历可能露一点尾巴】这两页的大号字体是懒加载，
 * 首次要从外挂 Flash 读 f140.bin（36KB，约 100ms，见 :1162 注释），
 * 会顶穿 100ms 的 DARK 窗口。但该字体每次开机只加载一次，之后稳定在 60~70ms。
 * 为一次性现象常驻加 50ms 黑屏不划算，故接受。
 *
 * 【要更好可改自适应 DARK】固定等待本质是按最坏情况取值，快页面白等。
 * 工程里已有 LV_EVENT_REFR_READY 的注册先例（见 :3279-3283 的 UI_FLUSH_TRACE），
 * 可在暗态置标志、收到 REFR_READY 即提前结束 DARK：快页面约 70ms 就亮，
 * 首次带字体加载的那次自然等满。本次未做，留作后续。
 *
 * 【真要单列时怎么办】把下面三行改回独立宏即可，状态机与入口函数都不用动。
 * ══════════════════════════════════════════════════════════════════════════ */
#define UI_FN_ENTER_FADE_OUT_MS UI_FN_EXIT_FADE_OUT_MS   // 150，与退出同节奏
#define UI_FN_ENTER_FADE_DARK_MS UI_FN_EXIT_FADE_DARK_MS // 100，够盖整屏 6 条(60~70ms)
#define UI_FN_ENTER_FADE_IN_MS UI_FN_EXIT_FADE_IN_MS     // 200，与退出同节奏

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-19 新增】功能盘 →「游戏」进入专用时长档
 *
 * 【解决什么】三个游戏的进入此前是硬切：enter_game_common() 拿到锁直接
 * games_start()，build_panel() 建满一屏对象 → 整屏 320x240 脏 → 拆成 6 条
 * 逐条上屏，满亮度下就是用户说的「进入时黑屏闪一下」。这是全机最后一条
 * 没有遮挡的转场，本档为它补齐。
 *
 * 【为什么不能直接套功能页档(100/120/280)】游戏与功能页有个本质区别：
 * 功能页的画面是静态的，晚亮早亮无所谓；而游戏开场自带入场动画——
 * 跳一跳的台子从 JUMP_DROP_HEIGHT(90px) 落下，按 JUMP_DROP_GRAVITY(3)
 * 加速、JUMP_ENGINE_MS(33ms/帧) 推进，约 8 帧 264ms 触底，含弹起落定共约
 * 400ms。若照搬功能页档，黑屏 120ms + 渐亮低可见段(伽马 t^2.2 下前 40%
 * 时间亮度仅约 13%，280ms×40%≈112ms)合计 232ms，整个下落几乎全被吃掉，
 * 用户进去时台子已经躺平了。
 *
 * 【本档怎么定的】用户诉求是「能看到一部分、知道有动画就行」，故按
 * 「保住最有辨识度的那一下」来配：最能让人认出有动画的不是匀速下落，
 * 而是【触底弹起】那个动作，它发生在 264ms 之后。
 *   · DARK 取 UI_FADE_DARK_MIN_MS(40)：只够盖住 build_panel 那一次整屏重绘，
 *     不多黑一毫秒（自适应出口会在 REFR_READY 到达后立刻走，40 是地板不是等待）；
 *   · IN 取 150（比功能页 280 短）：低可见段压到 150×40%≈60ms。
 * 时间线对齐后：总黑屏约 140ms，屏幕约 190ms 起已明显可见，
 * 而触底弹起在 264ms —— 弹起那一下完整落在高亮度区。
 * 净效果：下落前半段偏暗看不清，砸下来弹一下看得清清楚楚，
 * 「闪一下」则被彻底消灭。
 *
 * 【三个游戏共用一档】打地鼠/赛车开场是静态画面(地鼠洞、赛道)，
 * 早亮晚亮无差别，跟着这档只有好处；跳一跳靠这档保住弹起。
 * 故不再为单个游戏细分，避免状态机再长杈。
 *
 * 【游戏模块零改动】本档只改「什么时候调 games_start」，不碰任何游戏代码：
 * games_start / jump_start / build_panel / s_engine_tmr 全部原样，
 * 入场动画逻辑一行未动，只是整体推迟到暗态那一刻才开始。
 * ══════════════════════════════════════════════════════════════════════════ */
#define UI_GAME_ENTER_FADE_OUT_MS 100                  // 渐暗：与功能页同节奏，不发钝
#define UI_GAME_ENTER_FADE_DARK_MS UI_FADE_DARK_MIN_MS // 40：只盖住 build_panel 那次重绘
#define UI_GAME_ENTER_FADE_IN_MS 150                   // 渐亮：比功能页短，压缩低可见段

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-09-01 新增】提醒到期渐变档（闹钟响铃 / 番茄钟结束）
 *
 * 【解决什么问题】原先四个切换点【全部是硬切】：
 *   ① 主界面 GIF → 到期画面        （ui_show_countdown_expired / ui_show_alarm_ringing）
 *   ② 运行界面   → 到期画面        （countdown_enter_expired，人就停在番茄钟页时）
 *   ③ 到期画面   → 设定/编辑界面   （countdown_expire_reset_cb / alarm_ring_reset_cb）
 * 亮屏下整屏 320x240 被拆成 6 条逐条推上屏（实测 60~70ms），看得见横向擦除；
 * ①还额外把结束画面直接盖在 GIF 上，切换突兀。
 *
 * 【时长为何跟随功能盘档】用户 2026-09-01 明确指定「到期渐变的速度和进功能盘一样」。
 * 直接引用 UI_HOME_FADE_* 而非复制数值：那三个宏改了，本档自动跟随，不会走样。
 *
 * 【为什么不复用 UI_HOME_FADE_* 的状态机】那套（s_home_fade_*）的暗态动作是写死的
 * 「进/退功能盘」二选一，加不进第三种动作；而 s_cd_exit_fade_* 那套支持任意
 * dark_cb 函数指针（已被功能页进/退、游戏进/退共 5 个入口复用），故走后者。
 * ══════════════════════════════════════════════════════════════════════════ */
#define UI_EXPIRE_FADE_OUT_MS UI_HOME_FADE_OUT_MS   // 450：与进功能盘同节奏
#define UI_EXPIRE_FADE_DARK_MS UI_HOME_FADE_DARK_MS // 100：全黑上限
#define UI_EXPIRE_FADE_IN_MS UI_HOME_FADE_IN_MS     // 550：渐亮比渐暗略慢

typedef enum
{
    UI_CD_EXIT_FADE_IDLE = 0,
    UI_CD_EXIT_FADE_OUT,  // 渐暗 100→0
    UI_CD_EXIT_FADE_DARK, // 全黑：切页 + 等 flush + 等液晶响应
    UI_CD_EXIT_FADE_IN,   // 切页后渐亮 0→100
} ui_cd_exit_fade_phase_t;
static ui_cd_exit_fade_phase_t s_cd_exit_fade_phase = UI_CD_EXIT_FADE_IDLE;
static int64_t s_cd_exit_fade_start_us = 0;
static lv_timer_t *s_cd_exit_fade_tmr = NULL;
/* 【2026-08-17 新增】本次渐变各段的实际时长，由触发方在启动前设定。
 * 原先回调里直接写死 UI_CD_EXIT_FADE_* 三个宏；现在同一套状态机要服务两种节奏
 * （倒计时自动退出 / 功能页手动退出），故把时长外提成变量，触发方各自赋值。
 * 默认值取倒计时那档，保证任何未显式赋值的路径行为与改动前一致。 */
static uint32_t s_cd_fade_out_ms = UI_CD_EXIT_FADE_OUT_MS;
static uint32_t s_cd_fade_dark_ms = UI_CD_EXIT_FADE_DARK_MS;
static uint32_t s_cd_fade_in_ms = UI_CD_EXIT_FADE_IN_MS;
/* 【2026-08-18 新增】本次渐变的「最小暗场」，即自适应出口的地板值。
 * 原先固定用 UI_FADE_DARK_MIN_MS(40)，导致画得快的退出路径 43ms 就亮起来，
 * 而画得慢的进入路径天然黑 256ms，两边观感严重不对称（进入慢、退出快）。
 * 现提为变量：功能页退出单独抬到 UI_FN_EXIT_DARK_MIN_MS 与进入齐平，
 * 其余路径（倒计时自动退出 / 游戏退出 / 进入）仍取 40，行为与改动前一致。 */
static uint32_t s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS;
/* 【2026-08-17 新增】暗态那一刻要执行的动作（仅在 s_cd_fade_is_enter=false 时用）。
 * NULL = 默认 ui_function_menu_exit()（功能页退出），保持改动前行为；
 * 非 NULL = 自定义，目前用于游戏退出（games_stop + back_to_home）。
 * 用函数指针而非再加一个枚举分支：后续要新增别的"暗场里做的事"零成本。 */
static void (*s_cd_fade_dark_cb)(void) = NULL;
/* 【2026-09-01 新增】到期渐变「已发起、暗态动作尚未执行」的窗口标志。
 *
 * 【非加不可的理由】闹钟响铃是【重复事件】：每响一次就回调一次 ui_show_alarm_ringing()。
 * 原有的重复调用早退判据是「s_alarm_ringing 已置位 且 已停在闹钟页」，而改走渐变后
 * s_alarm_ringing 要等到【暗态】才置位（见 alarm_ringing_dark_action），于是渐暗那
 * 450ms 里判据恒不成立 —— 后续每一拍响铃都会再发起一次渐变，而 expire_fade_start()
 * 对「已有渐变在跑」的处理是【退化成硬切】，结果就是响铃期间反复硬切，比改之前还糟。
 * 本标志把这个窗口补上：渐变发起到暗态动作跑完之间，重复请求一律忽略。
 *
 * 【何时清】① 暗态动作执行完（正常路径，见各 *_dark_action 末尾）；
 *           ② 渐变被外部取消（countdown_exit_fade_cancel），否则残留会让此后
 *              所有到期切页被永久忽略 —— 这是本标志唯一的真风险，必须堵死。 */
static bool s_expire_fade_pending = false;
/* 【2026-08-16 新增】本次渐变的用途：同一套三段状态机（渐暗→暗态做事→渐亮）
 * 被两个场景复用，靠这个标志决定「暗态那一刻做什么事」：
 *   false = 退出：暗态时 ui_function_menu_exit() 切回功能盘（原有行为）；
 *   true  = 进入：暗态时 countdown_start() 启动倒计时 + 切到运行态版式，
 *           解决「确定后瞬间切运行态、能看到刷屏」的问题。
 * 之所以复用而非再写一套：两者节奏、状态流转完全一致，只有暗态动作不同。 */
static bool s_cd_fade_is_enter = false;
/* 【2026-08-18 新增】自适应 DARK 用：暗态动作引发的整屏重绘是否已 flush 完毕。
 * 进入 DARK 相位时清零，由 LV_EVENT_REFR_READY 回调置位（见 ui_fade_refr_ready_cb）。
 * volatile：写在 LVGL 事件回调、读在渐变定时器回调，虽同为 LVGL 线程，
 * 但两者是不同的调用路径，加 volatile 免得编译器把读提到循环外。 */
static volatile bool s_cd_fade_refr_done = false;

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-18 临时诊断】时间页专用渐变时序追踪（问题定位完即删）
 *
 * 【要回答什么】时间页进出时出现「长停顿 / 瞬亮 / 渐亮卡顿 / 瞬间退出」四种表现，
 * 推断它们是同一个原因的不同程度：暗态动作（time_page_render 那一路）在 LVGL
 * 线程里阻塞过久，期间渐变 timer 一次都推不动，等它返回时 elapsed_ms 已越过
 * 整段时长，bsp_board_lcd_fade_step_fine 一步落到终值（见 bsp_lcd.c:156）。
 * 本组日志用于证实/证伪该推断，核心是两个数：
 *   ① dark_cb() 前后时间戳之差 = 暗态动作真实阻塞时长；
 *   ② FADE_IN 首帧的 elapsed_ms —— 若已 ≥ s_cd_fade_in_ms(200)，即本次必然瞬亮。
 *
 * 【为什么只在时间页开】用户明确要求只加在时间里、其他地方不要动。四页共用同一套
 * 状态机，若无差别打印，日志会被其他页/倒计时自动退出淹没，反而看不清。
 * 判据 s_fn_page == FN_PAGE_TIME 覆盖两个方向：
 *   · 进入：s_fn_page 在 home_press_phase2_ready 之前就已由功能盘选中项决定？
 *     ——不，进入时 s_fn_page 要到 dark_cb 内部才被置位，故进入方向额外用
 *     s_fade_diag_armed 标志，由 home_enter_fade 在目标为时间页时置起；
 *   · 退出：此时 s_fn_page 仍是 FN_PAGE_TIME，直接判即可。
 *
 * 【为什么用 ESP_LOGW 而非 LOGI】本工程 STANDBY 每秒刷屏一条 LOGI，
 * W 级更容易在日志里一眼捞出来。
 *
 * 【时序影响】每次渐变最多打 6~8 条，且全在已经黑屏的相位边界上；
 * 但 ESP_LOGW 走 vprintf 有开销（见 BUG-034），故诊断本身会略微加重阻塞——
 * 读数时把这一项算作系统误差，只看量级不抠个位数。
 * ══════════════════════════════════════════════════════════════════════════ */
#define UI_TIME_FADE_DIAG 1 // 1=开启时间页渐变诊断；定位完置 0 或整段删除

#if UI_TIME_FADE_DIAG
/* 本次渐变是否为「时间页」相关：进入方向由 home_enter_fade 置位（那时 s_fn_page
 * 还没切过来），退出方向靠 s_fn_page 判定。两个方向都归一到这个标志上。 */
static bool s_fade_diag_armed = false;
/* 暗态动作开始时刻，用于算阻塞时长（跨两条日志，故提为静态） */
static int64_t s_fade_diag_dark_t0 = 0;
/* 本相位是否已打过首帧日志：FADE_IN 只关心首帧的 elapsed_ms，
 * 不加这个标志会把整段渐亮的每一帧都打出来（约 6~12 条），淹没关键信息。 */
static bool s_fade_diag_in_logged = false;
#define FADE_DIAG_ON() (s_fade_diag_armed)
#else
#define FADE_DIAG_ON() (false)
#endif

/**
 * @brief 显示刷新完成事件回调：为自适应 DARK 置位标志
 *
 * 挂在 lvgl_disp 的 LV_EVENT_REFR_READY 上，每个刷新周期结束触发一次。
 * 只在 DARK 相位期间有意义，其余时间置位也无害（进 DARK 时会重新清零）。
 * 【必须极轻】本回调在 LVGL 刷新收尾处同步调用，不能做任何耗时或加锁操作。
 */
static void ui_fade_refr_ready_cb(lv_event_t *e)
{
    (void)e;
    s_cd_fade_refr_done = true;
}

/* 左右耳切换图标渐暗渐亮状态（变量须在 ui_init 使用点之前定义，故置此而非 fade 块） */
typedef enum
{
    UI_HOME_ICON_FADE_IDLE = 0,
    UI_HOME_ICON_FADE_OUT,  // 渐暗 100→0 ‖ 收起 256→40
    UI_HOME_ICON_FADE_DARK, // 全黑：换图 + 等 flush + 等液晶响应
    UI_HOME_ICON_FADE_IN,   // 渐亮 0→100 ‖ 弹入 40→256
} ui_home_icon_fade_phase_t;
static ui_home_icon_fade_phase_t s_home_icon_fade_phase = UI_HOME_ICON_FADE_IDLE;
static int64_t s_home_icon_fade_start_us = 0;
static lv_timer_t *s_home_icon_fade_tmr = NULL;
static uint8_t s_home_icon_fade_bk = 100; // 本次转场基准亮度（触摸起手时快照）

#define UI_MENU_IDLE_TIMEOUT_MS 20000 // 熄屏时间
/* 主屏幕顶部状态栏（时间 / WiFi / 电量）总开关：1=显示，0=关闭。 \
 * 关闭后既不创建标签也不启动刷新定时器，相关回调内均有 NULL 早退保护，安全。 */
#define UI_SHOW_STATUS_BAR 0 /* 关闭顶部状态栏（时间/WiFi/电量全部不显示）*/

/* ── 触摸调节步进 & 上限宏 ──
 * 修改此处统一控制所有触摸步进和上限值 */
/* 闹钟分钟步进 2026-08-04 由 ±2 改为 ±1（0~59 整分循环），
 * 原 ALARM_MIN_STEP / ALARM_MIN_MAX 两宏随之废弃并删除。 */
/* 倒计时上限。2026-08-04 改为「左耳调十位、右耳调个位」的分位输入后，
 * 数值不再受 ±2 步进约束，可取到任意整数，故秒上限由 58 放宽到 59（真正的分内极限）。
 * CD_MIN_STEP / CD_SEC_STEP 两个步进宏随之废弃，已删除（改动详见 countdown_bump_digit）。 */
#define CD_MIN_MAX 60 ///< 倒计时分钟上限（0~60，60 分=合法）
#define CD_SEC_MAX 59 ///< 倒计时秒钟上限（0~59，60 秒=进位非法）

/* ═══════════════════════════════════════════════════════════════
 * 番茄时钟预设（2026-08-10 需求：倒计时页改造为番茄时钟）
 *
 * 页面布局（320×240）：
 *   顶部标题「番茄时钟」（复用 s_menu_title，不额外建标签）
 *   中部 2 行 × 3 列共 6 个圆角色块，分别为 1/3/5/10/15/30 分钟
 *   底部大号 MM:SS —— 即原有的「自定义倒计时」，逻辑一行未改，仅位置下移
 *
 * 【7 档循环】头部短按在 6 个色块 + 底部自定义之间循环（CD_SEL_CUSTOM 为第 7 档）。
 *   选中色块时：左/右耳短按不生效（预设时长固定），头部长按启动该预设分钟数。
 *   选中自定义时：左/右耳短按恢复原有「左十位、右个位」调值手感，
 *                 分/秒字段切换沿用原 s_cd.editing_sec —— 见 countdown_head_advance 说明。
 *
 * 【改预设值/配色】只改下面这张表即可，数量由 CD_PRESET_COUNT 自动推导，
 *   render/布局/循环全部按表长走，加减档位无需改其它代码。
 * ═══════════════════════════════════════════════════════════════ */
typedef struct
{
    uint8_t minutes;  ///< 预设时长（分钟），启动时按 minutes*60 秒计
    uint32_t color;   ///< 色块背景色（RGB888）
    const char *text; ///< 色块内显示文字，如 "1 min"
} cd_preset_t;

static const cd_preset_t s_cd_presets[] = {
    {1, 0xF4714E, "1 min"},   /* 橙 */
    {3, 0xC6BEF5, "3 min"},   /* 紫 */
    {5, 0xF3B8D0, "5 min"},   /* 粉 */
    {10, 0xAEC6E8, "10 min"}, /* 蓝 */
    {15, 0xB07E6E, "15 min"}, /* 棕 */
    {30, 0x7CBF9C, "30 min"}, /* 绿 */
};
#define CD_PRESET_COUNT (sizeof(s_cd_presets) / sizeof(s_cd_presets[0]))
#define CD_SEL_CUSTOM CD_PRESET_COUNT ///< 选中项下标 == 预设个数 时，表示选中底部「自定义」

/* 色块几何（320×240 实测排布：标题占顶部 ~24px，底部 MM:SS 占 ~60px） */
#define CD_TILE_W 68      ///< 色块宽
#define CD_TILE_H 60      ///< 色块高
#define CD_TILE_GAP_X 12  ///< 色块水平间距
#define CD_TILE_GAP_Y 8   ///< 色块垂直间距
#define CD_TILE_RADIUS 12 ///< 色块圆角半径（需求：需要有倒角）
#define CD_TILE_TOP_Y 34  ///< 第一行色块顶部 Y（自 panel 顶部起算）
/* 选中态只改文字颜色（2026-08-10 需求）：色块背景恒为表内原色，不变暗、不描边。
 * 选中=深色字（在几种浅色块上都够对比），未选中=白色字。 */
#define CD_TILE_TEXT_ON 0x1A1A1A  ///< 选中项文字色（近黑）
#define CD_TILE_TEXT_OFF 0xFFFFFF ///< 未选中项文字色（白）

/* ═══════════════════════════════════════════════════════════════
 * 闹钟编辑状态机（2步：时 → 分）
 *
 * 2026-08-04 需求变更：去掉「重复模式」与「开关」两步。
 *   · 重复模式：内部固定 ALARM_REPEAT_ONCE（只响一次，响完由 reminder 自动禁用）
 *   · 开关：保存即视为开启（enabled=true），不再让用户手动切换
 * 保留 reminder 层的 repeat/enabled 字段不动（NVS 结构不变、后端逻辑不变），
 * 仅在 UI 层写死取值，改动面最小。
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    ALARM_EDIT_HOUR,   // 时
    ALARM_EDIT_MINUTE, // 分
} alarm_edit_state_t;

/* ═══════════════════════════════════════════════════════════════
 * 倒计时状态枚举
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    CD_STATE_SET,
    CD_STATE_RUNNING,
    CD_STATE_EXPIRED,
} cd_state_t;

/* ═══════════════════════════════════════════════════════════════
 * 全局 / 静态变量
 * ═══════════════════════════════════════════════════════════════ */
lv_display_t *lvgl_disp = NULL;
static lv_obj_t *gif_obj = NULL;

/* LVGL/屏幕是否已初始化完成。供其他任务（如按键重置）在调用 lvgl_port_lock 前判断，
 * 避免在 LVGL 未就绪时（如 WiFi 还在重连、UI 尚未起来）触发 lvgl_port_lock 的 assert。
 * 在 lvgl_port_init() 成功后置 true。 */
static bool s_lvgl_ready = false;

/* ── 主界面 GIF 自动循环状态(均在 LVGL 线程读写,无需加锁/原子)──
 *   s_gif_cur_index   : 当前正在播放的 GIF 表索引;-1 表示尚未开始
 *   s_gif_pending_idx : 待切换到的索引;-1 表示当前无待切换(防重复排队)
 *   s_gif_switch_tmr  : 延迟执行 lv_gif_set_src 的 1_3-shot 定时器
 *                       (不能在 LV_EVENT_READY 回调里直接切图,见 main_gif_ready_cb 说明)
 */
static int s_gif_cur_index = -1;
static int s_gif_pending_idx = -1;
/* 开机 logo 静态图对象：盖在 gif_obj 之上，非 NULL 表示 logo 正在显示。
 * 仅 LVGL 线程创建(main_gif_create)与销毁(ui_boot_fade_timer_cb 渐暗到底时)。 */
static lv_obj_t *s_boot_logo_img = NULL;
static volatile bool s_boot_ready = false; // 真正初始化完成信号；任意线程可写，仅 LVGL 线程读
/* 删除开机 logo 的跨线程请求标志：ui_boot_fade_timer_cb 跑在 esp_timer 线程，
 * 绝不能在那里直接 lv_obj_del（非 LVGL 线程操作对象会撞断言 abort，同 BUG-029/030）。
 * 故渐暗到底时只置位，由 main_gif_switch_timer_cb（LVGL 线程）在下一拍真正删除并揭图。 */
static volatile bool s_boot_logo_del_pending = false;
/* 配网提示图对象（ui_show_provision_image 贴的全屏静态图）：仅【首次配网】路径非 NULL。
 * ★必须显式删除，不能像原注释那样指望"被后续主界面覆盖"：主界面 GIF 确实铺满整屏
 *   能盖住它，但【功能盘是居中大图标、不铺满】，盖不住的四角就会把这张全屏图露出来。
 *   且它常驻对象树底层，每次刷屏都要多合成一层 320×240，拖慢渲染。
 * 生命周期：ui_show_provision_image()（配网前，主流程 app_main 线程持锁）创建，
 *           ui_init() 创建主界面 UI 之前持锁删除。已配网设备该指针恒为 NULL。 */
static lv_obj_t *s_provision_img = NULL;
static lv_timer_t *s_gif_switch_tmr = NULL;
/* 情绪触发时指定要切到的 GIF 路径（非随机）。非 NULL 优先于 s_gif_pending_idx。
 * 由 ui_request_emotion_gif()（任意线程）设置，main_gif_switch_timer_cb（LVGL线程）消费。
 * 见 BUG-010：lv_gif_set_src 必须在 LVGL 线程调，故走 pending + 延迟 timer 机制。 */
static const char *volatile s_gif_pending_path = NULL;
/* logo「原地重播」请求：同一个文件循环重播不需要走 set_src 的卸载重装
 * （关文件→释放 draw_buf→重开→重分配→解首帧→全屏重绘），改用 lv_gif_restart。
 * 仍需延后一拍执行：ready_cb 返回后 lv_gif.c:705 会 lv_timer_pause，直接 restart 会被摁死。 */
static volatile bool s_gif_pending_restart = false;

/* ── 初次联网避让窗口 ────────────────────────────────────────────────────────
 * 现象（wdt 误报）：初次联网瞬间 WiFi/TLS/WebSocket 连接 + NVS 写等突发工作集中砸在
 *   CPU0，taskLVGL（GIF 解码优先级仅 5）被反复挤住，此刻若正好在跑 lv_gif_set_src
 *   （读文件 + 解码首帧 + 分配 draw_buf），会把这一拍彻底顶死，导致 IDLE0 连续得不到
 *   运行触发一次 task_wdt（10s 窗口）。只发生一次、联网后即消失。
 * 方案：初次联网成功后开一个短窗口（FIRST_ONLINE_DEFER_MS），期间 main_gif_switch_timer_cb
 *   不真正切图，而是把 timer 重排到窗口之后再执行（保留 pending 不丢弃），让联网突发先过去。
 * 只对【初次】生效：ui_notify_first_online() 只由初次联网主流程调用一次（重连不走）。
 * s_first_online_defer_until_us：避让截止时间(esp_timer 微秒)；0=未武装/已过窗口。 */
#define FIRST_ONLINE_DEFER_MS 2000
static volatile int64_t s_first_online_defer_until_us = 0;

/* ── 对话状态中性 GIF 标志（纯视觉，无舵机/震动）──
 *   s_neutral_active     : true=当前处于对话中（LISTENING/PLAYING），屏幕锁定在状态中性 GIF，
 *                          此时 main_gif_ready_cb 不再自动随机循环（避免待机循环抢图）。
 *   s_gif_pending_is_state: 标记本次 pending_path 是否来自“状态接口”（true）还是“情绪接口”（false）。
 *                          用于 main_gif_switch_timer_cb 里实现“状态 > 情绪”优先级兜底：
 *                          对话中若收到情绪 pending（is_state=false）则丢弃，不抢状态 GIF。
 * 两个标志均由 LVGL 线程 / 状态接口写，跨线程仅做布尔赋值（原子），无需加锁。 */
static volatile bool s_neutral_active = false;
/* 空闲态自动轮播总开关（2026-09-17 调试用）：false = 彻底停掉主界面空闲 GIF 自动轮播
 * + 随附空闲舵机动作（ready_cb 不排下一张、resume_loop 不恢复、开机不补投首张）。
 * 正式产品需 true；调试 GIF/舵机适配时置 false（由 ui_set_idle_carousel_enabled 设置）。 */
static volatile bool s_idle_carousel_enabled = true;
static volatile bool s_gif_pending_is_state = false;

/* 主时钟 UI */
static lv_obj_t *s_clock_d[6];
static lv_obj_t *s_clock_col[2];
static lv_obj_t *s_time_tz_lbl = NULL;
static lv_obj_t *s_time_date_lbl = NULL;
static lv_timer_t *s_main_tick_tmr = NULL;

/* ─── 全局浮动状态栏（挂在 top-layer，所有页面切换都常驻显示）───
 * 布局：[时间 HH:MM | 左上角]  …  [WiFi 信号 | 电量左侧]  [电量% | 右上角]
 * 三个元素均挂 lv_layer_top()，跨页面常驻，独立刷新。
 */
static lv_obj_t *s_battery_lbl = NULL;        ///< 右上角电量标签，纯文字 "85%"
static lv_obj_t *s_status_time_lbl = NULL;    ///< 左上角时间标签 "HH:MM"
static lv_obj_t *s_status_wifi_lbl = NULL;    ///< 电量左侧 WiFi 信号标签
static lv_timer_t *s_battery_tick_tmr = NULL; ///< 周期刷新电量+WiFi 的 LVGL 定时器（5s）
static lv_timer_t *s_status_time_tmr = NULL;  ///< 周期刷新状态栏时间的 LVGL 定时器（1s）

/* 功能菜单 */
static ui_view_t s_view = UI_VIEW_MAIN;
static fn_page_t s_fn_page = FN_PAGE_TIME;

/* 【2026-09-01】主界面空闲链路（自动 GIF 轮播 + 随附舵机动作）当前是否该干活。
 * 原判据是散落三处的裸 `s_view == UI_VIEW_MAIN`，现统一收口并补上「进功能盘渐变期」
 * 这个缺口 —— 完整根因与时序见 s_home_enter_pending 的声明处注释。
 * 三个使用点：main_gif_ready_cb / main_gif_switch_timer_cb / ui_resume_main_gif_loop，
 * 外加 ui_request_emotion_gif（同一判据，一并收口）。 */
static inline bool main_idle_loop_active(void)
{
    /* ★★【2026-09-01 补漏：到期渐变期也必须关闸】★★
     * 【实测日志（闹钟到期）】
     *     218245  到期渐变开始：主界面→闹钟响铃      ← flush 打断当前情绪舵机
     *     218655  情绪动作执行完毕: 8               ← interaction 收到 give，解阻塞
     *     218655  resume_loop 入口: s_view=0        ← s_view 仍是 MAIN，旧判据放行
     *     218655  resume_loop: 恢复循环 → pending_idx=2
     *     218705  timer_cb: 切图 idx=2 + 投递舵机动作 ← 又投出一条新的空闲动作
     *   现象即用户反馈的"闹钟/倒计时结束界面出来了，舵机还在跑上一个动作，跑完才归中"。
     *
     * 【为什么第一版没修到】上一轮只加了 s_home_enter_pending（进功能盘专用），
     *   而到期渐变走的是另一套状态机，此刻该标志为 false —— 两个条件全部放行。
     *   两条路径的病理完全相同：【切页动作被推迟到暗态，s_view 在渐暗期仍是 MAIN】，
     *   所以判据必须把"任何一种正在进行、且终点不是主界面的渐变"都算进来。
     *
     * 【为什么光 pause 轮播 timer 不够】expire_fade_start 里确实 pause 了
     *   s_gif_switch_tmr，但 ui_resume_main_gif_loop() 末尾会 lv_timer_resume()
     *   把它原样唤醒 —— pause 拦不住 resume，必须在判据层把 resume 本身挡掉。 */
    return (s_view == UI_VIEW_MAIN) && !s_home_enter_pending && !s_expire_fade_pending;
}
static lv_obj_t *s_menu_panel = NULL;
static lv_obj_t *s_menu_title = NULL;
static lv_obj_t *s_menu_body = NULL;
static lv_timer_t *s_menu_idle_tmr = NULL;
/* 功能盘专属：居中大图标（复用 s_menu_panel；无文字）。
 * 图片版是 lv_image，占位版是一个圆角色块 lv_obj —— 两者都支持 translate_x 做果冻位移。 */
static lv_obj_t *s_home_icon = NULL;

/* ═══════════════════════════════════════════════════════════════
 * 单层横向功能盘：数据表 + 导航状态
 *   长按耳进入 → 屏幕正中只显示「当前项」一个大图标（无文字），
 *   左右耳带果冻弹动横向切换，头部震动确认进入对应功能并锁定。
 *   复用 s_menu_panel（黑底全屏），新增一个居中图标对象。
 *
 * 混合图标方案（逐项独立）：
 *   每项既有 img（图片指针）又有 color（占位色块颜色）。
 *     - img != NULL → 显示这张图片（PNG 经 LVGL 图片转换器转成 lv_image_dsc_t，
 *       CF_RGB565A8 带透明、建议 ≤80x80，每张约 18KB flash，0 额外 RAM）。
 *     - img == NULL → 退回显示纯色圆块（图还没转好的项先用色块顶着）。
 *   这样可以一张一张地接图：转好一张图就补一个 LV_IMAGE_DECLARE 并把对应项的
 *   img 从 NULL 改成 &icon_xxx，其余项不受影响、始终能编译。
 * ═══════════════════════════════════════════════════════════════ */

/* 已就绪的图片图标（转换生成的 C 数组放在 main/ui/，变量名须为英文）。
 * 还没转好的图标先不声明，对应项 img 填 NULL 用色块占位。 */
LV_IMAGE_DECLARE(dishu); /* 打地鼠（你提供的地鼠图，80x47 RGB565A8） */
LV_IMAGE_DECLARE(picture1);
// LV_IMAGE_DECLARE(rl);   /* 日历图标：内置固件图已停用，改回外挂 Flash S:/img/rl.bin */
// LV_IMAGE_DECLARE(sz);   /* 闹钟图标：内置固件图已停用，改回外挂 Flash S:/img/sz.bin */
// LV_IMAGE_DECLARE(djs1); /* 倒计时图标：内置固件图已停用，改回外挂 Flash S:/img/djs1.bin */
// LV_IMAGE_DECLARE(tq);   /* 天气图标：内置固件图已停用，改回外挂 Flash S:/img/tq.bin */
// LV_IMAGE_DECLARE(sc);   /* 赛车图标：内置固件图已停用，改回外挂 Flash S:/img/sc.bin */
// LV_IMAGE_DECLARE(tyt);  /* 跳一跳图标：内置固件图已停用，改回外挂 Flash S:/img/tyt.bin */
LV_IMAGE_DECLARE(p2);
LV_IMAGE_DECLARE(p3);
LV_IMAGE_DECLARE(p4);
LV_IMAGE_DECLARE(p5);
LV_IMAGE_DECLARE(p6);

/* 功能盘单个条目：图片(可空) + 占位色 + 确认后进入的回调。无底部文字。 */
typedef struct
{
    const lv_image_dsc_t *img; // 内存图片图标（编译进固件的 C 数组）；NULL 则看 img_path
    const char *img_path;      // 外挂 Flash 图片路径，如 "S:/img/tq.bin"；优先级最高
    uint32_t color;            // 占位色块颜色（img 与 img_path 都为空时生效）
    void (*on_enter)(void);    // 头部确认后进入；游戏项包一层 enter_xxx
} home_item_t;

/* ⚠️ 2026-08-12 【临时】全盘统一图标：除倒计时(番茄钟)本身外的 7 个功能，
 *    img_path 全部临时指向番茄钟图 "S:/img/djs1.bin"，用于对比验证换图表现
 *    （8 项显示同一张图时，切换过程中的差异只可能来自刷新时序而非图片内容）。
 *    每行原始指向已就地保留在行尾「原: ...」注释里，恢复时逐行改回即可。
 *    数据结构、条目数量、进入回调、占位色一律未动，功能行为不受影响。 */
static const home_item_t s_home_items[] = {
    /* 时间页排第一：看时间是最高频需求，功能盘默认停在第 0 项，零操作即可达。
     * 拆分前时间只以小字挤在日历页右上角，是全机唯一的时间出口。 */
    {NULL, "S:/img/sj.bin", 0x4FC3F7, app_enter_time}, /* 时间【临时=番茄钟图】原: "S:/img/sj.bin" */
    // {&rl, NULL, 0x4FC3F7, app_enter_calendar},   /* 日历：内置固件图（已停用，对比用） */
    {NULL, "S:/img/rl.bin", 0x4FC3F7, app_enter_calendar}, /* 日历【临时=番茄钟图】原: "S:/img/rl.bin" */
    // {&sz, NULL, 0xFFB300, app_enter_alarm},      /* 闹钟：内置固件图（已停用，对比用） */
    {NULL, "S:/img/sz.bin", 0xFFB300, app_enter_alarm}, /* 闹钟【临时=番茄钟图】原: "S:/img/sz.bin" */
    // {&djs1, NULL, 0x81C784, app_enter_countdown},/* 倒计时：内置固件图（已停用，对比用） */
    {NULL, "S:/img/djs1.bin", 0x81C784, app_enter_countdown}, /* 倒计时(番茄钟)：本项即原图，未改 ✅ 240x240 */
    // {&tq, NULL, 0xFFD54F, app_enter_weather},    /* 天气：内置固件图（已停用，对比用） */
    {NULL, "S:/img/tq.bin", 0xFFD54F, app_enter_weather}, /* 天气【临时=番茄钟图】原: "S:/img/tq.bin" */
    {NULL, "S:/img/rl.bin", 0x8D6E63, enter_whack},       /* 打地鼠【临时=番茄钟图】原: {&dishu, NULL, ...} 内置图 */
    // {&sc, NULL, 0xE57373, enter_race},           /* 赛车：内置固件图（已停用，对比用） */
    {NULL, "S:/img/sz.bin", 0xE57373, enter_race}, /* 赛车【临时=番茄钟图】原: "S:/img/sc.bin" */
    // {&tyt, NULL, 0x9575CD, enter_jump},          /* 跳一跳：内置固件图（已停用，对比用） */
    {NULL, "S:/img/tq.bin", 0x9575CD, enter_jump}, /* 跳一跳【临时=番茄钟图】原: "S:/img/tyt.bin" */
};
#define HOME_ITEM_COUNT (sizeof(s_home_items) / sizeof(s_home_items[0]))

/* ── 功能盘图标 PSRAM 预读缓存 ────────────────────────────────────────────
 * 图标源是外挂 SPI Flash 的 .bin（"S:/img/xx.bin"）。LVGL 的 bin 解码器对文件源
 * 不整图载入，而是绘制时逐行 get_area；且图标是 RGB565A8（色数据与 A8 通道在
 * 文件里分区存放），每行要 seek+read 两次，构成换图的主要耗时。故开机时一次性
 * 读进 PSRAM，之后换图零文件 I/O。
 * 7 张 100x100 RGB565A8 ≈ 30KB/张，合计约 210KB，走 PSRAM 不占内部 SRAM。
 * 分配失败不致命：缓存未命中时自动回退到原文件路径，功能不受影响。
 * 只被 LVGL 软件渲染读取、不经 DMA，故不涉及 BUG-017 那类 DMA 取不到 PSRAM 的问题。 */
typedef struct
{
    lv_image_dsc_t dsc; // 交给 lv_image_set_src 的内存图描述符
    uint8_t *data;      // PSRAM 中的像素数据（含 A8 分区），NULL=未缓存
} home_icon_cache_t;

static home_icon_cache_t s_home_icon_cache[HOME_ITEM_COUNT];
static bool s_home_icon_cache_ready = false;

/* 功能盘导航状态 */
static int s_home_idx = 0;                     // 当前选中项下标
static volatile bool s_home_animating = false; // 头部确认弹动进行中（防连点打断）
/* HOME_PUNCH_PX 定义在下方左右耳切换处（当前为 20，随 2026-08-12 脏区排查调整），
 * 此处不再重复定义，避免与那份带排查记录的定义打架。 */

/* ── 【临时调试】翻页 flush 追踪探针（2026-08-12，定位功能盘"两图同框"）─────
 *
 * 【要回答的三个问题】一次实测同时定案：
 *   ① 一次翻页产生几块失效区域、各在哪
 *      → 若出现两块 x 起点相差约 20px 的同尺寸区域，即"新旧图并排画在两处"
 *   ② 这些区域最终被拆成几次 FLUSH、每次多大
 *      → draw buffer 上限 12800 像素；PARTIAL 模式按「buffer ÷ 脏区宽度」算出
 *        每批能画多少行，脏区一旦变宽就会被横向切成多条带分批上屏
 *   ③ 相邻两次 FLUSH 的时间间隔
 *      → 几十 ms = 分块上屏，肉眼可见"旧图被一条条擦掉"；
 *        ~0ms 且只有一次 FLUSH = LVGL 层清白，问题在面板扫描（撕裂）
 *
 * 【零时序影响】记录期只写内存 + esp_timer_get_time()，不打一个字节串口日志。
 * 打印统一推迟到切换结束后由 s_ftrace_tmr 一次性完成。探针本身若在热路径上
 * 做 I/O 就会改变时序、反过来掩盖问题，这是 [BUG-035] 排查中踩过的坑，刻意规避。
 *
 * 【用完即删】定位完成后把 UI_FLUSH_TRACE 置 0，整块编译剔除，零运行期开销。
 * ─────────────────────────────────────────────────────────────────────────── */
/* 2026-08-18：置 0 关闭。排查「从功能盘进游戏、前一秒动画卡住」时定位到本探针即元凶：
 * ui_ftrace_dump_cb 是 lv_timer 回调（跑在 taskLVGL 上），在按下确认后 UI_FTRACE_DUMP_MS
 * = 600ms 一次性打印约 51 行日志；console 走 UART0@115200 且未装驱动=轮询阻塞发送，
 * 约 5.6KB / 11520 B/s ≈ 480ms 内 LVGL 线程完全停摆 → 进度条停走、跳一跳格子悬在空中。
 * 探针原设计（见上方"零时序影响"）是为【翻页】准备的：翻页 200ms 就结束，600ms 后打印
 * 确实打不到被测过程；但游戏入场后是 3 秒开场动画，600ms 正落在动画中段，反而打进去了。 */
#define UI_FLUSH_TRACE 0

#if UI_FLUSH_TRACE
#define UI_FTRACE_MAX 48      // 单次翻页记录条数上限；写满即停（保首不保尾，开头才是关键）
#define UI_FTRACE_DUMP_MS 600 // 翻页后多久打印，须覆盖整个切换过程

/* 一条事件记录。刻意用 int16_t 压小，48 条约 0.6KB，放 .bss 不占堆。 */
typedef struct
{
    uint8_t kind; // 0=失效 1=FLUSH开始 2=FLUSH结束 3=REFR开始 4=REFR结束
    int16_t x1, y1, x2, y2;
    int64_t t_us; // 事件发生时刻（绝对微秒）
} ui_ftrace_ent_t;

static ui_ftrace_ent_t s_ftrace[UI_FTRACE_MAX];
static volatile uint8_t s_ftrace_n = 0;
static volatile bool s_ftrace_on = false;
static int64_t s_ftrace_t0 = 0;         // 翻页触发时刻，日志里以此为 0 点
static lv_timer_t *s_ftrace_tmr = NULL; // 延迟打印用；复用同一句柄，只 pause/resume

/* LVGL 事件回调：只落一条记录就返回，绝不做任何 I/O。
 * 在 lv_timer_handler 线程内被调用，与业务同线程，无需加锁。 */
static void ui_ftrace_evt_cb(lv_event_t *e)
{
    if (!s_ftrace_on || s_ftrace_n >= UI_FTRACE_MAX)
        return;

    lv_event_code_t code = lv_event_get_code(e);
    ui_ftrace_ent_t *ent = &s_ftrace[s_ftrace_n++];
    ent->t_us = esp_timer_get_time();

    switch (code)
    {
    case LV_EVENT_INVALIDATE_AREA:
        ent->kind = 0;
        break;
    case LV_EVENT_FLUSH_START:
        ent->kind = 1;
        break;
    case LV_EVENT_FLUSH_FINISH:
        ent->kind = 2;
        break;
    case LV_EVENT_REFR_START:
        ent->kind = 3;
        break;
    default: // LV_EVENT_REFR_READY
        ent->kind = 4;
        break;
    }

    /* 只有前三种带 lv_area_t* 参数（lv_refr.c:321 / :1431 / :1439）；
     * REFR_START/READY 的 param 恒为 NULL（lv_refr.c:393 / :445），不可解引用。 */
    const lv_area_t *a = (ent->kind <= 2) ? (const lv_area_t *)lv_event_get_param(e) : NULL;
    if (a != NULL)
    {
        ent->x1 = (int16_t)a->x1;
        ent->y1 = (int16_t)a->y1;
        ent->x2 = (int16_t)a->x2;
        ent->y2 = (int16_t)a->y2;
    }
    else
    {
        ent->x1 = ent->y1 = ent->x2 = ent->y2 = 0;
    }
}

/* 延迟打印：切换早已结束，此处随便打日志都不影响被测时序 */
static void ui_ftrace_dump_cb(lv_timer_t *t)
{
    lv_timer_pause(t);
    s_ftrace_on = false;

    static const char *KIND[] = {"失效    ", "FLUSH开始", "FLUSH结束", "REFR开始 ", "REFR结束 "};
    const uint8_t n = s_ftrace_n;

    ESP_LOGW(TAG, "════ 翻页 flush 追踪：共 %u 条（buffer 上限 %d 像素）════",
             (unsigned)n, BSP_LCD_WIDTH * BSP_LCD_HEIGHT / 6);

    int64_t prev_us = s_ftrace_t0;
    for (uint8_t i = 0; i < n; i++)
    {
        const ui_ftrace_ent_t *e = &s_ftrace[i];
        const long rel_ms = (long)((e->t_us - s_ftrace_t0) / 1000);
        const long gap_ms = (long)((e->t_us - prev_us) / 1000);
        prev_us = e->t_us;

        if (e->kind <= 2)
        {
            const int w = e->x2 - e->x1 + 1;
            const int h = e->y2 - e->y1 + 1;
            ESP_LOGW(TAG, "#%02u %s +%4ldms (间隔%3ldms) (%d,%d)-(%d,%d) %dx%d = %d 像素",
                     (unsigned)i, KIND[e->kind], rel_ms, gap_ms,
                     e->x1, e->y1, e->x2, e->y2, w, h, w * h);
        }
        else
        {
            ESP_LOGW(TAG, "#%02u %s +%4ldms (间隔%3ldms)",
                     (unsigned)i, KIND[e->kind], rel_ms, gap_ms);
        }
    }
    if (n >= UI_FTRACE_MAX)
        ESP_LOGW(TAG, "⚠️ 记录已写满 %d 条，后续事件被丢弃", UI_FTRACE_MAX);
    ESP_LOGW(TAG, "════ 追踪结束 ════");

    s_ftrace_n = 0;
}

/* 翻页入口调用：清空并开启记录，同时把打印 timer 重新计时。
 * 连点翻页时 lv_timer_reset 会把打印推后，避免打到一半又被新一轮覆盖。 */
static void ui_ftrace_begin(void)
{
    s_ftrace_on = false; // 先关，避免清空过程中被事件写入
    s_ftrace_n = 0;
    s_ftrace_t0 = esp_timer_get_time();
    s_ftrace_on = true;

    if (s_ftrace_tmr != NULL)
    {
        lv_timer_reset(s_ftrace_tmr);
        lv_timer_resume(s_ftrace_tmr);
    }
}
#endif /* UI_FLUSH_TRACE */

/* ───────────────────────────────────────────────────────────────────────────
 * 【临时调试】持续 FPS 统计：与上面 UI_FLUSH_TRACE（单次翻页逐事件追踪）不同，
 * 这里统计的是任意时段内的稳定帧率（GIF 播放、时钟走字等持续场景），
 * 每次 LV_EVENT_FLUSH_FINISH 计数+累计像素，每 1000ms 打印一次并清零。
 * 定位完成后把 UI_FPS_MONITOR 置 0，整块编译剔除，零运行期开销。
 * ─────────────────────────────────────────────────────────────────────────── */
/* 2026-08-18：一并置 0。同属"在 LVGL 线程上打日志"这一类：每秒 1 行约 130 字节，
 * UART0@115200 阻塞发送约 11ms —— 比 UI_FLUSH_TRACE 轻得多，但游戏全程每秒一次。 */
#define UI_FPS_MONITOR 0

#if UI_FPS_MONITOR
static volatile uint32_t s_fps_flush_count = 0;
static volatile uint64_t s_fps_pixel_sum = 0;
static int64_t s_fps_window_t0 = 0;

static void ui_fps_evt_cb(lv_event_t *e)
{
    const lv_area_t *a = (const lv_area_t *)lv_event_get_param(e);
    if (a != NULL)
    {
        s_fps_pixel_sum += (uint64_t)(a->x2 - a->x1 + 1) * (uint64_t)(a->y2 - a->y1 + 1);
    }
    s_fps_flush_count++;

    int64_t now = esp_timer_get_time();
    if (s_fps_window_t0 == 0)
    {
        s_fps_window_t0 = now;
        return;
    }
    int64_t elapsed_ms = (now - s_fps_window_t0) / 1000;
    if (elapsed_ms >= 1000)
    {
        uint32_t fps = (uint32_t)((uint64_t)s_fps_flush_count * 1000 / (uint64_t)elapsed_ms);
        ESP_LOGW("UI_FPS", "实测: %lu 次flush/%lldms ≈ %lu FPS，累计 %llu 像素（%.1f%%屏/次）",
                 (unsigned long)s_fps_flush_count, (long long)elapsed_ms, (unsigned long)fps,
                 (unsigned long long)s_fps_pixel_sum,
                 s_fps_flush_count ? (double)s_fps_pixel_sum / s_fps_flush_count / (BSP_LCD_WIDTH * BSP_LCD_HEIGHT) * 100.0 : 0.0);
        s_fps_flush_count = 0;
        s_fps_pixel_sum = 0;
        s_fps_window_t0 = now;
    }
}
#endif /* UI_FPS_MONITOR */

/* 闹钟编辑上下文 */
static struct
{
    alarm_edit_state_t state;
    uint8_t hour;
    uint8_t minute;
    alarm_repeat_t repeat;
    bool enabled;
} s_edit;

/* ── 闹钟响铃显示态 ──
 * true 时闹钟页画「红色时间 +『闹钟响铃』」的到期画面，与番茄时钟到期画面对齐。
 * 仅影响渲染，不参与任何闹钟数据/触发逻辑；由 ui_show_alarm_ringing() 置位，
 * 由 s_alarm_ring_tmr 在 ALARM_RING_HOLD_MS 后自动清除并回到常规闹钟页。 */
static bool s_alarm_ringing = false;
static lv_timer_t *s_alarm_ring_tmr = NULL;
/* 响铃提示停留时长（ms）——需求：响铃画面要一直守到
 * 【震动全部结束 + 闹钟本身完全结束】，之后才进入闹钟设置界面。
 * 由响铃参数（周期/次数/单次震动时长）推导，reminder.h 里改宏本值自动跟随。 */
#define ALARM_RING_HOLD_TAIL_MS 500 ///< 闹钟结束后画面额外停留的余量（ms）
#define ALARM_RING_HOLD_MS                                 \
    ((ALARM_RING_MAX_COUNT - 1) * ALARM_RING_INTERVAL_MS + \
     ALARM_RING_VIBRATE_MS + ALARM_RING_HOLD_TAIL_MS)

/* 闹钟页 UI 对象（单闹钟直接展示） */
static lv_obj_t *s_alarm_time_lbl = NULL;
static lv_obj_t *s_alarm_repeat_lbl = NULL;
static lv_obj_t *s_alarm_status_lbl = NULL;
static lv_obj_t *s_alarm_hint_lbl = NULL;

/* 天气页 UI 对象（2026-08 改版：左侧大图标 + 右侧温度/体感/风级三行，带分隔线） */
static lv_obj_t *s_wx_icon = NULL;      ///< 左侧 128×128 天气图标（RGB565A8）
static lv_obj_t *s_wx_temp_lbl = NULL;  ///< 大号温度"23"（montserrat_48）
static lv_obj_t *s_wx_deg_lbl = NULL;   ///< "°C"（font_cn_16，贴温度右上）
static lv_obj_t *s_wx_c_lbl = NULL;     ///< 温度单位补充标签
static lv_obj_t *s_wx_feels_lbl = NULL; ///< "体感"中文标签行（16px）
static lv_obj_t *s_wx_feels_en = NULL;  ///< "Feels Like"英文标签行（12px）
static lv_obj_t *s_wx_feels_val = NULL; ///< 体感数值行，如"30°C"
static lv_obj_t *s_wx_wind_lbl = NULL;  ///< "风级"中文标签行（16px）
static lv_obj_t *s_wx_wind_en = NULL;   ///< "Wind"英文标签行（12px）
static lv_obj_t *s_wx_wind_val = NULL;  ///< 风级数值行，如"东北风2级"
static lv_obj_t *s_wx_line1 = NULL;     ///< 温度与体感之间的分隔线
static lv_obj_t *s_wx_line2 = NULL;     ///< 体感与风级之间的分隔线

/* 闹钟编辑 UI 对象 */
static lv_obj_t *s_edit_panel = NULL;
static lv_obj_t *s_edit_gray_l = NULL;   ///< 左侧灰块（nz.png 示例 HH 背景，133×158，x=22）
static lv_obj_t *s_edit_gray_r = NULL;   ///< 右侧灰块（nz.png 示例 MM 背景，133×158，x=165）
static lv_obj_t *s_edit_sep_line = NULL; ///< HH/MM 的拦腰横切分割线（背景色，厚 3，把数字上下切断）
/* 2026-08-16：HH/MM 各自拆成「十位/个位」两个独立标签，各自固定锚点在灰块的
 * 隐形中线两侧（十位右边缘贴中线、个位左边缘贴中线），调整某一位时只有那一个
 * 标签的宽度变化、按自己的固定边对齐重新收缩/展开，另一位纹丝不动。
 * 若用单个 label 装两位数字再整体居中，字体不等宽（"1"比"0"窄）会导致改一位时
 * 两位一起被居中逻辑重新分配位置，出现「调一位、另一位跟着挪」的联动感。 */
static lv_obj_t *s_edit_hour_lbl = NULL;      ///< HH 十位
static lv_obj_t *s_edit_hour_ones_lbl = NULL; ///< HH 个位
static lv_obj_t *s_edit_pm_lbl = NULL;        ///< "PM" 指示（nz.png 示例，左下角）
static lv_obj_t *s_edit_colon_lbl = NULL;
static lv_obj_t *s_edit_min_lbl = NULL;      ///< MM 十位
static lv_obj_t *s_edit_min_ones_lbl = NULL; ///< MM 个位

/* 【2026-08-16】闹钟保存确认动效的「退出阶段」状态：整屏背光闪够 3 次后进入，
 * 驱动「趁黑切页 → 暗态等 flush/液晶响应 → 渐亮」三段收尾。
 * 声明放这里（而非动效实现处）是因为 alarm_edit_enter 要复位它，那里更早。 */
typedef enum
{
    ALARM_EXIT_PHASE_NONE = 0, // 未进入退出阶段（仍在闪烁）
    ALARM_EXIT_PHASE_DARK,     // 已切页，暗态停留等 flush + 液晶响应
    ALARM_EXIT_PHASE_IN,       // 渐亮 0→默认亮度
} alarm_exit_phase_t;
static alarm_exit_phase_t s_alarm_exit_phase = ALARM_EXIT_PHASE_NONE;
static uint32_t s_alarm_exit_elapsed = 0; ///< 退出阶段内已推进的毫秒数
/* 中央大数字用两个标签轮换：切换字段时「旧值缩小飞回底部」与「新值放大飞到中央」
 * 同时在屏上，单个标签无法同时呈现两种状态，故需要一进一出两份。 */
static lv_obj_t *s_edit_big_lbl[2] = {NULL, NULL};
static uint8_t s_edit_big_cur = 0; ///< 当前停在中央的那个标签下标（另一个用于飞出）
static lv_obj_t *s_edit_arrow_l = NULL;
static lv_obj_t *s_edit_arrow_r = NULL;
static lv_timer_t *s_edit_zoom_tmr = NULL;
static bool s_edit_zoomed = false;    ///< true=已转场（完整时间在底部、中央显示大数字）
static bool s_edit_switching = false; ///< true=时/分切换动画进行中（此时 render 不得覆盖插值位置）
static lv_obj_t *s_edit_repeat_lbl = NULL;
static lv_obj_t *s_edit_enable_lbl = NULL;
static lv_obj_t *s_edit_hint_lbl = NULL; ///< 「X小时X分钟后响铃」提示（标题正下方）

/* 倒计时上下文 */
static struct
{
    cd_state_t state;
    uint8_t minutes;
    uint8_t seconds;  ///< 设置阶段附加的秒数（0~CD_SEC_MAX）
    bool editing_sec; ///< 当前正在编辑秒钟（true）还是分钟（false）
    int timer_id;
    lv_timer_t *tick_tmr;
    /* 2026-08-10 番茄时钟：当前选中项。0 ~ CD_PRESET_COUNT-1 = 6 个预设色块，
     * == CD_SEL_CUSTOM 时选中底部自定义 MM:SS。minutes/seconds/editing_sec
     * 三个字段的含义与用法完全不变，仅在选中自定义档时才由左/右耳改写。 */
    uint8_t sel;
    /* 2026-08-16：自定义档默认值由 15:00 改为 00:00（需求：默认从零开始调） */
} s_cd = {.state = CD_STATE_SET, .minutes = 0, .seconds = 0, .editing_sec = false, .timer_id = -1, .tick_tmr = NULL, .sel = 0};

/* 倒计时 UI 对象 */
static lv_obj_t *s_cd_time_lbl = NULL;  ///< 运行/到期状态整体时间标签（MM:SS）
static lv_obj_t *s_cd_min_lbl = NULL;   ///< 设置状态：分钟标签（可独立上色）
static lv_obj_t *s_cd_colon_lbl = NULL; ///< 设置状态：冒号标签
static lv_obj_t *s_cd_sec_lbl = NULL;   ///< 设置状态：秒钟标签（可独立上色）
static lv_obj_t *s_cd_hint_lbl = NULL;
static lv_obj_t *s_cd_state_lbl = NULL;
/* 番茄时钟 6 个预设色块（含各自的文字子标签，随父对象自动销毁，无需单独存句柄） */
static lv_obj_t *s_cd_tiles[CD_PRESET_COUNT] = {NULL};
/* 到期画面停留计时器：到期后守 CD_EXPIRE_HOLD_MS 再自动复位回设置态 */
static lv_timer_t *s_cd_expire_tmr = NULL;
#define CD_EXPIRE_HOLD_MS 5000 ///< 到期画面停留时长（ms）

// 1 为启用月历组件，0 为回退到原本的文字显示
#define CONFIG_UI_USE_CALENDAR 1
/* 在 ui_port.c 的全局变量区域添加 */
#if CONFIG_UI_USE_CALENDAR
/* ── 整月点阵日历（2026-08-10 改版）───────────────────────────────────
 * 版式参照设计图：左侧大号日号 + 英文星期缩写，右侧用圆点阵列表示整月。
 *   · 点阵 7 列（周一~周日）× 最多 6 行，按真实星期列对齐（1 号落在它真正的星期列）
 *   · 灰点 = 已过去的日期；白点 = 今天及以后
 *   · 顶部 "< YYYY Mon >" 仅作装饰，不做翻月交互（左右耳在各功能页语义不同，避免冲突）
 * 原「上行星期 + 下行日期」的单周条已整体废弃；原右上角小时钟随时间页独立而移除。 */
#define CAL_WEEK_DAYS 7                                                   ///< 固定 7 列（周一~周日）
#define CAL_DOT_ROWS 6                                                    ///< 最多 6 行即可覆盖任意月份（31 天 + 最多偏移 6 列）
#define CAL_DOT_SIZE 14                                                   ///< 圆点直径
#define CAL_DOT_GAP_X 26                                                  ///< 列间距（含点径）：7 列总宽 = 6*26+14 = 170px
#define CAL_DOT_GAP_Y 32                                                  ///< 行间距（含点径）：6 行总高 = 5*32+14 = 174px
#define CAL_BIGDAY_SCALE 175                                              ///< 大号日号等比缩放（256=100%）
#define CAL_BIGDAY_RAW_W 156                                              ///< 两位数字在 font_num_96 下的原始布局宽度
#define CAL_WD_DX (4 - CAL_BIGDAY_RAW_W * (256 - CAL_BIGDAY_SCALE) / 256) ///< 星期缩写贴靠补偿
#define CAL_DOT_COLOR_PAST 0x9A9A9A                                       ///< 灰：已过去的日期
#define CAL_DOT_COLOR_FUTURE 0xFFFFFF                                     ///< 白：今天及以后

static lv_obj_t *s_calendar = NULL;       ///< 日历页根容器
static lv_obj_t *s_cal_ym_lbl = NULL;     ///< 顶部 "< 2026 Aug >"
static lv_obj_t *s_cal_bigday_lbl = NULL; ///< 左侧大号日号 "08"
static lv_obj_t *s_cal_wd_lbl = NULL;     ///< 大号日号右下角英文星期 "FRI"
/* ── 点阵：42 个 lv_obj → 单对象 + DRAW 事件手绘（2026-08-19）────────────────
 *
 * 【为什么改】原实现每格一个 lv_obj_create + 4 条 style，实测（[MEMPROBE]）日历页
 *   首次进入吃掉 11124 B 内部 SRAM，占五个功能页总占用 26976 B 的 41%，是最大单页。
 *   换算约 240 B/格——lv_obj 本体 52~56B 之外，local_style 数组、以及 lv_obj_create
 *   默认带滚动属性而分配的 spec_attr 才是大头。
 *
 * 【为什么这么改】42 个格子没有任何交互（无事件回调/无动画/无触摸），
 *   状态只有三种：不画 / 灰 / 白。这种"纯静态图元"用对象树承载纯属浪费——
 *   既吃内存，又让 LVGL 每帧多做 42 份样式解析、布局计算和脏区合并。
 *   改为一个容器对象 + LV_EVENT_DRAW_MAIN 里直接画 42 个圆：
 *     内存：42 个对象 → 1 个对象 + 42 字节状态数组
 *     渲染：少 42 个对象参与布局/脏区，对 [[BUG-040]] 那类带宽瓶颈是减负
 *
 * 【状态数组取代对象数组】原来"隐藏/设色"直接改对象属性，现在改成写这张表，
 *   由 draw 回调读表决定每格画不画、画什么色。语义一一对应，无行为变化。 */
typedef enum
{
    CAL_DOT_NONE = 0, ///< 空位（月初/月末不属于本月的格子），不画
    CAL_DOT_PAST,     ///< 已过去的日期 → 灰
    CAL_DOT_FUTURE,   ///< 今天及以后   → 白
} cal_dot_state_t;

static lv_obj_t *s_cal_dot_layer = NULL;                   ///< 点阵绘制层（单个对象，替代原 42 个）
static uint8_t s_cal_dot_st[CAL_DOT_ROWS * CAL_WEEK_DAYS]; ///< 每格状态，行优先，值为 cal_dot_state_t
static lv_coord_t s_cal_dot_x0 = 0, s_cal_dot_y0 = 0;      ///< 点阵左上角屏幕坐标（draw 回调要用）

static const char *const s_cal_mon_en[12] = {
    "Jan", "Feb", "Mar", "Apr", "May", "Jun",
    "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
static const char *const s_cal_wd_en[7] = {"SUN", "MON", "TUE", "WED", "THU", "FRI", "SAT"};

/** 求指定年月的天数（含闰年判断） */
static int cal_days_in_month(int year, int mon0)
{
    static const int t[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
    if (mon0 == 1)
    {
        bool leap = (year % 4 == 0) && ((year % 100 != 0) || (year % 400 == 0));
        return leap ? 29 : 28;
    }
    return t[mon0];
}
#endif /* CONFIG_UI_USE_CALENDAR：日历定义区结束 */

/* ── 纯时间页（2026-08-10 从原「时钟+日历」页拆出）──────────────────────
 * 版式参照设计图：居中大号 HH:MM（蓝），右上角两行小字——
 *   第一行 "SUN 2"（英文星期 + 日号，蓝）、第二行 "30°"（温度，白）。
 * 温度取 reminder 的天气缓存；该缓存在 reminder_init 时已从 NVS 载入
 * （reminder.c: nvs_load_weather_data），故断网/未联网时显示上次联网的数据。 */
#define TIME_CLOCK_COLOR 0x7EA6F0 ///< 大号时间与星期的蓝色（对齐设计图）
/* 版式偏移（2026-08-10 按用户指定值）：改这三个宏即可整体微调，无需动布局代码 */
#define TIME_CLOCK_X 20 ///< 大号时间距屏幕左边缘（垂直方向仍居中）。10→20 整体右移 10px
/*
 * 【2026-08-14 换回 215px MiSans + transform_scale_x 横向压瘦（用户指定，先压）】
 * 还原 2026-08-12 之前的原始方案：215px MiSans-Regular（外挂 Flash numbig.bin）
 * + transform_scale_x 把字横向压瘦成"瘦金体"版式，字高不变，只压横向。
 * 原始参数见 docs/功能盘五模块代码摘录.md，宏 TIME_CLOCK_SCALE_X=115。
 *
 * 【坑点】transform 走中间图层重采样，比原生绘制慢（2026-08-12 因此换过 Bebas
 * Neue 窄体，见 git 历史）。本次为用户指定方案，先压看效果，卡顿可回退。
 *
 * 【坑点】lv_font_conv 默认开启 RLE 压缩，但项目 lv_conf.h 未开 LV_USE_FONT_COMPRESSED，
 * 压缩版 bin 会导致 "Couldn't get the bitmap of a glyph"（屏幕上完全看不到数字）。
 * 转换时必须加 --no-compress。
 */
#define TIME_CLOCK_SCALE_X 115 ///< 大号时间横向压缩比例（256=100%，只压横向）
#define TIME_CORNER_X 15       ///< 右上角两行小字距屏幕右边缘（5→15，往屏幕中间收 10px）
#define TIME_CORNER_Y 20       ///< 右上角第一行（星期+日号）距屏幕顶部
/* 【2026-08-18 已停用】右上角两行小字原为「montserrat_48 打底 + transform 等比
 * 缩放」（256 = 100% = 48px，108/256 ≈ 42% → 字高 20.25px）。
 * 现改用原生 20px 外挂字体 font_cn_20_get()（S:/font/f20.bin），不再走 transform：
 *   · 为什么能换：解析 f20.bin 的 cmap 确认 U+0020~U+007F 全 96 个 ASCII 连续覆盖
 *     （星期缩写字母、数字、空格全有）+ 稀疏表含 °(U+00B0)，字号正好 20px；
 *   · 为什么要换：LVGL 9 对带 transform 的对象要先渲染到临时 layer 再重采样，
 *     这两行小字各付一次，是时间页整屏重绘（实测 DARK 256ms）里最不划算的开销；
 *   · 视觉代价：20.25px → 20px，差 0.25px，肉眼无差别。
 * 原注释里「不用 font_cn_32 是因为它不含 °」的结论对 f20.bin 不适用（f20 含 °）。
 * 本宏现已无人使用，保留仅为记录原始设计值与便于回退。 */
#define TIME_CORNER_SCALE 108
/* 两行小字的行距：字高 20px + 2px 间隙。
 * 原为 (48 * TIME_CORNER_SCALE / 256 + 2)，换原生 20px 字体后解耦成常量——
 * 数值不变（48×108/256+2 = 22），但不再让人误以为缩放仍然生效。 */
#define TIME_CORNER_LINE_H ((20 + 2))

/* 大号时间字体：外挂 Flash lv_binfont_create 懒加载，源文件
 * assets/font/numbig.bin。【2026-08-14】改回 MiSans-Regular 215px（即备份
 * numbig.bin.bak_215px_misans 的内容），配合上方 transform_scale_x/y 压瘦。
 * 原 Bebas Neue 145px 版本已由 numbig.bin.bak_215px_misans 之前的旧文件体系取代。
 * 体积约 84KB，加载策略不变：懒加载 + 失败回退。 */
#define TIME_CLOCK_FONT_PATH "S:/font/numbig.bin"
static lv_font_t *s_font_time_clock = NULL;  ///< 加载成功的 215px 时间字体；NULL=未加载/失败
static bool s_font_time_clock_tried = false; ///< 已尝试过（失败也不重试，避免反复读盘）

static const lv_font_t *time_big_font_get(void)
{
    if (!s_font_time_clock_tried)
    {
        s_font_time_clock_tried = true;
        s_font_time_clock = lv_binfont_create(TIME_CLOCK_FONT_PATH);
        if (s_font_time_clock == NULL)
            ESP_LOGW(TAG, "215px 时间字体加载失败(%s)，回退 montserrat_48", TIME_CLOCK_FONT_PATH);
        else
            ESP_LOGI(TAG, "215px 时间字体已加载: %s", TIME_CLOCK_FONT_PATH);
    }
    return s_font_time_clock ? (const lv_font_t *)s_font_time_clock : &lv_font_montserrat_48;
}

/* 天气页 "°C" 专用 48px 字体（外挂 Flash）。源文件 assets/font/f48big.bin，
 * 仅含 ° 与 C 两个字形，824 字节。
 *
 * 【为什么要单独做】温度数字用 lv_font_montserrat_48，但它是纯 ASCII 字体，
 * 不含 °（U+00B0）；font_cn_16 虽含 ° 但只有 16px，配 48px 数字显得过小；
 * font_cn_32 的字符集只有 30 来个游戏用字，同样没有 °。故只能外挂一份。
 *
 * 【为什么不编进固件】与 numbig.bin 同理，放外挂不占 app 分区；且本文件仅 824B，
 * 加载进 RAM 的开销可忽略。加载/兜底策略完全对齐 time_big_font_get()。 */
#define WX_DEG_FONT_PATH "S:/font/f48big.bin"
static lv_font_t *s_font_wx_deg = NULL;  ///< 加载成功的 48px °C 字体；NULL=未加载/失败
static bool s_font_wx_deg_tried = false; ///< 已尝试过（失败也不重试，避免反复读盘）

/**
 * @brief 取天气页 "°C" 字体：外挂加载成功用 48px，否则回退内置 font_cn_16
 *
 * 【坑点】同 time_big_font_get()：lv_binfont_create 失败返回 NULL，
 * 直接交给 lv_obj_set_style_text_font 会在渲染时空指针崩溃，必须兜底。
 * 回退到 16px 只是 ° 变小，不影响可用性——外挂 Flash 可能没烧或版本不对。
 */
static const lv_font_t *wx_deg_font_get(void)
{
    if (!s_font_wx_deg_tried)
    {
        s_font_wx_deg_tried = true;
        s_font_wx_deg = lv_binfont_create(WX_DEG_FONT_PATH);
        if (s_font_wx_deg == NULL)
            ESP_LOGW(TAG, "°C 48px 字体加载失败(%s)，回退 font_cn_16", WX_DEG_FONT_PATH);
        else
            ESP_LOGI(TAG, "°C 48px 字体已加载: %s", WX_DEG_FONT_PATH);
    }
    return s_font_wx_deg ? (const lv_font_t *)s_font_wx_deg : &font_cn_16;
}

/* 天气页温度大号字体（外挂 Flash）。源文件 assets/font/fw50.bin，
 * MiSans-Demibold 63px（数字高约 51px），含 "0123456789:°C"。
 * 2026-08-14 改为外挂 .bin 加载，替代此前编译进固件的 font_wx_50.c（省 app 分区）。
 * 加载/兜底策略对齐 wx_deg_font_get()。
 * ⚠️ 文件名须 ≤8.3 短名（FAT 未开 LFN），故 font_wx_50.bin → fw50.bin。 */
#define WX_TEMP_FONT_PATH "S:/font/fw50.bin"
static lv_font_t *s_font_wx_temp = NULL;  ///< 加载成功的天气温度字体；NULL=未加载/失败
static bool s_font_wx_temp_tried = false; ///< 已尝试过（失败也不重试，避免反复读盘）

static const lv_font_t *wx_temp_font_get(void)
{
    if (!s_font_wx_temp_tried)
    {
        s_font_wx_temp_tried = true;
        s_font_wx_temp = lv_binfont_create(WX_TEMP_FONT_PATH);
        if (s_font_wx_temp == NULL)
            ESP_LOGW(TAG, "天气温度字体加载失败(%s)，回退 montserrat_48", WX_TEMP_FONT_PATH);
        else
            ESP_LOGI(TAG, "天气温度字体已加载: %s", WX_TEMP_FONT_PATH);
    }
    return s_font_wx_temp ? (const lv_font_t *)s_font_wx_temp : &lv_font_montserrat_48;
}

static lv_obj_t *s_time_page = NULL;      ///< 时间页根容器（隐藏/显示以它为准）
static lv_obj_t *s_time_clock_lbl = NULL; ///< 居中大号 "12:31"
static lv_obj_t *s_time_wd_lbl = NULL;    ///< 右上角 "SUN 2"
static lv_obj_t *s_time_temp_lbl = NULL;  ///< 右上角第二行 "30°"

/** 低功耗常亮时钟是否正在显示。两处用途：
 *  ① main_clock_tick_cb 据此放行每分钟刷新（待机时 s_view 仍是 UI_VIEW_MAIN）；
 *  ② time_page_render 据此决定大号时间用【待机时钟样式】还是【功能盘时间页样式】。
 *  定义放在 time_page_render 之前，供其直接读取。 */
static bool s_standby_clock_on = false;

/* 待机时钟的大号时间垂直偏移（LV_ALIGN_CENTER 基准，正=下移）。
 * 闹钟页因为下方还有「重复/开关/提示」三行文字，用的是 -25 上移让位；
 * 待机时钟屏上只有时间一项，默认 0 正居中。占位可自调。 */
#define STANDBY_CLOCK_Y_OFS 0

/* 【2026-08-12 遮罩揭幕动画方案已撤销】原想用一块矩形从上往下收起，把
 * "SPI 分块传输快慢不均"伪装成"设计好的动画"。实测无效：遮罩本身的
 * 移动/resize 一样要走同一条 SPI 分块传输通道（同一个 W*H/6 draw buffer
 * 瓶颈），肉眼看依旧是卡顿的，只是卡顿的内容从"文字"变成"遮罩边缘"，
 * 没有本质改善。问题的真正根因是 draw buffer 过小导致一屏必须分 6 次
 * flush，这是硬件预算限制，不是能靠客户端动画掩盖的表现层问题。
 * 已改回最简单的一次性 unhide，不做任何入场特效。 */

/**
 * @brief 渲染「时间」页：居中大号 HH:MM + 右上角 星期/日号 + 温度
 */
static void time_page_render(void)
{
    if (s_menu_panel == NULL)
        return;

    bool first_create = (s_time_page == NULL);

#if UI_TIME_FADE_DIAG
    /* 【诊断】把暗态阻塞拆成「建对象/加载字体」与「其余」两段，定位耗时到底在哪：
     * first_create=1 那次含 lv_binfont_create 读 84KB 外挂 Flash，只发生一次；
     * 若 first_create=0 的那几次耗时依然很大，说明瓶颈不是字体加载。 */
    int64_t diag_t0 = esp_timer_get_time();
    int64_t diag_t_create = 0;
#endif

    if (s_time_page == NULL)
    {
        /* 1. 根容器：透明无边框，占满面板，整页隐藏的抓手 */
        s_time_page = lv_obj_create(s_menu_panel);
        lv_obj_set_size(s_time_page, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
        lv_obj_center(s_time_page);
        lv_obj_set_style_bg_opa(s_time_page, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_time_page, 0, 0);
        lv_obj_set_style_pad_all(s_time_page, 0, 0);
        lv_obj_clear_flag(s_time_page, LV_OBJ_FLAG_SCROLLABLE);

        /* 2. 大号时间：215px MiSans 数字字体，transform_scale_x 横向压瘦成"瘦金体"
         * 版式（字高不变、只压横向）。原始方案见 docs/功能盘五模块代码摘录.md。 */
        s_time_clock_lbl = lv_label_create(s_time_page);
        /* 横向压瘦用的 pivot：设在左侧(0) + 垂直居中(50%)，缩放后仍锚定左边缘。
         * 只有功能盘样式会真的缩放，待机时钟样式把 scale 置回 100% 后 pivot 无影响，
         * 故在此设一次即可，不随样式切换。 */
        lv_obj_set_style_transform_pivot_x(s_time_clock_lbl, 0, 0);
        lv_obj_set_style_transform_pivot_y(s_time_clock_lbl, LV_PCT(50), 0);
        lv_label_set_text(s_time_clock_lbl, "--:--");
        /* ★字体/颜色/缩放/对齐【不在此处设】，统一交给下方的「样式切换」块按当前
         * 模式（待机时钟 / 功能盘时间页）设定 —— 那段紧接着就会执行，不会有未设
         * 样式的中间帧。这样安排的收益：待机时钟只加载它自己要用的 f118.bin(33KB)，
         * 【不会白白去碰 time_big_font_get() 的 numbig.bin(84KB)】。 */

        /* 3. 右上角第一行：英文星期 + 日号 */
        s_time_wd_lbl = lv_label_create(s_time_page);
        /* 【2026-08-18 换原生 20px 外挂字体】原为 montserrat_48 + transform_scale
         * TIME_CORNER_SCALE(108/256≈42%)，实际字高 48×108/256 = 20.25px。
         * LVGL 9 对带 transform 的对象要先渲染到临时 layer 再重采样，这两行小字
         * 各付一次 layer 分配+重采样，是时间页整屏重绘（实测 DARK 256ms）里
         * 性价比最差的一项。f20.bin 本就在外挂 Flash 里（font_loader.c 早已定义
         * font_cn_20 却无人使用），字号正好 20px、字符表含 U+0020~U+007F 全 ASCII
         * 与 °(U+00B0)（已解析 cmap 验证），换过来视觉差 0.25px、肉眼无差别，
         * 却能彻底去掉 transform 走原生绘制。
         * ⚠️ 换字体与删 transform 必须同时做：只换字体会变成 20×42%＝8.4px。 */
        lv_obj_set_style_text_font(s_time_wd_lbl, font_cn_20_get(), 0);
        lv_obj_set_style_text_color(s_time_wd_lbl, lv_color_hex(TIME_CLOCK_COLOR), 0);
        lv_obj_align(s_time_wd_lbl, LV_ALIGN_TOP_RIGHT, -TIME_CORNER_X, TIME_CORNER_Y);
        lv_label_set_text(s_time_wd_lbl, "--- -");

        /* 4. 右上角第二行：温度（白色，贴在星期行正下方一个行高处） */
        s_time_temp_lbl = lv_label_create(s_time_page);
        /* 同星期行：原生 20px 外挂字体，去掉 transform（说明见上一处） */
        lv_obj_set_style_text_font(s_time_temp_lbl, font_cn_20_get(), 0);
        lv_obj_set_style_text_color(s_time_temp_lbl, lv_color_white(), 0);
        lv_obj_align(s_time_temp_lbl, LV_ALIGN_TOP_RIGHT,
                     -TIME_CORNER_X, TIME_CORNER_Y + TIME_CORNER_LINE_H);
        lv_label_set_text(s_time_temp_lbl, "--°");

        /* 三个文字标签先隐藏：让首次 unhide 那一帧只有纯背景，避免大字
         * 笔画与空白背景混进同一批横带 flush（下方一次性 unhide，不再
         * 拆帧，不再叠加任何入场动画——遮罩揭幕方案已验证无效并撤销，
         * 见本函数上方的撤销说明）。 */
        lv_obj_add_flag(s_time_clock_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_time_wd_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_time_temp_lbl, LV_OBJ_FLAG_HIDDEN);
    }

    /* ── 大号时间的「样式切换」（2026-08-20 新增）──────────────────────────
     * 待机时钟与功能盘时间页【共用同一个 s_time_clock_lbl 对象】，但需要两套样式：
     *   · 待机时钟   ：套用闹钟页那套 —— font_alarm_59（MiSans-Bold，编在固件内）
     *                  + 纯白 + 正中 + 不做横向压瘦
     *   · 功能盘时间页：原样保留 —— numbig.bin 215px（外挂 Flash）+ 蓝色
     *                  + 横向压瘦 45% + 左对齐
     * 故样式不能写死在建对象时，必须每次渲染按当前模式套一次。
     *
     * 【为什么只在模式变化时才写】样式属性一写就会让对象失效重排，而本函数每分钟
     * 都会被 tick 调一次。用 s_time_style_mode 记住上次套的是哪套，同模式直接跳过。
     *
     * 【待机时钟换成 118px 闹钟字体的两个附带收益】
     *   ① 重绘面积变小：215px 大字几乎占满整屏，118px 小一圈 —— BUG-040 那条
     *      「整屏重绘逐带刷新」的横带会短一些，每分钟跨分钟时不那么扎眼；
     *   ② 去掉 transform_scale_x：LVGL9 对带 transform 的对象要先渲染到临时 layer
     *      再重采样，本文件下方注释明确写过这是整屏重绘里性价比最差的一项。
     * 【注意】f118.bin 同样在外挂 Flash，首次用到时 lv_binfont_create 会读盘，
     *   若这一刻发生在 standby_task 上，其栈需有余量（已由 4096→6144，见 standby.c）。 */
    {
        static int s_time_style_mode = -1; /* -1=尚未套用；0=功能盘时间页；1=待机时钟 */
        int want_mode = s_standby_clock_on ? 1 : 0;
        if (want_mode != s_time_style_mode)
        {
            s_time_style_mode = want_mode;
            if (want_mode == 1)
            {
                /* 待机时钟样式：对齐【闹钟编辑页】（功能盘点"闹钟"直接进的就是它，
                 * 见 app_enter_alarm 注释），字体 font_alarm_118_get() = S:/font/f118.bin，
                 * Oswald 窄体、--size 145、字高 120px。
                 * ⚠ 不是 font_alarm_59（那是【闹钟展示页】alarm_page_create 用的 73px，
                 *   明显偏小——最初套错的就是它）。
                 * ⚠ f118.bin 当前字符集只有 0x30-0x39（0~9）【不含冒号 ':'】，因为闹钟
                 *   编辑页把时分拆成四个独立数字标签、示例图无冒号（见 alarm_edit_create
                 *   里被注释掉的 s_edit_colon_lbl）。待机时钟显示的是 "HH:MM"，故须把
                 *   f118.bin 重导为 --range 0x30-0x3A（补一个 ':'）并重烧外挂 Flash，
                 *   否则冒号位置会是空白/方框。 */
                lv_obj_set_style_text_font(s_time_clock_lbl, font_alarm_118_get(), 0);
                lv_obj_set_style_text_color(s_time_clock_lbl, lv_color_white(), 0);
                lv_obj_set_style_transform_scale_x(s_time_clock_lbl, LV_SCALE_NONE, 0); /* 256=不缩放 */
                lv_obj_align(s_time_clock_lbl, LV_ALIGN_CENTER, 0, STANDBY_CLOCK_Y_OFS);
            }
            else
            {
                /* 功能盘时间页样式（2026-08-31 改：与上面【待机时钟】完全同款）
                 * 需求：功能盘的时间字体&大小改成低功耗时钟同款，即
                 *   font_alarm_118_get()（S:/font/f118.bin，Oswald 窄体 120px）
                 *   + 纯白 + 居中 + 不做横向压瘦。
                 * 原样式（numbig.bin 215px 蓝色 + transform_scale_x 压瘦 + 左对齐）
                 * 已废弃；顺带去掉 transform 也省掉 LVGL9 的临时 layer 重采样开销。
                 * ⚠ time_big_font_get / TIME_CLOCK_COLOR / TIME_CLOCK_SCALE_X /
                 *   TIME_CLOCK_X 现已无人使用，保留定义不删以便回退。 */
                lv_obj_set_style_text_font(s_time_clock_lbl, font_alarm_118_get(), 0);
                lv_obj_set_style_text_color(s_time_clock_lbl, lv_color_white(), 0);
                lv_obj_set_style_transform_scale_x(s_time_clock_lbl, LV_SCALE_NONE, 0);
                lv_obj_align(s_time_clock_lbl, LV_ALIGN_CENTER, 0, STANDBY_CLOCK_Y_OFS);
            }
        }
    }

#if UI_TIME_FADE_DIAG
    diag_t_create = esp_timer_get_time(); /* 建对象+字体加载 结束点 */
#endif

    char buf[24];

    if (reminder_is_time_synced())
    {
        uint8_t h, m, s;
        reminder_get_current_time(&h, &m, &s);
        snprintf(buf, sizeof(buf), "%02d:%02d", h, m); /* 只到分钟，不显示秒 */
        lv_label_set_text(s_time_clock_lbl, buf);

        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        snprintf(buf, sizeof(buf), "%s %d",
                 s_cal_wd_en[tm_now.tm_wday & 0x7], tm_now.tm_mday);
        lv_label_set_text(s_time_wd_lbl, buf);
    }

    /* 温度：走 NVS 缓存，断网时仍显示上次联网获取的数值；从未取到过则留 "--°" */
    weather_data_t wx;
    if (reminder_get_weather_data(&wx) == ESP_OK && wx.valid && wx.temp[0] != '\0')
    {
        snprintf(buf, sizeof(buf), "%s°", wx.temp);
        lv_label_set_text(s_time_temp_lbl, buf);
    }

    /* 一次性 unhide，不拆帧、不做任何入场动画（原两步拆分与遮罩揭幕
     * 动画均已验证效果更差，撤销说明见本函数上方）。 */
    (void)first_create;

    lv_obj_clear_flag(s_time_page, LV_OBJ_FLAG_HIDDEN);
    if (s_time_clock_lbl)
        lv_obj_clear_flag(s_time_clock_lbl, LV_OBJ_FLAG_HIDDEN);

    /* 右上角「星期+日号 / 温度」两行小字：功能盘时间页显示，【待机时钟隐藏】
     * （2026-08-20 需求）——低功耗画面只留大号时间一项，屏上越干净越像座钟，
     * 也顺带把每分钟重绘要刷的区域再缩小一点（BUG-040 的横带更短）。
     * 两个标签是共用对象，故这里按模式决定藏或显，不能只在某一处写死。 */
    /* 2026-08-31 改：功能盘时间页也不再需要右上角「星期+日号 / 温度」，
     * 故两种模式一律隐藏（不再按 s_standby_clock_on 分支）。
     * 对象仍保留创建，仅不显示，便于将来恢复；同时整屏重绘面积更小。 */
    if (s_time_wd_lbl)
    {
        {
            lv_obj_add_flag(s_time_wd_lbl, LV_OBJ_FLAG_HIDDEN);
        }
    }
    if (s_time_temp_lbl)
    {

        lv_obj_add_flag(s_time_temp_lbl, LV_OBJ_FLAG_HIDDEN);

#if UI_TIME_FADE_DIAG
        /* 【诊断】本函数自身耗时拆分。注意：这里【不含】真正的绘制——
         * unhide 只是置脏标志，215px 大字的光栅化与 transform 重采样发生在之后的
         * lv_timer_handler 刷新周期里。故若本函数耗时很小、而 dark_cb 总时长很大，
         * 说明瓶颈在绘制而非本函数；两者都小则说明阻塞根本不在这条路径上。 */
        if (FADE_DIAG_ON())
            ESP_LOGW(TAG, "[FD]   time_page_render: 建对象/字体 %lld ms, 取值+unhide %lld ms, 合计 %lld ms (first_create=%d)",
                     (diag_t_create - diag_t0) / 1000,
                     (esp_timer_get_time() - diag_t_create) / 1000,
                     (esp_timer_get_time() - diag_t0) / 1000,
                     (int)first_create);
#endif
    }
}

/**
 * @brief 隐藏时间页（整组子对象随根容器一并隐藏）
 */
static void time_page_hide(void)
{
    if (s_time_page)
        lv_obj_add_flag(s_time_page, LV_OBJ_FLAG_HIDDEN);

    /* 文字标签重新隐藏：s_time_page 对象在多次进入间是复用的（只在首次创建），
     * 若不在这里重置，第二次及以后进入时会带着上次的"已显示"状态直接跟背景
     * 一起冒出来。故每次退出都重新隐藏，保证下次进入首帧仍是纯背景。 */
    if (s_time_clock_lbl)
        lv_obj_add_flag(s_time_clock_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_time_wd_lbl)
        lv_obj_add_flag(s_time_wd_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_time_temp_lbl)
        lv_obj_add_flag(s_time_temp_lbl, LV_OBJ_FLAG_HIDDEN);
}

/**
 * @brief 点阵绘制回调：读 s_cal_dot_st[] 一次性画出 42 个圆点
 *
 * 挂在 s_cal_dot_layer 的 LV_EVENT_DRAW_MAIN 上，取代原来的 42 个 lv_obj。
 *
 * 【坐标】s_cal_dot_x0/y0 是点阵左上角的【屏幕绝对坐标】（沿用原 lv_obj_set_pos
 *   传入的同一套值），而 lv_draw_rect 要的也是绝对坐标，故直接用、无需换算。
 *   注意不能加 layer->buf_area 之类的偏移——那是 draw buffer 内部坐标系，
 *   lv_draw_* 系列接受的 lv_area_t 本身就是屏幕坐标，由绘制单元自行裁剪。
 *
 * 【画圆】radius 给 LV_RADIUS_CIRCLE，lv_draw_rect 会把正方形渲染成正圆，
 *   与原来 lv_obj + lv_obj_set_style_radius(LV_RADIUS_CIRCLE) 的效果完全一致。
 *
 * 【必须极轻】本回调在 LVGL 刷新过程中同步调用，且日历页每次刷新都会跑，
 *   内部只做整数运算和 42 次 lv_draw_rect，不加锁、不读文件、不分配堆内存。
 */
static void cal_dot_layer_draw_cb(lv_event_t *e)
{
    lv_layer_t *layer = lv_event_get_layer(e);
    if (layer == NULL)
        return;

    /* 描述符只建一份，循环里只改颜色——避免 42 次重复初始化 */
    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.radius = LV_RADIUS_CIRCLE; /* 正方形 + 圆角全满 = 正圆 */
    dsc.border_width = 0;
    dsc.bg_opa = LV_OPA_COVER;

    for (int i = 0; i < CAL_DOT_ROWS * CAL_WEEK_DAYS; i++)
    {
        if (s_cal_dot_st[i] == CAL_DOT_NONE)
            continue; /* 空位不画，等价于原来的 LV_OBJ_FLAG_HIDDEN */

        const int r = i / CAL_WEEK_DAYS;
        const int c = i % CAL_WEEK_DAYS;

        lv_area_t a;
        a.x1 = s_cal_dot_x0 + c * CAL_DOT_GAP_X;
        a.y1 = s_cal_dot_y0 + r * CAL_DOT_GAP_Y;
        a.x2 = a.x1 + CAL_DOT_SIZE - 1; /* lv_area_t 是闭区间，故 -1 */
        a.y2 = a.y1 + CAL_DOT_SIZE - 1;

        dsc.bg_color = lv_color_hex(s_cal_dot_st[i] == CAL_DOT_PAST
                                        ? CAL_DOT_COLOR_PAST
                                        : CAL_DOT_COLOR_FUTURE);
        lv_draw_rect(layer, &dsc, &a);
    }
}

/**
 * @brief 渲染「日历」页：顶部年月 + 左侧大号日号/英文星期 + 右侧整月点阵
 *
 * 所有子对象都挂在 s_calendar 这一个容器下，隐藏父对象即可整页隐藏。
 * 点阵按「1 号落在它真实的星期列」排布，故每月首行左侧可能留空。
 */
static void calendar_page_render(void)
{
    if (s_menu_panel == NULL)
        return;

    if (s_calendar == NULL)
    {
        /* 1. 根容器：透明无边框，占满面板，作为所有子对象的定位父对象 */
        s_calendar = lv_obj_create(s_menu_panel);
        lv_obj_set_size(s_calendar, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
        lv_obj_center(s_calendar);
        lv_obj_set_style_bg_opa(s_calendar, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_calendar, 0, 0);
        lv_obj_set_style_pad_all(s_calendar, 0, 0);
        lv_obj_clear_flag(s_calendar, LV_OBJ_FLAG_SCROLLABLE);

        /* 2. 顶部年月 "< 2026 Aug >"：纯装饰，箭头不可点（不做翻月）。
         *    2026-08-14 由 font_cn_16 改为 font_cn_24 */
        s_cal_ym_lbl = lv_label_create(s_calendar);
        /* 【首次进日历卡顿修复 2026-08-17】原用 font_cn_16_get()（外挂 f16.bin，172KB），
         * 实测首次进日历要读盘 450ms（日志「字体已加载: S:/font/f16.bin」），
         * 而本标签内容只有 "<  2026 Aug  >" —— 纯 ASCII。
         * 改用已编译进固件的静态 font_cn_16（同页星期标签 :1101 本就在用它，
         * 另有倒计时/天气/闹钟/菜单标题等十余处引用，故【固件体积零增加】），
         * 字符集覆盖已核对：font_cn_16.c:24478 cmap[0] range_start=32/length=95，
         * 即完整可打印 ASCII，数字、大小写字母、'<' '>' 全部包含。
         * 注：下方大号日号仍用 font_num_140_get()（f140.bin 36KB，约 100ms），
         * 因 140px 无同尺寸静态字体可替，换掉会改变外观，故保留。 */
        lv_obj_set_style_text_font(s_cal_ym_lbl, &font_cn_16, 0);
        lv_obj_set_style_text_color(s_cal_ym_lbl, lv_color_white(), 0);
        lv_obj_align(s_cal_ym_lbl, LV_ALIGN_TOP_LEFT, 14, 12);
        lv_label_set_text(s_cal_ym_lbl, "<  ----  >");

        /* 3. 左侧大号日号 "08"：用 font_num_140（140px 字体，字高 117px），
         *    再等比缩到 CAL_BIGDAY_SCALE 得约 80px。
         *    等比缩放用 transform_scale（不带 _x，宽高同缩），与时间页只压横向不同。
         *    pivot 固定左中，否则缩小后文字会朝中心收，左边缘不再贴齐 14px。 */
        s_cal_bigday_lbl = lv_label_create(s_calendar);
        lv_obj_set_style_text_font(s_cal_bigday_lbl, font_num_140_get(), 0);
        lv_obj_set_style_text_color(s_cal_bigday_lbl, lv_color_white(), 0);
        lv_obj_set_style_transform_scale(s_cal_bigday_lbl, CAL_BIGDAY_SCALE, 0);
        lv_obj_set_style_transform_pivot_x(s_cal_bigday_lbl, 0, 0);
        lv_obj_set_style_transform_pivot_y(s_cal_bigday_lbl, LV_PCT(50), 0);
        lv_obj_align(s_cal_bigday_lbl, LV_ALIGN_LEFT_MID, 14, 10);
        lv_label_set_text(s_cal_bigday_lbl, "--");

        /* 4. 英文星期 "FRI"：贴在大号日号的右下角基线处 */
        s_cal_wd_lbl = lv_label_create(s_calendar);
        lv_obj_set_style_text_font(s_cal_wd_lbl, &font_cn_16, 0);
        lv_obj_set_style_text_color(s_cal_wd_lbl, lv_color_white(), 0);
        lv_obj_align_to(s_cal_wd_lbl, s_cal_bigday_lbl,
                        LV_ALIGN_OUT_RIGHT_BOTTOM, CAL_WD_DX, -6);
        lv_label_set_text(s_cal_wd_lbl, "---");

        /* 5. 42 个圆点（6 行 × 7 列）的几何参数，行优先编号。
         *    右对齐排布：点阵整体贴屏幕右侧，与左侧大号日号分列两边。
         *    ⚠️ 2026-08-19：这里算出的 x0/y0/grid_w/grid_h 现在有两个用途——
         *       ① 定位下面那个绘制层容器 ② 存进 s_cal_dot_x0/y0 供 draw 回调算每格坐标。
         *       改动这几个公式会同时影响容器位置和圆点位置，两者必须保持一致。 */
        const lv_coord_t grid_w = (CAL_WEEK_DAYS - 1) * CAL_DOT_GAP_X + CAL_DOT_SIZE;
        const lv_coord_t x0 = BSP_LCD_WIDTH - grid_w - 14; /* 距右边缘 14px */
        /* 起始 y：针对整屏 240 高垂直居中，上下留白严格相等。
         * 早期写死 44+2*行距，行距一改就顶出屏幕；后来按「顶部年月行以下」居中，
         * 又因扣掉 area_top 使基准整体偏下（实测上 53px / 下 13px 明显不均）。
         * 现直接对整屏居中，CAL_DOT_GAP_Y 怎么调上下都自动对称。 */
        const lv_coord_t grid_h = (CAL_DOT_ROWS - 1) * CAL_DOT_GAP_Y + CAL_DOT_SIZE;
        const lv_coord_t y0 = (BSP_LCD_HEIGHT - grid_h) / 2;
        /* 点阵改为「单个绘制层 + DRAW 事件手绘」（2026-08-19，详见 s_cal_dot_layer
         * 声明处注释）。这里只建一个透明容器承载 draw 事件，42 个圆点在
         * cal_dot_layer_draw_cb 里一次性画出，不再各建一个 lv_obj。
         *
         * 容器必须覆盖整个点阵区域：LVGL 只在对象自身矩形范围内派发 DRAW_MAIN
         * 并按该矩形裁剪，容器画小了点会被裁掉。故尺寸取 grid_w × grid_h。 */
        s_cal_dot_x0 = x0; /* 存给 draw 回调用（回调里拿不到这两个局部变量） */
        s_cal_dot_y0 = y0;

        s_cal_dot_layer = lv_obj_create(s_calendar);
        lv_obj_set_size(s_cal_dot_layer, grid_w, grid_h);
        lv_obj_set_pos(s_cal_dot_layer, x0, y0);
        lv_obj_set_style_bg_opa(s_cal_dot_layer, LV_OPA_TRANSP, 0);
        lv_obj_set_style_border_width(s_cal_dot_layer, 0, 0);
        lv_obj_set_style_pad_all(s_cal_dot_layer, 0, 0);
        lv_obj_clear_flag(s_cal_dot_layer, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_add_event_cb(s_cal_dot_layer, cal_dot_layer_draw_cb,
                            LV_EVENT_DRAW_MAIN, NULL);
    }

    if (reminder_is_time_synced())
    {
        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);

        char buf[24];
        snprintf(buf, sizeof(buf), "<  %d %s  >",
                 tm_now.tm_year + 1900, s_cal_mon_en[tm_now.tm_mon]);
        lv_label_set_text(s_cal_ym_lbl, buf);

        snprintf(buf, sizeof(buf), "%02d", tm_now.tm_mday);
        lv_label_set_text(s_cal_bigday_lbl, buf);
        /* 2026-08-12 需求：日历大号日号旁不再显示英文星期缩写 "FRI" */
        lv_label_set_text(s_cal_wd_lbl, "");

        /* 本月 1 号落在哪一列：由「今天的星期」倒推。
         * tm_wday 周日=0，先换算成周一=0 的列号，再减去 (mday-1) 天的位移并取正模。 */
        int today_col = (tm_now.tm_wday + 6) % 7;
        int first_col = ((today_col - (tm_now.tm_mday - 1)) % 7 + 7) % 7;
        int days = cal_days_in_month(tm_now.tm_year + 1900, tm_now.tm_mon);

        /* 只写状态表，不碰对象——实际绘制由 cal_dot_layer_draw_cb 读表完成。
         * 语义与改版前一一对应：HIDDEN→CAL_DOT_NONE、灰→PAST、白→FUTURE。 */
        for (int i = 0; i < CAL_DOT_ROWS * CAL_WEEK_DAYS; i++)
        {
            int day = i - first_col + 1; /* 该格对应几号（<1 或 >days 即空格） */
            if (day < 1 || day > days)
                s_cal_dot_st[i] = CAL_DOT_NONE; /* 月初/月末的空位不画点 */
            else
                /* 灰=已过去；白=今天及以后 */
                s_cal_dot_st[i] = (day < tm_now.tm_mday) ? CAL_DOT_PAST : CAL_DOT_FUTURE;
        }

        /* 状态表变了必须主动失效，否则 LVGL 认为该区域没变化、不会重画 */
        if (s_cal_dot_layer)
            lv_obj_invalidate(s_cal_dot_layer);
    }

    lv_obj_clear_flag(s_calendar, LV_OBJ_FLAG_HIDDEN);
}

/**
 * @brief 隐藏日历页（整组子对象随根容器一并隐藏）
 *
 * 与 alarm_page_hide() / weather_page_hide() 同风格：离开本页时必须整组隐藏，
 * 否则会残留到功能盘/其他功能页（它们共用同一个 s_menu_panel）。
 *
 * 【历史坑点·已消除】改版前右上角时钟 s_calendar_clock 是独立于 s_calendar 的
 * 兄弟对象，只 hide 父容器会漏掉它，导致 HH:MM 残留在所有其他页面上且停止走字。
 * 现在时间已拆成独立页面，本页所有对象都是 s_calendar 的子对象，隐藏父对象即可。
 */
static void calendar_page_hide(void)
{
    if (s_calendar)
        lv_obj_add_flag(s_calendar, LV_OBJ_FLAG_HIDDEN);
}

/* ═══════════════════════════════════════════════════════════════
 * SPIFFS 文件系统初始化
 * ═══════════════════════════════════════════════════════════════ */
void init_spiffs(void)
{
    ESP_LOGI("SPIFFS", "Initializing SPIFFS");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/spiffs",
        .partition_label = "assets",
        .max_files = 5,
        .format_if_mount_failed = false};
    esp_err_t ret = esp_vfs_spiffs_register(&conf);
    if (ret != ESP_OK)
    {
        if (ret == ESP_FAIL)
            ESP_LOGE("SPIFFS", "Failed to mount or format filesystem");
        else if (ret == ESP_ERR_NOT_FOUND)
            ESP_LOGE("SPIFFS", "Failed to find SPIFFS partition");
        else
            ESP_LOGE("SPIFFS", "Failed to initialize SPIFFS (%s)", esp_err_to_name(ret));
        return;
    }
    size_t total = 0, used = 0;
    ret = esp_spiffs_info(conf.partition_label, &total, &used);
    if (ret != ESP_OK)
        ESP_LOGE("SPIFFS", "Failed to get SPIFFS partition information (%s)", esp_err_to_name(ret));
    else
        ESP_LOGI("SPIFFS", "Partition size: total: %d, used: %d", total, used);
}

/* ═══════════════════════════════════════════════════════════════
 * GIF 动画
 * ═══════════════════════════════════════════════════════════════ */
/* ───────────────────────────────────────────────────────────────
 * 主界面待机 GIF 表(GIF 路径 ↔ 纯舵机动作 一一对应)
 *
 * 设计要点:
 *   - 每张 GIF 直接绑定一组三轴(头/左臂/右臂)舵机动作,【不走情绪矩阵】,
 *     因此【不带】震动马达、不带音频,只有舵机运动。
 *   - 动作用 servo_manager 的 servo_request_t 描述(幅度/方向/速度/循环/往返),
 *     通过 servo_manager_submit_request 非阻塞入队,由独立 worker 串行执行——
 *     既线程安全又不阻塞 LVGL 渲染线程。
 *   - 某一轴不想动:把该轴 .count 置 0(下方 apply 时会跳过)。
 *   - 扩展时【只需在本表追加一行】 { 新路径, {头动作},{左臂},{右臂} },无需改逻辑。
 *
 * servo_request_t 字段速查:
 *   channel    : CH_HEAD / CH_L_ARM / CH_R_ARM(由 apply 自动按轴填,表里写 0 占位即可)
 *   amplitude  : SERVO_AMPLITUDE_10/15/20/30(相对中位 90° 的偏摆角度)
 *   direction  : SERVO_DIR_LEFT(+)/RIGHT(-)/NEUTRAL(头:左右;臂:前后)
 *   speed_ms   : SERVO_SPEED_FAST/MID/SLOW/VERY_SLOW(ms/度,越大越慢)
 *   loop_count : 往返次数(oscillate=true 时生效)
 *   oscillate  : true=两侧往返(如摇头),false=单次到位后回中
 * ─────────────────────────────────────────────────────────────── */

/* 单轴动作:复用 servo_request_t,但 channel 由 apply 按轴自动覆盖,这里只关心动作参数。
 * count==0 表示该轴本张 GIF 不参与运动。 */
typedef struct
{
    servo_amplitude_t amplitude;
    servo_direction_t direction;
    servo_speed_level_t speed_ms;
    uint8_t count;  // 往返/重复次数;0 = 该轴不动
    bool oscillate; // true=往返,false=单次到位回中
} gif_servo_action_t;

typedef struct
{
    const char *gif_path;     // SPIFFS 路径,如 "S:/gif/1_3.gif"
    gif_servo_action_t head;  // 头部舵机动作
    gif_servo_action_t l_arm; // 左臂舵机动作
    gif_servo_action_t r_arm; // 右臂舵机动作
} main_gif_entry_t;

/* 各 GIF 的舵机动作(参数为初版默认值,可边测边调)。
 * 约定:{幅度, 方向, 速度, 次数, 是否往返}
 *
 * ★【2026-08-31 空闲收窄】本表 = 空闲轮播候选池，现只保留【中性系 9 张】：
 *     1_x 中性(3) + 2_x 中性聆听(3) + 3_x 中性表情(3)
 *   情绪系 4_x~16_x(共 32 张)已用 #if 0 包起来移出本池，原因：
 *     空闲(没人交互)时随机蹦出「傲娇/生气/委屈」等强情绪表情不合理，
 *     那 32 张应归触摸情绪走 interaction.c 的 g_emotion_matrix。
 *   ⚠️ 本表与下方 s_idle_actions 必须【等长且同序】(有 _Static_assert 护栏)，
 *      故那边的 idx9~idx40 已用同样的 #if 0 同步切掉。要恢复请【两处一起】改回 #if 1。
 */
static const main_gif_entry_t s_main_gif_table[] = {
    // 1_1.gif:中性(变体1)
    {"S:/gif/1_1.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, true}},

    // 1_2.gif:中性(变体2)
    {"S:/gif/1_2.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true}},

    // 1_3.gif:中性(变体3)
    {"S:/gif/1_3.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, true}},

    // 2_1.gif:中性聆听(变体1)
    {"S:/gif/2_1.gif",
     /*head */ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_SLOW, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_SLOW, 0, false}},

    // 2_2.gif:中性聆听(变体2)
    {"S:/gif/2_2.gif",
     /*head */ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_SLOW, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_SLOW, 0, false}},

    // 2_3.gif:中性聆听(变体3)
    {"S:/gif/2_3.gif",
     /*head */ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_SLOW, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_SLOW, 0, false}},

    // 3_1.gif:中性表情(变体1)
    {"S:/gif/3_1.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, true}},

    // 3_2.gif:中性表情(变体2)
    {"S:/gif/3_2.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, true}},

    // 3_3.gif:中性表情(变体3)
    {"S:/gif/3_3.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, true}},

#if 0  /* 【2026-08-31 空闲收窄】情绪系 GIF(4_x~16_x)移出空闲轮播；保留原文，恢复只需把 #if 0 改回 #if 1 */
    // 4_1.gif:傲娇(变体1)
    {"S:/gif/4_1.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false}},

    // 4_2.gif:傲娇(变体2)
    {"S:/gif/4_2.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false}},

    // 4_3.gif:傲娇(变体3)
    {"S:/gif/4_3.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false}},

    // 4_4.gif:傲娇(变体4)
    {"S:/gif/4_4.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false}},

    // 5_1.gif:兴奋(变体1)
    {"S:/gif/5_1.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true}},

    // 5_2.gif:兴奋(变体2)
    {"S:/gif/5_2.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true}},

    // 5_3.gif:兴奋(变体3)
    {"S:/gif/5_3.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true}},

    // 5_4.gif:兴奋(变体4)
    {"S:/gif/5_4.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true}},

    // 6_1.gif:好奇(变体1)
    {"S:/gif/6_1.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false}},

    // 6_2.gif:好奇(变体2)
    {"S:/gif/6_2.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false}},

    // 6_3.gif:好奇(变体3)
    {"S:/gif/6_3.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false}},

    // 7_1.gif:委屈(变体1)
    {"S:/gif/7_1.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 1, false}},

    // 7_2.gif:委屈(变体2)
    {"S:/gif/7_2.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_VERY_SLOW, 1, false}},

    // 8_1.gif:害羞(变体1)
    {"S:/gif/8_1.gif",
     /*head */ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false}},

    // 8_2.gif:害羞(变体2)
    {"S:/gif/8_2.gif",
     /*head */ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false}},

    // 9_1.gif:怕痒(变体1)
    {"S:/gif/9_1.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 4, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 4, true},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 4, true}},

    // 9_2.gif:怕痒(变体2)
    {"S:/gif/9_2.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 4, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 4, true},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 4, true}},

    // 9_3.gif:怕痒(变体3)
    {"S:/gif/9_3.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 4, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 4, true},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 4, true}},

    // 9_4.gif:怕痒(变体4)
    {"S:/gif/9_4.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 4, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 4, true},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 4, true}},

    // 10_1.gif:惊喜(变体1)
    {"S:/gif/10_1.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_MID, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false}},

    // 10_2.gif:惊喜(变体2)
    {"S:/gif/10_2.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false}},

    // 10_3.gif:惊喜(变体3)
    {"S:/gif/10_3.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_MID, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 1, false}},

    // 11_1.gif:慵懒(变体1)
    {"S:/gif/11_1.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_VERY_SLOW, 1, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_VERY_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 1, true}},

    // 12_1.gif:撒娇(变体1)
    {"S:/gif/12_1.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 2, true}},

    // 13_1.gif:治愈(变体1)
    {"S:/gif/13_1.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, true}},

    // 14_1.gif:犯困(变体1)
    {"S:/gif/14_1.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_VERY_SLOW, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_VERY_SLOW, 0, false}},

    // 14_2.gif:犯困(变体2)
    {"S:/gif/14_2.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_VERY_SLOW, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_VERY_SLOW, 0, false}},

    // 15_1.gif:生气(变体1)
    {"S:/gif/15_1.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, false}},

    // 15_2.gif:生气(变体2)
    {"S:/gif/15_2.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_FAST, 3, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, false}},

    // 16_1.gif:舒服(变体1)
    {"S:/gif/16_1.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false}},

    // 16_2.gif:舒服(变体2)
    {"S:/gif/16_2.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false}},

    // 16_3.gif:舒服(变体3)
    {"S:/gif/16_3.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, false}},
#endif /* 空闲收窄：情绪系 GIF 暂不参与空闲轮播 */

    /* ↓↓↓ 以后加 GIF 只加一行(路径 + 三轴动作)↓↓↓ */
};
#define MAIN_GIF_COUNT (sizeof(s_main_gif_table) / sizeof(s_main_gif_table[0]))

/* ═══════════════════════════════════════════════════════════════
 * 对话状态动作表 —— 状态切换时播放 GIF + 三轴舵机 + 震动（经情绪 worker 执行）
 *
 * 设计：每个对话状态（LISTENING/SPEAKING）对应一组「状态动作」，每条动作
 *       = {GIF 路径 + 三轴舵机绝对角度 + 震动序列}，三者各自独立配置。
 *       组内随机选一条，调 ui_interaction_play_custom 交给情绪 worker 执行。
 *       - is_state_gif=true：高优先级状态切图，不被对话中「丢弃情绪切图」兜底误伤；
 *       - keep_screen=true：动作播完屏幕停在该 GIF（不跳回待机循环），舵机仍归中。
 *       IDLE 不用本表：清 s_neutral_active + ui_resume_main_gif_loop 恢复待机循环。
 *
 * 扩展：往对应数组加一行 {gif, is_state, keep, vib, len, head, l_arm, r_arm} 即可。
 * 占位：GIF 先复用现有 1_1/1_1.gif；素材到位后只改 .gif_path（换中性 GIF）和动作参数。
 *       切图前 main_gif_switch_timer_cb 会 lv_fs_open 校验文件存在，不存在则保持当前画面（BUG-010）。
 * 角度约定：中心 90°；head 左+右-，臂 前+后-。
 * ═══════════════════════════════════════════════════════════════ */
/* 状态动作专用震动序列（与情绪 vib_* 独立，可自由调手感）
 * 【当前未引用】按需求「对话态暂时去除震动」，下面两表的 .vib_seq 均置 NULL。
 * 保留定义以便随时恢复：恢复时把对应条目改回 .vib_seq = s_vib_listen / s_vib_speak
 * 并把 .vib_seq_len 改回 sizeof(...)/sizeof(...[0]) 即可。
 * 加 __attribute__((unused)) 抑制「定义未使用」告警（static const 数组通常不告警，显式标注更稳）。 */
static const VibStep_t s_vib_listen[] __attribute__((unused)) = {{50, 60, 0}};              // 监听：轻震 1 次
static const VibStep_t s_vib_speak[] __attribute__((unused)) = {{70, 50, 50}, {70, 50, 0}}; // 说话：短促 2 次

/* 监听组（用户说话 LISTENING）：表达「我在听你说」。
 *
 * 【每轮随机换图机制】ui_set_neutral_gif_state 每次被调用都重新 esp_random()%count 抽一条
 * （见本文件 ui_set_neutral_gif_state）。session.c 在每轮「唤醒/TTS播完回监听」都会调它，
 * 故效果 = 「每一轮换一张新图，该轮之内锁定这张图无限循环播放」（GIF 自身 loop_count=0）。
 * 表里只有 1 条时 esp_random()%1 恒为 0 → 退化成固定一张（填表前的占位状态）。
 *
 * 【舵机】监听 = 用户正在说话、麦克风正在收音，此时舵机全部 count=0【不动】：
 *   ① 静止本身承担「倾听」语义（人认真听时身体是安定的）；
 *   ② 规避舵机机械噪声污染拾音（见本文件 main_gif_switch_timer_cb 附近那条
 *      「排查麦克风削波尖峰是否与舵机驱动噪声相关」的注释，空闲态都已在怀疑）。
 * 后续若觉得「完全不动」太死，可给 head 填极小幅度（±5°、SLOW、count=1）做微反应。
 *
 * 【扩展】加一行 {gif, is_state, keep, idle, vib, len, head, l_arm, r_arm} 即可，
 * 逻辑自动随机，无需改调用处。素材到位后只改 .gif_path。 */
static const ia_custom_action_t s_listening_actions[] = {
    {
        // 占位：安静注视
        .gif_path = "S:/gif/2_1.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false, // 对话动作不可被触摸打断
        .vib_seq = NULL,  // 暂时去除震动
        .vib_seq_len = 0,
        .head = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},    // 不动（count=0）：倾听
        .left_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0}, // 不动
        .right_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0},
    },
    {
        // 占位：专注
        .gif_path = "S:/gif/1_2.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false,
        .vib_seq = NULL,
        .vib_seq_len = 0,
        .head = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},
        .left_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0},
        .right_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0},
    },
    {
        // 占位：好奇倾听
        .gif_path = "S:/gif/3_3.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false,
        .vib_seq = NULL,
        .vib_seq_len = 0,
        .head = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},
        .left_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0},
        .right_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0},
    },
};

/* 说话组（大模型说话 PLAYING）：表达「我在跟你说」。
 *
 * 【每轮随机换图】同监听组：每轮 TTS_START 重新抽一条，该轮之内锁定该图无限循环。
 *
 * 【为何各条舵机参数刻意写得相近】说话的语义是【一致的一件事】，动作质感应统一：
 *   若这轮温柔点头、下轮大幅甩头，读起来像「情绪不稳定」而非「在说话」。
 *   故：GIF 每条不同（表情有变化不呆板），舵机幅度/速度保持相近（只微调）。
 *   → 「表情可以变，说话的样子不该变」。
 *
 * 【已知局限·待后续改】当前舵机是「摆 count 次就结束并归中」，与 TTS 实际时长【无关】：
 *   大模型回复十几秒时，舵机开头动几下便静止，而屏幕 GIF 仍在无限循环 → 视听割裂。
 *   根治方案 = 舵机改「循环律动直到 TTS 结束」(loop_until_stop 标志 + 停止接口，
 *   在 session.c 的 TTS 播完 / 唤醒打断两处调停止)，需改 interaction worker，本次未做。
 *   本次先只填表使「每轮随机换图」生效；舵机参数按用户要求后续调。 */
static const ia_custom_action_t s_speaking_actions[] = {
    {
        // 占位：说话（活泼）
        .gif_path = "S:/gif/2_2.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false, // 对话动作不可被触摸打断
        .vib_seq = NULL,  // 暂时去除震动
        .vib_seq_len = 0,
        .head = {100.0f, 80.0f, SERVO_SPEED_MID, 2},     // 头小幅律动 2 次（±10°）
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1}, // 臂小幅摆 1 次
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
    },
    {
        // 占位：说话（讲解）
        .gif_path = "S:/gif/1_1.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false,
        .vib_seq = NULL,
        .vib_seq_len = 0,
        .head = {102.0f, 78.0f, SERVO_SPEED_MID, 2}, // 与上条相近（仅微调 2°）
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
    },
    {
        // 占位：说话（兴奋）
        .gif_path = "S:/gif/3_1.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false,
        .vib_seq = NULL,
        .vib_seq_len = 0,
        .head = {98.0f, 82.0f, SERVO_SPEED_MID, 3}, // 幅度相近，次数略多
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
    },
    {
        // 占位：说话（温和）—— 原 3_14(舒服) 素材重编号后为 16_1
        .gif_path = "S:/gif/16_1.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false,
        .vib_seq = NULL,
        .vib_seq_len = 0,
        .head = {100.0f, 80.0f, SERVO_SPEED_SLOW, 2}, // 幅度同，速度稍慢
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
    },
};

/* 空闲动作专用震动序列（比对话更轻，"发呆"感；占位，后续调手感） */
static const VibStep_t s_vib_idle[] = {{35, 60, 0}}; // 空闲：极轻震 1 次

/* 空闲组（开机后随机 GIF 循环、未对话、未进待机）：一图一动作，「GIF 切换 = 动作切换」。
 *
 * 关键设计（根治"舵机不动"的 flush 风暴）：空闲动作【自己切图】(gif_path 填真实路径) +
 * 舵机 + 震动，由 worker 串行执行；keep_screen=false → 播完调 ui_resume_main_gif_loop
 * 触发下一条。节奏由【舵机摆动耗时】天然决定（ia_worker 等舵机摆完才算动作结束、才切
 * 下一张），故"舵机动多久 = GIF 多久切一次"，调舵机/震动时间即调节奏，无需死时间。
 *
 *   - gif_path=NULL：动作【不切图】。GIF 切换完全交还 READY 机制（main_gif_apply_index），
 *     GIF 完整播完一轮才切下一张，永不被动作打断；
 *   - keep_screen=true：动作播完【不】调 ui_resume_main_gif_loop，不触发下一张
 *     （切断"动作→切图→动作"的失控循环；下一张只由 GIF 的 READY 驱动）；
 *   - is_idle=true：低优先级，触摸情绪可 flush 打断。
 * 即：「GIF 播完一轮 → 切下一张 + 投递本表对应动作（只舵机+震动）」。舵机在这一轮 GIF
 * 期间摆完就安静，等下一轮 READY。GIF 一定完整，舵机也完整，不依赖调参。
 * 动作按 GIF 索引取（与 s_main_gif_table 对齐）；后续换中性 GIF 只改 s_main_gif_table + 本表动作参数。 */
static const ia_custom_action_t s_idle_actions[] = {
    {
        // idx 0 → 1_1.gif（中性·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 1 → 1_2.gif（中性·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {75.0f, 105.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 2 → 1_3.gif（中性·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 3 → 2_1.gif（中性聆听·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 0},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 0},
    },
    {
        // idx 4 → 2_2.gif（中性聆听·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 0},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 0},
    },
    {
        // idx 5 → 2_3.gif（中性聆听·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 0},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 0},
    },
    {
        // idx 6 → 3_1.gif（中性表情·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_MID, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 7 → 3_2.gif（中性表情·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {75.0f, 105.0f, SERVO_SPEED_MID, 2},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_MID, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 8 → 3_3.gif（中性表情·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_MID, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_MID, 1},
    },
#if 0  /* 【2026-08-31 空闲收窄】情绪系 GIF(4_x~16_x)移出空闲轮播；保留原文，恢复只需把 #if 0 改回 #if 1 */
    {
        // idx 9 → 4_1.gif（傲娇·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_FAST, 2},
        .left_arm = {105.0f, 75.0f, SERVO_SPEED_MID, 1},
        .right_arm = {75.0f, 105.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 10 → 4_2.gif（傲娇·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {70.0f, 110.0f, SERVO_SPEED_FAST, 2},
        .left_arm = {75.0f, 105.0f, SERVO_SPEED_MID, 1},
        .right_arm = {105.0f, 75.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 11 → 4_3.gif（傲娇·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_FAST, 2},
        .left_arm = {105.0f, 75.0f, SERVO_SPEED_MID, 1},
        .right_arm = {75.0f, 105.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 12 → 4_4.gif（傲娇·变体4）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {70.0f, 110.0f, SERVO_SPEED_FAST, 2},
        .left_arm = {75.0f, 105.0f, SERVO_SPEED_MID, 1},
        .right_arm = {105.0f, 75.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 13 → 5_1.gif（兴奋·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 3},
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_FAST, 3},
        .right_arm = {70.0f, 110.0f, SERVO_SPEED_FAST, 3},
    },
    {
        // idx 14 → 5_2.gif（兴奋·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {60.0f, 120.0f, SERVO_SPEED_FAST, 3},
        .left_arm = {70.0f, 110.0f, SERVO_SPEED_FAST, 3},
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_FAST, 3},
    },
    {
        // idx 15 → 5_3.gif（兴奋·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 3},
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_FAST, 3},
        .right_arm = {70.0f, 110.0f, SERVO_SPEED_FAST, 3},
    },
    {
        // idx 16 → 5_4.gif（兴奋·变体4）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {60.0f, 120.0f, SERVO_SPEED_FAST, 3},
        .left_arm = {70.0f, 110.0f, SERVO_SPEED_FAST, 3},
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_FAST, 3},
    },
    {
        // idx 17 → 6_1.gif（好奇·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 0},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_MID, 0},
    },
    {
        // idx 18 → 6_2.gif（好奇·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {70.0f, 110.0f, SERVO_SPEED_MID, 1},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_MID, 0},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 0},
    },
    {
        // idx 19 → 6_3.gif（好奇·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_MID, 0},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_MID, 0},
    },
    {
        // idx 20 → 7_1.gif（委屈·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_VERY_SLOW, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 1},
    },
    {
        // idx 21 → 7_2.gif（委屈·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {75.0f, 105.0f, SERVO_SPEED_VERY_SLOW, 1},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 1},
    },
    {
        // idx 22 → 8_1.gif（害羞·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 23 → 8_2.gif（害羞·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 24 → 9_1.gif（怕痒·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_FAST, 4},
        .left_arm = {105.0f, 75.0f, SERVO_SPEED_FAST, 4},
        .right_arm = {75.0f, 105.0f, SERVO_SPEED_FAST, 4},
    },
    {
        // idx 25 → 9_2.gif（怕痒·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {70.0f, 110.0f, SERVO_SPEED_FAST, 4},
        .left_arm = {75.0f, 105.0f, SERVO_SPEED_FAST, 4},
        .right_arm = {105.0f, 75.0f, SERVO_SPEED_FAST, 4},
    },
    {
        // idx 26 → 9_3.gif（怕痒·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_FAST, 4},
        .left_arm = {105.0f, 75.0f, SERVO_SPEED_FAST, 4},
        .right_arm = {75.0f, 105.0f, SERVO_SPEED_FAST, 4},
    },
    {
        // idx 27 → 9_4.gif（怕痒·变体4）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {70.0f, 110.0f, SERVO_SPEED_FAST, 4},
        .left_arm = {75.0f, 105.0f, SERVO_SPEED_FAST, 4},
        .right_arm = {105.0f, 75.0f, SERVO_SPEED_FAST, 4},
    },
    {
        // idx 28 → 10_1.gif（惊喜·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_MID, 2},
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
        .right_arm = {70.0f, 110.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 29 → 10_2.gif（惊喜·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {60.0f, 120.0f, SERVO_SPEED_MID, 2},
        .left_arm = {70.0f, 110.0f, SERVO_SPEED_MID, 1},
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 30 → 10_3.gif（惊喜·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_MID, 2},
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
        .right_arm = {70.0f, 110.0f, SERVO_SPEED_MID, 1},
    },
    {
        // idx 31 → 11_1.gif（慵懒·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_VERY_SLOW, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 1},
    },
    {
        // idx 32 → 12_1.gif（撒娇·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},
        .right_arm = {75.0f, 105.0f, SERVO_SPEED_SLOW, 2},
    },
    {
        // idx 33 → 13_1.gif（治愈·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 34 → 14_1.gif（犯困·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_VERY_SLOW, 1},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 0},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 0},
    },
    {
        // idx 35 → 14_2.gif（犯困·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {60.0f, 120.0f, SERVO_SPEED_VERY_SLOW, 1},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_VERY_SLOW, 0},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_VERY_SLOW, 0},
    },
    {
        // idx 36 → 15_1.gif（生气·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {110.0f, 70.0f, SERVO_SPEED_FAST, 3},
        .left_arm = {105.0f, 75.0f, SERVO_SPEED_FAST, 3},
        .right_arm = {75.0f, 105.0f, SERVO_SPEED_FAST, 3},
    },
    {
        // idx 37 → 15_2.gif（生气·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {70.0f, 110.0f, SERVO_SPEED_FAST, 3},
        .left_arm = {75.0f, 105.0f, SERVO_SPEED_FAST, 3},
        .right_arm = {105.0f, 75.0f, SERVO_SPEED_FAST, 3},
    },
    {
        // idx 38 → 16_1.gif（舒服·变体1）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 39 → 16_2.gif（舒服·变体2）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {75.0f, 105.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // idx 40 → 16_3.gif（舒服·变体3）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {105.0f, 75.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {100.0f, 80.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {80.0f, 100.0f, SERVO_SPEED_SLOW, 1},
    },
#endif /* 空闲收窄：情绪系 GIF 暂不参与空闲轮播 */
};
#define IDLE_ACTION_COUNT (sizeof(s_idle_actions) / sizeof(s_idle_actions[0]))

/* 编译期护栏：空闲动作表必须与主 GIF 表等长，否则后面的 GIF 又会取不到动作而静止。
 * 以后往 s_main_gif_table 加 GIF 忘了同步加动作，会在编译期直接报错，不再靠烧录才发现。
 * （放在 MAIN_GIF_COUNT 之后；两个宏均已定义。） */
_Static_assert(IDLE_ACTION_COUNT == MAIN_GIF_COUNT,
               "s_idle_actions 与 s_main_gif_table 长度必须一致：加 GIF 时请同步加空闲动作");

/**
 * @brief 把一张 GIF 的三轴舵机动作以并行方式非阻塞入队(只动舵机,无震动/音频)
 *
 * 三轴打包成一个 servo_parallel_request_t 提交，由 worker 调
 * bsp_servo_move_all_parallel 实现真正同步，避免逐轴串行入队导致的
 * 「头先动完、臂才开始」割裂感。
 */
static void main_gif_submit_servo(const main_gif_entry_t *entry)
{
    /* 三轴打包为并行请求，worker 调 bsp_servo_move_all_parallel 实现真正同步，
     * 避免逐轴串行入队导致的「头动完才动臂」割裂感。 */
    servo_parallel_request_t preq = {
        .head = {
            .amplitude = entry->head.amplitude,
            .direction = entry->head.direction,
            .speed_ms = entry->head.speed_ms,
            .count = entry->head.count,
            .oscillate = entry->head.oscillate,
        },
        .l_arm = {
            .amplitude = entry->l_arm.amplitude,
            .direction = entry->l_arm.direction,
            .speed_ms = entry->l_arm.speed_ms,
            .count = entry->l_arm.count,
            .oscillate = entry->l_arm.oscillate,
        },
        .r_arm = {
            .amplitude = entry->r_arm.amplitude,
            .direction = entry->r_arm.direction,
            .speed_ms = entry->r_arm.speed_ms,
            .count = entry->r_arm.count,
            .oscillate = entry->r_arm.oscillate,
        },
    };
    servo_manager_submit_parallel(&preq);
}

/**
 * @brief 选择下一张 GIF 的表索引
 *
 * 当前策略:随机(do/while 避免连续两次同一张,提升观感)。
 * 若想改顺序播放,把 UI_MAIN_GIF_RANDOM 置 0 即可退化为 (cur+1)%N。
 *
 * @param cur 当前索引(-1 表示尚未开始,允许任意)
 * @return    下一张的表索引 [0, MAIN_GIF_COUNT)
 */
static int main_gif_pick_next_index(int cur)
{
    if (MAIN_GIF_COUNT <= 1)
        return 0;
#if UI_MAIN_GIF_RANDOM
    int next;
    do
    {
        next = (int)(esp_random() % MAIN_GIF_COUNT);
    } while (next == cur); // 不与上一张重复
    return next;
#else
    return (cur + 1) % (int)MAIN_GIF_COUNT; // 顺序循环（cur=-1 时从 0 开始）
#endif
}

/**
 * @brief 校验 GIF 画布尺寸是否在安全范围内,超过屏幕面积过多则拒绝切图
 *
 * 【背书·血泪坑】lv_gif.c::gif_initialize 用 GIF 自身画布尺寸(而非屏幕尺寸)
 *   分配 draw_buf(ARGB8888,宽*高*4字节)。曾把 1_1.gif 误换成 512x512
 *   (屏幕仅 320x240),画布面积达屏幕 3.4 倍、单帧缓冲逼近 1MB,PSRAM 碎片化
 *   下分配失败——而 gif_initialize 分配失败时只置 draw_buf=NULL 并 return,
 *   并不会停 gifobj->timer,下一帧 gif_next_frame_task_cb 照常触发,解引用
 *   NULL 的 draw_buf->header.w 直接 LoadProhibited 崩溃。故切图前先用
 *   lv_gif_get_size 探测画布尺寸,超过屏幕面积 1.5 倍就拒绝,保留当前画面。
 *
 * @param gif_path 待切换的 GIF 路径
 * @return true=尺寸安全可以切图；false=尺寸异常,已打日志,调用方应保持当前画面
 */
/**
 * @brief 直接读 GIF 文件头取画布宽高（替代 lv_gif_get_size，零额外内存）
 *
 * 【为何不用 lv_gif_get_size】该库函数为了拿两个数，会在【栈上】声明整个
 *   GIFIMAGE 解码器结构体（约 24KB：ucFileBuf 4KB + usGIFTable 8KB +
 *   ucGIFPixels 8KB + 调色板/行缓冲）。而 LVGL 任务栈仅 8192B（见本文件
 *   lvgl_port_cfg.task_stack），24KB 局部变量直接撑穿栈块 16KB，写坏栈外
 *   邻接的堆内存 → 随机堆损坏 + 各处 LoadProhibited 崩溃（受害者随堆布局
 *   变化，曾表现为 WiFi esf_buf / MultiNet / lv_fs_close 崩）。
 *
 * 【本函数做法】GIF 头格式固定：字节 0~5 为魔数 "GIF87"/"GIF89"，
 *   字节 6~7 为画布宽（小端），字节 8~9 为画布高（小端）——与库内
 *   gif.c 的 INTELSHORT(&p[6]) / INTELSHORT(&p[8]) 完全一致。只读 10 字节，
 *   栈上仅一个 10 字节小数组，彻底避开 24KB 大结构体。
 *
 * @param path GIF 文件路径
 * @param w    输出画布宽
 * @param h    输出画布高
 * @return true=读取成功且为合法 GIF；false=打开失败/非 GIF/数据不足
 */
static bool gif_read_canvas_size(const char *path, uint16_t *w, uint16_t *h)
{
    lv_fs_file_t f;
    if (lv_fs_open(&f, path, LV_FS_MODE_RD) != LV_FS_RES_OK)
        return false;

    uint8_t hdr[10];
    uint32_t rd = 0;
    lv_fs_res_t res = lv_fs_read(&f, hdr, sizeof(hdr), &rd);
    lv_fs_close(&f);

    if (res != LV_FS_RES_OK || rd < sizeof(hdr))
        return false;
    if (memcmp(hdr, "GIF89", 5) != 0 && memcmp(hdr, "GIF87", 5) != 0)
        return false;

    *w = (uint16_t)(hdr[6] | (hdr[7] << 8)); // 小端，同库 INTELSHORT
    *h = (uint16_t)(hdr[8] | (hdr[9] << 8));
    return true;
}

static bool main_gif_check_size_safe(const char *gif_path)
{
    uint16_t w = 0, h = 0;
    if (!gif_read_canvas_size(gif_path, &w, &h) || w == 0 || h == 0)
    {
        ESP_LOGW(TAG, "GIF 尺寸探测失败,拒绝切图: %s", gif_path);
        return false;
    }
    /* 屏幕面积 1.5 倍上限:留一定余量给非全屏小图,同时挡住整倍数放大的误用素材 */
    uint32_t gif_area = (uint32_t)w * (uint32_t)h;
    uint32_t screen_area_limit = (uint32_t)BSP_LCD_WIDTH * (uint32_t)BSP_LCD_HEIGHT * 3 / 2;
    if (gif_area > screen_area_limit)
    {
        ESP_LOGW(TAG, "GIF 尺寸过大拒绝切图(可能导致 draw_buf 分配失败崩溃): %s %ux%u > 屏幕(%dx%d)*1.5",
                 gif_path, w, h, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
        return false;
    }
    return true;
}

/**
 * @brief 应用一张 GIF:切图(自动重新播放) + 可选驱动对应舵机动作(纯舵机,无震动/音频)
 *
 * 切图机制:lv_gif_set_src 内部 gif_initialize 会 resume timer 并立即播放,
 *   天然解决"播完一轮停在最后一帧"问题(根因:loop_count<0 时播完 lv_timer_pause)。
 *
 * @param idx        目标 GIF 表索引
 * @param with_servo 是否同时驱动该 GIF 对应的舵机动作
 *                   (首张开机时 servo_manager 队列尚未就绪,传 false 只显示不入队)
 *
 * @note 必须在【已持有 lvgl_port 锁】或【LVGL 线程回调】上下文调用(本函数只做 lv_* 调用,
 *       舵机动作仅是非阻塞入队,不会阻塞 LVGL 线程)。
 */
static void main_gif_apply_index(int idx, bool with_servo)
{
    if (gif_obj == NULL || idx < 0 || (size_t)idx >= MAIN_GIF_COUNT)
        return;

    const main_gif_entry_t *entry = &s_main_gif_table[idx];

    if (!main_gif_check_size_safe(entry->gif_path))
        return; // 尺寸异常,保持当前画面,不切图、不驱动舵机

    /* 【堆探针·2026-07-29 已定位真凶，暂停用】
     * 本探针曾是整起事故最早报警的位置，但每次都报「之前」就已损坏 → GIF 链路只是
     * 撞上别处留下的损坏，并非凶手。
     * 【真凶】FreeRTOS 软件定时器服务任务 Tmr Svc 栈溢出（栈仅 2048B，
     *   见 sdkconfig CONFIG_FREERTOS_TIMER_TASK_STACK_DEPTH）。其栈从内部 SRAM 堆分配，
     *   溢出浅时踩坏隔壁堆块 poison 标记 → 报 CORRUPT HEAP；溢出深时直接 abort 重启。
     *   两次崩溃均紧跟 session.c on_wait_user_timeout 的 ESP_LOGW（[流程#7-A]）之后，
     *   栈指针 0x3fcb77d0 / 0x3fcb7a40 落在同一区间。
     * 需再排查时取消注释即可。 */
    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@lv_gif_set_src之前 idx=%d", idx);
    lv_gif_set_src(gif_obj, entry->gif_path); // 切新图,内部自动重新播放
    // if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@lv_gif_set_src之后 idx=%d path=%s", idx, entry->gif_path);
    // WARN_TASK_STACK_LOW("HEAPCHK", NULL, 1024);

    s_gif_cur_index = idx;

    if (with_servo)
    {
        //! 暂时注释掉舵机动作
        // main_gif_submit_servo(entry); // 仅驱动三轴舵机,非阻塞入队
        // /////ESP_LOGI(TAG, "主界面 GIF 切换 → [%d] %s (+舵机动作)", idx, entry->gif_path);
    }
    /* with_servo=false 时不打印（低功耗待机每次 GIF 切换都走此分支，会刷屏日志） */
}

/**
 * @brief GIF 播完一整轮(最后一帧)时 LVGL 触发 LV_EVENT_READY,本回调被同步调用
 *
 * 【安全约束·关键】本回调运行在 gif_next_frame_task_cb 内部(lv_gif.c),
 *   该函数返回后还会继续使用 gif 的 draw_buf/timer。若此处直接 lv_gif_set_src,
 *   会销毁并重建 draw_buf 与 timer,造成迭代器失效/重入,导致卡死或崩溃。
 *   因此本回调【只】记录"下一张索引",真正切换交给独立 timer 在下一个 tick 执行。
 *
 * 本回调已在 LVGL 线程,严禁再调 lvgl_port_lock(自死锁)。
 */
static void main_gif_ready_cb(lv_event_t *e)
{
    (void)e;
    // 【堆探针·已停用】GIF 解码线程视角查堆；根因已修，全堆扫描太重触发看门狗，暂注释保留。
    // if (!heap_caps_check_integrity_all(true))
    //     ESP_LOGE("HEAPCHK", "★堆损坏@GIF ready_cb(解码线程视角)");
    // 【诊断-DBG4】GIF 每播完一轮都会进这里；重点看二级待机期间是否仍在持续触发
    // （若二级期间此日志仍频繁刷屏，说明 GIF 动画没被冻结，一直在后台播放/切换，验证OK后可删）
    // ESP_LOGW("GIFDBG", "[DBG4] ready_cb 触发: s_view=%d standby_deep=%d interaction_playing=%d neutral=%d",
    //          (int)s_view, (int)standby_is_deep_active(), (int)interaction_is_playing(), (int)s_neutral_active);
    // 仅在主界面才循环切换 GIF + 驱动舵机;进入功能菜单/闹钟编辑后 GIF 被隐藏,
    // 此时不应继续切图或驱动舵机(否则会出现"已在功能层却仍在动"的异常)。
    // 【2026-09-01】判据由裸 s_view 改为 main_idle_loop_active()：进功能盘的渐暗期
    // s_view 仍是 MAIN，旧判据会放行，正是"功能盘出来了舵机还在动"的三个漏口之一。
    if (!main_idle_loop_active())
        return;
    /* ★开机 logo 期间：logo 是叠在上层的【静态图】，下面的 gif_obj 虽被盖住但仍在解码播放，
     *   播完一轮照样进到这里。此时不能排队切图——否则等 logo 揭开时，主轮播已经自己
     *   悄悄换过好几张，"揭开即首张"的预期被破坏。直接早退，让它原地循环播同一张即可。
     *   （logo 本身不再重播：静态图不触发 LV_EVENT_READY，旧的循环重播链路自然失效。） */
    if (s_boot_logo_img != NULL)
        return;
    // 对话中（LISTENING/PLAYING）：屏幕锁定状态中性 GIF，不自动随机循环（防待机循环抢图）。
    // 状态回 IDLE 时 ui_set_neutral_gif_state(NEUTRAL_IDLE) 会清此标志并恢复循环。
    if (s_neutral_active)
        return;
    if (!s_idle_carousel_enabled)
        return; // 调试禁用空闲态自动轮播：不排下一张空闲图（情绪/状态切图走 pending_path 不受影响）
    if (s_gif_pending_idx >= 0)
        return; // 已有待切换,避免本轮重复排队
    s_gif_pending_idx = main_gif_pick_next_index(s_gif_cur_index);
    // ESP_LOGI("GIFDBG", "ready_cb: GIF播完一轮 → 排队下一张 idx=%d", s_gif_pending_idx);
    if (s_gif_switch_tmr != NULL)
        lv_timer_resume(s_gif_switch_tmr); // 唤醒延迟切换 timer,下个 tick 执行
}

/**
 * @brief 通知开机初始化真正完成，见 ui_port.h 注释
 *
 * 【行为】就绪信号一到就【立即】结束 logo 展示，启动"渐暗→揭图→渐亮"过渡。
 *   logo 是静态图，没有"播完一轮"的概念，故不存在等待整轮的问题，随时可以收场。
 *
 * 【揭图而非切图】logo 是叠在 gif_obj 之上的独立 lv_image，下面的主轮播首张在
 *   main_gif_create() 里就已载好。所以这里不需要排 s_gif_pending_idx，也不需要
 *   唤醒 s_gif_switch_tmr——渐暗到底那一刻 lv_obj_del 掉 logo 图即揭开下面的 GIF，
 *   零 set_src、零 draw_buf 重建，自然也就绕开了 BUG-010 那条"切图必须延迟一拍"的约束。
 *
 * 【跨线程】本函数由 main 任务调用（非 LVGL 线程）。这里只写 s_boot_fade_phase /
 *   起始时间戳并 resume timer（仅置 paused 标志），真正的删对象动作发生在
 *   ui_boot_fade_timer_cb（LVGL 线程）里，符合"LVGL 对象只在 LVGL 线程操作"的规则。
 *
 * 【兜底】若 logo 未创建（文件缺失已回退到正常轮播 / 尚未建对象），直接早退，
 *   此时画面本来就是主轮播，无需任何过渡。
 */
/* 见 ui_port.h 的完整说明：供 bsp_board_lcd_on() 等"直接拍亮度"的调用方在
 * 渐变期间避让，避免中途改写背光造成渐亮途中的亮度回弹。 */
bool ui_is_boot_fading(void)
{
    return s_boot_fade_phase != UI_BOOT_FADE_IDLE;
}

void ui_notify_boot_ready(void)
{
    s_boot_ready = true;

    if (s_boot_logo_img == NULL)
        return; // logo 未显示（未创建 / 文件缺失已回退）→ 画面已是主轮播，无需过渡
    if (s_view != UI_VIEW_MAIN)
        return; // 已进功能层，GIF 已隐藏，不在此处抢图（回主界面时 kick_resume 会恢复）

    ESP_LOGI("GIFDBG", "boot_ready: 触发渐暗→揭开 logo 静态图→渐亮");

    /* 渐变期间不响应新请求：若渐变尚未收尾(理论上不会，s_boot_ready 只置一次)直接忽略重入。 */
    if (s_boot_fade_phase != UI_BOOT_FADE_IDLE)
        return;
    s_boot_fade_phase = UI_BOOT_FADE_OUT;
    s_boot_fade_start_us = esp_timer_get_time();
    s_boot_swap_done_us = 0; // 清零：本轮尚未揭图，暗态窗口从揭图那一刻才起算
    if (s_boot_fade_tmr != NULL)
    {
        /* 周期性触发，每 UI_BOOT_FADE_STEP_MS 推进一步；到达 IDLE 相位时回调内 stop。
         * 已在运行时先 stop 再 start，避免 ESP_ERR_INVALID_STATE。 */
        esp_timer_stop(s_boot_fade_tmr); // 未运行时返回 INVALID_STATE，忽略即可
        esp_timer_start_periodic(s_boot_fade_tmr, (uint64_t)UI_BOOT_FADE_STEP_MS * 1000);
    }
    else
    {
        /* 兜底：渐变 timer 不可用 → 不做过渡，直接揭图并保持常亮，避免卡在黑屏。 */
        s_boot_fade_phase = UI_BOOT_FADE_IDLE;
        s_boot_logo_del_pending = true;
        if (s_gif_switch_tmr != NULL)
            lv_timer_resume(s_gif_switch_tmr);
        bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT);
    }
}

/**
 * @brief 开机 logo 背光渐变回调：只负责渐亮/渐暗背光 + 渐暗到底后触发揭图。
 *
 * ★运行在 esp_timer 线程（非 LVGL 线程），不是 lv_timer 回调。
 *   改用 esp_timer 的原因见 s_boot_fade_tmr 声明处：lv_timer 被开机 GIF 解码
 *   拖到实际间隔 142ms，渐变只跑 23 步，肉眼可见台阶。esp_timer 是硬件定时器，
 *   不受 LVGL 线程忙闲影响，能稳定跑满 UI_BOOT_FADE_STEP_MS。
 *
 * 【线程安全】本回调只做两件事：写 LEDC 寄存器（bsp_board_lcd_fade_step_fine）、
 *   读写自己的相位/时间戳静态变量。全程不碰任何 lv_obj / lv_timer，故无需持锁。
 *   需要动 LVGL 对象的地方（删 logo 图、唤醒切图 timer）一律置 pending 标志，
 *   交给 LVGL 线程的 main_gif_switch_timer_cb 执行。
 *
 * 非阻塞：每拍只调一次 fade_step_fine，不 vTaskDelay（esp_timer 回调禁止阻塞）。
 */
static void ui_boot_fade_timer_cb(void *arg)
{
    (void)arg;
    if (s_boot_fade_phase == UI_BOOT_FADE_IDLE)
    {
        esp_timer_stop(s_boot_fade_tmr);
        return;
    }

    uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - s_boot_fade_start_us) / 1000);

    /* 【诊断·渐变实际步数】排查"一节一节跳变"用，结论出来后可删。
     * 台阶可见 ⇔ 实际步数太少。本段统计每一段渐变真正被调用了多少次、
     * 相邻两次调用的最大间隔（被 GIF 解码抢占会拉大），据此判断瓶颈在哪：
     *   · 步数 ≈ 时长/STEP_MS 且最大间隔 ≈ STEP_MS → lv_timer 跑满，
     *     步数仍不够则说明要靠提高频率(改 esp_timer)解决；
     *   · 步数远少于理论值 / 最大间隔远大于 STEP_MS → LVGL 线程被拖住，
     *     必须挪到 esp_timer 才有意义。 */
    static uint32_t s_dbg_steps = 0;      // 本段已推进步数
    static uint32_t s_dbg_last_us = 0;    // 上次进入本函数的时刻
    static uint32_t s_dbg_max_gap_ms = 0; // 本段内最大调用间隔
    static ui_boot_fade_phase_t s_dbg_ph = UI_BOOT_FADE_IDLE;
    {
        uint32_t now_us = (uint32_t)esp_timer_get_time();
        if (s_dbg_ph != s_boot_fade_phase) // 换段：重置统计
        {
            s_dbg_ph = s_boot_fade_phase;
            s_dbg_steps = 0;
            s_dbg_max_gap_ms = 0;
            s_dbg_last_us = now_us;
        }
        uint32_t gap_ms = (now_us - s_dbg_last_us) / 1000;
        if (gap_ms > s_dbg_max_gap_ms)
            s_dbg_max_gap_ms = gap_ms;
        s_dbg_last_us = now_us;
        s_dbg_steps++;
    }

    bool done;
    switch (s_boot_fade_phase)
    {
    case UI_BOOT_FADE_IN:
        done = bsp_board_lcd_fade_step_fine(0, BSP_LCD_BK_DEFAULT_PCT, elapsed_ms, UI_BOOT_FADE_IN_MS);
        if (done)
        {
            ESP_LOGW("FADE_DIAG", "渐亮IN 完成: 实际步数=%lu 理论=%d 最大间隔=%lums (设定%dms)",
                     (unsigned long)s_dbg_steps, UI_BOOT_FADE_IN_MS / UI_BOOT_FADE_STEP_MS,
                     (unsigned long)s_dbg_max_gap_ms, UI_BOOT_FADE_STEP_MS);
            s_boot_fade_phase = UI_BOOT_FADE_IDLE;
            esp_timer_stop(s_boot_fade_tmr);
        }
        break;

    case UI_BOOT_FADE_OUT:
        done = bsp_board_lcd_fade_step_fine(BSP_LCD_BK_DEFAULT_PCT, 0, elapsed_ms, UI_BOOT_FADE_MS);
        if (done)
        {
            /* 暗到底（背光=0，此刻屏幕不可见，做任何画面变更都不会被看到）：
             * 把"删掉开机 logo 静态图"转交给 LVGL 线程执行。
             * ★本回调运行在 esp_timer 线程，绝不能在这里直接 lv_obj_del —— 非 LVGL
             *   线程操作对象会撞 lvgl_port_lock 断言直接 abort（同 BUG-029/BUG-030）。
             * 置位后由 main_gif_switch_timer_cb（LVGL 线程）在下一拍真正删除并揭图；
             * 那里同时也会处理可能存在的情绪/状态 pending 切图请求。 */
            s_boot_logo_del_pending = true;
            if (s_gif_switch_tmr != NULL)
                lv_timer_resume(s_gif_switch_tmr); // 唤醒 LVGL 线程去消费上面的删除请求
            ESP_LOGW("FADE_DIAG", "渐暗OUT 完成: 实际步数=%lu 理论=%d 最大间隔=%lums (设定%dms)",
                     (unsigned long)s_dbg_steps, UI_BOOT_FADE_MS / UI_BOOT_FADE_STEP_MS,
                     (unsigned long)s_dbg_max_gap_ms, UI_BOOT_FADE_STEP_MS);
            /* ★不能在这里直接转 IN2 开始渐亮：上面只是【请求】删 logo，真正的
             * lv_obj_del 要等 LVGL 线程下一拍才执行。直接渐亮＝和 LVGL 线程赛跑，
             * 它一慢就变成"背光先亮、logo 后删"，即用户看到的"亮着切图"。
             * 改为转入 WAIT_SWAP，全黑等揭图真正完成。 */
            s_boot_fade_phase = UI_BOOT_FADE_WAIT_SWAP;
            s_boot_fade_start_us = esp_timer_get_time(); // 兼作等待超时的计时基准
        }
        break;

    case UI_BOOT_FADE_WAIT_SWAP:
        /* 全黑保持（背光已是 0，本分支不碰背光＝屏幕维持不可见），直到 LVGL 线程
         * 在 main_gif_switch_timer_cb 里真正删掉 logo 并把 pending 清零，才开始渐亮。
         * 这样"揭图 → 渐亮"变成因果串行，不再依赖时长余量去赌先后。 */
        if (!s_boot_logo_del_pending)
        {
            /* ★揭图标志清零 ≠ 画面已就位：pending 是在 lv_obj_del【之前】清的，而
             * lv_obj_del 也只是摘对象 + 标脏，真正把 GIF 首帧推上屏的 flush 由
             * lv_timer_handler 异步完成。故这里【继续保持全黑】再等一个暗态窗口，
             * 让 flush 与液晶响应在看不见的时候做完（详见 UI_BOOT_SWAP_DARK_MS 注释）。 */
            if (s_boot_swap_done_us == 0)
            {
                s_boot_swap_done_us = esp_timer_get_time(); // 记下揭图时刻，开始计暗态窗口
                ESP_LOGW("FADE_DIAG", "揭图已完成（等待 %lums），暗态等 flush %dms",
                         (unsigned long)elapsed_ms, UI_BOOT_SWAP_DARK_MS);
            }
            else if ((esp_timer_get_time() - s_boot_swap_done_us) / 1000 >= UI_BOOT_SWAP_DARK_MS)
            {
                s_boot_fade_phase = UI_BOOT_FADE_IN2;
                s_boot_fade_start_us = esp_timer_get_time(); // 渐亮从此刻重新计时
            }
        }
        else if (elapsed_ms >= UI_BOOT_SWAP_WAIT_MAX_MS)
        {
            /* 兜底：LVGL 线程迟迟没消费（异常忙 / timer 被别的早退分支吃掉）。
             * 宁可退化成"可能看到切换"，也不能永久黑屏。 */
            ESP_LOGE("FADE_DIAG", "等揭图超时 %lums，强制渐亮（可能看到切换）",
                     (unsigned long)elapsed_ms);
            s_boot_fade_phase = UI_BOOT_FADE_IN2;
            s_boot_fade_start_us = esp_timer_get_time();
        }
        break;

    case UI_BOOT_FADE_IN2:
        done = bsp_board_lcd_fade_step_fine(0, BSP_LCD_BK_DEFAULT_PCT, elapsed_ms, UI_BOOT_FADE_IN_MS);
        if (done)
        {
            ESP_LOGW("FADE_DIAG", "渐亮IN2 完成: 实际步数=%lu 理论=%d 最大间隔=%lums (设定%dms)",
                     (unsigned long)s_dbg_steps, UI_BOOT_FADE_IN_MS / UI_BOOT_FADE_STEP_MS,
                     (unsigned long)s_dbg_max_gap_ms, UI_BOOT_FADE_STEP_MS);
            s_boot_fade_phase = UI_BOOT_FADE_IDLE;
            esp_timer_stop(s_boot_fade_tmr);
        }
        break;

    default:
        esp_timer_stop(s_boot_fade_tmr);
        break;
    }
}

/**
 * @brief 延迟切换 timer 回调:在 GIF 自己的 timer 回调彻底返回后的下一个
 *        lv_timer_handler 迭代中执行,此刻 gif_obj 已稳定(pause 在最后一帧),set_src 安全
 *
 * 同样在 LVGL 线程,严禁 lvgl_port_lock。1_3-shot:执行一次后 pause,等下次 READY 再 resume。
 */
static void main_gif_switch_timer_cb(lv_timer_t *t)
{
    lv_timer_pause(t);

    /* ── 最优先：消费"删除开机 logo 静态图"请求（由 ui_boot_fade_timer_cb 在
     *   esp_timer 线程置位，真正的 lv_obj_del 必须回到本 LVGL 线程执行）。
     * 放在所有早退分支【之前】：即便此刻已切到功能层（下面 s_view 判断会 return），
     * logo 图也必须删掉，否则它会永远盖在最上层，退回主界面时挡住 GIF。
     * 删除时机在渐暗到底、背光=0，屏幕不可见，故无视觉跳变。 */
    if (s_boot_logo_del_pending)
    {
        s_boot_logo_del_pending = false;
        if (s_boot_logo_img != NULL)
        {
            lv_obj_del(s_boot_logo_img);
            s_boot_logo_img = NULL; // 置空同时解除 ready_cb 的早退，主轮播恢复自动切换
            ESP_LOGI("GIFDBG", "timer_cb: 已删除开机 logo 静态图 → 揭开主轮播");

            /* ★开机首张动作「补投」（2026-08-31）──────────────────────────────
             * 【解决什么】舵机动作的投递点是 LV_EVENT_READY（GIF 播完一轮，见本函数
             *   末尾 idx 分支），故开机揭开的【第一张】GIF 整整一轮内没有任何舵机动作
             *   配套，表现为"GIF 已经显示了一两秒，舵机才开始动"。第二张起才正常。
             * 【怎么解决】揭图这一刻主动补投一次当前索引对应的空闲动作，把第一张也补齐。
             *
             * 【为什么放这里】必须同时满足三个条件，本行是唯一都满足的位置：
             *   ① 屏上已经是主轮播 GIF（logo 刚删）——放 application.c 会在 logo 期间
             *      就让舵机动起来，比原问题更怪；
             *   ② interaction_manager 已初始化——本点比 ui_notify_boot_ready 至少晚
             *      UI_BOOT_FADE_MS(400ms)，早已就绪，不会被 interaction.c 的
             *      s_ia_inited==false 静默丢弃；
             *   ③ 本函数就在 LVGL 线程，可直接读 s_gif_cur_index，无需跨文件接口。
             *
             * 【为什么不会复发"flush 风暴"】只复用 s_idle_actions 表内条目（其
             *   keep_screen=true，播完不调 ui_resume_main_gif_loop），不新增
             *   "动作→切图"的回边；且只在开机揭图这一次执行，READY 主循环完全不动。
             * 【为什么判 interaction_is_playing】揭图瞬间若已有动作在跑（如唤醒/状态
             *   动作抢先），补投会排队叠加成"开机连做两个动作"，故让位跳过。 */
            if (s_idle_carousel_enabled && s_view == UI_VIEW_MAIN && !interaction_is_playing() &&
                s_gif_cur_index >= 0 && (size_t)s_gif_cur_index < IDLE_ACTION_COUNT)
            {
                ESP_LOGI("GIFDBG", "timer_cb: 开机补投首张舵机动作 idx=%d", s_gif_cur_index);
                ui_interaction_play_custom(&s_idle_actions[s_gif_cur_index]);
            }
        }
    }

    /* ── 初次联网避让：窗口内不切图（切图含文件 I/O + 解码首帧，会与联网突发争 CPU0
     *   把 taskLVGL 顶死触发 wdt）。不丢 pending，把 timer 重排到窗口末端再触发一次，
     *   届时窗口已过、联网突发已散，正常切图。仅初次联网后短暂生效（见 s_first_online_defer_until_us）。 */
    if (s_first_online_defer_until_us != 0)
    {
        int64_t remain_us = s_first_online_defer_until_us - esp_timer_get_time();
        if (remain_us > 0)
        {
            /* 睡到窗口末端再跑一拍（+10ms 余量确保跨过截止点）；resume 后本回调下次触发时
             * remain<=0 走清零分支，恢复正常 10ms 周期后按原逻辑切图。 */
            uint32_t defer_ms = (uint32_t)(remain_us / 1000) + 10;
            lv_timer_set_period(t, defer_ms);
            lv_timer_resume(t);
            ESP_LOGI("GIFDBG", "timer_cb: 初次联网避让，延后 %lu ms 再切图", (unsigned long)defer_ms);
            return; // 保留 pending，本拍不切图
        }
        /* 窗口已过：清零标志并恢复正常周期，本拍继续正常切图 */
        s_first_online_defer_until_us = 0;
        lv_timer_set_period(t, 10);
    }

    // 双保险:READY 到延迟这一拍之间若已切到功能层,放弃本次切换(不切图、不动舵机)
    if (s_view != UI_VIEW_MAIN)
    {
        s_gif_pending_idx = -1;
        s_gif_pending_path = NULL;
        return;
    }

    /* ── 最优先：logo 原地重播（同一文件，零文件重开 / 零 draw_buf 重建）────── */
    if (s_gif_pending_restart)
    {
        s_gif_pending_restart = false;
        if (gif_obj != NULL && lv_gif_is_loaded(gif_obj))
            lv_gif_restart(gif_obj); // 只把文件指针 seek 回起点并重启帧定时器
        return;
    }

    /* ── 优先：情绪触发的指定 GIF 切换（pending_path 非 NULL）──────────────
     * 情绪的舵机/震动/音频由 interaction 引擎负责，这里【只切图】，不再驱动舵机
     * （否则与 interaction 的舵机动作重复叠加打架）。 */
    const char *path = s_gif_pending_path;
    if (path != NULL)
    {
        /* ── 优先级兜底「状态 > 情绪」：对话中（s_neutral_active）只放行状态接口设置的
         * pending（is_state=true）；若本次 pending 来自情绪接口（is_state=false），丢弃，
         * 保持当前状态中性 GIF 不被情绪抢图。正常情况下 play_random_emotion 已在源头屏蔽
         * 对话中触摸，这里是纵深防御。 */
        bool is_state = s_gif_pending_is_state;
        s_gif_pending_path = NULL;
        s_gif_pending_is_state = false;
        if (s_neutral_active && !is_state)
            return; // 对话中丢弃情绪切图请求
        if (gif_obj != NULL)
        {
            /* ★ 切图前先验证文件存在：lv_gif_set_src 切到不存在的文件会让 gif 对象
             * 进入坏状态，下一拍 gif_next_frame_task_cb→seekFile 访问无效句柄而崩溃
             * （StoreProhibited）。占位路径阶段文件多半不存在，故必须先试开校验。 */
            lv_fs_file_t f;
            if (lv_fs_open(&f, path, LV_FS_MODE_RD) == LV_FS_RES_OK)
            {
                lv_fs_close(&f);
                /* 文件存在只是第一关，尺寸超标同样会在 gif_initialize 分配
                 * draw_buf 失败后让 timer 继续跑，下一帧崩溃（同 BUG-010 根因）。 */
                if (main_gif_check_size_safe(path))
                {
                    ESP_LOGI("GIFDBG", "timer_cb: 切图(pending_path)=%s is_state=%d", path, is_state);
                    lv_gif_set_src(gif_obj, path); // 文件存在且尺寸安全才切
                    // s_gif_cur_index 不更新：情绪 GIF 不在 s_main_gif_table 索引体系内
                }
            }
            else
            {
                ESP_LOGW(TAG, "情绪 GIF 不存在，保持当前画面: %s", path);
            }
        }
        return;
    }

    /* ── 否则：自动随机循环切换（待机/无人触摸时）────────────────────────── */
    int idx = s_gif_pending_idx;
    s_gif_pending_idx = -1;
    if (idx < 0)
        return;

    /* ★【2026-09-01】最后一道闸：本分支末尾会 ui_interaction_play_custom() 投递
     * 空闲舵机动作，是"进功能盘后舵机还在动"这条链路的【实际出口】。
     * 上游三处（ready_cb / resume_loop / request_emotion_gif）已各自把住，这里再拦
     * 一次是纵深防御：pending_idx 可能是进功能盘【之前】就排好的，那时判据还成立，
     * 等本回调真正跑到时人已经在进功能盘的路上了。 */
    if (!main_idle_loop_active())
        return;

    // 动作播放中：不在这里自动切图（让位）。空闲动作 gif_path=NULL 不切图、keep_screen=true
    // 不 resume，故它播放期间被挡也不会停摆——下一张只由 GIF 的 READY 驱动（ready_cb），解耦。
    if (interaction_is_playing())
    {
        ESP_LOGI("GIFDBG", "切图被舵机挡住→丢弃 idx=%d", idx);
        return;
    }

    /* ── 深度待机（唯一低功耗档）中：屏已关(bsp_board_lcd_disp_off)、舵机 PWM 已停
     * (bsp_servo_idle)，直接丢弃本次切图，绝不触发任何情绪动作 / 舵机 PWM。────────────
     * 删一级后低功耗不再靠情绪队列驱动 GIF/头部动作（原一级那套 ui_interaction_play
     * 随机情绪已删除）：黑屏期什么队列都不放，退出深度待机由 ui_resume_main_gif_loop
     * 恢复空闲队列的正常随机 GIF 轮播。standby_is_deep_active() 为唯一低功耗标志。 */
    if (standby_is_deep_active())
    {
        // 【诊断-DBG5】深度待机期本 timer 仍被唤醒执行到这里，确认丢弃分支被命中的频率（验证OK后可删）
        ESP_LOGW("GIFDBG", "[DBG5] timer_cb: 深度待机中，丢弃本次切图（gif_obj=%p pending_idx此次作废）", (void *)gif_obj);
        return;
    }

    /* ── 远程控制态（app 手动下发 GIF/舵机后的 30s 冻结窗口）：冻结空闲序列 ──────────
     * 用户在 app 上手动指定了 GIF / 舵机角度，30s 内画面与姿态应【停在用户设定的样子】，
     * 故这里直接丢弃本次自动轮播（既不切图、也不投递下面的随附舵机动作）。
     * 注意：冻结【只针对空闲态的 GIF+舵机这一条链路】，唤醒/对话/提示音/按键等全部照常。
     * 退出由 remote_control 模块负责（30s 到期、或被触摸/唤醒/待机打断），退出时会调
     * ui_resume_main_gif_loop() 重新驱动本轮播。 */
    if (remote_control_is_active())
    {
        ESP_LOGI("GIFDBG", "timer_cb: 远程控制冻结中，丢弃本次空闲切图与舵机动作");
        return;
    }

    /* ── 切图：GIF 完整播完一轮(READY) 才走到这里切下一张（READY 驱动，GIF 永不被打断）。── */
    main_gif_apply_index(idx, /*with_servo=*/false);

    /* ── 空闲（非待机）：切完图，附带投递本图对应动作（只舵机+震动）。
     * 动作 gif_path=NULL（不切图）、keep_screen=true（不 resume、不触发下一张），舵机在
     * 这一轮 GIF 期间摆完即安静，下一张由 READY 驱动。GIF 与舵机各自完整、互不打断。
     * 动作按 GIF 索引取同序号（s_idle_actions 与 s_main_gif_table 对齐）。 */
    if (idx >= 0 && (size_t)idx < IDLE_ACTION_COUNT)
    {
        /* 【堆损坏排查·2026-07-29 已排除，暂停用】
         *   本回调运行在 taskLVGL 线程（栈 8192B，MALLOC_CAP_INTERNAL），曾因"栈在内部
         *   SRAM、与被踩地址同段"被列为头号嫌疑。实测水位从未触及 1024B 阈值 → 排除。
         *   真凶是 Tmr Svc 栈溢出（详见本文件 main_gif_apply_index 处注释）。 */
        // WARN_TASK_STACK_LOW("HEAPCHK", NULL, 1024);
        // 【堆探针·已停用】舵机入队前后查堆；根因已修，全堆扫描太重触发看门狗，暂注释保留。
        // if (!heap_caps_check_integrity_all(true))
        //     ESP_LOGE("HEAPCHK", "★堆损坏@投递舵机前(切图已完成) idx=%d", idx);
        ESP_LOGI("GIFDBG", "timer_cb: 切图 idx=%d + 投递舵机动作", idx);
        // 【排查用·临时注释】关闭空闲舵机动作，排查麦克风削波尖峰是否与舵机驱动噪声相关
        ui_interaction_play_custom(&s_idle_actions[idx]);
        // if (!heap_caps_check_integrity_all(true))
        //     ESP_LOGE("HEAPCHK", "★堆损坏@投递舵机后 idx=%d", idx);
    }
}

/**
 * @brief 请求把主界面 GIF 切到指定路径（情绪触发用，跨线程安全）
 *
 * 由 interaction 引擎在播放某情绪时调用，传入该情绪的 GIF 路径。
 * 本函数可在任意线程调用：只设置 pending 标记并唤醒延迟 timer，真正的
 * lv_gif_set_src 由 main_gif_switch_timer_cb 在 LVGL 线程执行（BUG-010：
 * lv_gif_set_src 必须在 LVGL 线程调，否则销毁/重建 draw_buf 引发崩溃）。
 *
 * @param gif_path 目标 GIF 的 SPIFFS/外挂 flash 路径（如 "S:/gif/happy.gif"）；
 *                 传 NULL 或空串则忽略（该情绪无指定 GIF 时不切图）。
 */
void ui_request_emotion_gif(const char *gif_path)
{
    if (gif_path == NULL || gif_path[0] == '\0')
        return;
    /* 【2026-09-01】同 main_gif_ready_cb：进功能盘渐暗期 s_view 仍是 MAIN，
     * 用统一判据把这段窗口也关上，避免往一块马上要被隐藏的 GIF 上切图。 */
    if (!main_idle_loop_active())
        return;                     // 非主界面不切（功能层/闹钟编辑时 GIF 已隐藏）
    s_gif_pending_is_state = false; // 本次 pending 来自情绪接口（优先级低于状态 GIF）
    s_gif_pending_path = gif_path;  // 指针赋值原子；指向常量字符串，生命周期安全
    if (s_gif_switch_tmr != NULL)
        lv_timer_resume(s_gif_switch_tmr); // 唤醒延迟切换 timer，下个 LVGL tick 执行
}

/**
 * @brief 请求把主界面 GIF 切到指定路径（对话状态动作用，跨线程安全，高优先级）
 *
 * 与 ui_request_emotion_gif 唯一区别：设 s_gif_pending_is_state=true，使本次切图被
 * main_gif_switch_timer_cb 视为「状态切图」（高优先级），不被对话中（s_neutral_active）
 * 的「丢弃情绪切图」兜底误伤（D.1：否则现象为「舵机震动动了但 GIF 没切」）。
 * 供 interaction worker 执行状态动作时切图调用。
 */
void ui_request_state_gif(const char *gif_path)
{
    if (gif_path == NULL || gif_path[0] == '\0')
        return;
    if (s_view != UI_VIEW_MAIN)
        return;                    // 非主界面不切
    s_gif_pending_is_state = true; // 高优先级状态切图（绕过兜底丢弃）
    s_gif_pending_path = gif_path; // 指针赋值原子；指向常量字符串，生命周期安全
    if (s_gif_switch_tmr != NULL)
        lv_timer_resume(s_gif_switch_tmr); // 唤醒延迟切换 timer，下个 LVGL tick 执行
}

/**
 * @brief 通知 UI「初次联网成功」，开启短暂 GIF 切图避让窗口（见头文件说明）
 *
 * 只做一次原子赋值（int64_t 写在 32 位 ESP32-S3 上非严格原子，但本值仅由 LVGL 线程
 * 消费、且写入与读取相差远大于一次写周期，撕裂不会造成逻辑错误——最坏是多避让/少避让
 * 一拍），无需加 LVGL 锁，可在 bsp_board_wifi_main（非 LVGL 线程）直接调用。
 */
void ui_notify_first_online(void)
{
    s_first_online_defer_until_us = esp_timer_get_time() + (int64_t)FIRST_ONLINE_DEFER_MS * 1000;
    ESP_LOGI("GIFDBG", "初次联网：开启 %d ms GIF 切图避让窗口", FIRST_ONLINE_DEFER_MS);
}

/**
 * @brief 按会话状态切换中性 GIF（待机/用户说话/大模型说话），跨线程安全
 *
 * 由 session 状态机在状态切换点调用（任意线程）。
 * - LISTENING / SPEAKING：置 s_neutral_active=true（锁定对话态，main_gif_ready_cb
 *   期间不再自动随机循环），从对应状态动作组随机选一条，调 ui_interaction_play_custom
 *   交给情绪 worker 执行【切图 + 三轴舵机 + 震动】（worker 内用 ui_request_state_gif
 *   高优先级切图、播完 keep_screen 停在该 GIF）。本函数只入队，不阻塞调用线程。
 * - IDLE：清 s_neutral_active，调 ui_resume_main_gif_loop() 恢复主界面自动随机循环（不带动作）。
 *
 * @param st 目标会话状态（neutral_gif_state_t）
 */
void ui_set_neutral_gif_state(neutral_gif_state_t st)
{
    /* ★★【2026-08-25 修复：s_neutral_active 永久泄漏】★★
     * IDLE 分支必须放在下面 s_view 早退【之前】，原因如下：
     *
     * 【原来的写法】函数第一行就是 `if (s_view != UI_VIEW_MAIN) return;`，
     *   IDLE 清标志排在它后面。于是只要会话【结束时】不在主界面，清标志这一步
     *   就被整个跳过 —— 而这恰恰是必然发生的场景：
     *     主界面唤醒开始对话（下方置 s_neutral_active=true）
     *     → 对话进行中倒计时到期，ui_show_countdown_expired() 把 s_view
     *       强切成 UI_VIEW_FUNCTION_MENU
     *     → TTS 播完，session.c 调本函数传 NEUTRAL_IDLE 想清标志
     *     → 撞上早退，标志【再也清不掉】。
     *
     * 【泄漏后果是永久性的，且三处连锁】
     *   ① main_gif_ready_cb()：if (s_neutral_active) return; → 主界面 GIF
     *      播完一轮不再排下一张，画面定格在最后一帧；
     *   ② ui_resume_main_gif_loop()：同样早退 → 轮播再也恢复不了；
     *   ③ main_gif_switch_timer_cb()：s_neutral_active && !is_state 会丢弃
     *      所有情绪切图 → 摸头/摸肚子全都没有画面反应。
     *   而音频链路与本标志无关，故表现为「屏幕卡死但对话还能正常进行」。
     *
     * 【为什么提前是安全的】清标志是【纯状态写】，不碰任何 LVGL 对象；
     *   紧随其后的 ui_resume_main_gif_loop() 自身第一行就判 s_view != MAIN
     *   并早退，非主界面时它不会去动 GIF。所以提前只会让标志被正确清掉，
     *   不会在功能层产生任何画面副作用。 */
    if (st == NEUTRAL_IDLE)
    {
        s_neutral_active = false;
        ui_resume_main_gif_loop(); // 内部自带 s_view==MAIN 判断与跨线程安全
        return;
    }

    /* LISTENING / SPEAKING 才需要限定主界面：它们要切图 + 投递舵机动作，
     * 在功能层执行会抢走功能页画面。IDLE 已在上面处理完，不受此限制。 */
    if (s_view != UI_VIEW_MAIN)
        return; // 非主界面（功能盘/游戏/闹钟编辑）不切，避免抢功能层画面

    /* LISTENING / SPEAKING：从对应状态动作组随机选一条 */
    const ia_custom_action_t *list = NULL;
    size_t count = 0;
    if (st == NEUTRAL_LISTENING)
    {
        list = s_listening_actions;
        count = sizeof(s_listening_actions) / sizeof(s_listening_actions[0]);
    }
    else /* NEUTRAL_SPEAKING */
    {
        list = s_speaking_actions;
        count = sizeof(s_speaking_actions) / sizeof(s_speaking_actions[0]);
    }
    if (list == NULL || count == 0)
        return;

    // 先锁对话态，再投递：确保投递后到 worker 真正切图之间，待机循环不会插入随机切图。
    s_neutral_active = true;

    /* ── 抢占正在播放的【空闲动作】：对话状态动作优先级高于空闲动作 ──────────────
     * 【为什么必须抢占】interaction worker 是【串行】的：一次只执行一条，且执行中会
     *   xSemaphoreTake(s_ia_d1_2_sem, 10s) 死等舵机摆完（见 interaction_play_custom_blocking）。
     *   若不抢占，唤醒时本次状态动作只是【入队排在空闲动作后面】，必须等空闲动作把整套
     *   count 次摆动做完才轮到 → 现象＝「唤醒了，但舵机还在做空闲动作、GIF 也不切，
     *   要等它演完」（可长达数秒），对话响应严重滞后。
     *
     * 【为什么用 is_idle_action 而不是 is_playing】只抢占低优先级空闲动作
     *   （is_idle=true）；对话状态动作本身 is_idle=false，故【绝不会误伤】正在执行的
     *   上一条对话动作（如 SPEAKING 打断 LISTENING 时不会互相踩）。
     *
     * 【两个 flush 的分工】这是本项目既定范式（见 interaction.h 中 interaction_flush_queue
     *   的说明，与 ui_interaction_play 的触摸情绪抢占写法一致）：
     *     - interaction_flush_queue()：清掉队列里【未执行】的存量请求；
     *     - servo_manager_flush()    ：打断【正在执行】的那条舵机插值。
     *
     * 【舵机不会停在歪角度】servo worker 收到打断后无条件平滑归中 90°（被打断时还会
     *   先清打断标志再归中，防归中自身被掐断，见 servo_manager.c 归中策略），故最终
     *   姿态一定是中位——这与 LISTENING「不动、安静倾听」的目标姿态天然一致。 */
    if (interaction_is_idle_action())
    {
        ESP_LOGI(TAG, "对话状态动作抢占当前空闲动作（立即打断，舵机归中）");
        interaction_flush_queue(); // 清队列中未执行的空闲动作
        servo_manager_flush();     // 打断正在执行的空闲舵机（worker 随后归中 90°）
    }

    const ia_custom_action_t *act = &list[esp_random() % count]; // 组内随机选一条
    ui_interaction_play_custom(act);                             // 切图 + 舵机 + 震动全交给情绪 worker 执行
}

/**
 * @brief 情绪播放完毕后，恢复主界面自动随机 GIF + 舵机循环（跨线程安全）
 *
 * 由 interaction worker（情绪播完、已清 is_playing 标志后）调用。情绪期间自动
 * 循环被暂停（main_gif_ready_cb 早退），情绪 GIF 又停在最后一帧不会再触发 READY，
 * 故需主动唤醒：设一个随机 pending 索引 + resume 延迟 timer，由 main_gif_switch_timer_cb
 * 在 LVGL 线程切到下一张随机 GIF，切图后其播完会再次触发 READY → 自动循环自然恢复。
 *
 * 跨线程安全：本函数只做指针/整型赋值 + lv_timer_resume（同 ui_request_emotion_gif 模式），
 * 真正的切图在 LVGL 线程执行，不在 SPIRAM worker 直接碰 GIF 对象（BUG-010）。
 * timer 回调内部会判 s_view==MAIN，若此刻已进功能盘则不误恢复（plan R3）。
 */
void ui_resume_main_gif_loop(void)
{
    // 【诊断-DBG3】函数入口打印 gif_obj 是否存在、当前 s_view、tmr 是否存在（验证OK后可删）
    ESP_LOGW("GIFDBG", "[DBG3] resume_loop 入口: s_view=%d gif_obj=%p tmr=%p s_gif_cur_index=%d",
             (int)s_view, (void *)gif_obj, (void *)s_gif_switch_tmr, s_gif_cur_index);
    /* 【2026-09-01 关键修复点】原判据是裸 s_view != UI_VIEW_MAIN。
     * 进功能盘改走渐变后，s_view 要到暗态（渐暗 450ms 之后）才置 HOME，而本函数
     * 恰恰会在 flush 打断动作后【立刻】被 interaction 收尾链路调到 —— 旧判据在这
     * 450ms 里放行，于是重新排图 + resume 轮播 timer，下一拍就投递出一条新的空闲
     * 舵机动作，正是"功能盘已显示、舵机还在动"的直接成因。详见 s_home_enter_pending。 */
    if (!main_idle_loop_active())
        return; // 已不在主界面（如已进功能盘/正在进的路上），不恢复
    /* ★对话态（LISTENING/SPEAKING）禁止恢复随机轮播 ────────────────────────────
     * 【竞态场景】唤醒瞬间：ui_set_neutral_gif_state 已置 s_neutral_active=true 并投递了
     *   状态动作，但 worker 仍在收尾【上一条空闲动作】；空闲动作 keep_screen=false，
     *   收尾时会调本函数（见 interaction_play_custom_blocking 结尾）。若不拦：
     *     ① 下面 s_gif_pending_path = NULL 会【抹掉】刚设好的状态 GIF pending；
     *     ② s_gif_pending_idx 被塞入一张空闲随机图 → 唤醒后先闪一张无关表情，
     *        才跳到监听图。
     * 【为何 main_gif_switch_timer_cb 里的 s_neutral_active 兜底拦不住】那道判断只作用于
     *   pending_path 分支（丢弃情绪切图），而本函数走的是【随机索引分支】，不经过那道闸，
     *   故必须在此处独立拦一次。
     * 【纵深防御】不止空闲动作收尾，任何路径在对话态误调本函数都会被挡住。
     * 状态回 IDLE 时 ui_set_neutral_gif_state(NEUTRAL_IDLE) 会先清 s_neutral_active
     * 再调本函数，那条正常恢复路径不受影响。 */
    if (s_neutral_active)
    {
        ESP_LOGI("GIFDBG", "resume_loop: 对话态中，忽略恢复请求（防抢状态 GIF）");
        return;
    }
    if (!s_idle_carousel_enabled)
        return; // 调试禁用空闲态：动作播完不恢复空闲轮播
    if (s_gif_switch_tmr == NULL)
        return;
    s_gif_pending_path = NULL; // 确保走随机分支而非情绪指定分支
    s_gif_pending_idx = main_gif_pick_next_index(s_gif_cur_index);
    ESP_LOGI("GIFDBG", "resume_loop: 动作播完恢复循环 → pending_idx=%d", s_gif_pending_idx);
    lv_timer_resume(s_gif_switch_tmr); // 下个 LVGL tick 切下一张，恢复循环
}

/**
 * @brief 定格主界面 GIF 动画（跨线程安全，需 LVGL 锁）——进深度待机第一步调用
 *
 * 供 standby.c 的 enter_deep_standby() 在【背光渐暗之前】调用，实现「先关 GIF 再降亮度」：
 *   1. lv_timer_pause(s_gif_switch_tmr)：掐断 main_gif_ready_cb 继续排队「切下一张」的链路，
 *      同时消除深度待机期间 s_gif_switch_tmr 反复被唤醒又丢弃的空转（治标又治本的第一环）。
 *   2. lv_gif_pause(gif_obj)：真正冻结 GIF 内部逐帧推进定时器，画面【定格在当前帧】
 *      （不黑屏、不复位到首帧），停止解码/刷新，彻底不再耗 CPU。
 *
 * 定格瞬间画面完全静止、与当前帧无差别，此刻屏幕仍亮，用户无感；之后再对静止画面渐暗。
 * ★根因关联：GIF 持续排队解码与 standby_task 同优先级(5)争抢 CPU，是背光渐暗延迟十几秒
 *   才开始的元凶——先定格 GIF 让出 CPU，standby_task 才能顺畅执行到后续的渐暗步骤。
 *
 * 跨线程：本函数从 standby_task（非 LVGL 线程）调用，碰 gif_obj 必须持 LVGL 锁（BUG-010）。
 * 取锁失败即超时跳过、打警告，不阻塞、不硬等——最坏退化回「GIF 停在最后一帧空转」的现状，
 * 绝不更糟，也绝不崩/卡死。s_gif_switch_tmr 的 pause 是纯 timer 指针操作，无需持锁，故放锁外。
 */
void ui_pause_main_gif(void)
{
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr); // 先停「切下一张」排队（无需 LVGL 锁）
    if (gif_obj == NULL)
        return;
    if (lvgl_port_lock(200))
    {
        lv_gif_pause(gif_obj); // 定格当前帧，停止内部解码/刷新
        lvgl_port_unlock();
        ESP_LOGI("GIFDBG", "ui_pause_main_gif: GIF 已定格当前帧（进深度待机）");
    }
    else
    {
        ESP_LOGW("GIFDBG", "ui_pause_main_gif 取 LVGL 锁超时，跳过定格（GIF 可能仍在最后一帧空转，无害退化）");
    }
}

/**
 * @brief 恢复主界面 GIF 动画播放（跨线程安全，需 LVGL 锁）——退深度待机、亮屏前调用
 *
 * 供 standby.c 的 standby_wake() 在 disp_on 成功【之后】、背光渐亮【之前】调用，保证
 * 「先恢复 GIF 再亮屏」：GIF 从定格帧继续播放的这段过渡，发生在背光仍为 0（屏不可见）的
 * 窗口内，亮起来时画面已经在动，用户无感。与 ui_pause_main_gif() 成对。
 *
 * 跨线程规避 / 取锁失败退化策略同 ui_pause_main_gif()。s_gif_switch_tmr 的 resume
 * 恢复自动轮播排队，纯 timer 指针操作、无需持锁，放锁外。
 */
void ui_resume_main_gif(void)
{
    if (gif_obj != NULL)
    {
        if (lvgl_port_lock(200))
        {
            lv_gif_resume(gif_obj); // 从定格帧继续播放
            lvgl_port_unlock();
            ESP_LOGI("GIFDBG", "ui_resume_main_gif: GIF 已从定格帧恢复播放（退深度待机）");
        }
        else
        {
            ESP_LOGW("GIFDBG", "ui_resume_main_gif 取 LVGL 锁超时，跳过恢复（GIF 可能仍停在定格帧，无害退化）");
        }
    }
    if (s_gif_switch_tmr != NULL)
        lv_timer_resume(s_gif_switch_tmr); // 恢复「切下一张」自动轮播排队
}

/**
 * @brief 开启/关闭主界面「空闲态自动轮播」（见 ui_port.h 声明）
 */
void ui_set_idle_carousel_enabled(bool enabled)
{
    s_idle_carousel_enabled = enabled;
}

/* ═══════════════════════════════════════════════════════════════════════════
 * 低功耗常亮时钟（2026-08-20 新增）
 *
 * 【需求】二级低功耗原本的终点是「渐变全黑 + 关显示控制器」，现改为
 *   「渐变到 STANDBY_CLOCK_BK_PCT(默认10%) + 常驻显示当前时间」，把设备
 *   静置形态从「黑砖」变成「暗态座钟」。进低功耗的其余步骤（回主界面 /
 *   关唤醒词 / 停马达 / 清队列 / 舵机归中停 PWM / 压音量）一律不变。
 *
 * 【为什么不走 menu_enter_fn_page(FN_PAGE_TIME)】那条路会把 s_view 切成
 *   UI_VIEW_FUNCTION_MENU，代价有三：
 *     ① 功能层空闲定时器（menu_idle_timeout_cb）到点会把界面踢回主界面并
 *        顺手 main_gif_kick_resume() —— 待机中 GIF 自己活过来，省电全白费；
 *     ② s_view 变了会影响触摸分发与 ui_resume_main_gif_loop 的早退判断；
 *     ③ 退待机要多一次跨线程切页。
 *   故本方案【不动 s_view】（保持 UI_VIEW_MAIN），只把时间页这一组对象
 *   叠在主界面之上显示 —— s_time_page 本就是自带根容器、靠 flag 独立
 *   show/hide 的组件（见 time_page_render / time_page_hide），零重写。
 *
 * 【与 ui_pause_main_gif 的分工】那个只负责「定格」（lv_gif_pause），对象
 *   仍然可见；以前无所谓（反正马上全黑），现在屏留着 10% 亮着，定格的 GIF
 *   会和时间页【叠在一起显示】，故本函数必须额外把 gif_obj 隐藏掉。
 * ═══════════════════════════════════════════════════════════════════════════ */

/* s_standby_clock_on 的定义已上移到 time_page_render 之前（时间页对象声明处），
 * 因为那个函数要读它来决定套哪套样式。此处不再重复定义。 */

void ui_standby_clock_show(void)
{
    if (!lvgl_port_lock(200))
    {
        // 取锁失败：不显示时钟，退化为「10% 亮度下停留在定格 GIF」。无害，
        // 且下次唤醒会走 hide 把状态复原，不会留下残影。
        ESP_LOGW(TAG, "ui_standby_clock_show 取 LVGL 锁超时，跳过显示待机时钟");
        return;
    }

    // 1) GIF：停排队 + 定格 + 【隐藏】。前两件 ui_pause_main_gif 已做过（幂等再做一次
    //    无副作用），隐藏是本函数独有的一步，理由见上方注释。
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj);
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }

    // 2) 【2026-08-20 修】先确保共享面板已创建 —— 这是「时钟只有先进过一次功能盘
    //    才显示得出来」的根因：s_menu_panel 是懒创建的（首次进功能盘时才
    //    lv_obj_create，见 ensure_menu_panel），而时间页 s_time_page 是它的子对象
    //    （time_page_render 里 lv_obj_create(s_menu_panel)）。开机后没进过功能盘就
    //    进低功耗时，面板为 NULL → time_page_render() 首行 `if (s_menu_panel == NULL)
    //    return;` 直接返回什么都不画 → 加上 GIF 已被本函数藏起来，屏幕上空无一物，
    //    10% 背光照着一块纯黑，观感就是「屏幕整个灭了」。ensure_menu_panel 幂等。
    ensure_menu_panel();

    // 3) 显示共享面板（全屏黑底），并把面板上【属于其他页面】的对象一并藏好，
    //    只留时间页可见。s_menu_panel 是功能盘/功能页共用的容器，若不清干净，
    //    上次停留过的标题/正文/功能盘大图标会残留在时钟背后。
    /* ★★【2026-08-25 修复：白名单式清理 → 全量清理】★★
     *
     * 【原来的写法错在哪】这里原先只 add_flag 了 s_menu_title / s_menu_body /
     *   s_home_icon 三个对象，然后就把整块 s_menu_panel clear_flag 放出来。
     *   可是 s_menu_panel 底下挂着的远不止这三个——番茄钟的 6 个预设色块与
     *   min/colon/sec 标签、闹钟页对象、天气页对象、日历页对象全在里面。
     *   凡是【不在这三个白名单里、且当前处于 unhide 状态】的子对象，都会随着
     *   面板一起重新显示出来。
     *   实测照片即此：低功耗时间页「12:22」直接压在番茄钟的六个彩色预设块 +
     *   「00:00」上面，四层叠加。
     *
     * 【改法】改用现成的 menu_clear_func_pages()——它逐个调 alarm_page_hide /
     *   countdown_page_hide / weather_page_hide / time_page_hide /
     *   calendar_page_hide 并藏掉功能盘大图标，是本文件里唯一"清得干净"的入口，
     *   进纯文字层时本来就用它。这里复用，不再自己维护一份必然漏项的白名单。
     *
     * 【顺序】必须先全清、再往下走 time_page_render() 把时间页单独放出来；
     *   menu_clear_func_pages 里的 time_page_hide 会先把它藏掉，正好由后面重画。
     *
     * 【s_edit_panel 单独处理】它是独立于 s_menu_panel 的另一块全屏面板（闹钟
     *   编辑页专用，闹钟响铃结束会进那一页），不在 menu_clear_func_pages 的覆盖
     *   范围内，故必须单独藏一次。 */
    menu_clear_func_pages();

    if (s_edit_panel != NULL)
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);

    if (s_menu_panel != NULL)
    {
        if (s_menu_title)
            lv_obj_add_flag(s_menu_title, LV_OBJ_FLAG_HIDDEN);
        if (s_menu_body)
            lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    }

    // 4) ★必须【先】置位再渲染：time_page_render 内部要读 s_standby_clock_on 决定
    //    大号时间套哪套样式（待机时钟=闹钟字体/白/居中，功能盘=215px/蓝/压瘦）。
    //    若像最初那样渲染完再置位，首帧会用错样式，还会白白去加载 84KB 外挂字体。
    //    本标志同时也是 main_clock_tick_cb 每分钟补刷的放行条件。
    s_standby_clock_on = true;

    // 5) 渲染时间页（内部含首次建对象 + 套样式 + 取当前时间/温度 + unhide 整组）
    time_page_render();

    lvgl_port_unlock();
    ESP_LOGI(TAG, "待机时钟已显示（低功耗常亮）");
}

void ui_standby_clock_hide(void)
{
    if (!lvgl_port_lock(200))
    {
        // 取锁失败：标志仍会被清（下面），避免 tick 继续刷一个已经要撤掉的页面；
        // 但对象没藏成，会退化为「醒来后时钟压在 GIF 上」。故此处【不清标志】而是
        // 直接返回，交由调用方的正常流程（standby_wake 后续仍会亮屏）呈现，
        // 下一次进/出待机会再次尝试。宁可这次没藏干净，也不能让 tick 与真实状态脱节。
        ESP_LOGW(TAG, "ui_standby_clock_hide 取 LVGL 锁超时，跳过隐藏待机时钟");
        return;
    }

    s_standby_clock_on = false; // 先清标志，立刻停掉每分钟刷新

    // 1) 收起时间页整组 + 共享面板
    time_page_hide();
    if (s_menu_panel != NULL)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);

    // 2) GIF 只做 unhide。「从定格帧恢复播放 + resume 轮播排队」是
    //    ui_resume_main_gif() 的职责，调用方紧接着就会调它，此处不重复。
    if (gif_obj != NULL)
        lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);

    lvgl_port_unlock();
    ESP_LOGI(TAG, "待机时钟已隐藏（退低功耗）");
}

/**
 * @brief 从功能层退回主界面后,重新启动 GIF 循环
 *
 * 在功能层期间我们屏蔽了 LV_EVENT_READY 的切换,而 GIF 播完一轮会被 LVGL 自动暂停
 * (loop_count<0),于是退回主界面时可能停在最后一帧、不再播放也不再触发切换。
 * 这里用 lv_gif_restart 把当前 GIF 从头重播,播完会再次触发 READY,循环随之恢复。
 *
 * @note 仅在【已持有 lvgl_port 锁】或【LVGL 线程】上下文调用。
 */
static void main_gif_kick_resume(void)
{
    if (gif_obj == NULL)
        return;
    s_gif_pending_idx = -1;  // 清掉功能层期间可能残留的待切换标记
    lv_gif_restart(gif_obj); // 当前张从头重播,播完触发 READY → 恢复"播完即切"循环
}

static void main_gif_create(void)
{
    if (gif_obj != NULL)
        return;
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL)
    {
        ESP_LOGE(TAG, "lv_screen_active 返回 NULL，跳过 GIF 创建");
        return;
    }
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_text_font(scr, &font_cn_16, 0);
    gif_obj = lv_gif_create(scr);
    lv_gif_set_color_format(gif_obj, LV_COLOR_FORMAT_RGB565);

    // 注册"播完一轮"事件回调(回调里只置 flag,真正切换见 main_gif_switch_timer_cb)
    lv_obj_add_event_cb(gif_obj, main_gif_ready_cb, LV_EVENT_READY, NULL);

    // 创建延迟切换 1_3-shot 定时器:周期取很小值(下个 tick 触发即可),先 pause
    s_gif_switch_tmr = lv_timer_create(main_gif_switch_timer_cb, 10, NULL);
    lv_timer_pause(s_gif_switch_tmr);

    /* 开机 logo 背光渐变专用 esp_timer（硬件定时器，不受 LVGL 线程忙闲影响）。
     * 只创建不启动，由 main_gif_create 末尾 / ui_notify_boot_ready 按需 start。
     * dispatch_method 用默认的 ESP_TIMER_TASK：回调在 esp_timer 任务上下文跑，
     * 可以调 ledc_* 与 ESP_LOGW（若用 ISR 分发则两者都不允许）。 */
    const esp_timer_create_args_t fade_tmr_args = {
        .callback = ui_boot_fade_timer_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "boot_fade",
    };
    esp_err_t fade_tmr_err = esp_timer_create(&fade_tmr_args, &s_boot_fade_tmr);
    if (fade_tmr_err != ESP_OK)
    {
        s_boot_fade_tmr = NULL; // 创建失败：下面 start 处会跳过渐变，直接常亮，不影响主流程
        ESP_LOGE(TAG, "开机渐变 esp_timer 创建失败: %s", esp_err_to_name(fade_tmr_err));
    }

    // 进功能盘背光渐变专用 timer，先 pause
    s_home_fade_tmr = lv_timer_create(ui_home_fade_timer_cb, UI_BOOT_FADE_STEP_MS, NULL);
    lv_timer_pause(s_home_fade_tmr);

    // 左右耳切换图标专用渐暗渐亮 timer，先 pause
    s_home_icon_fade_tmr = lv_timer_create(home_icon_fade_timer_cb, UI_BOOT_FADE_STEP_MS, NULL);
    lv_timer_pause(s_home_icon_fade_tmr);

    // 倒计时启动后退出功能页专用渐暗渐亮 timer，先 pause
    s_cd_exit_fade_tmr = lv_timer_create(countdown_exit_fade_timer_cb, UI_BOOT_FADE_STEP_MS, NULL);
    lv_timer_pause(s_cd_exit_fade_tmr);

#if UI_FLUSH_TRACE
    /* 【临时调试】翻页 flush 追踪的延迟打印 timer，先 pause，由 ui_ftrace_begin 唤醒。
     * 与 s_home_fade_tmr 同样复用静态句柄、只 pause/resume，不做 create/delete —— */
    s_ftrace_tmr = lv_timer_create(ui_ftrace_dump_cb, UI_FTRACE_DUMP_MS, NULL);
    lv_timer_pause(s_ftrace_tmr);
#endif

#if UI_BOOT_LOGO_GIF
    /* 开机 logo：静态图，只显示不重复播放。
     * 先把主轮播首张载进 gif_obj（此刻会被 logo 图整个盖住，看不见），再在其上叠
     * logo 静态图。就绪后渐暗到底时删掉 logo 图即揭开这张 GIF，无需再 set_src。 */
    int first = main_gif_pick_next_index(-1);
    main_gif_apply_index(first, /*with_servo=*/false); // 预载底图，不配舵机(interaction 队列尚未就绪)

    /* ★先验证文件可读：lv_image_set_src 指向不存在的文件只会画不出来（不崩），
     * 但会得到一块纯黑区域盖住底图 → 表现为"开机黑屏到 boot_ready"。
     * 故打不开就直接回退：不建 logo 图，画面即上面载好的主轮播首张。 */
    lv_fs_file_t logo_f;
    if (lv_fs_open(&logo_f, UI_BOOT_LOGO_PATH, LV_FS_MODE_RD) == LV_FS_RES_OK)
    {
        lv_fs_close(&logo_f);
        s_boot_logo_img = lv_image_create(scr);
        lv_image_set_src(s_boot_logo_img, UI_BOOT_LOGO_PATH);
        lv_obj_center(s_boot_logo_img);
        lv_obj_move_foreground(s_boot_logo_img); // 确保盖在 gif_obj 之上
        ESP_LOGI(TAG, "GIF待机动画已创建,开机 logo(静态图)=%s,底图首张 idx=%d", UI_BOOT_LOGO_PATH, first);
    }
    else
    {
        ESP_LOGW(TAG, "开机 logo 图不存在(%s),回退为直接进正常轮播 idx=%d", UI_BOOT_LOGO_PATH, first);
    }

    // 开机首次渐亮：先强制拉黑背光，再交给渐变 esp_timer 推进到 100%
    bsp_board_lcd_set_brightness(0);
    s_boot_fade_phase = UI_BOOT_FADE_IN;
    s_boot_fade_start_us = esp_timer_get_time();
    if (s_boot_fade_tmr != NULL)
    {
        esp_timer_start_periodic(s_boot_fade_tmr, (uint64_t)UI_BOOT_FADE_STEP_MS * 1000);
    }
    else
    {
        // 兜底：渐变 timer 创建失败 → 直接点亮，绝不能停在背光 0 的黑屏状态
        s_boot_fade_phase = UI_BOOT_FADE_IDLE;
        bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT);
    }
#else
    // 首张:只显示不配舵机(此刻 interaction 队列尚未就绪,且很快会切到下一张)
    int first = main_gif_pick_next_index(-1);
    main_gif_apply_index(first, /*with_servo=*/false);
    ESP_LOGI(TAG, "GIF待机动画已创建,首张索引=%d", first);
#endif

    lv_obj_center(gif_obj);
    lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
}

/* ═══════════════════════════════════════════════════════════════
 * LVGL 初始化
 * ═══════════════════════════════════════════════════════════════ */
static esp_err_t app_lvgl_init(void)
{
    // ★幂等保护：未配网分支会在 WiFi/BLE 之前先调 ui_show_provision_image() 提前
    //   初始化 LVGL 显示配网图；随后主流程 ui_init() 仍会走到这里。重复
    //   lvgl_port_init/add_disp 会二次申请 30KB DMA-SRAM 且产生野 disp 指针，
    //   故已就绪则直接返回，让 ui_init 继续创建 GIF 等上层 UI。
    if (s_lvgl_ready)
    {
        ESP_LOGI(TAG, "LVGL 已初始化（配网图提前初始化过），跳过重复初始化");
        return ESP_OK;
    }

    ESP_LOGI(TAG, "最大内部连续块: %d", heap_caps_get_largest_free_block(MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL));
    ESP_LOGI(TAG, "--- Memory Check ---");
    ESP_LOGI(TAG, "Free PSRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    ESP_LOGI(TAG, "Free SRAM: %d bytes", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL || board->lcd_panel == NULL)
    {
        ESP_LOGE(TAG, "LCD 未初始化！");
        return ESP_ERR_INVALID_STATE;
    }

    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 5,
        .task_stack = 8192,
        .task_affinity = 0,
        /* 【2026-08-17 实测结论】本值改小【治不了】「触摸后几百 ms 才开始动画」，勿再尝试。
         * 原因：esp_lvgl_port.c:248 只在 lv_timer_handler() 返回 LV_NO_TIMER_READY 时
         * 才用本值兜底；而本工程恒有 1000ms 周期的 lv_timer（:7081 时钟 tick、
         * :7095 状态栏时间），LVGL 永远返回「距下个定时器还有 N ms」，N 最大约 1000
         * 且【不受本值钳制】，任务直接睡 N ms → 实测首帧延迟 70~970ms。
         * 曾把本值 500→30 实测毫无变化（那条分支走不到），已还原，避免静止时白白空转。
         * 真正的解法是跨任务改 UI 后显式 lvgl_port_task_wake()，见功能盘触摸分支。 */
        .task_max_sleep_ms = 500,
        .timer_period_ms = 10,
        // 栈必须在内部 SRAM，因为 GIF 播放会读 SPIFFS（flash cache 禁用期间 PSRAM 不可访问）
        .task_stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
    };
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK)
        return err;
    s_lvgl_ready = true; // 标记 LVGL 已就绪：此后 lvgl_port_lock 才合法（见 ui_is_ready）
    PRINT_MEM_INFO(TAG, "lvgl_port 8KB INTERNAL 任务栈分配后");

    // ── LVGL 显示配置（内部 SRAM 单缓冲 PARTIAL 模式）─────────────────────────
    // 必须放内部 SRAM 的根因（已通过对照实验定位）：
    //   AFE/WakeNet 推理任务持续高频访问 PSRAM，会抢占 PSRAM 总线带宽。
    //   若 LCD flush buffer 在 PSRAM，SPI master 的 GDMA 来不及从 PSRAM 喂数据
    //   → SPI FIFO underflow → tx_color failed + 屏幕花屏。
    //   只有把 buffer 放内部 SRAM，DMA 才能恒速供数，彻底脱离 PSRAM 带宽竞争。
    //
    // 缓冲尺寸权衡：
    //   - W*H/4 = 38400 字节：DMA-capable 内部 SRAM 装不下（碎片化后子集 < 38KB）
    //   - W*H/8 = 19200 字节：当前可分配（启动时 SRAM free ~55KB），PARTIAL 模式
    //                         会按 1/8 屏多次 flush 完成整屏，GIF 仍能流畅
    //   - 单缓冲：PARTIAL 模式不需要双缓冲，避免内存翻倍
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = board->lcd_io,
        .panel_handle = board->lcd_panel,
        // 经实测的尺寸权衡（启动时 DMA-capable 内部 SRAM ~50KB 可用）：
        //   W*H/3 = 51200 字节：
        //   W*H/4 = 38400 字节：
        //   W*H/5 = 30720 字节：
        //   W*H/6 = 24576 字节：
        //   W*H/7 = 20480 字节：
        //   W*H/8 = 19200 字节：
        //   W*H/16 = 9600 字节：
        // PSRAM 全屏 buffer 导致 SPI DMA 无法访问 PSRAM 指针 → tx_color failed
        // 稳态：W*H/8 = 19200 字节，内部 SRAM + DMA，PARTIAL 模式分 8 次 flush
        .buffer_size = BSP_LCD_WIDTH * BSP_LCD_HEIGHT / 5, // 单块 24576 字节，双缓冲共 49152 字节，控制在 55~60KB 预算内
        .double_buffer = false,
        .hres = BSP_LCD_WIDTH,
        .vres = BSP_LCD_HEIGHT,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {.swap_xy = true, .mirror_x = true, .mirror_y = false},
        .flags = {.buff_dma = true, .swap_bytes = false, .buff_spiram = false}};

    PRINT_MEM_INFO(TAG, "lvgl_port_add_disp 前(即将申请双缓冲 2x24576B DMA-SRAM draw buffer)");
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    if (lvgl_disp == NULL)
    {
        // disp 创建失败时必须返回错误，否则 ui_init 后续会调 lv_screen_active()
        // 拿到失效对象，main_desplay_create 解引用导致 LoadProhibited 崩溃
        ESP_LOGE(TAG, "lvgl_port_add_disp 失败：DMA 内部 SRAM 不足 2x24576 字节连续区");
        PRINT_MEM_INFO(TAG, "LVGL flush buffer 双缓冲 DMA-SRAM 分配失败");
        return ESP_ERR_NO_MEM;
    }
    PRINT_MEM_INFO(TAG, "LVGL flush buffer 双缓冲 DMA-SRAM 分配后");

    /* 【2026-08-18】自适应 DARK 所需：注册刷新完成事件。
     * 必须在 lvgl_port_add_disp 成功之后（此时 lvgl_disp 已非 NULL，失败分支已 return）。
     * 与下方 UI_FLUSH_TRACE 的探针不同，本回调是功能性的、常驻注册，不受调试开关控制。
     * 回调体只置一个 bool，开销可忽略（见 ui_fade_refr_ready_cb）。 */
    lv_display_add_event_cb(lvgl_disp, ui_fade_refr_ready_cb, LV_EVENT_REFR_READY, NULL);

#if UI_FLUSH_TRACE
    /* 【临时调试】注册翻页 flush 追踪事件。必须在 lvgl_port_add_disp 成功之后，
     * 此时 lvgl_disp 已非 NULL（上面失败分支已 return）。
     * 五个事件共用一个回调，回调内按 code 分流。 */
    lv_display_add_event_cb(lvgl_disp, ui_ftrace_evt_cb, LV_EVENT_INVALIDATE_AREA, NULL);
    lv_display_add_event_cb(lvgl_disp, ui_ftrace_evt_cb, LV_EVENT_FLUSH_START, NULL);
    lv_display_add_event_cb(lvgl_disp, ui_ftrace_evt_cb, LV_EVENT_FLUSH_FINISH, NULL);
    lv_display_add_event_cb(lvgl_disp, ui_ftrace_evt_cb, LV_EVENT_REFR_START, NULL);
    lv_display_add_event_cb(lvgl_disp, ui_ftrace_evt_cb, LV_EVENT_REFR_READY, NULL);
    ESP_LOGW(TAG, "【临时调试】翻页 flush 追踪探针已启用（UI_FLUSH_TRACE=1），定位完请置 0");
#endif

#if UI_FPS_MONITOR
    lv_display_add_event_cb(lvgl_disp, ui_fps_evt_cb, LV_EVENT_FLUSH_FINISH, NULL);
    ESP_LOGW(TAG, "【临时调试】持续FPS统计探针已启用（UI_FPS_MONITOR=1），定位完请置 0");
#endif

    // 注意：esp_lvgl_port 在 double_buffer=false 时已自动注册为 PARTIAL 模式，
    // 这里不再调 lv_display_set_render_mode 覆盖（之前调用会与内部 flush 逻辑冲突）

    // 设置默认屏幕背景色为黑色（避免初始化期间花屏）
    if (lvgl_port_lock(1000))
    {
        lv_obj_t *screen = lv_screen_active();
        if (screen != NULL)
        {
            lv_obj_set_style_bg_color(screen, lv_color_black(), 0);
            lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
        }
        lvgl_port_unlock();
    }
    // 不再注册 RENDER_START/READY 回调：PARTIAL 模式下 RENDER_START 多次、READY 一次，
    // pause/resume 配对失衡使 GIF 永久 pause，在 PCBA 上加剧"GIF 卡第一帧"现象。
    return ESP_OK;
}
/* ═══════════════════════════════════════════════════════════════
 * 主时钟 UI
 * ═══════════════════════════════════════════════════════════════ */
#define CLOCK_DIGIT_W 40
#define CLOCK_COLON_W 22
#define CLOCK_H 60
#define CLOCK_Y 25

static void make_clock_cell(lv_obj_t *scr, lv_obj_t **out,
                            int32_t x, int32_t w, const char *init_text)
{
    *out = lv_label_create(scr);
    lv_obj_set_style_text_font(*out, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(*out, lv_color_white(), 0);
    lv_obj_set_style_text_align(*out, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(*out, LV_LABEL_LONG_CLIP);
    lv_obj_set_size(*out, w, CLOCK_H);
    lv_obj_set_pos(*out, x, CLOCK_Y);
    lv_label_set_text(*out, init_text);
}

static void main_desplay_create(void)
{
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL)
    {
        ESP_LOGE(TAG, "lv_screen_active 返回 NULL，跳过样式设置");
        return;
    }
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);
    lv_obj_set_scrollbar_mode(scr, LV_SCROLLBAR_MODE_OFF);
    lv_obj_set_style_text_font(scr, &font_cn_16, 0);
    ESP_LOGI(TAG, "主界面屏幕基础设置完成");
}

static void set_digit(lv_obj_t *lbl, uint8_t val)
{
    char buf[2] = {'0' + val, '\0'};
    lv_label_set_text(lbl, buf);
}

static void main_clock_refresh(void)
{
    if (s_clock_d[0] == NULL)
        return;

    if (reminder_is_time_synced())
    {
        uint8_t h, m, sec;
        reminder_get_current_time(&h, &m, &sec);
        set_digit(s_clock_d[0], h / 10);
        set_digit(s_clock_d[1], h % 10);
        set_digit(s_clock_d[2], m / 10);
        set_digit(s_clock_d[3], m % 10);
        set_digit(s_clock_d[4], sec / 10);
        set_digit(s_clock_d[5], sec % 10);

        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        static const char *const wday_cn[] = {
            "星期日", "星期一", "星期二", "星期三", "星期四", "星期五", "星期六"};
        char buf[32];
        snprintf(buf, sizeof(buf), "%d 月 %d 日  %s",
                 tm_now.tm_mon + 1, tm_now.tm_mday, wday_cn[tm_now.tm_wday & 0x7]);
        lv_label_set_text(s_time_date_lbl, buf);
    }
    else
    {
        for (int i = 0; i < 6; i++)
            lv_label_set_text(s_clock_d[i], "-");
        lv_label_set_text(s_time_date_lbl, "等待时间同步...");
    }
}

/**
 * @brief 每秒 tick：只在时间页/日历页可见时重绘对应页面
 *
 * 时间页只显示到分钟，所以画面每分钟才变一次；日历页需要它来跨零点换日
 * （大号日号、英文星期、点阵灰白分界都会变）。两页都不在时直接空转，
 * 避免无谓地抢 LVGL 锁。
 */
static void main_clock_tick_cb(lv_timer_t *t)
{
    (void)t;

    /* ⓪ 闸门：两种情况才需要刷新（2026-08-20 新增第二种）
     *   A. 功能层正停在时间页/日历页 —— 原有行为，一字未改。
     *   B. 低功耗常亮时钟正在显示 —— 此时 s_view 仍是 UI_VIEW_MAIN（待机时钟
     *      刻意不改视图，理由见 ui_standby_clock_show 上方注释），走不到 A 的
     *      判断，不放行的话时钟会永远停在【进入低功耗那一刻】的分钟数。
     *      待机时钟只显示时间页，故直接走 time_page_render 分支。 */
    bool standby_clock = s_standby_clock_on;

    /* ★【2026-08-25 新增：待机时钟与视图的一致性闸门】★
     * 【修的问题】上面 B 分支放行的前提是「待机时钟正在显示」，而它成立的隐含
     *   条件是 s_view 仍为 UI_VIEW_MAIN（ui_standby_clock_show 刻意不改视图）。
     *   但 ui_show_countdown_expired() / ui_show_alarm_ringing() 会在【低功耗
     *   期间】把 s_view 强切成 UI_VIEW_FUNCTION_MENU 去显示到期/响铃画面，
     *   却【不会】复位 s_standby_clock_on。于是本回调下一次跨分钟时仍走 B 分支
     *   无条件 time_page_render()，把时间页整组重新 unhide —— 直接【叠在】
     *   番茄钟页/闹钟页上面。实测现象即「倒计时页和时间页同框」。
     * 【改法】待机时钟态一旦发现视图已经不是主界面，说明有人把画面接管走了，
     *   本回调立刻让位，不再往上画。等唤醒流程把视图与标志都复位后自然恢复。
     * 【为何不在这里顺手清标志】清标志属于状态机复位，应由 ui_standby_clock_hide()
     *   统一负责（唤醒路径会调）。本回调只做渲染，不改别人的状态，避免两处抢着复位。 */
    if (standby_clock && s_view != UI_VIEW_MAIN)
        return;

    if (!standby_clock)
    {
        if (s_view != UI_VIEW_FUNCTION_MENU || s_menu_body == NULL)
            return;
        if (s_fn_page != FN_PAGE_TIME && s_fn_page != FN_PAGE_CALENDAR)
            return;
    }

    /* ① 渐变进行中一律不画（2026-08-18）
     *
     * 【解决什么】实测日志显示本 tick 的重绘会落在渐变的【渐亮】相位之后，
     * 即背光已经爬起来了才在画时间页，整屏刷新过程全被用户看见：
     *     [FD] IN 首帧 elapsed=19        ← 背光开始亮
     *     [FD]   time_page_render: ...   ← 亮着的时候在重绘  ★
     * 落在暗场还是亮场纯看运气——本 tick 是 1 秒固定节奏，与用户按下的时刻
     * 无关，这正是「偶发能看到刷屏」的来源。
     *
     * 【为什么可以直接跳过】画面只显示到分钟，晚一秒画毫无影响；且渐变结束后
     * 下一次 tick（≤1s）就会补上。转场期间该画的画面，暗态动作里已经画过了。 */
    if (s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
        return;

    /* ② 内容没变就不画（2026-08-18）
     *
     * 【原来的毛病】本回调每秒无条件重绘一次整个时间页/日历页，而两页显示的
     * 最小单位都是【分钟】——一分钟里有 59 次重绘画出来的东西跟上一次一模一样，
     * 纯属白刷。每次重绘都要抢 LVGL 锁 + 让 215px 大字重新光栅化 + 整屏 flush，
     * 是实打实的开销，也平白增加了「刷屏撞上转场」的机会。
     *
     * 【改法】记住上次画的是哪一分钟（用 时*60+分 编码），同一分钟内直接返回。
     * 跨分钟、跨天自然会变，日历页的日号/星期/点阵也都随之更新，不会漏刷。
     * 【为什么不用秒】两页都不显示秒，秒变了没有任何可见差别。
     * 【首次进页不受影响】进页是 render_fn_page 直接调 time_page_render，
     * 不走本 tick；本变量只影响「页面已经开着」时的周期刷新。 */
    static int s_last_drawn_minute = -1;
    uint8_t h = 0, m = 0, s = 0;
    reminder_get_current_time(&h, &m, &s);
    int cur_minute = (int)h * 60 + (int)m;
    if (cur_minute == s_last_drawn_minute)
        return;
    s_last_drawn_minute = cur_minute;

    if (lvgl_port_lock(10))
    {
#if CONFIG_UI_USE_CALENDAR
        /* 待机时钟态只可能是时间页，直接走 time_page_render，不看 s_fn_page
         * （它记的是上次在功能盘里停留的页，与待机时钟无关）。 */
        if (standby_clock || s_fn_page == FN_PAGE_TIME)
            time_page_render();
        else
            calendar_page_render();
#else
        time_page_render_text();
#endif
        lvgl_port_unlock();
    }
}

void ui_update_time(void)
{
    if (!lvgl_port_lock(100))
        return;
    main_clock_refresh();
    lvgl_port_unlock();
}

/* ═══════════════════════════════════════════════════════════════
 * 功能菜单框架
 * ═══════════════════════════════════════════════════════════════ */
static void menu_idle_timeout_cb(lv_timer_t *t)
{
    (void)t;
    s_menu_idle_tmr = NULL;
    /* 任意功能层（功能盘/功能页/闹钟编辑/游戏）空闲超时都回主界面 */
    if (s_view == UI_VIEW_MAIN)
        return;
    home_icon_fade_cancel(); /* 空闲超时离开功能层，撤销挂起的图标渐暗渐亮 */
    /* 【2026-08-18】本路径回主界面不经过 home_render，压扁提速的两个恢复点
     * （home_press_phase2_ready / home_render）都走不到，故在此补一次。
     * 漏掉的后果是定时器永久停在 16ms，主界面 GIF 全程双倍频率刷新。幂等。 */
    home_press_refr_boost(false);
    /* 闹钟编辑用的是独立面板 s_edit_panel（不是 s_menu_panel），
     * 超时回主界面时必须一并隐藏，否则编辑框会残留盖在 GIF 上。
     * 本路径不保存，等同于「放弃编辑」（2026-08-04：原先此视图直接 return 不超时）。
     *
     * ★ 必须留在这里、不能挪进渐变的暗态动作：暗态动作 ui_home_exit_apply()
     *   只认 s_menu_panel（长按退出走的是它），不处理 s_edit_panel；而且这两步
     *   （隐藏编辑面板 + 取消转场 timer）本身不产生可见刷屏，提前做不会被看见。 */
    if (s_view == UI_VIEW_ALARM_EDIT && s_edit_panel)
    {
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);
        /* 本路径绕开了 alarm_edit_exit，转场定时器需在此自行取消，
         * 否则面板已隐藏而定时器仍会到点触发一次无意义的 render。
         * 本函数即 LVGL 定时器回调，已在 LVGL 上下文中，不能再上锁。 */
        alarm_edit_zoom_cancel();
    }

    /* ★★【2026-09-01 修「功能盘超时退出是瞬间切换、没有渐变」】★★
     * 【现象】长按耳退出功能盘有「渐暗 → 暗态切页 → 渐亮」的完整过渡，
     *   但停留 20s 触发的空闲超时退出是硬切，两条出口手感不一致。
     * 【根因】本回调是 2026-08-31 渐变改造【之前】就写好的独立硬切实现：
     *   当时只把长按退出那条路（ui_func_layer_exit_to_main）改成了渐变，
     *   本函数下面那几行「隐藏面板 + unhide gif + s_view=MAIN」是同样动作的
     *   另一份拷贝，没人改到，于是一条路有渐变、另一条仍然硬切。
     * 【改法】删掉这份拷贝，改为调用同一套渐变退出，暗态动作即
     *   ui_home_exit_apply()（它已包含 games_stop / 隐藏 s_menu_panel /
     *   unhide gif_obj / s_view=MAIN / 恢复空闲轮播），故上面原有的
     *   「游戏视图先 games_stop」也一并由它接管，无需在此重复。
     * 【为什么不直接调 ui_func_layer_exit_to_main()】那个函数里有
     *   bsp_touch_request_vibrate()——震动的语义是「本次长按真的生效了」，
     *   而超时是设备自己退的，不该震；故走不带震动的内部启动器
     *   func_layer_exit_fade_start()（长按入口只是它 + 一次震动请求）。
     * 【线程】本函数是 lv_timer 回调，已在 LVGL 上下文；渐变启动器只做
     *   「置状态 + lv_timer_resume + lvgl_port_task_wake」，不上锁，安全。
     *   真正的切页在 ui_home_exit_apply() 里由 fade timer 回调发起，
     *   与长按退出走的是完全相同的路径。 */
    ESP_LOGI(TAG, "功能层空闲超时，渐变返回主界面");
    func_layer_exit_fade_start();
}

static void menu_kick_idle_timer(void)
{
    if (s_menu_idle_tmr != NULL)
    {
        lv_timer_reset(s_menu_idle_tmr);
        return;
    }
    s_menu_idle_tmr = lv_timer_create(menu_idle_timeout_cb, UI_MENU_IDLE_TIMEOUT_MS, NULL);
    lv_timer_set_repeat_count(s_menu_idle_tmr, 1);
}

static void menu_cancel_idle_timer(void)
{
    if (s_menu_idle_tmr != NULL)
    {
        lv_timer_del(s_menu_idle_tmr);
        s_menu_idle_tmr = NULL;
    }
}

static void ensure_menu_panel(void)
{
    if (s_menu_panel != NULL)
        return;
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL)
    {
        ESP_LOGE(TAG, "lv_screen_active 返回 NULL，跳过菜单面板创建");
        return;
    }

    /* 功能盘首次创建时预读图标进 PSRAM（幂等，只真正执行一次）。
     * 挂在这里而不是 ui_init：避开开机主路径，不给启动阶段再添 I/O；
     * 首次进功能盘走的是背光渐变流程（UI_HOME_FADE_OUT，600ms 渐暗），
     * 预读耗时被这段渐变吸收，用户无感。 */
    home_icon_cache_init();

    s_menu_panel = lv_obj_create(scr);
    lv_obj_set_size(s_menu_panel, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_align(s_menu_panel, LV_ALIGN_CENTER, 0, 0);
    lv_obj_set_style_bg_color(s_menu_panel, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(s_menu_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_menu_panel, 0, 0);
    lv_obj_set_style_pad_all(s_menu_panel, 8, 0);
    lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_SCROLLABLE);

    s_menu_title = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_color(s_menu_title, lv_color_hex(0xFFFFFF), 0);
    lv_obj_set_style_text_font(s_menu_title, &font_cn_16, 0);
    lv_obj_align(s_menu_title, LV_ALIGN_TOP_MID, 0, 4);

    s_menu_body = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_color(s_menu_body, lv_color_hex(0x88CCFF), 0);
    lv_obj_set_style_text_align(s_menu_body, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_long_mode(s_menu_body, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_menu_body, BSP_LCD_WIDTH - 24);
    lv_obj_align(s_menu_body, LV_ALIGN_CENTER, 0, 0);

    /* 功能盘大图标：屏幕正中（果冻动画作用对象），无文字。
     * 统一用 lv_image：有图的项设图片源；没图的项把图片源清空并用背景色+圆角
     * 当作占位色块（lv_image 也是 lv_obj，支持 bg/radius 样式）。具体在 home_render 切换。 */
    s_home_icon = lv_image_create(s_menu_panel);
    lv_obj_set_style_border_width(s_home_icon, 0, 0);
    lv_obj_align(s_home_icon, LV_ALIGN_CENTER, 0, 0);
    lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);

    lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    lv_obj_invalidate(scr);
}

/* ═══════════════════════════════════════════════════════════════
 * 闹钟重复模式转中文
 * 2026-08-04 UI 去掉「重复」展示后暂无调用者，保留备用（后续若恢复重复模式可直接用），
 * 加 unused 属性避免 -Wunused-function 告警。
 * ═══════════════════════════════════════════════════════════════ */
__attribute__((unused)) static const char *s_alarm_repeat_cn(alarm_repeat_t r)
{
    switch (r)
    {
    case ALARM_REPEAT_ONCE:
        return "一次";
    case ALARM_REPEAT_DAILY:
        return "每天";
    case ALARM_REPEAT_WEEKDAY:
        return "法定工作日";
    case ALARM_REPEAT_WEEKEND:
        return "周末";
    case ALARM_REPEAT_CUSTOM:
        return "自定义";
    default:
        return "";
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 闹钟展示页（单闹钟：直接显示时间、重复、开关状态）
 * ═══════════════════════════════════════════════════════════════ */
static void alarm_page_create(void)
{
    if (s_alarm_time_lbl)
        return;

    /* 大字号时间 */
    s_alarm_time_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_alarm_time_lbl, &font_alarm_59, 0);
    lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_white(), 0);
    lv_obj_align(s_alarm_time_lbl, LV_ALIGN_CENTER, 0, -25);

    /* 重复模式 */
    s_alarm_repeat_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_alarm_repeat_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_alarm_repeat_lbl, lv_color_hex(0x888888), 0);
    lv_obj_align(s_alarm_repeat_lbl, LV_ALIGN_CENTER, -30, 20);

    /* 开关状态 */
    s_alarm_status_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_alarm_status_lbl, &font_cn_16, 0);
    lv_obj_align(s_alarm_status_lbl, LV_ALIGN_CENTER, 30, 20);

    /* 操作提示 */
    s_alarm_hint_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_alarm_hint_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_alarm_hint_lbl, lv_color_hex(0x666666), 0);
    lv_obj_set_style_text_align(s_alarm_hint_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_alarm_hint_lbl, LV_ALIGN_BOTTOM_MID, 0, -8);
}

static void alarm_page_rebuild(void)
{
    if (!s_alarm_time_lbl)
        return;

    alarm_entry_t list[1];
    uint8_t count = 0;
    reminder_alarm_get_all(list, &count);

    /* 重复/状态两行已按需求去除：对象保留但恒隐藏，避免动到 create/hide 流程 */
    lv_obj_add_flag(s_alarm_repeat_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_alarm_status_lbl, LV_OBJ_FLAG_HIDDEN);

    /* 2026-08-10 新增：响铃态优先于一切，直接画到期画面并早退。
     * 与番茄时钟到期画面对齐 —— 红色大号时间 + 蓝色「闹钟响铃」提示。
     * 用一个显示标志而非给闹钟页新建状态机：本页原本就没有状态概念，
     * 加标志改动最小，且 s_alarm_ringing 只影响渲染，不碰任何闹钟数据。 */
    if (s_alarm_ringing)
    {
        char buf[16];
        if (count > 0)
            snprintf(buf, sizeof(buf), "%02d:%02d", list[0].hour, list[0].minute);
        else
            snprintf(buf, sizeof(buf), "--:--");
        lv_label_set_text(s_alarm_time_lbl, buf);
        lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_hex(0xFF3333), 0);

        /* 2026-08-14 需求：响铃时移除底部「长按后页键进入设置」提示文字。
         * 该 hint 标签在非响铃态本会写「长按后页键…」，响铃分支不碰它就会残留旧文案，
         * 与红色响铃画面不协调，故显式清空。 */
        lv_label_set_text(s_alarm_hint_lbl, "");
        return;
    }
    /* 非响铃态：hint 恢复常规灰色（响铃时被改成蓝色，必须还原） */
    lv_obj_set_style_text_color(s_alarm_hint_lbl, lv_color_hex(0x666666), 0);

    if (count == 0)
    {
        /* 未设置闹钟：明确的空状态，避免误导 */
        lv_label_set_text(s_alarm_time_lbl, "--:--");
        lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_hex(0x555555), 0);
        lv_label_set_text(s_alarm_hint_lbl, "长按后页键新建闹钟");
    }
    else
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d", list[0].hour, list[0].minute);
        lv_label_set_text(s_alarm_time_lbl, buf);
        lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_white(), 0);
        lv_label_set_text(s_alarm_hint_lbl, "长按后页键进入设置");
    }
}

static void alarm_page_show(void)
{
    alarm_page_create();
    /* 注意顺序：先 clear 再 rebuild。重复/状态两行的隐藏由 rebuild 负责，
     * 若反过来会被这里的 clear_flag 重新显示出来。 */
    lv_obj_clear_flag(s_alarm_time_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_alarm_hint_lbl, LV_OBJ_FLAG_HIDDEN);
    alarm_page_rebuild();
}

static void alarm_page_hide(void)
{
    if (s_alarm_time_lbl)
        lv_obj_add_flag(s_alarm_time_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_alarm_repeat_lbl)
        lv_obj_add_flag(s_alarm_repeat_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_alarm_status_lbl)
        lv_obj_add_flag(s_alarm_status_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_alarm_hint_lbl)
        lv_obj_add_flag(s_alarm_hint_lbl, LV_OBJ_FLAG_HIDDEN);

    /* 页面已隐藏，响铃提示定时器没有意义：留着它会在别的页面上改写显示态。
     * 同 countdown_page_hide() 的处理，一并清掉标志，下次进页面是干净的。 */
    if (s_alarm_ring_tmr != NULL)
    {
        lv_timer_del(s_alarm_ring_tmr);
        s_alarm_ring_tmr = NULL;
    }
    s_alarm_ringing = false;
}

/* 响铃提示停留结束：清除响铃态，回到常规闹钟页。
 * 由 LVGL 线程调用，已持锁，内部不得再 lvgl_port_lock。 */
static void alarm_ring_reset_cb(lv_timer_t *t)
{
    s_alarm_ring_tmr = NULL; /* 先摘句柄，防止其它路径重复 del */
    lv_timer_del(t);

    /* ★【2026-09-01 需求改造：响铃画面 → 闹钟编辑界面 也走渐变】★
     * 改前是硬切：s_edit_panel 是块全屏不透明面板，一 clear_flag 就整屏盖上来，
     * 亮屏下看得见逐条刷。改后藏进黑屏里。
     * quiet_main_screen=false：此刻已在功能页内，轮播早已 pause、舵机早已归中。
     *
     * ⚠️ s_alarm_ringing 的清零【挪进了暗态动作】：它同时控制渲染态与
     *   "响铃中任意触摸关闭闹钟"的触摸分支语义，若在这里提前清掉，渐暗那 450ms
     *   里画面还是响铃画面、触摸语义却已经变了，属于状态与画面不一致。 */
    if (expire_fade_start(alarm_reset_dark_action, /*quiet_main_screen=*/false,
                          "闹钟响铃→编辑界面"))
        return;

    alarm_reset_dark_action(); /* 渐变不可用：退化硬切，绝不把用户留在响铃画面上 */
}

/* 闹钟「响铃画面 → 主界面空闲 GIF」的暗态动作（2026-09-01 改版）。
 * 由 countdown_exit_fade_timer_cb 在 LVGL 线程调用（已持锁）；
 * 硬切兜底路径下由 alarm_ring_reset_cb 直接调用，同样在 LVGL 线程。 */
static void alarm_reset_dark_action(void)
{
    /* ★【2026-09-01 需求改版：响铃结束不再进编辑页，直接渐变回主界面空闲 GIF】★
     *
     * 【改前】alarm_edit_enter_ex(reset_to_zero=true) —— 进闹钟编辑页，且把编辑值
     *   强制显示成 00:00。用户反馈：响完不该把人丢进一个设置界面，应当回到空闲态。
     *
     * 【为什么可以整条删掉 reset_to_zero】那个参数只写 s_edit.hour/minute 两个
     *   【编辑态内存变量】，唯一目的是让紧接着要显示的编辑页显示 00:00。既然不再
     *   显示编辑页，归零就没有对象；而且下次从功能盘进编辑页走的是
     *   alarm_edit_enter(false)，那条路径会【无条件】用 reminder_alarm_get_all()
     *   的 NVS 值覆盖这两个变量（见 alarm_edit_enter_ex 的 count>0 分支），
     *   所以在这里留一个归零副作用既无用又容易误导。NVS 条目本来就没被动过
     *   （那条闹钟已被 reminder 按「只响一次」自动禁用），行为与手动退出一致。
     *
     * 【顺序要求】三步不可换序：
     *   ① alarm_page_hide()：清 s_alarm_ringing + 删响铃复位定时器 + 隐藏本页标签。
     *      不做的话响铃态标志会残留，下次 alarm_page_rebuild 会画出红色响铃样式。
     *   ② s_expire_fade_pending = false：必须在 ui_home_exit_apply() 之【前】清，
     *      因为它内部的 ui_resume_main_gif_loop() 要过 main_idle_loop_active()，
     *      而那道判据含 !s_expire_fade_pending —— 不先清就会「画面回到主界面但
     *      空闲轮播与舵机动作永远起不来」。
     *   ③ ui_home_exit_apply()：藏 s_menu_panel + 显 gif_obj + s_view=UI_VIEW_MAIN
     *      + 恢复空闲轮播（GIF 与该张对应的空闲舵机动作成对出现）。
     *
     * 【线程/锁】ui_home_exit_apply() 内部会 lvgl_port_lock(100)，而本回调运行在
     *   LVGL 线程（已持锁）。esp_lvgl_port 用的是 xSemaphoreTakeRecursive 递归锁，
     *   同一任务重复获取不会自锁，直接调用安全（同原先调 alarm_edit_enter_ex 的论证）。 */
    alarm_page_hide();
    s_expire_fade_pending = false;
    ui_home_exit_apply();
    ESP_LOGI(TAG, "闹钟响铃提示结束，已渐变回到主界面空闲 GIF");
}

/* ═══════════════════════════════════════════════════════════════
 * 天气展示页（2026-08-10 改版：左图标 + 右三行信息）
 *
 *  ┌──────────────┬──────────────────┐
 *  │              │      23°C        │ ← montserrat_48 + font_cn_16 的 °C
 *  │   ┌──────┐   │  ──────────────  │ ← 分隔线
 *  │   │ 128× │   │    体感 38°C     │ ← font_cn_16
 *  │   │ 128  │   │  ──────────────  │ ← 分隔线
 *  │   └──────┘   │    风级 3级      │ ← font_cn_16
 *  └──────────────┴──────────────────┘
 *
 * 【坐标基准】屏 320×240，但 s_menu_panel 有 pad_all=8，故子对象可用区域为
 * 304×224，且 LV_ALIGN_TOP_LEFT 的原点已在 (8,8)。下面所有坐标都是**相对
 * 内容区**的，已把用户要求的"左右各留 10、上下各留 20"减去 8 的内边距换算过：
 *   图标 x = 10-8 = 2，右栏 x = 170-8 = 162，上边距 y = 20-8 = 12。
 *
 * 【图标】128×128 RGB565A8，垂直居中于左半区（框 140 宽，图标 128，故 x 再 +6 居中）。
 * 【去除项】城市名、天气现象文字——按需求图标已表达天气，不再显示文字。
 * 数据全部来自和风 /v7/weather/now，无额外接口。
 * ═══════════════════════════════════════════════════════════════ */
/* 【2026-08-10 标题去除后重排】天气页顶部标题已隐藏，整版上移并撑满 224 高的内容区，
 * 使左图标与右信息栏都居中于整屏，贴合示例图的"左右两块等重"观感。 */
/* 【2026-08-11 改版】温度换 font_num_140（复用时间页已有字体，无需新增字模），
 * 右栏由 3 行改为 5 行：标签与数值各占一行（对齐示例图的"体感 / 38℃"两行写法）。
 *   温度96 → 线 → 体感(标签) → 体感值 → 线 → 风级(标签) → 风级值
 * 96 号字实测高约 96px，故温度行下移空间收紧，标签行用 font_cn_16（高约 16）。 */
#define WX_ICON_X 2  /* 图标左上角 x：贴左边缘（2026-08-12 左移让出空间，32px 字号信息栏文本变宽） */
#define WX_ICON_Y 48 /* 图标左上角 y：(224-128)/2 = 48，垂直居中 */
/* 2026-08-11 微调：右栏整体再向右移 15（162→177），分隔线相应缩短（140→120）。
 * 177+120=297，内容区宽 304，右侧留 7px 余量不越界。 */
/* 2026-08-11 微调：整体上移 12、右移到距屏幕右边缘 10px。
 * 内容区宽 304（屏 320 − s_menu_panel 的 pad_all 8×2），要求右边距 10px，
 * 换算到内容区坐标即右边缘 = 304 - (10 - 8) = 302。
 * 故 WX_RIGHT_X(202) + WX_RIGHT_W(100) = 302 正好贴齐。 */
#define WX_RIGHT_X 202              /* 右侧信息栏起始 x（温度/°C 专用，不要跟体感/风级共用，否则互相牵制） */
#define WX_RIGHT_W 100              /* 右侧信息栏宽度（= 分隔线长度，202+100=302） */
#define WX_INFO_X (WX_RIGHT_X - 45) /* 2026-08-12：体感/风级 4 个 label 单独用这个 x，比温度左移 50，再左移5（本次要求） */
#define WX_ROW1_Y 13                /* 温度 y（2026-08-13 下移5，8→13） */
#define WX_DEG_Y 15                 /* "°" y（2026-08-13 下移5，10→15） */
#define WX_C_Y 15                   /* "C" y（2026-08-13 下移5，10→15） */
#define WX_LINE1_Y 70               /* 分隔线 1 y */
/* 2026-08-12：标签+数值统一 font_cn_24（字高约24px），本次要求"分散一下"，间隔从6拉大到14 */
#define WX_ROW2_Y 80   /* "体感 Feels Like" 标签 y（2026-08-13 下移10，70→80） */
#define WX_ROW2V_Y 108 /* 体感数值 y（2026-08-13 下移10，98→108） */
#define WX_LINE2_Y 146 /* 分隔线 2 y（2026-08-13 下移10，136→146） */
#define WX_ROW3_Y 160  /* "风级 Wind" 标签 y（2026-08-13 下移10，150→160） */
#define WX_ROW3V_Y 198 /* 风级数值 y（2026-08-13 下移10，188→198） */
static void weather_page_create(void)
{
    if (s_wx_temp_lbl)
        return;

    /* 左侧天气图标（128×128，垂直居中）。
     * 初始不设 src：数据未就绪时由 rebuild 决定显示哪张或整个隐藏。 */
    s_wx_icon = lv_image_create(s_menu_panel);
    lv_obj_align(s_wx_icon, LV_ALIGN_TOP_LEFT, WX_ICON_X, WX_ICON_Y);
    lv_obj_add_flag(s_wx_icon, LV_OBJ_FLAG_HIDDEN);

    /* ⭐【必须保留，否则图标"看不见"】⭐
     * 和风官方 SVG 用 fill="currentColor"（靠 CSS 继承上色），单独转 PNG 时
     * 没有父元素可继承，渲染器一律落成**纯黑**——实测这 8 张图的颜色通道
     * 32768 字节全为 0，只有 alpha 通道有轮廓。黑图案画在黑底上完全看不见，
     * 这正是"图标不显示"的真因（对象/坐标/格式/体积均已排除）。
     * recolor 把不透明像素整体染成白色，线条形状由 alpha 通道决定，不受影响，
     * 观感与和风官网的黑线条图标一致（只是底黑线白，正负相反）。
     * 若日后换成本身带色的图标素材，把下面两行删掉即可恢复原色。 */
    lv_obj_set_style_image_recolor(s_wx_icon, lv_color_white(), 0);
    lv_obj_set_style_image_recolor_opa(s_wx_icon, LV_OPA_COVER, 0);

    /* 大号温度数字：字体在 rebuild 里用 wx_temp_font_get() 外挂加载，
     * create 阶段先用 montserrat_48 占位（避免外挂 Flash 未挂载时 lv_binfont_create 失败并永久回退） */
    s_wx_temp_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_temp_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_wx_temp_lbl, lv_color_white(), 0);
    lv_obj_align(s_wx_temp_lbl, LV_ALIGN_TOP_LEFT, WX_RIGHT_X, WX_ROW1_Y);

    /* "°C"（font_cn_16，贴大号数字右上；x 偏移在 rebuild 里按数字实宽动态对齐） */
    s_wx_deg_lbl = lv_label_create(s_menu_panel);
    /* 字体不在这里设：create 只跑一次，此刻外挂 Flash 未必已挂载，
     * lv_binfont_create 会失败并永久回退 16px。改到 rebuild 里每次设置。 */
    lv_obj_set_style_text_font(s_wx_deg_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_deg_lbl, lv_color_white(), 0);
    lv_label_set_text(s_wx_deg_lbl, "\xC2\xB0"); /* 仅 "°" */

    /* "C"（原 "°C" 的第二个字符拆出来，单独可调 y，用于修正 C 视觉偏低 */
    s_wx_c_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_c_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_c_lbl, lv_color_white(), 0);
    lv_label_set_text(s_wx_c_lbl, "C");

    // /* 分隔线 1（温度 / 体感之间）。用 1px 高的 lv_obj 画，比 lv_line 省事 */
    // s_wx_line1 = lv_obj_create(s_menu_panel);
    // lv_obj_set_size(s_wx_line1, WX_RIGHT_W, 1);
    // lv_obj_set_style_bg_color(s_wx_line1, lv_color_hex(0x666666), 0);
    // lv_obj_set_style_bg_opa(s_wx_line1, LV_OPA_COVER, 0);
    // lv_obj_set_style_border_width(s_wx_line1, 0, 0);
    // lv_obj_set_style_radius(s_wx_line1, 0, 0);
    // lv_obj_set_style_pad_all(s_wx_line1, 0, 0);
    // lv_obj_clear_flag(s_wx_line1, LV_OBJ_FLAG_SCROLLABLE);
    // lv_obj_align(s_wx_line1, LV_ALIGN_TOP_LEFT, WX_RIGHT_X, WX_LINE1_Y);

    /* "体感"标签行（灰字，与数值区分层次）。
     * 中英文拆成两个 label：一个 label 只能设一种字体，中文 16px / 英文 12px 必须分开。 */
    s_wx_feels_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_feels_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_feels_lbl, lv_color_white(), 0);
    lv_label_set_text(s_wx_feels_lbl, "体感"); /* 静态文案，rebuild 不再重设 */
    lv_obj_align(s_wx_feels_lbl, LV_ALIGN_TOP_LEFT, WX_INFO_X, WX_ROW2_Y);

    /* 英文 "Feels Like"（12px），贴中文右侧、垂直中线对齐。
     * 16px vs 12px 字号差带来的基线差，待烧录后按视觉效果微调 y 偏移。 */
    s_wx_feels_en = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_feels_en, font_cn_12_get(), 0);
    lv_obj_set_style_text_color(s_wx_feels_en, lv_color_white(), 0);
    lv_label_set_text(s_wx_feels_en, "Feels Like");
    lv_obj_align_to(s_wx_feels_en, s_wx_feels_lbl, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    /* 体感数值行（白字） */
    s_wx_feels_val = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_feels_val, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_feels_val, lv_color_white(), 0);
    lv_obj_align(s_wx_feels_val, LV_ALIGN_TOP_LEFT, WX_INFO_X, WX_ROW2V_Y);

    /* 分隔线 2（体感 / 风级之间） */
    s_wx_line2 = lv_obj_create(s_menu_panel);
    lv_obj_set_size(s_wx_line2, WX_RIGHT_W, 1);
    lv_obj_set_style_bg_color(s_wx_line2, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_wx_line2, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_wx_line2, 0, 0);
    lv_obj_set_style_radius(s_wx_line2, 0, 0);
    lv_obj_set_style_pad_all(s_wx_line2, 0, 0);
    lv_obj_clear_flag(s_wx_line2, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_wx_line2, LV_ALIGN_TOP_LEFT, WX_INFO_X, WX_LINE2_Y);

    /* "风级"标签行（灰字），中英文拆分同"体感" */
    s_wx_wind_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_wind_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_wind_lbl, lv_color_white(), 0);
    lv_label_set_text(s_wx_wind_lbl, "风级"); /* 静态文案 */
    lv_obj_align(s_wx_wind_lbl, LV_ALIGN_TOP_LEFT, WX_INFO_X, WX_ROW3_Y);

    /* 英文 "Wind"（12px），贴中文右侧、垂直中线对齐 */
    s_wx_wind_en = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_wind_en, font_cn_12_get(), 0);
    lv_obj_set_style_text_color(s_wx_wind_en, lv_color_white(), 0);
    lv_label_set_text(s_wx_wind_en, "Wind");
    lv_obj_align_to(s_wx_wind_en, s_wx_wind_lbl, LV_ALIGN_OUT_RIGHT_MID, 6, 0);

    /* 风级数值行（白字） */
    s_wx_wind_val = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_wind_val, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_wind_val, lv_color_white(), 0);
    lv_obj_align(s_wx_wind_val, LV_ALIGN_TOP_LEFT, WX_INFO_X, WX_ROW3V_Y);
}

static void weather_page_rebuild(void)
{
    if (!s_wx_temp_lbl)
        return;

    weather_data_t wd;
    reminder_get_weather_data(&wd);

    char buf[64];

    if (wd.valid)
    {
        /* ① 天气图标：按中文现象查表。未命中（如"沙尘暴"未做图）则隐藏，
         *    绝不显示错误图标误导用户——详见 weather_icons.h 的顺序说明 */
        const lv_image_dsc_t *icon = weather_icon_get(wd.text);
        /* [诊断 2026-08-10] 图标不显示排查：把查表输入、命中结果、对象指针、
         * 以及实际生效的坐标/尺寸全部打出来，定位后删除本段日志 */
        if (icon != NULL)
        {
            lv_image_set_src(s_wx_icon, icon);
            lv_obj_clear_flag(s_wx_icon, LV_OBJ_FLAG_HIDDEN);
        }
        else
        {
            lv_obj_add_flag(s_wx_icon, LV_OBJ_FLAG_HIDDEN);
        }

        /* ② 大号温度数字 + 紧贴右上的 "°C"
         * 【定位方向反过来】先把 °C 钉死在右边缘，再让温度数字贴到它左边。
         * 这样负号/三位数只会往**左**延伸，°C 的右边缘恒定贴齐 302，
         * 不会像"温度左对齐 + °C 跟在右边"那样把 °C 顶出屏幕。 */
        lv_label_set_text(s_wx_temp_lbl, wd.temp);

        /* 温度 + °C 用 wx_temp_font_get() 外挂加载（MiSans-Demibold 51px，含 "0123456789:°C"）。
         * 在 rebuild 里设置（非 create）：此刻外挂 Flash 已挂载，lv_binfont_create 才可能成功 */
        lv_obj_set_style_text_font(s_wx_temp_lbl, wx_temp_font_get(), 0);
        lv_obj_set_style_text_font(s_wx_deg_lbl, wx_temp_font_get(), 0);
        lv_obj_set_style_text_font(s_wx_c_lbl, wx_temp_font_get(), 0);

        /* 【写死像素，按"间隔"而非"右边界差"计算】
         * C 右边界钉死在内容区坐标 302（= 屏physical 320 右边距10px，见上方注释推导）。
         * "间隔"指两控件之间的空白：右邻控件左边界 - 左邻控件右边界 = 间隔。
         * 所以要先知道 C 的实宽才能算出 ° 的右边界，° 的实宽才能算出温度的右边界，
         * 层层向左推——不能直接拿右边界互相减，那样会把"间隔"和"控件宽度"叠在一起
         * 导致后一个控件的右边界跑到前一个控件的中间，出现重叠。 */
        lv_obj_update_layout(s_wx_deg_lbl);
        lv_obj_update_layout(s_wx_c_lbl);
        lv_obj_update_layout(s_wx_temp_lbl);
        lv_coord_t deg_w = lv_obj_get_width(s_wx_deg_lbl);
        lv_coord_t c_w = lv_obj_get_width(s_wx_c_lbl);
        lv_coord_t temp_w = lv_obj_get_width(s_wx_temp_lbl);

        lv_coord_t c_right = WX_RIGHT_X + WX_RIGHT_W; /* C 右边界 = 302（屏右10px） */
        lv_coord_t c_left = c_right - c_w;

        lv_coord_t deg_right = c_left - 3; /* ° 与 C 间隔 3px */
        lv_coord_t deg_left = deg_right - deg_w;

        lv_coord_t temp_right = deg_left - 5;       /* 温度 与 ° 间隔 5px */
        lv_coord_t temp_left = temp_right - temp_w; /* 温度数字实际左边界（受位数影响会变）*/

        lv_obj_align(s_wx_deg_lbl, LV_ALIGN_TOP_LEFT, deg_left, WX_DEG_Y);
        lv_obj_clear_flag(s_wx_deg_lbl, LV_OBJ_FLAG_HIDDEN);

        lv_obj_align(s_wx_c_lbl, LV_ALIGN_TOP_LEFT, c_left, WX_C_Y);
        lv_obj_clear_flag(s_wx_c_lbl, LV_OBJ_FLAG_HIDDEN);

        lv_obj_align(s_wx_temp_lbl, LV_ALIGN_TOP_LEFT, temp_left, WX_ROW1_Y);

        /* 2026-08-12：体感/风级改回 create 阶段写死的 WX_RIGHT_X，不再跟随温度左边界——
         * 32px 字号下"体感 Ti Gan"太宽，需要单独左移腾出空间，不能再与温度共用 x。
         * （之前这里用 lv_obj_set_x(temp_left) 每次 rebuild 都覆盖，导致 create 里的改动白改） */

        /* ③ 体感数值 / ④ 风级数值。
         * "体感""风级"两个标签是静态文案，已在 create 里设好，此处只更新数值行 */
        snprintf(buf, sizeof(buf), "%s\xC2\xB0"
                                   "C",
                 wd.feels);
        lv_label_set_text(s_wx_feels_val, buf);

        lv_label_set_text(s_wx_wind_val, wd.wind);
    }
    else
    {
        /* 断网且无 NVS 缓存：图标隐藏，温度占位，数值行留空（标签行保留） */
        lv_obj_add_flag(s_wx_icon, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_wx_temp_lbl, "--");
        lv_obj_add_flag(s_wx_deg_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_wx_c_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_wx_feels_val, "--");
        lv_label_set_text(s_wx_wind_val, "--");
    }
}

static void weather_page_show(void)
{
    weather_page_create();
    /* 进入页面触发一次拉取（异步，本次先用上次缓存渲染） */
    reminder_weather_fetch_now();
    weather_page_rebuild();
    lv_obj_clear_flag(s_wx_temp_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_feels_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_feels_val, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_wind_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_wind_val, LV_OBJ_FLAG_HIDDEN);
    /* 2026-08-13 修复：英文标签（Feels Like / Wind）此前漏了 show，切图回来时残留不显示 */
    lv_obj_clear_flag(s_wx_feels_en, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_wind_en, LV_OBJ_FLAG_HIDDEN);
    /* s_wx_line1（温度/体感分隔线）在 create 里已被注释掉不再创建，指针恒为 NULL，
     * 不能再 clear_flag，否则 assert 崩溃（本次实测复现） */
    lv_obj_clear_flag(s_wx_line2, LV_OBJ_FLAG_HIDDEN);
    /* s_wx_icon 与 s_wx_deg_lbl 的显隐由 rebuild 按数据有效性/查表结果决定，
     * 此处不能无条件 clear，否则数据无效时会露出上一次的残留图标 */
}

static void weather_page_hide(void)
{
    if (s_wx_icon)
        lv_obj_add_flag(s_wx_icon, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_temp_lbl)
        lv_obj_add_flag(s_wx_temp_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_deg_lbl)
        lv_obj_add_flag(s_wx_deg_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_c_lbl)
        lv_obj_add_flag(s_wx_c_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_feels_lbl)
        lv_obj_add_flag(s_wx_feels_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_feels_val)
        lv_obj_add_flag(s_wx_feels_val, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_wind_lbl)
        lv_obj_add_flag(s_wx_wind_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_wind_val)
        lv_obj_add_flag(s_wx_wind_val, LV_OBJ_FLAG_HIDDEN);
    /* 2026-08-13 修复：英文标签此前漏了 hide，切图离开天气页时残留不消失 */
    if (s_wx_feels_en)
        lv_obj_add_flag(s_wx_feels_en, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_wind_en)
        lv_obj_add_flag(s_wx_wind_en, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_line1)
        lv_obj_add_flag(s_wx_line1, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_line2)
        lv_obj_add_flag(s_wx_line2, LV_OBJ_FLAG_HIDDEN);
}

/* ═══════════════════════════════════════════════════════════════
 * 闹钟编辑界面（2步：时 → 分）
 *
 * 2026-08-10 版式改造：
 *   进入页面先按老样式在中央显示完整 "HH : MM"，1 秒后自动转场——
 *   完整时间整体下移到底部（保留橙色高亮当前位），屏幕中央换成
 *   放大版的单个值（当前编辑的小时或分钟）。左右两侧另加装饰箭头，
 *   提示用户可以触摸左右耳调节。交互逻辑完全不变：
 *     左耳短按=十位+1、右耳短按=个位+1、头部短按=时/分切换、头部长按=保存退出。
 * ═══════════════════════════════════════════════════════════════ */
#define ALARM_ACTIVE_COLOR 0xFFFFFF ///< 当前编辑位的高亮白（2026-08-14 由橙改白）
#define ALARM_ARROW_COLOR 0x555555  ///< 左右装饰箭头的暗灰，避免抢视觉重心
#define ALARM_ARROW_PAD_X 4         ///< 箭头距屏幕左/右边缘的内边距（px）
#define ALARM_ZOOM_DELAY_MS 1000    ///< 进入后多久自动开始转场动画（ms）
#define ALARM_ANIM_MS 300           ///< 转场/切换动画时长（ms）——缩小下滑与放大上移同时跑
#define ALARM_BIG_SCALE 384         ///< 中央大数字缩放：256=1.0×，384≈1.5×（48号字→约72号观感）
#define ALARM_BIG_Y (-10)           ///< 中央大数字相对屏幕中心的 Y 偏移（px，负=偏上）
#define ALARM_CENTER_Y (-20)        ///< 未转场时完整时间相对屏幕中心的 Y 偏移（px，原版式）
#define ALARM_CENTER_DX 55          ///< 未转场时时/分相对中线的水平间距（px，原版式）
#define ALARM_BOTTOM_Y (-14)        ///< 转场后完整时间距屏幕底部的偏移（px）
#define ALARM_BOTTOM_SCALE 128      ///< 转场后完整时间的缩放：128=0.5×（48号字→约24号观感）
/* 转场后时/分相对中线的水平间距（px）。
 * ⚠️ 不能沿用居中版式的 ALARM_CENTER_DX：transform_scale 是渲染期变换，
 * 只改画出来的大小，不改对象的布局尺寸与对齐基准。若间距仍按原值排，
 * 字被画成一半大而间距没缩，就会出现「17 和 :56 疏密不均、冒号贴着分钟」。
 * 故间距必须与缩放同比缩小：CENTER_DX × BOTTOM_SCALE / 256。 */
#define ALARM_BOTTOM_DX (ALARM_CENTER_DX * ALARM_BOTTOM_SCALE / LV_SCALE_NONE)
/* 底部合并成单个居中的 "HH:MM" 后，小时/分钟各自中心相对中线的偏移。
 * 整串 5 个字符（含冒号）在 48 号字下宽约 5×27=135px，缩到 0.5× 约 68px，
 * 时/分中心各在 ∓1/4 串宽 ≈ ∓17px 处。该值仅决定大数字飞入/飞出的起点，
 * 略有出入只影响动画起手位置，不影响静态版式。 */
#define ALARM_BOTTOM_HALF_DX 17
#define ALARM_HINT_Y 30      ///< 「X小时X分钟后响铃」距屏幕顶部的 Y（px，在标题下方）
#define ALARM_ANIM_STEPS 256 ///< 动画进度归一化分母（0~256 → 0.0~1.0，避免浮点）

static void alarm_edit_create(void)
{
    if (s_edit_panel != NULL)
        return;

    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL)
    {
        ESP_LOGE(TAG, "lv_screen_active 返回 NULL，跳过编辑面板创建");
        return;
    }

    s_edit_panel = lv_obj_create(scr);
    lv_obj_set_size(s_edit_panel, BSP_LCD_WIDTH, BSP_LCD_HEIGHT);
    lv_obj_set_style_bg_color(s_edit_panel, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_edit_panel, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_edit_panel, 0, 0);
    lv_obj_set_style_pad_all(s_edit_panel, 0, 0);
    lv_obj_clear_flag(s_edit_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);

    /* 【2026-08-14 新增】两块灰色圆角面板（nz.png 示例：#171717，各 138×168，中间断开）
     * 高上下各加 5（158→168），左右各向外扩 5（133→138） */
    s_edit_gray_l = lv_obj_create(s_edit_panel);
    lv_obj_set_size(s_edit_gray_l, 145, 168);
    lv_obj_set_style_bg_color(s_edit_gray_l, lv_color_hex(0x171717), 0);
    lv_obj_set_style_bg_opa(s_edit_gray_l, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_edit_gray_l, 0, 0);
    lv_obj_set_style_radius(s_edit_gray_l, 10, 0);
    lv_obj_set_style_pad_all(s_edit_gray_l, 0, 0);
    lv_obj_clear_flag(s_edit_gray_l, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_edit_gray_l, LV_ALIGN_TOP_LEFT, 12, 36);

    s_edit_gray_r = lv_obj_create(s_edit_panel);
    lv_obj_set_size(s_edit_gray_r, 145, 168);
    lv_obj_set_style_bg_color(s_edit_gray_r, lv_color_hex(0x171717), 0);
    lv_obj_set_style_bg_opa(s_edit_gray_r, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_edit_gray_r, 0, 0);
    lv_obj_set_style_radius(s_edit_gray_r, 10, 0);
    lv_obj_set_style_pad_all(s_edit_gray_r, 0, 0);
    lv_obj_clear_flag(s_edit_gray_r, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_align(s_edit_gray_r, LV_ALIGN_TOP_LEFT, 165, 36);

    /* 【2026-08-14 新增】HH/MM 的拦腰横切分割线：背景色线，厚 5、横跨两块灰块。
     * 定位在灰块垂直中部（拦腰）再上移 2，用与背景同色的线遮住数字中段，形成
     * 「数字被拦腰斩断」的翻页时钟错觉（数字上下两半之间露出一道背景色缝隙）。 */
    s_edit_sep_line = lv_obj_create(s_edit_panel);
    lv_obj_set_size(s_edit_sep_line, 298, 5);
    lv_obj_set_style_bg_color(s_edit_sep_line, lv_color_black(), 0);
    lv_obj_set_style_bg_opa(s_edit_sep_line, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(s_edit_sep_line, 0, 0);
    lv_obj_set_style_radius(s_edit_sep_line, 0, 0);
    lv_obj_align(s_edit_sep_line, LV_ALIGN_LEFT_MID, 12, -4);

    /* 【2026-08-14 注释】标题 "设置闹钟"：示例图 nz.png 无标题
    lv_obj_t *title = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &font_cn_16, 0);
    lv_label_set_text(title, "设置闹钟");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);
    */

    /* 小时 "HH"：拆成十位/个位两个独立标签，各自锚定在左灰块的隐形中线（灰块宽 145，
     * 中线在灰块内 x=72.5 处）两侧——
     *   十位：LV_ALIGN_RIGHT_MID，dx = -(145/2) 把右边缘从灰块右缘拉回贴中线；
     *   个位：LV_ALIGN_LEFT_MID， dx = +(145/2) 把左边缘从灰块左缘推到贴中线。
     * 这样调整某一位时只有那一个 label 的宽度变化、按自己固定的那条边收缩/展开，
     * 另一位因为锚点没变，绝对位置纹丝不动。 */
#define ALARM_GRAY_HALF_W 72                          // 灰块半宽（145/2 取整），十位/个位锚点与中线的偏移量
    s_edit_hour_lbl = lv_label_create(s_edit_gray_l); /* 十位 */
    lv_obj_set_style_text_font(s_edit_hour_lbl, font_alarm_118_get(), 0);
    lv_obj_set_style_text_color(s_edit_hour_lbl, lv_color_hex(0x999999), 0); /* 非高亮=暗淡白 */
    lv_label_set_recolor(s_edit_hour_lbl, true);
    lv_obj_align(s_edit_hour_lbl, LV_ALIGN_RIGHT_MID, -ALARM_GRAY_HALF_W, 0); /* 2026-08-16：右边缘贴左灰块隐形中线 */

    s_edit_hour_ones_lbl = lv_label_create(s_edit_gray_l); /* 个位 */
    lv_obj_set_style_text_font(s_edit_hour_ones_lbl, font_alarm_118_get(), 0);
    lv_obj_set_style_text_color(s_edit_hour_ones_lbl, lv_color_hex(0x999999), 0);
    lv_label_set_recolor(s_edit_hour_ones_lbl, true);
    lv_obj_align(s_edit_hour_ones_lbl, LV_ALIGN_LEFT_MID, ALARM_GRAY_HALF_W, 0); /* 2026-08-16：左边缘贴左灰块隐形中线 */

    /* 分钟 "MM"：同理拆成十位/个位，各自锚定右灰块的隐形中线两侧 */
    s_edit_min_lbl = lv_label_create(s_edit_gray_r); /* 十位 */
    lv_obj_set_style_text_font(s_edit_min_lbl, font_alarm_118_get(), 0);
    lv_obj_set_style_text_color(s_edit_min_lbl, lv_color_hex(0x999999), 0); /* 非高亮=暗淡白 */
    lv_label_set_recolor(s_edit_min_lbl, true);
    lv_obj_align(s_edit_min_lbl, LV_ALIGN_RIGHT_MID, -ALARM_GRAY_HALF_W, 0); /* 2026-08-16：右边缘贴右灰块隐形中线 */

    s_edit_min_ones_lbl = lv_label_create(s_edit_gray_r); /* 个位 */
    lv_obj_set_style_text_font(s_edit_min_ones_lbl, font_alarm_118_get(), 0);
    lv_obj_set_style_text_color(s_edit_min_ones_lbl, lv_color_hex(0x999999), 0);
    lv_label_set_recolor(s_edit_min_ones_lbl, true);
    lv_obj_align(s_edit_min_ones_lbl, LV_ALIGN_LEFT_MID, ALARM_GRAY_HALF_W, 0); /* 2026-08-16：左边缘贴右灰块隐形中线 */

    // /* 【2026-08-14 新增】PM 指示：左灰块内、贴 HH 左下方（nz.png x=32~46,y=167~178） */
    // s_edit_pm_lbl = lv_label_create(s_edit_gray_l);
    // lv_obj_set_style_text_font(s_edit_pm_lbl, font_cn_16_get(), 0);
    // lv_obj_set_style_text_color(s_edit_pm_lbl, lv_color_white(), 0);
    // lv_label_set_text(s_edit_pm_lbl, "PM");
    // lv_obj_align(s_edit_pm_lbl, LV_ALIGN_TOP_LEFT, 10, 146);

    /* 【2026-08-14 注释】冒号 ":" 标签（占位，示例图无冒号）
    s_edit_colon_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_colon_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_colon_lbl, lv_color_white(), 0);
    lv_label_set_text(s_edit_colon_lbl, ":");
    lv_obj_align(s_edit_colon_lbl, LV_ALIGN_CENTER, 0, -20);
    */
    /* 【2026-08-14 注释】分钟标签（占位，示例图无）
    s_edit_min_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_min_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_min_lbl, lv_color_white(), 0);
    lv_obj_align(s_edit_min_lbl, LV_ALIGN_CENTER, 55, -20);
    */

    /* 【2026-08-14 注释】重复模式标签（示例图无）
    s_edit_repeat_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_repeat_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_edit_repeat_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_edit_repeat_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_edit_repeat_lbl, LV_ALIGN_CENTER, 0, 30);
    */
    /* 【2026-08-14 注释】开关状态标签（示例图无）
    s_edit_enable_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_enable_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_edit_enable_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_edit_enable_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_edit_enable_lbl, LV_ALIGN_CENTER, 0, 55);
    */
    /* 【2026-08-14 注释】左右装饰箭头 < >（示例图无）
    s_edit_arrow_l = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_arrow_l, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_arrow_l, lv_color_hex(ALARM_ARROW_COLOR), 0);
    lv_label_set_text(s_edit_arrow_l, "<");
    lv_obj_clear_flag(s_edit_arrow_l, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(s_edit_arrow_l, LV_ALIGN_LEFT_MID, ALARM_ARROW_PAD_X, 0);

    s_edit_arrow_r = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_arrow_r, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_arrow_r, lv_color_hex(ALARM_ARROW_COLOR), 0);
    lv_label_set_text(s_edit_arrow_r, ">");
    lv_obj_clear_flag(s_edit_arrow_r, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_align(s_edit_arrow_r, LV_ALIGN_RIGHT_MID, -ALARM_ARROW_PAD_X, 0);
    */
    /* 【2026-08-14 注释】「X小时X分钟后响铃」提示（示例图无）
    s_edit_hint_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_hint_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_edit_hint_lbl, lv_color_hex(0xAAAAAA), 0);
    lv_obj_set_style_text_align(s_edit_hint_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_label_set_text(s_edit_hint_lbl, "");
    lv_obj_align(s_edit_hint_lbl, LV_ALIGN_TOP_MID, 0, ALARM_HINT_Y);
    */
    /* 【2026-08-14 注释】中央大号单值 ×2（切换动画载体，示例图无，切换动画一并注释）
    for (int i = 0; i < 2; i++)
    {
        s_edit_big_lbl[i] = lv_label_create(s_edit_panel);
        lv_obj_set_style_text_font(s_edit_big_lbl[i], &lv_font_montserrat_48, 0);
        lv_obj_set_style_text_color(s_edit_big_lbl[i], lv_color_hex(ALARM_ACTIVE_COLOR), 0);
        lv_obj_set_style_transform_pivot_x(s_edit_big_lbl[i], LV_PCT(50), 0);
        lv_obj_set_style_transform_pivot_y(s_edit_big_lbl[i], LV_PCT(50), 0);
        lv_obj_add_flag(s_edit_big_lbl[i], LV_OBJ_FLAG_HIDDEN);
    }
    */
}

/**
 * @brief 刷新「X小时X分钟后响铃」提示
 *
 * 按「当前编辑值 vs 此刻系统时间」算到下一次响铃还有多久：
 * 目标时刻若已过今天这个点，则顺延到明天（与 reminder 只看时分、每天循环的语义一致）。
 * 时间未同步（sntp 未就绪，年份仍停在 1970）时不显示，避免给出误导性的倒计时。
 * 调用方需持有 LVGL 锁。
 */
static void alarm_edit_update_hint(void)
{
    if (!s_edit_hint_lbl)
        return;

    time_t now = time(NULL);
    struct tm now_tm;
    localtime_r(&now, &now_tm);

    /* 年份 < 2020 视为时间未同步：此时算出来的倒计时没有意义 */
    if (now_tm.tm_year + 1900 < 2020)
    {
        lv_label_set_text(s_edit_hint_lbl, "");
        return;
    }

    int now_min = now_tm.tm_hour * 60 + now_tm.tm_min;
    int set_min = s_edit.hour * 60 + s_edit.minute;
    int diff = set_min - now_min;
    if (diff <= 0)
        diff += 24 * 60; /* 已过今天这个点 → 顺延到明天 */

    char buf[48];
    if (diff >= 60)
        snprintf(buf, sizeof(buf), "%d小时%d分钟后响铃", diff / 60, diff % 60);
    else
        snprintf(buf, sizeof(buf), "%d分钟后响铃", diff);
    lv_label_set_text(s_edit_hint_lbl, buf);
}

#if 0  /* ══ 2026-08-16 旧逻辑（合并单标签版式 + 中央大数字 + 放大动画）停用，与两灰框布局解耦 ══ */
/**
 * @brief 按当前是否已转场，摆放「完整时间」三个标签与「中央大数字」
 *
 * 两种版式：
 *   未转场（进入后 1 秒内）：完整 HH:MM 居中大字，中央大数字隐藏 —— 即老版式；
 *   已转场：完整 HH:MM 缩小并整体移到底部，中央显示放大的当前编辑值。
 *
 * 只负责位置/缩放/可见性，颜色高亮仍由 alarm_edit_render 统一处理。
 * 调用方需持有 LVGL 锁。
 */
/**
 * @brief 把完整 HH:MM 三个标签按给定缩放摆到指定 Y / 间距上
 *
 * 进场动画每帧要用插值后的中间值调它，静态版式与切换动画则传终点值，
 * 统一走这一个函数，避免「有的路径摆了、有的路径漏摆」导致标签停在
 * 上一段动画的残留位置上（切换时底部时间跟着乱飞就是这么来的）。
 *
 * @param scale 缩放（256=1.0×）
 * @param dx    时/分相对中线的水平间距（px）
 * @param y     相对屏幕中心的 Y 偏移（px）
 */
static void alarm_trio_place(int32_t scale, lv_coord_t dx, lv_coord_t y)
{
    if (!s_edit_hour_lbl || !s_edit_colon_lbl || !s_edit_min_lbl)
        return;

    /* ⭐ 三个标签合并成一个标签来摆位。
     *
     * 之前给时/冒号/分各算一个 dx 手动摆开，怎么调都对不齐，根因是
     * transform_scale 属于渲染期变换：只改画出来的大小，不改对象的布局宽度。
     * 「按原始 48 号字宽算出的间距」与「实际画成一半大的字形」永远对不上，
     * 表现就是 17 和 : 之间空一大块、: 和 56 却贴死。
     *
     * 现在改为把完整时间画进 s_edit_hour_lbl 一个标签里（文本形如 "17:56"），
     * 字形间距由字体自身度量决定，缩放时整体等比，永远不会散架；
     * 冒号/分钟两个标签退居幕后只作占位（隐藏），保留对象以免动到
     * create/闪烁/销毁等既有流程。高亮改用 LVGL 的 recolor 富文本实现。 */
    lv_obj_set_style_transform_pivot_x(s_edit_hour_lbl, LV_PCT(50), 0);
    lv_obj_set_style_transform_pivot_y(s_edit_hour_lbl, LV_PCT(50), 0);
    lv_obj_set_style_transform_scale(s_edit_hour_lbl, scale, 0);
    lv_obj_align(s_edit_hour_lbl, LV_ALIGN_CENTER, 0, y);

    lv_obj_add_flag(s_edit_colon_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_add_flag(s_edit_min_lbl, LV_OBJ_FLAG_HIDDEN);
    (void)dx; /* 合并成单标签后不再需要手算间距 */
}

/* 底部那一行在「以屏幕中心为原点」坐标系里的 Y（供动画插值与静态摆放共用） */
static inline lv_coord_t alarm_bottom_center_y(void)
{
    return BSP_LCD_HEIGHT / 2 + ALARM_BOTTOM_Y;
}

static void alarm_edit_apply_layout(void)
{
    if (!s_edit_hour_lbl || !s_edit_colon_lbl || !s_edit_min_lbl)
        return;

    /* ⚠️ 统一用 LV_ALIGN_CENTER 一种对齐方式：动画每帧插值出的也是「相对中心」的 Y，
     * 若静态版式改用 LV_ALIGN_BOTTOM_MID，两套坐标系的原点不同，
     * 动画结束那一帧会突然跳位（表现为底部时间瞬移/重叠）。 */
    if (s_edit_zoomed)
        alarm_trio_place(ALARM_BOTTOM_SCALE, ALARM_BOTTOM_DX, alarm_bottom_center_y());
    else
    {
        alarm_trio_place(LV_SCALE_NONE, ALARM_CENTER_DX, ALARM_CENTER_Y);
        for (int i = 0; i < 2; i++)
            if (s_edit_big_lbl[i])
                lv_obj_add_flag(s_edit_big_lbl[i], LV_OBJ_FLAG_HIDDEN);
    }
}

/* ── 转场/切换动画 ──────────────────────────────────────────────
 *
 * 两段动画共用一个 0~ALARM_ANIM_STEPS 的进度值，在同一个 exec_cb 里
 * 同时插值「缩放」和「Y 位置」，保证一进一出两个元素严格同步——
 * 若拆成两条 lv_anim 各管一个属性，二者的时间基准可能错开半帧，
 * 视觉上会出现「先缩小后位移」的割裂感。
 *
 * 进场动画（进入 1 秒后触发）：
 *   完整 HH:MM   1.0× 居中  →  0.5× 贴底
 *   中央大数字   0.5× 底部  →  1.5× 居中（复制自底部对应那一位）
 *
 * 切换动画（头部短按）：
 *   旧值 1.5× 居中 → 0.5× 飞回底部对应位置后隐藏
 *   新值 0.5× 底部 → 1.5× 居中
 *   底部完整 HH:MM 全程不动，只有橙色高亮换位。
 */

/* 取当前编辑字段在底部那行里的 X 偏移（相对中线），作为大数字飞入/飞出的起终点。
 * 底部已合并成单个居中的 "HH:MM"，小时占左半、分钟占右半，
 * 故各自中心约在 ∓(整串宽度/4) 处，用 ALARM_BOTTOM_HALF_DX 表示。 */
static lv_coord_t alarm_big_home_dx(alarm_edit_state_t state)
{
    return (state == ALARM_EDIT_HOUR) ? -ALARM_BOTTOM_HALF_DX : ALARM_BOTTOM_HALF_DX;
}

/* 把某个大数字标签按进度 p（0~ALARM_ANIM_STEPS）摆放在「底部老家 → 中央」之间。
 * p=0 完全贴合底部对应位（0.5×），p=满 完全到达中央（1.5×）。
 * dir_in=true 表示飞入（0→满），false 表示飞出（满→0）——两者只是 p 的走向不同，
 * 摆放公式完全一致，故共用本函数。 */
static void alarm_big_place(lv_obj_t *lbl, lv_coord_t home_dx, int32_t p)
{
    if (!lbl)
        return;

    /* 线性插值：scale 从 BOTTOM_SCALE 到 BIG_SCALE，位置从底部到中央 */
    const int32_t scale =
        ALARM_BOTTOM_SCALE + (ALARM_BIG_SCALE - ALARM_BOTTOM_SCALE) * p / ALARM_ANIM_STEPS;

    const lv_coord_t bottom_y = alarm_bottom_center_y();
    const lv_coord_t y = bottom_y + (ALARM_BIG_Y - bottom_y) * p / ALARM_ANIM_STEPS;
    const lv_coord_t x = home_dx + (0 - home_dx) * p / ALARM_ANIM_STEPS;

    lv_obj_set_style_transform_scale(lbl, scale, 0);
    lv_obj_align(lbl, LV_ALIGN_CENTER, x, y);
}
#endif /* ══ 旧逻辑结束 ══ */

/* 【2026-08-14 注释】放大缩小进场动画（示例图 nz.png 无进场转场）：
 * alarm_zoom_anim_cb / alarm_zoom_anim_ready / alarm_zoom_tmr_cb 三个一起注释。
 * 注意：alarm_edit_enter 里起转场定时器的那两行也一并注释了。后面要恢复时一起打开。
 *
 * 进场动画 exec_cb：完整时间缩小下滑 + 大数字放大上移，同帧完成
static void alarm_zoom_anim_cb(void *var, int32_t p)
{
    (void)var;
    if (!s_edit_hour_lbl || !s_edit_colon_lbl || !s_edit_min_lbl)
        return;
    const int32_t scale =
        LV_SCALE_NONE + (ALARM_BOTTOM_SCALE - LV_SCALE_NONE) * p / ALARM_ANIM_STEPS;
    const lv_coord_t dx =
        ALARM_CENTER_DX + (ALARM_BOTTOM_DX - ALARM_CENTER_DX) * p / ALARM_ANIM_STEPS;
    const lv_coord_t bottom_y = alarm_bottom_center_y();
    const lv_coord_t y = ALARM_CENTER_Y + (bottom_y - ALARM_CENTER_Y) * p / ALARM_ANIM_STEPS;
    alarm_trio_place(scale, dx, y);
    alarm_big_place(s_edit_big_lbl[s_edit_big_cur], alarm_big_home_dx(s_edit.state), p);
}

static void alarm_zoom_anim_ready(lv_anim_t *a)
{
    (void)a;
    s_edit_zoomed = true;
    alarm_edit_apply_layout();
    alarm_big_place(s_edit_big_lbl[s_edit_big_cur],
                    alarm_big_home_dx(s_edit.state), ALARM_ANIM_STEPS);
}

static void alarm_zoom_tmr_cb(lv_timer_t *t)
{
    s_edit_zoom_tmr = NULL;
    lv_timer_del(t);
    if (!s_edit_panel || !s_edit_big_lbl[s_edit_big_cur])
        return;
    char buf[8];
    snprintf(buf, sizeof(buf), "%02d",
             (s_edit.state == ALARM_EDIT_HOUR) ? s_edit.hour : s_edit.minute);
    lv_label_set_text(s_edit_big_lbl[s_edit_big_cur], buf);
    alarm_big_place(s_edit_big_lbl[s_edit_big_cur], alarm_big_home_dx(s_edit.state), 0);
    lv_obj_clear_flag(s_edit_big_lbl[s_edit_big_cur], LV_OBJ_FLAG_HIDDEN);
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, s_edit_panel);
    lv_anim_set_exec_cb(&a, alarm_zoom_anim_cb);
    lv_anim_set_values(&a, 0, ALARM_ANIM_STEPS);
    lv_anim_set_time(&a, ALARM_ANIM_MS);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_set_ready_cb(&a, alarm_zoom_anim_ready);
    lv_anim_start(&a);
}
*/

static void alarm_edit_render(void) // 显示界面
{
    if (!s_edit_panel)
        return;

    /* char buf[8]; */ /* 2026-08-16：仅供中央大数字分支（已 #if 0）使用 */

    /* 完整时间合并进 s_edit_hour_lbl 一个标签（见 alarm_trio_place 的说明），
     * 当前编辑位用 LVGL 的 recolor 富文本标记 #RRGGBB ...# 上橙色，
     * 另一位保持标签自身的白色。这样高亮效果与拆成两个标签时一致，
     * 但字形间距交给字体度量，缩放时整体等比、不会散架。 */
    /* 2026-08-10：冒号两侧各加一个空格。
     * 原文本形如 "15:29"，冒号紧贴时/分两侧，底部那行缩到 0.5×（ALARM_BOTTOM_SCALE）
     * 后字形挤在一起，视觉上像重叠；且 recolor 语法 "#RRGGBB 值#" 的**前导空格会被
     * LVGL 吃掉当分隔符**，而闭合的 "#" 不带空格，于是高亮位与冒号之间更紧一分，
     * 两侧疏密还不对称。统一加空格后间距均匀，放大/缩小都不再粘连。 */
    char timebuf[24];
    /* 2026-08-16：HH/MM 各自拆成十位/个位两个标签分别写值，不再拼一个两位数字符串。
     * 高亮（当前编辑字段）仍用 recolor 富文本，但现在是每一位单独判断着色，
     * 而不是整串一起标橙——十位/个位各自的文本长度也不会再互相影响。 */
    if (s_edit.state == ALARM_EDIT_HOUR)
    {
        snprintf(timebuf, sizeof(timebuf), "#%06X %d#", ALARM_ACTIVE_COLOR, s_edit.hour / 10);
        lv_label_set_text(s_edit_hour_lbl, timebuf);
        snprintf(timebuf, sizeof(timebuf), "#%06X %d#", ALARM_ACTIVE_COLOR, s_edit.hour % 10);
        lv_label_set_text(s_edit_hour_ones_lbl, timebuf);
    }
    else
    {
        snprintf(timebuf, sizeof(timebuf), "%d", s_edit.hour / 10);
        lv_label_set_text(s_edit_hour_lbl, timebuf);
        snprintf(timebuf, sizeof(timebuf), "%d", s_edit.hour % 10);
        lv_label_set_text(s_edit_hour_ones_lbl, timebuf);
    }

    if (s_edit.state == ALARM_EDIT_MINUTE)
    {
        snprintf(timebuf, sizeof(timebuf), "#%06X %d#", ALARM_ACTIVE_COLOR, s_edit.minute / 10);
        lv_label_set_text(s_edit_min_lbl, timebuf);
        snprintf(timebuf, sizeof(timebuf), "#%06X %d#", ALARM_ACTIVE_COLOR, s_edit.minute % 10);
        lv_label_set_text(s_edit_min_ones_lbl, timebuf);
    }
    else
    {
        snprintf(timebuf, sizeof(timebuf), "%d", s_edit.minute / 10);
        lv_label_set_text(s_edit_min_lbl, timebuf);
        snprintf(timebuf, sizeof(timebuf), "%d", s_edit.minute % 10);
        lv_label_set_text(s_edit_min_ones_lbl, timebuf);
    }

#if 0 /* 2026-08-16：中央大数字分支（s_edit_zoomed 恒 false + big_lbl 已注释为 NULL），随旧逻辑一并停用 */
    /* 中央大数字：跟随当前编辑字段实时刷新（左右耳每次 +1 都会走到这里，
     * 底部完整时间用的是同一份 s_edit.hour/minute，两者天然同步）。
     * 注意只在「已转场且不在切换动画中」时改文本——切换动画期间两个标签
     * 各自承载旧值/新值，此处贸然改写会把飞出的旧值也改成新值，动画就穿帮了。 */
    if (s_edit_zoomed && !s_edit_switching && s_edit_big_lbl[s_edit_big_cur])
    {
        snprintf(buf, sizeof(buf), "%02d",
                 (s_edit.state == ALARM_EDIT_HOUR) ? s_edit.hour : s_edit.minute);
        lv_label_set_text(s_edit_big_lbl[s_edit_big_cur], buf);
        /* 改文本会改变标签宽度，需重新居中。这里安全的前提是本分支已排除
         * 「切换动画进行中」（s_edit_switching），且进场动画期间 s_edit_zoomed
         * 尚为 false 也进不来——两条动画都不会与这次摆放打架。 */
        alarm_big_place(s_edit_big_lbl[s_edit_big_cur],
                        alarm_big_home_dx(s_edit.state), ALARM_ANIM_STEPS);
    }
#endif

    /* 重复/状态两行已按需求去除：对象保留但整行隐藏，避免动到 create/销毁流程 */
    if (s_edit_repeat_lbl)
        lv_obj_add_flag(s_edit_repeat_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_edit_enable_lbl)
        lv_obj_add_flag(s_edit_enable_lbl, LV_OBJ_FLAG_HIDDEN);

    /* 2026-08-14：非高亮=暗淡白，高亮=白（recolor 富文本）。四个位标签都设暗淡白基色。 */
    lv_obj_set_style_text_color(s_edit_hour_lbl, lv_color_hex(0x999999), 0);
    lv_obj_set_style_text_color(s_edit_hour_ones_lbl, lv_color_hex(0x999999), 0);
    lv_obj_set_style_text_color(s_edit_min_lbl, lv_color_hex(0x999999), 0);
    lv_obj_set_style_text_color(s_edit_min_ones_lbl, lv_color_hex(0x999999), 0);

    alarm_edit_update_hint(); /* 值变了 → 倒计时提示同步刷新 */

    /* 动画期间不要用静态版式覆盖插值结果，否则每帧都会被拽回终点位置 */
#if 0 /* 2026-08-16：apply_layout 属旧版式逻辑，已停用（数字改由 create 里 LV_ALIGN_CENTER 静态居中） */
    if (!s_edit_switching)
        alarm_edit_apply_layout();
#endif
}

/* 取消转场定时器 + 停掉进行中的转场/切换动画（幂等）。
 * 调用方需持有 LVGL 锁或处于 LVGL 上下文中。
 * 动画必须一并停：它每帧都在写 s_edit_* 标签，若面板已隐藏甚至退出编辑，
 * 残留动画会继续摆弄这些对象（ready_cb 还会改 s_edit_zoomed 等状态）。 */
static void alarm_edit_zoom_cancel(void)
{
    if (s_edit_zoom_tmr != NULL)
    {
        lv_timer_del(s_edit_zoom_tmr);
        s_edit_zoom_tmr = NULL;
    }
    if (s_edit_panel)
        lv_anim_del(s_edit_panel, NULL); /* 两条动画的 var 都是 s_edit_panel */
    s_edit_switching = false;
}

/**
 * @brief 进入闹钟编辑页
 *
 * @param reset_to_zero true=编辑值强制从 00:00 起（响铃结束后的入口，见
 *                      alarm_ring_reset_cb）；false=沿用 NVS 里已存的闹钟值
 *                      （功能盘正常入口，方便用户在原值上微调）。
 *
 * 2026-08-16 需求：①无已存闹钟时的默认值由 08:00 改为 00:00；
 *                  ②闹钟响完自动进编辑页时，编辑值重置为 00:00（只重置编辑值，
 *                    不动 NVS 条目——那条已被 reminder 按「只响一次」自动禁用）。
 */
static void alarm_edit_enter_ex(bool reset_to_zero)
{
    /* 【临时诊断】闹钟编辑不走 render_fn_page（app_enter_alarm 直接调本函数），
     * 故单独打一对探针，否则会漏掉这一页的 17 个对象 + f118.bin 字体懒加载。 */
    ui_mem_probe("before", "闹钟编辑");

    alarm_entry_t list[1];
    uint8_t count = 0;
    reminder_alarm_get_all(list, &count);

    if (reset_to_zero)
    {
        /* 响铃结束入口：不管 NVS 里存的是几点，编辑值都从 00:00 起 */
        s_edit.hour = 0;
        s_edit.minute = 0;
    }
    else if (count > 0)
    {
        s_edit.hour = list[0].hour;
        s_edit.minute = list[0].minute;
    }
    else
    {
        /* 2026-08-16：默认值 08:00 → 00:00 */
        s_edit.hour = 0;
        s_edit.minute = 0;
    }
    /* 重复/开关不再由用户编辑：固定「只响一次 + 保存即开启」（2026-08-04 需求） */
    s_edit.repeat = ALARM_REPEAT_ONCE;
    s_edit.enabled = true;

    s_edit.state = ALARM_EDIT_HOUR;

    /* 每次进入都从「完整时间居中」的老版式起步，1 秒后再转场（见 alarm_zoom_tmr_cb）。
     * 面板对象是复用的（create 里已 return 早退），故这两个状态必须在此显式复位，
     * 否则第二次进入会直接停在上次退出时的大数字版式。 */
    s_edit_zoomed = false;
    s_edit_switching = false;
    s_edit_big_cur = 0;
    /* 2026-08-16：退出渐变状态必须显式复位——若上次退出被异常打断残留非 NONE，
     * 本次长按保存时 alarm_edit_blink_cancel 会误判为「正常收尾」而放行，
     * 退出流程就再也走不通（同倒计时那处自我打断 bug 的教训）。 */
    s_alarm_exit_phase = ALARM_EXIT_PHASE_NONE;
    s_alarm_exit_elapsed = 0;

    if (lvgl_port_lock(100))
    {
        alarm_edit_create(); /* 创建LVGL对象必须在锁内，防止与LVGL定时器任务竞争 */
        /* 复用面板时可能残留上次的动画与显示状态，进入前一并清干净 */
        lv_anim_del(s_edit_panel, NULL);
        for (int i = 0; i < 2; i++)
            if (s_edit_big_lbl[i])
                lv_obj_add_flag(s_edit_big_lbl[i], LV_OBJ_FLAG_HIDDEN);
        alarm_edit_render();
        lv_obj_clear_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (s_menu_panel)
            lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);

        /* 【2026-08-14 注释】进场转场定时器（放大缩小动画已注释，不再起）
        s_edit_zoom_tmr = lv_timer_create(alarm_zoom_tmr_cb, ALARM_ZOOM_DELAY_MS, NULL);
        if (s_edit_zoom_tmr != NULL)
            lv_timer_set_repeat_count(s_edit_zoom_tmr, 1);
        */
        alarm_edit_zoom_cancel(); /* 防御：清残留动画（放大缩小已无，保留以防后续恢复） */
        lvgl_port_unlock();
    }
    s_view = UI_VIEW_ALARM_EDIT;
    /* 2026-08-04：原为 menu_cancel_idle_timer()（进编辑就永不超时），现改为与功能盘一致——
     * 30s 无触摸自动退回主界面（不保存）。任意触摸会在事件分发入口刷新计时器。 */
    menu_kick_idle_timer();
    ESP_LOGI(TAG, "进入闹钟编辑");

    /* 【临时诊断】与函数开头的 before 配对，差值 = 闹钟编辑页净占用 */
    ui_mem_probe("after", "闹钟编辑");
}

/* 常规入口（功能盘选闹钟 / 闹钟页长按进编辑）：沿用 NVS 里已存的值，便于微调。
 * 保持原签名，现有调用点无需改动。 */
static void alarm_edit_enter(void)
{
    alarm_edit_enter_ex(/*reset_to_zero=*/false);
}

/* ── 闹钟保存确认动效：整屏背光快速渐亮渐灭 3 次 ──
 *
 * 需求（2026-08-14）：长按头部 = 保存 + 退出 + 快速渐亮渐灭 3 次。
 * （2026-08-31：原先额外的「震动一次」已去除，触摸层按下沿已震过，避免两次。）
 * 实现要点：
 *   1) 由 2026-08-06 的「硬闪烁」改为「平滑渐变」，2026-08-16 又从「标签透明度」
 *      改为「整屏背光」——改 opa 时屏幕背光始终亮着，数字 opa=0 那一刻屏幕仍是
 *      亮的，切回功能盘时功能盘图标立刻以全不透明显现，没有任何遮挡，用户能
 *      看到刷新过程。改成压背光后 opa=0 对应物理黑屏，切页动作彻底藏住；
 *   2) 渐变不能用 vTaskDelay 阻塞（会卡住 LVGL 线程，参见 BUG-038），
 *      改用一个 lv_timer 每 ALARM_FADE_STEP_MS 推进一档亮度；
 *   3) 定时器句柄 s_alarm_fade_tmr 全程可被 alarm_edit_exit 取消，
 *      防止「动效未完就被其它入口退出」留下野指针（同 games 结算定时器的坑）。
 */
/* 2026-08-16：纯三角波（灭到底/亮到顶立刻反向）中间没有停顿，切换观感太急促。
 * 改为梯形波：渐灭 → 停在全暗一小段 → 渐亮 → 停在全亮一小段 → 下一周期。
 * 单周期 = 渐灭 + 暗停顿 + 渐亮 + 亮停顿。 */
#define ALARM_FADE_RAMP_MS 250        // 单段渐灭/渐亮耗时（ms）
#define ALARM_FADE_HOLD_DARK_MS 100   // 灭到底后的停顿时长（ms），期间背光恒为 0
#define ALARM_FADE_HOLD_BRIGHT_MS 100 // 亮到顶后的停顿时长（ms），期间背光恒为默认亮度
#define ALARM_FADE_CYCLE_MS \
    (ALARM_FADE_RAMP_MS * 2 + ALARM_FADE_HOLD_DARK_MS + ALARM_FADE_HOLD_BRIGHT_MS) // 单周期总时长
#define ALARM_FADE_CYCLES 3                                                        // 渐亮渐灭次数
#define ALARM_FADE_STEP_MS 20                                                      // 亮度推进步进（20ms 一档，够平滑又不刷太勤）

static lv_timer_t *s_alarm_fade_tmr = NULL; ///< 保存确认渐变动效定时器（NULL=未在渐变）
static uint32_t s_alarm_fade_elapsed = 0;   ///< 已推进的毫秒数（0=动效未开始）

/* 退出阶段状态 alarm_exit_phase_t / s_alarm_exit_phase / s_alarm_exit_elapsed
 * 声明在文件前部的闹钟编辑 UI 对象区（alarm_edit_enter 里要复位它，那里更早）。 */

/* 设置整屏背光亮度（0~255 归一化输入 → 0~BSP_LCD_BK_DEFAULT_PCT 亮度百分比）。
 * 2026-08-16：由原先改 4 个时间标签的 opa 改为压整屏背光——只有物理压暗背光才能
 * 在「灭到底」那一刻形成真正的黑屏，把随后的切页动作藏住（改 opa 屏幕仍亮着）。
 * 参数名保留 opa 语义（0=全灭、255=全亮），便于沿用原有的梯形波计算。 */
static void alarm_edit_time_set_opa(uint8_t opa)
{
    uint8_t pct = (uint8_t)(((uint32_t)opa * BSP_LCD_BK_DEFAULT_PCT) / 255);
    bsp_board_lcd_set_brightness(pct);
}

/**
 * @brief 取消渐变动效并恢复全亮显示（幂等）
 *
 * @return true=取消时动效确实还在跑（意味着这次保存尚未落地，调用方需自行补保存）
 *
 * 调用方需持有 LVGL 锁，或处于 LVGL 自身上下文中。
 * 返回值存在的原因：保存动作在动效的最后一步才执行，若中途被强行取消，
 * 用户已经确认过的保存就会被静默丢掉——由调用方决定补保存还是当作放弃。
 */
static bool alarm_edit_blink_cancel(void)
{
    /* ⚠️ 已进入退出阶段时【直接放行】：此时 alarm_edit_exit → 本函数 是退出流程
     * 自己的调用链（见 alarm_fade_tmr_cb 的退出分支），若在此 del 定时器并把背光
     * 拉回全亮，会把还要驱动「暗态等待 + 渐亮」的定时器杀掉、并立刻亮屏暴露切页，
     * 与倒计时那处「渐变自我打断」是同一类坑。返回 false 表示「不是被打断」，
     * 调用方无需补保存（保存已在进入退出阶段前由 alarm_edit_exit(true) 完成）。 */
    if (s_alarm_exit_phase != ALARM_EXIT_PHASE_NONE)
        return false;

    bool was_running = (s_alarm_fade_tmr != NULL);
    if (s_alarm_fade_tmr != NULL)
    {
        lv_timer_del(s_alarm_fade_tmr);
        s_alarm_fade_tmr = NULL;
    }
    s_alarm_fade_elapsed = 0;
    alarm_edit_time_set_opa(LV_OPA_COVER);
    return was_running;
}

/* 强制中止闹钟退出阶段并恢复背光（幂等）。
 * 供「待机强制回主界面」这类外部入口调用：此时 alarm_edit_blink_cancel 会因
 * 处于退出阶段而放行，若不额外清理，定时器会继续在主界面上推进背光、
 * 或把背光永久留在暗态。调用方需持有 LVGL 锁或处于 LVGL 上下文。 */
static void alarm_exit_fade_abort(void)
{
    if (s_alarm_exit_phase == ALARM_EXIT_PHASE_NONE)
        return;
    s_alarm_exit_phase = ALARM_EXIT_PHASE_NONE;
    s_alarm_exit_elapsed = 0;
    if (s_alarm_fade_tmr != NULL)
    {
        lv_timer_del(s_alarm_fade_tmr);
        s_alarm_fade_tmr = NULL;
    }
    s_alarm_fade_elapsed = 0;
    bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT); /* 背光可能停在暗态，拉回正常 */
}

/* 【2026-08-16 需求】退出时机卡在第 3 次「渐灭」结束的那一刻：此时背光已压到 0
 * （物理黑屏），趁黑切回功能盘闹钟图标，再等一小段让 LVGL flush + 液晶响应走完，
 * 最后渐亮呈现。避免像之前那样在亮屏状态下切图、被用户看到刷屏。
 *
 * ⚠️ 切页【必须】留出「暗态停留」窗口再渐亮：切页只是把 LVGL 对象改了，真正推到
 * 屏幕要等下一次 flush + 液晶响应。若切完立刻渐亮，新画面那一帧会在背光已经爬升
 * 时才落地，用户就看到「从暗到亮过程中画面在变」——这正是改背光后仍需三段式的原因，
 * 与 ui_home_fade_timer_cb 的 UI_HOME_FADE_DARK 阶段同一个道理。
 *
 * 总时长只算 (CYCLES-1) 个完整周期 + 最后一次的「渐灭」，随后进入退出三段式。 */
#define ALARM_FADE_TOTAL_MS \
    ((uint32_t)ALARM_FADE_CYCLE_MS * (ALARM_FADE_CYCLES - 1) + ALARM_FADE_RAMP_MS)
#define ALARM_EXIT_DARK_MS 200    // 切页后暗态停留：等 flush + 等液晶响应（背光恒 0）
#define ALARM_EXIT_FADE_IN_MS 500 // 切页后渐亮耗时
/* 退出阶段状态 alarm_exit_phase_t / s_alarm_exit_phase / s_alarm_exit_elapsed
 * 已提前声明在 alarm_edit_blink_cancel 之前（后者要靠它区分「打断」与「正常收尾」）。 */

/* 渐变动效定时器回调：由 LVGL 线程调用，已持锁，内部不得再 lvgl_port_lock */
static void alarm_fade_tmr_cb(lv_timer_t *t)
{
    /* ── 退出三段式：切页已在进入本阶段时完成，这里只负责暗态等待 + 渐亮 ── */
    if (s_alarm_exit_phase != ALARM_EXIT_PHASE_NONE)
    {
        s_alarm_exit_elapsed += ALARM_FADE_STEP_MS;
        if (s_alarm_exit_phase == ALARM_EXIT_PHASE_DARK)
        {
            /* 等 flush + 液晶响应走完，避免功能盘那帧在背光爬升时才落地 */
            if (s_alarm_exit_elapsed >= ALARM_EXIT_DARK_MS)
            {
                s_alarm_exit_phase = ALARM_EXIT_PHASE_IN;
                s_alarm_exit_elapsed = 0;
            }
            return;
        }
        /* ALARM_EXIT_PHASE_IN：线性渐亮到默认亮度，到顶即收尾自毁 */
        if (s_alarm_exit_elapsed >= ALARM_EXIT_FADE_IN_MS)
        {
            bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT);
            s_alarm_exit_phase = ALARM_EXIT_PHASE_NONE;
            s_alarm_exit_elapsed = 0;
            s_alarm_fade_tmr = NULL; /* 先摘句柄，防止 exit 路径重复 del */
            lv_timer_del(t);
            return;
        }
        uint8_t pct = (uint8_t)(((uint32_t)s_alarm_exit_elapsed * BSP_LCD_BK_DEFAULT_PCT) /
                                ALARM_EXIT_FADE_IN_MS);
        bsp_board_lcd_set_brightness(pct);
        return;
    }

    s_alarm_fade_elapsed += ALARM_FADE_STEP_MS;

    if (s_alarm_fade_elapsed >= ALARM_FADE_TOTAL_MS)
    {
        /* 闪够 3 次且第 3 次已渐灭到底（背光=0，物理黑屏）：
         * 趁黑保存退出切回功能盘，然后转入退出三段式的暗态停留阶段。
         * 注意此处【不】del 定时器——它还要继续驱动「暗态等待 + 渐亮」。
         *
         * ⚠️ 顺序关键：必须【先】置 s_alarm_exit_phase 再调 alarm_edit_exit()。
         * alarm_edit_exit → alarm_edit_blink_cancel 靠这个标志判断「是正常收尾
         * 还是被外部打断」；若先调 exit，此刻标志还是 NONE，cancel 会当成打断
         * 处理：del 掉本定时器 + 把背光拉回全亮 —— 实测表现就是「灭到底那一刻
         * 背光瞬间亮满、切页暴露在亮屏上」，正是要避免的现象。 */
        bsp_board_lcd_set_brightness(0); /* 确保严格归零，不受整数取整误差影响 */
        s_alarm_exit_phase = ALARM_EXIT_PHASE_DARK;
        s_alarm_exit_elapsed = 0;
        alarm_edit_exit(true);
        return;
    }

    /* 梯形波背光：每周期 渐灭(全亮→全灭) → 暗停顿 → 渐亮(全灭→全亮) → 亮停顿。
     * 两段停顿消除「灭到底/亮到顶立刻反弹」的仓促感。最后一次周期提前在
     * 渐灭结束时被上面的总时长判断截断，转入退出三段式。 */
    uint32_t pos = s_alarm_fade_elapsed % (uint32_t)ALARM_FADE_CYCLE_MS; // 周期内位置
    uint32_t dark_end = ALARM_FADE_RAMP_MS + ALARM_FADE_HOLD_DARK_MS;
    uint32_t bright_ramp_end = dark_end + ALARM_FADE_RAMP_MS;
    uint8_t opa;
    if (pos < ALARM_FADE_RAMP_MS)
        opa = (uint8_t)(255 - (pos * 255) / ALARM_FADE_RAMP_MS); /* 渐灭：全亮→全灭 */
    else if (pos < dark_end)
        opa = 0; /* 暗停顿：恒暗 */
    else if (pos < bright_ramp_end)
        opa = (uint8_t)(((pos - dark_end) * 255) / ALARM_FADE_RAMP_MS); /* 渐亮：全灭→全亮 */
    else
        opa = 255; /* 亮停顿：恒亮 */
    alarm_edit_time_set_opa(opa);
}

/**
 * @brief 头部长按：保存闹钟并退出，附带「整体时间快速渐亮渐灭 3 次」反馈
 *
 * 渐变走定时器，走完才 alarm_edit_exit(true)。
 * 【2026-08-31】原本这里还会额外震一次，已删（触摸层按下沿已震过，避免两次）。
 */
static void alarm_edit_confirm_with_feedback(void)
{
    if (s_alarm_fade_tmr != NULL)
        return; /* 已在渐变中，忽略重复长按 */

    /* ★【2026-08-31 问题6】原有的 bsp_motor_pulse() 已删除：会震两次。
     * 头部长按的按下沿在触摸层（bsp_touch.c，闹钟编辑属非主界面）已经震过一次，
     * 抬手确认时这里再震就成了第二次。保留触摸层那一次，反馈仍在。
     *
     * ★【2026-08-31 追加：确定改为「动作真正生效才震」】触摸层已把闹钟编辑页整块
     *   排除（head_vib_view 不含 UI_VIEW_ALARM_EDIT），本页头部长按已无来自触摸层的
     *   按下沿震动，改由此处发起。上面那道重入检查已通过，说明本次长按确实被受理、
     *   马上要起闪烁动效并保存；被 return 掉的重复长按不震。
     *   只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞（BUG-038）。 */
    bsp_touch_request_vibrate();

    if (lvgl_port_lock(100))
    {
        s_alarm_fade_elapsed = 0;
        s_alarm_fade_tmr = lv_timer_create(alarm_fade_tmr_cb, ALARM_FADE_STEP_MS, NULL);
        lvgl_port_unlock();
    }

    /* 定时器创建失败（内存不足等）兜底：直接保存退出，不因动效丢功能 */
    if (s_alarm_fade_tmr == NULL)
        alarm_edit_exit(true);
}

/**
 * @brief 把当前编辑值真正写入 reminder（不碰 UI）
 *
 * 从 alarm_edit_exit 里抽出来单独成函数：闪烁动效把"保存"推迟到了动画末尾，
 * 若中途被强制打断（如待机强制回主界面），需要有个不牵扯 UI 的入口补上这次保存。
 */
static void alarm_edit_commit(void)
{
    /* repeat/enabled 固定写死：只响一次、保存即开启（2026-08-04 需求） */
    alarm_entry_t entry = {
        .hour = s_edit.hour,
        .minute = s_edit.minute,
        .repeat = ALARM_REPEAT_ONCE,
        .enabled = true,
    };
    memset(entry.message, 0, sizeof(entry.message));

    alarm_entry_t list[1];
    uint8_t count = 0;
    reminder_alarm_get_all(list, &count);
    if (count > 0)
        reminder_alarm_update(0, &entry);
    else
        reminder_alarm_add(&entry);

    ESP_LOGI(TAG, "闹钟已保存: %02d:%02d（只响一次）", s_edit.hour, s_edit.minute);
}

static void alarm_edit_exit(bool save)
{
    /* 无论从哪个入口退出，都要先清掉可能仍在跑的闪烁定时器，避免野指针。
     * 注意：这里不必因"闪烁被打断"补保存——本函数自己就带 save 参数，
     * 走到这儿要么是闪烁正常播完（save=true 由回调传入），要么是用户主动取消。 */
    if (lvgl_port_lock(100))
    {
        alarm_edit_blink_cancel();
        alarm_edit_zoom_cancel(); /* 同理：转场定时器也必须随退出一并清掉 */
        lvgl_port_unlock();
    }

    if (save)
    {
        alarm_edit_commit();
    }
    else
    {
        ESP_LOGI(TAG, "闹钟编辑已取消");
    }

    /* 闹钟编辑直接从功能盘进入（去掉了只读显示页），退出后回到功能盘。 */
    if (lvgl_port_lock(100))
    {
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (s_menu_panel)
            lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
        s_view = UI_VIEW_HOME;
        s_home_animating = false;
        if (s_home_icon)
            lv_obj_set_style_translate_x(s_home_icon, 0, 0);
        home_render();
        lvgl_port_unlock();
    }
    menu_kick_idle_timer();
}

/**
 * @brief 小时分位 +1（2026-08-06 起小时与分钟统一为「左十右个」）
 *
 * @param hour   当前小时 0~23
 * @param is_tens true=十位 +1（0~2 循环）；false=个位 +1（0~9 循环）
 * @return 调整后的合法小时（0~23）
 *
 * 与分钟的差别：分钟两位各自独立循环（十位 0~5、个位 0~9）后必然合法，
 * 而小时十位到 2 时个位只能是 0~3，单纯本位循环会产生 24~29 这类非法值。
 * 处理方式是「本位继续 +1 直到合法」——效果上就是把非法档跳过去，
 * 例如十位=2 时个位 3→(4~9 跳过)→0，十位循环时 25→(跳过)→05。
 * 最多循环 10 次必然命中合法值（每位的合法集非空），不会死循环。
 */
static uint8_t alarm_hour_bump_digit(uint8_t hour, bool is_tens)
{
    uint8_t tens = hour / 10;
    uint8_t ones = hour % 10;

    for (int i = 0; i < 10; i++) /* 上限 10 次：本位最多绕一整圈 */
    {
        if (is_tens)
            tens = (tens + 1) % 3; /* 小时十位仅 0/1/2 */
        else
            ones = (ones + 1) % 10;

        uint8_t v = (uint8_t)(tens * 10 + ones);
        if (v <= 23)
            return v; /* 命中合法值即返回 */
    }
    return hour; /* 理论不可达，兜底不改值 */
}

/**
 * @brief 右耳短按：小时【个位】+1 / 分钟【个位】+1
 *
 * 2026-08-04 分钟改为「左十右个」分位调节（与倒计时页 countdown_bump_digit 同一手感）：
 *   个位 9→0 只在本位回绕，不向十位进位，两位相当于两个独立拨盘。
 * 2026-08-06 小时也统一为同一逻辑（见 alarm_hour_bump_digit），
 *   非法值 24~29 由 helper 内部跳过，用户感知不到。
 */
static void alarm_edit_value_next(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        /* 2026-08-06：小时改为与分钟同样的「左十右个」分位逻辑。
         * 个位 +1（本位循环 0~9），十位不动；越界（如 24~29）时跳过到合法值。 */
        s_edit.hour = alarm_hour_bump_digit(s_edit.hour, /*is_tens=*/false);
        break;
    case ALARM_EDIT_MINUTE:
        /* 个位 +1（本位循环）：分钟 0~59，个位满 9 回 0，十位不动 */
        s_edit.minute = (s_edit.minute / 10) * 10 + (s_edit.minute % 10 + 1) % 10;
        break;
    }
    /* ★【2026-08-31】震动跟随「数字真的变了」而非「按下」：触摸层已把闹钟编辑页
     * 的左右耳排除（bsp_touch.c 的 page_vib_view），改由此处发起。执行到这里
     * s_edit 的值已经被改写，震动与新数字上屏同一拍，不再是"先震后变"。
     * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞（BUG-038）。 */
    bsp_touch_request_vibrate();
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

/**
 * @brief 左耳短按：小时【十位】+1 / 分钟【十位】+1
 *
 * 2026-08-06 起时/分手感完全一致：左耳=十位、右耳=个位，均本位循环。
 */
static void alarm_edit_value_prev(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        /* 2026-08-06：小时十位 +1（本位循环 0~2），个位不动；越界时跳过到合法值。 */
        s_edit.hour = alarm_hour_bump_digit(s_edit.hour, /*is_tens=*/true);
        break;
    case ALARM_EDIT_MINUTE:
        /* 十位 +1（本位循环）：分钟十位 0~5，满 5 回 0，个位不动 */
        s_edit.minute = ((s_edit.minute / 10 + 1) % 6) * 10 + s_edit.minute % 10;
        break;
    }
    bsp_touch_request_vibrate(); /* 同右耳：数字变完才震，见 alarm_edit_value_next 说明 */
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

/* ── 时/分切换动画 ──
 * 旧值从中央缩小飞回它在底部的老家，新值同时从它的老家放大飞到中央。
 * 两者共用同一个进度值：新值用 p，旧值用 (满-p)，天然反向且严格同步。 */
/* 【2026-08-14 注释】切换动画（大数字飞出飞入）：示例图 nz.png 无大数字，触摸头部切换只改状态+高亮，
 * 不再飞大数字。alarm_switch_anim_cb / alarm_switch_anim_ready 两个一起注释。
 *
static void alarm_switch_anim_cb(void *var, int32_t p)
{
    (void)var;
    lv_obj_t *incoming = s_edit_big_lbl[s_edit_big_cur];
    lv_obj_t *outgoing = s_edit_big_lbl[1 - s_edit_big_cur];
    alarm_trio_place(ALARM_BOTTOM_SCALE, ALARM_BOTTOM_DX, alarm_bottom_center_y());
    alarm_big_place(incoming, alarm_big_home_dx(s_edit.state), p);
    alarm_edit_state_t prev =
        (s_edit.state == ALARM_EDIT_HOUR) ? ALARM_EDIT_MINUTE : ALARM_EDIT_HOUR;
    alarm_big_place(outgoing, alarm_big_home_dx(prev), ALARM_ANIM_STEPS - p);
}

static void alarm_switch_anim_ready(lv_anim_t *a)
{
    (void)a;
    lv_obj_t *outgoing = s_edit_big_lbl[1 - s_edit_big_cur];
    if (outgoing)
        lv_obj_add_flag(outgoing, LV_OBJ_FLAG_HIDDEN);
    s_edit_switching = false;
    alarm_big_place(s_edit_big_lbl[s_edit_big_cur],
                    alarm_big_home_dx(s_edit.state), ALARM_ANIM_STEPS);
    alarm_edit_apply_layout();
}
*/

/* 头部短按：循环切换编辑字段（时 ↔ 分，仅两项）。
 * 2026-08-14：切换动画已注释，这里只保留状态切换 + 高亮刷新。 */
static void alarm_edit_advance(void)
{
    s_edit.state = (s_edit.state == ALARM_EDIT_HOUR) ? ALARM_EDIT_MINUTE : ALARM_EDIT_HOUR;

    if (!lvgl_port_lock(100))
        return; /* 拿不到锁：高亮没能刷新，视觉上"没反应"，故此处也【不震】 */

    alarm_edit_render(); /* 只刷新高亮与提示，不做大数字飞出飞入动画 */
    lvgl_port_unlock();

    /* ★【2026-08-31】切字段成功（状态已改 + 高亮已重绘）才震。触摸层已把闹钟编辑页
     * 排除（bsp_touch.c 的 head_vib_view），本处是唯一震动来源，不会重复。
     * 放在 unlock 之后：只置标志（真正打点在触摸任务循环末尾），不阻塞 LVGL。 */
    bsp_touch_request_vibrate();
}

/* 反向切换字段：2026-08-04 左耳长按改作「取消」后已无调用者（仅两项时本就与 advance 等价）。
 * 保留备用，加 unused 属性避免 -Wunused-function 告警。 */
__attribute__((unused)) static void alarm_edit_back(void)
{
    s_edit.state = (s_edit.state == ALARM_EDIT_HOUR) ? ALARM_EDIT_MINUTE : ALARM_EDIT_HOUR;
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 倒计时页面（2026-08-04：左耳调十位、右耳调个位，分/秒仍由头部短按切换）
 * ═══════════════════════════════════════════════════════════════ */
static void countdown_page_create(void)
{
    if (s_cd_time_lbl != NULL)
        return;

    /* ── 6 个预设色块（2 行 × 3 列，圆角，各自背景色）──
     * 用 lv_obj 而非 lv_btn：本页不走 LVGL 输入设备，全部靠触摸事件手动路由，
     * 不需要按钮的点击态/焦点组，纯当色块用，开销更小。
     * 每块内挂一个居中偏上的文字标签（"1 min" 等），随父对象销毁，不单独存句柄。 */
    const lv_coord_t grid_w = CD_TILE_W * 3 + CD_TILE_GAP_X * 2;
    const lv_coord_t x0 = (BSP_LCD_WIDTH - grid_w) / 2; /* 网格左边距，整体水平居中 */
    for (size_t i = 0; i < CD_PRESET_COUNT; i++)
    {
        const lv_coord_t col = (lv_coord_t)(i % 3);
        const lv_coord_t row = (lv_coord_t)(i / 3);

        lv_obj_t *tile = lv_obj_create(s_menu_panel);
        lv_obj_remove_style_all(tile); /* 去掉默认边框/内边距，从零起样式 */
        lv_obj_set_size(tile, CD_TILE_W, CD_TILE_H);
        lv_obj_set_pos(tile, x0 + col * (CD_TILE_W + CD_TILE_GAP_X),
                       CD_TILE_TOP_Y + row * (CD_TILE_H + CD_TILE_GAP_Y));
        lv_obj_set_style_radius(tile, CD_TILE_RADIUS, 0); /* 倒角 */
        lv_obj_set_style_bg_color(tile, lv_color_hex(s_cd_presets[i].color), 0);
        lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0); /* 恒满不透明，选中态不改背景 */
        lv_obj_set_style_border_width(tile, 0, 0);      /* 无描边 */
        lv_obj_clear_flag(tile, LV_OBJ_FLAG_SCROLLABLE);

        lv_obj_t *lbl = lv_label_create(tile);
        /* 字体用 font_cn_16：本页已在用（state/hint 标签），且含 ASCII 可显示 "15 min"。
         * 未用 lv_font_montserrat_16 —— sdkconfig 里只开了 _14 和 _48 两档，
         * 为一个色块标签去改 sdkconfig 开新字体不划算（还会增大固件）。 */
        lv_obj_set_style_text_font(lbl, &font_cn_16, 0);
        lv_obj_set_style_text_color(lbl, lv_color_white(), 0);
        lv_label_set_text(lbl, s_cd_presets[i].text);
        lv_obj_align(lbl, LV_ALIGN_LEFT_MID, 6, -4); /* 2026-08-12 竖直居中靠上一点 */

        s_cd_tiles[i] = tile;
    }

    /* 运行/到期状态：整体时间标签 MM:SS（SET 状态隐藏）。
     * 2026-08-10：由屏幕中央下移到底部，给上方 6 个色块让位。 */
    s_cd_time_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_time_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_cd_time_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_cd_time_lbl, LV_ALIGN_BOTTOM_MID, 0, -4);

    /* 设置状态：分钟/冒号/秒钟 三个独立标签，可分别上色（RUNNING/EXPIRED 时隐藏）。
     * 2026-08-10：同样下移到底部，即需求里的「自定义倒计时放到下面」。
     * 三段的相对间距（±55）和分色逻辑原样保留，未作任何改动。 */
    s_cd_min_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_min_lbl, &lv_font_montserrat_48, 0);
    lv_obj_align(s_cd_min_lbl, LV_ALIGN_BOTTOM_MID, -55, -4);

    s_cd_colon_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_colon_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cd_colon_lbl, lv_color_white(), 0);
    lv_label_set_text(s_cd_colon_lbl, ":");
    lv_obj_align(s_cd_colon_lbl, LV_ALIGN_BOTTOM_MID, 0, -4);

    s_cd_sec_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_sec_lbl, &lv_font_montserrat_48, 0);
    lv_obj_align(s_cd_sec_lbl, LV_ALIGN_BOTTOM_MID, 55, -4);

    s_cd_state_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_state_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_cd_state_lbl, lv_color_hex(0x88CCFF), 0);
    lv_obj_set_style_text_align(s_cd_state_lbl, LV_TEXT_ALIGN_CENTER, 0);
    /* 2026-08-10：原在屏幕中央 +30，会压到第二行色块上；改为贴在底部大号 MM:SS 正上方。
     * 该标签仅 RUNNING/EXPIRED 有文字（"倒计时中..."/"倒计时结束!"），SET 态为空串不占视觉。 */
    lv_obj_align(s_cd_state_lbl, LV_ALIGN_BOTTOM_MID, 0, -60);

    s_cd_hint_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_hint_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_cd_hint_lbl, lv_color_hex(0x666666), 0);
    lv_label_set_long_mode(s_cd_hint_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_cd_hint_lbl, BSP_LCD_WIDTH - 16);
    lv_obj_set_style_text_align(s_cd_hint_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_cd_hint_lbl, LV_ALIGN_BOTTOM_MID, 0, -8);
}

/**
 * @brief 刷新 6 个预设色块的选中态（2026-08-10 番茄时钟）
 *
 * 选中项：白色 2px 描边 + 完全不透明；未选中：无描边 + 降透明度（CD_TILE_DIM_OPA）变暗。
 * 选中「自定义」档（s_cd.sel == CD_SEL_CUSTOM）时六块全部变暗，视觉焦点落到底部 MM:SS。
 *
 * @param visible false=整组隐藏（RUNNING/EXPIRED 态只显示大号剩余时间，不再展示选择器）
 */
static void countdown_tiles_refresh(bool visible)
{
    for (size_t i = 0; i < CD_PRESET_COUNT; i++)
    {
        if (s_cd_tiles[i] == NULL)
            continue;
        if (!visible)
        {
            lv_obj_add_flag(s_cd_tiles[i], LV_OBJ_FLAG_HIDDEN);
            continue;
        }
        lv_obj_clear_flag(s_cd_tiles[i], LV_OBJ_FLAG_HIDDEN);

        /* 2026-08-10 需求修正：六个色块的背景色【始终】按 s_cd_presets 表原色满不透明显示，
         * 不再靠"未选中变暗"来区分（原做法导致只有当前跳到的那块看着是正色，其余发灰）。
         * 选中态改为只动【文字颜色】：选中=深色文字，未选中=白色文字，色块本身纹丝不动。 */
        const bool on = (s_cd.sel == i);
        lv_obj_t *lbl = lv_obj_get_child(s_cd_tiles[i], 0); /* 创建时唯一的子对象=文字标签 */
        if (lbl)
            lv_obj_set_style_text_color(lbl,
                                        on ? lv_color_hex(CD_TILE_TEXT_ON)
                                           : lv_color_hex(CD_TILE_TEXT_OFF),
                                        0);
    }
}

static void countdown_page_render(void)
{
    if (!s_cd_time_lbl)
        return;

    /* ★★【2026-08-25 根因修复：不在番茄钟页时一律不渲染】★★
     *
     * 【本函数会改可见性】下面三个分支都会 lv_obj_clear_flag(..., HIDDEN)：
     *   SET 分支 → countdown_tiles_refresh(true) 把 6 个预设色块全部 unhide，
     *              外加 min/colon/sec 三个标签；
     *   RUNNING / EXPIRED 分支 → unhide s_cd_time_lbl。
     * 而原先唯一的守卫是上面那句"对象建了没有"，【完全不看当前停在哪一页】。
     *
     * 【导致的实测故障】驱动本函数的两个页面级 timer 在用户早已离开番茄钟页之后
     *   仍然活着（countdown_page_hide 只 cancel 了 expire_tmr，从不停 tick_tmr）：
     *       s_cd.tick_tmr（每秒）→ 到期 → countdown_enter_expired() → 本函数
     *                            → 5 秒后 countdown_expire_reset_cb → 本函数
     *   于是在【闹钟页 / 主界面 / 低功耗待机时钟】上，番茄钟的色块和数字被
     *   悄悄 unhide 出来。日志指纹＝人已不在该页，却打出
     *       "番茄时钟到期提示结束，已自动回到选择界面"
     *   随后 ui_standby_clock_show() 一 clear_flag(s_menu_panel)，这些已 unhide 的
     *   子对象连同待机时间页一起显示 → 实测照片：12:22 大字压在六个彩色预设块上。
     *
     * 【改法】渲染前先确认"我还在这一页"，否则直接返回，不碰任何对象。
     *   状态机（s_cd.state / timer_id）照常推进，只是不往屏幕上画——下次真正
     *   进入本页时 countdown_page_show() 会按当时状态完整重画，不丢任何信息。
     *
     * 【为何不会误伤正常路径】所有正常调用点都在"已经切到番茄钟页"之后：
     *   · render_fn_page(FN_PAGE_COUNTDOWN) 先设好 s_view/s_fn_page 再调 page_show；
     *   · ui_show_countdown_expired() 同样先 render_fn_page 再置 EXPIRED 重画；
     *   · 触摸调值 / countdown_cancel / countdown_enter_prepare_ui 均发生在本页内。 */
    if (s_view != UI_VIEW_FUNCTION_MENU || s_fn_page != FN_PAGE_COUNTDOWN)
        return;

    char buf[16];
    switch (s_cd.state)
    {
    case CD_STATE_SET:
    {
        countdown_tiles_refresh(/*visible=*/true); /* 设置态才显示预设选择器 */

        /* ★【2026-09-01】设定态把两个标签还原成创建时的贴底排版。
         * 【为什么必须还原】RUNNING/EXPIRED 分支把它们改成了 CENTER_MID；到期 5 秒后
         *   countdown_expire_reset_cb 会切回 SET 并重画本页，若不还原，s_cd_state_lbl
         *   就会停在屏幕正中压住第二行色块（s_cd_time_lbl 虽被隐藏但同理还原以免残留）。
         * 数值 -4 / -60 与 countdown_page_create() 里的创建值完全一致。 */
        lv_obj_align(s_cd_time_lbl, LV_ALIGN_BOTTOM_MID, 0, -4);
        lv_obj_align(s_cd_state_lbl, LV_ALIGN_BOTTOM_MID, 0, -60);

        /* 整体标签隐藏，改用分色的三段标签 */
        lv_obj_add_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cd_min_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cd_colon_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cd_sec_lbl, LV_OBJ_FLAG_HIDDEN);

        /* 橙色=当前编辑字段，白色=另一字段。
         * 2026-08-10：仅在选中「自定义」档时才分色高亮——选中预设色块时底部这组数字
         * 不可调，全部显示为白色（灰白），避免误导用户以为左/右耳还能改它。
         * 数值本身（minutes/seconds）不受选中项影响，始终保留上次的自定义倒计时值。 */
        const bool custom_on = (s_cd.sel == CD_SEL_CUSTOM);
        lv_color_t active = custom_on ? lv_color_hex(0xFF9500) : lv_color_hex(0x888888);
        lv_color_t inactive = custom_on ? lv_color_white() : lv_color_hex(0x888888);
        snprintf(buf, sizeof(buf), "%02d", s_cd.minutes);
        lv_label_set_text(s_cd_min_lbl, buf);
        lv_obj_set_style_text_color(s_cd_min_lbl,
                                    s_cd.editing_sec ? inactive : active, 0);
        snprintf(buf, sizeof(buf), "%02d", s_cd.seconds);
        lv_label_set_text(s_cd_sec_lbl, buf);
        lv_obj_set_style_text_color(s_cd_sec_lbl,
                                    s_cd.editing_sec ? active : inactive, 0);
        /* 冒号跟随整组明暗，未选中自定义时一并变灰 */
        lv_obj_set_style_text_color(s_cd_colon_lbl, inactive, 0);

        lv_label_set_text(s_cd_state_lbl, "");
        lv_label_set_text(s_cd_hint_lbl, "");
        break;
    }
    case CD_STATE_RUNNING:
    {
        countdown_tiles_refresh(/*visible=*/false); /* 运行态收起选择器，只留大号剩余时间 */

        /* ★【2026-09-01】运行态把大号 MM:SS 与状态文字移到屏幕正中。
         * 【原因】这两个标签创建时按 SET 态排版（贴底 BOTTOM_MID），SET 态下面还有
         *   6 个色块占着上半屏所以合理；但 RUNNING/EXPIRED 态色块全部隐藏，屏幕上
         *   只剩这两个标签却仍贴在底部，视觉上明显偏下。
         * 【改法】只在这两个状态里改对齐（CENTER_MID），SET 分支再改回 BOTTOM_MID，
         *   不动创建时的默认排版，也不动色块布局。
         *   -22 / +26 是让「时间」整体居中、「倒计时中...」贴在其下方一行。 */
        lv_obj_align(s_cd_time_lbl, LV_ALIGN_CENTER, 0, -22);
        lv_obj_align(s_cd_state_lbl, LV_ALIGN_CENTER, 0, 26);

        /* 三段标签隐藏，改回整体标签 */
        lv_obj_clear_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_min_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_colon_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_sec_lbl, LV_OBJ_FLAG_HIDDEN);

        /* 2026-08-16：渐变进入时会先切到本状态、但此刻计时还没启动（timer_id 仍为 -1），
         * 直接取 remain 会拿到 0 而画成 00:00。故 timer_id < 0 时按【设定值】显示，
         * 等渐亮结束真正启动计时后，tick 会接管刷新。 */
        uint32_t remain = 0;
        if (s_cd.timer_id < 0 ||
            reminder_timer_get_remain(s_cd.timer_id, &remain) != ESP_OK)
            remain = countdown_pick_duration_sec();
        snprintf(buf, sizeof(buf), "%02lu:%02lu",
                 (unsigned long)(remain / 60), (unsigned long)(remain % 60));
        lv_label_set_text(s_cd_time_lbl, buf);
        lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_hex(0xFF9500), 0);
        lv_label_set_text(s_cd_state_lbl, "倒计时中...");
        lv_label_set_text(s_cd_hint_lbl, "");
        break;
    }
    case CD_STATE_EXPIRED:
        countdown_tiles_refresh(/*visible=*/false); /* 到期态同样收起选择器 */

        /* ★【2026-09-01】到期态同 RUNNING：两个标签移到屏幕正中，理由见上一分支注释 */
        lv_obj_align(s_cd_time_lbl, LV_ALIGN_CENTER, 0, -22);
        lv_obj_align(s_cd_state_lbl, LV_ALIGN_CENTER, 0, 26);

        lv_obj_clear_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_min_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_colon_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_sec_lbl, LV_OBJ_FLAG_HIDDEN);

        lv_label_set_text(s_cd_time_lbl, "00:00");
        lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_hex(0xFF3333), 0);
        lv_label_set_text(s_cd_state_lbl, "倒计时结束!");
        lv_label_set_text(s_cd_hint_lbl, "");
        break;
    }
}

static void countdown_page_show(void)
{
    countdown_page_create();
    if (s_cd.state == CD_STATE_RUNNING)
    {
        uint32_t remain = 0;
        if (reminder_timer_get_remain(s_cd.timer_id, &remain) != ESP_OK || remain == 0)
        {
            /* 2026-08-10：进页面就发现已到期（在别的页面待过的情况）。
             * 这里只置状态，自动复位定时器由本函数末尾统一挂（见下方注释），
             * 避免与 countdown_page_render() 的调用顺序打架。 */
            s_cd.state = CD_STATE_EXPIRED;
            if (s_cd.tick_tmr != NULL)
            {
                lv_timer_pause(s_cd.tick_tmr);
                lv_timer_del(s_cd.tick_tmr);
                s_cd.tick_tmr = NULL;
            }
        }
        else if (s_cd.tick_tmr == NULL)
        {
            s_cd.tick_tmr = lv_timer_create(countdown_tick_cb, 1000, NULL);
        }
    }
    else if (s_cd.state == CD_STATE_EXPIRED)
    {
        if (s_cd.tick_tmr != NULL)
        {
            lv_timer_pause(s_cd.tick_tmr);
            lv_timer_del(s_cd.tick_tmr);
            s_cd.tick_tmr = NULL;
        }
    }
    /* render 内部会按状态决定哪些标签显示/隐藏，只需确保 hint/state 可见 */
    lv_obj_clear_flag(s_cd_hint_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_cd_state_lbl, LV_OBJ_FLAG_HIDDEN);
    countdown_page_render();

    /* 2026-08-10 方案 A：若本次进页面时已处于到期态，同样挂上自动复位定时器，
     * 保证任何进入 EXPIRED 的路径最终都会自己回到选择界面，不会停在死界面上。 */
    if (s_cd.state == CD_STATE_EXPIRED && s_cd_expire_tmr == NULL)
    {
        s_cd_expire_tmr = lv_timer_create(countdown_expire_reset_cb, CD_EXPIRE_HOLD_MS, NULL);
        if (s_cd_expire_tmr == NULL)
        {
            s_cd.state = CD_STATE_SET; /* 兜底同 countdown_enter_expired */
            s_cd.timer_id = -1;
            countdown_page_render();
        }
    }
}

static void countdown_page_hide(void)
{
    if (s_cd_time_lbl)
        lv_obj_add_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_cd_min_lbl)
        lv_obj_add_flag(s_cd_min_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_cd_colon_lbl)
        lv_obj_add_flag(s_cd_colon_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_cd_sec_lbl)
        lv_obj_add_flag(s_cd_sec_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_cd_hint_lbl)
        lv_obj_add_flag(s_cd_hint_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_cd_state_lbl)
        lv_obj_add_flag(s_cd_state_lbl, LV_OBJ_FLAG_HIDDEN);
    /* 2026-08-10：6 个预设色块也必须一并隐藏，否则会残留到日历/闹钟/天气等其它功能页 */
    for (size_t i = 0; i < CD_PRESET_COUNT; i++)
    {
        if (s_cd_tiles[i])
            lv_obj_add_flag(s_cd_tiles[i], LV_OBJ_FLAG_HIDDEN);
    }
    /* 页面已隐藏，到期复位定时器没有存在意义：留着它会在别的页面上悄悄改写
     * s_cd.state 并调 countdown_page_render()（对已隐藏对象操作虽安全但无谓）。
     * 状态直接复位为 SET，下次进页面就是干净的选择界面。 */
    if (s_cd_expire_tmr != NULL)
    {
        countdown_expire_reset_cancel();
        if (s_cd.state == CD_STATE_EXPIRED)
        {
            s_cd.state = CD_STATE_SET;
            s_cd.timer_id = -1;
        }
    }

    /* ★【2026-08-25 补漏：页面秒刷新定时器也必须停】★
     * 【原来漏了什么】上面只 cancel 了 s_cd_expire_tmr，而 s_cd.tick_tmr
     *   （1 秒一次的页面秒刷新）在离开本页后【从未被停过】，一路跑到倒计时到期，
     *   然后 countdown_tick_cb → countdown_enter_expired() → countdown_page_render()
     *   在别的页面上把番茄钟对象 unhide 出来（实测：闹钟页/待机时钟上叠出六个色块）。
     *   countdown_start_and_exit() 上方的注释其实早就写明了预期行为——
     *   "s_cd.tick_tmr 仅负责页面上的秒刷新，下次再进倒计时页时 countdown_page_show()
     *   会按 RUNNING 状态重建它"，只是这一步一直没实现。现在补上。
     * 【不影响倒计时本身】计时由 reminder 模块托管（reminder_timer_start），
     *   与本 timer 无关；停掉它只是不再刷新一个看不见的页面，到期提醒照常触发。
     * 【重建时机】countdown_page_show() 里 RUNNING 分支会按需重建（见该函数）。 */
    if (s_cd.tick_tmr != NULL)
    {
        lv_timer_pause(s_cd.tick_tmr);
        lv_timer_del(s_cd.tick_tmr);
        s_cd.tick_tmr = NULL;
    }
}
/**
 *  倒计时页面
 *  - 短按加减分钟，长按开始/取消
 *  - 运行时显示剩余时间，结束时显示提示并震动
 * */

/* 取消到期复位定时器（幂等）。调用方需持 LVGL 锁或处于 LVGL 上下文。 */
static void countdown_expire_reset_cancel(void)
{
    if (s_cd_expire_tmr != NULL)
    {
        lv_timer_del(s_cd_expire_tmr);
        s_cd_expire_tmr = NULL;
    }
}

/* 到期停留结束：回到选择界面（CD_STATE_SET），六个色块重新出现。
 * 由 LVGL 线程调用，已持锁，内部不得再 lvgl_port_lock。 */
static void countdown_expire_reset_cb(lv_timer_t *t)
{
    s_cd_expire_tmr = NULL; /* 先摘句柄，防止下方路径重复 del */
    lv_timer_del(t);

    if (s_cd.state != CD_STATE_EXPIRED)
        return; /* 期间用户已手动改变状态，不再插手 */

    /* ★【2026-09-01 需求改造：结束画面 → 设定界面 也走渐变】★
     * 改前是硬切：整屏从"红色 00:00 + 倒计时结束"直接跳回"六个预设色块 + MM:SS"，
     * 亮屏下能看到逐条擦除。改后藏进黑屏里。
     * quiet_main_screen=false：此刻已在功能页内，主界面轮播早已 pause、舵机早已归中，
     * 不需要也不应该再 flush 一次（多一次归中动作纯属抽动）。 */
    if (expire_fade_start(countdown_reset_dark_action, /*quiet_main_screen=*/false,
                          "番茄钟到期→设定界面"))
        return;

    countdown_reset_dark_action(); /* 渐变不可用：退化硬切，绝不把用户留在到期画面上 */
}

/* 番茄钟「到期画面 → 主界面空闲 GIF」的暗态动作（2026-09-01 改版）。
 * 由 countdown_exit_fade_timer_cb 在 LVGL 线程调用（已持锁）；
 * 硬切兜底路径下由 countdown_expire_reset_cb 直接调用，同样在 LVGL 线程。 */
static void countdown_reset_dark_action(void)
{
    /* ★【2026-09-01 需求改版：到期结束不再回设定界面，直接渐变回主界面空闲 GIF】★
     *
     * 【状态复位必须保留】改前的 countdown_page_render() 是"重绘设定界面"，现在不再
     *   显示这一页，但 s_cd.state / timer_id 这两个【状态机字段】仍必须复位：
     *   它们不属于渲染，下次真正进番茄钟页时 countdown_page_show() 会按当时状态重画，
     *   若停在 EXPIRED 就会一进页面又是"红色 00:00 + 到期"那一屏。
     *
     * 【为什么用 countdown_page_hide() 而不是 countdown_page_render()】前者隐藏本页
     *   全部标签与 6 个预设色块，防止残留到主界面之外的其它页；后者是画出来，与
     *   "马上要离开这一页"的意图相反。注意 page_hide 内部也会复位 state 并删到期
     *   定时器（幂等），上面两行显式复位是防御性的，语义更清楚。
     *
     * 【顺序要求】s_expire_fade_pending 必须在 ui_home_exit_apply() 之【前】清 ——
     *   理由与 alarm_reset_dark_action 完全相同（main_idle_loop_active 那道判据），
     *   不先清会「画面回到主界面但空闲轮播与舵机动作起不来」。
     *
     * 【线程/锁】同 alarm_reset_dark_action：本回调在 LVGL 线程已持锁，
     *   ui_home_exit_apply() 内的 lvgl_port_lock(100) 是递归锁，重复获取安全。 */
    s_cd.state = CD_STATE_SET;
    s_cd.timer_id = -1;
    countdown_page_hide();
    s_expire_fade_pending = false;
    ui_home_exit_apply();
    ESP_LOGI(TAG, "番茄时钟到期提示结束，已渐变回到主界面空闲 GIF");
}

/* 进入到期态的统一入口：置状态 + 重绘 + 挂上自动复位定时器。
 * 三处入口（tick 到期 / page_show 发现已到期）都走这里，避免漏挂定时器
 * 导致又停在死界面上。调用方需持 LVGL 锁或处于 LVGL 上下文。 */
static void countdown_enter_expired(void)
{
    /* ★【2026-09-01 需求改造（修改点 5）：运行界面 → 到期画面 也走渐变】★
     * 【这是哪条路径】用户【人就停在番茄钟页上】看着秒数走完的那一次到期，走的是
     *   countdown_tick_cb → 本函数，【不经过】ui_show_countdown_expired。
     *   若只改后者，就会出现"人不在页上到期有渐变、人在页上到期是硬切"的不一致。
     * quiet_main_screen=false：人已在功能页，主界面轮播早已 pause、舵机早已归中。
     *
     * 【自带的安全退化】另一个调用场景是"进页面时发现已到期"，那时外层的
     *   home_enter_fade 正在跑（s_cd_exit_fade_phase != IDLE），expire_fade_start
     *   会直接返回 false → 走下面的硬切，天然不会打断外层渐变，也不会嵌套。 */
    if (expire_fade_start(countdown_tick_expired_dark_action, /*quiet_main_screen=*/false,
                          "番茄钟运行→到期画面"))
        return;

    countdown_tick_expired_dark_action(); /* 渐变不可用：退化硬切，行为与改动前一致 */
}

/* 番茄钟「运行界面 → 到期画面」的暗态动作（2026-09-01）。
 * 内容原样搬自 countdown_enter_expired 的旧函数体，逻辑一行未改。 */
static void countdown_tick_expired_dark_action(void)
{
    s_cd.state = CD_STATE_EXPIRED;
    countdown_page_render();

    countdown_expire_reset_cancel(); /* 幂等：防止重复进入时挂上两个定时器 */
    s_cd_expire_tmr = lv_timer_create(countdown_expire_reset_cb, CD_EXPIRE_HOLD_MS, NULL);
    if (s_cd_expire_tmr == NULL)
    {
        /* 定时器创建失败兜底：直接复位，绝不把用户留在无法操作的到期界面上 */
        ESP_LOGW(TAG, "到期复位定时器创建失败，立即回到选择界面");
        s_cd.state = CD_STATE_SET;
        s_cd.timer_id = -1;
        countdown_page_render();
    }
    s_expire_fade_pending = false;
}

static void countdown_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_cd.state != CD_STATE_RUNNING)
        return;

    uint32_t remain = 0;
    if (reminder_timer_get_remain(s_cd.timer_id, &remain) != ESP_OK || remain == 0)
    {
        /* 2026-08-10：改走统一入口，内含 CD_EXPIRE_HOLD_MS 后自动回选择界面 */
        countdown_enter_expired();
        /* 注意：bsp_motor_pulse 阻塞 30ms，此处会短暂阻塞 LVGL。
         * 如需优化，可改为发送事件到非 LVGL 任务执行震动。
         * 注：真正的「到期长震 2 秒」在 reminder.c 的 REM_EVT_TIMER_EXPIRE 分支
         * （bsp_motor_pulse_level），那条不依赖本页面存活；此处这一下是页面内的短反馈。 */
        bsp_motor_pulse();
        if (s_cd.tick_tmr != NULL)
            lv_timer_set_repeat_count(s_cd.tick_tmr, 0);
        s_cd.tick_tmr = NULL;
        return;
    }

    char buf[16];
    snprintf(buf, sizeof(buf), "%02lu:%02lu",
             (unsigned long)(remain / 60), (unsigned long)(remain % 60));
    if (s_cd_time_lbl)
        lv_label_set_text(s_cd_time_lbl, buf);
}

static void countdown_start(void)
{
    /* 2026-08-10 番茄时钟：时长取当前选中项——
     *   选中 6 个预设色块之一 → 该预设的整分钟数（秒恒为 0）
     *   选中底部「自定义」     → 沿用原有 minutes*60 + seconds（逻辑未改）
     * 注意不要把预设值写回 s_cd.minutes/seconds，否则会冲掉用户上次设的自定义值，
     * 而需求要求自定义档「默认显示为上次倒计时」。 */
    uint32_t duration_sec = countdown_pick_duration_sec();

    if (duration_sec == 0)
    {
        ESP_LOGW(TAG, "倒计时时长为 0，忽略启动");
        return; /* 自定义档被调成 00:00 时不启动，避免建一个立刻到期的定时器 */
    }

    s_cd.timer_id = reminder_timer_start(duration_sec, "倒计时结束");
    if (s_cd.timer_id < 0)
    {
        ESP_LOGE(TAG, "倒计时启动失败");
        return;
    }
    s_cd.state = CD_STATE_RUNNING;
    if (lvgl_port_lock(100))
    {
        if (s_cd.tick_tmr == NULL)
            s_cd.tick_tmr = lv_timer_create(countdown_tick_cb, 1000, NULL);
        countdown_page_render();
        lvgl_port_unlock();
    }
    ESP_LOGI(TAG, "番茄时钟启动: %lu 分 %lu 秒 (选中项=%s)",
             (unsigned long)(duration_sec / 60), (unsigned long)(duration_sec % 60),
             s_cd.sel < CD_PRESET_COUNT ? s_cd_presets[s_cd.sel].text : "自定义");
}

/* ── 进入渐变专用：把 countdown_start 拆成「先画版式」+「后真正计时」两步 ──
 *
 * 【为什么要拆】进入渐变把 countdown_start 推迟到暗态执行，但倒计时从
 * reminder_timer_start 那一刻就真实走秒了，而屏幕还要等「暗态 50ms + 渐亮 500ms」
 * 才亮起来 —— 这 550ms 的秒变化用户全看不到，实测表现就是「画面刚亮起就已经
 * 掉了 1~2 秒、中间几次刷新像消失了」。
 * 拆开后：暗态只画版式（显示设定值，不启动计时），渐亮结束才真正启动计时，
 * 用户看到画面亮起时数字才从设定值开始走，一秒不丢。 */

/* 取当前选中项对应的倒计时秒数（预设色块取整分钟，自定义档取 minutes*60+seconds）。
 * 从 countdown_start 里抽出来，供「渐变进入」的两步各自调用，避免重复这段选择逻辑。
 * 注意不把预设值写回 s_cd.minutes/seconds，以免冲掉用户上次设的自定义值。 */
static uint32_t countdown_pick_duration_sec(void)
{
    if (s_cd.sel < CD_PRESET_COUNT)
        return (uint32_t)s_cd_presets[s_cd.sel].minutes * 60;
    return (uint32_t)s_cd.minutes * 60 + s_cd.seconds;
}

/* 第一步（暗态执行）：只切到运行态版式并按设定值渲染，【不】启动 reminder 计时。
 * 此时 s_cd.timer_id 仍是 -1，countdown_page_render 的 RUNNING 分支已针对这种情况
 * 改为按设定值显示（见那里的 2026-08-16 注释），故这里只切状态 + 渲染即可。
 * @return true=版式已切好可以继续渐亮；false=时长为 0 等不该启动的情况 */
static bool countdown_enter_prepare_ui(void)
{
    if (countdown_pick_duration_sec() == 0)
    {
        ESP_LOGW(TAG, "倒计时时长为 0，忽略启动");
        return false;
    }

    s_cd.state = CD_STATE_RUNNING; /* 先切状态，让 render 走 RUNNING 版式 */
    if (lvgl_port_lock(100))
    {
        countdown_page_render();
        lvgl_port_unlock();
    }
    return true;
}

/* 第二步（渐亮结束执行）：真正启动 reminder 计时 + 起 1s tick 页面刷新。
 * 版式已在第一步切好，这里不再重画整页，避免又出现一次可见刷新。 */
static void countdown_enter_start_timing(void)
{
    uint32_t duration_sec = countdown_pick_duration_sec();
    if (duration_sec == 0)
        return;

    s_cd.timer_id = reminder_timer_start(duration_sec, "倒计时结束");
    if (s_cd.timer_id < 0)
    {
        ESP_LOGE(TAG, "倒计时启动失败");
        s_cd.state = CD_STATE_SET; /* 启动失败要回退状态，否则页面停在假的运行态 */
        if (lvgl_port_lock(100))
        {
            countdown_page_render();
            lvgl_port_unlock();
        }
        return;
    }
    if (lvgl_port_lock(100))
    {
        if (s_cd.tick_tmr == NULL)
            s_cd.tick_tmr = lv_timer_create(countdown_tick_cb, 1000, NULL);
        lvgl_port_unlock();
    }
    ESP_LOGI(TAG, "番茄时钟启动（渐亮完成后计时）: %lu 分 %lu 秒 (选中项=%s)",
             (unsigned long)(duration_sec / 60), (unsigned long)(duration_sec % 60),
             s_cd.sel < CD_PRESET_COUNT ? s_cd_presets[s_cd.sel].text : "自定义");
}

/* ── 倒计时启动后延迟退出（2026-08-06，2026-08-16 改为渐变退出）──
 *
 * 需求：原实现启动后立刻退出，用户来不及看到"倒计时中"的画面，太突兀。
 * 改为启动后在本页停留 3 次 1s tick 刷新（3 秒，用户能看到数字连续走 3 下），
 * 然后不再硬切退出，而是背光渐暗到全黑时才真正切回功能盘，再渐亮——
 * 与闹钟保存退出的诉求一致：切页动作全程在黑屏里完成，避免亮屏时刷屏。
 *
 * 同样用一次性 lv_timer 而非 vTaskDelay —— 阻塞 LVGL 线程会卡死整个 UI。
 * 句柄 s_cd_exit_tmr 全程可取消：若用户在停留期间又摸了别的（腹背返回等），
 * 必须先 del 掉，否则定时器在页面已切走后触发会把用户从新页面踢回功能盘。
 */
#define COUNTDOWN_EXIT_TICK_COUNT 3                                // 启动后停留的 tick 刷新次数
#define COUNTDOWN_EXIT_DELAY_MS (COUNTDOWN_EXIT_TICK_COUNT * 1000) // 停留时长（ms）= tick 次数×1s

static lv_timer_t *s_cd_exit_tmr = NULL; ///< 启动后延迟退出定时器（NULL=未在倒数）

/* 取消延迟退出（幂等）。调用方需持有 LVGL 锁，或处于 LVGL 自身上下文。
 * 渐变本身（s_cd_exit_fade_*）由 countdown_exit_fade_cancel 单独取消，
 * 两者是「停留计时」与「退出渐变」两个独立阶段，调用方按需分别清。 */
static void countdown_exit_delay_cancel(void)
{
    if (s_cd_exit_tmr != NULL)
    {
        lv_timer_del(s_cd_exit_tmr);
        s_cd_exit_tmr = NULL;
    }
}

/* 取消倒计时退出渐变（幂等）。调用方需持有 LVGL 锁，或处于 LVGL 自身上下文。
 * 若在渐暗/暗态期间被打断（用户又摸了别的、待机强制回主界面等），背光可能
 * 停在半暗，必须显式拉回正常亮度——两个调用点（back_to_home / 待机强制回
 * 主界面）之后都会展示新画面，不能让用户看到一块暗屏。 */
static void countdown_exit_fade_cancel(void)
{
    if (s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
    {
        bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT);
        s_cd_exit_fade_phase = UI_CD_EXIT_FADE_IDLE;
    }
    /* 用途标志必须一并复位：残留 true 会让下一次【退出】渐变误走「进入」分支，
     * 暗态时去 countdown_start() 而不是切回功能盘。 */
    s_cd_fade_is_enter = false;
    /* 同理，自定义暗态动作也必须清掉：残留会让下一次退出渐变误走游戏退出分支。 */
    s_cd_fade_dark_cb = NULL;
    /* 【2026-09-01】到期渐变的「已发起」窗口标志同样一次性，必须一并归位：
     * 渐变被取消 ⇒ 暗态动作不会执行 ⇒ 没人清它 ⇒ 此后所有到期切页请求都被
     * expire_fade_start() 当成重复请求忽略掉，表现为「闹钟/番茄钟到期再也不出画面」。 */
    s_expire_fade_pending = false;
    if (s_cd_exit_fade_tmr != NULL)
        lv_timer_pause(s_cd_exit_fade_tmr);
}

/* 倒计时页背光渐变 timer 回调：渐暗→暗到底做事→渐亮，
 * 与 ui_home_fade_timer_cb 同一套推进模式，独立状态机/timer 互不干扰。
 *
 * 【一套两用】暗态那一刻做什么由 s_cd_fade_is_enter 决定：
 *   进入（true） ：countdown_start() 启动倒计时 + 切运行态版式；
 *   退出（false）：ui_function_menu_exit() 切回功能盘。
 * 两者节奏与状态流转完全相同，故复用同一状态机，避免再写一套重复代码。 */
static void countdown_exit_fade_timer_cb(lv_timer_t *t)
{
    if (s_cd_exit_fade_phase == UI_CD_EXIT_FADE_IDLE)
    {
        lv_timer_pause(t);
        return;
    }

    uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - s_cd_exit_fade_start_us) / 1000);
    bool done;
    switch (s_cd_exit_fade_phase)
    {
    case UI_CD_EXIT_FADE_OUT:
        done = bsp_board_lcd_fade_step_fine(BSP_LCD_BK_DEFAULT_PCT, 0, elapsed_ms, s_cd_fade_out_ms);
        if (done)
        {
            /* 暗到底：趁全黑做真正的画面变更，用户看不见切换过程 */
            if (s_cd_fade_is_enter)
            {
                /* 进入第一步：只切运行态版式并按设定值渲染，【不】启动计时——
                 * 计时推迟到渐亮结束（见 FADE_IN 分支），否则黑屏那 550ms 的
                 * 秒变化用户看不到，画面亮起就已经掉了 1~2 秒。
                 * 只在「设置中」才切，已在跑/已到期时只做渐变不重复启动。 */
                if (s_cd.state == CD_STATE_SET)
                {
                    if (!countdown_enter_prepare_ui())
                        s_cd_fade_is_enter = false; /* 时长为0：退化成普通渐变，不进运行态 */
                }
            }
            else if (s_cd_fade_dark_cb != NULL)
            {
                /* 【2026-08-18】先取出再清空，然后才调用：
                 * 本指针是「一次性」语义，用完必须归位，否则会残留到下一次渐变。
                 * 残留是真隐患：countdown_auto_exit / countdown_enter_fade 这两个
                 * 入口只设时长不设本指针，一旦捡到上一次的残留值，暗态就会执行错
                 * 的动作（例如倒计时自动退出时跑去执行"进入天气页"）。
                 * 先清后调而非调完再清：回调内部可能再次发起渐变并设置新的指针，
                 * 调完再清会把新设的值一并抹掉。 */
                void (*dark_cb)(void) = s_cd_fade_dark_cb;
                s_cd_fade_dark_cb = NULL;
#if UI_TIME_FADE_DIAG
                /* ★核心测点①：暗态动作阻塞时长。这一对时间戳之差就是
                 * time_page_render() 那一路在 LVGL 线程里独占了多久。 */
                s_fade_diag_dark_t0 = esp_timer_get_time();
                dark_cb();
                if (FADE_DIAG_ON())
                    ESP_LOGW(TAG, "[FD] dark_cb 阻塞 %lld ms (进入)",
                             (esp_timer_get_time() - s_fade_diag_dark_t0) / 1000);
#else
                dark_cb(); /* 自定义暗态动作（游戏退出 / 进入功能页）*/
#endif
            }
            else
            {
#if UI_TIME_FADE_DIAG
                /* ★核心测点①（退出方向）：默认暗态动作＝切回功能盘，
                 * 内部会 home_render → time_page_hide，同样可能长阻塞。 */
                s_fade_diag_dark_t0 = esp_timer_get_time();
                ui_function_menu_exit();
                if (FADE_DIAG_ON())
                    ESP_LOGW(TAG, "[FD] dark_cb 阻塞 %lld ms (退出)",
                             (esp_timer_get_time() - s_fade_diag_dark_t0) / 1000);
#else
                ui_function_menu_exit(); /* 退出：切回功能盘（默认）*/
#endif
            }
            /* 【2026-08-18】自适应 DARK：暗态动作刚刚产生了失效区，在此清零标志，
             * 之后的第一次 LV_EVENT_REFR_READY 即代表这批重绘已全部 flush 完成。
             * 必须放在暗态动作【之后】清：动作执行期间可能已触发过刷新事件。 */
            s_cd_fade_refr_done = false;
            s_cd_exit_fade_phase = UI_CD_EXIT_FADE_DARK;
            s_cd_exit_fade_start_us = esp_timer_get_time();
#if UI_TIME_FADE_DIAG
            /* OUT 段实耗：正常应≈s_cd_fade_out_ms(150)。若明显偏大，
             * 说明渐暗阶段就已经被抢占，问题不在暗态动作而在更早。 */
            if (FADE_DIAG_ON())
                ESP_LOGW(TAG, "[FD] OUT 结束 elapsed=%lu (设定 %lu) → 进 DARK",
                         (unsigned long)elapsed_ms, (unsigned long)s_cd_fade_out_ms);
            s_fade_diag_in_logged = false; /* 为本次 FADE_IN 首帧日志复位 */
#endif
        }
        break;

    case UI_CD_EXIT_FADE_DARK:
        /* 等 flush + 液晶响应走完，避免新画面那帧在背光爬升时落地导致闪烁。
         *
         * 【2026-08-18 改为自适应】原先是死等 s_cd_fade_dark_ms，按最坏情况取值，
         * 简单页面白等三四十毫秒（表现为"灭了之后干等一下"）。现在两个出口：
         *   ① 正常出口：重绘已 flush 完成（REFR_READY 到了）且已过最小暗场
         *      UI_FADE_DARK_MIN_MS（盖住液晶 20~40ms 灰阶响应，防残影被照出来）；
         *   ② 兜底出口：到达 s_cd_fade_dark_ms 上限仍没等到 REFR_READY
         *      （暗态动作没产生失效区时会走这里），行为与改动前一致，不会卡黑。 */
        if ((s_cd_fade_refr_done && elapsed_ms >= s_cd_fade_dark_min_ms) ||
            elapsed_ms >= s_cd_fade_dark_ms)
        {
#if UI_TIME_FADE_DIAG
            /* ★核心测点②：DARK 从哪个出口走的。
             *   refr=1 → 自适应出口，重绘已 flush 完成（健康）；
             *   refr=0 → 走满上限的兜底出口，说明这批重绘的 REFR_READY 从未到达，
             *            此时 elapsed 应≈s_cd_fade_dark_ms。
             * 用户反馈的「长时间停顿」若成立，这里的 elapsed 会明显大于设定值。 */
            if (FADE_DIAG_ON())
                ESP_LOGW(TAG, "[FD] DARK 结束 elapsed=%lu (上限 %lu) refr=%d → 进 IN",
                         (unsigned long)elapsed_ms, (unsigned long)s_cd_fade_dark_ms,
                         (int)s_cd_fade_refr_done);
#endif
            s_cd_exit_fade_phase = UI_CD_EXIT_FADE_IN;
            s_cd_exit_fade_start_us = esp_timer_get_time();
        }
        break;

    case UI_CD_EXIT_FADE_IN:
#if UI_TIME_FADE_DIAG
        /* ★核心测点③：渐亮首帧的 elapsed。这是「瞬亮」的直接判据——
         * bsp_lcd.c:156 里 elapsed_ms >= total_ms 会一步落到终值并返回 true，
         * 即整段渐亮被跳过。故若此处 elapsed 已 ≥ s_cd_fade_in_ms(200)，
         * 本次必然表现为瞬间亮；若只是偏大（如 80~150），则渐亮只剩几帧＝卡顿。
         * 只打首帧：不加 s_fade_diag_in_logged 会把渐亮每一帧都打出来。 */
        if (FADE_DIAG_ON() && !s_fade_diag_in_logged)
        {
            s_fade_diag_in_logged = true;
            ESP_LOGW(TAG, "[FD] IN 首帧 elapsed=%lu (设定 %lu)%s",
                     (unsigned long)elapsed_ms, (unsigned long)s_cd_fade_in_ms,
                     (elapsed_ms >= s_cd_fade_in_ms) ? "  ★渐亮被整段跳过=瞬亮" : "");
        }
#endif
        done = bsp_board_lcd_fade_step_fine(0, BSP_LCD_BK_DEFAULT_PCT, elapsed_ms, s_cd_fade_in_ms);
        if (done)
        {
#if UI_TIME_FADE_DIAG
            /* 本次渐变收尾：撤销诊断标志，避免残留到下一次非时间页的渐变
             * （与 s_cd_fade_dark_cb 的一次性语义同理）。 */
            if (FADE_DIAG_ON())
                ESP_LOGW(TAG, "[FD] ── 本次渐变结束 ──");
            s_fade_diag_armed = false;
#endif
            s_cd_exit_fade_phase = UI_CD_EXIT_FADE_IDLE;
            lv_timer_pause(t);
            /* 进入渐变收尾：此刻倒计时界面已完整亮起，这才
             *   ① 真正启动计时（黑屏期间不走秒，保证用户从设定值看起，一秒不丢）；
             *   ② 开始算「停留 3 次 tick」（若在渐变前就起，3 秒里近 1 秒被渐变吃掉，
             *      用户实际只看到 2 次跳动）。停留结束由 countdown_exit_tmr_cb 触发退出渐变。 */
            if (s_cd_fade_is_enter)
            {
                s_cd_fade_is_enter = false; /* 用途归位，下次默认按「退出」语义 */

                /* ★★【2026-08-25 修复：补齐「不重复启动」的另一半】★★
                 * 【原来的毛病】上面 FADE_OUT 分支的注释白纸黑字写着「已在跑/已到期时
                 *   只做渐变不重复启动」，但那道保护【只兑现了一半】：
                 *     · FADE_OUT 那半确实判了 s_cd.state == CD_STATE_SET 才切版式；
                 *     · 而本处（FADE_IN 收尾）却只看 s_cd_fade_is_enter，一律
                 *       countdown_enter_start_timing()，且该函数内部同样不看 state，
                 *       直接 reminder_timer_start() —— 于是「已在跑」的情况下会在
                 *       现有倒计时之上【再起一个】，与注释的承诺完全相反。
                 * 【判据为何是 RUNNING && timer_id < 0】这正是 FADE_OUT 那半成功切了
                 *   版式、但计时被刻意推迟到此刻的唯一状态组合（见 countdown_enter_prepare_ui：
                 *   它只置 state=RUNNING，不碰 timer_id，故此刻仍为 -1）。任何其它组合
                 *   都说明「版式没切成」或「计时已经在跑」，都不该再启动。
                 * 【日志的额外用途】若现场出现「每 3 分钟自发重启倒计时」，这条 WARN
                 *   能直接证明是否走了本路径，是排查该问题的关键取证点之一。 */
                if (s_cd.state == CD_STATE_RUNNING && s_cd.timer_id < 0)
                {
                    countdown_enter_start_timing();
                }
                else
                {
                    ESP_LOGW(TAG, "渐亮完成但倒计时状态不符(state=%d timer_id=%d)，跳过启动计时",
                             (int)s_cd.state, s_cd.timer_id);
                }

                /* 停留 + 自动退出的节奏保持不变：无论上面是否真的启动了计时，
                 * 都必须挂上停留定时器，否则会永久卡在倒计时页出不去。 */
                if (s_cd_exit_tmr == NULL)
                    s_cd_exit_tmr = lv_timer_create(countdown_exit_tmr_cb,
                                                    COUNTDOWN_EXIT_DELAY_MS, NULL);
                if (s_cd_exit_tmr == NULL)
                    ui_function_menu_exit(); /* 建 timer 失败兜底：直接退，不卡在页面里 */
            }
        }
        break;

    default:
        lv_timer_pause(t);
        break;
    }
}

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-09-01 新增】提醒到期渐变统一启动器
 *
 * 【服务哪些切换】闹钟/番茄钟相关的全部 5 个画面切换点，改前【全是硬切】：
 *   ① 主界面 GIF  → 番茄钟到期画面   （ui_show_countdown_expired）
 *   ② 主界面 GIF  → 闹钟响铃画面     （ui_show_alarm_ringing）
 *   ③ 番茄钟运行  → 到期画面         （countdown_enter_expired，人就停在页上时）
 *   ④ 到期画面    → 番茄钟设定界面   （countdown_expire_reset_cb）
 *   ⑤ 响铃画面    → 闹钟编辑界面     （alarm_ring_reset_cb）
 *
 * 【为什么复用 s_cd_exit_fade_* 而不新写一套】那套三段状态机
 * （渐暗 → 暗态执行 dark_cb → 渐亮）已被功能页进/退、游戏进/退共 5 个入口复用，
 * 且支持任意 dark_cb 函数指针；本次 5 个切换点的差别只在「暗态做什么」，
 * 与它的设计意图完全吻合。状态机本体一行未改。
 *
 * @param dark_cb          暗到底那一刻执行的切页动作（必须自己清 s_expire_fade_pending）
 * @param quiet_main_screen true = 本次是从【主界面】切过来（①②）：停主界面 GIF 轮播
 *                          并 flush 舵机归中；false = 已在功能页内部切换（③④⑤），
 *                          轮播早已 pause、舵机早已归中，无需重复动作。
 * @param what             日志用途标识
 * @return true = 渐变已发起（调用方直接返回）；false = 不可用，调用方应走硬切兜底
 *
 * 【线程】①②由 reminder_task 调用（栈在 PSRAM，非 LVGL 线程），③④⑤由 lv_timer
 * 回调调用（LVGL 线程、已持锁）。两种都安全：
 *   · lvgl_port_lock 是递归互斥量，同线程重入不自锁；
 *   · 本函数只做 atomic/队列/timer 标志操作，【不碰 flash/NVS】——
 *     这是 PSRAM 栈任务的铁律（关 cache 后 PSRAM 失联，栈读不到会直接断言 abort），
 *     servo_manager_flush() 内部仅 atomic_store + xQueueReset，符合要求。
 * ══════════════════════════════════════════════════════════════════════════ */
static bool expire_fade_start(void (*dark_cb)(void), bool quiet_main_screen, const char *what)
{
    if (dark_cb == NULL)
        return false;

    /* ⚠️【为什么"重复请求"的判断不在这里】本函数的返回值只有一个含义：
     *   false = 渐变不可用，【调用方应走硬切兜底】。
     * 若把 s_expire_fade_pending 的重复判断放进来一并 return false，调用方就会把
     * "画面切换已经在路上了"误当成"渐变坏了"，于是每一次重复请求都硬切一遍 ——
     * 比不改还糟。而它又不能一律 return true：退出方向（到期→设定 / 响铃→编辑）
     * 若被这样吞掉，状态就永远停在到期/响铃画面上了。
     * 两种方向的正确处置不同，故重复判断交给【需要它的那两个调用方】自己做
     * （ui_show_countdown_expired / ui_show_alarm_ringing，它们的正确处置是"直接返回"）。 */

    if (s_cd_exit_fade_tmr == NULL)
        return false; /* 无 timer：调用方走硬切兜底，功能不因动效缺失而失效 */

    /* 取锁 500ms：到期常紧跟在"退低功耗"之后，那一刻 GIF 轮播正在切图（文件 I/O +
     * 解码首帧），100ms 抢不过它 —— 与 ui_show_countdown_expired 原注释同一理由。 */
    if (!lvgl_port_lock(500))
        return false;

    if (s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
    {
        lvgl_port_unlock();
        return false; /* 已有别的渐变在跑：不打断它，调用方退化硬切 */
    }

    /* ★★【2026-09-01 补漏：闸门必须在 flush 之前置起】★★
     * 【为什么不能留在下面和其它状态一起设】servo_manager_flush() 是整条故障链的
     *   【触发源】：它打断情绪舵机 → worker give 信号量 → interaction 收尾解阻塞
     *   → ui_resume_main_gif_loop() → 重新排图并 resume 轮播 → 投递新的空闲舵机动作。
     *   这条链最快可以在 flush 返回后几毫秒内跑完，若闸门晚一步置起就有空窗，
     *   等于没修 —— 与 ui_home_enter() 里"先关闸再 flush"是同一条铁律。
     * 【为什么放在 quiet_main_screen 判断之外】③④⑤（功能页内部切换）虽然不 flush，
     *   置上也无害：那时 s_view 已不是 MAIN，判据本来就为假，且暗态动作会统一清掉。
     *   统一置位还顺带保证了 ui_show_* 那两个"重复请求直接返回"的判断立即生效。 */
    s_expire_fade_pending = true;

    if (quiet_main_screen)
    {
        /* ★★【顺序不可颠倒：先停轮播，再 flush 舵机】★★
         * 反过来的话，flush 到 pause 之间的那一拍里，主界面轮播还能再投递一条
         * 空闲舵机动作（main_gif_switch_timer_cb → ui_interaction_play_custom），
         * 于是"刚归完中又被新动作顶走"，渐亮时舵机还在动 —— 与进功能盘那个
         * 偶发 bug 完全同源（详见 s_home_enter_pending 声明处的时序分析）。
         *
         * 【为什么归中放在这里而不是暗态】用户 2026-09-01 明确要求「舵机归中在渐暗
         * 就开始」。归中速度见 bsp_config.h 的 SERVO_SPEED_CENTER（当前 30ms/度，
         * 同日由 15ms/度 降档，原因：15 显得太快、太机械）。
         * 时间预算：±30° 归中约 900ms，落在整段渐变窗口内（渐暗450 + 全黑100 +
         * 渐亮550 = 1100ms），画面完全亮起时已站好；±40° 约 1200ms 会拖到渐亮末段
         * 差一点点。若还要更慢，应连同 UI_EXPIRE_FADE_* 一起放长，别只调速度宏。 */
        if (s_gif_switch_tmr != NULL)
            lv_timer_pause(s_gif_switch_tmr);
        servo_manager_flush();
    }

    s_cd_fade_is_enter = false; /* 非倒计时「进入运行态」语义，走 dark_cb 分支 */
    s_cd_fade_dark_cb = dark_cb;
    s_cd_fade_out_ms = UI_EXPIRE_FADE_OUT_MS;
    s_cd_fade_dark_ms = UI_EXPIRE_FADE_DARK_MS;
    s_cd_fade_in_ms = UI_EXPIRE_FADE_IN_MS;
    /* 地板归位成默认值：本变量是「上一次谁设的就留着」的语义，若捡到功能页退出档
     * 设的加长地板，会白白多黑一段（同 home_enter_fade / game_enter_fade 的处理）。 */
    s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS;
    /* s_expire_fade_pending 已在本函数上方、flush 之前置起（见那里的说明），此处不再重复 */
    s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
    s_cd_exit_fade_start_us = esp_timer_get_time();
    lv_timer_resume(s_cd_exit_fade_tmr);
    lvgl_port_unlock();

    /* 与其它渐变入口一致：挂进 LVGL 后必须显式唤醒 taskLVGL，否则要等它自然睡醒
     * （最长约 1000ms）才开始渐暗，整个 OUT 段一步都推不动 → 表现为"愣一下然后
     * 瞬间黑屏"。完整根因见 ui_func_layer_exit_to_main 末尾那段注释。 */
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
    ESP_LOGI(TAG, "到期渐变开始：%s", what ? what : "");
    return true;
}

/* 停留够 3 次 tick：自毁停留定时器 → 触发退出渐变（渐暗→切页→渐亮）。
 * 由 LVGL 线程调用，已持锁，内部不得再 lvgl_port_lock。 */
static void countdown_exit_tmr_cb(lv_timer_t *t)
{
    s_cd_exit_tmr = NULL; /* 先摘句柄，防止退出路径重复 del */
    lv_timer_del(t);

    if (s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
        return; /* 防御：理论不会重入，仍判一次避免打断正在跑的渐变 */
#if UI_TIME_FADE_DIAG
    /* ★【2026-08-25】停留 3 秒后的【自动退出】渐变，同样 arm 上诊断。
     * 用户描述的"隔一小段才退出"很可能就是这一段——若它与手动触发的渐变
     * 发生重叠，日志里会出现两条"渐变开始"，一眼可辨。 */
    s_fade_diag_armed = true;
    s_fade_diag_in_logged = false;
    ESP_LOGW(TAG, "[FD] ══ 倒计时停留结束，自动退出：渐变开始 ══");
#endif
    s_cd_fade_out_ms = UI_CD_EXIT_FADE_OUT_MS; /* 倒计时档：自动退出，从容节奏 */
    s_cd_fade_dark_ms = UI_CD_EXIT_FADE_DARK_MS;
    s_cd_fade_in_ms = UI_CD_EXIT_FADE_IN_MS;
    s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS; /* 地板归位（本档 850ms 已足够从容，不需额外压） */
    s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
    s_cd_exit_fade_start_us = esp_timer_get_time();
    if (s_cd_exit_fade_tmr != NULL)
        lv_timer_resume(s_cd_exit_fade_tmr);
    else
        ui_function_menu_exit(); /* 渐变 timer 创建失败兜底：直接硬切，不因动效缺失卡住 */
}

/**
 * @brief 头部长按：渐变进入倒计时 + 停留 3 次 tick 后渐变退出功能页
 *        （2026-08-06 需求，2026-08-16 改为进入/退出都走背光渐变）
 *
 * 完整时序（两段渐变夹一段停留）：
 *   1) （2026-08-31 去除）原此处震动一次；现触感由触摸层按下沿统一给，避免震两次；
 *   2) 【进入渐变】背光渐暗 → 暗到底时才 countdown_start()（启动倒计时 + 切运行态
 *      版式）→ 暗态等 flush → 渐亮。切版式全程在黑屏里完成，解决原先「确定后
 *      瞬间切运行态、能看到刷屏」的问题；
 *   3) 渐亮结束后才开始算停留 3 秒（3 次 tick），让用户看清数字在走；
 *   4) 【退出渐变】停留结束 → 背光渐暗 → 暗到底切回功能盘 → 暗态等 flush → 渐亮。
 *
 * 倒计时本体由 reminder 模块托管（reminder_timer_start），不依赖本页面存活，
 * 退出后仍会照常到期提醒；s_cd.tick_tmr 仅负责页面上的秒刷新，下次再进倒计时页时
 * countdown_page_show() 会按 RUNNING 状态重建它。
 * 若不在「设置中」状态（已在跑或已到期），仍走同一套渐变+停留，手感一致。
 */
static void countdown_start_and_exit(void)
{
    if (s_cd_exit_tmr != NULL || s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
        return; /* 已在停留倒数中或渐变进行中，忽略重复长按 */

    /* ★【2026-08-31 问题6】原有的 bsp_motor_pulse() 已删除：会震两次。
     * 头部长按的按下沿在触摸层（bsp_touch.c，番茄钟页属非主界面）已经震过一次，
     * 抬手确认时这里再震就成了第二次。保留触摸层那一次，反馈仍在。
     *
     * ★【2026-08-31 追加：确定改为「动作真正生效才震」】触摸层已把功能页整块排除
     *   （head_vib_view 不含 UI_VIEW_FUNCTION_MENU），本页头部长按已无来自触摸层的
     *   按下沿震动。改由此处发起：上面那道重入检查已通过，说明本次长按确实被受理、
     *   马上要起【进入渐变 + 启动倒计时】；被 return 掉的那次不震。
     *   只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞（BUG-038）。 */
    bsp_touch_request_vibrate();

    /* 触发【进入渐变】：countdown_start() 被推迟到暗态执行（见 fade 回调的
     * UI_CD_EXIT_FADE_OUT 分支），停留计时则推迟到渐亮结束才起（见 FADE_IN 分支）。 */
    if (lvgl_port_lock(100))
    {
        s_cd_fade_is_enter = true;
#if UI_TIME_FADE_DIAG
        /* ★【2026-08-25】本条路径正是实测偶发故障的操作入口（番茄钟设定界面
         * 长按头部"开始+退出"），一并 arm 上 [FD] 诊断，与退出方向配套取证。
         * 打印 state/timer_id：暗态分支要靠 s_cd.state==CD_STATE_SET 才切版式，
         * 出问题那次若状态不符，这里能一眼看出来。 */
        s_fade_diag_armed = true;
        s_fade_diag_in_logged = false;
        ESP_LOGW(TAG, "[FD] ══ 进入倒计时（state=%d timer_id=%d）：渐变开始 ══",
                 (int)s_cd.state, s_cd.timer_id);
#endif
        s_cd_fade_out_ms = UI_CD_EXIT_FADE_OUT_MS; /* 倒计时档 */
        s_cd_fade_dark_ms = UI_CD_EXIT_FADE_DARK_MS;
        s_cd_fade_in_ms = UI_CD_EXIT_FADE_IN_MS;
        s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS; /* 地板归位，同 countdown_exit_tmr_cb */
        s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
        s_cd_exit_fade_start_us = esp_timer_get_time();
        if (s_cd_exit_fade_tmr != NULL)
            lv_timer_resume(s_cd_exit_fade_tmr);
        lvgl_port_unlock();
    }

    /* 渐变 timer 不可用（创建失败等）兜底：退回原来的「立即启动 + 立即起停留计时」，
     * 只是没有渐变遮挡，功能不因动效缺失而丢失。 */
    if (s_cd_exit_fade_tmr == NULL)
    {
        s_cd_fade_is_enter = false;
        s_cd_exit_fade_phase = UI_CD_EXIT_FADE_IDLE;
        if (s_cd.state == CD_STATE_SET)
            countdown_start();
        if (lvgl_port_lock(100))
        {
            s_cd_exit_tmr = lv_timer_create(countdown_exit_tmr_cb, COUNTDOWN_EXIT_DELAY_MS, NULL);
            lvgl_port_unlock();
        }
        if (s_cd_exit_tmr == NULL)
            ui_function_menu_exit();
    }
}

/**
 * @brief 功能页手动退出：用背光渐变遮挡整屏刷新（2026-08-17 新增）
 *
 * 【解决什么问题】原来长按退出是瞬间硬切：ui_function_menu_exit() 一调，整屏
 * 320x240 立刻重画，而绘制 buffer 只有 12800 像素（W*H/6），一屏被拆成 6 条
 * 320x40 逐条推上屏——UI_FLUSH_TRACE 实测每条 7~17ms、合计 60~70ms，亮屏下
 * 就是一道自上而下的横向擦除。
 *
 * 【怎么解决】复用倒计时那套三段状态机（渐暗→暗到底做事→渐亮），把
 * ui_function_menu_exit() 推迟到【暗态】那一刻才执行，整个刷屏过程藏在全黑里。
 * 时长走 UI_FN_EXIT_FADE_* 这档（150/100/200），比倒计时档短，手动返回不发钝。
 *
 * 【为何不用图形层动画】淡入淡出/滑动只会增加重绘量，撕得更厉害——能盖住刷屏
 * 过程的只有关背光这一种。
 *
 * 【线程】由触摸任务调用，内部自取 LVGL 锁；任何一步不可用都兜底硬切，
 * 保证功能不因动效缺失而失效。
 */
static void function_menu_exit_fade(void)
{
    if (s_view != UI_VIEW_FUNCTION_MENU)
        return; /* 不在功能页，无事可做 */

    if (s_cd_exit_fade_tmr == NULL || !lvgl_port_lock(100))
    {
        /* 兜底路径也是真的退出了 → 照样震一次（与下面渐变路径一致）。
         * 见下方渐变分支的完整说明。 */
        bsp_touch_request_vibrate();
        ui_function_menu_exit(); /* 兜底：无 timer / 拿不到锁 → 直接硬切 */
        return;
    }

    /* 已有渐变在跑（倒计时自动退出等）就不打断：它自己会走完并完成退出。 */
    if (s_cd_exit_fade_phase == UI_CD_EXIT_FADE_IDLE)
    {
        /* ★【2026-08-31】震动跟随「真正退出」而非「触摸」。
         * 触摸层已把功能页的头部排除（bsp_touch.c 的 head_vib_view），改由此处发起：
         * 走到这里才表示本次长按真的触发了退出（上面两道检查都已通过：确实在功能页、
         * 且当前没有别的渐变在跑）。落在 else 分支被忽略的那次不会震。
         * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞，此处正持 LVGL 锁，
         * 直接震会阻塞 LVGL 线程（同 BUG-038）。真正打点在触摸任务循环末尾。 */
        bsp_touch_request_vibrate();

        s_cd_fade_is_enter = false; /* 暗态动作 = ui_function_menu_exit() 切回功能盘 */
        s_cd_fade_dark_cb = NULL;   /* NULL = 走默认的 ui_function_menu_exit() */
#if UI_TIME_FADE_DIAG
        /* 【诊断】退出方向：此刻 s_fn_page 仍是当前页，直接判即可。
         *
         * ★【2026-08-25 放宽 arm 条件：由「仅时间页」改为「任意功能页」】★
         * 【为什么】这套 [FD] 诊断当初是为时间页渐变做的，arm 条件写死了
         *   s_fn_page == FN_PAGE_TIME。而实测反馈的偶发故障是
         *   「番茄钟/闹钟设定触发后长按头部退出，当前界面先闪烁一下、隔一小段才退出」——
         *   走的正是本函数这条退出渐变，却因为 arm 条件不成立【一条日志都不打】，
         *   整条路径处在诊断盲区里，几十次才出一次又无从取证。
         * 【改什么】只把 arm 条件放宽到"任意功能页"，四个测点（OUT 实耗 / 暗态动作
         *   阻塞 / DARK 出口 / IN 首帧是否被整段跳过）全部复用，渐变逻辑一行未动。
         * 【为何这四个点够用】6365 那个测点专门判「渐亮被整段跳过＝瞬间亮」，
         *   与"界面闪烁一下"的描述直接对应；若是暗态动作跑空或两套渐变叠加，
         *   也会在 dark_cb 阻塞时长与重复的"渐变开始"行上暴露出来。
         * 【定位完记得关】把 UI_TIME_FADE_DIAG 置 0 即可全部编译剔除。 */
        s_fade_diag_armed = true;
        ESP_LOGW(TAG, "[FD] ══ 退出功能页（page=%d）：渐变开始 ══", (int)s_fn_page);
#endif
        s_cd_fade_out_ms = UI_FN_EXIT_FADE_OUT_MS;
        s_cd_fade_dark_ms = UI_FN_EXIT_FADE_DARK_MS;
        s_cd_fade_in_ms = UI_FN_EXIT_FADE_IN_MS;
        /* 【2026-08-18】退出专属：抬高最小暗场，与进入的实测黑屏时长齐平。
         * 退出画得快（几十毫秒就 flush 完），若仍用 40ms 地板会 43ms 就亮起，
         * 而进入因整屏重绘天然黑 256ms —— 这正是「进入慢、退出快」的来源。
         * 只有这一条路径抬，其余四个入口保持 40ms 不变。 */
        s_cd_fade_dark_min_ms = UI_FN_EXIT_DARK_MIN_MS;
        s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
        s_cd_exit_fade_start_us = esp_timer_get_time();
        lv_timer_resume(s_cd_exit_fade_tmr);
    }
    lvgl_port_unlock();

    /* 同头部弹动分支：渐变 timer 挂进 LVGL 后必须显式唤醒 taskLVGL，
     * 否则要等它睡醒才开始渐暗（实测可达数百 ms，详见头部分支注释）。 */
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
}

/* 游戏退出的暗态动作：停游戏（删面板/引擎 timer）+ 切回功能盘。
 * 两步都会引发大面积重绘，故必须放在全黑期间做。
 * 【线程】由渐变回调在 LVGL 线程调用；jump_stop 等内部虽再取一次 LVGL 锁，
 * 但那是递归互斥量（esp_lcd_port 用 xSemaphoreCreateRecursiveMutex），
 * 同线程重入安全，且内部只做 lv_timer_del/lv_obj_del，无阻塞延时。 */
static void game_exit_dark_action(void)
{
    games_stop();
    back_to_home();
}

/**
 * @brief 游戏退出：同样用背光渐变遮挡刷屏（2026-08-17 新增）
 *
 * 与 function_menu_exit_fade 同一套状态机，只是暗态动作换成
 * 「games_stop() + back_to_home()」。游戏面板删除 + 功能盘重建是整屏级重绘，
 * 亮屏下同样能看到逐条擦除，故一并遮住。
 * 游戏内部逻辑与画面完全不动，只改退出这一下的呈现。
 */
static void game_exit_fade(void)
{
    if (s_cd_exit_fade_tmr == NULL || !lvgl_port_lock(100))
    {
        /* 兜底路径也是真的退出了 → 照样震一次（与下面渐变路径一致）。 */
        bsp_touch_request_vibrate();
        /* 兜底：无 timer / 拿不到锁 → 退回原来的硬切，功能不因动效缺失而失效 */
        games_stop();
        back_to_home();
        return;
    }

    if (s_cd_exit_fade_phase == UI_CD_EXIT_FADE_IDLE)
    {
        /* ★【2026-08-31】震动跟随「真正退出」而非「触摸」。
         * 触摸层已把游戏视图的头部排除（bsp_touch.c 的 head_vib_view），改由此处发起：
         * 走到这里才表示本次长按真的触发了退出（当前没有别的渐变在跑）。
         * 落在 else（已有渐变在跑）被忽略的那次不会震。
         * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞，此处正持 LVGL 锁，
         * 直接震会阻塞 LVGL 线程（同 BUG-038）。真正打点在触摸任务循环末尾。 */
        bsp_touch_request_vibrate();
        s_cd_fade_is_enter = false;
        s_cd_fade_dark_cb = game_exit_dark_action;
        s_cd_fade_out_ms = UI_FN_EXIT_FADE_OUT_MS;
        s_cd_fade_dark_ms = UI_FN_EXIT_FADE_DARK_MS;
        s_cd_fade_in_ms = UI_FN_EXIT_FADE_IN_MS;
        s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS; /* 游戏退出维持原速，不跟功能页退出一起减速 */
        s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
        s_cd_exit_fade_start_us = esp_timer_get_time();
        lv_timer_resume(s_cd_exit_fade_tmr);
    }
    lvgl_port_unlock();
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-18 新增】功能盘 → 功能页「进入」渐变
 *
 * 四段式：图标压扁抬回结束 → 渐暗到全黑 → 黑屏期画目标页 → 渐亮。
 * 目标页的整屏重绘全部发生在暗态，用户看不到分块刷屏（斜线/横带都被吃掉）。
 *
 * 【复用而非新写】直接挂到 game_exit_fade 用的同一套状态机上，只是把暗态动作
 * 换成目标页的 on_enter 回调、时长换成 UI_FN_ENTER_FADE_* 这档。
 * 状态机本身（countdown_exit_fade_timer_cb）一行未改，只在其暗态分支加了
 * 「回调用完即清空」的安全归位，见该处注释。
 *
 * 【线程】由 home_press_phase2_ready（lv_anim 的 ready 回调）在 LVGL 线程调用。
 * lvgl_port_lock 是递归互斥量，同线程重入安全（与 game_exit_dark_action 同理）。
 *
 * 【游戏项不走这里】赛车/打地鼠/跳一跳的入场动画（3 秒开场进度条、台子从天而降）
 * 若在暗态启动会被黑屏吃掉开头，取舍未定，故按用户 2026-08-18 指定暂不接入，
 * 保持原有的「直接调 on_enter」行为，游戏侧代码一行未动。
 *
 * @param on_enter 暗态那一刻要执行的进入动作（app_enter_time 等）
 */
static void home_enter_fade(void (*on_enter)(void))
{
    if (on_enter == NULL)
        return;

    /* 兜底：无 timer / 拿不到锁 / 已有渐变在跑 → 直接硬切，
     * 保证功能永不因动效缺失而失效（与 function_menu_exit_fade 同策略）。 */
    if (s_cd_exit_fade_tmr == NULL || !lvgl_port_lock(100))
    {
        on_enter();
        return;
    }

    if (s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
    {
        lvgl_port_unlock();
        on_enter(); /* 已有渐变在跑：不打断它，本次退化为硬切 */
        return;
    }

    s_cd_fade_is_enter = false; /* 非倒计时「进入运行态」语义，走 dark_cb 分支 */
    s_cd_fade_dark_cb = on_enter;
#if UI_TIME_FADE_DIAG
    /* 【诊断】只在目标是时间页时置起标志。此刻 s_fn_page 尚未切到 FN_PAGE_TIME
     * （要到暗态里 menu_enter_fn_page 才置位），故只能按 on_enter 函数指针判定。 */
    s_fade_diag_armed = (on_enter == app_enter_time);
    if (s_fade_diag_armed)
        ESP_LOGW(TAG, "[FD] ══ 进入时间页：渐变开始 ══");
#endif
    s_cd_fade_out_ms = UI_FN_ENTER_FADE_OUT_MS;
    s_cd_fade_dark_ms = UI_FN_ENTER_FADE_DARK_MS;
    s_cd_fade_in_ms = UI_FN_ENTER_FADE_IN_MS;
    /* 地板归位成默认值：本变量是「上一次谁设的就留着」的语义，
     * 若不复位，紧接在一次功能页退出之后的进入会捡到 240ms 的地板，
     * 白白多黑一段（进入本就被重绘拖到 256ms，不需要额外地板）。 */
    s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS;
    s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
    s_cd_exit_fade_start_us = esp_timer_get_time();
    lv_timer_resume(s_cd_exit_fade_tmr);
    lvgl_port_unlock();

    /* 与其它渐变入口一致：挂进 LVGL 后显式唤醒 taskLVGL，
     * 否则要等它自然睡醒才开始渐暗（实测可达数百 ms）。 */
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
}

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-19 新增】功能盘 →「游戏」进入渐变
 *
 * 与 home_enter_fade() 完全同一套状态机、同一个 dark_cb 机制，只有时长档不同
 * （走 UI_GAME_ENTER_FADE_*，理由见该组宏上方注释）。之所以另起一个函数而非
 * 给 home_enter_fade 加参数：后者有 5 个既有调用方（app_enter_time 等），
 * 加参数要动全部调用点；本函数只是把三行时长换掉，复制的代价小于改签名的风险。
 *
 * 【暗态动作是什么】game_enter_dark_action_*（三个无参包装），内部执行原来
 * enter_game_common() 里那套「切 s_view + games_start(id) + 停空闲计时」。
 * 关键：s_view 的切换必须跟 games_start 一起放进暗态，不能留在触摸那一刻——
 * 否则渐变那 290ms 里 s_view 仍是 UI_VIEW_HOME，期间来一次触摸会被
 * ui_dispatch_touch_event 当成功能盘操作分发（翻页/再次进游戏），
 * 而画面其实已经在切游戏了。这是本次接入唯一的真陷阱。
 *
 * @param on_enter 暗态那一刻要执行的进入动作（三个 game_enter_dark_action_*）
 * ══════════════════════════════════════════════════════════════════════════ */
static void game_enter_fade(void (*on_enter)(void))
{
    if (on_enter == NULL)
        return;

    /* 兜底：无 timer / 拿不到锁 / 已有渐变在跑 → 直接硬切，
     * 保证功能永不因动效缺失而失效（与 home_enter_fade 同策略）。 */
    if (s_cd_exit_fade_tmr == NULL || !lvgl_port_lock(100))
    {
        on_enter();
        return;
    }

    if (s_cd_exit_fade_phase != UI_CD_EXIT_FADE_IDLE)
    {
        lvgl_port_unlock();
        on_enter(); /* 已有渐变在跑：不打断它，本次退化为硬切 */
        return;
    }

    s_cd_fade_is_enter = false; /* 非倒计时「进入运行态」语义，走 dark_cb 分支 */
    s_cd_fade_dark_cb = on_enter;
    s_cd_fade_out_ms = UI_GAME_ENTER_FADE_OUT_MS;
    s_cd_fade_dark_ms = UI_GAME_ENTER_FADE_DARK_MS;
    s_cd_fade_in_ms = UI_GAME_ENTER_FADE_IN_MS;
    /* 地板归位成默认值：本变量是「上一次谁设的就留着」的语义，
     * 若捡到功能页退出档设的加长地板，会白白多黑一段，把入场动画多吃掉几帧。 */
    s_cd_fade_dark_min_ms = UI_FADE_DARK_MIN_MS;
    s_cd_exit_fade_phase = UI_CD_EXIT_FADE_OUT;
    s_cd_exit_fade_start_us = esp_timer_get_time();
    lv_timer_resume(s_cd_exit_fade_tmr);
    lvgl_port_unlock();

    /* 与其它渐变入口一致：挂进 LVGL 后显式唤醒 taskLVGL，
     * 否则要等它自然睡醒才开始渐暗（实测可达数百 ms）。 */
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
}

static void countdown_cancel(void)
{
    if (s_cd.timer_id >= 0)
    {
        reminder_timer_cancel(s_cd.timer_id);
        s_cd.timer_id = -1;
    }
    s_cd.state = CD_STATE_SET;
    if (lvgl_port_lock(100))
    {
        countdown_expire_reset_cancel(); /* 2026-08-10：手动取消时干掉到期复位定时器 */
        if (s_cd.tick_tmr != NULL)
        {
            lv_timer_pause(s_cd.tick_tmr);
            lv_timer_del(s_cd.tick_tmr);
            s_cd.tick_tmr = NULL;
        }
        countdown_page_render();
        lvgl_port_unlock();
    }
    ESP_LOGI(TAG, "倒计时已取消");
}

/**
 * @brief 倒计时设置：对当前编辑字段的某一位 +1（进位在本位内循环，不影响另一位）
 *
 * 2026-08-04 需求：改为「左耳调十位、右耳调个位」，比原来 ±2 步进按到目标值快得多。
 * 当前编辑的是分钟还是秒钟，仍由 s_cd.editing_sec 决定（头部短按切换）。
 *
 * 【本位循环语义】十位 9→0、个位 9→0，只在本位回绕，不向另一位进位。
 *   这样按十位不会意外改掉个位，用户能把两位当成两个独立拨盘，符合"左十右个"的直觉。
 *
 * 【上限裁剪】分钟上限 CD_MIN_MAX(60)、秒钟上限 CD_SEC_MAX(58)，
 *   本位 +1 后若超出上限则该位归 0（而非停在上限），保证任何按法都落在合法区间。
 *
 * @param is_tens true=十位+1（左耳），false=个位+1（右耳）
 */
static void countdown_bump_digit(bool is_tens)
{
    uint8_t *field = s_cd.editing_sec ? &s_cd.seconds : &s_cd.minutes;
    const int max = s_cd.editing_sec ? CD_SEC_MAX : CD_MIN_MAX;

    int tens = *field / 10;
    int units = *field % 10;

    if (is_tens)
        tens = (tens + 1) % 10;
    else
        units = (units + 1) % 10;

    int val = tens * 10 + units;
    if (val > max)
        val = is_tens ? units : tens * 10; /* 超上限：把刚加的那一位归 0 */

    *field = (uint8_t)val;
}

/**
 * @brief 头部短按：切换到下一个时长选项（2026-08-10 番茄时钟）
 *
 * 【循环顺序】1min → 3 → 5 → 10 → 15 → 30 → 自定义 → 回到 1min，共 CD_PRESET_COUNT+1 档。
 *
 * 【与原「分 ↔ 秒」切换的关系】原来头部短按用于切换自定义时间的分/秒编辑字段，
 * 现在头部短按被「切档」占用。为保住需求里的「自定义逻辑复用、不进行改变」，
 * 采用两级语义：
 *   · 停留在自定义档时，头部短按先在 分 → 秒 之间走一轮，
 *     再按一次才离开自定义档回到第 1 个预设。
 *   · 于是自定义档内的分/秒切换手感、左十右个调值全部原样保留，
 *     且 7 档循环依然只用头部短按一个按键即可走遍。
 * 完整一轮：1 → 3 → 5 → 10 → 15 → 30 → 自定义(编辑分) → 自定义(编辑秒) → 1 → …
 */
static void countdown_sel_advance(void)
{
    if (s_cd.sel == CD_SEL_CUSTOM)
    {
        /* 已在自定义档：先切分/秒；若刚编辑完秒（一轮走完），则退出自定义回到首个预设 */
        if (!s_cd.editing_sec)
        {
            s_cd.editing_sec = true; /* 分 → 秒 */
        }
        else
        {
            s_cd.editing_sec = false; /* 复位字段，下次进自定义仍从「分」开始 */
            s_cd.sel = 0;             /* 秒 → 离开自定义，回到 1min */
        }
    }
    else if (s_cd.sel + 1 < CD_PRESET_COUNT)
    {
        s_cd.sel++; /* 预设之间前进 */
    }
    else
    {
        s_cd.sel = CD_SEL_CUSTOM; /* 最后一个预设 → 自定义 */
        s_cd.editing_sec = false; /* 进自定义默认先编辑「分」 */
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 功能页面路由
 * ═══════════════════════════════════════════════════════════════ */
static void time_page_render_text(void)
{
    if (s_menu_body == NULL)
        return;

    if (!reminder_is_time_synced())
    {
        lv_label_set_text(s_menu_body, "等待时间同步...");
        return;
    }

    time_t now = time(NULL);
    struct tm tm_now;
    localtime_r(&now, &tm_now);
    static const char *const wday_cn[] = {
        "星期日", "星期一", "星期二", "星期三", "星期四", "星期五", "星期六"};

    char buf[384];
    int off = snprintf(buf, sizeof(buf),
                       "%04d年%02d月%02d日\n%s\n%02d:%02d:%02d\n\n中国标准时间",
                       tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday,
                       wday_cn[tm_now.tm_wday & 0x7],
                       tm_now.tm_hour, tm_now.tm_min, tm_now.tm_sec);

    calendar_entry_t cal_list[REMINDER_MAX_CALENDARS];
    uint8_t cal_count = 0;
    reminder_calendar_get_today(cal_list, &cal_count);

    if (cal_count > 0)
    {
        off += snprintf(buf + off, sizeof(buf) - off, "\n\n今日日程:");
        for (uint8_t i = 0; i < cal_count && i < 3; i++)
        {
            if (off >= (int)sizeof(buf) - 16)
                break;
            off += snprintf(buf + off, sizeof(buf) - off, "\n  %02d:%02d %s",
                            cal_list[i].hour, cal_list[i].minute, cal_list[i].message);
        }
    }
    else
    {
        off += snprintf(buf + off, sizeof(buf) - off, "\n\n今日无日程");
    }
    lv_label_set_text(s_menu_body, buf);
}

/* ── 【临时诊断，2026-08-19】功能页内部 SRAM 占用探针 ──────────────────────
 *
 * 【要回答的问题】开机 46KB 的内部 SRAM，到唤醒前只剩 16.8KB（净掉 30764 B），
 *   这一段发生在"把功能盘 8 项挨个点了一遍"的过程中。已确认的机制是
 *   sdkconfig 的 CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=512 把所有 <512B 的
 *   LVGL 分配钉在内部 SRAM，且功能页只 hide 不 delete。但按对象数×实测
 *   sizeof(lv_obj_t)=52~56B 估算只能解释约 12KB，剩下 18KB 无证据。
 *
 * 【判读】每页打两条（enter 前 / 渲染后），差值即该页真实成本：
 *   ① 首次进入掉 N、之后每次进入掉 ≈0  → 一次性占用，机制坐实，
 *      可以放心改 ALWAYSINTERNAL 把这些分配赶去 PSRAM；
 *   ② 每次进入都掉                      → 真泄漏，必须先揪出来再谈优化；
 *   ③ 某一页掉得远超其对象数            → 该页有对象之外的分配
 *      （天气页要重点看：它带 HTTPS + mbedTLS，t=99008 那次请求可疑）。
 *
 * 同时打 largest 连续块：总量够但连续块碎掉一样会让会话建不起来，
 * 这个指标比 free 更能预警"内存够却分配失败"。
 *
 * ★ 临时代码，定案后整块删除。UI_MEM_PROBE 宏在文件前部前向声明处定义，
 *   置 0 即可一键关闭本探针的全部代码。 */
#if UI_MEM_PROBE
static void ui_mem_probe(const char *stage, const char *page_name)
{
    ESP_LOGW(TAG, "[MEMPROBE] %-10s %-8s 内部SRAM free=%d largest=%d | PSRAM free=%d",
             stage, page_name,
             (int)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (int)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
             (int)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
}
// #else
// #define ui_mem_probe(stage, page_name) ((void)0)
#endif

static void render_fn_page(fn_page_t page)
{
    if (s_menu_title == NULL || s_menu_body == NULL)
        return;

    /* 【临时诊断】渲染前基线：与函数末尾的 after 配对，差值 = 本页新增占用 */
    ui_mem_probe("before", s_fn_page_titles[page]);

    alarm_page_hide();
    countdown_page_hide();
    weather_page_hide();

    /* 进入功能页时隐藏功能盘的大图标（避免残留遮挡） */
    if (s_home_icon)
        lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);

    lv_label_set_text(s_menu_title, s_fn_page_titles[page]);

#if CONFIG_UI_USE_CALENDAR
    /* 时间页与日历页现在是两个独立页面，进任何页前都先把这两页整组隐藏，
     * 否则会残留到其他功能页（它们共用同一个 s_menu_panel）。 */
    time_page_hide();
    calendar_page_hide();
#endif

    lv_label_set_text(s_menu_title, s_fn_page_titles[page]);

    /* 2026-08-10 需求：功能页顶部标题只保留倒计时页的"番茄时钟"。
     * 时间/日历/闹钟/天气四页内容本身已能表达页面身份，标题纯属占地方，
     * 且天气页要按示例图做「左图标 + 右信息」满屏布局，顶部标题会挤压版面。
     * 这里只隐藏标签对象，不清空 s_fn_page_titles[] —— 那张表还被
     * ESP_LOGI("进入功能页: %s") 用于日志，清空会让日志变空字符串。 */
    if (page == FN_PAGE_COUNTDOWN)
        lv_obj_clear_flag(s_menu_title, LV_OBJ_FLAG_HIDDEN);
    else
        lv_obj_add_flag(s_menu_title, LV_OBJ_FLAG_HIDDEN);

    switch (page)
    {
    case FN_PAGE_TIME:
#if CONFIG_UI_USE_CALENDAR
        // 模式 A: 专属大号时间页，隐藏共享文字标签
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        time_page_render();
#else
        // 模式 B: 回退到原始逻辑，显示文字标签
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        time_page_render_text();
#endif
        break;

    case FN_PAGE_CALENDAR:
#if CONFIG_UI_USE_CALENDAR
        // 模式 A: 专属点阵日历页，隐藏共享文字标签
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        calendar_page_render();
#else
        // 模式 B: 回退到原始逻辑（文字版日期/日程）
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        time_page_render_text();
#endif
        break;

    case FN_PAGE_ALARM:
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        alarm_page_show();
        break;

    case FN_PAGE_WEATHER:
        /* 天气页改用专属多 label 布局（参考图上半部风格），隐藏共享文字 label */
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        weather_page_show();
        break;

    case FN_PAGE_COUNTDOWN:
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        countdown_page_show();
        break;

    default:
        lv_label_set_text(s_menu_body, "");
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        break;
    }

    /* 【临时诊断】渲染后：与函数开头的 before 相减 = 本页这次进入的净占用。
     * 首次进入含"建对象 + 懒加载外挂字体"，第二次起理论上应接近 0。 */
    ui_mem_probe("after", s_fn_page_titles[page]);
}

/* ═══════════════════════════════════════════════════════════════
 * 功能菜单对外接口
 * ═══════════════════════════════════════════════════════════════ */
void ui_function_menu_enter(void)
{
    if (!lvgl_port_lock(100))
        return;
    ensure_menu_panel();
    if (s_view != UI_VIEW_FUNCTION_MENU)
    {
        s_view = UI_VIEW_FUNCTION_MENU;
        s_fn_page = FN_PAGE_TIME;
        ESP_LOGI(TAG, "进入功能菜单");
    }
    /* 2026-08-16：进功能层必须 pause GIF，仅 HIDDEN 是不够的 ——
     * GIF 内部逐帧解码 timer 仍在跑，持续读 flash + 解码 + 刷新，稳定抢占 LVGL 线程；
     * 播完一轮还会发 LV_EVENT_READY 把 s_gif_switch_tmr 唤醒，形成
     * 「ready → resume → 切图」自我唤醒环（机制详见 ui_show_ota_progress 里那段注释）。
     * 实测后果：倒计时页 1s tick 被随机延迟 1~3 秒，数字停在 01:00 好几秒后
     * 直接跳到 00:57，中间几次刷新全丢——因偶发、卡多久取决于当前 GIF 帧的解码耗时。
     * 与 back_to_home / ui_func_layer_exit_to_main 里的 resume 成对。 */
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj); /* 冻结内部解码 timer，让出 LVGL 线程 */
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }
    render_fn_page(s_fn_page);
    lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    menu_kick_idle_timer();
    lvgl_port_unlock();
}

void ui_function_menu_exit(void)
{
    /* 单层结构：功能页是从功能盘进入的，退出回到功能盘（而非直接回主界面）。
     * 面板保持显示，仅切换内容。 */
    if (s_view == UI_VIEW_FUNCTION_MENU)
    {
        back_to_home();
        ESP_LOGI(TAG, "退出功能页，返回功能盘");
    }
}

/**
 * @brief 倒计时到期：强制把界面切到番茄时钟页并显示到期画面（2026-08-10 新增）
 *
 * 【解决什么问题】原实现里倒计时由 reminder 模块后台托管，到期只震动、不碰屏幕；
 * 而 countdown_start_and_exit() 启动 2 秒后就退出了功能页，于是到期那一刻用户
 * 通常并不在番茄钟页上，`countdown_page_render()` 全打在已 hidden 的对象上，
 * 屏幕纹丝不动 —— 实测现象＝「日志显示到期了，屏幕没有任何变化」。
 *
 * 【本函数做什么】无论当前在主界面、功能盘还是别的功能页，一律：
 *   1) 切到功能菜单视图 + 番茄时钟页（s_fn_page = FN_PAGE_COUNTDOWN）
 *   2) 置 CD_STATE_EXPIRED 并渲染（红色 00:00 + 「倒计时结束」）
 *   3) 挂上 CD_EXPIRE_HOLD_MS 自动复位定时器，到点回选择界面
 * 不碰 GIF、不碰舵机 —— 按需求这是最小的一件事，其余留待后续。
 *
 * 【线程】由 reminder_task 调用（非 LVGL 线程），故内部必须自己 lvgl_port_lock。
 * 拿不到锁就直接返回：宁可这次不切页，也不能阻塞 reminder 任务拖延后续提醒判定。
 */
void ui_show_countdown_expired(void)
{
    /* ★★【2026-08-25 新增：到期必须先完整退出低功耗】★★
     * 【修的问题】本函数原先完全不看低功耗状态，深度待机中到期时：
     *   ① 背光只有 STANDBY_CLOCK_BK_PCT（2%），到期画面等于看不见；
     *   ② 待机时钟（时间页）还叠在上面，s_standby_clock_on 也不会被复位，
     *      下一次跨分钟 main_clock_tick_cb 会把时间页重新画上来 → 两页同框；
     *   ③ s_deep_standby 仍为 true，用户此后第一次触摸会被
     *      ui_dispatch_touch_event 的「待机唤醒消费掉本次事件」分支吃掉。
     *
     * 【为何必须放在 lvgl_port_lock 之前】standby_wake() 内部会调
     *   ui_standby_clock_hide() / ui_force_back_to_main() / ui_resume_main_gif_loop()，
     *   这几个都要自取 LVGL 锁。若本函数先持锁再调它，就是【自己等自己】的死锁。
     *
     * ★★【2026-08-25 二次修正：唤醒动作已前移到 reminder.c 的 poll 层】★★
     * 【踩过的坑】本函数最初直接在这里调 standby_wake() + standby_notify_activity()，
     *   实测【必崩】：
     *     assert failed: esp_task_stack_is_sane_cache_disabled() @ cache_utils.c:152
     *     Backtrace: reminder_task → on_reminder_trigger → ui_show_countdown_expired
     *                → standby_wake → bsp_board_codec_exit_lowpower → nvs_get_i32
     *                → spi_flash_disable_interrupts_caches_and_other_cpu
     *   栈指针 0x3c8c0d10 落在 0x3C 段（PSRAM）即铁证。
     *   根因：本函数由 reminder_task 调用，而【reminder_task 的栈在 PSRAM】
     *   （见 reminder.c 中 nvs_save_weather_data 处那条铁律）；standby_wake 内部
     *   读 NVS 恢复音量要关主 flash cache，关 cache 后 PSRAM 一并失联 → 栈读不到 →
     *   IDF 主动断言拦下。standby_notify_activity() 内部同样会调 standby_wake，
     *   一样危险，故两者都不能出现在这里。
     *
     * 【现在的分工】待机唤醒改由 reminder.c 的 poll_timer_callback 处理：它发现
     *   到期时正处于深度待机，就调 standby_request_wake()（纯原子置位，PSRAM 栈
     *   安全）并【本拍不投递】，等 standby_task（内部 SRAM 栈）完成唤醒后，下一拍
     *   再正常投递事件走到这里。因此本函数被调用时【必然已不在待机】。
     *
     * 下面这段只作为纵深防御：万一仍在待机（时序异常），宁可放弃本次切页，
     * 也绝不留下"待机时钟 + 功能页"的叠加死状态——震动提醒不受影响，不会丢。 */
    if (standby_is_deep_active())
    {
        ESP_LOGW(TAG, "到期切页时仍处于深度待机（异常时序），放弃本次切页，仅保留震动提醒");
        return;
    }

    /* ★【2026-08-25 新增：闹钟优先，响铃期间倒计时不抢屏】★
     * 【修的问题】闹钟与倒计时同一秒到期时，两个事件先后进同一个队列，
     *   reminder_task 先处理 TIMER_EXPIRE（切番茄钟页 + 震动 3 秒，阻塞本任务），
     *   3 秒后才处理 ALARM_TRIGGER 把闹钟页切上来 —— 用户看到的就是
     *   「倒计时只有震动没有画面（其实是被闹钟顶掉了），闹钟才有画面」。
     * 【为什么定闹钟优先】闹钟是用户设定的【绝对时刻】提醒，且有持续响铃周期
     *   （最多 4 次、十几秒）；倒计时是相对时长，到期画面只停留 5 秒。
     *   让短的让位于长的、相对的让位于绝对的，冲突时语义损失最小。
     * 【倒计时并不会丢】它的 3 秒长震照常执行（震动在 reminder_task 里，不经过本函数），
     *   用户仍能感知到"番茄钟也到了"，只是这一次不占屏。
     *
     * ★★【2026-08-25 已废弃并移除本拦截，改为在 reminder 层排队】★★
     * 上面这套做法是【丢弃画面】而不是【推迟提醒】，实测有三个后果：
     *   ① 走到本函数时事件早已投递，震动照常长震 3 秒，与闹钟震动混叠成一片；
     *   ② 画面被拦掉后【再也不补】，这一次番茄钟等于白算；
     *   ③ s_cd.state 残留在 EXPIRED，用户下次主动进倒计时页会先看到"结束界面"、
     *      几秒后才跳回设定界面（即实测反馈的第 9 条）。
     * 现已把守卫上移到 reminder.c 的 poll_timer_callback：闹钟响铃期间整个提醒
     * 【不投递、也不推进任何状态】，等闹钟响完下一拍再完整触发（震动 + 画面一起来）。
     * 因此这里不再需要任何拦截——留着反而会与上游的排队语义打架，故删除。 */

    /* 【兜底】唤醒没走成（standby_wake 内部取锁超时等）时，待机时钟仍然挂着。
     * 此时若继续切页，就会留下「待机时钟 + 番茄钟页」的叠加死状态——正是本次
     * 要根治的现象。宁可这一次不显示画面（reminder 侧的 3 秒长震照常执行，
     * 提醒不会丢），也绝不制造叠加态。
     *
     * ★【2026-09-01】本判断必须留在【发起渐变之前】，不能挪进暗态动作：
     *   渐暗都跑完 450ms 了才发现要放弃，用户会看到"屏幕无缘无故黑一下又亮回来"。
     *   s_standby_clock_on 是简单标量读，无锁读取足够安全（同下方 s_view 的读法）。 */
    if (s_standby_clock_on)
    {
        ESP_LOGW(TAG, "待机时钟仍在显示（唤醒未完成），放弃本次到期切页，仅保留震动提醒");
        return;
    }

    /* ★【2026-09-01】到期渐变已在路上：直接返回，别再发起第二次。
     * 理由与 ui_show_alarm_ringing 内的对应判断相同（那里有完整说明）。
     * 本入口虽不像响铃那样天然重复，但闹钟与倒计时同秒到期等场景仍可能撞上。 */
    if (s_expire_fade_pending)
        return;

    /* ★【2026-09-01 需求改造：硬切 → 背光渐变 + 舵机同步归中】★
     * 【改前】直接把到期画面盖在当前 GIF 上（整屏 6 条逐条刷，亮屏下看得见擦除），
     *   舵机则完全不管，手上那个空闲/情绪动作会【跑完整段】才停。
     * 【改后】渐暗（期间舵机归中）→ 黑屏里切页 → 渐亮。
     * quiet_main_screen=true：本切换是从主界面过来的，需要停轮播 + 归中。
     * 渐变不可用时（无 timer / 已有渐变在跑 / 取锁失败）退化为原来的硬切，
     * 功能绝不因动效缺失而丢失。 */
    if (expire_fade_start(countdown_expired_dark_action, /*quiet_main_screen=*/true,
                          "主界面→番茄钟到期"))
        return;

    ESP_LOGW(TAG, "到期渐变不可用，退化为硬切");
    if (!lvgl_port_lock(500))
    {
        ESP_LOGW(TAG, "倒计时到期切页失败：拿不到 LVGL 锁");
        return;
    }
    countdown_expired_dark_action();
    lvgl_port_unlock();
}

/**
 * @brief 番茄钟到期的【暗态切页动作】：主界面 → 到期画面
 *
 * 原样搬自 ui_show_countdown_expired() 的切页段落，逻辑一行未改，只是执行时机
 * 从"触发那一刻"推迟到了"背光渐暗到全黑之后"，整屏重绘全程藏在黑屏里。
 *
 * 【线程】由 countdown_exit_fade_timer_cb 在 LVGL 线程调用（已持锁）；
 * 硬切兜底路径下由 ui_show_countdown_expired 在 reminder_task 持锁后调用。
 * 两条路径都已持锁，故内部不再自取锁。
 */
static void countdown_expired_dark_action(void)
{
    ensure_menu_panel();

    /* ★★【2026-08-25 修复：必须显式压掉闹钟编辑面板，否则本页被它整个盖住】★★
     *
     * 【实测现象】"闹钟响完停在设定界面，之后触摸左右耳有震动但画面纹丝不动，
     *   一直到进低功耗强制切换才恢复" —— 看着像卡死，其实【一点没卡】：
     *   日志里触摸事件全部正常进来（BSP_TOUCH: 触摸事件: 9/8/1, 当前视图: 3），
     *   分发与渲染也全部照常执行，只是用户看不见。
     *
     * 【根因】s_edit_panel 是【独立挂在 active screen 上】的一块全屏面板
     *   （创建处：lv_obj_create(scr) + 全屏尺寸 + 黑底 + LV_OPA_COVER），
     *   与 s_menu_panel 平级，两者显隐互不影响。而闹钟响铃结束会自动进入闹钟
     *   编辑页（alarm_ring_reset_cb → alarm_edit_enter_ex）把它显示出来。
     *   此后本函数强制切到番茄钟页时只操作了 s_menu_panel 那一侧，于是番茄钟页
     *   确实被正确渲染，却被上面这块不透明黑底面板完全遮住。
     *
     * 【为何"进低功耗就好了"】enter_deep_standby() → ui_force_back_to_main() 里
     *   恰好有一句 lv_obj_add_flag(s_edit_panel, HIDDEN)，一执行遮挡就消失——
     *   与实测描述完全吻合，是本定位最有力的旁证。
     *
     * 【为何放这里】必须在下方 clear_flag(s_menu_panel) 之前压掉。本文件另外 4 处
     *   隐藏 s_edit_panel 的写法与此完全一致，此处只是补齐"强制切页"这个遗漏入口。 */
    if (s_edit_panel != NULL)
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);

    /* 强制进入功能菜单 + 番茄时钟页。这里直接改 s_view/s_fn_page 而不调
     * ui_function_menu_enter()，因为后者会把页面重置成 FN_PAGE_TIME。 */
    s_view = UI_VIEW_FUNCTION_MENU;
    s_fn_page = FN_PAGE_COUNTDOWN;

    /* 2026-08-16：同 ui_function_menu_enter —— 仅 HIDDEN 不停解码，必须 pause，
     * 否则 GIF 内部 timer 继续抢 LVGL 线程（详见那里的注释）。 */
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj);
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }

    /* ⚠️ 状态必须在 render_fn_page 之【后】置。
     * render_fn_page() 开头会调 countdown_page_hide()，而后者为了不让到期复位
     * 定时器在别的页面上乱改状态，会把 EXPIRED 打回 SET（见该函数内注释）。
     * 若先置 EXPIRED 再 render，状态会被这一步清掉，画出来的是设置界面。 */
    render_fn_page(s_fn_page);

    s_cd.state = CD_STATE_EXPIRED;
    countdown_page_render(); /* 用到期态重画一次：红色 00:00 +「倒计时结束!」 */

    lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);

    /* 挂自动复位定时器：CD_EXPIRE_HOLD_MS 后回到选择界面。
     * countdown_page_hide() 里已把上一轮的定时器取消掉，此处必为 NULL，
     * 仍判一次保持幂等，保证不会停在无法操作的到期界面上。 */
    if (s_cd_expire_tmr == NULL)
    {
        s_cd_expire_tmr = lv_timer_create(countdown_expire_reset_cb, CD_EXPIRE_HOLD_MS, NULL);
        if (s_cd_expire_tmr == NULL)
        {
            s_cd.state = CD_STATE_SET; /* 创建失败兜底，见 countdown_enter_expired */
            countdown_page_render();
        }
    }

    menu_kick_idle_timer(); /* 刷新熄屏计时，别让到期画面刚显示就被熄掉 */

    /* 【2026-09-01】暗态动作已完成，释放"渐变已发起"窗口，允许后续到期请求再发起。
     * 注意 CD_EXPIRE_HOLD_MS(5s) 的停留计时是从【暗态】起算的，而画面还要再过
     * DARK(100)+IN(550)=650ms 才完全亮起，故用户实际看到的到期画面约 4.35s。
     * 不把它推迟到渐亮收尾，是因为那要动 countdown_exit_fade_timer_cb 的 FADE_IN
     * 分支（那段只服务 s_cd_fade_is_enter 语义，注释里全是踩坑记录，不值得为
     * 0.65s 去动它）。 */
    s_expire_fade_pending = false;
    ESP_LOGI(TAG, "倒计时到期：已切到番茄时钟页显示到期画面");
}

/**
 * @brief 闹钟响铃：强制把界面切到闹钟页并显示响铃画面（2026-08-10 新增）
 *
 * 与 ui_show_countdown_expired() 完全同构，只是目标页换成 FN_PAGE_ALARM、
 * 提示文字换成「闹钟响铃」。同样不碰 GIF、不操作舵机。
 *
 * 【线程】由 reminder_task 调用（非 LVGL 线程），内部自行 lvgl_port_lock；
 * 拿不到锁直接返回，不阻塞 reminder 任务。
 */
void ui_show_alarm_ringing(void)
{
    /* 【重复调用早退】响铃回调是重复事件（每次响铃都回调一次），且其中一个
     * 调用点 reminder.c:980 持有 s_ctx.mutex。这里在**拿 LVGL 锁之前**先判：
     * 已经在响铃态且页面就停在闹钟页，说明画面已经是对的，直接返回。
     * 这样重复响铃不会反复阻塞在 lvgl_port_lock(100) 上，避免把 reminder
     * 的互斥锁按住 100ms 拖慢其它提醒的到期判定。
     * s_alarm_ringing / s_fn_page 都是简单标量读，无锁读取足够安全。 */
    if (s_alarm_ringing && s_view == UI_VIEW_FUNCTION_MENU && s_fn_page == FN_PAGE_ALARM)
        return;

    /* ★【2026-09-01 配套：补上上面那道判据在渐变期的盲区】★
     * 改走渐变后，s_alarm_ringing 要到【暗态】才置位（见 alarm_ringing_dark_action），
     * 于是渐暗那 450ms 里上面的判据恒不成立。若此时来了下一次响铃回调，就会重复
     * 发起 → expire_fade_start 因"已有渐变在跑"返回 false → 退化硬切，把正在跑的
     * 渐变整个作废。本判断把这段窗口补上：画面切换已经在路上了，直接返回即可。
     * 【只有本入口和番茄钟到期入口需要】退出方向（响铃→编辑）绝不能这样吞掉，
     * 否则会永远停在响铃画面上，故不做进 expire_fade_start。 */
    if (s_expire_fade_pending)
        return;

    /* ★【2026-08-25：与 ui_show_countdown_expired 同构的纵深防御】★
     * 同样【绝不可】在此调 standby_wake()/standby_notify_activity()——本函数也由
     * reminder_task（PSRAM 栈）调用，会踩"关 cache 后栈失联"的断言。完整踩坑
     * 记录见 ui_show_countdown_expired() 内的对应注释。
     * 唤醒已由 reminder.c 的 poll 层用 standby_request_wake() 提前完成，
     * 走到这里必然已不在待机；仍在待机说明时序异常，放弃切页只保留震动。 */
    if (standby_is_deep_active())
    {
        ESP_LOGW(TAG, "响铃切页时仍处于深度待机（异常时序），放弃本次切页，仅保留震动提醒");
        return;
    }

    /* 兜底：唤醒未完成时放弃切页，只保留震动，绝不留「待机时钟 + 闹钟页」叠加态。
     * 【2026-09-01】同 ui_show_countdown_expired：本判断必须留在发起渐变【之前】，
     * 否则会出现"黑一下又亮回来"。 */
    if (s_standby_clock_on)
    {
        ESP_LOGW(TAG, "待机时钟仍在显示（唤醒未完成），放弃本次响铃切页，仅保留震动提醒");
        return;
    }

    /* ★【2026-09-01 需求改造：硬切 → 背光渐变 + 舵机同步归中】★
     * 说明与番茄钟到期完全同构，见 ui_show_countdown_expired 内的对应注释。
     * 【本入口的额外注意】闹钟响铃是重复事件，本函数会被反复调用；渐暗期间
     * s_alarm_ringing 尚未置位（要到暗态才置），函数开头那道"重复调用早退"判据
     * 在这段窗口里失效，真正兜住重复请求的是 expire_fade_start 内的
     * s_expire_fade_pending —— 少了它，响铃期间会一拍一次地退化硬切。 */
    if (expire_fade_start(alarm_ringing_dark_action, /*quiet_main_screen=*/true,
                          "主界面→闹钟响铃"))
        return;

    ESP_LOGW(TAG, "响铃渐变不可用，退化为硬切");
    if (!lvgl_port_lock(500))
    {
        ESP_LOGW(TAG, "闹钟响铃切页失败：拿不到 LVGL 锁");
        return;
    }
    alarm_ringing_dark_action();
    lvgl_port_unlock();
}

/**
 * @brief 闹钟响铃的【暗态切页动作】：主界面 → 响铃画面
 *
 * 原样搬自 ui_show_alarm_ringing() 的切页段落，逻辑一行未改，只是执行时机推迟到
 * 背光渐暗到全黑之后。与 countdown_expired_dark_action 完全同构。
 *
 * 【线程】渐变路径由 LVGL 线程调用（已持锁）；硬切兜底路径由 reminder_task
 * 持锁后调用。两条路径都已持锁，内部不再自取锁。
 */
static void alarm_ringing_dark_action(void)
{
    ensure_menu_panel();

    /* ★【2026-08-25 修复：同 ui_show_countdown_expired，压掉闹钟编辑面板】★
     * s_edit_panel 与 s_menu_panel 平级、全屏不透明，只要可见就会盖住本页。
     * 本入口同样属于"强制切页"，必须显式压掉它。完整根因见
     * ui_show_countdown_expired() 内的对应注释。
     * 【本入口为何也需要】闹钟编辑页停留期间若又来一次闹钟响铃（多闹钟/重复闹钟），
     * 不压掉就会出现"响铃画面被编辑页盖住"，与倒计时那条同一病理。 */
    if (s_edit_panel != NULL)
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);

    s_view = UI_VIEW_FUNCTION_MENU;
    s_fn_page = FN_PAGE_ALARM;

    /* 2026-08-16：同 ui_function_menu_enter —— 仅 HIDDEN 不停解码，必须 pause。 */
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj);
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }

    /* ⚠️ 同番茄钟：标志必须在 render_fn_page 之【后】置。
     * render_fn_page() 开头会调 alarm_page_hide()，而后者会把 s_alarm_ringing
     * 清成 false；先置后 render 的话标志会被清掉，画出来是常规闹钟页。 */
    render_fn_page(s_fn_page);

    s_alarm_ringing = true;
    alarm_page_rebuild(); /* 用响铃态重画：红色时间 +「闹钟响铃」 */

    lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);

    /* ⚠️ 响铃期间必须【停掉】功能层空闲超时，不能只 kick 一次。
     *
     * 2026-08-11 修复：原先这里调 menu_kick_idle_timer() 刷新一次就不管了，
     * 但空闲超时只有 UI_MENU_IDLE_TIMEOUT_MS（10s），而闹钟全程响铃
     * (MAX_COUNT-1)×INTERVAL 可达 15~20s，且响铃期间**没有任何 UI 事件会再刷新它**——
     * 每次震动走的是 on_reminder_trigger → bsp_motor_pulse_level，
     * 受 s_alarm_ring_page_shown 拦截压根不碰 UI（那是死锁修复，不能改）。
     * 于是计时器从切页起一路跑到超时，日志即：
     *   「功能层空闲超时，返回主界面」→ 闹钟还在震，画面已退回 GIF 主界面。
     * 改为直接取消计时器；等 alarm_ring_reset_cb 进入编辑页时会重新 kick，
     * 那之后用户有触摸、按 10s 正常计时，行为不变。 */
    menu_cancel_idle_timer();

    /* 挂自动复位定时器：ALARM_RING_HOLD_MS 后清除响铃态回常规闹钟页。
     * alarm_page_hide() 里已把上一轮定时器删掉，此处必为 NULL，仍判一次保持幂等。 */
    if (s_alarm_ring_tmr == NULL)
    {
        s_alarm_ring_tmr = lv_timer_create(alarm_ring_reset_cb, ALARM_RING_HOLD_MS, NULL);
        if (s_alarm_ring_tmr == NULL)
        {
            /* 创建失败兜底：立刻退回常规显示，不把用户留在一个不会消失的响铃画面上 */
            ESP_LOGW(TAG, "响铃复位定时器创建失败，立即回到常规闹钟页");
            s_alarm_ringing = false;
            alarm_page_rebuild();
            /* 这条兜底路径不再有响铃画面要守，恢复正常空闲计时 */
            menu_kick_idle_timer();
        }
    }

    /* 此处【不】再 menu_kick_idle_timer()：上面刚 cancel 掉就是为了让响铃画面
     * 在整个响铃周期内不被空闲超时打断，kick 会把它原样重建回来。 */

    /* 【2026-09-01】暗态动作完成，释放"渐变已发起"窗口。此刻 s_alarm_ringing 已置位，
     * 后续重复响铃回调会被函数开头那道"已在响铃页"判据正常拦下，不会再进来。 */
    s_expire_fade_pending = false;
    ESP_LOGI(TAG, "闹钟响铃：已切到闹钟页显示响铃画面");
}

/* ui_page_next / ui_page_prev 已移除：
 * 功能页之间的左右耳翻页已取消（应用选择统一在"应用列表"层完成，
 * 进入应用后锁定当前页，想换应用摸腹/背返回列表重选）。 */

/* ═══════════════════════════════════════════════════════════════
 * 四层文字菜单：总设置 / 应用列表 / 游戏列表 渲染与导航
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief 隐藏所有功能页专属对象，只保留 title + body 文字标签
 *
 * 总设置/应用列表/游戏列表都复用 s_menu_panel，进入这些纯文字层前
 * 必须先清掉日历、闹钟、倒计时等功能页对象，避免残留遮挡。
 */
static void menu_clear_func_pages(void)
{
    home_icon_fade_cancel(); /* 离开功能盘进功能页，撤销挂起的图标渐暗渐亮 */
    alarm_page_hide();
    countdown_page_hide();
    weather_page_hide();
#if CONFIG_UI_USE_CALENDAR
    time_page_hide();     /* 时间页（大号 HH:MM）整组隐藏 */
    calendar_page_hide(); /* 日历页（点阵）整组隐藏 */
#endif
    /* 离开功能盘进入功能页/文字页时，隐藏功能盘的大图标 */
    if (s_home_icon)
        lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);
    if (s_menu_body)
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
}

/**
 * @brief 把一张外挂 Flash 的 .bin 图标整体读进 PSRAM，填好 lv_image_dsc_t
 *
 * LVGL 9 的 .bin 布局：12 字节 lv_image_header_t（见 lv_image_dsc.h），
 * 其后是像素数据。头字段全部小端，逐字节取，不依赖结构体位域内存布局。
 *
 * @param path  外挂 Flash 路径，如 "S:/img/sj.bin"
 * @param out   输出缓存槽（成功时 out->data 被赋值，失败时保持 NULL）
 * @return true=已缓存；false=打开/读取/分配失败（调用方应回退到文件路径）
 */
static bool home_icon_cache_load_one(const char *path, home_icon_cache_t *out)
{
    lv_fs_file_t f;
    if (lv_fs_open(&f, path, LV_FS_MODE_RD) != LV_FS_RES_OK)
    {
        ESP_LOGW(TAG, "图标预读: 打开失败 %s", path);
        return false;
    }

    /* ① 读 12 字节头 */
    uint8_t hdr[12];
    uint32_t rd = 0;
    if (lv_fs_read(&f, hdr, sizeof(hdr), &rd) != LV_FS_RES_OK || rd < sizeof(hdr))
    {
        lv_fs_close(&f);
        ESP_LOGW(TAG, "图标预读: 头读取不足 %s", path);
        return false;
    }
    if (hdr[0] != LV_IMAGE_HEADER_MAGIC)
    {
        lv_fs_close(&f);
        ESP_LOGW(TAG, "图标预读: magic 不符(0x%02X) %s", hdr[0], path);
        return false;
    }

    uint8_t cf = hdr[1];
    uint16_t flags = (uint16_t)(hdr[2] | (hdr[3] << 8));
    uint16_t w = (uint16_t)(hdr[4] | (hdr[5] << 8));
    uint16_t h = (uint16_t)(hdr[6] | (hdr[7] << 8));
    uint16_t stride = (uint16_t)(hdr[8] | (hdr[9] << 8));

    /* ② 取文件总长，反推像素数据字节数（不自己按 cf 推算，兼容各种格式） */
    uint32_t fsize = 0;
    if (lv_fs_seek(&f, 0, LV_FS_SEEK_END) != LV_FS_RES_OK ||
        lv_fs_tell(&f, &fsize) != LV_FS_RES_OK || fsize <= sizeof(hdr))
    {
        lv_fs_close(&f);
        ESP_LOGW(TAG, "图标预读: 取文件长度失败 %s", path);
        return false;
    }
    uint32_t data_size = fsize - sizeof(hdr);

    /* ③ 分配 PSRAM 并读入像素数据（跳过头） */
    uint8_t *buf = heap_caps_malloc(data_size, MALLOC_CAP_SPIRAM);
    if (buf == NULL)
    {
        lv_fs_close(&f);
        ESP_LOGW(TAG, "图标预读: PSRAM 分配 %lu 字节失败 %s",
                 (unsigned long)data_size, path);
        return false;
    }

    if (lv_fs_seek(&f, sizeof(hdr), LV_FS_SEEK_SET) != LV_FS_RES_OK)
    {
        heap_caps_free(buf);
        lv_fs_close(&f);
        return false;
    }

    /* 分块读：一次性读 30KB 在部分 FS 实现下会失败或超时，按 4KB 推进更稳 */
    uint32_t got = 0;
    while (got < data_size)
    {
        uint32_t want = data_size - got;
        if (want > 4096)
            want = 4096;
        rd = 0;
        if (lv_fs_read(&f, buf + got, want, &rd) != LV_FS_RES_OK || rd == 0)
            break;
        got += rd;
    }
    lv_fs_close(&f);

    if (got != data_size)
    {
        heap_caps_free(buf);
        ESP_LOGW(TAG, "图标预读: 数据不完整 %lu/%lu %s",
                 (unsigned long)got, (unsigned long)data_size, path);
        return false;
    }

    /* ④ 填描述符。header 各字段照抄文件头，data 指向 PSRAM */
    out->data = buf;
    out->dsc.header.magic = LV_IMAGE_HEADER_MAGIC;
    out->dsc.header.cf = cf;
    out->dsc.header.flags = flags;
    out->dsc.header.w = w;
    out->dsc.header.h = h;
    out->dsc.header.stride = stride;
    out->dsc.header.reserved_2 = 0;
    out->dsc.data_size = data_size;
    out->dsc.data = buf;
    out->dsc.reserved = NULL;
    out->dsc.reserved_2 = NULL;
    return true;
}

/**
 * @brief 开机时把功能盘所有外挂 Flash 图标预读进 PSRAM
 *
 * 幂等：重复调用直接返回。任何一张失败都不影响其它张，未命中的项
 * 在 home_render 里自动回退到原文件路径，功能不受损、只是换图慢一点。
 */
static void home_icon_cache_init(void)
{
    if (s_home_icon_cache_ready)
        return;
    s_home_icon_cache_ready = true;

    uint32_t ok = 0, bytes = 0;
    for (size_t i = 0; i < HOME_ITEM_COUNT; i++)
    {
        /* 只缓存外挂 Flash 项；内置固件图（it->img）本就在 flash 里可直接寻址，
         * 无需再复制一份到 PSRAM。 */
        if (s_home_items[i].img_path == NULL)
            continue;
        if (home_icon_cache_load_one(s_home_items[i].img_path, &s_home_icon_cache[i]))
        {
            ok++;
            bytes += s_home_icon_cache[i].dsc.data_size;
        }
    }
    ESP_LOGI(TAG, "功能盘图标预读完成: %lu/%u 张进 PSRAM, 共 %lu 字节",
             (unsigned long)ok, (unsigned)HOME_ITEM_COUNT, (unsigned long)bytes);
}

/* ── 左右耳切换图标：渐暗→换图→渐亮（消除斜切，2026-08-13）──────────────
 *
 * 【解决什么】换图斜切。写入显存(竖填×横扫)的斜切被面板扫描看见。方案：
 * 渐暗(背光100→0)期间旧图均匀变暗，暗到底后换图(背光0，斜切被遮)，
 * 再渐亮(0→100)新图均匀亮起。
 * 【为何不用缩放/位移动画】缩放/位移都是逐帧重绘 GRAM，背光亮着时每一帧
 * 的写入斜切都会露出；位移还会带回旧图新图并排。详见 BUG-041 实测记录。
 *
 * 【DARK 窗口】换图后 LVGL 异步渲染 + flush + 液晶灰阶响应(20~40ms)需要时间，
 * 若立即渐亮会在渐亮初期看到上一张残影，故插入 UI_HOME_ICON_FADE_DARK_MS 等待。 */
#define UI_HOME_ICON_FADE_OUT_MS 120 // 渐暗耗时
#define UI_HOME_ICON_FADE_DARK_MS 50 // 全黑：换 src + 等 flush + 等液晶响应
#define UI_HOME_ICON_FADE_IN_MS 150  // 渐亮耗时

/* 取消挂起的图标渐暗渐亮（退出功能盘/进功能页/待机回主界面时调用）。
 * 渐暗到一半被取消时，把背光恢复到基准亮度，避免停在暗态。幂等。 */
static void home_icon_fade_cancel(void)
{
    if (s_home_icon_fade_phase != UI_HOME_ICON_FADE_IDLE)
    {
        bsp_board_lcd_fade_step_fine(s_home_icon_fade_bk, s_home_icon_fade_bk, 1, 1);
    }
    s_home_icon_fade_phase = UI_HOME_ICON_FADE_IDLE;
    if (s_home_icon_fade_tmr != NULL)
        lv_timer_pause(s_home_icon_fade_tmr);
}

/* 渐暗渐亮推进回调：渐暗到底换图→等待→渐亮，跑在 LVGL 线程 */
static void home_icon_fade_timer_cb(lv_timer_t *t)
{
    if (s_home_icon_fade_phase == UI_HOME_ICON_FADE_IDLE)
    {
        lv_timer_pause(t);
        return;
    }
    uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - s_home_icon_fade_start_us) / 1000);
    bool done;
    switch (s_home_icon_fade_phase)
    {
    case UI_HOME_ICON_FADE_OUT:
        done = bsp_board_lcd_fade_step_fine(s_home_icon_fade_bk, 0, elapsed_ms, UI_HOME_ICON_FADE_OUT_MS);
        if (done)
        {
            home_render(); // 暗到底：换图
            s_home_icon_fade_phase = UI_HOME_ICON_FADE_DARK;
            s_home_icon_fade_start_us = esp_timer_get_time();
        }
        break;
    case UI_HOME_ICON_FADE_DARK:
        /* 等 flush + 液晶响应走完，避免渐亮初期看到上一张残影 */
        if (elapsed_ms >= UI_HOME_ICON_FADE_DARK_MS)
        {
            s_home_icon_fade_phase = UI_HOME_ICON_FADE_IN;
            s_home_icon_fade_start_us = esp_timer_get_time();
        }
        break;
    case UI_HOME_ICON_FADE_IN:
        done = bsp_board_lcd_fade_step_fine(0, s_home_icon_fade_bk, elapsed_ms, UI_HOME_ICON_FADE_IN_MS);
        if (done)
        {
            s_home_icon_fade_phase = UI_HOME_ICON_FADE_IDLE;
            lv_timer_pause(t);
        }
        break;
    default:
        lv_timer_pause(t);
        break;
    }
}

/**
 * @brief 渲染功能盘：屏幕正中显示当前项的大图标（无文字）
 *
 * 已假定调用方持有 LVGL 锁。隐藏旧的标题/正文文字与功能页对象，只留功能盘图标。
 */
static void home_render(void)
{
    if (s_home_icon == NULL)
        return;

    /* 隐藏功能页专属对象 + 旧文字面板，只留功能盘图标 */
    alarm_page_hide();
    countdown_page_hide();
    weather_page_hide();
#if CONFIG_UI_USE_CALENDAR
    time_page_hide();     /* 时间页（大号 HH:MM）整组隐藏 */
    calendar_page_hide(); /* 日历页（点阵）整组隐藏 */
#endif
    if (s_menu_title)
        lv_label_set_text(s_menu_title, "");
    if (s_menu_body)
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);

    const home_item_t *it = &s_home_items[s_home_idx];
    if (it->img_path != NULL || it->img != NULL)
    {
        /* 有图：显示图片，关掉占位色块的背景，尺寸由图片自身决定。
         * lv_image_set_src 同时吃两种源：传 const char* 走 LVGL 文件系统
         * （"S:/img/xx.bin" → 外挂 Flash），传 lv_image_dsc_t* 走内存图。
         *
         * 【优先级】PSRAM 预读缓存 > 外挂 Flash 路径 > 内置固件图。
         * 走缓存时是内存图，绘制期零文件 I/O —— 这是把换图暗场压到最短的关键，
         * 因为换图必须在灭屏期间完成（见 home_anim_phase1_ready）。
         * 缓存未命中（预读失败/PSRAM 不足）时自动回退到文件路径，功能不受影响。 */
        if (s_home_icon_cache[s_home_idx].data != NULL)
            lv_image_set_src(s_home_icon, &s_home_icon_cache[s_home_idx].dsc);
        else if (it->img_path != NULL)
            lv_image_set_src(s_home_icon, it->img_path);
        else
            lv_image_set_src(s_home_icon, it->img);
        lv_obj_set_style_bg_opa(s_home_icon, LV_OPA_TRANSP, 0);

        /* 【压扁动画残影修复 2026-08-17】尺寸必须钉成固定像素，不能留 LV_SIZE_CONTENT。
         * 根因：lv_image_set_scale_y → scale_update()（lv_image.c:960）失效"旧变换区域"
         * 时用的是 obj->coords，但它开头先调 lv_obj_update_layout()；对象是
         * LV_SIZE_CONTENT 时这一下会按新缩放重算 coords，于是"旧区域"用的已是新坐标，
         * 上一帧真正占的那块（压扁时位移最大的顶部）永远没进脏区 → 顶部残留成残影。
         * 固定尺寸后 coords 在动画期间恒定，失效区域计算即正确。
         *
         * 顺序要紧：先 update_layout 让 LV_SIZE_CONTENT 按【新图】算出真实尺寸，
         * 再读回来钉死。若直接读未刷新的 coords，拿到的是上一张图的尺寸。 */
        /* 【延迟修复 2026-08-17】原先用 lv_obj_update_layout() 同步刷新布局再读回
         * 宽高，实测按下到动画之间出现明显延迟：该函数是同步强制布局，图标走外挂
         * Flash 文件路径时会当场触发一次图片解码来确定尺寸，几十~上百毫秒卡在
         * LVGL 线程上。改为直接从图片自身取尺寸，零解码开销。 */
        if (s_home_icon_cache[s_home_idx].data != NULL)
        {
            /* 缓存命中：内存图描述符里的 header 就是现成尺寸 */
            lv_obj_set_size(s_home_icon,
                            s_home_icon_cache[s_home_idx].dsc.header.w,
                            s_home_icon_cache[s_home_idx].dsc.header.h);
        }
        else
        {
            /* 未命中（走文件路径/内置图）：LV_SIZE_CONTENT 由 LVGL 自行按图算，
             * 不在此处同步强求。此路径本就要读文件，钉尺寸的收益让位于响应速度；
             * 压扁残影仅在有缓存的常规路径上出现，此处保持原行为即可。 */
            lv_obj_set_size(s_home_icon, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
        }
    }
    else
    {
        /* 无图：清空图片源，用圆角色块占位（待转图项的临时显示） */
        lv_image_set_src(s_home_icon, NULL);
        lv_obj_set_size(s_home_icon, 96, 96);
        lv_obj_set_style_radius(s_home_icon, LV_RADIUS_CIRCLE, 0);
        lv_obj_set_style_bg_color(s_home_icon, lv_color_hex(it->color), 0);
        lv_obj_set_style_bg_opa(s_home_icon, LV_OPA_COVER, 0);
    }
    lv_obj_align(s_home_icon, LV_ALIGN_CENTER, 0, 0); /* 切换后重新居中 */
    /* 复位头部确认弹动可能残留的压扁状态（256 = 100% 高度，位移归零）：
     * 若上次弹动被中途打断（提前退出功能盘等），图标会停在压扁状态，
     * 这里统一归位，所有回到功能盘的入口都会经过 home_render。 */
    lv_image_set_scale_y(s_home_icon, 256);
    lv_obj_set_style_translate_y(s_home_icon, 0, 0);
    lv_obj_clear_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);

    /* 【2026-08-18 新增】压扁提速的兜底恢复。
     * 压扁动画若被中途打断（提前退功能盘 / 空闲超时 / 待机回主界面），
     * home_press_phase2_ready 不会执行，两个 LVGL 定时器就会永久停在 16ms，
     * 之后主界面 GIF 全程按双倍频率刷新 —— 这正是本改动最需要防的副作用。
     * 所有回到功能盘/换图的入口都必经 home_render，故统一在此兜底。
     * 函数自身幂等，未提速时调用为空操作。 */
    home_press_refr_boost(false);
}

/* ── 左右耳切换：横向果冻弹动 ────────────────────────────────────────────
 *
 * 【2026-08-12 撕裂问题定位记录，改动前务必先读】
 * 之前排查撕裂时走过几条弯路，结论如下：
 *   · 真正的约束是「单次 flush 的脏区不能超过一块 draw buffer」。
 *     当前 buffer_size = 320*240/6 = 12800 像素（见 disp_cfg），
 *     图标 100x100 = 10000 像素，本身装得下 → 能原子刷完。
 *   · 横移时脏区 = 旧位置 ∪ 新位置 = (100+PUNCH) x 100。
 *     PUNCH=28 时为 12800 像素，正好顶满 buffer，毫无余量；
 *     故降到 20px（12000 像素），留 800 像素余量，避免被拆成两次 flush。
 *   · ❌ 严禁在切换流程里调 lv_obj_invalidate(lv_screen_active()) —
 *     那会把脏区放大到整屏 76800 像素，必须分 6 块传输，
 *     肉眼可见「一点点擦除」，这正是之前撕裂加剧的根源。
 *   · ❌ 透明度渐变（lv_obj_set_style_opa 动画）会让 LVGL 建临时图层做
 *     alpha 混合，图层内存不足时被拆批上屏 → 图标内部出现斜切分界。
 *     位移动画不走图层路径，故优先选位移而非渐变。
 * ─────────────────────────────────────────────────────────────────────── */
#define HOME_PUNCH_PX 20 // 横向位移幅度：脏区 (100+20)x100=12000 < buffer 12800

/* 位移执行回调：用 translate_x 偏移，不破坏 lv_obj_align 的居中基准。
 * 不需要手动 invalidate：LVGL 改 translate 样式时内部已按「旧区域 + 新区域」
 * 各失效一次，手动再调只会放大脏区、适得其反。 */
static void home_anim_x_cb(void *obj, int32_t v)
{
    lv_obj_set_style_translate_x((lv_obj_t *)obj, (lv_coord_t)v, 0);
}

/* 【home_anim_phase2_ready 已随第二段回弹动画一并移除，2026-08-12】
 * 原因见 home_anim_phase1_ready 末尾注释：回弹用 translate_x 会破坏
 * obj->coords 与渲染位置的对齐，正是旧图擦不净的根源。标志改由
 * home_anim_phase1_ready 直接清除。 */

/* 第一段（弹出）结束：灭屏 → 擦旧图 → 换图 → 亮屏（不再起第二段） */
static void home_anim_phase1_ready(lv_anim_t *a)
{
    int dir = (int)(intptr_t)a->user_data; // +1 下一个 / -1 上一个

    /* ── 左右耳切换：换图（2026-08-12 清理后的最简版）────────────────────
     *
     * 本函数是第一段横移动画的 ready 回调，运行在 lv_timer_handler 内部。
     * 这里**只做换图**：归零位移 + 换 src，不强制刷新、不显式失效任何区域，
     * 让 LVGL 走自己的刷新周期去合并处理脏区。
     *
     * 【为何不能在这里调 lv_refr_now —— 实测日志证据】
     * 曾在此处强制刷新并显式失效一块固定框，结果日志显示一次刷新里挤进了
     * 四份互相错开的失效区域，无法合并，被拆成 15~17 块、耗时 90~107ms：
     *     #1~#8  (91,70)-(190,169)  ← 横移动画 8 帧，位置相同但未及渲染
     *     #9     (93,73)-(226,166)  ← 人为加的固定框
     *     #10    (91,70)-(190,169)  ← 归零前的旧位置
     *     #11    (110,70)-(209,169) ← 归零后的新位置（与旧位置差 HOME_PUNCH_PX）
     * 旧位置 91 与新位置 110 各画各的，这正是实拍"两张图同框"的成因——
     * 并非两图叠加，而是**并排画在相差 20px 的两处**。
     * 那 100ms 同步刷新还把 LVGL 线程堵死，表现为"动画结束后一段僵硬"。
     *
     * 交还给 LVGL 后，8 帧相同区域会在 lv_refr_join_area() 里被合并成 1 个，
     * 脏区收敛为「旧位置 ∪ 新位置」≈ 119x100 = 11900 像素 < buffer 12800。
     *
     * 【已被实测排除的方向，勿再重走】
     *   ✗ 旧图擦不干净         —— 脏区完整覆盖图标，最终静止画面本就干净
     *   ✗ translate 导致错位   —— coords 恒为 (110,70)-(209,169)，从未偏移
     *   ✗ lv_refr_now 不等 DMA —— IDF 5.3.4 下 LVGL_PORT_HANDLE_FLUSH_READY=1，
     *                             flush 走 on_color_trans_done 回调，确实同步等待
     *   ✗ 双缓冲                —— 实测单缓冲同样有问题
     *   ✗ 背光灭屏遮蔽          —— 遮不住分块过程，只是白闪一下
     *   ✗ 缩小图标 set_scale    —— 缩放是渲染期变换，不改 coords，脏区计算不变 */

    lv_obj_set_style_translate_x(s_home_icon, 0, 0); // 位移归零，回到居中基准

    s_home_idx = (s_home_idx + dir + (int)HOME_ITEM_COUNT) % (int)HOME_ITEM_COUNT;
    home_render(); // 换 src（内部会 clear HIDDEN 并居中）

    s_home_animating = false;
}

/**
 * @brief 左右耳切换：渐暗(果冻收起)→换图→渐亮(果冻弹入)（消除斜切，无同框无突兀）
 * @param dir +1=下一个；-1=上一个
 *
 * 已假定调用方持有 LVGL 锁。下标先推进，渐变期间连按只推进下标不出图，
 * 暗到底后一次性显示最终那张（连点不会斜切也不会同框）。
 */
static void home_jelly(int dir)
{
    if (s_home_icon == NULL)
        return;

    /* 渐暗/渐亮进行中：禁止切换（动画没结束不能切图），直接返回，
     * 不推进下标、不响应本次触摸。左右耳切换的 fade 与进功能盘的 fade
     * 都要拦——进功能盘渐亮那 1s 里 s_view 已是 HOME，但动画仍未结束。 */
    if (s_home_icon_fade_phase != UI_HOME_ICON_FADE_IDLE ||
        s_home_fade_phase != UI_HOME_FADE_IDLE)
        return;

    /* 下标推进（两个 fade 都空闲时才走到这里）。 */
    s_home_idx = (s_home_idx + dir + (int)HOME_ITEM_COUNT) % (int)HOME_ITEM_COUNT;

    /* ★【2026-08-31 问题2】震动跟随「真实切换」而非「触摸」。
     * 【修的问题】原先震动由触摸层 bsp_touch.c 在左右耳按下沿无条件打，
     *   但上面那道 fade 拦截会让本次触摸【不切图、不推进下标】直接 return，
     *   于是出现「屏幕没切换，却震了一下」。
     * 【改法】触摸层已不再为功能盘的左右耳震动（见 bsp_touch.c 的 page_vib_view），
     *   改由此处——已越过拦截、下标确实推进了——才请求震动，做到「切了才震」。
     * 【为什么不直接 bsp_motor_pulse()】本函数由触摸任务在【持 LVGL 锁】时调用，
     *   而 bsp_motor_pulse() 内含 30ms vTaskDelay，直接调会持锁阻塞 LVGL 线程
     *   （同 BUG-038 的坑）。故只置标志，由触摸任务解锁后统一执行。 */
    bsp_touch_request_vibrate();

#if UI_FLUSH_TRACE
    /* 【临时调试】从这一刻起记录所有失效/flush 事件，600ms 后自动打印 */
    ui_ftrace_begin();
#endif

    lv_obj_set_style_translate_x(s_home_icon, 0, 0);      // 确保无残留偏移
    s_home_icon_fade_bk = bsp_board_lcd_get_brightness(); // 快照当前亮度（standby 可能已降）

    /* 触发背光渐暗，暗到底后换图，再渐亮 */
    s_home_icon_fade_phase = UI_HOME_ICON_FADE_OUT;
    s_home_icon_fade_start_us = esp_timer_get_time();
    if (s_home_icon_fade_tmr != NULL)
        lv_timer_resume(s_home_icon_fade_tmr);
}

/* ── 头部确认弹动：图标"顶部下压一下再抬回"（2026-08-07 改为压扁式）──
 *
 * 需求（2026-08-06）：头部触摸进入功能时，功能图片也要有弹动效果。
 * 原版是整体等比缩小再 overshoot 弹回；改为参考跳一跳蓄力压扁手感：
 * 底部不动、顶部向下压扁，只压一下再抬回原状，不带弹簧回弹（无 overshoot）。
 *
 * 实现原理：lv_image_set_scale_y 只压缩纵向；image 的缩放锚点默认是图片中心
 * （lv_image.c:689 pivot 默认 LV_PCT(50)），故直接压会顶底同时向中间收。
 * 之前手动算 translate_y 补偿方向/时机对不齐（真机测出底部仍在动），
 * 改为把 pivot 钉到底边 LV_PCT(100)，锚点即不动点，缩放天然只压顶边。
 */
/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-17 改版】压扁 → 「压暗 + 轻微下沉」
 *
 * 【为什么换掉压扁】压扁每帧改变图标高度，而这块屏没有 TE 引脚、刷新与面板
 * 扫描不同步：flush 撞上扫描线扫过图标那一带的中途时，屏上会同时存在相邻两帧，
 * 上下两半的图标高度不同 → 实拍就是"一大一小两个图标同框、顶部被切平"。
 *
 * UI_FLUSH_TRACE 实测已排除软件侧成因：每帧都正确失效「旧区域 ∪ 新区域」，
 * flush 完整覆盖、单块、0~1ms；好坏两次的脏区逐项相同，唯一差别是相位。
 * 故残影不是"上一帧没擦掉"，而是"两帧同时在物理屏上"，软件擦不掉。
 *
 * 【判据】撕裂可见度只取决于【每帧的形变量】，与用什么 API 无关：
 *   · 跳一跳蓄力压扁 1400ms/约30帧 → 每帧约 1px → 看不出（同样是压扁）
 *   · 本处原压扁       90ms/约2帧  → 每帧约 15px → 一眼可见
 * 帧间隔实测约 45ms，故时长决定帧数：150ms ≈ 3帧。
 *
 * 【本方案】改为「整体下沉 HOME_PRESS_SINK_PX + 叠黑压暗」：
 *   · 压暗（recolor）完全不改几何，撕开也只是一条明暗略不同的横带；
 *   · 下沉总量仅 6px、分约 3 帧 → 每帧约 2px，撕开错位 2px 基本不可见；
 *   · 手感仍是"按下去"的实体反馈，配合原有震动足够。
 * ══════════════════════════════════════════════════════════════════════════ */
/* ── 压扁幅度：撕裂消不掉，就把每帧错位压到看不见的量级 ────────────────────
 * 【定位】屏无 TE 引脚，flush 与面板扫描永远不同步，撕裂是硬件天花板、无法消除。
 * 能做的只有减轻：每帧错位量 = 总形变量 ÷ 帧数，三个杠杆已全部用上——
 *   ① 帧数↑：提速 33→16ms + 时长 90→200ms（2帧 → 9帧）
 *   ② 分配均匀：ease_out → linear（峰值帧 = 平均帧，不再头重脚轻）
 *   ③ 总形变量↓：本宏 ← 最后一个杠杆
 *
 * 【调参阶梯】图标 100px 高，256 = 100%；顶边行程 = 100 - 100*scale/256。
 * 按 9 帧 linear 计算（峰值即平均）：
 *   180 → 压到 70px，行程 30px → 3.3px/帧   形变最足，锯齿仍可见
 *   195 → 压到 76px，行程 24px → 2.7px/帧 ← 当前取值，手感与观感的折中
 *   205 → 压到 80px，行程 20px → 2.2px/帧   再淡一档，按压感开始变弱
 *   215 → 压到 84px，行程 16px → 1.8px/帧   接近跳一跳蓄力那档（约1px/帧＝看不出）
 * 实拍后按此表直接调本宏即可，逻辑不用动；按下还有震动反馈兜底，
 * 形变减小不等于反馈减弱。 */
#define HOME_PRESS_SQUASH_SCALE_Y 195 /* 压扁到的纵向比例（原 180；LVGL 9：256 = 100%） */

/* ══════════════════════════════════════════════════════════════════════════
 * 【2026-08-18】压扁动画「提速 + 拉长」——降低每帧形变量
 *
 * 【为什么】撕开时上下两半的错位量 = 每帧形变量，而
 *       每帧形变量 = 总形变量 ÷ 帧数，  帧数 = 动画时长 ÷ 帧间隔
 * 总形变量定死 30px（图标 100px 高，scale_y 256→180 即压到 70px，pivot 在底边，
 * 顶边走 30px），不能改，改了就没有"按下去"的手感。故只能动帧数：
 *   · 拉长 = 增大分子（动画时长）
 *   · 提速 = 减小分母（帧间隔）
 *
 * 【实测帧间隔】45ms（= 33ms 定时器周期 + lvgl_port timer_period_ms=10 的调度
 * 粒度和渲染耗时）。周期改 16ms 后约 22ms。
 *
 *   压下段(30px)  现状 90ms/45ms  = 2帧   → 15px/帧  ← 一眼可见"两个图标同框"
 *                 提速 90ms/22ms  = 4帧   → 7.5px/帧
 *                 本改 150ms/22ms = 7帧   → 4.3px/帧
 *   抬回段(30px)  现状 150ms/45ms = 3.3帧 → 9px/帧
 *                 本改 150ms/22ms = 6.8帧 → 4.4px/帧 ← 光提速即可，时长不动
 * 故只拉长 a1 不拉长 a2，两段最后都落在 4px 出头，前后对称。
 *
 * 【两个定时器必须一起改】只提显示刷新率，动画值仍 33ms 才变一次，多刷的是
 * 重复帧；只提动画步进率，值变了也要等 33ms 才上屏。两者缺一都白搭。
 *   · 动画步进：lv_anim_get_timer()                （lv_anim.h:415）
 *   · 显示刷新：lv_display_get_refr_timer(lvgl_disp)（lv_display.h:611）
 * 均为 LVGL 9 公开 API，运行时改周期不回写 CONFIG_LV_DEF_REFR_PERIOD=33
 * （那只是新建定时器时的初值），故 sdkconfig 一个字节不动。
 *
 * 【本改动不做什么】不消除撕裂 —— 屏无 TE 引脚，flush 与面板扫描不同步这件事
 * 完全没变，只是错位量从 15px 降到 4px（"一大一小两个图标"→"边缘 4px 毛刺"）。
 * 也不影响左右耳切图：那条路是灭屏画一帧，帧率对单帧无效。
 *
 * 【为何 16ms 是下限】每帧脏区 = 旧∪新 ≈ 100x100 = 10000 像素 = 20000 字节，
 * SPI 80MHz 传约 2ms，加渲染共约 5~10ms，塞进 22ms 帧间隔有余量；
 * 再往 8ms 提就塞不下，会开始丢帧。
 * ══════════════════════════════════════════════════════════════════════════ */
#define HOME_PRESS_REFR_FAST_MS 16    /* 压扁期间的定时器周期（提速档） */
#define HOME_PRESS_REFR_NORMAL_MS 33  /* 常态周期，与 CONFIG_LV_DEF_REFR_PERIOD 一致 */
#define HOME_PRESS_SQUASH_DOWN_MS 200 /* 压下段时长（原 90 → 150 → 200，拉长以增加帧数） */
#define HOME_PRESS_SQUASH_UP_MS 200   /* 抬回段时长（原 150 → 200，与压下段对称） */

/* ── 【2026-08-18 第二轮】缓动曲线 ease_out → linear ──────────────────────
 * 【为什么】撕裂看着有多明显取决于【最坏那一帧】，不是平均值。
 * lv_anim_path_ease_out 是 cubic-bezier(0,0,0.58,1)（lv_anim.c:307），头快尾慢：
 * 按 9 帧算，第 1 帧就走掉总行程的约 18%，峰值约为平均值的 1.57 倍。
 *
 *   30px 总形变 / 9 帧   ease_out: 平均 3.3px，最坏帧 5.2px  ← 锯齿主要来源
 *                        linear  : 平均 3.3px，最坏帧 3.3px
 *
 * 改 linear 后每帧位移严格均匀，峰值即平均值，零成本砍掉约 36% 的最坏错位。
 * 代价仅是手感从"弹一下"变"匀速压下"，形变总量 30px 未动，按压反馈不减。 */
#define HOME_PRESS_SQUASH_PATH lv_anim_path_linear

/* 提速状态标记：确保 boost(false) 幂等，且不会把别处调过的周期误改回去 */
static bool s_home_press_boosted = false;

/**
 * @brief 头部压扁期临时提高 LVGL 帧率（幂等，须持有 LVGL 锁）
 *
 * @param on true=进入提速档（16ms）；false=恢复常态（33ms）
 *
 * 副作用可忽略：anim timer 虽是全局的，但动画值按经过时间算，提速只让别的动画
 * 更平滑、不会更快；且按下功能盘图标那一刻 GIF 不在播（s_view 已是 HOME）、
 * 游戏未起、左右耳切图 fade 被 home_jelly 开头拦着不可能并发。窗口仅 300ms。
 */
static void home_press_refr_boost(bool on)
{
    if (on == s_home_press_boosted)
        return; /* 幂等：状态未变直接返回 */

    uint32_t period = on ? HOME_PRESS_REFR_FAST_MS : HOME_PRESS_REFR_NORMAL_MS;

    lv_timer_t *anim_tmr = lv_anim_get_timer();
    if (anim_tmr != NULL)
        lv_timer_set_period(anim_tmr, period);

    if (lvgl_disp != NULL)
    {
        lv_timer_t *refr_tmr = lv_display_get_refr_timer(lvgl_disp);
        if (refr_tmr != NULL)
            lv_timer_set_period(refr_tmr, period);
    }

    s_home_press_boosted = on;
}

static void home_anim_squash_cb(void *obj, int32_t v)
{
    lv_image_set_scale_y((lv_obj_t *)obj, (uint16_t)v);
}

/* 判定某个功能盘项的进入回调是否为游戏项。
 * 【为何按函数指针判而不是给 home_item_t 加字段】加字段要改动 8 行数据表，
 * 每行都带着「原: ...」的历史注释，改动面大且易出错；按指针比对只需一处，
 * 且新增游戏时编译器会因为 enter_xxx 未列入而给不出提示——故在此显式列全，
 * 【新增游戏项时务必同步加进本函数】。 */
static bool home_item_is_game(void (*on_enter)(void))
{
    return on_enter == enter_whack ||
           on_enter == enter_race ||
           on_enter == enter_jump;
}

/* 抬回结束：复位压扁 + 清标志 + 真正进入当前功能 */
static void home_press_phase2_ready(lv_anim_t *a)
{
    (void)a;
    if (s_home_icon)
        lv_image_set_scale_y(s_home_icon, 256); /* 归位，避免累计误差 */
    s_home_animating = false;

    /* 【2026-08-18】动画正常结束：恢复常态帧率。
     * 必须放在 on_enter 之前 —— on_enter 会切 s_view、进功能页或起游戏，
     * 之后就不再经过 home_render 的兜底路径了。 */
    home_press_refr_boost(false);

    /* 进入功能页会切 s_view / 隐藏面板，放在动画末尾执行。
     *
     * 【2026-08-18】静态功能页（时间/日历/闹钟/定时器/天气）改走进入渐变：
     * 渐暗到全黑后才 render_fn_page，整屏重绘藏在暗态里，用户看不到分块刷屏。
     * 游戏三项（打地鼠/赛车/跳一跳）保持原有的直调行为——它们的入场动画
     * （3 秒开场进度条、台子从天而降）若在暗态启动会被黑屏吃掉开头，
     * 取舍未定，按用户指定暂不接入。 */
    void (*fn)(void) = s_home_items[s_home_idx].on_enter;
    if (fn == NULL)
        return;
    if (home_item_is_game(fn))
        fn(); /* 游戏：原样直调，不接渐变 */
    else
        home_enter_fade(fn); /* 静态功能页：四段式渐变 */
}

/* 压扁到位：起第二段抬回（无 overshoot，单纯压一下再回位） */
static void home_press_phase1_ready(lv_anim_t *a)
{
    (void)a;
    lv_anim_t a2;
    lv_anim_init(&a2);
    lv_anim_set_var(&a2, s_home_icon);
    lv_anim_set_exec_cb(&a2, home_anim_squash_cb);
    lv_anim_set_values(&a2, HOME_PRESS_SQUASH_SCALE_Y, 256);
    lv_anim_set_time(&a2, HOME_PRESS_SQUASH_UP_MS);
    lv_anim_set_path_cb(&a2, HOME_PRESS_SQUASH_PATH); /* ease_out → linear：削掉峰值帧 */
    lv_anim_set_ready_cb(&a2, home_press_phase2_ready);
    lv_anim_start(&a2);
}

/**
 * @brief 头部短按确认时的图标压扁动效（调用方需持有 LVGL 锁）
 *
 * 动画跑不起来（图标为空）时直接进入功能，保证功能永不因动效缺失而失效。
 */
static void home_press_jelly(void)
{
    if (s_home_icon == NULL)
    {
        if (s_home_items[s_home_idx].on_enter)
            s_home_items[s_home_idx].on_enter();
        return;
    }
    s_home_animating = true;

    /* 【2026-08-18】起手提速：把动画步进与显示刷新两个定时器一起压到 16ms，
     * 让 30px 的形变摊到更多帧上（详见 home_press_refr_boost 上方的账）。
     * 恢复点有二：正常结束在 home_press_phase2_ready，中途打断在 home_render。 */
    home_press_refr_boost(true);

    /* ① 消除拖影：把图标钉成固定尺寸，不再用 LV_SIZE_CONTENT。 */
    lv_obj_set_size(s_home_icon,
                    lv_obj_get_width(s_home_icon),
                    lv_obj_get_height(s_home_icon));
    /* ② 缩放锚点钉在图片底边（LV_PCT(100) = 图片高度的 100% 处），底边即不动点。 */
    lv_image_set_pivot_y(s_home_icon, LV_PCT(100));

    lv_anim_t a1;
    lv_anim_init(&a1);
    lv_anim_set_var(&a1, s_home_icon);
    lv_anim_set_exec_cb(&a1, home_anim_squash_cb);
    lv_anim_set_values(&a1, 256, HOME_PRESS_SQUASH_SCALE_Y);
    lv_anim_set_time(&a1, HOME_PRESS_SQUASH_DOWN_MS); /* 90→200：拉长以增加帧数 */
    lv_anim_set_path_cb(&a1, HOME_PRESS_SQUASH_PATH); /* ease_out → linear：削掉峰值帧 */
    lv_anim_set_ready_cb(&a1, home_press_phase1_ready);
    lv_anim_start(&a1);
}

/* 进游戏的【暗态动作】：切游戏视图 + 启动指定游戏 + 停空闲计时。
 *
 * 【2026-08-19】原先这三步跟「隐藏功能盘图标」一起写在 enter_game_common 里、
 * 在满亮度下直接执行，build_panel() 建满一屏对象引发的整屏重绘就是用户说的
 * 「进入时黑屏闪一下」。现在整块推迟到渐变的暗态那一刻执行，刷屏藏在全黑里。
 *
 * ⚠️ s_view 必须跟 games_start 一起放在这里，不能留在触摸那一刻：
 * 渐变全程约 290ms(OUT100+DARK40+IN150)，若 s_view 提前就置成 UI_VIEW_GAME，
 * 这期间画面还是功能盘，触摸却已按游戏分发；反之若留在触摸处置位则更糟——
 * 渐变期间 s_view 仍是 UI_VIEW_HOME，画面已在切游戏，触摸会被当成功能盘翻页。
 * 两者绑在同一时刻切换是唯一自洽的做法。
 *
 * 【线程】由渐变回调在 LVGL 线程调用（已持锁）；games_start 内部各游戏的
 * xxx_start() 会再取一次 LVGL 锁，那是递归互斥量，同线程重入安全
 * （与 game_exit_dark_action 同理）。 */
static void game_enter_dark_action(game_id_t id)
{
    /* 【2026-08-19 从 enter_game_common 挪来】隐藏功能盘大图标。
     * 必须在全黑里做：这一步会让图标那块区域失效并触发一次重绘，
     * 原先放在渐暗之前的满亮度下执行，用户能直接看到那次刷屏
     * （反馈原话「变暗也是刷新的、看到了刷新的过程」）。
     * 挪到暗态后，它与 games_start 的整屏重绘合并成同一批，一起藏在黑屏里。 */
    if (s_home_icon)
        lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);
    s_view = UI_VIEW_GAME;
    games_start(id);
    /* 游戏期间禁用空闲超时：难度选择/倒计时/游戏进行/结算都可能长时间无翻页式输入，
     * 不应被自动踢回主界面。退出游戏（腹背→back_to_home / 长按耳→exit_to_main /
     * 空闲回调本身）会重新建立或不再需要该计时器。 */
    menu_cancel_idle_timer();
}

/* 三个无参包装：s_cd_fade_dark_cb 是 void(*)(void)，带不了 game_id 参数，
 * 故按每个游戏各包一层（与 app_enter_time / app_enter_weather 同一写法）。 */
static void game_enter_dark_whack(void) { game_enter_dark_action(GAME_WHACK); }
static void game_enter_dark_race(void) { game_enter_dark_action(GAME_RACE); }
static void game_enter_dark_jump(void) { game_enter_dark_action(GAME_JUMP); }

/* 进游戏公共逻辑：隐藏功能盘图标 → 渐变进入（暗态里才真正切视图+起游戏） */
static void enter_game_common(void (*dark_action)(void))
{
    if (lvgl_port_lock(100))
    {
        /* 【2026-08-19】这里【只】撤销挂起的图标渐暗渐亮定时器——它纯粹是
         * 取消一个 lv_timer，不碰任何 lv_obj、不产生失效区，故留在满亮度下无害。
         *
         * ⚠️ 图标的 HIDDEN 已挪进暗态（见 game_enter_dark_action）：
         * 原先在此处隐藏，是在【渐暗尚未开始、屏幕仍全亮】时把图标那块区域弄脏，
         * LVGL 立刻重绘一屏，用户实测反馈「变暗时看到刷新过程」正是这一次。
         * 顺序曾是：满亮度隐藏图标→刷一屏(可见)→才开始渐暗；
         * 现改为：先渐暗→全黑→隐藏图标+建游戏画面→渐亮，全程只有一次重绘且藏在黑屏里。 */
        home_icon_fade_cancel();
        lvgl_port_unlock();
    }
    /* 【2026-08-19】改为走渐变：渐暗 → 暗态里 games_start（整屏重绘藏在全黑）
     * → 渐亮。时长走 UI_GAME_ENTER_FADE_* 这档（100/40/150），比功能页短，
     * 为的是把跳一跳的入场动画尽量留在亮起后播（详见该组宏上方注释）。
     * game_enter_fade 内部有完整兜底：拿不到锁/已有渐变在跑都会退回硬切，
     * 功能不因动效缺失而失效。 */
    game_enter_fade(dark_action);
}

/* 三个游戏的功能盘入口（赛车/跳一跳首版为占位游戏，进去显示"敬请期待"） */
static void enter_whack(void) { enter_game_common(game_enter_dark_whack); }
static void enter_race(void) { enter_game_common(game_enter_dark_race); }
static void enter_jump(void) { enter_game_common(game_enter_dark_jump); }

/**
 * @brief 进功能盘渐变收尾：暗到底那一刻真正切图（GIF→功能盘图标），
 *        原 ui_home_enter 的切图逻辑原样搬到此处，仅由 ui_home_fade_timer_cb 在
 *        UI_HOME_FADE_OUT 阶段到底时调用一次。
 */
/* 退出功能盘的暗态切页动作，定义在下方（与 ui_func_layer_exit_to_main 相邻），
 * 此处前置声明供 ui_home_fade_timer_cb 调用。 */
static void ui_home_exit_apply(void);

static void ui_home_enter_apply(void)
{
    if (!lvgl_port_lock(100))
    {
        /* ★【2026-09-01】取锁失败也必须把闸门放掉。
         * 本函数是 s_home_enter_pending 唯一的清除点，若在这里静默 return，标志会
         * 永久残留为 true → main_idle_loop_active() 恒假 → 主界面 GIF 轮播与空闲
         * 舵机【再也起不来】，比原来的偶发多动一下严重得多。
         * 此时进盘本身也失败了（画面没切），维持主界面行为反而是正确的退化方向。 */
        s_home_enter_pending = false;
        ESP_LOGW(TAG, "进功能盘切页取锁失败，本次不切页（闸门已释放）");
        return;
    }
    ensure_menu_panel();
    s_view = UI_VIEW_HOME;
    s_home_idx = 0;
    s_home_animating = false;
    /* 进入时清掉可能残留的位移，确保图标居中 */
    if (s_home_icon)
        lv_obj_set_style_translate_x(s_home_icon, 0, 0);
    /* 2026-08-16：进功能盘同样必须 pause GIF，仅 HIDDEN 不停解码（详见
     * ui_function_menu_enter 里的注释）。功能盘是进各功能页的必经层，
     * 这里不 pause，后面进倒计时页也一样会被 GIF 抢线程。 */
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj);
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }
    home_render();
    lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    menu_kick_idle_timer();
    /* 【2026-09-01】闸门使命完成：s_view 已是 HOME、轮播 timer 已 pause，
     * 此后那三道判据靠 s_view 自己就拦得住，标志必须归位，否则下次退回主界面后
     * 轮播永远起不来（main_idle_loop_active 恒 false）。
     * 放在 unlock 之前、与 s_view 的置位同处一个临界区内，避免中间态被别的线程读到。 */
    s_home_enter_pending = false;
    lvgl_port_unlock();
    ESP_LOGI(TAG, "进入功能盘");
}

/**
 * @brief 进功能盘背光渐变 timer 回调：渐暗→暗到底切图→渐亮，与开机 logo 渐变
 *        （ui_boot_fade_timer_cb）同一套推进模式，独立 timer 互不干扰。
 */
static void ui_home_fade_timer_cb(lv_timer_t *t)
{
    if (s_home_fade_phase == UI_HOME_FADE_IDLE)
    {
        lv_timer_pause(t);
        return;
    }

    uint32_t elapsed_ms = (uint32_t)((esp_timer_get_time() - s_home_fade_start_us) / 1000);
    bool done;
    switch (s_home_fade_phase)
    {
    case UI_HOME_FADE_OUT:
        done = bsp_board_lcd_fade_step_fine(BSP_LCD_BK_DEFAULT_PCT, 0, elapsed_ms, UI_HOME_FADE_OUT_MS);
        if (done)
        {
            /* 暗到底：按方向分流（问题4）。切页全程藏在全黑里，看不到刷屏。 */
            if (s_home_fade_is_enter)
                ui_home_enter_apply(); // 进：隐藏 GIF + 显示图标
            else
                ui_home_exit_apply(); // 退：隐藏图标面板 + 恢复 GIF
            s_home_fade_phase = UI_HOME_FADE_DARK;
            s_home_fade_start_us = esp_timer_get_time();
        }
        break;

    case UI_HOME_FADE_DARK:
        /* 等 flush + 液晶响应走完，避免图标那帧在背光爬升时落地导致闪烁 */
        if (elapsed_ms >= UI_HOME_FADE_DARK_MS)
        {
            s_home_fade_phase = UI_HOME_FADE_IN;
            s_home_fade_start_us = esp_timer_get_time();
        }
        break;

    case UI_HOME_FADE_IN:
        done = bsp_board_lcd_fade_step_fine(0, BSP_LCD_BK_DEFAULT_PCT, elapsed_ms, UI_HOME_FADE_IN_MS);
        if (done)
        {
            s_home_fade_phase = UI_HOME_FADE_IDLE;
            lv_timer_pause(t);
        }
        break;

    default:
        lv_timer_pause(t);
        break;
    }
}

/**
 * @brief 进入功能盘界面（主界面长按耳/长按头 触发）
 *
 * 视觉流程：背光渐暗→（暗到底）切图（GIF→功能盘图标）→背光渐亮，
 * 由 s_home_fade_tmr 驱动，本函数只负责触发，不阻塞。
 */
void ui_home_enter(void)
{
    /* ★★【2026-09-01 修正：先关闸，再 flush —— 顺序不可颠倒】★★
     *
     * 【原注释错在哪】这里原本写着「若此刻正在播情绪：flush 打断情绪舵机 → worker
     *   give → interaction 解阻塞 → 调 ui_resume_main_gif_loop，但下面已把 s_view
     *   设为 HOME，故 resume 内部判 s_view!=MAIN 直接返回」。
     *   这句话在 2026-08-31【进功能盘改走背光渐变】之后就不成立了：s_view = UI_VIEW_HOME
     *   已被搬进 ui_home_enter_apply()，而那个函数要等渐暗 450ms 跑完才执行。于是
     *   flush 引发的解阻塞链路跑在「s_view 还是 MAIN」的窗口里，守卫全部失效，
     *   轮播被重新唤醒并投递出一条【新的】空闲舵机动作 —— 实测现象即
     *   「画面已经渐变到功能盘了，舵机偶尔还在动，动完才快速归中」。
     *
     * 【为什么必须放在 flush 之前】flush 是这条链路的触发源：worker 打断动作后立刻
     *   give 信号量，interaction 那边可能在几毫秒内就走到 ui_resume_main_gif_loop()。
     *   闸门晚一步置位，就有一拍的空窗，等于没修。
     *
     * 【为什么不能改成"提前置 s_view"】见 s_home_enter_pending 声明处的说明：
     *   渐变期 s_view 与画面不一致会让触摸按新视图分发，那是另一个坑。 */
    s_home_enter_pending = true;

    // 进功能盘立即清空舵机队列 + 打断当前动作 + 平滑归中（非阻塞，不碰 LVGL）。
    // 解决「进盘后 servo_manager 队列堆积的旧动作继续做、舵机还重复动多次」的 bug。
    servo_manager_flush();

    /* 渐变期间不响应重入（双耳/头连续触发）：已在渐变中直接忽略。
     * ⚠️ 这里【不能】把 s_home_enter_pending 清回 false：正在跑的那次渐变还没到暗态，
     * 闸门必须继续关着，由它自己的 ui_home_enter_apply() 负责清。 */
    if (s_home_fade_phase != UI_HOME_FADE_IDLE)
        return;

    /* 【2026-08-31 问题4】必须显式置 true：进/退共用同一套状态机，
     * 上一次若是退出会把标志留在 false，不置位则本次进入会误跑退出分支。 */
    s_home_fade_is_enter = true;

    /* 兜底：无 timer 时直接硬切进盘，不因动效缺失而进不去（与退出侧对称） */
    if (s_home_fade_tmr == NULL)
    {
        ui_home_enter_apply();
        return;
    }

    s_home_fade_phase = UI_HOME_FADE_OUT;
    s_home_fade_start_us = esp_timer_get_time();
    lv_timer_resume(s_home_fade_tmr);

    /* 【2026-08-31】与退出方向同款唤醒（原因详见 ui_func_layer_exit_to_main 末尾注释）。
     * 进入方向此前没加也"看着正常"，是因为主界面 GIF 在播，帧定时器会不断唤醒
     * taskLVGL，OUT 段顺带被推进 —— 属于撞运气，不是设计保证：GIF 恰好暂停或
     * 帧间隔较长时同样会退化成"瞬间黑屏"。这里补齐，让两个方向都不依赖运气。 */
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
}

/**
 * @brief 退出功能盘渐变收尾：暗到底那一刻真正切页（功能盘图标→主界面 GIF）
 *
 * 【2026-08-31 问题4】原来这段就是 ui_func_layer_exit_to_main 的全部内容，直接硬切，
 * 整屏刷屏过程完全暴露；现原样搬到此处，改由 ui_home_fade_timer_cb 在【全黑】时调用，
 * 与进功能盘（ui_home_enter_apply）完全对称。逻辑一行未改，只是执行时机变了。
 */
static void ui_home_exit_apply(void)
{
    /* 若正从游戏中退出，先释放游戏资源（幂等：非游戏态为空操作） */
    games_stop();
    if (!lvgl_port_lock(100))
        return;
    home_icon_fade_cancel(); /* 返回主界面，撤销挂起的图标渐暗渐亮 */
    if (s_menu_panel)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    if (gif_obj != NULL)
        lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    s_view = UI_VIEW_MAIN;
    menu_cancel_idle_timer();

    /* ★★【2026-09-01 修「退出功能盘后没有舵机动作 / 画面可能还停在情绪 GIF」】★★
     *
     * 【原来这里是什么】main_gif_kick_resume() —— 只 lv_gif_restart()，让 gif_obj
     *   当前那张【原地从头重播】。它有两个问题：
     *
     *   ① 【没有舵机动作】空闲态的舵机动作只在【切图那一刻】投递一次，唯一出口是
     *      main_gif_switch_timer_cb 里的 ui_interaction_play_custom(&s_idle_actions[idx])
     *      （见 :3486 附近）。restart 不经过切图路径，故一条动作都不投递 —— 表现为
     *      "画面回来了、舵机却一直停在中位"，要等这张播完切到下一张才恢复。
     *      （舵机为何在中位：进功能盘时 worker 做完上一条动作就归中，之后没有新动作。）
     *
     *   ② 【画面可能不是空闲 GIF】若进功能盘之前屏幕上停的是【触摸情绪 GIF】，
     *      restart 重播的就是那张情绪图。而情绪切图【故意不更新 s_gif_cur_index】
     *      （见 :3425：情绪 GIF 不在 s_main_gif_table 索引体系内），所以按索引补投
     *      只会得到「情绪 GIF 的画面 + 空闲表某张的动作」这种错配。
     *
     * 【正确语义（2026-09-01 用户确认）】退出功能盘那一刻，设备就是【空闲态】了：
     *   那次触摸情绪反馈在进功能盘时就已经结束，不该被带出来。所以退出后应当完整地
     *   呈现空闲态的样子 —— 【空闲轮播的 GIF + 该张对应的空闲动作】，成对出现。
     *   判断依据是"现在是空闲态"这个身份，而不是"屏幕上那张图长什么样"：哪怕情绪
     *   GIF 与空闲表里某张恰好是同一个文件，也照样按空闲体系走、放空闲的那条动作。
     *
     * 【改法】不再 restart 旧画面，改调 ui_resume_main_gif_loop() —— 这条现成路径
     *   干的正是这件事：清掉情绪 pending（s_gif_pending_path=NULL，确保走随机索引
     *   分支而非情绪指定分支）→ 用 main_gif_pick_next_index() 排一张空闲 GIF →
     *   resume 轮播 timer。下一拍 main_gif_switch_timer_cb 正常切图，并在同一处
     *   投递 s_idle_actions[idx] —— 画面与动作天然配套，走的就是平时轮播那条路，
     *   不需要在这里另外补投任何动作。
     *
     * 【为什么不必再判待机/远程控制/对话态】ui_resume_main_gif_loop() 内部已依次
     *   判 main_idle_loop_active() 与 s_neutral_active，其后的 timer_cb 还会再判
     *   standby_is_deep_active() / remote_control_is_active() / interaction_is_playing()
     *   （见 :3447~3477）。复用这条路径等于把那几道闸一并复用，比在此处自己抄一遍
     *   判据更不容易漏 —— 上一版就是自己抄判据才漏掉了"画面可能是情绪 GIF"。
     *
     * 【为什么此刻闸门放行】main_idle_loop_active() 要求
     *   s_view==MAIN && !s_home_enter_pending && !s_expire_fade_pending：
     *   上一行刚把 s_view 置回 MAIN；enter_pending 属于"进"方向（本函数是"退"）；
     *   expire_pending 是到期渐变的另一套状态机，此刻均为 false。三条全放行。
     *
     * 【线程】本函数已持 lvgl_port 锁；ui_resume_main_gif_loop() 只做指针/整型赋值
     *   与 lv_timer_resume，不上锁、不阻塞（见其函数头"跨线程安全"说明），
     *   真正的切图在下一拍由 LVGL 线程执行（BUG-010 要求）。 */
    ui_resume_main_gif_loop();

    lvgl_port_unlock();
    ESP_LOGI(TAG, "功能层 → 返回主界面（恢复空闲轮播 GIF + 配套舵机动作）");
}

/**
 * @brief 任意功能层 → 返回主界面（长按耳/长按头）
 *
 * 【2026-08-31 问题4】由「直接硬切」改为「背光渐变」，复用进功能盘的同一套
 * 三段状态机（渐暗 → 暗到底切页 → 渐亮）与同一组 UI_HOME_FADE_* 时长宏，
 * 故进出手感完全对称。真正的切页动作在 ui_home_exit_apply()，全程藏在全黑里。
 *
 * 【兜底】timer 不存在时直接硬切 —— 宁可没有动效，也不能因为动效缺失退不出去
 * （与 function_menu_exit_fade / countdown 渐变同策略，见各自注释）。
 */
/**
 * @brief 【2026-09-01 新增】功能层渐变退出的【内部启动器】（不震动）
 *
 * 原来这段就是 ui_func_layer_exit_to_main() 去掉震动之后的全部内容，原样搬出，
 * 逻辑一行未改。抽出来的原因：空闲超时（menu_idle_timeout_cb）也要走同一套渐变，
 * 但超时是设备自己退的、不该震动，而震动请求原先写死在长按入口里。
 * 现在：长按 = 本函数 + 一次震动请求；超时 = 只调本函数。
 */
static void func_layer_exit_fade_start(void)
{
    /* 渐变期间不响应重入（连续长按/双耳同触 / 长按与超时撞车）：已在渐变中直接忽略，
     * 与 ui_home_enter 的重入保护对称。 */
    if (s_home_fade_phase != UI_HOME_FADE_IDLE)
        return;

    if (s_home_fade_tmr == NULL)
    {
        ui_home_exit_apply(); /* 兜底：无 timer → 直接硬切，功能不因动效缺失而失效 */
        return;
    }

    s_home_fade_is_enter = false; /* 暗态动作 = ui_home_exit_apply()（退出） */
    s_home_fade_phase = UI_HOME_FADE_OUT;
    s_home_fade_start_us = esp_timer_get_time();
    lv_timer_resume(s_home_fade_tmr);

    /* ★★【2026-08-31 修「退出只见渐亮、不见渐灭」】★★
     * 【现象】长按退出 → 短暂等待 → 瞬间黑屏 → 渐亮。OUT 段的 500ms 确实过去了
     *   （那就是"短暂等待"），但屏幕全程保持满亮，最后一步才跳到全黑。
     * 【根因】s_home_fade_tmr 是 lv_timer，靠 taskLVGL 的 lv_timer_handler() 推进。
     *   lv_timer_resume() 只是把它挂回定时器链，而此刻 taskLVGL 正睡在
     *   xEventGroupWaitBits 上（最长约 1000ms，本工程恒有 1000ms 周期定时器），
     *   lvgl_port_unlock() 只 give 互斥量、不置事件位，叫不醒它。
     *   于是整个 OUT 段【一步都没推进】，背光停在满亮；等 taskLVGL 自己睡醒时，
     *   elapsed_ms 早已 ≥ UI_HOME_FADE_OUT_MS，bsp_board_lcd_fade_step_fine()
     *   走 bsp_lcd.c:158 的"已到终点"捷径直接落到 0 —— 这就是「瞬间黑屏」。
     *   之后 timer 已在正常运转，DARK/IN 逐步推进，所以「渐亮」是好的。
     * 【为什么进入方向没事】ui_home_enter() 的调用点在触摸分发里，那条分支末尾
     *   已经有 lvgl_port_task_wake()，顺带把渐变也带起来了；
     *   而长按退出这条分支只调本函数，没人唤醒 taskLVGL。
     *   —— 所以不是"没复用"，状态机和宏都是共用的，是退出侧少了这一脚唤醒。
     * 【改法】显式置事件位打断等待，让 OUT 首帧立刻开跑。
     *
     * ★【2026-09-01 补充】超时入口（menu_idle_timeout_cb）走到这里时【已经在
     *   taskLVGL 里】，本次 lv_timer_handler 轮次结束就会继续跑，这一脚唤醒是
     *   多余但无害的（只是置一次事件位）；长按入口跑在触摸任务，则必须要它。 */
    lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
}

void ui_func_layer_exit_to_main(void)
{
    /* 渐变期间不响应重入（连续长按/双耳同触）：已在渐变中直接忽略，
     * 与 ui_home_enter 的重入保护对称。
     * ★ 这里必须【先判一次】再震动：震动只能在"本次长按真的生效"时发生，
     *   而真正的重入拒绝在 func_layer_exit_fade_start() 里（判的是同一个标志）。 */
    if (s_home_fade_phase != UI_HOME_FADE_IDLE)
        return;

    /* ★【2026-08-31】震动跟随「真正退出」而非「触摸」。
     * 触摸层已把功能盘的头部排除（bsp_touch.c 的 head_vib_view），改由此处发起：
     * 越过上面那道重入拒绝才算本次长按真的生效，此时震一下才名副其实。
     * 放在重入检查【之后】、两条退出路径（渐变 / 无 timer 兜底）【之前】，
     * 两条路都能覆盖到，且都只震一次。
     * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞，本函数由触摸任务调用，
     * 真正打点在触摸任务循环末尾（同左右耳、同头部进入）。 */
    bsp_touch_request_vibrate();

    /* 渐变本体（含无 timer 兜底硬切、重入保护、taskLVGL 唤醒）全部在这里，
     * 与空闲超时退出走的是同一份代码。 */
    func_layer_exit_fade_start();
}

bool ui_force_back_to_main(void)
{
    /* 强制从任意视图（含闹钟编辑）回到主界面：供待机模块在进入低功耗时调用，
     * 避免「已待机降亮度但界面仍卡在某菜单/编辑页」。
     * 与 ui_func_layer_exit_to_main 的区别：额外隐藏闹钟编辑面板，覆盖全部视图。
     *
     * 关键：若当前停在游戏视图，必须先释放游戏资源（面板/定时器）。
     * 否则游戏面板（挂在 active screen 上的独立对象）不会随 s_menu_panel 一起隐藏，
     * 会浮在最上层挡住 GIF，且其定时器继续跑、s_view 被改成 MAIN 后再也走不到
     * UI_VIEW_GAME 分支去清理 → 表现为「进出低功耗 / 静置后画面卡死、触摸全失效」。
     * games_stop() 幂等且自带 LVGL 递归锁，非游戏态为空操作，放锁外调用安全。 */
    games_stop();
    // 清掉进入功能层/游戏前残留的舵机请求 + 打断当前动作 + 归中（非阻塞，不碰 LVGL）。
    servo_manager_flush();

    /* ★★【2026-08-25：改为返回复位是否成功，取锁超时不再静默吞掉】★★
     * 【修的问题】原先取锁失败直接 return void，调用方（enter_deep_standby）
     *   完全无从知晓复位没做成，仍会继续往下走 ui_standby_clock_show() 把
     *   待机时钟叠上去 —— 实测现象＝「闹钟编辑页 + 低功耗时间页同框」。
     *   闹钟响铃结束会进【闹钟编辑页】（alarm_ring_reset_cb → alarm_edit_enter_ex），
     *   它用的是独立的 s_edit_panel，而 ui_standby_clock_show() 只藏
     *   s_menu_title/body/home_icon，压根不管 s_edit_panel，于是两层直接叠在一起。
     * 【改法】把"复位成功与否"如实返回，由调用方决定是整轮放弃还是继续，
     *   把「要么全复位、要么不进待机」这条原子性交还给调用方保证。
     * 【取锁超时 100→300ms】进待机时 GIF/舵机/响铃可能正忙，100ms 偏紧；
     *   本函数在 standby_task 里调用，多等 200ms 无副作用，能显著降低失败率。 */
    if (!lvgl_port_lock(300))
    {
        ESP_LOGW(TAG, "强制返回主界面：取 LVGL 锁超时，本次复位未执行");
        return false;
    }

    /* ★【2026-08-31 问题4 配套】撤销可能正在跑的功能盘进/退渐变（须持锁，故放在锁内）。
     * 【为什么必须加】退出功能盘改走背光渐变后，若渐变途中待机/低功耗触发本函数强制
     *   切回主界面，渐变仍会继续推进：暗态那一步会再执行一次切页，渐亮那一步会把背光
     *   拉回 BSP_LCD_BK_DEFAULT_PCT —— 直接把待机刚降下去的亮度顶亮，
     *   表现为「进了待机屏幕又自己亮起来」。进入方向同理（会把功能盘切回来）。
     * 【为什么只清标志不改亮度】本函数的调用方（enter_deep_standby）随后会自行设定
     *   目标亮度，这里不碰背光，避免两边抢着写。 */
    s_home_fade_phase = UI_HOME_FADE_IDLE;
    if (s_home_fade_tmr != NULL)
        lv_timer_pause(s_home_fade_tmr);
    /* 【2026-09-01】渐变被撤销 ⇒ ui_home_enter_apply() 不会再执行 ⇒ 闸门没人清。
     * 不在此补一刀，主界面 GIF 轮播会永久停摆（详见该标志声明处）。 */
    s_home_enter_pending = false;

    if (s_view == UI_VIEW_MAIN)
    {
        /* ★★【2026-09-01 配套：到期渐变也必须在这条早退分支里撤销】★★
         * 【为什么本分支够不着下面那句 countdown_exit_fade_cancel()】新加的
         *   「主界面 → 闹钟/番茄钟到期画面」渐变，其 s_view 要到【暗态】才切成
         *   FUNCTION_MENU；渐暗那 450ms 里 s_view 仍是 MAIN，于是走到本分支就
         *   提前 return 了，8945 那句根本执行不到。
         * 【不补会怎样】渐变继续推进 → 暗态把到期画面切到刚进待机的屏幕上（与
         *   待机时钟叠加）→ 渐亮再把背光顶回 100%，即"进了待机屏幕又自己亮起来"。
         *   这正是上面 s_home_fade 那段注释治过的同一类病，只是换了一套状态机。
         * 【为何只清状态机不碰背光】与 s_home_fade 的处理一致：调用方
         *   （enter_deep_standby）随后会自行设定目标亮度，两边抢着写会打架。
         *   故这里不调 countdown_exit_fade_cancel()（它内部会拉满背光）。 */
        s_cd_exit_fade_phase = UI_CD_EXIT_FADE_IDLE;
        if (s_cd_exit_fade_tmr != NULL)
            lv_timer_pause(s_cd_exit_fade_tmr);
        s_cd_fade_dark_cb = NULL;
        s_cd_fade_is_enter = false;
        s_expire_fade_pending = false;

        /* 已在主界面：仍要兜底藏一次两个面板。理由是"视图标志"与"面板可见性"
         * 理论上应当同步，但历史上出现过被强制切页/取锁失败打断而脱节的情况，
         * 这里花两次 add_flag 的代价把它们强行对齐，避免残留层叠在 GIF 上。 */
        if (s_edit_panel)
            lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (s_menu_panel)
            lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
        lvgl_port_unlock();
        return true; /* 已在主界面，视为复位成功 */
    }
    home_icon_fade_cancel(); /* 待机强制回主界面，撤销挂起的图标渐暗渐亮 */
    /* 清掉两个页面级一次性定时器（闹钟保存闪烁 / 倒计时延迟退出）：
     * 待机强制回主界面时它们可能仍在跑，留着会在主界面上误触发退出/保存。均幂等。
     * 闹钟若被打断在"闪烁中"，说明用户已经长按确认过保存，只是动效没播完，
     * 这里补一次 commit，避免确认过的闹钟被待机静默吞掉。 */
    if (alarm_edit_blink_cancel())
        alarm_edit_commit();
    alarm_exit_fade_abort();  /* 闹钟退出渐变（切页后的暗态/渐亮）若在跑，一并中止并复位背光 */
    alarm_edit_zoom_cancel(); /* 「1秒后转场」定时器同属页面级一次性定时器，一并清掉 */
    countdown_exit_delay_cancel();
    countdown_exit_fade_cancel(); /* 停留计时之后的退出渐变若也在跑，一并打断，避免背光卡在半暗 */
    if (s_edit_panel)
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN); /* 闹钟编辑面板 */
    if (s_menu_panel)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN); /* 功能/菜单面板 */
    if (gif_obj != NULL)
        lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    s_view = UI_VIEW_MAIN;
    main_gif_kick_resume();
    menu_cancel_idle_timer();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "待机 → 强制返回主界面");
    return true;
}

/**
 * @brief 返回功能盘（在功能页/游戏里摸腹/背时调用）
 *
 * 单层结构下，所有功能页/游戏的上一层都是功能盘。复位位移并重新渲染。
 */
static void back_to_home(void)
{
    if (!lvgl_port_lock(100))
        return;
    /* 清掉倒计时页可能仍在跑的「停留后退出」定时器：
     * 若用户在停留期间自己摸腹/背返回，定时器留着会在之后误触发一次退出。
     * 幂等，非倒计时路径为空操作。
     *
     * ⚠️ 这里【不能】调 countdown_exit_fade_cancel()：本函数正是退出渐变
     * 「暗到底切页」那一步要调用的目标（countdown_exit_fade_timer_cb →
     * ui_function_menu_exit → 本函数），若在此取消渐变，渐变会把自己打断——
     * phase 被清成 IDLE + timer 被 pause，回调返回后又把 phase 覆盖成 DARK，
     * 于是状态永久卡在 DARK 且无人推进：第一次表现为「渐亮被跳过、亮度瞬间拉满」，
     * 第二次因 phase != IDLE 直接 return，倒计时再也不会自动退出。
     * 外部打断的清理放在触摸分发入口（见 ui_dispatch_touch_event）统一处理。 */
    countdown_exit_delay_cancel();
    s_view = UI_VIEW_HOME;
    s_home_animating = false;
    if (s_home_icon)
        lv_obj_set_style_translate_x(s_home_icon, 0, 0);
    home_render();
    menu_kick_idle_timer();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "返回功能盘");
}

/**
 * @brief 从功能盘进入指定功能页（复用现有功能菜单逻辑）
 *
 * 直接复用 render_fn_page() 渲染目标页，并把视图切到 UI_VIEW_FUNCTION_MENU，
 * 这样翻页/编辑等既有逻辑全部沿用，零重写。
 */
static void menu_enter_fn_page(fn_page_t page)
{
    if (!lvgl_port_lock(100))
        return;
    s_view = UI_VIEW_FUNCTION_MENU;
    s_fn_page = page;
    render_fn_page(s_fn_page);
    menu_kick_idle_timer();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "进入功能页: %s", s_fn_page_titles[page]);
}

static void app_enter_time(void) { menu_enter_fn_page(FN_PAGE_TIME); }
static void app_enter_calendar(void) { menu_enter_fn_page(FN_PAGE_CALENDAR); }
/* 闹钟应用：从应用列表摸头确认后，直接进入闹钟编辑（去掉中间的只读显示页），
 * 用左右耳调值、头部下一步/确认、腹背放弃，符合统一三键模型。 */
static void app_enter_alarm(void) { alarm_edit_enter(); }
static void app_enter_countdown(void) { menu_enter_fn_page(FN_PAGE_COUNTDOWN); }
static void app_enter_weather(void) { menu_enter_fn_page(FN_PAGE_WEATHER); }

/**
 * @brief 供子模块（games.c）渲染纯文字画面：标题 + 正文
 */
void ui_menu_show_text(const char *title, const char *body)
{
    if (!lvgl_port_lock(100))
        return;
    if (s_menu_title && title)
        lv_label_set_text(s_menu_title, title);
    if (s_menu_body)
    {
        menu_clear_func_pages();
        lv_label_set_text(s_menu_body, body ? body : "");
    }
    lvgl_port_unlock();
}

/**
 * @brief 解绑/出厂重置前切到纯黑屏（取消绑定前调用）
 *
 * 背景（为何需要本接口）：
 *   解绑流程 clear_wifi_and_restart() 会做 NVS 擦除 + esp_wifi_restore，
 *   这些 flash 写操作会短暂禁用 flash cache。而主界面 GIF 每帧都要读 SPIFFS(flash)，
 *   cache 一禁用就取不到下一帧，画面便「卡在当前帧」直到 esp_restart 黑屏，体验像死机。
 *
 * 做法：
 *   在动 flash 之前先隐藏 GIF、并用 lv_refr_now() 同步刷出纯黑底。黑底由 screen 自身
 *   背景色渲染，只画一次、不读 flash，因此后续 cache 被禁用也不影响显示。
 *
 * ★不再显示「已重置...」文字（原先复用 s_menu_panel 显示）：本画面仅存在几百毫秒
 *   （擦 NVS→esp_restart），随后重启进入未配网分支会显示配网提示图 pw.bin
 *   （见 ui_show_provision_image）。黑屏→配网图的过渡比闪一行文字更干净。
 *   但【隐藏 GIF 的保护逻辑必须保留】——去掉它就会退化成 cache 禁用后的冻帧。
 *
 * @note 必须在调用方真正擦除 NVS / 重启【之前】调用，否则 cache 已禁用、刷不上屏。
 */
void ui_show_unbinding(void)
{
    /* LVGL 未就绪（如 WiFi 还在重连、屏幕尚未初始化）时直接返回：
     * 否则 lvgl_port_lock 会因 lvgl_mux 为空而 assert→abort，导致设备在清除
     * WiFi 凭证之前就重启（表现为「按键和 RST 一样只重启、密码没清掉」）。 */
    if (!s_lvgl_ready)
    {
        ESP_LOGW(TAG, "LVGL 未就绪，跳过解绑提示页（不影响后续 WiFi 凭证清除）");
        return;
    }

    /* 超时给足 1s：此刻系统正准备重启，宁可多等也要确保提示能刷上屏 */
    if (!lvgl_port_lock(1000))
        return;

    /* 1) 隐藏会逐帧读 flash 的 GIF，避免它在 cache 禁用后留下冻帧。
     * 2026-08-16：补 lv_gif_pause —— 仅 HIDDEN 不停内部解码 timer，解绑要擦 NVS/
     * 禁用 flash cache，此时 GIF 仍在逐帧读 flash 极其危险（同 ui_show_ota_progress
     * 的处理）。switch timer 也一并 pause，断掉「ready → resume → 切图」唤醒环。 */
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj);
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }

    /* 2) 藏掉可能正显示的功能菜单面板，确保屏上只剩纯黑底（不再写提示文字） */
    if (s_menu_panel != NULL)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);

    /* 3) 同步刷新：在解锁返回前把这一帧真正画到 LCD 上。
     *    关键——必须趁现在 flash cache 还可用时完成渲染，否则调用方一旦开始
     *    擦 NVS，cache 被禁用，这张提示页就再也刷不上去了。 */
    lv_refr_now(NULL);

    lvgl_port_unlock();
    ESP_LOGW(TAG, "解绑：已切纯黑屏（GIF 已隐藏，等待重启进配网）");
}

/**
 * @brief 显示「固件升级中」进度页（OTA 下载过程中调用，跨线程安全）
 *
 * 与 ui_show_unbinding() 同机制：隐藏逐帧读 flash 的 GIF、复用全屏黑底文字面板，
 * 独占整块屏幕给出明确的「升级中」反馈，避免用户误以为设备卡死。
 *
 * 由 bsp_ota 的下载任务在下载前及每次进度上报时调用（非 LVGL 线程），函数内部
 * 自持 LVGL 锁；取锁失败（如正忙）则直接跳过本次刷新，无害退化——下次进度回调
 * 会再刷一次。
 *
 * @param pct 下载进度百分比（0~100）
 */
void ui_show_ota_progress(int pct)
{
    if (!s_lvgl_ready)
        return;
    if (pct < 0)
        pct = 0;
    if (pct > 100)
        pct = 100;

    if (!lvgl_port_lock(100))
        return; // 取锁失败：跳过本次刷新，等下次进度回调再刷（无害）

    /* 隐藏 GIF，避免它在升级期间继续逐帧读 flash 与 OTA 抢 flash/CPU。
     * ★ 关键：仅 HIDDEN 不停解码——GIF 内部逐帧 timer 仍在跑，播完一轮照样发
     *   LV_EVENT_READY，被 main_gif_ready_cb 捕获后又 lv_timer_resume(s_gif_switch_tmr)，
     *   于是 switch timer 被反复唤醒继续切图/解码/读 flash，直到 esp_restart() 那一刻
     *   正好卡在 gif_blend_to_rgb565 导致重启崩溃、屏幕定死、新固件切换被干扰。
     *   因此这里必须同时 lv_gif_pause 冻结内部解码 timer + 暂停 switch timer 排队，
     *   双保险彻底断掉「ready → resume → 切图」的自我唤醒环。 */
    if (s_gif_switch_tmr != NULL)
        lv_timer_pause(s_gif_switch_tmr);
    if (gif_obj != NULL)
    {
        lv_gif_pause(gif_obj); // 冻结 GIF 内部逐帧解码 timer，从根上不再发 READY 事件
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    }

    /* 复用功能菜单全屏黑底面板，写入升级进度文案 */
    ensure_menu_panel();
    if (s_menu_panel != NULL)
    {
        menu_clear_func_pages(); // 清掉功能页专属对象，仅留 title+body
        if (s_menu_title)
        {
            lv_label_set_text(s_menu_title, "固件升级中");
            // 标题临时移到屏幕中心偏上（默认是 TOP_MID）；面板复用，别处会各自重设对齐
            lv_obj_align(s_menu_title, LV_ALIGN_CENTER, 0, -20);
        }
        if (s_menu_body)
        {
            char buf[16];
            snprintf(buf, sizeof(buf), "%d%%", pct);
            lv_label_set_text(s_menu_body, buf);
            lv_obj_align(s_menu_body, LV_ALIGN_CENTER, 0, 12); // 百分比在中心偏下
        }
        lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    }

    // 不调 lv_refr_now()：同步强刷会阻塞本函数直到整帧渲染完成，OTA 下载中反复调用
    // 既拖慢下载、又可能与已暂停的任务状态冲突卡住。改由 LVGL 自身刷新任务异步渲染即可，
    // 只需更新完 label 文本后立即释放锁返回。
    lvgl_port_unlock();
}

/* ═══════════════════════════════════════════════════════════════
 * UI 系统初始化
 * ═══════════════════════════════════════════════════════════════ */
// 状态栏（电池/WiFi/时间）相关函数前置声明
// 加 __attribute__((unused))：当 UI_SHOW_STATUS_BAR=0 时这几个回调不再被调用，
// 避免 -Wunused-function 警告（status_wifi_refresh 仍被 ui_update_wifi 引用，无需标注）
static void battery_label_create_top(void) __attribute__((unused));
static void battery_tick_cb(lv_timer_t *t) __attribute__((unused));
static void status_time_tick_cb(lv_timer_t *t) __attribute__((unused));
static void status_wifi_refresh(int rssi);

/**
 *测试
 */
void lv_set_full_screen_color(uint8_t r, uint8_t g, uint8_t b)
{
    lv_obj_t *scr = lv_scr_act();
    lv_obj_set_style_bg_color(scr, lv_color_make(r, g, b), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(scr, 0, LV_PART_MAIN);
    lv_obj_set_style_border_width(scr, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(scr, 0, LV_PART_MAIN);
}

/* ═══════════════════════════════════════════════════════════════
 * 配网提示图（未配网分支专用，早于 WiFi/BLE 显示）
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief 点亮屏幕并全屏显示配网提示图 pw.bin（320×240，外挂 flash S:/img/）
 *
 * 仅在【首次配网】场景调用：设备读 NVS 发现无 WiFi 凭证 → 启动 BluFi 前先给
 * 用户一张"设备未连接，请连接 APP"的静态提示图，避免配网全程黑屏。
 *
 * ★为什么可以早于 BLE：本函数只贴一张静态图，flush 完整屏后 LVGL 即彻底静默
 *   （无 GIF 解码、无动画 timer、无周期性刷屏），不会与 BT controller 使能窗口
 *   抢内部资源。真正危险的是 GIF 轮播那类持续刷屏负载，它仍留在 ui_init()
 *   中、位于配网完成之后（见 bsp_wifi.c 未配网分支注释与 BUG-026）。
 *
 * ★幂等：内部调用的 bsp_board_lcd_init/app_lvgl_init 均带早退标志，配网结束后
 *   主流程 application.c 再次初始化时会自动跳过。
 *
 * ★本函数贴的图【必须显式销毁】，句柄存于 s_provision_img，由 ui_init() 在创建
 *   主界面 UI 之前删除。曾经这里写的是"由后续 main_gif_create/main_desplay_create
 *   覆盖，无需手动销毁"——那是错的：主界面 GIF 铺满整屏确实能盖住它，但功能盘
 *   是【居中大图标、不铺满】，盖不住的四角会把这张全屏图露出来（表现为"功能盘
 *   四个角落有白色露出"，且仅首次配网那一程出现、RST 后消失）。
 *
 * @return 无（任何一步失败仅记日志，绝不阻断配网主流程——没图也要能配网）
 *
 * @note 调用者：bsp_board_wifi_main() 未配网分支，esp_wifi_start() 之前
 * @note 前置条件：bsp_board 单例已创建（步骤 1）；不要求 NVS 之外的任何模块
 */
void ui_show_provision_image(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL)
    {
        ESP_LOGE(TAG, "配网图：bsp_board 单例为空，跳过显示");
        return;
    }

    // ── 步骤 1：初始化 LCD 硬件（SPI + panel + 背光 PWM，背光此时仍为 0）────
    bsp_board_lcd_init(board);

    // ── 步骤 2：初始化 LVGL（含 draw buffer + taskLVGL）──────────────────────
    esp_err_t ret = app_lvgl_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "配网图：LVGL 初始化失败 (0x%x)，配网继续但无提示图", ret);
        return;
    }

    // ── 步骤 3：全屏贴图 ────────────────────────────────────────────────────
    // pw.bin 与赛车背景 c8.bin 同规格（320×240 LVGL bin），只显示一次且不刷新，
    // 故直接走 LVGL 文件系统直读，无需像 c8 那样预读进 PSRAM 缓存。
    if (lvgl_port_lock(1000))
    {
        lv_obj_t *screen = lv_screen_active();
        if (screen != NULL)
        {
            /* 句柄存到 s_provision_img，供 ui_init() 在建主界面前显式删除 */
            s_provision_img = lv_image_create(screen);
            lv_image_set_src(s_provision_img, UI_PROVISION_IMG_PATH);
            lv_obj_center(s_provision_img);
            ESP_LOGI(TAG, "配网提示图已显示: %s", UI_PROVISION_IMG_PATH);
        }
        else
            ESP_LOGE(TAG, "配网图：活动屏幕为空，跳过贴图");
        lvgl_port_unlock();
    }
    else
        ESP_LOGE(TAG, "配网图：获取 LVGL 锁超时，跳过贴图");

    // ── 步骤 4：开背光（放在贴图之后，避免用户先看到黑屏或残影）──────────
    if (lvgl_port_lock(1000))
    {
        bsp_board_lcd_on(board);
        lvgl_port_unlock();
    }
}

void ui_init(void)
{
    // init_spiffs();
    esp_err_t ret = app_lvgl_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "LVGL 初始化失败 (0x%x)，跳过 UI 创建", ret);
        return; // LVGL 不可用，跳过所有 UI 操作，避免访问 NULL 对象崩溃
    }

    { /* LVGL FS 诊断 */
        lv_fs_file_t f;
        lv_fs_res_t res = lv_fs_open(&f, "S:/1_3.gif", LV_FS_MODE_RD);
        ESP_LOGI("DIAG", "lv_fs_open result: %d (0=OK)", (int)res);
        if (res == LV_FS_RES_OK)
        {
            uint32_t size = 0;
            lv_fs_seek(&f, 0, LV_FS_SEEK_END);
            lv_fs_tell(&f, &size);
            ESP_LOGI("DIAG", "LVGL FS 文件大小: %lu bytes", (unsigned long)size);
            lv_fs_close(&f);
        }
    }

    if (lvgl_port_lock(1000))
    {
        /* ── 先删掉首次配网贴的全屏提示图，再建主界面 ──────────────────────
         * 仅首次配网路径非 NULL（已配网设备恒为 NULL，本段等同空操作）。
         * 必须删：它不铺满时会从功能盘四角露出来，且常驻底层拖慢每次刷屏。
         * 时机安全：此刻主界面尚未创建、屏幕内容随即被 main_gif_create 重画；
         * 线程安全：本段已持 lvgl_port_lock，符合"LVGL 对象只在持锁时操作"。 */
        if (s_provision_img != NULL)
        {
            lv_obj_del(s_provision_img);
            s_provision_img = NULL;
            ESP_LOGI(TAG, "已删除首次配网提示图 → 释放底层全屏图层");
        }

        main_gif_create();
        if (gif_obj)
            ESP_LOGI("DIAG", "gif_is_loaded=%d", lv_gif_is_loaded(gif_obj));
        main_desplay_create();
        lvgl_port_unlock();
    }

    if (lvgl_port_lock(100))
    {
        s_main_tick_tmr = lv_timer_create(main_clock_tick_cb, 1000, NULL);
        lvgl_port_unlock();
    }
    // lv_set_full_screen_color(0, 255, 255);

#if UI_SHOW_STATUS_BAR
    /* ─── 全局浮动状态栏（时间+WiFi+电量，顶层 layer，跨页常驻）─── */
    if (lvgl_port_lock(100))
    {
        battery_label_create_top();
        // 电量+WiFi 5 秒刷新一次（WiFi RSSI 拉取开销极小，与电量同 tick 节省一个 timer）
        s_battery_tick_tmr = lv_timer_create(battery_tick_cb, 5000, NULL);
        battery_tick_cb(s_battery_tick_tmr);
        // 状态栏时间 1 秒刷新一次（只显示 HH:MM，开销极小）
        s_status_time_tmr = lv_timer_create(status_time_tick_cb, 1000, NULL);
        status_time_tick_cb(s_status_time_tmr);
        lvgl_port_unlock();
    }
#endif /* UI_SHOW_STATUS_BAR */
}

/* ═══════════════════════════════════════════════════════════════
 * 触摸位置 → 情绪组映射（情绪队列表落地）
 *
 * 来源：用户「情绪队列表」（6 个检测位置 × 各 6 情绪）。每次触摸某位置时，
 * 从对应组里【随机挑一个情绪】交给 ui_interaction_play 播放（切 GIF + 舵机
 * + 震动 + 音频）。同情绪复用同一套 GIF/震动（见 interaction.c g_emotion_matrix）。
 *
 * 注：背部位置后续可能去除，届时删 emo_group_back / 头+背组的对应分支即可。
 * ═══════════════════════════════════════════════════════════════ */
/* ── 【调试开关】锁定触摸触发的情绪（2026-09-04 新增）────────────────────────
 * 情绪是从组里随机抽的（下方 play_random_emotion 的 esp_random）。调某一个情绪
 * 的动作序列时，摸半天碰不上、碰上了也不确定是不是它，没法验证。
 *
 * 打开下面这行 → 摸任意位置都播这一个情绪，专心调它；调完注释掉即恢复随机。
 * 可填任意 robot_emotion_t 枚举值，如 EMO_HAPPY / EMO_ANGRY / EMO_EXCITED。
 * ─────────────────────────────────────────────────────────────────────────── */
// #define IA_DEBUG_FORCE_EMOTION EMO_HAPPY

static const robot_emotion_t emo_group_head[] = {
    EMO_HAPPY, EMO_CURIOUS, EMO_TSUNDERE_BASE, EMO_TICKLISH, EMO_SLEEPY, EMO_GRIEVED};
static const robot_emotion_t emo_group_abdomen[] = {
    EMO_COMFORTABLE, EMO_ACT_CUTE, EMO_ANGRY, EMO_SHY, EMO_SURPRISED, EMO_SLUGGISH};
static const robot_emotion_t emo_group_back[] = {
    EMO_HEALING, EMO_TSUNDERE_BASE, EMO_GRIEVED, EMO_EXCITED, EMO_CURIOUS, EMO_TICKLISH};
static const robot_emotion_t emo_group_head_abdomen[] = {
    EMO_EXCITED, EMO_SHY_RUB, EMO_COMFORTABLE_ROLL, EMO_TSUNDERE_PET, EMO_SLEEPY, EMO_SURPRISED};
static const robot_emotion_t emo_group_head_back[] = {
    EMO_HEALING, EMO_TSUNDERE_BASE, EMO_GRIEVED, EMO_EXCITED, EMO_CURIOUS, EMO_TICKLISH};
static const robot_emotion_t emo_group_abdomen_back[] = {
    EMO_SLUGGISH_SIT, EMO_SURPRISED_HUG, EMO_TICKLISH_WIGGLE, EMO_COMFORTABLE, EMO_ANGRY, EMO_EXCITED};

/**
 * @brief 从情绪组里随机挑一个并触发播放（切 GIF + 舵机 + 震动 + 音频）
 * @param group 情绪组数组
 * @param count 组内情绪个数
 */
static void play_random_emotion(const robot_emotion_t *group, size_t count)
{
    if (group == NULL || count == 0)
        return;
    // 情绪/对话状态动作播放中：屏蔽新的情绪触摸（丢弃，不排队不打断，保证当前动作完整播完）。
    // 但「空闲动作」播放中【不屏蔽】——触摸情绪优先级高于空闲，应能打断空闲（由
    // ui_interaction_play 内部 flush 打断当前空闲动作后再入队本情绪）。
    // 注意：进功能盘的长按耳事件不走本函数，故不受屏蔽影响。
    if (interaction_is_playing() && !interaction_is_idle_action())
    {
        ESP_LOGI("TOUCH", "情绪/状态动作播放中，屏蔽本次触摸");
        return;
    }
    // 对话中（LISTENING/PLAYING）屏蔽情绪触摸：不播情绪、不切情绪图、不动舵机/震动，
    // 保证对话全程屏幕只显示状态中性 GIF，零抖动（用户明确诉求）。
    // 长按进功能盘的耳部事件不走本函数，不受影响。
    if (session_get_state() != SESSION_IDLE)
    {
        ESP_LOGI("TOUCH", "对话进行中，屏蔽情绪触摸（保持状态中性 GIF）");
        return;
    }
#ifdef IA_DEBUG_FORCE_EMOTION
    /* ★调试用：锁定情绪（2026-09-04 新增）
     * 情绪是从组里随机抽的，调某一个情绪的动作序列时根本碰不上、也不知道
     * 这次抽中的是不是它，无法验证。定义本宏后摸哪个位置都播指定情绪。
     * 调完把宏那行注释掉即恢复随机，见本文件上方 IA_DEBUG_FORCE_EMOTION 定义处。 */
    robot_emotion_t emo = IA_DEBUG_FORCE_EMOTION;
    ESP_LOGW("TOUCH", "【调试】情绪已锁定为 %d（IA_DEBUG_FORCE_EMOTION 生效，随机被跳过）", (int)emo);
    (void)group;
    (void)count;
#else
    robot_emotion_t emo = group[esp_random() % count];
    ESP_LOGI("TOUCH", "触摸触发情绪: %d（组内随机 %u 选 1）", (int)emo, (unsigned)count);
#endif
    ui_interaction_play(emo);
}

/* 数组长度宏（组定长 6，这里通用计算以防后续增减情绪） */
#define EMO_GROUP_LEN(g) (sizeof(g) / sizeof((g)[0]))

/* ═══════════════════════════════════════════════════════════════
 * 触摸事件分发（极简版：单闹钟 + 单倒计时）
 * ═══════════════════════════════════════════════════════════════ */
ui_view_t ui_get_current_view(void) { return s_view; }

void ui_dispatch_touch_event(touch_event_t event)
{
    if (event == TOUCH_EVENT_NONE)
        return;

    /* 任意触摸都是「真实交互」，立刻打断 app 远程控制的 30s 冻结窗口，恢复空闲序列。
     * center_servo=false（不归中）：本次触摸紧接着就会播情绪动作（或翻页/进菜单），
     * bsp_servo_move_smooth 自带「从当前角度插值到目标角」，让后续动作直接接管即可；
     * 若先归中会多出一次无意义的复位抽动。幂等，非远程控制态调用无副作用。 */
    remote_control_cancel(/*center_servo=*/false);

    /* 先记下「本次触摸发生时是否处于待机」。必须在 notify 之前取，
     * 因为下面的 standby_notify_activity() 会顺带唤醒、把待机标志清掉。 */
    bool was_standby = standby_is_deep_active();

    /* ★★【2026-08-31 二次修正：耳朵唤醒的震动也必须在唤醒【之前】立即震】★★
     * 【上一版为什么摸不到】上一版把 bsp_touch_request_vibrate() 放在下面
     *   standby_notify_activity() 之【后】的 was_standby 分支里，它只置标志，
     *   真正打点在触摸任务循环末尾 —— 也就是等 2.6s 阻塞转场跑完才震，
     *   且只有 30ms，短到手上没感觉。与身体铜箔那条完全同一个病因
     *   （详见 bsp_touch.c 待机唤醒块内的完整说明）。
     * 【改法】与身体铜箔统一：在这里、在唤醒转场开始之前，同步震一次足够长的脉冲。
     *   必须先 bsp_motor_ledc_init() 把被 enter_deep_standby 的 ledc_stop
     *   禁用掉的通道恢复出来，否则写 duty 出不了波形（幂等，重复配置安全）。
     * 【为何不放在下面 was_standby 分支里】那个分支在 standby_notify_activity()
     *   之后，转场已经跑完，时机就晚了。必须抢在 notify 之前。 */
    if (was_standby)
    {
        bsp_motor_ledc_init(); // 幂等：恢复被 ledc_stop 禁用的马达通道
        bsp_motor_pulse_level(BSP_MOTOR_DEFAULT_STRENGTH, BSP_TOUCH_WAKE_VIB_MS);
    }

    /* 任意触摸都算「活动」：刷新待机倒计时，且若在待机中则一并退出待机
     * （头部/腹背/左右翻页等任意部位皆可唤醒，唤醒逻辑收口在此函数内部）。 */
    standby_notify_activity();

    /* 待机被本次触摸唤醒时，消费掉这次事件：只做「唤醒」一件事，
     * 不再继续触发翻页/情绪/菜单等动作，避免“边唤醒边翻页”。 */
    if (was_standby)
    {
        /* 【震动不在这里】已在本函数开头、standby_notify_activity() 之前同步震过
         * （见那里的说明）。放这里会晚 2.6s（转场阻塞）且时长只有 30ms，摸不出来。 */
        ESP_LOGI(TAG, "触摸唤醒，退出待机（事件 %d，震动已在唤醒前发起）", (int)event);
        return;
    }

    /* 最高优先级：闹钟响铃中，任意触摸关闭闹钟 */
    if (reminder_get_state() == REMINDER_STATE_RINGING)
    {
        reminder_alarm_dismiss();
        if (lvgl_port_lock(100))
        {
            /* 2026-08-16：补 pause —— 关闹钟后会进闹钟编辑页（功能层），
             * 仅 HIDDEN 会让 GIF 内部解码 timer 继续抢 LVGL 线程。 */
            if (s_gif_switch_tmr != NULL)
                lv_timer_pause(s_gif_switch_tmr);
            if (gif_obj)
            {
                lv_gif_pause(gif_obj);
                lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
            }
            lvgl_port_unlock();
        }
        ESP_LOGI(TAG, "触摸关闭闹钟");
        return;
    }

    /* ── 功能层空闲计时器统一刷新（2026-08-04 修复）────────────────────────────
     * 【修复的问题】原先只有功能盘的左/右耳短按两处调了 menu_kick_idle_timer()，
     *   而头部短按确认、功能页内的所有操作（倒计时加减、头部切分/秒、闹钟编辑等）
     *   全都不刷新计时器。结果是：用户明明一直在操作，30s 一到照样被踢回主界面
     *   （日志「功能层空闲超时，返回主界面」）。
     * 【改法】在事件分发入口统一刷新一次，语义变为「功能层内任意触摸都算活动」，
     *   与用户直觉一致，也免去在十几个 case 里逐个补 kick 的遗漏风险。
     * 【为何放在这里】必须在待机唤醒 return（上面）之后——那一路的触摸只用于唤醒、
     *   不应视为功能层活动；也必须在 switch 之前，才能覆盖所有分支。
     * 【视图范围】UI_VIEW_MAIN 不需要（主界面本就没有这个计时器）。
     *   UI_VIEW_ALARM_EDIT 纳入：闹钟编辑与功能盘同样 30s 无触摸自动退出（2026-08-04 需求）。
     *   UI_VIEW_GAME 必须排除：进游戏时已显式 menu_cancel_idle_timer() 停用计时器
     *   （见 enter_game_common，理由是游戏中长时间无翻页式输入属正常），
     *   此处若 kick 会把它重新建起来，导致游戏中途被踢回主界面。 */
    if (s_view == UI_VIEW_HOME || s_view == UI_VIEW_FUNCTION_MENU || s_view == UI_VIEW_ALARM_EDIT)
    {
        menu_kick_idle_timer();
    }

    /* ── 对话进行中：主界面「整块」屏蔽触摸（2026-08-18 新增）──────────────
     * 【修的问题】原先只有 play_random_emotion() 里挡了对话态（见 :7820），
     *   而「短按/长按耳、长按头 → ui_home_enter() 进功能盘」这条路径不经过它，
     *   于是对话中一摸耳朵就切进功能盘，把正在播的对话状态 GIF（监听/说话）顶掉。
     * 【改法】在 switch 之前统一拦截：只要会话不是 SESSION_IDLE（即 LISTENING /
     *   PLAYING 全程），且当前停在主界面，就直接丢弃本次触摸——既不播情绪，
     *   也不进功能盘，保证对话全程屏幕只有状态 GIF。
     * ★★【2026-08-25 修改：由「仅主界面」扩大为「全部视图」】★★
     * 【原来为何限定 UI_VIEW_MAIN】当时的顾虑是：若用户在触摸前已身处功能盘/
     *   游戏/闹钟编辑，那些页面的翻页与退出必须继续可用，否则对话期间会被困在
     *   页面里出不来。这个顾虑在当时【成立】，因为那时功能层里可以照常喊唤醒词
     *   开对话（session 侧完全不看 UI 视图）。
     * 【现在为何可以去掉】本次同步在 session_on_wake_word() 里加了一条视图守卫：
     *   只要不在主界面就 return WAKE_IGNORED，功能层里【根本开不起对话】。
     *   于是「对话中」这个状态只可能发生在主界面，被困在功能页里的场景不复存在，
     *   限定条件失去意义，反而会漏掉「对话中恰好被强制切页到功能层」的窗口。
     * 【⚠️ 两条改动必须成对存在】只去掉本处限定而不加 session 侧守卫，会让用户
     *   在功能层被唤醒后彻底失去所有触摸、退不出去；只加守卫不去限定，则挡不住
     *   「对话中被到期事件强切到功能层后触摸乱入」。改动其一时务必同时复核另一处。
     * 【为何放这里】必须在待机唤醒 return 与闹钟 dismiss 之后——那两条优先级更高，
     *   对话中也应允许触摸唤醒/关闹钟；也必须在 switch 之前才能覆盖全部分支。 */
    if (session_get_state() != SESSION_IDLE)
    {
        ESP_LOGI(TAG, "对话进行中，屏蔽全部触摸（事件 %d，视图 %d，保持状态 GIF）",
                 (int)event, (int)s_view);
        return;
    }

    switch (s_view)
    {
    /* ─── 主界面：身体触摸触发情绪，短按耳进功能盘 ─── */
    case UI_VIEW_MAIN:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_HEAD: /* 头部：6 情绪随机 */
            play_random_emotion(emo_group_head, EMO_GROUP_LEN(emo_group_head));
            break;
        case TOUCH_EVENT_SHORT_ABDOMEN: /* 腹部：6 情绪随机 */
            play_random_emotion(emo_group_abdomen, EMO_GROUP_LEN(emo_group_abdomen));
            break;
        case TOUCH_EVENT_SHORT_BACK: /* 背部：6 情绪随机 */
            play_random_emotion(emo_group_back, EMO_GROUP_LEN(emo_group_back));
            break;
        case TOUCH_EVENT_COMBO_HEAD_ABDOMEN: /* 头+腹：6 情绪随机 */
            play_random_emotion(emo_group_head_abdomen, EMO_GROUP_LEN(emo_group_head_abdomen));
            break;
        case TOUCH_EVENT_COMBO_HEAD_BACK: /* 头+背：6 情绪随机 */
            play_random_emotion(emo_group_head_back, EMO_GROUP_LEN(emo_group_head_back));
            break;
        case TOUCH_EVENT_COMBO_ABDOMEN_BACK: /* 腹+背：6 情绪随机 */
            play_random_emotion(emo_group_abdomen_back, EMO_GROUP_LEN(emo_group_abdomen_back));
            break;
        /* 短按左/右耳 → 进功能盘（原为长按，改短按更灵敏）。
         * 长按耳保留兼容（仍进功能盘），避免误判长按时无响应。
         * 2026-08-07 新增：头部长按也进功能盘，三个入口并存，短按头部仍是情绪。 */
        case TOUCH_EVENT_SHORT_PREV_PAGE:
        case TOUCH_EVENT_SHORT_NEXT_PAGE:
        case TOUCH_EVENT_LONG_PREV_PAGE:
        case TOUCH_EVENT_LONG_NEXT_PAGE:
        case TOUCH_EVENT_LONG_HEAD:
            ui_home_enter(); /* 短按/长按耳、长按头 → 进功能盘 */
            break;
        default:
            break;
        }
        break;

    /* ─── 功能盘（单层）：左右耳果冻弹动横向切换，头部短按确认进入，长按头部退出 ─── */
    case UI_VIEW_HOME:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_PREV_PAGE: /* 左耳 → 上一个 */
            if (lvgl_port_lock(100))      /* 渐暗渐亮期间由 home_jelly 内部拒绝（动画没结束不能切） */
            {
                home_jelly(-1);
                menu_kick_idle_timer();
                lvgl_port_unlock();
                lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL); /* 同头部分支：不唤醒则切图动画要等 taskLVGL 睡醒 */
            }
            break;
        case TOUCH_EVENT_SHORT_NEXT_PAGE: /* 右耳 → 下一个 */
            if (lvgl_port_lock(100))      /* 渐暗渐亮期间由 home_jelly 内部拒绝（动画没结束不能切） */
            {
                home_jelly(+1);
                menu_kick_idle_timer();
                lvgl_port_unlock();
                lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL); /* 同头部分支：不唤醒则切图动画要等 taskLVGL 睡醒 */
            }
            break;
        case TOUCH_EVENT_SHORT_HEAD: /* 头部短按确认：震动 + 图标弹动 + 进入对应功能 */
            ESP_LOGW(TAG, "[T] 头部分支入口 t=%lld", esp_timer_get_time() / 1000);
            /* ★【2026-08-31】震动改为「确认真正进入才震」，不再由触摸层按下即震。
             * 原因同左右耳（见 home_jelly 内的说明）：本分支可能因 s_home_animating
             * 或拿不到锁而被拒绝，此时界面没有任何变化，却已经震过了 → 「震了却没进」。
             * 故触摸层已把功能盘的头部排除（bsp_touch.c 的 head_vib_view），
             * 改在下面【确认进入生效】的分支里回调请求震动。 */

            if (!s_home_animating && lvgl_port_lock(100))
            {
                /* 走到这里 = 动画未进行中且已拿到锁，本次进入必定生效 → 请求震动。
                 * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞，此处正持 LVGL 锁，
                 * 直接震会阻塞 LVGL 线程 30ms（同 BUG-038）。真正打点在触摸任务
                 * 解锁后的循环末尾。 */
                bsp_touch_request_vibrate();
                ESP_LOGW(TAG, "[T] 拿到锁 t=%lld", esp_timer_get_time() / 1000);
#if UI_FLUSH_TRACE
                ui_ftrace_begin(); /* 追踪压扁动画的脏区 */
#endif
                home_press_jelly(); /* 弹动结束的回调里才真正 on_enter */
                menu_kick_idle_timer();
                lvgl_port_unlock();
                /* 【响应延迟修复 2026-08-17】必须显式唤醒 taskLVGL，否则动画要等它睡醒。
                 * 本分支跑在【触摸任务】里：lv_anim_start 只是把动画挂进 LVGL 定时器链，
                 * 真正推进它的 taskLVGL 此刻正睡在 xEventGroupWaitBits 上，睡的时长是
                 * lv_timer_handler() 返回的「距下个定时器还有 N ms」——本工程恒有 1000ms
                 * 周期定时器（:7081 / :7095），故 N 最大约 1000ms，且不受 task_max_sleep_ms
                 * 钳制。lvgl_port_unlock() 只 give 互斥量、不置事件位，叫不醒它。
                 * 实测「拿到锁→首帧」延迟 70~970ms，即此坑。
                 * lvgl_port_task_wake 置事件位立刻打断等待，动画瞬时起跑。 */
                lvgl_port_task_wake(LVGL_PORT_EVENT_USER, NULL);
            }
            break;
        case TOUCH_EVENT_LONG_HEAD: /* 头部长按：退出回主界面 */
        case TOUCH_EVENT_LONG_PREV_PAGE:
        case TOUCH_EVENT_LONG_NEXT_PAGE:
            ui_func_layer_exit_to_main();
            break;
        default:
            break;
        }
        break;

    /* ─── 游戏运行：长按头部退出回功能盘，其余转发给游戏 ─── */
    case UI_VIEW_GAME:
        switch (event)
        {
        case TOUCH_EVENT_LONG_HEAD: /* 头部长按：退出游戏，回功能盘（渐变遮挡刷屏） */
            game_exit_fade();
            break;
        /* 注意：游戏视图下触摸层已不再产生 LONG_PREV/NEXT_PAGE（改作蓄力发短按），
         * 此处保留 case 仅为兜底，绝不回主界面，以免打断跳一跳长按蓄力。 */
        case TOUCH_EVENT_LONG_PREV_PAGE:
        case TOUCH_EVENT_LONG_NEXT_PAGE:
        default: /* 其余（左右耳/头部短按）转发给当前游戏处理 */
            games_handle_touch(event);
            break;
        }
        break;

    /* ─── 功能菜单：纯导航 + 页面专属操作 ─── */
    case UI_VIEW_FUNCTION_MENU:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_PREV_PAGE:
            /* 倒计时页：左耳短按 = 当前字段【十位】+1（2026-08-04 需求改动）。
             * 原为「减当前字段」，现改为左十右个的分位调节，调大数值更快。
             * 当前字段（分/秒）仍由头部短按切换。
             * 注：已去除功能页之间的左右耳翻页（日历/闹钟/定时器/天气切换），
             * 应用选择统一在"应用列表"层完成，进入应用后锁定当前页；
             * 想换应用请摸腹/背返回应用列表重新选。其它页此处无作用。 */
            /* 2026-08-10：仅在选中底部「自定义」档时才可调值；选中预设色块时左耳无作用 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET &&
                s_cd.sel == CD_SEL_CUSTOM)
            {
                countdown_bump_digit(/*is_tens=*/true);
                /* ★【2026-08-31】震动跟随「数字真的变了」而非「按下」：触摸层已把
                 * 功能页的左右耳排除（bsp_touch.c 的 page_vib_view），改由此处发起。
                 * 走到这里才表示三个条件（在番茄钟页 + 设置态 + 自定义档）全部成立、
                 * 值已被 countdown_bump_digit 改写；落在 if 外被忽略的那次不会震。
                 * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞（BUG-038）。 */
                bsp_touch_request_vibrate();
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            break;

        case TOUCH_EVENT_SHORT_NEXT_PAGE:
            /* 倒计时页：右耳短按 = 当前字段【个位】+1（2026-08-04 需求改动）。其它页无作用。 */
            /* 2026-08-10：同左耳，仅自定义档生效 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET &&
                s_cd.sel == CD_SEL_CUSTOM)
            {
                countdown_bump_digit(/*is_tens=*/false);
                bsp_touch_request_vibrate(); /* 同左耳：数字变完才震，见上方说明 */
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            break;

        case TOUCH_EVENT_LONG_NEXT_PAGE:
            /* ★【2026-08-31】功能页左右耳已从触摸层 page_vib_view 排除，
             * 两条真正生效的分支在此补震；else 分支自带 request，不重复。 */
            if (s_fn_page == FN_PAGE_ALARM)
            {
                bsp_touch_request_vibrate();
                alarm_edit_enter(); /* 闹钟页：进入编辑 */
            }
            else if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                bsp_touch_request_vibrate();
                countdown_start(); /* 倒计时页：启动 */
            }
            else
                function_menu_exit_fade(); /* 其他页：退出菜单（渐变遮挡刷屏） */
            break;

        case TOUCH_EVENT_LONG_PREV_PAGE:
            if (s_fn_page == FN_PAGE_COUNTDOWN)
            {
                /* ★【2026-08-31】功能页左右耳已从触摸层 page_vib_view 排除，
                 * 取消/复位这两条真正生效的分支在此补上震动；第三条分支
                 * （function_menu_exit_fade）内部自己会 request，不在此重复。 */
                if (s_cd.state == CD_STATE_RUNNING)
                {
                    bsp_touch_request_vibrate();
                    countdown_cancel(); /* 运行中：取消 */
                }
                else if (s_cd.state == CD_STATE_EXPIRED)
                {
                    bsp_touch_request_vibrate();
                    /* 已到期：手动提前重置（不等 CD_EXPIRE_HOLD_MS 自动复位） */
                    s_cd.state = CD_STATE_SET;
                    s_cd.timer_id = -1;
                    if (lvgl_port_lock(100))
                    {
                        countdown_expire_reset_cancel(); /* 必须先取消，否则自动复位定时器还会再触发一次 */
                        countdown_page_render();
                        lvgl_port_unlock();
                    }
                }
                else
                    function_menu_exit_fade(); /* 设置中：退出菜单（渐变遮挡刷屏） */
            }
            else
                function_menu_exit_fade(); /* 非倒计时页：退出菜单（渐变遮挡刷屏） */
            break;

        case TOUCH_EVENT_SHORT_HEAD:
            /* 番茄时钟设置页：头部短按 = 切到下一个时长选项（6 预设 → 自定义 → 循环）。
             * 2026-08-10 由原「分 ↔ 秒切换」改为切档，分/秒切换并入 countdown_sel_advance
             * 的自定义档内两级语义（详见该函数注释）。其它页无作用。 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                countdown_sel_advance();
                /* ★【2026-08-31】切档成功（确实在番茄钟页且处于设置态）才震，
                 * 与左右耳调值同一规则：先改值、后震动。 */
                bsp_touch_request_vibrate();
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            break;

        case TOUCH_EVENT_LONG_HEAD:
            /* 2026-08-06 需求：倒计时页的「开始」收敛到头部长按，
             * 且开始后直接退出（开始+退出+1s刷新+震动，详见 countdown_start_and_exit）。
             * 其它功能页维持原行为：头部长按 = 退出功能页回功能盘。 */
            if (s_fn_page == FN_PAGE_COUNTDOWN)
            {
                /* ★【2026-08-25 新增：到期态下长按头部＝关闭提示，不再起新倒计时】★
                 * 【修的问题】到期画面会停留 CD_EXPIRE_HOLD_MS(5s)，之后自动回设定页。
                 *   用户此刻想「关掉它」，最顺手的动作就是长按头部——而这个键在本页的
                 *   原语义是【开始倒计时】，于是一按就又起一个同样时长的倒计时，
                 *   用户完全无从察觉，表现为「番茄钟怎么又响了」。
                 * 【改法】到期态单独分流：清到期态回设定页，并取消自动复位定时器
                 *   （否则它还会到点再触发一次 render）。与左耳长按的到期分支同一套手法。
                 * 【只拦 EXPIRED】设置态/运行态的长按语义完全不变，不影响正常开始流程。 */
                if (s_cd.state == CD_STATE_EXPIRED)
                {
                    s_cd.state = CD_STATE_SET;
                    s_cd.timer_id = -1;
                    if (lvgl_port_lock(100))
                    {
                        countdown_expire_reset_cancel(); /* 必须先取消，否则自动复位还会再触发一次 */
                        countdown_page_render();
                        lvgl_port_unlock();
                    }
                    ESP_LOGI(TAG, "到期态长按头部：关闭到期提示，回到选择界面（不重启倒计时）");
                }
                else
                {
                    countdown_start_and_exit();
                }
            }
            else
                function_menu_exit_fade(); /* 其他页：退出菜单（渐变遮挡刷屏） */
            break;
        default:
            break;
        }
        break;

    /* ─── 闹钟编辑模式（2026-08-04 起仅「时 → 分」两步）───
     * 编辑「时」：左耳短按【十位】+1，右耳短按【个位】+1（0~23，非法值自动跳过）
     * 编辑「分」：左耳短按【十位】+1，右耳短按【个位】+1（本位循环，同倒计时页手感）
     * 头部短按：切换字段（时 ↔ 分）
     * 头部长按：✅ 保存退出 + 整体时间闪 3 次 + 震动 1 次（2026-08-06 主入口）
     * 右耳长按：✅ 确定（保存退出，无动效）   左耳长按：❌ 取消（放弃退出）
     *
     * 【2026-08-04 对齐倒计时】改动前是「右耳长按=放弃、左耳长按=切字段」，
     *   与倒计时页的「右耳长按=启动、左耳长按=取消」正好相反，两个页面操作手感打架。
     *   现统一为右确定 / 左取消。切字段仅保留头部短按一个入口（两项循环，够用）。
     * 注：重复模式固定「只响一次」、开关固定「保存即开启」，均不再由用户编辑。 */
    case UI_VIEW_ALARM_EDIT:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_NEXT_PAGE: /* 右耳短按：时 +1 / 分【个位】+1 */
            alarm_edit_value_next();
            break;
        case TOUCH_EVENT_SHORT_PREV_PAGE: /* 左耳短按：时 -1 / 分【十位】+1 */
            alarm_edit_value_prev();
            break;
        case TOUCH_EVENT_SHORT_HEAD: /* 头部短按：切换字段（时 ↔ 分） */
            alarm_edit_advance();
            break;
        /* 2026-08-06 需求：保存动作统一收敛到「头部长按」，并附带确认动效。
         * 头部长按 = 保存 + 退出 + 整体时间闪 3 次 + 震动 1 次。
         * 右耳长按保留为等价的"确定"，但不带动效（避免老用户习惯断档）。 */
        case TOUCH_EVENT_LONG_HEAD:
            alarm_edit_confirm_with_feedback();
            break;
        /* ★【2026-08-31】左右耳长按的震动补在此处：触摸层已把闹钟编辑页从
         * page_vib_view 排除（原本是"按下即震"，但那是【短按】的按下沿，长按到
         * 800ms 才真正确定/取消，震动与动作严重脱节）。改为真正执行退出时才震。 */
        case TOUCH_EVENT_LONG_NEXT_PAGE: /* 右耳长按：确定 → 保存退出 */
            bsp_touch_request_vibrate();
            alarm_edit_exit(true);
            break;
        case TOUCH_EVENT_LONG_PREV_PAGE: /* 左耳长按：取消 → 放弃退出（同倒计时"取消"） */
            bsp_touch_request_vibrate();
            alarm_edit_exit(false);
            break;
        default:
            break;
        }
        break;
    }
}

/**
 * @brief 外部模块手动推送 WiFi RSSI 到状态栏（带 LVGL 锁）
 *
 * 通常无需调用——状态栏定时器（battery_tick_cb）每 5s 自动拉取一次 RSSI。
 * 该接口保留给外部模块（如 MQTT 心跳）按需主动刷新。
 *
 * @param rssi WiFi 信号强度 dBm（负数）；传 0 表示未连接
 */
void ui_update_wifi(int rssi)
{
    if (s_status_wifi_lbl == NULL)
        return;
    if (lvgl_port_lock(100))
    {
        status_wifi_refresh(rssi);
        lvgl_port_unlock();
    }
}
/* ═══════════════════════════════════════════════════════════════
 * 全局浮动电量显示（挂在 LVGL top-layer，跟随所有页面常驻）
 * ═══════════════════════════════════════════════════════════════ */

/**
 * @brief 创建右上角电量标签并挂到 top-layer（顶层 layer 不受 screen 切换影响）
 *
 * 设计说明：
 *   - lv_layer_top() 是 LVGL 内置的全局浮动层，永远在所有 screen 之上
 *   - 即使后续切换功能菜单、闹钟编辑等界面，本标签也常驻可见
 *   - 当前使用纯文字（如 "85%"），后续要换图标只需修改 ui_update_battery() 文字格式
 *
 * @note 仅在首次调用时创建，重复调用安全（幂等）
 */
static void battery_label_create_top(void)
{
    if (s_battery_lbl != NULL)
        return; // 幂等

    lv_obj_t *top = lv_layer_top();

// ── 公共样式 helper 宏：所有状态栏标签视觉一致 ──
#define STATUS_LBL_STYLE(lbl)                                    \
    do                                                           \
    {                                                            \
        lv_obj_set_style_text_font((lbl), &font_cn_16, 0);       \
        lv_obj_set_style_text_color((lbl), lv_color_white(), 0); \
        lv_obj_set_style_bg_color((lbl), lv_color_black(), 0);   \
        lv_obj_set_style_bg_opa((lbl), LV_OPA_40, 0);            \
        lv_obj_set_style_pad_hor((lbl), 4, 0);                   \
        lv_obj_set_style_pad_ver((lbl), 1, 0);                   \
        lv_obj_set_style_radius((lbl), 3, 0);                    \
    } while (0)

    // 1. 右上角：电量标签（与时间对角分布）
    s_battery_lbl = lv_label_create(top);
    STATUS_LBL_STYLE(s_battery_lbl);
    lv_obj_align(s_battery_lbl, LV_ALIGN_TOP_RIGHT, -4, 4);
    lv_label_set_text(s_battery_lbl, "--%");

    // 2. 电量左侧：WiFi 信号图标（使用 Montserrat 14 自带的 LV_SYMBOL_WIFI 单字符图标）
    //    font_cn_16 是自定义中文字体不含 FontAwesome glyph，所以这里单独指定 Montserrat
    s_status_wifi_lbl = lv_label_create(top);
    STATUS_LBL_STYLE(s_status_wifi_lbl);
    lv_obj_set_style_text_font(s_status_wifi_lbl, &lv_font_montserrat_14, 0); // 覆盖默认 cn 字体
    // 右上角偏左：图标约 16px + 与电量 4px 间距 + 电量约 40px → 偏移 -52px 起步
    lv_obj_align(s_status_wifi_lbl, LV_ALIGN_TOP_RIGHT, -52, 4);
    lv_label_set_text(s_status_wifi_lbl, LV_SYMBOL_WIFI);

    // 3. 左上角：时间标签（与电量对角）
    s_status_time_lbl = lv_label_create(top);
    STATUS_LBL_STYLE(s_status_time_lbl);
    lv_obj_align(s_status_time_lbl, LV_ALIGN_TOP_LEFT, 4, 4);
    lv_label_set_text(s_status_time_lbl, "--:--");

#undef STATUS_LBL_STYLE
}

/**
 * @brief 更新电量显示
 * @param soc 电量百分比 0~100，传 -1 表示未知（显示 "--%"）
 *
 * 修改显示形式：后续如要加图标，仅需调整 snprintf 格式串，
 * 例如改为 "\xEF\x89\x83 %d%%"（FontAwesome 电池满图标）即可，
 * 函数其余部分无需改动。
 */
void ui_update_battery(int soc)
{
    if (s_battery_lbl == NULL)
        return; // 尚未创建，忽略

    char buf[16];
    if (soc < 0)
    {
        snprintf(buf, sizeof(buf), "--%%");
    }
    else
    {
        if (soc > 100)
            soc = 100;
        snprintf(buf, sizeof(buf), "%d%%", soc);
    }

    if (lvgl_port_lock(100))
    {
        lv_label_set_text(s_battery_lbl, buf);
        // 低电变色：<20% 黄，<10% 红，否则白
        lv_color_t c = lv_color_white();
        if (soc >= 0 && soc < 10)
            c = lv_color_hex(0xFF3B30);
        else if (soc >= 0 && soc < 20)
            c = lv_color_hex(0xFF9500);
        lv_obj_set_style_text_color(s_battery_lbl, c, 0);
        lvgl_port_unlock();
    }
}

/**
 * @brief 电量刷新定时器回调（LVGL 上下文，已持有 LVGL 锁）
 *
 * 调用 bsp_battery_get_percent() 拿到当前电量，
 * 因为本回调本身在 LVGL 线程内，所以直接 lv_label_set_text 即可（不再上锁）。
 */
static void battery_tick_cb(lv_timer_t *t)
{
    (void)t;
    uint8_t pct = bsp_battery_get_percent();
    uint32_t mv = bsp_battery_get_voltage_mv();

    char buf[16];
    if (mv == 0)
    {
        snprintf(buf, sizeof(buf), "--%%");
    }
    else
    {
        snprintf(buf, sizeof(buf), "%u%%", (unsigned)pct);
    }
    if (s_battery_lbl)
    {
        lv_label_set_text(s_battery_lbl, buf);
        lv_color_t c = lv_color_white();
        if (mv > 0 && pct < 10)
            c = lv_color_hex(0xFF3B30);
        else if (mv > 0 && pct < 20)
            c = lv_color_hex(0xFF9500);
        lv_obj_set_style_text_color(s_battery_lbl, c, 0);
    }

    // 顺带刷新 WiFi 信号（同一 LVGL 上下文，无需再加锁）
    status_wifi_refresh(bsp_wifi_get_rssi());
}

/**
 * @brief 根据 RSSI 刷新 WiFi 状态栏图标（已在 LVGL 上下文调用）
 *
 * 仅显示单个 LV_SYMBOL_WIFI 图标，强弱通过颜色区分：
 *   rssi == 0    灰色 — 未连接
 *   rssi >= -55  绿色 — 满格（信号强）
 *   rssi >= -70  白色 — 中等
 *   rssi >= -85  黄色 — 弱
 *   rssi <  -85  红色 — 极弱
 *
 * 不显示 dBm 数字，纯图标 + 颜色，符合手机/平板风格。
 */
static void status_wifi_refresh(int rssi)
{
    if (s_status_wifi_lbl == NULL)
        return;

    lv_color_t color;
    if (rssi == 0)
        color = lv_color_hex(0x808080); // 灰：未连接
    else if (rssi >= -55)
        color = lv_color_hex(0x34C759); // 绿：满格
    else if (rssi >= -70)
        color = lv_color_white(); // 白：中等
    else if (rssi >= -85)
        color = lv_color_hex(0xFF9500); // 黄：弱
    else
        color = lv_color_hex(0xFF3B30); // 红：极弱

    // 文本固定为 LV_SYMBOL_WIFI 单字符，宽度恒定，无需重新对齐
    lv_label_set_text(s_status_wifi_lbl, LV_SYMBOL_WIFI);
    lv_obj_set_style_text_color(s_status_wifi_lbl, color, 0);
}

/**
 * @brief 状态栏时间刷新定时器回调（1 秒一次，LVGL 上下文）
 *
 * 只显示 HH:MM；时间未同步前显示 "--:--"。
 * 注意：与功能菜单的 FN_PAGE_TIME 页面 body 中的全量时间信息（HH:MM:SS + 日期 + 日程）
 * 并不冲突，状态栏是跨页面常驻 glance，body 是详情页。
 */
static void status_time_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_status_time_lbl == NULL)
        return;

    char buf[8];
    if (!reminder_is_time_synced())
    {
        snprintf(buf, sizeof(buf), "--:--");
    }
    else
    {
        time_t now = time(NULL);
        struct tm tm_now;
        localtime_r(&now, &tm_now);
        snprintf(buf, sizeof(buf), "%02d:%02d", tm_now.tm_hour, tm_now.tm_min);
    }
    lv_label_set_text(s_status_time_lbl, buf);
    // 文本宽度可能变化，重新对齐到左上角
    lv_obj_align(s_status_time_lbl, LV_ALIGN_TOP_LEFT, 4, 4);
}
