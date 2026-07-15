/**
 * @file ui_port.c
 * @brief UI界面端口实现文件（极简版：单闹钟 + 单倒计时）
 *
 * 核心功能：基于LVGL实现智能玩偶的全量UI界面
 * 架构特点：
 *   1. 视图状态机：主界面 → 功能菜单(4页) → 闹钟编辑(4步)
 *   2. 异步渲染：所有LVGL操作均加锁保护
 *   3. 低耦合：与底层触摸、提醒系统通过接口解耦
 *
 * 本次改动：
 *   - 闹钟：列表选择 → 单闹钟直接展示 + 4步编辑(时/分/重复/开关)
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
#include "ui/interaction.h"  /* interaction_is_playing()：情绪播放中暂停自动循环 */
#include "session/session.h" /* session_get_state()：对话中屏蔽情绪触摸 */
#include "object.h"
#include <string.h>
#include <limits.h>
#include <stdio.h>
#include "driver/adc.h"

/* ── 外部自定义中文字体声明 ── */
LV_FONT_DECLARE(font_cn_16);

/* ── 功能页面顺序 ── */
typedef enum
{
    FN_PAGE_TIME = 0,  // 时钟+日历
    FN_PAGE_ALARM,     // 闹钟
    FN_PAGE_WEATHER,   // 天气
    FN_PAGE_COUNTDOWN, // 倒计时
    FN_PAGE_COUNT
} fn_page_t;

static const char *const s_fn_page_titles[FN_PAGE_COUNT] = {
    [FN_PAGE_TIME] = "时间",
    [FN_PAGE_ALARM] = "闹钟",
    [FN_PAGE_WEATHER] = "天气",
    [FN_PAGE_COUNTDOWN] = "倒计时",
};

static const char *TAG = "UI_PORT";

/* ── 函数前向声明 ── */
static void main_clock_refresh(void);
static void main_clock_tick_cb(lv_timer_t *t);
static void gif_auto_hide_cb(lv_timer_t *t);
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
static const char *s_alarm_repeat_cn(alarm_repeat_t r);
static void countdown_tick_cb(lv_timer_t *t);
static void alarm_edit_render(void);
static void countdown_page_render(void);
static void render_fn_page(fn_page_t page);
static void time_page_render_text(void);
static void alarm_page_create(void);
static void alarm_page_rebuild(void);
static void alarm_page_show(void);
static void alarm_page_hide(void);
static void alarm_edit_create(void);
static void alarm_edit_enter(void);
static void alarm_edit_exit(bool save);
static void alarm_edit_value_next(void);
static void alarm_edit_value_prev(void);
static void alarm_edit_advance(void);
static void alarm_edit_back(void);

/* ── 单层横向功能盘：渲染与导航 前向声明 ── */
static void menu_enter_fn_page(fn_page_t page); // 从功能盘进入指定功能页（复用功能菜单）
static void app_enter_calendar(void);           // 功能盘项：日历 → FN_PAGE_TIME
static void app_enter_alarm(void);              // 功能盘项：闹钟 → 直接进闹钟编辑
static void app_enter_countdown(void);          // 功能盘项：定时器 → FN_PAGE_COUNTDOWN
static void app_enter_weather(void);            // 功能盘项：天气 → FN_PAGE_WEATHER
static void enter_whack(void);                  // 功能盘项：打地鼠 → UI_VIEW_GAME + games_start
static void enter_race(void);                   // 功能盘项：赛车 → UI_VIEW_GAME + games_start
static void enter_jump(void);                   // 功能盘项：跳一跳 → UI_VIEW_GAME + games_start
static void home_render(void);                  // 渲染功能盘（居中大图标 + 名称）
static void home_jelly(int dir);                // 果冻弹动切换（dir=+1 下一个 / -1 上一个）
static void back_to_home(void);                 // 功能页/游戏 摸腹背 → 返回功能盘
static void menu_clear_func_pages(void);        // 隐藏所有功能页专属对象，仅留 title+body

/* ── 配置宏 ── */
#define UI_MAIN_GIF_RANDOM 1 // 主界面 GIF 是否随机（1=随机，0=顺序）。空闲 GIF 随机切换。
/* 开机 Logo GIF：开机先固定播这一张（独立文件，不进 s_main_gif_table），播完一轮后
 * 自动切入正常主界面轮播。因不在表内，随机/顺序逻辑永远不会再抽到它。
 * 后续把真实 logo GIF 放进 SPIFFS 的 gif/ 目录，文件名对应 UI_BOOT_LOGO_PATH 即可。 */
#define UI_BOOT_LOGO_GIF 0                  // 1=开机先播 logo，0=直接进正常轮播
#define UI_BOOT_LOGO_PATH "S:/gif/logo.gif" // 开机 logo GIF 路径（独立文件，待放入）
#define UI_MENU_IDLE_TIMEOUT_MS 30000       /* 主屏幕顶部状态栏（时间 / WiFi / 电量）总开关：1=显示，0=关闭。 \
                                             * 关闭后既不创建标签也不启动刷新定时器，相关回调内均有 NULL 早退保护，安全。 */
#define UI_SHOW_STATUS_BAR 0                /* 关闭顶部状态栏（时间/WiFi/电量全部不显示）*/

/* ── 触摸调节步进 & 上限宏 ──
 * 修改此处统一控制所有触摸步进和上限值 */
#define ALARM_MIN_STEP 2 ///< 闹钟分钟步进（+/-2，范围 0~ALARM_MIN_MAX，超出回0）
#define ALARM_MIN_MAX 58 ///< 闹钟分钟上限（0/2/4.../58，对应0~58分，60分=非法，上限取58）
#define CD_MIN_STEP 2    ///< 倒计时分钟步进（+/-2，范围 0~CD_MIN_MAX）
#define CD_MIN_MAX 60    ///< 倒计时分钟上限（0/2/4.../60，60分=合法，超出回0）
#define CD_SEC_STEP 2    ///< 倒计时秒钟步进（+/-2，范围 0~CD_SEC_MAX，超出回0）
#define CD_SEC_MAX 58    ///< 倒计时秒钟上限（0/2/4.../58，60秒=进位非法，上限取58）

/* ═══════════════════════════════════════════════════════════════
 * 闹钟编辑状态机（4步：时 → 分 → 重复 → 开关）
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    ALARM_EDIT_HOUR,   // 时
    ALARM_EDIT_MINUTE, // 分
    ALARM_EDIT_REPEAT, // 重复模式
    ALARM_EDIT_ENABLE, // 开关
} alarm_edit_state_t;

/* 重复模式数组（消除硬编码） */
static const alarm_repeat_t s_repeat_modes[] = {
    ALARM_REPEAT_ONCE,
    ALARM_REPEAT_DAILY,
    ALARM_REPEAT_WEEKDAY,
    ALARM_REPEAT_WEEKEND,
};
#define REPEAT_MODE_COUNT (sizeof(s_repeat_modes) / sizeof(s_repeat_modes[0]))

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
 *   s_gif_switch_tmr  : 延迟执行 lv_gif_set_src 的 one-shot 定时器
 *                       (不能在 LV_EVENT_READY 回调里直接切图,见 main_gif_ready_cb 说明)
 */
static int s_gif_cur_index = -1;
static int s_gif_pending_idx = -1;
static bool s_boot_logo_playing = false; // true=正在播开机 logo，播完一轮后由 ready_cb 切入主轮播
static lv_timer_t *s_gif_switch_tmr = NULL;
/* 情绪触发时指定要切到的 GIF 路径（非随机）。非 NULL 优先于 s_gif_pending_idx。
 * 由 ui_request_emotion_gif()（任意线程）设置，main_gif_switch_timer_cb（LVGL线程）消费。
 * 见 BUG-010：lv_gif_set_src 必须在 LVGL 线程调，故走 pending + 延迟 timer 机制。 */
static const char *volatile s_gif_pending_path = NULL;

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
static volatile bool s_gif_pending_is_state = false;

/* 主时钟 UI */
static lv_obj_t *s_clock_d[6];
static lv_obj_t *s_clock_col[2];
static lv_obj_t *s_time_tz_lbl = NULL;
static lv_obj_t *s_time_date_lbl = NULL;
static lv_timer_t *s_main_tick_tmr = NULL;
static lv_timer_t *s_gif_hide_tmr = NULL;

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
LV_IMAGE_DECLARE(p2);
LV_IMAGE_DECLARE(p3);
LV_IMAGE_DECLARE(p4);
LV_IMAGE_DECLARE(p5);
LV_IMAGE_DECLARE(p6);

/* 功能盘单个条目：图片(可空) + 占位色 + 确认后进入的回调。无底部文字。 */
typedef struct
{
    const lv_image_dsc_t *img; // 图片图标；NULL 则用 color 占位
    uint32_t color;            // 占位色块颜色（img==NULL 时生效）
    void (*on_enter)(void);    // 头部确认后进入；游戏项包一层 enter_xxx
} home_item_t;

static const home_item_t s_home_items[] = {
    {&picture1, 0x4FC3F7, app_enter_calendar}, /* 日历：浅蓝（待转图） */
    {&p2, 0xFFB300, app_enter_alarm},          /* 闹钟：橙黄（待转图） */
    {&p3, 0x81C784, app_enter_countdown},      /* 定时器：绿（待转图） */
    {&p4, 0xFFD54F, app_enter_weather},        /* 天气：黄（待转图） */
    {&dishu, 0x8D6E63, enter_whack},           /* 打地鼠：用你的地鼠图 ✅ */
    {&p5, 0xE57373, enter_race},               /* 赛车：红（待转图） */
    {&p6, 0x9575CD, enter_jump},               /* 跳一跳：紫（待转图） */
};
#define HOME_ITEM_COUNT (sizeof(s_home_items) / sizeof(s_home_items[0]))

/* 功能盘导航状态 */
static int s_home_idx = 0;                     // 当前选中项下标
static volatile bool s_home_animating = false; // 果冻动画进行中（防连点打断）
#define HOME_PUNCH_PX 28                       // 果冻弹动的横向位移幅度（像素）

/* 闹钟编辑上下文 */
static struct
{
    alarm_edit_state_t state;
    uint8_t hour;
    uint8_t minute;
    alarm_repeat_t repeat;
    bool enabled;
} s_edit;

/* 闹钟页 UI 对象（单闹钟直接展示） */
static lv_obj_t *s_alarm_time_lbl = NULL;
static lv_obj_t *s_alarm_repeat_lbl = NULL;
static lv_obj_t *s_alarm_status_lbl = NULL;
static lv_obj_t *s_alarm_hint_lbl = NULL;

/* 天气页 UI 对象（参考图上半部风格：左大号温度 + 右天气/体感/风 + 底部湿度/降水量） */
static lv_obj_t *s_wx_city_lbl = NULL;  ///< 城市名（左上）
static lv_obj_t *s_wx_temp_lbl = NULL;  ///< 大号温度数字"24"（montserrat_48）
static lv_obj_t *s_wx_deg_lbl = NULL;   ///< 小号度数符号"°"（font_cn_16，贴温度右上）
static lv_obj_t *s_wx_text_lbl = NULL;  ///< 天气现象"小雨"
static lv_obj_t *s_wx_feels_lbl = NULL; ///< "体感26°  西北风2级"
static lv_obj_t *s_wx_stats_lbl = NULL; ///< "湿度91%    降水量0.5mm"

/* 闹钟编辑 UI 对象 */
static lv_obj_t *s_edit_panel = NULL;
static lv_obj_t *s_edit_hour_lbl = NULL;
static lv_obj_t *s_edit_colon_lbl = NULL;
static lv_obj_t *s_edit_min_lbl = NULL;
static lv_obj_t *s_edit_repeat_lbl = NULL;
static lv_obj_t *s_edit_enable_lbl = NULL;
static lv_obj_t *s_edit_hint_lbl = NULL;

/* 倒计时上下文 */
static struct
{
    cd_state_t state;
    uint8_t minutes;
    uint8_t seconds;  ///< 设置阶段附加的秒数（0~CD_SEC_MAX）
    bool editing_sec; ///< 当前正在编辑秒钟（true）还是分钟（false）
    int timer_id;
    lv_timer_t *tick_tmr;
} s_cd = {.state = CD_STATE_SET, .minutes = 15, .seconds = 0, .editing_sec = false, .timer_id = -1, .tick_tmr = NULL};

/* 倒计时 UI 对象 */
static lv_obj_t *s_cd_time_lbl = NULL;  ///< 运行/到期状态整体时间标签（MM:SS）
static lv_obj_t *s_cd_min_lbl = NULL;   ///< 设置状态：分钟标签（可独立上色）
static lv_obj_t *s_cd_colon_lbl = NULL; ///< 设置状态：冒号标签
static lv_obj_t *s_cd_sec_lbl = NULL;   ///< 设置状态：秒钟标签（可独立上色）
static lv_obj_t *s_cd_hint_lbl = NULL;
static lv_obj_t *s_cd_state_lbl = NULL;

/* 情绪面板 */
static lv_obj_t *s_emo_panel = NULL;
static lv_obj_t *s_emo_name_lbl = NULL;
static lv_obj_t *s_emo_anim_lbl = NULL;
static lv_obj_t *s_emo_audio_lbl = NULL;
static lv_timer_t *s_emo_timer = NULL;

// 1 为启用月历组件，0 为回退到原本的文字显示
#define CONFIG_UI_USE_CALENDAR 1
/* 在 ui_port.c 的全局变量区域添加 */
#if CONFIG_UI_USE_CALENDAR
static lv_obj_t *s_calendar = NULL;
#endif
/**
 * @brief 实现月历网格渲染
 * 依据：使用 LVGL 内置日历组件并配合已有的 font_cn_16 字体
 */
#if CONFIG_UI_USE_CALENDAR
static lv_obj_t *s_calendar_clock = NULL; // 定义一个新的独立时钟标签

static void time_page_render_calendar(void)
{
    if (s_menu_panel == NULL)
        return;

    if (s_calendar == NULL)
    {
        // 1. 创建日历网格
        s_calendar = lv_calendar_create(s_menu_panel);
        lv_obj_set_size(s_calendar, 230, 190);
        lv_obj_align(s_calendar, LV_ALIGN_BOTTOM_MID, 0, 0);
        lv_obj_set_style_text_font(s_calendar, &font_cn_16, 0); // 设置字体为中文字体

        // 2. 创建一个“不被覆盖”的右上角时间标签
        s_calendar_clock = lv_label_create(s_menu_panel);
        lv_obj_set_style_text_font(s_calendar_clock, &font_cn_16, 0);
        lv_obj_set_style_text_color(s_calendar_clock, lv_color_white(), 0);
        // 精确对齐到右上角，这样它就永远在日历网格的上方
        lv_obj_align(s_calendar_clock, LV_ALIGN_TOP_RIGHT, -10, 8);
    }

    // 3. 同时更新日历日期和右上角的时间数字
    if (reminder_is_time_synced())
    {
        uint8_t h, m, s;
        reminder_get_current_time(&h, &m, &s);
        char time_buf[16];
        snprintf(time_buf, sizeof(time_buf), "%02d:%02d", h, m);
        lv_label_set_text(s_calendar_clock, time_buf); // 更新右上角时钟

        // 更新日历主体
        struct tm tm_now;
        time_t now = time(NULL);
        localtime_r(&now, &tm_now);
        lv_calendar_set_today_date(s_calendar, tm_now.tm_year + 1900, tm_now.tm_mon + 1, tm_now.tm_mday);
        lv_calendar_set_showed_date(s_calendar, tm_now.tm_year + 1900, tm_now.tm_mon + 1);
    }

    lv_obj_clear_flag(s_calendar, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_calendar_clock, LV_OBJ_FLAG_HIDDEN); // 确保时钟可见
}
#endif

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
    const char *gif_path;     // SPIFFS 路径,如 "S:/gif/one.gif"
    gif_servo_action_t head;  // 头部舵机动作
    gif_servo_action_t l_arm; // 左臂舵机动作
    gif_servo_action_t r_arm; // 右臂舵机动作
} main_gif_entry_t;

/* 各 GIF 的舵机动作(参数为初版默认值,可边测边调)。
 * 约定:{幅度, 方向, 速度, 次数, 是否往返} */
static const main_gif_entry_t s_main_gif_table[] = {
    // one.gif:活泼 —— 头快速往返摇 3 次,双臂中速各摆 2 次
    {"S:/gif/one.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true},
     /*l_arm*/ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, true}},

    // two.gif:好奇 —— 头缓慢侧偏一下(单次回中),双臂不动
    {"S:/gif/two.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 2, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 2, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 2, false}},

    // three.gif:舒服 —— 头慢速轻摇 2 次,双臂慢速小幅各摆 1 次
    {"S:/gif/three.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 3, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 3, true}},

    // four.gif:犯困 —— 头极慢大幅侧偏一下,双臂不动
    {"S:/gif/four.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 4, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 4, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 4, false}},

    // five.gif:惊喜 —— 头中速左右摆 2 次,双臂中速各摆 1 次
    {"S:/gif/five.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 5, true},
     /*l_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_MID, 5, false},
     /*r_arm*/ {SERVO_AMPLITUDE_15, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 5, false}},

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
 * 占位：GIF 先复用现有 three/four.gif；素材到位后只改 .gif_path（换中性 GIF）和动作参数。
 *       切图前 main_gif_switch_timer_cb 会 lv_fs_open 校验文件存在，不存在则保持当前画面（BUG-010）。
 * 角度约定：中心 90°；head 左+右-，臂 前+后-。
 * ═══════════════════════════════════════════════════════════════ */
/* 状态动作专用震动序列（与情绪 vib_* 独立，可自由调手感） */
static const VibStep_t s_vib_listen[] = {{50, 60, 0}};              // 监听：轻震 1 次
static const VibStep_t s_vib_speak[] = {{70, 50, 50}, {70, 50, 0}}; // 说话：短促 2 次

/* 监听组（用户说话 LISTENING）：占位 GIF three.gif + 轻摇头 + 轻震 */
static const ia_custom_action_t s_listening_actions[] = {
    {
        .gif_path = "S:/gif/three.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false, // 对话动作不可被触摸打断
        .vib_seq = s_vib_listen,
        .vib_seq_len = sizeof(s_vib_listen) / sizeof(s_vib_listen[0]),
        .head = {115.0f, 65.0f, SERVO_SPEED_SLOW, 2},   // 头轻摇 2 次
        .left_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0}, // 臂不动（count=0）
        .right_arm = {90.0f, 90.0f, SERVO_SPEED_MID, 0},
    },
};

/* 说话组（大模型说话 PLAYING）：占位 GIF four.gif + 点头 + 短促震 */
static const ia_custom_action_t s_speaking_actions[] = {
    {
        .gif_path = "S:/gif/four.gif",
        .is_state_gif = true,
        .keep_screen = true,
        .is_idle = false, // 对话动作不可被触摸打断
        .vib_seq = s_vib_speak,
        .vib_seq_len = sizeof(s_vib_speak) / sizeof(s_vib_speak[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 2},    // 头快摆 2 次
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1}, // 臂小幅摆 1 次
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
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
        // 对应 one.gif：头大幅左右摆 2 次 + 双臂摆（占位用明显幅度，验证舵机能动；后续按手感调小）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {120.0f, 60.0f, SERVO_SPEED_SLOW, 2},
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_SLOW, 2},
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_SLOW, 2},
    },
    {
        // 对应 two.gif：头大幅摆 2 次
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {125.0f, 55.0f, SERVO_SPEED_MID, 2},
        .left_arm = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},
        .right_arm = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},
    },
    {
        // 对应 three.gif：头大幅摆 1 次 + 双臂，不震动（节奏变化）
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = NULL,
        .vib_seq_len = 0,
        .head = {120.0f, 60.0f, SERVO_SPEED_SLOW, 1},
        .left_arm = {120.0f, 60.0f, SERVO_SPEED_SLOW, 1},
        .right_arm = {120.0f, 60.0f, SERVO_SPEED_SLOW, 1},
    },
    {
        // 对应 four.gif：头大幅摆 2 次 + 轻震
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {130.0f, 50.0f, SERVO_SPEED_MID, 2},
        .left_arm = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},
        .right_arm = {90.0f, 90.0f, SERVO_SPEED_SLOW, 0},
    },
    {
        // 对应 five.gif：头中幅摆 2 次 + 双臂各摆 1 次 + 轻震
        .gif_path = NULL,
        .is_state_gif = false,
        .keep_screen = true,
        .is_idle = true,
        .vib_seq = s_vib_idle,
        .vib_seq_len = sizeof(s_vib_idle) / sizeof(s_vib_idle[0]),
        .head = {115.0f, 65.0f, SERVO_SPEED_MID, 2},
        .left_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
        .right_arm = {110.0f, 70.0f, SERVO_SPEED_MID, 1},
    },
};
#define IDLE_ACTION_COUNT (sizeof(s_idle_actions) / sizeof(s_idle_actions[0]))

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
 *   分配 draw_buf(ARGB8888,宽*高*4字节)。曾把 three.gif 误换成 512x512
 *   (屏幕仅 320x240),画布面积达屏幕 3.4 倍、单帧缓冲逼近 1MB,PSRAM 碎片化
 *   下分配失败——而 gif_initialize 分配失败时只置 draw_buf=NULL 并 return,
 *   并不会停 gifobj->timer,下一帧 gif_next_frame_task_cb 照常触发,解引用
 *   NULL 的 draw_buf->header.w 直接 LoadProhibited 崩溃。故切图前先用
 *   lv_gif_get_size 探测画布尺寸,超过屏幕面积 1.5 倍就拒绝,保留当前画面。
 *
 * @param gif_path 待切换的 GIF 路径
 * @return true=尺寸安全可以切图；false=尺寸异常,已打日志,调用方应保持当前画面
 */
static bool main_gif_check_size_safe(const char *gif_path)
{
    uint16_t w = 0, h = 0;
    if (!lv_gif_get_size(gif_path, &w, &h) || w == 0 || h == 0)
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

    lv_gif_set_src(gif_obj, entry->gif_path); // 切新图,内部自动重新播放
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
    // 【诊断-DBG4】GIF 每播完一轮都会进这里；重点看二级待机期间是否仍在持续触发
    // （若二级期间此日志仍频繁刷屏，说明 GIF 动画没被冻结，一直在后台播放/切换，验证OK后可删）
    ESP_LOGW("GIFDBG", "[DBG4] ready_cb 触发: s_view=%d standby_deep=%d interaction_playing=%d neutral=%d",
             (int)s_view, (int)standby_is_deep_active(), (int)interaction_is_playing(), (int)s_neutral_active);
    // 仅在主界面才循环切换 GIF + 驱动舵机;进入功能菜单/闹钟编辑后 GIF 被隐藏,
    // 此时不应继续切图或驱动舵机(否则会出现"已在功能层却仍在动"的异常)。
    if (s_view != UI_VIEW_MAIN)
        return;
    // ★开机 logo 播完一轮 → 切入正常主界面轮播的表内首张,之后 logo 永不再出现。
    //   放在最靠前(仅次于 s_view 判断)是为了保证 logo 只播这一轮就必定让位给正常轮播,
    //   不受待机/对话/动作等其它早退标志影响。
    if (s_boot_logo_playing)
    {
        s_boot_logo_playing = false;
        s_gif_pending_idx = main_gif_pick_next_index(-1); // 表内首张(随机/顺序都在表索引内取,不含 logo)
        ESP_LOGI("GIFDBG", "ready_cb: 开机 logo 播完 → 切入主轮播 idx=%d", s_gif_pending_idx);
        if (s_gif_switch_tmr != NULL)
            lv_timer_resume(s_gif_switch_tmr);
        return;
    }
    // ★（已废弃，与「低功耗随机情绪驱动 GIF+头部+音频」冲突，改由 main_gif_switch_timer_cb
    //   的待机分支接管——见该函数内 standby_is_active() 判断）：原「待机定格当前GIF、
    //   停自动循环」逻辑。若待机改回不需要随机情绪轮播，可取消注释恢复本行为。
    // if (standby_is_active())
    //     return;
    // 任何动作（情绪/对话状态/空闲）播放中：暂停自动随机循环，让位给动作自己切图。
    // 空闲动作 keep_screen=false，播完会调 ui_resume_main_gif_loop 主动触发下一轮切图，
    // 故这里挡住不会导致停摆；反而避免「动作播放中 ready_cb 又排队切图」造成的重复切图/打断。
    if (interaction_is_playing())
    {
        ESP_LOGI("GIFDBG", "ready_cb: GIF播完一轮 但动作播放中(is_playing) → 早退不切图");
        return;
    }
    // 对话中（LISTENING/PLAYING）：屏幕锁定状态中性 GIF，不自动随机循环（防待机循环抢图）。
    // 状态回 IDLE 时 ui_set_neutral_gif_state(NEUTRAL_IDLE) 会清此标志并恢复循环。
    if (s_neutral_active)
        return;
    if (s_gif_pending_idx >= 0)
        return; // 已有待切换,避免本轮重复排队
    s_gif_pending_idx = main_gif_pick_next_index(s_gif_cur_index);
    ESP_LOGI("GIFDBG", "ready_cb: GIF播完一轮 → 排队下一张 idx=%d", s_gif_pending_idx);
    if (s_gif_switch_tmr != NULL)
        lv_timer_resume(s_gif_switch_tmr); // 唤醒延迟切换 timer,下个 tick 执行
}

/**
 * @brief 延迟切换 timer 回调:在 GIF 自己的 timer 回调彻底返回后的下一个
 *        lv_timer_handler 迭代中执行,此刻 gif_obj 已稳定(pause 在最后一帧),set_src 安全
 *
 * 同样在 LVGL 线程,严禁 lvgl_port_lock。one-shot:执行一次后 pause,等下次 READY 再 resume。
 */
static void main_gif_switch_timer_cb(lv_timer_t *t)
{
    lv_timer_pause(t);

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

    // 动作播放中：不在这里自动切图（让位）。空闲动作 gif_path=NULL 不切图、keep_screen=true
    // 不 resume，故它播放期间被挡也不会停摆——下一张只由 GIF 的 READY 驱动（ready_cb），解耦。
    if (interaction_is_playing())
        return;

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

    /* ── 切图：GIF 完整播完一轮(READY) 才走到这里切下一张（READY 驱动，GIF 永不被打断）。── */
    main_gif_apply_index(idx, /*with_servo=*/false);

    /* ── 空闲（非待机）：切完图，附带投递本图对应动作（只舵机+震动）。
     * 动作 gif_path=NULL（不切图）、keep_screen=true（不 resume、不触发下一张），舵机在
     * 这一轮 GIF 期间摆完即安静，下一张由 READY 驱动。GIF 与舵机各自完整、互不打断。
     * 动作按 GIF 索引取同序号（s_idle_actions 与 s_main_gif_table 对齐）。 */
    if (idx >= 0 && (size_t)idx < IDLE_ACTION_COUNT)
    {
        ESP_LOGI("GIFDBG", "timer_cb: 切图 idx=%d + 投递舵机动作", idx);
        ui_interaction_play_custom(&s_idle_actions[idx]);
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
    if (s_view != UI_VIEW_MAIN)
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
    if (s_view != UI_VIEW_MAIN)
        return; // 非主界面（功能盘/游戏/闹钟编辑）不切，避免抢功能层画面

    /* IDLE：退出对话态，恢复待机自动随机循环（不带动作） */
    if (st == NEUTRAL_IDLE)
    {
        s_neutral_active = false;
        ui_resume_main_gif_loop(); // 内部自带 s_view==MAIN 判断与跨线程安全
        return;
    }

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
    if (s_view != UI_VIEW_MAIN)
        return; // 已不在主界面（如已进功能盘），不恢复
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
 * @brief 兼容旧接口:供情绪动画映射表(s_animation_map)调用,仅切换 GIF 图源、不触发舵机
 *
 * 情绪触发时舵机由 interaction 本身负责,这里若再入队会重复叠加,故只换图。
 */
static void gif_switch_source(void)
{
    if (gif_obj == NULL)
        return;
    main_gif_apply_index(main_gif_pick_next_index(s_gif_cur_index), /*with_servo=*/false);
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

    // 创建延迟切换 one-shot 定时器:周期取很小值(下个 tick 触发即可),先 pause
    s_gif_switch_tmr = lv_timer_create(main_gif_switch_timer_cb, 10, NULL);
    lv_timer_pause(s_gif_switch_tmr);

#if UI_BOOT_LOGO_GIF
    // 开机先播独立的 logo GIF(不在表内,故随机/顺序逻辑永不再抽到它)。
    // 播完一轮触发 LV_EVENT_READY → main_gif_ready_cb 里切入表内首张进入正常轮播。
    s_boot_logo_playing = true;
    lv_gif_set_src(gif_obj, UI_BOOT_LOGO_PATH); // 直接播 logo,不走 main_gif_apply_index(它按表索引取图)
    s_gif_cur_index = -1;                       // logo 不属于表,当前索引标记无效
    ESP_LOGI(TAG, "GIF待机动画已创建,开机 logo=%s", UI_BOOT_LOGO_PATH);
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
        //   W*H/4 = 38400 字节：分配失败（DMA SRAM 连续块不够）
        //   W*H/5 = 30720 字节：分到但 WebSocket 等模块缺内存创建失败，1kSRAM 余量太小不稳
        //   W*H/6 = 24576 字节：分到但 WebSocket 等模块缺内存创建失败，2kSRAM 余量太小不稳
        //   W*H/7 = 20480 字节：分到但 WebSocket 等模块缺内存创建失败，8kSRAM 余量太小不稳已经带有拖影了
        //   W*H/8 = 19200 字节：分到且留 ~11KB 给其他模块（稳态） ，已经带有拖影了
        //   W*H/16 = 9600 字节：余量更大但 GIF 帧率会更慢
        // PSRAM 全屏 buffer 导致 SPI DMA 无法访问 PSRAM 指针 → tx_color failed
        // 稳态：W*H/8 = 19200 字节，内部 SRAM + DMA，PARTIAL 模式分 8 次 flush
        .buffer_size = BSP_LCD_WIDTH * BSP_LCD_HEIGHT / 7, // 30720 字节：理论上可分配但实测不稳，但是已经是极限了
        .double_buffer = false,
        .hres = BSP_LCD_WIDTH,
        .vres = BSP_LCD_HEIGHT,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {.swap_xy = true, .mirror_x = false, .mirror_y = true},
        .flags = {.buff_dma = true, .swap_bytes = false, .buff_spiram = false}};

    PRINT_MEM_INFO(TAG, "lvgl_port_add_disp 前(即将申请30720B DMA-SRAM draw buffer)");
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    if (lvgl_disp == NULL)
    {
        // disp 创建失败时必须返回错误，否则 ui_init 后续会调 lv_screen_active()
        // 拿到失效对象，main_desplay_create 解引用导致 LoadProhibited 崩溃
        ESP_LOGE(TAG, "lvgl_port_add_disp 失败：DMA 内部 SRAM 不足 30720 字节连续区");
        PRINT_MEM_INFO(TAG, "LVGL flush buffer 30720B DMA-SRAM 分配失败");
        return ESP_ERR_NO_MEM;
    }
    PRINT_MEM_INFO(TAG, "LVGL flush buffer 30720B DMA-SRAM 分配后");

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

static void main_clock_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_view == UI_VIEW_FUNCTION_MENU &&
        s_fn_page == FN_PAGE_TIME &&
        s_menu_body != NULL)
    {
        if (lvgl_port_lock(10))
        {
            time_page_render_text();
            lvgl_port_unlock();
        }
    }
}

static void gif_auto_hide_cb(lv_timer_t *t)
{
    (void)t;
    if (gif_obj)
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    s_gif_hide_tmr = NULL;
}

void ui_update_time(void)
{
    if (!lvgl_port_lock(100))
        return;
    main_clock_refresh();
    lvgl_port_unlock();
}

void ui_update_emotion(const char *emotion) {}

/* ═══════════════════════════════════════════════════════════════
 * 情绪数据 & 面板
 * ═══════════════════════════════════════════════════════════════ */
typedef struct
{
    const char *name;
    const char *anim;
    const char *audio;
} emo_entry_t;

static void show_random_emotion(const emo_entry_t *group, size_t count)
{
    const emo_entry_t *e = &group[esp_random() % count];
    ui_show_emotion(e->name, e->anim, e->audio);
    ESP_LOGI("TOUCH", "[%s] 动画:%s 音效:%s", e->name, e->anim, e->audio);
}

static void hide_emotion_cb(lv_timer_t *t)
{
    (void)t;
    if (s_emo_panel)
        lv_obj_add_flag(s_emo_panel, LV_OBJ_FLAG_HIDDEN);
    s_emo_timer = NULL;
}

void ui_show_emotion(const char *name, const char *anim_desc, const char *audio_desc)
{
    if (!lvgl_port_lock(100))
        return;
    lv_obj_t *scr = lv_screen_active();
    if (scr == NULL)
    {
        ESP_LOGE(TAG, "lv_screen_active 返回 NULL，跳过样式设置");
        return;
    }

    if (s_emo_panel == NULL)
    {
        s_emo_panel = lv_obj_create(scr);
        lv_obj_set_size(s_emo_panel, 220, 100);
        lv_obj_align(s_emo_panel, LV_ALIGN_BOTTOM_MID, 0, -8);
        lv_obj_set_style_bg_color(s_emo_panel, lv_color_hex(0x111111), 0);
        lv_obj_set_style_bg_opa(s_emo_panel, LV_OPA_80, 0);
        lv_obj_set_style_border_color(s_emo_panel, lv_color_hex(0xFFFFFF), 0);
        lv_obj_set_style_border_opa(s_emo_panel, LV_OPA_30, 0);
        lv_obj_set_style_border_width(s_emo_panel, 1, 0);
        lv_obj_set_style_radius(s_emo_panel, 12, 0);
        lv_obj_set_style_pad_all(s_emo_panel, 6, 0);
        lv_obj_clear_flag(s_emo_panel, LV_OBJ_FLAG_SCROLLABLE);

        s_emo_name_lbl = lv_label_create(s_emo_panel);
        lv_obj_set_style_text_color(s_emo_name_lbl, lv_color_hex(0xFFFFFF), 0);
        lv_obj_align(s_emo_name_lbl, LV_ALIGN_TOP_MID, 0, 2);

        s_emo_anim_lbl = lv_label_create(s_emo_panel);
        lv_obj_set_style_text_color(s_emo_anim_lbl, lv_color_hex(0x88CCFF), 0);
        lv_label_set_long_mode(s_emo_anim_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(s_emo_anim_lbl, 200);
        lv_obj_align(s_emo_anim_lbl, LV_ALIGN_CENTER, 0, 6);

        s_emo_audio_lbl = lv_label_create(s_emo_panel);
        lv_obj_set_style_text_color(s_emo_audio_lbl, lv_color_hex(0xFFDD88), 0);
        lv_label_set_long_mode(s_emo_audio_lbl, LV_LABEL_LONG_WRAP);
        lv_obj_set_width(s_emo_audio_lbl, 200);
        lv_obj_align(s_emo_audio_lbl, LV_ALIGN_BOTTOM_MID, 0, -2);
    }

    lv_label_set_text(s_emo_name_lbl, name);
    lv_label_set_text(s_emo_anim_lbl, anim_desc);
    lv_label_set_text(s_emo_audio_lbl, audio_desc);
    lv_obj_clear_flag(s_emo_panel, LV_OBJ_FLAG_HIDDEN);

    if (s_emo_timer != NULL)
        lv_timer_reset(s_emo_timer);
    else
        s_emo_timer = lv_timer_create(hide_emotion_cb, 3000, NULL);
    lv_timer_set_repeat_count(s_emo_timer, 1);
    lvgl_port_unlock();
}

/* ═══════════════════════════════════════════════════════════════
 * 功能菜单框架
 * ═══════════════════════════════════════════════════════════════ */
static void menu_idle_timeout_cb(lv_timer_t *t)
{
    (void)t;
    s_menu_idle_tmr = NULL;
    /* 任意功能层（功能盘/功能页/游戏）空闲超时都回主界面 */
    if (s_view == UI_VIEW_MAIN || s_view == UI_VIEW_ALARM_EDIT)
        return;
    /* 若停在游戏视图空闲超时，先释放游戏资源（幂等） */
    if (s_view == UI_VIEW_GAME)
        games_stop();
    if (s_menu_panel)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    if (gif_obj != NULL)
        lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    s_view = UI_VIEW_MAIN;
    main_gif_kick_resume(); // 重启 GIF 循环(功能层期间已暂停)
    ESP_LOGI(TAG, "功能层空闲超时，返回主界面");
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
 * ═══════════════════════════════════════════════════════════════ */
static const char *s_alarm_repeat_cn(alarm_repeat_t r)
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
    lv_obj_set_style_text_font(s_alarm_time_lbl, &lv_font_montserrat_48, 0);
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

    if (count == 0)
    {
        /* 未设置闹钟：明确的空状态，避免误导 */
        lv_label_set_text(s_alarm_time_lbl, "--:--");
        lv_label_set_text(s_alarm_repeat_lbl, "未设置");
        lv_label_set_text(s_alarm_status_lbl, "");
        lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_hex(0x555555), 0);
        lv_obj_set_style_text_color(s_alarm_repeat_lbl, lv_color_hex(0x555555), 0);
        lv_label_set_text(s_alarm_hint_lbl, "长按后页键新建闹钟");
    }
    else
    {
        char buf[16];
        snprintf(buf, sizeof(buf), "%02d:%02d", list[0].hour, list[0].minute);
        lv_label_set_text(s_alarm_time_lbl, buf);
        lv_label_set_text(s_alarm_repeat_lbl, s_alarm_repeat_cn(list[0].repeat));

        if (list[0].enabled)
        {
            lv_label_set_text(s_alarm_status_lbl, "已开启");
            lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_white(), 0);
            lv_obj_set_style_text_color(s_alarm_repeat_lbl, lv_color_hex(0x888888), 0);
            lv_obj_set_style_text_color(s_alarm_status_lbl, lv_color_hex(0xFF9500), 0);
        }
        else
        {
            lv_label_set_text(s_alarm_status_lbl, "已关闭");
            lv_obj_set_style_text_color(s_alarm_time_lbl, lv_color_hex(0x555555), 0);
            lv_obj_set_style_text_color(s_alarm_repeat_lbl, lv_color_hex(0x555555), 0);
            lv_obj_set_style_text_color(s_alarm_status_lbl, lv_color_hex(0x555555), 0);
        }
        lv_label_set_text(s_alarm_hint_lbl, "长按后页键进入设置");
    }
}

static void alarm_page_show(void)
{
    alarm_page_create();
    alarm_page_rebuild();
    lv_obj_clear_flag(s_alarm_time_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_alarm_repeat_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_alarm_status_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_alarm_hint_lbl, LV_OBJ_FLAG_HIDDEN);
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
}

/* ═══════════════════════════════════════════════════════════════
 * 天气展示页（参考图上半部风格，纯文字，背景由外部叠加）
 *
 *   杭州                         ← 左上城市
 *    ⎡24⎤°  小雨                 ← montserrat_48 大号数字 + 小号° + 天气
 *           体感26°  西北风2级    ← font_cn_16 灰字
 *   ───────────────────────────
 *   湿度91%      降水量0.5mm      ← 底部统计行
 *
 * 数据全部来自和风 /v7/weather/now，无额外接口。
 * ═══════════════════════════════════════════════════════════════ */
static void weather_page_create(void)
{
    if (s_wx_temp_lbl)
        return;

    /* 城市名（左上角） */
    s_wx_city_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_city_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_city_lbl, lv_color_white(), 0);
    lv_obj_align(s_wx_city_lbl, LV_ALIGN_TOP_LEFT, 0, 2);

    /* 大号温度数字（montserrat_48，仅数字） */
    s_wx_temp_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_temp_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_wx_temp_lbl, lv_color_white(), 0);
    lv_obj_align(s_wx_temp_lbl, LV_ALIGN_TOP_LEFT, 4, 36);

    /* 小号度数符号（font_cn_16，贴大号数字右上） */
    s_wx_deg_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_deg_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_deg_lbl, lv_color_white(), 0);
    lv_label_set_text(s_wx_deg_lbl, "\xC2\xB0");
    /* x 偏移在 rebuild 里按数字宽度动态对齐 */

    /* 天气现象（大号数字右侧，上行） */
    s_wx_text_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_text_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_text_lbl, lv_color_white(), 0);
    lv_obj_align(s_wx_text_lbl, LV_ALIGN_TOP_LEFT, 96, 44);

    /* 体感 + 风（大号数字右侧，下行，灰字） */
    s_wx_feels_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_feels_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_feels_lbl, lv_color_hex(0xAAAAAA), 0);
    lv_obj_align(s_wx_feels_lbl, LV_ALIGN_TOP_LEFT, 96, 66);

    /* 底部统计行：湿度 + 降水量 */
    s_wx_stats_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_wx_stats_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_wx_stats_lbl, lv_color_hex(0x88CCFF), 0);
    lv_obj_align(s_wx_stats_lbl, LV_ALIGN_BOTTOM_LEFT, 4, -18);
}

static void weather_page_rebuild(void)
{
    if (!s_wx_temp_lbl)
        return;

    weather_data_t wd;
    reminder_get_weather_data(&wd);

    char buf[64];

    /* 城市 */
    lv_label_set_text(s_wx_city_lbl, wd.city_name[0] ? wd.city_name : "定位中");

    if (wd.valid)
    {
        /* 大号温度数字 + 紧贴右上的小号° */
        lv_label_set_text(s_wx_temp_lbl, wd.temp);
        lv_obj_update_layout(s_wx_temp_lbl); /* 先刷新宽度再对齐° */
        lv_coord_t tw = lv_obj_get_width(s_wx_temp_lbl);
        lv_obj_align(s_wx_deg_lbl, LV_ALIGN_TOP_LEFT, 4 + tw + 2, 36);
        lv_obj_clear_flag(s_wx_deg_lbl, LV_OBJ_FLAG_HIDDEN);

        lv_label_set_text(s_wx_text_lbl, wd.text);

        snprintf(buf, sizeof(buf), "体感%s\xC2\xB0  %s", wd.feels, wd.wind);
        lv_label_set_text(s_wx_feels_lbl, buf);

        snprintf(buf, sizeof(buf), "湿度%s%%      降水量%smm", wd.humidity, wd.precip);
        lv_label_set_text(s_wx_stats_lbl, buf);
    }
    else
    {
        lv_label_set_text(s_wx_temp_lbl, "--");
        lv_obj_add_flag(s_wx_deg_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_label_set_text(s_wx_text_lbl, "等待天气数据");
        lv_label_set_text(s_wx_feels_lbl, "");
        lv_label_set_text(s_wx_stats_lbl, "湿度--%      降水量--mm");
    }
}

static void weather_page_show(void)
{
    weather_page_create();
    /* 进入页面触发一次拉取（异步，本次先用上次缓存渲染） */
    reminder_weather_fetch_now();
    weather_page_rebuild();
    lv_obj_clear_flag(s_wx_city_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_temp_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_text_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_feels_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_wx_stats_lbl, LV_OBJ_FLAG_HIDDEN);
    /* s_wx_deg_lbl 的显隐由 rebuild 按数据有效性决定 */
}

static void weather_page_hide(void)
{
    if (s_wx_city_lbl)
        lv_obj_add_flag(s_wx_city_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_temp_lbl)
        lv_obj_add_flag(s_wx_temp_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_deg_lbl)
        lv_obj_add_flag(s_wx_deg_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_text_lbl)
        lv_obj_add_flag(s_wx_text_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_feels_lbl)
        lv_obj_add_flag(s_wx_feels_lbl, LV_OBJ_FLAG_HIDDEN);
    if (s_wx_stats_lbl)
        lv_obj_add_flag(s_wx_stats_lbl, LV_OBJ_FLAG_HIDDEN);
}

/* ═══════════════════════════════════════════════════════════════
 * 闹钟编辑界面（4步：时 → 分 → 重复 → 开关）
 * ═══════════════════════════════════════════════════════════════ */
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

    /* 标题 */
    lv_obj_t *title = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_color(title, lv_color_white(), 0);
    lv_obj_set_style_text_font(title, &font_cn_16, 0);
    lv_label_set_text(title, "设置闹钟");
    lv_obj_align(title, LV_ALIGN_TOP_MID, 0, 8);

    /* 小时 */
    s_edit_hour_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_hour_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_hour_lbl, lv_color_hex(0xFF9500), 0);
    lv_obj_align(s_edit_hour_lbl, LV_ALIGN_CENTER, -55, -20);

    /* 冒号 */
    s_edit_colon_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_colon_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_colon_lbl, lv_color_white(), 0);
    lv_label_set_text(s_edit_colon_lbl, ":");
    lv_obj_align(s_edit_colon_lbl, LV_ALIGN_CENTER, 0, -20);

    /* 分钟 */
    s_edit_min_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_min_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_edit_min_lbl, lv_color_white(), 0);
    lv_obj_align(s_edit_min_lbl, LV_ALIGN_CENTER, 55, -20);

    /* 重复模式 */
    s_edit_repeat_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_repeat_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_edit_repeat_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_edit_repeat_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_edit_repeat_lbl, LV_ALIGN_CENTER, 0, 30);

    /* 开关状态（新增第四步） */
    s_edit_enable_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_enable_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_edit_enable_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_edit_enable_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_edit_enable_lbl, LV_ALIGN_CENTER, 0, 55);
}

static void alarm_edit_render(void) // 显示界面
{
    if (!s_edit_panel)
        return;

    char buf[8];
    snprintf(buf, sizeof(buf), "%02d", s_edit.hour);
    lv_label_set_text(s_edit_hour_lbl, buf);
    snprintf(buf, sizeof(buf), "%02d", s_edit.minute);
    lv_label_set_text(s_edit_min_lbl, buf);

    char rbuf[32];
    snprintf(rbuf, sizeof(rbuf), "重复: %s", s_alarm_repeat_cn(s_edit.repeat));
    lv_label_set_text(s_edit_repeat_lbl, rbuf);

    snprintf(rbuf, sizeof(rbuf), "状态: %s", s_edit.enabled ? "开启" : "关闭");
    lv_label_set_text(s_edit_enable_lbl, rbuf);

    /* 高亮当前编辑项 */
    lv_color_t active = lv_color_hex(0xFF9500);
    lv_color_t inactive = lv_color_white();
    lv_obj_set_style_text_color(s_edit_hour_lbl,
                                (s_edit.state == ALARM_EDIT_HOUR) ? active : inactive, 0);
    lv_obj_set_style_text_color(s_edit_min_lbl,
                                (s_edit.state == ALARM_EDIT_MINUTE) ? active : inactive, 0);
    lv_obj_set_style_text_color(s_edit_repeat_lbl,
                                (s_edit.state == ALARM_EDIT_REPEAT) ? active : inactive, 0);
    lv_obj_set_style_text_color(s_edit_enable_lbl,
                                (s_edit.state == ALARM_EDIT_ENABLE) ? active : inactive, 0);
}

static void alarm_edit_enter(void)
{
    alarm_entry_t list[1];
    uint8_t count = 0;
    reminder_alarm_get_all(list, &count);

    if (count > 0)
    {
        s_edit.hour = list[0].hour;
        s_edit.minute = list[0].minute;
        s_edit.repeat = list[0].repeat;
        s_edit.enabled = list[0].enabled;
    }
    else
    {
        s_edit.hour = 8;
        s_edit.minute = 0;
        s_edit.repeat = ALARM_REPEAT_ONCE;
        s_edit.enabled = true;
    }

    s_edit.state = ALARM_EDIT_HOUR;

    if (lvgl_port_lock(100))
    {
        alarm_edit_create(); /* 创建LVGL对象必须在锁内，防止与LVGL定时器任务竞争 */
        alarm_edit_render();
        lv_obj_clear_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (s_menu_panel)
            lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
        lvgl_port_unlock();
    }
    s_view = UI_VIEW_ALARM_EDIT;
    menu_cancel_idle_timer();
    ESP_LOGI(TAG, "进入闹钟编辑");
}

static void alarm_edit_exit(bool save)
{
    if (save)
    {
        alarm_entry_t entry = {
            .hour = s_edit.hour,
            .minute = s_edit.minute,
            .repeat = s_edit.repeat,
            .enabled = s_edit.enabled,
        };
        memset(entry.message, 0, sizeof(entry.message));

        alarm_entry_t list[1];
        uint8_t count = 0;
        reminder_alarm_get_all(list, &count);
        if (count > 0)
            reminder_alarm_update(0, &entry);
        else
            reminder_alarm_add(&entry);

        ESP_LOGI(TAG, "闹钟已保存: %02d:%02d %s",
                 s_edit.hour, s_edit.minute, s_edit.enabled ? "开启" : "关闭");
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

static void alarm_edit_value_next(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        s_edit.hour = (s_edit.hour + 1) % 24;
        break;
    case ALARM_EDIT_MINUTE:
        s_edit.minute = (s_edit.minute + ALARM_MIN_STEP <= ALARM_MIN_MAX)
                            ? s_edit.minute + ALARM_MIN_STEP
                            : 0;
        break;
    case ALARM_EDIT_REPEAT:
    {
        uint8_t idx = 0;
        for (uint8_t i = 0; i < REPEAT_MODE_COUNT; i++)
            if (s_repeat_modes[i] == s_edit.repeat)
            {
                idx = i;
                break;
            }
        s_edit.repeat = s_repeat_modes[(idx + 1) % REPEAT_MODE_COUNT];
        break;
    }
    case ALARM_EDIT_ENABLE:
        s_edit.enabled = !s_edit.enabled;
        break;
    }
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

static void alarm_edit_value_prev(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        s_edit.hour = (s_edit.hour + 23) % 24;
        break;
    case ALARM_EDIT_MINUTE:
        s_edit.minute = (s_edit.minute >= ALARM_MIN_STEP)
                            ? s_edit.minute - ALARM_MIN_STEP
                            : ALARM_MIN_MAX;
        break;
    case ALARM_EDIT_REPEAT:
    {
        uint8_t idx = 0;
        for (uint8_t i = 0; i < REPEAT_MODE_COUNT; i++)
            if (s_repeat_modes[i] == s_edit.repeat)
            {
                idx = i;
                break;
            }
        s_edit.repeat = s_repeat_modes[(idx + REPEAT_MODE_COUNT - 1) % REPEAT_MODE_COUNT];
        break;
    }
    case ALARM_EDIT_ENABLE:
        s_edit.enabled = !s_edit.enabled;
        break;
    }
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

/* 头部短按：循环切换编辑字段（时→分→重复→开关→时） */
static void alarm_edit_advance(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        s_edit.state = ALARM_EDIT_MINUTE;
        break;
    case ALARM_EDIT_MINUTE:
        s_edit.state = ALARM_EDIT_REPEAT;
        break;
    case ALARM_EDIT_REPEAT:
        s_edit.state = ALARM_EDIT_ENABLE;
        break;
    case ALARM_EDIT_ENABLE:
        s_edit.state = ALARM_EDIT_HOUR;
        break;
    }
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

/* 头部短按反向：循环切换编辑字段（时→开关→重复→分→时） */
static void alarm_edit_back(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        s_edit.state = ALARM_EDIT_ENABLE;
        break;
    case ALARM_EDIT_MINUTE:
        s_edit.state = ALARM_EDIT_HOUR;
        break;
    case ALARM_EDIT_REPEAT:
        s_edit.state = ALARM_EDIT_MINUTE;
        break;
    case ALARM_EDIT_ENABLE:
        s_edit.state = ALARM_EDIT_REPEAT;
        break;
    }
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

/* ═══════════════════════════════════════════════════════════════
 * 倒计时页面（保持不变）
 * ═══════════════════════════════════════════════════════════════ */
static void countdown_page_create(void)
{
    if (s_cd_time_lbl != NULL)
        return;

    /* 运行/到期状态：整体时间标签 MM:SS（SET 状态隐藏） */
    s_cd_time_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_time_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_cd_time_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_cd_time_lbl, LV_ALIGN_CENTER, 0, -20);

    /* 设置状态：分钟/冒号/秒钟 三个独立标签，可分别上色（RUNNING/EXPIRED 时隐藏） */
    s_cd_min_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_min_lbl, &lv_font_montserrat_48, 0);
    lv_obj_align(s_cd_min_lbl, LV_ALIGN_CENTER, -55, -20);

    s_cd_colon_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_colon_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cd_colon_lbl, lv_color_white(), 0);
    lv_label_set_text(s_cd_colon_lbl, ":");
    lv_obj_align(s_cd_colon_lbl, LV_ALIGN_CENTER, 0, -20);

    s_cd_sec_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_sec_lbl, &lv_font_montserrat_48, 0);
    lv_obj_align(s_cd_sec_lbl, LV_ALIGN_CENTER, 55, -20);

    s_cd_state_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_state_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_cd_state_lbl, lv_color_hex(0x88CCFF), 0);
    lv_obj_set_style_text_align(s_cd_state_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_cd_state_lbl, LV_ALIGN_CENTER, 0, 30);

    s_cd_hint_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_hint_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_cd_hint_lbl, lv_color_hex(0x666666), 0);
    lv_label_set_long_mode(s_cd_hint_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_cd_hint_lbl, BSP_LCD_WIDTH - 16);
    lv_obj_set_style_text_align(s_cd_hint_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_cd_hint_lbl, LV_ALIGN_BOTTOM_MID, 0, -8);
}

static void countdown_page_render(void)
{
    if (!s_cd_time_lbl)
        return;

    char buf[16];
    switch (s_cd.state)
    {
    case CD_STATE_SET:
    {
        /* 整体标签隐藏，改用分色的三段标签 */
        lv_obj_add_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cd_min_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cd_colon_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_clear_flag(s_cd_sec_lbl, LV_OBJ_FLAG_HIDDEN);

        /* 橙色=当前编辑字段，白色=另一字段 */
        lv_color_t active = lv_color_hex(0xFF9500);
        lv_color_t inactive = lv_color_white();
        snprintf(buf, sizeof(buf), "%02d", s_cd.minutes);
        lv_label_set_text(s_cd_min_lbl, buf);
        lv_obj_set_style_text_color(s_cd_min_lbl,
                                    s_cd.editing_sec ? inactive : active, 0);
        snprintf(buf, sizeof(buf), "%02d", s_cd.seconds);
        lv_label_set_text(s_cd_sec_lbl, buf);
        lv_obj_set_style_text_color(s_cd_sec_lbl,
                                    s_cd.editing_sec ? active : inactive, 0);

        lv_label_set_text(s_cd_state_lbl, "");
        lv_label_set_text(s_cd_hint_lbl, "");
        break;
    }
    case CD_STATE_RUNNING:
    {
        /* 三段标签隐藏，改回整体标签 */
        lv_obj_clear_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_min_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_colon_lbl, LV_OBJ_FLAG_HIDDEN);
        lv_obj_add_flag(s_cd_sec_lbl, LV_OBJ_FLAG_HIDDEN);

        uint32_t remain = 0;
        reminder_timer_get_remain(s_cd.timer_id, &remain);
        snprintf(buf, sizeof(buf), "%02lu:%02lu",
                 (unsigned long)(remain / 60), (unsigned long)(remain % 60));
        lv_label_set_text(s_cd_time_lbl, buf);
        lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_hex(0xFF9500), 0);
        lv_label_set_text(s_cd_state_lbl, "倒计时中...");
        lv_label_set_text(s_cd_hint_lbl, "");
        break;
    }
    case CD_STATE_EXPIRED:
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
}
/**
 *  倒计时页面
 *  - 短按加减分钟，长按开始/取消
 *  - 运行时显示剩余时间，结束时显示提示并震动
 * */

static void countdown_tick_cb(lv_timer_t *t)
{
    (void)t;
    if (s_cd.state != CD_STATE_RUNNING)
        return;

    uint32_t remain = 0;
    if (reminder_timer_get_remain(s_cd.timer_id, &remain) != ESP_OK || remain == 0)
    {
        s_cd.state = CD_STATE_EXPIRED;
        countdown_page_render();
        /* 注意：bsp_motor_pulse 阻塞 30ms，此处会短暂阻塞 LVGL。
         * 如需优化，可改为发送事件到非 LVGL 任务执行震动。 */
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
    s_cd.timer_id = reminder_timer_start((uint32_t)s_cd.minutes * 60 + s_cd.seconds, "倒计时结束");
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
    ESP_LOGI(TAG, "倒计时启动: %d 分 %d 秒", s_cd.minutes, s_cd.seconds);
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

static void render_fn_page(fn_page_t page)
{
    if (s_menu_title == NULL || s_menu_body == NULL)
        return;

    alarm_page_hide();
    countdown_page_hide();
    weather_page_hide();

    /* 进入功能页时隐藏功能盘的大图标（避免残留遮挡） */
    if (s_home_icon)
        lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);

    lv_label_set_text(s_menu_title, s_fn_page_titles[page]);

#if CONFIG_UI_USE_CALENDAR
    // 如果启用了月历，则尝试隐藏它（防止在其他页面显示）
    if (s_calendar)
        lv_obj_add_flag(s_calendar, LV_OBJ_FLAG_HIDDEN);
#endif

    lv_label_set_text(s_menu_title, s_fn_page_titles[page]);

    switch (page)
    {
    case FN_PAGE_TIME:
#if CONFIG_UI_USE_CALENDAR
        // 模式 A: 显示月历，隐藏文字标签
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        time_page_render_calendar();
#else
        // 模式 B: 回退到原始逻辑，显示文字标签
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
    if (gif_obj != NULL)
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
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
    alarm_page_hide();
    countdown_page_hide();
    weather_page_hide();
#if CONFIG_UI_USE_CALENDAR
    if (s_calendar)
        lv_obj_add_flag(s_calendar, LV_OBJ_FLAG_HIDDEN);
#endif
    /* 离开功能盘进入功能页/文字页时，隐藏功能盘的大图标 */
    if (s_home_icon)
        lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);
    if (s_menu_body)
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
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
    if (s_calendar)
        lv_obj_add_flag(s_calendar, LV_OBJ_FLAG_HIDDEN);
#endif
    if (s_menu_title)
        lv_label_set_text(s_menu_title, "");
    if (s_menu_body)
        lv_obj_add_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);

    const home_item_t *it = &s_home_items[s_home_idx];
    if (it->img != NULL)
    {
        /* 有图：显示图片，关掉占位色块的背景，尺寸由图片自身决定 */
        lv_image_set_src(s_home_icon, it->img);
        lv_obj_set_style_bg_opa(s_home_icon, LV_OPA_TRANSP, 0);
        lv_obj_set_size(s_home_icon, LV_SIZE_CONTENT, LV_SIZE_CONTENT);
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
    lv_obj_clear_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);
}

/* ── 果冻弹动动画：两段式位移（exec/ready 回调由 LVGL 在自身上下文调用）── */

/* 位移执行回调：用 translate_x 偏移，不破坏 lv_obj_align 的居中基准 */
static void home_anim_x_cb(void *obj, int32_t v)
{
    lv_obj_set_style_translate_x((lv_obj_t *)obj, (lv_coord_t)v, 0);
}

/* 第二段（回弹）结束：清除动画进行标志 */
static void home_anim_phase2_ready(lv_anim_t *a)
{
    (void)a;
    s_home_animating = false;
}

/* 第一段（弹出）结束：切到目标项并起第二段（带 overshoot 回弹的果冻感） */
static void home_anim_phase1_ready(lv_anim_t *a)
{
    int dir = (int)(intptr_t)a->user_data; // +1 下一个 / -1 上一个

    /* 换内容：第一段已把图标弹到 -dir*PUNCH 处，此刻切到目标项 */
    s_home_idx = (s_home_idx + dir + (int)HOME_ITEM_COUNT) % (int)HOME_ITEM_COUNT;
    home_render();

    /* 第二段：从 +dir*PUNCH 处回弹到 0（OVERSHOOT 路径制造果冻回弹） */
    lv_anim_t a2;
    lv_anim_init(&a2);
    lv_anim_set_var(&a2, s_home_icon);
    lv_anim_set_exec_cb(&a2, home_anim_x_cb);
    lv_anim_set_values(&a2, dir * HOME_PUNCH_PX, 0);
    lv_anim_set_time(&a2, 180);
    lv_anim_set_path_cb(&a2, lv_anim_path_overshoot);
    lv_anim_set_ready_cb(&a2, home_anim_phase2_ready);
    lv_anim_start(&a2);
}

/**
 * @brief 果冻弹动切换：摸耳后图标先朝反方向弹出、切项、再回弹归位
 * @param dir +1=下一个（图标先向左弹）；-1=上一个（图标先向右弹）
 *
 * 已假定调用方持有 LVGL 锁。s_home_animating 在外层判过，此处置位。
 */
static void home_jelly(int dir)
{
    if (s_home_icon == NULL)
        return;
    s_home_animating = true;

    /* 第一段：从 0 弹到 -dir*PUNCH（与切换方向相反，制造"被推开"的打击感） */
    lv_anim_t a1;
    lv_anim_init(&a1);
    lv_anim_set_var(&a1, s_home_icon);
    lv_anim_set_exec_cb(&a1, home_anim_x_cb);
    lv_anim_set_values(&a1, 0, -dir * HOME_PUNCH_PX);
    lv_anim_set_time(&a1, 120);
    lv_anim_set_path_cb(&a1, lv_anim_path_ease_out);
    lv_anim_set_user_data(&a1, (void *)(intptr_t)dir);
    lv_anim_set_ready_cb(&a1, home_anim_phase1_ready);
    lv_anim_start(&a1);
}

/* 进游戏公共逻辑：隐藏功能盘图标 + 切游戏视图 + 启动指定游戏 */
static void enter_game_common(game_id_t id)
{
    if (lvgl_port_lock(100))
    {
        if (s_home_icon)
            lv_obj_add_flag(s_home_icon, LV_OBJ_FLAG_HIDDEN);
        lvgl_port_unlock();
    }
    s_view = UI_VIEW_GAME;
    games_start(id);
    /* 游戏期间禁用空闲超时：难度选择/倒计时/游戏进行/结算都可能长时间无翻页式输入，
     * 不应被自动踢回主界面。退出游戏（腹背→back_to_home / 长按耳→exit_to_main /
     * 空闲回调本身）会重新建立或不再需要该计时器。 */
    menu_cancel_idle_timer();
}

/* 三个游戏的功能盘入口（赛车/跳一跳首版为占位游戏，进去显示"敬请期待"） */
static void enter_whack(void) { enter_game_common(GAME_WHACK); }
static void enter_race(void) { enter_game_common(GAME_RACE); }
static void enter_jump(void) { enter_game_common(GAME_JUMP); }

/**
 * @brief 进入功能盘界面（主界面长按耳触发）
 */
void ui_home_enter(void)
{
    // 进功能盘立即清空舵机队列 + 打断当前动作 + 平滑归中（非阻塞，不碰 LVGL）。
    // 解决「进盘后 servo_manager 队列堆积的旧动作继续做、舵机还重复动多次」的 bug。
    // 若此刻正在播情绪：flush 打断情绪舵机 → worker give done → interaction 解阻塞
    //   → 清 is_playing → 调 ui_resume_main_gif_loop，但下面已把 s_view 设为 HOME，
    //   故 resume 内部判 s_view!=MAIN 直接返回，不会误恢复主界面循环（plan R3）。
    servo_manager_flush();

    if (!lvgl_port_lock(100))
        return;
    ensure_menu_panel();
    s_view = UI_VIEW_HOME;
    s_home_idx = 0;
    s_home_animating = false;
    /* 进入时清掉可能残留的位移，确保图标居中 */
    if (s_home_icon)
        lv_obj_set_style_translate_x(s_home_icon, 0, 0);
    if (gif_obj != NULL)
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    home_render();
    lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    menu_kick_idle_timer();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "进入功能盘");
}

/**
 * @brief 任意功能层 → 直接返回主界面（长按耳）
 */
void ui_func_layer_exit_to_main(void)
{
    /* 若正从游戏中退出，先释放游戏资源（幂等：非游戏态为空操作） */
    games_stop();
    if (!lvgl_port_lock(100))
        return;
    if (s_menu_panel)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    if (gif_obj != NULL)
        lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    s_view = UI_VIEW_MAIN;
    main_gif_kick_resume();
    menu_cancel_idle_timer();
    lvgl_port_unlock();
    ESP_LOGI(TAG, "功能层 → 返回主界面");
}

void ui_force_back_to_main(void)
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
    if (!lvgl_port_lock(100))
        return;
    if (s_view == UI_VIEW_MAIN)
    {
        lvgl_port_unlock();
        return; /* 已在主界面，无需处理 */
    }
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

static void app_enter_calendar(void) { menu_enter_fn_page(FN_PAGE_TIME); }
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
 * @brief 显示「正在重置，请稍候…」解绑提示页（取消绑定/出厂重置前调用）
 *
 * 背景（为何需要本接口）：
 *   解绑流程 clear_wifi_and_restart() 会做 NVS 擦除 + esp_wifi_restore，
 *   这些 flash 写操作会短暂禁用 flash cache。而主界面 GIF 每帧都要读 SPIFFS(flash)，
 *   cache 一禁用就取不到下一帧，画面便「卡在当前帧」直到 esp_restart 黑屏，体验像死机。
 *
 * 优化做法：
 *   在动 flash 之前先切到一张【纯静态文字】画面——隐藏 GIF、显示复用的 s_menu_panel，
 *   并用 lv_refr_now() 同步把它刷到屏幕上。静态 label 只渲染一次、不逐帧读 flash，
 *   因此后续 cache 被禁用也不影响显示，用户看到的是明确的「正在重置」而非冻帧。
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

    /* 1) 隐藏会逐帧读 flash 的 GIF，避免它在 cache 禁用后留下冻帧 */
    if (gif_obj != NULL)
        lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);

    /* 2) 复用功能菜单的全屏黑底文字面板，写入提示文案 */
    ensure_menu_panel();
    if (s_menu_panel != NULL)
    {
        menu_clear_func_pages(); /* 清掉日历/闹钟等功能页专属对象，仅留 title+body */
        if (s_menu_title)
            lv_label_set_text(s_menu_title, "");
        if (s_menu_body)
            lv_label_set_text(s_menu_body, "正在重置中");
        lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    }

    /* 3) 同步刷新：在解锁返回前把这一帧真正画到 LCD 上。
     *    关键——必须趁现在 flash cache 还可用时完成渲染，否则调用方一旦开始
     *    擦 NVS，cache 被禁用，这张提示页就再也刷不上去了。 */
    lv_refr_now(NULL);

    lvgl_port_unlock();
    ESP_LOGW(TAG, "已显示解绑提示页（正在重置中）");
}

/* ═══════════════════════════════════════════════════════════════
 * 动画播放
 * ═══════════════════════════════════════════════════════════════ */
typedef struct
{
    const char *anim_id;
    void (*play_func)(void);
    const char *audio_file;
} animation_map_t;

/**
 * 动画映射表：将动画 ID 映射到对应的播放函数和音频文件路径。
 * 目前仅包含一个示例动画 "anim_happy_stars"，触发时会调用 gif_switch_source() 切换 GIF 图源，并输出关联的音频文件路径。
 */
static const animation_map_t s_animation_map[] = {
    {"anim_happy_stars", gif_switch_source, "S:/laugh_short.mp3"},
};

void ui_play_animation(const char *anim_id)
{
    if (anim_id == NULL)
        return;
    static const size_t MAP_LEN = sizeof(s_animation_map) / sizeof(s_animation_map[0]);
    size_t idx;
    for (idx = 0; idx < MAP_LEN; idx++)
        if (strcmp(anim_id, s_animation_map[idx].anim_id) == 0)
            break;
    if (idx == MAP_LEN)
    {
        ESP_LOGW(TAG, "未知动画 ID: %s", anim_id);
        return;
    }

    const animation_map_t *entry = &s_animation_map[idx];
    if (lvgl_port_lock(100))
    {
        if (gif_obj != NULL)
        {
            lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
            entry->play_func();
        }
        lvgl_port_unlock();
    }
    if (entry->audio_file != NULL)
        ESP_LOGI(TAG, "音频: %s", entry->audio_file);
    ESP_LOGI(TAG, "动画触发: %s", anim_id);
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
        lv_fs_res_t res = lv_fs_open(&f, "S:/one.gif", LV_FS_MODE_RD);
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
static const robot_emotion_t emo_group_head[] = {
    EMO_HAPPY, EMO_CURIOUS, EMO_TSUNDERE, EMO_TICKLISH, EMO_SLEEPY, EMO_GRIEVED};
static const robot_emotion_t emo_group_abdomen[] = {
    EMO_COMFORTABLE, EMO_ACT_CUTE, EMO_ANGRY, EMO_SHY, EMO_SURPRISED, EMO_SLUGGISH};
static const robot_emotion_t emo_group_back[] = {
    EMO_HEALING, EMO_TSUNDERE, EMO_GRIEVED, EMO_EXCITED, EMO_CURIOUS, EMO_TICKLISH};
static const robot_emotion_t emo_group_head_abdomen[] = {
    EMO_EXCITED, EMO_SHY_RUB, EMO_COMFORTABLE_ROLL, EMO_TSUNDERE_PET, EMO_SLEEPY, EMO_SURPRISED};
static const robot_emotion_t emo_group_head_back[] = {
    EMO_HEALING, EMO_TSUNDERE, EMO_GRIEVED, EMO_EXCITED, EMO_CURIOUS, EMO_TICKLISH};
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
    robot_emotion_t emo = group[esp_random() % count];
    ESP_LOGI("TOUCH", "触摸触发情绪: %d（组内随机 %u 选 1）", (int)emo, (unsigned)count);
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

    /* 先记下「本次触摸发生时是否处于待机」。必须在 notify 之前取，
     * 因为下面的 standby_notify_activity() 会顺带唤醒、把待机标志清掉。 */
    bool was_standby = standby_is_deep_active();

    /* 任意触摸都算「活动」：刷新待机倒计时，且若在待机中则一并退出待机
     * （头部/腹背/左右翻页等任意部位皆可唤醒，唤醒逻辑收口在此函数内部）。 */
    standby_notify_activity();

    /* 待机被本次触摸唤醒时，消费掉这次事件：只做「唤醒」一件事，
     * 不再继续触发翻页/情绪/菜单等动作，避免“边唤醒边翻页”。 */
    if (was_standby)
    {
        ESP_LOGI(TAG, "触摸唤醒，退出待机（事件 %d）", (int)event);
        return;
    }

    /* 最高优先级：闹钟响铃中，任意触摸关闭闹钟 */
    if (reminder_get_state() == REMINDER_STATE_RINGING)
    {
        reminder_alarm_dismiss();
        if (lvgl_port_lock(100))
        {
            if (gif_obj)
                lv_obj_add_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
            lvgl_port_unlock();
        }
        ESP_LOGI(TAG, "触摸关闭闹钟");
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
         * 长按耳保留兼容（仍进功能盘），避免误判长按时无响应。 */
        case TOUCH_EVENT_SHORT_PREV_PAGE:
        case TOUCH_EVENT_SHORT_NEXT_PAGE:
        case TOUCH_EVENT_LONG_PREV_PAGE:
        case TOUCH_EVENT_LONG_NEXT_PAGE:
            ui_home_enter(); /* 短按/长按耳 → 进功能盘 */
            break;
        default:
            break;
        }
        break;

    /* ─── 功能盘（单层）：左右耳果冻弹动横向切换，头部短按确认进入，长按头部退出 ─── */
    case UI_VIEW_HOME:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_PREV_PAGE: /* 左耳 → 上一个：图标先向右弹再切 */
            if (!s_home_animating && lvgl_port_lock(100))
            {
                home_jelly(-1);
                menu_kick_idle_timer();
                lvgl_port_unlock();
            }
            break;
        case TOUCH_EVENT_SHORT_NEXT_PAGE: /* 右耳 → 下一个：图标先向左弹再切 */
            if (!s_home_animating && lvgl_port_lock(100))
            {
                home_jelly(+1);
                menu_kick_idle_timer();
                lvgl_port_unlock();
            }
            break;
        case TOUCH_EVENT_SHORT_HEAD: /* 头部短按确认：震动 + 进入对应功能 */
            bsp_motor_pulse();
            if (s_home_items[s_home_idx].on_enter)
                s_home_items[s_home_idx].on_enter();
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
        case TOUCH_EVENT_LONG_HEAD: /* 头部长按：退出游戏，回功能盘 */
            games_stop();
            back_to_home();
            ESP_LOGI(TAG, "退出游戏，返回功能盘");
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
            /* 倒计时页：短按减分钟或减秒钟（当前编辑字段由头按切换）。
             * 注：已去除功能页之间的左右耳翻页（日历/闹钟/定时器/天气切换），
             * 应用选择统一在"应用列表"层完成，进入应用后锁定当前页；
             * 想换应用请摸腹/背返回应用列表重新选。其它页此处无作用。 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                if (s_cd.editing_sec)
                    s_cd.seconds = (s_cd.seconds >= CD_SEC_STEP) ? s_cd.seconds - CD_SEC_STEP : 0;
                else
                    s_cd.minutes = (s_cd.minutes >= CD_MIN_STEP) ? s_cd.minutes - CD_MIN_STEP : 0;
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            break;

        case TOUCH_EVENT_SHORT_NEXT_PAGE:
            /* 倒计时页：短按加分钟或加秒钟（当前编辑字段由头按切换）。其它页无作用。 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                if (s_cd.editing_sec)
                    s_cd.seconds = (s_cd.seconds + CD_SEC_STEP <= CD_SEC_MAX)
                                       ? s_cd.seconds + CD_SEC_STEP
                                       : 0;
                else
                    s_cd.minutes = (s_cd.minutes + CD_MIN_STEP <= CD_MIN_MAX)
                                       ? s_cd.minutes + CD_MIN_STEP
                                       : 0;
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            break;

        case TOUCH_EVENT_LONG_NEXT_PAGE:
            if (s_fn_page == FN_PAGE_ALARM)
                alarm_edit_enter(); /* 闹钟页：进入编辑 */
            else if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
                countdown_start(); /* 倒计时页：启动 */
            else
                ui_function_menu_exit(); /* 其他页：退出菜单 */
            break;

        case TOUCH_EVENT_LONG_PREV_PAGE:
            if (s_fn_page == FN_PAGE_COUNTDOWN)
            {
                if (s_cd.state == CD_STATE_RUNNING)
                    countdown_cancel(); /* 运行中：取消 */
                else if (s_cd.state == CD_STATE_EXPIRED)
                {
                    s_cd.state = CD_STATE_SET; /* 已到期：重置 */
                    if (lvgl_port_lock(100))
                    {
                        countdown_page_render();
                        lvgl_port_unlock();
                    }
                }
                else
                    ui_function_menu_exit(); /* 设置中：退出菜单 */
            }
            else
                ui_function_menu_exit(); /* 非倒计时页：退出菜单 */
            break;

        case TOUCH_EVENT_SHORT_HEAD:
            /* 倒计时设置页：头部短按切换编辑字段（分钟 ↔ 秒钟）。其它页无作用。 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                s_cd.editing_sec = !s_cd.editing_sec;
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            break;

        case TOUCH_EVENT_LONG_HEAD: /* 头部长按：退出功能页回功能盘 */
            ui_function_menu_exit();
            break;
        default:
            break;
        }
        break;

    /* ─── 闹钟编辑模式 ───
     * 左耳短按：当前字段 -     右耳短按：当前字段 +
     * 头部短按：向前切换字段（时→分→重复→开关→时，循环）
     * 左耳长按：向后切换字段（反向循环）
     * 头部长按：保存退出       右耳长按：放弃退出 */
    case UI_VIEW_ALARM_EDIT:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_NEXT_PAGE: /* 右耳短按：当前字段 + */
            alarm_edit_value_next();
            break;
        case TOUCH_EVENT_SHORT_PREV_PAGE: /* 左耳短按：当前字段 - */
            alarm_edit_value_prev();
            break;
        case TOUCH_EVENT_SHORT_HEAD: /* 头部短按：向前切换字段 */
            alarm_edit_advance();
            break;
        case TOUCH_EVENT_LONG_PREV_PAGE: /* 左耳长按：向后切换字段 */
            alarm_edit_back();
            break;
        case TOUCH_EVENT_LONG_HEAD: /* 头部长按：保存退出 */
            alarm_edit_exit(true);
            break;
        case TOUCH_EVENT_LONG_NEXT_PAGE: /* 右耳长按：放弃退出 */
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
