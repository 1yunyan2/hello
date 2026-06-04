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
#include "bsp/bsp_config.h"
#include "bsp/bsp_board.h"
#include "bsp/servo_manager.h" /* GIF 切换时只驱动舵机:非阻塞入队 + 独立 worker 执行 */
#include "esp_spiffs.h"
#include "ui/reminder.h"
#include "ui/standby.h"
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
static void alarm_edit_back_or_cancel(void);

/* ── 配置宏 ── */
#define UI_MAIN_GIF_RANDOM 1
#define UI_MENU_IDLE_TIMEOUT_MS 30000

/* ═══════════════════════════════════════════════════════════════
 * 闹钟编辑状态机（4步：时 → 分 → 重复 → 开关）
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    ALARM_EDIT_HOUR,
    ALARM_EDIT_MINUTE,
    ALARM_EDIT_REPEAT,
    ALARM_EDIT_ENABLE,
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

/* ── 主界面 GIF 自动循环状态(均在 LVGL 线程读写,无需加锁/原子)──
 *   s_gif_cur_index   : 当前正在播放的 GIF 表索引;-1 表示尚未开始
 *   s_gif_pending_idx : 待切换到的索引;-1 表示当前无待切换(防重复排队)
 *   s_gif_switch_tmr  : 延迟执行 lv_gif_set_src 的 one-shot 定时器
 *                       (不能在 LV_EVENT_READY 回调里直接切图,见 main_gif_ready_cb 说明)
 */
static int s_gif_cur_index = -1;
static int s_gif_pending_idx = -1;
static lv_timer_t *s_gif_switch_tmr = NULL;

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
    int timer_id;
    lv_timer_t *tick_tmr;
} s_cd = {.state = CD_STATE_SET, .minutes = 15, .timer_id = -1, .tick_tmr = NULL};

/* 倒计时 UI 对象 */
static lv_obj_t *s_cd_time_lbl = NULL;
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
    uint8_t count;     // 往返/重复次数;0 = 该轴不动
    bool oscillate;    // true=往返,false=单次到位回中
} gif_servo_action_t;

typedef struct
{
    const char *gif_path;       // SPIFFS 路径,如 "S:/gif/one.gif"
    gif_servo_action_t head;    // 头部舵机动作
    gif_servo_action_t l_arm;   // 左臂舵机动作
    gif_servo_action_t r_arm;   // 右臂舵机动作
} main_gif_entry_t;

/* 各 GIF 的舵机动作(参数为初版默认值,可边测边调)。
 * 约定:{幅度, 方向, 速度, 次数, 是否往返} */
static const main_gif_entry_t s_main_gif_table[] = {
    // one.gif:活泼 —— 头快速往返摇 3 次,双臂中速各摆 2 次
    {"S:/gif/one.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_LEFT, SERVO_SPEED_FAST, 3, true},
     /*l_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_MID, 2, true},
     /*r_arm*/ {SERVO_AMPLITUDE_20, SERVO_DIR_RIGHT, SERVO_SPEED_MID, 2, true}},

    // two.gif:好奇 —— 头缓慢侧偏一下(单次回中),双臂不动
    {"S:/gif/two.gif",
     /*head */ {SERVO_AMPLITUDE_20, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false}},

    // three.gif:舒服 —— 头慢速轻摇 2 次,双臂慢速小幅各摆 1 次
    {"S:/gif/three.gif",
     /*head */ {SERVO_AMPLITUDE_15, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 2, true},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_LEFT, SERVO_SPEED_SLOW, 1, true},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_RIGHT, SERVO_SPEED_SLOW, 1, true}},

    // four.gif:犯困 —— 头极慢大幅侧偏一下,双臂不动
    {"S:/gif/four.gif",
     /*head */ {SERVO_AMPLITUDE_30, SERVO_DIR_RIGHT, SERVO_SPEED_VERY_SLOW, 1, false},
     /*l_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false},
     /*r_arm*/ {SERVO_AMPLITUDE_10, SERVO_DIR_NEUTRAL, SERVO_SPEED_MID, 0, false}},

    /* ↓↓↓ 以后加 GIF 只加一行(路径 + 三轴动作)↓↓↓ */
};
#define MAIN_GIF_COUNT (sizeof(s_main_gif_table) / sizeof(s_main_gif_table[0]))

/**
 * @brief 把一张 GIF 的三轴舵机动作非阻塞入队(只动舵机,无震动/音频)
 *
 * 每个 count>0 的轴生成一个 servo_request_t 提交到 servo_manager 队列,
 * 由其独立 worker 串行执行;count==0 的轴跳过。
 */
static void main_gif_submit_servo(const main_gif_entry_t *entry)
{
    const struct
    {
        uint8_t channel;
        const gif_servo_action_t *act;
    } axes[] = {
        {CH_HEAD, &entry->head},
        {CH_L_ARM, &entry->l_arm},
        {CH_R_ARM, &entry->r_arm},
    };

    for (size_t i = 0; i < sizeof(axes) / sizeof(axes[0]); i++)
    {
        const gif_servo_action_t *a = axes[i].act;
        if (a->count == 0)
            continue; // 该轴本张不参与
        servo_request_t req = {
            .channel = axes[i].channel,
            .amplitude = a->amplitude,
            .direction = a->direction,
            .speed_ms = a->speed_ms,
            .loop_count = a->count,
            .oscillate = a->oscillate,
        };
        servo_manager_submit_request(&req); // 非阻塞入队,失败仅内部打日志,不影响 GIF
    }
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
    return (cur + 1) % (int)MAIN_GIF_COUNT; // 顺序循环
#endif
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

    lv_gif_set_src(gif_obj, entry->gif_path); // 切新图,内部自动重新播放
    s_gif_cur_index = idx;

    if (with_servo)
    {
        main_gif_submit_servo(entry); // 仅驱动三轴舵机,非阻塞入队
        ESP_LOGI(TAG, "主界面 GIF 切换 → [%d] %s (+舵机动作)", idx, entry->gif_path);
    }
    else
    {
        ESP_LOGI(TAG, "主界面 GIF 显示 → [%d] %s (首张不配舵机)", idx, entry->gif_path);
    }
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
    // 仅在主界面才循环切换 GIF + 驱动舵机;进入功能菜单/闹钟编辑后 GIF 被隐藏,
    // 此时不应继续切图或驱动舵机(否则会出现"已在功能层却仍在动"的异常)。
    if (s_view != UI_VIEW_MAIN)
        return;
    if (s_gif_pending_idx >= 0)
        return; // 已有待切换,避免本轮重复排队
    s_gif_pending_idx = main_gif_pick_next_index(s_gif_cur_index);
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

    int idx = s_gif_pending_idx;
    s_gif_pending_idx = -1;
    if (idx < 0)
        return;

    // 双保险:READY 到延迟这一拍之间若已切到功能层,放弃本次切换(不切图、不动舵机)
    if (s_view != UI_VIEW_MAIN)
        return;

    // 切换路径(每次切换都伴随对应舵机动作)
    main_gif_apply_index(idx, /*with_servo=*/true);
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
    s_gif_pending_idx = -1; // 清掉功能层期间可能残留的待切换标记
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

    // 首张:只显示不配舵机(此刻 interaction 队列尚未就绪,且很快会切到下一张)
    int first = main_gif_pick_next_index(-1);
    main_gif_apply_index(first, /*with_servo=*/false);

    lv_obj_center(gif_obj);
    lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "GIF待机动画已创建,首张索引=%d", first);
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
        .task_affinity = 1, // CPU0：与 afe_fetch/audio_feed(CPU1) 隔离，避免抢占唤醒词检测
        .task_max_sleep_ms = 500,
        .timer_period_ms = 10,
        // 栈必须在内部 SRAM，因为 GIF 播放会读 SPIFFS（flash cache 禁用期间 PSRAM 不可访问）
        .task_stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
    };
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK)
        return err;
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
        .buffer_size = BSP_LCD_WIDTH * BSP_LCD_HEIGHT / 5, // 30720 字节：理论上可分配但实测不稳，但是已经是极限了
        .double_buffer = false,
        .hres = BSP_LCD_WIDTH,
        .vres = BSP_LCD_HEIGHT,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {.swap_xy = true, .mirror_x = false, .mirror_y = true},
        .flags = {.buff_dma = true, .swap_bytes = false, .buff_spiram = false}};

    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    if (lvgl_disp == NULL)
    {
        // disp 创建失败时必须返回错误，否则 ui_init 后续会调 lv_screen_active()
        // 拿到失效对象，main_desplay_create 解引用导致 LoadProhibited 崩溃
        ESP_LOGE(TAG, "lvgl_port_add_disp 失败：DMA 内部 SRAM 不足 19200 字节连续区");
        PRINT_MEM_INFO(TAG, "LVGL flush buffer 19200B DMA-SRAM 分配失败");
        return ESP_ERR_NO_MEM;
    }
    PRINT_MEM_INFO(TAG, "LVGL flush buffer 19200B DMA-SRAM 分配后");

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
    if (s_view != UI_VIEW_FUNCTION_MENU)
        return;
    if (s_menu_panel)
        lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
    if (gif_obj != NULL)
        lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    s_view = UI_VIEW_MAIN;
    main_gif_kick_resume(); // 重启 GIF 循环(功能层期间已暂停)
    ESP_LOGI(TAG, "功能菜单空闲超时，返回主界面");
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
        return "仅一次";
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

    /* 操作提示 */
    s_edit_hint_lbl = lv_label_create(s_edit_panel);
    lv_obj_set_style_text_font(s_edit_hint_lbl, &font_cn_16, 0);
    lv_obj_set_style_text_color(s_edit_hint_lbl, lv_color_hex(0x666666), 0);
    lv_label_set_long_mode(s_edit_hint_lbl, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_edit_hint_lbl, BSP_LCD_WIDTH - 16);
    lv_obj_set_style_text_align(s_edit_hint_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_edit_hint_lbl, LV_ALIGN_BOTTOM_MID, 0, -12);
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

    /* 操作提示（按步骤区分） */
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        lv_label_set_text(s_edit_hint_lbl, "短按:+1/-1  长按后页:下一步  长按前页:取消");
        break;
    case ALARM_EDIT_MINUTE:
        lv_label_set_text(s_edit_hint_lbl, "短按:+1/-1  长按后页:下一步  长按前页:退回");
        break;
    case ALARM_EDIT_REPEAT:
        lv_label_set_text(s_edit_hint_lbl, "短按:切换  长按后页:下一步  长按前页:退回");
        break;
    case ALARM_EDIT_ENABLE:
        lv_label_set_text(s_edit_hint_lbl, "短按:开关  长按后页:保存  长按前页:退回");
        break;
    }
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

    if (lvgl_port_lock(100))
    {
        lv_obj_add_flag(s_edit_panel, LV_OBJ_FLAG_HIDDEN);
        if (s_menu_panel)
            lv_obj_clear_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
        alarm_page_rebuild();
        lvgl_port_unlock();
    }
    s_view = UI_VIEW_FUNCTION_MENU;
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
        s_edit.minute = (s_edit.minute + 1) % 60;
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
        s_edit.minute = (s_edit.minute + 59) % 60;
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
        alarm_edit_exit(true);
        return;
    }
    if (lvgl_port_lock(100))
    {
        alarm_edit_render();
        lvgl_port_unlock();
    }
}

static void alarm_edit_back_or_cancel(void)
{
    switch (s_edit.state)
    {
    case ALARM_EDIT_HOUR:
        alarm_edit_exit(false);
        return;
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

    s_cd_time_lbl = lv_label_create(s_menu_panel);
    lv_obj_set_style_text_font(s_cd_time_lbl, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_white(), 0);
    lv_obj_set_style_text_align(s_cd_time_lbl, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_cd_time_lbl, LV_ALIGN_CENTER, 0, -20);

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
        snprintf(buf, sizeof(buf), "%02d:00", s_cd.minutes);
        lv_label_set_text(s_cd_time_lbl, buf);
        lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_white(), 0);
        lv_label_set_text(s_cd_state_lbl, "设置时长");
        lv_label_set_text(s_cd_hint_lbl, "短按:+1/-1分  长按后页:开始");
        break;
    case CD_STATE_RUNNING:
    {
        uint32_t remain = 0;
        reminder_timer_get_remain(s_cd.timer_id, &remain);
        snprintf(buf, sizeof(buf), "%02lu:%02lu",
                 (unsigned long)(remain / 60), (unsigned long)(remain % 60));
        lv_label_set_text(s_cd_time_lbl, buf);
        lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_hex(0xFF9500), 0);
        lv_label_set_text(s_cd_state_lbl, "倒计时中...");
        lv_label_set_text(s_cd_hint_lbl, "长按前页:取消");
        break;
    }
    case CD_STATE_EXPIRED:
        lv_label_set_text(s_cd_time_lbl, "00:00");
        lv_obj_set_style_text_color(s_cd_time_lbl, lv_color_hex(0xFF3333), 0);
        lv_label_set_text(s_cd_state_lbl, "倒计时结束!");
        lv_label_set_text(s_cd_hint_lbl, "长按前页:返回设置");
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
    countdown_page_render();
    lv_obj_clear_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_cd_hint_lbl, LV_OBJ_FLAG_HIDDEN);
    lv_obj_clear_flag(s_cd_state_lbl, LV_OBJ_FLAG_HIDDEN);
}

static void countdown_page_hide(void)
{
    if (s_cd_time_lbl)
        lv_obj_add_flag(s_cd_time_lbl, LV_OBJ_FLAG_HIDDEN);
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
    s_cd.timer_id = reminder_timer_start(s_cd.minutes * 60, "倒计时结束");
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
    ESP_LOGI(TAG, "倒计时启动: %d 分钟", s_cd.minutes);
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
    {
        /* 进入天气页面时触发一次数据拉取（异步，下次进入时刷新） */
        reminder_weather_fetch_now();

        weather_data_t wd;
        reminder_get_weather_data(&wd);

        char buf[128];
        if (wd.valid)
        {
            snprintf(buf, sizeof(buf),
                     "%s\n\n"
                     "温度:  %s%sC\n"
                     "天气:  %s\n",
                     //  "湿度:  %s%%",
                     wd.city_name,
                     wd.temp, "\xC2\xB0",
                     wd.text
                     //  wd.humidity
            );
        }
        else
        {
            snprintf(buf, sizeof(buf),
                     "%s\n\n"
                     "温度:  --%sC\n"
                     "天气:  --\n"
                     //  "湿度:  --\n\n"
                     "等待天气数据...",
                     wd.city_name[0] ? wd.city_name : "定位中",
                     "\xC2\xB0");
        }
        lv_label_set_text(s_menu_body, buf);
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
        break;
    }

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
    if (!lvgl_port_lock(100))
        return;
    if (s_view == UI_VIEW_FUNCTION_MENU)
    {
        if (s_menu_panel)
            lv_obj_add_flag(s_menu_panel, LV_OBJ_FLAG_HIDDEN);
        if (gif_obj != NULL)
            lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
        s_view = UI_VIEW_MAIN;
        main_gif_kick_resume(); // 重启 GIF 循环(功能层期间已暂停)
        ESP_LOGI(TAG, "退出功能菜单，返回主界面");
    }
    menu_cancel_idle_timer();
    lvgl_port_unlock();
}

void ui_page_next(void)
{
    if (!lvgl_port_lock(100))
        return;
    if (s_view == UI_VIEW_FUNCTION_MENU)
    {
        s_fn_page = (fn_page_t)((s_fn_page + 1) % FN_PAGE_COUNT);
        render_fn_page(s_fn_page);
        menu_kick_idle_timer();
        ESP_LOGI(TAG, "菜单 → 下一页: %s", s_fn_page_titles[s_fn_page]);
    }
    lvgl_port_unlock();
}

void ui_page_prev(void)
{
    if (!lvgl_port_lock(100))
        return;
    if (s_view == UI_VIEW_FUNCTION_MENU)
    {
        s_fn_page = (fn_page_t)((s_fn_page + FN_PAGE_COUNT - 1) % FN_PAGE_COUNT);
        render_fn_page(s_fn_page);
        menu_kick_idle_timer();
        ESP_LOGI(TAG, "菜单 → 上一页: %s", s_fn_page_titles[s_fn_page]);
    }
    lvgl_port_unlock();
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
static void battery_label_create_top(void);
static void battery_tick_cb(lv_timer_t *t);
static void status_time_tick_cb(lv_timer_t *t);
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
}

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
    bool was_standby = standby_is_active();

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
    /* ─── 主界面：身体触摸触发情绪，耳朵长按进菜单 ─── */
    case UI_VIEW_MAIN:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_HEAD:
            ui_interaction_play(EMO_HAPPY);
            break;
        case TOUCH_EVENT_SHORT_ABDOMEN:
            ui_interaction_play(EMO_COMFORTABLE);
            break;
        case TOUCH_EVENT_SHORT_BACK:
            ui_interaction_play(EMO_TICKLISH);
            break;
        case TOUCH_EVENT_COMBO_HEAD_ABDOMEN:
            ui_interaction_play(EMO_SHY_RUB);
            break;
        case TOUCH_EVENT_COMBO_HEAD_BACK:
            ui_interaction_play(EMO_EXCITED);
            break;
        case TOUCH_EVENT_COMBO_ABDOMEN_BACK:
            ui_interaction_play(EMO_COMFORTABLE_ROLL);
            break;
        case TOUCH_EVENT_LONG_PREV_PAGE:
        case TOUCH_EVENT_LONG_NEXT_PAGE:
            ui_function_menu_enter();
            break;
        default:
            break;
        }
        break;

    /* ─── 功能菜单：纯导航 + 页面专属操作 ─── */
    case UI_VIEW_FUNCTION_MENU:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_PREV_PAGE:
            /* 倒计时页：短按减分钟；其他页：上翻页 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                s_cd.minutes = (s_cd.minutes > 1) ? s_cd.minutes - 1 : 1;
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            else
                ui_page_prev();
            break;

        case TOUCH_EVENT_SHORT_NEXT_PAGE:
            /* 倒计时页：短按加分钟；其他页：下翻页 */
            if (s_fn_page == FN_PAGE_COUNTDOWN && s_cd.state == CD_STATE_SET)
            {
                s_cd.minutes = (s_cd.minutes < 60) ? s_cd.minutes + 1 : 60;
                if (lvgl_port_lock(100))
                {
                    countdown_page_render();
                    lvgl_port_unlock();
                }
            }
            else
                ui_page_next();
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
        default:
            break;
        }
        break;

    /* ─── 闹钟编辑模式：4步操作 ─── */
    case UI_VIEW_ALARM_EDIT:
        switch (event)
        {
        case TOUCH_EVENT_SHORT_NEXT_PAGE:
            alarm_edit_value_next();
            break;
        case TOUCH_EVENT_SHORT_PREV_PAGE:
            alarm_edit_value_prev();
            break;
        case TOUCH_EVENT_LONG_NEXT_PAGE:
            alarm_edit_advance();
            break;
        case TOUCH_EVENT_LONG_PREV_PAGE:
            alarm_edit_back_or_cancel();
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
