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
#include "esp_spiffs.h"
#include "ui/reminder.h"
#include <string.h>
#include <limits.h>
#include <stdio.h>

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
#define UI_MAIN_GIF_RANDOM 0
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

/* 主时钟 UI */
static lv_obj_t *s_clock_d[6];
static lv_obj_t *s_clock_col[2];
static lv_obj_t *s_time_tz_lbl = NULL;
static lv_obj_t *s_time_date_lbl = NULL;
static lv_timer_t *s_main_tick_tmr = NULL;
static lv_timer_t *s_gif_hide_tmr = NULL;

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
static const char *const s_main_gif_paths[] = {
    "S:/one.gif",
    "S:/two.gif",
    "S:/three.gif",
    "S:/four.gif",
    "S:/five.gif",
};

static const char *pick_main_gif_path(void)
{
#if UI_MAIN_GIF_RANDOM
    size_t n = sizeof(s_main_gif_paths) / sizeof(s_main_gif_paths[0]);
    return s_main_gif_paths[esp_random() % n];
#else
    return s_main_gif_paths[0];
#endif
}

static void gif_switch_source(void)
{
    if (gif_obj == NULL)
        return;
    lv_gif_set_src(gif_obj, pick_main_gif_path());
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
    lv_gif_set_src(gif_obj, pick_main_gif_path());
    lv_obj_center(gif_obj);
    lv_obj_clear_flag(gif_obj, LV_OBJ_FLAG_HIDDEN);
    ESP_LOGI(TAG, "GIF待机动画已创建: %s", pick_main_gif_path());
}

/* ═══════════════════════════════════════════════════════════════
 * LVGL 初始化
 * ═══════════════════════════════════════════════════════════════ */
static esp_err_t app_lvgl_init(void)
{
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
        .task_priority = 3,
        .task_stack = 8192,
        .task_affinity = 1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 10,
        // 栈必须在内部 SRAM，因为 GIF 播放会读 SPIFFS（flash cache 禁用期间 PSRAM 不可访问）
        .task_stack_caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT,
    };
    esp_err_t err = lvgl_port_init(&lvgl_cfg);
    if (err != ESP_OK)
        return err;

    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = board->lcd_io,
        .panel_handle = board->lcd_panel,
        // .buffer_size = (BSP_LCD_WIDTH * BSP_LCD_HEIGHT) / 15,
        .buffer_size = (BSP_LCD_WIDTH * BSP_LCD_HEIGHT) / 8,

        .double_buffer = true,
        .hres = BSP_LCD_WIDTH,
        .vres = BSP_LCD_HEIGHT,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {.swap_xy = true, .mirror_x = false, .mirror_y = true},
        .flags = {.buff_dma = true, .swap_bytes = false, .buff_spiram = true}};

    lvgl_disp = lvgl_port_add_disp(&disp_cfg);
    if (lvgl_disp != NULL)
    {
        lv_display_set_render_mode(lvgl_disp, LV_DISPLAY_RENDER_MODE_PARTIAL);
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
    }
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
    alarm_edit_create();

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

    switch (page)
    {
    case FN_PAGE_TIME:
        time_page_render_text();
        lv_obj_clear_flag(s_menu_body, LV_OBJ_FLAG_HIDDEN);
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
                     "天气:  %s\n"
                     "湿度:  %s%%",
                     wd.city_name,
                     wd.temp, "\xC2\xB0",
                     wd.text,
                     wd.humidity);
        }
        else
        {
            snprintf(buf, sizeof(buf),
                     "%s\n\n"
                     "温度:  --%sC\n"
                     "天气:  --\n"
                     "湿度:  --\n\n"
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
void ui_init(void)
{
    init_spiffs();
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
}

/* ═══════════════════════════════════════════════════════════════
 * 触摸事件分发（极简版：单闹钟 + 单倒计时）
 * ═══════════════════════════════════════════════════════════════ */
ui_view_t ui_get_current_view(void) { return s_view; }

void ui_dispatch_touch_event(touch_event_t event)
{
    if (event == TOUCH_EVENT_NONE)
        return;

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

void ui_update_wifi(int rssi) {}
void ui_update_battery(int soc) {}
