/**
 * @file reminder.c
 * @brief 提醒系统实现 — 闹钟/倒计时/日历/天气 四合一引擎（修复版）
 *
 * 修复清单（共 13 处）：
 *  [FIX-1]  NVS 保存命令常量，闹钟/日历统一走异步队列
 *  [FIX-2]  事件类型新增 REM_EVT_SHUTDOWN，安全关闭任务
 *  [FIX-3]  poll_timer_callback 互斥锁协议修复（检查返回值）
 *  [FIX-4]  alarm_ring_start 定时器创建失败时回退到 IDLE
 *  [FIX-5]  nvs_save_calendars 改为异步（通过队列）
 *  [FIX-6]  nvs_save_task 支持多命令分发
 *  [FIX-7]  reminder_task 处理 SHUTDOWN 事件，安全自退出
 *  [FIX-8]  reminder_init 失败路径完整清理资源
 *  [FIX-9]  reminder_deinit 安全关闭（SHUTDOWN 信号 + 等待）
 *  [FIX-10] strncpy 显式 null 终止
 *  [FIX-11] reminder_task 栈增大到 12KB（天气 HTTP 需要）
 *  [FIX-12] reminder_alarm_update/add 不强制 enabled=true
 *  [FIX-13] esp_timer_create 失败后 alarm_ring_stop 安全处理
 */

#include "reminder.h"
#include "object.h"
#include "bsp/bsp_board.h"   /* bsp_wifi_is_offline_mode()：离线时跳过天气 HTTP 拉取 */
#include "session/session.h" /* session_get_state()：对话进行中不触发闹钟/倒计时（对话优先） */
#include "ui/standby.h"      /* standby_is_deep_active/standby_request_wake：待机中到期先亮屏 */
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_sntp.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "cJSON.h"
#include <string.h>
#include <sys/time.h>
#include "esp_crt_bundle.h"
#include "weather.h"
#include "esp_heap_caps.h"
static const char *TAG = "REMINDER";

/* ═══════════════════════════════════════════════════════════════════
 * 时区配置
 * ═══════════════════════════════════════════════════════════════════ */
#ifndef REMINDER_TZ
#define REMINDER_TZ "CST-8"
#endif

/* ═══════════════════════════════════════════════════════════════════
 * 演示模式
 * ═══════════════════════════════════════════════════════════════════ */
// #define REMINDER_MOCK_TIME

#ifdef REMINDER_MOCK_TIME
#ifndef REMINDER_MOCK_YEAR
#define REMINDER_MOCK_YEAR 2026
#endif
#ifndef REMINDER_MOCK_MON
#define REMINDER_MOCK_MON 4
#endif
#ifndef REMINDER_MOCK_DAY
#define REMINDER_MOCK_DAY 29
#endif
#ifndef REMINDER_MOCK_HOUR
#define REMINDER_MOCK_HOUR 10
#endif
#ifndef REMINDER_MOCK_MIN
#define REMINDER_MOCK_MIN 0
#endif
#endif /* REMINDER_MOCK_TIME */

/* ═══════════════════════════════════════════════════════════════════
 * [FIX-1] NVS 保存命令常量
 * ═══════════════════════════════════════════════════════════════════ */
#define NVS_SAVE_CMD_ALARMS 0x01
#define NVS_SAVE_CMD_CALENDARS 0x02
/* [FIX-14] 天气结果缓存也走异步队列：见 nvs_save_weather_data() 处的说明 */
#define NVS_SAVE_CMD_WEATHER 0x03
#define NVS_SAVE_CMD_EXIT 0xFF

/* ═══════════════════════════════════════════════════════════════════
 * 1. 内部事件定义
 * ═══════════════════════════════════════════════════════════════════ */
typedef enum
{
    REM_EVT_ALARM_TRIGGER,
    REM_EVT_TIMER_EXPIRE,
    REM_EVT_CALENDAR_TRIGGER,
    REM_EVT_WEATHER_FETCH,
    REM_EVT_ALARM_DISMISS,
    REM_EVT_ALARM_RING_TICK,
    /* [FIX-2] 安全关闭信号 */
    REM_EVT_SHUTDOWN,
} reminder_evt_type_t;

typedef struct
{
    reminder_evt_type_t type;
    uint8_t id;
    char message[REMINDER_MSG_MAX_LEN];
} reminder_evt_t;

/* ═══════════════════════════════════════════════════════════════════
 * 2. 运行时上下文
 * ═══════════════════════════════════════════════════════════════════ */
typedef struct
{
    reminder_trigger_cb_t trigger_cb;
    reminder_state_t state;

    alarm_entry_t alarms[REMINDER_MAX_ALARMS];
    uint8_t alarm_count;

    timer_entry_t timers[REMINDER_MAX_TIMERS];

    calendar_entry_t calendars[REMINDER_MAX_CALENDARS];
    uint8_t calendar_count;

    // weather_config_t weather_cfg;
    reminder_weather_cfg_t weather_cfg;

    weather_data_t weather_data; /* 天气实时数据，供 UI 读取 */
    /* [FIX-3 注] 天气标志仅在 poll_timer_callback 中读写（单上下文），
     * reminder_weather_config 写入时持锁，存在良性竞态（最多多/少播报一次） */
    bool weather_morning_done;
    bool weather_evening_done;
    /* 上次拉取的时间点（分钟总数，对应 WEATHER_FETCH_MIN_x），-1=从未拉取
     * 每天4个固定时间点：07:30/11:30/15:30/19:30，上电额外立即拉取一次 */
    int last_weather_fetch_min;

    uint8_t ringing_alarm_id;
    uint8_t ring_count;
    esp_timer_handle_t ring_timer;

    int last_alarm_check_min;
    int last_cal_check_min;

    esp_timer_handle_t poll_timer;
    QueueHandle_t evt_queue;
    TaskHandle_t task_handle;
    SemaphoreHandle_t mutex;
    bool sntp_synced;
    /// @brief 当前系统时间是否来自 NVS 兜底（而非真实 SNTP 同步）。
    /// true 表示时间"大致可用但日期未必准"：闹钟照常触发（只看时分，兜底下依然准点），
    /// 但日历**暂停触发**（日历要看年月日，断电多久日期就差多久，会在错误的日子误报）。
    /// SNTP 一旦同步成功即置回 false，日历自动恢复。
    bool time_from_nvs;
    bool initialized;
} reminder_ctx_t;

static reminder_ctx_t s_ctx = {0};

#define NVS_NAMESPACE "reminder"

/// @brief "系统时间是否有效"的判定门槛（Unix epoch 秒）：2020-01-01 00:00:00 UTC。
/// 小于该值说明 RTC 掉电后未被任何来源设置过（停在 1970），需要用 NVS 兜底值回填。
#define REMINDER_TIME_VALID_EPOCH 1577836800LL

/* ═══════════════════════════════════════════════════════════════════
 * 3. NVS 持久化
 * ═══════════════════════════════════════════════════════════════════ */
static QueueHandle_t s_save_queue = NULL;

static void nvs_save_alarms_immediate(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS 打开失败，无法保存闹钟");
        return;
    }
    nvs_set_u8(handle, "alarm_cnt", s_ctx.alarm_count);
    for (uint8_t i = 0; i < s_ctx.alarm_count; i++)
    {
        char key[16];
        snprintf(key, sizeof(key), "alarm_%d", i);
        nvs_set_blob(handle, key, &s_ctx.alarms[i], sizeof(alarm_entry_t));
    }
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "闹钟数据已保存，共 %d 条", s_ctx.alarm_count);
}

/* [FIX-5] 日历 NVS 写入拆为 immediate 版本，供异步任务调用 */
static void nvs_save_calendars_immediate(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS 打开失败，无法保存日历");
        return;
    }
    nvs_set_u8(handle, "cal_cnt", s_ctx.calendar_count);
    for (uint8_t i = 0; i < s_ctx.calendar_count; i++)
    {
        char key[16];
        snprintf(key, sizeof(key), "cal_%02d", i);
        nvs_set_blob(handle, key, &s_ctx.calendars[i], sizeof(calendar_entry_t));
    }
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "日历数据已保存，共 %d 条", s_ctx.calendar_count);
}

/* 定义在下方（天气小节），此处前置声明供 nvs_save_task 分发使用 */
static void nvs_save_weather_data_immediate(void);

/* [FIX-6][FIX-14] NVS 保存任务支持闹钟/日历/天气/退出四种命令 */
static void nvs_save_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    uint8_t cmd;
    while (1)
    {
        if (xQueueReceive(s_save_queue, &cmd, portMAX_DELAY) == pdTRUE)
        {
            if (cmd == NVS_SAVE_CMD_EXIT)
                break;

            xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
            switch (cmd)
            {
            case NVS_SAVE_CMD_ALARMS:
                nvs_save_alarms_immediate();
                break;
            case NVS_SAVE_CMD_CALENDARS:
                nvs_save_calendars_immediate();
                break;
            case NVS_SAVE_CMD_WEATHER:
                nvs_save_weather_data_immediate();
                break;
            default:
                ESP_LOGW(TAG, "NVS 保存任务收到未知命令: 0x%02X", cmd);
                break;
            }
            xSemaphoreGive(s_ctx.mutex);
        }
    }
    ESP_LOGI(TAG, "NVS 保存任务已退出");
    vTaskDelete(NULL);
}

static void nvs_save_alarms(void)
{
    uint8_t cmd = NVS_SAVE_CMD_ALARMS;
    if (s_save_queue)
        xQueueSend(s_save_queue, &cmd, 0);
}

/* [FIX-5] 日历保存改为异步 */
static void nvs_save_calendars(void)
{
    uint8_t cmd = NVS_SAVE_CMD_CALENDARS;
    if (s_save_queue)
        xQueueSend(s_save_queue, &cmd, 0);
}

static void nvs_load_alarms(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS 无闹钟数据（首次启动）");
        return;
    }
    uint8_t count = 0;
    if (nvs_get_u8(handle, "alarm_cnt", &count) == ESP_OK)
    {
        s_ctx.alarm_count = (count > REMINDER_MAX_ALARMS) ? REMINDER_MAX_ALARMS : count;
        for (uint8_t i = 0; i < s_ctx.alarm_count; i++)
        {
            char key[16];
            snprintf(key, sizeof(key), "alarm_%d", i);
            size_t len = sizeof(alarm_entry_t);
            nvs_get_blob(handle, key, &s_ctx.alarms[i], &len);
        }
        ESP_LOGI(TAG, "从 NVS 加载 %d 条闹钟", s_ctx.alarm_count);
    }
    nvs_close(handle);
}

static void nvs_load_calendars(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return;
    uint8_t count = 0;
    if (nvs_get_u8(handle, "cal_cnt", &count) == ESP_OK)
    {
        s_ctx.calendar_count = (count > REMINDER_MAX_CALENDARS) ? REMINDER_MAX_CALENDARS : count;
        for (uint8_t i = 0; i < s_ctx.calendar_count; i++)
        {
            char key[16];
            snprintf(key, sizeof(key), "cal_%02d", i);
            size_t len = sizeof(calendar_entry_t);
            nvs_get_blob(handle, key, &s_ctx.calendars[i], &len);
        }
        ESP_LOGI(TAG, "从 NVS 加载 %d 条日历事件", s_ctx.calendar_count);
    }
    nvs_close(handle);
}

static void nvs_save_weather_config(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;
    nvs_set_blob(handle, "weather_cfg", &s_ctx.weather_cfg, sizeof(weather_config_t));
    nvs_commit(handle);
    nvs_close(handle);
}

static void nvs_load_weather_config(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        goto use_defaults;
    size_t len = sizeof(weather_config_t);
    if (nvs_get_blob(handle, "weather_cfg", &s_ctx.weather_cfg, &len) == ESP_OK)
    {
        nvs_close(handle);
        return;
    }
    nvs_close(handle);

use_defaults:
    s_ctx.weather_cfg.schedule = WEATHER_SCHEDULE_MORNING;
    strncpy(s_ctx.weather_cfg.city_code, WEATHER_DEFAULT_CITY,
            sizeof(s_ctx.weather_cfg.city_code) - 1);
    s_ctx.weather_cfg.city_code[sizeof(s_ctx.weather_cfg.city_code) - 1] = '\0'; /* [FIX-10] */
    strncpy(s_ctx.weather_cfg.city_name, "北京",
            sizeof(s_ctx.weather_cfg.city_name) - 1);
    s_ctx.weather_cfg.city_name[sizeof(s_ctx.weather_cfg.city_name) - 1] = '\0'; /* [FIX-10] */
}

/**
 * @brief 缓存最近一次成功获取的天气**结果**到 NVS
 *
 * 【与 weather_cfg 的区别】weather_cfg 存的是**配置**（城市、播报时段），本函数存的是
 * **实测数据**（温度/天气/湿度/风力…）。此前只存配置不存结果，导致断网开机时天气页
 * 空白（"等待天气数据 / 湿度—% / 降水量—mm"）。
 *
 * 每次成功获取都覆盖，NVS 里始终是"最后一次联网拿到的天气"。
 *
 * 【★ 必须由 nvs_save_task 调用，禁止在 reminder_task 里直接调】
 * reminder_task 的栈分配在 PSRAM（见 reminder_init 的 MALLOC_CAP_SPIRAM）。
 * NVS 写入最终会走 spi_flash_disable_interrupts_caches_and_other_cpu()，
 * 关闭 flash cache 的同时 PSRAM 也不可访问；此时若当前任务的栈在 PSRAM，
 * 返回地址和局部变量全部读不到，IDF 会主动断言拦下：
 *     assert failed: spi_flash_disable_interrupts_caches_and_other_cpu
 *     cache_utils.c:152 (esp_task_stack_is_sane_cache_disabled())
 * nvs_save_task 的栈是 MALLOC_CAP_INTERNAL，才是安全的执行上下文。
 * 这与 [FIX-5] 把日历保存改异步是同一个原因。
 *
 * @note 调用者：nvs_save_task()，且调用时已持有 s_ctx.mutex
 */
static void nvs_save_weather_data_immediate(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;
    nvs_set_blob(handle, "weather_dat", &s_ctx.weather_data, sizeof(weather_data_t));
    nvs_commit(handle);
    nvs_close(handle);
}

/* [FIX-14] 天气结果保存改为异步：投队列交给内部 SRAM 栈的 nvs_save_task 执行 */
static void nvs_save_weather_data(void)
{
    uint8_t cmd = NVS_SAVE_CMD_WEATHER;
    if (s_save_queue)
        xQueueSend(s_save_queue, &cmd, 0);
}

/**
 * @brief 从 NVS 读回上次缓存的天气结果（开机时调用，供断网时显示）
 *
 * 读不到（首次开机、尚未成功获取过）则保持 valid=false，UI 显示空占位，与旧行为一致。
 *
 * @note 调用者：reminder_init()
 */
static void nvs_load_weather_data(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK)
        return;
    size_t len = sizeof(weather_data_t);
    if (nvs_get_blob(handle, "weather_dat", &s_ctx.weather_data, &len) == ESP_OK &&
        len == sizeof(weather_data_t))
    {
        ESP_LOGI(TAG, "已从 NVS 载入上次天气缓存：%s %s°C",
                 s_ctx.weather_data.text, s_ctx.weather_data.temp);
    }
    nvs_close(handle);
}

/* ═══════════════════════════════════════════════════════════════════
 * 4. SNTP 时间同步
 * ═══════════════════════════════════════════════════════════════════ */
/**
 * @brief 把当前系统时间写入 NVS，作为下次断网开机的时间兜底
 *
 * 【为什么需要】ESP32 掉电后 RTC 清零，若开机时没网（SNTP 同步不上），系统时间会停在
 * 1970-01-01，导致 poll_timer_callback 里所有依赖时间的功能全部失效。设备第一次一定
 * 是配网成功的，所以 NVS 里必然存过一份"最后一次联网时的时间"，可以拿它当起点。
 *
 * 【覆盖规则】只在**真正 SNTP 同步成功**时调用（有更准的来源才覆盖）。离线兜底恢复的
 * 时间**绝不写回 NVS**——那是不断劣化的估算值，写回去会把好数据一次次冲旧。
 *
 * @note 调用者：sntp_sync_notification_cb()（SNTP 同步成功回调）
 */
static void nvs_save_last_time(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK)
        return;
    int64_t now = (int64_t)time(NULL);
    nvs_set_i64(handle, "last_time", now);
    nvs_commit(handle);
    nvs_close(handle);
    ESP_LOGI(TAG, "已保存联网时间到 NVS（供下次断网开机兜底）");
}

static void sntp_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "SNTP 时间同步完成");
    s_ctx.sntp_synced = true;
    s_ctx.time_from_nvs = false; /* 拿到真实时间，兜底标记撤销 → 日历恢复触发 */
    nvs_save_last_time();        /* 有更准的来源，覆盖 NVS 旧值 */
}

static void sntp_time_sync_init(void)
{
    setenv("TZ", REMINDER_TZ, 1);
    tzset();

#ifdef REMINDER_MOCK_TIME
    struct tm mock_tm = {
        .tm_year = REMINDER_MOCK_YEAR - 1900,
        .tm_mon = REMINDER_MOCK_MON - 1,
        .tm_mday = REMINDER_MOCK_DAY,
        .tm_hour = REMINDER_MOCK_HOUR,
        .tm_min = REMINDER_MOCK_MIN,
        .tm_sec = 0,
        .tm_isdst = -1,
    };
    time_t t = mktime(&mock_tm);
    struct timeval tv = {.tv_sec = t, .tv_usec = 0};
    settimeofday(&tv, NULL);
    s_ctx.sntp_synced = true;
    ESP_LOGW(TAG, "演示模式：时间设为 %04d-%02d-%02d %02d:%02d（无 WiFi）",
             REMINDER_MOCK_YEAR, REMINDER_MOCK_MON, REMINDER_MOCK_DAY,
             REMINDER_MOCK_HOUR, REMINDER_MOCK_MIN);
#else
    /* ── 开机时间兜底：先用 NVS 里"上次联网时的时间"回填系统时钟 ──────────────
     * 设备首次必然配网成功，故 NVS 里一定存过时间。断网开机时若不回填，系统时间会
     * 停在 1970，poll_timer_callback 里闹钟/日历/天气全部失效（见 sntp_synced 判断）。
     * 回填后 RTC 从这个起点继续走，闹钟（只看时分）可正常准点触发。
     * 注意：只在系统时间明显无效（早于 2020 年）时才回填——若 RTC 里已有本次开机
     * SNTP 同步过的真实时间，绝不能用 NVS 旧值把它覆盖回去。
     * 回填得到的时间标记 time_from_nvs=true，日历据此暂停触发（日期可能已偏差）。 */
    {
        time_t sys_now = time(NULL);
        if (sys_now < REMINDER_TIME_VALID_EPOCH) /* 系统时钟还停在 1970 */
        {
            nvs_handle_t handle;
            if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK)
            {
                int64_t saved = 0;
                if (nvs_get_i64(handle, "last_time", &saved) == ESP_OK &&
                    saved > REMINDER_TIME_VALID_EPOCH)
                {
                    struct timeval tv = {.tv_sec = (time_t)saved, .tv_usec = 0};
                    settimeofday(&tv, NULL);
                    s_ctx.sntp_synced = true;  /* 有可用时间 → 放行闹钟等时间相关功能 */
                    s_ctx.time_from_nvs = true; /* 但标记为兜底 → 日历暂停触发 */

                    struct tm t;
                    time_t st = (time_t)saved;
                    localtime_r(&st, &t);
                    ESP_LOGW(TAG, "无网络，已用 NVS 兜底时间回填：%04d-%02d-%02d %02d:%02d"
                                  "（闹钟可用；日历因日期可能偏差暂停触发）",
                             t.tm_year + 1900, t.tm_mon + 1, t.tm_mday, t.tm_hour, t.tm_min);
                }
                nvs_close(handle);
            }
        }
    }

    ESP_LOGI(TAG, "初始化 SNTP 时间同步... 时区=%s", REMINDER_TZ);
    esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
    esp_sntp_setservername(0, "ntp.aliyun.com");
    esp_sntp_setservername(1, "pool.ntp.org");
    sntp_set_time_sync_notification_cb(sntp_sync_notification_cb);
    esp_sntp_init();
#endif
}

/* ═══════════════════════════════════════════════════════════════════
 * 5. 闹钟匹配与响铃控制
 * ═══════════════════════════════════════════════════════════════════ */
static bool alarm_should_trigger(const alarm_entry_t *alarm, const struct tm *now_tm)
{
    if (!alarm->enabled)
        return false;
    if (now_tm->tm_hour != alarm->hour || now_tm->tm_min != alarm->minute)
        return false;

    int wday = now_tm->tm_wday;
    switch (alarm->repeat)
    {
    case ALARM_REPEAT_ONCE:
    case ALARM_REPEAT_DAILY:
        return true;
    case ALARM_REPEAT_WEEKDAY:
        return (wday >= 1 && wday <= 5);
    case ALARM_REPEAT_WEEKEND:
        return (wday == 0 || wday == 6);
    case ALARM_REPEAT_CUSTOM:
        return (alarm->weekday_mask & (1 << wday)) != 0;
    default:
        return false;
    }
}

static void ring_timer_callback(void *arg)
{
    reminder_evt_t evt = {.type = REM_EVT_ALARM_RING_TICK};
    xQueueSend(s_ctx.evt_queue, &evt, 0);
}

/* ★【2026-08-25 新增】本次 trigger_cb 是否为"本轮响铃的第一次"。
 *
 * 仅在 alarm_ring_start() 调 trigger_cb 的那一小段内为 true，供 application.c
 * 判断"能否安全操作 UI"（第一次不持 s_ctx.mutex，后续 tick 持锁）。
 * 与 s_ctx.state 彻底解耦——详见 reminder.h 中 reminder_is_first_ring_callback()
 * 的说明：两个语义此前挤在同一个 state 上，导致"响铃画面已出、state 却还没置
 * RINGING"的空窗，头部触摸因此进不了关闹钟分支。 */
static volatile bool s_first_ring_cb = false;

bool reminder_is_first_ring_callback(void)
{
    return s_first_ring_cb;
}

static void alarm_ring_start(uint8_t alarm_id, const char *message)
{
    s_ctx.ringing_alarm_id = alarm_id;
    s_ctx.ring_count = 0;

    ESP_LOGW(TAG, "闹钟 #%d 开始响铃: %s", alarm_id, message);

    /* ★★【2026-08-25 修复：state 置 RINGING 必须在 trigger_cb 之【后】】★★
     *
     * 【修的问题】"闹钟响了但屏幕不切到闹钟页，只有空闲 GIF + 舵机动一下"，偶发。
     *
     * 【根因链】application.c 的 on_reminder_trigger() 用 s_alarm_ring_page_shown
     *   这个门闩保证"只有第一次响铃（不持锁那次）才切页"（那是死锁修复，不能去掉），
     *   它的兜底复位条件写的是：
     *       if (reminder_get_state() != REMINDER_STATE_RINGING)
     *           s_alarm_ring_page_shown = false;
     *   而本函数原先【第一行】就把 state 置成了 RINGING，紧接着才回调 →
     *   第一次响铃回调进来时 state 已是 RINGING，**复位条件永远不成立**。
     *   与此同时 alarm_ring_stop() 走"响满次数自动关闭"这条路时**不发任何回调**，
     *   于是标志一旦置 true 就再也没人清：
     *     · 若上一次闹钟是被用户触摸关掉的（走 REM_EVT_ALARM_DISMISS，有回调且
     *       此时 state 已 IDLE）→ 标志被清 → 下次能正常切页；
     *     · 若上一次闹钟是响满自动结束的 → 标志残留 true → **下次闹钟不切页**。
     *   这正是"有时候切、有时候不切"的偶发根源（中间若恰好有倒计时/日历回调
     *   进来也会顺带清掉标志，让现象更加随机）。
     *
     * 【改法】把置位挪到 trigger_cb 之后。这样第一次响铃回调进来时 state 仍是
     *   IDLE/NOTIFYING，复位条件成立 → 清标志 → 正常切页；而后续的
     *   REM_EVT_ALARM_RING_TICK 回调进来时 state 已是 RINGING，门闩照常拦住，
     *   死锁修复的语义完全不变。
     *
     * 【为何安全】本函数由 reminder_task 的 REM_EVT_ALARM_TRIGGER 分支调用，
     *   全程不持 s_ctx.mutex；trigger_cb 期间 state 短暂不是 RINGING，只影响
     *   两处判断且均无害：poll_timer_callback 的"响铃中跳过"（同一分钟已由
     *   last_alarm_check_min 拦住，不会重复触发）、触摸关闹钟分支（此刻画面刚
     *   切出来，用户来不及触摸）。下面 FIX-4 的错误回退路径不受影响。 */
    /* ★★【2026-08-25 二次修正：state 恢复为「回调之前就置位」】★★
     *
     * 【上一轮为什么把它挪到回调之后】当时 application.c 借用
     *   `reminder_get_state() != RINGING` 来判断"是不是本轮第一次回调"，
     *   若先置 RINGING 那个判据就永远不成立，导致门闩永久卡住、闹钟偶发不切页。
     *
     * 【但那样修出了新问题】state 置位被推迟到 trigger_cb 之后，而 trigger_cb 里
     *   要跑完 ui_show_alarm_ringing()（取 LVGL 锁最多 500ms + 整页渲染）。
     *   这段时间【响铃画面已经显示出来了，state 却还不是 RINGING】，于是
     *   ui_port 的「响铃中任意触摸关闭闹钟」分支进不去 —— 实测现象＝
     *   **闹钟结束界面上头部触摸完全无反应**（耳朵走别的路径故仍可用）。
     *   我当时在注释里写"此刻画面刚切出来，用户来不及触摸"，这个判断是错的：
     *   画面一出来正是用户要摸的时候。
     *
     * 【现在的正解】把"是否第一次回调"拆成独立标志 s_first_ring_cb（见上方），
     *   state 则恢复成【回调之前】就置位。两个语义各用各的标志，互不牵制：
     *     · state=RINGING 立刻生效 → 触摸关闹钟分支全程可用；
     *     · s_first_ring_cb 只在本次回调期间为真 → 门闩判断依旧准确。 */
    s_ctx.state = REMINDER_STATE_RINGING;

    s_first_ring_cb = true; /* 仅本次回调期间为真，供 application 判断可否碰 UI */
    if (s_ctx.trigger_cb)
        s_ctx.trigger_cb(REMINDER_TYPE_ALARM, message, true);
    s_first_ring_cb = false;

    s_ctx.ring_count++;

    if (s_ctx.ring_timer == NULL)
    {
        esp_timer_create_args_t args = {
            .callback = ring_timer_callback,
            .name = "alarm_ring",
        };
        esp_err_t err = esp_timer_create(&args, &s_ctx.ring_timer);
        if (err != ESP_OK)
        {
            /* [FIX-4] 定时器创建失败，回退到 IDLE 状态，避免永远卡在 RINGING */
            ESP_LOGE(TAG, "响铃定时器创建失败: %s，回退到 IDLE", esp_err_to_name(err));
            s_ctx.state = REMINDER_STATE_IDLE;
            return;
        }
    }
    esp_timer_start_periodic(s_ctx.ring_timer, ALARM_RING_INTERVAL_MS * 1000);
}

static void alarm_ring_stop(void)
{
    if (s_ctx.state != REMINDER_STATE_RINGING)
        return;

    if (s_ctx.ring_timer)
        esp_timer_stop(s_ctx.ring_timer);

    /* 一次性闹钟触发后自动禁用 */
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (s_ctx.ringing_alarm_id < s_ctx.alarm_count &&
        s_ctx.alarms[s_ctx.ringing_alarm_id].repeat == ALARM_REPEAT_ONCE)
    {
        s_ctx.alarms[s_ctx.ringing_alarm_id].enabled = false;
        nvs_save_alarms();
        ESP_LOGI(TAG, "一次性闹钟 #%d 已自动禁用", s_ctx.ringing_alarm_id);
    }
    xSemaphoreGive(s_ctx.mutex);

    s_ctx.state = REMINDER_STATE_IDLE;
    s_ctx.ring_count = 0;
    ESP_LOGI(TAG, "闹钟响铃已停止");
}

/* ═══════════════════════════════════════════════════════════════════
 * 6. 天气获取
 * ═══════════════════════════════════════════════════════════════════ */
// typedef struct
// {
//     char *buf;
//     size_t len;
//     size_t capacity;
// } http_response_t;

// static esp_err_t http_event_handler(esp_http_client_event_t *evt)
// {
//     http_response_t *resp = (http_response_t *)evt->user_data;
//     if (evt->event_id == HTTP_EVENT_ON_DATA && resp != NULL)
//     {
//         if (resp->len + evt->data_len < resp->capacity)
//         {
//             memcpy(resp->buf + resp->len, evt->data, evt->data_len);
//             resp->len += evt->data_len;
//             resp->buf[resp->len] = '\0';
//         }
//     }
//     return ESP_OK;
// }

/* 前向声明：url_encode 定义在本文件后半部分 */
static void url_encode(const char *src, char *dst, size_t dst_size);

/* --- reminder.c 内部 --- */

static esp_err_t weather_fetch_and_notify(void)
{
#if WEATHER_PROVIDER == 0
#define WEATHER_PROVIDER_NAME "和风"
#else
#define WEATHER_PROVIDER_NAME "心知"
#endif
    ESP_LOGI(TAG, "使用新组件获取天气(%s): %s",
             WEATHER_PROVIDER_NAME, s_ctx.weather_cfg.city_name);

    // 注意：这里调用的 weather_config_t 是新组件定义的！
    // api_key / api_host / type 全部由 reminder.h 的 WEATHER_PROVIDER 宏决定，
    // 改宏即可在心知/和风之间切换，本函数无需改动。
    weather_config_t config = {
        .api_key = WEATHER_API_KEY,          // 使用我们在 reminder.h 定义的宏
        .api_host = WEATHER_API_HOST,        // 和风=专属 host，心知=NULL
        .city = s_ctx.weather_cfg.city_name, // 使用我们本地存储的城市名
#if WEATHER_PROVIDER == 0
        .type = WEATHER_HEFENG // 和风天气
#else
        .type = WEATHER_XINZHI // 心知天气
#endif
    };

    weather_info_t *info = weather_get(&config);

    if (info == NULL)
    {
        ESP_LOGE(TAG, "天气获取失败");
        return ESP_FAIL;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    // 更新温度（取整，UI 大号显示"24°"无需小数）
    snprintf(s_ctx.weather_data.temp, sizeof(s_ctx.weather_data.temp), "%.0f", info->temperature);
    // 更新描述
    if (info->weather)
    {
        strncpy(s_ctx.weather_data.text, info->weather, sizeof(s_ctx.weather_data.text) - 1);
        s_ctx.weather_data.text[sizeof(s_ctx.weather_data.text) - 1] = '\0';
    }
    // 更新湿度（整数百分比，如"91"）
    snprintf(s_ctx.weather_data.humidity, sizeof(s_ctx.weather_data.humidity), "%.0f", info->humidity);
    // 更新降水量（保留 1 位小数，单位 mm；不下雨时和风返回 0.0 属正常）
    snprintf(s_ctx.weather_data.precip, sizeof(s_ctx.weather_data.precip), "%.1f", info->precip);
    // 更新体感温度（取整，如"26"）
    snprintf(s_ctx.weather_data.feels, sizeof(s_ctx.weather_data.feels), "%.0f", info->feels_like);
    // 更新风向+风力（和风 wind_dir 已含"风"字，如"西北风"；拼成"西北风2级"）
    if (info->wind_dir && info->wind_scale)
        snprintf(s_ctx.weather_data.wind, sizeof(s_ctx.weather_data.wind), "%s%s级", info->wind_dir, info->wind_scale);
    else if (info->wind_dir)
        snprintf(s_ctx.weather_data.wind, sizeof(s_ctx.weather_data.wind), "%s", info->wind_dir);
    else
        s_ctx.weather_data.wind[0] = '\0';

    s_ctx.weather_data.valid = true;
    xSemaphoreGive(s_ctx.mutex);

    /* ★ 缓存本次结果到 NVS：断网开机时读出来直接显示，避免天气页空着
     *   （"等待天气数据 / 湿度—% / 降水量—mm"）。每次成功获取都覆盖，保持最新。 */
    nvs_save_weather_data();

    weather_info_free(info); // 必须释放
    return ESP_OK;
}
// static esp_err_t weather_fetch_and_notify(void)
// {
//     ESP_LOGI(TAG, "获取天气信息: %s (%s)",
//              s_ctx.weather_cfg.city_name, s_ctx.weather_cfg.city_code);

//     /* 从堆分配响应缓冲区，避免撑爆任务栈 */
//     char *response_buf = malloc_zeroed(2048);
//     if (response_buf == NULL)
//     {
//         ESP_LOGE(TAG, "天气缓冲区分配失败");
//         return ESP_FAIL;
//     }

//     http_response_t resp = {
//         .buf = response_buf,
//         .len = 0,
//         .capacity = 2048,
//     };

//     char url[256];
//     char encoded_city[64];
//     url_encode(s_ctx.weather_cfg.city_code, encoded_city, sizeof(encoded_city));

// #if WEATHER_API_TYPE == 0
//     /* 和风天气：location 在前，key 在后 */
//     snprintf(url, sizeof(url), WEATHER_API_URL_FMT,
//              encoded_city, WEATHER_API_KEY);
// #else
//     /* 心知天气：key 在前，location 在后 */
//     snprintf(url, sizeof(url), WEATHER_API_URL_FMT,
//              WEATHER_API_KEY, encoded_city);
// #endif

//     esp_http_client_config_t config = {
//         .url = url,
//         .event_handler = http_event_handler,
//         .user_data = &resp,
//         .timeout_ms = WEATHER_FETCH_TIMEOUT_MS,
//         // .skip_cert_common_name_check = true,
//         .crt_bundle_attach = esp_crt_bundle_attach,
//         // .skip_server_cert_verify = true, /* 跳过证书验证，专属域名 HTTPS 需要 */
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     if (client == NULL)
//     {
//         ESP_LOGE(TAG, "HTTP 客户端初始化失败");
//         heap_caps_free(response_buf);
//         return ESP_FAIL;
//     }

//     esp_err_t err = esp_http_client_perform(client);
//     int status_code = esp_http_client_get_status_code(client);
//     esp_http_client_cleanup(client);

//     if (err != ESP_OK || (status_code != 200 && status_code != 0))
//     {
//         ESP_LOGW(TAG, "天气 API 请求失败: err=%s, status=%d（使用占位消息）",
//                  esp_err_to_name(err), status_code);
//     }
//     else
//     {
//         ESP_LOGI(TAG, "天气 API 响应 (%d字节)", (int)resp.len);

//         cJSON *root = cJSON_Parse(resp.buf);
//         if (root)
//         {
// #if WEATHER_API_TYPE == 0
//             /* ── 和风天气 JSON: { "now": { "text", "temp", "humidity" } } ── */
//             cJSON *now_obj = cJSON_GetObjectItem(root, "now");
//             if (now_obj)
//             {
//                 const char *text = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "text"));
//                 const char *temp = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "temp"));
//                 const char *humidity = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "humidity"));
// #else
//             /* ── 心知天气 JSON: { "results": [{ "now": { "text", "temperature", "humidity" } }] } ── */
//             cJSON *results = cJSON_GetObjectItem(root, "results");
//             cJSON *first_result = (results && cJSON_IsArray(results)) ? cJSON_GetArrayItem(results, 0) : NULL;
//             cJSON *now_obj = first_result ? cJSON_GetObjectItem(first_result, "now") : NULL;
//             if (now_obj)
//             {
//                 const char *text = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "text"));
//                 const char *temp = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "temperature"));
//                 const char *humidity = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "humidity"));
// #endif
//                 if (text && temp)
//                 {
//                     strncpy(s_ctx.weather_data.temp, temp, sizeof(s_ctx.weather_data.temp) - 1);
//                     s_ctx.weather_data.temp[sizeof(s_ctx.weather_data.temp) - 1] = '\0';
//                     strncpy(s_ctx.weather_data.text, text, sizeof(s_ctx.weather_data.text) - 1);
//                     s_ctx.weather_data.text[sizeof(s_ctx.weather_data.text) - 1] = '\0';
//                     if (humidity)
//                     {
//                         strncpy(s_ctx.weather_data.humidity, humidity, sizeof(s_ctx.weather_data.humidity) - 1);
//                         s_ctx.weather_data.humidity[sizeof(s_ctx.weather_data.humidity) - 1] = '\0';
//                     }
//                     s_ctx.weather_data.valid = true;
//                     ESP_LOGI(TAG, "天气数据已更新: %s %s°C %s%%", text, temp, humidity ? humidity : "?");
//                 }
//                 else
//                 {
//                     ESP_LOGW(TAG, "天气数据解析异常");
//                 }
//             }
//             cJSON_Delete(root);
//         }
//         else
//         {
//             ESP_LOGW(TAG, "天气 JSON 解析失败");
//         }
//     }

//     heap_caps_free(response_buf);
//     return ESP_OK;
// }

/* ═══════════════════════════════════════════════════════════════════
 * 7. 轮询定时器回调
 * ═══════════════════════════════════════════════════════════════════ */
static void poll_timer_callback(void *arg)
{
    reminder_evt_t evt = {0};

    /* ── 倒计时检查（不依赖 SNTP） ── */
    {
        int64_t now_us = esp_timer_get_time();

        /* [FIX-3] 检查互斥锁获取结果，未持锁时跳过本轮检查 */
        if (xSemaphoreTake(s_ctx.mutex, 0) == pdTRUE)
        {
            for (uint8_t i = 0; i < REMINDER_MAX_TIMERS; i++)
            {
                timer_entry_t *t = &s_ctx.timers[i];
                if (!t->active)
                    continue;

                int64_t elapsed_sec = (now_us - t->start_time_us) / 1000000;
                if (elapsed_sec >= (int64_t)t->duration_sec)
                {
                    /* ★★【2026-08-25 新增：对话优先，对话中不触发倒计时】★★
                     *
                     * 【需求】对话进行中屏幕只显示对话 GIF，不允许任何提醒抢屏或震动。
                     *
                     * 【★ 为什么屏蔽点必须放在这里，而不是放到事件处理层 ★】
                     *   这是本改动最关键的一点。下面那两句是【不可逆】的：
                     *     t->active = false;              ← 倒计时被标记为"已用掉"
                     *     xQueueSend(...REM_EVT_TIMER_EXPIRE...);
                     *   若改在 reminder_task 的 REM_EVT_TIMER_EXPIRE 分支、或在
                     *   application.c 的 on_reminder_trigger() 里丢弃，此刻 active
                     *   早已被清成 false —— 这次倒计时就【彻底消失、永不补发】，
                     *   用户专注一轮番茄钟会被静默作废且毫不知情。
                     *
                     *   放在这里 continue 则完全不同：不投递事件、也【不推进任何状态】，
                     *   t->active 保持 true，下一拍（1 秒后）本判断依然成立 →
                     *   对话一结束就立即正常触发。等于零成本实现了"延后到对话结束"，
                     *   代码比"丢弃"还少一行。
                     *
                     * 【画面语义无违和】到期画面文案是「倒计时结束!」，本就是过去式，
                     *   晚几分钟看到不会让人误以为计时不准。 */
                    /* ★★【2026-08-25 改：提醒优先级最高，到点直接掐断对话】★★
                     *
                     * 【需求变更】此前是"对话优先"——对话中扣住提醒不发（continue 等待）。
                     *   现改为【提醒优先】：到点就把正在进行的对话立即关掉（含正在播的 TTS），
                     *   然后照常走震动 + 切到期画面的正常流程。
                     *
                     * 【为什么不再 continue 等待】旧做法有个躲不掉的代价：闹钟只匹配
                     *   「时:分」那 60 秒，对话一旦超过 1 分钟，这次闹钟就【永久丢失】。
                     *   改为打断对话后，提醒一定会准点发生，该缺陷随之消失。
                     *
                     * 【为什么可以在这里调】session_interrupt_for_reminder() 内部只做
                     *   两次 xQueueSend，不碰 flash/NVS/LVGL、不阻塞，因此在 PSRAM 栈的
                     *   reminder_task 上调用是安全的（详见该函数注释与本文件
                     *   nvs_save_weather_data 处的 PSRAM 栈铁律）。
                     *
                     * 【不再 continue】打断是异步的（投队列后会话任务才真正收尾），
                     *   但本次到期【照常投递】，不必等会话关完 —— 震动与切页各走各的路径，
                     *   会话收尾期间 UI 已经可以显示提醒画面，不存在互斥。 */
                    if (session_get_state() != SESSION_IDLE)
                        session_interrupt_for_reminder();

                    /* ★★【2026-08-25 新增：闹钟响铃期间，整个倒计时提醒排队等候】★★
                     *
                     * 【需求】闹钟与倒计时撞在一起时，要先把闹钟响完，再完整地做一次
                     *   倒计时提醒（震动 + 到期画面），而不是两者混在一起。
                     *
                     * 【原来错在哪】此前是在 UI 层（ui_show_countdown_expired）拦画面，
                     *   那是【丢弃】不是【推迟】：事件已经投递、t->active 已被清成 false，
                     *   于是 ①震动照常长震 3 秒，与闹钟震动混叠；②画面被拦掉后再也不补，
                     *   这一次番茄钟等于白算；③s_cd.state 还停在 EXPIRED，用户下次主动
                     *   进倒计时页会先看到"结束界面"、几秒后才跳回设定界面（即实测第 9 条）。
                     *
                     * 【正确做法】守卫上移到这里，与上面"对话优先"完全同一手法：
                     *   不投递、不推进任何状态，t->active 保持 true，下一拍重判仍成立。
                     *   闹钟一停（alarm_ring_stop 把 state 置回 IDLE），下一拍（≤1 秒）
                     *   倒计时就完整触发：3 秒长震 + 切到期画面，一样都不少。
                     *
                     * 【时序代价】倒计时提醒会比真正到期晚十几秒（等闹钟响满 4 次）。
                     *   到期画面文案是「倒计时结束!」，过去式，不会让人误以为计时不准。 */
                    if (s_ctx.state == REMINDER_STATE_RINGING)
                        continue;

                    /* ★★【2026-08-25 新增：深度待机中先亮屏，本拍不投递】★★
                     *
                     * 【为什么不能"投递了再让 UI 去唤醒"】曾经这么改过，直接崩：
                     *   投递后 reminder_task 会走 on_reminder_trigger →
                     *   ui_show_countdown_expired → standby_wake → 读 NVS 恢复音量 →
                     *   spi_flash_disable_interrupts_caches_and_other_cpu() 关 cache。
                     *   而【reminder_task 的栈在 PSRAM】（见本文件 nvs_save_weather_data
                     *   处那条铁律），关 cache 后 PSRAM 不可访问、栈失联，IDF 断言 abort：
                     *     assert failed: esp_task_stack_is_sane_cache_disabled()
                     *                    @ cache_utils.c:152
                     *   栈指针 0x3c8c0d10 落在 0x3C 段（PSRAM）即铁证。
                     *
                     * 【正确做法】本任务只调 standby_request_wake()——它只做一次原子
                     *   置位，不碰 flash/NVS/LVGL，PSRAM 栈上完全安全；真正的唤醒由
                     *   standby_task（栈在内部 SRAM）在下一拍执行。
                     *
                     * 【为什么 continue 而不是投递】与上面的对话守卫同一手法：不投递、
                     *   也不推进 t->active，下一拍本判断依然成立。等屏幕亮起来
                     *   （唤醒转场约 2 秒）之后再正常投递，此时 ui_show_countdown_expired
                     *   走的就是"不在待机"的正常路径，既不会踩 PSRAM 栈，也不会留下
                     *   "待机时钟 + 功能页"的叠加态。代价是到期提醒延后约 2~3 秒。 */
                    if (standby_is_deep_active())
                    {
                        standby_request_wake(STANDBY_WAKE_SRC_COUNTDOWN_EXPIRE);
                        continue;
                    }

                    evt.type = REM_EVT_TIMER_EXPIRE;
                    evt.id = i;
                    strncpy(evt.message, t->message, REMINDER_MSG_MAX_LEN - 1);
                    evt.message[REMINDER_MSG_MAX_LEN - 1] = '\0'; /* [FIX-10] */
                    t->active = false;
                    xQueueSend(s_ctx.evt_queue, &evt, 0);
                }
            }
            xSemaphoreGive(s_ctx.mutex);
        }
    }

    if (!s_ctx.sntp_synced)
        return;

    if (s_ctx.state == REMINDER_STATE_RINGING)
        return;

    time_t now = time(NULL);
    struct tm now_tm;
    localtime_r(&now, &now_tm);
    int current_min = now_tm.tm_hour * 60 + now_tm.tm_min;

    /* ── 闹钟检查（最高优先级） ── */
    if (current_min != s_ctx.last_alarm_check_min)
    {
        /* [FIX-3] */
        if (xSemaphoreTake(s_ctx.mutex, 0) == pdTRUE)
        {
            for (uint8_t i = 0; i < s_ctx.alarm_count; i++)
            {
                if (alarm_should_trigger(&s_ctx.alarms[i], &now_tm))
                {
                    /* ★★【2026-08-25 改：提醒优先级最高，到点直接掐断对话】★★
                     * 与上面倒计时段完全同一原则，理由见那里的完整注释。
                     *
                     * ⭐【本改动顺带根治了"闹钟丢失"】旧做法是对话中 continue 等待，
                     *   而闹钟只匹配「时:分」那 60 秒（见 alarm_should_trigger），
                     *   对话一旦超过 1 分钟这次闹钟就【永久丢失】——那是"对话优先"
                     *   决策下无法绕开的代价。现在改为打断对话，闹钟必定准点响，
                     *   该缺陷自然消失。 */
                    if (session_get_state() != SESSION_IDLE)
                        session_interrupt_for_reminder();

                    /* 深度待机中：同倒计时段——只发异步唤醒请求，本拍不投递、
                     * 不推进 last_alarm_check_min，等屏幕亮起后下一拍正常响铃。
                     * ⚠️ 绝不可在本任务里直接调 standby_wake()：reminder_task 栈在
                     * PSRAM，而它内部读 NVS 要关 flash cache → 栈失联 → 断言 abort。
                     * 完整说明见上面倒计时段的对应注释。
                     * 【时限】唤醒约 2 秒，通常仍落在同一分钟内，不影响本次响铃； */
                    if (standby_is_deep_active())
                    {
                        standby_request_wake(STANDBY_WAKE_SRC_ALARM_RING);
                        continue;
                    }

                    evt.type = REM_EVT_ALARM_TRIGGER;
                    evt.id = i;
                    strncpy(evt.message, s_ctx.alarms[i].message, REMINDER_MSG_MAX_LEN - 1);
                    evt.message[REMINDER_MSG_MAX_LEN - 1] = '\0'; /* [FIX-10] */
                    xQueueSend(s_ctx.evt_queue, &evt, 0);
                    s_ctx.last_alarm_check_min = current_min;
                    xSemaphoreGive(s_ctx.mutex);
                    return;
                }
            }
            xSemaphoreGive(s_ctx.mutex);
        }
    }

    /* ── 日历事件检查 ──
     * ★ 兜底时间下暂停触发：日历匹配的是"年/月/日+时分"（下方 cal->year/month/day），
     *   而 NVS 兜底时间只是"上次联网时刻 + 本次开机后经过的时长"，掉电期间流逝的时间
     *   完全丢失——断电一晚日期可能还对，断电三天日期就差三天，日历会在错误的日子误报
     *   或彻底错过。闹钟只看时分（每天循环），兜底下依然准点，故不受此限制。
     *   SNTP 一旦同步成功，time_from_nvs 置回 false，日历自动恢复触发。 */
    if (s_ctx.time_from_nvs)
    {
        /* 仍推进检查时间戳，避免恢复联网后一次性补触发一大批过期事件 */
        s_ctx.last_cal_check_min = current_min;
    }
    else if (current_min != s_ctx.last_cal_check_min)
    {
        /* [FIX-3] */
        if (xSemaphoreTake(s_ctx.mutex, 0) == pdTRUE)
        {
            for (uint8_t i = 0; i < s_ctx.calendar_count; i++)
            {
                calendar_entry_t *cal = &s_ctx.calendars[i];
                if (!cal->enabled)
                    continue;
                if (cal->year == (uint16_t)(now_tm.tm_year + 1900) &&
                    cal->month == (uint8_t)(now_tm.tm_mon + 1) &&
                    cal->day == (uint8_t)now_tm.tm_mday &&
                    cal->hour == (uint8_t)now_tm.tm_hour &&
                    cal->minute == (uint8_t)now_tm.tm_min)
                {
                    evt.type = REM_EVT_CALENDAR_TRIGGER;
                    evt.id = i;
                    strncpy(evt.message, cal->message, REMINDER_MSG_MAX_LEN - 1);
                    evt.message[REMINDER_MSG_MAX_LEN - 1] = '\0'; /* [FIX-10] */
                    xQueueSend(s_ctx.evt_queue, &evt, 0);
                    cal->enabled = false;
                    s_ctx.last_cal_check_min = current_min;
                }
            }
            xSemaphoreGive(s_ctx.mutex);
        }
    }

    /* ── 天气数据拉取检查（4个固定时间点：07:30/11:30/15:30/19:30，无语音播报）──
     * 轮询精度 1 分钟，命中时间点且本轮未拉取则触发；上电后由
     * reminder_weather_fetch_now() 立即拉取一次，不依赖此处。 */
    if (s_ctx.weather_cfg.schedule != WEATHER_SCHEDULE_DISABLED &&
        s_ctx.state == REMINDER_STATE_IDLE)
    {
        static const int k_fetch_mins[] = {
            WEATHER_FETCH_MIN_0, /* 07:30 */
            WEATHER_FETCH_MIN_1, /* 11:30 */
            WEATHER_FETCH_MIN_2, /* 15:30 */
            WEATHER_FETCH_MIN_3, /* 19:30 */
        };
        int now_min = now_tm.tm_hour * 60 + now_tm.tm_min;
        for (int i = 0; i < 4; i++)
        {
            /* 当前分钟命中某个时间点，且该时间点今天还未拉取过 */
            if (now_min == k_fetch_mins[i] &&
                s_ctx.last_weather_fetch_min != k_fetch_mins[i])
            {
                evt.type = REM_EVT_WEATHER_FETCH;
                xQueueSend(s_ctx.evt_queue, &evt, 0);
                s_ctx.last_weather_fetch_min = k_fetch_mins[i];
                break;
            }
        }
        /* 跨天重置（午夜 00:00 时清除上次记录，使次日照常触发） */
        if (now_min == 0)
            s_ctx.last_weather_fetch_min = -1;
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 8. 提醒任务
 * ═══════════════════════════════════════════════════════════════════ */
static void reminder_task(void *arg)
{
    reminder_evt_t evt;
    ESP_LOGI(TAG, "提醒任务启动");
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余

    while (1)
    {
        if (xQueueReceive(s_ctx.evt_queue, &evt, portMAX_DELAY) != pdTRUE)
            continue;

        switch (evt.type)
        {
        case REM_EVT_ALARM_TRIGGER:
            ESP_LOGW(TAG, ">>> 闹钟 #%d 触发: %s <<<", evt.id, evt.message);
            alarm_ring_start(evt.id, evt.message);
            break;

        case REM_EVT_ALARM_RING_TICK:
            if (s_ctx.state != REMINDER_STATE_RINGING)
                break;

            s_ctx.ring_count++;

            if (s_ctx.ring_count >= ALARM_RING_MAX_COUNT)
            {
                ESP_LOGW(TAG, "闹钟 #%d 响铃超时（%d 次），自动关闭",
                         s_ctx.ringing_alarm_id, s_ctx.ring_count);
                alarm_ring_stop();
                break;
            }

            ESP_LOGI(TAG, "闹钟 #%d 第 %d 次响铃",
                     s_ctx.ringing_alarm_id, s_ctx.ring_count);
            if (s_ctx.trigger_cb)
            {
                xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
                if (s_ctx.ringing_alarm_id < s_ctx.alarm_count)
                    s_ctx.trigger_cb(REMINDER_TYPE_ALARM,
                                     s_ctx.alarms[s_ctx.ringing_alarm_id].message, true);
                xSemaphoreGive(s_ctx.mutex);
            }
            break;

        case REM_EVT_ALARM_DISMISS:
            ESP_LOGI(TAG, "用户关闭闹钟");
            alarm_ring_stop();
            if (s_ctx.trigger_cb)
                s_ctx.trigger_cb(REMINDER_TYPE_ALARM, "闹钟已关闭", false);
            break;

        case REM_EVT_TIMER_EXPIRE:
            ESP_LOGI(TAG, "倒计时 #%d 到期: %s", evt.id, evt.message);
            s_ctx.state = REMINDER_STATE_NOTIFYING;
            /* ⚠️【顺序：先切页，再震动】2026-08-10 修正。
             *
             * 原实现把 bsp_motor_pulse_level 放在 trigger_cb 之前，而该函数内含
             * vTaskDelay 整段阻塞（现为 TIMER_EXPIRE_VIBRATE_MS = 3 秒）。
             * trigger_cb 里要调 ui_show_countdown_expired() 切界面，于是变成
             * 「震完 3 秒才切页」—— 实测日志：到期 1081709 → 震动结束 1083729
             * → 画面才动，用户感知就是"震动早就停了界面才变"。
             *
             * 改为 trigger_cb 在前：切页几乎与到期同刻发生，随后震动 3 秒，
             * 画面与震动同时进行（需求：切页可以比震动早，绝不能等震动结束）。
             * ui_show_countdown_expired() 内部只等最多 100ms 的 LVGL 锁，
             * 不会明显推迟震动起始时刻。 */
            if (s_ctx.trigger_cb)
                s_ctx.trigger_cb(REMINDER_TYPE_TIMER, evt.message, false);

            /* 到期长震一次（时长/强度见 reminder.h 的 TIMER_EXPIRE_VIBRATE_* 宏）。
             * bsp_motor_pulse_level 内含 vTaskDelay 阻塞，本处在 reminder_task 自己的
             * 任务上下文，阻塞 3 秒不影响 LVGL 刷屏与音频链路。 */
            ESP_LOGI(TAG, "到期震动开始: 强度=%d 时长=%dms", TIMER_EXPIRE_VIBRATE_LEVEL, TIMER_EXPIRE_VIBRATE_MS);
            bsp_motor_pulse_level(TIMER_EXPIRE_VIBRATE_LEVEL, TIMER_EXPIRE_VIBRATE_MS);
            ESP_LOGI(TAG, "到期震动结束");
            s_ctx.state = REMINDER_STATE_IDLE;
            break;

        case REM_EVT_CALENDAR_TRIGGER:
            ESP_LOGI(TAG, "日历事件 #%d 触发: %s", evt.id, evt.message);
            s_ctx.state = REMINDER_STATE_NOTIFYING;
            if (s_ctx.trigger_cb)
                s_ctx.trigger_cb(REMINDER_TYPE_CALENDAR, evt.message, false);
            nvs_save_calendars(); /* 已改为异步队列 [FIX-5] */
            s_ctx.state = REMINDER_STATE_IDLE;
            break;

        case REM_EVT_WEATHER_FETCH:
            // ★ 离线模式拦截（本模块最关键的一处）：weather_fetch_and_notify() 是
            //   **同步阻塞** HTTP。断网时它要走完 DNS 解析失败 → connect 超时的完整
            //   链路（数秒），期间整个 reminder 任务被卡住，闹钟/倒计时/日历的到期
            //   判定全部被拖延 → 本地提醒功能受连累。离线时直接跳过，保住本地功能。
            if (bsp_wifi_is_offline_mode())
            {
                ESP_LOGW(TAG, "离线模式，跳过天气播报（本地闹钟/日历不受影响）");
                break;
            }
            ESP_LOGI(TAG, "执行天气播报");
            s_ctx.state = REMINDER_STATE_NOTIFYING;
            weather_fetch_and_notify();
            s_ctx.state = REMINDER_STATE_IDLE;
            break;

        /* [FIX-7] 安全关闭：收到 SHUTDOWN 信号后自删除 */
        case REM_EVT_SHUTDOWN:
            ESP_LOGI(TAG, "提醒任务收到关闭信号，退出");
            s_ctx.task_handle = NULL;
            vTaskDelete(NULL);
            return; /* 不可达，防御性写法 */

        default:
            ESP_LOGW(TAG, "未知事件类型: %d", evt.type);
            break;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 9. 系统生命周期
 * ═══════════════════════════════════════════════════════════════════ */
esp_err_t reminder_init(reminder_trigger_cb_t cb)
{
    if (s_ctx.initialized)
    {
        ESP_LOGW(TAG, "提醒系统已初始化，跳过");
        return ESP_OK;
    }

    /* [FIX-8] 清理上次 init 失败遗留的资源（防二次调用泄漏） */
    if (s_ctx.mutex)
    {
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
    }
    if (s_ctx.evt_queue)
    {
        vQueueDelete(s_ctx.evt_queue);
        s_ctx.evt_queue = NULL;
    }
    if (s_save_queue)
    {
        vQueueDelete(s_save_queue);
        s_save_queue = NULL;
    }

    ESP_LOGI(TAG, "初始化提醒系统...");
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.trigger_cb = cb;
    s_ctx.last_alarm_check_min = -1;
    s_ctx.last_cal_check_min = -1;

    s_ctx.mutex = xSemaphoreCreateMutex();
    if (s_ctx.mutex == NULL)
    {
        ESP_LOGE(TAG, "互斥锁创建失败");
        return ESP_FAIL;
    }

    s_ctx.evt_queue = xQueueCreate(10, sizeof(reminder_evt_t));
    if (s_ctx.evt_queue == NULL)
    {
        ESP_LOGE(TAG, "事件队列创建失败");
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
        return ESP_FAIL;
    }

    s_save_queue = xQueueCreate(4, sizeof(uint8_t));
    if (s_save_queue == NULL)
    {
        ESP_LOGE(TAG, "NVS 保存队列创建失败");
        vQueueDelete(s_ctx.evt_queue);
        s_ctx.evt_queue = NULL;
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
        return ESP_FAIL;
    }
    {
        BaseType_t r = xTaskCreateWithCaps(nvs_save_task, "nvs_save",
                                           3072, NULL, 1, NULL,
                                           MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (r != pdPASS)
        {
            ESP_LOGE(TAG, "NVS 保存任务创建失败");
            vQueueDelete(s_save_queue);
            s_save_queue = NULL;
            vQueueDelete(s_ctx.evt_queue);
            s_ctx.evt_queue = NULL;
            vSemaphoreDelete(s_ctx.mutex);
            s_ctx.mutex = NULL;
            return ESP_FAIL;
        }
        PRINT_TASK_CREATED(TAG, "nvs_save", 3072, 1); // 栈在内部SRAM
    }

    nvs_load_alarms();
    nvs_load_calendars();
    nvs_load_weather_config();
    nvs_load_weather_data(); /* 载入上次天气缓存，断网时天气页不再空白 */

    sntp_time_sync_init();

    /* 大 buffer 已改为堆分配，栈恢复为 8KB */
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        reminder_task, "reminder_task", 8192,
        NULL, 2, &s_ctx.task_handle, 0, MALLOC_CAP_SPIRAM);

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "提醒任务创建失败");
        /* [FIX-8] 完整清理：包括 NVS 任务 */
        uint8_t exit_cmd = NVS_SAVE_CMD_EXIT;
        xQueueSend(s_save_queue, &exit_cmd, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        vQueueDelete(s_save_queue);
        s_save_queue = NULL;
        vQueueDelete(s_ctx.evt_queue);
        s_ctx.evt_queue = NULL;
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
        return ESP_FAIL;
    }
    PRINT_TASK_CREATED(TAG, "reminder_task", 8192, 0); // 栈在PSRAM

    esp_timer_create_args_t timer_args = {
        .callback = poll_timer_callback,
        .name = "reminder_poll",
    };
    esp_err_t err = esp_timer_create(&timer_args, &s_ctx.poll_timer);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "轮询定时器创建失败: %s", esp_err_to_name(err));
        /* [FIX-8] 完整清理 */
        reminder_evt_t shutdown_evt = {.type = REM_EVT_SHUTDOWN};
        xQueueSend(s_ctx.evt_queue, &shutdown_evt, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(500));
        uint8_t exit_cmd = NVS_SAVE_CMD_EXIT;
        xQueueSend(s_save_queue, &exit_cmd, 0);
        vTaskDelay(pdMS_TO_TICKS(200));
        vQueueDelete(s_save_queue);
        s_save_queue = NULL;
        vQueueDelete(s_ctx.evt_queue);
        s_ctx.evt_queue = NULL;
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
        return err;
    }
    esp_timer_start_periodic(s_ctx.poll_timer, 1000000);

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "提醒系统初始化完成");

    /* ★ 离线拦截：下面两步都是同步阻塞 HTTP（IP 定位 + 天气拉取），断网时每一步都要
     *   走完 DNS 解析失败 → connect 超时的完整链路（数秒），把开机流程白白拖慢。
     *   离线时直接跳过：城市用 NVS 里的配置，天气用 NVS 里的上次缓存，均已在上面载入。 */
    if (bsp_wifi_is_offline_mode())
    {
        ESP_LOGW(TAG, "离线模式，跳过 IP 定位与天气拉取（使用 NVS 缓存数据）");
        return ESP_OK;
    }

    // 3.5 IP 自动定位城市（WiFi 已连接，获取天气城市代码）
    reminder_auto_locate_city();
    // 3.6 定位成功后立即拉取一次天气数据（否则要等到定时播报才有数据）
    reminder_weather_fetch_now();
    return ESP_OK;
}

// ─── reminder_on_offline_mode ────────────────────────────────────────────────

/**
 * @brief 通知提醒系统进入离线模式（接口说明见 reminder.h）
 */
void reminder_on_offline_mode(void)
{
    if (!s_ctx.initialized)
        return;

#ifndef REMINDER_MOCK_TIME
    /* 停掉 SNTP 轮询：射频已关，UDP 包发不出去，留着纯属周期性空转 */
    esp_sntp_stop();
#endif

    /* 关闭定时天气拉取：避免 07:30/11:30/15:30/19:30 四个时间点各白跑一次超时链路。
     * 天气页继续显示 NVS 里的上次缓存（nvs_load_weather_data 已在 init 载入）。 */
    s_ctx.weather_cfg.schedule = WEATHER_SCHEDULE_DISABLED;

    ESP_LOGW(TAG, "提醒系统已切到离线模式：SNTP 与定时天气拉取已停止；"
                  "闹钟/倒计时照常，天气显示上次缓存数据");
}

void reminder_deinit(void)
{
    if (!s_ctx.initialized)
        return;

    /* 1. 停止定时器 — 不再产生新事件 */
    if (s_ctx.poll_timer)
    {
        esp_timer_stop(s_ctx.poll_timer);
        esp_timer_delete(s_ctx.poll_timer);
        s_ctx.poll_timer = NULL;
    }
    if (s_ctx.ring_timer)
    {
        esp_timer_stop(s_ctx.ring_timer);
        esp_timer_delete(s_ctx.ring_timer);
        s_ctx.ring_timer = NULL;
    }

    /* 2. [FIX-9] 安全关闭 NVS 保存任务 */
    if (s_save_queue)
    {
        uint8_t exit_cmd = NVS_SAVE_CMD_EXIT;
        xQueueSend(s_save_queue, &exit_cmd, pdMS_TO_TICKS(100));
        vTaskDelay(pdMS_TO_TICKS(300)); /* 等待任务自行退出 */
    }

    /* 3. [FIX-9] 安全关闭 reminder_task：发送 SHUTDOWN 信号并等待 */
    if (s_ctx.task_handle && s_ctx.evt_queue)
    {
        reminder_evt_t shutdown_evt = {.type = REM_EVT_SHUTDOWN};
        xQueueSend(s_ctx.evt_queue, &shutdown_evt, pdMS_TO_TICKS(100));

        /* 等待任务自行退出（最多 5 秒，覆盖 HTTP 超时） */
        for (int i = 0; i < 50; i++)
        {
            if (s_ctx.task_handle == NULL)
                break; /* 任务已自删除，句柄被清零 */
            vTaskDelay(pdMS_TO_TICKS(100));
        }

        /* 如果仍未退出，强制删除 */
        if (s_ctx.task_handle != NULL)
        {
            ESP_LOGW(TAG, "提醒任务未响应关闭信号，强制删除");
            vTaskDelete(s_ctx.task_handle);
            s_ctx.task_handle = NULL;
        }
    }

    /* 4. 释放队列和互斥锁 */
    if (s_ctx.evt_queue)
    {
        vQueueDelete(s_ctx.evt_queue);
        s_ctx.evt_queue = NULL;
    }
    if (s_save_queue)
    {
        vQueueDelete(s_save_queue);
        s_save_queue = NULL;
    }
    if (s_ctx.mutex)
    {
        vSemaphoreDelete(s_ctx.mutex);
        s_ctx.mutex = NULL;
    }

#ifndef REMINDER_MOCK_TIME
    esp_sntp_stop();
#endif
    s_ctx.initialized = false;
    ESP_LOGI(TAG, "提醒系统已销毁");
}

reminder_state_t reminder_get_state(void)
{
    return s_ctx.state;
}

bool reminder_is_time_synced(void)
{
    return s_ctx.sntp_synced;
}

bool reminder_get_current_time(uint8_t *hour, uint8_t *minute, uint8_t *second)
{
    if (!s_ctx.sntp_synced)
    {
        if (hour)
            *hour = 0;
        if (minute)
            *minute = 0;
        if (second)
            *second = 0;
        return false;
    }

    time_t now = time(NULL);
    struct tm now_tm;
    localtime_r(&now, &now_tm);

    if (hour)
        *hour = (uint8_t)now_tm.tm_hour;
    if (minute)
        *minute = (uint8_t)now_tm.tm_min;
    if (second)
        *second = (uint8_t)now_tm.tm_sec;
    return true;
}

/* ═══════════════════════════════════════════════════════════════════
 * 10. 对外接口 — 闹钟
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 若新设闹钟的时间正是"当前这一分钟"，抑制它在本分钟内触发（顺延到明天）
 *
 * 【修的问题】"现在 1:01，我设一个 1:01 的闹钟，偶发不响"。
 *
 * 【原来的真实行为】触发扫描的外层闸是 `current_min != last_alarm_check_min`，
 *   而 last_alarm_check_min 只在【闹钟真正响过】时才推进，平时几乎恒为"开"。
 *   所以设一个当前分钟的闹钟，下一拍（≤1 秒）就会立刻响 —— 这既不是用户预期，
 *   又与"上一次刚在本分钟响过导致闸门关闭"的情形叠加，表现出忽响忽不响的偶发感。
 *
 * 【本函数做什么】保存闹钟时若其 时:分 恰好等于当前 时:分，就把
 *   last_alarm_check_min 推到当前分钟，关掉本分钟剩余时间的扫描闸门。
 *   于是本分钟内不会触发；跨到下一分钟后 tm_min 不再匹配，自然顺延到
 *   下一天的同一时刻 —— 与主流闹钟产品一致："设一个正在发生的时间＝明天这个点"。
 *
 * 【为何复用 last_alarm_check_min 而不新增变量】它本来就是"本分钟已处理过"的
 *   语义载体，此处正是同一含义，不必再引入一个会与它打架的新状态。
 *
 * @note 调用方必须已持有 s_ctx.mutex。
 */
static void alarm_suppress_if_current_minute(const alarm_entry_t *entry)
{
    if (entry == NULL || !s_ctx.sntp_synced)
        return;

    time_t now = time(NULL);
    struct tm now_tm;
    localtime_r(&now, &now_tm);

    if (now_tm.tm_hour == entry->hour && now_tm.tm_min == entry->minute)
    {
        s_ctx.last_alarm_check_min = now_tm.tm_hour * 60 + now_tm.tm_min;
        ESP_LOGI(TAG, "闹钟设为当前分钟(%02d:%02d)，本分钟内不触发，顺延到明天",
                 entry->hour, entry->minute);
    }
}

int reminder_alarm_add(const alarm_entry_t *entry)
{
    if (entry == NULL)
        return -1;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    if (s_ctx.alarm_count >= REMINDER_MAX_ALARMS)
    {
        ESP_LOGW(TAG, "闹钟已满 (%d)", REMINDER_MAX_ALARMS);
        xSemaphoreGive(s_ctx.mutex);
        return -1;
    }

    uint8_t new_id = s_ctx.alarm_count;
    s_ctx.alarms[new_id] = *entry;
    s_ctx.alarms[new_id].id = new_id;
    /* [FIX-12] 不强制 enabled=true，由调用方决定 */
    s_ctx.alarm_count++;

    alarm_suppress_if_current_minute(entry); /* 设为当前分钟 → 顺延到明天，不当场响 */

    nvs_save_alarms();
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "添加闹钟 #%d: %02d:%02d [enabled=%d]",
             new_id, entry->hour, entry->minute, entry->enabled);
    return new_id;
}

/**
 * @brief 添加一个闹钟
 *
 * @param entry 闹钟信息
 * @return uint8_t 闹钟 ID
 * @return ESP_ERR_NOT_FOUND 闹钟 ID 不存在
 * @return ESP_OK 成功
 */
esp_err_t reminder_alarm_delete(uint8_t alarm_id)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (alarm_id >= s_ctx.alarm_count)
    {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

#if REMINDER_MAX_ALARMS > 1
    /* 多闹钟：前移覆盖 */
    for (uint8_t i = alarm_id; i < s_ctx.alarm_count - 1; i++)
    {
        s_ctx.alarms[i] = s_ctx.alarms[i + 1];
        s_ctx.alarms[i].id = i;
    }
#else
    /* 单闹钟：直接清零，无数组移位 */
    memset(&s_ctx.alarms[0], 0, sizeof(alarm_entry_t));
#endif
    s_ctx.alarm_count--;
    nvs_save_alarms();
    xSemaphoreGive(s_ctx.mutex);
    ESP_LOGI(TAG, "删除闹钟 #%d", alarm_id);
    return ESP_OK;
}

esp_err_t reminder_alarm_set_enabled(uint8_t alarm_id, bool enabled)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (alarm_id >= s_ctx.alarm_count)
    {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }
    s_ctx.alarms[alarm_id].enabled = enabled;
    nvs_save_alarms();
    xSemaphoreGive(s_ctx.mutex);
    ESP_LOGI(TAG, "闹钟 #%d %s", alarm_id, enabled ? "启用" : "禁用");
    return ESP_OK;
}

esp_err_t reminder_alarm_update(uint8_t alarm_id, const alarm_entry_t *entry)
{
    if (entry == NULL)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (alarm_id >= s_ctx.alarm_count)
    {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }

    alarm_entry_t updated = *entry;
    updated.id = alarm_id;
    /* [FIX-12] 不强制 enabled=true，保留调用方传入的开关状态 */
    s_ctx.alarms[alarm_id] = updated;

    alarm_suppress_if_current_minute(&updated); /* 设为当前分钟 → 顺延到明天，不当场响 */

    nvs_save_alarms();
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "更新闹钟 #%d: %02d:%02d [enabled=%d]",
             alarm_id, updated.hour, updated.minute, updated.enabled);
    return ESP_OK;
}

void reminder_alarm_get_all(alarm_entry_t *out_list, uint8_t *out_count)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (out_list)
        memcpy(out_list, s_ctx.alarms, sizeof(alarm_entry_t) * s_ctx.alarm_count);
    if (out_count)
        *out_count = s_ctx.alarm_count;
    xSemaphoreGive(s_ctx.mutex);
}

esp_err_t reminder_alarm_dismiss(void)
{
    if (s_ctx.state != REMINDER_STATE_RINGING)
    {
        ESP_LOGW(TAG, "当前无响铃闹钟，忽略关闭指令");
        return ESP_ERR_NOT_FOUND;
    }
    reminder_evt_t evt = {.type = REM_EVT_ALARM_DISMISS};
    xQueueSend(s_ctx.evt_queue, &evt, pdMS_TO_TICKS(100));
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════
 * 11. 对外接口 — 倒计时
 * ═══════════════════════════════════════════════════════════════════ */
int reminder_timer_start(uint32_t duration_sec, const char *message)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    int slot = -1;
    for (uint8_t i = 0; i < REMINDER_MAX_TIMERS; i++)
    {
        if (!s_ctx.timers[i].active)
        {
            slot = i;
            break;
        }
    }
    if (slot < 0)
    {
        ESP_LOGW(TAG, "倒计时已满 (%d)", REMINDER_MAX_TIMERS);
        xSemaphoreGive(s_ctx.mutex);
        return -1;
    }

    s_ctx.timers[slot].id = (uint8_t)slot;
    s_ctx.timers[slot].active = true;
    s_ctx.timers[slot].duration_sec = duration_sec;
    s_ctx.timers[slot].start_time_us = esp_timer_get_time();
    strncpy(s_ctx.timers[slot].message,
            message ? message : "倒计时到了",
            REMINDER_MSG_MAX_LEN - 1);
    s_ctx.timers[slot].message[REMINDER_MSG_MAX_LEN - 1] = '\0'; /* [FIX-10] */

    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "启动倒计时 #%d: %lu秒 [%s]",
             slot, (unsigned long)duration_sec, s_ctx.timers[slot].message);
    return slot;
}

esp_err_t reminder_timer_cancel(uint8_t timer_id)
{
    if (timer_id >= REMINDER_MAX_TIMERS)
        return ESP_ERR_NOT_FOUND;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (!s_ctx.timers[timer_id].active)
    {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }
    s_ctx.timers[timer_id].active = false;
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "取消倒计时 #%d", timer_id);
    return ESP_OK;
}

esp_err_t reminder_timer_get_remain(uint8_t timer_id, uint32_t *out_remain)
{
    if (timer_id >= REMINDER_MAX_TIMERS || out_remain == NULL)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (!s_ctx.timers[timer_id].active)
    {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }
    int64_t elapsed = (esp_timer_get_time() - s_ctx.timers[timer_id].start_time_us) / 1000000;
    int64_t remain = (int64_t)s_ctx.timers[timer_id].duration_sec - elapsed;
    *out_remain = (remain > 0) ? (uint32_t)remain : 0;
    xSemaphoreGive(s_ctx.mutex);
    return ESP_OK;
}

/**
 * @brief 查询最近一个倒计时的剩余秒数（2026-08-25 新增，供低功耗临期判断）
 *
 * 语义与返回值见 reminder.h。实现要点：
 *   · 只读遍历，不改任何状态，不投递任何事件；
 *   · 剩余秒数算法与 reminder_timer_get_remain() 完全一致（保持口径统一）；
 *   · ★ 互斥量用【短超时】而非 portMAX_DELAY —— 调用方是 standby_task，
 *     处在低功耗判定的关键路径上，绝不能被 reminder 的锁长时间卡住。
 *     拿不到锁就返回 INVALID_STATE，由调用方按"查不到就照常进待机"处理，
 *     退化行为与本次改动之前完全一致，无害。
 */
esp_err_t reminder_get_nearest_expire_sec(uint32_t *out_sec)
{
    if (out_sec == NULL)
        return ESP_ERR_INVALID_ARG;
    if (!s_ctx.initialized || s_ctx.mutex == NULL)
        return ESP_ERR_INVALID_STATE;

    if (xSemaphoreTake(s_ctx.mutex, pdMS_TO_TICKS(10)) != pdTRUE)
        return ESP_ERR_INVALID_STATE; /* 拿不到锁：放弃本次查询，不阻塞调用方 */

    int64_t now_us = esp_timer_get_time();
    bool found = false;
    uint32_t min_remain = 0;

    for (uint8_t i = 0; i < REMINDER_MAX_TIMERS; i++)
    {
        const timer_entry_t *t = &s_ctx.timers[i];
        if (!t->active)
            continue;

        int64_t elapsed = (now_us - t->start_time_us) / 1000000;
        int64_t remain = (int64_t)t->duration_sec - elapsed;
        uint32_t r = (remain > 0) ? (uint32_t)remain : 0;

        if (!found || r < min_remain)
        {
            min_remain = r;
            found = true;
        }
    }

    xSemaphoreGive(s_ctx.mutex);

    if (!found)
        return ESP_ERR_NOT_FOUND; /* 当前没有任何倒计时在跑 */

    *out_sec = min_remain;
    return ESP_OK;
}

/* ═══════════════════════════════════════════════════════════════════
 * 12. 对外接口 — 日历
 * ═══════════════════════════════════════════════════════════════════ */
int reminder_calendar_add(const calendar_entry_t *entry)
{
    if (entry == NULL)
        return -1;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (s_ctx.calendar_count >= REMINDER_MAX_CALENDARS)
    {
        ESP_LOGW(TAG, "日历事件已满 (%d)", REMINDER_MAX_CALENDARS);
        xSemaphoreGive(s_ctx.mutex);
        return -1;
    }

    uint8_t new_id = s_ctx.calendar_count;
    s_ctx.calendars[new_id] = *entry;
    s_ctx.calendars[new_id].id = new_id;
    s_ctx.calendars[new_id].enabled = true;
    s_ctx.calendar_count++;

    nvs_save_calendars(); /* [FIX-5] 已改为异步 */
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "添加日历事件 #%d: %04d-%02d-%02d %02d:%02d [%s]",
             new_id, entry->year, entry->month, entry->day,
             entry->hour, entry->minute, entry->message);
    return new_id;
}

esp_err_t reminder_calendar_delete(uint8_t cal_id)
{
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    if (cal_id >= s_ctx.calendar_count)
    {
        xSemaphoreGive(s_ctx.mutex);
        return ESP_ERR_NOT_FOUND;
    }
    for (uint8_t i = cal_id; i < s_ctx.calendar_count - 1; i++)
    {
        s_ctx.calendars[i] = s_ctx.calendars[i + 1];
        s_ctx.calendars[i].id = i;
    }
    s_ctx.calendar_count--;
    nvs_save_calendars(); /* [FIX-5] 已改为异步 */
    xSemaphoreGive(s_ctx.mutex);
    ESP_LOGI(TAG, "删除日历事件 #%d", cal_id);
    return ESP_OK;
}

void reminder_calendar_get_today(calendar_entry_t *out_list, uint8_t *out_count)
{
    if (!s_ctx.sntp_synced)
    {
        if (out_count)
            *out_count = 0;
        return;
    }

    time_t now = time(NULL);
    struct tm now_tm;
    localtime_r(&now, &now_tm);
    uint8_t count = 0;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    for (uint8_t i = 0; i < s_ctx.calendar_count; i++)
    {
        calendar_entry_t *cal = &s_ctx.calendars[i];
        if (cal->enabled &&
            cal->year == (uint16_t)(now_tm.tm_year + 1900) &&
            cal->month == (uint8_t)(now_tm.tm_mon + 1) &&
            cal->day == (uint8_t)now_tm.tm_mday)
        {
            if (out_list)
                out_list[count] = *cal;
            count++;
        }
    }
    xSemaphoreGive(s_ctx.mutex);

    if (out_count)
        *out_count = count;
}

/* ═══════════════════════════════════════════════════════════════════
 * 13. 对外接口 — 天气
 * ═══════════════════════════════════════════════════════════════════ */
// esp_err_t reminder_weather_config(const weather_config_t *config)
// {
//     if (config == NULL)
//         return ESP_ERR_INVALID_ARG;

//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//     s_ctx.weather_cfg = *config;
//     s_ctx.weather_morning_done = false;
//     s_ctx.weather_evening_done = false;
//     nvs_save_weather_config();
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "天气配置更新: 城市=%s, 时段=%d",
//              config->city_name, config->schedule);
//     return ESP_OK;
// }
esp_err_t reminder_weather_config(const reminder_weather_cfg_t *config)
{
    if (config == NULL)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    s_ctx.weather_cfg = *config;
    s_ctx.weather_morning_done = false;
    s_ctx.weather_evening_done = false;
    nvs_save_weather_config();
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "天气配置更新: 城市=%s, 时段=%d",
             config->city_name, config->schedule);
    return ESP_OK;
}
esp_err_t reminder_weather_fetch_now(void)
{
    reminder_evt_t evt = {.type = REM_EVT_WEATHER_FETCH};
    return xQueueSend(s_ctx.evt_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE
               ? ESP_OK
               : ESP_FAIL;
}

/* ═══════════════════════════════════════════════════════════════════
 * 14. IP 自动定位城市
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 通用 HTTP GET 请求（复用 http_event_handler）
 *
 * @param url      请求 URL
 * @param buf      响应缓冲区
 * @param buf_size 缓冲区大小
 * @return ESP_OK 请求成功 / ESP_FAIL 请求失败
 */
// static esp_err_t http_get(const char *url, char *buf, size_t buf_size)
// {
//     memset(buf, 0, buf_size);
//     http_response_t resp = {
//         .buf = buf,
//         .len = 0,
//         .capacity = buf_size,
//     };

//     esp_http_client_config_t config = {
//         .url = url,
//         .event_handler = http_event_handler,
//         .user_data = &resp,
//         .timeout_ms = 10000,
//         .skip_cert_common_name_check = true,
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     if (client == NULL)
//     {
//         ESP_LOGE(TAG, "HTTP 客户端初始化失败");
//         return ESP_FAIL;
//     }

//     esp_err_t err = esp_http_client_perform(client);
//     int status_code = esp_http_client_get_status_code(client);
//     esp_http_client_cleanup(client);

//     if (err != ESP_OK || (status_code != 200 && status_code != 0))
//     {
//         ESP_LOGW(TAG, "HTTP 请求失败: err=%s, status=%d", esp_err_to_name(err), status_code);
//         return ESP_FAIL;
//     }
//     return ESP_OK;
// }

/**
 * @brief URL 编码（将中文等非 ASCII 字符转为 %XX 格式）
 *
 * @param src     原始字符串
 * @param dst     输出缓冲区
 * @param dst_size 输出缓冲区大小
 */
// static void url_encode(const char *src, char *dst, size_t dst_size)
// {
//     size_t j = 0;
//     for (size_t i = 0; src[i] != '\0' && j < dst_size - 1; i++)
//     {
//         unsigned char c = (unsigned char)src[i];
//         if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z') ||
//             (c >= '0' && c <= '9') || c == '-' || c == '_' || c == '.' || c == '~')
//         {
//             dst[j++] = c;
//         }
//         else if (j + 4 <= dst_size)
//         {
//             j += snprintf(dst + j, dst_size - j, "%%%02X", c);
//         }
//     }
//     dst[j] = '\0';
// }

esp_err_t reminder_auto_locate_city(void)
{
    ESP_LOGI(TAG, "开始使用天气组件进行 IP 定位...");

    // 1. 调用新组件的 IP 定位接口
    location_info_t *loc = get_city_by_ip(NULL);

    if (loc == NULL || loc->city == NULL)
    {
        ESP_LOGE(TAG, "IP 定位失败，保留默认城市");
        if (loc)
            location_info_free(loc);
        return ESP_FAIL;
    }

    // 2. 更新到你的 NVS 配置中
    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

    // 心知天气直接通过中文城市名（如 "北京"）即可查询，无需繁琐的 GeoAPI 转换
    strncpy(s_ctx.weather_cfg.city_name, loc->city, sizeof(s_ctx.weather_cfg.city_name) - 1);
    s_ctx.weather_cfg.city_name[sizeof(s_ctx.weather_cfg.city_name) - 1] = '\0';

    // city_code 可以废弃或直接用中文名覆盖，保证向后兼容
    strncpy(s_ctx.weather_cfg.city_code, loc->city, sizeof(s_ctx.weather_cfg.city_code) - 1);
    s_ctx.weather_cfg.city_code[sizeof(s_ctx.weather_cfg.city_code) - 1] = '\0';

    nvs_save_weather_config();
    xSemaphoreGive(s_ctx.mutex);

    ESP_LOGI(TAG, "IP 定位成功: %s", s_ctx.weather_cfg.city_name);

    // 3. 释放组件内存
    location_info_free(loc);

    return ESP_OK;
}

// esp_err_t reminder_auto_locate_city(void)
// {
//     /* 从堆分配缓冲区，避免撑爆调用者栈 */
//     char *buf = malloc_zeroed(1024);
//     if (buf == NULL)
//     {
//         ESP_LOGE(TAG, "IP 定位缓冲区分配失败");
//         return ESP_FAIL;
//     }

//     /* ── 步骤 1：请求 ip-api.com 获取当前城市名 ── */
//     ESP_LOGI(TAG, "开始 IP 定位...");
//     if (http_get(IP_LOCATION_API_URL, buf, 1024) != ESP_OK)
//     {
//         ESP_LOGW(TAG, "IP 定位请求失败，保留默认城市");
//         heap_caps_free(buf);
//         return ESP_FAIL;
//     }

//     ESP_LOGI(TAG, "IP 定位响应: %s", buf);

//     cJSON *ip_root = cJSON_Parse(buf);
//     if (ip_root == NULL)
//     {
//         ESP_LOGW(TAG, "IP 定位 JSON 解析失败");
//         heap_caps_free(buf);
//         return ESP_FAIL;
//     }

//     cJSON *status = cJSON_GetObjectItem(ip_root, "status");
//     if (!cJSON_IsString(status) || strcmp(status->valuestring, "success") != 0)
//     {
//         ESP_LOGW(TAG, "IP 定位返回状态异常");
//         cJSON_Delete(ip_root);
//         heap_caps_free(buf);
//         return ESP_FAIL;
//     }

//     cJSON *city = cJSON_GetObjectItem(ip_root, "city");
//     if (!cJSON_IsString(city) || city->valuestring == NULL)
//     {
//         ESP_LOGW(TAG, "IP 定位未返回城市名");
//         cJSON_Delete(ip_root);
//         heap_caps_free(buf);
//         return ESP_FAIL;
//     }

//     char city_name[32];
//     strncpy(city_name, city->valuestring, sizeof(city_name) - 1);
//     city_name[sizeof(city_name) - 1] = '\0';
//     cJSON_Delete(ip_root);
//     ESP_LOGI(TAG, "IP 定位城市: %s", city_name);

// #if WEATHER_API_TYPE == 0
//     /* ── 步骤 2（和风）：请求 GeoAPI 查询城市代码 ── */
//     char geo_url[256];
//     char encoded_geo_city[64];
//     url_encode(city_name, encoded_geo_city, sizeof(encoded_geo_city));
//     snprintf(geo_url, sizeof(geo_url), WEATHER_GEO_API_URL_FMT, encoded_geo_city, WEATHER_API_KEY);

//     if (http_get(geo_url, buf, 1024) != ESP_OK)
//     {
//         ESP_LOGW(TAG, "和风 GeoAPI 请求失败，保留默认城市");
//         heap_caps_free(buf);
//         return ESP_FAIL;
//     }

//     ESP_LOGI(TAG, "GeoAPI 响应: %s", buf);

//     cJSON *geo_root = cJSON_Parse(buf);
//     heap_caps_free(buf);
//     buf = NULL;

//     if (geo_root == NULL)
//     {
//         ESP_LOGW(TAG, "GeoAPI JSON 解析失败");
//         return ESP_FAIL;
//     }

//     cJSON *location = cJSON_GetObjectItem(geo_root, "location");
//     if (!cJSON_IsArray(location) || cJSON_GetArraySize(location) == 0)
//     {
//         ESP_LOGW(TAG, "GeoAPI 未找到匹配城市");
//         cJSON_Delete(geo_root);
//         return ESP_FAIL;
//     }

//     cJSON *first = cJSON_GetArrayItem(location, 0);
//     cJSON *loc_id = cJSON_GetObjectItem(first, "id");
//     cJSON *loc_name = cJSON_GetObjectItem(first, "name");

//     if (!cJSON_IsString(loc_id) || !cJSON_IsString(loc_name))
//     {
//         ESP_LOGW(TAG, "GeoAPI 返回数据格式异常");
//         cJSON_Delete(geo_root);
//         return ESP_FAIL;
//     }

//     /* ── 步骤 3：更新 weather_cfg 并持久化到 NVS ── */
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     strncpy(s_ctx.weather_cfg.city_code, loc_id->valuestring,
//             sizeof(s_ctx.weather_cfg.city_code) - 1);
//     s_ctx.weather_cfg.city_code[sizeof(s_ctx.weather_cfg.city_code) - 1] = '\0';

//     strncpy(s_ctx.weather_cfg.city_name, loc_name->valuestring,
//             sizeof(s_ctx.weather_cfg.city_name) - 1);
//     s_ctx.weather_cfg.city_name[sizeof(s_ctx.weather_cfg.city_name) - 1] = '\0';

//     nvs_save_weather_config();
//     xSemaphoreGive(s_ctx.mutex);

//     cJSON_Delete(geo_root);
// #else
//     /* ── 心知天气：直接用城市名作为 location，无需 GeoAPI ── */
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     strncpy(s_ctx.weather_cfg.city_code, city_name,
//             sizeof(s_ctx.weather_cfg.city_code) - 1);
//     s_ctx.weather_cfg.city_code[sizeof(s_ctx.weather_cfg.city_code) - 1] = '\0';

//     strncpy(s_ctx.weather_cfg.city_name, city_name,
//             sizeof(s_ctx.weather_cfg.city_name) - 1);
//     s_ctx.weather_cfg.city_name[sizeof(s_ctx.weather_cfg.city_name) - 1] = '\0';

//     nvs_save_weather_config();
//     xSemaphoreGive(s_ctx.mutex);

//     heap_caps_free(buf);
// #endif

//     ESP_LOGI(TAG, "IP 定位成功: %s → %s", s_ctx.weather_cfg.city_name,
//              s_ctx.weather_cfg.city_code);
//     return ESP_OK;
// }

esp_err_t reminder_get_weather_data(weather_data_t *data)
{
    if (data == NULL)
        return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    *data = s_ctx.weather_data;
    strncpy(data->city_name, s_ctx.weather_cfg.city_name, sizeof(data->city_name) - 1);
    data->city_name[sizeof(data->city_name) - 1] = '\0';
    xSemaphoreGive(s_ctx.mutex);

    return s_ctx.weather_data.valid ? ESP_OK : ESP_FAIL;
}
