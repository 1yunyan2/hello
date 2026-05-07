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
    uint8_t last_weather_day;

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
    bool initialized;
} reminder_ctx_t;

static reminder_ctx_t s_ctx = {0};

#define NVS_NAMESPACE "reminder"

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

/* [FIX-6] NVS 保存任务支持闹钟/日历/退出三种命令 */
static void nvs_save_task(void *arg)
{
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

/* ═══════════════════════════════════════════════════════════════════
 * 4. SNTP 时间同步
 * ═══════════════════════════════════════════════════════════════════ */
static void sntp_sync_notification_cb(struct timeval *tv)
{
    ESP_LOGI(TAG, "SNTP 时间同步完成");
    s_ctx.sntp_synced = true;
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

static void alarm_ring_start(uint8_t alarm_id, const char *message)
{
    s_ctx.state = REMINDER_STATE_RINGING;
    s_ctx.ringing_alarm_id = alarm_id;
    s_ctx.ring_count = 0;

    ESP_LOGW(TAG, "闹钟 #%d 开始响铃: %s", alarm_id, message);

    if (s_ctx.trigger_cb)
        s_ctx.trigger_cb(REMINDER_TYPE_ALARM, message, true);
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
    ESP_LOGI(TAG, "使用新组件获取天气(心知): %s", s_ctx.weather_cfg.city_name);

    // 注意：这里调用的 weather_config_t 是新组件定义的！
    weather_config_t config = {
        .api_key = WEATHER_API_KEY,          // 使用我们在 reminder.h 定义的宏
        .api_host = NULL,                    // 心知不需要 host
        .city = s_ctx.weather_cfg.city_name, // 使用我们本地存储的城市名
        .type = WEATHER_XINZHI               // 指定心知类型
    };

    weather_info_t *info = weather_get(&config);

    if (info == NULL)
    {
        ESP_LOGE(TAG, "天气获取失败");
        return ESP_FAIL;
    }

    xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
    // 更新温度
    snprintf(s_ctx.weather_data.temp, sizeof(s_ctx.weather_data.temp), "%.1f", info->temperature);
    // 更新描述
    if (info->weather)
    {
        strncpy(s_ctx.weather_data.text, info->weather, sizeof(s_ctx.weather_data.text) - 1);
    }
    // 心知免费版无湿度，设为固定值或 "--"
    strncpy(s_ctx.weather_data.humidity, "--", sizeof(s_ctx.weather_data.humidity) - 1);

    s_ctx.weather_data.valid = true;
    xSemaphoreGive(s_ctx.mutex);

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

    /* ── 日历事件检查 ── */
    if (current_min != s_ctx.last_cal_check_min)
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

    /* ── 天气数据拉取检查（每 4 小时自动拉取，无语音播报） ── */
    if (s_ctx.weather_cfg.schedule != WEATHER_SCHEDULE_DISABLED &&
        s_ctx.state == REMINDER_STATE_IDLE)
    {
        int current_hour = now_tm.tm_hour;
        /* 首次拉取 或 距上次拉取超过 4 小时 */
        if (s_ctx.last_weather_day == 0 ||
            (current_hour - s_ctx.last_weather_day + 24) % 24 >= 4)
        {
            evt.type = REM_EVT_WEATHER_FETCH;
            xQueueSend(s_ctx.evt_queue, &evt, 0);
            s_ctx.last_weather_day = current_hour;
        }
    }
}

/* ═══════════════════════════════════════════════════════════════════
 * 8. 提醒任务
 * ═══════════════════════════════════════════════════════════════════ */
static void reminder_task(void *arg)
{
    reminder_evt_t evt;
    ESP_LOGI(TAG, "提醒任务启动");

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
            if (s_ctx.trigger_cb)
                s_ctx.trigger_cb(REMINDER_TYPE_TIMER, evt.message, false);
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
        BaseType_t r = xTaskCreate(nvs_save_task, "nvs_save", 4096, NULL, 1, NULL);
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
    }

    nvs_load_alarms();
    nvs_load_calendars();
    nvs_load_weather_config();

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
    // 3.5 IP 自动定位城市（WiFi 已连接，获取天气城市代码）
    reminder_auto_locate_city();
    // 3.6 定位成功后立即拉取一次天气数据（否则要等到定时播报才有数据）
    reminder_weather_fetch_now();
    return ESP_OK;
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
