// /**
//  * @file reminder.c
//  * @brief 提醒系统实现 — 闹钟/倒计时/日历/天气 四合一引擎
//  *
//  * 核心机制：
//  *   1. 使用 esp_timer 创建 1 秒轮询定时器（心跳）
//  *   2. 轮询回调仅做"事件投递"，不做耗时操作
//  *   3. 独立的 reminder_task 处理所有提醒逻辑（播报、HTTP、震动等）
//  *   4. 优先级队列：闹钟 > 倒计时 > 日历 > 天气
//  *
//  * 闹钟响铃流程：
//  *   闹钟到点 → 进入 RINGING 状态 → 每 5 秒重复播报+震动+舵机
//  *   → 用户语音"关闭" → reminder_alarm_dismiss() → 回到 IDLE
//  *   → 或 60 秒无响应 → 自动关闭
//  *
//  * 线程模型：
//  *   - poll_timer_callback: esp_timer 上下文，仅投递事件到队列（轻量）
//  *   - reminder_task: 独立 FreeRTOS 任务，处理所有提醒逻辑（8KB 栈）
//  *   - mutex 保护共享数据（闹钟/日历/倒计时列表）
//  */

// #include "reminder.h"
// #include "esp_log.h"
// #include "esp_timer.h"
// #include "esp_sntp.h"
// #include "esp_http_client.h"
// #include "nvs_flash.h"
// #include "nvs.h"
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "freertos/queue.h"
// #include "freertos/semphr.h"
// #include "cJSON.h"
// #include <string.h>
// #include <sys/time.h>

// static const char *TAG = "REMINDER";

// /* ═══════════════════════════════════════════════════════════════════
//  * 1. 内部事件定义（定时器→任务的通信协议）
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief 提醒事件类型（投递到 reminder_task 的事件队列）
//  */
// typedef enum {
//     REM_EVT_ALARM_TRIGGER,     ///< 闹钟到点（附带闹钟 ID）
//     REM_EVT_TIMER_EXPIRE,      ///< 倒计时到期（附带 timer ID）
//     REM_EVT_CALENDAR_TRIGGER,  ///< 日历事件触发（附带事件 ID）
//     REM_EVT_WEATHER_FETCH,     ///< 天气播报时间到
//     REM_EVT_ALARM_DISMISS,     ///< 用户关闭闹钟
//     REM_EVT_ALARM_SNOOZE,      ///< 用户贪睡
//     REM_EVT_ALARM_RING_TICK,   ///< 闹钟响铃循环（每 5 秒重复）
// } reminder_evt_type_t;

// /**
//  * @brief 提醒事件结构体
//  */
// typedef struct {
//     reminder_evt_type_t type;  ///< 事件类型
//     uint8_t  id;               ///< 关联的闹钟/倒计时/日历 ID
//     uint8_t  snooze_min;       ///< 贪睡分钟数（仅 SNOOZE 事件使用）
//     char     message[REMINDER_MSG_MAX_LEN]; ///< 提醒消息内容
// } reminder_evt_t;

// /* ═══════════════════════════════════════════════════════════════════
//  * 2. 运行时上下文（模块内单例）
//  * ═══════════════════════════════════════════════════════════════════ */

// typedef struct {
//     /* 回调 */
//     reminder_trigger_cb_t trigger_cb;

//     /* 状态 */
//     reminder_state_t state;                ///< 当前状态

//     /* 闹钟数据 */
//     alarm_entry_t    alarms[REMINDER_MAX_ALARMS];
//     uint8_t          alarm_count;

//     /* 倒计时数据 */
//     timer_entry_t    timers[REMINDER_MAX_TIMERS];

//     /* 日历数据 */
//     calendar_entry_t calendars[REMINDER_MAX_CALENDARS];
//     uint8_t          calendar_count;

//     /* 天气配置 */
//     weather_config_t weather_cfg;
//     bool             weather_morning_done;   ///< 今天早间播报已完成
//     bool             weather_evening_done;   ///< 今天晚间播报已完成
//     uint8_t          last_weather_day;       ///< 上次天气播报的日期（跨日重置用）

//     /* 闹钟响铃状态 */
//     uint8_t          ringing_alarm_id;       ///< 当前正在响铃的闹钟 ID
//     uint8_t          ring_count;             ///< 已响铃次数
//     esp_timer_handle_t ring_timer;           ///< 响铃循环定时器（每 5 秒触发）

//     /* 防重复触发：记录上次触发的分钟数，避免同一分钟内重复触发 */
//     int              last_alarm_check_min;   ///< 上次闹钟检查的分钟（-1=未检查）
//     int              last_cal_check_min;     ///< 上次日历检查的分钟

//     /* 系统资源 */
//     esp_timer_handle_t poll_timer;           ///< 1 秒轮询定时器
//     QueueHandle_t    evt_queue;              ///< 事件队列（定时器→任务）
//     TaskHandle_t     task_handle;            ///< 提醒任务句柄
//     SemaphoreHandle_t mutex;                 ///< 互斥锁（保护共享数据）
//     bool             sntp_synced;            ///< SNTP 是否已同步
//     bool             initialized;            ///< 模块是否已初始化
// } reminder_ctx_t;

// static reminder_ctx_t s_ctx = {0};

// #define NVS_NAMESPACE "reminder"

// /* ═══════════════════════════════════════════════════════════════════
//  * 3. NVS 持久化（内部函数）
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief 保存所有闹钟到 NVS
//  *
//  * 以 "alarm_cnt" 存总数，"alarm_0"~"alarm_7" 存各条 blob。
//  * 调用时机：add / delete / enable / disable 后立即保存。
//  */
// static void nvs_save_alarms(void)
// {
//     nvs_handle_t handle;
//     if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
//         ESP_LOGE(TAG, "NVS 打开失败，无法保存闹钟");
//         return;
//     }

//     nvs_set_u8(handle, "alarm_cnt", s_ctx.alarm_count);
//     for (uint8_t i = 0; i < s_ctx.alarm_count; i++) {
//         char key[16];
//         snprintf(key, sizeof(key), "alarm_%d", i);
//         nvs_set_blob(handle, key, &s_ctx.alarms[i], sizeof(alarm_entry_t));
//     }

//     nvs_commit(handle);
//     nvs_close(handle);
//     ESP_LOGI(TAG, "闹钟数据已保存，共 %d 条", s_ctx.alarm_count);
// }

// /**
//  * @brief 从 NVS 加载闹钟
//  */
// static void nvs_load_alarms(void)
// {
//     nvs_handle_t handle;
//     if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
//         ESP_LOGW(TAG, "NVS 无闹钟数据（首次启动）");
//         return;
//     }

//     uint8_t count = 0;
//     if (nvs_get_u8(handle, "alarm_cnt", &count) == ESP_OK) {
//         s_ctx.alarm_count = (count > REMINDER_MAX_ALARMS) ? REMINDER_MAX_ALARMS : count;
//         for (uint8_t i = 0; i < s_ctx.alarm_count; i++) {
//             char key[16];
//             snprintf(key, sizeof(key), "alarm_%d", i);
//             size_t len = sizeof(alarm_entry_t);
//             nvs_get_blob(handle, key, &s_ctx.alarms[i], &len);
//         }
//         ESP_LOGI(TAG, "从 NVS 加载 %d 条闹钟", s_ctx.alarm_count);
//     }
//     nvs_close(handle);
// }

// /**
//  * @brief 保存所有日历事件到 NVS
//  */
// static void nvs_save_calendars(void)
// {
//     nvs_handle_t handle;
//     if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
//         ESP_LOGE(TAG, "NVS 打开失败，无法保存日历");
//         return;
//     }

//     nvs_set_u8(handle, "cal_cnt", s_ctx.calendar_count);
//     for (uint8_t i = 0; i < s_ctx.calendar_count; i++) {
//         char key[16];
//         snprintf(key, sizeof(key), "cal_%02d", i);
//         nvs_set_blob(handle, key, &s_ctx.calendars[i], sizeof(calendar_entry_t));
//     }

//     nvs_commit(handle);
//     nvs_close(handle);
//     ESP_LOGI(TAG, "日历数据已保存，共 %d 条", s_ctx.calendar_count);
// }

// /**
//  * @brief 从 NVS 加载日历事件
//  */
// static void nvs_load_calendars(void)
// {
//     nvs_handle_t handle;
//     if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
//         return;
//     }

//     uint8_t count = 0;
//     if (nvs_get_u8(handle, "cal_cnt", &count) == ESP_OK) {
//         s_ctx.calendar_count = (count > REMINDER_MAX_CALENDARS) ? REMINDER_MAX_CALENDARS : count;
//         for (uint8_t i = 0; i < s_ctx.calendar_count; i++) {
//             char key[16];
//             snprintf(key, sizeof(key), "cal_%02d", i);
//             size_t len = sizeof(calendar_entry_t);
//             nvs_get_blob(handle, key, &s_ctx.calendars[i], &len);
//         }
//         ESP_LOGI(TAG, "从 NVS 加载 %d 条日历事件", s_ctx.calendar_count);
//     }
//     nvs_close(handle);
// }

// /**
//  * @brief 保存天气配置到 NVS
//  */
// static void nvs_save_weather_config(void)
// {
//     nvs_handle_t handle;
//     if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
//         return;
//     }
//     nvs_set_blob(handle, "weather_cfg", &s_ctx.weather_cfg, sizeof(weather_config_t));
//     nvs_commit(handle);
//     nvs_close(handle);
// }

// /**
//  * @brief 从 NVS 加载天气配置，首次启动使用默认值
//  */
// static void nvs_load_weather_config(void)
// {
//     nvs_handle_t handle;
//     if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) != ESP_OK) {
//         goto use_defaults;
//     }

//     size_t len = sizeof(weather_config_t);
//     if (nvs_get_blob(handle, "weather_cfg", &s_ctx.weather_cfg, &len) == ESP_OK) {
//         nvs_close(handle);
//         return;
//     }
//     nvs_close(handle);

// use_defaults:
//     /* 首次启动默认配置 */
//     s_ctx.weather_cfg.schedule = WEATHER_SCHEDULE_MORNING;
//     strncpy(s_ctx.weather_cfg.city_code, WEATHER_DEFAULT_CITY,
//             sizeof(s_ctx.weather_cfg.city_code) - 1);
//     strncpy(s_ctx.weather_cfg.city_name, "杭州",
//             sizeof(s_ctx.weather_cfg.city_name) - 1);
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 4. SNTP 时间同步
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief SNTP 同步完成回调
//  *
//  * 同步成功后，系统时间有效，闹钟/日历/时间显示功能可用。
//  */
// static void sntp_sync_notification_cb(struct timeval *tv)
// {
//     ESP_LOGI(TAG, "SNTP 时间同步完成");
//     s_ctx.sntp_synced = true;
// }

// /**
//  * @brief 初始化 SNTP 时间同步
//  *
//  * 使用阿里云 NTP 服务器（国内低延迟）+ pool.ntp.org 备选。
//  * 时区设为东八区（CST-8）。
//  */
// static void sntp_time_sync_init(void)
// {
//     ESP_LOGI(TAG, "初始化 SNTP 时间同步...");

//     /* 设置时区为东八区（中国标准时间） */
//     setenv("TZ", "CST-8", 1);
//     tzset();

//     esp_sntp_setoperatingmode(SNTP_OPMODE_POLL);
//     esp_sntp_setservername(0, "ntp.aliyun.com");
//     esp_sntp_setservername(1, "pool.ntp.org");
//     sntp_set_time_sync_notification_cb(sntp_sync_notification_cb);
//     esp_sntp_init();
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 5. 闹钟匹配与响铃控制
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief 判断闹钟是否应在当前时刻触发
//  *
//  * 检查规则：
//  *   1. enabled == true
//  *   2. 时、分精确匹配
//  *   3. 根据重复模式检查星期几
//  *
//  * @param alarm   闹钟条目
//  * @param now_tm  当前本地时间
//  * @return true=应触发
//  */
// static bool alarm_should_trigger(const alarm_entry_t *alarm, const struct tm *now_tm)
// {
//     if (!alarm->enabled) {
//         return false;
//     }
//     if (now_tm->tm_hour != alarm->hour || now_tm->tm_min != alarm->minute) {
//         return false;
//     }

//     int wday = now_tm->tm_wday; // 0=周日

//     switch (alarm->repeat) {
//     case ALARM_REPEAT_ONCE:
//     case ALARM_REPEAT_DAILY:
//         return true;
//     case ALARM_REPEAT_WEEKDAY:
//         return (wday >= 1 && wday <= 5);
//     case ALARM_REPEAT_WEEKEND:
//         return (wday == 0 || wday == 6);
//     case ALARM_REPEAT_CUSTOM:
//         return (alarm->weekday_mask & (1 << wday)) != 0;
//     default:
//         return false;
//     }
// }

// /**
//  * @brief 闹钟响铃循环定时器回调（每 ALARM_RING_INTERVAL_MS 触发一次）
//  *
//  * 仅投递 REM_EVT_ALARM_RING_TICK 事件到队列，实际播报在 reminder_task 中执行。
//  */
// static void ring_timer_callback(void *arg)
// {
//     reminder_evt_t evt = {
//         .type = REM_EVT_ALARM_RING_TICK,
//     };
//     xQueueSend(s_ctx.evt_queue, &evt, 0);
// }

// /**
//  * @brief 启动闹钟响铃循环
//  *
//  * 进入 RINGING 状态，创建周期定时器，每 5 秒重复播报。
//  * 首次播报在调用时立即执行（不等定时器第一次触发）。
//  *
//  * @param alarm_id  触发的闹钟 ID
//  * @param message   闹钟提醒内容
//  */
// static void alarm_ring_start(uint8_t alarm_id, const char *message)
// {
//     s_ctx.state = REMINDER_STATE_RINGING;
//     s_ctx.ringing_alarm_id = alarm_id;
//     s_ctx.ring_count = 0;

//     ESP_LOGW(TAG, "闹钟 #%d 开始响铃: %s", alarm_id, message);

//     /* 首次立即播报 */
//     if (s_ctx.trigger_cb) {
//         s_ctx.trigger_cb(REMINDER_TYPE_ALARM, message, true);
//     }
//     s_ctx.ring_count++;

//     /* 启动响铃循环定时器 */
//     if (s_ctx.ring_timer == NULL) {
//         esp_timer_create_args_t args = {
//             .callback = ring_timer_callback,
//             .name = "alarm_ring",
//         };
//         esp_timer_create(&args, &s_ctx.ring_timer);
//     }
//     esp_timer_start_periodic(s_ctx.ring_timer, ALARM_RING_INTERVAL_MS * 1000);
// }

// /**
//  * @brief 停止闹钟响铃，回到 IDLE 状态
//  *
//  * 停止循环定时器，清除 RINGING 状态。
//  * 如果是一次性闹钟，自动禁用。
//  */
// static void alarm_ring_stop(void)
// {
//     if (s_ctx.state != REMINDER_STATE_RINGING) {
//         return;
//     }

//     /* 停止响铃定时器 */
//     if (s_ctx.ring_timer) {
//         esp_timer_stop(s_ctx.ring_timer);
//     }

//     /* 一次性闹钟触发后自动禁用 */
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//     if (s_ctx.ringing_alarm_id < s_ctx.alarm_count &&
//         s_ctx.alarms[s_ctx.ringing_alarm_id].repeat == ALARM_REPEAT_ONCE) {
//         s_ctx.alarms[s_ctx.ringing_alarm_id].enabled = false;
//         nvs_save_alarms();
//         ESP_LOGI(TAG, "一次性闹钟 #%d 已自动禁用", s_ctx.ringing_alarm_id);
//     }
//     xSemaphoreGive(s_ctx.mutex);

//     s_ctx.state = REMINDER_STATE_IDLE;
//     s_ctx.ring_count = 0;
//     ESP_LOGI(TAG, "闹钟响铃已停止");
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 6. 天气获取（独立任务中执行，避免栈溢出）
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief HTTP 响应缓冲区
//  */
// typedef struct {
//     char   *buf;
//     size_t  len;
//     size_t  capacity;
// } http_response_t;

// /**
//  * @brief HTTP 事件回调 — 累积响应数据
//  */
// static esp_err_t http_event_handler(esp_http_client_event_t *evt)
// {
//     http_response_t *resp = (http_response_t *)evt->user_data;
//     if (evt->event_id == HTTP_EVENT_ON_DATA && resp != NULL) {
//         if (resp->len + evt->data_len < resp->capacity) {
//             memcpy(resp->buf + resp->len, evt->data, evt->data_len);
//             resp->len += evt->data_len;
//             resp->buf[resp->len] = '\0';
//         }
//     }
//     return ESP_OK;
// }

// /**
//  * @brief 执行天气 API 请求并通过回调播报
//  *
//  * 在 reminder_task 上下文中执行（有足够栈空间）。
//  *
//  * API 接入说明（后续只需填入 WEATHER_API_KEY 和城市代码即可）：
//  *   和风天气免费版 API：
//  *     URL: https://devapi.qweather.com/v7/weather/now?location=<城市代码>&key=<API_KEY>
//  *     返回 JSON 中 now.text = 天气描述, now.temp = 温度
//  *
//  *   心知天气 API：
//  *     URL: https://api.seniverse.com/v3/weather/now.json?key=<KEY>&location=<城市>
//  *     返回 JSON 中 results[0].now.text / results[0].now.temperature
//  *
//  * @return ESP_OK / ESP_FAIL
//  */
// static esp_err_t weather_fetch_and_notify(void)
// {
//     ESP_LOGI(TAG, "获取天气信息: %s (%s)",
//              s_ctx.weather_cfg.city_name, s_ctx.weather_cfg.city_code);

//     char response_buf[2048] = {0};
//     http_response_t resp = {
//         .buf = response_buf,
//         .len = 0,
//         .capacity = sizeof(response_buf),
//     };

//     /* 构建天气 API URL（修改 WEATHER_API_URL_FMT 和 WEATHER_API_KEY 即可接入） */
//     char url[256];
//     snprintf(url, sizeof(url), WEATHER_API_URL_FMT,
//              s_ctx.weather_cfg.city_code, WEATHER_API_KEY);

//     esp_http_client_config_t config = {
//         .url = url,
//         .event_handler = http_event_handler,
//         .user_data = &resp,
//         .timeout_ms = WEATHER_FETCH_TIMEOUT_MS,
//     };

//     esp_http_client_handle_t client = esp_http_client_init(&config);
//     if (client == NULL) {
//         ESP_LOGE(TAG, "HTTP 客户端初始化失败");
//         return ESP_FAIL;
//     }

//     esp_err_t err = esp_http_client_perform(client);
//     int status_code = esp_http_client_get_status_code(client);
//     esp_http_client_cleanup(client);

//     char weather_msg[128];

//     if (err != ESP_OK || (status_code != 200 && status_code != 0)) {
//         ESP_LOGW(TAG, "天气 API 请求失败: err=%s, status=%d（使用占位消息）",
//                  esp_err_to_name(err), status_code);
//         /*
//          * API 未接入时使用占位消息
//          * 接入后删除此分支，改用下方 JSON 解析结果
//          */
//         snprintf(weather_msg, sizeof(weather_msg),
//                  "现在为您播报%s的天气情况", s_ctx.weather_cfg.city_name);
//     } else {
//         ESP_LOGI(TAG, "天气 API 响应 (%d字节)", (int)resp.len);

//         /*
//          * ── JSON 解析逻辑（和风天气格式）──
//          *
//          * 响应格式：
//          *   { "code": "200", "now": { "temp": "25", "text": "晴" } }
//          *
//          * 接入步骤：
//          *   1. 在 reminder.h 中填入真实 WEATHER_API_KEY
//          *   2. 确认 WEATHER_DEFAULT_CITY 城市代码正确
//          *   3. 取消下方注释即可
//          */
//         /* ── 和风天气 JSON 解析（取消注释即用） ── */
//         /*
//         cJSON *root = cJSON_Parse(resp.buf);
//         if (root) {
//             cJSON *now_obj = cJSON_GetObjectItem(root, "now");
//             if (now_obj) {
//                 const char *text = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "text"));
//                 const char *temp = cJSON_GetStringValue(cJSON_GetObjectItem(now_obj, "temp"));
//                 if (text && temp) {
//                     snprintf(weather_msg, sizeof(weather_msg),
//                              "%s今天%s，当前气温%s度", s_ctx.weather_cfg.city_name, text, temp);
//                 } else {
//                     snprintf(weather_msg, sizeof(weather_msg),
//                              "%s天气数据解析异常", s_ctx.weather_cfg.city_name);
//                 }
//             }
//             cJSON_Delete(root);
//         } else {
//             snprintf(weather_msg, sizeof(weather_msg),
//                      "%s天气数据解析失败", s_ctx.weather_cfg.city_name);
//         }
//         */

//         /* 临时占位（API 接入后删除此行，启用上方解析） */
//         snprintf(weather_msg, sizeof(weather_msg),
//                  "现在为您播报%s的天气情况", s_ctx.weather_cfg.city_name);
//     }

//     /* 通过回调通知上层播报 */
//     if (s_ctx.trigger_cb) {
//         s_ctx.trigger_cb(REMINDER_TYPE_WEATHER, weather_msg, false);
//     }
//     return ESP_OK;
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 7. 轮询定时器回调（1 秒心跳 — 仅做检测和事件投递）
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief 1 秒轮询回调 — 提醒系统的心跳
//  *
//  * 在 esp_timer 任务上下文中执行（栈小，不可做耗时操作）。
//  * 仅检测条件是否满足，满足则投递事件到 evt_queue，
//  * 由 reminder_task 负责执行实际的播报/HTTP/震动操作。
//  *
//  * 检查顺序（体现优先级：闹钟 > 倒计时 > 日历 > 天气）：
//  *   1. 闹钟到点？
//  *   2. 倒计时到期？
//  *   3. 日历事件匹配？
//  *   4. 天气播报时段？
//  *
//  * 防重复机制：
//  *   闹钟和日历使用 last_xxx_check_min 记录上次触发的分钟数，
//  *   同一分钟内只触发一次，避免 60 次重复触发。
//  */
// static void poll_timer_callback(void *arg)
// {
//     reminder_evt_t evt = {0};

//     /* ── 倒计时检查（不依赖 SNTP，始终可用） ── */
//     {
//         int64_t now_us = esp_timer_get_time();

//         xSemaphoreTake(s_ctx.mutex, 0); // 非阻塞获取，失败则跳过本次
//         for (uint8_t i = 0; i < REMINDER_MAX_TIMERS; i++) {
//             timer_entry_t *t = &s_ctx.timers[i];
//             if (!t->active) {
//                 continue;
//             }
//             int64_t elapsed_sec = (now_us - t->start_time_us) / 1000000;
//             if (elapsed_sec >= (int64_t)t->duration_sec) {
//                 /* 倒计时到期 → 投递事件 */
//                 evt.type = REM_EVT_TIMER_EXPIRE;
//                 evt.id = i;
//                 strncpy(evt.message, t->message, REMINDER_MSG_MAX_LEN - 1);
//                 t->active = false; // 标记为已完成
//                 xQueueSend(s_ctx.evt_queue, &evt, 0);
//             }
//         }
//         xSemaphoreGive(s_ctx.mutex);
//     }

//     /* SNTP 未同步时，闹钟/日历/天气无法工作 */
//     if (!s_ctx.sntp_synced) {
//         return;
//     }

//     /* 闹钟正在响铃时，不检查新的提醒（闹钟优先级最高，独占） */
//     if (s_ctx.state == REMINDER_STATE_RINGING) {
//         return;
//     }

//     time_t now = time(NULL);
//     struct tm now_tm;
//     localtime_r(&now, &now_tm);

//     int current_min = now_tm.tm_hour * 60 + now_tm.tm_min;

//     /* ── 闹钟检查（最高优先级） ── */
//     if (current_min != s_ctx.last_alarm_check_min) {
//         xSemaphoreTake(s_ctx.mutex, 0);
//         for (uint8_t i = 0; i < s_ctx.alarm_count; i++) {
//             if (alarm_should_trigger(&s_ctx.alarms[i], &now_tm)) {
//                 evt.type = REM_EVT_ALARM_TRIGGER;
//                 evt.id = i;
//                 strncpy(evt.message, s_ctx.alarms[i].message, REMINDER_MSG_MAX_LEN - 1);
//                 xQueueSend(s_ctx.evt_queue, &evt, 0);
//                 s_ctx.last_alarm_check_min = current_min;
//                 xSemaphoreGive(s_ctx.mutex);
//                 return; // 闹钟最高优先级，触发后立即返回，不处理其他提醒
//             }
//         }
//         xSemaphoreGive(s_ctx.mutex);
//     }

//     /* ── 日历事件检查 ── */
//     if (current_min != s_ctx.last_cal_check_min) {
//         xSemaphoreTake(s_ctx.mutex, 0);
//         for (uint8_t i = 0; i < s_ctx.calendar_count; i++) {
//             calendar_entry_t *cal = &s_ctx.calendars[i];
//             if (!cal->enabled) {
//                 continue;
//             }
//             if (cal->year   == (uint16_t)(now_tm.tm_year + 1900) &&
//                 cal->month  == (uint8_t)(now_tm.tm_mon + 1) &&
//                 cal->day    == (uint8_t)now_tm.tm_mday &&
//                 cal->hour   == (uint8_t)now_tm.tm_hour &&
//                 cal->minute == (uint8_t)now_tm.tm_min) {

//                 evt.type = REM_EVT_CALENDAR_TRIGGER;
//                 evt.id = i;
//                 strncpy(evt.message, cal->message, REMINDER_MSG_MAX_LEN - 1);
//                 xQueueSend(s_ctx.evt_queue, &evt, 0);

//                 cal->enabled = false; // 日历事件为一次性
//                 s_ctx.last_cal_check_min = current_min;
//             }
//         }
//         xSemaphoreGive(s_ctx.mutex);
//     }

//     /* ── 天气播报检查（最低优先级） ── */
//     if (s_ctx.weather_cfg.schedule != WEATHER_SCHEDULE_DISABLED &&
//         s_ctx.state == REMINDER_STATE_IDLE) {

//         /* 跨日重置 */
//         if (now_tm.tm_mday != s_ctx.last_weather_day) {
//             s_ctx.weather_morning_done = false;
//             s_ctx.weather_evening_done = false;
//             s_ctx.last_weather_day = now_tm.tm_mday;
//         }

//         bool should_fetch = false;

//         /* 早间 */
//         if ((s_ctx.weather_cfg.schedule == WEATHER_SCHEDULE_MORNING ||
//              s_ctx.weather_cfg.schedule == WEATHER_SCHEDULE_BOTH) &&
//             !s_ctx.weather_morning_done &&
//             now_tm.tm_hour == WEATHER_MORNING_HOUR &&
//             now_tm.tm_min == WEATHER_MORNING_MINUTE &&
//             current_min != s_ctx.last_cal_check_min) {
//             should_fetch = true;
//             s_ctx.weather_morning_done = true;
//         }

//         /* 晚间 */
//         if ((s_ctx.weather_cfg.schedule == WEATHER_SCHEDULE_EVENING ||
//              s_ctx.weather_cfg.schedule == WEATHER_SCHEDULE_BOTH) &&
//             !s_ctx.weather_evening_done &&
//             now_tm.tm_hour == WEATHER_EVENING_HOUR &&
//             now_tm.tm_min == WEATHER_EVENING_MINUTE) {
//             should_fetch = true;
//             s_ctx.weather_evening_done = true;
//         }

//         if (should_fetch) {
//             evt.type = REM_EVT_WEATHER_FETCH;
//             xQueueSend(s_ctx.evt_queue, &evt, 0);
//         }
//     }
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 8. 提醒任务（所有耗时操作在此执行）
//  * ═══════════════════════════════════════════════════════════════════ */

// /**
//  * @brief 提醒系统主任务
//  *
//  * 从事件队列中读取事件，执行对应操作：
//  *   - ALARM_TRIGGER：启动响铃循环
//  *   - ALARM_RING_TICK：重复播报（超时检查）
//  *   - ALARM_DISMISS：停止响铃
//  *   - ALARM_SNOOZE：停止当前响铃，创建延迟倒计时
//  *   - TIMER_EXPIRE：播报倒计时到期
//  *   - CALENDAR_TRIGGER：播报日历事件
//  *   - WEATHER_FETCH：HTTP 请求天气并播报
//  *
//  * @param arg 未使用
//  */
// static void reminder_task(void *arg)
// {
//     reminder_evt_t evt;

//     ESP_LOGI(TAG, "提醒任务启动");

//     while (1) {
//         /* 阻塞等待事件（无超时，省电） */
//         if (xQueueReceive(s_ctx.evt_queue, &evt, portMAX_DELAY) != pdTRUE) {
//             continue;
//         }

//         switch (evt.type) {

//         /* ── 闹钟触发 ── */
//         case REM_EVT_ALARM_TRIGGER:
//             ESP_LOGW(TAG, ">>> 闹钟 #%d 触发: %s <<<", evt.id, evt.message);
//             alarm_ring_start(evt.id, evt.message);
//             break;

//         /* ── 闹钟响铃循环（每 5 秒） ── */
//         case REM_EVT_ALARM_RING_TICK:
//             if (s_ctx.state != REMINDER_STATE_RINGING) {
//                 break; // 已被关闭，忽略残留事件
//             }

//             s_ctx.ring_count++;

//             /* 超时自动关闭 */
//             if (s_ctx.ring_count >= ALARM_RING_MAX_COUNT) {
//                 ESP_LOGW(TAG, "闹钟 #%d 响铃超时（%d 次），自动关闭",
//                          s_ctx.ringing_alarm_id, s_ctx.ring_count);
//                 alarm_ring_stop();
//                 break;
//             }

//             /* 重复播报 */
//             ESP_LOGI(TAG, "闹钟 #%d 第 %d 次响铃",
//                      s_ctx.ringing_alarm_id, s_ctx.ring_count);
//             if (s_ctx.trigger_cb) {
//                 xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//                 if (s_ctx.ringing_alarm_id < s_ctx.alarm_count) {
//                     s_ctx.trigger_cb(REMINDER_TYPE_ALARM,
//                                      s_ctx.alarms[s_ctx.ringing_alarm_id].message,
//                                      true);
//                 }
//                 xSemaphoreGive(s_ctx.mutex);
//             }
//             break;

//         /* ── 用户关闭闹钟 ── */
//         case REM_EVT_ALARM_DISMISS:
//             ESP_LOGI(TAG, "用户关闭闹钟");
//             alarm_ring_stop();
//             /* 通知上层"闹钟已关闭"（可选播报） */
//             if (s_ctx.trigger_cb) {
//                 s_ctx.trigger_cb(REMINDER_TYPE_ALARM, "闹钟已关闭", false);
//             }
//             break;

//         /* ── 贪睡 ── */
//         case REM_EVT_ALARM_SNOOZE: {
//             ESP_LOGI(TAG, "闹钟贪睡 %d 分钟", evt.snooze_min);
//             alarm_ring_stop();

//             /* 创建一个倒计时作为贪睡提醒 */
//             xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//             char snooze_msg[REMINDER_MSG_MAX_LEN];
//             if (s_ctx.ringing_alarm_id < s_ctx.alarm_count) {
//                 snprintf(snooze_msg, sizeof(snooze_msg), "贪睡提醒：%s",
//                          s_ctx.alarms[s_ctx.ringing_alarm_id].message);
//             } else {
//                 strncpy(snooze_msg, "贪睡提醒时间到", sizeof(snooze_msg) - 1);
//             }
//             xSemaphoreGive(s_ctx.mutex);

//             reminder_timer_start(evt.snooze_min * 60, snooze_msg);

//             if (s_ctx.trigger_cb) {
//                 char ack_msg[64];
//                 snprintf(ack_msg, sizeof(ack_msg), "好的，%d分钟后再提醒你", evt.snooze_min);
//                 s_ctx.trigger_cb(REMINDER_TYPE_ALARM, ack_msg, false);
//             }
//             break;
//         }

//         /* ── 倒计时到期 ── */
//         case REM_EVT_TIMER_EXPIRE:
//             ESP_LOGI(TAG, "倒计时 #%d 到期: %s", evt.id, evt.message);
//             s_ctx.state = REMINDER_STATE_NOTIFYING;
//             if (s_ctx.trigger_cb) {
//                 s_ctx.trigger_cb(REMINDER_TYPE_TIMER, evt.message, false);
//             }
//             s_ctx.state = REMINDER_STATE_IDLE;
//             break;

//         /* ── 日历事件触发 ── */
//         case REM_EVT_CALENDAR_TRIGGER:
//             ESP_LOGI(TAG, "日历事件 #%d 触发: %s", evt.id, evt.message);
//             s_ctx.state = REMINDER_STATE_NOTIFYING;
//             if (s_ctx.trigger_cb) {
//                 s_ctx.trigger_cb(REMINDER_TYPE_CALENDAR, evt.message, false);
//             }
//             /* 持久化已禁用的日历事件 */
//             nvs_save_calendars();
//             s_ctx.state = REMINDER_STATE_IDLE;
//             break;

//         /* ── 天气播报 ── */
//         case REM_EVT_WEATHER_FETCH:
//             ESP_LOGI(TAG, "执行天气播报");
//             s_ctx.state = REMINDER_STATE_NOTIFYING;
//             weather_fetch_and_notify();
//             s_ctx.state = REMINDER_STATE_IDLE;
//             break;

//         default:
//             ESP_LOGW(TAG, "未知事件类型: %d", evt.type);
//             break;
//         }
//     }
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 9. 对外接口 — 系统生命周期
//  * ═══════════════════════════════════════════════════════════════════ */

// esp_err_t reminder_init(reminder_trigger_cb_t cb)
// {
//     if (s_ctx.initialized) {
//         ESP_LOGW(TAG, "提醒系统已初始化，跳过");
//         return ESP_OK;
//     }
//     if (cb == NULL) {
//         ESP_LOGE(TAG, "触发回调不能为 NULL");
//         return ESP_ERR_INVALID_ARG;
//     }

//     ESP_LOGI(TAG, "初始化提醒系统...");
//     memset(&s_ctx, 0, sizeof(s_ctx));
//     s_ctx.trigger_cb = cb;
//     s_ctx.last_alarm_check_min = -1;
//     s_ctx.last_cal_check_min = -1;

//     /* 创建互斥锁 */
//     s_ctx.mutex = xSemaphoreCreateMutex();
//     if (s_ctx.mutex == NULL) {
//         ESP_LOGE(TAG, "互斥锁创建失败");
//         return ESP_FAIL;
//     }

//     /* 创建事件队列（容量 10，足够缓冲突发事件） */
//     s_ctx.evt_queue = xQueueCreate(10, sizeof(reminder_evt_t));
//     if (s_ctx.evt_queue == NULL) {
//         ESP_LOGE(TAG, "事件队列创建失败");
//         vSemaphoreDelete(s_ctx.mutex);
//         return ESP_FAIL;
//     }

//     /* 从 NVS 加载持久化数据 */
//     nvs_load_alarms();
//     nvs_load_calendars();
//     nvs_load_weather_config();

//     /* 初始化 SNTP */
//     sntp_time_sync_init();

//     /* 创建提醒任务（8KB 栈，优先级 3，运行在任意核心） */
//     BaseType_t ret = xTaskCreate(
//         reminder_task,
//         "reminder_task",
//         8192,
//         NULL,
//         3,           // 优先级 3（低于音频任务的 5，高于空闲任务）
//         &s_ctx.task_handle
//     );
//     if (ret != pdPASS) {
//         ESP_LOGE(TAG, "提醒任务创建失败");
//         vQueueDelete(s_ctx.evt_queue);
//         vSemaphoreDelete(s_ctx.mutex);
//         return ESP_FAIL;
//     }

//     /* 创建并启动 1 秒轮询定时器 */
//     esp_timer_create_args_t timer_args = {
//         .callback = poll_timer_callback,
//         .name = "reminder_poll",
//     };
//     esp_err_t err = esp_timer_create(&timer_args, &s_ctx.poll_timer);
//     if (err != ESP_OK) {
//         ESP_LOGE(TAG, "轮询定时器创建失败");
//         return err;
//     }
//     esp_timer_start_periodic(s_ctx.poll_timer, 1000000); // 1 秒

//     s_ctx.initialized = true;
//     ESP_LOGI(TAG, "提醒系统初始化完成");
//     return ESP_OK;
// }

// void reminder_deinit(void)
// {
//     if (!s_ctx.initialized) {
//         return;
//     }

//     if (s_ctx.poll_timer) {
//         esp_timer_stop(s_ctx.poll_timer);
//         esp_timer_delete(s_ctx.poll_timer);
//     }
//     if (s_ctx.ring_timer) {
//         esp_timer_stop(s_ctx.ring_timer);
//         esp_timer_delete(s_ctx.ring_timer);
//     }
//     if (s_ctx.task_handle) {
//         vTaskDelete(s_ctx.task_handle);
//     }
//     if (s_ctx.evt_queue) {
//         vQueueDelete(s_ctx.evt_queue);
//     }
//     if (s_ctx.mutex) {
//         vSemaphoreDelete(s_ctx.mutex);
//     }

//     esp_sntp_stop();
//     s_ctx.initialized = false;
//     ESP_LOGI(TAG, "提醒系统已销毁");
// }

// reminder_state_t reminder_get_state(void)
// {
//     return s_ctx.state;
// }

// bool reminder_is_time_synced(void)
// {
//     return s_ctx.sntp_synced;
// }

// bool reminder_get_current_time(uint8_t *hour, uint8_t *minute, uint8_t *second)
// {
//     if (!s_ctx.sntp_synced) {
//         if (hour)   *hour = 0;
//         if (minute) *minute = 0;
//         if (second) *second = 0;
//         return false;
//     }

//     time_t now = time(NULL);
//     struct tm now_tm;
//     localtime_r(&now, &now_tm);

//     if (hour)   *hour   = (uint8_t)now_tm.tm_hour;
//     if (minute) *minute = (uint8_t)now_tm.tm_min;
//     if (second) *second = (uint8_t)now_tm.tm_sec;
//     return true;
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 10. 对外接口 — 闹钟
//  * ═══════════════════════════════════════════════════════════════════ */

// int reminder_alarm_add(const alarm_entry_t *entry)
// {
//     if (entry == NULL) {
//         return -1;
//     }

//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     if (s_ctx.alarm_count >= REMINDER_MAX_ALARMS) {
//         ESP_LOGW(TAG, "闹钟已满 (%d)", REMINDER_MAX_ALARMS);
//         xSemaphoreGive(s_ctx.mutex);
//         return -1;
//     }

//     uint8_t new_id = s_ctx.alarm_count;
//     s_ctx.alarms[new_id] = *entry;
//     s_ctx.alarms[new_id].id = new_id;
//     s_ctx.alarms[new_id].enabled = true;
//     s_ctx.alarm_count++;

//     nvs_save_alarms();
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "添加闹钟 #%d: %02d:%02d [%s]",
//              new_id, entry->hour, entry->minute, entry->message);
//     return new_id;
// }

// esp_err_t reminder_alarm_delete(uint8_t alarm_id)
// {
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     if (alarm_id >= s_ctx.alarm_count) {
//         xSemaphoreGive(s_ctx.mutex);
//         return ESP_ERR_NOT_FOUND;
//     }

//     /* 前移覆盖 */
//     for (uint8_t i = alarm_id; i < s_ctx.alarm_count - 1; i++) {
//         s_ctx.alarms[i] = s_ctx.alarms[i + 1];
//         s_ctx.alarms[i].id = i;
//     }
//     s_ctx.alarm_count--;

//     nvs_save_alarms();
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "删除闹钟 #%d", alarm_id);
//     return ESP_OK;
// }

// esp_err_t reminder_alarm_set_enabled(uint8_t alarm_id, bool enabled)
// {
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     if (alarm_id >= s_ctx.alarm_count) {
//         xSemaphoreGive(s_ctx.mutex);
//         return ESP_ERR_NOT_FOUND;
//     }

//     s_ctx.alarms[alarm_id].enabled = enabled;
//     nvs_save_alarms();
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "闹钟 #%d %s", alarm_id, enabled ? "启用" : "禁用");
//     return ESP_OK;
// }

// void reminder_alarm_get_all(alarm_entry_t *out_list, uint8_t *out_count)
// {
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//     if (out_list) {
//         memcpy(out_list, s_ctx.alarms, sizeof(alarm_entry_t) * s_ctx.alarm_count);
//     }
//     if (out_count) {
//         *out_count = s_ctx.alarm_count;
//     }
//     xSemaphoreGive(s_ctx.mutex);
// }

// esp_err_t reminder_alarm_dismiss(void)
// {
//     if (s_ctx.state != REMINDER_STATE_RINGING) {
//         ESP_LOGW(TAG, "当前无响铃闹钟，忽略关闭指令");
//         return ESP_ERR_NOT_FOUND;
//     }

//     reminder_evt_t evt = {
//         .type = REM_EVT_ALARM_DISMISS,
//     };
//     xQueueSend(s_ctx.evt_queue, &evt, pdMS_TO_TICKS(100));
//     return ESP_OK;
// }

// esp_err_t reminder_alarm_snooze(uint8_t snooze_minutes)
// {
//     if (s_ctx.state != REMINDER_STATE_RINGING) {
//         return ESP_ERR_NOT_FOUND;
//     }
//     if (snooze_minutes == 0) {
//         snooze_minutes = 5; // 默认 5 分钟
//     }

//     reminder_evt_t evt = {
//         .type = REM_EVT_ALARM_SNOOZE,
//         .snooze_min = snooze_minutes,
//     };
//     xQueueSend(s_ctx.evt_queue, &evt, pdMS_TO_TICKS(100));
//     return ESP_OK;
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 11. 对外接口 — 倒计时
//  * ═══════════════════════════════════════════════════════════════════ */

// int reminder_timer_start(uint32_t duration_sec, const char *message)
// {
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     int slot = -1;
//     for (uint8_t i = 0; i < REMINDER_MAX_TIMERS; i++) {
//         if (!s_ctx.timers[i].active) {
//             slot = i;
//             break;
//         }
//     }

//     if (slot < 0) {
//         ESP_LOGW(TAG, "倒计时已满 (%d)", REMINDER_MAX_TIMERS);
//         xSemaphoreGive(s_ctx.mutex);
//         return -1;
//     }

//     s_ctx.timers[slot].id = (uint8_t)slot;
//     s_ctx.timers[slot].active = true;
//     s_ctx.timers[slot].duration_sec = duration_sec;
//     s_ctx.timers[slot].start_time_us = esp_timer_get_time();
//     strncpy(s_ctx.timers[slot].message,
//             message ? message : "倒计时到了",
//             REMINDER_MSG_MAX_LEN - 1);

//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "启动倒计时 #%d: %lu秒 [%s]",
//              slot, (unsigned long)duration_sec, s_ctx.timers[slot].message);
//     return slot;
// }

// esp_err_t reminder_timer_cancel(uint8_t timer_id)
// {
//     if (timer_id >= REMINDER_MAX_TIMERS) {
//         return ESP_ERR_NOT_FOUND;
//     }

//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//     if (!s_ctx.timers[timer_id].active) {
//         xSemaphoreGive(s_ctx.mutex);
//         return ESP_ERR_NOT_FOUND;
//     }
//     s_ctx.timers[timer_id].active = false;
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "取消倒计时 #%d", timer_id);
//     return ESP_OK;
// }

// esp_err_t reminder_timer_get_remain(uint8_t timer_id, uint32_t *out_remain)
// {
//     if (timer_id >= REMINDER_MAX_TIMERS || out_remain == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//     if (!s_ctx.timers[timer_id].active) {
//         xSemaphoreGive(s_ctx.mutex);
//         return ESP_ERR_NOT_FOUND;
//     }

//     int64_t elapsed = (esp_timer_get_time() - s_ctx.timers[timer_id].start_time_us) / 1000000;
//     int64_t remain = (int64_t)s_ctx.timers[timer_id].duration_sec - elapsed;
//     *out_remain = (remain > 0) ? (uint32_t)remain : 0;

//     xSemaphoreGive(s_ctx.mutex);
//     return ESP_OK;
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 12. 对外接口 — 日历
//  * ═══════════════════════════════════════════════════════════════════ */

// int reminder_calendar_add(const calendar_entry_t *entry)
// {
//     if (entry == NULL) {
//         return -1;
//     }

//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     if (s_ctx.calendar_count >= REMINDER_MAX_CALENDARS) {
//         ESP_LOGW(TAG, "日历事件已满 (%d)", REMINDER_MAX_CALENDARS);
//         xSemaphoreGive(s_ctx.mutex);
//         return -1;
//     }

//     uint8_t new_id = s_ctx.calendar_count;
//     s_ctx.calendars[new_id] = *entry;
//     s_ctx.calendars[new_id].id = new_id;
//     s_ctx.calendars[new_id].enabled = true;
//     s_ctx.calendar_count++;

//     nvs_save_calendars();
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "添加日历事件 #%d: %04d-%02d-%02d %02d:%02d [%s]",
//              new_id, entry->year, entry->month, entry->day,
//              entry->hour, entry->minute, entry->message);
//     return new_id;
// }

// esp_err_t reminder_calendar_delete(uint8_t cal_id)
// {
//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);

//     if (cal_id >= s_ctx.calendar_count) {
//         xSemaphoreGive(s_ctx.mutex);
//         return ESP_ERR_NOT_FOUND;
//     }

//     for (uint8_t i = cal_id; i < s_ctx.calendar_count - 1; i++) {
//         s_ctx.calendars[i] = s_ctx.calendars[i + 1];
//         s_ctx.calendars[i].id = i;
//     }
//     s_ctx.calendar_count--;

//     nvs_save_calendars();
//     xSemaphoreGive(s_ctx.mutex);

//     ESP_LOGI(TAG, "删除日历事件 #%d", cal_id);
//     return ESP_OK;
// }

// void reminder_calendar_get_today(calendar_entry_t *out_list, uint8_t *out_count)
// {
//     if (!s_ctx.sntp_synced) {
//         if (out_count) *out_count = 0;
//         return;
//     }

//     time_t now = time(NULL);
//     struct tm now_tm;
//     localtime_r(&now, &now_tm);
//     uint8_t count = 0;

//     xSemaphoreTake(s_ctx.mutex, portMAX_DELAY);
//     for (uint8_t i = 0; i < s_ctx.calendar_count; i++) {
//         calendar_entry_t *cal = &s_ctx.calendars[i];
//         if (cal->enabled &&
//             cal->year  == (uint16_t)(now_tm.tm_year + 1900) &&
//             cal->month == (uint8_t)(now_tm.tm_mon + 1) &&
//             cal->day   == (uint8_t)now_tm.tm_mday) {
//             if (out_list) {
//                 out_list[count] = *cal;
//             }
//             count++;
//         }
//     }
//     xSemaphoreGive(s_ctx.mutex);

//     if (out_count) *out_count = count;
// }

// /* ═══════════════════════════════════════════════════════════════════
//  * 13. 对外接口 — 天气
//  * ═══════════════════════════════════════════════════════════════════ */

// esp_err_t reminder_weather_config(const weather_config_t *config)
// {
//     if (config == NULL) {
//         return ESP_ERR_INVALID_ARG;
//     }

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

// esp_err_t reminder_weather_fetch_now(void)
// {
//     reminder_evt_t evt = {
//         .type = REM_EVT_WEATHER_FETCH,
//     };
//     return xQueueSend(s_ctx.evt_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE
//            ? ESP_OK : ESP_FAIL;
// }
