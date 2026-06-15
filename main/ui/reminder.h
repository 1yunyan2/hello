#pragma once

/**
 * @file reminder.h
 * @brief 提醒系统总控接口 — 闹钟/倒计时/日历/天气 四大提醒功能
 *
 * 架构设计：
 *   ┌─────────────────────────────────────────────────────┐
 *   │               提醒系统 (reminder.c)                  │
 *   │                                                      │
 *   │  ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌────────┐ │
 *   │  │ 闹钟模块 │ │ 倒计时   │ │ 日历事件 │ │ 天气   │ │
 *   │  │(alarm)   │ │(timer)   │ │(calendar)│ │(weather)│ │
 *   │  └────┬─────┘ └────┬─────┘ └────┬─────┘ └───┬────┘ │
 *   │       └──────┬──────┴──────┬─────┘            │      │
 *   │              ▼             ▼                  ▼      │
 *   │     【优先级调度器】    【NVS持久化】  【HTTP任务】   │
 *   │              │                                       │
 *   │              ▼                                       │
 *   │     trigger_cb → session 层 TTS 播报 + 情绪动作      │
 *   └─────────────────────────────────────────────────────┘
 *
 * 优先级体系（高→低）：
 *   1. ALARM  — 闹钟（最高优先级，可打断其他提醒和空闲状态）
 *   2. TIMER  — 倒计时到期
 *   3. CALENDAR — 日历事件
 *   4. WEATHER — 天气播报（最低优先级，有其他提醒时延后）
 *
 * 闹钟响铃与关闭：
 *   - 闹钟触发后进入 RINGING 状态，持续震动+动作+语音播报
 *   - 用户通过语音"关闭闹钟/停止"关闭，或超时自动关闭
 *   - RINGING 期间不响应其他低优先级提醒
 *
 * 时间显示：
 *   - SNTP 同步后，LCD 持续显示当前时间
 *   - 提醒触发时 LCD 切换显示提醒内容
 */

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include "esp_err.h"

/* ═══════════════════════════════════════════════════════════════════
 * 1. 常量与配置宏
 * ═══════════════════════════════════════════════════════════════════ */

#define REMINDER_MAX_ALARMS 1     ///< 最大闹钟数量，后续使用条件编译将多个变为？个
#define REMINDER_MAX_TIMERS 1     /// todo<最大倒计时数量，后续使用条件编译将多倒计时变为单个，为什么会有倒计时？计算闹钟的吗？
#define REMINDER_MAX_CALENDARS 16 ///< 最大日历事件数量
#define REMINDER_MSG_MAX_LEN 64   ///< 提醒消息最大长度（UTF-8 字节）

/* ── 闹钟响铃参数 ── */
#define ALARM_RING_INTERVAL_MS 5000 ///< 闹钟响铃间隔（每 5 秒重复播报一次）
#define ALARM_RING_MAX_COUNT 3      // 时间超过这个次数还没关闭，就自动关闭（15秒）
#define ALARM_RING_TIMEOUT_SEC 30   ///< 闹钟响铃超时（秒），超时自动关闭

/* ── 天气拉取时间（宏定义，修改此处即可调整播报时段） ── */
#define WEATHER_MORNING_HOUR 7         ///< 早间天气播报 — 小时（24h 制）
#define WEATHER_MORNING_MINUTE 30      ///< 早间天气播报 — 分钟
#define WEATHER_EVENING_HOUR 19        ///< 晚间天气播报 — 小时
#define WEATHER_EVENING_MINUTE 0       ///< 晚间天气播报 — 分钟
#define WEATHER_FETCH_TIMEOUT_MS 10000 ///< 天气 HTTP 请求超时（毫秒）

/* ── 白天天气自动拉取时间点（4个固定时间点，每4小时一次，以分钟总数表示）──
 * 07:30 / 11:30 / 15:30 / 19:30，用 tm_hour*60+tm_min 直接比较
 * 上电后额外立即拉取一次，不依赖这些时间点 */
#define WEATHER_FETCH_MIN_0 (WEATHER_MORNING_HOUR * 60 + WEATHER_MORNING_MINUTE) ///< 07:30
#define WEATHER_FETCH_MIN_1 (WEATHER_FETCH_MIN_0 + 4 * 60)                       ///< 11:30
#define WEATHER_FETCH_MIN_2 (WEATHER_FETCH_MIN_0 + 8 * 60)                       ///< 15:30
#define WEATHER_FETCH_MIN_3 (WEATHER_EVENING_HOUR * 60 + WEATHER_EVENING_MINUTE) ///< 19:30

/* ═══════════════════════════════════════════════════════════════════
 * 天气 API 切换开关
 *   WEATHER_PROVIDER = 0 → 和风天气（免费版允许商用，每月 5 万次免费额度）
 *   WEATHER_PROVIDER = 1 → 心知天气（免费版禁商用，仅原型/个人使用）
 * 切换方式：只改下面这一个宏值，重新编译即可。
 * 底层 gzip 解压 / Location ID 查询 / JSON 解析全部由
 * managed_components/ningzixi__weather 组件完成，本层无需关心。
 * ═══════════════════════════════════════════════════════════════════ */
#define WEATHER_PROVIDER 0 ///< 默认心知；改成 0 切换到和风

#if WEATHER_PROVIDER == 0
/* ---- 和风天气 ---- */
#define WEATHER_API_KEY "8b3ac5056d6e454fb65a5ece41d2f62f"
/* 认证开发者专属域名（注意：不是免费版的 devapi.qweather.com）。
 * 组件会用它同时拼天气查询和 GeoAPI 城市查询两个请求。 */
#define WEATHER_API_HOST "nj6tuqw55f.re.qweatherapi.com"
#define WEATHER_DEFAULT_CITY "北京" ///< 组件用中文城市名查 Location ID
#else
/* ---- 心知天气 ---- */
#define WEATHER_API_KEY "SDmjR5pLP_1OT23gb"
#define WEATHER_API_HOST NULL       ///< 心知不需要 host
#define WEATHER_DEFAULT_CITY "北京" ///< 心知直接用城市名查询
#endif

/* ═══════════════════════════════════════════════════════════════════
 * 2. 枚举定义
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 提醒类型枚举（同时作为优先级，值越小优先级越高）
 */
typedef enum
{
    REMINDER_TYPE_ALARM = 0,    ///< 闹钟（最高优先级，可打断一切）
    REMINDER_TYPE_TIMER = 1,    ///< 倒计时到期
    REMINDER_TYPE_CALENDAR = 2, ///< 日历事件
    REMINDER_TYPE_WEATHER = 3,  ///< 天气播报（最低优先级）
} reminder_type_t;

/**
 * @brief 闹钟重复模式
 */
typedef enum
{
    ALARM_REPEAT_ONCE = 0, ///< 仅响一次（响完自动禁用）
    ALARM_REPEAT_DAILY,    ///< 每天重复
    ALARM_REPEAT_WEEKDAY,  ///< 工作日重复（周一~周五）
    ALARM_REPEAT_WEEKEND,  ///< 周末重复（周六~周日）
    ALARM_REPEAT_CUSTOM,   ///< 自定义（按 weekday_mask 位图）
} alarm_repeat_t;

/**
 * @brief 提醒系统当前状态
 */
typedef enum
{
    REMINDER_STATE_IDLE = 0,  ///< 空闲（正常显示时间）
    REMINDER_STATE_RINGING,   ///< 闹钟响铃中（等待用户语音关闭或超时）
    REMINDER_STATE_NOTIFYING, ///< 正在播报提醒（倒计时/日历/天气，播完自动回 IDLE）
} reminder_state_t;

/**
 * @brief 天气播报时段配置
 */
typedef enum
{
    WEATHER_SCHEDULE_MORNING = 0, ///< 仅早间播报
    WEATHER_SCHEDULE_EVENING,     ///< 仅晚间播报
    WEATHER_SCHEDULE_BOTH,        ///< 早晚各一次
    WEATHER_SCHEDULE_DISABLED,    ///< 关闭天气播报
} weather_schedule_t;

/* ═══════════════════════════════════════════════════════════════════
 * 3. 数据结构
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 闹钟条目
 *
 * NVS 存储 key: "alarm_0" ~ "alarm_7"
 */
typedef struct
{
    uint8_t id;                         ///< 闹钟 ID（0 ~ REMINDER_MAX_ALARMS-1）
    bool enabled;                       ///< 是否启用
    uint8_t hour;                       ///< 小时（0~23）
    uint8_t minute;                     ///< 分钟（0~59）
    alarm_repeat_t repeat;              ///< 重复模式
    uint8_t weekday_mask;               ///< 自定义重复的星期位图（bit0=周日, bit1=周一...）
    char message[REMINDER_MSG_MAX_LEN]; ///< 语音提醒内容（如"该起床了"）
} alarm_entry_t;

/**
 * @brief 倒计时条目（一次性，到期后自动清除）
 */
typedef struct
{
    uint8_t id;                         ///< 倒计时 ID（0 ~ REMINDER_MAX_TIMERS-1）
    bool active;                        ///< 是否正在计时
    uint32_t duration_sec;              ///< 总时长（秒）
    int64_t start_time_us;              ///< 开始时刻（esp_timer_get_time 微秒）
    char message[REMINDER_MSG_MAX_LEN]; ///< 到期提醒内容（如"水烧好了"）
} timer_entry_t;

/**
 * @brief 日历事件条目
 *
 * NVS 存储 key: "cal_00" ~ "cal_15"
 */
typedef struct
{
    uint8_t id;                         ///< 事件 ID
    bool enabled;                       ///< 是否启用
    uint16_t year;                      ///< 年份（如 2026）
    uint8_t month;                      ///< 月份（1~12）
    uint8_t day;                        ///< 日期（1~31）
    uint8_t hour;                       ///< 小时（0~23）
    uint8_t minute;                     ///< 分钟（0~59）
    char message[REMINDER_MSG_MAX_LEN]; ///< 事件描述（如"下午3点开会"）
} calendar_entry_t;

/**
 * @brief 天气配置
 */
// typedef struct
// {
//     weather_schedule_t schedule; ///< 播报时段
//     char city_code[16];          ///< 城市标识（和风="101010100"，心知="beijing"）
//     char city_name[32];          ///< 城市显示名称（用于语音播报，如"北京"）
// } weather_config_t;
typedef struct
{
    weather_schedule_t schedule; ///< 播报时段
    char city_code[16];          ///< 城市标识（和风="101010100"，心知="beijing"）
    char city_name[32];          ///< 城市显示名称（用于语音播报，如"北京"）
} reminder_weather_cfg_t;

/**
 * @brief 天气实时数据（由 weather_fetch_and_notify 解析后存储，供 UI 读取）
 */
typedef struct
{
    char city_name[32]; ///< 城市名（如"杭州"）
    char temp[8];       ///< 当前温度（如"25"）
    char text[32];      ///< 天气状况（如"晴"、"多云"）
    // char humidity[8];   ///< 湿度百分比（如"60"）
    bool valid; ///< 数据是否有效（true=已获取成功）
} weather_data_t;

/**
 * @brief 提醒触发回调函数类型
 *
 * 上层（session）注册此回调，提醒系统触发时调用。
 * 上层负责：TTS 语音播报 + 情绪动作触发 + LCD 显示切换
 *
 * @param type      提醒类型（用于判断优先级和播报方式）
 * @param message   提醒文字内容（用于 TTS 播报）
 * @param is_alarm_start  true=闹钟开始响铃（需要持续提醒直到关闭）
 *                        false=普通一次性提醒
 */
typedef void (*reminder_trigger_cb_t)(reminder_type_t type, const char *message, bool is_alarm_start);

/* ═══════════════════════════════════════════════════════════════════
 * 4. 对外接口 — 系统生命周期
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 初始化提醒系统
 *
 * 执行：SNTP 时间同步 → NVS 加载已存储提醒 → 启动轮询定时器 → 启动提醒任务
 *
 * @param cb 提醒触发回调（不可为 NULL）
 * @return ESP_OK / ESP_FAIL
 */
esp_err_t reminder_init(reminder_trigger_cb_t cb);

/**
 * @brief 销毁提醒系统（释放所有资源）
 */
void reminder_deinit(void);

/**
 * @brief 获取提醒系统当前状态
 * @return IDLE / RINGING / NOTIFYING
 */
reminder_state_t reminder_get_state(void);

/**
 * @brief 查询 SNTP 是否已同步（系统时间是否有效）
 * @return true=时间有效，false=尚未同步
 */
bool reminder_is_time_synced(void);

/**
 * @brief 获取当前本地时间（供 LCD 显示用）
 *
 * SNTP 未同步时返回 false，hour/minute/second 输出为 0。
 *
 * @param[out] hour    小时（0~23）
 * @param[out] minute  分钟（0~59）
 * @param[out] second  秒（0~59）
 * @return true=时间有效 / false=SNTP 未同步
 */
bool reminder_get_current_time(uint8_t *hour, uint8_t *minute, uint8_t *second);

/* ═══════════════════════════════════════════════════════════════════
 * 5. 对外接口 — 闹钟操作
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 添加闹钟
 * @param entry 闹钟条目（id 字段由内部分配）
 * @return 分配的 ID（0~7），失败返回 -1
 */
int reminder_alarm_add(const alarm_entry_t *entry);

/**
 * @brief 删除闹钟
 * @return ESP_OK / ESP_ERR_NOT_FOUND
 */
esp_err_t reminder_alarm_delete(uint8_t alarm_id);

/**
 * @brief 启用/禁用闹钟
 */
esp_err_t reminder_alarm_set_enabled(uint8_t alarm_id, bool enabled);
esp_err_t reminder_alarm_update(uint8_t alarm_id, const alarm_entry_t *entry);

/**
 * @brief 获取所有闹钟
 * @param out_list  输出数组（长度 >= REMINDER_MAX_ALARMS）
 * @param out_count 输出实际数量
 */
void reminder_alarm_get_all(alarm_entry_t *out_list, uint8_t *out_count);

/**
 * @brief 关闭当前正在响铃的闹钟（语音指令"关闭闹钟"时调用）
 *
 * 用户语音说"关闭"/"停止"/"好了"时，session 层解析后调用此函数。
 * 如果当前没有闹钟在响铃，调用无副作用。
 *
 * @return ESP_OK=成功关闭 / ESP_ERR_NOT_FOUND=当前无响铃闹钟
 */
esp_err_t reminder_alarm_dismiss(void);

/* ═══════════════════════════════════════════════════════════════════
 * 6. 对外接口 — 倒计时操作
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 启动倒计时
 * @param duration_sec 倒计时秒数
 * @param message      到期提醒文字
 * @return timer ID / -1 失败
 */
int reminder_timer_start(uint32_t duration_sec, const char *message);

/**
 * @brief 取消倒计时
 */
esp_err_t reminder_timer_cancel(uint8_t timer_id);

/**
 * @brief 查询剩余秒数
 */
esp_err_t reminder_timer_get_remain(uint8_t timer_id, uint32_t *out_remain);

/* ═══════════════════════════════════════════════════════════════════
 * 7. 对外接口 — 日历事件操作
 * ═══════════════════════════════════════════════════════════════════ */

int reminder_calendar_add(const calendar_entry_t *entry);
esp_err_t reminder_calendar_delete(uint8_t cal_id);
void reminder_calendar_get_today(calendar_entry_t *out_list, uint8_t *out_count);

/* ═══════════════════════════════════════════════════════════════════
 * 8. 对外接口 — 天气播报
 * ═══════════════════════════════════════════════════════════════════ */

/**
 * @brief 配置天气播报参数
 */
// esp_err_t reminder_weather_config(const weather_config_t *config);
esp_err_t reminder_weather_config(const reminder_weather_cfg_t *config);

/**
 * @brief 手动触发天气播报（用户语音"今天天气怎么样"时调用）
 */
esp_err_t reminder_weather_fetch_now(void);

/**
 * @brief IP 自动定位城市（WiFi 连接后调用）
 *
 * 流程：请求 ip-api.com 获取城市名 → 和风 GeoAPI 查城市代码 → 写入 weather_cfg
 * 失败时保留默认城市（北京），不影响天气功能。
 *
 * @return ESP_OK 定位成功 / ESP_FAIL 定位失败（保留默认城市）
 */
esp_err_t reminder_auto_locate_city(void);

/**
 * @brief 获取当前天气实时数据（供 UI 显示）
 *
 * @param[out] data  天气数据输出
 * @return ESP_OK 数据有效 / ESP_FAIL 数据尚未获取
 */
esp_err_t reminder_get_weather_data(weather_data_t *data);
