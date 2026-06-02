/**
 * @file standby.c
 * @brief 无活动待机（省电）模块实现
 *
 * 核心是一个 1s 周期任务 standby_task：
 *   - 每秒读取会话状态，若会话活跃（非 IDLE）则视为活动，刷新时间戳；
 *   - 距上次活动超过 STANDBY_LIGHT_TIMEOUT_MS 且当前未待机 → 进入待机；
 *   - 待机中每隔 SWING_PERIOD_MS 提交一次头部舵机左右摆动动作。
 *
 * 所有共享状态用临界区/原子读写保护，时间戳用 esp_timer_get_time（单调微秒时钟）。
 */
#include "standby.h"
#include "bsp/bsp_board.h"
#include "bsp/servo_manager.h"
#include "wake_word/custom_wake_word.h"
#include "session/session.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STANDBY";

// ─── 可调参数（时间阈值做成宏，方便后续调整）─────────────────────────────
// 两级待机时间均从「上次活动」起算（绝对计时，进入二级不清空一级计时）：
//   - 一级（轻度）：LIGHT_TIMEOUT_MS 后降亮度 + 关语音检测 + 头部摆动。
//   - 二级（深度）：DEEP_TIMEOUT_MS 后进一步省电（light/deep sleep）。
//     ★ 二级目前仅预留宏，逻辑尚未实现，待一级验证后再做。
#define STANDBY_LIGHT_TIMEOUT_MS 60000    ///< 一级（轻度）待机阈值：1 分钟无活动
#define STANDBY_DEEP_TIMEOUT_MS 300000    ///< 二级（深度）待机阈值：5 分钟无活动（绝对计时，从上次活动起算）
#define STANDBY_CHECK_MS 1000             ///< 周期任务检查间隔（1 秒）
#define SWING_PERIOD_MS 2500              ///< 待机摆头周期（每 2.5s 换一次方向）
#define HEAD_CENTER_DEG 90.0f             ///< 头部中位角度
#define HEAD_SWING_DEG 30.0f              ///< ★摆动幅度（±x°，即 (90-x)°~(90+x)°，后续直接改这里）
#define SWING_SPEED SERVO_SPEED_VERY_SLOW ///< ★摆头速度（取最慢档 50ms/度，后续可换 SLOW 等档位）

// ─── 二级（深度）待机配置 ───────────────────────────────────────────────────
// ⚠️⚠️⚠️ 二级深度待机目前仅搭好「框架 + 占位」，未启用真实睡眠 API。
//   原因：睡眠/电源管理代码是高风险项，配置不当会导致设备醒不过来或反复重启，
//   必须充分理解后再逐项启用。当前 enter_deep_standby() 只打印日志，不真正睡眠。
//
// 两套候选方案（启用时二选一，见 enter_deep_standby 内注释）：
//   方案 1 — light sleep：CPU 暂停但 RAM 保留，醒来就地继续，联网恢复快，省电中等。
//   方案 2 — deep sleep ：最省电（μA 级），但醒来等于软重启，WiFi/WS/模型全部重连
//                         （约 2~3s），睡眠期间断网、收不到服务端推送。
// 唤醒源：头部触摸铜箔（TTP223 → GPIO 中断），与一级「摸头唤醒」一致。
#define STANDBY_DEEP_ENABLE 0          ///< 二级深度待机总开关（0=禁用占位，1=启用，暂保持 0）
#define STANDBY_DEEP_USE_LIGHT_SLEEP 1 ///< 1=方案1 light sleep，0=方案2 deep sleep（启用后生效）

// ─── 模块状态 ───────────────────────────────────────────────────────────────
// s_last_active_us：上次活动时间戳（微秒，esp_timer 单调时钟）
// s_standby：当前是否待机。均用 volatile + 简单读写，配合 64 位读取注意原子性。
static volatile int64_t s_last_active_us = 0;
static volatile bool s_standby = false;      // 是否处于一级（轻度）待机
static volatile bool s_deep_standby = false; // 是否已触发二级（深度）待机（占位，防重复触发）
static volatile bool s_inited = false;

/**
 * @brief 刷新「上次活动」时间戳（内部）
 */
static inline void touch_activity(void)
{
    s_last_active_us = esp_timer_get_time();
}

void standby_notify_activity(void)
{
    // 任意活动统一收口到这里：刷新活动时间，且若正处于待机则一并退出待机。
    // standby_wake() 内部先 touch_activity() 再判断 !s_standby 直接返回，
    // 所以非待机时这里等价于「只刷计时」，待机时则完整恢复（亮度/监听/头部归中）。
    // 这样头部/腹背/左右翻页（耳朵）任意触摸、以及对话会话激活，都能退出待机。
    standby_wake();
}

bool standby_is_active(void)
{
    return s_standby;
}

/**
 * @brief 进入待机：降亮度 + 关监听 + 置标志（摆头由周期任务负责）
 */
static void enter_standby(void)
{
    if (s_standby)
        return;
    ESP_LOGI(TAG, "无活动超过 %d ms，进入待机", STANDBY_LIGHT_TIMEOUT_MS);
    s_standby = true;
    bsp_board_lcd_set_brightness(BSP_LCD_BK_STANDBY_PCT); // 背光降到 50%
    wake_word_stop();                                     // 关闭唤醒词监听（仅切 is_running）

    // 左右臂归中端正：保证待机姿态统一，且会排在残留情绪动作之后执行，把臂部摆正。
    // 头部不在此归中——由周期任务接管后开始慢摆。
    servo_manager_submit_angle(CH_L_ARM, HEAD_CENTER_DEG, SERVO_SPEED_SLOW); // 左臂归中 90°
    servo_manager_submit_angle(CH_R_ARM, HEAD_CENTER_DEG, SERVO_SPEED_SLOW); // 右臂归中 90°
}

/**
 * @brief 进入二级（深度）待机 —— ⚠️ 当前为占位实现，不执行真实睡眠
 *
 * 框架已就位，但真实睡眠 API 受 STANDBY_DEEP_ENABLE 宏保护，默认禁用（=0）。
 * 启用前必须先完成以下准备（否则设备会醒不过来 / 反复重启）：
 *   1. 配置头部触摸引脚（BSP_TOUCH_1_PIN）为 GPIO 唤醒源（esp_sleep_enable_gpio_wakeup
 *      或 esp_sleep_enable_ext1_wakeup），并确认 TTP223 输出电平与唤醒触发沿匹配。
 *   2. 睡前优雅停掉/通知各模块：关 LCD、停舵机、断开或挂起 WS/MQTT。
 *   3. 选定方案（见下方两套），二选一启用。
 *
 * 方案 1（light sleep，STANDBY_DEEP_USE_LIGHT_SLEEP=1）：
 *   esp_light_sleep_start();  // 阻塞在此，GPIO 中断后从下一行就地继续，RAM 保留
 *   // 醒来后：标志复位、恢复 LCD/监听即可，无需重建模型。
 *
 * 方案 2（deep sleep，STANDBY_DEEP_USE_LIGHT_SLEEP=0）：
 *   esp_deep_sleep_start();   // 不返回！GPIO 中断唤醒后从 app_main 重新启动，
 *   // 需在启动早期用 esp_sleep_get_wakeup_cause() 判断是否「从深睡醒来」做差异化初始化。
 */
static void enter_deep_standby(void)
{
#if STANDBY_DEEP_ENABLE
    ESP_LOGW(TAG, "进入二级深度待机（%s）",
             STANDBY_DEEP_USE_LIGHT_SLEEP ? "light sleep" : "deep sleep");
    // TODO: 1) 配置头部触摸 GPIO 唤醒源
    // TODO: 2) 关 LCD / 停舵机 / 处理 WS、MQTT
    // TODO: 3) 二选一：esp_light_sleep_start() 或 esp_deep_sleep_start()
    //         具体见本函数 doc 注释。当前未启用，避免误睡眠。
#else
    // 占位：仅打印，便于先验证「5 分钟超时被正确检测到」这条链路，不真正睡眠。
    ESP_LOGW(TAG, "[占位] 已达二级（深度）待机阈值 %d ms，但深度睡眠未启用（STANDBY_DEEP_ENABLE=0）",
             STANDBY_DEEP_TIMEOUT_MS);
#endif
}

void standby_wake(void)
{
    // 幂等：先刷新活动时间，避免唤醒后立刻又超时
    touch_activity();
    if (!s_standby)
        return;
    ESP_LOGI(TAG, "退出待机，恢复正常状态");
    s_standby = false;
    bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT);                   // 背光恢复 100%
    wake_word_start();                                                      // 重新开启唤醒词监听
    servo_manager_submit_angle(CH_HEAD, HEAD_CENTER_DEG, SERVO_SPEED_SLOW); // 头部归中
}

/**
 * @brief 待机监测周期任务
 *
 * 职责：
 *   1. 每秒检测会话是否活跃（session_get_state != IDLE），活跃则刷新活动时间，
 *      天然覆盖「正在对话 / TTS 播放」场景，避免误进待机。
 *   2. 非待机且空闲超时 → enter_standby。
 *   3. 待机期间按 SWING_PERIOD_MS 节奏提交头部左右摆动。
 */
static void standby_task(void *arg)
{
    bool swing_left = false;   // 下一次摆动方向（左/右交替）
    int64_t last_swing_us = 0; // 上次提交摆头的时间戳

    ESP_LOGI(TAG, "待机监测任务启动（超时 %d ms）", STANDBY_LIGHT_TIMEOUT_MS);
    touch_activity(); // 初始化活动时间，从启动开始计时

    while (1)
    {
        // ── 1. 会话活跃即视为活动（覆盖对话中 / TTS 播放）──────────────────
        if (session_get_state() != SESSION_IDLE)
            touch_activity();

        int64_t now_us = esp_timer_get_time();
        int64_t idle_ms = (now_us - s_last_active_us) / 1000;

        // ── 2. 空闲超时进入待机 ────────────────────────────────────────────
        if (!s_standby && idle_ms >= STANDBY_LIGHT_TIMEOUT_MS)
        {
            enter_standby();
            last_swing_us = 0; // 进入待机后立即触发首次摆头
        }

        // ── 3. 待机摆头（左右交替，慢速；走 servo_manager 队列不抢线程）────
        if (s_standby && (now_us - last_swing_us) / 1000 >= SWING_PERIOD_MS)
        {
            float target = swing_left ? (HEAD_CENTER_DEG - HEAD_SWING_DEG)
                                      : (HEAD_CENTER_DEG + HEAD_SWING_DEG);
            servo_manager_submit_angle(CH_HEAD, target, SWING_SPEED); // 仅头部舵机，最慢速左右摇
            swing_left = !swing_left;
            last_swing_us = now_us;
        }

        vTaskDelay(pdMS_TO_TICKS(STANDBY_CHECK_MS));
    }
}

void standby_init(void)
{
    if (s_inited)
        return;
    s_inited = true;
    touch_activity();

    // 周期任务：栈放 SPIRAM，节省内部 SRAM；优先级低（仅做轻量 poll + 入队）
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        standby_task,
        "standby",
        4096,
        NULL,
        3, // 低优先级，不与音频/舵机争抢
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
        ESP_LOGE(TAG, "待机监测任务创建失败！");
    else
        ESP_LOGI(TAG, "待机模块初始化完成");
}
