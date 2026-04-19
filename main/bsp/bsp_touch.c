/**
 * @file bsp_touch.c
 * @brief 铜箔触摸扫描 BSP — 仅负责采集和事件入队，不涉及上层逻辑
 *
 * 架构原则：
 *   - BSP 层只做硬件读取和事件入队，不依赖 UI/interaction 等上层模块
 *   - 上层通过 bsp_touch_get_event() 轮询取事件，再自行映射到情绪/游戏动作
 *
 * 事件检测能力：
 *   1. 短按  — 按下后在 LONG_PRESS_MS 内释放
 *   2. 长按  — 按住超过 LONG_PRESS_MS（抬手前触发，只触发一次）
 *   3. 组合  — 两个铜箔同时按下（优先于单按检测）
 *
 * 触觉反馈：
 *   - 每次事件（短按/长按/组合）触发时给一次 30ms 震动脉冲（即时感知反馈）
 *   - 情绪播放时的震动模式由 interaction.c 独立控制，两者职责不同
 */

#include "bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bsp_config.h"

static const char *TAG = "BSP_TOUCH";

// ─── 队列 ────────────────────────────────────────────────────────────────────
#define TOUCH_EVENT_QUEUE_LEN 16
static QueueHandle_t s_touch_event_queue = NULL;

// ─── 触摸参数 ─────────────────────────────────────────────────────────────────
#define TOUCH_THRESH_PERCENT 0.01f ///< 1% 阈值变化率，适合铜箔/杜邦线
#define TOUCH_MIN_DELTA      1000  ///< 最小绝对变化量，防止基线漂移引起误触
#define POWER_ON_MASK_MS     2000  ///< 上电后屏蔽 2 秒，避免初始化噪声
#define LONG_PRESS_MS        1200  ///< 长按判定阈值：持续 > 1.2 秒则为长按
#define SCAN_INTERVAL_MS     20    ///< 扫描周期（ms）

// ─── 按钮状态结构体 ──────────────────────────────────────────────────────────
typedef struct
{
    touch_pad_t channel;        ///< 触摸通道（对应 GPIO）
    uint32_t    baseline;       ///< 未触摸时的稳定基线值
    bool        is_pressed;     ///< 当前是否处于按下状态
    uint32_t    press_start_ms; ///< 本次按下的起始时间（ms）
    bool        long_fired;     ///< 长按事件已发出（防止同一次按住重复触发）
} touch_btn_t;

static touch_btn_t btn_head    = {.channel = BSP_TOUCH_1_PIN};
static touch_btn_t btn_abdomen = {.channel = BSP_TOUCH_3_PIN};
static touch_btn_t btn_back    = {.channel = BSP_TOUCH_2_PIN};

// ─── 内部辅助 ─────────────────────────────────────────────────────────────────

/** @brief 向触摸事件队列发送事件（非阻塞，队满丢弃并打印警告） */
static void send_touch_event(touch_event_t event)
{
    if (xQueueSend(s_touch_event_queue, &event, 0) != pdTRUE) {
        ESP_LOGW(TAG, "触摸事件队列已满，丢弃事件 %d", (int)event);
    }
}

/** @brief 多次采样取平均，获取稳定的触摸基线 */
static uint32_t touch_get_baseline(touch_pad_t ch)
{
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        uint32_t val;
        touch_pad_read_raw_data(ch, &val);
        sum += val;
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    return sum / 10;
}

/** @brief 读取单个通道当前值，判断是否处于按下状态 */
static bool touch_btn_is_pressed(touch_btn_t *btn)
{
    uint32_t val;
    touch_pad_read_raw_data(btn->channel, &val);
    int32_t delta = (int32_t)(val - btn->baseline);
    /* S3 触摸时数值增大：delta 为正且超过绝对阈值或百分比阈值即判为按下 */
    return (delta > TOUCH_MIN_DELTA) ||
           (delta > (int32_t)(btn->baseline * TOUCH_THRESH_PERCENT));
}

// ─── 公开 API ─────────────────────────────────────────────────────────────────

/**
 * @brief 给震动马达一次 30ms 短脉冲（触觉即时反馈）
 *
 * 在事件被检测到的瞬间调用，让用户感知到触摸被响应。
 * 情绪播放时的震动模式由 interaction.c 独立管理，两者不冲突。
 */
void bsp_motor_pulse(void)
{
    gpio_set_level(BSP_MOTOR_VIB_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(30));
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
}

/**
 * @brief 初始化触摸传感器和震动马达
 *
 * 必须在 touch_scan_task 启动前由 BSP 初始化序列调用，
 * 或在 touch_scan_task 内部首行调用（当前采用后者）。
 */
void bsp_touch_init(void)
{
    /* 创建事件队列 */
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    /* 震动马达 GPIO 初始化，强制低电平防止上电乱抖 */
    gpio_config_t motor_conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BSP_MOTOR_VIB_PIN),
    };
    gpio_config(&motor_conf);
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);

    /* ESP32-S3 电容触摸初始化 */
    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V);
    touch_pad_config(btn_head.channel);
    touch_pad_config(btn_abdomen.channel);
    touch_pad_config(btn_back.channel);
    touch_pad_filter_disable(); /* 关闭滤波，保证低延迟响应 */

    /* 稳定后采集基线 */
    vTaskDelay(pdMS_TO_TICKS(100));
    btn_head.baseline    = touch_get_baseline(btn_head.channel);
    btn_abdomen.baseline = touch_get_baseline(btn_abdomen.channel);
    btn_back.baseline    = touch_get_baseline(btn_back.channel);

    ESP_LOGI(TAG, "触摸初始化完成 | 基线: 头=%lu 腹=%lu 背=%lu",
             btn_head.baseline, btn_abdomen.baseline, btn_back.baseline);
}

/**
 * @brief 非阻塞读取触摸事件（由上层 dispatch 任务轮询）
 *
 * @param out_event 输出事件类型
 * @return true  = 取到事件
 *         false = 队列为空
 */
bool bsp_touch_get_event(touch_event_t *out_event)
{
    if (s_touch_event_queue == NULL || out_event == NULL) return false;
    return xQueueReceive(s_touch_event_queue, out_event, 0) == pdTRUE;
}

// ─── 单按钮状态机 ─────────────────────────────────────────────────────────────

/**
 * @brief 更新单个按钮的状态机并在需要时发出事件
 *
 * 调用方需保证每 SCAN_INTERVAL_MS 调用一次。
 *
 * 状态转换：
 *   释放 → 按下（上升沿）: 记录时间，等待长按判定
 *   按下中 → 超过 LONG_PRESS_MS: 发送长按事件（只发一次）
 *   按下 → 释放（下降沿）: 若未触发长按则发送短按事件
 *
 * @param btn       按钮实例
 * @param now_ms    当前系统时间（ms）
 * @param short_evt 短按事件枚举值
 * @param long_evt  长按事件枚举值
 */
static void touch_btn_update(touch_btn_t *btn, uint32_t now_ms,
                              touch_event_t short_evt, touch_event_t long_evt)
{
    bool pressed = touch_btn_is_pressed(btn);

    if (pressed && !btn->is_pressed) {
        /* 上升沿：开始按下 */
        btn->is_pressed     = true;
        btn->press_start_ms = now_ms;
        btn->long_fired     = false;
    } else if (pressed && btn->is_pressed && !btn->long_fired) {
        /* 持续按住：检测是否达到长按阈值 */
        uint32_t held_ms = now_ms - btn->press_start_ms;
        if (held_ms >= LONG_PRESS_MS) {
            btn->long_fired = true;
            ESP_LOGI(TAG, "长按: evt=%d (持续 %lums)", (int)long_evt, (unsigned long)held_ms);
            send_touch_event(long_evt);
            bsp_motor_pulse(); /* 长按触觉反馈（稍重，让用户确认触发） */
        }
    } else if (!pressed && btn->is_pressed) {
        /* 下降沿：释放 */
        btn->is_pressed = false;
        if (!btn->long_fired) {
            /* 未触发过长按 → 这是一次短按 */
            ESP_LOGI(TAG, "短按: evt=%d", (int)short_evt);
            send_touch_event(short_evt);
            bsp_motor_pulse(); /* 短按触觉反馈 */
        }
    }
}

// ─── 主扫描任务 ───────────────────────────────────────────────────────────────

/**
 * @brief 触摸扫描主任务（由 application.c 创建）
 *
 * 扫描逻辑优先级：
 *   1. 上电屏蔽期：忽略所有触摸
 *   2. 组合检测（两点同时按）→ 优先发出组合事件，抑制本轮单按检测
 *   3. 单按/长按检测
 *
 * @param pvParameters bsp_board_t* 实例指针（当前未使用，预留）
 */
void touch_scan_task(bsp_board_t *pvParameters)
{
    bsp_touch_init();
    ESP_LOGI(TAG, "触摸扫描任务启动");

    bool combo_fired = false; /* 当前组合是否已发出过（防止持续按住重复发送） */

    while (1) {
        uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());

        /* ── 上电屏蔽期：前 2 秒不响应，等待电容基线稳定 ── */
        if (now_ms < POWER_ON_MASK_MS) {
            vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
            continue;
        }

        /* ── 读取三路当前物理状态（用于组合判断） ── */
        bool h = touch_btn_is_pressed(&btn_head);
        bool a = touch_btn_is_pressed(&btn_abdomen);
        bool b = touch_btn_is_pressed(&btn_back);

        bool any_combo = (h && a) || (h && b) || (a && b);

        if (any_combo && !combo_fired) {
            /* ── 组合事件：首次检测到两点同时按下 ── */
            combo_fired = true;

            touch_event_t combo_evt;
            if      (h && a && !b) combo_evt = TOUCH_EVENT_COMBO_HEAD_ABDOMEN;
            else if (h && b && !a) combo_evt = TOUCH_EVENT_COMBO_HEAD_BACK;
            else if (a && b && !h) combo_evt = TOUCH_EVENT_COMBO_ABDOMEN_BACK;
            else                   combo_evt = TOUCH_EVENT_COMBO_HEAD_ABDOMEN; /* 三点同按：取头+腹 */

            ESP_LOGI(TAG, "组合触摸: evt=%d (h=%d a=%d b=%d)", (int)combo_evt, h, a, b);
            send_touch_event(combo_evt);
            bsp_motor_pulse();

            /* 将三个按钮标记为"长按已发"，阻止本轮及后续单按/长按事件干扰 */
            btn_head.is_pressed    = true; btn_head.long_fired    = true;
            btn_abdomen.is_pressed = true; btn_abdomen.long_fired = true;
            btn_back.is_pressed    = true; btn_back.long_fired    = true;

        } else if (!any_combo) {
            /* ── 组合全部释放，重置组合标志 ── */
            combo_fired = false;

            /* ── 单按 / 长按检测 ── */
            touch_btn_update(&btn_head,    now_ms,
                             TOUCH_EVENT_SHORT_HEAD,    TOUCH_EVENT_LONG_HEAD);
            touch_btn_update(&btn_abdomen, now_ms,
                             TOUCH_EVENT_SHORT_ABDOMEN, TOUCH_EVENT_LONG_ABDOMEN);
            touch_btn_update(&btn_back,    now_ms,
                             TOUCH_EVENT_SHORT_BACK,    TOUCH_EVENT_LONG_BACK);
        }

        vTaskDelay(pdMS_TO_TICKS(SCAN_INTERVAL_MS));
    }
}
