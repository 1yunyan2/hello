/**
 * @file bsp_touch.c
 * @brief 触摸按键板级支持包
 *
 * 支持两套触摸方案，通过 bsp_config.h 中的 BSP_USE_TTP223 切换：
 *   BSP_USE_TTP223 1 — TTP223-TD 外置芯片，GPIO 数字读取，低有效
 *   BSP_USE_TTP223 0 — ESP32-S3 内置电容触摸，能量阈值检测
 */

#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bsp/bsp_config.h"
#include "ui/interaction.h"
#include "ui/ui_port.h"
#define BSP_USE_TTP223 1

#if !BSP_USE_TTP223
#include "driver/touch_pad.h"
#endif

static const char *TAG = "BSP_TOUCH";

#define TOUCH_EVENT_QUEUE_LEN 8
static QueueHandle_t s_touch_event_queue = NULL;

// ── 公共时序参数 ──────────────────────────────────────────────────────────────
#define BODY_PRESS_MIN_MS 200
#define PAGE_SHORT_PRESS_MIN_MS 100
#define PAGE_LONG_PRESS_MS 1000
#define PRESS_DEBOUNCE 2
#define RELEASE_DEBOUNCE 3

// ── 方案专属参数 ──────────────────────────────────────────────────────────────
#if BSP_USE_TTP223
// TTP223 上电后约 500ms 完成内部基线校准，留 1s 裕量
#define POWER_ON_MASK_TIME 1500
#else
// 内置电容触摸上电后基线漂移窗口较长
#define POWER_ON_MASK_TIME 4000
#define TOUCH_THRESH_PERCENT 0.15f
#define TOUCH_MIN_DELTA 2000
#endif

// ── 按键状态结构体 ────────────────────────────────────────────────────────────
typedef struct
{
#if BSP_USE_TTP223
    gpio_num_t pin; // TTP223 OUT 引脚
#else
    touch_pad_t channel; // ESP32 内置触摸通道
    uint32_t baseline;   // 无触摸时的基线原始值
#endif
    bool is_pressed;
    uint32_t press_start_ms;
    uint8_t press_count;
    uint8_t release_count;
} touch_btn_t;

// ── 按键实例 ──────────────────────────────────────────────────────────────────
#if BSP_USE_TTP223
static touch_btn_t btn_head = {.pin = BSP_TOUCH_1_PIN};
static touch_btn_t btn_back = {.pin = BSP_TOUCH_2_PIN};
static touch_btn_t btn_abdomen = {.pin = BSP_TOUCH_3_PIN};
static touch_btn_t btn_prev_page = {.pin = BSP_TOUCH_PREV_PIN};
static touch_btn_t btn_next_page = {.pin = BSP_TOUCH_NEXT_PIN};
#else
static touch_btn_t btn_head = {.channel = BSP_TOUCH_1_PIN};
static touch_btn_t btn_abdomen = {.channel = BSP_TOUCH_3_PIN};
static touch_btn_t btn_back = {.channel = BSP_TOUCH_2_PIN};
static touch_btn_t btn_prev_page = {.channel = BSP_TOUCH_PREV_PIN};
static touch_btn_t btn_next_page = {.channel = BSP_TOUCH_NEXT_PIN};
#endif

static bool s_combo_ha_active = false;
static uint8_t s_combo_ha_cnt = 0;
static bool s_combo_hb_active = false;
static uint8_t s_combo_hb_cnt = 0;
static bool s_combo_ab_active = false;
static uint8_t s_combo_ab_cnt = 0;

/**
 * @brief 翻页键占用者枚举
 * 用于实现翻页键互斥逻辑，防止前后页同时按下产生冲突
 */
typedef enum
{
    PAGE_OWNER_NONE = 0, // 无人占用
    PAGE_OWNER_PREV,     // 前页键占用
    PAGE_OWNER_NEXT      // 后页键占用
} page_owner_t;

// 当前翻页键占用者
static page_owner_t s_page_owner = PAGE_OWNER_NONE;
// 翻页键最后活跃时间（用于松手后冷却，防止电容耦合误触身体键）
static uint32_t s_page_active_ms = 0;

/**
 * @brief 发送触摸事件到队列
 * 函数含义：将检测到的触摸事件发送到事件队列，供后续处理
 * @param event 参数含义：要发送的触摸事件类型
 */
static void send_touch_event(touch_event_t event)
{
    // API含义：FreeRTOS队列发送函数，将数据发送到队列
    // API参数含义：
    //   s_touch_event_queue：队列句柄
    //   &event：要发送的数据指针
    //   0：阻塞时间（0表示不阻塞，立即返回）
    xQueueSend(s_touch_event_queue, &event, 0);
}

/**
 * @brief 马达震动脉冲
 * 函数含义：控制震动马达产生一个30毫秒的震动脉冲，用于触觉反馈
 */
void bsp_motor_pulse(void)
{
    // API含义：设置GPIO引脚电平
    // API参数含义：
    //   BSP_MOTOR_VIB_PIN：GPIO引脚号
    //   1：输出高电平（马达启动）
    gpio_set_level(BSP_MOTOR_VIB_PIN, 1);

    // API含义：FreeRTOS任务延时，单位为系统时钟节拍
    // API参数含义：pdMS_TO_TICKS(30)：将30毫秒转换为系统时钟节拍数
    vTaskDelay(pdMS_TO_TICKS(30));

    // API含义：设置GPIO引脚电平
    // API参数含义：
    //   BSP_MOTOR_VIB_PIN：GPIO引脚号
    //   0：输出低电平（马达停止）
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
}

// ═══════════════════════════════════════════════════════════════════════════
// TTP223 方案：GPIO 数字读取
// ═══════════════════════════════════════════════════════════════════════════
#if BSP_USE_TTP223

void bsp_touch_init(void)
{
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    gpio_config_t motor_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BSP_MOTOR_VIB_PIN),
    };
    gpio_config(&motor_conf);
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);

    // TTP223 OUT 引脚：输入 + 内部上拉（低有效，悬空时保持高电平不误触）
    gpio_config_t touch_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << BSP_TOUCH_1_PIN) |
                        (1ULL << BSP_TOUCH_2_PIN) |
                        (1ULL << BSP_TOUCH_3_PIN) |
                        (1ULL << BSP_TOUCH_PREV_PIN) |
                        (1ULL << BSP_TOUCH_NEXT_PIN),
    };
    gpio_config(&touch_conf);

    ESP_LOGI(TAG, "触摸初始化完成 (TTP223 GPIO 低有效)");
    ESP_LOGI(TAG, "引脚: 头=%d 背=%d 腹=%d 前页=%d 后页=%d",
             BSP_TOUCH_1_PIN, BSP_TOUCH_2_PIN, BSP_TOUCH_3_PIN,
             BSP_TOUCH_PREV_PIN, BSP_TOUCH_NEXT_PIN);
}

// TTP223 低有效：OUT=0 表示触摸
static inline bool read_btn_pressed(const touch_btn_t *btn)
{
    return gpio_get_level(btn->pin) == 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// 内置电容触摸方案：能量阈值检测
// ═══════════════════════════════════════════════════════════════════════════
#else // !BSP_USE_TTP223

static uint32_t touch_get_baseline(touch_pad_t ch)
{
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++)
    {
        touch_pad_sw_start();
        vTaskDelay(pdMS_TO_TICKS(10));
        uint32_t val;
        touch_pad_read_raw_data(ch, &val);
        sum += val;
    }
    return sum / 10;
}

void bsp_touch_init(void)
{
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    gpio_config_t motor_conf = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BSP_MOTOR_VIB_PIN),
    };
    gpio_config(&motor_conf);
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);

    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V);
    touch_pad_config(btn_head.channel);
    touch_pad_config(btn_abdomen.channel);
    touch_pad_config(btn_back.channel);
    touch_pad_config(btn_prev_page.channel);
    touch_pad_config(btn_next_page.channel);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    touch_pad_filter_disable();
    vTaskDelay(pdMS_TO_TICKS(100));

    btn_head.baseline = touch_get_baseline(btn_head.channel);
    btn_abdomen.baseline = touch_get_baseline(btn_abdomen.channel);
    btn_back.baseline = touch_get_baseline(btn_back.channel);
    btn_prev_page.baseline = touch_get_baseline(btn_prev_page.channel);
    btn_next_page.baseline = touch_get_baseline(btn_next_page.channel);

    ESP_LOGI(TAG, "触摸初始化完成 (内置电容触摸)");
    ESP_LOGI(TAG, "基线: 头=%lu 腹=%lu 背=%lu 前页=%lu 后页=%lu",
             btn_head.baseline, btn_abdomen.baseline, btn_back.baseline,
             btn_prev_page.baseline, btn_next_page.baseline);
}

static int32_t read_btn_delta(touch_btn_t *btn)
{
    uint32_t val;
    touch_pad_read_raw_data(btn->channel, &val);
    return (int32_t)(val - btn->baseline);
}

static bool read_btn_pressed(touch_btn_t *btn)
{
    int32_t delta = read_btn_delta(btn);
    return (delta > TOUCH_MIN_DELTA) &&
           (delta > (int32_t)(btn->baseline * TOUCH_THRESH_PERCENT));
}

#endif // BSP_USE_TTP223

/**
 * @brief 获取触摸事件
 * 函数含义：从事件队列中获取一个触摸事件（非阻塞）
 * @param out_event 参数含义：输出参数，用于存储获取到的事件
 * @return 返回值含义：true=成功获取到事件，false=队列无事件
 */
bool bsp_touch_get_event(touch_event_t *out_event)
{
    // API含义：从FreeRTOS队列接收数据
    // API参数含义：
    //   s_touch_event_queue：队列句柄
    //   out_event：接收数据的缓冲区指针
    //   0：阻塞时间（0表示不阻塞）
    // 返回值含义：pdTRUE=成功接收，pdFALSE=失败
    return xQueueReceive(s_touch_event_queue, out_event, 0) == pdTRUE;
}

/**
 * @brief 更新身体按键状态
 * 函数含义：处理头部/腹部/背部等身体触摸按键的状态机、消抖和事件发送
 * @param btn        参数含义：按键结构体指针
 * @param pressed    参数含义：当前是否检测到按下（原始状态）
 * @param in_combo   参数含义：是否处于组合按键状态（若是则忽略单键）
 * @param now_ms     参数含义：当前时间戳（毫秒）
 * @param name       参数含义：按键名称（用于日志）
 * @param short_evt  参数含义：短按事件类型
 */
static void update_body_btn(touch_btn_t *btn, bool pressed, bool in_combo,
                            uint32_t now_ms, touch_event_t short_evt)
{
    // 如果处于组合按键状态或被其他按键占用，彻底重置状态并返回
    if (in_combo)
    {
        btn->is_pressed = false;
        btn->press_count = 0;
        btn->release_count = 0;
        btn->press_start_ms = 0;
        return;
    }

    // 检测到按下
    if (pressed)
    {
        btn->release_count = 0; // 清零释放计数器
        if (!btn->is_pressed)
        {
            btn->press_count++; // 按下计数器加1
            // 连续检测到PRESS_DEBOUNCE次按下，确认是真按下
            if (btn->press_count >= PRESS_DEBOUNCE)
            {
                btn->is_pressed = true;
                btn->press_start_ms = now_ms; // 记录按下开始时间
            }
        }
    }
    // 检测到释放
    else
    {
        btn->press_count = 0; // 清零按下计数器
        if (btn->is_pressed)
        {
            btn->release_count++; // 释放计数器加1
            // 连续检测到RELEASE_DEBOUNCE次释放，确认是真释放
            if (btn->release_count >= RELEASE_DEBOUNCE)
            {
                // 计算按下持续时间
                uint32_t held = now_ms - btn->press_start_ms;
                // 如果按下时间超过身体按键阈值，发送短按事件
                if (held >= BODY_PRESS_MIN_MS)
                {
                    send_touch_event(short_evt);
                }
                // 重置按键状态
                btn->is_pressed = false;
                btn->press_start_ms = 0;
                btn->release_count = 0;
            }
        }
    }
}

/**
 * @brief 更新翻页键状态
 * 函数含义：处理前后翻页键的状态机、消抖、长短按判断和事件发送
 * @param btn           参数含义：按键结构体指针
 * @param pressed_raw   参数含义：当前是否检测到按下（原始状态）
 * @param blocked       参数含义：是否被互斥逻辑阻塞
 * @param now_ms        参数含义：当前时间戳（毫秒）
 * @param name          参数含义：按键名称（用于日志）
 * @param short_evt     参数含义：短按事件类型
 * @param long_evt      参数含义：长按事件类型
 * @param long_press_ms 参数含义：长按阈值时间（毫秒）
 */
static void update_page_btn(touch_btn_t *btn, bool pressed_raw, bool blocked,
                            uint32_t now_ms,
                            touch_event_t short_evt, touch_event_t long_evt,
                            uint32_t long_press_ms)
{
    // 只有未被阻塞时才认为是有效按下
    bool pressed = pressed_raw && !blocked;

    // 检测到按下
    if (pressed)
    {
        btn->release_count = 0;
        if (!btn->is_pressed)
        {
            btn->press_count++;
            if (btn->press_count >= PRESS_DEBOUNCE)
            {
                btn->is_pressed = true;
                btn->press_start_ms = now_ms;
            }
        }
    }
    // 检测到释放
    else
    {
        btn->press_count = 0;
        if (btn->is_pressed)
        {
            btn->release_count++;
            if (btn->release_count >= RELEASE_DEBOUNCE)
            {
                // 计算按下持续时间
                uint32_t held = now_ms - btn->press_start_ms;
                touch_event_t evt = TOUCH_EVENT_NONE;
                const char *kind = NULL;

                // 判断是长按还是短按
                if (held >= long_press_ms)
                {
                    evt = long_evt;
                    kind = "长按";
                }
                else if (held >= PAGE_SHORT_PRESS_MIN_MS)
                {
                    evt = short_evt;
                    kind = "短按";
                }

                // 发送对应事件
                if (evt != TOUCH_EVENT_NONE)
                {
                    send_touch_event(evt);
                }

                // 重置按键状态
                btn->is_pressed = false;
                btn->press_start_ms = 0;
                btn->release_count = 0;
            }
        }
    }
}

/**
 * @brief 触摸扫描任务
 * 函数含义：FreeRTOS任务，持续扫描所有触摸按键，处理组合按键、互斥逻辑和事件分发
 * @param pvParameters 参数含义：任务参数（未使用）
 */
void touch_scan_task(void *pvParameters)
{
    // 初始化触摸硬件
    bsp_touch_init();
    printf("\n触摸铜箔就绪\n");

    touch_event_t event;

    // 主循环
    while (1)
    {
#if BSP_USE_TTP223
        // TTP223 已在芯片内完成测量，直接读 GPIO
        vTaskDelay(pdMS_TO_TICKS(20));
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms < POWER_ON_MASK_TIME)
            continue;

        bool h_raw = read_btn_pressed(&btn_head);
        bool a_raw = read_btn_pressed(&btn_abdomen);
        bool b_raw = read_btn_pressed(&btn_back);
        bool prev_raw = read_btn_pressed(&btn_prev_page);
        bool next_raw = read_btn_pressed(&btn_next_page);

        // 翻页键有信号时屏蔽头部（防 PCB 串扰）
        if (prev_raw || next_raw)
            h_raw = false;

        bool h = h_raw, a = a_raw, b = b_raw;

        // TTP223 输出数字量，无法比较 delta 强弱，直接进组合判断
        bool combo_ha = h && a;
        bool combo_hb = h && b;
        bool combo_ab = a && b;

#else // !BSP_USE_TTP223
        touch_pad_sw_start();
        vTaskDelay(pdMS_TO_TICKS(5));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms < POWER_ON_MASK_TIME)
        {
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        int32_t dh = read_btn_delta(&btn_head);
        int32_t da = read_btn_delta(&btn_abdomen);
        int32_t db = read_btn_delta(&btn_back);
        int32_t dp = read_btn_delta(&btn_prev_page);
        int32_t dn = read_btn_delta(&btn_next_page);

        int32_t thresh_abs = TOUCH_MIN_DELTA;

        // 翻页键串扰兜底：翻页键有信号时清零头通道 delta
        if (dn > thresh_abs || dp > thresh_abs)
            dh = 0;

        bool h_raw = (dh > thresh_abs) && (dh > (int32_t)(btn_head.baseline * TOUCH_THRESH_PERCENT));
        bool a_raw = (da > thresh_abs) && (da > (int32_t)(btn_abdomen.baseline * TOUCH_THRESH_PERCENT));
        bool b_raw = (db > thresh_abs) && (db > (int32_t)(btn_back.baseline * TOUCH_THRESH_PERCENT));
        bool prev_raw = (dp > thresh_abs) && (dp > (int32_t)(btn_prev_page.baseline * TOUCH_THRESH_PERCENT));
        bool next_raw = (dn > thresh_abs) && (dn > (int32_t)(btn_next_page.baseline * TOUCH_THRESH_PERCENT));

        bool h = h_raw, a = a_raw, b = b_raw;

        // 身体键组合过滤：多键同时按时只保留 delta 足够大的
        {
            int body_cnt = (int)h + (int)a + (int)b;
            if (body_cnt > 1)
            {
                int32_t max_body = dh;
                if (da > max_body)
                    max_body = da;
                if (db > max_body)
                    max_body = db;
                int32_t body_thresh = max_body * 7 / 10;
                h = h && (dh >= body_thresh);
                a = a && (da >= body_thresh);
                b = b && (db >= body_thresh);
            }
        }

        // 翻页键互斥：只保留 delta 更大的
        if (prev_raw && next_raw)
        {
            if (dp >= dn)
                next_raw = false;
            else
                prev_raw = false;
        }

        // 翻页键按下时屏蔽所有身体键
        if (prev_raw || next_raw)
        {
            h = false;
            a = false;
            b = false;
        }

        bool combo_ha = h && a;
        bool combo_hb = h && b;
        bool combo_ab = a && b;

#endif // BSP_USE_TTP223

        /* ── 翻页键互斥逻辑 ── */
        if (s_page_owner == PAGE_OWNER_NONE)
        {
            // 前页先按下，前页占用
            if (prev_raw && !next_raw)
                s_page_owner = PAGE_OWNER_PREV;
            // 后页先按下，后页占用
            else if (next_raw && !prev_raw)
                s_page_owner = PAGE_OWNER_NEXT;
        }

        // 判断是否被阻塞
        bool prev_blocked = (s_page_owner != PAGE_OWNER_NONE && s_page_owner != PAGE_OWNER_PREV);
        bool next_blocked = (s_page_owner != PAGE_OWNER_NONE && s_page_owner != PAGE_OWNER_NEXT);

        /* ══════════════════════════════════════════════════════════
         * 【核心改动】根据当前视图决定行为
         * ══════════════════════════════════════════════════════════ */
        // API含义：获取当前UI视图状态
        ui_view_t cur_view = ui_get_current_view();

        /* 所有视图统一使用 3 秒长按阈值 */
        uint32_t long_press_ms = PAGE_LONG_PRESS_MS;

        /* 组合触摸：仅主界面启用 */
        bool body_combo_enabled = (cur_view == UI_VIEW_MAIN);
        if (body_combo_enabled)
        {
            // 头+腹组合（需连续 PRESS_DEBOUNCE 次才触发，防止滑动误触）
            if (combo_ha)
            {
                if (!s_combo_ha_active)
                {
                    if (++s_combo_ha_cnt >= PRESS_DEBOUNCE)
                    {
                        s_combo_ha_active = true;
                        send_touch_event(TOUCH_EVENT_COMBO_HEAD_ABDOMEN);
                    }
                }
            }
            else
            {
                s_combo_ha_active = false;
                s_combo_ha_cnt = 0;
            }

            // 头+背组合
            if (combo_hb)
            {
                if (!s_combo_hb_active)
                {
                    if (++s_combo_hb_cnt >= PRESS_DEBOUNCE)
                    {
                        s_combo_hb_active = true;
                        send_touch_event(TOUCH_EVENT_COMBO_HEAD_BACK);
                    }
                }
            }
            else
            {
                s_combo_hb_active = false;
                s_combo_hb_cnt = 0;
            }

            // 腹+背组合
            if (combo_ab)
            {
                if (!s_combo_ab_active)
                {
                    if (++s_combo_ab_cnt >= PRESS_DEBOUNCE)
                    {
                        s_combo_ab_active = true;
                        send_touch_event(TOUCH_EVENT_COMBO_ABDOMEN_BACK);
                    }
                }
            }
            else
            {
                s_combo_ab_active = false;
                s_combo_ab_cnt = 0;
            }
        }
        else
        {
            // 非主界面，清除组合按键状态
            s_combo_ha_active = false;
            s_combo_ha_cnt = 0;
            s_combo_hb_active = false;
            s_combo_hb_cnt = 0;
            s_combo_ab_active = false;
            s_combo_ab_cnt = 0;
        }

        /* 身体单按钮：仅主界面启用（非主界面时 in_combo=true 跳过） */
        bool body_enabled = (cur_view == UI_VIEW_MAIN);
        bool page_any_active = prev_raw || next_raw;
        if (page_any_active)
            s_page_active_ms = now_ms;
        // 翻页键松开后 100ms 内仍屏蔽身体键，等待电容耦合信号衰减
        bool page_blocking = page_any_active || ((now_ms - s_page_active_ms) < 100);

        // 更新头部按键
        update_body_btn(&btn_head, h, combo_ha || combo_hb || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_HEAD);
        update_body_btn(&btn_abdomen, a, combo_ha || combo_ab || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_ABDOMEN);
        update_body_btn(&btn_back, b, combo_hb || combo_ab || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_BACK);

        update_page_btn(&btn_prev_page, prev_raw, prev_blocked, now_ms,
                        TOUCH_EVENT_SHORT_PREV_PAGE, TOUCH_EVENT_LONG_PREV_PAGE, long_press_ms);
        update_page_btn(&btn_next_page, next_raw, next_blocked, now_ms,
                        TOUCH_EVENT_SHORT_NEXT_PAGE, TOUCH_EVENT_LONG_NEXT_PAGE, long_press_ms);

        /* 占用方释放：当按键释放后，清除占用状态 */
        if (s_page_owner == PAGE_OWNER_PREV && !btn_prev_page.is_pressed && !prev_raw)
            s_page_owner = PAGE_OWNER_NONE;
        if (s_page_owner == PAGE_OWNER_NEXT && !btn_next_page.is_pressed && !next_raw)
            s_page_owner = PAGE_OWNER_NONE;

        /* ── 事件分发：统一交给 ui_dispatch_touch_event ── */
        if (bsp_touch_get_event(&event))
        {
            ESP_LOGI(TAG, "触摸事件: %d, 当前视图: %d", (int)event, (int)ui_get_current_view());
            ui_dispatch_touch_event(event);
        }

        // 延时15毫秒，控制扫描周期约20毫秒（50Hz）
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}