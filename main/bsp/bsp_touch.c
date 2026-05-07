/**
 * @file bsp_touch.c
 * @brief 触摸按键板级支持包
 * 功能含义：实现电容触摸按键的扫描、消抖、组合按键检测以及事件分发，
 *          支持头部、腹部、背部三个身体触摸点和前后翻页键，
 *          根据当前UI视图动态调整按键行为和长按阈值。
 */

#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/touch_pad.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bsp/bsp_config.h"
#include "ui/interaction.h"
#include "ui/ui_port.h"

// 日志标签，用于ESP_LOG输出
static const char *TAG = "BSP_TOUCH";

// 触摸事件队列长度
#define TOUCH_EVENT_QUEUE_LEN 8
// 触摸事件队列句柄
static QueueHandle_t s_touch_event_queue = NULL;

// 触摸触发阈值百分比（相对于基线的变化率）
#define TOUCH_THRESH_PERCENT 0.15f
// 上电屏蔽时间（毫秒），上电后4秒内不检测触摸，避免上电干扰和基线漂移
#define POWER_ON_MASK_TIME 4000
// 触摸最小绝对变化量（原始值），低于此值认为是噪声
#define TOUCH_MIN_DELTA 2000
// 身体按键（头/腹/背）有效按压最小持续时间（毫秒）
#define BODY_PRESS_MIN_MS 200
// 翻页键短按最小持续时间（毫秒）
#define PAGE_SHORT_PRESS_MIN_MS 100
// 翻页键长按阈值时间（毫秒）
#define PAGE_LONG_PRESS_MS 1000
// 按下消抖计数（需要连续检测到多少次按下才算真按下）
#define PRESS_DEBOUNCE 2
// 释放消抖计数（需要连续检测到多少次释放才算真释放）
#define RELEASE_DEBOUNCE 3

/**
 * @brief 触摸按键状态结构体
 * 用于记录单个触摸按键的完整状态
 */
typedef struct
{
    touch_pad_t channel;     // 参数含义：触摸通道号（ESP32触摸引脚编号）
    uint32_t baseline;       // 参数含义：触摸基线值（无触摸时的原始读数）
    bool is_pressed;         // 参数含义：当前是否处于按下状态
    uint32_t press_start_ms; // 参数含义：按下开始的时间戳（毫秒）
    uint8_t press_count;     // 参数含义：按下消抖计数器
    uint8_t release_count;   // 参数含义：释放消抖计数器
} touch_btn_t;

// 头部触摸按键实例
static touch_btn_t btn_head = {.channel = BSP_TOUCH_1_PIN};
// 腹部触摸按键实例
static touch_btn_t btn_abdomen = {.channel = BSP_TOUCH_3_PIN};
// 背部触摸按键实例
static touch_btn_t btn_back = {.channel = BSP_TOUCH_2_PIN};
// 前页触摸按键实例
static touch_btn_t btn_prev_page = {.channel = BSP_TOUCH_PREV_PIN};
// 后页触摸按键实例
static touch_btn_t btn_next_page = {.channel = BSP_TOUCH_NEXT_PIN};

// 组合按键激活标志及消抖计数（需连续 PRESS_DEBOUNCE 次才触发）
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

/**
 * @brief 获取触摸基线值
 * 函数含义：连续读取10次触摸原始数据并取平均值，作为触摸基线
 * @param ch 参数含义：触摸通道号
 * @return 返回值含义：计算得到的基线值
 */
static uint32_t touch_get_baseline(touch_pad_t ch)
{
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++)
    {
        // API含义：启动软件控制的触摸传感器测量
        touch_pad_sw_start();

        // 延时10毫秒等待测量完成
        vTaskDelay(pdMS_TO_TICKS(10));

        uint32_t val;
        // API含义：读取触摸传感器原始数据
        // API参数含义：
        //   ch：触摸通道号
        //   &val：输出参数，用于存储读取到的原始值
        touch_pad_read_raw_data(ch, &val);

        sum += val;
    }
    return sum / 10; // 返回10次测量的平均值
}

/**
 * @brief 触摸初始化函数
 * 函数含义：初始化触摸按键驱动、GPIO、事件队列，并获取各按键的初始基线
 */
void bsp_touch_init(void)
{
    // API含义：创建FreeRTOS队列
    // API参数含义：
    //   TOUCH_EVENT_QUEUE_LEN：队列长度
    //   sizeof(touch_event_t)：每个元素的大小
    // 返回值含义：队列句柄
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    // GPIO配置结构体，用于震动马达
    gpio_config_t motor_conf = {
        .mode = GPIO_MODE_OUTPUT,                    // GPIO模式：输出
        .pin_bit_mask = (1ULL << BSP_MOTOR_VIB_PIN), // 要配置的GPIO引脚位掩码
    };

    // API含义：配置GPIO参数
    // API参数含义：&motor_conf：GPIO配置结构体指针
    gpio_config(&motor_conf);

    // API含义：设置GPIO引脚电平，初始化为低电平（马达停止）
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);

    // API含义：初始化触摸传感器驱动
    touch_pad_init();

    // API含义：设置触摸传感器充电/放电电压
    // API参数含义：
    //   TOUCH_HVOLT_2V7：高电压2.7V
    //   TOUCH_LVOLT_0V5：低电压0.5V
    //   TOUCH_HVOLT_ATTEN_0V：高电压衰减0V
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V);

    // API含义：配置单个触摸通道
    // API参数含义：btn_head.channel：要配置的触摸通道号
    touch_pad_config(btn_head.channel);
    touch_pad_config(btn_abdomen.channel);
    touch_pad_config(btn_back.channel);
    touch_pad_config(btn_prev_page.channel);
    touch_pad_config(btn_next_page.channel);

    // API含义：设置触摸传感器工作模式为软件控制模式
    // API参数含义：TOUCH_FSM_MODE_SW：软件控制模式
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);

    // API含义：禁用触摸传感器硬件滤波
    touch_pad_filter_disable();

    // 延时100毫秒，等待传感器稳定
    vTaskDelay(pdMS_TO_TICKS(100));

    // 获取各按键的基线值
    btn_head.baseline = touch_get_baseline(btn_head.channel);
    btn_abdomen.baseline = touch_get_baseline(btn_abdomen.channel);
    btn_back.baseline = touch_get_baseline(btn_back.channel);
    btn_prev_page.baseline = touch_get_baseline(btn_prev_page.channel);
    btn_next_page.baseline = touch_get_baseline(btn_next_page.channel);

    // API含义：输出日志信息
    // API参数含义：
    //   TAG：日志标签
    //   "触摸初始化完成"：日志内容
    ESP_LOGI(TAG, "触摸初始化完成");
    ESP_LOGI(TAG, "基线: 头=%lu 腹=%lu 背=%lu 前页=%lu 后页=%lu",
             btn_head.baseline, btn_abdomen.baseline, btn_back.baseline,
             btn_prev_page.baseline, btn_next_page.baseline);
}

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
 * @brief 读取按键变化量
 * 函数含义：读取触摸按键当前值与基线的差值
 * @param btn 参数含义：按键结构体指针
 * @return 返回值含义：当前值与基线的差值（可能为正或负）
 */
static int32_t read_btn_delta(touch_btn_t *btn)
{
    uint32_t val;
    // API含义：读取触摸传感器原始数据
    touch_pad_read_raw_data(btn->channel, &val);

    // 返回当前值与基线的差值
    return (int32_t)(val - btn->baseline);
}

/**
 * @brief 判断按键是否按下
 * 函数含义：根据变化量判断按键是否被按下
 * @param btn 参数含义：按键结构体指针
 * @return 返回值含义：true=按下，false=未按下
 */
static bool read_btn_pressed(touch_btn_t *btn)
{
    int32_t delta = read_btn_delta(btn);

    // 判断条件：
    // 1. 绝对变化量 > TOUCH_MIN_DELTA
    // 2. 相对变化量 > 基线的 TOUCH_THRESH_PERCENT (5%)
    return (delta > TOUCH_MIN_DELTA) &&
           (delta > (int32_t)(btn->baseline * TOUCH_THRESH_PERCENT));
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
                            uint32_t now_ms, const char *name,
                            touch_event_t short_evt)
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
                            uint32_t now_ms, const char *name,
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
        // API含义：启动一次软件触摸测量
        touch_pad_sw_start();

        // 延时5毫秒等待测量完成
        vTaskDelay(pdMS_TO_TICKS(5));

        // API含义：获取当前系统时间（毫秒）
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

        // 上电初期屏蔽触摸检测，避免干扰
        if (now_ms < POWER_ON_MASK_TIME)
        {
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        /* ── 一次性读取全部 5 通道 delta ── */
        int32_t dh = read_btn_delta(&btn_head);
        int32_t da = read_btn_delta(&btn_abdomen);
        int32_t db = read_btn_delta(&btn_back);
        int32_t dp = read_btn_delta(&btn_prev_page);
        int32_t dn = read_btn_delta(&btn_next_page);

        int32_t thresh_abs = TOUCH_MIN_DELTA;

        /* 软件兜底：翻页键通道有信号时，清零头通道。
         * PCB 走线串扰导致翻页键触摸时头通道 h 值异常暴增，
         * 此操作在 h_raw 计算之前从源头消除误触发。 */
        if (dn > thresh_abs || dp > thresh_abs)
            dh = 0;

        /* 各通道初步判断：delta > 绝对阈值 且 > 基线×相对阈值 */
        bool h_raw = (dh > thresh_abs) && (dh > (int32_t)(btn_head.baseline * TOUCH_THRESH_PERCENT));
        bool a_raw = (da > thresh_abs) && (da > (int32_t)(btn_abdomen.baseline * TOUCH_THRESH_PERCENT));
        bool b_raw = (db > thresh_abs) && (db > (int32_t)(btn_back.baseline * TOUCH_THRESH_PERCENT));
        bool prev_raw = (dp > thresh_abs) && (dp > (int32_t)(btn_prev_page.baseline * TOUCH_THRESH_PERCENT));
        bool next_raw = (dn > thresh_abs) && (dn > (int32_t)(btn_next_page.baseline * TOUCH_THRESH_PERCENT));

        bool h = h_raw, a = a_raw, b = b_raw;

        /* ── 第一步：身体键之间组合过滤（保留原逻辑，允许多键组合） ── */
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

        /* ── 第二步：翻页键之间互斥（只保留 delta 更大的） ── */
        if (prev_raw && next_raw)
        {
            if (dp >= dn)
                next_raw = false;
            else
                prev_raw = false;
        }

        /* ── 第三步：翻页键按下时，直接屏蔽所有身体键（彻底杜绝串扰） ── */
        if (prev_raw || next_raw)
        {
            h = false;
            a = false;
            b = false;
        }

        // 判断组合按键
        bool combo_ha = h && a; // 头+腹
        bool combo_hb = h && b; // 头+背
        bool combo_ab = a && b; // 腹+背

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
                        "头部", TOUCH_EVENT_SHORT_HEAD);
        // 更新腹部按键
        update_body_btn(&btn_abdomen, a, combo_ha || combo_ab || !body_enabled || page_blocking, now_ms,
                        "腹部", TOUCH_EVENT_SHORT_ABDOMEN);
        // 更新背部按键
        update_body_btn(&btn_back, b, combo_hb || combo_ab || !body_enabled || page_blocking, now_ms,
                        "背部", TOUCH_EVENT_SHORT_BACK);

        /* 翻页按钮（传入动态长按阈值） */
        // 更新前页键
        update_page_btn(&btn_prev_page, prev_raw, prev_blocked, now_ms,
                        "前页", TOUCH_EVENT_SHORT_PREV_PAGE, TOUCH_EVENT_LONG_PREV_PAGE,
                        long_press_ms);
        // 更新后页键
        update_page_btn(&btn_next_page, next_raw, next_blocked, now_ms,
                        "后页", TOUCH_EVENT_SHORT_NEXT_PAGE, TOUCH_EVENT_LONG_NEXT_PAGE,
                        long_press_ms);

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