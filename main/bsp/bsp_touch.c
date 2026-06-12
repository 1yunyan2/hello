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
#include "driver/ledc.h"
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

/* 最近一次翻页键（左/右耳）按压时长（毫秒），供跳一跳「长按蓄力」读取。
 * 在 update_page_btn 松手算出 held 后写入，bsp_touch_last_page_hold_ms() 读取。 */
static volatile uint32_t s_last_page_hold_ms = 0;

uint32_t bsp_touch_last_page_hold_ms(void)
{
    return s_last_page_hold_ms;
}

/**
 * @brief 查询翻页键当前实时按压时长（毫秒）
 *
 * 供跳一跳 engine_cb 每帧调用，驱动「按住期间小人/台子压扁 + 蓄力条实时增长」。
 * 若当前没有翻页键被按住，返回 0。
 *
 * 实现：直接读 btn_prev_page / btn_next_page 的 is_pressed + press_start_ms，
 * 与扫描任务共享内存，无锁（uint32_t 对齐读原子安全，最坏差一个扫描帧 ≈20ms）。
 */
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

/* 查询翻页键当前实时按压时长（毫秒）。供跳一跳 engine_cb 每帧驱动压扁动画。
 * 放在 btn_prev_page / btn_next_page 声明之后，避免前向引用编译错误。 */
uint32_t bsp_touch_page_held_ms(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (btn_prev_page.is_pressed && btn_prev_page.press_start_ms > 0)
        return now - btn_prev_page.press_start_ms;
    if (btn_next_page.is_pressed && btn_next_page.press_start_ms > 0)
        return now - btn_next_page.press_start_ms;
    return 0;
}

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
 * @brief 初始化震动马达 LEDC PWM 通道
 *
 * 马达由「GPIO 高低电平开关」升级为 LEDC PWM，用占空比调震动强度。
 * 资源隔离见 BUG-015：独立 TIMER_2 + CH4，时钟源强制 XTAL 与舵机/背光统一，
 * 否则同 speed_mode 共享时钟源会触发 "timer clock conflict" 致舵机初始化失败。
 *
 * 极性：马达低有效（OUT=0 通电）。LEDC duty 与输出电平的关系：
 *   duty=0    → 输出恒「低」电平 → 低有效下 = 满功率通电（一直震！）
 *   duty=MAX  → 输出恒「高」电平 → 低有效下 = 断电停止
 * ★ 因此「停止」必须用 duty=MAX，不是 0。初始化时若写 duty=0 会上电狂震。
 * 在 bsp_touch_init() 中替代原来的马达 GPIO 配置调用。
 */
static void bsp_motor_ledc_init(void)
{
    // 定时器：独立 TIMER_2，XTAL 时钟源，5kHz，10bit
    ledc_timer_config_t motor_timer = {
        .speed_mode = BSP_MOTOR_LEDC_MODE,
        .timer_num = BSP_MOTOR_LEDC_TIMER,
        .duty_resolution = BSP_MOTOR_LEDC_RES,
        .freq_hz = BSP_MOTOR_LEDC_FREQ_HZ,
        .clk_cfg = BSP_MOTOR_LEDC_CLK, // 强制 XTAL，与舵机/背光统一（BUG-015）
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // 通道：独立 CH4，绑定马达引脚。
    // ★ 初始 duty=MAX（输出恒高电平），低有效下=断电=停止，避免上电狂震。
    ledc_channel_config_t motor_channel = {
        .speed_mode = BSP_MOTOR_LEDC_MODE,
        .channel = BSP_MOTOR_LEDC_CHANNEL,
        .timer_sel = BSP_MOTOR_LEDC_TIMER,
        .gpio_num = BSP_MOTOR_VIB_PIN,
        .duty = BSP_MOTOR_DUTY_MAX, // 初始停止（恒高电平 → 低有效断电）
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_channel));
}

/**
 * @brief 设置震动马达持续输出强度
 * @param strength 震动强度百分比 0~100：0=停止，100=最强
 *
 * 低有效极性处理：strength 表示「通电占比」，但马达 OUT=0 才通电，
 * 因此实际写入 LEDC 的占空 = 满占空 - 通电占空（反相）。
 *   strength=0   → LEDC duty=MAX（恒高电平）→ 断电停止
 *   strength=100 → LEDC duty=0（恒低电平）  → 满功率通电
 */
void bsp_motor_set(uint8_t strength)
{
    if (strength > 100)
        strength = 100;

    // 期望通电占空（正逻辑），再反相得到实际 LEDC 占空（低有效）
    uint32_t on_duty = (uint32_t)BSP_MOTOR_DUTY_MAX * strength / 100;
    uint32_t ledc_duty = BSP_MOTOR_DUTY_MAX - on_duty;

    ledc_set_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, ledc_duty);
    ledc_update_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL);
}

/**
 * @brief 震动马达带强度的单次脉冲
 * @param strength 震动强度百分比 0~100
 * @param ms       持续时长（毫秒），结束后自动停止
 * @note 内部含 vTaskDelay 阻塞，仅可在任务上下文调用
 */
void bsp_motor_pulse_level(uint8_t strength, uint32_t ms)
{
    bsp_motor_set(strength);
    vTaskDelay(pdMS_TO_TICKS(ms));
    bsp_motor_set(0); // 停止
}

/**
 * @brief 马达震动脉冲（默认强度，约 30ms）
 * 函数含义：用 LEDC PWM 以默认强度震动 30ms，用于触摸/唤醒等触觉反馈
 */
void bsp_motor_pulse(void)
{
    bsp_motor_pulse_level(BSP_MOTOR_DEFAULT_STRENGTH, 30);
}

// ═══════════════════════════════════════════════════════════════════════════
// 【震动 PWM 方波测试任务】—— 验证占空比可调 + 示波器观测方波
//
// 用法：在 application.c 里 xTaskCreate(motor_pwm_test_task, ...) 启动即可。
// 行为：循环把占空比设成 0→25→50→75→100→(回0) 一档一档走，每档持续 3 秒，
//       串口打印当前档位。用 bsp_motor_set() 持续输出（非脉冲），方便示波器
//       Stop 抓波形。
//
// 示波器观测要点：
//   - 想看到 5kHz 方波细节，时基拉到 ~50µs/格（一个周期 200µs）；
//   - 占空比 0% 与 100% 是直流端点（恒高/恒低），不是方波，看到直线属正常；
//   - 25/50/75% 才是真方波，占空比逐档变化肉眼可辨。
// 测试完成后，把 application.c 里创建本任务的代码注释掉即可。
// ═══════════════════════════════════════════════════════════════════════════
void motor_pwm_test_task(void *pvParameters)
{
    (void)pvParameters;

    // 复用触摸初始化里的 LEDC 配置；若单独跑本测试（未启动 touch_scan_task），
    // 需保证马达 LEDC 已初始化。这里直接调一次初始化，幂等无副作用。
    bsp_motor_ledc_init();

    // 待测占空比档位（%）
    static const uint8_t test_levels[] = {0, 25, 50, 75, 100};
    const int n = sizeof(test_levels) / sizeof(test_levels[0]);

    ESP_LOGI(TAG, "═══ 震动 PWM 方波测试启动：每档持续 3 秒 ═══");

    while (1)
    {
        for (int i = 0; i < n; i++)
        {
            uint8_t lv = test_levels[i];
            bsp_motor_set(lv); // 持续输出该占空比，便于示波器观测
            ESP_LOGI(TAG, "震动占空比 = %u%%（%s）", lv,
                     (lv == 0)     ? "停止/恒高直流"
                     : (lv == 100) ? "满震/恒低直流"
                                   : "PWM 方波");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TTP223 方案：GPIO 数字读取
// ═══════════════════════════════════════════════════════════════════════════
#if BSP_USE_TTP223

void bsp_touch_init(void)
{
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    // 震动马达：LEDC PWM 初始化（独立 T2/CH4，XTAL 时钟源，初始停止）
    bsp_motor_ledc_init();

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

    // 震动马达：LEDC PWM 初始化（独立 T2/CH4，XTAL 时钟源，初始停止）
    bsp_motor_ledc_init();

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

/* 头部长按阈值：松手时超过此时长发 LONG_HEAD（退出/返回），否则发 SHORT_HEAD */
#define HEAD_LONG_PRESS_MS 800

/**
 * @brief 更新身体按键状态
 * @param btn        按键结构体指针
 * @param pressed    当前是否检测到按下（原始状态）
 * @param in_combo   是否处于组合按键状态（若是则忽略单键）
 * @param now_ms     当前时间戳（毫秒）
 * @param short_evt  短按事件类型
 * @param long_evt   长按事件类型（TOUCH_EVENT_NONE 表示该键不支持长按）
 */
static void update_body_btn(touch_btn_t *btn, bool pressed, bool in_combo,
                            uint32_t now_ms, touch_event_t short_evt, touch_event_t long_evt)
{
    // 组合键期间彻底重置，不产生单键事件
    if (in_combo)
    {
        btn->is_pressed = false;
        btn->press_count = 0;
        btn->release_count = 0;
        btn->press_start_ms = 0;
        return;
    }

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
    else
    {
        btn->press_count = 0;
        if (btn->is_pressed)
        {
            btn->release_count++;
            if (btn->release_count >= RELEASE_DEBOUNCE)
            {
                uint32_t held = now_ms - btn->press_start_ms;
                if (held >= BODY_PRESS_MIN_MS)
                {
                    /* 支持长按的键（头部）：按压超过阈值发长按事件，否则发短按 */
                    if (long_evt != TOUCH_EVENT_NONE && held >= HEAD_LONG_PRESS_MS)
                        send_touch_event(long_evt);
                    else
                        send_touch_event(short_evt);
                }
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
                            uint32_t long_press_ms, bool game_mode)
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

                /* 记录本次按压时长，供跳一跳「长按蓄力」读取（任何视图都记，开销极小）*/
                if (held >= PAGE_SHORT_PRESS_MIN_MS)
                    s_last_page_hold_ms = held;

                if (game_mode)
                {
                    /* ── 游戏视图：长按耳不再触发「回主界面」，统一当作一次翻页键操作 ──
                     * 跳一跳靠 bsp_touch_last_page_hold_ms() 读 held 决定蓄力大小，
                     * 因此无论长短，只要超过最小按压阈值都发 short_evt（游戏自行解读时长）。
                     * 这样既保留了「按住越久蓄力越大」，又不会在游戏里误触退出主界面。 */
                    if (held >= PAGE_SHORT_PRESS_MIN_MS)
                    {
                        evt = short_evt;
                        kind = "游戏蓄力";
                    }
                }
                else
                {
                    // 非游戏视图：维持原长/短按判定逻辑
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
                }

                // 发送对应事件
                if (evt != TOUCH_EVENT_NONE)
                {
                    send_touch_event(evt);
                }
                (void)kind;

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

        /* 身体单按钮：所有视图均启用（头/腹/背在各层都有用途）：
         *   - 主界面：头/腹/背触发情绪反馈
         *   - 总设置/应用列表/游戏列表/功能页/游戏：头=确认、腹/背=返回
         *   - 闹钟编辑：头=下一步/确认、腹/背=放弃退出
         * 注意：组合键(头+腹等)仍仅主界面启用，见上方 body_combo_enabled，
         *       因此非主界面单按头/腹/背不会被组合逻辑吞掉。 */
        bool body_enabled = true;
        bool page_any_active = prev_raw || next_raw;
        if (page_any_active)
            s_page_active_ms = now_ms;
        // 翻页键松开后 100ms 内仍屏蔽身体键，等待电容耦合信号衰减
        bool page_blocking = page_any_active || ((now_ms - s_page_active_ms) < 100);

        /* ── 触觉震动反馈（按下边沿触发一次）──────────────────────────────
         * 规则（按需求）：
         *   1. 左/右耳（翻页键）任何视图按下 → 震动；
         *   2. 头部在「非主界面」（功能盘/功能页/游戏/闹钟编辑）按下 → 震动；
         *   3. 主界面（情绪界面 UI_VIEW_MAIN）的头/腹/背触摸 → 不震动。
         * 边沿检测：此处各按键 is_pressed 仍是「上一帧」值（update_* 尚未执行），
         * 配合本帧 *_raw / h 判断「松→按」上升沿，避免按住期间连续震动。
         * 必须放在下面三个 update_body_btn / update_page_btn 之前，否则 is_pressed
         * 已被置位，上升沿检测失效。
         * 注意 bsp_motor_pulse() 内含 30ms 阻塞，仅按下瞬间触发一次，开销可接受。 */
        {
            // 头部震动适用视图：除主界面外的所有视图
            bool head_vib_view = (cur_view != UI_VIEW_MAIN);
            bool vib = false;

            // 左/右耳：上升沿且未被互斥阻塞 → 任何视图都震
            if (prev_raw && !prev_blocked && !btn_prev_page.is_pressed)
                vib = true;
            if (next_raw && !next_blocked && !btn_next_page.is_pressed)
                vib = true;

            // 头部：非主界面、上升沿、未被翻页屏蔽 → 震
            if (head_vib_view && h && !page_blocking && !btn_head.is_pressed)
                vib = true;

            if (vib)
                bsp_motor_pulse();
        }

        /* 头部：短按=情绪/游戏确认，长按≥800ms=退出/返回 */
        update_body_btn(&btn_head, h, combo_ha || combo_hb || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_HEAD, TOUCH_EVENT_LONG_HEAD);
        /* 腹/背：仅主界面情绪，不参与应用/游戏退出，无长按事件 */
        update_body_btn(&btn_abdomen, a, combo_ha || combo_ab || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_ABDOMEN, TOUCH_EVENT_NONE);
        update_body_btn(&btn_back, b, combo_hb || combo_ab || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_BACK, TOUCH_EVENT_NONE);

        /* 游戏视图下启用「蓄力模式」：长按耳不发长按事件、改记按压时长供跳一跳读取 */
        bool game_mode = (cur_view == UI_VIEW_GAME);

        update_page_btn(&btn_prev_page, prev_raw, prev_blocked, now_ms,
                        TOUCH_EVENT_SHORT_PREV_PAGE, TOUCH_EVENT_LONG_PREV_PAGE, long_press_ms, game_mode);
        update_page_btn(&btn_next_page, next_raw, next_blocked, now_ms,
                        TOUCH_EVENT_SHORT_NEXT_PAGE, TOUCH_EVENT_LONG_NEXT_PAGE, long_press_ms, game_mode);

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
