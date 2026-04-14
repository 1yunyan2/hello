/**
 * @file bsp_touch.c
 * @brief 电容触摸按键 + 震动马达驱动（基于 ESP32-S3 硬件触摸控制器）
 *
 * 本模块实现三个触摸铜箔按键（左/中/右）的状态机检测，支持：
 *   - 短按（< 400ms）：触发 UI 切换操作（预留 TODO 接入 LVGL）
 *   - 长按（> 1000ms）：触发设置/特殊动作
 *   - 左右同按组合：触发退出游戏/静音等组合手势
 *
 * 触摸触发原理：
 *   ESP32-S3 内置电容触摸检测，铜箔连接到对应 GPIO（Touch Channel）。
 *   手指按压时，铜箔对地电容增大，导致充放电周期变化，ADC 读数剧烈改变。
 *   本模块通过读数与基线的偏差百分比（TOUCH_THRESH_PERCENT）判断是否触发。
 *
 * 震动马达（BSP_MOTOR_VIB_PIN = GPIO16）：
 *   拉高 GPIO 50ms 产生单次"嗒"声，提供触觉反馈。
 *
 * 依赖：
 *   - driver/touch_pad（ESP32-S3 内置触摸控制器驱动）
 *   - bsp_config.h（触摸和马达引脚宏）
 *   - bsp_board.h（BSP 单例，任务参数传递）
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/touch_pad.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "bsp_config.h"
#include "bsp/bsp_board.h"
// #include "driver/touch_sensor.h"
// #include "driver/touch_sensor_common.h"
static const char *TAG = "BSP_TOUCH";

// ==========================================
// 1. 硬件引脚与阈值配置
// ==========================================

// 触摸变化率阈值（与基线相比的变化百分比，范围 0.01 ~ 0.20）
// 铜箔贴在外壳内侧，手指按上时电容值会剧烈变化。
// 根据你的外壳厚度，这个值需要微调，通常 0.05 (5%) 到 0.1 (10%) 是个好起点。
#define TOUCH_THRESH_PERCENT 0.08 ///< 触发阈值：读数偏离基线 8% 以上视为按下（可按外壳厚度调整）

// 手势时间判定基准（毫秒）
#define TIME_SHORT_CLICK_MAX 400  ///< 短按最大持续时间（ms），按下到松开 < 400ms 判定为短按
#define TIME_LONG_PRESS_MIN  1000 ///< 长按最小持续时间（ms），持续按下 > 1000ms 判定为长按

/**
 * @brief 触摸按键状态记录结构体
 *
 * 每个物理按键（左/中/右）对应一个实例，存储其通道号、基线值和当前状态。
 * 由 touch_scan_task 中的状态机轮询更新。
 */
typedef struct
{
    touch_pad_t channel;   ///< ESP32-S3 触摸通道号（TOUCH_PAD_NUM1 / NUM2 / NUM7）
    uint32_t baseline;     ///< 初始化时采集的环境基线电容值（无触摸时的稳定读数）
    bool is_pressed;       ///< 当前物理按压状态（true = 正在按压，false = 已松开）
    uint32_t press_time;   ///< 按下时刻的系统时间戳（FreeRTOS tick → ms，用于计算持续时间）
    bool event_handled;    ///< 长按事件是否已触发（防止一次长按重复触发多次）
} touch_btn_t;

static touch_btn_t btn_left = {.channel = BSP_TOUCH_1_PIN};
static touch_btn_t btn_mid = {.channel = BSP_TOUCH_2_PIN};
static touch_btn_t btn_right = {.channel = BSP_TOUCH_3_PIN};

// ==========================================
// 2. 触觉反馈机制 (马达震动)
// ==========================================

/**
 * @brief 震动马达单次脉冲（提供触觉反馈）
 *
 * 将 BSP_MOTOR_VIB_PIN（GPIO16）拉高 50ms 后拉低，产生一次清脆的"嗒"震动感。
 * 短按触发 1 次，长按成功额外触发 2 次，提示用户手势已被识别。
 *
 * @param 无
 * @return void
 *
 * @note 调用者：bsp_touch_process_btn()（短按/长按检测后调用）
 * @note 前置条件：BSP_MOTOR_VIB_PIN GPIO 必须已在 bsp_touch_init() 中配置为输出模式
 * @note 此函数内部调用 vTaskDelay，必须在 FreeRTOS 任务上下文中使用
 */
void bsp_motor_pulse(void)
{
    // 拉高马达引脚 → 马达通电震动 → 延时 50ms → 拉低断电，产生单次脉冲感
    gpio_set_level(BSP_MOTOR_VIB_PIN, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0);
}

// ==========================================
// 3. 触摸板初始化与基线校准
// ==========================================

/**
 * @brief 初始化触摸控制器 + 震动马达，并采集环境基线
 *
 * 内部步骤：
 *   1. 配置 BSP_MOTOR_VIB_PIN（GPIO16）为 GPIO 输出（初始低电平）
 *   2. 调用 touch_pad_init() 初始化 ESP32-S3 触摸控制器
 *   3. 设置充放电电压（2.7V / 0.5V），影响灵敏度和功耗
 *   4. 配置三个触摸通道（btn_left / btn_mid / btn_right）
 *   5. 配置 IIR_16 软件滤波（抗 EMI 干扰），并启用滤波器
 *   6. 延时 100ms 等待滤波器稳定
 *   7. 读取三个通道的原始值作为环境基线（baseline）
 *
 * @param 无
 * @return void（失败时打印日志，不 panic）
 *
 * @note 调用者：touch_scan_task()（任务启动后首先调用）
 * @note 注意：初始化时设备应处于非触摸状态，否则基线会偏高导致触发灵敏度降低
 */
void bsp_touch_init(void)
{
    // ── 步骤 1：配置震动马达 GPIO（推挽输出，初始低电平）────────────────────
    gpio_config_t io_conf = {
        .mode         = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << BSP_MOTOR_VIB_PIN), // 仅配置 GPIO16
    };
    gpio_config(&io_conf);
    gpio_set_level(BSP_MOTOR_VIB_PIN, 0); // 初始低电平（马达断电）

    // ── 步骤 2：初始化触摸控制器 ──────────────────────────────────────────────
    touch_pad_init(); // 启动 ESP32-S3 内置触摸控制器，复位所有通道状态

    // ── 步骤 3：设置充放电电压（影响灵敏度和功耗）──────────────────────────
    // HVOLT_2V7 / LVOLT_0V5：充电至 2.7V，放电至 0.5V，电压差越大越灵敏
    // HVOLT_ATTEN_1V：高压侧衰减 1V，进一步提升长按检测稳定性
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);

    // ── 步骤 4：配置三个触摸通道（激活对应 Touch ADC）────────────────────────
    touch_pad_config(btn_left.channel);  // 左键（TOUCH1，GPIO1）
    touch_pad_config(btn_mid.channel);   // 中键（TOUCH2，GPIO2）
    touch_pad_config(btn_right.channel); // 右键（TOUCH7，GPIO7）

    // ── 步骤 5：配置 IIR_16 软件滤波器（ESP32-S3 特有，极大提升抗 EMI 能力）──
    touch_filter_config_t filter_info = {
        .mode         = TOUCH_PAD_FILTER_IIR_16, // IIR 16 阶低通滤波，平滑高频噪声
        .debounce_cnt = 1,                        // 去抖计数：1 次采样确认（响应快）
        .noise_thr    = 0,                        // 噪声阈值 0：不做额外噪声过滤
        .jitter_step  = 4,                        // 抖动步长：±4 计数内视为稳定
        .smh_lvl      = TOUCH_PAD_SMOOTH_IIR_2,  // 平滑级别：IIR_2（轻度平滑）
    };
    touch_pad_filter_set_config(&filter_info);
    touch_pad_filter_enable(); // 启用滤波器，后续读数通过 touch_pad_filter_read_smooth 获取

    // ── 步骤 6：等待滤波器稳定 ────────────────────────────────────────────────
    // 100ms 等待 IIR 滤波器收敛到稳定值，避免基线采集到启动瞬间的噪声
    vTaskDelay(pdMS_TO_TICKS(100));

    // ── 步骤 7：读取三通道环境基线（无触摸时的稳定参考值）──────────────────
    // 基线用于后续判断触发：|读数 - 基线| / 基线 > TOUCH_THRESH_PERCENT 即为触发
    uint32_t val;
    touch_pad_read_raw_data(btn_left.channel,  &val); btn_left.baseline  = val;
    touch_pad_read_raw_data(btn_mid.channel,   &val); btn_mid.baseline   = val;
    touch_pad_read_raw_data(btn_right.channel, &val); btn_right.baseline = val;

    ESP_LOGI(TAG, "触摸初始化完成. 基线 - 左:%lu, 中:%lu, 右:%lu",
             btn_left.baseline, btn_mid.baseline, btn_right.baseline);
}

// ==========================================
// 4. 核心逻辑：状态机判定 (放在 FreeRTOS Task 中运行)
// ==========================================

/**
 * @brief 单个触摸按键状态机处理（短按/长按/组合按检测）
 *
 * 每次调用读取一次该通道的平滑滤波值，与基线比较判断是否按下，
 * 然后更新按键状态机：
 *   - 下降沿（刚按下）：记录时间戳，触发马达震动反馈
 *   - 持续按下 > TIME_LONG_PRESS_MIN：触发长按事件（只触发一次）
 *   - 上升沿（松开）且 < TIME_SHORT_CLICK_MAX：触发短按事件
 *
 * @param btn      按键状态结构体指针（btn_left / btn_mid / btn_right 其中之一）
 * @param btn_name 按键名称字符串（用于日志，如 "左键" "中键" "右键"）
 * @return void
 *
 * @note 调用者：touch_scan_task()（每 20ms 轮询三个按键）
 * @note 此函数应在 FreeRTOS 任务上下文中以固定频率调用（推荐 20ms）
 * @note TODO：短按/长按事件触发后需接入 UI/LVGL 消息队列（当前仅打印日志）
 */
void bsp_touch_process_btn(touch_btn_t *btn, const char *btn_name)
{
    uint32_t current_val;
    // 获取经过滤波的平滑数据
    touch_pad_filter_read_smooth(btn->channel, &current_val);

    // ESP32-S3 触摸触发逻辑：按下去时，电容读数会【急剧增大】或【急剧减小】(取决于硬件排版)
    // 这里以读数改变超过设定的百分比为触发条件
    uint32_t diff = (current_val > btn->baseline) ? (current_val - btn->baseline) : (btn->baseline - current_val);
    bool physical_pressed = (diff > (btn->baseline * TOUCH_THRESH_PERCENT));

    uint32_t now_ms = pdTICKS_TO_MS(xTaskGetTickCount());

    // 状态机：刚被按下 (下降沿)
    if (physical_pressed && !btn->is_pressed)
    {
        btn->is_pressed = true;
        btn->press_time = now_ms;
        btn->event_handled = false;
        bsp_motor_pulse(); // 触发马达哒嗒声
        // ESP_LOGI(TAG, "%s 物理按下", btn_name);
    }
    // 状态机：一直按着 (长按判定)
    else if (physical_pressed && btn->is_pressed)
    {
        if (!btn->event_handled && (now_ms - btn->press_time) > TIME_LONG_PRESS_MIN)
        {
            ESP_LOGW(TAG, ">>>> 触发手势: %s 长按 (触发UI设置/特殊动作) <<<<", btn_name);
            bsp_motor_pulse(); // 长按成功再震一下提示用户
            bsp_motor_pulse();
            btn->event_handled = true; // 防止重复触发长按
        }
    }
    // 状态机：松开手 (上升沿，短按判定)
    else if (!physical_pressed && btn->is_pressed)
    {
        btn->is_pressed = false;
        uint32_t hold_time = now_ms - btn->press_time;

        if (!btn->event_handled && hold_time < TIME_SHORT_CLICK_MAX)
        {
            ESP_LOGI(TAG, ">>>> 触发手势: %s 短按 (触发UI切换) <<<<", btn_name);

            // TODO: 这里向你的 UI/LVGL 线程发送指令
            // if (btn == &btn_left)  { lv_msg_send(MSG_UI_PREV, NULL); }
            // if (btn == &btn_mid)   { lv_msg_send(MSG_UI_ENTER, NULL); }
            // if (btn == &btn_right) { lv_msg_send(MSG_UI_NEXT, NULL); }
        }
    }
}

/**
 * @brief 触摸扫描专属 FreeRTOS 任务（50Hz 轮询三个触摸按键）
 *
 * 任务启动后首先调用 bsp_touch_init() 完成硬件初始化，
 * 然后以 20ms 为周期循环扫描三个按键状态，并检测左右同按组合手势。
 *
 * 数据流：
 *   触摸铜箔 → ESP32-S3 Touch ADC → IIR 滤波 → bsp_touch_process_btn → 事件 → UI/LVGL（TODO）
 *
 * @param pvParameters BSP 实例指针（当前未使用，预留扩展）
 * @return void（永远运行的 FreeRTOS 任务）
 *
 * @note 调用者：应在系统初始化序列中通过 xTaskCreatePinnedToCore() 创建（当前预留）
 * @note 栈大小建议：4096 字节（含滤波运算和日志输出）
 * @note 组合手势（左右同按）检测后有 500ms 冷却时间，防止误触连续触发
 */
void touch_scan_task(bsp_board_t *pvParameters)
{

    bsp_touch_init();

    while (1)
    {
        bsp_touch_process_btn(&btn_left, "左键");
        bsp_touch_process_btn(&btn_mid, "中键");
        bsp_touch_process_btn(&btn_right, "右键");

        // 检测组合键: 左右同时按下
        if (btn_left.is_pressed && btn_right.is_pressed)
        {
            ESP_LOGE(TAG, ">>>> 触发组合手势: 左右同按 (退出游戏/静音) <<<<");
            vTaskDelay(pdMS_TO_TICKS(500)); // 防误触冷却
        }

        // 20ms 扫描一次 (50Hz的采样率，足够捕捉人类动作)
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}