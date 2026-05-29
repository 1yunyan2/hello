/**
 * @file servo_driver.c
 * @brief 独立舵机基础驱动实现（直接操作 ESP-IDF LEDC，无第三方组件依赖）
 *
 * 原理：
 *   标准模拟舵机用 50Hz（周期 20ms）的 PWM 控制，脉宽决定角度：
 *     - 0°   ≈ 500μs 脉宽
 *     - 180° ≈ 2400μs 脉宽
 *   本驱动用 LEDC 在 50Hz 下输出对应占空比即可控制角度。
 */

#include "servo_driver.h"
#include <math.h>
#include <string.h>
#include "driver/ledc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"

static const char *TAG = "SERVO_DRV";

// ── PWM 参数（标准模拟舵机规格，如需匹配特殊舵机可在此调整）────────────
#define SERVO_FREQ_HZ 50        // 舵机 PWM 频率：50Hz
#define SERVO_PERIOD_US 20000   // 周期 = 1/50Hz = 20000μs
#define SERVO_MIN_PULSE_US 500  // 0° 对应脉宽
#define SERVO_MAX_PULSE_US 2400 // 180° 对应脉宽
#define SERVO_MAX_ANGLE 180.0f  // 物理最大行程
#define SERVO_LEDC_MODE LEDC_LOW_SPEED_MODE
#define SERVO_LEDC_TIMER LEDC_TIMER_0
// 13 位分辨率：8192 级，约 2.4μs/级，足够平滑；50*8192 远低于时钟上限
#define SERVO_DUTY_RES LEDC_TIMER_13_BIT
#define SERVO_DUTY_MAX ((1 << 13) - 1) // 8191

// ── 内部状态（每通道）────────────────────────────────────────────────
typedef struct
{
    bool in_use;            // 该通道是否已配置
    float min_angle;        // 软限位下限
    float max_angle;        // 软限位上限
    float cur_angle;        // 最近一次写入的角度（用作"当前角度"）
    SemaphoreHandle_t lock; // 该通道互斥锁
} servo_ch_state_t;

static servo_ch_state_t s_ch[SERVO_MAX_CHANNELS];
static int s_ch_count = 0;
static bool s_inited = false;

// LEDC 通道映射表（通道 i 使用 LEDC_CHANNEL_i）
static const ledc_channel_t k_ledc_ch[SERVO_MAX_CHANNELS] = {
    LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2, LEDC_CHANNEL_3,
    LEDC_CHANNEL_4, LEDC_CHANNEL_5, LEDC_CHANNEL_6, LEDC_CHANNEL_7};

// ── 私有：角度 -> LEDC 占空比 ────────────────────────────────────────
static uint32_t angle_to_duty(float angle)
{
    if (angle < 0.0f)
        angle = 0.0f;
    if (angle > SERVO_MAX_ANGLE)
        angle = SERVO_MAX_ANGLE;
    // 线性映射角度到脉宽，再把脉宽换算成占空比
    float pulse_us = SERVO_MIN_PULSE_US +
                     (SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US) * (angle / SERVO_MAX_ANGLE);
    float duty = pulse_us / (float)SERVO_PERIOD_US * (float)SERVO_DUTY_MAX;
    return (uint32_t)(duty + 0.5f);
}

// ── 私有：软限位裁剪 ─────────────────────────────────────────────────
static float clamp_safe_angle(uint8_t channel, float target)
{
    float safe = target;
    if (safe < s_ch[channel].min_angle)
        safe = s_ch[channel].min_angle;
    if (safe > s_ch[channel].max_angle)
        safe = s_ch[channel].max_angle;
    if (safe != target)
        ESP_LOGW(TAG, "通道 %d 触发软限位! 修正 %.1f -> %.1f", channel, target, safe);
    return safe;
}

// ── 私有：底层写一路 PWM（不加锁，不裁剪，仅由已加锁/已裁剪的调用方使用）──
static void hw_write(uint8_t channel, float angle)
{
    uint32_t duty = angle_to_duty(angle);
    ledc_set_duty(SERVO_LEDC_MODE, k_ledc_ch[channel], duty);
    ledc_update_duty(SERVO_LEDC_MODE, k_ledc_ch[channel]);
    s_ch[channel].cur_angle = angle;
}

// ============================================================
// API: 初始化
// ============================================================
esp_err_t servo_driver_init(const servo_channel_cfg_t *channels, int channel_count)
{
    if (s_inited)
        return ESP_OK;
    if (channels == NULL || channel_count < 1 || channel_count > SERVO_MAX_CHANNELS)
        return ESP_ERR_INVALID_ARG;

    // 1. 配置 LEDC 定时器（50Hz）
    ledc_timer_config_t timer_cfg = {
        .speed_mode = SERVO_LEDC_MODE,
        .timer_num = SERVO_LEDC_TIMER,
        .duty_resolution = SERVO_DUTY_RES,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "LEDC 定时器配置失败: %s", esp_err_to_name(err));
        return err;
    }

    // 2. 逐路配置 LEDC 通道 + 互斥锁 + 状态
    for (int i = 0; i < channel_count; i++)
    {
        ledc_channel_config_t ch_cfg = {
            .gpio_num = channels[i].gpio_num,
            .speed_mode = SERVO_LEDC_MODE,
            .channel = k_ledc_ch[i],
            .timer_sel = SERVO_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
        };
        err = ledc_channel_config(&ch_cfg);
        if (err != ESP_OK)
        {
            ESP_LOGE(TAG, "通道 %d (GPIO%d) LEDC 配置失败: %s",
                     i, channels[i].gpio_num, esp_err_to_name(err));
            return err;
        }

        s_ch[i].in_use = true;
        s_ch[i].min_angle = channels[i].min_angle;
        s_ch[i].max_angle = channels[i].max_angle;
        s_ch[i].cur_angle = 90.0f; // 假设起点为中点
        s_ch[i].lock = xSemaphoreCreateMutex();
        if (s_ch[i].lock == NULL)
        {
            ESP_LOGE(TAG, "通道 %d 互斥锁创建失败，内存不足!", i);
            return ESP_ERR_NO_MEM;
        }
    }

    s_ch_count = channel_count;
    s_inited = true;
    ESP_LOGI(TAG, "舵机驱动初始化完成，共 %d 路", channel_count);

    // 3. 上电缓慢归中到 90°，防止从随机位置猛跳抽搐
    for (int i = 0; i < channel_count; i++)
        servo_move_smooth((uint8_t)i, 90.0f, SERVO_SPEED_MID);

    return ESP_OK;
}

// ============================================================
// API: 平滑运动
// ============================================================
void servo_move_smooth(uint8_t channel, float target, uint32_t step_ms)
{
    if (!s_inited || channel >= s_ch_count || !s_ch[channel].in_use)
    {
        ESP_LOGE(TAG, "舵机未就绪或通道 %d 非法!", channel);
        return;
    }

    xSemaphoreTake(s_ch[channel].lock, portMAX_DELAY);

    float safe_target = clamp_safe_angle(channel, target);
    float current = s_ch[channel].cur_angle;

    // 抖动死区：差值 < 1° 不动
    if (fabsf(safe_target - current) < 1.0f)
    {
        xSemaphoreGive(s_ch[channel].lock);
        return;
    }

    // 瞬间模式
    if (step_ms == 0)
    {
        hw_write(channel, safe_target);
        xSemaphoreGive(s_ch[channel].lock);
        return;
    }

    // 逐度插值
    float dir = (safe_target > current) ? 1.0f : -1.0f;
    for (float a = current;
         (dir > 0) ? (a <= safe_target) : (a >= safe_target);
         a += dir)
    {
        hw_write(channel, a);
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    // 兜底精准对齐目标，消除浮点累积误差
    hw_write(channel, safe_target);

    xSemaphoreGive(s_ch[channel].lock);
}

// ============================================================
// API: 瞬间写入 / 读取角度
// ============================================================
void servo_write_angle(uint8_t channel, float angle)
{
    servo_move_smooth(channel, angle, 0);
}

float servo_read_angle(uint8_t channel)
{
    if (!s_inited || channel >= s_ch_count || !s_ch[channel].in_use)
        return -1.0f;
    return s_ch[channel].cur_angle;
}

// ============================================================
// API: 多轴并行
// ============================================================
void servo_move_all_parallel(const float *targets, uint32_t step_ms, int count)
{
    if (!s_inited || targets == NULL || count < 1 || count > s_ch_count)
        return;

    // 依次加锁所有参与通道
    for (int i = 0; i < count; i++)
        xSemaphoreTake(s_ch[i].lock, portMAX_DELAY);

    float safe[SERVO_MAX_CHANNELS];
    float cur[SERVO_MAX_CHANNELS];
    int max_steps = 0;
    for (int i = 0; i < count; i++)
    {
        safe[i] = clamp_safe_angle((uint8_t)i, targets[i]);
        cur[i] = s_ch[i].cur_angle;
        int steps = (int)fabsf(safe[i] - cur[i]);
        if (steps > max_steps)
            max_steps = steps;
    }

    if (max_steps < 1 || step_ms == 0)
    {
        for (int i = 0; i < count; i++)
            hw_write((uint8_t)i, safe[i]);
        for (int i = 0; i < count; i++)
            xSemaphoreGive(s_ch[i].lock);
        return;
    }

    // 线性插值：每步同时写各路
    for (int step = 1; step <= max_steps; step++)
    {
        float t = (float)step / (float)max_steps;
        for (int i = 0; i < count; i++)
            hw_write((uint8_t)i, cur[i] + t * (safe[i] - cur[i]));
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    for (int i = 0; i < count; i++)
        hw_write((uint8_t)i, safe[i]);

    for (int i = 0; i < count; i++)
        xSemaphoreGive(s_ch[i].lock);
}
