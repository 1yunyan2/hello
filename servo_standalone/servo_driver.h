#pragma once
/**
 * @file servo_driver.h
 * @brief 独立舵机基础驱动（ESP32 / ESP32-S3，纯 ESP-IDF LEDC PWM）
 *
 * 设计目标：
 *   - 零外部组件依赖：只用 ESP-IDF 自带的 driver/ledc，不需要 iot_servo 等托管组件。
 *   - 即拿即用：把整个 servo_standalone/ 文件夹丢进别人的工程，加进 CMake 即可编译。
 *   - 功能完整：初始化、平滑插值运动、瞬间到位、软限位保护、多轴并行运动。
 *
 * 典型用法：
 *   @code
 *   // 1. 定义每路舵机的引脚与软限位
 *   servo_channel_cfg_t chans[] = {
 *       { .gpio_num = 9,  .min_angle = 45.0f, .max_angle = 135.0f }, // 头
 *       { .gpio_num = 14, .min_angle = 10.0f, .max_angle = 160.0f }, // 左臂
 *       { .gpio_num = 4,  .min_angle = 10.0f, .max_angle = 160.0f }, // 右臂
 *   };
 *   // 2. 初始化（FreeRTOS 调度器启动后调用）
 *   servo_driver_init(chans, 3);
 *   // 3. 让 0 号舵机以中速平滑转到 135 度
 *   servo_move_smooth(0, 135.0f, SERVO_SPEED_MID);
 *   @endcode
 *
 * @note 所有运动函数内部使用 vTaskDelay，必须在 FreeRTOS 任务上下文中调用，禁止在中断里调用。
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

// ============================================================
// 速度档位（step_ms：每移动 1 度等待的毫秒数，数值越大越慢）
// ============================================================
#define SERVO_SPEED_INSTANT 0U    ///< 瞬间到位（无平滑过渡，慎用：上电归中禁止使用）
#define SERVO_SPEED_VERY_FAST 2U  ///< 极快（2ms/度）
#define SERVO_SPEED_FAST 5U       ///< 快速（5ms/度，适合挥手、点头）
#define SERVO_SPEED_MID 15U       ///< 中速（15ms/度，适合大多数动作）
#define SERVO_SPEED_SLOW 30U      ///< 慢速（30ms/度）
#define SERVO_SPEED_VERY_SLOW 50U ///< 极慢（50ms/度，细腻表达）

// 最多支持的舵机路数（按需调整，LEDC 低速模式硬件通道上限通常为 8）
#define SERVO_MAX_CHANNELS 8

/**
 * @brief 单路舵机配置
 */
typedef struct
{
    int gpio_num;    ///< 该路舵机信号线对应的 GPIO 编号
    float min_angle; ///< 软限位下限（度），低于此值会被自动裁剪
    float max_angle; ///< 软限位上限（度），高于此值会被自动裁剪
} servo_channel_cfg_t;

/**
 * @brief 初始化舵机驱动（配置 LEDC 50Hz PWM 并将各路缓慢归中到 90°）
 *
 * @param channels      舵机配置数组（每个元素一路舵机）
 * @param channel_count 路数（1 ~ SERVO_MAX_CHANNELS）
 * @return ESP_OK 成功；ESP_ERR_INVALID_ARG 参数非法；其他为 LEDC 配置失败
 *
 * @note 内部会调用 vTaskDelay 做归中，必须在 FreeRTOS 调度器启动后调用。
 */
esp_err_t servo_driver_init(const servo_channel_cfg_t *channels, int channel_count);

/**
 * @brief 平滑地把指定通道舵机驱动到目标角度
 *
 * 按 1°/step_ms 的速度逐度插值逼近目标，避免舵机猛跳产生抽搐。
 * 目标角度会先经过软限位裁剪。
 *
 * @param channel 通道索引（0 ~ channel_count-1，对应 servo_driver_init 传入的顺序）
 * @param target  目标角度（度，0~180，自动受软限位裁剪）
 * @param step_ms 每度延时（毫秒），0 表示瞬间到位，推荐用 SERVO_SPEED_* 宏
 *
 * @note 线程安全：同一通道内部用互斥锁串行化，不同通道可并发。
 */
void servo_move_smooth(uint8_t channel, float target, uint32_t step_ms);

/**
 * @brief 直接写入角度（瞬间到位，无平滑），等价于 servo_move_smooth(ch, angle, 0)
 * @param channel 通道索引
 * @param angle   目标角度（度，自动受软限位裁剪）
 */
void servo_write_angle(uint8_t channel, float angle);

/**
 * @brief 读取某通道当前角度（驱动内部记录的最近一次写入值）
 * @param channel 通道索引
 * @return 当前角度（度）；通道非法返回 -1.0f
 */
float servo_read_angle(uint8_t channel);

/**
 * @brief 多路舵机同时平滑运动到各自目标（真正并行，同步到达）
 *
 * 以行程最大的一路为步数基准，所有路在相同时间内同步到位，
 * 避免串行调用产生的"一个先动完另一个才开始"的割裂感。
 *
 * @param targets     目标角度数组（长度需 >= channel_count）
 * @param step_ms     每步延时（毫秒）
 * @param count       本次同时驱动的通道数（从通道 0 开始连续）
 */
void servo_move_all_parallel(const float *targets, uint32_t step_ms, int count);

#ifdef __cplusplus
}
#endif
