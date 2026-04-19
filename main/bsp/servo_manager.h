#pragma once
/**
 * servo_manager.h
 * 高层舵机动作管理器 — 非阻塞接口、动作队列、按模式/索引调用
 *
 * 设计要点：
 * - 对外暴露少量 API：初始化 / 非阻塞下发动作 / index->模式映射 / 校准 NVS 接口
 * - 内部用 FreeRTOS 队列 + worker task 串行执行动作，保证线程安全（不需改动 bsp_servo_move_smooth）
 * - 支持任意幅度/方向/速度组合，且提供 index 映射生成器（方便 UI 用索引触发预设）
 *
 * 依赖：bsp/bsp_board.h（提供 bsp_servo_move_smooth、SERVO_SPEED_*、CH_* 宏）
 */

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "bsp_config.h" // SERVO_SPEED_* 宏的唯一定义来源（避免与 bsp_servo.c 重复）

// 幅度等级（对应具体角度偏差）
typedef enum
{
    SERVO_AMPLITUDE_10 = 10,
    SERVO_AMPLITUDE_15 = 15,
    SERVO_AMPLITUDE_20 = 20,
    SERVO_AMPLITUDE_30 = 30,
    // 后续可增加幅度等级
} servo_amplitude_t;

// 方向（语义���HEAD、ARM 统一使用 same sign：LEFT/FRONT = +，RIGHT/BACK = -）
typedef enum
{
    SERVO_DIR_NEUTRAL = 0, // 中立
    SERVO_DIR_LEFT = 1,    // head: left (+), arm: front (+)
    SERVO_DIR_RIGHT = -1,  // head: right (-), arm: back (-)
} servo_direction_t;

// 速度档位类型：直接使用 uint32_t，具体数值复用 bsp_config.h 的 SERVO_SPEED_* 宏
// 调用时传入 SERVO_SPEED_FAST / SERVO_SPEED_MID 等宏，无需记数字
typedef uint32_t servo_speed_level_t;

// 动作请求结构（用户填充后通过 api 提交）
typedef struct
{
    uint8_t channel;              // CH_HEAD/CH_L_ARM/CH_R_ARM
    servo_amplitude_t amplitude;  // 幅度：10/15/20/30
    servo_direction_t direction;  // 方向
    servo_speed_level_t speed_ms; // 速度档位（ms/度）
    uint8_t loop_count;           // 循环次数（oscillate 模式时生效）
    bool oscillate;               // true = 在两侧往返（如 left->right->left ...），false = 单次到位（并回中）
} servo_request_t;

/**
 * @brief 初始化 servo_manager（创建队列 + worker task）
 * @return ESP_OK 成功，其他 esp_err 失败
 */
esp_err_t servo_manager_init(void);

/**
 * @brief 反初始化 servo_manager，停止 worker（谨慎调用）
 */
void servo_manager_deinit(void);

/**
 * @brief 非阻塞提交舵机动作请求（入队后立即返回）
 * @param req 请求结构体指针（caller 保持其内存直到 api 返回）
 * @return ESP_OK 成功入队，ESP_ERR_NO_MEM/ESP_ERR_INVALID_ARG 等表示失败
 */
esp_err_t servo_manager_submit_request(const servo_request_t *req);

/**
 * @brief 用一个简单的 index（0..N-1）生成一个模式并入队执行。
 *        这个函数用于 UI 以数字索引触发“37 种”或更多组合，内部根据固定规则生成 amplitude/speed/direction。
 * @param channel 通道
 * @param index   索引（>=0，函数会在内部对可用组合循环）
 * @param loop_count 循环次数（oscillate 情况下有效）
 * @param oscillate 是否左右往返（true）还是单次到位并回中（false）
 * @return ESP_OK 成功入队
 */
esp_err_t servo_manager_submit_by_index(uint8_t channel, uint16_t index, uint8_t loop_count, bool oscillate);

/**
 * @brief 直接按角度（deg）/速度 下发动作（同步封装为入队）
 * @param channel 舵机通道
 * @param angle_deg 目标角度（0..180），会被 bsp_servo 内部软限位裁剪
 * @param speed_ms step_ms（0 表示瞬间）
 * @return ESP_OK 成功入队
 */
esp_err_t servo_manager_submit_angle(uint8_t channel, float angle_deg, uint32_t speed_ms);

/**
 * @brief 保���单个通道的校准脉宽（microseconds）到 NVS（非易失）
 * @param channel 通道号
 * @param min_us  最小脉宽（如 500）
 * @param max_us  最大脉宽（如 2400）
 * @return ESP_OK 成功
 */
esp_err_t servo_manager_save_calibration(uint8_t channel, uint32_t min_us, uint32_t max_us);

/**
 * @brief 读取单个通道���校准脉宽（若无则返回 false 并不修改 out_*）
 * @param channel 通道号
 * @param out_min_us 输出最小脉宽
 * @param out_max_us 输出最大脉宽
 * @return true 找到且填充成功，false 未找到或出错
 */
bool servo_manager_load_calibration(uint8_t channel, uint32_t *out_min_us, uint32_t *out_max_us);
