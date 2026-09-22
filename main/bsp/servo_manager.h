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
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h" // SemaphoreHandle_t（带完成通知的提交接口）
#include "bsp_config.h"      // SERVO_SPEED_* 宏的唯一定义来源（避免与 bsp_servo.c 重复）
#include "bsp_board.h"       // bsp_servo_seq_axis_t / BSP_SERVO_SEQ_MAX_POINTS（动作序列底层类型）

// 幅度等级（对应具体角度偏差）
typedef enum
{
    SERVO_AMPLITUDE_5 = 5,
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

// 单轴并行子动作：用于 servo_parallel_request_t，描述某一轴在一次并行动作中的参数。
// count==0 表示该轴本次不参与（apply 时保持 90° 中位）。
typedef struct
{
    servo_amplitude_t amplitude;  // 幅度：10/15/20/30（相对 90° 中位的偏摆角度）
    servo_direction_t direction;  // 方向（NEUTRAL=不偏）
    servo_speed_level_t speed_ms; // 速度档位（ms/度）
    uint8_t count;                // 往返/重复次数；0 = 该轴不动
    bool oscillate;               // true=两侧往返，false=单次到位回中
} servo_axis_action_t;

// 三轴并行动作请求：三轴【同时】运动（各自动作可不同，但在同一时间窗口内并行执行）。
// 与 servo_request_t（单轴、串行入队）不同，本请求由 worker 调 bsp_servo_move_all_parallel
// 实现真正的三轴同步，避免「头先动完、臂才开始」的轮流割裂感。
typedef struct
{
    servo_axis_action_t head;  // 头部动作
    servo_axis_action_t l_arm; // 左臂动作
    servo_axis_action_t r_arm; // 右臂动作
} servo_parallel_request_t;

// 单轴【绝对角度】子动作：直接给出两个目标角（与情绪表 ActionStep_t 1:1，零转换误差）。
// count==0 表示该轴本次不参与（执行时该轴保持 90° 中位）。
typedef struct
{
    float angle_1;     // 前半段目标角度（度，0~180，bsp 内部软限位裁剪）
    float angle_2;     // 后半段目标角度（度）；单向动作填 90.0f（归中）
    uint32_t speed_ms; // step_ms（速度档位，值越大越慢）
    uint8_t count;     // 循环次数；0 = 该轴不动（恒 90°）
} servo_abs_axis_t;

// 三轴【绝对角度】并行请求：三轴同时按 angle_1→angle_2 往返 count 次，末尾全轴归中。
// 与 servo_parallel_request_t（幅度/方向语义）不同，本请求直接吃绝对角度，
// 供 interaction 情绪动作复用（情绪表本就是绝对角度），避免幅度/方向换算误差。
typedef struct
{
    servo_abs_axis_t head;
    servo_abs_axis_t l_arm;
    servo_abs_axis_t r_arm;
} servo_abs_parallel_request_t;

/** 动作序列每轴最大步数（须与 interaction.h 的 ACTION_SEQ_MAX_STEPS 一致）
 *  ★2026-09-11 由 8 提到 32：情绪 19「怕痒扭动」已用满 8 步，上限先于表达力
 *    成为瓶颈。代价仅约 1KB flash（表是 const 放 flash，不占内部 SRAM）。 */
#define SERVO_SEQ_MAX_STEPS 32

/**
 * @brief 动作序列中的一步：目标角度 + 该步专属速度 + 到达后停留（2026-09-11 新增）
 *
 * 与 interaction.h 的 ActionSeqStep_t 字段一一对应（那边配置侧、这边执行侧）。
 * speed 留 0 表示"沿用本轴的 seq_speed"，由 servo_exec_seq 负责解析成实际值。
 *
 * ★2026-09-18 新增 hold_ms：走到本步角度后原地停 hold_ms 毫秒再走下一步。
 *   0 = 不停留，故所有旧配置写法不变、语义不变。
 *   【为什么需要它】执行层时间的唯一来源是位移（帧数 = 角度差 × speed ÷ 帧长），
 *   "原地不动"耗时恒为 0，会被钳成 1 帧。所以连写多个相同角度（或 ±1° 交替）
 *   都换不来停顿——想停，只能显式填这个字段。
 */
typedef struct
{
    float angle;     ///< 该步目标角度（度，0~180，bsp 内部软限位裁剪）
    uint32_t speed;  ///< 该步速度（step_ms）；★0 = 沿用本轴 seq_speed
    uint32_t hold_ms; ///< ★走到本步角度后【原地停留】的毫秒数（0 = 不停留）
} servo_seq_step_t;

/**
 * @brief 单轴动作序列（2026-09-04 新增）
 *
 * 一个轴的完整生命周期：90°(起始) → target(终点) → seq[](动作序列) → 归中90°
 * 与 interaction.h 的 ActionSeq_t 字段一一对应（那边是配置侧，这边是执行侧）。
 *
 * 【与 servo_abs_axis_t 的区别】旧结构只能表达"A↔B 往返 N 次"，且执行时三轴
 * 被强行同步（速度取三轴最慢者）。本结构支持多步任意角度，且三轴各走各的。
 *
 * ★2026-09-11：seq 由「纯角度数组 + 一个统一速度」改为「每步自带速度」，
 *   以支持一个动作内的节奏变化（如猛地甩出去 + 慢慢晃回来）。seq_speed 保留，
 *   降级为【默认值】——每步 speed 填 0 即沿用它，故不想变速的动作写法不变。
 */
typedef struct
{
    float target;                              ///< 终点角度：从当前位置走到这里
    uint32_t speed;                            ///< 走向终点的速度（step_ms）
    servo_seq_step_t seq[SERVO_SEQ_MAX_STEPS]; ///< 动作序列（角度 + 每步速度）
    uint8_t seq_len;                           ///< 序列步数（0 = 到终点即止）
    uint32_t seq_speed;                        ///< 序列【默认】速度（每步 speed=0 时用它）
} servo_seq_axis_t;

/**
 * @brief 三轴动作序列请求：三轴【各走各的】，各自归中。
 *
 * 与 servo_abs_parallel_request_t 的关键差异：三轴不再共用同一时间窗口，
 * 各轴按自己的 speed/seq_speed 独立推进。
 *
 * ★2026-09-17：归中不再等三轴到齐。每个轴走完自己的终点+序列后【立即】
 *   回到自己的归位角（头 90° / 臂 ARM_CENTER_DEG），由 servo_exec_seq 把
 *   归中作为该轴序列的最后一段实现。此前是"全部走完后 worker 统一归中一次"，
 *   实测表现为双臂做完后干等头部摇完 8 步才一起回中。
 *   worker 那处统一归中保留，仅作 flush 打断时的兜底。
 */
typedef struct
{
    servo_seq_axis_t head;
    servo_seq_axis_t l_arm;
    servo_seq_axis_t r_arm;
} servo_seq_request_t;

/**
 * @brief 非阻塞提交【三轴动作序列】请求，可选完成通知。
 *
 * 每轴按 归位角→target→seq[]→归位角 执行，三轴独立推进互不等待（含各自的归中，
 * 见 servo_seq_request_t 的说明）。
 * 供 interaction 情绪动作使用（2026-09-04 起取代 abs_parallel 路径）。
 *
 * @param req      三轴序列请求（caller 保持其内存直到本函数返回）
 * @param done_sem 完成信号量：非 NULL 时执行完（含被 flush 打断）后 give；NULL=不通知
 * @return ESP_OK 成功入队
 */
esp_err_t servo_manager_submit_seq_notify(const servo_seq_request_t *req,
                                          SemaphoreHandle_t done_sem);

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
 * @brief 非阻塞提交【三轴并行】动作请求（入队后立即返回）
 *
 * 三轴在同一时间窗口内【同时】运动到各自目标（各轴动作可不同），
 * 由 worker 调 bsp_servo_move_all_parallel 实现真正同步，
 * 避免单轴串行入队造成的「头/左臂/右臂轮流动」割裂感。
 *
 * @param req 三轴并行请求（caller 保持其内存直到本函数返回）
 * @return ESP_OK 成功入队，ESP_ERR_INVALID_STATE/ESP_ERR_NO_MEM 表示失败
 */
esp_err_t servo_manager_submit_parallel(const servo_parallel_request_t *req);

/**
 * @brief 非阻塞提交【绝对角度三轴并行】动作请求，可选完成通知。
 *
 * 三轴按各自 angle_1→angle_2 往返 count 次后全轴归中，worker 串行执行。
 * 供 interaction 情绪动作使用（情绪表是绝对角度，零换算误差）。
 *
 * @param req      绝对角度三轴请求（caller 保持其内存直到本函数返回）
 * @param done_sem 完成信号量：非 NULL 时，worker 执行完（含正常播完归中 或 被
 *                 servo_manager_flush 打断归中）后 xSemaphoreGive 通知调用方；
 *                 NULL 表示不通知（自动循环/待机用 NULL）。
 * @return ESP_OK 成功入队
 */
esp_err_t servo_manager_submit_abs_parallel_notify(const servo_abs_parallel_request_t *req,
                                                   SemaphoreHandle_t done_sem);

/**
 * @brief 清空舵机动作队列并打断当前正在执行的动作，舵机平滑归中 90° 停住。
 *
 * 行为：置中断标志 → xQueueReset 清掉所有未执行请求 → 正在执行的并行动作
 * 在下一个循环边界 break 跳出 → worker 统一做一次平滑归中并清标志。
 * 用于进功能盘 / 强制回主界面时立即停舵机（解决队列堆积导致的「还重复动多次」）。
 *
 * @note 非阻塞：仅置标志 + ResetQueue，归中由 worker 线程执行（避免双线程抢舵机）。
 *       被打断的请求若带 done_sem，仍会被 give（防调用方永久阻塞）。
 * @return ESP_OK
 */
esp_err_t servo_manager_flush(void);

/**
 * @brief 清队并打断当前动作，可选择【打断后是否归中】。
 *
 * servo_manager_flush() 等价于 servo_manager_flush_ex(true)，语义完全不变。
 *
 * 【为何需要 center=false】归中服务的是「打断后没有后续动作接管」的场景
 * （进功能盘 / 强制回主 / OTA 停止）——动作停在半路姿态难看，故带回中位。
 * 但远程控制（app 下发舵机角度）是【有后续接管】的：打断空闲动作后，rc_worker
 * 紧接着就要把手臂送到用户指定角度。此时归中不仅多余，还会与 rc_worker 抢同一把
 * 通道锁，产生竞态——实测表现为「到位后又被慢速拉回 90°」「先归中再走到目标」
 * 「归中走一半停住」三种随机现象（2026-08-05）。
 * 同一思路见 remote_control_cancel(center_servo=false) 的注释。
 *
 * @param center true=打断后平滑归中 90°（原行为）；false=停在当前角度，交由调用方接管
 * @return ESP_OK
 */
esp_err_t servo_manager_flush_ex(bool center);

/**
 * @brief 查询 servo_manager 是否「真正空闲」（队列空 且 worker 当前无请求在执行/归中）
 *
 * 供 standby.c 进二级待机前判断舵机是否已彻底静止：相比「固定延时猜测」，本接口
 * 直接反映 worker 的真实执行状态，避免归中动作还没走完就被 bsp_servo_idle()
 * 的 ledc_stop 撞车，产生偶发的「进/退二级瞬间抖一下」。
 *
 * @return true 队列空且 worker 空闲，false 仍有请求排队或正在执行（含归中中）
 */
bool servo_manager_is_idle(void);

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
