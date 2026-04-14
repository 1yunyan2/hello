/**
 * @file bsp_servo.c
 * @brief 机器人三轴舵机控制实现（基于全局 BSP 架构）
 *
 * 本模块管理头部、左臂、右臂三路 PWM 舵机：
 *   - 使用 ESP32-S3 LEDC（LED 控制器）输出 50Hz PWM 信号驱动舵机
 *   - 内置软限位保护（_clamp_safe_angle），防止超出物理极限角度
 *   - 提供平滑插值运动（bsp_servo_move_smooth），避免上电抽搐
 *
 * 依赖：
 *   - iot_servo 组件（封装 LEDC PWM 舵机驱动）
 *   - bsp_board.h（BSP 单例，servo_initialized 状态位）
 *   - bsp_config.h（引脚与通道宏定义）
 */

#include "bsp_board.h"
#include "iot_servo.h"
#include <math.h>
#include "bsp/bsp_config.h"
static const char *TAG = "BSP_SERVO";

// ==========================================
// 1. 情绪/动作指令枚举 (对应你 Excel 表格的第一列)
// 注意：此枚举在 interaction.h 中也有相同定义（UI 层使用），
//       bsp_servo.c 内部仅用于舵机动作映射，保持两处同步。
// ==========================================

/**
 * @brief 机器人情绪/动作枚举
 *
 * 每个枚举值对应一套完整的情绪表现（舵机姿态 + 屏幕动画 + 音效 + 震动）。
 * 与 interaction.c 中的 InteractionMatrix_t 表格一一对应，
 * 调用 bsp_interaction_play(emotion_id) 触发完整表现。
 */
typedef enum
{
    EMO_HAPPY = 0,   ///< 开心：快速摇头 + 双臂前摆
    EMO_CURIOUS,     ///< 好奇：缓慢侧头 + 双臂前举
    EMO_TSUNDERE,    ///< 傲娇：头部轻偏 + 双臂后收
    EMO_TICKLISH,    ///< 怕痒：极速抖头 + 双臂快速前后摆
    EMO_SLEEPY,      ///< 犯困：缓慢点头 + 双臂下垂
    EMO_GRIEVED,     ///< 委屈：头部低垂 + 双臂内收
    EMO_COMFORTABLE, ///< 舒服：慢速摇头 + 双臂微展
    EMO_ACT_CUTE,    ///< 撒娇：头部倾斜 + 双臂上举
    EMO_ANGRY,       ///< 生气：快速摇头 + 双臂用力前摆
    EMO_SHY,         ///< 害羞：头部低垂 + 双臂遮脸
    EMO_SURPRISED,   ///< 惊喜：头部快速抬起 + 双臂上扬
    EMO_SLUGGISH,    ///< 慵懒：极慢摇头 + 双臂低垂
    EMO_HEALING,     ///< 治愈：慢速点头 + 双臂微展
    EMO_EXCITED      ///< 兴奋：快速大幅摇头 + 双臂大幅前后摆
    // ... 在这里继续添加你表格里剩下的情绪
} robot_emotion_t;

// 强保护机制：物理边界软限位 (Soft Limits)
// ⚠️ 组装好外壳后，请务必根据实际情况修改这几个极限值！
// 超出范围时 _clamp_safe_angle 会自动修正并打印警告日志。
#define HEAD_MIN_ANGLE  45.0f  ///< 头部向左最大极限角度（度），防止颈部过度旋转损坏舵机
#define HEAD_MAX_ANGLE 135.0f  ///< 头部向右最大极限角度（度）
#define L_ARM_MIN_ANGLE 10.0f  ///< 左臂向后最大极限角度（度），防止手臂撞到机身
#define L_ARM_MAX_ANGLE 160.0f ///< 左臂向前最大极限角度（度），防止撞头
#define R_ARM_MIN_ANGLE 10.0f  ///< 右臂向后最大极限角度（度）
#define R_ARM_MAX_ANGLE 160.0f ///< 右臂向前最大极限角度（度），防止撞头

// ==========================================
// 私有函数：角度边界裁剪 (防止物理撞击)
// ==========================================

/**
 * @brief 将目标角度限制在通道物理软限位范围内
 *
 * 若目标角度超出对应通道的软限位（HEAD/L_ARM/R_ARM），
 * 则裁剪至边界值并打印警告日志，防止舵机超范围运动损坏机械结构。
 *
 * @param channel      舵机通道（CH_HEAD / CH_L_ARM / CH_R_ARM，来自 bsp_config.h）
 * @param target_angle 调用方传入的目标角度（度，0.0f ~ 180.0f）
 * @return float       经过裁剪后的安全角度（在软限位范围内）
 *
 * @note 调用者：bsp_servo_move_smooth()（内部自动调用，外部无需直接使用）
 * @note 对于未知通道，强制返回 90.0f（安全中点），并不会 panic
 */
static float _clamp_safe_angle(uint8_t channel, float target_angle)
{
    float safe_angle = target_angle;
    switch (channel)
    {
    case CH_HEAD:
        if (safe_angle < HEAD_MIN_ANGLE)
            safe_angle = HEAD_MIN_ANGLE;
        if (safe_angle > HEAD_MAX_ANGLE)
            safe_angle = HEAD_MAX_ANGLE;
        break;
    case CH_L_ARM:
        if (safe_angle < L_ARM_MIN_ANGLE)
            safe_angle = L_ARM_MIN_ANGLE;
        if (safe_angle > L_ARM_MAX_ANGLE)
            safe_angle = L_ARM_MAX_ANGLE;
        break;
    case CH_R_ARM:
        if (safe_angle < R_ARM_MIN_ANGLE)
            safe_angle = R_ARM_MIN_ANGLE;
        if (safe_angle > R_ARM_MAX_ANGLE)
            safe_angle = R_ARM_MAX_ANGLE;
        break;
    default:
        safe_angle = 90.0f; // 未知通道强制归中
        break;
    }

    if (safe_angle != target_angle)
    {
        ESP_LOGW(TAG, "通道 %d 触发软限位保护! 修正 %.1f -> %.1f", channel, target_angle, safe_angle);
    }
    return safe_angle;
}

// ==========================================
// API: 舵机硬件生命周期初始化
// ==========================================

/**
 * @brief 初始化三轴舵机硬件，上电后缓慢归中至 90°
 *
 * 内部步骤：
 *   1. 配置 LEDC 参数（50Hz，脉宽 500~2500μs，3 个通道）
 *   2. 调用 iot_servo_init()（ESP32-S3 LEDC LOW_SPEED_MODE）
 *   3. 依次调用 bsp_servo_move_smooth 缓慢将三轴归中到 90°，防止上电抽搐
 *   4. 向 bsp_board->board_status 置位 BOARD_STATUS_SERVO_READY
 *
 * @param bsp_board BSP 实例指针
 *                  - 输出：servo_initialized 字段由此函数填充
 *                  - 输出：board_status 中的 BOARD_STATUS_SERVO_READY 位置位
 * @return void（初始化失败时打印错误日志，servo_initialized 置 false，不 panic）
 *
 * @note 调用者：application.c（初始化序列中，当前已预留）
 * @note 前置条件：FreeRTOS 调度器已启动（bsp_servo_move_smooth 内部调用 vTaskDelay）
 */
void bsp_board_servo_init(bsp_board_t *bsp_board)
{
    if (bsp_board == NULL)
    {
        ESP_LOGE(TAG, "BSP 实例为空，舵机初始化失败!");
        return;
    }

    ESP_LOGI(TAG, "正在初始化躯体舵机模块...");

    // ── 步骤 1：配置 LEDC PWM 舵机参数 ─────────────────────────────────────
    servo_config_t servo_cfg = {
        .max_angle    = 180,              // 物理最大行程 180°
        .min_width_us = 500,              // 0° 对应脉宽 500μs（标准舵机规格）
        .max_width_us = 2500,             // 180° 对应脉宽 2500μs
        .freq         = 50,               // PWM 驱动频率 50Hz（标准模拟舵机要求）
        .timer_number = LEDC_TIMER_0,     // 使用 LEDC 定时器 0（4 个可选，避免与 LED/蜂鸣器冲突）
        .channels = {
            .servo_pin = {
                BSP_SERVO_HEAD_PIN,       // GPIO38：头部舵机
                BSP_SERVO_L_ARM_PIN,      // GPIO47：左臂舵机
                BSP_SERVO_R_ARM_PIN,      // GPIO21：右臂舵机
            },
            .ch = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2}, // 三路独立 LEDC 通道
        },
        .channel_number = 3,              // 启用 3 路通道（头 + 左臂 + 右臂）
    };

    // ── 步骤 2：初始化硬件驱动（ESP32-S3 使用 LOW_SPEED_MODE）──────────────
    // LOW_SPEED_MODE 由软件定时器驱动，分辨率更高，适合低频 PWM（50Hz 舵机）
    esp_err_t err = iot_servo_init(LEDC_LOW_SPEED_MODE, &servo_cfg);

    if (err == ESP_OK)
    {
        // ── 步骤 3：标记初始化成功 ────────────────────────────────────────────
        bsp_board->servo_initialized = true;
        ESP_LOGI(TAG, "三轴舵机硬件初始化成功!");

        // ── 步骤 4：上电缓慢归中（防止舵机从随机位置快速跳到目标位置产生抽搐）──
        // SERVO_SPEED_MID = 15ms/度，从任意位置到 90° 最长约 1.35 秒
        bsp_servo_move_smooth(CH_HEAD,  90.0f, SERVO_SPEED_MID); // 头部归中
        bsp_servo_move_smooth(CH_L_ARM, 90.0f, SERVO_SPEED_MID); // 左臂归中
        bsp_servo_move_smooth(CH_R_ARM, 90.0f, SERVO_SPEED_MID); // 右臂归中

        // ── 步骤 5：置位事件标志位，通知其他模块（如 interaction）舵机已就绪──
        if (bsp_board->board_status != NULL)
        {
            xEventGroupSetBits(bsp_board->board_status, BOARD_STATUS_SERVO_READY);
        }
    }
    else
    {
        // 初始化失败（引脚冲突或 LEDC 资源被占用），标记未就绪，后续调用会拒绝执行
        bsp_board->servo_initialized = false;
        ESP_LOGE(TAG, "舵机初始化失败! 请检查引脚占用或底层库.");
    }
}

// ==========================================
// API: 安全平滑运动引擎
// ==========================================

/**
 * @brief 安全平滑地驱动指定通道舵机到目标角度
 *
 * 内部步骤：
 *   1. 检查 servo_initialized 标志，未初始化拒绝执行
 *   2. 通过 _clamp_safe_angle() 将目标角度限制在软限位范围内
 *   3. 读取当前角度（iot_servo_read_angle），计算差值
 *   4. 差值 < 1.0° 则跳过（消除抖动死区）
 *   5. step_ms == 0 时直接写入（瞬间模式）
 *   6. step_ms > 0 时按 1°/step_ms 步进插值，每步 vTaskDelay(step_ms)
 *   7. 循环结束后兜底写入目标角度，确保精准停位
 *
 * @param channel  舵机通道（CH_HEAD / CH_L_ARM / CH_R_ARM，来自 bsp_config.h）
 * @param target   目标角度（度，0.0f ~ 180.0f，自动受软限位裁剪）
 * @param step_ms  步进延时（毫秒/度）：
 *                 0 = 瞬间（危险，慎用）
 *                 SERVO_SPEED_FAST(5) / MID(15) / SLOW(30) = 推荐值
 * @return void（未就绪或读取失败时打印日志并提前返回）
 *
 * @note 调用者：bsp_board_servo_init()（归中）、bsp_interaction_play()（情绪动作）
 * @note 此函数内部调用 vTaskDelay，必须在 FreeRTOS 任务上下文中调用，不可在中断中使用
 * @note 线程安全：当前未加锁，若多个任务并发调用同一通道可能产生竞争，建议上层串行化
 */
void bsp_servo_move_smooth(uint8_t channel, float target, uint32_t step_ms)
{
    bsp_board_t *board = bsp_board_get_instance();

    // ── 前置检查：舵机必须已初始化 ───────────────────────────────────────────
    if (board == NULL || !board->servo_initialized)
    {
        ESP_LOGE(TAG, "舵机未就绪，拒绝执行动作指令!");
        return;
    }

    // ── 步骤 1：软限位裁剪（防止超出物理范围损坏机械结构）──────────────────
    float safe_target = _clamp_safe_angle(channel, target);

    // ── 步骤 2：读取当前实际角度（iot_servo_read_angle 返回 LEDC 寄存器推算值）──
    float current = 0.0f;
    esp_err_t err = iot_servo_read_angle(LEDC_LOW_SPEED_MODE, channel, &current);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "读取通道 %d 角度失败!", channel);
        return;
    }

    // ── 步骤 3：抖动死区过滤（差值 < 1° 不运动，消除因浮点精度产生的微抖）──
    if (fabs(safe_target - current) < 1.0f)
    {
        return; // 已经在目标位置，无需运动
    }

    // ── 步骤 4：瞬间模式（step_ms == 0，直接写入目标，无平滑过渡）──────────
    if (step_ms == 0)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, safe_target);
        return;
    }

    // ── 步骤 5：平滑插值运动（1°/step_ms，逐步逼近目标角度）────────────────
    // 每次循环移动 1 度，然后等待 step_ms 毫秒，产生匀速平滑效果
    float step_dir = (safe_target > current) ? 1.0f : -1.0f; // 确定运动方向

    for (float a = current;
         (step_dir > 0) ? (a <= safe_target) : (a >= safe_target);
         a += step_dir)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, a);
        vTaskDelay(pdMS_TO_TICKS(step_ms)); // 每度等待 step_ms ms
    }

    // ── 步骤 6：兜底对齐（确保最终精准停在目标位置，消除循环步进的浮点累积误差）──
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, safe_target);
}