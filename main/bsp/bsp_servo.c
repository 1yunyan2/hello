/**
 * @file bsp_servo.c
 * @brief 机器人三轴舵机控制实现（基于全局 BSP 架构）
 *
 * 本模块管理头部、左臂、右臂三路 PWM 舵机：
 *   - 使用 ESP32-S3 LEDC（LED 控制器）输出 50Hz PWM 信号驱动舵机
 *   - 内置软限位保护（clamp_safe_angle），防止超出物理极限角度
 *   - 提供平滑插值运动（bsp_servo_move_smooth），避免上电抽搐
 *
 * 依赖：
 *   - iot_servo 组件（封装 LEDC PWM 舵机驱动）
 *   - bsp_board.h（BSP 单例，servo_initialized 状态位）
 *   - bsp_config.h（引脚与通道宏定义）
 */

#include "bsp/bsp_board.h"
#include "iot_servo.h"
#include <math.h>
#include "freertos/semphr.h" // 互斥锁，保证多任务调用线程安全
#include "bsp/bsp_config.h"
// 注意：robot_emotion_t 唯一定义在 interaction.h，此处不重复定义。
// 注意：不 include servo_manager.h，避免与上层形成循环依赖。

static const char *TAG = "BSP_SERVO";

/**
 * 每通道独立互斥锁（CH_HEAD=0 / CH_L_ARM=1 / CH_R_ARM=2）
 *
 * 设计目标：
 *   - 同一通道同一时刻只允许一个任务驱动，防止 LEDC 写入竞争。
 *   - 不同通道之间互不阻塞，头部与手臂可在不同任务中并发运动。
 *   - interaction worker 和 servo_manager worker 都经此锁，天然线程安全。
 *   - 在 bsp_board_servo_init() 内创建，bsp_servo_move_smooth() 内加/解锁。
 */
static SemaphoreHandle_t s_ch_mutex[3] = {NULL, NULL, NULL};

// ==========================================
// 1. 情绪/动作指令枚举 (对应你 Excel 表格的第一列)
// 注意：此枚举在 interaction.h 中也有相同定义（UI 层使用），
//       bsp_servo.c 内部仅用于舵机动作映射，保持两处同步。
// ==========================================

// 强保护机制：物理边界软限位 (Soft Limits)
// ⚠️ 组装好外壳后，请务必根据实际情况修改这几个极限值！
// 超出范围时 clamp_safe_angle 会自动修正并打印警告日志。
//! 需要修改,以90为0度,左右各80为极限
#define HEAD_MIN_ANGLE 0.0f    ///< 头部向左最大极限角度（度），防止颈部过度旋转损坏舵机
#define HEAD_MAX_ANGLE 180.0f  ///< 头部向右最大极限角度（度）
#define L_ARM_MIN_ANGLE 0.0f   ///< 左臂向后最大极限角度（度），防止手臂撞到机身
#define L_ARM_MAX_ANGLE 180.0f ///< 左臂向前最大极限角度（度），防止撞头
#define R_ARM_MIN_ANGLE 0.0f   ///< 右臂向后最大极限角度（度）
#define R_ARM_MAX_ANGLE 180.0f ///< 右臂向前最大极限角度（度），防止撞头

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
static float clamp_safe_angle(uint8_t channel, float target_angle)
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

    // ── 步骤 0：创建每通道互斥锁（必须在 bsp_servo_move_smooth 首次调用前就绪）──
    // 即使后续硬件 init 失败，锁也已创建；因 servo_initialized=false，
    // bsp_servo_move_smooth 会在加锁前提前返回，不影响正确性。
    for (int i = 0; i < 3; i++)
    {
        if (s_ch_mutex[i] == NULL)
        {
            s_ch_mutex[i] = xSemaphoreCreateMutex();
            if (s_ch_mutex[i] == NULL)
            {
                ESP_LOGE(TAG, "通道 %d 互斥锁创建失败，内存不足!", i);
                bsp_board->servo_initialized = false;
                return;
            }
        }
    }

    // ── 步骤 1：配置 LEDC PWM 舵机参数 ─────────────────────────────────────
    servo_config_t servo_cfg = {
        .max_angle = 180,             // 物理最大行程 180°
        .min_width_us = 500,          // 0° 对应脉宽 500μs（标准舵机规格）
        .max_width_us = 2400,         // 180° 对应脉宽 2400μs
        .freq = 50,                   // PWM 驱动频率 50Hz（标准模拟舵机要求）
        .timer_number = LEDC_TIMER_0, // 使用 LEDC 定时器 0（4 个可选，避免与 LED/蜂鸣器冲突）
        .channels = {
            .servo_pin = {
                BSP_SERVO_HEAD_PIN,  // GPIO38：头部舵机
                BSP_SERVO_L_ARM_PIN, // GPIO47：左臂舵机
                BSP_SERVO_R_ARM_PIN, // GPIO21：右臂舵机
            },
            .ch = {LEDC_CHANNEL_0, LEDC_CHANNEL_1, LEDC_CHANNEL_2}, // 三路独立 LEDC 通道
        },
        .channel_number = 3, // 启用 3 路通道（头 + 左臂 + 右臂）
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
        bsp_servo_move_smooth(CH_HEAD, 90.0f, SERVO_SPEED_MID);  // 头部归中
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
 *   2. 通过 clamp_safe_angle() 将目标角度限制在软限位范围内
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
 * @note 调用者：bsp_board_servo_init()（归中）、interaction worker（情绪动作）、servo_manager worker
 * @note 此函数内部调用 vTaskDelay，必须在 FreeRTOS 任务上下文中调用，不可在中断中使用
 * @note 线程安全：内部持 s_ch_mutex[channel] 互斥锁，同通道串行，不同通道并发安全
 */
void bsp_servo_move_smooth(uint8_t channel, float target, uint32_t step_ms)
{
    bsp_board_t *board = bsp_board_get_instance();

    // ── 前置检查 1：舵机必须已初始化 ─────────────────────────────────────────
    if (board == NULL || !board->servo_initialized)
    {
        ESP_LOGE(TAG, "舵机未就绪，拒绝执行动作指令!");
        return;
    }

    // ── 前置检查 2：通道编号合法且互斥锁已就绪 ──────────────────────────────
    if (channel >= 3 || s_ch_mutex[channel] == NULL)
    {
        ESP_LOGE(TAG, "无效通道 %d 或互斥锁未初始化!", channel);
        return;
    }

    // ── 加锁：独占该通道直到本次运动完成 ─────────────────────────────────────
    // portMAX_DELAY：永久等待，保证请求不丢失（worker task 串行化保证不会长时间持锁）
    xSemaphoreTake(s_ch_mutex[channel], portMAX_DELAY);

    // ── 步骤 1：软限位裁剪（防止超出物理范围损坏机械结构）──────────────────
    float safe_target = clamp_safe_angle(channel, target);

    // ── 步骤 2：读取当前实际角度（iot_servo_read_angle 返回 LEDC 寄存器推算值）──
    float current = 0.0f;
    esp_err_t err = iot_servo_read_angle(LEDC_LOW_SPEED_MODE, channel, &current);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "读取通道 %d 角度失败!", channel);
        xSemaphoreGive(s_ch_mutex[channel]); // 务必在所有提前返回处解锁
        return;
    }

    // ── 步骤 3：抖动死区过滤（差值 < 1° 不运动，消除因浮点精度产生的微抖）──
    if (fabs(safe_target - current) < 1.0f)
    {
        xSemaphoreGive(s_ch_mutex[channel]); // 已在目标位置，解锁后返回
        return;
    }

    // ── 步骤 4：瞬间模式（step_ms == 0，直接写入目标，无平滑过渡）──────────
    if (step_ms == 0)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, safe_target);
        xSemaphoreGive(s_ch_mutex[channel]);
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

    // ── 解锁：本次运动完成，释放通道 ─────────────────────────────────────────
    xSemaphoreGive(s_ch_mutex[channel]);
}

// ==========================================
// API: 三轴同时平滑运动（真正并行）
// ==========================================

/**
 * @brief 三轴舵机同时运动到各自目标角度（线性插值并行）
 *
 * 以三轴中行程最大的轴为步数基准，所有轴在相同时间内同步到达目标，
 * 避免串行调用导致的"头先动完、臂才开始"的割裂感。
 *
 * @param head_target  头部目标角度（度）
 * @param larm_target  左臂目标角度（度）
 * @param rarm_target  右臂目标角度（度）
 * @param step_ms      最长轴每步延时（毫秒），对应 SERVO_SPEED_xxx
 */
void bsp_servo_move_all_parallel(float head_target, float larm_target, float rarm_target, uint32_t step_ms)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL || !board->servo_initialized)
        return;

    for (int i = 0; i < 3; i++)
    {
        if (s_ch_mutex[i] == NULL)
            return;
        xSemaphoreTake(s_ch_mutex[i], portMAX_DELAY);
    }

    float h_safe = clamp_safe_angle(CH_HEAD, head_target);
    float l_safe = clamp_safe_angle(CH_L_ARM, larm_target);
    float r_safe = clamp_safe_angle(CH_R_ARM, rarm_target);

    float h_cur = 0.0f, l_cur = 0.0f, r_cur = 0.0f;
    iot_servo_read_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, &h_cur);
    iot_servo_read_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, &l_cur);
    iot_servo_read_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, &r_cur);

    // 以三轴中行程最大的为总步数，保证同时到达
    int max_steps = (int)fmaxf(fmaxf(fabsf(h_safe - h_cur), fabsf(l_safe - l_cur)), fabsf(r_safe - r_cur));

    if (max_steps < 1 || step_ms == 0)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, h_safe);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, l_safe);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, r_safe);
        for (int i = 0; i < 3; i++)
            xSemaphoreGive(s_ch_mutex[i]);
        return;
    }

    // 线性插值：每步同时写三轴，t 从 1/max_steps 到 1
    for (int step = 1; step <= max_steps; step++)
    {
        float t = (float)step / (float)max_steps;
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, h_cur + t * (h_safe - h_cur));
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, l_cur + t * (l_safe - l_cur));
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, r_cur + t * (r_safe - r_cur));
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }

    // 兜底：精准落在目标位置
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, h_safe);
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, l_safe);
    iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, r_safe);

    for (int i = 0; i < 3; i++)
        xSemaphoreGive(s_ch_mutex[i]);
}
