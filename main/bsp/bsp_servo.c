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
#include <stdatomic.h>         // atomic_bool 打断标志（跨核安全）
#include "freertos/FreeRTOS.h" // pdMS_TO_TICKS
#include "freertos/task.h"     // vTaskDelay（错峰归中用）
#include "freertos/semphr.h"   // 互斥锁，保证多任务调用线程安全
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

/**
 * 舵机运动打断标志（atomic，跨核安全）。
 * 上层（servo_manager_flush）置 true 请求立即中止正在进行的插值运动；
 * bsp_servo_move_all_parallel / bsp_servo_move_smooth 的插值步循环每步检查，
 * 为 true 则立即停在当前角度并退出（不再走完整个行程）。
 * 由上层在「打断后、开始新动作前」清回 false（见 servo_manager worker）。
 * 这样进功能盘 flush 时舵机最坏只滞后一个插值步（step_ms，几十 ms）即停。
 */
static atomic_bool s_servo_abort = ATOMIC_VAR_INIT(false);

void bsp_servo_request_abort(void)
{
    atomic_store(&s_servo_abort, true); // 中断标志
}

void bsp_servo_clear_abort(void)
{
    atomic_store(&s_servo_abort, false);
}

bool bsp_servo_abort_requested(void)
{
    return atomic_load(&s_servo_abort);
}

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

// 单脉冲判定实验开关（2026-07-10 已测完，保留备查）：1=上电只发1~2个90°脉冲后永久断信号。
// ★实验结论（铁证）：断信号后舵机仍一路走完 90° —— 本款舵机为"记忆型"（保持最后目标
//   继续运动），断脉冲不失力。因此小步进/脉冲串等一切软件限速手段对上电归中【无效】，
//   上电回正速度=舵机硬件全速（规格属性，不可调）。要慢速上电归中只能硬件换型
//   （失力型模拟舵机 或 速度可编程的串行总线舵机）。
#define SERVO_SINGLE_PULSE_TEST 0

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
        // 注：servo 组件已收编到 components/servo（2026-07-10）并把 init 初始 duty 改为 0
        // （真正零占空比，不输出任何脉冲）——init 静默、舵机纹丝不动，上电首个指令完全由
        // 下面的软启动脉冲串控制，无需再"掐断 init 自带输出"。

        ESP_LOGI(TAG, "三轴舵机硬件初始化成功!");

#if SERVO_SINGLE_PULSE_TEST
        // ══ 【单脉冲判定实验，测完把宏改回 0】═══════════════════════════════════
        // 目的：一次定性回答"这颗舵机断信号后到底停不停"，终结两种互斥解释：
        //   A. 失力型（标准模拟舵机）：断脉冲即失力 → 只走一小段(~10°)就停 → 脉冲串限速可行
        //   B. 记忆型：断脉冲仍自行走完目标 → 一路走到 90° → 软件限速彻底无解
        // 操作：把头掰离 90°（越远越明显）→ 上电 → 观察舵机走多远。
        // 保障：发完唯一脉冲后【不置 servo_initialized】——后续一切舵机指令（GIF 空闲动作等）
        //   均被丢弃，观察窗口纯净、不限时。测完改回正式逻辑（#else 分支）。
        for (int i = 0; i < 3; i++)
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, (uint8_t)i, 90.0f); // 开始输出 90°
        vTaskDelay(pdMS_TO_TICKS(40)); // 40ms ≈ 保证输出 1~2 个完整 50Hz 脉冲
        for (int i = 0; i < 3; i++)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0); // 永久断信号
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
        }
        ESP_LOGW(TAG, "★单脉冲测试：已发 1~2 个 90° 脉冲并永久断信号。观察舵机：");
        ESP_LOGW(TAG, "★  只走一小段就停 = 失力型（脉冲串可行）｜一路走完90° = 记忆型（软件无解）");
        ESP_LOGW(TAG, "★  测试模式下舵机不再响应任何指令（未置 initialized），测完把宏改回 0");
        // 注意：不置 servo_initialized、不置 SERVO_READY —— 测试模式舵机全程静默
#else
        // ── 步骤 3：上电归中——三路直接持续输出 90° ─────────────────────────────
        //   - 正常开机：固件保证深度待机/关机前三轴已归中 90°（standby.c），物理就在 90°，
        //     脉冲一来纹丝不动，零甩动；
        //   - 断电期间被外力掰歪：上电回正一次（速度取决于单脉冲测试结论：失力型可换回
        //     脉冲串软启动限速；记忆型则为舵机全速，硬件属性不可调）。
        //   - iot_servo_init 已收编改为 duty=0 静默启动（components/servo），首个脉冲的
        //     时机由这里完全掌控。
        for (int i = 0; i < 3; i++)
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, (uint8_t)i, 90.0f);

        // ── 步骤 4：标记初始化成功 + 置位就绪事件 ────────────────────────────────
        bsp_board->servo_initialized = true;
        if (bsp_board->board_status != NULL)
        {
            xEventGroupSetBits(bsp_board->board_status, BOARD_STATUS_SERVO_READY);
        }
#endif // SERVO_SINGLE_PULSE_TEST
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

/**
 * @brief 读取指定舵机通道的当前角度（LEDC 寄存器推算值）
 *
 * 供上层（如 standby.c 进一级低功耗时）按当前角度动态计算归中所需的
 * step_ms，使手臂归中总耗时能与熄屏渐变时长对齐。
 *
 * @note 线程安全：持该通道互斥锁读取，与 bsp_servo_move_smooth 互斥。
 */
bool bsp_servo_read_angle(uint8_t channel, float *out_angle)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL || !board->servo_initialized || out_angle == NULL)
        return false;
    if (channel >= 3 || s_ch_mutex[channel] == NULL)
        return false;

    xSemaphoreTake(s_ch_mutex[channel], portMAX_DELAY);
    float current = 0.0f;
    esp_err_t err = iot_servo_read_angle(LEDC_LOW_SPEED_MODE, channel, &current);
    xSemaphoreGive(s_ch_mutex[channel]);

    if (err != ESP_OK)
        return false;
    *out_angle = current;
    return true;
}

// ==========================================
// API: 舵机低功耗休眠 / 恢复（供二级待机 standby 使用）
// ==========================================

/**
 * @brief 让三轴舵机进入低功耗休眠（停止 PWM 输出，失去保持力矩）
 *
 * 二级（深度）待机省电用：对 CH_HEAD/CH_L_ARM/CH_R_ARM 三路舵机的 LEDC 通道
 * 逐个 ledc_stop()，停止 PWM 脉冲输出。舵机收不到脉冲后会松开保持力矩，
 * 静态电流（尤其堵转/抖动）随之下降，这是舵机省电的主要来源。
 *
 * 设计要点（务必遵守，否则会连累其它外设）：
 *   - 只 stop 舵机自己的三个通道（LEDC_CHANNEL_0/1/2），idle_level=0（引脚拉低）。
 *   - ★绝不触碰共享的 LEDC_TIMER_0 本身：停 timer 会让背光(T1)/马达(T2) 之外
 *     依赖同 speed_mode 时钟的逻辑出问题；停单通道是安全的（参见 BUG-015 教训）。
 *   - 不删除 servo_manager 的 worker 任务/队列：恢复时毫秒级即可，避免任务重建坑。
 *
 * @note 与 bsp_servo_resume() 配对使用；幂等（未就绪直接返回）。
 * @note 调用本函数后，bsp_servo_move_smooth 仍可被调用并自动重新输出（write_angle
 *       内部会重置 duty），但语义上应先 resume 再运动，保持状态清晰。
 */
void bsp_servo_idle(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL || !board->servo_initialized)
        return; // 未就绪：无需停止

    // 逐通道加锁操作，避免与正在进行的插值运动写入竞争（与 move_smooth 同锁）。
    const uint8_t chs[3] = {CH_HEAD, CH_L_ARM, CH_R_ARM};

    // ── 第一步：三路 duty 先拉 0 并 update（引脚变为无脉冲的干净低电平）────────────
    // ★防"停止瞬间抽搐"：直接 ledc_stop 可能在脉冲高电平段拦腰截断，产生一个畸形短脉冲，
    //   舵机会把它解读成一个极端角度、向一侧猛抽一下（实测三轴同时向右甩约 45°）。
    //   先把 duty 归 0，让输出自然变为恒低（对舵机=无信号，不产生任何角度指令）。
    for (int i = 0; i < 3; i++)
    {
        uint8_t ch = chs[i];
        if (s_ch_mutex[ch] == NULL)
            continue;
        xSemaphoreTake(s_ch_mutex[ch], portMAX_DELAY);
        ledc_set_duty(LEDC_LOW_SPEED_MODE, ch, 0);
        ledc_update_duty(LEDC_LOW_SPEED_MODE, ch);
        xSemaphoreGive(s_ch_mutex[ch]);
    }

    // ── 第二步：等一个完整 PWM 周期，确保"最后一个在途脉冲"完整走完 ─────────────
    // 舵机 PWM 为 50Hz（周期 20ms），等 25ms 保证 update 生效且当前周期内的脉冲输出完毕，
    // 之后引脚上已是持续低电平，此时 stop 不可能再截出畸形脉冲。
    vTaskDelay(pdMS_TO_TICKS(25));

    // ── 第三步：真正停通道（idle_level=0，引脚保持低电平）。只动通道，不动 timer ────
    // ★绝不触碰共享的 LEDC_TIMER_0 本身（参见函数头注释与 BUG-015 教训）。
    for (int i = 0; i < 3; i++)
    {
        uint8_t ch = chs[i];
        if (s_ch_mutex[ch] == NULL)
            continue;
        xSemaphoreTake(s_ch_mutex[ch], portMAX_DELAY);
        ledc_stop(LEDC_LOW_SPEED_MODE, ch, 0);
        xSemaphoreGive(s_ch_mutex[ch]);
    }
    ESP_LOGI(TAG, "舵机已进入低功耗休眠（三路 PWM 已停止，duty 先归零防截断抽搐）");
}

/**
 * @brief 从低功耗休眠恢复舵机，并缓慢归中（与 bsp_servo_idle 配对）
 *
 * 唤醒（退出二级待机）时调用：重新让三路 LEDC 通道输出 90° 对应的 PWM 脉冲。
 * iot_servo_write_angle 内部会重新 set_duty + update_duty，自动恢复被 ledc_stop
 * 关掉的通道输出，无需重建 LEDC 配置。
 *
 * 采用慢速归中（SERVO_SPEED_SLOW），避免舵机从“失力松弛位置”猛地跳回 90° 抽搐。
 *
 * @note 幂等：未就绪直接返回。内部走 bsp_servo_move_smooth（自带每通道锁）。
 */
void bsp_servo_resume(void)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (board == NULL || !board->servo_initialized)
        return;

    // ★直接写 90°，不再走 move_smooth 扫描式归中（修"退低功耗舵机先抽到0°再慢慢转回90°"）：
    //   进深度待机前 enter_deep_standby 已保证三轴【先归中 90° 再停 PWM】，物理位置就在 90°。
    //   旧实现 move_smooth 第一步 iot_servo_read_angle 读"当前角"做插值起点——但该函数是拿
    //   LEDC duty 寄存器反算角度，而 bsp_servo_idle 停止前已把 duty 清 0（防截断抽搐），
    //   反算结果恒为 0°→ 插值从假起点 0° 逐度扫到 90°→ 发给舵机的第一个脉冲就是 0°，
    //   物理上舵机（实际在90°）猛跳到 0° 再慢慢扫回 90°，表现为"退出像重启归中"。
    //   现直接写 90°：舵机本来就在 90°，脉冲一来纹丝不动，零跳变、零堵转电流。
    //   保留逐路 + 200ms 错峰（防御：万一某轴被外力掰离 90°，单发 90° 脉冲会产生一次快速
    //   回位，逐路错峰确保任一时刻只有一路可能在动，电流尖峰不叠加）。
    const uint8_t chs[3] = {CH_HEAD, CH_L_ARM, CH_R_ARM};
    for (int i = 0; i < 3; i++)
    {
        uint8_t ch = chs[i];
        if (s_ch_mutex[ch] != NULL)
        {
            xSemaphoreTake(s_ch_mutex[ch], portMAX_DELAY);
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, 90.0f); // 恢复 PWM 输出 90°（write 内部 set_duty+update）
            xSemaphoreGive(s_ch_mutex[ch]);
        }
        if (i < 2)
            vTaskDelay(pdMS_TO_TICKS(200)); // 路间错峰 200ms，摊平可能的瞬时电流尖峰
    }
    ESP_LOGI(TAG, "舵机已从低功耗休眠恢复（直接写90°，无扫描归中）");
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
    bool aborted = false;
    for (int step = 1; step <= max_steps; step++)
    {
        // 每步检查打断请求：进功能盘/强制回主 flush 时立即停在当前角度，
        // 不再走完整个行程（把打断延迟从「一整轮动作」降到「一个 step」≈几十 ms）。
        if (bsp_servo_abort_requested())
        {
            aborted = true;
            break;
        }
        float t = (float)step / (float)max_steps;
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, h_cur + t * (h_safe - h_cur));
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, l_cur + t * (l_safe - l_cur));
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, r_cur + t * (r_safe - r_cur));
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }

    // 兜底：未被打断时精准落在目标位置（被打断则停在当前插值角度，不强制到位）
    if (!aborted)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, h_safe);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, l_safe);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, r_safe);
    }

    for (int i = 0; i < 3; i++)
        xSemaphoreGive(s_ch_mutex[i]);
}
