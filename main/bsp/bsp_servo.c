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
#include "esp_timer.h"         // 【诊断·验证完可随诊断日志一并删除】esp_timer_get_time 测拿锁耗时
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

/**
 * 插值运动的固定帧间隔（毫秒）—— 所有平滑运动统一按此节拍写入 PWM。
 *
 * 【为什么是 20ms】
 *   1. 与舵机 50Hz PWM 周期严格对齐：舵机每 20ms 才采样一个脉冲，写得比这更密纯属浪费；
 *   2. tick=100Hz（CONFIG_FREERTOS_HZ=100，sdkconfig:1740）下 20ms = 恰好 2 个 tick，
 *      pdMS_TO_TICKS 无截断误差。★对比旧版：step_ms=2/5 时 pdMS_TO_TICKS 整除后为 0，
 *      FAST/VERY_FAST 实际【完全不延时】= 全速到位，平滑形同虚设；
 *   3. 更新率恒定 50Hz，高于人眼闪烁融合阈值，慢速档不再有「一步一步走」的观感。
 *
 * 【速度语义不变】各档 SERVO_SPEED_xxx 仍是「毫秒/度」，总耗时 = 行程度数 × step_ms，
 *   本宏只决定这段时间被切成多少帧，不改变快慢。例：VERY_SLOW(50) 走 30°：
 *     旧 = 30 帧 × 50ms（每帧 1.0°，20Hz）｜新 = 75 帧 × 20ms（每帧 0.4°，50Hz），均 1500ms。
 */
#define SERVO_FRAME_MS 40U

/**
 * 运行时帧长覆盖（★抖动排查用，定位完可连同 bsp_servo_debug_set_frame_ms 一并删除）
 *
 * ★必须定义在 SERVO_FRAME_MS 宏【之后】：下面的 servo_frame_ms() 要用到它，
 *   放在文件更靠前的位置会报 'SERVO_FRAME_MS' undeclared（2026-09-03 踩过）。
 *
 * 【为什么需要它】SERVO_FRAME_MS 是编译期宏，要对比两种帧长只能烧两次板，
 *   靠记忆比较"这次比上次抖不抖"极不可靠。做成运行时变量后，同一次烧录里
 *   就能让两种帧长交替出现，眼睛直接对比。
 *
 * 【为什么改帧长而不改速度】总耗时 = 行程 × step_ms，与帧长无关。
 *   所以改帧长【不改变动作快慢】，只改变这段时间被切成多少帧，
 *   即每帧位移 = 帧长 ÷ step_ms。对比时唯一变量就是每帧位移，
 *   不会混入"变快了所以看不出抖"的干扰。
 *
 * 0 = 沿用编译期的 SERVO_FRAME_MS（默认，业务行为完全不变）。
 */
static _Atomic uint32_t s_frame_ms_override = 0;

/** 取本次运动实际使用的帧长（override 为 0 时用编译期常量） */
static inline uint32_t servo_frame_ms(void)
{
    uint32_t v = atomic_load(&s_frame_ms_override);
    return (v == 0U) ? (uint32_t)SERVO_FRAME_MS : v;
}

void bsp_servo_debug_set_frame_ms(uint32_t frame_ms)
{
    atomic_store(&s_frame_ms_override, frame_ms);
}

// 单脉冲判定实验开关（2026-07-10 已测完，保留备查）：1=上电只发1~2个90°脉冲后永久断信号。
// ★实验结论（铁证）：断信号后舵机仍一路走完 90° —— 本款舵机为"记忆型"（保持最后目标
//   继续运动），断脉冲不失力。因此小步进/脉冲串等一切软件限速手段对上电归中【无效】，
//   上电回正速度=舵机硬件全速（规格属性，不可调）。要慢速上电归中只能硬件换型
//   （失力型模拟舵机 或 速度可编程的串行总线舵机）。
#define SERVO_SINGLE_PULSE_TEST 0

/* ══ 死区标定实验开关（2026-09-03 新增，★测完务必改回 0）════════════════════
 *
 * 【为什么要做这个实验】要治「慢速转动时的轻微抖动」，必须先知道舵机死区的真值。
 *   死区 = 目标角与实际角相差多少度以内，舵机当没看见、电机不通电。
 *   慢速时每帧位移小于死区，舵机就得攒好几帧才跨过去，一跨过就用硬件全速冲完，
 *   于是运动变成「跳一下、停一会、再跳一下」，这就是抖动的观感来源。
 *
 *   跳动频率 = 运动速度 ÷ 死区。VERY_SLOW 是 20°/s：
 *       死区 1.00° → 20 次/秒 ← 正好落在人眼最敏感的 10~30 次/秒，看得很清楚
 *       死区 0.47° → 43 次/秒 ← 远离敏感区，就不该这么明显
 *   MG90S 规格书写的是约 0.47°，本文件注释里沿用的是约 1°，差一倍结论完全相反。
 *   ★所以这个数必须实测，它是后续一切方案的地基，不能靠猜。
 *
 * ══ 2026-09-03 往复法实测结果（模式 1）════════════════════════════════════
 *   ★三个轴都要到 【3.0°】 才出现正常往复。硬件：左右臂 SG90（塑料齿），
 *     头部 MG90S（金属齿）。两种型号阈值一致，且都是空载。
 *
 *   3° 是规格书 0.47° 的六倍多，这个差距不可能全是电气死区，**主要是齿隙**：
 *   往复运动每次换向都要先让电机空转吃掉齿间空回，输出轴才跟着动，所以
 *       往复法测到的 = 电气死区 + 齿隙        ← 已测得 ≈ 3.0°
 *       单向法测到的 = 电气死区（不换向，齿轮始终贴同一侧，不吃齿隙）← 待测
 *
 *   ★为什么必须再测单向：真实的慢速运动是【朝一个方向连续走】的，中途不换向，
 *     所以它根本不吃齿隙，受制的只有电气死区。拿 3° 去推算抖动会严重高估。
 *     若电气死区真是 0.47°，VERY_SLOW(20°/s) 的跳动频率就是 43 次/秒，
 *     远离人眼敏感区，那抖动就另有原因，整个排查方向要换。
 *
 * 【模式 1·往复放大法】以 90° 为基准来回往复，幅度逐级放大，第一个能被
 *   【看见/听见/摸到】规律往复的幅度 = 电气死区 + 齿隙。
 *
 *   ★为什么不用"单向逐步推进、看它第几步动"：0.2° 换算到舵机臂上只有零点几毫米，
 *     即便跨过死区跳了 1°，也不过 0.3mm，肉眼根本分辨不出来（2026-09-03 实测踩过这个坑）。
 *     而人眼、耳朵、手指对【有节奏的往复】极其敏感，同样的微小幅度立刻就能察觉。
 *
 * 【模式 2·单向微步法】测纯电气死区，验证规格书的 0.47° 到底能不能达到。
 *   ★关键设计：不靠"看位移大小"，靠"听节奏快慢"。人对时间节奏的分辨力，
 *     远高于对零点几毫米位移的分辨力，这是模式 1 的教训换来的。
 *
 *   每组固定步间隔 250ms 单向微步推进，只改步长 δ：
 *       δ ≥ 电气死区 → 每一步都跨得过门槛 → 每 250ms 响一次，节奏均匀密集
 *       δ < 电气死区 → 要攒 k 步才跨过去 → 每 k×250ms 才响一次，节奏明显稀疏
 *   ★所以你只需要判断一件事：这一组的咔咔声是不是【每 250ms 一次】。
 *     从哪一档开始变成每 250ms 一次，那一档就是电气死区。
 *
 *   每组开头有个预置动作：先退 6° 再正向走回起点。这一进一退是为了让齿轮
 *   【贴紧正向一侧】，后续微步全部同向，就绕开了齿隙，测到的才是纯电气死区。
 *
 * 【安全性】本模式下【故意不置 servo_initialized】，与上面 SERVO_SINGLE_PULSE_TEST
 *   同一套路：一切上层舵机指令（情绪动作、空闲动作、待机归中）都会被
 *   bsp_servo_move_smooth 的前置检查拒绝，观察窗口纯净，不会被业务动作干扰。
 */
#define SERVO_DEADBAND_CALIB_TEST 0

/** 标定模式：1 = 往复放大法（测 死区+齿隙，已测得 3.0°）
 *            2 = 单向微步法（测 纯电气死区，验证 0.47° 能否达到）★当前 */
#define SERVO_CALIB_MODE 2

// ── 模式 1（往复放大法）参数 ──────────────────────────────────────────────
#define CALIB_BASE_DEG 90.0f ///< 往复基准角（中位，远离两端，避免任何限位干扰）
#define CALIB_CYCLES 6       ///< 每个幅度往复几次。次数够多才能看出"有节奏"而非偶发
#define CALIB_HALF_MS 400    ///< 往复半周期（毫秒）。400ms 一来一回=1.25Hz，最容易被眼睛捕捉

/* 幅度序列（度）：从远小于死区，逐级放大到远大于死区。
 * ★第一个能让你【看见/听见/摸到】规律往复的幅度 = 电气死区 + 齿隙。
 * 2026-09-03 实测三轴均在 3.0° 这一档才正常往复，故在 2 与 3 之间补一档 2.5，
 * 把阈值夹得更细——若 2.5 也动，说明真值落在 2.0~2.5 而非 2.5~3.0。 */
#define CALIB_AMP_LIST {0.2f, 0.4f, 0.6f, 0.8f, 1.0f, 1.5f, 2.0f, 2.5f, 3.0f}

// ── 模式 2（单向微步法）参数 ──────────────────────────────────────────────
// ★单向法一次只测一个轴：判据是"听节奏"，三轴同响会糊成一片分辨不出。
//   头部是 MG90S，左右臂是 SG90，两种型号都要各测一次再对比。
#define CALIB_UNI_CHANNEL CH_HEAD    ///< 被测轴：先 CH_HEAD(MG90S)，再改 CH_L_ARM(SG90) 复测
#define CALIB_UNI_BASE_DEG 80.0f     ///< 单向推进的起点（留出正向 48° 余量，不碰限位）
#define CALIB_UNI_PREP_BACK_DEG 6.0f ///< 预置回退量：先退这么多再正向走回起点，把齿轮压向正侧
// 步间隔 500ms（2026-09-03 由 250 放宽）：250ms=4Hz 太快，人来不及数清"响了几次"；
// 500ms=2Hz 既能听出均匀节奏，跨不过死区时（每 1000/1500ms 一次）的稀疏感也更刺耳好认。
#define CALIB_UNI_INTERVAL_MS 500      ///< 步间隔（毫秒）。这是判据基准，务必固定不要改
#define CALIB_UNI_MAX_STEPS 16         ///< 每组最多走几步（小步长组靠它兜底，免得一组太长）
#define CALIB_UNI_MAX_TRAVEL_DEG 12.0f ///< 每组最大总位移（大步长组靠它兜底，免得撞限位）

/* 单向步长序列（度）：0.47 是 SG90/MG90S 规格书的死区带宽 5μs 换算值，是本次要验证的靶心。
 *
 * ★前两档是【对照组】，不是测量组，作用是先让你知道"该听到什么"：
 *     0.0  阴性对照：连续写同一个角度，舵机【绝对不该动】。
 *          若这组你也觉得"动了"，说明看到的是每组开头的预置动作或残余振动，
 *          观察方法本身有问题，后面所有读数都不可信 —— 这一档专门用来暴露它。
 *     3.0  阳性对照：已知必定每步都动（往复法实测 3° 一定动），
 *          用它建立"每 500ms 响一次"到底是什么节奏感，作为后面各档的比对基准。
 *
 * ★2026-09-03 教训：上一轮只给未知档位，用户用"动没动"来判断，
 *   而 16 步累计 3.2° 是肉眼可见的，导致每一档看起来都"动了"，无法分辨。
 *   判据必须是【节奏疏密】而非【动没动】，对照组就是为了把这件事讲清楚。 */
#define CALIB_UNI_STEP_LIST {0.0f, 3.0f, 0.2f, 0.3f, 0.47f, 0.6f, 0.8f, 1.0f, 1.5f, 2.0f}

/* ══ 2026-09-03 单向法实测结果（模式 2，被测轴 CH_HEAD / MG90S）════════════
 *   0.2° / 0.3° / 0.47° / 0.6°  → 听起来「连齿」，不是干净的一步一响
 *   0.8° / 1.0° / 1.5° / 2.0°   → 正常，一点点地动，节奏干净
 *   ⇒ ★纯电气死区（含最小可分辨步长）≈ 0.8°，落在 0.6~0.8 之间。
 *
 * 【与两个已知数对照】
 *   规格书 0.47°（5μs 死区带宽）→ ❌ 达不到。实测要 0.8° 才吃得动，约为规格的 1.7 倍。
 *   往复法 3.0°（死区+齿隙）    → 齿隙 ≈ 3.0 - 0.8 = 2.2°，且 SG90/MG90S 都一样。
 *
 * 【已排除：不是 duty 量化造成的】曾怀疑 0.2/0.3 那几档是 LEDC 取整不均匀导致，
 *   实算否定：14bit@50Hz 分辨率 0.111°/级，0.2° 的实际步进在 0.111 与 0.222 间跳
 *   （最大/最小 = 2.0，确实不均），但 0.47° 与 1.5° 的量化是【完全均匀】的
 *   （每步恰好 0.444 / 1.444），却照样"连齿"；而 0.8° 的量化反倒没那么均匀
 *   （0.778 与 0.667 混合），却听着正常。量化均匀度与好坏不相关 ⇒ 病因是物理的，
 *   不是数值的。★别再回头查 duty 取整。
 *
 * 【"连齿"是什么】步长小于死区时，舵机攒好几步才跨过一次门槛，跨过时电机以硬件
 *   全速冲完这一小段再停。于是电机在"起停—起停"之间反复冲击齿轮，齿面在齿隙内
 *   来回磕碰，听感就是连续的齿轮咯噔声，而不是干净的一步一响。
 *   ★这正是慢速运动抖动的现场还原：VERY_SLOW 每帧 0.4°、SLOW 每帧 0.67°，
 *     全都落在 0.8° 门槛以下，所以业务动作跑慢档时必然复现同样的"连齿+抖动"。
 *
 * 【由此得到的硬结论】要让运动不抖，每帧位移必须 ≥ 0.8°。而每帧位移 = 20/step_ms，
 *   于是 step_ms ≤ 25 才安全：
 *       VERY_SLOW(50) 每帧 0.40° ✘ 抖  ｜ SLOW(30) 每帧 0.67° ✘ 抖
 *       SLOWER(25)    每帧 0.80° ✔ 临界｜ MID(15)  每帧 1.33° ✔ 正常
 *   ⇒ 软件在死区以下再怎么细分插值都是无效的（这也解释了 09-02 一阶低通为何失败）。
 */

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

#if SERVO_DEADBAND_CALIB_TEST && (SERVO_CALIB_MODE == 1)
/**
 * @brief 死区标定实验任务（★验证完连同 SERVO_DEADBAND_CALIB_TEST 宏一并删除）
 *
 * 只做一件事：被测轴以 90° 为基准来回往复，幅度从 0.2° 逐级放大到 3.0°，
 * 每个幅度往复 6 次（一来一回 0.8 秒），组间静默 1.5 秒便于分辨。全程约 50 秒。
 * ★你要记的只有一个数：从哪个幅度开始，舵机才第一次动。那个幅度就是死区。
 *
 * @note 直接调 iot_servo_write_angle 而不走 bsp_servo_move_smooth，
 *       绕开软限位、死区过滤、互斥锁与插值，确保写下去的就是我想要的裸角度。
 * @note 普通 xTaskCreate（栈在内部 SRAM）：任务内要碰 LEDC 寄存器，
 *       不能像某些业务任务那样把栈放 PSRAM。跑完 vTaskDelete(NULL) 自删，不驻留。
 */
static void servo_deadband_calib_task(void *arg)
{
    (void)arg;
    static const float amps[] = CALIB_AMP_LIST;
    const int amp_n = (int)(sizeof(amps) / sizeof(amps[0]));
    const uint8_t chs[3] = {CH_HEAD, CH_L_ARM, CH_R_ARM}; // 三轴一起摆，并排放着好对比

    ESP_LOGW(TAG, "════════ 死区标定实验（往复放大法·三轴同步）════════");
    ESP_LOGW(TAG, "三轴同时摆  基准角=%.1f°  每组往复%d次  半周期%dms",
             CALIB_BASE_DEG, CALIB_CYCLES, CALIB_HALF_MS);
    ESP_LOGW(TAG, "★三种感官任选，灵敏度由低到高：");
    ESP_LOGW(TAG, "★  ①【看】舵机臂有没有在规律地来回摆（幅度小时看不见很正常）");
    ESP_LOGW(TAG, "★  ②【听】有没有节奏均匀的齿轮咔咔声（比眼睛灵敏得多）");
    ESP_LOGW(TAG, "★  ③【摸】手指轻搭在舵机臂或外壳上，感受有没有规律的震动（最灵敏）");
    ESP_LOGW(TAG, "★三轴并排，请分别记下【头/左臂/右臂各自从第几组开始动】");
    ESP_LOGW(TAG, "★  三轴阈值一致 → 共性问题（电源或这批舵机的通病）");
    ESP_LOGW(TAG, "★  只有某一轴偏大 → 那一颗的个体毛病，换掉即可");

    // 三轴先到基准角，静置等停稳，再开始逐级往复
    for (int c = 0; c < 3; c++)
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, chs[c], CALIB_BASE_DEG);
    ESP_LOGW(TAG, "三轴已到基准角 %.1f°，静置 3 秒后开始，从最小幅度 0.2° 起", CALIB_BASE_DEG);
    vTaskDelay(pdMS_TO_TICKS(3000));

    // ── 逐级放大幅度，每级往复若干次 ────────────────────────────────────────
    // 【为什么用往复而不是单向递增】单向 0.2° 在舵机臂上只有零点几毫米位移，
    //   肉眼根本分辨不出来；而往复运动人眼极其敏感，哪怕幅度同样只有零点几毫米，
    //   只要它在有节奏地动，就能被看见、听见或摸到。这是本实验能成立的关键。
    for (int i = 0; i < amp_n; i++)
    {
        const float amp = amps[i];
        ESP_LOGW(TAG, "──── 第%d/%d组  幅度 %.1f°  （%.1f° ↔ %.1f°）持续约%.1f秒 ────",
                 i + 1, amp_n, amp, CALIB_BASE_DEG, CALIB_BASE_DEG + amp,
                 (float)(CALIB_CYCLES * CALIB_HALF_MS * 2) / 1000.0f);

        for (int k = 1; k <= CALIB_CYCLES; k++)
        {
            // 三轴在同一时刻写同一个目标角，同摆同停，便于并排横向对比
            for (int c = 0; c < 3; c++)
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, chs[c], CALIB_BASE_DEG + amp);
            vTaskDelay(pdMS_TO_TICKS(CALIB_HALF_MS));

            for (int c = 0; c < 3; c++)
                iot_servo_write_angle(LEDC_LOW_SPEED_MODE, chs[c], CALIB_BASE_DEG);
            vTaskDelay(pdMS_TO_TICKS(CALIB_HALF_MS));
        }

        ESP_LOGW(TAG, "     ↑第%d组结束。刚才若在规律往复 → 有效死区 ≤ %.1f°", i + 1, amp);
        vTaskDelay(pdMS_TO_TICKS(1500)); // 组间静默，便于分辨"上一组"和"下一组"
    }

    // ── 结果解读 ────────────────────────────────────────────────────────────
    ESP_LOGW(TAG, "════════ 标定结束，请按下面解读 ════════");
    ESP_LOGW(TAG, "★每一轴第一个出现规律往复的幅度 = 那一轴的有效死区（含齿隙）");
    ESP_LOGW(TAG, "★三轴阈值接近  → 共性问题，指向电源或这批舵机的通病");
    ESP_LOGW(TAG, "★某一轴明显偏大 → 那一颗的个体毛病，换掉即可");
    ESP_LOGW(TAG, "★留意第4组(0.8°)：它正是 SLOWER 档跑 180° 时的每帧位移，");
    ESP_LOGW(TAG, "★  该组不动就说明慢速运动时舵机每帧都跨不过门槛，只能攒几帧猛跳一次");
    ESP_LOGW(TAG, "★注意：往复测法测到的是【死区+齿隙】，不是纯电气死区，两者要靠单向测法分离");
    ESP_LOGW(TAG, "★测完把 SERVO_DEADBAND_CALIB_TEST 改回 0 才能恢复正常业务");

    vTaskDelete(NULL); // 一次性任务，跑完自删，不占常驻栈
}
#endif // SERVO_DEADBAND_CALIB_TEST && MODE 1

#if SERVO_DEADBAND_CALIB_TEST && (SERVO_CALIB_MODE == 2)
/**
 * @brief 单向微步标定任务：测【纯电气死区】，验证规格书 0.47° 能否达到
 *        （★验证完连同 SERVO_DEADBAND_CALIB_TEST 宏一并删除）
 *
 * 【与模式 1 的本质区别】模式 1 是往复，每次换向都要吃齿隙，测到的是
 *   「电气死区 + 齿隙」（已实测 3.0°）。本模式全程只朝一个方向推进，
 *   齿轮始终贴在同一侧，齿隙不参与，测到的才是纯电气死区。
 *
 * 【判据：听节奏，不是看位移】每组步间隔固定 CALIB_UNI_INTERVAL_MS(250ms)，只改步长 δ：
 *     δ ≥ 电气死区 → 每步都跨得过门槛 → 每 250ms 响一次，密集均匀
 *     δ < 电气死区 → 攒 k 步才跨过去 → 每 k×250ms 响一次，明显稀疏
 *   ★从哪一档开始变成「每 250ms 一次」，那一档就是电气死区。
 *   这个判据把"分辨零点几毫米位移"换成了"分辨节奏快慢"，后者人类灵敏得多。
 *
 * @note 一次只测一个轴（CALIB_UNI_CHANNEL）：三轴同响会糊成一片，节奏判据失效。
 *       头部 MG90S 测完，把宏改成 CH_L_ARM 再测一遍 SG90，两种型号对比。
 * @note 直接调 iot_servo_write_angle，绕开软限位/死区过滤/互斥锁/插值，
 *       确保写下去的就是我想要的裸角度。
 */
static void servo_unidir_calib_task(void *arg)
{
    (void)arg;
    static const float steps[] = CALIB_UNI_STEP_LIST;
    const int step_n = (int)(sizeof(steps) / sizeof(steps[0]));
    const uint8_t ch = CALIB_UNI_CHANNEL;

    ESP_LOGW(TAG, "════════ 单向微步标定（测纯电气死区）════════");
    ESP_LOGW(TAG, "被测轴=%u（0=头MG90S 1=左臂SG90 2=右臂SG90）  起点=%.1f°  步间隔=%dms",
             (unsigned)ch, CALIB_UNI_BASE_DEG, CALIB_UNI_INTERVAL_MS);
    ESP_LOGW(TAG, "★★判据【不是】动没动，是【响了几次】★★");
    ESP_LOGW(TAG, "★  不管步长多小，一组走完的累计位移都一样大，光看'动没动'分辨不出任何东西。");
    ESP_LOGW(TAG, "★  要数的是：微步期间舵机响/震了几次。满 N 次(每%dms一次，均匀)=跨得过死区；",
             CALIB_UNI_INTERVAL_MS);
    ESP_LOGW(TAG, "★  明显不足 N 次(节奏稀疏、一顿一顿)=跨不过，要攒好几步才动一次。");
    ESP_LOGW(TAG, "★  0.2° 在舵机臂上只有约 0.07mm，单步肉眼绝对看不见，【必须靠手指摸或耳朵听】");
    ESP_LOGW(TAG, "★前两组是对照组：第1组步长0(绝对不该动)、第2组步长3°(必定每步都动)，");
    ESP_LOGW(TAG, "★  先用它们校准你的感觉，再看后面的未知档位。");
    ESP_LOGW(TAG, "★每组开头会先退%.0f°再正向走回起点（日志会打'预置中'），那是把齿轮压向正侧，",
             CALIB_UNI_PREP_BACK_DEG);
    ESP_LOGW(TAG, "★  是 6° 的大动作，比测试步长大三十倍，【千万别把它当成测试结果】。");
    ESP_LOGW(TAG, "★  等日志打出'微步开始'再开始数。");

    for (int i = 0; i < step_n; i++)
    {
        const float d = steps[i];

        // 步数：总位移不超过 CALIB_UNI_MAX_TRAVEL_DEG，且不超过 CALIB_UNI_MAX_STEPS 步。
        // 小步长受步数上限约束（免得一组拖太久），大步长受位移上限约束（免得撞限位）。
        // ★d==0 是阴性对照组，不能拿它做除数，直接固定走满步数（反复写同一个角度）。
        int n;
        if (d <= 0.0f)
        {
            n = CALIB_UNI_MAX_STEPS;
        }
        else
        {
            n = (int)(CALIB_UNI_MAX_TRAVEL_DEG / d);
            if (n > CALIB_UNI_MAX_STEPS)
                n = CALIB_UNI_MAX_STEPS;
            if (n < 2)
                n = 2;
        }

        // ── 本组抬头：先播报参数，再区分对照组 / 测量组 ──────────────────────
        ESP_LOGW(TAG, "──── 第%d/%d组  步长 %.2f°  共%d步  总位移%.1f°  持续%.1f秒 ────",
                 i + 1, step_n, d, n, (float)n * d,
                 (float)(n * CALIB_UNI_INTERVAL_MS) / 1000.0f);
        if (d <= 0.0f)
            ESP_LOGW(TAG, "     【阴性对照】步长为0，舵机绝对不该动。动了=你看错了对象");
        else if (i == 1)
            ESP_LOGW(TAG, "     【阳性对照】必定每步都动，记住这个节奏，后面拿它做基准");

        // ── 预置：先退再正向走回起点，让齿轮贴紧【正向】一侧 ──────────────────
        // ★这是单向法成立的前提：后续微步全部同向，齿隙已在这一步被吃掉，
        //   不会混进测量结果。少了这一步，第一步的读数仍会包含齿隙。
        // ★它是 6° 大动作，比测试步长大三十倍，最容易被误当成测试结果，
        //   所以打日志明确划界，并留 1.5 秒静默把它和微步在时间上分开。
        ESP_LOGW(TAG, "     预置中…（先退%.0f°再走回，把齿轮压向正侧，这不是测试内容）",
                 CALIB_UNI_PREP_BACK_DEG);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, CALIB_UNI_BASE_DEG - CALIB_UNI_PREP_BACK_DEG);
        vTaskDelay(pdMS_TO_TICKS(700));
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, CALIB_UNI_BASE_DEG);
        vTaskDelay(pdMS_TO_TICKS(1500)); // 等它彻底停稳，余振不会被误当成响应

        ESP_LOGW(TAG, "     ★微步开始（%d步 × %dms），从现在起听节奏", n, CALIB_UNI_INTERVAL_MS);
        for (int k = 1; k <= n; k++)
        {
            iot_servo_write_angle(LEDC_LOW_SPEED_MODE, ch, CALIB_UNI_BASE_DEG + (float)k * d);
            vTaskDelay(pdMS_TO_TICKS(CALIB_UNI_INTERVAL_MS));
        }

        ESP_LOGW(TAG, "     ↑第%d组微步结束（步长%.2f°）。刚才响了几次？满%d次=跨得过门槛",
                 i + 1, d, n);
        vTaskDelay(pdMS_TO_TICKS(2000)); // 组间静默，便于分辨上一组和下一组
    }

    ESP_LOGW(TAG, "════════ 单向标定结束，请按下面解读 ════════");
    ESP_LOGW(TAG, "★第一个「每%dms响一次」的步长 = 该轴纯电气死区", CALIB_UNI_INTERVAL_MS);
    ESP_LOGW(TAG, "★把它与往复法的 3.0° 相减，差值就是齿隙（纯机械量，软件消不掉）");
    ESP_LOGW(TAG, "★若 0.47° 那组就已经密集 → 规格达标，慢速抖动【另有原因】，方向要换");
    ESP_LOGW(TAG, "★若要到 1.5° 以上才密集 → 电气死区本身就大，慢速走停是物理必然");
    ESP_LOGW(TAG, "★换 CALIB_UNI_CHANNEL 为 CH_L_ARM 再测一遍，对比 SG90 与 MG90S");
    ESP_LOGW(TAG, "★测完把 SERVO_DEADBAND_CALIB_TEST 改回 0 才能恢复正常业务");

    vTaskDelete(NULL); // 一次性任务，跑完自删，不占常驻栈
}
#endif // SERVO_DEADBAND_CALIB_TEST && MODE 2

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
        vTaskDelay(pdMS_TO_TICKS(40));                                     // 40ms ≈ 保证输出 1~2 个完整 50Hz 脉冲
        for (int i = 0; i < 3; i++)
        {
            ledc_set_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i, 0); // 永久断信号
            ledc_update_duty(LEDC_LOW_SPEED_MODE, (ledc_channel_t)i);
        }
        ESP_LOGW(TAG, "★单脉冲测试：已发 1~2 个 90° 脉冲并永久断信号。观察舵机：");
        ESP_LOGW(TAG, "★  只走一小段就停 = 失力型（脉冲串可行）｜一路走完90° = 记忆型（软件无解）");
        ESP_LOGW(TAG, "★  测试模式下舵机不再响应任何指令（未置 initialized），测完把宏改回 0");
        // 注意：不置 servo_initialized、不置 SERVO_READY —— 测试模式舵机全程静默
#elif SERVO_DEADBAND_CALIB_TEST
        // ══ 【死区标定实验，测完把 SERVO_DEADBAND_CALIB_TEST 改回 0】═════════════
        // 与上面单脉冲实验同一套路：【故意不置 servo_initialized】，于是
        // bsp_servo_move_smooth 的前置检查会拒绝一切上层指令（情绪动作、空闲动作、
        // 待机归中），观察窗口纯净不受业务干扰。标定任务直接写裸角度。
#if SERVO_CALIB_MODE == 1
        xTaskCreate(servo_deadband_calib_task, "servo_calib", 4096, NULL, 5, NULL);
        ESP_LOGW(TAG, "★死区标定模式1【往复放大法】已启动：测 电气死区+齿隙");
#else
        xTaskCreate(servo_unidir_calib_task, "servo_calib", 4096, NULL, 5, NULL);
        ESP_LOGW(TAG, "★死区标定模式2【单向微步法】已启动：测 纯电气死区，靶心 0.47°");
#endif
        ESP_LOGW(TAG, "★标定期间舵机不再响应任何业务指令，请看后续标定日志");
        // 注意：不置 servo_initialized、不置 SERVO_READY —— 标定模式全程独占舵机
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

    // ── 步骤 5：定帧线性插值运动（固定 SERVO_FRAME_MS 一帧，步长按速度自动缩放）────
    // 【为什么不用「1°/step_ms」】旧写法每步固定走 1°，更新率 = 1000/step_ms：
    //   VERY_SLOW(50ms/度) 时只有 20Hz，低于人眼闪烁融合阈值 → 肉眼可见「一步一步走」。
    //   且 tick=100Hz 下 pdMS_TO_TICKS(2)/(5) 整除后为 0，FAST/VERY_FAST 实际不延时=全速。
    // 【现在】固定 20ms 一帧（与 50Hz 舵机 PWM 周期对齐，=2 tick 可精确表达），
    //   总耗时仍按 step_ms 语义计算（total_deg × step_ms），故各档速度观感不变，
    //   变的只是把同一段行程切得更细：VERY_SLOW 由 1°/50ms 变为 0.4°/20ms，更新率 20→50Hz。
    // 【2026-09-02 回退记录】此处曾改为一阶低通滤波（set += α(target-set)，对齐参考实现的
    //   「方案三」），实测【无效】：慢速抖动依旧，且因收尾段每帧位移衰减到远小于舵机死区
    //   （MG90S 约 ±1°），出现「中段快、末段原地磨蹭不动」的新问题，比线性更差，故回退。
    float total_deg = fabsf(safe_target - current);
    uint32_t total_ms = (uint32_t)(total_deg * (float)step_ms); // 维持原速度语义
    const uint32_t frame_ms = servo_frame_ms(); // ★抖动排查：可被运行时覆盖，默认=SERVO_FRAME_MS
    int frames = (int)(total_ms / frame_ms);
    if (frames < 1)
        frames = 1; // 行程极短时至少走一帧，保证必定到位

    // ★用 vTaskDelayUntil 绝对定时（2026-09-03 抖动修复，理由详见
    //   bsp_servo_move_all_parallel 插值循环处的完整注释）：
    //   vTaskDelay 是相对延时，被抢占多久就多等多久，误差逐帧累积，
    //   帧间隔抖动导致写入时刻相对 PWM 周期漂移，脉宽序列不匀 → 运动抖动。
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frame_ticks = pdMS_TO_TICKS(frame_ms);

    bool aborted = false;
    for (int f = 1; f <= frames; f++)
    {
        // 每帧检查打断请求：远程控制抢占 / 进功能盘 flush 时立即停在当前角度，
        // 不再走完整个行程（打断延迟 = 一帧 20ms，比旧版一个 step 更灵敏）。
        // 与 bsp_servo_move_all_parallel 的 aborted 处理保持同一套语义。
        if (bsp_servo_abort_requested())
        {
            aborted = true;
            break;
        }
        float t = (float)f / (float)frames; // 归一化进度 0~1
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, current + t * (safe_target - current));
        vTaskDelayUntil(&last_wake, frame_ticks); // 恒定帧间隔（绝对定时，不累积抢占误差）
    }

    // ── 步骤 6：兜底对齐（确保最终精准停在目标位置，消除循环步进的浮点累积误差）──
    // ★被打断时【必须跳过】：否则这一笔会直接把目标角 PWM 写下去，舵机靠机械惯性
    // 一路转到目标位，打断等于白做。被打断则停在当前插值角度，由后续动作从此处接管。
    if (!aborted)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, safe_target);
    }

    // ── 解锁：本次运动完成，释放通道 ─────────────────────────────────────────
    xSemaphoreGive(s_ch_mutex[channel]);
}

/**
 * @brief 抢占式平滑运动：拿到通道锁【之后】才清打断标志，随即插值
 *
 * 与 bsp_servo_move_smooth 的【唯一】差别：在 xSemaphoreTake 成功之后、插值之前，
 * 多调一次 bsp_servo_clear_abort()。其余逻辑逐行相同。
 *
 * 【为什么需要它】抢占方（remote_control 的 rc_worker）原本的写法是：
 *     bsp_servo_clear_abort();          // ① 清标志
 *                                       // ② ← 窗口：此处可被抢占
 *     bsp_servo_move_smooth(...);       // ③ 进函数后才拿锁
 * ①③ 之间不是原子的。窗口 ② 里若 servo_manager worker 恰好取到新请求，它会先清标志、
 * 再开始新动作并【持有通道锁】；等 rc_worker 走到 ③ 时锁已被占，只能阻塞等待，而拿到锁
 * 时新动作可能又置了标志 → 插值首步 break → 手臂停在半路不动。窗口只有几个调度周期宽，
 * 故表现为约 1/10 的偶发「下发后停住、再点一次才到位」（2026-08-05 实测）。
 * 本函数把顺序倒过来：【先拿锁 → 再清标志 → 立即插值】，清标志时锁已在手，
 * servo_manager worker 即使想开始新动作也拿不到锁，窗口消失。
 *
 * 【为什么不直接改 bsp_servo_move_smooth】那个函数被 servo_manager worker 自己调用，
 * 若改成「进函数就清打断标志」，flush 打断机制会整个失效（worker 一进去就把上层刚置的
 * 标志抹掉）。故只能给抢占方单开入口，原函数及其全部调用方保持不动。
 *
 * @param channel  舵机通道（CH_HEAD / CH_L_ARM / CH_R_ARM）
 * @param target   目标绝对角（度，受软限位裁剪）
 * @param step_ms  步进延时（毫秒/度），0 = 瞬间模式
 * @note 仅供「抢占后立即接管」的场景使用（当前唯一调用方：remote_control 的 rc_worker）。
 *       调用前应先确保被抢占方已让出锁（rc_worker 用 servo_manager_is_idle() 轮询）。
 */
void bsp_servo_move_smooth_preempt(uint8_t channel, float target, uint32_t step_ms)
{
    bsp_board_t *board = bsp_board_get_instance();

    if (board == NULL || !board->servo_initialized)
    {
        ESP_LOGE(TAG, "舵机未就绪，拒绝执行动作指令!");
        return;
    }

    if (channel >= 3 || s_ch_mutex[channel] == NULL)
    {
        ESP_LOGE(TAG, "无效通道 %d 或互斥锁未初始化!", channel);
        return;
    }

    // 【诊断·2026-08-05·验证完即删】记录拿锁耗时：区分「卡在等锁」与「拿到锁但没走」
    int64_t t_lock0 = esp_timer_get_time();
    xSemaphoreTake(s_ch_mutex[channel], portMAX_DELAY);
    int64_t lock_wait_ms = (esp_timer_get_time() - t_lock0) / 1000;

    // ★与 bsp_servo_move_smooth 的唯一差别：锁已在手，此刻清标志不会被任何人插队。
    //   清在读角度之前即可——后面的插值循环每步都会再查一次。
    bsp_servo_clear_abort();

    float safe_target = clamp_safe_angle(channel, target);

    float current = 0.0f;
    esp_err_t err = iot_servo_read_angle(LEDC_LOW_SPEED_MODE, channel, &current);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "读取通道 %d 角度失败!", channel);
        xSemaphoreGive(s_ch_mutex[channel]);
        return;
    }

    // 【诊断】进入运动前的全部关键量：等锁多久、当前角、目标角、限位后目标
    ESP_LOGW(TAG, "[诊断] ch=%u 等锁=%lldms current=%.1f target=%.1f safe=%.1f step=%ums",
             (unsigned)channel, lock_wait_ms, current, target, safe_target, (unsigned)step_ms);

    if (fabs(safe_target - current) < 1.0f)
    {
        // 【诊断】死区提前返回——手臂不动的候选原因之一
        ESP_LOGW(TAG, "[诊断] ch=%u 命中死区(|%.1f-%.1f|<1) 直接返回，未运动",
                 (unsigned)channel, safe_target, current);
        xSemaphoreGive(s_ch_mutex[channel]);
        return;
    }

    if (step_ms == 0)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, safe_target);
        xSemaphoreGive(s_ch_mutex[channel]);
        return;
    }

    // 定帧线性插值（与 bsp_servo_move_smooth 同一套算法，详见该函数步骤 5 注释）
    float total_deg = fabsf(safe_target - current);
    uint32_t total_ms = (uint32_t)(total_deg * (float)step_ms);
    const uint32_t frame_ms = servo_frame_ms(); // ★抖动排查：可被运行时覆盖，默认=SERVO_FRAME_MS
    int frames = (int)(total_ms / frame_ms);
    if (frames < 1)
        frames = 1;

    // ★绝对定时（2026-09-03 抖动修复，理由详见 bsp_servo_move_all_parallel 处注释）
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frame_ticks = pdMS_TO_TICKS(frame_ms);

    bool aborted = false;
    int steps = 0; // 【诊断】实际走了几帧
    float last_a = current;
    for (int f = 1; f <= frames; f++)
    {
        // 保留每帧检查：本次运动仍可被【后续】的新指令/flush 打断（那是预期行为）。
        if (bsp_servo_abort_requested())
        {
            aborted = true;
            break;
        }
        float t = (float)f / (float)frames;
        float a = current + t * (safe_target - current);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, a);
        steps++;
        last_a = a;
        vTaskDelayUntil(&last_wake, frame_ticks); // 绝对定时，同 bsp_servo_move_smooth
    }

    if (!aborted)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, channel, safe_target);
    }

    // 【诊断】循环结果：走了几步、是否被打断、停在哪
    ESP_LOGW(TAG, "[诊断] ch=%u 结束 steps=%d aborted=%d 停在=%.1f 兜底写入=%s",
             (unsigned)channel, steps, (int)aborted, last_a, aborted ? "跳过" : "已写");

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

    // 以三轴中行程最大的为基准算总耗时，再按 SERVO_FRAME_MS 拆帧，保证三轴同时到达。
    // 【与旧版差异】旧版 max_steps = 最大行程度数（1°/步），延时 step_ms；
    //   现改为按总耗时拆成 20ms 的定帧（详见 bsp_servo_move_smooth 步骤 5 注释）。
    //   总耗时不变，更新率提升到 50Hz，消除慢速档肉眼可见的逐步跳变。
    float max_deg = fmaxf(fmaxf(fabsf(h_safe - h_cur), fabsf(l_safe - l_cur)), fabsf(r_safe - r_cur));
    uint32_t total_ms = (uint32_t)(max_deg * (float)step_ms);
    const uint32_t frame_ms = servo_frame_ms(); // ★抖动排查：可被运行时覆盖，默认=SERVO_FRAME_MS
    int max_steps = (int)(total_ms / frame_ms);

    if (max_deg < 1.0f || max_steps < 1 || step_ms == 0)
    {
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, h_safe);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, l_safe);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, r_safe);
        for (int i = 0; i < 3; i++)
            xSemaphoreGive(s_ch_mutex[i]);
        return;
    }

    // ── 线性插值：每步同时写三轴，t 从 1/max_steps 到 1 ────────────────────────
    // 【2026-09-03 抖动修复】原实现有两个缺陷，是运动抖动的真凶：
    //
    //   缺陷①：用 vTaskDelay(frame_ms) 计时，它是【相对延时】——从"本次调用时刻"
    //     再等 frame_ms。而三次 write_angle 本身耗时、且本任务随时可能被 LVGL/
    //     音频/唤醒词抢占，于是每帧实际间隔 = frame_ms + 本帧被抢占的时间，
    //     误差【逐帧累积】。帧间隔在 tick 边界上抖动 → 写入时刻相对 PWM 周期漂移
    //     → 有的 PWM 周期收到两次更新（前一次被覆盖，该帧位移丢失）、有的一次没有
    //     （位移重复）→ 实际吐出的脉宽序列不匀 → 舵机忠实跟随 → 肉眼可见抖动。
    //     ★改用 vTaskDelayUntil：以【绝对时刻】递推，无论本帧被抢占多久，
    //       下一帧的唤醒时刻都锚定在 last_wake + frame_ms，误差不累积。
    //
    //   缺陷②：三轴分三次 write_angle，中间可能被抢占，导致三轴的 duty 更新落到
    //     不同的 PWM 周期，三轴不同步（表现为动作发散、互相错拍）。
    //     ★改用 vTaskSuspendAll 包住三次写入，保证三轴 duty 在同一个 PWM 周期内一起生效。
    //
    //   ★这两条都与"每帧位移大小"无关，正好解释了为什么 A/B 对照（每帧 0.8° vs
    //     0.2°）抖得一样、为什么空载照抖、为什么调死区参数全都无效。
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t frame_ticks = pdMS_TO_TICKS(frame_ms);

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
        float ha = h_cur + t * (h_safe - h_cur);
        float la = l_cur + t * (l_safe - l_cur);
        float ra = r_cur + t * (r_safe - r_cur);

        // ★三轴写入尽量不被打断：保证三路 duty 落在同一个 PWM 周期，一起生效。
        //   用 vTaskSuspendAll（禁止任务调度）而【不用 taskENTER_CRITICAL】：
        //   后者会关中断，而 iot_servo_write_angle 内的 SERVO_CHECK 宏在参数非法时
        //   会调 ESP_LOGE —— 日志要拿锁、可能阻塞，在关中断的临界区里调用会崩。
        //   vTaskSuspendAll 只挡住同核的任务切换，不关中断，日志路径依然安全，
        //   而"三轴写入不被别的任务插队"这个目的同样达到。
        vTaskSuspendAll();
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_HEAD, ha);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_L_ARM, la);
        iot_servo_write_angle(LEDC_LOW_SPEED_MODE, CH_R_ARM, ra);
        xTaskResumeAll();

        // ★绝对时刻递推，抢占不累积误差（替代原 vTaskDelay 的相对延时）
        vTaskDelayUntil(&last_wake, frame_ticks);
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
