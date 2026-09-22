/**
 * servo_manager.c
 * 实现：队列化、worker task、index->模式映射、NVS 校准存取
 *
 * 说明：
 * - 该模块不直接驱动 LEDC，而是复用已有的 bsp_servo_move_smooth()。
 * - 对外调用都是非阻塞的（入队即返回），worker 串行执行，保证任意并发调用都线程安全。
 */

#include "servo_manager.h"
#include <string.h>
#include <stdlib.h>
#include <stdatomic.h> // atomic_bool 中断标志（跨核安全，见 plan R2）
#include "esp_heap_caps.h"
#include <math.h> // fabs()，用于 servo_manager_submit_angle 中的角度差计算
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "bsp_board.h" // 提供 bsp_servo_move_smooth、SERVO_SPEED_*、CH_*
#include "bsp_config.h"
#include "object.h" // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM

static const char *TAG = "SERVO_MGR";

#define SERVO_MGR_QUEUE_LEN 64 // 舵机动作请求队列长度
#define SERVO_MGR_TASK_PRIO 6  // 舵机管理器任务优先级
/* 舵机管理器任务栈大小（分配在PSRAM中）
 *
 * ★2026-09-18 由 4096 提到 6144 —— 本次给动作序列加 hold_ms[] 的连带影响：
 *   worker 栈上同时躺着两坨大对象，且【同时存活】：
 *     · servo_worker_task 的局部 item（含 servo_seq_request_t，3 轴 × 34 点）
 *     · servo_exec_seq  的局部 axes[3]（同规模）
 *   hold_ms[] 给每轴 +136B，两处合计约 +790B——占 4096 栈的近 20%，
 *   而这两处本就是本任务栈占用的绝对大头。栈分配在 PSRAM，加这 2KB 的
 *   代价为零（不占内部 SRAM），用它换掉一个"可能只剩几百字节余量"的隐患。
 *   若后续再把 SERVO_SEQ_MAX_STEPS 调大，这里必须跟着重新评估。 */
#define SERVO_MGR_TASK_STACK 6144
#define NVS_NAMESPACE "servo_calib" // NVS命名空间，用于存储舵机校准参数

// 请求类型标签：区分「单轴串行请求」与「三轴并行请求」，worker 据此分派执行路径
typedef enum
{
    REQ_KIND_SINGLE = 0,   // 单轴动作（servo_request_t），串行执行
    REQ_KIND_PARALLEL,     // 三轴并行动作（servo_parallel_request_t），同步执行
    REQ_KIND_ABS_PARALLEL, // 三轴绝对角度并行动作（servo_abs_parallel_request_t）
    REQ_KIND_SEQ,          // 三轴动作序列（servo_seq_request_t）：终点+序列+各自归中，三轴各走各的
} req_kind_t;

// 内部队列项结构体，包装舵机请求（用 kind 区分 union 的有效成员）
typedef struct
{
    req_kind_t kind; // 请求类型
    union
    {
        servo_request_t single;               // kind==REQ_KIND_SINGLE 时有效
        servo_parallel_request_t parallel;    // kind==REQ_KIND_PARALLEL 时有效
        servo_abs_parallel_request_t abs_par; // kind==REQ_KIND_ABS_PARALLEL 时有效
        servo_seq_request_t seq;              // kind==REQ_KIND_SEQ 时有效
    } u;
    SemaphoreHandle_t done; // 非 NULL：执行完毕后 give 通知调用方（完成等待用）
} internal_req_t;

// 全局静态变量
static QueueHandle_t s_queue = NULL; // 舵机动作请求队列句柄
static TaskHandle_t s_worker = NULL; // 舵机工作线程句柄
static bool s_inited = false;        // 初始化状态标志

// ★队列本体（64×sizeof(internal_req_t)≈3.6KB）改用 xQueueCreateStatic + PSRAM 静态存储区，
//   避免 xQueueCreate 默认从内部 SRAM 分配这一整块（回收 ~3.6KB 内部 SRAM）。
//   队列只是数据搬运，worker 取出后才调 bsp_servo_*（不涉及 Flash/NVS），队列存储区放
//   PSRAM 没有 cache 关闭期不可访问的风险（与 udp_logger.c 同款做法）。
static StaticQueue_t s_queue_struct;    // 队列控制块（TCB 级，静态放 .bss）
static uint8_t *s_queue_storage = NULL; // heap_caps_malloc(..., MALLOC_CAP_SPIRAM) 的存储区

// flush 中断标志：servo_manager_flush() 置 true，正在执行的并行动作循环检测到后
// 在下一个循环边界 break 跳出；worker 归中后清回 false。atomic 保证跨核可见（plan R2）。
static atomic_bool s_flush_req = ATOMIC_VAR_INIT(false);

// flush「打断后是否归中」标志：servo_manager_flush_ex(center) 在置 s_flush_req 前写入。
// true（默认/旧行为）=打断后平滑归中 90°；false=停在当前角度，由调用方接管后续运动。
// 远程控制走 false：rc_worker 紧接着要把手臂送到用户指定角，归中会与它抢通道锁产生竞态。
static atomic_bool s_flush_center = ATOMIC_VAR_INIT(true);

// ★worker「真正空闲」标志（供 standby.c 进二级前判断舵机是否已彻底静止，见 servo_manager_is_idle）。
//   根因：旧实现里 standby.c 只查 interaction_is_playing()==false + 固定等 200ms 就认为舵机
//   已静止，但 servo_manager worker 的归中动作（bsp_servo_move_all_parallel 90/90/90）是在本
//   worker 任务里异步执行的，耗时取决于归中前的角度偏移，200ms 不一定够。一旦 standby.c 在
//   worker 仍在写 iot_servo_write_angle 时就调 bsp_servo_idle()（ledc_stop），两者对同一 LEDC
//   通道竞争写，会造成偶发的「进/退二级瞬间抖一下」（概率性，取决于是否撞上这个窗口）。
//   本标志在 worker 取出请求开始执行时置 true，执行完（含归中）后置 false，队列为空且本标志
//   为 false 才是「真空闲」，比固定延时可靠。
static atomic_bool s_worker_busy = ATOMIC_VAR_INIT(false);

/* 内部：将方向与幅度转换为目标角（正负号依据 bsp 语义：head left = +） */
static inline float calc_target_angle(servo_direction_t dir, servo_amplitude_t amp)
{
    if (dir == SERVO_DIR_NEUTRAL)
        return 90.0f;                            // 中性位置为90度
    int sign = (dir == SERVO_DIR_LEFT) ? 1 : -1; // 左为正，右为负
    return 90.0f + sign * (float)amp;            // 计算目标角度
}

/* 内部：取某一轴的「反方向」（NEUTRAL 保持 NEUTRAL），用于 oscillate 往返 */
static inline servo_direction_t opposite_dir(servo_direction_t dir)
{
    if (dir == SERVO_DIR_NEUTRAL)
        return SERVO_DIR_NEUTRAL;
    return (dir == SERVO_DIR_LEFT) ? SERVO_DIR_RIGHT : SERVO_DIR_LEFT;
}

/* 内部：取某通道的「归位角」（2026-09-16 起头/臂分离）
 * 头部回 SERVO_CENTER_DEG(90°)，手臂回 ARM_CENTER_DEG(15°)。归中速度仍由调用方传。 */
static inline float servo_center_deg(uint8_t channel)
{
    return (channel == CH_HEAD) ? SERVO_CENTER_DEG : ARM_CENTER_DEG;
}

/* 内部：执行一个单轴串行请求（原逻辑，保持不变） */
static void servo_exec_single(const servo_request_t *r)
{
    // 降为 DEBUG:GIF 轮播会高频入队舵机请求,INFO 会持续刷屏。
    // 触摸情绪的业务级日志在 interaction.c(开始/完毕)不受影响,仍为 INFO。
    // 需要调舵机细节时,把本 TAG 的日志等级调到 DEBUG 即可恢复。
    ESP_LOGD(TAG, "执行单轴请求: ch=%d amp=%d dir=%d speed=%u loop=%u osc=%d",
             r->channel, r->amplitude, (int)r->direction, (unsigned)r->speed_ms, (unsigned)r->loop_count, r->oscillate);

    // 计算主要目标角度和相反方向角度
    float primary = calc_target_angle(r->direction, r->amplitude);
    float opposite = calc_target_angle(opposite_dir(r->direction), r->amplitude);

    if (r->oscillate)
    {
        // 往返振荡模式：在primary和opposite之间来回运动
        for (uint8_t i = 0; i < r->loop_count; i++)
        {
            bsp_servo_move_smooth(r->channel, primary, r->speed_ms);  // 移动到主要位置
            bsp_servo_move_smooth(r->channel, opposite, r->speed_ms); // 移动到相反位置
        }
        // 最后回中：头部回 90°，手臂回 ARM_CENTER_DEG（2026-09-16 头臂分离）
        bsp_servo_move_smooth(r->channel, servo_center_deg(r->channel), SERVO_SPEED_CENTER);
    }
    else
    {
        // 单次到位模式：移动到目标位置后回中
        bsp_servo_move_smooth(r->channel, primary, r->speed_ms);
        // 回中以保证一致性（UI 习惯），若不需要可改为不回中；角度随通道头/臂分离
        bsp_servo_move_smooth(r->channel, servo_center_deg(r->channel), SERVO_SPEED_CENTER);
    }
}

/* ════════════════════════════════════════════════════════════════════════════
 * 内部：执行一个【三轴动作序列】请求（2026-09-04 新增）
 *
 * 【一个轴的完整生命周期，四段】
 *
 *      归位角(起始) →  target(终点)  →  seq[](动作序列)  →  归位角(归中)
 *                      ╰──────────────── 本函数负责 ──────────────────╯
 *
 *   起始角是上一次动作结束时的位置（本函数末尾也归中，故恒为归位角）。
 *   本函数把"终点 + 序列 + 归中"三者拼成一串角度点交给
 *   bsp_servo_move_seq_parallel，三轴各走各的速度与步数，互不等待。
 *
 *   ★2026-09-17：归中【已从 worker 下沉到本函数】（见下方 ③）。此前归中是
 *   worker 在本函数返回后统一做的一次三轴同步动作，导致先走完的轴干等最慢的
 *   轴（双臂等头部摇摆）。worker 那处调用保留，降级为 flush 打断时的兜底。
 *
 * 【为什么把三段拼成一串】对底层而言它们没有区别，都是"依次走到这些角度"。
 *   拼成一串可以让三轴的时间线完全独立——若先统一走 target、再统一走序列、
 *   最后统一归中，每段之间就多出一个"三轴都得到齐"的同步点，正是要消除的东西。
 *   ★归中在 2026-09-17 也被并入这一串（此前它正是最后一个同步点）。
 *
 * 返回 true=被打断（提前退出），false=正常跑完。
 * ════════════════════════════════════════════════════════════════════════════ */
static bool servo_exec_seq(const servo_seq_request_t *p)
{
    const servo_seq_axis_t *src[3] = {&p->head, &p->l_arm, &p->r_arm};
    // 下标 0/1/2 顺序 = 头/左臂/右臂（与 bsp_servo_move_seq_parallel 内部约定一致）。
    // 仅用于取本轴的归位角（头 90° / 臂 ARM_CENTER_DEG），见下方 ③。
    const uint8_t chs[3] = {CH_HEAD, CH_L_ARM, CH_R_ARM};
    bsp_servo_seq_axis_t axes[3];

    for (int i = 0; i < 3; i++)
    {
        uint8_t n = 0;

        // 该轴不参与本动作：target 与 seq 都没配（speed=0 或 target=0 且无序列）
        if (src[i]->speed == 0 && src[i]->seq_len == 0)
        {
            axes[i].n_points = 0;
            axes[i].step_ms[0] = 0;
            axes[i].hold_ms[0] = 0; // ★2026-09-18：一并清零，不留未初始化值
            continue;
        }

        // ① 终点：作为序列的第一个点，用本轴自己的 speed
        //
        // ★2026-09-11 顺带修复：这一步终于用上 speed 了。旧版底层每轴只接受
        //   一个 step_ms，整条（含"走向终点"这步）被迫统一取 seq_speed ——
        //   也就是说配置里的 speed 在【配了序列时完全不生效】，是个静默失效的
        //   字段。速度改为逐点后该限制消失，两个速度现在都真实生效。
        if (src[i]->speed > 0)
        {
            axes[i].points[n] = src[i]->target;
            axes[i].step_ms[n] = src[i]->speed;
            axes[i].hold_ms[n] = 0; // ★2026-09-18：终点不设停留（停留属于 seq 步）
            n++;
        }

        // ② 动作序列：依次追加，超出底层容量则截断（表里不该配这么多）
        //    ★每步 speed 填 0 = "沿用本轴 seq_speed"，必须在此解析成实际值：
        //      底层 bsp_servo_seq_axis_t 不认 0 语义（见该结构注释）。
        for (uint8_t s = 0; s < src[i]->seq_len && n < BSP_SERVO_SEQ_MAX_POINTS; s++)
        {
            uint32_t st = src[i]->seq[s].speed;
            if (st == 0)
                st = src[i]->seq_speed; // 该步未单独指定 → 用本轴序列默认速度
            if (st == 0)
                st = src[i]->speed; // 默认也没配 → 退回终点速度（底层另有兜底）

            axes[i].points[n] = src[i]->seq[s].angle;
            axes[i].step_ms[n] = st;
            // ★2026-09-18：停留时长原样透传给底层（0 = 不停留，无需解析默认值）
            axes[i].hold_ms[n] = src[i]->seq[s].hold_ms;
            n++;
        }

        /* ③ 归中：作为本轴序列的【最后一个点】（2026-09-17 新增）
         *
         * 【为什么挪到这里】此前归中是 worker 在本函数返回后做的【一次三轴同步
         *   动作】（见 worker 内 bsp_servo_move_all_parallel）。而本函数"三轴全部
         *   走完才返回"，于是先做完的轴只能停在末位干等最慢的轴：实测情绪 20
         *   （头 8 步 MID 摇摆、双臂 5 步）表现为「双臂到了 170° 不动，一直等到
         *   头摇完 8 步，三轴才一起归中」——用户明确反馈这是错的。
         *
         * 【改法】归中下沉成每轴自己的最后一段：某轴走完自己的点就立即回中位，
         *   不再理会其余轴是否还在走。三轴因此各走各的完整生命周期：
         *       当前角 → target → seq[] → 归位角
         *   worker 的统一归中【保留】，降级为 flush 打断时的兜底（见 worker 处注释）：
         *   正常跑完时三轴已在归位角，那次调用是幂等的 no-op，无副作用。
         *
         * 【容量】本点占用底层 points[] 的一个坑位，故 bsp_board.h 的
         *   BSP_SERVO_SEQ_MAX_POINTS 需 ≥ 配置侧步数 +2（终点 1 + 归中 1），
         *   已由 interaction.c 的 static_assert 在编译期卡住。此处仍加边界判断：
         *   极端情况下（配置侧步数填满）就【不追加】，自动退回 worker 统一归中，
         *   宁可归中晚一点，也不越界写数组。
         *
         * 【n > 0 的含义】该轴参与了本动作才归中；speed=0 且 seq_len=0 的轴
         *   （"本情绪不参与"或低功耗模式下的手臂）n 恒为 0，不追加、保持钉住。 */
        if (n > 0 && n < BSP_SERVO_SEQ_MAX_POINTS)
        {
            axes[i].points[n] = servo_center_deg(chs[i]); // 头 90° / 臂 ARM_CENTER_DEG
                                                          // axes[i].step_ms[n] = SERVO_SPEED_CENTER;      // 与 worker 统一归中同速
                                                          // 改成按轴分开
            axes[i].step_ms[n] = (chs[i] == CH_HEAD) ? SERVO_SPEED_CENTER
                                                     : SERVO_SPEED_CENTER_ARM; // 新增宏
            axes[i].hold_ms[n] = 0;                                            // ★2026-09-18：归中段不停留
            n++;
        }

        axes[i].n_points = n;
    }

    // flush 已被请求：直接返回，不启动本动作
    if (atomic_load(&s_flush_req))
        return true;

    // 三轴独立推进，全部走完才返回；内部逐帧检查 abort
    bool aborted = bsp_servo_move_seq_parallel(axes);

    // ★补检：move_seq_parallel 被 abort 打断后返回 true，但 flush 也可能在
    //   期间被置位。二者任一为真都算被打断——理由同 servo_exec_abs_parallel
    //   里那条 2026-08-05 注释：worker 误判"正常跑完"会强行归中抢走通道锁。
    return aborted || atomic_load(&s_flush_req) || bsp_servo_abort_requested();
}

/* 内部：执行一个三轴并行请求 —— 三轴【同时】运动，调 bsp_servo_move_all_parallel 实现真正同步。
 *
 * 思路（与 interaction.c 情绪动作一致）：
 *   - 各轴 primary = 90 + 方向*幅度，opposite = 90 - 方向*幅度（NEUTRAL 轴恒为 90，原地不动）。
 *   - 以三轴中 count 最大者为外层循环次数；某轴在它自己的 count 用完后保持 90°。
 *   - oscillate=true：每轮 primary→opposite 各一次（往返）；false：仅 primary（单次到位）。
 *   - 速度统一取三轴中的某一个有效 speed（move_all_parallel 只接受一个 step_ms，三轴共用时间窗口）。
 */
// 返回 true=被 flush 打断（提前退出），false=正常跑完。归中由 worker 统一负责。
static bool servo_exec_parallel(const servo_parallel_request_t *p)
{
    const servo_axis_action_t *axes[3] = {&p->head, &p->l_arm, &p->r_arm};

    // 各轴 primary / opposite 角度（NEUTRAL 或 count==0 的轴恒 90°，原地不参与）
    float prim[3], opp[3];
    bool act[3]; // 该轴是否参与运动
    uint8_t max_loop = 0;
    servo_speed_level_t speed = SERVO_SPEED_MID; // 兜底速度，取有效轴中最慢的
    bool any_osc = false;

    for (int i = 0; i < 3; i++)
    {
        act[i] = (axes[i]->count > 0 && axes[i]->direction != SERVO_DIR_NEUTRAL);
        if (act[i])
        {
            prim[i] = calc_target_angle(axes[i]->direction, axes[i]->amplitude);
            opp[i] = calc_target_angle(opposite_dir(axes[i]->direction), axes[i]->amplitude);
            if (axes[i]->count > max_loop)
                max_loop = axes[i]->count;
            if (axes[i]->oscillate)
                any_osc = true;
            // 取最慢轴的速度：speed_ms 值越大越慢，保证所有轴都有时间走完行程
            if (axes[i]->speed_ms > speed)
                speed = axes[i]->speed_ms;
        }
        else
        {
            prim[i] = 90.0f; // 不参与的轴保持中位
            opp[i] = 90.0f;
        }
    }

    if (max_loop == 0)
        return false; // 三轴都不动，直接返回（未被打断）

    ESP_LOGD(TAG, "执行并行请求: max_loop=%u speed=%u osc=%d", (unsigned)max_loop, (unsigned)speed, any_osc);

    for (uint8_t loop = 0; loop < max_loop; loop++)
    {
        // 每轮循环边界检查 flush 中断标志：被请求打断则立即跳出（plan R1/步骤1）
        if (atomic_load(&s_flush_req))
            return true;

        // 前半段：三轴同时到 primary（本轴 count 用完后回 90°）
        bsp_servo_move_all_parallel(
            (loop < axes[0]->count) ? prim[0] : 90.0f,
            (loop < axes[1]->count) ? prim[1] : 90.0f,
            (loop < axes[2]->count) ? prim[2] : 90.0f,
            speed);
        // ★段间补检：理由同 servo_exec_abs_parallel，见该函数内注释。
        if (atomic_load(&s_flush_req) || bsp_servo_abort_requested())
            return true;

        // 后半段：仅 oscillate 时三轴同时到 opposite（往返）
        if (any_osc)
        {
            bsp_servo_move_all_parallel(
                (loop < axes[0]->count && axes[0]->oscillate) ? opp[0] : ((loop < axes[0]->count) ? prim[0] : 90.0f),
                (loop < axes[1]->count && axes[1]->oscillate) ? opp[1] : ((loop < axes[1]->count) ? prim[1] : 90.0f),
                (loop < axes[2]->count && axes[2]->oscillate) ? opp[2] : ((loop < axes[2]->count) ? prim[2] : 90.0f),
                speed);
            if (atomic_load(&s_flush_req) || bsp_servo_abort_requested())
                return true;
        }
    }

    return false; // 正常跑完（归中由 worker 统一负责）
}

/* 内部：执行一个【绝对角度】三轴并行请求 —— 三轴同时 angle_1→angle_2 往返 count 次。
 * 与情绪表 ActionStep_t 语义 1:1，不做幅度/方向换算（零误差）。
 * 返回 true=被 flush 打断，false=正常跑完。归中由 worker 统一负责。 */
static bool servo_exec_abs_parallel(const servo_abs_parallel_request_t *p)
{
    const servo_abs_axis_t *axes[3] = {&p->head, &p->l_arm, &p->r_arm};

    // 外层循环次数 = 三轴 count 最大者；某轴 count 用完后保持 90°
    uint8_t max_loop = 0;
    uint32_t speed = SERVO_SPEED_MID; // 取最慢轴速度，保证都走得完
    for (int i = 0; i < 3; i++)
    {
        if (axes[i]->count > max_loop)
            max_loop = axes[i]->count;
        if (axes[i]->count > 0 && axes[i]->speed_ms > speed)
            speed = axes[i]->speed_ms;
    }
    if (max_loop == 0)
        return false;

    for (uint8_t loop = 0; loop < max_loop; loop++)
    {
        if (atomic_load(&s_flush_req))
            return true;

        // 前半段：三轴同时到 angle_1（本轴 count 用完后回 90°）
        bsp_servo_move_all_parallel(
            (loop < axes[0]->count) ? axes[0]->angle_1 : 90.0f,
            (loop < axes[1]->count) ? axes[1]->angle_1 : 90.0f,
            (loop < axes[2]->count) ? axes[2]->angle_1 : 90.0f,
            speed);
        // ★段间补检（2026-08-05）：move_all_parallel 内部被 abort 打断后【静默返回】，
        //   若只靠循环顶部那一次检查，count=1（绝大多数情绪动作）时外层只跑一轮，
        //   两个 move 都被打断也照样走到函数末尾 return false ⇒ worker 误判「正常跑完」，
        //   使 s_flush_center 失效（!aborted 恒真）而强行归中，抢走 rc_worker 要的通道锁。
        //   实测表现：下发指令后当前动作立刻停住、但目标位置到不了，第二次下发才行。
        if (atomic_load(&s_flush_req) || bsp_servo_abort_requested())
            return true;

        // 后半段：三轴同时到 angle_2（本轴 count 用完后回 90°）
        bsp_servo_move_all_parallel(
            (loop < axes[0]->count) ? axes[0]->angle_2 : 90.0f,
            (loop < axes[1]->count) ? axes[1]->angle_2 : 90.0f,
            (loop < axes[2]->count) ? axes[2]->angle_2 : 90.0f,
            speed);
        if (atomic_load(&s_flush_req) || bsp_servo_abort_requested())
            return true;
    }

    return false; // 正常跑完（归中由 worker 统一负责）
}

/* 内部：worker 主循环，串行消费动作请求 */
static void servo_worker_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    internal_req_t item;
    for (;;)
    {
        // 阻塞等待队列中的舵机动作请求
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            bool aborted = false;
            atomic_store(&s_worker_busy, true); // 开始执行（含随后可能的归中），标记「忙」

            // ★ 取出新请求、开始执行前清两个打断标志：避免「flush 时队列空、worker 阻塞
            //   在 xQueueReceive，没有正在执行的请求可打断 → 标志悬留为 true →
            //   下一条新请求（如情绪舵机）一执行就被立即跳过/中止」的竞态。
            //   清后，打断标志只影响「flush 那一刻已在执行中」的请求，新请求从干净起步。
            atomic_store(&s_flush_req, false);
            bsp_servo_clear_abort();

            // 按请求类型分派：单轴串行 / 幅度方向并行 / 绝对角度并行
            if (item.kind == REQ_KIND_PARALLEL)
                aborted = servo_exec_parallel(&item.u.parallel);
            else if (item.kind == REQ_KIND_ABS_PARALLEL)
                aborted = servo_exec_abs_parallel(&item.u.abs_par);
            else if (item.kind == REQ_KIND_SEQ)
                aborted = servo_exec_seq(&item.u.seq); // 终点+序列+各自归中，三轴各走各的
            else
                servo_exec_single(&item.u.single); // 单轴自带归中，不参与 flush

            /* 归中策略（plan R1）：
             * - 并行动作（含绝对角度）统一在此归中，无论正常跑完还是被 flush 打断。
             * - 被打断时：必须【先清 flush 标志】，否则归中本身也会因标志为真而无法进行。
             *   清标志后再做一次独立的平滑归中，把舵机带回中位。
             * - 正常跑完：标志本就为 false，直接归中。
             *
             * ★2026-09-17 REQ_KIND_SEQ 的性质已变：动作序列请求的归中【已在
             *   servo_exec_seq 内部下沉为每轴的最后一段】（某轴走完立即回中，
             *   不再等其余轴）。因此对 SEQ 而言，本处这次调用：
             *     · 正常跑完（aborted=false）：三轴已各自归位，本调用是幂等的
             *       no-op（move_all_parallel 走 0 行程），留着不影响观感；
             *     · 被 flush 打断：三轴停在半路、没走到各自的归中段，正是靠本处
             *       把它们拉回中位——这是打断后唯一还会归中的地方，【不能删】。
             *   另两路（PARALLEL / ABS_PARALLEL）的行为完全未变，仍由本处统一归中。 */
            if (item.kind == REQ_KIND_PARALLEL || item.kind == REQ_KIND_ABS_PARALLEL ||
                item.kind == REQ_KIND_SEQ)
            {
                // ★2026-08-05：被打断时是否归中，取决于「打断方有没有后续动作接管」。
                //   center=false（远程控制）：直接停在当前角度，【不清打断标志、不归中】。
                //   清标志的活儿交给打断方（rc_submit 延时后清），worker 抢着清会把标志
                //   在被打断方看到之前抹掉；归中则会与 rc_worker 抢通道锁 → 实测出现
                //   「到位后又被拉回 90°」「先归中再到目标」等随机现象。
                //   注意 aborted=false（正常跑完）时不受影响，照常归中。
                bool do_center = !aborted || atomic_load(&s_flush_center);
                // ★清标志与「是否归中」是两件独立的事，【不能绑在一起】。
                //   被打断就必须清：否则标志留成 true，接管方（rc_worker）的
                //   bsp_servo_move_smooth 插值循环首步即读到 abort → break，一度不走，
                //   表现为「下发后当前动作停住，目标位置 100% 到不了」。
                //   曾把这段挂在 do_center 下，导致 center=false 时连标志也一起跳过 —— 那正是
                //   把「偶发失败」变成「必然失败」的原因（2026-08-05 实测十次全中）。
                //   归中路径同样依赖它先清（见下方 if：否则归中自己也会被立即打断）。
                if (aborted)
                {
                    atomic_store(&s_flush_req, false);
                    bsp_servo_clear_abort();
                }
                if (do_center)
                {
                    /* 【2026-09-01】速度由 SERVO_SPEED_MID(15ms/度) 改为独立的
                     * SERVO_SPEED_CENTER(当前 30ms/度)：用户反馈归中太快、显得机械。
                     * 单列一个宏而不是直接改 MID —— MID 是"大多数情绪动作"的速度，
                     * 动它会把所有情绪的手感一起改掉。调速只需改 bsp_config.h 那一处。
                     * 本行是【所有并行/绝对角度请求】归中的唯一出口：无论正常播完
                     * 还是被 servo_manager_flush() 打断（进功能盘、闹钟/番茄钟到期）
                     * 都走这里，故改这一行即覆盖用户能感知到的全部归中。 */
                    bsp_servo_move_all_parallel(SERVO_CENTER_DEG, ARM_CENTER_DEG, ARM_CENTER_DEG,
                                                SERVO_SPEED_CENTER); // 头90°/臂15°（2026-09-16 头臂分离）
                }
            }

            // 完成通知：带 done 信号量的请求（情绪动作）执行完毕后唤醒等待方。
            // 无论正常完成还是被打断都 give，防止调用方永久阻塞（plan R4）。
            if (item.done != NULL)
                xSemaphoreGive(item.done);

            atomic_store(&s_worker_busy, false); // 本条请求（含归中）已彻底执行完毕，真正空闲
        }
    }
}

/* 初始化舵机管理器 */
esp_err_t servo_manager_init(void)
{
    if (s_inited)
        return ESP_OK;

    // 创建舵机动作请求队列（存储区放 PSRAM，回收 ~3.6KB 内部 SRAM）
    s_queue_storage = heap_caps_malloc(SERVO_MGR_QUEUE_LEN * sizeof(internal_req_t),
                                       MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_queue_storage)
    {
        ESP_LOGE(TAG, "队列 PSRAM 存储区分配失败");
        return ESP_ERR_NO_MEM;
    }
    s_queue = xQueueCreateStatic(SERVO_MGR_QUEUE_LEN, sizeof(internal_req_t),
                                 s_queue_storage, &s_queue_struct);
    if (!s_queue)
    {
        heap_caps_free(s_queue_storage);
        s_queue_storage = NULL;
        ESP_LOGE(TAG, "创建队列失败");
        return ESP_ERR_NO_MEM;
    }

    /* 栈分配在 SPIRAM，节省内部 SRAM */

    BaseType_t r = xTaskCreatePinnedToCoreWithCaps(
        servo_worker_task,
        "servo_mgr",
        SERVO_MGR_TASK_STACK, NULL, SERVO_MGR_TASK_PRIO, &s_worker,
        tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (r == pdPASS)
        PRINT_TASK_CREATED(TAG, "servo_mgr", SERVO_MGR_TASK_STACK, 0); // 栈在PSRAM
    if (r != pdPASS)
    {
        vQueueDelete(s_queue); // 静态队列 delete 不释放存储区，需手动 free
        s_queue = NULL;
        heap_caps_free(s_queue_storage);
        s_queue_storage = NULL;
        ESP_LOGE(TAG, "创建 worker 任务失败");
        return ESP_ERR_NO_MEM;
    }

    s_inited = true;
    ESP_LOGI(TAG, "servo_manager 初始化完成");
    return ESP_OK;
}

/* 反初始化舵机管理器 */
void servo_manager_deinit(void)
{
    if (!s_inited)
        return;
    // 简单处理，不做复杂优雅停止（可扩展）
    if (s_worker)
    {
        vTaskDelete(s_worker);
        s_worker = NULL;
    }
    if (s_queue)
    {
        vQueueDelete(s_queue); // 静态队列：仅注销队列，存储区由下面手动 free
        s_queue = NULL;
    }
    if (s_queue_storage)
    {
        heap_caps_free(s_queue_storage); // 释放 PSRAM 静态存储区，避免泄漏
        s_queue_storage = NULL;
    }
    s_inited = false;
}

/* 非阻塞入队通用函数 - 向队列发送提交舵机动作请求 */
esp_err_t servo_manager_submit_request(const servo_request_t *req)
{
    if (!s_inited || req == NULL)
        return ESP_ERR_INVALID_STATE;
    internal_req_t item;
    item.kind = REQ_KIND_SINGLE;
    item.done = NULL; // 单轴请求不带完成通知
    memcpy(&item.u.single, req, sizeof(item.u.single));
    if (xQueueSend(s_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "队列已满，拒绝请求");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* 非阻塞入队三轴并行请求 —— 三轴同步运动，参见 servo_exec_parallel */
esp_err_t servo_manager_submit_parallel(const servo_parallel_request_t *req)
{
    if (!s_inited || req == NULL)
        return ESP_ERR_INVALID_STATE;
    internal_req_t item;
    item.kind = REQ_KIND_PARALLEL;
    item.done = NULL; // 自动循环/待机的并行请求不带完成通知
    memcpy(&item.u.parallel, req, sizeof(item.u.parallel));
    if (xQueueSend(s_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "队列已满，拒绝并行请求");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* 非阻塞入队【绝对角度三轴并行】请求，可选完成通知（情绪动作用） */
esp_err_t servo_manager_submit_abs_parallel_notify(const servo_abs_parallel_request_t *req,
                                                   SemaphoreHandle_t done_sem)
{
    if (!s_inited || req == NULL)
        return ESP_ERR_INVALID_STATE;
    internal_req_t item;
    item.kind = REQ_KIND_ABS_PARALLEL;
    item.done = done_sem; // 非 NULL 时执行完毕 give 通知
    memcpy(&item.u.abs_par, req, sizeof(item.u.abs_par));
    if (xQueueSend(s_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "队列已满，拒绝绝对角度并行请求");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* 非阻塞入队【三轴动作序列】请求，可选完成通知（情绪动作用，2026-09-04 起） */
esp_err_t servo_manager_submit_seq_notify(const servo_seq_request_t *req,
                                          SemaphoreHandle_t done_sem)
{
    if (!s_inited || req == NULL)
        return ESP_ERR_INVALID_STATE;
    internal_req_t item;
    item.kind = REQ_KIND_SEQ;
    item.done = done_sem; // 非 NULL 时执行完毕 give 通知
    memcpy(&item.u.seq, req, sizeof(item.u.seq));
    if (xQueueSend(s_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "队列已满，拒绝动作序列请求");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

/* 清空队列并打断当前动作，舵机平滑归中 90°（用于进功能盘/强制回主）。
 * 非阻塞：置标志 + ResetQueue；正在执行的动作在循环边界 break，归中由 worker 做。
 * 注意 xQueueReset 会丢弃队列里【未执行】请求——若它们带 done_sem，则其 give 永不发生。
 * 当前设计下情绪只入队一条且 interaction take 带超时兜底，故安全（plan R4）。 */
esp_err_t servo_manager_flush(void)
{
    return servo_manager_flush_ex(/*center=*/true); // 保持原语义：打断后归中
}

esp_err_t servo_manager_flush_ex(bool center)
{
    if (!s_inited)
        return ESP_ERR_INVALID_STATE; // 初始化无效
    // ★须在置打断标志【之前】写：worker 可能在几十 us 内就走到归中判断处，
    //   晚写会被读到上一轮的残值。
    atomic_store(&s_flush_center, center);
    atomic_store(&s_flush_req, true); // 通知 servo_exec 外层循环在轮边界跳出
    bsp_servo_request_abort();        // ★ 通知 bsp 插值步循环立即停（几十 ms 内），不等整轮
    xQueueReset(s_queue);             // 清掉所有未执行请求
    // ESP_LOGI(TAG, "servo flush：清队 + 立即打断当前插值，舵机归中");
    return ESP_OK;
}

bool servo_manager_is_idle(void)
{
    if (!s_inited || s_queue == NULL)
        return true;
    // 队列为空 且 worker 当前没有正在执行的请求（含归中），才是真正静止。
    // uxQueueMessagesWaiting 与 s_worker_busy 分属两个状态源，理论上有极小窗口
    // （worker 刚 xQueueReceive 取走一条、s_worker_busy 还没来得及置 true），
    // 但调用方（standby.c）本身是轮询等待，下一拍会再次确认，不影响正确性。
    return (uxQueueMessagesWaiting(s_queue) == 0) && !atomic_load(&s_worker_busy);
}

/*预留接口，按 index 生成模式并提交 - 通过索引触发预设组合，适用于 UI 场景
 *按角度直接包裹提交（把角度转换为 request 并入队）
 */
esp_err_t servo_manager_submit_angle(uint8_t channel, float angle_deg, uint32_t speed_ms)
{
    if (!s_inited)
        return ESP_ERR_INVALID_STATE;
    // 直接把 target 作为“primary”且不 oscillate
    servo_request_t r = {0};
    r.channel = channel;
    // 计算 amplitude & direction from angle relative to 90
    float delta = angle_deg - 90.0f;
    if (fabs(delta) < 0.5f) // 如果目标角度非常接近中性位置（90度），则直接视为中立，避免微小偏差导致频繁震荡
    {
        r.direction = SERVO_DIR_NEUTRAL;
        r.amplitude = (servo_amplitude_t)0;
    }
    else
    {
        r.direction = (delta > 0.0f) ? SERVO_DIR_LEFT : SERVO_DIR_RIGHT;
        r.amplitude = (servo_amplitude_t)(fabs(delta) + 0.5f);
        // clip amplitude to allowed discrete values is not necessary
    }
    r.speed_ms = (servo_speed_level_t)(speed_ms == 0 ? SERVO_SPEED_VERY_FAST : speed_ms);
    r.loop_count = 1;
    r.oscillate = false;
    return servo_manager_submit_request(&r);
}

/* index -> 模式生成器（按规则生成 amplitude / speed / direction） */
static void generate_mode_from_index(uint16_t index, servo_amplitude_t *out_amp, servo_direction_t *out_dir, servo_speed_level_t *out_speed)
{
    // 组合规则（可按需调整）：
    // speeds[] = {VERY_SLOW, SLOW, MID, FAST, VERY_FAST}
    // amps[]   = {30,20,15,10}
    // dirs[]   = {LEFT, RIGHT}
    // 生成顺序：对 speeds 外层, amps 中层, dirs 内层 -> 5*4*2 = 40 组合
    // 我们仅需要 37 个，所以会以循环方式取 index%40，最后若 index 对应中间值可映射成中性动作
    servo_speed_level_t speeds[5] = {SERVO_SPEED_VERY_SLOW, SERVO_SPEED_SLOW, SERVO_SPEED_MID, SERVO_SPEED_FAST, SERVO_SPEED_VERY_FAST};
    servo_amplitude_t amps[4] = {SERVO_AMPLITUDE_30, SERVO_AMPLITUDE_20, SERVO_AMPLITUDE_15, SERVO_AMPLITUDE_10};
    servo_direction_t dirs[2] = {SERVO_DIR_LEFT, SERVO_DIR_RIGHT};

    uint16_t idx = index % (5 * 4 * 2); // 0..39// 循环映射到 40 个组合
    uint16_t i_speed = idx / (4 * 2);   // 0..4// 5 个速度档位循环
    uint16_t rem = idx % (4 * 2);       // 0..7// 4 个幅度 * 2 个方向循环
    uint16_t i_amp = rem / 2;           // 0..3// 4 个幅度循环
    uint16_t i_dir = rem % 2;           // 0..1// 2 个方向循环

    *out_speed = speeds[i_speed];
    *out_amp = amps[i_amp];
    *out_dir = dirs[i_dir];

    // 特殊：如果 index maps to last few slots we can pick neutral center to reach exactly 37 combos,
    // 但上层不必关心，index->mode 保证循环且覆盖常用组合。
}

/* 按 index 提交（非阻塞）- 通过索引提交预设的舵机动模式 */
esp_err_t servo_manager_submit_by_index(uint8_t channel, uint16_t index, uint8_t loop_count, bool oscillate)
{
    if (!s_inited)
        return ESP_ERR_INVALID_STATE;
    servo_request_t r;
    memset(&r, 0, sizeof(r)); // 置零
    r.channel = channel;
    generate_mode_from_index(index, &r.amplitude, &r.direction, &r.speed_ms);
    r.loop_count = (loop_count == 0) ? 1 : loop_count;
    r.oscillate = oscillate;
    return servo_manager_submit_request(&r);
}

/* NVS 校准存取（保存 min/max microseconds）- 保存舵机校准参数到非易失存储 */
esp_err_t servo_manager_save_calibration(uint8_t channel, uint32_t min_us, uint32_t max_us)
{
    esp_err_t err;
    nvs_handle_t h;
    err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h);
    if (err != ESP_OK)
        return err;
    char key_min[16], key_max[16]; // 舵机校准参数的 key
    snprintf(key_min, sizeof(key_min), "min_%u", channel);
    snprintf(key_max, sizeof(key_max), "max_%u", channel);
    err = nvs_set_u32(h, key_min, min_us);
    if (err == ESP_OK)
        err = nvs_set_u32(h, key_max, max_us);
    if (err == ESP_OK)
        err = nvs_commit(h);
    nvs_close(h);
    return err;
}

/* 从NVS加载舵机校准参数 */
bool servo_manager_load_calibration(uint8_t channel, uint32_t *out_min_us, uint32_t *out_max_us)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) != ESP_OK)
        return false;
    char key_min[16], key_max[16];
    snprintf(key_min, sizeof(key_min), "min_%u", channel);
    snprintf(key_max, sizeof(key_max), "max_%u", channel);
    uint32_t minv = 0, maxv = 0;
    esp_err_t e1 = nvs_get_u32(h, key_min, &minv);
    esp_err_t e2 = nvs_get_u32(h, key_max, &maxv);
    nvs_close(h);
    if (e1 == ESP_OK && e2 == ESP_OK)
    {
        if (out_min_us)
            *out_min_us = minv;
        if (out_max_us)
            *out_max_us = maxv;
        return true;
    }
    return false;
}