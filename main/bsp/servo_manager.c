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

static const char *TAG = "SERVO_MGR";

#define SERVO_MGR_QUEUE_LEN 64      // 舵机动作请求队列长度
#define SERVO_MGR_TASK_PRIO 6       // 舵机管理器任务优先级
#define SERVO_MGR_TASK_STACK 4096   // 舵机管理器任务栈大小（分配在PSRAM中）
#define NVS_NAMESPACE "servo_calib" // NVS命名空间，用于存储舵机校准参数

// 请求类型标签：区分「单轴串行请求」与「三轴并行请求」，worker 据此分派执行路径
typedef enum
{
    REQ_KIND_SINGLE = 0,   // 单轴动作（servo_request_t），串行执行
    REQ_KIND_PARALLEL,     // 三轴并行动作（servo_parallel_request_t），同步执行
    REQ_KIND_ABS_PARALLEL, // 三轴绝对角度并行动作（servo_abs_parallel_request_t）
} req_kind_t;

// 内部队列项结构体，包装舵机请求（用 kind 区分 union 的有效成员）
typedef struct
{
    req_kind_t kind; // 请求类型
    union
    {
        servo_request_t single;                // kind==REQ_KIND_SINGLE 时有效
        servo_parallel_request_t parallel;     // kind==REQ_KIND_PARALLEL 时有效
        servo_abs_parallel_request_t abs_par;  // kind==REQ_KIND_ABS_PARALLEL 时有效
    } u;
    SemaphoreHandle_t done; // 非 NULL：执行完毕后 give 通知调用方（完成等待用）
} internal_req_t;

// 全局静态变量
static QueueHandle_t s_queue = NULL; // 舵机动作请求队列句柄
static TaskHandle_t s_worker = NULL; // 舵机工作线程句柄
static bool s_inited = false;        // 初始化状态标志

// flush 中断标志：servo_manager_flush() 置 true，正在执行的并行动作循环检测到后
// 在下一个循环边界 break 跳出；worker 归中后清回 false。atomic 保证跨核可见（plan R2）。
static atomic_bool s_flush_req = ATOMIC_VAR_INIT(false);

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
        // 最后回中到90度位置
        bsp_servo_move_smooth(r->channel, 90.0f, SERVO_SPEED_MID);
    }
    else
    {
        // 单次到位模式：移动到目标位置后回中
        bsp_servo_move_smooth(r->channel, primary, r->speed_ms);
        // 回中以保证一致性（UI 习惯），若不需要可改为不回中
        bsp_servo_move_smooth(r->channel, 90.0f, SERVO_SPEED_MID);
    }
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

        // 后半段：仅 oscillate 时三轴同时到 opposite（往返）
        if (any_osc)
        {
            bsp_servo_move_all_parallel(
                (loop < axes[0]->count && axes[0]->oscillate) ? opp[0] : ((loop < axes[0]->count) ? prim[0] : 90.0f),
                (loop < axes[1]->count && axes[1]->oscillate) ? opp[1] : ((loop < axes[1]->count) ? prim[1] : 90.0f),
                (loop < axes[2]->count && axes[2]->oscillate) ? opp[2] : ((loop < axes[2]->count) ? prim[2] : 90.0f),
                speed);
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

        // 后半段：三轴同时到 angle_2（本轴 count 用完后回 90°）
        bsp_servo_move_all_parallel(
            (loop < axes[0]->count) ? axes[0]->angle_2 : 90.0f,
            (loop < axes[1]->count) ? axes[1]->angle_2 : 90.0f,
            (loop < axes[2]->count) ? axes[2]->angle_2 : 90.0f,
            speed);
    }

    return false;
}

/* 内部：worker 主循环，串行消费动作请求 */
static void servo_worker_task(void *arg)
{
    internal_req_t item;
    for (;;)
    {
        // 阻塞等待队列中的舵机动作请求
        if (xQueueReceive(s_queue, &item, portMAX_DELAY) == pdTRUE)
        {
            bool aborted = false;

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
            else
                servo_exec_single(&item.u.single); // 单轴自带归中，不参与 flush

            /* 归中策略（plan R1）：
             * - 并行动作（含绝对角度）统一在此归中，无论正常跑完还是被 flush 打断。
             * - 被打断时：必须【先清 flush 标志】，否则归中本身也会因标志为真而无法进行。
             *   清标志后再做一次独立的平滑归中，把舵机带回中位。
             * - 正常跑完：标志本就为 false，直接归中。 */
            if (item.kind == REQ_KIND_PARALLEL || item.kind == REQ_KIND_ABS_PARALLEL)
            {
                if (aborted)
                {
                    // 先清两个打断标志，再归中——否则归中的插值/外层循环也会被立即打断，归不了中。
                    atomic_store(&s_flush_req, false);
                    bsp_servo_clear_abort();
                }
                bsp_servo_move_all_parallel(90.0f, 90.0f, 90.0f, SERVO_SPEED_MID);
            }

            // 完成通知：带 done 信号量的请求（情绪动作）执行完毕后唤醒等待方。
            // 无论正常完成还是被打断都 give，防止调用方永久阻塞（plan R4）。
            if (item.done != NULL)
                xSemaphoreGive(item.done);
        }
    }
}

/* 初始化舵机管理器 */
esp_err_t servo_manager_init(void)
{
    if (s_inited)
        return ESP_OK;

    // 创建舵机动作请求队列
    s_queue = xQueueCreate(SERVO_MGR_QUEUE_LEN, sizeof(internal_req_t));
    if (!s_queue)
    {
        ESP_LOGE(TAG, "创建队列失败");
        return ESP_ERR_NO_MEM;
    }

    /* 栈分配在 SPIRAM，节省内部 SRAM */

    BaseType_t r = xTaskCreatePinnedToCoreWithCaps(
        servo_worker_task,
        "servo_mgr",
        SERVO_MGR_TASK_STACK, NULL, SERVO_MGR_TASK_PRIO, &s_worker,
        tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (r != pdPASS)
    {
        vQueueDelete(s_queue);
        s_queue = NULL;
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
        vQueueDelete(s_queue);
        s_queue = NULL;
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

/* 清空队列并打断当前动作，舵机平滑归中 90°（用于进功能盘/强制回主）。
 * 非阻塞：置标志 + ResetQueue；正在执行的动作在循环边界 break，归中由 worker 做。
 * 注意 xQueueReset 会丢弃队列里【未执行】请求——若它们带 done_sem，则其 give 永不发生。
 * 当前设计下情绪只入队一条且 interaction take 带超时兜底，故安全（plan R4）。 */
esp_err_t servo_manager_flush(void)
{
    if (!s_inited)
        return ESP_ERR_INVALID_STATE;
    atomic_store(&s_flush_req, true);  // 通知 servo_exec 外层循环在轮边界跳出
    bsp_servo_request_abort();          // ★ 通知 bsp 插值步循环立即停（几十 ms 内），不等整轮
    xQueueReset(s_queue);               // 清掉所有未执行请求
    ESP_LOGI(TAG, "servo flush：清队 + 立即打断当前插值，舵机归中");
    return ESP_OK;
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