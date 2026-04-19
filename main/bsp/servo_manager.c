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

// 内部队列项结构体，包装舵机请求
typedef struct
{
    servo_request_t req; // 舵机动作请求结构体
} internal_req_t;

// 全局静态变量
static QueueHandle_t s_queue = NULL; // 舵机动作请求队列句柄
static TaskHandle_t s_worker = NULL; // 舵机工作线程句柄
static bool s_inited = false;        // 初始化状态标志

/* 内部：将方向与幅度转换为目标角（正负号依据 bsp 语义：head left = +） */
static inline float calc_target_angle(servo_direction_t dir, servo_amplitude_t amp)
{
    if (dir == SERVO_DIR_NEUTRAL)
        return 90.0f;                            // 中性位置为90度
    int sign = (dir == SERVO_DIR_LEFT) ? 1 : -1; // 左为正，右为负
    return 90.0f + sign * (float)amp;            // 计算目标角度
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
            servo_request_t *r = &item.req;
            ESP_LOGI(TAG, "执行请求: ch=%d amp=%d dir=%d speed=%u loop=%u osc=%d",
                     r->channel, r->amplitude, (int)r->direction, (unsigned)r->speed_ms, (unsigned)r->loop_count, r->oscillate);

            // 计算主要目标角度和相反方向角度
            float primary = calc_target_angle(r->direction, r->amplitude);
            float opposite = calc_target_angle((r->direction == SERVO_DIR_NEUTRAL) ? SERVO_DIR_NEUTRAL : (r->direction == SERVO_DIR_LEFT ? SERVO_DIR_RIGHT : SERVO_DIR_LEFT),
                                               r->amplitude);

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
    memcpy(&item.req, req, sizeof(item.req));
    if (xQueueSend(s_queue, &item, 0) != pdTRUE)
    {
        ESP_LOGW(TAG, "队列已满，拒绝请求");
        return ESP_ERR_NO_MEM;
    }
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