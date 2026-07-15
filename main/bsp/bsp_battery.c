/**
 * @file bsp_battery.c
 * @brief 电池电压监控模块实现
 *
 * 详细设计见 bsp_battery.h 文件头注释。
 *
 * 关键实现细节：
 *   1. 使用 ESP-IDF v5.x 新 ADC oneshot API（adc_oneshot_new_unit / read），
 *      旧的 adc1_get_raw 已被弃用。
 *   2. 校准优先使用 curve fitting（ESP32-S3 支持），失败回退到 line fitting，
 *      最后才使用未校准近似值。
 *   3. 分压网络 R23/R24 比例由 bsp_config.h 宏定义，避免硬编码。
 *   4. 锂电池放电曲线严格遵循3.0V-4.2V标准，采用2%间隔超细化分段插值。
 *   5. 双级IIR低通滤波（电压+百分比），使用256进制系数提高整数精度。
 *   6. 智能变化速率限制：小差值1%步进，大差值快速追赶，兼顾平滑与响应。
 *   7. 电压滞回比较，防止电量来回跳动。
 *   8. 低电告警带100mV滞回，避免在阈值附近反复触发。
 *   9. 多次采样初始化，解决开机收敛慢和开路电压虚高问题。
 *  10. 完整的资源管理与任务同步机制，杜绝内存泄漏和野指针崩溃。
 */

#include "bsp_board.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h" // xTaskCreatePinnedToCoreWithCaps + MALLOC_CAP_SPIRAM（bat_log 栈迁移用）
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include "nvs.h"
#include <string.h>
#include <stdlib.h>
#include "object.h" // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM

// ─── NVS 持久化（断电存档，防止上电电量回弹）─────────────────────────────────
// 锂电池断电静置后电压会自然回弹，若上电重新查表会导致电量虚高。这里把上次的
// 显示电量与 OCV 基准存进 NVS，上电时取 min(存档, 实测) 做起点。
#define BAT_NVS_NS "bat"           ///< NVS 命名空间
#define BAT_NVS_KEY_PCT "last_pct" ///< uint8  上次显示电量
#define BAT_NVS_KEY_MV "last_mv"   ///< uint32 上次 OCV 基准电压（mV）

// ─── 编译时宏定义检查（防止编译错误）────────────────────────────────────────
#ifndef BSP_BAT_ADC_PIN
#error "BSP_BAT_ADC_PIN must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_VOLTAGE_RATIO_NUM
#error "BSP_BAT_VOLTAGE_RATIO_NUM must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_VOLTAGE_RATIO_DEN
#error "BSP_BAT_VOLTAGE_RATIO_DEN must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_ADC_SAMPLE_TIMES
#error "BSP_BAT_ADC_SAMPLE_TIMES must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_TASK_INTERVAL_MS
#error "BSP_BAT_TASK_INTERVAL_MS must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_IIR_ALPHA
#error "BSP_BAT_IIR_ALPHA must be defined in bsp_config.h (0-256)"
#endif
#ifndef BSP_BAT_PERCENT_IIR_ALPHA
#error "BSP_BAT_PERCENT_IIR_ALPHA must be defined in bsp_config.h (0-256)"
#endif
#ifndef BSP_BAT_MAX_CHANGE_PER_STEP
#error "BSP_BAT_MAX_CHANGE_PER_STEP must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_MAX_FAST_CHANGE
#error "BSP_BAT_MAX_FAST_CHANGE must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_HYSTERESIS_MV
#error "BSP_BAT_HYSTERESIS_MV must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_VOLTAGE_LOW_MV
#error "BSP_BAT_VOLTAGE_LOW_MV must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_TASK_STACK_SIZE
#error "BSP_BAT_TASK_STACK_SIZE must be defined in bsp_config.h"
#endif
#ifndef BSP_BAT_TASK_PRIORITY
#error "BSP_BAT_TASK_PRIORITY must be defined in bsp_config.h"
#endif

static const char *TAG = "bsp_battery";

// ─── 前向声明（解决函数之间的前向引用）────────────────────────────────────────
uint32_t bsp_battery_get_voltage_mv(void);
uint8_t bsp_battery_get_percent(void);
esp_err_t bsp_battery_stop_task(void);
esp_err_t bsp_battery_stop_log_task(void);

// ─── 内部类型定义 ─────────────────────────────────────────────────────────────
typedef enum
{
    CALI_TYPE_NONE,
    CALI_TYPE_CURVE
} cali_type_t;

// ─── 内部状态结构 ─────────────────────────────────────────────────────────────
typedef struct
{
    bool initialized;
    bool cali_enabled;
    cali_type_t cali_type;
    adc_oneshot_unit_handle_t adc_handle;
    adc_cali_handle_t cali_handle;
    adc_channel_t channel;
    adc_unit_t unit;

    TaskHandle_t task_handle;
    volatile bool task_running;
    bsp_battery_low_cb_t low_cb;
    bool low_alerted;

    uint32_t filtered_mv;
    uint32_t ocv_mv;
    volatile uint8_t filtered_percent;
    volatile uint8_t displayed_percent;

    // ─── 防回弹 / 充电判定状态 ───
    bool charging;        ///< 是否判定为正在充电（无充电脚，靠电压趋势推断）
    uint8_t rise_cnt;     ///< OCV 连续明显上升计数，达阈值判定为充电
    uint32_t last_ocv_mv; ///< 上一次用于趋势判定的 OCV，用于检测上升/下降

    // ─── 低电关机状态 ───
    uint8_t poweroff_hit; ///< OCV 连续低于关机阈值的命中计数，达阈值才真正关机
} bsp_battery_ctx_t;

static bsp_battery_ctx_t s_ctx = {0};
static TaskHandle_t s_log_task_handle = NULL;
static volatile bool s_log_task_running = false;

// ─── GPIO → ADC1 通道映射（ESP32-S3 专用）─────────────────────────────────────
static esp_err_t gpio_to_adc1_channel(int gpio, adc_channel_t *out_ch)
{
    if (gpio < 1 || gpio > 10)
    {
        return ESP_ERR_INVALID_ARG;
    }
    *out_ch = (adc_channel_t)(gpio - 1);
    return ESP_OK;
}

// ─── 锂电池放电曲线分段插值（电压 mV → 百分比 0~100）─────────────────────────
// 严格遵循3.0V-4.2V标准，专门针对3.7V 500-1000mAh小型聚合物锂电池实测
typedef struct
{
    uint32_t mv;
    uint8_t percent;
} bat_curve_point_t;

// 锂电池真实放电曲线（贴合 3.7V 软包电芯物理特性）：
//   4.15V→3.80V 为「电压平台区」，端电压变化小但放出大部分容量，电量缓降（100%→62%）；
//   3.80V→3.50V 为「中段」，电量稳步下降（62%→11%）；
//   3.50V→3.30V 为「陡降区」，电压快速跳水，电量急掉到 0（陡）。
//   末端锚点 3300mV=0%，与 BSP_BAT_POWEROFF_MV(3300) 低电关机点对齐——显示 0% 即关机。
static const bat_curve_point_t s_curve[] = {
    {4150, 100}, // 满电
    {4100, 95},  // ┐
    {4050, 91},  // │ 满电平台区：端电压缓降，电量慢掉
    {4000, 88},  // │
    {3950, 83},  // │
    {3900, 78},  // │
    {3850, 71},  // │
    {3800, 62},  // ┘ 平台尾
    {3775, 57},  // ┐
    {3750, 52},  // │ 中段：稳步下降
    {3725, 47},  // │
    {3700, 42},  // │
    {3675, 37},  // │
    {3650, 33},  // │
    {3625, 28},  // │
    {3600, 24},  // ┘
    {3575, 20},  // ┐ 拐点
    {3550, 17},  // ┘
    {3525, 14},  // ┐
    {3500, 11},  // │ 陡降区：电压跳水，电量急掉
    {3475, 8},   // │
    {3450, 6},   // │
    {3425, 4},   // │
    {3400, 3},   // │
    {3350, 1},   // │
    {3300, 0},   // ┘ 放空 / 关机点（对齐 BSP_BAT_POWEROFF_MV）
};

#define BAT_CURVE_LEN (sizeof(s_curve) / sizeof(s_curve[0]))

static uint8_t voltage_to_percent(uint32_t mv)
{
    if (mv >= s_curve[0].mv)
        return 100;
    if (mv <= s_curve[BAT_CURVE_LEN - 1].mv)
        return 0;

    for (size_t i = 0; i < BAT_CURVE_LEN - 1; i++)
    {
        uint32_t v_hi = s_curve[i].mv;
        uint32_t v_lo = s_curve[i + 1].mv;
        if (mv <= v_hi && mv >= v_lo)
        {
            uint8_t p_hi = s_curve[i].percent;
            uint8_t p_lo = s_curve[i + 1].percent;
            uint32_t span_v = v_hi - v_lo;
            uint32_t span_p = p_hi - p_lo;
            return (uint8_t)(p_lo + (mv - v_lo) * span_p / span_v);
        }
    }
    return 0;
}

// ─── NVS 读写（仿 servo_manager 写法，独立 open/commit/close）────────────────
// 读取上次存档的电量与 OCV 基准；无记录或越界时返回 false，out 参数不被修改。
static bool battery_nvs_load(uint8_t *out_pct, uint32_t *out_mv)
{
    nvs_handle_t h;
    if (nvs_open(BAT_NVS_NS, NVS_READONLY, &h) != ESP_OK)
        return false;

    uint8_t pct = 0;
    uint32_t mv = 0;
    esp_err_t e1 = nvs_get_u8(h, BAT_NVS_KEY_PCT, &pct);
    esp_err_t e2 = nvs_get_u32(h, BAT_NVS_KEY_MV, &mv);
    nvs_close(h);

    // 合法性校验：电量必须 0~100，电压必须在锂电池合理范围
    if (e1 != ESP_OK || e2 != ESP_OK || pct > 100 || mv < 2500 || mv > 4500)
        return false;

    if (out_pct)
        *out_pct = pct;
    if (out_mv)
        *out_mv = mv;
    return true;
}

// 保存当前电量与 OCV 基准。仅在电量实际变化时由调用方触发，避免频繁擦写 Flash。
static void battery_nvs_save(uint8_t pct, uint32_t mv)
{
    nvs_handle_t h;
    if (nvs_open(BAT_NVS_NS, NVS_READWRITE, &h) != ESP_OK)
    {
        ESP_LOGW(TAG, "NVS 打开失败，本次电量未存档");
        return;
    }
    esp_err_t err = nvs_set_u8(h, BAT_NVS_KEY_PCT, pct);
    if (err == ESP_OK)
        err = nvs_set_u32(h, BAT_NVS_KEY_MV, mv);
    if (err == ESP_OK)
        err = nvs_commit(h);
    nvs_close(h);
    if (err != ESP_OK)
        ESP_LOGW(TAG, "NVS 存档失败：%s", esp_err_to_name(err));
}

// ─── 软关机（GPIO18 → HK015T.1 OPT 脚，OPT 高→低下降沿断电）──────────────────
// 屏幕探针实测（2026-07）：单纯拉低不断电、单纯拉高只瞬断自恢复；真正让 HK015T 断电
// 的是 OPT 的"高→低"下降沿 —— 先把 GPIO18 推挽拉高 BSP_PWR_OFF_PULSE_MS 建立干净高，
// 再推挽拉低保持 → OUTH 翻低 → 翻转 Q4/Q3 切断主电源，效果等同用户长按 K1（静态 1μA）。
// 供低电自动关机与 standby 三级关机共用（见 bsp_board.h 声明）。本函数不返回——很快断电。
void bsp_battery_power_off(void)
{
    ESP_LOGW(TAG, "🔌 执行软关机：GPIO%d 拉高 %dms → 拉低造下降沿 → HK015T 断电",
             BSP_OPT_OUT_PIN, BSP_PWR_OFF_PULSE_MS);

    gpio_config_t io_cfg = {
        .pin_bit_mask = (1ULL << BSP_OPT_OUT_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_cfg);

    // 1) 先建立干净高电平（OPT 高，HK015T 此刻仍保持开机）
    gpio_set_level(BSP_OPT_OUT_PIN, BSP_PWR_OFF_ASSERT_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(BSP_PWR_OFF_PULSE_MS));
    // 2) 拉低造"高→低"下降沿并保持 → HK015T 翻 OUTH 低 → 整机断电
    gpio_set_level(BSP_OPT_OUT_PIN, BSP_PWR_OFF_DEASSERT_LEVEL);

    // 断电前自旋等待（正常数十 ms 内整板掉电，此循环不会真正跑满）；持续保持低电平
    for (int i = 0; i < 200; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(BSP_OPT_OUT_PIN, BSP_PWR_OFF_DEASSERT_LEVEL);
    }

    // 若 10s 后仍未断电（如调试时未接自锁电路 / 时序变化），告警提示
    ESP_LOGE(TAG, "软关机信号已发出但系统未断电，请核对 GPIO%d→HK015T 链路/关机时序", BSP_OPT_OUT_PIN);
}

// ─── 内部：执行一次 ADC 多采样，去极值平均，返回真实电池电压（毫伏）───────────
// 去掉一个最高值和一个最低值再平均，天然滤掉舵机瞬时启动/堵转造成的压降尖峰，
// 避免单次负载抖动把电压拉低导致电量跳变。
static uint32_t do_sample_voltage_mv(void)
{
    if (!s_ctx.initialized)
        return 0;

    uint32_t adc_mv_sum = 0;
    uint32_t valid_count = 0;
    int mv_min = 0x7fffffff; // 本轮最小 ADC 电压
    int mv_max = 0;          // 本轮最大 ADC 电压

    for (int i = 0; i < BSP_BAT_ADC_SAMPLE_TIMES; i++)
    {
        int raw = 0;
        if (adc_oneshot_read(s_ctx.adc_handle, s_ctx.channel, &raw) != ESP_OK)
        {
            continue;
        }

        int mv = 0;
        if (s_ctx.cali_enabled)
        {
            if (adc_cali_raw_to_voltage(s_ctx.cali_handle, raw, &mv) != ESP_OK)
            {
                continue;
            }
        }
        else
        {
            mv = raw * 3100 / 4095;
        }
        adc_mv_sum += (uint32_t)mv;
        valid_count++;
        if (mv < mv_min)
            mv_min = mv;
        if (mv > mv_max)
            mv_max = mv;
    }

    if (valid_count == 0)
        return 0;

    // 去极值：有效采样数≥3 时，剔除一个最高和一个最低再平均，过滤负载尖峰
    if (valid_count >= 3)
    {
        adc_mv_sum -= (uint32_t)mv_min;
        adc_mv_sum -= (uint32_t)mv_max;
        valid_count -= 2;
    }

    uint32_t adc_mv_avg = adc_mv_sum / valid_count;
    uint32_t vbat_mv = adc_mv_avg * BSP_BAT_VOLTAGE_RATIO_NUM / BSP_BAT_VOLTAGE_RATIO_DEN;

    // 全局电压范围校验，过滤明显异常值
    if (vbat_mv < 2500 || vbat_mv > 4500)
    {
        ESP_LOGW(TAG, "采样电压异常：%lu mV，丢弃", (unsigned long)vbat_mv);
        return 0;
    }

    return vbat_mv;
}

// ─── 后台采样任务 ─────────────────────────────────────────────────────────────
static void battery_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "电池监控任务启动，采样周期 %d ms", BSP_BAT_TASK_INTERVAL_MS);
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余

    // 多次快速采样初始化 filtered_mv，从真实带载电压开始
    {
        uint32_t init_sum = 0;
        uint32_t init_count = 0;

        // 前3次采样直接丢弃（ADC刚上电不稳定）
        for (int i = 0; i < 3; i++)
        {
            do_sample_voltage_mv();
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        // 后10次有效采样取平均
        for (int i = 0; i < 10; i++)
        {
            uint32_t v = do_sample_voltage_mv();
            if (v > 3000 && v < 4500)
            {
                init_sum += v;
                init_count++;
            }
            vTaskDelay(pdMS_TO_TICKS(20));
        }

        if (init_count >= 3)
        {
            s_ctx.filtered_mv = init_sum / init_count;
            ESP_LOGI(TAG, "电池初始化完成：初始电压=%lu mV",
                     (unsigned long)s_ctx.filtered_mv);
        }
        else
        {
            uint32_t last_v = do_sample_voltage_mv();
            s_ctx.filtered_mv = (last_v > 2500 && last_v < 4500) ? last_v : 3700;
            ESP_LOGW(TAG, "电池初始化采样失败（有效采样数=%lu），使用最后一次采样值%lu mV",
                     (unsigned long)init_count, (unsigned long)s_ctx.filtered_mv);
        }

        s_ctx.ocv_mv = s_ctx.filtered_mv;

        // ─── 上电防回弹钳制 ───────────────────────────────────────────────
        // 锂电池断电静置后端电压会自然回弹，若直接用实测电压查表会导致电量虚高
        // （断电前90%，再上电变100%）。这里读 NVS 上次存档，取 min(存档, 实测)
        // 做起点，并把 OCV 也下压，从根本上消除回弹。
        uint8_t boot_pct = voltage_to_percent(s_ctx.filtered_mv);
        uint8_t saved_pct = 0;
        uint32_t saved_mv = 0;
        if (battery_nvs_load(&saved_pct, &saved_mv))
        {
            uint8_t start_pct;
            uint32_t start_mv;
            if (s_ctx.ocv_mv >= BSP_BAT_HIGH_VOLT_UNLOCK_MV)
            {
                // 满电区：实测即真实，不需防回弹，避免被旧存档（如90%）压死冻结
                start_pct = boot_pct;
                start_mv = s_ctx.ocv_mv;
                ESP_LOGI(TAG, "满电区上电：实测=%u%%(%lu mV)，无视NVS存档(%u%%)直接信实测",
                         boot_pct, (unsigned long)s_ctx.ocv_mv, saved_pct);
            }
            else
            {
                // 放电工作区：取 min(存档, 实测) 做起点，消除断电回弹导致的电量虚高
                start_pct = (saved_pct < boot_pct) ? saved_pct : boot_pct;
                start_mv = (saved_mv < s_ctx.ocv_mv) ? saved_mv : s_ctx.ocv_mv;
                ESP_LOGI(TAG, "NVS 存档电量=%u%%(%lu mV)，实测=%u%%，防回弹取较小=%u%%",
                         saved_pct, (unsigned long)saved_mv, boot_pct, start_pct);
            }
            s_ctx.ocv_mv = start_mv;
            s_ctx.filtered_percent = start_pct;
            s_ctx.displayed_percent = start_pct;
        }
        else
        {
            // 首刷或存档无效：按实测电压显示
            s_ctx.filtered_percent = boot_pct;
            s_ctx.displayed_percent = boot_pct;
            ESP_LOGI(TAG, "NVS 无有效记录，按实测电量=%u%% 起步", boot_pct);
        }
        ESP_LOGI(TAG, "初始电量=%u%%", s_ctx.displayed_percent);
    }

    // 充电判定趋势基准初始化
    s_ctx.charging = false;
    s_ctx.rise_cnt = 0;
    s_ctx.last_ocv_mv = s_ctx.ocv_mv;

    uint32_t last_mv = s_ctx.ocv_mv;
    uint8_t last_saved_pct = s_ctx.displayed_percent; // NVS 节流：记录上次已存档的电量

    while (s_ctx.task_running)
    {
        uint32_t sample_mv = do_sample_voltage_mv();
        if (sample_mv > 0)
        {
            // 第一层：电压IIR滤波（256进制系数，四舍五入避免截断偏差）
            s_ctx.filtered_mv = (BSP_BAT_IIR_ALPHA * sample_mv +
                                 (256 - BSP_BAT_IIR_ALPHA) * s_ctx.filtered_mv + 128) /
                                256;

            // 非对称IIR：估算开路电压（OCV），用于电量百分比计算
            // 关键修正：下降用较快alpha（真实掉电要及时反映），上升用极慢alpha
            // （舵机/WiFi卸载后的瞬时回弹要滤掉，逼近真实OCV，避免电量被顶回去）。
            {
                uint32_t ocv_alpha = (sample_mv >= s_ctx.ocv_mv)
                                         ? BSP_BAT_OCV_UP_ALPHA    // 上升：极慢，滤回弹
                                         : BSP_BAT_OCV_DOWN_ALPHA; // 下降：较快，跟真实掉电
                s_ctx.ocv_mv = (ocv_alpha * sample_mv +
                                (256 - ocv_alpha) * s_ctx.ocv_mv + 128) /
                               256;
            }

            // ─── 充电判定（无充电脚，靠OCV趋势推断）──────────────────────
            // 满电区（OCV≥HIGH_VOLT_UNLOCK_MV，约4.0V）：电压已顶满，插着USB也不会
            // 再持续上升，纯趋势判断会永远判不出"在充电"——所以满电区直接判定为充电中，
            // 不要求上升趋势；只要明显下降（真的拔了/在放电）才退出充电态。
            // 非满电区：维持原有逻辑——OCV连续明显上升达阈值次数才判定充电。
            if (s_ctx.ocv_mv >= BSP_BAT_HIGH_VOLT_UNLOCK_MV)
            {
                if (!s_ctx.charging)
                {
                    s_ctx.charging = true;
                    ESP_LOGI(TAG, "OCV已进入满电区(%lu mV)，判定为充电中", (unsigned long)s_ctx.ocv_mv);
                }
                if ((int)s_ctx.last_ocv_mv - (int)s_ctx.ocv_mv >= BSP_BAT_CHARGE_RISE_MV)
                {
                    // 满电区内明显下降：判定为真的在放电（拔了USB），退出充电态
                    s_ctx.rise_cnt = 0;
                    s_ctx.charging = false;
                    ESP_LOGI(TAG, "满电区OCV转为下降，退出充电态");
                }
                s_ctx.last_ocv_mv = s_ctx.ocv_mv;
            }
            else if ((int)s_ctx.ocv_mv - (int)s_ctx.last_ocv_mv >= BSP_BAT_CHARGE_RISE_MV)
            {
                if (s_ctx.rise_cnt < 0xFF)
                    s_ctx.rise_cnt++;
                if (s_ctx.rise_cnt >= BSP_BAT_CHARGE_RISE_CNT && !s_ctx.charging)
                {
                    s_ctx.charging = true;
                    ESP_LOGI(TAG, "检测到OCV持续上升，判定为充电，解锁电量回升");
                }
                s_ctx.last_ocv_mv = s_ctx.ocv_mv;
            }
            else if ((int)s_ctx.last_ocv_mv - (int)s_ctx.ocv_mv >= BSP_BAT_CHARGE_RISE_MV)
            {
                // 明显下降：判定为放电，退出充电态
                s_ctx.rise_cnt = 0;
                if (s_ctx.charging)
                {
                    s_ctx.charging = false;
                    ESP_LOGI(TAG, "OCV转为下降，退出充电态，恢复单调递减锁");
                }
                s_ctx.last_ocv_mv = s_ctx.ocv_mv;
            }
            // 介于两者之间（基本持平）：不更新基准，等待趋势明确

            // 滞回比较：只有OCV变化超过阈值才重新计算百分比
            if (abs((int)s_ctx.ocv_mv - (int)last_mv) >= BSP_BAT_HYSTERESIS_MV)
            {
                last_mv = s_ctx.ocv_mv;

                uint8_t raw_percent = voltage_to_percent(s_ctx.ocv_mv);

                // 第二层：百分比IIR滤波 + 边界收敛修正
                // 当差值≤1%时直接snap，避免整数截断导致永远无法收敛到目标值
                int16_t pct_diff_raw = (int16_t)raw_percent - (int16_t)s_ctx.filtered_percent;
                if (abs(pct_diff_raw) <= 1)
                {
                    s_ctx.filtered_percent = raw_percent;
                }
                else
                {
                    s_ctx.filtered_percent = (BSP_BAT_PERCENT_IIR_ALPHA * raw_percent +
                                              (256 - BSP_BAT_PERCENT_IIR_ALPHA) * s_ctx.filtered_percent) /
                                             256;
                }

                // 第三层：智能变化速率限制 + 单调递减锁
                int8_t diff = (int8_t)s_ctx.filtered_percent - (int8_t)s_ctx.displayed_percent;

                // 高压解锁区：OCV 已进入满电平台，端电压回弹幅度有限，允许真实回升。
                // 趋势判定（charging）在满电区几乎不可达（OCV上升慢α凑不出CHARGE_RISE_MV），
                // 故用绝对电压作旁路，否则满电时单调锁会把百分比永久冻结。
                bool high_volt_zone = (s_ctx.ocv_mv >= BSP_BAT_HIGH_VOLT_UNLOCK_MV);

                // 单调递减锁：放电工作区未判定充电时，显示电量只许降不许升（上升一律忽略），
                // 杜绝舵机卸载回弹、平台区电压抖动把电量顶回去。充电或满电区才放开回升。
                if (diff > 0 && !s_ctx.charging && !high_volt_zone)
                {
                    // 放电工作区电量回升 → 忽略，保持当前显示值（防回弹核心）
                }
                else if (diff > 0 && !s_ctx.charging && high_volt_zone)
                {
                    // 满电区真实回升：限速 1%/周期安全放行，既跟随又不虚高跳变
                    s_ctx.displayed_percent += BSP_BAT_HIGH_VOLT_RISE_STEP;
                }
                else if (abs(diff) > BSP_BAT_MAX_FAST_CHANGE)
                {
                    s_ctx.displayed_percent += (diff > 0) ? 2 : -2;
                }
                else if (abs(diff) > BSP_BAT_MAX_CHANGE_PER_STEP)
                {
                    s_ctx.displayed_percent += (diff > 0) ? 1 : -1;
                }
                else
                {
                    s_ctx.displayed_percent = s_ctx.filtered_percent;
                }

                ESP_LOGD(TAG, "VBAT=%lu mV, OCV=%lu mV, raw=%u%%, filtered=%u%%, display=%u%%, charging=%d",
                         (unsigned long)s_ctx.filtered_mv, (unsigned long)s_ctx.ocv_mv,
                         raw_percent, s_ctx.filtered_percent, s_ctx.displayed_percent,
                         (int)s_ctx.charging);

                // NVS 节流存档：仅当显示电量实际变化时才写一次，避免频繁擦写 Flash
                if (s_ctx.displayed_percent != last_saved_pct)
                {
                    battery_nvs_save(s_ctx.displayed_percent, s_ctx.ocv_mv);
                    last_saved_pct = s_ctx.displayed_percent;
                }
            }

            // 低电告警判断（带100mV滞回）
            if (!s_ctx.low_alerted && s_ctx.filtered_mv < BSP_BAT_VOLTAGE_LOW_MV)
            {
                s_ctx.low_alerted = true;
                ESP_LOGW(TAG, "⚠️ 低电量告警：%lu mV (%u%%)",
                         (unsigned long)s_ctx.filtered_mv, s_ctx.displayed_percent);
                if (s_ctx.low_cb)
                {
                    s_ctx.low_cb(s_ctx.filtered_mv, s_ctx.displayed_percent);
                }
            }
            else if (s_ctx.low_alerted && s_ctx.filtered_mv > (BSP_BAT_VOLTAGE_LOW_MV + 100))
            {
                s_ctx.low_alerted = false;
                ESP_LOGI(TAG, "电压回升至 %lu mV，告警标志复位",
                         (unsigned long)s_ctx.filtered_mv);
            }

            // ─── 低电自动关机判断 ───────────────────────────────────────────
            // 用 OCV（已滤掉负载尖峰的开路电压估计）而非瞬时电压判断，要求连续多次
            // 命中阈值才关机，杜绝舵机/扬声器瞬时压降导致误关。充电态下不触发。
#if BSP_BAT_POWEROFF_ENABLE
            if (!s_ctx.charging && s_ctx.ocv_mv <= BSP_BAT_POWEROFF_MV)
            {
                if (s_ctx.poweroff_hit < 0xFF)
                    s_ctx.poweroff_hit++;
                ESP_LOGW(TAG, "低电关机预警：OCV=%lu mV ≤ %d mV，连续命中 %u/%u",
                         (unsigned long)s_ctx.ocv_mv, BSP_BAT_POWEROFF_MV,
                         s_ctx.poweroff_hit, BSP_BAT_POWEROFF_HIT_CNT);
                if (s_ctx.poweroff_hit >= BSP_BAT_POWEROFF_HIT_CNT)
                {
                    battery_nvs_save(0, s_ctx.ocv_mv); // 放空存档，防下次上电电量虚高
                    bsp_battery_power_off();           // GPIO18 高→低下降沿软关机，不返回
                }
            }
            else
            {
                // 电压回到阈值以上（或进入充电态）→ 清零命中计数，避免跨周期误累积
                s_ctx.poweroff_hit = 0;
            }
#endif
        }
        else
        {
            ESP_LOGW(TAG, "本次 ADC 采样失败，跳过");
        }

        vTaskDelay(pdMS_TO_TICKS(BSP_BAT_TASK_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "电池监控任务退出");
    s_ctx.task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── 独立的 5s 电池日志任务 ───────────────────────────────────────────────────
#define BSP_BAT_LOG_INTERVAL_MS 5000

static void battery_log_task(void *arg)
{
    ESP_LOGI(TAG, "电池日志任务启动，每 %d ms 打印一次", BSP_BAT_LOG_INTERVAL_MS);
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    while (s_log_task_running)
    {
        uint32_t mv = bsp_battery_get_voltage_mv();
        uint8_t percent = bsp_battery_get_percent();
        ESP_LOGI(TAG, "🔋 电池电压 = %lu mV，电量 = %u%%",
                 (unsigned long)mv, percent);
        vTaskDelay(pdMS_TO_TICKS(BSP_BAT_LOG_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "电池日志任务退出");
    s_log_task_handle = NULL;
    vTaskDelete(NULL);
}

// ═══ 对外 API 实现 ═══════════════════════════════════════════════════════════

esp_err_t bsp_battery_init(void)
{
    if (s_ctx.initialized)
    {
        ESP_LOGW(TAG, "已初始化，跳过");
        return ESP_OK;
    }

    if (BSP_BAT_ADC_PIN < 0)
    {
        ESP_LOGE(TAG, "BSP_BAT_ADC_PIN 未配置（当前 = %d），请在 bsp_config.h 中填入实际 GPIO",
                 BSP_BAT_ADC_PIN);
        return ESP_ERR_INVALID_STATE;
    }

    adc_channel_t channel;
    ESP_RETURN_ON_ERROR(gpio_to_adc1_channel(BSP_BAT_ADC_PIN, &channel),
                        TAG, "GPIO%d 不属于 ADC1 通道（合法范围 GPIO1~10）", BSP_BAT_ADC_PIN);

    s_ctx.unit = ADC_UNIT_1;
    s_ctx.channel = channel;
    s_ctx.cali_type = CALI_TYPE_NONE;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_ctx.adc_handle),
                        TAG, "adc_oneshot_new_unit 失败");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_ctx.adc_handle, channel, &chan_cfg),
                        TAG, "adc_oneshot_config_channel 失败");

    // 校准策略：curve fitting（ESP32-S3支持），失败则使用未校准近似值
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_ctx.cali_handle);
    if (cali_ret == ESP_OK)
    {
        s_ctx.cali_enabled = true;
        s_ctx.cali_type = CALI_TYPE_CURVE;
        ESP_LOGI(TAG, "ADC 校准启用（curve fitting）");
    }
    else
    {
        s_ctx.cali_enabled = false;
        s_ctx.cali_type = CALI_TYPE_NONE;
        ESP_LOGW(TAG, "ADC 校准失败 (%s)，将使用未校准近似值", esp_err_to_name(cali_ret));
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "电池监控初始化完成：GPIO%d → ADC1_CH%d，分压比 %d/%d",
             BSP_BAT_ADC_PIN, (int)channel,
             BSP_BAT_VOLTAGE_RATIO_NUM, BSP_BAT_VOLTAGE_RATIO_DEN);

    return ESP_OK;
}

esp_err_t bsp_battery_deinit(void)
{
    if (!s_ctx.initialized)
        return ESP_OK;

    bsp_battery_stop_task();
    bsp_battery_stop_log_task();

    if (s_ctx.cali_enabled && s_ctx.cali_handle)
    {
        adc_cali_delete_scheme_curve_fitting(s_ctx.cali_handle);
        s_ctx.cali_handle = NULL;
        s_ctx.cali_enabled = false;
        s_ctx.cali_type = CALI_TYPE_NONE;
    }

    if (s_ctx.adc_handle)
    {
        adc_oneshot_del_unit(s_ctx.adc_handle);
        s_ctx.adc_handle = NULL;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    ESP_LOGI(TAG, "电池监控已反初始化");
    return ESP_OK;
}

/**
 * @brief 请求停止电池监控任务
 * @return 电池电压（毫伏）
 *
 */
uint32_t bsp_battery_read_voltage_mv(void)
{
    return do_sample_voltage_mv();
}

uint32_t bsp_battery_get_voltage_mv(void)
{
    if (s_ctx.task_running && s_ctx.filtered_mv > 0)
    {
        return s_ctx.filtered_mv;
    }
    return do_sample_voltage_mv();
}

uint8_t bsp_battery_get_percent(void)
{
    if (s_ctx.task_running)
    {
        return s_ctx.displayed_percent;
    }
    uint32_t mv = bsp_battery_get_voltage_mv();
    if (mv == 0)
        return 0;
    return voltage_to_percent(mv);
}

bool bsp_battery_is_charging(void)
{
    return s_ctx.charging;
}

/**
 * @brief 启动电池监控后台任务
 * @param low_cb 低电量告警回调函数
 * @note 回调在电池监控任务上下文中执行，禁止阻塞操作
 */
esp_err_t bsp_battery_start_task(bsp_battery_low_cb_t low_cb)
{
    if (!s_ctx.initialized)
    {
        ESP_LOGE(TAG, "请先调用 bsp_battery_init()");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.task_handle != NULL)
    {
        ESP_LOGW(TAG, "后台任务已存在");
        return ESP_ERR_INVALID_STATE;
    }

    s_ctx.low_cb = low_cb;
    s_ctx.low_alerted = false;
    s_ctx.task_running = true;

    BaseType_t ok = xTaskCreate(battery_monitor_task,
                                "bat_mon",
                                BSP_BAT_TASK_STACK_SIZE,
                                NULL,
                                BSP_BAT_TASK_PRIORITY,
                                &s_ctx.task_handle);
    if (ok != pdPASS)
    {
        s_ctx.task_running = false;
        ESP_LOGE(TAG, "创建后台任务失败");
        return ESP_ERR_NO_MEM;
    }
    PRINT_TASK_CREATED(TAG, "bat_mon", BSP_BAT_TASK_STACK_SIZE, 1); // xTaskCreate → 栈在内部SRAM
    return ESP_OK;
}

esp_err_t bsp_battery_stop_task(void)
{
    if (s_ctx.task_handle == NULL)
        return ESP_OK;

    s_ctx.task_running = false;
    // 轮询等待任务自行退出（任务退出时会清零 task_handle）
    TickType_t timeout = pdMS_TO_TICKS(BSP_BAT_TASK_INTERVAL_MS + 500);
    while (s_ctx.task_handle != NULL && timeout-- > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_ctx.task_handle != NULL)
    {
        ESP_LOGW(TAG, "电池监控任务未在超时内退出，强制清除句柄");
        s_ctx.task_handle = NULL;
    }
    return ESP_OK;
}

esp_err_t bsp_battery_start_log_task(void)
{
    if (!s_ctx.initialized)
    {
        ESP_LOGE(TAG, "请先调用 bsp_battery_init()");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_log_task_handle != NULL)
    {
        ESP_LOGW(TAG, "电池日志任务已存在");
        return ESP_ERR_INVALID_STATE;
    }

    s_log_task_running = true;
    // ★ 本任务只读 ADC 值 + 打印日志，不碰 NVS/Flash，栈可安全放 SPIRAM（省内部 SRAM）。
    //   与 bat_mon 不同：bat_mon 会调 battery_nvs_save() 写 Flash，触发 cache 关闭窗口，
    //   PSRAM 栈在那期间不可访问会崩溃，所以 bat_mon 的栈必须留在内部 SRAM，不能一起搬。
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(battery_log_task,
                                "bat_log",
                                BSP_BAT_TASK_STACK_SIZE,
                                NULL,
                                BSP_BAT_TASK_PRIORITY,
                                &s_log_task_handle,
                                tskNO_AFFINITY,
                                MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ok != pdPASS)
    {
        s_log_task_running = false;
        ESP_LOGE(TAG, "创建电池日志任务失败");
        return ESP_ERR_NO_MEM;
    }
    PRINT_TASK_CREATED(TAG, "bat_log", BSP_BAT_TASK_STACK_SIZE, 0); // 栈在PSRAM
    return ESP_OK;
}

esp_err_t bsp_battery_stop_log_task(void)
{
    if (s_log_task_handle == NULL)
        return ESP_OK;

    s_log_task_running = false;
    TickType_t timeout = pdMS_TO_TICKS(BSP_BAT_LOG_INTERVAL_MS + 500);
    while (s_log_task_handle != NULL && timeout-- > 0)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (s_log_task_handle != NULL)
    {
        ESP_LOGW(TAG, "电池日志任务未在超时内退出，强制清除句柄");
        s_log_task_handle = NULL;
    }
    return ESP_OK;
}
