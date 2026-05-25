/**
 * @file bsp_battery.c
 * @brief 电池电压监控模块实现
 *
 * 详细设计见 bsp_battery.h 文件头注释。
 *
 * 关键实现细节：
 *   1. 使用 ESP-IDF v5.x 新 ADC oneshot API（adc_oneshot_new_unit / read），
 *      旧的 adc1_get_raw 已被弃用。
 *   2. 校准优先使用 curve fitting（ESP32-S3 支持），失败时回退到不校准（仅原始电压）。
 *   3. 分压网络 R23/R24 比例由 bsp_config.h 宏定义，避免硬编码。
 *   4. 锂电池放电曲线采用分段线性插值，比纯线性映射准确得多。
 *   5. IIR 低通滤波：v_new = α·v_sample + (1-α)·v_old，α 由宏配置。
 *   6. 低电告警带 100mV 滞回，避免在阈值附近反复触发。
 */

#include "bsp_board.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "driver/gpio.h"
#include <string.h>

static const char *TAG = "bsp_battery";

// ─── 内部状态结构 ─────────────────────────────────────────────────────────────
typedef struct {
    bool             initialized;        ///< ADC 是否已初始化
    bool             cali_enabled;       ///< 校准句柄是否有效
    adc_oneshot_unit_handle_t adc_handle;///< ADC1 oneshot 句柄
    adc_cali_handle_t cali_handle;       ///< curve fitting 校准句柄
    adc_channel_t    channel;            ///< 实际使用的 ADC 通道
    adc_unit_t       unit;               ///< ADC 单元（始终为 ADC_UNIT_1）

    TaskHandle_t     task_handle;        ///< 后台采样任务句柄
    volatile bool    task_running;       ///< 任务运行标志（用于优雅退出）
    bsp_battery_low_cb_t low_cb;         ///< 低电量告警回调
    bool             low_alerted;        ///< 低电告警是否已触发（用于滞回）

    uint32_t         filtered_mv;        ///< IIR 滤波后的电压（毫伏）
} bsp_battery_ctx_t;

static bsp_battery_ctx_t s_ctx = {0};

// ─── GPIO → ADC1 通道映射（ESP32-S3 专用）─────────────────────────────────────
// ESP32-S3 ADC1 对应 GPIO1~GPIO10，通道号 = GPIO号 - 1
// 例：GPIO5 → ADC1_CH4，GPIO1 → ADC1_CH0
static esp_err_t gpio_to_adc1_channel(int gpio, adc_channel_t *out_ch)
{
    if (gpio < 1 || gpio > 10) {
        return ESP_ERR_INVALID_ARG;
    }
    *out_ch = (adc_channel_t)(gpio - 1);
    return ESP_OK;
}

// ─── 锂电池放电曲线分段插值（电压 mV → 百分比 0~100）─────────────────────────
// 该曲线为常见 LiPo 18650 的典型放电曲线，比线性映射准确得多
typedef struct {
    uint32_t mv;
    uint8_t  percent;
} bat_curve_point_t;

static const bat_curve_point_t s_curve[] = {
    {4200, 100}, {4050, 90}, {3950, 80}, {3850, 70},
    {3800, 60},  {3750, 50}, {3700, 40}, {3650, 30},
    {3600, 20},  {3450, 10}, {3300, 5},  {3000, 0},
};
#define BAT_CURVE_LEN (sizeof(s_curve) / sizeof(s_curve[0]))

static uint8_t voltage_to_percent(uint32_t mv)
{
    // 上下边界裁剪
    if (mv >= s_curve[0].mv) return 100;
    if (mv <= s_curve[BAT_CURVE_LEN - 1].mv) return 0;

    // 在分段曲线中找到包含 mv 的区间，做线性插值
    for (size_t i = 0; i < BAT_CURVE_LEN - 1; i++) {
        uint32_t v_hi = s_curve[i].mv;
        uint32_t v_lo = s_curve[i + 1].mv;
        if (mv <= v_hi && mv >= v_lo) {
            uint8_t p_hi = s_curve[i].percent;
            uint8_t p_lo = s_curve[i + 1].percent;
            uint32_t span_v = v_hi - v_lo;
            uint32_t span_p = p_hi - p_lo;
            return (uint8_t)(p_lo + (mv - v_lo) * span_p / span_v);
        }
    }
    return 0;
}

// ─── 内部：执行一次 ADC 多采样平均，返回真实电池电压（毫伏）──────────────────
static uint32_t do_sample_voltage_mv(void)
{
    if (!s_ctx.initialized) return 0;

    uint32_t adc_mv_sum = 0;
    uint32_t valid_count = 0;

    for (int i = 0; i < BSP_BAT_ADC_SAMPLE_TIMES; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_ctx.adc_handle, s_ctx.channel, &raw) != ESP_OK) {
            continue;
        }

        int mv = 0;
        if (s_ctx.cali_enabled) {
            // 校准后直接得到毫伏
            if (adc_cali_raw_to_voltage(s_ctx.cali_handle, raw, &mv) != ESP_OK) {
                continue;
            }
        } else {
            // 无校准回退：raw / 4095 × 3100 (ATTEN_12 量程粗略估算，精度差)
            mv = raw * 3100 / 4095;
        }
        adc_mv_sum += (uint32_t)mv;
        valid_count++;
    }

    if (valid_count == 0) return 0;

    uint32_t adc_mv_avg = adc_mv_sum / valid_count;
    // 反向还原分压：真实电压 = ADC电压 × (R_HIGH + R_LOW) / R_LOW
    uint32_t vbat_mv = adc_mv_avg * BSP_BAT_VOLTAGE_RATIO_NUM / BSP_BAT_VOLTAGE_RATIO_DEN;
    return vbat_mv;
}

// ─── 后台采样任务 ─────────────────────────────────────────────────────────────
static void battery_monitor_task(void *arg)
{
    ESP_LOGI(TAG, "电池监控任务启动，采样周期 %d ms", BSP_BAT_TASK_INTERVAL_MS);

    // 首次采样初始化 filtered_mv，避免从 0 开始爬升
    s_ctx.filtered_mv = do_sample_voltage_mv();

    while (s_ctx.task_running) {
        uint32_t sample_mv = do_sample_voltage_mv();
        if (sample_mv > 0) {
            // IIR 低通滤波：v_new = α·sample + (1-α)·v_old
            // 用百分比避免浮点：filtered = (α·sample + (100-α)·filtered) / 100
            uint32_t alpha = BSP_BAT_IIR_ALPHA_PERCENT;
            s_ctx.filtered_mv = (alpha * sample_mv + (100 - alpha) * s_ctx.filtered_mv) / 100;

            uint8_t percent = voltage_to_percent(s_ctx.filtered_mv);
            ESP_LOGD(TAG, "VBAT=%lu mV (raw=%lu), %u%%",
                     (unsigned long)s_ctx.filtered_mv, (unsigned long)sample_mv, percent);

            // 低电告警判断（带 100mV 滞回）
            if (!s_ctx.low_alerted && s_ctx.filtered_mv < BSP_BAT_VOLTAGE_LOW_MV) {
                s_ctx.low_alerted = true;
                ESP_LOGW(TAG, "⚠️ 低电量告警：%lu mV (%u%%)",
                         (unsigned long)s_ctx.filtered_mv, percent);
                if (s_ctx.low_cb) {
                    s_ctx.low_cb(s_ctx.filtered_mv, percent);
                }
            } else if (s_ctx.low_alerted && s_ctx.filtered_mv > (BSP_BAT_VOLTAGE_LOW_MV + 100)) {
                // 电压回升超过滞回区间，允许下次再次告警
                s_ctx.low_alerted = false;
                ESP_LOGI(TAG, "电压回升至 %lu mV，告警标志复位",
                         (unsigned long)s_ctx.filtered_mv);
            }
        } else {
            ESP_LOGW(TAG, "本次 ADC 采样失败，跳过");
        }

        vTaskDelay(pdMS_TO_TICKS(BSP_BAT_TASK_INTERVAL_MS));
    }

    ESP_LOGI(TAG, "电池监控任务退出");
    s_ctx.task_handle = NULL;
    vTaskDelete(NULL);
}

// ═══ 对外 API 实现 ═══════════════════════════════════════════════════════════

esp_err_t bsp_battery_init(void)
{
    if (s_ctx.initialized) {
        ESP_LOGW(TAG, "已初始化，跳过");
        return ESP_OK;
    }

    // 占位检查：BSP_BAT_ADC_PIN 必须由硬件确认后填入有效 GPIO
    if (BSP_BAT_ADC_PIN < 0) {
        ESP_LOGE(TAG, "BSP_BAT_ADC_PIN 未配置（当前 = %d），请在 bsp_config.h 中填入实际 GPIO",
                 BSP_BAT_ADC_PIN);
        return ESP_ERR_INVALID_STATE;
    }

    // GPIO → ADC1 通道转换
    adc_channel_t channel;
    ESP_RETURN_ON_ERROR(gpio_to_adc1_channel(BSP_BAT_ADC_PIN, &channel),
                        TAG, "GPIO%d 不属于 ADC1 通道（合法范围 GPIO1~10）", BSP_BAT_ADC_PIN);

    s_ctx.unit = ADC_UNIT_1;
    s_ctx.channel = channel;

    // 1. 创建 ADC1 oneshot 单元
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &s_ctx.adc_handle),
                        TAG, "adc_oneshot_new_unit 失败");

    // 2. 配置通道：12位精度 + 12dB 衰减（量程 0~3.1V）
    adc_oneshot_chan_cfg_t chan_cfg = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(s_ctx.adc_handle, channel, &chan_cfg),
                        TAG, "adc_oneshot_config_channel 失败");

    // 3. 创建 curve fitting 校准句柄（ESP32-S3 支持）
    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id = ADC_UNIT_1,
        .chan = channel,
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    esp_err_t cali_ret = adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_ctx.cali_handle);
    if (cali_ret == ESP_OK) {
        s_ctx.cali_enabled = true;
        ESP_LOGI(TAG, "ADC 校准启用（curve fitting）");
    } else {
        s_ctx.cali_enabled = false;
        ESP_LOGW(TAG, "ADC 校准创建失败 (%s)，将使用未校准近似值", esp_err_to_name(cali_ret));
    }

    s_ctx.initialized = true;
    ESP_LOGI(TAG, "电池监控初始化完成：GPIO%d → ADC1_CH%d，分压比 %d/%d",
             BSP_BAT_ADC_PIN, (int)channel,
             BSP_BAT_VOLTAGE_RATIO_DEN, BSP_BAT_VOLTAGE_RATIO_NUM);

    return ESP_OK;
}

esp_err_t bsp_battery_deinit(void)
{
    if (!s_ctx.initialized) return ESP_OK;

    // 先停掉后台任务
    bsp_battery_stop_task();

    if (s_ctx.cali_enabled && s_ctx.cali_handle) {
        adc_cali_delete_scheme_curve_fitting(s_ctx.cali_handle);
        s_ctx.cali_handle = NULL;
        s_ctx.cali_enabled = false;
    }
    if (s_ctx.adc_handle) {
        adc_oneshot_del_unit(s_ctx.adc_handle);
        s_ctx.adc_handle = NULL;
    }

    memset(&s_ctx, 0, sizeof(s_ctx));
    ESP_LOGI(TAG, "电池监控已反初始化");
    return ESP_OK;
}

uint32_t bsp_battery_read_voltage_mv(void)
{
    return do_sample_voltage_mv();
}

uint32_t bsp_battery_get_voltage_mv(void)
{
    // 若后台任务在运行，返回滤波值；否则现场采样一次
    if (s_ctx.task_running && s_ctx.filtered_mv > 0) {
        return s_ctx.filtered_mv;
    }
    return do_sample_voltage_mv();
}

uint8_t bsp_battery_get_percent(void)
{
    uint32_t mv = bsp_battery_get_voltage_mv();
    if (mv == 0) return 0;
    return voltage_to_percent(mv);
}

esp_err_t bsp_battery_start_task(bsp_battery_low_cb_t low_cb)
{
    if (!s_ctx.initialized) {
        ESP_LOGE(TAG, "请先调用 bsp_battery_init()");
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.task_handle != NULL) {
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
    if (ok != pdPASS) {
        s_ctx.task_running = false;
        ESP_LOGE(TAG, "创建后台任务失败");
        return ESP_ERR_NO_MEM;
    }
    return ESP_OK;
}

esp_err_t bsp_battery_stop_task(void)
{
    if (s_ctx.task_handle == NULL) return ESP_OK;
    s_ctx.task_running = false;
    // 任务在下个循环 tick 退出（最长等待 BSP_BAT_TASK_INTERVAL_MS）
    // 此处不强删，避免在持有 ADC 锁时被删导致死锁
    return ESP_OK;
}
