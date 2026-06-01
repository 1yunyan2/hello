/**
 * @file main.c
 * @brief audio_test_standalone — 麦克风 + AFE/VAD + PCM 能量诊断 demo
 *
 * 用途：拿到一块新板子，烧录这个 demo 可以快速验证：
 *   1) ES8311 焊接 + I2C/I2S 时钟 → 看启动期 [I2C诊断] / [PCM诊断]
 *   2) 麦克风采集是否有信号       → 看 [PCM] peak / rms
 *   3) WebRTC VAD 是否能区分人声  → 看 [VAD] 状态变化
 *
 * 所用接口全部来自 ESP-IDF 公开组件：
 *   - esp_codec_dev   : 操作 ES8311
 *   - esp-sr (AFE)    : 通用音频前端 + WebRTC VAD（参数取自官方文档默认值）
 */

#include "bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_vad.h"
#include "esp_afe_sr_iface.h"
#include "esp_afe_sr_models.h"
#include <math.h>
#include <inttypes.h>

static const char *TAG = "AUDIO_TEST";

/* AFE 句柄（演示用全局变量；正式工程建议封装到上下文结构体） */
static const esp_afe_sr_iface_t *s_afe_iface = NULL;
static esp_afe_sr_data_t        *s_afe_data  = NULL;

/* ─── 采集任务：codec → PCM 诊断 → 投喂 AFE ───────────────────────────────── */
static void audio_feed_task(void *arg)
{
    bsp_board_t *bsp = (bsp_board_t *)arg;

    int chunk = s_afe_iface->get_feed_chunksize(s_afe_data);  // AFE 期望的采样点数
    int16_t *buf = heap_caps_malloc(chunk * sizeof(int16_t),
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!buf) {
        ESP_LOGE(TAG, "采集缓冲区分配失败 (%d bytes)", (int)(chunk * sizeof(int16_t)));
        vTaskDelete(NULL);
        return;
    }

    /* PCM 诊断累计变量，每 ~3 秒打印一次 */
    uint32_t iter = 0;
    int32_t  peak = 0;
    uint64_t sumsq = 0;
    const uint32_t PRINT_EVERY = 16000u / (uint32_t)chunk * 3u;

    ESP_LOGI(TAG, "采集任务启动 (feed chunk=%d samples = %d bytes)",
             chunk, (int)(chunk * sizeof(int16_t)));

    while (1) {
        esp_err_t ret = esp_codec_dev_read(bsp->codec_dev, buf, chunk * sizeof(int16_t));
        if (ret != ESP_OK) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        /* 累计 peak / 平方和 */
        for (int i = 0; i < chunk; i++) {
            int32_t v  = buf[i];
            int32_t av = v < 0 ? -v : v;
            if (av > peak) peak = av;
            sumsq += (uint64_t)(v * v);
        }
        if (++iter >= PRINT_EVERY) {
            uint64_t total = (uint64_t)iter * (uint64_t)chunk;
            uint32_t rms   = (uint32_t)sqrt((double)sumsq / (double)total);
            ESP_LOGI(TAG, "[PCM] peak=%" PRId32 " rms=%" PRIu32
                          "  原始bytes[0..7]=%04X %04X %04X %04X %04X %04X %04X %04X",
                     peak, rms,
                     (uint16_t)buf[0], (uint16_t)buf[1], (uint16_t)buf[2], (uint16_t)buf[3],
                     (uint16_t)buf[4], (uint16_t)buf[5], (uint16_t)buf[6], (uint16_t)buf[7]);
            iter = 0; peak = 0; sumsq = 0;
        }

        /* 投喂 AFE（单麦 "M" 模式，buf 长度 = chunk samples） */
        s_afe_iface->feed(s_afe_data, buf);
    }
}

/* ─── VAD 任务：从 AFE fetch 结果里读 vad_state 并打印 ───────────────────── */
static void afe_fetch_task(void *arg)
{
    vad_state_t last = VAD_SILENCE;
    while (1) {
        afe_fetch_result_t *r = s_afe_iface->fetch(s_afe_data);
        if (!r || r->ret_value == ESP_FAIL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }
        if (r->vad_state != last) {
            if (r->vad_state == VAD_SPEECH)
                ESP_LOGI(TAG, "[VAD] >>> 检测到说话");
            else
                ESP_LOGI(TAG, "[VAD] <<< 说话结束");
            last = r->vad_state;
        }
    }
}

/* ─── app_main ─────────────────────────────────────────────────────────────── */
void app_main(void)
{
    ESP_LOGI(TAG, "════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  audio_test_standalone — 麦克风 + VAD demo");
    ESP_LOGI(TAG, "════════════════════════════════════════════════");

    /* 防止某些复用引脚启动期误动作（视板型可调，无相关外设可移除） */
    gpio_config_t io_conf = {
        .pin_bit_mask    = (1ULL << GPIO_NUM_14),
        .mode            = GPIO_MODE_OUTPUT,
        .pull_up_en      = GPIO_PULLUP_DISABLE,
        .pull_down_en    = GPIO_PULLDOWN_ENABLE,
        .intr_type       = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_14, 0);

    /* 1. BSP 基础设施 */
    bsp_board_t *bsp = bsp_board_get_instance();
    bsp_board_nvs_init(bsp);

    /* 2. ES8311 硬件初始化 + 打开设备 + 设置增益 */
    audio_init(bsp);

    /* 3. AFE 初始化（单麦 "M"，关 AEC/AGC/Wakenet/SE，开 NS+VAD） */
    afe_config_t *cfg = afe_config_init("M", NULL, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (!cfg) { ESP_LOGE(TAG, "afe_config_init 失败"); return; }

    cfg->aec_init       = false;        /* 没有参考信号 */
    cfg->se_init        = false;        /* 单麦无 BSS */
    cfg->wakenet_init   = false;        /* 不需要唤醒词模型 */
    cfg->ns_init        = true;         /* 开 NS 降噪 */
    cfg->agc_init       = false;        /* 关 AGC，防止把信号二次放大到截幅 */
    cfg->vad_init       = true;         /* 开 WebRTC VAD */
    cfg->vad_mode       = VAD_MODE_3;   /* 严格模式：抗拍桌/摩擦/风扇等突发噪音 */
    cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;

    s_afe_iface = esp_afe_handle_from_config(cfg);
    if (!s_afe_iface) { ESP_LOGE(TAG, "esp_afe_handle_from_config 失败"); afe_config_free(cfg); return; }

    s_afe_data = s_afe_iface->create_from_config(cfg);
    afe_config_free(cfg);
    if (!s_afe_data) { ESP_LOGE(TAG, "AFE create_from_config 失败（PSRAM 不足？）"); return; }

    ESP_LOGI(TAG, "AFE 初始化完成 (feed=%d samples, fetch=%d samples)",
             s_afe_iface->get_feed_chunksize(s_afe_data),
             s_afe_iface->get_fetch_chunksize(s_afe_data));

    /* 4. 启动采集任务（投喂 AFE）+ VAD 任务（消费 AFE 输出） */
    xTaskCreatePinnedToCore(audio_feed_task, "audio_feed", 8192, bsp, 5, NULL, 1);
    xTaskCreatePinnedToCore(afe_fetch_task,  "afe_fetch",  4096, NULL, 5, NULL, 1);

    ESP_LOGI(TAG, "═══ 初始化完成，请对着麦克风说话 ═══");
    ESP_LOGI(TAG, "[PCM] 验证麦克风信号；[VAD] 验证人声检测");
}
