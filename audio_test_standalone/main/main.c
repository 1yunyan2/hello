/**
 * @file main.c
 * @brief audio_test_standalone 入口
 *
 * 纯离线音频测试（无需任何模型文件）：
 *   - 麦克风采集 → [PCM诊断] peak/rms/原始bytes 打印（约 3s 一次）
 *   - AFE 降噪 + WebRTC VAD → [VAD] 语音活动状态变化打印
 *
 * 不需要 model 分区、不需要 .pb 文件、不需要 SPIFFS。
 */

#include "bsp_board.h"
#include "custom_wake_word.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_vad.h"

static const char *TAG = "AUDIO_TEST";

static void on_vad_state(vad_state_t state)
{
    static vad_state_t last_state = VAD_SILENCE;
    if (state == last_state)
        return;

    if (state == VAD_SPEECH)
        ESP_LOGI(TAG, "[VAD] >>> 检测到说话");
    else
        ESP_LOGI(TAG, "[VAD] <<< 说话结束");

    last_state = state;
}

void app_main(void)
{
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════");
    ESP_LOGI(TAG, "  audio_test_standalone — 纯离线麦克风+VAD+PCM诊断测试");
    ESP_LOGI(TAG, "════════════════════════════════════════════════════════");

    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << GPIO_NUM_14),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_14, 0);

    bsp_board_t *bsp = bsp_board_get_instance();
    bsp_board_nvs_init(bsp);

    // 轻量初始化：仅 AFE + VAD，不需要 MultiNet 模型文件
    if (wake_word_init_lite() != ESP_OK) {
        ESP_LOGE(TAG, "wake_word_init_lite 失败");
        return;
    }

    bsp_wake_word_set_vad_callback(on_vad_state);

    audio_init(bsp);

    ESP_LOGI(TAG, "═══ 初始化完成，请对着麦克风说话 ═══");
    ESP_LOGI(TAG, "看 [PCM诊断] 验证麦克风信号；看 [VAD] 验证人声检测");
}
