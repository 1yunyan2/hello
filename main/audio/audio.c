#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "esp_log.h"
#include "esp_err.h"
static const char *TAG = "MAX98357A_TEST";

// 和上面接线表完全对应，不用修改
#define BCLK_PIN        GPIO_NUM_5
#define WS_PIN          GPIO_NUM_6
#define DOUT_PIN        GPIO_NUM_4

i2s_chan_handle_t tx_chan;

void i2s_init(void) {
    i2s_chan_config_t tx_chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&tx_chan_cfg, &tx_chan, NULL));

    i2s_std_config_t tx_std_cfg = {
        // 模块支持8K~96KHz，44100Hz兼容性最好
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(44100),
        // 模块是单声道，用飞利浦标准I2S，16bit位宽，和模块完全匹配
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED, // 模块无需MCLK，和参数说明一致
            .bclk = BCLK_PIN,
            .ws   = WS_PIN,
            .dout = DOUT_PIN,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv   = false,
            },
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_chan, &tx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_chan));
}

void play_audio_task(void *pvParameters) {
    // 单声道音频buffer，100个采样点
    int16_t buffer[100];
    size_t bytes_written = 0;
    int tick = 0;

    ESP_LOGI(TAG, "MAX98357A 音频测试启动");

    while (1) {
        // 生成440Hz标准音方波，幅度拉满，确保能听到声音
        for (int i = 0; i < 100; i++) {
            tick++;
            int16_t val = (tick % 100 < 50) ? 30000 : -30000;
            buffer[i] = val;
        }

        // 写入I2S数据，portMAX_DELAY确保写入完成
        esp_err_t res = i2s_channel_write(tx_chan, buffer, sizeof(buffer), &bytes_written, portMAX_DELAY);
        
        if (res != ESP_OK) {
            ESP_LOGE(TAG, "I2S写入错误: %s", esp_err_to_name(res));
        }

        // 每10000个采样点打印一次心跳，确认代码正常运行
        if (tick % 10000 == 0) {
            ESP_LOGI(TAG, "正常写入字节数: %d", bytes_written);
        }                                          
    }
}


