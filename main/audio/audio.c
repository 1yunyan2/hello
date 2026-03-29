#include "audio.h"

static const char *TAG = "AUDIO_ES8311";

#define I2C_SDA_PIN BSP_CODEC_SDA_PIN
#define I2C_SCL_PIN BSP_CODEC_SCL_PIN
#define I2S_MCLK_PIN BSP_CODEC_MCLK_PIN
#define I2S_BCLK_PIN BSP_CODEC_BCLK_PIN
#define I2S_WS_PIN BSP_CODEC_WS_PIN
#define I2S_DIN_PIN BSP_CODEC_DIN_PIN
#define I2S_DOUT_PIN BSP_CODEC_DOUT_PIN

// 全局音频设备句柄
esp_codec_dev_handle_t codec_dev;
i2s_chan_handle_t rx_handle;
i2s_chan_handle_t tx_handle;

void audio_init(void)
{
    ESP_LOGI(TAG, "正在初始化 ES8311...");
    // 🌟 修复大坑二：栈内存扩大到 8192，并绑定到 CPU 核心 1 专职运算语音
    xTaskCreatePinnedToCore(audio_feed_task, "audio_feed", 8192, NULL, 5, NULL, 1);

    // 1. 初始化 I2C 控制总线
    i2c_master_bus_handle_t bus_handle;
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_PIN,
        .scl_io_num = I2C_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, &bus_handle));

    ESP_LOGI(TAG, "开始探测 I2C 总线上的设备...");
    int found = 0;
    for (uint16_t addr = 1; addr < 127; addr++)
    {
        // probe 函数会在总线上寻找对应地址的设备，超时设为 50 毫秒
        esp_err_t probe_ret = i2c_master_probe(bus_handle, addr, 50);
        if (probe_ret == ESP_OK)
        {
            ESP_LOGI(TAG, " 扫描的设备地址为: 0x%02x", addr);
            found++;
        }
    }
    if (found == 0)
    {
        ESP_LOGE(TAG, " 未扫描到设备地址，请检查SDA/SCL连线！");
    }

    audio_codec_i2c_cfg_t i2c_cfg = {
        .bus_handle = bus_handle,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

    // 2. 初始化 I2S 数据总线
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &tx_handle, &rx_handle));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),                           // 🚨 SR 引擎强制要求 16000Hz 采样率
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(16, I2S_SLOT_MODE_MONO), // 16bit 单声道
        .gpio_cfg = {
            .mclk = I2S_MCLK_PIN,
            .bclk = I2S_BCLK_PIN,
            .ws = I2S_WS_PIN,
            .dout = I2S_DOUT_PIN,
            .din = I2S_DIN_PIN,
        },
    };
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(rx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(tx_handle, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(rx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(tx_handle));

    // 3. 组合并打开 ES8311 Codec 设备
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        // .pa_pin = PA_PIN,//作用：输出音频时，将音频输出到 PA 引脚
        .pa_pin = -1,                               // 因为这个开发板将pa使能直接连接了电源，默认拉高，一直长开
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH, // 双工模式
        .use_mclk = true,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);
    audio_codec_i2s_cfg_t i2s_config = {
        .rx_handle = rx_handle,
        .tx_handle = tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_config);
    esp_codec_dev_cfg_t codec_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    codec_dev = esp_codec_dev_new(&codec_config); // 创建音频设备句柄

    // 设置麦克风增益和喇叭音量
    esp_codec_dev_set_in_gain(codec_dev, 40); // 录音增益大一些
    esp_codec_dev_set_out_vol(codec_dev, 70); // 喇叭音量小一些

    esp_codec_dev_sample_info_t sample_info = {
        .sample_rate = 16000,
        .bits_per_sample = 16,
        .channel = 1, // 单声道
    };
    esp_codec_dev_open(codec_dev, &sample_info);
    ESP_LOGI(TAG, "✅ ES8311 麦克风与功放初始化完成！");
}

// 录音投喂任务：死循环抓取麦克风声音塞给语音识别引擎
void audio_feed_task(void *arg)
{
    size_t chunk_size = custom_wake_word_get_chunksize(); // 获取唤醒词引擎需要的数据块大小
    if (chunk_size == 0)
        chunk_size = 512;

    int16_t *buffer = malloc(chunk_size * sizeof(int16_t));
    ESP_LOGI(TAG, "🎙️ 正在持续监听环境声音...");

    while (1)
    {
        // 从 ES8311 麦克风读取数据
        esp_err_t ret = esp_codec_dev_read(codec_dev, buffer, chunk_size * sizeof(int16_t));
        if (ret == ESP_OK)
        {
            // 将 16KHz 的音频流喂给唤醒词引擎
            custom_wake_word_feed(buffer, chunk_size);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    free(buffer);
    vTaskDelete(NULL);
}