#include "bsp_board.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "wake_words/custom_wake_word.h"

static const char *TAG = "BSP_CODEC";

// ==================== 私有硬件初始化函数 ====================

static void bsp_board_codec_i2c_init(bsp_board_t *bsp_board, i2c_master_bus_handle_t *bus_handle)
{
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = BSP_CODEC_SDA_PIN,
        .scl_io_num = BSP_CODEC_SCL_PIN,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, bus_handle));
}

static void bsp_board_codec_i2s_init(bsp_board_t *bsp_board, i2s_chan_handle_t *rx_handle, i2s_chan_handle_t *tx_handle)
{
    // 配置 I2S 通道为 Master 模式
    i2s_chan_config_t i2s_chan_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    // 启用回调后自动清除缓冲区，防止 DMA 循环发送最后一帧数据
    i2s_chan_config.auto_clear_after_cb = true;

    // 创建 I2S 收发通道
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_chan_config, tx_handle, rx_handle));
    i2s_std_config_t std_config = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(BSP_CODEC_SAMPLE_RATE),
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(BSP_CODEC_BITS_PER_SAMPLE, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = BSP_CODEC_MCLK_PIN,
            .bclk = BSP_CODEC_BCLK_PIN,
            .ws = BSP_CODEC_WS_PIN,
            .dout = BSP_CODEC_DOUT_PIN,
            .din = BSP_CODEC_DIN_PIN,
        },
    };

    // 初始化 I2S 为标准模式并启用通道
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_handle, &std_config));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_handle, &std_config));
    ESP_ERROR_CHECK(i2s_channel_enable(*rx_handle));
    ESP_ERROR_CHECK(i2s_channel_enable(*tx_handle));
}

// ==================== 公开 BSP 初始化函数 ====================

void bsp_board_codec_init(bsp_board_t *bsp_board)
{
    // 1. 创建 I2C 控制接口
    i2c_master_bus_handle_t bus_handle = NULL;
    bsp_board_codec_i2c_init(bsp_board, &bus_handle);

    audio_codec_i2c_cfg_t i2c_cfg = {
        .bus_handle = bus_handle,
        .addr = ES8311_CODEC_DEFAULT_ADDR,
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

    // 2. 创建 GPIO 控制接口
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    // 3. 配置 ES8311 编解码器
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = -1,                               // PA 使能直接连接电源，默认拉高
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH, // 双工模式（录音 + 播放）
        .use_mclk = true,
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);

    // ========== 4. 创建 I2S 数据接口 ==========
    i2s_chan_handle_t rx_handle = NULL, tx_handle = NULL;
    bsp_board_codec_i2s_init(bsp_board, &rx_handle, &tx_handle);
    audio_codec_i2s_cfg_t i2s_config = {
        .rx_handle = rx_handle,
        .tx_handle = tx_handle,
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_config);

    // 5. 创建音频设备句柄（存入 bsp_board）
    esp_codec_dev_cfg_t codec_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT,
        .codec_if = codec_if,
        .data_if = data_if,
    };
    bsp_board->codec_dev = esp_codec_dev_new(&codec_config);
    assert(bsp_board->codec_dev);

    // 设置 Codec 初始化完成标志
    xEventGroupSetBits(bsp_board->board_status, CODEC_BIT);
}

bool bsp_board_check_status(bsp_board_t *bsp_board, EventBits_t bits_to_check, TickType_t wait_ticks)
{
    EventBits_t bits = xEventGroupWaitBits(bsp_board->board_status, bits_to_check, pdFALSE, pdTRUE, wait_ticks);
    return (bits & bits_to_check) == bits_to_check;
}

// ==================== 音频采集任务（原 audio.c）====================

// 录音投喂任务：持续从麦克风读取 PCM 数据，投喂给唤醒词引擎
void audio_feed_task(void *arg)
{
    bsp_board_t *bsp_board = (bsp_board_t *)arg;

    size_t chunk_size = custom_wake_word_get_chunksize();
    if (chunk_size == 0)
        chunk_size = 512;

    int16_t *buffer = malloc(chunk_size * sizeof(int16_t));
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "audio_feed_task: 内存不足，无法分配音频缓冲区");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "正在持续监听环境声音...");

    while (1)
    {
        esp_err_t ret = esp_codec_dev_read(bsp_board->codec_dev, buffer, chunk_size * sizeof(int16_t));
        if (ret == ESP_OK)
        {
            custom_wake_word_feed(buffer, chunk_size);
        }
        else
        {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// 音频完整初始化：硬件初始化 + 打开设备 + 创建采集任务
void audio_init(bsp_board_t *bsp_board)
{
    ESP_LOGI(TAG, "正在初始化 ES8311 音频...");

    // 1. 初始化硬件（I2C + I2S + ES8311），设置 CODEC_BIT
    bsp_board_codec_init(bsp_board);

    // 2. 打开音频设备，配置采样参数
    esp_codec_dev_sample_info_t sample_info = {
        .sample_rate = BSP_CODEC_SAMPLE_RATE,
        .bits_per_sample = BSP_CODEC_BITS_PER_SAMPLE,
        .channel = 1,
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(bsp_board->codec_dev, &sample_info));

    // 3. 设置麦克风增益和喇叭音量
    esp_codec_dev_set_in_gain(bsp_board->codec_dev, 40);
    esp_codec_dev_set_out_vol(bsp_board->codec_dev, 70);

    ESP_LOGI(TAG, "ES8311 初始化完成！");

    // 4. 所有硬件就绪后才创建任务，避免竞态条件（原 P0 隐患）
    xTaskCreatePinnedToCore(audio_feed_task, "audio_feed", 8192, bsp_board, 5, NULL, 1);
}
