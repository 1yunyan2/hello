#include "bsp_board.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "wake_word/custom_wake_word.h"

static const char *TAG = "BSP_CODEC";

// 外部模块可注册此钩子，在会话期间持续接收原始 PCM（16-bit 单声道）
static void (*s_pcm_hook)(const int16_t *data, size_t samples) = NULL;

void audio_set_pcm_hook(void (*hook)(const int16_t *data, size_t samples))
{
    s_pcm_hook = hook;
}

// ==================== 私有硬件初始化函数 ====================

static void bsp_board_codec_i2c_init(bsp_board_t *bsp_board, i2c_master_bus_handle_t *bus_handle)
{
    // 配置 I2C 总线参数（ES8311 控制接口）
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,                // 使用 I2C 端口 0
        .sda_io_num = BSP_CODEC_SDA_PIN,      // SDA 数据线引脚
        .scl_io_num = BSP_CODEC_SCL_PIN,      // SCL 时钟线引脚
        .clk_source = I2C_CLK_SRC_DEFAULT,    // 使用默认时钟源
        .glitch_ignore_cnt = 7,               // 毛刺滤波计数（过滤线路噪声）
        .flags.enable_internal_pullup = true, // 启用内部上拉（省去外部电阻）
    };
    // 创建 I2C 主机总线，句柄存入 bus_handle 供后续使用
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, bus_handle));
}

static void bsp_board_codec_i2s_init(bsp_board_t *bsp_board,
                                     i2s_chan_handle_t *rx_handle,
                                     i2s_chan_handle_t *tx_handle)
{
    // 创建 I2S 通道配置（Master 模式，设备编号 0）
    i2s_chan_config_t i2s_chan_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    // 启用回调后自动清零 DMA 缓冲区，防止上一帧数据被循环重放
    i2s_chan_config.auto_clear_after_cb = true;

    // 同时创建发送（TX）和接收（RX）通道，共用同一 I2S 控制器
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_chan_config, tx_handle, rx_handle));

    // 配置 I2S 标准（Philips）模式参数
    i2s_std_config_t std_config = {
        // 时钟：根据采样率自动计算 MCLK / BCLK 分频
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(BSP_CODEC_SAMPLE_RATE),
        // 槽位：Philips 格式，16-bit，单声道
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(BSP_CODEC_BITS_PER_SAMPLE, I2S_SLOT_MODE_MONO),
        // GPIO 引脚映射
        .gpio_cfg = {
            .mclk = BSP_CODEC_MCLK_PIN, // 主时钟（提供给 ES8311 作为参考时钟）
            .bclk = BSP_CODEC_BCLK_PIN, // 位时钟
            .ws = BSP_CODEC_WS_PIN,     // 字选择（左右声道同步）
            .dout = BSP_CODEC_DOUT_PIN, // 数据输出（ESP → ES8311 → 扬声器）
            .din = BSP_CODEC_DIN_PIN,   // 数据输入（麦克风 → ES8311 → ESP）
        },
    };

    // 将 RX / TX 通道分别初始化为标准 I2S 模式
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_handle, &std_config));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_handle, &std_config));

    // 使能 RX 通道（开始接收麦克风数据）
    ESP_ERROR_CHECK(i2s_channel_enable(*rx_handle));
    // 使能 TX 通道（开始发送播放数据）
    ESP_ERROR_CHECK(i2s_channel_enable(*tx_handle));
}

// ==================== 公开 BSP 初始化函数 ====================

void bsp_board_codec_init(bsp_board_t *bsp_board)
{
    // ── 步骤 1：创建 I2C 控制总线 ──────────────────────────────────────────
    i2c_master_bus_handle_t bus_handle = NULL;
    bsp_board_codec_i2c_init(bsp_board, &bus_handle);

    // 封装 I2C 句柄为 Codec 控制接口（用于向 ES8311 发送寄存器命令）
    audio_codec_i2c_cfg_t i2c_cfg = {
        .bus_handle = bus_handle,
        .addr = ES8311_CODEC_DEFAULT_ADDR, // ES8311 I2C 地址（默认 0x18）
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

    // ── 步骤 2：创建 GPIO 控制接口（PA 使能等 GPIO 操作的抽象层）──────────
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    // ── 步骤 3：配置并创建 ES8311 Codec 驱动实例 ──────────────────────────
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,
        .gpio_if = gpio_if,
        .pa_pin = -1,                               // PA 使能引脚（-1 = 不使用，PA 直接连电源）
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH, // 全双工：同时支持录音和播放
        .use_mclk = true,                           // 使用 MCLK 作为 ES8311 时钟参考
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);

    // ── 步骤 4：创建 I2S 数据通道 ──────────────────────────────────────────
    i2s_chan_handle_t rx_handle = NULL, tx_handle = NULL;
    bsp_board_codec_i2s_init(bsp_board, &rx_handle, &tx_handle);

    // 封装 I2S RX/TX 句柄为 Codec 数据接口
    audio_codec_i2s_cfg_t i2s_config = {
        .rx_handle = rx_handle, // 接收（录音）通道
        .tx_handle = tx_handle, // 发送（播放）通道
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_config);

    // ── 步骤 5：创建顶层音频设备句柄，存入 bsp_board 供其他模块使用 ────────
    esp_codec_dev_cfg_t codec_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT, // 同时支持输入（麦克风）和输出（扬声器）
        .codec_if = codec_if,                  // Codec 控制接口（I2C 寄存器操作）
        .data_if = data_if,                    // Codec 数据接口（I2S 音频流）
    };
    bsp_board->codec_dev = esp_codec_dev_new(&codec_config);

    // 强制断言：codec_dev 必须创建成功，否则后续录音/播放无法进行
    assert(bsp_board->codec_dev);

    // 置位 CODEC_BIT，通知其他等待模块 Codec 硬件已就绪
    xEventGroupSetBits(bsp_board->board_status, CODEC_BIT);
}

bool bsp_board_check_status(bsp_board_t *bsp_board, EventBits_t bits_to_check, TickType_t wait_ticks)
{
    // 等待指定的状态位全部置位（pdTRUE = AND 等待）
    EventBits_t bits = xEventGroupWaitBits(
        bsp_board->board_status,
        bits_to_check, // 需要检查的位掩码
        pdFALSE,       // 返回时不清除位
        pdTRUE,        // 所有位都满足才返回（AND 模式）
        wait_ticks);   // 超时时间（0 = 立即返回）

    // 判断所有请求的位是否均已置位
    return (bits & bits_to_check) == bits_to_check;
}

// ==================== 音频采集任务 ====================

// 录音投喂任务：持续从麦克风读取 PCM 数据，投喂给唤醒词引擎
void audio_feed_task(void *arg)
{
    bsp_board_t *bsp_board = (bsp_board_t *)arg;

    // 获取唤醒词引擎每次需要的采样点数（一般为 512）
    size_t chunk_size = custom_wake_word_get_chunksize();
    if (chunk_size == 0)
        chunk_size = 512; // 引擎未就绪时使用安全默认值

    // 分配音频读取缓冲区（16-bit PCM，单声道）
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
        // 从 Codec 设备读取一帧 PCM 数据（阻塞直到数据就绪）
        esp_err_t ret = esp_codec_dev_read(bsp_board->codec_dev, buffer, chunk_size * sizeof(int16_t));
        if (ret == ESP_OK)
        {
            // 将读取到的 PCM 帧喂给唤醒词引擎进行识别
            custom_wake_word_feed(buffer, chunk_size);
            // 若会话模块已注册钩子（唤醒后），同步转发给编码器
            if (s_pcm_hook)
                s_pcm_hook(buffer, chunk_size);
        }
        else
        {
            // 读取失败（如 DMA 未就绪），延迟 10ms 后重试
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
}

// 音频完整初始化：硬件初始化 + 打开设备 + 创建采集任务
void audio_init(bsp_board_t *bsp_board)
{
    ESP_LOGI(TAG, "正在初始化 ES8311 音频...");

    // 步骤 1：初始化 I2C + I2S + ES8311 硬件，置位 CODEC_BIT
    bsp_board_codec_init(bsp_board);

    // 步骤 2：打开音频设备，配置采样参数（采样率/位深/声道）
    esp_codec_dev_sample_info_t sample_info = {
        .sample_rate = BSP_CODEC_SAMPLE_RATE,         // 16000 Hz
        .bits_per_sample = BSP_CODEC_BITS_PER_SAMPLE, // 16-bit
        .channel = 1,                                 // 单声道
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(bsp_board->codec_dev, &sample_info));

    // 步骤 3：设置麦克风增益（40 = ~20dB，适合近讲场景）
    esp_codec_dev_set_in_gain(bsp_board->codec_dev, 40);

    // 步骤 4：设置扬声器音量（0~100，70 为适中音量）
    esp_codec_dev_set_out_vol(bsp_board->codec_dev, 70);

    ESP_LOGI(TAG, "ES8311 初始化完成！");

    // 步骤 5：所有硬件就绪后创建音频采集任务（固定到 CPU 核心 1，避免与 WiFi 竞争）
    // 注意：必须在 codec_dev 完全打开后才创建任务，否则 read 会失败
    xTaskCreatePinnedToCore(audio_feed_task, "audio_feed",
                            8192,      // 栈大小（含 DMA 缓冲区，不可太小）
                            bsp_board, // 传入 bsp_board 指针供任务使用
                            5,         // 优先级（高于普通任务，保证实时性）
                            NULL,      // 不需要保存任务句柄
                            1);        // 绑定到 CPU 核心 1
}
