#include "bsp_board.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"

static const char *TAG = "BSP_CODEC";

// ═══════════════════════════════════════════════════════════════════════════════
// 私有硬件初始化函数（仅在本文件内使用，外部不可见）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 创建 I2C 主机总线（ES8311 寄存器控制接口）
 *
 * ES8311 使用 I2C 接收来自 ESP32 的寄存器读写命令，用于配置：
 * 工作模式（全双工）、麦克风增益、ADC/DAC 参数等。
 * I2C 为低速控制总线，仅在初始化阶段使用，运行期间很少调用。
 *
 * @param bsp_board  BSP 实例指针（当前未使用，预留扩展用）
 * @param bus_handle 输出参数：创建成功的 I2C 主机总线句柄
 * @return void（失败时 ESP_ERROR_CHECK 触发系统重启）
 *
 * @note 调用者：bsp_board_codec_init()（内部私有调用）
 * @note 引脚：SDA=GPIO8，SCL=GPIO15（定义在 bsp_config.h）
 */
static void bsp_board_codec_i2c_init(bsp_board_t *bsp_board, i2c_master_bus_handle_t *bus_handle)
{
    // ── 配置 I2C 主机总线参数 ────────────────────────────────────────────────
    i2c_master_bus_config_t i2c_bus_config = {
        .i2c_port = I2C_NUM_0,                // 使用 I2C 控制器 0（ESP32-S3 共有 2 个）
        .sda_io_num = BSP_CODEC_SDA_PIN,      // SDA 数据线（GPIO8），双向数据传输
        .scl_io_num = BSP_CODEC_SCL_PIN,      // SCL 时钟线（GPIO15），单向时钟输出
        .clk_source = I2C_CLK_SRC_DEFAULT,    // 使用默认时钟源（APB 时钟，约 80MHz）
        .glitch_ignore_cnt = 7,               // 毛刺滤波：忽略 7 个时钟周期以内的干扰脉冲
        .flags.enable_internal_pullup = true, // 启用芯片内部上拉电阻，省去外部 4.7kΩ 上拉电阻
    };

    // ── 创建 I2C 主机总线，句柄输出到 bus_handle ────────────────────────────
    // 失败原因：引脚被其他外设占用，或 I2C 控制器已初始化
    ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_config, bus_handle));
}

/**
 * @brief 创建 I2S 双向通道（ES8311 音频数据接口）
 *
 * I2S（Inter-IC Sound）是专用音频总线，负责传输 PCM 原始音频数据。
 * 与 I2C 分工：I2C 负责控制（配置寄存器），I2S 负责高速数据流传输。
 * 同时创建 TX（播放）和 RX（录音）通道，实现全双工音频收发。
 *
 * @param bsp_board  BSP 实例指针（当前未使用，预留扩展用）
 * @param rx_handle  输出参数：创建成功的 I2S 接收通道句柄（麦克风→ESP32）
 * @param tx_handle  输出参数：创建成功的 I2S 发送通道句柄（ESP32→扬声器）
 * @return void（失败时 ESP_ERROR_CHECK 触发系统重启）
 *
 * @note 调用者：bsp_board_codec_init()（内部私有调用）
 * @note 引脚：MCLK=GPIO17, BCLK=GPIO9, WS=GPIO5, DIN=GPIO4, DOUT=GPIO6
 * @note 参数：16kHz / 16-bit / 单声道 Philips 标准格式
 */
static void bsp_board_codec_i2s_init(bsp_board_t *bsp_board,
                                     i2s_chan_handle_t *rx_handle,
                                     i2s_chan_handle_t *tx_handle)
{
    // ── 步骤 1：配置 I2S 通道基础参数（Master 模式）───────────────────────────
    // Master 模式：ESP32 提供 BCLK 和 WS 时钟，ES8311 作为 Slave 跟随时钟
    i2s_chan_config_t i2s_chan_config = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);

    // 启用回调后自动清零 DMA 缓冲区：防止上一帧数据在缓冲区循环时被重复播放
    // 对于语音对话场景特别重要：避免播放完毕后出现杂音"尾巴"
    i2s_chan_config.auto_clear_after_cb = true;

    // ── 步骤 2：同时创建 TX（发送）和 RX（接收）通道 ─────────────────────────
    // 两个通道共享同一 I2S 控制器（I2S_NUM_0），时序严格同步
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_chan_config, tx_handle, rx_handle));

    // ── 步骤 3：配置 I2S 标准（Philips 格式）音频参数 ───────────────────────
    i2s_std_config_t std_config = {
        // 时钟配置：根据采样率 16kHz 自动计算 MCLK/BCLK 分频比
        // MCLK = 采样率 × 256 = 4.096 MHz（ES8311 需要的 Master Clock）
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(BSP_CODEC_SAMPLE_RATE),

        // 槽位配置：Philips 格式（数据延迟 1 个 BCLK），16-bit，单声道
        // 单声道：左右声道数据相同，减少数据量，适合麦克风和单扬声器
        .slot_cfg = I2S_STD_PHILIP_SLOT_DEFAULT_CONFIG(
            BSP_CODEC_BITS_PER_SAMPLE, I2S_SLOT_MODE_MONO),

        // GPIO 引脚映射（对应 bsp_config.h 中的引脚定义）
        .gpio_cfg = {
            .mclk = BSP_CODEC_MCLK_PIN, // GPIO17：主时钟，ES8311 内部 PLL 参考源
            .bclk = BSP_CODEC_BCLK_PIN, // GPIO9：位时钟，每个采样位一个脉冲
            .ws = BSP_CODEC_WS_PIN,     // GPIO5：字选择/帧同步，16kHz = 16000次/秒切换
            .dout = BSP_CODEC_DOUT_PIN, // GPIO6：播放数据（ESP32→ES8311→扬声器）
            .din = BSP_CODEC_DIN_PIN,   // GPIO4：录音数据（麦克风→ES8311→ESP32）
        },
    };

    // ── 步骤 4：将两个通道初始化为标准 I2S 模式 ──────────────────────────────
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_handle, &std_config));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_handle, &std_config));

    // ── 步骤 5：使能通道，开始工作 ───────────────────────────────────────────
    // 使能后 DMA 开始工作：RX 持续从麦克风采集数据，TX 持续向扬声器输出数据。
    // 【必须在这里 enable】：本版 esp_codec_dev 在 open/close 时只做 disable，
    // 不会主动 enable，它假定通道交给它之前已处于 enabled 状态。若不在此 enable，
    // codec_dev 一上来就 disable 会报 "the channel has not been enabled yet"。
    // （注意：采集任务务必只有一处，否则多任务抢 read 锁会破坏 enable/disable 状态机）
    ESP_ERROR_CHECK(i2s_channel_enable(*rx_handle)); // 录音通道：开始采集麦克风数据
    ESP_ERROR_CHECK(i2s_channel_enable(*tx_handle)); // 播放通道：开始向扬声器输出
}

// ═══════════════════════════════════════════════════════════════════════════════
// 公开 BSP 初始化函数
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief ES8311 音频编解码器完整初始化（I2C + I2S + Codec 驱动 + 设备句柄）
 *
 * 整合 I2C 控制接口、I2S 数据接口和 ES8311 Codec 驱动，创建顶层音频设备句柄。
 * 使用 esp_codec_dev 中间件层统一管理，上层只需通过 esp_codec_dev_read/write
 * 操作音频数据，无需直接操作 I2C/I2S 底层 API。
 *
 * @param bsp_board BSP 实例指针
 *                  - 输入：board_status（置位 CODEC_BIT 用）
 *                  - 输出：codec_dev 字段由此函数填充，后续音频读写依赖此句柄
 * @return void（任一步骤失败时 ESP_ERROR_CHECK 触发系统重启）
 *
 * @note 调用者：bsp_codec.c → audio_init()（内部调用）
 * @note 完成后置位 CODEC_BIT，通知其他模块音频硬件已就绪
 * @note codec_dev 全双工：同时支持录音（RX）和播放（TX）
 */
void bsp_board_codec_init(bsp_board_t *bsp_board)
{
    // ── 步骤 1：创建 I2C 控制总线（ES8311 寄存器读写接口）────────────────────
    i2c_master_bus_handle_t bus_handle = NULL;
    bsp_board_codec_i2c_init(bsp_board, &bus_handle);

    // [PCBA 诊断] 裸 I2C 回读 ES8311 chip ID 寄存器，判断 I2C 通信是否真实可靠
    // R0xFD 出厂值 = 0x83 (CHIP_ID1)；R0xFE 出厂值 = 0x11 (CHIP_ID2)
    // 若读回值正确 → I2C 干净，问题在模拟侧（MCLK/AVDD/VMID/MIC 焊接）
    // 若读失败或值错误 → I2C 工艺问题（R17/R18 上拉、SDA/SCL 焊点），需补焊或换电阻
    {
        i2c_device_config_t probe_cfg = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = ES8311_CODEC_DEFAULT_ADDR >> 1, // codec_dev 用的是 8-bit 形式
            .scl_speed_hz = 100000,
        };
        i2c_master_dev_handle_t probe_dev = NULL;
        if (i2c_master_bus_add_device(bus_handle, &probe_cfg, &probe_dev) == ESP_OK)
        {
            uint8_t reg_id1 = 0xFD, reg_id2 = 0xFE, reg_ver = 0xFF;
            uint8_t v_id1 = 0xAA, v_id2 = 0xAA, v_ver = 0xAA;
            esp_err_t r1 = i2c_master_transmit_receive(probe_dev, &reg_id1, 1, &v_id1, 1, 50);
            esp_err_t r2 = i2c_master_transmit_receive(probe_dev, &reg_id2, 1, &v_id2, 1, 50);
            esp_err_t r3 = i2c_master_transmit_receive(probe_dev, &reg_ver, 1, &v_ver, 1, 50);
            ESP_LOGW(TAG, "[I2C诊断] ES8311 chip_id R0xFD=0x%02X(应=0x83) R0xFE=0x%02X(应=0x11) R0xFF=0x%02X | err=%d/%d/%d",
                     v_id1, v_id2, v_ver, r1, r2, r3);
            if (v_id1 == 0x83 && v_id2 == 0x11)
            {
                ESP_LOGI(TAG, "[I2C诊断] ✓ I2C 通信正确 → 问题在模拟侧，去测 MCLK/AVDD/VMID");
            }
            else
            {
                ESP_LOGE(TAG, "[I2C诊断] ✗ I2C 数据不正确 → 检查 R17/R18 上拉电阻和 SDA/SCL 焊点");
            }
            i2c_master_bus_rm_device(probe_dev);
        }
        else
        {
            ESP_LOGE(TAG, "[I2C诊断] 添加探测设备失败，I2C 总线异常");
        }
    }

    // 将 I2C 总线句柄封装为 Codec 控制接口（统一抽象层）
    // ES8311_CODEC_DEFAULT_ADDR = 0x18（ES8311 固定 I2C 地址，ADDR 引脚接地）8-bit 左移形式 0x30
    // 而 ESP-IDF 新版 i2c_master 接口需要 7-bit 形式 0x18

    audio_codec_i2c_cfg_t i2c_cfg = {
        .bus_handle = bus_handle,
        .addr = ES8311_CODEC_DEFAULT_ADDR, // ES8311 I2C 设备地址 0x18
    };
    const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

    // ── 步骤 2：创建 GPIO 控制接口（功放 PA 使能等 GPIO 操作的抽象层）──────
    // 即使 PA 引脚未使用（pa_pin = -1），GPIO 接口也必须创建（框架要求）
    const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

    // ── 步骤 3：创建并配置 ES8311 Codec 驱动实例 ─────────────────────────────
    es8311_codec_cfg_t es8311_cfg = {
        .ctrl_if = ctrl_if,                         // I2C 控制接口（寄存器操作）
        .gpio_if = gpio_if,                         // GPIO 接口（PA 控制等）
        .pa_pin = -1,                               // 功放使能引脚：-1 = 未使用（PA 常开）
        .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH, // 全双工：同时支持录音和播放
        .use_mclk = true,                           // 使用 MCLK 作为 ES8311 内部 PLL 参考
    };
    const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);

    // ── 步骤 4：创建 I2S 数据通道（音频 PCM 数据传输接口）───────────────────
    i2s_chan_handle_t rx_handle = NULL, tx_handle = NULL;
    bsp_board_codec_i2s_init(bsp_board, &rx_handle, &tx_handle);

    // 保存 TX 句柄到 bsp_board，供 play_task 绕过 codec_dev mutex 直接写入
    // 背景：codec_dev(IN_OUT) read/write 共享同一把 mutex，play_task 持锁 ~64ms
    //       会导致 audio_feed_task 无法及时 read，AFE FEED ringbuffer 溢出
    bsp_board->i2s_tx_handle = tx_handle; //! 这一步不属于 Codec 初始化，但为了性能优化需要在这里保存 TX 句柄

    // 将 I2S TX/RX 句柄封装为 Codec 数据接口（统一抽象层）
    audio_codec_i2s_cfg_t i2s_config = {
        .rx_handle = rx_handle, // 接收通道：麦克风采集方向（INPUT）
        .tx_handle = tx_handle, // 发送通道：扬声器播放方向（OUTPUT）
    };
    const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_config);

    // ── 步骤 5：创建顶层音频设备句柄，挂载到 bsp_board ──────────────────────
    // 顶层句柄整合了控制接口（I2C）和数据接口（I2S），上层只需操作此句柄
    esp_codec_dev_cfg_t codec_config = {
        .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT, // 全双工：INPUT（麦克风）+ OUTPUT（扬声器）
        .codec_if = codec_if,                  // ES8311 Codec 控制接口
        .data_if = data_if,                    // I2S 数据接口
    };
    bsp_board->codec_dev = esp_codec_dev_new(&codec_config);

    // ── 步骤 6：检查 codec_dev 是否创建成功 ───────────────────────────────────
    if (bsp_board->codec_dev == NULL)
    {
        // 可能原因：ES8311 硬件未连接、I2C 地址错误、内存不足
        ESP_LOGE(TAG, "Codec 设备创建失败，请检查 ES8311 硬件连接（SDA=%d SCL=%d）",
                 BSP_CODEC_SDA_PIN, BSP_CODEC_SCL_PIN);
        return; // 不触发 panic，允许系统在无音频情况下继续运行（调试用）
    }

    // ── 步骤 7：置位 CODEC_BIT，通知其他模块音频硬件已就绪 ──────────────────
    xEventGroupSetBits(bsp_board->board_status, CODEC_BIT);
}

/**
 * @brief 完整音频初始化：硬件 + 设备打开 + 增益设置
 *
 * 依次执行：
 *   1. bsp_board_codec_init()：初始化 I2C+I2S+ES8311 硬件（置位 CODEC_BIT）
 *   2. esp_codec_dev_open()：打开音频设备，配置采样参数
 *   3. 设置麦克风输入增益和扬声器输出音量
 *
 * @param bsp_board BSP 实例指针（codec_dev 在此函数完成后可用）
 * @return void
 *
 * @note 调用者：main.c → app_main()（步骤 2）
 * @note 采集任务不在此创建，由 main.c 统一启动（避免抢 codec_dev 锁）
 */
void audio_init(bsp_board_t *bsp_board)
{
    ESP_LOGI(TAG, "正在初始化 ES8311 音频编解码器...");

    // ── 步骤 1：硬件初始化（I2C + I2S + ES8311 + codec_dev 句柄）─────────────
    bsp_board_codec_init(bsp_board);

    // ── 步骤 2：打开音频设备，配置采样参数 ────────────────────────────────────
    // open() 会向 ES8311 写入寄存器：配置 ADC/DAC 工作参数、PLL 分频等
    esp_codec_dev_sample_info_t sample_info = {
        .sample_rate = BSP_CODEC_SAMPLE_RATE,         // 16000 Hz
        .bits_per_sample = BSP_CODEC_BITS_PER_SAMPLE, // 16-bit
        .channel = 1,                                 // 单声道（节省带宽和内存）
    };
    ESP_ERROR_CHECK(esp_codec_dev_open(bsp_board->codec_dev, &sample_info));

    // ── 步骤 3：设置麦克风增益（ADC PGA 增益，0~100，>50 饱和失真）──────────
    // 增益过小：语音信号弱，VAD 和 MultiNet 识别率下降（必须大声才能触发）
    // 增益过大>50：产生饱和失真，同样影响识别率
    // 43→46：配合 AFE AGC(WAKENET) 使用，硬件增益提升语音底线幅度，
    //        AGC 再做软件自适应补偿，无需大喊即可达到模型所需置信度
    esp_codec_dev_set_in_gain(bsp_board->codec_dev, 24);

    // ── 步骤 4：设置扬声器音量（0~100，60 为适中音量）──────────────────────
    // 音量过大可能导致 ES8311 内部 DAC 饱和，产生爆音
    esp_codec_dev_set_out_vol(bsp_board->codec_dev, 50);

    ESP_LOGI(TAG, "ES8311 初始化完成（增益=24, 音量=50）");

    // ── 说明：本函数【不再】创建采集任务 ──────────────────────────────────────
    // 采集任务（读 codec_dev → 投喂 AFE → VAD）由 main.c 统一负责并启动。
    // 若在这里也启动一个采集任务，两个任务会同时 esp_codec_dev_read() 抢同一把
    // codec_dev mutex，导致读取时序错乱、I2S 报 "channel is not enabled"、
    // AFE ringbuffer empty。因此采集逻辑只能有一处，统一放在 main.c。
}
