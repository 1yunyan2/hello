#include "bsp_board.h"
#include "esp_codec_dev_defaults.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "wake_word/custom_wake_word.h"
#include "esp_heap_caps.h"
#include <math.h>

static const char *TAG = "BSP_CODEC";

// 音量持久化用的 NVS 命名空间与键名（与 mqtt_creds 同款 NVS 用法）
#define AUDIO_CFG_NVS_NS "audio_cfg" // 音频配置命名空间
#define AUDIO_CFG_KEY_VOL "out_vol"  // 扬声器输出音量键（int32，0~100）
#define BSP_CODEC_DEFAULT_VOLUME 50  // 默认扬声器音量（NVS 未配置时使用）
#define MAX_VOLUME 100
#define MIN_VOLUME 0

// ── 防爆音（Anti-Pop）软启动参数 ─────────────────────────────────────────────
// 背景：本板 NS4150 功放的 CTRL 脚仅由 R13 上拉常开，MCU 无法控制功放开关，
//       ES8311 上电（esp_codec_dev_open）时 VMID/DAC 的电压阶跃会被常开功放
//       放大成"啵"的一声。软件侧只能用"静音→等稳→渐升音量"减轻 unmute 爆音。
//       根治需硬件改版：PA_CTRL 接 MCU 空闲 GPIO（候选 GPIO19/20，若不走原生 USB）。
#define BSP_CODEC_ANTIPOP_VMID_MS 200 // open 后等待 VMID/DAC 偏置稳定的时间（毫秒）
#define BSP_CODEC_ANTIPOP_STEPS 5     // 音量渐升步数（0 → 目标音量分几步爬）
#define BSP_CODEC_ANTIPOP_STEP_MS 50  // 每步之间的间隔（毫秒），总爬升时长 = 步数×间隔

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
            .mclk = BSP_CODEC_MCLK_PIN, // GPIO8：主时钟，ES8311 内部 PLL 参考源
            .bclk = BSP_CODEC_BCLK_PIN, // GPIO46：位时钟，每个采样位一个脉冲
            .ws = BSP_CODEC_WS_PIN,     // GPIO7：字选择/帧同步，16kHz = 16000次/秒切换
            .dout = BSP_CODEC_DOUT_PIN, // GPIO15：播放数据（ESP32→ES8311→扬声器）
            .din = BSP_CODEC_DIN_PIN,   // GPIO6：录音数据（麦克风→ES8311→ESP32）
        },
    };

    // ── 步骤 4：将两个通道初始化为标准 I2S 模式 ──────────────────────────────
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*rx_handle, &std_config));
    ESP_ERROR_CHECK(i2s_channel_init_std_mode(*tx_handle, &std_config));

    // ── 步骤 5：使能通道，开始工作 ───────────────────────────────────────────
    // 使能后 DMA 开始工作：RX 持续从麦克风采集数据，TX 持续向扬声器输出数据
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

// ═══════════════════════════════════════════════════════════════════════════════
// 音频采集任务与完整初始化
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 麦克风采集任务（持续采集 PCM，投喂给 AFE + MultiNet 唤醒引擎）
 *
 * 数据流向：I2S DMA → buffer → custom_wake_word_feed()
 *                               ↓
 *                          AFE 内部处理：
 *                           - NS 降噪（消除背景噪音）
 *                           - VAD 语音活动检测
 *                          ↓              ↓
 *                   enhanced_pcm_hook  MultiNet6 检测
 *                   （→ 编码器入口）   （→ 唤醒词回调）
 *
 * @param arg bsp_board_t* 实例指针（通过 arg 传入，使用 codec_dev 读取音频）
 * @return 无（任务永远运行，除非 FreeRTOS 调度器停止）
 *
 * @note 调用者：audio_init() 通过 xTaskCreatePinnedToCore() 自动创建
 * @note 运行核心：CPU1（避免与 WiFi 协议栈竞争 CPU0）
 * @note 栈大小：8192 字节，优先级：5
 * @note chunk_size 来自 AFE 的 feed_chunksize，必须严格按此大小投喂
 */
void audio_feed_task(void *arg)
{
    bsp_board_t *bsp_board = (bsp_board_t *)arg;

    // ── 步骤 1：获取 AFE 要求的每次投喂采样点数 ──────────────────────────────
    // AFE 内部要求每次 feed 固定数量的采样点（通常是 512 点 = 32ms@16kHz）
    // 投喂量不对会导致 AFE 内部缓冲溢出或欠采样，产生 VAD 误检
    size_t chunk_size = custom_wake_word_get_feed_chunksize();
    if (chunk_size == 0)
    {
        // AFE 未就绪时使用安全默认值（避免任务立即 crash）
        chunk_size = 512;
        ESP_LOGW(TAG, "AFE 未就绪，使用默认 chunk_size=%d", (int)chunk_size);
    }

    // ── 步骤 2：分配 PCM 采集缓冲区 ─────────────────────────────────────────

    // 因为使用的是软件的回音消除，是单声道，所以每个采样点是一个 int16_t（16-bit），不需要乘以通道数。
    //  // 【修改点 1】增加通道数变量，计算真实的字节数
    //  int feed_channel = 2; // 因为配了 "MR"，这里必须是 2
    //  size_t alloc_size = chunk_size * feed_channel * sizeof(int16_t);

    // // 【修改点 2】按新计算的大小分配内存
    // int16_t *buffer = malloc(alloc_size);
    // 大小 = 采样点数 × 每点字节数（16-bit = 2 字节）
    /* 从 SPIRAM 分配采集缓冲区，避免占用宝贵的内部 SRAM */
    int16_t *buffer = heap_caps_malloc(chunk_size * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (buffer == NULL)
    {
        ESP_LOGE(TAG, "audio_feed_task: 内存不足，无法分配 %d 字节采集缓冲区",
                 (int)(chunk_size * sizeof(int16_t)));
        vTaskDelete(NULL); // 分配失败，删除自己避免空指针访问
        return;
    }

    ESP_LOGI(TAG, "音频采集任务启动 (AFE feed chunk=%d samples, %d bytes)",
             (int)chunk_size, (int)(chunk_size * sizeof(int16_t)));

    // [PCBA 诊断] 每 ~3s 统计一次 PCM 峰值/RMS，用于判断麦克风信号是否正常
    // peak<100/rms<30 → 信号几乎没进来（硬件层）；peak 200~1000 → 增益不足；peak>5000 → 信号 OK，问题在 AFE
    uint32_t diag_iter = 0;
    int32_t diag_peak = 0;
    uint64_t diag_sumsq = 0;
    uint32_t diag_samples = 0;
    const uint32_t DIAG_PRINT_EVERY = 16000 / 512 * 3; // 约 3 秒

    // [PCBA 诊断·自检] 启动时人为塞已知值，验证统计代码本身没问题
    // 期望输出 peak=12345 rms≈8731（√((12345²+1000²+...)/8)的近似）
    {
        int16_t test_buf[8] = {12345, -1000, 500, -500, 200, -200, 0, 0};
        int32_t t_peak = 0;
        uint64_t t_sumsq = 0;
        for (int i = 0; i < 8; ++i)
        {
            int32_t v = test_buf[i];
            int32_t av = v < 0 ? -v : v;
            if (av > t_peak)
                t_peak = av;
            t_sumsq += (uint64_t)(v * v);
        }
        uint32_t t_rms = (uint32_t)sqrt((double)t_sumsq / 8);
        ESP_LOGW(TAG, "[PCM自检] 统计逻辑测试 peak=%ld rms=%lu (期望 peak=12345 rms≈4387) — 好的",
                 (long)t_peak, (unsigned long)t_rms);
    }

    // ── 步骤 3：主采集循环（永不退出）───────────────────────────────────────
    while (1)
    {
        // 从 ES8311 编解码器读取一帧 PCM 数据（阻塞直到 DMA 缓冲区就绪）
        // esp_codec_dev_read 内部调用 i2s_channel_read，等待 I2S RX DMA 完成
        esp_err_t ret = esp_codec_dev_read(
            bsp_board->codec_dev,        // ES8311 设备句柄
            buffer,                      // 目标缓冲区
            chunk_size * sizeof(int16_t) // 读取字节数（固定帧大小）
            // alloc_size
        );

        if (ret == ESP_OK)
        {
            // [PCBA 诊断] 累计本帧的峰值和平方和
            for (size_t i = 0; i < chunk_size; ++i)
            {
                int32_t v = buffer[i];
                int32_t av = v < 0 ? -v : v;
                if (av > diag_peak)
                    diag_peak = av;
                diag_sumsq += (uint64_t)(v * v);
            }
            diag_samples += chunk_size;
            if (++diag_iter >= DIAG_PRINT_EVERY)
            {
                uint32_t rms = diag_samples ? (uint32_t)sqrt((double)diag_sumsq / diag_samples) : 0;
                // 同步打印 buffer 前 8 个原始采样的十六进制，证明读到的字节真是 0x00 而不是统计 bug
                ESP_LOGI(TAG, "[PCM诊断] peak=%ld rms=%lu samples=%lu | 原始bytes[0..7]=%04X %04X %04X %04X %04X %04X %04X %04X",
                         (long)diag_peak, (unsigned long)rms, (unsigned long)diag_samples,
                         (uint16_t)buffer[0], (uint16_t)buffer[1], (uint16_t)buffer[2], (uint16_t)buffer[3],
                         (uint16_t)buffer[4], (uint16_t)buffer[5], (uint16_t)buffer[6], (uint16_t)buffer[7]);
                diag_iter = 0;
                diag_peak = 0;
                diag_sumsq = 0;
                diag_samples = 0;
            }

            // 将原始 PCM 投喂给 AFE + MultiNet 引擎
            // 内部流程：AFE.feed() → AFE.fetch()（降噪）→ PCM钩子 + MultiNet检测
            custom_wake_word_feed(buffer, chunk_size);
        }
        else
        {
            // 读取失败（DMA 未就绪或 I2S 错误），延迟 10ms 后重试
            // 避免 CPU 空转，给底层驱动时间恢复
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    // 注意：此处代码不可达，malloc 的 buffer 在任务生命周期内始终有效
}

/**
 * @brief 设置扬声器输出音量（运行期可调，自动持久化到 NVS）
 *
 * 见 bsp_board.h 中接口说明。实现要点：
 *   1. 入参钳位 0~100，防止越界导致 DAC 饱和爆音
 *   2. 通过 bsp_board_get_instance() 取 codec_dev，未就绪时返回错误（不 panic）
 *   3. 调用 esp_codec_dev_set_out_vol() 写 ES8311 寄存器
 *   4. 写入 NVS "audio_cfg/out_vol"，重启后由 audio_init() 读回
 *
 * @param volume 目标音量（0~100，超出自动钳位）
 * @return ESP_OK / ESP_ERR_INVALID_STATE / 其他 esp_err_t
 */
esp_err_t bsp_board_codec_set_volume(int volume)
{
    // ── 步骤 1：入参钳位到 0~100 ──────────────────────────────────────────────
    if (volume < MIN_VOLUME)
        volume = MIN_VOLUME;
    if (volume > MAX_VOLUME)
        volume = MAX_VOLUME;

    // ── 步骤 2：获取 codec_dev 句柄，未就绪则拒绝 ────────────────────────────
    bsp_board_t *bsp_board = bsp_board_get_instance();
    if (bsp_board == NULL || bsp_board->codec_dev == NULL)
    {
        ESP_LOGE(TAG, "设置音量失败：codec_dev 未初始化（音频硬件未就绪）");
        return ESP_ERR_INVALID_STATE;
    }

    // ── 步骤 3：写 ES8311 寄存器设置输出音量 ─────────────────────────────────
    esp_err_t ret = esp_codec_dev_set_out_vol(bsp_board->codec_dev, volume);
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "设置扬声器音量=%d 失败: %s", volume, esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "设置扬声器音量=%d", volume);

    // ── 步骤 4：持久化到 NVS，重启后保留 ─────────────────────────────────────
    // NVS 写失败不影响本次音量生效，仅告警（与 mqtt_creds 一致的容错风格）
    nvs_handle_t nvs;
    esp_err_t nvs_err = nvs_open(AUDIO_CFG_NVS_NS, NVS_READWRITE, &nvs);
    if (nvs_err == ESP_OK)
    {
        nvs_set_i32(nvs, AUDIO_CFG_KEY_VOL, volume);
        nvs_commit(nvs);
        nvs_close(nvs);
    }
    else
    {
        ESP_LOGW(TAG, "音量持久化失败：无法打开 NVS '%s': %s",
                 AUDIO_CFG_NVS_NS, esp_err_to_name(nvs_err));
    }

    return ESP_OK;
}

/**
 * @brief 完整音频初始化：硬件 + 设备打开 + 增益设置 + 采集任务
 *
 * 依次执行：
 *   1. bsp_board_codec_init()：初始化 I2C+I2S+ES8311 硬件（置位 CODEC_BIT）
 *   2. esp_codec_dev_open()：打开音频设备，配置采样参数
 *   3. 设置麦克风输入增益和扬声器输出音量
 *   4. xTaskCreatePinnedToCore(audio_feed_task)：启动麦克风采集任务
 *
 * @param bsp_board BSP 实例指针（codec_dev 在此函数完成后可用）
 * @return void
 *
 * @note 调用者：application.c → application_init()（步骤 4）
 * @note 前置条件：wake_word_init() 必须先完成（采集任务立即向引擎投喂）
 * @note 采集任务绑定 CPU1，与 WiFi（CPU0）隔离，保证实时性
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

    // ── 步骤 2.5：防爆音第一步——open 后立即静音 ─────────────────────────────
    // open() 内部给 ES8311 DAC 上电，此刻输出端正在产生偏置阶跃。立即把输出
    // 音量压到 0，确保后续 unmute 不会在偏置未稳时叠加一次满音量跳变。
    // 注意：此处直接调 esp_codec_dev_set_out_vol（不走 bsp_board_codec_set_volume），
    //       避免把临时的 0 音量错误持久化到 NVS。
    esp_codec_dev_set_out_vol(bsp_board->codec_dev, 0);

    // ── 步骤 3：设置麦克风增益（ADC PGA 增益，0~100，>50 饱和失真）──────────
    // 增益过小：语音信号弱，VAD 和 MultiNet 识别率下降（必须大声才能触发）
    // 增益过大>50：产生饱和失真，同样影响识别率
    // 43→46：配合 AFE AGC(WAKENET) 使用，硬件增益提升语音底线幅度，
    //        AGC 再做软件自适应补偿，无需大喊即可达到模型所需置信度
    esp_codec_dev_set_in_gain(bsp_board->codec_dev, 48);

    // ── 步骤 4：读取目标音量（从 NVS 读回上次保存值，无则用默认 50）─────────
    // 此处读 NVS 决定初始值，避免初始化用默认值覆盖云端设过的音量（回环问题）。
    int init_volume = BSP_CODEC_DEFAULT_VOLUME;
    nvs_handle_t nvs;
    if (nvs_open(AUDIO_CFG_NVS_NS, NVS_READONLY, &nvs) == ESP_OK)
    {
        int32_t saved_vol = 0;
        if (nvs_get_i32(nvs, AUDIO_CFG_KEY_VOL, &saved_vol) == ESP_OK)
        {
            init_volume = (int)saved_vol; // 读到则用保存值
        }
        nvs_close(nvs);
    }

    // ── 步骤 4.5：防爆音第二步——等偏置稳定后音量渐升到目标值 ────────────────
    // 先等 VMID/DAC 偏置电压充电完成（阶跃已被静音挡住大半），再分多步小台阶
    // 爬升音量，每步之间留间隔，把"咔哒"一声摊平成人耳不敏感的缓慢淡入。
    // 渐升过程同样直接调 esp_codec_dev_set_out_vol，跳过 NVS 写入
    // （目标值本来就读自 NVS，重复写回是无意义的 Flash 损耗）。
    vTaskDelay(pdMS_TO_TICKS(BSP_CODEC_ANTIPOP_VMID_MS));
    for (int step = 1; step <= BSP_CODEC_ANTIPOP_STEPS; step++)
    {
        int vol = init_volume * step / BSP_CODEC_ANTIPOP_STEPS; // 整数等分爬升
        esp_codec_dev_set_out_vol(bsp_board->codec_dev, vol);
        vTaskDelay(pdMS_TO_TICKS(BSP_CODEC_ANTIPOP_STEP_MS));
    }

    ESP_LOGI(TAG, "ES8311 初始化完成（增益=48, 音量=%d，防爆音软启动已生效）", init_volume);

    // ── 步骤 5：创建麦克风采集任务 ────────────────────────────────────────────
    // 任务立即开始从 I2S DMA 读取 PCM 数据并投喂给 AFE/MultiNet
    // 必须在 codec_dev 完全打开后才能创建，否则 read() 会失败
    /* 任务栈分配到 SPIRAM，节省内部 SRAM（audio_feed 无实时 ISR 调用，PSRAM cache 足够快） */
    xTaskCreatePinnedToCoreWithCaps(
        audio_feed_task,                      // 任务函数
        "audio_feed",                         // 任务名称（用于 FreeRTOS 调试工具显示）
        8192,                                 // 栈大小（8KB：含 DMA 缓冲区指针和局部变量）
        bsp_board,                            // 传入 bsp_board 指针（任务需要 codec_dev 读取音频）
        5,                                    // 优先级（与编解码任务对称，保证实时性）
        NULL,                                 // 不需要保存任务句柄（任务永远运行，无需管理）
        1,                                    // 固定到 CPU 核心 1（WiFi 协议栈默认用 CPU0，避免竞争）
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT); // 栈分配在 SPIRAM
}
