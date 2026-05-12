#include "audio_encoder.h"
#include "esp_audio_enc.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_enc_reg.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#define TAG "[AP] Encoder"

// ─── 编码器任务配置宏 ─────────────────────────────────────────────────────────
#define AUDIO_ENCODER_TASK_CORE_ID 0        ///< 编码任务绑定 CPU0（CPU1 被 audio_feed_task 独占，编码与播放同核靠 vTaskDelay 让步）
#define AUDIO_ENCODER_TASK_STACK_SIZE 32768 ///< 栈大小 32KB（OPUS 编码运算需要较大栈，含 FFT 等中间状态）
#define AUDIO_ENCODER_TASK_PRIORITY 5       ///< 优先级 5（与解码器、播放任务对称）

/**
 * @brief 编码器内部结构体（对外不透明，通过 audio_encoder_t* 访问）
 *
 * 字段说明：
 *   input_buffer  : BYTEBUF 类型，PCM 数据可自由拼接（无帧边界约束）
 *   output_buffer : NOSPLIT 类型，OPUS 帧必须整帧写入（有帧边界约束）
 *   enc            : OPUS 编码器句柄（esp_audio_codec 框架）
 *   is_running     : 任务退出标志（false 时任务自删除）
 */
struct audio_encoder
{
    RingbufHandle_t input_buffer;      ///< 输入缓冲区：接收来自 AFE 降噪后的 PCM 数据
    RingbufHandle_t output_buffer;     ///< 输出缓冲区：存放编码完成的 OPUS 帧
    esp_audio_enc_handle_t enc;        ///< OPUS 编码器句柄（esp_audio_codec 框架管理）
    bool is_running;                   ///< 任务运行标志：false 时编码循环退出，任务自删除
    volatile TaskHandle_t task_handle; ///< 任务句柄，用于超时强制终止和确认退出
};

// ═══════════════════════════════════════════════════════════════════════════════
// 编码器任务（核心编码循环）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief OPUS 音频编码主任务（持续将 PCM 帧编码为 OPUS 帧）
 *
 * 数据流：
 *   [enc_input: BYTEBUF] → 分段读取凑够一帧 PCM → esp_audio_enc_process() → [enc_output: NOSPLIT]
 *
 * 关键设计细节：
 *   1. PCM 帧可能跨多次 xRingbufferReceiveUpTo 才能凑齐（BYTEBUF 分段读）
 *   2. OPUS 编码结果写入 NOSPLIT 缓冲区（整帧写入，下游一次性读取）
 *   3. 输出缓冲区满时丢帧（不阻塞），保证实时性不卡顿
 *
 * @param arg audio_encoder_t* 实例指针（类型转换后使用）
 * @return 无（任务运行直到 is_running 为 false）
 *
 * @note 调用者：audio_encoder_start() 通过 xTaskCreatePinnedToCoreWithCaps() 创建
 * @note 运行核心：CPU0，栈 32KB SPIRAM，优先级 5
 */
void audio_encoder_task(void *arg)
{
    audio_encoder_t *audio_encoder = (audio_encoder_t *)arg;

    // ── 步骤 1：查询每帧所需的 PCM 输入字节数和 OPUS 输出字节数 ──────────────
    // OPUS 20ms 帧 @ 16kHz 单声道 16-bit = 320采样点 × 2字节 = 640 字节 PCM
    // OPUS 输出大小由编码器内部决定（通常 20~100 字节，取决于音频内容和比特率）
    int in_frame_size = 0, out_frame_size = 0;
    esp_audio_enc_get_frame_size(audio_encoder->enc, &in_frame_size, &out_frame_size);

    // ── 步骤 2：分配输入和输出帧缓冲区 ──────────────────────────────────────
    void *in_buf = malloc_zeroed(in_frame_size); // PCM 帧缓冲（凑帧用）
    if (in_buf == NULL)
    {
        ESP_LOGE(TAG, "输入帧缓冲区分配失败（需要 %d 字节）", in_frame_size);
        vTaskDelete(NULL);
        return;
    }

    void *out_buf = malloc_zeroed(out_frame_size); // OPUS 帧缓冲（编码结果）
    if (out_buf == NULL)
    {
        ESP_LOGE(TAG, "输出帧缓冲区分配失败（需要 %d 字节）", out_frame_size);
        free(in_buf);
        vTaskDelete(NULL);
        return;
    }

    // ── 步骤 3：初始化帧描述符（描述输入/输出缓冲区的地址和大小）─────────────
    esp_audio_enc_in_frame_t in_frame = {
        .buffer = in_buf,     // PCM 数据源指针
        .len = in_frame_size, // 每帧 PCM 字节数（固定）
    };
    esp_audio_enc_out_frame_t out_frame = {
        .buffer = out_buf,     // OPUS 数据目标指针
        .len = out_frame_size, // 最大输出字节数（实际编码后由 encoded_bytes 给出）
    };

    // ── 步骤 4：编码主循环 ────────────────────────────────────────────────────
    // 凑帧期间的让步计数器：每读够 4 段（即可能多次 continue 才凑齐一帧）就主动让步 1 tick，
    // 防止 enc_input 堆积时 encoder_task 在凑帧 continue 路径上出现紧循环饿死 IDLE0。
    int accum_segments = 0;

    while (audio_encoder->is_running)
    {
        // 从输入缓冲区读取一段 PCM 数据（BYTEBUF 类型，可能不足一帧）
        // xRingbufferReceiveUpTo：最多读 in_frame_size 字节，100ms 超时避免死锁
        size_t size_read = 0;
        void *buf_read = xRingbufferReceiveUpTo(
            audio_encoder->input_buffer,
            &size_read,
            pdMS_TO_TICKS(100), // 超时 100ms：无数据时检查 is_running 标志
            in_frame_size);     // 最多读 in_frame_size 字节

        if (!buf_read)
            continue; // 超时无数据（静音期），继续等待下一帧

        // 将读到的 PCM 片段追加到帧缓冲区
        memcpy(in_buf, buf_read, size_read);

        // 归还环形缓冲区的使用权（BYTEBUF 读完必须立即归还，否则缓冲区会占满）
        vRingbufferReturnItem(audio_encoder->input_buffer, buf_read);

        // 扣减本帧剩余需要的字节数，移动写入指针
        in_frame_size -= size_read;
        in_buf += size_read;

        if (in_frame_size > 0)
        {
            // 当前帧数据尚未凑满，继续读取下一段
            // ★ 关键：累计读取段数，每 4 段（≈ 1 帧编码周期）主动让步，
            //   防止 enc_input 堆积时连续 continue 紧循环饿死 IDLE0/WDT。
            //   使用 taskYIELD() 而非 vTaskDelay(1)：同优先级任务间切换即可，
            //   避免强制睡眠 1 tick 拖慢凑帧速率。
            if (++accum_segments >= 4)
            {
                accum_segments = 0;
                taskYIELD(); // 主动让步，防止饿死 IDLE0/WDT
            }
            continue;
        }
        accum_segments = 0; // 凑齐一帧后重置段计数

        // ── 一帧 PCM 已凑齐（in_frame_size 减为 0），执行 OPUS 编码 ────────
        // 重置指针和剩余长度，准备接收下一帧
        in_buf = in_frame.buffer;
        in_frame_size = in_frame.len;

        // 调用 OPUS 编码器处理一帧（640 字节 PCM → 约 60 字节 OPUS）
        esp_audio_enc_process(audio_encoder->enc, &in_frame, &out_frame);

        // 将编码结果写入输出缓冲区（NOSPLIT：整帧原子写入）
        // 超时为 0：缓冲区满时立即丢帧（不阻塞），保证实时性
        BaseType_t ret = xRingbufferSend(
            audio_encoder->output_buffer,
            out_frame.buffer,
            out_frame.encoded_bytes, // 实际编码字节数（不是固定值）
            0);                      // 超时 0ms：满了立即返回 pdFAIL

        if (ret == pdFAIL)
        {
            // 下游消费速度跟不上（WebSocket 发送阻塞），丢帧警告
            // 正常情况下不会发生（enc_output 8KB 足够缓冲数秒音频）
            ESP_LOGW(TAG, "enc_output 缓冲区满，丢弃一帧 OPUS 数据");
        }

        // 每编码完一帧主动让步，防止 SILK 编码偶发耗时拉满 CPU0 饿死同核任务。
        // 使用 taskYIELD() 而非 vTaskDelay(2ms)：
        //   - vTaskDelay(2) 强制睡 2 tick，累计 10% 吞吐损失，在追赶积压时更慢；
        //   - taskYIELD() 仅在同优先级就绪队列里切一圈，无就绪任务时立即回来。
        // IDLE0 喂狗依靠切换到 play_task / decoder_task 时自然轮到 IDLE0。
        taskYIELD(); // 主动让步，防止偶发编码耗时拉满 CPU0 饿死同核任务
    }

    // ── 任务退出：释放帧缓冲区，清空句柄，自删除 ─────────────────────────────
    free(in_frame.buffer);
    free(out_frame.buffer);
    audio_encoder->task_handle = NULL; // 通知 stop() 任务已安全退出
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 生命周期管理
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 创建 OPUS 编码器实例，注册并配置编码参数
 *
 * @param sample_rate 采样率（Hz），应为 BSP_CODEC_SAMPLE_RATE（16000）
 * @param channels    声道数，应为 1（单声道）
 * @return audio_encoder_t* 编码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 * @note 创建后需调用 audio_encoder_set_buffer() 绑定缓冲区，再调用 audio_encoder_start()
 */
audio_encoder_t *audio_encoder_create(int sample_rate, int channels)
{
    // ── 步骤 1：分配并清零编码器结构体 ───────────────────────────────────────
    audio_encoder_t *audio_encoder = malloc_zeroed(sizeof(audio_encoder_t));

    // ── 步骤 2：向 esp_audio_codec 框架注册 OPUS 编码器（全局只需注册一次）──
    // 注册后框架可识别 ESP_AUDIO_TYPE_OPUS 类型，后续 open() 才能找到对应实现
    // ★ 使用静态标志确保只注册一次，避免反复 register/unregister 导致内部 SRAM 泄漏（每次约 0.7KB）
    {
        static bool s_opus_enc_registered = false;
        if (!s_opus_enc_registered)
        {
            ESP_ERROR_CHECK(esp_opus_enc_register());
            s_opus_enc_registered = true;
        }
    }

    // ── 步骤 3：配置 OPUS 编码参数 ────────────────────────────────────────────
    esp_opus_enc_config_t opus_config = {
        .sample_rate = sample_rate,                   // 采样率：16000 Hz
        .bits_per_sample = BSP_CODEC_BITS_PER_SAMPLE, // 位深：16-bit
        .channel = channels,                          // 声道：1（单声道）
        // 比特率：16kbps（原 24kbps 会触发 SILK NSQ 延迟决策路径，单帧耗时尖峰达 30-50ms，
        // 拉满 CPU0 引发 task_wdt；16kbps 仍足以传清晰语音，用于 ASR 识别无明显影响）
        .bitrate = 24000,
        .frame_duration = ESP_OPUS_ENC_FRAME_DURATION_20_MS, // 帧时长：20ms（320采样点）
        // 复杂度：0（原 3，SILK 在 complexity≥2 启用 delayed-decision NSQ，CPU 占用翻倍；
        // 0 改用简单 NSQ，帧内耗时更平稳，适合 CPU 紧张场景）
        .complexity = 0,
        .application_mode = ESP_OPUS_ENC_APPLICATION_VOIP, // 模式：VOIP（针对语音优化）
        .enable_fec = false,                               // 禁用前向纠错（WiFi 无线不需要，有线更稳定）
        .enable_dtx = false,                               // 禁用不连续传输（保持连续流，避免静音期丢帧）
        .enable_vbr = false,                               // 禁用可变比特率（固定码率，保证延迟稳定性）
    };

    // 封装为通用编码器配置结构体（框架层接口）
    esp_audio_enc_config_t config = {
        .cfg = &opus_config,
        .cfg_sz = sizeof(esp_opus_enc_config_t),
        .type = ESP_AUDIO_TYPE_OPUS,
    };

    // ── 步骤 4：打开编码器，获取句柄 ─────────────────────────────────────────
    // 内部分配 OPUS 编码状态机、滤波器系数等运行时资源
    ESP_ERROR_CHECK(esp_audio_enc_open(&config, &audio_encoder->enc));

    return audio_encoder;
}

/**
 * @brief 销毁编码器实例（关闭句柄 → 注销类型 → 释放内存）
 *
 * @param audio_encoder 编码器实例指针（调用后不可再使用）
 *
 * @note 调用者：audio_processor.c → audio_processor_destroy()
 */
void audio_encoder_destroy(audio_encoder_t *audio_encoder)
{
    // 关闭编码器：释放 OPUS 运行时内部状态（编码缓冲区、滤波器状态等）
    esp_audio_enc_close(audio_encoder->enc);

    // ★ 不再注销 OPUS 编码器类型：全局只注册一次，避免反复 register/unregister 泄漏内部 SRAM
    // esp_audio_enc_unregister(ESP_AUDIO_TYPE_OPUS);

    // 释放编码器结构体本身
    free(audio_encoder);
}

/**
 * @brief 绑定输入/输出环形缓冲区（供编码任务读写数据）
 *
 * @param audio_encoder  编码器实例指针
 * @param input_buffer   PCM 输入缓冲区（BYTEBUF 类型，来自 audio_processor_create）
 * @param output_buffer  OPUS 输出缓冲区（NOSPLIT 类型，来自 audio_processor_create）
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
void audio_encoder_set_buffer(audio_encoder_t *audio_encoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer)
{
    audio_encoder->input_buffer = input_buffer;   // PCM 数据来源（麦克风采集链路末端）
    audio_encoder->output_buffer = output_buffer; // OPUS 数据去向（WebSocket 发送链路入口）
}

/**
 * @brief 启动编码任务（设置运行标志，创建 FreeRTOS 任务）
 *
 * @param audio_encoder 编码器实例指针（已绑定缓冲区）
 *
 * @note 调用者：audio_processor.c → audio_processor_start()
 */
void audio_encoder_start(audio_encoder_t *audio_encoder)
{
    // ── 前置检查：确保缓冲区已绑定 ────────────────────────────────────────────
    if (!audio_encoder->input_buffer || !audio_encoder->output_buffer)
    {
        ESP_LOGW(TAG, "编码器缓冲区未设置，拒绝启动任务");
        return;
    }

    // ── 设置运行标志（任务主循环检查此标志决定是否继续）──────────────────────
    audio_encoder->is_running = true;

    // ── 创建编码任务（栈分配在 SPIRAM，节省内部 SRAM）───────────────────────
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        audio_encoder_task,                          // 任务函数
        "encoder_task",                              // 任务名称
        AUDIO_ENCODER_TASK_STACK_SIZE,               // 栈大小：32KB
        audio_encoder,                               // 任务参数：编码器实例指针
        AUDIO_ENCODER_TASK_PRIORITY,                 // 优先级：5
        (TaskHandle_t *)&audio_encoder->task_handle, // 保存句柄，用于超时强制终止
        AUDIO_ENCODER_TASK_CORE_ID,                  // 绑定核心：CPU0
        MALLOC_CAP_SPIRAM);                          // 栈内存来源：外部 SPIRAM

    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "编码任务创建失败（内存不足或参数错误）");
        audio_encoder->task_handle = NULL;
    }
}

/**
 * @brief 停止编码任务（清除运行标志，等待任务自然退出）
 *
 * @param audio_encoder 编码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_stop()
 * @note 200ms 等待时间基于任务内 100ms 超时读取，确保任务有机会检测到 is_running=false
 */
void audio_encoder_stop(audio_encoder_t *audio_encoder)
{
    // 清除运行标志：任务在下次 xRingbufferReceiveUpTo 超时（100ms）后检测到并退出
    audio_encoder->is_running = false;

    // 等待任务自然退出（最多 400ms：100ms 读超时 + 编码耗时 + 余量）
    for (int i = 0; i < 20 && audio_encoder->task_handle != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(20));

    // 超时仍未退出：强制终止，防止 SPIRAM 栈永久泄漏（32KB per session）
    if (audio_encoder->task_handle != NULL)
    {
        ESP_LOGW(TAG, "编码任务超时未退出，强制终止以释放 SPIRAM 栈");
        vTaskDelete(audio_encoder->task_handle);
        audio_encoder->task_handle = NULL;
    }
}
