/**
 * @file audio_decoder.c
 * @brief OPUS 音频解码器模块实现（OPUS → PCM）
 *
 * 从云端 WebSocket 接收的 OPUS 压缩音频帧，经解码后输出 PCM 数据，
 * 由播放任务（audio_processor_play_task）送入 ES8311 DAC → 扬声器。
 *
 * 数据流：
 *   WebSocket 接收 → audio_processor_write() → [dec_input: NOSPLIT 5KB]
 *   → audio_decoder_task（本任务）→ [dec_output: BYTEBUF 40KB]
 *   → audio_processor_play_task → esp_codec_dev_write() → 扬声器
 */

#include "audio_decoder.h"
#include "esp_audio_dec.h"
#include "esp_audio_dec_default.h"
#include "esp_audio_dec_reg.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "esp_log.h"

#define TAG "[AP] Decoder"

// ─── 解码器任务配置宏 ─────────────────────────────────────────────────────────
#define AUDIO_DECODER_TASK_CORE_ID    0      ///< 解码任务绑定 CPU0（与编码/播放同核）
#define AUDIO_DECODER_TASK_STACK_SIZE 32768  ///< 栈大小 32KB（OPUS 解码需要较大栈）
#define AUDIO_DECODER_TASK_PRIORITY   5      ///< 优先级 5（与编码器对称）

/**
 * @brief 解码器内部结构体（对外不透明，通过 audio_decoder_t* 访问）
 *
 * 字段说明：
 *   input_buffer  : NOSPLIT 类型，每次 xRingbufferReceive 读取整个 OPUS 帧
 *   output_buffer : BYTEBUF 类型，PCM 数据可自由分段写入（无帧边界约束）
 *   dec            : OPUS 解码器句柄（esp_audio_codec 框架）
 *   sample_rate    : 采样率（用于计算 PCM 输出缓冲区大小）
 *   channels       : 声道数（同上）
 *   is_running     : 任务退出标志
 */
struct audio_decoder
{
    RingbufHandle_t input_buffer;  ///< 输入缓冲区：OPUS 帧（NOSPLIT，整帧读取）
    RingbufHandle_t output_buffer; ///< 输出缓冲区：PCM 数据（BYTEBUF，分段写入）
    esp_audio_dec_handle_t dec;    ///< OPUS 解码器句柄

    int sample_rate; ///< 采样率（Hz），用于计算 PCM 输出缓冲区大小
    int channels;    ///< 声道数

    bool is_running; ///< 任务运行标志：false 时解码循环退出，任务自删除
};

// ═══════════════════════════════════════════════════════════════════════════════
// 解码器任务（核心解码循环）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief OPUS 音频解码主任务（持续将 OPUS 帧解码为 PCM 帧）
 *
 * 数据流：
 *   [dec_input: NOSPLIT] → xRingbufferReceive（整帧）→ esp_audio_dec_process() → [dec_output: BYTEBUF]
 *
 * 关键设计细节：
 *   1. NOSPLIT 类型保证每次 xRingbufferReceive 返回一完整 OPUS 帧（不被拆分）
 *   2. 输出缓冲区（40KB ≈ 1.28s PCM）足够大，正常不会满
 *   3. 输出缓冲区满时丢帧（xRingbufferSend timeout=0），保证实时性
 *
 * @param arg audio_decoder_t* 实例指针
 * @return 无（任务运行直到 is_running 为 false）
 *
 * @note 调用者：audio_decoder_start() 通过 xTaskCreatePinnedToCoreWithCaps() 创建
 * @note 运行核心：CPU0，栈 32KB SPIRAM，优先级 5
 */
void audio_decoder_task(void *arg)
{
    audio_decoder_t *audio_decoder = (audio_decoder_t *)arg;

    // ── 步骤 1：计算 PCM 输出缓冲区大小 ──────────────────────────────────────
    // 公式：采样率 × 声道数 × 每采样字节数 × 帧时长(ms) / 1000
    // 例：16000Hz × 1ch × 2bytes × 60ms / 1000 = 1920 字节/帧
    // 注意：云端编码时使用 60ms 帧，此处解码输出也必须是 60ms 的 PCM
    size_t out_buffer_size = (size_t)audio_decoder->sample_rate  // 采样率 16000
                             * audio_decoder->channels            // 声道数 1
                             * 2                                  // 16-bit = 2 字节/采样
                             / 1000                               // 换算 ms → s
                             * 60;                                // 帧时长 60ms

    // ── 步骤 2：分配 PCM 输出缓冲区 ─────────────────────────────────────────
    void *out_buffer = malloc_zeroed(out_buffer_size); // 1920 字节（清零防止噪音）

    esp_audio_dec_out_frame_t out_frame = {
        .buffer = out_buffer,      // 解码输出目标地址
        .len    = out_buffer_size, // 最大可容纳字节数
    };

    // ── 步骤 3：解码主循环 ────────────────────────────────────────────────────
    while (audio_decoder->is_running)
    {
        // 从 dec_input（NOSPLIT）读取一个完整的 OPUS 帧（不会被拆分）
        // 100ms 超时：无数据时定期检查 is_running 标志
        size_t size_read = 0;
        void *buf_read = xRingbufferReceive(
            audio_decoder->input_buffer,
            &size_read,
            pdMS_TO_TICKS(100)); // 100ms 超时

        if (!buf_read)
            continue; // 超时无数据（TTS 结束后的静音期），继续等待

        // 构造解码输入帧描述符（指向 RingBuf 内部内存，避免拷贝）
        esp_audio_dec_in_raw_t in_frame = {
            .buffer = buf_read,   // OPUS 帧数据指针（指向 RingBuf 内部）
            .len    = size_read,  // 帧大小（字节，由编码端决定，约 20~100 字节）
        };

        // 执行 OPUS 解码（1920字节 PCM → out_frame）
        esp_audio_err_t ret = esp_audio_dec_process(
            audio_decoder->dec, &in_frame, &out_frame);

        // 归还环形缓冲区使用权（解码完成后立即归还，减少缓冲区占用时间）
        // 必须在使用完 buf_read 数据后才能归还
        vRingbufferReturnItem(audio_decoder->input_buffer, buf_read);

        // 错误处理
        if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH)
        {
            // 输出缓冲区不足：out_buffer_size 计算与实际帧时长不符
            // 通常说明云端编码帧时长与解码配置（60ms）不一致
            ESP_LOGW(TAG, "PCM 输出缓冲区不足，请检查帧时长配置");
        }

        if (ret != ESP_OK)
            continue; // 解码失败（数据损坏），丢弃本帧，继续处理下一帧

        // ── 将解码后的 PCM 数据写入播放缓冲区 ─────────────────────────────
        // BYTEBUF 类型：支持分段写入，PCM 数据无帧边界约束
        // 超时为 0：dec_output 满时立即返回失败（丢帧），保证解码不被播放阻塞
        BaseType_t buf_ret = xRingbufferSend(
            audio_decoder->output_buffer,
            out_frame.buffer,
            out_frame.decoded_size, // 实际解码输出字节数（通常就是 out_buffer_size）
            0);                     // 超时 0ms：满了立即返回 pdFALSE

        if (buf_ret != pdTRUE)
        {
            // dec_output 40KB 通常不会满（1.28s 缓冲），满了说明播放任务异常
            ESP_LOGW(TAG, "dec_output 缓冲区满，丢弃一帧 PCM 数据（播放可能卡顿）");
        }
    }

    // ── 任务退出：释放 PCM 输出缓冲区，自删除 ────────────────────────────────
    free(out_buffer);
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 生命周期管理
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 创建 OPUS 解码器实例，注册并配置解码参数
 *
 * @param sample_rate 采样率（Hz），必须与云端编码端一致（16000）
 * @param channels    声道数（1 = 单声道）
 * @return audio_decoder_t* 解码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
audio_decoder_t *audio_decoder_create(int sample_rate, int channels)
{
    // ── 步骤 1：分配并清零解码器结构体 ───────────────────────────────────────
    audio_decoder_t *audio_decoder = (audio_decoder_t *)malloc_zeroed(sizeof(audio_decoder_t));

    // 保存采样参数（任务内计算 PCM 输出缓冲区大小时使用）
    audio_decoder->sample_rate = sample_rate;
    audio_decoder->channels    = channels;

    // ── 步骤 2：注册 OPUS 解码器 ──────────────────────────────────────────────
    ESP_ERROR_CHECK(esp_opus_dec_register());

    // ── 步骤 3：配置解码参数（必须与编码端严格一致）─────────────────────────
    esp_opus_dec_cfg_t opus_cfg = {
        .sample_rate    = sample_rate,                          // 采样率：16000Hz
        .channel        = channels,                             // 声道数：1
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS,   // 帧时长：60ms（与编码端对齐！）
        .self_delimited = false,                                // 标准 OPUS 封包格式（非自分界）
    };

    // 封装为通用解码器配置
    esp_audio_dec_cfg_t dec_cfg = {
        .cfg    = &opus_cfg,
        .cfg_sz = sizeof(esp_opus_dec_cfg_t),
        .type   = ESP_AUDIO_TYPE_OPUS,
    };

    // ── 步骤 4：打开解码器，获取句柄 ─────────────────────────────────────────
    ESP_ERROR_CHECK(esp_audio_dec_open(&dec_cfg, &audio_decoder->dec));

    return audio_decoder;
}

/**
 * @brief 销毁解码器实例（关闭句柄 → 注销类型 → 释放内存）
 *
 * @param audio_decoder 解码器实例指针（调用后不可再使用）
 *
 * @note 调用者：audio_processor.c → audio_processor_destroy()
 */
void audio_decoder_destroy(audio_decoder_t *audio_decoder)
{
    // 关闭解码器：释放 OPUS 解码状态机内部资源
    esp_audio_dec_close(audio_decoder->dec);

    // 注销 OPUS 解码器类型（从框架注册表移除）
    esp_audio_dec_unregister(ESP_AUDIO_TYPE_OPUS);

    // 释放解码器结构体
    free(audio_decoder);
}

/**
 * @brief 绑定输入/输出环形缓冲区
 *
 * @param audio_decoder  解码器实例指针
 * @param input_buffer   OPUS 帧输入（NOSPLIT 类型，整帧读取）
 * @param output_buffer  PCM 输出（BYTEBUF 类型，分段写入）
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
void audio_decoder_set_buffer(audio_decoder_t *audio_decoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer)
{
    audio_decoder->input_buffer  = input_buffer;  // OPUS 来源（WebSocket 接收链路）
    audio_decoder->output_buffer = output_buffer; // PCM 去向（播放任务链路入口）
}

/**
 * @brief 启动解码任务
 *
 * @param audio_decoder 解码器实例指针（已绑定缓冲区）
 *
 * @note 调用者：audio_processor.c → audio_processor_start()
 */
void audio_decoder_start(audio_decoder_t *audio_decoder)
{
    // ── 前置检查 ──────────────────────────────────────────────────────────────
    if (!audio_decoder->input_buffer || !audio_decoder->output_buffer)
    {
        ESP_LOGW(TAG, "解码器缓冲区未设置，拒绝启动任务");
        return;
    }

    // ── 设置运行标志 ──────────────────────────────────────────────────────────
    audio_decoder->is_running = true;

    // ── 创建解码任务（栈分配在 SPIRAM）───────────────────────────────────────
    xTaskCreatePinnedToCoreWithCaps(
        audio_decoder_task,              // 任务函数
        "decoder_task",                  // 任务名称
        AUDIO_DECODER_TASK_STACK_SIZE,   // 栈大小：32KB
        audio_decoder,                   // 任务参数：解码器实例指针
        AUDIO_DECODER_TASK_PRIORITY,     // 优先级：5
        NULL,                            // 不保存任务句柄
        AUDIO_DECODER_TASK_CORE_ID,      // 绑定核心：CPU0
        MALLOC_CAP_SPIRAM);              // 栈内存来源：外部 SPIRAM
}

/**
 * @brief 停止解码任务（清除运行标志，等待任务自然退出）
 *
 * @param audio_decoder 解码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_stop()
 */
void audio_decoder_stop(audio_decoder_t *audio_decoder)
{
    // 清除运行标志，任务在下次 xRingbufferReceive 超时（100ms）后检测到并退出
    audio_decoder->is_running = false;

    // 等待 200ms 确保任务安全退出（与编码器保持对称）
    vTaskDelay(pdMS_TO_TICKS(200));
}
