/**
 * @file audio_decoder.c
 * @brief 音频解码器模块实现（OPUS → PCM）
 */

#include "audio_decoder.h"
#include "esp_audio_dec.h"
#include "esp_audio_dec_default.h"
#include "esp_audio_dec_reg.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "esp_log.h"

#define TAG "[AP] Decoder"

// 解码器任务配置
#define AUDIO_DECODER_TASK_CORE_ID 0        // 固定到 CPU 核心 0
#define AUDIO_DECODER_TASK_STACK_SIZE 32768 // 栈大小 32KB
#define AUDIO_DECODER_TASK_PRIORITY 5       // 优先级（与编码器对称）

/**
 * @brief 音频解码器数据结构
 */
struct audio_decoder
{
    RingbufHandle_t input_buffer;  ///< 输入环形缓冲区（存放 OPUS 压缩数据）
    RingbufHandle_t output_buffer; ///< 输出环形缓冲区（存放解码后 PCM 数据）
    esp_audio_dec_handle_t dec;    ///< 解码器句柄

    int sample_rate; ///< 采样率（Hz）
    int channels;    ///< 声道数

    bool is_running; ///< 运行状态标志
};

// ─── 解码器任务：持续从输入缓冲读 OPUS，解码后写入输出缓冲 ──────────────

void audio_decoder_task(void *arg)
{
    audio_decoder_t *audio_decoder = (audio_decoder_t *)arg;

    // 计算每帧 PCM 输出大小：采样率 × 声道 × 2字节/样本 × 帧时长(ms) / 1000
    // 例：16000 × 2ch × 2 × 60ms/1000 = 3840 字节
    size_t out_buffer_size = audio_decoder->sample_rate * audio_decoder->channels * 2 // 16-bit = 2 bytes/sample
                             / 1000 * 60;                                             // 60ms 帧

    // 分配 PCM 输出缓冲区
    void *out_buffer = malloc_zeroed(out_buffer_size);

    esp_audio_dec_out_frame_t out_frame = {
        .buffer = out_buffer,
        .len = out_buffer_size,
    };

    while (audio_decoder->is_running)
    {
        // 从输入缓冲区读取一个 OPUS 数据包（阻塞 100ms 超时）
        size_t size_read = 0;
        void *buf_read = xRingbufferReceive(
            audio_decoder->input_buffer, &size_read, pdMS_TO_TICKS(100));

        if (!buf_read)
            continue; // 超时无数据，继续等待

        // 构造输入帧描述符
        esp_audio_dec_in_raw_t in_frame = {
            .buffer = buf_read,
            .len = size_read,
        };

        // 执行 OPUS 解码（in_frame → out_frame）
        esp_audio_err_t ret = esp_audio_dec_process(audio_decoder->dec, &in_frame, &out_frame);

        // 归还环形缓冲区使用权（解码完成后立即归还，减少缓冲区占用）
        vRingbufferReturnItem(audio_decoder->input_buffer, buf_read);

        if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH)
        {
            // 输出缓冲区不足（帧时长配置与实际不符），记录警告
            ESP_LOGW(TAG, "Output buffer not enough");
        }

        if (ret != ESP_OK)
            continue; // 解码失败，丢弃本帧

        // 将解码后的 PCM 数据写入输出缓冲区（满则丢帧，不阻塞）
        BaseType_t buf_ret = xRingbufferSend(
            audio_decoder->output_buffer,
            out_frame.buffer,
            out_frame.decoded_size, // 实际解码输出字节数
            0);                     // 超时 0：满了直接返回失败

        if (buf_ret != pdTRUE)
            ESP_LOGW(TAG, "Output buffer full"); // 下游播放任务消费太慢
    }
    free(out_buffer);
    vTaskDelete(NULL);
}

// ─── 创建解码器实例 ─────────────────────────────────────────────────────

audio_decoder_t *audio_decoder_create(int sample_rate, int channels)
{
    // 分配并清零解码器结构体
    audio_decoder_t *audio_decoder = (audio_decoder_t *)malloc_zeroed(sizeof(audio_decoder_t));

    // 保存采样参数（任务内计算输出缓冲区大小时使用）
    audio_decoder->sample_rate = sample_rate;
    audio_decoder->channels = channels;

    // 向编解码框架注册 OPUS 解码器
    ESP_ERROR_CHECK(esp_opus_dec_register());

    // 配置 OPUS 解码参数（必须与编码端保持一致）
    esp_opus_dec_cfg_t opus_cfg = {
        .sample_rate = sample_rate,
        .channel = channels,
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS, // 帧时长：60ms（与编码端对齐）
        .self_delimited = false,                             // 标准 OPUS 封包格式
    };

    // 封装为通用解码器配置
    esp_audio_dec_cfg_t dec_cfg = {
        .cfg = &opus_cfg,
        .cfg_sz = sizeof(esp_opus_dec_cfg_t),
        .type = ESP_AUDIO_TYPE_OPUS,
    };

    // 打开解码器，获取句柄
    ESP_ERROR_CHECK(esp_audio_dec_open(&dec_cfg, &audio_decoder->dec));

    return audio_decoder;
}

// ─── 销毁解码器实例 ─────────────────────────────────────────────────────

void audio_decoder_destroy(audio_decoder_t *audio_decoder)
{
    // 关闭解码器（释放内部解码状态）
    esp_audio_dec_close(audio_decoder->dec);

    // 注销 OPUS 解码器
    esp_audio_dec_unregister(ESP_AUDIO_TYPE_OPUS);

    // 释放解码器结构体
    free(audio_decoder);
}

// ─── 设置输入/输出缓冲区 ────────────────────────────────────────────────

void audio_decoder_set_buffer(audio_decoder_t *audio_decoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer)
{
    // 保存输入缓冲区句柄（OPUS 数据来源）
    audio_decoder->input_buffer = input_buffer;
    // 保存输出缓冲区句柄（PCM 数据去向，由播放任务消费）
    audio_decoder->output_buffer = output_buffer;
}

// ─── 启动解码器任务 ─────────────────────────────────────────────────────

void audio_decoder_start(audio_decoder_t *audio_decoder)
{
    // 前置检查：输入和输出缓冲区必须已设置
    if (!audio_decoder->input_buffer || !audio_decoder->output_buffer)
    {
        ESP_LOGW(TAG, "Input or output buffer not set");
        return;
    }

    // 设置运行标志
    audio_decoder->is_running = true;

    // 创建解码任务：固定到 CPU 核心 0，栈分配在 SPIRAM
    xTaskCreatePinnedToCoreWithCaps(
        audio_decoder_task,            // 任务函数
        "decoder_task",                // 任务名称
        AUDIO_DECODER_TASK_STACK_SIZE, // 栈大小（32KB）
        audio_decoder,                 // 传入解码器实例指针
        AUDIO_DECODER_TASK_PRIORITY,   // 优先级
        NULL,                          // 不需要保存任务句柄
        AUDIO_DECODER_TASK_CORE_ID,    // 固定到核心 0
        MALLOC_CAP_SPIRAM);            // 任务栈分配到外部 SPIRAM
}

// ─── 停止解码器任务 ─────────────────────────────────────────────────────

void audio_decoder_stop(audio_decoder_t *audio_decoder)
{
    // 清除运行标志，任务检测到后退出并自删除
    audio_decoder->is_running = false;

    // 等待 200ms 让任务有机会退出（与编码器保持对称）
    vTaskDelay(pdMS_TO_TICKS(200));
}
