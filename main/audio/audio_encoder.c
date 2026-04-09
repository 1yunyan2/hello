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

// 编码器任务配置
#define AUDIO_ENCODER_TASK_CORE_ID 0        // 绑定到 CPU 核心 0
#define AUDIO_ENCODER_TASK_STACK_SIZE 32768 // 栈大小 32KB（OPUS 编码运算量较大）
#define AUDIO_ENCODER_TASK_PRIORITY 5       // 优先级（与解码器对称）

struct audio_encoder
{
    RingbufHandle_t input_buffer;  ///< 输入环形缓冲区（接收原始 PCM 数据）
    RingbufHandle_t output_buffer; ///< 输出环形缓冲区（存放 OPUS 压缩数据）
    esp_audio_enc_handle_t enc;    ///< 编码器句柄

    bool is_running; ///< 运行状态标志（false 时任务退出）
};

// ─── 编码器任务：持续从输入缓冲读 PCM，编码后写入输出缓冲 ────────────────

void audio_encoder_task(void *arg)
{
    audio_encoder_t *audio_encoder = (audio_encoder_t *)arg;

    // 向编码器查询每帧所需的输入字节数和编码后的输出字节数
    int in_frame_size = 0, out_frame_size = 0;
    esp_audio_enc_get_frame_size(audio_encoder->enc, &in_frame_size, &out_frame_size);

    // 分配输入帧缓冲区（存放待编码的原始 PCM 数据）
    void *in_buf = malloc_zeroed(in_frame_size);
    if (in_buf == NULL)
    {
        ESP_LOGE(TAG, "输入帧缓冲区分配失败（需要 %d 字节）", in_frame_size);
        vTaskDelete(NULL);
        return;
    }

    // 分配输出帧缓冲区（存放编码后的 OPUS 数据）
    void *out_buf = malloc_zeroed(out_frame_size);
    if (out_buf == NULL)
    {
        ESP_LOGE(TAG, "输出帧缓冲区分配失败（需要 %d 字节）", out_frame_size);
        free(in_buf);
        vTaskDelete(NULL);
        return;
    }

    // 初始化输入帧描述符
    esp_audio_enc_in_frame_t in_frame = {
        .buffer = in_buf,
        .len = in_frame_size,
    };

    // 初始化输出帧描述符
    esp_audio_enc_out_frame_t out_frame = {
        .buffer = out_buf,
        .len = out_frame_size,
    };

    while (audio_encoder->is_running)
    {
        // 从输入缓冲区分段读取 PCM 数据，直到凑满一帧（阻塞 100ms 超时）
        size_t size_read = 0;
        void *buf_read = xRingbufferReceiveUpTo(
            audio_encoder->input_buffer, &size_read,
            pdMS_TO_TICKS(100), in_frame_size);

        if (!buf_read)
            continue; // 超时无数据，继续等待

        // 将读到的片段追加到输入帧缓冲
        memcpy(in_buf, buf_read, size_read);

        // 归还环形缓冲区的使用权（必须及时归还，否则缓冲区会满）
        vRingbufferReturnItem(audio_encoder->input_buffer, buf_read);

        // 扣减本帧剩余需要的字节数
        in_frame_size -= size_read;
        in_buf += size_read; // 移动指针到下一个写入位置

        if (in_frame_size > 0)
            continue; // 当前帧数据尚未凑满，继续读取

        // ─── 一帧 PCM 已凑齐，执行 OPUS 编码 ──────────────────────────────
        // 重置缓冲区指针和剩余长度，准备接收下一帧
        in_buf = in_frame.buffer;
        in_frame_size = in_frame.len;

        // 执行 OPUS 编码（in_frame → out_frame）
        esp_audio_enc_process(audio_encoder->enc, &in_frame, &out_frame);

        // 将编码结果写入输出缓冲区（不阻塞：满则丢弃，避免实时流卡顿）
        BaseType_t ret = xRingbufferSend(
            audio_encoder->output_buffer,
            out_frame.buffer,
            out_frame.encoded_bytes, // 编码后实际字节数（非固定值）
            0);                      // 超时为 0：满了直接返回失败

        if (ret == pdFAIL)
            ESP_LOGW(TAG, "Failed to write to output buffer"); // 下游消费太慢，丢帧
    }

    // 任务退出时释放两个帧缓冲区
    free(in_frame.buffer);
    free(out_frame.buffer);
    vTaskDelete(NULL);
}

// ─── 创建编码器实例 ─────────────────────────────────────────────────────

audio_encoder_t *audio_encoder_create(int sample_rate, int channels)
{
    // 分配并清零编码器结构体
    audio_encoder_t *audio_encoder = malloc_zeroed(sizeof(audio_encoder_t));

    // 向编解码框架注册 OPUS 编码器（只需注册一次）
    ESP_ERROR_CHECK(esp_opus_enc_register());

    // 配置 OPUS 编码参数
    esp_opus_enc_config_t opus_config = {
        .sample_rate = sample_rate,
        .bits_per_sample = BSP_CODEC_BITS_PER_SAMPLE, // 16-bit
        .channel = channels,
        .bitrate = 24000,                                    // 比特率：32kbps（VoIP 场景推荐值）
        .frame_duration = ESP_OPUS_ENC_FRAME_DURATION_20_MS, // 帧时长：60ms（每帧 960 采样点）
        .complexity = 3,                                     // 复杂度：最低，节省 CPU
        .application_mode = ESP_OPUS_ENC_APPLICATION_VOIP,   // 应用模式：VoIP（针对语音优化）
        .enable_fec = false,                                 // 禁用前向纠错（有线/WiFi 不需要）
        .enable_dtx = false,                                 // 禁用不连续传输（保持连续流）
        .enable_vbr = true,                                  // 禁用可变比特率（保证延迟稳定）
    };

    // 封装为通用编码器配置
    esp_audio_enc_config_t config = {
        .cfg = &opus_config,
        .cfg_sz = sizeof(esp_opus_enc_config_t),
        .type = ESP_AUDIO_TYPE_OPUS,
    };

    // 打开编码器，获取句柄
    ESP_ERROR_CHECK(esp_audio_enc_open(&config, &audio_encoder->enc));

    return audio_encoder;
}

// ─── 销毁编码器实例 ─────────────────────────────────────────────────────

void audio_encoder_destroy(audio_encoder_t *audio_encoder)
{
    // 关闭编码器（释放内部编码状态和缓冲区）
    esp_audio_enc_close(audio_encoder->enc);

    // 从框架中注销 OPUS 编码器（释放注册占用的资源）
    esp_audio_enc_unregister(ESP_AUDIO_TYPE_OPUS);

    // 释放编码器结构体本身
    free(audio_encoder);
}

// ─── 设置输入/输出缓冲区 ────────────────────────────────────────────────

void audio_encoder_set_buffer(audio_encoder_t *audio_encoder, RingbufHandle_t input_buffer, RingbufHandle_t output_buffer)
{
    // 保存输入缓冲区句柄（PCM 数据来源）
    audio_encoder->input_buffer = input_buffer;
    // 保存输出缓冲区句柄（OPUS 数据去向）
    audio_encoder->output_buffer = output_buffer;
}

// ─── 启动编码器任务 ─────────────────────────────────────────────────────

void audio_encoder_start(audio_encoder_t *audio_encoder)
{
    // 前置检查：输入和输出缓冲区必须已设置
    if (!audio_encoder->input_buffer || !audio_encoder->output_buffer)
    {
        ESP_LOGW(TAG, "Input or output buffer not set");
        return;
    }

    // 设置运行标志（任务内部轮询此标志决定是否继续）
    audio_encoder->is_running = true;

    // 创建编码任务：固定到 CPU 核心 0，栈分配在 SPIRAM（节省内部 SRAM）
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        audio_encoder_task,            // 任务函数
        "encoder_task",                // 任务名称
        AUDIO_ENCODER_TASK_STACK_SIZE, // 栈大小（32KB）
        audio_encoder,                 // 传入编码器实例指针
        AUDIO_ENCODER_TASK_PRIORITY,   // 优先级
        NULL,                          // 不需要保存任务句柄
        AUDIO_ENCODER_TASK_CORE_ID,    // 固定到核心 0
        MALLOC_CAP_SPIRAM);            // 任务栈分配到外部 SPIRAM

    if (ret != pdPASS)
        ESP_LOGW(TAG, "Failed to create encoder task");
}

// ─── 停止编码器任务 ─────────────────────────────────────────────────────

void audio_encoder_stop(audio_encoder_t *audio_encoder)
{
    // 清除运行标志，任务检测到后会退出循环并自删除
    audio_encoder->is_running = false;

    // 等待 200ms 让任务有机会退出（避免句柄悬空）
    vTaskDelay(pdMS_TO_TICKS(200));
}
