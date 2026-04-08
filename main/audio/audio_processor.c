/**
 * @file audio_processor.c
 * @brief 音频处理器 — 编解码管道的中枢协调者
 *
 * 本模块是音频上行（麦克风→云端）和下行（云端→扬声器）的数据枢纽，
 * 内部管理编码器、解码器和四个环形缓冲区，提供统一的读写接口。
 *
 * 数据流架构：
 *
 *   上行（录音→编码→发送）：
 *     麦克风 PCM → [enc_input] → 编码器(OPUS) → [enc_output] → WebSocket 发送任务
 *
 *   下行（接收→解码→播放）：
 *     WebSocket 接收 → [dec_input] → 解码器(PCM) → [dec_output] → 播放任务 → 扬声器
 *
 * 四个环形缓冲区的类型选择：
 *   enc_input  / dec_output = BYTEBUF（字节流，PCM 数据可自由拼接）
 *   enc_output / dec_input  = NOSPLIT（不拆分，OPUS 帧必须保持完整性）
 */

#include "audio_processor.h"
#include "audio_encoder.h"
#include "audio_decoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "esp_log.h"

#define TAG "Audio Processor"

/* 播放任务配置 */
#define AUDIO_PROCESSOR_TASK_STACK_SIZE (4 * 1024) // 栈大小 4KB（播放逻辑简单，无需更大）
#define AUDIO_PROCESSOR_TASK_PRIORITY 5            // 优先级（与编解码器任务对称）
#define AUDIO_PROCESSOR_TASK_CORE_ID 0             // 固定到 CPU 核心 0

// ─── 环形缓冲区大小配置（单位：字节）────────────────────────────────────────
#define ENC_INPUT_BUF_SIZE 20480  // 编码器输入（原始 PCM）：~640ms @16kHz 单声道
#define ENC_OUTPUT_BUF_SIZE 8192  // 编码器输出（OPUS 帧）：增大以容纳 Hello 握手期间积压的帧
#define DEC_INPUT_BUF_SIZE 5120   // 解码器输入（OPUS 帧）：云端下发的音频缓冲
#define DEC_OUTPUT_BUF_SIZE 40960 // 解码器输出（PCM 播放）：~1.28s 缓冲，保证播放流畅

/**
 * @brief 音频处理器内部结构体
 *
 * 管理编码器/解码器实例和四个环形缓冲区，
 * 以及播放任务的生命周期控制。
 */
struct audio_processor
{
    audio_encoder_t *encoder; ///< OPUS 编码器实例（PCM → OPUS）
    audio_decoder_t *decoder; ///< OPUS 解码器实例（OPUS → PCM）

    RingbufHandle_t enc_input;  ///< 编码器输入缓冲（BYTEBUF：麦克风 PCM 数据入口）
    RingbufHandle_t enc_output; ///< 编码器输出缓冲（NOSPLIT：OPUS 帧，WebSocket 发送任务消费）
    RingbufHandle_t dec_input;  ///< 解码器输入缓冲（NOSPLIT：云端下发的 OPUS 帧）
    RingbufHandle_t dec_output; ///< 解码器输出缓冲（BYTEBUF：解码后 PCM，播放任务消费）

    volatile bool is_running;      ///< 运行标志（控制播放任务循环）
    TaskHandle_t play_task_handle; ///< 播放任务句柄（用于等待任务退出）
};

// ─── 播放任务 ──────────────────────────────────────────────────────────────

/**
 * @brief 音频播放任务 — 从解码器输出缓冲区读取 PCM 数据并送入扬声器
 *
 * 持续从 dec_output 读取解码后的 PCM 数据，通过 esp_codec_dev_write 发送到
 * ES8311 DAC → I2S → 扬声器。每次最多读取 2048 字节，100ms 超时以便响应
 * is_running 标志的变化（用于优雅退出）。
 *
 * @param arg audio_processor_t* 实例指针
 */
static void audio_processor_play_task(void *arg)
{
    audio_processor_t *audio_processor = (audio_processor_t *)arg;
    bsp_board_t *board = bsp_board_get_instance();

    /* 防御性检查：BSP 实例与 codec_dev 必须就绪，否则直接退出 */
    if (board == NULL || board->codec_dev == NULL)
    {
        ESP_LOGE(TAG, "play_task abort: board=%p codec_dev=%p",
                 board, board ? board->codec_dev : NULL);
        audio_processor->play_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    while (audio_processor->is_running)
    {
        size_t size_read = 0;
        /* 使用 100ms 超时，确保能及时响应 is_running 置 false */
        void *buf = xRingbufferReceiveUpTo(audio_processor->dec_output, &size_read,
                                           pdMS_TO_TICKS(100), 2048);
        if (buf)
        {
            /* 将 PCM 数据写入 Codec 设备（阻塞直到 I2S DMA 发送完毕） */
            esp_codec_dev_write(board->codec_dev, buf, size_read);
            vRingbufferReturnItem(audio_processor->dec_output, buf);
        }
    }

    audio_processor->play_task_handle = NULL;
    vTaskDelete(NULL);
}

// ─── 公开 API：生命周期管理 ────────────────────────────────────────────────

/**
 * @brief 创建音频处理器实例
 *
 * 依次创建编码器、解码器和四个环形缓冲区，并将缓冲区绑定到编解码器。
 * 任一环形缓冲区创建失败则整体回滚，避免半初始化对象导致后续崩溃。
 *
 * @return audio_processor_t* 成功返回处理器指针，失败返回 NULL
 */
audio_processor_t *audio_processor_create(void)
{
    audio_processor_t *audio_processor = (audio_processor_t *)malloc_zeroed(sizeof(audio_processor_t));

    /* 创建编解码器实例（单声道 16kHz，与硬件采集参数一致） */
    audio_processor->encoder = audio_encoder_create(BSP_CODEC_SAMPLE_RATE, 1);
    audio_processor->decoder = audio_decoder_create(BSP_CODEC_SAMPLE_RATE, 1);

    /* 编解码器创建失败 → 立即回滚，避免后续空指针崩溃 */
    if (!audio_processor->encoder || !audio_processor->decoder)
    {
        ESP_LOGE(TAG, "audio_processor_create: encoder/decoder alloc failed, rollback");
        if (audio_processor->encoder)
            audio_encoder_destroy(audio_processor->encoder);
        if (audio_processor->decoder)
            audio_decoder_destroy(audio_processor->decoder);
        free(audio_processor);
        return NULL;
    }

    /* 创建四个环形缓冲区（全部分配在 SPIRAM，节省内部 SRAM） */
    audio_processor->enc_input = xRingbufferCreateWithCaps(ENC_INPUT_BUF_SIZE, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);
    audio_processor->enc_output = xRingbufferCreateWithCaps(ENC_OUTPUT_BUF_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    audio_processor->dec_input = xRingbufferCreateWithCaps(DEC_INPUT_BUF_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    audio_processor->dec_output = xRingbufferCreateWithCaps(DEC_OUTPUT_BUF_SIZE, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);

    /* 任一 ringbuf 创建失败 → 整体回滚，避免半初始化对象导致后续崩溃 */
    if (!audio_processor->enc_input || !audio_processor->enc_output ||
        !audio_processor->dec_input || !audio_processor->dec_output)
    {
        ESP_LOGE(TAG, "audio_processor_create: ringbuf alloc failed, rollback");
        if (audio_processor->enc_input)
            vRingbufferDelete(audio_processor->enc_input);
        if (audio_processor->enc_output)
            vRingbufferDelete(audio_processor->enc_output);
        if (audio_processor->dec_input)
            vRingbufferDelete(audio_processor->dec_input);
        if (audio_processor->dec_output)
            vRingbufferDelete(audio_processor->dec_output);
        audio_encoder_destroy(audio_processor->encoder);
        audio_decoder_destroy(audio_processor->decoder);
        free(audio_processor);
        return NULL;
    }

    /* 将环形缓冲区绑定到编解码器（编码器：enc_input→enc_output，解码器：dec_input→dec_output） */
    audio_encoder_set_buffer(audio_processor->encoder, audio_processor->enc_input, audio_processor->enc_output);
    audio_decoder_set_buffer(audio_processor->decoder, audio_processor->dec_input, audio_processor->dec_output);
    PRINT_MEM_INFO(TAG, "音频管道环形缓冲区创建后");
    return audio_processor;
}

/**
 * @brief 销毁音频处理器实例
 * 释放四个环形缓冲区、编解码器实例和结构体本身
 */
void audio_processor_destroy(audio_processor_t *audio_processor)
{
    vRingbufferDelete(audio_processor->enc_input);
    vRingbufferDelete(audio_processor->enc_output);
    vRingbufferDelete(audio_processor->dec_input);
    vRingbufferDelete(audio_processor->dec_output);

    audio_encoder_destroy(audio_processor->encoder);
    audio_decoder_destroy(audio_processor->decoder);

    free(audio_processor);
}

// ─── 公开 API：启停控制 ────────────────────────────────────────────────────

/**
 * @brief 启动音频处理器
 * 依次启动编码器任务、解码器任务和播放任务
 */
void audio_processor_start(audio_processor_t *audio_processor)
{
    audio_processor->is_running = true;
    audio_encoder_start(audio_processor->encoder);
    audio_decoder_start(audio_processor->decoder);

    /* 创建播放任务：固定到核心 0，栈分配在 SPIRAM */
    xTaskCreatePinnedToCoreWithCaps(audio_processor_play_task, "play_task",
                                    AUDIO_PROCESSOR_TASK_STACK_SIZE, audio_processor,
                                    AUDIO_PROCESSOR_TASK_PRIORITY,
                                    &audio_processor->play_task_handle,
                                    AUDIO_PROCESSOR_TASK_CORE_ID, MALLOC_CAP_SPIRAM);
}

/**
 * @brief 停止音频处理器
 * 停止编解码器任务，等待播放任务退出（最多 300ms）
 */
void audio_processor_stop(audio_processor_t *audio_processor)
{
    audio_processor->is_running = false;
    audio_encoder_stop(audio_processor->encoder);
    audio_decoder_stop(audio_processor->decoder);
    /* 等待 play_task 退出（最多 300ms，每 100ms 检查一次） */
    for (int i = 0; i < 3 && audio_processor->play_task_handle != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(100));
}

// ─── 公开 API：数据读写 ────────────────────────────────────────────────────

/**
 * @brief 从编码器输出缓冲区读取 OPUS 数据（阻塞式）
 * 供 WebSocket 发送任务调用，会一直阻塞直到有 OPUS 帧可用
 * @return 实际读取的字节数，0 表示读取失败
 */
size_t audio_processor_read(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    size_t size_read = 0;
    void *buf_read = xRingbufferReceive(audio_processor->enc_output, &size_read, portMAX_DELAY);
    if (!buf_read)
        return 0;

    if (size_read > size)
    {
        ESP_LOGW(TAG, "Buffer size is too small");
        size_read = size;
    }

    memcpy(buffer, buf_read, size_read);
    vRingbufferReturnItem(audio_processor->enc_output, buf_read);
    return size_read;
}

/**
 * @brief 将麦克风采集的 PCM 数据写入编码器输入缓冲区
 * 这是音频上行链路的数据入口（麦克风 PCM → 编码器）
 */
void audio_processor_write_pcm(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    xRingbufferSend(audio_processor->enc_input, buffer, size, portMAX_DELAY);
}

/**
 * @brief 将云端下发的 OPUS 数据写入解码器输入缓冲区
 * 这是音频下行链路的数据入口（WebSocket 接收 → 解码器）
 */
void audio_processor_write(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    xRingbufferSend(audio_processor->dec_input, buffer, size, portMAX_DELAY);
}

/**
 * @brief 清空解码器输入和输出缓冲区
 *
 * 用于语音打断场景：用户说唤醒词打断 AI 说话时，
 * 立即清除所有积压的 TTS 音频数据，停止扬声器输出。
 */
void audio_processor_flush_output(audio_processor_t *audio_processor)
{
    size_t size;
    void *buf;
    /* 清空解码器输入（未解码的 OPUS 帧） */
    while ((buf = xRingbufferReceive(audio_processor->dec_input, &size, 0)) != NULL)
        vRingbufferReturnItem(audio_processor->dec_input, buf);
    /* 清空解码器输出（已解码但未播放的 PCM 数据） */
    while ((buf = xRingbufferReceive(audio_processor->dec_output, &size, 0)) != NULL)
        vRingbufferReturnItem(audio_processor->dec_output, buf);
}

/**
 * @brief 带超时的 OPUS 数据读取
 *
 * 类似 audio_processor_read，但不会永久阻塞。
 * 供发送任务使用，超时后可以检查会话状态决定是否继续。
 *
 * @param timeout_ms 超时时间（毫秒），0 = 立即返回
 * @return 实际读取的字节数，0 表示超时或无数据
 */
size_t audio_processor_read_timeout(audio_processor_t *audio_processor,
                                    void *buffer, size_t size, uint32_t timeout_ms)
{
    size_t size_read = 0;
    void *buf_read = xRingbufferReceive(audio_processor->enc_output,
                                        &size_read, pdMS_TO_TICKS(timeout_ms));
    if (!buf_read)
        return 0;

    if (size_read > size)
    {
        ESP_LOGW(TAG, "Buffer size is too small");
        size_read = size;
    }

    memcpy(buffer, buf_read, size_read);
    vRingbufferReturnItem(audio_processor->enc_output, buf_read);
    return size_read;
}
