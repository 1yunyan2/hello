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

#define AUDIO_PROCESSOR_TASK_STACK_SIZE (4 * 1024)
#define AUDIO_PROCESSOR_TASK_PRIORITY 5
#define AUDIO_PROCESSOR_TASK_CORE_ID 0

struct audio_processor
{
    audio_encoder_t *encoder;
    audio_decoder_t *decoder;

    RingbufHandle_t enc_input;
    RingbufHandle_t enc_output;
    RingbufHandle_t dec_input;
    RingbufHandle_t dec_output;

    volatile bool is_running;
    TaskHandle_t play_task_handle; // 播放任务句柄
};

static void audio_processor_play_task(void *arg)
{
    audio_processor_t *audio_processor = (audio_processor_t *)arg;
    bsp_board_t *board = bsp_board_get_instance();

    while (audio_processor->is_running)
    {
        size_t size_read = 0;
        // 使用 100ms 超时，确保能响应 is_running 置 false
        void *buf = xRingbufferReceiveUpTo(audio_processor->dec_output, &size_read,
                                           pdMS_TO_TICKS(100), 2048);
        if (buf)
        {
            esp_codec_dev_write(board->codec_dev, buf, size_read);
            vRingbufferReturnItem(audio_processor->dec_output, buf);
        }
    }

    audio_processor->play_task_handle = NULL;
    vTaskDelete(NULL);
}

audio_processor_t *audio_processor_create(void)
{
    audio_processor_t *audio_processor = (audio_processor_t *)malloc_zeroed(sizeof(audio_processor_t));

    // hello 是单声道硬件采集，配置单声道编解码
    audio_processor->encoder = audio_encoder_create(BSP_CODEC_SAMPLE_RATE, 1);
    audio_processor->decoder = audio_decoder_create(BSP_CODEC_SAMPLE_RATE, 1);

    //! pcm是字节缓存，配置单字节编解码，opus是帧数据，不拆分缓存
    audio_processor->enc_input = xRingbufferCreateWithCaps(20480, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);
    audio_processor->enc_output = xRingbufferCreateWithCaps(2560, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    audio_processor->dec_input = xRingbufferCreateWithCaps(5120, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    audio_processor->dec_output = xRingbufferCreateWithCaps(40960, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);

    audio_encoder_set_buffer(audio_processor->encoder, audio_processor->enc_input, audio_processor->enc_output);
    audio_decoder_set_buffer(audio_processor->decoder, audio_processor->dec_input, audio_processor->dec_output);

    return audio_processor;
}

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

void audio_processor_start(audio_processor_t *audio_processor)
{
    audio_processor->is_running = true;
    audio_encoder_start(audio_processor->encoder);
    audio_decoder_start(audio_processor->decoder);

    xTaskCreatePinnedToCoreWithCaps(audio_processor_play_task, "play_task",
                                    AUDIO_PROCESSOR_TASK_STACK_SIZE, audio_processor,
                                    AUDIO_PROCESSOR_TASK_PRIORITY,
                                    &audio_processor->play_task_handle,
                                    AUDIO_PROCESSOR_TASK_CORE_ID, MALLOC_CAP_SPIRAM);
}

void audio_processor_stop(audio_processor_t *audio_processor)
{
    audio_processor->is_running = false;
    audio_encoder_stop(audio_processor->encoder);
    audio_decoder_stop(audio_processor->decoder);
    // 等待 play_task 退出（最多 300ms）
    for (int i = 0; i < 3 && audio_processor->play_task_handle != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(100));
}

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

void audio_processor_write_pcm(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    // 将采集的原始PCM推入编码器缓存区
    xRingbufferSend(audio_processor->enc_input, buffer, size, portMAX_DELAY);
}

void audio_processor_write(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    xRingbufferSend(audio_processor->dec_input, buffer, size, portMAX_DELAY);
}

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
