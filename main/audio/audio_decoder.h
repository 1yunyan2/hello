#pragma once

#include "freertos/FreeRTOS.h"

#include "freertos/ringbuf.h"

/**
 * @brief 音频解码器句柄类型定义
 */
typedef struct audio_decoder audio_decoder_t;

/**
 * @brief 创建音频解码器实例
 * @param sample_rate 采样率（单位：Hz）
 * @param channels 声道数
 * @return 返回创建的解码器句柄
 */
audio_decoder_t *audio_decoder_create(int sample_rate, int channels);

/**
 * @brief 销毁音频解码器实例
 * @param audio_decoder 解码器句柄
 */
void audio_decoder_destroy(audio_decoder_t *audio_decoder);

/* 给 decoder 设置输入和输出的缓冲区 */
/* 输入类型为 NOSPLIT，输出类型为字节型*/
/**
 * @brief 设置解码器的输入和输出缓冲区
 * @param audio_decoder 解码器句柄
 * @param input_buffer 输入环形缓冲区句柄
 * @param output_buffer 输出环形缓冲区句柄
 */
void audio_decoder_set_buffer(audio_decoder_t *audio_decoder, RingbufHandle_t input_buffer, RingbufHandle_t output_buffer);

/**
 * @brief 启动解码器任务
 * @param audio_decoder 解码器句柄
 */
void audio_decoder_start(audio_decoder_t *audio_decoder);

/**
 * @brief 停止解码器任务
 * @param audio_decoder 解码器句柄
 */
void audio_decoder_stop(audio_decoder_t *audio_decoder);
