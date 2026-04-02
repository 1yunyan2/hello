#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h" //环形缓冲区

/**
 * @brief 音频编码器句柄类型定义
 */
typedef struct audio_encoder audio_encoder_t;

/**
 * @brief 创建音频编码器实例
 * @param sample_rate 采样率（单位：Hz）
 * @param channels 声道数
 * @return 返回创建的编码器句柄
 */
audio_encoder_t *audio_encoder_create(int sample_rate, int channels);

/**
 * @brief 销毁音频编码器实例
 * @param audio_encoder 编码器句柄
 */
void audio_encoder_destroy(audio_encoder_t *audio_encoder);

/* 给 encoder 设置输入和输出的缓冲区 */
/* 输入类型为字节型，因为是pcm格式数据，所以可以自由拼接数据大小；输出类型为 NoSPLIT 型，opus格式数据有帧大小，所以不可以拆分*/
/**
 * @brief 设置编码器的输入和输出缓冲区
 * @param audio_encoder 编码器句柄
 * @param input_buffer 输入环形缓冲区句柄
 * @param output_buffer 输出环形缓冲区句柄
 */
void audio_encoder_set_buffer(audio_encoder_t *audio_encoder, RingbufHandle_t input_buffer, RingbufHandle_t output_buffer);

/**
 * @brief 启动编码器任务
 * @param audio_encoder 编码器句柄
 */
void audio_encoder_start(audio_encoder_t *audio_encoder);

/**
 * @brief 停止编码器任务
 * @param audio_encoder 编码器句柄
 */
void audio_encoder_stop(audio_encoder_t *audio_encoder);
