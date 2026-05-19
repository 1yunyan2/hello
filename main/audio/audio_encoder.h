#pragma once

/**
 * @file audio_encoder.h
 * @brief OPUS 音频编码器模块接口（PCM → OPUS）
 *
 * 本模块将麦克风采集的原始 16-bit PCM 数据编码为 OPUS 压缩格式，
 * 通过 RingBuffer 与上下游解耦：
 *
 *   上游（audio_feed_task）                  下游（ws_sender_task）
 *   write_pcm → [enc_input: BYTEBUF 20KB] → audio_encoder_task → [enc_output: NOSPLIT 8KB] → 发送
 *
 * 编码参数（在 audio_encoder_create() 中配置）：
 *   采样率  : 16000 Hz
 *   比特率  : 24 kbps（VoIP 场景优化）
 *   帧时长  : 20 ms（每帧 320 采样点 = 640 字节 PCM）
 *   复杂度  : 0（最低，避开 SILK delayed-decision NSQ 路径，CPU 负载更平稳）
 *   模式    : OPUS_APPLICATION_VOIP（针对语音优化）
 *   FEC/DTX/VBR : 全部禁用（保证延迟稳定性）
 *
 * @note 编码器任务绑定 CPU1（与 audio_feed_task 同核），栈分配在 SPIRAM（节省内部 SRAM）
 */

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

/**
 * @brief 编码器句柄（不透明类型）
 *
 * 内部结构在 audio_encoder.c 中定义（struct audio_encoder），
 * 对外只暴露指针，隐藏实现细节（C 语言的"类"封装模式）。
 *
 * 内部字段（仅供参考）：
 *   - input_buffer  : RingbufHandle_t（BYTEBUF，PCM 输入）
 *   - output_buffer : RingbufHandle_t（NOSPLIT，OPUS 输出）
 *   - enc            : esp_audio_enc_handle_t（OPUS 编码器句柄）
 *   - is_running     : volatile bool（任务运行标志，跨任务可见）
 *   - in_buf/out_buf : 帧缓冲（16 字节对齐 SPIRAM，在 start 中分配 stop 中释放）
 */
typedef struct audio_encoder audio_encoder_t;

/**
 * @brief 创建 OPUS 编码器实例（注册并打开 OPUS 编码器）
 *
 * 内部步骤：
 *   1. malloc_zeroed 分配 audio_encoder_t 结构体
 *   2. esp_opus_enc_register() 向框架注册 OPUS 编码器类型
 *   3. 配置 OPUS 参数（24kbps / 20ms帧 / 复杂度3 / VOIP 模式）
 *   4. esp_audio_enc_open() 打开编码器，获取句柄
 *
 * @param sample_rate 采样率（Hz），应与 BSP_CODEC_SAMPLE_RATE 一致（16000）
 * @param channels    声道数，应为 1（单声道）
 * @return audio_encoder_t* 编码器实例指针，失败返回 NULL
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 * @note 必须在 audio_encoder_set_buffer() 之前调用
 */
audio_encoder_t *audio_encoder_create(int sample_rate, int channels);

/**
 * @brief 销毁编码器实例，释放所有资源
 *
 * 步骤：esp_audio_enc_close() → esp_audio_enc_unregister() → free()
 *
 * @param audio_encoder 编码器实例指针（调用后不可再使用）
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_destroy()
 * @note 调用前应先调用 audio_encoder_stop() 确保任务已停止
 */
void audio_encoder_destroy(audio_encoder_t *audio_encoder);

/**
 * @brief 绑定输入和输出环形缓冲区
 *
 * 必须在 audio_encoder_start() 之前调用，否则任务启动时缓冲区为 NULL 会警告退出。
 *
 * 缓冲区类型约束（在 audio_processor.c 中创建时强制）：
 *   input_buffer  : RINGBUF_TYPE_BYTEBUF（PCM 数据可自由拼接，无帧边界）
 *   output_buffer : RINGBUF_TYPE_NOSPLIT（OPUS 帧必须保持整帧完整性，不可跨块）
 *
 * @param audio_encoder  编码器实例指针
 * @param input_buffer   PCM 输入缓冲区句柄（由 audio_processor_create 创建）
 * @param output_buffer  OPUS 输出缓冲区句柄（由 audio_processor_create 创建）
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_create()（绑定关系建立）
 */
void audio_encoder_set_buffer(audio_encoder_t *audio_encoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer);

/**
 * @brief 启动编码任务（设置 is_running=true，创建 FreeRTOS 任务）
 *
 * 任务配置：
 *   - 名称：encoder_task
 *   - CPU 核心：1（与 audio_feed_task 同核，编码与采集共核靠让步调度）
 *   - 栈大小：32KB（SPIRAM 分配，OPUS 运算需要较大栈空间）
 *   - 优先级：5
 *
 * 任务主循环：
 *   while(is_running) {
 *     从 input_buffer 分段读 PCM，凑满一帧（100ms 超时）
 *     esp_audio_enc_process() 编码 → 写入 output_buffer（满则丢帧）
 *   }
 *
 * @param audio_encoder 编码器实例指针（已绑定缓冲区）
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_start()
 * @note 前置条件：audio_encoder_set_buffer() 已调用
 */
void audio_encoder_start(audio_encoder_t *audio_encoder);

/**
 * @brief 停止编码任务（设置 is_running=false，等待任务自然退出）
 *
 * 清除 is_running 标志后等待 200ms，让任务有机会从 xRingbufferReceiveUpTo
 * 超时返回并退出循环，然后自删除。
 *
 * @param audio_encoder 编码器实例指针
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_stop()
 * @note 停止后缓冲区中可能仍有数据，需要手动 flush 或重建缓冲区
 */
void audio_encoder_stop(audio_encoder_t *audio_encoder);
