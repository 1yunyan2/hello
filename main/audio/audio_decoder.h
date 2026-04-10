#pragma once

/**
 * @file audio_decoder.h
 * @brief OPUS 音频解码器模块接口（OPUS → PCM）
 *
 * 本模块将从云端接收的 OPUS 压缩音频帧解码为 16-bit PCM，
 * 通过 RingBuffer 与上下游解耦：
 *
 *   上游（WebSocket 接收回调）                        下游（play_task）
 *   write → [dec_input: NOSPLIT 5KB] → audio_decoder_task → [dec_output: BYTEBUF 40KB] → 扬声器
 *
 * 解码参数（必须与云端编码端保持一致）：
 *   采样率  : 16000 Hz
 *   帧时长  : 60 ms（每帧 960 采样点 = 1920 字节 PCM，与云端对齐）
 *   声道数  : 1（单声道）
 *   格式    : 标准 OPUS 封包（self_delimited = false）
 *
 * @note 解码器任务绑定 CPU0，栈分配在 SPIRAM
 * @note dec_input 使用 NOSPLIT 类型，确保 OPUS 帧整帧读取（不跨块分割）
 * @note dec_output 使用 BYTEBUF 类型，PCM 数据可自由分段读取
 */

#include "freertos/FreeRTOS.h"
#include "freertos/ringbuf.h"

/**
 * @brief 解码器句柄（不透明类型）
 *
 * 内部结构在 audio_decoder.c 中定义（struct audio_decoder），
 * 对外只暴露指针，隐藏实现细节。
 *
 * 内部字段（仅供参考）：
 *   - input_buffer  : RingbufHandle_t（NOSPLIT，OPUS 帧输入）
 *   - output_buffer : RingbufHandle_t（BYTEBUF，PCM 输出）
 *   - dec            : esp_audio_dec_handle_t（OPUS 解码器句柄）
 *   - sample_rate    : int（采样率，计算输出缓冲区大小用）
 *   - channels       : int（声道数）
 *   - is_running     : bool（任务运行标志）
 */
typedef struct audio_decoder audio_decoder_t;

/**
 * @brief 创建 OPUS 解码器实例（注册并打开 OPUS 解码器）
 *
 * 内部步骤：
 *   1. malloc_zeroed 分配 audio_decoder_t 结构体
 *   2. 保存采样参数（play_task 计算 PCM 输出大小时使用）
 *   3. esp_opus_dec_register() 向框架注册 OPUS 解码器类型
 *   4. 配置解码参数（60ms 帧 / 非自分界格式）
 *   5. esp_audio_dec_open() 打开解码器，获取句柄
 *
 * @param sample_rate 采样率（Hz），必须与编码端一致（16000）
 * @param channels    声道数，必须与编码端一致（1 = 单声道）
 * @return audio_decoder_t* 解码器实例指针，失败返回 NULL
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 * @note 解码参数必须与 audio_encoder.c 中的编码参数严格一致，否则解码失败
 */
audio_decoder_t *audio_decoder_create(int sample_rate, int channels);

/**
 * @brief 销毁解码器实例，释放所有资源
 *
 * 步骤：esp_audio_dec_close() → esp_audio_dec_unregister() → free()
 *
 * @param audio_decoder 解码器实例指针（调用后不可再使用）
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_destroy()
 * @note 调用前应先调用 audio_decoder_stop() 确保任务已停止
 */
void audio_decoder_destroy(audio_decoder_t *audio_decoder);

/**
 * @brief 绑定输入和输出环形缓冲区
 *
 * 必须在 audio_decoder_start() 之前调用。
 *
 * 缓冲区类型约束：
 *   input_buffer  : RINGBUF_TYPE_NOSPLIT（OPUS 帧必须整帧读取，不可被拆分跨块）
 *   output_buffer : RINGBUF_TYPE_BYTEBUF（PCM 数据无帧边界，可自由分段读取）
 *
 * @param audio_decoder  解码器实例指针
 * @param input_buffer   OPUS 帧输入缓冲区句柄（由 audio_processor_create 创建）
 * @param output_buffer  PCM 输出缓冲区句柄（由 audio_processor_create 创建）
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
void audio_decoder_set_buffer(audio_decoder_t *audio_decoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer);

/**
 * @brief 启动解码任务（设置 is_running=true，创建 FreeRTOS 任务）
 *
 * 任务配置：
 *   - 名称：decoder_task
 *   - CPU 核心：0
 *   - 栈大小：32KB（SPIRAM 分配）
 *   - 优先级：5
 *
 * 任务主循环：
 *   while(is_running) {
 *     从 dec_input 读一整个 OPUS 帧（100ms 超时）
 *     esp_audio_dec_process() 解码 → 写入 dec_output（满则丢帧）
 *   }
 *
 * @param audio_decoder 解码器实例指针（已绑定缓冲区）
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_start()
 * @note 前置条件：audio_decoder_set_buffer() 已调用
 */
void audio_decoder_start(audio_decoder_t *audio_decoder);

/**
 * @brief 停止解码任务（设置 is_running=false，等待任务自然退出）
 *
 * 清除 is_running 标志后等待 200ms，让任务从 xRingbufferReceive 超时返回并退出。
 *
 * @param audio_decoder 解码器实例指针
 * @return void
 *
 * @note 调用者：audio_processor.c → audio_processor_stop()
 */
void audio_decoder_stop(audio_decoder_t *audio_decoder);
