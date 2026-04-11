#pragma once

/**
 * @file audio_processor.h
 * @brief 音频处理器模块接口 — 编解码管道中枢
 *
 * 本模块是音频上行（麦克风→云端）和下行（云端→扬声器）的数据枢纽，
 * 内部封装 OPUS 编码器、解码器和四个环形缓冲区，对外提供统一的读写接口。
 *
 * 数据流架构（四个环形缓冲区）：
 *
 *   上行（录音链路）：
 *     麦克风 PCM → audio_processor_write_pcm() → [enc_input: BYTEBUF 20KB]
 *     → audio_encoder_task（OPUS 编码）→ [enc_output: NOSPLIT 8KB]
 *     → audio_processor_read_timeout() → WebSocket 发送任务 → 云端
 *
 *   下行（播放链路）：
 *     云端 → WebSocket 接收回调 → audio_processor_write() → [dec_input: NOSPLIT 16KB]
 *     → audio_decoder_task（OPUS 解码）→ [dec_output: BYTEBUF 40KB]
 *     → audio_processor_play_task → I2S TX → ES8311 → 扬声器
 *
 * 缓冲区类型选择说明：
 *   BYTEBUF（字节流）：PCM 数据无帧边界，可自由拼接和分段读取
 *   NOSPLIT（不拆分）：OPUS 帧必须整帧写入/读取，禁止跨块拆分
 *
 * @note 调用者：session.c（会话启动时创建，会话结束时销毁）
 */

#include <stddef.h>
#include <stdint.h>

/** @brief 音频处理器句柄（不透明类型，内部结构在 audio_processor.c 中定义） */
typedef struct audio_processor audio_processor_t;

/**
 * @brief 创建音频处理器实例
 *
 * 依次创建 OPUS 编码器、OPUS 解码器和四个环形缓冲区，
 * 并将缓冲区绑定到对应编解码器（绑定完成后等待 audio_processor_start 启动任务）。
 * 任一环形缓冲区创建失败则整体回滚，不返回半初始化对象。
 *
 * @param 无
 * @return audio_processor_t* 成功返回处理器实例指针，失败返回 NULL
 *
 * @note 调用者：session.c → session_on_wake_word()（新会话启动时调用）
 * @note 创建后必须调用 audio_processor_start() 才能开始处理数据
 */
audio_processor_t *audio_processor_create(void);

/**
 * @brief 销毁音频处理器实例，释放所有资源
 *
 * 依次删除四个环形缓冲区、编码器实例、解码器实例和结构体本身。
 * 调用前必须先调用 audio_processor_stop()，确保内部任务已退出。
 *
 * @param audio_processor 音频处理器实例指针（调用后不可再使用）
 * @return void
 *
 * @note 调用者：session.c → session_close()（会话关闭时调用）
 * @note 调用前必须先调用 audio_processor_stop()，否则会有悬挂任务
 */
void audio_processor_destroy(audio_processor_t *audio_processor);

/**
 * @brief 启动音频处理器（依次启动编码器任务、解码器任务和播放任务）
 *
 * 设置 is_running 标志后，创建三个 FreeRTOS 任务：
 *   - encoder_task（CPU0，32KB SPIRAM 栈，优先级 5）
 *   - decoder_task（CPU0，32KB SPIRAM 栈，优先级 5）
 *   - play_task   （CPU0，4KB SPIRAM 栈，优先级 5）
 *
 * @param audio_processor 音频处理器实例指针（已通过 audio_processor_create 创建）
 * @return void
 *
 * @note 调用者：session.c → session_on_wake_word()（会话开始后立即启动）
 * @note 必须在 audio_processor_create() 之后调用
 */
void audio_processor_start(audio_processor_t *audio_processor);

/**
 * @brief 停止音频处理器（按顺序安全停止三个任务）
 *
 * 停止顺序（顺序不可调换，保证 TTS 末尾不截断）：
 *   1. 停止编码器任务（停止上行录音）
 *   2. 等待解码器排水（把 dec_input 剩余 OPUS 帧全部解码写入 dec_output）
 *   3. 停止播放任务（等待 dec_output 清空后自然退出）
 *
 * @param audio_processor 音频处理器实例指针
 * @return void
 *
 * @note 调用者：session.c → session_close()（会话关闭时调用）
 * @note 此函数可能阻塞最长约 30 秒（等待解码器排水超时）
 */
void audio_processor_stop(audio_processor_t *audio_processor);

/**
 * @brief 从编码器输出缓冲区读取 OPUS 数据（永久阻塞）
 *
 * 从 enc_output（NOSPLIT 环形缓冲区）中读取一整个 OPUS 帧，
 * 复制到 buffer 后返回实际字节数。若 enc_output 为空则阻塞等待。
 *
 * @param audio_processor 音频处理器实例指针
 * @param buffer          目标缓冲区（调用方分配，大小应至少为 512 字节）
 * @param size            目标缓冲区大小（字节）
 * @return size_t         实际读取的 OPUS 帧字节数，0 表示读取失败
 *
 * @note 调用者：session.c → ws_sender_task()（WebSocket 发送任务）
 * @note 此函数永久阻塞，推荐在需要超时的场景使用 audio_processor_read_timeout()
 */
size_t audio_processor_read(audio_processor_t *audio_processor, void *buffer, size_t size);

/**
 * @brief 将麦克风采集的 PCM 数据写入编码器输入缓冲区（上行链路数据入口）
 *
 * 将 AFE 降噪后的 16-bit PCM 帧写入 enc_input（BYTEBUF）。
 * 内部阻塞等待（portMAX_DELAY），直到缓冲区有足够空间。
 *
 * @param audio_processor 音频处理器实例指针
 * @param buffer          PCM 数据源缓冲区（16-bit 样本，来自 on_enhanced_pcm 钩子）
 * @param size            PCM 数据字节数（= 采样点数 × 2）
 * @return void
 *
 * @note 调用者：session.c → on_enhanced_pcm()（AFE 降噪后的 PCM 钩子回调）
 * @note 此函数会阻塞，在 enc_input 缓冲区满时等待编码器消费
 */
void audio_processor_write_pcm(audio_processor_t *audio_processor, void *buffer, size_t size);

/**
 * @brief 将云端下发的 OPUS 数据写入解码器输入缓冲区（下行链路数据入口）
 *
 * 将 WebSocket 收到的一整个 OPUS 帧写入 dec_input（NOSPLIT）。
 * 内部阻塞等待（portMAX_DELAY），直到缓冲区有足够空间（背压机制）。
 *
 * @param audio_processor 音频处理器实例指针
 * @param buffer          OPUS 帧数据指针（来自 WebSocket Binary Frame）
 * @param size            OPUS 帧字节数
 * @return void
 *
 * @note 调用者：session.c → protocol_event_handler()（PROTOCOL_EVENT_AUDIO 事件）
 * @note NOSPLIT 类型要求整帧写入，buffer 指向的必须是完整的一帧 OPUS 数据
 */
void audio_processor_write(audio_processor_t *audio_processor, void *buffer, size_t size);

/**
 * @brief 清空解码器输入和输出缓冲区（立即停止扬声器输出）
 *
 * 非阻塞快速清空 dec_input（未解码的 OPUS 帧）和 dec_output（已解码未播放的 PCM）。
 * 用于语音打断场景：用户说唤醒词中断 AI 讲话，立即清除积压的 TTS 音频。
 *
 * @param audio_processor 音频处理器实例指针
 * @return void
 *
 * @note 调用者：session.c → session_on_wake_word()（打断 TTS 时调用）
 * @note 打断后解码器排水（drain）立即完成，audio_processor_stop() 不会延迟
 */
void audio_processor_flush_output(audio_processor_t *audio_processor);

/**
 * @brief 带超时的 OPUS 数据读取（供发送任务使用，避免永久阻塞）
 *
 * 类似 audio_processor_read，但超过 timeout_ms 后返回 0 而不是无限等待。
 * ws_sender_task 使用此函数，超时后可检查会话状态（如是否已关闭）再决定是否继续。
 *
 * @param audio_processor 音频处理器实例指针
 * @param buffer          目标缓冲区（调用方分配）
 * @param size            目标缓冲区大小（字节）
 * @param timeout_ms      超时时间（毫秒）：0 = 立即返回，无需等待
 * @return size_t         实际读取的 OPUS 帧字节数，0 表示超时或无数据
 *
 * @note 调用者：session.c → ws_sender_task()（WebSocket 发送主循环）
 * @note 内部使用 pdMS_TO_TICKS(timeout_ms) 换算超时
 */
size_t audio_processor_read_timeout(audio_processor_t *audio_processor,
                                    void *buffer, size_t size, uint32_t timeout_ms);

/**
 * @brief 读取 AEC 回声消除参考 PCM（当前扬声器正在播放的音频副本）
 *
 * play_task 写入 I2S 时同步向 aec_ref_buf 推送一份副本，
 * 本函数由 custom_wake_word_feed 内部通过 aec_ref_cb 间接调用，
 * 为 AFE 的 AEC 算法提供参考信号，从而从麦克风信号中消除回声。
 *
 * 非阻塞：若缓冲区数据不足（会话未启动或播放静音），其余部分用零填充。
 * 零参考 ＝ AEC 不做任何消减 → 安全退化为无回声消除模式。
 *
 * @param audio_processor 音频处理器实例指针
 * @param buf             输出缓冲区（调用方分配，int16_t，大小 = samples）
 * @param samples         需要读取的采样点数量（通常等于 AFE feed chunksize）
 * @return void
 *
 * @note 调用者：session.c 注册的 aec_ref_provider 回调
 * @note 线程安全：FreeRTOS ring buffer 内部加锁，CPU0（play_task）写，CPU1（feed_task）读
 */
void audio_processor_read_ref_pcm(audio_processor_t *audio_processor,
                                  int16_t *buf, size_t samples);