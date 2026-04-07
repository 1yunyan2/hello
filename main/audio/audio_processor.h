#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct audio_processor audio_processor_t;

/**
 * @brief 创建音频处理器实例
 * 
 * 初始化音频处理器的核心数据结构，包括编码器、解码器和四个环形缓冲区。
 * 该实例是音频处理链路（麦克风→云端 和 云端→扬声器）的中枢协调者。
 * 
 * @return audio_processor_t* 成功返回处理器实例指针，失败返回NULL
 */
audio_processor_t *audio_processor_create(void);

/**
 * @brief 销毁音频处理器实例
 * 
 * 停止相关任务，释放所有动态分配的资源，包括内存和环形缓冲区。
 * 调用此函数后，不应再使用该实例指针。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 */
void audio_processor_destroy(audio_processor_t *audio_processor);

/**
 * @brief 启动音频处理器
 * 
 * 开启音频处理器内部的任务，使其开始处理PCM和OPUS数据流。
 * 必须在调用write_pcm或read等操作前启动处理器。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 */
void audio_processor_start(audio_processor_t *audio_processor);

/**
 * @brief 停止音频处理器
 * 
 * 停止音频处理器内部的所有任务，暂停数据处理流程。
 * 可用于会话结束时节省CPU和电源资源。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 */
void audio_processor_stop(audio_processor_t *audio_processor);

/**
 * @brief 从编码器输出缓冲区读取OPUS数据
 * 
 * 供网络发送任务调用，将编码后的OPUS音频帧读取到指定缓冲区以发送至云端。
 * 该函数会阻塞直至有数据可读或超时。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 * @param buffer 目标缓冲区，用于存放读取的OPUS数据
 * @param size 请求读取的最大字节数
 * @return size_t 实际读取的字节数
 */
size_t audio_processor_read(audio_processor_t *audio_processor, void *buffer, size_t size);

/**
 * @brief 将原始PCM数据写入编码器输入缓冲区
 * 
 * 接收来自麦克风采集任务的16-bit PCM数据，并将其送入编码器进行OPUS压缩。
 * 这是音频上行链路（设备→云端）的数据入口。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 * @param buffer 包含PCM数据的源缓冲区
 * @param size PCM数据的大小（字节）
 */
void audio_processor_write_pcm(audio_processor_t *audio_processor, void *buffer, size_t size);

/**
 * @brief 将OPUS数据写入解码器输入缓冲区
 * 
 * 接收来自云端的OPUS音频帧，并将其送入解码器进行PCM还原。
 * 这是音频下行链路（云端→设备）的数据入口。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 * @param buffer 包含OPUS数据的源缓冲区
 * @param size OPUS数据的大小（字节）
 */
void audio_processor_write(audio_processor_t *audio_processor, void *buffer, size_t size);

/**
 * @brief 清空编码器输出缓冲区
 * 
 * 移除缓冲区内所有积压的OPUS音频帧。在多轮对话切换时调用，
 * 以清除上一轮对话残留的语音数据，防止误发。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 */
void audio_processor_flush_output(audio_processor_t *audio_processor);

// 带超时的 OPUS 读取（供发送任务使用，避免 portMAX_DELAY 死锁）
// timeout_ms = 0 立即返回，返回 0 表示超时/无数据
/**
 * @brief 带超时的OPUS数据读取
 * 
 * 类似于audio_processor_read，但提供了超时机制，避免在无数据时永久阻塞，
 * 防止任务死锁，特别适用于需要响应外部事件的发送循环。
 * 
 * @param audio_processor 指向音频处理器实例的指针
 * @param buffer 目标缓冲区，用于存放读取的OPUS数据
 * @param size 请求读取的最大字节数
 * @param timeout_ms 超时时间（毫秒），0表示立即返回
 * @return size_t 实际读取的字节数，0表示超时或无数据
 */
size_t audio_processor_read_timeout(audio_processor_t *audio_processor,
                                    void *buffer, size_t size, uint32_t timeout_ms);