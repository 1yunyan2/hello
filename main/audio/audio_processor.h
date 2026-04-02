#pragma once

#include <stddef.h>
#include <stdint.h>

typedef struct audio_processor audio_processor_t;

audio_processor_t *audio_processor_create(void);
void audio_processor_destroy(audio_processor_t *audio_processor);
void audio_processor_start(audio_processor_t *audio_processor);
void audio_processor_stop(audio_processor_t *audio_processor);

// 读取 OPUS 编码数据（准备发往云端）
size_t audio_processor_read(audio_processor_t *audio_processor, void *buffer, size_t size);

// 写入外部 PCM 数据给编码器（如麦克风采集的数据）
void audio_processor_write_pcm(audio_processor_t *audio_processor, void *buffer, size_t size);

// 写入 OPUS 数据供本地解码和播放（从云端接收的数据）
void audio_processor_write(audio_processor_t *audio_processor, void *buffer, size_t size);

// 带超时的 OPUS 读取（供发送任务使用，避免 portMAX_DELAY 死锁）
// timeout_ms = 0 立即返回，返回 0 表示超时/无数据
size_t audio_processor_read_timeout(audio_processor_t *audio_processor,
                                    void *buffer, size_t size, uint32_t timeout_ms);