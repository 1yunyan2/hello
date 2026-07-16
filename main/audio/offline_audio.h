#pragma once

/**
 * @file offline_audio.h
 * @brief 离线音频播放模块（外挂 flash → 解码 → 扬声器）
 *
 * 从外挂 SPI flash 的 FAT 分区（挂载点 /S）读取离线音频文件并播放，
 * 断网可用，与云端大模型对话链路完全独立（独立解码实例，不碰
 * audio_decoder.c 的云端实例与 RingBuffer 通道）。
 *
 * ── 支持格式（按扩展名自动分发）──────────────────────────────────────────
 *   .mp3 : MP3 文件（esp_audio_simple_dec 内置帧解析，任意分块喂入）
 *          素材要求：PC 端 ffmpeg 规范化为 16kHz 单声道
 *          ffmpeg -i in.mp3 -ar 16000 -ac 1 -b:a 32k out.mp3
 *   .p3  : 小智 P3 格式（4字节头 [1B类型,1B保留,2B长度大端] + OPUS 裸包）
 *          素材要求：xiaozhi-esp32/scripts/p3_tools/ 批量转换
 *          固定 16kHz 单声道 60ms 帧
 *
 * ── 喇叭仲裁（v1 从简策略）────────────────────────────────────────────────
 *   1. 仅在会话空闲（SESSION_IDLE）时允许播放，对话中触发返回
 *      ESP_ERR_INVALID_STATE（不排队、不打断 TTS）
 *   2. 播放途中若唤醒词命中开启会话，本模块检测到状态变化立即让路停播
 *   3. 离线音频之间：后触发的顶掉先播的（先 stop 再 play）
 *
 * ── 内存画像 ─────────────────────────────────────────────────────────────
 *   任务栈 32KB SPIRAM + 读缓冲 8KB SPIRAM + PCM 缓冲 8KB SPIRAM，
 *   解码器实例状态（~24KB）由组件内部 malloc 分配（≥4KB 自动走 PSRAM，
 *   见 sdkconfig CONFIG_SPIRAM_MALLOC_ALWAYSINTERNAL=4096），
 *   全部随播放结束释放，内部 SRAM 仅零碎小分配。
 *
 * ── 50 条语音编号规范（产线烧录）─────────────────────────────────────────
 *   路径：/S/voice/vNN.mp3 或 /S/voice/vNN.p3（NN = 00~49）
 *   编号 ↔ 触发场景映射表随产品定义补充维护于此：
 *     v00 : （待定义，如：开机欢迎）
 *     v01 : （待定义，如：断网提示）
 *     ...
 */

#include "esp_err.h"
#include <stdbool.h>

// ─── 编译期格式裁剪开关 ───────────────────────────────────────────────────────
// 置 0 可裁掉对应格式分支（省固件体积；MP3 解码器代码约 20~40KB）。
// 注意：两者都置 0 时模块退化为空壳，play 恒返回 ESP_ERR_NOT_SUPPORTED。
#ifndef OFFLINE_AUDIO_ENABLE_MP3
#define OFFLINE_AUDIO_ENABLE_MP3 1 ///< MP3 分支（简单方案：素材免转格式，仅需 PC 端重采样 16k 单声道）
#endif
#ifndef OFFLINE_AUDIO_ENABLE_P3
#define OFFLINE_AUDIO_ENABLE_P3 1 ///< P3/OPUS 分支（优化方案：空间省 ~27%，音质更好，需 PC 端转换）
#endif

/**
 * @brief 异步播放一条离线音频（创建独立播放任务，立即返回）
 *
 * 按扩展名自动选择解码分支（.mp3 / .p3，大小写不敏感）。
 * 若已有离线音频在播，先停掉旧的再播新的（顶掉语义）。
 *
 * @param path 音频文件绝对路径（如 "/S/voice/v00.mp3"）
 * @return
 *   - ESP_OK                  播放任务已创建
 *   - ESP_ERR_INVALID_STATE   会话非空闲（对话中），拒绝播放
 *   - ESP_ERR_INVALID_ARG     path 为空或扩展名不支持
 *   - ESP_ERR_NOT_SUPPORTED   对应格式分支被编译裁剪
 *   - ESP_ERR_NO_MEM          内存不足（路径副本/任务创建失败）
 *
 * @note 线程安全：可从任意任务调用（内部自旋等待旧任务退出，最长 ~600ms）
 * @note 文件不存在等 IO 错误在播放任务内检测，仅打日志（异步无法回传）
 */
esp_err_t offline_audio_play(const char *path);

/**
 * @brief 请求停止当前离线播放（异步，置停止标志后立即返回）
 *
 * 播放任务在下一个解码循环检测到标志后自行清理退出（毫秒级）。
 * 无播放进行时调用无副作用。
 */
void offline_audio_stop(void);

/**
 * @brief 查询是否有离线音频正在播放
 * @return true = 播放任务存活中
 */
bool offline_audio_is_playing(void);
