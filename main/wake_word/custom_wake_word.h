
#pragma once // 防止重复定义

#include <stdint.h>
#include <stddef.h>
#include "esp_err.h"
#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "model_path.h"
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"
#include "esp_vad.h"            // vad_state_t: VAD_SILENCE / VAD_SPEECH
#include "esp_afe_sr_iface.h"   // AFE 音频前端（NS 降噪 + VAD）
#include "esp_afe_sr_models.h"  // esp_afe_handle_from_config

// 唤醒词触发回调，参数为显示文字（如"云炎"或"Hello Echo"）
typedef void (*wake_word_detected_cb_t)(const char *wake_word_display);

/**
 * @brief 初始化唤醒词引擎，从 NVS 恢复上次保存的唤醒词和语言
 */
esp_err_t bsp_wake_word_init(wake_word_detected_cb_t cb);

/**
 * @brief 从 NVS 读取命令词到 dest（内部使用，供外部诊断调用）
 */
void bsp_wake_word_load_from_nvs(char *dest, size_t max_len);

/**
 * @brief 更新唤醒词（MQTT 收到指令后调用）
 *
 * 语言自动从 wake_word_display 检测：含汉字→中文模型，纯 ASCII→英文模型。
 * 英文命令词自动转大写，无需调用方处理。
 *
 * @param wake_word_display  显示文字，如 "云炎" 或 "Hello Echo"
 * @param wake_word_pinyin   命令词，中文用拼音 "yun yan"，英文用单词 "hello echo"
 */
esp_err_t wake_word_update(const char *wake_word_display, const char *wake_word_pinyin);

/**
 * @brief 获取 MultiNet 每次需要的音频采样点数量（一般为 512）
 */
size_t custom_wake_word_get_chunksize(void);

/**
 * @brief 获取 AFE feed 每次需要的采样点数量
 * audio_feed_task 必须按此大小投喂原始 PCM，否则 AFE 内部会报错
 * @return 采样点数（非字节数），0 表示 AFE 未初始化
 */
size_t custom_wake_word_get_feed_chunksize(void);

/**
 * @brief 将麦克风采集的 16-bit PCM 数据喂给引擎
 */
void custom_wake_word_feed(const int16_t *data, size_t len);

/**
 * @brief 停止引擎监听（MultiNet 停止，AFE/VAD 继续运行）
 */
void bsp_wake_word_stop(void);

/**
 * @brief 恢复引擎监听
 */
void bsp_wake_word_start(void);

// ─── VAD / 增强 PCM 接口 ────────────────────────────────────────────────

/**
 * @brief VAD 状态变化回调（从 AFE fetch 任务调用，勿在回调内阻塞）
 * @param state  VAD_SPEECH = 检测到语音，VAD_SILENCE = 静音
 */
typedef void (*vad_state_cb_t)(vad_state_t state);

/**
 * @brief 增强 PCM 数据钩子（AFE 降噪后输出，供会话模块送编码器）
 * @param data     降噪后 16-bit PCM 数据
 * @param samples  采样点数量
 */
typedef void (*enhanced_pcm_cb_t)(const int16_t *data, size_t samples);

/**
 * @brief 注册 VAD 状态变化回调（传 NULL = 注销）
 */
void bsp_wake_word_set_vad_callback(vad_state_cb_t cb);

/**
 * @brief 注册增强 PCM 钩子（传 NULL = 注销）
 */
void bsp_wake_word_set_enhanced_pcm_hook(enhanced_pcm_cb_t hook);

/**
 * @brief 查询当前 VAD 状态
 */
vad_state_t bsp_wake_word_get_vad_state(void);
