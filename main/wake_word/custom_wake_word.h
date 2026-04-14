
#pragma once // 防止重复包含

/**
 * @file custom_wake_word.h
 * @brief 自定义唤醒词引擎接口（MultiNet6 + AFE 音频前端）
 *
 * 本模块封装了 ESP-SR MultiNet6 命令词检测引擎和 AFE（Audio Front End）音频前端，
 * 提供完整的唤醒词检测能力：
 *
 * 数据流：
 *   麦克风(原始PCM) → audio_feed_task → custom_wake_word_feed()
 *   → AFE(NS 降噪 + VAD 检测)
 *   → enhanced_pcm_hook（降噪后 PCM → 编码器 → 云端）
 *   → MultiNet6 detect（命令词检测）
 *   → wake_word_detected_cb_t（触发回调 → session_on_wake_word）
 *
 * 功能特性：
 *   - 支持中文（mn6_cn）和英文（mn6_en）两种语言模型，运行时热切换
 *   - 支持通过 MQTT 动态更新唤醒词（wake_word_update），持久化到 NVS
 *   - AFE 集成 NS 噪声抑制（消除背景噪音）和 VAD 语音活动检测
 *   - 唤醒词和语言设置断电保持（NVS "sys_config" 命名空间）
 *
 * 默认唤醒词：
 *   中文："你好伙伴" (拼音: ni hao huo ban)
 *   英文："Hello Echo"
 *
 * @note wake_word_init() 必须在 audio_init() 之前调用（采集任务需要 feed_chunksize）
 * @note MultiNet6 命令词要求至少 2 个音节/单词，否则初始化时自动回退到默认词
 */

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
#include "esp_vad.h"           // vad_state_t: VAD_SILENCE / VAD_SPEECH
#include "esp_afe_sr_iface.h"  // AFE 音频前端（NS 降噪 + VAD）
#include "esp_afe_sr_models.h" // esp_afe_handle_from_config

/**
 * @brief 唤醒词触发回调函数类型
 *
 * MultiNet6 检测到命令词时，从 afe_fetch_task 中调用此回调。
 * 回调函数不应在内部阻塞（会阻塞 AFE fetch 任务导致环形缓冲区积压）。
 *
 * @param wake_word_display 唤醒词显示文字（如 "你好伙伴" 或 "Hello Echo"）
 */
typedef void (*wake_word_detected_cb_t)(const char *wake_word_display);

/**
 * @brief 初始化唤醒词引擎（AFE + MultiNet6），从 NVS 恢复上次保存的唤醒词和语言
 *
 * 内部步骤：
 *   1. 创建互斥锁（保护 input_buffer 并发访问）
 *   2. 扫描 SPIFFS "model" 分区，建立模型文件列表
 *   3. 从 NVS 读取上次使用的显示词，判断语言（中/英）
 *   4. 初始化 AFE：单麦配置（M），开启 NS + VAD，关闭 AEC/SE/AGC
 *   5. 加载对应语言的 MultiNet6 模型（3s 检测窗口）
 *   6. 从 NVS 加载命令词，英文自动转大写，词数不足 2 时回退默认值
 *   7. 注册命令词到模型，编译 FST（有限状态转换器）
 *   8. 启动 afe_fetch_task（绑定 CPU1，优先级 5）
 *
 * @param cb 唤醒词触发回调（检测到命令词时从 afe_fetch_task 调用）
 * @return ESP_OK 成功；ESP_FAIL 分区未找到或内存不足
 *
 * @note 调用者：application.c → application_init()（步骤 3，在 audio_init() 之前）
 * @note 前置条件：NVS 已初始化（bsp_board_nvs_init 已调用）
 */
esp_err_t wake_word_init(wake_word_detected_cb_t cb);

/**
 * @brief 从 NVS 读取保存的命令词到 dest（供外部诊断或日志调用）
 *
 * 根据当前显示词语言（中/英）选择对应回退默认值，
 * 若 NVS 中无记录则填入对应语言的出厂默认命令词。
 *
 * @param dest    目标字符串缓冲区（调用方分配）
 * @param max_len 缓冲区大小（字节），建议 ≥ 64
 * @return void
 *
 * @note 调用者：wake_word_init()（初始化时内部调用）
 *
 */
void wake_word_load_from_nvs(char *dest, size_t max_len);

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
 * @brief 获取 MultiNet 每次 detect() 所需的采样点数量（通常为 512）
 *
 * @return size_t 采样点数（mn6 通常为 512 = 32ms @ 16kHz），0 表示模型未就绪
 *
 * @note 调用者：custom_wake_word_feed()（判断何时凑够一帧送 detect）
 */
size_t custom_wake_word_get_chunksize(void);

/**
 * @brief 获取 AFE feed 每次所需的采样点数量
 *
 * audio_feed_task 必须严格按此大小投喂原始 PCM，否则 AFE 内部 ringbuffer 会报错。
 * 此值由 AFE 内部配置决定（通常为 s_afe_feed_chunksize，约 512~1024）。
 *
 * @return size_t 每次 feed 所需采样点数（非字节数），0 表示 AFE 未初始化
 *
 * @note 调用者：bsp_codec.c → audio_feed_task()（初始化采集缓冲区大小）
 */
size_t custom_wake_word_get_feed_chunksize(void);

/**
 * @brief 将麦克风采集的原始 PCM 数据投喂给 AFE 音频前端引擎
 *
 * 将 I2S 采集到的 PCM 数据送入 AFE（s_afe_iface->feed()），
 * AFE 内部进行 NS 降噪和 VAD 检测，结果由 afe_fetch_task 异步取出。
 *
 * @param data 16-bit PCM 数据指针（来自 esp_codec_dev_read 输出）
 * @param len  采样点数量（必须等于 custom_wake_word_get_feed_chunksize() 的返回值）
 * @return void
 *
 * @note 调用者：bsp_codec.c → audio_feed_task()（采集主循环）
 * @note is_running 为 false 时直接返回，不投喂 AFE
 */
void custom_wake_word_feed(const int16_t *data, size_t len);

/**
 * @brief 停止 MultiNet 命令词检测（AFE/VAD 继续运行，仅关闭词检测）
 *
 * 设置 is_running = false，afe_fetch_task 中的 MultiNet detect 循环停止。
 * AFE 和 VAD 仍然运行（enhanced_pcm_hook 继续有数据）。
 *
 * @param 无
 * @return void
 *
 * @note 调用者：会话模块（TTS 播放期间，防止唤醒词假触发）
 */
void wake_word_stop(void);

/**
 * @brief 恢复 MultiNet 命令词检测监听
 *
 * 设置 is_running = true，afe_fetch_task 恢复 MultiNet detect 检测循环。
 * 通常在会话结束后调用，使设备重新进入唤醒词监听状态。
 *
 * @param 无
 * @return void
 *
 * @note 调用者：session.c → session_close()（会话关闭后恢复监听）
 */
void wake_word_start(void);

// ─── VAD / 增强 PCM 接口 ────────────────────────────────────────────────

/**
 * @brief VAD 状态变化回调函数类型
 *
 * 从 afe_fetch_task 中同步调用，每帧 AFE 输出后更新一次。
 * 禁止在回调内阻塞（会饿死 afe_fetch_task，导致 AFE ringbuffer 溢出）。
 *
 * @param state VAD_SPEECH = 检测到语音活动；VAD_SILENCE = 静音
 */
typedef void (*vad_state_cb_t)(vad_state_t state);

/**
 * @brief 增强 PCM 数据钩子函数类型（AFE 降噪后输出，供会话模块送编码器）
 *
 * 每帧 AFE 输出后从 afe_fetch_task 中同步调用。
 * 禁止在回调内阻塞（同上）。
 *
 * @param data    AFE 降噪后的 16-bit PCM 帧数据指针
 * @param samples 本帧采样点数量（= AFE fetch_chunksize）
 */
typedef void (*enhanced_pcm_cb_t)(const int16_t *data, size_t samples);

/**
 * @brief 注册 VAD 状态变化回调（传 NULL 则注销）
 *
 * @param cb 回调函数指针，NULL = 注销当前回调
 * @return void
 *
 * @note 调用者：session.c（当前未使用，VAD 通过 bsp_wake_word_get_vad_state 轮询）
 */
void bsp_wake_word_set_vad_callback(vad_state_cb_t cb);

/**
 * @brief 注册增强 PCM 钩子（传 NULL 则注销）
 *
 * 注册后，每帧 AFE 降噪输出都会同步调用此钩子，
 * 会话模块通过此钩子将降噪后的 PCM 送入编码器。
 *
 * @param hook 钩子函数指针，NULL = 注销（停止向编码器送数据）
 * @return void
 *
 * @note 调用者：session.c → session_on_wake_word()（会话启动时注册）、
 *              session_close()（会话结束时注销，传 NULL）
 */
void bsp_wake_word_set_enhanced_pcm_hook(enhanced_pcm_cb_t hook);

/**
 * @brief 查询当前 VAD 状态（线程安全只读）
 *
 * @return vad_state_t VAD_SPEECH = 当前检测到语音；VAD_SILENCE = 静音
 *
 * @note 调用者：session.c → on_enhanced_pcm()（PCM 钩子内轮询 VAD 状态）
 */
vad_state_t bsp_wake_word_get_vad_state(void);

// ─── AEC 参考信号接口 ────────────────────────────────────────────────────

/**
 * @brief AEC 参考信号提供者回调函数类型
 *
 * 每次 custom_wake_word_feed() 被调用时，AFE feed 前同步调用此回调，
 * 由调用方填充当前扬声器播放的 PCM 数据（等长于 AFE feed chunksize）。
 * 无播放时填零，AEC 不做减法（安全退化为纯 NS 模式）。
 *
 * @param buf     输出缓冲区（调用方须填充 samples 个 int16_t）
 * @param samples 需要填充的采样点数量（= AFE feed chunksize）
 */
typedef void (*aec_ref_cb_t)(int16_t *buf, size_t samples);

/**
 * @brief 注册 AEC 参考信号提供者（传 NULL 则注销，退化为零参考）
 *
 * 注册后每次 AFE feed 时调用此回调获取参考 PCM，与麦克风数据交织后
 * 送入 AFE（"MR" 格式），AFE AEC 算法消除扬声器在麦克风中的回声。
 *
 * @param cb 回调函数指针，NULL = 注销（使用零参考，AEC 不消减任何信号）
 * @return void
 *
 * @note 调用者：session.c → session_on_wake_word()（会话启动时注册），
 *              session_close()（会话结束时注销，传 NULL）
 */
void custom_wake_word_set_aec_ref(aec_ref_cb_t cb);
