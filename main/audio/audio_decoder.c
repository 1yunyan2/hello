/**
 * @file audio_decoder.c
 * @brief OPUS 音频解码器模块实现（OPUS → PCM）
 *
 * 从云端 WebSocket 接收的 OPUS 压缩音频帧，经解码后输出 PCM 数据，
 * 由播放任务（audio_processor_play_task）送入 ES8311 DAC → 扬声器。
 *
 * 数据流：
 *   WebSocket 接收 → audio_processor_write() → [dec_input: NOSPLIT 5KB]
 *   → audio_decoder_task（本任务）→ [dec_output: BYTEBUF 40KB]
 *   → audio_processor_play_task → esp_codec_dev_write() → 扬声器
 */

#include "audio_decoder.h"
#include "esp_audio_dec.h"
#include "esp_audio_dec_default.h"
#include "esp_audio_dec_reg.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "esp_log.h"

#define TAG "[AP] Decoder"

// ─── 解码器任务配置宏 ─────────────────────────────────────────────────────────
#define AUDIO_DECODER_TASK_CORE_ID 0        ///< 解码任务绑定 CPU0（与编码/播放同核）
#define AUDIO_DECODER_TASK_STACK_SIZE 32768 ///< 栈大小 32KB（OPUS 解码需要较大栈）
#define AUDIO_DECODER_TASK_PRIORITY 5       ///< 优先级 5（与编码器对称）

/**
 * @brief 解码器内部结构体（对外不透明，通过 audio_decoder_t* 访问）
 *
 * 字段说明：
 *   input_buffer  : NOSPLIT 类型，每次 xRingbufferReceive 读取整个 OPUS 帧
 *   output_buffer : BYTEBUF 类型，PCM 数据可自由分段写入（无帧边界约束）
 *   dec            : OPUS 解码器句柄（esp_audio_codec 框架）
 *   sample_rate    : 采样率（用于计算 PCM 输出缓冲区大小）
 *   channels       : 声道数（同上）
 *   is_running     : 任务退出标志
 */
struct audio_decoder
{
    RingbufHandle_t input_buffer;  ///< 输入缓冲区：OPUS 帧（NOSPLIT，整帧读取）
    RingbufHandle_t output_buffer; ///< 输出缓冲区：PCM 数据（BYTEBUF，分段写入）
    esp_audio_dec_handle_t dec;    ///< OPUS 解码器句柄

    int sample_rate; ///< 采样率（Hz），用于计算 PCM 输出缓冲区大小
    int channels;    ///< 声道数

    bool is_running;          ///< 任务运行标志：false 时解码主循环退出
    volatile bool drain_done; ///< 排水完成标志：主循环退出后排完 dec_input 剩余帧后置 true
};

// ═══════════════════════════════════════════════════════════════════════════════
// 解码器任务（核心解码循环）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief OPUS 音频解码主任务（持续将 OPUS 帧解码为 PCM 帧）
 *
 * 数据流：
 *   [dec_input: NOSPLIT] → xRingbufferReceive（整帧）→ esp_audio_dec_process() → [dec_output: BYTEBUF]
 *
 * 关键设计细节：
 *   1. NOSPLIT 类型保证每次 xRingbufferReceive 返回一完整 OPUS 帧（不被拆分）
 *   2. 输出缓冲区（40KB ≈ 1.28s PCM）足够大，正常不会满
 *   3. 输出缓冲区满时丢帧（xRingbufferSend timeout=0），保证实时性
 *
 * @param arg audio_decoder_t* 实例指针
 * @return 无（任务运行直到 is_running 为 false）
 *
 * @note 调用者：audio_decoder_start() 通过 xTaskCreatePinnedToCoreWithCaps() 创建
 * @note 运行核心：CPU0，栈 32KB SPIRAM，优先级 5
 */
void audio_decoder_task(void *arg)
{
    audio_decoder_t *audio_decoder = (audio_decoder_t *)arg;

    // ── 步骤 1：计算 PCM 输出缓冲区大小 ──────────────────────────────────────
    // 公式：采样率 × 声道数 × 每采样字节数 × 帧时长(ms) / 1000
    // 例：16000Hz × 1ch × 2bytes × 60ms / 1000 = 1920 字节/帧
    // 注意：云端编码时使用 60ms 帧，此处解码输出也必须是 60ms 的 PCM
    // size_t out_buffer_size = (size_t)audio_decoder->sample_rate // 采样率 16000
    //                          * audio_decoder->channels          // 声道数 1
    //                          * 2                                // 16-bit = 2 字节/采样
    //                          / 1000                             // 换算 ms → s
    //                          * 60;                              // 帧时长 60ms

    size_t out_buffer_size = 8192; //! 分配固定大小ram内存，放置解析数据过大卡死

    // ── 步骤 2：分配 PCM 输出缓冲区 ─────────────────────────────────────────
    void *out_buffer = malloc_zeroed(out_buffer_size); // 1920 字节（清零防止噪音）

    esp_audio_dec_out_frame_t out_frame = {
        .buffer = out_buffer,   // 解码输出目标地址
        .len = out_buffer_size, // 最大可容纳字节数
    };

    // ── 步骤 3：解码主循环 ────────────────────────────────────────────────────
    while (audio_decoder->is_running)
    {
        // 从 dec_input（NOSPLIT）读取一个完整的 OPUS 帧（不会被拆分）
        // 100ms 超时：无数据时定期检查 is_running 标志
        size_t size_read = 0;
        void *buf_read = xRingbufferReceive(
            audio_decoder->input_buffer,
            &size_read,
            pdMS_TO_TICKS(100)); // 100ms 超时

        if (!buf_read)
            continue; // 超时无数据（TTS 结束后的静音期），继续等待

        // 构造解码输入帧描述符（指向 RingBuf 内部内存，避免拷贝）
        esp_audio_dec_in_raw_t in_frame = {
            .buffer = buf_read, // OPUS 帧数据指针（指向 RingBuf 内部）
            .len = size_read,   // 帧大小（字节，由编码端决定，约 20~100 字节）
        };

        // 🌟 救命修复 1：每次解码前，必须重置 out_frame.len！
        // 否则底层解码器会越改越小，导致后续解码全部报 error:-4
        out_frame.len = out_buffer_size;

        // 执行 OPUS 解码（1920字节 PCM → out_frame）
        esp_audio_err_t ret = esp_audio_dec_process(
            audio_decoder->dec, &in_frame, &out_frame);

        // 归还环形缓冲区使用权（解码完成后立即归还，减少缓冲区占用时间）
        // 必须在使用完 buf_read 数据后才能归还
        vRingbufferReturnItem(audio_decoder->input_buffer, buf_read);

        // 错误处理
        if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH)
        {
            // 输出缓冲区不足：out_buffer_size 计算与实际帧时长不符
            // 通常说明云端编码帧时长与解码配置（60ms）不一致
            ESP_LOGW(TAG, "PCM 输出缓冲区不足，请检查帧时长配置");
        }

        if (ret != ESP_OK)
        {

            // 🌟 救命修复 2：解码失败时，必须延时释放 CPU，喂看门狗！
            // 防止程序瞬间进入死循环吃满 CPU，解决喇叭兹拉声和 AFE 溢出问题
            vTaskDelay(1);
            continue; // 解码失败（数据损坏），丢弃本帧，继续处理下一帧
        }
        // ── 将解码后的 PCM 数据写入播放缓冲区 ─────────────────────────────
        // BYTEBUF 类型：支持分段写入，PCM 数据无帧边界约束
        // 背压链路：play_task(I2S实时) → dec_output → decoder 等待 → dec_input 积压
        // → audio_processor_write 阻塞 → WebSocket 接收自然降速。
        // 使用 100ms 超时循环重试：既保留背压不丢帧，又能及时响应 stop 信号。
        // 旧方案 portMAX_DELAY 会导致 stop 时任务无法及时退出。
        // 这样解码速率与播放速率自动对齐，彻底避免 PCM 丢帧和快进卡顿。
        // xRingbufferSend(
        //     audio_decoder->output_buffer,
        //     out_frame.buffer,
        //     out_frame.decoded_size, // 实际解码输出字节数
        //     portMAX_DELAY);         // 阻塞等待，不丢帧
        while (audio_decoder->is_running)
        {
            if (xRingbufferSend(
                    audio_decoder->output_buffer,
                    out_frame.buffer,
                    out_frame.decoded_size, // 实际解码输出字节数
                    pdMS_TO_TICKS(100)))    // 100ms 超时，失败则检查 is_running 后重试
                break;                      // 写入成功，跳出重试循环
        }
    }

    // ── 排水阶段：主循环退出后，继续把 dec_input 剩余帧解码写入 dec_output ──
    // 此时 audio_processor->is_running 仍为 true，play_task 仍在消耗 dec_output。
    // portMAX_DELAY 写入安全：play_task 不停则 dec_output 始终有人消费。
    // 退出条件：dec_input 为空（非阻塞读返回 NULL）。
    {
        size_t drain_size = 0;
        void *drain_buf = NULL;
        while ((drain_buf = xRingbufferReceive(
                    audio_decoder->input_buffer, &drain_size, 0)) != NULL)
        {
            esp_audio_dec_in_raw_t drain_in = {
                .buffer = drain_buf,
                .len = drain_size,
            };
            out_frame.len = out_buffer_size; // 每次重置，防止底层覆盖

            esp_audio_err_t ret = esp_audio_dec_process(
                audio_decoder->dec, &drain_in, &out_frame);
            vRingbufferReturnItem(audio_decoder->input_buffer, drain_buf);

            if (ret != ESP_OK)
                continue; // 帧损坏，跳过

            // 写入 dec_output：play_task 仍运行，portMAX_DELAY 不会死锁
            xRingbufferSend(audio_decoder->output_buffer,
                            out_frame.buffer, out_frame.decoded_size,
                            portMAX_DELAY);
        }
    }
    // 通知 audio_decoder_stop() 排水完成，可以停止 play_task
    audio_decoder->drain_done = true;

    // ── 任务退出：释放 PCM 输出缓冲区，自删除 ────────────────────────────────
    free(out_buffer);
    vTaskDelete(NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 生命周期管理
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 创建 OPUS 解码器实例，注册并配置解码参数
 *
 * @param sample_rate 采样率（Hz），必须与云端编码端一致（16000）
 * @param channels    声道数（1 = 单声道）
 * @return audio_decoder_t* 解码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
audio_decoder_t *audio_decoder_create(int sample_rate, int channels)
{
    // ── 步骤 1：分配并清零解码器结构体 ───────────────────────────────────────
    audio_decoder_t *audio_decoder = (audio_decoder_t *)malloc_zeroed(sizeof(audio_decoder_t));

    // 保存采样参数（任务内计算 PCM 输出缓冲区大小时使用）
    audio_decoder->sample_rate = sample_rate;
    audio_decoder->channels = channels;

    // ── 步骤 2：注册 OPUS 解码器 ──────────────────────────────────────────────
    ESP_ERROR_CHECK(esp_opus_dec_register());

    // ── 步骤 3：配置解码参数（必须与编码端严格一致）─────────────────────────
    esp_opus_dec_cfg_t opus_cfg = {
        .sample_rate = sample_rate,                          // 采样率：16000Hz
        .channel = channels,                                 // 声道数：1
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS, // 帧时长：60ms（与编码端对齐！）
        .self_delimited = false,                             // 标准 OPUS 封包格式（非自分界）
    };

    // 封装为通用解码器配置
    esp_audio_dec_cfg_t dec_cfg = {
        .cfg = &opus_cfg,
        .cfg_sz = sizeof(esp_opus_dec_cfg_t),
        .type = ESP_AUDIO_TYPE_OPUS,
    };

    // ── 步骤 4：打开解码器，获取句柄 ─────────────────────────────────────────
    ESP_ERROR_CHECK(esp_audio_dec_open(&dec_cfg, &audio_decoder->dec));

    return audio_decoder;
}

/**
 * @brief 销毁解码器实例（关闭句柄 → 注销类型 → 释放内存）
 *
 * @param audio_decoder 解码器实例指针（调用后不可再使用）
 *
 * @note 调用者：audio_processor.c → audio_processor_destroy()
 */
void audio_decoder_destroy(audio_decoder_t *audio_decoder)
{
    // 关闭解码器：释放 OPUS 解码状态机内部资源
    esp_audio_dec_close(audio_decoder->dec);

    // 注销 OPUS 解码器类型（从框架注册表移除）
    esp_audio_dec_unregister(ESP_AUDIO_TYPE_OPUS);

    // 释放解码器结构体
    free(audio_decoder);
}

/**
 * @brief 绑定输入/输出环形缓冲区
 *
 * @param audio_decoder  解码器实例指针
 * @param input_buffer   OPUS 帧输入（NOSPLIT 类型，整帧读取）
 * @param output_buffer  PCM 输出（BYTEBUF 类型，分段写入）
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
void audio_decoder_set_buffer(audio_decoder_t *audio_decoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer)
{
    audio_decoder->input_buffer = input_buffer;   // OPUS 来源（WebSocket 接收链路）
    audio_decoder->output_buffer = output_buffer; // PCM 去向（播放任务链路入口）
}

/**
 * @brief 启动解码任务
 *
 * @param audio_decoder 解码器实例指针（已绑定缓冲区）
 *
 * @note 调用者：audio_processor.c → audio_processor_start()
 */
void audio_decoder_start(audio_decoder_t *audio_decoder)
{
    // ── 前置检查 ──────────────────────────────────────────────────────────────
    if (!audio_decoder->input_buffer || !audio_decoder->output_buffer)
    {
        ESP_LOGW(TAG, "解码器缓冲区未设置，拒绝启动任务");
        return;
    }

    // ── 设置运行标志 ──────────────────────────────────────────────────────────
    audio_decoder->is_running = true;

    // ── 创建解码任务（栈分配在 SPIRAM）───────────────────────────────────────
    xTaskCreatePinnedToCoreWithCaps(
        audio_decoder_task,            // 任务函数
        "decoder_task",                // 任务名称
        AUDIO_DECODER_TASK_STACK_SIZE, // 栈大小：32KB
        audio_decoder,                 // 任务参数：解码器实例指针
        AUDIO_DECODER_TASK_PRIORITY,   // 优先级：5
        NULL,                          // 不保存任务句柄
        AUDIO_DECODER_TASK_CORE_ID,    // 绑定核心：CPU0
        MALLOC_CAP_SPIRAM);            // 栈内存来源：外部 SPIRAM
}

/**
 * @brief 停止解码任务（清除运行标志，等待任务自然退出）
 *
 * @param audio_decoder 解码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_stop()
 */
void audio_decoder_stop(audio_decoder_t *audio_decoder)
{
    // 清除运行标志：主循环在当前 portMAX_DELAY 完成后检测到 is_running=false 退出
    audio_decoder->is_running = false;

    // 等待排水完成（最多 30 秒）：
    //   解码器退出主循环后会进入排水阶段，把 dec_input 剩余帧全部写入 dec_output，
    //   drain_done 置 true 后才返回，play_task 随后播完所有积压的 PCM。
    // 若用户打断（audio_processor_flush_output 已清空 dec_input），排水立即完成。
    for (int i = 0; i < 300 && !audio_decoder->drain_done; i++)
        vTaskDelay(pdMS_TO_TICKS(100));

    if (!audio_decoder->drain_done)
        ESP_LOGW(TAG, "解码器排水超时（30s），可能丢失末尾音频");
}
