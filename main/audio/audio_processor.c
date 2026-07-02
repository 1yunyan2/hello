/**
 * @file audio_processor.c
 * @brief 音频处理器 — 编解码管道的中枢协调者
 *
 * 本模块是音频上行（麦克风→云端）和下行（云端→扬声器）的数据枢纽，
 * 内部管理编码器、解码器和四个环形缓冲区，提供统一的读写接口。
 *
 * 数据流架构：
 *
 *   上行（录音→编码→发送）：
 *     麦克风 PCM → [enc_input] → 编码器(OPUS) → [enc_output] → WebSocket 发送任务
 *
 *   下行（接收→解码→播放）：
 *     WebSocket 接收 → [dec_input] → 解码器(PCM) → [dec_output] → 播放任务 → 扬声器
 *
 * 四个环形缓冲区的类型选择：
 *   enc_input  / dec_output = BYTEBUF（字节流，PCM 数据可自由拼接）
 *   enc_output / dec_input  = NOSPLIT（不拆分，OPUS 帧必须保持完整性）
 */

#include "audio_processor.h"
#include "audio_encoder.h"
#include "audio_decoder.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "driver/i2s_std.h"
#include "esp_log.h"

#define TAG "Audio Processor"

/* 播放任务配置 */
#define AUDIO_PROCESSOR_TASK_STACK_SIZE (4 * 1024) // 栈大小 4KB（播放逻辑简单，无需更大）
#define AUDIO_PROCESSOR_TASK_PRIORITY 5            // 优先级（与编解码器任务对称）
#define AUDIO_PROCESSOR_TASK_CORE_ID 0             // 固定到 CPU 核心 0

// ─── 环形缓冲区大小配置（单位：字节）────────────────────────────────────────
// 编码器输入（原始 PCM）：131072 B ≈ 4.1s @ 16kHz 16-bit 单声道
// 历史值 81920 B（~2.56s）在 SILK 编码偶发耗时 + CPU0 拥堵时仍会被打满，
// 导致 audio_processor_write_pcm 丢帧（即"尾部音频丢失"），扩到 4s 给 encoder 更多追赶时间。
#define ENC_INPUT_BUF_SIZE 131072 // 采样率16kHz × 2字节/采样点 × 1声道 = 131072 / (16000*18/2*1) ≈ 4.1秒
// 编码器输出（OPUS 帧）：32KB ≈ 500+ 帧（每帧约 60 字节）
// 历史 12288（~200 帧）在 SILK NSQ 尖峰 + ws_sender 网络抖动时仍会打满丢帧，
// 扩到 32KB 吸收 CPU0 拥堵时的编码追赶脉冲，同时保留 start 握手期的积压余量。
#define ENC_OUTPUT_BUF_SIZE 32768
#define DEC_INPUT_BUF_SIZE 32768  //! 原为5120 解码器输入（OPUS 帧）：云端下发的音频缓冲,
#define DEC_OUTPUT_BUF_SIZE 40960 // 解码器输出（PCM 播放）：~1.28s 缓冲，保证播放流畅
// AEC 参考缓冲区：play_task 写入 I2S 时同步推送一份副本，audio_feed_task 读取后
// 作为 AFE AEC 算法的参考信号（从麦克风中消除扬声器回声）。
// 大小 8192 字节 ≈ 256ms @16kHz，足够吸收 play_task(CPU0) 和 feed_task(CPU1) 的速率差。
#define AEC_REF_BUF_SIZE 8192

/**
 * @brief 音频处理器内部结构体
 *
 * 管理编码器/解码器实例和四个环形缓冲区，
 * 以及播放任务的生命周期控制。
 */
struct audio_processor
{
    audio_encoder_t *encoder; ///< OPUS 编码器实例（PCM → OPUS）
    audio_decoder_t *decoder; ///< OPUS 解码器实例（OPUS → PCM）

    RingbufHandle_t enc_input;   ///< 编码器输入缓冲（BYTEBUF：麦克风 PCM 数据入口）
    RingbufHandle_t enc_output;  ///< 编码器输出缓冲（NOSPLIT：OPUS 帧，WebSocket 发送任务消费）
    RingbufHandle_t dec_input;   ///< 解码器输入缓冲（NOSPLIT：云端下发的 OPUS 帧）
    RingbufHandle_t dec_output;  ///< 解码器输出缓冲（BYTEBUF：解码后 PCM，播放任务消费）
    RingbufHandle_t aec_ref_buf; ///< AEC 参考缓冲（BYTEBUF：play_task 写入 I2S 时推送副本）
                                 ///< audio_feed_task 读取后交给 AFE AEC 算法消除回声

    volatile bool is_running;      ///< 运行标志（控制播放任务循环）
    volatile bool mute_output;     ///< 静音标志：唤醒词打断时置位，play_task 丢弃 dec_output 数据不写 I2S
    TaskHandle_t play_task_handle; ///< 播放任务句柄（用于等待任务退出）
};

// ─── 播放任务 ──────────────────────────────────────────────────────────────

/**
 * @brief 音频播放任务 — 从解码器输出缓冲区读取 PCM 数据并送入扬声器
 *
 * 持续从 dec_output 读取解码后的 PCM 数据，通过 esp_codec_dev_write 发送到
 * ES8311 DAC → I2S → 扬声器。每次最多读取 2048 字节，100ms 超时以便响应
 * is_running 标志的变化（用于优雅退出）。
 *
 * @param arg audio_processor_t* 实例指针
 */
static void audio_processor_play_task(void *arg)
{
    audio_processor_t *audio_processor = (audio_processor_t *)arg;
    bsp_board_t *board = bsp_board_get_instance();

    if (board == NULL || board->i2s_tx_handle == NULL)
    {
        audio_processor->play_task_handle = NULL;
        vTaskDelete(NULL);
        return;
    }

    // 静音包 640 B = 20ms @16kHz 16-bit 单声道，与下面 I2S write 20ms 超时对齐：
    // 每次灌静音消耗的"音频时长"恰好 = write 等待 DMA 释放的最大时长，DMA 抖动最小。
    static const uint8_t silence_buf[640] = {0};

    // 🟢 智能预缓冲状态机变量
    bool prebuffering = true;
    TickType_t buffer_start_tick = xTaskGetTickCount();
    // 仅用于日志的"本轮预缓冲起点"。与开闸判据无关（开闸只看 bytes_waiting），
    // 每次进入预缓冲阶段时刷新，使"耗时"反映本轮蓄水时长而非跨轮累计值。
    // buffer_start_tick 刻意不在短句开闸后重置（防旧 tick 误开闸），
    // 导致多轮对话中 wait_ms 累计跨轮时间，日志"耗时"会显示成 8~17 秒的荒谬值。
    TickType_t prebuf_log_start_tick = xTaskGetTickCount();
    // 连续欠载计数器：连续 N 次从 dec_output 读不到数据才触发预缓冲重置，
    // 过滤解码器两帧之间的瞬间间隙（约 20ms），避免误触发"滴滴"电音。
    int underrun_count = 0;
#define UNDERRUN_THRESHOLD 3 // 连续 3 次 × 20ms = 60ms 真空才重置

    // active_playback：当前轮次是否正在（或曾经）播放 TTS。
    // 读到真实 PCM 时置 true；预缓冲以超时（无数据）退出时重置为 false。
    // 效果：TTS 结束后 LISTENING 期的 660ms 噪音告警被抑制；
    //       下一轮 TTS 真正开始时重新激活告警。
    bool active_playback = false;

    while (audio_processor->is_running)
    {
        // 🟢 阶段 1：预缓冲蓄水期
        if (prebuffering)
        {
            size_t bytes_waiting = 0;
            // 查询当前解码器输出了多少 PCM 数据准备播放
            vRingbufferGetInfo(audio_processor->dec_output, NULL, NULL, NULL, NULL, &bytes_waiting);

            // 计算从开始蓄水到现在过了多久（开闸判据 / 空窗期 delay 决策用，不改语义）
            uint32_t wait_ms = (xTaskGetTickCount() - buffer_start_tick) * portTICK_PERIOD_MS;
            // 仅供日志显示的本轮蓄水时长，避免跨轮累计导致"耗时"虚高
            uint32_t prebuf_ms = (xTaskGetTickCount() - prebuf_log_start_tick) * portTICK_PERIOD_MS;

            // 🚀 核心调优区：三分支开闸 / 蓄水 / 空窗期
            // 关键设计：开闸的唯一充分条件是 bytes_waiting > 0；
            //   wait_ms 仅用作"已在预缓冲状态待了多久"，不再用作"新 TTS 触发器"。
            //   这样 TTS 自然结束后即使 buffer_start_tick 是旧值，也不会误开闸进入
            //   阶段 2 → underrun 高频循环（曾导致 CPU0 拥堵 + MultiNet buffer overflow）。
            if (bytes_waiting >= 16384)
            {
                // ① 攒够水（≈0.5s）→ 开闸正常播放
                prebuffering = false;
                underrun_count = 0;
                active_playback = true;
                ESP_LOGI(TAG, "TTS 预缓冲完成 (数据:%d B, 耗时:%d ms)，开始流畅播放",
                         bytes_waiting, (int)prebuf_ms);
            }
            else if (wait_ms >= 600 && bytes_waiting > 0)
            {
                // ② 超时但已有少量数据（大模型短句）→ 开闸，避免死锁
                prebuffering = false;
                underrun_count = 0;
                active_playback = true;
                ESP_LOGI(TAG, "TTS 预缓冲短句开闸 (数据:%d B, 耗时:%d ms)",
                         bytes_waiting, (int)prebuf_ms);
            }
            else
            {
                // ③ 蓄水中 或 空窗期（wait_ms>=600 且 bytes_waiting==0）
                //    都保持 prebuffering，持续灌静音维持 I2S 时钟稳定。
                //    新 TTS 数据一到，下一轮就走 ① 或 ② 开闸。
                size_t silence_written = 0;
                i2s_channel_write(board->i2s_tx_handle, silence_buf, sizeof(silence_buf),
                                  &silence_written, pdMS_TO_TICKS(20));
                // 空窗期降频让出 CPU0：
                //   - 蓄水期（wait_ms < 600）：delay 10ms，与原行为一致，确保 IDLE0 喂狗
                //   - 空窗期（wait_ms >= 600 且无数据）：delay 30ms，给 MultiNet 让出
                //     CPU 时间片，避免 play_task 高频自转挤占唤醒词推理。
                vTaskDelay(pdMS_TO_TICKS((wait_ms >= 600 && bytes_waiting == 0) ? 30 : 10));
                // ⚠️ 不重置 buffer_start_tick：wait_ms 持续累计无害，
                //    开闸只看 bytes_waiting，永远不会因旧 tick 误触发。
                continue;
            }
        }

        // 🟢 阶段 2：正常流式连贯播放
        size_t size_read = 0;
        void *buf = xRingbufferReceiveUpTo(audio_processor->dec_output, &size_read, pdMS_TO_TICKS(20), 2048);

        if (buf)
        {
            underrun_count = 0; // 读到数据，清零欠载计数
            if (!audio_processor->mute_output)
            {
                active_playback = true; // 正在播放真实 TTS，激活欠载告警
                size_t bytes_written = 0;
                // 【看门狗修复】portMAX_DELAY → 200ms：
                // I2S DMA buffer 若短暂满载，portMAX_DELAY 会让 play_task 永久阻塞，
                // IDLE0（CPU0）超过 5s 喂不了狗 → task_wdt 触发。
                // 200ms 足够覆盖 DMA 释放一个 slot（@32kHz 约 32ms），
                // 同时远小于看门狗超时（5s），即使偶发超时也只丢失极少量 PCM（< 10ms）。
                esp_err_t wr_ret = i2s_channel_write(board->i2s_tx_handle, buf, size_read,
                                                     &bytes_written, portMAX_DELAY);
                if (wr_ret != ESP_OK)
                    ESP_LOGW(TAG, "I2S write 超时 (size=%d ret=%d)，可能瞬间卡顿", (int)size_read, wr_ret);
                xRingbufferSend(audio_processor->aec_ref_buf, buf, size_read, 0);
            }
            vRingbufferReturnItem(audio_processor->dec_output, buf);
        }
        else
        {
            // 🟢 阶段 3：缓冲欠载（网络太卡，播放速度追上了云端下载速度）
            // ⚠️ 极其关键：必须立刻重新进入预缓冲状态！否则会出现鬼畜的"一帧一卡"电音！
            // prebuffering = true;
            // buffer_start_tick = xTaskGetTickCount(); // 重置计时器，重新开始蓄水
            // 连续超过 UNDERRUN_THRESHOLD 次（约 60ms）读不到数据，才判定为真正欠载。
            // 单次超时可能只是解码器两帧之间的瞬间空档，不应立刻重置预缓冲，
            // 否则静音包和有效 PCM 交替写入 I2S DAC 会产生"滴滴"电音。
            underrun_count++;
            if (underrun_count >= UNDERRUN_THRESHOLD)
            {
                underrun_count = 0;
                prebuffering = true;
                buffer_start_tick = xTaskGetTickCount();     // 重置计时器，重新开始蓄水
                prebuf_log_start_tick = xTaskGetTickCount(); // 同步刷新日志起点，反映本轮蓄水时长
                // 仅在当前轮次 TTS 真正播过数据后才告警。
                // active_playback 在预缓冲超时无数据退出时已置 false，
                // 所以 TTS 结束后的 LISTENING 噪音循环不会再打印。
                if (active_playback)
                    ESP_LOGW(TAG, "dec_output 持续欠载 %d ms，重新进入预缓冲", UNDERRUN_THRESHOLD * 20);
            }
            // 欠载期间持续喂静音包，维持 I2S 时钟稳定
            size_t silence_written = 0;
            i2s_channel_write(board->i2s_tx_handle, silence_buf, sizeof(silence_buf), &silence_written, pdMS_TO_TICKS(20));
        }
    }

    /* ── 排水阶段：确保 is_running 被置 false 后，最后一个字完整播完 ── */
    {
        size_t drain_size = 0;
        void *drain_buf = NULL;
        while ((drain_buf = xRingbufferReceiveUpTo(audio_processor->dec_output,
                                                   &drain_size, pdMS_TO_TICKS(200), 2048)) != NULL)
        {
            size_t drain_written = 0;
            i2s_channel_write(board->i2s_tx_handle, drain_buf, drain_size, &drain_written, portMAX_DELAY);
            xRingbufferSend(audio_processor->aec_ref_buf, drain_buf, drain_size, 0);
            vRingbufferReturnItem(audio_processor->dec_output, drain_buf);
        }
    }

    audio_processor->play_task_handle = NULL;
    // ★ 本任务栈由 xTaskCreatePinnedToCoreWithCaps(...SPIRAM) 分配，自删必须用 WithCaps，
    //   否则 4KB SPIRAM 栈不会被 idle 回收 → 每轮会话泄漏。
    vTaskDeleteWithCaps(NULL);
}
// ─── 公开 API：生命周期管理 ────────────────────────────────────────────────

/**
 * @brief 创建音频处理器实例
 *
 * 依次创建编码器、解码器和四个环形缓冲区，并将缓冲区绑定到编解码器。
 * 任一环形缓冲区创建失败则整体回滚，避免半初始化对象导致后续崩溃。
 *
 * @return audio_processor_t* 成功返回处理器指针，失败返回 NULL
 */
audio_processor_t *audio_processor_create(void)
{
    audio_processor_t *audio_processor = (audio_processor_t *)malloc_zeroed(sizeof(audio_processor_t));

    /* 创建编解码器实例（单声道 16kHz，与硬件采集参数一致） */
    audio_processor->encoder = audio_encoder_create(BSP_CODEC_SAMPLE_RATE, 1);
    audio_processor->decoder = audio_decoder_create(BSP_CODEC_SAMPLE_RATE, 1);

    /* 编解码器创建失败 → 立即回滚，避免后续空指针崩溃 */
    if (!audio_processor->encoder || !audio_processor->decoder)
    {
        ESP_LOGE(TAG, "audio_processor_create: encoder/decoder alloc failed, rollback");
        if (audio_processor->encoder)
            audio_encoder_destroy(audio_processor->encoder);
        if (audio_processor->decoder)
            audio_decoder_destroy(audio_processor->decoder);
        free(audio_processor);
        return NULL;
    }

    /* 创建五个环形缓冲区（全部分配在 SPIRAM，节省内部 SRAM） */
    audio_processor->enc_input = xRingbufferCreateWithCaps(ENC_INPUT_BUF_SIZE, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);
    audio_processor->enc_output = xRingbufferCreateWithCaps(ENC_OUTPUT_BUF_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    audio_processor->dec_input = xRingbufferCreateWithCaps(DEC_INPUT_BUF_SIZE, RINGBUF_TYPE_NOSPLIT, MALLOC_CAP_SPIRAM);
    audio_processor->dec_output = xRingbufferCreateWithCaps(DEC_OUTPUT_BUF_SIZE, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);
    /* AEC 参考缓冲：BYTEBUF，play_task → I2S 写入时推副本，feed_task 读取供 AFE AEC */
    audio_processor->aec_ref_buf = xRingbufferCreateWithCaps(AEC_REF_BUF_SIZE, RINGBUF_TYPE_BYTEBUF, MALLOC_CAP_SPIRAM);

    /* 任一 ringbuf 创建失败 → 整体回滚，避免半初始化对象导致后续崩溃 */
    if (!audio_processor->enc_input || !audio_processor->enc_output ||
        !audio_processor->dec_input || !audio_processor->dec_output ||
        !audio_processor->aec_ref_buf)
    {
        ESP_LOGE(TAG, "audio_processor_create: ringbuf alloc failed, rollback");
        // ★ WithCaps 创建的 ringbuf 必须用 WithCaps 删除，回滚路径同样遵守，避免 caps 泄漏
        if (audio_processor->enc_input)
            vRingbufferDeleteWithCaps(audio_processor->enc_input);
        if (audio_processor->enc_output)
            vRingbufferDeleteWithCaps(audio_processor->enc_output);
        if (audio_processor->dec_input)
            vRingbufferDeleteWithCaps(audio_processor->dec_input);
        if (audio_processor->dec_output)
            vRingbufferDeleteWithCaps(audio_processor->dec_output);
        if (audio_processor->aec_ref_buf)
            vRingbufferDeleteWithCaps(audio_processor->aec_ref_buf);
        audio_encoder_destroy(audio_processor->encoder);
        audio_decoder_destroy(audio_processor->decoder);
        free(audio_processor);
        return NULL;
    }

    /* 将环形缓冲区绑定到编解码器（编码器：enc_input→enc_output，解码器：dec_input→dec_output） */
    audio_encoder_set_buffer(audio_processor->encoder, audio_processor->enc_input, audio_processor->enc_output);
    audio_decoder_set_buffer(audio_processor->decoder, audio_processor->dec_input, audio_processor->dec_output);
    PRINT_MEM_INFO(TAG, "音频管道环形缓冲区创建后");
    return audio_processor;
}

/**
 * @brief 销毁音频处理器实例
 * 释放四个环形缓冲区、编解码器实例和结构体本身
 */
void audio_processor_destroy(audio_processor_t *audio_processor)
{
    // play_task 超时仍未退出：强制终止，确保所有内存被释放，不留内存泄漏。
    // audio_processor_stop 已等待 2s；到此仍未退出属于 I2S 硬件异常，
    // 强制删除虽然会丢失末尾音频，但总好过每次会话 ~272KB PSRAM 永久泄漏。
    if (audio_processor->play_task_handle != NULL)
    {
        ESP_LOGW("AUDIO_PROC", "play_task 超时未退出，强制终止以释放内存");
        // ★ 栈在 SPIRAM caps 分配，强杀也必须用 WithCaps，否则 4KB 栈泄漏。
        //   先存句柄、置 NULL、再删，防止与自删路径竞态二次释放。
        TaskHandle_t h = audio_processor->play_task_handle;
        audio_processor->play_task_handle = NULL;
        vTaskDeleteWithCaps(h);
    }

    // ★ ringbuf 由 xRingbufferCreateWithCaps(...SPIRAM) 创建，必须用 WithCaps 版本删除，
    //   否则 caps 分配的 SPIRAM（5 个共 ~224KB）不会被回收 → 每轮会话泄漏一整套缓冲。
    vRingbufferDeleteWithCaps(audio_processor->enc_input);
    vRingbufferDeleteWithCaps(audio_processor->enc_output);
    vRingbufferDeleteWithCaps(audio_processor->dec_input);
    vRingbufferDeleteWithCaps(audio_processor->dec_output);
    vRingbufferDeleteWithCaps(audio_processor->aec_ref_buf);

    audio_encoder_destroy(audio_processor->encoder);
    audio_decoder_destroy(audio_processor->decoder);

    free(audio_processor);
    // 给 CPU0 的 IDLE 任务留出 100ms 时间，让其释放已 vTaskDelete 但尚未清理的
    // SPIRAM 任务栈（encoder 32KB + decoder 32KB + play 4KB），
    // 确保内存快照反映真实的 PSRAM 剩余量，而非快照时序偏差导致的虚假"泄漏"。
    vTaskDelay(pdMS_TO_TICKS(100));
    PRINT_MEM_INFO(TAG, "音频处理器销毁后（ringbuf+编解码器均已释放）");
}

// ─── 公开 API：启停控制 ────────────────────────────────────────────────────

/**
 * @brief 启动音频处理器
 * 依次启动编码器任务、解码器任务和播放任务
 */
void audio_processor_start(audio_processor_t *audio_processor)
{
    audio_processor->is_running = true;
    audio_encoder_start(audio_processor->encoder);
    audio_decoder_start(audio_processor->decoder);

    /* 创建播放任务：固定到核心 0，栈分配在 SPIRAM */
    xTaskCreatePinnedToCoreWithCaps(audio_processor_play_task, "play_task",
                                    AUDIO_PROCESSOR_TASK_STACK_SIZE, audio_processor,
                                    AUDIO_PROCESSOR_TASK_PRIORITY,
                                    &audio_processor->play_task_handle,
                                    AUDIO_PROCESSOR_TASK_CORE_ID, MALLOC_CAP_SPIRAM);
    PRINT_MEM_INFO(TAG, "音频处理器启动后（编解码+播放任务均已创建）");
}

/**
 * @brief 停止音频处理器
 *
 * 停止顺序（顺序不可调换）：
 *   1. 停编码器（上行停止，不影响下行播放）
 *   2. 等解码器排水（decoder 把 dec_input 剩余帧全部写入 dec_output）
 *      ── 此阶段 is_running 仍为 true，play_task 继续消耗 dec_output ──
 *   3. 排水完成后才停 play_task（is_running = false）
 *      play_task 继续运行直到 dec_output 清空后自然退出
 *   4. 等待 play_task 彻底退出（最多 2 秒）
 *
 * 结果：TTS 末尾音频完整播放，不截断。
 * 打断场景：audio_processor_flush_output() 已清空 dec_input，
 *           排水立即完成，本函数退出不会延迟。
 */
void audio_processor_stop(audio_processor_t *audio_processor)
{
    /* ① 停编码器（上行） */
    audio_encoder_stop(audio_processor->encoder);

    /* ② 等解码器把 dec_input 剩余帧全部播完（play_task 此时仍在运行） */
    audio_decoder_stop(audio_processor->decoder);

    /* ③ 解码排水完成，停播放任务 */
    audio_processor->is_running = false;

    /* ④ 等 play_task 退出（每次消耗 2048 字节 PCM，最多等 2 秒） */
    for (int i = 0; i < 10 && audio_processor->play_task_handle != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(200));
}

// ─── 公开 API：数据读写 ────────────────────────────────────────────────────

/**
 * @brief 从编码器输出缓冲区读取 OPUS 数据（阻塞式）
 * 供 WebSocket 发送任务调用，会一直阻塞直到有 OPUS 帧可用
 * @return 实际读取的字节数，0 表示读取失败
 */
size_t audio_processor_read(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    size_t size_read = 0;
    void *buf_read = xRingbufferReceive(audio_processor->enc_output, &size_read, portMAX_DELAY);
    if (!buf_read)
        return 0;

    if (size_read > size)
    {
        ESP_LOGW(TAG, "Buffer size is too small");
        size_read = size;
    }

    memcpy(buffer, buf_read, size_read);
    vRingbufferReturnItem(audio_processor->enc_output, buf_read);
    return size_read;
}

/**
 * @brief 将麦克风采集的 PCM 数据写入编码器输入缓冲区
 * 这是音频上行链路的数据入口（麦克风 PCM → 编码器）
 *
 * 【关键】本函数在 afe_fetch_task（CPU1）中通过 on_enhanced_pcm 调用，
 * 绝不能用 portMAX_DELAY！否则 enc_input 满时阻塞 afe_fetch_task，
 * 导致 AFE fetch 停止 → AFE FEED ringbuffer 溢出 → VAD/唤醒词全部失效。
 * 使用 50ms 超时：写不进去就丢帧，丢几帧上行 PCM 只影响 ASR 质量，
 * 远好于卡死整个 AFE 链路。
 */
void audio_processor_write_pcm(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    // 超时从 50ms 降至 10ms：
    // 旧 50ms 会在 enc_input 满时阻塞 afe_fetch_task 整整一帧半，
    // 导致 AFE FEED ringbuffer 溢出（fetch 跟不上 feed 速率）。
    // 丢几帧上行 PCM 只影响 ASR 质量，远好于卡死整条 AFE 链路。

    // ── 预警：enc_input 接近满（>75%）说明 encoder 跟不上 ─────────────
    // 限频 500ms：避免每帧 PCM 都打印，调试期足以定位触发场景（UI 渲染脉冲、长会话等）
    static TickType_t s_high_warn_tick = 0;
    size_t pending = 0;
    vRingbufferGetInfo(audio_processor->enc_input, NULL, NULL, NULL, NULL, &pending);
    if (pending > ENC_INPUT_BUF_SIZE * 3 / 4)
    {
        TickType_t now = xTaskGetTickCount();
        if ((uint32_t)((now - s_high_warn_tick) * portTICK_PERIOD_MS) >= 500)
        {
            ESP_LOGW(TAG, "enc_input 积压预警：%u/%u B (%u%%)，encoder 即将跟不上",
                     (unsigned)pending, (unsigned)ENC_INPUT_BUF_SIZE,
                     (unsigned)(100UL * pending / ENC_INPUT_BUF_SIZE));
            s_high_warn_tick = now;
        }
    }

    if (xRingbufferSend(audio_processor->enc_input, buffer, size, 0) != pdTRUE)
    {
        // 限频 + 累计计数：满载时通常连续丢几十帧，逐帧打印会刷屏并加剧 CPU 拥堵
        static TickType_t s_drop_tick = 0;
        static uint32_t s_drop_count = 0;
        s_drop_count++;
        TickType_t now = xTaskGetTickCount();
        if ((uint32_t)((now - s_drop_tick) * portTICK_PERIOD_MS) >= 500)
        {
            ESP_LOGE(TAG, "enc_input 满，最近 500ms 累计丢弃 %lu 帧 PCM（每帧 %d B）",
                     (unsigned long)s_drop_count, (int)size);
            s_drop_count = 0;
            s_drop_tick = now;
        }
    }
}

/**
 * @brief 将云端下发的 OPUS 数据写入解码器输入缓冲区
 * 这是音频下行链路的数据入口（WebSocket 接收 → 解码器）
 *
 * 【关键】本函数在 WebSocket 事件回调中调用（protocol_event_handler → PROTOCOL_EVENT_AUDIO），
 * 绝不能用 portMAX_DELAY！否则 dec_input 满时会阻塞整个 WebSocket 接收线程，
 * 导致 STT 文本、TTS_STOP 等控制消息全部收不到 → 会话超时。
 * 使用 100ms 超时：写不进去就丢帧，丢几帧 OPUS 只是轻微卡顿，远好于卡死整条链路。
 */
void audio_processor_write(audio_processor_t *audio_processor, void *buffer, size_t size)
{
    if (audio_processor == NULL || audio_processor->dec_input == NULL)
        return;

    // 不能用 portMAX_DELAY：WS 事件回调如果永久阻塞，STT/TTS_STOP 等控制帧全部收不到。
    // 最多重试 30 次（共 3 秒）：3 秒内扬声器仍无法消化则放弃本帧并告警，
    // 避免会话死锁，代价是极端 I2S 卡顿时轻微音频丢失。
    int retry = 0;
    while (audio_processor->is_running && retry < 30)
    {
        if (xRingbufferSend(audio_processor->dec_input, buffer, size, pdMS_TO_TICKS(100)) == pdTRUE)
            break;
        retry++;
    }
    if (retry >= 30)
        ESP_LOGW("AUDIO_PROC", "dec_input 持续满载 3s，丢弃 %d 字节 TTS 帧", (int)size);
}
/**
 * @brief 清空解码器输入和输出缓冲区
 *
 * 用于语音打断场景：用户说唤醒词打断 AI 说话时，
 * 立即清除所有积压的 TTS 音频数据，停止扬声器输出。
 */
void audio_processor_flush_output(audio_processor_t *audio_processor)
{
    size_t size;
    void *buf;
    /* 清空解码器输入（未解码的 OPUS 帧） */
    while ((buf = xRingbufferReceive(audio_processor->dec_input, &size, 0)) != NULL)
        vRingbufferReturnItem(audio_processor->dec_input, buf);
    /* 清空解码器输出（已解码但未播放的 PCM 数据） */
    while ((buf = xRingbufferReceive(audio_processor->dec_output, &size, 0)) != NULL)
        vRingbufferReturnItem(audio_processor->dec_output, buf);
    /* 置静音标志：play_task 在缓冲再次有数据时也不写 I2S，防止打断后新帧被重复播出 */
    audio_processor->mute_output = true;
}

/**
 * @brief 清空编码器输入和输出缓冲区（上行链路）
 *
 * 用于唤醒词打断场景：避免旧会话尚未发出的 PCM/OPUS 残留帧，
 * 在新 started 到来后被 ws_sender_task 当作新一轮的开头音频上传，
 * 污染新轮 ASR 识别结果。
 *
 * enc_input：BYTEBUF（PCM 字节流） → ReceiveUpTo 一次拉空
 * enc_output：NOSPLIT（OPUS 帧）   → Receive 循环逐帧释放
 */
void audio_processor_flush_input(audio_processor_t *audio_processor)
{
    if (audio_processor == NULL)
        return;

    size_t size;
    void *buf;

    /* 清空编码器输入（BYTEBUF：尚未编码的 PCM 字节流）*/
    if (audio_processor->enc_input != NULL)
    {
        while ((buf = xRingbufferReceiveUpTo(audio_processor->enc_input, &size, 0, SIZE_MAX)) != NULL)
            vRingbufferReturnItem(audio_processor->enc_input, buf);
    }

    /* 清空编码器输出（NOSPLIT：已编码但尚未发送的 OPUS 帧）*/
    if (audio_processor->enc_output != NULL)
    {
        while ((buf = xRingbufferReceive(audio_processor->enc_output, &size, 0)) != NULL)
            vRingbufferReturnItem(audio_processor->enc_output, buf);
    }
}

/**
 * @brief 带超时的 OPUS 数据读取
 *
 * 类似 audio_processor_read，但不会永久阻塞。
 * 供发送任务使用，超时后可以检查会话状态决定是否继续。
 *
 * @param timeout_ms 超时时间（毫秒），0 = 立即返回
 * @return 实际读取的字节数，0 表示超时或无数据
 */
size_t audio_processor_read_timeout(audio_processor_t *audio_processor,
                                    void *buffer, size_t size, uint32_t timeout_ms)
{
    size_t size_read = 0;
    void *buf_read = xRingbufferReceive(audio_processor->enc_output,
                                        &size_read, pdMS_TO_TICKS(timeout_ms));
    if (!buf_read)
        return 0;

    if (size_read > size)
    {
        ESP_LOGW(TAG, "Buffer size is too small");
        size_read = size;
    }

    memcpy(buffer, buf_read, size_read);
    vRingbufferReturnItem(audio_processor->enc_output, buf_read);
    return size_read;
}

/**
 * @brief 读取 AEC 回声消除参考 PCM（非阻塞，不足时零填充）
 *
 * play_task 写入 I2S 时同步向 aec_ref_buf 推送副本；本函数由
 * session.c 注册的 aec_ref_provider 回调调用，供 AFE AEC 使用。
 *
 * 实现：尝试从 aec_ref_buf 读取 samples 个采样点，
 *       读到多少算多少（BYTEBUF 可能分段返回），不足部分用零填充。
 *       零填充等价于"无播放声"，AEC 不会对该段做减法 → 安全退化。
 *
 * @param audio_processor 音频处理器实例指针
 * @param buf             输出缓冲区（int16_t，调用方分配）
 * @param samples         需要的采样点数
 */
void audio_processor_read_ref_pcm(audio_processor_t *audio_processor,
                                  int16_t *buf, size_t samples)
{
    size_t bytes_needed = samples * sizeof(int16_t);
    size_t bytes_filled = 0;

    while (bytes_filled < bytes_needed)
    {
        size_t sz = 0;
        void *item = xRingbufferReceiveUpTo(audio_processor->aec_ref_buf,
                                            &sz,
                                            0, /* 非阻塞 */
                                            bytes_needed - bytes_filled);
        if (!item)
            break; /* 缓冲区已空，剩余部分用零填充 */

        memcpy((uint8_t *)buf + bytes_filled, item, sz);
        vRingbufferReturnItem(audio_processor->aec_ref_buf, item);
        bytes_filled += sz;
    }

    /* 不足部分置零 → AEC 参考 = 静音 → 不做回声消除（安全退化）*/
    if (bytes_filled < bytes_needed)
        memset((uint8_t *)buf + bytes_filled, 0, bytes_needed - bytes_filled);
}

/**
 * @brief 查询下行解码链路当前积压的字节总数（dec_input + dec_output）
 *
 * 用于 TTS_STOP 后判定"扬声器是否真正排空"。
 * 当返回 0 时，意味着：
 *   - 没有未解码的 OPUS 帧待处理（dec_input 空）
 *   - 没有已解码未播放的 PCM 数据（dec_output 空）
 *   - 仅剩 I2S DMA 队列内极少量残留（< 50ms），由调用方再叠加物理静音兜底覆盖
 *
 * 实现：调用 vRingbufferGetInfo 获取每个环形缓冲当前 items_waiting，求和。
 *       该 API 是非阻塞快速查询，可在任意任务上下文调用。
 *
 * @param audio_processor 音频处理器实例指针
 * @return size_t         当前下行积压字节总数；NULL 入参或两者皆空返回 0
 */
// size_t audio_processor_get_pending_bytes(audio_processor_t *audio_processor)
// {
//     if (audio_processor == NULL)
//         return 0;

//     size_t dec_in_pending = 0;  // 解码器输入侧（OPUS 帧）积压
//     size_t dec_out_pending = 0; // 解码器输出侧（PCM 数据）积压

//     /* vRingbufferGetInfo 第 6 个参数返回当前已写入未读出的字节数（items_waiting）*/
//     if (audio_processor->dec_input != NULL)
//         vRingbufferGetInfo(audio_processor->dec_input, NULL, NULL, NULL, NULL, &dec_in_pending);
//     if (audio_processor->dec_output != NULL)
//         vRingbufferGetInfo(audio_processor->dec_output, NULL, NULL, NULL, NULL, &dec_out_pending);

//     return dec_in_pending + dec_out_pending;
// }

bool audio_processor_is_playing(audio_processor_t *audio_processor)
{
    if (audio_processor == NULL)
        return false;

    size_t dec_out_pending = 0; // 解码器输出侧（PCM 数据）积压

    /* vRingbufferGetInfo 第 6 个参数返回当前已写入未读出的字节数（items_waiting）*/
    if (audio_processor->dec_output != NULL)
        vRingbufferGetInfo(audio_processor->dec_output, NULL, NULL, NULL, NULL, &dec_out_pending);

    return dec_out_pending > 0;

    //   size_t dec_in_pending = 0;  // 解码器输入侧（未解码 OPUS 帧）积压
    // size_t dec_out_pending = 0; // 解码器输出侧（已解码 PCM 数据）积压

    // /* vRingbufferGetInfo 第 6 个参数返回当前已写入未读出的字节数（items_waiting）。
    //  * 必须同时查 dec_input + dec_output（BUG-013）：仅查 dec_output 时，
    //  * 若收尾阶段解码偶发被 CPU 抢占而短暂跟不上，dec_output 可能在 dec_input
    //  * 还压着未解码帧时先见底 → 误判"播完" → 提前切 LISTENING → 残音被当成下一轮输入。
    //  * dec_input 缓冲越大该窗口越大，故两侧都为空才算真正播完。 */
    // if (audio_processor->dec_input != NULL)
    //     vRingbufferGetInfo(audio_processor->dec_input, NULL, NULL, NULL, NULL, &dec_in_pending);
    // if (audio_processor->dec_output != NULL)
    //     vRingbufferGetInfo(audio_processor->dec_output, NULL, NULL, NULL, NULL, &dec_out_pending);

    // return (dec_in_pending + dec_out_pending) > 0;
}

void audio_processor_unmute_output(audio_processor_t *audio_processor)
{
    if (audio_processor == NULL)
        return;
    audio_processor->mute_output = false;
}
