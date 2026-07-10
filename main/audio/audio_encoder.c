#include "audio_encoder.h"
#include "esp_audio_enc.h"
#include "esp_audio_enc_default.h"
#include "esp_audio_enc_reg.h"
#include "object.h"
#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_heap_caps.h"
#include "esp_err.h"
#include "esp_log.h"
#include <string.h>

#define TAG "[AP] Encoder"

// ─── 编码器任务配置宏 ─────────────────────────────────────────────────────────
#define AUDIO_ENCODER_TASK_CORE_ID 1        ///< 编码任务绑定 CPU1（与 audio_feed_task 同核共享，靠 vTaskDelay/taskYIELD 让步）
#define AUDIO_ENCODER_TASK_STACK_SIZE 32768 ///< 栈大小 32KB（OPUS 编码运算需要较大栈，含 FFT 等中间状态）
#define AUDIO_ENCODER_TASK_PRIORITY 5       ///< 优先级 5（与解码器、播放任务对称）

/**
 * @brief 编码器内部结构体（对外不透明，通过 audio_encoder_t* 访问）
 *
 * 字段说明：
 *   input_buffer  : BYTEBUF 类型，PCM 数据可自由拼接（无帧边界约束）
 *   output_buffer : NOSPLIT 类型，OPUS 帧必须整帧写入（有帧边界约束）
 *   enc            : OPUS 编码器句柄（esp_audio_codec 框架）
 *   is_running     : 任务退出标志（false 时任务自删除）
 */
struct audio_encoder
{
    RingbufHandle_t input_buffer;      ///< 输入缓冲区：接收来自 AFE 降噪后的 PCM 数据
    RingbufHandle_t output_buffer;     ///< 输出缓冲区：存放编码完成的 OPUS 帧
    esp_audio_enc_handle_t enc;        ///< OPUS 编码器句柄（esp_audio_codec 框架管理）
    volatile bool is_running;          ///< 任务运行标志：volatile 确保跨任务可见，stop() 设 false 后任务循环退出
    volatile TaskHandle_t task_handle; ///< 任务句柄，用于超时强制终止和确认退出
    // ── 帧缓冲（在 start() 中一次性分配，stop() 中统一释放，避免强杀任务时堆泄漏） ──
    void *in_buf;       ///< PCM 凑帧缓冲（16 字节对齐，SPIRAM）
    void *out_buf;      ///< OPUS 输出缓冲（16 字节对齐，SPIRAM）
    int in_frame_size;  ///< 每帧 PCM 字节数（编码器查询所得，固定值）
    int out_frame_size; ///< OPUS 输出最大字节数
    // ── 运行时状态计数器（迁出 static 局部变量，避免 stop→start 后状态残留） ──
    int frames_in_rush;          ///< 紧急态强制让步计数（积压时使用）
    int accum_segments;          ///< 凑帧段计数（紧循环防饿死用）
    int pending_check_cnt;       ///< 每 8 帧采样一次 pending 字节
    size_t pending_bytes_cached; ///< 上次采样的 pending 字节数（节流 vRingbufferGetInfo）
    int drop_cnt;                ///< enc_output 丢帧累计（每 1 秒汇总打印）
};

// ═══════════════════════════════════════════════════════════════════════════════
// 编码器任务（核心编码循环）
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief OPUS 音频编码主任务（持续将 PCM 帧编码为 OPUS 帧）
 *
 * 数据流：
 *   [enc_input: BYTEBUF] → 分段读取凑够一帧 PCM → esp_audio_enc_process() → [enc_output: NOSPLIT]
 *
 * 关键设计细节：
 *   1. PCM 帧可能跨多次 xRingbufferReceiveUpTo 才能凑齐（BYTEBUF 分段读）
 *   2. OPUS 编码结果写入 NOSPLIT 缓冲区（整帧写入，下游一次性读取）
 *   3. 输出缓冲区满时丢帧（不阻塞），保证实时性不卡顿
 *
 * @param arg audio_encoder_t* 实例指针（类型转换后使用）
 * @return 无（任务运行直到 is_running 为 false）
 *
 * @note 调用者：audio_encoder_start() 通过 xTaskCreatePinnedToCoreWithCaps() 创建
 * @note 运行核心：CPU1（与 audio_feed_task 同核），栈 32KB SPIRAM，优先级 5
 */
void audio_encoder_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    audio_encoder_t *audio_encoder = (audio_encoder_t *)arg;

    // ── 帧缓冲已由 audio_encoder_start() 预分配在结构体中 ───────────────────
    // 这样即使任务被 stop() 强杀（vTaskDeleteWithCaps），缓冲也由 stop() 统一释放，避免堆泄漏。
    // OPUS 20ms 帧 @ 16kHz 单声道 16-bit = 320采样点 × 2字节 = 640 字节 PCM
    const int frame_size_total = audio_encoder->in_frame_size; // 一帧需要的固定字节数
    int frame_remaining = frame_size_total;                    // 本帧剩余待读字节数（消耗变量）
    uint8_t *write_ptr = (uint8_t *)audio_encoder->in_buf;     // 当前帧写入位置（指针算术用 uint8_t*）

    // ── 帧描述符：传给 esp_audio_enc_process 的固定结构 ─────────────────────
    esp_audio_enc_in_frame_t in_frame = {
        .buffer = audio_encoder->in_buf,
        .len = frame_size_total,
    };
    esp_audio_enc_out_frame_t out_frame = {
        .buffer = audio_encoder->out_buf,
        .len = audio_encoder->out_frame_size,
    };

    // ── 编码主循环 ──────────────────────────────────────────────────────────
    // 让步策略说明：
    //   - 凑帧期间：每 4 段主动 vTaskDelay(1)，避免紧循环饿死 IDLE0/WDT
    //   - 编码完成后：
    //       正常态（积压 < 1.5s）→ vTaskDelay(1) 让 IDLE 喂狗
    //       紧急态（积压 ≥ 1.5s）→ taskYIELD() 全速追赶，每 50 帧强制 vTaskDelay(1) 保 IDLE0
    while (audio_encoder->is_running)
    {
        // 从输入缓冲区读取一段 PCM 数据（BYTEBUF 类型，可能不足一帧）
        size_t size_read = 0;
        void *buf_read = xRingbufferReceiveUpTo(
            audio_encoder->input_buffer,
            &size_read,
            pdMS_TO_TICKS(100), // 超时 100ms：无数据时检查 is_running 标志
            frame_remaining);   // 最多读 frame_remaining 字节，避免越界

        if (!buf_read)
            continue; // 超时无数据（静音期），继续等待下一帧

        // 将读到的 PCM 片段追加到帧缓冲区
        memcpy(write_ptr, buf_read, size_read);

        // 归还环形缓冲区的使用权（BYTEBUF 读完必须立即归还）
        vRingbufferReturnItem(audio_encoder->input_buffer, buf_read);

        // 扣减剩余字节，前移写指针
        frame_remaining -= size_read;
        write_ptr += size_read;

        if (frame_remaining > 0)
        {
            // 当前帧未凑满，继续读取下一段
            if (++audio_encoder->accum_segments >= 4)
            {
                audio_encoder->accum_segments = 0;
                vTaskDelay(1); // 主动让步，防止饿死 IDLE0/WDT
            }
            continue;
        }
        audio_encoder->accum_segments = 0; // 凑齐一帧后重置段计数

        // ── 一帧 PCM 已凑齐，重置指针为下一帧准备 ───────────────────────────
        write_ptr = (uint8_t *)in_frame.buffer;
        frame_remaining = frame_size_total;

        // 调用 OPUS 编码器处理一帧
        esp_audio_enc_process(audio_encoder->enc, &in_frame, &out_frame);

        // 将编码结果写入输出缓冲区（NOSPLIT：整帧原子写入；超时 0：满了立即丢帧）
        BaseType_t ret = xRingbufferSend(
            audio_encoder->output_buffer,
            out_frame.buffer,
            out_frame.encoded_bytes,
            0);

        if (ret == pdFAIL)
        {
            // Opt-9：累计计数，每 50 帧（≈1 秒）汇总一次，避免日志刷屏
            if (++audio_encoder->drop_cnt >= 50)
            {
                ESP_LOGW(TAG, "enc_output 持续阻塞，1 秒内丢弃 %d 帧 OPUS",
                         audio_encoder->drop_cnt);
                audio_encoder->drop_cnt = 0;
            }
        }

        // ── 自适应让步策略（Opt-4：节流 vRingbufferGetInfo，每 8 帧采样一次）──
        if (++audio_encoder->pending_check_cnt >= 8)
        {
            audio_encoder->pending_check_cnt = 0;
            vRingbufferGetInfo(audio_encoder->input_buffer, NULL, NULL, NULL, NULL,
                               &audio_encoder->pending_bytes_cached);
        }
        if (audio_encoder->pending_bytes_cached >= 48000) // 1.5s × 16kHz × 2B
        {
            if (++audio_encoder->frames_in_rush >= 50)
            {
                audio_encoder->frames_in_rush = 0;
                vTaskDelay(1); // 紧急态安全网：1 秒强制让步一次保 IDLE0
            }
            else
            {
                taskYIELD(); // 紧急态：仅切给同优先级 ready 任务
            }
        }
        else
        {
            audio_encoder->frames_in_rush = 0;
            vTaskDelay(1); // 正常态：让 IDLE0 喂狗
        }
    }

    // ── 任务退出：清空句柄，自删除（帧缓冲由 stop() 统一释放） ──────────────
    audio_encoder->task_handle = NULL; // 通知 stop() 任务已安全退出
    vTaskDeleteWithCaps(NULL);         // ★ 必须 WithCaps 才能回收 SPIRAM 栈
}

// ═══════════════════════════════════════════════════════════════════════════════
// 生命周期管理
// ═══════════════════════════════════════════════════════════════════════════════

/**
 * @brief 创建 OPUS 编码器实例，注册并配置编码参数
 *
 * @param sample_rate 采样率（Hz），应为 BSP_CODEC_SAMPLE_RATE（16000）
 * @param channels    声道数，应为 1（单声道）
 * @return audio_encoder_t* 编码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 * @note 创建后需调用 audio_encoder_set_buffer() 绑定缓冲区，再调用 audio_encoder_start()
 */
audio_encoder_t *audio_encoder_create(int sample_rate, int channels)
{
    // ── 步骤 1：分配并清零编码器结构体（Bug-6：检查失败立即返回 NULL）────────
    audio_encoder_t *audio_encoder = malloc_zeroed(sizeof(audio_encoder_t));
    if (audio_encoder == NULL)
    {
        ESP_LOGE(TAG, "编码器结构体分配失败（%u 字节）",
                 (unsigned)sizeof(audio_encoder_t));
        return NULL;
    }

    // ── 步骤 2：向 esp_audio_codec 框架注册 OPUS 编码器（全局只需注册一次）──
    // 注册后框架可识别 ESP_AUDIO_TYPE_OPUS 类型，后续 open() 才能找到对应实现
    // ★ 使用静态标志确保只注册一次，避免反复 register/unregister 导致内部 SRAM 泄漏（每次约 0.7KB）
    {
        static bool s_opus_enc_registered = false;
        if (!s_opus_enc_registered)
        {
            ESP_ERROR_CHECK(esp_opus_enc_register());
            s_opus_enc_registered = true;
        }
    }

    // ── 步骤 3：配置 OPUS 编码参数 ────────────────────────────────────────────
    esp_opus_enc_config_t opus_config = {
        .sample_rate = sample_rate,                   // 采样率：16000 Hz
        .bits_per_sample = BSP_CODEC_BITS_PER_SAMPLE, // 位深：16-bit
        .channel = channels,                          // 声道：1（单声道）
        // 比特率：24kbps（实测稳定；过往尝试 16kbps 提升 CPU 余量但语音质量略损，最终回到 24k）
        .bitrate = 24000,
        // 帧时长越长编码效率越高但延迟越大，60ms 是 20ms 的 3 倍，CPU 占用约降到 20ms 的 1/3，适合对延迟要求不苛刻的语音交互场景）
        // 帧时长为20ms,就会一次fps=50的速率产生OPUS帧,如果网络状况不佳或者对方处理能力有限,可能会导致积压和丢帧.60ms的帧时长可以降低编码频率,减少CPU占用,同时在网络抖动时提供更好的缓冲能力,适合对延迟要求不苛刻的语音交互场景。
        // 帧时长为60ms,每帧包含960采样点(16kHz * 0.06s),相较于20ms的320采样点,可以提供更高的编码效率和更好的语音质量,同时降低CPU占用,适合对延迟要求不苛刻的语音交互场景。
        .frame_duration = ESP_OPUS_ENC_FRAME_DURATION_60_MS, // 帧时长：60ms（960采样点）,
                                                             // 复杂度：0（原 3，SILK 在 complexity≥2 启用 delayed-decision NSQ，CPU 占用翻倍；
                                                             // 0 改用简单 NSQ，帧内耗时更平稳，适合 CPU 紧张场景）
        .complexity = 0,
        .application_mode = ESP_OPUS_ENC_APPLICATION_VOIP, // 模式：VOIP（针对语音优化）
        .enable_fec = false,                               // 禁用前向纠错（WiFi 无线不需要，有线更稳定）
        .enable_dtx = false,                               // 禁用不连续传输（保持连续流，避免静音期丢帧）
        .enable_vbr = false,                               // 禁用可变比特率（固定码率，保证延迟稳定性）
    };

    // 封装为通用编码器配置结构体（框架层接口）
    esp_audio_enc_config_t config = {
        .cfg = &opus_config,
        .cfg_sz = sizeof(esp_opus_enc_config_t),
        .type = ESP_AUDIO_TYPE_OPUS,
    };

    // ── 步骤 4：打开编码器（Bug-6：软失败，不再用 ESP_ERROR_CHECK abort 整机）
    esp_err_t err = esp_audio_enc_open(&config, &audio_encoder->enc);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "OPUS 编码器打开失败: %s", esp_err_to_name(err));
        free(audio_encoder);
        return NULL;
    }

    return audio_encoder;
}

/**
 * @brief 销毁编码器实例（关闭句柄 → 注销类型 → 释放内存）
 *
 * @param audio_encoder 编码器实例指针（调用后不可再使用）
 *
 * @note 调用者：audio_processor.c → audio_processor_destroy()
 */
void audio_encoder_destroy(audio_encoder_t *audio_encoder)
{
    // 关闭编码器：释放 OPUS 运行时内部状态（编码缓冲区、滤波器状态等）
    esp_audio_enc_close(audio_encoder->enc);

    // ★ 不再注销 OPUS 编码器类型：全局只注册一次，避免反复 register/unregister 泄漏内部 SRAM
    // esp_audio_enc_unregister(ESP_AUDIO_TYPE_OPUS);

    // 释放编码器结构体本身
    free(audio_encoder);
}

/**
 * @brief 绑定输入/输出环形缓冲区（供编码任务读写数据）
 *
 * @param audio_encoder  编码器实例指针
 * @param input_buffer   PCM 输入缓冲区（BYTEBUF 类型，来自 audio_processor_create）
 * @param output_buffer  OPUS 输出缓冲区（NOSPLIT 类型，来自 audio_processor_create）
 *
 * @note 调用者：audio_processor.c → audio_processor_create()
 */
void audio_encoder_set_buffer(audio_encoder_t *audio_encoder,
                              RingbufHandle_t input_buffer,
                              RingbufHandle_t output_buffer)
{
    audio_encoder->input_buffer = input_buffer;   // PCM 数据来源（麦克风采集链路末端）
    audio_encoder->output_buffer = output_buffer; // OPUS 数据去向（WebSocket 发送链路入口）
}

/**
 * @brief 启动编码任务（设置运行标志，创建 FreeRTOS 任务）
 *
 * @param audio_encoder 编码器实例指针（已绑定缓冲区）
 *
 * @note 调用者：audio_processor.c → audio_processor_start()
 */
void audio_encoder_start(audio_encoder_t *audio_encoder)
{
    // ── 前置检查：确保缓冲区已绑定 ────────────────────────────────────────────
    if (!audio_encoder->input_buffer || !audio_encoder->output_buffer)
    {
        ESP_LOGW(TAG, "编码器缓冲区未设置，拒绝启动任务");
        return;
    }

    // ── 步骤 1：查询每帧 PCM/OPUS 字节数（OPUS 20ms@16kHz mono = 640 字节 PCM）
    int in_frame_size = 0, out_frame_size = 0;
    esp_audio_enc_get_frame_size(audio_encoder->enc, &in_frame_size, &out_frame_size);
    audio_encoder->in_frame_size = in_frame_size;
    audio_encoder->out_frame_size = out_frame_size;

    // ── 步骤 2：Opt-2 + Opt-10：用 16 字节对齐分配帧缓冲（适配 ESP32-S3 PIE SIMD）
    // 提升到结构体字段后，任务被强杀时 stop() 仍能正确释放，避免堆泄漏（配合 Bug-7 修复）
    audio_encoder->in_buf = heap_caps_aligned_alloc(
        16, in_frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    audio_encoder->out_buf = heap_caps_aligned_alloc(
        16, out_frame_size, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!audio_encoder->in_buf || !audio_encoder->out_buf)
    {
        ESP_LOGE(TAG, "帧缓冲分配失败（in=%d, out=%d）", in_frame_size, out_frame_size);
        heap_caps_free(audio_encoder->in_buf);
        heap_caps_free(audio_encoder->out_buf);
        audio_encoder->in_buf = NULL;
        audio_encoder->out_buf = NULL;
        return;
    }
    memset(audio_encoder->in_buf, 0, in_frame_size);
    memset(audio_encoder->out_buf, 0, out_frame_size);

    // ── 步骤 3：Opt-3 + Opt-4 + Opt-9：清零运行时计数器（避免 stop→start 残留状态）
    audio_encoder->accum_segments = 0;
    audio_encoder->frames_in_rush = 0;
    audio_encoder->pending_check_cnt = 0;
    audio_encoder->pending_bytes_cached = 0;
    audio_encoder->drop_cnt = 0;

    // ── 步骤 4：设置运行标志，创建任务 ───────────────────────────────────────
    audio_encoder->is_running = true;

    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        audio_encoder_task,                          // 任务函数
        "encoder_task",                              // 任务名称
        AUDIO_ENCODER_TASK_STACK_SIZE,               // 栈大小：32KB
        audio_encoder,                               // 任务参数：编码器实例指针
        AUDIO_ENCODER_TASK_PRIORITY,                 // 优先级：5
        (TaskHandle_t *)&audio_encoder->task_handle, // 保存句柄，用于超时强制终止
        AUDIO_ENCODER_TASK_CORE_ID,                  // 绑定核心：CPU1
        MALLOC_CAP_SPIRAM);                          // 栈内存来源：外部 SPIRAM

    if (ret == pdPASS)
        PRINT_TASK_CREATED(TAG, "encoder_task", AUDIO_ENCODER_TASK_STACK_SIZE, 0); // 栈在PSRAM
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "编码任务创建失败（内存不足或参数错误）");
        audio_encoder->is_running = false;
        audio_encoder->task_handle = NULL;
        // 回滚已分配的帧缓冲
        heap_caps_free(audio_encoder->in_buf);
        heap_caps_free(audio_encoder->out_buf);
        audio_encoder->in_buf = NULL;
        audio_encoder->out_buf = NULL;
    }
}

/**
 * @brief 停止编码任务（清除运行标志，等待任务自然退出）
 *
 * @param audio_encoder 编码器实例指针
 *
 * @note 调用者：audio_processor.c → audio_processor_stop()
 * @note 200ms 等待时间基于任务内 100ms 超时读取，确保任务有机会检测到 is_running=false
 */
void audio_encoder_stop(audio_encoder_t *audio_encoder)
{
    // 清除运行标志：任务在下次 xRingbufferReceiveUpTo 超时（100ms）后检测到并退出
    audio_encoder->is_running = false;

    // 等待任务自然退出（最多 400ms：100ms 读超时 + 编码耗时 + 余量）
    for (int i = 0; i < 20 && audio_encoder->task_handle != NULL; i++)
        vTaskDelay(pdMS_TO_TICKS(20));

    // Bug-7：超时仍未退出 → 必须用 vTaskDeleteWithCaps 才能回收 SPIRAM 栈（32KB）
    // 普通 vTaskDelete 不会释放 caps 分配的栈，每次强杀都会泄漏一整个栈空间
    if (audio_encoder->task_handle != NULL)
    {
        ESP_LOGW(TAG, "编码任务超时未退出，强制终止以释放 SPIRAM 栈");
        TaskHandle_t h = audio_encoder->task_handle;
        audio_encoder->task_handle = NULL;
        vTaskDeleteWithCaps(h); // ★ WithCaps 版本才能释放 caps 栈
    }

    // 兜底释放帧缓冲（无论正常退出还是强杀，缓冲都在结构体字段中）
    if (audio_encoder->in_buf)
    {
        heap_caps_free(audio_encoder->in_buf);
        audio_encoder->in_buf = NULL;
    }
    if (audio_encoder->out_buf)
    {
        heap_caps_free(audio_encoder->out_buf);
        audio_encoder->out_buf = NULL;
    }
}
