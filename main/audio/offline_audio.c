/**
 * @file offline_audio.c
 * @brief 离线音频播放模块实现（外挂 flash /S → simple_dec 解码 → ES8311 扬声器）
 *
 * 数据流：
 *   fopen("/S/voice/xxx.mp3|.p3") → 分块读入 SPIRAM 缓冲
 *   → esp_audio_simple_dec（MP3 内置帧解析 / RAW_OPUS 整帧喂入）
 *   → PCM → esp_codec_dev_write() → 扬声器
 *
 * 与云端链路的隔离设计（关键，勿破坏）：
 *   - 解码器【库】与云端共用（同一注册表），但【实例】完全独立，
 *     绝不触碰 audio_decoder.c 的 audio_decoder_t 及其 RingBuffer
 *   - 注册遵循项目铁律"全局只注册一次、永不注销"（audio_decoder.c:244 同款，
 *     反复 register/unregister 会泄漏内部 SRAM）；重复注册是覆盖语义
 *     （esp_audio_dec_reg.h:47），与云端各自的注册互不影响
 *   - 任务栈 SPIRAM 分配，自删必须 vTaskDeleteWithCaps（BUG-011/023 铁律）
 */

#include <stdio.h>
#include <string.h>
#include <strings.h> // strcasecmp
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/idf_additions.h" // xTaskCreatePinnedToCoreWithCaps / vTaskDeleteWithCaps
#include "esp_log.h"
#include "esp_heap_caps.h"
#include "esp_codec_dev.h"

#include "esp_audio_simple_dec.h" // simple decoder 通用 API
#include "esp_mp3_dec.h"          // esp_mp3_dec_register()
#include "esp_opus_dec.h"         // esp_opus_dec_register() + esp_opus_dec_cfg_t

#include "offline_audio.h"
#include "bsp/bsp_config.h"   // BSP_CODEC_SAMPLE_RATE (16000)
#include "bsp/bsp_board.h"    // bsp_board_get_instance()->codec_dev
#include "session/session.h"  // session_get_state() 喇叭仲裁
#include "object.h"           // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM

#define TAG "[OfflineAudio]"

// ─── 任务与缓冲配置 ───────────────────────────────────────────────────────────
#define OFFLINE_TASK_STACK_SIZE 32768 ///< 32KB SPIRAM 栈（与云端编解码任务对齐，MP3 解码栈需求偏大，稳定优先）
#define OFFLINE_TASK_PRIORITY 5       ///< 与云端音频任务同级
#define OFFLINE_TASK_CORE_ID 0        ///< CPU0（与云端解码/播放同核，靠让步调度）
#define OFFLINE_READ_BUF_SIZE 8192    ///< 文件读缓冲（SPIRAM）：MP3 分块喂入解析器
#define OFFLINE_PCM_BUF_SIZE 8192     ///< PCM 输出缓冲（SPIRAM）：16k 单声道 60ms 帧仅 1920B，留足余量
#define OFFLINE_P3_MAX_PACKET 2048    ///< P3 单个 OPUS 包长度上限（24kbps/60ms 实际 ~180B，防脏数据越界）
#define OFFLINE_STOP_WAIT_MS 600      ///< 顶掉旧播放时等待旧任务退出的上限

// ─── 模块状态（单实例：同一时刻最多一条离线音频在播）──────────────────────────
static volatile bool s_playing = false;  ///< 播放任务存活标志（任务退出前最后清零）
static volatile bool s_stop_req = false; ///< 停止请求标志（play 顶掉/stop/会话让路 共用）

/**
 * @brief 解码器注册（全局一次，永不注销）
 *
 * MP3 与 OPUS 解码器 ops 注册进 esp_audio_codec 全局注册表。
 * 重复注册是覆盖同一 static const ops 指针，幂等无害；
 * 与云端 audio_decoder.c 的 esp_opus_dec_register() 互不冲突。
 */
static void offline_dec_register_once(void)
{
    static bool s_registered = false;
    if (s_registered)
        return;
#if OFFLINE_AUDIO_ENABLE_MP3
    esp_mp3_dec_register();
#endif
#if OFFLINE_AUDIO_ENABLE_P3
    esp_opus_dec_register();
#endif
    s_registered = true;
}

/**
 * @brief 首帧解码成功后校验音频参数（采样率/声道）
 *
 * codec 硬件固定 16kHz 单声道，素材不符会变速/变调（设备端不做重采样）。
 * 只警告不中断：声音仍可出，便于现场定位是素材问题而非链路问题。
 */
static void check_audio_info(esp_audio_simple_dec_handle_t dec)
{
    esp_audio_simple_dec_info_t info = {0};
    if (esp_audio_simple_dec_get_info(dec, &info) != ESP_AUDIO_ERR_OK)
        return;
    ESP_LOGI(TAG, "音频参数: %lu Hz, %d 声道, %d bit",
             (unsigned long)info.sample_rate, info.channel, info.bits_per_sample);
    if (info.sample_rate != BSP_CODEC_SAMPLE_RATE || info.channel != 1)
    {
        ESP_LOGW(TAG, "⚠️ 素材参数与 codec(16kHz/单声道)不符，会变速变调！"
                      "请用 ffmpeg -ar 16000 -ac 1 重新转换素材");
    }
}

/**
 * @brief 播放中的统一让路检查
 *
 * @return true = 应停止播放（收到 stop 请求，或会话已非空闲需让路给对话）
 */
static bool should_abort(void)
{
    if (s_stop_req)
        return true;
    // 唤醒词命中开启会话 → 立即让路，避免与 TTS 播放路径在 codec 写入上交织
    if (session_get_state() != SESSION_IDLE)
    {
        ESP_LOGI(TAG, "会话已激活，离线播放让路停止");
        return true;
    }
    return false;
}

/**
 * @brief 将一段 PCM 写入扬声器（阻塞直到 DMA 收完）
 * @return true 写入成功；false codec 未就绪
 */
static bool write_pcm(const uint8_t *pcm, uint32_t len)
{
    bsp_board_t *board = bsp_board_get_instance();
    if (!board || !board->codec_dev)
    {
        ESP_LOGE(TAG, "codec_dev 未初始化，无法播放");
        return false;
    }
    esp_codec_dev_write(board->codec_dev, (void *)pcm, len);
    return true;
}

#if OFFLINE_AUDIO_ENABLE_MP3
/**
 * @brief MP3 分支解码主循环（官方 simple_decoder_test.c 姿势）
 *
 * simple decoder 的 MP3 内置 ES 解析器：任意大小分块喂入，
 * 内部自动找帧边界并缓存不完整帧；raw.consumed 指示本次消费量。
 *
 * @param f       已打开的 MP3 文件
 * @param read_buf/pcm_buf 调用方分配的 SPIRAM 缓冲
 * @return ESP_OK 播完或被停止；其他 = 解码/参数错误
 */
static esp_err_t play_mp3_loop(FILE *f, uint8_t *read_buf, uint8_t *pcm_buf)
{
    // 打开 MP3 simple decoder（内置解析器模式：use_frame_dec=false）
    esp_audio_simple_dec_cfg_t cfg = {
        .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
        .dec_cfg = NULL,
        .cfg_size = 0,
        .use_frame_dec = false,
    };
    esp_audio_simple_dec_handle_t dec = NULL;
    esp_audio_err_t ret = esp_audio_simple_dec_open(&cfg, &dec);
    if (ret != ESP_AUDIO_ERR_OK)
    {
        ESP_LOGE(TAG, "MP3 解码器打开失败 ret=%d", ret);
        return ESP_FAIL;
    }

    esp_err_t result = ESP_OK;
    bool first_frame = true;
    while (!should_abort())
    {
        // ── 读一块原始 MP3 数据 ──────────────────────────────────────────
        size_t n = fread(read_buf, 1, OFFLINE_READ_BUF_SIZE, f);
        if (n == 0)
            break; // 文件读完（解析器内部缓存的残帧不足一帧，直接结束）

        esp_audio_simple_dec_raw_t raw = {
            .buffer = read_buf,
            .len = (uint32_t)n,
            .eos = (n < OFFLINE_READ_BUF_SIZE), // 最后一块置 eos，让解析器冲刷缓存
        };

        // ── 本块内可能含多帧：循环解码直到全部消费 ──────────────────────
        while (raw.len > 0 && !should_abort())
        {
            esp_audio_simple_dec_out_t out = {
                .buffer = pcm_buf,
                .len = OFFLINE_PCM_BUF_SIZE,
            };
            ret = esp_audio_simple_dec_process(dec, &raw, &out);
            if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH)
            {
                // 8KB 装不下一帧 PCM：素材不是 16k 单声道（如 48k 立体声）
                ESP_LOGE(TAG, "PCM 缓冲不足(需 %lu B)，素材参数超标，请重新转换",
                         (unsigned long)out.needed_size);
                result = ESP_FAIL;
                goto mp3_done;
            }
            if (ret != ESP_AUDIO_ERR_OK)
            {
                ESP_LOGE(TAG, "MP3 解码失败 ret=%d，中止播放", ret);
                result = ESP_FAIL;
                goto mp3_done;
            }
            if (out.decoded_size > 0)
            {
                if (first_frame)
                {
                    first_frame = false;
                    check_audio_info(dec); // 首帧校验采样率/声道
                }
                if (!write_pcm(pcm_buf, out.decoded_size))
                {
                    result = ESP_FAIL;
                    goto mp3_done;
                }
            }
            // 防呆：解析器既不消费也不出帧 → 等更多数据，跳出去读下一块
            if (raw.consumed == 0 && out.decoded_size == 0)
                break;
            raw.buffer += raw.consumed;
            raw.len -= raw.consumed;
            raw.consumed = 0;
        }
    }
mp3_done:
    esp_audio_simple_dec_close(dec);
    return result;
}
#endif // OFFLINE_AUDIO_ENABLE_MP3

#if OFFLINE_AUDIO_ENABLE_P3
/**
 * @brief P3 分支解码主循环
 *
 * P3 帧结构（小智格式，convert_audio_to_p3.py 产出）：
 *   [1B 类型][1B 保留][2B 长度(大端)] + OPUS 裸包
 * RAW_OPUS 类型要求整帧喂入（一次 process 恰好一个 OPUS 包）。
 *
 * @return ESP_OK 播完或被停止；其他 = 文件损坏/解码错误
 */
static esp_err_t play_p3_loop(FILE *f, uint8_t *read_buf, uint8_t *pcm_buf)
{
    // RAW_OPUS 裸流无文件头，参数必须显式给定（与 p3_tools 转换参数一致）
    esp_opus_dec_cfg_t opus_cfg = {
        .sample_rate = BSP_CODEC_SAMPLE_RATE, // 16000
        .channel = 1,
        .frame_duration = ESP_OPUS_DEC_FRAME_DURATION_60_MS,
        .self_delimited = false,
    };
    esp_audio_simple_dec_cfg_t cfg = {
        .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_RAW_OPUS,
        .dec_cfg = &opus_cfg,
        .cfg_size = sizeof(opus_cfg),
        .use_frame_dec = true, // 输入即完整帧，无需解析器
    };
    esp_audio_simple_dec_handle_t dec = NULL;
    esp_audio_err_t ret = esp_audio_simple_dec_open(&cfg, &dec);
    if (ret != ESP_AUDIO_ERR_OK)
    {
        ESP_LOGE(TAG, "OPUS 解码器打开失败 ret=%d", ret);
        return ESP_FAIL;
    }

    esp_err_t result = ESP_OK;
    bool first_frame = true;
    while (!should_abort())
    {
        // ── 读 4 字节 P3 帧头 ────────────────────────────────────────────
        uint8_t header[4];
        size_t n = fread(header, 1, sizeof(header), f);
        if (n == 0)
            break; // 正常播完
        if (n < sizeof(header))
        {
            ESP_LOGW(TAG, "P3 帧头不完整(%u B)，文件尾部截断，结束播放", (unsigned)n);
            break;
        }
        // header[0]=类型 header[1]=保留 header[2..3]=包长(大端)
        uint16_t packet_len = ((uint16_t)header[2] << 8) | header[3];
        if (packet_len == 0 || packet_len > OFFLINE_P3_MAX_PACKET)
        {
            ESP_LOGE(TAG, "P3 包长异常(%u B)，文件损坏或非 P3 格式，中止", packet_len);
            result = ESP_FAIL;
            break;
        }

        // ── 读出一个完整 OPUS 包 ─────────────────────────────────────────
        if (fread(read_buf, 1, packet_len, f) != packet_len)
        {
            ESP_LOGW(TAG, "P3 包体不完整，文件尾部截断，结束播放");
            break;
        }

        // ── 整帧解码（RAW_OPUS：一次一包）────────────────────────────────
        esp_audio_simple_dec_raw_t raw = {
            .buffer = read_buf,
            .len = packet_len,
        };
        esp_audio_simple_dec_out_t out = {
            .buffer = pcm_buf,
            .len = OFFLINE_PCM_BUF_SIZE,
        };
        ret = esp_audio_simple_dec_process(dec, &raw, &out);
        if (ret != ESP_AUDIO_ERR_OK)
        {
            // 单包损坏跳过继续（与云端解码器丢帧策略一致），连续失败靠包长校验兜底
            ESP_LOGW(TAG, "OPUS 包解码失败 ret=%d，跳过本包", ret);
            continue;
        }
        if (out.decoded_size > 0)
        {
            if (first_frame)
            {
                first_frame = false;
                check_audio_info(dec);
            }
            if (!write_pcm(pcm_buf, out.decoded_size))
            {
                result = ESP_FAIL;
                break;
            }
        }
    }
    esp_audio_simple_dec_close(dec);
    return result;
}
#endif // OFFLINE_AUDIO_ENABLE_P3

/**
 * @brief 离线播放任务（一次性：播完/出错/被停 → 清理资源自删）
 *
 * @param arg 堆上的路径字符串副本（本任务负责 free）
 */
static void offline_play_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG);
    char *path = (char *)arg;
    FILE *f = NULL;
    uint8_t *read_buf = NULL;
    uint8_t *pcm_buf = NULL;

    // ── 资源准备：读缓冲 + PCM 缓冲（均 SPIRAM）+ 打开文件 ─────────────────
    read_buf = heap_caps_malloc(OFFLINE_READ_BUF_SIZE, MALLOC_CAP_SPIRAM);
    pcm_buf = heap_caps_malloc(OFFLINE_PCM_BUF_SIZE, MALLOC_CAP_SPIRAM);
    if (!read_buf || !pcm_buf)
    {
        ESP_LOGE(TAG, "SPIRAM 缓冲分配失败");
        goto cleanup;
    }
    f = fopen(path, "rb");
    if (!f)
    {
        ESP_LOGE(TAG, "打不开音频文件: %s（确认已烧录到外挂 flash /S/voice/）", path);
        goto cleanup;
    }
    ESP_LOGI(TAG, "▶ 开始播放: %s", path);

    // ── 按扩展名分发解码分支 ─────────────────────────────────────────────
    {
        const char *ext = strrchr(path, '.');
        esp_err_t result = ESP_ERR_NOT_SUPPORTED;
#if OFFLINE_AUDIO_ENABLE_MP3
        if (ext && strcasecmp(ext, ".mp3") == 0)
            result = play_mp3_loop(f, read_buf, pcm_buf);
#endif
#if OFFLINE_AUDIO_ENABLE_P3
        if (ext && strcasecmp(ext, ".p3") == 0)
            result = play_p3_loop(f, read_buf, pcm_buf);
#endif
        ESP_LOGI(TAG, "■ 播放结束: %s (%s)", path,
                 result == ESP_OK ? (s_stop_req ? "被停止" : "正常播完") : "出错中止");
    }

cleanup:
    // ── 资源回收（与分配严格配对）────────────────────────────────────────
    if (f)
        fclose(f);
    if (read_buf)
        free(read_buf);
    if (pcm_buf)
        free(pcm_buf);
    free(path); // play() 中分配的路径副本

    s_playing = false; // 必须在自删前最后置位，play() 靠它判断旧任务已退出
    // ★ SPIRAM 栈任务自删铁律：必须 WithCaps，否则栈+TCB 泄漏（BUG-011/023）
    vTaskDeleteWithCaps(NULL);
}

// ═══════════════════════════════════════════════════════════════════════════════
// 公开 API
// ═══════════════════════════════════════════════════════════════════════════════

esp_err_t offline_audio_play(const char *path)
{
    // ── 参数与格式检查 ───────────────────────────────────────────────────
    if (!path || path[0] == '\0')
        return ESP_ERR_INVALID_ARG;
    const char *ext = strrchr(path, '.');
    if (!ext)
        return ESP_ERR_INVALID_ARG;
    bool is_mp3 = (strcasecmp(ext, ".mp3") == 0);
    bool is_p3 = (strcasecmp(ext, ".p3") == 0);
    if (!is_mp3 && !is_p3)
    {
        ESP_LOGE(TAG, "不支持的扩展名: %s（仅 .mp3 / .p3）", ext);
        return ESP_ERR_INVALID_ARG;
    }
    // 对应分支被编译裁剪时明确报错，而不是静默无声
    if ((is_mp3 && !OFFLINE_AUDIO_ENABLE_MP3) || (is_p3 && !OFFLINE_AUDIO_ENABLE_P3))
    {
        ESP_LOGE(TAG, "格式 %s 的分支已被编译裁剪（见 offline_audio.h 开关）", ext);
        return ESP_ERR_NOT_SUPPORTED;
    }

    // ── 喇叭仲裁：对话中拒绝（不排队不打断，v1 从简）─────────────────────
    if (session_get_state() != SESSION_IDLE)
    {
        ESP_LOGW(TAG, "会话进行中，拒绝离线播放: %s", path);
        return ESP_ERR_INVALID_STATE;
    }

    // ── 顶掉语义：停旧任务并等其退出（释放 codec 写入权后再开新的）───────
    if (s_playing)
    {
        s_stop_req = true;
        int waited = 0;
        while (s_playing && waited < OFFLINE_STOP_WAIT_MS)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            waited += 10;
        }
        if (s_playing)
        {
            // 理论不该发生：解码循环每帧都查 stop 标志（60ms 粒度）
            ESP_LOGE(TAG, "旧播放任务 %d ms 未退出，放弃本次播放", OFFLINE_STOP_WAIT_MS);
            return ESP_ERR_TIMEOUT;
        }
    }

    // ── 解码器注册（全局一次）+ 路径副本 + 创建播放任务 ──────────────────
    offline_dec_register_once();

    size_t path_len = strlen(path) + 1;
    char *path_copy = heap_caps_malloc(path_len, MALLOC_CAP_SPIRAM);
    if (!path_copy)
        return ESP_ERR_NO_MEM;
    memcpy(path_copy, path, path_len);

    s_stop_req = false;
    s_playing = true; // 先置位再创建，防止并发 play 竞态
    BaseType_t ok = xTaskCreatePinnedToCoreWithCaps(
        offline_play_task, "offline_audio",
        OFFLINE_TASK_STACK_SIZE, path_copy,
        OFFLINE_TASK_PRIORITY, NULL,
        OFFLINE_TASK_CORE_ID, MALLOC_CAP_SPIRAM);
    if (ok != pdPASS)
    {
        s_playing = false;
        free(path_copy);
        ESP_LOGE(TAG, "播放任务创建失败（SPIRAM 不足？）");
        return ESP_ERR_NO_MEM;
    }
    PRINT_TASK_CREATED(TAG, "offline_audio", OFFLINE_TASK_STACK_SIZE, 0);
    return ESP_OK;
}

void offline_audio_stop(void)
{
    if (s_playing)
        s_stop_req = true;
}

bool offline_audio_is_playing(void)
{
    return s_playing;
}
