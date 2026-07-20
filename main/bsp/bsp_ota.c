/**
 * @file bsp_ota.c
 * @brief OTA 固件升级实现（混合方案：esp_https_ota + 两阶段版本提交）
 */

#include "bsp_ota.h"
#include "esp_https_ota.h"
#include "esp_app_desc.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
#include "esp_log.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_tls.h" // 【临时调试】抠底层 TLS 错误码
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "mbedtls/sha256.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <strings.h> // strcasecmp
#include <stdio.h>   // sprintf
#include <stdlib.h>
#include "object.h"     // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM
#include "ui/standby.h" // standby_notify_activity：OTA 期间持续喂计时器，防止进深度待机
#include "ui/ui_port.h" // ui_show_ota_progress / ui_pause_main_gif：升级中屏幕反馈
#include "session/session.h" // session_stop_for_ota：升级前彻底停语音链路，腾内部 SRAM/CPU 给下载
#include "bsp/bsp_board.h" // bsp_battery_stop_task / bsp_battery_stop_log_task / bsp_touch_stop_for_ota
#include "wake_word/custom_wake_word.h" // bsp_wake_word_stop_for_ota：升级前停唤醒引擎+麦克风采集
#include "ui/interaction.h" // interaction_stop_for_ota：停 GIF 切图 + 舵机 + 震动（触摸随之哑火）

#define TAG "OTA"

// NVS 命名空间与键名
#define NVS_NS_OTA "ota"                  ///< OTA 命名空间
#define NVS_KEY_PENDING_VER "pending_ver" ///< 待验证的新版本号（升级中）
#define NVS_KEY_PENDING_URL "pending_url" ///< 待验证的固件 URL
#define NVS_KEY_COMMITTED "ota_version"   ///< 已提交的版本号（正常运行中）
#define NVS_KEY_SUCCESS_VER "succ_ver"    ///< 升级成功待上报的版本号（重启后补发 success 用）

// 进度上报节流参数
#define OTA_REPORT_MIN_STEP 5   ///< 进度每变化 ≥5% 上报一次
#define OTA_REPORT_MIN_MS 1000  ///< 或距上次上报 ≥1 秒上报一次
#define OTA_SHA_READ_CHUNK 4096 ///< 回读分区算 SHA256 的分块大小（不放栈）

/**
 * @brief OTA 任务参数结构体（在堆上传递，任务内部释放）
 */
typedef struct
{
    char url[512];    ///< 固件下载地址
    char version[32]; ///< 新版本号
    char sha256[65];  ///< 期望 SHA256（64 hex + 结束符）；空串表示不校验
    uint32_t size;    ///< 固件字节数（progress 分母）；0 表示未知
} ota_params_t;

/**
 * @brief 进度回调（在 protocol_mqtt_start 注册）
 */
static bsp_ota_progress_cb_t s_progress_cb = NULL;

/** 注册进度回调 */
void bsp_ota_register_progress_cb(bsp_ota_progress_cb_t cb)
{
    s_progress_cb = cb;
}

/** 触发进度回调（回调未注册时静默）。err_msg 仅 FAILED 用，其余传 NULL */
static inline void ota_report(bsp_ota_status_t status, int progress,
                              const char *version, const char *err_msg)
{
    if (s_progress_cb)
        s_progress_cb(status, progress, version, err_msg);
}

/**
 * @brief 比较两个语义化版本号，判断 new_ver 是否比 cur 更新
 *
 * 算法：按 '.' 拆分逐段比较数字大小，前面段相同时段数多的为新。
 * 例如："1.2.3" < "1.2.4" < "1.3.0" < "1.3.0.1"
 *
 * @param cur     当前版本字符串
 * @param new_ver 待比较的新版本字符串
 * @return true   new_ver 比 cur 更新
 * @return false  new_ver <= cur，或参数无效
 */
static bool is_newer_version(const char *cur, const char *new_ver)
{
    if (!cur || !new_ver) // 参数无效
        return false;

    char cur_buf[32], new_buf[32];
    strncpy(cur_buf, cur, sizeof(cur_buf) - 1);
    cur_buf[sizeof(cur_buf) - 1] = '\0';
    strncpy(new_buf, new_ver, sizeof(new_buf) - 1);
    new_buf[sizeof(new_buf) - 1] = '\0';

    char *cur_save = NULL, *new_save = NULL;
    // strtok_r三个参数：字符串，分隔符，保存状态的指针
    char *cur_tok = strtok_r(cur_buf, ".", &cur_save); // 按 '.' 拆分
    char *new_tok = strtok_r(new_buf, ".", &new_save);

    while (cur_tok && new_tok)
    {
        int c = atoi(cur_tok); // atoi的参数：字符串转换成数字
        int n = atoi(new_tok);
        if (n > c) // 新版本号比旧版本号大
            return true;
        if (n < c)
            return false;
        cur_tok = strtok_r(NULL, ".", &cur_save); // 含义：继续拆分
        new_tok = strtok_r(NULL, ".", &new_save);
    }
    // 前段全等：段数多的版本视为更新（如 1.0.0 < 1.0.0.1）
    return (new_tok != NULL);
}

// ─────────────────────────────────────────────────────────────────────────
// NVS 操作（两阶段提交）
// ─────────────────────────────────────────────────────────────────────────

/**
 * @brief 写入 NVS pending 版本信息（升级前调用）
 *
 * 用途：即使断电，下次启动也能从 NVS 知道"曾尝试升级到 X 版本"。
 * 升级成功并 mark_valid 后会被提升为 committed。
 */
static void stage_pending(const char *version, const char *url)
{
    nvs_handle_t h;
    esp_err_t err = nvs_open(NVS_NS_OTA, NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "nvs_open(ota) 失败: %s", esp_err_to_name(err));
        return;
    }
    nvs_set_str(h, NVS_KEY_PENDING_VER, version);
    nvs_set_str(h, NVS_KEY_PENDING_URL, url);
    nvs_commit(h); // 同步写入
    nvs_close(h);  /// 关闭 NVS 句柄
}

/**
 * @brief 写入"升级成功待上报"标记（校验通过、重启前调用）
 *
 * 升级成功必须重启才能跑新固件，而重启会断开 MQTT，重启前发的 success
 * 很可能送不达。因此先把版本号落 NVS，等新固件重连成功后再补发 success。
 */
static void write_pending_success(const char *version)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_OTA, NVS_READWRITE, &h) != ESP_OK)
    {
        ESP_LOGW(TAG, "nvs_open(ota) 失败，无法记录 success 标记");
        return;
    }
    nvs_set_str(h, NVS_KEY_SUCCESS_VER, version);
    nvs_commit(h);
    nvs_close(h);
}

/** 取出"升级成功待上报"标记（重启后由 MQTT 重连成功时调用） */
bool bsp_ota_take_pending_success(char *out_version, size_t out_size)
{
    if (!out_version || out_size == 0)
        return false;

    nvs_handle_t h;
    if (nvs_open(NVS_NS_OTA, NVS_READONLY, &h) != ESP_OK)
        return false; // 命名空间不存在 = 无标记

    size_t len = out_size;
    esp_err_t err = nvs_get_str(h, NVS_KEY_SUCCESS_VER, out_version, &len);
    nvs_close(h);
    return (err == ESP_OK);
}

/** 清除"升级成功待上报"标记（success 上报后调用） */
void bsp_ota_clear_pending_success(void)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NS_OTA, NVS_READWRITE, &h) != ESP_OK)
        return;
    nvs_erase_key(h, NVS_KEY_SUCCESS_VER);
    nvs_commit(h);
    nvs_close(h);
}

/**
 * @brief 回读备用分区计算 SHA256，与期望值比对
 *
 * 分步 OTA 是边下载边写入备用分区的，下完后回读分区算指纹即可校验完整性。
 * 注意：读取范围必须精确等于固件实际字节数，多读分区尾部 0xFF 会导致校验失败。
 *
 * @param part        备用（更新）分区
 * @param fw_size     固件实际字节数
 * @param expect_hex  期望的 SHA256（64 位小写十六进制字符串）
 * @return true 校验通过；false 不通过或出错
 */
static bool verify_partition_sha256(const esp_partition_t *part, size_t fw_size, const char *expect_hex)
{
    if (!part || fw_size == 0 || !expect_hex || strlen(expect_hex) != 64)
    {
        ESP_LOGE(TAG, "SHA256 校验参数无效（fw_size=%u）", (unsigned)fw_size);
        return false;
    }

    uint8_t *buf = malloc(OTA_SHA_READ_CHUNK); // ★ 分块读，绝不整包读进内部 SRAM
    if (!buf)
    {
        ESP_LOGE(TAG, "SHA256 缓冲区 malloc 失败");
        return false;
    }

    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0); // 0 = SHA-256（非 SHA-224）

    bool ok = true;
    for (size_t off = 0; off < fw_size;)
    {
        size_t n = (fw_size - off) < OTA_SHA_READ_CHUNK ? (fw_size - off) : OTA_SHA_READ_CHUNK;
        if (esp_partition_read(part, off, buf, n) != ESP_OK)
        {
            ESP_LOGE(TAG, "分区回读失败 @ 0x%x", (unsigned)off);
            ok = false;
            break;
        }
        mbedtls_sha256_update(&ctx, buf, n);
        off += n;
    }

    uint8_t digest[32];
    if (ok)
        mbedtls_sha256_finish(&ctx, digest);
    mbedtls_sha256_free(&ctx);
    free(buf);
    if (!ok)
        return false;

    // 把 32 字节摘要转成 64 位小写十六进制
    char calc_hex[65];
    for (int i = 0; i < 32; i++)
        sprintf(calc_hex + i * 2, "%02x", digest[i]);
    calc_hex[64] = '\0';

    if (strcasecmp(calc_hex, expect_hex) != 0)
    {
        ESP_LOGE(TAG, "SHA256 不匹配！期望=%s 实际=%s", expect_hex, calc_hex);
        return false;
    }
    ESP_LOGI(TAG, "SHA256 校验通过: %s", calc_hex);
    return true;
}

// ─────────────────────────────────────────────────────────────────────────
// 异步 OTA 下载任务
// ─────────────────────────────────────────────────────────────────────────

/**
 * @brief OTA 失败统一收尾：延时 → 重启（本函数不返回）
 *
 * 由于升级前已调 session_stop_for_ota() 彻底停掉语音链路（不可简单软恢复），
 * 按产品约定：无论成功或失败都重启——失败重启后 bootloader 因备用分区未提交
 * (PENDING_VERIFY 未 mark_valid) 自动回滚到旧固件，旧固件照常运行。
 * LCD 不显示失败结果（只在下载过程显示进度条），失败静默重启。
 *
 * @param reason 失败原因短语，仅用于串口日志
 */
static void ota_fail_restart(const char *reason)
{
    ESP_LOGE(TAG, "OTA 失败(%s)，3 秒后重启回退旧固件", reason ? reason : "");
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart(); // 备用分区未提交，重启后 bootloader 自动回滚到旧固件
}

/**
 * @brief OTA 下载与重启任务（独立任务执行，避免阻塞 MQTT 回调）
 *
 * 失败处理：升级前已停语音链路，失败后统一走 ota_fail_restart()（显示失败页→重启回退）；
 * 成功处理：显示成功页，延迟 3 秒后 esp_restart()，让日志有机会刷新到串口。
 */
static void ota_task(void *pvParameters)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    ota_params_t *p = (ota_params_t *)pvParameters;
    ESP_LOGI(TAG, "开始 OTA：url=%s version=%s size=%u sha256=%s",
             p->url, p->version, (unsigned)p->size,
             p->sha256[0] ? p->sha256 : "(无,跳过校验)");

    // 写入 pending 版本（断电也能恢复）
    stage_pending(p->version, p->url);

    // ── 升级前：关闭所有与下载无关的任务，把 CPU/内存/带宽全让给固件下载 ──────────
    // 原则：OTA 无论成功失败都会 esp_restart()，无需恢复，故可放心彻底关停——
    //       关坏了也无所谓，3 秒后重启一切重建。
    // 保留（下载必须）：WiFi、MQTT/心跳（上报进度+收指令）、LVGL/屏幕（显示进度条）、ota_task 本身。
    // 关闭（全部无关任务）：
    //   ① 唤醒词引擎（afe_fetch/mn_detect）——占满 CPU1 做神经网络推理，最大的资源占用者；
    //   ② 语音会话 + audio_processor（采集/编解码）——最吃内部 SRAM；
    //   ③ GIF 动画——逐帧读 flash、抢 CPU；
    //   ④ 电池监控 + 电池日志——多余的 ADC 采样与串口打印。
    ESP_LOGW(TAG, "OTA：关闭无关任务（唤醒词/麦克风/语音/触摸/动画/电池），全力下载固件");

    // ① 唤醒词引擎 + 麦克风采集（最大的 CPU/资源占用者，独立于 session 常驻运行）。
    //    停 detect + 挂起 feed：不再识别唤醒词、不再 MultiNet overflow、不再采麦克风。
    //    故意不杀 afe_fetch_task 死循环、不置 s_afe_data=NULL——feed 停后 fetch 取不到
    //    帧走 vTaskDelay 安全空转，既不崩也不刷屏。详见 bsp_wake_word_stop_for_ota 注释。
    bsp_wake_word_stop_for_ota();

    // ② 语音会话（异步关闭：停并销毁 audio_processor，回 IDLE，释放内部 SRAM）
    session_stop_for_ota();
    vTaskDelay(pdMS_TO_TICKS(500)); // 等 session_event_task 完成关闭并释放内存

    // ③ 触摸扫描任务（自删退出，让出 CPU）。虽然 interaction 已停使触摸按下无动作，
    //    但扫描任务本身仍在耗 CPU，升级期间彻底停掉更干净。
    bsp_touch_stop_for_ota();

    // ④ interaction：停 GIF 切图 + 舵机 + 震动（丢弃一切情绪/动作请求）。
    interaction_stop_for_ota();
    ui_pause_main_gif(); // 再定格主界面 GIF 当前帧，停止其自动轮播解码

    // ⑤ 电池监控与日志任务（纯调试/后台采样，OTA 期间无用）
    bsp_battery_stop_log_task(); // 停「每 5s 打印电压」日志任务
    bsp_battery_stop_task();     // 停后台电池采样任务

    ui_show_ota_progress(0); // 显示「固件升级中 0%」，隐藏 GIF 独占屏幕

    esp_http_client_config_t http_cfg = {
        .url = p->url,
        .timeout_ms = 30000, // 30 秒超时
        .keep_alive_enable = true,
        // ★ 启用 mbedTLS 证书包校验，支持 https:// 下载（cert bundle 已在 sdkconfig 开启）
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
        // ★ 分段下载（双保险）：用 HTTP Range 把固件切成小段请求，服务器每段只回
        //   max_http_request_size 字节，TLS record 随之变小。根本修复靠 sdkconfig 的
        //   MBEDTLS_SSL_IN_CONTENT_LEN=16384（已能一次收下 16KB 大 record），此处作为
        //   冗余防护：万一将来换服务器发出更大 record、或证书链变大，分段可继续兜底。
        //   前提：服务器支持 Range 请求（当前服务器已实测返回 206 Partial Content）。
        .partial_http_download = true,
        .max_http_request_size = 8192, // 每段字节数，≤ IN_CONTENT_LEN(16384) 即可
    };

    // ── 1. 开始 OTA 会话 ──────────────────────────────────────────────
    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &ota_handle);
    if (err != ESP_OK || ota_handle == NULL)
    {
        ESP_LOGE(TAG, "esp_https_ota_begin 失败: %s", esp_err_to_name(err));
        ota_report(BSP_OTA_FAILED, 0, p->version, "下载连接失败，无法访问固件地址");
        free(p);
        ota_fail_restart("下载连接失败"); // 显示失败页 → 延时 → 重启（不返回）
    }

    // ── 2. 分步下载循环，过程中上报进度 ───────────────────────────────
    // 进度分母优先用后端下发的 size；缺省则退化为镜像头声明的总长度（可能为 0）
    int total = (p->size > 0) ? (int)p->size : (int)esp_https_ota_get_image_size(ota_handle);
    int last_progress = -1;
    int64_t last_report_ms = 0;

    while ((err = esp_https_ota_perform(ota_handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS)
    {
        standby_notify_activity(); // 刷新低功耗计时器
        int read = esp_https_ota_get_image_len_read(ota_handle);
        int progress = (total > 0) ? (int)((int64_t)read * 100 / total) : 0;
        if (progress > 100)
            progress = 100;

        int64_t now_ms = esp_timer_get_time() / 1000;
        // 节流：进度每变化 ≥5% 或距上次 ≥1 秒，才上报一次（避免刷爆 MQTT）
        if (progress - last_progress >= OTA_REPORT_MIN_STEP ||
            now_ms - last_report_ms >= OTA_REPORT_MIN_MS)
        {
            ESP_LOGI(TAG, "更新中 %d%% (%d/%d)", progress, read, total);
            // 按后端口径：下载过程也统一上报 upgrading（带进度），用户全程只看到"更新中"
            ota_report(BSP_OTA_UPGRADING, progress, p->version, NULL);
            ui_show_ota_progress(progress); // 屏幕同步显示「固件升级中 XX%」
            last_progress = progress;
            last_report_ms = now_ms;
        }
    }

    if (err != ESP_OK || !esp_https_ota_is_complete_data_received(ota_handle))
    {
        ESP_LOGE(TAG, "更新未完成: %s", esp_err_to_name(err));
        ota_report(BSP_OTA_FAILED, 0, p->version, "固件更新中断，数据不完整");
        esp_https_ota_abort(ota_handle);
        free(p);
        ota_fail_restart("下载中断/数据不完整"); // 显示失败页 → 延时 → 重启回退（不返回）
    }
    ota_report(BSP_OTA_UPGRADING, 100, p->version, NULL);
    ui_show_ota_progress(100); // 屏幕补到 100%（下载循环已退出，此处不再进循环体刷新）
    ESP_LOGI(TAG, "固件下载完成");

    standby_notify_activity(); // 校验读整个分区耗时数秒，先刷新计时防此间进待机

    // ── 3. SHA256 完整性校验（防止坏包烧进设备变砖）─────────────────────
    if (p->sha256[0] != '\0')
    {
        ESP_LOGI(TAG, "开始 SHA256 校验（读整个分区，约数秒）...");
        const esp_partition_t *update_part = esp_ota_get_next_update_partition(NULL);
        size_t fw_size = (p->size > 0)
                             ? (size_t)p->size
                             : (size_t)esp_https_ota_get_image_len_read(ota_handle);
        if (!verify_partition_sha256(update_part, fw_size, p->sha256))
        {
            ESP_LOGE(TAG, "SHA256 校验失败，丢弃固件，不切换分区");
            ota_report(BSP_OTA_FAILED, 0, p->version, "SHA256 校验失败，固件已损坏");
            esp_https_ota_abort(ota_handle); // 不调 finish，备用分区不会被启用
            free(p);
            ota_fail_restart("SHA256 校验失败"); // 显示失败页 → 延时 → 重启回退（不返回）
        }
    }
    else
    {
        ESP_LOGW(TAG, "后端未下发 sha256，跳过完整性校验");
    }

    ESP_LOGI(TAG, "SHA256 校验通过，开始写 otadata 收尾...");

    // ── 4. 收尾：写 otadata，把备用分区置为 PENDING_VERIFY ──────────────
    err = esp_https_ota_finish(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_https_ota_finish 失败: %s", esp_err_to_name(err));
        ota_report(BSP_OTA_FAILED, 0, p->version, "固件写入收尾失败");
        free(p);
        ota_fail_restart("固件写入收尾失败"); // 显示失败页 → 延时 → 重启回退（不返回）
    }

    // ── 5. 上报 upgrading + 记录待上报 success，延时后重启 ──────────────
    ota_report(BSP_OTA_UPGRADING, 100, p->version, NULL);
    write_pending_success(p->version); // 重启后由 MQTT 重连成功时补发 success
    ESP_LOGI(TAG, "OTA 校验通过，3 秒后重启切换到新固件...");
    free(p);
    vTaskDelay(pdMS_TO_TICKS(3000));
    esp_restart(); // 重启后 bootloader 切换到新固件
}

// ─────────────────────────────────────────────────────────────────────────
// 对外接口
// ─────────────────────────────────────────────────────────────────────────

/**
 * @brief 触发 OTA 下载任务
 *
 * @param url       OTA 服务器地址
 * @param version   OTA 服务器版本号
 * @return esp_err_t
 */
esp_err_t bsp_ota_trigger(const bsp_ota_req_t *req)
{
    if (!req || !req->url || strlen(req->url) == 0)
    {
        ESP_LOGE(TAG, "url 为空");
        return ESP_ERR_INVALID_ARG; // 参数错误
    }
    const char *version = req->version;

    // 版本号比较：拒绝同版本/降级
    const char *cur = bsp_ota_get_current_version();
    if (version && !is_newer_version(cur, version))
    {
        ESP_LOGW(TAG, "版本未升级（当前=%s 请求=%s），跳过", cur, version);
        return ESP_ERR_INVALID_VERSION; // 版本未升级
    }

    // 堆上分配任务参数（任务内部 free）
    ota_params_t *p = malloc(sizeof(ota_params_t));
    if (!p)
    {
        ESP_LOGE(TAG, "申请 ota 参数结构体 malloc 失败");
        return ESP_ERR_NO_MEM;
    }
    strncpy(p->url, req->url, sizeof(p->url) - 1); // 将 URL 复制到参数结构体中
    p->url[sizeof(p->url) - 1] = '\0';             // 最后增加字符串截断符
    // 含义：将版本号复制到参数结构体中，判断是否为未知版本，如未知版本则设置为 unknown
    strncpy(p->version, version ? version : "unknown", sizeof(p->version) - 1);
    p->version[sizeof(p->version) - 1] = '\0';
    // 拷贝 SHA256（可为空 → 不校验）和文件大小（progress 分母）
    if (req->sha256)
    {
        strncpy(p->sha256, req->sha256, sizeof(p->sha256) - 1);
        p->sha256[sizeof(p->sha256) - 1] = '\0';
    }
    else
    {
        p->sha256[0] = '\0';
    }
    p->size = req->size;

    // ★ 必须用 INTERNAL 栈：esp_https_ota 内部做 Flash 操作期间会禁用 Cache，
    //   SPIRAM 栈在此时不可访问会触发 panic。
    //   8KB 栈：HTTP client(~3KB) + TLS握手(~4KB) + 用户栈(~1KB)
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        ota_task, "ota_task",
        8192, p, 5, NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "ota_task 创建失败，内部 SRAM 不足");
        free(p);
        return ESP_FAIL;
    }
    PRINT_TASK_CREATED(TAG, "ota_task", 8192, 1); // 栈在内部SRAM
    return ESP_OK;
}
/**
 * @brief 标记当前 OTA 分区有效，取消回滚
 *验证新固件是否有效
 * 仅当刚 OTA 升级首次启动时调用。
 */
void bsp_ota_mark_valid(void)
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    if (!running)
    {
        ESP_LOGE(TAG, "获取运行分区失败");
        return;
    }

    // factory 分区无 OTA 状态机概念，无需 mark_valid
    if (strcmp(running->label, "factory") == 0)
    {
        ESP_LOGI(TAG, "当前从 factory 分区运行，跳过验证");
        return;
    }

    ESP_LOGI(TAG, "当前运行分区: %s @ 0x%lx",
             running->label, (unsigned long)running->address);

    esp_ota_img_states_t state;
    if (esp_ota_get_state_partition(running, &state) != ESP_OK)
    {
        ESP_LOGE(TAG, "获取分区状态失败");
        return;
    }

    // 只有 PENDING_VERIFY 状态需要处理（说明刚 OTA 升级首次启动）
    if (state == ESP_OTA_IMG_PENDING_VERIFY)
    {
        ESP_LOGI(TAG, "新固件首次启动验证 → 标记为有效，取消回滚");
        esp_ota_mark_app_valid_cancel_rollback();

        // 把 NVS pending 版本提升为 committed
        nvs_handle_t h;
        if (nvs_open(NVS_NS_OTA, NVS_READWRITE, &h) == ESP_OK)
        {
            char buf[64];
            size_t len = sizeof(buf);
            if (nvs_get_str(h, NVS_KEY_PENDING_VER, buf, &len) == ESP_OK)
            {
                nvs_set_str(h, NVS_KEY_COMMITTED, buf);
                nvs_erase_key(h, NVS_KEY_PENDING_VER);
                nvs_erase_key(h, NVS_KEY_PENDING_URL);
                nvs_commit(h);
                ESP_LOGI(TAG, "已提交版本号: %s", buf);
            }
            nvs_close(h);
        }
    }
    else
    {
        ESP_LOGI(TAG, "分区状态正常（state=%d），无需验证操作", (int)state);
    }
}

/** 获取当前运行版本号 */
const char *bsp_ota_get_current_version(void)
{
    const esp_app_desc_t *desc = esp_app_get_description(); // 获取当前运行版本号
    // 如果版本号中的第一个字符不为空，则返回版本号
    return (desc && desc->version[0]) ? desc->version : "unknown"; // 返回版本号
}
