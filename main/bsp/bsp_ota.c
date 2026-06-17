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
 * @brief OTA 下载与重启任务（独立任务执行，避免阻塞 MQTT 回调）
 *
 * 失败处理：若下载失败，不重启，旧固件 ota_0 继续运行；
 * 成功处理：延迟 3 秒后 esp_restart()，让日志有机会刷新到串口。
 */
static void ota_task(void *pvParameters)
{
    ota_params_t *p = (ota_params_t *)pvParameters;
    ESP_LOGI(TAG, "开始 OTA：url=%s version=%s size=%u sha256=%s",
             p->url, p->version, (unsigned)p->size,
             p->sha256[0] ? p->sha256 : "(无,跳过校验)");

    // 写入 pending 版本（断电也能恢复）
    stage_pending(p->version, p->url);

    esp_http_client_config_t http_cfg = {
        .url = p->url,
        .timeout_ms = 30000, // 30 秒超时
        .keep_alive_enable = true,
        // ★ 启用 mbedTLS 证书包校验，支持 https:// 下载（cert bundle 已在 sdkconfig 开启）
        .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };

    // ── 1. 开始 OTA 会话 ──────────────────────────────────────────────
    esp_https_ota_handle_t ota_handle = NULL;
    esp_err_t err = esp_https_ota_begin(&ota_cfg, &ota_handle);
    if (err != ESP_OK || ota_handle == NULL)
    {
        ESP_LOGE(TAG, "esp_https_ota_begin 失败: %s", esp_err_to_name(err));
        ota_report(BSP_OTA_FAILED, 0, p->version, "下载连接失败，无法访问固件地址");
        free(p);
        vTaskDelete(NULL); // 不重启，旧固件继续跑
        return;
    }

    // ── 2. 分步下载循环，过程中上报进度 ───────────────────────────────
    // 进度分母优先用后端下发的 size；缺省则退化为镜像头声明的总长度（可能为 0）
    int total = (p->size > 0) ? (int)p->size : (int)esp_https_ota_get_image_size(ota_handle);
    int last_progress = -1;
    int64_t last_report_ms = 0;

    while ((err = esp_https_ota_perform(ota_handle)) == ESP_ERR_HTTPS_OTA_IN_PROGRESS)
    {
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
        vTaskDelete(NULL); // 不重启
        return;
    }
    ota_report(BSP_OTA_UPGRADING, 100, p->version, NULL);
    ESP_LOGI(TAG, "固件下载完成");

    // ── 3. SHA256 完整性校验（防止坏包烧进设备变砖）─────────────────────
    if (p->sha256[0] != '\0')
    {
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
            vTaskDelete(NULL); // 不重启，旧固件安然继续跑
            return;
        }
    }
    else
    {
        ESP_LOGW(TAG, "后端未下发 sha256，跳过完整性校验");
    }

    // ── 4. 收尾：写 otadata，把备用分区置为 PENDING_VERIFY ──────────────
    err = esp_https_ota_finish(ota_handle);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "esp_https_ota_finish 失败: %s", esp_err_to_name(err));
        ota_report(BSP_OTA_FAILED, 0, p->version, "固件写入收尾失败");
        free(p);
        vTaskDelete(NULL); // 不重启
        return;
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
