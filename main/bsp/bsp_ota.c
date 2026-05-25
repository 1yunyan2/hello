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
#include "esp_heap_caps.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

#define TAG "OTA"

// NVS 命名空间与键名
#define NVS_NS_OTA "ota"                  ///< OTA 命名空间
#define NVS_KEY_PENDING_VER "pending_ver" ///< 待验证的新版本号（升级中）
#define NVS_KEY_PENDING_URL "pending_url" ///< 待验证的固件 URL
#define NVS_KEY_COMMITTED "ota_version"   ///< 已提交的版本号（正常运行中）

/**
 * @brief OTA 任务参数结构体（在堆上传递，任务内部释放）
 */
typedef struct
{
    char url[512];    ///< 固件下载地址
    char version[32]; ///< 新版本号
} ota_params_t;

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
    ESP_LOGI(TAG, "开始 OTA：url=%s version=%s", p->url, p->version);

    // 写入 pending 版本（断电也能恢复）
    stage_pending(p->version, p->url);

    esp_http_client_config_t http_cfg = {
        .url = p->url,
        .timeout_ms = 30000, // 30 秒超时
        .keep_alive_enable = true,
        // HTTPS 时取消下行注释（需要在 sdkconfig 启用 mbedTLS 证书包）:
        // .crt_bundle_attach = esp_crt_bundle_attach,
    };
    esp_https_ota_config_t ota_cfg = {
        .http_config = &http_cfg,
    };
    // 使用esp_https_ota()从服务器下载并写入备用OTA分区
    esp_err_t ret = esp_https_ota(&ota_cfg);
    free(p);

    if (ret == ESP_OK)
    {
        ESP_LOGI(TAG, "OTA 下载完成，3 秒后重启切换到新固件...");
        vTaskDelay(pdMS_TO_TICKS(3000));
        esp_restart(); // 重启后bootloader 切换到新固件
    }
    else
    {
        ESP_LOGE(TAG, "OTA 失败: %s", esp_err_to_name(ret));
        // 失败不重启，旧固件继续运行；pending 记录保留以便云端排查
    }
    vTaskDelete(NULL);
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
esp_err_t bsp_ota_trigger(const char *url, const char *version)
{
    if (!url || strlen(url) == 0)
    {
        ESP_LOGE(TAG, "url 为空");
        return ESP_ERR_INVALID_ARG; // 参数错误
    }

    // 版本号比较：拒绝同版本/降级
    const char *cur = bsp_ota_get_current_version();
    // 版本号比较：拒绝同版本/降级
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
    strncpy(p->url, url, sizeof(p->url) - 1); // 将 URL 复制到参数结构体中
    p->url[sizeof(p->url) - 1] = '\0';        // 最后增加字符串截断符
    // 含义：将版本号复制到参数结构体中，判断是否为未知版本，如未知版本则设置为 unknown
    strncpy(p->version, version ? version : "unknown", sizeof(p->version) - 1);
    p->version[sizeof(p->version) - 1] = '\0';

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
