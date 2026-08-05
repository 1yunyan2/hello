/**
 * @file auth.c
 * @brief 设备认证模块实现 — 用 deviceToken 换取 accessToken
 *
 * 整体流程：
 * App 绑定玩具时下发 deviceToken（长期凭证，存在 NVS）
 * 设备每次联网后，拿 deviceToken 调用后端 HTTP 接口换取 accessToken（短效凭证）
 * accessToken 用于 WebSocket 连接的 Bearer 认证
 */

#include "auth.h"
#include "object.h"
#include "esp_http_client.h"
#include "esp_crt_bundle.h"
#include "esp_log.h"
#include "cJSON.h"
#include "nvs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/** @brief 模块日志标签 */
#define TAG "Auth"

/** @brief 服务器不可达标志：Auth HTTP 失败时置 false，成功时置 true */
static volatile bool s_server_reachable = true;

/**
 * @brief 内部扩展结构体（面向对象封装设计）
 *
 * auth_t 对外公开；auth_wrapper_t 在此基础上追加了 HTTP 响应相关的运行时缓冲区。
 */
typedef struct
{
    auth_t auth;         ///< 基类，包含对外的 access_token
    char *response;      ///< 内部私有：HTTP 响应体动态缓冲区（通过 realloc 拼接）
    size_t response_len; ///< 内部私有：当前已接收的响应体字节数
} auth_wrapper_t;

/**
 * @brief HTTP 客户端事件回调
 * * @param[in] evt HTTP 客户端事件指针，包含事件类型、数据等信息
 * @return esp_err_t 总是返回 ESP_OK（内存不足导致拼接失败会记录但不中断整个框架机制）
 * @note 调用者：ESP HTTP Client 底层回调触发
 */
static esp_err_t auth_http_event_handler(esp_http_client_event_t *evt)
{
    // ─── 逻辑块 1：获取绑定的扩展实例 ────────────────────────────
    // 说明：从 evt->user_data 强转取出我们的内部封装结构 auth_wrapper_t
    auth_wrapper_t *wrapper = (auth_wrapper_t *)evt->user_data;

    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP Error");
        break;

    case HTTP_EVENT_ON_DATA:
    {
        // ─── 逻辑块 2：处理分块传输到达的数据 ────────────────────────
        // 说明：校验状态码（仅收录 200/201 的正常 JSON 体）。扩展缓冲区内存大小并将新的 chunk 拷入末尾。
        // API：esp_http_client_get_status_code, realloc, memcpy
        // 数据：修改 wrapper->response 和 wrapper->response_len
        int status_code = esp_http_client_get_status_code(evt->client);
        if (status_code != 200 && status_code != 201)
            return ESP_OK;

        size_t new_len = wrapper->response_len + evt->data_len;
        char *new_buffer = realloc(wrapper->response, new_len);
        if (!new_buffer)
            return ESP_FAIL; // 内存不足，通知上层失败

        memcpy(new_buffer + wrapper->response_len, evt->data, evt->data_len);
        wrapper->response = new_buffer;
        wrapper->response_len = new_len;
        /* 【堆损坏排查·2026-07-29 已验证无问题，暂停用】本处 realloc 按精确长度分配、
         * 响应体不带 '\0'，曾怀疑下游按 C 字符串处理会越界。实测：下游用的是
         * cJSON_ParseWithLength（带长度，不依赖 '\0'），整条 Auth 链路全程堆干净。
         * 保留注释，需再查时取消注释即可。 */
        // HEAP_PROBE("HEAPCHK", "auth响应realloc后");
        break;
    }

    case HTTP_EVENT_ON_FINISH:
        // ─── 逻辑块 3：请求接收完毕 ──────────────────────────────────
        // 说明：日志打印完整的响应体，便于调试。
        ESP_LOGI(TAG, "Auth API Response: %.*s", wrapper->response_len, wrapper->response);
        break;

    default:
        break;
    }
    return ESP_OK;
}

/**
 * @brief 创建认证实例
 * * @param 无
 * @return auth_t* 认证实例指针
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
auth_t *auth_create(void)
{
    // 说明：通过 object.h 的 malloc_zeroed 在 SPIRAM 分配空间（强制转换为基类暴露给外部）
    auth_wrapper_t *wrapper = malloc_zeroed(sizeof(auth_wrapper_t));
    return (auth_t *)wrapper;
}

/**
 * @brief 销毁认证实例并释放内存
 * * @param[in] auth 认证实例指针
 * @return 无
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
void auth_destroy(auth_t *auth)
{
    // 说明：强转回 wrapper 类型，彻底释放结构体中管理的子内存后，再释放本体
    auth_wrapper_t *wrapper = (auth_wrapper_t *)auth;
    free(wrapper->response);
    free(wrapper->auth.access_token);
    free(wrapper);
}

/**
 * @brief 发起 HTTP 请求，执行登录获取 AccessToken
 * * @param[in,out] auth         认证实例指针
 * @param[in]     device_token NVS 中缓存的长期凭证
 * @return 无
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
void auth_perform(auth_t *auth, const char *device_token)
{
    auth_wrapper_t *wrapper = (auth_wrapper_t *)auth;

    // ─── 逻辑块 1：清空历史状态 ──────────────────────────────────
    // 说明：释放并重置 response 内部缓冲，确保对象能够重复用于下一次请求。
    // API：free
    free(wrapper->response);
    wrapper->response = NULL;
    wrapper->response_len = 0;

    // ─── 逻辑块 2：配置并发起 HTTP POST 请求（含 1 次重试）─────
    // 说明：组装 JSON 格式的 deviceToken 字段，调用 ESP-IDF HTTP 客户端发起 POST 阻塞请求。
    //       失败后等待 1 秒重试 1 次，两次均失败则标记服务器不可达。
    // API：esp_http_client_init, esp_http_client_set_header, cJSON_CreateObject, cJSON_AddStringToObject, cJSON_PrintUnformatted, esp_http_client_set_post_field, esp_http_client_perform, esp_http_client_cleanup
    // 数据：构建 post_body 字符串用于请求体发送。
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "deviceToken", device_token);
    char *post_body = cJSON_PrintUnformatted(root);
    cJSON_Delete(root);

    esp_err_t ret = ESP_FAIL;
    int status_code = 0;
    const int max_retries = 3;               // 同步最多尝试 3 次（约 9 秒），失败后由后台 session_reconnect_task 指数退避无限重试
    PRINT_MEM_INFO(TAG, "Auth HTTP 请求前"); // 拆分埋点：区分"请求本身开销"与"请求前已有的基线"
    for (int attempt = 0; attempt < max_retries; attempt++)
    {
        esp_http_client_config_t config = {
            .url = AUTH_LOGIN_URL, // 认证 API 地址
            .method = HTTP_METHOD_POST,
            .event_handler = auth_http_event_handler,
            .user_data = wrapper,
            .timeout_ms = 3000,                         // 3秒超时，与 WebSocket 网络超时一致，服务器不可达时快速失败
            .crt_bundle_attach = esp_crt_bundle_attach, // HTTPS 根证书校验
        };
        esp_http_client_handle_t client = esp_http_client_init(&config);
        esp_http_client_set_header(client, "Content-Type", "application/json");
        esp_http_client_set_post_field(client, post_body, strlen(post_body));

        ESP_LOGI(TAG, "Auth 第 %d/%d 次尝试...", attempt + 1, max_retries);
        /* 【堆损坏排查·2026-07-29 已验证无问题，暂停用】
         * 曾怀疑 TLS 握手打穿 ws_reconn 的 6144B 栈。实测水位：
         *   入口5628 → Auth前4140 → 握手前3884 → 握手后2828 → 建连后2828
         * 最低仍剩 2828B（用了一半多点），TLS 握手【没有】打穿栈；堆也全程干净。
         * 结论：ws_reconn 与 Auth/TLS 链路排除嫌疑。 */
        // HEAP_PROBE("HEAPCHK", "http_perform前");
        // PRINT_STACK_AT(TAG, "http_perform前");
        ret = esp_http_client_perform(client);
        // PRINT_STACK_AT(TAG, "http_perform后");
        // HEAP_PROBE("HEAPCHK", "http_perform后");
        status_code = esp_http_client_get_status_code(client);
        esp_http_client_cleanup(client);
        // HEAP_PROBE("HEAPCHK", "http_cleanup后");
        PRINT_MEM_INFO(TAG, "Auth HTTP 请求完成");

        if (ret == ESP_OK && (status_code == 200 || status_code == 201))
        {
            s_server_reachable = true; // 服务器可达
            break;                     // 成功，跳出重试循环
        }

        ESP_LOGW(TAG, "Auth 第 %d 次失败 (ret=%s, status=%d)", attempt + 1,
                 esp_err_to_name(ret), status_code);
        if (attempt < max_retries - 1)
            vTaskDelay(pdMS_TO_TICKS(2000)); // 每次重试间隔 2 秒
    }
    free(post_body);

    if (ret != ESP_OK)
    {
        ESP_LOGW(TAG, "Auth 发送失败: %s（%d 次均失败），将由后台重连任务继续尝试",
                 esp_err_to_name(ret), max_retries);
        s_server_reachable = false; // 服务器不可达，通知上层跳过后续连接
        return;
    }
    if (status_code != 200 && status_code != 201)
    {
        ESP_LOGW(TAG, "Auth 失败，HTTP 状态码: %d（%d 次均失败），将由后台重连任务继续尝试",
                 status_code, max_retries);
        s_server_reachable = false;
        return;
    }

    // ─── 逻辑块 3：解析响应 JSON，提取并持久化 accessToken ──────────
    // 说明：利用 cJSON 解析返回数据（兼容多层结构格式），提取到 token 后立即缓存至 NVS 中作为兜底机制。
    // API：cJSON_ParseWithLength, cJSON_GetObjectItem, strdup, nvs_open, nvs_set_str, nvs_commit, nvs_close, cJSON_Delete
    // 数据：修改 wrapper->auth.access_token，并在 NVS 写入。
    /* 【已验证无问题，暂停用】此处用 WithLength 是对的（响应体无 '\0'，不会越界读） */
    // HEAP_PROBE("HEAPCHK", "cJSON解析前");
    cJSON *resp_json = cJSON_ParseWithLength(wrapper->response, wrapper->response_len);
    // HEAP_PROBE("HEAPCHK", "cJSON解析后");
    if (resp_json)
    {
        cJSON *token_item = cJSON_GetObjectItem(resp_json, "accessToken");
        if (!token_item)
        {
            cJSON *data_item = cJSON_GetObjectItem(resp_json, "data");
            if (data_item)
                token_item = cJSON_GetObjectItem(data_item, "accessToken");
        }

        if (cJSON_IsString(token_item))
        {
            free(wrapper->auth.access_token);
            wrapper->auth.access_token = strdup(token_item->valuestring);
            ESP_LOGI(TAG, "[OK] 成功拿到 accessToken: %s", wrapper->auth.access_token);

            nvs_handle_t h;
            if (nvs_open("net_config", NVS_READWRITE, &h) == ESP_OK)
            {
                /* 【已验证无问题，暂停用】nvs 写 Flash 会关 Cache，期间 SPIRAM 不可访问，
                 * 曾怀疑关 Cache 窗口内的并发访问写坏堆。实测此前后堆均干净。 */
                // HEAP_PROBE("HEAPCHK", "nvs写token前");
                nvs_set_str(h, "access_token", wrapper->auth.access_token);
                nvs_commit(h);
                nvs_close(h);
                // HEAP_PROBE("HEAPCHK", "nvs写token后");
            }
        }
        cJSON_Delete(resp_json);
        // HEAP_PROBE("HEAPCHK", "cJSON_Delete后");  // 【已验证无问题，暂停用】
    }
}

/**
 * @brief 查询服务器是否可达
 * @return true 可达，false 不可达
 * @note 调用者：session.c, mqtt_protocol.c
 */
bool auth_is_server_reachable(void)
{
    return s_server_reachable;
}
