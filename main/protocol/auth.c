/**
 * @file auth.c
 * @brief 设备认证模块 — 用 deviceToken 换取 accessToken
 *
 * 整体流程：
 *   App 绑定玩具时下发 deviceToken（长期凭证，存在 NVS）
 *   设备每次联网后，拿 deviceToken 调用后端 HTTP 接口换取 accessToken（短效凭证）
 *   accessToken 用于 WebSocket 连接的 Bearer 认证
 *
 * 接口信息：
 *   POST http://122.224.191.2:4888/api/auth/device-login
 *   请求体: {"deviceToken": "xxx"}
 *   响应体: {"accessToken": "yyy"} 或 {"data": {"accessToken": "yyy"}}
 */

#include "auth.h"
#include "object.h"
#include "esp_http_client.h"
#include "esp_log.h"
#include "cJSON.h"
#include "nvs.h"

#define TAG "Auth"

/**
 * @brief 内部扩展结构体（对外隐藏实现细节）
 *
 * auth_t 是公开的，只暴露 access_token 字段；
 * auth_wrapper_t 在此基础上追加了 HTTP 响应缓冲区，
 * 通过强制类型转换 (auth_t*) ↔ (auth_wrapper_t*) 实现"继承"。
 *
 * 内存布局：
 *   ┌─────────────────────┐
 *   │ auth_t              │  ← 对外可见部分
 *   │   └─ access_token   │
 *   ├─────────────────────┤
 *   │ response            │  ← 内部私有：HTTP 响应体缓冲区指针
 *   │ response_len        │  ← 内部私有：已接收的响应体长度
 *   └─────────────────────┘
 */
typedef struct
{
    auth_t auth;          // 基类，包含 access_token
    char *response;       // HTTP 响应体动态缓冲区（realloc 拼接）
    size_t response_len;  // 当前已接收的响应体字节数
} auth_wrapper_t;

/**
 * @brief HTTP 事件回调 — 负责将分片到达的响应数据拼接成完整 JSON
 *
 * esp_http_client 的数据是分块到达的（chunked transfer），
 * 每收到一块数据触发 HTTP_EVENT_ON_DATA，我们用 realloc 动态扩展缓冲区，
 * 把碎片拼接在一起，最终在 auth_perform() 中统一解析。
 *
 * 回调时序：
 *   HTTP_EVENT_ON_DATA  → 可能触发多次，每次追加数据到 response
 *   HTTP_EVENT_ON_FINISH → 请求完成，打印完整响应（调试用）
 *   HTTP_EVENT_ERROR     → 网络/协议错误
 */
static esp_err_t auth_http_event_handler(esp_http_client_event_t *evt)
{
    /* user_data 在 esp_http_client_config_t 中设置，指向我们的 wrapper */
    auth_wrapper_t *wrapper = (auth_wrapper_t *)evt->user_data;
    switch (evt->event_id)
    {
    case HTTP_EVENT_ERROR:
        ESP_LOGE(TAG, "HTTP Error");
        break;

    case HTTP_EVENT_ON_DATA:
    {
        /* 非 200 状态码的响应体不需要收集（可能是错误页面 HTML） */
        int status_code = esp_http_client_get_status_code(evt->client);
        if (status_code != 200)
            return ESP_OK;

        /* 动态扩展缓冲区：原有长度 + 本次收到的长度 */
        size_t new_len = wrapper->response_len + evt->data_len;
        char *new_buffer = realloc(wrapper->response, new_len);
        if (!new_buffer)
            return ESP_FAIL; // 内存不足，通知上层失败

        /* 把本次数据追加到缓冲区尾部 */
        memcpy(new_buffer + wrapper->response_len, evt->data, evt->data_len);
        wrapper->response = new_buffer;
        wrapper->response_len = new_len;
        break;
    }

    case HTTP_EVENT_ON_FINISH:
        /* 请求完成，打印完整响应便于调试 */
        ESP_LOGI(TAG, "Auth API Response: %.*s", wrapper->response_len, wrapper->response);
        break;

    default:
        break;
    }
    return ESP_OK;
}

// ─── 公开 API ──────────────────────────────────────────────────────────────

/**
 * @brief 创建认证实例
 * @return auth_t* 认证实例指针（实际分配的是 auth_wrapper_t，内部扩展）
 */
auth_t *auth_create(void)
{
    auth_wrapper_t *wrapper = malloc_zeroed(sizeof(auth_wrapper_t));
    return (auth_t *)wrapper;
}

/**
 * @brief 销毁认证实例，释放所有动态内存
 * @param auth 认证实例指针
 */
void auth_destroy(auth_t *auth)
{
    auth_wrapper_t *wrapper = (auth_wrapper_t *)auth;
    free(wrapper->response);          // 释放 HTTP 响应缓冲区
    free(wrapper->auth.access_token); // 释放 accessToken 字符串
    free(wrapper);                    // 释放结构体本身
}

/**
 * @brief 执行设备登录 — 用 deviceToken 换取 accessToken
 *
 * 完整流程：
 *   1. 构建 JSON 请求体 {"deviceToken": "xxx"}
 *   2. POST 到后端 /api/auth/device-login
 *   3. 解析响应 JSON，提取 accessToken
 *   4. 成功则存入 NVS 缓存（下次启动可作为备用）
 *
 * 调用后通过 auth->access_token 获取结果：
 *   - 非 NULL → 登录成功
 *   - NULL    → 登录失败（网络错误/服务端拒绝/JSON解析失败）
 *
 * @param auth         认证实例指针
 * @param device_token App 绑定时下发的长期凭证（从 NVS 读取）
 */
void auth_perform(auth_t *auth, const char *device_token)
{
    auth_wrapper_t *wrapper = (auth_wrapper_t *)auth;

    /* 清空上一次的响应缓冲区（支持重复调用） */
    free(wrapper->response);
    wrapper->response = NULL;
    wrapper->response_len = 0;

    // ── 第一步：配置并发起 HTTP POST 请求 ──────────────────────────────────

    esp_http_client_config_t config = {
        .url = AUTH_LOGIN_URL,                    // "http://122.224.191.2:4888/api/auth/device-login"
        .method = HTTP_METHOD_POST,               // POST 方法
        .event_handler = auth_http_event_handler, // 数据到达时的回调
        .user_data = wrapper,                     // 回调中通过 evt->user_data 拿到 wrapper
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);
    esp_http_client_set_header(client, "Content-Type", "application/json");

    /* 构建请求体 JSON: {"deviceToken": "xxx"} */
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "deviceToken", device_token);
    char *post_body = cJSON_PrintUnformatted(root); // 序列化为字符串（无格式化）
    cJSON_Delete(root);                             // JSON 对象用完即释放

    /* 发送请求（阻塞式，返回时数据已全部收到） */
    esp_http_client_set_post_field(client, post_body, strlen(post_body));
    esp_err_t ret = esp_http_client_perform(client);
    free(post_body); // 请求发完，释放请求体

    int status_code = esp_http_client_get_status_code(client);
    esp_http_client_cleanup(client); // 释放 HTTP 客户端资源

    /* 检查请求结果 */
    if (ret != ESP_OK || status_code != 200)
    {
        ESP_LOGE(TAG, "Login failed, status: %d", status_code);
        return; // auth->access_token 保持 NULL，调用方据此判断失败
    }

    // ── 第二步：解析响应 JSON，提取 accessToken ────────────────────────────

    cJSON *resp_json = cJSON_ParseWithLength(wrapper->response, wrapper->response_len);
    if (resp_json)
    {
        /*
         * 兼容后端两种可能的响应格式：
         *   格式 A: {"accessToken": "yyy"}           ← 直接在顶层
         *   格式 B: {"data": {"accessToken": "yyy"}} ← 嵌套在 data 里
         */
        cJSON *token_item = cJSON_GetObjectItem(resp_json, "accessToken");
        if (!token_item)
        {
            cJSON *data_item = cJSON_GetObjectItem(resp_json, "data");
            if (data_item)
                token_item = cJSON_GetObjectItem(data_item, "accessToken");
        }

        if (cJSON_IsString(token_item))
        {
            /* 拿到 accessToken，复制一份存到 auth 结构体 */
            free(wrapper->auth.access_token);
            wrapper->auth.access_token = strdup(token_item->valuestring);
            ESP_LOGI(TAG, "✅ 成功拿到 accessToken: %s", wrapper->auth.access_token);

            // ── 第三步：存入 NVS 缓存（断网时可用旧 token 兜底）────────────
            nvs_handle_t h;
            if (nvs_open("net_config", NVS_READWRITE, &h) == ESP_OK)
            {
                nvs_set_str(h, "access_token", wrapper->auth.access_token);
                nvs_commit(h); // 写入闪存（不 commit 只在 RAM 中）
                nvs_close(h);
            }
        }
        cJSON_Delete(resp_json); // 释放 JSON 解析树
    }
}
