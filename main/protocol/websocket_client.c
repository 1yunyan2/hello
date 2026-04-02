#include "websocket_client.h"
#include "esp_websocket_client.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define TAG "WS_Client"

// WebSocket 连接超时（毫秒）
#define WS_CONNECT_TIMEOUT_MS 5000

static esp_websocket_client_handle_t s_client = NULL;
static volatile bool s_connected = false;
static ws_receive_cb_t s_receive_cb = NULL;
static SemaphoreHandle_t s_mutex = NULL;

// ─── 内部事件处理 ────────────────────────────────────────────────────────────

static void ws_event_handler(void *handler_args,
                             esp_event_base_t base,
                             int32_t event_id,
                             void *event_data)
{
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id)
    {
    case WEBSOCKET_EVENT_CONNECTED:
        s_connected = true;
        ESP_LOGI(TAG, "WebSocket 已连接");
        break;

    case WEBSOCKET_EVENT_DISCONNECTED:
        s_connected = false;
        ESP_LOGW(TAG, "WebSocket 已断开");
        break;

    case WEBSOCKET_EVENT_DATA:
        // 只处理二进制帧（OPUS 数据）
        if (data->op_code == 0x02 && data->data_len > 0 && s_receive_cb)
            s_receive_cb(data->data_ptr, data->data_len);
        break;

    case WEBSOCKET_EVENT_ERROR:
        s_connected = false;
        ESP_LOGE(TAG, "WebSocket 错误");
        break;

    default:
        break;
    }
}

// ─── 公开 API ────────────────────────────────────────────────────────────────

esp_err_t ws_client_start(const char *uri, ws_receive_cb_t receive_cb)
{
    if (s_client != NULL)
    {
        ESP_LOGW(TAG, "WebSocket 已在运行，先停止");
        ws_client_stop();
    }

    if (s_mutex == NULL)
        s_mutex = xSemaphoreCreateMutex();

    s_receive_cb = receive_cb;
    s_connected = false;

    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .reconnect_timeout_ms = 0, // 不自动重连（由 session 管理）
        .network_timeout_ms = WS_CONNECT_TIMEOUT_MS,
    };

    s_client = esp_websocket_client_init(&ws_cfg);
    if (s_client == NULL)
    {
        ESP_LOGE(TAG, "WebSocket 客户端初始化失败");
        return ESP_FAIL;
    }

    esp_websocket_register_events(s_client, WEBSOCKET_EVENT_ANY,
                                  ws_event_handler, NULL);

    esp_err_t err = esp_websocket_client_start(s_client);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "WebSocket 启动失败: %s", esp_err_to_name(err));
        esp_websocket_client_destroy(s_client);
        s_client = NULL;
        return ESP_FAIL;
    }

    // 等待连接建立（最多 WS_CONNECT_TIMEOUT_MS）
    int waited = 0;
    while (!s_connected && waited < WS_CONNECT_TIMEOUT_MS)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        waited += 50;
    }

    if (!s_connected)
    {
        ESP_LOGE(TAG, "WebSocket 连接超时");
        esp_websocket_client_stop(s_client);
        esp_websocket_client_destroy(s_client);
        s_client = NULL;
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "WebSocket 连接成功: %s", uri);
    return ESP_OK;
}

void ws_client_stop(void)
{
    if (s_client == NULL)
        return;

    s_connected = false;
    esp_websocket_client_stop(s_client);
    esp_websocket_client_destroy(s_client);
    s_client = NULL;
    s_receive_cb = NULL;
    ESP_LOGI(TAG, "WebSocket 已关闭");
}

esp_err_t ws_client_send_binary(const void *data, size_t len)
{
    if (!s_connected || s_client == NULL)
        return ESP_FAIL;

    int sent = esp_websocket_client_send_bin(s_client, (const char *)data, len,
                                             pdMS_TO_TICKS(100));
    if (sent < 0)
    {
        ESP_LOGW(TAG, "发送失败（缓冲区满或已断开）");
        return ESP_FAIL;
    }
    return ESP_OK;
}

bool ws_client_is_connected(void)
{
    return s_connected && s_client != NULL;
}
