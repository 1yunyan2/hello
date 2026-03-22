
#include "protocol.h"

static const char *MQTT_TAG = "MQTT_RECEIVER";

// MQTT 事件回调函数
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        ESP_LOGI(MQTT_TAG, "✅ MQTT 服务器连接成功！");

        // 🌟 自动获取你设备的实际 MAC 地址，确保订阅的主题是唯一的
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        char topic[64];
        snprintf(topic, sizeof(topic), "echopals/%02X%02X%02X/wakeword/set", mac[3], mac[4], mac[5]);

        esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(MQTT_TAG, "📡 正在监听此主题: %s", topic);
        break;
    }
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "💌 收到云端数据！");
        // printf("➤ 内容: %.*s\r\n", event->data_len, event->data);
        // // 这里后续将接入解析 JSON 并更新唤醒词的逻辑
        // // 1. 将原始数据转换为 C 字符串
        char *json_data = calloc(1, event->data_len + 1);
        memcpy(json_data, event->data, event->data_len);

        // 2. 解析 JSON
        cJSON *root = cJSON_Parse(json_data);
        if (root)
        {
            cJSON *word = cJSON_GetObjectItem(root, "wake_word");
            if (cJSON_IsString(word))
            {
                ESP_LOGW(MQTT_TAG, "🚀 准备更新唤醒词为: %s", word->valuestring);

                // 🌟 3. 调用业务层接口更新唤醒词
                if (custom_wake_word_update(word->valuestring) == ESP_OK)
                {
                    ESP_LOGI(MQTT_TAG, "✅ 唤醒词更新并存入 NVS 成功！");
                }
            }
            cJSON_Delete(root);
        }
        free(json_data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(MQTT_TAG, "❌ MQTT 已断开，检查网络...");
        break;

    default:
        break;
    }
}

// 启动 MQTT 客户端
void mqtt_app_start(void)
{
    // 🌟 核心修复：使用 static 将结构体存入静态区，防止栈溢出导致的崩溃
    static esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://broker.emqx.io:1883",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    ESP_LOGI(MQTT_TAG, "🚀 MQTT 客户端正在安全启动...");
}