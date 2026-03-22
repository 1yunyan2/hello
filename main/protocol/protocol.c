
#include "protocol.h"

// ==================== MQTT 接收专用代码 ====================
static const char *MQTT_TAG = "MQTT_RECEIVER";

// MQTT 事件回调函数
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(MQTT_TAG, "✅ MQTT 服务器连接成功！");
        // 连接成功后，立刻订阅你的专属主题（这里假设 MAC 后缀是 F29318，你可以根据实际修改）
        esp_mqtt_client_subscribe(client, "echopals/F29318/wakeword/set", 0);
        ESP_LOGI(MQTT_TAG, "📡 已向服务器订阅主题: echopals/F29318/wakeword/set");
        break;

    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "💌 收到云端(MQTTX)发来的数据！");
        // 注意：MQTT 的字符串没有 \0 结尾，必须用 %.*s 配合长度来打印
        printf("➤ 主题(Topic): %.*s\r\n", event->topic_len, event->topic);
        printf("➤ 内容(Data): %.*s\r\n", event->data_len, event->data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(MQTT_TAG, "❌ MQTT 服务器断开连接，会自动重连...");
        break;

    case MQTT_EVENT_ERROR:
        ESP_LOGE(MQTT_TAG, "🚨 MQTT 发生错误");
        break;

    default:
        break;
    }
}

// 启动 MQTT 客户端
void mqtt_app_start(void)
{

    // 使用 {0} 强制将所有成员初始化为 NULL/0，这是最安全的做法
    esp_mqtt_client_config_t mqtt_cfg = {0};
    esp_mqtt_client_config_t mqtt_cfg = {
        // 使用 EMQX 提供的免费公共测试服务器，免密登录，最适合新手测试
        .broker.address.uri = "mqtt://broker.emqx.io:1883",
    };

    esp_mqtt_client_handle_t client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(client);
    ESP_LOGI(MQTT_TAG, "🚀 MQTT 客户端已启动，正在连接 broker.emqx.io ...");
}
// =========================================================