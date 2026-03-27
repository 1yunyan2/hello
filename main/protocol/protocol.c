#include "protocol.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *MQTT_TAG = "MQTT_PROTOCOL";

// 🌟 新增：全局静态 MQTT 客户端句柄和连接状态标志
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static volatile bool s_mqtt_connected = false;

// 🌟 新增：获取电池电量的模拟函数（后续接了 ADC 后替换这里）
static int get_battery_level(void)
{
    return 85;
}

// 🌟 新增：后台心跳发送任务
static void heartbeat_task(void *arg)
{
    // 获取设备真实 MAC 地址，提取后 3 个字节作为 Device ID
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char device_id[16];
    snprintf(device_id, sizeof(device_id), "%02X%02X%02X", mac[3], mac[4], mac[5]);

    // 拼接心跳专属 Topic
    char topic[64];
    snprintf(topic, sizeof(topic), "echopal/device/%s/heartbeat", device_id);

    ESP_LOGI(MQTT_TAG, "💓 心跳任务已启动 | DeviceID: %s | Topic: %s", device_id, topic);

    while (1)
    {
        // 只有当 MQTT 处于连接状态时，才组装和发送心跳
        if (s_mqtt_connected && s_mqtt_client != NULL)
        {
            // 1. 构建 JSON 对象
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "deviceId", device_id);
            cJSON_AddNumberToObject(root, "battery", get_battery_level());

            // 2. 压缩成无多余空格的 JSON 字符串
            char *json_str = cJSON_PrintUnformatted(root);

            if (json_str != NULL)
            {
                // 3. 通过 MQTT 发送出去，QoS 设为 1
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_str, 0, 1, 0);
                ESP_LOGI(MQTT_TAG, "💓 心跳已发送 (msg_id=%d): %s", msg_id, json_str);

                // 🚨 极度致命警告：必须 free 释放 JSON 字符串内存！
                free(json_str);
            }

            // 🚨 极度致命警告：销毁 cJSON 根节点释放内存！
            cJSON_Delete(root);
        }

        // 每隔 30 秒发送一次 (30000 毫秒)
        vTaskDelay(pdMS_TO_TICKS(30000));
    }

    vTaskDelete(NULL);
}

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

        // 🌟 标记连接成功，心跳任务可以开始干活了
        s_mqtt_connected = true;

        // 订阅唤醒词更新主题
        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        char topic[64];
        snprintf(topic, sizeof(topic), "echopal/%02X%02X%02X/wakeword/set", mac[3], mac[4], mac[5]);

        esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(MQTT_TAG, "📡 正在监听此主题: %s", topic);
        break;
    }
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "💌 收到云端数据！");

        char *json_data = calloc(1, event->data_len + 1);
        memcpy(json_data, event->data, event->data_len);

        // 解析 JSON 更新唤醒词
        cJSON *root = cJSON_Parse(json_data);
        if (root)
        {
            cJSON *word = cJSON_GetObjectItem(root, "wake_word");
            if (cJSON_IsString(word))
            {
                ESP_LOGW(MQTT_TAG, "🚀 准备更新唤醒词为: %s", word->valuestring);

                esp_err_t update_err = custom_wake_word_update(word->valuestring);
                if (update_err == ESP_OK)
                {
                    ESP_LOGI(MQTT_TAG, "🎉 唤醒词更新成功！现在可以离线唤醒了！");
                }
                else
                {
                    ESP_LOGE(MQTT_TAG, "❌ 唤醒词更新失败");
                }
            }
            cJSON_Delete(root);
        }
        free(json_data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(MQTT_TAG, "❌ MQTT 已断开，检查网络...");
        // 🌟 标记断开，暂停心跳发送
        s_mqtt_connected = false;
        break;

    default:
        break;
    }
}

// 启动 MQTT 客户端
void mqtt_app_start(void)
{
    static esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://122.224.191.2:1883",
        .credentials.username = "xtc",
        .credentials.authentication.password = "Xtc@12345",
    };

    // 🌟 将实例赋值给全局变量
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(MQTT_TAG, "🚀 MQTT 客户端正在安全启动...");

    // 🌟 启动心跳专属后台任务 (分配 4096 字节栈空间)
    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 4, NULL);
}