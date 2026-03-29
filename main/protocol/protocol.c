#include "protocol.h"
#include "esp_wifi.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *MQTT_TAG = "MQTT_PROTOCOL";

// 全局静态 MQTT 客户端句柄和连接状态标志
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static volatile bool s_mqtt_connected = false;

// 独立处理唤醒词更新的后台任务
static void async_update_wakeword_task(void *pvParameters)
{
    char *new_word = (char *)pvParameters; // 从 MQTT 消息中获取唤醒词

    ESP_LOGI(MQTT_TAG, "异步任务开始重建唤醒词模型: %s", new_word);

    esp_err_t update_err = custom_wake_word_update(new_word);

    if (update_err == ESP_OK)
    {
        ESP_LOGI(MQTT_TAG, "唤醒词更新成功！现在可以离线唤醒了！");
    }
    else
    {
        ESP_LOGE(MQTT_TAG, " 唤醒词更新失败");
    }

    // 释放刚才申请的内存
    free(new_word);

    // 任务执行完毕后自杀销毁
    vTaskDelete(NULL);
}

// ❗ 重要 ❗: 这是一个【示例】函数，用于演示如何读取ADC值作为电量。
// 您必须根据您的硬件电路设计，修改此函数。
static int get_battery_level(void)
{
    // ========================== 硬件配置 START ==========================
    // TODO 1: 配置您的ADC通道。例如，如果您的电池电压检测引脚连接到 GPIO36，
    // 那么这里就是 ADC1_CHANNEL_0。
#define BATT_ADC_CHANNEL ADC1_CHANNEL_0

    // TODO 2: 配置ADC的衰减和位宽。这应该在初始化时完成一次即可。
    // 为确保函数可以独立演示，这里每次都调用，但最佳实践是将其移至初始化函数。
    adc1_config_width(ADC_WIDTH_BIT_12);                          // 12-bit, 0-4095
    adc1_config_channel_atten(BATT_ADC_CHANNEL, ADC_ATTEN_DB_11); // 11dB anttenu, for up to ~3.9V range
    // ========================== 硬件配置 END ==========================

    int adc_raw = adc1_get_raw(BATT_ADC_CHANNEL);

    // ========================== 逻辑转换 START ==========================
    // TODO 3: 将ADC原始值转换为百分比。
    // 这个转换逻辑【完全取决于】您的硬件：
    //  - 您的参考电压 (Vref) 是多少？
    //  - 您是否使用了分压电阻？分压比是多少？
    //  - 您的电池满电和没电时对应的电压分别是多少？(例如，锂电池 4.2V -> 100%, 3.2V -> 0%)

    // 下面是一个【极简的线性映射示例】，假设ADC读数4000为100%，2000为0%。
    // ！！！您必须用您自己的、准确的转换公式来替换它！！！
    int percent = (adc_raw - 2000) * 100 / (4000 - 2000);
    if (percent > 100)
    {
        percent = 100;
    }
    if (percent < 0)
    {
        percent = 0;
    }
    // ========================== 逻辑转换 END ==========================

    return percent;
}

// 后台心跳发送任务
static void heartbeat_task(void *arg)
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char device_id[16];
    snprintf(device_id, sizeof(device_id), "%02X%02X%02X", mac[3], mac[4], mac[5]);

    char topic[64];
    snprintf(topic, sizeof(topic), "echopal/device/%s/heartbeat", device_id);

    ESP_LOGI(MQTT_TAG, "心跳任务已启动 | DeviceID: %s | Topic: %s", device_id, topic);

    wifi_ap_record_t ap_info;

    while (1)
    {
        if (s_mqtt_connected && s_mqtt_client != NULL)
        {
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "deviceId", device_id);

            // 1. 补全电量显示
            cJSON_AddNumberToObject(root, "battery", get_battery_level());

            // 2. 增加 WiFi 信号强度显示
            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
            {
                // RSSI (Received Signal Strength Indicator) 是一个负数，绝对值越小信号越强
                cJSON_AddNumberToObject(root, "wifi_signal", ap_info.rssi);
            }
            else
            {
                cJSON_AddNumberToObject(root, "wifi_signal", -100); // 获取失败时给一个默认差值
            }

            char *json_str = cJSON_PrintUnformatted(root); // 格式化成 JSON 字符串

            if (json_str != NULL)
            {
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_str, 0, 1, 0);
                ESP_LOGI(MQTT_TAG, " 心跳已发送 (msg_id=%d): %s", msg_id, json_str);
                free(json_str);
            }
            cJSON_Delete(root);
        }
        vTaskDelay(pdMS_TO_TICKS(30000));
    }
    vTaskDelete(NULL);
}

// MQTT 事件回调函数
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client; //  获取 MQTT 客户端句柄

    switch ((esp_mqtt_event_id_t)event_id)
    {
    // MQTT 连接成功，就监听订阅的主题
    case MQTT_EVENT_CONNECTED:
    {
        ESP_LOGI(MQTT_TAG, " MQTT 服务器连接成功！");
        s_mqtt_connected = true;

        uint8_t mac[6];
        esp_wifi_get_mac(WIFI_IF_STA, mac);
        char topic[64];
        snprintf(topic, sizeof(topic), "echopal/%02X%02X%02X/wakeword/set", mac[3], mac[4], mac[5]);

        esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(MQTT_TAG, " 正在监听此主题: %s", topic);
        break;
    }
    // MQTT 收到数据
    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, " 收到云端数据！");

        char *json_data = calloc(1, event->data_len + 1);
        memcpy(json_data, event->data, event->data_len);

        cJSON *root = cJSON_Parse(json_data);
        if (root)
        {
            cJSON *word = cJSON_GetObjectItem(root, "wake_word");
            if (cJSON_IsString(word))
            {
                ESP_LOGW(MQTT_TAG, " 准备更新唤醒词为: %s", word->valuestring);
                char *word_copy = strdup(word->valuestring); // 复制字符串

                if (word_copy)
                {
                    // ✅ 修复隐患：检查 xTaskCreate 是否成功，防止 OOM 时发生内存泄漏
                    BaseType_t ret = xTaskCreate(async_update_wakeword_task, "async_ww_update", 6144, word_copy, 4, NULL);
                    if (ret != pdPASS)
                    {
                        ESP_LOGE(MQTT_TAG, "内存不足，无法创建唤醒词更新任务！");
                        free(word_copy); // 创建失败必须手动释放内存
                    }
                }
            }
            cJSON_Delete(root);
        }
        free(json_data);
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(MQTT_TAG, " MQTT 已断开，检查网络...");
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

    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(MQTT_TAG, " MQTT 客户端正在安全启动...");

    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 4, NULL);
}