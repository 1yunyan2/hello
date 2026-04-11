/**
 * @file mqtt_protocol.c
 * @brief MQTT 设备管理协议实现（心跳上报、唤醒词热更新、重置通知）
 *
 * 内部模块结构：
 *   protocol_mqtt_start()
 *     ├─ mqtt_credentials_load()     从 NVS 加载凭证
 *     ├─ esp_mqtt_client_init/start   创建并启动 MQTT 客户端
 *     └─ heartbeat_task               后台心跳任务（每 50s 上报）
 *
 *   mqtt_event_handler（MQTT 事件回调）
 *     ├─ CONNECTED  → 订阅 wake-word 主题
 *     └─ DATA       → 解析 JSON → async_update_wakeword_task（异步更新唤醒词）
 *
 *   async_update_wakeword_task → wake_word_update()（唤醒词引擎）
 */
#include "mqtt_protocol.h"

static const char *MQTT_TAG = "MQTT"; ///< 日志 TAG

// ─── MQTT 凭证（运行时从 NVS 加载，回退到编译期默认值）────────────────────
#define MQTT_DEFAULT_URI  "mqtt://122.224.191.2:1883" ///< 默认 Broker 地址（测试环境）
#define MQTT_DEFAULT_USER "xtc"                        ///< 默认 MQTT 用户名
#define MQTT_DEFAULT_PASS "Xtc@12345"                  ///< 默认 MQTT 密码

// 运行时凭证缓冲区（由 mqtt_credentials_load 从 NVS 填充，否则保持默认值）
static char s_mqtt_uri[128]  = MQTT_DEFAULT_URI;
static char s_mqtt_user[64]  = MQTT_DEFAULT_USER;
static char s_mqtt_pass[64]  = MQTT_DEFAULT_PASS;

/**
 * @brief 从 NVS "mqtt_creds" 命名空间加载 MQTT 凭证
 *
 * 依次尝试读取 broker_url、username、password 三个键。
 * 任意一项读取失败时，该项保持编译期默认值，其他项不受影响。
 * 命名空间不存在时（如首次上电），直接返回使用全部默认值。
 *
 * @return void
 *
 * @note 调用者：protocol_mqtt_start()（启动 MQTT 前的第一步）
 */
static void mqtt_credentials_load(void)
{
    nvs_handle_t nvs;
    esp_err_t err = nvs_open("mqtt_creds", NVS_READONLY, &nvs);
    if (err != ESP_OK)
    {
        ESP_LOGW(MQTT_TAG, "未找到 mqtt_creds NVS 命名空间，使用默认凭证: %s", esp_err_to_name(err));
        return;
    }

    size_t len;

    len = sizeof(s_mqtt_uri);
    err = nvs_get_str(nvs, "broker_url", s_mqtt_uri, &len);
    if (err != ESP_OK)
    {
        ESP_LOGW(MQTT_TAG, "无法读取broker_url: %s，使用默认值", esp_err_to_name(err));
        strncpy(s_mqtt_uri, MQTT_DEFAULT_URI, sizeof(s_mqtt_uri));
    }

    len = sizeof(s_mqtt_user);
    err = nvs_get_str(nvs, "username", s_mqtt_user, &len);
    if (err != ESP_OK)
    {
        ESP_LOGW(MQTT_TAG, "无法读取username: %s，使用默认值", esp_err_to_name(err));
        strncpy(s_mqtt_user, MQTT_DEFAULT_USER, sizeof(s_mqtt_user));
    }

    len = sizeof(s_mqtt_pass);
    err = nvs_get_str(nvs, "password", s_mqtt_pass, &len);
    if (err != ESP_OK)
    {
        ESP_LOGW(MQTT_TAG, "无法读取password: %s，使用默认值", esp_err_to_name(err));
        strncpy(s_mqtt_pass, MQTT_DEFAULT_PASS, sizeof(s_mqtt_pass));
    }

    nvs_close(nvs);
    ESP_LOGI(MQTT_TAG, "MQTT 凭证已从 NVS 加载，Broker: %s", s_mqtt_uri);
}

// 全局静态 MQTT 客户端句柄和连接状态标志
static esp_mqtt_client_handle_t s_mqtt_client = NULL;
static volatile bool s_mqtt_connected = false;

// ─── 工具函数：获取设备 MAC 后三字节作为短 ID ──────────────────────────────
// 多处需要 device_id（心跳、订阅、重置通知），统一提取避免重复代码
/**
 * @brief 从设备MAC地址生成短ID
 *
 * 提取WiFi STA模式下的MAC地址后三个字节，并格式化为6位十六进制字符串。
 * 该短ID用于MQTT主题命名（如 heartbeat、wake-word）以区分不同设备。
 *
 * @param[out] out 输出缓冲区，用于存放格式化的短ID字符串
 * @param out_size 输出缓冲区大小
 */
static void get_short_device_id(char *out, size_t out_size)
{
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    snprintf(out, out_size, "%02X%02X%02X", mac[3], mac[4], mac[5]);
}

/**
 * @brief 异步执行唤醒词更新的后台任务参数结构体
 *
 * 由 mqtt_event_handler 在堆上分配并传给 async_update_wakeword_task，
 * 任务执行完毕后由任务内部 free() 释放。
 */
typedef struct
{
    char display[64]; ///< 显示文字（如 "云炎"、"Hello Echo"），用于自动检测语言
    char pinyin[64];  ///< 命令词（中文拼音 "yun yan" 或英文单词 "hello echo"）
} ww_update_params_t;

// 独立处理唤醒词更新的后台任务
/**
 * @brief 异步执行唤醒词更新的任务
 *
 * 创建一个独立的FreeRTOS任务来调用wake_word_update接口，
 * 避免在MQTT事件回调中执行耗时操作而阻塞网络通信。
 * 任务完成后会释放传入的参数内存并自我删除。
 *
 * @param pvParameters 指向ww_update_params_t结构体的指针
 */
static void async_update_wakeword_task(void *pvParameters)
{
    ww_update_params_t *params = (ww_update_params_t *)pvParameters;

    ESP_LOGI(MQTT_TAG, "异步任务: display=%s pinyin=%s", params->display, params->pinyin);

    esp_err_t update_err = wake_word_update(params->display, params->pinyin);

    if (update_err == ESP_OK)
        ESP_LOGI(MQTT_TAG, "唤醒词更新成功！");
    else
        ESP_LOGE(MQTT_TAG, "唤醒词更新失败");

    UBaseType_t high_water_mark = uxTaskGetStackHighWaterMark(NULL);
    ESP_LOGW("WW_TASK", "栈剩余: %lu 字节", (uint32_t)high_water_mark);
    free(params);
    vTaskDelete(NULL);
}

//!  重要 ❗: 这是一个【示例】函数，用于演示如何读取ADC值作为电量。
// 您必须根据您的硬件电路设计，修改此函数。
/**
 * @brief 获取电池电量百分比
 *
 * 【示例代码】通过ADC1通道0读取电池电压，并将其线性映射为0-100%的电量值。
 * ⚠️ 此函数为示意，您必须根据实际的硬件分压电路参数修改计算公式。
 *
 * @return int 电池电量百分比 (0-100)
 */
static int get_battery_level(void)
{
#define BATT_ADC_CHANNEL ADC1_CHANNEL_0
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(BATT_ADC_CHANNEL, ADC_ATTEN_DB_11);

    int adc_raw = adc1_get_raw(BATT_ADC_CHANNEL);

    // TODO: 根据实际硬件（分压比、参考电压）替换下列线性映射公式
    int percent = (adc_raw - 2000) * 100 / (4000 - 2000);
    if (percent > 100)
        percent = 100;
    if (percent < 0)
        percent = 0;

    return percent;
}

// 后台心跳发送任务
/**
 * @brief 发送周期性心跳消息的后台任务
 *
 * 在独立任务中每5秒向MQTT服务器发布一次包含设备ID、电池电量和Wi-Fi信号强度的JSON消息。
 * 该任务会持续运行，直到被外部显式删除。
 *
 * @param arg 未使用
 */
static void heartbeat_task(void *arg)
{
    char device_id[16];
    get_short_device_id(device_id, sizeof(device_id));

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
            cJSON_AddNumberToObject(root, "battery", get_battery_level());

            if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
                cJSON_AddNumberToObject(root, "wifi_signal", ap_info.rssi);
            else
                cJSON_AddNumberToObject(root, "wifi_signal", -100);

            char *json_str = cJSON_PrintUnformatted(root);
            if (json_str != NULL)
            {
                int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_str, 0, 1, 0);
                ESP_LOGI(MQTT_TAG, "心跳已发送 (msg_id=%d): %s", msg_id, json_str);
                free(json_str);
            }
            cJSON_Delete(root);
        }
        vTaskDelay(pdMS_TO_TICKS(50000)); //! 更改心跳速度为5s，
    }
    vTaskDelete(NULL);
}

// MQTT 事件回调函数
/**
 * @brief MQTT客户端事件处理回调
 *
 * 处理MQTT连接、断开、数据接收等事件。在连接成功时订阅唤醒词更新主题；
 * 在收到特定主题消息时，解析JSON并启动异步任务来更新设备的唤醒词。
 *
 * @param handler_args 用户自定义参数（未使用）
 * @param base 事件基类（ESP_EVENT_ANY_ID）
 * @param event_id 具体的事件ID（如MQTT_EVENT_CONNECTED）
 * @param event_data 指向esp_mqtt_event_t结构体的指针，包含事件详情
 */
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t event = event_data;
    esp_mqtt_client_handle_t client = event->client;

    switch ((esp_mqtt_event_id_t)event_id)
    {
    case MQTT_EVENT_CONNECTED:
    {
        ESP_LOGI(MQTT_TAG, "MQTT 服务器连接成功！");
        s_mqtt_connected = true;

        char dev_id[16];
        get_short_device_id(dev_id, sizeof(dev_id));
        char topic[64];
        snprintf(topic, sizeof(topic), "echopal/device/%s/wake-word", dev_id);

        esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(MQTT_TAG, "正在监听此主题: %s", topic);
        break;
    }

    case MQTT_EVENT_DATA:
        ESP_LOGI(MQTT_TAG, "收到云端数据！");
        ESP_LOGI(MQTT_TAG, "收到主题: %.*s", event->topic_len, event->topic);
        ESP_LOGI(MQTT_TAG, "收到数据: %.*s", event->data_len, event->data);

        if (event->topic_len > 0 && strstr(event->topic, "wake-word") != NULL)
        {
            ESP_LOGI(MQTT_TAG, "拦截到唤醒词更新指令！");

            char *json_data = calloc(1, event->data_len + 1);
            if (!json_data)
            {
                ESP_LOGE(MQTT_TAG, "内存不足，无法分配 JSON 缓冲区");
                break;
            }
            memcpy(json_data, event->data, event->data_len);

            cJSON *root = cJSON_Parse(json_data);
            free(json_data);
            if (root)
            {
                cJSON *pinyin_item = cJSON_GetObjectItem(root, "wakeWordPinyin");
                cJSON *display_item = cJSON_GetObjectItem(root, "wakeWord");
                if (cJSON_IsString(pinyin_item) && pinyin_item->valuestring != NULL &&
                    cJSON_IsString(display_item) && display_item->valuestring != NULL)
                {
                    ESP_LOGW(MQTT_TAG, "准备更新唤醒词: wakeWord=%s pinyin=%s",
                             display_item->valuestring, pinyin_item->valuestring);

                    ww_update_params_t *params = malloc(sizeof(ww_update_params_t));
                    if (params)
                    {
                        strncpy(params->display, display_item->valuestring, sizeof(params->display) - 1);
                        params->display[sizeof(params->display) - 1] = '\0';
                        strncpy(params->pinyin, pinyin_item->valuestring, sizeof(params->pinyin) - 1);
                        params->pinyin[sizeof(params->pinyin) - 1] = '\0';

                        BaseType_t ret = xTaskCreate(async_update_wakeword_task, "async_ww_update",
                                                     4096, params, 4, NULL);
                        if (ret != pdPASS)
                        {
                            ESP_LOGE(MQTT_TAG, "内存不足，无法创建唤醒词更新任务！");
                            free(params);
                        }
                    }
                }
                cJSON_Delete(root);
            }
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(MQTT_TAG, "MQTT 已断开，检查网络...");
        s_mqtt_connected = false;
        break;

    default:
        break;
    }
}

// 启动 MQTT 客户端
/**
 * @brief 初始化并启动 MQTT 客户端（见 .h 文件 Doxygen 说明）
 *
 * @note 调用者：application.c → application_init()（Wi-Fi 就绪后）
 */
void protocol_mqtt_start(void)
{
    mqtt_credentials_load();

    esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = s_mqtt_uri,
        .credentials.username = s_mqtt_user,
        .credentials.authentication.password = s_mqtt_pass,
    };
    s_mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(s_mqtt_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGI(MQTT_TAG, "MQTT 客户端正在启动...");
    xTaskCreate(heartbeat_task, "heartbeat_task", 4096, NULL, 4, NULL);
};

/**
 * @brief 发送设备重置通知（见 .h 文件 Doxygen 说明）
 *
 * 消息格式：{"event":"factory_reset"}，QoS 1 发布到 echopal/device/{id}/reset。
 *
 * @note 调用者：出厂重置逻辑
 */
void send_reset_notification(void)
{
    // 检查 MQTT 客户端是否已初始化和连接
    if (s_mqtt_client == NULL || !s_mqtt_connected)
    {
        ESP_LOGW(MQTT_TAG, "MQTT客户端未就绪，无法发送重置通知");
        return;
    }

    // 获取设备 ID 并构建 MQTT Topic
    char device_id[16];
    get_short_device_id(device_id, sizeof(device_id));
    char topic[64];
    snprintf(topic, sizeof(topic), "echopal/device/%s/reset", device_id);
    ESP_LOGI(MQTT_TAG, "重置DeviceID: %s | Topic: %s", device_id, topic);

    // // 4. 获取时间戳（系统启动以来的微秒数，uint64_t）
    // uint64_t timestamp = esp_timer_get_time();

    // 5. 使用 cJSON 构建标准 JSON（和你 heartbeat_task 风格一致）
    cJSON *root = cJSON_CreateObject();

    // 添加字段：event（字符串）
    cJSON_AddStringToObject(root, "event", "factory_reset");
    // cJSON_AddNumberToObject(root, "timestamp", (double)timestamp);

    // 6. 转为紧凑 JSON 字符串（节省流量）
    char *json_str = cJSON_PrintUnformatted(root);
    if (json_str == NULL)
    {
        ESP_LOGE(MQTT_TAG, "生成 JSON 字符串失败");
        cJSON_Delete(root);
        return;
    }

    // 7. 发布消息（QoS 1 确保送达）
    int msg_id = esp_mqtt_client_publish(s_mqtt_client, topic, json_str, 0, 1, 0);
    if (msg_id > 0)
    {
        ESP_LOGI(MQTT_TAG, "重置通知已发送 (msg_id=%d): %s", msg_id, json_str);
    }
    else
    {
        ESP_LOGE(MQTT_TAG, "重置通知发送失败");
    }

    // 8. 【重要】释放内存，避免泄漏（和 heartbeat_task 一致）
    cJSON_Delete(root);
    free(json_str);
}