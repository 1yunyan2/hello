#include "WIFI.h"

#define CLEAR_WIFI_BUTTON_PIN GPIO_NUM_0
#define MAX_RETRY_COUNT 5 // 最大重连次数

static const char *TAG = "EchoPals";
static EventGroupHandle_t wifi_event_group; // 事件组

const int CONNECTED_BIT = BIT0;
const int WIFI_FAIL_BIT = BIT1;
const int PROV_DONE_BIT = BIT2; // 配网完成标志

// 全局状态标志
static bool s_wifi_prov_initialized = false;
static bool s_is_provisioning = false; // 是否正在配网中
static int s_retry_num = 0;

// 安全重启函数
void clear_wifi_and_restart(void)
{
    ESP_LOGW(TAG, "🚨 正在清除已保存的 WiFi 账号密码...");
    esp_err_t err = wifi_prov_mgr_reset_provisioning();
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "清除成功，即将重启...");
    }
    else
    {
        ESP_LOGE(TAG, "清除失败: %s", esp_err_to_name(err));
    }

    // 给日志打印留出时间
    vTaskDelay(pdMS_TO_TICKS(1000));
    esp_restart();
}

// 优化的按键任务（减小栈内存，加快轮询，增加状态保护）
static void button_monitor_task(void *pvParameters)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CLEAR_WIFI_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE};
    gpio_config(&io_conf);

    int press_count = 0;
    while (1)
    {
        if (gpio_get_level(CLEAR_WIFI_BUTTON_PIN) == 0)
        {
            press_count++;
            if (press_count >= 30)
            { // 30 * 100ms = 3秒
                if (s_wifi_prov_initialized)
                { // 确保底层API已就绪
                    clear_wifi_and_restart();
                }
                press_count = 0;
            }
        }
        else
        {
            press_count = 0;
        }
        vTaskDelay(pdMS_TO_TICKS(100)); // 100ms精确延时
    }
}

// 自定义数据处理（防越界处理）
static esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf && inlen > 0 && inlen < 512)
    { // 增加长度限制防御恶意发包
        // 安全拷贝，防止没有结束符导致解析崩溃
        char *safe_str = calloc(1, inlen + 1);
        if (safe_str)
        {
            memcpy(safe_str, inbuf, inlen);
            ESP_LOGI(TAG, "📦 收到安全数据: %s", safe_str);
            // TODO: 在这里解析 safe_str (比如 JSON)
            free(safe_str);
        }
    }

    const char response[] = "{\"status\":\"OK\"}";
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL)
        return ESP_ERR_NO_MEM;

    *outlen = strlen(response);
    return ESP_OK;
}

// 配网事件处理
static void prov_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "🚀 配网启动，请打开APP");
            s_is_provisioning = true; // 进入配网模式，禁止自动重连
            break;
        case WIFI_PROV_CRED_RECV:
            ESP_LOGI(TAG, "📶 收到账号密码，验证中...");
            break;
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "🎉 密码正确！");
            s_is_provisioning = false; // 验证成功，退出配网模式
            break;
        case WIFI_PROV_CRED_FAIL:
            ESP_LOGE(TAG, "❌ 密码错误！");
            // 密码错误时，千万不要调用 esp_wifi_connect
            break;
        case WIFI_PROV_END:
            ESP_LOGI(TAG, "🛑 配网流程彻底结束。");
            xEventGroupSetBits(wifi_event_group, PROV_DONE_BIT); // 通知主逻辑释放资源
            break;
        default:
            break;
        }
    }
}

// IP与WiFi状态事件处理
static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // 🌟 核心修复：只有不在配网期间，才允许重连
        if (!s_is_provisioning)
        {
            if (s_retry_num < MAX_RETRY_COUNT)
            {
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGW(TAG, "WiFi 掉线，正在重连... (%d/%d)", s_retry_num, MAX_RETRY_COUNT);
            }
            else
            {
                xEventGroupSetBits(wifi_event_group, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "重连失败，放弃连接。");
            }
        }
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "✅ 成功获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0; // 重置重连计数器
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

void wifi_main()
{
    // 1. 初始化事件组 (带空指针保护)
    wifi_event_group = xEventGroupCreate();
    if (wifi_event_group == NULL)
    {
        ESP_LOGE(TAG, "事件组创建失败，内存耗尽！");
        return;
    }

    // 2. 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    // 3. 网络与事件循环初始化 (防止重复创建 panic)
    ESP_ERROR_CHECK(esp_netif_init());
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
    {
        ESP_ERROR_CHECK(err);
    }
    esp_netif_create_default_wifi_sta();

    // 4. 注册事件监听
    esp_event_handler_instance_t instance_any_id, instance_got_ip, prov_end_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL, &prov_end_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ip_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_ip_event_handler, NULL, &instance_got_ip));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 5. 初始化配网管理器
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM};
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    // 标志已初始化，此时按键任务若检测到长按才允许重置
    s_wifi_prov_initialized = true;
    // 启动按键监控任务 (内存缩减到 2048)
    xTaskCreate(button_monitor_task, "btn_task", 4096, NULL, 5, NULL);

    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned)
    {
        ESP_LOGI(TAG, "设备未配网，启动蓝牙广播...");
        uint8_t mac[6];
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
        // char service_name[16];
        // snprintf(service_name, sizeof(service_name), "EchoPals-%02X%02X%02X", mac[3], mac[4], mac[5]);
        // 使用 MAC 后缀作为动态 PoP 密码，比写死 "abcd1234" 更安全
        // char pop_key[16];
        // snprintf(pop_key, sizeof(pop_key), "Pals%02X%02X", mac[4], mac[5]);
        // ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create("custom-data"));
        // ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
        //     WIFI_PROV_SECURITY_1, (const void *)pop_key, service_name, NULL));
        // ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL));
        // ESP_LOGI(TAG, "==== 蓝牙: %s, 密码: %s ====", service_name, pop_key);

        // 使用固定密码
        const char *security_key = "abcd1234";
        char service_name[16];
        snprintf(service_name, sizeof(service_name), "EchoPals-%02X%02X%02X", mac[3], mac[4], mac[5]);
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create("custom-data"));
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_1,
            security_key,
            service_name,
            NULL));
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL));

        ESP_LOGI(TAG, "==== 蓝牙: %s, 密码: %s ====", service_name, security_key);

        // 🌟 阻塞等待用户配网操作完成
        xEventGroupWaitBits(wifi_event_group, PROV_DONE_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

        // 配网完毕，彻底释放管理器和几十KB的蓝牙基带内存
        wifi_prov_mgr_deinit(); // 释放管理器内存
        // esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);//释放蓝牙内存
    }
    else
    {
        ESP_LOGI(TAG, "设备已配网，直接连接...");
        // 已配网状态下，管理器没用了，直接释放，省下蓝牙内存
        wifi_prov_mgr_deinit();
        // esp_bt_controller_mem_release(ESP_BT_MODE_BTDM);

        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    // 6. 最终的阻塞等待：保证跳出这个函数时，网络100%是可用的
    ESP_LOGI(TAG, "等待网络彻底就绪...");
    EventBits_t bits = xEventGroupWaitBits(wifi_event_group, CONNECTED_BIT | WIFI_FAIL_BIT, pdFALSE, pdFALSE, portMAX_DELAY);

    if (bits & CONNECTED_BIT)
    {
        ESP_LOGI(TAG, "🚀 网络初始化完美收官，主程序可以开始干活了！");
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        ESP_LOGE(TAG, "💀 网络连接最终失败。");
        // 这里可以做异常降级，比如点亮红灯等
    }
}