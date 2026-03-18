#include "WIFI.h"
#include <string.h> // 🌟 新增头文件，因为后面用到了 strdup 和 strlen
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// 自定义WiFi重置的引脚
#define CLEAR_WIFI_BUTTON_PIN GPIO_NUM_0 // boot按键引脚
static const char *TAG = "EchoPals";     // 定义一个全局的 TAG
// 定义一个全局事件组，用来同步网络状态
static EventGroupHandle_t wifi_event_group;
const int CONNECTED_BIT = BIT0;

void clear_wifi_and_restart(void)
{
    ESP_LOGW("WIFI_CLEAR", "🚨 警告：正在清除已保存的 WiFi 账号密码...");
    esp_err_t err = wifi_prov_mgr_reset_provisioning(); // 清除已保存的 WiFi 账号密码
    if (err == ESP_OK)
    {
        ESP_LOGI(TAG, "清除成功，设备将回到未配网状态。");
    }
    else
    {
        ESP_LOGE(TAG, "清除失败: %s", esp_err_to_name(err));
    }
    // 稍微等 1 秒钟，让上面的串口日志打印完
    vTaskDelay(1000);
    // 强行重启芯片，让它像刚出厂一样重新跑 wifi_main() 里的流程
    esp_restart();
}
static void button_monitor_task(void *pvParameters)
{
    // 1. 配置 GPIO 引脚为输入模式，并开启内部上拉电阻
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CLEAR_WIFI_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE, // 开启内部上拉（平时是高电平）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE // 我们这里使用轮询方式，不用中断
    };
    gpio_config(&io_conf);

    int press_count = 0; // 用于记录按下的时长

    while (1)
    {
        // 判断引脚是否为低电平（即按键被按下，连通了 GND）
        if (gpio_get_level(CLEAR_WIFI_BUTTON_PIN) == 0)
        {
            press_count++;
            // 长按了 3 秒钟
            if (press_count >= 3)
            {
                ESP_LOGW(TAG, "检测到长按操作，准备清除 WiFi 并重启...");
                clear_wifi_and_restart(); // 调用你已经写好的清除并重启函数
                press_count = 0;          // 防抖归零
            }
        }
        else
        {
            press_count = 0;
        }
        vTaskDelay(1000);
    }
}

// 回调函数，接收到前端的信息以后触发回调进行逻辑
static esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                          uint8_t **outbuf, ssize_t *outlen, void *priv_data)
{
    if (inbuf)
    {
        // 打印前端发来的内容
        ESP_LOGI(TAG, "📦 收到前端发来的自定义数据啦！长度: %d", (int)inlen);
        ESP_LOGI(TAG, "📝 内容是: %.*s", (int)inlen, (char *)inbuf);

        // 【你可以在这里写逻辑】：比如解析收到的 JSON，保存账号密码 Token 等
    }

    // 给前端回一个信，告诉他板子收到了
    const char response[] = "{\"status\":\"OK\",\"msg\":\"Device got your data!\"}";

    // 必须分配内存来返回数据，底层用完会自动释放
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL)
    {
        ESP_LOGE(TAG, "内存不足");
        return ESP_ERR_NO_MEM;
    }
    *outlen = strlen(response);
    return ESP_OK;
}
// 事件回调函数：当手机通过蓝牙给板子发指令时，这里会被触发
static void prov_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "🚀 配网已启动！请打开手机APP扫码或搜索蓝牙");
            break;
        case WIFI_PROV_CRED_RECV:
            ESP_LOGI(TAG, "📶 成功收到手机发来的 WiFi 账号和密码！");
            break;
        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "🎉 密码验证正确，配网成功！");
            break;
        case WIFI_PROV_END:
            // 配网完成后，系统会自动关闭蓝牙以节省内存和功耗
            ESP_LOGI(TAG, "🛑 配网流程结束，蓝牙已自动释放。");
            break;
        default:
            break;
        }
    }
}
// 新增：专门监听 WiFi 和 IP 事件的回调
static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) // 如果WiFi 启动了
    {
        esp_wifi_connect(); // 启动 WiFi
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) // 如果WiFi 断开了
    {
        // 如果断开了，尝试重连，并清除已连接标志
        esp_wifi_connect();
        xEventGroupClearBits(wifi_event_group, CONNECTED_BIT);
        ESP_LOGW(TAG, "WiFi 掉线，正在尝试重连...");
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "成功获取到 IP 地址: " IPSTR, IP2STR(&event->ip_info.ip));
        // 发出信号：网络彻底通畅了！
        xEventGroupSetBits(wifi_event_group, CONNECTED_BIT);
    }
}

void wifi_main()
{

    // freertos的事件组，作用是判断网络是否已经连接成功和否已经获取到IP地址
    wifi_event_group = xEventGroupCreate();
    xTaskCreate(button_monitor_task, "button_task", 4096, NULL, 5, NULL);
    // 1 初始化 NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    // 2. 初始化网络
    ESP_ERROR_CHECK(esp_netif_init());                // 初始化网络接口
    ESP_ERROR_CHECK(esp_event_loop_create_default()); // 创建默认的事件循环
    esp_netif_create_default_wifi_sta();              // 创建默认的WiFi 模式为从模式STA，接收WiFi
    // 3. 注册事件监听器
    esp_event_handler_instance_t instance_any_id, instance_got_ip, prov_end_instance;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, NULL, &prov_end_instance));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ip_event_handler, NULL, &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_ip_event_handler, NULL, &instance_got_ip));
    // 4. 初始化 WiFi 硬件驱动
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT(); // 默认配置
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    // 5. 初始化配网
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,                                        // 配网方案为BLE
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM}; // BLE事件处理函数，配置完以后不需要ble，
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));
    bool provisioned = false;                                    // 是否已经配网
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned)); // 检查nvs是否已经配网
    // 获取设备MAC地址用于标识,
    uint8_t mac[6];
    char mac_addr[18];
    ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac)); // 获取MAC地址
    snprintf(mac_addr, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    if (!provisioned)
    {
        // // 设备未配网，启动配网流程
        ESP_LOGI(TAG, "设备未配网，启动蓝牙广播...");
        // 配网加密密钥，后续可以更改为随机密码
        const char *security_key = "abcd1234";
        char service_name[16];
        snprintf(service_name, sizeof(service_name), "EchoPals-%02X%02X%02X", mac[3], mac[4], mac[5]);
        // 根据uuid生成一个带有名字的通道
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create("custom-data"));
        // 🌟 第二步：邮局开门营业（正式启动蓝牙配网广播），遍历内存的链表，根据链表名动态生成蓝牙特征
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning( // 配网方式
            WIFI_PROV_SECURITY_1,                         // 配网加密方式
            (const void *)security_key,                   // 配网密码
            // NULL,
            service_name, // 配网服务名
            NULL)         // 配网服务数据
        );
        // 接收到ap发来的数据之后进行处理，将带有我们自定义同通道名的数据放到回调函数中
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register("custom-data", custom_prov_data_handler, NULL));
        ESP_LOGI(TAG, "==== 自定义通道注册成功！请搜索蓝牙: %s ====", service_name);
    }
    else
    {
        ESP_LOGI(TAG, "设备已配网，直接连接...");
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
        // 注意：wifi_ip_event_handler 会在 STA_START 时自动调用 esp_wifi_connect()
    }
}
