#include "bsp_board.h"
#include "protocol/mqtt_protocol.h"

#define CLEAR_WIFI_BUTTON_PIN GPIO_NUM_0 // 长按清除 WiFi 的按键引脚
#define MAX_RETRY_COUNT 5                // 断线后最大自动重连次数

static const char *TAG = "EchoPals";

// ─── 模块级状态变量 ──────────────────────────────────────────────────────
static bool s_wifi_prov_initialized = false; // 配网管理器是否已初始化（防止按键误触发重置）
static bool s_is_provisioning = false;       // 是否正处于配网流程中（配网期间禁止自动重连）
static int s_retry_num = 0;                  // 当前已重连次数

// ─── 安全重启 ────────────────────────────────────────────────────────────

void clear_wifi_and_restart(void)
{
    ESP_LOGW(TAG, "正在清除已保存的 WiFi 账号密码...");

    // 发送设备重置通知到MQTT服务器
    send_reset_notification();

    // 调用配网管理器 API 清除 NVS 中存储的 WiFi 凭证
    esp_err_t err = wifi_prov_mgr_reset_provisioning();
    if (err == ESP_OK)
        ESP_LOGI(TAG, "清除成功，即将重启...");
    else
        ESP_LOGE(TAG, "清除失败: %s", esp_err_to_name(err));

    // 等待 1 秒，保证日志输出完毕再重启
    vTaskDelay(pdMS_TO_TICKS(1000));

    // 软件复位
    esp_restart();
}

// ─── 按键监控任务 ────────────────────────────────────────────────────────

static void button_monitor_task(void *pvParameters)
{
    // 配置 GPIO0 为输入，内部上拉，禁用中断（轮询方式）
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CLEAR_WIFI_BUTTON_PIN),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    // 应用 GPIO 配置
    gpio_config(&io_conf);

    int press_count = 0; // 连续低电平帧计数（每帧 10ms）

    while (1)
    {
        if (gpio_get_level(CLEAR_WIFI_BUTTON_PIN) == 0)
        {
            // 按键按下（低电平），累加计数
            press_count++;
            if (press_count >= 300) // 300 × 10ms = 3 秒长按
            {
                // 确保配网管理器 API 已就绪后才执行重置，防止过早调用崩溃
                if (s_wifi_prov_initialized)
                    clear_wifi_and_restart();
                press_count = 0;
            }
        }
        else
        {
            // 按键松开，重置计数
            press_count = 0;
        }
        // 10ms 轮询间隔，平衡响应速度与 CPU 占用
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── BLE 配网自定义数据处理 ──────────────────────────────────────────────

static esp_err_t custom_prov_data_handler(uint32_t session_id,
                                          const uint8_t *inbuf,
                                          ssize_t inlen,
                                          uint8_t **outbuf,
                                          ssize_t *outlen,
                                          void *priv_data)
{
    ESP_LOGI(TAG, "🟢 自定义端点回调触发！session_id: %lu, 收到数据长度: %d", session_id, (int)inlen);

    // 基础数据校验
    if (inbuf == NULL || inlen <= 0 || inlen >= 2048)
    {
        ESP_LOGE(TAG, "收到无效数据，长度异常: %d", (int)inlen);
        goto send_response;
    }

    // 安全拷贝，保证字符串以\0结尾，避免内存越界
    char *safe_str = calloc(1, inlen + 1);
    if (!safe_str)
    {
        ESP_LOGE(TAG, "内存分配失败，无法处理数据");
        goto send_response;
    }
    memcpy(safe_str, inbuf, inlen);
    ESP_LOGI(TAG, "收到原始数据: %s", safe_str);

    // 🌟 直接将收到的纯字符串（即 Token）保存到 NVS，不再进行 JSON 解析！
    nvs_handle_t h;
    esp_err_t err = nvs_open("net_config", NVS_READWRITE, &h);
    if (err == ESP_OK)
    {
        // 注意：如果你和后端约定叫 device_token，这里可以把 ws_token 改名
        err = nvs_set_str(h, "ws_token", safe_str);
        if (err == ESP_OK)
        {
            nvs_commit(h);
            ESP_LOGI(TAG, "✅ Token已永久保存到NVS！Token值: %s", safe_str);
        }
        else
        {
            ESP_LOGE(TAG, "Token写入NVS失败: %s", esp_err_to_name(err));
        }
        nvs_close(h);
    }
    else
    {
        ESP_LOGE(TAG, "NVS打开失败: %s", esp_err_to_name(err));
    }

    // 释放内存资源
    free(safe_str);

send_response:
    // 固定的 JSON 响应体，告诉前端设备接收成功
    const char response[] = "{\"status\":\"OK\"}";
    *outbuf = (uint8_t *)strdup(response);
    if (*outbuf == NULL)
        return ESP_ERR_NO_MEM;
    *outlen = strlen(response);
    return ESP_OK;
}
// ─── 配网事件处理 ────────────────────────────────────────────────────────

static void prov_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            ESP_LOGI(TAG, "配网启动，请打开 APP");
            // 进入配网流程，此期间禁止自动重连（避免与配网状态机冲突）
            s_is_provisioning = true;
            break;

        case WIFI_PROV_CRED_RECV:
            // APP 已发来 SSID + 密码，正在验证中
            ESP_LOGI(TAG, "收到账号密码，验证中...");
            break;

        case WIFI_PROV_CRED_SUCCESS:
            ESP_LOGI(TAG, "密码正确！");
            // 验证成功，退出配网保护，允许后续自动重连
            s_is_provisioning = false;
            // WiFi 已确认连上，手动停止配网（因为禁用了 auto_stop）
            // 配网管理器会向手机回报 success，然后触发 WIFI_PROV_END
            wifi_prov_mgr_stop_provisioning();
            break;

        case WIFI_PROV_CRED_FAIL:
            // 密码错误，禁止在此处手动调用 esp_wifi_connect（由状态机控制）
            ESP_LOGE(TAG, "密码错误！");
            break;

        case WIFI_PROV_END:
        {
            ESP_LOGI(TAG, "配网流程彻底结束。");
            bsp_board_t *board = (bsp_board_t *)arg;
            if (board)
            {
                // 设置 PROV_DONE_BIT，解除 bsp_board_wifi_main 中的阻塞等待
                xEventGroupSetBits(board->board_status, PROV_DONE_BIT);
            }
            break;
        }

        default:
            break;
        }
    }
}

// ─── WiFi / IP 事件处理 ──────────────────────────────────────────────────

static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    bsp_board_t *bsp_board = (bsp_board_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // 配网期间由配网管理器控制连接，不能抢先调用 connect
        // 否则会用空凭证连接导致立刻失败，配网管理器状态被污染为 "failed"
        if (!s_is_provisioning)
            esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // 掉线：配网期间不重连（避免抢占配网状态机）
        if (!s_is_provisioning)
        {
            if (s_retry_num < MAX_RETRY_COUNT)
            {
                // 未超过最大重试次数，继续尝试重连
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGW(TAG, "WiFi 掉线，正在重连... (%d/%d)", s_retry_num, MAX_RETRY_COUNT);
            }
            else
            {
                // 超过最大重试，置位 WIFI_FAIL_BIT 告知上层放弃
                if (bsp_board)
                    xEventGroupSetBits(bsp_board->board_status, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "重连失败，放弃连接。");
            }
        }
        // 无论原因，掉线时清除 WIFI_BIT（通知其他模块网络不可用）
        if (bsp_board)
            xEventGroupClearBits(bsp_board->board_status, WIFI_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "成功获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));

        // 连接成功，重置重连计数
        s_retry_num = 0;

        // 置位 WIFI_BIT，解除 bsp_board_wifi_main 末尾的阻塞等待
        if (bsp_board)
            xEventGroupSetBits(bsp_board->board_status, WIFI_BIT);
    }
}

// ─── WiFi 主初始化入口 ───────────────────────────────────────────────────

void bsp_board_wifi_main(bsp_board_t *bsp_board)
{
    // 前置依赖检查：NVS 必须已初始化（WiFi 凭证存放在 NVS）
    if (!bsp_board_check_status(bsp_board, NVS_BIT, 0))
    {
        ESP_LOGE(TAG, "NVS not initialized");
        return;
    }

    // // 临时硬编码写入 Token，测试用
    // nvs_handle_t h;
    // if (nvs_open("net_config", NVS_READWRITE, &h) == ESP_OK)
    // {
    //     nvs_set_str(h, "ws_token", "test_token_888");
    //     nvs_commit(h);
    //     nvs_close(h);
    //     ESP_LOGI(TAG, "✅ 测试Token已写入：test_token_888");
    // }

    // 初始化 TCP/IP 网络协议栈
    ESP_ERROR_CHECK(esp_netif_init());

    // 创建默认事件循环（防止重复创建导致 panic）
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE)
        ESP_ERROR_CHECK(err);

    // 创建默认 WiFi STA 网络接口（分配 IP、DNS 等）
    esp_netif_create_default_wifi_sta();

    // 注册三类事件监听，将 bsp_board 指针传入作为上下文
    esp_event_handler_instance_t instance_any_id, instance_got_ip, prov_end_instance;
    // 监听配网事件（WIFI_PROV_EVENT）→ 用于更新 PROV_DONE_BIT
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_PROV_EVENT, ESP_EVENT_ANY_ID, &prov_event_handler, bsp_board, &prov_end_instance));
    // 监听 WiFi 事件（连接/断开）→ 用于重连和更新 WIFI_BIT
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_ip_event_handler, bsp_board, &instance_any_id));
    // 监听 IP 获取事件 → 用于置位 WIFI_BIT
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_ip_event_handler, bsp_board, &instance_got_ip));

    // 初始化 WiFi 驱动（使用默认配置）
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // 初始化 BLE 配网管理器（scheme 选 BLE，配网结束后自动释放蓝牙内存）
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    // 管理器已就绪，允许按键任务执行重置操作
    s_wifi_prov_initialized = true;

    // 启动按键监控任务（检测长按 GPIO0 触发 WiFi 重置）
    xTaskCreate(button_monitor_task, "btn_task", 4096, NULL, 5, NULL);

    // 查询 NVS 中是否已存储过 WiFi 凭证
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned)
    {
        ESP_LOGI(TAG, "设备未配网，启动蓝牙广播...");

        // 读取 MAC 地址后三字节，生成唯一蓝牙服务名
        uint8_t mac[6];
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
        char service_name[16];
        snprintf(service_name, sizeof(service_name), "EchoPals-%02X%02X%02X", mac[3], mac[4], mac[5]);
        ESP_LOGE(TAG, " 我的真实MAC地址是: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
        // 使用固定 PoP 密码（生产环境建议换为 MAC 派生的动态密码）
        const char *security_key = "abcd1234";

        //! 创建自定义数据端点（用于 APP 下发额外配置）,用于接收token
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create("custom-data"));
        wifi_prov_mgr_disable_auto_stop(100);
        // 启动 BLE 配网广播（SECURITY_1 = 带 PoP 校验）
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_1, security_key, service_name, NULL));

        // 注册自定义端点的数据处理回调
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register(
            "custom-data", custom_prov_data_handler, NULL));

        ESP_LOGI(TAG, "==== 蓝牙: %s, 密码: %s ====", service_name, security_key);

        // 阻塞等待用户通过 APP 完成配网（添加2分钟超时机制）
        EventBits_t wait_bits = xEventGroupWaitBits(bsp_board->board_status, PROV_DONE_BIT,
                                                    pdFALSE, pdFALSE, pdMS_TO_TICKS(120000)); // 120秒超时

        if (!(wait_bits & PROV_DONE_BIT))
        {
            ESP_LOGE(TAG, "配网超时，强制退出配网流程");
            // 清理配网资源防止内存泄漏
            wifi_prov_mgr_deinit();
            // 重置状态变量
            s_wifi_prov_initialized = false;
            s_is_provisioning = false;
            // 触发错误处理（可选：重启配网或系统）
            esp_restart();
        }

        // 配网流程结束，释放配网管理器（回收 BLE 基带内存，约几十 KB）
        wifi_prov_mgr_deinit();
    }
    else
    {
        ESP_LOGI(TAG, "设备已配网，直接连接...");

        // 已配网无需管理器，直接释放节省内存
        wifi_prov_mgr_deinit();

        // 设置 WiFi 为 STA 模式并启动驱动（触发 WIFI_EVENT_STA_START）
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    // 最终阻塞：等待网络彻底就绪或彻底失败后才返回
    // 调用方拿到函数返回即可认为网络状态已确定
    ESP_LOGI(TAG, "等待网络彻底就绪...");
    EventBits_t bits = xEventGroupWaitBits(
        bsp_board->board_status,
        WIFI_BIT | WIFI_FAIL_BIT, // 等待任一位被置位
        pdFALSE,                  // 不清除位（其他模块也可能读取）
        pdFALSE,                  // 任一位满足即返回（OR 等待）
        portMAX_DELAY);

    if (bits & WIFI_BIT)
        ESP_LOGI(TAG, "网络就绪，主程序可以开始了！");
    else if (bits & WIFI_FAIL_BIT)
    {
        // 连接彻底失败，等待 30 秒后自动重启（给用户时间观察日志）
        ESP_LOGE(TAG, "网络连接最终失败，30秒后自动重启...");
        vTaskDelay(pdMS_TO_TICKS(30000));
        esp_restart();
    }
}
