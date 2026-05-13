#include "bsp_board.h"
#include "protocol/mqtt_protocol.h"
#include "protocol/auth.h"
#include "esp_heap_caps.h"
// ─── 模块常量 ─────────────────────────────────────────────────────────────────
#define CLEAR_WIFI_BUTTON_PIN GPIO_NUM_0 ///< 清除 WiFi 凭证的长按按键（Boot 按钮）
#define MAX_RETRY_COUNT 5                ///< WiFi 断线后最大自动重连次数

static const char *TAG = "EchoPals";

// ─── 模块级状态变量 ────────────────────────────────────────────────────────────
/// @brief 配网管理器是否已初始化（防止按键在管理器就绪前触发重置导致崩溃）
static bool s_wifi_prov_initialized = false;
/// @brief 是否正处于 BLE 配网流程中（配网期间禁止断线重连，避免与配网状态机冲突）
static bool s_is_provisioning = false;
/// @brief 当前已重连次数（超过 MAX_RETRY_COUNT 后置位 WIFI_FAIL_BIT）
static int s_retry_num = 0;

// ─── clear_wifi_and_restart ──────────────────────────────────────────────────

/**
 * @brief 清除 NVS 中存储的 WiFi 凭证并软件重启
 *
 * 用于恢复出厂网络状态：擦除已配对的 WiFi SSID 和密码，
 * 重启后设备会重新进入 BLE 配网模式，等待 App 重新配网。
 * 重启前会向 MQTT 发送重置通知，便于服务端记录设备重置事件。
 *
 * @param 无
 * @return 无（函数内部调用 esp_restart() 重启设备，不会返回）
 *
 * @note 调用者：button_monitor_task()（检测到长按 3 秒后调用）
 * @note 副作用：所有 NVS WiFi 配置被清除，设备重启
 */
void clear_wifi_and_restart(void)
{
    ESP_LOGW(TAG, "正在清除已保存的 WiFi 账号密码...");
    ESP_LOGW(TAG, "正在清除已保存的driver-token...");
    // ── 向 MQTT 发送重置通知（让服务端知道设备主动重置）─────────────────────
    // 注意：此时 WiFi 可能仍然连接，发送还能成功
    send_reset_notification();

    // ── 清除 NVS 中的认证 Token ──────────────────────────────────────────────
    nvs_handle_t h;
    esp_err_t err = nvs_open("net_config", NVS_READWRITE, &h);
    if (err == ESP_OK)
    {
        // 清除 device token (ws_token) 和 access token
        err = nvs_erase_key(h, "ws_token");
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG, "清除 ws_token 时出错: %s", esp_err_to_name(err));

        err = nvs_erase_key(h, "access_token");
        if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND)
            ESP_LOGW(TAG, "清除 access_token 时出错: %s", esp_err_to_name(err));

        nvs_commit(h);
        nvs_close(h);
        ESP_LOGI(TAG, "Driver token 和 Access token 清除成功");
    }
    else
    {
        ESP_LOGE(TAG, "无法打开 net_config 命名空间清除 tokens: %s", esp_err_to_name(err));
    }

    // ── 清除 NVS 中的 WiFi 凭证 ──────────────────────────────────────────────
    // wifi_prov_mgr_reset_provisioning() 内部删除 WiFi 配置分区中的 SSID/密码键值对
    // err = wifi_prov_mgr_reset_provisioning();
    err = esp_wifi_restore(); // 它可以在任何状态下安全地抹除 WiFi 配置
    if (err == ESP_OK)
        ESP_LOGI(TAG, "WiFi 凭证清除成功，即将重启...");
    else
        ESP_LOGE(TAG, "WiFi 凭证清除失败: %s", esp_err_to_name(err));

    // ── 等待 1 秒，保证日志输出完毕，便于用户通过串口观察 ───────────────────
    vTaskDelay(pdMS_TO_TICKS(1000));

    // ── 软件复位（重启后重新进入配网模式）───────────────────────────────────
    esp_restart();
}

// ─── button_monitor_task ─────────────────────────────────────────────────────

/**
 * @brief 按键监控任务：检测 GPIO0 长按（3 秒）触发 WiFi 重置
 *
 * 以 10ms 为周期轮询 GPIO0 电平：
 *   - 按下（低电平）：累加计数
 *   - 连续 300 次（3 秒）：触发 clear_wifi_and_restart()
 *   - 松开（高电平）：重置计数
 *
 * @param pvParameters 未使用（FreeRTOS 任务参数）
 * @return 无（永远运行的 FreeRTOS 任务）
 *
 * @note 调用者：bsp_board_wifi_main() 通过 xTaskCreatePinnedToCoreWithCaps() 创建
 * @note 运行核心：CPU0，栈 4096 字节（SPIRAM 分配）
 * @note 内部保护：s_wifi_prov_initialized 为 false 时不执行重置（管理器未就绪）
 */
static void button_monitor_task(void *pvParameters)
{
    // ── 步骤 1：配置 GPIO0 为输入模式（内部上拉，轮询检测）─────────────────
    // Boot 按键（GPIO0）通过 10kΩ 上拉连接到 3.3V，按下时接地，电平变低
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << CLEAR_WIFI_BUTTON_PIN), // 只配置 GPIO0
        .mode = GPIO_MODE_INPUT,                         // 输入模式
        .pull_up_en = GPIO_PULLUP_ENABLE,                // 启用内部上拉（按键断开时保持高电平）
        .pull_down_en = GPIO_PULLDOWN_DISABLE,           // 禁用下拉
        .intr_type = GPIO_INTR_DISABLE,                  // 不使用中断（轮询模式）
    };
    gpio_config(&io_conf);

    int press_count = 0; // 连续低电平帧计数（每帧 10ms）

    // ── 步骤 2：轮询主循环 ────────────────────────────────────────────────────
    while (1)
    {
        if (gpio_get_level(CLEAR_WIFI_BUTTON_PIN) == 0)
        {
            // 按键按下（低电平），累加计数
            press_count++;
            if (press_count >= 300) // 300 × 10ms = 3 秒持续按压
            {
                // 前置检查：确保配网管理器已初始化，避免过早调用导致崩溃
                if (s_wifi_prov_initialized)
                    clear_wifi_and_restart(); // 触发清除和重启（不会返回）
                press_count = 0;              // 不可达代码，保险起见重置计数
            }
        }
        else
        {
            // 按键松开（高电平），重置计数（需要连续 3 秒不间断才触发）
            press_count = 0;
        }

        // 10ms 轮询间隔：响应速度（≤10ms 延迟）与 CPU 占用（~1%）的平衡点
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ─── custom_prov_data_handler ────────────────────────────────────────────────

/**
 * @brief BLE 配网自定义数据端点回调（接收 App 下发的 device Token）
 *
 * 在 BLE 配网过程中，App 除了推送 WiFi SSID/密码外，还会通过自定义端点
 * "custom-data" 下发设备绑定 Token（deviceToken）。本函数处理该数据：
 *   1. 解析 JSON：{"token": "xxxxxx"}
 *   2. 提取 token 字段，写入 NVS（命名空间 "net_config"，键 "ws_token"）
 *   3. 调用 wifi_prov_mgr_stop_provisioning() 结束配网流程
 *   4. 返回固定 JSON 响应 {"status":"OK"}
 *
 * @param session_id 当前 BLE 配网会话 ID（调试用）
 * @param inbuf      App 发来的原始数据（JSON 字符串）
 * @param inlen      原始数据长度（字节）
 * @param outbuf     响应数据指针（由此函数分配，框架负责释放）
 * @param outlen     响应数据长度
 * @param priv_data  私有数据（未使用）
 * @return ESP_OK 处理成功（即使 token 解析失败也返回 OK，附带响应体）
 *
 * @note 调用者：BLE 配网框架（wifi_prov_mgr_endpoint_register 注册后自动调用）
 * @note 线程安全：由配网框架在单独任务中调用，需注意 NVS 并发写入
 */
static esp_err_t custom_prov_data_handler(uint32_t session_id,
                                          const uint8_t *inbuf,
                                          ssize_t inlen,
                                          uint8_t **outbuf,
                                          ssize_t *outlen,
                                          void *priv_data)
{
    ESP_LOGI(TAG, "自定义端点回调触发！session_id: %lu, 收到数据长度: %d",
             session_id, (int)inlen);

    // ── 步骤 1：基础数据校验（防止空指针和超大输入）─────────────────────────
    if (inbuf == NULL || inlen <= 0 || inlen >= 2048)
    {
        ESP_LOGE(TAG, "收到无效数据，长度异常: %d", (int)inlen);
        goto send_response; // 跳到响应部分，返回 OK 但不做任何操作
    }

    // ── 步骤 2：安全拷贝输入数据（添加 \0 结尾，防止字符串越界）─────────────
    char *safe_str = calloc(1, inlen + 1); // calloc 自动清零（包含 \0 结尾）
    if (!safe_str)
    {
        ESP_LOGE(TAG, "内存分配失败，无法处理配网数据");
        goto send_response;
    }
    memcpy(safe_str, inbuf, inlen);
    ESP_LOGI(TAG, "收到原始配网数据: %s", safe_str);

    // ── 步骤 3：JSON 解析，提取 token 字段 ────────────────────────────────────
    cJSON *root = cJSON_Parse(safe_str);
    if (!root)
    {
        ESP_LOGE(TAG, "JSON 解析失败，数据不是合法 JSON 格式");
        free(safe_str);
        goto send_response;
    }

    cJSON *token_item = cJSON_GetObjectItem(root, "token");
    if (!cJSON_IsString(token_item) || token_item->valuestring == NULL)
    {
        ESP_LOGE(TAG, "JSON 中未找到 'token' 字段，或 token 不是字符串类型");
        cJSON_Delete(root);
        free(safe_str);
        goto send_response;
    }

    ESP_LOGI(TAG, "成功提取 Token: %.20s...", token_item->valuestring);

    // ── 步骤 4：将 Token 写入 NVS 持久化 ──────────────────────────────────────
    // 命名空间 "net_config"，键 "ws_token"（session.c 读取时使用键 "device_token"）
    nvs_handle_t h;
    esp_err_t err = nvs_open("net_config", NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS 打开失败（net_config）: %s", esp_err_to_name(err));
        cJSON_Delete(root);
        free(safe_str);
        goto send_response;
    }

    err = nvs_set_str(h, "ws_token", token_item->valuestring);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Token 写入 NVS 失败: %s", esp_err_to_name(err));
        nvs_close(h);
        cJSON_Delete(root);
        free(safe_str);
        goto send_response;
    }

    err = nvs_commit(h); // 将写入缓冲区刷入 Flash（掉电不丢失）
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS commit 失败: %s", esp_err_to_name(err));
        nvs_close(h);
        cJSON_Delete(root);
        free(safe_str);
        goto send_response;
    }

    nvs_close(h);
    ESP_LOGI(TAG, "Token 已永久写入 NVS！");

    // ── 步骤 5：Token 保存成功后，主动停止配网广播 ────────────────────────────
    // wifi_prov_mgr_disable_auto_stop(3000) 保证本函数返回（App 收到 {status:OK} 响应）
    // 之后 3000ms 才真正断开 BLE，避免原先"App 显示配网失败"的竞态问题。
    // 不调此函数 → PROV_DONE_BIT 永远不置位 → bsp_board_wifi_main 阻塞 120s 后强制重启，
    // session_init / audio_init 等后续所有初始化永远不执行（偶发"写 NVS 成功后卡死"根因）。
    wifi_prov_mgr_stop_provisioning();

    // ── 步骤 6：释放资源 ───────────────────────────────────────────────────────
    cJSON_Delete(root);
    free(safe_str);

    // ── 步骤 7：构造并返回响应 ────────────────────────────────────────────────
send_response:
    // 固定响应体：{"status":"OK"}，告知 App 已收到数据（无论成功失败）
    const char response[] = "{\"status\":\"OK\"}";
    *outbuf = (uint8_t *)strdup(response); // 由框架在发送后 free
    if (*outbuf == NULL)
        return ESP_ERR_NO_MEM;
    *outlen = strlen(response);
    return ESP_OK;
}

// ─── prov_event_handler ──────────────────────────────────────────────────────

/**
 * @brief BLE 配网事件处理（5 种配网生命周期事件）
 *
 * 处理 BLE 配网管理器产生的事件，维护配网状态变量，
 * 在配网结束时置位 PROV_DONE_BIT 解除 bsp_board_wifi_main() 的阻塞等待。
 *
 * @param arg        用户参数（bsp_board_t* 指针，用于置位事件组）
 * @param event_base 事件基类（WIFI_PROV_EVENT）
 * @param event_id   具体事件 ID
 * @param event_data 事件相关数据（各事件含义不同，本函数未使用）
 * @return void
 *
 * @note 调用者：esp_event 框架（esp_event_handler_instance_register 注册）
 * @note 线程安全：事件回调在 esp_event 任务中执行，禁止在回调中阻塞
 */
static void prov_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data)
{
    if (event_base == WIFI_PROV_EVENT)
    {
        switch (event_id)
        {
        case WIFI_PROV_START:
            // 配网广播已启动，App 可以扫描到设备蓝牙信号
            ESP_LOGI(TAG, "BLE 配网启动，请打开 App 扫描并配网");
            s_is_provisioning = true; // 标记配网进行中，禁止断线自动重连

            break;

        case WIFI_PROV_CRED_RECV:
            // App 已发来 WiFi SSID 和密码，管理器正在验证连接
            ESP_LOGI(TAG, "收到 WiFi 账号密码，正在验证连接...");
            break;

        case WIFI_PROV_CRED_SUCCESS:
            // WiFi 密码验证成功，已获取到 IP
            ESP_LOGI(TAG, "WiFi 密码正确，连接成功！");
            s_is_provisioning = false; // 退出配网保护，允许后续断线自动重连
            break;

        case WIFI_PROV_CRED_FAIL:
            // WiFi 密码错误（SSID 不存在或密码错误）
            // 不在此处手动调用 esp_wifi_connect()，由配网状态机控制后续流程
            ESP_LOGE(TAG, "WiFi 密码错误！请重新配网");
            break;

        case WIFI_PROV_END:
        {
            // 配网流程彻底结束（成功配网或超时），无论哪种情况都解除阻塞
            ESP_LOGI(TAG, "BLE 配网流程结束");
            bsp_board_t *board = (bsp_board_t *)arg;
            if (board)
            {
                // 置位 PROV_DONE_BIT，解除 bsp_board_wifi_main() 中的
                // xEventGroupWaitBits(PROV_DONE_BIT) 阻塞
                xEventGroupSetBits(board->board_status, PROV_DONE_BIT);
            }
            break;
        }

        default:
            break;
        }
    }
}

// ─── wifi_ip_event_handler ───────────────────────────────────────────────────

/**
 * @brief WiFi 连接 / 断开 / 获取 IP 事件处理
 *
 * 统一处理三类 WiFi 相关事件：
 *   - WIFI_EVENT_STA_START      : STA 模式启动 → 立即尝试连接
 *   - WIFI_EVENT_STA_DISCONNECTED: 断线 → 重连（配网期间除外）或置位失败标志
 *   - IP_EVENT_STA_GOT_IP       : 获取到 IP → 置位 WIFI_BIT，解除等待
 *
 * @param arg        用户参数（bsp_board_t* 指针，操作事件组）
 * @param event_base 事件基类（WIFI_EVENT 或 IP_EVENT）
 * @param event_id   具体事件 ID
 * @param event_data 事件数据（获取 IP 时为 ip_event_got_ip_t*）
 * @return void
 *
 * @note 调用者：esp_event 框架（两个 handler instance 共享此函数）
 * @note 线程安全：事件回调禁止阻塞，WIFI_BIT 操作是原子的
 */
static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    bsp_board_t *bsp_board = (bsp_board_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // ── STA 模式已启动，立即尝试连接（使用 NVS 中已存储的 SSID/密码）────
        esp_wifi_connect();
    }
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED)
    {
        // ── WiFi 断线处理 ──────────────────────────────────────────────────────
        if (!s_is_provisioning) // 配网期间不触发重连（避免抢占配网状态机）
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
                // 超过最大重试次数，放弃重连，通知上层处理（通常是重启）
                if (bsp_board)
                    xEventGroupSetBits(bsp_board->board_status, WIFI_FAIL_BIT);
                ESP_LOGE(TAG, "WiFi 重连失败，已达最大重试次数 (%d)，放弃连接", MAX_RETRY_COUNT);
            }
        }

        // 无论原因，断线时清除 WIFI_BIT（通知其他模块网络不可用，停止 WebSocket 操作）
        if (bsp_board)
            xEventGroupClearBits(bsp_board->board_status, WIFI_BIT);
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        // ── 成功获取 IP 地址 ──────────────────────────────────────────────────
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "成功获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));

        // 连接成功，重置重连计数（下次断线时从 0 开始重新计数）
        s_retry_num = 0;

        // 置位 WIFI_BIT，解除 bsp_board_wifi_main() 末尾的 xEventGroupWaitBits 阻塞
        if (bsp_board)
            xEventGroupSetBits(bsp_board->board_status, WIFI_BIT);
    }
}

// ─── bsp_board_wifi_main ─────────────────────────────────────────────────────

/**
 * @brief WiFi 完整初始化入口（阻塞直至网络就绪或彻底失败）
 *
 * 完整流程：
 *   1. 前置检查（NVS_BIT 必须已置位）
 *   2. 初始化 TCP/IP 协议栈和默认事件循环
 *   3. 注册 WiFi/IP/配网 三类事件监听
 *   4. 初始化 WiFi 驱动（WIFI_INIT_CONFIG_DEFAULT）
 *   5. 初始化 BLE 配网管理器
 *   6a. NVS 无凭证 → BLE 配网广播，等待 App 配网（120s 超时）
 *   6b. NVS 有凭证 → 直接 STA 连接
 *   7. 最终阻塞等待 WIFI_BIT 或 WIFI_FAIL_BIT
 *   8. WIFI_FAIL_BIT → 等待 30s 后重启
 *
 * @param bsp_board BSP 实例指针（通过 board_status 管理状态位）
 * @return void（阻塞直到网络就绪；WIFI_FAIL_BIT 时触发重启不返回）
 *
 * @note 调用者：application.c → application_init()（步骤 5）
 * @note 前置条件：NVS_BIT 已置位（bsp_board_nvs_init() 已完成）
 * @note 此函数是阻塞的，可能等待数十秒（配网超时 120s）
 * @note BLE 配网密码：固定 "abcd1234"（生产建议改为 MAC 派生动态密码）
 */
void bsp_board_wifi_main(bsp_board_t *bsp_board)
{
    // ── 前置条件检查：NVS 必须已初始化（WiFi 凭证存在 NVS 中）─────────────
    if (!bsp_board_check_status(bsp_board, NVS_BIT, 0))
    {
        ESP_LOGE(TAG, "NVS 未初始化，无法启动 WiFi");
        return;
    }

    // ── 步骤 1：初始化 TCP/IP 协议栈 ─────────────────────────────────────────
    ESP_ERROR_CHECK(esp_netif_init());

    // ── 步骤 2：创建默认事件循环（防止重复创建导致 panic）───────────────────
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) // INVALID_STATE = 已存在，可忽略
        ESP_ERROR_CHECK(err);

    // 创建默认 WiFi STA 网络接口（分配 IP、DNS、路由等网络信息）
    esp_netif_create_default_wifi_sta();

    // ── 步骤 3：注册三类事件监听 ─────────────────────────────────────────────
    esp_event_handler_instance_t instance_any_id, instance_got_ip, prov_end_instance;

    // 监听 BLE 配网事件（WIFI_PROV_EVENT：START/CRED_RECV/SUCCESS/FAIL/END）
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_PROV_EVENT, ESP_EVENT_ANY_ID,
        &prov_event_handler, bsp_board, &prov_end_instance));

    // 监听 WiFi 事件（STA_START 和 STA_DISCONNECTED）
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_ip_event_handler, bsp_board, &instance_any_id));

    // 监听 IP 获取事件（STA_GOT_IP）→ 置位 WIFI_BIT
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_ip_event_handler, bsp_board, &instance_got_ip));

    // ── 步骤 4：初始化 WiFi 驱动（使用默认配置，自动分配缓冲区）─────────────
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // ── 步骤 5：初始化 BLE 配网管理器 ────────────────────────────────────────
    // scheme_ble：使用蓝牙 BLE 作为配网传输通道
    // FREE_BTDM：配网结束后自动释放 BLE 基带内存（约 60KB），回收给系统使用
    wifi_prov_mgr_config_t config = {
        .scheme = wifi_prov_scheme_ble,
        .scheme_event_handler = WIFI_PROV_SCHEME_BLE_EVENT_HANDLER_FREE_BTDM,
    };
    ESP_ERROR_CHECK(wifi_prov_mgr_init(config));

    // 管理器初始化完成，允许按键任务执行重置操作（防止管理器未就绪时崩溃）
    s_wifi_prov_initialized = true;

    // ── 步骤 6：启动按键监控任务（GPIO0 长按 3s 触发 WiFi 重置）─────────────
    // xTaskCreatePinnedToCoreWithCaps(
    //     button_monitor_task, "btn_task",
    //     4096, NULL, 5, NULL,
    //     0,                  // CPU0
    //     MALLOC_CAP_SPIRAM); // 栈分配在外部 SPIRAM（节省内部 SRAM）

    // button_monitor_task 会调用 nvs_erase_key/nvs_set_str/nvs_commit（Flash 操作），
    // Flash 操作占用 SPI 总线期间 CPU 需访问任务栈，栈必须在内部 SRAM，否则 WDT 复位。
    xTaskCreatePinnedToCoreWithCaps(
        button_monitor_task, "btn_task",
        3072, NULL, 5, NULL,
        0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "[内存] btn_task 创建后 → 内部SRAM剩余: %u B，PSRAM剩余: %u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    // ── 步骤 7：检查是否已配网 ───────────────────────────────────────────────
    bool provisioned = false;
    ESP_ERROR_CHECK(wifi_prov_mgr_is_provisioned(&provisioned));

    if (!provisioned)
    {
        // ════ 未配网分支：启动 BLE 广播，等待 App 配网 ════════════════════════
        ESP_LOGI(TAG, "设备未配网，启动 BLE 配网广播...");

        // 读取 MAC 地址后三字节，生成唯一蓝牙服务名（格式：EchoPals-AABBCC）
        // 确保多台设备同时配网时不冲突
        uint8_t mac[6];
        ESP_ERROR_CHECK(esp_wifi_get_mac(WIFI_IF_STA, mac));
        char service_name[16];
        snprintf(service_name, sizeof(service_name),
                 "EchoPals-%02X%02X%02X", mac[3], mac[4], mac[5]);
        ESP_LOGE(TAG, "设备 MAC: %02x:%02x:%02x:%02x:%02x:%02x",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

        const char *security_key = "abcd1234"; // PoP（Proof of Possession）密码

        // 创建自定义数据端点（用于 App 下发 device Token，与 WiFi 凭证分开传输）
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_create("custom-data"));
        wifi_prov_mgr_disable_auto_stop(3000); // 禁用自动停止（3000ms 延迟）

        // 启动 BLE 广播（SECURITY_1 = 带 PoP 校验，防止未授权配网）
        ESP_ERROR_CHECK(wifi_prov_mgr_start_provisioning(
            WIFI_PROV_SECURITY_1, security_key, service_name, NULL));

        // 注册自定义端点处理函数（接收 App 下发的 Token）
        ESP_ERROR_CHECK(wifi_prov_mgr_endpoint_register(
            "custom-data", custom_prov_data_handler, NULL));

        ESP_LOGI(TAG, "BLE 广播启动 → 蓝牙名: %s, 配网密码: %s", service_name, security_key);

        // 阻塞等待 App 完成配网（PROV_DONE_BIT 由 prov_event_handler 置位）
        // 超时 120 秒：防止设备永远卡在配网模式
        EventBits_t wait_bits = xEventGroupWaitBits(
            bsp_board->board_status, PROV_DONE_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(120000)); // 120s 超时

        if (!(wait_bits & PROV_DONE_BIT))
        {
            // 配网超时（用户 120 秒内未完成配网），强制退出并重启
            ESP_LOGE(TAG, "配网超时（120 秒），强制重启设备");
            wifi_prov_mgr_deinit();
            s_wifi_prov_initialized = false;
            s_is_provisioning = false;
            esp_restart();
        }

        // 配网流程结束，释放 BLE 基带内存（配网管理器内部调用 FREE_BTDM 释放蓝牙）
        wifi_prov_mgr_deinit();
    }
    else
    {
        // ════ 已配网分支：直接 STA 模式连接（跳过 BLE 广播）══════════════════
        ESP_LOGI(TAG, "设备已配网，直接连接 WiFi...");

        // 无需配网管理器，立即释放（节省约 60KB 内存）
        wifi_prov_mgr_deinit();

        // 设置 STA 模式并启动 WiFi 驱动（触发 WIFI_EVENT_STA_START → esp_wifi_connect()）
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
        ESP_ERROR_CHECK(esp_wifi_start());
    }

    // ── 步骤 8：最终阻塞等待网络就绪或彻底失败 ──────────────────────────────
    // 调用方拿到函数返回即可认为网络状态已确定（要么 WIFI_BIT 置位，要么重启）
    ESP_LOGI(TAG, "等待 WiFi 连接完成...");
    EventBits_t bits = xEventGroupWaitBits(
        bsp_board->board_status,
        WIFI_BIT | WIFI_FAIL_BIT, // 等待任一位被置位（OR 等待，pdFALSE）
        pdFALSE,                  // 不清除位（其他模块也可能等待 WIFI_BIT）
        pdFALSE,                  // OR 模式：任一位满足即返回（不需要两个都满足）
        portMAX_DELAY);           // 永久等待（超时由 WIFI_FAIL_BIT 分支处理重启）

    if (bits & WIFI_BIT)
    {
        ESP_LOGI(TAG, "WiFi 就绪，网络可用！");
    }
    else if (bits & WIFI_FAIL_BIT)
    {
        // WiFi 彻底失败（5 次重连全部失败），等 30s 后自动重启
        // 等待让用户有时间通过串口查看错误日志
        ESP_LOGE(TAG, "WiFi 连接最终失败，30 秒后自动重启...");
        vTaskDelay(pdMS_TO_TICKS(30000));
        esp_restart();
    }
}
