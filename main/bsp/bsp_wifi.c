#include "bsp_board.h"
#include "protocol/mqtt_protocol.h"
#include "protocol/auth.h"
#include "esp_heap_caps.h"
#include "freertos/timers.h" /* WiFi 断线去抖软件定时器 */
#include "ui/ui_port.h"      /* ui_show_unbinding(): 解绑前显示静态提示页，避免 GIF 卡冻帧 */

/* ── BluFi 配网相关（替代原 Unified Provisioning）─────────────────────────── */
#include "esp_blufi_api.h"        /* BluFi 事件枚举、回调结构、send 接口 */
#include "esp_blufi.h"            /* esp_blufi_adv_start/stop、profile 等 */
#include "blufi/blufi_security.h" /* 工程内拷贝：安全协商 + NimBLE 主机管理 */
#include "esp_mac.h"              /* esp_read_mac（蓝牙名派生）*/
#if CONFIG_BT_NIMBLE_ENABLED
#include "services/gap/ble_svc_gap.h" /* ble_svc_gap_device_name_set */
#endif
// ─── 模块常量 ─────────────────────────────────────────────────────────────────
#define CLEAR_WIFI_BUTTON_PIN GPIO_NUM_0 ///< 清除 WiFi 凭证的长按按键（Boot 按钮）
#define MAX_RETRY_COUNT 5                ///< WiFi 断线后最大自动重连次数

/// @brief WiFi 断线去抖时长（ms）：断线后等这么久仍未恢复，才通知上层断开 WS/MQTT。
/// 绝大多数 WiFi 抖动在 1~2 秒内自愈，期间不重建协议层 → 避免内部 SRAM 碎片化。
#define WIFI_DEBOUNCE_MS 1500

static const char *TAG = "EchoPals";

// ─── 模块级状态变量 ────────────────────────────────────────────────────────────

/// @brief 是否正处于 BLE 配网流程中（配网期间禁止断线重连，避免与配网状态机冲突）
static bool s_is_provisioning = false;
/// @brief 当前已重连次数（超过 MAX_RETRY_COUNT 后置位 WIFI_FAIL_BIT）
static int s_retry_num = 0;

/// @brief WiFi 断线去抖定时器（one-shot）：断线时启动，GOT_IP 时取消
static TimerHandle_t s_wifi_debounce_timer = NULL;
/// @brief 供去抖定时器回调访问的 bsp_board 指针（事件 handler 中保存）
static bsp_board_t *s_debounce_board = NULL;

// ─── BluFi 配网状态变量 ────────────────────────────────────────────────────────
/// @brief 当前 BLE（GATT）是否已连接（手机已连上设备蓝牙）
static bool s_blufi_ble_connected = false;
/// @brief 供 BluFi 回调访问的 bsp_board 指针（wifi_main 中保存）
static bsp_board_t *s_blufi_board = NULL;
/// @brief BluFi 收到并下发给 esp_wifi 的 STA 配置（SSID/密码暂存）
static wifi_config_t s_blufi_sta_config = {0};
/// @brief 是否有待回传的 WiFi 列表请求（GET_WIFI_LIST 非阻塞扫描标志）：
/// 回调里启动非阻塞扫描时置 true，SCAN_DONE 事件回传后清 false。
/// 用于区分"配网请求的扫描"与其它来源的扫描，避免误回传。
static bool s_blufi_wifi_list_pending = false;

// ─── wifi_debounce_timer_cb ──────────────────────────────────────────────────

/**
 * @brief WiFi 断线去抖定时器回调（真正"宣告网络不可用"的地方）
 *
 * 断线后 WIFI_DEBOUNCE_MS 内若 GOT_IP 恢复，本回调会被取消、永不执行；
 * 只有断线持续超过去抖时长，才在此清除 WIFI_BIT，通知 WS/MQTT 等上层断开重连。
 * 这样短暂抖动不会触发协议层 destroy/create，避免内部 SRAM 碎片累积。
 *
 * @param xTimer 定时器句柄（未使用，board 指针从模块静态变量取）
 * @note 运行在 FreeRTOS Timer 服务任务上下文，禁止阻塞
 */
static void wifi_debounce_timer_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    ESP_LOGW(TAG, "WiFi 断线持续超过 %d ms，确认掉线，通知上层断开", WIFI_DEBOUNCE_MS);
    if (s_debounce_board)
        xEventGroupClearBits(s_debounce_board->board_status, WIFI_BIT);
}

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

    // ── 先切到静态「正在重置」提示页，再动 flash ─────────────────────────────
    // 必须在 NVS 擦除 / esp_wifi_restore【之前】调用：这些 flash 写操作会禁用
    // flash cache，逐帧读 SPIFFS 的主界面 GIF 会卡在当前帧（像死机）。这里先把
    // 画面换成纯静态文字并同步刷屏，后续 cache 被禁也不影响显示。
    ui_show_unbinding();

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
 */
/**
 * @brief 按键重置一次性任务：在 8192B 大栈上执行 clear_wifi_and_restart()
 *
 * btn_task 自身栈仅 3072B，不足以承载 ui_show_unbinding() 的 LVGL 刷屏。
 * 这里用独立大栈任务跑清除+重启逻辑，跑完即 esp_restart（不返回）。
 *
 * @param pv 未使用
 */
static void btn_reset_task(void *pv)
{
    (void)pv;
    clear_wifi_and_restart(); // 内部最终 esp_restart()，不会返回
    vTaskDelete(NULL);        // 兜底：理论上不可达
}

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
                // 不能在本任务（btn_task 栈仅 3072B）直接调用 clear_wifi_and_restart()：
                // 其内部 ui_show_unbinding() 在 LVGL 就绪时会跑 lv_refr_now + DMA 刷屏
                // （大量 memcpy 像素），3072B 栈会溢出（StoreProhibited 0x1D），
                // 与 mqtt_protocol.c 云端解绑路径同坑。改用一次性 8192B 任务执行。
                // 该任务内部最终 esp_restart，跑完即销毁，不占常驻内部 SRAM 水位。
                xTaskCreatePinnedToCoreWithCaps(
                    btn_reset_task, "btn_reset",
                    8192, NULL, 5, NULL,
                    tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                vTaskDelete(NULL); // btn_task 使命完成自删（清除+重启在 btn_reset 任务里完成）
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

// ─── blufi_save_token_from_json ──────────────────────────────────────────────

/**
 * @brief 解析手机下发的 JSON 并把 device Token 写入 NVS（BluFi 自定义数据帧用）
 *
 * 迁移说明：本函数从原 Unified Provisioning 的 custom_prov_data_handler 抽取而来，
 * token 的解析与持久化逻辑保持完全一致（命名空间 "net_config"、键 "ws_token"），
 * 确保 session.c 读取端无需任何改动。区别仅在于数据来源：
 *   - 原来：BLE 配网框架的 "custom-data" 端点回调
 *   - 现在：BluFi 的 ESP_BLUFI_EVENT_RECV_CUSTOM_DATA 回调
 *
 * @param data 手机发来的原始字节（期望是 JSON 字符串 {"token":"xxx"}）
 * @param len  原始数据长度（字节）
 * @return ESP_OK 写入成功；其他为失败（失败仅记日志，不影响 WiFi 配网主流程）
 *
 * @note token 与 WiFi 凭证的到达没有严格先后，本函数与 WiFi 连接解耦：
 *       收到即独立落 NVS，避免竞态（计划「风险与注意点」第四条）。
 * @note 线程安全：在 BluFi/NimBLE 任务上下文调用，注意 NVS 并发写入。
 */
static esp_err_t blufi_save_token_from_json(const uint8_t *data, int len)
{
    // ── 步骤 1：基础数据校验（防止空指针和超大输入）─────────────────────────
    if (data == NULL || len <= 0 || len >= 2048)
    {
        ESP_LOGE(TAG, "收到无效自定义数据，长度异常: %d", len);
        return ESP_ERR_INVALID_ARG;
    }

    // ── 步骤 2：安全拷贝输入数据（添加 \0 结尾，防止字符串越界）─────────────
    char *safe_str = calloc(1, len + 1); // calloc 自动清零（包含 \0 结尾）
    if (!safe_str)
    {
        ESP_LOGE(TAG, "内存分配失败，无法处理 token 数据");
        return ESP_ERR_NO_MEM;
    }
    memcpy(safe_str, data, len);
    ESP_LOGI(TAG, "收到 BluFi 自定义数据: %s", safe_str);

    // ── 步骤 3：JSON 解析，提取 token 字段 ────────────────────────────────────
    esp_err_t result = ESP_FAIL;
    cJSON *root = cJSON_Parse(safe_str);
    if (!root)
    {
        ESP_LOGE(TAG, "JSON 解析失败，数据不是合法 JSON 格式");
        free(safe_str);
        return ESP_FAIL;
    }

    cJSON *token_item = cJSON_GetObjectItem(root, "token");
    if (!cJSON_IsString(token_item) || token_item->valuestring == NULL)
    {
        ESP_LOGE(TAG, "JSON 中未找到 'token' 字段，或 token 不是字符串类型");
        goto cleanup;
    }

    ESP_LOGI(TAG, "成功提取 Token: %.20s...", token_item->valuestring);

    // ── 步骤 4：将 Token 写入 NVS 持久化（键名与读取端 session.c 严格一致）──
    nvs_handle_t h;
    esp_err_t err = nvs_open("net_config", NVS_READWRITE, &h);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS 打开失败（net_config）: %s", esp_err_to_name(err));
        goto cleanup;
    }

    err = nvs_set_str(h, "ws_token", token_item->valuestring);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "Token 写入 NVS 失败: %s", esp_err_to_name(err));
        nvs_close(h);
        goto cleanup;
    }

    err = nvs_commit(h); // 将写入缓冲区刷入 Flash（掉电不丢失）
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS commit 失败: %s", esp_err_to_name(err));
        nvs_close(h);
        goto cleanup;
    }

    nvs_close(h);
    ESP_LOGI(TAG, "Token 已永久写入 NVS！");
    result = ESP_OK;

cleanup:
    cJSON_Delete(root);
    free(safe_str);
    return result;
}

// ─── blufi_event_callback ────────────────────────────────────────────────────

/**
 * @brief BluFi 配网事件回调（取代原 prov_event_handler）
 *
 * BluFi 把配网全过程拆成若干事件回调（手机连蓝牙、收到 SSID/密码、请求连 AP、
 * 收到自定义数据等）。本回调维护配网状态、把收到的凭证下发给 esp_wifi、
 * 落地 device token，并在适当时机向手机回报 WiFi 连接结果。
 *
 * 与原 Unified Provisioning 的对应关系：
 *   - WIFI_PROV_START      → ESP_BLUFI_EVENT_INIT_FINISH（这里启动广播）
 *   - custom-data 端点回调 → ESP_BLUFI_EVENT_RECV_CUSTOM_DATA（落 token）
 *   - WIFI_PROV_CRED_RECV  → ESP_BLUFI_EVENT_RECV_STA_SSID/PASSWD
 *   - WIFI_PROV_END        → 由 IP_EVENT_STA_GOT_IP 置 PROV_DONE_BIT（见 wifi_ip_event_handler）
 *
 * @param event BluFi 事件类型
 * @param param 事件参数（按 event 取对应联合体成员）
 * @return void
 *
 * @note 调用者：BluFi 协议栈（esp_blufi_register_callbacks 注册后自动调用）
 * @note 线程安全：在 BluFi/NimBLE 任务上下文执行，禁止阻塞；不要在此释放蓝牙
 *       自身（蓝牙释放放在 bsp_board_wifi_main 主流程，见 BUG-023 教训）。
 */
static void blufi_event_callback(esp_blufi_cb_event_t event, esp_blufi_cb_param_t *param)
{
    switch (event)
    {
    case ESP_BLUFI_EVENT_INIT_FINISH:
        // BluFi 协议栈就绪，开始 BLE 广播，手机可扫描到设备
        ESP_LOGI(TAG, "BluFi 初始化完成，开始 BLE 广播，等待手机配网...");
        s_is_provisioning = true; // 标记配网进行中，禁止断线自动重连
        esp_blufi_adv_start();
        break;

    case ESP_BLUFI_EVENT_DEINIT_FINISH:
        ESP_LOGI(TAG, "BluFi 反初始化完成");
        break;

    case ESP_BLUFI_EVENT_BLE_CONNECT:
        // 手机已连上设备蓝牙（GATT 连接建立）→ 停广播 + 初始化安全协商
        ESP_LOGI(TAG, "手机已连接蓝牙，开始安全协商");
        s_blufi_ble_connected = true;
        esp_blufi_adv_stop();
        blufi_security_init();
        break;

    case ESP_BLUFI_EVENT_BLE_DISCONNECT:
        // 手机断开蓝牙 → 释放安全上下文。是否重新广播取决于配网是否已成功：
        //   · 配网未成功就断开（手机退出/超时）→ 重新广播，等待下次连接配网
        //   · 配网已成功后断开（正常收尾）→ 主流程此刻正在释放蓝牙（NimBLE deinit），
        //     此时再调 esp_blufi_adv_start() 会撞上正在关闭的协议栈，报
        //     "error setting advertisement data; rc=30"（BLE_HS_EINVAL）。
        //   用 WIFI_BIT 是否已置位（=已拿到 IP=配网成功）来区分这两种场景。
        // 修改日期 6/22：原代码无条件重新广播，导致配网成功收尾时产生无害但刺眼的 rc=30 报错。
        s_blufi_ble_connected = false;
        blufi_security_deinit();
        if (s_blufi_board && bsp_board_check_status(s_blufi_board, WIFI_BIT, 0))
        {
            // 配网已成功，蓝牙即将被主流程释放，不再广播（避免 rc=30）
            ESP_LOGI(TAG, "配网已成功，手机断开蓝牙（正常收尾），不再广播");
        }
        else
        {
            // 配网尚未完成，重新广播等待下一次连接
            ESP_LOGW(TAG, "配网未完成，手机断开蓝牙，重新广播等待下次连接");
            esp_blufi_adv_start();
        }
        break;

    case ESP_BLUFI_EVENT_SET_WIFI_OPMODE:
        // 手机要求设备进入某种 WiFi 模式（配网时通常是 STA）
        ESP_LOGI(TAG, "BluFi 设置 WiFi 模式: %d", param->wifi_mode.op_mode);
        esp_wifi_set_mode(param->wifi_mode.op_mode);
        break;

    case ESP_BLUFI_EVENT_RECV_STA_SSID:
        // 收到家庭 WiFi 的 SSID（暂存到 s_blufi_sta_config，等密码到齐再连）
        if (param->sta_ssid.ssid_len >= sizeof(s_blufi_sta_config.sta.ssid))
        {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            ESP_LOGE(TAG, "SSID 过长，非法");
            break;
        }
        memset(s_blufi_sta_config.sta.ssid, 0, sizeof(s_blufi_sta_config.sta.ssid));
        memcpy(s_blufi_sta_config.sta.ssid, param->sta_ssid.ssid, param->sta_ssid.ssid_len);
        esp_wifi_set_config(WIFI_IF_STA, &s_blufi_sta_config);
        ESP_LOGI(TAG, "收到 WiFi SSID: %s", s_blufi_sta_config.sta.ssid);
        break;

    case ESP_BLUFI_EVENT_RECV_STA_PASSWD:
        // 收到家庭 WiFi 的密码
        if (param->sta_passwd.passwd_len >= sizeof(s_blufi_sta_config.sta.password))
        {
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            ESP_LOGE(TAG, "WiFi 密码过长，非法");
            break;
        }
        memset(s_blufi_sta_config.sta.password, 0, sizeof(s_blufi_sta_config.sta.password));
        memcpy(s_blufi_sta_config.sta.password, param->sta_passwd.passwd, param->sta_passwd.passwd_len);
        esp_wifi_set_config(WIFI_IF_STA, &s_blufi_sta_config);
        ESP_LOGI(TAG, "收到 WiFi 密码（长度 %d）", param->sta_passwd.passwd_len);
        break;

    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        // 手机下发完凭证，请求设备连接 AP
        ESP_LOGI(TAG, "BluFi 请求连接 WiFi，开始连接...");
        s_is_provisioning = false; // 解除配网保护，允许 wifi_ip_event_handler 正常重连
        esp_wifi_disconnect();     // 先断开（若之前已连过），确保触发连接回调
        esp_wifi_connect();
        break;

    case ESP_BLUFI_EVENT_REQ_DISCONNECT_FROM_AP:
        ESP_LOGI(TAG, "BluFi 请求断开 WiFi");
        esp_wifi_disconnect();
        break;

    case ESP_BLUFI_EVENT_REPORT_ERROR:
        ESP_LOGE(TAG, "BluFi 报告错误，错误码 %d", param->report_error.state);
        esp_blufi_send_error_info(param->report_error.state);
        break;

    case ESP_BLUFI_EVENT_GET_WIFI_LIST:
        // ── 手机请求设备扫描周边 WiFi 列表（小程序"获取 WiFi 列表"会用到）─────
        // 说明：EspBlufi App 手动输入 SSID，不触发此事件，故对 App 零影响；微信小程序
        //       等客户端通常先让设备扫描 AP 列表返回供用户选择，不实现会一直卡"获取中..."。
        // 非阻塞：此处只启动扫描后立即返回，不堵塞 NimBLE 回调线程；扫描完成由
        //         wifi_ip_event_handler 的 WIFI_EVENT_SCAN_DONE 分支取结果并回传。
        // 修改日期 6/22：新增，解决小程序配网"获取 WiFi 列表"一直转圈的问题。
        ESP_LOGI(TAG, "BluFi 请求扫描 WiFi 列表，启动非阻塞扫描...");
        s_blufi_wifi_list_pending = true; // 标记：本次扫描完成后需回传给手机
        {
            wifi_scan_config_t scan_config = {0}; // 全 0：扫描所有 SSID/全信道
            esp_err_t scan_ret = esp_wifi_scan_start(&scan_config, false /* 非阻塞 */);
            if (scan_ret != ESP_OK)
            {
                ESP_LOGE(TAG, "WiFi 扫描启动失败: %s", esp_err_to_name(scan_ret));
                s_blufi_wifi_list_pending = false;
                esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            }
        }
        break;

    case ESP_BLUFI_EVENT_GET_WIFI_STATUS:
    {
        // 手机查询当前 WiFi 状态，回报连接结果（手机据此显示成功/失败）
        wifi_mode_t mode;
        esp_wifi_get_mode(&mode);
        if (bsp_board_check_status(s_blufi_board, WIFI_BIT, 0))
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
        else
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);
        break;
    }

    case ESP_BLUFI_EVENT_RECV_SLAVE_DISCONNECT_BLE:
        ESP_LOGI(TAG, "BluFi 请求关闭 GATT 连接");
        esp_blufi_disconnect();
        break;

    case ESP_BLUFI_EVENT_RECV_CUSTOM_DATA:
        // 手机通过自定义数据帧下发 device token（取代原 custom-data 端点）
        ESP_LOGI(TAG, "收到 BluFi 自定义数据，长度 %" PRIu32, param->custom_data.data_len);
        blufi_save_token_from_json(param->custom_data.data, (int)param->custom_data.data_len);
        break;

    default:
        // 其余事件（WiFi 列表扫描、SoftAP 配置、证书等）本项目不使用，忽略
        break;
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
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_SCAN_DONE)
    {
        // ── 非阻塞 WiFi 扫描完成：取结果回传给手机（BluFi GET_WIFI_LIST 的下半段）──
        // 说明：BluFi 回调 ESP_BLUFI_EVENT_GET_WIFI_LIST 里只非阻塞启动了扫描并立即返回
        //       （不阻塞 NimBLE 回调线程）；扫描真正完成后由本事件取 AP 列表并发回手机。
        //       仅在配网请求扫描时(s_blufi_wifi_list_pending)才回传，避免误把其它扫描结果发出。
        // 修改日期 6/22：新增，配合非阻塞 GET_WIFI_LIST，解决小程序"获取 WiFi 列表"转圈。
        if (!s_blufi_wifi_list_pending)
            return;                          // 非配网触发的扫描，忽略
        s_blufi_wifi_list_pending = false;

        uint16_t ap_count = 0;
        esp_wifi_scan_get_ap_num(&ap_count);
        if (ap_count == 0)
        {
            ESP_LOGW(TAG, "未扫描到任何 WiFi，回传空列表");
            esp_blufi_send_wifi_list(0, NULL);
            return;
        }

        wifi_ap_record_t *ap_list = malloc(sizeof(wifi_ap_record_t) * ap_count);
        if (ap_list == NULL)
        {
            ESP_LOGE(TAG, "WiFi 列表内存分配失败");
            esp_wifi_clear_ap_list(); // 释放驱动内部缓存，避免泄漏
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            return;
        }
        esp_wifi_scan_get_ap_records(&ap_count, ap_list);

        esp_blufi_ap_record_t *blufi_list = malloc(sizeof(esp_blufi_ap_record_t) * ap_count);
        if (blufi_list == NULL)
        {
            ESP_LOGE(TAG, "BluFi 列表内存分配失败");
            free(ap_list);
            esp_blufi_send_error_info(ESP_BLUFI_DATA_FORMAT_ERROR);
            return;
        }
        for (int i = 0; i < ap_count; i++)
        {
            blufi_list[i].rssi = ap_list[i].rssi;
            memcpy(blufi_list[i].ssid, ap_list[i].ssid, sizeof(ap_list[i].ssid));
        }

        esp_blufi_send_wifi_list(ap_count, blufi_list);
        ESP_LOGI(TAG, "已回传 %u 个 WiFi 给手机", ap_count);
        free(ap_list);
        free(blufi_list);
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

        // ── 断线去抖：不立即清 WIFI_BIT ───────────────────────────────────────
        // WiFi 几百毫秒的抖动通常会自愈。若每次抖动都立即清 WIFI_BIT，会连锁触发
        // WS/MQTT 销毁重建（destroy/create），反复申请释放内部 SRAM 小结构 → 碎片化，
        // 稳定后内存"回不到原值"。这里改为启动 WIFI_DEBOUNCE_MS 的 one-shot 定时器，
        // 持续掉线超过该时长才真正清 WIFI_BIT；期间 GOT_IP 恢复则取消定时器（见下）。
        s_debounce_board = bsp_board; // 供定时器回调使用
        if (s_wifi_debounce_timer != NULL)
        {
            // 已在运行则重置计时；未运行则启动。xTimerStart 对已运行定时器等价于复位。
            // 注意：处于 ISR 之外的事件任务上下文，使用普通（非 FromISR）API。
            xTimerStart(s_wifi_debounce_timer, 0);
        }
        else if (bsp_board)
        {
            // 定时器尚未创建（理论上 wifi_main 已创建，此为兜底）：直接清位，保持旧行为
            xEventGroupClearBits(bsp_board->board_status, WIFI_BIT);
        }
    }
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP)
    {
        // ── 成功获取 IP 地址 ──────────────────────────────────────────────────
        ip_event_got_ip_t *event = (ip_event_got_ip_t *)event_data;
        ESP_LOGI(TAG, "成功获取 IP: " IPSTR, IP2STR(&event->ip_info.ip));

        // 连接成功，重置重连计数（下次断线时从 0 开始重新计数）
        s_retry_num = 0;

        // ── 去抖：网络已恢复，取消"宣告掉线"定时器 ───────────────────────────
        // 若本次断线在去抖窗口内恢复，定时器回调不会执行，WIFI_BIT 从未被清，
        // WS/MQTT 也就不会经历销毁重建 → 从源头避免碎片。
        if (s_wifi_debounce_timer != NULL)
            xTimerStop(s_wifi_debounce_timer, 0);

        // 置位 WIFI_BIT，解除 bsp_board_wifi_main() 末尾的 xEventGroupWaitBits 阻塞
        if (bsp_board)
            xEventGroupSetBits(bsp_board->board_status, WIFI_BIT);

        // ── BluFi 配网期：拿到 IP 即视为配网成功 ─────────────────────────────
        // BluFi 没有 Unified Provisioning 的 WIFI_PROV_END 事件，改由「拿到 IP」
        // 作为配网结束信号：① 向手机回报连接成功（手机 App 显示配网成功）；
        // ② 置 PROV_DONE_BIT 解除 bsp_board_wifi_main 的配网等待，触发蓝牙释放。
        if (s_blufi_ble_connected)
        {
            wifi_mode_t mode;
            esp_wifi_get_mode(&mode);
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
        }
        if (bsp_board)
            xEventGroupSetBits(bsp_board->board_status, PROV_DONE_BIT);
    }
}

// ─── bsp_wifi_get_rssi ───────────────────────────────────────────────────────

/**
 * @brief 获取当前 STA 连接的 WiFi 信号强度 RSSI
 *
 * 调用 esp_wifi_sta_get_ap_info() 读取关联 AP 的信号强度。
 *
 * @return RSSI 值（单位 dBm，负数，值越大信号越好）；未连接或读取失败时返回 0
 *
 * @note 调用者：UI 状态栏定时器（每数秒拉取一次刷新显示）
 * @note 该函数仅读取已缓存的 AP 信息，开销极小，可频繁调用
 */
int bsp_wifi_get_rssi(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
    {
        return ap_info.rssi;
    }
    return 0; // 未连接或读取失败
}

// ─── BluFi 回调注册表 + 已配网判断 ───────────────────────────────────────────

/// @brief BluFi 回调集合：事件处理 + 安全协商（DH/AES/CRC，来自 blufi_security.c）
static esp_blufi_callbacks_t s_blufi_callbacks = {
    .event_cb = blufi_event_callback,
    .negotiate_data_handler = blufi_dh_negotiate_data_handler,
    .encrypt_func = blufi_aes_encrypt,
    .decrypt_func = blufi_aes_decrypt,
    .checksum_func = blufi_crc_checksum,
};

/**
 * @brief 判断设备是否已配网（NVS 中是否存有可用的 WiFi SSID）
 *
 * BluFi 不像 wifi_prov_mgr 提供 is_provisioned() 接口，这里改为直接读取
 * esp_wifi 持久化在 NVS 的 STA 配置：只要 ssid 非空即视为已配网。
 *
 * @return true 已配网（NVS 有 SSID）；false 未配网（需走 BluFi 配网流程）
 * @note 必须在 esp_wifi_init() 之后调用（否则读不到 NVS 配置）。
 */
static bool wifi_is_provisioned(void)
{
    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK)
        return false;
    return cfg.sta.ssid[0] != '\0'; // SSID 非空 = 已存过凭证
}

// ─── bsp_board_wifi_main ─────────────────────────────────────────────────────

/**
 * @brief WiFi 完整初始化入口（阻塞直至网络就绪或彻底失败）
 *
 * 完整流程（已从 Unified Provisioning 迁移到 BluFi）：
 *   1. 前置检查（NVS_BIT 必须已置位）
 *   2. 初始化 TCP/IP 协议栈和默认事件循环
 *   3. 注册 WiFi/IP 两类事件监听（配网事件改由 BluFi 回调处理）
 *   4. 初始化 WiFi 驱动并设为 STA 模式
 *   5. 启动按键监控任务 + 打印设备标识
 *   6a. NVS 无凭证 → 启动 BluFi（BLE）配网，等待手机配网（120s 超时），结束后释放蓝牙
 *   6b. NVS 有凭证 → 直接 esp_wifi_start() 连接（不开蓝牙）
 *   7. 最终阻塞等待 WIFI_BIT 或 WIFI_FAIL_BIT
 *   8. WIFI_FAIL_BIT → 等待 30s 后重启
 *
 * @param bsp_board BSP 实例指针（通过 board_status 管理状态位）
 * @return void（阻塞直到网络就绪；WIFI_FAIL_BIT 时触发重启不返回）
 *
 * @note 调用者：application.c → application_init()（步骤 5），签名未变。
 * @note 前置条件：NVS_BIT 已置位（bsp_board_nvs_init() 已完成）
 * @note 配网安全：BluFi 用 DH 密钥协商替代原固定 PoP 密码 "abcd1234"。
 */
void bsp_board_wifi_main(bsp_board_t *bsp_board)
{
    // ── 前置条件检查：NVS 必须已初始化（WiFi 凭证存在 NVS 中）─────────────
    if (!bsp_board_check_status(bsp_board, NVS_BIT, 0))
    {
        ESP_LOGE(TAG, "NVS 未初始化，无法启动 WiFi");
        return;
    }
    s_blufi_board = bsp_board; // 供 BluFi 回调访问状态事件组

    // ── 步骤 1：初始化 TCP/IP 协议栈 ─────────────────────────────────────────
    ESP_ERROR_CHECK(esp_netif_init());

    // ── 步骤 2：创建默认事件循环（防止重复创建导致 panic）───────────────────
    esp_err_t err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) // INVALID_STATE = 已存在，可忽略
        ESP_ERROR_CHECK(err);

    // 创建默认 WiFi STA 网络接口（分配 IP、DNS、路由等网络信息）
    esp_netif_create_default_wifi_sta();

    // ── 步骤 3：注册 WiFi / IP 事件监听（配网事件由 BluFi 回调处理，不再注册 WIFI_PROV_EVENT）──
    esp_event_handler_instance_t instance_any_id, instance_got_ip;

    // 监听 WiFi 事件（STA_START 和 STA_DISCONNECTED）
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        WIFI_EVENT, ESP_EVENT_ANY_ID,
        &wifi_ip_event_handler, bsp_board, &instance_any_id));

    // 监听 IP 获取事件（STA_GOT_IP）→ 置位 WIFI_BIT + 配网期置 PROV_DONE_BIT
    ESP_ERROR_CHECK(esp_event_handler_instance_register(
        IP_EVENT, IP_EVENT_STA_GOT_IP,
        &wifi_ip_event_handler, bsp_board, &instance_got_ip));

    // ── 步骤 3.5：创建 WiFi 断线去抖定时器（one-shot，不自动重载）───────────
    if (s_wifi_debounce_timer == NULL)
    {
        s_wifi_debounce_timer = xTimerCreate(
            "wifi_debounce",
            pdMS_TO_TICKS(WIFI_DEBOUNCE_MS),
            pdFALSE, // one-shot：触发一次后停止，不自动重载
            NULL,
            wifi_debounce_timer_cb);
        if (s_wifi_debounce_timer == NULL)
            ESP_LOGE(TAG, "WiFi 去抖定时器创建失败，将退化为断线立即通知上层");
    }

    // ── 步骤 4：初始化 WiFi 驱动并设为 STA 模式 ──────────────────────────────
    // BluFi 配网与已配网直连都基于 STA：配网时手机把家庭 WiFi 凭证下发到 STA。
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    // ── 步骤 5：启动按键监控任务（GPIO0 长按 3s 触发 WiFi 重置）─────────────
    // button_monitor_task 会调用 nvs_erase_key/nvs_set_str/nvs_commit（Flash 操作），
    // Flash 操作占用 SPI 总线期间 CPU 需访问任务栈，栈必须在内部 SRAM，否则 WDT 复位。
    xTaskCreatePinnedToCoreWithCaps(
        button_monitor_task, "btn_task",
        3072, NULL, 5, NULL,
        0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "[内存] btn_task 创建后 → 内部SRAM剩余: %u B，PSRAM剩余: %u B",
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
             (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    // ── 步骤 5.5：生成 MAC 派生蓝牙名 + 打印设备唯一标识 ─────────────────────
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_STA, mac);
    char service_name[18];
    snprintf(service_name, sizeof(service_name),
             "EchoPals-%02X%02X%02X", mac[3], mac[4], mac[5]);
    ESP_LOGI(TAG, "🆔 DeviceID: %02X%02X%02X | MAC: %02X:%02X:%02X:%02X:%02X:%02X",
             mac[3], mac[4], mac[5], mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    // ── 步骤 6：检查是否已配网（读 NVS STA 配置的 SSID 是否非空）─────────────
    if (!wifi_is_provisioned())
    {
        // ════ 未配网分支：启动 BluFi（BLE）配网，等待手机配网 ════════════════
        ESP_LOGI(TAG, "设备未配网，启动 BluFi 配网...");
        ESP_LOGI(TAG, "📱 配网入口 → 蓝牙名: %s（用 EspBlufi App 或小程序扫描）", service_name);

        // 有屏幕时：生成二维码显示在 LCD 上，方便用户扫码定位设备
        // 无屏幕时（裸板）：仅靠蓝牙广播，串口打印设备 ID 供调试核对
#if CONFIG_BSP_HAS_DISPLAY
        ui_show_qrcode(service_name);
#endif

        // 启动 WiFi 驱动（配网期需要 STA 就绪以便后续 connect）
        ESP_ERROR_CHECK(esp_wifi_start());

        // 拉起 BT 控制器（NimBLE 下由该接口初始化 BLE 控制器）
#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
        esp_err_t bt_err = esp_blufi_controller_init();
        if (bt_err != ESP_OK)
        {
            ESP_LOGE(TAG, "BT 控制器初始化失败: %s，跳过配网直接重启", esp_err_to_name(bt_err));
            esp_restart();
        }
#endif
        // 注册 BluFi 回调 + 启动 NimBLE 主机（INIT_FINISH 回调里会 adv_start）
        esp_err_t blufi_err = esp_blufi_host_and_cb_init(&s_blufi_callbacks);
        if (blufi_err != ESP_OK)
        {
            ESP_LOGE(TAG, "BluFi 初始化失败: %s，重启", esp_err_to_name(blufi_err));
            esp_restart();
        }

        // 设置 MAC 派生蓝牙名（覆盖 blufi_init.c 的兜底默认名）
#if CONFIG_BT_NIMBLE_ENABLED
        ble_svc_gap_device_name_set(service_name);
#endif
        ESP_LOGI(TAG, "BluFi 配网就绪 → 蓝牙名: %s（手机用 EspBlufi / 微信小程序配网）", service_name);

        // 阻塞等待配网完成（PROV_DONE_BIT 由 wifi_ip_event_handler 在 GOT_IP 时置位）
        // 超时 120 秒：防止设备永远卡在配网模式
        EventBits_t wait_bits = xEventGroupWaitBits(
            bsp_board->board_status, PROV_DONE_BIT,
            pdFALSE, pdFALSE, pdMS_TO_TICKS(120000)); // 120s 超时

        if (!(wait_bits & PROV_DONE_BIT))
        {
            ESP_LOGE(TAG, "配网超时（120 秒），强制重启设备");
            s_is_provisioning = false;
            esp_restart();
        }

        // 配网成功后撤掉二维码（有屏幕时）
#if CONFIG_BSP_HAS_DISPLAY
        ui_hide_qrcode();
#endif

        // ── 配网成功：立即释放蓝牙（BluFi profile + NimBLE 主机 + 控制器）─────
        // 用户要求"无论 App 还是小程序配网成功，蓝牙都释放"，与原 FREE_BTDM 思路一致。
        // 释放放在主流程（非 BluFi 回调内），避免在回调里 deinit 自身（见 BUG-023）。
        ESP_LOGI(TAG, "[内存] 释放蓝牙前 → 内部SRAM: %u B，PSRAM: %u B",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
        esp_blufi_host_deinit();
#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
        esp_blufi_controller_deinit();
#endif
        ESP_LOGI(TAG, "[内存] 释放蓝牙后 → 内部SRAM: %u B，PSRAM: %u B",
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
                 (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    }
    else
    {
        // ════ 已配网分支：直接 STA 连接（不开蓝牙，省内存）══════════════════
        ESP_LOGI(TAG, "设备已配网，直接连接 WiFi...");
        // 启动 WiFi 驱动（触发 WIFI_EVENT_STA_START → esp_wifi_connect()）
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
