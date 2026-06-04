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
#include "bsp/bsp_board.h"
#include "bsp/bsp_ota.h"
// 舵机控制直接走 bsp/bsp_board.h 的 bsp_servo_move_smooth（绝对角度定位），无需 servo_manager.h
#include "auth.h"
#include "session/session.h" // session_debug_kill_ws() — MQTT 远程伪造 WS 断连测试
#include "object.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *MQTT_TAG = "MQTT"; ///< 日志 TAG

// ─── 音量映射：app 下发 0~100 → 硬件音量 0~APP_VOLUME_HW_MAX ────────────────
// app 端永远以 0~100 表达音量；硬件实际可用上限不确定（受功放/喇叭/听感影响），
// 故在此处做一层线性缩放，只需调整 APP_VOLUME_HW_MAX 一个宏即可整体改变最大音量。
//   value=0   → 硬件 0（静音）
//   value=100 → 硬件 APP_VOLUME_HW_MAX
// 当前设为 100 表示不缩放（与历史行为一致），按需下调。
#define APP_VOLUME_HW_MAX 65 ///< app 满音量(100)对应的硬件音量上限

// ─── MQTT 凭证（运行时从 NVS 加载，回退到编译期默认值）────────────────────
#define MQTT_DEFAULT_URI "mqtt://122.224.191.2:1883" ///< 默认 Broker 地址（测试环境）
#define MQTT_DEFAULT_USER "xtc"                      ///< 默认 MQTT 用户名
#define MQTT_DEFAULT_PASS "Xtc@12345"                ///< 默认 MQTT 密码

// 运行时凭证缓冲区（由 mqtt_credentials_load 从 NVS 填充，否则保持默认值）
static char s_mqtt_uri[128] = MQTT_DEFAULT_URI;
static char s_mqtt_user[64] = MQTT_DEFAULT_USER;
static char s_mqtt_pass[64] = MQTT_DEFAULT_PASS;

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

// MQTT 重连退避状态
static volatile TaskHandle_t s_mqtt_reconnect_handle = NULL;
static int s_mqtt_reconnect_attempts = 0;
#define MQTT_RECONNECT_BASE_DELAY_MS 5000 // MQTT 基础退避 5 秒（比 WebSocket 慢，避免同时竞争）

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
    char display[64]; ///< 显示文字（如 "云炎"、"start Echo"），用于自动检测语言
    char pinyin[64];  ///< 命令词（中文拼音 "yun yan" 或英文单词 "start echo"）
} ww_update_params_t;

/**
 * @brief 异步执行设备解绑的后台任务
 *
 * 从 MQTT 事件回调中独立出来，避免在回调栈上执行 NVS 写 + MQTT publish + esp_restart。
 * 延迟 500ms 确保 MQTT 回调已正常返回后再执行解绑流程。
 */
static void async_unbind_task(void *pvParameters)
{
    // 等待 MQTT 回调返回，再执行 NVS 写和 MQTT publish，避免回调栈上的死锁风险
    vTaskDelay(pdMS_TO_TICKS(500));
    ESP_LOGW(MQTT_TAG, "执行云端解绑：清除凭证并重启...");
    clear_wifi_and_restart(); // 内部调用 esp_restart()，不会返回
    vTaskDelete(NULL);        // 不会执行到此处
}

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
            // 电量百分比：复用 bsp_battery 模块（IIR 滤波 + 锂电放电曲线），未初始化时返回 0
            cJSON_AddNumberToObject(root, "battery", bsp_battery_get_percent());

            // if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK)
            //     cJSON_AddNumberToObject(root, "wifi_signal", ap_info.rssi);
            // else
            //     cJSON_AddNumberToObject(root, "wifi_signal", -100);

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

// ─── MQTT 重连退避任务 ─────────────────────────────────────────────────────
/**
 * @brief MQTT 指数退避重连任务
 * 停止 MQTT 客户端的自动重连，改为手动控制退避策略，
 * 避免 MQTT 重连与 WebSocket 重连同时竞争内部 SRAM。
 *
 * @param arg 未使用
 */
static void mqtt_reconnect_task(void *arg)
{
    // 指数退避：5s → 10s → 20s → 40s → 60s（上限）
    s_mqtt_reconnect_attempts++;
    int shift = s_mqtt_reconnect_attempts - 1;
    if (shift > 4)
        shift = 4;
    int delay_ms = MQTT_RECONNECT_BASE_DELAY_MS * (1 << shift);
    if (delay_ms > 60000)
        delay_ms = 60000;

    ESP_LOGW(MQTT_TAG, "MQTT 第 %d 次重连，%d 秒后执行...", s_mqtt_reconnect_attempts, delay_ms / 1000);
    vTaskDelay(pdMS_TO_TICKS(delay_ms));

    // ★ 检查服务器可达性：不可达时跳过本次重连，避免浪费内部 SRAM
    if (!auth_is_server_reachable())
    {
        ESP_LOGW(MQTT_TAG, "服务器不可达，跳过 MQTT 重连");
        goto exit;
    }

    // 内部 SRAM 不足时跳过
    size_t internal_free = heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    if (internal_free < 8192)
    {
        ESP_LOGW(MQTT_TAG, "[MEM] 内部 SRAM 仅剩 %d B，跳过 MQTT 重连", (int)internal_free);
        goto exit;
    }

    ESP_LOGI(MQTT_TAG, "正在重连 MQTT...");
    esp_mqtt_client_start(s_mqtt_client);
    ESP_LOGW(MQTT_TAG, "MQTT 重连尝试完成");

exit:
    s_mqtt_reconnect_handle = NULL;
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
        s_mqtt_reconnect_attempts = 0; // 连接成功，重置退避计数

        char dev_id[16];
        get_short_device_id(dev_id, sizeof(dev_id));
        char topic[64];
        snprintf(topic, sizeof(topic), "echopal/device/%s/wake-word", dev_id);

        esp_mqtt_client_subscribe(client, topic, 0);
        ESP_LOGI(MQTT_TAG, "正在监听此主题: %s", topic);

        // 订阅设备指令主题（云端解绑等控制命令）
        snprintf(topic, sizeof(topic), "echopal/device/%s/command", dev_id);
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
                PRINT_MEM_INFO(MQTT_TAG, "json_data calloc 失败");
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
                    // ★ 启动时服务端常下发 retained 消息，内容往往与当前 NVS 完全相同。
                    //   若无差异直接跳过，可避免 wake_word_update 带来的副作用：
                    //     - FST 重建 ~300ms
                    //     - NVS 写 Flash ~50-100ms
                    //     - AFE reset_buffer 把 AGC/NS 的自适应状态清零（冷启动首次唤醒变迟钝的主因）
                    //   收益：上电即可用，首次唤醒不再踩"更新窗口 + AGC 冷启动"双重坑。
                    if (wake_word_is_same(display_item->valuestring, pinyin_item->valuestring))
                    {
                        ESP_LOGI(MQTT_TAG, "唤醒词无变化 (wakeWord=%s)，跳过更新以保留 AFE 自适应状态",
                                 display_item->valuestring);
                        cJSON_Delete(root);
                        break;
                    }

                    ESP_LOGW(MQTT_TAG, "准备更新唤醒词: wakeWord=%s pinyin=%s",
                             display_item->valuestring, pinyin_item->valuestring);

                    ww_update_params_t *params = malloc(sizeof(ww_update_params_t));
                    if (!params)
                    {
                        PRINT_MEM_INFO(MQTT_TAG, "ww_update_params_t malloc 失败");
                    }
                    if (params)
                    {
                        strncpy(params->display, display_item->valuestring, sizeof(params->display) - 1);
                        params->display[sizeof(params->display) - 1] = '\0';
                        strncpy(params->pinyin, pinyin_item->valuestring, sizeof(params->pinyin) - 1);
                        params->pinyin[sizeof(params->pinyin) - 1] = '\0';

                        // ★ 必须使用内部 SRAM 栈！
                        // 此任务调用 wake_word_update → nvs_write_str → SPI Flash 写操作。
                        // ESP32-S3 flash 操作期间临时禁用 Data Cache，SPIRAM 通过同一 Cache 访问，
                        // 若任务栈在 SPIRAM，cache_utils.c 内部断言失败 → panic 重启。
                        // 使用 MALLOC_CAP_INTERNAL 确保栈始终可访问。
                        BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(async_update_wakeword_task, "async_ww_update",
                                                                         4096, params, 4, NULL,
                                                                         tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                        if (ret != pdPASS)
                        {
                            ESP_LOGE(MQTT_TAG, "内存不足，无法创建唤醒词更新任务！");
                            PRINT_MEM_INFO(MQTT_TAG, "async_ww_update 4KB INTERNAL 栈分配失败");
                            free(params);
                        }
                        else
                        {
                            static bool s_printed_async_ww = false;
                            if (!s_printed_async_ww)
                            {
                                PRINT_MEM_INFO(MQTT_TAG, "async_ww_update 4KB INTERNAL 栈首次分配后");
                                s_printed_async_ww = true;
                            }
                        }
                    }
                }
                cJSON_Delete(root);
            }
        }
        else if (event->topic_len > 0 && strstr(event->topic, "command") != NULL)
        {
            ESP_LOGI(MQTT_TAG, "拦截到设备指令！");

            char *json_data = calloc(1, event->data_len + 1);
            if (!json_data)
            {
                ESP_LOGE(MQTT_TAG, "内存不足，无法分配 command JSON 缓冲区");
                break;
            }
            memcpy(json_data, event->data, event->data_len);

            cJSON *root = cJSON_Parse(json_data);
            free(json_data);
            if (root)
            {
                cJSON *type_item = cJSON_GetObjectItem(root, "type");
                if (cJSON_IsString(type_item) && strcmp(type_item->valuestring, "unbind") == 0)
                {
                    ESP_LOGW(MQTT_TAG, "收到云端解绑指令，启动异步解绑任务...");
                    // ★ 必须用异步任务：clear_wifi_and_restart() 内部做 MQTT publish + NVS 写 + esp_restart，
                    //   不能在 MQTT 事件回调中直接执行，否则会与 MQTT 内部锁死锁
                    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
                        async_unbind_task, "async_unbind",
                        3072, NULL, 5, NULL,
                        tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
                    if (ret != pdPASS)
                        ESP_LOGE(MQTT_TAG, "内存不足，无法创建解绑任务！");
                }
                else if (cJSON_IsString(type_item) && strcmp(type_item->valuestring, "ota") == 0)
                {
                    // ★ OTA 升级指令处理
                    // JSON 格式: {"type":"ota","url":"http://.../firmware.bin","version":"1.0.1"}
                    // bsp_ota_trigger 内部创建独立异步任务，不阻塞 MQTT 事件循环
                    cJSON *url_item = cJSON_GetObjectItem(root, "url");
                    cJSON *ver_item = cJSON_GetObjectItem(root, "version");
                    if (cJSON_IsString(url_item) && url_item->valuestring)
                    {
                        const char *ver = (cJSON_IsString(ver_item) && ver_item->valuestring)
                                              ? ver_item->valuestring
                                              : "unknown";
                        ESP_LOGW(MQTT_TAG, "收到 OTA 指令: url=%s version=%s",
                                 url_item->valuestring, ver);
                        esp_err_t ota_err = bsp_ota_trigger(url_item->valuestring, ver);
                        if (ota_err != ESP_OK)
                        {
                            ESP_LOGE(MQTT_TAG, "OTA 触发失败: %s", esp_err_to_name(ota_err));
                        }
                    }
                    else
                    {
                        ESP_LOGE(MQTT_TAG, "OTA 指令缺少 url 字段");
                    }
                }
                else if (cJSON_IsString(type_item) && strcmp(type_item->valuestring, "volume") == 0)
                {
                    // ★ 音量调节指令处理
                    // JSON 格式: {"type":"volume","value":80}（value 为 0~100 整数）
                    // 设置音量是轻量寄存器写 + NVS，无需异步任务（与 unbind/ota 的重操作不同）
                    cJSON *value_item = cJSON_GetObjectItem(root, "value");
                    if (cJSON_IsNumber(value_item))
                    {
                        int vol = value_item->valueint;
                        // 先把 app 的 0~100 钳位，避免越界后映射出负值/超额
                        if (vol < 0)
                            vol = 0;
                        if (vol > 100)
                            vol = 100;
                        // 线性映射到硬件音量 0~APP_VOLUME_HW_MAX（+50 用于四舍五入）
                        int hw_vol = (vol * APP_VOLUME_HW_MAX + 50) / 100;
                        ESP_LOGW(MQTT_TAG, "收到音量指令: value=%d → 硬件音量=%d", vol, hw_vol);
                        // bsp_board_codec_set_volume 内部自动钳位 0~100 并持久化到 NVS
                        esp_err_t vol_err = bsp_board_codec_set_volume(hw_vol);
                        if (vol_err != ESP_OK)
                        {
                            ESP_LOGE(MQTT_TAG, "设置音量失败: %s", esp_err_to_name(vol_err));
                        }
                    }
                    else
                    {
                        ESP_LOGE(MQTT_TAG, "音量指令缺少有效的 value 数字字段");
                    }
                }
                else if (cJSON_IsString(type_item) && strcmp(type_item->valuestring, "ws_kill") == 0)
                {
                    // ★【调试】伪造 WS 断连指令，用于测试断链重连逻辑
                    // JSON 格式: {"type":"ws_kill"}
                    // session_debug_kill_ws 内部另起异步任务执行 close（不能在 MQTT 回调里直接断），
                    // 等效服务端 FIN，触发 PROTOCOL_EVENT_DISCONNECTED → 退避重连。
                    ESP_LOGW(MQTT_TAG, "收到 ws_kill 调试指令，触发主动断连测试");
                    session_debug_kill_ws();
                }
                else if (cJSON_IsString(type_item) && strcmp(type_item->valuestring, "servo") == 0)
                {
                    // ★ 舵机控制指令处理
                    // JSON 格式: {"type":"servo","servo":"head","angle":90}
                    //   servo: 舵机名（字符串，固定三选一，与前端约定）
                    //          "head"      → CH_HEAD  头部
                    //          "left_arm"  → CH_L_ARM 左臂
                    //          "right_arm" → CH_R_ARM 右臂
                    //   angle: 目标角度（0~180，超出由 bsp_servo 内部软限位裁剪）
                    // 速度写死为 SERVO_SPEED_MID（15ms/度），JSON 不携带 speed、不携带 direction。
                    // servo_manager_submit_angle 为非阻塞入队操作（与 volume 同属轻量指令），
                    // 故可直接在 MQTT 事件回调中调用，无需像 unbind/ota 那样另起异步任务。
                    cJSON *servo_item = cJSON_GetObjectItem(root, "servo");
                    cJSON *offset_item = cJSON_GetObjectItem(root, "offset");
                    if (cJSON_IsString(servo_item) && servo_item->valuestring && cJSON_IsNumber(offset_item))
                    {
                        const char *servo_name = servo_item->valuestring;
                        float offset = (float)offset_item->valuedouble;

                        // 舵机名 → 通道宏映射（固定协议，与前端约定一致）
                        int channel = -1;
                        if (strcmp(servo_name, "head") == 0)
                            channel = CH_HEAD;
                        else if (strcmp(servo_name, "left_arm") == 0)
                            channel = CH_L_ARM;
                        else if (strcmp(servo_name, "right_arm") == 0)
                            channel = CH_R_ARM;

                        if (channel < 0)
                        {
                            ESP_LOGE(MQTT_TAG, "舵机指令 servo=\"%s\" 未知（仅支持 head/left_arm/right_arm）", servo_name);
                        }
                        else
                        {
                            ESP_LOGW(MQTT_TAG, "收到舵机指令: servo=%s(ch=%d) angle=%.1f", servo_name, channel, offset);
                            float absolute_angle = 90.0f + offset; // 转换：0→90, +30→120, -30→60
                            // 直接调底层 bsp_servo_move_smooth：按绝对角度定位，自带软限位/平滑/去抖，停位即止。
                            // 不经 servo_manager_submit_angle —— 其内部"拆幅度+方向再回中"逻辑会导致到位后回弹、
                            // 且把绝对角度塞进 amplitude enum 会造成角度大小错乱。
                            // 注意：此函数内部含 vTaskDelay（平滑插值，最长约 1.3s），会阻塞 MQTT 回调线程；
                            //       MQTT 指令频率低，单条可接受，若后续需连发再改异步。
                            bsp_servo_move_smooth((uint8_t)channel, absolute_angle, SERVO_SPEED_MID);
                        }
                    }
                    else
                    {
                        ESP_LOGE(MQTT_TAG, "舵机指令缺少有效的 servo(字符串)/angle(数字) 字段");
                    }
                }
                else
                {
                    ESP_LOGW(MQTT_TAG, "未知指令类型: %s", cJSON_IsString(type_item) ? type_item->valuestring : "null");
                }
                cJSON_Delete(root);
            }
        }
        break;

    case MQTT_EVENT_DISCONNECTED:
    {
        ESP_LOGW(MQTT_TAG, "MQTT 已断开，启动退避重连...");
        s_mqtt_connected = false;

        // ★ P3：停止 MQTT 默认自动重连，改为手动指数退避
        esp_mqtt_client_stop(s_mqtt_client);

        // 创建退避重连任务（如果尚未在运行）
        if (s_mqtt_reconnect_handle == NULL)
        {
            xTaskCreatePinnedToCoreWithCaps(mqtt_reconnect_task, "mqtt_reconn",
                                            3072, NULL, 3,
                                            (TaskHandle_t *)&s_mqtt_reconnect_handle,
                                            tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
            static bool s_printed_mqtt_reconn = false;
            if (!s_printed_mqtt_reconn)
            {
                PRINT_MEM_INFO(MQTT_TAG, "mqtt_reconn 3KB SPIRAM 栈首次分配后");
                s_printed_mqtt_reconn = true;
            }
        }
        break;
    }

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
    /* 心跳任务栈分配在 SPIRAM，节省内部 SRAM */
    xTaskCreatePinnedToCoreWithCaps(heartbeat_task, "heartbeat_task", 4096, NULL, 4, NULL,
                                    tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
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