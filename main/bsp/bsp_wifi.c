#include "bsp_board.h"
#include "protocol/mqtt_protocol.h"
#include "protocol/auth.h"
#include "esp_heap_caps.h"
#include "freertos/timers.h" /* WiFi 断线去抖软件定时器 */
#include "ui/ui_port.h"      /* ui_show_unbinding(): 解绑前显示静态提示页，避免 GIF 卡冻帧 */
#include "ui/reminder.h"     /* reminder_on_offline_mode(): 进离线时停 SNTP + 定时天气拉取 */
#include "esp_timer.h"       /* [DIAG] esp_timer_get_time()：微秒级时间戳，定位扫描各阶段耗时 */
#include "esp_netif.h"       /* [DNS诊断] esp_netif_get_dns_info()：拿到 IP 后打印设备实际 DNS 服务器 */
#include "udp_logger.h"      /* 纯电池调试用：拿到 IP 后把日志同时广播到 UDP，见 GOT_IP 分支 */
#include "object.h"          /* PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM：栈+堆打印 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* ── BluFi 配网相关（替代原 Unified Provisioning）─────────────────────────── */
#include "esp_blufi_api.h"        /* BluFi 事件枚举、回调结构、send 接口 */
#include "esp_blufi.h"            /* esp_blufi_adv_start/stop、profile 等 */
#include "blufi/blufi_security.h" /* 工程内拷贝：安全协商 + NimBLE 主机管理 */
#include "esp_mac.h"              /* esp_read_mac（蓝牙名派生）*/
#if CONFIG_BT_NIMBLE_ENABLED
#include "services/gap/ble_svc_gap.h" /* ble_svc_gap_device_name_set */
#include "esp_rom_sys.h"
#endif
// ─── 模块常量 ─────────────────────────────────────────────────────────────────
#define CLEAR_WIFI_BUTTON_PIN GPIO_NUM_0 ///< 清除 WiFi 凭证的长按按键（Boot 按钮）
#define MAX_RETRY_COUNT 5                ///< WiFi 断线后最大自动重连次数（已配网运行态用）

/// @brief 配网态连接失败最大重试次数：BluFi 配网期间下发凭证后连不上，连续尝试
/// 这么多次仍拿不到 IP，就判定"本次配网失败"，向 App 回报 CONN_FAIL 并停下等
/// App 重发凭证（不重启、不置 WIFI_FAIL_BIT，与运行态重连逻辑完全隔离）。
/// 3 次足以排除偶发抖动，又不会让用户等太久（约 6~10s）。
#define PROV_MAX_RETRY_COUNT 3

/// @brief WiFi 断线去抖时长（ms）：断线后等这么久仍未恢复，才通知上层断开 WS/MQTT。
/// 绝大多数 WiFi 抖动在 1~2 秒内自愈，期间不重建协议层 → 避免内部 SRAM 碎片化。
#define WIFI_DEBOUNCE_MS 1500

/// @brief 运行态重连额度耗尽后的"复位看门狗"缓冲时长（ms）。
/// 运行态（已配网、非开机首连）断线重连 MAX_RETRY_COUNT 次仍失败时，不再置
/// WIFI_FAIL_BIT 永久放弃（那样运行态无人接管 → 彻底卡死，只能人工软重启，见修复背景），
/// 而是启动本看门狗：期间仍继续 esp_wifi_connect() 尝试自愈；若 WIFI_RESET_WATCHDOG_MS
/// 内成功拿到 IP（GOT_IP 会取消本定时器），则不复位；否则判定真掉网，esp_restart() 刷新。
/// 复位=用户原本手动软重启的自动化版：内存干净重来，规避运行态无限重连的 SRAM 碎片/泄漏
/// （见 BUG-011 / BUG-023）。给 8 秒缓冲：让 reason=1 这类驱动 1 秒内连抖 5 次、AP 稍后自愈
/// 的正常波动不被误复位，只有真正持续掉网才复位。
#define WIFI_RESET_WATCHDOG_MS 8000

static const char *TAG = "EchoPals";

// ─── 模块级状态变量 ────────────────────────────────────────────────────────────

/// @brief 是否正处于 BLE 配网流程中（配网期间禁止断线重连，避免与配网状态机冲突）
static bool s_is_provisioning = false;
/// @brief 是否正在执行解绑/重置（清凭证 + 即将 esp_restart）。置位后 esp_wifi_restore()
/// 触发的 WIFI_EVENT_STA_DISCONNECTED 里的重连 / 去抖等重活全部跳过——马上就要重启，
/// 这些操作纯属多余，且叠在 2304 字节的 sys_evt 栈上会栈溢出崩溃（解绑必崩的根因）。
static volatile bool s_is_resetting = false;
/// @brief GOT_IP 事件请求主流程补写信道提示到 NVS。NVS 写栈开销大，不能在 sys_evt
/// 小栈（2304B）上做，故 GOT_IP 回调仅置位，由 bsp_board_wifi_main 在大栈上执行。
static volatile bool s_need_save_channel_hint = false;
/// @brief GOT_IP 事件请求主流程向手机补发 BluFi 配网成功报告。同样为避免 sys_evt
/// 小栈溢出，回调仅置位，由 bsp_board_wifi_main 在大栈上、蓝牙释放前补发。
static volatile bool s_need_send_prov_report = false;
/// @brief 当前已重连次数（超过 MAX_RETRY_COUNT 后置位 WIFI_FAIL_BIT）
static int s_retry_num = 0;
/// @brief 运行态离线模式标志：运行态重连额度耗尽后置位，**本次开机周期内永不清除**。
/// 置位后：不再 esp_wifi_connect()、不再启动复位看门狗、不再 esp_restart()，
/// 且已 esp_wifi_stop() 关射频；所有网络功能靠 bsp_wifi_is_offline_mode() 自行拦截。
/// 恢复联网需用户手动关机重开（冷启动后本变量自然复位为 false）。
/// volatile：由 sys_evt 事件回调置位，被多个业务任务读取。
static volatile bool s_offline_mode = false;
/// @brief 配网态连接尝试计数：BluFi 配网期间专用，与 s_retry_num（运行态）隔离。
/// REQ_CONNECT_TO_AP 触发连接后每次断线 +1，达到 PROV_MAX_RETRY_COUNT 判定配网失败；
/// App 重新下发凭证（RECV_STA_SSID/PASSWD）时清零，让二次配网干净开始。
static int s_prov_retry_num = 0;
/// @brief 连续 reason=201(NO_AP_FOUND) 的次数：用于判断信道记忆是否真的失效。
/// 单次 201 可能只是同 SSID 多 BSSID 环境下的偶然波动（这次没扫到，下次驱动
/// 自己就换到了对的 AP），连续 2 次才认为记忆过期，值得清空退回全信道扫描。
static int s_no_ap_found_count = 0;

/// @brief [DIAG-TEST] 真实 SSID 备份 + 是否已注入过一次性错误 SSID 的标志。
/// 用于验证"连续 2 次 201 才清空信道提示"回退逻辑：只污染一次 SSID，让它
/// 自然触发失败→回退→用真实 SSID 重连成功的完整闭环，不需要人工改回、
/// 也不会真的耗到 MAX_RETRY_COUNT 触发重启。测试完毕整块删除。
static char s_diag_real_ssid[33] = {0};
static bool s_diag_ssid_corrupted = false;

/// @brief WiFi 断线去抖定时器（one-shot）：断线时启动，GOT_IP 时取消
static TimerHandle_t s_wifi_debounce_timer = NULL;
/// @brief 运行态重连额度耗尽后的复位看门狗（one-shot）：达上限时启动，GOT_IP 时取消；
/// 超时（WIFI_RESET_WATCHDOG_MS 内仍未拿到 IP）则 esp_restart() 刷新设备。
static TimerHandle_t s_wifi_reset_timer = NULL;
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

    // ★关闭 UDP 日志 socket：断线确认（非短暂抖动）后，旧 socket 绑定的连接状态
    //   已经失效，留着它没意义，且避免和重连后 GOT_IP 的 udp_logger_start() 出现
    //   "该关未关、该建又建"的混乱。GOT_IP 恢复时会重新 start，重建全新 socket。
    // udp_logger_stop();
}

// ─── wifi_reset_watchdog_cb ──────────────────────────────────────────────────

/**
 * @brief 运行态重连额度耗尽后的"复位看门狗"回调（真正执行软复位的地方）
 *
 * 运行态断线重连 MAX_RETRY_COUNT 次仍失败时启动本 one-shot 定时器（见断线事件处理）。
 * WIFI_RESET_WATCHDOG_MS 内若 GOT_IP 恢复，本回调会被取消、永不执行；只有缓冲期满
 * 仍未拿到 IP（判定真掉网），才在此 esp_restart() 刷新设备——等价于用户原本手动软
 * 重启，让内存干净重来，规避运行态无限重连的 SRAM 碎片/泄漏（BUG-011 / BUG-023）。
 *
 * @param xTimer 定时器句柄（未使用）
 * @note 运行在 FreeRTOS Timer 服务任务上下文；esp_restart() 不返回。
 */
static void wifi_reset_watchdog_cb(TimerHandle_t xTimer)
{
    (void)xTimer;
    // ★ 离线模式保险：进入离线模式时已 xTimerStop 本定时器，但 xTimerStop 是"投递
    //   命令到定时器服务队列"而非立即生效，若本回调恰好已被排入队列，stop 拦不住它，
    //   仍会走到下面 esp_restart() → 离线模式失效（设备照样重启）。故此处再判一次，
    //   把这个竞态窗口彻底堵死。
    if (s_offline_mode)
        return;

    ESP_LOGE(TAG, "WiFi 重连 %d 次全部失败，且缓冲 %d ms 内仍未恢复 → 软复位刷新设备",
             MAX_RETRY_COUNT, WIFI_RESET_WATCHDOG_MS);
    esp_restart(); // 内存干净重来；不返回
}

// ─── bsp_wifi_is_offline_mode ────────────────────────────────────────────────

/**
 * @brief 查询设备是否已进入运行态离线模式（接口说明见 bsp_board.h）
 *
 * @return true = 已离线，所有网络功能必须跳过
 */
bool bsp_wifi_is_offline_mode(void)
{
    return s_offline_mode;
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
    // ★先置"正在重置"标志：esp_wifi_restore() 会触发 WIFI_EVENT_STA_DISCONNECTED，
    //   该事件在 sys_evt 任务（栈仅 2304B）上跑，若照常执行重连 + 去抖等重活会栈溢出
    //   崩溃（解绑必崩根因）。此刻马上就要 esp_restart，重连毫无意义，故让事件处理
    //   开头据此标志直接 return，避开重活。
    s_is_resetting = true;

    // wifi_prov_mgr_reset_provisioning() 内部删除 WiFi 配置分区中的 SSID/密码键值对
    // err = wifi_prov_mgr_reset_provisioning();
    err = esp_wifi_restore(); // 它可以在任何状态下安全地抹除 WiFi 配置
    if (err == ESP_OK)
        ESP_LOGI(TAG, "WiFi 凭证清除成功，立即重启...");
    else
        ESP_LOGE(TAG, "WiFi 凭证清除失败: %s", esp_err_to_name(err));

    // ── 立即软件复位（重启后重新进入配网模式）─────────────────────────────
    // ★不再 vTaskDelay(1000) 等日志：esp_wifi_restore() 会触发一连串 WiFi 事件，
    //   这些事件在 sys_evt 任务上处理，与本次 restore/重启并发时容易撞车（栈溢出
    //   或 esp_event 锁失效断言，即解绑必崩）。凭证已清、目标就是重启回配网态，
    //   故清完立即 esp_restart()，把事件在 sys_evt 上闹事的时间窗口压到最小。
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
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余

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
                PRINT_TASK_CREATED(TAG, "btn_reset", 8192, 1); // 栈在内部SRAM
                vTaskDelete(NULL);                             // btn_task 使命完成自删（清除+重启在 btn_reset 任务里完成）
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
        s_prov_retry_num = 0; // App 重发凭证 → 上一轮失败计数作废，二次配网干净开始
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
        s_prov_retry_num = 0; // App 重发凭证 → 上一轮失败计数作废，二次配网干净开始
        ESP_LOGI(TAG, "收到 WiFi 密码（长度 %d）", param->sta_passwd.passwd_len);
        break;

    case ESP_BLUFI_EVENT_REQ_CONNECT_TO_AP:
        // 手机下发完凭证，请求设备连接 AP
        // 注意：这里【不再】置 s_is_provisioning = false。保持配网态，让断线失败
        // 走配网专用失败逻辑（有限次重试→回报 CONN_FAIL→停下等 App 重发），而不是
        // 掉进运行态那套"耗尽 5 次→WIFI_FAIL_BIT→重启"的路径。s_is_provisioning
        // 只在真正拿到 IP（GOT_IP）时才清零，标志配网成功结束。
        ESP_LOGI(TAG, "BluFi 请求连接 WiFi，开始连接...");
        s_prov_retry_num = 0;  // 本次连接尝试从 0 开始计数（二次配网也在此清零）
        esp_wifi_disconnect(); // 先断开（若之前已连过），确保触发连接回调
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
static void save_ap_channel_hint(void); // 前向声明：定义在文件后部，GOT_IP 分支中调用

static void wifi_ip_event_handler(void *arg, esp_event_base_t event_base,
                                  int32_t event_id, void *event_data)
{
    bsp_board_t *bsp_board = (bsp_board_t *)arg;

    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START)
    {
        // ── STA 模式已启动，立即尝试连接（使用 NVS 中已存储的 SSID/密码）────
        ESP_LOGW(TAG, "[DIAG] WIFI_EVENT_STA_START @ %lld us，调用 esp_wifi_connect()",
                 esp_timer_get_time());
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
            return; // 非配网触发的扫描，忽略
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
        // ★解绑/重置拦截：clear_wifi_and_restart() 里 esp_wifi_restore() 会触发本事件，
        //   此刻设备正准备 esp_restart，重连 + 去抖等重活既多余又会撑爆 sys_evt 栈
        //   （2304B）导致解绑必崩。据 s_is_resetting 直接返回，让重启流程干净收尾。
        if (s_is_resetting)
        {
            // ESP_LOGW(TAG, "解绑/重置进行中，忽略断线事件（即将重启）");
            esp_rom_printf("[WIFI return \n");
            return;
        }

        // ★离线模式拦截（必须放在最前）：进入离线模式时会调 esp_wifi_disconnect() +
        //   esp_wifi_stop()，这两个调用本身还会再抛一次 STA_DISCONNECTED 事件。若不在
        //   此短路，事件会重新落回下方"重连额度耗尽"分支 → 再次 stop → 再抛事件，
        //   形成递归自激。离线后本事件已无任何处理意义（不重连、不去抖、不重启），
        //   直接返回即可。
        if (s_offline_mode)
            return;

        wifi_event_sta_disconnected_t *disc = (wifi_event_sta_disconnected_t *)event_data;
        ESP_LOGW(TAG, "[DIAG] WIFI_EVENT_STA_DISCONNECTED @ %lld us，reason=%d",
                 esp_timer_get_time(), disc ? disc->reason : -1);

        // ══ 配网态失败处理（与运行态重连逻辑完全隔离）════════════════════════
        // 仅当正处于 BluFi 配网流程、且手机蓝牙仍连着时走这里。典型场景：手机下发
        // 了错误密码 → 反复连不上。ESP32 驱动无法 100% 区分"密码错/信号差"，密码
        // 错最常见报 reason=15(握手超时)/205/2(认证失败) 等；这里不纠结精确原因，
        // 统一用"连续尝试 PROV_MAX_RETRY_COUNT 次仍拿不到 IP"判定本次配网失败。
        //   · 未达上限 → 继续 esp_wifi_connect() 重试；
        //   · 达到上限 → ① 清除 NVS 里刚写入的错误 WiFi 账号密码（关键，见下）；
        //     ② 向手机回报 CONN_FAIL；③ 停止重连、清零计数、保持配网态与蓝牙连接，
        //     静候 App 重新下发凭证（RECV_STA_SSID/PASSWD 会再清零 → 无限次重配）。
        // 全程不置 WIFI_FAIL_BIT、不重启、不断蓝牙，避免掉进运行态那套失败重启路径。
        //
        // 为什么必须清凭证：esp_wifi_set_config()（在 RECV_STA_PASSWD 里调用）会把
        // SSID/密码【自动持久化到 NVS】，不管密码对错。若不清，设备一旦重启就会
        // 被 wifi_is_provisioned() 判定为"已配网"→ 走已配网直连分支拿错密码死连→
        // 重启→再死连，永远回不到配网模式（正是之前"错误后无法二次配网"的真因）。
        //
        // 【踩坑】不能用 esp_wifi_restore() 清：它是重量级操作，会重置整个 WiFi
        // 配置栈（含 pmksa_cache）。在配网中途、STA 刚断开且后续还会 set_config
        // 的场景下调用，会与 BluFi 第二轮下发凭证时的 wpa_config_reload 打架，实测
        // 直接 LoadProhibited 崩溃（pmksa_cache_flush 空指针），且凭证清不干净、
        // 重启后又被判"已配网"。改用温和方式：disconnect 停稳 + 空 config 覆盖，
        // 只抹 SSID/密码，不动 WiFi 栈本身。
        if (s_is_provisioning && s_blufi_ble_connected)
        {
            if (s_prov_retry_num < PROV_MAX_RETRY_COUNT)
            {
                esp_wifi_connect();
                s_prov_retry_num++;
                ESP_LOGW(TAG, "配网态连接失败（reason=%d），重试... (%d/%d)",
                         disc ? disc->reason : -1, s_prov_retry_num, PROV_MAX_RETRY_COUNT);
            }
            else
            {
                ESP_LOGE(TAG, "配网态连续 %d 次连接失败，判定密码/凭证错误：回报 App 失败 + 清凭证 + 等待重发",
                         PROV_MAX_RETRY_COUNT);
                s_prov_retry_num = 0; // 复位，等 App 重发凭证后重新计数

                // ① 先趁 WiFi 状态还正常，通过 BluFi 内置的"WiFi 连接状态上报"帧
                //    告诉 App：连接失败。与成功上报走同一函数/同一回调，App 端只需
                //    判 status：CONN_SUCCESS(0)=成功，CONN_FAIL(1)=失败。
                //    放在清凭证之前，避免清除操作扰乱 mode / 上报时序。
                wifi_mode_t mode;
                esp_wifi_get_mode(&mode);
                esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_FAIL, 0, NULL);

                // ② 停稳 STA，再用全零 wifi_config 覆盖，抹掉刚被持久化的错误账号
                //    密码（同样会写回 NVS，把错凭证清成空）。不用 restore，避免崩溃。
                //    ★wifi_config_t 达 132 字节，本函数跑在 sys_evt 任务（栈仅 2304B），
                //    不能在栈上放这么大的局部变量（会栈溢出，是 86d0553 引入的解绑必崩根因）。
                //    改为函数级 static：不占栈；事件回调由单一 sys_evt 任务串行处理，无重入。
                esp_wifi_disconnect();
                static wifi_config_t empty_cfg;
                memset(&empty_cfg, 0, sizeof(empty_cfg));
                esp_err_t clr_err = esp_wifi_set_config(WIFI_IF_STA, &empty_cfg);
                ESP_LOGW(TAG, "已用空配置覆盖清除错误 WiFi 凭证: %s", esp_err_to_name(clr_err));

                // ③ 不再重连、不重启、不断蓝牙，保持配网态静待 App 重新下发凭证。
                //    App 重发 SSID/密码 → RECV_STA_* 重新 set_config + 清零计数 → 原地重配。
            }
            return; // 配网态到此结束，不落入运行态重连 / 去抖逻辑
        }

        if (!s_is_provisioning) // 配网期间不触发重连（避免抢占配网状态机）
        {
            // ── 信道提示纠错：连续 2 次 reason=201(NO_AP_FOUND) 才清空提示退回全扫 ──
            // 201 的语义是"扫描范围内压根没找到匹配的 AP"，本应对应"记忆的信道已
            // 过期（路由器换信道了）"这种场景；但实测发现同一个 SSID 背后可能存在
            // 多个不同 BSSID 的 AP（Mesh 子节点 / 路由器多 AP 负载均衡），单次 201
            // 可能只是这次没扫到其中一个 AP 的偶然波动，下一次驱动自己就能换到能连
            // 上的那个 —— 若第 1 次就清空提示会误伤这种正常波动。改为连续 2 次才
            // 认定记忆真的过期，值得清空。其他断线原因（信号弱 246、握手超时 15 等）
            // 大多是网络暂时不稳，信道本身没问题，不应清除提示，也不计入这个计数——
            // 否则会把"正常抖动重连"错误降级成全信道扫描，拖慢本可以立即重连成功的场景。
            // 只在信道提示还在生效（scan_method=FAST_SCAN）时才需要清，普通全扫模式
            // 下这个分支不生效，不影响原有重连行为。
            if (disc && disc->reason == WIFI_REASON_NO_AP_FOUND)
            {
                s_no_ap_found_count++;
                ESP_LOGW(TAG, "[DIAG] reason=201，连续计数=%d", s_no_ap_found_count);
                if (s_no_ap_found_count >= 2)
                {
                    // ★同上：wifi_config_t 132 字节，不能放 sys_evt 小栈，改函数级 static。
                    static wifi_config_t cfg;
                    memset(&cfg, 0, sizeof(cfg));
                    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) == ESP_OK &&
                        cfg.sta.scan_method == WIFI_FAST_SCAN)
                    {
                        cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
                        cfg.sta.channel = 0;
                        // ── [DIAG-TEST] 已注释：还原测试污染 SSID 的实验代码 ──────
                        // 配合上方 load_ap_channel_hint 里的 SSID 注入实验，已注释。
                        // if (s_diag_ssid_corrupted && s_diag_real_ssid[0] != '\0')
                        // {
                        //     strncpy((char *)cfg.sta.ssid, s_diag_real_ssid, sizeof(cfg.sta.ssid) - 1);
                        //     s_diag_ssid_corrupted = false;
                        //     ESP_LOGW(TAG, "[DIAG-TEST] 已还原真实 SSID：%s", cfg.sta.ssid);
                        // }
                        esp_wifi_set_config(WIFI_IF_STA, &cfg);
                        ESP_LOGW(TAG, "[DIAG] 连续 2 次 reason=201，信道提示已过期，清除后退回全信道扫描");
                    }
                    s_no_ap_found_count = 0;
                }
            }
            else
            {
                // 非 201 的失败：说明这次至少找到了 AP（只是认证/握手等其他环节出问题），
                // 不代表信道记忆有问题，清零计数避免和不相关的失败次数累积到一起。
                s_no_ap_found_count = 0;
            }

            if (s_retry_num < MAX_RETRY_COUNT)
            {
                // 未超过最大重试次数，继续尝试重连
                esp_wifi_connect();
                s_retry_num++;
                ESP_LOGW(TAG, "WiFi 掉线，正在重连... (%d/%d)", s_retry_num, MAX_RETRY_COUNT);
            }
            else
            {
                // ══ 运行态重连额度耗尽：进入永久离线模式 ══════════════════════════
                // 【演进史】本分支先后有三代行为，坑都踩过一遍，务必别改回去：
                //   一代：置 WIFI_FAIL_BIT 永久放弃 → 该位只有开机首连的 wifi_main 在等，
                //         运行态置位后无人接管，s_retry_num 又只在 GOT_IP 清零 → 此后
                //         再无代码调 esp_wifi_connect()，设备"静默卡死"，且 WS/MQTT/天气
                //         等任务仍在空转重试，白耗内存。
                //   二代：继续 esp_wifi_connect() + 8s 复位看门狗 → 到期 esp_restart()。
                //         能自愈，但断网环境下会陷入"重连失败→重启→再重连失败"的循环，
                //         设备反复重启、本地功能（游戏/GIF/闹钟）全被打断，用户体验差。
                //   三代（当前）：判定彻底掉网，进入**永久离线模式**——既不重连也不重启，
                //         主动关停 WiFi 并置位全局标志，让所有网络消费者提前返回退出，
                //         把设备干净地降级成"纯本地机"，本地功能全部照常可用。
                //         恢复联网需用户手动关机重开（冷启动后 s_offline_mode 复位）。
                //
                // 【与一代的本质区别】一代是"被动卡死"（标志无人接管、任务照样空转）；
                //   本代是"主动关停"：① 有明确的 s_offline_mode 标志供全局查询；
                //   ② ws_reconn / mqtt_reconn 等重连任务会读到标志后**真正退出并自删**，
                //   不再空转占用 6KB 内部 SRAM / 3KB PSRAM；③ 射频已关，功耗更低。
                //
                // 【为什么在 sys_evt 上下文只做轻量操作】本回调跑在 sys_evt 任务上，栈仅
                //   2304B。故这里只做：置标志 + 停定时器 + wifi 关停 + 打日志，绝不做
                //   NVS 写 / UI 调用 / 建任务等重活（解绑必崩就是栈溢出踩的坑）。
                s_offline_mode = true;

                // ① 停掉复位看门狗：否则它到期仍会 esp_restart()，离线模式直接失效。
                //    （回调内另有 s_offline_mode 短路兜底，防 xTimerStop 的投递延迟竞态）
                if (s_wifi_reset_timer != NULL)
                    xTimerStop(s_wifi_reset_timer, 0);

                // ② 停掉断线去抖定时器并立即清 WIFI_BIT：已判定彻底掉网，无需再等
                //    1.5s 去抖确认，让上层（WS/MQTT）尽快感知网络不可用。
                if (s_wifi_debounce_timer != NULL)
                    xTimerStop(s_wifi_debounce_timer, 0);
                if (bsp_board != NULL)
                    xEventGroupClearBits(bsp_board->board_status, WIFI_BIT);

                // ③ 关闭 UDP 日志 socket：断网后它已失效，且离线不会再有 GOT_IP 重建。
                // udp_logger_stop();

                // ④ 彻底关停 WiFi：disconnect 停掉驱动层自动重连，stop 关射频省电。
                //    注意不做 esp_wifi_deinit()——deinit 会释放驱动内部资源，与仍可能
                //    在途的事件回调存在竞态，且离线后也不再需要那点内存，得不偿失。
                //    这两个调用会各再抛一次 STA_DISCONNECTED 事件，已由本函数开头的
                //    s_offline_mode 短路拦截，不会递归回到这里。
                esp_wifi_disconnect();
                esp_wifi_stop();

                // ⑤ 通知提醒系统停掉 SNTP 轮询与定时天气拉取。
                //    该函数刻意做得极轻（esp_sntp_stop + 一次字段赋值 + 一条日志），
                //    可安全在 sys_evt 的 2304B 小栈上调用；闹钟/倒计时不受影响。
                reminder_on_offline_mode();

                ESP_LOGE(TAG, "WiFi 重连 %d 次全部失败 → 已进入【离线模式】：停止一切重连，"
                              "关闭射频与全部联网功能（对话/MQTT/天气/OTA）",
                         MAX_RETRY_COUNT);
                ESP_LOGE(TAG, "本地功能（GIF/舵机/震动/触摸/游戏/唤醒词/闹钟/本地音频）不受影响；"
                              "恢复联网请手动关机后重新开机");
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
        ESP_LOGW(TAG, "[DIAG] IP_EVENT_STA_GOT_IP @ %lld us", esp_timer_get_time());

        // ── [DNS诊断] 打印设备实际拿到的 DNS 服务器地址 ─────────────────────────
        //   现象：设备 getaddrinfo() 对 api.strailine-space.com 返回 202(EAI_FAIL)，
        //   而电脑（用 8.8.8.8）能正常解析出 122.224.191.2 → 怀疑设备侧 DNS 不通：
        //   要么 DHCP 没下发 DNS server，要么下发的内网 DNS 解析不了该域名。
        //   这里只【读】netif 的 DNS 配置并打印（无堆分配、无阻塞），可安全放在
        //   sys_evt 小栈（2304B）上——不像 getaddrinfo() 那样会走查询+吃栈+阻塞。
        //   若打印出 DNS=0.0.0.0 → DHCP 没给 DNS；若是路由器内网 IP → 换成公共 DNS
        //   （esp_netif_set_dns_info 设 8.8.8.8/223.5.5.5）即可修复。诊断完删除本块。
        {
            esp_netif_dns_info_t dns_main = {0};
            esp_netif_dns_info_t dns_backup = {0};
            esp_netif_get_dns_info(event->esp_netif, ESP_NETIF_DNS_MAIN, &dns_main);
            esp_netif_get_dns_info(event->esp_netif, ESP_NETIF_DNS_BACKUP, &dns_backup);
            ESP_LOGW(TAG, "[DNS诊断] 主 DNS = " IPSTR " | 备 DNS = " IPSTR,
                     IP2STR(&dns_main.ip.u_addr.ip4), IP2STR(&dns_backup.ip.u_addr.ip4));
        }

        //! ★纯电池调试用：拿到 IP 后启动 UDP 日志单播（发到 udp_logger.c 里配置的电脑 IP），
        //   电脑用 tools/udp_log_listen.py 监听同端口即可看到日志——解决"纯锂电池供电无法
        //   接串口看低功耗全程日志"的问题。幂等，断线重连多次调用无副作用；不影响原 UART
        //   日志输出（两路并存）。传 NULL 用 udp_logger.c 里的默认目标 IP。
        // udp_logger_start(NULL, 0);

        // 连接成功，重置重连计数（下次断线时从 0 开始重新计数）
        s_retry_num = 0;
        s_no_ap_found_count = 0;

        // ★信道提示的 NVS 写【不在此处做】：本回调运行在 sys_evt 任务（栈仅 2304B），
        //   save_ap_channel_hint() 内部 nvs_open/set_blob/commit 栈开销很大，叠加本分支
        //   已有的 udp_logger_start(建socket) + esp_blufi_send_wifi_conn_report(BluFi加密帧)
        //   会撑爆 sys_evt 栈（实测配网拿到 IP 瞬间 stack overflow 崩溃重启）。改为置位
        //   请求标志，由 bsp_board_wifi_main() 主流程在自己的大栈上补写（见 WIFI_BIT 就绪后）。
        s_need_save_channel_hint = true;

        // ── 去抖：网络已恢复，取消"宣告掉线"定时器 ───────────────────────────
        // 若本次断线在去抖窗口内恢复，定时器回调不会执行，WIFI_BIT 从未被清，
        // WS/MQTT 也就不会经历销毁重建 → 从源头避免碎片。
        if (s_wifi_debounce_timer != NULL)
            xTimerStop(s_wifi_debounce_timer, 0);

        // ── 复位看门狗：网络已恢复，取消"缓冲期满就软复位"的定时器 ─────────────
        // 运行态曾重连耗尽启动了复位看门狗；此刻 AP 回来并拿到 IP，说明是可自愈的
        // 波动，撤销复位。s_retry_num 下方清零，下次断线额度从头再来。
        if (s_wifi_reset_timer != NULL)
            xTimerStop(s_wifi_reset_timer, 0);

        // 置位 WIFI_BIT，解除 bsp_board_wifi_main() 末尾的 xEventGroupWaitBits 阻塞
        if (bsp_board)
            xEventGroupSetBits(bsp_board->board_status, WIFI_BIT);

        // ── BluFi 配网期：拿到 IP 即视为配网成功 ─────────────────────────────
        // BluFi 没有 Unified Provisioning 的 WIFI_PROV_END 事件，改由「拿到 IP」作为
        // 配网结束信号。★向手机回报连接成功的 esp_blufi_send_wifi_conn_report【不在此处
        // 调用】：它内部要组 BluFi 加密帧、走协议栈，栈开销不小，叠加本分支的 socket/NVS
        // 操作会撑爆 sys_evt 小栈（2304B）。改为置标志，由 bsp_board_wifi_main 主流程在
        // 大栈上、蓝牙释放前补发（时序上仍早于 esp_blufi_host_deinit，手机能正常收到）。
        s_need_send_prov_report = true;
        // 配网真正成功（拿到 IP）→ 解除配网态：此后断线才走运行态重连逻辑。
        // 也清零配网态计数，保持状态干净。
        s_is_provisioning = false;
        s_prov_retry_num = 0;
        if (bsp_board)
            xEventGroupSetBits(bsp_board->board_status, PROV_DONE_BIT);
    }
}

// ─── save_ap_channel_hint / load_ap_channel_hint ─────────────────────────────

/**
 * @brief 将当前已连接 AP 的信道 + BSSID 存入 NVS（信道定向连接的加速提示）
 *
 * 已配网直连时 esp_wifi 默认逐信道（1~13）扫描寻找目标 AP，实测耗时 ~2.4s。
 * 存下本次连接成功的信道号和 BSSID，下次启动时填入 wifi_config_t 后，驱动
 * 只需在该信道定向查找，省去全信道扫描。
 *
 * @note 调用者：wifi_ip_event_handler() 的 IP_EVENT_STA_GOT_IP 分支
 * @note 每次成功连接都刷新一次，而非只存一次：路由器信道可能变化（如自动
 *       选道重选），需要保持提示信息与实际信道同步。
 */
static void save_ap_channel_hint(void)
{
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) != ESP_OK)
        return;

    nvs_handle_t h;
    if (nvs_open("net_config", NVS_READWRITE, &h) != ESP_OK)
        return;
    nvs_set_u8(h, "wifi_ch", ap_info.primary);
    nvs_set_blob(h, "wifi_bssid", ap_info.bssid, sizeof(ap_info.bssid));
    nvs_commit(h);
    nvs_close(h);
    ESP_LOGI(TAG, "已记录信道提示：channel=%d，下次连接可跳过全信道扫描", ap_info.primary);
}

/**
 * @brief 从 NVS 读取上次记录的信道 + BSSID，填入 STA 配置（连接前调用）
 *
 * 若 NVS 中无记录（首次连接、或曾清除过），保持 wifi_config 原样不变，
 * 驱动退化为默认的全信道扫描 —— 不影响可用性，仅少了加速效果。若信道已
 * 变化导致定向连接失败，esp_wifi 会自动回退全信道扫描，连上后下次又会
 * 被 save_ap_channel_hint() 刷新为新信道。
 *
 * @note 调用者：bsp_board_wifi_main() 已配网直连分支，在 esp_wifi_start() 之前
 */
static void load_ap_channel_hint(void)
{
    nvs_handle_t h;
    if (nvs_open("net_config", NVS_READONLY, &h) != ESP_OK)
        return;

    uint8_t channel = 0;
    size_t bssid_len = 6;
    uint8_t bssid[6] = {0};
    bool have_channel = (nvs_get_u8(h, "wifi_ch", &channel) == ESP_OK) && channel > 0;
    bool have_bssid = (nvs_get_blob(h, "wifi_bssid", bssid, &bssid_len) == ESP_OK) && bssid_len == 6;
    nvs_close(h);

#define DIAG_FORCE_WRONG_CHANNEL 1 // 保留宏定义供下方 bssid_set 清空块复用（SSID 测试不依赖它触发信道错误）

    if (!have_channel)
        return;

    wifi_config_t cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &cfg) != ESP_OK)
        return;
    cfg.sta.scan_method = WIFI_FAST_SCAN; // 定向扫描（只扫指定信道，按 SSID 匹配）
    cfg.sta.channel = channel;            // 记录的信道号

    // ── [DIAG-TEST] 已注释：人为把 SSID 改成不存在名字强制触发 201 的实验代码 ──
    // 该实验用于验证"连续 2 次 201 才清空信道记忆"的回退逻辑，已完成使命。
    // 现在要测真实的"密码错误"配网失败：若保留会把 SSID 污染成假名字，逼出的是
    // reason=201（找不到 AP），而非真正的密码错误码（reason=15 握手超时等），
    // 会干扰判断。故整块注释。确认稳定后可连同 s_diag_* 变量一并删除。
    //
    // #if DIAG_FORCE_WRONG_CHANNEL
    //     strncpy(s_diag_real_ssid, "TC-BJ", sizeof(s_diag_real_ssid) - 1);
    //     if (!s_diag_ssid_corrupted)
    //     {
    //         strncpy((char *)cfg.sta.ssid, "TC-BJ-NOTEXIST", sizeof(cfg.sta.ssid) - 1);
    //         s_diag_ssid_corrupted = true;
    //         ESP_LOGW(TAG, "[DIAG-TEST] 故意把 SSID 改成不存在的 \"%s\"...", cfg.sta.ssid);
    //     }
    // #endif
    //
    // #if DIAG_FORCE_WRONG_CHANNEL   // 强制清空 bssid_set，不让旧 BSSID 帮驱动纠错
    //     cfg.sta.bssid_set = false;
    //     memset(cfg.sta.bssid, 0, sizeof(cfg.sta.bssid));
    // #endif

    // 不设 bssid_set：只给信道提示，不做 MAC 精确匹配。
    // 之前 channel+bssid_set 组合在断线重连路径上触发了 esp-idf 驱动的不稳定行为
    // （实测：连续 4 次 reason=201 NO_AP_FOUND，且日志里未见驱动真正切换信道，
    // 直到第 5 次耗光重试后才碰巧成功）。只给 channel，匹配逻辑退回按 SSID 找
    // （和原全信道扫描一致，同名 WiFi 冲突风险不变），只是把扫描范围从 13 个
    // 信道收窄到 1 个。have_bssid/bssid 变量暂时不再使用，NVS 里继续存着，
    // 留作后续排查 bssid_set 问题的数据。
    (void)have_bssid;
    // ── 诊断日志：写入前打印将要下发的字段，确认 scan_method 是否为默认全扫 ──
    ESP_LOGW(TAG, "[DIAG] 写入前 cfg: scan_method=%d channel=%d bssid_set=%d bssid=%02X:%02X:%02X:%02X:%02X:%02X ssid=%s",
             cfg.sta.scan_method, cfg.sta.channel, cfg.sta.bssid_set,
             cfg.sta.bssid[0], cfg.sta.bssid[1], cfg.sta.bssid[2], cfg.sta.bssid[3], cfg.sta.bssid[4], cfg.sta.bssid[5],
             cfg.sta.ssid);
    esp_err_t set_err = esp_wifi_set_config(WIFI_IF_STA, &cfg);
    ESP_LOGW(TAG, "[DIAG] esp_wifi_set_config 返回: %s", esp_err_to_name(set_err));

    // 回读校验：确认驱动真的记住了这些字段（而不是被内部逻辑重置）
    wifi_config_t verify_cfg = {0};
    if (esp_wifi_get_config(WIFI_IF_STA, &verify_cfg) == ESP_OK)
    {
        ESP_LOGW(TAG, "[DIAG] 回读校验 cfg: scan_method=%d channel=%d bssid_set=%d",
                 verify_cfg.sta.scan_method, verify_cfg.sta.channel, verify_cfg.sta.bssid_set);
    }
    ESP_LOGI(TAG, "已加载信道提示：channel=%d，尝试定向连接（连不上会自动回退全信道扫描）", channel);
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

    // ── 步骤 3.6：创建运行态"复位看门狗"定时器（one-shot）───────────────────
    // 运行态重连额度耗尽后启动；缓冲期满仍未恢复则 esp_restart() 刷新（见断线事件处理）。
    if (s_wifi_reset_timer == NULL)
    {
        s_wifi_reset_timer = xTimerCreate(
            "wifi_reset_wd",
            pdMS_TO_TICKS(WIFI_RESET_WATCHDOG_MS),
            pdFALSE, // one-shot：触发一次即复位，不自动重载
            NULL,
            wifi_reset_watchdog_cb);
        if (s_wifi_reset_timer == NULL)
            ESP_LOGE(TAG, "WiFi 复位看门狗定时器创建失败，达上限将退化为立即软复位");
    }

    // ── 步骤 4：初始化 WiFi 驱动并设为 STA 模式 ──────────────────────────────
    // BluFi 配网与已配网直连都基于 STA：配网时手机把家庭 WiFi 凭证下发到 STA。
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    PRINT_MEM_INFO(TAG, "esp_wifi_init+set_mode 后");

    // ── 步骤 5：启动按键监控任务（GPIO0 长按 3s 触发 WiFi 重置）─────────────
    // button_monitor_task 会调用 nvs_erase_key/nvs_set_str/nvs_commit（Flash 操作），
    // Flash 操作占用 SPI 总线期间 CPU 需访问任务栈，栈必须在内部 SRAM，否则 WDT 复位。
    xTaskCreatePinnedToCoreWithCaps(
        button_monitor_task, "btn_task",
        3072, NULL, 5, NULL,
        0, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
    PRINT_TASK_CREATED(TAG, "btn_task", 3072, 1); // 栈在内部SRAM

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
        // 配网入口仅靠蓝牙广播 + 串口打印设备 ID；不再显示配网二维码（已删除该逻辑）。

        // ── 显示配网提示图（仅未配网分支，必须在 BT controller 拉起之前）────────
        // 首次配网全程原本是黑屏，用户无从判断设备状态；这里先点屏贴一张静态
        // "设备未连接 / 连接 APP" 提示图（pw.bin）。
        // ★安全性：只贴静态图，flush 完整屏后 LVGL 彻底静默（无 GIF 解码、无动画
        //   timer），不会在 BLE 使能窗口内并发刷屏抢内部资源（对比 BUG-026：真正
        //   危险的是 GIF 轮播那类持续负载，它仍留在配网之后的 ui_init()）。
        // ★已配网分支不调用本函数，开机顺序与改动前完全一致。
        ui_show_provision_image();

        // ★WiFi 与 BLE 的初始化顺序：必须【先 esp_wifi_start()，再拉起 BT controller】。
        //   根因（实测 coex_hook_check_wifi_sleep 野指针 LoadProhibited）：ESP32-S3 的
        //   WiFi/BT 软件共存(coexist)在 BT controller 使能中断时会回调 coexist 钩子去查
        //   WiFi 侧睡眠状态；而 WiFi 侧的 coexist 上下文要到 esp_wifi_start() 才建立完成。
        //   若 BT 先起、WiFi 未 start，BT enable 中断时 coexist 读到未初始化的 WiFi 侧
        //   数据结构 → 野指针崩溃（未配网分支必现，且清配网后无限重启）。故先 start WiFi
        //   把 coexist 上下文建好，BT 再 init。配网期 STA 不需要立刻连（等手机下发凭证），
        //   先 start 不影响配网逻辑。
        //   注意：本顺序只在【未配网分支】需要（此分支才同时开 WiFi+BT）；已配网分支不开
        //   BT、无 coexist，UI/GIF 可照常尽早显示，不受影响。
        ESP_ERROR_CHECK(esp_wifi_start());

        // 拉起 BT 控制器（NimBLE 下由该接口初始化 BLE 控制器）
#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
        esp_err_t bt_err = esp_blufi_controller_init();
        if (bt_err != ESP_OK)
        {
            ESP_LOGE(TAG, "BT 控制器初始化失败: %s，跳过配网直接重启", esp_err_to_name(bt_err));
            esp_restart();
        }
        PRINT_MEM_INFO(TAG, "esp_blufi_controller_init 后(BLE controller已起)");
#endif
        // 设置 MAC 派生蓝牙名（必须在 host_and_cb_init 之前，sync 回调里才能用正确的名字广播）
        blufi_set_device_name(service_name);
        // 注册 BluFi 回调 + 启动 NimBLE 主机（sync 回调里会设名字并 adv_start）
        esp_err_t blufi_err = esp_blufi_host_and_cb_init(&s_blufi_callbacks);
        if (blufi_err != ESP_OK)
        {
            ESP_LOGE(TAG, "BluFi 初始化失败: %s，重启", esp_err_to_name(blufi_err));
            esp_restart();
        }
        ESP_LOGI(TAG, "BluFi 配网就绪 → 蓝牙名: %s（手机用 EspBlufi / 微信小程序配网）", service_name);
        PRINT_MEM_INFO(TAG, "esp_blufi_host_and_cb_init 后(NimBLE host已起)");

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

        // ★向手机补发"配网成功"报告（GOT_IP 回调因 sys_evt 栈太小只置了标志）。
        //   必须在蓝牙释放【之前】发，否则 NimBLE 主机已 deinit、帧发不出去。此处运行
        //   在 bsp_board_wifi_main 大栈，安全。
        if (s_need_send_prov_report && s_blufi_ble_connected)
        {
            s_need_send_prov_report = false;
            wifi_mode_t mode;
            esp_wifi_get_mode(&mode);
            esp_blufi_send_wifi_conn_report(mode, ESP_BLUFI_STA_CONN_SUCCESS, 0, NULL);
            ESP_LOGI(TAG, "已向手机回报配网成功");
        }

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
        ESP_LOGW(TAG, "[DIAG] 进入已配网分支 @ %lld us", esp_timer_get_time());
        PRINT_MEM_INFO(TAG, "已配网分支入口(esp_wifi_start前)");

        // 加载上次连接成功的信道/BSSID 提示，让驱动定向连接而非全信道扫描
        // （实测全信道扫描耗时 ~2.4s；信道若已变化，esp_wifi 会自动回退全扫）
        load_ap_channel_hint();
        ESP_LOGW(TAG, "[DIAG] load_ap_channel_hint 返回 @ %lld us", esp_timer_get_time());

        // 启动 WiFi 驱动（触发 WIFI_EVENT_STA_START → esp_wifi_connect()）
        ESP_LOGW(TAG, "[DIAG] 调用 esp_wifi_start() 前 @ %lld us", esp_timer_get_time());
        ESP_ERROR_CHECK(esp_wifi_start());
        ESP_LOGW(TAG, "[DIAG] esp_wifi_start() 返回 @ %lld us", esp_timer_get_time());
        PRINT_MEM_INFO(TAG, "已配网分支(esp_wifi_start后)");
    }

    // ── 排查 pp.c:4384 / ppProcTxSecFrame 卡死看门狗：关闭 WiFi 省电模式 ────────
    // 日志曾出现 `wifi:pm start, type: 1`（Modem-sleep 已开）。某些 IDF 版本在 PS
    // 模式下 TX 安全帧（硬件加密）路径存在已知卡死 bug：连上后一旦开始上行发包，
    // ppTask 在 CPU0 上自旋不让出，IDLE0 饿死触发 task_wdt。此处强制 WIFI_PS_NONE
    // 关闭省电，用以证伪"省电 bug"这一原因。必须在 esp_wifi_start() 之后调用才生效。
    // 注意：关省电会增加平均功耗，若确认非省电引起，后续应改回 WIFI_PS_MIN_MODEM。
    {
        esp_err_t ps_err = esp_wifi_set_ps(WIFI_PS_NONE);
        ESP_LOGW(TAG, "[排查] esp_wifi_set_ps(WIFI_PS_NONE) 返回: %s", esp_err_to_name(ps_err));
    }

    // ── 步骤 8：最终阻塞等待网络就绪或彻底失败 ──────────────────────────────
    // 调用方拿到函数返回即可认为网络状态已确定（要么 WIFI_BIT 置位，要么重启）
    ESP_LOGI(TAG, "等待 WiFi 连接完成...");
    ESP_LOGW(TAG, "[DIAG] 开始阻塞等待 WIFI_BIT @ %lld us", esp_timer_get_time());
    EventBits_t bits = xEventGroupWaitBits(
        bsp_board->board_status,
        WIFI_BIT | WIFI_FAIL_BIT, // 等待任一位被置位（OR 等待，pdFALSE）
        pdFALSE,                  // 不清除位（其他模块也可能等待 WIFI_BIT）
        pdFALSE,                  // OR 模式：任一位满足即返回（不需要两个都满足）
        portMAX_DELAY);           // 永久等待（超时由 WIFI_FAIL_BIT 分支处理重启）

    if (bits & WIFI_BIT)
    {
        ESP_LOGI(TAG, "WiFi 就绪，网络可用！");

        // ★在主流程大栈上补写信道提示（GOT_IP 回调因 sys_evt 栈太小只置了标志）。
        //   此处运行在 bsp_board_wifi_main 任务栈，NVS 写安全。幂等：仅标志置位时执行。
        if (s_need_save_channel_hint)
        {
            s_need_save_channel_hint = false;
            save_ap_channel_hint();
        }

        /* ★初次联网避让：此处是初次联网主流程的必经点（重连走 GOT_IP 回调+去抖，不重入
         *   本函数），故只会调用一次。开一个短窗口让 GIF 切图暂避联网突发，消除初次联网
         *   后 taskLVGL 被顶死引发的一次 task_wdt 误报。见 ui_notify_first_online 说明。 */
        ui_notify_first_online();
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
