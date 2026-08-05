/**
 * @file udp_logger.c
 * @brief UDP 日志单播实现（见 udp_logger.h 说明）
 */

#include "udp_logger.h"
#include "esp_log.h"
#include "esp_system.h"    // esp_reset_reason()：读取上次复位原因（RTC 寄存器），UDP 抓不到串口/bootloader 日志时用它定位重启
#include "esp_heap_caps.h" // xTaskCreatePinnedToCoreWithCaps：初始化任务栈放 SPIRAM，不占调用者栈
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h> // va_list/va_copy/va_end
#include <errno.h>  // errno（初始化失败诊断日志用）
#include "lwip/sockets.h"
#include "lwip/netdb.h"
#include "object.h" // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM

#define UDP_LOGGER_DEFAULT_PORT 3333
// ★默认改回【单播】到开发电脑固定 IP：广播（x.x.x.255）虽免维护 IP，但每包都走
//   完整发送路径、无法被 ARP 缓存复用，对 lwIP 负担明显更大。实测在启动/低功耗切换
//   等日志爆发期，广播灌包会冲垮与 WebSocket 共用的 lwIP tcpip_task 收发管道
//   （UDP/TCP RECVMBOX 各仅 6），导致 WS 长连接握手不上或时断时续（见 2026-07-14 排查）。
//   单播只发给这一台电脑，路由器不广播、lwIP 可复用 ARP，压力小得多。
//   代价：换网/电脑 IP 变了需改这里重烧（配合发送任务节流，是当前最稳的组合）。
//   电脑端 tools/udp_log_listen.py 监听同端口即可（它 bind 0.0.0.0，单播/广播都能收）。
//   若需临时收广播，可由调用方给 udp_logger_start() 传具体地址覆盖本默认值。
// ★2026-07-29 更新为当前开发电脑 WLAN 实际 IP（原 .243 已失效）。换网/换机必须同步改这里重烧，
//   否则包全发到不存在的地址且 UDP 是 fire-and-forget，电脑侧完全静默、设备侧也不报错。
#define UDP_LOGGER_DEFAULT_DEST_IP "192.168.1.245"
#define UDP_LOGGER_BUF_SIZE 512 // 单条日志最大长度，超出截断（够用；ESP_LOG 单行一般 <200B）
#define UDP_LOGGER_QUEUE_LEN 32 // 发送队列深度：突发日志（如启动阶段）允许暂存这么多条，超出直接丢弃最旧的

static int s_sock = -1;
static struct sockaddr_in s_dest_addr;
static vprintf_like_t s_orig_vprintf = NULL; // 原始输出函数（一般走 UART），保留以便同时输出
static bool s_started = false;
static QueueHandle_t s_log_queue = NULL; // udp_logger_vprintf 只入队，真正的 sendto 由 udp_logger_send_task 做

// ★队列里传递定长结构体（而非指针）：调用者栈上的 va_list 格式化结果必须在入队前
//   拷贝成值语义，发送任务才能在调用者返回之后仍安全读取。
typedef struct
{
    uint16_t len;
    char data[UDP_LOGGER_BUF_SIZE];
} udp_log_item_t;

// ★队列本体（约 32×518B≈16.5KB）改用 xQueueCreateStatic + PSRAM 静态存储区，
//   避免 xQueueCreate 默认从内部 SRAM 分配这一整块。队列本身只是数据搬运，不涉及
//   Flash/NVS 操作，PSRAM 存储区在这里没有 BUG-010 那类 cache 关闭期不可访问的风险。
static StaticQueue_t s_log_queue_struct;
static uint8_t *s_log_queue_storage = NULL; // heap_caps_malloc(..., MALLOC_CAP_SPIRAM)

static const char *TAG = "UDP_LOGGER";

// ★启动宽限期（毫秒）：接管日志后的头这么长时间内，udp_logger_vprintf 只把日志
//   写 UART、【不入队发 UDP】。原因：DNS 查询、WS/MQTT 建连都走 UDP/网络，而它们
//   与 UDP 日志出包争用同一个 lwIP tcpip_task 和 UDP 收发 mbox（UDP_RECVMBOX_SIZE=6）。
//   系统启动阶段日志爆发（GIF/PCM/待机刷屏），若同时全速灌 UDP 出包，会把 DNS 应答
//   挤不进 mbox → getaddrinfo() 超时返回 202 → 域名解析失败 → WS 连不上。实测关掉
//   UDP 日志则一切正常，证明就是 UDP 出包挤占了 DNS 的 UDP 通道。故在建连最脆弱的
//   启动期让 UDP 完全让路（这段日志插 USB 用 UART 照样看得全），等网络连稳后再放行。
#define UDP_LOGGER_STARTUP_GRACE_MS 20000
static uint32_t s_grace_until_tick = 0; // 达到此 tick 之前不发 UDP（0=尚未接管）

/**
 * @brief 替换 esp_log 的 vprintf：只管格式化 + 入队，不碰 socket
 *
 * ★为何改成队列而非直接在本函数里调 sendto（历史版本的做法）：直接发送需要用
 *   互斥锁串行化对 socket 的访问，而 sendto 在某些时刻（网络层繁忙/lwIP 内部
 *   阻塞）即使设了 O_NONBLOCK 也可能耗时变长；调用者持锁期间变慢，会导致其他
 *   任务的 ESP_LOG 抢不到锁而阻塞或跳过 UDP，且随着系统日志频率上升锁竞争持续
 *   恶化（实测抢锁失败率从 0 一路涨到 40%+），最终表现为“运行一段后 UDP 日志
 *   彻底不再更新”。改为入队后，本函数只做 vsnprintf（纯 CPU，无 IO/无锁等待）
 *   + xQueueSend（极短超时，队列满就丢弃这一条，不阻塞调用者），无论网络/socket
 *   状态如何，都不会拖慢调用 ESP_LOG 的业务线程。
 *
 * @note 运行在调用 ESP_LOG* 的任意任务上下文（包括 sys_evt 等小栈系统任务），
 *       必须快、不可长阻塞、且不可在栈上分配大对象——本函数只用栈上的定长结构体
 *       （由编译器决定是否需要额外栈空间，UDP_LOGGER_BUF_SIZE=512B 在此前
 *       静态缓冲区方案里已验证安全）拷贝后立即入队，不持有跨越 IO 的锁。
 */
static int udp_logger_vprintf(const char *fmt, va_list args)
{
    // ★启动宽限期内不入队发 UDP：让 DNS/WS 建连的 UDP/网络流量独占 lwIP 通道，
    //   避免 UDP 日志出包挤占 DNS 应答 mbox 导致 getaddrinfo 202、WS 连不上。
    //   s_grace_until_tick==0 表示尚未接管日志（发送任务还没设置基准），也不发。
    bool in_grace = (s_grace_until_tick == 0) ||
                    ((int32_t)(xTaskGetTickCount() - s_grace_until_tick) < 0);

    if (s_log_queue != NULL && !in_grace)
    {
        udp_log_item_t item;
        int len = vsnprintf(item.data, sizeof(item.data), fmt, args);
        if (len > 0)
        {
            if (len >= (int)sizeof(item.data))
                len = sizeof(item.data) - 1; // 截断，避免越界
            item.len = (uint16_t)len;
            // 不等待：队列满说明发送任务跟不上（罕见突发），丢弃这一条日志，
            // 不能阻塞调用者——调用者可能是任何任务，包括高优先级的音频/协议任务。
            xQueueSend(s_log_queue, &item, 0);
        }
    }

    // 再走原输出（UART）：即使这步阻塞/变慢，UDP 那份已经先入队，不受影响。
    if (s_orig_vprintf != NULL)
        return s_orig_vprintf(fmt, args);
    return 0;
}

/**
 * @brief UDP 发送任务：唯一持有/操作 socket 的地方，从队列取日志逐条发送
 *
 * ★所有 socket 相关操作（创建、sendto、失败重建、关闭）都收敛到这一个任务里
 *   串行执行，天然不存在跨任务竞争，不再需要互斥锁保护 s_sock。即使某次 sendto
 *   变慢/阻塞，也只影响这个专用任务本身的发送节奏（顶多日志攒在队列里稍晚发出），
 *   不会拖慢任何调用 ESP_LOG 的业务线程。
 */
static void udp_logger_send_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    const char *dest_ip = (const char *)arg;
    uint16_t port = UDP_LOGGER_DEFAULT_PORT;

    ESP_LOGI(TAG, "初始化任务启动，目标=%s:%u", dest_ip, port);

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    if (sock < 0)
    {
        ESP_LOGE(TAG, "socket() 创建失败，errno=%d，放弃启动", errno);
        vTaskDelete(NULL);
        return;
    }

    // ★关键修复（此前重写为队列方案时误删）：UDP socket 默认是【阻塞】的，sendto
    //   在 ARP 解析中/发送缓冲区满/网络状态异常等情况下会实际阻塞——即使把 sendto
    //   放进了独立任务，一旦某次 sendto 永久阻塞（比如目标不可达、无人消费的
    //   ARP 请求），这个任务会卡死在第一次 sendto 里再也出不来，队列后续日志全部
    //   堆积到满后被静默丢弃，表现为"启动成功打印了一次，之后再无任何 UDP 日志"。
    //   显式设 O_NONBLOCK，sendto 发不出去立即返回 EWOULDBLOCK 而不阻塞。
    int flags = fcntl(sock, F_GETFL, 0);
    if (fcntl(sock, F_SETFL, flags | O_NONBLOCK) < 0)
        ESP_LOGW(TAG, "设置 O_NONBLOCK 失败，errno=%d（sendto 仍可能阻塞）", errno);

    // ★开启广播权限：目标为 255.255.255.255（或子网广播 x.x.x.255）时，socket 必须
    //   显式设置 SO_BROADCAST，否则 sendto 到广播地址会返回 EACCES 失败，日志发不出去。
    //   单播目标不需要此选项，但设了也无副作用，故无条件开启，兼容调用方传单播 IP 的情况。
    int broadcast_enable = 1;
    if (setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast_enable, sizeof(broadcast_enable)) < 0)
        ESP_LOGW(TAG, "设置 SO_BROADCAST 失败，errno=%d（广播地址将发送失败）", errno);

    struct sockaddr_in dest_addr;
    memset(&dest_addr, 0, sizeof(dest_addr));
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(port);
    if (inet_pton(AF_INET, dest_ip, &dest_addr.sin_addr) != 1)
    {
        ESP_LOGE(TAG, "inet_pton 解析目标IP失败: %s，放弃启动", dest_ip);
        close(sock);
        vTaskDelete(NULL);
        return;
    }

    s_dest_addr = dest_addr;
    s_sock = sock;
    // ★重建场景（udp_logger_stop 后再次 start）不能重复接管：esp_log_set_vprintf
    //   若再次传入 udp_logger_vprintf，会把"当前正在生效的 udp_logger_vprintf 自己"
    //   存进 s_orig_vprintf，造成自调用死循环。只在首次接管。
    if (s_orig_vprintf == NULL)
        s_orig_vprintf = esp_log_set_vprintf(udp_logger_vprintf);
    // ★设置启动宽限期基准：从现在（接管日志的瞬间）起，头 UDP_LOGGER_STARTUP_GRACE_MS
    //   毫秒内 udp_logger_vprintf 只写 UART 不发 UDP，把网络让给 DNS/WS 建连。放在接管
    //   之后设置，确保 in_grace 判断从接管起才生效（此前 s_grace_until_tick==0 也不发）。
    {
        uint32_t now = xTaskGetTickCount();
        s_grace_until_tick = now + pdMS_TO_TICKS(UDP_LOGGER_STARTUP_GRACE_MS);
        if (s_grace_until_tick == 0) // 极罕见回绕到 0，避开"0=未接管"语义
            s_grace_until_tick = 1;
    }
    s_started = true;
    ESP_LOGI(TAG, "启动完成，sock=%d，开始接管日志输出（前 %d ms 宽限期只走UART，让路给DNS/WS建连）",
             sock, UDP_LOGGER_STARTUP_GRACE_MS);

    // ★复位原因诊断：UDP 抓不到 bootloader 的 rst:0x.. 与 "Brownout detector was triggered"
    //   （那些走 UART0），故在 UDP 接管日志后，用 esp_reset_reason() 补打上一次复位原因。
    //   该值读自 RTC 寄存器，任何时机调用都一致。用于定位低功耗切换瞬间的偶发重启：
    //   ESP_RST_BROWNOUT=欠压/负载冲击掉压；ESP_RST_TASK_WDT/INT_WDT=看门狗；ESP_RST_PANIC=软件崩溃。
    esp_reset_reason_t rst = esp_reset_reason();
    const char *rst_str = "未知";
    switch (rst)
    {
    case ESP_RST_POWERON:
        rst_str = "POWERON(上电)";
        break;
    case ESP_RST_SW:
        rst_str = "SW(软件esp_restart)";
        break;
    case ESP_RST_PANIC:
        rst_str = "PANIC(软件崩溃)";
        break;
    case ESP_RST_INT_WDT:
        rst_str = "INT_WDT(中断看门狗)";
        break;
    case ESP_RST_TASK_WDT:
        rst_str = "TASK_WDT(任务看门狗)";
        break;
    case ESP_RST_WDT:
        rst_str = "WDT(其他看门狗)";
        break;
    case ESP_RST_BROWNOUT:
        rst_str = "BROWNOUT(欠压复位)";
        break;
    case ESP_RST_DEEPSLEEP:
        rst_str = "DEEPSLEEP(深睡唤醒)";
        break;
    case ESP_RST_EXT:
        rst_str = "EXT(外部复位)";
        break;
    default:
        break;
    }
    ESP_LOGW(TAG, "⚡ 上次复位原因: %s (reason=%d)", rst_str, (int)rst);

    udp_log_item_t item;
    for (;;)
    {
        // 阻塞等待队列有数据：本任务优先级低、独占 socket 操作，阻塞不影响任何人。
        if (xQueueReceive(s_log_queue, &item, portMAX_DELAY) != pdTRUE)
            continue;
        if (s_sock < 0)
            continue; // udp_logger_stop() 已关闭 socket，丢弃剩余队列内容
        // ★排查结论：sendto 失败绝大多数是 errno=12(ENOMEM)——系统启动阶段
        //   （AFE/唤醒词模型/MQTT/WebSocket 密集初始化）internal RAM 一度探底到
        //   1~2 万字节，lwIP 内部分配 pbuf 失败，包根本没上网卡，与网络/目标IP
        //   无关。此处短暂让出 CPU 后重试几次：多数情况下几毫秒后其他任务已释放
        //   内存，重试即可发出去；仍失败就丢弃这一条，不无限重试拖慢发送任务。
        int sent = -1;
        for (int attempt = 0; attempt < 3; attempt++)
        {
            sent = sendto(s_sock, item.data, item.len, 0, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
            if (sent >= 0 || errno != ENOMEM)
                break;
            vTaskDelay(pdMS_TO_TICKS(2)); // 仅 ENOMEM 重试：等其他任务释放内存后再试
        }

        // ★发送节流：每处理一条日志后主动让出 2ms，把 UDP 出包速率压平，避免在
        //   启动/低功耗切换等日志爆发期，独立发送任务全速灌包冲垮与 WebSocket 共用的
        //   lwIP tcpip_task 收发管道（UDP/TCP RECVMBOX 各仅 6），导致 WS 长连接握手
        //   不上或时断时续。宁可日志晚几毫秒发出，也不与业务 TCP 抢瞬时带宽。
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void udp_logger_start(const char *dest_ip, uint16_t port)
{
    (void)port; // 端口固定用 UDP_LOGGER_DEFAULT_PORT（当前不支持自定义端口，保持接口简单）
    if (s_started)
        return;
    if (dest_ip == NULL || dest_ip[0] == '\0')
        dest_ip = UDP_LOGGER_DEFAULT_DEST_IP;

    if (s_log_queue == NULL)
    {
        s_log_queue_storage = heap_caps_malloc(UDP_LOGGER_QUEUE_LEN * sizeof(udp_log_item_t),
                                               MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
        if (s_log_queue_storage == NULL)
            return; // PSRAM 不足：放弃启动，日志仍走原路径
        s_log_queue = xQueueCreateStatic(UDP_LOGGER_QUEUE_LEN, sizeof(udp_log_item_t),
                                         s_log_queue_storage, &s_log_queue_struct);
        if (s_log_queue == NULL)
        {
            heap_caps_free(s_log_queue_storage);
            s_log_queue_storage = NULL;
            return; // 内存不足：放弃启动，日志仍走原路径
        }
    }

    // ★不在当前上下文（可能是 sys_evt 等小栈任务）直接创建 socket，改投递到独立任务执行。
    //   dest_ip 若来自调用方传参需保证生命周期；当前调用点固定传 NULL（用默认值，
    //   指向 .rodata 常量字符串），生命周期天然安全。
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        udp_logger_send_task, "udp_log_send", 4096,
        (void *)dest_ip, tskIDLE_PRIORITY + 1, NULL,
        tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ret == pdPASS)
        PRINT_TASK_CREATED(TAG, "udp_log_send", 4096, 0); // 首选栈在PSRAM
    if (ret != pdPASS)
    {
        ESP_LOGW(TAG, "SPIRAM 建发送任务失败(ret=%d)，改用内部SRAM重试", (int)ret);
        ret = xTaskCreatePinnedToCoreWithCaps(
            udp_logger_send_task, "udp_log_send", 4096,
            (void *)dest_ip, tskIDLE_PRIORITY + 1, NULL,
            tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (ret == pdPASS)
            PRINT_TASK_CREATED(TAG, "udp_log_send", 4096, 1); // 回退栈在内部SRAM
        else
            ESP_LOGE(TAG, "内部SRAM建发送任务仍失败(ret=%d)，本次放弃启动UDP日志", (int)ret);
    }
}

void udp_logger_stop(void)
{
    if (!s_started)
        return;
    if (s_orig_vprintf != NULL)
    {
        esp_log_set_vprintf(s_orig_vprintf);
        s_orig_vprintf = NULL;
    }
    if (s_sock >= 0)
    {
        close(s_sock);
        s_sock = -1;
    }
    s_started = false;
}
