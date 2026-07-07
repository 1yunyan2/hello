/**
 * @file udp_logger.c
 * @brief UDP 日志单播实现（见 udp_logger.h 说明）
 */

#include "udp_logger.h"
#include "esp_log.h"
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

#define UDP_LOGGER_DEFAULT_PORT 3333
// ★开发电脑默认 IP：改成你自己电脑的局域网 IPv4（ipconfig 里 WLAN 的 IPv4 地址）。
//   若设备/电脑换了网络导致 IP 变化，改这里重新烧录，或调用方传参覆盖。
#define UDP_LOGGER_DEFAULT_DEST_IP "192.168.1.252"
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

static const char *TAG = "UDP_LOGGER";

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
    if (s_log_queue != NULL)
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
    s_started = true;
    ESP_LOGI(TAG, "启动完成，sock=%d，开始接管日志输出", sock);

    udp_log_item_t item;
    for (;;)
    {
        // 阻塞等待队列有数据：本任务优先级低、独占 socket 操作，阻塞不影响任何人。
        if (xQueueReceive(s_log_queue, &item, portMAX_DELAY) != pdTRUE)
            continue;
        if (s_sock < 0)
            continue; // udp_logger_stop() 已关闭 socket，丢弃剩余队列内容
        // sendto 失败（网络异常/发送队列满等）静默丢弃即可：只影响本任务自己，
        // 不占用、不拖慢任何业务线程，不需要像旧版那样为了避免"拖累调用者"而
        // 设置连续失败重建 socket——网络恢复后 sendto 自然会恢复成功。
        sendto(s_sock, item.data, item.len, 0, (struct sockaddr *)&s_dest_addr, sizeof(s_dest_addr));
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
        s_log_queue = xQueueCreate(UDP_LOGGER_QUEUE_LEN, sizeof(udp_log_item_t));
        if (s_log_queue == NULL)
            return; // 内存不足：放弃启动，日志仍走原路径
    }

    // ★不在当前上下文（可能是 sys_evt 等小栈任务）直接创建 socket，改投递到独立任务执行。
    //   dest_ip 若来自调用方传参需保证生命周期；当前调用点固定传 NULL（用默认值，
    //   指向 .rodata 常量字符串），生命周期天然安全。
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        udp_logger_send_task, "udp_log_send", 4096,
        (void *)dest_ip, tskIDLE_PRIORITY + 1, NULL,
        tskNO_AFFINITY, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (ret != pdPASS)
    {
        ESP_LOGW(TAG, "SPIRAM 建发送任务失败(ret=%d)，改用内部SRAM重试", (int)ret);
        ret = xTaskCreatePinnedToCoreWithCaps(
            udp_logger_send_task, "udp_log_send", 4096,
            (void *)dest_ip, tskIDLE_PRIORITY + 1, NULL,
            tskNO_AFFINITY, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
        if (ret != pdPASS)
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
