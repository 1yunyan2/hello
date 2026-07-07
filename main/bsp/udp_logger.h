#pragma once

/**
 * @file udp_logger.h
 * @brief 无线串口：把 ESP_LOG 日志同时通过 UDP 单播发到指定电脑
 *
 * 用途：设备纯锂电池供电（无 USB、无法接串口）时，仍能实时观察日志——尤其是
 *   低功耗（待机）进入/退出全过程。日志通过 WiFi UDP 直接发到开发电脑，电脑上
 *   用 tools/udp_log_listen.py 监听同一端口即可查看。
 *
 * ★为何用单播而非广播：受限广播 255.255.255.255 在很多路由器/AP 上会被丢弃或不
 *   转发（尤其开了 AP 隔离），设备网段定向广播（如 192.168.1.255）也依赖路由器
 *   配合，环境不确定时容易收不到。单播只依赖"设备和电脑在同一网段、能互通"，
 *   是最不挑环境的方案。
 *
 * 机制：esp_log_set_vprintf() 把日志输出函数替换为本模块的实现——本实现【先照常
 *   走原 UART 输出】（插着 USB 时串口仍能看），【再额外发一份 UDP 单播】。二者并存，
 *   互不影响。UDP 发送失败（未连网/socket 出错）时静默跳过，不阻塞、不刷屏。
 *
 * 生命周期：WiFi 拿到 IP 后调 udp_logger_start()（见 bsp_wifi.c GOT_IP 分支）。
 *   幂等：重复调用只初始化一次。断网期间发送失败被静默忽略，恢复后自动继续。
 */

#include <stdint.h>

/**
 * @brief 启动 UDP 日志单播（幂等）
 *
 * 创建 UDP socket 并用 esp_log_set_vprintf() 接管日志输出。必须在 WiFi 已拿到
 * IP（能发 UDP）之后调用。
 *
 * @param dest_ip   目标电脑的 IPv4 地址字符串（如 "192.168.1.252"）；传 NULL 或
 *                  空串则用编译期默认值 UDP_LOGGER_DEFAULT_DEST_IP（见 .c 文件顶部）。
 *                  若非 NULL，调用方须保证该字符串在初始化任务执行期间有效
 *                  （建议传 NULL 或 .rodata 常量字符串，不要传栈上局部变量）。
 * @param port      当前未使用（固定用 UDP_LOGGER_DEFAULT_PORT=3333），保留参数位为
 *                  以后支持自定义端口；传 0 即可。
 * @note 内部把真正的 socket 创建投递到独立任务执行（栈 4096B/SPIRAM），本函数本身
 *       只创建任务、几乎不占调用者栈——可安全在小栈上下文（如系统事件回调）调用。
 * @note 调用者：bsp_wifi.c wifi_ip_event_handler() 的 IP_EVENT_STA_GOT_IP 分支
 *       （该回调运行在 sys_evt 任务，栈仅 2304B，是本接口设计"不占调用者栈"的原因）。
 */
void udp_logger_start(const char *dest_ip, uint16_t port);

/**
 * @brief 停止 UDP 日志广播，恢复默认日志输出（关闭 socket）
 * @note 一般无需调用；保留用于调试或释放资源。
 */
void udp_logger_stop(void);
