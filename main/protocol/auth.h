#pragma once
/**
 * @file auth.h
 * @brief 设备认证模块头文件 — 负责交换 WebSocket 令牌
 * * 职责：定义通过长期 deviceToken 换取短效 accessToken 的接口及数据结构。
 */

#include <stdbool.h>

/** @brief 请求认证令牌的后端 HTTP API 地址 */
// #define AUTH_LOGIN_URL "http://122.224.191.2:4889/api/auth/device-login"
#define AUTH_LOGIN_URL "https://api.strailine-space.com/api/auth/device-login"

/**
 * @brief 认证实体结构体
 * 仅对外暴漏 access_token 成员，隐藏内部实现细节（如 HTTP 缓冲等）
 */
typedef struct
{
    char *access_token; ///< 换取到的短效 WebSocket Bearer 令牌字符串
} auth_t;

/**
 * @brief 创建认证实例
 * * @param 无
 * @return auth_t* 创建好的认证实例指针
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
auth_t *auth_create(void);

/**
 * @brief 销毁认证实例并释放内存
 * * @param[in] auth 待销毁的实例指针
 * @return 无
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
void auth_destroy(auth_t *auth);

/**
 * @brief 发起 HTTP 请求，执行登录获取 AccessToken
 * * @param[in,out] auth         认证实例指针（操作结果存储在内）
 * @param[in]     device_token NVS 中缓存的长期设备凭证
 * @return 无
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
void auth_perform(auth_t *auth, const char *device_token);

/**
 * @brief 查询服务器是否可达（Auth HTTP 最近一次是否成功）
 *
 * 用于 session/mqtt 在发起连接前快速判断服务器是否可达，
 * 避免在服务器不可达时白白消耗内部 SRAM 建立 TLS 连接。
 *
 * @return true  服务器可达（最近一次 Auth 成功，或尚未尝试）
 * @return false 服务器不可达（最近一次 Auth 超时/失败）
 *
 * @note 调用者：session.c -> session_init(), session_reconnect_task()
 */
bool auth_is_server_reachable(void);