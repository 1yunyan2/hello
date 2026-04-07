#pragma once

/**
 * @file auth.h
 * @brief 设备认证模块头文件
 * 负责通过 deviceToken 换取 accessToken
 */

#define AUTH_LOGIN_URL "http://122.224.191.2:4888/api/auth/device-login"

typedef struct
{
    char *access_token; // 换取到的短效 WebSocket 令牌
} auth_t;

auth_t *auth_create(void);
void auth_destroy(auth_t *auth);

/**
 * @brief 执行登录获取 AccessToken
 * @param auth 认证实例指针
 * @param device_token NVS中缓存的长期凭证
 */
void auth_perform(auth_t *auth, const char *device_token);