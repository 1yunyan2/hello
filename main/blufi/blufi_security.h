/**
 * @file blufi_security.h
 * @brief BluFi 配网的安全协商 + NimBLE 主机初始化接口声明
 *
 * 本文件 / blufi_security.c / blufi_init.c 三者整体来源于 ESP-IDF 官方示例：
 *   $IDF_PATH/examples/bluetooth/blufi/main/
 * 因为 NimBLE 下 BluFi 的「主机初始化」与「DH/AES 安全协商」实现
 * 并未由 BluFi 组件直接对外导出，必须由应用工程自行拷贝集成。
 *
 * 原始许可证：Unlicense OR CC0-1.0（Espressif 示例代码，可自由使用）
 *
 * 提供两类接口：
 *   1. 安全协商：DH 密钥协商回调 + AES 加解密 + CRC 校验（注册进 esp_blufi_callbacks_t）
 *   2. 主机管理：NimBLE 主机/控制器的初始化与反初始化（供配网开始/结束时调用）
 */
#pragma once

#include "esp_err.h"
#include "esp_blufi_api.h"
#include "esp_log.h"

// ─── 统一日志宏（沿用示例命名，避免改动 .c 中大量调用点）───────────────────
#define BLUFI_EXAMPLE_TAG "BLUFI"
#define BLUFI_INFO(fmt, ...)  ESP_LOGI(BLUFI_EXAMPLE_TAG, fmt, ##__VA_ARGS__)
#define BLUFI_ERROR(fmt, ...) ESP_LOGE(BLUFI_EXAMPLE_TAG, fmt, ##__VA_ARGS__)

// ─── 安全协商接口（注册到 esp_blufi_callbacks_t）────────────────────────────
/// DH 密钥协商数据处理（手机与设备交换公钥，协商出 AES 会话密钥）
void blufi_dh_negotiate_data_handler(uint8_t *data, int len, uint8_t **output_data, int *output_len, bool *need_free);
/// 用协商出的密钥对 BluFi 帧做 AES-CFB 加密
int blufi_aes_encrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
/// 用协商出的密钥对 BluFi 帧做 AES-CFB 解密
int blufi_aes_decrypt(uint8_t iv8, uint8_t *crypt_data, int crypt_len);
/// BluFi 帧 CRC16 校验
uint16_t blufi_crc_checksum(uint8_t iv8, uint8_t *data, int len);

/// 申请并初始化安全上下文（在 BLE 连接建立时调用）
int blufi_security_init(void);
/// 释放安全上下文（在 BLE 断开时调用）
void blufi_security_deinit(void);

// ─── NimBLE 主机 / 控制器管理接口（blufi_init.c 实现）───────────────────────
int esp_blufi_gap_register_callback(void);
esp_err_t esp_blufi_host_init(void);
/// 注册 BluFi 回调 + 启动 NimBLE 主机（配网开始时调用）
esp_err_t esp_blufi_host_and_cb_init(esp_blufi_callbacks_t *callbacks);
/// 停止并释放 NimBLE 主机（配网结束/超时后调用，回收内存）
esp_err_t esp_blufi_host_deinit(void);
/// 初始化 BT 控制器（NimBLE 下由该接口拉起 BLE 控制器）
esp_err_t esp_blufi_controller_init(void);
/// 反初始化 BT 控制器（配网结束后释放 ~110KB 蓝牙内存）
esp_err_t esp_blufi_controller_deinit(void);
/// 设置蓝牙广播名，必须在 esp_blufi_host_and_cb_init 之前调用
void blufi_set_device_name(const char *name);
