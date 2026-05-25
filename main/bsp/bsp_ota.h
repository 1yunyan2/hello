#pragma once

/**
 * @file bsp_ota.h
 * @brief OTA 固件升级模块（混合方案）
 *
 * 设计要点：
 *   - 下载层：调用 ESP-IDF 官方 esp_https_ota() 组件，稳定可靠
 *   - 版本管理：借鉴工业级 Pending/Committed 两阶段提交
 *   - 防回滚：检测 PENDING_VERIFY 状态，调用 esp_ota_mark_app_valid_cancel_rollback()
 *
 * 触发流程：
 *   云端 MQTT → mqtt_protocol.c command 主题分支 → bsp_ota_trigger(url, version)
 *     → 异步任务 → esp_https_ota() → esp_restart()
 *
 * 启动验证：
 *   application_init() 末尾 → bsp_ota_mark_valid()
 *     → 检测 PENDING_VERIFY → 取消回滚 + NVS pending→committed
 *
 * 安全保证：
 *   - 下载中断：旧固件 ota_0 不受影响
 *   - 新固件崩溃：Bootloader 自动回滚
 *   - 版本回退：is_newer_version 拦截
 */

#include "esp_err.h"

/**
 * @brief 触发 OTA 升级（异步，不阻塞调用方）
 *
 * 调用方在 MQTT 事件回调中调用本函数。内部创建独立任务执行下载，
 * 因此不会阻塞 MQTT 事件循环。
 *
 * 内部流程：
 *   1. 版本号比较：若 version <= 当前版本，直接返回 ESP_ERR_INVALID_VERSION
 *   2. 写入 NVS pending_ver / pending_url（断电恢复用）
 *   3. 调用 esp_https_ota() 下载到 ota_1 分区
 *   4. 下载成功后延迟 3 秒 → esp_restart()
 *
 * @param url     固件下载地址（http:// 或 https://）
 * @param version 新固件版本号（语义化版本，如 "1.2.3"）
 * @return        ESP_OK 任务创建成功；ESP_ERR_INVALID_ARG / NO_MEM / INVALID_VERSION / FAIL
 */
esp_err_t bsp_ota_trigger(const char *url, const char *version);

/**
 * @brief 标记当前固件为有效（必须在 application_init 末尾调用）
 *
 * 功能：
 *   1. 检测当前固件分区状态是否为 ESP_OTA_IMG_PENDING_VERIFY
 *   2. 若是（说明刚 OTA 升级完首次启动），调用 esp_ota_mark_app_valid_cancel_rollback()
 *   3. 把 NVS 中的 pending_ver 提升为 committed_ver
 *
 * 若新固件启动后未调用本函数（例如崩溃在 init 前期），
 * Bootloader 会在下次启动时检测到 PENDING_VERIFY 未提交，自动回滚到旧固件。
 */
void bsp_ota_mark_valid(void);

/**
 * @brief 获取当前运行固件的版本号
 *
 * 从 esp_app_get_description() 读取，对应根 CMakeLists.txt 中的 PROJECT_VER 宏。
 *
 * @return 版本号字符串（如 "1.0.0"），永不为 NULL
 */
const char *bsp_ota_get_current_version(void);
