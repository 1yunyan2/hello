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
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

/**
 * @brief OTA 进度状态枚举（对应后端 ota-status 主题的 status 字段）
 *
 * 时序：UPGRADING（更新中，progress 0~100，含下载+校验全过程）
 *     → SUCCESS（新固件重启、MQTT 重连成功后补发）
 * 任一环节出错则上报 FAILED（带中文 errorMsg），设备不重启、旧固件继续运行。
 */
typedef enum
{
    BSP_OTA_UPGRADING = 0, ///< 更新中（progress 有效，下载+校验全过程统一为此状态）
    BSP_OTA_SUCCESS,       ///< 升级成功（重启后补发）
    BSP_OTA_FAILED,        ///< 失败（带 errorMsg 上报，设备不重启）
} bsp_ota_status_t;

/**
 * @brief OTA 进度回调函数类型
 *
 * 在 ota_task 任务上下文中被调用（非 MQTT 事件回调栈，publish 安全）。
 * 实现方（mqtt_protocol）负责把状态发布到 echopal/device/{id}/ota-status。
 *
 * @param status   当前状态
 * @param progress 更新进度 0~100（UPGRADING 有效；SUCCESS 传 100；FAILED 传 0）
 * @param version  目标固件版本号
 * @param err_msg  失败原因（仅 FAILED 时有意义，中文描述；其余状态传 NULL）
 */
typedef void (*bsp_ota_progress_cb_t)(bsp_ota_status_t status, int progress,
                                      const char *version, const char *err_msg);

/**
 * @brief OTA 触发参数（由后端 command 指令解析而来）
 *
 * 注意：各指针通常指向 cJSON 内部字符串，bsp_ota_trigger 会在返回前
 *       把内容拷进任务私有堆参数，因此调用方返回后即可安全删除 cJSON。
 */
typedef struct
{
    const char *url;     ///< 固件下载地址（http:// 或 https://）
    const char *version; ///< 新固件版本号（语义化版本，如 "1.2.3"）
    const char *sha256;  ///< 固件 SHA256（64 位十六进制字符串）；为 NULL 则跳过校验
    uint32_t size;       ///< 固件字节数（progress 分母）；为 0 时退化用已下载长度估算
} bsp_ota_req_t;

/**
 * @brief 注册 OTA 进度回调（在 protocol_mqtt_start 中调用一次即可）
 *
 * @param cb 进度回调；传 NULL 可注销
 */
void bsp_ota_register_progress_cb(bsp_ota_progress_cb_t cb);

/**
 * @brief 触发 OTA 升级（异步，不阻塞调用方）
 *
 * 调用方在 MQTT 事件回调中调用本函数。内部创建独立任务执行下载，
 * 因此不会阻塞 MQTT 事件循环。
 *
 * 内部流程：
 *   1. 版本号比较：若 version <= 当前版本，直接返回 ESP_ERR_INVALID_VERSION
 *   2. 写入 NVS pending_ver / pending_url（断电恢复用）
 *   3. 分步下载（esp_https_ota_begin/perform）到备用分区，过程中回调 UPGRADING 进度
 *   4. 回读备用分区计算 SHA256 与 req->sha256 比对，不一致则丢弃、不重启
 *   5. 校验通过 → 回调 UPGRADING(100) → NVS 记录"待上报 success" → 延迟 3 秒 → esp_restart()
 *
 * @param req 触发参数（见 bsp_ota_req_t）
 * @return    ESP_OK 任务创建成功；ESP_ERR_INVALID_ARG / NO_MEM / INVALID_VERSION / FAIL
 */
esp_err_t bsp_ota_trigger(const bsp_ota_req_t *req);

/**
 * @brief 取出"升级成功待上报"标记（新固件重启后调用）
 *
 * 若上一轮升级成功并已重启，本函数返回 true 并把目标版本号写入 out_version；
 * 调用方据此向后端补发 success，随后应调用 bsp_ota_clear_pending_success()。
 *
 * @param out_version 输出缓冲区（建议 ≥32 字节）
 * @param out_size    缓冲区大小
 * @return true 存在待上报标记；false 无
 */
bool bsp_ota_take_pending_success(char *out_version, size_t out_size);

/**
 * @brief 清除"升级成功待上报"标记（success 上报成功后调用）
 */
void bsp_ota_clear_pending_success(void);

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
