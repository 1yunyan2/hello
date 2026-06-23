/*
 * BluFi 主机 / 控制器初始化（NimBLE 版）
 *
 * 来源：ESP-IDF 官方示例 examples/bluetooth/blufi/main/blufi_init.c
 * 许可证：Unlicense OR CC0-1.0（Espressif 示例，可自由使用）
 *
 * 修改点（相对原示例）：
 *   1. #include "blufi_example.h" → #include "blufi_security.h"
 *   2. 仅保留 CONFIG_BT_NIMBLE_ENABLED 分支（本工程固定 NimBLE，省去 Bluedroid 代码）
 *   3. BLUFI_DEVICE_NAME 由 bsp_wifi.c 在初始化前通过 ble_svc_gap_device_name_set
 *      设置为 MAC 派生名（EchoPals-AABBCC），此处给一个兜底默认名
 *
 * 这些函数提供 NimBLE 协议栈的拉起/释放：
 *   - esp_blufi_controller_init/deinit ：BT 控制器（底层射频）
 *   - esp_blufi_host_and_cb_init       ：注册回调 + 启动 NimBLE 主机
 *   - esp_blufi_host_deinit            ：停止主机并释放（配网完成后回收内存）
 */
#include <stdio.h>
#include "esp_err.h"
#include "esp_blufi_api.h"
#include "esp_log.h"
#include "esp_blufi.h"
#include "blufi_security.h"
#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
#include "esp_bt.h"
#endif

#ifdef CONFIG_BT_NIMBLE_ENABLED
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"
#include "console/console.h"

/// 蓝牙设备名，由 bsp_wifi.c 在 esp_blufi_host_and_cb_init 前调用 blufi_set_device_name() 设置
static char s_ble_device_name[32] = "EchoPals";

void blufi_set_device_name(const char *name)
{
    if (name) {
        snprintf(s_ble_device_name, sizeof(s_ble_device_name), "%s", name);
    }
}

void ble_store_config_init(void);

static void blufi_on_reset(int reason)
{
    MODLOG_DFLT(ERROR, "Resetting state; reason=%d\n", reason);
}

static void blufi_on_sync(void)
{
    ble_svc_gap_device_name_set(s_ble_device_name); // 同步完成后设置名字再广播
    esp_blufi_profile_init();
}

void bleprph_host_task(void *param)
{
    ESP_LOGI("BLUFI", "BLE Host Task Started");
    /* This function will return only when nimble_port_stop() is executed */
    nimble_port_run();

    nimble_port_freertos_deinit();
}

esp_err_t esp_blufi_host_init(void)
{
    esp_err_t err;
    err = esp_nimble_init();
    if (err) {
        BLUFI_ERROR("%s failed: %s\n", __func__, esp_err_to_name(err));
        return ESP_FAIL;
    }

    /* Initialize the NimBLE host configuration. */
    ble_hs_cfg.reset_cb = blufi_on_reset;
    ble_hs_cfg.sync_cb = blufi_on_sync;
    ble_hs_cfg.gatts_register_cb = esp_blufi_gatt_svr_register_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    ble_hs_cfg.sm_io_cap = 4;
    ble_hs_cfg.sm_sc = 0;

    int rc;
    rc = esp_blufi_gatt_svr_init();
    assert(rc == 0);

#if CONFIG_BT_NIMBLE_GAP_SERVICE
    /* Set the default device name. */
    rc = ble_svc_gap_device_name_set(BLUFI_DEVICE_NAME);
    assert(rc == 0);
#endif

    /* XXX Need to have template for store */
    ble_store_config_init();

    esp_blufi_btc_init();

    err = esp_nimble_enable(bleprph_host_task);
    if (err) {
        BLUFI_ERROR("%s failed: %s\n", __func__, esp_err_to_name(err));
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t esp_blufi_host_deinit(void)
{
    esp_err_t ret = ESP_OK;

    esp_blufi_gatt_svr_deinit();
    ret = nimble_port_stop();
    if (ret != ESP_OK) {
        return ret;
    }
    if (ret == 0) {
        esp_nimble_deinit();
    }

    ret = esp_blufi_profile_deinit();
    if (ret != ESP_OK) {
        return ret;
    }

    esp_blufi_btc_deinit();

    return ret;
}

esp_err_t esp_blufi_gap_register_callback(void)
{
    return ESP_OK;
}

esp_err_t esp_blufi_host_and_cb_init(esp_blufi_callbacks_t *callbacks)
{
    esp_err_t ret = ESP_OK;

    ret = esp_blufi_register_callbacks(callbacks);
    if (ret) {
        BLUFI_ERROR("%s blufi register failed, error code = %x\n", __func__, ret);
        return ret;
    }

    ret = esp_blufi_gap_register_callback();
    if (ret) {
        BLUFI_ERROR("%s gap register failed, error code = %x\n", __func__, ret);
        return ret;
    }

    ret = esp_blufi_host_init();
    if (ret) {
        BLUFI_ERROR("%s initialise host failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ret;
}

#endif /* CONFIG_BT_NIMBLE_ENABLED */

#if CONFIG_BT_CONTROLLER_ENABLED || !CONFIG_BT_NIMBLE_ENABLED
esp_err_t esp_blufi_controller_init(void)
{
    esp_err_t ret = ESP_OK;
#if CONFIG_IDF_TARGET_ESP32
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
#endif

    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        BLUFI_ERROR("%s initialize bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        BLUFI_ERROR("%s enable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }
    return ret;
}

esp_err_t esp_blufi_controller_deinit(void)
{
    esp_err_t ret = ESP_OK;
    ret = esp_bt_controller_disable();
    if (ret) {
        BLUFI_ERROR("%s disable bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bt_controller_deinit();
    if (ret) {
        BLUFI_ERROR("%s deinit bt controller failed: %s\n", __func__, esp_err_to_name(ret));
        return ret;
    }

    return ret;
}
#endif
