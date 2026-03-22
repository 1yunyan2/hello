
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_log.h>
#include <nvs_flash.h>
#include <esp_wifi.h>
#include <esp_event.h>
#include <wifi_provisioning/manager.h>
#include <wifi_provisioning/scheme_ble.h>
#include "esp_http_client.h"
#include "esp_crt_bundle.h" // 用于 HTTPS 的根证书校验
#include "cJSON.h"

#include <string.h> // 🌟 新增头文件，因为后面用到了 strdup 和 strlen
#include "driver/gpio.h"

#include "esp_bt.h"
#include "freertos/event_groups.h"

void wifi_main();
