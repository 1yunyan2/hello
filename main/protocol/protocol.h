#include "mqtt_client.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "cJSON.h"
#include "wake_words/custom_wake_word.h"

// 初始化并启动 MQTT 客户端
void mqtt_app_start(void);