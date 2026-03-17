
#include "WIFI/WIFI.h"

void app_main(void) {
    // i2s_init();
    // xTaskCreate(play_audio_task, "play_audio", 4096, NULL, 5, NULL);

        // 初始化 WiFi 网络
    char payload[150] = {0};
    wifi_main( payload, sizeof(payload));

}