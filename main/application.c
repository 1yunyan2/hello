#include "application.h"

static const char *TAG = "application";

// 专门用来初始化 SPIFFS 仓库的函数
void init_spiffs(void)
{
    ESP_LOGI(TAG, "正在初始化 SPIFFS 文件系统...");
    esp_vfs_spiffs_conf_t conf = {
        .base_path = "/model",        // 我们把仓库的门牌号命名为 /model
        .partition_label = "storage", // 对应 partitions.csv 里的 storage
        .max_files = 5,
        .format_if_mount_failed = true // 第一次跑会自动格式化，不用怕
    };

    esp_err_t ret = esp_vfs_spiffs_register(&conf);

    if (ret == ESP_OK)
    {
        size_t total = 0, used = 0;
        esp_spiffs_info(conf.partition_label, &total, &used);
        ESP_LOGI(TAG, "✅ SPIFFS 仓库挂载成功！总空间: %d bytes, 已用: %d bytes", total, used);
    }
    else
    {
        ESP_LOGE(TAG, "❌ SPIFFS 挂载失败 (%s)", esp_err_to_name(ret));
    }
}
void application_init(void)
{
    // 🌟 1. 初始化唤醒词引擎（目前不设回调，仅初始化）
    esp_err_t err = custom_wake_word_init(NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "唤醒词引擎初始化失败，请检查模型文件是否在 storage 分区");
    }
    // i2s_init();
    // xTaskCreate(play_audio_task, "play_audio", 4096, NULL, 5, NULL);

    // init_spiffs();
    // 初始化 WiFi 网络
    wifi_main();

    // 🌟 在这里插入！确保拿到 IP 后再启动 MQTT
    mqtt_app_start();
}
