#include "application.h"

static const char *TAG = "application";

// 🌟 当识别到唤醒词时，底层会自动调用此函数！
void my_wake_word_callback(const char *wake_word_pinyin)
{
    ESP_LOGW("WAKE_UP", "🎯 卧槽！唤醒成功！听到了名字: [%s]", wake_word_pinyin);
    // 后续我们可以在这里触发录音任务，发往科大讯飞大模型
    // 注意：未来如果你接了大模型，这句 start 要等大模型播放完语音后再调用
    custom_wake_word_start();
}

void application_init(void)
{

    // 🌟 1. 必须优先初始化 NVS（因为引擎初始化需要去 NVS 读记忆的唤醒词）
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }

    // init_spiffs(); // 初始化文件系统

    // 🌟 1. 初始化唤醒词引擎（目前不设回调，仅初始化）
    esp_err_t err = custom_wake_word_init(NULL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "唤醒词引擎初始化失败，请检查模型文件是否在 model 分区");
    }

    // 3. 启动 ES8311 麦克风，开启音频采集任务
    audio_init();
    // 🌟 修复大坑二：栈内存扩大到 8192，并绑定到 CPU 核心 1 专职运算语音
    xTaskCreatePinnedToCore(audio_feed_task, "audio_feed", 8192, NULL, 5, NULL, 1);
    // 初始化 WiFi 网络
    bsp_board_wifi_main();

    // 拿到 IP 后再启动 MQTT
    mqtt_app_start();
}
