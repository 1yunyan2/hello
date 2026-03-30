#include "application.h"

static const char *TAG = "application";

// 🌟 当识别到唤醒词时，底层会自动调用此函数！
void my_wake_word_callback(const char *wake_word_pinyin)
{
    ESP_LOGW("WAKE_UP", "我的唤醒词为：[%s]", wake_word_pinyin);
    // 后续我们可以在这里触发录音任务，发往科大讯飞大模型
    // 注意：未来如果你接了大模型，这句 start 要等大模型播放完语音后再调用

    custom_wake_word_start();
}

void application_init(void)
{

    /*
// API调用链：
bsp_board_nvs_init()                    // 初始化NVS Flash
├── ESP_ERR_NVS_NO_FREE_PAGES       // 错误处理分支
│   └── nvs_flash_erase()           // 擦除NVS分区
│       └── nvs_flash_init()        // 重新初始化
└── ESP_ERR_NVS_NEW_VERSION_FOUND   // 版本不匹配处理
└── nvs*/

    bsp_board_nvs_init();

    //  1. 初始化唤醒词引擎
    esp_err_t err = custom_wake_word_init(my_wake_word_callback);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "唤醒词引擎初始化失败");
    }

    // 3. 启动 ES8311 麦克风，开启音频采集任务
    audio_init();

    // 初始化 WiFi 网络
    bsp_board_wifi_main();

    // 拿到 IP 后再启动 MQTT
    mqtt_app_start();
}
