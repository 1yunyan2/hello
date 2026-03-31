#include "application.h"

void my_wake_word_callback(const char *wake_word_pinyin)
{
    ESP_LOGW("WAKE_UP", "我的唤醒词为：[%s]", wake_word_pinyin);
    // 发送事件给音频处理模块
    // 后续我们可以在这里触发录音任务，发往科大讯飞大模型
    // 注意：未来如果你接了大模型，这句 start 要等大模型播放完语音后再调用

    custom_wake_word_start();
}

void application_init(void)
{
    bsp_board_t *bsp_board = bsp_board_get_instance();

    /*
 API调用链：
bsp_board_nvs_init()                    // 初始化NVS Flash
├── ESP_ERR_NVS_NO_FREE_PAGES       // 错误处理分支
│   └── nvs_flash_erase()           // 擦除NVS分区
│       └── nvs_flash_init()        // 重新初始化
└── ESP_ERR_NVS_NEW_VERSION_FOUND   // 版本不匹配处理
└── nvs*/

    bsp_board_nvs_init(bsp_board);

    //  1. 初始化唤醒词引擎
    custom_wake_word_init(my_wake_word_callback);

    //! 待舍弃
    audio_init(bsp_board);

    // 初始化 WiFi 网络
    bsp_board_wifi_main(bsp_board);

    // 拿到 IP 后再启动 MQTT
    mqtt_app_start();
}
