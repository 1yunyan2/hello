# audio_test_standalone — main 组件
#
# 源文件全部直接放在 main/ 目录下，从主工程提取：
#   bsp_board.c      ← main/bsp/bsp_board.c
#   bsp_codec.c      ← main/bsp/bsp_codec.c
#   custom_wake_word.c ← main/wake_word/custom_wake_word.c
#   object.h         ← main/object.h
#   bsp_config.h     ← main/bsp/bsp_config.h
#   bsp_board.h      ← 自定义精简版（去掉 LCD/舵机/wifi/电池等无关声明）
#   main.c           ← 新写：调启动链 + 注册 VAD 回调打印
#
# REQUIRES：
#   driver           — GPIO/I2C/I2S 底层
#   esp_codec_dev    — ES8311 Codec 抽象层
#   esp-sr           — AFE（NS/AEC/VAD），不使用 MultiNet
#   nvs_flash        — NVS（custom_wake_word 用来读唤醒词配置，即使不用也得初始化）
#   esp_event        — FreeRTOS EventGroup
#   esp_timer        — ESP_LOGI 时间戳
#   spiffs           — esp_srmodel_init 内部依赖（即使无模型文件也需链接）
idf_component_register(
    SRCS
        "main.c"
        "bsp_board.c"
        "bsp_codec.c"
        "custom_wake_word.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_codec_dev
        esp-sr
        nvs_flash
        esp_event
        esp_timer
        spiffs
)
