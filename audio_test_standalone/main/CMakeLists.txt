# audio_test_standalone — main 组件
#
# 文件清单（全部独立编写或仅含 ES8311 驱动代码，无业务依赖）：
#   main.c        — app_main + AFE 初始化 + 采集任务 + VAD 任务
#   bsp_board.c/h — BSP 单例 + NVS 初始化 + EventGroup 状态位
#   bsp_codec.c   — ES8311 I2C/I2S/Codec 硬件初始化
#   bsp_config.h  — GPIO 引脚宏定义
#
# REQUIRES：
#   driver        — GPIO / I2C / I2S 底层
#   esp_codec_dev — ES8311 Codec 抽象层
#   esp-sr        — AFE（NS + WebRTC VAD）
#   nvs_flash     — NVS 初始化（AFE 内部可能用到）
#   esp_event     — FreeRTOS EventGroup
#   esp_timer     — ESP_LOGI 时间戳
#   spiffs        — esp_srmodel_init 内部依赖（即使无模型文件也需链接）
idf_component_register(
    SRCS
        "main.c"
        "bsp_board.c"
        "bsp_codec.c"
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
