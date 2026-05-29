# main 组件注册
# - SRCS：参与编译的源文件
# - INCLUDE_DIRS：头文件搜索路径
# - REQUIRES：依赖的 ESP-IDF 组件（driver=GPIO/SPI，esp_lcd=LCD 面板驱动）
idf_component_register(
    SRCS
        "main.c"
        "lcd_demo.c"
    INCLUDE_DIRS "."
    REQUIRES driver esp_lcd
)
