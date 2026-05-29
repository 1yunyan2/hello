# 把 servo_standalone 作为一个 ESP-IDF 组件注册
# 只依赖 ESP-IDF 自带的 driver（LEDC），无任何第三方托管组件
idf_component_register(
    SRCS "servo_driver.c"
    INCLUDE_DIRS "."
    REQUIRES driver
)
