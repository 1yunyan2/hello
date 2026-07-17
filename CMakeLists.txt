# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

# ─── OTA 固件版本号 ────────────────────────────────────────────────────
# 该值会被嵌入到 esp_app_desc_t.version，运行时由 esp_app_get_description() 读取。
# bsp_ota.c 的 is_newer_version() 依赖此值做语义化版本比较（如 "1.0.0" < "1.0.1"）。
# 不设置时默认从 git tag/commit hash 生成，会导致版本比较失效。
# 升级流程：修改下行版本号 → idf.py build → 把 build/Echo.bin 放到 HTTP 服务器
set(PROJECT_VER "1.0.0")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Echo)
# spiffs_create_partition_image(assets assetsss/gif FLASH_IN_PROJECT)
