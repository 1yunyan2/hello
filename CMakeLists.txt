# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Echo)
spiffs_create_partition_image(assets gif FLASH_IN_PROJECT)
# 替换掉原来的 spiffs 指令
# fatfs_create_spiflash_image(storage gif FLASH_IN_PROJECT)