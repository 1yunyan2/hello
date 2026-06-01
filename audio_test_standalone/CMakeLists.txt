# audio_test_standalone — 顶层项目 CMakeLists
#
# 独立音频测试工程，用于验证 ES8311 + AFE + 唤醒词的完整链路，
# 与主工程 Echo2 共享同一 ESP-IDF 环境，无需任何外部依赖。

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(audio_test_standalone)
