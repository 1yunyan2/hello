#pragma once

/**
 * @file object.h
 * @brief 通用内存分配工具
 * 提供从 SPIRAM（外部 PSRAM）分配并清零内存的快捷函数，
 * 替代标准 malloc + memset 组合，全局各模块均可使用。
 *
 * 为什么用 SPIRAM：
 *   ESP32-S3 内部 SRAM 仅约 512KB，需要留给 WiFi/BLE 协议栈和 DMA 缓冲区；
 *   大块业务数据（编解码缓冲区、协议结构体等）统一分配到外部 SPIRAM（最大 8MB）。
 */

#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
#include "esp_log.h"

// 打印当前内存状态的超级宏
#define PRINT_MEM_INFO(tag, msg)                                                       \
    do                                                                                 \
    {                                                                                  \
        ESP_LOGI(tag, "--- 内存快照: %s ---", msg);                                    \
        ESP_LOGI(tag, "内部 RAM: 剩余 %lu B (历史最低 %lu B)",                         \
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),          \
                 (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL)); \
        ESP_LOGI(tag, "外部 PSRAM: 剩余 %lu B (历史最低 %lu B)",                       \
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),            \
                 (unsigned long)heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM));   \
    } while (0)

/**
 * @brief 从 SPIRAM 分配指定大小的内存并清零
 *
 * 等价于 calloc，但强制从 SPIRAM 分配。
 * 失败时返回 NULL（SPIRAM 耗尽），调用方需自行检查。
 *
 * @param size 需要分配的字节数
 * @return void* 成功返回清零后的内存指针，失败返回 NULL
 */
static inline void *malloc_zeroed(size_t size)
{
    /* 从 SPIRAM（外部 PSRAM）分配指定大小的内存 */
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (ptr)
    {
        /* 将分配的内存全部清零，避免残留脏数据 */
        memset(ptr, 0, size);
    }
    return ptr;
}
