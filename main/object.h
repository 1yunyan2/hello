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
#include "freertos/FreeRTOS.h" // uxTaskGetStackHighWaterMark / pcTaskGetName（栈水位打印宏依赖）
#include "freertos/task.h"

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
 * @brief 任务创建后打印：栈大小 + 栈所在内存 + 当前内部/外部堆剩余
 *
 * 用法：紧跟在 xTaskCreate* 调用之后。
 *   - stack_bytes：传入创建时的栈深度（字节，即 xTaskCreate 的 usStackDepth 参数值）
 *   - caps_is_internal：栈是否在内部 SRAM（1=内部SRAM，0=SPIRAM），用于一眼区分是否吃内部SRAM
 *
 * 说明：栈只是"预留"，此处堆剩余是任务创建瞬间的快照，反映该任务栈实际吃掉的堆。
 *       想看栈用了多少，用任务函数体入口的 PRINT_TASK_STACK_HWM。
 */
#define PRINT_TASK_CREATED(tag, task_name, stack_bytes, caps_is_internal)                       \
    do                                                                                          \
    {                                                                                           \
        ESP_LOGI(tag, "[栈] 任务 %-16s 栈 %5lu B @%s | 内部SRAM剩余 %lu B, PSRAM剩余 %lu B",    \
                 (task_name), (unsigned long)(stack_bytes),                                     \
                 (caps_is_internal) ? "内部SRAM" : "PSRAM",                                     \
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),                   \
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));                    \
    } while (0)

/**
 * @brief 在任务函数体内打印本任务栈的历史最小剩余（高水位）
 *
 * 用法：放在任务函数入口（或循环里周期打印）。剩余越小说明栈越吃紧；
 *       若某任务长期剩余很大，可调小它的栈来回收内存。
 */
#define PRINT_TASK_STACK_HWM(tag)                                                               \
    do                                                                                          \
    {                                                                                           \
        ESP_LOGI(tag, "[栈水位] %-16s 历史最小剩余 %lu B",                                      \
                 pcTaskGetName(NULL),                                                           \
                 (unsigned long)uxTaskGetStackHighWaterMark(NULL));                             \
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
