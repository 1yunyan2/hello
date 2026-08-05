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

/**
 * @brief 内存/栈调试打印总开关（0=全部编译成空语句，1=正常打印）
 *
 * 关掉的是下面四个宏：PRINT_MEM_INFO / PRINT_MEM_FRAG_INFO /
 * PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM，全工程生效
 * （application.c 的 PRINT_INTERNAL_HEAP_STEP / HEAP_CHECK_STEP 也跟随本开关）。
 *
 * 【为何默认关闭】这些打印散布在整个启动链路上（BSP/唤醒词/协议/会话/UI 各模块），
 *   每条都要走一次 heap_caps 遍历 + 控制台输出，累计拖慢开机；且它们只在排查
 *   "内部SRAM 不够/碎片/栈水位" 时才有意义，平时属于纯噪声。
 *
 * 【何时打开】查内存问题时改成 1；配合 application.c 的 APP_BOOT_DIAG_ENABLE
 *   一起打开，可拿到完整的启动内存画像。
 */
#ifndef MEM_DEBUG_LOG_ENABLE
#define MEM_DEBUG_LOG_ENABLE 1
#endif

#if MEM_DEBUG_LOG_ENABLE

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
 * @brief 打印内存快照 + 碎片化程度（在 PRINT_MEM_INFO 基础上加"最大连续块"）
 *
 * 为什么需要"最大连续块"：剩余总量够、但最大连续块不够时，大块分配（LVGL draw_buf、
 * 解码器缓冲等）依然会失败。两者的差距即碎片化程度，剩余多而连续块小 = 碎片严重。
 * DMA 块单独打印：LCD/I2S 的 DMA 缓冲只能落在既是内部 SRAM 又支持 DMA 的区域。
 */
#define PRINT_MEM_FRAG_INFO(tag, msg)                                                  \
    do                                                                                 \
    {                                                                                  \
        PRINT_MEM_INFO(tag, msg);                                                      \
        ESP_LOGI(tag, "最大连续块: 内部 %lu B | DMA %lu B | PSRAM %lu B",              \
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL), \
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_DMA |      \
                                                                 MALLOC_CAP_INTERNAL), \
                 (unsigned long)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM));  \
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
#define PRINT_TASK_CREATED(tag, task_name, stack_bytes, caps_is_internal)                    \
    do                                                                                       \
    {                                                                                        \
        ESP_LOGI(tag, "[栈] 任务 %-16s 栈 %5lu B @%s | 内部SRAM剩余 %lu B, PSRAM剩余 %lu B", \
                 (task_name), (unsigned long)(stack_bytes),                                  \
                 (caps_is_internal) ? "内部SRAM" : "PSRAM",                                  \
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_INTERNAL),                \
                 (unsigned long)heap_caps_get_free_size(MALLOC_CAP_SPIRAM));                 \
    } while (0)

/**
 * @brief 在任务函数体内打印本任务栈的历史最小剩余（高水位）
 *
 * 用法：放在任务函数入口（或循环里周期打印）。剩余越小说明栈越吃紧；
 *       若某任务长期剩余很大，可调小它的栈来回收内存。
 */
#define PRINT_TASK_STACK_HWM(tag)                                   \
    do                                                              \
    {                                                               \
        ESP_LOGI(tag, "[栈水位] %-16s 历史最小剩余 %lu B",          \
                 pcTaskGetName(NULL),                               \
                 (unsigned long)uxTaskGetStackHighWaterMark(NULL)); \
    } while (0)

/**
 * @brief 【堆损坏排查·2026-07-29】栈水位告警：低于阈值才打印，平时静默
 *
 * 【与 PRINT_TASK_STACK_HWM 的区别】上面那个宏只能读「调用者自己」的栈
 *   （uxTaskGetStackHighWaterMark(NULL)），且每次必打。本宏接受任意任务句柄，
 *   可以在 A 任务里监视 B 任务；且只在剩余低于 warn_bytes 时才输出，
 *   适合放在高频回调里长时间常驻，不会刷屏。
 *
 * 【为何需要它】随机堆损坏（块头魔数被 SRAM 指针值覆写）的典型成因是任务栈溢出，
 *   把栈上局部指针压进了栈外邻接的堆块。栈水位是【累积】指标——不需要复现崩溃，
 *   跑一段时间就能看出谁快见底。这是低频问题（几十次触发一次）唯一不依赖复现的抓手。
 *
 * @param tag        日志 TAG
 * @param handle     被监视任务句柄（NULL=自己）
 * @param warn_bytes 阈值（字节），历史最小剩余低于它才打印
 */
#define WARN_TASK_STACK_LOW(tag, handle, warn_bytes)                             \
    do                                                                           \
    {                                                                            \
        UBaseType_t _hwm = uxTaskGetStackHighWaterMark((TaskHandle_t)(handle));  \
        if (_hwm < (UBaseType_t)(warn_bytes))                                    \
        {                                                                        \
            ESP_LOGE(tag, "[栈水位告警] %-16s 历史最小剩余仅 %lu B (<%lu B)!",   \
                     pcTaskGetName((TaskHandle_t)(handle)),                      \
                     (unsigned long)_hwm, (unsigned long)(warn_bytes));          \
        }                                                                        \
    } while (0)

/**
 * @brief 【堆损坏排查·2026-07-29】无条件打印本任务栈水位（带自定义位置标签）
 *
 * 与 PRINT_TASK_STACK_HWM 的区别：可传一段文字说明「测的是哪个阶段」。
 * 排查栈溢出时，同一任务要在多个阶段各测一次（如 TLS 握手前/后），
 * 只靠任务名分不清是哪一次，故加 where 标签。
 */
#define PRINT_STACK_AT(tag, where)                                  \
    do                                                              \
    {                                                               \
        ESP_LOGW(tag, "[栈水位@%s] %-14s 历史最小剩余 %lu B",       \
                 (where), pcTaskGetName(NULL),                      \
                 (unsigned long)uxTaskGetStackHighWaterMark(NULL)); \
    } while (0)

/**
 * @brief 【堆损坏排查·2026-07-29】只扫内部 SRAM 的堆完整性夹逼探针
 *
 * 【为何只扫内部 SRAM】heap_caps_check_integrity_all(全堆含 PSRAM) 太重会触发看门狗；
 *   本次事故被踩地址 0x3fd17357 在内部 SRAM，只扫 INTERNAL 不漏且开销小一个数量级。
 * 【读法】首次报「★堆损坏@X」的那个 X，就是损坏发生在其之前的最近一步。
 */
#define HEAP_PROBE(tag, where)                                        \
    do                                                                \
    {                                                                 \
        if (!heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))    \
            ESP_LOGE(tag, "★堆损坏@%s", (where));                     \
    } while (0)

#else /* MEM_DEBUG_LOG_ENABLE == 0：全部编译成空语句 */

/* 保留 do{}while(0) 形式，保证 `if (x) PRINT_MEM_INFO(...); else ...` 这类
 * 写法在关闭后语法依然成立；参数不求值（本工程内四个宏的实参均无副作用）。 */
#define PRINT_MEM_INFO(tag, msg) \
    do                           \
    {                            \
    } while (0)
#define PRINT_MEM_FRAG_INFO(tag, msg) \
    do                                \
    {                                 \
    } while (0)
#define PRINT_TASK_CREATED(tag, task_name, stack_bytes, caps_is_internal) \
    do                                                                    \
    {                                                                     \
    } while (0)
#define PRINT_TASK_STACK_HWM(tag) \
    do                            \
    {                             \
    } while (0)
#define WARN_TASK_STACK_LOW(tag, handle, warn_bytes) \
    do                                               \
    {                                                \
    } while (0)
#define PRINT_STACK_AT(tag, where) \
    do                             \
    {                              \
    } while (0)
#define HEAP_PROBE(tag, where) \
    do                         \
    {                          \
    } while (0)

#endif /* MEM_DEBUG_LOG_ENABLE */

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
