#pragma once

/**
 * @file heap_forensic.h
 * @brief 【堆损坏排查·2026-07-29】堆损坏"取证"工具：定位被踩内存块的真正属主
 *
 * ── 背景 ────────────────────────────────────────────────────────────────────
 * 现场症状：运行数十秒后 heap poison 报块尾被覆写，例如
 *     CORRUPT HEAP: Bad tail at 0x3fd17357. Expected 0xbaad5678 got 0x00000000
 * 此后每次遍历堆都重复报同一地址（同一次开机内地址恒定），但跨重启会变
 * （曾出现 Bad head @0x3fcdd524）。说明是"越界写"而非针对某个固定结构体。
 *
 * ── 为何需要本模块 ──────────────────────────────────────────────────────────
 * 之前用 heap_caps_check_integrity 夹逼探针只能回答"损坏发生在哪两步之间"，
 * 回答不了"被踩的那块内存是谁申请的"。而 .map 文件只含静态符号，堆块是运行时
 * 分配的，查不到属主。开启 CONFIG_HEAP_TRACING_STANDALONE 后，每次 malloc 都会
 * 记录调用栈（alloced_by[]），本模块据此把地址反查成"谁分配的"。
 *
 * ── 判读方法（关键）────────────────────────────────────────────────────────
 * poison 报的地址是【块尾标记】的位置，不是块首。真正被越界写的受害块是
 * "结束地址最接近且小于该地址"的那一块。找到它之后：
 *   - 该块的 alloced_by[] 调用栈 = 受害者是谁；
 *   - 凶手通常是【紧邻其后】写数据的代码，多为把 size 算少了 1 的 memcpy/strcpy/写 '\0'。
 * 故本模块同时打印"命中块"与"其后相邻块"，两边夹着看。
 */

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief 初始化并启动堆追踪（记录每次分配的调用栈）
 *
 * 应在 app_main 尽早调用，越早启动记录越完整。
 * 仅在 CONFIG_HEAP_TRACING_STANDALONE 开启时有效，否则本函数为空操作。
 *
 * @param num_records 追踪记录条数（每条约 40~70 字节，取决于栈深度配置）
 * @return true 启动成功；false 未开启追踪或内存不足
 */
bool heap_forensic_start(size_t num_records);

/**
 * @brief 按地址反查属主：打印包含/邻近该地址的堆块及其分配调用栈
 *
 * @param addr        poison 报出的损坏地址（如 0x3fd17357）
 * @param where       调用位置标签，便于在日志中区分是哪一次取证
 */
void heap_forensic_identify(uintptr_t addr, const char *where);

/**
 * @brief 扫描内部 SRAM，若发现损坏则自动对损坏地址做一次属主反查
 *
 * 与单纯的 heap_caps_check_integrity 探针的区别：本函数在发现损坏后，
 * 会顺带把"被踩块是谁分配的"一并打印出来，一步到位。
 *
 * @param where 调用位置标签
 * @return true 堆完好；false 检测到损坏（已打印取证信息）
 */
bool heap_forensic_check(const char *where);
