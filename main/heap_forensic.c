/**
 * @file heap_forensic.c
 * @brief 【堆损坏排查·2026-07-29】堆损坏取证：把被踩地址反查成"谁分配的"
 *
 * 实现要点见 heap_forensic.h 的文件头注释。
 */

#include "heap_forensic.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_heap_trace.h"
#include "esp_err.h"
#include <inttypes.h>

static const char *TAG = "HEAPFOR";

/* 追踪记录缓冲区：分配在 SPIRAM，避免占用本就吃紧的内部 SRAM
 * （内部 SRAM 正是被踩的区域，若把 trace buffer 也放这里会扰乱现场布局）。*/
static heap_trace_record_t *s_records = NULL;
static size_t s_record_num = 0;
static bool s_tracing = false;

bool heap_forensic_start(size_t num_records)
{
#if !CONFIG_HEAP_TRACING_STANDALONE
    (void)num_records;
    ESP_LOGW(TAG, "未开启 CONFIG_HEAP_TRACING_STANDALONE，取证功能不可用");
    return false;
#else
    if (s_tracing)
        return true;

    /* trace buffer 放 SPIRAM：不干扰内部 SRAM 的堆布局（现场就在内部 SRAM） */
    s_records = (heap_trace_record_t *)heap_caps_calloc(
        num_records, sizeof(heap_trace_record_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_records == NULL)
    {
        ESP_LOGE(TAG, "trace buffer 分配失败（需要 %u B @SPIRAM）",
                 (unsigned)(num_records * sizeof(heap_trace_record_t)));
        return false;
    }
    s_record_num = num_records;

    esp_err_t err = heap_trace_init_standalone(s_records, s_record_num);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "heap_trace_init_standalone 失败: %s", esp_err_to_name(err));
        heap_caps_free(s_records);
        s_records = NULL;
        return false;
    }

    /* HEAP_TRACE_ALL：记录所有分配与释放。
     * 不用 HEAP_TRACE_LEAKS——那个模式在 free 时会把记录删掉，而我们要查的块
     * 很可能【尚未释放】就被踩了，但也可能是 UAF（已释放仍被写），两种都要留证。*/
    err = heap_trace_start(HEAP_TRACE_ALL);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "heap_trace_start 失败: %s", esp_err_to_name(err));
        return false;
    }

    s_tracing = true;
    ESP_LOGW(TAG, "★堆追踪已启动：%u 条记录，栈深度 %d，缓冲区 %u B @SPIRAM",
             (unsigned)s_record_num, CONFIG_HEAP_TRACING_STACK_DEPTH,
             (unsigned)(s_record_num * sizeof(heap_trace_record_t)));
    return true;
#endif
}

#if CONFIG_HEAP_TRACING_STANDALONE
/**
 * @brief 打印一条追踪记录的分配调用栈
 *
 * alloced_by[] 里是返回地址，用 idf.py monitor 或 addr2line 可自动还原成
 * 「文件:行号」。freed_by[0] 非空表示该块【已被释放】——若被踩的是已释放的块，
 * 那就是 use-after-free，性质与单纯越界写不同，须区分。
 */
static void dump_record(const heap_trace_record_t *r, const char *role)
{
    uintptr_t start = (uintptr_t)r->address;
    uintptr_t end = start + r->size;
    bool freed = (r->freed_by[0] != NULL);

    ESP_LOGE(TAG, "  [%s] 块 0x%08" PRIxPTR " ~ 0x%08" PRIxPTR " (size=%u) %s",
             role, start, end, (unsigned)r->size,
             freed ? "★已释放(疑似UAF)" : "使用中");

    ESP_LOGE(TAG, "    分配调用栈(alloced_by):");
    for (int i = 0; i < CONFIG_HEAP_TRACING_STACK_DEPTH; i++)
    {
        if (r->alloced_by[i] == NULL)
            break;
        ESP_LOGE(TAG, "      %d: %p", i, r->alloced_by[i]);
    }

    if (freed)
    {
        ESP_LOGE(TAG, "    释放调用栈(freed_by):");
        for (int i = 0; i < CONFIG_HEAP_TRACING_STACK_DEPTH; i++)
        {
            if (r->freed_by[i] == NULL)
                break;
            ESP_LOGE(TAG, "      %d: %p", i, r->freed_by[i]);
        }
    }
}
#endif

void heap_forensic_identify(uintptr_t addr, const char *where)
{
#if !CONFIG_HEAP_TRACING_STANDALONE
    (void)addr;
    (void)where;
#else
    if (!s_tracing)
    {
        ESP_LOGW(TAG, "追踪未启动，无法对 0x%08" PRIxPTR " 取证", addr);
        return;
    }

    size_t count = heap_trace_get_count();
    ESP_LOGE(TAG, "════ 堆损坏取证 @%s ════ 目标地址 0x%08" PRIxPTR "，扫描 %u 条记录",
             where, addr, (unsigned)count);

    /* ── 关键换算（依据 IDF multi_heap_poisoning.c 的内存布局）────────────────
     *   [poison_head_t 8B: head_canary + alloc_size][ 用户数据 alloc_size ][tail_canary 4B]
     *                                                ↑                     ↑
     *                                    heap_trace 记录的 address      poison 报的地址
     *   即："Bad tail at X" 中的 X == 用户数据起始 + alloc_size。
     *   所以对受害块而言 X 恰好等于 (address + size)，本函数据此精确匹配"前驱块"。
     *   "Bad head at X" 则是 X == 用户数据起始 - 8。
     * 一次遍历同时求三者：命中块(含 addr) / 前驱块(尾部紧邻 addr) / 后继块。*/
    heap_trace_record_t hit = {0}, prev = {0}, next = {0}, exact = {0};
    bool has_hit = false, has_prev = false, has_next = false, has_exact = false;

    for (size_t i = 0; i < count; i++)
    {
        heap_trace_record_t r;
        if (heap_trace_get(i, &r) != ESP_OK || r.address == NULL)
            continue;

        uintptr_t start = (uintptr_t)r.address;
        uintptr_t end = start + r.size;

        /* ★ 精确命中：end == addr 说明 addr 正是该块的 tail_canary 位置，
         *   即"Bad tail at addr"报的就是这一块——受害者可以 100% 确定。
         *   (start - 8 == addr) 则对应 "Bad head at addr" 的情形。*/
        if (end == addr || (start >= 8 && start - 8 == addr))
        {
            exact = r;
            has_exact = true;
        }

        if (addr >= start && addr < end)
        {
            hit = r;
            has_hit = true; // 地址落在块体内
        }
        /* 前驱：块尾在 addr 之前（含紧贴），取最接近的一个 */
        if (end <= addr && (!has_prev || end > (uintptr_t)prev.address + prev.size))
        {
            prev = r;
            has_prev = true;
        }
        /* 后继：块首在 addr 之后，取最接近的一个 */
        if (start > addr && (!has_next || start < (uintptr_t)next.address))
        {
            next = r;
            has_next = true;
        }
    }

    /* ★★ 精确命中优先：能匹配上就无需再猜，受害块已 100% 确定 */
    if (has_exact)
    {
        ESP_LOGE(TAG, "  ★★ 精确命中：该地址正是下面这块的 poison 标记位置 ★★");
        dump_record(&exact, "受害块(确定)");
        ESP_LOGE(TAG, "  → 凶手是紧邻其后写数据的代码：把上面 alloced_by 的地址"
                      "用 addr2line 还原，即可知这块内存归谁，再查谁写越界。");
    }

    if (has_hit)
        dump_record(&hit, "命中(地址在块内)");
    else if (!has_exact)
        ESP_LOGE(TAG, "  未找到包含该地址的块（可能分配早于追踪启动，或已被释放并复用）");

    /* 前驱块：精确匹配失败时的兜底猜测——poison 尾标记紧跟块体之后，
     * "结束地址最接近 addr 的那块"极可能就是被写越界的受害者。*/
    if (has_prev && !has_exact)
        dump_record(&prev, "前驱(最可能的受害块)");
    if (has_next && !has_exact)
        dump_record(&next, "后继(参考)");

    ESP_LOGE(TAG, "════ 取证结束 ════");
#endif
}

bool heap_forensic_check(const char *where)
{
    if (heap_caps_check_integrity(MALLOC_CAP_INTERNAL, true))
        return true;

    /* heap_caps_check_integrity 只把损坏地址打印到控制台（"CORRUPT HEAP: Bad tail at 0x..."），
     * 不会以返回值形式给出地址，故此处无法自动拿到 addr。
     * 使用方法：从日志里抄下该地址，调用 heap_forensic_identify() 反查属主。*/
    ESP_LOGE(TAG, "★堆损坏@%s —— 请从上一行 CORRUPT HEAP 抄下地址，"
                  "用 heap_forensic_identify() 反查属主",
             where);
    return false;
}
