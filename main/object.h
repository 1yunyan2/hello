#pragma once
#include <stdlib.h>
#include <string.h>
#include "esp_heap_caps.h"
static inline void *malloc_zeroed(size_t size)
{
    // 从 SPIRAM 分配指定大小的内存
    void *ptr = heap_caps_malloc(size, MALLOC_CAP_SPIRAM);
    if (ptr)
    {
        // 将分配的内存清零
        memset(ptr, 0, size);
    }
    return ptr;
}
