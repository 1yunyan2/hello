"""一次性修复：把 LVGLImage.py 生成的 jt1~jt7.c / jp.c 头部 include
从多分支(落到 lvgl/lvgl.h)改成 j4.c 那种带 __has_include 自动探测的写法，
使其在本项目(include 路径只有 lvgl.h，无 lvgl/ 前缀)下能正确编译。"""
import re
import pathlib

GOOD = '''#ifdef __has_include
    #if __has_include("lvgl.h")
        #ifndef LV_LVGL_H_INCLUDE_SIMPLE
            #define LV_LVGL_H_INCLUDE_SIMPLE
        #endif
    #endif
#endif

#if defined(LV_LVGL_H_INCLUDE_SIMPLE)
    #include "lvgl.h"
#else
    #include "lvgl/lvgl.h"
#endif
'''

BAD = re.compile(
    r'#if defined\(LV_LVGL_H_INCLUDE_SIMPLE\)\s*\n'
    r'#include "lvgl\.h"\s*\n'
    r'#elif defined\(LV_LVGL_H_INCLUDE_SYSTEM\)\s*\n'
    r'#include <lvgl\.h>\s*\n'
    r'#elif defined\(LV_BUILD_TEST\)\s*\n'
    r'#include "\.\./lvgl\.h"\s*\n'
    r'#else\s*\n'
    r'#include "lvgl/lvgl\.h"\s*\n'
    r'#endif\s*\n'
)

games = pathlib.Path(__file__).resolve().parent.parent / 'main' / 'games'
files = ['jt%d.c' % n for n in range(1, 8)] + ['jp.c']
for fn in files:
    p = games / fn
    txt = p.read_text(encoding='utf-8')
    new, n = BAD.subn(GOOD, txt, count=1)
    if n == 1:
        p.write_text(new, encoding='utf-8')
        print(fn, 'patched')
    else:
        print(fn, 'PATTERN NOT FOUND')
