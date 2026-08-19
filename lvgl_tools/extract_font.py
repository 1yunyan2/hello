#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
从设计稿图片抠字形 → 生成 LVGL 位图字体 (.c)

用途：用户的设计稿是网上找的、无源字体，且要免费商用。直接从设计稿里
把每个字形（数字/中文/符号）的像素抠出来（带抗锯齿 alpha），组装成一个
自定义 lv_font_t，做到宽高 1:1 临摹，完全不依赖任何字体文件。

用法：
    python extract_font.py <图片路径> <字体名> <字符1:left,top,right,bottom> <字符2:...> ...

示例（天气屏 23°C，320x240 图片）：
    python extract_font.py tq_320x240.png font_wx_temp "2:172,42,199,90" "3:207,42,234,91" ...
"""
import sys
import numpy as np
from PIL import Image

BPP = 8          # 8bpp = 每像素 1 字节 alpha，抗锯齿最保真
BG_LEVEL = 30    # 设计稿背景亮度，低于此值视为透明（去背景）
GLYPH_GAP = 2    # 数字间默认间距（像素）


def extract_glyph(img_gray, left, top, right, bottom):
    """抠出 [left,right]x[top,bottom] 区域，转为 alpha 并裁掉空白边缘。"""
    region = img_gray[top:bottom + 1, left:right + 1].astype(np.float32)
    # 亮度→alpha：背景(暗)归零，文字(亮)到 255，中间值保留抗锯齿过渡
    alpha = np.clip((region - BG_LEVEL) * 255.0 / (255 - BG_LEVEL), 0, 255).astype(np.uint8)
    # 裁掉四周全透明行列
    ys, xs = np.where(alpha > 0)
    if len(xs) == 0:
        return None
    return alpha[ys.min():ys.max() + 1, xs.min():xs.max() + 1]


def gen_c(name, glyphs):
    """glyphs: list of (char, alpha2d)。生成 LVGL 8bpp 字体 C 代码。"""
    out = []
    w = out.append
    # 头 + include 探测块
    w('/** 由 extract_font.py 从设计稿抠图生成，%dbpp，免费商用（临摹字形，不含任何字体文件） */' % BPP)
    w('#ifdef __has_include')
    w('    #if __has_include("lvgl.h")')
    w('        #ifndef LV_LVGL_H_INCLUDE_SIMPLE')
    w('            #define LV_LVGL_H_INCLUDE_SIMPLE')
    w('        #endif')
    w('    #endif')
    w('#endif')
    w('')
    w('#ifdef LV_LVGL_H_INCLUDE_SIMPLE')
    w('#include "lvgl.h"')
    w('#else')
    w('#include "lvgl/lvgl.h"')
    w('#endif')
    w('')
    w('#ifndef %s' % name.upper())
    w('#define %s 1' % name.upper())
    w('#endif')
    w('')
    w('#if %s' % name.upper())
    w('')
    w('/*-----------------')
    w(' *    BITMAPS')
    w(' *-----------------*/')
    w('static LV_ATTRIBUTE_LARGE_CONST const uint8_t glyph_bitmap[] = {')
    # 拼字形位图
    dsc = []   # (bitmap_index, adv_w, box_w, box_h, ofs_x, ofs_y)
    cur = 0
    for ch, alpha in glyphs:
        h, wdt = alpha.shape
        dsc.append((cur, wdt, h))
        # 逐字节输出
        for yy in range(h):
            row = alpha[yy]
            line = ', '.join('0x%02x' % v for v in row)
            w('    %s,' % line)
        cur += h * wdt
    w('};')
    w('')
    w('/*---------------------')
    w(' *  GLYPH DESCRIPTION')
    w(' *---------------------*/')
    w('static const lv_font_fmt_txt_glyph_dsc_t glyph_dsc[] = {')
    w('    {.bitmap_index = 0, .adv_w = 0, .box_w = 0, .box_h = 0, .ofs_x = 0, .ofs_y = 0} /* id=0 reserved */,')
    for i, (ch, alpha) in enumerate(glyphs, start=1):
        h, wdt = alpha.shape
        bi = dsc[i - 1][0]
        adv = (wdt + GLYPH_GAP) * 10
        w('    {.bitmap_index = %d, .adv_w = %d, .box_w = %d, .box_h = %d, .ofs_x = 0, .ofs_y = 0},' % (bi, adv, wdt * 10, h * 10))
    w('};')
    w('')
    # cmap：逐个字符映射（每个字符一个 FORMAT0_TINY range）
    w('/*--------------------')
    w(' *  CHARACTER MAPPING')
    w(' *--------------------*/')
    w('static const lv_font_fmt_txt_cmap_t cmaps[] =')
    w('{')
    for i, (ch, alpha) in enumerate(glyphs):
        cp = ord(ch)
        w('    {.range_start = %d, .range_length = 1, .glyph_id_start = %d,' % (cp, i + 1))
        w('     .unicode_list = NULL, .glyph_id_ofs_list = NULL, .list_length = 0, .type = LV_FONT_FMT_TXT_CMAP_FORMAT0_TINY},')
    w('};')
    w('')
    w('static const lv_font_fmt_txt_dsc_t font_dsc = {')
    w('    .glyph_bitmap = glyph_bitmap,')
    w('    .glyph_dsc = glyph_dsc,')
    w('    .cmaps = cmaps,')
    w('    .kern_dsc = NULL,')
    w('    .kern_scale = 16,')
    w('    .cmap_num = %d,' % len(glyphs))
    w('    .bpp = %d,' % BPP)
    w('    .kern_classes = 0,')
    w('    .bitmap_format = LV_FONT_FMT_TXT_PLAIN,')
    w('};')
    w('')
    max_h = max(a.shape[0] for _, a in glyphs)
    w('const lv_font_t %s = {' % name)
    w('    .get_glyph_dsc = lv_font_get_glyph_dsc_fmt_txt,')
    w('    .get_glyph_bitmap = lv_font_get_bitmap_fmt_txt,')
    w('    .line_height = %d,' % max_h)
    w('    .base_line = 0,')
    w('    .subpx = LV_FONT_SUBPX_NONE,')
    w('    .underline_position = -1,')
    w('    .underline_thickness = 1,')
    w('    .static_bitmap = 0,')
    w('    .dsc = &font_dsc,')
    w('    .fallback = NULL,')
    w('};')
    w('')
    w('#endif /* %s */' % name.upper())
    return '\n'.join(out)


def main():
    args = sys.argv[1:]
    if len(args) < 3:
        print(__doc__)
        sys.exit(1)
    img_path = args[0]
    name = args[1]
    img = Image.open(img_path).convert('L')
    gray = np.array(img)
    glyphs = []
    for spec in args[2:]:
        ch, box = spec.split(':', 1)
        # ch 可以是单个字符(如 "2"/"C")或码点数字(如 "176" 代表 °)
        cp = ord(ch) if len(ch) == 1 else int(ch)
        l, t, r, b = map(int, box.split(','))
        alpha = extract_glyph(gray, l, t, r, b)
        if alpha is None:
            print('警告: 码点 %d 在区域内无像素，跳过' % cp)
            continue
        glyphs.append((cp, alpha))
        print('  抠出 U+%04X(%s) : %d x %d px (宽高比 %.2f)' % (cp, chr(cp), alpha.shape[1], alpha.shape[0], alpha.shape[1] / alpha.shape[0]))
    code = gen_c(name, glyphs)
    out_path = 'main/ui/%s.c' % name
    with open(out_path, 'w', encoding='utf-8') as f:
        f.write(code)
    print('已生成 %s (%d 个字形)' % (out_path, len(glyphs)))


if __name__ == '__main__':
    main()
