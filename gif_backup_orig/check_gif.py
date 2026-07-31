# -*- coding: utf-8 -*-
"""
GIF 入库体检工具 —— 检查 GIF 在本项目 LCD(LVGL 9.5) 上能否正常显示。

背景：LVGL 9.5 的 gif_blend_to_rgb565() 未实现 disposal=0/1/3 的透明像素语义
      （managed_components/lvgl__lvgl/src/widgets/gif/lv_gif.c:323-337），
      会把本该"保留上一帧"的透明像素强行涂成 GIF 的背景色，导致画面被破坏。
      症状颜色 = 该 GIF 调色板里"背景索引"那一格的颜色，故各文件表现不同：
        纯黑背景 -> 实心图形被涂黑只剩描边（看着像"颜色翻转"）
        近黑背景 -> 黑横      纯白背景 -> 白横      纯绿背景 -> 绿点

判定规则：disposal ∈ {0,1,3} 且 帧内含透明像素  ->  必翻车
         其余情况（无透明像素，或 disposal==2）  ->  安全

用法：
    python check_gif.py 某个.gif              # 查单个
    python check_gif.py ../assets/gif/*.gif   # 查多个
    python check_gif.py ../assets/gif         # 查整个目录
"""
import sys
import os
import struct
import glob

# ── 本项目的硬性约束 ────────────────────────────────────────────────
SCREEN_W = 320       # BSP_LCD_WIDTH  (main/bsp/bsp_config.h:95)
SCREEN_H = 240       # BSP_LCD_HEIGHT (main/bsp/bsp_config.h:96)
SIZE_LIMIT_KB = 40   # 单文件体积预算


def parse_gif(path):
    """只读 GIF 结构，不解码像素。返回体检所需的关键字段。"""
    d = open(path, 'rb').read()
    if len(d) < 13 or d[0:3] != b'GIF':
        raise ValueError('不是合法的 GIF 文件')

    # 逻辑屏幕描述符
    w, h, packed, bg_idx, _ = struct.unpack('<HHBBB', d[6:13])
    p = 13
    gpal = None
    if packed & 0x80:                       # 有全局调色板
        n = 2 << (packed & 7)
        gpal = [tuple(d[p + i * 3: p + i * 3 + 3]) for i in range(n)]
        p += n * 3

    n_frames = 0
    n_transp = 0
    disposals = set()
    oversize_frame = False
    gce_pending = None

    while p < len(d) and d[p] != 0x3B:       # 0x3B = 文件结束符
        if d[p] == 0x21:                     # 扩展块
            label = d[p + 1]
            p += 2
            if label == 0xF9:                # 图形控制扩展(GCE)
                blk = d[p]
                gp = d[p + 1]
                gce_pending = {
                    'disposal': (gp >> 2) & 7,
                    'transp': bool(gp & 1),
                }
                p += 1 + blk
            while p < len(d) and d[p] != 0:  # 跳过子块
                p += d[p] + 1
            p += 1
        elif d[p] == 0x2C:                   # 图像描述符
            ix, iy, iw, ih, ip = struct.unpack('<HHHHB', d[p + 1:p + 10])
            p += 10
            if ip & 0x80:                    # 局部调色板
                p += (2 << (ip & 7)) * 3
            p += 1                           # LZW 最小码长
            while p < len(d) and d[p] != 0:  # 跳过图像数据子块
                p += d[p] + 1
            p += 1

            n_frames += 1
            if ix + iw > w or iy + ih > h:
                oversize_frame = True
            if gce_pending:
                disposals.add(gce_pending['disposal'])
                if gce_pending['transp']:
                    n_transp += 1
            gce_pending = None
        else:
            break

    return {
        'w': w, 'h': h, 'bg_idx': bg_idx, 'gpal': gpal,
        'n_frames': n_frames, 'n_transp': n_transp,
        'disposals': disposals, 'oversize_frame': oversize_frame,
        'size': len(d),
    }


def check(path):
    """体检单个文件，返回 True=通过"""
    name = os.path.basename(path)
    try:
        r = parse_gif(path)
    except Exception as e:
        print('%-16s  解析失败: %s' % (name, e))
        return False

    problems = []

    # ① 致命：透明像素 + disposal 0/1/3  -> 显示必翻车
    bad_blend = bool(r['disposals'] & {0, 1, 3}) and r['n_transp'] > 0
    if bad_blend:
        bg = r['gpal'][r['bg_idx']] if (r['gpal'] and r['bg_idx'] < len(r['gpal'])) else None
        hint = ''
        if bg:
            lum = 0.299 * bg[0] + 0.587 * bg[1] + 0.114 * bg[2]
            look = ('白横' if lum > 200 else
                    '黑横/实心被涂黑' if lum < 40 else
                    '色块污染')
            hint = '  预计症状: 透明处被涂成 RGB%s -> %s' % (str(bg), look)
        problems.append('透明像素会被涂成背景色（%d/%d 帧含透明，disposal=%s）%s'
                        % (r['n_transp'], r['n_frames'],
                           sorted(r['disposals']), hint))

    # ② 画布超屏 -> 被裁切 + 白吃 PSRAM
    if r['w'] > SCREEN_W or r['h'] > SCREEN_H:
        problems.append('画布 %dx%d 超出屏幕 %dx%d，会被裁切'
                        % (r['w'], r['h'], SCREEN_W, SCREEN_H))

    # ③ 子帧越界（畸形文件）
    if r['oversize_frame']:
        problems.append('存在超出画布边界的子帧（文件可能损坏）')

    # ④ 体积预算
    kb = r['size'] / 1024.0
    over_size = kb > SIZE_LIMIT_KB

    ok = not problems and not over_size
    mark = '[通过]' if ok else '[打回]'

    print('%s %-16s %7.1f KB  %3d帧  %dx%d  透明帧=%-3d disposal=%s'
          % (mark, name, kb, r['n_frames'], r['w'], r['h'],
             r['n_transp'], sorted(r['disposals']) or '-'))

    for msg in problems:
        print('        x %s' % msg)
    if over_size:
        print('        x 体积 %.1f KB 超过预算 %d KB' % (kb, SIZE_LIMIT_KB))

    return ok


def main():
    args = sys.argv[1:]
    if not args:
        print(__doc__)
        return 1

    # 展开目录和通配符
    files = []
    for a in args:
        if os.path.isdir(a):
            files += sorted(glob.glob(os.path.join(a, '*.gif')))
        else:
            files += sorted(glob.glob(a)) or [a]

    if not files:
        print('没找到任何 .gif 文件')
        return 1

    print('=' * 78)
    print('GIF 入库体检  (屏幕 %dx%d, 单文件预算 %d KB)'
          % (SCREEN_W, SCREEN_H, SIZE_LIMIT_KB))
    print('=' * 78)

    passed = sum(1 for f in files if check(f))

    print('-' * 78)
    print('合计 %d 个文件: 通过 %d, 打回 %d' % (len(files), passed, len(files) - passed))
    print()
    print('修复命令（去掉透明像素）:')
    print('  ffmpeg -y -i 输入.gif -vf "split[a][b];[a]palettegen='
          'reserve_transparent=0[p];[b][p]paletteuse" -gifflags -transdiff 输出.gif')
    print('压体积可在 -vf 开头加 fps=12, 或 palettegen=max_colors=32')

    return 0 if passed == len(files) else 2


if __name__ == '__main__':
    sys.exit(main())
