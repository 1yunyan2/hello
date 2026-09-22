# -*- coding: utf-8 -*-
"""按真实时间轴打印四张中性 GIF 的"变化能量"曲线，用于提取节拍。
1g.py 会把相同帧合并并把 duration 累加，所以必须用 duration 累加的真实时间轴。"""
import sys
from PIL import Image, ImageChops
try:
    from PIL import ImageSequence
except Exception:
    pass

def load(path):
    im = Image.open(path)
    frames, t = [], 0
    for fr in ImageSequence.Iterator(im):
        d = fr.info.get('duration', 0) or im.info.get('duration', 0) or 0
        frames.append((t, d, fr.convert('RGBA')))
        t += d
    return frames, t

for name in sys.argv[1:]:
    fr, total = load(name)
    print("=" * 70)
    print("%s  frames=%d  total=%dms" % (name, len(fr), total))
    prev = None
    peak = 0
    rows = []
    for (t, d, img) in fr:
        if prev is None:
            v = 0
        else:
            v = sum(ImageChops.difference(img, prev).convert('L').point(lambda x: 1 if x > 8 else 0).getdata())
        peak = max(peak, v)
        rows.append((t, d, v))
        prev = img
    for (t, d, v) in rows:
        bar = "#" * int(60 * v / peak) if peak else ""
        print("  t=%5d d=%4d diff=%5d %s" % (t, d, v, bar))
