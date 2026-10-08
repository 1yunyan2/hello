# -*- coding: utf-8 -*-
"""
统计 assets/gif/ 下每张 GIF「解码后 RGB565 字节流」里出现的【危险字节值】。

为什么统计这个：
  现场偶发「整屏翻转 / 瞬时全黑 / 花屏」。固件里没有任何运行期路径会发
  INVOFF(0x20)/DISPOFF(0x28)/SLPIN(0x10)/SWRESET(0x01) 这几条命令，所以判定是
  SPI 传输中【命令/数据相位错乱】——某个【像素字节】被面板当成命令执行了。
  那么故障概率 ∝ 该字节值在像素流里出现的次数。逐图统计即可验证：
    ① 出故障的图是否恰好是"危险字节密度"最高的图
    ② 若确认，就可以在素材侧把这些字节值"洗掉"（改 1 个 LSB，肉眼不可见）
       —— 这是唯一能让故障"不出现"而不是"被补救"的软件手段。

字节值的多集与字节序无关（大端小端只是每个 16bit 内两字节互换），故无需关心
data_endian / swap_bytes 配置。
"""
import glob
import os
import sys

import numpy as np
from PIL import Image, ImageSequence

# ST7789 单字节命令表（只需要能"一次吃掉一个字节"就有杀伤力的那些）
NAMES = {
    0x01: "SWRESET",   # ★★★ 软复位：INV 归零 + 显示关闭 —— 一字节同时造成"翻转+黑屏"
    0x10: "SLPIN",     # ★★★ 睡眠（黑屏）
    0x11: "SLPOUT",    # 恢复（无害）
    0x12: "NORON",
    0x13: "NORON2",
    0x20: "INVOFF",    # ★★★ 反转
    0x21: "INVON",     # 恢复（无害）
    0x22: "ALLPOFF",   # ★★ 全黑
    0x23: "ALLPON",
    0x28: "DISPOFF",   # ★★★ 黑屏
    0x29: "DISPON",    # 恢复（无害）
    0x2A: "CASET",     # 吃 4 参数
    0x2B: "RASET",     # 吃 4 参数
    0x2C: "RAMWR",     # 相位错乱入口
    0x36: "MADCTL",    # 偏色/镜像
}
# "粘性致命"四件套：一旦命中，必须靠软件补发才能恢复（其余要么无害、要么自愈）
LETHAL = (0x01, 0x10, 0x20, 0x28)

# 空闲轮播池（s_main_gif_table，ui_port.c:1955）：故障正发生在"一直空闲运行"时，优先看这 7 张
IDLE_POOL = {"1_2.gif", "1_3.gif", "1_4.gif", "1_5.gif", "1_6.gif", "1_8.gif", "1_9.gif"}


def frame_bytes(fp):
    """逐帧展开为 RGB565 两字节/像素，返回 (帧数, 累计字节总数, 256 桶直方图)。"""
    hist = np.zeros(256, dtype=np.int64)
    frames = 0
    total = 0
    for fr in ImageSequence.Iterator(Image.open(fp)):
        frames += 1
        a = np.asarray(fr.convert("RGB"), dtype=np.uint16)
        v = ((a[..., 0] & 0xF8) << 8) | ((a[..., 1] & 0xFC) << 3) | (a[..., 2] >> 3)
        b = np.concatenate([(v >> 8).ravel(), (v & 0xFF).ravel()])
        hist += np.bincount(b, minlength=256)
        total += b.size
    return frames, total, hist


def main():
    files = sorted(glob.glob("assets/gif/*.gif"))
    if not files:
        print("no files")
        return 1
    rows = []
    for fp in files:
        name = os.path.basename(fp)
        try:
            frames, total, hist = frame_bytes(fp)
        except Exception as e:  # noqa: BLE001
            print(f"!! {name}: {e}", file=sys.stderr)
            continue
        lethal = int(sum(hist[k] for k in LETHAL))
        rows.append((name, frames, total, hist, lethal))
        print(f"  ... {name} frm={frames} lethal={lethal} "
              f"0x01={hist[0x01]} 0x10={hist[0x10]} 0x20={hist[0x20]} 0x28={hist[0x28]} "
              f"0x22={hist[0x22]} 0x23={hist[0x23]}",
              flush=True)

    rows.sort(key=lambda r: -r[4])
    hdr = (f"{'file':10}{'pool':>5}{'frm':>5}{'bytes':>11}{'0x00%':>7}{'0xFF%':>7} |"
           f"{'0x01':>7}{'0x10':>6}{'0x20':>7}{'0x28':>7} |{'LETHAL':>8}{'/frame':>9}")
    print("\n" + hdr)
    print("-" * len(hdr))
    for name, frames, total, hist, lethal in rows:
        print(f"{name:10}{'IDLE' if name in IDLE_POOL else '':>5}{frames:>5}{total:>11}"
              f"{100*hist[0]/total:>6.1f}%{100*hist[0xFF]/total:>6.1f}% |"
              f"{hist[0x01]:>7}{hist[0x10]:>6}{hist[0x20]:>7}{hist[0x28]:>7} |"
              f"{lethal:>8}{lethal/frames:>9.1f}")

    # ---- 第二张表：空闲池 7 张的"全命令字节普查"（按帧归一，便于跨图比较）----
    # ★=粘性致命（需软件补发才能恢复）  ☆=瞬时花屏类（全屏全亮/全灭/相位错乱）
    # + =恢复类（撞上反而有益）  - =无害
    mark = {0x01: "★", 0x10: "★", 0x20: "★", 0x28: "★",
            0x22: "☆", 0x23: "☆", 0x2A: "☆", 0x2B: "☆", 0x2C: "☆", 0x36: "☆",
            0x11: "+", 0x21: "+", 0x29: "+", 0x12: "-", 0x13: "-"}
    keys = sorted(NAMES.keys())
    print("\n=== 空闲池 7 张：命令字节普查（每帧出现次数）===")
    hdr2 = "file      frm " + "".join(f"{('0x%02X' % k):>8}" for k in keys)
    print(hdr2)
    print("-" * len(hdr2))
    for name, frames, total, hist, lethal in sorted(rows, key=lambda r: r[0]):
        if name not in IDLE_POOL:
            continue
        print(f"{name:10}{frames:>4} " + "".join(f"{hist[k]/frames:>8.1f}" for k in keys))
    print("图例: " + "  ".join(f"0x{k:02X}={NAMES[k]}{mark.get(k, '')}" for k in keys))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
