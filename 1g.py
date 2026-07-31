#!/usr/bin/env python3
"""
Project Echo GIF 体检 + 自动修复工具 v1.0
================================
把 gif_src/ 下的原始 GIF 逐个体检,自动修掉"LVGL 9.5 GIF 解码器不支持的
disposal=1 透明像素"问题,输出到 assets/gif/,供 2.py 打包、3.py 烧进外挂 Flash。

完整流程(数字脚本接力,全在根目录按顺序跑):
    python 1g.py  ← 本脚本,gif_src/*.gif → assets/gif/*.gif (体检+按需修复)
    python 2.py   ← assets/ → storage.bin
    python 3.py   ← storage.bin 烧到设备外挂 Flash
    设备重启 → /S 挂载 → 代码读 "S:/gif/xxx.gif" 等

── 背景(为什么需要这一步)────────────────────────────────────────────
LVGL 9.5 的 GIF 解码器(managed_components/lvgl__lvgl/src/widgets/gif/lv_gif.c
gif_blend_to_rgb565 函数)只正确实现了 disposal=2(恢复背景色)这一种情况。
GIF 最常见的"disposal=1 + 透明像素"编码方式(意思是"这块保留上一帧,不重画"),
被 LVGL 错误地画成了"整块涂成背景色",导致画面出现大片色块(黑横/白横/
颜色翻转/绿块等,颜色取决于该 GIF 调色板里背景色索引具体是什么颜色)。

而 one.gif/three.gif(disposal=2)、two.gif(无透明)恰好绕开了这个坑,所以
同一份代码"有的 GIF 正常、有的 GIF 花屏",不是运气,是编码方式不同。

本脚本对每个 GIF 检测"disposal==1 且含透明像素"这一条件,命中就自动转成
"每帧完整重画、不含透明像素"的安全格式(牺牲一定体积换正确显示),不命中
就原样复制,不因转换损失画质。

顺带检查画布尺寸是否超过屏幕 320x240 (ui_port.c 里 1.5 倍面积上限会拒绝
切图保持黑屏),超屏自动等比缩小到屏幕内。

依赖: pip install pillow
"""
import os
import sys
import struct
import shutil

try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass

try:
    from PIL import Image, ImageSequence
except ImportError:
    print('❌ 缺少 Pillow 库，请先运行: pip install pillow')
    sys.exit(1)


# === 路径配置(按需改这里)==================================================
SOURCE_DIR = 'gif_src'         # 原始 GIF 存放目录(根目录下,新素材放这里)
OUTPUT_DIR = 'assets/gif'      # 成品 GIF 输出目录(会被 2.py 打包)
SCREEN_W, SCREEN_H = 320, 240  # LCD 逻辑分辨率(main/bsp/bsp_config.h)


# ── 第一步: 只读探测 GIF 是否会踩 LVGL 9.5 的坑 ────────────────────────────
def probe_gif(path):
    """解析 GIF 头部结构,返回 (是否有风险, 帧数, 画布宽, 画布高, 详情字符串)。
    只做字节级解析,不依赖 Pillow(Pillow 会自动合成帧,拿不到原始 disposal 标记)。"""
    d = open(path, 'rb').read()
    if d[:3] != b'GIF':
        return None
    w, h, packed, bgidx, _ = struct.unpack('<HHBBB', d[6:13])
    p = 13
    if packed & 0x80:
        p += (2 << (packed & 0x07)) * 3

    n_frames = 0
    n_transp_frames = 0
    disposals = set()
    has_full_frame = False  # 是否存在至少一帧铺满整个画布(x=0,y=0,w=画布宽,h=画布高)
    while p < len(d) and d[p] != 0x3B:
        if d[p] == 0x21:  # 扩展块
            label = d[p + 1]
            p += 2
            if label == 0xF9:  # Graphic Control Extension
                bs = d[p]
                gp = d[p + 1]
                disposals.add((gp >> 2) & 0x07)
                if gp & 0x01:
                    n_transp_frames += 1
                p += 1 + bs
            while p < len(d) and d[p] != 0:
                p += d[p] + 1
            p += 1
        elif d[p] == 0x2C:  # 图像描述符
            n_frames += 1
            fx, fy, fw, fh, ip = struct.unpack('<HHHHB', d[p + 1:p + 10])
            if fx == 0 and fy == 0 and fw == w and fh == h:
                has_full_frame = True
            p += 10
            if ip & 0x80:
                p += (2 << (ip & 0x07)) * 3
            p += 1  # LZW min code size
            while p < len(d) and d[p] != 0:
                p += d[p] + 1
            p += 1
        else:
            break

    # 坑①: disposal=1(叠加) + 有透明像素 —— LVGL 把透明像素错误涂成背景色
    risky_disposal = (1 in disposals) and (n_transp_frames > 0)
    # 坑②: 所有帧都只是局部矩形,从未有一帧铺满整个画布 —— LVGL 的 draw_buf
    #       分配后不做背景色预填(managed_components/lvgl__lvgl/src/draw/
    #       lv_draw_buf.c:269 draw_buf_malloc 只是裸 malloc,不清零/不填色),
    #       矩形范围以外的像素永远是未初始化的垃圾数据,显示为大片黑块/花屏。
    #       只要有一帧完整铺满画布(通常是第1帧),后续局部帧才有"底"可叠加,
    #       这种情况就是安全的。
    risky_no_full_frame = not has_full_frame
    risky = risky_disposal or risky_no_full_frame
    over_screen = (w > SCREEN_W or h > SCREEN_H)
    reasons = []
    if risky_disposal:
        reasons.append('透明像素踩坑')
    if risky_no_full_frame:
        reasons.append('无全屏帧(局部矩形无底色)')
    detail = 'disposal=%s 透明帧=%d/%d 画布=%dx%d 全屏帧=%s' % (
        sorted(disposals), n_transp_frames, n_frames, w, h,
        '有' if has_full_frame else '无')
    return dict(risky=risky, over_screen=over_screen, w=w, h=h,
                n_frames=n_frames, detail=detail, reasons=reasons)


# ── 第二步: 命中风险的文件,重新导出成安全格式 ──────────────────────────────
def unlock_if_exists(dst_path):
    """输出目录里的旧文件可能带只读属性(常见于微信/浏览器下载的文件,
    cp/复制时会把只读属性一起搬过来),覆盖前先解锁,否则 copy2/save 会报
    PermissionError 中断整个批处理。"""
    if os.path.exists(dst_path):
        os.chmod(dst_path, 0o666)


BLACK = (0, 0, 0)  # LVGL draw_buf 分配后未初始化,矩形外区域实际显示为黑,
                    # 故"合成后的透明区域"必须固定填黑,才能跟 LVGL 的真实
                    # 显示效果一致(而不是让 Pillow 自己选一个不确定的颜色)


def fix_gif(src_path, dst_path, target_size=None):
    """用 Pillow 逐帧合成出【真实可见画面】,再重新量化成 256 色、不含透明索引,
    保存 —— 这样每帧都是完整画面,LVGL 只会走它唯一实现正确的"无透明直查表"
    分支,不可能再画错。

    【关键坑,2026-07-29 踩过】早期版本直接 fr.convert('RGB'),这一步会把
    alpha=0 的透明像素强制转换成 Pillow 内部选定的某个"占位色"——具体是什么
    颜色取决于该 GIF 的调色板细节,不同文件结果不同且不可预测(实测 2_x 系列
    变成灰色 137,137,137,3_5/3_6 变成暗红)。而 1_1/1_2/1_3 之所以当初蒙对了,
    是因为那批素材背景本来就是不透明的纯黑(RGBA=0,0,0,255),不含真正的透明
    像素,convert('RGB') 蒙对结果纯属巧合,并不是这个方法真的正确。

    正确做法:先转 RGBA 拿到真实 alpha 通道,显式地把 alpha=0(透明)的像素
    替换成 BLACK,alpha>0(不透明)的像素保留原色——这样才能跟 LVGL 实际显示
    效果(矩形外永远是黑)保持一致。"""
    im = Image.open(src_path)
    frames, durations = [], []
    for fr in ImageSequence.Iterator(im):
        rgba = fr.convert('RGBA')
        if target_size:
            rgba = rgba.resize(target_size, Image.LANCZOS)
        black_bg = Image.new('RGB', rgba.size, BLACK)
        black_bg.paste(rgba, mask=rgba.split()[3])  # 只贴不透明部分,透明部分露出黑底
        pal = black_bg.quantize(colors=256, method=Image.MEDIANCUT,
                                 dither=Image.FLOYDSTEINBERG)
        pal.info.pop('transparency', None)
        frames.append(pal)
        durations.append(fr.info.get('duration', 40))

    frames[0].save(
        dst_path, save_all=True, append_images=frames[1:],
        duration=durations, loop=0, disposal=1, optimize=False,
    )


def main():
    print('=' * 60)
    print('📦 Project Echo GIF 体检 + 自动修复工具 v1.0')
    print('=' * 60 + '\n')

    src_abs = os.path.abspath(SOURCE_DIR)
    out_abs = os.path.abspath(OUTPUT_DIR)

    if not os.path.isdir(src_abs):
        print(f'❌ 源目录不存在: {src_abs}')
        print(f'   请把要处理的 GIF 放进 ./{SOURCE_DIR}/ 下')
        return 1

    os.makedirs(out_abs, exist_ok=True)

    files = sorted(f for f in os.listdir(src_abs) if f.lower().endswith('.gif'))
    if not files:
        print(f'❌ {SOURCE_DIR}/ 下没有找到 .gif 文件')
        return 1

    print(f'📁 待处理: {len(files)} 个 GIF\n')
    print('%-16s %-8s %-8s %s' % ('文件', '结果', '体积变化', '详情'))
    print('-' * 78)

    n_fixed = 0
    n_safe = 0
    n_resized = 0

    for fn in files:
        sp = os.path.join(src_abs, fn)
        info = probe_gif(sp)
        if info is None:
            print('%-16s ❌ 跳过    %-8s 不是合法 GIF 文件' % (fn, '-'))
            continue

        # 文件名含非 ASCII 字符(中文等)会撑爆 FAT 短文件名(8.3 格式)字段,
        # wl_fatfsgen.py 打包时直接 PaddingError 崩溃(2.py 曾因中文名"害羞蹭蹭.gif"
        # 报 "subcon build 12 bytes but was allowed only 8")。这里强制拒绝中文名,
        # 逼着改成 ASCII 命名(如 3_1.gif),不让它有机会流进 assets/gif/。
        if any(ord(c) > 127 for c in fn):
            print('%-16s ❌ 拒绝    含非ASCII字符,FAT打包会崩溃,请改成英文/数字命名后重放入 %s/'
                  % (fn, SOURCE_DIR))
            continue
        dp = os.path.join(out_abs, fn)

        target_size = None
        resize_note = ''
        if info['over_screen']:
            # 超屏:按屏幕高度等比缩小,保证不被 ui_port.c 的 1.5 倍面积上限拒绝
            ratio = min(SCREEN_W / info['w'], SCREEN_H / info['h'])
            target_size = (int(info['w'] * ratio), int(info['h'] * ratio))
            resize_note = ' [超屏 %dx%d→已缩至%dx%d]' % (
                info['w'], info['h'], target_size[0], target_size[1])
            n_resized += 1

        unlock_if_exists(dp)  # 目标已存在且只读(如上次跑到一半的残留)时先解锁

        if info['risky'] or target_size:
            fix_gif(sp, dp, target_size)
            o, n = os.path.getsize(sp), os.path.getsize(dp)
            reason = '+'.join(info['reasons']) if info['reasons'] else '仅超屏缩放'
            print('%-16s 🔧 已修复  %+6.0f%%  %s | %s%s'
                  % (fn, (n - o) / o * 100, info['detail'], reason, resize_note))
            n_fixed += 1
        else:
            shutil.copy2(sp, dp)
            print('%-16s ✅ 安全    原样复制  %s' % (fn, info['detail']))
            n_safe += 1

    print('-' * 78)
    print(f'✅ 安全直接用: {n_safe} 个   🔧 已自动修复: {n_fixed} 个'
          + (f'(其中缩放 {n_resized} 个)' if n_resized else ''))
    print(f'\n输出目录: {out_abs}')
    print('下一步: python 2.py   (打包 assets/ → storage.bin)')
    return 0


if __name__ == '__main__':
    sys.exit(main())
