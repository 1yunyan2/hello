#!/usr/bin/env python3
"""
Project Echo 图片转换工具 v1.0
================================
把根目录 img_src/ 下的 PNG 原图,批量转成 LVGL v9 二进制 .bin,
输出到 assets/img/,供 2.py 打包、3.py 烧进外挂 Flash。

完整流程(三个脚本接力,全在根目录按数字顺序跑):
    python 1.py   ← 本脚本,img_src/*.png → assets/img/*.bin
    python 2.py   ← assets/ → storage.bin
    python 3.py   ← storage.bin 烧到设备外挂 Flash
    设备重启 → /S 挂载 → 代码读 "S:/img/cc.bin" 等

格式说明:
    --cf RGB565A8 = RGB565 + 8位透明通道,和现有 .c 图片(s1.c/p2.c 等)一致,
                    图标抠图不丢透明。是 LVGL v9 格式,与本项目固件匹配。
    脚本只认 PNG(带透明),JPG 会被底层工具自动跳过——UI 素材请统一用 PNG。

依赖(已装则跳过):  pip install pypng lz4
"""
import os
import sys
import subprocess

# Windows 默认 GBK 控制台打不了 emoji,强制 UTF-8(Python 3.7+ 支持)
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass


# === 路径与格式配置(按需改这里)===
SOURCE_DIR = 'img_src'         # 原图目录(根目录下),放 PNG
OUTPUT_DIR = 'assets/img'    # 成品 .bin 输出目录(会被 packer 打包)
COLOR_FORMAT = 'RGB565A8'      # 默认颜色格式,与现有 .c 图片一致(带透明)

# === 按文件指定为不透明 RGB565(2026-08-12 新增)===
# 背景:功能盘换图时可见"两张图斜切同框",根因是无 TE 引脚无法做垂直同步
#      (详见 memory/bugs/BUG-041.md),撕裂**无法根除**,只能缩短可见窗口。
# 手段:把写入 GRAM 的数据量和渲染耗时降下来 ——
#      RGB565A8 = 3 字节/像素(2 色 + 1 alpha),绘制时要逐像素读 alpha 做混合;
#      RGB565   = 2 字节/像素,直接 blit 覆盖,不做混合。
#      100x100 图标:30012 → 20012 字节(-33%),渲染走快路径。
# 前提:这些图标周围本就是纯黑底(s_menu_panel bg_opa=COVER 黑),
#      去掉透明通道后圆角外变成黑色实心,与背景同色,视觉无差别。
# ⚠️ 只列功能盘图标。游戏/背景图(c8/ds/pw 等)不要加进来——它们可能依赖透明叠加。
OPAQUE_RGB565_FILES = {
    'sj.png',    # 时间
    'rl.png',    # 日历
    'sz.png',    # 闹钟
    'djs1.png',  # 倒计时
    'tq.png',    # 天气
    'sc.png',    # 赛车
    'tyt.png',   # 跳一跳
}
# LVGL v9 官方转换脚本(项目自带,随 lvgl 组件下载)
CONVERTER = os.path.join(
    'managed_components', 'lvgl__lvgl', 'scripts', 'LVGLImage.py'
)


def list_source_pngs(src_abs: str):
    """列出待转的 PNG,返回 PNG 文件名列表"""
    pngs = []
    others = []
    for fn in sorted(os.listdir(src_abs)):
        full = os.path.join(src_abs, fn)
        if not os.path.isfile(full):
            continue
        if fn.lower().endswith('.png'):
            pngs.append(fn)
        else:
            others.append(fn)
    print('📁 img_src/ 原图:')
    for fn in pngs:
        sz = os.path.getsize(os.path.join(src_abs, fn))
        print(f'   ✅ {fn:30s} {sz:>10,} 字节  (PNG,将转换)')
    for fn in others:
        sz = os.path.getsize(os.path.join(src_abs, fn))
        print(f'   ⏭️  {fn:30s} {sz:>10,} 字节  (非PNG,跳过)')
    return pngs


def main() -> int:
    print('========================================')
    print('🎨 Project Echo 图片转换工具 v1.0')
    print('========================================\n')

    src_abs = os.path.abspath(SOURCE_DIR)
    out_abs = os.path.abspath(OUTPUT_DIR)

    if not os.path.isdir(src_abs):
        print(f'❌ 原图目录不存在: {src_abs}')
        print(f'   请在根目录建 ./{SOURCE_DIR}/ 并放入 PNG 原图')
        return 1

    if not os.path.isfile(CONVERTER):
        print(f'❌ 找不到 LVGL 转换脚本:\n   {CONVERTER}')
        print('   请确认 lvgl 组件已下载(idf.py reconfigure 会拉取)')
        return 1

    pngs = list_source_pngs(src_abs)
    if not pngs:
        print(f'\n❌ {SOURCE_DIR}/ 下没有 PNG,无可转换')
        return 1

    os.makedirs(out_abs, exist_ok=True)

    # 分两批转换:功能盘图标走不透明 RGB565(见 OPAQUE_RGB565_FILES 说明),
    # 其余图统一走默认的 RGB565A8。两批分别调 LVGLImage.py。
    opaque = [fn for fn in pngs if fn in OPAQUE_RGB565_FILES]
    normal = [fn for fn in pngs if fn not in OPAQUE_RGB565_FILES]

    # 提示清单里写了但 img_src/ 下不存在的文件名(改名/漏放时及早发现)
    missing = OPAQUE_RGB565_FILES - set(pngs)
    if missing:
        print(f'⚠️  OPAQUE_RGB565_FILES 中这些文件在 {SOURCE_DIR}/ 下不存在,已忽略:')
        for fn in sorted(missing):
            print(f'   ❓ {fn}')
        print()

    def run_batch(files, cf, label):
        """转换一批文件。

        ⚠️ LVGLImage.py 的 input 参数**只接受一个路径**(见其 :1476 argparse 定义,
           传多个会被 argparse 拒绝),所以这里逐个文件调用,不能一次传一批。
        --background 0x000000:RGB565 无 alpha 通道,原 PNG 的透明像素需要一个底色
           来合成。功能盘背景是纯黑(s_menu_panel bg_opa=COVER 黑),故合成到黑底,
           圆角外与背景同色,视觉上看不出差别。 """
        if not files:
            return True
        print(f'🎨 [{label}] {cf} → BIN  共 {len(files)} 张')
        for fn in files:
            cmd = [
                sys.executable, CONVERTER,
                '--ofmt', 'BIN',
                '--cf', cf,
                '-o', out_abs,
            ]
            if cf == 'RGB565':
                cmd += ['--background', '0x000000'] # 透明像素合成到黑底
            cmd.append(os.path.join(src_abs, fn))

            result = subprocess.run(cmd, capture_output=True, text=True)
            if result.returncode != 0:
                print(f'   ❌ {fn} 转换失败')
                if result.stdout:
                    print('--- stdout ---\n' + result.stdout)
                if result.stderr:
                    print('--- stderr ---\n' + result.stderr)
                return False
            print(f'   • {fn}')
        print()
        return True

    print()
    if not run_batch(opaque, 'RGB565', '功能盘图标·不透明'):
        return 1
    if not run_batch(normal, COLOR_FORMAT, '其余图·带透明'):
        return 1

    # 列出转换结果
    bins = [f for f in sorted(os.listdir(out_abs)) if f.lower().endswith('.bin')]
    print(f'\n✅ 转换完成,{OUTPUT_DIR}/ 现有 {len(bins)} 个 .bin:')
    for fn in bins:
        sz = os.path.getsize(os.path.join(out_abs, fn))
        print(f'   📦 {fn:30s} {sz:>10,} 字节  → 设备读 "S:/img/{fn}"')

    print(f'\n下一步: python 2.py  →  python 3.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
