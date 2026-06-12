#!/usr/bin/env python3
"""
Project Echo 图片转换工具 v1.0
================================
把根目录 img_src/ 下的 PNG 原图,批量转成 LVGL v9 二进制 .bin,
输出到 assetsss/img/,供 2.py 打包、3.py 烧进外挂 Flash。

完整流程(三个脚本接力,全在根目录按数字顺序跑):
    python 1.py   ← 本脚本,img_src/*.png → assetsss/img/*.bin
    python 2.py   ← assetsss/ → storage.bin
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
OUTPUT_DIR = 'assetsss/img'    # 成品 .bin 输出目录(会被 packer 打包)
COLOR_FORMAT = 'RGB565A8'      # 颜色格式,与现有 .c 图片一致(带透明)
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

    print(f'\n🎨 转换中: {COLOR_FORMAT} → BIN  输出到 {OUTPUT_DIR}/\n')

    # 整个目录递归转(脚本一次只收一个 input,给目录即批量)
    cmd = [
        sys.executable, CONVERTER,
        '--ofmt', 'BIN',
        '--cf', COLOR_FORMAT,
        '-o', out_abs,
        src_abs,
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    if result.returncode != 0:
        print('❌ 转换失败')
        if result.stdout:
            print('--- stdout ---\n' + result.stdout)
        if result.stderr:
            print('--- stderr ---\n' + result.stderr)
        return 1
    if result.stdout.strip():
        print(result.stdout.strip())

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
