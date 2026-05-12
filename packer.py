#!/usr/bin/env python3
"""
Project Echo 资源打包工具 v1.0
================================

把本地 assetsss/ 目录直接打成 32MB FAT 镜像 (storage.bin),布局与固件
  esp_vfs_fat_spiflash_mount_rw_wl("/S", "ext_storage", &cfg, ...)
完全一致 (wear-leveling + sector 4096),写完直接给 burner.py 烧。

使用流程:
    python packer.py        ← 本脚本,assetsss/ → storage.bin
    python burner.py        ← 把 storage.bin 烧到设备外挂 Flash
    设备重启 → /S 挂载成功 → 代码可读 /S/gif/one.gif 等

依赖: ESP-IDF v5.x 自带的 wl_fatfsgen.py
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


# === 必须和固件参数严格一致 ===
PARTITION_SIZE = 32 * 1024 * 1024   # 对应 main/ui/flash.c TOTAL_FLASH_SIZE
SECTOR_SIZE    = 4096               # 对应 mount_config.allocation_unit_size

SOURCE_DIR  = 'assetsss'
OUTPUT_FILE = 'storage.bin'

# IDF 路径优先取环境变量,否则用本机固定路径(从 .vscode/settings.json 拿到的)
IDF_PATH = os.environ.get(
    'IDF_PATH',
    r'D:\ESP-IDF\.espressif\v5.3.4\esp-idf'
)


def list_source_files(src_abs: str) -> int:
    """打印将要打包的文件列表,返回总字节数"""
    print('📁 待打包内容:')
    total = 0
    count = 0
    for root, _, files in os.walk(src_abs):
        for fn in sorted(files):
            full = os.path.join(root, fn)
            rel = os.path.relpath(full, src_abs).replace('\\', '/')
            sz = os.path.getsize(full)
            print(f'   /S/{rel:40s} {sz:>10,} 字节')
            total += sz
            count += 1
    print(f'   ── {count} 个文件,合计 {total:,} 字节 ({total/1024/1024:.2f} MB)')
    return total


def main() -> int:
    print('========================================')
    print('📦 Project Echo 资源打包工具 v1.0')
    print('========================================\n')

    src_abs = os.path.abspath(SOURCE_DIR)
    out_abs = os.path.abspath(OUTPUT_FILE)

    if not os.path.isdir(src_abs):
        print(f'❌ 源目录不存在: {src_abs}')
        print(f'   请把要烧到设备的资源放进 ./{SOURCE_DIR}/ 下')
        return 1

    fatfs_dir = os.path.join(IDF_PATH, 'components', 'fatfs')
    tool = os.path.join(fatfs_dir, 'wl_fatfsgen.py')
    if not os.path.isfile(tool):
        print(f'❌ 找不到 wl_fatfsgen.py:\n   {tool}')
        print('   请确认 ESP-IDF 已安装,或修改本脚本顶部的 IDF_PATH 常量')
        return 1

    total_bytes = list_source_files(src_abs)
    if total_bytes == 0:
        print(f'❌ {SOURCE_DIR}/ 是空的,没东西可打')
        return 1

    # FAT 元数据 + WL 元数据 大约要 100KB,留 5% 安全余量
    if total_bytes > PARTITION_SIZE * 0.95:
        print(f'\n⚠️ 警告:资源 {total_bytes/1024/1024:.2f}MB 接近分区上限 '
              f'{PARTITION_SIZE/1024/1024:.0f}MB,可能装不下')

    print(f'\n📦 正在打包 → {OUTPUT_FILE}')
    print(f'   分区: {PARTITION_SIZE//1024//1024}MB | 扇区: {SECTOR_SIZE}B | wear-leveling: 开\n')

    # wl_fatfsgen.py 用 from fatfs_utils.xxx 这种相对包引用,
    # 必须把 cwd 设到 components/fatfs 才能找到模块
    cmd = [
        sys.executable, tool,
        src_abs,
        '--output_file',     out_abs,
        '--partition_size',  str(PARTITION_SIZE),
        '--sector_size',     str(SECTOR_SIZE),
        '--long_name_support',  # 支持长文件名,避免被截成 8.3
    ]

    result = subprocess.run(cmd, cwd=fatfs_dir, capture_output=True, text=True)
    if result.returncode != 0:
        print('❌ 打包失败')
        if result.stdout:
            print('--- stdout ---\n' + result.stdout)
        if result.stderr:
            print('--- stderr ---\n' + result.stderr)
        return 1

    if not os.path.isfile(out_abs):
        print(f'❌ wl_fatfsgen 报告成功但 {OUTPUT_FILE} 不存在,异常')
        return 1

    out_size = os.path.getsize(out_abs)
    # 校验 FAT 镜像合法性:
    # WL 模式下首扇区是轮转备用区,真正的 BPB 在物理扇区 1
    # 引导签名应在偏移 SECTOR_SIZE + 510
    bpb_off = SECTOR_SIZE
    sig_off = bpb_off + 510
    with open(out_abs, 'rb') as f:
        f.seek(bpb_off)
        bpb_head = f.read(11)         # 跳转 + OEM 名 (例如 'MSDOS5.0')
        f.seek(sig_off)
        sig = f.read(2)

    sig_ok = (sig == b'\x55\xaa')
    bpb_ok = bpb_head[0] in (0xEB, 0xE9)  # BPB 必以 short/near jump 开头

    print(f'✅ 打包完成: {OUTPUT_FILE}')
    print(f'   大小: {out_size:,} 字节 ({out_size/1024/1024:.2f} MB)')
    print(f'   BPB 头 (偏移 0x{bpb_off:x}): {bpb_head.hex()}  '
          f'{"✓" if bpb_ok else "✗"}')
    print(f'   引导签名 (偏移 0x{sig_off:x}): {sig.hex()}  '
          f'{"✓ 有效" if sig_ok else "✗ 无效"}')

    if not (sig_ok and bpb_ok):
        print('\n⚠️ 镜像格式校验未通过,烧进去可能挂载失败')
        return 1
    print(f'\n下一步: python burner.py')
    return 0


if __name__ == '__main__':
    sys.exit(main())
