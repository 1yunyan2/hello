#!/usr/bin/env python3
"""
Project Echo 一键全流程 v1.0
================================
依次执行 1.py(转换) → 2.py(打包) → 3.py(烧录),工厂流水线一条命令跑完:

    python 0.py

逻辑:
    1.py 图片转换(img_src/*.png → assetsss/img/*.bin)
      └ 成功 → 2.py 打包(assetsss/ → storage.bin)
          └ 成功 → 3.py 烧录(storage.bin → 外挂 Flash,持续等设备,不返回)
    任一步失败 → 立即停止报错,绝不带着错误镜像去烧。

说明:
    本脚本不 import 1/2/3(它们是数字名,且各自独立),而是用 subprocess
    像命令行一样依次"运行"它们 —— 与手动敲 python 1.py / 2.py / 3.py 等价。
    3.py 是连续烧录死循环(烧完一台等下一台),所以放最后、接管控制台不返回。
"""
import sys
import subprocess

# Windows 默认 GBK 控制台打不了 emoji,强制 UTF-8(Python 3.7+ 支持)
try:
    sys.stdout.reconfigure(encoding='utf-8', errors='replace')
    sys.stderr.reconfigure(encoding='utf-8', errors='replace')
except Exception:
    pass


def run_step(script: str, title: str) -> bool:
    """运行一个子脚本,实时透传输出。返回是否成功(退出码 0)。"""
    print('\n' + '=' * 44)
    print(f'▶️  {title}  (python {script})')
    print('=' * 44)
    # 不捕获输出 → 子脚本的进度/提示实时显示在控制台
    result = subprocess.run([sys.executable, script])
    if result.returncode != 0:
        print(f'\n❌ {title} 失败(退出码 {result.returncode}),流程中止。')
        return False
    return True


def main() -> int:
    print('========================================')
    print('🚀 Project Echo 一键全流程(转换→打包→烧录)')
    print('========================================')

    # 第 ① 步:图片转换
    if not run_step('1.py', '第①步 图片转换'):
        return 1

    # 第 ② 步:打包
    if not run_step('2.py', '第②步 资源打包'):
        return 1

    # 第 ③ 步:烧录(死循环连续烧录,接管控制台不返回)
    print('\n' + '=' * 44)
    print('▶️  第③步 产线烧录  (python 3.py)')
    print('   ⚠️ 持续等待设备,烧完一台等下一台,Ctrl+C 退出')
    print('=' * 44)
    subprocess.run([sys.executable, '3.py'])
    return 0


if __name__ == '__main__':
    sys.exit(main())
