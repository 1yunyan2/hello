#!/usr/bin/env python3
"""
监听设备通过 UDP 广播出来的 ESP_LOG 日志（udp_logger.c 配套工具）。

用途：设备纯锂电池供电（未接 USB/串口）时，仍可在电脑上实时看日志——
      尤其是低功耗（待机）进入/退出全过程，串口够不到的场景。

用法：
    python tools/udp_log_listen.py            # 监听默认端口 3333
    python tools/udp_log_listen.py --port 3333

前提：电脑与设备在同一局域网/同一网段（UDP 广播不跨网段）。
"""
import argparse
import socket
import sys


def main():
    parser = argparse.ArgumentParser(description="监听设备 UDP 日志广播")
    parser.add_argument("--port", type=int, default=3333, help="UDP 监听端口（默认 3333，须与设备侧一致）")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(("", args.port))

    print(f"[udp_log_listen] 监听 UDP 广播端口 {args.port}，等待设备日志...（Ctrl+C 退出）", file=sys.stderr)

    try:
        while True:
            data, addr = sock.recvfrom(65536)
            try:
                text = data.decode("utf-8", errors="replace")
            except Exception:
                text = repr(data)
            sys.stdout.write(text)
            sys.stdout.flush()
    except KeyboardInterrupt:
        print("\n[udp_log_listen] 已停止", file=sys.stderr)
    finally:
        sock.close()


if __name__ == "__main__":
    main()
