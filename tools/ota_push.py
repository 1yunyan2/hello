#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ota_push.py — Echo2 OTA 一条龙模拟推送工具
================================================

在你的电脑上一键完成 OTA 升级模拟（设备与电脑需在同一局域网/热点）：

  1. 起一个 HTTP 服务，托管 build/Echo.bin（设备从这里下载固件）
  2. 自动计算固件的 sha256 与 size（与固件回读校验完全一致）
  3. 用本机局域网 IP 拼出下载 URL
  4. 连接 MQTT broker，把 OTA 指令发布到 echopal/device/{deviceId}/command
  5. 订阅 echopal/device/{deviceId}/ota-status，实时打印设备上报的进度

依赖：
    pip install paho-mqtt

用法示例：
    # 最简：用默认 broker，固件取 build/Echo.bin，自动探测本机 IP
    python tools/ota_push.py --device A1B2C3 --version 1.0.1

    # 指定固件路径 / broker / 端口
    python tools/ota_push.py --device A1B2C3 --version 1.0.1 \
        --bin build/Echo.bin --broker 122.224.191.2 --mqtt-port 1883 \
        --user xtc --password Xtc@12345 --http-port 8000

    # 负向测试：故意篡改 sha256，验证设备校验失败后不变砖
    python tools/ota_push.py --device A1B2C3 --version 1.0.1 --corrupt-sha

deviceId 怎么拿：设备串口日志里 MQTT 订阅主题 echopal/device/XXXXXX/command，
其中 XXXXXX（6 位十六进制 = MAC 后三字节）就是 deviceId。
"""

import argparse
import hashlib
import os
import socket
import sys
import threading
import time
from functools import partial
from http.server import HTTPServer, SimpleHTTPRequestHandler

try:
    import paho.mqtt.client as mqtt
except ImportError:
    print("[错误] 缺少 paho-mqtt 库，请先安装：  pip install paho-mqtt")
    sys.exit(1)


# ─────────────────────────────────────────────────────────────────────────
# 工具函数
# ─────────────────────────────────────────────────────────────────────────
def _score_lan_ip(ip):
    """给候选 IP 打分：分数越高越像"设备能访问到的家用局域网 IP"。

    踩过的坑：直接 connect 8.8.8.8 取出口 IP，会被 VPN / WSL / VMware 等
    虚拟网卡的路由抢走，得到设备根本访问不到的地址（如 OpenVPN 的 11.10.0.62）。
    这里改为枚举所有网卡 IP 后按网段优先级评分，自动避开虚拟网卡。
    """
    parts = ip.split(".")
    if len(parts) != 4 or not all(p.isdigit() for p in parts):
        return -100
    a, b = int(parts[0]), int(parts[1])

    # 明确排除：回环 / 链路本地(169.254)
    if a == 127 or (a == 169 and b == 254):
        return -100

    # 192.168.x：最典型的家用路由 WiFi 网段，最高优先级
    if a == 192 and b == 168:
        return 100
    # 172.16~31.x：常见私网，但 172.17~28 常被 Docker/WSL 占用 → 降权
    if a == 172 and 16 <= b <= 31:
        # WSL 默认 172.x（如 172.28.48.1），明显降权但仍可作兜底
        return 30 if b in (17, 18, 19, 28) else 60
    # 10.x：私网，但子网掩码 /30 之类多为 VPN 点对点 → 给中等分
    if a == 10:
        return 50
    # 其他（公网/特殊网段，多半是 VPN 适配器如 11.x）→ 最低
    return 0


def get_lan_ip():
    """探测本机局域网 IP（枚举所有网卡，优先 192.168.x，自动跳过 VPN/WSL 虚拟网卡）"""
    candidates = []

    # 方式一：枚举主机名解析出的所有 IPv4（覆盖多网卡）
    try:
        hostname = socket.gethostname()
        for info in socket.getaddrinfo(hostname, None, socket.AF_INET):
            ip = info[4][0]
            if ip not in candidates:
                candidates.append(ip)
    except Exception:
        pass

    # 方式二：connect 取出口 IP（作为补充候选，可能是 VPN，靠评分淘汰）
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
        if ip not in candidates:
            candidates.append(ip)
    except Exception:
        pass
    finally:
        s.close()

    if not candidates:
        return "127.0.0.1"

    # 按评分选最优；并列时取第一个
    best = max(candidates, key=_score_lan_ip)
    # 若有多个候选，打印出来方便用户核对（避免又选错网卡却不自知）
    if len(candidates) > 1:
        ranked = sorted(candidates, key=_score_lan_ip, reverse=True)
        print(f"[网卡] 探测到多个 IP，自动选用 {best}")
        print(f"       候选(按优先级): {', '.join(ranked)}")
        print(f"       如果设备访问不到，请用 --ip 手动指定设备同网段的那个\n")
    return best


def calc_sha256_and_size(path):
    """计算固件的 sha256（小写十六进制）和字节数 —— 与固件 verify_partition_sha256 一致"""
    h = hashlib.sha256()
    size = 0
    with open(path, "rb") as f:
        while True:
            chunk = f.read(65536)
            if not chunk:
                break
            h.update(chunk)
            size += len(chunk)
    return h.hexdigest(), size


def start_http_server(directory, port):
    """在后台线程起一个 HTTP 服务，根目录指向固件所在目录"""
    handler = partial(SimpleHTTPRequestHandler, directory=directory)
    # 允许端口快速复用，避免反复运行时 "address already in use"
    httpd = HTTPServer(("0.0.0.0", port), handler)
    t = threading.Thread(target=httpd.serve_forever, daemon=True)
    t.start()
    return httpd


# ─────────────────────────────────────────────────────────────────────────
# MQTT 回调
# ─────────────────────────────────────────────────────────────────────────
def on_connect(client, userdata, flags, rc, *args):
    status_topic = userdata["status_topic"]
    if rc == 0:
        print(f"[MQTT] 已连接 broker，订阅进度主题: {status_topic}")
        client.subscribe(status_topic, qos=1)
        # 连接成功后立即发布 OTA 指令
        cmd_topic = userdata["cmd_topic"]
        payload = userdata["payload"]
        client.publish(cmd_topic, payload, qos=1)
        print(f"[MQTT] 已发布 OTA 指令到: {cmd_topic}")
        print(f"       {payload}")
        print("[等待] 监听设备上报进度（Ctrl+C 退出）...\n")
    else:
        print(f"[MQTT] 连接失败 rc={rc}")


def on_message(client, userdata, msg):
    text = msg.payload.decode("utf-8", errors="replace")
    print(f"[ota-status] {text}")
    # 收到 success 即可认为整条链路跑通
    if '"success"' in text:
        print("\n[完成] 设备已上报 success，OTA 全流程验证通过！")


# ─────────────────────────────────────────────────────────────────────────
# 主流程
# ─────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Echo2 OTA 一条龙模拟推送工具",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--device", required=True, help="deviceId（MAC 后三字节 6 位十六进制，如 A1B2C3）")
    parser.add_argument("--version", required=True, help="新固件版本号（须比设备当前版本新，如 1.0.1）")
    parser.add_argument("--bin", default="build/Echo.bin", help="固件路径（默认 build/Echo.bin）")
    parser.add_argument("--broker", default="122.224.191.2", help="MQTT broker 地址")
    parser.add_argument("--mqtt-port", type=int, default=1883, help="MQTT 端口（默认 1883）")
    parser.add_argument("--user", default="xtc", help="MQTT 用户名")
    parser.add_argument("--password", default="Xtc@12345", help="MQTT 密码")
    parser.add_argument("--http-port", type=int, default=8000, help="本地 HTTP 服务端口（默认 8000）")
    parser.add_argument("--ip", default=None, help="手动指定本机局域网 IP（默认自动探测）")
    parser.add_argument("--corrupt-sha", action="store_true",
                        help="负向测试：故意篡改 sha256，验证设备校验失败后不变砖")
    args = parser.parse_args()

    # 1. 检查固件文件
    bin_path = os.path.abspath(args.bin)
    if not os.path.isfile(bin_path):
        print(f"[错误] 固件不存在: {bin_path}\n请先 idf.py build 生成固件。")
        sys.exit(1)
    bin_dir = os.path.dirname(bin_path)
    bin_name = os.path.basename(bin_path)

    # 2. 计算 sha256 + size
    sha256, size = calc_sha256_and_size(bin_path)
    if args.corrupt_sha:
        # 翻转最后一位十六进制，制造一个"对不上"的指纹
        last = sha256[-1]
        bad = "0" if last != "0" else "1"
        sha256 = sha256[:-1] + bad
        print("[负向测试] 已篡改 sha256 最后一位，设备应校验失败、不重启、不变砖")

    # 3. 确定下载 URL
    ip = args.ip or get_lan_ip()
    url = f"http://{ip}:{args.http_port}/{bin_name}"

    # 4. 起 HTTP 服务
    start_http_server(bin_dir, args.http_port)

    # 5. 组装 OTA 指令 JSON（字段与后端协议、固件解析完全对齐）
    ts = int(time.time() * 1000)
    payload = (
        '{'
        f'"type":"ota",'
        f'"version":"{args.version}",'
        f'"url":"{url}",'
        f'"sha256":"{sha256}",'
        f'"size":{size},'
        f'"timestamp":{ts}'
        '}'
    )

    cmd_topic = f"echopal/device/{args.device}/command"
    status_topic = f"echopal/device/{args.device}/ota-status"

    # 打印汇总
    print("=" * 60)
    print("Echo2 OTA 模拟推送")
    print("=" * 60)
    print(f"固件文件 : {bin_path}")
    print(f"固件大小 : {size} 字节")
    print(f"SHA256   : {sha256}")
    print(f"下载 URL : {url}")
    print(f"HTTP 服务: http://0.0.0.0:{args.http_port}  (根目录 {bin_dir})")
    print(f"broker   : {args.broker}:{args.mqtt_port}")
    print(f"deviceId : {args.device}")
    print("=" * 60)
    print()

    # 6. 连接 MQTT，发指令 + 订阅进度
    userdata = {
        "cmd_topic": cmd_topic,
        "status_topic": status_topic,
        "payload": payload,
    }
    # paho-mqtt 2.x 需要指定 CallbackAPIVersion；兼容 1.x
    try:
        client = mqtt.Client(
            callback_api_version=mqtt.CallbackAPIVersion.VERSION2,
            userdata=userdata,
        )
    except (AttributeError, TypeError):
        client = mqtt.Client(userdata=userdata)

    client.username_pw_set(args.user, args.password)
    client.on_connect = on_connect
    client.on_message = on_message

    try:
        client.connect(args.broker, args.mqtt_port, keepalive=60)
    except Exception as e:
        print(f"[错误] 无法连接 broker {args.broker}:{args.mqtt_port} — {e}")
        sys.exit(1)

    try:
        client.loop_forever()
    except KeyboardInterrupt:
        print("\n[退出] 已停止。")
        client.disconnect()


if __name__ == "__main__":
    main()
