import serial
import serial.tools.list_ports
import time
import sys

TOTAL_SIZE = 32 * 1024 * 1024
TARGET_PORT = "COM9"


def port_exists(name):
    return any(p.device == name for p in serial.tools.list_ports.comports())


def open_port_safely(name, baud=115200, timeout=0.3):
    ser = serial.Serial()
    ser.port = name
    ser.baudrate = baud
    ser.timeout = timeout
    ser.write_timeout = 1   # 防止 firmware 进入 dump 模式停 RX 后 write 卡死
    ser.dtr = False
    ser.rts = False
    ser.open()
    return ser


def run_extract():
    print("========================================")
    print("📦 Project Echo 黄金镜像提取工具 (同步版)")
    print("========================================")

    print(f"\n📢 请按下板子的 [RST] 键 (将检测 {TARGET_PORT} 断开重连)...")

    # 1. 等 COM9 消失：复位的明确信号
    while port_exists(TARGET_PORT):
        time.sleep(0.05)
    print("✅ 检测到设备复位，等待 USB 重新枚举...")

    # 2. 等 COM9 重新出现并尽快打开（DTR/RTS 关闭，避免再次触发复位）
    ser = None
    reopen_deadline = time.time() + 10
    while time.time() < reopen_deadline:
        if port_exists(TARGET_PORT):
            try:
                ser = open_port_safely(TARGET_PORT)
                break
            except Exception:
                ser = None
                time.sleep(0.05)
        else:
            time.sleep(0.02)

    if not ser:
        print(f"❌ {TARGET_PORT} 未在 10s 内重新出现，请检查 USB 连接。")
        return

    # 3. 等 firmware 主动喊话 ESP32_READY_CMD_WAIT
    #    收到这个就说明 usb_serial_jtag 驱动已装好、进入 10s 命令窗口
    print(">>> 等待 firmware 进入命令窗口...")
    sync_deadline = time.time() + 8
    synced = False
    while time.time() < sync_deadline:
        try:
            line = ser.readline().decode(errors='ignore')
        except Exception:
            line = ""
        if "ESP32_READY_CMD_WAIT" in line:
            synced = True
            break

    if not synced:
        print("❌ 8s 内未捕获到 ESP32_READY_CMD_WAIT，可能 firmware 未就绪。请重试 RST。")
        ser.close()
        return

    # 4. 已同步：发送 DUMP
    #    ⚠️ 不调 ser.flush()！firmware 一旦进入 dump 模式就不再读 RX FIFO，
    #    OS 待发缓冲被设备 NAK 永远排不空，flush() 会在 Windows 上死锁。
    #    单次 write 进 OS 缓冲后立即返回，不需要 flush。
    print(">>> 已同步，发送指令 DUMP...")
    try:
        ser.write(b"DUMP\n")
    except serial.SerialTimeoutException:
        print("⚠️ ser.write 超时（OS 写缓冲未排空），firmware 可能不再读 RX —— 继续等接收。")

    # 5. 等待 READY_TO_DUMP（诊断模式：把 firmware 原始输出实时回显）
    print(">>> 等待 READY_TO_DUMP — 下面是 firmware 的原始输出 ↓↓↓")
    print("-" * 60)
    ready = False
    raw_buf = b""
    ready_deadline = time.time() + 15  # 放长到 15s 方便诊断
    while time.time() < ready_deadline:
        try:
            chunk = ser.read(256)
        except Exception:
            chunk = b""
        if chunk:
            raw_buf += chunk
            try:
                sys.stdout.write(chunk.decode(errors='replace'))
                sys.stdout.flush()
            except Exception:
                pass
            if b"READY_TO_DUMP" in raw_buf:
                ready = True
                break
    print()
    print("-" * 60)

    if not ready:
        print(f"❌ 15s 内未收到 READY_TO_DUMP。共收到 {len(raw_buf)} 字节。")
        ser.close()
        return

    # ⚠️ 关键：把诊断循环里 ser.read 多读到的二进制数据捞出来
    #    READY_TO_DUMP\n 后面的字节是镜像开头，必须写进文件不能丢
    marker = b"READY_TO_DUMP\n"
    idx = raw_buf.rfind(marker)
    if idx < 0:
        # 极少见：只匹配到 READY_TO_DUMP（无 \n），保守处理
        idx = raw_buf.rfind(b"READY_TO_DUMP")
        leftover = raw_buf[idx + len(b"READY_TO_DUMP"):] if idx >= 0 else b""
        # 再剥掉可能的换行
        leftover = leftover.lstrip(b"\r\n")
    else:
        leftover = raw_buf[idx + len(marker):]
    if leftover:
        print(f"💾 从诊断缓冲找回 {len(leftover)} 字节镜像数据（避免头部缺失）")

    # 6. 接收 32MB 数据
    print("✅ 握手成功！开始接收 32MB 黄金镜像 (预计 2~5 分钟)...")
    ser.timeout = 1
    received = 0
    extract_start = time.time()
    last_print = 0.0
    stall_start = time.time()
    CHUNK = 4 * 1024  # 小块读，进度条刷新更频繁
    with open("storage_golden.bin", "wb") as f:
        # 先把诊断阶段抢读到的字节写入文件
        if leftover:
            to_write = leftover[:TOTAL_SIZE]
            f.write(to_write)
            received += len(to_write)
        while received < TOTAL_SIZE:
            chunk = ser.read(min(CHUNK, TOTAL_SIZE - received))
            if chunk:
                f.write(chunk)
                received += len(chunk)
                stall_start = time.time()
                now = time.time()
                if now - last_print >= 0.2 or received == TOTAL_SIZE:
                    last_print = now
                    elapsed = now - extract_start
                    speed = (received / 1024) / elapsed if elapsed > 0 else 0
                    percent = (received / TOTAL_SIZE) * 100
                    sys.stdout.write(
                        f"\r进度: [{percent:6.2f}%] {received/1024/1024:6.2f}MB | "
                        f"{speed:6.1f} KB/s | 用时 {elapsed:5.1f}s"
                    )
                    sys.stdout.flush()
            else:
                # 超过 10s 完全没数据 = 真卡死
                if time.time() - stall_start > 10:
                    print(f"\n❌ 已 10s 无数据，传输停滞。已收 {received/1024/1024:.2f}MB")
                    break
                if time.time() - extract_start > 600:
                    print("\n❌ 提取总超时 (>10min)。")
                    break

    if received >= TOTAL_SIZE:
        print(f"\n\n🎉 提取成功！耗时 {time.time() - extract_start:.1f} 秒")
        print("💾 文件已保存: storage_golden.bin")
    else:
        print(f"\n❌ 仅接收到 {received/1024/1024:.2f}MB，传输不完整。")

    ser.close()


if __name__ == "__main__":
    run_extract()
