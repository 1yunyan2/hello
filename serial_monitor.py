import serial
import sys

PORT = "COM5"
BAUD = 115200

print("=" * 50)
print(f"  Echo2 串口监视器  {PORT} @ {BAUD}")
print("  退出: Ctrl+C")
print("=" * 50)

try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    print(f"已连接 {PORT}\n")
    while True:
        line = ser.readline()
        if line:
            try:
                print(line.decode("utf-8", errors="replace"), end="")
            except Exception:
                print(line)
except serial.SerialException as e:
    print(f"\n错误: {e}")
    print("请确认设备已插入且 COM5 正确")
except KeyboardInterrupt:
    print("\n已退出")
finally:
    try:
        ser.close()
    except Exception:
        pass
