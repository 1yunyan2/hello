import serial
import serial.tools.list_ports
import time
import os

BIN_FILE = 'storage.bin'
CHUNK_SIZE = 128 * 1024 

def find_esp32_port():
    """寻找 ESP32 设备"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.upper()
        if any(k in desc for k in ["USB SERIAL", "USB 串行", "ESP32", "JTAG"]):
            return port.device
    return None

def run_burn():
    print("========================================")
    print("🚀 Project Echo 产线自动化烧录工具 v3.0 (防断联无敌版)")
    print("========================================")

    if not os.path.exists(BIN_FILE):
        print(f"❌ 错误：在当前目录下找不到 {BIN_FILE}")
        return

    while True:
        print("\n📢 [等待中] 请通过 USB 连接机器人...")
        
        port = None
        while not port:
            port = find_esp32_port()
            time.sleep(0.5)
        
        print(f"✅ 发现设备: {port}")
        
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 10
        ser.write_timeout = 15   # 防止卡死
        ser.dtr = False         # 防止重启
        ser.rts = False
        
        try:
            # 1. 智能判断，避免 "Port is already open" 报错
            if not ser.is_open:
                ser.open()
            
            # 2. 给底层驱动充足的缓冲时间
            time.sleep(1.5) 
            
            print(">>> 正在捕捉板子信号，如果没反应，请按一下板子的 RST 键...")
            
            # 3. 聪明的握手等待：听板子喊话
            synced = False
            for _ in range(50): # 等待约 5 秒
                try:
                    line = ser.readline().decode(errors='ignore').strip()
                    if "ESP32_READY_CMD_WAIT" in line:
                        synced = True
                        break
                    elif line:
                        print(f"[设备日志]: {line}")
                except Exception:
                    pass
            
            if not synced:
                print("❌ 未捕获到设备的握手信号。准备重新尝试...")
                continue
                
            print(">>> ✅ 捕获到信号！发送激活指令 START...")
            try:
                ser.write(b"START\n")
            except Exception:
                pass # 忽略小缓冲错误
                
            print(">>> 激活指令已送达！芯片正在全盘擦除 (约 81s)，进度由设备日志播报...")

            # 4. 等待擦除完成的反馈
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if "READY_FOR_DATA" in line:
                    break
                elif line:
                    print(f"[设备日志]: {line}")
            
            # 5. 开始同步文件
            file_size = os.path.getsize(BIN_FILE)
            start_time = time.time()
            sent = 0

            print(">>> ✅ 设备就绪！开始高速同步数据...")
            with open(BIN_FILE, 'rb') as f:
                while sent < file_size:
                    data = f.read(CHUNK_SIZE)
                    ser.write(data)
                    ser.flush()
                    while True:
                        resp = ser.readline().decode(errors='ignore').strip()
                        if "ACK" in resp:
                            sent += len(data)
                            print(f"\r进度: [{(sent/file_size)*100:6.2f}%] 已同步: {sent/1024/1024:6.2f}MB", end="")
                            break
            
            duration = time.time() - start_time
            print(f"\n🎉 烧录成功！耗时: {duration:.1f} 秒")
            print("🔔 [请拔线]：检测到当前设备尚未拔除，等待拔出...")

            # 等待设备拔出
            while True:
                current_ports = [p.device for p in serial.tools.list_ports.comports()]
                if port not in current_ports:
                    print("✅ 设备已移除。")
                    break
                time.sleep(0.5)

            print("----------------------------------------")
            
        except Exception as e:
            print(f"\n❌ 传输异常: {e}")
            print("请检查线缆连接后重试。")
            
        finally:
            # 🚀 无论发生什么错误，强制释放端口，根绝残留！
            if 'ser' in locals() and ser.is_open:
                ser.close()
            time.sleep(1)

if __name__ == "__main__":
    run_burn()