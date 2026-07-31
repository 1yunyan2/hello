import serial
import serial.tools.list_ports
import time
import os

BIN_FILE = 'storage.bin'
CHUNK_SIZE = 128 * 1024
# ⭐ 设备端报告的真实 Flash 容量(握手时读到,单位字节)
DEVICE_FLASH_SIZE = None


def find_esp32_port():
    """寻找 ESP32 设备"""
    ports = serial.tools.list_ports.comports()
    for port in ports:
        desc = port.description.upper()
        if any(k in desc for k in ["USB SERIAL", "USB 串行", "ESP32", "JTAG"]):
            return port.device
    return None


def wait_for_port(timeout=None):
    """等待 ESP32 串口出现。timeout=None 表示一直等；超时返回 None。
    复位后 USB-Serial-JTAG 会断开重枚举，端口会短暂消失再回来，故需要重新等。"""
    t0 = time.time()
    while True:
        p = find_esp32_port()
        if p:
            return p
        if timeout is not None and (time.time() - t0) > timeout:
            return None
        time.sleep(0.3)


def hard_reset_board(port):
    """⭐ 软件复位板子，替代已去除的 RST 按键。

    固件的强制烧录窗口只在「开机后 5 秒」内存在（bsp_flash.c 的
    BURN_FORCE_WINDOW_MS），设备一旦跑起来就再也握不上手。没有 RST 键时，
    只能由 PC 侧通过 USB 的 DTR/RTS 把芯片拉复位，人为制造这个窗口。

    两种硬件都兼容：
      A. ESP32-S3 内置 USB-Serial-JTAG：RTS 控制 EN(CHIP_PU)，DTR 控制 GPIO0；
         复位瞬间 USB 会断开重新枚举，端口短暂消失属正常。
      B. 外置 USB-UART(CH340/CP210x) + 自动下载电路：RTS 拉低 EN 即复位。

    关键：全程保持 DTR=False(GPIO0 高)，确保复位后芯片跑用户固件，
    而不是掉进 ROM 下载模式（掉进去就没有烧录窗口了）。
    """
    try:
        s = serial.Serial()
        s.port = port
        s.baudrate = 115200
        s.timeout = 0.2
        # open 之前先摆好电平，避免 open 瞬间抖动误触发
        s.dtr = False
        s.rts = False
        s.open()
        time.sleep(0.05)
        s.dtr = False
        s.rts = True      # ① EN 拉低 → 芯片进入复位
        time.sleep(0.15)
        s.dtr = False
        s.rts = False     # ② EN 释放 → 芯片出复位，从 flash 正常启动
        time.sleep(0.05)
        s.close()
        return True
    except Exception as e:
        print(f"⚠️  DTR/RTS 复位失败: {e}")
        return False


def esptool_reset(port):
    """兜底复位：直接借 esptool 的复位逻辑（和 idf.py flash 用的是同一套，
    带多种时序重试，比裸 DTR/RTS 更强）。--after hard_reset 会在结束时
    复位芯片并运行用户固件，正好落进烧录窗口。"""
    import subprocess
    import sys
    print(">>> 兜底方案：调用 esptool 强制复位（等同 idf.py flash 的自动复位）...")
    try:
        r = subprocess.run(
            [sys.executable, "-m", "esptool", "--port", port,
             "--after", "hard_reset", "chip_id"],
            capture_output=True, text=True, timeout=90)
        if r.returncode == 0:
            print(">>> ✅ esptool 复位成功")
            return True
        print(f">>> ❌ esptool 复位失败:\n{r.stdout[-500:]}{r.stderr[-500:]}")
        return False
    except Exception as e:
        print(f"⚠️  esptool 复位异常: {e}")
        return False


def repack_storage_bin(size_bytes):
    """⭐ 容量不匹配时自动调用 2.py <设备真实容量> 重新打包 storage.bin，
    免去人工判断 16M/32M 再手动敲命令这一步（3.py 已经从握手里读到真实容量了）。"""
    import subprocess
    import sys
    print(f">>> 正在调用: python 2.py {size_bytes}")
    try:
        r = subprocess.run(
            [sys.executable, "2.py", str(size_bytes)],
            capture_output=True, text=True, encoding='utf-8', errors='replace',
            timeout=300)
        print(r.stdout[-2000:])
        if r.returncode != 0:
            print(r.stderr[-1000:])
            return False
        return True
    except Exception as e:
        print(f"⚠️  调用 2.py 异常: {e}")
        return False


def run_burn():
    # ⚠️ 函数体内对 DEVICE_FLASH_SIZE 有赋值，不声明 global 的话 Python 会把它
    #    整个当成局部变量，模块级第 9 行那个全局量形同虚设（旧版就是这样，
    #    只是碰巧每轮都先赋 None 才没报 UnboundLocalError）。
    global DEVICE_FLASH_SIZE

    print("========================================")
    print("🚀 Project Echo 产线自动化烧录工具 v3.0 (防断联无敌版)")
    print("========================================")

    if not os.path.exists(BIN_FILE):
        print(f"❌ 错误：在当前目录下找不到 {BIN_FILE}")
        return

    attempt = 0
    while True:
        attempt += 1
        print("\n📢 [等待中] 请通过 USB 连接机器人...")

        port = wait_for_port()
        print(f"✅ 发现设备: {port}")

        # ⭐ 无 RST 按键方案：脚本自己把板子复位，制造固件的 5 秒强制烧录窗口。
        #    第 1 轮用轻量的 DTR/RTS 复位；抓不到握手就从第 2 轮起换 esptool 兜底。
        if attempt == 1:
            print(">>> 正在自动复位板子（DTR/RTS，无需按 RST）...")
            hard_reset_board(port)
        else:
            esptool_reset(port)

        # 复位后 USB-Serial-JTAG 会重新枚举，端口可能短暂消失，等它回来
        port = wait_for_port(15) or port

        # 每轮重置，防止设备没上报 FLASH_SIZE 时后面读到未赋值的局部变量
        DEVICE_FLASH_SIZE = None

        ser = serial.Serial()
        ser.port = port
        ser.baudrate = 115200
        ser.timeout = 10
        ser.write_timeout = 15   # 防止卡死
        # ⚠️⚠️ DTR 必须为 True，否则 PC 发出去的 START 根本进不了芯片！
        #   ESP32-S3 内置 USB-Serial-JTAG 是标准 CDC-ACM 设备：DTR 表示
        #   「host 端已打开这个串口」。DTR=False 时设备侧认为对端没打开，
        #   host 写入的字节不会被交付到 USB-JTAG 的 RX FIFO，于是固件的
        #   usb_serial_jtag_ll_read_rxfifo() 永远读到 0 字节。
        #   注意方向是**不对称的**：设备→PC 的日志照常收得到，只有
        #   PC→设备被挡住。症状就是「日志一切正常、START 石沉大海」：
        #   脚本打印了"激活指令已送达"，设备却继续刷 ESP32_READY_CMD_WAIT
        #   直到 5 秒窗口耗尽，最后走正常挂载（日志无 STARTING_ERASE）。
        #   旧注释写 "防止重启" 是把 USB-JTAG 当成了 CH340/CP210x —— 那类
        #   外置芯片的 DTR 才接 GPIO0/EN 会引起复位；S3 内置 USJ 的 DTR
        #   纯粹是 CDC 流控信号，置 True 不会复位芯片。
        ser.dtr = True
        ser.rts = False         # RTS 保持 False，避免拉低 EN 造成复位

        try:
            # 1. 智能判断，避免 "Port is already open" 报错
            if not ser.is_open:
                ser.open()

            # ⭐ open() 之后再显式置一次：部分驱动在 open 时会按默认值
            #    重新驱动控制线，open 前设的值可能被覆盖。
            ser.dtr = True
            ser.rts = False

            # 2. 给底层驱动一点缓冲时间（刚复位过，要尽早开读，别错过窗口）
            time.sleep(0.3)

            print(">>> 正在捕捉板子信号（最多等 30 秒，覆盖整个开机过程）...")

            # 3. 聪明的握手等待：听板子喊话，同时抓 FLASH_SIZE
            #    ⚠️ 这里绝不能按「行数」计数：ESP32 开机日志本身就有 50 行
            #    （esp_image/psram/heap_init/...），旧版 for range(50) 的配额会被
            #    开机日志吃光，恰好在 spi_flash 认出外挂 flash 那一行退出，
            #    而 ESP32_READY_CMD_WAIT 就在再往后一两行 —— 临门一脚放弃。
            #    改为按「时间」计：短 timeout + 30 秒 deadline，读多少行都不怕。
            # ⚠️ 固件强制烧录窗口只有 1000ms（bsp_flash.c: BURN_FORCE_WINDOW_MS），
            # 握手成功后必须立刻发 START 抢窗口 —— 之前的版本先花时间等
            # FLASH_SIZE 那一行（最坏 8*0.5s=4s），窗口早关了，START 发出去
            # 设备根本收不到，只会误判"没命令"继续正常开机（症状：日志直接
            # 跳到挂载成功→WiFi→LVGL，没有 STARTING_ERASE）。
            # 现在策略：边读边记 FLASH_SIZE，但一旦看到 ESP32_READY_CMD_WAIT
            # 立刻发 START，FLASH_SIZE 校验挪到发送之后（若之前已读到就先校验）。
            synced = False
            flash_size_reported = None
            ser.timeout = 0.5          # 握手阶段短超时，保证 deadline 精确
            deadline = time.time() + 30
            while time.time() < deadline:
                try:
                    line = ser.readline().decode(errors='ignore').strip()
                    if line.startswith("FLASH_SIZE:"):
                        flash_size_reported = int(line.split(":")[1])
                        continue
                    if "ESP32_READY_CMD_WAIT" in line:
                        synced = True
                        break
                    elif line:
                        print(f"[设备日志]: {line}")
                except Exception:
                    pass

            if not synced:
                ser.timeout = 10
                print("❌ 未捕获到设备的握手信号。准备重新尝试...")
                continue

            # ⭐ 先抢窗口发 START，越快越好
            print(">>> ✅ 捕获到信号！发送激活指令 START...")
            # ⚠️ 必须 flush()：pyserial 的 write 是带缓冲的，不 flush 时数据可能
            #    还躺在 OS 发送队列里没真正推上 USB 线，而代码已经转去做读操作，
            #    等真正发出去时固件窗口早关了。症状：Python 显示"已发送"，
            #    设备却把窗口喊满一次都没收到命令。
            # ⚠️ 也不要再无脑吞异常：write 失败必须让人看见，否则会误判成
            #    "发了但设备没收到"，白白往固件侧查半天。
            try:
                n = ser.write(b"START\n")
                ser.flush()
                print(f">>> START 已写入串口 ({n} 字节) 并 flush")
            except Exception as e:
                print(f"❌ START 发送失败: {e}")

            # ⭐⭐ 确认设备真的收到了 START，而不是盲目宣布"已送达"。
            #   固件收到 START 后第一件事就是 printf("STARTING_ERASE")
            #   （bsp_flash.c: start_production_burning 开头），这是唯一可信的
            #   "命令已生效"证据。旧版不做确认，一旦命令没送进去（比如 DTR 没
            #   置位），脚本照样打印"激活指令已送达"，然后卡死在等 READY_FOR_DATA
            #   的死循环里，而设备早就正常挂载启动了 —— 误导性极强。
            #   固件强制窗口共 5s、每 500ms 重喊一次锚点，所以这里边等边补发
            #   START，把剩余窗口全部利用上；顺带继续收 FLASH_SIZE。
            erase_started = False
            confirm_deadline = time.time() + 6.0
            last_resend = 0.0
            while time.time() < confirm_deadline:
                try:
                    line = ser.readline().decode(errors='ignore').strip()
                    if not line:
                        continue
                    if "STARTING_ERASE" in line:
                        erase_started = True
                        print(f"[设备日志]: {line}")
                        break
                    if line.startswith("FLASH_SIZE:"):
                        flash_size_reported = int(line.split(":")[1])
                        continue
                    print(f"[设备日志]: {line}")
                    # 设备还在喊锚点 = 命令尚未生效，趁窗口没关抓紧补发
                    if "ESP32_READY_CMD_WAIT" in line and (time.time() - last_resend) > 0.2:
                        try:
                            ser.write(b"START\n")
                            ser.flush()
                            last_resend = time.time()
                        except Exception:
                            pass
                except Exception:
                    pass

            ser.timeout = 10  # 握手结束，恢复长超时（擦除阶段可能几十秒无输出）

            if not erase_started:
                print("\n" + "=" * 60)
                print("❌ START 已写出，但设备始终没有回应 STARTING_ERASE。")
                print("   说明命令没能送进芯片，最常见原因是串口 DTR 未置位：")
                print("   USB-Serial-JTAG 在 DTR=False 时不会把 host 写入的字节")
                print("   交付给设备（日志能收到，命令发不进去）。")
                print("   设备本轮将按正常流程挂载启动，稍后自动重试...")
                print("=" * 60 + "\n")
                continue

            # ⭐ 读到了设备真实 Flash 容量，检查当前 storage.bin 是否匹配
            if flash_size_reported:
                DEVICE_FLASH_SIZE = flash_size_reported
                print(f">>> ✅ 检测到设备 Flash 容量: {DEVICE_FLASH_SIZE/1024/1024:.0f}MB "
                      f"({DEVICE_FLASH_SIZE} 字节)")
                bin_size = os.path.getsize(BIN_FILE)
                if bin_size != DEVICE_FLASH_SIZE:
                    print(f"\n{'='*60}")
                    print(f"⚠️  容量不匹配：storage.bin={bin_size/1024/1024:.0f}MB，"
                          f"设备 Flash={DEVICE_FLASH_SIZE/1024/1024:.0f}MB")
                    print(f"   注意：设备可能已经在擦除中（START 已发出）。")
                    print(f"   本轮结束后将自动调用 2.py 用设备真实容量重新打包...")
                    print(f"{'='*60}\n")
                    if 'ser' in locals() and ser.is_open:
                        ser.close()
                    if not repack_storage_bin(DEVICE_FLASH_SIZE):
                        print("❌ 自动重新打包失败，请手动检查 2.py 报错后重试。")
                        return
                    print(">>> ✅ 重新打包完成，等待设备重启后自动重试烧录...")
                    time.sleep(2)
                    continue

            # 走到这里 erase_started 必为 True（否则上面已 continue），
            # 即设备确实回了 STARTING_ERASE，擦除是真的开始了。
            print(">>> ✅ 已确认设备进入擦除（STARTING_ERASE），进度由设备日志播报...")

            # 4. 等待擦除完成的反馈
            while True:
                line = ser.readline().decode(errors='ignore').strip()
                if "READY_FOR_DATA" in line:
                    break
                elif line:
                    print(f"[设备日志]: {line}")

            # 5. 开始同步文件（只烧设备容量大小，防止越界）
            file_size = os.path.getsize(BIN_FILE)
            if DEVICE_FLASH_SIZE and file_size > DEVICE_FLASH_SIZE:
                file_size = DEVICE_FLASH_SIZE
                print(f">>> storage.bin 比芯片大，只烧前 {file_size/1024/1024:.0f}MB")
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
