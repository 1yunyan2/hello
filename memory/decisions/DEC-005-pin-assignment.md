---
name: ESP32-S3-WROOM-1-N16R8 引脚分配方案（v2）
date: 2026-04-03
status: 使用中
---

# ESP32-S3-WROOM-1-N16R8 引脚完整分析

> 芯片: ESP32-S3 (双核 Xtensa LX7, 240MHz)
> 模组: WROOM-1-N16R8 (16MB Octal Flash + 8MB Octal PSRAM)
> 封装: 共 49 个 GPIO (GPIO 0-48)
> 版本: v2 — 新增舵机、震动传感器、铜箔触摸、ST7789 显示屏，移除 LED

---

## 一、不可用引脚（16 个）— 绝对不能碰

### Octal SPI（Flash + PSRAM 内部占用，12 个）

| GPIO | 内部用途 |
|------|---------|
| 26 | SPICS1 (PSRAM CS) |
| 27 | SPIHD (IO4) |
| 28 | SPIWP (IO5) |
| 29 | SPICS0 (Flash CS) |
| 30 | SPICLK |
| 31 | SPIQ (IO0) |
| 32 | SPID (IO1) |
| 33 | SPI IO2 |
| 34 | SPI IO3 |
| 35 | SPI IO6 |
| 36 | SPI IO7 |
| 37 | SPI DQS |

### 模组未引出（4 个）

| GPIO | 说明 |
|------|------|
| 22 | WROOM-1 未引出 |
| 23 | WROOM-1 未引出 |
| 24 | WROOM-1 未引出 |
| 25 | WROOM-1 未引出 |

---

## 二、完整引脚分配方案（v2）

### A. 音频模块 — ES8311（7 个引脚）

| GPIO | 用途 | 总线 | 定义位置 | 状态 |
|------|------|------|---------|------|
| **8** | I2C SDA | I2C_NUM_0 | bsp_config.h:4 | ✅ 已用 |
| **15** | I2C SCL | I2C_NUM_0 | bsp_config.h:5 | ✅ 已用 |
| **17** | I2S MCLK | I2S_NUM_0 | bsp_config.h:6 | ✅ 已用 |
| **9** | I2S BCLK | I2S_NUM_0 | bsp_config.h:7 | ✅ 已用 |
| **5** | I2S WS/LRCK | I2S_NUM_0 | bsp_config.h:8 | ✅ 已用 |
| **4** | I2S DIN (Mic→ESP) | I2S_NUM_0 | bsp_config.h:9 | ✅ 已用 |
| **6** | I2S DOUT (ESP→Spk) | I2S_NUM_0 | bsp_config.h:10 | ✅ 已用 |

### B. 功放 + 按键（2 个引脚）

| GPIO | 用途 | 说明 | 状态 |
|------|------|------|------|
| **7** | PA 功放使能 | 高电平使能功放 | 🔵 规划中 |
| **0** | WiFi 清除按键 | Strapping 引脚，内部上拉，启动后作按键 | ✅ 已用 |

### C. ST7789 显示屏 — SPI（6 个引脚）🆕

| GPIO | 用途 | 说明 | 理由 |
|------|------|------|------|
| **47** | SPI SCLK | SPI 时钟 | 普通 GPIO，无限制 |
| **48** | SPI MOSI (SDA) | SPI 数据线 | 普通 GPIO，无限制 |
| **21** | CS 片选 | 低电平选中 | 普通 GPIO，无限制 |
| **38** | DC 数据/命令 | 高=数据，低=命令 | ✅ **改用 38 替代原来的 45**，避开 Strapping 引脚 |
| **16** | RST 复位 | 低电平复位 | 普通 GPIO，无限制 |
| **40** | BL 背光 | PWM 调光 | 普通 GPIO，LEDC 通道 |

> **为什么改 DC 引脚？** 原方案用 GPIO 45（Strapping 引脚），ST7789 初始化时 DC 线会被拉高拉低，如果在启动阶段发生，可能将 VDD_SPI 切换为 1.8V 导致 Flash 读取异常。GPIO 38 无任何限制，更安全。

### D. 三个舵机 — PWM/LEDC（3 个引脚）🆕

| GPIO | 用途 | LEDC 通道 | 理由 |
|------|------|----------|------|
| **39** | 舵机 1（如头部） | LEDC_CHANNEL_0 | 普通 GPIO，无限制，远离音频引脚避免干扰 |
| **41** | 舵机 2（如左翼） | LEDC_CHANNEL_1 | 普通 GPIO，无限制 |
| **42** | 舵机 3（如右翼） | LEDC_CHANNEL_2 | 普通 GPIO，无限制 |

> **舵机技术参数**: 标准舵机需要 50Hz PWM（周期 20ms），脉宽 0.5ms-2.5ms 对应 0°-180°。ESP32-S3 的 LEDC 外设有 8 个通道，3 个舵机只用 3 个，余量充足。
>
> **供电注意**: 舵机电流较大（每个 200-500mA），**不能直接用 ESP32 的 3.3V 供电**，必须外接 5V 电源，只共地和信号线连 ESP32。

### E. 震动感知传感器（1 个引脚）🆕

| GPIO | 用途 | 类型 | 理由 |
|------|------|------|------|
| **3** | 震动传感器 | 数字输入 (中断) | Strapping 引脚但启动后完全可用；支持 GPIO 中断，适合检测震动脉冲 |

> **传感器类型建议**:
> - **SW-420** (数字输出): 检测到震动输出高电平，简单好用，推荐
> - **ADXL345** (I2C 加速度计): 可以共用 GPIO 8/15 的 I2C 总线，不需额外引脚，但驱动复杂
>
> 如果用 SW-420 这类数字传感器，GPIO 3 配置为中断输入（上升沿触发），功耗极低。

### F. 三个感应铜箔 — Touch（3 个引脚）🆕

| GPIO | Touch 通道 | 用途 | 理由 |
|------|-----------|------|------|
| **1** | Touch1 | 铜箔触摸区域 1 | ✅ ADC1_CH0 + Touch1，最佳触摸引脚 |
| **2** | Touch2 | 铜箔触摸区域 2 | ✅ ADC1_CH1 + Touch2，最佳触摸引脚 |
| **10** | Touch10 | 铜箔触摸区域 3 | ✅ Touch10，无其他功能冲突 |

> **为什么选 GPIO 1, 2, 10？**
> - ESP32-S3 只有 GPIO 1-14 支持电容触摸感应，必须在这个范围内选
> - GPIO 1, 2 紧邻排列，PCB 布线方便
> - GPIO 10 与 1, 2 有间距，适合放在不同位置的铜箔
> - 避开了已用的 GPIO 4, 5, 6, 8, 9（音频）和 GPIO 15（I2C）
>
> **铜箔设计建议**:
> - 铜箔面积建议 10mm × 10mm 以上，越大灵敏度越高
> - 铜箔上方覆盖 0.5-2mm 的绝缘层（亚克力/塑料外壳）
> - 铜箔到 GPIO 的走线尽量短，远离大电流线路
> - 使用 ESP-IDF 的 `touch_pad` 驱动，支持自动校准和滤波

### G. 系统占用（2 个引脚）

| GPIO | 用途 | 说明 |
|------|------|------|
| **43** | UART0 TXD | 调试串口输出 |
| **44** | UART0 RXD | 调试串口输入 |

---

## 三、完整引脚状态图（v2）

```
ESP32-S3-WROOM-1-N16R8 引脚状态图 v2
新增: 舵机×3 + 震动 + 铜箔×3 + ST7789  移除: LED

GPIO  状态        用途                    模块
─────────────────────────────────────────────────
 0    🟢 已用     WiFi 清除按键            按键
 1    🟢 新增     铜箔触摸 1 (Touch1)      触摸感应
 2    🟢 新增     铜箔触摸 2 (Touch2)      触摸感应
 3    🟢 新增     震动传感器 (中断输入)     传感器
 4    🟢 已用     I2S DIN                  音频
 5    🟢 已用     I2S WS                   音频
 6    🟢 已用     I2S DOUT                 音频
 7    🔵 规划     PA 功放使能              音频
 8    🟢 已用     I2C SDA                  音频
 9    🟢 已用     I2S BCLK                 音频
10    🟢 新增     铜箔触摸 3 (Touch10)     触摸感应
11    ⚪ 空闲     预留                     —
12    ⚪ 空闲     预留                     —
13    ⚪ 空闲     预留                     —
14    ⚪ 空闲     预留                     —
15    🟢 已用     I2C SCL                  音频
16    🟢 启用     ST7789 RST               显示屏
17    🟢 已用     I2S MCLK                 音频
18    ⚪ 空闲     预留                     —
19    ⚠️ 冲突?    BLE RTS / USB_D-         —
20    ⚪ 空闲     预留 (USB_D+)            —
21    🟢 启用     ST7789 CS                显示屏
22    🚫 不可用   模组未引出
23    🚫 不可用   模组未引出
24    🚫 不可用   模组未引出
25    🚫 不可用   模组未引出
26-37 🚫 不可用   Octal SPI (Flash+PSRAM)
38    🟢 新增     ST7789 DC（数据/命令）    显示屏
39    🟢 新增     舵机 1（头部）            动力
40    🟢 启用     ST7789 背光 (PWM)        显示屏
41    🟢 新增     舵机 2（左翼）            动力
42    🟢 新增     舵机 3（右翼）            动力
43    🟡 系统     UART0 TXD                调试
44    🟡 系统     UART0 RXD                调试
45    ⚪ 释放     原 LCD DC，已改用 38      —
46    ⚪ 释放     原 LED，已移除            —
47    🟢 启用     ST7789 SPI SCLK          显示屏
48    🟢 启用     ST7789 SPI MOSI          显示屏

─────────────────────────────────────────────────
图例:
🟢 已用/新增 (22个)  🔵 规划 (1个)   🟡 系统 (2个)
⚠️ 冲突 (1个)       ⚪ 空闲 (7个)   🚫 不可用 (16个)
```

---

## 四、统计（v2）

| 类别 | 数量 | GPIO 编号 |
|------|------|----------|
| 音频 (I2C+I2S) | 7 | 4, 5, 6, 8, 9, 15, 17 |
| 显示屏 (ST7789 SPI) | 6 | 16, 21, 38, 40, 47, 48 |
| 舵机 (PWM) | 3 | 39, 41, 42 |
| 触摸铜箔 (Touch) | 3 | 1, 2, 10 |
| 震动传感器 | 1 | 3 |
| 按键 | 1 | 0 |
| PA 功放 (规划中) | 1 | 7 |
| 系统 (UART0) | 2 | 43, 44 |
| 冲突 | 1 | 19 |
| **空闲可用** | **7** | **11, 12, 13, 14, 18, 20, 45** |
| 释放 (Strapping) | *(含在空闲中)* | 45, 46 |
| 不可用 | 16 | 22-37 |
| **总计** | **49** | |

> 使用率: 24/33 可用引脚 = **73%**，剩余 7 个空闲引脚可用于未来扩展。
> 其中 GPIO 45, 46 是 Strapping 引脚（可用但需注意启动电平）。

---

## 五、建议更新 bsp_config.h

```c
#pragma once

// ===== 音频模块 (ES8311) =====
// #define BSP_CODEC_PA_PIN         7    // PA 功放使能（未启用）
#define BSP_CODEC_SDA_PIN           8    // I2C SDA
#define BSP_CODEC_SCL_PIN           15   // I2C SCL
#define BSP_CODEC_MCLK_PIN          17   // I2S MCLK
#define BSP_CODEC_BCLK_PIN          9    // I2S BCLK
#define BSP_CODEC_WS_PIN            5    // I2S WS/LRCK
#define BSP_CODEC_DIN_PIN           4    // I2S DIN (Mic→ESP)
#define BSP_CODEC_DOUT_PIN          6    // I2S DOUT (ESP→Speaker)
#define BSP_CODEC_SAMPLE_RATE       16000
#define BSP_CODEC_BITS_PER_SAMPLE   16

// ===== ST7789 显示屏 (SPI) =====
#define BSP_LCD_MOSI_PIN            48   // SPI MOSI
#define BSP_LCD_SCLK_PIN            47   // SPI SCLK
#define BSP_LCD_CS_PIN              21   // 片选
#define BSP_LCD_DC_PIN              38   // 数据/命令（改用 38 避开 Strapping）
#define BSP_LCD_RST_PIN             16   // 复位
#define BSP_LCD_BK_PIN              40   // 背光 (PWM)
#define BSP_LCD_WIDTH               240
#define BSP_LCD_HEIGHT              320

// ===== 舵机 (PWM/LEDC) =====
#define BSP_SERVO_1_PIN             39   // 舵机 1（头部）
#define BSP_SERVO_2_PIN             41   // 舵机 2（左翼）
#define BSP_SERVO_3_PIN             42   // 舵机 3（右翼）

// ===== 触摸铜箔 (Touch) =====
#define BSP_TOUCH_PAD_1_PIN         1    // Touch1
#define BSP_TOUCH_PAD_2_PIN         2    // Touch2
#define BSP_TOUCH_PAD_3_PIN         10   // Touch10

// ===== 震动传感器 =====
#define BSP_VIBRATION_PIN           3    // 数字输入（GPIO 中断）

// ===== 按键 =====
// CLEAR_WIFI_BUTTON_PIN 定义在 bsp_wifi.c 中 (GPIO 0)
```

---

## 六、硬件注意事项

### 舵机供电
- ❌ 不能用 ESP32 的 3.3V 给舵机供电
- ✅ 舵机用独立 5V 电源，与 ESP32 共 GND
- 信号线 (GPIO 39/41/42) 直连舵机信号端即可（3.3V 逻辑电平对多数舵机兼容）

### 触摸铜箔抗干扰
- 铜箔走线远离 I2S 数据线（高频信号会干扰触摸检测）
- GPIO 1, 2 靠近音频引脚 4, 5, 6，PCB 布局时注意**地线隔离**
- 建议触摸走线与音频走线之间插入一根 GND 走线

### ST7789 SPI 速度
- ESP32-S3 SPI 最高 80MHz，ST7789 支持到约 62.5MHz
- 建议初始用 40MHz，稳定后可尝试提高
