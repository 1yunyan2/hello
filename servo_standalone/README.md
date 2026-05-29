# servo_standalone — 独立舵机基础驱动

ESP32 / ESP32-S3 通用的模拟舵机驱动，**零第三方依赖**，只用 ESP-IDF 自带的 LEDC。
拿到这个文件夹就能直接编译使用，不带任何项目业务/触摸耦合代码。

## 功能
- `servo_driver_init()` — 初始化任意 1~8 路舵机，上电自动缓慢归中到 90°
- `servo_move_smooth()` — 平滑插值运动（逐度移动，可调速度），不抽搐
- `servo_write_angle()` — 瞬间到位
- `servo_read_angle()` — 读取当前角度
- `servo_move_all_parallel()` — 多轴同时同步运动
- 内置软限位保护（每路可设 min/max 角度，超限自动裁剪）
- 每通道独立互斥锁，多任务调用线程安全

## 文件
| 文件 | 作用 |
|------|------|
| `servo_driver.h` | 对外 API 和速度宏 |
| `servo_driver.c` | 驱动实现（LEDC 50Hz PWM） |
| `example_main.c` | 使用示例（可直接当 app_main） |
| `CMakeLists.txt` | ESP-IDF 组件注册脚本 |

## 接线
- 舵机信号线 → `servo_channel_cfg_t.gpio_num` 指定的 GPIO
- 舵机电源用**独立 5V**（不要用板子 3V3 直供，电流不够）
- **舵机电源地必须与 ESP32 的 GND 共地**

## 用法

### 方式 A：作为独立组件
1. 把整个 `servo_standalone/` 文件夹拷进工程的 `components/` 目录。
2. 在需要用的源文件里 `#include "servo_driver.h"`。
3. `idf.py build` 即可，组件会被自动发现。

### 方式 B：直接放进 main
把 `servo_driver.c` / `servo_driver.h` 拷进 `main/`，
并在 `main/CMakeLists.txt` 的 `SRCS` 里加上 `servo_driver.c`、
`REQUIRES`（或 `PRIV_REQUIRES`）里加上 `driver`。

## 最小示例
```c
#include "servo_driver.h"

servo_channel_cfg_t chans[] = {
    { .gpio_num = 9, .min_angle = 0.0f, .max_angle = 180.0f },
};
servo_driver_init(chans, 1);
servo_move_smooth(0, 150.0f, SERVO_SPEED_MID); // 中速转到 150°
```

## 参数调整
若舵机角度对不上，改 `servo_driver.c` 顶部：
- `SERVO_MIN_PULSE_US` / `SERVO_MAX_PULSE_US`（脉宽，默认 500~2400μs）
- `SERVO_MAX_ANGLE`（行程，默认 180°）

速度档位见 `servo_driver.h`：`SERVO_SPEED_FAST/MID/SLOW` 等，数值是「每度等待毫秒数」，越大越慢。
