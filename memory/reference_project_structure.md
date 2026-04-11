---
name: 项目目录结构
description: Echo2 完整代码目录树 + 模块职责说明（2026-04-11 基准）
type: reference
originSessionId: d00b77de-da96-4f9b-a731-7b2efcff7c2f
---
# Echo2 项目目录结构

> 基准日期：2026-04-11  
> 平台：ESP32-S3 / ESP-IDF v5.3.4

---

## 完整目录树

```
Echo2/
├── CMakeLists.txt                  # 顶层构建入口（ESP-IDF 项目根）
├── sdkconfig                       # menuconfig 生成的配置（勿手改）
├── sdkconfig.defaults              # 默认配置预设
├── partitions.csv                  # Flash 分区表
├── dependencies.lock               # 组件版本锁定文件
├── idf_component.yml               # (main/) 组件依赖声明
│
├── main/                           # 主应用源码目录
│   ├── CMakeLists.txt              # 注册所有源文件到编译系统
│   ├── main.c                      # 程序入口：app_main()
│   ├── application.c / .h          # 应用层：系统初始化编排、事件总线
│   ├── object.h                    # 全局类型定义（枚举、公共结构体）
│   │
│   ├── bsp/                        # Board Support Package（硬件抽象层）
│   │   ├── bsp_config.h            # 所有 GPIO / 参数宏定义（引脚分配总表）
│   │   ├── bsp_board.c / .h        # 全局单例 bsp_board_t，EventGroup 位定义
│   │   ├── bsp_codec.c             # ES8311 音频编解码芯片（I2C+I2S 双总线）
│   │   ├── bsp_wifi.c              # WiFi STA 连接 + BLE 配网（Bluedroid）
│   │   ├── bsp_lcd.c               # ST7789 LCD 240×320 SPI 驱动 + 背光控制
│   │   ├── bsp_touch.c             # 触摸屏中断注册（GPIO 3），事件→会话队列
│   │   └── bsp_servo.c             # 舵机 PWM（LEDC 外设，GPIO 18，50Hz）
│   │
│   ├── wake_word/                  # 唤醒词引擎
│   │   ├── custom_wake_word.c / .h # MultiNet 离线唤醒词检测，AFE 管道接入
│   │
│   ├── audio/                      # 音频数据处理层
│   │   ├── audio_processor.c / .h  # AFE 降噪+VAD，PCM 环形缓冲（四层设计）
│   │   ├── audio_encoder.c / .h    # Opus 编码器（CBR 24kbps / 60ms / 复杂度3）
│   │   └── audio_decoder.c / .h    # Opus 解码器，输出 PCM 至 I2S DAC 播放
│   │
│   ├── protocol/                   # 网络协议层
│   │   ├── auth.c / .h             # Token 认证（accessToken 2h过期，主动+被动双刷）
│   │   ├── websocket_client.c / .h # WebSocket 客户端（JSON 握手，Opus 音频流）
│   │   └── mqtt_protocol.c / .h    # MQTT 协议（备用/OTA 通道，当前未主用）
│   │
│   ├── session/                    # 会话状态机
│   │   └── session.c / .h          # 四状态机：IDLE→LISTENING→THINKING→SPEAKING
│   │
│   └── ui/                         # 用户界面层（2026-04-11 新增）
│       └── interaction.c / .h      # LCD 动画状态联动（待开发完善）
│
├── managed_components/             # ESP-IDF 组件管理器下载的依赖
│   └── ...                         # esp_opus, esp_afe, esp-sr 等
│
├── memory/                         # Claude 记忆系统（非固件代码）
│   └── ...
│
└── .vscode/                        # VS Code 调试/智能提示配置
    ├── c_cpp_properties.json
    ├── launch.json
    └── settings.json
```

---

## 模块依赖关系

```
main.c
  └─► application.c          ← 系统编排入口
        ├─► bsp/              ← 硬件初始化（codec / wifi / lcd / touch / servo）
        ├─► wake_word/        ← 唤醒词检测（依赖 AFE / MultiNet）
        ├─► session/          ← 会话状态机（驱动整个对话流程）
        │     ├─► audio/      ← 编解码（session 调度采集和播放）
        │     ├─► protocol/   ← WS 连接 + Token 刷新
        │     └─► ui/         ← LCD 动画联动
        └─► protocol/auth.c   ← Token 初始化（application 层触发）
```

---

## 核心数据流

```
麦克风(I2S) → AFE降噪/VAD → Opus编码 → WebSocket发送
                                            ↓
                                       服务端(ASR+LLM+TTS)
                                            ↓
WebSocket接收 ← Opus音频流 ← TTS响应
     ↓
 Opus解码 → I2S DAC → 扬声器
     ↓
 LCD动画（同步 SPEAKING 状态）
```

---

## 关键文件速查

| 需要改什么 | 找哪个文件 |
|-----------|-----------|
| GPIO 引脚分配 | `bsp/bsp_config.h` |
| 唤醒词阈值 | `wake_word/custom_wake_word.c` |
| Opus 编码参数 | `audio/audio_encoder.c` |
| WebSocket 握手/消息格式 | `protocol/websocket_client.c` |
| Token 刷新逻辑 | `protocol/auth.c` |
| 会话状态流转 | `session/session.c` |
| LCD 显示内容 | `bsp/bsp_lcd.c` + `ui/interaction.c` |
| 系统初始化顺序 | `application.c` |
