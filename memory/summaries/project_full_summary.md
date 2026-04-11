---
name: Echo2 项目全量技术总结
description: 基于 main/ 目录代码生成的总分总结构技术文档，含5模块详解、设计亮点、技术难点
type: project
---

<!-- 版本 v1 — 2026-04-10 首次生成 -->

# Echo2 项目总体技术文档

## 一、总述

### 1.1 项目核心定位

Echo2 是 Echopals 品牌面向消费者的**嵌入式智能语音交互产品**，运行于 ESP32-S3 芯片。
核心价值：**离线唤醒 → 在线 ASR/LLM/TTS → 实时语音播放**的完整语音对话闭环，
具备多轮连续对话、语音打断、MQTT 远程管理等能力。

### 1.2 技术栈

| 层次 | 技术选型 |
|------|---------|
| 芯片 | ESP32-S3（双核 Xtensa LX7，8 MB SPIRAM） |
| 固件框架 | ESP-IDF v5.3.4 |
| 编程语言 | 纯 C（snake_case 命名规范） |
| 音频编解码 | ES8311（I2S 接口），OPUS 压缩（24 kbps，60 ms 帧） |
| 离线唤醒 | 乐鑫 AFE（音频前端）+ MultiNet6（命令词识别引擎） |
| 云端通信 | WebSocket（二进制帧传音频，JSON 帧传控制消息） |
| 设备管理 | MQTT（心跳、唤醒词远程更新） |
| 存储 | NVS Flash（凭证、唤醒词、WiFi 配置持久化） |
| 实时调度 | FreeRTOS（EventGroup / Queue / Timer / RingBuffer） |
| 显示 | ST7789 LCD 240×320，RGB565 色彩 |

### 1.3 整体架构

```
┌──────────────────────────────────────────────────────────────────────┐
│  应用层  application.c  →  启动编排 + 模块依赖管理                   │
├────────────────┬─────────────────┬────────────────┬──────────────────┤
│  session/      │  audio/         │  wake_word/    │  protocol/       │
│  会话状态机    │  编解码管道      │  离线唤醒引擎  │  网络协议栈      │
│  多轮对话      │  4 层环形缓冲   │  AFE + MultiNet│  WS + MQTT + Auth│
├────────────────┴─────────────────┴────────────────┴──────────────────┤
│  bsp/  硬件抽象层（ES8311 / LCD / WiFi / NVS / GPIO）                 │
├──────────────────────────────────────────────────────────────────────┤
│  硬件  ESP32-S3 + ES8311 + ST7789 LCD + 麦克风 + 扬声器              │
└──────────────────────────────────────────────────────────────────────┘
```

### 1.4 核心功能

1. **离线唤醒** — 中/英文唤醒词，运行时动态切换，MQTT 远程下发新词
2. **实时语音对话** — OPUS 压缩，延迟约 200 ms 端到端
3. **多轮连续对话** — TTS 播放完自动进入下一轮，无需再次唤醒
4. **语音打断** — 播放期间唤醒词打断，立即清空缓冲切换输入态
5. **鲁棒重连** — 指数退避重连 + Token 定期刷新，网络中断自愈
6. **BLE 配网** — 首次上电通过 BLE 推送 WiFi 凭证 + 服务 Token

---

## 二、分述：各模块详解

### 2.1 BSP 模块（`main/bsp/`）

**核心职责**：硬件抽象层，屏蔽底层芯片驱动，向上层提供单例实例 + EventGroup 接口。

**关键文件**：

| 文件 | 职责 |
|------|------|
| `bsp_config.h` | 全量 GPIO 引脚宏定义、音频/LCD 硬件参数 |
| `bsp_board.h/c` | 全局单例 `bsp_board_t`、EventGroup 跨模块状态同步 |
| `bsp_codec.c` | ES8311 初始化（I2C 控制 + I2S 数据）、麦克风采集任务 |
| `bsp_lcd.c` | ST7789 LCD SPI 驱动初始化、背光控制 |
| `bsp_wifi.c` | WiFi STA 连接、BLE Provisioning 配网、按键监控 |

**硬件引脚**：

```c
I2C: SDA=GPIO8, SCL=GPIO15
I2S: MCLK=GPIO17, BCLK=GPIO9, WS=GPIO5, DIN=GPIO4, DOUT=GPIO6
LCD SPI: CS=GPIO10, MOSI=GPIO11, SCLK=GPIO12, DC=GPIO13, RST=GPIO14, BK=GPIO48
触摸: GPIO1/2/7 | 震动: GPIO16 | 舵机: GPIO21/38/47
音频参数: 16000 Hz / 16-bit / 单声道
```

**EventGroup 位**：`WIFI_BIT=BIT2, NVS_BIT=BIT3, CODEC_BIT=BIT4, LCD_BIT=BIT5, WIFI_FAIL_BIT=BIT6, PROV_DONE_BIT=BIT7`

**对外接口**：`bsp_board_get_instance()` / `audio_init()` / `bsp_board_wifi_main()` / `bsp_board_lcd_on/off()` / `bsp_board_check_status()`

---

### 2.2 Audio 模块（`main/audio/`）

**核心职责**：四层环形缓冲解耦上行（麦克风→编码→WS）和下行（WS→解码→扬声器）链路。

**关键文件**：`audio_processor.h/c`（中枢）/ `audio_encoder.h/c`（PCM→OPUS）/ `audio_decoder.h/c`（OPUS→PCM）

**环形缓冲**：

| 缓冲区 | 类型 | 大小 | 用途 |
|---------|------|------|------|
| enc_input | BYTEBUF | 20 KB | PCM 输入积累（无帧边界） |
| enc_output | NOSPLIT | 8 KB | OPUS 帧输出（整帧不拆） |
| dec_input | NOSPLIT | 5 KB | OPUS 帧输入（整帧不拆） |
| dec_output | BYTEBUF | 40 KB | PCM 播放缓冲（无帧边界） |

**OPUS 参数**：24 kbps / 60ms 帧 / 复杂度 3 / VOIP 模式 / FEC-DTX-VBR 全关

**任务**：编码/解码任务各 32KB 栈绑 CPU0，播放任务 4KB 绑 CPU0，优先级均为 5

**对外接口**：`audio_processor_create/start/stop/destroy()` / `write_pcm()` / `write()` / `read_timeout()` / `flush_output()`

---

### 2.3 Wake Word 模块（`main/wake_word/`）

**核心职责**：AFE 降噪 + VAD + MultiNet6 离线唤醒，支持中/英双语、热更新唤醒词。

**关键文件**：`custom_wake_word.h/c`

**架构**：

```
audio_feed_task → AFE（NS+VAD） → ┬─ enhanced_pcm_hook → session 编码器
                                   └─ MultiNet6 检测 → user_callback → session_on_wake_word()
```

**MultiNet 参数**：中文阈值 0.6 / 英文阈值 0.4 / 检测窗口 3000ms / 最少 2 音节

**语言自动检测**：UTF-8 汉字首字节 `0xE4~0xE9` → 中文模型；纯 ASCII → 英文大写模型

**NVS 键**：`sys_config` 命名空间，`wakeword`（拼音/英文）/ `ww_disp`（显示词）；默认"你好伙伴"

**对外接口**：`wake_word_init/start/stop()` / `custom_wake_word_feed()` / `wake_word_update()` / `set_enhanced_pcm_hook()` / `set_vad_callback()` / `get_vad_state()`

---

### 2.4 Session 模块（`main/session/`）

**核心职责**：系统业务逻辑中枢，驱动三态状态机，管理唤醒→推流→播放→关闭完整生命周期。

**关键文件**：`session.h/c`

**状态机**：

```
IDLE →（唤醒词）→ LISTENING →（TTS_START）→ PLAYING →（TTS_STOP）→ LISTENING（连续对话）→（COMPLETE/超时）→ IDLE
```

**关键定时参数**：

```c
SESSION_TIMEOUT_MS = 60000        // 会话超时 60s
EOS_SILENCE_MS = 800              // VAD 静音检测 800ms
VAD_GRACE_MS = 500                // 唤醒词尾音消退保护 500ms
TOKEN_REFRESH_MS = 110*60*1000    // Token 主动刷新 110min
重连: 指数退避 5s/10s/20s/40s/60s，最多 5 次
```

**事件队列防栈溢出**：定时器回调仅 `xQueueSend()` 推信号，`session_event_task` 独立执行网络操作

**ws_sender_task 两阶段**：阶段1等待 SERVER_READY_BIT（丢弃保持管道通畅），阶段2正式发送 OPUS 帧

**对外接口**：`session_init(ws_uri)` / `session_on_wake_word(display)` / `session_get_state()`

---

### 2.5 Protocol 模块（`main/protocol/`）

**核心职责**：三子层网络协议栈（WebSocket + MQTT + Auth）。

**关键文件**：`websocket_client.h/c` / `mqtt_protocol.h/c` / `auth.h/c`

**WebSocket 双轨**：Text Frame（JSON 控制）+ Binary Frame（OPUS 音频）

**上行 JSON**：`hello`（握手）/ `stop`（停止监听）/ `abort`（打断TTS）

**下行 JSON**：`started`（握手响应）/ `stt`（识别结果）/ `llm`（情感）/ `tts`（TTS控制）/ `iot`（设备控制）/ `complete` / `error`

**协议事件枚举**：`CONNECTED / DISCONNECTED / HELLO / STT / LLM / TTS_START / TTS_SENTENCE_START / TTS_STOP / AUDIO / IOT / ERROR / COMPLETE`

**认证链**：`deviceToken`（NVS长效）→ POST `/api/auth/device-login` → `accessToken`（~2h）→ WS Bearer 头，NVS 缓存备份

**MQTT 主题**：发布 `heartbeat`（50s）/ `reset`；订阅 `wake-word`（远程更新）；回调内禁止耗时操作，需创建异步任务

**WS 接口**：`protocol_create/connect/disconnect/is_connected()` / `send_hello/audio_data/stop_listening/abort_speaking()` / `register_callback()`

---

## 三、总结

### 3.1 整体运行流程

```
上电 → NVS → 唤醒词引擎 → 麦克风采集（AFE+MultiNet 开始工作）
→ WiFi/BLE配网 → MQTT心跳 → WebSocket预连接（Token换取+TLS）
→ [持续监听唤醒词]
→ 用户说"你好伙伴" → MultiNet6检测 → LISTENING
→ 推 OPUS → VAD 800ms静音 → stop → 云端 ASR/LLM/TTS
→ TTS_START → PLAYING → 解码播放 → TTS_STOP
→ 连续对话 或 关闭 → IDLE
```

### 3.2 设计亮点

| 亮点 | 实现方式 |
|------|---------|
| 零等待唤醒响应 | WiFi 就绪即预建 WebSocket，唤醒词触发直接发 Hello |
| 环形缓冲解耦 | 四层 RingBuffer（BYTEBUF/NOSPLIT 按帧边界选型） |
| 定时器安全 | 回调仅推事件入队，网络操作剥离到独立任务 |
| 语音打断 | abort 消息 + flush 缓冲，毫秒级切换 |
| 鲁棒认证 | 短效 Token + NVS 备份 + 110min 主动刷新 |
| 内存隔离 | 编解码大缓冲全用 SPIRAM |
| 唤醒词热更新 | MQTT → 异步任务 → MultiNet 重加载，无需重启 |
| 多语言自动识别 | UTF-8 字节特征判断中/英，自动加载对应模型 |

### 3.3 技术难点

| 难点 | 解决方案 |
|------|---------|
| 唤醒词尾音误触发 | VAD_GRACE_MS=500ms 消退保护期 |
| OPUS 帧完整性 | NOSPLIT 环形缓冲保证整帧读写 |
| 多核任务竞争 | 编解码绑 CPU0，采集/AFE 绑 CPU1 |
| HTTP 分块传输 | auth.c 动态 realloc 拼接，完整后统一解析 |
| WebSocket 幽灵连接 | disconnect() 无论连接状态都调 stop() |
| 编码器背压 | sender_task 阶段1消费但不发，防 enc_output 溢出 |

### 3.4 关键文件速查

```
main/application.c              → 启动编排
main/bsp/bsp_config.h           → GPIO 和音频参数
main/bsp/bsp_board.h/c          → 全局单例 + EventGroup
main/bsp/bsp_codec.c            → ES8311 + 麦克风采集任务
main/bsp/bsp_lcd.c              → ST7789 LCD 驱动
main/bsp/bsp_wifi.c             → WiFi + BLE 配网
main/audio/audio_processor.h/c  → 编解码管道中枢
main/audio/audio_encoder.h/c    → PCM → OPUS
main/audio/audio_decoder.h/c    → OPUS → PCM
main/wake_word/custom_wake_word.h/c → AFE + MultiNet6
main/session/session.h/c        → 会话状态机（业务中枢）
main/protocol/websocket_client.h/c → WebSocket 双轨通信
main/protocol/mqtt_protocol.h/c → MQTT 设备管理
main/protocol/auth.h/c          → HTTP 认证
```
