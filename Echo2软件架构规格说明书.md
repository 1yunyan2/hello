<!-- 文档密级：内部 -->

| 项目 | 内容 |
| --- | --- |
| 文档名称 | Echo2 智能语音交互设备 软件架构规格说明书 |
| 文档编号 | ECHO2-SA-001 |
| 文档版本 | V1.0 |
| 文档密级 | 内部 |
| 文档状态 | 草稿 |
| 项目编号 | ECHOPALS-ECHO2 |

| 拟制 | 审核 | 批准 |
| --- | --- | --- |
|      |      |      |
| 日期： | 日期： | 日期： |

> 本文档的版权属于本公司所有，任何使用、复制、公开此文档的行为都必须经过本公司的书面允许。

---

# 变更记录

| 版本 | 修订 | 修改内容 | 修改人 | 日期 |
| --- | --- | --- | --- | --- |
| V1.0 | 01 | 拟制 |  | 2026-08-20 |
|  |  |  |  |  |
|  |  |  |  |  |

---

# 目　　录

- [1 引言](#1-引言)
  - [1.1 文档目标](#11-文档目标)
  - [1.2 输入文档](#12-输入文档)
  - [1.3 术语与缩略语](#13-术语与缩略语)
- [2 假设和约束](#2-假设和约束)
  - [2.1 假定](#21-假定)
  - [2.2 约束](#22-约束)
- [3 架构分析](#3-架构分析)
- [4 软件逻辑架构](#4-软件逻辑架构)
  - [4.1 硬件驱动模块（BSP）](#41-硬件驱动模块bsp)
  - [4.2 音频通路模块](#42-音频通路模块)
  - [4.3 语音唤醒模块](#43-语音唤醒模块)
  - [4.4 会话控制模块](#44-会话控制模块)
  - [4.5 通信协议模块](#45-通信协议模块)
  - [4.6 显示与 UI 模块](#46-显示与-ui-模块)
  - [4.7 触摸交互与情绪表达模块](#47-触摸交互与情绪表达模块)
  - [4.8 舵机与震动执行模块](#48-舵机与震动执行模块)
  - [4.9 提醒服务模块](#49-提醒服务模块)
  - [4.10 游戏模块](#410-游戏模块)
  - [4.11 待机与低功耗模块](#411-待机与低功耗模块)
  - [4.12 OTA 与配网模块](#412-ota-与配网模块)
- [5 运行架构视图](#5-运行架构视图)
  - [5.1 启动时序](#51-启动时序)
  - [5.2 并发任务视图](#52-并发任务视图)
  - [5.3 状态机视图](#53-状态机视图)
  - [5.4 数据流视图](#54-数据流视图)
- [6 内存与存储架构](#6-内存与存储架构)
- [7 软件故障处理](#7-软件故障处理)
- [8 待办与遗留问题](#8-待办与遗留问题)

---

# 1 引言

## 1.1 文档目标

本说明书针对 Echo2 智能语音交互设备的嵌入式软件编写，目的是对该设备的应用固件进行结构化设计描述：根据软件需求，分析数据流和处理流程，划分软件层次结构和功能模块，进行软件需求分配，为后期的详细设计、测试与维护做准备。

本设计说明书的预期读者为本项目组开发人员、测试人员，以及后期系统集成和软件维护人员。

## 1.2 输入文档

- 《Echo2 软件需求规格说明书》，以最新版为准
- 《Echo2 硬件原理图与引脚分配表》，以最新版为准
- 《Echo2 云端交互协议说明（WebSocket / MQTT）》，以最新版为准
- 仓库内《项目任务清单》 `docs/项目任务清单.md`

## 1.3 术语与缩略语

| 缩略语 | 全称 | 说明 |
| --- | --- | --- |
| BSP | Board Support Package | 板级支持包，硬件驱动层 |
| AFE | Audio Front End | 乐鑫音频前端算法（AEC/NS/VAD） |
| VAD | Voice Activity Detection | 语音活动检测 |
| ASR | Automatic Speech Recognition | 自动语音识别 |
| TTS | Text To Speech | 语音合成 |
| WW | Wake Word | 唤醒词 |
| PSRAM | Pseudo Static RAM | 片外扩展内存（本项目 Octal 模式 80MHz） |
| OTA | Over The Air | 空中固件升级 |
| LVGL | Light and Versatile Graphics Library | 嵌入式图形库 |

---

# 2 假设和约束

## 2.1 假定

- 本文所依据的硬件原理图与引脚分配不发生重大变更；
- 云端交互协议（WebSocket 会话通道、MQTT 控制通道）接口保持向后兼容；
- 设备运行在有 WiFi 覆盖的家庭/室内环境，允许短时断网（由离线模式兜底）。

## 2.2 约束

### 2.2.1 平台约束

| 项 | 取值 |
| --- | --- |
| 主控芯片 | ESP32-S3-WROOM-1 N16R8（双核 Xtensa LX7，240MHz） |
| 片上 SRAM | 512KB 内部 SRAM |
| 片外 PSRAM | 8MB，Octal 模式，80MHz |
| 主 Flash | 16MB |
| 外挂 Flash | 16MB / 32MB（各批次板卡不统一，固件运行期探测实际容量） |
| 开发框架 | ESP-IDF v5.3.4 |
| 图形库 | LVGL 9.5 |
| 语音算法 | esp-sr（AFE + WakeNet + MultiNet） |

### 2.2.2 规范约束

- 开发语言限定为纯 C 语言，不引入 C++ 业务代码；
- 头文件必须使用 `#pragma once`，禁止使用 `#ifndef` 防护宏；
- 函数命名采用下划线分隔（snake_case），禁止驼峰命名；
- 所有 `.c` / `.h` 业务文件必须置于 `main/` 或 `components/` 目录下；
- 禁止修改 ESP-IDF 系统源码与第三方组件库文件，适配需求一律通过 BSP 层封装解决；
- 软件模块划分需完全覆盖软件需求。

### 2.2.3 方法约束

- 在文档中主要采用自然语言进行描述，并辅以结构/功能块图说明；
- 软件使用模块化方法进行结构设计，跨模块调用只允许经由被调方头文件中声明的公开接口；
- 设计以稳定性、可维护性优先于极致性能；
- 涉及硬件资源的并发访问一律通过队列或互斥量串行化，禁止多任务直驱同一外设。

---

# 3 架构分析

Echo2 的核心用例为「唤醒 → 拾音 → 云端识别与生成 → 播放应答」的语音对话闭环，并在此闭环之外挂载了本地交互（触摸情绪、舵机动作、小游戏）、时间服务（闹钟/倒计时/日历/天气）与设备管理（配网、OTA、远程控制、低功耗）三类支撑用例。

按用例分解可导出以下架构模块：

| 用例 | 导出模块 |
| --- | --- |
| 离线语音唤醒 | 语音唤醒模块、音频通路模块、BSP 音频驱动 |
| 语音对话 | 会话控制模块、通信协议模块、音频通路模块 |
| 情绪与动作表达 | 触摸交互与情绪表达模块、舵机与震动执行模块、显示与 UI 模块 |
| 时间与提醒服务 | 提醒服务模块、显示与 UI 模块 |
| 本地娱乐 | 游戏模块、显示与 UI 模块、BSP 触摸驱动 |
| 设备联网与管理 | OTA 与配网模块、通信协议模块 |
| 续航管理 | 待机与低功耗模块、BSP 电池/显示/舵机驱动 |

---

# 4 软件逻辑架构

整体固件架构以分层形式实现，自上而下分为应用层、服务与组件层、硬件驱动层三层。上层不得跨层直接操作寄存器，下层不得反向依赖上层。

```
+----------------------------------------------------------------------+
|  应用层                                                              |
|  会话控制 session | 触摸交互 interaction | 提醒服务 reminder          |
|  游戏 games       | 待机低功耗 standby   | 远程控制 remote_control     |
+----------------------------------------------------------------------+
|  服务与组件层                                                        |
|  通信协议 websocket_client / mqtt_protocol / auth                    |
|  音频通路 audio_processor / audio_encoder(OPUS) / audio_decoder      |
|  离线音频 offline_audio | 语音唤醒 custom_wake_word(AFE+MultiNet)     |
|  UI 门面 ui_port + LVGL 9.5 + 字体加载 font_loader                    |
|  舵机管理 servo_manager | OTA bsp_ota | 配网 blufi | 日志 udp_logger   |
+----------------------------------------------------------------------+
|  硬件驱动层（BSP）                                                    |
|  bsp_board / bsp_codec / bsp_lcd / bsp_touch / bsp_servo              |
|  bsp_wifi / bsp_battery / bsp_flash / bsp_pca9536                    |
+----------------------------------------------------------------------+
|  ESP-IDF v5.3.4（FreeRTOS / lwIP / esp-sr / SPIFFS / NVS）            |
+----------------------------------------------------------------------+
```

- **应用层**：实现对话流程编排、交互逻辑、提醒调度、游戏玩法与功耗策略，不直接接触寄存器。
- **服务与组件层**：提供协议编解码、音频编解码、图形绘制、执行器排队等可复用能力，向上屏蔽实现细节。
- **硬件驱动层**：完成底层硬件初始化与寄存器操作，解除上层应用与硬件的耦合。

下文各模块统一按「模块需求 / 模块功能 / 模块接口 / 模块属性 / 故障处理」五节描述。

---

## 4.1 硬件驱动模块（BSP）

### 4.1.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 硬件驱动模块 | M-BSP | （待需求规格填入） |

### 4.1.2 模块功能

驱动模块与板卡硬件紧密相关，是软件与外部设备之间交互的底层中间件，其中包含多个子模块，主要实现硬件外设的初始化、数据收发与状态读取。各子模块功能描述见下表。

| 序号 | 子模块 | 源文件 | 功能描述 |
| --- | --- | --- | --- |
| 1 | 板级总控 | `bsp/bsp_board.c` | 单例板级上下文创建、EventGroup 建立、NVS 初始化、各子系统初始化编排入口 |
| 2 | 音频编解码芯片 | `bsp/bsp_codec.c` | ES8311 codec 与 I2S 初始化、麦克风采集任务、扬声器输出、音量控制与低功耗进出 |
| 3 | 显示屏 | `bsp/bsp_lcd.c` | ST7789 SPI 屏初始化（80MHz）、背光 PWM 与渐变、显示开关 |
| 4 | 触摸 | `bsp/bsp_touch.c` | TTP223 触摸位扫描、去抖与长按判定、事件上报；震动马达 PWM 控制 |
| 5 | 舵机 | `bsp/bsp_servo.c` | LEDC PWM 舵机驱动、限速平滑运动、并行运动、急停与空闲释放 |
| 6 | 无线网络 | `bsp/bsp_wifi.c` | WiFi STA 连接与重连、NVS 凭据管理、按键长按重置、设备名生成 |
| 7 | 电池 | `bsp/bsp_battery.c` | ADC 采样、OCV 电量估算、充电状态判定、电量监控与日志任务 |
| 8 | 外挂 Flash | `bsp/bsp_flash.c` | 外挂 SPI Flash 容量探测（JEDEC）、FATFS 挂载、产线烧录与整片擦除 |
| 9 | IO 扩展 | `bsp/bsp_pca9536.c` | PCA9536 I2C IO 扩展芯片读写 |
| 10 | 网络日志 | `bsp/udp_logger.c` | 日志镜像到 UDP，便于无串口现场取证 |

### 4.1.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `bsp_board_get_instance` | 板级单例获取，必须最先调用 |
| 2 | `bsp_board_nvs_init` | NVS 初始化 |
| 3 | `bsp_board_codec_init` / `audio_init` | 音频硬件初始化 |
| 4 | `bsp_board_codec_set_volume` | 扬声器音量设置（含 NVS 持久化） |
| 5 | `bsp_board_codec_enter_lowpower` / `_exit_lowpower` | codec 低功耗进出 |
| 6 | `bsp_board_lcd_init` / `_on` / `_off` | 屏幕初始化与开关 |
| 7 | `bsp_board_lcd_disp_on` / `_disp_off` | 显示通路开关（须持 LVGL 锁调用） |
| 8 | `bsp_board_lcd_set_brightness` / `_fade_brightness` / `_fade_step` | 背光亮度与渐变 |
| 9 | `bsp_board_servo_init` | 舵机 LEDC 初始化 |
| 10 | `bsp_servo_move_smooth` / `_move_all_parallel` | 舵机平滑运动 / 多轴并行运动 |
| 11 | `bsp_servo_request_abort` / `_clear_abort` / `_abort_requested` | 舵机急停控制 |
| 12 | `bsp_servo_idle` / `_resume` | 舵机停 PWM 释放与恢复 |
| 13 | `touch_scan_task` / `bsp_touch_get_event` | 触摸扫描任务与事件读取 |
| 14 | `bsp_motor_pulse` / `bsp_motor_set` | 震动马达脉冲 / 电平设置 |
| 15 | `bsp_board_wifi_main` / `clear_wifi_and_restart` | 联网主流程 / 清除凭据重启 |
| 16 | `bsp_board_check_status` | 板级状态自检 |

### 4.1.4 模块属性

- 模块类型：新开发（部分驱动移植自参考工程后重写）；
- 任务栈归属：`audio_feed` 位于 PSRAM，`touch_scan` 强制位于内部 SRAM（涉及 Flash 操作，见 7.3）；
- 复用第三方组件：`esp_lcd`、`esp_codec_dev`、`driver/ledc`、`driver/i2s`。

### 4.1.5 故障处理

- I2C 从机 NACK：记录错误码并重试，不调用 `ESP_ERROR_CHECK` 直接 abort（abort 会掩盖真实根因）；
- 舵机 GPIO14 上电弱上拉误触发：`application_init` 最开头强制拉低占位；
- 外挂 Flash 容量与打包镜像不符：运行期读取实际容量并按实际值擦写，防止越界。

---

## 4.2 音频通路模块

### 4.2.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 音频通路模块 | M-AUD | （待需求规格填入） |

### 4.2.2 模块功能

负责上行与下行两条音频通路的缓冲、编解码与播放调度。

| 序号 | 子模块 | 源文件 | 功能描述 |
| --- | --- | --- | --- |
| 1 | 音频处理器 | `audio/audio_processor.c` | 上下行环形缓冲管理、播放任务、输入/输出缓冲冲刷、播放态查询 |
| 2 | 编码器 | `audio/audio_encoder.c` | PCM → OPUS 编码（24kbps CBR），供上行发送 |
| 3 | 解码器 | `audio/audio_decoder.c` | 云端 OPUS → PCM 解码，供扬声器播放 |
| 4 | 离线音频 | `audio/offline_audio.c` | 播放外挂 Flash `/S/voice/` 下的本地音频，按扩展名分发 MP3 / P3(OPUS) 解码 |

上行链路：麦克风 I2S → AFE 降噪/回声消除 → OPUS 编码 → WebSocket 上传。
下行链路：WebSocket 下发 OPUS → 解码 → 环形缓冲 → I2S 播放。

### 4.2.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `audio_processor_create` / `_destroy` | 实例生命周期 |
| 2 | `audio_processor_start` / `_stop` | 启动/停止编解码与播放任务 |
| 3 | `audio_processor_read` / `_read_timeout` | 读取上行编码数据 |
| 4 | `audio_processor_write` / `_write_pcm` | 写入下行数据 |
| 5 | `audio_processor_flush_input` / `_flush_output` | 缓冲冲刷（打断时使用） |
| 6 | `audio_processor_is_playing` | 播放态查询（用于会话状态判定） |
| 7 | `audio_processor_read_ref_pcm` | 读取参考信号供 AEC |
| 8 | `offline_audio_play` / `_stop` / `_is_playing` | 本地离线音频播放 |

### 4.2.4 模块属性

- 模块类型：新开发；
- 编解码任务栈各 32KB，均分配于 PSRAM；离线播放任务栈 32KB PSRAM；
- 编码参数决策见《DEC-001 OPUS 24kbps CBR》《DEC-002 四层环形缓冲》。

### 4.2.5 故障处理

- 环形缓冲写满：丢弃最旧数据并打印告警，不阻塞采集任务；
- 解码失败：跳过该帧继续解码，连续失败超阈值则终止本轮播放并回到 IDLE；
- 离线播放仲裁：对话进行中拒绝播放，云端播放中让路，离线音频之间后者顶掉前者。

---

## 4.3 语音唤醒模块

### 4.3.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 语音唤醒模块 | M-WW | （待需求规格填入） |

### 4.3.2 模块功能

基于 esp-sr 实现离线语音唤醒与语音活动检测。加载 WakeNet 唤醒模型与 MultiNet 命令词模型，持续消费 `audio_feed` 投喂的 PCM，检出唤醒词后回调会话模块；同时输出 VAD 状态供会话端点判定使用。支持云端下发更换唤醒词（重建 FST）。

### 4.3.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `wake_word_init` | 引擎初始化并注册检出回调 |
| 2 | `wake_word_start` / `wake_word_stop` | 启动 / 停止检测 |
| 3 | `custom_wake_word_feed` | 投喂 PCM 数据 |
| 4 | `custom_wake_word_get_chunksize` / `_get_feed_chunksize` | 获取算法要求的帧长 |
| 5 | `wake_word_update` / `_is_same` / `_load_from_nvs` | 唤醒词更新与持久化 |
| 6 | `wake_word_set_det_threshold` | 检出阈值调节 |
| 7 | `bsp_wake_word_set_vad_callback` / `wake_word_get_vad_state` | VAD 状态订阅与查询 |
| 8 | `custom_wake_word_set_aec_ref` | 设置 AEC 参考信号 |
| 9 | `bsp_wake_word_stop_for_ota` | OTA / 解绑前停止引擎，释放 CPU1 |

### 4.3.4 模块属性

- 模块类型：基于 esp-sr 组件封装开发；
- 模型存放于 `model` 分区（SPIFFS，8MB）；
- 初始化任务 `ww_init` 栈 8192B，**必须位于内部 SRAM**（模型加载读 Flash 期间 cache 关闭）；
- 内部子任务 `afe_fetch` / `mn_detect` 由组件自行创建并钉在 CPU1，优先级 5。

### 4.3.5 故障处理

- 模型加载失败：打印错误并降级为「无唤醒词」模式，设备其余功能仍可用；
- 模型加载期 CPU0 抢占导致看门狗告警：初始化任务临时提优先级并改钉 CPU1（见 7.3）；
- esp-sr 库内 CTC 解码崩溃（未结案缺陷）：定位为库内缺陷，已向上游提交 issue，止血策略为「崩溃后快速恢复且用户无感知」，游戏与会话进度持久化。

---

## 4.4 会话控制模块

### 4.4.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 会话控制模块 | M-SESS | （待需求规格填入） |

### 4.4.2 模块功能

对话流程的中枢状态机，负责唤醒响应、拾音起止判定、上行发送编排、下行播放调度、打断处理与多轮超时控制。维护三态状态机：

| 状态 | 含义 |
| --- | --- |
| `SESSION_IDLE` | 待机：唤醒词监听中，WebSocket 保持预连接，无音频处理 |
| `SESSION_LISTENING` | 监听：麦克风 PCM → OPUS → WebSocket → 云端 |
| `SESSION_PLAYING` | 播放：云端 TTS → 解码 → 扬声器，唤醒词引擎同时监听打断 |

唤醒返回值区分三种语义：`WAKE_IGNORED`（忽略）、`WAKE_NEW_SESSION`（新会话，播提示音）、`WAKE_INTERRUPT`（打断播放，不播提示音）。

### 4.4.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `session_init` | 初始化并建立 WebSocket 预连接 |
| 2 | `session_on_wake_word` | 唤醒事件入口，返回 `wake_result_t` |
| 3 | `session_get_state` | 线程安全的状态只读查询 |
| 4 | `session_stop_for_ota` | OTA 前停止会话 |
| 5 | `session_debug_kill_ws` | 调试用：强制断开 WebSocket 验证重连 |

### 4.4.4 模块属性

- 模块类型：新开发；
- 内部任务：`session_evt_tsk`（8KB PSRAM，CPU1，prio 5）、`ws_sender`（4KB PSRAM，CPU0，prio 4）；
- 事件驱动，跨任务通信全部经 FreeRTOS 队列与 EventGroup，禁止直接跨任务调用。

### 4.4.5 故障处理

- 服务端 `SERVER_READY` 丢失：设置多轮超时（含 40s 硬上限）强制回到 IDLE，避免永久挂起；
- WebSocket 断开：由协议层触发指数退避重连，会话回到 IDLE 并给出 UI 提示；
- 长 TTS 尾音被识别为下一轮输入：播放结束后冲刷输入缓冲并延迟启用 VAD。

---

## 4.5 通信协议模块

### 4.5.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 通信协议模块 | M-PROTO | （待需求规格填入） |

### 4.5.2 模块功能

| 序号 | 子模块 | 源文件 | 功能描述 |
| --- | --- | --- | --- |
| 1 | 会话通道 | `protocol/websocket_client.c` | WebSocket 建链、Hello/Start 握手、二进制音频帧收发、文本分片重组、断线重连 |
| 2 | 控制通道 | `protocol/mqtt_protocol.c` | MQTT 订阅下行指令（音量、唤醒词更新、解绑、远程控制、OTA 触发）、心跳上报、重连退避 |
| 3 | 鉴权 | `protocol/auth.c` | 设备鉴权、accessToken 获取与主动/被动双重刷新（有效期 2h）、服务可达性探测 |

### 4.5.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `protocol_create` / `_destroy` | 会话通道实例生命周期 |
| 2 | `protocol_connect` / `_disconnect` / `_disconnect_timeout` | 连接控制 |
| 3 | `protocol_is_connected` | 连接状态查询 |
| 4 | `protocol_send_start` | 发送开始会话指令 |
| 5 | `protocol_send_audio_data` | 发送上行音频帧 |
| 6 | `protocol_send_stop_listening` / `_send_abort_speaking` | 结束拾音 / 打断播放 |
| 7 | `protocol_register_callback` | 注册下行事件回调 |
| 8 | `protocol_mqtt_start` | 启动 MQTT 控制通道 |
| 9 | `send_reset_notification` | 上报重置/解绑事件 |
| 10 | `auth_create` / `_perform` / `_is_server_reachable` / `_destroy` | 鉴权流程 |

### 4.5.4 模块属性

- 模块类型：新开发；
- 内部任务：`heartbeat_task`（4KB，prio 4）、`mqtt_reconn`（PSRAM 栈，须 `vTaskDeleteWithCaps` 自删）、`async_ww_update`、`async_unbind`（8KB 内部 SRAM，一次性任务）；
- 依赖组件：`esp_websocket_client`、`mqtt`、`esp_http_client`、`mbedTLS`。

### 4.5.5 故障处理

- 断线重连：指数退避重连，重连任务必须以 `xTaskCreate*WithCaps` 创建并以 `vTaskDeleteWithCaps` 自删，否则每轮泄漏栈与 TCB；
- MQTT 客户端不得在其自身事件回调内调用 `esp_mqtt_client_stop`（必然失败使退避失效），须投递到独立任务执行；
- 连续重连失败超阈值：置「运行态离线标志」并彻底停止重连，进入离线模式（时间走 NVS 兜底，闹钟与日历照常触发）。

---

## 4.6 显示与 UI 模块

### 4.6.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 显示与 UI 模块 | M-UI | （待需求规格填入） |

### 4.6.2 模块功能

以 `ui_port.c` 作为唯一 UI 门面，封装 LVGL 9.5 的全部调用，向业务层提供线程安全的页面与元素操作接口。业务任务不得直接调用 `lv_*` API，一律经门面转发，由门面统一持 `lvgl_port_lock`。

主要能力：主页 GIF 表情轮播、状态 GIF（空闲/监听/说话三态）、功能盘五模块页（时间、天气、闹钟、日历、游戏）、配网二维码页、OTA 进度页、解绑提示页、待机时钟页、倒计时到期与闹钟响铃页。

字体资源由 `font_loader.c` 统一管理，含中文 12/16/24/32 号与时间 144 号、日历 74 号、闹钟 59 号等专用大字号。

### 4.6.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `ui_init` | UI 初始化（须在 LVGL port 初始化之后） |
| 2 | `ui_show_emotion` / `ui_update_emotion` | 情绪画面切换 |
| 3 | `ui_request_emotion_gif` / `ui_request_state_gif` | 跨任务请求切图（状态切图须置 `is_state`） |
| 4 | `ui_play_animation` | 播放指定动画 |
| 5 | `ui_pause_main_gif` / `ui_resume_main_gif` / `ui_resume_main_gif_loop` | 主页 GIF 暂停与恢复 |
| 6 | `ui_function_menu_enter` / `_exit` | 功能盘进出 |
| 7 | `ui_home_enter` / `ui_force_back_to_main` / `ui_func_layer_exit_to_main` | 页面返回 |
| 8 | `ui_update_wifi` / `_battery` / `_time` | 状态栏刷新 |
| 9 | `ui_show_provision_image` | 配网引导页 |
| 10 | `ui_show_ota_progress` / `ui_show_unbinding` | OTA / 解绑提示 |
| 11 | `ui_show_alarm_ringing` / `ui_show_countdown_expired` | 提醒触发画面 |
| 12 | `ui_standby_clock_show` / `_hide` | 待机时钟 |
| 13 | `ui_dispatch_touch_event` | 触摸事件分发入口 |
| 14 | `ui_get_current_view` | 当前页面查询 |
| 15 | `ui_notify_first_online` / `ui_notify_boot_ready` | 联网就绪 / 开机就绪通知 |

### 4.6.4 模块属性

- 模块类型：新开发（基于 LVGL 9.5 组件）；
- LVGL 绘制任务 `taskLVGL` 钉在 CPU0，优先级 5；
- 显示缓冲为 `W*H/6`，位于 PSRAM；SPI 时钟 80MHz；
- 图片与 GIF 资源以 C 数组形式内联的部分位于 Flash 只读段，不占内部 SRAM；大尺寸背景图走 PSRAM。

### 4.6.5 故障处理

- 在 LVGL 未初始化时调用门面接口会撞 `lvgl_port_lock` 断言：门面内部以 `s_lvgl_ready` 标志判定，未就绪时直接返回而非断言；
- `ready_cb` 内不得直接 `lv_gif_set_src`（解码器重入卡死）：改为置 pending 标志 + 定时器延后切换；
- 显示刷新逐带可见、换图瞬间双图同框：经完整排查判定为无 TE 引脚的硬件带宽天花板，非软件缺陷，不再投入软件规避（见第 8 节）。

---

## 4.7 触摸交互与情绪表达模块

### 4.7.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 触摸交互与情绪表达模块 | M-IA | （待需求规格填入） |

### 4.7.2 模块功能

将触摸事件路由为「情绪表达」组合动作。维护 6 个触摸位置 × 6 种情绪的情绪矩阵，命中后随机选取一条情绪，串行执行「切 GIF + 舵机动作 + 震动序列 + 音频提示」四通道组合。提供独立的自定义动作通道 `ia_custom_action_t`，供状态 GIF、空闲动作与远程控制复用而不污染情绪矩阵。

`remote_control.c` 提供云端下发的舵机与 GIF 直控通道，与本地情绪表达共用同一 worker 串行执行，避免执行器打架。

### 4.7.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `interaction_manager_init` | 初始化 worker 任务与队列 |
| 2 | `ui_interaction_play` | 按触摸位置播放情绪组合动作 |
| 3 | `ui_interaction_play_custom` | 播放自定义组合动作 |
| 4 | `interaction_is_playing` / `_is_idle_action` | 执行态查询 |
| 5 | `interaction_set_lowpower` | 低功耗模式下裁剪动作（停手臂、关震动） |
| 6 | `interaction_flush_queue` | 清空待执行队列 |
| 7 | `interaction_stop_for_ota` | OTA 前停止 |
| 8 | `remote_control_init` / `_submit_servo` / `_submit_gif` / `_is_active` / `_cancel` | 远程直控通道 |

### 4.7.4 模块属性

- 模块类型：新开发；
- worker 任务 `ia_worker`：栈 8192B（PSRAM），优先级 5，不绑核；
- 触摸硬件为 TTP223，灵敏度由硬件决定，软件仅能调整响应快慢。

### 4.7.5 故障处理

- 组合动作执行中收到新事件：按队列排队，不并发执行；低功耗或退出场景调用 `interaction_flush_queue` 丢弃积压；
- 动作通道内禁止阻塞式延时（会卡住 LVGL 线程导致动画被吃掉），马达等执行器一律用「非阻塞置位 + 引擎按时关闭」。

---

## 4.8 舵机与震动执行模块

### 4.8.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 舵机与震动执行模块 | M-SERVO | （待需求规格填入） |

### 4.8.2 模块功能

以队列化管理器串行化所有舵机运动请求，是「业务层 → servo_manager 队列 → bsp_servo 直驱」三层调用链的中间层。支持单轴相对/绝对运动、多轴并行运动、运动完成通知、队列冲刷与急停、校准参数 NVS 持久化。

**约束**：业务层禁止绕过 `servo_manager` 直接调用 `bsp_servo_*`，两套接口混用会导致运动指令互相打架。

### 4.8.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `servo_manager_init` / `_deinit` | 管理器生命周期 |
| 2 | `servo_manager_submit_request` | 提交单轴运动请求 |
| 3 | `servo_manager_submit_parallel` | 提交多轴并行运动 |
| 4 | `servo_manager_submit_abs_parallel_notify` | 绝对角度并行运动 + 完成通知 |
| 5 | `servo_manager_submit_by_index` / `_submit_angle` | 按索引 / 按角度提交 |
| 6 | `servo_manager_flush` / `_flush_ex` | 队列冲刷（非阻塞，须先于 abort 清除） |
| 7 | `servo_manager_is_idle` | 空闲查询 |
| 8 | `servo_manager_save_calibration` / `_load_calibration` | 校准参数持久化 |

### 4.8.4 模块属性

- 模块类型：新开发；
- worker 任务 `servo_mgr`：栈 4096B（PSRAM），优先级 6，不绑核；
- 舵机为记忆型舵机：断 PWM 后仍会走完目标角度，软件限速对上电归中无效，如需真正的慢速须更换总线舵机。

### 4.8.5 故障处理

- 队列满：丢弃最新请求并返回错误码，绝不阻塞调用方（零超时入队）；
- 低功耗停 PWM 与运动中的竞态：必须先 `flush`（非阻塞）后清除 abort 标志，顺序颠倒会导致停不下来；
- 舵机与 I2C 共用 timer 或引脚复用冲突：由引脚分配表统一约束（见《DEC-005 GPIO 引脚分配》）。

---

## 4.9 提醒服务模块

### 4.9.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 提醒服务模块 | M-REM | （待需求规格填入） |

### 4.9.2 模块功能

| 序号 | 子功能 | 功能描述 |
| --- | --- | --- |
| 1 | 时间同步 | SNTP 校时；断网时以 NVS 保存的时间兜底，保证闹钟与日历照常触发 |
| 2 | 闹钟 | 增删改查、启用/停用、到点响铃与关闭 |
| 3 | 倒计时 | 启动、取消、剩余时间查询、到期提示 |
| 4 | 日历 | 日程增删、当日日程查询 |
| 5 | 天气 | 和风天气 now 接口拉取（温度、体感、风、湿度、降水量等），自动定位城市，结果缓存 NVS |

### 4.9.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `reminder_init` / `_deinit` | 模块生命周期 |
| 2 | `reminder_on_offline_mode` | 切入离线模式通知 |
| 3 | `reminder_get_state` / `_is_time_synced` / `_get_current_time` | 状态与时间查询 |
| 4 | `reminder_alarm_add` / `_delete` / `_update` / `_set_enabled` / `_get_all` / `_dismiss` | 闹钟管理 |
| 5 | `reminder_timer_start` / `_cancel` / `_get_remain` | 倒计时管理 |
| 6 | `reminder_calendar_add` / `_delete` / `_get_today` | 日历管理 |
| 7 | `reminder_weather_config` / `_fetch_now` / `_auto_locate_city` / `_get_weather_data` | 天气服务 |

### 4.9.4 模块属性

- 模块类型：新开发；
- NVS 写入统一投递到 `nvs_save` 任务执行，该任务栈以 `MALLOC_CAP_INTERNAL` 分配；
- 天气拉取走 HTTPS，占用 mbedTLS 栈，须留足余量。

### 4.9.5 故障处理

- **PSRAM 栈任务禁止直接写 NVS / 擦写 Flash / 写 OTA**：Flash 操作期间 cache 关闭，PSRAM 映射同时失效，IDF 会在 `esp_task_stack_is_sane_cache_disabled()` 主动断言。所有 NVS 写入一律投递到内部 SRAM 栈的 `nvs_save` 任务；
- 天气拉取失败：沿用上次缓存值并在 UI 上标注数据时间；
- 时间未同步时触发闹钟：以 NVS 兜底时间判定，并在同步成功后重新校准下一次触发点。

---

## 4.10 游戏模块

### 4.10.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 游戏模块 | M-GAME | （待需求规格填入） |

### 4.10.2 模块功能

提供三款本地单机小游戏，统一由 `games.c` 调度：

| 序号 | 游戏 | 源文件 | 触发方式 |
| --- | --- | --- | --- |
| 1 | 打地鼠 | `games/game_whack.c` | 按下即触发 |
| 2 | 赛车 | `games/game_race.c` | 按下即触发 |
| 3 | 跳一跳 | `games/game_jump.c` | 按下蓄力、松手起跳 |

统一行为：结算画面仅保留背景图，结算后 1 秒自动重开（one-shot `lv_timer`）。

### 4.10.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `games_start` | 启动指定游戏 |
| 2 | `games_stop` | 退出游戏（须取消结算定时器） |
| 3 | `games_handle_touch` | 触摸事件分发 |
| 4 | `games_get_current` | 当前游戏查询（用于按游戏分流触发语义） |
| 5 | `games_get_name` | 游戏名查询 |

### 4.10.4 模块属性

- 模块类型：新开发；
- 全部图片资源以 C 数组形式内联，位于 Flash 只读段，不占内部 SRAM；大尺寸背景图走 PSRAM 解码；
- 游戏逻辑运行在 LVGL 定时器回调中，不新建任务。

### 4.10.5 故障处理

- 游戏停止或重开时必须取消结算定时器，否则定时器回调访问已释放对象形成野指针；
- 游戏进行中发生底层异常复位：进度持久化后恢复，保证用户无感知（止血策略，待落地）。

---

## 4.11 待机与低功耗模块

### 4.11.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| 待机与低功耗模块 | M-STBY | （待需求规格填入） |

### 4.11.2 模块功能

以「无活动超时」驱动的功耗分级管理。活动定义为对话、触摸或唤醒；充电状态下刷新活动时间戳，跳过全部降级。当前保留两档：

| 档位 | 触发条件 | 行为 |
| --- | --- | --- |
| 深度待机 | 超时（默认 60s）无活动 | 关屏、停舵机 PWM、关震动、codec 进低功耗，唤醒词监听保持 |
| 关机 | 长按电源键 | 拉低电源使能引脚断电 |

不使用 ESP-IDF 的真 sleep，仅关闭外设——因为唤醒词监听必须持续运行。

### 4.11.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `standby_init` | 初始化 |
| 2 | `standby_notify_activity` | 上报活动，刷新超时计时 |
| 3 | `standby_wake` | 强制唤醒 |
| 4 | `standby_is_deep_active` | 深度待机态查询 |

### 4.11.4 模块属性

- 模块类型：新开发；
- 运行于 LVGL 定时器与电池监控任务中，不新建常驻任务。

### 4.11.5 故障处理

- 唤醒黑屏卡死：`disp_on_off` 必须持 `lvgl_port_lock` 调用；恢复流程中任一步失败必须整段回滚，不得停留在半恢复状态，并由自动重试机制补偿；
- 充电中误入待机：电量估算在满电区（OCV ≥ 4000mV）直接判定为充电中，避免 `is_charging()` 恒假。

---

## 4.12 OTA 与配网模块

### 4.12.1 模块需求

| 功能模块名称 | 功能模块编号 | 对应软件需求编号 |
| --- | --- | --- |
| OTA 与配网模块 | M-OTA | （待需求规格填入） |

### 4.12.2 模块功能

| 序号 | 子功能 | 源文件 | 功能描述 |
| --- | --- | --- | --- |
| 1 | 固件升级 | `bsp/bsp_ota.c` | 云端触发 OTA，下载写入备用分区，进度回调驱动 UI，重启后自检标记有效 |
| 2 | BluFi 配网 | `blufi/blufi_init.c` | BLE 配网，动态设备名（MAC 派生 `EchoPals-XXXXXX`），CustomData 通道下发 token |
| 3 | 配网引导 | `ui/ui_port.c` | 未配网时显示二维码 / 引导图 |
| 4 | 产线烧录 | `bsp/bsp_flash.c` + `2.py` / `3.py` | 外挂 Flash 资源镜像烧录，容量自动探测与重打包 |

### 4.12.3 模块接口

| 序号 | 名称 | 备注 |
| --- | --- | --- |
| 1 | `bsp_ota_trigger` | 触发升级 |
| 2 | `bsp_ota_register_progress_cb` | 注册进度回调 |
| 3 | `bsp_ota_mark_valid` | 标记当前固件有效（回滚保护） |
| 4 | `bsp_ota_take_pending_success` / `_clear_pending_success` | 升级成功提示的取用与清除 |
| 5 | `bsp_ota_get_current_version` | 当前版本查询 |
| 6 | `clear_wifi_and_restart` | 清除配网信息并重启 |

### 4.12.4 模块属性

- 模块类型：新开发；
- `ota_task` 栈 8192B，**必须位于内部 SRAM**（OTA 写 Flash 期间 cache 关闭）；
- BLE Controller 与 WiFi 共存受限，配网态与工作态启动时二选一；
- A/B 双分区（`ota_0` / `ota_1`），各 3.9MB。

### 4.12.5 故障处理

- OTA 期间必须先停止唤醒引擎、触摸扫描、会话与交互（`*_stop_for_ota` 系列接口），释放 CPU1 与 DMA，避免与 Flash 写入并发导致崩溃；
- 下载中断或校验失败：不切换 `otadata`，重启后仍运行原分区；
- 长按重置 WiFi 时 LVGL 可能尚未初始化：重置函数内部先判 `s_lvgl_ready`，未就绪则跳过 UI 提示直接执行 `esp_wifi_restore`。

---

# 5 运行架构视图

## 5.1 启动时序

`app_main()` → `application_init()`，严格按以下顺序初始化，顺序约束不可调换：

| 步骤 | 动作 | 约束原因 |
| --- | --- | --- |
| 0 | GPIO14 强制拉低 | 上电弱上拉会误触发舵机抖动，连锁导致 I2C NACK |
| 1 | `bsp_flash_init` + 资源扫描 | 外挂 Flash 挂载，UI 资源依赖 |
| 2 | `bsp_board_get_instance` | 创建 EventGroup，必须最先 |
| 3 | `bsp_board_nvs_init` | 早于所有使用 NVS 的模块 |
| 4 | LCD + UI 初始化 | **必须早于配网**，否则配网页调 `lvgl_port_lock` 时 LVGL 未 init 会断言崩溃 |
| 5 | `audio_init` | 早于唤醒引擎（feed 任务立即投喂） |
| 6 | `ww_init` 任务（钉 CPU1，prio 6） | 模型加载让出 CPU0 给 taskLVGL，开机 GIF 才不卡 |
| 7 | `bsp_board_wifi_main` | 阻塞等待联网 / 进入配网 |
| 8 | `protocol_mqtt_start` | 必须在联网后 |
| 9 | `session_init` | WebSocket 预连接 |
| 10 | `bsp_board_servo_init` | 舵机 LEDC 硬件初始化 |
| 11 | `servo_manager_init` | 依赖步骤 10 |
| 12 | `interaction_manager_init` | 依赖步骤 11 |
| 13 | `touch_scan` / `touch_dispatch` 任务 | 依赖步骤 12 |

函数返回后系统进入事件驱动模式，所有逻辑由各任务与回调推进。

## 5.2 并发任务视图

| 任务名 | 栈 | 优先级 | 绑核 | 栈所在内存 | 职责 |
| --- | --- | --- | --- | --- | --- |
| `main` | 8192 | 1 | CPU0 | 内部 SRAM | 初始化编排，完成后退出 |
| `taskLVGL` | 组件配置 | 5 | CPU0 | 内部 SRAM | LVGL 绘制与定时器 |
| `audio_feed` | 8192 | 5 | CPU1 | PSRAM | I2S 采集并投喂 AFE |
| `afe_fetch` | esp-sr 内建 | 5 | CPU1 | — | AFE 取帧处理 |
| `mn_detect` | esp-sr 内建 | 5 | CPU1 | — | MultiNet 命令词识别 |
| `ww_init` | 8192 | 6（临时提升） | CPU1 | **内部 SRAM** | 一次性模型加载，跑完自删 |
| `encoder_task` | 32768 | 5 | CPU1 | PSRAM | OPUS 编码 |
| `decoder_task` | 32768 | 5 | CPU0 | PSRAM | OPUS 解码 |
| `play_task` | 见模块宏 | 见模块宏 | 见模块宏 | PSRAM | 扬声器播放 |
| `offline_audio` | 32768 | 5 | CPU0 | PSRAM | 本地音频播放，一次性 |
| `session_evt_tsk` | 8192 | 5 | CPU1 | PSRAM | 会话事件处理 |
| `ws_sender` | 4096 | 4 | CPU0 | PSRAM | WebSocket 上行发送 |
| `heartbeat_task` | 4096 | 4 | 不绑 | PSRAM | MQTT 心跳 |
| `mqtt_reconn` | 3072 | — | 不绑 | PSRAM | MQTT 重连（须 WithCaps 自删） |
| `async_ww_update` | 4096 | — | 不绑 | 内部 SRAM | 唤醒词更新，一次性 |
| `async_unbind` | 8192 | — | 不绑 | 内部 SRAM | 云端解绑，一次性 |
| `ia_worker` | 8192 | 5 | 不绑 | PSRAM | 情绪组合动作串行执行 |
| `servo_mgr` | 4096 | 6 | 不绑 | PSRAM | 舵机指令队列执行 |
| `touch_scan` | 4096 | — | 不绑 | **内部 SRAM** | 触摸扫描（涉及 Flash 操作） |
| `btn_task` | 3072 | 5 | CPU0 | 内部 SRAM | 按键监控（常驻） |
| `btn_reset` | 8192 | 5 | 不绑 | 内部 SRAM | 长按重置 WiFi，一次性 |
| `bat_mon` | 见 bsp_config | 见 bsp_config | 不绑 | 内部 SRAM | 电池采样与电量估算 |
| `bat_log` | 见 bsp_config | 见 bsp_config | 不绑 | PSRAM | 电量日志 |
| `ota_task` | 8192 | 5 | 不绑 | **内部 SRAM** | OTA 下载写入 |
| `nvs_save` | 一次性 | — | 不绑 | **内部 SRAM** | NVS 写入代理 |
| `udp_log_send` | 4096 | — | 不绑 | PSRAM | 日志 UDP 外发 |
| `erase_prog` | 3072 | 1 | 不绑 | 内部 SRAM | 产线擦除进度显示 |
| `Tmr Svc` | 4096 | 1 | — | 内部 SRAM | FreeRTOS 软件定时器服务（栈已从 2048 提升） |

**绑核策略**：CPU0 承载图形与网络协议栈，CPU1 承载实时音频链路（`audio_feed` / `afe_fetch` / `mn_detect` / `encoder_task`），两者互不抢占。CPU0 容量守恒——在 CPU0 上腾挪只能在「模型加载快」与「GIF 流畅」之间二选一，故一次性重计算一律外派到 CPU1。

## 5.3 状态机视图

系统存在两套**相互独立**的状态机，不得混淆：

**（1）会话状态机**

```
              唤醒词 / 提示音
   IDLE  ---------------------->  LISTENING
     ^                                 |
     |                                 | VAD 静音判定 / 主动结束
     |  播放结束 / 超时                  v
     +---------------------------  PLAYING
                        ^              |
                        +--------------+
                     唤醒词打断（WAKE_INTERRUPT）
```

**（2）功耗状态机**

```
   正常运行  --超时 60s 无活动-->  深度待机（关屏 / 停舵机 / 关震动 / codec 低功耗）
      ^                                  |
      +------- 触摸 / 唤醒 / 充电 --------+

   正常运行  --长按电源键-->  关机（拉低电源使能引脚）
```

「空闲」不等于「待机」：空闲是会话状态机的 IDLE，待机是功耗状态机的降级档，待机是空闲的子状态。待机 GIF 表只读 `gif_path`，不带震动与音频，与情绪矩阵完全独立。

## 5.4 数据流视图

**上行（用户说话）**

```
麦克风 -> I2S DMA -> audio_feed -> AFE(AEC/NS/VAD) -+-> WakeNet/MultiNet -> 唤醒回调 -> session
                                                    |
                                                    +-> 环形缓冲 -> encoder_task(OPUS)
                                                          -> ws_sender -> WebSocket -> 云端
```

**下行（设备应答）**

```
云端 -> WebSocket -> session_evt_tsk -> 环形缓冲 -> decoder_task(OPUS->PCM)
     -> play_task -> I2S -> ES8311 -> 扬声器
                                   +-> AEC 参考信号回灌 AFE
```

**本地交互**

```
TTP223 -> touch_scan -> 事件队列 -> touch_dispatch -+-> games_handle_touch（游戏中）
                                                    +-> ui_dispatch_touch_event（功能盘）
                                                    +-> ui_interaction_play（情绪矩阵）
                                                          -> ia_worker -+-> ui_request_*_gif
                                                                        +-> servo_manager
                                                                        +-> bsp_motor_set
                                                                        +-> offline_audio_play
```

---

# 6 内存与存储架构

## 6.1 内存分区原则

内部 SRAM 是最稀缺资源，分配遵循以下三条硬规则：

1. **默认放 PSRAM**：任务栈默认以 `xTaskCreatePinnedToCoreWithCaps(..., MALLOC_CAP_SPIRAM)` 分配；
2. **三类任务栈必须放内部 SRAM**：会执行 Flash 擦写（NVS 写、OTA 写、FATFS/SPIFFS 写）的任务、模型加载任务、涉及 Flash 操作的驱动任务。原因是 PSRAM 与 Flash 共用 cache/MMU，Flash 操作期间 cache 关闭会使 PSRAM 映射同时失效，IDF 将主动断言 panic；
3. **PSRAM 栈任务必须用 `vTaskDeleteWithCaps` 自删**：用普通 `vTaskDelete` 自删会泄漏栈内存与 TCB，在周期性重连一类场景下形成缓慢泄漏。

## 6.2 Flash 分区表

主 Flash 16MB，分区如下（`partitions.csv`）：

| 名称 | 类型 | 子类型 | 偏移 | 大小 | 用途 |
| --- | --- | --- | --- | --- | --- |
| `nvs` | data | nvs | 0x9000 | 24KB | 配网凭据、音量、闹钟、日历、天气缓存、舵机校准 |
| `otadata` | data | ota | 0xF000 | 8KB | OTA 分区选择记录 |
| `phy_init` | data | phy | 0x11000 | 4KB | 射频校准数据 |
| `ota_0` | app | ota_0 | 0x20000 | 0x3F0000（约 3.9MB） | 应用固件 A |
| `ota_1` | app | ota_1 | 自动 | 0x3F0000（约 3.9MB） | 应用固件 B |
| `model` | data | spiffs | 自动 | 0x800000（8MB） | esp-sr 语音模型 |

## 6.3 外挂 Flash 资源分区

外挂 SPI Flash（16MB 或 32MB，各批次不统一）挂载为 FATFS，路径前缀 `/S`：

| 路径 | 内容 |
| --- | --- |
| `/S/assets/gif/` | 表情与状态 GIF 资源 |
| `/S/voice/` | 离线提示音（MP3 / P3） |

容量在运行期通过 JEDEC ID 探测，固件不写死容量常量，防止在 16MB 板上按 32MB 越界擦写。产线工具需按实际容量重新打包镜像。

---

# 7 软件故障处理

## 7.1 故障处理总则

- 设计遵循「故障 → 安全」原则：任一模块异常不得导致整机不可用，优先降级而非中止；
- 可恢复错误一律返回错误码由调用方处置，禁止在业务路径上使用 `ESP_ERROR_CHECK`（其 abort 行为会掩盖真实根因）；
- 关键故障统一经串口与 UDP 日志双通道输出，便于无串口现场取证。

## 7.2 故障分级与响应

| 级别 | 定义 | 响应 |
| --- | --- | --- |
| L1 提示 | 不影响功能（如天气拉取失败） | 使用缓存值，UI 标注 |
| L2 降级 | 单一功能不可用（如唤醒模型加载失败） | 关闭该功能，其余功能继续 |
| L3 重连 | 网络中断 | 指数退避重连，超阈值转离线模式 |
| L4 复位 | 不可恢复的运行时异常 | 记录现场后复位，重启后恢复进度使用户无感知 |

## 7.3 已知系统性风险与设计对策

| 风险 | 机理 | 设计对策 |
| --- | --- | --- |
| PSRAM 栈任务写 Flash 触发断言 | PSRAM 与 Flash 共用 cache/MMU，Flash 操作关 cache 使栈不可读 | NVS/OTA/模型加载类任务栈强制内部 SRAM；NVS 写统一投递 `nvs_save` 任务 |
| PSRAM 栈任务泄漏 | 普通 `vTaskDelete` 无法释放 `WithCaps` 分配的栈 | 成对使用 `xTaskCreate*WithCaps` / `vTaskDeleteWithCaps` |
| 定时器回调栈溢出 | `Tmr Svc` 栈过小，回调内打印日志 + 中断嵌套帧压爆栈 | `Tmr Svc` 栈提至 4096；定时器回调内禁止打日志、禁止阻塞操作 |
| CPU0 优先级倒挂 | `taskLVGL`(prio 5) 压着 `main`(prio 1)，重计算被饿死触发看门狗 | 一次性重计算外派 CPU1，或临时提升优先级 |
| 阻塞调用卡死 LVGL 线程 | 在 LVGL 回调中调用含 `vTaskDelay` 的接口 | 执行器一律非阻塞置位 + 引擎按时关闭 |
| LVGL 未就绪时被调用 | 配网 / 按键路径可在 LVGL init 之前触发 | UI 门面内部以 `s_lvgl_ready` 判定，未就绪直接返回 |
| CPU1 无看门狗覆盖 | 实时音频任务全在 CPU1，锁死锁时无告警 | 建议开启 CPU1 的 TWDT 覆盖（见第 8 节） |

## 7.4 系统报错输出

- 串口（USB-Serial-JTAG）实时输出分级日志；
- 联网状态下镜像到 UDP 日志服务，便于整机装配后无线取证；
- 崩溃时输出 backtrace 与堆状态快照；现场卡死时勿断电，通过 USB-JTAG 执行 `thread apply all bt` 取证。

---

# 8 待办与遗留问题

| 编号 | 问题 | 状态 | 说明 |
| --- | --- | --- | --- |
| 1 | esp-sr MultiNet CTC 解码崩溃 | 未结案 | 定位为 esp-sr 库内 double free，已向上游提交 issue；本地止血为进度持久化 |
| 2 | 长时间运行后全系统静默僵死 | 排查中 | 头号嫌疑为 PSRAM 堆锁被永久持有；需开启 CPU1 看门狗覆盖以取证 |
| 3 | 屏幕刷新逐带可见 / 换图双图同框 | 已判定为硬件限制 | 无 TE 引脚，无法与扫描线同步，硬件未引出信号即无软件解，不再投入 |
| 4 | MQTT 账密硬编码、未启用 TLS、Flash/NVS 未加密 | 待处理 | 已记录为安全待办，当前阶段优先功能开发 |
| 5 | 开机爆音 | 硬件方案 | PA_CTRL 未接 GPIO，软件音量软启动无效，终选 NMOS 延时开关硬件方案 |
| 6 | 低功耗关机档断电异常 | 待硬件确认 | 需按实际原理图确认 OPT-OUT 真实 GPIO，并用示波器测引脚电压 |
| 7 | 各模块「对应软件需求编号」列 | 待填 | 需与《软件需求规格说明书》完成追溯对齐 |
| 8 | 模块「功能模块编号」体系 | 待定 | 当前为临时编号（M-XXX），待与需求规格统一编号规则 |
