# Echo2 BUG 总表

> 所有已记录的 Bug，按发现日期排序。每条 bug 有独立详情文件，此处为快速索引。

| 编号 | 标题 | 状态 | 发现日期 | 修复日期 | 关联文件 |
|------|------|------|----------|----------|----------|
| [BUG-001](BUG-001.md) | 蓝牙内存释放崩溃 | 已规避 | 2026-03-22 | 2026-03-22 | `bsp_wifi.c` |
| [BUG-002](BUG-002.md) | SPIFFS + SPIRAM 初始化内存冲突 | 已规避 | 2026-03-31 | 2026-03-31 | 启动初始化 |
| [BUG-003](BUG-003.md) | xTaskCreatePinnedToCoreWithCaps 参数顺序错误 | 已修复 | 2026-04-08 | 2026-04-08~09 | `bsp_wifi.c` |
| [BUG-004](BUG-004.md) | WebSocket 握手消息类型字段错误 | 已修复 | 2026-04-08 | 2026-04-08 | `websocket_client.c` |
| [BUG-005](BUG-005.md) | HTTP 认证返回 201 被当作失败处理 | 已修复 | 2026-04-07 | 2026-04-07 | `auth.c:74` `auth.c:182` |
| [BUG-006](BUG-006.md) | MultiNet 检测循环占满 CPU 导致 AFE ringbuffer 溢出 | 已修复 | 2026-04-09 | 2026-04-09 | `custom_wake_word.c` |
| [BUG-007](BUG-007.md) | 唤醒词尾音误触发 EOS 静音检测 | 已修复 | 2026-04-09 | 2026-04-09 | `session.c` |
| [BUG-008](BUG-008.md) | 定时器回调直接调 session_close 导致栈溢出 | 已修复 | 2026-04-14 | 2026-04-14 | `session.c` |

---

## 按模块分类

### 硬件 / BSP 层
- **BUG-001** — 蓝牙内存释放时机不对，`esp_bt_mem_release()` 过早调用
- **BUG-002** — SPIFFS 与 SPIRAM 地址映射冲突，改用 NVS 规避
- **BUG-003** — `xTaskCreatePinnedToCoreWithCaps` API 参数顺序与 `xTaskCreate` 不同

### 协议层
- **BUG-004** — WebSocket `type:"hello"` → 应为 `type:"started"`（服务端协议字段）
- **BUG-005** — HTTP POST 创建资源返回 201，但原代码只接受 200

### 音频 / AI 处理层
- **BUG-006** — MultiNet detect 未限频，紧凑循环抢占 CPU 导致 AFE 缓冲溢出
- **BUG-007** — 唤醒词尾音静音被误判为 EOS，引入 500ms 消退保护期（VAD_GRACE_MS）

### 会话管理层
- **BUG-008** — 定时器回调中直接调 `session_close()` 栈溢出死机，改用事件队列投递

---

## 每日修复时间线

### 2026-04-07（提交 `d5adbba`）
| 类型 | 内容 | 文件 |
|------|------|------|
| 修复 BUG-005 | HTTP 认证兼容 201 状态码 | `auth.c:74` `auth.c:182` |
| 新增 | LCD 驱动初始代码 | `bsp_lcd.c`（新文件） |
| 改进 | WS 连接地址切换为带 Token 的正式接口 | `application.c:93` |

---

### 2026-04-08（提交 `e96b8bc` `0bbfe76`）
| 类型 | 内容 | 文件 |
|------|------|------|
| 修复 BUG-004 | WS 握手 `"type":"hello"` → `"type":"started"` | `websocket_client.c` |
| 修复 BUG-003 | `xTaskCreatePinnedToCoreWithCaps` 参数顺序修正 | `bsp_wifi.c` |
| 集成 | ESP-AFE 音频前端框架（NS 降噪 + WebRTC VAD） | `custom_wake_word.c/h` |
| 改进 | 音频发送超时 10s → 100ms（实时场景要求） | `websocket_client.c` |
| 改进 | BLE 配网 Token 接收增加 JSON 格式验证 | `bsp_wifi.c` |
| 改进 | 编解码器创建失败回滚逻辑（防内存泄漏） | `audio_processor.c` |
| 修复 | 重连任务内存泄漏，堆分配替代栈分配 | `session.c` |

---

### 2026-04-09（提交 `24d10cc` `aee301c` `0153c28`）
| 类型 | 内容 | 文件 |
|------|------|------|
| 修复 BUG-006 | MultiNet 每次 fetch 限制 1 次 detect，防 CPU 占满 | `custom_wake_word.c` |
| 修复 BUG-007 | 引入 VAD_GRACE_MS=500ms 消退保护期 | `session.c` |
| 决策 DEC-004 | 改用 AFE 内置 WebRTC VAD 替代帧能量检测 | `session.c` |
| 改进 | 移除 `"type":"listen"` 指令，服务端不支持 | `websocket_client.c` `session.c` |
| 改进 | session 事件队列架构（定时器只 xQueueSend，防栈溢出） | `session.c` |
| 调参 | OPUS 比特率 32→24kbps，帧长 60→20ms，复杂度 0→3 | `audio_encoder.c` |
| 调参 | 麦克风增益 10→40（适配近讲） | `bsp_codec.c` |
| ⚠️ 临时错误 | 启用 VBR — 次日发现延迟抖动，已回退 | `audio_encoder.c` |
| ⚠️ 临时错误 | 数据格式改为 `pcm` — 次日改回 `opus` | `websocket_client.c` |

---

### 2026-04-10（提交 `7279cc6` `a884816`）
| 类型 | 内容 | 文件 |
|------|------|------|
| 回退修复 | 禁用 VBR（昨日开启后发现延迟抖动），确立 DEC-001 | `audio_encoder.c` |
| 回退修复 | 音频格式从 `pcm` 改回 `opus`（协议对齐） | `websocket_client.c` |
| 修复 | 新增 `transcript` 类型路由到 stt 处理器 | `websocket_client.c` |
| 修复 | ws_sender_task 增加 `!s_stop_sent` 防重复发送 | `session.c` |
| 集成 | `bsp_lcd.c` 加入 CMakeLists（ST7789 驱动正式入构建） | `CMakeLists.txt` |
| 文档 | 13 个文件大规模注释补全（+1553 行） | 全模块 |

---

### 2026-04-14（提交 `c2df9e1`）
| 类型 | 内容 | 文件 |
|------|------|------|
| 修复 BUG-008 | 定时器超时回调改用事件队列投递，防栈溢出 | `session.c` |
| 改进 | TTS_STOP 方案 A/B 双模式预留（`#if 0/1` 切换） | `session.c` |
| 改进 | COMPLETE 事件恢复会话关闭逻辑 | `session.c` |
| 重命名 | `bsp_wake_word_load_from_nvs` → `wake_word_load_from_nvs` | `custom_wake_word.c/h` |
| 清理 | 删除 `bsp_board_check_status()` 30 行死代码 | `bsp_codec.c` |
| 修正 | 日志 emoji → 文本标记（串口兼容） | `auth.c` |
| 新增 | 舵机三轴 PWM 控制模块 | `bsp_servo.c` |
| 新增 | 电容触摸 + 震动马达模块 | `bsp_touch.c` |
| 新增 | UI 交互层 + 提醒系统框架 | `ui/interaction.c/h` `ui/reminder.c/h` |
| 依赖 | 新增 `espressif/servo ^0.1.0` 组件 | `idf_component.yml` |

---

## 尚未解决 / 待观察

| 编号 | 描述 | 备注 |
|------|------|------|
| BUG-001 | 蓝牙内存未回收，约 40KB SPIRAM 浪费 | 当前可接受，后续版本修复 |

---

## 新增 Bug 规范

新增 Bug 时：
1. 在 `bugs/` 目录创建 `BUG-XXX.md`，编号连续
2. 在本表格**总表**末尾追加一行索引
3. 在**每日修复时间线**对应日期追加记录
4. 在 `MEMORY.md` 的踩坑日志行追加链接
5. 标注精确文件路径和行号（见 `feedback_bug_detail.md`）
