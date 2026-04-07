# Echo2 项目记忆存放点

> 此目录是 Claude Code 自动记忆系统的**可见副本**，方便用户随时查阅。
> 系统记忆源文件位于：`C:\Users\esthe\.claude\projects\d--new-Echopals-Echo2\memory\`

---

## 记忆索引

### 用户信息
- [用户语言偏好](user_language.md) — 用户是中国人，所有对话必须使用中文

### 用户偏好 / 反馈
- [用户偏好风格](feedback_style.md) — 喜欢总分总结构的分析和规划方式
- [新文件追踪偏好](feedback_new_file_tracking.md) — 每次使用新文件时记住问题并主动总结
- [自动记忆写入偏好](feedback_auto_memory.md) — 主动记录对话细节、偏好、错误点、修改记录
- [主动建议偏好](feedback_proactive.md) — 用户鼓励主动发挥，提出良性建议
- [坑点记录规范](feedback_bug_detail.md) — 所有坑点必须标注精确文件路径和行号

### 项目信息
- [项目概况](project_overview.md) — ESP32-S3 智能语音助手，唤醒→ASR→LLM→TTS 完整对话流程

### 参考资料
- [开发环境](reference_dev_env.md) — ESP-IDF v5.3.4 路径和工具链配置
- [大类指令清单](reference_commands.md) — 10 大类 30 个子能力，说大类名自动全部执行
- [四大场景命令](reference_scenarios.md) — 查看结构/修复bug/添加功能/优化代码 四大标准流程

---

## 踩坑日志 — [ALL_BUGS.md](bugs/ALL_BUGS.md)（合并版，共 33 条）

| 类别 | 数量 | 说明 |
|------|------|------|
| 严重（未处理） | 7 | 凭证泄露、崩溃、内存泄漏、死锁 |
| 历史已修复 | 12 | 分区名、栈溢出、引脚配错、并发 |
| 中等（待处理） | 14 | 硬编码、缓冲区、返回值未检查 |
| 低优先级 | 6 | TODO 标记、注释代码 |

---

## 决策记录 (`decisions/`)

| 编号 | 决策 | 关键内容 |
|------|------|---------|
| [DEC-001](decisions/DEC-001-es8311-codec.md) | ES8311 替代 MAX98357A | 全双工音频，I2C+I2S 双总线，含对比表 |
| [DEC-002](decisions/DEC-002-opus-codec.md) | OPUS 编码 16kHz/24kbps | 压缩比 10:1，20ms 帧长，含编码参数配置 |
| [DEC-003](decisions/DEC-003-websocket-realtime.md) | WS 语音 + MQTT 控制 | 双协议分工，含 MQTT 主题设计 |
| [DEC-004](decisions/DEC-004-bsp-architecture.md) | BSP 板级支持包 | 模块划分、初始化顺序、演进历史 |
| [DEC-005](decisions/DEC-005-pin-assignment.md) | 引脚分配方案 v2 | 新增舵机×3+震动+铜箔×3+ST7789，移除LED |

---

## 每日总结 (`daily/`)

| 日期 | 摘要 |
|------|------|
| [2026-04-03](daily/2026-04-03.md) | 记忆系统建设、辅助功能全部启用、33 条坑点提取 |

---

## 待处理 TODO

- `main/bsp/bsp_wifi.c:94` — BLE 配网 JSON 参数解析未实现
- `main/protocol/mqtt_protocol.c:93` — ADC 电池电量映射公式需适配硬件
