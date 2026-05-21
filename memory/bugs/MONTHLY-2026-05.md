---
name: monthly-bugs-2026-05
description: 2026-05 月度新增 BUG 发现-解决对照表（BUG-013/014/015/016 速查）
metadata:
  type: project
---

# 2026-05 月度新增 BUG 对照表

> 本月新增 4 个坑点（BUG-013/014/015/016）。每个 bug 详档另见独立文件，本表只做**发现 ↔ 解决**两段对照速查。

| 编号 | 模块 | 发现日期 | 解决日期 | 状态 | 提交 |
|---|---|---|---|---|---|
| [BUG-013](BUG-013.md) | session / audio | 2026-05-11 | 2026-05-11 | 已修复（含在 23a1b34） | 23a1b34 |
| [BUG-014](BUG-014.md) | session | 2026-05-12 | 2026-05-12 | 已修复 | 23a1b34 |
| [BUG-015](BUG-015.md) | bsp / application | 2026-05-14 | 2026-05-14 | 已修复 | aecca09 |
| [BUG-016](BUG-016.md) | bsp_codec / 硬件 | 2026-05-15 | — | 诊断指纹已沉淀，**根因属硬件虚焊**，无代码修复 | 6edb24f / 6b51101 |

---

## BUG-013：长 TTS 残留被识别为下一轮输入

### 发现
- 触发条件：LLM 长回复（≥3s）后多轮对话
- 症状：服务端 `TTS_STOP` 已到，喇叭仍在响 → AEC 没消干净 → VAD 把残音判 SPEECH → 编码上传 → 服务端把残音当新提问
- 关键日志：`检测到说话结束（VAD 静音 600ms），通知服务器`（紧跟 TTS_STOP）
- 根因链：3 层缓冲（dec_input 5s + dec_output 1.28s + DMA/振膜）+ 1500ms 兜底太短 + 仅靠 VAD 判排空

### 解决
- 新增接口 `audio_processor_get_pending_bytes()` 读 `dec_input + dec_output` 实际水位
- 排空判定改"三重 AND 锁"：① 物理静音≥400ms ② pending_bytes==0 ③ VAD_SILENCE 连续 8 帧
- 兜底 1500ms → 5000ms
- 关键文件：[main/audio/audio_processor.c](../../../main/audio/audio_processor.c)、[main/audio/audio_processor.h](../../../main/audio/audio_processor.h)、[main/session/session.c](../../../main/session/session.c)

---

## BUG-014：多轮 SERVER_READY 丢失 + 风扇噪声玩坏尾端字保护

### 发现（两条独立症状）
- **A**：多轮第二句卡 9~10s，云端报 `Error committing input audio buffer: buffer too small`
- **B**：风扇环境下"几点了"单句被录 26 秒
- 根因 A：`PROTOCOL_EVENT_COMPLETE` 在多轮分支 ClearBits(SERVER_READY)，但多轮不再握手，**没人重新 Set 回来** → ws_sender 哑火 → enc_output 撑爆
- 根因 B：尾端字保护 `if (avg_amp > 100) vad = SPEECH` 被风扇持续 100+ 振幅持续覆盖 → EOS 定时器每帧被掐掉 → 永远等不到 600ms 静音

### 解决
- 修复 A：COMPLETE 多轮分支 `ClearBits → SetBits`（或直接删除该行），保持 READY 态
- 修复 B：尾端字保护加 **800ms 熔断窗口**（`TAIL_PROTECT_MAX_MS`）— 救尾字最多 800ms，超过就放手交给 AFE VAD
- 修复 C 兜底：单句最大时长 **8s 硬切**（`UTTERANCE_MAX_MS=8000`）— 任何感知层失效都能切断
- 修复 D：7 处状态切换点同步清零 `s_first_silence_tick / s_speech_start_tick`
- 关键文件：[main/session/session.c](../../../main/session/session.c)

---

## BUG-015：GPIO14 FSPIWP 弱上拉 → I2C NACK + 共 timer 舵机失灵

### 发现
- 现象：舵机接 GPIO 14 后上电瞬间 I2C 0x30 报 NACK，**同时 GPIO 4 右臂舵机完全不动**（连锁失灵）
- 隔离实验：移除 GPIO 14 → 4/9 正常；加回 14 → 必炸
- 根因：ESP32-S3 GPIO14 复用 FSPIWP/SUBSPIWP 字段标 `I1`（默认输入+弱上拉到 1）→ 上电瞬间舵机线 3.3V → 舵机抖动堵转 1A+ → 共用 3.3V 电源轨压降 → ES8311/触摸 IC 短暂掉电 → I2C 握手 NACK；同时 LEDC 同 timer 多通道初始化原子失败 → GPIO 4/9 跟着挂

### 解决
- 软件方案（已验证）：在 [main/application.c](../../../main/application.c) `application_init()` 最开头、**早于所有外设初始化**，用 `gpio_config` 显式：output / pull_up_disable / pull_down_disable / intr_disable，并 `set_level(0)`
- 为什么用 `gpio_config` 不用 3 行简写：显式覆盖 FSPIWP 默认 `I1=1`、显式禁中断、原子配置、ESP_ERROR_CHECK 暴露失败
- 硬件方案（推荐补充，未做）：GPIO14 → GND 加 10kΩ 下拉电阻，消除上电最初 ~50ms 软件未运行的硬件抖动
- 隐患：① 上电 50ms 仍有硬抖 ② 启动顺序绝不能乱 ③ 未来 LCD 改 QSPI 会复用回 GPIO14 ④ deep sleep 唤醒需重验 ⑤ 关机抖动

---

## BUG-016：PCM 全 0 / FFFF / 高字节恒 0x00 — ES8311 硬件诊断指纹

### 发现
- 状态：**纯硬件问题（焊接/连线），代码无修改**
- three/four 分支同时复现 → 与代码版本无关
- 沉淀产物：`[PCM 诊断]` 打印的 4 种 bytes 模式指纹表

| 模式 | 含义 | 排查方向 |
|---|---|---|
| `0000 0000` peak=0 rms=0 | DIN 完全没数据 | GPIO15(DIN)/MCLK/BCLK 任一断线，或 ES8311 没启动 |
| `FFFF FFFF` peak高 rms低 | DIN 悬空被上拉吃住 | GPIO15 焊点断开 |
| `0077 0077 00C7 00C7`（高字节恒 0x00、采样两两重复）peak=32768 | I2S 位宽/帧对齐错位 | BCLK/WS 不稳，或 ES8311 寄存器格式与 ESP32 slot 配置不一致 |
| 段 A FFFF / 段 B 0000 / 段 C 杂值反复切换 | 接触瞬通瞬断 | 经典虚焊 |

### 解决（排查路径，非代码 fix）
- 排查捷径：`[PCM 自检]` 启动期塞已知值算 peak/rms，**peak=12345 通过 → 诊断代码可信**，跳过自证岔路
- 修复路径（成本递增）：
  1. 重熔补焊 ES8311 ASDOUT/BCLK/MCLK 引脚根部（QFN 加助焊剂）
  2. 万用表蜂鸣档点测三根线
  3. 示波器量：MCLK ~4.096 MHz / BCLK ~512 kHz / WS 16 kHz 50% / DIN 跟随 BCLK
  4. 抓 I2C 看 ES8311 地址 0x18 是否 ACK
  5. ES8311 自身：VDDA/VDDD、RESET 拉高、晶振起振
- 引脚映射（PCB 新版）：SDA=16 SCL=17 MCLK=8 BCLK=46 WS=7 DIN=15 DOUT=6（是 BUG-015 引脚大调整后的新映射）

---

## 月度经验沉淀

1. **服务端事件 ≠ 本地物理完成**（BUG-013）— 所有"播放结束/就绪"判定都要叠加本地链路状态
2. **强制改判逻辑必须有熔断**（BUG-014）— 振幅门、VAD 强覆盖等都需要时间窗或硬切兜底
3. **多轮模式下事件位 Clear 要问"谁 Set 回来"**（BUG-014）— 握手类事件位不能在无握手动作的状态切换里轻易清
4. **ESP32-S3 GPIO 复用字段 `I1` 要警惕**（BUG-015）— 上电默认弱上拉到 1，对舵机/外设是定时炸弹
5. **PCM 模式指纹比"听感"更可靠**（BUG-016）— 字节模式诊断 4 类一一对应硬件失效点，比"麦克风像没声音"准 10 倍
