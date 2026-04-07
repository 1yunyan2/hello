---
name: 双协议架构 — WebSocket 语音 + MQTT 控制
date: 2026-03
status: 已确定
related_commits: 07a1707, c547cd5
---

## 决策
采用双协议架构：MQTT 负责心跳、配置、远程控制；WebSocket 负责实时语音流传输。

## 背景
设备需要同时支持两种通信场景：①低频的状态上报和配置下发 ②高频的实时音频双向传输。单一协议无法同时满足。

## 对比分析

| 特性 | 纯 MQTT | 纯 WebSocket | MQTT + WebSocket（当前） |
|------|---------|-------------|----------------------|
| 控制指令 | ✅ 原生支持 QoS | 需自行实现 | ✅ MQTT 处理 |
| 音频流 | ❌ 不适合大数据 | ✅ 全双工流 | ✅ WebSocket 处理 |
| 断线重连 | ✅ 自动重连 | 需手动实现 | 各自独立重连 |
| 服务端复杂度 | 低 | 中 | 中（两个端点） |
| 延迟 | 50-200ms | 10-50ms | 控制走 MQTT，音频走 WS |

## 选择理由
1. **关注点分离**: MQTT 管控制面，WebSocket 管数据面，互不干扰
2. **可靠性**: MQTT 有 QoS 机制保证配置指令不丢失；WebSocket 专注低延迟传输
3. **灵活性**: MQTT broker 可独立于 WebSocket 服务器部署和扩展
4. **ESP-IDF 支持**: 两个组件都有官方实现（`esp_mqtt` + `esp_websocket_client`）

## 协议分工

```
MQTT（低频控制面）:
├── 心跳上报（5s 间隔）— 设备ID、电量、WiFi信号
├── 唤醒词远程更新 — 云端下发新唤醒词
├── 配置下发 — 服务器地址、参数调整
└── 状态通知 — 开机、重启、异常

WebSocket（高频数据面）:
├── 上行音频流 — OPUS 编码后的录音数据
├── 下行音频流 — TTS 合成的 OPUS 音频
└── 会话控制 — 开始/结束对话信令
```

## MQTT 主题设计
```
echopals/{MAC后3字节}/heartbeat    — 心跳上报
echopals/{MAC后3字节}/wakeword/set — 唤醒词下发
echopals/{MAC后3字节}/config       — 配置下发
echopals/{MAC后3字节}/status       — 状态通知
```

## 影响范围
- `main/protocol/mqtt_protocol.c` — MQTT 客户端、心跳任务、唤醒词更新
- `main/protocol/websocket_client.c` — WebSocket 连接管理
- `main/session/session.c` — 会话管理，协调音频编解码和 WS 传输
