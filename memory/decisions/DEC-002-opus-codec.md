---
name: 音频传输使用 OPUS 编码
date: 2026-03
status: 已确定
related_commits: 07a1707
---

## 决策
WebSocket 实时音频传输使用 OPUS 编解码，16kHz 单声道，24kbps。

## 背景
设备录音产生的原始 PCM 数据量大（16kHz × 16bit × 1ch = 256kbps），直接通过 WiFi 传输浪费带宽且延迟高，需要选择合适的压缩编码。

## 对比分析

| 特性 | PCM（无压缩） | OPUS | AAC | Speex |
|------|-------------|------|-----|-------|
| 码率 | 256 kbps | 24 kbps | 48 kbps | 24 kbps |
| 压缩比 | 1:1 | ~10:1 | ~5:1 | ~10:1 |
| 延迟 | 0 | 20-60ms | 100ms+ | 30ms |
| 语音质量 | 原始 | 优秀 | 良好 | 一般 |
| ESP-IDF 支持 | 原生 | esp_audio_codec | 需第三方 | 已停更 |
| CPU 占用 | 无 | 中等 | 高 | 低 |

## 选择理由
1. **压缩率**: 24kbps 约为 PCM 的 1/10，大幅节省带宽
2. **语音优化**: OPUS 专为语音和音乐设计，同码率下语音质量最佳
3. **低延迟**: 20ms 帧长，适合实时对话场景
4. **官方支持**: ESP-IDF 的 `esp_audio_codec` 组件原生支持 OPUS 编解码
5. **双核利用**: ESP32-S3 双核（240MHz），Core0 运行主逻辑，Core1 运行编解码，互不干扰

## 参数配置
```c
// 编码器参数
采样率: 16000 Hz
声道数: 1（单声道）
位深:   16-bit
码率:   24000 bps
帧长:   20ms（320 个采样点）

// 单帧数据量
PCM 输入:  320 samples × 2 bytes = 640 bytes
OPUS 输出: ~60 bytes（变长）
```

## 影响范围
- `main/audio/audio_encoder.c` — PCM→OPUS 编码
- `main/audio/audio_decoder.c` — OPUS→PCM 解码
- `main/audio/audio_processor.c` — 编解码管道和 Ring Buffer 管理
- `idf_component.yml` — 依赖 `esp_audio_codec` 组件
