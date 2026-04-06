---
name: 选择 ES8311 替代 MAX98357A
date: 2026-03
status: 已确定
related_commits: ee3c02c
---

## 决策
音频编解码器从 MAX98357A（纯 DAC）切换为 ES8311（ADC+DAC 全双工）。

## 背景
项目需要实现"听+说"双向语音交互。初期选用 MAX98357A 仅能播放音频，无法采集麦克风输入，必须更换。

## 对比分析

| 特性 | MAX98357A | ES8311 |
|------|-----------|--------|
| 类型 | I2S DAC（纯播放） | I2S Codec（ADC+DAC） |
| 录音 | ❌ 不支持 | ✅ 支持（内置 ADC） |
| 播放 | ✅ 3W 功放 | ✅ 需外接功放 |
| 接口 | I2S 数据线 | I2C 控制 + I2S 数据 |
| 驱动 | 无需配置（硬件直通） | 需要 I2C 寄存器配置 |
| ESP-IDF 支持 | 基础 I2S 驱动 | esp_codec_dev v1.5.4 官方组件 |

## 选择理由
1. **功能完整性**: ES8311 同时支持录音和播放，一颗芯片搞定全双工
2. **官方支持**: ESP-IDF 的 `esp_codec_dev` 组件原生支持 ES8311，有成熟的初始化流程
3. **音质**: 24-bit ADC，信噪比 95dB，适合语音采集

## 引脚占用
```
I2C 控制总线:
  SDA → BSP_CODEC_I2C_SDA_PIN
  SCL → BSP_CODEC_I2C_SCL_PIN

I2S 数据总线:
  MCLK → BSP_CODEC_MCLK_PIN
  BCLK → BSP_CODEC_BCLK_PIN  
  WS   → BSP_CODEC_WS_PIN
  DOUT → BSP_CODEC_DOUT_PIN
  DIN  → BSP_CODEC_DIN_PIN
```
引脚定义集中在 `main/bsp/bsp_config.h`。

## 踩过的坑
- I2S 引脚 MCLK/BCLK 配置错误导致无声音（见 BUG-H05）
- 需要先初始化 I2C，再初始化 I2S，顺序不能反

## 影响范围
- `main/bsp/bsp_codec.c` — ES8311 驱动和音频采集任务
- `main/bsp/bsp_config.h` — 引脚定义
- `idf_component.yml` — 依赖 `esp_codec_dev` 组件
