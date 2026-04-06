---
name: 项目概况
description: EchoPals Echo2 是基于 ESP32-S3 的智能语音助手项目，包含唤醒词、ASR、LLM、TTS 完整对话流程
type: project
---

EchoPals Echo2 是一个 ESP32-S3 智能语音助手项目，核心功能是语音对话（唤醒→录音→ASR→LLM→TTS→播放）。

**硬件**: ESP32-S3 + ES8311 音频编解码器 + LCD(240×320)
**协议**: MQTT（心跳/配置）+ WebSocket（实时语音流）
**音频**: OPUS 编解码，16kHz 单声道，24kbps
**唤醒词**: ESP-SR MultiNet6，支持中英文双模型动态切换（mn6_cn/mn6_en）
**分区**: 8MB model 分区用于存放唤醒词模型

**Why:** 了解项目全貌有助于在后续会话中快速理解用户需求的上下文。
**How to apply:** 当用户提到模块名称或功能时，结合此架构理解其在整体中的位置。
