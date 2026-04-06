---
name: BSP 板级支持包架构
date: 2026-03
status: 已确定
related_commits: ee3c02c, dd0b7bd
---

## 决策
将硬件驱动抽象为 BSP（Board Support Package）层，统一管理所有外设。

## 背景
项目初期，硬件引脚号、I2C/I2S 配置散落在各业务模块中（audio.c、WIFI.c、main.c），导致：
- 修改一个引脚需要搜索所有文件
- 硬件版本切换极其痛苦
- 多处硬编码引脚号容易不一致（BUG-H05 就是因此产生）

## BSP 层架构

```
main/bsp/
├── bsp_config.h      — 所有引脚定义、硬件参数（唯一真相源）
├── bsp_board.c/.h    — 板级初始化、NVS、状态管理（单例模式）
├── bsp_codec.c/.h    — ES8311 音频驱动、采集任务
├── bsp_wifi.c/.h     — WiFi 配网、BLE 配网、重连机制
└── bsp_lcd.c/.h      — LCD 显示驱动（开发中）
```

## 设计原则
1. **引脚集中**: 所有 GPIO 引脚号只在 `bsp_config.h` 中定义一次
2. **单例模式**: `bsp_board_get_instance()` 返回全局唯一的板级实例
3. **初始化顺序**: NVS → I2C → I2S → Codec → WiFi → LCD（有依赖关系）
4. **禁止绕过**: 业务层（application、session、protocol）禁止直接操作硬件寄存器

## 初始化流程
```
app_main()
  └── application_init()
        ├── bsp_board_nvs_init()        — NVS 闪存
        ├── bsp_codec_init()            — ES8311 + I2C + I2S
        ├── bsp_wifi_init()             — WiFi + BLE 配网
        ├── wake_word_engine_init()     — 唤醒词引擎
        ├── mqtt_app_start()            — MQTT 连接
        └── session_init()              — 会话管理
```

## 演进历史
1. **初期**: 引脚散落在 `audio.c`、`WIFI.c` 中（硬编码）
2. **ee3c02c**: 引入 BSP 概念，创建 `bsp/` 目录
3. **dd0b7bd**: 将 NVS 初始化封装到 BSP，引脚改为宏定义
4. **e1bd277**: 将 `audio` 模块重命名为 `bsp_codec`，明确归属

## 影响范围
- 新增硬件外设必须在 BSP 层实现驱动
- 修改引脚配置只需改 `bsp_config.h` 一处
- 业务层通过 BSP API 获取硬件能力，不关心底层细节
