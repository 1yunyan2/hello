#pragma once

/**
 * @file bsp_config.h
 * @brief ES8311 + I2S 引脚和采样参数定义（audio_test_standalone 专用最小集）
 *
 * 引脚映射（如硬件改板，按下表修改即可）：
 *   I2C  : SDA=16, SCL=17
 *   I2S  : MCLK=8, BCLK=46, WS=7, DIN=6, DOUT=15
 */

// ─── ES8311 音频编解码器引脚 ──────────────────────────────────────────────
// I2C 控制总线（配置 ES8311 寄存器：工作模式、增益等）
// I2S 数据总线（16kHz / 16-bit 音频流）
#define BSP_CODEC_SDA_PIN  16  // I2C SDA 数据线
#define BSP_CODEC_SCL_PIN  17  // I2C SCL 时钟线
#define BSP_CODEC_MCLK_PIN 8   // I2S MCLK 主时钟（提供给 ES8311 PLL）
#define BSP_CODEC_BCLK_PIN 46  // I2S BCLK 位时钟
#define BSP_CODEC_WS_PIN   7   // I2S WS / LRCK 帧同步
#define BSP_CODEC_DIN_PIN  6   // I2S DIN 数据输入（麦克风 → ESP32）
#define BSP_CODEC_DOUT_PIN 15  // I2S DOUT 数据输出（ESP32 → 扬声器）

// ─── 音频采样参数 ────────────────────────────────────────────────────────
#define BSP_CODEC_SAMPLE_RATE     16000  ///< 采样率 16kHz
#define BSP_CODEC_BITS_PER_SAMPLE 16     ///< 采样位深 16-bit
