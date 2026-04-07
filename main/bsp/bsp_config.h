#pragma once

/**
 * @file bsp_config.h
 * @brief 板级硬件引脚与参数配置
 * 集中定义 ESP32-S3 开发板上所有外设的 GPIO 引脚分配和音频参数
 */

// ─── I2S 音频编解码器引脚 ───────────────────────────────────────────────────
// #define BSP_CODEC_PA_PIN 7       // 功放使能引脚（未使用）
#define BSP_CODEC_SDA_PIN 8         // I2C 数据线（配置编解码器寄存器）
#define BSP_CODEC_SCL_PIN 15        // I2C 时钟线
#define BSP_CODEC_MCLK_PIN 17       // I2S 主时钟
#define BSP_CODEC_BCLK_PIN 9        // I2S 位时钟
#define BSP_CODEC_WS_PIN 5          // I2S 帧同步（左右声道选择）
#define BSP_CODEC_DIN_PIN 4         // I2S 数据输入（麦克风 → ESP32）
#define BSP_CODEC_DOUT_PIN 6        // I2S 数据输出（ESP32 → 扬声器）

// ─── 音频采样参数 ───────────────────────────────────────────────────────────
#define BSP_CODEC_SAMPLE_RATE 16000       // 采样率 16kHz（语音识别标准）
#define BSP_CODEC_BITS_PER_SAMPLE 16      // 16 位采样精度

// ─── LED 引脚（暂未启用）─────────────────────────────────────────────────────
// #define BSP_LED_PIN 46

// ─── LCD 显示屏引脚（暂未启用）──────────────────────────────────────────────
// #define BSP_LCD_MOSI_PIN 48      // SPI 数据线
// #define BSP_LCD_SCLK_PIN 47      // SPI 时钟线
// #define BSP_LCD_CS_PIN 21        // 片选
// #define BSP_LCD_DC_PIN 45        // 数据/命令选择
// #define BSP_LCD_RST_PIN 16       // 复位
// #define BSP_LCD_BK_PIN 40        // 背光控制
// #define BSP_LCD_WIDTH 240        // 屏幕宽度（像素）
// #define BSP_LCD_HEIGHT 320       // 屏幕高度（像素）
