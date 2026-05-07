#pragma once

/**
 * @file bsp_config.h
 * @brief 板级硬件引脚与参数全量配置
 *
 * 集中定义 ESP32-S3 开发板上所有外设的 GPIO 引脚分配和音频参数。
 * 修改引脚时只需在此处修改，所有使用该宏的模块自动生效。
 *
 * 引脚分配概览：
 *   I2C  : SDA=8,  SCL=15
 *   I2S  : MCLK=17, BCLK=9, WS=5, DIN=4, DOUT=6
 *   LCD  : CS=10, MOSI=11, SCLK=12, DC=13, RST=14, BK=48
 *   Touch: GPIO1, GPIO2, GPIO7
 *   Motor: GPIO16(震动), GPIO21(右臂), GPIO38(头), GPIO47(左臂)
 */

// ─── 1. ES8311 音频编解码器引脚 ─────────────────────────────────────────────
// ES8311 通过 I2C 接收寄存器配置命令，通过 I2S 传输音频 PCM 数据
// I2C: 低速控制总线（初始化时配置 ES8311 工作模式、增益等）
// I2S: 高速数据总线（运行时传输 16kHz / 16-bit 音频流）

#define BSP_CODEC_SDA_PIN 8  ///<            ES8311 I2C 数据线（SDA），配置编解码器寄存器
#define BSP_CODEC_SCL_PIN 15 ///<            ES8311 I2C 时钟线（SCL）

#define BSP_CODEC_MCLK_PIN 17 ///<  8          I2S 主时钟（MCLK），提供给 ES8311 作为参考时钟源
#define BSP_CODEC_BCLK_PIN 9  ///<            I2S 位时钟（BCLK），每个采样位产生一个时钟沿
#define BSP_CODEC_WS_PIN 5    ///<            I2S 帧同步（WS / LRCK），区分左右声道，单声道时也必须保留
#define BSP_CODEC_DIN_PIN 4   ///<            I2S 数据输入（DIN）：麦克风采集数据流向 ESP32
#define BSP_CODEC_DOUT_PIN 6  ///<            I2S 数据输出（DOUT）：ESP32 播放数据流向 ES8311 → 扬声器

// ─── 2. 音频采样参数 ─────────────────────────────────────────────────────────
// 这些参数必须与 AFE（音频前端）和 OPUS 编解码器的配置保持一致

#define BSP_CODEC_SAMPLE_RATE 16000  ///< 采样率 16kHz（AFE、MultiNet、OPUS 的标准输入要求）
#define BSP_CODEC_BITS_PER_SAMPLE 16 ///< 采样位深 16-bit（每个采样点占 2 字节）

// ─── 3. 触摸铜箔引脚 ─────────────────────────────────────────────────────────
// ESP32-S3 内置电容触摸检测，触摸铜箔直接连接到对应 GPIO
// 注意：触摸引脚与普通 GPIO 共用，触摸检测期间该引脚不可作普通 GPIO 使用

#define BSP_TOUCH_1_PIN 1     ///< 触摸铜箔 1（对应 ESP32-S3 TOUCH1 通道）头部
#define BSP_TOUCH_2_PIN 2     ///< 触摸铜箔 2（对应 ESP32-S3 TOUCH2 通道）背部
#define BSP_TOUCH_3_PIN 3     ///< 触摸铜箔 3（对应 ESP32-S3 TOUCH7 通道）腹部
#define BSP_TOUCH_PREV_PIN 7  ///< 翻页前一页触摸（对应 ESP32-S3 TOUCH3 通道）GPIO3 空闲
#define BSP_TOUCH_NEXT_PIN 14 ///< 翻页后一页触摸（对应 ESP32-S3 TOUCH14 通道）GPIO14 由 LCD RST 迁出

// ─── 4. ST7789 LCD 屏幕 SPI 引脚 ────────────────────────────────────────────
// ST7789 使用 SPI 接口，仅支持写入（没有 MISO），时钟可高达 80MHz
// DC 引脚区分数据（高电平）和命令（低电平）

#define BSP_LCD_CS_PIN 10   ///< LCD 片选（CS/NSS），低电平激活
#define BSP_LCD_MOSI_PIN 11 ///< LCD 数据线（MOSI），主发从收，单向写
#define BSP_LCD_SCLK_PIN 12 ///< LCD 时钟线（SCLK），最高 80MHz
#define BSP_LCD_DC_PIN 13   ///< LCD 数据/命令选择（D/C）：高=数据，低=命令
#define BSP_LCD_RST_PIN 18  ///< LCD 硬件复位（RST），低电平触发复位（从 GPIO14 迁出，腾出 TOUCH14 给翻页）
#define BSP_LCD_BK_PIN 48   ///< LCD 背光控制（BK），高电平开启背光

#define BSP_LCD_WIDTH 320  ///< LCD 屏幕宽度（像素，横向）
#define BSP_LCD_HEIGHT 240 ///< LCD 屏幕高度（像素，纵向）

// ─── 5. 运动与反馈外设引脚 ──────────────────────────────────────────────────
// 所有运动外设通过 PWM 信号驱动
// 震动马达：提供触觉反馈（如唤醒、提醒）
// 舵机（Servo）：控制机器人肢体姿态，范围通常 0~180°

#define BSP_MOTOR_VIB_PIN 16   ///< 震动马达 PWM 引脚（触觉反馈）
#define BSP_SERVO_R_ARM_PIN 21 ///< 右臂舵机 PWM 引脚
#define BSP_SERVO_HEAD_PIN 38  ///< 头部舵机 PWM 引脚
#define BSP_SERVO_L_ARM_PIN 47 ///< 左臂舵机 PWM 引脚
// 舵机逻辑通道映射 (供上层调用)
#define CH_HEAD 0  ///< 头部舵机逻辑通道编号（对应 LEDC_CHANNEL_0，引脚 GPIO38）
#define CH_L_ARM 1 ///< 左臂舵机逻辑通道编号（对应 LEDC_CHANNEL_1，引脚 GPIO47）
#define CH_R_ARM 2 ///< 右臂舵机逻辑通道编号（对应 LEDC_CHANNEL_2，引脚 GPIO21）

// 设备状态标志位 (添加舵机的 BIT)
#define BOARD_STATUS_SERVO_READY (1 << 3) ///< 舵机已初始化就绪（向 board_status 事件组置位）

// ─── 6. 舵机速度宏（step_ms：每度等待毫秒数，值越大运动越慢）───────────────────
// 统一在此定义，供 bsp_servo.c 和 servo_manager.h 共同引用，避免重复定义
#define SERVO_SPEED_INSTANT 0U    ///< 瞬间到位（无平滑，上电归中禁止使用）
#define SERVO_SPEED_VERY_FAST 2U  ///< 极快（2ms/度，适合快速抖动动作）
#define SERVO_SPEED_FAST 5U       ///< 快速（5ms/度，适合挥手、点头等活泼动作）
#define SERVO_SPEED_MID 15U       ///< 中速（15ms/度，适合大多数情绪动作）
#define SERVO_SPEED_SLOW 30U      ///< 慢速（30ms/度，适合慵懒、委屈等缓慢动作）
#define SERVO_SPEED_VERY_SLOW 50U ///< 极慢（50ms/度，适合细腻的情感表达）
// ─── 7. 外部 Flash 引脚配置 ─────────────────────────────────────────────

// 使用你确定的 SPI3 引脚
#define EXT_FLASH_MOSI 41
#define EXT_FLASH_MISO 42
#define EXT_FLASH_SCLK 39
#define EXT_FLASH_CS 40