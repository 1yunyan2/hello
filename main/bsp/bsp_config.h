// #pragma once

// /**
//  * @file bsp_config.h
//  * @brief 板级硬件引脚与参数全量配置
//  *
//  * 集中定义 ESP32-S3 开发板上所有外设的 GPIO 引脚分配和音频参数。
//  * 修改引脚时只需在此处修改，所有使用该宏的模块自动生效。
//  *
//  * 引脚分配概览：
//  *   I2C  : SDA=8,  SCL=15
//  *   I2S  : MCLK=17, BCLK=9, WS=5, DIN=4, DOUT=6
//  *   LCD  : CS=10, MOSI=11, SCLK=12, DC=13, RST=14, BK=48
//  *   Touch: GPIO1, GPIO2, GPIO7
//  *   Motor: GPIO16(震动), GPIO21(右臂), GPIO38(头), GPIO47(左臂)
//  */

// // ─── 1. ES8311 音频编解码器引脚 ─────────────────────────────────────────────
// // ES8311 通过 I2C 接收寄存器配置命令，通过 I2S 传输音频 PCM 数据
// // I2C: 低速控制总线（初始化时配置 ES8311 工作模式、增益等）
// // I2S: 高速数据总线（运行时传输 16kHz / 16-bit 音频流）

// #define BSP_CODEC_SDA_PIN 8   // 16        ES8311 I2C 数据线（SDA），配置编解码器寄存器
// #define BSP_CODEC_SCL_PIN 15  // 17       ES8311 I2C 时钟线（SCL）
// #define BSP_CODEC_MCLK_PIN 17 // 8          I2S 主时钟（MCLK），提供给 ES8311 作为参考时钟源
// #define BSP_CODEC_BCLK_PIN 9  // 46          I2S 位时钟（BCLK），每个采样位产生一个时钟沿
// #define BSP_CODEC_DIN_PIN 4   // 15         I2S 数据输入（DIN）：麦克风采集数据流向 ESP32
// #define BSP_CODEC_WS_PIN 5    // 7        I2S 帧同步（WS / LRCK），区分左右声道，单声道时也必须保留
// #define BSP_CODEC_DOUT_PIN 6  // 6         I2S 数据输出（DOUT）：ESP32 播放数据流向 ES8311 → 扬声器

// // ─── 2. 音频采样参数 ─────────────────────────────────────────────────────────
// // 这些参数必须与 AFE（音频前端）和 OPUS 编解码器的配置保持一致

// #define BSP_CODEC_SAMPLE_RATE 16000  ///< 采样率 16kHz（AFE、MultiNet、OPUS 的标准输入要求）
// #define BSP_CODEC_BITS_PER_SAMPLE 16 ///< 采样位深 16-bit（每个采样点占 2 字节）

// // ─── 3. 触摸铜箔引脚 ─────────────────────────────────────────────────────────
// // ESP32-S3 内置电容触摸检测，触摸铜箔直接连接到对应 GPIO
// // 注意：触摸引脚与普通 GPIO 共用，触摸检测期间该引脚不可作普通 GPIO 使用

// #define BSP_TOUCH_1_PIN 1     // 47 触摸铜箔 1（对应 ESP32-S3 TOUCH1 通道）头部
// #define BSP_TOUCH_2_PIN 2     // 1  触摸铜箔 2（对应 ESP32-S3 TOUCH2 通道）背部
// #define BSP_TOUCH_3_PIN 3     // 2  触摸铜箔 3（对应 ESP32-S3 TOUCH7 通道）腹部
// #define BSP_TOUCH_PREV_PIN 7  // 48 翻页前一页触摸（对应 ESP32-S3 TOUCH3 通道）GPIO3 空闲
// #define BSP_TOUCH_NEXT_PIN 14 // 21 翻页后一页触摸（对应 ESP32-S3 TOUCH14 通道）GPIO14 由 LCD RST 迁出

// // ─── 4. ST7789 LCD 屏幕 SPI 引脚 ────────────────────────────────────────────
// // ST7789 使用 SPI 接口，仅支持写入（没有 MISO），时钟可高达 80MHz
// // DC 引脚区分数据（高电平）和命令（低电平）

// #define BSP_LCD_CS_PIN 10   // 41 LCD 片选（CS/NSS），低电平激活
// #define BSP_LCD_MOSI_PIN 11 // 39 LCD 数据线（MOSI），主发从收，单向写
// #define BSP_LCD_SCLK_PIN 12 // 38 LCD 时钟线（SCLK），最高 80MHz
// #define BSP_LCD_DC_PIN 13   // 40 LCD 数据/命令选择（D/C）：高=数据，低=命令
// #define BSP_LCD_RST_PIN 18  // 45 LCD 硬件复位（RST），低电平触发复位（从 GPIO14 迁出，腾出 TOUCH14 给翻页）
// #define BSP_LCD_BK_PIN 48   // 42 LCD 背光控制（BK），高电平开启背光

// #define BSP_LCD_WIDTH 320  ///< LCD 屏幕宽度（像素，横向）
// #define BSP_LCD_HEIGHT 240 ///< LCD 屏幕高度（像素，纵向）

// // ─── 5. 运动与反馈外设引脚 ──────────────────────────────────────────────────
// // 所有运动外设通过 PWM 信号驱动
// // 震动马达：提供触觉反馈（如唤醒、提醒）
// // 舵机（Servo）：控制机器人肢体姿态，范围通常 0~180°

// #define BSP_MOTOR_VIB_PIN 16   // 3 震动马达 PWM 引脚（触觉反馈）
// #define BSP_SERVO_R_ARM_PIN 21 // 4 右臂舵机 PWM 引脚
// #define BSP_SERVO_HEAD_PIN 38  // 9 头部舵机 PWM 引脚
// #define BSP_SERVO_L_ARM_PIN 47 // 14  左臂舵机 PWM 引脚
// // 舵机逻辑通道映射 (供上层调用)
// #define CH_HEAD 0  ///< 头部舵机逻辑通道编号（对应 LEDC_CHANNEL_0，引脚 GPIO38）
// #define CH_L_ARM 1 ///< 左臂舵机逻辑通道编号（对应 LEDC_CHANNEL_1，引脚 GPIO47）
// #define CH_R_ARM 2 ///< 右臂舵机逻辑通道编号（对应 LEDC_CHANNEL_2，引脚 GPIO21）

// // 设备状态标志位 (添加舵机的 BIT)
// #define BOARD_STATUS_SERVO_READY (1 << 3) ///< 舵机已初始化就绪（向 board_status 事件组置位）

// // ─── 6. 舵机速度宏（step_ms：每度等待毫秒数，值越大运动越慢）───────────────────
// // 统一在此定义，供 bsp_servo.c 和 servo_manager.h 共同引用，避免重复定义
// #define SERVO_SPEED_INSTANT 0U    ///< 瞬间到位（无平滑，上电归中禁止使用）
// #define SERVO_SPEED_VERY_FAST 2U  ///< 极快（2ms/度，适合快速抖动动作）
// #define SERVO_SPEED_FAST 5U       ///< 快速（5ms/度，适合挥手、点头等活泼动作）
// #define SERVO_SPEED_MID 15U       ///< 中速（15ms/度，适合大多数情绪动作）
// #define SERVO_SPEED_SLOW 30U      ///< 慢速（30ms/度，适合慵懒、委屈等缓慢动作）
// #define SERVO_SPEED_VERY_SLOW 50U ///< 极慢（50ms/度，适合细腻的情感表达）
// // ─── 7. 外部 Flash 引脚配置 ─────────────────────────────────────────────

// // 使用你确定的 SPI3 引脚
// #define EXT_FLASH_MOSI 42 // 11
// #define EXT_FLASH_MISO 41 // 13
// #define EXT_FLASH_SCLK 39 // 10
// #define EXT_FLASH_CS 40   // 12

// // ─── 8. 电池电压检测（VBAT_ADC）─────────────────────────────────────────────
// // VBAT 经 R23(200kΩ) + R24(200kΩ) 等比分压后接入 ADC 引脚，再经 C43(1uF) 滤波。
// // 分压比 = R24 / (R23 + R24) = 1/2，故 真实电压 = ADC采样电压 × 2
// // 锂电池 3.0V~4.2V → ADC 端 1.5V~2.1V，落在 ADC_ATTEN_DB_12 量程内

// // ⚠️ 引脚说明：
// //   原理图标注 VBAT_ADC 接到 IO5（GPIO5），但当前固件 GPIO5 已被 I2S WS 占用，
// //   故此处先用占位 -1，等硬件 / 引脚分配确定后再填入实际 GPIO。
// //   ESP32-S3 可用 ADC1 通道：GPIO1~GPIO10（ADC2 与 WiFi 冲突，禁止使用）

// #define BSP_BAT_ADC_PIN -1 ///< ⚠️ 占位：电池分压采样 GPIO（待硬件确认后改为实际引脚，必须属于 ADC1）

// // 电池分压网络参数（用于将 ADC 电压换算回真实电池电压）
// #define BSP_BAT_DIVIDER_R_HIGH 200 ///< 分压上臂电阻 R23 (kΩ)
// #define BSP_BAT_DIVIDER_R_LOW 200  ///< 分压下臂电阻 R24 (kΩ)
// // 还原系数 = (R_HIGH + R_LOW) / R_LOW，乘以 ADC 电压即得真实电池电压
// #define BSP_BAT_VOLTAGE_RATIO_NUM (BSP_BAT_DIVIDER_R_HIGH + BSP_BAT_DIVIDER_R_LOW)
// #define BSP_BAT_VOLTAGE_RATIO_DEN (BSP_BAT_DIVIDER_R_LOW)

// // 电池电压阈值（毫伏），用于电量判断与低电告警
// #define BSP_BAT_VOLTAGE_MAX_MV 4200 ///< 锂电池满电电压（4.2V）
// #define BSP_BAT_VOLTAGE_MIN_MV 3000 ///< 锂电池放电截止电压（3.0V，再低会损伤电芯）
// #define BSP_BAT_VOLTAGE_LOW_MV 3300 ///< 低电量告警阈值（3.3V，剩余约 10%）

// // 采样行为参数（用户可按需修改）
// #define BSP_BAT_ADC_SAMPLE_TIMES 16    ///< 单次采集的 ADC 多次平均次数，越大噪声越小
// #define BSP_BAT_TASK_INTERVAL_MS 10000 ///< 后台采样任务周期（毫秒），默认 10s 一次
// #define BSP_BAT_TASK_STACK_SIZE 3072   ///< 后台任务栈大小（字节）
// #define BSP_BAT_TASK_PRIORITY 3        ///< 后台任务优先级（低优先级即可）
// #define BSP_BAT_IIR_ALPHA_PERCENT 30   ///< IIR 低通滤波系数 α（百分比 0~100，越小越平滑）

// #define BSP_OPT_OUT_PIN // 18 外部控制/指示输出
// #define BSP_BOOT_PIN    // 0 系统烧录/启动选择引脚

#pragma once

/* ── 硬件能力开关 ────────────────────────────────────────────────────────────
 * Echo2 正式板（有屏幕）：保持 CONFIG_BSP_HAS_DISPLAY 1
 * 裸开发板（无屏幕）    ：注释掉下面这行，LCD/UI/二维码代码自动剔除
 * ─────────────────────────────────────────────────────────────────────────── */
#define CONFIG_BSP_HAS_DISPLAY 0

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

#define BSP_CODEC_SDA_PIN 16  // 16        ES8311 I2C 数据线（SDA），配置编解码器寄存器
#define BSP_CODEC_SCL_PIN 17  // 17        ES8311 I2C 时钟线（SCL）
#define BSP_CODEC_MCLK_PIN 8  // 8         I2S 主时钟（MCLK），提供给 ES8311 作为参考时钟源
#define BSP_CODEC_BCLK_PIN 46 // 46        I2S 位时钟（BCLK），每个采样位产生一个时钟沿
#define BSP_CODEC_WS_PIN 7    // 7         I2S 帧同步（WS / LRCK），区分左右声道，单声道时也必须保留
#define BSP_CODEC_DIN_PIN 6   // 15        I2S 数据输入（DIN）：麦克风采集数据流向 ESP32
#define BSP_CODEC_DOUT_PIN 15 // 6         I2S 数据输出（DOUT）：ESP32 播放数据流向 ES8311 → 扬声器

// ─── 2. 音频采样参数 ─────────────────────────────────────────────────────────
// 这些参数必须与 AFE（音频前端）和 OPUS 编解码器的配置保持一致

#define BSP_CODEC_SAMPLE_RATE 16000 ///< 采样率 16kHz（AFE、MultiNet、OPUS 的标准输入要求）
// #define BSP_DODEC_BITS_PER_SAMPLE 24000 ///< 采样率 24kHz（AFE、MultiNet、OPUS 的标准输入要求）
#define BSP_CODEC_BITS_PER_SAMPLE 16 ///< 采样位深 16-bit（每个采样点占 2 字节）

// ─── 3. 触摸铜箔引脚 ─────────────────────────────────────────────────────────
// ESP32-S3 内置电容触摸检测，触摸铜箔直接连接到对应 GPIO
// 注意：触摸引脚与普通 GPIO 共用，触摸检测期间该引脚不可作普通 GPIO 使用

#define BSP_TOUCH_1_PIN 47    // 47 触摸铜箔 1（对应 ESP32-S3 TOUCH1 通道）头部
#define BSP_TOUCH_2_PIN 1     // 1  触摸铜箔 2（对应 ESP32-S3 TOUCH2 通道）背部
#define BSP_TOUCH_3_PIN 2     // 2  触摸铜箔 3（对应 ESP32-S3 TOUCH7 通道）腹部
#define BSP_TOUCH_PREV_PIN 48 // 48 翻页前一页触摸（对应 ESP32-S3 TOUCH3 通道）GPIO3 空闲
#define BSP_TOUCH_NEXT_PIN 21 // 21 翻页后一页触摸（对应 ESP32-S3 TOUCH14 通道）GPIO14 由 LCD RST 迁出

// ─── 4. ST7789 LCD 屏幕 SPI 引脚 ────────────────────────────────────────────
// ST7789 使用 SPI 接口，仅支持写入（没有 MISO），时钟可高达 80MHz
// DC 引脚区分数据（高电平）和命令（低电平）

#define BSP_LCD_CS_PIN 41   // 41 LCD 片选（CS/NSS），低电平激活
#define BSP_LCD_MOSI_PIN 39 // 39 LCD 数据线（MOSI），主发从收，单向写
#define BSP_LCD_SCLK_PIN 40 // 40 LCD 时钟线（SCLK），最高 80MHz（与 lcd_demo_standalone 一致：硬件实测此接线）
#define BSP_LCD_DC_PIN 38   // 38 LCD 数据/命令选择（D/C）：高=数据，低=命令（与 lcd_demo_standalone 一致）
#define BSP_LCD_RST_PIN 45  // 45 LCD 硬件复位（RST），低电平触发复位（从 GPIO14 迁出，腾出 TOUCH14 给翻页）
#define BSP_LCD_BK_PIN 42   // 42 LCD 背光控制（BK），高电平开启背光（GPIO42 普通脚，octal PSRAM 占 33-37 不冲突）

// ─── 背光 LEDC PWM 调光配置 ─────────────────────────────────────────────────
// 背光从「GPIO 开关」升级为 LEDC PWM 调光，支持 0~100% 亮度（待机模式需 50%）。
// ★ 资源隔离：舵机已占用 LEDC_TIMER_0 + CHANNEL_0/1/2（见 bsp_servo.c），
//   背光必须使用独立 timer/channel，否则共定时器会导致两边频率打架失灵（见 BUG-015）。
// 这些宏使用 ledc.h 的枚举名，使用方（bsp_lcd.c）须先 #include "driver/ledc.h"。
#define BSP_LCD_BK_LEDC_TIMER LEDC_TIMER_1       ///< 背光独立定时器（避开舵机 TIMER_0）
#define BSP_LCD_BK_LEDC_CHANNEL LEDC_CHANNEL_3   ///< 背光独立通道（避开舵机 CH0/1/2）
#define BSP_LCD_BK_LEDC_MODE LEDC_LOW_SPEED_MODE ///< 低速模式（与舵机同模式）
// ★ 时钟源必须与舵机一致：ESP32-S3 LEDC 同一 speed_mode 下所有 timer 共享时钟源。
//   实测：舵机库（iot_servo）对 50Hz 用 LEDC_AUTO_CLK 会选中 XTAL(40MHz，枚举号 11)。
//   背光若用 APB(号4) 或 RC_FAST(号9) 都会触发 "timer clock conflict ... attempt to 11"
//   致舵机初始化失败（报错 ledc_set_timer_div timer clock conflict）。
//   故背光也强制 XTAL，与舵机统一。分辨率取 10bit：5kHz×2^10=5.12MHz < 40MHz 可产出
//   （13bit 需 40.96MHz > 40MHz XTAL 做不出）。10bit = 1024 级调光，肉眼足够。
#define BSP_LCD_BK_LEDC_CLK LEDC_USE_XTAL_CLK ///< 与舵机统一的时钟源（XTAL 40MHz，枚举号 11）
#define BSP_LCD_BK_LEDC_FREQ_HZ 5000          ///< 背光 PWM 频率 5kHz（无可闻噪声、无屏幕频闪）
#define BSP_LCD_BK_LEDC_RES LEDC_TIMER_12_BIT ///< 12 位分辨率（占空范围 0~8191，与舵机一致）
#define BSP_LCD_BK_DUTY_MAX 4095              ///< 12 位满占空（对应 100% 亮度）
#define BSP_LCD_BK_DEFAULT_PCT 100            ///< 正常点亮亮度（%）
#define BSP_LCD_BK_STANDBY_PCT 10             ///< 待机模式亮度（%）

// ★进入一级低功耗的「过渡总时长」：熄屏亮度渐变（bsp_lcd.c）与手臂归中动态调速
//   （standby.c enter_standby）共用此值，确保手臂归中和屏幕变暗同时完成。
//   两处均引用本宏，不要各自定义独立时长，否则会失去同步意义。
#define BSP_STANDBY_ENTER_TRANSITION_MS 2000U

// 注意：以下 WIDTH/HEIGHT 是 **LVGL 逻辑分辨率（旋转后视角）**，不是 P3 物理分辨率。
// P3 屏物理为 240×320 竖屏，UI 通过 LVGL swap_xy=true 旋转为 320×240 横屏显示。
// 因此 WIDTH=320 HEIGHT=240 = LVGL 画布尺寸，已在 ui_port.c 与之配套。
// 物理面板坐标由 esp_lcd 框架在 draw_bitmap 时按 swap_xy 自动换算。
#define BSP_LCD_WIDTH 320  ///< LVGL 逻辑宽度（横屏旋转后视角）
#define BSP_LCD_HEIGHT 240 ///< LVGL 逻辑高度（横屏旋转后视角）

// ★ 面板自身扫描频率（ST7789 FRCTRL2 / 寄存器 0xC6 的 bits[4:0] RTNA）
//   注意：这是「液晶多久把屏幕扫一遍」，不是 LVGL 的刷新周期
//   (CONFIG_LV_DEF_REFR_PERIOD，那个是「我们多久送一次数据」)。
//   本屏未引出 TE 引脚（[BUG-041]），写 GRAM 与扫描无法同步，撕裂无法根除；
//   但撕裂画面会一直显示到下一遍扫描盖掉，提高扫描频率可缩短其存留时间：
//     60Hz→16.7ms，111Hz→9.0ms，119Hz→8.4ms
//   取值：0x00=119Hz 0x01=111Hz 0x03=99Hz 0x05=90Hz 0x07=82Hz
//         0x09=75Hz  0x0B=69Hz  0x0D=64Hz 0x0F=60Hz(出厂默认) 0x1F=39Hz
//   若出现闪烁/偏色/花屏，按上表往回退档；填 0x0F 即恢复默认行为。
#define BSP_LCD_FRAME_RATE_REG 0x01 ///< 面板扫描频率寄存器值（0x01≈111Hz）

// ─── 5. 运动与反馈外设引脚 ──────────────────────────────────────────────────
// 所有运动外设通过 PWM 信号驱动
// 震动马达：提供触觉反馈（如唤醒、提醒）
// 舵机（Servo）：控制机器人肢体姿态，范围通常 0~180°

#define BSP_MOTOR_VIB_PIN 3 // 3 震动马达 PWM 引脚（触觉反馈）

// ─── 震动马达 LEDC PWM 配置 ─────────────────────────────────────────────────
// 马达从「GPIO 高低电平开关」升级为 LEDC PWM，用占空比调震动强度（震感更强可控）。
// ★ 资源隔离（同 BUG-015 教训）：舵机占用 TIMER_0 + CH0/1/2，背光占用 TIMER_1 + CH3，
//   马达必须用独立的 TIMER_2 + CH4，且时钟源强制 XTAL 与舵机/背光统一——
//   ESP32-S3 同一 speed_mode 下所有 LEDC timer 共享时钟源，混用 APB/AUTO_CLK
//   会触发 "timer clock conflict" 致舵机初始化失败。
// ★ 极性：马达为低有效（OUT=0 通电）。LEDC 输出需反相——idle 输出高电平（断电），
//   震动时占空比越大代表低电平时间越长 → 通电越久 → 震感越强。
#define BSP_MOTOR_LEDC_TIMER LEDC_TIMER_2       ///< 马达独立定时器（避开舵机 T0、背光 T1）
#define BSP_MOTOR_LEDC_CHANNEL LEDC_CHANNEL_4   ///< 马达独立通道（避开舵机 CH0-2、背光 CH3）
#define BSP_MOTOR_LEDC_MODE LEDC_LOW_SPEED_MODE ///< 低速模式（与舵机/背光同模式）
#define BSP_MOTOR_LEDC_CLK LEDC_USE_XTAL_CLK    ///< 时钟源强制 XTAL（40MHz），与舵机/背光统一
#define BSP_MOTOR_LEDC_FREQ_HZ 5000             ///< 马达 PWM 频率 5kHz（同背光，无可闻噪声）
#define BSP_MOTOR_LEDC_RES LEDC_TIMER_10_BIT    ///< 10 位分辨率（占空 0~1023，与背光/舵机一致）
#define BSP_MOTOR_DUTY_MAX 1023                 ///< 10 位满占空
#define BSP_MOTOR_DEFAULT_STRENGTH 100          ///< 触摸反馈默认震动强度（%）

#define BSP_SERVO_R_ARM_PIN 9 // 左臂舵机 PWM 引脚
#define BSP_SERVO_HEAD_PIN 14 // 头部舵机 PWM 引脚
#define BSP_SERVO_L_ARM_PIN 4 // 右臂舵机 PWM 引脚
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
#define EXT_FLASH_MOSI 11 // 11
#define EXT_FLASH_MISO 13 // 13
#define EXT_FLASH_SCLK 10 // 10
#define EXT_FLASH_CS 12   // 12

#define BSP_BAT_ADC_PIN 5  // 5 电池电压检测（分压后接入 ADC）
#define BSP_OPT_OUT_PIN 18 // 18 外部控制/指示输出
#define BSP_BOOT_PIN 0     // 0 系统烧录/启动选择引脚

// 电池分压网络参数（用于将 ADC 电压换算回真实电池电压）
#define BSP_BAT_DIVIDER_R_HIGH 200 ///< 分压上臂电阻 R23 (kΩ)
#define BSP_BAT_DIVIDER_R_LOW 200  ///< 分压下臂电阻 R24 (kΩ)
// 还原系数 = (R_HIGH + R_LOW) / R_LOW，乘以 ADC 电压即得真实电池电压
#define BSP_BAT_VOLTAGE_RATIO_NUM (BSP_BAT_DIVIDER_R_HIGH + BSP_BAT_DIVIDER_R_LOW)
#define BSP_BAT_VOLTAGE_RATIO_DEN (BSP_BAT_DIVIDER_R_LOW)

// 电池电压阈值（毫伏），用于电量判断与低电告警
#define BSP_BAT_VOLTAGE_MAX_MV 4150 ///< 锂电池满电电压（4.15V）
#define BSP_BAT_VOLTAGE_MIN_MV 3250 ///< 锂电池放电截止电压（3.25V，再低会损伤电芯）
#define BSP_BAT_VOLTAGE_LOW_MV 3400 ///< 低电量告警阈值（3.4V，

// 采样行为参数（用户可按需修改）
#define BSP_BAT_ADC_SAMPLE_TIMES 16    ///< 单次采集的 ADC 多次平均次数，越大噪声越小
#define BSP_BAT_TASK_INTERVAL_MS 10000 ///< 后台采样任务周期（毫秒），默认 10s 一次
#define BSP_BAT_TASK_STACK_SIZE 3072   ///< 后台任务栈大小（字节）
#define BSP_BAT_TASK_PRIORITY 3        ///< 后台任务优先级（低优先级即可）

// IIR 滤波与平滑参数（256进制系数，精度更高）
// α=77/256≈30%，α=51/256≈20%，α=128/256=50%
#define BSP_BAT_IIR_ALPHA 77          ///< 电压IIR滤波系数（256进制，77/256≈30%）
#define BSP_BAT_PERCENT_IIR_ALPHA 128 ///< 百分比IIR滤波系数（256进制，128/256=50%）
#define BSP_BAT_MAX_CHANGE_PER_STEP 1 ///< 每次采样周期最大电量变化（%）
#define BSP_BAT_MAX_FAST_CHANGE 5     ///< 差值超过此值时每次变化2%（加快收敛）
#define BSP_BAT_HYSTERESIS_MV 10      ///< 电压滞回阈值（mV），防止百分比来回跳动

// ─── 防回弹 / 抗跳动参数（OCV 还原 + 充电判定）──────────────────────────────
// 锂电池端电压 = 真实开路电压OCV − 负载电流×内阻。舵机/WiFi/音频负载一变，端电压
// 就抖；卸载后电压回弹。下面这组参数用"非对称慢速IIR"还原真实OCV：
//   下降跟得较快（真实掉电要反映），上升压到极慢（把舵机卸载回弹滤掉）。
#define BSP_BAT_OCV_DOWN_ALPHA 51 ///< OCV下降跟随α（256进制，51/256≈20%，较快跟随真实掉电）
#define BSP_BAT_OCV_UP_ALPHA 32   ///< OCV上升跟随α（256进制，32/256≈12.5%，极慢以滤掉回弹）
#define BSP_BAT_CHARGE_RISE_MV 30 ///< 单次OCV上升超过此值（mV）计一次"上升"
#define BSP_BAT_CHARGE_RISE_CNT 3 ///< 连续上升达此次数判定为充电（解锁电量回升）

// ─── 满电区绝对电压旁路（解决满电时百分比被单调递减锁冻结）─────────────────────
// 趋势判定在满电区数学上几乎不可达（OCV上升用慢α，单步涨幅凑不出CHARGE_RISE_MV），
// 故新增"绝对电压"补充信号：OCV高于阈值即认为进入满电平台，此区端电压回弹幅度有限，
// 允许显示电量跟随OCV缓慢回升，而放电工作区仍保持单调递减锁防回弹。
#define BSP_BAT_HIGH_VOLT_UNLOCK_MV 4000 ///< OCV≥此值(mV)进入满电区，放行电量回升（低于满电平台、高于正常放电工作区）
#define BSP_BAT_HIGH_VOLT_RISE_STEP 1    ///< 满电区每采样周期最大回升步进（%），防止一次跳太多虚高

// ─── 低电关机（GPIO18 → HK015T.1 OPT 软关机）─────────────────────────────────
// 硬件：开关机由 HK015T.1 单键自锁芯片（U13）+ K1 长按实现真正断电（静态 1μA）。
// GPIO18(BSP_OPT_OUT_PIN, OPT-OUT) 经 R29 1k 接 HK015T.1 的 OPT(pin6)，R26 100k 下拉。
//
// ★关机时序（屏幕探针实测 T2 确定，2026-07 更正）：
//   经实测，单纯把 GPIO18 拉低【不断电】；单纯拉高也只瞬断自恢复、锁不住。
//   真正让 HK015T 断电的是 **OPT 的"高→低"下降沿**：先把 GPIO18 推挽拉高一小段
//   (BSP_PWR_OFF_PULSE_MS) 建立干净高电平，再推挽拉低保持 → OUTH(pin1) 翻低 →
//   翻转 Q4/Q3 切断主电源（等同用户长按 K1）。重新开机由用户按 K1 冷启动。
//   （旧固件"拉高保持"锁不住电、还把三级关机做成借重启死循环，已废弃。）
// 正常工作期 GPIO18 保持默认输入(hi-Z)，OPT 浮空≈1.44V=开机，只在关机时才驱动它。
//
// 触发策略：锂电池接近 3.3V 已近放空，为避免舵机/扬声器瞬时负载压降误关，要求滤波后
// 的 OCV 连续多次（BSP_BAT_POWEROFF_HIT_CNT）低于阈值才执行关机。
#define BSP_BAT_POWEROFF_ENABLE 1                              ///< 1=启用低电自动关机，0=仅告警不关机（便于调试时关掉）
#define BSP_BAT_POWEROFF_MV 3300                               ///< 低电关机阈值（mV）：OCV≤此值即视为放空，对应 0% 电量
#define BSP_BAT_POWEROFF_HIT_CNT 5                             ///< 连续命中次数：OCV 连续这么多次低于阈值才真正关机，滤掉瞬时尖峰
#define BSP_PWR_OFF_ASSERT_LEVEL 1                             ///< 关机脉冲的“高”电平（先驱动此电平建立干净高）
#define BSP_PWR_OFF_DEASSERT_LEVEL (!BSP_PWR_OFF_ASSERT_LEVEL) ///< 关机脉冲随后回落并保持的“低”电平（下降沿触发 HK015T 断电）
#define BSP_PWR_OFF_PULSE_MS 300                               ///< 关机脉冲高电平保持时长（ms）：建立干净高后再拉低造下降沿
