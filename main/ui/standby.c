/**
 * @file standby.c
 * @brief 无活动待机（省电）模块实现
 *
 * 核心是一个 1s 周期任务 standby_task：
 *   - 每秒读取会话状态，若会话活跃（非 IDLE）则视为活动，刷新时间戳；
 *   - 距上次活动超过 STANDBY_LIGHT_TIMEOUT_MS 且当前未待机 → 进入待机；
 *   - 待机中每隔 SWING_PERIOD_MS 提交一次头部舵机左右摆动动作。
 *
 * 所有共享状态用临界区/原子读写保护，时间戳用 esp_timer_get_time（单调微秒时钟）。
 */
#include "standby.h"
#include "bsp/bsp_board.h"
#include "bsp/servo_manager.h"
#include "wake_word/custom_wake_word.h"
#include "session/session.h"
#include "ui/ui_port.h"
#include "ui/interaction.h" // interaction_set_lowpower()：低功耗头部专属模式开关
#include "esp_lvgl_port.h"  // lvgl_port_lock/unlock：开关屏必须与 LVGL 刷新互斥（见 enter_deep_standby/standby_wake）
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h> // fmaxf/fabsf：手臂归中速度动态计算（enter_standby，与熄屏时长对齐）

static const char *TAG = "STANDBY";

#define STANDBY_LIGHT_TIMEOUT_MS 20000 // 一级
#define STANDBY_DEEP_TIMEOUT_MS 40000
#define STANDBY_SHUTDOWN_TIMEOUT_MS 70000
#define STANDBY_SHUTDOWN_ENABLE 1 ///< 三级开关
// 三级关机 = 二级稳态复查电池后，直接调 bsp_battery_power_off()（GPIO18/OPT 高→低下降沿
// 命令 HK015T 断电，屏幕探针实测有效）。旧的「借重启 + RTC 魔数 + app_main 拉高」实测锁不住
// 电、还造成三级关机死循环，已废弃。
#define STANDBY_PWR_RECHECK_RISE_MV 20 ///< 二级复查：相对进二级基准电压上升≥此值(mV)判为中途插USB→取消三级关机
#define STANDBY_CHECK_MS 1000          ///< 周期任务检查间隔（1 秒）
// ★调试开关：允许"插着USB也进低功耗"。本板无 VBUS 脚，靠电压趋势推断插没插USB，
//   插USB时会判为充电→刷新活动时间→永远进不了低功耗。测低功耗逻辑时身边只有USB供电，
//   把此宏改为 1 即可屏蔽"插USB拦截"（standby_task 1b 的 touch_activity 放行），让插着USB
//   也能正常进一/二/三级待机。★验证完务必改回 0，否则真机插USB充电时会误进待机甚至关机。
#define STANDBY_DEBUG_ALLOW_USB_SLEEP 1 ///< 1=插USB也进低功耗（仅调试用）；0=正常拦截

// ─── 待机期「插USB→唤醒」检测（goal 2，本板无 VBUS 脚，靠电压相对基准回升推断）──────
// 进入待机后负载骤降会让端电压回弹上升，故先等 settle 再捕获基准，且要求相对基准
// 连续多次明显上升才判为插USB，避免把回弹误判成充电。命中即 standby_wake() 回空闲。
#define STANDBY_USB_SETTLE_MS 5000 ///< 进一级待机后等此时长让回弹稳定，再捕获充电检测基准
#define STANDBY_USB_RISE_MV 40     ///< 待机期电压相对基准上升≥此值(mV)记一次「疑似插USB」
#define STANDBY_USB_RISE_CNT 3     ///< 连续命中次数达此值判为插USB→唤醒（约数秒，滤回弹）
// ★快通道（档2）：插USB是「陡升」——实测一插端电压猛涨~190mV，远超进待机回弹（几十mV）。
//   故相对「进待机瞬间电压 s_standby_enter_mv」单次猛涨≥JUMP_MV 即刻判插USB→立即唤醒，
//   不等 settle、不用连续，延迟压到一个采样拍(~1s)，达到近似手机的「插入即醒」。
//   阈值须设在回弹上限之上、真插USB之下；建议用日志实测回弹幅度后再微调此值。
#define STANDBY_USB_JUMP_MV 120 ///< 相对进待机电压单次上升≥此值(mV)即刻判插USB（快通道）
// ★拔USB检测（未待机时，1b 用）：相对跟踪基准连续下降才清「插USB」锁存，滤单次波动/负载尖峰。
#define STANDBY_USB_DROP_MV 50            ///< 电压相对跟踪基准下降≥此值(mV)记一次「疑似掉电」
#define STANDBY_USB_DROP_CNT 3            ///< 连续掉电命中判为拔USB→清锁存，允许重新进低功耗
#define STANDBY_SWING_ENABLE 1            ///< ★一级待机摆头总开关（1=每2.5s摆头；0=不摆头，舵机静止）
#define SWING_PERIOD_MS 2500              ///< 待机摆头周期（每 2.5s 换一次方向）
#define HEAD_CENTER_DEG 90.0f             ///< 头部中位角度
#define HEAD_SWING_DEG 30.0f              ///< ★摆动幅度（±x°，即 (90-x)°~(90+x)°，后续直接改这里）
#define SWING_SPEED SERVO_SPEED_VERY_SLOW ///< ★摆头速度（取最慢档 50ms/度，后续可换 SLOW 等档位）

#define STANDBY_DEEP_ENABLE 1 ///< 二级（A 层：关屏+停舵机）总开关（1=启用）

// ★二级待机是否保留语音唤醒（编译期开关）：
//   1 = 二级仍开着唤醒词监听，可「喊一声」唤醒（体验好，但 MultiNet 持续占 CPU、功耗偏高，
//       参见历史 BUG-006 唤醒词 CPU 占满）；
//   0 = 二级关闭唤醒词监听，只能触摸唤醒（更省电，是本 A 层方案的推荐值）。
//   注意：本开关仅在「A 层熄屏（CPU 不睡）」方案下有意义；若将来升级到 light/deep sleep，
//        CPU 睡眠时无法跑 MultiNet 推理，语音唤醒物理上不可用，本开关自然失效。
#define STANDBY_DEEP_KEEP_WAKE_WORD 0 ///< 1=二级保留语音唤醒；0=二级仅触摸唤醒（省电，默认）

// ─── 模块状态 ───────────────────────────────────────────────────────────────
// s_last_active_us：上次活动时间戳（微秒，esp_timer 单调时钟）
// s_standby：当前是否待机。均用 volatile + 简单读写，配合 64 位读取注意原子性。
static volatile int64_t s_last_active_us = 0;
static volatile bool s_standby = false;      // 是否处于一级（轻度）待机
static volatile bool s_deep_standby = false; // 是否已触发二级（深度）待机（占位，防重复触发）
static volatile bool s_inited = false;
static volatile bool s_shutdown_standby = false; // 是否已触发三级（关机）
static volatile uint32_t s_deep_base_mv = 0;     // 进二级时(负载已恒定至最小)的基准电压 mV，供三级复查比对
// ─── 待机期插USB检测状态（goal 2）───
static volatile int64_t s_standby_enter_us = 0;  // 进入一级待机的时间戳，用于 settle 后再取基准
static volatile uint32_t s_standby_enter_mv = 0; // 进入一级待机瞬间的端电压 mV，供快通道判「陡升插USB」
static volatile uint32_t s_charge_base_mv = 0;   // 充电检测基准电压 mV（0=尚未捕获，待 settle 后捕获）
static volatile int s_charge_rise_hit = 0;       // 电压相对基准连续明显上升的命中计数
// ─── 插USB锁存 + 趋势跟踪（未待机时防再入睡 / 判拔出，见 standby_task 步骤 1b）───
static volatile bool s_usb_present = false; // 锁存：当前是否判定插着USB（true→不进低功耗）
static volatile uint32_t s_usb_base_mv = 0; // 上升/下降趋势跟踪基准 mV（0=尚未建立）
static volatile int s_usb_drop_hit = 0;     // 相对基准连续下降命中计数（滤单次波动）
// ─── 一级退出「归中 gate」状态（D.4）：flush 后异步等归中完成再切 GIF，见 standby_wake ───
// 亮度/音量渐变当前用阻塞版（bsp_board_lcd_fade_brightness/bsp_board_codec_exit_lowpower），
// 直接在 standby_wake() 里跑完再返回，不需要标志位；归中动作是异步的（worker 执行），
// 故仍需下面这对标志等 GATE_MS 后再恢复 GIF 循环。非阻塞渐变版（fade_step 单步 API）
// 已在 bsp_lcd.c/bsp_codec.c 实现，如后续要切换为非阻塞可复用，无需改动本段架构。
static volatile bool s_lp_gif_pending = false;   // true=已发起归中，等待 GATE_MS 后切回正常GIF循环
static volatile int64_t s_lp_gif_pending_us = 0; // 发起归中（flush）的时间戳
#define STANDBY_LP_GATE_MS 400                   // 归中等待时长：与 enter_deep_standby 的 flush 等待手法一致（够 worker 归中完成）

/**
 * @brief 刷新「上次活动」时间戳（内部）
 */
static inline void touch_activity(void)
{
    s_last_active_us = esp_timer_get_time();
}

void standby_notify_activity(void)
{
    // 任意活动统一收口到这里：刷新活动时间，且若正处于待机则一并退出待机。
    // standby_wake() 内部先 touch_activity() 再判断 !s_standby 直接返回，
    // 所以非待机时这里等价于「只刷计时」，待机时则完整恢复（亮度/监听/头部归中）。
    // 这样头部/腹背/左右翻页（耳朵）任意触摸、以及对话会话激活，都能退出待机。
    standby_wake();
}

bool standby_is_active(void)
{
    return s_standby;
}

bool standby_is_deep_active(void)
{
    return s_deep_standby;
}

/**
 * @brief 进入待机：降亮度 + 关监听 + 置标志（摆头由周期任务负责）
 */
static void enter_standby(void)
{
    if (s_standby)
        return;
    ESP_LOGI(TAG, "无活动超过 %d ms，进入待机", STANDBY_LIGHT_TIMEOUT_MS);
    s_standby = true;
    // 充电检测：记进入待机时刻，基准置 0（等 settle 后由 standby_task 捕获），命中清零
    s_standby_enter_us = esp_timer_get_time();
    // ★快通道基准 s_standby_enter_mv：改为在关负载动作「之后」捕获（见本函数末尾）。
    //   旧实现在此处取「负载还在、尚未回弹」的瞬时最低点做锚，前提是「回弹幅度 < JUMP_MV」；
    //   但满电电池内阻高，关屏/停舵机/停语音后回弹可破 120mV，会被 2d 快通道误判为插USB
    //   而自激唤醒（自己退出低功耗的根因）。改为在负载卸掉、电压已回弹到位后再取锚，锚点
    //   本身已含回弹，之后只有真插USB才会再明显抬高，从根上消除回弹误触发。此处先置0占位。
    s_standby_enter_mv = 0;
    s_charge_base_mv = 0;
    s_charge_rise_hit = 0;
    /* 进入待机前，把界面强制退回主界面：避免「已降亮度但仍卡在某菜单/闹钟编辑页」。
     * 该函数内部自带 LVGL 锁，跨任务调用安全；已在主界面则直接返回。 */
    ui_force_back_to_main();
    wake_word_stop();                                          // 关闭唤醒词监听（仅切 is_running）
    ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // ★新增：立刻停止震动马达 PWM，防止待机期间误触发高功耗

    // ★步骤1（顺序修正）：低功耗标志与清队【必须最先做】，早于 flush + 手臂归中。
    //   - interaction_set_lowpower(true) 前移：原实现放在末尾（距 flush 约 1.3s），这段窗口内
    //     worker 若已开始执行存量情绪，lowpower 仍为 false → 手臂照常摆（问题5「进低功耗手臂不立即
    //     归中、还运行一段」的根因之一）。前移后，之后 worker 取出的任何情绪都按低功耗（手臂钉90）执行。
    //   - interaction_flush_queue()：清掉 ia_queue 里【未执行】的存量情绪（一级待机 GIF 定时器持续入队），
    //     否则它们会排在归中之后被 worker 执行，把手臂/头又带跑（问题1a/5 根因）。
    interaction_set_lowpower(true);
    interaction_flush_queue();

    // ★步骤2：flush 打断 servo_manager 里正在执行的舵机动作 + 清 servo_manager 队列，并清打断标志。
    //   （interaction_flush_queue 清的是 interaction 层队列；这里清的是 servo_manager 层队列，两层都要清。）
    servo_manager_flush();

    // ★背光/音量渐变提前到手臂归中之前（修复「偶发进一级但屏幕不降亮度」）：
    //   旧实现把这两行放在手臂同步归中之后。手臂同步归中调用 bsp_servo_move_smooth 会
    //   xSemaphoreTake(该通道 mutex, portMAX_DELAY) 永久等待——而一级待机的随机情绪头部
    //   动作用 bsp_servo_move_all_parallel 一次性抢三轴 mutex（含左右臂）并持有整个插值
    //   过程，即使手臂目标是 90°不动、只有头部在动。若情绪循环持续排队，enter_standby
    //   可能要等相当久才能抢到臂锁，期间背光渐变代码根本没执行到——表现为「s_standby
    //   已 true、头部在正常播随机情绪，但屏幕迟迟不变暗」（已通过读代码定位，未烧录实测）。
    //   背光/音量渐变本身与舵机无关，提前到此处执行不受该锁影响，让降亮度成为进待机
    //   最先对用户可见的反馈。手臂归中逻辑本次不动，仍保持原同步实现顺序执行。
    bsp_board_lcd_fade_brightness(BSP_LCD_BK_DEFAULT_PCT, BSP_LCD_BK_STANDBY_PCT);
    bsp_board_codec_enter_lowpower(); // ★音量线性渐变压低（不写NVS），防低功耗期离线音频/提示音过大

    // ★步骤3：手臂归中改为【同步】bsp_servo_move_smooth，不再走 servo_manager 队列入队。
    //   原实现用 servo_manager_submit_angle 入队两条归中请求，但 interaction_play_blocking 收尾会
    //   再 flush 一次（interaction.c）——若那次 flush 发生在这两条归中之后，会把归中请求一起清掉，
    //   表现为「手臂没归中就停在情绪臂位」（问题5）。改为在本任务上下文同步归中：ia_queue 已清、
    //   servo_manager 已 flush，per-channel mutex 无长期竞争，同步调用会稳稳把手臂带回 90° 且不会被清掉。
    //   头部不在此归中——随后由 interaction 低功耗模式接管，播随机情绪的头部动作。
    bsp_servo_move_smooth(CH_L_ARM, HEAD_CENTER_DEG, SERVO_SPEED_FAST); // 左臂同步归中 90°（快速 5ms/度，进低功耗手臂尽快归位）
    bsp_servo_move_smooth(CH_R_ARM, HEAD_CENTER_DEG, SERVO_SPEED_FAST); // 右臂同步归中 90°（快速）

    // ★快通道锚点：负载已全部卸掉（关屏渐暗/停语音/停马达/停舵机/压音量都已完成），
    //   此刻电压已回弹到位，以此稳定值为锚。之后 2d 快通道相对它涨≥JUMP_MV 才判插USB，
    //   回弹已包含在锚里、不再计入，从根上消除「进待机后回弹自激唤醒」。
    s_standby_enter_mv = bsp_battery_read_voltage_mv();
}

/**
 * @brief 进入二级（深度）待机：彻底关屏 + 停舵机 PWM（A 层省电，CPU 不睡）
 *
 * 在一级待机基础上进一步省电：
 *   1. 背光线性渐暗到0 + bsp_board_lcd_disp_off() 关显示控制器（比一级仅降亮度更彻底，
 *      且与一级/退出的渐变手感一致，不再是瞬间跳变）；
 *   2. bsp_servo_idle()     —— 停三路舵机 PWM 输出，舵机失力，静态电流下降；
 *      （同时让周期任务停止摆头：见 standby_task 中 s_deep_standby 判断）
 *
 * ★不进入任何 sleep：CPU 全速运行，reminder 的闹钟轮询/倒计时照常工作（见本文件
 *   顶部宏区说明）。退出由摸头/任意触摸/会话激活触发，无需配置 GPIO 唤醒源。
 *
 * 幂等：已处于二级则直接返回，避免重复关屏/停舵机。
 */
static void enter_deep_standby(void)
{
#if STANDBY_DEEP_ENABLE
    if (s_deep_standby)
        return;
    s_deep_standby = true;
    ESP_LOGW(TAG, "无活动超过 %d ms，进入二级待机（关屏 + 停舵机，CPU 不睡，闹钟照常）",
             STANDBY_DEEP_TIMEOUT_MS);

    // 1) 关屏：先背光线性渐暗到 0（与一级 enter_standby 手感一致，不再是瞬间跳变），
    //    再只关显示控制器（bsp_board_lcd_disp_off，背光已是0不会有二次跳变）。
    //    起点用 BSP_LCD_BK_STANDBY_PCT（一级待机亮度）而非 DEFAULT_PCT：进二级前必经
    //    一级，此刻背光已是 STANDBY_PCT，若从 DEFAULT_PCT 起渐变会先跳变一下再暗下去。
    //    背光渐变本身不碰 SPI/LVGL，无需持锁；只有「关显示控制器」这步走 SPI 发 DISPOFF，
    //    与 LVGL 刷新任务共用同一 SPI panel 句柄，必须持锁再调（同一级约定）。
    bsp_board_lcd_fade_brightness(BSP_LCD_BK_STANDBY_PCT, 0);
    if (lvgl_port_lock(1000))
    {
        // 【诊断-DBG1】关显示控制器前打印，确认此刻是否真的走到这一步（验证OK后可删）
        ESP_LOGW(TAG, "[DBG1] 即将 disp_off，关屏前时间戳=%lld us", esp_timer_get_time());
        bsp_board_lcd_disp_off(bsp_board_get_instance());
        lvgl_port_unlock();
    }
    else
    {
        ESP_LOGE(TAG, "进二级取 LVGL 锁超时，跳过关显示控制器（背光已渐暗，仅停舵机）");
    }

    // 2) ★先清空【两层】队列 + 打断正在执行的动作，再停 PWM —— 否则会“停了又被点亮”：
    //    一级待机期间 main_gif_switch_timer_cb 持续往 interaction 队列(ia_queue)塞随机情绪，
    //    进二级那一刻 ia_queue 里可能还有排队情绪，或 servo_manager 正卡在插值循环里。
    //    若不先清 ia_queue 就 bsp_servo_idle()，worker 随后取出存量情绪 → iot_servo_write_angle
    //    重新 set_duty 点亮刚被 ledc_stop 的 PWM，表现为“二级了舵机（头部）还在动”（问题1a 根因）。
    //    - interaction_flush_queue()：清 interaction 层未执行的存量情绪请求；
    //    - servo_manager_flush()：清 servo_manager 层队列 + 打断正在执行的插值动作。
    interaction_flush_queue(); // 清空队列请求
    servo_manager_flush();     // 清空队列、打断正在执行的插值动作
    // 3) flush 是非阻塞的：正在执行的插值步要几十 ms 才在循环边界真停下，worker 收尾还会归中，
    //    且 interaction worker 可能正卡在情绪播放中（等 servo done 信号量）。
    //    ★根因修复（偶发「进/退二级瞬间抖一下」）：旧实现在 interaction_is_playing()==false
    //    后只固定再等 200ms 就认为 servo_manager 已归中完毕，但归中动作（bsp_servo_move_all_
    //    parallel 90/90/90）是在 servo_manager worker 里异步执行的，耗时取决于之前的角度偏移，
    //    200ms 不一定够——一旦没等够就调下面的 bsp_servo_idle()（ledc_stop），会和 worker 仍在
    //    写 iot_servo_write_angle 撞车，两者竞争同一 LEDC 通道，表现为偶发的瞬间抖动。
    //    改为轮询 servo_manager_is_idle()（队列空+worker无请求在执行，含归中）为 true，
    //    直接反映真实执行状态，不再依赖固定延时猜测。
    {
        int guard = 0;
        while ((interaction_is_playing() || !servo_manager_is_idle()) && guard < 60) // 60*50ms = 3s 上限兜底
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            guard++;
        }
    }

    // 4) 停舵机 PWM：只停舵机三路通道，不动共享 timer，不影响背光/马达（见 bsp_servo_idle）。
    bsp_servo_idle();

    // 4b) ★捕获二级基准电压：此刻屏已关、舵机已停，负载恒定到最小，端电压接近真实 OCV，
    //     "进一级判一次并锁定"的决策到这里已确认在用电池。以此为基准，供三级关机前复查
    //     「二级期间电压是否明显上升」——上升即中途插了 USB，取消断电（见 enter_shutdown）。
    //     用同步即时读取（多次采样平均，不受后台慢速 IIR 滞后影响），反映当前稳态端电压。
    s_deep_base_mv = bsp_battery_read_voltage_mv();
    ESP_LOGI(TAG, "二级基准电压=%lu mV（供三级复查是否中途插USB）", (unsigned long)s_deep_base_mv);
    // 二级负载更恒定，用它作为更干净的充电检测基准，覆盖一级基准并清零命中（避免一级→二级
    // 二次回弹被误判成上升）。
    if (s_deep_base_mv > 0)
    {
        s_charge_base_mv = s_deep_base_mv;
        s_charge_rise_hit = 0;
    }

    // 5) 语音唤醒（编译期开关 STANDBY_DEEP_KEEP_WAKE_WORD，见宏区说明）：
    //    一级待机已 wake_word_stop()。若本开关=1，则二级把唤醒词重新打开，使二级期间
    //    也能「喊一声」唤醒（代价：MultiNet 持续占 CPU、功耗偏高）；=0 则维持关闭（更省电）。
#if STANDBY_DEEP_KEEP_WAKE_WORD
    wake_word_start();
    ESP_LOGI(TAG, "二级保留语音唤醒（STANDBY_DEEP_KEEP_WAKE_WORD=1）");
#endif
#else
    // 占位：仅打印，便于先验证「二级超时被正确检测到」这条链路。
    ESP_LOGW(TAG, "[占位] 已达二级待机阈值 %d ms，但二级未启用（STANDBY_DEEP_ENABLE=0）",
             STANDBY_DEEP_TIMEOUT_MS);
#endif
}
/**
 * @brief 进入三级（纯关机）：向电源管理 IC 发送断电指令，系统彻底停止
 *
 * 幂等：已触发则直接返回。执行断电指令后 CPU 停止，无唤醒逻辑，
 * 只能通过物理按键重新上电开机。
 */
static void enter_shutdown(void)
{
#if STANDBY_SHUTDOWN_ENABLE
    if (s_shutdown_standby)
        return;

    // 0. 断电前最后一道复查「是否待机中途插了 USB」──────────────────────────────
    //    正常情况 standby_task 的 2d 充电检测会在到三级前就唤醒；这里作为断电前的兜底：
    //    二级期间屏已关、舵机已停，负载恒定，端电压趋势最干净，用「相对进二级基准电压
    //    s_deep_base_mv 是否明显上升」复查。明显上升 → 判为插了 USB（在充电，断了会被 USB
    //    拉起重启）→ 直接 standby_wake() 退出待机回空闲（goal 2），绝不真断电。
    uint32_t v_now = bsp_battery_read_voltage_mv();
    if (v_now > 0 && s_deep_base_mv > 0 &&
        (int)v_now - (int)s_deep_base_mv >= STANDBY_PWR_RECHECK_RISE_MV)
    {
        ESP_LOGW(TAG, "断电前复查电压上升(%lu→%lu mV，阈值+%d)，判定插USB，取消关机并唤醒回空闲",
                 (unsigned long)s_deep_base_mv, (unsigned long)v_now, STANDBY_PWR_RECHECK_RISE_MV);
        standby_wake(); // 退出待机回空闲（goal 2）
        return;
    }

    s_shutdown_standby = true;
    ESP_LOGW(TAG, "无活动超过 %d ms 且复查确认在用电池，执行三级关机（GPIO 下降沿断电）",
             STANDBY_SHUTDOWN_TIMEOUT_MS);

    // 1. 确保二级待机（关屏、停舵机）已执行，避免断电瞬间屏幕花屏或舵机抖动
    enter_deep_standby();

    // 2. ★真断电：GPIO18(OPT) 高→低下降沿命令 HK015T 切断主电源（屏幕探针实测 T2 有效，
    //    运行态直接生效、不返回）。旧的"借重启拉高"实测锁不住电、还造成三级关机死循环，已废弃。
    //    电量已由电池监控任务节流存档，无需再存。
    bsp_battery_power_off();
#else
    ESP_LOGW(TAG, "[占位] 已达三级关机阈值，但关机未启用");
#endif
}

/**
 * @brief 恢复震动马达 LEDC 通道（退出低功耗时用，一级/二级路径共用）
 *
 * enter_standby 用 ledc_stop 停了马达 PWM，退出时须重新 config 通道，
 * 初始 duty=MAX（恒高电平 → 低有效断电，即停止态），后续震动才能正常驱动。
 * 一级、二级两条唤醒路径都要调（原实现只有一级分支恢复，二级唤醒漏了 → 问题2）。
 */
static void restore_motor_pwm(void)
{
    ledc_channel_config_t motor_channel = {
        .speed_mode = BSP_MOTOR_LEDC_MODE,
        .channel = BSP_MOTOR_LEDC_CHANNEL,
        .timer_sel = BSP_MOTOR_LEDC_TIMER,
        .gpio_num = BSP_MOTOR_VIB_PIN,
        .duty = BSP_MOTOR_DUTY_MAX, // 初始停止（恒高电平 → 低有效断电）
        .hpoint = 0,
    };
    ledc_channel_config(&motor_channel);
}

void standby_wake(void)
{
    // 幂等：先刷新活动时间，避免唤醒后立刻又超时
    touch_activity();
    if (!s_standby)
        return;
    ESP_LOGI(TAG, "退出待机，恢复正常状态");
    // 【诊断-DBG0】打印此刻 s_deep_standby 的真实值，确认走一级还是二级恢复分支（验证OK后可删）
    ESP_LOGW(TAG, "[DBG0] standby_wake 入口 s_deep_standby=%d", (int)s_deep_standby);

    // ★根因修复（进/退二级瞬间三舵机快速抖一下，几十次偶发一次）：
    //   旧实现在这里就把 s_standby 清 false，但下面 ui_resume_main_gif_loop() 会
    //   lv_timer_resume 唤醒 GIF 切换定时器（周期仅 10ms，见 ui_port.c main_gif_create），
    //   几乎必然在 bsp_servo_resume() 真正执行（在本函数二级分支末尾）之前就被触发一次。
    //   该定时器回调 main_gif_switch_timer_cb 里靠 standby_is_active()/standby_is_deep_active()
    //   判断「是否还在二级、要不要丢弃这次切图」——若 s_standby 已提前清零，会被误判为已回到
    //   正常空闲态，从而投递一条新的空闲动作（含三轴绝对角度舵机请求），与还没跑完的
    //   bsp_servo_resume() 归中同时抢舵机，两条指令先后覆盖，表现为一次可见的瞬间抖动。
    //   改为把 s_standby/s_deep_standby 的清零推迟到本函数二级恢复流程（含 bsp_servo_resume）
    //   全部跑完之后，恢复期间 standby_is_active()/is_deep_active() 仍如实返回「还在二级」，
    //   GIF 定时器回调据此正确丢弃抢跑的切图请求，不再投递多余的舵机动作。
    //   （一级恢复分支同理，见本函数末尾一并处理。）

    // ── 若之前已进入二级（关屏+停舵机），先恢复二级再恢复一级 ──────────────────
    // 顺序：开显示控制器（背光仍是0，不可见）→ 背光线性渐亮到 100%（与 enter_deep_standby
    // 进二级的渐暗对称，不再是 bsp_board_lcd_on 那种直接跳变到 100%）→ 舵机恢复。
    if (s_deep_standby)
    {
        // ★开显示控制器必须持 LVGL 锁再调：走 SPI 给 ST7789 发 DISPON，与 LVGL 刷新任务
        //   共用同一 SPI panel 句柄。不加锁会与刷新事务抢 SPI，导致
        //   esp_lcd_panel_disp_on_off 失败或永久阻塞——这正是「打印了退出待机但屏仍黑、
        //   触摸任务卡死、后续触摸全无响应」的根因（与 application.c 开机开屏先加锁的约定一致）。
        //
        // ★★问题A 修复（偶发"退出低功耗但屏黑、舵机/震动却在动"）：开屏必须先于所有恢复动作，
        //   且【开屏失败绝不半恢复】。原实现取锁失败只打日志就继续跑 servo_resume/背光/震动，
        //   于是出现"屏没开成但舵机在动"。现改为：取锁重试 2 次（各 1000ms）；仍失败则【整段回滚】——
        //   保持 s_standby/s_deep_standby 为 true（不清），直接 return，不碰舵机/背光/震动，
        //   下次触摸活动会重新进入本函数整段重来。宁可这次不醒，也不半醒。
        bool disp_on_ok = false;
        for (int attempt = 0; attempt < 2 && !disp_on_ok; attempt++)
        {
            if (lvgl_port_lock(1000))
            {
                bsp_board_lcd_disp_on(bsp_board_get_instance()); // 只开显示控制器，背光仍是0
                // 【诊断-DBG2】disp_on 成功后，尝试调用前打印将要恢复轮播的现场信息（验证OK后可删）
                ESP_LOGW(TAG, "[DBG2] disp_on 成功(attempt=%d)，即将调 ui_resume_main_gif_loop，时间戳=%lld us",
                         attempt, esp_timer_get_time());
                // ★恢复 GIF 自动轮播：待机时 main_gif_ready_cb 停了自循环、GIF 定格在最后一帧
                //   并被 LVGL pause（不再触发 ready_cb），故醒来必须显式重启轮播，否则屏幕卡在
                //   一张静止 GIF 不再切换。ui_resume_main_gif_loop 内部走 lv_timer_resume，
                //   是 LVGL 操作，必须在本锁内调用。
                ui_resume_main_gif_loop();
                lvgl_port_unlock();
                disp_on_ok = true;
            }
        }
        if (!disp_on_ok)
        {
            // 两次都没取到锁：整段放弃本次唤醒，标志【保持不变】（仍是二级待机），不做任何
            // 舵机/背光/震动恢复，避免"屏黑但舵机动"的半恢复态。下次触摸/活动会重试。
            ESP_LOGE(TAG, "[DBG2-FAIL] 退二级两次取 LVGL 锁均超时，放弃本次唤醒（保持待机，下次活动重试）");
            return;
        }

        // 开屏成功，确认退出二级。此后各步都是幂等/可安全执行的恢复动作。
        // ★s_deep_standby 不在此处清零（原实现在此清），改到 bsp_servo_resume() 归中完成之后
        // 才清（连同 s_standby 一起），原因见本函数开头的根因说明：本函数马上要调用的
        // ui_resume_main_gif_loop() 会在 10ms 内唤醒 GIF 切换定时器，其回调靠
        // standby_is_deep_active() 判断是否要丢弃抢跑的切图请求——若这里提前清零，
        // 会被误判为已经退出二级，从而投递多余的舵机动作与 bsp_servo_resume() 抢舵机。

        // 背光线性渐亮 0→100%：屏幕先亮起来（与进二级的渐暗对称）。放在 servo_resume 之前，
        // 保证"屏幕优先亮"——servo_resume 现为错峰归中，阻塞约 1~2s，不能让它挤在开屏前面。
        bsp_board_lcd_fade_brightness(0, BSP_LCD_BK_DEFAULT_PCT);

        // ★补全「一级状态」恢复（原实现二级分支漏了这三项，是问题2「退二级无预期动作」的根因）：
        //   二级是在一级基础上叠加的，enter_standby 置的低功耗标志/停的马达/压低的音量在进二级时
        //   并未撤销，故从二级唤醒必须把它们一并恢复，否则：
        //   - lowpower 标志不清 → interaction 情绪播放手臂恒钉 90°、震动恒跳过（手臂对触摸不动）；
        //   - 马达通道不恢复 → 之后所有震动无输出；
        //   - 音量不恢复 → 停在低功耗压低值。
        //   ★顺序：先清 lowpower 标志/清队/恢复马达/音量，再 servo_resume 归中——这样 resume 归中
        //     期间标志已是"正常态"，不会再有存量情绪把刚归中的舵机带跑。
        interaction_flush_queue();       // 清 interaction 存量情绪，防 servo_resume 归中后被残留请求带跑
        interaction_set_lowpower(false); // 清低功耗标志：情绪播放恢复手臂+震动正常参与
        restore_motor_pwm();             // 恢复震动马达 LEDC 通道
        bsp_board_codec_exit_lowpower(); // 音量线性渐变恢复为用户设置值（阻塞，读 NVS）

        bsp_servo_resume(); // 恢复三路 PWM 并错峰归中（含头部，见 bsp_servo.c）；放最后，屏已亮

        // ★归中已彻底完成，此刻才清零两个标志：此前若 GIF 定时器抢跑触发过
        // main_gif_switch_timer_cb，都会因 standby_is_deep_active()==true 被正确丢弃，
        // 不会再有额外的舵机动作与上面的 bsp_servo_resume() 竞争，抖动窗口从根上消除。
        s_deep_standby = false;
        s_standby = false;

        // 退出待机统一重开唤醒词监听（无论二级期间是否在监听，恢复后都要可用）。
        // 幂等：wake_word_start 内部若已在运行则无副作用。
        wake_word_start();
        return; // 二级路径已完成全部恢复，不再走一级分支
    }

    // ── 仅一级待机的恢复 ──────────────────────────────────────────────────────
    // ★当前用阻塞版渐变（简单直接）：standby_wake() 被触摸/MQTT/唤醒词回调调用时
    //   会阻塞约 BSP_LCD_BK_FADE_MS(800ms)+BSP_CODEC_LOWPOWER_FADE_MS(500ms)。
    //   非阻塞版（fade_step 单步 API，见 bsp_board_lcd_fade_step/bsp_board_codec_fade_step）
    //   已实现在 bsp_lcd.c/bsp_codec.c，效果验证后如需切换为非阻塞，只需在此改调用
    //   + 补一对轮询标志（仿 s_lp_gif_pending 手法），无需改动整体架构。
    interaction_flush_queue();                                                     // 清 interaction 存量情绪，防归中后被残留请求带跑
    interaction_set_lowpower(false);                                               // 清低功耗标志：情绪播放恢复手臂+震动正常参与
    bsp_board_codec_exit_lowpower();                                               // 音量线性渐变恢复为用户设置值（阻塞，读 NVS）
    bsp_board_lcd_fade_brightness(BSP_LCD_BK_STANDBY_PCT, BSP_LCD_BK_DEFAULT_PCT); // 背光线性渐变恢复100%（阻塞）
    restore_motor_pwm();                                                           // 恢复震动马达 LEDC 通道（与二级路径共用）
    wake_word_start();                                                             // 重新开启唤醒词监听

    // ★退出低功耗归中：先 flush 清队+打断当前情绪动作，但 flush 只是异步信号——
    //   worker 收尾归中前，队列里若已有下一条情绪请求（一级待机期间自动循环持续
    //   提交），会被当成干净的新请求正常执行，头部会继续跑该情绪动作而不停下
    //   （这正是"退出低功耗后头部仍在跑情绪队列"的根因）。故这里额外同步调用
    //   bsp_servo_move_all_parallel 直接把三轴（含头部）带回 90°，固定用中性速度
    //   SERVO_SPEED_MID，确保退出低功耗后头部立即归中，不受队列后续请求影响
    //   （与 bsp_servo_resume() 二级退出的确定性归中语义一致）。
    servo_manager_flush();
    bsp_servo_move_all_parallel(90.0f, 90.0f, 90.0f, SERVO_SPEED_MID);

    // ★与二级分支同一根因处理：舵机同步归中已彻底完成，此刻才清 s_standby。
    //   一级分支的 GIF 循环恢复走 s_lp_gif_pending + STANDBY_LP_GATE_MS 延后机制（非立即
    //   resume），抢跑窗口本就比二级小，但为了 standby_is_active() 在归中完成前如实反映
    //   状态、避免任何路径误判为已空闲而抢投舵机动作，仍统一推迟到此处清零。
    s_standby = false;

    s_lp_gif_pending = true;
    s_lp_gif_pending_us = esp_timer_get_time();
}

/**
 * @brief 待机监测周期任务
 *
 * 职责：
 *   1. 每秒检测会话是否活跃（session_get_state != IDLE），活跃则刷新活动时间，
 *      天然覆盖「正在对话 / TTS 播放」场景，避免误进待机。
 *   2. 非待机且空闲超时 → enter_standby。
 *   3. 待机期间按 SWING_PERIOD_MS 节奏提交头部左右摆动。
 */
static void standby_task(void *arg)
{
    bool swing_left = false;   // 下一次摆动方向（左/右交替）
    int64_t last_swing_us = 0; // 上次提交摆头的时间戳

    ESP_LOGI(TAG, "待机监测任务启动（超时 %d ms）", STANDBY_LIGHT_TIMEOUT_MS);
    touch_activity(); // 初始化活动时间，从启动开始计时

    while (1)
    {
        // ── 1. 会话活跃即视为活动（覆盖对话中 / TTS 播放）──────────────────
        if (session_get_state() != SESSION_IDLE)
            touch_activity();

        // ── 1b. 未进低功耗前：插着 USB 就不进低功耗（三态趋势 + 锁存）──────────────
        // 本板无独立 VBUS 检测脚，用「电池端电压趋势」推断是否插 USB，三态：
        //   电压上升 = 充电(插USB) → 置位锁存 s_usb_present；
        //   连续下降 = 掉电(用电池) → 清锁存（允许重新进低功耗）；
        //   持平     = 满电无法估算 → 保持不翻转（沿用上一次明确判定）。
        // ★用滤波电压 bsp_battery_get_voltage_mv() 抗单次尖峰；拔出要求连续 STANDBY_USB_DROP_CNT
        //   次下降才认，滤掉舵机/喇叭瞬时压降与 ADC 噪声。旧版依赖 bsp_battery_is_charging()——
        //   它在满电区(OCV≥4.0V)恒为真、且后台每 10s 才更新，会导致「满电电池永不入睡」和
        //   「唤醒后 charging 未及时置位→10s 内又睡回去」的偶发竞态，故换成本地每秒趋势判定。
        // ★仅在未待机(!s_standby)时跑：进待机后负载骤降会回弹上升，若在待机中据此刷新计时
        //   会把三级关机永久挡住（历史bug）。待机中的插USB改由 2d 快/慢通道负责。
        if (!s_standby)
        {
            uint32_t v = bsp_battery_get_voltage_mv(); // 滤波后端电压，抗单次尖峰
            if (v > 0)
            {
                if (s_usb_base_mv == 0)
                    s_usb_base_mv = v; // 首次建立跟踪基准
                if ((int)v - (int)s_usb_base_mv >= STANDBY_USB_RISE_MV)
                {
                    s_usb_present = true; // 上升=充电→锁存"插USB"
                    s_usb_drop_hit = 0;
                    s_usb_base_mv = v; // 抬高基准，跟随充电电压上行
                }
                else if ((int)s_usb_base_mv - (int)v >= STANDBY_USB_DROP_MV)
                {
                    if (++s_usb_drop_hit >= STANDBY_USB_DROP_CNT)
                    {
                        s_usb_present = false; // 连续下降=掉电→清锁存，允许重新入睡
                        s_usb_drop_hit = 0;
                        s_usb_base_mv = v; // 压低基准，跟随放电电压下行
                    }
                }
                else
                {
                    s_usb_drop_hit = 0; // 持平=满电→保持锁存不翻转
                }
            }
            // ★放行条件 = 本地趋势锁存 ‖ 后台满电区兜底：
            //   s_usb_present（趋势锁存）负责「刚插USB/刚从待机唤醒」这段后台 charging 标志
            //     还没跟上的窗口 → 修掉「唤醒后 10s 内又睡回去」的偶发竞态；
            //   bsp_battery_is_charging() 在满电区(≥4.0V)恒为真，负责「满电插USB、电压顶在
            //     4.1~4.2V 持平、纯趋势判不出上升」的情形 → 修掉「插着USB满电时仍进待机、且
            //     进去后电压没上涨空间唤不醒」的回归（实测 4172mV 满电插USB 曾误入待机）。
            //   代价：满电电池「不插USB」时也会在电压掉到 4.0V 以下前不进待机，但只是延迟
            //     （醒着耗电很快跌破 4.0V 后正常入睡），远小于「插USB睡死唤不醒」的代价。
#if !STANDBY_DEBUG_ALLOW_USB_SLEEP
            if (s_usb_present || bsp_battery_is_charging())
                touch_activity(); // 插着 USB → 刷新活动时间，不进低功耗
#endif
        }

        int64_t now_us = esp_timer_get_time();
        int64_t idle_ms = (now_us - s_last_active_us) / 1000;

        // ── 2. 空闲超时进入一级待机 ────────────────────────────────────────
        if (!s_standby && idle_ms >= STANDBY_LIGHT_TIMEOUT_MS)
        {
            enter_standby();
            last_swing_us = 0; // 进入待机后立即触发首次摆头
        }

        // ── 2b. 继续空闲到二级阈值 → 进入二级待机（关屏 + 停舵机）──────────────
        // 绝对计时：从上次活动起算，与一级共用 s_last_active_us（见宏区说明）。
        // 进入二级后由 enter_deep_standby 自身幂等防重复；摆头随即停止（见步骤 3 条件）。
        if (s_standby && !s_deep_standby && idle_ms >= STANDBY_DEEP_TIMEOUT_MS)
        {
            enter_deep_standby();
        }
        // ── 2c. 继续空闲到三级阈值 → 执行纯关机（硬件断电）
        if (s_standby && s_deep_standby && !s_shutdown_standby && idle_ms >= STANDBY_SHUTDOWN_TIMEOUT_MS)
        {
            enter_shutdown();
        }

        // ── 2c-log. ★回弹电压观测（本轮临时诊断用，不改任何判定逻辑）────────────────
        //   目的：把「进二级后电池端电压随时间回弹爬升」的真实曲线打出来，用于下一轮
        //   据实测把 STANDBY_PWR_RECHECK_RISE_MV（断电前复查阈值，现为 20mV）和
        //   基准采样时机定准。当前 s_deep_base_mv 在进二级瞬间就采（负载卸除、电压仍在回弹爬升期），
        //   到三级复查(70s)时电压比它高出几十 mV → 被误判为「中途插USB」→ 取消断电 + 亮屏，
        //   这就是「一/二级反复自亮、十来分钟才第一次断电」的主因（问题1b）。
        //   ★下一轮据本日志调完阈值/时机后，删除本 2c-log 观测块。
        if (s_deep_standby && s_deep_base_mv > 0)
        {
            uint32_t v_dbg = bsp_battery_read_voltage_mv();
            if (v_dbg > 0)
            {
                int diff = (int)v_dbg - (int)s_deep_base_mv;
                ESP_LOGI(TAG, "[回弹观测] 二级基准=%lu mV 当前=%lu mV 差值=%+d mV（复查阈值+%d→≥即误判插USB）idle=%lldms",
                         (unsigned long)s_deep_base_mv, (unsigned long)v_dbg, diff,
                         STANDBY_PWR_RECHECK_RISE_MV, idle_ms);
            }
        }

        // ── 2d. 待机期插USB检测：快通道(陡升即刻醒) + 慢通道(settle后连续缓升兜底) ──────
        // 无 VBUS 脚，用电压趋势代理，两条通道并行：
        //   快通道(档2)：相对「进待机瞬间电压 s_standby_enter_mv」单次猛涨≥STANDBY_USB_JUMP_MV
        //     （实测插USB~190mV，远超进待机回弹的几十mV）→ 立即唤醒，不等 settle、不用连续，
        //     延迟压到一个采样拍(~1s)，达到近似手机的「插入即醒」。
        //   慢通道：进待机后等 STANDBY_USB_SETTLE_MS 让回弹稳定再捕获基准（二级已由
        //     enter_deep_standby 用更干净的 s_deep_base_mv 提前设好），之后相对基准连续
        //     STANDBY_USB_RISE_CNT 次上升≥STANDBY_USB_RISE_MV 才判插USB，兜住缓慢/微弱充电。
        // 命中任一通道即置「插USB」锁存 s_usb_present 再 standby_wake()，让醒后 1b 据此不再回睡。
        // ★调试期(STANDBY_DEBUG_ALLOW_USB_SLEEP=1)整块跳过：否则插着USB供电时会被立即判为插USB→唤醒，测不下去。
#if !STANDBY_DEBUG_ALLOW_USB_SLEEP
        if (s_standby)
        {
            uint32_t v_now = bsp_battery_read_voltage_mv();

            // 快通道：相对进待机电压陡升 → 立即唤醒
            if (v_now > 0 && s_standby_enter_mv > 0 &&
                (int)v_now - (int)s_standby_enter_mv >= STANDBY_USB_JUMP_MV)
            {
                ESP_LOGW(TAG, "待机期电压陡升(%lu→%lu mV，≥+%d)判为插USB[快通道]，立即唤醒",
                         (unsigned long)s_standby_enter_mv, (unsigned long)v_now, STANDBY_USB_JUMP_MV);
                s_usb_present = true; // 置锁存，醒后 1b 据此不再回睡
                s_usb_base_mv = v_now;
                s_usb_drop_hit = 0;
                standby_wake(); // 退出待机（含二级则先开屏/恢复舵机），回到空闲态
            }
            // 慢通道：settle 后建基准
            else if (s_charge_base_mv == 0)
            {
                if ((now_us - s_standby_enter_us) / 1000 >= STANDBY_USB_SETTLE_MS)
                {
                    s_charge_base_mv = bsp_battery_read_voltage_mv();
                    s_charge_rise_hit = 0;
                    if (s_charge_base_mv > 0)
                        ESP_LOGI(TAG, "待机充电检测基准=%lu mV", (unsigned long)s_charge_base_mv);
                }
            }
            // 慢通道：基准已建 → 连续缓升兜底
            else
            {
                if (v_now > 0 && (int)v_now - (int)s_charge_base_mv >= STANDBY_USB_RISE_MV)
                {
                    if (++s_charge_rise_hit >= STANDBY_USB_RISE_CNT)
                    {
                        ESP_LOGW(TAG, "待机期电压持续上升(%lu→%lu mV)判为插USB[慢通道]，唤醒回空闲",
                                 (unsigned long)s_charge_base_mv, (unsigned long)v_now);
                        s_usb_present = true; // 置锁存，醒后 1b 据此不再回睡
                        s_usb_base_mv = v_now;
                        s_usb_drop_hit = 0;
                        standby_wake(); // 退出待机（含二级则先开屏/恢复舵机），回到空闲态
                    }
                }
                else
                {
                    s_charge_rise_hit = 0; // 未达上升阈值→连击清零，要求"连续"上升
                }
            }
        }
#endif // !STANDBY_DEBUG_ALLOW_USB_SLEEP（2d 待机期插USB唤醒，调试期整块跳过）

        // ── 3.（已废弃，改由随机情绪头部动作驱动，见 D.2 ui_port.c 待机分支）──────
        // 原「一级固定 ±30° 摆头」逻辑：现改为一级待机播随机情绪（头部动作 + GIF），
        // 手臂钉90/关震动由 interaction_set_lowpower 处理，节奏由 GIF-READY 驱动
        // （main_gif_switch_timer_cb），不再需要本任务自己的定时摆头。暂保留代码存档，
        // 需要回退时把 STANDBY_SWING_ENABLE 改回 1 且放开下面注释即可。
#if 0 && STANDBY_SWING_ENABLE
        if (s_standby && !s_deep_standby && (now_us - last_swing_us) / 1000 >= SWING_PERIOD_MS)
        {
            float target = swing_left ? (HEAD_CENTER_DEG - HEAD_SWING_DEG)
                                      : (HEAD_CENTER_DEG + HEAD_SWING_DEG);
            servo_manager_submit_angle(CH_HEAD, target, SWING_SPEED); // 仅头部舵机，最慢速左右摇
            swing_left = !swing_left;
            last_swing_us = now_us;
        }
#else
        (void)swing_left;
        (void)last_swing_us;
#endif

        // ── 3b. 一级退出「归中 gate」到期检查（D.4）：flush 后等 GATE_MS 让 worker
        //   把三轴（含手臂）归中完毕，再恢复正常 GIF 自动轮播。跨线程安全：仅在本任务
        //   （standby_task 自身）读写 s_lp_gif_pending，不与 standby_wake 的写产生竞态
        //   （standby_wake 只置位一次，这里判断到期后清位，先后顺序不敏感）。
        if (s_lp_gif_pending && (now_us - s_lp_gif_pending_us) / 1000 >= STANDBY_LP_GATE_MS)
        {
            s_lp_gif_pending = false;
            if (lvgl_port_lock(1000))
            {
                ui_resume_main_gif_loop(); // 归中已完成，切回正常随机 GIF 循环
                lvgl_port_unlock();
            }
            else
            {
                ESP_LOGE(TAG, "退一级取 LVGL 锁超时，本次未能恢复GIF循环（下次活动会重试）");
            }
        }

        vTaskDelay(pdMS_TO_TICKS(STANDBY_CHECK_MS));
    }
}

void standby_init(void)
{
    if (s_inited)
        return;
    s_inited = true;
    touch_activity();

    // 周期任务：栈放 SPIRAM，节省内部 SRAM；优先级低（仅做轻量 poll + 入队）
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        standby_task,
        "standby",
        4096,
        NULL,
        3, // 低优先级，不与音频/舵机争抢
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
        ESP_LOGE(TAG, "待机监测任务创建失败！");
    else
        ESP_LOGI(TAG, "待机模块初始化完成");
}
