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
#include "esp_lvgl_port.h" // lvgl_port_lock/unlock：开关屏必须与 LVGL 刷新互斥（见 enter_deep_standby/standby_wake）
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "STANDBY";

// ─── 可调参数（时间阈值做成宏，方便后续调整）─────────────────────────────
// 两级待机时间均从「上次活动」起算（绝对计时，进入二级不清空一级计时）：
//   - 一级（轻度）：LIGHT_TIMEOUT_MS 后降亮度 + 关语音检测 + 头部摆动。
//   - 二级（深度）：DEEP_TIMEOUT_MS 后进一步省电——A 层方案：关屏 + 停舵机，CPU 不睡
//     （见下方「二级（深度）待机配置」注释；逻辑已实现，enter_deep_standby/standby_wake）。
#define STANDBY_LIGHT_TIMEOUT_MS 10000     ///< 一级（轻度）待机阈值：60s 无活动
#define STANDBY_DEEP_TIMEOUT_MS 100000     ///< 二级（深度）待机阈值：100s 无活动（绝对计时，从上次活动起算）
#define STANDBY_SHUTDOWN_TIMEOUT_MS 150000 ///< 三级（关机）阈值：150s 无活动（绝对计时）
#define STANDBY_SHUTDOWN_ENABLE 1          ///< 三级（关机）总开关（1=启用）
// 三级关机 = 二级稳态复查电池后，直接调 bsp_battery_power_off()（GPIO18/OPT 高→低下降沿
// 命令 HK015T 断电，屏幕探针实测有效）。旧的「借重启 + RTC 魔数 + app_main 拉高」实测锁不住
// 电、还造成三级关机死循环，已废弃。
#define STANDBY_PWR_RECHECK_RISE_MV 20 ///< 二级复查：相对进二级基准电压上升≥此值(mV)判为中途插USB→取消三级关机
#define STANDBY_CHECK_MS 1000          ///< 周期任务检查间隔（1 秒）
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
    // ★快通道基准：进待机「瞬间」（负载还在、尚未回弹）的端电压，是这段时间的最低点，
    //   之后无论回弹还是插USB都只会更高——以它为锚，猛涨≥JUMP_MV 才可能是插USB（见 2d）。
    s_standby_enter_mv = bsp_battery_read_voltage_mv();
    s_charge_base_mv = 0;
    s_charge_rise_hit = 0;
    /* 进入待机前，把界面强制退回主界面：避免「已降亮度但仍卡在某菜单/闹钟编辑页」。
     * 该函数内部自带 LVGL 锁，跨任务调用安全；已在主界面则直接返回。 */
    ui_force_back_to_main();
    bsp_board_lcd_set_brightness(BSP_LCD_BK_STANDBY_PCT);      // 背光降到 20%
    wake_word_stop();                                          // 关闭唤醒词监听（仅切 is_running）
    ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // ★新增：立刻停止震动马达 PWM，防止待机期间误触发高功耗

    // ★步骤1：先 flush 打断进待机前正在播的 idle/情绪舵机动作 + 清队，并清空打断标志。
    //   （否则残留动作会排在下面的手臂归中之前先跑完，手臂跟着摆——这是回归原因。）
    servo_manager_flush();
    // ★步骤2：显式把左右臂归中钉到 90°（恢复原始正确行为）。flush 已清队+清打断标志，
    //   故这两条会作为干净的新请求被 worker 执行，把手臂稳稳归中后保持不动。
    //   头部不在此归中——随后由周期任务接管开始慢摆（见 standby_task）。
    servo_manager_submit_angle(CH_L_ARM, HEAD_CENTER_DEG, SERVO_SPEED_SLOW); // 左臂归中 90°
    servo_manager_submit_angle(CH_R_ARM, HEAD_CENTER_DEG, SERVO_SPEED_SLOW); // 右臂归中 90°
}

/**
 * @brief 进入二级（深度）待机：彻底关屏 + 停舵机 PWM（A 层省电，CPU 不睡）
 *
 * 在一级待机基础上进一步省电：
 *   1. bsp_board_lcd_off()  —— 关背光 + 关显示控制器（比一级仅降亮度 50% 省得多）；
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

    // 1) 关屏：bsp_board_lcd_off 内部先关背光再关显示控制器，省电最佳。
    //    ★必须持 LVGL 锁再调：关显示走 SPI 给 ST7789 发 DISPOFF，与 LVGL 刷新任务的 draw_bitmap
    //      共用同一 SPI panel 句柄。不加锁会与刷新事务抢 SPI，导致 disp_on_off 失败/阻塞
    //      （与 application.c 开机调 bsp_board_lcd_on 时先加锁的约定保持一致）。
    //    仅把关屏一句包进锁内：拿锁会阻塞 LVGL 刷新，下面的舵机停止/delay 不能占着锁。
    if (lvgl_port_lock(1000))
    {
        bsp_board_lcd_off(bsp_board_get_instance());
        lvgl_port_unlock();
    }
    else
    {
        ESP_LOGE(TAG, "进二级取 LVGL 锁超时，跳过关屏（仅停舵机）");
    }

    // 2) ★先清空舵机队列 + 打断正在执行的动作，再停 PWM —— 否则会“停了又被点亮”：
    //    摆头每 2.5s 入队一次，进二级那一刻队列里可能还有残留摆头动作，或 worker 正卡在
    //    插值循环里。若直接 bsp_servo_idle()，worker 随后的 iot_servo_write_angle /
    //    归中会重新 set_duty 点亮刚被 ledc_stop 的 PWM，表现为“二级了舵机还在动”。
    //    servo_manager_flush() 置打断标志 + xQueueReset 清残留请求（非阻塞）。
    servo_manager_flush();
    // 3) flush 是非阻塞的：正在执行的插值步要几十 ms 才在循环边界真正停下，且 worker 收尾
    //    还会做一次归中。这里给一小段时间让 worker 完全静默（停止再写 LEDC），再断 PWM。
    vTaskDelay(pdMS_TO_TICKS(400));

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

void standby_wake(void)
{
    // 幂等：先刷新活动时间，避免唤醒后立刻又超时
    touch_activity();
    if (!s_standby)
        return;
    ESP_LOGI(TAG, "退出待机，恢复正常状态");
    s_standby = false;

    // ── 若之前已进入二级（关屏+停舵机），先恢复二级再恢复一级 ──────────────────
    // 顺序：开屏 → 舵机恢复（resume 内部已慢速归中）。开屏走 bsp_board_lcd_on，
    // 它内部会把背光直接拉回 100%，因此下面一级的 set_brightness 仅在“未进二级”时需要。
    if (s_deep_standby)
    {
        s_deep_standby = false;
        // ★开屏必须持 LVGL 锁再调：bsp_board_lcd_on 内部走 SPI 给 ST7789 发 DISPON，
        //   与 LVGL 刷新任务共用同一 SPI panel 句柄。不加锁会与刷新事务抢 SPI，导致
        //   esp_lcd_panel_disp_on_off 失败或永久阻塞——这正是「打印了退出待机但屏仍黑、
        //   触摸任务卡死、后续触摸全无响应」的根因（与 application.c 开机开屏先加锁的约定一致）。
        if (lvgl_port_lock(1000))
        {
            bsp_board_lcd_on(bsp_board_get_instance()); // 开显示 + 背光回 100%
            // ★恢复 GIF 自动轮播：待机时 main_gif_ready_cb 停了自循环、GIF 定格在最后一帧
            //   并被 LVGL pause（不再触发 ready_cb），故醒来必须显式重启轮播，否则屏幕卡在
            //   一张静止 GIF 不再切换。ui_resume_main_gif_loop 内部走 lv_timer_resume，
            //   是 LVGL 操作，必须在本锁内调用。
            ui_resume_main_gif_loop();
            lvgl_port_unlock();
        }
        else
        {
            ESP_LOGE(TAG, "退二级取 LVGL 锁超时，本次未能开屏（下次活动会重试）");
        }
        bsp_servo_resume(); // 恢复三路 PWM 并慢速归中（含头部）
        // 退出待机统一重开唤醒词监听（无论二级期间是否在监听，恢复后都要可用）。
        // 幂等：wake_word_start 内部若已在运行则无副作用。
        wake_word_start();
        return; // 二级路径已完成全部恢复，不再走一级分支
    }

    // ── 仅一级待机的恢复 ──────────────────────────────────────────────────────
    bsp_board_lcd_set_brightness(BSP_LCD_BK_DEFAULT_PCT); // 背光恢复 100%
    // ★新增：恢复震动马达 PWM 输出
    ledc_channel_config_t motor_channel = {
        .speed_mode = BSP_MOTOR_LEDC_MODE,
        .channel = BSP_MOTOR_LEDC_CHANNEL,
        .timer_sel = BSP_MOTOR_LEDC_TIMER,
        .gpio_num = BSP_MOTOR_VIB_PIN,
        .duty = BSP_MOTOR_DUTY_MAX, // 初始停止（恒高电平 → 低有效断电）
        .hpoint = 0,
    };
    ledc_channel_config(&motor_channel);
    wake_word_start();                                                      // 重新开启唤醒词监听
    servo_manager_submit_angle(CH_HEAD, HEAD_CENTER_DEG, SERVO_SPEED_SLOW); // 头部归中
    // ★恢复 GIF 自动轮播（同二级说明）：一级待机也停了 GIF 自循环，需显式重启，否则醒来
    //   屏幕卡在定格 GIF。ui_resume_main_gif_loop 走 lv_timer_resume，须持 LVGL 锁调用。
    if (lvgl_port_lock(1000))
    {
        ui_resume_main_gif_loop();
        lvgl_port_unlock();
    }
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
            if (s_usb_present || bsp_battery_is_charging())
                touch_activity(); // 插着 USB → 刷新活动时间，不进低功耗
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

        // ── 2d. 待机期插USB检测：快通道(陡升即刻醒) + 慢通道(settle后连续缓升兜底) ──────
        // 无 VBUS 脚，用电压趋势代理，两条通道并行：
        //   快通道(档2)：相对「进待机瞬间电压 s_standby_enter_mv」单次猛涨≥STANDBY_USB_JUMP_MV
        //     （实测插USB~190mV，远超进待机回弹的几十mV）→ 立即唤醒，不等 settle、不用连续，
        //     延迟压到一个采样拍(~1s)，达到近似手机的「插入即醒」。
        //   慢通道：进待机后等 STANDBY_USB_SETTLE_MS 让回弹稳定再捕获基准（二级已由
        //     enter_deep_standby 用更干净的 s_deep_base_mv 提前设好），之后相对基准连续
        //     STANDBY_USB_RISE_CNT 次上升≥STANDBY_USB_RISE_MV 才判插USB，兜住缓慢/微弱充电。
        // 命中任一通道即置「插USB」锁存 s_usb_present 再 standby_wake()，让醒后 1b 据此不再回睡。
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

        // ── 3. 待机摆头（仅一级、且未进二级时；走 servo_manager 队列不抢线程）──
        // 进入二级后舵机 PWM 已停，不能再摆头——否则会重新点亮 PWM 抵消省电。
        // ★由 STANDBY_SWING_ENABLE 控制：=0 时一级待机不摆头，舵机进待机归中后保持静止。
#if STANDBY_SWING_ENABLE
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
