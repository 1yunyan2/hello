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
#include <math.h>      // fmaxf/fabsf：手臂归中速度动态计算（enter_standby，与熄屏时长对齐）
#include <stdatomic.h> // atomic_bool/atomic_store/atomic_load：进入过程「原子门闩」s_entering 跨任务安全读写
#include "object.h"    // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM

static const char *TAG = "STANDBY";

#define STANDBY_LIGHT_TIMEOUT_MS 15000 // 一级
#define STANDBY_DEEP_TIMEOUT_MS 45000
#define STANDBY_SHUTDOWN_TIMEOUT_MS 70000

#define STANDBY_SHUTDOWN_ENABLE 1       ///< 三级开关，插电测试
#define STANDBY_PWR_RECHECK_RISE_MV 200 ///< 二级复查：相对进二级基准电压上升≥此值(mV)判为中途插USB→取消三级关机
#define STANDBY_CHECK_MS 1000           ///< 周期任务检查间隔（1 秒）
#define STANDBY_DEBUG_ALLOW_USB_SLEEP 0 ///< 1=插USB也进低功耗（仅调试用）；0=正常拦截
#define STANDBY_USB_SETTLE_MS 5000      ///< 进一级待机后等此时长让回弹稳定，再捕获充电检测基准
#define STANDBY_USB_RISE_MV 150         ///< 待机期电压相对基准上升≥此值(mV)记一次「疑似插USB」
#define STANDBY_USB_RISE_CNT 3          ///< 连续命中次数达此值判为插USB→唤醒（约数秒，滤回弹）
#define STANDBY_USB_JUMP_MV 120         ///< 相对进待机电压单次上升≥此值(mV)即刻判插USB（快通道）
#define STANDBY_USB_DROP_MV 50          ///< 电压相对跟踪基准下降≥此值(mV)记一次「疑似掉电」
#define STANDBY_USB_DROP_CNT 3          ///< 连续掉电命中判为拔USB→清锁存，允许重新进低功耗

// ─── ★一级待机插USB检测（进一级立即建基准 + 低频复查上升）─────────────────────────
// 方案(用户定，2026-07-09)：
//   1) 进一级【最开头】立即连采 L1_BASE_SAMPLES 次取平均，作为基准 s_l1_usb_base_mv。
//      此刻用户还没插USB（是干净的"插前"电压），多次平均滤单次噪声。
//      实测教训：原来等 settle(5s) 才取基准 → 用户在这5s内已插USB → 基准抓成插后高值 → 判不出。
//      故必须在进一级瞬间就锚，且早于渐变/归中，杜绝"太早插USB把涨压吸进基准"。
//   2) 之后每 L1_RECHECK_MS 抓一次当前电压，相对基准涨≥L1_RISE_MV 即判插USB→唤醒。
//      低频复查（非每秒）：插USB是持久状态，20s 一次足够，省电且避开摆头/瞬时抖动。
// ★基准抓在进一级【最开头】（舵机停止前），比舵机停止后早1~2s，杜绝这段窗口插USB污染基准。
//   代价：基准是"带舵机负载"的较低电压，舵机停止后正常回弹(实测≤150mV)会计入上升。故阈值
//   L1_USB_RISE_MV 设在回弹之上、插USB涨幅之下(实测插USB涨~288mV)→回弹150够不到、插USB能判出。
#define STANDBY_L1_BASE_SAMPLES 3         ///< 一级：进一级瞬间连采几次取平均建基准（滤噪）
#define STANDBY_L1_RECHECK_MS 20000       ///< 一级：每隔此时长复查一次电压是否相对基准上升
#define STANDBY_L1_USB_RISE_MV 200        ///< 一级：复查时电压相对基准上升≥此值(mV)即判插USB→唤醒（>回弹150，<插USB288）
#define STANDBY_SWING_ENABLE 1            ///< ★一级待机摆头总开关（1=每2.5s摆头；0=不摆头，舵机静止）
#define SWING_PERIOD_MS 2500              ///< 待机摆头周期（每 2.5s 换一次方向）
#define HEAD_CENTER_DEG 90.0f             ///< 头部中位角度
#define HEAD_SWING_DEG 30.0f              ///< ★摆动幅度（±x°，即 (90-x)°~(90+x)°，后续直接改这里）
#define SWING_SPEED SERVO_SPEED_VERY_SLOW ///< ★摆头速度（取最慢档 50ms/度，后续可换 SLOW 等档位）
// ─── 模块状态 ───────────────────────────────────────────────────────────────
// s_last_active_us：上次活动时间戳（微秒，esp_timer 单调时钟）
// s_standby：当前是否待机。均用 volatile + 简单读写，配合 64 位读取注意原子性。
static volatile int64_t s_last_active_us = 0;
static volatile bool s_standby = false;      // 一级（轻度）待机
static volatile bool s_deep_standby = false; // 二级（深度）待机
static volatile bool s_inited = false;
static volatile bool s_shutdown_standby = false; // 三级（关机）

static atomic_bool s_entering = ATOMIC_VAR_INIT(false); // true=正在执行 enter_standby()/enter_deep_standby()（含阻塞渐暗），此窗口内 wake 一律忽略
static volatile uint32_t s_deep_base_mv = 0;            // 进二级时(负载已恒定至最小)的基准电压 mV，供三级复查比对
// ─── 待机期插USB检测状态（goal 2）───
static volatile int64_t s_standby_enter_us = 0;  // 进入一级待机的时间戳，用于 settle 后再取基准
static volatile uint32_t s_standby_enter_mv = 0; // 进入一级待机瞬间的端电压 mV，供快通道判「陡升插USB」
static volatile uint32_t s_charge_base_mv = 0;   // 充电检测基准电压 mV（0=尚未捕获，待 settle 后捕获）
static volatile int s_charge_rise_hit = 0;       // 电压相对基准连续明显上升的命中计数
// ─── 插USB锁存 + 趋势跟踪（未待机时防再入睡 / 判拔出，见 standby_task 步骤 1b）───
static volatile bool s_usb_present = false; // 锁存：当前是否判定插着USB（true→不进低功耗）
static volatile uint32_t s_usb_base_mv = 0; // 上升/下降趋势跟踪基准 mV（0=尚未建立）
static volatile int s_usb_drop_hit = 0;     // 相对基准连续下降命中计数（滤单次波动）
// ─── 一级待机插USB检测状态（进一级建基准 + 每 L1_RECHECK_MS 复查上升，见 STANDBY_L1_* 宏）───
static volatile uint32_t s_l1_usb_base_mv = 0;   // 进一级瞬间连采平均建立的基准 mV（0=未建立）
static volatile int64_t s_l1_last_recheck_us = 0; // 上次复查时间戳，用于每 L1_RECHECK_MS 复查一次

static volatile bool s_lp_gif_pending = false;   // 归中标志：GIF 播放中，等待归中完成
static volatile int64_t s_lp_gif_pending_us = 0; // 发起归中（flush）的时间戳
#define STANDBY_LP_GATE_MS 400                   // 归中等待时长：与 enter_deep_standby 的 flush 等待手法一致（够 worker 归中完成）

// ─── 退二级「唤醒失败」补偿状态 ───────────────────────────────────────────
// standby_wake() 二级分支开屏要取 LVGL 锁，若 LVGL 任务恰好在 lv_timer_handler()
// 里处理耗时的 GIF 解码/渲染（偶发单帧较慢），两次 1000ms 重试都可能失败。原实现
// 失败后整段回滚、只等"下次触摸活动"重新调用 standby_wake()——但若用户只碰了一次
// 就不再有触摸，永远等不到下一次，表现为"退不出二级、卡住"。
// 加此标志：失败后置位，standby_task 周期任务（1s一拍）据此自动重试，不依赖用户
// 再次触摸。重试时机与触摸触发是同一份 standby_wake()，其内部已有幂等判断
// （!s_standby 直接返回），可安全反复调用。
static volatile bool s_wake_retry_pending = false;

/**
 * @brief 刷新「上次活动」时间戳（内部）
 */
static inline void touch_activity(void)
{
    // [临时诊断] 待机中若计时被刷新，打印调用来源返回地址，定位「计时被谁重置」。
    //   __builtin_return_address(0) 是调用 touch_activity 的上一层地址，配合
    //   xtensa-esp32s3-elf-addr2line -e build/xxx.elf <地址> 即可反查具体函数/行号。
    //   （wake_word_callback / ui_dispatch_touch_event 等都会经 standby_notify_activity→standby_wake→本函数）
    //   验证定位到根因后删除本块。
    if (s_standby)
    {
        int64_t idle_before = (esp_timer_get_time() - s_last_active_us) / 1000;
        ESP_LOGW(TAG, "[计时被刷] 待机中 touch_activity 被调用！idle=%lldms 将清零，caller=%p",
                 idle_before, __builtin_return_address(0));
    }
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
    s_standby_enter_us = esp_timer_get_time();
    s_standby_enter_mv = 0;
    s_charge_base_mv = 0;
    s_charge_rise_hit = 0;

    // ★一级插USB检测基准：在进一级【最开头】立即连采 L1_BASE_SAMPLES 次取平均建立。
    //   此刻早于下面的关屏渐变/停舵机/归中，抢在用户插USB之前→基准干净（用户定：抓最开头防污染）。
    //   多次平均滤单次 ADC 噪声。基准是"带舵机负载"的较低电压，舵机停止的回弹靠高阈值(200mV)躲开。
    //   之后 standby_task 每 L1_RECHECK_MS 复查电压是否相对它上升≥L1_USB_RISE_MV。
    {
        uint32_t sum = 0;
        int valid = 0;
        for (int i = 0; i < STANDBY_L1_BASE_SAMPLES; i++)
        {
            uint32_t v = bsp_battery_read_voltage_mv();
            if (v > 0)
            {
                sum += v;
                valid++;
            }
        }
        s_l1_usb_base_mv = valid > 0 ? (sum / valid) : 0;
        s_l1_last_recheck_us = s_standby_enter_us; // 复查计时从进一级起算
        ESP_LOGI(TAG, "一级插USB检测基准=%lu mV（进一级瞬间连采%d次平均）",
                 (unsigned long)s_l1_usb_base_mv, STANDBY_L1_BASE_SAMPLES);
    }

    ui_force_back_to_main();                                   // 强制回归主页面
    wake_word_stop();                                          // 关闭唤醒词监听（仅切 is_running）
    ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // 停止震动马达 PWM

    // ★步骤1（顺序修正）：低功耗标志与清队【必须最先做】，早于 flush + 手臂归中。
    //   - interaction_set_lowpower(true) 前移：原实现放在末尾（距 flush 约 1.3s），这段窗口内
    //     worker 若已开始执行存量情绪，lowpower 仍为 false → 手臂照常摆（问题5「进低功耗手臂不立即
    //     归中、还运行一段」的根因之一）。前移后，之后 worker 取出的任何情绪都按低功耗（手臂钉90）执行。
    //   - interaction_flush_queue()：清掉 ia_queue 里【未执行】的存量情绪（一级待机 GIF 定时器持续入队），
    //     否则它们会排在归中之后被 worker 执行，把手臂/头又带跑（问题1a/5 根因）。
    ESP_LOGW(TAG, "[进一级耗时] T0 起点 t=%lld us", esp_timer_get_time());
    interaction_set_lowpower(true);
    interaction_flush_queue();
    ESP_LOGW(TAG, "[进一级耗时] T1 flush_queue后 t=%lld us", esp_timer_get_time());

    // ★步骤2：flush 打断 servo_manager 里正在执行的舵机动作 + 清 servo_manager 队列，并清打断标志。
    //   （interaction_flush_queue 清的是 interaction 层队列；这里清的是 servo_manager 层队列，两层都要清。）
    servo_manager_flush();

    // ★原子门闩：只把这一次背光线性渐变（100%→待机亮度，阻塞约 800ms）包起来。渐变期间触摸被
    //   直接忽略（见 standby_wake 拦截），避免半路插手导致 GIF 丢失只剩背光；渐变一结束立刻释放，
    //   之后的压音量/停马达/舵机归中都允许触摸打断（触摸本就要恢复这些，不该被门闩连累）。
    atomic_store(&s_entering, true);
    bsp_board_lcd_fade_brightness(BSP_LCD_BK_DEFAULT_PCT, BSP_LCD_BK_STANDBY_PCT); // 亮度修改（线性渐变，阻塞）
    atomic_store(&s_entering, false);

    bsp_board_codec_enter_lowpower(); // ★音量线性渐变压低（不写NVS），防低功耗期离线音频/提示音过大

    bsp_servo_move_smooth(CH_L_ARM, HEAD_CENTER_DEG, SERVO_SPEED_FAST); // 左臂同步归中 90°（快速 5ms/度，进低功耗手臂尽快归位）
    bsp_servo_move_smooth(CH_R_ARM, HEAD_CENTER_DEG, SERVO_SPEED_FAST); // 右臂同步归中 90°（快速）

    // ★快通道锚点：负载已全部卸掉（关屏渐暗/停语音/停马达/停舵机/压音量都已完成），
    //   此刻电压已回弹到位，以此稳定值为锚。之后 2d 快通道相对它涨≥JUMP_MV 才判插USB，
    //   回弹已包含在锚里、不再计入，从根上消除「进待机后回弹自激唤醒」。
    s_standby_enter_mv = bsp_battery_read_voltage_mv();
    // 注：一级插USB检测基准 s_l1_usb_base_mv 已在本函数【最开头】连采平均建好（早于此处），
    //   此处不再重取。s_standby_enter_mv 仅供二级快通道用。
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
    if (s_deep_standby)
        return;
    s_deep_standby = true;
    ESP_LOGW(TAG, "无活动超过 %d ms，进入二级待机（关屏 + 停舵机，CPU 不睡，闹钟照常）",
             STANDBY_DEEP_TIMEOUT_MS);

    atomic_store(&s_entering, true); // 原子
    bsp_board_lcd_fade_brightness(BSP_LCD_BK_STANDBY_PCT, 0);
    atomic_store(&s_entering, false);
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

    // 4) 停舵机
    bsp_servo_idle();

    // 4b) ★捕获二级基准电压：此刻屏已关、舵机已停，负载恒定到最小，端电压接近真实 OCV，
    //     "进一级判一次并锁定"的决策到这里已确认在用电池。以此为基准，供三级关机前复查
    //     「二级期间电压是否明显上升」——上升即中途插了 USB，取消断电（见 enter_shutdown）。
    //     用同步即时读取（多次采样平均，不受后台慢速 IIR 滞后影响），反映当前稳态端电压。
    vTaskDelay(pdMS_TO_TICKS(3000)); // 延迟再进行电压监测
    s_deep_base_mv = bsp_battery_read_voltage_mv();
    ESP_LOGI(TAG, "二级基准电压=%lu mV（供三级复查是否中途插USB）", (unsigned long)s_deep_base_mv);
    // 二级负载更恒定，用它作为更干净的充电检测基准，覆盖一级基准并清零命中（避免一级→二级
    // 二次回弹被误判成上升）。
    // ★同步更新快通道基准 s_standby_enter_mv：原实现只覆盖了慢通道 s_charge_base_mv，
    //   快通道仍在用"进一级瞬间"的旧电压比对——但进二级会关屏+停舵机，负载骤降造成的正常
    //   回弹（实测 100~150mV）用旧的一级基准算，会被当成陡升直接命中 STANDBY_USB_JUMP_MV
    //   阈值，导致进二级后必然误判插USB并立即唤醒（快通道每次必现）。这里一并覆盖为二级
    //   基准，之后快通道只需对"二级负载已稳定"之后的电压变化判断，不再被这次回弹污染。
    if (s_deep_base_mv > 0)
    {
        s_charge_base_mv = s_deep_base_mv;
        s_charge_rise_hit = 0;
        s_standby_enter_mv = s_deep_base_mv;
    }
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

/**
 * @brief 退出待机：亮度/音量渐变恢复 + 开屏 + 恢复舵机 PWM + 归中 + 置标志
 */
void standby_wake(void)
{
    // 刷新时间
    touch_activity();
    if (!s_standby)
        return;

    if (atomic_load(&s_entering))
    {
        ESP_LOGW(TAG, "进入待机流程进行中（背光渐暗），本次触摸/唤醒直接忽略");
        return;
    }
    ESP_LOGI(TAG, "退出待机，恢复正常状态");
    // 【诊断-DBG0】打印此刻 s_deep_standby 的真实值，确认走一级还是二级恢复分支（验证OK后可删）
    ESP_LOGW(TAG, "[DBG0] standby_wake 入口 s_deep_standby=%d", (int)s_deep_standby);
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
                // 【诊断-DBG2】disp_on 成功后打印，确认走到这一步（验证OK后可删）
                ESP_LOGW(TAG, "[DBG2] disp_on 成功(attempt=%d)，时间戳=%lld us",
                         attempt, esp_timer_get_time());

                lvgl_port_unlock();
                disp_on_ok = true;
            }
        }
        if (!disp_on_ok)
        {
            // 两次都没取到锁：整段放弃本次唤醒，标志【保持不变】（仍是二级待机），不做任何
            // 舵机/背光/震动恢复，避免"屏黑但舵机动"的半恢复态。
            // 置位补偿标志，交给 standby_task 周期任务自动重试，不依赖用户再次触摸。
            ESP_LOGE(TAG, "[DBG2-FAIL] 退二级两次取 LVGL 锁均超时，放弃本次唤醒（保持待机，转交周期任务自动重试）");
            s_wake_retry_pending = true;
            return;
        }

        s_wake_retry_pending = false; // 开屏成功：清掉补偿标志，避免残留触发多余的周期任务重试

        bsp_board_lcd_fade_brightness(0, BSP_LCD_BK_DEFAULT_PCT);

        interaction_flush_queue();       // 清 interaction 存量情绪，防 servo_resume 归中后被残留请求带跑
        interaction_set_lowpower(false); // 清低功耗标志：情绪播放恢复手臂+震动正常参与
        restore_motor_pwm();             // 恢复震动马达 LEDC 通道
        bsp_board_codec_exit_lowpower(); // 音量线性渐变恢复为用户设置值（阻塞，读 NVS）

        bsp_servo_resume(); // 恢复三路 PWM 并错峰归中（含头部，见 bsp_servo.c）；放最后，屏已亮

        s_deep_standby = false;
        s_standby = false;

        if (lvgl_port_lock(1000))
        {
            ui_resume_main_gif_loop();
            lvgl_port_unlock();
        }
        else
        {
            // 取锁失败：不放弃，复用一级分支同款的 s_lp_gif_pending 轮询补偿——
            // standby_task 每秒会检查一次并重试取锁恢复 GIF 循环，见本函数末尾一级
            // 分支同样的用法。避免二级这里独此一次尝试失败就永久卡住无 GIF 画面。
            ESP_LOGE(TAG, "退二级恢复GIF轮播取 LVGL 锁超时（背光/舵机已恢复），转交周期任务补偿重试");
            s_lp_gif_pending = true;
            s_lp_gif_pending_us = esp_timer_get_time() - STANDBY_LP_GATE_MS; // 立即到期，下一拍就重试
        }

        wake_word_start();
        return;
    }

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

    s_standby = false;

    s_lp_gif_pending = true;
    s_lp_gif_pending_us = esp_timer_get_time();
}

/**
 * @brief 待机监测周期任务
 */
static void standby_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余
    bool swing_left = false;   // 下一次摆动方向（左/右交替）
    int64_t last_swing_us = 0; // 上次提交摆头的时间戳

    ESP_LOGI(TAG, "待机监测任务启动（超时 %d ms）", STANDBY_LIGHT_TIMEOUT_MS);
    touch_activity(); // 初始化活动时间，从启动开始计时

    while (1)
    {
        // ── 1. 会话活跃即视为活动（覆盖对话中 / TTS 播放）──────────────────
        if (session_get_state() != SESSION_IDLE)
            touch_activity();
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
            last_swing_us = 0; // 舍弃
        }

        // ── 2a-log. ★三级低功耗倒计时打印（临时诊断：定位计时被谁重置）──────────
        //   每个轮询拍（1s）打印当前 idle 及距一/二/三级各还差多少秒。
        //   若某一拍 idle 突然回落（如从 15000 掉回 0），说明 s_last_active_us 被刷新，
        //   紧接着 touch_activity 的 [计时被刷] 日志会指出是谁刷的。验证完删除本块。
        {
            int64_t to_l1 = (int64_t)STANDBY_LIGHT_TIMEOUT_MS - idle_ms;    // 距一级
            int64_t to_l2 = (int64_t)STANDBY_DEEP_TIMEOUT_MS - idle_ms;     // 距二级
            int64_t to_l3 = (int64_t)STANDBY_SHUTDOWN_TIMEOUT_MS - idle_ms; // 距三级
            ESP_LOGI(TAG, "[倒计时] idle=%lldms | 一级%s(余%lldms) 二级%s(余%lldms) 三级%s(余%lldms)",
                     idle_ms,
                     s_standby ? "已进" : "未进", to_l1 > 0 ? to_l1 : 0,
                     s_deep_standby ? "已进" : "未进", to_l2 > 0 ? to_l2 : 0,
                     s_shutdown_standby ? "已进" : "未进", to_l3 > 0 ? to_l3 : 0);
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
        // ── 2d-一级：一级待机(未进二级)插USB检测：每 L1_RECHECK_MS 复查一次相对基准是否上升 ──
        // 基准 s_l1_usb_base_mv 已在 enter_standby() 进一级【最开头】连采平均建好（干净的"插前"电压）。
        // 此处每隔 L1_RECHECK_MS(20s) 抓一次当前电压，相对基准涨≥L1_USB_RISE_MV(200mV，>回弹<插USB)
        // 即判插USB→置锁存+唤醒。低频复查：插USB是持久状态，不必每秒查，省电且避开摆头/瞬时抖动。
        if (s_standby && !s_deep_standby && s_l1_usb_base_mv > 0)
        {
            if ((now_us - s_l1_last_recheck_us) / 1000 >= STANDBY_L1_RECHECK_MS)
            {
                s_l1_last_recheck_us = now_us;
                uint32_t v_now = bsp_battery_read_voltage_mv();
                int diff = (int)v_now - (int)s_l1_usb_base_mv;
                ESP_LOGI(TAG, "[一级插USB复查] 基准=%lu 当前=%lu diff=%+d mV（阈值+%d）",
                         (unsigned long)s_l1_usb_base_mv, (unsigned long)v_now, diff, STANDBY_L1_USB_RISE_MV);

                if (v_now > 0 && diff >= STANDBY_L1_USB_RISE_MV)
                {
                    ESP_LOGW(TAG, "一级待机电压相对基准上升(%lu→%lu mV，≥+%d)判为插USB，唤醒回空闲",
                             (unsigned long)s_l1_usb_base_mv, (unsigned long)v_now, STANDBY_L1_USB_RISE_MV);
                    s_usb_present = true; // 置锁存，醒后 1b 据此不再回睡
                    s_usb_base_mv = v_now;
                    s_usb_drop_hit = 0;
                    standby_wake();
                }
            }
        }
        // ── 2d-二级：二级待机专用 快通道(陡升即刻醒) + 慢通道(settle后连续缓升兜底) ──────
        else if (s_standby) // 即 s_standby && s_deep_standby
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

        // ── 3c. 退二级「唤醒失败」自动补偿重试：见 s_wake_retry_pending 定义处说明。
        //   仅在仍处于二级待机时重试才有意义；一旦已经不在二级（比如被其他路径唤醒）
        //   直接清标志，不做多余调用。standby_wake() 内部幂等（!s_standby 直接返回），
        //   可安全反复调用，不会与其他唤醒路径冲突。
        if (s_wake_retry_pending)
        {
            if (!s_deep_standby)
            {
                s_wake_retry_pending = false; // 已不在二级（被别的路径唤醒），无需再重试
            }
            else
            {
                ESP_LOGW(TAG, "周期任务自动重试：重新尝试退出二级待机");
                standby_wake(); // 成功则内部会清 s_wake_retry_pending；仍失败则保持置位，下一拍再试
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
        5, // 低优先级，不与音频/舵机争抢
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
        ESP_LOGE(TAG, "待机监测任务创建失败！");
    else
    {
        ESP_LOGI(TAG, "待机模块初始化完成");
        PRINT_TASK_CREATED(TAG, "standby", 4096, 0); // 栈在PSRAM
    }
}
