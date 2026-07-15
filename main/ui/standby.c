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
#include <stdatomic.h> // atomic_bool/atomic_store/atomic_load：进入过程「原子门闩」s_entering 跨任务安全读写
#include "object.h"    // PRINT_TASK_CREATED / PRINT_TASK_STACK_HWM

static const char *TAG = "STANDBY";

// ─── 低功耗两级时间阈值（2026-07-10 删一级后，数值为占位可自调）──────────────────
//   两级模型：空闲 DEEP_TIMEOUT → 深度待机（关屏+停舵机）；继续空闲 SHUTDOWN_TIMEOUT →
//   三级关机。中间不再有"一级轻度待机"（降亮度/摆头/情绪队列驱动 GIF 全部删除）。
#define STANDBY_DEEP_TIMEOUT_MS 40000     ///< 空闲超此值直接进深度待机（关屏+停舵机），占位可调
#define STANDBY_SHUTDOWN_TIMEOUT_MS 60000 ///< 空闲超此值执行三级关机（硬件断电），占位可调

#define STANDBY_SHUTDOWN_ENABLE 1       ///< 三级开关，插电测试
#define STANDBY_PWR_RECHECK_RISE_MV 200 ///< 深度待机复查：相对基准电压上升≥此值(mV)判为中途插USB→取消关机
#define STANDBY_CHECK_MS 1000           ///< 周期任务检查间隔（1 秒）
#define STANDBY_DEBUG_ALLOW_USB_SLEEP 1 ///< 1=插USB也进低功耗（仅调试用）；0=正常拦截
#define STANDBY_USB_SETTLE_MS 5000      ///< 进深度待机后等此时长让回弹稳定，再捕获充电检测基准
#define STANDBY_USB_RISE_MV 150         ///< 待机期电压相对基准上升≥此值(mV)记一次「疑似插USB」
#define STANDBY_USB_RISE_CNT 3          ///< 连续命中次数达此值判为插USB→唤醒（约数秒，滤回弹）
#define STANDBY_USB_JUMP_MV 120         ///< 相对进待机电压单次上升≥此值(mV)即刻判插USB（快通道）
#define STANDBY_USB_DROP_MV 50          ///< 电压相对跟踪基准下降≥此值(mV)记一次「疑似掉电」
#define STANDBY_USB_DROP_CNT 3          ///< 连续掉电命中判为拔USB→清锁存，允许重新进低功耗

// ─── 深度待机舵机归中参数（进深度待机时三轴先并行归中 90° 再停 PWM）──────────────
//   替代旧的"手臂 smooth + 头部 flush 异步收尾 + 轮询 is_idle"错峰配合：改为一次性
//   bsp_servo_move_all_parallel 同步归中（该函数为阻塞插值，走到位才返回），再固定
//   等待一小段（宏可调，兜底插值收尾）后 bsp_servo_idle 停 PWM，根除抢 LEDC 通道竞态。
#define STANDBY_SERVO_CENTER_SPEED SERVO_SPEED_SLOW ///< 归中速度
#define STANDBY_SERVO_CENTER_WAIT_MS 5000           ///< 归中后固定等待再停 PWM（占位，可调）

// ─── 关机前震动提醒（距三级关机剩 WARN_MS 时震动 WARN_VIB_MS 提醒用户）──────────────
//   平时深度待机停马达省电；仅在临近关机的窗口内震动一段，提醒"想留着就动一下"。
#define STANDBY_SHUTDOWN_WARN_MS 10000    ///< 距三级关机剩此值(ms)时开始震动提醒
#define STANDBY_SHUTDOWN_WARN_VIB_MS 3000 ///< 震动提醒持续时长(ms)

#define HEAD_CENTER_DEG 90.0f ///< 头部中位角度（进深度待机归中用）
// ─── 模块状态 ───────────────────────────────────────────────────────────────
// s_last_active_us：上次活动时间戳（微秒，esp_timer 单调时钟）
// s_deep_standby：当前是否处于低功耗（深度待机）。删一级后它是唯一的低功耗总标志。
//   均用 volatile + 简单读写，配合 64 位读取注意原子性。
static volatile int64_t s_last_active_us = 0;
static volatile bool s_deep_standby = false; // 深度（唯一）待机：关屏+停舵机，CPU 不睡
static volatile bool s_inited = false;
static volatile bool s_shutdown_standby = false; // 三级（关机）

static atomic_bool s_entering = ATOMIC_VAR_INIT(false); // true=正在执行 enter_deep_standby()（含阻塞渐暗/归中），此窗口内 wake 一律忽略
static volatile uint32_t s_deep_base_mv = 0;            // 进深度待机(负载已恒定至最小)的基准电压 mV，供三级复查比对
// ─── 待机期插USB检测状态（goal 2）───
static volatile int64_t s_standby_enter_us = 0;  // 进入深度待机的时间戳，用于 settle 后再取基准
static volatile uint32_t s_standby_enter_mv = 0; // 进入深度待机后的端电压 mV，供快通道判「陡升插USB」
static volatile uint32_t s_charge_base_mv = 0;   // 充电检测基准电压 mV（0=尚未捕获，待 settle 后捕获）
static volatile int s_charge_rise_hit = 0;       // 电压相对基准连续明显上升的命中计数
// ─── 插USB锁存 + 趋势跟踪（未待机时防再入睡 / 判拔出，见 standby_task 步骤 1b）───
static volatile bool s_usb_present = false; // 锁存：当前是否判定插着USB（true→不进低功耗）
static volatile uint32_t s_usb_base_mv = 0; // 上升/下降趋势跟踪基准 mV（0=尚未建立）
static volatile int s_usb_drop_hit = 0;     // 相对基准连续下降命中计数（滤单次波动）

// ─── 关机前震动提醒状态（防重复触发/重复停）───
static volatile bool s_warn_vibrating = false; // 当前是否处于关机前震动提醒窗口

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
// （!s_deep_standby 直接返回），可安全反复调用。
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
    if (s_deep_standby)
    {
        int64_t idle_before = (esp_timer_get_time() - s_last_active_us) / 1000;
        ESP_LOGW(TAG, "[计时被刷] 待机中 touch_activity 被调用！idle=%lldms 将清零，caller=%p",
                 idle_before, __builtin_return_address(0));
    }
    s_last_active_us = esp_timer_get_time();
}

void standby_notify_activity(void)
{

    standby_wake();
}

bool standby_is_deep_active(void)
{
    return s_deep_standby;
}

/**
 * @brief 进入深度待机（唯一低功耗档）：清情绪队列 + 关屏 + 三轴归中后停 PWM + 压音量
 */
static void enter_deep_standby(void)
{
    if (s_deep_standby)
        return;
    s_deep_standby = true;
    s_standby_enter_us = esp_timer_get_time();
    s_standby_enter_mv = 0;
    s_charge_base_mv = 0;
    s_charge_rise_hit = 0;
    ESP_LOGW(TAG, "无活动超过 %d ms，进入深度待机（关屏 + 停舵机，CPU 不睡，闹钟照常）",
             STANDBY_DEEP_TIMEOUT_MS);

    // 1) 回主页 + 关唤醒词 + 停震动马达（原一级省电动作前移到此）
    ui_force_back_to_main();                                   // 强制回归主页面
    wake_word_stop();                                          // 关闭唤醒词监听（仅切 is_running）
    ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // 停止震动马达 PWM（省电）

    interaction_set_lowpower(true); // 停止 interaction 层低功耗
    interaction_flush_queue();      // 清 interaction 层队列
    servo_manager_flush();          // 清 servo_manager 层队列 + 打断正在执行的插值动作

    // 2) ★先定格 GIF，再降亮度（顺序契约「先关 GIF 再线性降低亮度」）。定格瞬间画面静止、
    //    与当前帧无差别，此刻屏仍亮但用户无感；关键作用是掐断 GIF 排队解码对 CPU 的持续占用，
    //    让同优先级(5)的 standby_task 立刻能顺畅跑到下面的渐暗步骤，根治「渐暗延迟十几秒」。
    ui_pause_main_gif();

    // 3) 背光线性渐暗 100%→0（起点改为 DEFAULT，因不再有一级先降到 STANDBY 亮度），
    //    再关显示控制器。原子门闩把这段阻塞渐变包住，期间触摸被忽略（见 standby_wake 拦截）。
    atomic_store(&s_entering, true);
    bsp_board_lcd_fade_brightness(BSP_LCD_BK_DEFAULT_PCT, 0);
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
        ESP_LOGE(TAG, "进深度待机取 LVGL 锁超时，跳过关显示控制器（背光已渐暗，仅停舵机）");
    }

    // 4) 音量线性渐变压低（不写NVS），防低功耗期离线音频/提示音过大
    bsp_board_codec_enter_lowpower();

    // 5) 三轴归中 90° 后停 PWM（★修正版顺序，缺一步都会出实测 bug）：
    //    5a. 等 servo_manager_flush 的打断真正生效、worker 收尾归中做完（轮询 is_idle，3s 兜底）。
    //        ★不能省：flush 置的 abort 标志要几十 ms 才在 worker 插值循环边界被消费，
    //        若跳过此步直接清标志，会把标志抢在 worker 看到之前清掉 → flush 失效、动作继续跑。
    {
        int guard = 0;
        while (!servo_manager_is_idle() && guard < 60) // 60*50ms = 3s 上限兜底
        {
            vTaskDelay(pdMS_TO_TICKS(50));
            guard++;
        }
    }
    //    5b. 清打断标志。★头部不归中的根因：flush 置 abort=true 后若 worker 空闲则无人清，
    //        下面直调的 bsp_servo_move_all_parallel 每步都查该标志，首步即 break、归中零步退出
    //        （worker 内部归中前也先 clear，见 servo_manager.c:279 同款手法）。
    bsp_servo_clear_abort();
    //    5c. 兜底归中：worker 收尾已归中则三轴行程≈0 瞬间返回（无害）；worker 本来空闲的
    //        场景则这里真正把三轴带回 90°（阻塞插值，走到位才返回）。
    bsp_servo_move_all_parallel(HEAD_CENTER_DEG, HEAD_CENTER_DEG, HEAD_CENTER_DEG,
                                STANDBY_SERVO_CENTER_SPEED);
    //    5d. 短等 50ms 让最后一个 PWM 周期（50Hz=20ms）完整输出，再停三路 PWM 失力省电。
    vTaskDelay(pdMS_TO_TICKS(50));
    bsp_servo_idle();

    // 6) 捕获基准电压：此刻屏已关、舵机已停，负载恒定到最小，端电压接近真实 OCV。以此为基准，
    //    供三级关机前复查「深度待机期间电压是否明显上升」——上升即中途插了 USB，取消断电
    //    （见 enter_shutdown）。同时作为快/慢通道 USB 检测的干净基准（覆盖并清零命中，避免
    //    关屏停舵机负载骤降的正常回弹被误判成插USB陡升）。
    // 【诊断-E1~E2】进深度待机收尾打点：实测 standby_task 曾在此段后无声卡死（基准电压
    //   日志缺失、倒计时停打，2026-07-10），加打点定位。定位修复后删除本组打点。
    ESP_LOGW(TAG, "[E1] 舵机已停，开始3s等待后取基准电压");
    vTaskDelay(pdMS_TO_TICKS(3000)); // 延迟再进行电压监测，等回弹稳定
    s_deep_base_mv = bsp_battery_read_voltage_mv();
    ESP_LOGW(TAG, "[E2] 基准电压读取完成");
    ESP_LOGI(TAG, "深度待机基准电压=%lu mV（供三级复查是否中途插USB）", (unsigned long)s_deep_base_mv);
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
 * @brief 恢复震动马达 LEDC 通道（退出深度待机 / 关机前震动提醒用）
 *
 * enter_deep_standby 用 ledc_stop 停了马达 PWM，退出时（或关机前震动提醒时）须重新
 * config 通道，初始 duty=MAX（恒高电平 → 低有效断电，即停止态），后续震动才能正常驱动。
 * 关机前震动提醒复用本函数恢复通道后，再设 duty=0（低电平通电）驱动震动。
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
    if (!s_deep_standby)
        return;

    if (atomic_load(&s_entering))
    {
        ESP_LOGW(TAG, "进入待机流程进行中（背光渐暗/归中），本次触摸/唤醒直接忽略");
        return;
    }
    ESP_LOGI(TAG, "退出深度待机，恢复正常状态");
    // 删一级后低功耗只剩深度待机一档：开显示控制器（背光仍是0，不可见）→ 背光线性渐亮到
    // 100%（与 enter_deep_standby 的渐暗对称）→ 舵机恢复。
    {
        // ★开显示控制器必须持 LVGL 锁再调：走 SPI 给 ST7789 发 DISPON，与 LVGL 刷新任务
        //   共用同一 SPI panel 句柄。不加锁会与刷新事务抢 SPI，导致
        //   esp_lcd_panel_disp_on_off 失败或永久阻塞——这正是「打印了退出待机但屏仍黑、
        //   触摸任务卡死、后续触摸全无响应」的根因（与 application.c 开机开屏先加锁的约定一致）。
        //
        // ★★问题A 修复（偶发"退出低功耗但屏黑、舵机/震动却在动"）：开屏必须先于所有恢复动作，
        //   且【开屏失败绝不半恢复】。原实现取锁失败只打日志就继续跑 servo_resume/背光/震动，
        //   于是出现"屏没开成但舵机在动"。现改为：取锁重试 2 次（各 1000ms）；仍失败则【整段回滚】——
        //   保持 s_deep_standby 为 true（不清），直接 return，不碰舵机/背光/震动，
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
            // 两次都没取到锁：整段放弃本次唤醒，标志【保持不变】（仍是深度待机），不做任何
            // 舵机/背光/震动恢复，避免"屏黑但舵机动"的半恢复态。
            // 置位补偿标志，交给 standby_task 周期任务自动重试，不依赖用户再次触摸。
            ESP_LOGE(TAG, "[DBG2-FAIL] 退深度待机两次取 LVGL 锁均超时，放弃本次唤醒（保持待机，转交周期任务自动重试）");
            s_wake_retry_pending = true;
            return;
        }

        s_wake_retry_pending = false; // 开屏成功：清掉补偿标志，避免残留触发多余的周期任务重试

        // ★先恢复 GIF，再渐亮（与进待机「先关 GIF 再降亮度」对称的退出契约）。此刻 disp_on
        //   已成功但背光仍为 0（屏不可见），GIF 从定格帧继续播放的这段过渡用户完全看不到；
        //   等下面渐亮起来时画面已经在动，无「先亮起静止帧、再突然动」的突兀感。
        ui_resume_main_gif();

        // 【诊断-W1~W6】退深度待机逐步打点：实测出现"渐亮停在25%、触摸失效、标志不清"的
        //   无声卡死（2026-07-10），加打点定位卡在哪一步。定位修复后删除本组打点。
        ESP_LOGW(TAG, "[W1] 开始背光渐亮");
        bsp_board_lcd_fade_brightness(0, BSP_LCD_BK_DEFAULT_PCT);
        ESP_LOGW(TAG, "[W2] 渐亮完成，开始清队/清低功耗标志");

        interaction_flush_queue();       // 清 interaction 存量情绪，防 servo_resume 归中后被残留请求带跑
        interaction_set_lowpower(false); // 清低功耗标志：情绪播放恢复手臂+震动正常参与
        ESP_LOGW(TAG, "[W3] 清队完成，恢复马达通道");
        // 恢复震动马达 LEDC 通道到【停止态】。★退出低功耗不需要任何震动：若唤醒时正处于
        //   关机前震动提醒窗口（s_warn_vibrating=true、马达正 duty=0 震动中），这里必须显式
        //   停马达 + 清标志，否则马达停不干净、标志残留会影响下次判断。
        restore_motor_pwm();                                       // 重置通道，初始 duty=MAX（恒高=低有效断电，停止态）
        ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // 再显式停一次，确保退出静默无震动
        s_warn_vibrating = false;                                  // 清关机前震动提醒标志
        ESP_LOGW(TAG, "[W4] 马达已恢复停止态，开始恢复音量（读NVS，阻塞）");
        bsp_board_codec_exit_lowpower(); // 音量线性渐变恢复为用户设置值（阻塞，读 NVS）
        ESP_LOGW(TAG, "[W5] 音量恢复完成，开始舵机恢复");

        bsp_servo_resume(); // 恢复三路 PWM（直接写90°，无扫描归中，见 bsp_servo.c）；放最后，屏已亮
        ESP_LOGW(TAG, "[W6] 舵机恢复完成，清待机标志");

        s_deep_standby = false;

        if (lvgl_port_lock(1000))
        {
            ui_resume_main_gif_loop();
            lvgl_port_unlock();
        }
        else
        {
            // 取锁失败：不放弃，用 s_lp_gif_pending 轮询补偿——standby_task 每秒会检查一次
            // 并重试取锁恢复 GIF 循环（见 standby_task 3b 块）。避免独此一次尝试失败就永久
            // 卡住无 GIF 画面。恢复的是空闲队列的正常随机 GIF 轮播。
            ESP_LOGE(TAG, "退深度待机恢复GIF轮播取 LVGL 锁超时（背光/舵机已恢复），转交周期任务补偿重试");
            s_lp_gif_pending = true;
            s_lp_gif_pending_us = esp_timer_get_time() - STANDBY_LP_GATE_MS; // 立即到期，下一拍就重试
        }

        wake_word_start();
        return;
    }
}

/**
 * @brief 待机监测周期任务
 */
static void standby_task(void *arg)
{
    PRINT_TASK_STACK_HWM(TAG); // 打印本任务栈历史最小剩余

    ESP_LOGI(TAG, "待机监测任务启动（深度待机超时 %d ms）", STANDBY_DEEP_TIMEOUT_MS);
    touch_activity(); // 初始化活动时间，从启动开始计时

    while (1)
    {
        // ── 1. 会话活跃即视为活动（覆盖对话中 / TTS 播放）──────────────────
        if (session_get_state() != SESSION_IDLE)
            touch_activity();
        if (!s_deep_standby)
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

        // ── 2. 空闲超时直接进入深度待机（删一级后：关屏 + 停舵机，一步到位）────────
        if (!s_deep_standby && idle_ms >= STANDBY_DEEP_TIMEOUT_MS)
        {
            enter_deep_standby();
        }

        // ── 2a-log. ★两级低功耗倒计时打印（临时诊断：定位计时被谁重置）──────────
        //   每个轮询拍（1s）打印当前 idle 及距深度待机/关机各还差多少秒。
        //   若某一拍 idle 突然回落（如从 45000 掉回 0），说明 s_last_active_us 被刷新，
        //   紧接着 touch_activity 的 [计时被刷] 日志会指出是谁刷的。验证完删除本块。
        {
            int64_t to_deep = (int64_t)STANDBY_DEEP_TIMEOUT_MS - idle_ms;    // 距深度待机
            int64_t to_off = (int64_t)STANDBY_SHUTDOWN_TIMEOUT_MS - idle_ms; // 距三级关机
            ESP_LOGI(TAG, "[倒计时] idle=%lldms | 深度待机%s(余%lldms) 关机%s(余%lldms)",
                     idle_ms,
                     s_deep_standby ? "已进" : "未进", to_deep > 0 ? to_deep : 0,
                     s_shutdown_standby ? "已进" : "未进", to_off > 0 ? to_off : 0);
        }

        // ── 2b. 关机前震动提醒：深度待机中，距三级关机剩 WARN_MS 起震动 WARN_VIB_MS。──
        //   平时深度待机已停马达省电（enter_deep_standby 里 ledc_stop）；此处在临近关机的
        //   窗口内恢复马达通道并驱动震动，震满 WARN_VIB_MS 后停回，提醒用户"想留着就动一下"。
        //   屏保持黑，纯震动。s_warn_vibrating 防重复启/停。
        if (s_deep_standby && !s_shutdown_standby)
        {
            int64_t warn_start = (int64_t)STANDBY_SHUTDOWN_TIMEOUT_MS - STANDBY_SHUTDOWN_WARN_MS;
            int64_t warn_end = warn_start + STANDBY_SHUTDOWN_WARN_VIB_MS;
            bool in_window = (idle_ms >= warn_start && idle_ms < warn_end);
            if (in_window && !s_warn_vibrating)
            {
                s_warn_vibrating = true;
                restore_motor_pwm();                                           // 恢复马达 LEDC 通道（初始停止态）
                ledc_set_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 0); // duty=0 → 低有效马达通电震动
                ledc_update_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL);
                ESP_LOGW(TAG, "关机前震动提醒：距关机剩%dms，震动%dms",
                         STANDBY_SHUTDOWN_WARN_MS, STANDBY_SHUTDOWN_WARN_VIB_MS);
            }
            else if (!in_window && s_warn_vibrating)
            {
                s_warn_vibrating = false;
                ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // 停震动，回省电态
                ESP_LOGW(TAG, "关机前震动提醒结束，停马达");
            }
        }

        // ── 2c. 继续空闲到三级阈值 → 执行纯关机（硬件断电）
        if (s_deep_standby && !s_shutdown_standby && idle_ms >= STANDBY_SHUTDOWN_TIMEOUT_MS)
        {
            enter_shutdown();
        }

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

#if !STANDBY_DEBUG_ALLOW_USB_SLEEP
        if (s_deep_standby)
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

        // ── 3b. 退深度待机「恢复 GIF gate」到期检查：standby_wake 取 LVGL 锁恢复 GIF 循环
        //   失败时置位 s_lp_gif_pending，这里每拍重试取锁恢复空闲队列的正常随机 GIF 轮播。
        //   跨线程安全：仅本任务（standby_task 自身）读写 s_lp_gif_pending，不与 standby_wake
        //   的写产生竞态（standby_wake 只置位一次，这里判断到期后清位，先后顺序不敏感）。
        if (s_lp_gif_pending && (now_us - s_lp_gif_pending_us) / 1000 >= STANDBY_LP_GATE_MS)
        {
            s_lp_gif_pending = false;
            if (lvgl_port_lock(1000))
            {
                ui_resume_main_gif_loop(); // 切回正常随机 GIF 循环（空闲队列）
                lvgl_port_unlock();
            }
            else
            {
                ESP_LOGE(TAG, "退深度待机取 LVGL 锁超时，本次未能恢复GIF循环（下次活动会重试）");
            }
        }

        // ── 3c. 退深度待机「唤醒失败」自动补偿重试：见 s_wake_retry_pending 定义处说明。
        //   仅在仍处于深度待机时重试才有意义；一旦已不在深度待机（被其他路径唤醒）直接清标志。
        //   standby_wake() 内部幂等（!s_deep_standby 直接返回），可安全反复调用。
        if (s_wake_retry_pending)
        {
            if (!s_deep_standby)
            {
                s_wake_retry_pending = false; // 已不在深度待机（被别的路径唤醒），无需再重试
            }
            else
            {
                ESP_LOGW(TAG, "周期任务自动重试：重新尝试退出深度待机");
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
