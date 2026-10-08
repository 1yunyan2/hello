#include "standby.h"
#include "remote_control.h" // remote_control_cancel()：进深度待机时退出远程控制态并归中
#include "bsp/bsp_board.h"
#include "bsp/servo_manager.h"
#include "wake_word/custom_wake_word.h"
#include "session/session.h"
#include "ui/ui_port.h"
#include "ui/reminder.h"    // reminder_get_nearest_expire_sec()：进待机前的临期检查
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
#define STANDBY_DEEP_TIMEOUT_MS 60000     ///< 空闲超此值直接进深度待机（关屏+停舵机），占位可调
#define STANDBY_SHUTDOWN_TIMEOUT_MS 90000 ///< 空闲超此值执行三级关机（硬件断电），占位可调

/**
 * @brief 进深度待机前的「临期提醒」保护窗口（秒）
 *
 * 若最近一个倒计时将在本窗口内到期，则本轮【不进】深度待机，推迟到下一拍再判。
 *
 * 【为什么必须有】enter_deep_standby() 是一段长达 8~10 秒的阻塞流程（背光渐变
 * 1.7s + 舵机归中轮询上限 3s + 压音量读 NVS + 取基准电压延时 3s），而倒计时到期
 * 检测是 1 秒一拍。两者一旦重叠，到期唤醒会与仍在往下跑的进入流程并发抢背光/
 * 舵机/音量，实测表现为「屏幕只变暗、界面卡死」。与其事后补救，不如从源头
 * 避免两条长流程重叠——语义上也自洽："马上要提醒了就先别睡"。
 *
 * 【取值 15 秒的依据】必须大于 enter_deep_standby() 的最坏执行时长（约 10 秒），
 * 留 5 秒余量。调大只会让临期时多醒一会儿（本就要被提醒唤醒），无副作用。
 */
#define STANDBY_REMINDER_GUARD_SEC 15

#define STANDBY_SHUTDOWN_ENABLE 1       ///< 三级开关，插电测试，1=开启，0=关闭
#define STANDBY_PWR_RECHECK_RISE_MV 200 ///< 深度待机复查：相对基准电压上升≥此值(mV)判为中途插USB→取消关机
#define STANDBY_CHECK_MS 1000           ///< 周期任务检查间隔（1 秒）
/**
 * @brief standby_task 栈大小（字节，分配在【内部 SRAM】）
 *
 * 2026-07-31：栈从 SPIRAM 改到内部 SRAM（原放 SPIRAM 进深度待机必崩，
 * 见 standby_init 处注释），大小维持原值 4096 不变——内部 SRAM 余量紧张，
 * 不额外加码。若实测栈溢出，按 standby_task 里打印的水位再上调。
 */
/* 2026-08-20：4096 → 6144。进深度待机新增了 ui_standby_clock_show()，其中
 * ensure_menu_panel() 会触发 home_icon_cache_init() 逐个打开外挂 Flash 图标文件
 * （lv_fs_open/read）。实测本任务栈最深处只剩 1956 字节（见下方 [栈水位] 日志），
 * 再叠一条文件系统调用链余量不明，先加 2KB 观察实测水位再回调。 */
/* 2026-08-25：6144 → 8192。standby_task 新增了"唤醒完成后立即显示提醒页"的职责
 * （见 standby_task 循环开头消费 s_wake_pending 那段），会走
 * ui_show_countdown_expired / ui_show_alarm_ringing → render_fn_page →
 * countdown_page_show / alarm_page_rebuild 这条 UI 渲染链。
 * 实测本任务在走完 NVS 读取后仅剩 3588 字节（见 [栈水位] 日志），
 * 再叠一条 UI 渲染链余量不足，加 2KB。内部 SRAM 当时空闲约 52KB，代价可接受。 */
#define STANDBY_TASK_STACK 8192
#define STANDBY_DEBUG_ALLOW_USB_SLEEP 0 ///< 1=插USB也进低功耗（仅调试用）；0=正常拦截
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
/* 2026-08-31：关机前震动提醒的强度（%），0=停、100=最强。
 * 【为什么以前没有这个宏】旧代码直接写死 ledc_set_duty(..., 0)，而马达是【低有效】，
 *   duty=0 即"恒低电平常通"= 100% 满强度，等于无法调节。现在改为按百分比换算
 *   duty = BSP_MOTOR_DUTY_MAX - BSP_MOTOR_DUTY_MAX*本值/100，与 bsp_motor_set() 同源。
 * 【与 BSP_MOTOR_DEFAULT_STRENGTH 的区别】那条是触摸反馈的默认强度（100%），
 *   属于短脉冲；本条是持续 3 秒的长震，独立成宏以便单独调弱不影响触摸手感。 */
#define STANDBY_SHUTDOWN_WARN_STRENGTH 60 ///< 关机前震动提醒强度（%），0~100

#define STANDBY_CLOCK_BK_PCT 10 ///< 低功耗常亮时钟的背光亮度（%），占位可自调

// ─── 进/退低功耗的「三段式转场」时长（2026-08-20）──────────────────────────
//   【为什么要三段】渐变的是【背光】，画面是瞬间切换的。若在屏幕亮着时切画面，
//   看到的就是「啪一下换掉」；若先把画面藏了再渐暗，则是在一块已经全黑的屏上
//   渐暗，等于什么都看不见。两种都会丢失「渐变」的观感（实测两种都出现过）。
//   故统一改为：渐暗到全黑 → 【在全黑中换画面】→ 再渐亮到目标亮度。
//   每一段渐变期间屏幕上都有内容，渐变才可见；换画面藏在全黑里，用户无感。
//   附带好处：时间页首次渲染发生在背光=0 时，BUG-040 那条「整屏重绘逐带刷新」
//   的横带正好被全黑盖住（与功能盘进时间页现有的做法同理）。
//
//   ⚠ 这四段全部走 bsp_board_lcd_fade_brightness_fine()（伽马校正 + 每 10ms 一步），
//   不再用旧的 bsp_board_lcd_fade_brightness()（固定 16 步 = 每 125ms 才跳一档，
//   线性 duty 无伽马）——那正是实测「渐暗有断层、一格一格跳」的原因。
#define STANDBY_FADE_OUT_MS 1200 ///< 进低功耗：100% → 全黑 的渐暗时长（占位可调）
#define STANDBY_CLOCK_IN_MS 500  ///< 进低功耗：全黑 → 时钟亮度 的渐亮时长（占位可调）
#define STANDBY_CLOCK_OUT_MS 400 ///< 退低功耗：时钟亮度 → 全黑 的渐暗时长（占位可调）
#define STANDBY_FADE_IN_MS 1200  ///< 退低功耗：全黑 → 100% 的渐亮时长（占位可调）
/* ⚠【2026-08-31 已废弃，保留仅作历史说明，代码中不再引用】
 * 曾经的用途：关机前（震动提醒开始那一刻）把背光从 STANDBY_CLOCK_BK_PCT 渐暗到 0，
 *   做一段"要关机了"的黑屏预告。
 * 废弃原因：需求变更为「三级不熄屏，保持与二级一致的低亮度时钟显示」，
 *   熄屏调用已从 standby_task 的 2b 块移除。若日后要恢复黑屏预告，
 *   在 2b 块震动前重新加回 bsp_board_lcd_fade_brightness_fine(STANDBY_CLOCK_BK_PCT, 0, 本值) 即可。 */
#define STANDBY_SHUTDOWN_FADE_MS 400 ///< （已废弃）关机前熄屏渐暗时长

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

/* ★★【2026-08-25 新增】退出待机的「原子门闩」，与进入侧的 s_entering 完全对称。
 *
 * 【为什么必须有】standby_wake() 原先只靠开头 `if (!s_deep_standby) return;` 一道闸，
 *   而 s_deep_standby = false 是在整段转场【结束时】（[W6] 之后）才置的。
 *   中间那 2 秒多（时钟渐暗 400ms + 换 GIF + 渐亮 1200ms + 清队/马达/音量/舵机）
 *   它一直为 true，于是任何人再进来都会把整段转场【从头再跑一遍】。
 *   实测日志即两套流程完全交织：
 *       [W1] ... [W1] ... [W2][W3][W4] ... [W2][W3][W4]
 *   两套背光渐变互相打架 → 用户看到「屏幕闪烁然后才亮起」。
 *
 * 【为什么不能靠提前置 s_deep_standby=false 解决】那个标志是对外语义
 *   （standby_is_deep_active() 被 reminder/ui_port 大量读取，用于判断"要不要先亮屏"）。
 *   提前置假会让转场还没走完就对外宣称"已经醒了"，反而制造新的时序洞。
 *   故另起一个纯内部的门闩，只管"转场是否正在进行"，职责单一。
 *
 * 【与 s_entering 的关系】一进一出、两个方向各一把锁，语义完全对称：
 *   进入转场期间拒绝重复进入，退出转场期间拒绝重复退出。 */
static atomic_bool s_waking = ATOMIC_VAR_INIT(false);

/* ★【2026-08-25 新增】「待执行的唤醒请求」标志，由 standby_task 统一消费。
 *
 * 【两个来源共用本标志】
 *   ① standby_request_wake()：供【栈在 PSRAM 的任务】（reminder_task）使用的
 *      安全入口。那类任务绝不能自己调 standby_wake() —— 后者要读 NVS 恢复音量，
 *      而关 flash cache 期间 PSRAM 栈会失联，IDF 直接断言 abort
 *      （esp_task_stack_is_sane_cache_disabled @ cache_utils.c:152，实测已复现）。
 *   ② standby_wake() 自身撞上 s_entering 门闩时的自我挂起（见下）。
 *   两者语义一致——"稍后由 standby_task 执行唤醒"，故合用一个标志，
 *   消费点也合并在 standby_task 循环开头 + enter_deep_standby() 末尾两处。
 *
 * 【为什么需要它】原先 standby_wake() 撞上 s_entering 就【静默丢弃】本次唤醒。
 *   触摸场景下无所谓（用户再摸一下即可），但【倒计时/闹钟到期】是一次性事件：
 *   reminder 每秒轮询一次，到期那一拍若正好落在转场窗口里，唤醒就永久丢失，
 *   而 ui_show_countdown_expired() 仍会继续切页 → 最终留下
 *   「s_deep_standby=true + 视图停在功能页 + 背光 2% + 待机时钟与功能页同框」
 *   的死状态，即实测反馈的"屏幕只变暗、卡死"。
 *
 * 【为何用 atomic_bool】置位方来自任意任务（reminder_task / 触摸任务 / MQTT），
 *   消费方是 standby_task 自己，跨任务读写必须原子。
 *
 * 【消费时机很关键】见 enter_deep_standby() 末尾处注释：必须等整个进入流程
 *   （含压音量、舵机归中停 PWM、取基准电压）全部走完才消费，绝不能在
 *   s_entering 清零后立刻消费。 */
static atomic_bool s_wake_pending = ATOMIC_VAR_INIT(false);

/* 本次唤醒请求的来源（standby_wake_src_t）。决定 standby_task 唤醒完成后是否
 * 立刻把提醒页画上——详见 standby.h 中 standby_wake_src_t 的说明。
 * 用 atomic_int 而非枚举变量：跨任务读写必须原子。 */
static atomic_int s_wake_src = ATOMIC_VAR_INIT((int)STANDBY_WAKE_SRC_GENERIC);
static volatile uint32_t s_deep_base_mv = 0; // 进深度待机(负载已恒定至最小)的基准电压 mV，供三级复查比对
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

// ─── 退二级「唤醒失败」补偿状态【2026-08-20 已删除】────────────────────────
// 原 s_wake_retry_pending 标志用于补偿 standby_wake() 里 disp_on 两次取 LVGL 锁
// 均失败、整段回滚不唤醒的情形。低功耗终点改为「常亮时钟」后，进待机不再
// disp_off，退待机也就没有 disp_on 可开，那段回滚逻辑与本标志一并删除。

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
    // ★ 两件事缺一不可（2026-07-31 补回 touch_activity）：
    //   1) touch_activity()：刷新空闲计时。【未待机时这是本函数唯一有效动作】——
    //      standby_wake() 开头就是 `if (!s_deep_standby) return;`，正常运行态下它
    //      立刻返回什么都不做。此前漏调导致唤醒/触摸/MQTT 远程指令在非待机状态下
    //      全都刷不到倒计时，表现为「一直在用设备，它照样到点就关屏」。
    //   2) standby_wake()：若当前已在深度待机，则一并唤醒（幂等，非待机直接返回）。
    touch_activity();
    standby_wake();
}

bool standby_is_deep_active(void)
{
    return s_deep_standby;
}

/**
 * @brief 请求退出待机（异步，PSRAM 栈任务专用安全入口）
 *
 * 实现刻意保持到「只有一次原子写」这一步：不碰 flash / NVS / LVGL，不阻塞，
 * 因此在任何任务上下文调用都安全。真正的唤醒由 standby_task 下一拍执行。
 *
 * 详细背景见 standby.h 中的接口注释，以及 standby_wake() 上方的栈约束警告。
 */
void standby_request_wake(standby_wake_src_t src)
{
    /* 来源先写、标志后写：standby_task 是先读标志再读来源，反序会读到旧来源。
     * 多个来源同时请求时（闹钟与倒计时同一秒到期）后写者胜出——闹钟段在
     * poll_timer_callback 里排在倒计时段之后，故闹钟自然优先，符合预期。 */
    atomic_store(&s_wake_src, (int)src);
    atomic_store(&s_wake_pending, true);
}

/**
 * @brief 进入深度待机（唯一低功耗档）：清情绪队列 + 关屏 + 三轴归中后停 PWM + 压音量
 */
static void enter_deep_standby(void)
{
    if (s_deep_standby)
        return;

    /* ★★【2026-08-25：UI 复位提到最前，且失败就整轮放弃 —— 进入流程的原子性前提】★★
     *
     * 【修的问题】原先 ui_force_back_to_main() 排在下面第 1 步，且它取锁失败时只是
     *   静默 return void。于是出现过这样的半成品状态：界面【没】复位（比如闹钟响铃
     *   结束后停在闹钟编辑页 s_edit_panel），而本函数照常往下走，把待机时钟叠上去
     *   —— 实测现象＝「闹钟结束界面 & 低功耗时间显示界面同框」。
     *
     * 【改法】① 提到 s_deep_standby = true 【之前】——这样失败时直接 return，
     *   没有任何状态被改过，不需要回滚，下一拍（1 秒后）自然重试；
     *   ② 检查返回值，复位不成功就【绝不进待机】。宁可这一轮不省电（最多多亮 1 秒，
     *   下一拍就会再试），也不能留下"半进半出"的叠加界面 —— 那是用户看得见的故障，
     *   而晚 1 秒待机用户根本感知不到。
     *
     * 【为什么必须排在 remote_control_cancel 之前】后者会异步归中舵机，属于"已经
     *   动了硬件"。若放在它后面才判失败返回，舵机已白归中一次（虽幂等但多一次抽动）。
     *   现在整个函数的第一个副作用就是 UI 复位，失败即退，干净。 */
    if (!ui_force_back_to_main())
    {
        ESP_LOGW(TAG, "进待机前 UI 复位失败（LVGL 锁忙），本轮放弃进入待机，下一拍重试");
        return;
    }

    s_deep_standby = true;
    s_standby_enter_us = esp_timer_get_time();
    s_standby_enter_mv = 0;
    s_charge_base_mv = 0;
    s_charge_rise_hit = 0;
    ESP_LOGW(TAG, "无活动超过 %d ms，进入深度待机（关屏 + 停舵机，CPU 不睡，闹钟照常）",
             STANDBY_DEEP_TIMEOUT_MS);

    // 退出 app 远程控制的冻结窗口。center_servo=true（归中）：深度待机本就要求三轴
    // 归位后再停 PWM，且此后没有任何动作会接管舵机，故必须显式归中。
    // 注意：归中由 rc_worker 异步执行，与下面 enter_deep_standby 自身的归中互不冲突
    // （bsp_servo 每通道有独立 mutex，且目标角同为 90°，先后到达结果一致）。
    remote_control_cancel(/*center_servo=*/true);

    // 1) 关唤醒词 + 停震动马达（UI 复位已在函数最前完成，见上方原子性说明）
    wake_word_stop();                                          // 关闭唤醒词监听（仅切 is_running）
    ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1); // 停止震动马达 PWM（省电）

    interaction_set_lowpower(true); // 停止 interaction 层低功耗
    interaction_flush_queue();      // 清 interaction 层队列
    servo_manager_flush();          // 清 servo_manager 层队列 + 打断正在执行的插值动作

    // 2) ★先定格 GIF，再降亮度（顺序契约「先关 GIF 再线性降低亮度」）。定格瞬间画面静止、
    //    与当前帧无差别，此刻屏仍亮但用户无感；关键作用是掐断 GIF 排队解码对 CPU 的持续占用，
    //    让同优先级(5)的 standby_task 立刻能顺畅跑到下面的渐暗步骤，根治「渐暗延迟十几秒」。
    ui_pause_main_gif();

    // 3) ★三段式转场：渐暗到全黑 → 全黑中换成时钟 → 再渐亮到时钟亮度。
    //    原子门闩把整段阻塞转场包住，期间触摸被忽略（见 standby_wake 拦截）。
    //
    //    ★原先渐暗后还会 bsp_board_lcd_disp_off() 关显示控制器（给 ST7789 发
    //    DISPOFF，与背光是两套独立开关），现已【整段删除】：屏要常驻显示时间，
    //    画面输出必须保持开启，否则只会得到一块亮着却什么都没有的板子。
    //    连带 standby_wake() 里配对的 disp_on 双重试+整段回滚也一并删除（见那里）。
    atomic_store(&s_entering, true);

    //    3a. 渐暗到全黑。此刻 GIF 仍留在屏上（ui_pause_main_gif 只定格不隐藏），
    //        用户能【看着它慢慢暗下去】——这正是以前那个渐灭观感的来源。
    //        若像上一版那样先把 GIF 藏了再渐暗，就是在一块已经全黑的屏上渐暗，
    //        什么都看不见，表现为「GIF 啪一下没了」。
    bsp_board_lcd_fade_brightness_fine(BSP_LCD_BK_DEFAULT_PCT, 0, STANDBY_FADE_OUT_MS);

    //    3b. 全黑中换画面：隐藏定格的 GIF，把功能盘现成的时间页叠上来。
    //        背光此刻为 0，整个切换过程（含时间页 215px 大字的整屏重绘，实测
    //        DARK 256ms、逐带刷新可见——BUG-040）全部被黑屏盖住，用户无感。
    ui_standby_clock_show();

    //    3c. 从全黑渐亮到时钟亮度，时钟慢慢浮现。
    bsp_board_lcd_fade_brightness_fine(0, STANDBY_CLOCK_BK_PCT, STANDBY_CLOCK_IN_MS);

    atomic_store(&s_entering, false);

    // 4) 音量线性渐变压低（不写NVS），防低功耗期离线音频/提示音过大
    bsp_board_codec_enter_lowpower();

    /* 【调栈用·2026-07-31】上一行内部走 nvs_get_i32 → esp_flash_read，是本任务栈
     * 最深的一段（也是栈放 SPIRAM 时必崩的那处）。此处打印水位，用于把
     * STANDBY_TASK_STACK 从暂定的 50KB 下调到实测值 + 余量。确定后可删本行。 */
    ESP_LOGW(TAG, "[栈水位] 走完 NVS 读取后，standby 任务栈剩余 %u 字节（当前分配 %d）",
             (unsigned)(uxTaskGetStackHighWaterMark(NULL) * sizeof(StackType_t)),
             STANDBY_TASK_STACK);

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
    bsp_servo_move_all_parallel(HEAD_CENTER_DEG, ARM_CENTER_DEG, ARM_CENTER_DEG,
                                STANDBY_SERVO_CENTER_SPEED); // 头90°/臂15°（2026-09-16 头臂分离）
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

    /* ★★【2026-08-25 新增：补偿转场期间被挂起的唤醒请求】★★
     *
     * 【为什么消费点必须放在函数【最末尾】，而不是 s_entering 清零那一刻】
     *   s_entering 只包住"背光渐暗 → 换时钟 → 渐亮"这约 2 秒，而它清零之后
     *   本函数还有三大步没做完：压音量（读 NVS，阻塞）、舵机归中并停 PWM、
     *   延时 3 秒取基准电压。若在 s_entering 清零后立刻 standby_wake()，
     *   两条流程就会【并发抢同一批外设】：
     *     · wake 刚 bsp_servo_resume() 恢复 PWM，本函数紧接着 bsp_servo_idle() 停掉；
     *     · wake 刚 codec_exit_lowpower()，本函数紧接着 codec_enter_lowpower()；
     *   —— 那正是本次要修的"进入流程非原子"病灶本身，绝不能自己再制造一次。
     *   等整个进入流程走完再唤醒，进/退两条链路首尾相接、状态完全对称。
     *
     * 【代价】到期唤醒最多多等约 6 秒（3s 舵机等待上限 + 3s 取基准电压延时）。
     *   相比"卡死在暗屏"，这个延迟完全可接受；且 standby_task 里的"临期不进
     *   待机"主防线会让绝大多数到期根本走不到这条兜底路径上来。
     *
     * 【幂等】standby_wake() 开头会判 !s_deep_standby 直接返回，重复调用无害。 */
    if (atomic_exchange(&s_wake_pending, false))
    {
        /* ★【2026-08-25 补：必须判"当前确实还在待机"才补偿】★
         * 【原来的毛病】这里无条件调 standby_wake()。而挂起标志的置位方（触摸路径）
         *   在 ui_dispatch_touch_event 里【已经把那次触摸当作"已唤醒"消费掉了】
         *   （was_standby 分支打印"触摸唤醒，退出待机"后直接 return），
         *   标志却没人清。于是几秒后用户再摸一次正常唤醒时，本处又拿着这个陈旧标志
         *   补跑一次完整转场 —— 两套 [W1]→[W6] 交织，屏幕闪烁。
         * 【改法】补偿的前提是"现在真的还睡着"。若期间已被别的路径唤醒，
         *   这个请求的目的早已达成，标志消费掉即可，不必再跑一遍。
         *   与 standby_wake() 内新增的 s_waking 门闩形成双保险。 */
        if (s_deep_standby)
        {
            ESP_LOGW(TAG, "转场期间有唤醒请求被挂起，进入流程已走完，立即补偿唤醒");
            standby_wake();
        }
        else
        {
            ESP_LOGW(TAG, "转场期间的唤醒请求已由其它路径完成，丢弃陈旧标志不重复唤醒");
        }
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

    // 1. 确保二级待机（停舵机等）已执行，避免断电瞬间舵机抖动。
    //    ★注意：正常路径下到这里 s_deep_standby 已为 true，本调用【幂等直接 return】，
    //    不会再动背光——熄屏必须靠下面第 1b 步自己做。
    enter_deep_standby();

    // 1b. ★兜底熄屏（2026-08-31）：正常路径下屏幕早在【震动提醒开始那一刻】就已熄灭
    //     （见 standby_task 的 2b 块），此处是兜底——覆盖那些没经过震动窗口就直接
    //     走到关机的路径（例如刚进二级就被外部直接推到三级、或震动窗口被跳过）。
    //     幂等：屏已黑时再写一次 duty=0 无副作用。传 0ms 走瞬时分支，不拖慢断电。
    bsp_board_lcd_fade_brightness_fine(STANDBY_CLOCK_BK_PCT, 0, 0);

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
        /* ★【2026-08-25 改：不再静默丢弃，改为挂起等转场结束补偿】★
         * 原先这里直接 return，触摸场景无害（用户再摸一下即可），但到期提醒是
         * 一次性事件，丢了就永久丢了，会留下"只变暗、卡死"的死状态。
         * 详见 s_wake_pending 定义处的完整说明。 */
        atomic_store(&s_wake_pending, true);
        ESP_LOGW(TAG, "进入待机转场进行中（背光渐暗/归中），本次唤醒已挂起，转场结束后补偿");
        return;
    }

    /* ★★【2026-08-25 新增：退出侧原子门闩，防止两套转场交织】★★
     * 上面 !s_deep_standby 那道闸拦不住重入 —— 该标志要等本函数最末尾
     * （[W6] 之后）才置 false，中间 2 秒多的转场期内它一直为 true。
     * 实测两次唤醒完全重叠、两套背光渐变打架，表现为「屏幕闪烁后才亮起」。
     * 这里用 test-and-set 一次性拦下：已有转场在跑就直接返回，不排队也不补偿
     * ——因为正在跑的那一次本来就会把设备唤醒，目的已经达成，重复跑纯属有害。
     * 与进入侧 s_entering 完全对称。 */
    if (atomic_exchange(&s_waking, true))
    {
        ESP_LOGW(TAG, "退出待机转场已在进行中，忽略本次重复唤醒请求");
        return;
    }

    ESP_LOGI(TAG, "退出深度待机，恢复正常状态");
    // 删一级后低功耗只剩深度待机一档：收起待机时钟 → 恢复 GIF → 背光从
    // STANDBY_CLOCK_BK_PCT 线性渐亮回 100%（与 enter_deep_standby 的渐暗对称）→ 舵机恢复。
    {
        // ★★2026-08-20：原先此处有一大段「开显示控制器（disp_on）」逻辑——取 LVGL 锁
        //   重试 2 次、两次都失败就整段回滚保持待机、并置 s_wake_retry_pending 交给
        //   standby_task 周期重试。它存在的唯一理由是 enter_deep_standby 里配对的
        //   disp_off。现在低功耗改为「屏留 STANDBY_CLOCK_BK_PCT 常驻显示时间」，
        //   进待机不再关显示控制器，这里自然【没有对象可开】，整段删除。
        //   留着它有害无益：既白占一次取锁，失败时还会整段 return 不唤醒，
        //   凭空制造一个「醒不来」的故障点。s_wake_retry_pending 随之不再有人置位。

        // ★三段式转场（与进待机对称）：时钟渐暗到全黑 → 全黑中换回 GIF → 再渐亮到 100%。
        //   分工：ui_standby_clock_hide 负责【收时钟 + 把 gif_obj unhide】，
        //        ui_resume_main_gif 负责【从定格帧恢复播放 + 恢复轮播排队】，两者不重叠。
        //
        //   ⚠ 不能像上一版那样「先 unhide GIF、再从时钟亮度直接渐亮到 100%」：那样
        //   GIF 会在背光尚为 STANDBY_CLOCK_BK_PCT 时【瞬间弹出】，而背光的感知亮度
        //   是伽马非线性的（低占空比看着已经不算暗），观感就是「啪一下亮了」，
        //   后面那段渐亮反而不明显——实测反馈的「退出渐亮丢失」正是这么来的。

        // 【诊断-W1~W6】退深度待机逐步打点：实测出现"渐亮停在25%、触摸失效、标志不清"的
        //   无声卡死（2026-07-10），加打点定位卡在哪一步。定位修复后删除本组打点。
        ESP_LOGW(TAG, "[W1] 开始退出转场（时钟渐暗→换GIF→渐亮）");

        // 1) 时钟渐暗到全黑（屏上是时钟，用户能看着它暗下去）
        bsp_board_lcd_fade_brightness_fine(STANDBY_CLOCK_BK_PCT, 0, STANDBY_CLOCK_OUT_MS);
        // 2) 全黑中换回 GIF（收时钟 + unhide + 从定格帧恢复播放），用户无感
        ui_standby_clock_hide();

        /* ★★【2026-08-25 新增：唤醒时必须把 UI 视图一并复位】★★
         * 【修的问题】本函数原先只恢复"画面"，不碰 s_view。而低功耗期间视图是
         *   【可能被改掉】的：ui_show_countdown_expired() / ui_show_alarm_ringing()
         *   到期时会把 s_view 强制切成 UI_VIEW_FUNCTION_MENU。于是唤醒后出现
         *   "画面看着是主界面（GIF 露出来了），内部却仍是功能页"的错位：
         *     · ui_resume_main_gif_loop() 判 s_view != MAIN 直接早退 → GIF 轮播永不恢复；
         *     · 下一次触摸走 UI_VIEW_FUNCTION_MENU 分支，主界面的"短按耳进功能盘"
         *       根本走不到 → 实测反馈的"退出之后无法进入功能盘，再摸就卡死"。
         * 【为何复用 ui_force_back_to_main】它已经包办了这里需要的全套：隐藏
         *   s_edit_panel / s_menu_panel、unhide gif_obj、s_view = MAIN、
         *   main_gif_kick_resume()、取消倒计时延迟退出与退出渐变、停功能层空闲
         *   计时器。且它自带 `s_view == MAIN 直接返回` 的幂等早退，正常唤醒
         *   （没被切过视图）时等同空操作，零副作用。
         * 【为何夹在 hide 与 resume 之间】① 必须在 hide 之后：hide 要先收走时间页；
         *   ② 必须在 ui_resume_main_gif() 之前：本函数内部会 main_gif_kick_resume()
         *   重排轮播，而 ui_resume_main_gif() 负责从定格帧恢复播放，顺序颠倒会打架；
         *   ③ 此刻背光为 0，整个复位过程藏在全黑里，用户完全无感。 */
        ui_force_back_to_main();

        ui_resume_main_gif();
        // 3) 从全黑渐亮到 100%，GIF 慢慢亮起来（复用与开机/进待机同一条丝滑渐变）
        bsp_board_lcd_fade_brightness_fine(0, BSP_LCD_BK_DEFAULT_PCT, STANDBY_FADE_IN_MS);
        ESP_LOGW(TAG, "[W2] 渐亮完成，开始清队/清低功耗标志");

        interaction_flush_queue();       // 清 interaction 存量情绪，防 servo_resume 归中后被残留请求带跑
        interaction_set_lowpower(false); // 清低功耗标志：情绪播放恢复手臂+震动正常参与
        ESP_LOGW(TAG, "[W3] 清队完成，恢复马达通道");
        /* 恢复震动马达 LEDC 通道到【停止态，但通道保持使能】。
         * 若唤醒时正处于关机前震动提醒窗口（s_warn_vibrating=true、马达正在震），
         * 这里必须把马达停下 + 清标志，否则马达停不干净、标志残留会影响下次判断。
         *
         * ★★【2026-08-31 关键修改：去掉这里的 ledc_stop】★★
         * 【原来的写法】restore_motor_pwm() 之后又补一句
         *     ledc_stop(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, 1);
         *   注释理由是"再显式停一次，确保退出静默无震动"。
         * 【为什么必须删】ledc_stop() 会把【通道整个禁用】并把管脚钉死在 idle_level=1，
         *   此后 ledc_set_duty()/ledc_update_duty() 写进去的值不会再输出到管脚 ——
         *   也就是说 bsp_motor_pulse() 从此【彻底失效】，直到有人再 config 一次通道。
         *   本轮新增的"退低功耗震动一下"正是走 bsp_motor_pulse()，若保留这句，
         *   震动会【静默失败】（日志照打、手上没感觉），极难排查。
         * 【删掉安全吗】安全，且电气行为完全等价：restore_motor_pwm() 已把通道
         *   config 成 duty = BSP_MOTOR_DUTY_MAX = 恒高电平，而马达是【低有效】，
         *   高电平 = 断电 = 不震。区别只在"通道使能着输出恒高"vs"通道禁用钉死在高"，
         *   管脚电平一样，马达一样不动，但前者保留了后续 set_duty 的可用性。
         * 【省电影响】可忽略：LEDC 通道使能本身不驱动负载，真正的耗电在马达上，
         *   而马达此刻是断电的；何况此时已退出低功耗、回到正常运行态。 */
        restore_motor_pwm();      // 重置通道，duty=MAX（恒高=低有效断电，停止态），通道保持使能
        s_warn_vibrating = false; // 清关机前震动提醒标志
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

        /* ★【2026-08-31】清触摸按键的「持续按压闩锁」。
         * 【修的问题】用某个触摸位置唤醒待机后，正是那一个位置随后按不动，
         *   其余位置立刻摸都正常（用户实测指纹）。
         * 【原因】唤醒那次触摸在触摸层正常发出了事件并置了 consumed 闩锁，
         *   而 ui_dispatch_touch_event 判 was_standby 后把事件直接丢弃了——
         *   事件没了，闩锁却留着。加之本函数整段转场约 2.6 秒全程阻塞、且就跑在
         *   触摸任务栈上，这期间扫描一帧未跑，"连续离手帧数"累不够，闩锁清不掉。
         * 【为何放这里】必须在转场全部结束之后：此刻扫描任务即将恢复运行，
         *   把停摆期间的过期按键状态整体丢弃重建，语义正确。
         *   纯内存赋值、不阻塞不读 NVS，不违反本函数所在任务栈的约束。 */
        bsp_touch_clear_latch();

        /* ★【2026-08-25】释放退出门闩：转场全部走完，允许下一次唤醒。
         * ⚠️ 必须放在下面这个 return 之【前】——本函数是「if(...){...return;} }」结构，
         * 漏掉就会让门闩永久停在 true，此后【再也无法退出待机】，
         * 那比原来的重入问题严重得多。上方所有早退分支都在置位之前，无需各自释放。 */
        atomic_store(&s_waking, false);
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
        /* ── 0. 消费「异步唤醒请求」（2026-08-25 新增）────────────────────────
         * 两个来源共用同一个标志：
         *   ① standby_request_wake()：栈在 PSRAM 的任务（reminder_task）唯一能用的
         *      安全入口 —— 它自己不能调 standby_wake()（内部读 NVS 要关 flash cache，
         *      而关 cache 后 PSRAM 栈失联，IDF 会断言 abort）；
         *   ② standby_wake() 撞上 s_entering 门闩时的自我挂起。
         *
         * ★ 由本任务执行唤醒是安全的：standby_task 的栈显式分配在【内部 SRAM】
         *   （见 standby_init 的 MALLOC_CAP_INTERNAL），关 cache 期间栈依然可读。
         *
         * 放在循环最前：请求最迟 1 拍（STANDBY_CHECK_MS=1s）内被处理，
         * 让"到期先亮屏"的等待尽可能短。standby_wake 自带非待机早退，幂等。 */
        if (atomic_exchange(&s_wake_pending, false))
        {
            /* 先读标志后读来源，与 standby_request_wake 的"先写来源后写标志"配对 */
            standby_wake_src_t src = (standby_wake_src_t)atomic_exchange(
                &s_wake_src, (int)STANDBY_WAKE_SRC_GENERIC);

            if (s_deep_standby)
            {
                ESP_LOGW(TAG, "收到异步唤醒请求（来源=%d），执行退出待机", (int)src);
                standby_wake();
            }

            /* ★★【2026-08-25：唤醒完成后立刻把提醒页画上，不给 GIF 抢屏的机会】★★
             *
             * 【修的问题】原先只负责"醒"，画面交给 reminder 下一拍投递事件时再切。
             *   实测这中间隔了唤醒转场 2.6s + 一拍轮询，而 standby_wake() 收尾会
             *   ui_resume_main_gif_loop() 恢复 GIF 轮播并立即切图（重活）：
             *       922.249 [W6] 舵机恢复完成       ← 唤醒收尾
             *       922.339 timer_cb: 切图 idx=20   ← GIF 开始切图，占住 LVGL 线程
             *       922.989 >>> 闹钟 #0 触发 <<<    ← 此时才去切页
             *   而 ui_show_alarm_ringing() 抢锁只等 100ms，抢不过刚起跑的切图，
             *   直接放弃（日志里因此【没有】"已强制切到闹钟页"那条 INFO）——
             *   用户看到的就是"闹钟在响，屏幕却是待机 GIF 界面"。
             *
             * 【改法】唤醒一完成就在本任务里直接画。此刻我们紧挨着 standby_wake()
             *   返回，GIF 那一拍切图刚起步或还没起步，抢锁成功率高得多；即便这次
             *   没抢到，reminder 下一拍投递事件时还会再调一次，属于双保险。
             *
             * 【幂等】两个 UI 函数都可重复调用：ui_show_alarm_ringing 开头有
             *   "已在响铃态且已停在闹钟页就直接返回"的早退；ui_show_countdown_expired
             *   重复调用只是重画一次到期画面。
             *
             * 【栈】本任务栈在内部 SRAM 且已加到 8192（见 STANDBY_TASK_STACK 注释），
             *   足够走 render_fn_page → countdown_page_show / alarm_page_rebuild 这条链。 */
            if (src == STANDBY_WAKE_SRC_ALARM_RING)
            {
                ESP_LOGW(TAG, "唤醒来源=闹钟响铃，立即显示响铃画面");
                ui_show_alarm_ringing();
            }
            else if (src == STANDBY_WAKE_SRC_COUNTDOWN_EXPIRE)
            {
                ESP_LOGW(TAG, "唤醒来源=倒计时到期，立即显示到期画面");
                ui_show_countdown_expired();
            }
        }

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

        // ── 1c. app 远程控制冻结窗口内：持续刷新活动时间，不进低功耗 ──────────────
        //   冻结期（默认 30s）的语义是「画面与姿态属于用户刚下发的设定」，设备正在
        //   展示用户的操作结果，属于有意义的运行状态而非无人值守。若此期间照常累计
        //   空闲时长，会出现「刚在 app 上摆好姿势，没几秒就关屏」——尤其 STANDBY_DEEP_TIMEOUT_MS
        //   (40s) 与冻结窗口(30s) 接近时，冻结一结束几乎立刻待机，体验割裂。
        //   手法与上面「插着 USB 就刷新」一致：只喂时间戳，不碰任何状态机。
        //   冻结结束（30s 到期或被触摸/唤醒打断）后本条自然失效，重新开始正常计时。
        if (remote_control_is_active())
            touch_activity();

        /* ══════════════════════════════════════════════════════════════════════
         * ★★ 1d. 活动源总闸：待机优先级最低，任何任务在跑都不许计时 ★★
         *        （2026-08-25 按「待机是所有功能里优先级最低的那个」重新收口）
         *
         * 【总原则】只要设备正在做【任何一件用户可感知的事】，就不能算"无人值守"：
         *     · 对话中（LISTENING / PLAYING）        → 见上方步骤 1
         *     · 停留在功能盘 / 功能页 / 闹钟编辑 / 游戏 → 本条视图判断
         *     · 闹钟响铃中 / 倒计时到期提醒中         → 本条 reminder 判断
         *     · app 远程控制冻结窗口                  → 见上方 1c
         *     · 插着 USB                              → 见上方 1b
         *     · 任意触摸                              → 事件驱动，见 ui_dispatch_touch_event
         *   以上任一成立就把空闲计时喂满，待机只在【全部都不成立】时才推进。
         *
         * 【为什么全部集中在 standby_task 里逐拍轮询，而不是在各模块里到处调
         *   standby_notify_activity()】后者要在几十个出入口逐个补调用，漏一个就复现，
         *   而且【停留期间】仍会累计（只有"进入/退出"那一刻被刷新）。逐拍轮询是
         *   "只要还在这个状态就一直刷"，一处生效、覆盖所有进出路径，也天然包含了
         *   "退出即重置"的效果（退出那一刻 idle 刚被喂过，从 0 重新算 60 秒）。
         *   ——本项目已有的 1b/1c 就是这个手法，本条只是把它补全到所有活动源。
         * ══════════════════════════════════════════════════════════════════════ */

        /* 1d-①【提醒进行中】闹钟正在响 / 倒计时刚到期正在提醒（RINGING / NOTIFYING）。
         * 【修的问题】实测日志里，倒计时到期→震动 3 秒→到期画面停留 5 秒，全程
         *   没有任何一步刷新 idle：
         *       I (968959) 倒计时 #0 到期        ← 此时 idle 已 48.8s
         *       I (973979) 番茄时钟到期提示结束
         *       W (980469) 无活动超过 60000ms，进入深度待机   ← 画面刚看完就黑屏
         *   提醒正在发生却被判为"无人值守"，显然违背待机最低优先级的原则。 */
        if (reminder_get_state() != REMINDER_STATE_IDLE)
            touch_activity();

        /* 1d-②【停留在功能层】非主界面即视为用户正在使用。
         *
         * 【修的问题】standby 的空闲计时原先只由「触摸事件」刷新
         *   （ui_dispatch_touch_event 开头的 standby_notify_activity）。用户进了功能盘
         *   之后如果只是【停留查看】而不再触摸——看天气页、看日历、盯着番茄钟设定——
         *   计时照常累加。实测日志：
         *       [倒计时] idle=38770ms  ← 功能层里什么都没干，已经空转了 38 秒
         *       功能层空闲超时，返回主界面
         *   回到主界面时 idle 已逼近 60s 阈值，用户刚被踢回主界面没几秒屏就黑了，
         *   而且这正是「进待机流程」与「倒计时到期」两条长流程容易撞车的时间窗。
         *
         * 【为什么"停留在功能层"不该算无人值守】用户明确进入了某个功能页，屏幕上
         *   显示的是他要看的内容，这与主界面挂机是两回事。语义上等同于 1c 的
         *   「远程控制冻结窗口」——设备正在展示用户要的结果。
         *
         * 【为什么放在 standby_task 里逐拍刷新，而不是"退出功能盘时重置一次"】
         *   后者要在 menu_idle_timeout_cb / ui_func_layer_exit_to_main /
         *   ui_force_back_to_main / back_to_home / 游戏退出 等每一个出口都补一次调用，
         *   漏掉任意一个就复现；而且功能层【停留期间】本身仍在累计，停久了照样进待机。
         *   逐拍刷新一处生效、覆盖所有进出路径，且天然包含了"退出即重置"的效果
         *   （退出那一刻 idle 刚被喂过，等于从 0 开始重新算 60 秒）。
         *
         * 【为什么四个非主界面视图都算】HOME/FUNCTION_MENU/ALARM_EDIT 都有 20s 功能层
         *   空闲超时兜底，停留最长 20 秒就会被自动踢回主界面，不会无限期不待机；
         *   UI_VIEW_GAME 虽然显式取消了那个超时（游戏中长时间无翻页属正常），
         *   但游戏进行中本就不该关屏，刷新正是期望行为。
         *
         * 【残留风险与已有对策】若视图因异常卡在非主界面，本条会让设备永不进低功耗。
         *   这正是本轮同步在修的问题（standby_wake 中补 ui_force_back_to_main 复位视图），
         *   且用户任意一次触摸都能让功能层回到主界面，不构成死局。
         * 【为什么必须带 !s_deep_standby 前提】两个理由：
         *   ① 已进待机后，idle 计时的用途变成「累计到三级关机阈值」，此时刷新会
         *      让设备永远关不了机；
         *   ② touch_activity() 内部有 `if (s_deep_standby)` 的 [计时被刷] 诊断 WARN，
         *      待机中每秒调一次会把日志刷爆，淹掉真正有用的信息。
         *   未待机时两个问题都不存在，正是本条要生效的窗口。 */

        if (!s_deep_standby && ui_get_current_view() != UI_VIEW_MAIN)
            touch_activity();

        int64_t now_us = esp_timer_get_time();
        int64_t idle_ms = (now_us - s_last_active_us) / 1000;

        // ── 2. 空闲超时直接进入深度待机（删一级后：关屏 + 停舵机，一步到位）────────
        if (!s_deep_standby && idle_ms >= STANDBY_DEEP_TIMEOUT_MS)
        {
            /* ★★【2026-08-25 新增：临期提醒保护】★★
             * 进待机是一段 8~10 秒的阻塞流程，与 1 秒一拍的倒计时到期检测重叠时
             * 会互相抢外设（详见 STANDBY_REMINDER_GUARD_SEC 的说明）。这里在真正
             * 睡下去之前先问一句「最近的提醒还有多久」，临期就本轮跳过。
             *
             * 【查询失败一律照常进待机】NOT_FOUND（没有倒计时在跑）、
             * INVALID_STATE（模块没初始化 / 拿不到锁）都退化为原有行为，
             * 绝不能因为查询本身出问题就永远不进低功耗（那是更严重的耗电故障）。 */
            uint32_t soon_sec = 0;
            if (reminder_get_nearest_expire_sec(&soon_sec) == ESP_OK &&
                soon_sec <= STANDBY_REMINDER_GUARD_SEC)
            {
                ESP_LOGW(TAG, "最近提醒剩 %lu 秒（≤%d），本轮不进深度待机，避免与到期流程重叠",
                         (unsigned long)soon_sec, STANDBY_REMINDER_GUARD_SEC);
            }
            else
            {
                enter_deep_standby();
            }
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

                /* ★【2026-08-31 撤销：关机前不再熄屏】★
                 * 【曾经的做法】这里先把背光从 STANDBY_CLOCK_BK_PCT 渐暗到 0 再震动，
                 *   理由是"让用户先看到屏灭、再感到震动，关机预告更明确"。
                 * 【为什么撤销】需求变更：三级关机前的这段窗口要与二级保持一致 ——
                 *   继续保留「背光 STANDBY_CLOCK_BK_PCT 常驻显示待机时钟」的观感，
                 *   不做黑屏。关机预告只由震动承担，屏幕内容/亮度全程不变，
                 *   直到 enter_shutdown() 真正断电为止。
                 * 【影响面】仅去掉这一次渐暗调用，不动任何状态标志：
                 *   s_warn_vibrating 的一次性语义、下方 !in_window 的停震配对、
                 *   standby_wake() 的三段式转场（渐暗→换 GIF→渐亮）均不受影响。 */

                restore_motor_pwm(); // 恢复马达 LEDC 通道（初始停止态 duty=MAX）
                /* ★【2026-08-31】三级关机前震动强度：100% → STANDBY_SHUTDOWN_WARN_STRENGTH（50%）。
                 * 【为何不是直接写 duty】马达是【低有效】：duty 越小通电越久、震得越强，
                 *   duty=MAX 才是停止。所以 N% 强度对应的 duty = MAX - MAX*N/100，
                 *   与 bsp_touch.c:302-303 bsp_motor_set() 的换算完全一致（原先写死的
                 *   duty=0 即 100% 满强度）。此处不直接调 bsp_motor_set() 是为了保持
                 *   本文件"自己 config + 自己 set duty"的既有写法，避免跨模块引入依赖。 */
                ledc_set_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL,
                              BSP_MOTOR_DUTY_MAX - (uint32_t)BSP_MOTOR_DUTY_MAX * STANDBY_SHUTDOWN_WARN_STRENGTH / 100);
                ledc_update_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL);
                ESP_LOGW(TAG, "关机前震动提醒：距关机剩%dms，震动%dms，强度%d%%（屏幕保持时钟显示不熄屏）",
                         STANDBY_SHUTDOWN_WARN_MS, STANDBY_SHUTDOWN_WARN_VIB_MS,
                         STANDBY_SHUTDOWN_WARN_STRENGTH);
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

        // ── 3c.【2026-08-20 删除】退深度待机「唤醒失败」自动补偿重试。
        //   它补偿的是 standby_wake() 里 disp_on 取锁两次失败、整段回滚不唤醒的情形。
        //   低功耗改为常亮时钟后进待机不再 disp_off，那段回滚逻辑已删，
        //   s_wake_retry_pending 再无人置位，本重试块随之失去意义，一并删除。

        vTaskDelay(pdMS_TO_TICKS(STANDBY_CHECK_MS));
    }
}

void standby_init(void)
{
    if (s_inited)
        return;
    s_inited = true;
    touch_activity();

    // ⚠ 栈必须在内部 SRAM，不能放 SPIRAM！（2026-07-31 修，BUG-010 家族）
    //   本任务的 enter_deep_standby() 路径会调 bsp_board_codec_enter_lowpower()
    //   (bsp_codec.c:482)，其中 nvs_get_i32 读音量要访问 SPI flash；IDF 读 flash 前
    //   必须 spi_flash_disable_interrupts_caches_and_other_cpu() 关 cache，而 cache
    //   一关 SPIRAM 就不可访问 → 栈失联 → esp_task_stack_is_sane_cache_disabled()
    //   断言 abort（实测进深度待机必崩，日志停在「即将 disp_off」后）。
    //   同 application.c 触摸任务（game_whack 读写 NVS 高分）的处理方式。
    //   代价：内部 SRAM 多占 STANDBY_TASK_STACK 字节，换取待机路径可安全碰 flash。
    BaseType_t ret = xTaskCreatePinnedToCoreWithCaps(
        standby_task,
        "standby",
        STANDBY_TASK_STACK,
        NULL,
        5, // 低优先级，不与音频/舵机争抢
        NULL,
        tskNO_AFFINITY,
        MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    if (ret != pdPASS)
        ESP_LOGE(TAG, "待机监测任务创建失败！");
    else
    {
        ESP_LOGI(TAG, "待机模块初始化完成");
        PRINT_TASK_CREATED(TAG, "standby", STANDBY_TASK_STACK, 1); // 栈在内部SRAM
    }
}
