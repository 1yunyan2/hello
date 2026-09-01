/**
 * @file remote_control.c
 * @brief App 远程手动控制实现（舵机角度 / GIF 显示 + 30s 冻结窗口）
 *
 * 内部结构：
 *   remote_control_init()
 *     ├─ xQueueCreate(s_cmd_q)         指令队列（深度 8，值拷贝，不含裸指针）
 *     ├─ xTaskCreate(rc_worker_task)   worker：串行执行舵机运动（可阻塞）
 *     └─ esp_timer_create(s_expire_tmr) 30s 一次性超时定时器
 *
 *   remote_control_submit_servo/gif（MQTT 回调线程）
 *     └─ rc_is_blocked() 屏蔽判断 → 入队 → rc_enter_or_refresh()（重置 30s）
 *
 *   rc_worker_task（独立任务）
 *     ├─ RC_CMD_SERVO  → bsp_servo_move_smooth（阻塞，最长约 1.3s）
 *     ├─ RC_CMD_GIF    → ui_request_state_gif（跨线程安全，内部走 pending+timer）
 *     └─ RC_CMD_CENTER → 三轴并行归中 90°（退出远程控制态时）
 *
 *   s_expire_tmr 到期（esp_timer 线程）
 *     └─ rc_leave(center=true) → 投递归中 + 清冻结标志 + 恢复空闲 GIF 轮播
 *
 * 关键设计说明见 remote_control.h 顶部注释（冻结语义 / 归中策略 / 屏蔽规则 / 线程模型）。
 */
#include "remote_control.h"

#include <string.h>
#include <strings.h> // strcasecmp：GIF 后缀大小写不敏感比对
#include <stdatomic.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_timer.h"
#include "esp_log.h"

#include "bsp/bsp_board.h"   // bsp_servo_move_smooth / CH_* / SERVO_SPEED_*
#include "bsp/servo_manager.h" // servo_manager_flush：清 servo_manager 层队列 + 打断正在执行的那条
#include "ui/ui_port.h"      // ui_request_state_gif / ui_resume_main_gif_loop / ui_get_current_view
#include "ui/standby.h"      // standby_notify_activity（内部会 standby_wake 亮屏）
#include "ui/interaction.h"  // interaction_flush_queue：远程指令抢占正在播的空闲/情绪动作
#include "session/session.h" // session_get_state：对话中屏蔽

static const char *TAG = "RC"; ///< 日志 TAG（Remote Control）

// ─── 可调参数 ────────────────────────────────────────────────────────────────
#define RC_FREEZE_MS 30000   ///< 冻结窗口时长（ms）：收到指令后空闲序列被冻结多久
#define RC_QUEUE_LEN 1       ///< 指令队列深度=1：只保留【最新】目标，见 rc_submit 的 xQueueOverwrite
#define RC_TASK_STACK 4096   ///< worker 栈（内部 SRAM）：只调舵机+切图接口，4K 充裕
#define RC_TASK_PRIO 6       ///< worker 优先级：高于 taskLVGL(5)，保证滑动指令即刻响应不被 GIF 解码压住
#define RC_GIF_DIR "S:/gif/" ///< GIF 目录前缀（LVGL 文件系统盘符写法，对应 fopen 的 /S/gif/）

/**
 * @brief 远程舵机运动速度（step_ms：每度等待毫秒数）
 *
 * 【为何用 INSTANT(0) 而非情绪动作那套 MID(15)】
 * app 端是拖动 3D 形象实时下发角度，语义是「我要的最终位置是 X」而非「依次经过这些位置」，
 * 要求「滑哪里走到哪里」的即时跟随感。而 bsp_servo_move_smooth 的平滑模式是
 * 【每走 1 度 vTaskDelay(step_ms)】的逐度插值，在本项目上有两个硬伤：
 *   1) CONFIG_FREERTOS_HZ=100 → 1 tick = 10ms，任何 <10ms 的 step_ms 都无法实现，
 *      设 15ms 实际每度至少 10ms，走 80° 就要 0.8s 起步；
 *   2) 每步都要重新等调度，期间 taskLVGL 解 GIF、音频任务抢 CPU，单步常被拖到
 *      20~30ms → 累积成肉眼可见的「一顿一顿地挪」。
 * INSTANT(0) 走 bsp_servo_move_smooth 的瞬间分支：直接 iot_servo_write_angle 写目标
 * PWM 后立即返回，全程无 vTaskDelay，微秒级完成。舵机本身有机械惯性，给定目标 PWM
 * 后会以自身速度转过去，平滑感由硬件保证，不需要软件插值（另见 BUG-019/舵机记忆型特性）。
 *
 * 注意：仅改远程控制这一条路径，情绪矩阵/空闲动作仍用各自的速度宏，柔和感不受影响。
 *
 * 【2026-08-05 实测复位为 INSTANT】此前本宏被改成 SERVO_SPEED_MID(15ms/度)，与上述
 * 全部论证矛盾。实测两个后果：
 *   ① 90° 行程 = 90×15ms ≈ 1.35s，期间 rc_worker 持通道锁不返回，后续下发的角度
 *      只能在队列里等 —— 日志实证：240115 开始运动，240945 收到的指令直到 241445
 *      （≈1.35s 后）才打印「执行舵机」，且一次连出两条（积压后集中 drain）。
 *   ② bsp_servo_move_smooth 的插值循环现已支持 abort（2026-08-05 新增），导致
 *      rc_worker【自己的运动】也会被下一次点击的 request_abort 打断在半路且不做
 *      兜底对齐 → 连点时手臂每次只挪几度，表现为「点五六次都没反应」。
 * 【最终取值 MID(15ms/度)·用户指定】上述两个后果的真正根因已在别处修掉——rc_worker
 * 现在会先轮询 servo_manager_is_idle() 等抢占落地再清 abort 标志（见 rc_worker），不再
 * 靠猜延时；故 1.35s 的持锁不会再堵住后续指令，可以放心用大步长换视觉平滑。
 *   · 曾短暂改为 INSTANT(0) 验证链路，实测「能到位，但运动和归中都太快」（用户反馈）。
 *   · MID 下 90° 行程 = 90×15ms ≈ 1.35s，动作舒缓，适合玩具观感。
 *   · 连点时新指令的 abort 会把上一次插值截断在当前角、由新目标接管，正是「滑哪里
 *     走到哪里」的预期语义——不会因为单次运动变长而变得迟钝。
 * 【调档】嫌慢往上调 FAST(5,≈450ms)/VERY_FAST(2,≈180ms)，嫌快往下调 SLOW(30,≈2.7s)；
 *         全部档位见 bsp_config.h:262-267。
 *
 * 【头部与手臂分开】头部要驮整个头壳（屏幕/喇叭等），负载与转动惯量都比手臂大得多，
 * 快速起停会甩动、异响甚至丢步，故单列一档、可比手臂更慢。手臂负载轻，想更跟手时
 * 只调 RC_ARM_SPEED 即可，不影响头部。两者互不干扰。
 */
#define RC_HEAD_SPEED SERVO_SPEED_MID ///< 头部：负载重，MID(15ms/度)，90° ≈ 1.35s
#define RC_ARM_SPEED SERVO_SPEED_MID  ///< 手臂：负载轻，可按需上调 FAST/VERY_FAST

/**
 * @brief 远程控制【归中】速度（30s 冻结窗口到期时三轴回 90°）
 *
 * 与 RC_HEAD_SPEED / RC_ARM_SPEED 分开：归中不是「跟手」动作，而是「休息复位」，要的
 * 是柔和可见的过程，瞬间弹回中位很难看。取 SLOW(30ms/度)，90° ≈ 2.7s，比跟手明显更缓。
 * 注意归中走 bsp_servo_move_all_parallel（三轴同时插值、共用一个 step_ms），故【头臂
 * 无法在归中时分别设速】——三轴同步是它存在的意义，拆开会失去同步感。
 * 持锁 2.7s 不会挡住新指令——move_all_parallel 每步检查 abort，新指令的
 * request_abort 会在一个 step(30ms) 内把归中截断，rc_worker 随即转去执行新角度。
 * 【调档】嫌慢往上调 MID(15,≈1.35s)，嫌快往下调 VERY_SLOW(50,≈4.5s)。
 */
#define RC_CENTER_SPEED SERVO_SPEED_SLOW

/** @brief 指令类型 */
typedef enum
{
    RC_CMD_SERVO = 0, ///< 舵机运动到指定绝对角
    RC_CMD_GIF,       ///< 切换到指定 GIF 并循环播放
    RC_CMD_CENTER,    ///< 三轴归中 90°（退出远程控制态时内部投递，非 MQTT 来源）
} rc_cmd_type_t;

/**
 * @brief 队列元素：整体值拷贝，【不含任何裸指针】
 *
 * gif 文件名用定长数组内联存储而非 const char*，因为 MQTT 回调里的字符串来自
 * cJSON 内部缓冲，回调返回后 cJSON_Delete 即失效，worker 异步取出时会读到野指针。
 */
typedef struct
{
    rc_cmd_type_t type;
    uint8_t channel;               ///< RC_CMD_SERVO 专用：舵机通道
    float angle;                   ///< RC_CMD_SERVO 专用：目标绝对角 0~180（已由 -90~+90 偏移换算而来）
    char gif[REMOTE_GIF_NAME_MAX]; ///< RC_CMD_GIF 专用：文件名（不含目录前缀）
} rc_cmd_t;

// ─── 模块状态 ────────────────────────────────────────────────────────────────
static QueueHandle_t s_cmd_q = NULL;           ///< 指令队列
static esp_timer_handle_t s_expire_tmr = NULL; ///< 30s 冻结超时定时器（一次性）
static bool s_inited = false;                  ///< 初始化完成标志

/** 冻结标志：MQTT 线程写、LVGL 定时器回调高频读，故用 atomic 保证可见性 */
static atomic_bool s_active = ATOMIC_VAR_INIT(false);

/**
 * @brief 判断当前状态是否应屏蔽远程指令
 *
 * 三类屏蔽场景的理由见 remote_control.h 顶部「屏蔽规则」。
 * 深度待机【不在此列】——那是要唤醒后执行，不是丢弃。
 *
 * @param[out] why 输出屏蔽原因字符串（仅用于日志），未屏蔽时不修改
 * @return true 应丢弃本次指令
 */
static bool rc_is_blocked(const char **why)
{
    // 对话中（LISTENING/PLAYING）：远程改图会和状态 GIF 打架，甩头会打断对话体验
    if (session_get_state() != SESSION_IDLE)
    {
        *why = "对话进行中";
        return true;
    }
    // 游戏中：游戏独占屏幕与舵机，远程切图会直接盖掉游戏画面
    if (ui_get_current_view() == UI_VIEW_GAME)
    {
        *why = "游戏进行中";
        return true;
    }
    return false;
}

/**
 * @brief 冻结窗口到期回调（esp_timer 线程）
 *
 * 走「有归中」的退出路径：投递归中指令 + 清冻结标志 + 恢复空闲 GIF 轮播。
 * @param arg 未使用
 */
static void rc_expire_cb(void *arg)
{
    (void)arg;
    ESP_LOGI(TAG, "远程控制 %d 秒窗口到期，归中并恢复空闲序列", RC_FREEZE_MS / 1000);
    remote_control_cancel(/*center_servo=*/true);
}

/**
 * @brief 进入或续期远程控制态（重置 30s 冻结窗口）
 *
 * esp_timer_stop 对未启动的定时器返回 ESP_ERR_INVALID_STATE，此处忽略返回值：
 * 「先停再启」是重置一次性定时器的标准手法，首次进入时 stop 失败属正常。
 */
static void rc_enter_or_refresh(void)
{
    atomic_store(&s_active, true);
    esp_timer_stop(s_expire_tmr); // 首次进入时未启动，返回错误可忽略
    esp_timer_start_once(s_expire_tmr, (uint64_t)RC_FREEZE_MS * 1000ULL);
}

/**
 * @brief worker 任务：串行执行队列中的指令
 *
 * 必须独立成任务的原因：bsp_servo_move_smooth 内含 vTaskDelay（插值平滑），
 * 单次最长阻塞约 1.3s。若在 MQTT 事件回调里直接执行，app 快速连点会把 MQTT
 * 事件循环堵死，导致 keepalive 超时断连（见 remote_control.h「线程模型」）。
 *
 * @param arg 未使用
 */
static void rc_worker_task(void *arg)
{
    (void)arg;
    rc_cmd_t cmd;
    for (;;)
    {
        if (xQueueReceive(s_cmd_q, &cmd, portMAX_DELAY) != pdTRUE)
            continue;

        switch (cmd.type)
        {
        case RC_CMD_SERVO:
            // ── 等「抢占真正生效」再清打断标志 ────────────────────────────────
            // rc_submit 已置 abort + flush，但被打断方要走到插值循环的下一个检查点
            // 才读得到标志，最慢 SERVO_SPEED_VERY_SLOW=50ms/度。故不能按固定延时猜，
            // 须轮询 servo_manager_is_idle()（队列空 且 worker 无请求在执行）确认它
            // 已让出通道锁。同款手法见 standby.c:197-204（进深度待机前等舵机静止）。
            // 上限 300ms 兜底：正常 50~100ms 内即空闲；超时也照常往下走，最坏就是
            // bsp_servo_move_smooth 在互斥锁上多等一会儿，不会卡死。
            {
                int guard = 0;
                while (!servo_manager_is_idle() && guard < 30)
                {
                    vTaskDelay(pdMS_TO_TICKS(10));
                    guard++;
                }
                // ★这里【不再】清打断标志。清标志与拿锁必须原子，否则二者之间的窗口里
                //   servo_manager worker 可能取到新请求、先清标志再持锁开跑，使本次运动
                //   拿不到锁/拿到时又读到新标志 → 插值首步 break → 手臂停在半路不动
                //   （实测约 1/10 偶发）。改由 bsp_servo_move_smooth_preempt 在【拿到锁之后】
                //   清，锁在手时无人能插队。本循环仍需保留：它负责等被抢占方先让出锁，
                //   否则 preempt 会在 xSemaphoreTake 上干等。
            }
            {
                // 头部负载重（驮整个头壳），单列一档速度；手臂负载轻，可调更快。
                uint32_t speed = (cmd.channel == CH_HEAD) ? RC_HEAD_SPEED : RC_ARM_SPEED;
                ESP_LOGI(TAG, "执行舵机: ch=%u → %.1f° (step=%ums)",
                         (unsigned)cmd.channel, cmd.angle, (unsigned)speed);
                // 直接走底层绝对角定位：自带软限位/平滑插值/去抖，到位即停。
                // 不经 servo_manager_submit_angle —— 其「拆幅度+方向再回中」语义会导致
                // 到位后回弹，且把绝对角塞进 amplitude 枚举会造成角度错乱。
                // 用 _preempt 变体：拿到锁后才清打断标志，消除「清标志→拿锁」之间的竞态窗口。
                bsp_servo_move_smooth_preempt(cmd.channel, cmd.angle, speed);
            }
            break;

        case RC_CMD_GIF:
        {
            // 拼成 LVGL 盘符路径。用「状态切图」接口而非情绪切图：状态切图是高优先级，
            // 不会被 ui_port 内部「对话中丢弃情绪切图」的兜底逻辑误伤。
            char path[sizeof(RC_GIF_DIR) + REMOTE_GIF_NAME_MAX];
            snprintf(path, sizeof(path), RC_GIF_DIR "%s", cmd.gif);
            ESP_LOGI(TAG, "执行切图: %s", path);
            // 内部只设 pending 标记 + 唤醒 LVGL 线程延迟 timer，真正的 lv_gif_set_src
            // 在 LVGL 线程执行（BUG-010：lv_gif_set_src 必须在 LVGL 线程调）。
            // 文件不存在时由该路径统一处理（保持当前画面 + 打日志），此处不做 IO。
            ui_request_state_gif(path);
            break;
        }

        case RC_CMD_CENTER:
            ESP_LOGI(TAG, "执行归中: 三轴 → 90°");
            // 三轴并行归中：一次调用同时插值三路，比串行三次快且不割裂。
            // 用 RC_CENTER_SPEED 而非 RC_SERVO_SPEED：归中是「休息复位」，要柔和过程，
            // 不能像跟手指令那样快速到位（瞬间弹回中位很难看）。
            bsp_servo_move_all_parallel(90.0f, 90.0f, 90.0f, RC_CENTER_SPEED);
            break;

        default:
            break;
        }
    }
}

bool remote_control_init(void)
{
    if (s_inited)
        return true; // 幂等

    s_cmd_q = xQueueCreate(RC_QUEUE_LEN, sizeof(rc_cmd_t));
    if (s_cmd_q == NULL)
    {
        ESP_LOGE(TAG, "指令队列创建失败，远程控制模块不可用");
        return false;
    }

    const esp_timer_create_args_t targs = {
        .callback = rc_expire_cb,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "rc_expire",
    };
    if (esp_timer_create(&targs, &s_expire_tmr) != ESP_OK)
    {
        ESP_LOGE(TAG, "超时定时器创建失败，远程控制模块不可用");
        vQueueDelete(s_cmd_q);
        s_cmd_q = NULL;
        return false;
    }

    // worker 栈放内部 SRAM（默认）：其中调用的 bsp_servo_move_smooth 会碰 LEDC 寄存器，
    // 且 ui_request_state_gif 内部可能触及 SPI flash 路径，栈在 SPIRAM 有 BUG-010 风险。
    if (xTaskCreate(rc_worker_task, "rc_worker", RC_TASK_STACK, NULL, RC_TASK_PRIO, NULL) != pdPASS)
    {
        ESP_LOGE(TAG, "worker 任务创建失败，远程控制模块不可用");
        esp_timer_delete(s_expire_tmr);
        s_expire_tmr = NULL;
        vQueueDelete(s_cmd_q);
        s_cmd_q = NULL;
        return false;
    }

    s_inited = true;
    ESP_LOGI(TAG, "远程控制模块就绪（冻结窗口 %d ms，队列深度 %d）", RC_FREEZE_MS, RC_QUEUE_LEN);
    return true;
}

/**
 * @brief 公共入队逻辑：屏蔽判断 → 刷新待机 → 入队 → 进入/续期冻结窗口
 *
 * @param cmd 待入队指令（值拷贝）
 * @param tag 日志用的指令描述
 * @return true 已入队
 */
static bool rc_submit(const rc_cmd_t *cmd, const char *tag)
{
    if (!s_inited)
    {
        ESP_LOGW(TAG, "模块未初始化，丢弃指令(%s)", tag);
        return false;
    }

    const char *why = NULL;
    if (rc_is_blocked(&why))
    {
        ESP_LOGW(TAG, "%s，丢弃远程指令(%s)", why, tag);
        return false;
    }

    // 刷新待机计时并唤醒：standby_notify_activity 内部直接调 standby_wake()，
    // 故深度待机（屏已关）时会在此自动亮屏 + 恢复舵机 PWM，随后指令才有意义
    // ——用户在 app 上主动操作，就是期望看到设备有反应（尤其 GIF 必须有屏）。
    // 同时防止运动中途跨过空闲阈值被 enter_deep_standby 强行归中打断本次控制。
    standby_notify_activity();

    // ── 打断正在播放的空闲/情绪动作 ──────────────────────────────────────────
    // 必须做，否则远程指令要排队等它走完：interaction worker 与 rc_worker 是两个
    // 独立任务，但抢【同一个舵机通道的 mutex】（bsp_servo_move_smooth 内 portMAX_DELAY
    // 持锁直到本次插值走完）。实测表现为「先把当前动作序列走完，才走 MQTT 下发的位置」，
    // 延迟可达 2s。三步缺一不可：
    //   1) request_abort  → 中止【正在执行】那条的逐度插值循环（每步检查标志，几十 ms 内停）
    //   2) flush_queue    → 清掉 interaction 队列里【尚未执行】的存量动作，防止刚打断又来一条
    //   3) clear_abort    → 清标志，否则紧接着 rc_worker 自己的运动也会被立即中止
    // 远程控制是用户主动操作，优先级高于自动播放的空闲动作，抢占是预期行为。
    //
    // 【2026-08-05 修】原来只有 abort + interaction_flush + clear 三步，实测无效，
    // 表现为「等当前动作走完才执行下发角度」甚至「-90 被执行成归中 90」。两个原因：
    //   ① interaction_flush_queue 只清 interaction 队列，清不到 servo_manager 的队列。
    //      空闲动作由 ui_interaction_play_custom 投递后最终落到 servo_manager worker，
    //      它照跑不误，且每条动作收尾会归中 90°——这就是「变成归中」的来源。
    //      故必须补 servo_manager_flush()（见 interaction.c:830 注释写明的二者分工，
    //      以及 interaction_stop_for_ota 的三步齐全写法）。
    //   ② clear_abort 紧跟 abort 之后几微秒执行，而被打断方要走到插值循环的下一个
    //      检查点才读得到标志（最坏一个 step_ms）。标志在被看到之前就被清掉 = 没打断。
    bsp_servo_request_abort();  // 1) 中止【正在执行】那条的逐度插值（bsp_servo.c 循环每步检查）
    interaction_flush_queue();  // 2) 清 interaction 队列里【尚未执行】的存量动作
    // 3) 清 servo_manager 队列 + 打断其 worker 正在执行的那条。
    //    ★传 center=false：本次打断【有后续接管】——紧接着 rc_worker 就要把手臂送到用户
    //    指定角度。若让 worker 归中，它会与 rc_worker 抢同一把通道锁，实测产生三种随机
    //    现象：「到位后被慢速拉回 90°」「先归中再走到目标」「归中走一半停住」。
    servo_manager_flush_ex(/*center=*/false);
    // 4) 【不在此清 abort 标志】——清标志必须等「打断真正生效」之后，而那要等被打断方
    //    走到插值循环的下一个检查点。曾在此写死 vTaskDelay(20ms) 后清，实测无效：
    //    情绪动作最慢用 SERVO_SPEED_VERY_SLOW=50ms/度（如情绪 11 慵懒，interaction.c:287），
    //    50ms 才检查一次，20ms 窗口把标志抢在它看到之前就清掉 → 打断失效、动作继续持锁跑，
    //    表现为「舵机在跑动作序列，我的命令不执行」。standby.c:195 早有同款告诫。
    //    改为在 rc_worker 里【轮询等 servo_manager 真正空闲】后再清（见 rc_worker），
    //    等待放在专用任务里做，MQTT 回调保持非阻塞。

    // ── 只保留最新目标：xQueueOverwrite 覆盖式入队（队列深度=1）──────────────
    // app 拖动 3D 形象时会连续下发大量角度，语义是「最终位置是 X」而非「依次经过」。
    // 若用 xQueueSend 逐条缓存，worker 会把每个中间角度都走一遍 → 表现为「一顿一顿
    // 地走」且严重滞后于手指。覆盖式入队让过时目标直接被丢弃、只执行最新的，实现
    // 「滑哪里走到哪里」。xQueueOverwrite 对深度 1 的队列永不阻塞、永不失败。
    xQueueOverwrite(s_cmd_q, cmd);

    rc_enter_or_refresh(); // 进入或续期 30s 冻结窗口
    return true;
}

bool remote_control_submit_servo(uint8_t channel, float angle)
{
    // 通道白名单校验：非法通道直接拒，避免把越界值传到 LEDC 层
    if (channel != CH_HEAD && channel != CH_L_ARM && channel != CH_R_ARM)
    {
        ESP_LOGE(TAG, "非法舵机通道 %u，丢弃", (unsigned)channel);
        return false;
    }

    rc_cmd_t cmd = {
        .type = RC_CMD_SERVO,
        .channel = channel,
        // 【偏移语义】app 下发的是相对中位的偏移角 -90~+90（0=中位/正前方），
        // 在此换算成舵机物理绝对角：-90→0°, 0→90°, +90→180°。
        // 前端用偏移量表达更直观（正负=左右），故协议层保持偏移、驱动层用绝对角。
        // 超范围由 bsp_servo 的 clamp_safe_angle 软限位裁剪。
        .angle = 90.0f + angle,
    };
    ESP_LOGI(TAG, "收到远程舵机指令: ch=%u 偏移%+.1f° → 绝对角 %.1f°",
             (unsigned)channel, angle, cmd.angle);
    return rc_submit(&cmd, "servo");
}

/**
 * @brief 校验 app 下发的 GIF 文件名是否安全合法
 *
 * 固件不维护编号映射表，直接信任 app 给的文件名，故必须在此做路径安全校验，
 * 防止形如 "../voice/xxx" 的输入跳出 /S/gif 目录。
 *
 * @param file 待校验文件名
 * @return true 合法可用
 */
static bool rc_gif_name_valid(const char *file)
{
    if (file == NULL || file[0] == '\0')
        return false;

    size_t len = strlen(file);
    if (len >= REMOTE_GIF_NAME_MAX)
    {
        ESP_LOGE(TAG, "GIF 文件名过长(%u ≥ %d)，丢弃", (unsigned)len, REMOTE_GIF_NAME_MAX);
        return false;
    }
    // 禁止任何路径分隔符与上跳，确保只能访问 /S/gif/ 下的一级文件
    if (strchr(file, '/') || strchr(file, '\\') || strstr(file, ".."))
    {
        ESP_LOGE(TAG, "GIF 文件名含非法路径字符，丢弃: %s", file);
        return false;
    }
    // 必须是 .gif（大小写不敏感）：避免误播非 GIF 文件导致解码器异常
    if (len < 4 || strcasecmp(file + len - 4, ".gif") != 0)
    {
        ESP_LOGE(TAG, "GIF 文件名后缀非 .gif，丢弃: %s", file);
        return false;
    }
    return true;
}

bool remote_control_submit_gif(const char *file)
{
    if (!rc_gif_name_valid(file))
        return false;

    rc_cmd_t cmd = {.type = RC_CMD_GIF};
    // 定长内联拷贝：MQTT 回调返回后 cJSON 缓冲即失效，绝不能存指针（见 rc_cmd_t 注释）
    strncpy(cmd.gif, file, sizeof(cmd.gif) - 1);
    cmd.gif[sizeof(cmd.gif) - 1] = '\0';

    ESP_LOGI(TAG, "收到远程 GIF 指令: %s", cmd.gif);
    return rc_submit(&cmd, "gif");
}

bool remote_control_is_active(void)
{
    return atomic_load(&s_active);
}

void remote_control_cancel(bool center_servo)
{
    if (!s_inited)
        return;

    // 幂等：非远程控制态直接返回，允许各交互入口无脑调用。
    // atomic_exchange 保证并发调用只有一个走进下面的收尾流程。
    if (!atomic_exchange(&s_active, false))
        return;

    esp_timer_stop(s_expire_tmr); // 到期回调路径中调用时已自然停止，返回错误可忽略

    if (center_servo)
    {
        // 无后续动作接管（到期 / 进待机）：归中回 90°，回到空闲序列的基准姿态。
        // 投递给 worker 异步执行，不阻塞调用方（可能是 esp_timer 或 standby 任务）。
        rc_cmd_t cmd = {.type = RC_CMD_CENTER};
        // 覆盖式入队：队列深度=1，归中作为「最新目标」顶掉可能残留的过时角度指令
        xQueueOverwrite(s_cmd_q, &cmd);
    }
    else
    {
        // 有后续动作立即接管（触摸情绪 / 唤醒对话）：不归中，让后续动作直接从当前
        // 角度平滑过渡（bsp_servo_move_smooth 自带插值起点=当前角），避免复位抽动。
        ESP_LOGI(TAG, "远程控制被交互打断，舵机让位（不归中）");
    }

    // 恢复主界面空闲 GIF 自动轮播。内部自带 s_view==MAIN 判断与跨线程安全，
    // 若此刻已进功能盘/游戏则不误恢复。
    ui_resume_main_gif_loop();
}
