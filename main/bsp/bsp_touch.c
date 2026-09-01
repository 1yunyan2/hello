/**
 * @file bsp_touch.c
 * @brief 触摸按键板级支持包
 *
 * 支持两套触摸方案，通过 bsp_config.h 中的 BSP_USE_TTP223 切换：
 *   BSP_USE_TTP223 1 — TTP223-TD 外置芯片，GPIO 数字读取，低有效
 *   BSP_USE_TTP223 0 — ESP32-S3 内置电容触摸，能量阈值检测
 */

#include "bsp/bsp_board.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "bsp/bsp_config.h"
#include "ui/interaction.h"
#include "ui/ui_port.h"
/* standby_is_deep_active() / standby_notify_activity()：
 * 待机中身体铜箔「按下即唤醒」需要直接查询待机状态并触发唤醒，见扫描循环内的说明。 */
#include "ui/standby.h"
#include "games/games.h" // games_get_current()：按具体游戏切换触摸触发方式
#include "object.h"      // PRINT_TASK_STACK_HWM：任务栈高水位打印
#define BSP_USE_TTP223 1

#if !BSP_USE_TTP223
#include "driver/touch_pad.h"
#endif

static const char *TAG = "BSP_TOUCH";

#define TOUCH_EVENT_QUEUE_LEN 8
static QueueHandle_t s_touch_event_queue = NULL;

/* 最近一次翻页键（左/右耳）按压时长（毫秒），供跳一跳「长按蓄力」读取。
 * 在 update_page_btn 松手算出 held 后写入，bsp_touch_last_page_hold_ms() 读取。 */
static volatile uint32_t s_last_page_hold_ms = 0;

// OTA 升级时置位：touch_scan_task 在循环开头检测到后自删（vTaskDelete(NULL)），
// 让出 CPU、彻底停止触摸扫描。不可逆——OTA 结束必重启，无需恢复。
static volatile bool s_touch_stop_for_ota = false;
void bsp_touch_stop_for_ota(void) { s_touch_stop_for_ota = true; }

// 本轮是否需要震动：按下沿 / 上层确认生效时置位，等事件分发完再真正震
// （避免 bsp_motor_pulse() 的 30ms 阻塞拖慢「按下即触发」，也避免持 LVGL 锁时阻塞）
static volatile bool s_vib_pending = false;

uint32_t bsp_touch_last_page_hold_ms(void)
{
    return s_last_page_hold_ms;
}

/* 【2026-08-31】上层「确认已生效」后请求的一次震动（问题2：震动跟真实切换，不跟触摸）。
 *
 * 【为什么要这个转发口】功能盘左右耳切换的震动必须由 home_jelly() 真正推进下标后才发，
 * 但 home_jelly() 是在【触摸任务】里、且【持着 LVGL 锁】被调用的；若在那里直接调
 * bsp_motor_pulse()，其内含的 30ms vTaskDelay 会持锁阻塞 LVGL 线程 30ms（同 BUG-038 的坑）。
 * 故上层只置标志，真正的震动复用本文件循环末尾「事件分发后、锁已释放」的统一打点。
 * 与既有的按下沿 s_vib_pending 是同一个标志，天然去重：同一轮里两边都置位也只震一次。 */
void bsp_touch_request_vibrate(void)
{
    s_vib_pending = true;
}

/**
 * @brief 查询翻页键当前实时按压时长（毫秒）
 *
 * 供跳一跳 engine_cb 每帧调用，驱动「按住期间小人/台子压扁 + 蓄力条实时增长」。
 * 若当前没有翻页键被按住，返回 0。
 *
 * 实现：直接读 btn_prev_page / btn_next_page 的 is_pressed + press_start_ms，
 * 与扫描任务共享内存，无锁（uint32_t 对齐读原子安全，最坏差一个扫描帧 ≈20ms）。
 */
// ── 公共时序参数 ──────────────────────────────────────────────────────────────
#define BODY_PRESS_MIN_MS 10       // 身体键最短按压时长（ms），低于这个值的按压会被丢弃（防误触）
#define PAGE_SHORT_PRESS_MIN_MS 10 // 翻页键短按最短时长（ms），低于这个值的按压会被丢弃（防误触）
#define PAGE_LONG_PRESS_MS 500
#define PRESS_DEBOUNCE 2
#define RELEASE_DEBOUNCE 3
/* 头部长按阈值：松手时超过此时长发 LONG_HEAD（退出/返回），否则发 SHORT_HEAD */
#define HEAD_LONG_PRESS_MS 400

/* ═══════════════════════════════════════════════════════════════════════════
 * 【2026-08-31 问题7】持续按压闩锁：一次持续接触只允许产生一次事件
 *
 * 【修的问题】手指按住不松、只是摩擦/扭动，屏幕会频繁「进功能盘→切图标→退出」
 *   循环刷屏。实测日志（事件 11=LONG_NEXT_PAGE / 9=SHORT_NEXT_PAGE）间隔仅
 *   360/600/450ms，长短按交替出现 —— 说明手指微动导致接触面积变化，
 *   TTP223 的 OUT 电平被真实地拉断又接通，一次持续接触被切成了多次独立按压。
 *
 * 【为什么原有消抖挡不住】RELEASE_DEBOUNCE=3 帧 × 20ms ≈ 60ms，摩擦扭动造成的
 *   瞬时断开轻松超过 60ms，于是被判定为"真的松手了"，下一次接触就是"新按压"。
 *
 * 【为什么在主界面尤其明显】主界面下左右耳的短按和长按【全都是】"进功能盘"
 *   （ui_port.c 的 UI_VIEW_MAIN 分支），四种判定没有一个是空挡，
 *   于是每一次微动都被放大成一次真实的界面跳转 → 肉眼看到的就是狂跳。
 *   头部长按同样进功能盘，故头部也有相同现象。
 *
 * 【本闩锁怎么做】按键发过一次事件后置 consumed，此后即使再检测到"按下"也不再
 *   发事件；只有连续检测到【真正未触摸】达到 TOUCH_RELEASE_CONFIRM_MS 才解除，
 *   允许下一次事件。即「按住不放 = 一次事件，之后彻底安静」。
 *
 * ⚠【必须数 pressed_raw，不能数 pressed】pressed = pressed_raw && !blocked，
 *   双耳互斥时 pressed 会变 false 但手指其实还在；若拿 pressed 计离手帧，
 *   互斥一发生就误判成离手，闩锁立即失效等于白改。
 *
 * ⚠【游戏视图整体豁免】跳一跳靠松手 held 决定蓄力、打地鼠/赛车要快速连击，
 *   闩锁一旦介入会把连续操作吞掉直接玩不了，故 game_mode / press_trigger
 *   两种模式完全不走闩锁，行为与改动前一字不变。
 *
 * 【调参】嫌连按发钝就调小本值；仍有误触发就调大。300ms 的依据：实测误触发
 *   间隔 360~600ms，而正常"松手再按"的真实离手时间远超 300ms。
 * ═══════════════════════════════════════════════════════════════════════════ */
#define TOUCH_RELEASE_CONFIRM_MS 100 // 确认「真正离手」所需的连续未触摸时长（ms）
/* 换算成扫描帧数：TTP223 分支扫描周期 20ms（见 touch_scan_task 的 vTaskDelay） */
#define TOUCH_RELEASE_CONFIRM_FRAMES (TOUCH_RELEASE_CONFIRM_MS / 20)
// ── 方案专属参数 ──────────────────────────────────────────────────────────────
#if BSP_USE_TTP223
// TTP223 上电后约 500ms 完成内部基线校准，留 1s 裕量
#define POWER_ON_MASK_TIME 1500
#else
// 内置电容触摸上电后基线漂移窗口较长
#define POWER_ON_MASK_TIME 4000
#define TOUCH_THRESH_PERCENT 0.15f
#define TOUCH_MIN_DELTA 2000
#endif

// ── 按键状态结构体 ────────────────────────────────────────────────────────────
typedef struct
{
#if BSP_USE_TTP223
    gpio_num_t pin; // TTP223 OUT 引脚
#else
    touch_pad_t channel; // ESP32 内置触摸通道
    uint32_t baseline;   // 无触摸时的基线原始值
#endif
    bool is_pressed;
    uint32_t press_start_ms;
    uint8_t press_count;
    uint8_t release_count;
    /* ── 持续按压闩锁（2026-08-31 问题7，详见 TOUCH_RELEASE_CONFIRM_MS 注释）── */
    bool consumed;       // 本次持续接触已发过事件，未真正离手前不再发
    uint8_t idle_frames; // 连续检测到「真正未触摸」(pressed_raw==false) 的帧数
} touch_btn_t;

// ── 按键实例 ──────────────────────────────────────────────────────────────────
#if BSP_USE_TTP223
static touch_btn_t btn_head = {.pin = BSP_TOUCH_1_PIN};
static touch_btn_t btn_back = {.pin = BSP_TOUCH_2_PIN};
static touch_btn_t btn_abdomen = {.pin = BSP_TOUCH_3_PIN};
static touch_btn_t btn_prev_page = {.pin = BSP_TOUCH_PREV_PIN};
static touch_btn_t btn_next_page = {.pin = BSP_TOUCH_NEXT_PIN};
#else
static touch_btn_t btn_head = {.channel = BSP_TOUCH_1_PIN};
static touch_btn_t btn_abdomen = {.channel = BSP_TOUCH_3_PIN};
static touch_btn_t btn_back = {.channel = BSP_TOUCH_2_PIN};
static touch_btn_t btn_prev_page = {.channel = BSP_TOUCH_PREV_PIN};
static touch_btn_t btn_next_page = {.channel = BSP_TOUCH_NEXT_PIN};
#endif

/* 查询翻页键当前实时按压时长（毫秒）。供跳一跳 engine_cb 每帧驱动压扁动画。
 * 放在 btn_prev_page / btn_next_page 声明之后，避免前向引用编译错误。 */
uint32_t bsp_touch_page_held_ms(void)
{
    uint32_t now = (uint32_t)(esp_timer_get_time() / 1000);
    if (btn_prev_page.is_pressed && btn_prev_page.press_start_ms > 0)
        return now - btn_prev_page.press_start_ms;
    if (btn_next_page.is_pressed && btn_next_page.press_start_ms > 0)
        return now - btn_next_page.press_start_ms;
    return 0;
}

/* ── 清除全部按键的「持续按压闩锁」（2026-08-31 退低功耗触摸失灵修复）──────────
 * 【修的问题】用某个触摸位置唤醒深度待机后，【正是那一个】位置随后按不动，
 *   其余四个位置立刻摸都正常。
 * 【根因】唤醒那一次触摸在 update_page_btn / update_body_btn 里正常发出了事件，
 *   同时按惯例置了 btn->consumed = true（闩锁：同一次持续接触只发一次）。
 *   但上层 ui_dispatch_touch_event 判定 was_standby 后【直接 return 丢弃了这个
 *   事件】——事件被消费掉了，闩锁的代价却留了下来。触摸层无从得知上层丢弃了它。
 *   此后要靠 idle_frames 连续累够 TOUCH_RELEASE_CONFIRM_FRAMES 帧才会清 consumed，
 *   而只要中间有一帧读到手指还在就 idle_frames = 0 从头再来。
 * 【为何偶发】退待机转场（standby_wake）约 2.6 秒全程阻塞，且就跑在触摸任务
 *   自己的栈上，这期间扫描一帧都没跑。转场结束时手指若仍搭着、或恰好又碰了一下，
 *   计数就被反复打断，consumed 永久残留 → 该键彻底哑掉。手指干脆离开则自愈，
 *   故表现为「偶发」。
 * 【为何清全部而不只清被摸的那个】触摸层拿不到「上层丢弃了哪个事件」的信息，
 *   要反查得新增事件→按键映射，改动面反而更大。而转场那 2.6 秒扫描本就没跑，
 *   五个键的闩锁状态【全是过期数据】，整体丢弃重建在语义上本来就是对的。
 *   未被摸的四个键 consumed 本就是 false，清了等于空操作，零副作用。
 * 【idle_frames 一并置满】而非清零：置满表示「已确认离手」，避免刚清完 consumed
 *   又因计数从 0 起步而在下一次接触前被重新判成「尚未离手」。
 * @note 纯内存赋值，无阻塞 / 无 NVS / 无锁 —— 可安全地在 standby_wake() 这种
 *       跑在 PSRAM 栈任务上的上下文里直接调用（见 BUG-039 的 PSRAM 栈禁忌）。
 * @note 幂等，重复调用无副作用。
 * 【放置位置】必须在 TOUCH_RELEASE_CONFIRM_FRAMES 宏与五个 btn_* 实例声明【之后】，
 *   避免前向引用编译错误（同 bsp_touch_page_held_ms 的理由）。 */
void bsp_touch_clear_latch(void)
{
    touch_btn_t *all[] = {&btn_head, &btn_abdomen, &btn_back,
                          &btn_prev_page, &btn_next_page};
    for (int i = 0; i < (int)(sizeof(all) / sizeof(all[0])); i++)
    {
        all[i]->consumed = false;                           // 解除闩锁，允许下一次接触正常发事件
        all[i]->idle_frames = TOUCH_RELEASE_CONFIRM_FRAMES; // 直接判定为「已确认离手」
    }
}

static bool s_combo_ha_active = false;
static uint8_t s_combo_ha_cnt = 0;
static bool s_combo_hb_active = false;
static uint8_t s_combo_hb_cnt = 0;
static bool s_combo_ab_active = false;
static uint8_t s_combo_ab_cnt = 0;

/**
 * @brief 翻页键占用者枚举
 * 用于实现翻页键互斥逻辑，防止前后页同时按下产生冲突
 */
typedef enum
{
    PAGE_OWNER_NONE = 0, // 无人占用
    PAGE_OWNER_PREV,     // 前页键占用
    PAGE_OWNER_NEXT      // 后页键占用
} page_owner_t;

// 当前翻页键占用者
static page_owner_t s_page_owner = PAGE_OWNER_NONE;
// 翻页键最后活跃时间（用于松手后冷却，防止电容耦合误触身体键）
static uint32_t s_page_active_ms = 0;

/**
 * @brief 发送触摸事件到队列
 * 函数含义：将检测到的触摸事件发送到事件队列，供后续处理
 * @param event 参数含义：要发送的触摸事件类型
 */
static void send_touch_event(touch_event_t event)
{
    // API含义：FreeRTOS队列发送函数，将数据发送到队列
    // API参数含义：
    //   s_touch_event_queue：队列句柄
    //   &event：要发送的数据指针
    //   0：阻塞时间（0表示不阻塞，立即返回）
    xQueueSend(s_touch_event_queue, &event, 0);
}

/**
 * @brief 初始化震动马达 LEDC PWM 通道
 *
 * 马达由「GPIO 高低电平开关」升级为 LEDC PWM，用占空比调震动强度。
 * 资源隔离见 BUG-015：独立 TIMER_2 + CH4，时钟源强制 XTAL 与舵机/背光统一，
 * 否则同 speed_mode 共享时钟源会触发 "timer clock conflict" 致舵机初始化失败。
 *
 * 极性：马达低有效（OUT=0 通电）。LEDC duty 与输出电平的关系：
 *   duty=0    → 输出恒「低」电平 → 低有效下 = 满功率通电（一直震！）
 *   duty=MAX  → 输出恒「高」电平 → 低有效下 = 断电停止
 * ★ 因此「停止」必须用 duty=MAX，不是 0。初始化时若写 duty=0 会上电狂震。
 * 在 bsp_touch_init() 中替代原来的马达 GPIO 配置调用。
 *
 * 【2026-08-31】由 static 改为对外可见：开机震动反馈（application_init 最前面）
 * 需要在 touch_scan_task 创建之前就把马达 LEDC 准备好。幂等，重复调用无副作用
 * （bsp_touch_init 里仍会再调一次，LEDC 重复配置同参数是安全的）。
 */
void bsp_motor_ledc_init(void)
{
    // 定时器：独立 TIMER_2，XTAL 时钟源，5kHz，10bit
    ledc_timer_config_t motor_timer = {
        .speed_mode = BSP_MOTOR_LEDC_MODE,
        .timer_num = BSP_MOTOR_LEDC_TIMER,
        .duty_resolution = BSP_MOTOR_LEDC_RES,
        .freq_hz = BSP_MOTOR_LEDC_FREQ_HZ,
        .clk_cfg = BSP_MOTOR_LEDC_CLK, // 强制 XTAL，与舵机/背光统一（BUG-015）
    };
    ESP_ERROR_CHECK(ledc_timer_config(&motor_timer));

    // 通道：独立 CH4，绑定马达引脚。
    // ★ 初始 duty=MAX（输出恒高电平），低有效下=断电=停止，避免上电狂震。
    ledc_channel_config_t motor_channel = {
        .speed_mode = BSP_MOTOR_LEDC_MODE,
        .channel = BSP_MOTOR_LEDC_CHANNEL,
        .timer_sel = BSP_MOTOR_LEDC_TIMER,
        .gpio_num = BSP_MOTOR_VIB_PIN,
        .duty = BSP_MOTOR_DUTY_MAX, // 初始停止（恒高电平 → 低有效断电）
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&motor_channel));
}

/**
 * @brief 设置震动马达持续输出强度
 * @param strength 震动强度百分比 0~100：0=停止，100=最强
 *
 * 低有效极性处理：strength 表示「通电占比」，但马达 OUT=0 才通电，
 * 因此实际写入 LEDC 的占空 = 满占空 - 通电占空（反相）。
 *   strength=0   → LEDC duty=MAX（恒高电平）→ 断电停止
 *   strength=100 → LEDC duty=0（恒低电平）  → 满功率通电
 */
void bsp_motor_set(uint8_t strength)
{
    if (strength > 100)
        strength = 100;

    // 期望通电占空（正逻辑），再反相得到实际 LEDC 占空（低有效）
    uint32_t on_duty = (uint32_t)BSP_MOTOR_DUTY_MAX * strength / 100;
    uint32_t ledc_duty = BSP_MOTOR_DUTY_MAX - on_duty;

    ledc_set_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL, ledc_duty);
    ledc_update_duty(BSP_MOTOR_LEDC_MODE, BSP_MOTOR_LEDC_CHANNEL);
}

/**
 * @brief 震动马达带强度的单次脉冲
 * @param strength 震动强度百分比 0~100
 * @param ms       持续时长（毫秒），结束后自动停止
 * @note 内部含 vTaskDelay 阻塞，仅可在任务上下文调用
 */
void bsp_motor_pulse_level(uint8_t strength, uint32_t ms)
{
    bsp_motor_set(strength);
    vTaskDelay(pdMS_TO_TICKS(ms));
    bsp_motor_set(0); // 停止
}

/**
 * @brief 马达震动脉冲（默认强度，约 30ms）
 * 函数含义：用 LEDC PWM 以默认强度震动 30ms，用于触摸/唤醒等触觉反馈
 */
void bsp_motor_pulse(void)
{
    bsp_motor_pulse_level(BSP_MOTOR_DEFAULT_STRENGTH, 30);
}

// ═══════════════════════════════════════════════════════════════════════════
// 【震动 PWM 方波测试任务】—— 验证占空比可调 + 示波器观测方波
//
// 用法：在 application.c 里 xTaskCreate(motor_pwm_test_task, ...) 启动即可。
// 行为：循环把占空比设成 0→25→50→75→100→(回0) 一档一档走，每档持续 3 秒，
//       串口打印当前档位。用 bsp_motor_set() 持续输出（非脉冲），方便示波器
//       Stop 抓波形。
//
// 示波器观测要点：
//   - 想看到 5kHz 方波细节，时基拉到 ~50µs/格（一个周期 200µs）；
//   - 占空比 0% 与 100% 是直流端点（恒高/恒低），不是方波，看到直线属正常；
//   - 25/50/75% 才是真方波，占空比逐档变化肉眼可辨。
// 测试完成后，把 application.c 里创建本任务的代码注释掉即可。
// ═══════════════════════════════════════════════════════════════════════════
void motor_pwm_test_task(void *pvParameters)
{
    (void)pvParameters;

    // 复用触摸初始化里的 LEDC 配置；若单独跑本测试（未启动 touch_scan_task），
    // 需保证马达 LEDC 已初始化。这里直接调一次初始化，幂等无副作用。
    bsp_motor_ledc_init();

    // 待测占空比档位（%）
    static const uint8_t test_levels[] = {0, 25, 50, 75, 100};
    const int n = sizeof(test_levels) / sizeof(test_levels[0]);

    ESP_LOGI(TAG, "═══ 震动 PWM 方波测试启动：每档持续 3 秒 ═══");

    while (1)
    {
        for (int i = 0; i < n; i++)
        {
            uint8_t lv = test_levels[i];
            bsp_motor_set(lv); // 持续输出该占空比，便于示波器观测
            ESP_LOGI(TAG, "震动占空比 = %u%%（%s）", lv,
                     (lv == 0)     ? "停止/恒高直流"
                     : (lv == 100) ? "满震/恒低直流"
                                   : "PWM 方波");
            vTaskDelay(pdMS_TO_TICKS(3000));
        }
    }
}

// ═══════════════════════════════════════════════════════════════════════════
// TTP223 方案：GPIO 数字读取
// ═══════════════════════════════════════════════════════════════════════════
#if BSP_USE_TTP223

void bsp_touch_init(void)
{
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    // 震动马达：LEDC PWM 初始化（独立 T2/CH4，XTAL 时钟源，初始停止）
    bsp_motor_ledc_init();

    // TTP223 OUT 引脚：输入 + 内部上拉（低有效，悬空时保持高电平不误触）
    gpio_config_t touch_conf = {
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << BSP_TOUCH_1_PIN) |
                        (1ULL << BSP_TOUCH_2_PIN) |
                        (1ULL << BSP_TOUCH_3_PIN) |
                        (1ULL << BSP_TOUCH_PREV_PIN) |
                        (1ULL << BSP_TOUCH_NEXT_PIN),
    };
    gpio_config(&touch_conf);

    ESP_LOGI(TAG, "触摸初始化完成 (TTP223 GPIO 低有效)");
    ESP_LOGI(TAG, "引脚: 头=%d 背=%d 腹=%d 前页=%d 后页=%d",
             BSP_TOUCH_1_PIN, BSP_TOUCH_2_PIN, BSP_TOUCH_3_PIN,
             BSP_TOUCH_PREV_PIN, BSP_TOUCH_NEXT_PIN);
}

// TTP223 低有效：OUT=0 表示触摸
static inline bool read_btn_pressed(const touch_btn_t *btn)
{
    return gpio_get_level(btn->pin) == 0;
}

// ═══════════════════════════════════════════════════════════════════════════
// 内置电容触摸方案：能量阈值检测
// ═══════════════════════════════════════════════════════════════════════════
#else // !BSP_USE_TTP223

static uint32_t touch_get_baseline(touch_pad_t ch)
{
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++)
    {
        touch_pad_sw_start();
        vTaskDelay(pdMS_TO_TICKS(10));
        uint32_t val;
        touch_pad_read_raw_data(ch, &val);
        sum += val;
    }
    return sum / 10;
}

void bsp_touch_init(void)
{
    s_touch_event_queue = xQueueCreate(TOUCH_EVENT_QUEUE_LEN, sizeof(touch_event_t));

    // 震动马达：LEDC PWM 初始化（独立 T2/CH4，XTAL 时钟源，初始停止）
    bsp_motor_ledc_init();

    touch_pad_init();
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_0V);
    touch_pad_config(btn_head.channel);
    touch_pad_config(btn_abdomen.channel);
    touch_pad_config(btn_back.channel);
    touch_pad_config(btn_prev_page.channel);
    touch_pad_config(btn_next_page.channel);
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_SW);
    touch_pad_filter_disable();
    vTaskDelay(pdMS_TO_TICKS(100));

    btn_head.baseline = touch_get_baseline(btn_head.channel);
    btn_abdomen.baseline = touch_get_baseline(btn_abdomen.channel);
    btn_back.baseline = touch_get_baseline(btn_back.channel);
    btn_prev_page.baseline = touch_get_baseline(btn_prev_page.channel);
    btn_next_page.baseline = touch_get_baseline(btn_next_page.channel);

    ESP_LOGI(TAG, "触摸初始化完成 (内置电容触摸)");
    ESP_LOGI(TAG, "基线: 头=%lu 腹=%lu 背=%lu 前页=%lu 后页=%lu",
             btn_head.baseline, btn_abdomen.baseline, btn_back.baseline,
             btn_prev_page.baseline, btn_next_page.baseline);
}

static int32_t read_btn_delta(touch_btn_t *btn)
{
    uint32_t val;
    touch_pad_read_raw_data(btn->channel, &val);
    return (int32_t)(val - btn->baseline);
}

static bool read_btn_pressed(touch_btn_t *btn)
{
    int32_t delta = read_btn_delta(btn);
    return (delta > TOUCH_MIN_DELTA) &&
           (delta > (int32_t)(btn->baseline * TOUCH_THRESH_PERCENT));
}

#endif // BSP_USE_TTP223

/**
 * @brief 获取触摸事件
 * 函数含义：从事件队列中获取一个触摸事件（非阻塞）
 * @param out_event 参数含义：输出参数，用于存储获取到的事件
 * @return 返回值含义：true=成功获取到事件，false=队列无事件
 */
bool bsp_touch_get_event(touch_event_t *out_event)
{
    // API含义：从FreeRTOS队列接收数据
    // API参数含义：
    //   s_touch_event_queue：队列句柄
    //   out_event：接收数据的缓冲区指针
    //   0：阻塞时间（0表示不阻塞）
    // 返回值含义：pdTRUE=成功接收，pdFALSE=失败
    return xQueueReceive(s_touch_event_queue, out_event, 0) == pdTRUE;
}

/**
 * @brief 更新身体按键状态
 * @param btn        按键结构体指针
 * @param pressed    当前是否检测到按下（原始状态）
 * @param in_combo   是否处于组合按键状态（若是则忽略单键）
 * @param now_ms     当前时间戳（毫秒）
 * @param short_evt  短按事件类型
 * @param long_evt   长按事件类型（TOUCH_EVENT_NONE 表示该键不支持长按）
 */
static void update_body_btn(touch_btn_t *btn, bool pressed, bool in_combo,
                            uint32_t now_ms, touch_event_t short_evt, touch_event_t long_evt,
                            bool pressed_raw)
{
    /* ── 持续按压闩锁：先按【原始】信号维护"真正离手"计数（问题7）──
     * 必须用 pressed_raw 而不是 pressed：后者已被 in_combo / page_blocking 掩过，
     * 那两种情况下手指其实还在，拿它计离手帧会让闩锁提前解除、等于白改。 */
    if (!pressed_raw)
    {
        if (btn->idle_frames < 255)
            btn->idle_frames++;
        if (btn->idle_frames >= TOUCH_RELEASE_CONFIRM_FRAMES)
            btn->consumed = false; // 确认真正离手，允许下一次接触再发事件
    }
    else
    {
        btn->idle_frames = 0;
    }

    // 组合键期间彻底重置，不产生单键事件
    if (in_combo)
    {
        btn->is_pressed = false;
        btn->press_count = 0;
        btn->release_count = 0;
        btn->press_start_ms = 0;
        return;
    }

    if (pressed)
    {
        btn->release_count = 0;
        if (!btn->is_pressed)
        {
            btn->press_count++;
            if (btn->press_count >= PRESS_DEBOUNCE)
            {
                btn->is_pressed = true;
                btn->press_start_ms = now_ms;
            }
        }
    }
    else
    {
        btn->press_count = 0;
        if (btn->is_pressed)
        {
            btn->release_count++;
            if (btn->release_count >= RELEASE_DEBOUNCE)
            {
                uint32_t held = now_ms - btn->press_start_ms;
                if (held >= BODY_PRESS_MIN_MS && !btn->consumed)
                {
                    /* 支持长按的键（头部）：按压超过阈值发长按事件，否则发短按 */
                    if (long_evt != TOUCH_EVENT_NONE && held >= HEAD_LONG_PRESS_MS)
                        send_touch_event(long_evt);
                    else
                        send_touch_event(short_evt);
                    btn->consumed = true; // 本次持续接触已消费，离手前不再发
                }
                btn->is_pressed = false;
                btn->press_start_ms = 0;
                btn->release_count = 0;
            }
        }
    }
}

/**
 * @brief 更新翻页键状态
 * 函数含义：处理前后翻页键的状态机、消抖、长短按判断和事件发送
 * @param btn           参数含义：按键结构体指针
 * @param pressed_raw   参数含义：当前是否检测到按下（原始状态）
 * @param blocked       参数含义：是否被互斥逻辑阻塞
 * @param now_ms        参数含义：当前时间戳（毫秒）
 * @param name          参数含义：按键名称（用于日志）
 * @param short_evt     参数含义：短按事件类型
 * @param long_evt      参数含义：长按事件类型
 * @param long_press_ms 参数含义：长按阈值时间（毫秒）
 */
static void update_page_btn(touch_btn_t *btn, bool pressed_raw, bool blocked,
                            uint32_t now_ms,
                            touch_event_t short_evt, touch_event_t long_evt,
                            uint32_t long_press_ms, bool game_mode,
                            bool press_trigger, bool vibrate_on_press)
{
    // 只有未被阻塞时才认为是有效按下
    bool pressed = pressed_raw && !blocked;

    /* ── 持续按压闩锁（问题7）──
     * 【游戏整体豁免】跳一跳靠松手 held 决定蓄力、打地鼠/赛车要快速连击，
     * 闩锁会把连续操作吞掉导致没法玩，故这两种模式行为与改动前完全一致。 */
    const bool latch_enabled = !game_mode && !press_trigger;

    /* 按【原始】信号维护"真正离手"计数：必须用 pressed_raw 而非 pressed，
     * 双耳互斥时 pressed 会变 false 但手指还在，用它会让闩锁提前解除。 */
    if (!pressed_raw)
    {
        if (btn->idle_frames < 255)
            btn->idle_frames++;
        if (btn->idle_frames >= TOUCH_RELEASE_CONFIRM_FRAMES)
            btn->consumed = false; // 确认真正离手
    }
    else
    {
        btn->idle_frames = 0;
    }

    // 检测到按下
    if (pressed)
    {
        btn->release_count = 0;
        if (!btn->is_pressed)
        {
            btn->press_count++;
            if (btn->press_count >= PRESS_DEBOUNCE)
            {
                btn->is_pressed = true;
                btn->press_start_ms = now_ms;

                /* ★【2026-08-31 问题7】震动与「按下被真正认定」同一时刻发起。
                 * 放在这里而不是扫描循环里自行判上升沿，是为了让震动判据与事件判据
                 * 完全一致（同一个 if 分支）——一次按压要么两者都发生，要么都不发生，
                 * 彻底消除「切换了却没震」的随机不一致。
                 * 仅置标志、不直接震：bsp_motor_pulse() 含 30ms 阻塞，真正打点在
                 * 扫描循环末尾事件分发之后（见 touch_scan_task 结尾）。 */
                /* 闩锁生效时，被吞掉的重复按压也不该震动 —— 否则会退回问题2
                 * 「震了却没切换」的不一致。震动判据与事件判据必须始终同源。 */
                if (vibrate_on_press && !(latch_enabled && btn->consumed))
                    s_vib_pending = true;

                /* ── 按下即触发模式（打地鼠 / 赛车）──
                 * 这两个游戏要的是「碰到就算一次敲击/换道」，等松手会白白多等
                 * 一个松手消抖（RELEASE_DEBOUNCE 帧），手感发钝。
                 * 故在按下确认的这一瞬间直接发事件，松手分支不再补发（见下方）。
                 * ⚠ 跳一跳必须走松手路径：它靠松手时的按压时长决定蓄力大小。*/
                if (press_trigger)
                    send_touch_event(short_evt);
            }
        }
    }
    // 检测到释放
    else
    {
        btn->press_count = 0;
        if (btn->is_pressed)
        {
            btn->release_count++;
            if (btn->release_count >= RELEASE_DEBOUNCE)
            {
                // 计算按下持续时间
                uint32_t held = now_ms - btn->press_start_ms;
                touch_event_t evt = TOUCH_EVENT_NONE;
                const char *kind = NULL;

                /* 记录本次按压时长，供跳一跳「长按蓄力」读取（任何视图都记，开销极小）*/
                if (held >= PAGE_SHORT_PRESS_MIN_MS)
                    s_last_page_hold_ms = held;

                if (press_trigger)
                {
                    /* 按下时已经发过事件了，松手只负责收尾（清状态），不能再发一次，
                     * 否则一次触摸会被算成两次敲击/两次换道。*/
                }
                else if (game_mode)
                {
                    /* ── 游戏视图：长按耳不再触发「回主界面」，统一当作一次翻页键操作 ──
                     * 跳一跳靠 bsp_touch_last_page_hold_ms() 读 held 决定蓄力大小，
                     * 因此无论长短，只要超过最小按压阈值都发 short_evt（游戏自行解读时长）。
                     * 这样既保留了「按住越久蓄力越大」，又不会在游戏里误触退出主界面。 */
                    if (held >= PAGE_SHORT_PRESS_MIN_MS)
                    {
                        evt = short_evt;
                        kind = "游戏蓄力";
                    }
                }
                else
                {
                    // 非游戏视图：维持原长/短按判定逻辑
                    if (held >= long_press_ms)
                    {
                        evt = long_evt;
                        kind = "长按";
                    }
                    else if (held >= PAGE_SHORT_PRESS_MIN_MS)
                    {
                        evt = short_evt;
                        kind = "短按";
                    }
                }

                /* 发送对应事件。
                 * 【闩锁】非游戏视图下，同一次持续接触只发一次：手指按住不放、
                 * 仅摩擦扭动造成的重复按压在此被吞掉（问题7）。
                 * 游戏视图（latch_enabled=false）不受影响，行为与改动前一致。 */
                if (evt != TOUCH_EVENT_NONE && !(latch_enabled && btn->consumed))
                {
                    send_touch_event(evt);
                    if (latch_enabled)
                        btn->consumed = true; // 本次持续接触已消费，离手前不再发
                }
                (void)kind;

                // 重置按键状态
                btn->is_pressed = false;
                btn->press_start_ms = 0;
                btn->release_count = 0;
            }
        }
    }
}

/**
 * @brief 触摸扫描任务
 * 函数含义：FreeRTOS任务，持续扫描所有触摸按键，处理组合按键、互斥逻辑和事件分发
 * @param pvParameters 参数含义：任务参数（未使用）
 */
void touch_scan_task(void *pvParameters)
{
    // 初始化触摸硬件
    bsp_touch_init();
    printf("\n触摸铜箔就绪\n");
    PRINT_TASK_STACK_HWM("touch_scan"); // 打印本任务栈历史最小剩余

    touch_event_t event;

    // 主循环
    while (1)
    {
        // OTA 升级：停止扫描并自删任务，让出 CPU 给固件下载（不可逆，重启前）
        if (s_touch_stop_for_ota)
        {
            ESP_LOGW("BSP_TOUCH", "OTA：触摸扫描任务退出，让出 CPU");
            vTaskDelete(NULL);
        }
#if BSP_USE_TTP223
        // TTP223 已在芯片内完成测量，直接读 GPIO
        vTaskDelay(pdMS_TO_TICKS(20));
        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms < POWER_ON_MASK_TIME)
            continue;

        bool h_raw = read_btn_pressed(&btn_head);
        bool a_raw = read_btn_pressed(&btn_abdomen);
        bool b_raw = read_btn_pressed(&btn_back);
        bool prev_raw = read_btn_pressed(&btn_prev_page);
        bool next_raw = read_btn_pressed(&btn_next_page);

        /* 【2026-08-31 问题7】闩锁专用：头部的【真·原始】电平，在串扰屏蔽之前快照。
         * 下面那句串扰屏蔽会把 h_raw 强行清 0，若拿被清过的值去数"离手帧"，
         * 按住头部期间只要碰一下耳朵就会被误判成离手、闩锁提前解除。 */
        bool h_raw_true = h_raw;

        // 翻页键有信号时屏蔽头部（防 PCB 串扰）
        if (prev_raw || next_raw)
            h_raw = false;

        bool h = h_raw, a = a_raw, b = b_raw;

        // TTP223 输出数字量，无法比较 delta 强弱，直接进组合判断
        bool combo_ha = h && a;
        bool combo_hb = h && b;
        bool combo_ab = a && b;

#else // !BSP_USE_TTP223
        touch_pad_sw_start();
        vTaskDelay(pdMS_TO_TICKS(5));

        uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        if (now_ms < POWER_ON_MASK_TIME)
        {
            vTaskDelay(pdMS_TO_TICKS(15));
            continue;
        }

        int32_t dh = read_btn_delta(&btn_head);
        int32_t da = read_btn_delta(&btn_abdomen);
        int32_t db = read_btn_delta(&btn_back);
        int32_t dp = read_btn_delta(&btn_prev_page);
        int32_t dn = read_btn_delta(&btn_next_page);

        int32_t thresh_abs = TOUCH_MIN_DELTA;

        // 翻页键串扰兜底：翻页键有信号时清零头通道 delta
        if (dn > thresh_abs || dp > thresh_abs)
            dh = 0;

        bool h_raw = (dh > thresh_abs) && (dh > (int32_t)(btn_head.baseline * TOUCH_THRESH_PERCENT));
        bool a_raw = (da > thresh_abs) && (da > (int32_t)(btn_abdomen.baseline * TOUCH_THRESH_PERCENT));
        bool b_raw = (db > thresh_abs) && (db > (int32_t)(btn_back.baseline * TOUCH_THRESH_PERCENT));
        bool prev_raw = (dp > thresh_abs) && (dp > (int32_t)(btn_prev_page.baseline * TOUCH_THRESH_PERCENT));
        bool next_raw = (dn > thresh_abs) && (dn > (int32_t)(btn_next_page.baseline * TOUCH_THRESH_PERCENT));

        bool h = h_raw, a = a_raw, b = b_raw;

        /* 【问题7】闩锁用的头部原始信号。本分支的串扰屏蔽发生在算 h_raw 之【前】
         * （上面把 dh 清 0），拿不到未屏蔽的值，故直接取 h_raw。
         * 本分支当前未编译（BSP_USE_TTP223=1），保留以确保切换方案时仍可编译。 */
        bool h_raw_true = h_raw;

        // 身体键组合过滤：多键同时按时只保留 delta 足够大的
        {
            int body_cnt = (int)h + (int)a + (int)b;
            if (body_cnt > 1)
            {
                int32_t max_body = dh;
                if (da > max_body)
                    max_body = da;
                if (db > max_body)
                    max_body = db;
                int32_t body_thresh = max_body * 7 / 10;
                h = h && (dh >= body_thresh);
                a = a && (da >= body_thresh);
                b = b && (db >= body_thresh);
            }
        }

        // 翻页键互斥：只保留 delta 更大的
        if (prev_raw && next_raw)
        {
            if (dp >= dn)
                next_raw = false;
            else
                prev_raw = false;
        }

        // 翻页键按下时屏蔽所有身体键
        if (prev_raw || next_raw)
        {
            h = false;
            a = false;
            b = false;
        }

        bool combo_ha = h && a;
        bool combo_hb = h && b;
        bool combo_ab = a && b;

#endif // BSP_USE_TTP223

        /* ── 翻页键互斥逻辑 ── */
        if (s_page_owner == PAGE_OWNER_NONE)
        {
            // 前页先按下，前页占用
            if (prev_raw && !next_raw)
                s_page_owner = PAGE_OWNER_PREV;
            // 后页先按下，后页占用
            else if (next_raw && !prev_raw)
                s_page_owner = PAGE_OWNER_NEXT;
        }

        // 判断是否被阻塞
        bool prev_blocked = (s_page_owner != PAGE_OWNER_NONE && s_page_owner != PAGE_OWNER_PREV);
        bool next_blocked = (s_page_owner != PAGE_OWNER_NONE && s_page_owner != PAGE_OWNER_NEXT);

        /* ══════════════════════════════════════════════════════════
         * 【核心改动】根据当前视图决定行为
         * ══════════════════════════════════════════════════════════ */
        // API含义：获取当前UI视图状态
        ui_view_t cur_view = ui_get_current_view();

        /* 所有视图统一使用 3 秒长按阈值 */
        uint32_t long_press_ms = PAGE_LONG_PRESS_MS;

        /* 组合触摸：仅主界面启用 */
        bool body_combo_enabled = (cur_view == UI_VIEW_MAIN);
        if (body_combo_enabled)
        {
            // 头+腹组合（需连续 PRESS_DEBOUNCE 次才触发，防止滑动误触）
            if (combo_ha)
            {
                if (!s_combo_ha_active)
                {
                    if (++s_combo_ha_cnt >= PRESS_DEBOUNCE)
                    {
                        s_combo_ha_active = true;
                        send_touch_event(TOUCH_EVENT_COMBO_HEAD_ABDOMEN);
                    }
                }
            }
            else
            {
                s_combo_ha_active = false;
                s_combo_ha_cnt = 0;
            }

            // 头+背组合
            if (combo_hb)
            {
                if (!s_combo_hb_active)
                {
                    if (++s_combo_hb_cnt >= PRESS_DEBOUNCE)
                    {
                        s_combo_hb_active = true;
                        send_touch_event(TOUCH_EVENT_COMBO_HEAD_BACK);
                    }
                }
            }
            else
            {
                s_combo_hb_active = false;
                s_combo_hb_cnt = 0;
            }

            // 腹+背组合
            if (combo_ab)
            {
                if (!s_combo_ab_active)
                {
                    if (++s_combo_ab_cnt >= PRESS_DEBOUNCE)
                    {
                        s_combo_ab_active = true;
                        send_touch_event(TOUCH_EVENT_COMBO_ABDOMEN_BACK);
                    }
                }
            }
            else
            {
                s_combo_ab_active = false;
                s_combo_ab_cnt = 0;
            }
        }
        else
        {
            // 非主界面，清除组合按键状态
            s_combo_ha_active = false;
            s_combo_ha_cnt = 0;
            s_combo_hb_active = false;
            s_combo_hb_cnt = 0;
            s_combo_ab_active = false;
            s_combo_ab_cnt = 0;
        }

        /* 身体单按钮：所有视图均启用（头/腹/背在各层都有用途）：
         *   - 主界面：头/腹/背触发情绪反馈
         *   - 总设置/应用列表/游戏列表/功能页/游戏：头=确认、腹/背=返回
         *   - 闹钟编辑：头=下一步/确认、腹/背=放弃退出
         * 注意：组合键(头+腹等)仍仅主界面启用，见上方 body_combo_enabled，
         *       因此非主界面单按头/腹/背不会被组合逻辑吞掉。 */
        bool body_enabled = true;
        bool page_any_active = prev_raw || next_raw;
        if (page_any_active)
            s_page_active_ms = now_ms;
        // 翻页键松开后 100ms 内仍屏蔽身体键，等待电容耦合信号衰减
        bool page_blocking = page_any_active || ((now_ms - s_page_active_ms) < 100);

        /* ── 触觉震动反馈 ────────────────────────────────────────────────
         * 规则（按需求）：
         *   1. 左/右耳（翻页键）按下 → 震动（功能盘除外，见 page_vib_view）；
         *   2. 头部在「非主界面」（功能盘/功能页/游戏/闹钟编辑）按下 → 震动；
         *   3. 主界面（情绪界面 UI_VIEW_MAIN）的头/腹/背触摸 → 不震动。
         *
         * ★【2026-08-31 问题2】功能盘（UI_VIEW_HOME）的左右耳【不在触摸层震】。
         *   本层不知道上层有没有真的切换，而 home_jelly()（ui_port.c）在渐暗/渐亮
         *   未结束时会直接 return、不推进下标 → 「屏幕没切，却震了一下」。
         *   改由 home_jelly() 真正推进下标后回调 bsp_touch_request_vibrate() 发起。
         *
         * ★【2026-08-31 问题7】左右耳震动的判据已【下沉到 update_page_btn 内部】，
         *   与事件判据合并为同一个 if 分支（该函数 btn->is_pressed = true 处）。
         *   【原来的毛病】此处曾用【单帧】上升沿 `raw && !blocked && !is_pressed`，
         *     而事件那边用的是【累计消抖】：press_count 连续 PRESS_DEBOUNCE 帧才算按下。
         *     两者判据不同 → 用户报的现象：左耳按住时 s_page_owner=PREV、右耳被判
         *     blocked；此时松左耳立刻按右耳，右耳上升沿那一帧恰落在 owner 尚未释放的
         *     窗口内（owner 释放要求 !is_pressed && !raw 两个条件），vib 判据一次性
         *     失败且**再无第二次机会**（下一帧 raw 仍为 1，不再是上升沿）；而 press_count
         *     会在 owner 释放后继续累加到阈值，事件照常发出 → 「切换触发了，震动没触发」。
         *     同一动作有时震有时不震，差别只是按/松的相位是否跨过那一帧，故表现随机。
         *   【改后】一次按压要么两者都发生、要么都不发生，不可能再不一致。
         *
         * 头部震动仍留在本处：头部走 update_body_btn，其消抖与阻塞条件
         * （page_blocking）本就在同一帧内一次性判定，不存在上述跨帧错位问题。 */
        /* 头部震动适用视图：除【主界面】和【功能盘】外的所有视图。
         * ★【2026-08-31】功能盘排除的理由与左右耳完全相同（见下方 page_vib_view）：
         *   功能盘里头部短按=进入功能、长按=退出回主界面，两者都可能被上层拒绝
         *   （进入要过 !s_home_animating + 拿锁；退出要过各自的渐变状态判断），
         *   在触摸层无条件震就会出现「震了却没进/没退」。改由上层确认真正生效后
         *   回调 bsp_touch_request_vibrate() 发起，做到「切换成功才震」。
         *   其余视图（功能页/游戏/闹钟编辑）头部语义单一、不存在被拒绝的情况，
         *   仍保持按下即震，手感更跟手。 */
        /* ★【2026-08-31】头部震动一律改为「上层确认动作真正生效后回调」，触摸层
         *   只负责【还没有接管的那些视图】按下即震。
         *
         * 已由上层接管（本处不震，见各函数内的 bsp_touch_request_vibrate 调用）：
         *   · UI_VIEW_MAIN          — 主界面头部短按=情绪(自带震动序列)、长按=进功能盘
         *   · UI_VIEW_HOME          — 功能盘头部短按=进入功能 / 长按=退出回主界面
         *   · UI_VIEW_FUNCTION_MENU — 功能页头部长按=退出回功能盘（function_menu_exit_fade）
         *   · UI_VIEW_GAME          — 游戏内头部长按=退出回功能盘（game_exit_fade）
         * 这四类的共同点是：上层都有"可能拒绝本次操作"的分支（渐变进行中 / 拿不到锁 /
         * 重入保护），在触摸层按下即震必然出现「震了却没反应」。
         *
         * 剩下仍在本处震的：闹钟编辑等语义单一、不存在被拒绝情况的视图，保持
         * 按下即震，手感更跟手。 */
        /* ★【2026-08-31 追加】闹钟编辑页（UI_VIEW_ALARM_EDIT）也一并交给上层。
         *   需求：「闹钟和番茄钟的确定/调值改成数字真的变了再震，不是按下就切换（就震）」。
         *   闹钟编辑页的头部短按=切时/分字段、头部长按=保存退出（带 3 次闪烁动效），
         *   在触摸层按下即震会出现「震在前、数字/画面在后」的错位感，且长按被
         *   重入保护（s_alarm_fade_tmr != NULL）忽略时同样会「震了却没反应」。
         *   改由 ui_port.c 的 alarm_edit_advance / alarm_edit_confirm_with_feedback
         *   在真正改完值 / 真正起了动效之后回调 bsp_touch_request_vibrate() 发起。 */
        bool head_vib_view = (cur_view != UI_VIEW_MAIN &&
                              cur_view != UI_VIEW_HOME &&
                              cur_view != UI_VIEW_FUNCTION_MENU &&
                              cur_view != UI_VIEW_GAME &&
                              cur_view != UI_VIEW_ALARM_EDIT);
        /* 左右耳震动适用视图：除功能盘（由 home_jelly 回调发起）与闹钟编辑页外。
         * ★【2026-08-31 追加】闹钟编辑页的左右耳短按 = 十位/个位 +1，属于「改数字」，
         *   同样按需求改为【数字变完再震】，由 alarm_edit_value_next/prev 回调发起。
         * 注：番茄钟（倒计时）设定页属 UI_VIEW_FUNCTION_MENU，其左右耳调值同理，
         *   已在 countdown_bump_digit 一侧回调，见 ui_port.c。 */
        /* ★【跳一跳：按下震 → 蓄力震】跳一跳的左右耳是「按住蓄力、松手起跳」，
         *   按下震一下与蓄力语义无关，需求改为【按住全程持续震、强度随蓄力上升】。
         *   持续震只能由游戏侧按帧驱动（game_jump.c charge_vib_update），
         *   故触摸层这一下必须去掉，否则起跳前会先多出一次独立的 30ms 脉冲。
         *   仅排除跳一跳，其余游戏（打地鼠/赛车）与所有非游戏视图行为完全不变。 */
        bool jump_charge_view = (cur_view == UI_VIEW_GAME &&
                                 games_get_current() == GAME_JUMP);
        bool page_vib_view = (cur_view != UI_VIEW_HOME &&
                              cur_view != UI_VIEW_ALARM_EDIT &&
                              cur_view != UI_VIEW_FUNCTION_MENU &&
                              !jump_charge_view);

        /* ★★【2026-08-31 新增：深度待机中，五个铜箔一律「按下即唤醒」】★★
         *
         * 【修的问题】实测「只有耳朵能退低功耗、头/腹/背摸了没反应」。
         *   根因不在震动，而在【事件压根发不出来】：
         *     · 左右耳走 update_page_btn，press_trigger 之外还有「按下即认定」的分支，
         *       按下当帧就 send_touch_event → 分发 → standby_notify_activity → 唤醒；
         *     · 头/腹/背走 update_body_btn，它【只在松手时】发事件（:545-566，要等
         *       RELEASE_DEBOUNCE 帧确认离手，且按压时长要过 BODY_PRESS_MIN_MS）。
         *   于是待机中摸头/腹/背，按住的全程一个事件都没有，用户感觉"没反应"；
         *   非要等到松手才可能唤醒，手感上完全不像"一摸就醒"。
         *
         * 【为什么不能改 update_body_btn 本身】那会把正常运行态下的头/腹/背
         *   全部变成按下即触发，直接破坏现有语义：头部要靠按压时长区分
         *   短按(情绪/确认) 与 长按≥HEAD_LONG_PRESS_MS(退出/返回)，按下即触发
         *   就再也分不出长短按了。所以只能【只在待机这一种状态下】特事特办。
         *
         * 【本块做什么】待机中检测到任意身体铜箔的【上升沿】，立刻直接调
         *   standby_notify_activity() 唤醒，不经过事件队列。
         *   · 不发 touch_event：唤醒本来就该"消费掉这次触摸"（ui_port.c 的
         *     was_standby 分支就是这个语义），发了反而要靠上层再拦一次。
         *   · 同时置 s_vib_pending：这样五个铜箔唤醒都震一下，与耳朵一致
         *     （耳朵那条路的震动由 ui_port.c was_standby 分支的
         *     bsp_touch_request_vibrate() 发起，两边最终都落到同一个标志）。
         *   · 同时把三个身体键标记 consumed=true：唤醒已经消费了这次接触，
         *     用户松手时不能再补发一个短按事件去触发情绪/确认，否则就是
         *     "一摸既醒来又顺手点了个东西"。consumed 会在真正离手
         *     （TOUCH_RELEASE_CONFIRM_FRAMES 帧无信号，见 update_body_btn 开头）
         *     后自动清除，不影响下一次触摸。
         *
         * 【阻塞说明】standby_notify_activity() 内部的 standby_wake() 是整段
         *   阻塞转场（约 2.6s：渐暗→换 GIF→渐亮→读 NVS 恢复音量→舵机恢复），
         *   会把触摸任务卡住这么久。这与耳朵唤醒路径的行为完全一致
         *   （那条也是在 ui_dispatch_touch_event 里同步走完转场），属既有设计，
         *   本块不引入新的阻塞特性。转场期间的重复唤醒由 standby.c 的
         *   s_waking 原子门闩拦下，不会两套转场交织。
         *
         * 【放置位置】必须在下面三个 update_body_btn 之【前】——它们会把
         *   btn->is_pressed 置位，跑完再判上升沿就永远判不出来了（与紧邻的
         *   头部震动块同一个理由）。 */
        if (standby_is_deep_active())
        {
            /* 用未经组合/串扰屏蔽掩盖的原始信号：待机中只关心"有没有人碰"，
             * 组合键、翻页串扰屏蔽那套语义在唤醒场景下毫无意义，
             * 用被掩盖过的 h/a/b 反而会漏掉"同时碰到头和肚子"这类唤醒。 */
            bool body_touched = h_raw_true || a_raw || b_raw;
            bool body_edge = (h_raw_true && !btn_head.is_pressed) ||
                             (a_raw && !btn_abdomen.is_pressed) ||
                             (b_raw && !btn_back.is_pressed);
            if (body_touched && body_edge)
            {
                ESP_LOGI(TAG, "待机中身体铜箔按下（头%d 腹%d 背%d），立即唤醒",
                         (int)h_raw_true, (int)a_raw, (int)b_raw);

                /* 先标 consumed，再唤醒：唤醒会阻塞约 2.6s，期间本任务停摆，
                 * 若把标记放在唤醒之后，中途的状态就不是自洽的。 */
                btn_head.consumed = true;
                btn_abdomen.consumed = true;
                btn_back.consumed = true;

                /* ★★【2026-08-31 二次修正：唤醒震动必须在这里【立即同步】震】★★
                 *
                 * 【上一版为什么摸不到】上一版只置 s_vib_pending，指望循环末尾那个
                 *   统一打点去震。它确实会执行，但有两个致命问题：
                 *   ① 【时机】打点在 standby_notify_activity() 返回【之后】，而后者是
                 *      约 2.6s 的阻塞转场（渐暗→换GIF→渐亮→读NVS恢复音量→舵机恢复）。
                 *      于是"手指碰到"和"震动发生"之间隔了 2.6 秒，早已不构成反馈。
                 *   ② 【时长】那个打点调的是 bsp_motor_pulse() = 仅 30ms。对比三级
                 *      关机前提醒用的是 3000ms（STANDBY_SHUTDOWN_WARN_VIB_MS）才明显可感 ——
                 *      30ms 短到马达机械惯性还没转起来就断电了，手上几乎没有感觉。
                 *   两条叠加，结果就是实测的"日志有唤醒、手上没震动"。
                 *
                 * 【为什么必须先 restore】enter_deep_standby() 里
                 *   ledc_stop(..., idle_level=1) 已把马达通道【整个禁用】并把管脚钉在高电平，
                 *   此刻直接 ledc_set_duty 是写不出波形的（这正是本文件唯一的坑）。
                 *   而 standby_wake() 里那次 restore_motor_pwm() 在转场【末尾】才跑，
                 *   等不及。故这里先自行把通道 config 回来（幂等，同参数重复配置安全），
                 *   再驱动。转场末尾那次 restore 会把它重置回停止态，正好收尾。
                 *
                 * 【用 pulse_level 而不是 pulse】需要自定义时长：
                 *   BSP_TOUCH_WAKE_VIB_MS 取 150ms —— 足够让马达真正转起来被手感知，
                 *   又远短于三级提醒的 3000ms（那是"要关机了"的强提示，语义不同）。
                 *   强度沿用 BSP_MOTOR_DEFAULT_STRENGTH（100%），与触摸反馈一致。
                 *
                 * 【阻塞代价】本次多阻塞 BSP_TOUCH_WAKE_VIB_MS，紧接着就是 2.6s 的
                 *   唤醒转场，这点时间可以忽略，也不会影响任何按键判定（三个身体键
                 *   已在上面标了 consumed，耳朵键此刻没被按）。 */
                bsp_motor_ledc_init(); // 幂等：把被 ledc_stop 禁用的通道恢复出来
                bsp_motor_pulse_level(BSP_MOTOR_DEFAULT_STRENGTH, BSP_TOUCH_WAKE_VIB_MS);

                standby_notify_activity(); // 阻塞转场，内部幂等 + s_waking 门闩防重入
            }
        }

        /* 头部：非主界面、上升沿、未被翻页屏蔽 → 震。
         * 边沿检测依赖 btn_head.is_pressed 仍是「上一帧」值，故必须放在下面
         * update_body_btn 之前，否则 is_pressed 已被置位、上升沿检测失效。
         * 只置标志不直接震：bsp_motor_pulse() 含 30ms 阻塞，会把下面
         * update_page_btn 的「按下即触发」事件整整推迟 30ms；真正打点在循环末尾。 */
        /* 【2026-08-31 问题7】追加 !btn_head.consumed：手指按住头部不放、仅摩擦扭动时，
         * 事件已被闩锁吞掉，震动也必须一并吞掉，否则会退回「震了却没反应」的不一致。 */
        if (head_vib_view && h && !page_blocking && !btn_head.is_pressed && !btn_head.consumed)
            s_vib_pending = true;

        /* 头部：短按=情绪/游戏确认，长按≥800ms=退出/返回
         * 末参 pressed_raw 传【未经组合/屏蔽掩盖】的原始信号，供闩锁判定真正离手。 */
        update_body_btn(&btn_head, h, combo_ha || combo_hb || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_HEAD, TOUCH_EVENT_LONG_HEAD, h_raw_true);
        /* 腹/背：仅主界面情绪，不参与应用/游戏退出，无长按事件 */
        update_body_btn(&btn_abdomen, a, combo_ha || combo_ab || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_ABDOMEN, TOUCH_EVENT_NONE, a_raw);
        update_body_btn(&btn_back, b, combo_hb || combo_ab || !body_enabled || page_blocking, now_ms,
                        TOUCH_EVENT_SHORT_BACK, TOUCH_EVENT_NONE, b_raw);

        /* 游戏视图下启用「蓄力模式」：长按耳不发长按事件、改记按压时长供跳一跳读取 */
        bool game_mode = (cur_view == UI_VIEW_GAME);

        /* ── 按下即触发：仅打地鼠 / 赛车 ──
         * 这两个游戏要「碰到就算一次敲击/换道」，不等松手，手感更跟手。
         * 跳一跳排除在外：它靠松手时的按压时长决定蓄力大小，必须走松手路径。*/
        bool press_trigger = false;
        if (game_mode)
        {
            int gid = games_get_current();
            press_trigger = (gid == GAME_WHACK || gid == GAME_RACE);
        }

        /* vibrate_on_press = page_vib_view：功能盘（HOME）的左右耳震动改由 home_jelly()
         * 确认真实切换后回调 bsp_touch_request_vibrate() 发起（问题2），故这里不震；
         * 其余视图在「按下被认定」的同一刻震一次（问题7，判据与事件统一）。 */
        update_page_btn(&btn_prev_page, prev_raw, prev_blocked, now_ms,
                        TOUCH_EVENT_SHORT_PREV_PAGE, TOUCH_EVENT_LONG_PREV_PAGE, long_press_ms,
                        game_mode, press_trigger, page_vib_view);
        update_page_btn(&btn_next_page, next_raw, next_blocked, now_ms,
                        TOUCH_EVENT_SHORT_NEXT_PAGE, TOUCH_EVENT_LONG_NEXT_PAGE, long_press_ms,
                        game_mode, press_trigger, page_vib_view);

        /* 占用方释放：当按键释放后，清除占用状态 */
        if (s_page_owner == PAGE_OWNER_PREV && !btn_prev_page.is_pressed && !prev_raw)
            s_page_owner = PAGE_OWNER_NONE;
        if (s_page_owner == PAGE_OWNER_NEXT && !btn_next_page.is_pressed && !next_raw)
            s_page_owner = PAGE_OWNER_NONE;

        /* ── 事件分发：统一交给 ui_dispatch_touch_event ── */
        if (bsp_touch_get_event(&event))
        {
            ESP_LOGI(TAG, "触摸事件: %d, 当前视图: %d", (int)event, (int)ui_get_current_view());
            ui_dispatch_touch_event(event);
        }

        /* 震动放在事件分发之后：内含 30ms 阻塞，先把事件送出去再震，
         * 保证「按下即触发」的响应不被震动拖慢（标志位在上方按下沿置位）。*/
        if (s_vib_pending)
        {
            s_vib_pending = false;
            bsp_motor_pulse();
        }

        // 延时15毫秒，控制扫描周期约20毫秒（50Hz）
        vTaskDelay(pdMS_TO_TICKS(15));
    }
}
