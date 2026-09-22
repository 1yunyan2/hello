#pragma once
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>   /* snprintf：QRCODE_MAKE_CONTENT 宏展开时需要 */
#include <string.h>  /* strlen：调用方传给 lv_qrcode_update 时需要 */
#include "ui/interaction.h" /* 引入触摸事件枚举touch_event_t */

/* ═══════════════════════════════════════════════════════════════
 * 视图状态枚举
 * 含义：定义UI界面的核心显示状态，用于触摸事件的逻辑分发
 *
 * 层级关系（新版单层横向大图标功能盘）：
 *   主界面 ─长按耳→ 功能盘(HOME) ─摸头→ 功能页(日历/闹钟/定时器/天气) 或 游戏(打地鼠)
 *   功能盘：左右耳=横向切换图标(带果冻弹动)，头部=震动确认进入并锁定，
 *           腹/背 或 长按耳=返回主界面；进入功能页后腹/背返回功能盘。
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    UI_VIEW_MAIN = 0,      // 主界面：显示时钟、触摸触发情绪反馈
    UI_VIEW_HOME,          // 功能盘（单层）：横向大图标 日历/闹钟/定时器/天气/打地鼠
    UI_VIEW_GAME,          // 具体游戏运行视图（由功能盘确认进入）
    UI_VIEW_FUNCTION_MENU, // 功能页容器：时间/闹钟/倒计时/天气（由功能盘进入）
    UI_VIEW_ALARM_EDIT,    // 闹钟编辑界面：设置闹钟时间、重复模式
} ui_view_t;

/* ═══════════════════════════════════════════════════════════════
 * 系统生命周期接口
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief UI系统初始化入口
 * 函数含义：完成SPIFFS文件系统、LVGL图形库、主界面时钟、定时器的全流程初始化
 * 调用时机：系统上电后，硬件初始化完成后调用
 */
void ui_init(void);

/**
 * @brief 点亮屏幕并全屏显示首次配网提示图（pw.bin）
 *
 * 函数含义：自包含地完成 LCD + LVGL 初始化，贴一张静态配网提示图后开背光。
 * 调用时机：【仅首次配网】——bsp_board_wifi_main() 检测到设备未配网、在
 *           esp_wifi_start() 与 BLE controller 拉起【之前】调用。已配网开机
 *           路径完全不经过本函数，UI 仍按原顺序在 WiFi 之后初始化。
 * @note 只贴静态图、不播 GIF，贴完 LVGL 即静默，故可安全早于 BLE（见 BUG-026）。
 * @note 内部各步均带幂等早退，配网完成后 ui_init() 可正常继续。
 */
void ui_show_provision_image(void);

/* ═══════════════════════════════════════════════════════════════
 * 数据更新接口
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief 更新WiFi信号强度显示
 * @param rssi 参数含义：WiFi信号强度值（单位dBm，负数，值越大信号越好）
 */
void ui_update_wifi(int rssi);

/**
 * @brief 更新电池电量显示
 * @param soc 参数含义：电池电量百分比（0~100）
 */
void ui_update_battery(int soc);

/**
 * @brief 手动刷新时间显示
 * 函数含义：强制更新主界面的时钟数字和日期显示
 */
void ui_update_time(void);

/**
 * @brief 更新情绪显示
 * @param emotion 参数含义：情绪类型字符串，用于匹配对应的表情和动画
 */
void ui_update_emotion(const char *emotion);

/**
 * @brief 显示情绪面板（带动画、音效描述）
 * @param name 参数含义：情绪名称（如开心、好奇）
 * @param anim_desc 参数含义：动画效果描述（如眯眼笑+冒星星）
 * @param audio_desc 参数含义：对应音效描述（如短促笑声）
 */
void ui_show_emotion(const char *name, const char *anim_desc, const char *audio_desc);

/**
 * @brief 播放指定ID的GIF动画
 * @param anim_id 参数含义：动画唯一ID，用于匹配动画映射表
 */
void ui_play_animation(const char *anim_id);

/* ═══════════════════════════════════════════════════════════════
 * 功能菜单 / 翻页导航接口
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief 进入功能菜单界面
 * 函数含义：从主界面切换到功能菜单，默认显示时间日期页
 */
void ui_function_menu_enter(void);

/**
 * @brief 退出功能菜单，返回上一层（应用列表）
 */
void ui_function_menu_exit(void);

/**
 * @brief 倒计时到期：强制切到番茄时钟页并显示到期画面（2026-08-10 新增）
 *
 * 供 reminder 到期回调调用。无论当前在哪个界面，都会切到番茄时钟页显示
 * 红色 00:00 + 「倒计时结束」，停留 3 秒后自动回到预设选择界面。
 * 不改变 GIF、不操作舵机。
 *
 * @note 内部自行 lvgl_port_lock，可从非 LVGL 线程（reminder_task）调用。
 */
void ui_show_countdown_expired(void);

/**
 * @brief 闹钟响铃：强制切到闹钟页并显示响铃画面（2026-08-10 新增）
 *
 * 与 ui_show_countdown_expired() 同构。无论当前在哪个界面，都会切到闹钟页
 * 显示红色时间 + 「闹钟响铃」，停留 3 秒后自动回到常规闹钟页。
 * 不改变 GIF、不操作舵机。
 *
 * @note 内部自行 lvgl_port_lock，可从非 LVGL 线程（reminder_task）调用。
 */
void ui_show_alarm_ringing(void);

/* ═══════════════════════════════════════════════════════════════
 * 功能盘（单层横向大图标）导航接口
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief 进入功能盘界面（单层横向大图标）
 * 函数含义：从主界面切换到功能盘，左右耳横向切换图标（带果冻弹动），
 *           头部震动确认进入对应功能并锁定
 * 调用时机：主界面长按左/右耳
 */
void ui_home_enter(void);

/**
 * @brief 退出当前功能层，直接返回主界面
 * 函数含义：无论当前处于设置/列表/功能页哪一层，统一回到主界面并恢复 GIF
 * 调用时机：任意功能层长按左/右耳
 */
void ui_func_layer_exit_to_main(void);

/**
 * @brief 强制从任意视图（含闹钟编辑）返回主界面
 * 函数含义：供待机模块在进入低功耗时调用，确保界面不卡在某菜单/编辑页
 * 调用时机：standby 进入一级待机（enter_standby）时
 *
 * @return true=已复位到主界面（或本就在主界面）；
 *         false=取 LVGL 锁超时，本次【什么都没做】，界面仍停在原视图
 *
 * ⚠️【调用方必须检查返回值】2026-08-25 起本函数改为有返回值。原先取锁失败只是
 *   静默 return，导致 enter_deep_standby() 以为界面已复位、继续把待机时钟叠上去，
 *   实测出现「闹钟编辑页 + 低功耗时间页同框」（闹钟响铃结束会进闹钟编辑页，
 *   它用独立的 s_edit_panel，不受 ui_standby_clock_show 的清理覆盖）。
 *   返回 false 时调用方应当【整轮放弃】当前动作、下一拍重试，
 *   而不是带着"半复位"的界面继续往下走。
 */
bool ui_force_back_to_main(void);

/**
 * @brief 在功能菜单文字面板上显示「标题 + 正文」（带 LVGL 锁）
 *
 * 复用功能菜单共享面板（title + body 两个标签），供子模块（如 games.c）
 * 渲染纯文字画面，避免子模块直接持有 LVGL 对象。
 *
 * @param title 顶部标题（可为 NULL，表示不改标题）
 * @param body  居中正文（支持 \n 换行；可为 NULL）
 */
void ui_menu_show_text(const char *title, const char *body);

/**
 * @brief 显示「正在重置，请稍候…」解绑提示页
 *
 * 取消绑定/出厂重置会擦 NVS + esp_wifi_restore，期间 flash cache 被禁用，
 * 逐帧读 SPIFFS 的主界面 GIF 会卡在当前帧（看着像死机）。本接口在动 flash
 * 之前把画面切成一张纯静态文字提示并同步刷屏，避免冻帧。
 *
 * @note 必须在调用方真正擦除 NVS / 重启【之前】调用。
 */
void ui_show_unbinding(void);

/**
 * @brief 显示「固件升级中 XX%」进度页（OTA 下载过程调用，跨线程安全，内部自持 LVGL 锁）
 *
 * 隐藏主界面 GIF、独占全屏显示升级进度，给用户明确反馈并避免 GIF 逐帧读 flash 与
 * OTA 抢资源。取锁失败则跳过本次刷新（下次进度回调会再刷），无害退化。
 *
 * @param pct 下载进度百分比（0~100，越界自动裁剪）
 */
void ui_show_ota_progress(int pct);

/* ═══════════════════════════════════════════════════════════════
 * 触摸事件分发接口（由 touch_scan_task 触摸扫描任务调用）
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief 触摸事件核心分发函数
 * 函数含义：根据当前UI视图状态，将触摸事件分发到对应的处理逻辑
 * @param event 参数含义：触摸事件类型（短按/长按/组合按键等）
 */
void ui_dispatch_touch_event(touch_event_t event);

/**
 * @brief 请求把主界面 GIF 切到指定路径（情绪触发用，跨线程安全）
 *
 * 由 interaction 引擎播放情绪时调用，传入该情绪的 GIF 路径。可在任意线程调用：
 * 只设 pending 标记并唤醒 LVGL 线程延迟 timer，真正的 lv_gif_set_src 在 LVGL
 * 线程执行（BUG-010：lv_gif_set_src 必须在 LVGL 线程调）。仅在主界面生效。
 * @param gif_path 目标 GIF 路径（NULL/空串忽略）
 */
void ui_request_emotion_gif(const char *gif_path);

/**
 * @brief 请求把主界面 GIF 切到指定路径（对话状态动作用，跨线程安全，高优先级）
 *
 * 与 ui_request_emotion_gif 同机制，唯一区别：标记为「状态切图」（高优先级），
 * 不会被对话中（s_neutral_active）的「丢弃情绪切图」兜底误伤。供 interaction
 * worker 执行状态动作时切图调用。
 * @param gif_path 目标 GIF 路径（NULL/空串忽略）
 */
void ui_request_state_gif(const char *gif_path);

/**
 * @brief 通知 UI「初次联网成功」，开启一个短暂的 GIF 切图避让窗口（跨线程安全）
 *
 * 背景：初次联网瞬间 WiFi/TLS/WebSocket 连接等突发工作集中在 CPU0，会把优先级仅 5 的
 * taskLVGL 挤住；此刻若正好在跑 GIF 切图（文件 I/O + 解码首帧），会把这一拍顶死导致
 * 一次 task_wdt 误报（10s 窗口，仅联网后出现一次）。本函数开一个 FIRST_ONLINE_DEFER_MS
 * 窗口，期间 main_gif_switch_timer_cb 延后切图（不丢 pending），让联网突发先过去。
 *
 * 只应在【初次】联网主流程调用一次（重连不调用），故窗口只武装一次。仅做原子赋值，
 * 可在任意线程调用，无需持 LVGL 锁。
 */
void ui_notify_first_online(void);

/**
 * @brief 通知 UI「全部初始化已完成」，开机 logo 可以让位给正常主界面轮播
 *
 * 开机期间 logo GIF 会循环播放以覆盖不定长的初始化耗时（实测 8~21s 浮动），
 * 直到本函数置位 s_boot_ready 才切入 s_main_gif_table 轮播。
 *
 * 【跨线程】由 main 任务调用（非 LVGL 线程）：仅做单字标志写入 + 设 pending 标记，
 * 真正切图延后一拍在 LVGL 线程执行，无需持 LVGL 锁。
 */
void ui_notify_boot_ready(void);

/**
 * @brief 查询开机背光渐变（logo 渐亮/渐暗/切图后渐亮）是否正在进行中
 *
 * 用途：让"直接把背光拍到某个亮度"的调用方（典型是 bsp_board_lcd_on()）在渐变
 * 期间避让，不要中途改写背光。
 *
 * 【为什么需要】配网结束路径上，ui_init() 内部已启动 UI_BOOT_FADE_IN 渐亮，
 * 返回后 application.c 又调 bsp_board_lcd_on() 把背光直接拍到 100%；而渐变
 * timer 下一拍（4ms 后）按自己的时间进度算出较低亮度又写回去 —— 用户看到的就是
 * 渐亮途中"突然亮一下再暗回来"的回弹。已配网设备因时序不同不出现，故仅首次配网可见。
 *
 * 【线程安全】只读一个枚举变量，任意线程可调，无需持 LVGL 锁。
 *
 * @return true=渐变进行中（调用方不应改写背光）；false=空闲，可自由设置亮度
 */
bool ui_is_boot_fading(void);

/**
 * @brief 情绪播放完毕后恢复主界面自动随机 GIF + 舵机循环（跨线程安全）
 *
 * 由 interaction worker 在情绪播完、清 is_playing 标志后调用。仅设 pending 标记
 * + 唤醒延迟 timer，真正切图在 LVGL 线程执行；内部判 s_view==MAIN，已进功能盘则不恢复。
 */
void ui_resume_main_gif_loop(void);

/**
 * @brief 定格主界面 GIF（进深度待机第一步，背光渐暗前调用，跨线程安全）
 *
 * 停「切下一张」排队 + lv_gif_pause 冻结当前帧，停止解码/刷新。画面定格不黑屏，
 * 屏仍亮时定格用户无感；根治 GIF 与 standby_task 争 CPU 致背光渐暗延迟的问题。
 * 与 ui_resume_main_gif() 成对。内部加 LVGL 锁碰 gif_obj，取锁超时安全跳过。
 */
void ui_pause_main_gif(void);

/**
 * @brief 恢复主界面 GIF（退深度待机、亮屏前调用，跨线程安全）
 *
 * disp_on 后、背光渐亮前调用：GIF 从定格帧继续播的过渡发生在屏不可见时，用户无感。
 * 与 ui_pause_main_gif() 成对。内部加 LVGL 锁碰 gif_obj，取锁超时安全跳过。
 */
void ui_resume_main_gif(void);

/**
 * @brief 开启/关闭主界面「空闲态自动轮播」（空闲 GIF + 随附空闲舵机动作）
 *
 * 调试 GIF/舵机适配时置 false，彻底停掉空闲态自动轮播（ready_cb 不排下一张、
 * resume_loop 不恢复、开机不补投首张空闲动作），让「循环播放当前情绪」或「MQTT
 * 指令」独占。情绪/状态切图走 pending_path 分支，不受本开关影响。
 * 正式产品置 true 恢复待机空闲表现。默认 true。
 * @param enabled true=开启空闲轮播（默认） false=彻底禁用
 */
void ui_set_idle_carousel_enabled(bool enabled);

/**
 * @brief 显示「低功耗常亮时钟」（进深度待机，背光渐暗前调用，跨线程安全）
 *
 * 二级低功耗的终点由「渐变全黑 + 关显示控制器」改为「渐变到 10% + 常驻显示时间」，
 * 本函数负责其中的画面部分：隐藏并定格主界面 GIF，把功能盘里现成的时间页
 * （s_time_page 整组）叠在主界面之上显示。刻意【不改 s_view】（保持 UI_VIEW_MAIN），
 * 避免踩功能层空闲定时器自动复活 GIF 等坑，详见 ui_port.c 实现处注释。
 *
 * @note 必须在 ui_pause_main_gif() 之后、背光渐暗之前调用（先掐断 GIF 解码占用，
 *       再渲染时钟，沿用「先关 GIF 再降亮度」的既有契约）。
 * @note 内部自持 LVGL 锁，取锁超时安全跳过（退化为停留在定格 GIF，无残影）。
 * @note 与 ui_standby_clock_hide() 成对。
 */
void ui_standby_clock_show(void);

/**
 * @brief 隐藏「低功耗常亮时钟」（退深度待机，背光渐亮前调用，跨线程安全）
 *
 * 收起时间页整组与共享面板，并把 gif_obj unhide。GIF 的「从定格帧恢复播放 +
 * 恢复轮播排队」仍由 ui_resume_main_gif() 负责，本函数不重复。
 *
 * @note 必须在背光渐亮【之前】调用：此刻背光仍是 10%，切换过程用户基本看不见；
 *       等渐亮起来时画面已是动着的 GIF，无「时钟先亮再突然跳成 GIF」的突兀感。
 * @note 与 ui_standby_clock_show() 成对。
 */
void ui_standby_clock_hide(void);

/* ═══════════════════════════════════════════════════════════════
 * 对话状态中性 GIF 接口
 * ═══════════════════════════════════════════════════════════════ */
/**
 * @brief 会话状态对应的中性 GIF 分组（纯视觉，无舵机/震动）
 */
typedef enum
{
    NEUTRAL_IDLE = 0,  // 待机：恢复主界面自动随机循环
    NEUTRAL_LISTENING, // 用户说话（监听）
    NEUTRAL_SPEAKING,  // 大模型说话（TTS 播放）
} neutral_gif_state_t;

/**
 * @brief 按会话状态切换中性 GIF（跨线程安全，仅切图、不驱动舵机/震动）
 *
 * 由 session 状态机在状态切换点调用。LISTENING/SPEAKING 锁定屏幕在对应状态的
 * 中性 GIF（期间停掉待机自动循环）；IDLE 恢复主界面自动随机循环。仅在主界面生效。
 * @param st 目标会话状态
 */
void ui_set_neutral_gif_state(neutral_gif_state_t st);

/**
 * @brief 获取当前UI视图状态
 * @return 返回值含义：当前UI的视图枚举值ui_view_t
 */
ui_view_t ui_get_current_view(void);
