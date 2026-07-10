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
 */
void ui_force_back_to_main(void);

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
 * @brief 情绪播放完毕后恢复主界面自动随机 GIF + 舵机循环（跨线程安全）
 *
 * 由 interaction worker 在情绪播完、清 is_playing 标志后调用。仅设 pending 标记
 * + 唤醒延迟 timer，真正切图在 LVGL 线程执行；内部判 s_view==MAIN，已进功能盘则不恢复。
 */
void ui_resume_main_gif_loop(void);

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
