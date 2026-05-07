#ifndef UI_PORT_H
#define UI_PORT_H
#include <stdbool.h>
#include <stdint.h>
#include "ui/interaction.h" /* 引入触摸事件枚举touch_event_t */

/* ═══════════════════════════════════════════════════════════════
 * 视图状态枚举
 * 含义：定义UI界面的3种核心显示状态，用于触摸事件的逻辑分发
 * ═══════════════════════════════════════════════════════════════ */
typedef enum
{
    UI_VIEW_MAIN = 0,      // 主界面：显示时钟、触摸触发情绪反馈
    UI_VIEW_FUNCTION_MENU, // 功能菜单界面：时间/闹钟/倒计时/天气4个功能页
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
 * @brief 退出功能菜单，返回主界面
 */
void ui_function_menu_exit(void);

/**
 * @brief 功能菜单向后翻页
 * 函数含义：切换到下一个功能页，循环切换
 */
void ui_page_next(void);

/**
 * @brief 功能菜单向前翻页
 * 函数含义：切换到上一个功能页，循环切换
 */
void ui_page_prev(void);

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
 * @brief 获取当前UI视图状态
 * @return 返回值含义：当前UI的视图枚举值ui_view_t
 */
ui_view_t ui_get_current_view(void);

static void time_page_render_text(void);

#endif /* UI_PORT_H */