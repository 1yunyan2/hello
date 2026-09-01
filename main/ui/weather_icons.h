/**
 * @file weather_icons.h
 * @brief 天气图标资源声明 + 中文天气现象 → 图标映射
 *
 * 【图标来源】和风天气开源图标库（https://github.com/qwd/Icons，MIT 协议）
 * 【格式】128×128 RGB565A8（带 8bit alpha 通道，可直接叠在黑色背景上）
 * 【文件】main/ui/{100,101,104,302,305,400,501,502}.c
 *         文件名沿用和风 icon 码，变量名统一加 img_wx_ 前缀
 *         （C 标识符不能以数字开头，故不能直接用 100 做变量名）
 *
 * 【为什么用中文匹配而不是 icon 码】
 * 上游天气组件 managed_components/ningzixi__weather 的 weather_info_t 结构体里
 * 只有 char* weather（中文现象），解析 JSON 时压根没取 now.icon 字段。
 * 想拿码号就得改托管组件（下次 idf.py reconfigure 会被覆盖）+ 改 NVS 结构体。
 * 用中文关键词匹配则整条数据链路一行不动，成本最低。
 */
#pragma once

#include "lvgl.h"
#include <string.h>

/* ── 8 张图标资源（定义在各自的 .c 文件里） ────────────────────────── */
extern const lv_image_dsc_t img_wx_100; /* 晴     */
extern const lv_image_dsc_t img_wx_101; /* 多云   */
extern const lv_image_dsc_t img_wx_104; /* 阴     */
extern const lv_image_dsc_t img_wx_302; /* 雷阵雨 */
extern const lv_image_dsc_t img_wx_305; /* 雨     */
extern const lv_image_dsc_t img_wx_400; /* 雪     */
extern const lv_image_dsc_t img_wx_501; /* 雾     */
extern const lv_image_dsc_t img_wx_502; /* 霾     */

/**
 * @brief 中文天气现象 → 图标 查表
 *
 * 用 strstr() 做**子串包含**匹配，不是全等比较。
 * 原因：和风返回的现象有几十种说法（小雨/中雨/大雨/阵雨/强阵雨/毛毛雨…），
 * 全列出来不现实，但它们都含"雨"字，匹配一个关键词即可覆盖一整类。
 *
 * ⚠️⚠️ 【顺序敏感，改动前必读】⚠️⚠️
 * 本表从上往下逐条试，**命中即停**。所以复合词必须排在其组成词之前：
 *   - "雷阵雨" 同时含「雷」和「雨」→「雷」必须在「雨」前，否则显示成普通雨
 *   - "雨夹雪" 同时含「雨」和「雪」→「雪」必须在「雨」前
 *   - "晴间多云" 同时含「晴」和「云」→「云」在「晴」前（视觉上更接近多云）
 * 新增条目时务必想清楚它和已有条目有没有字面包含关系。
 */
typedef struct
{
    const char *keyword;         ///< 中文关键词（子串匹配）
    const lv_image_dsc_t *image; ///< 对应图标
} weather_icon_map_t;

static const weather_icon_map_t s_weather_icon_map[] = {
    {"雷", &img_wx_302}, /* 雷阵雨/雷电 —— 必须在「雨」之前 */
    {"雪", &img_wx_400}, /* 雪/雨夹雪   —— 必须在「雨」之前 */
    {"雨", &img_wx_305}, /* 小雨/中雨/大雨/阵雨…            */
    {"雾", &img_wx_501}, /* 雾/浓雾                          */
    {"霾", &img_wx_502}, /* 霾/中度霾                        */
    {"阴", &img_wx_104}, /* 阴                               */
    {"云", &img_wx_101}, /* 多云/少云/晴间多云 —— 在「晴」前 */
    {"晴", &img_wx_100}, /* 晴                               */
};

/**
 * @brief 按天气现象文字取图标
 *
 * @param text 中文天气现象（如"多云"、"小雨"），可为 NULL
 * @return 命中的图标；**未命中或入参为空时返回 NULL**（调用方应据此隐藏图标控件）
 *
 * 未命中是正常情况（如"沙尘暴"、"冰雹"未做图标），不是错误。
 * 兜底策略为隐藏图标，绝不显示错误图标误导用户。
 */
static inline const lv_image_dsc_t *weather_icon_get(const char *text)
{
    if (text == NULL || text[0] == '\0')
        return NULL;

    const size_t n = sizeof(s_weather_icon_map) / sizeof(s_weather_icon_map[0]);
    for (size_t i = 0; i < n; i++)
    {
        if (strstr(text, s_weather_icon_map[i].keyword) != NULL)
            return s_weather_icon_map[i].image;
    }
    return NULL; /* 未命中 → 调用方隐藏图标 */
}
