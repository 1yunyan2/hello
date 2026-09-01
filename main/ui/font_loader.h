#pragma once

#include "lvgl.h"

/* ═══════════════════════════════════════════════════════════════
 * 外挂 Flash 位图字体懒加载接口
 *
 * 这些字体由 lv_font_conv --format bin 生成（assets/font/ 下的 .bin），
 * 运行时经 lv_binfont_create 从外挂 Flash /S 分区加载，不占 app 分区。
 * 每个 get() 首次调用时读盘，失败回退内置字体（保证外挂未烧时不出错）。
 *
 * 数字/中文字体陆续迁往 .bin，逐步把编译进固件的 font_*.c 移出 app 分区。
 * ═══════════════════════════════════════════════════════════════ */

const lv_font_t *font_cn_12_get(void);    /* 12px 中文（体感风级℃°度 + ASCII），英文副文案 */
const lv_font_t *font_cn_16_get(void);    /* 16px 中文（大字符集 + ASCII），日历年月 */
const lv_font_t *font_cn_20_get(void);    /* 20px 中文（日历年月折中字号） */
const lv_font_t *font_cn_24_get(void);    /* 24px 中文（体感风级等） */
const lv_font_t *font_cn_32_get(void);    /* 32px 中文（年月 + 游戏用字） */
const lv_font_t *font_num_140_get(void);  /* 140px 数字 0123456789: */
const lv_font_t *font_time_144_get(void); /* 144px 数字（时间页，Heavy） */
const lv_font_t *font_cal_74_get(void);   /* 74px 数字（日历日号） */
const lv_font_t *font_alarm_118_get(void);/* 118px 数字（闹钟时间，nz.png 示例全高） */
