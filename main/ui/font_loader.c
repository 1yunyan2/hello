#include "font_loader.h"
#include "esp_log.h"

#define TAG "font_loader"

/* 通用懒加载宏：外挂 .bin 字体，首次调用时 lv_binfont_create 读盘，
 * 失败回退内置字体（避免外挂 Flash 未烧/版本不符时空指针崩溃）。
 * 与 ui_port.c 里 time_big_font_get()/wx_temp_font_get() 同一套策略。 */
#define DEFINE_BIN_FONT(fn, bin_path, fallback)                  \
    static lv_font_t *s_##fn = NULL;                             \
    static bool s_##fn##_tried = false;                          \
    const lv_font_t *fn##_get(void)                              \
    {                                                            \
        if (!s_##fn##_tried)                                     \
        {                                                        \
            s_##fn##_tried = true;                               \
            s_##fn = lv_binfont_create(bin_path);                \
            if (s_##fn == NULL)                                  \
                ESP_LOGW(TAG, "字体加载失败(%s)，回退", bin_path); \
            else                                                 \
                ESP_LOGI(TAG, "字体已加载: %s", bin_path);        \
        }                                                        \
        return s_##fn ? (const lv_font_t *)s_##fn : fallback;    \
    }

/* ⚠️ 文件名必须 ≤8.3 短名（主名≤8字符）：外挂 FAT 挂载时 sdkconfig 未开 LFN
 * （CONFIG_FATFS_LFN_NONE），超长名会 fs_open errno 22 打不开。故 font_xxx.bin 一律缩成短名。 */
/* 16px 中文：日历年月（大字符集 + 完整 ASCII）。
 * fallback 用 montserrat_14（项目 lv_conf 未启用 montserrat_16，14 是最接近的可用 ASCII 字体） */
DEFINE_BIN_FONT(font_cn_16, "S:/font/f16.bin", &lv_font_montserrat_14)

/* 20px 中文：日历年月（16 太小 / 24 太大，20 折中，字符集复用 font_cn_32） */
DEFINE_BIN_FONT(font_cn_20, "S:/font/f20.bin", &lv_font_montserrat_14)

/* 12px 中文：英文副文案（Feels Like / Wind） */
DEFINE_BIN_FONT(font_cn_12, "S:/font/f12.bin", &lv_font_montserrat_14)

/* 24px 中文（暂未使用，先备着） */
DEFINE_BIN_FONT(font_cn_24, "S:/font/f24.bin", &lv_font_montserrat_14)

/* 32px 中文：日历年月 + 游戏居中大字 */
DEFINE_BIN_FONT(font_cn_32, "S:/font/f32.bin", &lv_font_montserrat_48)

/* 140px 数字：日历大号日号 */
DEFINE_BIN_FONT(font_num_140, "S:/font/f140.bin", &lv_font_montserrat_48)

/* 144px 数字：时间页 HH:MM（Heavy） */
DEFINE_BIN_FONT(font_time_144, "S:/font/ft144.bin", &lv_font_montserrat_48)

/* 74px 数字：日历日号 */
DEFINE_BIN_FONT(font_cal_74, "S:/font/fc74.bin", &lv_font_montserrat_48)

/* 118px 数字：闹钟时间（nz.png 示例，Oswald 窄体，全高约 120px） */
DEFINE_BIN_FONT(font_alarm_118, "S:/font/f118.bin", &lv_font_montserrat_48)
