#pragma once

/**
 * @file cube3d.h
 * @brief 通用等轴测立方体（2.5D）控件 —— LVGL 9.5.0 纯 C 实现
 *
 * 用一个 lv_obj + LV_EVENT_DRAW_MAIN 事件回调，在绘制阶段用 lv_draw_triangle
 * 直接画出「顶面 + 正面 + 右侧面」三个平行四边形，拼成等轴测立方体。
 *
 * 特点：
 *   - 零额外内存：不开 canvas，不分配像素缓冲，只在绘制事件里画矢量三角形。
 *   - 大小可调（边长）、颜色可调（三面各一色）、可压扁（高度比例）。
 *   - 参数存在堆分配的结构体里挂到 user_data，对象删除时自动释放（无悬空指针）。
 *
 * 坐标约定（等轴测）：
 *   设边长 a，cos30≈0.866。对象包围盒宽 = 2*a*cos30，高 = 2*a。
 *   立方体「逻辑底面中心」对齐对象底边中点，方便像台子一样按脚底锚定。
 *
 * 压扁：squash_pct 0~100，把立方体「竖直高度」线性压到 (100-pct)% ——
 *   顶面下沉、正面/右侧面变矮，模拟跳一跳蓄力下压。
 *
 * 用法：
 *   lv_obj_t *cube = cube3d_create(parent);
 *   cube3d_set_geometry(cube, edge_len);
 *   cube3d_set_colors(cube, top, front, right);   // 或 cube3d_set_top_color 自动调暗侧面
 *   cube3d_set_squash(cube, 0);                    // 0=不压，100=满压
 *   // 定位用普通 lv_obj_set_pos / lv_obj_set_size 之外，建议用 cube3d_place() 按底边中点摆放
 */

#include "lvgl.h"

/** @brief 创建一个等轴测立方体对象（默认边长 0、需再调 set_geometry）*/
lv_obj_t *cube3d_create(lv_obj_t *parent);

/** @brief 设置边长（像素，决定顶面菱形大小）。首次调用会把厚度默认设为=边长 */
void cube3d_set_geometry(lv_obj_t *cube, int edge_len);

/** @brief 设置竖直厚度（独立于边长，可做扁台/高台等非对称外形）*/
void cube3d_set_depth(lv_obj_t *cube, int depth);

/** @brief 设置侧面亮色条带位掩码（bit i = 第 i 条带亮，共 4 条；0=无条带）*/
void cube3d_set_stripes(lv_obj_t *cube, uint8_t mask);

/**
 * @brief 设置顶面/正面几何花纹类型（0=无花纹，1~CUBE3D_PATTERN_MAX 为各种几何图案）。
 * 花纹用线条/色块/同心菱形/圆等基本形状画，颜色自动基于台面色提亮/调暗。
 * 类型一览：1=井盖双环 2=中心方块 3=十字 4=招牌横条 5=正面圆标 6=内边框
 *          7=双横线 8=三同心环 9=顶面圆点 10=对角半分 11=正面竖带 12=四角点
 *          13=双圆环靶心 14=斜分双色 15=三竖带 16=棋盘格 17=按钮圆 18=双面圆点
 *          19=礼盒双带 20=花砖点阵
 */
#define CUBE3D_PATTERN_MAX 20
void cube3d_set_pattern(lv_obj_t *cube, uint8_t pattern);

/** @brief 开/关底部半透明深色椭圆影子 */
void cube3d_set_shadow(lv_obj_t *cube, bool on);

/** @brief 直接设置三面颜色（RGB888）*/
void cube3d_set_colors(lv_obj_t *cube, uint32_t top, uint32_t front, uint32_t right);

/**
 * @brief 只给顶面色，正面/右侧面由顶面色自动调暗生成（更省心，立体阴影一致）
 * @param top_rgb 顶面 RGB888
 */
void cube3d_set_top_color(lv_obj_t *cube, uint32_t top_rgb);

/** @brief 设置压扁百分比 0~100（0=原高，100=最扁），刷新重绘 */
void cube3d_set_squash(lv_obj_t *cube, int squash_pct);

/**
 * @brief 按「不压扁时的顶面中心」把立方体摆到屏幕坐标 (cx, top_y)
 *        （小人脚踩 top_y 这条线）。压扁为自上而下：底面固定、顶面下沉，
 *        top_y 不随 squash 漂移。
 */
void cube3d_place(lv_obj_t *cube, int cx, int top_y);
