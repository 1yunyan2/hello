---
name: idle-pool-neutral-only
description: 空闲轮播池只放真中性 1_1~1_9；2026-08-31「空闲收窄」按旧编号把 2_x(傲娇)/3_x(兴奋) 误当中性留在池里
metadata:
  node_type: memory
  type: project
  originSessionId: fbbf2d77-73da-4aaf-b40b-1e1ad8c930fe
  modified: 2026-09-28T08:54:20.757Z
---

2026-09-28 把主界面**空闲轮播池**收敛为「真中性 9 张 = `1_1~1_9`」。
涉及两个必须成对改的数组（都在 `main/ui/ui_port.c`）：
- `s_main_gif_table`（GIF + {幅度,方向,速度,次数,往返}）
- `s_idle_actions`（同序的空闲舵机动作，`ia_custom_action_t` / 旧 `ActionStep_t`）
- 两者由 `_Static_assert(IDLE_ACTION_COUNT == MAIN_GIF_COUNT)` 强制等长 → **只改一处会编译报错**。

**🔴 为什么要重做（核心教训）**
2026-08-31 那次「空闲收窄」是按**旧编号认知**做的：当时以为 `1_x/2_x/3_x` 都是中性，于是留下
「1_x 中性(3) + 2_x 中性聆听(3) + 3_x 中性表情(3)」。但按 `gif_src/_名称对照.txt`：
**`2_x` = 傲娇、`3_x` = 兴奋** —— 空闲池里实际混着 6 张强情绪，与「收窄」的初衷正好相反。
⇒ **代码注释里的中性别名一律不可信**（本例注释写着"中性聆听/中性表情"，实际是傲娇/兴奋），
判断某张图属于哪一组的唯一依据是 `gif_src/_名称对照.txt`（或 `assets/gif/` 实际文件）。
⇒ 同族错误还有：`s_idle_actions` 的 idx 注释、`s_listening_actions` 里 `3_3.gif` 被当「好奇倾听」用。

**⚠️ `#if 0` 里那 32 条是【旧编号体系】的死数据**
主表 `#if 0` 块内是 `4_4 / 5_3 / 5_4 / 6_3 / 7_3 / 7_4 / 8_3 / 9_2~9_4 / 10_2 / 10_3 / 15_1 / 15_2 / 16_1~16_3` 等，
这些文件名在 `assets/gif/` **根本不存在**（现网只到 `14_3`，共 41 张）。
⇒ **不能"把 `#if 0` 改回 `#if 1`"来恢复**，那会引用一堆不存在的文件。

**⚠️ 遗留（本次未动）**
- 触摸侧 6 位置组（`ui_port.c` 的 `emo_group_*`）填的是**基础枚举**（`EMO_HAPPY`/`EMO_CURIOUS`/`EMO_TICKLISH`…），
  而 `g_emotion_matrix` 里只有 41 条**编号枚举**（`EMO_NEUTRAL1`/`EMO_CURIOUS1`/`EMO_EXCITED1`…），
  两边交集只有 4 个（`EMO_TSUNDERE_BASE`×3、`EMO_TSUNDERE_PET`×1）→ **36 个槽位里 32 个查不到**，只打
  `未在矩阵中找到情绪 ID` 然后 return（`interaction.c` 查表处）。
- `s_speaking_actions` 引用了 `S:/gif/16_1.gif`，**该文件不存在**。
- `1_4~1_9` 新增进空闲池的动作参数是**占位**（沿 `1_1~1_3` 风格：±15°/±10°、SLOW），手感待上板调。

**How to apply**：以后凡「按情绪分组分配 GIF」的需求，先打开 `gif_src/_名称对照.txt` 核对
每一张图的真实情绪，再动 `s_main_gif_table` / `s_idle_actions` / `emo_group_*` 任一处；
改空闲池两表必须成对。相关：[[neutral_1x6_9_redesign]]、[[emotion_4x_14x_sets]]、
[[excited_emotion_four]]、[[feedback_check_memory_before_coding]]。
## ★2026-09-29 再收窄：空闲池 = 7 张（用户指定移出 1_1 / 1_7）
现役 7 条、顺序即 idx 0~6：**1_2、1_3、1_4、1_5、1_6、1_8、1_9**。
1_1 与 1_7 的条目**原文保留在各自的 #if 0 里**（块内有 ★2026-09-29 标记），
恢复时必须**两表成对改回** —— 等长同序由 `_Static_assert(IDLE_ACTION_COUNT == MAIN_GIF_COUNT)` 护栏。

改动（`main/ui/ui_port.c`）：
- GIF 表：1_1 / 1_7 两条活条目整块移入第一段 `#if 0`；动作表：同样两条移入其 `#if 0`。
- 剩余条目注释 idx 全部重编（1_2→0、1_3→1、1_4→2、1_5→3、1_6→4、1_8→5、1_9→6）。
- 两处文档头 + 两处块头注释同步更新（含"启用 9 条"→"7 条"、"本表 idx 3~8"→"idx 2~6"）。
- ⚠️ 移出的两条在 `#if 0` 里仍写着 `keep_screen = false`，是死码（`keep_screen=false` 计数 9→7）。

顺带查实（同批）：GIF 表里每图自带的三轴参数（`gif_servo_action_t`）**当前完全无效** ——
`main_gif_submit_servo(entry)` 在 `ui_port.c:3578` 一带早已被注释、`with_servo` 恒 false，
空闲真正的动作只来自 `s_idle_actions`；已在块头加注。
⨀未编译未上板。
