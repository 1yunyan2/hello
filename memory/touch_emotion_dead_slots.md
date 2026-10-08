---
name: touch_emotion_dead_slots
description: 触摸情绪 6 组 36 槽里 32 槽(4 有效)指向无矩阵行的基础枚举⇒摸上去什么都不发生并打 E 日志
metadata:
  node_type: memory
  type: project
  originSessionId: 154170cf-c4cc-466f-a6f5-bd85c773d00a
  modified: 2026-09-29T03:41:09.899Z
---

# 触摸情绪大面积命中"死槽"（6 组 36 槽里 32 槽无效）

## 症状

`E INTERACTION: 未在矩阵中找到情绪 ID: 1，请在 g_emotion_matrix 中补充！`
出现在 `TOUCH: 触摸触发情绪: 1（组内随机 6 选 1）` 之后。

## 根因（2026-09-28 实测统计）

`g_emotion_matrix[]`（[interaction.c:162](main/ui/interaction.c#L162)）实有 **41 行**，
只覆盖 **22~55（EMO_NEUTRAL1~EMO_COMFORTABLE3）+ EMO_TSUNDERE_BASE(2)/PET(16)/PEEK(17)/ROLL(18)**。
枚举 0~21 里其余十几个基础态（EMO_HAPPY / CURIOUS / TICKLISH / SLEEPY / GRIEVED /
COMFORTABLE / ACT_CUTE / ANGRY / SHY / SURPRISED / SLUGGISH / HEALING / EXCITED /
SHY_RUB / COMFORTABLE_ROLL / SLUGGISH_SIT / SURPRISED_HUG / TICKLISH_WIGGLE）**一行都没有**。

而触摸组表 [ui_port.c:9720-9731](main/ui/ui_port.c#L9720-L9731) 里塞的**基本全是这些无行枚举**：

| 组 | 有效槽 / 6 |
|---|---|
| 头（位置1） | 1（只有 TSUNDERE_BASE） |
| 腹 | **0**（六个全无行） |
| 背 | 1 |
| 头+腹 | 1（只有 TSUNDERE_PET） |
| 头+背 | 1 |
| 腹+背 | **0** |

⇒ 摸上去 5/6 到 6/6 的概率**什么都不发生**，只打一条 E 日志。
全表 36 槽里只有 4 槽有效（TSUNDERE_BASE ×3 组 + TSUNDERE_PET ×1）。
（旧记录 [[idle_pool_neutral_only]] 写「36 槽里 35 个是死槽」，本次精算为 32，出入是它把三处 TSUNDERE_BASE 只算了 1 处。）

## 可选修法（二选一，待用户定）

- 把组表里的枚举换成 `EMO_*1/2/3` 新编号（矩阵里有行的那些）—— 改动小，但改的是"用户指定的情绪队列表"内容
- 或把基础枚举补进 `g_emotion_matrix` —— 要重写舵机动作序列，工作量大

⚠️ 触摸侧历史上有超范围改 `bsp_touch.c` 被叫停的记录，动手前先说清范围。

## ★2026-09-29 已修（本条现状已过期，留作病因档案）
6 个触摸位置组已全部重填为 `g_emotion_matrix` 里【真实存在】的编号枚举：
32 个非中性情绪各出现一次、组不等长（5~6 个），**中性 9 条不进触摸**（中性只归空闲轮播）。
实测：32/32 覆盖、零死枚举、零重复（逐组列在 ui_port.c 的 `emo_group_*` 注释里）。
⇒ 触摸现在**每次都必然出情绪动作**（改前 5/6~6/6 概率什么都不发生）。
⚠️ 以后新增情绪，先确认它已进 `g_emotion_matrix`，否则触摸只打日志、无声无息。
