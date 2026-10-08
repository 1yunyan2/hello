---
name: idle-tempo-action-driven
description: 空闲轮播节拍源由「GIF READY 驱动」改为「动作播完驱动」(keep_screen=false)；根因=动作时长≈GIF周期再+60ms震动 ⇒ READY 那拍被 is_playing 丢弃 ⇒ GIF 多演一整轮
metadata:
  node_type: memory
  type: project
  originSessionId: fbbf2d77-73da-4aaf-b40b-1e1ad8c930fe
  modified: 2026-09-29T06:08:58.705Z
---

2026-09-29 把**空闲轮播的节拍源**从「GIF 自己播完一轮(READY)」改成「**动作播完立刻切图**」。

**症状**：舵机只做一套，GIF 演两轮（= 动作时长的两倍）；每轮都打日志
`GIFDBG: 切图被舵机挡住→丢弃 idx=N`（`ui_port.c:3975`）。用户原话：「舵机停止了，LCD 会多演示整整一次 GIF」。

**根因（不是动作数据错，矩阵一个字节没错）**：
9 条空闲动作 = `g_emotion_matrix` 的 EMO_NEUTRAL1~9【原样搬入】。用 `_tools/replay_emotion.py`
按执行层真实语义重放，**最长轴/GIF 周期 = 0.99~1.05**（1_1 超140ms、1_2 超20、1_3 超60、
1_4 超350、1_6 超310、1_7~1_9 超100~120），**再加动作前的 60ms 震动**（`s_vib_idle`，在舵机之前阻塞播）
⇒ 必然"动作比 GIF 略长"。而空闲通道的切图判据是「GIF 播完一轮(READY) **且** 动作已结束」，
READY 到达那一拍动作还在跑 ⇒ `main_gif_switch_timer_cb` 把这次切图**整条丢弃**（不是延后）
⇒ 只能再等整整一轮 READY。

⭐**同一套动作在情绪通道"好好的"、在空闲通道翻倍——差的是切图判据，不是动作数据**：
情绪通道是「阻塞跑完 → 立刻硬切」(interaction.c:2626)，空闲通道是「等 READY」。数据本就是按"整拍"调的，
在情绪通道里超出的 0.1~0.35s 被硬切吃掉、看不见；在空闲通道里它放大成整整一轮。

**改法（唯一功能性改动）**：`s_idle_actions` 9 条 `.keep_screen` true→false
（ui_port.c:2494/2538/2581/2712/2766/2812/2860/2906/2950）⇒ 动作收尾调
`ui_resume_main_gif_loop()`（interaction.c:2387-2388）→ 10ms 后 timer_cb 切图 + 投递下一动作。

- 🔴 老注释「keep_screen=true 是为了根治 flush 风暴」**是过时且自相矛盾的说法**（那段同时描述了
  "动作自己切图+动作驱动"和"gif_path=NULL+READY驱动"两个时代）。真正的回边是
  **「动作自己切图 + resume 又切一次」两条源打架**，而 `gif_path` 早已是 NULL ⇒ 回边不存在。
- READY 那条路（`main_gif_ready_cb`）保留但不再起节拍作用：动作比 GIF 长时它总被
  `interaction_is_playing()` 挡下；动作比 GIF 短时，动作驱动的切图会重建 GIF 帧定时器、
  把尚未到期的 READY 一并取消（依据 lv_gif_set_src 销毁重建 timer 的注释，见 ui_port.c:3580）。
- ⚠️**代价**：GIF 不再保证完整播完一轮。8 条观感无差别（切点差 100ms 内），但
  **1_5（动作 3920 / GIF 7330 = 0.53 拍）会被切掉近一半 —— 本次未处理**。
- 顺带：`application.c:858` 的 `ui_set_idle_carousel_enabled(false)` 已被用户注释掉、859 置 true
  ⇒ 轮播确实是开的，"上板看不到"的顾虑不成立。

⨀未编译未上板。相关：[[neutral_1x6_9_redesign]]、[[reference_servo_frame_timing]]、
[[idle_pool_neutral_only]]、[[healing_11x_redesign]]。
