---
name: reference_servo_frame_timing
description: "舵机序列真实推进步长 = per_frame×base_frame_ms，不是 own_fm；同情绪三轴须同档位"
metadata:
  type: reference
---

2026-09-21 校 1_6~1_9 时间线时查清的**执行层事实**
（bsp_servo.c，此前各组都按 `own_fm` 算总时长，是错的）。

**机制**
- `base_frame_ms` = 三轴【首段】fm 的**最小值**（bsp_servo.c:1552-1565）
- `per_frame[i] = max(1, own_fm / base_frame_ms)`（:1618）
- `frames[i] = int(deg × step_ms) / own_fm`（:1613，**整除、无 +1**）
- 主循环以 `base_frame_ms` 为节拍，某轴每 `per_frame` 个节拍才推进一帧
- ⇒ 单轴某段耗时 = `frames × per_frame × base_frame_ms`（**不是** `frames × own_fm`）

**推论（关键）**
1. 若 `own_fm < base_frame_ms`（例如 base=40 时的 MID/FAST=20），该轴每帧**仍花 40ms**，
   慢档不会像标称那样省时间。**只有 own_fm 是 base_frame_ms 的整数倍时耗时才可精确手算。**
2. ⇒ **同一情绪内让三轴同档位（fm 相同），时间线才可精确预测。**
3. fm 对照：VERY_FAST/FAST/MID → 20；**SLOWER(20) 与 SLOW(25) → 都是 40**；VERY_SLOW(50) → 60。
   故本组采用【头 @SLOW(25) + 臂 @SLOWER(20)】—— 两者 fm 都是 40，时间线干净，
   而头每度仍真的慢 25%（满足 8_x 原则②"头比四肢慢一档"）。
4. 纯 hold 步（位移 0）会先空跑 1 帧 = `base_frame_ms` 才开始计时；
   `hold_frames = ceil(hold_ms / base_frame_ms)`。
5. 归中速度（头 MID / 臂 FAST，fm=20 < base 40）同样按 base_frame_ms 推进，
   实际比标称慢一倍 —— 算总时长时必须带上。

**校验手法**：`_audit_neutral/replay2.py` 按 `servo_exec_seq` 语义重放三轴时间线，
断言 ①每帧位移 ≥ 死区 0.8° ②单轴点数 ≤ 32 ③总时长 ≈ 素材周期整数倍。


**2026-09-22 补充（做 1_6~1_9 第二轮时实测修正）**
- 🔴 **"总时长 ≈ 素材周期整数倍"实际是"最接近的整拍数"**：拍长 40ms，
  素材周期除不尽时就凑不出整数倍（1_9 的 750ms ÷ 40 = 18.75 ⇒ 4 拍只能 3000ms）。
- 🔴 **已通过的 1_8 实测 2640ms**（不是旧记的 2480ms —— 旧数是用本文件里
  "own_fm" 那套**已作废**的算法算的），比值 1.056 ⇒ **该规则的实际容忍度比 11_x 写的松**。
- **≤1s 静止红线只约束【主语轴/头】**：1_8 左臂有 1440ms 静止段却通过了。
- **快拍上限**：base=40 时最快 1.6°/帧，8° 的动作也要 200ms ⇒ 素材 120ms 一拍的节奏
  **物理不可达**（想更快只能让三轴首段全上 MID 把 base 压到 20）。

⨀ **未编译未上板**。

相关：[[neutral_1x6_9_redesign]] [[comfortable_14x_redesign]] [[sleepy_12x_redesign]] [[project_servo_motion_curve]]
