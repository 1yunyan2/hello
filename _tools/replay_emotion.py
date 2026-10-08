# -*- coding: utf-8 -*-
"""
情绪动作序列【时间线重放 / 校验】脚本
================================================================================
直接从 main/ui/interaction.c 的 g_emotion_matrix[] 里解析真代码（括号配对，
非贪婪正则会被 seq 内层 `},` 截断），按 bsp_servo_move_seq_parallel 的真实语义
重放三轴时间线，回答三个问题：

  ① 每个轴这一条动作【实际跑多久】？总时长是不是 GIF 周期的（接近）整数倍？
  ② 每一步的【实际位移】是多少？有没有掉进舵机死区（0.8°）？
  ③ 有没有"头部小范围 + 快"的组合（本轮需求要避开的磨损形态）？

为什么必须重放而不能手算
--------------------------------------------------------------------------------
执行层（bsp_servo.c:1540-1632）不是"每度等 step_ms"那么简单，有四条会改变结果：

  1. base_frame_ms = 所有轴【所有段】中 servo_auto_frame_ms 的【最小值】（:1552-1563）
     —— 是全局量，不是每轴一个。
  2. frames  = int(deg × own_st) / own_fm        （:1613，整除，无 +1）
  3. per_frame = max(1, int(own_fm / base_frame_ms))  （:1618）
  4. 单段耗时 = frames × per_frame × base_frame_ms
     ⇒ 代数化简后 = deg × own_st（名义值），但【当 own_fm < base_frame_ms 时
     per_frame 被钳成 1，该段耗时会翻倍】—— 这就是"混档位会把时间算错一倍"的坑。

另有两条只影响观感不影响时长，但必须检查：
  · 纯 hold 步（位移 0）frames 被钳成 1，会先空跑 1 个粗帧才开始计停留；
  · seq_len 与实际步数不符时，多出来的步数是自欺（写大了会读到数组零值）。

用法:  python _tools/replay_emotion.py [1_1 1_2 13_1 13_2 ...]
       不带参数 = 重放矩阵里所有【未被注释】的条目。
"""

import io
import re
import sys

SRC = "main/ui/interaction.c"

# ─── 执行层常量（与 bsp_servo.c / bsp_config.h 逐字对齐，改那边必须同步改这里）──
SERVO_FRAME_MS = 20
DEADBAND_DEG = 0.8
DEADBAND_MARGIN = 1.5
FRAME_MS_MAX = 60

SPEED_MS = {  # 速度宏 → step_ms（ms/度），bsp_config.h:294-300
    "SERVO_SPEED_INSTANT": 0,
    "SERVO_SPEED_VERY_FAST": 5,
    "SERVO_SPEED_FAST": 10,
    "SERVO_SPEED_MID": 15,
    "SERVO_SPEED_SLOWER": 20,
    "SERVO_SPEED_SLOW": 25,
    "SERVO_SPEED_VERY_SLOW": 50,
}
SPEED_LABEL = {v: k.replace("SERVO_SPEED_", "") for k, v in SPEED_MS.items()}

CENTER_DEG = {"head": 90.0, "left_arm": 15.0, "right_arm": 15.0}      # bsp_config.h:329-330
CENTER_SPEED = {"head": 15, "left_arm": 10, "right_arm": 10}         # SERVO_SPEED_CENTER / _ARM
AXES = ("head", "left_arm", "right_arm")

# 素材总时长（实测 assets/gif/*.gif 逐帧 duration 累加），用于比值列
# ★2026-09-28 重测更正：1_3/1_4/1_5 旧值一律写 5000 是错的。
#   实测 = 1_1 5000 / 1_2 5000 / 1_3 7000 / 1_4 5330 / 1_5 7330 /
#          1_6 6330 / 1_7 5000 / 1_8 5000 / 1_9 5000 / 13_1 6000 / 13_2 5000
GIF_MS = {
    "1_1": 5000, "1_2": 5000, "1_3": 7000, "1_4": 5330, "1_5": 7330,
    "1_6": 6330, "1_7": 5000, "1_8": 5000, "1_9": 5000,
    "2_1": 5000, "2_2": 5000, "2_3": 5000, "2_4": 5000,
    "13_1": 6000, "13_2": 5000,
}

SEG_LIMIT = 34  # BSP_SERVO_SEQ_MAX_POINTS：终点 1 + 序列 32 + 归中 1


# ══════════════════════════════════════════════════════════════════════════════
#  一、解析 C 源码
# ══════════════════════════════════════════════════════════════════════════════
def strip_comments(text):
    """去掉 // 行注释与 /* */ 块注释；字符串字面量内的 // 不会出现（本文件里没有）。"""
    out, i, n = [], 0, len(text)
    while i < n:
        if text.startswith("//", i):
            j = text.find("\n", i)
            i = n if j < 0 else j
        elif text.startswith("/*", i):
            j = text.find("*/", i + 2)
            i = n if j < 0 else j + 2
        else:
            out.append(text[i])
            i += 1
    return "".join(out)


def match_block(text, start):
    """start 指向 '{'，返回 (块内容, 块后一个字符的下标)。"""
    depth, i = 0, start
    while i < len(text):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:i], i + 1
        i += 1
    raise ValueError("花括号不配对")


def parse_axis(block):
    """解析一个轴的 { .target, .speed, .seq{...}, .seq_len, .seq_speed }。"""
    def num(field):
        m = re.search(r"\.%s\s*=\s*(-?[\d.]+)f?" % field, block)
        return float(m.group(1)) if m else None

    def macro(field):
        m = re.search(r"\.%s\s*=\s*(SERVO_SPEED_\w+)" % field, block)
        return SPEED_MS[m.group(1)] if m else None

    steps = []
    sm = re.search(r"\.seq\s*=\s*\{", block)
    if sm:
        body, _ = match_block(block, sm.end() - 1)
        for raw in re.findall(r"\{([^{}]*)\}", body):
            parts = [p.strip() for p in raw.split(",")]
            angle = float(re.sub(r"f$", "", parts[0]))
            spd = SPEED_MS.get(parts[1], 0) if len(parts) > 1 else 0
            hold = int(parts[2]) if len(parts) > 2 else 0
            steps.append({"angle": angle, "speed": spd, "hold": hold})

    return {
        "target": num("target") or 0.0,
        "speed": macro("speed") or 0,
        "seq": steps,
        "seq_len": int(re.search(r"\.seq_len\s*=\s*(\d+)", block).group(1))
        if re.search(r"\.seq_len\s*=\s*(\d+)", block) else 0,
        "seq_speed": macro("seq_speed") or 0,
    }


def uncomment_one_layer(text):
    """剥掉每行开头一层 `//`（只一层）。用于分析被整块注释掉的条目。"""
    out = []
    for ln in text.split("\n"):
        m = re.match(r"^(\s*)//\s?(.*)$", ln)
        out.append(m.group(1) + m.group(2) if m else ln)
    return "\n".join(out)


def parse_matrix(uncomment=False):
    raw = io.open(SRC, encoding="utf-8").read()
    if uncomment:
        raw = uncomment_one_layer(raw)
    text = strip_comments(raw)
    start = text.index("g_emotion_matrix[] = {")
    body, _ = match_block(text, text.index("{", start))

    rows, i = [], 0
    while True:
        j = body.find("{", i)
        if j < 0:
            break
        blk, end = match_block(body, j)
        i = end + 1
        m_id = re.search(r"\.emotion_id\s*=\s*(\w+)", blk)
        m_gif = re.search(r'/gif/(\d+_\d+)\.gif', blk)
        if not m_id or not m_gif:
            continue
        row = {"id": m_id.group(1), "no": m_gif.group(1)}
        for axis in AXES:
            am = re.search(r"\.%s\s*=\s*\{" % axis, blk)
            row[axis] = parse_axis(match_block(blk, am.end() - 1)[0]) if am else None
        rows.append(row)
    return rows


# ══════════════════════════════════════════════════════════════════════════════
#  二、按执行层语义重放
# ══════════════════════════════════════════════════════════════════════════════
def auto_frame_ms(step_ms):
    if step_ms == 0:
        return SERVO_FRAME_MS
    need = step_ms * DEADBAND_DEG * DEADBAND_MARGIN
    fm = int((need + SERVO_FRAME_MS - 0.001) // SERVO_FRAME_MS) * SERVO_FRAME_MS
    return max(SERVO_FRAME_MS, min(FRAME_MS_MAX, fm))


def build_points(axis_cfg, axis_name):
    """还原 servo_manager.c:196-248 的拼串：终点 → 序列 → 归中。"""
    pts = []
    if axis_cfg["speed"] > 0:
        pts.append({"angle": axis_cfg["target"], "step": axis_cfg["speed"], "hold": 0, "tag": "target"})
    for k, s in enumerate(axis_cfg["seq"][:axis_cfg["seq_len"]]):
        st = s["speed"] or axis_cfg["seq_speed"] or axis_cfg["speed"]
        pts.append({"angle": s["angle"], "step": st, "hold": s["hold"], "tag": "seq[%d]" % k})
    if pts:
        pts.append({"angle": CENTER_DEG[axis_name], "step": CENTER_SPEED[axis_name],
                    "hold": 0, "tag": "归中"})
    return pts


def replay(row):
    cfg = {a: row[a] for a in AXES}
    pts = {a: build_points(cfg[a], a) for a in AXES if cfg[a]}

    # ① 全局基准帧 = 所有轴所有段里 auto_frame_ms 的最小值
    base = min((auto_frame_ms(p["step"]) for a in pts for p in pts[a] if p["step"]),
               default=SERVO_FRAME_MS)

    result, problems = {}, []
    for a in AXES:
        if not pts[a]:
            result[a] = None
            continue
        total, cur, steps = 0, CENTER_DEG[a], []
        for p in pts[a]:
            dst = max(10.0, min(170.0, p["angle"]))  # clamp_safe_angle
            deg = abs(dst - cur)
            fm = auto_frame_ms(p["step"])
            frames = max(1, int(deg * p["step"]) // fm)
            per_frame = max(1, int(fm // base))
            hold_frames = 0 if p["hold"] == 0 else -(-p["hold"] // base)
            dur = frames * per_frame * base + hold_frames * base
            total += dur
            steps.append({"tag": p["tag"], "from": cur, "to": dst, "deg": deg,
                          "step": p["step"], "frames": frames, "per_frame": per_frame,
                          "hold": p["hold"], "dur": dur,
                          "per_step_deg": (deg / frames) if frames else 0.0,
                          "per_ms": per_frame * base})
            cur = dst
        result[a] = {"total": total, "steps": steps}

    # ② 断言
    for a in AXES:
        r = result[a]
        if not r:
            continue
        if len(pts[a]) > SEG_LIMIT:
            problems.append("%s 轴段数 %d > 上限 %d（会被静默截断）" % (a, len(pts[a]), SEG_LIMIT))
        for s in r["steps"]:
            if s["deg"] > 0.01 and s["per_step_deg"] < DEADBAND_DEG:
                problems.append("%s %s 每步位移 %.2f° < 死区 %.1f°（会啸叫磨齿轮）"
                                % (a, s["tag"], s["per_step_deg"], DEADBAND_DEG))
            if s["hold"] < 0:
                problems.append("%s %s hold_ms 为负" % (a, s["tag"]))

    def check_seq_len(a):
        c = row[a]
        if c and c["seq_len"] != len(c["seq"]):
            problems.append("%s seq_len=%d 但实际写了 %d 步" % (a, c["seq_len"], len(c["seq"])))
    for a in AXES:
        check_seq_len(a)

    return result, problems, base


# ══════════════════════════════════════════════════════════════════════════════
#  三、打印
# ══════════════════════════════════════════════════════════════════════════════
def report(row):
    res, problems, base = replay(row)
    gif = GIF_MS.get(row["no"])
    totals = {a: res[a]["total"] for a in AXES if res[a]}
    mx = max(totals.values())

    print("=" * 78)
    print("%s  (%s)   基准帧 %dms   GIF 周期 %s" %
          (row["no"], row["id"], base, ("%dms" % gif) if gif else "?"))
    print("-" * 78)
    for a in AXES:
        r = res[a]
        if not r:
            print("  %-10s —— 不参与" % a)
            continue
        print("  %-10s 合计 %5dms   %s" % (a, r["total"], SPEED_LABEL.get(r["steps"][0]["step"], "?")))
        for s in r["steps"]:
            flag = ""
            if s["deg"] > 0.01 and s["per_step_deg"] < DEADBAND_DEG * 1.25:
                flag = "  [!]贴死区"
            if s["deg"] < 20.0 and s["step"] <= 15 and s["deg"] > 0.01:
                flag += "  [!]小幅度+快"
            print("      %-9s %6.1f° → %6.1f°  %5.1f°  @%-9s %4d帧/%4dms  hold%4dms%s"
                  % (s["tag"], s["from"], s["to"], s["deg"],
                     SPEED_LABEL.get(s["step"], str(s["step"])), s["frames"], s["dur"] - s["hold"], s["hold"], flag))
    if gif:
        print("  → 最长轴 %dms / GIF %dms = %.3f  (%s)" %
              (mx, gif, mx / gif, "整拍" if abs(mx / gif - round(mx / gif)) < 0.06 else "非整数倍 ⚠️"))
    if problems:
        print("  [!] 问题 %d 处：" % len(problems))
        for p in problems:
            print("     - " + p)
    else:
        print("  [OK] 自检通过")
    print()


def main():
    args = sys.argv[1:]
    uncomment = "--all" in args
    want = {a for a in args if not a.startswith("--")}
    rows = [r for r in parse_matrix(uncomment) if not want or r["no"] in want]
    print("解析到 %d 条未注释条目\n" % len(rows))
    for r in rows:
        report(r)


if __name__ == "__main__":
    main()
