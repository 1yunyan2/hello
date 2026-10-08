# -*- coding: utf-8 -*-
"""
全矩阵头部磨损扫描
================================================================================
需求1：「头部避免小范围快速，防止磨损」——用户澄清过：小范围本身没问题，
       问题是【速度快 + 幅度小】，会反复磨同一小段齿弧。

g_emotion_matrix[] 里绝大多数条目被 // 注释掉了（只有极少数在跑），但"要改多少"
问的是【全部条目】，所以本脚本先把注释层剥掉再解析。

判据（两个阈值都报，便于你定"改多少"的口径）
  严格口径：单步行程 < 20°  且  速度 ≤ MID(15ms/°)     → 红线
  宽松口径：单步行程 < 25°  且  速度 ≤ FAST(10ms/°)    → 观察项

另报一个与上面无关的机械红线：每帧位移 < 0.8°（SERVO_DEADBAND_DEG）——
舵机收不到有效指令却持续通电 ⇒ 啸叫 + 干磨齿轮。

用法: python _tools/scan_head_wear.py            # 全量汇总 + 明细
      python _tools/scan_head_wear.py -v         # 每条都打印
"""
import io
import re
import sys
import os

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import replay_emotion as R  # noqa: E402

SRC = "main/ui/interaction.c"
UNCOMMENTED = "_tmp_uncommented.c"

# ── 口径 ────────────────────────────────────────────────────────────────────
STRICT_DEG, STRICT_STEP = 20.0, 15   # < 20° 且 ≤ MID  → 红线
LOOSE_DEG, LOOSE_STEP = 25.0, 10     # < 25° 且 ≤ FAST → 观察项
SPEED_NAME = {0: "INSTANT", 5: "VERY_FAST", 10: "FAST", 15: "MID",
              20: "SLOWER", 25: "SLOW", 50: "VERY_SLOW"}


def uncomment_one_layer(text):
    """剥掉每行开头一层 `//`。只剥一层——剥多了会把原本就注释掉的内容误当代码。"""
    out = []
    for ln in text.split("\n"):
        m = re.match(r"^(\s*)//\s?(.*)$", ln)
        out.append(m.group(1) + m.group(2) if m else ln)
    return "\n".join(out)


def head_steps(axis, name):
    """还原该轴实际发出的点：target → seq[...] → 归中。返回 (步列表, 是否自动归中计入)。"""
    if not axis:
        return []
    pts = []
    if axis["speed"] > 0:
        pts.append(("target", axis["target"], axis["speed"]))
    for k, s in enumerate(axis["seq"][:axis["seq_len"]]):
        st = s["speed"] or axis["seq_speed"] or axis["speed"]
        pts.append(("seq[%d]" % k, s["angle"], st))
    return pts


def scan():
    raw = io.open(SRC, encoding="utf-8").read()
    io.open(UNCOMMENTED, "w", encoding="utf-8", newline="\n").write(uncomment_one_layer(raw))
    R.SRC = UNCOMMENTED
    return R.parse_matrix()


def main():
    rows = scan()
    verbose = "-v" in sys.argv

    strict_hits, loose_hits, deadzone_hits = [], [], []
    no_head = []

    for row in rows:
        cfg = row["head"]
        if not cfg:
            no_head.append(row)
            continue
        pts = head_steps(cfg, "head")
        cur, flags = 90.0, []
        for tag, ang, st in pts:
            dst = max(10.0, min(170.0, ang))
            deg = abs(dst - cur)
            cur = dst
            if st == 0:
                continue
            fm = R.auto_frame_ms(st)
            frames = max(1, int(deg * st) // fm)
            per_step = deg / frames if frames else 0
            item = (tag, deg, st, per_step)
            if deg > 0.01 and per_step < R.DEADBAND_DEG:
                deadzone_hits.append((row, item))
            if 0.01 < deg < STRICT_DEG and st <= STRICT_STEP:
                strict_hits.append((row, item))
                flags.append(item)
            elif 0.01 < deg < LOOSE_DEG and st <= LOOSE_STEP:
                loose_hits.append((row, item))

        if verbose or flags:
            mark = "  ★红线" if flags else ""
            print("%-6s %-16s 头共%d步%s" % (row["no"], row["id"], len(pts), mark))
            for tag, deg, st, ps in flags:
                print("        %-9s %5.1f° @%-9s 每帧%.2f°"
                      % (tag, deg, SPEED_NAME.get(st, st), ps))

    strict_rows = sorted({r["no"] for r, _ in strict_hits})
    loose_only = sorted({r["no"] for r, _ in loose_hits} - set(strict_rows))

    print("\n" + "=" * 78)
    print("总条目 %d 条；其中无头部动作 %d 条，有头部动作 %d 条"
          % (len(rows), len(no_head), len(rows) - len(no_head)))
    print("-" * 78)
    print("严格口径（<%.0f° 且 ≤MID）：命中 %d 条 / %d 个步  → %s"
          % (STRICT_DEG, len(strict_rows), len(strict_hits), " ".join(strict_rows) or "无"))
    print("宽松口径（<%.0f° 且 ≤FAST）：再补 %d 条 / %d 个步  → %s"
          % (LOOSE_DEG, len(loose_only), len(loose_hits), " ".join(loose_only) or "无"))
    print("死区红线（每帧<%.1f°）：%d 个步  → %s"
          % (R.DEADBAND_DEG, len(deadzone_hits),
             " ".join(sorted({r["no"] for r, _ in deadzone_hits})) or "无"))

    # ── 更贴物理的一张表：头部动作里有多少比例被压在"小弧"上 ────────────────
    # 齿轮只认"走过多少度"，不认"分了几步"。所以真正决定磨损集中度的不是
    # 单步大小，而是【小步累计行程 / 头部总行程】——占比越高，越说明这条情绪的
    # 头部就是在一个小齿弧里来回蹭。
    table = []
    for row in rows:
        cfg = row["head"]
        if not cfg:
            continue
        cur, tot, small_deg, small_n, all_n = 90.0, 0.0, 0.0, 0, 0
        for tag, ang, st in head_steps(cfg, "head"):
            dst = max(10.0, min(170.0, ang))
            deg = abs(dst - cur)
            cur = dst
            if st == 0 or deg <= 0.01:
                continue
            all_n += 1
            tot += deg
            if deg < STRICT_DEG:
                small_deg += deg
                small_n += 1
        if all_n:
            table.append((small_deg / tot if tot else 0, row["no"], row["id"],
                          tot, small_deg, small_n, all_n))

    table.sort(reverse=True)
    print("\n头部【小弧占比】排行（小弧 = 单步 <%.0f°；占比 = 小弧累计° / 头部总°）" % STRICT_DEG)
    print("%-6s %-16s %8s %10s %8s %6s" % ("编号", "情绪", "总行程", "小弧累计", "占比", "小步数/总步"))
    for ratio, no, eid, tot, sd, sn, an in table:
        if ratio < 0.55:
            continue
        print("%-6s %-16s %7.0f° %9.0f° %7.0f%% %5d/%-4d"
              % (no, eid, tot, sd, ratio * 100, sn, an))
    print("=" * 78)


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
