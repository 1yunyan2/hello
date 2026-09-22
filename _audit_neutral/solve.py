# -*- coding: utf-8 -*-
"""1_6/1_7/1_9 重做版：解算 hold 让总时长精确落在目标拍数上，并校验。"""
import math

SPEED = dict(VERY_FAST=5, FAST=10, MID=15, SLOWER=20, SLOW=25, VERY_SLOW=50)
SLOW, SLOWER = SPEED['SLOW'], SPEED['SLOWER']
C_HEAD, C_ARM = 90.0, 15.0
CS_HEAD, CS_ARM = SPEED['MID'], SPEED['FAST']


def auto_fm(st):
    return 20 if st == 0 else max(20, min(60, int((st * 1.2 + 19.999) // 20) * 20))


def u_move(deg, st, fm):
    return max(1, int(abs(deg) * st) // fm)


def u_hold(h, base):
    return 0 if h <= 0 else 1 + int(math.ceil(h / base))


class Axis:
    """target + seq；seq 里 hold 为 None 的槽位是可解的（记在 flex 里）"""
    def __init__(self, name, target, sweep, seq, seq_speed, center, start):
        self.name, self.target, self.sweep, self.seq, self.seq_speed = name, target, sweep, seq, seq_speed
        self.center, self.start = center, start
        self.base = None

    def units(self, base):
        pts = [(self.target, self.sweep, 0)]
        pts += [(a, sp or self.seq_speed, h) for (a, sp, h) in self.seq]
        pts.append((self.center, CS_HEAD if self.center == C_HEAD else CS_ARM, 'C'))
        n, cur = 0, self.start
        for dst, st, hold in pts:
            fm = auto_fm(st)
            pf = max(1, fm // base)
            if hold == 'C':
                n += u_move(abs(dst - cur), st, fm) * pf
            else:
                n += u_move(abs(dst - cur), st, fm) * pf
                n += u_hold(hold, base) * pf
            cur = dst
        return n


def solve(axes, target_units, base=40, slots=None):
    """slots: [(axis_idx, seq_idx, lo, hi)] 可自由取值的 hold 槽；先试单槽，再全 0"""
    for (ai, si, lo, hi) in slots:
        best = None
        for h in range(lo, hi + 1, 40):
            axes[ai].seq[si] = (axes[ai].seq[si][0], axes[ai].seq[si][1], h)
            if max(a.units(base) for a in axes) == target_units and all(
                    (a.sweep == 0 and not a.seq) or a.units(base) == target_units for a in axes):
                best = h
                break
        if best is not None:
            return {ai: best}
    return None


def report(name, axes, gif_ms, target_units):
    base = 40
    print("\n" + "=" * 100)
    print("%s   base_frame_ms=%d  素材周期=%dms  目标=%d拍×40ms=%dms" %
          (name, base, gif_ms, target_units, target_units * 40))
    print("=" * 100)
    res, bad = {}, []
    for a in axes:
        if a.sweep == 0 and not a.seq:
            print("  [%s] 不参与（停在 %.0f°）" % (a.name, a.start))
            res[a.name] = 0
            continue
        pts = [(a.target, a.sweep, 0)]
        pts += [(x, sp or a.seq_speed, h) for (x, sp, h) in a.seq]
        pts.append((a.center, CS_HEAD if a.center == C_HEAD else CS_ARM, 'C'))
        n, cur, mx, cnt = 0, a.start, 0, 0
        lines = []
        for dst, st, hold in pts:
            fm, pf = auto_fm(st), None
            pf = max(1, fm // base)
            deg = abs(dst - cur)
            mv = u_move(deg, st, fm) * pf
            cnt += 1
            mark = ' ❌死区' if deg > 0 and deg / max(1, mv // pf) < 0.8 else ''
            if hold == 'C':
                lines.append("    归中 →%5.1f° st=%2d 位移%5.1f°  %3d 拍 = %5dms" % (dst, st, deg, mv, mv * base))
            else:
                lines.append("    位移 →%5.1f° st=%2d 位移%5.1f°  %3d 拍 = %5dms%s" % (dst, st, deg, mv, mv * base, mark))
            if mark:
                bad.append("%s→%.0f° 每帧<0.8°" % (a.name, dst))
            n += mv
            if hold != 'C' and hold:
                hu = u_hold(hold, base) * pf
                lines.append("      ···  停 %5dms  %3d 拍 = %5dms  ← 静止" % (hold, hu, hu * base))
                n += hu
                mx = max(mx, hold + 40)
            cur = dst
        res[a.name] = n * base
        print("  [%s] 合计 %dms  (%d 拍 / %d 点)" % (a.name, n * base, n, cnt))
        for l in lines:
            print(l)
        print("     ▸ 最长静止 %dms %s | 点数 %d %s" %
              (mx, '✅' if mx <= 1000 else '⚠️>1s', cnt, '✅' if cnt <= 32 else '❌>32'))
        if cnt > 32:
            bad.append("%s 点数 %d>32" % (a.name, cnt))
    t = max(res.values())
    print("  ── 总时长 %dms = %.3f × %dms   %s" % (t, t / gif_ms, gif_ms, '✅' if t == target_units * 40 else '❌'))
    print("  " + ("✅ 断言全过" if not bad else "❌ " + " | ".join(bad)))


# ══════════════════════════════════════════════════════════════════
# 1_6 素材周期 3170ms → 79 拍 = 3160ms
h6 = Axis('头  ', 68.0, SLOW, [(68.0, 0, 320), (80.0, SLOW, 0), (66.0, SLOW, 0),
                              (88.0, SLOW, 320), (74.0, SLOW, 0), (90.0, SLOW, 0)], SLOW, C_HEAD, 90.0)
l6 = Axis('左臂', 34.0, SLOWER, [(34.0, 0, 680), (30.0, SLOWER, 400), (22.0, SLOWER, 0),
                                (36.0, SLOWER, 600), (15.0, SLOWER, 0)], SLOWER, C_ARM, 15.0)
r6 = Axis('右臂', 0.0, 0, [], 0, C_ARM, 15.0)
report("1_6 中性表情3 · 主语头（两次偏右打量 + 2 个 360ms 停，对上素材两处 380ms 静止）", [h6, l6, r6], 3170, 79)
print("1_6 原始: 头 %d 拍, 左臂 %d 拍" % (h6.units(40), l6.units(40)))

# 1_7 素材周期 1000ms → 75 拍 = 3000ms
h7 = Axis('头  ', 74.0, SLOW, [(74.0, 0, 160), (88.0, SLOW, 0), (72.0, SLOW, 0), (86.0, SLOW, 0),
                              (70.0, SLOW, 0), (86.0, SLOW, 0), (78.0, SLOW, 0), (90.0, SLOW, 0)], SLOW, C_HEAD, 90.0)
l7 = Axis('左臂', 38.0, SLOWER, [(38.0, 0, 320), (15.0, SLOWER, 360), (38.0, SLOWER, 360),
                                (15.0, SLOWER, 0)], SLOWER, C_ARM, 15.0)
r7 = Axis('右臂', 32.0, SLOWER, [(32.0, 0, 680), (15.0, SLOWER, 360), (32.0, SLOWER, 480),
                                (15.0, SLOWER, 0)], SLOWER, C_ARM, 15.0)
report("1_7 中性聆听1 · 主语两臂（全程不停）+ 头改连续小摆", [h7, l7, r7], 1000, 75)
print("1_7 原始: 头 %d 拍, 左臂 %d 拍, 右臂 %d 拍" % (h7.units(40), l7.units(40), r7.units(40)))

# 1_9 素材周期 750ms → 75 拍 = 3000ms = 4×750
h9 = Axis('头  ', 70.0, SLOW, [(70.0, 0, 200), (88.0, SLOW, 240), (70.0, SLOW, 240),
                              (88.0, SLOW, 240), (90.0, SLOW, 0)], SLOW, C_HEAD, 90.0)
l9 = Axis('左臂', 38.0, SLOWER, [(38.0, 0, 200), (20.0, SLOWER, 0), (40.0, SLOWER, 240),
                                (26.0, SLOWER, 0), (38.0, SLOWER, 200), (15.0, SLOWER, 0)], SLOWER, C_ARM, 15.0)
r9 = Axis('右臂', 34.0, SLOWER, [(34.0, 0, 320), (18.0, SLOWER, 0), (36.0, SLOWER, 240),
                                (20.0, SLOWER, 0), (34.0, SLOWER, 240), (15.0, SLOWER, 0)], SLOWER, C_ARM, 15.0)
report("1_9 中性聆听3 · 主语头（4 拍，每拍 750ms 对上素材 750ms 强事件）", [h9, l9, r9], 750, 75)
print("1_9 原始: 头 %d 拍, 左臂 %d 拍, 右臂 %d 拍" % (h9.units(40), l9.units(40), r9.units(40)))
