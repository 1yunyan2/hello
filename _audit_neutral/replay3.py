# -*- coding: utf-8 -*-
"""1_6/1_7/1_9 重做版：设计 + 精确解 hold + 校验。
模型同 replay2.py：base_frame_ms = 三轴首段 fm 的最小值；
单段耗时 = frames * per_frame * base_frame_ms，frames = int(deg*st)//own_fm（整除无+1）；
纯 hold 步先空跑 1 个 per_frame 再计时。"""
import math

FRAME = 20
SPEED = dict(VERY_FAST=5, FAST=10, MID=15, SLOWER=20, SLOW=25, VERY_SLOW=50)
SLOW, SLOWER = SPEED['SLOW'], SPEED['SLOWER']
C_HEAD, C_ARM = 90.0, 15.0
CS_HEAD, CS_ARM = SPEED['MID'], SPEED['FAST']


def auto_fm(st):
    if st == 0:
        return FRAME
    return max(FRAME, min(60, int((st * 1.2 + FRAME - 0.001) // FRAME) * FRAME))


def units(deg, st, own_fm):
    """位移段占多少个 base 节拍"""
    return max(1, int(abs(deg) * st) // own_fm)


def hold_u(h_ms, base):
    """纯 hold 段：先空跑 1 拍，再 ceil(h_ms/base) 拍"""
    return 1 + int(math.ceil(h_ms / base)) if h_ms > 0 else 0


def build(target, sweep, seq, seq_speed, center):
    """返回 [(dst, st, hold), ...]，含首段 target 与末段归中（末段带标记）"""
    pts = [(target, sweep, 0)]
    pts += [(a, sp or seq_speed or sweep, h) for (a, sp, h) in seq]
    pts.append((center, CS_HEAD if center == C_HEAD else CS_ARM, -1))   # -1 = 归中
    return pts


def eval_axis(spec, center, start, base):
    pts = build(spec['target'], spec['speed'], spec['seq'], spec['seq_speed'], center)
    total, cur, det = 0, start, []
    for dst, st, hold in pts:
        fm = auto_fm(st)
        pf = max(1, fm // base)
        deg = abs(dst - cur)
        if hold == -1:                       # 归中段
            f = max(1, int(deg * st) // fm)
            t = f * pf * base
            det.append(('归中', dst, st, deg, f, pf, t))
        else:
            f = max(1, int(deg * st) // fm)
            t = f * pf * base
            hu = hold_u(hold, base)
            det.append(('位移', dst, st, deg, f, pf, t))
            if hu:
                det.append(('停', dst, 0, 0, hu, pf, hu * pf * base))
            t += hu * pf * base
        total += t
        cur = dst
    return total, det


def report(name, specs, gif_ms, target_ms):
    act = [t[1] for t in specs if t[1]['speed'] != 0 or t[1]['seq']]
    base = min(auto_fm(s['speed']) for s in act)
    print("\n" + "=" * 96)
    print("%s   base_frame_ms=%d   素材周期=%dms   目标总时长=%dms" % (name, base, gif_ms, target_ms))
    print("=" * 96)
    res, bad = {}, []
    for tag, spec, center, start in specs:
        if spec['speed'] == 0 and not spec['seq']:
            print("  [%s] 不参与（停在 %.0f°）" % (tag, start))
            res[tag] = 0
            continue
        t, det = eval_axis(spec, center, start, base)
        res[tag] = t
        print("  [%s] 合计 %dms  (%d 点)" % (tag, t, len(det)))
        for kind, dst, st, deg, f, pf, tseg in det:
            if kind == '归中':
                print("        归中 →%5.1f° st=%2d 位移%5.1f° %3d×%d×%d = %5dms" % (dst, st, deg, f, pf, base, tseg))
            elif kind == '停':
                print("        ···  停           %3d×%d×%d = %5dms   ← 静止" % (f, pf, base, tseg))
            else:
                m = ''
                if deg > 0 and deg / f < 0.8:
                    m = ' ❌死区'
                    bad.append("%s→%.0f° 每帧%.2f°" % (tag, dst, deg / f))
                print("        位移 →%5.1f° st=%2d 位移%5.1f° %3d×%d×%d = %5dms%s" % (dst, st, deg, f, pf, base, tseg, m))
        if len(det) > 32:
            bad.append("%s 点数 %d>32" % (tag, len(det)))
        # 最长静止段
        st_max = 0
        for kind, dst, st, deg, f, pf, tseg in det:
            if kind == '停':
                st_max = max(st_max, tseg)
        print("        ▸ 最长静止段 %dms %s" % (st_max, '❌>1000' if st_max > 1000 else '✅'))
        if st_max > 1000:
            bad.append("%s 静止 %dms>1000" % (tag, st_max))
    total = max(res.values())
    print("  ── 总时长 %dms；素材周期 %dms；比值 %.3f" % (total, gif_ms, total / gif_ms))
    print("     与目标 %dms 差 %+dms   %s" % (target_ms, total - target_ms, '✅' if abs(total - target_ms) <= 40 else '❌'))
    if abs(total - target_ms) > 40:
        bad.append("总时长偏差 %+dms" % (total - target_ms))
    print("  " + ("✅ 断言全过" if not bad else "❌ " + " | ".join(bad)))
    return res


# ══════════════════════════════════════════════════════════════════════
# 1_6  素材周期 3170ms → 目标 3160ms(=79 拍×40ms，量化下限)
#   素材：0-1000 快拍8下 / 1000-1380 静380 / 1380-2500 渐进+高潮(峰+1580) / 2500-2880 静380 / 2880-3170 收尾
#   舵机做不到 120ms 一拍的快拍（40ms 粒度下最快 1.6°/帧，8° 也要 200ms）⇒ 压成 2 次大行程 + 2 个 320ms 停
report("1_6 中性表情3 · 主语头(两次偏右打量 + 2 个 320ms 停，对上素材两处静止)", [
    ('头  ', dict(target=68.0, speed=SLOW, seq_speed=SLOW, seq=[
        (68.0, 0, 280), (82.0, SLOW, 0), (66.0, SLOW, 0),
        (88.0, SLOW, 280), (74.0, SLOW, 0), (90.0, SLOW, 0)]), C_HEAD, 90.0),
    ('左臂', dict(target=34.0, speed=SLOWER, seq_speed=SLOWER, seq=[
        (34.0, 0, 1000), (22.0, SLOWER, 0), (36.0, SLOWER, 760), (15.0, SLOWER, 0)]), C_ARM, 15.0),
    ('右臂', dict(target=0.0, speed=0, seq_speed=0, seq=[]), C_ARM, 15.0),
], 3170, 3160)

# 1_7  素材周期 1000ms → 目标 3000ms(3 周期)
#   素材：全程 diff 恒 490~520、零停顿（匀速三角波）⇒ 舵机必须"全程不停"
report("1_7 中性聆听1 · 主语两臂(全程不停的反相往复) + 头改连续小摆(旧版头停 2400ms)", [
    ('头  ', dict(target=74.0, speed=SLOW, seq_speed=SLOW, seq=[
        (74.0, 0, 0), (88.0, SLOW, 0), (72.0, SLOW, 0), (86.0, SLOW, 0),
        (70.0, SLOW, 0), (84.0, SLOW, 240), (68.0, SLOW, 0), (82.0, SLOW, 0), (90.0, SLOW, 0)]), C_HEAD, 90.0),
    ('左臂', dict(target=38.0, speed=SLOWER, seq_speed=SLOWER, seq=[
        (38.0, 0, 360), (15.0, SLOWER, 360), (38.0, SLOWER, 360), (15.0, SLOWER, 0)]), C_ARM, 15.0),
    ('右臂', dict(target=32.0, speed=SLOWER, seq_speed=SLOWER, seq=[
        (32.0, 0, 560), (15.0, SLOWER, 360), (32.0, SLOWER, 480), (15.0, SLOWER, 0)]), C_ARM, 15.0),
], 1000, 3000)

# 1_9  素材周期 750ms（实测！不是旧记录的 3000ms）→ 目标 3000ms = 4×750
#   素材：每 750ms 一次强事件(眼形互换，约 250ms 完成)，之间 500ms 回落
report("1_9 中性聆听3 · 主语头(4 拍，每拍 ~750ms 对上素材的 750ms 强事件) + 两臂错开", [
    ('头  ', dict(target=70.0, speed=SLOW, seq_speed=SLOW, seq=[
        (70.0, 0, 200), (88.0, SLOW, 240), (70.0, SLOW, 240), (88.0, SLOW, 320), (90.0, SLOW, 0)]), C_HEAD, 90.0),
    ('左臂', dict(target=38.0, speed=SLOWER, seq_speed=SLOWER, seq=[
        (38.0, 0, 240), (20.0, SLOWER, 0), (40.0, SLOWER, 240), (26.0, SLOWER, 0),
        (38.0, SLOWER, 200), (15.0, SLOWER, 0)]), C_ARM, 15.0),
    ('右臂', dict(target=34.0, speed=SLOWER, seq_speed=SLOWER, seq=[
        (34.0, 0, 360), (18.0, SLOWER, 0), (36.0, SLOWER, 240), (20.0, SLOWER, 0),
        (34.0, SLOWER, 240), (15.0, SLOWER, 0)]), C_ARM, 15.0),
], 750, 3000)

# 1_8（已通过，仅作对照）
report("1_8 中性聆听2 · 【已通过，不动】", [
    ('头  ', dict(target=84.0, speed=SLOW, seq_speed=SLOW, seq=[
        (84.0, 0, 180), (88.0, SLOW, 500), (79.0, SLOW, 200), (86.0, SLOW, 840)]), C_HEAD, 90.0),
    ('左臂', dict(target=28.0, speed=SLOWER, seq_speed=SLOWER, seq=[(28.0, 0, 1400)]), C_ARM, 15.0),
    ('右臂', dict(target=15.0, speed=SLOWER, seq_speed=SLOWER, seq=[(15.0, 0, 500), (28.0, SLOWER, 900)]), C_ARM, 15.0),
], 2500, 2480)
