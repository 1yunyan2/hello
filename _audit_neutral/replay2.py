# -*- coding: utf-8 -*-
"""
1_6~1_9 中性四张：舵机时间线重放校验（修正版）

★修正：真实推进步长不是 own_fm，而是 per_frame*base_frame_ms
    per_frame[i] = max(1, own_fm // base_frame_ms)
  base_frame_ms = 三轴【首段】fm 的最小值
  ⇒ 若某轴 own_fm < base_frame_ms（如 base=40 时的 MID/FAST=20），
    它实际每帧仍花 base_frame_ms，而不是 own_fm。
  ⇒ 结论：同一情绪内让所有轴同档位（fm 相同）时间线才可精确预测。
    本项目采用【头 SLOW(25) + 臂 SLOWER(20)】——两者 auto_fm 都是 40，
    时间线干净，且头每度真的比臂慢 25%（满足 8_x 原则②）。
"""
import math

FRAME = 20
SPEED = dict(VERY_FAST=5, FAST=10, MID=15, SLOWER=20, SLOW=25, VERY_SLOW=50)
CENTER_HEAD, CENTER_ARM = 90.0, 15.0
CENTER_SPEED_HEAD, CENTER_SPEED_ARM = SPEED['MID'], SPEED['FAST']


def auto_fm(st):
    if st == 0:
        return FRAME
    need = st * 0.8 * 1.5
    return max(FRAME, min(60, int((need + FRAME - 0.001) // FRAME) * FRAME))


def build(spec, center):
    pts = []
    if spec['speed'] > 0:
        pts.append((spec['target'], spec['speed'], 0))
    for a, sp, h in spec['seq']:
        st = sp or spec['seq_speed'] or spec['speed']
        pts.append((a, st, h))
    pts.append((center, CENTER_SPEED_HEAD if center == CENTER_HEAD else CENTER_SPEED_ARM, 0))
    return pts


def run(spec, center, start, base_fm):
    total, det, cur = 0, [], start
    for dst, st, hold in build(spec, center):
        fm = auto_fm(st)
        pf = max(1, fm // base_fm)
        deg = abs(dst - cur)
        frames = max(1, int(deg * st) // fm)
        t = frames * pf * base_fm
        hf = 0 if hold == 0 else int(math.ceil(hold / base_fm))
        det.append((dst, st, deg, frames, t, pf, hf, hf * base_fm))
        total += t + hf * base_fm
        cur = dst
    return total, det


def check(name, head, l_arm, r_arm, gif_ms, target_mult=1):
    specs = [('头  ', head, CENTER_HEAD, 90.0), ('左臂', l_arm, CENTER_ARM, 15.0), ('右臂', r_arm, CENTER_ARM, 15.0)]
    act = [s for s in specs if not (s[1]['speed'] == 0 and not s[1]['seq'])]
    base_fm = min(auto_fm(s[1]['speed']) for s in act)
    res, bad = {}, []
    print(f"\n{'█'*76}\n█ {name}   base_frame_ms={base_fm}   素材周期={gif_ms}ms\n{'█'*76}")
    for tag, spec, center, start in specs:
        if spec['speed'] == 0 and not spec['seq']:
            print(f"  [{tag}] 不参与（钉在 {start:.0f}° 不动）")
            continue
        t, det = run(spec, center, start, base_fm)
        res[tag] = t
        print(f"  [{tag}] 合计 {t}ms")
        for dst, st, deg, frames, tseg, pf, hf, ht in det:
            mark = ''
            if deg > 0 and deg / frames < 0.8:
                mark = '  ❌死区'; bad.append(f"{tag}→{dst}° 每帧{deg/frames:.2f}°<0.8")
            print(f"        →{dst:6.1f}° st={st:2d} 位移{deg:5.1f}° {frames:3d}帧×{pf}×{base_fm}"
                  f"={tseg:5d}ms 停{hf:3d}帧={ht:5d}ms{mark}")
        if len(det) > 32:
            bad.append(f"{tag} 点数 {len(det)} > 32")
    total = max(res.values())
    print(f"  ── 总时长 {total}ms")
    for tag, t in res.items():
        print(f"     [{tag}] {t}ms = GIF 的 {t/gif_ms:.3f} 周期")
    k = round(total / gif_ms)
    print(f"  ── 最接近整数倍: {k}×{gif_ms} = {k*gif_ms}ms  差 {total-k*gif_ms:+d}ms "
          f"({abs(total-k*gif_ms)/gif_ms*100:.2f}%)")
    print("  " + ("✅ 断言全过" if not bad else "❌ " + " | ".join(bad)))
    return total


# ══════════════════════════════════════════════════════════════════
SLOW, SLOWER = SPEED['SLOW'], SPEED['SLOWER']

# ── 1_6 中性表情3：主语=头（偏右打量 + 长停）；左臂独自慢抬一点，右臂不动
t = check("1_6  中性表情3 · 主语：头（打量式，长停）· 单臂配角", dict(
    target=79.0, speed=SLOW, seq_speed=SLOW,
    seq=[(79.0, 0, 960), (86.0, SLOW, 600), (81.0, SLOW, 800)]),
    dict(target=33.0, speed=SLOWER, seq_speed=SLOWER, seq=[(33.0, 0, 2300)]),
    dict(target=0.0, speed=0, seq_speed=0, seq=[]),
    gif_ms=3165)

# ── 1_7 中性聆听1：主语=两臂（严格反相，一拍 1000ms）；头只轻偏一次
t = check("1_7  中性聆听1 · 主语：两臂（4 拍反相交替）· 头轻偏两次", dict(
    target=83.0, speed=SLOW, seq_speed=SLOW,
    seq=[(83.0, 0, 1200), (87.0, SLOW, 1200)]),
    dict(target=34.0, speed=SLOWER, seq_speed=SLOWER,
         seq=[(34.0, 0, 600), (15.0, SLOWER, 640), (34.0, SLOWER, 640), (15.0, SLOWER, 600)]),
    dict(target=15.0, speed=SLOWER, seq_speed=SLOWER,
         seq=[(15.0, 0, 960), (34.0, SLOWER, 640), (15.0, SLOWER, 640), (34.0, SLOWER, 240)]),
    gif_ms=1000)

# ── 1_8 中性聆听2：主语=头（4 个跟眨眼同步的"嗯"）；两臂极慢错开起手
t = check("1_8  中性聆听2 · 主语：头（4 次眨眼同步小动）· 两臂错开起手", dict(
    target=84.0, speed=SLOW, seq_speed=SLOW,
    seq=[(84.0, 0, 180), (88.0, SLOW, 500), (79.0, SLOW, 200), (86.0, SLOW, 840)]),
    dict(target=28.0, speed=SLOWER, seq_speed=SLOWER, seq=[(28.0, 0, 1400)]),
    dict(target=15.0, speed=SLOWER, seq_speed=SLOWER,   # ★必须填 15 基线，填 28 会多出一次上下摆
         seq=[(15.0, 0, 500), (28.0, SLOWER, 900)]),
    gif_ms=2500)

# ── 1_9 中性聆听3：主语=头（A慢漂→B急停）；两臂一起慢张→快合
t = check("1_9  中性聆听3 · 主语：头（A慢漂+B急停）· 两臂同步开合", dict(
    target=84.0, speed=SLOW, seq_speed=SLOW,
    seq=[(84.0, 0, 620), (80.0, SLOW, 600), (84.0, SLOW, 500), (76.0, SLOW, 300)]),
    dict(target=36.0, speed=SLOWER, seq_speed=SLOWER,
         seq=[(36.0, 0, 1100), (15.0, SLOWER, 500)]),
    dict(target=30.0, speed=SLOWER, seq_speed=SLOWER,
         seq=[(30.0, 0, 1300), (15.0, SLOWER, 700)]),
    gif_ms=3000)
