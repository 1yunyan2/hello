# -*- coding: utf-8 -*-
"""
1_6~1_9 中性四张：素材节拍复核 + 舵机时间线重放校验

重放语义严格照抄 main/bsp/bsp_servo.c：
  servo_exec_seq()        : 归位角 → target → seq[] → 归中
  servo_auto_frame_ms()   : fm = clamp(ceil(st*0.8*1.5/20)*20, 20, 60)
  SEQ_LOAD_NEXT 宏        : frames = int(deg*st) // fm   (至少 1 帧)
                            hold_frames = ceil(hold_ms / base_frame_ms)
  base_frame_ms           : 三轴【首段】fm 的最小值
  单轴总时长              = Σ(frames*fm) + Σ(hold_frames*base_frame_ms)
"""
import math
from PIL import Image, ImageSequence

FRAME = 20
DEADBAND = 0.8
MARGIN = 1.5

SPEED = dict(INSTANT=0, VERY_FAST=5, FAST=10, MID=15, SLOWER=20, SLOW=25, VERY_SLOW=50)
CENTER_HEAD, CENTER_ARM = 90.0, 15.0
CENTER_SPEED_HEAD, CENTER_SPEED_ARM = SPEED['MID'], SPEED['FAST']


def auto_fm(st):
    if st == 0:
        return FRAME
    need = st * DEADBAND * MARGIN
    fm = int((need + FRAME - 0.001) // FRAME) * FRAME
    return max(FRAME, min(60, fm))


def axis_points(spec, center):
    """照抄 servo_exec_seq 的点链构造"""
    pts = []
    if spec['speed'] > 0:
        pts.append((spec['target'], spec['speed'], 0))
    for step in spec['seq']:
        a, sp, h = step
        if sp == 0:
            sp = spec['seq_speed']
        if sp == 0:
            sp = spec['speed']
        pts.append((a, sp, h))
    pts.append((center, CENTER_SPEED_ARM if center == CENTER_ARM else CENTER_SPEED_HEAD, 0))
    return pts


def axis_time(spec, center, start, base_fm):
    pts = axis_points(spec, center)
    cur, total, detail = start, 0, []
    for (dst, st, hold) in pts:
        fm = auto_fm(st)
        deg = abs(dst - cur)
        frames = max(1, int(deg * st) // fm)
        t = frames * fm
        hf = 0 if hold == 0 else int(math.ceil(hold / base_fm))
        # ★纯粹的"原地停留"步：位移为 0，仍要空跑 1 帧才进入 hold
        dummy = fm if deg == 0 and hold else 0
        detail.append((dst, st, deg, frames, t, dummy, hf, hf * base_fm))
        total += t + hf * base_fm
        cur = dst
    return total, detail


def replay(name, head, l_arm, r_arm, gif_ms, required_multiple=None, notes=""):
    specs = [('head', head, CENTER_HEAD, 90.0), ('L', l_arm, CENTER_ARM, 15.0), ('R', r_arm, CENTER_ARM, 15.0)]
    active = [s for s in specs if not (s[1]['speed'] == 0 and len(s[1]['seq']) == 0)]
    base_fm = min(auto_fm(s[1]['speed']) for s in active)
    times = {}
    for tag, spec, center, start in specs:
        if spec['speed'] == 0 and len(spec['seq']) == 0:
            continue
        t, det = axis_time(spec, center, start, base_fm)
        times[tag] = (t, det)
    total = max(t for t, _ in times.values())
    print(f"\n{'='*72}\n{name}   base_frame_ms={base_fm}   GIF={gif_ms}ms   {notes}")
    for tag, (t, det) in times.items():
        print(f"  [{tag}] 总 {t}ms   (单帧位移最低 {min((d[2]/max(1,d[3]) if d[2]>0 else 99) for d in det):.2f}°)")
        for d in det:
            dst, st, deg, frames, tseg, dummy, hf, ht = d
            print(f"      →{dst:6.1f}° st={st:2d} 位移{deg:5.1f}° {frames:3d}帧={tseg:5d}ms"
                  f" 空跑{dummy:3d} 停{hf:3d}帧={ht:5d}ms")
    for tag, (t, _) in times.items():
        n = round(t / gif_ms, 3)
        print(f"  [{tag}] {t}ms / GIF {gif_ms}ms = {n:.3f} 周期")
    print(f"  ⇒ 总时长 {total}ms = GIF 的 {total/gif_ms:.3f} 倍  {'✅' if required_multiple else ''}")
    if required_multiple:
        k = round(total / gif_ms)
        print(f"  ⇒ 最接近整数倍: {k} × {gif_ms} = {k*gif_ms}ms (差 {total - k*gif_ms:+d}ms, "
              f"{abs(total-k*gif_ms)/gif_ms*100:.1f}%)")
    # ---- 断言 ----
    bad = []
    for tag, (t, det) in times.items():
        for d in det:
            dst, st, deg, frames, tseg, dummy, hf, ht = d
            if deg > 0 and deg / frames < DEADBAND:
                bad.append(f"{tag} 每帧位移 {deg/frames:.2f}° < 死区 {DEADBAND}° @ →{dst}°")
    for tag, (t, det) in times.items():
        npts = len(det)
        if npts > 34:
            bad.append(f"{tag} 点数 {npts} > 32 上限")
    print("  " + ("✅ 断言全过" if not bad else "❌ " + " | ".join(bad)))
    return total


# ══════════════════════════════════════════════════════════════════
# 一、素材节拍复核
# ══════════════════════════════════════════════════════════════════
def gif_info(path):
    im = Image.open(path)
    durs, frames = [], []
    for f in ImageSequence.Iterator(im):
        durs.append(f.info.get('duration', 0))
        frames.append(f.convert('RGBA').copy())
    return frames, durs


def beats(path, label):
    frames, durs = gif_info(path)
    total = sum(durs)
    print(f"\n{label}: {len(frames)}帧 {total}ms 每帧{durs[0]}ms")
    # 逐帧与首帧的像素差（只看非透明像素）
    base = frames[0]
    diffs = []
    for f in frames:
        b = f.tobytes()
        diffs.append(b)
    first = diffs[0]
    series = []
    for i, b in enumerate(diffs):
        n = sum(1 for x, y in zip(first[::7], b[::7]) if x != y)
        series.append(n)
    # 找局部极大（周期）
    peaks = [i for i in range(1, len(series) - 1)
             if series[i] >= series[i-1] and series[i] > series[i+1] and series[i] > max(series) * 0.6]
    t_peaks = [sum(durs[:i]) for i in peaks]
    print(f"  差分峰值帧 idx={peaks[:12]}")
    print(f"  对应真实时间(ms)={t_peaks[:12]}")
    if len(t_peaks) > 1:
        gaps = [t_peaks[i+1] - t_peaks[i] for i in range(len(t_peaks)-1)]
        print(f"  峰间距(ms)={gaps}")
    return series, durs


for n, lbl in [(6, '1_6 中性表情3'), (7, '1_7 中性聆听1'), (8, '1_8 中性聆听2'), (9, '1_9 中性聆听3')]:
    beats(f'assets/gif/1_{n}.gif', lbl)
