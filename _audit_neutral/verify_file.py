# -*- coding: utf-8 -*-
"""从 interaction.c 真源码里解析 EMO_NEUTRAL6/7/9 三行并重放 —— 防止手抄写错。
配对花括号解析（非贪婪正则会被 seq 内层 }, 截断）。"""
import io, re, math

SPEED = dict(VERY_FAST=5, FAST=10, MID=15, SLOWER=20, SLOW=25, VERY_SLOW=50)
SRC = io.open('main/ui/interaction.c', encoding='utf-8').read()


def fm(st):
    return 20 if st == 0 else max(20, min(60, int((st * 1.2 + 19.999) // 20) * 20))


def block(txt, start):
    """从 txt[start] 的 '{' 开始配平，返回内部文本"""
    i = txt.index('{', start)
    d, j = 0, i
    while True:
        if txt[j] == '{':
            d += 1
        elif txt[j] == '}':
            d -= 1
            if d == 0:
                return txt[i + 1:j]
        j += 1


def grab_axis(row, name):
    m = re.search(r'\.' + name + r'\s*=\s*', row)
    if not m:
        return None
    inner = block(row, m.end())
    t = re.search(r'\.target\s*=\s*([\d.]+)f', inner)
    s = re.search(r'\.speed\s*=\s*([A-Z_]+|0)', inner)
    sl = re.search(r'\.seq_len\s*=\s*(\d+)', inner)
    ss = re.search(r'\.seq_speed\s*=\s*([A-Z_]+|0)', inner)
    seq = []
    sm = re.search(r'\.seq\s*=\s*', inner)
    if sm:
        seqtxt = block(inner, sm.end())
        for a, sp, h in re.findall(r'\{\s*([\d.]+)f\s*,\s*(SERVO_SPEED_[A-Z_]+|0)\s*,\s*(\d+)\s*\}', seqtxt):
            seq.append((float(a), 0 if sp == '0' else SPEED[sp.replace('SERVO_SPEED_', '')], int(h)))
    return dict(target=float(t.group(1)), speed=0 if s.group(1) == '0' else SPEED[s.group(1).replace('SERVO_SPEED_', '')],
                seq_len=int(sl.group(1)), seq_speed=0 if ss.group(1) == '0' else SPEED[ss.group(1).replace('SERVO_SPEED_', '')],
                seq=seq)


def replay(spec, center, start, base, cs):
    pts = [(spec['target'], spec['speed'], 0)]
    pts += [(a, sp or spec['seq_speed'], h) for (a, sp, h) in spec['seq']]
    pts.append((center, cs, -1))
    n, cur, mx, cnt = 0, start, 0, 0
    lines = []
    for dst, st, hold in pts:
        f = fm(st); pf = max(1, f // base); deg = abs(dst - cur); cnt += 1
        if hold == -1:
            mv = max(1, int(deg * st) // f) * pf
            lines.append("      归中 →%5.1f° %5.1f°  %3d拍 %5dms" % (dst, deg, mv, mv * base))
        else:
            frames = max(1, int(deg * st) // f)
            mv = frames * pf
            lines.append("      位移 →%5.1f° %5.1f°  %3d拍 %5dms" % (dst, deg, mv, mv * base))
            if hold:
                hu = (1 + int(math.ceil(hold / base))) * pf
                lines.append("        ···  停 %4dms     %3d拍 %5dms  ← 静止" % (hold, hu, hu * base))
                n += hu; mx = max(mx, hold + 40)
        n += mv; cur = dst
    return n * base, cnt, mx, lines


TARGET = {'EMO_NEUTRAL6': ('中性表情3', 3170), 'EMO_NEUTRAL7': ('中性聆听1', 1000), 'EMO_NEUTRAL9': ('中性聆听3', 750)}
print("── 从 interaction.c 真源码解析重放（头 @SLOW / 臂 @SLOWER ⇒ base_frame_ms=40）──")
for eid, (nm, gifp) in TARGET.items():
    at = SRC.index('.emotion_id = ' + eid)
    row = SRC[at:SRC.index('\n    },', at)]
    print("\n" + "=" * 84)
    print("▌%s  「%s」素材周期 %dms" % (eid, nm, gifp))
    res = {}
    for name, center, start, cs in [('头', 90.0, 90.0, SPEED['MID']), ('左臂', 15.0, 15.0, SPEED['FAST']),
                                    ('右臂', 15.0, 15.0, SPEED['FAST'])]:
        sp = grab_axis(row, {'头': 'head', '左臂': 'left_arm', '右臂': 'right_arm'}[name])
        if sp['speed'] == 0 and sp['seq_len'] == 0:
            print("  [%s] 不参与" % name); res[name] = 0; continue
        t, cnt, mx, lines = replay(sp, center, start, 40, cs)
        res[name] = t
        print("  [%s] %dms  %d 点  最长静止 %dms %s" % (name, t, cnt, mx, '✅' if mx <= 1000 else '⚠️>1s'))
        for l in lines:
            print(l)
    tot = max(res.values())
    k = tot / gifp
    print("  ── 总时长 %dms = %.3f × %dms  %s" % (tot, k, gifp, '✅' if abs(k - round(k)) < 0.02 else '⚠️'))
