# -*- coding: utf-8 -*-
"""按 servo_exec_seq 语义重放 interaction.c 中 10_1(EMO_ACT_CUTE1) 三轴时间线。
不编译，只解析真代码。校验：seq_len 与实际步数一致 / 容量不超 / 三轴总时长对齐素材。"""
import re, sys

SRC = "main/ui/interaction.c"
src = open(SRC, encoding="utf-8").read()

i = src.index("EMO_ACT_CUTE1")
j = src.index("{", src.rindex("\n    {", 0, i))
depth = 0
for k in range(j, len(src)):
    if src[k] == "{":
        depth += 1
    elif src[k] == "}":
        depth -= 1
        if depth == 0:
            end = k + 1
            break
blk = src[j:end]
print("10_1 结构体行号 %d ~ %d" % (src[:j].count("\n") + 1, src[:end].count("\n") + 1))

SPEED = {"VERY_FAST": 5, "FAST": 10, "MID": 15, "SLOWER": 20, "SLOW": 25,
         "VERY_SLOW": 50, "INSTANT": 0}


def sp(tok):
    tok = tok.strip()
    if tok == "0":
        return 0
    m = re.search(r"SERVO_SPEED_(\w+)", tok)
    return SPEED[m.group(1)] if m else int(tok)


def axis_body(name):
    m = re.search(r"\." + name + r"\s*=\s*\{", blk)
    p = m.end() - 1
    depth = 0
    for k in range(p, len(blk)):
        if blk[k] == "{":
            depth += 1
        elif blk[k] == "}":
            depth -= 1
            if depth == 0:
                return blk[p:k + 1]
    raise SystemExit("unclosed ." + name)


def parse(name):
    body = axis_body(name)
    tgt = float(re.search(r"\.target\s*=\s*([\d.]+)f", body).group(1))
    spd = sp(re.search(r"\.speed\s*=\s*(SERVO_SPEED_\w+|\d+)", body).group(1))
    ss = sp(re.search(r"\.seq_speed\s*=\s*(SERVO_SPEED_\w+|\d+)", body).group(1))
    sl = int(re.search(r"\.seq_len\s*=\s*(\d+)", body).group(1))
    sm = re.search(r"\.seq\s*=\s*\{", body)
    p = sm.end() - 1
    depth = 0
    for k in range(p, len(body)):
        if body[k] == "{":
            depth += 1
        elif body[k] == "}":
            depth -= 1
            if depth == 0:
                se = k + 1
                break
    steps = []
    for s in re.finditer(r"\{([^{}]*)\}", body[p + 1:se - 1]):
        pr = [x.strip() for x in s.group(1).split(",")]
        a = float(pr[0].rstrip("f"))
        v = sp(pr[1]) if len(pr) > 1 else 0
        h = int(pr[2]) if len(pr) > 2 else 0
        steps.append((a, v, h))
    assert len(steps) == sl, "%s: seq_len=%d 但实际步数=%d" % (name, sl, len(steps))
    return tgt, spd, ss, steps


CENTER = {"head": 90.0, "left_arm": 15.0, "right_arm": 15.0}
CEN_SPD = {"head": 15, "left_arm": 10, "right_arm": 10}
FRAME = 20  # 零位移一步约 1 帧
ok = True
for name in ("head", "left_arm", "right_arm"):
    tgt, spd, ss, steps = parse(name)
    cur = CENTER[name]
    pts = []
    t = 0
    if spd > 0:                                  # .target：仅 speed>0 才追加
        d = abs(tgt - cur)
        t += d * spd if d > 0.8 else FRAME
        cur = tgt
        pts.append(tgt)
    for a, v, h in steps:                        # seq[]：speed 解析 step→seq→axis
        ms = v or ss or spd
        d = abs(a - cur)
        t += d * ms if d > 0.8 else FRAME
        t += h
        cur = a
        pts.append(a)
    c = CENTER[name]                             # 归中：servo_manager 恒定追加
    d = abs(c - cur)
    t += d * CEN_SPD[name] if d > 0.8 else FRAME
    pts.append(c)

    cap_pts, cap_seq = len(pts) + 1 <= 34, len(steps) <= 32
    ok &= cap_pts and cap_seq
    print("%-10s target=%5.1f spd=%3d seq_len=%2d 点数=%2d 点数+1=%2d 总时长=%5dms 末位=%5.1f  %s"
          % (name, tgt, spd, len(steps), len(pts), len(pts) + 1, t, steps[-1][0],
             "OK" if (cap_pts and cap_seq) else "★超容量"))

print("\n花括号平衡: %s (%d/%d)" % (blk.count("{") == blk.count("}"),
                                   blk.count("{"), blk.count("}")))
print("全部容量检查: %s" % ("通过" if ok else "失败"))
sys.exit(0 if ok else 1)
