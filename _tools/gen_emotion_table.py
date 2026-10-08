# -*- coding: utf-8 -*-
"""
从 main/ui/interaction.c 的 g_emotion_matrix[] 抽取 41 条情绪动作，
自动生成"白话版"动作描述 CSV（给非开发人员看）。

【为什么要写脚本而不是手抄】
  表跨 3000+ 行、其中 37 条还被 // 注释掉，手抄一遍必然有错字/漏步。
  本脚本直接从真源码解析：注释行按"剥掉前导 // "还原，再配对花括号取值；
  速度宏从 bsp_config.h 读，不硬编码，改了档位会自动跟着变。

用法：python _tools/gen_emotion_table.py
输出：docs/情绪动作_白话版.csv
"""
import io
import os
import re
import csv

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SRC = os.path.join(ROOT, "main", "ui", "interaction.c")
CFG = os.path.join(ROOT, "main", "bsp", "bsp_config.h")
OUT = os.path.join(ROOT, "docs", "情绪动作_白话版.csv")

CJK = re.compile(r"[一-鿿]")


def load_speed_macros():
    """从 bsp_config.h 读 SERVO_SPEED_xxx 的真实数值（注释掉的 #define 不会匹配）。"""
    txt = io.open(CFG, encoding="utf-8").read()
    d = {}
    for line in txt.split("\n"):
        m = re.match(r"\s*#define\s+(SERVO_SPEED_\w+)\s+(\d+)U", line)
        if m:
            d[m.group(1)] = int(m.group(2))
    return d


MACROS = load_speed_macros()

# ── 速度档 → 白话词（词序按用户 2026-09-22 定稿：20ms/度 叫"慢"）────────────
SPEED_WORD = {0: "默认", 5: "很快", 10: "快", 15: "中速",
              20: "慢", 25: "很慢", 50: "极慢"}
SPEED_LEGEND = ("速度对照：很快=5ms/度　快=10ms/度　中速=15ms/度　"
                "慢=20ms/度　很慢=25ms/度　极慢=50ms/度")

# ── 轴的初始姿势与归位角（依据 bsp_config.h:329-330）──────────────────────
#   axis键, 列名, 归位角, 姿势白话名, 负方向词, 正方向词
AXES = [
    ("head",      "头部怎么动", 90.0, "正中间", "左", "右"),
    ("left_arm",  "左手怎么动", 15.0, "正垂直", "后", "前"),
    ("right_arm", "右手怎么动", 15.0, "正垂直", "后", "前"),
]

# 编号首位 → 情绪组中文名（依据 interaction.c 的分组注释）
GROUP_NAME = {
    "1": "中性", "2": "傲娇", "3": "兴奋", "4": "好奇", "5": "委屈",
    "6": "害羞", "7": "怕痒", "8": "惊喜", "9": "慵懒", "10": "撒娇",
    "11": "治愈", "12": "犯困", "13": "生气", "14": "舒服",
}


def strip_comments(text):
    """剥掉 C 注释，把被 // 注释掉的条目还原成可解析代码。

    规则：
      1. 去掉块注释 /* */；
      2. ★只剥掉【一层】行首 // —— 整个情绪条目被 // 注释掉时只需还原一层；
         若剥两层，条目内部那些"额外注释掉的备用步进"（例如
         `//  {15.0f, SERVO_SPEED_VERY_FAST, 0},`）会被误当成真步数，
         导致 seq_len 自检大面积误报（2026-09-22 踩过）；
      3. 丢掉行尾 // 之后的内容；
      4. ★关键：剥完后如果整行是纯中文散文（含汉字、且没有 = { } "），
         说明它是被注释掉的自然语言注释行，直接丢弃 —— 否则注释里的
         文字可能混进代码流干扰花括号配对。
    """
    text = re.sub(r"/\*.*?\*/", "", text, flags=re.S)
    out = []
    for line in text.split("\n"):
        m = re.match(r"^\s*//\s?", line)
        if m:
            line = line[m.end():]
        idx = line.find("//")
        if idx >= 0:
            line = line[:idx]
        if CJK.search(line) and "=" not in line and "{" not in line and '"' not in line:
            continue
        out.append(line)
    return "\n".join(out)


def match_block(text, start):
    """从 text[start] == '{' 开始配对花括号，返回 (块内容, '}'下标)。"""
    assert text[start] == "{", "起点不是花括号：%r" % text[start:start + 40]
    depth = 0
    for i in range(start, len(text)):
        c = text[i]
        if c == "{":
            depth += 1
        elif c == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:i], i
    raise ValueError("花括号未配对")


def find_field(block, field):
    m = re.search(r"\." + re.escape(field) + r"\s*=\s*([^,}\n]+)", block)
    return m.group(1).strip() if m else None


def num(expr):
    """C 字面量 → float：'68.0f'→68.0；'SERVO_SPEED_MID'→15（查 bsp_config.h）。"""
    if expr is None:
        return None
    expr = expr.strip()
    m = re.fullmatch(r"([-+]?\d+(?:\.\d+)?)f?", expr)
    if m:
        return float(m.group(1))
    return float(MACROS[expr]) if expr in MACROS else None


def parse_axis(entry, axis):
    """解析单个轴块。返回 dict；该条目里没有这个轴则返回 None。"""
    m = re.search(r"\." + re.escape(axis) + r"\s*=\s*\{", entry)
    if not m:
        return None
    block, _ = match_block(entry, m.end() - 1)

    steps = []
    sm = re.search(r"\.seq\s*=\s*\{", block)
    if sm:
        seq_block, _ = match_block(block, sm.end() - 1)
        i = 0
        while True:
            j = seq_block.find("{", i)
            if j < 0:
                break
            step, end = match_block(seq_block, j)
            parts = [p.strip() for p in step.split(",") if p.strip()]
            if parts:
                spd_raw = parts[1] if len(parts) > 1 else None
                hold = num(parts[2]) if len(parts) > 2 else 0.0
                steps.append({
                    "angle": num(parts[0]),
                    "speed_raw": spd_raw,
                    "hold": hold if hold is not None else 0.0,
                })
            i = end + 1

    seq_len = num(find_field(block, "seq_len"))
    return {
        "target": num(find_field(block, "target")),
        "speed_raw": find_field(block, "speed"),
        "seq_len": int(seq_len) if seq_len is not None else None,
        "seq_speed_raw": find_field(block, "seq_speed"),
        "steps": steps,
    }


def describe(ax, cfg, notes, tag):
    """把一个轴翻译成白话多行文本。"""
    axis_key, _, center, pose_name, neg_word, pos_word = cfg
    if ax is None:
        notes.append("%s：条目里找不到该轴，按“不参与”处理" % tag)
        return "本动作不参与（保持初始姿势不动）"

    steps, n = ax["steps"], ax["seq_len"]
    tgt = ax["target"]
    tgt_speed = num(ax["speed_raw"]) if ax["speed_raw"] else None

    # 不参与：seq_len=0、无步、target=0、速度=0（四条同时成立才判定）
    if (not steps and (n in (0, None)) and (tgt in (0.0, None))
            and (tgt_speed in (0.0, None))):
        return "本动作不参与（保持初始姿势不动）"

    def word(raw):
        """速度词：本步显式速度优先，留空(0/缺省)则用本轴默认速度。"""
        v = num(raw) if raw is not None else None
        if v in (None, 0.0):
            v = num(ax["seq_speed_raw"])
        return SPEED_WORD.get(int(v) if v is not None else 0, "默认")

    lines = ["【初始姿势】%s（%g°）" % (pose_name, center)]

    # ① 走向终点
    if tgt is None:
        tgt = center
    d = tgt - center
    w = word(ax["speed_raw"])
    if abs(d) < 1e-6:
        lines.append("【起始】%g → %g，%s，没有位移" % (center, tgt, w))
    else:
        lines.append("【起始】%g → %g，%s，%s %g°" %
                     (center, tgt, w, pos_word if d > 0 else neg_word, abs(d)))

    # ② 动作序列逐步
    prev = tgt
    for k, s in enumerate(steps, 1):
        a = s["angle"]
        if a is None:
            notes.append("%s 第%d步角度解析失败" % (tag, k))
            continue
        dd = a - prev
        if abs(dd) < 1e-6:
            seg = "原地不动"
        else:
            seg = "%s %g°" % (pos_word if dd > 0 else neg_word, abs(dd))
        extra = "，停 %.1f 秒" % (s["hold"] / 1000.0) if s["hold"] > 0 else ""
        lines.append("第%d步：%s，%s%s" % (k, seg, word(s["speed_raw"]), extra))
        prev = a

    # ③ 归位
    if abs(prev - center) < 1e-6:
        lines.append("【结束归位】已在%s，无需动作" % pose_name)
    else:
        lines.append("【结束归位】从 %g° 收回 %g°，回到%s" %
                     (prev, abs(prev - center), pose_name))
    return "\n".join(lines)


def main():
    text = strip_comments(io.open(SRC, encoding="utf-8").read())

    start = text.index("g_emotion_matrix[] = {")
    body, _ = match_block(text, text.index("{", start))

    entries, i = [], 0
    while True:
        j = body.find("{", i)
        if j < 0:
            break
        e, end = match_block(body, j)
        entries.append(e)
        i = end + 1

    rows, problems, notes = [], [], []
    seen_ids = {}
    for e in entries:
        eid_m = re.search(r"\.emotion_id\s*=\s*(\w+)", e)
        gif_m = re.search(r'\.gif_path\s*=\s*"([^"]*)"', e)
        if not eid_m or not gif_m:
            problems.append("条目缺 emotion_id 或 gif_path：%r" % e[:60])
            continue
        eid, gif = eid_m.group(1), gif_m.group(1)
        if eid in seen_ids:
            problems.append("emotion_id 重复：%s（%s 与 %s）" % (eid, gif, seen_ids[eid]))
        seen_ids[eid] = gif

        gm = re.search(r"/gif/(\d+)_(\d+)\.gif", gif)
        if not gm:
            problems.append("gif_path 不符合 N_M.gif：%s（%s）" % (gif, eid))
            continue
        grp, idx = gm.group(1), gm.group(2)
        no = "%s_%s" % (grp, idx)
        cname = "%s%s" % (GROUP_NAME.get(grp, "未知组"), idx)

        cells = []
        for axis_key, _, _, _, _, _ in AXES:
            cfg = [c for c in AXES if c[0] == axis_key][0]
            parsed = parse_axis(e, axis_key)
            cells.append(describe(parsed, cfg, notes, "%s·%s" % (no, axis_key)))
            if parsed and parsed["seq_len"] is not None:
                if parsed["seq_len"] != len(parsed["steps"]):
                    problems.append("%s %s：seq_len=%d 但实际解析出 %d 步" %
                                    (no, axis_key, parsed["seq_len"], len(parsed["steps"])))
        cells[0] += "\n" + SPEED_LEGEND
        rows.append([no, cname] + cells)

    rows.sort(key=lambda r: (int(r[0].split("_")[0]), int(r[0].split("_")[1])))

    with io.open(OUT, "w", encoding="utf-8-sig", newline="") as f:
        w = csv.writer(f)
        w.writerow(["编号", "情绪名"] + [c[1] for c in AXES])
        w.writerows(rows)

    print("写出 %d 条 -> %s" % (len(rows), OUT))
    print("速度宏：%s" % MACROS)
    if problems:
        print("\n⚠️ 需人工确认 %d 处：" % len(problems))
        for p in problems:
            print("  - " + p)
    else:
        print("自检通过：无异常")
    if notes:
        print("\nℹ️ 提示 %d 条：" % len(notes))
        for p in notes:
            print("  - " + p)


if __name__ == "__main__":
    main()
