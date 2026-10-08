# -*- coding: utf-8 -*-
"""给回退后的 1_4 / 1_5 补上"来源:回退版"标记注释 + 1_4/1_5 之间补一个空行。

注释里【不写】`.seq_len =` 这类带点号的字段名 —— scan_head_wear.py / replay_emotion.py
的 --all（剥一层 //）会把行内注释也脱掉，那种写法会被正则当成真代码。
"""
import io
import re
import sys

SRC = "main/ui/interaction.c"

NOTE4 = """    /* ★2026-09-28 用户指定【回退】：本条保留旧版动作，本轮不做寿命改造。
     *   来源 = git HEAD(6e21302) 里被整块注释掉的原文，逐字未改，只去了注释符。
     *   ⚠️ 扫描脚本会报它"小步快动"（头部 10°/20°@MID 来回、左臂末步写 0.0f
     *      会被软限位钳到 10°、右臂整条关着），这是【回退版自带的状态】不是漏改，
     *      未经用户指示不要动。*/
"""
NOTE5 = """    /* ★2026-09-28 用户指定【回退】：本条保留旧版动作，本轮不做寿命改造。
     *   来源 = git HEAD(6e21302) 里被整块注释掉的原文，逐字未改，只去了注释符。
     *   ⚠️ 回退版自带状态：最长轴 3860ms / GIF 周期 7330ms = 0.53 倍（播到一半
     *      就被硬切）；头部末尾 5 步是 5°/10° 的微动。不是漏改。*/
"""


def main():
    raw = io.open(SRC, encoding="utf-8", newline="").read()
    eol = "\r\n" if raw.count("\r\n") > raw.count("\n") // 2 else "\n"
    t = raw.replace("\r\n", "\n")

    for title, note in (("1_4", NOTE4), ("1_5", NOTE5)):
        pat = r"(    // ── 1: 中性" + title + r"[^\n]*\n)"
        if len(re.findall(pat, t)) != 1:
            raise SystemExit("[X] 标题 %s 命中不为 1，已中止" % title)
        t = re.sub(pat, lambda m: m.group(1) + note, t, count=1)

    # 1_4 的 `},` 与 1_5 的标题之间补空行（旧 HEAD 原文此处没有）
    t = t.replace("    },\n    // ── 1: 中性1_5",
                  "    },\n\n    // ── 1: 中性1_5", 1)

    io.open(SRC, "w", encoding="utf-8", newline="").write(eol.join(t.split("\n")))
    print("[done] 已补标记注释与空行")


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
