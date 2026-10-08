# -*- coding: utf-8 -*-
"""
把草稿按【唯一标记行】替换进 main/ui/interaction.c。

为什么不直接用 str.replace / str.index：
  本项目历史上踩过坑 —— 文件里存在【被注释掉的同名标题】，str.index() 命中过
  前部那块，结果写坏文件。所以对每个锚点都断言"全文恰好出现 1 次"，
  不满足就直接中止、不落盘。

★行尾：读进来是什么行尾，就按什么行尾写回去（此前一版强制 \n，把整个文件从
  CRLF 拉平成了 LF，制造了两千多行假 diff）。

用法: python _tools/_splice.py [--dry]
"""
import io
import os
import shutil
import sys

SRC = "main/ui/interaction.c"
BAK = "main/ui/interaction.c.bak_splice"

# (草稿文件, 起始锚点行片段, 结束锚点行片段, 结束锚点前保留几行, 起始锚点前吃掉几行, 说明)
#   tail_keep: 结束锚点的【作用域开头】往往在锚点前几行（例如 2_1 那条的 `// ══`
#   分隔线），必须原样留下，否则会把下一个条目的开头吃掉。
#   head_drop: 起始锚点【前】要一起替换掉的行数。被整块 // 注释掉的条目，它的
#   `// {` 在标记行【之前】（旧写法把标题写在花括号里面），而新草稿是"标题在外、
#   { 在内"，所以要把那一行旧 `// {` 一并吃掉，head_drop = 1。
JOBS = [
    ("_tmp_1x69.txt", "── 1_6: 中性①", "── 10- 2_1傲娇①", 1, 1, "1_6~1_9"),
]


def find_unique(lines, frag):
    hits = [i for i, ln in enumerate(lines) if frag in ln]
    if len(hits) != 1:
        raise SystemExit("[X] 锚点 %r 命中 %d 次（应为 1 次），已中止，未改动任何文件"
                         % (frag, len(hits)))
    return hits[0]


def main():
    dry = "--dry" in sys.argv
    raw = io.open(SRC, encoding="utf-8", newline="").read()
    eol = "\r\n" if raw.count("\r\n") > raw.count("\n") // 2 else "\n"
    lines = raw.replace("\r\n", "\n").split("\n")
    print("[行尾] 原文以 %s 为主" % ("CRLF" if eol == "\r\n" else "LF"))

    plan = []
    for draft, a_frag, b_frag, tail_keep, head_drop, desc in JOBS:
        i = find_unique(lines, a_frag) - head_drop
        j = find_unique(lines, b_frag)
        if not i < j:
            raise SystemExit("[X] %s 的锚点顺序反了" % desc)
        end = j - tail_keep
        while end > i and lines[end - 1].strip() == "":
            end -= 1
        plan.append((i, end, draft, desc))

    for i, end, draft, desc in plan:
        print("[plan] %-10s  interaction.c 第 %d~%d 行（1-based） <- %s"
              % (desc, i + 1, end, draft))
    if dry:
        print("[dry] 未改动")
        return

    for i, end, draft, desc in reversed(plan):
        new = io.open(draft, encoding="utf-8", newline="").read().replace("\r\n", "\n")
        new = new.rstrip("\n")
        lines[i:end] = new.split("\n")
        print("[ok]   %-10s 已换入 %d 行（吃掉 %d 行）"
              % (desc, len(new.split("\n")), end - i))

    shutil.copyfile(SRC, BAK)
    io.open(SRC, "w", encoding="utf-8", newline="").write(eol.join(lines))
    print("[done] 已写回 %s（备份 %s）" % (SRC, BAK))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
