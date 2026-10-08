# -*- coding: utf-8 -*-
"""
把 1_4 / 1_5 回退成"改动前"的版本（git HEAD 里那两个被整块注释掉的条目），
其余条目（1_1/1_2/1_3/1_6~1_9）一律不动。

为什么从 HEAD 取而不是从 .bak_splice：
  备份是在 1_6~1_9 拼接【之前】拍的，但那时 1_2~1_5 已经改成新版了，
  备份里的 1_4/1_5 也是新版 ⇒ 拿不到"原来的"。HEAD 才是改动前的权威。

剥注释规则：`^(\s*)// ?` -> `\1`（只剥最前面那一层 `//`）。
  这样 `    // // ── 1: 中性1_4` -> `    // ── 1: 中性1_4`（保留分隔线），
       `    // {`               -> `    {`
  回退后的条目【保持启用】（与本轮其余八条一致），不是恢复成注释态。
"""
import io
import re
import shutil
import subprocess
import sys

SRC = "main/ui/interaction.c"
BAK = "main/ui/interaction.c.bak_revert145"
ANCHOR_A = "// ── 1: 中性1_4"
ANCHOR_B = "// ── 1: 中性1_6"


def find_unique(lines, frag):
    hits = [i for i, ln in enumerate(lines) if frag in ln]
    if len(hits) != 1:
        raise SystemExit("[X] 锚点 %r 命中 %d 次（应为 1 次），已中止"
                         % (frag, len(hits)))
    return hits[0]


def uncomment(line):
    return re.sub(r"^(\s*)// ?", r"\1", line)


def main():
    dry = "--dry" in sys.argv

    # 1) 从 HEAD 抽出被注释掉的 1_4 + 1_5 整段
    head = subprocess.run(["git", "show", "HEAD:" + SRC],
                          capture_output=True).stdout.decode("utf-8")
    hl = head.replace("\r\n", "\n").split("\n")
    i4 = find_unique(hl, "// // ── 1: 中性1_4")
    closes = [k for k in range(i4, len(hl)) if hl[k].strip() == "// },"]
    if len(closes) < 2:
        raise SystemExit("[X] HEAD 里找不到 1_4/1_5 的结束花括号")
    end = closes[1] + 1                      # closes[0]=1_4 尾, closes[1]=1_5 尾
    old_block = hl[i4:end]
    new_block = [uncomment(l) for l in old_block]
    print("[HEAD] 摘出旧版 1_4+1_5：HEAD 第 %d~%d 行，共 %d 行"
          % (i4 + 1, end, len(old_block)))
    for l in new_block:
        if ".emotion_id" in l or "seq_len" in l:
            print("        " + l.strip())

    # 2) 工作区定位
    raw = io.open(SRC, encoding="utf-8", newline="").read()
    eol = "\r\n" if raw.count("\r\n") > raw.count("\n") // 2 else "\n"
    lines = raw.replace("\r\n", "\n").split("\n")
    a = find_unique(lines, ANCHOR_A)
    b = find_unique(lines, ANCHOR_B)
    if not a < b:
        raise SystemExit("[X] 锚点顺序反了")
    print("[plan] 用 %d 行旧版替换 SRC 第 %d~%d 行（1-based）"
          % (len(new_block), a + 1, b))       # b 是 1_6 标题，保留不动

    if dry:
        print("[dry] 未改动")
        return

    lines[a:b] = new_block + [""]
    shutil.copyfile(SRC, BAK)
    io.open(SRC, "w", encoding="utf-8", newline="").write(eol.join(lines))
    print("[done] 已写回 %s（备份 %s）" % (SRC, BAK))


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    main()
