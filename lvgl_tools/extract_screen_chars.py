# -*- coding: utf-8 -*-
"""扫描 main/ui 下所有可能渲染到屏幕的中文字符。
策略（安全优先，宁多勿漏）：
  - 取每个 .c 里所有【字符串字面量】中的中文
  - 仅排除注释行 (// 或 * 或 /*) 与 ESP_LOGx / printf 日志行
  - snprintf / 结构体表项 / set_text 等一律保留
结果去重后写入 _screen_chars.txt（UTF-8），供裁剪 font_cn_16/32 使用。
"""
import re, glob, os

UI_DIR = os.path.join(os.path.dirname(__file__), "..", "main", "ui")

LOG = re.compile(r'ESP_LOG[IWEDV]\b|\bprintf\b')
STR = re.compile(r'"((?:[^"\\]|\\.)*)"')
CJK = re.compile('[一-鿿]')

chars = set()
detail = {}
skipped_log_chars = set()

for f in sorted(glob.glob(os.path.join(UI_DIR, "*.c"))):
    name = os.path.basename(f)
    if name.startswith("font_cn"):
        continue
    for ln in open(f, encoding="utf-8", errors="ignore"):
        s = ln.lstrip()
        if s.startswith("//") or s.startswith("*") or s.startswith("/*"):
            continue
        is_log = LOG.search(ln)
        for m in STR.findall(ln):
            for c in m:
                if not CJK.match(c):
                    continue
                if is_log:
                    skipped_log_chars.add(c)
                else:
                    chars.add(c)
                    detail.setdefault(name, set()).add(c)

out = "".join(sorted(chars))
report = []
report.append("=== 屏幕中文字符（去重） ===")
report.append("数量: %d" % len(chars))
report.append(out)
report.append("")
report.append("=== 按文件 ===")
for f in sorted(detail):
    report.append("%-16s %3d  %s" % (f, len(detail[f]), "".join(sorted(detail[f]))))
report.append("")
# 仅出现在日志、未被屏幕用到的字（这些不进字库）
only_log = sorted(skipped_log_chars - chars)
report.append("=== 仅日志使用、已排除（不进字库）共 %d 字 ===" % len(only_log))
report.append("".join(only_log))

txt = "\n".join(report)
with open(os.path.join(os.path.dirname(__file__), "_screen_chars.txt"), "w", encoding="utf-8") as fp:
    fp.write(out)
with open(os.path.join(os.path.dirname(__file__), "_screen_report.txt"), "w", encoding="utf-8") as fp:
    fp.write(txt)
print("done. screen chars =", len(chars), " | log-only excluded =", len(only_log))
