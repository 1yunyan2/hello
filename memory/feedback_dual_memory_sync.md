---
name: 系统memory与项目memory双写规范
description: 每次写入系统memory时，必须同步写入项目memory目录，CHANGELOG.html尤其要及时更新
type: feedback
originSessionId: d00b77de-da96-4f9b-a731-7b2efcff7c2f
---
# 双写规范

**规则**：每次向系统 memory（`C:\Users\esthe\.claude\projects\...\memory\`）写入任何文件时，必须同步写入项目 memory（`d:\new\Echopals\Echo2\memory\`）。两处内容保持完全一致。

**Why**：系统 memory 是 Claude 内部读取的来源，项目 memory 是用户在 IDE/文件系统中可见的版本。只写系统 memory 会导致用户打开项目目录时看不到最新内容，产生信息断层。

**How to apply**：
1. 每次用 `Write` 工具写系统 memory 文件后，立刻用同样内容写到项目 memory 对应路径
2. 目录结构两处完全镜像：`bugs/`、`daily/`、`decisions/`、`summaries/`、`weekly/` 均需同步
3. **CHANGELOG.html 尤其要及时更新**：每次有新提交或新坑点，必须在 CHANGELOG.html 中追加对应条目，不能只记录在 .md 文件里
4. `MEMORY.md` 索引两处同步更新
5. 批量操作时用 `cp` 命令镜像，单文件操作时写两次 `Write`
