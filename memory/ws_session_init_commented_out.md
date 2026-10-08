---
name: ws_session_init_commented_out
description: application.c 里 session_init 是注释状态⇒当前源码重编后会话/WebSocket 层根本不启动；板上跑的是还启用着的旧构建
metadata:
  node_type: memory
  type: project
  originSessionId: 154170cf-c4cc-466f-a6f5-bd85c773d00a
  modified: 2026-09-29T05:48:01.717Z
---

# ⚠️ 当前源码里 `session_init()` 是注释状态 —— 重编就没有 WebSocket 了

## 事实（2026-09-29 查证）

- [application.c:924-927](main/application.c#L924-L927)：`session_init("wss://ai.strailine-space.com/ws/omni")`
  和 `PRINT_INTERNAL_HEAP_STEP("session_init")` **全是注释**。
- 全仓（main/ + components/，含非 .c/.h）grep：**没有任何一处活跃的 `session_init` 调用**，
  所有出现都在注释或文档里。
- 但**上次构建**的 `build/esp-idf/main/CMakeFiles/__idf_main.dir/application.c.obj`（2026-09-28 16:38）
  里同时含有 `session_init` 符号引用 **和字符串字面量 `wss://ai.strailine-space.com/ws/omni`**。
  **注释编不进目标文件** ⇒ 编译那一刻这行是活的。
- 板上日志印证：`[heap] session_init internal free: …` + `Session: 最终请求的完整 WebSocket URI` 都打出来了，
  且顺序正好卡在 `触摸扫描任务完成` 之前，与源码 926→936 的顺序一致。

⇒ **板上跑的是"活的"那份，工作区源码已被改成"死的"。** 用现在这份源码 `idf.py build` 烧进去，
会话/WS 模块整体不启动，连现在这种 401 重试循环都不会有。

## 排查/取证手法（可复用）

`grep -a` 直接查**目标文件**，比看源码更能确定"编进去的到底是什么"：
```bash
grep -ac "wss://ai.strailine-space.com/ws/omni" build/esp-idf/main/CMakeFiles/__idf_main.dir/application.c.obj
```
⚠️ 本机 Git Bash 的 `strings build/Echo.bin` **输出 0 行（工具在本环境失效）**，别用它下结论
——我曾据此误判"字符串不存在"，换 `grep -a` 才发现全在。
⚠️ 目标文件 mtime 比 elf 新（如 session.c.obj 9/29 13:18 > Echo.elf 9/28 16:49）= 有一次构建没走到链接。

## WS URI 的运行期来源

`wss://ai.strailine-space.com/ws/omni` 在源码里**只存在于注释**；运行期日志却打出它
⇒ 只可能来自 **NVS 键 `ws_uri`**（[session.c:1430-1431](main/session/session.c#L1430-L1431) 读，
**全仓无写入方**，是旧固件/配网时代写进去的）。所以那个 URI 是"从 NVS 里翻出来的老值"。

相关：[[auth_401_blocks_websocket]]
