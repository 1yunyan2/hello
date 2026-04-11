---
name: accessToken 刷新机制
description: accessToken 2小时过期，session.c 中实现主动+被动双重刷新策略
type: project
---

accessToken 有效期 **2 小时**，由后端 `/api/auth/device-login` 签发。

**Why:** token 过期会导致 WebSocket 被服务端断开，影响语音交互。

**How to apply:**
- `session.c` 中 `TOKEN_REFRESH_MS = 110分钟`，定时器 `s_token_refresh_timer` 主动刷新
- 断线时 `PROTOCOL_EVENT_DISCONNECTED` 被动刷新
- 两者共用 `session_reconnect_task`，通过 `s_reconnect_handle` 防重复
- 会话进行中延迟刷新，不打断对话
