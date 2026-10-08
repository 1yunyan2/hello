---
name: auth_401_blocks_websocket
description: WebSocket 连不上的真因=前置 device-login 被服务器 401 拒；含 IDF 401 告警来源与 auth.c 丢弃非2xx响应体的盲点
metadata:
  node_type: memory
  type: project
  originSessionId: 154170cf-c4cc-466f-a6f5-bd85c773d00a
  modified: 2026-09-29T06:59:39.284Z
---

# 「WebSocket 有问题」= device-login 401，WS 从未发起

## 已验证事实（2026-09-28 实测日志 + 代码/IDF 源码核对）

1. `W HTTP_CLIENT: This request requires authentication, but does not provide header information for that`
   **不是我们打印的、也不是服务器返回的文案**，是 ESP-IDF `esp_http_client.c:1827` 在
   「HTTP 401 **且响应无 `WWW-Authenticate` 头**」时的固定分支。⇒ 服务器明确回了 401。
2. `Auth 第 N 次失败 (ret=ESP_OK, status=401)` 里的 **`ret=ESP_OK` 是关键**：
   TCP + TLS + HTTP 整条链路都通、服务器在线并应答，**只有凭证被拒**。
   同日志里 `esp-x509-crt-bundle: Certificate validated` 每次都有、MQTT 心跳照常发
   ⇒ 网络/DNS/域名/服务器搬迁全部排除。
3. auth 失败后 [session.c:1208](main/session/session.c#L1208) 直接 `continue` 进退避重试
   ⇒ **WS 连接压根没发起**，所以日志里一条 WS 握手错误都没有。不是"连不上"，是"没去连"。
4. 间隔实测：3 次尝试/轮（每轮约 6~10s，含 2s 间隔），轮间 20s，永不停 → 一直空转烧内部 SRAM。
   偶发 `esp-tls: [sock=55] select() timeout` → `ESP_ERR_HTTP_CONNECT` 是次要态（网络抖动）。

## 代码盲点（定位被挡住的唯一原因）

[auth.c:63-65](main/protocol/auth.c#L63-L65)：
`if (status_code != 200 && status_code != 201) return ESP_OK;`
⇒ **非 2xx 的响应体被整条丢弃**，所以 `Auth API Response:` 后面永远是空的。
服务器 401 时大概率回了 `{"code":401,"message":"…"}`，全被吃掉。**不是服务器没给理由。**

## 时间线对照（最重要的一条）

- `CHANGES.md:27`（6/22）：仍卡「缺真实 deviceToken，device-login 返回 401，需后端给测试 token」
- `memory/daily/2026-07-24.md:19`（7/24）：迁 https 当天 **「accessToken 获取成功」**
⇒ **同一域名同一接口曾经通、凭证也对上过**。现在又 401，最可能是**凭证侧变了**而非代码坏了。

## 待办

- [x] **2026-09-29 已改（第 1、2、3 条）**：见下方「本次改动」
- [ ] **首选零改码动作：给 A40C90 重新配网**（长按按键重置 → App 重新绑定配网下发新 token）。
      重置会 erase `ws_token`（[bsp_wifi.c:209](main/bsp/bsp_wifi.c#L209)），强制后端重签。
      好 ⇒ 确诊为②以外的"陈旧脏值"；仍 401 ⇒ 转后端查绑定记录。
- [ ] 后端侧最直接：让后端**直接查 A40C90 有没有 deviceToken 记录**，与能连的那台对比
- [ ] 顺带：`max_retries` 3→1 减少空转（未做）

## ⭐ 2026-09-29 二次定位：不是"没 token"，是"token 在服务器侧不认"

**新事实（用户报 + 翻旧日志对齐得出）：**
- 用户报「**另外一台设备初始化正常、能连**」⇒ 后端接口/契约/证书/机制全排除。
- 翻本会话原始 jsonl 按消息分块对齐 deviceId，得到：
  - **A40C90**（一直报 401 的这台）：日志有 `I (7973) Session: 检测到 deviceToken，accessToken 交给后台重连任务异步换取`
    ⇒ **NVS 里 deviceToken 是非空的**。"空 token"假设**被证伪**。
  - 8C6BF8（另一段旧的舵机远控日志）：`W (7265) [WARN] 无 deviceToken`，与 401 无关，别混进来。
- 取证手法（可复用）：按 jsonl 的**每条 user 消息分块**统计关键词，比全库 grep 更能避免
  "把不同设备/不同会话的日志混成一条时间线"。
  ⚠️ 工具结果（tool_result）在 jsonl 里也是 role=user 但无 text 块，会污染统计，要按 `content` 里
  取 text 并滤掉短消息。

**决定性推理：device-login 的请求体只有 `{"deviceToken":"…"}` 一个字段，请求头只有 `Content-Type`**
（[auth.c:153](main/protocol/auth.c#L153)、[auth.c:180-181](main/protocol/auth.c#L180-L181)），
**没有 Device-Id / MAC / Client-Id**。⇒ 服务器能据以判别身份的唯有这个值本身；
同一固件在另一台能通 ⇒ **401 只可能来自这个值**。

⇒ **结论：A40C90 的 NVS `ws_token` 是"有值但在服务器侧已失效/未绑定"的旧凭证。**
机理：deviceToken 由后端在**设备绑定时**生成、配网时由 App 经 BluFi 下发
（[docs/前端配网对接说明.md](docs/前端配网对接说明.md) 第三节）。
设备端只有在走**本机重置流程**时才会擦掉它（[bsp_wifi.c:209](main/bsp/bsp_wifi.c#L209) `nvs_erase_key(h,"ws_token")`）；
若解绑是在**云端/App 侧**做的而没走设备重置，设备就留着一个已被注销的 token ⇒ 表现正是"有值、每次都被拒"。

**剩下的二分（等日志/后端回答）：**
- ① token 是旧测试脏值（如 `test123`）/长度异常/带尾随 `\r\n` ⇒ 设备侧问题，**重新配网**即可；
- ② 长度正常仍被拒 ⇒ 服务器侧未绑定/已注销，**需后端查 A40C90 的绑定记录**。

## 本次改动（2026-09-29，纯诊断，未编译未上板）

[main/protocol/auth.c](main/protocol/auth.c) 两处：

1. `auth_http_event_handler` 的 `HTTP_EVENT_ON_DATA` 分支：
   删掉 `int status_code = ...; if (status_code != 200 && status_code != 201) return ESP_OK;`
   改为 `if (evt->data_len <= 0) return ESP_OK;` —— **任何状态码的响应体都收**，
   于是 401 时 ON_FINISH 的 `Auth API Response:` 终于会打出服务器给的原因。
   安全性：响应缓冲只有 ①ON_FINISH 打印 ②拿 token 时 cJSON 解析 两个用途，而 ② 之前有
   状态码判定早退，故行为不变。排查完把状态码过滤加回来即可复原。
2. `auth_perform` 重试循环开头加 `free(wrapper->response); wrapper->response = NULL;
   wrapper->response_len = 0;` —— 原先整个函数只在入口清一次缓冲，多轮重试的响应体
   会在日志里首尾相连，读不出哪句属于哪一次。
3. **【诊断改动 2】** 逻辑块 1 之后新增"逻辑块 1.5"（[auth.c:147-172](main/protocol/auth.c#L147-L172)）：
   打印 deviceToken **指纹** —— `长度 + 前8字符 + 后4字符`，空则打 W 警告。
   目的正是做上面那个二分（脏值 vs 服务器侧失效）。**只打首尾、不整条打印**（别把完整凭证刷进日志）。
   安全性：纯读、无 NVS 写、无堆操作，跑在 auth_perform 里（栈够用）。

⚠️ 编译前先看 [[ws_session_init_commented_out]]：`application.c:926` 的 `session_init` 是注释状态，
不放开会话层根本不启动。

相关：[[project_token_refresh]]、[[blufi_provisioning_migration]]、[[project_mqtt_security_todo]]
