# 修改记录

> 带日期前缀的代码改动日志。详细背景见 memory/blufi_provisioning_migration.md。

## 6/22 修改 BluFi 配网（两处）

### 1. 修复链接错误：CMakeLists 漏加 blufi 源文件
- 文件：`main/CMakeLists.txt`（第 60-61 行）
- 问题：`SRCS` 未包含 `blufi/blufi_init.c`、`blufi/blufi_security.c`，导致 `idf.py build`
  链接报 undefined reference（`blufi_security_init`、`esp_blufi_host_and_cb_init`、
  `blufi_dh_negotiate_data_handler`、`blufi_aes_encrypt` 等）。
- 修改：在 SRCS 列表补入这两个文件。

### 2. 去掉配网成功收尾时的 `rc=30` 广播报错
- 文件：`main/bsp/bsp_wifi.c`，`blufi_event_callback()` 的 `ESP_BLUFI_EVENT_BLE_DISCONNECT` 分支（约第 328 行）
- 问题：原代码在蓝牙断开事件里**无条件**调用 `esp_blufi_adv_start()` 重新广播。
  配网成功后手机断开，此刻主流程正在释放蓝牙（NimBLE deinit），重新广播撞上
  正在关闭的协议栈，打印 `error setting advertisement data; rc=30`（BLE_HS_EINVAL）。
  报错无害（蓝牙照常释放、内存正常回收），但日志刺眼。
- 修改：用 `WIFI_BIT` 是否已置位（=已拿到 IP=配网成功）来区分两种断开场景：
  - 配网已成功 → 不再广播（蓝牙即将被主流程释放），消除 rc=30；
  - 配网未完成 → 照常重新广播，等待下一次连接配网。

### 当前进展（6/22 实测，乐鑫 EspBlufi App）
- ✅ BluFi 配网传 WiFi 账密 → 设备联网成功（`Station connect Wi-Fi now, got IP`）
- ✅ Custom Data 通道下发 token → 设备解析存 NVS（`{"token":"test123"}` → `Token 已永久写入 NVS！`）
- ⚠️ 仍卡：缺真实 deviceToken，device-login 换 accessToken 返回 401（**需后端提供测试 token + 接口入参规范**）
