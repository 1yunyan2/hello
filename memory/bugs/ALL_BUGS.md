# Echo2 踩坑日志（完整版）

> 从 Git 历史 + 代码深度扫描 交叉提取，共 33 个坑点
> 最后更新：2026-04-03

---

## 一、严重问题（7 个）— 必须优先处理

### BUG-001 | MQTT 硬编码凭证（安全风险）
- **位置**: `main/protocol/mqtt_protocol.c:4-6`
- **问题**: 硬编码了 MQTT broker URI、用户名和密码（`122.224.191.2:1883`, `xtc`, `Xtc@12345`）
- **风险**: 代码泄露即凭证泄露，生产环境致命
- **建议**: 移至 NVS 或通过 BLE 配网下发，代码中只放默认公共 broker
- **状态**: ❌ 未处理 ：我认为无事

### BUG-002 | malloc 返回值用 assert 检查
- **位置**: `main/audio/audio_encoder.c:39-44` / `main/bsp/bsp_codec.c:118-121`
- **问题**: `assert(ptr)` 在 Release 模式下会被编译器移除，SPIRAM 分配失败直接崩溃无日志
- **根因**: `object.h:5-14` 中的 `malloc_zeroed()` 所有调用点都缺乏正确检查
- **修复**: audio_encoder.c 改为 NULL 检查 + ESP_LOGE + 安全退出；bsp_codec.c 改为 NULL 检查 + return
- **状态**: ✅ 已修复（2026-04-03 场景4优化）

### BUG-003 | 唤醒词互斥锁 portMAX_DELAY 死锁风险
- **位置**: `main/wake_word/custom_wake_word.c:376,444,451`
- **问题**: `xSemaphoreTake(buffer_mutex, portMAX_DELAY)` 无限等待，若 AFE fetch 任务持锁卡住则整个系统死锁
- **建议**: 改为 `pdMS_TO_TICKS(500)` 并处理超时
- **状态**: ❌ 未处理

### BUG-004 | NVS 句柄泄漏
- **位置**: `main/wake_word/custom_wake_word.c:57-74`
- **问题**: `nvs_read_str()` 和 `nvs_write_str()` 中多个 return 路径未调用 `nvs_close(h)`
- **建议**: 使用 goto 清理模式确保所有路径都关闭句柄
- **状态**: ❌ 未处理

### BUG-005 | BLE 配网 strdup 内存泄漏
- **位置**: `main/bsp/bsp_wifi.c:103`
- **问题**: `custom_prov_data_handler` 中 `strdup()` 分配内存，注释说"协议栈负责释放"但无保证
- **建议**: 改用静态缓冲区或验证协议栈释放机制
- **状态**: ❌ 未处理

### BUG-006 | 会话定时器未检查创建结果
- **位置**: `main/session/session.c:226-232`
- **问题**: `xTimerCreate()` 内存不足返回 NULL，后续 `xTimerStart/Stop` 直接崩溃
- **建议**: 添加 NULL 检查和错误处理
- **状态**: ❌ 未处理

### BUG-007 | WebSocket mutex 未在 stop 时销毁
- **位置**: `main/protocol/websocket_client.c:65,116-127`
- **问题**: `s_mutex` 在 `ws_client_start()` 创建，`ws_client_stop()` 未删除，反复启停会泄漏
- **建议**: stop 时添加 `vSemaphoreDelete(s_mutex); s_mutex = NULL;`
- **状态**: ❌ 未处理

---

## 二、已修复的历史坑点（12 个）— 防止重蹈覆辙

### BUG-H01 | SPIFFS 分区名反复修改 ⭐
- **位置**: `partitions.csv`
- **历程**: `storage`(edbed9a) → `srmodel`(edbed9a) → `model`(ee3c02c)，改了 3 次
- **教训**: 分区名修改必须同步检查 partitions.csv + 代码中所有 `esp_spiffs_init()` 调用
- **相关提交**: edbed9a, ee3c02c
- **状态**: ✅ 已修复（最终使用 `model`）

### BUG-H02 | 蓝牙内存释放导致系统崩溃 ⭐
- **位置**: 原 `main/WIFI/WIFI.c`（现已重构为 bsp_wifi）
- **问题**: `esp_bt_controller_mem_release(ESP_BT_MODE_BTDM)` 导致系统不稳定
- **相关提交**: 1de38fb
- **状态**: ⚠️ 已规避（注释掉），未根治

### BUG-H03 | SPIFFS 初始化内存冲突 ⭐
- **位置**: `main/application.c:39`（当时的行号）
- **问题**: `init_spiffs()` 与其他模块内存冲突
- **相关提交**: 1de38fb
- **状态**: ⚠️ 已规避（注释掉），未根治

### BUG-H04 | MQTT 配置结构体栈溢出
- **位置**: `main/protocol/mqtt_protocol.c`
- **问题**: `esp_mqtt_client_config_t` 放栈上（>1KB），超出 FreeRTOS 任务栈
- **修复**: 改为 `static` 静态分配
- **相关提交**: ba1b8c3, c547cd5
- **状态**: ✅ 已修复

### BUG-H05 | I2S 音频引脚配置错误
- **位置**: `main/bsp/bsp_config.h`（引脚定义）
- **历程**: MCLK=17,BCLK=9 → MCLK=3,BCLK=2 → 又改回 MCLK=17,BCLK=9
- **⚠️ 注意**: 当前 bsp_config.h 的值与 c547cd5 提交时不同，需验证硬件文档
- **相关提交**: c547cd5, dd0b7bd
- **状态**: ⚠️ 需验证当前值是否正确

### BUG-H06 | 唤醒词引擎多线程并发
- **位置**: `main/wake_word/custom_wake_word.c`
- **问题**: 音频缓冲区和状态标志无锁保护
- **修复**: 添加 `buffer_mutex` 互斥锁
- **相关提交**: dd0b7bd, e7ad379
- **状态**: ✅ 已修复（但锁策略仍有 BUG-003 的死锁风险）

### BUG-H07 | 音频任务栈溢出
- **位置**: `main/audio/audio.c`（已重构）
- **修复**: 栈大小 4096 → 8192，绑定 CPU1
- **相关提交**: c547cd5
- **状态**: ✅ 已修复

### BUG-H08 | 唤醒词代码被全部注释掉
- **位置**: `main/wake_word/custom_wake_word.c` + `.h`（242+72 行全注释）
- **原因**: 可能临时规避某问题
- **相关提交**: 4a4f816（注释）→ ba1b8c3（恢复）
- **状态**: ✅ 已恢复

### BUG-H09 | MQTT 主题硬编码 MAC 地址
- **位置**: `main/protocol/mqtt_protocol.c`
- **修复**: 改为动态获取 `esp_wifi_get_mac()` + `snprintf`
- **相关提交**: ba1b8c3
- **状态**: ✅ 已修复

### BUG-H10 | NVS 初始化代码分散重复
- **位置**: 原 `main/application.c` 多处重复
- **修复**: 封装为 `bsp_board_nvs_init()`
- **相关提交**: dd0b7bd
- **状态**: ✅ 已修复

### BUG-H11 | 串口号 COM4/COM5 反复切换
- **位置**: VSCode 配置文件
- **教训**: 更换 USB 口后 Windows 可能分配不同 COM 端口，烧录前先确认
- **相关提交**: ee3c02c, edbed9a
- **状态**: ✅ 已知问题

### BUG-H12 | 唤醒词阈值硬编码
- **位置**: `main/wake_word/custom_wake_word.c`
- **修复**: 添加 `multinet_det_treshold` 可配置变量，默认 0.85→0.6
- **相关提交**: e7ad379
- **状态**: ✅ 已修复

---

## 三、中等问题（14 个）— 建议近期处理

### BUG-008 | WebSocket 地址硬编码
- **位置**: `main/session/session.c:37`
- **问题**: 硬编码 `ws://192.168.1.100:8080/audio`，无法切换服务器
- **建议**: 从 NVS 动态加载

### BUG-009 | 设备 ID 仅用 MAC 后 3 字节
- **位置**: `main/protocol/mqtt_protocol.c:109`
- **问题**: 3 字节仅 16M 种组合，大规模部署可能重复
- **建议**: 使用完整 6 字节 MAC

### BUG-010 | 音频编码器内存泄漏
- **位置**: `main/audio/audio_encoder.c:102-103`
- **问题**: 编码器创建失败时 `in_frame.buffer` 和 `out_frame.buffer` 未释放

### BUG-011 | 解码器缓冲区大小硬编码
- **位置**: `main/audio/audio_decoder.c:44-45`
- **问题**: PCM 输出缓冲固定按 60ms 帧计算，帧时长变化会出错

### BUG-012 | Ring Buffer 大小硬编码
- **位置**: `main/audio/audio_processor.c:62-65`
- **问题**: 20480/2560/5120/40960 字节写死，音频参数变化会溢出
- **修复**: 提取为 `ENC_INPUT_BUF_SIZE` 等命名常量，便于统一调整
- **状态**: ✅ 已修复（2026-04-03 场景4优化）

### BUG-013 | esp_codec_dev_write 返回值未检查
- **位置**: `main/audio/audio_processor.c:44`
- **问题**: DMA 写入失败无法感知

### BUG-014 | AFE fetch 任务无优雅退出机制
- **位置**: `main/wake_word/custom_wake_word.c:183`
- **问题**: `while(1)` 无法被外部控制停止

### BUG-015 | bsp_codec 大缓冲区栈分配
- **位置**: `main/bsp/bsp_codec.c:154`
- **问题**: `audio_feed_task` 中 chunk_size 大小的缓冲区在栈上分配

### BUG-016 | 心跳任务无中止机制
- **位置**: `main/protocol/mqtt_protocol.c:118-142`
- **问题**: `while(1)` 无法被主动停止

### BUG-017 | 唤醒词更新任务栈 4KB 偏小
- **位置**: `main/protocol/mqtt_protocol.c:206-207`
- **问题**: MultiNet6 推理可能需要更多栈空间，建议 8192

### BUG-018 | WS 发送任务栈 4KB 偏小
- **位置**: `main/session/session.c:281-283`
- **问题**: 音频编码 + WebSocket 发送可能栈溢出，建议 8192

### BUG-019 | audio_processor 播放任务未检查 board 指针
- **位置**: `main/audio/audio_processor.c:34,44`
- **问题**: `bsp_board_get_instance()` 返回 NULL 则直接崩溃

### BUG-020 | SPIRAM 分配无回退
- **位置**: `main/audio/audio_processor.c:62-65`
- **问题**: Ring buffer 约 68KB 在 SPIRAM 分配，SPIRAM 初始化失败则全部失败

### BUG-021 | NVS 命名空间分散
- **位置**: `main/wake_word/custom_wake_word.c:8` + `main/session/session.c:38`
- **问题**: "sys_config"、"mqtt_creds"、"net_config" 散落各处，易冲突
- **建议**: 集中到 `bsp_config.h` 定义

---

## 四、低优先级（6 个）— 可择机处理

### BUG-022 | TODO: BLE 配网数据未解析
- **位置**: `main/bsp/bsp_wifi.c:94`

### BUG-023 | TODO: ADC 电池电量映射公式
- **位置**: `main/protocol/mqtt_protocol.c:93`

### BUG-024 | 注释掉的 bsp_wake_word_start()
- **位置**: `main/application.c:61`

### BUG-025 | 注释掉的 power_monitor_init()
- **位置**: `main/application.c:97`

### BUG-026 | LED/LCD 配置全部被注释
- **位置**: `main/bsp/bsp_config.h:3,15,17-22`

### BUG-028 | session.c s_current_wake_word 多任务竞态
- **位置**: `main/session/session.c:57,271`
- **问题**: `s_current_wake_word` 在 `session_on_wake_word`（主任务）写入，`protocol_event_handler`（事件任务）读取，无同步
- **修复**: 添加 `s_wake_word_mutex` 互斥锁保护读写
- **状态**: ✅ 已修复（2026-04-03 场景4优化）

### BUG-027 | WiFi 失败后硬等 30 秒
- **位置**: `main/bsp/bsp_wifi.c:326`
- **问题**: `vTaskDelay(30000)` 阻塞，用户无法在等待期间操作

---

## 统计

| 类别 | 数量 | 说明 |
|------|------|------|
| 严重（未处理） | 7 | 安全、崩溃、内存泄漏 |
| 历史已修复 | 12 | 防止回退 |
| 中等（待处理） | 14 | 稳定性和健壮性 |
| 低优先级 | 6 | TODO 和代码清理 |
| **合计** | **33** | |
