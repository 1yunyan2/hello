# 项目核心信息
- 项目名称：Echo2（Echopals 智能语音产品）
- 硬件平台：ESP32-S3
- 开发框架：ESP-IDF v5.3.4
- 开发语言：纯 C 语言
- 核心功能：离线语音唤醒、音频采集、AI 语音交互

# 代码规范
- 头文件必须使用 #pragma once，禁止使用 #ifndef 防护
- 函数命名采用下划线分隔（snake_case），禁止驼峰命名
- 所有 .c/.h 文件必须放在 main/ 或 components/ 目录下
- 禁止修改 ESP-IDF 系统源码、组件库文件

# 常用命令
- 编译：idf.py build
- 烧录：idf.py flash
- 监控：idf.py monitor
- 完整流程：idf.py flash monitor

# 权限规则
- 禁止修改 esp-idf/ 目录、系统目录、系统文件、配置文件等（如 sdkconfig 仅在授权后修改）
- 执行 idf.py 命令前无需二次确认，自动执行

# 个人偏好
- 代码必须添加详细注释，关键逻辑标注说明
- 生成代码时优先考虑稳定性、可维护性，而非极致性能
- 每次修改代码后，自动检查编译错误，给出修复建议