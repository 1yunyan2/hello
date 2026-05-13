#pragma once

/**
 * @brief 应用层初始化
 * 按顺序启动各子系统：BSP硬件 → WiFi → MQTT → 唤醒词 → 会话模块
 */
void application_init(void);
