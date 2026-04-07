#pragma once

/**
 * @file application.h
 * @brief 应用层初始化入口头文件
 * 负责整合 BSP、协议、唤醒词等模块的统一启动
 */

#include "bsp/bsp_board.h"
#include "protocol/mqtt_protocol.h"
#include "wake_word/custom_wake_word.h"

/**
 * @brief 应用层初始化
 * 按顺序启动各子系统：BSP硬件 → WiFi → MQTT → 唤醒词 → 会话模块
 */
void application_init(void);
