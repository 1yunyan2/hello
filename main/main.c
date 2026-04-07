/**
 * @file main.c
 * @brief 程序入口
 * ESP-IDF 框架要求的 app_main() 入口函数，仅负责调用应用层初始化
 */

#include "application.h"

/**
 * @brief ESP-IDF 程序入口
 * 系统启动后由 FreeRTOS 主任务调用，所有业务逻辑在 application_init() 中展开
 */
void app_main(void)
{
    application_init();
}