/**
 * @file example_main.c
 * @brief servo_driver 使用示例（可直接作为工程的 app_main 跑）
 *
 * 接线：把舵机信号线接到下面 gpio_num 指定的引脚，舵机电源用独立 5V，
 *       并把舵机电源地与 ESP32 的 GND 共地。
 *
 * 编译方式见 README.md。
 */

#include "servo_driver.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

static const char *TAG = "SERVO_DEMO";

void app_main(void)
{
    // 配置 3 路舵机（按自己的接线改 gpio_num 和软限位）
    servo_channel_cfg_t chans[] = {
        {.gpio_num = 9, .min_angle = 45.0f, .max_angle = 135.0f},  // 例：头部
        {.gpio_num = 14, .min_angle = 10.0f, .max_angle = 160.0f}, // 例：左臂
        {.gpio_num = 4, .min_angle = 10.0f, .max_angle = 160.0f},  // 例：右臂
    };

    if (servo_driver_init(chans, 3) != ESP_OK)
    {
        ESP_LOGE(TAG, "舵机初始化失败");
        return;
    }

    // 用并行函数预设几个"组合动作"（三轴同步到位，整体协调不割裂）
    // 数组下标对应通道：[0]=头 [1]=左臂 [2]=右臂
    const float ACT_CENTER[3] = {90.0f, 90.0f, 90.0f};  // 全部归中（立正）
    const float ACT_HANDS_UP[3] = {90.0f, 160.0f, 160.0f}; // 双臂举高（万岁）
    const float ACT_CHEER[3] = {120.0f, 160.0f, 30.0f};    // 歪头 + 一上一下（庆祝）
    const float ACT_GREET[3] = {60.0f, 30.0f, 150.0f};     // 反方向歪头招手

    while (1)
    {
        ESP_LOGI(TAG, "组合动作：双臂举高（三轴并行）");
        servo_move_all_parallel(ACT_HANDS_UP, SERVO_SPEED_FAST, 3);
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "组合动作：庆祝（歪头+一上一下）");
        servo_move_all_parallel(ACT_CHEER, SERVO_SPEED_MID, 3);
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "组合动作：招手问好");
        servo_move_all_parallel(ACT_GREET, SERVO_SPEED_MID, 3);
        vTaskDelay(pdMS_TO_TICKS(800));

        ESP_LOGI(TAG, "组合动作：归中立正");
        servo_move_all_parallel(ACT_CENTER, SERVO_SPEED_MID, 3);
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}
