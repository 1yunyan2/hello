#ifndef CUSTOM_WAKE_WORD_H
#define CUSTOM_WAKE_WORD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>  // ?
#include "esp_err.h" // ?

#include "custom_wake_word.h"
#include <stdio.h>
#include <string.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "nvs_flash.h"
#include "nvs.h"

// 乐鑫 ESP-SR 命令词识别头文件
#include "esp_mn_iface.h"
#include "esp_mn_models.h"
#include "esp_mn_speech_commands.h"

#ifdef __cplusplus
extern "C"
{
#endif

    // 定义当唤醒词被触发时的回调函数类型
    typedef void (*wake_word_detected_cb_t)(const char *wake_word_pinyin);

    /**
     * @brief 初始化自定义唤醒词引擎
     * @param cb 当听到唤醒词时触发的回调函数
     * @return esp_err_t
     */
    esp_err_t custom_wake_word_init(wake_word_detected_cb_t cb);

    /**
     * @brief 动态更新自定义唤醒词 (通过手机蓝牙接收后调用此函数)
     * @param new_pinyin 新的拼音，格式必须为纯小写且带空格，如 "xiao zhi"
     * @return esp_err_t
     */
    esp_err_t custom_wake_word_update(const char *new_pinyin);

    /**
     * @brief 获取引擎每次需要处理的音频采样点数量
     * @return size_t 采样点数量 (一般为 512)
     */
    size_t custom_wake_word_get_chunksize(void);

    /**
     * @brief 将麦克风采集的音频数据喂给引擎
     * @param data 16-bit PCM 音频数据指针
     * @param len 数据长度 (采样点个数，非字节数)
     */
    void custom_wake_word_feed(const int16_t *data, size_t len);

    /**
     * @brief 停止引擎监听
     */
    void custom_wake_word_stop(void);

    /**
     * @brief 恢复引擎监听
     */
    void custom_wake_word_start(void);

#ifdef __cplusplus
}
#endif

#endif // CUSTOM_WAKE_WORD_H