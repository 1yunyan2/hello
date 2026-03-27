#pragma once

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2s_std.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_err.h"
#include "bsp/bsp_config.h"

// ES8311 官方驱动头文件
#include "esp_codec_dev.h"
#include "esp_codec_dev_defaults.h"
#include "wake_words/custom_wake_word.h"

void audio_init(void);
void audio_feed_task(void *arg);