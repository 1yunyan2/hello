// #include <stdio.h>
// #include <string.h>
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "nvs_flash.h"
// #include "esp_wifi.h"
// #include "esp_event.h"
// #include "esp_http_server.h"
// #include "esp_log.h"
// #include "driver/i2c.h"
// #include "driver/i2s_std.h"
// #include "es8311.h"
// #include "esp_sr.h"

// // 硬件配置（根据你的实际接线修改）
// #define I2C_SDA_PIN GPIO_NUM_4
// #define I2C_SCL_PIN GPIO_NUM_5
// #define I2S_BCLK_PIN GPIO_NUM_16
// #define I2S_LRCLK_PIN GPIO_NUM_17
// #define I2S_DIN_PIN GPIO_NUM_15

// // WiFi配置
// #define WIFI_MODE_AP 1 // 1=AP热点模式 0=STA连接现有WiFi
// #define WIFI_SSID "AI-Doll-Config"
// #define WIFI_PASS "12345678"

// // NVS配置
// #define NVS_NAMESPACE "wakeword_config"
// #define NVS_KEY "custom_wakeword"
// #define MAX_WAKEWORD_LEN 32

// static const char *TAG = "WakeWordSystem";
// char current_wakeword[MAX_WAKEWORD_LEN] = "小爱同学";
// static esp_sr_handle_t sr_handle = NULL;
// static i2s_chan_handle_t rx_chan = NULL;
// static TaskHandle_t wakeword_detect_task_handle = NULL;

// // ES8311初始化
// static void es8311_init(void)
// {
//     i2c_config_t i2c_conf = {
//         .mode = I2C_MODE_MASTER,
//         .sda_io_num = I2C_SDA_PIN,
//         .scl_io_num = I2C_SCL_PIN,
//         .sda_pullup_en = GPIO_PULLUP_ENABLE,
//         .scl_pullup_en = GPIO_PULLUP_ENABLE,
//         .master.clk_speed = 100000,
//     };
//     i2c_param_config(I2C_NUM_0, &i2c_conf);
//     i2c_driver_install(I2C_NUM_0, i2c_conf.mode, 0, 0, 0);

//     es8311_codec_init();
//     es8311_set_adc_gain(ES8311_ADC_GAIN_12DB);
//     ESP_LOGI(TAG, "ES8311语音模块初始化完成");
// }

// // I2S音频采集初始化
// static void i2s_init(void)
// {
//     i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
//     i2s_new_channel(&chan_cfg, NULL, &rx_chan);

//     i2s_std_config_t std_cfg = {
//         .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(16000),
//         .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(I2S_DATA_BIT_WIDTH_16BIT, I2S_SLOT_MODE_MONO),
//         .gpio_cfg = {
//             .bclk = I2S_BCLK_PIN,
//             .ws = I2S_LRCLK_PIN,
//             .dout = I2S_GPIO_UNUSED,
//             .din = I2S_DIN_PIN,
//             .invert_flags = {
//                 .bclk_inv = false,
//                 .ws_inv = false,
//             },
//         },
//     };

//     i2s_channel_init_std_mode(rx_chan, &std_cfg);
//     i2s_channel_enable(rx_chan);
//     ESP_LOGI(TAG, "I2S音频采集初始化完成");
// }

// // 从NVS读取唤醒词
// static esp_err_t read_wakeword_from_nvs(void)
// {
//     nvs_handle_t nvs_handle;
//     esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &nvs_handle);
//     if (err != ESP_OK)
//     {
//         ESP_LOGW(TAG, "使用默认唤醒词: %s", current_wakeword);
//         return err;
//     }

//     size_t len = MAX_WAKEWORD_LEN;
//     err = nvs_get_str(nvs_handle, NVS_KEY, current_wakeword, &len);
//     nvs_close(nvs_handle);

//     if (err == ESP_OK)
//     {
//         ESP_LOGI(TAG, "读取唤醒词成功: %s", current_wakeword);
//     }
//     return err;
// }

// // 保存唤醒词到NVS
// static esp_err_t save_wakeword_to_nvs(const char *new_wakeword)
// {
//     if (new_wakeword == NULL || strlen(new_wakeword) == 0 || strlen(new_wakeword) >= MAX_WAKEWORD_LEN)
//     {
//         return ESP_ERR_INVALID_ARG;
//     }

//     nvs_handle_t nvs_handle;
//     esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &nvs_handle);
//     if (err != ESP_OK)
//         return err;

//     err = nvs_set_str(nvs_handle, NVS_KEY, new_wakeword);
//     if (err == ESP_OK)
//     {
//         err = nvs_commit(nvs_handle);
//         if (err == ESP_OK)
//         {
//             strncpy(current_wakeword, new_wakeword, MAX_WAKEWORD_LEN - 1);
//             current_wakeword[MAX_WAKEWORD_LEN - 1] = '\0';
//         }
//     }

//     nvs_close(nvs_handle);
//     return err;
// }

// // 生成并加载新的唤醒词模型
// static esp_err_t generate_and_load_wakeword_model(const char *wakeword)
// {
//     // 停止当前的语音识别
//     if (wakeword_detect_task_handle != NULL)
//     {
//         vTaskDelete(wakeword_detect_task_handle);
//         wakeword_detect_task_handle = NULL;
//     }

//     if (sr_handle != NULL)
//     {
//         esp_sr_destroy(sr_handle);
//         sr_handle = NULL;
//     }

//     ESP_LOGI(TAG, "正在生成唤醒词模型: %s", wakeword);

//     // 纯文本生成唤醒词模型（ESP-SR 2.4.2核心功能）
//     esp_sr_model_data_t *model_data = esp_sr_wake_word_model_generate_from_text(
//         wakeword,
//         ESP_SR_LANGUAGE_CHINESE,
//         ESP_SR_MODEL_SIZE_SMALL);

//     if (model_data == NULL)
//     {
//         ESP_LOGE(TAG, "唤醒词模型生成失败");
//         return ESP_FAIL;
//     }

//     // 初始化语音识别引擎
//     esp_sr_config_t sr_config = {
//         .model_data = model_data,
//         .sample_rate = 16000,
//         .channels = 1,
//         .bits_per_sample = 16,
//     };

//     esp_err_t err = esp_sr_init(&sr_config, &sr_handle);
//     if (err != ESP_OK)
//     {
//         ESP_LOGE(TAG, "语音识别引擎初始化失败: %s", esp_err_to_name(err));
//         esp_sr_wake_word_model_free(model_data);
//         return err;
//     }

//     ESP_LOGI(TAG, "唤醒词模型加载成功");
//     return ESP_OK;
// }

// // 唤醒词检测任务
// static void wakeword_detect_task(void *arg)
// {
//     int16_t audio_buffer[512];
//     size_t bytes_read;

//     while (1)
//     {
//         i2s_channel_read(rx_chan, audio_buffer, sizeof(audio_buffer), &bytes_read, portMAX_DELAY);

//         if (bytes_read > 0 && sr_handle != NULL)
//         {
//             esp_sr_result_t result;
//             esp_err_t err = esp_sr_process(sr_handle, audio_buffer, bytes_read / 2, &result);

//             if (err == ESP_OK && result.detected)
//             {
//                 ESP_LOGI(TAG, "唤醒词检测成功: %s", current_wakeword);
//                 // 在这里添加唤醒后的业务逻辑
//                 // 例如：播放提示音、启动对话等
//             }
//         }

//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// // HTTP API: 获取当前唤醒词
// static esp_err_t api_get_wakeword(httpd_req_t *req)
// {
//     httpd_resp_set_type(req, "application/json");
//     char resp[128];
//     snprintf(resp, sizeof(resp), "{\"code\":0,\"wakeword\":\"%s\"}", current_wakeword);
//     httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
//     return ESP_OK;
// }

// // HTTP API: 设置新唤醒词
// static esp_err_t api_set_wakeword(httpd_req_t *req)
// {
//     char buf[256];
//     int ret = httpd_req_recv(req, buf, sizeof(buf) - 1);
//     if (ret <= 0)
//     {
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "请求数据为空");
//         return ESP_FAIL;
//     }
//     buf[ret] = '\0';

//     // 解析JSON请求
//     char *wakeword_start = strstr(buf, "\"wakeword\":\"");
//     if (wakeword_start == NULL)
//     {
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "请求格式错误");
//         return ESP_FAIL;
//     }
//     wakeword_start += 11;

//     char *wakeword_end = strchr(wakeword_start, '"');
//     if (wakeword_end == NULL)
//     {
//         httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "唤醒词格式错误");
//         return ESP_FAIL;
//     }
//     *wakeword_end = '\0';

//     // 保存并生成新模型
//     esp_err_t err = save_wakeword_to_nvs(wakeword_start);
//     if (err != ESP_OK)
//     {
//         httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "保存唤醒词失败");
//         return ESP_FAIL;
//     }

//     err = generate_and_load_wakeword_model(wakeword_start);
//     if (err != ESP_OK)
//     {
//         httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "生成唤醒词模型失败");
//         // 恢复原来的唤醒词
//         generate_and_load_wakeword_model(current_wakeword);
//         return ESP_FAIL;
//     }

//     // 重新启动唤醒词检测任务
//     xTaskCreate(wakeword_detect_task, "wakeword_detect", 4096, NULL, 5, &wakeword_detect_task_handle);

//     httpd_resp_set_type(req, "application/json");
//     const char *resp = "{\"code\":0,\"message\":\"唤醒词设置成功，已实时生效\"}";
//     httpd_resp_send(req, resp, HTTPD_RESP_USE_STRLEN);
//     return ESP_OK;
// }

// // 启动HTTP服务器
// static void start_http_server(void)
// {
//     httpd_config_t config = HTTPD_DEFAULT_CONFIG();
//     config.server_port = 80;
//     httpd_handle_t server = NULL;

//     if (httpd_start(&server, &config) == ESP_OK)
//     {
//         httpd_uri_t get_uri = {
//             .uri = "/api/wakeword",
//             .method = HTTP_GET,
//             .handler = api_get_wakeword};
//         httpd_register_uri_handler(server, &get_uri);

//         httpd_uri_t set_uri = {
//             .uri = "/api/wakeword",
//             .method = HTTP_POST,
//             .handler = api_set_wakeword};
//         httpd_register_uri_handler(server, &set_uri);

//         ESP_LOGI(TAG, "HTTP服务器已启动");
// #if WIFI_MODE_AP
//         ESP_LOGI(TAG, "请连接WiFi: %s，密码: %s", WIFI_SSID, WIFI_PASS);
//         ESP_LOGI(TAG, "然后在APP中访问: http://192.168.4.1/api/wakeword");
// #else
//         ESP_LOGI(TAG, "设备已连接到WiFi: %s", STA_SSID);
// #endif
//     }
// }

// // WiFi初始化
// static void wifi_init(void)
// {
//     ESP_ERROR_CHECK(esp_netif_init());
//     ESP_ERROR_CHECK(esp_event_loop_create_default());

// #if WIFI_MODE_AP
//     esp_netif_create_default_wifi_ap();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     wifi_config_t wifi_config = {
//         .ap = {
//             .ssid = WIFI_SSID,
//             .ssid_len = strlen(WIFI_SSID),
//             .password = WIFI_PASS,
//             .max_connection = 1,
//             .authmode = WIFI_AUTH_WPA2_PSK},
//     };

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_start());
// #else
//     esp_netif_create_default_wifi_sta();
//     wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
//     ESP_ERROR_CHECK(esp_wifi_init(&cfg));

//     wifi_config_t wifi_config = {
//         .sta = {
//             .ssid = STA_SSID,
//             .password = STA_PASS,
//         },
//     };

//     ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
//     ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
//     ESP_ERROR_CHECK(esp_wifi_connect());
// #endif
// }

// void app_main(void)
// {
//     // 初始化NVS
//     esp_err_t ret = nvs_flash_init();
//     if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
//     {
//         ESP_ERROR_CHECK(nvs_flash_erase());
//         ret = nvs_flash_init();
//     }
//     ESP_ERROR_CHECK(ret);

//     // 初始化硬件
//     es8311_init();
//     i2s_init();

//     // 读取并加载初始唤醒词
//     read_wakeword_from_nvs();
//     generate_and_load_wakeword_model(current_wakeword);

//     // 启动唤醒词检测任务
//     xTaskCreate(wakeword_detect_task, "wakeword_detect", 4096, NULL, 5, &wakeword_detect_task_handle);

//     // 启动WiFi和HTTP服务器
//     wifi_init();
//     start_http_server();
// }