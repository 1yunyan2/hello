// #include "bsp_board.h"
// #include "esp_codec_dev_defaults.h"
// #include "driver/i2s_std.h"
// #include "driver/i2c_master.h"

// void bsp_board_codec_init(bsp_board_t *bsp_board)
// {
//     // ========== 1. 创建 I2C 控制接口 ==========
//     i2c_master_bus_handle_t bus_handle = NULL;
//     bsp_board_codec_i2c_init(bsp_board, &bus_handle);

//     audio_codec_i2c_cfg_t i2c_cfg = {
//         .bus_handle = bus_handle,          // I2C 总线句柄
//         .addr = ES8311_CODEC_DEFAULT_ADDR, // ES8311 默认地址
//     };
//     const audio_codec_ctrl_if_t *ctrl_if = audio_codec_new_i2c_ctrl(&i2c_cfg);

//     // ========== 2. 创建 GPIO 控制接口 ==========
//     const audio_codec_gpio_if_t *gpio_if = audio_codec_new_gpio();

//     // ========== 3. 配置 ES8311 Codec 参数 ==========
//     es8311_codec_cfg_t es8311_cfg = {
//         .ctrl_if = ctrl_if,                         // 控制接口
//         .gpio_if = gpio_if,                         // GPIO 接口
//         .pa_pin = BSP_CODEC_PA_PIN,                 // 功放使能引脚
//         .codec_mode = ESP_CODEC_DEV_WORK_MODE_BOTH, // 双工模式（录音 + 播放）
//         .use_mclk = true,                           // 使用主时钟
//     };
//     const audio_codec_if_t *codec_if = es8311_codec_new(&es8311_cfg);

//     // ========== 4. 创建 I2S 数据接口 ==========
//     i2s_chan_handle_t rx_handle = NULL, tx_handle = NULL;
//     bsp_board_codec_i2s_init(bsp_board, &rx_handle, &tx_handle);

//     audio_codec_i2s_cfg_t i2s_config = {
//         .rx_handle = rx_handle, // 接收通道（录音）
//         .tx_handle = tx_handle, // 发送通道（播放）
//     };
//     const audio_codec_data_if_t *data_if = audio_codec_new_i2s_data(&i2s_config);

//     // ========== 5. 创建音频设备句柄 ==========
//     esp_codec_dev_cfg_t codec_config = {
//         .dev_type = ESP_CODEC_DEV_TYPE_IN_OUT, // 输入输出类型
//         .codec_if = codec_if,                  // Codec 接口
//         .data_if = data_if,                    // 数据接口
//     };
//     bsp_board->codec_dev = esp_codec_dev_new(&codec_config);
//     assert(bsp_board->codec_dev);

//     // 设置 Codec 初始化完成标志
//     // xEventGroupSetBits(bsp_board->board_status, CODEC_BIT);
// }
