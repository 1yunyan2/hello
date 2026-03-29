#include "bsp_board.h"

bsp_board_t *bsp_board_get_instance(void)
{
    return 0;
}

void bsp_board_nvs_init(void)
{

    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||   // NVS分区表已满
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) // NVS分区表被格式化
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        nvs_flash_init();
    }
}
