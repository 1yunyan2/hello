#include "bsp_board.h"

// 全局唯一 BSP 实例，程序生命周期内只有一个
static bsp_board_t bsp_board = {0};

/**
 * @brief 获取板级支持包单例实例
 */
bsp_board_t *bsp_board_get_instance(void)
{
    // 判断事件组是否已创建（首次调用时为 NULL）
    if (!bsp_board.board_status)
    {
        // 创建 FreeRTOS 事件组，用于多模块间的状态同步
        bsp_board.board_status = xEventGroupCreate();
    }
    // 返回全局唯一实例指针
    return &bsp_board;
}

void bsp_board_nvs_init(bsp_board_t *bsp_board)
{
    // 尝试初始化 NVS Flash（非易失存储）
    esp_err_t ret = nvs_flash_init();

    // 若 NVS 分区表已满或版本不匹配，则先擦除再重新初始化
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||   // NVS 分区没有可用页
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) // NVS 版本与当前固件不兼容
    {
        // 擦除整个 NVS 分区
        ESP_ERROR_CHECK(nvs_flash_erase());
        // 擦除后重新初始化
        ret = nvs_flash_init();
    }

    // 若仍然失败则触发致命错误（系统无法继续运行）
    ESP_ERROR_CHECK(ret);

    // 设置 NVS_BIT，通知其他模块 NVS 已就绪
    xEventGroupSetBits(bsp_board->board_status, NVS_BIT);
}
