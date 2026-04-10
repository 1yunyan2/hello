#include "bsp_board.h"

/**
 * @brief 全局唯一 BSP 实例
 *
 * 静态分配，程序整个生命周期存在，不可销毁。
 * 初始值全为 0（board_status = NULL，codec_dev = NULL 等）。
 * 通过 bsp_board_get_instance() 获取，禁止外部直接访问此变量。
 */
static bsp_board_t bsp_board = {0};

// ─── bsp_board_get_instance ──────────────────────────────────────────────────

/**
 * @brief 获取全局唯一 BSP 单例实例（懒初始化 EventGroup）
 *
 * 首次调用时检测 board_status 是否为 NULL，若是则创建 FreeRTOS EventGroup。
 * EventGroup 用于跨任务/模块的状态同步，是整个 BSP 模块的核心同步原语。
 *
 * @param  无
 * @return bsp_board_t* 全局 BSP 实例指针，永远不为 NULL
 *
 * @note 调用者：application.c → application_init()（系统启动第一步）
 * @note 线程安全：首次调用在 app_main 单线程阶段，无竞争风险
 */
bsp_board_t *bsp_board_get_instance(void)
{
    // ── 懒初始化：检查事件组是否已创建 ──────────────────────────────────────
    // board_status 初始值为 NULL（全局 bss 段清零），只在首次调用时创建
    if (!bsp_board.board_status)
    {
        // 创建 FreeRTOS 事件组：支持 24 个独立状态位（BIT0~BIT23）
        // 各模块通过置位/清位/等待 BIT 实现无轮询的同步等待
        bsp_board.board_status = xEventGroupCreate();
    }

    // 返回全局唯一实例指针（调用方不得 free 此指针）
    return &bsp_board;
}

// ─── bsp_board_nvs_init ──────────────────────────────────────────────────────

/**
 * @brief 初始化 NVS Flash（非易失存储），完成后置位 NVS_BIT
 *
 * NVS 用于持久化存储：WiFi 凭证、device_token、唤醒词、MQTT 凭证等。
 * 若分区格式损坏或固件版本不匹配，自动擦除后重建（所有存储数据丢失）。
 *
 * @param bsp_board BSP 实例指针（通过 board_status 置位 NVS_BIT）
 * @return void（失败时 ESP_ERROR_CHECK 触发系统重启）
 *
 * @note 调用者：application.c → application_init()（步骤 2，NVS 初始化）
 * @note 必须早于所有使用 NVS 的模块：bsp_wifi、custom_wake_word、session、auth
 */
void bsp_board_nvs_init(bsp_board_t *bsp_board)
{
    // ── 步骤 1：尝试初始化 NVS Flash ─────────────────────────────────────────
    // nvs_flash_init() 扫描 NVS 分区表，加载键值对索引到内存
    esp_err_t ret = nvs_flash_init();

    // ── 步骤 2：处理分区损坏或版本不匹配 ─────────────────────────────────────
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES ||   // NVS 分区没有可用空闲页（写入过多）
        ret == ESP_ERR_NVS_NEW_VERSION_FOUND) // NVS 版本与当前固件不兼容（固件升级后）
    {
        // 擦除整个 NVS 分区（慎重：会清除所有已保存的配置，包括 WiFi 密码和 Token）
        ESP_ERROR_CHECK(nvs_flash_erase());
        // 擦除后重新初始化，此时分区已空，初始化必然成功
        ret = nvs_flash_init();
    }

    // 若仍然失败（硬件故障或分区表错误），触发 panic 重启
    ESP_ERROR_CHECK(ret);

    // ── 步骤 3：置位 NVS_BIT，通知其他模块 NVS 已就绪 ───────────────────────
    // bsp_board_wifi_main() 会在开始前检查此位，确保 WiFi 凭证可读
    xEventGroupSetBits(bsp_board->board_status, NVS_BIT);
}
