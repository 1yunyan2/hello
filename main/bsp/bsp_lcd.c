#include "bsp_board.h"
#include "driver/gpio.h"
#include "driver/ledc.h"      // 背光 PWM 调光（LEDC 外设）
#include "esp_lcd_panel_io.h" // esp_lcd_panel_io_tx_param（写 ST7789 厂商寄存器）
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_timer.h" // bsp_board_lcd_fade_brightness_fine：按真实经过时间推进渐变

/**
 * @brief 设置 LCD 背光亮度（内部 helper，LEDC PWM 占空比方式）
 *
 * 将 0~100 的百分比线性映射到 10 位 LEDC 占空比（0~1023），写入并立即生效。
 * 占空比越大背光越亮（对应硬件「高电平开启背光」）。
 *
 * @param pct 亮度百分比，0=熄灭，100=最亮（超过 100 自动钳到 100）
 * @return void
 *
 * @note 调用者：bsp_board_lcd_init/on/off、bsp_board_lcd_set_brightness（待机模块）
 * @note 前置条件：bsp_board_lcd_init() 已完成 LEDC timer/channel 配置
 */
static uint8_t s_bk_cur_pct = 0; // 最后一次写入的亮度百分比（供换图渐亮恢复基准，不写死 100%）

static void bsp_lcd_bk_set_percent(uint8_t pct)
{
    if (pct > 100)
        pct = 100; // 钳位，防止占空比越界
    // ★ 实测结论：本板背光为「高电平点亮」——GPIO42 高电平占比越大越亮。
    //   （两次实测确认：duty=DUTY_MAX/高电平→全亮；duty=0/低电平→全灭，故不取反。）
    //   duty 直接正比于亮度：pct=100→duty=DUTY_MAX(最亮)；pct=0→duty=0(灭)。
    uint32_t duty = (uint32_t)pct * BSP_LCD_BK_DUTY_MAX / 100;
    esp_err_t e1 = ledc_set_duty(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_CHANNEL, duty);
    esp_err_t e2 = ledc_update_duty(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_CHANNEL);
    s_bk_cur_pct = pct; // 记录最后一次写入亮度，供换图渐亮恢复基准
    // 【诊断】打印实际写入的占空比与返回值，确认软件链路生效（验证 OK 后可删）
    // ESP_LOGW("BSP_LCD_BK", "亮度=%u%% → duty=%lu/%d (高电平点亮, set=%d upd=%d)",
    //          pct, (unsigned long)duty, BSP_LCD_BK_DUTY_MAX, e1, e2);
}

/**
 * @brief 设置 LCD 背光亮度（对外 API，供待机/省电模块调用）
 *
 * @param percent 亮度百分比 0~100
 * @return void
 * @note 调用者：standby 待机模块（进入待机降至 50%，退出恢复 100%）
 */
void bsp_board_lcd_set_brightness(uint8_t percent)
{
    bsp_lcd_bk_set_percent(percent);
}

/**
 * @brief 读取当前背光亮度（最后一次写入的百分比）
 *
 * 供换图渐亮在 standby 降亮度（如 BSP_LCD_BK_STANDBY_PCT）后仍恢复到正确基准，
 * 而不是擅自写死 100%。
 */
uint8_t bsp_board_lcd_get_brightness(void)
{
    return s_bk_cur_pct;
}

// 背光线性渐变总耗时：与手臂归中动态调速共用 BSP_STANDBY_ENTER_TRANSITION_MS（bsp_config.h），
// 确保 enter_standby() 时熄屏和手臂归中同时完成，不要改成独立数值。
#define BSP_LCD_BK_FADE_MS BSP_STANDBY_ENTER_TRANSITION_MS
#define BSP_LCD_BK_FADE_STEPS 16 // 步数，每步间隔 = FADE_MS/STEPS

/**
 * @brief 背光亮度线性渐变（阻塞，供待机模块进/退一级低功耗调用）
 *
 * 从 from_pct 线性渐变到 to_pct，分 BSP_LCD_BK_FADE_STEPS 步、每步间隔
 * BSP_LCD_BK_FADE_MS/FADE_STEPS 毫秒，总耗时 BSP_LCD_BK_FADE_MS，避免瞬间跳变。
 *
 * @param from_pct 起始亮度百分比 0~100
 * @param to_pct   目标亮度百分比 0~100
 * @note 阻塞：本函数会 vTaskDelay 直到渐变完成（约 BSP_LCD_BK_FADE_MS），
 *       调用者须能接受此阻塞（当前设计为 standby_task 自身低优先级任务内调用）。
 */
void bsp_board_lcd_fade_brightness(uint8_t from_pct, uint8_t to_pct)
{
    if (from_pct > 100)
        from_pct = 100;
    if (to_pct > 100)
        to_pct = 100;

    const uint32_t step_ms = BSP_LCD_BK_FADE_MS / BSP_LCD_BK_FADE_STEPS;
    for (int s = 1; s <= BSP_LCD_BK_FADE_STEPS; s++)
    {
        int pct = (int)from_pct + ((int)to_pct - (int)from_pct) * s / BSP_LCD_BK_FADE_STEPS;
        bsp_lcd_bk_set_percent((uint8_t)pct);
        vTaskDelay(pdMS_TO_TICKS(step_ms));
    }
    bsp_lcd_bk_set_percent(to_pct); // 兜底精确落点
}

/**
 * @brief 背光亮度渐变——非阻塞单步版（按已过时间算当前应有亮度并设置一次）
 *
 * 与 bsp_board_lcd_fade_brightness()（阻塞版）配套：供不能阻塞的调用方
 * （如 standby_wake() 可能被触摸/MQTT任务同步调用）使用——只置标志记录起点，
 * 真正的渐变推进交给低优先级轮询任务（standby_task）每 tick 调本函数一次。
 *
 * @param from_pct    起始亮度百分比 0~100
 * @param to_pct      目标亮度百分比 0~100
 * @param elapsed_ms  距渐变开始已过的毫秒数
 * @param total_ms    渐变总耗时（毫秒），通常传 BSP_LCD_BK_FADE_MS 的调用方常量
 * @return true=渐变已到达终点（elapsed_ms >= total_ms，已设为 to_pct），false=尚在进行中
 * @note 非阻塞，无 vTaskDelay，可在任意任务的轮询循环中调用。
 */
bool bsp_board_lcd_fade_step(uint8_t from_pct, uint8_t to_pct, uint32_t elapsed_ms, uint32_t total_ms)
{
    if (from_pct > 100)
        from_pct = 100;
    if (to_pct > 100)
        to_pct = 100;

    if (total_ms == 0 || elapsed_ms >= total_ms)
    {
        bsp_lcd_bk_set_percent(to_pct);
        return true;
    }
    int pct = (int)from_pct + ((int)to_pct - (int)from_pct) * (int)elapsed_ms / (int)total_ms;
    bsp_lcd_bk_set_percent((uint8_t)pct);
    return false;
}

/**
 * @brief 感知亮度线性的背光渐变——非阻塞单步版（伽马校正 + 全 10 位精度）
 *
 * 与 bsp_board_lcd_fade_step() 的区别（这是"渐变不丝滑"的两个根因）：
 *
 *  ① 精度：旧版进度经过 0~100 的整数百分比，只有 101 个档位，再乘 1023/100 映射
 *     到 duty。开机渐变 1200ms/20ms = 60 步，整数除法让相邻步常算出同一个 pct、
 *     偶尔跳 2，台阶不均匀。本版进度用 0~1024 定点直算 duty，硬件 10 位精度全部用上。
 *
 *  ② 伽马：人眼亮度感知近似 2.2 次方，而 duty 与亮度是线性关系。线性扫 duty 的结果是
 *     亮端 90%→100% 几乎看不出变化（时间被浪费），暗端 3%→2%→1% 却是 33%、50% 的
 *     感知跳变——所有可见跳变都挤在暗端。本版先对归一化进度做 t^2.2 再映射 duty，
 *     使【感知亮度】随时间线性变化：暗端展开、亮端压缩，跳变被均摊掉。
 *
 * 伽马用整数近似 t^2.2 ≈ t^2 * (0.8 + 0.2*t)，全程 int32 定点（Q10），不引入浮点
 * （LVGL 线程每 16~20ms 调一次，避免软浮点开销与潜在的 FPU 上下文问题）。
 *
 * @param from_pct   起始亮度百分比 0~100
 * @param to_pct     目标亮度百分比 0~100
 * @param elapsed_ms 距渐变开始已过的毫秒数
 * @param total_ms   渐变总耗时（毫秒）
 * @return true=已到达终点（本次已设为 to_pct），false=仍在渐变中
 * @note 非阻塞，无 vTaskDelay。当前调用者：ui_port.c ui_boot_fade_timer_cb（开机 logo 渐变）
 * @note 不替换 bsp_board_lcd_fade_step()：standby 的一/二级待机仍用旧版，行为不受影响。
 */
bool bsp_board_lcd_fade_step_fine(uint8_t from_pct, uint8_t to_pct, uint32_t elapsed_ms, uint32_t total_ms)
{
    if (from_pct > 100)
        from_pct = 100;
    if (to_pct > 100)
        to_pct = 100;

    if (total_ms == 0 || elapsed_ms >= total_ms)
    {
        bsp_lcd_bk_set_percent(to_pct); // 精确落点，走百分比接口保证与其它路径终值一致
        return true;
    }

    /* 起止亮度换算到 Q10 定点的归一化值（0~1024），不再受 101 级百分比限制 */
    const int32_t from_q = (int32_t)from_pct * 1024 / 100;
    const int32_t to_q = (int32_t)to_pct * 1024 / 100;

    /* 时间进度 t ∈ [0,1024)，按已过时间线性推进 */
    const int32_t t = (int32_t)((uint64_t)elapsed_ms * 1024 / total_ms);

    /* ① 先在【感知域】按时间线性插值，得到当前应有的"感知亮度" L。
     *    ★伽马必须作用在亮度轴上、与时间方向无关：若改为对 t 先做伽马再插值，
     *    曲线在渐亮方向正确、渐暗方向却会翻转成"先慢后快"，把跳变全甩到暗端
     *    （实测 duty 末段每步跨 130~180），等于换个方向复发原来的毛病。 */
    int32_t l = from_q + (to_q - from_q) * t / 1024;
    if (l < 0)
        l = 0;
    if (l > 1024)
        l = 1024;

    /* ② 再对感知亮度整体做伽马 L^2.2 ≈ L^2 * (0.8 + 0.2*L)，映射到物理 duty。
     *    Q10 定点逐级降幂避免溢出：
     *      l2 = l*l >> 10              （L^2）
     *      k  = 819 + (205 * l >> 10)  （0.8 + 0.2L，819≈0.8*1024，205≈0.2*1024）
     *      g  = l2 * k >> 10           （L^2 * k）
     *    两个方向都得到同一条亮度曲线，只是扫描方向相反，观感对称。 */
    const int32_t l2 = (l * l) >> 10;
    const int32_t k = 819 + ((205 * l) >> 10);
    int32_t g = (l2 * k) >> 10;
    if (g > 1024)
        g = 1024; // 钳位，防定点累积误差越界

    uint32_t duty = (uint32_t)g * BSP_LCD_BK_DUTY_MAX / 1024;
    ledc_set_duty(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_CHANNEL, duty);
    ledc_update_duty(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_CHANNEL);
    return false;
}

/* 每步间隔（ms）：FreeRTOS tick = 10ms（CONFIG_FREERTOS_HZ=100），取 1 个 tick，
 * 即最细的可行粒度。1000ms 的渐变可分 100 步，远密于旧版固定 16 步。 */
#define BSP_LCD_BK_FADE_FINE_STEP_MS 10

/**
 * @brief 感知亮度线性的背光渐变——【阻塞版】（伽马校正 + 全 10 位精度 + 时长可传）
 *
 * 与 bsp_board_lcd_fade_brightness()（旧阻塞版）的关系：
 *   旧版固定 BSP_LCD_BK_FADE_STEPS(16) 步、时长写死 BSP_LCD_BK_FADE_MS(2000ms)，
 *   即【每 125ms 才跳一档】，且走线性 duty 无伽马 —— 这正是待机渐暗肉眼可见
 *   "断层/一格一格跳"的两个原因（详见 bsp_board_lcd_fade_step_fine 上方注释对
 *   精度与伽马的完整分析）。旧版保持原样不动，其它调用方行为不受影响。
 *
 * 本版把已有的 bsp_board_lcd_fade_step_fine()（非阻塞单步、开机 logo 渐变在用的
 * 那套丝滑实现）包成阻塞循环，每 BSP_LCD_BK_FADE_FINE_STEP_MS 推进一步，并额外
 * 支持传入总时长 —— 供 standby 的「渐暗到全黑 → 暗中换画面 → 再渐亮」三段式
 * 转场分段控制节奏（每段时长不同，不能再用写死的 2000ms）。
 *
 * @param from_pct  起始亮度百分比 0~100
 * @param to_pct    目标亮度百分比 0~100
 * @param total_ms  本段渐变总耗时（毫秒）；传 0 则直接跳到 to_pct（无渐变）
 * @note 阻塞：内部 vTaskDelay 直到走完 total_ms，调用方须能接受（当前调用方为
 *       standby_task，低优先级轮询任务，可安全阻塞）。
 * @note 实际耗时按 esp_timer 真实经过时间推进，任务调度抖动不会拉长总时长
 *       （只会让中间某几步跨度稍大），保证转场节奏稳定。
 */
void bsp_board_lcd_fade_brightness_fine(uint8_t from_pct, uint8_t to_pct, uint32_t total_ms)
{
    if (total_ms == 0)
    {
        bsp_lcd_bk_set_percent(to_pct > 100 ? 100 : to_pct);
        return;
    }

    const int64_t t0 = esp_timer_get_time();
    while (1)
    {
        uint32_t elapsed = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (bsp_board_lcd_fade_step_fine(from_pct, to_pct, elapsed, total_ms))
            break; // 已到终点（内部已精确落到 to_pct）
        vTaskDelay(pdMS_TO_TICKS(BSP_LCD_BK_FADE_FINE_STEP_MS));
    }
}

/**
 * @brief 初始化 LCD 显示屏（ST7789，240×320，RGB565）
 *
 * 完整 LCD 初始化流程：
 *   1. 配置背光 GPIO（初始关闭，等上层主动开启）
 *   2. 初始化 SPI2 总线（80MHz，自动 DMA 分配）
 *   3. 创建 SPI LCD 通信接口（含 DC/CS/时钟配置）
 *   4. 初始化 ST7789 面板驱动（含颜色反转，IPS 屏必需）
 *   5. 硬件复位 → 软件初始化 → 关闭显示（等待上层显式开启）
 *
 * @param bsp_board BSP 实例指针
 *                  - 输出：lcd_io 字段填充 SPI 接口句柄
 *                  - 输出：lcd_panel 字段填充面板驱动句柄
 * @return void（失败时 ESP_ERROR_CHECK 触发系统重启）
 *
 * @note 调用者：application.c（当前已预留，LCD 功能启用后取消注释）
 * @note 引脚：CS=10, MOSI=11, SCLK=12, DC=13, RST=14, BK=48（bsp_config.h）
 * @note 颜色格式：RGB565（每像素 2 字节），小端字节序
 */
/// @brief LCD 是否已完成初始化。★未配网分支会在 WiFi/BLE 之前提前初始化 LCD
/// 显示配网图，之后主流程 application.c 仍会再调一次本函数；重复执行会二次
/// spi_bus_initialize（返回 INVALID_STATE）并覆盖 panel 句柄造成资源泄漏，
/// 故用本标志早退。
static bool s_lcd_inited = false;

/// @brief 显示控制器当前是否处于"已打开"状态（DISPON 已发、DISP OFF 未发）。
/// ★ 仅用于 bsp_board_lcd_reassert_state() 的早退判据：显示还没打开时不要去
/// 补发 DISPON，否则会抢在开机"先关显示 → 渐亮背光"的流程前面把画面亮出来。
/// 由 bsp_board_lcd_on()/disp_on() 置位、bsp_board_lcd_off()/disp_off() 清零。
static bool s_lcd_display_on = false;

void bsp_board_lcd_init(bsp_board_t *bsp_board)
{
    if (s_lcd_inited)
    {
        ESP_LOGI("BSP_LCD", "LCD 已初始化（配网图提前初始化过），跳过重复初始化");
        return;
    }
    s_lcd_inited = true;

    gpio_reset_pin(BSP_LCD_SCLK_PIN); // 释放 时钟线（SCLK）
    gpio_reset_pin(BSP_LCD_MOSI_PIN); // 释放 数据线（MOSI）
    gpio_reset_pin(BSP_LCD_DC_PIN);   // 释放数据/命令选择（D/C）
    gpio_reset_pin(BSP_LCD_CS_PIN);   // 释放 片选（CS/NSS）
    gpio_reset_pin(BSP_LCD_BK_PIN);   // 释放 背光控制（BK）
    gpio_reset_pin(BSP_LCD_RST_PIN);  // 释放 硬件复位（RST）
    // ── 步骤 1：配置背光 LEDC PWM（调光）────────────────────────────────────
    // 背光引脚（GPIO42）由 LEDC PWM 驱动，支持 0~100% 亮度调节（待机模式需 50%）。
    // LEDC 输出经 GPIO Matrix 自动路由到 GPIO42（ESP32-S3 LEDC 无 IO_MUX 直连），无需手动映射。
    // ★ 使用独立 TIMER_1 + CHANNEL_3，与舵机的 TIMER_0/CH0-2 隔离（见 BUG-015）。
    // 初始占空比 0（背光关闭），避免屏幕在初始化过程中显示乱码。
    ledc_timer_config_t bk_timer_config = {
        .speed_mode = BSP_LCD_BK_LEDC_MODE,     // 低速模式
        .timer_num = BSP_LCD_BK_LEDC_TIMER,     // 独立定时器 TIMER_1
        .duty_resolution = BSP_LCD_BK_LEDC_RES, // 10 位分辨率（0~1023）
        .freq_hz = BSP_LCD_BK_LEDC_FREQ_HZ,     // 5kHz
        .clk_cfg = BSP_LCD_BK_LEDC_CLK,         // RC_FAST：与舵机时钟源统一，避免冲突
    };
    ESP_ERROR_CHECK(ledc_timer_config(&bk_timer_config));

    ledc_channel_config_t bk_channel_config = {
        .gpio_num = BSP_LCD_BK_PIN, // 背光引脚 GPIO42
        .speed_mode = BSP_LCD_BK_LEDC_MODE,
        .channel = BSP_LCD_BK_LEDC_CHANNEL, // 独立通道 CHANNEL_3
        .timer_sel = BSP_LCD_BK_LEDC_TIMER, // 绑定到 TIMER_1
        .duty = 0,                          // 初始占空比 0 = 背光关闭
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&bk_channel_config));

    /* 【诊断·背光调光档位数】排查"渐亮/渐暗一小格一小格跳变"用，结论出来后可删。
     *
     * 判据：LEDC 实际可用分辨率受硬件约束 —— 位数 ≤ log2(时钟源 / 频率)。
     *   本配置 XTAL 40MHz / 5kHz = 8000 → log2(8000) ≈ 12.97 位，10 位理论够用。
     * 若下面打印的【实际频率】明显偏离 5000，或初始化时 IDF 报
     *   "requested frequency and duty resolution can not be achieved"，
     * 说明档位数被硬件砍到远少于 1024，那才是台阶可见的真因（此时应提高
     * BSP_LCD_BK_LEDC_RES 到 12/13 位并同步改 BSP_LCD_BK_DUTY_MAX）。
     * 若实际频率正好 5000、档位数 1024 属实，则软件侧已无余量，需转查屏本身
     * 的背光驱动在低占空比下的非线性（LED 恒流驱动死区）。 */
    {
        uint32_t real_freq = ledc_get_freq(BSP_LCD_BK_LEDC_MODE, BSP_LCD_BK_LEDC_TIMER);
        ESP_LOGW("BK_DIAG", "背光PWM: 期望频率=%dHz 实际频率=%luHz 分辨率=%d位 档位数=%d",
                 BSP_LCD_BK_LEDC_FREQ_HZ, (unsigned long)real_freq,
                 (int)BSP_LCD_BK_LEDC_RES, BSP_LCD_BK_DUTY_MAX + 1);
        /* 逐档扫描实际写入的 duty，验证低亮度区是否真有细分档位：
         * 打印 1%~10% 对应的 duty，若相邻百分比算出同一个 duty，说明暗端确实无余量。 */
        ESP_LOGW("BK_DIAG", "暗端 duty 映射(1%%~10%%): %lu %lu %lu %lu %lu %lu %lu %lu %lu %lu",
                 (unsigned long)(1 * BSP_LCD_BK_DUTY_MAX / 100), (unsigned long)(2 * BSP_LCD_BK_DUTY_MAX / 100),
                 (unsigned long)(3 * BSP_LCD_BK_DUTY_MAX / 100), (unsigned long)(4 * BSP_LCD_BK_DUTY_MAX / 100),
                 (unsigned long)(5 * BSP_LCD_BK_DUTY_MAX / 100), (unsigned long)(6 * BSP_LCD_BK_DUTY_MAX / 100),
                 (unsigned long)(7 * BSP_LCD_BK_DUTY_MAX / 100), (unsigned long)(8 * BSP_LCD_BK_DUTY_MAX / 100),
                 (unsigned long)(9 * BSP_LCD_BK_DUTY_MAX / 100), (unsigned long)(10 * BSP_LCD_BK_DUTY_MAX / 100));
    }

    // ── 步骤 2：初始化 SPI2 总线 ─────────────────────────────────────────────
    // SPI2_HOST（HSPI）：ESP32-S3 第二个 SPI 控制器，支持 DMA 加速传输
    // max_transfer_sz 设为整屏大小，保证一次 flush_fb 不会截断
    spi_bus_config_t buscfg = {
        .sclk_io_num = BSP_LCD_SCLK_PIN, // 时钟线（GPIO12），最高 80MHz
        .mosi_io_num = BSP_LCD_MOSI_PIN, // 数据线（GPIO11），主发从收，LCD 单向写
        .miso_io_num = -1,               // 无 MISO（ST7789 不支持读回，只写）
        .quadwp_io_num = -1,             // 不使用四线 SPI（QSPI）
        .quadhd_io_num = -1,             // 不使用四线 SPI
        // 最大 DMA 传输字节数 = 整屏像素数 × 每像素字节数 + 余量
        // 240×320×2 = 153600 字节 ≈ 150KB，确保整帧刷新不溢出
        .max_transfer_sz = BSP_LCD_WIDTH * BSP_LCD_HEIGHT * 2 + 8,
    };
    // SPI_DMA_CH_AUTO：自动分配 DMA 通道，使用 DMA 可大幅降低 CPU 占用
    // 注意：LCD 引脚（CS=41,MOSI=39,SCLK=38）对应 SPI2_HOST（HSPI），不是 SPI3
    ESP_ERROR_CHECK(spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO));

    // ── 步骤 3：创建 SPI LCD 通信接口 ────────────────────────────────────────
    // 此接口封装了 SPI 事务的时序细节，上层只需调用 esp_lcd_panel_* API
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = BSP_LCD_DC_PIN, // DC 引脚（GPIO13）：高=数据，低=命令
        .cs_gpio_num = BSP_LCD_CS_PIN, // CS 引脚（GPIO10）：低电平选中 LCD
        .pclk_hz = 80 * 1000 * 1000,   // 时钟频率为：80 MHz
        .lcd_cmd_bits = 8,             // 命令字段位宽（ST7789 固定 8-bit）
        .lcd_param_bits = 8,           // 参数字段位宽（ST7789 固定 8-bit）
        .spi_mode = 0,                 // SPI 模式 0（CPOL=0，CPHA=0）
        .trans_queue_depth = 1,        // 事务队列深度（最多 10 个异步事务排队）
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(
        (esp_lcd_spi_bus_handle_t)SPI2_HOST, &io_config, &bsp_board->lcd_io));

    // ── 步骤 4：初始化 ST7789 LCD 面板驱动 ───────────────────────────────────
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = BSP_LCD_RST_PIN,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB, // RGB 像素排列顺序（R高位，B低位）
        .bits_per_pixel = 16,                       // 每像素 16-bit（RGB565 格式）
        .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,  // 小端字节序（ESP32 原生字节序）
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(
        bsp_board->lcd_io, &panel_config, &bsp_board->lcd_panel));

    // ── 步骤 5：关闭背光（等待初始化完成后再开启，避免花屏）────────────────
    bsp_lcd_bk_set_percent(0); // 背光关闭（PWM 占空比 0）

    // ── 步骤 6：硬件复位显示屏 ────────────────────────────────────────────────
    // RST 引脚拉低一段时间，复位 ST7789 内部寄存器到出厂默认值
    ESP_ERROR_CHECK(esp_lcd_panel_reset(bsp_board->lcd_panel));

    // ── 步骤 7：初始化 ST7789 面板驱动（写入初始化寄存器序列）──────────────
    // 内部向 ST7789 发送约 20 条初始化命令，设置显示方向、颜色格式等
    ESP_ERROR_CHECK(esp_lcd_panel_init(bsp_board->lcd_panel));

    // ── 步骤 7.5：提高面板自身扫描频率（FRCTRL2 / 0xC6）────────────────────
    // 【2026-08-19 减轻画面撕裂】
    //
    // 这里调的是「液晶自己多久把屏幕扫一遍」，与 LVGL 的刷新周期
    // (CONFIG_LV_DEF_REFR_PERIOD) 完全是两回事：
    //   · LVGL 刷新周期  = 我们多久往屏上送一次新数据
    //   · 面板扫描频率   = 屏幕多久把 GRAM 里的内容点亮一遍
    //
    // 本屏没有引出 TE 引脚（见 [BUG-041]），SPI 写入 GRAM 与面板扫描无法同步，
    // 写到一半被扫出去就会看到「上半新、下半旧」的错位。撕裂无法消除，但
    // 【撕裂画面会一直显示到下一遍扫描盖掉为止】——默认 60Hz 时要停留 16.7ms，
    // 提到 111Hz 后只停留 9ms，存留时间减半，观感上明显变淡。
    //
    // ST7789 FRCTRL2 取值（bits[4:0] RTNA）：
    //   0x00=119Hz  0x01=111Hz  0x03=99Hz  0x05=90Hz  0x07=82Hz
    //   0x09=75Hz   0x0B=69Hz   0x0D=64Hz  0x0F=60Hz(默认)  0x1F=39Hz
    // 取 0x01(111Hz)：接近上限又留一档余量。若出现闪烁/偏色/花屏，
    // 往回退到 0x03(99Hz) 或 0x05(90Hz)；填 0x0F 即恢复出厂默认。
    //
    // 注意：驱动默认初始化序列不写这个寄存器，故此处必须放在
    // esp_lcd_panel_init() 之后，否则会被初始化序列覆盖。
    {
        const uint8_t frctrl2 = BSP_LCD_FRAME_RATE_REG;
        esp_err_t err = esp_lcd_panel_io_tx_param(bsp_board->lcd_io, 0xC6, &frctrl2, 1);
        if (err != ESP_OK)
        {
            // 不做 ESP_ERROR_CHECK：这只是观感优化，写失败不该让整机起不来
            ESP_LOGW("BSP_LCD", "设置面板扫描频率失败(0x%02X): %s",
                     frctrl2, esp_err_to_name(err));
        }
    }

    // ── 步骤 8：颜色反转（IPS 屏必须开启，否则颜色像底片负片）──────────────
    // 普通 TN 屏不需要反转，IPS 屏（本设备使用）必须开启此选项
    ESP_ERROR_CHECK(esp_lcd_panel_invert_color(bsp_board->lcd_panel, true));

    // 可选配置（根据屏幕安装方向决定是否开启镜像/轴交换）:
    // ESP_ERROR_CHECK(esp_lcd_panel_mirror(bsp_board->lcd_panel, true, false));
    // ESP_ERROR_CHECK(esp_lcd_panel_swap_xy(bsp_board->lcd_panel, true));

    // ── 步骤 9：关闭显示（等待上层主动调用 bsp_board_lcd_on() 开启）────────
    // 初始化完成但不立即显示，让上层决定何时打开（可以先准备好画面再开背光）
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, false));
    xEventGroupSetBits(bsp_board->board_status, LCD_BIT); // 置位 LCD_BIT，通知其他模块 LCD 已就绪
}

/**
 * @brief 打开 LCD 背光和显示
 *
 * 先启用显示控制器输出，再点亮背光 LED，避免背光先亮时看到初始化过渡帧。
 *
 * @param bsp_board BSP 实例指针（访问 lcd_panel 句柄和背光 GPIO）
 * @return void
 *
 * @note 调用者：application.c 或业务层（需要显示时调用）
 * @note 前置条件：bsp_board_lcd_init() 已成功调用
 */
void bsp_board_lcd_on(bsp_board_t *bsp_board)
{
    // 先启用 ST7789 显示输出（DISPON 命令），再点亮背光
    // 顺序：控制器输出 → 背光点亮，避免背光亮时显示未就绪的画面
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, true)); // 启用显示
    s_lcd_display_on = true;                                                // 供 reassert 早退判据用
    bsp_lcd_bk_set_percent(BSP_LCD_BK_DEFAULT_PCT);                         // 点亮背光（默认 100%）
}

/**
 *
 * @brief 关闭 LCD 背光和显示
 *
 * 先关闭背光 LED，再关闭显示控制器，省电效果最佳。
 *
 * @param bsp_board BSP 实例指针（访问 lcd_panel 句柄和背光 GPIO）
 * @return void
 *
 * @note 调用者：业务层（休眠/省电时调用）
 * @note 前置条件：bsp_board_lcd_init() 已成功调用
 */
void bsp_board_lcd_off(bsp_board_t *bsp_board)
{
    // 先关背光（用户立即看不到画面），再关显示控制器
    bsp_lcd_bk_set_percent(0);                                               // 关闭背光（PWM 占空比 0）
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, false)); // 关闭显示
    s_lcd_display_on = false;                                                // 供 reassert 早退判据用
}

void bsp_board_lcd_disp_off(bsp_board_t *bsp_board)
{
    // 只关显示控制器，背光不动（调用方应已通过 bsp_board_lcd_fade_brightness
    // 把背光线性渐暗到 0，此时关显示不会有可见跳变）
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, false));
    s_lcd_display_on = false; // 供 reassert 早退判据用
}

void bsp_board_lcd_disp_on(bsp_board_t *bsp_board)
{
    // 只开显示控制器，背光不动（此刻背光仍为 0，调用方随后应用
    // bsp_board_lcd_fade_brightness 把背光线性渐亮，不会有可见跳变）
    ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(bsp_board->lcd_panel, true));
    s_lcd_display_on = true; // 供 reassert 早退判据用
}

/**
 * @brief 重新断言面板关键状态：SLPOUT → INVON → DISPON（幂等，画面无跳变）
 *
 * 【为什么需要它 —— 2026-09-29 现场故障】
 *   运行中偶发两种症状，且【一旦出现就永久保持】：
 *     · 【整片全黑】背光正常亮着，但屏幕全黑；此时手动切到功能盘（LVGL 换掉
 *       整屏内容）依旧全黑，而触摸、日志、舵机全部正常。
 *       → 说明面板处于"display off"态：面板不输出 GRAM，所以 LVGL 画什么都是黑，
 *         与画面内容、与 GIF 素材【完全无关】。
 *     · 【整幅反相】画面变成底片负片，同样永久保持。
 *       → INV 位被关。正常态是 bsp_board_lcd_init() 步骤 8 打开的那一次
 *         esp_lcd_panel_invert_color(true)（INVON），即"反相 = 那一位被关掉了"。
 *
 * 【为什么判定是"被写坏"而不是"固件关的"】全工程没有任何运行时路径会发这几条
 *   命令，逐条查证：
 *     · invert_color() 只在初始化调用一次，且参数恒为 true（只发 INVON，不发 INVOFF）
 *     · esp_lcd_panel_disp_on_off(false)（DISPOFF）无任何活跃调用 —— 实现见
 *       bsp_board_lcd_off()/disp_off()，standby.c 里那处已整段删除（见 standby.c 注释）
 *     · SLPIN/SLPOUT 全工程无调用
 *   所以只能是 SPI 传输中命令/数据相位错乱，把某个数据/参数字节当命令执行了：
 *   撞上 0x28/0x10 → 全黑，撞上 0x20 → 反相，撞上 0x11/0x29 → 恰好恢复。
 *   本机 SPI 跑 80MHz（见 io_config.pclk_hz），约为 ST7789 串行写规格的 5.3 倍，
 *   而 [BUG-017] 已记录过「40MHz 余量已不足」。故障集中在"切图那一刻"（一次整屏
 *   153,600 字节的连续突发，出错窗口最大）也与此吻合。
 *
 * 【本函数做什么】把三条命令重发一遍，把面板状态掰回已知正确值。三条都是重设
 *   同一个状态位，幂等，不改变画面内容，无可见跳变。调用点在切图收口，于是
 *   "永久保持"被降级为"最长到下一次切图（约 5~6 秒）就自动恢复"。
 *
 * 【为什么不在本函数里加 delay】ST7789 的 SLPOUT 需要面板内部约 120ms 才真正
 *   醒来，紧接着发的 DISPON 有被丢弃的可能。本函数的调用节拍就是切图节拍
 *   （约 5~6 秒一次），下一次切图会自然补发 —— 用切图节拍代替 delay，避免在
 *   LVGL 线程里插入最长 120ms 的阻塞造成切图卡顿。
 *
 * 【限制·必须知道】面板没有 MISO（见 spi_bus_config.miso_io_num = -1），状态
 *   不可回读：本函数既无法确认面板当前是否真的异常，也无法验证写回是否生效，
 *   只能靠"故障是否消失"来判定。
 *
 * @param bsp_board BSP 实例指针（访问 lcd_io 句柄）
 * @return void（失败只打警告，不做 ESP_ERROR_CHECK：这只是补救，不该让整机挂掉）
 */

/* ── 观测埋点：重断言统计（2026-09-29 加，纯为排查「颜色翻转 / 整片全黑」偶发故障）──
 *
 * 【为什么必须加】常规日志里【没有任何一行】记录"我们往面板重发了状态"这件事，所以
 *   现场看到的翻转/全黑在日志里完全不可见 —— 用户原话："颜色翻转之后还是日志正常"。
 *   于是故障发生了多少次、什么节奏、跟什么时刻对齐，全都没有数据。
 *
 * 【这一版要回答的三个问题】
 *   ① 重断言的真实频率是多少？—— 若恒为每 5~6 秒一次，说明它只是"切图节拍"的副产物，
 *      与故障无关；若间隔忽长忽短、甚至出现"远小于 5 秒"的间隔，说明另有触发源
 *      （唤醒 / 预载 / 以后新增的挂点），故障的时间轴才对得上。
 *   ② 故障到底是"没有被修"还是"修了但看不见"？—— 用户报"翻转了"的时刻，对着日志看
 *      那一秒前后有没有重断言行：有 ⇒ 是补发在起作用（或补发没起作用）；没有 ⇒ 损坏
 *      发生在两次切图之间，且当时没有任何补发动作。
 *   ③ 损坏节奏是否均匀？—— 30 秒汇总里的 最小/最大/均值 一眼看得出是"匀速节拍"还是
 *      "突发成簇"（成簇往往指向电源/EMI/温度这类外部诱因）。
 *
 * 【为什么限流】逐条日志最快 250ms 一条。当前节拍（每 5~6 秒）远低于此，限流不会丢任何
 *   一条现场数据；它只在"有人把补发挂到更高频的调用点"时才生效 —— 那种情况下 flood 本身
 *   就会变成新的时序干扰源，还会把用户依赖的"日志正常"淹没。计数不受限流影响，始终精确。
 *
 * 【可整段删除】本段不参与任何控制流，删掉即恢复原样。 */
#define BSP_LCD_REASSERT_TAG "LCDST"               // 独立 tag：现场可单独筛这一路观测数据，不与 BSP_LCD 混
#define BSP_LCD_REASSERT_LOG_MIN_US (250 * 1000)   // 逐条日志限流下限：250ms
#define BSP_LCD_REASSERT_WIN_US (30 * 1000 * 1000) // 汇总窗口：30 秒

static uint32_t s_reassert_total = 0;       ///< 上电以来累计重断言次数（权威计数，不受限流影响）
static int64_t s_reassert_prev_us = 0;      ///< 上一次重断言时刻（0 = 尚未发生过）
static int64_t s_reassert_win_start_us = 0; ///< 当前汇总窗口起点（0 = 尚未开窗）
static uint32_t s_reassert_win_cnt = 0;     ///< 当前窗口内次数
static int64_t s_reassert_win_sum_us = 0;   ///< 当前窗口内间隔合计（用于算均值）
static int64_t s_reassert_win_min_us = -1;  ///< 当前窗口内最小间隔（-1 = 无样本）
static int64_t s_reassert_win_max_us = 0;   ///< 当前窗口内最大间隔
static int64_t s_reassert_log_last_us = 0;  ///< 上一次打逐条日志的时刻（限流用）

void bsp_board_lcd_reassert_state(bsp_board_t *bsp_board)
{
    if (bsp_board == NULL || bsp_board->lcd_io == NULL)
        return;
    if (!s_lcd_display_on)
        return; // 显示尚未打开（开机渐亮前 / 待机关屏后）：不抢状态

    /* ── 观测埋点：统计 + 打点（纯日志，不参与控制流；删掉即可恢复原样）──────
     * 顺序说明：先"关旧窗口"，再把本次计入新窗口，最后打本次日志。
     * 这样窗口汇总统计的永远是【已经发生完的】那一段，不会把本次算半截。 */
    const int64_t now_us = esp_timer_get_time();
    const int64_t gap_us = (s_reassert_prev_us > 0) ? (now_us - s_reassert_prev_us) : -1;

    /* ① 关掉已满 30 秒的旧窗口：打一行汇总（不影响任何面板动作） */
    if (s_reassert_win_start_us != 0 && now_us - s_reassert_win_start_us >= BSP_LCD_REASSERT_WIN_US)
    {
        const int64_t avg_us = (s_reassert_win_cnt > 0) ? (s_reassert_win_sum_us / s_reassert_win_cnt) : 0;
        const int64_t min_us = (s_reassert_win_min_us >= 0) ? s_reassert_win_min_us : 0;
        ESP_LOGI(BSP_LCD_REASSERT_TAG,
                 "30秒汇总：重断言 %u 次，间隔 均=%lld 最小=%lld 最大=%lld ms（最小≈最大 ⇒ 匀速节拍，悬殊 ⇒ 突发成簇）",
                 (unsigned)s_reassert_win_cnt,
                 (long long)(avg_us / 1000), (long long)(min_us / 1000), (long long)(s_reassert_win_max_us / 1000));
        s_reassert_win_start_us = now_us;
        s_reassert_win_cnt = 0;
        s_reassert_win_sum_us = 0;
        s_reassert_win_min_us = -1;
        s_reassert_win_max_us = 0;
    }

    /* ② 计入本次 */
    if (s_reassert_win_start_us == 0)
    {
        s_reassert_win_start_us = now_us;
        /* 首条日志必须打得出来：把"上次打日志时刻"往前推一个限流周期 */
        s_reassert_log_last_us = now_us - BSP_LCD_REASSERT_LOG_MIN_US;
    }
    s_reassert_prev_us = now_us;
    s_reassert_total++;
    s_reassert_win_cnt++;
    if (gap_us > 0)
    {
        s_reassert_win_sum_us += gap_us;
        if (s_reassert_win_min_us < 0 || gap_us < s_reassert_win_min_us)
            s_reassert_win_min_us = gap_us;
        if (gap_us > s_reassert_win_max_us)
            s_reassert_win_max_us = gap_us;
    }

    /* ③ 本次日志（限流，见上方宏注释） */
    if (now_us - s_reassert_log_last_us >= BSP_LCD_REASSERT_LOG_MIN_US)
    {
        s_reassert_log_last_us = now_us;
        if (gap_us < 0)
            ESP_LOGI(BSP_LCD_REASSERT_TAG, "重断言 #%u（首次）", (unsigned)s_reassert_total);
        else
            ESP_LOGI(BSP_LCD_REASSERT_TAG, "重断言 #%u 距上次=%lld ms",
                     (unsigned)s_reassert_total, (long long)(gap_us / 1000));
    }

    /* 顺序不可换：
     *   1) 0x11 SLPOUT —— 面板若被 SLPIN 睡下，只发 DISPON 是唤不醒的，必须先醒
     *   2) 0x21 INVON  —— 修"整幅反相"
     *   3) 0x29 DISPON —— 修"整片全黑"
     * 三条都走 esp_lcd_panel_io_tx_param 直发（与步骤 7 发 0xC6 同一个口子），
     * 不走 esp_lcd_panel_* 高层接口，避免引入额外副作用。 */
    static const struct
    {
        uint8_t cmd;
        const char *name;
    } seq[] = {
        {0x11, "SLPOUT"},
        {0x21, "INVON"},
        {0x29, "DISPON"},
    };
    for (size_t i = 0; i < sizeof(seq) / sizeof(seq[0]); i++)
    {
        esp_err_t err = esp_lcd_panel_io_tx_param(bsp_board->lcd_io, seq[i].cmd, NULL, 0);
        if (err != ESP_OK)
        {
            // 只警告：这是补救性动作，写失败不该让整机起不来
            ESP_LOGW("BSP_LCD", "重断言面板状态失败(%s/0x%02X): %s",
                     seq[i].name, seq[i].cmd, esp_err_to_name(err));
        }
    }
}
