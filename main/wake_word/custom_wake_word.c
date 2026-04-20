/**
 * @file custom_wake_word.c
 * @brief 自定义唤醒词引擎实现（MultiNet6 + AFE 音频前端）
 *
 * 模块内部结构（私有函数调用顺序）：
 *   wake_word_init()
 *     ├─ nvs_read_str()          从 NVS 读取上次配置
 *     ├─ load_model_for_lang()   加载 MultiNet6 语言模型
 *     ├─ wake_word_load_from_nvs()  读取命令词
 *     ├─ count_words()           校验词数 ≥ 2
 *     ├─ register_command_word() 注册命令词到 FST
 *     └─ afe_fetch_task          启动 AFE 消费任务
 *
 *   afe_fetch_task（FreeRTOS 任务，CPU1）
 *     ├─ s_afe_iface->fetch()    取出降噪后 PCM
 *     ├─ s_vad_cb()              触发 VAD 回调
 *     ├─ s_enhanced_pcm_hook()   送编码器
 *     └─ multinet_iface->detect() 检测命令词 → user_callback
 */
#include "custom_wake_word.h"
#include "bsp/bsp_board.h"
#include "object.h"
#include "esp_heap_caps.h"
#include "freertos/ringbuf.h"
static const char *TAG = "BSP_WakeWord";
// 前向声明 afe_fetch_task，解决编译错误 "implicit declaration of function 'afe_fetch_task'"
static void afe_fetch_task(void *arg);

// ─── 常量定义 ────────────────────────────────────────────────────────────
#define WAKE_COMMAND_ID 1                // 唤醒词在命令词表中的固定 ID（唯一一条命令）
#define NVS_NAMESPACE "sys_config"       // NVS 命名空间（与其他模块共享）
#define NVS_KEY_WAKEWORD "wakeword"      // NVS Key：命令词（拼音或英文）
#define NVS_KEY_DISP_WORD "ww_disp"      // NVS Key：显示文字（用于下次启动判断语言）
#define DEFAULT_DISP_CN "你好"           // 出厂默认中文显示词
#define DEFAULT_WAKEWORD_CN "ni hao "    // 出厂默认中文命令词（拼音）
#define DEFAULT_DISP_EN "Hello Echo"     // 出厂默认英文显示词
#define DEFAULT_WAKEWORD_EN "HELLO ECHO" // 出厂默认英文命令词（mn6_en 词表全大写）
#define AUDIO_BUFFER_MAX 8192            // MultiNet 音频积累缓冲区最大采样点数
// AEC 交织缓冲区大小：最大 feed chunksize（每通道）* 2 通道
// 通常 chunksize = 512，因此此处取保守上限 1024 * 2 = 2048 个 int16_t
#define AEC_MAX_FEED_SAMPLES 1024 // 每通道最大采样点数

// ─── 模块级静态变量 ──────────────────────────────────────────────────────
static esp_mn_iface_t *multinet_iface = NULL;          // MultiNet 接口函数表指针
static model_iface_data_t *multinet_model_data = NULL; // MultiNet 模型运行时数据
static srmodel_list_t *models = NULL;                  // SPIFFS 模型分区扫描结果列表

// ─── AFE 音频前端（AEC 回声消除 + NS 降噪 + VAD）────────────────────────
// 数据流（"MR" 双通道格式）：
//   麦克风(原始PCM)
//   参考信号(play_task当前播放的PCM) → 交织为 [mic,ref,mic,ref,...] → AFE.feed()
//   AFE 内部：AEC(消除回声) → NS(降噪) → VAD → 干净单通道 PCM → fetch()
//     → MultiNet(命令词检测) + enhanced_pcm_hook(→编码器→云端)

static const esp_afe_sr_iface_t *s_afe_iface = NULL; // AFE 接口函数表（只读，库提供）//! 接口是相当于一个函数指针表，存储的是数据的指针地址
static esp_afe_sr_data_t *s_afe_data = NULL;         // AFE 运行时数据（需要 destroy 释放）
static int s_afe_feed_chunksize = 0;                 // AFE feed 每次需要的采样点数（audio_feed_task 按此投喂）
static int s_afe_fetch_chunksize = 0;                // AFE fetch 每次输出的采样点数

static volatile bool is_running = false;             // 引擎运行标志（volatile：可能在中断/任务间读写）// 供 custom_wake_word_feed 和 afe_fetch_task 访问
static wake_word_detected_cb_t user_callback = NULL; // 用户注册的触发回调
static SemaphoreHandle_t buffer_mutex = NULL;        // 保护 input_buffer 的互斥锁

// ─── AEC 参考信号回调 ─────────────────────────────────────────────────────
// session.c 在会话启动时注册，关闭时注销（置 NULL）。
// feed 函数每帧调用此回调获取参考 PCM，与麦克风交织后送 AFE。
static volatile aec_ref_cb_t s_aec_ref_cb = NULL;

// AEC 交织缓冲区（动态分配到 SPIRAM，在 wake_word_init 中 heap_caps_malloc 分配）
// 原为静态数组（2KB + 4KB = 6KB 内部 SRAM BSS），改为指针避免占用宝贵内部 SRAM
static int16_t *s_aec_ref = NULL;         // 参考信号（临时），1024 * 2 = 2KB
static int16_t *s_aec_interleaved = NULL; // MR 交织输入送 AFE，1024 * 2 * 2 = 4KB

// ─── VAD / 增强 PCM 接口回调状态 ────────────────────────────────────────
static vad_state_cb_t s_vad_cb = NULL;                         // VAD 状态回调
static enhanced_pcm_cb_t s_enhanced_pcm_hook = NULL;           // 降噪后 PCM 数据钩子:AFE输出;增强PCM数据回调（供编码器使用）
static volatile vad_state_t s_current_vad_state = VAD_SILENCE; // 当前 VAD 状态

static char current_wake_word[64] = {0}; // 当前生效的命令词（拼音或全大写英文）
static char current_disp_word[64] = {0}; // 当前生效的显示文字（用于语言判断）

static int16_t *input_buffer = NULL; // MultiNet 音频积累缓冲区（跨帧拼接用），SPIRAM 动态分配
static size_t input_buffer_len = 0;  // 缓冲区当前有效采样点数

// ─── MultiNet 检测分离任务 ────────────────────────────────────────────────
// 【关键优化】把 detect() 从 afe_fetch_task 剥离，解决 AFE FEED 溢出：
//   afe_fetch_task（CPU1，优先级 5）：只做 fetch() + VAD + enhanced_pcm_hook，
//                                     将降噪 PCM 非阻塞推入 s_mn_pcm_buf 立即返回。
//   multinet_detect_task（CPU1，优先级 3）：消费 s_mn_pcm_buf，调用 detect()。
//
// 旧设计：fetch() (~20ms) + detect() (~20ms) = 40ms > 帧周期 32ms → FEED 溢出。
// 新设计：fetch() (~20ms) + 非阻塞推队列 (<1ms) = ~21ms < 32ms → FEED 不溢出。
#define MN_PCM_BUF_SAMPLES 16384               // PCM 队列容量（约 256ms @16kHz，8KB SPIRAM）
static RingbufHandle_t s_mn_pcm_buf = NULL;    // afe_fetch_task → multinet_detect_task 的 PCM 管道
static TaskHandle_t s_mn_detect_handle = NULL; // MultiNet 检测任务句柄
static void multinet_detect_task(void *arg);   // 前向声明

// ─── 语言检测 ─────────────────────────────────────────────────────────────
// UTF-8 中文汉字首字节范围：0xE4 ~ 0xE9（覆盖 CJK 统一汉字主区）
// 含该字节则判定为中文，否则视为英文

/**
 * @brief 检测字符串是否包含中文汉字（UTF-8 编码）
 *
 * 通过扫描首字节范围判断：UTF-8 汉字的首字节在 0xE4~0xE9 区间内。
 * 用于自动检测唤醒词语言，无需调用方显式传入语言参数。
 *
 * @param s 待检测的 UTF-8 字符串指针
 * @return true  字符串中含有汉字（判定为中文）
 * @return false 全为 ASCII 字符（判定为英文）
 *
 * @note 调用者：wake_word_init()、wake_word_update()、wake_word_load_from_nvs()
 */
static bool is_chinese_text(const char *s)
{
    const unsigned char *p = (const unsigned char *)s;
    while (*p)
    {
        // 检测 UTF-8 中文三字节序列的首字节
        if (*p >= 0xE4 && *p <= 0xE9)
            return true;
        p++;
    }
    return false; // 全为 ASCII，判定为英文
}

// ─── NVS 工具函数 ─────────────────────────────────────────────────────────

/**
 * @brief 从 NVS 读取字符串键值，失败时填入默认值
 *
 * 以只读模式打开 NVS_NAMESPACE 命名空间，读取指定 key 的字符串值。
 * 若命名空间不存在或 key 不存在，则将 fallback 复制到 dest。
 *
 * @param key      NVS 键名（如 "wakeword"、"ww_disp"）
 * @param dest     目标缓冲区（调用方分配）
 * @param max_len  缓冲区大小（字节，含结束符）
 * @param fallback 读取失败时使用的默认字符串
 * @return void
 *
 * @note 调用者：wake_word_init()、wake_word_load_from_nvs()
 * !键名是公共数据入口，dest是私有的存储数据的
 *
 */
static void nvs_read_str(const char *key, char *dest, size_t max_len, const char *fallback)
{
    nvs_handle_t h;
    // 以只读模式打开命名空间
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK)
    {
        size_t sz = max_len;
        // 读取字符串键值
        if (nvs_get_str(h, key, dest, &sz) == ESP_OK)
        {
            // 读取成功，关闭句柄并返回
            nvs_close(h);
            ESP_LOGI(TAG, "NVS [%s]: %s", key, dest);
            return;
        }
        // 键不存在或读取出错，关闭句柄
        nvs_close(h);
    }
    // 回退：使用默认值并保证字符串有结束符
    strncpy(dest, fallback, max_len - 1);
    dest[max_len - 1] = '\0';
    ESP_LOGI(TAG, "NVS [%s] 未找到，使用默认: %s", key, dest);
}

/**
 * @brief 向 NVS 写入字符串键值（覆盖已有值，持久化到 Flash）
 *
 * 以读写模式打开 NVS_NAMESPACE 命名空间，写入后调用 nvs_commit() 确保落盘。
 * 若命名空间打开失败（如 NVS 未初始化），直接返回，不崩溃。
 *
 * @param key   NVS 键名（如 "wakeword"、"ww_disp"）
 * @param value 待写入的字符串值
 * @return void
 *
 * @note 调用者：wake_word_update()（更新唤醒词时持久化）
 */
static void nvs_write_str(const char *key, const char *value)
{
    nvs_handle_t h;
    // 以读写模式打开命名空间（失败时直接返回，不崩溃）
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    // 写入字符串键值
    nvs_set_str(h, key, value);
    // 提交写入（持久化到 Flash，掉电不丢失）
    nvs_commit(h);
    // 关闭句柄，释放 NVS 内部资源
    nvs_close(h);
}

// 公开接口：根据当前显示词语言，从 NVS 读取对应命令词到 dest
void wake_word_load_from_nvs(char *dest, size_t max_len)
{
    // 根据当前已记录的显示词判断语言，选择对应的回退默认值
    bool cn = is_chinese_text(current_disp_word);
    nvs_read_str(NVS_KEY_WAKEWORD, dest, max_len,
                 cn ? DEFAULT_WAKEWORD_CN : DEFAULT_WAKEWORD_EN);
}

// ─── 词数统计 ─────────────────────────────────────────────────────────────
// 按空格分隔统计词/音节数量
// 规则：中文拼音"yun yan"=2，英文"hello echo"=2，单音节"yun"=1（不允许）

/**
 * @brief 统计字符串中的词（音节）数量，按空格分隔
 *
 * MultiNet6 要求命令词至少包含 2 个音节/单词，否则 esp_mn_commands_update()
 * 在构建 FST 时会内部崩溃。此函数用于初始化和更新时的前置安全校验。
 *
 * @param s 输入字符串（如 "yun yan"、"HELLO ECHO"）
 * @return int 词数量（空字符串或 NULL 返回 0）
 *
 * @note 调用者：wake_word_init()（校验 NVS 词）、wake_word_update()（校验新词）
 */
static int count_words(const char *s)
{
    if (!s || !*s)
        return 0;
    int count = 0;
    bool in_word = false;
    while (*s)
    {
        if (*s == ' ')
        {
            // 遇到空格，退出当前词
            in_word = false;
        }
        else if (!in_word)
        {
            // 非空格且未在词中，开始一个新词
            in_word = true;
            count++;
        }
        s++;
    }
    return count;
}

// ─── 语言模型加载 ─────────────────────────────────────────────────────────
// 在持锁状态下调用；切换语言时销毁旧模型并创建新模型

/**
 * @brief 加载指定语言的 MultiNet6 模型（销毁旧模型后创建新模型）
 *
 * 从 SPIFFS models 列表中筛选目标语言的 MultiNet 模型，获取接口句柄后
 * 销毁旧模型（若存在）并创建新模型实例。同时根据语言设置检测阈值：
 *   - 中文（mn6_cn）：0.6（prob 分布高，0.6 足以过滤噪声）
 *   - 英文（mn6_en）：0.4（BPE 路径长，prob 天然偏低，0.4 才能正常触发）
 *
 * @param lang 目标语言字符串（ESP_MN_CHINESE 或 ESP_MN_ENGLISH）
 * @return ESP_OK    模型加载成功
 * @return ESP_FAIL  模型未找到或内存不足
 *
 * @note 调用者：wake_word_init()（初始化时）、wake_word_update()（语言切换时）
 * @note 调用前必须持有 buffer_mutex，防止与 afe_fetch_task 并发访问模型
 */
static esp_err_t load_model_for_lang(const char *lang)
{
    // 在 SPIFFS 模型列表中筛选出目标语言的 MultiNet 模型名称
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, lang);
    if (mn_name == NULL)
    {
        ESP_LOGE(TAG, "未找到 '%s' 语言模型", lang);
        return ESP_FAIL;
    }

    // 根据模型名称获取对应的接口函数表（iface）
    esp_mn_iface_t *new_iface = (esp_mn_iface_t *)esp_mn_handle_from_name(mn_name);
    if (new_iface == NULL)
    {
        ESP_LOGE(TAG, "获取 '%s' MultiNet 句柄失败", lang);
        return ESP_FAIL;
    }

    // 如果当前已有模型在运行，先销毁旧模型释放内存（mn6 约 3MB SPIRAM）
    if (multinet_model_data != NULL)
    {
        multinet_iface->destroy(multinet_model_data);
        multinet_model_data = NULL;
    }

    // 切换到新接口
    multinet_iface = new_iface;

    // 创建新模型实例，检测窗口 3000ms（3 秒内未说完则超时重置）
    multinet_model_data = multinet_iface->create(mn_name, 3000);
    if (multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "创建 '%s' MultiNet 模型失败", lang);
        multinet_iface = NULL;
        return ESP_FAIL;
    }

    // 设置语言差异阈值：
    //   中文(cn)：0.18（正常音量 prob 集中 0.25~0.37，0.18 保留召回余量；
    //             原 0.12 过松，TTS 残留经 AEC 后的 prob≈0.22 会误触发自激）
    //   英文(en)：0.4（BPE 路径长，prob 天然偏低，0.4 才能正常触发）
    float threshold = (strcmp(lang, ESP_MN_ENGLISH) == 0) ? 0.4f : 0.18f;
    multinet_iface->set_det_threshold(multinet_model_data, threshold);

    ESP_LOGW(TAG, "已加载语言模型: %s", mn_name);
    return ESP_OK;
}

// ─── 命令词注册 ───────────────────────────────────────────────────────────

/**
 * !FST是：命令词识别模型，用于识别用户输入的命令词。
 * !SR是：语音识别模型，用于识别用户输入的语音内容。
 * @brief 向当前 MultiNet6 模型注册命令词并重建 FST
 * 模型创建，注册，更新
 * 步骤：
 *   1. alloc 命令词槽（清除旧词表）
 *   2. 尝试注册 current_wake_word，失败则回退到对应语言的出厂默认词
 *   3. 调用 esp_mn_commands_update() 触发 FST 编译
 *   4. 打印错误词列表（调试）
 *
 * @return ESP_OK（始终成功，失败词会回退到默认）
 *
 * @note 调用者：wake_word_init()（初始化末尾）、wake_word_update()（更新时）
 * @note 调用前需已持有 buffer_mutex 并加载了有效的 multinet_model_data
 */
static esp_err_t register_command_word(void)
{
    // 分配命令词槽位（内部会清除之前注册的所有命令词）
    esp_mn_commands_alloc(multinet_iface, multinet_model_data);

    // 尝试注册当前命令词（格式要求：中文拼音空格分隔 / 英文大写空格分隔）
    esp_err_t err = esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
    if (err != ESP_OK)
    {
        // 命令词格式不符合模型要求，回退到出厂默认词
        const char *def = is_chinese_text(current_disp_word)
                              ? DEFAULT_WAKEWORD_CN
                              : DEFAULT_WAKEWORD_EN;
        ESP_LOGW(TAG, "命令词 [%s] 格式不支持，回退为默认词 [%s]", current_wake_word, def);
        strncpy(current_wake_word, def, sizeof(current_wake_word) - 1);
        // 使用默认词再次注册
        esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
    }

    // 触发 FST（有限状态转换器）重建，将命令词编译进模型
    // 返回值为失败词列表（注意：返回类型是 esp_mn_error_t*，非 esp_err_t）
    esp_mn_error_t *mn_err = esp_mn_commands_update();
    if (mn_err != NULL && mn_err->num > 0)
    {
        // 打印每个注册失败的词（通常是音节超出词表范围）
        ESP_LOGE(TAG, "命令词注册有 %d 个失败:", mn_err->num);
        for (int i = 0; i < mn_err->num; i++)
            ESP_LOGE(TAG, "  ✗ %s", mn_err->phrases[i]->string);
    }

    // 打印当前已注册的命令词列表（调试用）
    esp_mn_commands_print();
    return ESP_OK;
}

// ─── 公开 API：初始化 ────────────────────────────────────────────────────

/**
 * @brief 初始化唤醒词引擎（AFE + MultiNet6），从 NVS 恢复上次保存的配置
 *
 * 完整初始化步骤（见 .h 文件 @note 顺序）：
 *   1. 保存用户回调 + 创建 buffer_mutex 互斥锁
 *   2. 扫描 SPIFFS "model" 分区，建立 srmodel_list_t
 *   3. 从 NVS 读取 current_disp_word（上次显示词），判断语言
 *   4. 初始化 AFE 配置（单麦 M，开启 NS+VAD，关闭 WakeNet/AEC/SE/AGC）
 *   5. 创建 AFE 实例，缓存 feed/fetch chunksize
 *   6. 调用 load_model_for_lang() 加载对应 MultiNet6 模型
 *   7. 从 NVS 加载命令词，校验词数 ≥ 2，调用 register_command_word()
 *   8. 置 is_running=true，启动 afe_fetch_task（CPU1，优先级 5）
 *
 * @param cb 唤醒词触发回调（检测到命令词时从 afe_fetch_task 调用）
 * @return ESP_OK    初始化成功
 * @return ESP_FAIL  模型分区未找到或 AFE/MultiNet 内存不足
 *
 * @note 调用者：application.c → application_init()（步骤 3）
 * @note 前置条件：NVS 已初始化（bsp_board_nvs_init 已调用）
 */
esp_err_t wake_word_init(wake_word_detected_cb_t cb)
{
    PRINT_MEM_INFO(TAG, "唤醒词初始化前");
    // 保存用户回调，唤醒词触发时调用
    user_callback = cb;

    // 创建互斥锁，用于保护 input_buffer 在投喂任务与更新任务间的并发访问
    buffer_mutex = xSemaphoreCreateMutex();
    if (buffer_mutex == NULL)
    {
        ESP_LOGE(TAG, "互斥锁创建失败");
        return ESP_FAIL;
    }

    // 清零缓冲区长度（防止旧数据干扰）
    input_buffer_len = 0;

    // 从 SPIRAM 分配三个音频缓冲区（原为静态数组，合计 ~10KB 内部 SRAM）
    // s_aec_ref + s_aec_interleaved + input_buffer = 2KB + 4KB + 4KB
    s_aec_ref = heap_caps_malloc(AEC_MAX_FEED_SAMPLES * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    s_aec_interleaved = heap_caps_malloc(AEC_MAX_FEED_SAMPLES * 2 * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    input_buffer = heap_caps_malloc(AUDIO_BUFFER_MAX * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (!s_aec_ref || !s_aec_interleaved || !input_buffer)
    {
        ESP_LOGE(TAG, "音频缓冲区分配失败（SPIRAM 不足）");
        free(s_aec_ref);
        free(s_aec_interleaved);
        free(input_buffer);
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 扫描 SPIFFS "model" 分区，建立模型文件列表
    models = esp_srmodel_init("model");
    //=-1 是 esp_srmodel_init 在分区未挂载或缺失时的错误标志
    if (models == NULL || models->num == -1)
    {
        // 分区未挂载或 partitions.csv 中 model 分区缺失
        ESP_LOGE(TAG, "模型分区初始化失败，请检查 partitions.csv");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL; //! 锁的顺序不能错，先删除锁再置 NULL
        return ESP_FAIL;
    }

    // 从 NVS 读取上次保存的显示词，用于推断上次使用的语言
    nvs_read_str(NVS_KEY_DISP_WORD, current_disp_word,
                 sizeof(current_disp_word), DEFAULT_DISP_CN);

    // 根据显示词语言选择并加载对应 MultiNet6 模型
    const char *lang = is_chinese_text(current_disp_word) ? ESP_MN_CHINESE : ESP_MN_ENGLISH;

    // ── AFE 初始化：AEC(回声消除) + NS(降噪) + VAD，单麦+参考信号 ──────
    // 输入格式 "MR"：M = 1路麦克风，R = 1路参考信号（当前扬声器播放的PCM）。
    // play_task 写 I2S 时同步推副本到 aec_ref_buf，custom_wake_word_feed
    // 交织 [mic,ref,...] 后送入 AFE，AFE AEC 消除 TTS 回声，
    // 使 MultiNet 能在大模型说话时检测到唤醒词（支持打断）。
    afe_config_t *afe_cfg = afe_config_init("MR", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (afe_cfg == NULL)
    {
        ESP_LOGE(TAG, "AFE 配置创建失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 精确控制各子模块开关
    afe_cfg->wakenet_init = false;             // 不用 WakeNet，MultiNet 做唤醒词
    afe_cfg->aec_init = true;                  // ★ 开启 AEC 回声消除（需要 "MR" 格式参考信号）
    afe_cfg->aec_mode = AEC_MODE_SR_LOW_COST;  // 低功耗 AEC（SR 场景推荐）
    afe_cfg->se_init = false;                  // 单麦无需 BSS/MASE 多麦阵列处理
    afe_cfg->ns_init = true;                   // ★ 开启 NS 噪声抑制（核心功能）
    afe_cfg->afe_ns_mode = AFE_NS_MODE_WEBRTC; // WebRTC NS 模式，兼顾降噪效果和语音质量
    // ★ 开启 AGC（自动增益控制）：AFE_AGC_MODE_WAKENET 专为唤醒词优化。
    // 问题根因：WebRTC NS 会同时抑制噪声和安静的语音，导致 MultiNet 置信度极低（必须大喊才识别）。
    // WAKENET AGC 在 NS 输出后自适应补偿增益：放大安静人声，同时限制底噪放大幅度。
    // 效果：普通音量说"你好小熊"的置信度提升，阈值 0.3 可覆盖更多正常发音。
    afe_cfg->agc_init = true;
    afe_cfg->agc_mode = AFE_AGC_MODE_WAKENET;                 // WAKENET AGC 模式，专为唤醒词场景设计，配合 WebRTC NS 使用
    afe_cfg->vad_init = true;                                 //! ★ 开启 VAD 语音活动检测（核心功能）
    afe_cfg->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM; // 尽量用 PSRAM 省内部 SRAM
    afe_cfg->vad_mode = VAD_MODE_1;                           // VAD 模式 1：适合唤醒词场景，快速响应，适度误触；

    // 获取 AFE 接口并创建实例
    s_afe_iface = esp_afe_handle_from_config(afe_cfg);
    if (s_afe_iface == NULL)
    {
        ESP_LOGE(TAG, "AFE 接口获取失败");
        afe_config_free(afe_cfg);
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 创建数据，复制到 s_afe_data ，销毁 afe_cfg 释放内存
    s_afe_data = s_afe_iface->create_from_config(afe_cfg);
    PRINT_MEM_INFO(TAG, "AFE模型加载后");
    afe_config_free(afe_cfg); // 配置已复制到 AFE 内部，可以释放
    afe_cfg = NULL;
    if (s_afe_data == NULL)
    {
        ESP_LOGE(TAG, "AFE 实例创建失败（内存不足？）");
        s_afe_iface = NULL;
        /**
        // !为什么置为空？因为后续代码会检查 s_afe_iface 是否为 NULL
        来判断是否初始化成功，如果不置空可能会误以为 AFE 初始化成功了，
         从而导致程序无法运行
         */
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 缓存 AFE 的帧大小，供 audio_feed_task 和 fetch 逻辑使用
    /**
     * 统一的 chunksize，系统可以确保音频数据流的同步性和处理效率
     * 避免缓冲区溢出或欠载的问题。
     */
    s_afe_feed_chunksize = s_afe_iface->get_feed_chunksize(s_afe_data);
    s_afe_fetch_chunksize = s_afe_iface->get_fetch_chunksize(s_afe_data);
    ESP_LOGI(TAG, "AFE 初始化完成: feed_chunk=%d, fetch_chunk=%d",
             s_afe_feed_chunksize, s_afe_fetch_chunksize);

    // 打印 AFE 处理管线（调试用，类似 [input] → |NS| → |VAD| → [output]）
    s_afe_iface->print_pipeline(s_afe_data);

    // ★ 关键：提前创建 PCM 队列并启动 afe_fetch_task，防止模型加载期间 FEED 溢出
    //
    // 问题根源：audio_feed_task 在 audio_init() 时就已启动，一直在调用 feed()。
    //           MultiNet 模型加载 + FST 编译耗时 ~2s，这段时间内 afe_fetch_task 未运行，
    //           AFE FEED ringbuffer 积满 → 连续打印 "Ringbuffer of AFE(FEED) is full"。
    //
    // 解决：AFE 创建完成后立即启动 afe_fetch_task，让 fetch() 持续消费 FEED buffer。
    //       此时 is_running=false 且 multinet_model_data=NULL，fetch 任务只调 fetch()
    //       和 taskYIELD()，不推 PCM 队列，不做 detect。模型加载完全不受影响。
    s_mn_pcm_buf = xRingbufferCreateWithCaps(MN_PCM_BUF_SAMPLES * sizeof(int16_t),
                                             RINGBUF_TYPE_BYTEBUF,
                                             MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    if (s_mn_pcm_buf == NULL)
    {
        ESP_LOGE(TAG, "MultiNet PCM 队列创建失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // afe_fetch_task：高优先级，持续消费 AFE FETCH buffer（也防止 FEED 积压）
    // is_running=false 时只调 fetch()+yield，不做 detect 和队列推送，模型加载安全
    xTaskCreatePinnedToCoreWithCaps(afe_fetch_task, "afe_fetch",
                                    4096, NULL, 5, NULL, 1,
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    ESP_LOGI(TAG, "afe_fetch_task 提前启动，AFE FEED 将不再溢出");

    if (load_model_for_lang(lang) != ESP_OK)
    {
        ESP_LOGE(TAG, "MultiNet唤醒词模型加载失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }
    PRINT_MEM_INFO(TAG, "MultiNet唤醒词模型加载后");

    //! 从 NVS 加载命令词；英文词自动转全大写（mn6_en 词表要求）
    wake_word_load_from_nvs(current_wake_word, sizeof(current_wake_word));
    if (!is_chinese_text(current_disp_word))
    {
        for (int i = 0; current_wake_word[i]; i++)
            current_wake_word[i] = toupper((unsigned char)current_wake_word[i]);
        // toupper() 函数将小写字母转换为大写字母，如果字符不是小写字母，则返回原字符不变。
    }

    // 安全校验：词数 < 2 说明 NVS 中存的是旧版单音节词，强制回退默认值
    // 原因：esp_mn_commands_update() 在单音节输入时会内部崩溃
    if (count_words(current_wake_word) < 2)
    {
        const char *def = is_chinese_text(current_disp_word)
                              ? DEFAULT_WAKEWORD_CN
                              : DEFAULT_WAKEWORD_EN;
        ESP_LOGW(TAG, "NVS 词 [%s] 太短，重置为默认: %s", current_wake_word, def);
        strncpy(current_wake_word, def, sizeof(current_wake_word) - 1);
        strncpy(current_disp_word,
                is_chinese_text(current_disp_word) ? DEFAULT_DISP_CN : DEFAULT_DISP_EN,
                sizeof(current_disp_word) - 1);
    }

    // 向模型注册命令词，编译 FST
    register_command_word();

    ESP_LOGW(TAG, "唤醒词引擎初始化完成. 显示=%s 命令词=%s",
             current_disp_word, current_wake_word);

    // 允许 custom_wake_word_feed 开始处理音频帧，同时解锁 multinet_detect_task 的检测逻辑
    is_running = true;

    // 启动 MultiNet 检测任务（低优先级：detect 慢但不阻塞 AFE 链路）
    // 此时模型和 FST 均已就绪，detect_task 可立即开始工作
    xTaskCreatePinnedToCoreWithCaps(multinet_detect_task, "mn_detect",
                                    4096, // 栈大小（detect 有一定调用深度）
                                    NULL,
                                    4, // 优先级 3：低于 afe_fetch(5)，detect 慢时让路
                                    &s_mn_detect_handle,
                                    1, // 同 CPU1，共享缓存
                                    MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);

    return ESP_OK;
}

// ─── 公开 API：唤醒词是否与当前一致（用于 MQTT 跳过无意义更新）──────────

/**
 * @brief 判断传入的显示词/命令词与当前生效的唤醒词是否完全相同
 *
 * 实现要点：
 *   - display 原样字符串比较
 *   - pinyin：中文原样比较；英文临时转大写后再比较（与 wake_word_update 对齐）
 *   - 引擎未初始化（current_wake_word 为空）直接返回 false，强制走更新流程
 *
 * @param display 新的显示词（非 NULL）
 * @param pinyin  新的命令词（非 NULL）
 * @return true   完全一致，调用方应跳过 wake_word_update
 * @return false  任一字段不同或输入非法
 *
 * @note 调用者：mqtt_protocol.c（MQTT 上线 retained 消息处理）
 */
bool wake_word_is_same(const char *display, const char *pinyin)
{
    // 非法入参直接返回 false，交由更新路径的校验逻辑处理
    if (display == NULL || pinyin == NULL)
        return false;
    // 引擎未初始化（尚未读取过 NVS），无法比较，直接返回 false 让 update 走完整流程
    if (current_disp_word[0] == '\0' || current_wake_word[0] == '\0')
        return false;

    // 显示词必须完全相同
    if (strcmp(display, current_disp_word) != 0)
        return false;

    // 根据语言决定命令词的比较方式：
    //   中文：原样比较（current_wake_word 存的就是小写拼音）
    //   英文：wake_word_update 会把新词转大写存入 current_wake_word，
    //         因此这里也把传入的 pinyin 临时转大写再比较，避免大小写差异导致误判"不同"
    bool new_is_cn = is_chinese_text(display);
    if (new_is_cn)
    {
        return strcmp(pinyin, current_wake_word) == 0;
    }
    else
    {
        // 英文走大写比较（逐字符大写比对，无需额外分配缓冲区）
        const unsigned char *a = (const unsigned char *)pinyin;
        const unsigned char *b = (const unsigned char *)current_wake_word;
        while (*a && *b)
        {
            if (toupper(*a) != *b)
                return false;
            a++;
            b++;
        }
        // 两字符串必须同时到达末尾，否则长度不一致
        return (*a == '\0' && *b == '\0');
    }
}

// ─── 公开 API：运行时更新唤醒词 ─────────────────────────────────────────

/**
 * @brief 更新唤醒词（MQTT 收到指令后调用，支持中英文热切换）
 *
 * 执行步骤（持锁保护全程）：
 *   1. 前置校验：引擎已初始化 + 词数 ≥ 2
 *   2. 持 buffer_mutex 锁，停止 feed 循环（is_running=false）
 *   3. 若语言发生变化（中↔英），调用 load_model_for_lang() 切换模型
 *   4. 更新 current_disp_word + current_wake_word（英文自动转大写）
 *   5. 调用 register_command_word() 重建 FST
 *   6. 调用 multinet_iface->clean() 清除历史状态
 *   7. 持久化到 NVS（下次上电恢复）
 *   8. 恢复 is_running=true，释放锁
 *
 * @param wake_word_display  显示文字（如 "云炎" 或 "Hello Echo"），用于语言自动检测
 * @param wake_word_pinyin   命令词（中文拼音 "yun yan" / 英文单词 "hello echo"）
 * @return ESP_OK              更新成功
 * @return ESP_FAIL            引擎未初始化或语言切换失败
 * @return ESP_ERR_INVALID_ARG 命令词词数 < 2
 *
 * @note 调用者：mqtt_protocol.c → async_update_wakeword_task()
 */
esp_err_t wake_word_update(const char *wake_word_display, const char *wake_word_pinyin)
{
    // 前置检查：引擎必须已初始化（模型句柄非空）
    if (multinet_iface == NULL || multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "引擎未初始化，无法更新");
        return ESP_FAIL;
    }

    // 前置校验：命令词词数 ≥ 2，否则 FST 构建时会崩溃
    // 中文举例：「云烟」→ "yun yan"（2音节 ✓）；「云」→ "yun"（1音节 ✗）
    if (count_words(wake_word_pinyin) < 2)
    {
        ESP_LOGW(TAG, "唤醒词太短！[%s] 必须包含至少 2 个音节/单词", wake_word_pinyin);
        return ESP_ERR_INVALID_ARG;
    }

    // 持锁：更新期间暂停音频投喂，防止与 feed 任务并发访问模型
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    is_running = false;   // 停止 feed 循环内的检测
    input_buffer_len = 0; // 清空积累的音频帧，避免旧数据干扰新模型

    ESP_LOGI(TAG, "更新唤醒词: display=[%s] pinyin=[%s]",
             wake_word_display, wake_word_pinyin);

    // 判断新词目标语言（从 display 字段检测，无需用户指定 lang 参数）
    bool new_is_cn = is_chinese_text(wake_word_display);
    const char *new_lang = new_is_cn ? ESP_MN_CHINESE : ESP_MN_ENGLISH;

    // 判断当前已加载的模型语言
    bool cur_is_cn = is_chinese_text(current_disp_word);
    const char *cur_lang = cur_is_cn ? ESP_MN_CHINESE : ESP_MN_ENGLISH;

    // 若语言发生变化，销毁旧模型并加载新语言模型
    if (new_is_cn != cur_is_cn)
    {
        ESP_LOGI(TAG, "语言切换: %s → %s", cur_lang, new_lang);
        if (load_model_for_lang(new_lang) != ESP_OK)
        {
            // 加载失败，恢复运行状态并释放锁，保持旧模型继续工作
            ESP_LOGE(TAG, "语言切换失败，保持原模型");
            is_running = true;
            xSemaphoreGive(buffer_mutex);
            return ESP_FAIL;
        }
    }

    // 更新显示词（保证尾部有结束符，防止越界读取）
    strncpy(current_disp_word, wake_word_display, sizeof(current_disp_word) - 1);
    current_disp_word[sizeof(current_disp_word) - 1] = '\0';

    // 更新命令词；英文自动转全大写（mn6_en 词表要求大写）
    strncpy(current_wake_word, wake_word_pinyin, sizeof(current_wake_word) - 1);
    current_wake_word[sizeof(current_wake_word) - 1] = '\0';
    if (!new_is_cn)
    {
        for (int i = 0; current_wake_word[i]; i++)
            current_wake_word[i] = toupper((unsigned char)current_wake_word[i]);
        ESP_LOGI(TAG, "英文词自动转大写: %s", current_wake_word);
    }

    // 重新注册命令词到当前模型（覆盖旧词）
    register_command_word();

    // 清除模型内部的音频历史状态，从干净状态开始检测
    multinet_iface->clean(multinet_model_data);

    // ★ NVS 写在锁内（is_running=false）、释放锁之前执行。
    // 原因：flash 写操作会临时禁用 Data Cache，若此时 multinet_detect_task（SPIRAM栈）
    // 正在运行，SPIRAM 不可访问 → cache_utils.c assert 崩溃重启。
    // 将 NVS 写提前到锁内，multinet_detect_task 因等锁而阻塞，不访问 SPIRAM，安全。
    // ★ 调用者 async_update_wakeword_task 必须使用内部 SRAM 栈（MALLOC_CAP_INTERNAL），
    //   否则 flash 操作禁用 Data Cache 时本任务自身栈也不可访问 → 崩溃。
    nvs_write_str(NVS_KEY_WAKEWORD, current_wake_word);
    nvs_write_str(NVS_KEY_DISP_WORD, current_disp_word);

    // 🟢 关键：在恢复运行前，彻底重置底层状态，防止旧数据导致死锁或误触发
    if (s_afe_iface && s_afe_data)
    {
        s_afe_iface->reset_buffer(s_afe_data); // 重置 AFE 内部 Ringbuffer
    }

    // 🟢 关键：清空中间 PCM 队列中残留的旧数据
    if (s_mn_pcm_buf != NULL)
    {
        size_t sz;
        void *item;
        while ((item = xRingbufferReceive(s_mn_pcm_buf, &sz, 0)) != NULL)
        {
            vRingbufferReturnItem(s_mn_pcm_buf, item);
        }
    }
    // NVS 写完成后再恢复运行并释放锁，确保 multinet_detect_task 重新运行时 flash 已空闲
    is_running = true;
    input_buffer_len = 0;
    xSemaphoreGive(buffer_mutex);

    ESP_LOGW(TAG, "唤醒词更新成功！display=%s command=%s",
             current_disp_word, current_wake_word);

    return ESP_OK;
}

// ─── 公开 API：获取每帧采样点数 ─────────────────────────────────────────

size_t custom_wake_word_get_chunksize(void)
{
    // 向模型查询每次 detect() 需要消耗的采样点数（mn6 通常为 512）
    if (multinet_iface && multinet_model_data)
        return multinet_iface->get_samp_chunksize(multinet_model_data);
    return 0; // 模型未就绪时返回 0，调用方使用安全默认值
}

// ─── AFE fetch 任务（独立任务，与 feed 并行运行）─────────────────────
// 数据流：AFE fetch(降噪+VAD) → 降噪PCM → MultiNet detect
//  → enhanced_pcm_hook → 编码器 → 云端
//! 【关键设计】AFE 要求 feed 和 fetch 在不同任务中"并行执行"：
// !  若 feed 和 fetch 在同一任务中"串行"，fetch 阻塞时 feed 停止，ringbuffer 永远空。
//   - audio_feed_task（bsp_codec.c）：持续调 s_afe_iface->feed() 投喂原始 PCM
//   - afe_fetch_task（本函数）：持续调 s_afe_iface->fetch() 取出降噪后 PCM
/**
 * @brief AFE 降噪输出消费任务（持续从 AFE 取帧，分发给 VAD/编码器/MultiNet）
 * 数据流（每帧循环）：
 *   s_afe_iface->fetch()  →  更新 s_current_vad_state，触发 s_vad_cb
 *                         →  调用 s_enhanced_pcm_hook（→ 编码器 → 云端）
 *                         →  积累到 input_buffer，凑满 mn_chunksize 后送 detect
 *                         →  detect 命中 → 置 wake_triggered，锁外调 user_callback
 * @param arg FreeRTOS 任务参数（未使用，传 NULL）
 * @return void（任务不返回，永久循环直到设备复位）
 * @note 调用者：wake_word_init()（xTaskCreatePinnedToCore，CPU1，优先级 5）
 * @note 必须与 audio_feed_task 在不同任务中并行运行，否则 AFE ringbuffer 饿死
 */
static void afe_fetch_task(void *arg)
{
    ESP_LOGI(TAG, "AFE fetch 任务启动");
    /**
     * 风险：
如果你的 CPU1 音频任务真的因为 Bug 死循环了，由于 WDT 关了，CPU1 就真的永久卡死了，
只能靠 CPU0 的重启逻辑或硬件复位来恢复。
通常在音视频产品中，更规范的做法不是关 IDLE WDT，而是：
在长时间循环中主动插入 vTaskDelay(1) 喂狗。
或者把耗时处理切碎，让 IDLE 有机会运行。
     */
    // !注意：CPU1 的 IDLE 任务 WDT 监控已在 sdkconfig 中关闭
    // （CONFIG_ESP_TASK_WDT_CHECK_IDLE_TASK_CPU1 = n）
    // 原因：afe_fetch + audio_feed 都在 CPU1 优先级 5 运行，设计上占满 CPU1，
    // IDLE1 无法运行是正常行为，不是故障。
    while (1)
    {
        // 阻塞等待 AFE 输出一帧降噪后的 PCM（内部自动同步 feed 速率）
        afe_fetch_result_t *res = s_afe_iface->fetch(s_afe_data);
        // 如果ress 为空或者失败，则跳过本帧数据，继续等待下一帧
        if (res == NULL || res->ret_value == ESP_FAIL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // ── 更新 VAD 状态 + 通知回调 ────────────────────────────────────
        s_current_vad_state = res->vad_state;
        if (s_vad_cb)
            s_vad_cb(res->vad_state);

        // ── 降噪 PCM 送给增强钩子（→ 编码器 → 云端）──────────────────
        // 无论 MultiNet 是否在运行，钩子都需要收到数据
        /**
         * res->data: AFE 降噪后的 16-bit PCM 音频数据
         * res->data_size / sizeof(int16_t): 计算有多少个采样点（因为每个采样点是2字节）
         * if (s_enhanced_pcm_hook): 检查是否有模块注册了这个回调
         * 调用回调: 将处理好的音频数据传递给注册的模块
         */
        if (s_enhanced_pcm_hook)
            s_enhanced_pcm_hook(res->data, res->data_size / sizeof(int16_t));

        // ── 降噪 PCM 推入 MultiNet 队列（非阻塞，<1ms）────────────────────
        // detect() 已迁移到 multinet_detect_task（优先级 3）异步执行，
        // afe_fetch_task（优先级 5）不再因 detect 卡住，可以及时消费 AFE FETCH buffer。
        // 队列满时直接丢弃：detect 暂时跟不上不影响 AFE 链路，只轻微影响识别率。
        if (is_running && multinet_model_data != NULL && s_mn_pcm_buf != NULL)
        {
            // 0 超时 = 非阻塞，绝不阻塞 fetch 循环
            xRingbufferSend(s_mn_pcm_buf, res->data, res->data_size, 0);
        }
        else
        {
            taskYIELD(); // 未运行时让 audio_feed 等同优先级任务运行
        }
    }
    // 打印任务历史最小剩余栈空间（字节）
    ESP_LOGD("STACK", "%s 栈剩余: %lu 字节", pcTaskGetName(NULL), (unsigned long)uxTaskGetStackHighWaterMark(NULL));
}

// ─── MultiNet 检测任务（从 PCM 队列消费，异步执行 detect）──────────────
/**
 * @brief MultiNet 命令词检测任务（解耦版）
 *
 * 从 s_mn_pcm_buf 读取降噪 PCM，积累到 mn_chunksize 后调用 detect()。
 * 与 afe_fetch_task 解耦，detect 慢时不再阻塞 AFE 链路。
 *
 * 数据流：
 *   afe_fetch_task → (s_mn_pcm_buf ringbuf) → multinet_detect_task
 *                                               → multinet detect()
 *                                               → user_callback（唤醒词触发）
 *
 * @param arg FreeRTOS 任务参数（未使用）
 * @note 优先级 4（低于 afe_fetch 的 5），detect 慢时自动让路给 AFE 链路
 * @note 绑定 CPU1，与 afe_fetch_task 共享 L1 缓存，减少 SPIRAM 访问延迟
 */
static void multinet_detect_task(void *arg)
{
    ESP_LOGI(TAG, "MultiNet 检测任务启动（优先级 4，CPU1）");

    while (1)
    {
        // 1. 动态计算等待时间：如果池子里有货，就不排队，立刻处理下一帧
        int mn_chunksize = (multinet_iface && multinet_model_data) ? multinet_iface->get_samp_chunksize(multinet_model_data) : 512;

        TickType_t wait_time = (input_buffer_len >= (size_t)mn_chunksize) ? 0 : pdMS_TO_TICKS(100);

        size_t recv_size = 0;
        void *item = xRingbufferReceive(s_mn_pcm_buf, &recv_size, wait_time);

        if (item != NULL)
        {
            if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
            {
                size_t enhanced_samples = recv_size / sizeof(int16_t);
                if (input_buffer_len + enhanced_samples <= AUDIO_BUFFER_MAX)
                {
                    memcpy(&input_buffer[input_buffer_len], item, recv_size);
                    input_buffer_len += enhanced_samples;
                }
                else
                {
                    ESP_LOGW(TAG, "MultiNet buffer overflow，清空重来");
                    input_buffer_len = 0;
                }
                xSemaphoreGive(buffer_mutex);
            }
            vRingbufferReturnItem(s_mn_pcm_buf, item);
        }

        if (!is_running || multinet_model_data == NULL)
            continue;

        // 2. 执行检测（if 配合外层循环实现了 while 的效果，但每次都会放锁，不卡系统）
        if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            bool wake_triggered = false;
            if (input_buffer_len >= (size_t)mn_chunksize && is_running)
            {
                esp_mn_state_t mn_state = multinet_iface->detect(multinet_model_data, input_buffer);

                if (mn_state == ESP_MN_STATE_DETECTED)
                {
                    esp_mn_results_t *mn_result = multinet_iface->get_results(multinet_model_data);
                    for (int i = 0; i < mn_result->num; i++)
                    {
                        if (mn_result->command_id[i] == WAKE_COMMAND_ID)
                        {
                            ESP_LOGI(TAG, "听到唤醒词了! prob=%f", mn_result->prob[i]);
                            wake_triggered = true;
                            break;
                        }
                    }
                    multinet_iface->clean(multinet_model_data);
                }
                else if (mn_state == ESP_MN_STATE_TIMEOUT)
                {
                    multinet_iface->clean(multinet_model_data);
                }

                if (wake_triggered)
                {
                    is_running = false;
                    input_buffer_len = 0;
                }
                else
                {
                    size_t remaining = input_buffer_len - (size_t)mn_chunksize;
                    memmove(input_buffer, &input_buffer[mn_chunksize], remaining * sizeof(int16_t));
                    input_buffer_len = remaining;
                }
            }
            xSemaphoreGive(buffer_mutex);

            if (wake_triggered && user_callback)
            {
                user_callback(current_disp_word);
            }
        }
    }
}
// ─── 公开 API：音频帧投喂（麦克风采集任务持续调用）────────────────────
//
// 【仅负责 feed】将原始 PCM 投喂给 AFE，降噪后的数据由 afe_fetch_task 取出处理。
// 调用约束：audio_feed_task 必须按 s_afe_feed_chunksize 大小调用此函数。

/**
 * @brief 将麦克风采集的原始 PCM 帧投喂给 AFE（上行数据入口）
 *
 * 调用 s_afe_iface->feed()，将 I2S 采集到的 PCM 送入 AFE 内部 ringbuffer。
 * AFE 自动进行 NS 降噪，降噪结果由 afe_fetch_task 异步取出。
 *
 * @param data 16-bit PCM 数据指针（来自 I2S DMA 读取的原始音频）
 * @param len  采样点数（必须等于 custom_wake_word_get_feed_chunksize()）
 * @return void
 *
 * @note 调用者：bsp_codec.c → audio_feed_task()（采集主循环中持续调用）
 * @note AFE 接口线程安全，feed 无需加锁
 */
void custom_wake_word_feed(const int16_t *data, size_t len)
{
    // AFE 未就绪时直接返回
    if (s_afe_data == NULL)
        return;

    // ── AEC "MR" 模式：交织麦克风 + 参考信号后送 AFE ───────────────────────
    // AFE 期望格式：[mic[0], ref[0], mic[1], ref[1], ...]（每通道 len 个采样）
    // 无参考信号（s_aec_ref_cb == NULL）时参考置零，AEC 不做任何减法，
    // 等价于纯 NS 模式，安全退化。

    // 获取参考信号：清零后尝试从回调填充（无播放时保持零）
    memset(s_aec_ref, 0, len * sizeof(int16_t)); // 先清零，确保回调不填充时参考为零
    aec_ref_cb_t cb = s_aec_ref_cb;              // 读一次，避免并发改变
    if (cb)
        cb(s_aec_ref, len); // 从回调获取当前播放的音频

    // 交织为 [mic, ref, mic, ref, ...]
    // todo: 为什么要交织？因为 AFE 的 "MR" 模式设计就是要求输入数据按照麦克风和参考信号交织的格式提供，
    for (size_t i = 0; i < len; i++)
    {
        s_aec_interleaved[2 * i] = data[i];          // 麦克风采样
        s_aec_interleaved[2 * i + 1] = s_aec_ref[i]; // 参考采样（扬声器回声）
    }

    // 投喂 AFE（内部线程安全）
    s_afe_iface->feed(s_afe_data, s_aec_interleaved);
}

// ─── 公开 API：停止引擎监听 ─────────────────────────────────────────────

/**
 * @brief 停止 MultiNet 命令词检测（AFE/VAD 继续运行，仅关闭词检测）
 *
 * 持锁后清空 input_buffer 并将 is_running 置为 false。
 * afe_fetch_task 检查到 is_running=false 后跳过 detect，直接 taskYIELD。
 * AFE feed/fetch 和 enhanced_pcm_hook 不受影响，编码器数据流保持畅通。
 *
 * @return void
 *
 * @note 调用者：session.c → session_on_wake_word()（TTS 播放期间防误触）
 */
void wake_word_stop(void)
{
    // 持锁后清空缓冲区并置运行标志为 false
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    is_running = false;
    xSemaphoreGive(buffer_mutex);
}

// ─── 公开 API：恢复引擎监听 ─────────────────────────────────────────────

/**
 * @brief 恢复 MultiNet 命令词检测监听
 *
 * 持锁后清空 input_buffer（丢弃会话期间积压的音频），并调用
 * s_afe_iface->reset_buffer() 重置 AFE 内部 ringbuf，防止旧音频帧
 * 在会话结束后立即误触下一次唤醒。最后置 is_running=true，恢复检测。
 *
 * @return void
 *
 * @note 调用者：session.c → session_close()（会话关闭后恢复监听）
 */
void wake_word_set_det_threshold(float threshold)
{
    if (multinet_iface && multinet_model_data)
        multinet_iface->set_det_threshold(multinet_model_data, threshold);
}

void wake_word_start(void)
{
    // 持锁后清空残留缓冲区，避免旧数据触发误识别，然后允许 feed 继续
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    // 重置 AFE 内部 ringbuf，丢弃积压的旧音频数据
    if (s_afe_iface && s_afe_data)
        s_afe_iface->reset_buffer(s_afe_data); // 重要！重置 AFE 内部状态，确保旧数据不干扰新检测
    // 确保恢复正常阈值（防止 TTS 被打断或异常关闭后阈值卡在 0.55）
    if (multinet_iface && multinet_model_data)
        multinet_iface->set_det_threshold(multinet_model_data, 0.18f);
    is_running = true;
    xSemaphoreGive(buffer_mutex);

    // 清空 MultiNet PCM 队列中会话期间积压的旧音频帧，防止恢复监听后误触发
    if (s_mn_pcm_buf != NULL)
    {
        size_t sz;
        void *item;
        while ((item = xRingbufferReceive(s_mn_pcm_buf, &sz, 0)) != NULL)
            vRingbufferReturnItem(s_mn_pcm_buf, item);
    }
}

// ─── 公开 API：获取 AFE feed 帧大小 ────────────────────────────────────

/**
 * @brief 获取 AFE feed 每次所需采样点数，供 audio_feed_task 分配缓冲区
 *
 * @return size_t s_afe_feed_chunksize（AFE 初始化后由 get_feed_chunksize 查询），
 *                若 AFE 未初始化则返回安全默认值 512
 *
 * @note 调用者：bsp_codec.c → audio_feed_task()（任务启动时查询缓冲区大小）
 */
size_t custom_wake_word_get_feed_chunksize(void)
{
    // audio_feed_task 必须按此大小投喂原始 PCM 给 AFE
    // 463 s_afe_feed_chunksize = s_afe_iface->get_feed_chunksize(s_afe_data);
    return (s_afe_feed_chunksize > 0) ? (size_t)s_afe_feed_chunksize : 512;
}

// ─── VAD / 增强 PCM 接口实现 ─────────────────────────────────────────────

void bsp_wake_word_set_vad_callback(vad_state_cb_t cb)
{
    s_vad_cb = cb;
}

/**
 * @brief 注册增强 PCM 数据回调（供编码器使用）
 */
void wake_word_set_enhanced_pcm_hook(enhanced_pcm_cb_t hook)
{
    // AFE 已就位：降噪后的 PCM 在 custom_wake_word_feed() 的 fetch 回调中输出
    // 不再需要底层 audio_set_pcm_hook 临时桥接
    s_enhanced_pcm_hook = hook; // 这样设计的目的是为了让编码器能够直接获取到经过 AFE 处理的音频数据
}

vad_state_t wake_word_get_vad_state(void) // 获取当前 VAD 状态
{
    return s_current_vad_state;
}

// ─── 公开 API：AEC 参考信号注册 ─────────────────────────────────────────

/**
 * @brief 注册 AEC 参考信号提供者（传 NULL 则注销，退化为零参考）
 *
 * @param cb 每次 feed 时同步调用，由调用方填充 samples 个参考 PCM 采样点。
 *           NULL → 参考恒为零 → AEC 不做回声消减（IDLE/LISTENING 安全退化）。
 *
 * @note 调用者：session.c → session_on_wake_word()（注册），session_close()（注销）
 */
void custom_wake_word_set_aec_ref(aec_ref_cb_t cb)
{
    s_aec_ref_cb = cb;
}
