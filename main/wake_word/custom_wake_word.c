#include "custom_wake_word.h"
#include "bsp/bsp_board.h" // 引入 bsp_board 用于挂载原始 PCM 钩子

static const char *TAG = "BSP_WakeWord";

// ─── 常量定义 ────────────────────────────────────────────────────────────
#define WAKE_COMMAND_ID 1                    // 唤醒词在命令词表中的固定 ID（唯一一条命令）
#define NVS_NAMESPACE "sys_config"           // NVS 命名空间（与其他模块共享）
#define NVS_KEY_WAKEWORD "wakeword"          // NVS Key：命令词（拼音或英文）
#define NVS_KEY_DISP_WORD "ww_disp"          // NVS Key：显示文字（用于下次启动判断语言）
#define DEFAULT_DISP_CN "你好伙伴"           // 出厂默认中文显示词
#define DEFAULT_WAKEWORD_CN "ni hao huo ban" // 出厂默认中文命令词（拼音）
#define DEFAULT_DISP_EN "Hello Echo"         // 出厂默认英文显示词
#define DEFAULT_WAKEWORD_EN "HELLO ECHO"     // 出厂默认英文命令词（mn6_en 词表全大写）
#define AUDIO_BUFFER_MAX 2048                // 音频环形缓冲区最大采样点数

// ─── 模块级静态变量 ──────────────────────────────────────────────────────
static esp_mn_iface_t *multinet_iface = NULL;          // MultiNet 接口函数表指针
static model_iface_data_t *multinet_model_data = NULL; // MultiNet 模型运行时数据
static srmodel_list_t *models = NULL;                  // SPIFFS 模型分区扫描结果列表

static volatile bool is_running = false;             // 引擎运行标志（volatile：可能在中断/任务间读写）
static wake_word_detected_cb_t user_callback = NULL; // 用户注册的触发回调
static SemaphoreHandle_t buffer_mutex = NULL;        // 保护 input_buffer 的互斥锁

// ─── VAD / 增强 PCM 接口回调状态 ────────────────────────────────────────
static vad_state_cb_t s_vad_cb = NULL;
static enhanced_pcm_cb_t s_enhanced_pcm_hook = NULL;
static volatile vad_state_t s_current_vad_state = VAD_SILENCE;

static char current_wake_word[64] = {0}; // 当前生效的命令词（拼音或全大写英文）
static char current_disp_word[64] = {0}; // 当前生效的显示文字（用于语言判断）

static int16_t input_buffer[AUDIO_BUFFER_MAX]; // 音频积累缓冲区（跨帧拼接用）
static size_t input_buffer_len = 0;            // 缓冲区当前有效采样点数

// ─── 语言检测 ─────────────────────────────────────────────────────────────
// UTF-8 中文汉字首字节范围：0xE4 ~ 0xE9（覆盖 CJK 统一汉字主区）
// 含该字节则判定为中文，否则视为英文

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

// 从 NVS 读取字符串；若读取失败则填入 fallback 默认值
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

// 向 NVS 写入字符串（覆盖已有值）
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
void bsp_wake_word_load_from_nvs(char *dest, size_t max_len)
{
    // 根据当前已记录的显示词判断语言，选择对应的回退默认值
    bool cn = is_chinese_text(current_disp_word);
    nvs_read_str(NVS_KEY_WAKEWORD, dest, max_len,
                 cn ? DEFAULT_WAKEWORD_CN : DEFAULT_WAKEWORD_EN);
}

// ─── 词数统计 ─────────────────────────────────────────────────────────────
// 按空格分隔统计词/音节数量
// 规则：中文拼音"yun yan"=2，英文"hello echo"=2，单音节"yun"=1（不允许）

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
    //   中文(cn)：0.6（prob 分布高，0.6 足以过滤噪声）
    //   英文(en)：0.4（BPE 路径长，prob 天然偏低，0.4 才能正常触发）
    float threshold = (strcmp(lang, ESP_MN_ENGLISH) == 0) ? 0.4f : 0.6f;
    multinet_iface->set_det_threshold(multinet_model_data, threshold);

    ESP_LOGW(TAG, "已加载语言模型: %s", mn_name);
    return ESP_OK;
}

// ─── 命令词注册 ───────────────────────────────────────────────────────────

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

esp_err_t bsp_wake_word_init(wake_word_detected_cb_t cb)
{
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

    // 扫描 SPIFFS "model" 分区，建立模型文件列表
    models = esp_srmodel_init("model");
    if (models == NULL || models->num == -1)
    {
        // 分区未挂载或 partitions.csv 中 model 分区缺失
        ESP_LOGE(TAG, "模型分区初始化失败，请检查 partitions.csv");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 从 NVS 读取上次保存的显示词，用于推断上次使用的语言
    nvs_read_str(NVS_KEY_DISP_WORD, current_disp_word,
                 sizeof(current_disp_word), DEFAULT_DISP_EN);

    // 根据显示词语言选择并加载对应 MultiNet6 模型
    const char *lang = is_chinese_text(current_disp_word) ? ESP_MN_CHINESE : ESP_MN_ENGLISH;
    if (load_model_for_lang(lang) != ESP_OK)
    {
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 从 NVS 加载命令词；英文词自动转全大写（mn6_en 词表要求）
    bsp_wake_word_load_from_nvs(current_wake_word, sizeof(current_wake_word));
    if (!is_chinese_text(current_disp_word))
    {
        for (int i = 0; current_wake_word[i]; i++)
            current_wake_word[i] = toupper((unsigned char)current_wake_word[i]);
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

    // 允许 custom_wake_word_feed   开始处理音频帧
    is_running = true;
    return ESP_OK;
}

// ─── 公开 API：运行时更新唤醒词 ─────────────────────────────────────────

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

    // 将新词持久化到 NVS（下次上电自动恢复）
    nvs_write_str(NVS_KEY_WAKEWORD, current_wake_word);
    nvs_write_str(NVS_KEY_DISP_WORD, current_disp_word);
    ESP_LOGW(TAG, "唤醒词更新成功！display=%s command=%s",
             current_disp_word, current_wake_word);

    // 恢复运行并释放锁，feed 任务可以继续投喂音频
    is_running = true;
    input_buffer_len = 0;
    xSemaphoreGive(buffer_mutex);
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

// ─── 公开 API：音频帧投喂（麦克风采集任务持续调用）────────────────────

void custom_wake_word_feed(const int16_t *data, size_t len)
{
    // 快速路径：引擎未运行或模型未就绪时直接返回，避免加锁开销
    if (!is_running || multinet_model_data == NULL)
        return;

    // 尝试获取互斥锁（500ms 超时，防止更新任务持锁时投喂任务永久阻塞）
    if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(500)) != pdTRUE)
    {
        ESP_LOGW(TAG, "获取音频缓冲锁超时，丢弃本帧");
        return;
    }

    // 将新帧数据追加到积累缓冲区
    if (input_buffer_len + len <= AUDIO_BUFFER_MAX)
    {
        memcpy(&input_buffer[input_buffer_len], data, len * sizeof(int16_t));
        input_buffer_len += len;
    }
    else
    {
        // 缓冲区溢出（下游检测速度跟不上投喂速度），清零重来
        ESP_LOGW(TAG, "Audio buffer overflow!");
        input_buffer_len = 0;
    }

    // 获取模型本次 detect() 需要消耗的采样点数
    int chunksize = multinet_iface->get_samp_chunksize(multinet_model_data);
    bool wake_triggered = false;

    // 只要缓冲区中有足够的数据，就循环送入模型检测
    while (input_buffer_len >= chunksize && is_running)
    {
        // 将 chunksize 个采样点送入模型进行一轮检测，返回当前状态
        esp_mn_state_t mn_state = multinet_iface->detect(multinet_model_data, input_buffer);

        if (mn_state == ESP_MN_STATE_DETECTED)
        {
            // 检测到命令词，取出结果列表
            esp_mn_results_t *mn_result = multinet_iface->get_results(multinet_model_data);
            for (int i = 0; i < mn_result->num; i++)
            {
                if (mn_result->command_id[i] == WAKE_COMMAND_ID)
                {
                    // 命令词 ID 匹配，标记触发
                    ESP_LOGI(TAG, "听到唤醒词了! display=%s prob=%f",
                             current_disp_word, mn_result->prob[i]);
                    wake_triggered = true;
                    break;
                }
            }
            // 重置模型内部音频状态，准备下一轮检测
            multinet_iface->clean(multinet_model_data);
        }
        else if (mn_state == ESP_MN_STATE_TIMEOUT)
        {
            // 3 秒窗口内未听到完整唤醒词，重置状态重新开始
            // 说一半被打断也走这条路：超时 → clean → 下一帧从零开始
            multinet_iface->clean(multinet_model_data);
        }

        if (wake_triggered)
        {
            // 触发成功：停止引擎（防止立即重复触发），清空缓冲区
            is_running = false;
            input_buffer_len = 0;
            break;
        }

        if (is_running)
        {
            // 将已消耗的 chunksize 个采样点从缓冲区头部移除（滑动窗口）
            size_t remaining = input_buffer_len - chunksize;
            memmove(input_buffer, &input_buffer[chunksize], remaining * sizeof(int16_t));
            input_buffer_len = remaining;
        }
    }

    // 释放互斥锁，让更新任务可以进入
    xSemaphoreGive(buffer_mutex);

    // 锁外执行回调（避免在持锁状态下调用用户代码导致死锁）
    if (wake_triggered && user_callback)
        user_callback(current_disp_word);
}

// ─── 公开 API：停止引擎监听 ─────────────────────────────────────────────

void bsp_wake_word_stop(void)
{
    // 持锁后清空缓冲区并置运行标志为 false
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    is_running = false;
    xSemaphoreGive(buffer_mutex);
}

// ─── 公开 API：恢复引擎监听 ─────────────────────────────────────────────

void bsp_wake_word_start(void)
{
    // 持锁后清空残留缓冲区，避免旧数据触发误识别，然后允许 feed 继续
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    is_running = true;
    xSemaphoreGive(buffer_mutex);
}

// ─── VAD / 增强 PCM 接口实现 ─────────────────────────────────────────────

void bsp_wake_word_set_vad_callback(vad_state_cb_t cb)
{
    s_vad_cb = cb;
}

void bsp_wake_word_set_enhanced_pcm_hook(enhanced_pcm_cb_t hook)
{
    s_enhanced_pcm_hook = hook;
    // 【临时桥接】：在 AFE(降噪模块) 完全实现前，直接将底层的原始 PCM 挂载过去
    // 这样能保证你的音频流能立刻通过 WebSocket 顺利发给大模型进行对话
    audio_set_pcm_hook(hook);
}

vad_state_t bsp_wake_word_get_vad_state(void)
{
    return s_current_vad_state;
}
