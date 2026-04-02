#include "custom_wake_word.h"
#include "esp_afe_sr_models.h" // esp_afe_handle_from_config, afe_config_init

static const char *TAG = "BSP_WakeWord";

// ─── 常量定义 ────────────────────────────────────────────────────────────────
#define WAKE_COMMAND_ID 1
#define NVS_NAMESPACE "sys_config"
#define NVS_KEY_WAKEWORD "wakeword"
#define NVS_KEY_DISP_WORD "ww_disp"
#define DEFAULT_DISP_CN "云炎"
#define DEFAULT_WAKEWORD_CN "yun yan"
#define DEFAULT_DISP_EN "Hello Echo"
#define DEFAULT_WAKEWORD_EN "HELLO ECHO"

// ─── 模块级静态变量 ──────────────────────────────────────────────────────────

// MultiNet（命令词识别）
static esp_mn_iface_t *multinet_iface = NULL;
static model_iface_data_t *multinet_model_data = NULL;
static srmodel_list_t *models = NULL;

// AFE（音频前端：降噪 + VAD）
static const esp_afe_sr_iface_t *afe_handle = NULL;
static esp_afe_sr_data_t *afe_data = NULL;
static TaskHandle_t fetch_task_handle = NULL;

// 状态与回调
static volatile bool is_running = false;
static wake_word_detected_cb_t user_callback = NULL;
static SemaphoreHandle_t buffer_mutex = NULL;

static char current_wake_word[64] = {0};
static char current_disp_word[64] = {0};

// VAD & 增强 PCM 钩子
static volatile vad_state_t current_vad_state = VAD_SILENCE;
static vad_state_cb_t vad_cb = NULL;
static enhanced_pcm_cb_t enhanced_pcm_hook = NULL;

// ─── 语言检测 ─────────────────────────────────────────────────────────────────

static bool is_chinese_text(const char *s)
{
    const unsigned char *p = (const unsigned char *)s;
    while (*p)
    {
        if (*p >= 0xE4 && *p <= 0xE9)
            return true;
        p++;
    }
    return false;
}

// ─── NVS 工具函数 ─────────────────────────────────────────────────────────────

static void nvs_read_str(const char *key, char *dest, size_t max_len, const char *fallback)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &h) == ESP_OK)
    {
        size_t sz = max_len;
        if (nvs_get_str(h, key, dest, &sz) == ESP_OK)
        {
            nvs_close(h);
            ESP_LOGW(TAG, "NVS [%s]: %s", key, dest);
            return;
        }
        nvs_close(h);
    }
    strncpy(dest, fallback, max_len - 1);
    dest[max_len - 1] = '\0';
    ESP_LOGI(TAG, "NVS [%s] 未找到，使用默认: %s", key, dest);
}

static void nvs_write_str(const char *key, const char *value)
{
    nvs_handle_t h;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &h) != ESP_OK)
        return;
    nvs_set_str(h, key, value);
    nvs_commit(h);
    nvs_close(h);
}

void bsp_wake_word_load_from_nvs(char *dest, size_t max_len)
{
    bool cn = is_chinese_text(current_disp_word);
    nvs_read_str(NVS_KEY_WAKEWORD, dest, max_len,
                 cn ? DEFAULT_WAKEWORD_CN : DEFAULT_WAKEWORD_EN);
}

// ─── 词数统计 ─────────────────────────────────────────────────────────────────

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
            in_word = false;
        }
        else if (!in_word)
        {
            in_word = true;
            count++;
        }
        s++;
    }
    return count;
}

// ─── 语言模型加载 ─────────────────────────────────────────────────────────────

static esp_err_t load_model_for_lang(const char *lang)
{
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, lang);
    if (mn_name == NULL)
    {
        ESP_LOGE(TAG, "未找到 '%s' 语言模型", lang);
        return ESP_FAIL;
    }
    esp_mn_iface_t *new_iface = (esp_mn_iface_t *)esp_mn_handle_from_name(mn_name);
    if (new_iface == NULL)
    {
        ESP_LOGE(TAG, "获取 '%s' MultiNet 句柄失败", mn_name);
        return ESP_FAIL;
    }
    if (multinet_model_data != NULL)
    {
        multinet_iface->destroy(multinet_model_data);
        multinet_model_data = NULL;
    }
    multinet_iface = new_iface;
    multinet_model_data = multinet_iface->create(mn_name, 3000);
    if (multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "创建 '%s' MultiNet 模型失败", mn_name);
        multinet_iface = NULL;
        return ESP_FAIL;
    }
    float threshold = (strcmp(lang, ESP_MN_ENGLISH) == 0) ? 0.4f : 0.6f;
    multinet_iface->set_det_threshold(multinet_model_data, threshold);
    ESP_LOGW(TAG, "已加载语言模型: %s", mn_name);
    return ESP_OK;
}

// ─── 命令词注册 ───────────────────────────────────────────────────────────────

static esp_err_t register_command_word(void)
{
    esp_mn_commands_alloc(multinet_iface, multinet_model_data);
    esp_err_t err = esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
    if (err != ESP_OK)
    {
        const char *def = is_chinese_text(current_disp_word)
                              ? DEFAULT_WAKEWORD_CN
                              : DEFAULT_WAKEWORD_EN;
        ESP_LOGW(TAG, "命令词 [%s] 格式不支持，回退为默认词 [%s]", current_wake_word, def);
        strncpy(current_wake_word, def, sizeof(current_wake_word) - 1);
        esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
    }
    esp_mn_error_t *mn_err = esp_mn_commands_update();
    if (mn_err != NULL && mn_err->num > 0)
    {
        ESP_LOGE(TAG, "命令词注册有 %d 个失败:", mn_err->num);
        for (int i = 0; i < mn_err->num; i++)
            ESP_LOGE(TAG, "  x %s", mn_err->phrases[i]->string);
    }
    esp_mn_commands_print();
    return ESP_OK;
}

// ─── AFE fetch 任务 ──────────────────────────────────────────────────────────
// 持续从 AFE 取出降噪后的音频，更新 VAD 状态，并送入 MultiNet 检测

static void afe_fetch_task(void *arg)
{
    while (1)
    {
        // fetch 内部阻塞直到处理完一帧（约 32ms @ 16kHz/512 samples）
        afe_fetch_result_t *res = afe_handle->fetch(afe_data);
        if (!res || res->ret_value == ESP_FAIL)
            continue;

        // ── 1. 更新 VAD 状态，状态变化时触发回调 ──────────────────────────
        if (res->vad_state != current_vad_state)
        {
            current_vad_state = res->vad_state;
            if (vad_cb)
                vad_cb(current_vad_state);
        }

        // ── 2. 增强 PCM 钩子（会话录音阶段，转发降噪音频给编码器）────────
        if (enhanced_pcm_hook && res->data && res->data_size > 0)
            enhanced_pcm_hook(res->data, res->data_size / sizeof(int16_t));

        // ── 3. MultiNet 检测（仅在 is_running 时执行）──────────────────────
        if (!is_running || !multinet_model_data)
            continue;

        // 持锁保护 MultiNet：与 wake_word_update 互斥
        if (xSemaphoreTake(buffer_mutex, pdMS_TO_TICKS(10)) != pdTRUE)
            continue;

        if (!is_running || !multinet_model_data)
        {
            xSemaphoreGive(buffer_mutex);
            continue;
        }

        esp_mn_state_t mn_state = multinet_iface->detect(multinet_model_data, res->data);

        bool wake_triggered = false;
        if (mn_state == ESP_MN_STATE_DETECTED)
        {
            esp_mn_results_t *mn_result = multinet_iface->get_results(multinet_model_data);
            for (int i = 0; i < mn_result->num; i++)
            {
                if (mn_result->command_id[i] == WAKE_COMMAND_ID)
                {
                    ESP_LOGI(TAG, "听到唤醒词! display=%s prob=%.2f",
                             current_disp_word, mn_result->prob[i]);
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
            is_running = false;

        xSemaphoreGive(buffer_mutex);

        // 锁外执行回调（避免持锁时调用用户代码死锁）
        if (wake_triggered && user_callback)
            user_callback(current_disp_word);
    }
}

// ─── 公开 API：初始化 ────────────────────────────────────────────────────────

esp_err_t bsp_wake_word_init(wake_word_detected_cb_t cb)
{
    user_callback = cb;

    buffer_mutex = xSemaphoreCreateMutex();
    if (buffer_mutex == NULL)
    {
        ESP_LOGE(TAG, "互斥锁创建失败");
        return ESP_FAIL;
    }

    // 扫描模型分区
    models = esp_srmodel_init("model");
    if (models == NULL || models->num == -1)
    {
        ESP_LOGE(TAG, "模型分区初始化失败，请检查 partitions.csv");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 从 NVS 恢复上次使用的显示词/命令词
    nvs_read_str(NVS_KEY_DISP_WORD, current_disp_word,
                 sizeof(current_disp_word), DEFAULT_DISP_EN);

    const char *lang = is_chinese_text(current_disp_word) ? ESP_MN_CHINESE : ESP_MN_ENGLISH;
    if (load_model_for_lang(lang) != ESP_OK)
    {
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    bsp_wake_word_load_from_nvs(current_wake_word, sizeof(current_wake_word));
    if (!is_chinese_text(current_disp_word))
    {
        for (int i = 0; current_wake_word[i]; i++)
            current_wake_word[i] = toupper((unsigned char)current_wake_word[i]);
    }
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
    register_command_word();

    // ── 初始化 AFE：单麦克风"M"，语音识别模式，低功耗 ──────────────────────
    // "M" = 1 麦克风，无参考信号（无 AEC）
    // AFE_TYPE_SR: 语音识别场景（含 NS 噪声抑制，不含非线性噪声抑制）
    // AFE_MODE_LOW_COST: 低功耗模式（适合 ESP32-S3 单核 VAD 场景）
    afe_config_t *afe_config = afe_config_init("M", models, AFE_TYPE_SR, AFE_MODE_LOW_COST);
    if (afe_config == NULL)
    {
        ESP_LOGE(TAG, "AFE 配置创建失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 调整 VAD 参数：加快语音开始响应，避免吃字
    afe_config->vad_init = true;
    afe_config->vad_mode = VAD_MODE_1;  // 适中灵敏度
    afe_config->vad_min_speech_ms = 64; // 64ms 语音才触发（减少误触）
    afe_config->vad_min_noise_ms = 800; // 800ms 静音才判断说完
    afe_config->memory_alloc_mode = AFE_MEMORY_ALLOC_MORE_PSRAM;
    afe_config->afe_perferred_core = 1; // AFE SE 任务绑定 Core 1

    afe_handle = esp_afe_handle_from_config(afe_config);
    if (afe_handle == NULL)
    {
        ESP_LOGE(TAG, "AFE 句柄获取失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    afe_data = afe_handle->create_from_config(afe_config);
    if (afe_data == NULL)
    {
        ESP_LOGE(TAG, "AFE 实例创建失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 启动 AFE fetch 任务（Core 1，与 audio_feed_task 同核）
    BaseType_t ret = xTaskCreatePinnedToCore(
        afe_fetch_task, "afe_fetch",
        4096, NULL, 5,
        &fetch_task_handle, 1);
    if (ret != pdPASS)
    {
        ESP_LOGE(TAG, "AFE fetch 任务创建失败");
        return ESP_FAIL;
    }

    // 允许 feed → fetch 开始处理
    is_running = true;
    ESP_LOGW(TAG, "唤醒词引擎初始化完成（含 AFE）. 显示=%s 命令词=%s",
             current_disp_word, current_wake_word);
    return ESP_OK;
}

// ─── 公开 API：运行时更新唤醒词 ─────────────────────────────────────────────

esp_err_t wake_word_update(const char *wake_word_display, const char *wake_word_pinyin)
{
    if (multinet_iface == NULL || multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "引擎未初始化，无法更新");
        return ESP_FAIL;
    }
    if (count_words(wake_word_pinyin) < 2)
    {
        ESP_LOGW(TAG, "唤醒词太短！[%s] 必须包含至少 2 个音节/单词", wake_word_pinyin);
        return ESP_ERR_INVALID_ARG;
    }

    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    is_running = false;

    bool new_is_cn = is_chinese_text(wake_word_display);
    bool cur_is_cn = is_chinese_text(current_disp_word);

    if (new_is_cn != cur_is_cn)
    {
        const char *new_lang = new_is_cn ? ESP_MN_CHINESE : ESP_MN_ENGLISH;
        if (load_model_for_lang(new_lang) != ESP_OK)
        {
            ESP_LOGE(TAG, "语言切换失败，保持原模型");
            is_running = true;
            xSemaphoreGive(buffer_mutex);
            return ESP_FAIL;
        }
    }

    strncpy(current_disp_word, wake_word_display, sizeof(current_disp_word) - 1);
    current_disp_word[sizeof(current_disp_word) - 1] = '\0';

    strncpy(current_wake_word, wake_word_pinyin, sizeof(current_wake_word) - 1);
    current_wake_word[sizeof(current_wake_word) - 1] = '\0';
    if (!new_is_cn)
    {
        for (int i = 0; current_wake_word[i]; i++)
            current_wake_word[i] = toupper((unsigned char)current_wake_word[i]);
    }

    register_command_word();
    multinet_iface->clean(multinet_model_data);

    nvs_write_str(NVS_KEY_WAKEWORD, current_wake_word);
    nvs_write_str(NVS_KEY_DISP_WORD, current_disp_word);
    ESP_LOGW(TAG, "唤醒词更新成功！display=%s command=%s",
             current_disp_word, current_wake_word);

    is_running = true;
    xSemaphoreGive(buffer_mutex);
    return ESP_OK;
}

// ─── 公开 API：获取每帧采样点数（供 audio_feed_task 使用）────────────────────

size_t custom_wake_word_get_chunksize(void)
{
    // 优先返回 AFE feed chunksize（AFE 初始化后此路径生效）
    if (afe_handle && afe_data)
        return afe_handle->get_feed_chunksize(afe_data);
    // 回退：AFE 未就绪时用 MultiNet chunksize
    if (multinet_iface && multinet_model_data)
        return multinet_iface->get_samp_chunksize(multinet_model_data);
    return 0;
}

// ─── 公开 API：音频帧投喂（audio_feed_task 持续调用）────────────────────────

void custom_wake_word_feed(const int16_t *data, size_t len)
{
    // 将原始 PCM 送入 AFE；AFE 内部异步处理，fetch 任务取结果
    if (afe_handle && afe_data)
        afe_handle->feed(afe_data, data);
}

// ─── 公开 API：停止/恢复引擎监听 ─────────────────────────────────────────────

void bsp_wake_word_stop(void)
{
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    is_running = false;
    xSemaphoreGive(buffer_mutex);
}

void bsp_wake_word_start(void)
{
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    is_running = true;
    xSemaphoreGive(buffer_mutex);
    // 清除 MultiNet 内部历史状态，从干净状态开始检测
    if (multinet_iface && multinet_model_data)
        multinet_iface->clean(multinet_model_data);
    // 重置 AFE ring buffer，丢弃会话期间积累的音频
    if (afe_handle && afe_data)
        afe_handle->reset_buffer(afe_data);
}

// ─── 公开 API：VAD / 增强 PCM 钩子 ──────────────────────────────────────────

void bsp_wake_word_set_vad_callback(vad_state_cb_t cb)
{
    vad_cb = cb;
}

void bsp_wake_word_set_enhanced_pcm_hook(enhanced_pcm_cb_t hook)
{
    enhanced_pcm_hook = hook;
}

vad_state_t bsp_wake_word_get_vad_state(void)
{
    return current_vad_state;
}
