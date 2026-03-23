#include "custom_wake_word.h"

static const char *TAG = "CustomWakeWordC";

#define WAKE_COMMAND_ID 1
#define NVS_NAMESPACE "sys_config"
#define NVS_KEY_WAKEWORD "wakeword"
#define DEFAULT_WAKEWORD "xiao zhi" // 出厂默认唤醒词拼音
#define AUDIO_BUFFER_MAX 2048       // 音频内部缓存大小

// 全局状态与实例
static esp_mn_iface_t *multinet_iface = NULL;
static model_iface_data_t *multinet_model_data = NULL;
static srmodel_list_t *models = NULL;

static volatile bool is_running = false;
static wake_word_detected_cb_t user_callback = NULL;
static SemaphoreHandle_t buffer_mutex = NULL;

// C语言环形缓冲替代 std::vector
static int16_t input_buffer[AUDIO_BUFFER_MAX];
static size_t input_buffer_len = 0;
static char current_wake_word[64] = {0};

// 内部函数：从 NVS 读取保存的唤醒词
void load_wakeword_from_nvs(char *dest, size_t max_len)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READONLY, &my_handle);
    if (err == ESP_OK)
    {
        size_t required_size = max_len;
        err = nvs_get_str(my_handle, NVS_KEY_WAKEWORD, dest, &required_size);
        nvs_close(my_handle);
        if (err == ESP_OK)
        {
            ESP_LOGI(TAG, "从 NVS 恢复自定义唤醒词: %s", dest);
            return;
        }
    }
    // 读取失败或首次开机，使用默认值
    ESP_LOGI(TAG, "未找到记忆的唤醒词，使用默认值: %s", DEFAULT_WAKEWORD);
    strncpy(dest, DEFAULT_WAKEWORD, max_len - 1);
}

// 内部函数：保存唤醒词到 NVS
static esp_err_t save_wakeword_to_nvs(const char *pinyin)
{
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &my_handle);
    if (err != ESP_OK)
        return err;

    err = nvs_set_str(my_handle, NVS_KEY_WAKEWORD, pinyin);
    if (err == ESP_OK)
    {
        err = nvs_commit(my_handle);
    }
    nvs_close(my_handle);
    return err;
}

esp_err_t custom_wake_word_init(wake_word_detected_cb_t cb)
{
    user_callback = cb;
    buffer_mutex = xSemaphoreCreateMutex();
    if (buffer_mutex == NULL)
    {
        ESP_LOGE(TAG, "❌ 互斥锁创建失败！内存不足？");
        return ESP_FAIL;
    }
    input_buffer_len = 0;

    // 1. 初始化 SR 模型列表
    models = esp_srmodel_init("srmodel"); // 假设你的模型在分区表中叫 model
    if (models == NULL || models->num == -1)
    {
        ESP_LOGE(TAG, "模型分区初始化失败，请检查 partitions.csv");
        return ESP_FAIL;
    }

    // 2. 寻找中文 MultiNet 模型
    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, "cn");
    if (mn_name == NULL)
    {
        ESP_LOGW(TAG, "未找到 'cn' 语言模型，回退到默认 MultiNet");
        mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, NULL);
    }
    if (mn_name == NULL)
    {
        ESP_LOGE(TAG, "加载 MultiNet 失败！引擎指针为空");
        return ESP_FAIL;
    }

    // 3. 创建引擎实例 (3000ms 为支持的最长语音时长)
    multinet_iface = (esp_mn_iface_t *)esp_mn_handle_from_name(mn_name);
    multinet_model_data = multinet_iface->create(mn_name, 3000);

    // 设置识别阈值，小智项目中默认设为 0.2，提高灵敏度
    multinet_iface->set_det_threshold(multinet_model_data, 0.2);

    // 4. 从记忆(NVS)中读取名字并注册
    load_wakeword_from_nvs(current_wake_word, sizeof(current_wake_word));

    esp_mn_commands_clear();
    esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
    esp_mn_commands_update();

    ESP_LOGI(TAG, "自定义唤醒词引擎初始化完成. 当前听命于: [%s]", current_wake_word);

    is_running = true;
    return ESP_OK;
}

esp_err_t custom_wake_word_update(const char *new_pinyin)
{
    if (multinet_iface == NULL || multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "引擎未初始化，无法更新");
        return ESP_FAIL;
    }

    // 暂停检测防止多线程冲突
    custom_wake_word_stop();

    ESP_LOGI(TAG, "准备更新设备唤醒词为: %s", new_pinyin);

    // 更新引擎内部词典
    esp_mn_commands_clear();
    esp_err_t err = esp_mn_commands_add(WAKE_COMMAND_ID, new_pinyin);
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "添加命令词失败！请确保手机发来的拼音是全小写，且用空格隔开。");
        // 恢复老名字
        esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
        esp_mn_commands_update();
        custom_wake_word_start();
        return err;
    }

    esp_mn_commands_update();

    // 更新本地记录并存入 NVS 闪存
    strncpy(current_wake_word, new_pinyin, sizeof(current_wake_word) - 1);
    save_wakeword_to_nvs(current_wake_word);

    ESP_LOGI(TAG, "设备唤醒词更新成功并已保存到闪存！");

    custom_wake_word_start();
    return ESP_OK;

    // ESP_LOGI(TAG, "准备保存唤醒词到NVS: %s", new_pinyin);

    // // 1. 直接保存到NVS
    // esp_err_t err = save_wakeword_to_nvs(new_pinyin);
    // if (err == ESP_OK)
    // {
    //     // 2. 更新本地缓存（可选，仅用于日志）
    //     strncpy(current_wake_word, new_pinyin, sizeof(current_wake_word) - 1);
    //     ESP_LOGI(TAG, "✅ 【核心目标达成】唤醒词已成功保存到NVS闪存！断电不丢失！");
    //     return ESP_OK;
    // }
    // else
    // {
    //     ESP_LOGE(TAG, "❌ NVS保存失败");
    //     return err;
    // }
}

size_t custom_wake_word_get_chunksize(void)
{
    if (multinet_iface && multinet_model_data)
    {
        return multinet_iface->get_samp_chunksize(multinet_model_data);
    }
    return 0;
}

void custom_wake_word_feed(const int16_t *data, size_t len)
{
    if (!is_running || multinet_model_data == NULL)
        return;

    xSemaphoreTake(buffer_mutex, portMAX_DELAY);

    // 将新数据追加到内部缓冲区
    if (input_buffer_len + len <= AUDIO_BUFFER_MAX)
    {
        memcpy(&input_buffer[input_buffer_len], data, len * sizeof(int16_t));
        input_buffer_len += len;
    }
    else
    {
        // 缓冲区溢出保护
        ESP_LOGW(TAG, "Audio buffer overflow!");
        input_buffer_len = 0;
    }

    int chunksize = multinet_iface->get_samp_chunksize(multinet_model_data);

    // 当缓冲区数据足够一个 chunk 时，交给引擎处理
    while (input_buffer_len >= chunksize && is_running)
    {

        // 提取一个 chunk 进行检测
        esp_mn_state_t mn_state = multinet_iface->detect(multinet_model_data, input_buffer);

        if (mn_state == ESP_MN_STATE_DETECTED)
        {
            esp_mn_results_t *mn_result = multinet_iface->get_results(multinet_model_data);
            for (int i = 0; i < mn_result->num && is_running; i++)
            {

                // 如果返回的 ID 匹配我们设置的唤醒词 ID
                if (mn_result->command_id[i] == WAKE_COMMAND_ID)
                {
                    ESP_LOGI(TAG, "听到唤醒词了! prob=%f", mn_result->prob[i]);

                    // 暂停接收音频，防止连续多次触发
                    is_running = false;
                    input_buffer_len = 0;

                    // 触发用户的回调函数去执行亮灯/语音回复等操作
                    if (user_callback)
                    {
                        user_callback(current_wake_word);
                    }
                }
            }
            // 清理状态机，准备下一次识别
            multinet_iface->clean(multinet_model_data);
        }
        else if (mn_state == ESP_MN_STATE_TIMEOUT)
        {
            multinet_iface->clean(multinet_model_data);
        }

        // 把处理过的数据从缓冲区移除 (向前平移)
        if (is_running)
        {
            size_t remaining = input_buffer_len - chunksize;
            memmove(input_buffer, &input_buffer[chunksize], remaining * sizeof(int16_t));
            input_buffer_len = remaining;
        }
    }

    xSemaphoreGive(buffer_mutex);
}

void custom_wake_word_stop(void)
{
    is_running = false;
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    xSemaphoreGive(buffer_mutex);
}

void custom_wake_word_start(void)
{
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    is_running = true;
    xSemaphoreGive(buffer_mutex);
}