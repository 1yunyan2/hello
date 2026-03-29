#include "custom_wake_word.h"

static const char *TAG = "CustomWakeWordC";

#define WAKE_COMMAND_ID 1
#define NVS_NAMESPACE "sys_config"
#define NVS_KEY_WAKEWORD "wakeword" // 键值标签
#define DEFAULT_WAKEWORD "yun yan"  // 出厂默认唤醒词拼音
#define AUDIO_BUFFER_MAX 2048       // 音频内部缓存大小

// 全局状态与实例
static esp_mn_iface_t *multinet_iface = NULL;          // 模型接口
static model_iface_data_t *multinet_model_data = NULL; // 模型数据
static srmodel_list_t *models = NULL;                  // 模型列表

static volatile bool is_running = false;             // 运行状态
static wake_word_detected_cb_t user_callback = NULL; // 用户回调
static SemaphoreHandle_t buffer_mutex = NULL;        // 缓冲锁

// C语言环形缓冲替代 std::vector
static int16_t input_buffer[AUDIO_BUFFER_MAX]; // 音频缓存
static size_t input_buffer_len = 0;            // 缓存长度
static char current_wake_word[64] = {0};       // 当前唤醒词

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
    strncpy(dest, DEFAULT_WAKEWORD, max_len - 1); // 使用默认值
    dest[max_len - 1] = '\0';                     // 确保字符串有结束符
}

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
    // 互斥锁
    buffer_mutex = xSemaphoreCreateMutex();

    if (buffer_mutex == NULL)
    {
        ESP_LOGE(TAG, "❌ 互斥锁创建失败！内存不足？");
        return ESP_FAIL;
    }
    input_buffer_len = 0;

    models = esp_srmodel_init("model");
    if (models == NULL || models->num == -1)
    {
        ESP_LOGE(TAG, "模型分区初始化失败，请检查 partitions.csv");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    char *mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, "cn");
    if (mn_name == NULL)
    {
        ESP_LOGW(TAG, "未找到 'cn' 语言模型，回退到默认 MultiNet");
        mn_name = esp_srmodel_filter(models, ESP_MN_PREFIX, NULL);
    }
    if (mn_name == NULL)
    {
        ESP_LOGE(TAG, "加载 MultiNet 失败！引擎指针为空");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    multinet_iface = (esp_mn_iface_t *)esp_mn_handle_from_name(mn_name); // 获取 MultiNet 接口
    if (multinet_iface == NULL)
    {
        ESP_LOGE(TAG, "获取 MultiNet 句柄失败");
        vSemaphoreDelete(buffer_mutex);
        buffer_mutex = NULL;
        return ESP_FAIL;
    }
    multinet_model_data = multinet_iface->create(mn_name, 3000); // 创建 MultiNet 模型数据
    if (multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "创建 MultiNet 模型数据失败");
        vSemaphoreDelete(buffer_mutex); // 彻底清除锁
        buffer_mutex = NULL;
        return ESP_FAIL;
    }

    // 设置较高阈值防止误触发
    multinet_iface->set_det_threshold(multinet_model_data, 0.85);

    load_wakeword_from_nvs(current_wake_word, sizeof(current_wake_word)); // 恢复唤醒词

    // 清除，添加，更新
    esp_mn_commands_clear();
    esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
    esp_mn_commands_update();

    ESP_LOGI(TAG, "自定义唤醒词引擎初始化完成. [%s]", current_wake_word);

    is_running = true;
    return ESP_OK;
}

esp_err_t custom_wake_word_update(const char *new_pinyin) // 更新唤醒词
{
    if (multinet_iface == NULL || multinet_model_data == NULL)
    {
        ESP_LOGE(TAG, "引擎未初始化，无法更新");
        return ESP_FAIL;
    }
    custom_wake_word_stop();
    ESP_LOGI(TAG, "准备更新设备唤醒词为: %s", new_pinyin);
    memset(current_wake_word, 0, sizeof(current_wake_word));
    strncpy(current_wake_word, new_pinyin, sizeof(current_wake_word) - 1);

    esp_mn_commands_clear();
    // esp_err_t err = esp_mn_commands_add(WAKE_COMMAND_ID, new_pinyin);        // 添加新的唤醒词
    esp_err_t err = esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word); // 添加新的唤醒词
    if (err != ESP_OK)
    {
        ESP_LOGE(TAG, "添加命令词失败！请确保手机发来的拼音是可读拼音，且用空格隔开。");
        load_wakeword_from_nvs(current_wake_word, sizeof(current_wake_word));
        esp_mn_commands_add(WAKE_COMMAND_ID, current_wake_word);
        esp_mn_commands_update();
        custom_wake_word_start();
        return err;
    }

    esp_mn_commands_update();
    save_wakeword_to_nvs(current_wake_word); // 保存到 NVS
    ESP_LOGI(TAG, "设备唤醒词更新成功并已保存到闪存！");
    custom_wake_word_start();
    return ESP_OK;
}

size_t custom_wake_word_get_chunksize(void) // 获取 MultiNet 模型数据块大小
{
    if (multinet_iface && multinet_model_data) // 检查指针
    {
        return multinet_iface->get_samp_chunksize(multinet_model_data); // 获取块大小
    }
    return 0;
}

// feed 投喂
void custom_wake_word_feed(const int16_t *data, size_t len)
{
    if (!is_running || multinet_model_data == NULL)
        return;

    xSemaphoreTake(buffer_mutex, portMAX_DELAY); // 获取锁

    if (input_buffer_len + len <= AUDIO_BUFFER_MAX) // 如果剩余空间足够，则将数据添加到缓冲区中
    {
        memcpy(&input_buffer[input_buffer_len], data, len * sizeof(int16_t)); // 将数据复制到缓冲区中
        input_buffer_len += len;
    }
    else
    {
        ESP_LOGW(TAG, "Audio buffer overflow!"); // 缓冲区溢出
        input_buffer_len = 0;
    }

    int chunksize = multinet_iface->get_samp_chunksize(multinet_model_data); // 获取数据块大小

    // ✅ 标记位，用于将业务回调和锁彻底剥离
    bool wake_triggered = false;

    while (input_buffer_len >= chunksize && is_running)
    // 只要缓冲区有数据且>=数据块大小且引擎运行中
    {

        esp_mn_state_t mn_state = multinet_iface->detect(multinet_model_data, input_buffer);
        // 检测: 检测唤醒词: 返回结果: ESP_MN_STATE_DETECTED: 唤醒词已触发

        if (mn_state == ESP_MN_STATE_DETECTED)
        {
            esp_mn_results_t *mn_result = multinet_iface->get_results(multinet_model_data);
            for (int i = 0; i < mn_result->num; i++)
            {
                if (mn_result->command_id[i] == WAKE_COMMAND_ID)
                {
                    ESP_LOGI(TAG, "听到唤醒词了! prob=%f", mn_result->prob[i]);
                    wake_triggered = true; // 标记已唤醒
                    break;                 // 跳出 for 循环
                }
            }
            multinet_iface->clean(multinet_model_data);
        }
        else if (mn_state == ESP_MN_STATE_TIMEOUT)
        {
            multinet_iface->clean(multinet_model_data);
        }

        // ✅ 防暴毙核心逻辑：如果听到唤醒词，立刻打断，跳出 while！
        if (wake_triggered)
        {
            is_running = false;
            input_buffer_len = 0;
            break; // 彻底跳出 while，避开下方的 memmove
        }

        // 仅在未唤醒时平移缓冲区
        if (is_running)
        {
            size_t remaining = input_buffer_len - chunksize; // 剩余数据长度
            memmove(input_buffer, &input_buffer[chunksize], remaining * sizeof(int16_t));
            // 平移缓冲区：移动数据，将剩余数据移动到缓冲区的起始位置
            input_buffer_len = remaining; // 更新缓冲区长度
        }
    }

    xSemaphoreGive(buffer_mutex); // ✅ 永远先放锁！

    // ✅ 脱离危险区后，再安全地触发回调
    if (wake_triggered && user_callback)
    {
        user_callback(current_wake_word);
    }
}

void custom_wake_word_stop(void)
{
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    is_running = false;
    xSemaphoreGive(buffer_mutex);
}

void custom_wake_word_start(void)
{
    xSemaphoreTake(buffer_mutex, portMAX_DELAY);
    input_buffer_len = 0;
    is_running = true;
    xSemaphoreGive(buffer_mutex);
}
