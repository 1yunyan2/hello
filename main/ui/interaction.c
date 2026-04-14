/**
 * @file bsp_interaction.c
 * @brief 情绪矩阵与动作解析引擎
 */

#include "interaction.h"

static const char *TAG = "INTERACTION";

// ==========================================
// 1. 数据结构定义 (严格对齐你的 Excel 表头)
// ==========================================
/**
 * @brief 单轴舵机动作参数（情绪矩阵中每个舵机轴的动作描述）
 *
 * 一个动作步骤描述"在此情绪下，该轴舵机如何运动"：
 *   - 先运动到 angle_1（前半段，如 左转/前摆）
 *   - 再运动到 angle_2（后半段，如 右转/后摆；单向动作填 90 即归中）
 *   - 以上两步组成一次循环，共执行 count 次
 *
 * 角度说明：中心点为 90°。
 *   头部：左+ 右-（左转 30° → angle=120，右转 30° → angle=60）
 *   手臂：前+ 后-（前摆 20° → angle=110，后摆 15° → angle=75）
 */
typedef struct
{
    float angle_1;  ///< 动作前半段目标角度（度），如左转或前摆的目标位置
    float angle_2;  ///< 动作后半段目标角度（度），如右转或后摆；单向动作填 90（归中）
    uint32_t speed; ///< 运动速度延时（step_ms，对应 SERVO_SPEED_xxx 宏，值越大越慢）
    uint8_t count;  ///< 该轴在此情绪下的循环执行次数（0 表示不动）
} ActionStep_t;

/**
 * @brief 情绪矩阵行（情绪 ID → 全套硬件动作映射表的一行）
 *
 * g_emotion_matrix[] 数组中每个元素对应一种情绪，
 * bsp_interaction_play() 按 emotion_id 查表后，依次执行各字段定义的动作。
 */
typedef struct
{
    robot_emotion_t emotion_id; ///< 情绪枚举 ID，作为查表键（对应 robot_emotion_t）
    const char *screen_anim;    ///< 屏幕动画标识字符串（传给 UI 层的动画名称，预留接口）
    uint8_t motor_mode;         ///< 震动马达模式（0:无振动, 1:轻1次50ms, 2:短促2次, 3:连续, 4:长震1次）
    const char *audio_file;     ///< 音效文件名（传给音频层播放，预留接口）
    ActionStep_t head;          ///< 头部舵机（CH_HEAD）动作参数
    ActionStep_t left_arm;      ///< 左臂舵机（CH_L_ARM）动作参数
    ActionStep_t right_arm;     ///< 右臂舵机（CH_R_ARM）动作参数
} InteractionMatrix_t;

// ==========================================
// 2. 情绪动作矩阵表 (你的 Excel 数据都在这里)
// ⚠️ 角度换算说明：中心点为 90度。
// 头部：左+ 右- (如 左30度=120, 右30度=60)
// 手臂：前+ 后- (如 前20度=110, 后15度=75)
// ==========================================
const InteractionMatrix_t g_emotion_matrix[] = {
    // 【第 1 行】开心
    {
        .emotion_id = EMO_HAPPY,
        .screen_anim = "anim_happy_stars",
        .motor_mode = 2,
        .audio_file = "laugh_short.mp3",
        // 头部：左30 -> 右30 (120, 60)，快速，执行 3 次
        .head = {120.0f, 60.0f, SERVO_SPEED_FAST, 3},
        // 左臂：前20 -> 后15 (110, 75)，中速，执行 2 次
        .left_arm = {110.0f, 75.0f, SERVO_SPEED_MID, 2},
        // 右臂：前15 -> 后20 (105, 70)，中速，执行 2 次
        .right_arm = {105.0f, 70.0f, SERVO_SPEED_MID, 2}},

    // 【第 2 行】好奇
    {
        .emotion_id = EMO_CURIOUS,
        .screen_anim = "anim_curious_q",
        .motor_mode = 1,
        .audio_file = "doubt.mp3",
        // 头部：左25 -> 右25 (115, 65)，缓慢，执行 2 次
        .head = {115.0f, 65.0f, SERVO_SPEED_SLOW, 2},
        // 左臂：前30 (120)，缓慢，执行 1 次
        .left_arm = {120.0f, 90.0f, SERVO_SPEED_SLOW, 1},
        // 右臂：前30 (120)，缓慢，执行 1 次
        .right_arm = {120.0f, 90.0f, SERVO_SPEED_SLOW, 1}},

    // 【第 3 行】傲娇
    {
        .emotion_id = EMO_TSUNDERE,
        .screen_anim = "anim_tsundere",
        .motor_mode = 0,
        .audio_file = "hmph.mp3",
        // 头部：左15 (105)，极慢，执行 1 次
        .head = {105.0f, 90.0f, 50, 1}, // 50 是自定义极慢速
        // 左臂：后30 (60)，快速，执行 1 次
        .left_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1},
        // 右臂：后30 (60)，快速，执行 1 次
        .right_arm = {60.0f, 90.0f, SERVO_SPEED_FAST, 1}},

    // 【第 4 行】怕痒
    {
        .emotion_id = EMO_TICKLISH,
        .screen_anim = "anim_ticklish",
        .motor_mode = 3,
        .audio_file = "ticklish.mp3",
        // 头部：左15 -> 右15 (105, 75)，极快速，执行 5 次
        .head = {105.0f, 75.0f, 2, 5}, // 2 是极快
        // 左臂：前25 -> 后25 (115, 65)，快速，执行 3 次
        .left_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 3},
        // 右臂：前25 -> 后25 (115, 65)，快速，执行 3 次
        .right_arm = {115.0f, 65.0f, SERVO_SPEED_FAST, 3}}

    // ... 你可以对照你的 Excel，把剩下的几十个表情全按照这个格式粘进来
};

// ==========================================
// 3. 私有：马达震动控制 (这里接入你的 GPIO 16)
// ==========================================

/**
 * @brief 按指定模式触发震动马达脉冲（私有函数）
 *
 * 通过直接操作 GPIO16 产生指定节奏的震动脉冲序列。
 * 震动模式与 InteractionMatrix_t.motor_mode 字段对应：
 *   0 = 无震动（直接返回）
 *   1 = 轻微 1 次（50ms 高电平）
 *   2 = 短促 2 次（50ms 高 → 50ms 低 → 50ms 高）
 *
 * @param mode 震动模式（0~4，目前实现了 0/1/2）
 * @return void
 *
 * @note 调用者：bsp_interaction_play()（步骤 4，执行舵机前先触发震动）
 * @note 使用 vTaskDelay 阻塞，不适合在中断上下文调用
 */
static void trigger_vibration_motor(uint8_t mode)
{
    // BSP_MOTOR_VIB_PIN 需要在 bsp_board.h 中定义为 16
    if (mode == 0)
        return;

    // 简单示例，实际可根据 mode 设计不同长短的脉冲
    if (mode == 1)
    { // 轻微1次
        gpio_set_level(16, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(16, 0);
    }
    else if (mode == 2)
    { // 短促2次
        gpio_set_level(16, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(16, 0);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(16, 1);
        vTaskDelay(pdMS_TO_TICKS(50));
        gpio_set_level(16, 0);
    }
}

// ==========================================
// 4. 核心解析引擎：同步多轴运动与外部硬件
// ==========================================
void bsp_interaction_play(robot_emotion_t target_emotion)
{
    const InteractionMatrix_t *cmd = NULL;
    int table_size = sizeof(g_emotion_matrix) / sizeof(g_emotion_matrix[0]);

    // 1. 查表寻找对应情绪
    for (int i = 0; i < table_size; i++)
    {
        if (g_emotion_matrix[i].emotion_id == target_emotion)
        {
            cmd = &g_emotion_matrix[i];
            break;
        }
    }

    if (cmd == NULL)
    {
        ESP_LOGE(TAG, "未找到情绪 ID: %d", target_emotion);
        return;
    }

    ESP_LOGI(TAG, ">>> 开始执行情绪动作: %d <<<", target_emotion);

    // 2. 触发屏幕 UI (发消息给 LVGL 任务)
    // 伪代码: ui_manager_play_anim(cmd->screen_anim);
    ESP_LOGI(TAG, "-> 播放屏幕动画: %s", cmd->screen_anim);

    // 3. 触发音频播放 (发消息给 ES8311 任务)
    // 伪代码: audio_player_play_file(cmd->audio_file);
    ESP_LOGI(TAG, "-> 播放音效: %s", cmd->audio_file);

    // 4. 触发马达震动
    trigger_vibration_motor(cmd->motor_mode);

    // 5. 舵机同步解析引擎
    // 找出三个舵机中，执行次数最多的一个，作为外层大循环
    uint8_t max_loop = cmd->head.count;
    if (cmd->left_arm.count > max_loop)
        max_loop = cmd->left_arm.count;
    if (cmd->right_arm.count > max_loop)
        max_loop = cmd->right_arm.count;

    for (uint8_t loop = 0; loop < max_loop; loop++)
    {

        // 动作前半段 (如 左转 / 前摆)
        if (loop < cmd->head.count)
            bsp_servo_move_smooth(CH_HEAD, cmd->head.angle_1, cmd->head.speed);
        if (loop < cmd->left_arm.count)
            bsp_servo_move_smooth(CH_L_ARM, cmd->left_arm.angle_1, cmd->left_arm.speed);
        if (loop < cmd->right_arm.count)
            bsp_servo_move_smooth(CH_R_ARM, cmd->right_arm.angle_1, cmd->right_arm.speed);

        // 动作后半段 (如 右转 / 后摆)
        if (loop < cmd->head.count)
            bsp_servo_move_smooth(CH_HEAD, cmd->head.angle_2, cmd->head.speed);
        if (loop < cmd->left_arm.count)
            bsp_servo_move_smooth(CH_L_ARM, cmd->left_arm.angle_2, cmd->left_arm.speed);
        if (loop < cmd->right_arm.count)
            bsp_servo_move_smooth(CH_R_ARM, cmd->right_arm.angle_2, cmd->right_arm.speed);
    }

    // 6. 动作执行完毕，所有舵机平滑归中，恢复待机姿态
    bsp_servo_move_smooth(CH_HEAD, 90.0f, SERVO_SPEED_MID);
    bsp_servo_move_smooth(CH_L_ARM, 90.0f, SERVO_SPEED_MID);
    bsp_servo_move_smooth(CH_R_ARM, 90.0f, SERVO_SPEED_MID);

    ESP_LOGI(TAG, ">>> 情绪动作执行完毕 <<<");
}