#pragma once

#include <stdbool.h>
#include <stdint.h>

/**
 * @file remote_control.h
 * @brief App 远程手动控制（舵机角度 / GIF 显示）—— 一次性动作，不进情绪序列
 *
 * ═══ 需求背景 ═══
 * App 端是一个 3D 形象，用户手动点击某个部位 → 下发 MQTT 指令 → 设备对应舵机摆动；
 * 或用户手动选一张 GIF → 下发文件名 → 设备切到该 GIF 循环播放。
 * 这类操作是「单次手动控制」，不保存为动作序列、不参与情绪矩阵，与既有的
 * ia_custom_action_t / robot_emotion_t 两套体系完全解耦。
 *
 * ═══ 核心行为：30s 冻结窗口 ═══
 * 收到任一远程指令后进入「远程控制态」，此期间：
 *   - 空闲动作序列的【GIF 自动轮播】与【随附舵机动作】被冻结（不切图、不动舵机）；
 *   - 其余功能全部照常运行（唤醒词、对话、离线提示音、按键、闹钟…），冻结只针对
 *     「空闲态的 GIF + 舵机」这一条链路，不是全局挂起。
 * 30s 无新指令则自动到期；期间收到新指令则【顶掉旧的并重置 30s】（不排队）。
 *
 * ═══ 退出路径与舵机归中策略 ═══
 * 两类退出，舵机处理方式【故意不同】：
 *   1) 无后续动作接管 → 归中（90°）后再回空闲序列
 *      - 30s 自然到期
 *      - 进入深度待机（低功耗本就要求归位）
 *   2) 有后续动作接管 → 【不归中】，直接让位
 *      - 触摸触发情绪动作
 *      - 唤醒 / 进入对话
 *      理由：bsp_servo_move_smooth 内部自带「从当前角度插值到目标角度」，后续动作
 *      直接发它自己的目标角即可平滑过渡；若先归中会多出一次无意义的复位抽动。
 *
 * ═══ 屏蔽规则 ═══
 * 以下状态下收到远程指令 → 直接丢弃 + 打日志（不排队、不回执）：
 *   - 对话中（session_get_state() != SESSION_IDLE）：远程改图/甩头会打断对话体验，
 *     且对话本身在驱动状态 GIF（听/说三态），两者会打架。
 *   - 游戏中（ui_get_current_view() == UI_VIEW_GAME）：游戏独占屏幕与舵机。
 *   - OTA 升级中：升级期设备应完全独占，任何外部干预都是风险。
 * 深度待机（屏已关）【不屏蔽】：内部经 standby_notify_activity() 自动唤醒亮屏后执行，
 * 因为用户在 app 上主动操作就是期望看到设备有反应（含 GIF 画面）。
 *
 * ═══ 线程模型 ═══
 * MQTT 事件回调线程【只入队】（微秒级）→ 独立 worker 任务执行舵机运动。
 * 必须如此：bsp_servo_move_smooth 内含 vTaskDelay，单次最长阻塞约 1.3s，若直接在
 * MQTT 回调里跑，用户在 app 上快速连点会把 MQTT 事件循环堵死，导致 keepalive
 * 超时断连。同项目 unbind / ota 指令已是「回调入队 + 异步任务」模式，此处对齐。
 *
 * 模块依赖：bsp_board.h（舵机）、ui_port.h（切图/视图查询）、standby.h（待机唤醒）、
 *          session.h（对话态查询）
 */

/** @brief GIF 文件名最大长度（不含目录前缀），含结尾 '\0' */
#define REMOTE_GIF_NAME_MAX 48

/**
 * @brief 初始化远程控制模块（创建指令队列 + worker 任务 + 30s 超时定时器）
 *
 * 幂等：重复调用直接返回。必须在舵机、LCD/UI、待机模块均初始化完成后调用，
 * 因为 worker 任务一旦跑起来就可能立刻调用 bsp_servo_move_smooth / 切图接口。
 *
 * @return true 初始化成功；false 队列/任务/定时器创建失败（模块整体不可用，
 *         后续 submit 均静默丢弃，不影响其他功能）
 * @note 调用者：application.c → application_init()（standby_init 之前或之后均可）
 */
bool remote_control_init(void);

/**
 * @brief 提交一条远程舵机指令（非阻塞，供 MQTT 回调直接调用）
 *
 * 内部只做「状态屏蔽判断 + 入队」，真正的 bsp_servo_move_smooth 由 worker 任务执行。
 * 入队成功即进入/续期远程控制态（重置 30s 冻结窗口）。
 *
 * @param channel 舵机通道（CH_HEAD / CH_L_ARM / CH_R_ARM），非法值直接丢弃
 * @param angle   【相对中位的偏移角】-90~+90（度），0 = 中位（正前方）；
 *                内部换算成舵机物理绝对角：-90→0°, 0→90°, +90→180°。
 *                超出物理范围由 bsp_servo 内部软限位裁剪，此处不做额外限制
 * @return true 已入队；false 被屏蔽 / 队列满 / 模块未初始化
 * @note 调用者：mqtt_protocol.c → mqtt_event_handler 的 "servo" 指令分支
 */
bool remote_control_submit_servo(uint8_t channel, float angle);

/**
 * @brief 提交一条远程 GIF 指令（非阻塞，供 MQTT 回调直接调用）
 *
 * 只接收【文件名】（如 "1_3.gif"），设备内部拼成 "S:/gif/<file>" 播放。
 * 固件【不维护】任何「编号 → 文件名」映射表——映射关系由 app / 后端持有，
 * 这样后续往外挂 flash 新增 GIF 时固件无需任何改动、无需重新烧录。
 *
 * 安全校验（不通过即丢弃 + 打日志）：
 *   - 必须以 ".gif" 结尾（大小写不敏感）；
 *   - 不得包含 '/' '\\' 或 ".."（防止跳出 /S/gif 目录）；
 *   - 长度 < REMOTE_GIF_NAME_MAX。
 * 文件是否真实存在【不在此处校验】：由 UI 层切图时统一处理（不存在则保持当前画面
 * 并打日志），避免在 MQTT 回调线程里做文件 IO。
 *
 * @param file GIF 文件名（不含路径），如 "1_3.gif"
 * @return true 已入队；false 被屏蔽 / 文件名非法 / 队列满 / 模块未初始化
 * @note 调用者：mqtt_protocol.c → mqtt_event_handler 的 "gif" 指令分支
 */
bool remote_control_submit_gif(const char *file);

/**
 * @brief 查询当前是否处于远程控制态（30s 冻结窗口内）
 *
 * 供空闲动作序列在切图/投递舵机动作前查询，为 true 则跳过本次（冻结）。
 * 轻量原子读，可在 LVGL 定时器回调等高频路径中调用。
 *
 * @return true 冻结中（空闲序列应让位）；false 正常
 * @note 调用者：ui_port.c → main_gif_switch_timer_cb（空闲随机轮播分支）
 */
bool remote_control_is_active(void);

/**
 * @brief 打断并退出远程控制态（供真实交互 / 待机调用）
 *
 * 幂等：非远程控制态下调用无副作用，可安全地在各交互入口无脑调用。
 *
 * @param center_servo 退出时是否把三轴舵机归中到 90°：
 *        - true ：无后续动作接管的场景（进入深度待机）。注意归中是【阻塞】操作，
 *                 内部改为投递给 worker 异步执行，调用方不会被阻塞。
 *        - false：有后续动作立即接管的场景（触摸情绪 / 唤醒对话）。让后续动作
 *                 直接从当前角度平滑过渡，避免多一次复位抽动。
 * @return void
 * @note 调用者：ui_port.c（触摸分发、进游戏）、application.c（唤醒命中）、
 *              standby.c（进入深度待机，传 true）
 */
void remote_control_cancel(bool center_servo);
