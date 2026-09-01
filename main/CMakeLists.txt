idf_component_register(SRCS
"main.c"
"bsp/bsp_board.c"
"bsp/bsp_wifi.c"
"bsp/udp_logger.c"
"bsp/bsp_codec.c"
# "bsp/bsp_pca9536.c"   # 【暂时停用】PCA9536 IO 扩展器驱动，暂不使用，已移出编译

"bsp/bsp_lcd.c"
"bsp/bsp_touch.c"
"bsp/bsp_servo.c"
"bsp/servo_manager.c"
"wake_word/custom_wake_word.c"
"protocol/mqtt_protocol.c"
"protocol/websocket_client.c"
"audio/audio_processor.c"
"audio/audio_encoder.c"
"audio/audio_decoder.c"
"audio/offline_audio.c"
"session/session.c"
"application.c"
"protocol/auth.c"
"ui/interaction.c"
"ui/reminder.c"
"ui/ui_port.c"
"ui/standby.c"
"ui/remote_control.c"
"ui/font_loader.c"
"ui/font_cn_16.c"
# 2026-08-14 数字/中文字体已改外挂 Flash .bin（见 assets/font/，经 ui/font_loader.c 懒加载）：
# "ui/font_cn_32.c"    # → font_cn_32.bin（日历年月 + 游戏大字）
# "ui/font_cn_24.c"    # → font_cn_24.bin（未使用）
# "ui/font_cn_12.c"    # → font_cn_12.bin（英文副文案）
# "ui/font_num_140.c"  # → font_num_140.bin（日历日号）
# "ui/font_wx_50.c"    # → font_wx_50.bin（天气温度）
# "ui/font_time_144.c" # → font_time_144.bin（时间页）
# "ui/font_cal_74.c"   # → font_cal_74.bin（日历日号）
"ui/font_alarm_59.c"  # 闹钟（59 暂不放外挂，仍编译进固件）
"ui/dishu.c"
"ui/picture1.c"
# 2026-08-10 移除 rl/tq/sz/djs1 四个内置图标（共 725KB 死重量）：
# 功能盘早已改走外挂 Flash S:/img/*.bin，这四个 C 数组的声明与引用在 ui_port.c 中
# 均已注释，编译进固件却永不使用。物理文件保留在 main/ui/ 下，需要时取消注释即可恢复。
# 腾出的空间用于 140px 大号数字字体（224KB）。
# "ui/rl (1).c"
# "ui/tq (1).c"
# "ui/sz (1).c"
# "ui/djs1 (1).c"
# 天气图标（和风开源图标库，128×128 RGB565A8）。文件名沿用和风 icon 码，
# 变量名统一为 img_wx_XXX（C 标识符不能以数字开头）。映射表见 ui/weather_icons.h
"ui/100.c"
"ui/101.c"
"ui/104.c"
"ui/302.c"
"ui/305.c"
"ui/400.c"
"ui/501.c"
"ui/502.c"
"games/games.c"
"games/game_whack.c"
"games/game_race.c"
"games/game_jump.c"
"games/p2.c"
"games/p3.c"
"games/p4.c"
"games/ds_bg.c"
"games/ds2_mole.c"
"games/ds1.c"
"games/p5.c"
"games/ds3.c"
"games/p6.c"
"games/sc.c"
"games/tyt.c"
"games/c1.c"
"games/c2.c"
"games/c3.c"
"games/c4.c"
"games/c5.c"
"games/c6.c"
"games/c7.c"
"games/jt1.c"
"games/jt3.c"
"games/jt4.c"
"games/jt5.c"
"games/jt6.c"
"games/jt7.c"
"games/jp.c"
"bsp/bsp_flash.c"
"bsp/bsp_battery.c"
"bsp/bsp_ota.c"
"blufi/blufi_init.c"
"blufi/blufi_security.c"
"heap_forensic.c"  # 【堆损坏排查·2026-07-29】纯诊断模块，不参与业务逻辑
INCLUDE_DIRS "."
EMBED_FILES "audio/test_assets/test.mp3" "audio/test_assets/test.p3" "audio/test_assets/cs.mp3"
)
# 注：main 组件默认会包含所有公共组件（包括 esp_adc），无需显式 PRIV_REQUIRES。
# 之前尝试添加 PRIV_REQUIRES esp_adc 会触发 IDF 把 main 当作普通组件，
# 反而导致 esp_wifi / mqtt / wifi_provisioning 等不再自动注入而报错。

