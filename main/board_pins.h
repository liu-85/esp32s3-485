#ifndef BOARD_PINS_H
#define BOARD_PINS_H

/**
 * 引脚表选择器 —— 根据目标芯片自动 include 对应的引脚定义。
 *
 *   ESP32-S3 → board_pins_s3.h（4 通道完整方案）
 *   ESP32-C3 → board_pins_c3.h（2 通道降级方案）
 *
 * 业务代码（motor / clutch / filament_sensor / ams_controller / web_server）
 * 全部 #include "board_pins.h"，不用改。
 *
 * ★ 判断目标芯片用 IDF 官方自动生成的宏 CONFIG_IDF_TARGET_ESP32C3
 *   （在 build 目录的 sdkconfig.h 里，由 Kconfig 系统注入；CMake 编译时
 *   把 build/include 加入头文件搜索路径，main.c 的 #include "sdkconfig.h"
 *   就是靠这个解析的）。之前误用的 ESP_IDF_TARGET_C3 从未被定义，
 *   导致 C3 固件也走了 S3 分支 → 选 S3 引脚表 → GPIO 越界崩溃。
 *   全工程（board_pins.h / main.c / config.c / web_server.c）都统一用
 *   CONFIG_IDF_TARGET_ESP32C3 判断。
 */
#include "sdkconfig.h"   /* CONFIG_IDF_TARGET_ESP32C3（IDF 官方自动注入） */

#if defined(CONFIG_IDF_TARGET_ESP32C3)
  #include "board_pins_c3.h"
#else
  #include "board_pins_s3.h"
#endif

#endif /* BOARD_PINS_H */
