# ===========================================================================
# main 组件 —— 编译单元与网页资源
# ===========================================================================
#
# 支持双目标：esp32s3（4 通道完整方案） / esp32c3（2 通道降级方案）
# 由 board_pins.h 自动根据 ESP_IDF_TARGET 选择对应引脚表：
#   ESP32-S3 → board_pins_s3.h
#   ESP32-C3 → board_pins_c3.h
# 所有业务代码（motor / clutch / filament_sensor / ams_controller）
# 只 #include "board_pins.h"，不需要为两个目标分别写。
#
# 两件事：
#   1. SRCS          本工程的 C 源文件
#   2. EMBED_FILES   把 web/ 下的三个前端文件**编译进固件**（.rodata 段）
#
# 为什么把网页嵌进固件，而不是放进 SPIFFS 分区：
#   · 零额外工具依赖（不用 spiffsgen 那一步，不会因为工具链差异构建失败）
#   · 读取是纯内存操作，没有文件系统挂载失败的风险
#   · 现在有真正的双分区 OTA 了，网页要改直接升级固件就行
#   partitions.csv 里的 storage 分区先留着，供以后放可热更资源 / 导出日志用。
#
# 【由这段代码生成的符号】
#   _binary_index_html_start / _end   ← web/index.html
#   _binary_app_js_start     / _end   ← web/app.js
#   _binary_style_css_start  / _end   ← web/style.css
#   web_server.c 里用 extern 引用它们。
#
# 【新增的文件说明】
#   除了你给的结构，这里多加了三个文件，理由如下：
#     board_pins.h    —— 引脚表。原项目把引脚集中在 hardware_config.py，
#                        这里对应成单独的头文件，改接线只动这一处。
#     bambu_proto.c/h —— 拓竹 MQTT 报文解析（对应 get_event_info.py +
#                        bambu_const.py），单独放是为了让 bambu_mqtt.c
#                        只管收发、不掺业务解析。
#     log_buffer.c/h  —— 日志环形缓冲（对应 logout.py），Web 端"设备日志"
#                        面板读的就是它。
#
#   ★ 另外 MQTT 那个模块的文件名是 bambu_mqtt.c/h，**不是** mqtt_client.c/h：
#     ESP-IDF 自带的 mqtt 组件里已经有 mqtt_client.h 了，同名会把它的头文件
#     遮住，导致 esp_mqtt_client_init 这些符号找不到、编译直接失败。
#     改叫 bambu_mqtt 之后正好和原来的 bambu/bambu_mqtt.py 对上，命名更一致。
# ---------------------------------------------------------------------------

set(WEB_DIR "${CMAKE_CURRENT_LIST_DIR}/../web")

idf_component_register(
    SRCS
        "main.c"
        "config.c"
        "wifi_manager.c"
        "web_server.c"
        "bambu_mqtt.c"
        "comm_485.c"
        "motor.c"
        "clutch.c"
        "adc_shared.c"
        "current_sense.c"
        "filament_sensor.c"
        "ams_controller.c"
        "bambu_proto.c"
        "log_buffer.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        esp_http_server
        mqtt
        app_update
        driver
        # ★ 2026-10-22 新增两项：
        #   esp_adc        —— 微动改成"单 IO 双微动电阻编码"后要读 ADC
        #                     （adc_oneshot_* 在 esp_adc 组件里，不在 driver 里）
        #   esp_driver_uart —— 485 直连打印机要自己开一路 UART
        #                     （driver 是兼容壳，显式写出来免得以后被裁掉）
        esp_adc
        esp_driver_uart
        json
        mdns
        # ★ esp_psram 虽然一行代码都没调，但必须列在这里：
        #   sdkconfig.defaults 里的 CONFIG_SPIRAM / CONFIG_SPIRAM_MODE_OCT 等
        #   选项是定义在 esp_psram 组件自己的 Kconfig 里的。组件不在依赖列表里，
        #   它的 Kconfig 就可能不被解析 → 那几行配置被当成"未知配置项"丢掉，
        #   PSRAM 悄悄没打开（不报错，只是内存少了，很难发现）。
        esp_psram
    EMBED_FILES
        "${WEB_DIR}/index.html"
        "${WEB_DIR}/app.js"
        "${WEB_DIR}/style.css"
)
