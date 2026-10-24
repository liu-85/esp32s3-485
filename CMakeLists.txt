# ===========================================================================
# ESP-AMS-S3 —— 基于 ESP-IDF 的拓竹打印机自动换料系统（ESP32-S3）
# ===========================================================================
#
# 这是 ESP-IDF 工程的顶层 CMakeLists.txt。它只做两件事：
#   1. 指定工程名
#   2. 把 ESP-IDF 的构建系统拉进来（下面那一行 include）
#
# 真正的编译单元在 main/CMakeLists.txt 里。
#
# 构建步骤（先装好 ESP-IDF，并 source 过 export.sh / export.ps1）:
#     idf.py set-target esp32s3      # 只需第一次
#     idf.py build
#     idf.py -p COM4 flash monitor
#
# 注意：目标芯片必须是 esp32s3。本工程用的是 42 针 S3 模组（8MB Flash），
#       引脚表在 main/board_pins.h 里，改接线只改那一个文件。
# ---------------------------------------------------------------------------

cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# ---------------------------------------------------------------------------
# 版本号即"出厂编号"：app_version 会出现在 /status 和网页上，用户刷完固件
# 第一件事就是核对它，分不清刷没刷上会白排查半天。
#
# 0.19.0（2026-10-23）：
#   · 485 运动指令按参考实现重写成 16 通道状态机（send_out / on_use /
#     pull_back 语义对齐），Dxx 回真实的 use_flag / pressure / left_char，
#     状态变化才投递物理动作；「485 运动执行」默认打开
#   · 「自吸上料」整套删除（第三只微动、AUTOLOAD 命令/状态、网页入口、
#     配置掩码）；掩码那一字节改名保留占位，配置长度不变、不清用户数据
#
# 0.30.1（2026-10-23）：
#   · 照搬 Tianshu src/BambuBus.cpp 的运动状态机：7 态 / 单占用
#     （get_loaded_filament 语义）/ accept 组合 / 送料先卸载本台其它 3 槽 /
#     send_out→on_use 的 5 秒闸门 / on_use 期间 meters 按时间累加
#   · 应答补真实 meters（Cxx 与 Dxx 都回累计米数）；use_flag 改为按台扫描
#     （get_filament_use_flag 语义）；pressure / left_char 与 Tianshu 真值对齐
#   · 帧类分发：Cxx(0x03) 与 Dxx(0x04) 各按自身字段布局驱动同一个状态机
#     （现场 /bus485raw 实证：Cxx 携带运动意图，Dxx 恒为 0x01 查询帧 → 天然 no-op）
#   · 修「到位后反复进退料」：状态机离开 stop_on_use 时取消本地续送窗口
#     （ams_485_exit_decide_window）—— 打印机到位后仅约 0.9s 就发退料，而窗口
#     有 6s，不关窗就会正转续送与退料方向互搏
#   · 修「到位后没有及时停电机」：poll_485_decide 的续送由阻塞 400ms 改为
#     非阻塞计时，不再卡住 ams_task、不再让 STOP/退料指令排队等待
#
# 0.30.2（2026-10-24）：
#   · 修「挤出机只拉进一点就退料、如此循环」：mot_pressure 在 before_on_use
#     （预送，本固件此时已停电机）误报 0x4700「正在推料」，与已停电机的事实
#     自相矛盾，打印机原地等压力下降→超时判挤出失败→发退料。改报 0x2B00，
#     与 Tianshu get_pressure_value 实机行为一致（prev_motion=m 使三元恒取
#     0x2B00 分支）
#   · 修日志丢行致状态迁移看不全：全局空轮询帧（无运动意图）不再刷日志、
#     不动去重游标；顺带把「运动命令」里 slot 的错误指针偏移（slot=0 打出
#     整串 "0123"）改成直接打槽号
# ---------------------------------------------------------------------------
project(esp-ams-s3 VERSION 0.30.2)
