/**
 * main.c —— 启动流程
 * ============================================================================
 *
 * 这个文件只干一件事：**按正确的顺序把各个模块拉起来**。它不含任何业务逻辑。
 *
 * ----------------------------------------------------------------------------
 * ★ 启动顺序为什么是这个样子（每一步都有理由，改之前先读）
 * ----------------------------------------------------------------------------
 *
 *   0. NVS 初始化
 *      所有配置都存在 NVS 里，必须第一个起来。
 *
 *   1. 日志缓冲
 *      放在最前面，这样后面每个模块的启动日志都能进环形缓冲、在网页上看得到。
 *      （MicroPython 版是先建 Web 再建日志，结果启动阶段的东西全看不到。）
 *
 *   2. 配置
 *      wifi / MQTT / 通道映射 / 引脚相关的参数都要读它。
 *
 *   3. 引脚自检 + 打印接线表
 *      在初始化硬件**之前**做。接线表有错（重复占用、踩保留脚、strapping 脚
 *      当输出）的话，宁可在动电机之前就报出来。
 *
 *   4. 执行机构：电机 → 离合 → 微动
 *      顺序有讲究：
 *        · 电机先初始化 —— 它把两个 LEDC 通道的输出清成 0，保证上电不动；
 *        · 离合第二 —— 它一上来就把 4 路全部写成"断开"，绝不能带着上次的
 *          残留状态起跑；
 *        · 微动最后 —— 它只是输入，什么时候起都不影响安全。
 *
 *   5. AMS 控制器
 *      它会把电机/离合/微动/打印机上报接起来。**故意放在 WiFi 之前**：
 *      这样即使 WiFi 连不上，网页连热点也控制不了……所以真正的理由是这个 ——
 *      它会立刻开始轮询微动，用户一插料就能触发，不用等网络。
 *
 *   6. WiFi
 *      先尝试直连已保存的路由器；连不上（或压根没配过）才开配置热点。
 *
 *   7. Web 服务
 *      必须在 WiFi 之后 —— 它要把 socket 挂到 netif 上，netif 是 WiFi 建的。
 *
 *   8. 确认固件升级有效
 *      只有走到这里（前面全部成功）才调 esp_ota_mark_app_valid_cancel_rollback()。
 *      这一句是"回滚保险"的关键：新固件启动后如果不主动确认，下一次重启
 *      引导程序就会认为它有问题、自动切回上一版。
 *      放在最后是刻意的 —— 只有"WiFi 也起来了、网页也起来了"才算体检通过。
 *
 * ----------------------------------------------------------------------------
 * ★ 对比 MicroPython 版最大的不同：这里没有"堆碎片"这一说
 * ----------------------------------------------------------------------------
 * Python 版 main.py 的阶段划分（先开射频、再预建监听 socket、最后才 import
 * 应用）全部是为了绕开"GC 不压缩、最大连续块不够"这个物理限制。那种写法
 * 极其脆弱：往前面插一个 import 就可能让射频申请不到 24KB 连续内存而报
 * Wifi Unknown Error 0x0101。
 *
 * ESP-IDF 下 WiFi 驱动用的是**静态分配**的缓冲，编译期就定好了，不存在
 * "抢不到连续内存"的问题。所以这里的顺序完全可以按逻辑依赖来排，而不用
 * 按内存来排 —— 这就是重写的最大收益之一。
 */

#include <stdio.h>
#include <string.h>

#include "sdkconfig.h"        /* CONFIG_IDF_TARGET / CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ */

#include "esp_app_desc.h"
#include "esp_chip_info.h"    /* esp_chip_info() —— ★ v5.0 起不再由 esp_system.h 间接包含 */
#include "esp_idf_version.h"  /* ESP_IDF_VERSION / ESP_IDF_VERSION_VAL */
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "nvs_flash.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "ams_controller.h"
#include "bambu_mqtt.h"
#include "board_pins.h"
#include "clutch.h"
#include "comm_485.h"
#include "config.h"
#include "current_sense.h"
#include "filament_sensor.h"
#include "log_buffer.h"
#include "motor.h"
#include "web_server.h"
#include "wifi_manager.h"

/* ==========================================================================
 * 常量
 * ========================================================================== */

/** 开机等 WiFi 拿 IP 的最长时间（每存一组 SSID 各等这么久） */
#define BOOT_WIFI_TIMEOUT_MS 15000

/* ==========================================================================
 * 一、启动横幅
 * ========================================================================== */

/**
 * 把一整段多行文本打到**串口**（不进网页那圈 48 行的日志缓冲）。
 *
 * ★★ 为什么要区分"串口"和"网页日志"（2026-09-24 现场要求）★★
 *   现场原话：「启动后间隔一直在发自检的日志，自检一次就可以了，
 *   把日志再精简一下。」
 *
 *   根因是**环形缓冲只有 48 行**，而开机一次就要往里塞约 30 行
 *   （横幅 10 行 + 接线表 8 行 + 业务配置 12 行）。于是网页上一点开，
 *   满屏全是"每次开机都一模一样"的自检信息，真正有用的换料日志
 *   （阶段 / 收到指令 / 退料 / 进料 / 换色完成）全被顶出去了。
 *
 *   这些东西的价值是"**接好线第一次上电时看一遍**"，不是"每次都看"。
 *   所以：完整内容走串口（`idf.py monitor` 或任意串口工具都看得到），
 *   网页上只留一行摘要。业务配置本来网页「设置」面板里全都有，
 *   接线表也可以在网页「上电诊断」里展开看（web_set_pins_text）。
 */
static void serial_block(const char *text)
{
    if (!text || !text[0]) {
        return;
    }
    fputs(text, stdout);
    size_t n = strlen(text);
    if (text[n - 1] != '\n') {
        fputc('\n', stdout);
    }
    fflush(stdout);
}

static void print_banner(void)
{
    const esp_app_desc_t *app = esp_app_get_description();
    const esp_partition_t *part = esp_ota_get_running_partition();
    esp_chip_info_t chip = {0};
    esp_chip_info(&chip);
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 0, 0)
    unsigned chip_major = (unsigned)(chip.revision / 100);
    unsigned chip_minor = (unsigned)(chip.revision % 100);
#else
    unsigned chip_major = (unsigned)chip.revision;
    unsigned chip_minor = 0;
#endif

    /* ===== 完整横幅：只给串口 ===== */
    printf("============================================================\n");
#if defined(CONFIG_IDF_TARGET_ESP32C3)
    printf("  ESP-AMS-C3  ——  拓竹打印机自动换料系统（ESP-IDF 版）\n");
    printf("  共享电机 + 2 路电磁离合 | 完全依赖 MQTT 事件 + 超时模式\n");
#else
    printf("  ESP-AMS-S3  ——  拓竹打印机自动换料系统（ESP-IDF 版）\n");
    printf("  共享电机 + %d 路电磁离合 | 每路 1 个 ADC 口读双微动 | MQTT/485 可选\n",
           BOARD_CHANNEL_COUNT);
#endif
    printf("============================================================\n");
    printf("  版本    : %s\n", app ? app->version : "?");
    printf("  编译    : %s %s\n", app ? app->date : "?", app ? app->time : "?");
    printf("  IDF     : %s\n", app ? app->idf_ver : "?");
    printf("  运行分区: %s\n", part ? part->label : "?");
    printf("  芯片    : %s rev%u.%u，%d 核 @ %dMHz\n",
           CONFIG_IDF_TARGET, chip_major, chip_minor, chip.cores,
           CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ);
    printf("  空闲内存: %u 字节\n", (unsigned)esp_get_free_heap_size());
    printf("  复位原因: %d\n", (int)esp_reset_reason());
    printf("============================================================\n");
    fflush(stdout);

    /* ===== 网页日志里只留一行摘要 ===== */
    ams_log("开机：%s · 分区 %s · 空闲 %uB · 复位原因 %d（详细横幅见串口）",
            CONFIG_IDF_TARGET,
            part ? part->label : "?",
            (unsigned)esp_get_free_heap_size(),
            (int)esp_reset_reason());
}

/**
 * 打印接线表 + 跑引脚自检。
 *
 * 这一段是从 MicroPython 版搬过来的，因为它的价值被反复验证过：
 * 用户第一次接完线通电，最需要的不是"启动成功"，而是
 * **"我接的到底对不对、哪只脚接到了哪儿"**。
 *
 * ★ 接线表走串口 + 网页「上电诊断」；网页日志里只留一行"自检通过"。
 *   （原来这一句在 main.c 和 web_server.c 里**各打了一遍**，
 *     每次开机都能看到两条一模一样的"引脚配置自检通过"。）
 */
static bool print_wiring_and_check(void)
{
    /* ★ 400 → 静态 1200 → 1400：8 通道的接线表约 900 字节（中文 UTF-8 一字
     *   3 字节），400 会从第 3 路开始截断；2026-10-22 又加了「电流采样」一行
     *   和「ADC2 微动」告警行，1200 也贴边了。
     *   而 C3 的 main 任务栈只有 4KB，放 1.4KB 的局部数组太奢侈 → 用静态缓冲
     *   （BSS）。本函数只在开机跑一次，无并发。 */
    static char buf[1400];
    board_pins_describe(buf, sizeof(buf));
    serial_block(buf);
    web_set_pins_text(buf);      /* 网页「上电诊断」里可展开看 */

    char err[160] = {0};
    if (board_pins_validate(err, sizeof(err))) {
        ams_log("接线自检通过：全部输出脚都在安全引脚上"
                "（完整接线表见串口，或网页「上电诊断」）");
        return true;
    }

    /* ★ 自检**未通过**是必须让人看见的：保持进网页日志，且多条一起打。
     *   这不是噪音，是"电机一直叫 / 反复重启"的根因提示。 */
    ams_log_err("引脚配置自检未通过：%s", err);
    ams_log_err("请修改 main/board_pins.h 后重新编译。");
    ams_log_err("（Strapping 脚当输出用会导致上电启动模式被改掉，");
    ams_log_err("  典型现象是反复重启、网页打不开、电机一直叫。）");
    /* ★ 只报警不中断：让用户能连上网页、看到这条错误、再决定怎么改。
     *   直接停机反而让人无从下手（MicroPython 版就是这么做的）。 */
    return false;
}

/**
 * AUTO 模式：开机先试 485 的等待窗口。
 *
 * 120 秒 = 2 分钟（现场要求）。打印机一上电就每 ~200ms 广播一次在线检测，
 * 所以真的接上了的话 1 秒内就能收到帧；2 分钟是留给"打印机还在开机/自检"
 * 这种情形的余量。
 */
#define COMM_AUTO_PROBE_MS   120000

/* ==========================================================================
 * 一·五、升级确认（可能被提前调用，所以做成幂等）
 * ==========================================================================
 *
 * sdkconfig 里开了 CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE：新固件起来后必须
 * 主动"确认有效"，否则下次重启引导程序就切回上一版 —— 现象是"升级成功了，
 * 但一重启又变回旧版本"，非常难查。
 *
 * ★ 为什么允许"提前"确认：AUTO 模式开机要先花最多 2 分钟等 485 打印机，
 *   而原来那句确认在最后（第 8 步）。如果用户在这 2 分钟内断电，就从来没
 *   确认过 → 每次都被回滚，表现为"刷完不进系统"。所以 AUTO 模式在探测前
 *   先确认一次：能跑到这里，电机/离合/微动/AMS 全都初始化完了，固件本身
 *   显然是健康的，网页那条判据在这里不适用（485 模式下压根没有网页）。
 */
static bool     s_ota_confirmed;
static bool     s_ota_failed;

static esp_err_t ota_confirm(const char *why)
{
    if (s_ota_confirmed) {
        return ESP_OK;
    }
    esp_err_t e = esp_ota_mark_app_valid_cancel_rollback();
    if (e == ESP_OK) {
        s_ota_confirmed = true;
        ams_log("固件体检通过（%s），已确认本次升级有效（不会再回滚）", why);
    } else if (!s_ota_failed) {
        s_ota_failed = true;      /* 只报一次，别在 2 分钟窗口里刷屏 */
        ams_log_err("确认升级失败: %s（下次重启可能会回滚）",
                    esp_err_to_name(e));
    }
    return e;
}

/* ==========================================================================
 * 二、网络
 * ========================================================================== */
static void start_network(void)
{
    esp_err_t err = wifi_mgr_init();
    if (err != ESP_OK) {
        ams_log_err("WiFi 初始化失败：%s —— 网页将无法访问",
                    esp_err_to_name(err));
        return;
    }

    /* ---- 先试直连（不扫描） ---- */
    bool connected = wifi_mgr_auto_connect(BOOT_WIFI_TIMEOUT_MS);

    if (connected) {
        ams_log("========================================================");
        ams_log("已联网: %s   IP = %s   信号 %d dBm",
                wifi_mgr_current_ssid(), wifi_mgr_sta_ip(), wifi_mgr_rssi());
        ams_log("同一网络下用浏览器打开  http://%s", wifi_mgr_sta_ip());
        ams_log("或者（支持 mDNS 的设备）  http://ams.local");
        ams_log("========================================================");
        /* ★ 连上路由器时**不开**热点。
         *   单射频下 AP+STA 并存要分时复用，会明显拖慢吞吐 ——
         *   而这个网页有 20KB 上下，快慢差别是能感觉到的。
         *   需要重新配网时，在网页上点「打开配置热点」即可（ESP-IDF 下
         *   随时开关都安全，不像 MicroPython 那版在碎堆上开热点会直接复位）。 */
        return;
    }

    /* ---- 直连失败 → 转配网模式 ---- */
    ams_log_warn("没有可用的 WiFi 记录或全部连不上，转入配网模式");
    if (wifi_mgr_ap_start() == ESP_OK) {
        ams_log("========================================================");
        ams_log("已打开配置热点: %s", config_get()->ap_ssid);
        ams_log("手机连上这个热点，浏览器打开 http://%s",
                wifi_mgr_ap_ip()[0] ? wifi_mgr_ap_ip() : "192.168.4.1");
        ams_log("========================================================");
    } else {
        ams_log_err("配置热点也起不来，设备将处于离线状态");
    }
}

/* ==========================================================================
 * 三、主函数
 * ========================================================================== */

void app_main(void)
{
    /* ---- 0. NVS ---- */
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        /* NVS 分区写满或版本升级：擦掉重建。这是 ESP-IDF 的标准处理，
         * 代价是配置回到默认值（会在日志里说明） */
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        /* NVS 起不来不直接 abort —— 继续跑，config 会用一份内存里的默认值，
         * 至少用户还能连上网页看到"配置存不下来"这件事 */
        printf("[ams] NVS 初始化失败: %s\n", esp_err_to_name(err));
    }

    /* ---- 1. 日志缓冲（越早越好） ---- */
    ams_log_init();

    /* ---- 2. 配置 ---- */
    config_init();

    /* ---- 3. 横幅 + 接线表 + 引脚自检 ----
     * 详细内容走串口，网页日志里只留"开机"一行 + "接线自检"一行。 */
    print_banner();
    print_wiring_and_check();

    {
        /* 业务配置（WiFi 记录 / 打印机 / 通道映射 / 参数）只打串口 ——
         * 这些网页「设置」面板里全都能看到，塞进 48 行的日志缓冲纯属
         * 把换料日志挤出去。 */
        char desc[640];
        config_describe(desc, sizeof(desc));
        serial_block(desc);
    }

    /* ---- 4. 执行机构（顺序见文件头说明） ---- */
    if (motor_init() != ESP_OK) {
        ams_log_err("电机初始化失败，手动与自动送料都将不可用");
    }
    if (clutch_init() != ESP_OK) {
        ams_log_err("电磁离合初始化失败 —— 出于安全考虑，"
                    "后续所有送料动作都会被拒绝");
    } else {
        /* 开机先确保全断开。这一句是"绝不带着残留状态起跑"的落实点。 */
        clutch_release_all_settled();
    }
    if (sensor_init() != ESP_OK) {
        ams_log_err("微动初始化失败，各通道将走降级模式（按时间推进送料）");
    }

    /* ---- 4.5 电机电流采样（ISEN）+ 堵转保护 ----
     * 放在电机/离合之后：它的保护动作要调 motor_stop() 和 clutch_release_all()，
     * 这两个模块必须已经初始化完。失败不影响送料，只是没有电流显示和保护。 */
#if BOARD_CURRENT_SENSE_ENABLED
    if (current_sense_init() != ESP_OK) {
        ams_log_err("电流采样初始化失败 —— 电流不显示、堵转保护不可用，"
                    "送料功能不受影响");
    }
#endif

    /* ---- 5. AMS 控制器 ----
     * ams_init() 内部会建 MQTT 客户端（打印机参数没配全时只是建而不连，
     * 这是正常状态）。esp-mqtt 每 5 秒自己重连一次，所以这时候 WiFi
     * 还没起来也没关系。 */
    if (ams_init() != ESP_OK) {
        ams_log_err("AMS 控制器初始化失败");
    } else {
        if (ams_start() != ESP_OK) {
            ams_log_err("AMS 任务启动失败");
        } else {
            ams_log("AMS 任务已启动");
        }
    }

    /* ---- 6. 通信方式 + 网络 ----
     *
     * ★ 为什么 MQTT 与 485 必须二选一：
     *   两条通路都能表达"打印机要换料"，同时开着就等于**两份**指令抢同一个
     *   AMS 状态机 —— 一条说"送料"、另一条说"退料"，电机听谁的？
     *   所以模式在启动时定死，切换只写配置、重启生效。
     *
     * ★★ 2026-10-22 新增 AUTO（默认）：先试 485 且**不开 WiFi**。
     *   原因是硬的 —— ADC2（GPIO11~20）和 WiFi 共用射频前端，WiFi 一开
     *   ADC2 就采不到值，微动 6/7/8 那三路直接降级。而 485 直连是"生产形态",
     *   本来就不需要网络。所以顺序必须是：
     *        先 485 探测（WiFi 关着）→ 成功就保持；失败才开 WiFi 走 MQTT。
     *   把 start_network() 放在探测**之后**，就是这个道理。 */
    uint8_t comm = config_get_comm_mode();
    bool need_wifi = true;
    bool need_mqtt = false;

    if (comm == COMM_MODE_AUTO) {
        /* 2 分钟窗口内先确认升级，见 ota_confirm() 的说明 */
        ota_confirm("机构、微动、AMS 均已初始化");

        /* ★★ 2026-10-23：「WiFi 上电开关」优先于原有的"485 通了就关 WiFi" ★★
         *
         * 原设计（上电探 485，通了就全程不开 WiFi）是为了保住 ADC2
         * （微动 6/7/8 = GPIO11/12/13）。可它有个致命副作用：
         * 打印机的总线枚举**只在上电后那一小会儿**做完，等 2 分钟超时
         * WiFi 起来时握手早结束了 —— 现场根本看不到任何 485 通信，
         * "刷了固件到底有没有效"无法验证。
         *
         * 所以默认改成：**WiFi 上电就开、485 同时跑、但不启 MQTT**
         * （MQTT 与 485 同时投命令会打架）。链路调通之后，在网页上把
         * 「WiFi 上电」关掉，就回到原来的省电/保住 ADC2 逻辑。
         */
        bool wifi_on = config_get_wifi_on();
        esp_err_t e485 = comm_485_init();

        if (wifi_on) {
            ams_log("通信方式：自动（485 优先）；WiFi 上电开关 = 开");
            ams_log("  → 485 与 WiFi 同时运行，网页可实时看 /bus485raw");
            ams_log_warn("  → 代价：WiFi 占用 ADC2，微动 6/7/8 读不到");
            ams_log("  → 调通后在网页把「WiFi 上电」关掉即可恢复那三路");
            if (e485 != ESP_OK) {
                ams_log_err("RS-485 启动失败（%s）—— 打印机将认为 AMS 离线",
                            esp_err_to_name(e485));
            }
            /* 不启 MQTT；need_wifi 保持 true，由下面 start_network() 拉起 */
        } else {
            ams_log("通信方式：自动 —— 先试 RS-485，最多等 %d 秒；"
                    "这期间**不开 WiFi**（WiFi 上电开关 = 关）",
                    (int)(COMM_AUTO_PROBE_MS / 1000));

            if (e485 == ESP_OK && comm_485_wait_printer(COMM_AUTO_PROBE_MS)) {
                need_wifi = false;
                ams_log("485：打印机在线（已收 %lu 帧）→ 保持 RS-485，"
                        "WiFi 本次不启动",
                        (unsigned long)comm_485_rx_frames());
                ams_log("  ⇒ 8 路微动全部可读（ADC2 没被射频占用）");
                ams_log("  ⇒ 网页本次不可用；要看网页：断电，拔掉 485 的 A/B "
                        "（或关掉打印机）后重启，2 分钟超时会自动开 WiFi");
            } else {
                if (e485 == ESP_OK) {
                    ams_log_warn("485：等了 %d 秒没收到打印机任何一帧 → 放弃 485",
                                 (int)(COMM_AUTO_PROBE_MS / 1000));
                    comm_485_deinit();
                } else {
                    ams_log_warn("485：启动失败（%s）→ 放弃 485",
                                 esp_err_to_name(e485));
                }
                ams_log_warn("改为开启 WiFi + MQTT（打印机 %s）",
                             config_get()->mqtt_host);
                need_mqtt = true;
            }
        }
    } else if (comm == COMM_MODE_485_DBG) {
        /* ★ 调试专用：485 与 WiFi **同时**跑，且不起 MQTT。
         *
         * 为什么需要它：AUTO 模式为了保住 ADC2（微动 6/7/8）开机不开 WiFi，
         * 可打印机的总线枚举**只在上电后那一小会儿**做，等 2 分钟后 WiFi
         * 起来，握手早就结束了 —— 现场等于看不到任何 485 通信。
         * 这个模式把 WiFi 一开始就拉起来，网页能实时看 `/bus485raw`。
         * 代价明确：微动 6/7/8（GPIO11/12/13，ADC2）读不到。
         * **不启 MQTT**：两条通路同时下指令会互相打架。 */
        ams_log("通信方式：RS-485 + WiFi 同时（调试）—— WiFi 开机即起，"
                "网页可以看 485 全过程");
        ams_log_warn("  代价：WiFi 占用 ADC2，微动 6/7/8 读不到（仅调试用）");
        if (comm_485_init() != ESP_OK) {
            ams_log_err("RS-485 启动失败 —— 打印机将认为 AMS 离线");
        }
    } else if (comm == COMM_MODE_485) {
        ams_log("通信方式：RS-485 直连打印机（MQTT **不启动**）");
        ams_log("  WiFi 保持开启 —— 网页可用；代价是微动 6/7/8（ADC2）"
                "读不到，那三路降级");
        if (comm_485_init() != ESP_OK) {
            ams_log_err("RS-485 启动失败 —— 打印机将认为 AMS 离线；"
                        "需要退回 MQTT 请在网页「硬件调试」里改通信方式后重启");
        }
    } else {
        need_mqtt = true;
    }

    if (need_wifi) {
        start_network();
    } else {
        ams_log("WiFi 未启动（485 直连模式）：省电，且 ADC2 保持可用");
    }

    if (need_mqtt) {
        ams_log("通信方式：MQTT（打印机 %s）", config_get()->mqtt_host);
        ams_connect_printer();
    }

    /* ---- 7. Web 服务 ---- */
    bool web_ok = (web_server_start() == ESP_OK);

    /* ---- 8. 升级体检 ----
     * ★ 这一句不能省。sdkconfig 里开了
     *   CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE，新固件启动后如果不主动
     *   "确认有效"，下一次重启引导程序就认为它有问题、自动切回上一版 ——
     *   现象是"升级成功了，但一重启又变回旧版本"，非常难查。
     *
     *   放在这里是刻意的：只有 WiFi/网页这条链路真的起来了才确认。
     *   如果新固件把网页写坏了，就不会确认 → 下次重启自动回滚。
     *   这正是双分区 OTA 最值钱的那个特性。 */
    if (web_ok) {
        ota_confirm("Web 服务已启动");
    } else if (!s_ota_confirmed) {
        ams_log_err("Web 服务启动失败 —— **不**确认本次升级，"
                    "重启后将自动回滚到上一版固件");
    } else {
        ams_log_warn("Web 服务启动失败（升级已在探测前确认过，不会回滚）");
    }

    /* ---- 9. 收尾 ---- */
    if (web_ok) {
        const char *ip = wifi_mgr_sta_ip();
        if (ip[0]) {
            ams_log("连上同一网络后用浏览器访问 http://%s  （或 http://ams.local）", ip);
        } else if (wifi_mgr_ap_is_on()) {
            ams_log("连上热点 %s 后用浏览器访问 http://%s",
                    config_get()->ap_ssid,
                    wifi_mgr_ap_ip()[0] ? wifi_mgr_ap_ip() : "192.168.4.1");
        }
    }

    ams_log("启动完成，用时 %d ms。AMS 已在后台运行，"
            "微动触发、打印机换料请求、网页操作都会即时响应。",
            (int)(esp_timer_get_time() / 1000));

    /* 注意：app_main 返回后 FreeRTOS 会回收这个任务的栈，
     * 所有常驻工作都在各自的 FreeRTOS 任务里：
     *   ams_task      —— 换料状态机、微动轮询、状态灯
     *   httpd         —— Web 服务
     *   mqtt_task     —— 打印机通信（esp-mqtt 自带）
     *   esp_timer     —— 微动去抖（5ms 周期）
     * 诊断期间 ams_task 每 100 轮会打一条心跳日志，可以判断它是否活着。
     * 如果 ams_task 卡死（比如卡在 motor_run 或 sensor 查询），
     * 网页仍然可以访问，但看不到任何换料动作。
     */
}
