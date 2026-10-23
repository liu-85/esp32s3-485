/**
 * web_server.h —— 配置网页与 REST 接口
 * ============================================================================
 *
 * 对应 MicroPython 版的 AMS_WEB.py。**接口路径和 JSON 字段名尽量保持不变**，
 * 这样原来那个 1390 行的网页几乎可以照用（只多加了新传感器的几块面板）。
 *
 * ----------------------------------------------------------------------------
 * ★ 换到 ESP-IDF 之后，这个文件比 Python 版短了一大半
 * ----------------------------------------------------------------------------
 * Python 版 AMS_WEB.py 有 1400 多行，其中很大一部分是在**手搓一个 HTTP 服务**：
 *
 *   · 自己 accept / 自己解析请求行和请求头
 *   · 自己判断 Content-Length、自己处理 chunked 之外的各种边界
 *   · 自己做一条"分块发送"的路径，因为 C3 只有 60KB 空闲堆，不能把
 *     65KB 的 index.html 一次性读进内存（为此专门写了 send_file 分段发）
 *   · 自己维护 3 个 worker 轮转、自己实现 keep-alive 超时
 *   · 最要命的是：这一切都要写成非阻塞的，否则 uasyncio 事件循环被按住，
 *     网页就卡死（这就是"加了 OTA 之后页面彻底打不开"的源头）
 *
 * ESP-IDF 的 esp_http_server 把这些全包了：它跑在自己的任务里、自带
 * keep-alive、自带分块发送、自带 URI 路由。所以我们只需给每个接口写一个
 * 几十行的 handler，**并且可以放心地写阻塞代码**（比如扫描 WiFi 那 2 秒）。
 *
 * ----------------------------------------------------------------------------
 * 接口一览
 * ----------------------------------------------------------------------------
 *   GET  /                      首页（index.html，已编译进固件）
 *   GET  /app.js                前端脚本
 *   GET  /style.css             前端样式
 *   GET  /status                ★ 总状态（2 秒轮询一次，前端所有面板都靠它）
 *   GET  /log                   最近的设备日志（对应 logout.py 的环形缓冲）
 *   GET  /get_mqtt_info         读回打印机参数（注意：**不回传访问码**）
 *
 *   POST /wifi_scan             扫描附近 WiFi（阻塞 1.5~3 秒）
 *   POST /wifi_connect          连接指定 WiFi
 *   POST /mqtt_connect          保存打印机参数并重连
 *   POST /access_set            保存通道映射与颜色
 *   POST /hardware_test         单通道点动（网页"硬件调试"面板）
 *   POST /jog_set               {"seconds":1.0,"speed_pct":100}
 *                               改「进退响应时间」**和手动点动 PWM**。
 *                               两个都可选：只传一个也行，没传的保持原值。
 *                               PWM 是现场用来一档一档试"多少占空比能带动
 *                               电机和离合"的量法（2026-09-23 现场要求）。
 *   POST /ap_set                开 / 关配置热点
 *   POST /ota_upload            ★ 上传整机固件并升级（真正的双分区 OTA）
 *
 *   ---- 以下是这次新增的（对应新加的微动流程）----
 *   POST /sensor_set            逐路开关「缓冲送料」功能
 *                                body: {channel, enabled}
 *   POST /creep_set             改换料收尾的蠕动参数（次数/时长/速度）
 *   POST /extruder_src_set      切换「挤出机到位信号」来源（GPIO / MQTT）
 *   POST /stop                  紧急停止（停电机 + 断开全部离合）
 *
 *   ---- 现场可调项（2026-09-22：把写死的时序搬到网页上）----
 *   POST /assist_set            {"enabled","speed_pct","ms","hold"}
 *                                辅助送料的占空比与**时长**（时长以前写死在代码里）。
 *                                重复周期跟着时长走、没有单独参数：
 *                                校准中（stg=8/19）周期 = 时长 + 0.5s，
 *                                打印中（stg=0）周期固定 5s —— 见 ams_controller.c
 *                                里 ASSIST_CAL_GAP_MS 的注释（为什么要 0.5 秒）。
 *                                hold（2026-09-23 新增）：1 = 辅助阶段内离合
 *                                保持吸合、只脉冲电机；0 = 老行为（每次吸合-断开）。
 *                                注意 hold=0 是合法值，判"没传"用的是 -1。
 *   POST /retract_set           {"wait_ms","cont_ms","creep_ms","gap_ms","creep_max"}
 *                                退料四段时长：连续拉料上限 / 蠕动单次 / 蠕动间隔 /
 *                                蠕动轮数（0 = 只做连续退料）。没传的字段保持原值。
 *
 * ----------------------------------------------------------------------------
 * ★ 关于 OTA：为什么这次能升级"整机固件"
 * ----------------------------------------------------------------------------
 * MicroPython 版的分区表只有单个 factory 应用分区，没有备用分区，所以只能
 * 更新"应用与界面"（.ams 包），整机固件必须插 USB。
 *
 * ESP-IDF 这版有 ota_0 / ota_1 两块应用分区（见 partitions.csv）：
 *   ① 新固件写到**当前没在跑的那一块**
 *   ② 校验通过后改 otadata，标记"下次从这一块启动"
 *   ③ 重启 → 新固件生效
 * 写入过程中断电，旧分区依然完好，设备照常启动 —— 这是原子升级。
 * 而且第一次启动新固件如果崩溃，引导程序会自动回滚到上一版。
 */

#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 监听端口。和 MicroPython 版一致，方便老用户直接访问。 */
#define WEB_SERVER_PORT 80

/**
 * 启动 Web 服务。
 *
 * 必须在 wifi_mgr_init() 之后调用（它要在 netif 上注册），
 * 但**不要求** WiFi 已经连上 —— 热点模式下也能用（这正是配网的入口）。
 */
esp_err_t web_server_start(void);

/** 停掉 Web 服务（恢复出厂 / 重启前收尾用） */
void web_server_stop(void);

/** 服务是否在跑 */
bool web_server_is_running(void);

/**
 * 累计处理过多少次请求（诊断用）。
 * 顺手给个"服务是不是活着"的判据 —— 打不开网页时先看这个数有没有涨。
 */
uint32_t web_server_request_count(void);

/**
 * 把**完整接线表**（多行文本）交给网页，让「上电诊断」面板能展开查看。
 *
 * 由 main.c 在启动时调一次（main 里的 print_wiring_and_check）。
 * 存在的意义：接线表这段内容只在"第一次接线上电"时有用，但它是多行的，
 * 塞进 48 行的日志环形缓冲会把换料日志挤出去，所以日志里只留一行摘要，
 * 完整内容搬到网页 + 串口。
 *
 * 必须在 web_server_start() 之前调；start() 里不会再覆盖它。
 */
void web_set_pins_text(const char *text);

#ifdef __cplusplus
}
#endif

#endif /* WEB_SERVER_H */
