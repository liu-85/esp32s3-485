/**
 * bambu_mqtt.h —— 与拓竹打印机通信（MQTT over TLS）
 * ============================================================================
 *
 * 对应 MicroPython 版的 bambu/bambu_mqtt.py。协议内容在 bambu_proto.c，
 * 这里只管"连上、收、发、断了重连"。
 *
 * ----------------------------------------------------------------------------
 * ★ 为什么文件叫 bambu_mqtt 而不是 mqtt_client ★
 * ----------------------------------------------------------------------------
 * ESP-IDF 自带的 mqtt 组件里已经有一个 `mqtt_client.h` 了。我们这边如果也叫
 * 这个名字，`#include "mqtt_client.h"` 会先命中本目录的同名文件，
 * 把 ESP-IDF 的头文件**遮住**，于是 esp_mqtt_client_init 这些全都找不到，
 * 编译直接失败。
 *
 * 所以这里叫 bambu_mqtt —— 顺带还和原来的 bambu/bambu_mqtt.py 对上了，
 * 命名反而更一致。
 *
 * ----------------------------------------------------------------------------
 * ★ 换成 ESP-IDF 之后，这里能省掉 Python 版一半的代码
 * ----------------------------------------------------------------------------
 * Python 版为了"网页不能卡"写了很多防御：
 *   · poll_msg()          —— 非阻塞收包，避免 wait_msg() 按住事件循环
 *   · mqtt_alive_cached() —— 网页只读缓存，不真的发 PINGREQ
 *   · tcp_reachable()     —— 连接前先用普通 socket 探 1 秒，避免 SSL 卡几十秒
 *   · _ping_guarded()     —— 给 ping 加 0.8 秒硬超时
 *
 * 这些在 ESP-IDF 下**全都不需要**，因为 esp-mqtt 跑在自己的任务里：
 *   · 收包是回调，天然不阻塞别的任务
 *   · 断线重连由 esp-mqtt 自己管（reconnect_timeout_ms）
 *   · 连接超时由 network.timeout_ms 控制
 *   · bambu_mqtt_is_connected() 只是读一个内部标志，一次网络 I/O 都没有，
 *     所以网页可以随便调
 *
 * 也就是说："网页因为 MQTT 而卡死"这一整类问题，从架构上消失了。
 *
 * ----------------------------------------------------------------------------
 * 拓竹的 MQTT 约定
 * ----------------------------------------------------------------------------
 *   端口     8883（TLS）
 *   用户名   bblp
 *   密码     打印机「设置 → 网络 → 访问码」
 *   订阅     device/{序列号}/report
 *   发布     device/{序列号}/request
 *   证书     自签，不可能校验通过 —— 官方做法就是跳过校验
 *
 *   跳过的是"证明对端是拓竹打印机"，**不影响链路加密**：数据和 Python 版
 *   一样是 TLS 加密传输的。
 */

#ifndef BAMBU_MQTT_H
#define BAMBU_MQTT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"
#include "bambu_proto.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 回调
 * ========================================================================== */

/**
 * 每收到一条**含 print 段**的状态上报就回调一次。
 *
 * ⚠️ 这个回调运行在 esp-mqtt 的任务里，**不要**在里面做耗时操作
 *    （送料、阻塞等状态、写 NVS）。正确做法是把 report 丢进一个队列，
 *    让 ams_controller 的任务去处理 —— 本项目就是这么做的。
 *
 * @param report 已解析好的结构体；回调返回后其内容即失效，
 *               需要保留的请自己拷贝（stage_text 指向静态表，可以直接存）
 */
typedef void (*bambu_mqtt_report_cb_t)(const bambu_report_t *report,
                                       void *user);

/* ==========================================================================
 * 生命周期
 * ========================================================================== */

/**
 * 初始化 MQTT 客户端。
 *
 * @param cb    收到状态上报时的回调（可以为 NULL，只用来看连接状态）
 * @param user  回调时原样传回的指针
 *
 * 如果 config 里打印机参数不完整，**不会**返回错误 —— 会把客户端建起来但
 * 不连接，等用户在网页上填完参数再调 bambu_mqtt_reconfigure() 连上去。
 * 这样开机时"还没配打印机"是正常状态，不是失败。
 */
esp_err_t bambu_mqtt_init(bambu_mqtt_report_cb_t cb, void *user);

/** 停止并销毁客户端（恢复出厂 / 重启前收尾用） */
void bambu_mqtt_deinit(void);

/**
 * 用 config 里的最新参数**重建**连接。
 *
 * 网页上改完打印机参数后调它 —— 不用让用户去重启设备。
 * 内部会先销毁旧客户端再建新的，所以旧 socket 不会泄漏
 * （Python 版为此专门写了 close_client()，因为 umqtt 的对象是直接覆盖的）。
 */
esp_err_t bambu_mqtt_reconfigure(void);

/* ==========================================================================
 * 状态查询（全部是纯读操作，不做任何网络 I/O，网页可以随便调）
 * ========================================================================== */

/** 打印机参数是否已配全（IP + 序列号 + 访问码） */
bool bambu_mqtt_is_configured(void);

/** 当前是否已连接 */
bool bambu_mqtt_is_connected(void);

/** 当前配置里的打印机地址；未配置返回 "" */
const char *bambu_mqtt_host(void);

/** 当前配置里的打印机序列号；未配置返回 "" */
const char *bambu_mqtt_serial(void);

/** 累计收到多少条状态上报 */
uint32_t bambu_mqtt_rx_count(void);

/** 距离上一次收到上报过了多少毫秒；从未收到返回 UINT32_MAX */
uint32_t bambu_mqtt_ms_since_last_rx(void);

/** 最近一次连接失败的原因（人类可读）；没失败过返回 "" */
const char *bambu_mqtt_last_error(void);

/* ==========================================================================
 * 发送
 * ========================================================================== */

/** 往 device/{serial}/request 发一段内容。返回 >0 表示已交给驱动 */
int bambu_mqtt_publish(const char *payload, int len);

/** 发一条 G-code（自动套 gcode_line 外壳） */
int bambu_mqtt_send_gcode(const char *gcode);

/** 让打印机继续打印（换料完成后调） */
int bambu_mqtt_send_resume(void);

/** 开始定时推送状态（连上后调一次，之后打印机会主动上报） */
int bambu_mqtt_send_push_start(void);

/** 查询一次全量状态 */
int bambu_mqtt_send_pushall(void);

#ifdef __cplusplus
}
#endif

#endif /* BAMBU_MQTT_H */
