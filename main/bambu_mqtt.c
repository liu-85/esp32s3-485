/**
 * bambu_mqtt.c —— 拓竹打印机 MQTT 通信的实现
 * ============================================================================
 *
 * 用 ESP-IDF 的 esp-mqtt（组件名 mqtt）。它内部跑一个独立任务，所有事件都从
 * 那个任务的回调里出来，所以主循环永远不会被网络 I/O 按住 —— 这一点直接
 * 解决了 Python 版最难缠的一类 bug（详见头文件里的对比）。
 *
 * ----------------------------------------------------------------------------
 * 报文分片怎么处理
 * ----------------------------------------------------------------------------
 * 拓竹的状态上报（pushall / 定时 start）可能有 2~8KB，而 MQTT 的接收缓冲
 * 默认只有 2KB（CONFIG_MQTT_BUFFER_SIZE）。超过缓冲长度时 esp-mqtt 会把一条
 * 报文**拆成多次 MQTT_EVENT_DATA** 回调，每次给一段：
 *
 *     event->total_data_len      整条报文总长
 *     event->current_data_offset 这一段的起始偏移
 *     event->data_len            这一段的长度
 *
 * 所以必须自己拼。这里用一个 8KB 的静态缓冲累积，收满（offset + len == total）
 * 才去解析。缓冲区大小按"拓竹报文最大可能长度"留的余量，正常情况下报文
 * 只有 1~3KB，远用不到。
 *
 * ⚠️ 如果一条报文比缓冲还大，直接丢弃并打日志 —— 宁可这一条丢了，
 *    也不要越界写内存。
 */

#include "bambu_mqtt.h"

#include <stdio.h>
#include <string.h>

#include "esp_event.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
/* ★ 这是 ESP-IDF 自带的 esp-mqtt 头文件。本模块自己的头文件叫 bambu_mqtt.h，
 *   所以这里不会互相遮住（原因见 bambu_mqtt.h 顶部说明）。 */
#include "mqtt_client.h"

#include "config.h"
#include "log_buffer.h"

/* ==========================================================================
 * 常量
 * ========================================================================== */

/** 拼装缓冲区大小。拓竹的 pushall 报文实测 1~3KB，8KB 留了足够余量。 */
#define MQTT_RX_BUF_SIZE 8192

/** 主题字符串长度上限：device/ + 序列号 + /report */
#define MQTT_TOPIC_MAX 96

/** keepalive（秒）—— 和 Python 版一致 */
#define MQTT_KEEPALIVE_S 60

/* ==========================================================================
 * 内部状态
 * ========================================================================== */
static esp_mqtt_client_handle_t s_client;
static bambu_mqtt_report_cb_t   s_report_cb;
static void                    *s_report_user;

static volatile bool s_connected;
static char s_topic_report[MQTT_TOPIC_MAX];
static char s_topic_request[MQTT_TOPIC_MAX];
static char s_broker_uri[CONFIG_HOST_MAX + 16];
static char s_last_error[80];
static uint32_t s_rx_count;
static int64_t  s_last_rx_us = -1;

/* 报文拼装 */
static char   s_rx_buf[MQTT_RX_BUF_SIZE];
static size_t s_rx_len;
static size_t s_rx_expected;
static bool   s_rx_overflow;

static void rx_reset(void)
{
    s_rx_len = 0;
    s_rx_expected = 0;
    s_rx_overflow = false;
}

/* ==========================================================================
 * 事件处理：跑在 esp-mqtt 自己的任务里
 * ========================================================================== */

static void mqtt_event_handler(void *args, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    (void)args;
    (void)base;
    esp_mqtt_event_handle_t event = (esp_mqtt_event_handle_t)event_data;

    switch ((esp_mqtt_event_id_t)event_id) {

    case MQTT_EVENT_CONNECTED:
        s_connected = true;
        s_last_error[0] = '\0';
        ams_log("已连接打印机 MQTT: %s", bambu_mqtt_host());

        /* 订阅状态上报主题 */
        if (esp_mqtt_client_subscribe(s_client, s_topic_report, 0) < 0) {
            ams_log_err("订阅 %s 失败", s_topic_report);
        } else {
            ams_log("已订阅: %s", s_topic_report);
        }

        /* 订阅成功后主动要一次全量状态，这样网页上马上就有数据，
         * 不用等打印机的下一个定时上报周期 */
        bambu_mqtt_send_push_start();
        bambu_mqtt_send_pushall();
        break;

    case MQTT_EVENT_DISCONNECTED:
        s_connected = false;
        rx_reset();
        ams_log_warn("与打印机的连接已断开，5 秒后自动重连");
        break;

    case MQTT_EVENT_DATA: {
        /* ---- 只处理 report 主题 ---- */
        if (event->topic_len > 0) {
            size_t want = strlen(s_topic_report);
            if ((size_t)event->topic_len != want ||
                strncmp(event->topic, s_topic_report, want) != 0) {
                return;   /* 别的主题（理论上不会有），忽略 */
            }
        }

        /* ---- 分片拼装 ---- */
        size_t total = (event->total_data_len > 0)
                           ? (size_t)event->total_data_len
                           : (size_t)event->data_len;
        size_t offset = (size_t)event->current_data_offset;

        if (offset == 0) {
            rx_reset();
            if (total > MQTT_RX_BUF_SIZE) {
                s_rx_overflow = true;
                ams_log_err("打印机报文 %u 字节，超过拼装缓冲 %d 字节，本条丢弃",
                            (unsigned)total, MQTT_RX_BUF_SIZE);
                return;
            }
            s_rx_expected = total;
        }

        if (s_rx_overflow) {
            return;
        }
        if (offset + (size_t)event->data_len > MQTT_RX_BUF_SIZE) {
            s_rx_overflow = true;
            ams_log_err("报文偏移越界（offset=%u len=%d），本条丢弃",
                        (unsigned)offset, (int)event->data_len);
            return;
        }

        memcpy(s_rx_buf + offset, event->data, (size_t)event->data_len);
        s_rx_len = offset + (size_t)event->data_len;

        /* 还没收全就等下一片 */
        if (s_rx_len < s_rx_expected) {
            return;
        }

        /* ---- 收全了，解析 ---- */
        s_rx_buf[s_rx_len] = '\0';
        s_rx_count++;
        s_last_rx_us = esp_timer_get_time();

        bambu_report_t report;
        esp_err_t err = bambu_proto_parse(s_rx_buf, s_rx_len, &report);
        if (err != ESP_OK) {
            /* 不是错误：打印机偶尔会发心跳类报文，里面没有 print 段 */
            rx_reset();
            return;
        }

        if (s_report_cb) {
            s_report_cb(&report, s_report_user);
        }
        rx_reset();
        break;
    }

    case MQTT_EVENT_ERROR: {
        const char *kind = "未知错误";
        if (event->error_handle) {
            if (event->error_handle->error_type ==
                MQTT_ERROR_TYPE_TCP_TRANSPORT) {
                kind = "TCP/TLS 传输失败（打印机没开机或不在同一网段？）";
            } else if (event->error_handle->error_type ==
                       MQTT_ERROR_TYPE_CONNECTION_REFUSED) {
                kind = "连接被拒绝（用户名 / 访问码不对？）";
            }
        }
        snprintf(s_last_error, sizeof(s_last_error), "%s", kind);
        ams_log_err("打印机 MQTT 出错: %s", kind);
        break;
    }

    case MQTT_EVENT_BEFORE_CONNECT:
        /* 每次重连尝试都会走到这里。不打印 —— 打印机没开机时这里会刷屏，
         * 反而把有用的日志挤掉。 */
        break;

    case MQTT_EVENT_SUBSCRIBED:
        ams_log("订阅已确认（msg_id=%d）", event->msg_id);
        break;

    default:
        break;
    }
}

/* ==========================================================================
 * 主题拼装
 * ========================================================================== */

static void build_topics(const ams_config_t *cfg)
{
    snprintf(s_topic_report, sizeof(s_topic_report),
             "device/%s/report", cfg->mqtt_serial);
    snprintf(s_topic_request, sizeof(s_topic_request),
             "device/%s/request", cfg->mqtt_serial);
    snprintf(s_broker_uri, sizeof(s_broker_uri),
             "mqtts://%s:%u", cfg->mqtt_host, (unsigned)cfg->mqtt_port);
}

/* ==========================================================================
 * 建立 / 重建客户端
 * ========================================================================== */

static void destroy_client(void)
{
    if (s_client) {
        esp_mqtt_client_stop(s_client);
        esp_mqtt_client_destroy(s_client);
        s_client = NULL;
    }
    s_connected = false;
    rx_reset();
}

static esp_err_t create_and_start(void)
{
    ams_config_t *cfg = config_get();

    if (!config_mqtt_ready()) {
        snprintf(s_last_error, sizeof(s_last_error), "%s",
                 "打印机参数未配置完整");
        return ESP_ERR_INVALID_STATE;
    }

    build_topics(cfg);

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker = {
            .address = {
                .uri = s_broker_uri,
            },
            /* ★ 拓竹用的是自签证书，不可能校验通过 —— 官方也认可跳过校验。
             *   跳过的是"证明对端是拓竹打印机"，不影响链路加密。 */
            .verification = {
                .certificate                 = NULL,
                .skip_cert_common_name_check = true,
                .use_global_ca_store         = false,
            },
        },
        .credentials = {
            .username  = cfg->mqtt_user,
            .client_id = cfg->mqtt_client_id,
            .authentication = {
                .password = cfg->mqtt_pass,
            },
        },
        .session = {
            .keepalive = MQTT_KEEPALIVE_S,
        },
        .network = {
            /* 连接与读写超时。打印机没开机时这个值决定了重试节奏：
             * 太长会让"打印机不在线"看起来像卡住，太短会频繁刷日志。
             * 5 秒是个平衡点。 */
            .timeout_ms           = 5000,
            .reconnect_timeout_ms = 5000,
        },
        .buffer = {
            /* 收发缓冲各 2KB（与 CONFIG_MQTT_BUFFER_SIZE 一致）。
             * 报文超过这个长度时会分片回调，由上面的拼装逻辑处理。 */
            .size     = 2048,
            .out_size = 2048,
        },
    };

    s_client = esp_mqtt_client_init(&mqtt_cfg);
    if (!s_client) {
        snprintf(s_last_error, sizeof(s_last_error), "%s",
                 "创建 MQTT 客户端失败（内存不足？）");
        ams_log_err("创建 MQTT 客户端失败（内存不足？）");
        return ESP_FAIL;
    }

    esp_err_t err = esp_mqtt_client_register_event(
        s_client, ESP_EVENT_ANY_ID, mqtt_event_handler, NULL);
    if (err != ESP_OK) {
        ams_log_err("注册 MQTT 事件处理器失败: %s", esp_err_to_name(err));
        destroy_client();
        return err;
    }

    err = esp_mqtt_client_start(s_client);
    if (err != ESP_OK) {
        ams_log_err("启动 MQTT 客户端失败: %s", esp_err_to_name(err));
        destroy_client();
        return err;
    }

    ams_log("打印机 MQTT 客户端已启动: %s（序列号 %s）",
            s_broker_uri, cfg->mqtt_serial);
    return ESP_OK;
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */

esp_err_t bambu_mqtt_init(bambu_mqtt_report_cb_t cb, void *user)
{
    s_report_cb = cb;
    s_report_user = user;
    rx_reset();

    if (!config_mqtt_ready()) {
        ams_log("打印机 MQTT 尚未配置完整（IP / 序列号 / 访问码），"
                "跳过自动连接；在网页「打印机配置」里填好后会立即连接");
        snprintf(s_last_error, sizeof(s_last_error), "%s",
                 "打印机参数未配置完整");
        return ESP_OK;   /* 不是错误，是正常状态 */
    }
    return create_and_start();
}

void bambu_mqtt_deinit(void)
{
    destroy_client();
    s_report_cb = NULL;
    s_report_user = NULL;
}

esp_err_t bambu_mqtt_reconfigure(void)
{
    ams_log("打印机参数已更新，重建 MQTT 连接");
    destroy_client();

    /* 给驱动一点时间把旧 socket 彻底放掉 */
    vTaskDelay(pdMS_TO_TICKS(100));

    if (!config_mqtt_ready()) {
        snprintf(s_last_error, sizeof(s_last_error), "%s",
                 "打印机参数未配置完整");
        return ESP_ERR_INVALID_STATE;
    }
    return create_and_start();
}

/* ==========================================================================
 * 状态查询
 * ========================================================================== */

bool bambu_mqtt_is_configured(void)
{
    return config_mqtt_ready();
}

bool bambu_mqtt_is_connected(void)
{
    /* ★ 纯读一个 bool，零网络 I/O。
     *   这正是 Python 版费了好大劲才做到的（那边要 mqtt_alive_cached()），
     *   在这里是天然的。网页 /api/status 可以放心地每秒调它。 */
    return s_connected && (s_client != NULL);
}

const char *bambu_mqtt_host(void)
{
    return config_get()->mqtt_host;
}

const char *bambu_mqtt_serial(void)
{
    return config_get()->mqtt_serial;
}

uint32_t bambu_mqtt_rx_count(void)
{
    return s_rx_count;
}

uint32_t bambu_mqtt_ms_since_last_rx(void)
{
    if (s_last_rx_us < 0) {
        return UINT32_MAX;
    }
    int64_t delta = esp_timer_get_time() - s_last_rx_us;
    if (delta < 0) {
        return 0;
    }
    return (uint32_t)(delta / 1000);
}

const char *bambu_mqtt_last_error(void)
{
    return s_last_error;
}

/* ==========================================================================
 * 发送
 * ========================================================================== */

int bambu_mqtt_publish(const char *payload, int len)
{
    if (!s_client || !s_connected || !payload || len <= 0) {
        return -1;
    }
    int msg_id = esp_mqtt_client_publish(s_client, s_topic_request,
                                         payload, len, 0, 0);
    if (msg_id < 0) {
        /* 发布失败最常见的原因是发送缓冲满（TCP 拥塞）。
         * 不报错 —— 主流程下一步还会再发，没必要把日志刷满。 */
        return -1;
    }
    return msg_id;
}

int bambu_mqtt_send_gcode(const char *gcode)
{
    char buf[768];
    int n = bambu_cmd_gcode(buf, sizeof(buf), gcode);
    if (n < 0) {
        ams_log_err("G-code 太长，发不出去（%u 字符）",
                    (unsigned)strlen(gcode));
        return -1;
    }
    return bambu_mqtt_publish(buf, n);
}

int bambu_mqtt_send_resume(void)
{
    char buf[160];
    int n = bambu_cmd_resume(buf, sizeof(buf));
    return n > 0 ? bambu_mqtt_publish(buf, n) : -1;
}

int bambu_mqtt_send_push_start(void)
{
    char buf[160];
    int n = bambu_cmd_push_start(buf, sizeof(buf));
    return n > 0 ? bambu_mqtt_publish(buf, n) : -1;
}

int bambu_mqtt_send_pushall(void)
{
    char buf[160];
    int n = bambu_cmd_pushall(buf, sizeof(buf));
    return n > 0 ? bambu_mqtt_publish(buf, n) : -1;
}
