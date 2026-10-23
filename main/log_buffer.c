/**
 * log_buffer.c —— 日志环形缓冲的实现
 * ============================================================================
 *
 * 一个很朴素的定长环形缓冲：
 *
 *     [0][1][2] ... [AMS_LOG_LINES-1]     头    ← 最旧
 *                                         尾    ← 最新
 *
 * 每格是 AMS_LOG_LINE_MAX 字节的字符数组，连行号一起存，方便网页端判断
 * "这一行是不是新来的"。写入时只动尾部一格，满了就整体前移一格（memmove）。
 *
 * 48 行 × 112 字节的 memmove 大约是 5KB 的内存搬运，在 240MHz 的 S3 上不到
 * 30 微秒 —— 比维护两个写指针、处理环绕读要简单得多，而且不会写错。
 * 日志本来就不是高频操作（换料、联网、网页请求才会写一条），这点开销无所谓。
 */

#include "log_buffer.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"   /* xTaskGetTickCount() —— 别依赖 semphr.h 间接带入 */
#include "esp_log.h"

static const char *TAG = "ams";

typedef struct {
    uint32_t seq;                        /* 递增行号，用于网页端判断新增 */
    char     text[AMS_LOG_LINE_MAX];
} log_line_t;

static log_line_t         s_lines[AMS_LOG_LINES];
static int                s_count;       /* 当前有效行数，<= AMS_LOG_LINES */
static uint32_t           s_seq;         /* 累计写过多少行 */
static SemaphoreHandle_t  s_lock;
static bool               s_inited;

void ams_log_init(void)
{
    if (s_inited) {
        return;
    }
    s_count = 0;
    s_seq = 0;
    memset(s_lines, 0, sizeof(s_lines));
    /* 用递归锁：日志函数可能被持锁路径间接调用，普通锁会自死锁 */
    s_lock = xSemaphoreCreateRecursiveMutex();
    s_inited = true;
}

void ams_log_clear(void)
{
    if (!s_inited) {
        return;
    }
    if (xSemaphoreTakeRecursive(s_lock, portMAX_DELAY) == pdTRUE) {
        s_count = 0;
        memset(s_lines, 0, sizeof(s_lines));
        xSemaphoreGiveRecursive(s_lock);
    }
}

int ams_log_count(void)
{
    if (!s_inited) {
        return 0;
    }
    int n = 0;
    if (xSemaphoreTakeRecursive(s_lock, portMAX_DELAY) == pdTRUE) {
        n = s_count;
        xSemaphoreGiveRecursive(s_lock);
    }
    return n;
}

int ams_log_recent_lines(char *buf, size_t buflen, int max_lines,
                         uint32_t *seqs, int seq_cap, int *out_count)
{
    if (out_count) {
        *out_count = 0;
    }
    if (!buf || buflen == 0) {
        return 0;
    }
    buf[0] = '\0';

    if (!s_inited) {
        return 0;
    }
    if (xSemaphoreTakeRecursive(s_lock, portMAX_DELAY) != pdTRUE) {
        return 0;
    }

    int take = s_count;
    if (max_lines > 0 && max_lines < take) {
        take = max_lines;
    }
    int start = s_count - take;   /* 从最新的 take 行里最早的那行开始 */

    size_t off = 0;
    int    n   = 0;
    for (int i = start; i < s_count; i++) {
        const char *line = s_lines[i].text;
        size_t len = strlen(line);
        if (off + len + 2 >= buflen) {   /* +2 = 换行 + 结尾 '\0' */
            break;
        }
        memcpy(buf + off, line, len);
        off += len;
        buf[off++] = '\n';

        /* 同步记下行号。行数和 seqs 必须在调用方那边能一一对上，
         * 所以"放不下就不放"的判据用 n < seq_cap —— 调用方把 seq_cap
         * 传成 max_lines 就一定对得上（buf 塞不下时会提前 break）。 */
        if (seqs && n < seq_cap) {
            seqs[n] = s_lines[i].seq;
        }
        n++;
    }
    buf[off] = '\0';

    xSemaphoreGiveRecursive(s_lock);
    if (out_count) {
        *out_count = n;
    }
    return (int)off;
}

int ams_log_recent(char *buf, size_t buflen, int max_lines)
{
    return ams_log_recent_lines(buf, buflen, max_lines, NULL, 0, NULL);
}

/* --------------------------------------------------------------------------
 * 内部：按 UTF-8 字符边界截断
 * -------------------------------------------------------------------------- */

/**
 * 把 s 截到不超过 max_bytes 字节，且**不切断**任何 UTF-8 字符。
 *
 * 为什么需要它：每格只有 AMS_LOG_LINE_MAX 字节，直接用 snprintf("%s%s")
 * 是按**字节**截的。中文一个字 3 字节，只要可用长度不是 3 的倍数，最后那个
 * 字就会被切成半个 —— 网页上显示成一个乱码方块，更糟的是浏览器会把后面
 * 的换行也一起吞掉，导致整段日志错位。用户报的"网页日志混乱"里就有它一份。
 *
 * @return 可以安全截取的字节数（可能小于 max_bytes，极端情况为 0）
 */
static size_t utf8_truncate(const char *s, size_t max_bytes)
{
    size_t len = strlen(s);
    if (len <= max_bytes) {
        return len;                        /* 根本没超，原样返回 */
    }

    size_t n = max_bytes;
    while (n > 0) {
        unsigned char c = (unsigned char)s[n - 1];
        if ((c & 0xC0u) == 0x80u) {
            n--;                           /* 落在续字节上：不是字符边界，往左找 */
            continue;
        }
        /* s[n-1] 是某个字符的首字节，看它一共声明了几个字节 */
        size_t need = (c < 0x80u)              ? 1u
                    : ((c & 0xE0u) == 0xC0u)   ? 2u
                    : ((c & 0xF0u) == 0xE0u)   ? 3u
                                               : 4u;
        if (n - 1u + need <= max_bytes) {
            return n - 1u + need;          /* 这个字完整落在范围内，收工 */
        }
        n--;                               /* 不完整，把它也退掉 */
    }
    return 0;
}

/* --------------------------------------------------------------------------
 * 内部：把一行塞进环形缓冲
 * -------------------------------------------------------------------------- */
static void log_push(const char *prefix, const char *body)
{
    if (!s_inited) {
        /* 还没初始化就直接回显，绝不因为"日志系统没起来"把调用方搞挂 */
        printf("%s%s\n", prefix, body);
        return;
    }

    if (xSemaphoreTakeRecursive(s_lock, portMAX_DELAY) != pdTRUE) {
        return;
    }

    /* 满了就把整体前移一格，腾出最后一格给新行 */
    if (s_count == AMS_LOG_LINES) {
        memmove(&s_lines[0], &s_lines[1],
                sizeof(log_line_t) * (AMS_LOG_LINES - 1));
        s_count = AMS_LOG_LINES - 1;
    }

    log_line_t *slot = &s_lines[s_count];
    s_seq++;
    slot->seq = s_seq;

    /* 分:秒.毫秒 时间戳，和 Python 版 logout.py 的格式一致 */
    uint32_t ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    uint32_t total_s = ms / 1000u;
    char stamp[16];
    int sn = snprintf(stamp, sizeof(stamp), "%02u:%02u.%03u ",
                      (unsigned)((total_s / 60u) % 100u),
                      (unsigned)(total_s % 60u),
                      (unsigned)(ms % 1000u));
    if (sn < 0) {
        sn = 0;
    }

    /* ★ 时间戳 + 正文拼好后按 UTF-8 字符边界截断（见 utf8_truncate）。
     *   先拼成整串再截，是为了让"截断位置"只受一个上限约束 ——
     *   否则时间戳和正文各截一次，边界对不上反而更容易切坏。 */
    char line[AMS_LOG_LINE_MAX * 2];
    snprintf(line, sizeof(line), "%s%s", prefix, body);

    size_t avail = sizeof(slot->text) - 1u - (size_t)sn;
    size_t blen  = utf8_truncate(line, avail);

    memcpy(slot->text, stamp, (size_t)sn);
    memcpy(slot->text + sn, line, blen);
    slot->text[(size_t)sn + blen] = '\0';

    s_count++;
    xSemaphoreGiveRecursive(s_lock);
}

/** 三个入口共用的实现：先格式化，再分别送串口和缓冲 */
static void log_format(const char *level_tag, const char *prefix,
                       esp_log_level_t level, const char *fmt, va_list ap)
{
    char body[AMS_LOG_LINE_MAX * 2];
    vsnprintf(body, sizeof(body), fmt, ap);

    log_push(prefix, body);

    /* 串口那一份交给 ESP_LOG，保持和 ESP-IDF 其它日志同样的格式 */
    switch (level) {
    case ESP_LOG_ERROR:
        ESP_LOGE(TAG, "%s%s", prefix, body);
        break;
    case ESP_LOG_WARN:
        ESP_LOGW(TAG, "%s%s", prefix, body);
        break;
    default:
        ESP_LOGI(TAG, "%s%s", level_tag, body);
        break;
    }
}

void ams_log(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_format("", "", ESP_LOG_INFO, fmt, ap);
    va_end(ap);
}

void ams_log_warn(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    log_format("警告：", "[警告] ", ESP_LOG_WARN, fmt, ap);
    va_end(ap);
}

void ams_log_err(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    /* ★ 前缀带 '★' —— 网页端的日志面板就是靠它把错误行标红的
     *   （和 Python 版 logout.py 的做法保持一致，前端不用改判断逻辑） */
    log_format("", "★ ", ESP_LOG_ERROR, fmt, ap);
    va_end(ap);
}
