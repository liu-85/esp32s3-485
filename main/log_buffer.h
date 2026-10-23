/**
 * log_buffer.h —— 日志环形缓冲
 * ============================================================================
 *
 * 对应 MicroPython 版的 logout.py：除了照原样打到串口，还把最近若干行留在
 * 内存里，供网页右侧的「设备日志」面板读取 —— 这样调试时不用一直插着 USB。
 *
 * 和 Python 版的差异：
 *   · 用 FreeRTOS 互斥锁保护，多个任务并发写不会撕裂（Python 版靠 GIL 侥幸躲过）
 *   · 每行固定长度、固定条数，内存占用是编译期确定的，不会因为某条日志特别长
 *     而突然吃掉一大块堆（Python 版为此专门做了 MemoryError 兜底）
 *   · 同时转给 ESP_LOG，所以串口那边和以前一样能看到
 *
 * 内存代价：AMS_LOG_LINES × AMS_LOG_LINE_MAX = 48 × 144 ≈ 6.9 KB，常驻。
 * 空闲内存紧张时可以调小这两个值，但别指望它能当文件系统用 —— 真正的历史
 * 日志请走串口。
 *
 * ★ 112 → 144 的由来（2026-09-24）：
 *   现场截图里那句"辅助送料回执…通道2 PWM duty=204/2"后面被**硬生生切断**了
 *   —— 不是显示问题，是这行超过了 AMS_LOG_LINE_MAX，log_push 里的
 *   utf8_truncate 把它截掉了，后面"硬件回读 IN1=…"整段全丢。
 *   中文一个字 3 字节，112 字节里还要塞 11 字节时间戳，正文只剩 101 字节
 *   ≈ 33 个汉字 —— 稍微带点参数就顶格。所以：
 *     · 上限提到 144（+1.7KB 常驻，换掉一整类"日志看不全"的返工，值）；
 *     · 同时把所有长日志重写成短句式（见 tools/check_log_length.py，
 *       CI 之前就会把"可能被截断的行"列出来）。
 *   ⚠️ 别再靠"把话写长"来表达信息 —— 一条日志一件事，说清就停。
 */

#ifndef LOG_BUFFER_H
#define LOG_BUFFER_H

#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 保留多少行（头部最旧、尾部最新） */
#define AMS_LOG_LINES 48
/** 每行最多多少字节（超长截断，防止一条日志吃掉整块内存）。
 *  ★ 改小它之前先看上面那段说明：太小会把带参数的日志**从中间切断**，
 *    而截断是静默的 —— 现场只会觉得"日志怎么缺了一截"。 */
#define AMS_LOG_LINE_MAX 144

/**
 * 初始化日志缓冲。纯内存操作，不会失败；重复调用是安全的 no-op。
 * 放在 app_main 最前面调用，这样后面所有模块的日志都能进缓冲。
 */
void ams_log_init(void);

/** 清空缓冲（只清内存里的，串口历史不受影响） */
void ams_log_clear(void);

/** 当前缓冲里有多少行 */
int ams_log_count(void);

/**
 * 取最近的日志，拼成一段以 '\n' 分隔的文本写进 buf。
 *
 * @param buf        输出缓冲
 * @param buflen     缓冲大小
 * @param max_lines  最多取多少行（<=0 表示全取）
 * @return           实际写入的字节数
 *
 * 返回的是**副本**，调用方拿去序列化时不会和正在写入的任务打架。
 */
int ams_log_recent(char *buf, size_t buflen, int max_lines);

/**
 * 同上，但把每行的**行号 seq** 一并输出。
 *
 * @param seqs      输出每行的 seq；可以传 NULL
 * @param seq_cap   seqs 能装多少个 —— **建议传和 max_lines 一样的值**，
 *                  这样行数和 seq 一定一一对应
 * @param out_count 实际输出的行数；可以传 NULL
 *
 * 为什么要 seq：网页端如果只拿到"一串文本"，就只能靠整串比对判断有没有
 * 新行 —— 环形缓冲一滚动整串都变了，前端只能整屏重建，表现就是闪烁 +
 * 丢行，而且"清屏"按钮怎么点都清不干净（它没法知道"清到哪一行"）。
 * 带上行号之后，前端只要记住"渲染到哪一号了"，就能只追加新行；
 * 清屏也只是把 DOM 清掉、"游标"不动。
 */
int ams_log_recent_lines(char *buf, size_t buflen, int max_lines,
                         uint32_t *seqs, int seq_cap, int *out_count);

/* --------------------------------------------------------------------------
 * 写日志的三个入口。行为上和 Python 版的 logout() 一致：串口 + 内存各一份。
 * -------------------------------------------------------------------------- */

/** 普通信息 */
void ams_log(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** 警告（串口是黄色，网页上高亮） */
void ams_log_warn(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

/** 错误（串口是红色，网页上高亮）—— 以前带 is_error=True 的那些调用 */
void ams_log_err(const char *fmt, ...) __attribute__((format(printf, 1, 2)));

#ifdef __cplusplus
}
#endif

#endif /* LOG_BUFFER_H */
