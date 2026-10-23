/**
 * comm_485.c —— 拓竹 BambuBus（RS-485）直连打印机的实现
 * ============================================================================
 *
 * 帧格式（照 hu504011383/Tianshu 的 BambuBus.cpp 核对过，别凭印象改）：
 *
 *   短帧（帧头 bit7 = 1）：
 *     偏移  0     1          2     3      4        5 ...      LEN-2 LEN-1
 *     内容  0x3D  T|0x80     LEN   CRC8   CMD      payload    CRC16 CRC16
 *
 *   长帧（帧头 bit7 = 0）：
 *     偏移  0     1    2-3      4-5      6     7-8     9-10   11-12   13 ...   LEN-2 LEN-1
 *     内容  0x3D  0x00 pkg_num  pkg_len  CRC8  target  source  type    payload CRC16 CRC16
 *
 *   · LEN / pkg_len = **整包总字节数**（从 0x3D 到 CRC16 末字节）
 *   · CRC8 覆盖：短帧 data[0..2]（3 字节）；长帧 data[0..5]（6 字节）
 *   · CRC16 覆盖：除自身外整包，**低字节先发**
 *   · 多字节一律小端
 *   · 打印机发的短帧第 2 字节固定 0xC5；我们回的短帧用 0xC0
 *
 * ----------------------------------------------------------------------------
 * ★★ 为什么这版几乎整篇重写（2026-10-22 晚）★★
 * ----------------------------------------------------------------------------
 * 旧版只做"握手 + 记日志"：0x05 回 2 字节设备类型，0x20 回 1 字节，其余只打印。
 * 真机现象是"收 1014 帧 / 发 966 帧，打印机界面里没有 AMS"。
 *
 * 对照实测能用的实现后确认：BambuBus **没有"设备类型"这个字段**。打印机的流程是
 *
 *   ① 广播 0x05 {kind=0x00}  → 每台 AMS 回一份 **29 字节注册包**（内含身份/序列号片段）
 *   ② 点名 0x05 {kind=0x01, idx} → 对应那台再回一次，并把 20 字节身份回传给打印机核对
 *   ③ 长帧 0x402 要序列号、0x103 要版本号
 *   ④ 长帧 0x211 要每个槽的颜色/温度/材料名
 *   ⑤ 长帧 0x21A 心跳
 *
 * 只有 ①②③ 都通了，打印机才会把它列进 AMS 列表；④ 决定槽位上显示什么颜色。
 * 所以本版把 ①~⑤ 全部实现，字节内容照 Tianshu 逐字对齐。
 *
 * ----------------------------------------------------------------------------
 * 运动指令（0x03 / 0x04）现在是真的驱动电机了
 * ----------------------------------------------------------------------------
 * PR #14 那一版是"只记日志 + 回空闲"，因为字段含义没现场数据、怕把料退反。
 * 第十六轮拿到了 /bus485raw 的双向实测帧，并把语义和 Tianshu 逐条对齐了：
 *
 *   cmd 0x03（Cxx，12 字节）  = 运动命令！按 statu_flag/motion_flag 走状态机
 *   cmd 0x04（Dxx，13 字节）  = 状态查询，只回状态，不驱动机
 *
 * ★ 上一版最大的错：把 flags=0x03/motion=0x00（send_out 正转送料）当成了退料，
 *   于是"点进料电机不转"。现在的映射见上面「运动状态机」那一整块注释。
 *
 * 开关 `485 运动执行`（网页可改、立即生效，默认开）关掉时仍会回状态，
 * 只是不驱动机 —— 这是留给你单独验证总线时用的。
 */

#include "comm_485.h"

#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.h"
#include "log_buffer.h"

#if BOARD_485_ENABLED

#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#include "ams_controller.h"

/* ==========================================================================
 * 常量
 * ========================================================================== */
#define BUS_HEAD        0x3D
#define BUS_MAX_FRAME   999
#define BUS_TASK_STACK  4096
/* ★ 优先级 12（原 7）：1.2288Mbaud 的接收任务不能被普通任务压住，
 *   否则 FIFO 溢出丢字节 → 整帧 CRC 错 → 打印机认为 AMS 掉线。 */
#define BUS_TASK_PRIO   12

#define AMS_SLOTS       4                        /* 一台 AMS 固定 4 槽 */
#define AMS_UNITS       COMM_485_AMS_UNITS
#define AMS_TOTAL_SLOT  (AMS_UNITS * AMS_SLOTS)

/* 4 槽"剩余料"位图（left_char）：每槽占 2 bit。
 *   0b01 = 有料静止  0b11 = 有料且正在运动
 * 虚拟 AMS 的 4 个槽背后就是 4 个实体料盘位，恒为"有料"；整片写 0 会让
 * 打印机把槽当成空的 → 槽位操作全部变灰。
 * 现在改由 mot_left_char() 按每槽真实运动状态算，不再写死 0x55。 */

/* 统计 */
static bool     s_ready;
static bool     s_stop;             /* 通知 bus_task 退出（deinit 用） */
static TaskHandle_t s_task;         /* bus_task 句柄，deinit 时要等它真的停 */
static bool     s_online_seen;
static uint32_t s_rx_frames;
static uint32_t s_tx_frames;
static uint32_t s_crc_errors;
static uint32_t s_last_cmd;
static int64_t  s_last_rx_us;
static uint8_t  s_pkg_num;          /* 运动类应答的滚动序号 0~7 */
static uint8_t  s_reg_mask;         /* bit n = 第 n 台已注册通过 */
/* ★ 0x06 握手的"钉住台号"（0xFF = 还没定），理由见 on_req06() */
static uint8_t  s_req06_unit = 0xFF;
/* ★ 兜底自愈：总线静默超过 60 秒才认为打印机重启过、把注册位图清零。
 *   原来是 10 秒 —— 太短了：打印机空闲时（不做任何 AMS 操作）总线本来
 *   就会长时间没有流量，10 秒一清，下一帧广播就重报一遍，打印机把它当成
 *   "设备接入"重新枚举，于是 AMS 反复出现/消失。 */
static int64_t  s_last_rearm_us;
static bool     s_idle_rearmed;

/* 各类帧的计数器（用来限流日志，别把网页日志刷爆） */
static uint32_t s_rx_buf_warns;     /* RX FIFO 堆积告警，最多打 5 次 */
static uint32_t s_cnt_reg;
static uint32_t s_cnt_req06;
static uint32_t s_cnt_ver;
static uint32_t s_cnt_sn;
static uint32_t s_cnt_fil;
static uint32_t s_cnt_mc;
static uint32_t s_cnt_08;
static uint32_t s_cnt_motion;
static uint32_t s_cnt_other;

/* ==========================================================================
 * 运动状态机（★ 2026-10-23 第十六轮新增）
 * ==========================================================================
 *
 * 打印机不是"发一条进料指令就完事"，它按 BambuBus 的状态机一帧一帧地推：
 *
 *   flags=0x03 motion=0x00  → send_out           ★ 进料：电机**正转**持续送料
 *   flags=0x09 motion=0xA5  → before_on_use       预送（把料拱到挤出机入口）
 *   flags=0x07 motion=0x7F  → on_use              挤出机在拉料，AMS 停止自驱
 *   flags=0x07 motion=0x00  → stop_on_use         停止
 *   flags=0x09 motion=0x3F  → before_pull_back    准备退料
 *   （随后 slot=0xFF 且 flags=0x03/0x00） → pull_back  ★ 退料：电机**反转**
 *
 * ★★ 上一版把 send_out 当成了退料 ★★ —— 现场表现正是"点进料，电机不转"。
 *    证据（Tianshu AMCU.cpp）：send_out → motorCmd(ch, 1) 正转，
 *    pull_back → motorCmd(ch, -1) 反转；on_use 落到 default → motorCmd(ch, 0) 停。
 *
 * 另外三件事也必须跟着状态走，否则打印机会"中途放弃"（即使电机转了）：
 *   · Dxx/Cxx 的 use_flag    ：有通道在送/退料 → 0x02；在 on_use 系 → 0x04
 *   · Dxx 的 pressure        ：送/退料 0x4700，on_use 系 0x2B00，空闲 0xFFFF
 *   · left_char 每槽 2 bit   ：0b01 有料静止，0b11 有料且运动中
 *     恒报"空闲"= 打印机认为 AMS 根本没动 → 直接判失败。
 */
enum {
    MOT_IDLE = 0,
    MOT_SEND_OUT,
    MOT_BEFORE_ON_USE,
    MOT_ON_USE,
    MOT_STOP_ON_USE,
    MOT_BEFORE_PULL_BACK,
    MOT_PULL_BACK,
};

static uint8_t  s_mot[BOARD_CHANNEL_COUNT];  /* 每通道运动状态 */
static int8_t   s_now_mot_ch = -1;           /* 当前占用通道；-1 = 无 */
static uint32_t s_sendout_us[BOARD_CHANNEL_COUNT];
/* 日志去重：只在"命令组合"变化时打一行（现场点一下进料就能看到唯一证据） */
static uint8_t  s_noted_mot_ch = 0xFF;
static uint8_t  s_noted_flags;
static uint8_t  s_noted_motion;
static uint8_t  s_noted_slot = 0xFF;

/* 接收状态机 */
static uint8_t  s_buf[BUS_MAX_FRAME + 8];
static int      s_idx;

/* ==========================================================================
 * 原始报文环形缓冲（★ 2026-10-22 深夜新增）
 * ==========================================================================
 *
 * ★★ 为什么必须加这个 ★★
 *
 *   用户的现场形态是「AUTO 模式：开机先跑 485、**不开 WiFi**，2 分钟收不到
 *   打印机才开 WiFi」。而打印机的总线枚举**只在上电后那一小会儿做**。
 *   结果就是：真正要看的握手全过程（0x05 广播 / 0x01 点名 / 0x06 握手 /
 *   0x402 序列号 / 0x103 版本）全都发生在**网页还连不上的那 2 分钟里**，
 *   等网页能打开时，总线上只剩心跳了 —— 等于每次排查都是瞎的。
 *
 *   有了这个缓冲，WiFi 起来之后打开 `/bus485raw` 就能回看**从上电第一帧
 *   开始**的双向报文，一行一帧、带毫秒时间戳和方向。这才有可能拿我们的
 *   帧去和 Tianshu（用户手上那块能认 4 台的板子）的帧逐字节对齐。
 *
 *   代价：40 条 × 34 字节 ≈ 1.4KB RAM。相对它的价值可以忽略。
 *
 * ★ 2026-10-23 扩到 256 帧：40 帧只够看"当前这一刻"，退料-进料全过程
 *   会被打印阶段的空闲轮询（每 ~100ms 一帧）刷掉。256 帧 ≈ 25 秒窗口，
 *   足够把一次完整的退料→进料→到位→退料信号序列完整保留下来。
 *   RAM 代价：256 × 34 ≈ 8.7KB，S3 有 512KB SRAM，仍可忽略。
 */
#define BUS_LOG_N     256       /* 保留最近 256 帧 */
#define BUS_LOG_BYTES 32        /* 每帧只存前 32 字节（够看头和台号） */

typedef struct {
    uint32_t ms;                /* 开机以来的毫秒数 */
    uint8_t  dir;               /* 0 = 我方发出（TX），1 = 收到（RX） */
    uint8_t  len;               /* 整帧字节数（可能大于存下来的 32） */
    uint8_t  d[BUS_LOG_BYTES];
} bus_rec_t;

static bus_rec_t s_log[BUS_LOG_N];
static int       s_log_head;    /* 下一条写入位置（环形） */
static uint32_t  s_log_seq;     /* 累计写入条数，用来判断有没有新帧 */

static void bus_record(uint8_t dir, const uint8_t *f, int len)
{
    if (!f || len <= 0) {
        return;
    }
    bus_rec_t *r = &s_log[s_log_head];
    r->ms  = (uint32_t)(esp_timer_get_time() / 1000);
    r->dir = dir;
    r->len = (uint8_t)((len > 255) ? 255 : len);
    int n = (len > BUS_LOG_BYTES) ? BUS_LOG_BYTES : len;
    memcpy(r->d, f, (size_t)n);
    s_log_head = (s_log_head + 1) % BUS_LOG_N;
    s_log_seq++;
}

/**
 * 把环形缓冲导成文本，一行一帧：
 *     1234ms RX 8  3D C0 08 C5 05 00 ...
 *     1250ms TX 29 3D C0 1D B4 05 00 00 ...
 *
 * 返回写入的字符数（不含结尾 0）。buflen 建议 ≥ 2400。
 */
int comm_485_dump(char *buf, size_t buflen)
{
    if (!buf || buflen < 64) {
        return 0;
    }
    int written = snprintf(buf, buflen,
        "# 485 原始报文（最近 %d 帧，共录过 %lu 帧）\n"
        "# 格式: <毫秒> <方向> <整帧字节数> <十六进制，最多前 %d 字节>\n"
        "# 状态: rx=%lu tx=%lu crc_err=%lu 注册位图=0x%02X 台数=%d\n",
        BUS_LOG_N, (unsigned long)s_log_seq, BUS_LOG_BYTES,
        (unsigned long)s_rx_frames, (unsigned long)s_tx_frames,
        (unsigned long)s_crc_errors, (unsigned)s_reg_mask, AMS_UNITS);
    if (written < 0) {
        return 0;
    }

    int total = (s_log_seq < BUS_LOG_N) ? (int)s_log_seq : BUS_LOG_N;
    int start = (s_log_head - total + BUS_LOG_N) % BUS_LOG_N;

    for (int i = 0; i < total && (size_t)written + 120 < buflen; i++) {
        const bus_rec_t *r = &s_log[(start + i) % BUS_LOG_N];
        int n = r->len;
        if (n > BUS_LOG_BYTES) {
            n = BUS_LOG_BYTES;
        }
        int k = snprintf(buf + written, buflen - (size_t)written,
                         "%lums %s %u ", (unsigned long)r->ms,
                         r->dir ? "RX" : "TX", (unsigned)r->len);
        if (k < 0) {
            break;
        }
        written += k;
        for (int b = 0; b < n && (size_t)written + 4 < buflen; b++) {
            k = snprintf(buf + written, buflen - (size_t)written,
                         "%02X ", r->d[b]);
            if (k < 0) {
                break;
            }
            written += k;
        }
        if ((size_t)written + 2 < buflen) {
            buf[written++] = '\n';
            buf[written]   = '\0';
        }
    }
    return written;
}

/* ==========================================================================
 * 校验
 * ========================================================================== */
static uint8_t crc8_calc(const uint8_t *d, size_t n)
{
    uint8_t crc = 0x66;     /* 初值 0x66，多项式 0x39，不反射、末异或 0 */
    for (size_t i = 0; i < n; i++) {
        crc ^= d[i];
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x80) ? (uint8_t)((crc << 1) ^ 0x39)
                               : (uint8_t)(crc << 1);
        }
    }
    return crc;
}

static uint16_t crc16_calc(const uint8_t *d, size_t n)
{
    uint16_t crc = 0x913D;  /* 初值 0x913D，多项式 0x1021(CCITT)，不反射 */
    for (size_t i = 0; i < n; i++) {
        crc ^= (uint16_t)d[i] << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ==========================================================================
 * 发送
 * ========================================================================== */

/**
 * 把一整帧发出去。
 *
 * ★ DE 的时序是这条总线最容易翻车的地方：
 *   拉高 DE → 等收发器换向（几微秒）→ 写 UART → **等发送彻底完成** → 拉低 DE。
 *   "等发送完成"必须用 uart_wait_tx_done()（它等的是移位寄存器也空了），
 *   只等 FIFO 空了就拉低 DE，最后一个字节会被截掉 —— 表现为 CRC 永远错。
 *   而且拉低 DE 之后再留 2µs 释放总线。
 */
static void bus_send(const uint8_t *frame, size_t n)
{
    if (!s_ready || n == 0 || n > BUS_MAX_FRAME) {
        return;
    }
    gpio_set_level((gpio_num_t)BOARD_PIN_485_DE, 1);
    esp_rom_delay_us(3);
    uart_write_bytes(COMM_485_UART_NUM, frame, n);
    uart_wait_tx_done(COMM_485_UART_NUM, pdMS_TO_TICKS(50));
    esp_rom_delay_us(2);
    gpio_set_level((gpio_num_t)BOARD_PIN_485_DE, 0);
    s_tx_frames++;
    bus_record(0, frame, (int)n);      /* 留一份，网页 /bus485raw 可回看 */
}

/**
 * 填长度 + 两层 CRC 再发出去。total = 整包字节数（含 CRC16）。
 * 这样上层只管摆数据，不用手算校验 —— 手算 CRC 是这类协议最常见的错源。
 */
static void tx_frame_ex(uint8_t *f, int total, int len_field)
{
    if (total < 8 || total > BUS_MAX_FRAME) {
        return;
    }
    if (f[1] & 0x80) {
        f[2] = (uint8_t)len_field;
        f[3] = crc8_calc(f, 3);
    } else {
        f[4] = (uint8_t)(total & 0xFF);
        f[5] = (uint8_t)((total >> 8) & 0xFF);
        f[6] = crc8_calc(f, 6);
    }
    uint16_t c = crc16_calc(f, (size_t)(total - 2));
    f[total - 2] = (uint8_t)(c & 0xFF);
    f[total - 1] = (uint8_t)(c >> 8);
    bus_send(f, (size_t)total);
}

/** 常规发送：LEN 字段 = 实际总长度 */
static void tx_frame(uint8_t *f, int total)
{
    tx_frame_ex(f, total, total);
}

/** 短帧：hdr 传 0xC0 或 0xC0 | (n<<3)。本版所有短帧都靠固定模板发，
 *  留着它是为了以后接 0x06 / 0x07 这类"回一个空模板"的场合。 */
static void __attribute__((unused))
send_short(uint8_t hdr, uint8_t cmd, const uint8_t *pl, int plen)
{
    uint8_t f[64];
    int n = 0;

    if (plen < 0 || plen > (int)sizeof(f) - 9) {
        plen = 0;
    }
    f[n++] = BUS_HEAD;
    f[n++] = hdr;
    f[n++] = 0;                            /* LEN，tx_frame 会填 */
    f[n++] = 0;                            /* CRC8，tx_frame 会填 */
    f[n++] = cmd;
    for (int i = 0; i < plen; i++) {
        f[n++] = pl[i];
    }
    tx_frame(f, n + 2);
}

/**
 * 长帧：target = 请求里的 source（打印机地址），
 *       source = 请求里的 target（**打印机指定我们的地址**），
 *       pkg 沿用请求里的序号。
 *
 * ★★ 2026-10-22 深夜 修「手机 APP 报 AMS A 固件与打印机不匹配」★★
 *
 * 旧写法把 source 硬编码成 0x700（AMS08）。但打印机的 AMS 地址不是固定的：
 *   · 选「AMS Lite」类型     → 打印机按 **0x1200** 寻址
 *   · 选「AMS/2 Pro/HT」类型 → 打印机按 **0x700** 寻址
 * 它把"我认为你是谁"写在请求的 target_address 里（BMCU370 的
 * `BambuBus_address = printer_data_long.target_address` 就是这个意思）。
 *
 * 我们回 0x700、它问 0x1200 → 源地址对不上 → **整帧丢弃**：版本读不到
 * （APP 就报"固件与打印机不匹配"）、槽位读不到（界面全灰）、第二台更不可
 * 能注册上。
 *
 * 正确做法：把它问的地址原样当作我们的源地址回过去。这样不管是 lite 还是
 * AMS 模式都自动成立，不需要用户去网页里选型号。
 */
static uint16_t s_my_addr = COMM_485_DEV_TYPE;   /* 最近一次被寻址的地址 */

static void send_long(uint16_t type, uint16_t target, uint16_t src,
                      uint16_t pkg, const uint8_t *pl, int plen)
{
    static uint8_t f[256];

    if (plen < 0 || plen > (int)sizeof(f) - 17) {
        return;
    }
    if (src == 0) {
        src = COMM_485_DEV_TYPE;
    }
    f[0] = BUS_HEAD;
    f[1] = 0x00;                           /* bit7 = 0 → 长帧 */
    f[2] = (uint8_t)(pkg & 0xFF);
    f[3] = (uint8_t)(pkg >> 8);
    f[4] = 0;                              /* pkg_len 低，tx_frame 会填 */
    f[5] = 0;
    f[6] = 0;                              /* CRC8，tx_frame 会填 */
    f[7] = (uint8_t)(target & 0xFF);
    f[8] = (uint8_t)(target >> 8);
    f[9] = (uint8_t)(src & 0xFF);
    f[10] = (uint8_t)(src >> 8);
    f[11] = (uint8_t)(type & 0xFF);
    f[12] = (uint8_t)(type >> 8);
    if (plen > 0) {
        memcpy(f + 13, pl, (size_t)plen);
    }
    tx_frame(f, 13 + plen + 2);
}

/* ==========================================================================
 * 日志（限流）
 * ========================================================================== */
static void log_frame(const char *tag, const uint8_t *f, int len)
{
    char hex[24 * 3 + 4];
    int off = 0;
    int show = (len > 24) ? 24 : len;
    for (int i = 0; i < show && off < (int)sizeof(hex) - 4; i++) {
        off += snprintf(hex + off, sizeof(hex) - (size_t)off, "%02X ", f[i]);
    }
    if (len > show) {
        snprintf(hex + off, sizeof(hex) - (size_t)off, "...");
    }
    ams_log("485 %s(%dB) %s", tag, len, hex);
}

/**
 * 限流日志：前 3 次全打，之后每 200 次打一次。
 * ★ 打印机每秒会发十几帧，逐帧打印会把 4KB 的日志环形缓冲冲光，
 *   真正有用的开机信息反而看不到。
 */
static void log_limited(const char *tag, const uint8_t *f, int len, uint32_t *cnt)
{
    (*cnt)++;
    if (*cnt > 3 && (*cnt % 200) != 0) {
        return;
    }
    log_frame(tag, f, len);
}

/* ==========================================================================
 * 报文模板（★ 全部逐字节照 Tianshu 的 BambuBus.cpp，别手改）
 * ========================================================================== */

/**
 * ① 注册包的身份数据（22 字节，放在 29 字节注册包的索引 5..26）。
 *    索引 0 = 0x00 广播应答 / 0x01 点名应答；索引 1 = AMS 台号。
 *
 * ★ 后面 20 字节是打印机要回传核对的"身份"，内容必须和真机一致 ——
 *   这是照实测可用的实现抄的，不是可读的文本，别试图"修正"它。
 */
static const uint8_t REG_ID[22] = {
    0x01, 0x00, 0x16,
    0x0E, 0x7D, 0x32, 0x31, 0x31, 0x38, 0x15, 0x00,
    0x36, 0x39, 0x37, 0x33,
    0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00, 0x00
};

#define REG_FRAME_LEN  29

/** 组一帧注册包并发出。kind: 0x00 广播 / 0x01 点名；unit: 台号（0 起） */
static void send_register(uint8_t kind, uint8_t unit)
{
    uint8_t b[REG_FRAME_LEN];

    b[0] = BUS_HEAD;
    b[1] = 0xC0;
    b[2] = 0;                              /* LEN，tx_frame 填 */
    b[3] = 0;                              /* CRC8，tx_frame 填 */
    b[4] = 0x05;
    memcpy(b + 5, REG_ID, sizeof(REG_ID));
    b[5] = kind;
    b[6] = unit;
    /* ★★ 2026-10-23 撤销 0.16.0 的"身份每台不同"，这是错的 ★★
     *
     * 0.16.0 我让 b[9] += unit（台 0 → 0x7D、台 1 → 0x7E），理由是
     * "打印机按序列号认设备，两台序列号相同会被判重"。现场结果是
     * **只有 AMS-A，第二台彻底不出现** —— 正好印证这个假设是反的。
     *
     * Tianshu 的四套注册包 `F00_res[4][29]` 摆在那里，20 字节身份
     * **逐字节完全相同**（都是 `16 0E 7D 32 31 31 38 15 00 36 39 37 33
     * FF FF FF FF 00 00 00`），只有 [6] 台号不同：
     *     {..., 0x05, 0x01, 0x00, 0x16, 0x0E, 0x7D, ...}
     *     {..., 0x05, 0x01, 0x01, 0x16, 0x0E, 0x7D, ...}
     *     {..., 0x05, 0x01, 0x02, 0x16, 0x0E, 0x7D, ...}
     *     {..., 0x05, 0x01, 0x03, 0x16, 0x0E, 0x7D, ...}
     * 这块板子在用户的 A1（固件 1.7.0）上实测出 AMS-A/B/C/D 共 16 槽。
     *
     * 结论：这 20 字节是**型号指纹**（打印机拿它比对白名单），不是序列号。
     * 动它 = 指纹对不上 = 那一台不注册。真正带台号的只有 [6]，
     * 以及序列号包(0x402)里的 [4]/[34]/[65] —— 那两处才需要区分。 */
    tx_frame(b, REG_FRAME_LEN);
}

/**
 * ③ 序列号包（66 字节）。
 *    [0]=0x0F 长度、[1..15]=15 位 SN、[33..34]=标识、[65]=台号。
 *    台号同时改到 [4]（SN 第 4 位）和 [34]，两台才不会重号。
 */
#define SN_FRAME_LEN   66

static void build_sn(uint8_t *b, uint8_t unit)
{
    memset(b, 0, SN_FRAME_LEN);
    b[0] = 0x0F;
    memset(b + 1, 0x30, 15);
    b[33] = 0x0E;
    b[34] = 0x7D;
    b[35] = 0x32; b[36] = 0x31; b[37] = 0x31; b[38] = 0x38;
    b[39] = 0x15; b[40] = 0x00;
    b[41] = 0x36; b[42] = 0x39; b[43] = 0x37; b[44] = 0x33;
    memset(b + 45, 0xFF, 4);
    memset(b + 49, 0xFF, 8);
    b[57] = 0xBB; b[58] = 0x44;
    memset(b + 59, 0xFF, 6);

    b[4]  = (uint8_t)(0x30 + unit);
    b[34] = (uint8_t)(0xA0 + unit);
    b[65] = unit;
}

/**
 * ③ 版本包（22 字节）：4 字节版本号（小端）+ 19 字节硬件代号 + 台号。
 *
 * ★★ 2026-10-22 深夜 修「手机 APP：AMS A 固件与打印机不匹配」★★
 *
 * 旧版照 Tianshu 发 `00 00 00 5A`（0.0.0.90）+ "AMS08"。Tianshu 在旧版打印机
 * 固件上能混过去，但新版打印机会拿这个版本号跟自己的白名单比 —— 0.0.0.90
 * 显然不在里面，于是报 HMS **0500-0400-0001-0044**
 * 「The firmware of AMS A does not match the printer」。
 *
 * 这里改用**真机固件版本号**（BMCU370 实测抓到的）：
 *   · AMS      → 00.00.06.49  → 字节 {0x31,0x06,0x00,0x00}，代号 "AMS08"
 *   · AMS Lite → 00.01.02.03  → 字节 {0x03,0x02,0x01,0x00}，代号 "AMS_F102"
 * 型号由打印机寻址的地址决定（0x700 = AMS / 0x1200 = Lite），不用用户手选。
 *
 * 台号两个位置都写（BMCU370 用 [20]，Tianshu 用 [21]）—— 打印机读哪个都能对上。
 */
#define VER_FRAME_LEN  22

static void build_ver(uint8_t *b, uint8_t unit, uint16_t my_addr)
{
    memset(b, 0, VER_FRAME_LEN);
    if (my_addr == COMM_485_ADDR_LITE) {
        b[0] = 0x03; b[1] = 0x02; b[2] = 0x01; b[3] = 0x00;
        memcpy(b + 4, "AMS_F102", 8);
    } else {
        b[0] = 0x31; b[1] = 0x06; b[2] = 0x00; b[3] = 0x00;
        memcpy(b + 4, "AMS08", 5);
    }
    b[VER_FRAME_LEN - 2] = unit;              /* 索引 20，BMCU370 的位置 */
    b[VER_FRAME_LEN - 1] = unit;              /* 索引 21，Tianshu 的位置 */
}

/**
 * ④ 槽位耗材信息（131 字节）。
 *    [0]=台号 [1]=槽号 [19..26]=ID [27..46]=材料名
 *    [59..62]=RGBA [79..80]=最高温 [81..82]=最低温
 *
 * ★ 按偏移逐字段填，不手抄 131 个字节 —— 手抄错一个字节整个包就废了，
 *   而且错在哪极难看出来。
 */
#define FIL_FRAME_LEN  131
static uint8_t TPL_FIL[FIL_FRAME_LEN];

static void fil_template_init(void)
{
    memset(TPL_FIL, 0, sizeof(TPL_FIL));
    memcpy(TPL_FIL + 19, "GFB00", 5);        /* 耗材 ID：GFB00 = Bambu ABS */
    /* ★ 名字必须和 ID 自洽（2026-10-23）。原来这里是 "PLA"，而 ID 是
     *   GFB00（ABS 的料号）—— 打印机拿 ID 判材料类型、拿名字做显示，
     *   两者打架时 APP 会把该槽判成"参数异常"，表现就是**不能设色、
     *   不能点进料**。Tianshu 的模板里这两个字段是配套的（GFB00 + ABS），
     *   直接照抄它，别自作聪明换名字。 */
    memcpy(TPL_FIL + 27, "ABS", 3);
    TPL_FIL[59] = 0xDD;
    TPL_FIL[60] = 0xB1;
    TPL_FIL[61] = 0xD4;
    TPL_FIL[62] = 0xFF;
    TPL_FIL[79] = 0x18;                      /* 280℃ */
    TPL_FIL[80] = 0x01;
    TPL_FIL[81] = 0xF0;                      /* 240℃ */
    TPL_FIL[82] = 0x00;
}

/** ⑤ MC 在线应答：把请求里的 6 字节原样回，首字节是台号 */
static uint8_t s_mc[6];

/** 运动状态应答（0x03 短帧，44 字节）。内容按"空闲、未动作"填，见文件头。 */
static uint8_t s_cxx[44] = {
    0x3D, 0xC0, 0x2C, 0x00, 0x03,
    /* ---- 以下 33 字节照参考实现的模板，索引 5 起 ---- */
    0x00, 0x00, 0x00, 0x00,       /* [5..8]  flagx / channel */
    0x00, 0x00, 0x00, 0x00,       /* [9..12] meters = 0.0f（-1.0f 是"无料"哨兵，会让打印机犹豫） */
    0x00, 0x00,                   /* [13..14] pressure */
    0x00, 0xC0, 0x00, 0xC0, 0x5D, 0xFF, 0xFC, 0xFF,
    0xFC, 0xFF, 0x00, 0x00, 0x44, 0x00, 0x55, 0xC1,   /* [29]=left_char */
    0xC3, 0xEC, 0xBC, 0x01, 0x01, 0x01, 0x01,
    /* ---- 尾部 ---- */
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00                   /* CRC16，tx_frame 填 */
};

/** 运动状态应答（0x04 长帧，60 字节）。同上，索引 17 起 33 字节模板。 */
static uint8_t s_dxx[60] = {
    0x3D, 0xC0, 0x3C, 0x00, 0x04,
    0x00, 0x75, 0x01, 0x11,
    0x04, 0x04, 0x04, 0xFF,
    0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,   /* [17..24] meters=0.0f */
    0x00, 0x00, 0x00, 0xC0, 0x00, 0xC0, 0x5D, 0xFF,
    0xFC, 0xFF, 0xFC, 0xFF, 0x00, 0x00, 0x44, 0x00,
    0x55, 0xC1, 0xC3, 0xEC, 0xBC, 0x01, 0x01, 0x01,   /* [41]=left_char */
    0x01,
    0x00, 0x00, 0x00, 0x00,
    0xFF, 0xFF, 0xFF, 0xFF,
    0x00, 0x00                   /* CRC16，tx_frame 填 */
};

/** ② 打印机写耗材信息（0x08）的确认帧，固定 8 字节 */
static const uint8_t TPL_ACK08[8] = {
    0x3D, 0xC0, 0x08, 0x00, 0x08, 0x60, 0x00, 0x00
};

/* ==========================================================================
 * ① 在线检测 / 注册
 * ========================================================================== */
static void on_online_detect(const uint8_t *f, int len)
{
    if (!s_online_seen) {
        s_online_seen = true;
        ams_log("485：收到打印机在线检测（0x05）→ 开始注册 %d 台 AMS",
                AMS_UNITS);
    }
    /* ★ 广播帧的 payload 只有 1 字节 → 整帧 8 字节，别写 len < 9 把它挡掉 */
    if (len < 7) {
        return;
    }

    uint8_t kind = f[5];

    /* ★★ 2026-10-22 深夜新增：打印机的"重新编址"指令（0x05 / 0x03 / 0xFF）★★
     *
     * 最新 BMCU（唯一在 A1 + 1.7.0 上稳定跑出 4 台 16 槽的实现）里，
     * `get_package_online_detect()` 第一件事就是判这一帧：
     *     if (buf[5] == 0x03 && buf[6] == 0xFF) { 全部清零，重新枚举 }
     * 这是打印机**主动**要求所有设备忘掉台号的唯一信号（用户在屏幕上改
     * AMS 配置、或打印机重启时会发）。
     *
     * 有了它，就不需要我上一版那个"总线静默 10 秒就清零"的拍脑袋超时了 ——
     * 那个超时在打印机空闲时会把注册状态清掉，下一帧广播又重报一遍，
     * 打印机看到"新设备接入"→ 重新枚举 → 表现就是**一会连上一会没了**。
     */
    if (kind == 0x03 && len >= 8 && f[6] == 0xFF) {
        s_reg_mask   = 0;
        s_req06_unit = 0xFF;
        ams_log_warn("485：打印机要求重新编址（0x03/0xFF）→ 清零注册状态");
        return;
    }

    if (kind == 0x00) {
        /* 广播：自报家门。
         *
         * ★★ 2026-10-22 深夜 修「一会连上一会又没了」★★
         *
         * 上一版给已注册的台加了"每 15 秒全量重报一次"当自愈。**这是错的**：
         * 最新 BMCU 的写法是
         *     if (have_registered) return;          // 注册成功之后永不重报
         * 它重置注册状态只有两个触发点：打印机发 0x03/0xFF（重新编址），
         * 或者本机 ams.online 变 false。**没有任何定时重报**。
         *
         * 每 15 秒朝总线再喊一次"我是一台新 AMS"，打印机那边就是一次
         * 设备接入/重枚举事件 → 界面上 AMS 反复出现、消失。这和现场
         * "连接不稳定、一会连上一会又没有了"完全吻合。
         *
         * 所以现在：**已注册的台一律静默**，只回未注册的台；台间仍隔 5ms
         * （两台共用一个 UART，不能同时发）。
         */
        bool sent_any = false;
        for (int u = 0; u < AMS_UNITS; u++) {
            if (s_reg_mask & (1u << u)) {
                continue;                      /* 已注册 → 闭嘴，别刷总线 */
            }
            if (sent_any) {
                esp_rom_delay_us(5000);        /* 台间 5ms，否则两台撞车 */
            }
            send_register(0x00, (uint8_t)u);
            sent_any = true;
        }
        if (!sent_any) {
            log_limited("广播(已注册,静默)", f, len, &s_cnt_reg);
        } else {
            log_limited("注册广播", f, len, &s_cnt_reg);
        }
        return;
    }

    if (kind == 0x01) {
        /* 点名：只能由对应台号应答。
         *
         * ★ 2026-10-23：台号 bit7 是标志位，要 mask 掉。
         *   现场 /bus485raw 里这台打印机点名发的是 0x80 / 0x81 / 0x83，
         *   直接拿 f[6] 比 `< AMS_UNITS` 全部判"越界"→ 一帧都不回。
         *   mask 0x7F 之后：0x80→台0、0x81→台1、0x83→台3（超出 2 台，丢弃）。
         *   台 0 本来就没问题（0x00 也照样回），受影响的是台 1 ——
         *   它的序列号/版本一直拿不到，很可能就是这个原因。
         *
         * 多发一帧的代价（约 290µs 总线时间）远小于"该回不回"。 */
        if (len < 8) {
            return;
        }
        uint8_t u = (uint8_t)(f[6] & 0x7F);
        if (u >= AMS_UNITS) {
            log_limited("点名(台号越界)", f, len, &s_cnt_reg);
            return;
        }
        uint8_t b[REG_FRAME_LEN];
        memset(b, 0, sizeof(b));
        b[0] = BUS_HEAD;
        b[1] = 0xC0;
        b[4] = 0x05;
        memcpy(b + 5, REG_ID, sizeof(REG_ID));
        b[5] = 0x01;
        b[6] = u;

        /* ★★ 2026-10-22 深夜：改为"把打印机回传的身份原样抄回去"。★★
         *
         * 旧写法发自己的 REG_ID 模板，再拿模板和打印机回传的 20 字节比对。
         * 问题在于模板是从某个特定固件版本抄来的 —— 打印机固件一换，
         * 它回传的身份就可能不同，一比对就永远"待核对"，第二台也就永远
         * 注册不上。BMCU370 的做法是
         *     memcpy(online_detect_res + 7, buf + 7, 20);
         * 也就是**打印机是权威，它回传什么我们就回什么**，天然一致。
         */
        if (len >= 27) {
            memcpy(b + 7, f + 7, 20);
        }
        tx_frame(b, REG_FRAME_LEN);

        if (len >= 27) {
            if (!(s_reg_mask & (1u << u))) {
                s_reg_mask |= (uint8_t)(1u << u);
                ams_log("485：★ AMS #%d 注册通过（打印机已回传身份数据）", u + 1);
            }
        } else if (!(s_reg_mask & (1u << u))) {
            log_limited("点名应答(待核对)", f, len, &s_cnt_reg);
        }
        return;
    }

    log_limited("在线检测(未知子类型)", f, len, &s_cnt_reg);
}

/* ==========================================================================
 * ①b 0x06 REQx6 —— 打印机逐个台号握手
 * ==========================================================================
 * ★★ 2026-10-22 晚 补：这是"第二台 AMS 不出现"的另一半原因。★★
 *
 * 旧版把 0x06 归到"不应答"里（和 0x20 心跳、0x07 NFC 一起），理由是
 * "实测能用的参考实现也不应答"。但 Tianshu 的 send_for_Fxx() 里明明有：
 *
 *     else if (buf[4] == 0x06) {
 *         uint8_t ams_num = buf[5];              // ← 请求里带的是**台号**
 *         unsigned char F06_res[] = {
 *             0x3D, 0xC0, 0x0B, 0xA0, 0x06, 0x01, 0, 0, 0, 0, 0, 0 };
 *         F06_res[1] = 0xC0 | (package_num << 3);
 *         F06_res[5] = ams_num;                  // ← 回包第 5 字节 = 台号
 *         package_send_with_crc(F06_res, sizeof(F06_res));
 *     }
 *
 * 0x05 广播只能让打印机"发现有东西在"，**逐台的归属确认靠 0x06**。
 * 不回这一帧 → 打印机只知道第 1 台 → 界面上只有 AMS-A 的 4 个槽。
 */
static void on_req06(const uint8_t *f, int len)
{
    if (len < 6) {
        return;
    }
    uint8_t u = f[5];
    if (u >= AMS_UNITS) {
        log_limited("0x06(台号越界)", f, len, &s_cnt_req06);
        return;
    }
    /* ★ 照 Tianshu 的做法：第一次收到 0x06 把台号"钉住"，之后一律用它。
     *   参考实现里是
     *       if (res_for_06_num == 0xFF) res_for_06_num = ams_num;
     *       else ams_num = res_for_06_num;
     *   看着别扭，但那是真机上能认到 4 台 AMS 的写法 —— 这一步打印机可能
     *   连发多帧不同台号，全部用同一台号回才是它预期的行为。 */
    if (s_req06_unit == 0xFF) {
        s_req06_unit = u;
    } else {
        u = s_req06_unit;
    }

    uint8_t b[12];
    memset(b, 0, sizeof(b));
    b[0] = BUS_HEAD;
    b[1] = 0xC0;
    b[2] = 0;                              /* LEN，tx_frame_ex 填 */
    b[3] = 0;                              /* CRC8，tx_frame_ex 填 */
    b[4] = 0x06;
    b[5] = u;                              /* 台号：照 Tianshu 放在索引 5 */
    /* ★★ 2026-10-22 深夜：LEN 字段改回 Tianshu 的 0x0B ★★
     *
     * 上一版我"顺手指正"了这个自相矛盾的地方：模板 [2] 写 0x0B（11），
     * 却实发 12 字节，我改成了"LEN = 实际总长 12"。
     *
     * 这个改动是**自作聪明**，很可能就是"第二台永远不出现"的原因之一：
     * 0x06 是打印机**逐台**握手确认的帧；只认到第 1 台，正好对应
     * "只按我们自己认为对的方式回了第 1 次"。
     * 参考实现在同一台打印机（固件 1.7.0）上能认到 4 台，它的 0x0B
     * 就是"能用"的证据 —— "看着不合理"不是改它的理由，我手上没有
     * 打印机的解析代码。所以按字节原样复刻：实发 12、LEN 写 0x0B。 */
    tx_frame_ex(b, (int)sizeof(b), 0x0B);
    log_limited("0x06 握手", f, len, &s_cnt_req06);
}

/* ==========================================================================
 * ③ 版本 / 序列号
 * ========================================================================== */
static void on_version(uint16_t type, uint16_t target, uint16_t my_addr,
                       uint16_t pkg, const uint8_t *pl, int plen)
{
    uint8_t unit = 0;

    if (type == 0x402) {
        /* 序列号查询：台号在 payload[33] */
        if (plen > 33) {
            unit = pl[33];
        } else if (plen > 0) {
            unit = pl[0];
        }
        if (unit >= AMS_UNITS) {
            unit = 0;
        }
        uint8_t b[SN_FRAME_LEN];
        build_sn(b, unit);
        send_long(type, target, my_addr, pkg, b, SN_FRAME_LEN);
    } else {
        if (plen > 0) {
            unit = pl[0];
        }
        if (unit >= AMS_UNITS) {
            unit = 0;
        }
        uint8_t b[VER_FRAME_LEN];
        build_ver(b, unit, my_addr);
        send_long(type, target, my_addr, pkg, b, VER_FRAME_LEN);
    }

    if (plen > 1) {
        log_limited("版本/序列号查询", pl, plen, &s_cnt_ver);
    }
}

/* ==========================================================================
 * ④ 槽位耗材信息
 * ========================================================================== */
static void on_filament_info(uint16_t target, uint16_t my_addr, uint16_t pkg,
                             uint16_t type, const uint8_t *pl, int plen)
{
    if (plen < 2) {
        return;
    }
    uint8_t unit = pl[0];
    uint8_t slot = pl[1];
    if (unit >= AMS_UNITS || slot >= AMS_SLOTS) {
        return;
    }

    static uint8_t b[FIL_FRAME_LEN];
    memcpy(b, TPL_FIL, sizeof(b));
    b[0] = unit;
    b[1] = slot;

    /* 材料名跟着 ID 走（GFB00 = ABS）。打印机拿 ID 判类型、名字做显示，
     * 两者不一致会把这个槽判成"参数异常" → 不能设色 / 不能点进料。 */
    memcpy(b + 27, "ABS", 3);
    b[30] = 0x00;

    int ch = unit * AMS_SLOTS + slot;      /* 物理通道（0 起） */
    if (ch < BOARD_CHANNEL_COUNT) {
        uint32_t rgb = config_get()->color_list[ch];
        b[59] = (uint8_t)((rgb >> 16) & 0xFF);
        b[60] = (uint8_t)((rgb >> 8) & 0xFF);
        b[61] = (uint8_t)(rgb & 0xFF);
        b[62] = 0xFF;

        /* ★★ 温度范围必须有合理值（2026-10-23）★★
         * 原来直接抄 config 的 temper，而它出厂是 0 —— 于是报给打印机的是
         * temp_max=0 / temp_min=0。APP 拿到 0℃ 的耗材参数会判该槽不可用，
         * 表现就是**槽位点不动、不能设色**。Tianshu 的模板固定 280/240，
         * 这里只在"明显不合理"时才兜底，不动用户自己设的值。 */
        int t = config_get()->temper[ch];
        if (t < 150 || t > 330) {
            t = 280;
        }
        b[79] = (uint8_t)(t & 0xFF);
        b[80] = (uint8_t)((t >> 8) & 0xFF);
        int lo = (t > 20) ? (t - 20) : 0;
        b[81] = (uint8_t)(lo & 0xFF);
        b[82] = (uint8_t)((lo >> 8) & 0xFF);
    }

    /* ★ 2026-10-23：type 原样回。打印机会用 0x211 也会用 **0x0411**
     *   （AMS 2 Pro / AMS HT 的新版耗材信息查询）来问，回错 type 会被丢。 */
    send_long(type, target, my_addr, pkg, b, FIL_FRAME_LEN);
    log_limited("耗材信息查询", pl, plen, &s_cnt_fil);
}

/* ==========================================================================
 * ⑤ MC 在线
 * ========================================================================== */
static void on_mc_online(uint16_t target, uint16_t my_addr, uint16_t pkg,
                         const uint8_t *pl, int plen)
{
    /* ★ 照 Tianshu 的 long_packge_MC_online = {0,0,0,0,0,0}，只把 [0] 改成台号。
     *   旧版是把请求里的 6 字节原样抄回去 —— 能通，但不是参考实现的行为。
     *   既然现场证据只有 Tianshu（同一台打印机认到 4 台），就按它的字节来。 */
    int n = (plen < 6) ? plen : 6;
    memset(s_mc, 0, sizeof(s_mc));
    if (n > 0 && pl[0] < AMS_UNITS) {
        s_mc[0] = pl[0];
    }
    (void)plen;
    send_long(0x21A, target, my_addr, pkg, s_mc, (int)sizeof(s_mc));
    log_limited("MC 在线", pl, plen, &s_cnt_mc);
}

/* ==========================================================================
 * 运动指令（cmd 0x03 → Cxx / cmd 0x04 → Dxx）
 * ==========================================================================
 *
 * ★★ 2026-10-23 第十六轮：上一版把 send_out 当成了退料 ★★
 *
 *   现场：槽位显示已经正常了，但点「进料」电机和电磁一动不动。两个原因：
 *     ① 网页上的「485 运动执行」开关是关的（act485=0），固件明确拒绝驱动；
 *     ② 更致命 —— 语义翻译反了。打印机点进料发的是 flags=0x03/motion=0x00，
 *        参考实现里这是 **send_out（正转送料）**，我们却映射成了 ams_post_retract。
 *        而 0x07/0x7F 才是 on_use（挤出机在拉、AMS 该停），我们反倒当成了进料。
 *
 *   证据（Tianshu AMCU.cpp）：
 *     case need_send_out: motorCmd(cur, 1)   ← 正转，进料
 *     case need_pull_back: motorCmd(cur,-1)  ← 反转，退料
 *     default(waiting/on_use): motorCmd(cur, 0) ← 停
 *
 * 应答字段也照参考实现走真实状态 —— 恒报"空闲"的话，打印机即使看到电机转
 * 也会判 AMS 没动，直接放弃：
 *     use_flag  ← 有通道送/退料 0x02；on_use 系 0x04；否则 0x00（Cxx 兜底 0x02）
 *     pressure  ← 送/退料 0x4700；on_use 系 0x2B00；空闲 0xFFFF
 *     left_char ← 每槽 2 bit：0b01 有料静止、0b11 有料且运动中
 */

/* 有通道在送/退料 → 0x02；在 on_use 系 → 0x04；全空闲 → 0x00 */
static uint8_t mot_use_flag(void)
{
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (s_mot[i] == MOT_IDLE) {
            continue;
        }
        if (s_mot[i] == MOT_SEND_OUT || s_mot[i] == MOT_PULL_BACK) {
            return 0x02;
        }
        return 0x04;
    }
    return 0x00;
}

/* 压力值：Tianshu（唯一实测多台可用的参考）get_pressure_value() 的真值：
 *   send_out        0x4700
 *   before_on_use   0x4700   ★ 关键：prev==send_out 时报 0x4700（料还在挤出口、AMS 仍在推）
 *   on_use          0x2B00
 *   stop_on_use     0x2B00
 *   before_pull_back 0x2B00
 *   pull_back       0x4700
 *   idle            0xFFFF
 *
 * ★★ 0.25.0 的 bug：把 before_on_use 也归进 0x2B00，导致预送阶段打印机误以为
 *   「料已就位」→ 让挤出机正转拉料 → 但料实际还没送到（预送阶段仍在推）→ 空转
 *   → 判「挤出失败」→ 到位后 226ms 就发退料 → 报 0700-8003。 */
static uint16_t mot_pressure(int ch)
{
    if (ch < 0 || ch >= BOARD_CHANNEL_COUNT) {
        return 0xFFFF;
    }
    switch (s_mot[ch]) {
    case MOT_SEND_OUT:
    case MOT_BEFORE_ON_USE:      /* ★ 预送仍报 0x4700，同 send_out */
    case MOT_PULL_BACK:
        return 0x4700;
    case MOT_ON_USE:
    case MOT_STOP_ON_USE:
    case MOT_BEFORE_PULL_BACK:
        return 0x2B00;
    default:
        return 0xFFFF;
    }
}

/* 台内 4 槽的"剩余料"位图：每槽 2 bit，0b01 有料静止 / 0b11 有料且运动中 */
static uint8_t mot_left_char(int unit)
{
    uint8_t d = 0;
    for (int i = 0; i < AMS_SLOTS; i++) {
        int ch = unit * AMS_SLOTS + i;
        int bits = 1;
        if (ch < BOARD_CHANNEL_COUNT && s_mot[ch] != MOT_IDLE) {
            bits = 3;
        }
        d = (uint8_t)(d | (bits << (i * 2)));
    }
    return d;
}

static const char *mot_name(uint8_t m)
{
    switch (m) {
    case MOT_SEND_OUT:         return "进料(send_out)";
    case MOT_BEFORE_ON_USE:    return "预送(before_on_use)";
    case MOT_ON_USE:           return "随动(on_use)";
    case MOT_STOP_ON_USE:      return "停止(stop_on_use)";
    case MOT_BEFORE_PULL_BACK: return "准备退料";
    case MOT_PULL_BACK:        return "退料(pull_back)";
    default:                   return "空闲(idle)";
    }
}

/* 把一条通道推到 new_mot；返回 true 表示状态真的变了 */
static bool mot_set(int ch, uint8_t new_mot, int64_t now_us)
{
    if (ch < 0 || ch >= BOARD_CHANNEL_COUNT) {
        return false;
    }
    if (s_mot[ch] == new_mot) {
        return false;
    }
    ams_log("485 运动：通道%d %s → %s", ch + 1,
            mot_name(s_mot[ch]), mot_name(new_mot));
    s_mot[ch] = new_mot;
    if (new_mot == MOT_SEND_OUT) {
        s_sendout_us[ch] = (uint32_t)(now_us / 1000);
    }
    return true;
}

/*
 * 状态变了才投递物理动作 —— 打印机每 100ms 就发一帧，每帧都投会把队列刷爆。
 * 开关关着时只更新状态、不发指令（网页上打开立即生效）。
 */
static void mot_apply(int ch)
{
    if (ch < 0 || ch >= BOARD_CHANNEL_COUNT) {
        return;
    }
    if (!config_get_485_actuate()) {
        return;
    }
    switch (s_mot[ch]) {
    case MOT_SEND_OUT:
        /* ★ 进料：只有 send_out(0x03/0x00) 才正转送料。 */
        if (!ams_post_485_drive(ch, 1)) {
            ams_log_warn("485：通道%d 正忙，送料指令被丢弃", ch + 1);
        }
        break;
    case MOT_BEFORE_ON_USE:
        /* ★★★ 预送(before_on_use 0x09/0xA5) → 停电机（waiting）★★★
         *
         *   以 Tianshu（唯一在 A1+1.7.0 实测多台可用的实现，且电机是 DRV8833
         *   简单电机、无编码器压力闭环，与我们的硬件一致）的 AMCU.cpp 为准：
         *       AMCU_motion() 的 switch(act)：
         *         need_send_out(send_out)              → motorCmd(cur, 1) 正转
         *         need_pull_back(pull_back/before_pb)  → motorCmd(cur, -1) 反转
         *         waiting(before_on_use/on_use/stop_on_use/idle) → motorCmd(cur, 0) 停
         *
         *   也就是说「预送 before_on_use」→ waiting → **停电机**，让挤出机自己
         *   把料拉进热端。BMCU 的 Motion_control.cpp 里 before_on_use 走 hold_load
         *   正转，那是因为 BMCU 有 AS5600 编码器 + PID 压力闭环（能"顶住不放"）；
         *   我们是简单电机，照 Tianshu 停电机才对。0.25.0 把预送当正转是错的。 */
        ams_post_stop();
        break;
    case MOT_PULL_BACK:
        /* ★ 退料：反转持续退料，直到打印机发「停止」指令停。 */
        if (!ams_post_485_drive(ch, -1)) {
            ams_log_warn("485：通道%d 正忙，退料指令被丢弃", ch + 1);
        }
        break;
    case MOT_BEFORE_PULL_BACK:
        /* ★★ 准备退料(before_pull_back 0x09/0x3F) → 反转退料，不是停电机 ★★
         *
         *   BMCU 官方 Motion_control.cpp 的 motor_motion_switch()：
         *       case before_pull_back:
         *           MOTOR_CONTROL[num].set_motion(
         *               filament_motion_before_pull_back, 300, time_now);
         *   filament_motion_before_pull_back 是带编码器反馈的反转退料动作
         *   （累计退料距离，最多 2.0f 距离单位）。0.24.0 把它当「停电机等
         *   下一帧 0x03/0xFF」是错的 —— 现场抓包：before_pull_back(0x09/3F)
         *   到 pull_back(0x03/0xFF) 隔 2.5 秒，这 2.5 秒电机停着不退料，
         *   打印机等不到料退出，进一步加剧退料失败。 */
        if (!ams_post_485_drive(ch, -1)) {
            ams_log_warn("485：通道%d 正忙，准备退料指令被丢弃", ch + 1);
        }
        break;
    case MOT_ON_USE:
    case MOT_STOP_ON_USE:
        /* ★★ 2026-10-23 进料到位 → 只停电机，不蠕动 ★★
         *
         *   现场实测：到位(stop_on_use)后仅 0.9 秒，打印机就发「准备退料」，
         *   然后「退料」。而之前这里做 5 次 × 1 秒的**阻塞蠕动**，正好撞上
         *   打印机这个节拍 —— 该退料时电机还在正转蠕动，料被反复送进又退出，
         *   就是「进料一点马上退料、一直往复」的直接原因。
         *
         *   BMCU 参考实现里 stop_on_use / on_use 都只 set_motion(stop) 停电机，
         *   根本没有「到位后蠕动」这个动作（蠕动是从 MQTT 换料流程误搬来的）。
         *   485 协议下「料咬稳」由打印机自己控制（它退料后自己会重新进料），
         *   固件到位只该停电机、把控制权立即交还打印机。 */
        ams_post_stop();
        break;
    default:
        /* idle → 停 */
        ams_post_stop();
        break;
    }
}

/* 打印机点名了某个槽 → 单通道状态机。返回 true = 状态变了 */
static bool mot_step_channel(int ch, uint8_t flags, uint8_t motion, int64_t now_us)
{
    /* ★★ 2026-10-24 用户拍板：Cxx/Dxx 都进状态机 + 现场 flags 恒 0x01 ★★
     *
     *   Tianshu（BambuBus.cpp:251~255）判据：
     *       send_out       = (0x03, 0x00)
     *       before_on_use  = (0x09, 0xA5)   （本固件曾扩展到 0x7F）
     *       stop_on_use    = (0x07, 0x00)
     *       on_use         = (0x07, 0x7F)
     *       before_pullb   = (0x09, 0x3F)
     *
     *   现场 /bus485raw 抓到的帧（A系列原始数据包 6-7 系列，Tianshu 实证退料
     *   `0x09/0x3F` 帧之外，A1 打印机实际下发运动帧时 statu_flag 恒为 0x01）：
     *       Cxx st=0x01 ch=0 motion=0x00 → 进料意图
     *       Cxx st=0x01 ch=0 motion=0x3F → 退料意图
     *   按 Tianshu 判据 flags=0x01 一条都接不住 → 状态机永停 MOT_IDLE →
     *   我方回 use_flag=0x0F / pressure=0 / left_char=0xFF 的矛盾应答 →
     *   打印机判「进料失败」→ 退回外挂 + 进退往复。
     *
     *   用户拍板「保持 Tianshu 判据」但现场 flags=0x01 必须也接住，否则电机
     *   还是不动。故进料 / 退料两条主路径放宽为同时接受 0x01：
     *     is_send_out      = flags==0x03 && motion==0x00   (Tianshu)
     *                        || flags==0x01 && motion==0x00 (现场)
     *     is_before_pullb  = flags==0x09 && motion==0x3F   (Tianshu)
     *                        || flags==0x01 && motion==0x3F (现场)
     *   before_on_use / on_use / stop_on_use 保持 Tianshu 判据（现场没抓到
     *   这两条意图的 flags=0x01 帧，且 0x07 系有 5 秒闸门兜底，不影响主路径）。 */
    const bool is_send_out      = ((flags == 0x03) || (flags == 0x01)) && (motion == 0x00);
    const bool is_before_on_use = (flags == 0x09) && (motion == 0x7F || motion == 0xA5);
    const bool is_stop_on_use   = (flags == 0x07) && (motion == 0x00);
    const bool is_on_use        = (flags == 0x07) && (motion == 0x7F);
    const bool is_before_pullb  = ((flags == 0x09) || (flags == 0x01)) && (motion == 0x3F);

    const bool allow_any  = (s_now_mot_ch < 0) || (s_now_mot_ch == ch);
    const bool allow_stop = (s_now_mot_ch == ch);
    bool changed = false;

    /* ★ 退料结束边沿：MOT_PULL_BACK 期间收到「没料」信号（flags=0x09 且
     *   motion≠0x3F）→ 立即停电机、断离合、回空闲。
     *   ★★ 2026-10-23 用户明确要求：删除退料尾拉 —— 补拉会跟打印机自己
     *   收料动作打架，导致「进料退料循环」。退料实际长度由打印机
     *   change_filament_gcode 决定。*/
    if (s_mot[ch] == MOT_PULL_BACK && !is_send_out &&
        flags == 0x09 && motion != 0x3F) {
        if (mot_set(ch, MOT_IDLE, now_us)) {
            if (config_get_485_actuate()) {
                ams_post_stop();
            }
            s_now_mot_ch = -1;
        }
        return true;
    }

    if (is_send_out) {
        /* 送料不受 allow_any 限制：允许打印机直接改到另一路去送 */
        if (s_now_mot_ch >= 0 && s_now_mot_ch != ch) {
            int prev = (int)s_now_mot_ch;
            if (mot_set(prev, MOT_IDLE, now_us)) {
                if (config_get_485_actuate()) {
                    ams_post_stop();
                }
            }
        }
        s_now_mot_ch = (int8_t)ch;
        changed = mot_set(ch, MOT_SEND_OUT, now_us);
    } else if (is_before_on_use && allow_any) {
        s_now_mot_ch = (int8_t)ch;
        changed = mot_set(ch, MOT_BEFORE_ON_USE, now_us);
    } else if (is_on_use && allow_any) {
        /* ★ 正在 send_out 时先让它把料推够 5 秒再转 on_use ——
         *   参考实现同样有这道 5 秒闸门，否则刚送一点点就被打断。 */
        if (s_mot[ch] == MOT_SEND_OUT) {
            uint32_t now_ms = (uint32_t)(now_us / 1000);
            if ((uint32_t)(now_ms - s_sendout_us[ch]) < 5000) {
                s_now_mot_ch = (int8_t)ch;
                return false;
            }
        }
        s_now_mot_ch = (int8_t)ch;
        changed = mot_set(ch, MOT_ON_USE, now_us);
    } else if (is_stop_on_use && allow_stop) {
        /* ★ 到位指令：立即停，不再蠕动/尾拉。后续续送/断料由该通道
         *   「开始/停止」双微动（常开 4.7k + 常闭 10k 串联）判定。
         *   ★★ 2026-10-24 用户拍板：到位开窗，ams_task 轮询 poll_485_decide
         *   按双微动决定续送（开始微动）或断料（停止微动），6s 超时收手。 */
        if (s_mot[ch] == MOT_SEND_OUT || s_mot[ch] == MOT_BEFORE_ON_USE) {
            changed = mot_set(ch, MOT_STOP_ON_USE, now_us);
            if (changed && config_get_485_actuate()) {
                ams_485_enter_decide_window(ch);
            }
        }
    } else if (is_before_pullb && allow_stop) {
        if (s_mot[ch] == MOT_ON_USE || s_mot[ch] == MOT_BEFORE_ON_USE ||
            s_mot[ch] == MOT_STOP_ON_USE) {
            changed = mot_set(ch, MOT_BEFORE_PULL_BACK, now_us);
        }
    }

    if (changed) {
        mot_apply(ch);
    }
    return changed;
}

/* 槽号 = 0xFF（打印机没点名）→ 全局状态机 */
static void mot_step_global(uint8_t flags, uint8_t motion, int64_t now_us)
{
    /* ★ 全局"空闲"信号：flags=0x01 && motion=0x00（site 实证的全局 idle 轮询）
     *   → 所有通道回 idle。放在最前，避免被下面的 0x01 运动放宽误判。 */
    if (flags == 0x01 && motion == 0x00) {
        bool any = false;
        for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
            any = mot_set(i, MOT_IDLE, now_us) || any;
        }
        if (any) {
            s_now_mot_ch = -1;
            if (config_get_485_actuate()) {
                ams_post_stop();
            }
        }
        return;
    }

    /* ★ 退料（before_pull_back）：Tianshu 判据 flags=0x09 && motion=0x3F，
     *   ★★ 2026-10-24 用户拍板放宽 ★★ 现场 flags 恒 0x01，退料意图同放宽到 0x01。 */
    if (((flags == 0x09) || (flags == 0x01)) && motion == 0x3F) {
        if (s_now_mot_ch >= 0 && s_mot[s_now_mot_ch] == MOT_ON_USE) {
            int ch = (int)s_now_mot_ch;
            if (mot_set(ch, MOT_BEFORE_PULL_BACK, now_us)) {
                mot_apply(ch);
            }
        }
        return;
    }

    /* ★ 进料（send_out）全局：Tianshu 判据 flags=0x03 && motion=0x00，
     *   ★★ 2026-10-24 用户拍板放宽 ★★ 现场 flags 恒 0x01，进料意图同放宽。
     *   "进料但未点名"的全局信号：
     *     · 当前通道在"准备退料" → 正式退料
     *     · 当前通道在"到位停止" → 直接退料（STOP 是明确到位态，不需 5s 闸门） */
    if (((flags == 0x03) || (flags == 0x01)) && motion == 0x00) {
        if (s_now_mot_ch >= 0 && s_mot[s_now_mot_ch] == MOT_BEFORE_PULL_BACK) {
            int ch = (int)s_now_mot_ch;
            if (mot_set(ch, MOT_PULL_BACK, now_us)) {
                mot_apply(ch);
            }
        } else if (s_now_mot_ch >= 0 && s_mot[s_now_mot_ch] == MOT_STOP_ON_USE) {
            int ch = (int)s_now_mot_ch;
            if (mot_set(ch, MOT_PULL_BACK, now_us)) {
                mot_apply(ch);
            }
        }
        return;
    }

    /* 其它组合：当前通道正在 on_use 系时别动它（参考实现直接 return） */
    if (s_now_mot_ch >= 0) {
        uint8_t m = s_mot[s_now_mot_ch];
        if (m == MOT_ON_USE || m == MOT_BEFORE_ON_USE || m == MOT_STOP_ON_USE) {
            return;
        }
    }
    bool any = false;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        any = mot_set(i, MOT_IDLE, now_us) || any;
    }
    if (any) {
        s_now_mot_ch = -1;
        if (config_get_485_actuate()) {
            ams_post_stop();
        }
    }
}

static void handle_motion(const uint8_t *f, int len, bool short_frame,
                          uint16_t target, uint16_t pkg)
{
    s_cnt_motion++;
    if (s_cnt_motion <= 3 || (s_cnt_motion % 200) == 0) {
        log_frame(short_frame ? "运动(短)" : "运动(长)", f, len);
    }

    /* 短帧：cmd 在 [4]，payload 从 [5] 起
     * 长帧：type 在 [11..12]，payload 从 [13] 起 —— 别拿 f[4] 当 cmd
     *       （长帧的 [4..5] 是包长低/高字节） */
    const int off = short_frame ? 5 : 13;
    const int cmd = short_frame
                    ? ((len > 4) ? f[4] : 0)
                    : ((len > 12) ? (f[11] | ((int)f[12] << 8)) : 0);

    /* ★★ 字段偏移照最新 BMCU（_ref/bmcu-pj 的 bambu_bus_ams.cpp）★★
     *
     *   两台 A1 的实测 /bus485raw 双向帧证明：真正下发「进料/退料」的是
     *   Cxx（cmd=0x03），不是 Dxx。两者的字段顺序不一样：
     *
     *     Cxx(cmd 0x03, 12B)：[5]=台号 [6]=statu_flag [7]=channel(槽) [8]=motion_flag
     *     Dxx(cmd 0x04, 13B)：[5]=台号 [6]=statu_flag [7]=motion_flag [9]=channel(槽)
     *
     *   实测信号序列（一次完整进料→退料）：
     *     Cxx st=0x03 ch=0 motion=0x00  → send_out 进料
     *     Cxx st=0x07 ch=0 motion=0x7F  → on_use 到位随动
     *     Cxx st=0x07 ch=0 motion=0x00  → stop_on_use 到位停止
     *     Cxx st=0x09 ch=0 motion=0x3F  → before_pull_back 准备退料
     *     Cxx st=0x03 ch=FF motion=0x00 → pull_back 正式退料
     *
     *   而 Dxx 的 statu_flag 全程恒 0x01（空闲轮询），从不变 —— 它只是状态查询。
     */
    const uint8_t unit = (len > off + 0) ? f[off + 0] : 0xFF;
    uint8_t flags, motion, slot;
    if (cmd == 0x03) {
        /* Cxx：flags=[6] channel=[7] motion=[8] */
        flags  = (len > off + 1) ? f[off + 1] : 0;
        slot   = (len > off + 2) ? f[off + 2] : 0xFF;
        motion = (len > off + 3) ? f[off + 3] : 0;
    } else {
        /* Dxx：flags=[6] motion=[7] slot=[9] */
        flags  = (len > off + 1) ? f[off + 1] : 0;
        motion = (len > off + 2) ? f[off + 2] : 0;
        slot   = (len > off + 4) ? f[off + 4] : 0xFF;
    }

    /* 打印机点了我们没有的台 —— 原样回一帧（维持总线节奏），但不跑状态机，
     * 免得把"台2"的状态记到"台1"头上。 */
    bool unit_ok = (unit < AMS_UNITS);
    uint8_t ru = unit_ok ? unit : 0;

    /* ★ 每次命令组合变化都记一行：现场点「进料」时，这一行就是唯一证据 */
    if ((cmd == 0x03 || cmd == 0x04) &&
        (unit != s_noted_mot_ch || flags != s_noted_flags ||
         motion != s_noted_motion || slot != s_noted_slot)) {
        s_noted_mot_ch = unit;
        s_noted_flags  = flags;
        s_noted_motion = motion;
        s_noted_slot   = slot;
        ams_log("485 运动命令：台%d %s flags=0x%02X motion=0x%02X slot=%s",
                (int)unit + 1, unit_ok ? "" : "(无此台)",
                flags, motion,
                (slot < AMS_SLOTS) ? "0123" + slot : "0xFF/其它");
    }

    /* ---- 状态机：先更新状态，再照新状态组应答，最后才真正驱动电机 ---- */
    /* ★★★ 2026-10-23 决定性修正：状态机只在 Dxx(cmd=0x04) 跑，Cxx 只应答 ★★★
     *
     *   Tianshu（github.com/hu504011383/Tianshu，唯一在 A1+1.7.0 实测多台可用的
     *   实现）的 BambuBus_run()：
     *       case BambuBus_package_filament_motion_short (Cxx cmd=0x03):
     *           send_for_Cxx(buf, len);        // 只应答，不跑状态机
     *       case BambuBus_package_filament_motion_long (Dxx cmd=0x04):
     *           send_for_Dxx(buf, len);        // 才跑 process_motion_state_machine
     *
     *   现场抓包：打印机**成对**发 Cxx + Dxx（每帧 Cxx 后面紧跟一帧字段相同的 Dxx）。
     *   两者都进状态机会导致同一个动作被处理两次，虽然 mot_set 的 changed 守卫能
     *   消解"重复驱动"，但会引入 Cxx/Dxx 字段在边界时刻不一致时的竞态，表现就是
     *   打印机「一会进料一会退料、犹豫不决」。
     *
     *   0.21.0 曾因为"只让 Cxx 进状态机导致电机不停"而改成两者都进 —— 那是把
     *   "只跑 Cxx"错当成了唯一选择。
     *
     * ★★★ 2026-10-24 用户拍板：Cxx(0x03) 与 Dxx(0x04) 都驱动状态机 ★★★
     *
     *   现场 /bus485raw 实测：打印机把「进料/退料」意图放在 Cxx(cmd=0x03) 里下发
     *   （statu_flag 恒 0x01、motion_flag 在 0x00/0x3F 之间切换），Dxx(cmd=0x04)
     *   只是空闲轮询、字段恒不变 —— 单跑 Dxx 时 Cxx 携带的运动意图被完全忽略，
     *   状态机永停 MOT_IDLE，电机永远不动，打印机退回外挂 + 抖退。
     *
     *   打印机 Cxx/Dxx 成对发（同字段），若两者都进状态机会双触发。mot_set() 的
     *   "同状态返回 false" 守卫天然消解重复驱动：Cxx 先到达时状态已变，Dxx 紧随
     *   的相同字段不再产生 changed → 不会二次驱动电机。边界时刻字段不一致时
     *   由 5 秒闸门（on_use 在 send_out 未满 5s 不转）+ allow_stop 兜底。
     *   用户已确认采用"两者都进 + Cxx 判据"。
     */
    const int64_t now_us = esp_timer_get_time();
    if ((cmd == 0x03 || cmd == 0x04) && unit_ok) {
        int ch = (int)ru * AMS_SLOTS + (int)((slot < AMS_SLOTS) ? slot : 0);
        if (slot < AMS_SLOTS) {
            mot_step_channel(ch, flags, motion, now_us);
        } else {
            mot_step_global(flags, motion, now_us);
        }
    }

    if (cmd == 0x04) {
        /* ---------- Dxx：60 字节，cmd 必须是 0x04 ---------- */
        const int ask = (slot < AMS_SLOTS) ? ((int)ru * AMS_SLOTS + (int)slot) : -1;
        const uint16_t pres = (ask >= 0) ? mot_pressure(ask) : 0xFFFF;
        const uint8_t  uflag = (ask >= 0) ? mot_use_flag() : 0x00;

        s_dxx[1]  = (uint8_t)(0xC0 | ((s_pkg_num & 0x07) << 3));
        s_dxx[4]  = 0x04;                      /* ★ cmd 必须回 0x04 */
        s_dxx[5]  = ru;                        /* 台号 */
        s_dxx[9]  = 0x0F;                      /* 在位标志：台内 4 槽全在位 */
        s_dxx[10] = 0x0F;
        s_dxx[11] = 0x0F;
        s_dxx[12] = 0xFF;
        s_dxx[13] = 0x00;                      /* NFC 标志 */
        s_dxx[17] = ru;                        /* 台号（第二处） */
        s_dxx[19] = uflag;                     /* ★ 真实 use_flag，不是写死 0 */
        s_dxx[20] = (slot < AMS_SLOTS) ? slot : 0xFF;
        s_dxx[25] = (uint8_t)(pres & 0xFF);    /* ★ 真实压力值，低字节在前 */
        s_dxx[26] = (uint8_t)(pres >> 8);
        s_dxx[41] = mot_left_char((int)ru);    /* ★ 运动中的槽报 0b11 */
        tx_frame(s_dxx, (int)sizeof(s_dxx));
    } else {
        /* ---------- Cxx：44 字节，cmd = 0x03（运动命令的应答）---------- */
        /* 字段照最新 BMCU bambubus_ams_motion_package_struct：
         *   [5]=ams_num [7]=filament_use_flag [8]=filament_channel
         *   [13..14]=pressure [29]=filament_stu_flag(left_char) */
        uint8_t slot3 = slot;                   /* Cxx 的槽号已在解析里取到（[7]=channel） */
        int ask = (slot3 < AMS_SLOTS) ? ((int)ru * AMS_SLOTS + (int)slot3) : -1;
        uint8_t flagx = mot_use_flag();
        if (flagx == 0x00) {
            flagx = 0x02;                      /* ★ 空闲也要报 0x02（参考实现同样兜底） */
        }
        uint16_t pres = (ask >= 0) ? mot_pressure(ask) : 0xFFFF;

        s_cxx[1]  = (uint8_t)(0xC0 | ((s_pkg_num & 0x07) << 3));
        s_cxx[4]  = 0x03;
        s_cxx[5]  = ru;                        /* ★ 台号 */
        s_cxx[7]  = flagx;
        s_cxx[8]  = (slot3 < AMS_SLOTS) ? slot3 : 0xFF;
        s_cxx[13] = (uint8_t)(pres & 0xFF);
        s_cxx[14] = (uint8_t)(pres >> 8);
        s_cxx[29] = mot_left_char((int)ru);
        tx_frame(s_cxx, (int)sizeof(s_cxx));
    }
    s_pkg_num = (uint8_t)((s_pkg_num + 1) & 0x07);
    (void)target;
    (void)pkg;

    if (s_cnt_motion == 1 && !config_get_485_actuate()) {
        ams_log_warn("485：收到运动指令，但「485 运动执行」是关的 —— "
                     "只回状态、电机不会转（网页里可打开）");
    }
}
/* ==========================================================================
 * 帧分发
 * ========================================================================== */
static void handle_short(const uint8_t *f, int len)
{
    int cmd = f[4];
    s_last_cmd = (uint32_t)cmd;

    switch (cmd) {
    case 0x05:
        on_online_detect(f, len);
        break;
    case 0x03: {   /* filament_motion_short */
        handle_motion(f, len, true, 0, 0);
        break;
    }
    case 0x04: {   /* filament_motion_long（个别机型做成短帧） */
        handle_motion(f, len, true, 0, 0);
        break;
    }
    case 0x08:     /* set_filament_info：回固定 8 字节确认 */
        tx_frame((uint8_t *)TPL_ACK08, (int)sizeof(TPL_ACK08));
        log_limited("写耗材信息", f, len, &s_cnt_08);
        break;
    case 0x06:     /* REQx6：打印机逐台握手，必须按点名台号应答 */
        on_req06(f, len);
        break;
    case 0x20:     /* 心跳：打印机只靠长帧判断在线，这里不应答 */
    case 0x07:     /* NFC 检测：本硬件没有 */
        log_limited("短帧(不应答)", f, len, &s_cnt_other);
        break;
    default:
        log_limited("未知短帧", f, len, &s_cnt_other);
        break;
    }
}

static void handle_long(const uint8_t *f, int len)
{
    uint16_t pkg  = (uint16_t)(f[2] | ((uint16_t)f[3] << 8));
    /* ★ 长包地址方向：f[7..8] = target（**我们**），f[9..10] = source（打印机）。
     *   旧版把 f[9..10] 当 target 用，方向记反了 —— 但回包时正好又对上了，
     *   所以一直没暴露。这里改成语义正确的名字，并且把 f[7..8] 当成
     *   "打印机指定我们的地址"记下来，回包时原样用作源地址。 */
    uint16_t peer = (uint16_t)(f[9] | ((uint16_t)f[10] << 8));   /* 打印机地址 */
    uint16_t mine = (uint16_t)(f[7] | ((uint16_t)f[8] << 8));    /* 打印机指定的我方地址 */
    uint16_t type = (uint16_t)(f[11] | ((uint16_t)f[12] << 8));
    const uint8_t *pl = f + 13;
    int plen = len - 15;
    s_last_cmd = (uint32_t)type;

    /* ★★ 2026-10-23 关键修正：源地址**固定 0x0700，绝不跟随打印机** ★★
     *
     * 0.16.0 我把源地址改成"打印机按什么地址问、我们就按什么地址答"
     * （照 BMCU370 的 `data.source_address = printer_data_long.target_address`），
     * 现场立刻从「固件版本不匹配」变成
     *      「检测到 AMS/AMS lite 与设备的连接方式异常」
     *
     * 原因很直接：一旦打印机按 AMS Lite 的地址问，我们就按 lite 的地址回，
     * 可我们的**版本包里写的是 "AMS08"、序列号包也是 AMS 模板** ——
     * 地址说"我是 lite"，身份说"我是 AMS"，自相矛盾。打印机据此判定拓扑
     * 异常，于是整台 AMS 都不给操作（槽位全灰、不能进料、不能设色）。
     *
     * Tianshu（唯一在你这台 A1 + 1.7.0 上实测出 4 台 16 槽的实现）写得很死：
     *     // 始终使用AMS08地址与打印机通信
     *     bambubus_ams_address = BambuBus_address_AMS08;   // = 0x0700
     *     data.source_address  = BambuBus_address_AMS08;
     * 它 5 处长包发送全是常量 0x0700，一次都没有跟随。
     *
     * 所以这里一律用 COMM_485_DEV_TYPE（0x0700）回包。打印机的寻址地址
     * 只用来打日志/报警：它按 lite 地址问我们，说明它以为总线上是 lite，
     * 那要么打印机设置里 AMS 类型选错，要么打印机固件还不够新。
     */
    if ((mine == COMM_485_ADDR_AMS || mine == COMM_485_ADDR_LITE) &&
        mine != s_my_addr) {
        s_my_addr = mine;                  /* 仅作诊断记录，不参与回包地址 */
        if (mine == COMM_485_ADDR_LITE) {
            ams_log_warn("485：★ 打印机按 AMS Lite 地址(0x%04X)寻址，"
                         "但本机按标准 AMS(0x%04X)应答 —— 型号可能不符",
                         (unsigned)mine, (unsigned)COMM_485_DEV_TYPE);
        } else {
            ams_log("485：打印机按 AMS 地址(0x%04X)寻址 → 正常",
                    (unsigned)mine);
        }
    }

    if (plen < 0) {
        plen = 0;
    }

    switch (type) {
    case 0x21A:
        on_mc_online(peer, COMM_485_DEV_TYPE, pkg, pl, plen);
        break;
    case 0x211:
        on_filament_info(peer, COMM_485_DEV_TYPE, pkg, 0x211, pl, plen);
        break;
    case 0x0411:
        /* ★★ 2026-10-23：AMS 2 Pro / AMS HT 的耗材信息查询 ★★
         *
         * 现场 /bus485raw 里这台打印机**从来没发过 0x211**，发的是 0x0411：
         *     RX 19 3D 05 00 00 13 00 8E 00 07 00 03 11 04 00 00 00 00
         *     payload = 00 00 00 00   → 台 0 槽 0
         * 过一会儿变成 01 00 00 00    → 台 1 槽 0（payload[0] 就是台号）
         *
         * 这就是"槽位一直显示空、不能进退料"的根源：它在用新协议逐台问
         * 耗材信息，我们当成"未知长帧"扔了 → 它拿不到任何 tray 数据
         * → 4 个槽一直是"空" → 空槽当然不给进料/设色。
         *
         * 0x0411 = 0x211 | 0x0200，是同构的新版本号。先按同样的 131 字节
         * 结构回，type 原样回 0x0411。若结构真有差异，打印机最多丢弃这帧，
         * 不会比"完全不应答"更糟。 */
        on_filament_info(peer, COMM_485_DEV_TYPE, pkg, 0x0411, pl, plen);
        break;
    case 0x103:
    case 0x402:
        on_version(type, peer, COMM_485_DEV_TYPE, pkg, pl, plen);
        break;
    case 0x218: {  /* 写耗材信息（长包）→ 3 字节确认 */
        static const uint8_t ack[3] = { 0x00, 0x00, 0x00 };
        send_long(0x218, peer, COMM_485_DEV_TYPE, pkg, ack, (int)sizeof(ack));
        log_limited("写耗材信息(长)", pl, plen, &s_cnt_08);
        break;
    }
    case 0x0003:
    case 0x0004:
        handle_motion(f, len, false, peer, pkg);
        break;
    default:
        log_limited("未知长帧", f, len, &s_cnt_other);
        break;
    }
}

/* ==========================================================================
 * 接收状态机
 * ========================================================================== */

/** 按已有字节数推断整包长度；0 = 还判断不出来 */
static int frame_total(void)
{
    if (s_idx < 3) {
        return 0;
    }
    if (s_buf[1] & 0x80) {
        return s_buf[2];                        /* 短帧：LEN 在 index 2 */
    }
    if (s_idx < 6) {
        return 0;
    }
    return (int)(s_buf[4] | ((uint16_t)s_buf[5] << 8));   /* 长帧：pkg_len */
}

/** 校验一整帧 */
static bool frame_valid(const uint8_t *f, int len)
{
    if (len < 6) {
        return false;
    }
    if (f[1] & 0x80) {
        if (f[3] != crc8_calc(f, 3)) {
            return false;
        }
        uint16_t want = (uint16_t)(f[len - 2] | (f[len - 1] << 8));
        return want == crc16_calc(f, (size_t)(len - 2));
    }
    if (len < 15) {
        return false;
    }
    if (f[6] != crc8_calc(f, 6)) {
        return false;
    }
    uint16_t want = (uint16_t)(f[len - 2] | (f[len - 1] << 8));
    return want == crc16_calc(f, (size_t)(len - 2));
}

static void rx_byte(uint8_t b)
{
    if (s_idx == 0) {
        if (b == BUS_HEAD) {
            s_buf[s_idx++] = b;
        }
        return;
    }
    if (s_idx >= (int)sizeof(s_buf)) {
        s_idx = 0;
        return;
    }
    s_buf[s_idx++] = b;

    int total = frame_total();
    if (total == 0) {
        if (s_idx >= 6) {
            s_idx = 0;
        }
        return;
    }
    if (total < 6 || total > BUS_MAX_FRAME) {
        s_idx = 0;
        return;
    }
    if (s_idx < total) {
        return;
    }

    if (!frame_valid(s_buf, total)) {
        s_crc_errors++;
        /* 只丢一个字节再找下一个 0x3D：多字节噪声/半截帧时，
         * 一丢丢整包会连着丢好几帧 */
        memmove(s_buf, s_buf + 1, (size_t)(s_idx - 1));
        s_idx--;
        return;
    }

    s_rx_frames++;
    s_last_rx_us = esp_timer_get_time();
    bus_record(1, s_buf, total);       /* 留一份，网页 /bus485raw 可回看 */
    if (s_buf[1] & 0x80) {
        handle_short(s_buf, total);
    } else {
        handle_long(s_buf, total);
    }
    s_idx = 0;
}

static void bus_task(void *arg)
{
    (void)arg;
    uint8_t chunk[256];

    while (!s_stop) {
        /* ---- 兜底自愈：静默 60 秒 ----
         * 打印机重新上电会重新枚举总线。我们注册成功后是静默的（跟 BMCU
         * 一致），所以打印机重启后不会主动来问我们 —— 只能靠它发
         * 0x05/0x03/0xFF（重新编址）来把我们叫醒，那条在
         * on_online_detect() 里处理。
         *
         * 这里只留一个很长的兜底：60 秒完全没有收到任何帧，才认为对端
         * 重启过。原来是 10 秒 —— 打印机空闲时总线本来就长时间没流量，
         * 10 秒一清就会周期性重报注册包，造成 AMS 反复上下线。 */
        if (s_last_rx_us > 0) {
            int64_t idle_us = esp_timer_get_time() - s_last_rx_us;
            if (idle_us > 60000000) {
                if (!s_idle_rearmed) {
                    s_idle_rearmed = true;
                    s_reg_mask     = 0;
                    s_req06_unit   = 0xFF;
                    ams_log_warn("485：总线静默 >60 秒（打印机重启/断线）"
                                 "→ 清零注册状态，等它重新枚举");
                }
            } else {
                s_idle_rearmed = false;
            }
        }

        /* RX FIFO 快满说明接收任务被饿过（WiFi 抢 CPU 时最常见）→ 那一帧
         * 基本注定丢字节、CRC 错。记一次日志，别静默失败。 */
        size_t buffered = 0;
        if (uart_get_buffered_data_len(COMM_485_UART_NUM, &buffered) == ESP_OK &&
            buffered > (COMM_485_RX_BUF * 3 / 4)) {
            if (s_rx_buf_warns < 5) {
                s_rx_buf_warns++;
                ams_log_warn("485：接收缓冲堆积 %u 字节（>3/4）→ 有丢帧风险",
                             (unsigned)buffered);
            }
        }

        int n = uart_read_bytes(COMM_485_UART_NUM, chunk, sizeof(chunk),
                                pdMS_TO_TICKS(10));
        if (n <= 0) {
            continue;
        }
        for (int i = 0; i < n; i++) {
            rx_byte(chunk[i]);
        }
    }
    s_task = NULL;
    vTaskDelete(NULL);
}

/* ==========================================================================
 * 生命周期
 * ========================================================================== */
esp_err_t comm_485_init(void)
{
    if (s_ready) {
        return ESP_OK;
    }

    /* ---- DE：先压成低电平（= 收），绝不能上电就跑成发送态把总线拽住 ---- */
    gpio_config_t de = {
        .pin_bit_mask = (1ULL << BOARD_PIN_485_DE),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t err = gpio_config(&de);
    if (err != ESP_OK) {
        ams_log_err("485 DE 引脚(GPIO%d)配置失败: %s",
                    BOARD_PIN_485_DE, esp_err_to_name(err));
        return err;
    }
    gpio_set_level((gpio_num_t)BOARD_PIN_485_DE, 0);

    /* ---- UART：1.2288M 8E1 ---- */
    uart_config_t uc = {
        .baud_rate  = COMM_485_BAUD,
        .data_bits  = UART_DATA_8_BITS,
        .parity     = UART_PARITY_EVEN,     /* ★ BambuBus 是偶校验，不是无校验 */
        .stop_bits  = UART_STOP_BITS_1,
        .flow_ctrl  = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,        /* APB 80MHz 分频，误差可忽略 */
    };
    err = uart_param_config(COMM_485_UART_NUM, &uc);
    if (err != ESP_OK) {
        ams_log_err("485 UART 参数配置失败: %s", esp_err_to_name(err));
        return err;
    }
    err = uart_set_pin(COMM_485_UART_NUM, BOARD_PIN_485_TX, BOARD_PIN_485_RX,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (err != ESP_OK) {
        ams_log_err("485 UART 引脚配置失败(TX=GPIO%d RX=GPIO%d): %s",
                    BOARD_PIN_485_TX, BOARD_PIN_485_RX, esp_err_to_name(err));
        return err;
    }
    /* ★ RX 缓冲 4KB：1.2288Mbaud 下 1 字节 8.1µs，4KB 约 33ms 的流量。
     *   接收任务每 20ms 取一次，正常有余量；如果哪次 Flash 写把任务卡住超过
     *   33ms 就会静默丢字节 → 整帧 CRC 错。真遇到就把这里再放大。 */
    err = uart_driver_install(COMM_485_UART_NUM, COMM_485_RX_BUF, 0, 0, NULL, 0);
    if (err != ESP_OK) {
        ams_log_err("485 UART 驱动安装失败: %s", esp_err_to_name(err));
        return err;
    }

    s_idx = 0;
    s_stop = false;
    s_ready = true;
    s_reg_mask      = 0;
    s_req06_unit    = 0xFF;
    s_idle_rearmed  = false;
    s_rx_buf_warns  = 0;
    s_log_head      = 0;
    s_log_seq       = 0;

    fil_template_init();

    /* ★★ 2026-10-22 深夜：把 485 接收任务**钉到 core 1** 并提高优先级 ★★
     *
     * 1.2288Mbaud 下 1 字节只有 8.1µs，一帧 131 字节约 1.06ms —— 接收侧
     * 必须**连续不断地**把 FIFO 取空，中间卡住十几毫秒就会丢字节、整帧
     * CRC 报错，打印机那边就表现为"这台 AMS 偶尔不见了"。
     *
     * ESP32-S3 的 WiFi 任务默认跑在 core 0、优先级 23。原来 bus485 没绑核、
     * 优先级只有 7，WiFi 一发包就把接收任务压住 → 这就是"网页/485 同时开
     * （调试模式）时连接不稳定"的根因。现在：
     *   · xTaskCreatePinnedToCore(..., core 1)：离开 WiFi 所在的 core 0
     *   · 优先级 12：高于普通任务，低于 WiFi（WiFi 仍在 core 0，互不干扰）
     * 纯 485 模式（WiFi 关闭）下这条改动没有副作用。 */
    BaseType_t ok = xTaskCreatePinnedToCore(bus_task, "bus485", BUS_TASK_STACK,
                                            NULL, BUS_TASK_PRIO, &s_task, 1);
    if (ok != pdPASS) {
        ams_log_err("485 接收任务创建失败");
        s_task  = NULL;
        s_ready = false;
        uart_driver_delete(COMM_485_UART_NUM);
        return ESP_ERR_NO_MEM;
    }

    ams_log("RS-485 已启动：UART%d %d 8E1，TX=GPIO%d RX=GPIO%d DE=GPIO%d"
            "，地址 0x%04X，扮演 %d 台 AMS（共 %d 槽）",
            COMM_485_UART_NUM, COMM_485_BAUD,
            BOARD_PIN_485_TX, BOARD_PIN_485_RX, BOARD_PIN_485_DE,
            (unsigned)COMM_485_DEV_TYPE, AMS_UNITS, AMS_TOTAL_SLOT);
    ams_log("485 就绪：等打印机广播 0x05 → 回注册包 → 认到后会打"
            "「★ AMS #n 注册通过」");
    if (!config_get_485_actuate()) {
        ams_log_warn("485 暂不驱动电机：运动指令只回「空闲、未动作」"
                     "（网页「485 运动执行」可打开）");
    } else {
        ams_log_warn("485 会真驱动电机：打印机点进料/退料时机构会动作");
    }
    return ESP_OK;
}

void comm_485_deinit(void)
{
    if (!s_ready) {
        return;
    }
    s_ready = false;

    /* ★ 必须先让接收任务真的退出，再删 UART 驱动。
     *   顺序反了的话 bus_task 还在 uart_read_bytes 里，驱动被删掉之后
     *   它会一直返回错误、空转烧 CPU（优先级 7，比 idle 高，能把系统拖死）。
     *   任务的读超时是 20ms，所以最多等 100ms 就够。 */
    s_stop = true;
    for (int i = 0; i < 20 && s_task != NULL; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    s_task = NULL;

    uart_driver_delete(COMM_485_UART_NUM);
    gpio_set_level((gpio_num_t)BOARD_PIN_485_DE, 0);
    s_ready = false;
}

bool comm_485_ready(void)
{
    return s_ready;
}

bool comm_485_online_seen(void)
{
    return s_online_seen;
}

int comm_485_registered_count(void)
{
    int n = 0;
    for (int i = 0; i < AMS_UNITS; i++) {
        if (s_reg_mask & (1u << i)) {
            n++;
        }
    }
    return n;
}

uint32_t comm_485_rx_frames(void)
{
    return s_rx_frames;
}

bool comm_485_wait_printer(uint32_t timeout_ms)
{
    /* ★ 判据只看"有没有收到帧"，不要求注册成功：
     *   打印机一上电就每 ~200ms 广播一次 0x05，收到帧就说明总线通、
     *   打印机在。而注册成功还依赖打印机那边的 AMS 配置，拿它当判据
     *   会在"打印机不认我们"时误判成"没接打印机"，然后错误地开 WiFi。 */
    if (!s_ready) {
        return false;
    }
    if (s_rx_frames > 0) {
        return true;
    }

    int64_t t0 = esp_timer_get_time();
    uint32_t next_log = 30;                 /* 每 30 秒报一次进度 */

    while (s_rx_frames == 0) {
        uint32_t el = (uint32_t)((esp_timer_get_time() - t0) / 1000);
        if (el >= timeout_ms) {
            break;
        }
        if (el >= next_log) {
            ams_log("485 探测中… 已等 %u 秒 / 共 %u 秒，还没收到打印机任何帧",
                    (unsigned)el, (unsigned)(timeout_ms / 1000));
            next_log += 30;
        }
        vTaskDelay(pdMS_TO_TICKS(100));
    }

    return s_rx_frames > 0;
}

int comm_485_describe_json(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return 0;
    }
    int64_t now = esp_timer_get_time();
    long idle_ms = (s_last_rx_us > 0) ? (long)((now - s_last_rx_us) / 1000) : -1;
    return snprintf(buf, buflen,
                    "{\"ready\":%d,\"rx\":%lu,\"tx\":%lu,\"crc\":%lu,"
                    "\"cmd\":%lu,\"online\":%d,\"idle_ms\":%ld,\"dev\":%d,"
                    "\"addr\":%d,\"act\":%d,\"units\":%d,\"reg\":%d}",
                    s_ready ? 1 : 0,
                    (unsigned long)s_rx_frames, (unsigned long)s_tx_frames,
                    (unsigned long)s_crc_errors, (unsigned long)s_last_cmd,
                    s_online_seen ? 1 : 0, idle_ms,
                    COMM_485_DEV_TYPE, (int)s_my_addr,
                    config_get_485_actuate() ? 1 : 0, AMS_UNITS, (int)s_reg_mask);
}

#else  /* !BOARD_485_ENABLED —— 本板型没有 485 电路，给一组空实现 */

esp_err_t comm_485_init(void)
{
    ams_log_warn("本板型没有 RS-485 电路（IO 不够），通信方式固定为 MQTT");
    return ESP_ERR_NOT_SUPPORTED;
}

void comm_485_deinit(void) {}

bool comm_485_ready(void) { return false; }

bool comm_485_online_seen(void) { return false; }

int comm_485_registered_count(void) { return 0; }

uint32_t comm_485_rx_frames(void) { return 0; }

bool comm_485_wait_printer(uint32_t timeout_ms)
{
    (void)timeout_ms;
    return false;
}

int comm_485_describe_json(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return 0;
    }
    return snprintf(buf, buflen,
                    "{\"ready\":0,\"rx\":0,\"tx\":0,\"crc\":0,\"cmd\":0,"
                    "\"online\":0,\"idle_ms\":-1,\"dev\":0,\"act\":0,"
                    "\"units\":0,\"reg\":0}");
}

/* 本板型没有 485 → 原始报文永远是空的。网页 /bus485raw 照常能打开，
 * 只是提示一句，不至于 404。 */
int comm_485_dump(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return 0;
    }
    return snprintf(buf, buflen,
                    "# 本板型没有 RS-485 电路，没有总线报文可看\n");
}

#endif /* BOARD_485_ENABLED */
