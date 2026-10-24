/**
 * ams_controller.c —— AMS 业务状态机的实现
 * ============================================================================
 *
 * 这个文件把三样东西接在一起：
 *
 *     打印机（MQTT 上报 / 命令）
 *          ↕
 *     ams_controller  ←── 命令队列 ←── 网页 / 微动触发
 *          ↕
 *     电机 / 电磁离合 / 微动（GPIO）
 *
 * ----------------------------------------------------------------------------
 * 任务模型
 * ----------------------------------------------------------------------------
 * 只有一个 FreeRTOS 任务 `ams_task`：
 *
 *     while (1) {
 *         ① 离合体检（防两路同时吸合）
 *         ② 有新的打印机上报？→ 处理（可能触发换料）
 *         ③ 有命令？→ 取出来执行（**阻塞**执行，几秒也照跑）
 *         ④ 检查"开始送料"微动是否被触发
 *         ⑤ 更新状态灯
 *         ⑥ 睡 20ms
 *     }
 *
 * 第 ③ 步会阻塞这个任务好几秒（送料、等挤出机、蠕动……），这完全没有问题 ——
 * Web 服务在 httpd 的任务里，MQTT 在 esp-mqtt 的任务里，都不受影响。
 * 这一点是相对 MicroPython 版最大的结构性简化：Python 版为了不让事件循环
 * 被按住，把每个动作都拆成了 `await` 片段，代码读起来非常绕。
 *
 * ----------------------------------------------------------------------------
 * 安全约定（**改这个文件时请务必保持**）
 * ----------------------------------------------------------------------------
 *   1. 任何"吸合离合 + 转电机"的组合，都必须走下面的 drive_channel() /
 *      creep_feed()，它们内部保证：无论正常返回还是出错，最后一定是
 *      「电机停 + 全部离合断开」。
 *   2. 不允许在别处直接调 clutch_engage() —— 那会绕过 drive_channel 的收尾。
 *   3. 主循环每个周期都调 clutch_assert_single()，发现多路吸合立即全部断开。
 */

#include "ams_controller.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "bambu_mqtt.h"
#include "bambu_proto.h"
#include "clutch.h"
#include "config.h"
#include "filament_sensor.h"
#include "log_buffer.h"
#include "motor.h"

/* ==========================================================================
 * 常量
 * ========================================================================== */
#define AMS_CMD_QUEUE_LEN 6
#define AMS_TASK_STACK    8192
#define AMS_TASK_PRIO     5

/** 给打印机发完一条命令后，等它"有进展"的最长时间 */
#define AMS_PRINTER_WAIT_MS 8000

/** 打印机重复上报换料请求时，最多重试几次（超过就停下来报错，别无限折腾） */
#define AMS_EXCHANGE_MAX_RETRY 5

/** 快速退料指令发出去之后，只等这么点时间让命令落地；
 *  ★ 真正"打印机做完切刀 + 吐料"的等待在 do_exchange 里，靠 ams_status
 *    握手完成（见 wait_printer_need_withdraw_ms）。旧版就是拿这个 3000ms
 *    当"切刀已经做完了"，实测差了 40 秒 —— 那时打印机还在等喷嘴升温。 */
#define AMS_PRIME_WAIT_MS 300

/** ★ 等打印机报「退料完成，需要退线」（ams_status=260）的最长时间。
 *  实测要 40~56 秒（其中 42 秒是喷嘴从 140℃ 升到 245℃），
 *  喷嘴从室温冷启动时更久，所以给足 3 分钟。 */
#define AMS_UNLOAD_READY_TIMEOUT_MS 180000

/** 打印机的 ams_status 取值（Top-AMS bambu.hpp 的常量表，A1 实测一致） */
#define AMS_PSTAT_IDLE          0    /* 空闲 / 退料完成 */
#define AMS_PSTAT_UNLOADING     259  /* 退料（切刀 + 吐料）进行中 */
#define AMS_PSTAT_NEED_WITHDRAW 260  /* ★ 退料完成，需要 AMS 退线 */

/* ★ 退料的节奏参数已经全部搬到 config 里（网页「硬件调试」可改）：
 *     蠕动退料时长  config_get_retract_creep_ms()   → retract.creep_ms
 *     蠕动间隔      config_get_retract_gap_ms()     → retract.gap_ms
 *     蠕动轮数上限  config_get_retract_creep_max()  → retract.creep_max
 *     连续退料时长  config_get_retract_cont_ms()    → retract.cont_ms
 *   旧版这里写死 2000 / 1000 / 12，现场想调一次就得重烧固件，所以搬走了。 */

/** 进料前等热端升到目标温度的最长时间 */
#define AMS_HEAT_WAIT_MS 30000

/** 同一个"触发指纹"在这个时间窗内被当成残留上报（见 handle_report）。
 *  取 60 秒：远大于打印机状态上报的延迟（1~3 秒），又远小于切片两次
 *  换色之间的间隔 —— 所以既能挡住重复换料，又不会漏掉连续换色。 */
#define AMS_EXCHANGE_RESIDUAL_MS 60000

/* ==========================================================================
 * 内部状态
 * ========================================================================== */

static QueueHandle_t      s_cmd_queue;
static SemaphoreHandle_t  s_report_lock;
static SemaphoreHandle_t  s_state_lock;
static TaskHandle_t       s_task;

static ams_state_t s_state = AMS_STATE_IDLE;
static int         s_active_material = -1;
static ams_diag_t  s_diag;
static uint32_t    s_last_error_us = UINT32_MAX;

/* 最近一次打印机上报（网页用，也用于"等打印机有进展"） */
static bambu_report_t s_last_report;
static bool           s_has_report;
static volatile bool  s_report_pending;

/* 换料请求去重 */
static int  s_exchange_attempts;
static bool s_change_active;

/* ---- ★ 切刀指令已经发出去了（等 do_exchange 去接那个握手）----
 * handle_report 一拿到通道号就发 `M109 + M620 S255/T255/M621 S255`，
 * 但那时候打印机还要先升喷嘴温度、再切刀、再吐料，40 秒往上。所以：
 *   · s_prime_sent      = 指令真的发出去了（没发出去就别在 do_exchange 里等，
 *                         否则手动/网页触发的换料会白等 3 分钟超时）
 *   · s_prime_base_status = 发指令那一刻打印机的 ams_status，用来识别"陈旧 260" */
static volatile bool s_prime_sent;
static int           s_prime_base_status = -1;

/* ---- 换料"触发指纹"与已完成时刻 ----
 * 打印机会按秒重复上报同一个状态，换料完成后它还会继续报
 * change_needed=true 好几秒。只靠 s_change_active 去重是不够的：换料一
 * 成功就得把它清零（否则后续换料永远被拒），清零后下一帧立刻又排一次
 * 命令 —— 同一个颜色换两次就是这么来的。
 * 指纹 = 触发源（哪条通路 + 哪个通道），指纹没变且在 AMS_EXCHANGE_RESIDUAL_MS
 * 窗口内，就当它是上次那次触发的残留；指纹变了立刻放行。 */
static int      s_change_fp;
static int64_t  s_change_fp_us;
static int      s_pending_fp;     /* 已排队命令对应的指纹，成功时转正为 s_change_fp */

/* ---- 热床温度记忆（对应 Top-AMS 的 bed_target_temper_max）----
 * 切片用 M140 S{next_extruder + 1} 借"目标床温"传通道号，打印机上报的
 * bed_target 于是变成 1~16。真实床温必须在被改写之前记住，换完再还回去。
 * 取"见过的最高值"：不同层/不同材料的床温不一样（PLA 首层 60、后续 55），
 * 取最高能保证恢复后不会偏低。0 = 还没有可信记录。 */
static float    s_bed_target_max;

/* 辅助送料节流：所有"该做辅助送料"的阶段共用同一个"上次做的时刻"。
 *
 * ★ 2026-09-22 真机教训 —— 旧逻辑错在哪：
 *   以前 stg=8 / stg=19 各配一个"是否已做"的一次性标志，整个阶段只送一次。
 *   但**这台打印机的流量校准就是 stg=8**（从不报 19，见 bambu_proto.c 的阶段表），
 *   而它在真机上要跑 3 分 6 秒（板子日志 07:22.513 进校准挤出 → 10:28.446 卸载）。
 *   结果那 3 分钟里只有开头 1.5 秒电机在转，之后一直静止 ——
 *   用户现场看到的就是"流量校准没有辅助送料、电机没转"。
 *   现在改成：整个阶段**周期性地送**，不再是一次性。
 *
 * 顺带一个可用性设计：周期跟着"每次时长"走，所以把网页上的辅助送料时长
 * 调大，送料就变密；打印中若把时长调到 ≥5000ms，因为周期就是 5000ms，
 * 会变成连续辅助（这一条是刻意的，别当成 bug 修掉）。 */
static int64_t s_last_assist_time_us;   /* 上次辅助送料时刻；0 = 允许立刻做一次 */
#define ASSIST_REPEAT_US   (5 * 1000 * 1000)  /* 打印中（stg=0）：周期 5 秒 */
#define ASSIST_CAL_GAP_MS  500                /* 校准中（stg=8/19）：只停 0.5 秒 ——
                                               * 校准期间挤出头在持续吃料，
                                               * 间隔放长料就拱不上、挤出机只能硬拉。
                                               *
                                               * ★ 为什么是 0.5 秒而不是随便一个数：
                                               *   辅助送料只在**收到报文**那一拍才判一次，
                                               *   而真机实测报文是每 ~2.0 秒一拍
                                               *   （run5 校准段中位 2014ms、最大 4022ms）。
                                               *   周期设成「时长 1.5s + 0.5s = 2s」刚好
                                               *   咬住报文节奏 → 校准期间几乎每拍都送，
                                               *   现场就是持续在推料。设成大于 2.5 秒的话，
                                               *   会因为"这一拍还不够、下一拍在 2 秒后"而
                                               *   掉成每 4 秒一次。 */
/* 上次看到的 stg_cur，用于避免每次上报都打印日志 */
static int  s_last_seen_stg = -1;

/* 状态灯 */
static bool     s_led_on;
static uint32_t s_led_tick;

/* 微动触发去重：记录上一次看到的「开始送料微动」序号
 * ★ 2026-10-23：「自吸」功能已整套删除，原来这里还有一份
 *   s_last_autoload_seq[]，一并去掉了。 */
static uint32_t s_last_start_seq[BOARD_CHANNEL_COUNT];

/* ==========================================================================
 * ★★ 2026-10-24 用户拍板：485 进料「到位立即停」后的双微动决策层 ★★
 *
 *   打印机 send_out(0x03/0x00) → 我方进料 → 打印机 stop_on_use(0x07/0x00)
 *   到位 → ams_post_stop() 立即停电机 + 断离合。
 *
 *   但「到位」只说明料被 AMS 推到热端咬合，**未必咬稳**。用户要求：到位后
 *   靠该通道的两只微动（常开 4.7k 开始位 + 常闭 10k 停止位，串联走 ADC）
 *   决定「续送 / 断料」：
 *
 *       到位瞬间（stop_on_use）进入 s_485_decide_ch = ch 的窗口期
 *         ┌ 停止微动触发（STOP，料已咬进挤出机）→ 保持断料，窗口结束
 *         ├ 开始微动触发（START，料没咬稳 / 余料要补）→ 短促续送一段
 *         │   直到停止微动再触发，才收手断离合
 *         └ 窗口超时（默认 6s，料根本没动）→ 超时断离合，回到 idle
 *
 *   这层跑在 ams_task 主循环（不阻塞 485 状态机，不阻塞打印机节奏），
 *   续送段用 motor_set_dir_speed(FEED, 90) 短脉冲（每次 ~400ms）+ 立即
 *   复查停止微动，避免把料顶过头。
 * ========================================================================== */
#define AMS_485_DECIDE_TIMEOUT_MS  6000   /* 到位后决策窗口总长 */
#define AMS_485_DECIDE_CREEP_MS    400    /* 单次续送脉冲时长 */

static int      s_485_decide_ch    = -1;   /* 待决策通道；-1 = 无 */
static uint32_t s_485_decide_us    = 0;    /* 进入决策窗口的时刻 */
static uint32_t s_485_creek_us     = 0;    /* 上一次续送脉冲的起始时刻 */
static uint32_t s_485_creep_end_ms = 0;    /* 续送脉冲预计结束时刻；0 = 未在续送 */
static bool     s_485_decide_is_stop = false; /* true=到位后窗口(6s超时); false=随动窗口(不超时) */

/**
 * 由 485 状态机在 on_use / before_on_use / stop_on_use 时调用
 * （comm_485.c 的 mot_set 在转入这些状态时统一调用）。
 *
 * ★ 0.31.0：on_use/before_on_use 期间也开窗口（对照 Tianshu switchInterrupt），
 *   但不设 6 秒超时（随动期间可能很长）。只有 stop_on_use 的窗口才超时。
 */
void ams_485_enter_decide_window(int ch)
{
    s_485_decide_ch    = ch;
    s_485_decide_us    = (uint32_t)(esp_timer_get_time() / 1000);
    s_485_creek_us     = 0;
    s_485_creep_end_ms = 0;
    s_485_decide_is_stop = false;   /* 默认随动窗口，不超时 */
}

/**
 * ★★ 0.30.1：状态机离开 stop_on_use 时**必须**调用本函数取消续送窗口 ★★
 *
 *   现场：打印机在到位(stop_on_use)后**仅约 0.9 秒**就发 before_pull_back 退料，
 *   而本决策窗口长达 6 秒。窗口不取消的话，退料期间 poll_485_decide() 仍可能
 *   正转续送，与退料指令**方向相反**地互搏 —— 表现就是"到位后又反复进退料"。
 *
 *   comm_485.c 在 mot_set() 里、任何通道离开 MOT_STOP_ON_USE 时调用本函数。
 *   这里只清标志、不碰硬件：调用方紧接着一定会投递新的电机指令（mot_apply），
 *   由 ams_task 统一接管电机，避免跨任务操作电机。
 */
void ams_485_exit_decide_window(void)
{
    s_485_decide_ch    = -1;
    s_485_creep_end_ms = 0;
    s_485_decide_is_stop = false;
}

/** 由 comm_485.c 在进入 stop_on_use 时标记：到位后的窗口才设 6s 超时 */
void ams_485_mark_decide_stop(void)
{
    s_485_decide_is_stop = true;
    s_485_decide_us    = (uint32_t)(esp_timer_get_time() / 1000);
}

/** 485 决策层主循环（ams_task 每拍调用，全程非阻塞） */
static void poll_485_decide(void)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    /* 0. 续送脉冲到点收手 —— 放在窗口判断之前，保证残留脉冲一定被收掉。
     *    ★ 0.30.1 由阻塞 vTaskDelay(400ms) 改为"记结束时刻"的非阻塞写法：
     *      原来那 400ms 会把 ams_task 按住，队列里的 STOP/退料指令最多要
     *      等 400ms 才执行 —— 这正是"上报到位后没有及时停止电机"。 */
    if (s_485_creep_end_ms != 0 &&
        (int32_t)(now_ms - s_485_creep_end_ms) >= 0) {
        int c = s_485_decide_ch;
        s_485_creep_end_ms = 0;
        motor_stop();
        /* ★ 0.31.0：随动窗口续送后停止微动触发只停电机，不断离合、不关窗 */
        if (c >= 0 && sensor_triggered(c, SENSOR_STOP)) {
            if (s_485_decide_is_stop) {
                ams_log("485 决策：通道%d 续送后停止微动触发 → 收手断离合", c + 1);
                s_485_decide_ch = -1;
                clutch_release_all();
            } else {
                ams_log("485 决策：通道%d 续送后停止微动触发 → 停电机（随动中不关窗）", c + 1);
            }
        }
        return;
    }

    int ch = s_485_decide_ch;
    if (ch < 0) {
        return;
    }

    uint32_t since = now_ms - s_485_decide_us;

    /* 窗口超时 → 放弃决策，回到纯 idle
     * ★ 0.31.0：只有 stop_on_use 的到位窗口才检查 6s 超时。
     *   on_use / before_on_use 的随动窗口不超时（挤出机可能拉料很久），
     *   窗口的关闭由 mot_set 转态时调 exit_decide_window 完成。 */
    if (s_485_decide_is_stop && since >= AMS_485_DECIDE_TIMEOUT_MS) {
        ams_log("485 决策：通道%d 到位后 %lu ms 内微动无响应 → 断离合回空闲",
                ch + 1, (unsigned long)since);
        s_485_decide_ch = -1;
        /* 超时收手：若离合还吸着（续送中断开）则统一断 */
        motor_stop();
        clutch_release_all();
        return;
    }

    /* 1. 停止微动触发 → 料已咬稳，保持断料
     * ★ 0.31.0：随动窗口（on_use/before_on_use）期间不因停止微动而关窗，
     *   因为挤出机还在拉料、状态随时会变。只有到位窗口（stop_on_use）
     *   的停止微动触发才表示"料已到位咬稳"，可以关窗。
     *   但随动窗口里停止微动触发时仍然停电机（不续送），只是不关窗。 */
    if (sensor_triggered(ch, SENSOR_STOP)) {
        if (s_485_decide_is_stop) {
            ams_log("485 决策：通道%d 停止微动触发 → 料已咬稳，保持断料", ch + 1);
            s_485_decide_ch = -1;
        }
        /* 随动期间：停止微动触发 = 料在位，不续送，等下一拍 */
        return;
    }

    /* 2. 开始微动触发（缓冲器空，余料要补）→ 短促续送一段（非阻塞） */
    if (s_485_creep_end_ms == 0 &&
        sensor_triggered(ch, SENSOR_START) &&
        (since - s_485_creek_us >= 600)) {   /* 每 600ms 才允许一次，避免连续顶料 */
        ams_log("485 决策：通道%d 开始微动触发 → 续送 %d ms（PWM 90%%）",
                ch + 1, AMS_485_DECIDE_CREEP_MS);
        motor_set_dir_speed(MOTOR_DIR_FEED, MOTOR_SPEED_485_FEED);
        s_485_creek_us     = now_ms;
        s_485_creep_end_ms = now_ms + AMS_485_DECIDE_CREEP_MS;
        return;
    }

    /* 3. 两只微动都没触发（中间态）→ 保持当前：离合已吸、电机停，等下一拍 */
}


/* ==========================================================================
 * 一、状态读写
 * ========================================================================== */

static void set_state(ams_state_t st, int material)
{
    if (s_state_lock &&
        xSemaphoreTake(s_state_lock, pdMS_TO_TICKS(100)) == pdTRUE) {
        s_state = st;
        s_active_material = material;
        xSemaphoreGive(s_state_lock);
    } else {
        s_state = st;
        s_active_material = material;
    }
}

ams_state_t ams_get_state(void)
{
    return s_state;
}

int ams_active_material(void)
{
    return s_active_material;
}

bool ams_is_busy(void)
{
    return s_state != AMS_STATE_IDLE;
}

const char *ams_state_text(void)
{
    switch (s_state) {
    case AMS_STATE_IDLE:     return "空闲";
    case AMS_STATE_JOG:      return "手动点动";
    case AMS_STATE_RETRACT:  return "退料中";
    case AMS_STATE_LOAD:     return "送料中";
    case AMS_STATE_ASSIST:   return "辅助送料中";
    case AMS_STATE_CREEP:    return "蠕动送料中";
    case AMS_STATE_EXCHANGE: return "换料中";
    case AMS_STATE_ERROR:    return "出错";
    default:                 return "未知";
    }
}

int ams_current_printer_channel(void)
{
    return config_get_filament_current();
}

/**
 * 把 0xRRGGBB 说成人话（"橙色"）。
 *
 * ★ 现场要求（2026-09-24）原话："（报文未携带颜色信息，按通道号换料）实际是
 *   通道号就有颜色，把这个颜色在日志中显示出来。"
 *   —— 通道颜色在网页「通道设置」里早就配好了，日志却只报通道号，看日志的
 *   人还得回头翻网页才知道那是哪个料。带上颜色名，一眼就能对上料盘。
 *
 * 判据是 HSV 里最朴素的那套，不做色彩管理：
 *   · 三通道差 < 32            → 黑 / 灰 / 白（灰度系没有色相可谈）
 *   · 亮而偏暗的暖色（20~45°） → 棕
 *   · 其余按色相分档
 * 认不出来宁可按色相给个近似名，也**不返回空串** —— 日志里空着等于没写。
 */
static const char *color_name_of(uint32_t rgb)
{
    int r = (int)((rgb >> 16) & 0xFFu);
    int g = (int)((rgb >> 8) & 0xFFu);
    int b = (int)(rgb & 0xFFu);

    int mx = r, mn = r;
    if (g > mx) mx = g;
    if (b > mx) mx = b;
    if (g < mn) mn = g;
    if (b < mn) mn = b;
    int d = mx - mn;

    if (d < 32) {
        if (mx < 60)  return "黑";
        if (mx < 170) return "灰";
        return "白";
    }

    /* 色相 0~359 度，整数算 —— 为了一句日志不值得引入浮点 */
    int h;
    if (mx == r)           h = 60 * (g - b) / d;
    else if (mx == g)      h = 120 + 60 * (b - r) / d;
    else                   h = 240 + 60 * (r - g) / d;
    if (h < 0) h += 360;

    if (h < 15 || h >= 345) return "红";
    if (h < 45)  return (mx < 140) ? "棕" : "橙";
    if (h < 70)  return "黄";
    if (h < 165) return "绿";
    if (h < 200) return "青";
    if (h < 255) return "蓝";
    if (h < 290) return "紫";
    if (h < 330) return "品红";
    return "粉";
}

/**
 * 某个打印机通道的颜色，写成日志片段：`橙色 #FF8000`。
 *
 * ⚠️ 配置里 0 表示"没配过颜色"（见 config.h 的 color_list），所以纯黑
 *   （#000000）在这个语义下没法表达，会显示成"未配颜色" —— 这是配置层的
 *   既有约定，不是这里的判断错。
 */
static void color_text_of_channel(int printer_ch, char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return;
    }
    int mat = (printer_ch > 0) ? config_material_index_of(printer_ch) : -1;
    uint32_t c = (mat >= 0) ? (config_get_color(mat) & 0xFFFFFFu) : 0u;
    if (c == 0) {
        snprintf(buf, buflen, "未配颜色");
        return;
    }
    snprintf(buf, buflen, "%s #%.6lX", color_name_of(c), (unsigned long)c);
}

int ams_channel_text(int printer_channel, char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return 0;
    }
    if (printer_channel <= 0) {
        return snprintf(buf, buflen, "未知");
    }
    int mat = config_material_index_of(printer_channel);
    if (mat < 0) {
        return snprintf(buf, buflen, "通道 %d（未映射）", printer_channel);
    }
    /* ★ 带上颜色名（见 color_name_of 的说明）。这里**只放名字、不放 #RRGGBB**
     *   是因为调用方给的是 48 字节的 cur_txt / new_txt，中文一个字 3 字节，
     *   再加 7 个字符的十六进制会顶到边界；要十六进制的地方在
     *   color_text_of_channel()，那边有独立的缓冲。 */
    uint32_t c = config_get_color(mat) & 0xFFFFFFu;
    return snprintf(buf, buflen, "通道 %d（料盘位 %d · %s）",
                    printer_channel, mat + 1,
                    (c == 0) ? "未配颜色" : color_name_of(c));
}

static void record_error(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    char msg[80];
    vsnprintf(msg, sizeof(msg), fmt, ap);
    va_end(ap);

    strncpy(s_diag.last_error, msg, sizeof(s_diag.last_error) - 1);
    s_diag.last_error_ms = 0;
    s_last_error_us = esp_timer_get_time();
    ams_log_err("%s", msg);
}

/* ==========================================================================
 * 二、状态灯
 * ========================================================================== */

static void led_init(void)
{
    if (BOARD_PIN_LED < 0) {
        return;
    }
    gpio_config_t cfg = {
        /* ★ 原来是 (unsigned)(BOARD_PIN_LED < 64 ? BOARD_PIN_LED : 0)。
         *   ESP32-C3 上 BOARD_PIN_LED = -1（板载 WS2812 挂在 strapping 脚上，
         *   本工程不用它），而 -1 < 64 成立 → 取到 -1 → (unsigned)(-1) =
         *   0xFFFFFFFF → 左移 42 亿位越界。GCC 报 -Wshift-count-overflow，
         *   结果也不是 0，是未定义行为。
         *   虽然上面那句 `if (BOARD_PIN_LED < 0) return;` 已经提前兜住、
         *   这一行永远不会真正生效，但写法本身是错的 —— 谁把那个 return
         *   挪走就会踩上。所以显式判一遍非负。                            */
        .pin_bit_mask = (BOARD_PIN_LED >= 0 && BOARD_PIN_LED < 64)
                            ? (1ULL << BOARD_PIN_LED) : 0ULL,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level((gpio_num_t)BOARD_PIN_LED, !BOARD_LED_ACTIVE_LEVEL);
    s_led_on = false;
}

static void led_set(bool on)
{
    if (BOARD_PIN_LED < 0) {
        return;
    }
    gpio_set_level((gpio_num_t)BOARD_PIN_LED, on ? BOARD_LED_ACTIVE_LEVEL
                                                : !BOARD_LED_ACTIVE_LEVEL);
    s_led_on = on;
}

/**
 * 状态灯语义（和 Python 版一致）：
 *     空闲       慢闪（1Hz）—— 一切正常，等指令
 *     动作中     常亮       —— 正在动，别碰机构
 *     出错       快闪（5Hz）—— 需要人看一眼
 */
static void led_update(void)
{
    if (BOARD_PIN_LED < 0) {
        return;
    }
    s_led_tick++;

    uint32_t period;
    if (s_state == AMS_STATE_ERROR) {
        period = 10;      /* 10 × 20ms = 200ms 一个相位 → 5Hz */
    } else if (s_state != AMS_STATE_IDLE) {
        led_set(true);
        return;
    } else {
        period = 25;      /* 25 × 20ms = 500ms 一个相位 → 1Hz */
    }

    if (s_led_tick % period == 0) {
        led_set(!s_led_on);
    }
}

/* ==========================================================================
 * 三、总线守护：所有复合动作都从这里走
 * ========================================================================== */

/** 最近一次驱动动作的回执；任何一次 drive_channel()/drive_channel_speed()
 *  都会刷新它。诊断用，不参与任何控制逻辑。
 *  类型定义与 getter 在 ams_controller.h（网页 /status 也要读它）。 */
static drive_receipt_t s_last_drive;

/**
 * 这一路的「微动反馈」是否可用 —— 只看「缓冲送料」这一个开关。
 *
 * ★ 2026-10-23：「自吸」功能已整套删除，原来这里还有
 *   `|| config_autoload_enabled(material_index)` 一项，一并去掉了。
 *   现在只有「缓冲送料」开着的通道才走微动模式（分步推、停止微动触发即停），
 *   关着的走按时间 / 闭环的降级模式。
 */
static bool micro_feedback_available(int material_index)
{
    return config_sensor_enabled(material_index);
}

/* ==========================================================================
 * 三·五、缓冲器闭环（2026-10-14 现场需求）
 * ==========================================================================
 *
 * 现场做了个**缓冲器**（装在挤出机之前，两只微动）：
 *     · 开始送料微动触发 = 缓冲器被挤出机抽空了 → 该补料
 *     · 停止送料微动触发 = 缓冲器满了           → 停
 *
 * 本次改了三件事，都跟它有关：
 *
 * ① **微动触发不再无条件动作**（poll_trigger_switches）
 *      以前：任何状态下微动碰一下就跑一次送料。
 *      现在：必须**有打印任务**，而且只有**当前通道**的微动算数。
 *      理由（现场原话）："有打印任务时触发才送料，没有打印任务时不动作；
 *      打印时只有正确通道的触发才按逻辑动作，其他通道不动作"。
 *
 * ② **打印中 / 校准阶段：送与停由微动决定，不再按时间间隔脉冲**
 *      （handle_report 的辅助送料一节）→ 见 buffer_need_feed()。
 *
 * ③ **微动无反馈保护**（buffer_track_activity）
 *      连续 AMS_BUFFER_DEAD_MS（15 秒）两只微动都没有任何状态变化 →
 *      判定"微动失效 / 没接好"，**退回原来的按时间间隔辅助送料**，把这场
 *      打印跑完再说。
 *
 * ★ 三块状态为什么放在这里而不是 handle_report 里：
 *   保护计时必须**不依赖打印机报文**（报文断了就没人计时了），所以挂在
 *   主循环每拍调用的 buffer_track_activity() 上；而 ① ② 只是读它算出来的
 *   结论（s_buf_fallback / buffer_need_feed），没有第二份状态。
 */

/** 打印机状态快照：只取控制要用的几个字段。
 *  bambu_report_t 整份很大，主循环 20ms 拍一次，没必要整份拷到栈上。 */
typedef struct {
    bool has;
    int  gcode_state;      /* 0=空闲 1=准备 2=打印中 3=暂停 4=结束 5=失败 */
    int  stg_cur;
    bool is_printing;
    int  mc_percent;
    int  layer_num;
    bool change_needed;    /* 打印机来换料请求了 */
} print_snap_t;

static print_snap_t print_snapshot(void)
{
    print_snap_t s;
    memset(&s, 0, sizeof(s));
    if (s_report_lock && xSemaphoreTake(s_report_lock, pdMS_TO_TICKS(5)) == pdTRUE) {
        s.has           = s_has_report;
        s.gcode_state   = s_last_report.gcode_state;
        s.stg_cur       = s_last_report.stg_cur;
        s.is_printing   = s_last_report.is_printing;
        s.mc_percent    = s_last_report.mc_percent;
        s.layer_num     = s_last_report.layer_num;
        s.change_needed = s_last_report.change_needed;
        xSemaphoreGive(s_report_lock);
    }
    return s;
}

/**
 * 「stg_cur=0 是不是真的在打印」—— 判据必须三条同时成立。
 *
 * ★ 为什么不能只看 stg_cur（2026-09-23 run7 真机踩过，详见 handle_report 里
 *   那段长注释）：stg_cur=0 既是"打印中"，也是打印机空闲时挂着的值；而刚
 *   下发打印任务时 print_type 已经变了、stg_cur 还留着上一次的 0。
 *   所以还要 gcode_state==RUNNING（is_printing），并且任务**推进过**
 *   （mc_percent > 0 或 layer_num >= 1）。自检时这两条都是 0 → 不算在打印。
 *
 * 只有 handle_report 和缓冲器闭环两处用，抽成一个函数免得两边判据写歪。
 */
static bool really_printing_at(int stg_cur, bool is_printing,
                               int mc_percent, int layer_num)
{
    return (stg_cur == 0) && is_printing &&
           (mc_percent > 0 || layer_num >= 1);
}

/**
 * 现在有打印任务吗。
 *
 * ★ 「有任务」= gcode_state ∈ {准备, 打印中, 暂停}。
 *   **暂停也算**：暂停时用户就在机器边上（清堵、看料、扯料头），这时候
 *   缓冲器微动该能补料；把它排除掉反而会让"暂停时按了微动没反应"变成一个
 *   新的现场疑问。空闲 / 已结束 / 失败 = 没有任务 —— 那才是"不该动"的时候。
 */
bool ams_print_job_active(void)
{
    print_snap_t s = print_snapshot();
    return s.has && s.gcode_state >= 1 && s.gcode_state <= 3;
}

/**
 * 现在处在「打印机在吃料」的阶段吗：打印中 / 校准挤出 / 流量校准。
 *
 * ★ change_needed（打印机来换料请求）不算 —— 换料永远优先，
 *   它的送料是另一条路（do_exchange），不该被缓冲闭环抢总线。
 */
bool ams_feeding_stage(void)
{
    print_snap_t s = print_snapshot();
    if (!s.has || s.change_needed) {
        return false;
    }
    return (s.stg_cur == 8 || s.stg_cur == 19 ||
            really_printing_at(s.stg_cur, s.is_printing,
                               s.mc_percent, s.layer_num));
}

/** 当前记录在用的**料盘位下标**（0 起）；-1 = 不知道 */
static int current_material_index(void)
{
    int ch = config_get_filament_current();
    return (ch > 0) ? config_material_index_of(ch) : -1;
}

/* ---- 缓冲器闭环状态 ---- */

/** 微动最近一次"动过"的时刻（微秒）—— 无反馈保护的计时基准 */
static int64_t  s_buf_last_act_us[BOARD_CHANNEL_COUNT];
/** 上次看到的触发序号 [ch][0]=停止送料 [ch][1]=开始送料 */
static uint32_t s_buf_seq[BOARD_CHANNEL_COUNT][2];
static bool     s_buf_seeded;                            /* 序号基线建好没有 */
static bool     s_buf_fallback[BOARD_CHANNEL_COUNT];     /* 保护生效 → 退回定时送料 */
static bool     s_buf_need[BOARD_CHANNEL_COUNT];         /* 滞回：上一拍要不要送 */
static int64_t  s_buf_last_pulse_us[BOARD_CHANNEL_COUNT];/* 上次补料脉冲时刻 */
static bool     s_buf_job_active;                        /* 上一拍"有打印任务"吗 */
/** 上次重置滞回时的阶段号：阶段一变就按当前微动电平重新播种 */
static int      s_buf_last_stage = -1;

/**
 * 这一路的**硬件**够不够跑缓冲闭环：开了「缓冲送料」且两只微动都有引脚。
 *
 * ⚠️ 只有"停止"没有"开始"是不够的 —— 那样只知道该停、不知道该不该开，
 *    退化成原来的闭环送料就行，别硬凑。
 */
static bool buffer_hw_ok(int mat)
{
    return mat >= 0 && mat < BOARD_CHANNEL_COUNT &&
           config_sensor_enabled(mat) &&
           sensor_pin_present(mat, SENSOR_STOP) &&
           sensor_pin_present(mat, SENSOR_START);
}

/** 这一路的缓冲闭环**现在生效**吗（硬件够 + 没被保护降级） */
static bool buffer_closed_loop(int mat)
{
    return buffer_hw_ok(mat) && !s_buf_fallback[mat];
}

/**
 * 缓冲器要不要补料 —— 电平判据 + 滞回。
 *
 *   停止送料微动触发 → 满了 → **不送**（最高优先，防顶料）
 *   开始送料微动触发 → 空了 → 送
 *   两只都没触发（缓冲器在中间）→ **保持上一拍的决定**，免得在中间来回抖
 *
 * ★ 为什么中间态要滞回而不是"默认不送"：
 *   不送的话，缓冲器一进中间态就停，等它被抽空到开始微动触发才又送 ——
 *   完全可行但补料会一顿一顿的；保持决定等于"送就送到满、停就停到空"，
 *   这才是缓冲器该有的工作方式。
 */
static bool buffer_need_feed(int mat)
{
    if (sensor_triggered(mat, SENSOR_STOP)) {
        s_buf_need[mat] = false;
    } else if (sensor_triggered(mat, SENSOR_START)) {
        s_buf_need[mat] = true;
    }
    return s_buf_need[mat];
}

/**
 * 刷新"微动最近活动时刻"，并做**无反馈保护**。主循环每拍调用。
 *
 * ★ 为什么不放在 handle_report 里：那边只有**收到打印机报文**才跑。
 *   打印机不推报文时（比如刚连上、或 MQTT 静默），保护计时就冻住了 ——
 *   而"微动坏了"恰恰可能同时表现为"什么都没动"。挂在主循环上才稳。
 */
static void buffer_track_activity(void)
{
    int64_t now = esp_timer_get_time();

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        uint32_t st = sensor_trigger_seq(i, SENSOR_STOP);
        uint32_t sp = sensor_trigger_seq(i, SENSOR_START);

        if (!s_buf_seeded) {
            s_buf_seq[i][0]      = st;
            s_buf_seq[i][1]      = sp;
            s_buf_last_act_us[i] = now;
            continue;
        }
        if (st != s_buf_seq[i][0] || sp != s_buf_seq[i][1]) {
            s_buf_seq[i][0]      = st;
            s_buf_seq[i][1]      = sp;
            s_buf_last_act_us[i] = now;
            if (s_buf_fallback[i]) {
                s_buf_fallback[i] = false;
                ams_log("料盘位%d 微动恢复动作 → 回到缓冲器闭环送料", i + 1);
            }
        }
    }
    s_buf_seeded = true;

    /* 打印任务结束 → 清掉全部回退标记。
     * 不能因为上一场打印里的一次误判就把这一路**永久**降级成定时送料。 */
    bool job = ams_print_job_active();
    if (s_buf_job_active && !job) {
        for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
            s_buf_fallback[i]      = false;
            s_buf_last_pulse_us[i] = 0;   /* 下一场打印的第一拍就能立刻补料 */
        }
    }
    s_buf_job_active = job;

    /* 阶段一变 → 按当前微动电平重新播种滞回，免得拿上一个阶段的旧决定。 */
    {
        print_snap_t s = print_snapshot();
        if (s.stg_cur != s_buf_last_stage) {
            s_buf_last_stage = s.stg_cur;
            for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
                if (sensor_triggered(i, SENSOR_STOP)) {
                    s_buf_need[i] = false;
                } else if (sensor_triggered(i, SENSOR_START)) {
                    s_buf_need[i] = true;
                } else {
                    s_buf_need[i] = false;   /* 中间态：先当"不送"，等它抽空 */
                }
            }
        }
    }

    /* ---- 无反馈保护 ---- */
    int mat = current_material_index();
    if (mat < 0 || !buffer_hw_ok(mat) || s_buf_fallback[mat]) {
        return;
    }
    if (!config_get_assist_enabled()) {
        /* 辅助送料总开关关着 → 本来就不送料，谈不上"微动没反馈" */
        return;
    }
    if (!ams_feeding_stage()) {
        /* 不在吃料的阶段（换料 / 空闲 / 自检）→ 把计时往前推，
         * 免得阶段刚开始就来吃一个上一次的旧时间戳。 */
        s_buf_last_act_us[mat] = now;
        return;
    }
    if ((now - s_buf_last_act_us[mat]) < (int64_t)AMS_BUFFER_DEAD_MS * 1000) {
        return;
    }

    s_buf_fallback[mat]  = true;
    s_buf_last_act_us[mat] = now;
    ams_log_warn("料盘位%d：两只微动已连续 %u 秒没有任何动作 → 判定微动"
                 "无反馈，本阶段退回按时间间隔的辅助送料（保护）。"
                 "请检查这一路的微动接线 / 安装位置",
                 mat + 1, (unsigned)(AMS_BUFFER_DEAD_MS / 1000));
}

int ams_buffer_mode(int channel)
{
    if (!buffer_hw_ok(channel)) {
        return AMS_BUF_OFF;
    }
    return s_buf_fallback[channel] ? AMS_BUF_FALLBACK : AMS_BUF_ACTIVE;
}

/**
 * 吸合通道 → 按方向转 ms 毫秒（可选：中途查停止微动）→ 停 → 断开全部离合。
 *
 * @param material_index  料盘位下标（0 起）
 * @param direction       1 进料 / -1 退料
 * @param max_ms          最长转多久
 * @param stop_on_sensor  true 时每 AMS_FILAMENT_STEP_MS 查一次「停止送料微动」，
 *                        触发就提前结束（有微动的通道才有意义）
 * @param out_triggered   非 NULL 时，返回是否是被微动提前结束的
 * @return true 动作正常完成（不代表"送到了"，只代表没出错）
 */
static bool drive_channel_speed(int material_index, int direction,
                                uint8_t speed_pct, uint32_t max_ms,
                                bool stop_on_sensor, bool *out_triggered);

static bool drive_channel(int material_index, int direction, uint32_t max_ms,
                          bool stop_on_sensor, bool *out_triggered)
{
    return drive_channel_speed(material_index, direction, 100, max_ms,
                               stop_on_sensor, out_triggered);
}

/**
 * 同 drive_channel，但指定电机速度（PWM 占空比百分比）。
 * 用于辅助送料等需要非全速跑的场景。
 */
static bool drive_channel_speed(int material_index, int direction,
                                uint8_t speed_pct, uint32_t max_ms,
                                bool stop_on_sensor, bool *out_triggered)
{
    /* ★ 先把回执清成"什么都没发生"：任何一条提前 return 都会把它留在这个
     *   状态 —— 调用方（尤其是辅助送料）据此就能知道"日志打了，但动作
     *   其实没发生"。这是本轮修"日志在打、电机不动"的核心手段。 */
    s_last_drive.clutch_ok = false;
    s_last_drive.skipped   = false;
    s_last_drive.dir       = (int8_t)((direction > 0) ? 1 : -1);
    s_last_drive.speed_pct = speed_pct;
    s_last_drive.duty_raw  = (uint16_t)motor_pct_to_duty(speed_pct);
    s_last_drive.ran_ms    = 0;
    s_last_drive.duty_in1  = 0;
    s_last_drive.duty_in2  = 0;
    s_last_drive.level_in1 = -1;
    s_last_drive.level_in2 = -1;

    if (out_triggered) {
        *out_triggered = false;
    }
    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        record_error("送料失败：料盘位 %d 不存在", material_index + 1);
        return false;
    }

    motor_readback_t rb = {0, 0, -1, -1};   /* 硬件回读，见函数尾部的赋值 */

    esp_err_t err = clutch_engage(material_index + 1);
    if (err != ESP_OK) {
        record_error("料盘位%d 离合吸合失败（%s）—— 本次**没有**驱动电机",
                     material_index + 1, esp_err_to_name(err));
        clutch_release_all();
        return false;                       /* 回执里 clutch_ok 保持 false */
    }
    clutch_set_busy(true);
    s_last_drive.clutch_ok = true;          /* 离合真的吸合了 */

    bool triggered = false;
    /* 微动模式：停止送料微动可用 = 这一路「缓冲送料」开着 + 真的接了脚。
     * ★ 2026-10-23：原来这里还接受「自吸」开关，自吸已删除。 */
    bool use_sensor = stop_on_sensor && micro_feedback_available(material_index) &&
                      sensor_pin_present(material_index, SENSOR_STOP);

    if (use_sensor) {
        uint32_t elapsed = 0;
        while (elapsed < max_ms) {
            uint32_t run = AMS_FILAMENT_STEP_MS;
            if (elapsed + run > max_ms) {
                run = max_ms - elapsed;
            }
            motor_run((motor_dir_t)direction, run);
            elapsed += run;

            if (sensor_triggered(material_index, SENSOR_STOP)) {
                triggered = true;
                break;
            }
        }
        /* 微动分支走的是 motor_run()（全速），回执按全速记 */
        s_last_drive.speed_pct = 100;
        s_last_drive.duty_raw  = (uint16_t)motor_pct_to_duty(100);
        s_last_drive.ran_ms    = elapsed;
        motor_readback(&rb);              /* ★ 停之前回读硬件，见 drive_receipt_t */
        motor_stop();
    } else {
        /* ★ 关键：direction 是 int（±1），必须 cast 成 motor_dir_t。
         * 不 cast 的话 C 编译器可能把 -1 当 0（STOP）处理，
         * 退料方向就会丢，电机不转（之前 bug 的根因） */
        motor_dir_t dir = (motor_dir_t)direction;

        /* ★ 幂等短路探测：motor_set_dir_speed() 遇到"同方向 + 同速度 +
         *   已在转"会直接 return（**完全不碰硬件**）。这是"发了指令但车
         *   没动"的另一条可能路径（见 drive_receipt_t）。先探再调，
         *   把它记进回执，现场才能分清是没驱动还是驱动了带不动。 */
        if (motor_get_dir() == dir &&
            motor_get_speed_pct() == (int)speed_pct &&
            motor_elapsed_ms() != 0) {
            s_last_drive.skipped = true;
        }

        motor_run_speed(dir, max_ms, speed_pct);
        s_last_drive.ran_ms = max_ms;
        /* ★ 回读必须在这里 —— motor_stop() 一执行两路就都归 0 了，
         *   之后再读只会得到"什么都没写"，毫无意义。 */
        motor_readback(&rb);
        motor_stop();
    }

    /* ★ 把回读结果记进回执（不在临界区里做，避免白占锁） */
    s_last_drive.duty_in1  = rb.duty_in1;
    s_last_drive.duty_in2  = rb.duty_in2;
    s_last_drive.level_in1 = (int8_t)rb.level_in1;
    s_last_drive.level_in2 = (int8_t)rb.level_in2;

    if (out_triggered) {
        *out_triggered = triggered;
    }

    /* ★ 收尾：无论如何都要回到安全状态。
     *   这里不用 try/finally（C 没法那么写），但保证只有这一条返回路径 ——
     *   上面所有分支最后都落到这里。 */
    clutch_release_all_settled();
    return true;
}

const drive_receipt_t *ams_last_drive_receipt(void)
{
    return &s_last_drive;
}

/**
 * 蠕动间隔：慢速短脉冲之间留 150ms，让挤出机齿轮转到位再拱下一下。
 * 没有间隔就变成连续慢速送料，齿轮还没转到位料又顶上来，容易打折。
 */
#define CREEP_GAP_MS 150

/**
 * ★ 换料/485 进料到位后蠕动收尾的单次时长 = 1 秒（现场要求"5 次 × 1 秒"，
 *   2026-09-23），次数 = 5。485 模式进料到位（on_use）后也复用这套参数，
 *   把料头拱稳，防止"进料成功又立马退料"。
 */
#define CREEP_EXCHANGE_MS    1000
#define CREEP_EXCHANGE_TIMES 5

/* ==========================================================================
 * 四、基本动作：送料 / 退料
 * ========================================================================== */

/** 是否有任何一路装了微动（=「缓冲送料」任一位开着）
 *  ★ 2026-10-23：原来还并上「自吸」掩码，自吸已删除。 */
static bool has_any_sensor(void)
{
    return sensor_enabled_mask() != 0;
}

/** 闭环送料每段的时长 —— 现场要求："信号没变化 → 2 秒 → 继续送入" */
#define AMS_LOAD_SEGMENT_MS 2000
/**
 * 闭环送料的**总时长上限**。
 *
 * ★ 必须有的刹车：2026-09-23 现场事故 —— 送料因为等不到到位信号一直推，
 *   把外挂料盘上的整卷料喂空，最后只能手动在打印机上点「耗材已加载」继续。
 *   20 秒 = 10 段，按本机料路长度足够到位；到点还没信号就停机告警，
 *   交给 handle_report 的"载入打印材料"逻辑下一拍再补推。
 */
#define AMS_LOAD_TOTAL_MS   20000

/**
 * 闭环送料：一段一段往前推，每段结束查一次「耗材已到挤出机」。
 *
 * ★ 现场要求的确切语义（2026-09-23，用户原话）：
 *     "拉出或送入 → 等待信号 → 信号没变化 → 2 秒 → 继续拉出或送入
 *      → 直到接收到需要的信号；送料过程中收到挤出机耗材到位信号后
 *      立即停止送料，开始蠕动送料 5 次，每次 1 秒。"
 *
 * ★ 为什么不能"盲推 8000ms 再干等 15000ms"（旧做法，真机实测的毛病）：
 *   旧版一次把 8000ms 推满，期间完全不看打印机状态 —— 料早到位了它还在推
 *   （现场表现为空推、把料拱弯），料没到位它推完才进入 15 秒干等。
 *   实测到的边界：打印机报「耗材已到挤出机」是 56:25.101，而我们的等待在
 *   56:25.067 超时 —— 差 **34 毫秒**，就差了那么一条日志。
 *   闭环之后这个边界不存在了：信号一到立刻停。
 *
 * 两个"必须"：
 *   ① 到位标志只在**进入本函数时**清一次，中途绝不再清 —— 到位事件就是在
 *      这两秒一段里来的，清一下就把刚拿到的信号丢了（这正是旧版
 *      "日志说料到位了、我们还在傻等 15 秒"的根因）。
 *   ② 一定要有总时长上限，见 AMS_LOAD_TOTAL_MS 的说明。
 *
 * @param out_inplace  非 NULL 时返回"结束时挤出机是否已到位"
 * @return true = 硬件层没出错（到位与否看 out_inplace）
 */
/* ★ 前置声明，别删。
 *   poll_printer_inplace() 的**定义**在下面（闭环送料那一节的末尾，约 620 行），
 *   但 load_closed_loop() 里要先用它。C 的规则是"调用点之前必须能看到声明或
 *   定义"，看不到就是隐式声明 —— 而 ESP-IDF 默认带 `-Werror=all`，
 *   隐式声明直接是**编译错误**。
 *
 *   2026-09-23 就是栽在这里：本机没有 gcc / ESP-IDF，全部本地自检
 *   （check_c_static / check_printf_args / check_config_layout）都是绿的，
 *   一推上去 CI 两个目标全红。为此补了 tools/check_c_call_order.py 专门盯这类
 *   问题，它会明确报出这一行。 */
static bool poll_printer_inplace(void);

static bool load_closed_loop(int material_index, uint32_t total_ms,
                             bool *out_inplace)
{
    if (out_inplace) {
        *out_inplace = false;
    }
    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        record_error("送料失败：料盘位 %d 不存在", material_index + 1);
        return false;
    }

    esp_err_t err = clutch_engage(material_index + 1);
    if (err != ESP_OK) {
        record_error("送料：料盘位%d 离合吸合失败（%s）",
                     material_index + 1, esp_err_to_name(err));
        clutch_release_all();
        return false;
    }
    clutch_set_busy(true);

    extruder_inplace_clear();      /* 见上面 ① */

    bool     inplace = false;
    uint32_t elapsed = 0;
    uint32_t seg     = 0;
    while (elapsed < total_ms) {
        uint32_t run = AMS_LOAD_SEGMENT_MS;
        if (elapsed + run > total_ms) {
            run = total_ms - elapsed;
        }
        seg++;
        /* 全速推 —— 慢速占空比在这个机构上带不动电机（见 do_retract 的说明）。
         * motor_run() 会阻塞 run 毫秒，所以"段"就是一次推进。 */
        motor_run((motor_dir_t)1, run);
        elapsed += run;

        if (extruder_inplace_triggered()) {
            inplace = true;
            break;
        }
        /* ★ 再看一眼打印机的最新报文 —— 阻塞期间 handle_report 是跑不了的，
         *   不偷看就只能等推满 20 秒（run7 实测就是这样白推了 20 秒）。 */
        if (poll_printer_inplace()) {
            inplace = true;
            break;
        }
        /* 前两段报个进度，之后交给 handle_report 的节流日志，别刷屏 */
        if (seg <= 2) {
            ams_log("  送料（闭环）：第 %u 段推完 %ums，还没等到到位信号…",
                    (unsigned)seg, (unsigned)run);
        }
    }
    motor_stop();
    clutch_release_all_settled();

    if (out_inplace) {
        *out_inplace = inplace;
    }
    return true;
}

/**
 * 在**阻塞式推进**的过程中偷看一眼打印机的最新报文。
 *
 * ★★ 为什么必须这么做（2026-09-23 run7 真机定位到的根因）★★
 *
 *   架构上有一条铁律：`handle_report()` 是**跑在 ams_task 里**的
 *   （见 on_mqtt_report 的注释和 ams_task 的主循环）。而"闭环送料/连续退料"
 *   也是在 ams_task 里跑的、每段要 motor_run() 阻塞最长 2 秒。
 *   于是矛盾出现了：**我们一边推料，一边收不到打印机的任何消息** ——
 *   MQTT 任务只把最新一帧塞进 s_last_report 信箱，真正的解析要等
 *   ams_task 空出来。等我们推完 20 秒 + 蠕动 5 秒回到主循环，那些
 *   "耗材已到挤出机"才一起涌进来。
 *
 *   run7 的日志把这条缝隙量了出来（4 次换色，次次如此）：
 *     打印机报 hw_switch_state=1   10:53:43   ← 料其实 8 秒就到了
 *     板子还在推，一直推到          10:53:55
 *     板子报"送到"、发 resume       10:53:52~55
 *     板子才打出"打印机上报耗材已到挤出机"
 *   也就是说：**每次换色白推约 20 秒**（4 次就是 80 秒），
 *   而且是在料早就到位的情况下继续顶着料推 —— 对料和齿轮都不好。
 *
 *   修法：不去动 handle_report 的架构（那会牵动整个状态机），
 *   只加一个**只读**的偷看 —— 直接从信箱里读最新一帧的到位提示。
 *   拿到就把"到位"记进传感器层（和正常路径完全一致），后续逻辑不用改。
 *
 * @return true = 打印机最新一帧说"耗材已到挤出机"
 */
static bool poll_printer_inplace(void)
{
    bool hit = false;
    if (s_report_lock &&
        xSemaphoreTake(s_report_lock, pdMS_TO_TICKS(50)) == pdTRUE) {
        hit = s_has_report && (s_last_report.extruder_inplace_hint == 1);
        xSemaphoreGive(s_report_lock);
    }
    if (hit && !extruder_inplace_triggered()) {
        extruder_inplace_notify_from_mqtt();
        ams_log("打印机已报「耗材到挤出机」（推进途中读到，立刻收手）");
    }
    return hit;
}

/**
 * 送料：把指定料盘位的料往挤出机方向推。
 *
 * 有「停止送料微动」且**允许用它停**：分步推进，每步之后查微动，触发即停。
 * 否则：闭环送料 —— 2 秒一段，边推边看打印机的到位信号。
 *
 * @param stop_on_buf_switch
 *        「停止送料微动触发就停」这一条**要不要执行**。
 *
 *        ★ 换料时必须传 false（2026-10-14 现场要求，原话）：
 *          "在换料时收到打印机耗材到位时才停止，按原来的逻辑，对微动到位
 *           停止的动作不执行停止，避免换料时耗材到挤出机阻力过大，缓冲器
 *           阻力小，导致耗材不能送到挤出机导致报故障。"
 *
 *          道理是这样的：换料的**目标**是把料送到挤出机。料从缓冲器里过，
 *          缓冲器阻力小、挤出机阻力大，所以料会先在缓冲器里堆起来把缓冲
 *          微动顶住 —— 这时如果按微动停，料**根本没到挤出机**，打印机随后
 *          就报"耗材未到位"。所以换料过程只认打印机的到位信号（闭环），
 *          不看缓冲器微动。
 *
 *          手动「送料」按钮保持 true（那是现场调试缓冲区用的动作）。
 */
static bool do_load(int material_index, bool stop_on_buf_switch)
{
    set_state(AMS_STATE_LOAD, material_index);

    /* ★ 到位标志在这里统一清一次。
     *   以前是"哪里等、哪里清"，结果送料阶段收到的到位事件会被后面那次
     *   clear() 抹掉 —— 真机表现为"料明明到位了，我们还在等 15 秒然后报
     *   '料可能没咬住'"，紧接着打印机又报"耗材已到挤出机"，前后差 34ms。 */
    extruder_inplace_clear();

    if (stop_on_buf_switch &&
        micro_feedback_available(material_index) &&
        sensor_pin_present(material_index, SENSOR_STOP)) {
        /* ---- 路径 A：有微动 → 分步推，微动触发即停 ---- */
        ams_log("  进料：开始送料通道 %d（按微动反馈）", material_index + 1);
        uint32_t max_ms = (uint32_t)AMS_LOAD_RETRY_TIMES * AMS_FILAMENT_STEP_MS;
        if (max_ms > AMS_NO_LIMIT_LOAD_MS) {
            max_ms = AMS_NO_LIMIT_LOAD_MS;
        }
        bool triggered = false;
        if (!drive_channel(material_index, 1, max_ms, true, &triggered)) {
            return false;
        }
        s_diag.load_ok++;
        if (triggered) {
            ams_log("停止送料微动触发，送料结束");
        } else {
            ams_log_warn("  送料：推满 %ums 停止送料微动也没触发，"
                         "可能是料没到位、或者料盘位 %d 的微动没接好",
                         (unsigned)max_ms, material_index + 1);
        }
    } else {
        /* ---- 路径 B：无微动（C3 默认）→ 闭环送料 ----
         *
         * ★ 这条路**不再提微动**。本机压根没装停止送料微动，旧版每次
         *   都打"送料 8000ms 结束，但停止送料微动没触发" —— 现场反馈
         *   这条警告是纯噪音（2026-09-23）。现在的判据是打印机的到位信号。 */
        if (!stop_on_buf_switch && micro_feedback_available(material_index)) {
            ams_log("  进料：本路有微动，但本次只认「打印机报到位」—— "
                    "缓冲器阻力比挤出机小，按微动停料就到不了挤出机");
        }
        ams_log("  进料：开始送料通道 %d（闭环推进，最多 %ums）",
                material_index + 1, (unsigned)AMS_LOAD_TOTAL_MS);

        bool inplace = false;
        if (!load_closed_loop(material_index, AMS_LOAD_TOTAL_MS, &inplace)) {
            return false;
        }
        s_diag.load_ok++;
        if (inplace) {
            ams_log("  送料：打印机已报「耗材已到挤出机」，立刻停送料");
        } else {
            ams_log_warn("  送料：推满 %ums 也没等到「耗材已到挤出机」，"
                         "先停机（避免把料盘喂空）—— 下一拍「载入打印材料」"
                         "阶段还会再补推一次",
                         (unsigned)AMS_LOAD_TOTAL_MS);
        }
    }

    return true;
}

/** 连续退料时每次"跑一小段"的时长：分段跑，每段之间查一次信号，避免拉过头 */
#define AMS_RETRACT_POLL_MS 250
/**
 * 收到"挤出机已空"之后再全速多拉这一小段，把料彻底退出挤出机齿轮。
 *
 * ★ 2026-09-23 按现场要求从 500ms 抬到 2000ms；
 *   2026-10-23 现场明确改成 1000ms：挤出机报「没料」后再全速多拉 1 秒，
 *   把料彻底退出挤出机齿轮，然后停。1 秒已经足够让料脱离齿轮，
 *   2 秒会在外部料盘上多绕一圈、甚至把料头拉到脱离料盘导轮。
 */
#define AMS_RETRACT_TAIL_MS 1000

/**
 * 打印机有没有明确说"挤出机里现在没料"。
 *
 * 只认 hw_switch_state == 0（解析成 extruder_inplace_hint）。**不能拿
 * extruder_inplace_seq() 当"料走了"的判据** —— 那个序号只在"变成有料"时
 * 推进（extruder_inplace_notify_from_mqtt() 只在 hint==1 时被调用），
 * 用它判空会误判。
 */
static bool printer_says_extruder_empty(void)
{
    bool empty = false;
    if (xSemaphoreTake(s_report_lock, 0) == pdTRUE) {
        empty = (s_last_report.extruder_inplace_hint == 0);
        xSemaphoreGive(s_report_lock);
    }
    return empty;
}

/** 蠕动退料一次（短脉冲反向推一小段，需要吸合离合） */
static void creep_retract_once(int material_index, uint16_t pulse_ms,
                                uint8_t speed_pct)
{
    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        return;
    }
    esp_err_t err = clutch_engage(material_index + 1);
    if (err != ESP_OK) {
        /* ★ 这里**必须**报出来。吸合失败时旧版是静默返回，现象就是
         *   "日志说在退料、电机一声不响"，现场根本猜不到是离合没吸上。 */
        ams_log_err("  退料（蠕动）：料盘位%d 离合吸合失败（%s），本次蠕动跳过",
                    material_index + 1, esp_err_to_name(err));
        clutch_release_all();
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(CLUTCH_ENGAGE_MS));
    motor_run_speed((motor_dir_t)-1, pulse_ms, speed_pct);
    /* 跑完立刻停：不停的话下一轮之间那 gap_ms 电机还在空转 */
    motor_stop();
    clutch_release(material_index + 1);
    vTaskDelay(pdMS_TO_TICKS(CLUTCH_RELEASE_MS));
}

/**
 * 连续退料（全速反向拉），边拉边看打印机有没有报「挤出机已空」。
 *
 * ★ 为什么现在它是**第一个**动作，而不是以前的收尾动作：
 *   打印机报 `ams_status=260` 的含义是"我这边切完刀、吐完料、挤出机已经
 *   跑回冲刷区了，该你把料线收回去"。真机实测（2026-09-22 run5）它在这
 *   之后约 2 秒就会弹"请拉出耗材"—— 留给我们的窗口很短；而 45% 占空比的
 *   慢速蠕动在这个机构上**根本转不动**（12 轮 × 2000ms 合计 24 秒一根料
 *   都没拉动，电机连声都没有），只有全速连续拉才拉得动。
 *   所以顺序倒过来：先全速拉，拉到它报无料就停。
 *
 * @param max_ms   最长拉多久（到点就停，避免空转把料拉断/把料盘拽乱）
 * @param tail_ms  收到"无料"之后再全速多拉的这一小段
 * @param out_hw_failed  非 NULL 时返回"硬件层就失败了"（离合没吸上）。
 *                       这种情况下料**没被退出来**，调用方必须中止换料，
 *                       不能带着旧料去进新料。
 * @return true = 收到了「挤出机已空」信号
 */
static bool retract_continuous_until_empty(int material_index,
                                           uint32_t max_ms, uint32_t tail_ms,
                                           bool *out_hw_failed)
{
    if (out_hw_failed) {
        *out_hw_failed = false;
    }
    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        return false;
    }
    esp_err_t err = clutch_engage(material_index + 1);
    if (err != ESP_OK) {
        record_error("退料：料盘位%d 离合吸合失败（%s）",
                     material_index + 1, esp_err_to_name(err));
        clutch_release_all();
        if (out_hw_failed) {
            *out_hw_failed = true;
        }
        return false;
    }
    clutch_set_busy(true);

    bool got = false;
    uint32_t elapsed = 0;
    while (elapsed < max_ms) {
        uint32_t slice = AMS_RETRACT_POLL_MS;
        if (elapsed + slice > max_ms) {
            slice = max_ms - elapsed;
        }
        /* 全速（100%）—— 慢速在这个机构上带不动，见上面的说明 */
        motor_run((motor_dir_t)-1, slice);
        elapsed += slice;
        if (printer_says_extruder_empty()) {
            got = true;
            break;
        }
    }

    if (got && tail_ms) {
        motor_run((motor_dir_t)-1, tail_ms);
    }
    motor_stop();
    clutch_release_all_settled();
    return got;
}

/**
 * 退料：把料从挤出机/缓冲区收回到料盘。
 *
 * ★ 当前策略（2026-09-22 run5 真机重定，顺序是"连续优先"）：
 *   1. **全速连续退料**，最长 retract.cont_ms；期间每 250ms 查一次打印机
 *      上报的 hw_switch_state，一报"挤出机已空"（== 0）就停，再多拉
 *      AMS_RETRACT_TAIL_MS 把料彻底退出齿轮；
 *   2. 拉满时间还没等到信号 → 蠕动补拉 retract.creep_max 轮
 *      （每轮 retract.creep_ms，轮间 retract.gap_ms 等信号）；轮数配 0
 *      就是"只做连续退料"；
 *   3. 全程没等到信号也不阻断 —— 告警后照常进新料（宁可糙一点，也不能
 *      把打印机卡在暂停态）。
 *
 * 为什么不是"先蠕动"：45% 占空比在这个机构上带不动电机，实机 24 秒蠕动
 * 一根料都没拉动；真正把料拉出来的是全速连续退料。详见
 * retract_continuous_until_empty() 的说明。
 *
 * 有微动时仍走原有的"分步推、微动触发即停"路径。
 */
static bool do_retract(int material_index)
{
    set_state(AMS_STATE_RETRACT, material_index);

    /* 参数全走配置：网页「硬件调试」改完立刻生效，不用重烧固件 */
    const uint8_t  creep_pct = config_get()->creep_speed_pct;
    const uint16_t cont_ms   = config_get_retract_cont_ms();
    const uint16_t creep_ms  = config_get_retract_creep_ms();
    const uint16_t gap_ms    = config_get_retract_gap_ms();
    const uint8_t  creep_max = config_get_retract_creep_max();

    ams_log("  退料：开始退料料盘位 %d（先连续 %ums，拉不出来再蠕动 %u 轮 × %ums @%u%%）",
            material_index + 1, (unsigned)cont_ms,
            (unsigned)creep_max, (unsigned)creep_ms, (unsigned)creep_pct);

    /* ---- 路径 A：有微动 → 分步推，微动触发即停 ---- */
    if (micro_feedback_available(material_index) &&
        sensor_pin_present(material_index, SENSOR_STOP)) {
        uint32_t max_ms = (uint32_t)AMS_RETRACT_STEPS * AMS_FILAMENT_STEP_MS;
        bool triggered = false;
        bool ok = drive_channel(material_index, -1, max_ms, true, &triggered);
        if (!ok) {
            return false;
        }
        s_diag.retract_ok++;
        ams_log("退料结束（%s）", triggered ? "微动触发" : "到达时长上限");
        return true;
    }

    /* ---- 路径 B：无微动（C3 默认）→ 全速连续拉 + 蠕动补拉 ----
     *
     * ★ 2026-09-22 真机重定顺序（run5 证据）：
     *     · 打印机的"退料"是两个半场 —— 它那半：升喷嘴温度 → 切刀 → 移回
     *       冲刷区 → 把热端里那截吐掉；我们那半：把料从挤出机收回到料盘。
     *       分界线就是 ams_status=260，而调用方（do_exchange）**已经在外面
     *       等到它才进来**，所以这里每一步都发生在"料已切断、挤出机已回
     *       冲刷区"之后。
     *     · 旧版是"先蠕动 12 轮（39 秒）再连续退料"，同一次换色里那 12 轮
     *       一根料都没拉动 —— 45% 占空比带不动这个电机；结果打印机在 260
     *       之后约 2 秒就弹"请拉出耗材"，我们却在傻等。真正把料拉出来的是
     *       最后那 5 秒全速连续退料（打印机 hw_switch_state 同一秒 1 → 0）。
     *     · 所以顺序倒过来：**先全速连续拉**，拉到打印机报「挤出机已空」
     *       立即停；拉不出来才退回到慢速蠕动补拉（轮数可配，0 = 不蠕动）。
     */
    bool empty = false;
    if (config_get()->extruder_src == EXTRUDER_SRC_MQTT) {
        ams_log("  退料（连续）：全速拉料，最长 %ums；打印机一报"
                "「挤出机已空」就停，再补拉 %ums",
                (unsigned)cont_ms, (unsigned)AMS_RETRACT_TAIL_MS);

        bool hw_failed = false;
        empty = retract_continuous_until_empty(material_index, cont_ms,
                                               AMS_RETRACT_TAIL_MS,
                                               &hw_failed);
        if (hw_failed) {
            ams_log_err("  退料（连续）：离合/总线失败，料没退出来，换料中止");
            return false;
        }
        if (empty) {
            ams_log("  退料（连续）：打印机已报「挤出机已空」，拉料结束");
        } else {
            ams_log_warn("  退料（连续）：拉满 %ums 仍没等到「挤出机已空」信号"
                         "（料可能已退到缓冲，或打印机的 hw_switch_state 没跟上）",
                         (unsigned)cont_ms);
        }

        if (!empty && creep_max > 0) {
            ams_log("  退料（蠕动）：每次退 %ums + 等信号 %ums @%u%%，最多 %u 轮",
                    (unsigned)creep_ms, (unsigned)gap_ms,
                    (unsigned)creep_pct, (unsigned)creep_max);
            for (uint32_t round = 1; round <= creep_max; round++) {
                ams_log("  退料（蠕动）：第 %u/%u 次，退 %ums…",
                        (unsigned)round, (unsigned)creep_max,
                        (unsigned)creep_ms);
                creep_retract_once(material_index, creep_ms, creep_pct);

                /* 退完这一下，等挤出机信号最多 gap_ms */
                int64_t deadline = esp_timer_get_time() +
                                   (int64_t)gap_ms * 1000;
                while (esp_timer_get_time() < deadline) {
                    if (printer_says_extruder_empty()) {
                        empty = true;
                        break;
                    }
                    vTaskDelay(pdMS_TO_TICKS(50));
                }
                if (empty) {
                    ams_log("  退料（蠕动）：收到「挤出机已空」信号，停止蠕动");
                    break;
                }
            }
            if (!empty) {
                ams_log_warn("  退料（蠕动）：%u 轮都没等到「挤出机已空」信号，"
                             "按超时兜底继续进料",
                             (unsigned)creep_max);
            }
        } else if (!empty) {
            ams_log_warn("  退料：蠕动轮数配成 0（不蠕动），本次只做了连续退料");
        }
    } else {
        /* GPIO 模式（S3 默认）：那根挤出机到位线上没有"空了"的边沿可用，
         * 只能按时间推进 —— 等一段再全速拉满。
         * ⚠️ 这里**必须**保留一次真的退料动作：GPIO 分支里只等不拉的话，
         *    S3 上"退料"就整个消失了（料还在挤出机里就去进新料 → 顶死）。 */
        uint32_t wait_ms = config_get_retract_wait_ms();
        ams_log("  退料（等无料）：GPIO 模式，等待 %u ms", (unsigned)wait_ms);
        vTaskDelay(pdMS_TO_TICKS(wait_ms));
        ams_log("  退料（连续）：GPIO 模式，连续退料 %ums", (unsigned)cont_ms);
        if (!drive_channel(material_index, -1, cont_ms, false, NULL)) {
            ams_log_err("  退料（连续）：退料失败");
            return false;
        }
    }

    s_diag.retract_ok++;
    ams_log("退料完成（料已从挤出机收回料盘）");
    return true;
}

/**
 * 485 运动指令直接驱动电机（**非阻塞**）。
 *
 * ★ 2026-10-23 现场定位的根因：485（BMCU）通信下，进料/退料是打印机通过
 *   Cxx（cmd=0x03）运动帧下发的 ——
 *     收到 st=0x03 ch=0（进料）→ 正转；
 *     收到 st=0x03 ch=FF（退料）→ 反转；
 *     收到 st=0x07（到位）→ 停。
 *   这跟 MQTT 换料的 do_load/do_retract（20 秒闭环 + hw_switch_state）是
 *   两套互斥的机制，不能复用 —— do_load 会推满 20 秒、do_retract 会等
 *   MQTT 的"挤出机已空"信号，在 485 模式里这些信号根本不来，料就被顶过头。
 *
 * 本函数用 motor_set_dir_speed()（非阻塞，立即启动、不停），配合
 * clutch_engage() 把离合咬住，让电机**持续转**，直到打印机发「到位/停止」
 * 指令 → ams_post_stop() 收掉。这样电机完全跟随打印机的 485 节奏。
 *
 * ★★ 2026-10-24 用户拍板：进料占空比 100 → 90 ★★
 *   现场反馈"PWM 值太低还是蠕动送料" —— 满速 255 占空比下 DRV8833 的
 *   电流采样 + 死区时序会让电机在低负载段出现微停/蠕动，90% 占空比
 *   （duty=229）落在电机额定工作区间，送料顺滑。退料仍用满速。
 *
 * @param material_index 料盘位下标（0 起）
 * @param direction      1 = 进料（正转）；-1 = 退料（反转）
 */
static void do_485_drive(int material_index, int direction)
{
    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        record_error("485 驱动：料盘位 %d 不存在", material_index + 1);
        return;
    }

    esp_err_t err = clutch_engage(material_index + 1);
    if (err != ESP_OK) {
        record_error("485 驱动：料盘位%d 离合吸合失败（%s）",
                     material_index + 1, esp_err_to_name(err));
        clutch_release_all();
        return;
    }
    clutch_set_busy(true);

    /* ★★ 2026-10-24：进料 90%（消除蠕动），退料满速 ★★ */
    motor_dir_t dir = (direction < 0) ? MOTOR_DIR_RETRACT : MOTOR_DIR_FEED;
    int speed_pct = (direction < 0) ? MOTOR_SPEED_FULL : MOTOR_SPEED_485_FEED;
    motor_set_dir_speed(dir, speed_pct);

    set_state(AMS_STATE_LOAD, material_index);
    ams_log("485 驱动：料盘位%d %s（PWM %d%%，非阻塞，等待打印机的到位/停止指令）",
            material_index + 1, direction < 0 ? "退料" : "进料", speed_pct);
}


/* ==========================================================================
 * 五、探测当前料盘
 * ========================================================================== */

/**
 * 探测当前正在使用的是哪一路。
 *
 * 原理（和上游一致）：当前有料的通道反向转动时，料线会绷紧并把送料臂推到
 * 限位，微动触发；空料通道反转时轮子空转，微动不动。
 *
 * ⚠️ 本版本用「停止送料微动」当反馈，属于**近似**：如果机械结构上反转
 *    根本就压不到那只微动，探测就会全部走完一遍返回"未知"。
 *    那种情况下程序会退回使用 NVS 里记录的 filament_current ——
 *    也就是"未安装微动"时的行为，不会出错，只是不能自动识别。
 *
 * 探测到之后会反向转同样的时长把料退回去，避免长期绷紧。
 */
static int probe_current_filament(int fallback)
{
    if (!has_any_sensor()) {
        ams_log("未启用微动，跳过料盘探测，沿用记录值: %d", fallback);
        return fallback;
    }

    /* 探测顺序：先试上次用的通道，再试其余通道 */
    int order[BOARD_CHANNEL_COUNT];
    int n = 0;
    int fallback_mat = config_material_index_of(fallback);
    if (fallback_mat >= 0) {
        order[n++] = fallback_mat;
    }
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (i != fallback_mat) {
            order[n++] = i;
        }
    }

    const uint32_t step_ms = 500;
    const uint32_t max_steps = AMS_NO_LIMIT_PROBE_MS / step_ms;

    for (int idx = 0; idx < n; idx++) {
        int mat = order[idx];
        if (!micro_feedback_available(mat)) {
            continue;
        }

        ams_log("探测料盘位%d 是否有料…", mat + 1);
        set_state(AMS_STATE_RETRACT, mat);

        if (clutch_engage(mat + 1) != ESP_OK) {
            clutch_release_all();
            continue;
        }
        clutch_set_busy(true);

        bool found = false;
        uint32_t steps = 0;
        for (steps = 0; steps < max_steps; steps++) {
            motor_run(MOTOR_DIR_RETRACT, step_ms);
            if (sensor_triggered(mat, SENSOR_STOP)) {
                found = true;
                break;
            }
        }
        motor_stop();

        if (found) {
            /* 反向回位：把刚才拉紧的那一段料推回去 */
            motor_run(MOTOR_DIR_FEED, (steps + 1) * step_ms);
            motor_stop();
            clutch_release_all_settled();
            int ch = config_printer_channel_of(mat);
            ams_log("探测到当前料盘: %d（料盘位%d）", ch, mat + 1);
            return ch;
        }
        clutch_release_all_settled();
    }

    ams_log("探测未命中，沿用记录值: %d", fallback);
    return fallback;
}

/* ==========================================================================
 * 六、换料主流程
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * ★ 为什么换料过程中要"使唤"打印机，而不是全靠我们自己的电机
 * --------------------------------------------------------------------------
 * 热端里那一段料是**熔融**的。如果只靠远端电机硬拽，会出现：打滑、拉断、
 * 拉出来的丝粗细不匀、甚至把料头拽变形导致后面进不去。
 *
 * 打印机的工具头齿轮离热端最近、扭矩也足，让它自己先把料吐到缓冲里，
 * 我们再把料收回盘 —— 两段式接力，这是 Top-AMS 实测的稳定做法。
 *
 * ⚠️ 调温必须用 M109 / M190，**不能**用 M104 / M140：
 *    A1 系列 1.04 固件下，经 MQTT 发 M104/M140 是不生效的（打印机把它
 *    当"设了就设了"但不真的执行），只有 M109/M190 会真的等温。
 *    这条是 Top-AMS 在 bambu.hpp 里专门写了注释的坑。
 * -------------------------------------------------------------------------- */

/**
 * 换料前的"快速退料"：请求打印机把热端里的料吐出来。
 *
 * @param mat_cur 当前料盘位下标，用来查它的换料温度；-1 表示未知（用默认值）
 *
 * 时序照 Top-AMS：
 *      M109 S<temp>      热端升到该料盘位的换料温度（太凉退不出来）
 *      M620 S255         进入切割流程
 *      T255              切刀动作
 *      M621 S255         退出切割流程
 *
 * ★ 调用时机：在 handle_report 一拿到通道号就发，**不要**留到 do_exchange。
 *   （旧注释把原因写成"必须在打印机还 RUNNING 时才生效" —— 2026-09-22 真机
 *   实测**否掉了这条**：那时打印机已经是 PAUSE，切刀照样执行、`M190 S76`
 *   也在 2 秒内把 bed_target_temper 从 3 改成 76。真正不能省的只有结尾的
 *   `\n`，那个已由 bambu_cmd_gcode() 统一兜住。）
 *
 *   留在这里发仍然是对的，但理由是**时序**而不是"状态门禁"：切片把 M400 U1
 *   写在 M140 后面十几行，要趁这段窗口把"还床温 + 切刀"交代给打印机；
 *   等到 do_exchange 再发，我们自己的电机已经准备收线了，打印机再切刀
 *   会跟收线抢料。
 *
 * ★ 这条发完**只等 AMS_PRIME_WAIT_MS（300ms）让命令落地**，不做长等待。
 *   "打印机什么时候真的切完、吐完"由 do_exchange 里的
 *   wait_printer_need_withdraw_ms() 靠 ams_status 握手来判 ——
 *   旧版拿 3000ms 当"切刀已经做完"，实测差了 40 秒以上（那 40 秒打印机
 *   在等喷嘴从 140℃ 升到 245℃），结果我们的电机在料还没切断时就去拽。
 *
 * MQTT 没连上时不报错、只警告：退料仍然可以只靠本机电机完成（只是
 * 更容易打滑），不该因为发不出这条指令就把整次换料判失败。
 */
static void exchange_prime_extrude(int mat_cur)
{
    int temp = config_get_temper(mat_cur);
    char g[96];
    int n = snprintf(g, sizeof(g),
                     "M109 S%d\n"
                     "M620 S255\n"
                     "T255\n"
                     "M621 S255\n",
                     temp);
    if (n <= 0 || n >= (int)sizeof(g)) {
        return;
    }

    /* 记下发指令那一刻的 ams_status —— 等握手时用它区分"刚变过来的 260"
     * 和"上一轮残留的 260"。 */
    {
        int base = -1;
        if (xSemaphoreTake(s_report_lock, 0) == pdTRUE) {
            base = s_last_report.ams_status;
            xSemaphoreGive(s_report_lock);
        }
        s_prime_base_status = base;
    }

    if (bambu_mqtt_send_gcode(g) < 0) {
        s_prime_sent = false;
        ams_log_warn("  退料：快速退料指令没发出去（MQTT 未连接？），"
                     "只能靠自己硬退，注意别拉断");
        return;
    }
    s_prime_sent = true;
    ams_log("  退料：已请求打印机快速退料（切刀 + 吐料），热端 %d℃ —— "
            "等它报「退料完成需要退线」才轮到我们动手", temp);
    vTaskDelay(pdMS_TO_TICKS(AMS_PRIME_WAIT_MS));
}

/**
 * ★ 等打印机把「切刀 + 退到冲刷区 + 把热端里的料吐出去」做完。
 *
 * 判定依据就是打印机自己报的 `print.ams_status`（见 bambu_proto.h）：
 *      259 → 退料进行中（喷嘴到温、开始切刀）
 *      260 → ★ 退料完成，需要 AMS 退线  ← 等到它，我们才开始退料蠕动
 *
 * 2026-09-22 真机实测的变化史（指令 10:00:01 发出）：
 *      10:00:43 → 259   喷嘴升到 245℃
 *      10:00:56 → 260   切完 + 吐完
 * 也就是说**指令到 260 之间隔了 55 秒**。旧版固件盲等 3000ms 就去拽料，
 * 那时料还是完整的一根、还被挤出机齿轮咬着 —— 电机在打滑，
 * `hw_switch_state` 一整天没变成 0，打印机最后卡在 stg_cur=24（载入打印
 * 材料）上报错。这一次把顺序摆正。
 *
 * @return true = 等到了 260；false = 超时 / 打印机已经不在暂停态
 */
static bool wait_printer_need_withdraw_ms(uint32_t timeout_ms)
{
    if (!s_prime_sent) {
        /* 没发过切刀指令（网页/手动触发的换料），别白等 */
        ams_log("  未发过切刀指令，跳过打印机退料握手，直接退料");
        return false;
    }
    s_prime_sent = false;   /* 消费掉，避免下一次误等 */

    int  base      = s_prime_base_status;
    bool left_260  = (base != AMS_PSTAT_NEED_WITHDRAW);
    bool saw_259   = false;
    int  last_seen = INT32_MIN;

    ams_log("  等打印机完成切刀 + 吐料（ams_status 要从 %d 走到 %d）…",
            base, AMS_PSTAT_NEED_WITHDRAW);

    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        int cur = -1;
        bool paused = true;
        if (xSemaphoreTake(s_report_lock, 0) == pdTRUE) {
            cur    = s_last_report.ams_status;
            paused = s_last_report.is_paused;
            xSemaphoreGive(s_report_lock);
        }

        if (cur != last_seen && cur >= 0) {
            last_seen = cur;
            if (cur == AMS_PSTAT_UNLOADING) {
                saw_259 = true;
                ams_log("  打印机：退料/切刀进行中（ams_status=%d）", cur);
            } else if (cur == AMS_PSTAT_NEED_WITHDRAW) {
                /* 到 260 了 —— 但要排除"上一轮留下的陈旧 260"：
                 * 必须先看见它离开过 260（或起始就不是 260）。 */
                if (left_260) {
                    ams_log("  打印机：退料完成，需要退线（ams_status=%d）"
                            "→ 现在开始退料", cur);
                    return true;
                }
            } else {
                /* 又回到 0 或别的值 —— 说明刚离开过 260 */
                left_260 = true;
            }
        }

        if (!paused) {
            ams_log_warn("  等切刀握手时打印机已不在暂停态（可能被取消），中止等待");
            return false;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }

    ams_log_warn("  等 %us 没等到打印机的退料完成信号（ams_status 最后=%d，"
                 "见过 259=%s）—— 按超时继续退料，料可能还没被切断",
                 (unsigned)(timeout_ms / 1000), last_seen, saw_259 ? "是" : "否");
    return false;
}

/**
 * 换料进料前：把热端升到新料盘位的温度，并等到温。
 *
 * 温度没到就送料，料头会在冷热端里顶住、打弯，进不去 —— 表现为"送料
 * 送不动"。Top-AMS 也是发完 M109 然后自旋等到温（它等的是
 * nozzle_target_temper >= temp - 5，这里照做）。
 *
 * 等不到就继续走：卡在这里会让打印机一直停在暂停态，比"温度差几度"
 * 严重得多。差几度顶多多退一次料。
 */
static void exchange_heat_nozzle(int mat_new)
{
    int temp = config_get_temper(mat_new);
    char g[32];
    if (snprintf(g, sizeof(g), "M109 S%d", temp) <= 0) {
        return;
    }
    if (bambu_mqtt_send_gcode(g) < 0) {
        ams_log_warn("  进料：升温指令没发出去（MQTT 未连接？），按原温度继续");
        return;
    }
    ams_log("  进料：已请求热端升到 %d℃，等到温…", temp);

    int64_t deadline = esp_timer_get_time() + (int64_t)AMS_HEAT_WAIT_MS * 1000;
    while (esp_timer_get_time() < deadline) {
        float t = -1.0f;
        if (xSemaphoreTake(s_report_lock, 0) == pdTRUE) {
            t = s_last_report.nozzle_target;
            xSemaphoreGive(s_report_lock);
        }
        if (t >= (float)temp - 5.0f) {
            ams_log("  进料：热端已到温（设定 %.0f℃）", (double)t);
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(200));
    }
    ams_log_warn("  进料：等热端到温超时（%ums），继续送料（差几度顶多多退一次）",
                 (unsigned)AMS_HEAT_WAIT_MS);
}

/**
 * 等打印机"有进展"：stg_cur 或 gcode_state 变了就算。
 *
 * ★ 为什么不直接等某条特定应答：esp-mqtt 是异步回调模型，没有"等一条指定
 *   消息"的原生接口；而且拓竹不同固件版本的应答格式并不一致。
 *   Python 版的做法是 wait_msg_timeout(8000) —— "8 秒内收到任何消息就继续"，
 *   本质上也只是一个超时等待。这里做得稍好一点：等到打印机的阶段真的变了
 *   就立刻继续，最多等 AMS_PRINTER_WAIT_MS。
 */
static void __attribute__((unused)) wait_printer_progress(int prev_stg, int prev_gcode_state)
{
    int64_t deadline = esp_timer_get_time() +
                       (int64_t)AMS_PRINTER_WAIT_MS * 1000;
    while (esp_timer_get_time() < deadline) {
        if (s_has_report &&
            (s_last_report.stg_cur != prev_stg ||
             s_last_report.gcode_state != prev_gcode_state)) {
            return;
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    ams_log_warn("等打印机回应超时（%ums），继续下一步",
                 (unsigned)AMS_PRINTER_WAIT_MS);
}

/**
 * 换料主流程里的蠕动收尾：
 *   ① 等挤出机到位信号（硬件 GPIO 或 MQTT 事件）
 *   ② 到位后等 CREEP_DELAY_MS（1 秒，让打印机把料头咬稳、齿轮复位）
 *   ③ 蠕动 CREEP_EXCHANGE_TIMES（5）次
 *
 * 返回 true = 到位且蠕动完成；false = 等不到位（料没咬住，打印机会报缺料）
 */
#define CREEP_DELAY_MS       1000

static bool feed_until_extruder_ms(int mat_new, uint32_t wait_ms)
{
    /* ① 等挤出机到位
     *
     * ★ 先看"是不是已经到位了"。送料那段（do_load）已经清过标志，并且
     *   在 2 秒一段地盯着到位信号 —— 事件很可能在那边就来了。这里再
     *   clear() 一次等于把刚拿到的信号扔掉，正是旧版把"差 34 毫秒"变成
     *   "白等 15 秒、然后报料没咬住"的根因（真机日志：等超时在 56:25.067，
     *   打印机报到位在 56:25.101）。 */
    if (!extruder_inplace_triggered()) {
        /* ★ 先偷看一眼最新报文再决定要不要等 —— 送料那段跑完时，打印机可能
         *   早就报过到位了，只是那句话还堵在信箱里没被 handle_report 处理
         *   （见 poll_printer_inplace 的说明）。不先看这一眼的后果就是
         *   run7 里那 5 秒蠕动等待 + "料可能没咬住"的假警告。 */
        poll_printer_inplace();
    }
    if (!extruder_inplace_triggered()) {
        ams_log("  进料（蠕动）：等待挤出机到位信号（最多 %ums）…",
                (unsigned)wait_ms);
        if (!extruder_inplace_wait(wait_ms)) {
            ams_log_warn("  进料（蠕动）：等 %ums 没等到到位，料可能没咬住",
                         (unsigned)wait_ms);
            return false;
        }
    }
    ams_log("  进料（蠕动）：挤出机到位");

    /* ② 到位后等 1 秒，让打印机齿轮把料头咬稳 */
    ams_log("  进料（蠕动）：间隔 %ums 后开始蠕动收尾（%u 次 × %ums）",
            (unsigned)CREEP_DELAY_MS, (unsigned)CREEP_EXCHANGE_TIMES,
            (unsigned)CREEP_EXCHANGE_MS);
    vTaskDelay(pdMS_TO_TICKS(CREEP_DELAY_MS));

    /* ③ 蠕动 5 次 × 1 秒 */
    set_state(AMS_STATE_CREEP, mat_new);
    esp_err_t err = clutch_engage(mat_new + 1);
    if (err != ESP_OK) {
        record_error("换料蠕动：料盘位%d 离合吸合失败", mat_new + 1);
        clutch_release_all();
        return false;
    }
    clutch_set_busy(true);
    ams_log("  进料（蠕动）：%u 次 × %ums @ %u%%",
            (unsigned)CREEP_EXCHANGE_TIMES, (unsigned)CREEP_EXCHANGE_MS,
            (unsigned)config_get()->creep_speed_pct);
    for (int i = 0; i < CREEP_EXCHANGE_TIMES; i++) {
        motor_run_speed(MOTOR_DIR_FEED, CREEP_EXCHANGE_MS,
                        config_get()->creep_speed_pct);
        motor_stop();
        if (i + 1 < CREEP_EXCHANGE_TIMES) {
            vTaskDelay(pdMS_TO_TICKS(CREEP_GAP_MS));
        }
    }
    clutch_release_all_settled();
    ams_log("  进料（蠕动）：蠕动收尾完成");
    return true;
}

/* ==========================================================================
 * 工具：日志节流 / 打印机弹窗自动确认
 * ========================================================================== */

/**
 * 日志「状态变化才显示」—— 周期性动作的降噪。
 *
 * ★ 现场要求（2026-09-24）原话："精简设备日志只显示主要信息，连续的报文
 *   只显示一条…只有在状态变化时显示。"
 *
 * 演进过程（两个版本，别退回去）：
 *   v1 log_throttle(key, window) —— 按**时间**节流：key 不变也每 window 打
 *      一行。校准阶段每 2 秒一次、打印中每 5 秒一次，一小时能刷出上百行；
 *      而且那句"继续中（已省略 28 条相同日志）"**本身也是噪音** ——
 *      现场截图里投诉的正是它。
 *   v2 本函数 —— 按**内容**节流：key 变了才打。同样的动作重复一万次
 *      也只留一行，日志从"采样流"变回"事件流"。
 *
 * 为什么还留一个 ttl_ms：完全不重复的话，连续几小时的打印期间日志会安静得
 * 让人怀疑是不是卡死了。ttl_ms 到点后允许再打一行当**慢心跳**（措辞会说明
 * 是心跳）；填 0 = 永不重复，只认变化。
 *
 * slot 是"记忆槽"：不同用途的日志各记一槽，否则两种日志会互相把对方的
 * key 顶掉，变成轮流刷屏 —— 那比不节流还糟。
 */
enum {
    /** 辅助送料主行（为什么送 / 占空比 / 时长 / 周期） */
    LOGSLOT_ASSIST = 0,
    /** 辅助送料执行回执（离合、duty、硬件回读） */
    LOGSLOT_ASSIST_RC,
    /** 「载入打印材料」阶段的主动推料 */
    LOGSLOT_STAGE24,
    /** 缓冲器闭环：补料 / 停料 / 保护降级（2026-10-14） */
    LOGSLOT_BUF,
    /** 「微动触发了但被门禁挡住」的提示（没有打印任务 / 不是当前通道） */
    LOGSLOT_TRIG,
    LOGSLOT_COUNT          /* 必须放在最后 */
};

/** 每个槽上次打过的 key；空串 = 没打过（下一发一定通过） */
static char    s_log_key[LOGSLOT_COUNT][112];
/** 每个槽上次打印的时刻 */
static int64_t s_log_key_us[LOGSLOT_COUNT];

/**
 * 只在 key 变化（或距上次已过 ttl_ms）时返回"该打"。
 *
 * @param slot   记忆槽，见 LOGSLOT_*
 * @param ttl_ms 允许重复的最短间隔；**0 = 永不重复**（纯变化触发）
 * @return -1 = 本次不打印；
 *          0 = key 变化后的第一行（调用方按"状态变化"措辞）；
 *          1 = 到了 ttl_ms 的慢心跳（调用方按"仍在继续"措辞）
 */
static int log_changed(int slot, uint32_t ttl_ms, const char *key)
{
    if (slot < 0 || slot >= LOGSLOT_COUNT) {
        return 0;              /* 槽号写错了就当"每次都打"，别把日志吞掉 */
    }
    int64_t now  = esp_timer_get_time();
    bool    same = (s_log_key[slot][0] != '\0') &&
                   (strcmp(key, s_log_key[slot]) == 0);

    if (same) {
        if (ttl_ms == 0 ||
            (now - s_log_key_us[slot]) < (int64_t)ttl_ms * 1000) {
            return -1;
        }
        s_log_key_us[slot] = now;
        return 1;
    }

    snprintf(s_log_key[slot], sizeof(s_log_key[slot]), "%s", key);
    s_log_key_us[slot] = now;
    return 0;
}

/**
 * 清空记忆槽。slot < 0 = 全部清空。
 *
 * ★ 必须挂在"一次换色"的边界上（do_exchange 开头）：不变量是"标注过的东西
 *   要能重新标注"。如果不请，第二次换色时辅助送料的回执会被判成"和上次
 *   一样"而再也不打 —— 现场会以为功能坏了，反过来又变成一次新的"电机没动"
 *   悬案。这条边界和打印机的一次换料对话是重合的。
 */
static void log_changed_reset(int slot)
{
    for (int i = 0; i < LOGSLOT_COUNT; i++) {
        if (slot >= 0 && i != slot) {
            continue;
        }
        s_log_key[i][0]  = '\0';
        s_log_key_us[i]  = 0;
    }
}

/**
 * 打印机弹窗自动确认。
 *
 * ★ 现场要求（2026-09-23）：换料时送料没到位，打印机会弹「耗材已加载 /
 *   请拉出耗材」这类框，必须**手动在打印机屏上点一下**才能继续 ——
 *   而那时料其实已经到位了，纯粹是在等人。
 *
 * 做法：换料流程走完（料确实送进去了）之后，如果打印机还挂着错误码，
 *   就发一条 `clean_print_error` 把它按掉。
 *
 * ⚠️ 只在这一个位置发、只在"我们刚把料送好"之后发：
 *   这个命令是按掉打印机**当前**的错误，无脑发会把真故障也一起吞掉。
 *
 * @param mat 刚送好的料盘位；< 0 表示没送到 —— 此时**不按**，别掩盖真故障
 */
static void auto_confirm_printer_dialog(int mat)
{
    if (mat < 0) {
        return;
    }
    int err = 0;
    if (xSemaphoreTake(s_report_lock, 0) == pdTRUE) {
        err = s_last_report.print_error;
        xSemaphoreGive(s_report_lock);
    }
    if (err == 0) {
        return;    /* 没有弹窗，什么都不用做 */
    }

    char js[192];
    int n = snprintf(js, sizeof(js),
                     "{\"print\":{\"sequence_id\":\"%u\","
                     "\"command\":\"clean_print_error\",\"print_error\":%d}}",
                     (unsigned)(esp_timer_get_time() / 1000), err);
    if (n <= 0 || n >= (int)sizeof(js)) {
        return;
    }
    if (bambu_mqtt_publish(js, n) >= 0) {
        ams_log("  弹窗自动确认：已按掉打印机的错误码 %d（料已就位）", err);
    } else {
        ams_log_warn("  弹窗自动确认失败（MQTT 未连接？），"
                     "如果打印机还在等，请在屏上手动点一下");
    }
}

/**
 * 换料主流程（重写版）。
 *
 * 时序：
 *   1. 探测当前料盘（有微动时）
 *   2. 退料：先把当前通道的料从挤出机/缓冲区收回到料盘
 *   3. 进料：把新通道的料送到挤出机入口，等到位后蠕动收尾
 *   4. 辅助送料：帮打印机把新料咬住
 *   5. 发 resume 让打印机继续 G-code（冲刷、流量校准等由打印机完成）
 *
 * ★ 关键改动（修复暂停后卡死问题）：
 *   - 原来在暂停状态下发 M400/M83 是无效的（暂停中 G-code 排队不执行），
 *     导致 wait_printer_progress 超时，整个流程卡住 ~50s 不发 resume。
 *   - 现在改为：退料/进料全程不做打印机同步等待，只做 AMS 侧动作。
 *     完成后发 resume，打印机自己继续执行 M73 P101 之后的冲刷 + 流量校准。
 *   - 冲刷完进入校准阶段（stg=8，本机流量校准就是它）后，由 handle_report
 *     的辅助送料逻辑**周期性地**推料，见那边的注释。
 */
static bool do_exchange(int printer_channel)
{
    int mat_new = config_material_index_of(printer_channel);
    if (mat_new < 0) {
        record_error("换料失败：打印机通道 %d 没有对应的料盘位", printer_channel);
        return false;
    }

    set_state(AMS_STATE_EXCHANGE, mat_new);

    /* ★ 一次换色的起点。
     *   它有两个用途：
     *     ① 给日志算出"本次换色耗时"（现场要求：换色完成时给出时长）；
     *     ② 清空"日志状态变化"记忆槽 —— 见 log_changed_reset 的说明，
     *        不清的话下一轮换色的辅助送料回执会被判成"和上次一样"而消失。 */
    int64_t t_all          = esp_timer_get_time();
    int64_t t_wait_done    = 0;   /* 等到打印机报"该退料了"的时刻 */
    int64_t t_retract_done = 0;   /* 退料做完的时刻 */
    log_changed_reset(-1);

    /* ★ 日志样式对齐 Top-AMS 的现场显示（用户要求一眼能看懂换到第几次、换哪一路）：
     *     ######## 开始第 N 次换色 ########
     *     当前通道 X → 目标通道 Y
     *     （随后）正在退出当前通道 X → 退料完成 → 开始送入通道 Y → 送料完成 */
    ams_log("######## 开始第 %u 次换色 ########",
            (unsigned)(s_diag.exchange_ok + s_diag.exchange_fail + 1));

    int current = config_get_filament_current();
    current = probe_current_filament(current);

    /* ★ 挤出机"有没有料"这个上报（MQTT hw_switch_state）只能当**参考**，
     *   不能当权威判据：
     *   - extruder_inplace_hint == 0  → 打印机明确说无料，可以省掉电机退线这一步
     *   - extruder_inplace_hint == 1  → 只说明"工具头检测到有料"，
     *     **不代表**里面就是目标通道的料。实测 2026-09-22 那一整场打印，
     *     这个键 432 帧报告里只出现过 1 次、值恒为 1（增量报文只在值变化时
     *     才带键 = 从头到尾没变过），所以它给不出"料现在在不在"的信息。
     *   - extruder_inplace_hint == -1（还没收到过上报）→ 按"有料"处理，别省步骤
     *
     *   ★ 绝对不要再用它去决定"要不要换料"—— 那条路已经坑死过一次，
     *     见下面那段长注释。 */
    int hint = -1;
    {
        if (xSemaphoreTake(s_report_lock, 0) == pdTRUE) {
            hint = s_last_report.extruder_inplace_hint;
            xSemaphoreGive(s_report_lock);
        }
    }
    bool extruder_empty = (hint == 0);

    char cur_txt[48] = {0};
    char new_txt[48] = {0};
    ams_channel_text(current, cur_txt, sizeof(cur_txt));
    ams_channel_text(printer_channel, new_txt, sizeof(new_txt));

    if (extruder_empty) {
        ams_log("  挤出机无料，跳过退料，直接进新通道 %s", new_txt);
    } else {
        ams_log("  当前通道 %s，目标通道 %s", cur_txt, new_txt);
    }
    /* ----------------------------------------------------------------------
     * ★★ 这里原来有一条"挤出机有料 且 current==目标通道 → 无需更换，直接 resume"，
     *    2026-09-22 真机事故后**已删除**。别看它像优化，它必然把打印搞死。
     *    三条理由，条条都是实测：
     *
     *  ① 走到这里的时候，料**已经被切断、热端已经是空的**。
     *     handle_report 一拿到通道号就把 `M620 S255 / T255 / M621 S255`
     *     （切刀 + 把热端里的料吐出来）发出去了 —— 因为那条指令必须趁
     *     打印机还在 RUNNING 时发。也就是说"切刀"和"判要不要换"的顺序
     *     本来就是反的；再判成"不用换"，就变成"切了没人装回去"。
     *
     *  ② 打印机此刻就停在 stg_cur=24（**载入打印材料**），它要的正是一次进料。
     *     2026-09-22 真机时间线（打印机侧报文）：
     *       +319s  stg_cur=24 + gcode_state=PAUSE ← 换料请求
     *       +321s  bed_target_temper 3 → 76      ← 我们的 M190 生效
     *       +325s  gcode_state=RUNNING            ← resume 生效，但没进料
     *       +368s  print_error=318750723          ← 等不到料，报错
     *     打印机在"载入打印材料"这一步等的是**料**，不是 resume。
     *
     *  ③ 这条判定依赖的两个输入都不可信：
     *     · NVS 里的"当前通道"是上次手动上料留下的值，随时可能过期；
     *     · hw_switch_state 实测整场打印 432 帧里只出现 1 次、值为 1
     *       （增量报文只在"值变化"时才带这个键，说明**一次都没变过**），
     *       拿它判"挤出机有料"等于没有判据。
     *     本次事故就是两个同时踩中：NVS 记的 3 == 目标 3，但料已经被切走。
     *
     *  Top-AMS 的"同一耗材直接 resume"能成立，是因为它把判定放在发切刀
     *  **之前**（main.cpp:298，和 change_filament 互斥），压根不会出现
     *  "切了再说不换"。我们这里判定在切刀之后，只能二选一 ——
     *  **既然已经切了，就必须退料 + 进料把它装回去**。
     *
     *  结论：**"跳过"这件事只能判定在切刀之前**。2026-09-23 已按现场要求
     *  把它挪到 handle_report 的「② 同料判定」里 —— 那边判完直接 resume，
     *  压根不会发出切刀。到了这个函数里的请求，一律是"料真的需要动"的那种，
     *  老老实实走完整流程，**不要在这里再加"跳过"分支**（在这里跳 =
     *  切了没人装回去）。
     * ---------------------------------------------------------------------- */
    if (current == printer_channel) {
        ams_log("  注意：目标通道 %s 与记录的当前通道相同 —— 能走到这里说明"
                "当时挤出机是空的（同料跳过那条要求「挤出机有料」才生效），"
                "所以仍然要把料装回去", new_txt);
    }

    /* ---------- 步骤〇：★ 等打印机把"切刀 + 退到冲刷区"做完 ----------
     *
     * 2026-09-22 真机暴露的顺序错误：handle_report 一拿到通道号就把
     * `M109 + M620 S255/T255/M621 S255` 发出去了（那是对的），但打印机
     * 收到之后还要先**把喷嘴升到 M109 的目标温度**（实测这一步 42 秒），
     * 再切刀、再移回冲刷区把热端里那截料吐出去 —— 干完这些它才报
     * `ams_status=260`（"退料完成，需要退线"）。
     *
     * 旧固件只盲等 3000ms 就开始退料蠕动，那时料还是完整的一根、还被挤出机
     * 齿轮咬着，我们的电机在打滑，`hw_switch_state` 从头到尾没变成 0，
     * 打印机最后卡在 stg_cur=24（载入打印材料）上报错。
     *
     * 这一步就是把它那半场等完。等不到（超时 / 机器不给这个字段）也会继续，
     * 只是日志会显式告警 —— 宁可退得糙一点，也不能把打印机卡在暂停态。 */
    wait_printer_need_withdraw_ms(AMS_UNLOAD_READY_TIMEOUT_MS);
    t_wait_done = esp_timer_get_time();

    /* ---------- 步骤一：退料 ----------
     * 把当前通道的料从挤出机/缓冲区收回到料盘。
     * 退料由 AMS 电机完成，不需要打印机侧配合（打印机暂停中
     * G1 E-50 不会执行，发 M400 也没用）。
     *
     * ★ 走到这里"要不要换"已经不用再判了 —— 见上面那段说明：切刀都发过了，
     *   料必须在这次流程里被退出来、再把新料送进去。只有两种例外会跳过退料：
     *     · 打印机明确上报挤出机无料（hint == 0）；
     *     · NVS 里没有"当前通道"（extruder == 0），压根不知道该退哪一路。
     *   后一种必须显式告警，别让日志看着像"正常跳过"。 */
    if (!extruder_empty && current > 0) {
        int mat_cur = config_material_index_of(current);
        if (mat_cur >= 0) {
            /* 日志样式对齐 Top-AMS 的现场显示：正在退出当前通道 N → 退料完成 */
            ams_log("正在退出当前通道 %s …", cur_txt);
            /* 打印机的快速退料（切刀 + 吐料 + 退到冲刷区）已经在
             * handle_report 里发过、并且上面已经等到它报 260 了。
             * 这里只做属于我们自己的那半件事：把料收回料盘。 */
            if (!do_retract(mat_cur)) {
                record_error("退料失败，换料中止");
                s_diag.exchange_fail++;
                set_state(AMS_STATE_ERROR, -1);
                return false;
            }
        } else {
            /* 通道号有效但查不到对应料盘位 —— 是配置问题，别静默跳过 */
            ams_log_warn("  退料：通道 %d 找不到对应料盘位，跳过退料", current);
        }
    } else if (current <= 0) {
        /* NVS 里 extruder == 0。Top-AMS 遇到这种情况是直接报错的
         * （"请设置当前所使用通道,否则无法退料再进料"），我们选择继续：
         * 打印机已经切了刀、正等料，报错只会更糟。但要显著告警。 */
        ams_log_warn("  退料：记录里没有当前通道（NVS extruder=0），"
                     "电机不知道退哪一路 —— 只能靠打印机刚才那次快速退料清空，"
                     "直接送新料（下次上料后请在网页把「当前通道」设对）");
    } else {
        ams_log("  退料：打印机已报挤出机无料，跳过退料步骤");
    }

    t_retract_done = esp_timer_get_time();

    /* ---------- 步骤二：进料 + 蠕动 ----------
     * 把新通道的料送到挤出机入口，等到位后蠕动收尾确保咬住。
     * 挤出机到位信号：C3 走 MQTT hw_switch_state，S3 走 GPIO。 */
    ams_log("开始送入通道 %s …", new_txt);

    /* ★ 先把热端升到新料的温度再送料（见 exchange_heat_nozzle）。
     *   温度不对，料头会在冷热端里顶住、打弯。 */
    exchange_heat_nozzle(mat_new);

    /* ★ 先让打印机的挤出机齿轮转起来（Top-AMS main.cpp:135 的做法：
     *   `G1 E150 F500` → 等 3 秒 → 再让自己的电机送料）。
     *
     *   这是"料送不进去"的关键一环：只靠我们自己推，料头顶到挤出机齿轮口
     *   就停住了 —— 齿轮不转，料进不了咬合点，`hw_switch_state` 永远变不成
     *   1，打印机就一直卡在 stg_cur=24 等料。齿轮跟着转，料才能被咬住、
     *   被带进热端，随后 resume 之后的冲刷块（切片宏里的 FLUSH_START）
     *   才能真正把新料挤出来。
     *
     *   ⚠️ 这条指令在打印机暂停态也会执行（和 M190 / M620 一样）。
     *   150mm / F500 是 Top-AMS 的取值，别乱改 —— 太小料咬不到位。 */
    {
        const char *gear = "G1 E150 F500";
        if (bambu_mqtt_send_gcode(gear) >= 0) {
            ams_log("  进料：已请求挤出机齿轮辅助进料（%s），等 3 秒让它转起来",
                    gear);
        } else {
            ams_log_warn("  进料：齿轮辅助指令没发出去（MQTT 未连接？），"
                         "只靠本机电机推，料可能咬不住");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }

    /* ★ stop_on_buf_switch = false（2026-10-14 现场要求）：
     *   换料途中**不执行"微动到位就停"** —— 缓冲器阻力小，料会先在那里堆住
     *   把微动顶住，按它停就送不到挤出机，打印机随后报故障。
     *   这条路只认打印机的「耗材已到挤出机」信号。 */
    if (!do_load(mat_new, false)) {
        record_error("进料失败，换料中止");
        s_diag.exchange_fail++;
        set_state(AMS_STATE_ERROR, -1);
        return false;
    }

    /* 蠕动收尾（等到位 → 5 次 × 1 秒），即使等不到也强制执行。
     * ★ 这里只再等 5 秒：do_load 已经闭环推了最多 20 秒，而且全程在盯
     *   到位信号（信号一到就停）。再干等 15 秒只会让打印机在暂停态多停
     *   十几秒 —— 现场反馈正是"等待过程太长"（2026-09-23）。 */
    if (!feed_until_extruder_ms(mat_new, 5000)) {
        ams_log_warn("  未等到挤出机到位，料可能没咬住，仍尝试继续");
    }

    /* ★ 收尾补救：如果打印机此时正挂着"缺料 / 请拉出耗材"这类弹窗，
     *   而料其实已经被我们送进去了，就替现场把它按掉。
     *   （现场要求："打印机如果弹窗了，能不能自动进行确认"） */
    auto_confirm_printer_dialog(mat_new);

    /* ---------- 步骤三：记录状态 + resume ----------
     * 辅助送料不在这里做——正确的时机在 handle_report 的 stg=8/19/0 阶段。
     * 冲刷（切片的 FLUSH_START 块）由打印机在 resume 之后自己执行，
     * 固件**不要**再加一次，否则会双重冲刷。 */
    config_set_filament_current(printer_channel);
    s_diag.exchange_ok++;
    ams_log("送料完成：通道 %s 已装载到挤出机", new_txt);

    /* ★ 本次换色耗时（现场要求 2026-09-24："在日志中换色完成后增加一个本次
     *   换色时长"）。
     *
     *   为什么还要跟一条分段明细：只给总数的话，38 秒到底是"等切刀等掉了"
     *   还是"我们自己的进度慢"，光看总数分不出来 —— 而这两件事的处置完全
     *   相反（前者要找打印机/切片侧，后者要调板子的参数）。
     *   口径（与 run7 的板子侧统计一致）：
     *     等切刀 = 开始换色 → 打印机报 ams_status=260（切刀 + 吐料 + 移床）
     *     退料   = 260 → 料收回料盘
     *     进料   = 退料完 → 料送到挤出机并蠕动收尾
     *   ⚠️ 不包含后面"发送 resume 之后"的冲刷 / 校准 —— 那段是切片宏在跑，
     *      不在本函数的计时范围里。 */
    {
        int64_t t_end = esp_timer_get_time();
        /* ⚠️ 这一行**必须短**：超出 AMS_LOG_LINE_MAX 会被静默截断，而截掉的
         *   恰好是后面那半句（"本次耗时"就在后半段）。
         *   所以这里只给通道号，颜色在 上面的 "正在退出当前通道 …" /
         *   "开始送入通道 …" 两行里已经有了。
         *   通道号不能用 config_get_filament_current() —— 上一行
         *   config_set_filament_current() 刚把它改成新通道，那样会打印成
         *   "通道3→通道3"。源通道是 current。 */
        /* ⚠️ 缓冲必须 ≥ 18 字节，别改小：GCC 算 `%d` 的最坏长度按
         *   -2147483648 取 11 字符，加上字面量 "通道" 的 6 字节 = 17，
         *   再留 NUL 就是 18。原来的 16 字节正好差一点，被
         *   -Werror=format-truncation 判死（CI #60 就是挂在这行）。
         *   实际通道号只有 1~4，但编译器不知道，所以这里按最坏情况给。 */
        char from_lbl[24];
        if (current > 0) {
            snprintf(from_lbl, sizeof(from_lbl), "通道%d", current);
        } else {
            snprintf(from_lbl, sizeof(from_lbl), "未知");
        }
        ams_log("换色完成 %s→通道%d 本次耗时 %.1f 秒（已发 resume）",
                from_lbl, printer_channel,
                (double)(t_end - t_all) / 1000000.0);
        ams_log("  明细：等切刀 %ums 退料 %ums 进料 %ums",
                (unsigned)((t_wait_done - t_all) / 1000),
                (unsigned)((t_retract_done - t_wait_done) / 1000),
                (unsigned)((t_end - t_retract_done) / 1000));
    }
    set_state(AMS_STATE_IDLE, -1);

    /* resume 让打印机的 G-code 从 M73 P101 之后继续执行：
     * 冲刷（M73 P102/P103）→ 流量校准（M620/M621）→ 恢复打印 */
    bambu_mqtt_send_resume();
    return true;
}

/* ==========================================================================
 * 七、打印机上报处理
 * ========================================================================== */

/**
 * MQTT 回调（跑在 esp-mqtt 的任务里）。
 *
 * ★ 这里只做"搬数据"，绝不做耗时操作 —— 具体处理交给 ams_task。
 *   用互斥锁而不是队列，因为上报是**累积状态**：连来 3 条的话，
 *   最新的那条已经包含前面所有信息，中间的直接覆盖掉即可，不需要排队。
 */
static void on_mqtt_report(const bambu_report_t *report, void *user)
{
    (void)user;
    if (report == NULL) {
        return;
    }
    if (s_report_lock &&
        xSemaphoreTake(s_report_lock, pdMS_TO_TICKS(50)) == pdTRUE) {
        s_last_report = *report;
        s_has_report = true;
        s_report_pending = true;
        xSemaphoreGive(s_report_lock);
    }
}

/**
 * 自动颜色匹配辅助函数。
 * 输入目标颜色（0xRRGGBB），输出最接近的打印机通道号（1 起）。
 * 如果 4 个通道都没配颜色（全 0），返回 -1。
 */
static int auto_match_channel(int target_color)
{
    if (target_color < 0) {
        ams_log("  报文未携带颜色信息，无法自动匹配");
        return -1;
    }
    int best_mat = -1;
    uint32_t dist = config_color_match((uint32_t)target_color, &best_mat);
    if (best_mat < 0) {
        ams_log_warn("自动匹配失败：4 个通道都没配颜色，退回按通道号换料");
        return -1;
    }
    int printer_ch = config_printer_channel_of(best_mat);
    if (printer_ch < 1 || printer_ch > BOARD_CHANNEL_COUNT) {
        ams_log_warn("自动匹配到料盘位 %d，但未映射到打印机通道", best_mat + 1);
        return -1;
    }
    /* ★ 一行说完就够了（现场要求"只显示主要信息"）。
     *   原来这里跟一段"通道N 颜色: 0x……"的 4 行循环 —— 那是排查期的草稿：
     *   颜色在网页「通道设置」一眼就能看到，日志里刷 4 行只会把真正的事件
     *   （换色开始 / 退料 / 送料）冲散。 */
    ams_log("自动匹配：目标 0x%.6X → 通道%d（色差 %lu）",
            (unsigned int)(target_color & 0xFFFFFF),
            printer_ch, (unsigned long)dist);
    return printer_ch;
}

/**
 * 处理一条打印机上报（在 ams_task 里调用，可以阻塞，几秒无所谓）。
 *
 * 三件事：
 *   1. 如果报文带目标颜色，用自动匹配覆盖默认的通道号
 *   2. 如果打印机请求换料，把换料命令排进队列（带去重，一秒只排一次）
 *   3. 如果打印机处在校准 / 打印阶段（stg=8、19、0），**周期性地**做辅助
 *      送料，帮打印机把料咬住。校准阶段（stg=8）本身要跑两三分钟，所以
 *      这里是反复做，不是只做一次 —— 见下方注释里 2026-09-22 的真机教训。
 */
/** 「载入打印材料」阶段自动送料的起点（0 = 当前不在送）。见 handle_report */
static int64_t s_stage24_push_start_us;

/** 「载入打印材料」阶段自动送料的时长上限（秒）—— 到点就停，别把料盘喂空 */
#define AMS_STAGE24_PUSH_MAX_S 45
/** 「载入打印材料」阶段自动送料的占空比：全速。慢速在这个机构上带不动 */
#define AMS_STAGE24_PUSH_PCT   100

/* ==========================================================================
 * 辅助送料「阶段内保持离合吸合」
 * ============================================================================
 * ★ 现场要求（2026-09-23）原话：
 *     "在需要辅助送料阶段，全程保持吸合，值脉冲电机，其他阶段电磁断开。
 *      这个红线是同时只能 1 路吸合。"
 *
 * 改动前的做法：每送一次都是**一整套**动作 ——
 *     clutch_engage() → 转 assist_ms → motor_stop() → clutch_release_all_settled()
 * 而校准阶段（stg=8）只停 0.5 秒就再来一次，于是 3 分 15 秒里
 * **吸合-断开 97 次**（run7 实测数字）—— 离合线圈和齿轮在反复咬合脱开。
 *
 * 改动后：只要还处在"需要辅助送料的阶段"，离合就**一直吸着**，电机照旧
 * 按周期脉冲（转 assist_ms → 停 → 再转）。离开这些阶段、或者打印机来了
 * 换料请求、或者任何别的动作要用总线 → 立刻断开（"其他阶段电磁断开"）。
 *
 * ★★ 红线：任何时刻最多只允许 1 路吸合 ★★
 *   这条**不靠本模块自己保证**，而是继续走 clutch_engage()：它内部第一步
 *   就是"无条件把 4 路全写成断开 → 等机械脱开 → 回读复核 → 才吸合目标"，
 *   所以"保持"永远不会退化成"又吸了一路"。同一路重复调用是幂等的
 *   （clutch_engage 直接返回，不做机械动作），所以每拍调一次也不会抖。
 *   主循环每周期还会调一次 clutch_assert_single(true) 兜底。
 *
 * ⚠️ 为什么保持期间**不设** AMS_STATE_ASSIST 状态：
 *   设了会让 ams_is_busy() 在整段校准/打印期间恒为 true —— 手动点动会被
 *   "设备正忙"挡住、状态灯一直亮、"载入打印材料"的补推也进不来。
 *   保持的只是**离合的机械状态**，不是"总线被占"。所以状态照旧只在
 *   真正脉冲的那 ms 里是"辅助送料中"。
 */
/** 当前为哪一路保持吸合；-1 = 没有保持 */
static int      s_assist_hold_mat = -1;
/** 保持期间累计脉冲了几次（日志/网页用，不参与控制） */
static uint32_t s_assist_hold_pulses;
/** 本轮保持的开始时刻（esp_timer，微秒）；0 = 没有在保持 —— 散热上限用 */
static int64_t  s_assist_hold_start_us;
/** 散热窗口的结束时刻；在这个时刻之前不重新吸合 */
static int64_t  s_assist_hold_cool_until_us;

/* ---- 连续保持的上限（安全阀，不是功能参数）----
 *
 * ★ 为什么"全程保持吸合"还必须有这个上限：
 *   "打印中"（stg_cur=0）这个阶段是**整场打印**，几小时。真按字面"全程保持"
 *   执行，电磁离合的线圈要连续通电几个小时 —— 那是实打实的发热（线圈按几瓦
 *   算，摸上去会烫），而离合线圈过热会退磁、驱动板（ULN2803）也一直在抗那个
 *   电流。机械寿命是保住了，线圈寿命要赔进去。
 *
 *   5 分钟这个数的取法：
 *     · 校准阶段（stg=8）总共只有 3 分 15 秒（run7 实测）—— 上限**永远不会**
 *       在校准里触发，所以"校准期间全程吸着"这个核心诉求原样保留；
 *     · 打印中每小时最多断 12 次、每次只断 3 秒 → 相对原来"每 2 秒咬一次"
 *       （每小时 1800 次）已经少了 99.3%，机械那笔账照样赚到了。
 *
 *   ★ 想让它彻底不松口（离合摸上去不烫就可以），把 ASSIST_HOLD_MAX_MS 调大
 *     或改成 UINT32_MAX 即可 —— 但请先实测连续保持 10 分钟后的线圈温度。 */
#define ASSIST_HOLD_MAX_MS   300000u  /* 连续保持最多 5 分钟 */
#define ASSIST_HOLD_COOL_MS  3000u    /* 到点后断开散热 3 秒，再自动吸回去 */

/**
 * 结束保持：断开全部离合。不在辅助阶段 / 有别的动作要用总线时调它。
 * 没在保持状态时是空操作，可以随便调。
 */
static void assist_hold_release(void)
{
    if (s_assist_hold_mat < 0) {
        return;
    }
    int      mat = s_assist_hold_mat;
    uint32_t n   = s_assist_hold_pulses;
    s_assist_hold_mat    = -1;
    s_assist_hold_pulses = 0;
    /* 本轮计时清零：下一次吸合是**新的一轮**保持，散热阀的 5 分钟从头算。
     * 不清的话，热保护断开 3 秒后吸回来时，start_us 还是 5 分钟前那个值
     * → 下一拍立刻又被判超时 → 变成"每 3 秒断一次"，散热窗口白设。 */
    s_assist_hold_start_us = 0;
    clutch_release_all_settled();
    ams_log("辅助送料保持结束：通道%d 离合已断开（本轮共脉冲 %u 次）",
            mat + 1, (unsigned)n);
}

/**
 * 保持模式下的一次脉冲：确保目标通道吸合，然后**只**让电机转 ms。
 *
 * 和 drive_channel_speed() 的唯一区别：收尾**不释放离合**。
 *
 * @return true = 离合吸合成功（不代表"带得动"，那要看回执里的硬件回读）
 */
static bool assist_hold_pulse(int material_index, uint8_t pct, uint32_t ms)
{
    /* 先把回执清成"什么都没发生"，任何提前 return 都会把它留在这个状态 ——
     * 调用方据此就能分清"日志打了但动作没发生"。 */
    s_last_drive.clutch_ok = false;
    s_last_drive.skipped   = false;
    s_last_drive.dir       = 1;
    s_last_drive.speed_pct = pct;
    s_last_drive.duty_raw  = (uint16_t)motor_pct_to_duty(pct);
    s_last_drive.ran_ms    = 0;
    s_last_drive.duty_in1  = 0;
    s_last_drive.duty_in2  = 0;
    s_last_drive.level_in1 = -1;
    s_last_drive.level_in2 = -1;

    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        record_error("辅助送料保持：料盘位 %d 不存在", material_index + 1);
        return false;
    }

    /* ★ 红线在这里：走 clutch_engage() 而不是直接写引脚。
     *   它内部无条件先全部断开再吸合目标；同一路重复调用幂等。 */
    esp_err_t err = clutch_engage(material_index + 1);
    if (err != ESP_OK) {
        record_error("辅助送料保持：料盘位%d 离合吸合失败（%s）—— "
                     "本次**没有**驱动电机", material_index + 1,
                     esp_err_to_name(err));
        clutch_release_all();
        s_assist_hold_mat = -1;      /* 保持失效，下一拍重新吸 */
        return false;
    }
    clutch_set_busy(true);
    s_last_drive.clutch_ok = true;
    s_assist_hold_mat      = material_index;
    s_assist_hold_pulses++;

    /* 本轮保持的起点：只在"刚吸上"那一拍记一次。
     * 之后的每一拍 clutch_engage() 走幂等短路、不复位这个值 ——
     * 否则散热阀的 5 分钟会被无限续期，永远不触发。 */
    if (s_assist_hold_start_us == 0) {
        s_assist_hold_start_us = esp_timer_get_time();
    }

    /* 幂等短路探测（同 drive_channel_speed）：motor_set_dir_speed 遇到
     * "同方向 + 同速度 + 已在转"会直接返回、完全不碰硬件。 */
    if (motor_get_dir() == MOTOR_DIR_FEED &&
        motor_get_speed_pct() == (int)pct &&
        motor_elapsed_ms() != 0) {
        s_last_drive.skipped = true;
    }

    motor_run_speed(MOTOR_DIR_FEED, ms, pct);
    s_last_drive.ran_ms = ms;

    motor_readback_t rb = {0, 0, -1, -1};
    motor_readback(&rb);          /* ★ 必须在 motor_stop() 之前读 */
    motor_stop();                 /* ★ 只停电机 —— 离合**保持吸合** */

    s_last_drive.duty_in1  = rb.duty_in1;
    s_last_drive.duty_in2  = rb.duty_in2;
    s_last_drive.level_in1 = (int8_t)rb.level_in1;
    s_last_drive.level_in2 = (int8_t)rb.level_in2;
    return true;
}

/**
 * 当前保持吸合的料盘位 +1（1~4）；0 = 没有保持。网页「硬件状态」显示用。
 * 只读一个 int，跨任务读不需要锁。
 */
int ams_assist_hold_channel(void)
{
    return (s_assist_hold_mat >= 0) ? (s_assist_hold_mat + 1) : 0;
}

/**
 * 缓冲补料脉冲（2026-10-14）：保持离合吸合 + 让电机转 ms，
 * **期间分段复查「停止送料微动」，触发即提前收手**。
 *
 * ★ 为什么不直接用 assist_hold_pulse() 转满 ms：
 *   现场要的是"以缓冲器微动的动作来执行**停止送料**"。转满 1.5 秒才停的
 *   话，缓冲器早就被顶满了 —— 微动这时候只是"事后确认"，等于没起作用。
 *   分段（AMS_FILAMENT_STEP_MS，500ms）复查才能真正在微动触发的那一拍收手。
 *
 * ★ 为什么不直接用 drive_channel_speed(stop_on_sensor=true)：
 *   那个函数收尾会 `clutch_release_all_settled()` 把离合断开 ——
 *   而打印期间我们希望离合**一直吸着**（现场 2026-09-23 的要求：
 *   "在需要辅助送料阶段，全程保持吸合，只脉冲电机"）。所以复用
 *   assist_hold_pulse()，它只停电机、不放离合。
 *
 * @param out_stopped 非 NULL 时返回"是不是被停止送料微动提前结束的"
 * @return true = 硬件层没出错（不代表送到了）
 */
static bool buffer_feed_pulse(int material_index, uint8_t pct, uint32_t ms,
                              bool *out_stopped)
{
    if (out_stopped) {
        *out_stopped = false;
    }
    uint32_t remain = ms;
    while (remain > 0) {
        uint32_t seg = (remain > AMS_FILAMENT_STEP_MS) ? AMS_FILAMENT_STEP_MS
                                                       : remain;
        if (!assist_hold_pulse(material_index, pct, seg)) {
            return false;
        }
        remain -= seg;
        if (sensor_triggered(material_index, SENSOR_STOP)) {
            if (out_stopped) {
                *out_stopped = true;
            }
            break;
        }
    }
    return true;
}

static void handle_report(const bambu_report_t *r)
{
    /* ---- 热床温度记忆（必须在用它之前做）----
     * 切片用 M140 S{next_extruder + 1} 借"目标床温"传通道号，所以换色时
     * 打印机上报的 bed_target 会变成 1~16。真实床温要在被改写**之前**
     * 记住，换完再用 M190 还回去 —— 不还的话热床真的会按 M140 的设定
     * 降到 1~4℃，当前这一层直接粘不住。 */
    if (!r->has_bed_target) {
        /* ★ 这一帧压根没提床温 —— 打印机只发增量字段，**什么都别做**。
         *   2026-09-22 真机事故：以前这里把解析层的默认值 0 当成"床温真的
         *   关了"，于是每收到一帧不带该字段的报文就把刚记住的 76℃ 清掉。
         *   换色那一刻必然"没有热床温度的历史记录" → M190 从没发出去 →
         *   热床被 M140 S{next+1} 留在 3℃ 凉了一整晚。
         *   这一条就是那次卡死的根因。 */
    } else if (r->bed_channel > 0) {
        /* 这一帧的床温是通道号，不是温度 —— 别污染记忆值 */
    } else if (r->bed_target >= 17.0f) {
        /* 取见过的最高值：不同层/不同材料的床温不同（PLA 首层 60、后续 55），
         * 取最高能保证恢复后不会偏低。 */
        if (r->bed_target > s_bed_target_max) {
            s_bed_target_max = r->bed_target;
            ams_log("记住热床温度 %.0f℃（换完用它发 M190 恢复）",
                    (double)s_bed_target_max);
        }
    } else {
        /* 0（空闲 / 打印结束）或 17 以下的异常值：没有可信的床温，
         * 清掉记忆，下次开始打印重新学 —— 免得拿上一次打印的旧温度去恢复。 */
        s_bed_target_max = 0.0f;
    }

    /* ★ 只记录阶段变化，不刷屏 */
    if (r->stg_cur != s_last_seen_stg) {
        ams_log("阶段：%s", bambu_stage_text(r->stg_cur));
        s_last_seen_stg = r->stg_cur;
        /* ★ 阶段一变就把辅助送料的节流清零，让新阶段**立刻**先送一次。
         *   不清的话要等满一整个周期才动，而校准阶段总共才两三分钟，
         *   开头那一下不动，现场的感受就是"这个阶段没有辅助送料"。 */
        s_last_assist_time_us = 0;
    }

    /* ---- 挤出机到位（MQTT 来源）---- */
    if (config_get()->extruder_src == EXTRUDER_SRC_MQTT &&
        r->extruder_inplace_hint == 1 &&
        !extruder_inplace_triggered()) {
        ams_log("打印机上报耗材已到挤出机");
        extruder_inplace_notify_from_mqtt();
    }
    /* hw_switch_state == 0 表示"无料"—— 不需要同步更新 extruder_inplace
     * 状态（那只在 do_retract 里通过 s_last_report 直接读取） */

    /* ---- 辅助送料：跟着打印机的阶段**周期性地**做 ----
     *
     * stg=8  （校准挤出）     → ★ 本机真机实测：流量校准就是报这个阶段，
     *                          它**从不报 19**（3 分钟的校准全靠这一条覆盖）
     * stg=19 （校准挤出流量） → 别的固件版本用这个码，留作兜底
     * stg=0  （打印中）       → 按层慢慢消耗，5 秒一次就够
     *
     * 之所以校准比打印密：校准期间挤出头在**持续**吃料，间隔放长料就拱不上；
     * 而打印要连续几个小时，间隔必须放长，否则一直顶着料容易把料憋弯、
     * 电机也会一直发热。 */

    bool assist_do = false;
    bool assist_is_buffer = false;   /* true = 本拍走"缓冲器闭环"（微动决定） */
    const char *assist_why = "";
    int assist_mat = -1;
    uint32_t assist_period_ms = ASSIST_REPEAT_US / 1000;

    /* ★★ 「打印中」这一档必须再验一次"真的在打印" ★★
     *   —— 2026-09-23 run7 真机踩到的坑，用户原话：
     *     "在阶段打印中之后就开始了辅助送料，其实这个阶段是不需要辅助送料的，
     *      打印机在自检。"
     *
     *   根因不是"辅助送料逻辑写错了"，是**阶段码被误读**：
     *   stg_cur=0 在拓竹的表里既是"打印中"，也是打印机空闲时会挂着的值。
     *   代码里原本有一条校正（print_type=="idle" 且 stg_cur==0 → 视为空闲），
     *   但收到打印任务的那一瞬间打印机是这样的：
     *       10:47:04  print_type=idle    stg_cur=0     → 校正成空闲，没事
     *       10:50:19  print_type=cloud   stg_cur=0     ← 打印任务刚进来
     *       10:50:28  stg_cur=255                         打印机才把阶段码刷成空闲
     *       10:50:30  stg_cur=2 热床预热                  真正的流程从这里才开始
     *   中间这 9 秒里 print_type 已经变成 cloud、stg_cur 还留着上一次的 0，
     *   校正条件不成立 → 板子读到"打印中" → 在**打印机自检**时开始辅助送料。
     *   日志里就是那两条 08:49.530 / 08:57.697（= 墙上时间 10:50:19~10:50:28）。
     *
     *   所以"打印中"这一档多加两条硬证据，缺一不可：
     *     · gcode_state == RUNNING（is_printing）—— 打印机自己说在跑；
     *     · mc_percent > 0 或 layer_num >= 1 —— 这个任务已经**推进过**了。
     *   自检阶段 mc_percent 还是 0、layer_num 还是 0，两条都不满足 → 不动。
     *   而真正的打印中 mc_percent 一路在涨（51% → 70% → 84% → 97%），必然满足。
     *
     *   注意：校准档（stg=8/19）**不加**这个门禁 —— 校准期间 gcode_state 是
     *   RUNNING、mc_percent 也在涨，加了也没影响，但没必要让它跟着一起变脆。 */
    bool stg0_is_really_printing = really_printing_at(r->stg_cur, r->is_printing,
                                                      r->mc_percent,
                                                      r->layer_num);

    /* ★ 本拍是否**仍处在**需要辅助送料的阶段 —— 这跟"是否到了该送的时刻"
     *   是两件事，别混（现场要求："全程保持吸合、只脉冲电机，其他阶段断开"）：
     *     assist_want_hold >= 0 → 还在阶段里 → 离合要**一直吸着**，哪怕这一拍
     *                             还没到脉冲时刻也不许断开；
     *     assist_want_hold < 0  → 不在阶段里、或打印机来换料请求了
     *                             → 立刻断开（= "其他阶段电磁断开"）。
     *   改动前是"每送一次就吸合-断开一整套"，所以校准那 3 分 15 秒里
     *   机械咬了 97 次（run7 实测）。 */
    int assist_want_hold = -1;

    if (config_get_assist_enabled() &&
        (r->stg_cur == 8 || r->stg_cur == 19 || stg0_is_really_printing) &&
        !r->change_needed) {
        /* ★ 换料请求（change_needed）也算"不在辅助阶段"，所以把它并进上面的
         *   条件里。理由：辅助送料会占住共享电机最多 assist_ms，而打印机在
         *   ams_status=260 只留 ~2 秒窗口（260 → 318750723 只隔 2 秒），
         *   晚一步退料它就报"请拉出耗材"。换料永远优先 —— 连保持也要收掉。 */
        int cur_ch = config_get_filament_current();
        int mat = (cur_ch > 0) ? config_material_index_of(cur_ch) : -1;
        if (mat >= 0) {
            assist_want_hold = mat;
            if (r->stg_cur == 0) {
                assist_why       = "打印中";
                assist_period_ms = ASSIST_REPEAT_US / 1000;
            } else {
                assist_why       = (r->stg_cur == 8) ? "校准挤出" : "流量校准";
                /* 周期 = 单次时长 + 1 秒间隔：把网页上的时长调大，
                 * 校准期间的送料自然就变密、趋近连续。 */
                assist_period_ms = (uint32_t)config_get_assist_ms() +
                                   ASSIST_CAL_GAP_MS;
            }

            int64_t now = esp_timer_get_time();
            bool due;

            /* ★★ 2026-10-14：这一路的缓冲器闭环可用时，送与停**由微动决定**，
             *    不再按时间间隔脉冲。现场要求原话：
             *      "打印中和流量校准阶段以缓冲器微动的动作来执行送料和停止
             *       送料，不再间隔送料，只有在微动触发保护时才执行原方案的
             *       间隔辅助送料。"
             *
             *    判据用**电平 + 滞回**（buffer_need_feed），不是"序号变化"：
             *      开始送料微动触发（缓冲器被抽空）→ 送，一直送到
             *      停止送料微动触发（缓冲器满）    → 停。
             *    这才是缓冲器该有的工作方式 —— 挤出机吃一点、我们补一点，
             *    而不是每 2 秒 / 5 秒盲目顶一段料进去。
             *
             *    ⚠️ "停"还有第二层保险：脉冲本身由 buffer_feed_pulse()
             *       每 500ms 复查一次停止微动，触发即收手。 */
            if (buffer_closed_loop(mat)) {
                assist_is_buffer = true;
                assist_why       = "缓冲器微动";
                assist_period_ms = (uint32_t)config_get_assist_ms() +
                                   AMS_BUFFER_PULSE_GAP_MS;
                bool need = buffer_need_feed(mat);
                bool gap  = (s_buf_last_pulse_us[mat] == 0) ||
                            ((now - s_buf_last_pulse_us[mat]) >=
                             (int64_t)AMS_BUFFER_PULSE_GAP_MS * 1000);
                due = need && gap;
            } else {
                due = (s_last_assist_time_us == 0) ||
                      ((now - s_last_assist_time_us) >=
                       (int64_t)assist_period_ms * 1000);

                /* 微动保护生效中 → 提示一次（key 不变就不再打），
                 * 否则用户会觉得"我明明装了缓冲器，怎么还在定时送料"。 */
                if (buffer_hw_ok(mat) && s_buf_fallback[mat] &&
                    log_changed(LOGSLOT_BUF, 0u, "buf-fallback") >= 0) {
                    ams_log("料盘位%d 的缓冲器微动无反馈（保护中）→ "
                            "本阶段按时间间隔辅助送料", mat + 1);
                }
            }

            /* ★ 连续保持的**散热阀**（ASSIST_HOLD_MAX_MS / ASSIST_HOLD_COOL_MS，
             *   理由见上面那两个宏的注释）：到点断开散热，窗口内不打脉冲 ——
             *   离合这时已经断开了，再打脉冲等于又吸一次，正好把散热窗口废掉。
             *   窗口结束、且仍在辅助阶段 → 下一拍自动吸回（clutch_engage 幂等）。
             *   校准阶段（stg=8）总共只有 3 分 15 秒，永远碰不到这个阀，
             *   所以"校准期间全程吸着"这个核心诉求原样保留。 */
            if (s_assist_hold_mat >= 0 && s_assist_hold_start_us > 0 &&
                (now - s_assist_hold_start_us) >=
                    (int64_t)ASSIST_HOLD_MAX_MS * 1000) {
                assist_hold_release();
                s_assist_hold_cool_until_us =
                    now + (int64_t)ASSIST_HOLD_COOL_MS * 1000;
                /* ★ 这里**故意不打日志**（现场要求 2026-09-24：「像警告电磁
                 *   吸合时间长这类的就不需要了」）。
                 *   "到点断开散热"是**设计内的正常动作**，不是故障；而它在
                 *   长打印里每 5 分钟就要来一次，一条 [警告] 刷一晚就把真正
                 *   要看的换料日志全冲走了。
                 *   想确认这个机制在工作，看 /status 的 assist_hold_ch
                 *   （-1 = 当前没保持）就够了。 */
            }

            if (now < s_assist_hold_cool_until_us) {
                due = false;
            }

            if (due) {
                assist_do  = true;
                assist_mat = mat;
                s_last_assist_time_us = now;
                if (assist_is_buffer) {
                    /* 缓冲模式不用 assist_period_ms 节流，单独记一个"上次
                     * 补料时刻"，两次脉冲之间至少隔 AMS_BUFFER_PULSE_GAP_MS，
                     * 给电机和驱动留口气。 */
                    s_buf_last_pulse_us[mat] = now;
                }
            }
        }
    }

    /* ---- 不处在辅助送料阶段 → 电磁断开 ----
     * 就是现场要的"其他阶段电磁断开"。放在这里（而不是等到下面那一大堆
     * 换料逻辑之后）是为了让"离开阶段"这件事**立刻**生效：从校准阶段切到
     * 别的阶段的那一拍，离合就断开了。 */
    if (assist_want_hold < 0) {
        assist_hold_release();
    }

    if (assist_do) {
        uint8_t  assist_pct = config_get_assist_speed_pct();
        uint16_t assist_ms  = config_get_assist_ms();
        bool     hold       = (config_get_assist_hold() != 0);

        /* ★ 辅助送料必须**占住状态机**（AMS_STATE_ASSIST）。
         *   以前这里不设状态，后果三条，都是现场踩出来的：
         *     1. 网页状态永远是"空闲"、/status 的 busy 也是 false —— 用户没法
         *        从任何界面看出"它到底动没动"，只能靠听电机声，于是就有了
         *        "日志显示在辅助送料、可电机没动作"这种没法证伪的反馈；
         *     2. ams_is_busy() 返回 false → 别的动作（手动点动）
         *        以为总线空闲，能在辅助送料正跑着的时候抢走共享电机；
         *     3. 状态灯不亮（led_update 靠 s_state != IDLE 判"动作中"）。
         *   必须在**调用驱动之前**设，进入临界区前状态就已经是一致的。 */
        ams_state_t prev_state = ams_get_state();
        set_state(AMS_STATE_ASSIST, assist_mat);

        /* ★ 一条日志把"为什么送 / 哪一路 / 占空比 / 时长 / 周期"都带上。
         *   真机教训：60% 这种低占空比在这个机构上**带不动电机**
         *   （20kHz 堵转几乎没有可听噪声），所以实际占空比必须印出来；
         *   周期也印出来，否则现场没法判断"到底会不会再来一次"。
         *
         * ★ 降噪（现场要求 2026-09-24 "只在状态变化时显示"）：同一条**只打
         *   一次**，参数或阶段变了才打新的（见 log_changed）。
         *   校准阶段每 2 秒、打印中每 5 秒各一次，原来一小时能刷上百行完全
         *   一样的内容，把阶段变化 / 退料 / 换料那几条真正有用的冲没了。
         *   10 分钟给一次"仍在继续"的慢心跳，免得多小时的打印里看着像卡死。
         *
         * ★ 行必须短：AMS_LOG_LINE_MAX 之外会被静默截断。所以这里用
         *   "通道2=橙色"这种紧凑写法，不写整句。 */
        char key[112];
        snprintf(key, sizeof(key), "assist|%s|%d|%u|%u|%u|%d",
                 assist_why, config_get_filament_current(),
                 (unsigned)assist_pct, (unsigned)assist_ms,
                 (unsigned)assist_period_ms, hold ? 1 : 0);
        int ast = log_changed(LOGSLOT_ASSIST, 600000u, key);
        if (ast == 0) {
            char acol[32];
            color_text_of_channel(config_get_filament_current(),
                                  acol, sizeof(acol));
            if (assist_is_buffer) {
                ams_log("缓冲补料：通道%d=%s @%u%% × %ums"
                        "（送/停由缓冲器微动决定，不再定时）",
                        config_get_filament_current(), acol,
                        (unsigned)assist_pct, (unsigned)assist_ms);
            } else {
                ams_log("辅助送料 %s：通道%d=%s @%u%% × %ums 每%ums%s",
                        assist_why, config_get_filament_current(), acol,
                        (unsigned)assist_pct, (unsigned)assist_ms,
                        (unsigned)assist_period_ms,
                        hold ? " 保持吸合" : "");
            }
        } else if (ast > 0) {
            ams_log("辅助送料 %s：通道%d 仍在继续（状态无变化，"
                    "每 10 分钟提示一次）",
                    assist_why, config_get_filament_current());
        }

        /* ★★ 这里是修"日志在打、电机却没动"的关键 ★★
         *
         *   以前这一行是 `(void)drive_channel_speed(...)` —— 返回值直接扔掉，
         *   而"辅助送料…"那句日志在上面**已经打出去了**。
         *   于是离合吸合失败时（残留吸合冲突 ESP_ERR_CLUTCH_CONFLICT，
         *   或 3 秒拿不到锁 ESP_ERR_TIMEOUT），函数只是 record_error 再
         *   return false，现场看到的**仍然**是"辅助送料（校准挤出）…"，
         *   完全看不出"这一次根本没有电机被驱动过"。
         *
         *   现在把返回值接住，并读硬件回执：离合到底吸合没有、写进 LEDC 的
         *   duty 原值是多少、电机真通了多久。**成功也打出来** —— 因为
         *   "辅助送料 60%（duty 153/255）"和"手动点动 100%（duty 255/255）"
         *   并排看，才能一眼判断是不是这个占空比带不动本机构的电机。 */
        bool ok;
        bool buf_stopped = false;
        if (assist_is_buffer) {
            /* ★ 缓冲补料（2026-10-14）：离合保持吸合、只脉冲电机，
             *   但**每 500ms 复查一次「停止送料微动」**，触发即收手 ——
             *   这才是现场要的"以缓冲器微动的动作来执行停止送料"。
             *
             * ⚠️ 这里**故意不看**网页上的「辅助送料保持吸合」开关：
             *   缓冲模式下一次补料只有一两秒、而整场打印几小时，如果每次
             *   都"吸合-断开"一整套，机械照样在反复咬合（就是当初提出
             *   "全程保持吸合"的那个问题）。所以缓冲模式恒走保持式脉冲。 */
            ok = buffer_feed_pulse(assist_mat, assist_pct, assist_ms,
                                   &buf_stopped);
            if (ok && buf_stopped &&
                log_changed(LOGSLOT_BUF, 0u, "buf-full") >= 0) {
                ams_log("缓冲器已满（停止送料微动触发）→ 停止送料");
            }
        } else if (hold) {
            /* 保持模式：离合吸着不放，**只**脉冲电机（现场要的就是这个） */
            ok = assist_hold_pulse(assist_mat, assist_pct, assist_ms);
        } else {
            /* 老行为：每次一整套"吸合 → 转 → 停 → 断开"。
             * 先 release 一次，防的是"刚才还在保持、网页上刚把保持关掉"
             * 这个切换瞬间可能残留的吸合。没在保持时它是空操作。 */
            assist_hold_release();
            ok = drive_channel_speed(assist_mat, 1, assist_pct, assist_ms,
                                     false, NULL);
        }
        const drive_receipt_t *rc = ams_last_drive_receipt();

        if (!ok || !rc->clutch_ok) {
            ams_log_err("辅助送料未执行 通道%d 离合没吸合（见上一条错误）"
                        "→ 本次没有驱动电机，与占空比无关",
                        config_get_filament_current());
        } else {
            /* ★ 回执行同样"状态变化才打"（现场要求见 log_changed 的说明）：
             *   占空比 / 时长不变的话，每脉冲一次都印一遍完全相同的数字
             *   没有任何信息量 —— 原来校准阶段这一条就刷了 90 多遍。
             *   ttl=0 = 纯变化触发；一次换色结束会清槽（do_exchange 开头），
             *   所以下一轮换色照样能看见它，不会"第一次之后就再也不打"。 */
            char rkey[96];
            snprintf(rkey, sizeof(rkey), "assistrc|%d|%u|%u|%lu|%d|%d",
                     rc->skipped ? 1 : 0, (unsigned)rc->speed_pct,
                     (unsigned)rc->duty_raw, (unsigned long)rc->ran_ms,
                     (int)rc->level_in1, (int)rc->level_in2);
            if (log_changed(LOGSLOT_ASSIST_RC, 0, rkey) >= 0) {
                if (rc->skipped) {
                    ams_log_warn("辅助送料被跳过：电机已在同方向同速度运行");
                } else {
                    ams_log("辅助送料回执 通道%d duty=%u/255(%u%%) %lums%s"
                            " 回读 IN1=%u/255(%d) IN2=%u/255(%d)",
                            config_get_filament_current(),
                            (unsigned)rc->duty_raw, (unsigned)rc->speed_pct,
                            (unsigned long)rc->ran_ms,
                            hold ? " 保持" : "",
                            (unsigned)rc->duty_in1, (int)rc->level_in1,
                            (unsigned)rc->duty_in2, (int)rc->level_in2);
                }
            }
        }

        /* 状态还回去（通常是"空闲"）—— 别让"辅助送料中"卡住状态机。
         * 回到空闲时料盘位要跟着清成 -1，否则网页会显示"空闲（料盘位2）" */
        set_state(prev_state, (prev_state == AMS_STATE_IDLE) ? -1 : assist_mat);
    }

    /* ---- 载入打印材料：打印机在等料，我们就主动往里送 ----
     *
     * ★ 现场要求（2026-09-23）原话："接收到打印文件后，如果当前挤出机为
     *   无耗材，他好像有一个进料的状态，在进料时送入目标通道的耗材。"
     *
     * 打印机停在 stg_cur=24（载入打印材料）时就是在**等料**，分两种情况：
     *   · 它同时发了换料请求（change_needed）→ 走下面的换料流程，这里不碰；
     *   · 它只是在等料（打印刚开始、挤出机空着）→ 我们主动推。
     *
     * 目标通道优先取报文里的热床信道（切片的 M140 S{next_extruder+1} 会把
     * 目标通道送过来），取不到再退回"当前通道"。
     *
     * ★ 判据为什么是"无料"而不是"只要在这个阶段就推"：料已经在挤出机里
     *   还硬推，会把料拱弯（现场见过的"空推"）。所以只在有**无料证据**时推：
     *     · hint == 0  → 打印机明确报过无料；
     *     · hint < 0   → 打印机从没报过这个字段，同时我们也没见过"到位"事件
     *                    （extruder_inplace_triggered() 为假），按"可能空着"处理。
     *
     * ★ 每次只推 2 秒（一拍一段），要不要继续由打印机的下一拍报文决定 ——
     *   "挤出机一到位就停"因此是天然成立的；也不会长时间占住共享电机
     *   （打印机随时可能发来换料请求，换料永远优先）。
     * ★ 另有 45 秒看门狗：真等不到就停手告警。现场出现过一直推、把外挂
     *   料盘整卷喂空、最后只能手动在打印机上点「耗材已加载」的事故。 */
    if (r->stg_cur == 24 && !r->change_needed && !ams_is_busy() &&
        config_get_assist_enabled() &&
        (r->extruder_inplace_hint == 0 ||
         (r->extruder_inplace_hint < 0 && !extruder_inplace_triggered()))) {
        int tgt = (r->bed_channel > 0) ? r->bed_channel
                                       : config_get_filament_current();
        int mat = (tgt > 0) ? config_material_index_of(tgt) : -1;
        if (mat >= 0) {
            int64_t now = esp_timer_get_time();
            if (s_stage24_push_start_us == 0) {
                s_stage24_push_start_us = now;
            }
            uint32_t pushed_s =
                (uint32_t)((now - s_stage24_push_start_us) / 1000000);
            if (pushed_s < AMS_STAGE24_PUSH_MAX_S) {
                char key[64];
                snprintf(key, sizeof(key), "stage24push|%d", mat);
                /* 只在开始推时打一条 + 每 10 分钟慢心跳：这个动作最长持续
                 * 45 秒、每 2 秒一轮，逐轮记会把日志刷满
                 * （现场要求见 log_changed 的说明）。 */
                if (log_changed(LOGSLOT_STAGE24, 600000u, key) >= 0) {
                    char tcol[32];
                    color_text_of_channel(tgt, tcol, sizeof(tcol));
                    ams_log("载入打印材料 通道%d=%s 无料→推料（已送 %us/%us）",
                            tgt, tcol, (unsigned)pushed_s,
                            (unsigned)AMS_STAGE24_PUSH_MAX_S);
                }
                (void)drive_channel_speed(mat, 1, AMS_STAGE24_PUSH_PCT,
                                          AMS_LOAD_SEGMENT_MS, false, NULL);
            } else if (pushed_s < AMS_STAGE24_PUSH_MAX_S + 1) {
                ams_log_warn("载入打印材料 连续送料 %us 仍没等到挤出机到位，"
                             "先停下（料路可能卡住）",
                             (unsigned)AMS_STAGE24_PUSH_MAX_S);
            }
        }
    } else if (r->stg_cur != 24 || r->change_needed) {
        s_stage24_push_start_us = 0;    /* 离开这个阶段 → 看门狗归零 */
    }

    /* ---- 换料请求 ---- */
    if (!r->change_needed) {
        /* 打印机不再请求换料了 → 重置去重计数 */
        s_change_active = false;
        s_exchange_attempts = 0;
        return;
    }

    /* 打印机发来了换料请求，说明触发条件（mc_percent==101 或 ams_stage==1）成立
     * 默认：报文里的通道号（0 起，+1 转成 1 起） */
    int printer_ch = r->filament_next + 1;

    /* ======================================================================
     * ★★ 先挡重复上报，**再**打日志 ★★
     * ======================================================================
     * 现场要求（2026-09-24）原话："连续的报文只显示一条，只有在状态变化时
     * 显示。" 而打印机在等着换料的每一秒都会重发**同一条**请求。
     *
     * 原来这三段日志（收到打印机指令 / 颜色匹配 / 报文未携带颜色）写在两道
     * 守卫**之前**，于是同一个请求被原样打了 N 遍 —— 现场贴的日志里
     * 73:01.989 和 73:03.043 就是同一个请求的两份复印件，中间只差一秒。
     *
     * 现在把两道守卫提到最前面：
     *   ① s_change_active —— 这个请求已经在处理了；
     *   ② 触发指纹去重   —— 换料完成后打印机还会用旧状态继续上报，见下面注释。
     * 只有**真的要走后续流程**的那一拍才会往下走，日志自然就一条。
     */
    if (s_change_active) {
        return;
    }

    /* ---- 触发指纹去重（见 s_change_fp 的说明）----
     * 换料完成后打印机还会用旧状态继续上报 change_needed=true，
     * 指纹没变就当它是残留上报，直接忽略 —— 否则同一个颜色会被换两次。
     * 指纹变了（切片真的要求换到别的通道）立刻放行，连续换色不受影响。
     * ⚠️ 这里用**原始通道号**算指纹（颜色匹配还没做），必须和下面
     *    s_pending_fp = fp 处用的是同一个口径，否则两边对不上。 */
    int fp = 0;
    if (r->bed_channel > 0) {
        fp = 1000 + r->bed_channel;
    } else if (r->mc_percent == 101) {
        fp = 2000 + printer_ch;
    } else if (r->ams_stage == 1) {
        fp = 3000 + printer_ch;
    }
    if (fp != 0 && fp == s_change_fp &&
        (esp_timer_get_time() - s_change_fp_us) <
            (int64_t)AMS_EXCHANGE_RESIDUAL_MS * 1000) {
        return;
    }

    /* ===================== 到这里才是一条**新的**换料请求 ===================== */

    /* 打印触发原因（让用户知道是哪条通路触发的），并带上这个通道的颜色 —— 
     * 现场要求："通道号就有颜色，把这个颜色在日志中显示出来"。 */
    {
        char ch_buf[64];
        ams_channel_text(r->filament_next + 1, ch_buf, sizeof(ch_buf));
        if (r->bed_channel > 0) {
            ams_log("收到指令：热床信道 → %s（切片写的 M140 S{next+1}）", ch_buf);
        } else if (r->mc_percent == 101) {
            ams_log("收到指令：M73 P101 换料请求 → %s", ch_buf);
        } else if (r->ams_stage == 1) {
            ams_log("收到指令：M400 U1 等待换料 → %s", ch_buf);
        } else {
            ams_log("收到指令：换料请求 → %s（触发原因未知）", ch_buf);
        }
    }

    /* ★ 自动颜色匹配：如果报文带目标颜色且本机配置了颜色，优先用匹配结果 */
    if (r->target_color >= 0) {
        ams_log("  带目标颜色 0x%.6X，尝试自动匹配…",
                (unsigned int)(r->target_color & 0xFFFFFF));
        int matched = auto_match_channel(r->target_color);
        if (matched > 0) {
            printer_ch = matched;
            ams_log("  匹配结果：通道 %d（已覆盖原始通道 %d）",
                    printer_ch, r->filament_next + 1);
        } else {
            ams_log("  自动匹配失败，退回原始通道 %d", printer_ch);
        }
    } else {
        /* ★ 现场原话："实际是通道号就有颜色，把这个颜色在日志中显示出来。"
         *   报文确实不带颜色，但本机在网页上配过 —— 直接把配的那份印出来，
         *   看日志的人不用再回头翻网页。 */
        char col[40];
        color_text_of_channel(printer_ch, col, sizeof(col));
        ams_log("  报文未携带颜色 → 按通道号换料：通道%d=%s", printer_ch, col);
    }

    if (printer_ch < 1 || printer_ch > BOARD_CHANNEL_COUNT) {
        if (!s_change_active) {
            ams_log("  动作：通道 %d 超出本机 1~%d，拒绝换料",
                    printer_ch, BOARD_CHANNEL_COUNT);
            record_error("打印机请求通道 %d，超出本机支持的 1~%d",
                         printer_ch, BOARD_CHANNEL_COUNT);
            s_change_active = true;
        }
        return;
    }

    /* ======================================================================
     * ★ 打印机侧预处理 —— 必须在这里做，不能留到 do_exchange
     * ======================================================================
     * 时序是这次修复里最关键的一点：
     *
     *   切片宏里 M140 S{next_extruder+1} 写在很前面，M400 U1 写在后面 ——
     *   A1 的宏是第 6 行 vs 第 25 行，仓库里那份真实切片输出（main/切片G.txt）
     *   里甚至隔了 28 行。也就是说**打印机还在 RUNNING 的时候，热床信令
     *   就已经到了**。
     *
     *   Top-AMS 正是趁这个窗口把「恢复床温 + 快速退料」发出去的。
     *
     *   （2026-09-22 补注：旧注释在这里写"必须趁 RUNNING，停机再发 G-code 流
     *   就不动了"—— 真机实测**不成立**，PAUSE 期间发的 `M190` / `M620/T255`
     *   都照常执行。窗口的真正意义是**赶在我们自己收线之前**，不是状态门禁。）
     *
     *   所以这里一拿到通道号就发两条：
     *     ① M190 S<bed>             把借走的热床温度还回去
     *     ② M109 + M620/T255/M621   让打印机自己切刀、把热端里的料吐到缓冲
     *   然后我们自己的电机再去收线（在 do_exchange 里，那时打印机已经暂停）。
     * ====================================================================== */

    /* ---- ① 归还热床温度 ----
     * 借了就必须还：不还的话热床会真的按 M140 的设定降到 1~4℃。
     * 发完 M190 之后打印机的 bed_target 立刻变回真实值，所以这段只会走一次。 */
    if (r->bed_channel > 0) {
        if (s_bed_target_max > 0.0f) {
            char g[64];   /* 留足余量：GCC 对 %.0f 的最大长度算得很宽 */
            /* ★ 结尾的 \n 不能省 —— gcode_line 的 param 必须以换行结尾，
             *   少了它这条 M190 会被打印机静默丢弃，热床就一直留在 1~4℃。
             *
             * ★ 别用 `snprintf(g, sizeof(g), "%s\n", msg)` 去拼：GCC 会算出
             *   "48 字节的 msg + \n" 可能放不下 48 字节的 g，于是
             *   -Werror=format-truncation 直接编译失败（ESP-IDF 默认开
             *   -Werror=all）。这里一次成型，日志再单独格一遍（不带 \n）。 */
            snprintf(g, sizeof(g), "M190 S%.0f\n", (double)s_bed_target_max);
            if (bambu_mqtt_send_gcode(g) >= 0) {
                ams_log("  已请求恢复热床温度：M190 S%.0f"
                        "（不恢复的话它会真的降到 1~4℃）",
                        (double)s_bed_target_max);
            } else {
                ams_log_warn("  恢复热床温度失败（MQTT 未连接？）—— "
                             "打印完请检查床温，可能已经掉了");
            }
        } else {
            ams_log_warn("  没有热床温度的历史记录，无法自动恢复 —— "
                         "如果发现热床凉了，请手动设回去");
        }
    }

    /* ---- ② 同料判定：目标通道 == 当前通道 → 不切刀、不进退料，直接放行 ----
     *
     * ★ 现场要求（2026-09-23）原话："首次目标通道与当前通道一致，不要发送
     *   切刀任务，直接按正常流程走，如果对比不一样再换料。"
     *
     * ★ 为什么必须在这里判、而不能等到 do_exchange 里再判：
     *   切刀指令（M109 + M620/T255/M621）就是在下面 ③ 那一小段里发出去的。
     *   一旦发出去，料**已经被切断、热端已经是空的**，那时再判"不用换"
     *   就成了"切了没人装回去" —— do_exchange 里那段长注释记的正是
     *   2026-09-22 的那次事故。判定必须赶在切刀**之前**。
     *
     * 这个做法与 Top-AMS 一致（main.cpp:298）：
     *     同一耗材,无需换料 → 恢复床温 → 延时 1 秒 → print_resume
     *
     * ⚠️ 只在"挤出机里有料"（hint != 0）时才敢跳过。挤出机空着时不能跳 ——
     *    那种情况下跳过就没人送料了，交给下面的正常流程
     *    （它会跳过退料、只做进料）。 */
    {
        int  cur_ch  = config_get_filament_current();
        bool same    = (cur_ch > 0 && cur_ch == printer_ch);
        bool has_fil = (r->extruder_inplace_hint != 0);
        if (same && has_fil) {
            char scol[32];
            color_text_of_channel(printer_ch, scol, sizeof(scol));
            ams_log("  同一耗材：通道%d=%s 就是当前通道且有料 → 不发切刀、不动",
                    printer_ch, scol);
            /* 归还热床温度在上面 ① 里已经发过了（bed_channel > 0 时发 M190） */
            s_change_active = true;
            s_pending_fp    = fp;
            /* 等打印机把暂停动作做完再 resume（Top-AMS 也是等 1 秒），
             * 太快发它会跟打印机自己的暂停流程抢时序 */
            vTaskDelay(pdMS_TO_TICKS(1000));
            bambu_mqtt_send_resume();
            ams_log("  已发送 resume，打印机继续走后续冲刷 / 校准");
            return;
        }
    }

    /* ---- ③ 快速退料：先让打印机动手 ----
     * 挤出机已经明确上报"无料"（hint == 0）就跳过 —— 没料可退，
     * 硬发 M620 只会白换一个 HMS 报错。hint == -1 表示还没收到过这个
     * 字段，按"有料"处理（宁可多发一次）。 */
    {
        int cur_ch  = config_get_filament_current();
        int mat_cur = (cur_ch > 0) ? config_material_index_of(cur_ch) : -1;
        if (r->extruder_inplace_hint != 0) {
            exchange_prime_extrude(mat_cur);
        } else {
            ams_log("  挤出机已报无料，跳过打印机的快速退料");
        }
    }

    s_change_active = true;
    s_pending_fp = fp;

    ams_log("  动作：开始执行换料（目标通道 %d）", printer_ch);
    ams_cmd_t cmd = { .id = AMS_CMD_EXCHANGE, .channel = printer_ch };
    if (!ams_post_cmd(&cmd)) {
        ams_log("  换料命令排队失败（设备正忙），等打印机的下一次请求");
        s_change_active = false;
        s_pending_fp = 0;
        /* 切刀指令已经发出去了但换料没排上 —— 别把"待握手"的标记留着，
         * 否则下一次手动/网页触发的换料会在 do_exchange 里白等 3 分钟。 */
        s_prime_sent = false;
    }
}

/* ==========================================================================
 * 八、微动触发的自动动作
 * ========================================================================== */

/**
 * 检查「开始送料微动」。
 *
 * 用"序号变了"判断有没有新的触发，而不是看电平 —— 电平是持续状态，
 * 用它在每次循环里都会判定成"触发了"，会无限重复动作。
 *
 * ★ 2026-10-23：「自吸微动」和整套自吸上料流程已按现场要求删除，
 *   这里只剩「开始送料微动 → 缓冲送料」这一条通路。
 */
static void poll_trigger_switches(void)
{
    /* ★ 缓冲器闭环的"微动活动/保护"计时**每拍都要跑**，所以放在
     *   ams_is_busy() 之前 —— 否则一次长动作（换料几十秒）会把保护计时
     *   冻住，保护就永远不触发（或者一结束就误触发）。 */
    buffer_track_activity();

    if (ams_is_busy()) {
        /* 有动作在跑时不响应新触发。但仍然要同步序号，
         * 否则动作结束后会"补做"一次刚刚错过的触发。 */
        for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
            s_last_start_seq[i] = sensor_trigger_seq(i, SENSOR_START);
        }
        return;
    }

    /* ★ 两道门禁（2026-10-14 现场要求，原话）：
     *     "在有打印任务时触发才送料，在没有打印任务时不动作；在打印时只有
     *      正确通道的触发才按逻辑动作，其他通道不动作。"
     *   ① job     —— 有打印任务（准备 / 打印中 / 暂停）
     *   ② cur_mat —— 这一路的微动属于"打印机当前在用的那一路"
     */
    bool job     = ams_print_job_active();
    int  cur_mat = current_material_index();

    /* ★ 打印中 / 校准阶段，送料由 handle_report 的辅助送料一节**独占**：
     *     · 有缓冲器的通道 → 缓冲闭环（微动决定送/停）
     *     · 没有的通道     → 原来的按时间间隔脉冲
     *   这一拍**绝不能**再排一条 AMS_CMD_LOAD 进去 —— 那是一次"全速推到
     *   挤出机、最长 20 秒"的动作，会两边抢共享电机、还会把缓冲器顶爆。
     *
     *   条件必须和 handle_report 那边的门禁**完全一致**
     *   （config_get_assist_enabled() && feeding_stage && !change_needed）：
     *   不一致就会出现"两边都以为对方在管 → 谁也不管"或者"两个都在管"。
     *   辅助送料**总开关关掉**时不接管 —— 那种情况下保留"按微动手动送一次"
     *   这条手动通路，方便现场排查。 */
    bool stage_owns_feed = config_get_assist_enabled() && ams_feeding_stage();

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        uint32_t st = sensor_trigger_seq(i, SENSOR_START);

        bool start_new = (st != s_last_start_seq[i]);
        s_last_start_seq[i] = st;

        if (!config_sensor_enabled(i) || !start_new) {
            continue;
        }

        /* ---- 缓冲送料的三道门 ---- */
        if (!job) {
            /* 没有打印任务 → 不动作：开机时台面震动、放料时手碰到微动，
             * 都不该让电机自己转起来。 */
            if (log_changed(LOGSLOT_TRIG, 5000u, "nojob") >= 0) {
                ams_log("料盘位%d 开始送料微动触发，但当前**没有打印任务** → "
                        "不动作（有打印任务时才按微动送料）", i + 1);
            }
            continue;
        }
        if (i != cur_mat) {
            if (log_changed(LOGSLOT_TRIG, 5000u, "wrongch") >= 0) {
                ams_log("料盘位%d 开始送料微动触发，但它不是打印机当前通道"
                        "（当前 = 料盘位%d）→ 不动作",
                        i + 1, (cur_mat >= 0) ? cur_mat + 1 : 0);
            }
            continue;
        }
        if (stage_owns_feed) {
            /* 打印/校准阶段的送料由辅助送料那套接管 —— 只提示一次
             * （key 不变就不再打），否则每按一下都刷一行。 */
            if (log_changed(LOGSLOT_BUF, 0u, "stage-owns") >= 0) {
                ams_log("料盘位%d 开始送料微动触发 → 本阶段由**%s**决定送/停"
                        "（不是一次性送料）", i + 1,
                        buffer_closed_loop(cur_mat) ? "缓冲器闭环"
                                                    : "按时间间隔辅助送料");
            }
            continue;
        }

        ams_log("料盘位%d 开始送料微动触发 → 开始送料（停止微动触发即停）",
                i + 1);
        ams_cmd_t cmd = { .id = AMS_CMD_LOAD, .channel = i };
        ams_post_cmd(&cmd);
        return;
    }
}

/* ==========================================================================
 * 九、命令执行
 * ========================================================================== */

/**
 * 辅助送料自检：立刻按网页配置的占空比 / 时长**手动**驱动一次辅助送料。
 *
 * 和 do_jog（手动点动）的区别在"用哪个参数"：
 *   点动    → config_get_jog_speed_pct()（网页「手动点动」里那个 PWM 框）
 *   本自检  → config_get_assist_speed_pct()（网页「辅助送料」里的 PWM 速度）
 * 两个按钮合起来就是一次**受控对照实验**：
 *   点动转、自检不转 → 两边占空比不一样，把低的那个往上调
 *   两个都不转       → 硬件链路问题（H 桥使能脚 / 离合齿轮 / 接线）
 * 这条实验是"辅助送料没作用"这个现场问题唯一能一次说清的做法 ——
 * 因为辅助送料正常是由打印机阶段驱动的，等一轮打印要十几分钟。
 *
 * ⚠️ 自检**故意不走"保持吸合"**：它是一次性动作，做完就断开，
 *   免得用户以为按一下之后离合会一直吸着。
 */
static void do_assist_test(int material_index)
{
    if (material_index < 0) {
        int cur = config_get_filament_current();
        material_index = (cur > 0) ? config_material_index_of(cur) : -1;
    }
    if (material_index < 0 || material_index >= BOARD_CHANNEL_COUNT) {
        record_error("辅助送料自检：没有可用的通道（网页上先确认当前通道）");
        return;
    }

    uint8_t  pct = config_get_assist_speed_pct();
    uint16_t ms  = config_get_assist_ms();

    set_state(AMS_STATE_ASSIST, material_index);
    ams_log("辅助送料自检：料盘位%d @%u%% × %ums（网页上配置的值）",
            material_index + 1, (unsigned)pct, (unsigned)ms);

    bool ok = drive_channel_speed(material_index, 1, pct, ms, false, NULL);
    const drive_receipt_t *rc = ams_last_drive_receipt();

    if (!ok || !rc->clutch_ok) {
        ams_log_err("辅助送料自检失败：离合没吸合上 —— 本次**没有**驱动电机，"
                    "问题在离合吸合，与占空比无关");
    } else {
        ams_log("辅助送料自检结果：%s —— 请求 duty=%u/255（%u%%）；"
                "**硬件回读** IN1=%u/255(引脚%d) IN2=%u/255(引脚%d)，"
                "通电 %ums",
                rc->skipped ? "★被幂等跳过（电机本来就在同方向同速度转）"
                            : "已驱动",
                (unsigned)rc->duty_raw, (unsigned)rc->speed_pct,
                (unsigned)rc->duty_in1, (int)rc->level_in1,
                (unsigned)rc->duty_in2, (int)rc->level_in2,
                (unsigned)rc->ran_ms);
        if (!rc->skipped && rc->duty_in1 == 0 && rc->duty_in2 == 0) {
            ams_log_err("⚠ 回读发现两路 LEDC 的原值都是 0 —— 信号**没写进硬件**，"
                        "这是软件问题，不是占空比/机械问题");
        } else if (!rc->skipped) {
            ams_log("  ★ 信号确实出去了（回读非 0）。若此时电机仍不转，"
                    "就是占空比/机械：把网页上的辅助送料速度往上调，"
                    "或用「手动点动」（全速 duty=255）对照一次");
        }
    }
    set_state(AMS_STATE_IDLE, -1);
}

static void do_jog(int material_index, int direction, int ms)
{
    if (ms <= 0) {
        ms = config_get_jog_ms();
    }
    int dir = (direction < 0) ? -1 : 1;

    /* ★ PWM 占空比现在可调（网页「手动点动」里那个框，2026-09-23 现场要求）：
     *   目的是让用户**目测出"能带动电机和离合的临界占空比"** —— 从 5% 一档
     *   档往上加，看电机什么时候能可靠转起来。原来写死全速，只能回答
     *   "通不通"，回答不了"要多大才转得动"。0 = 未设置 → 默认全速 100%，
     *   所以老配置下行为和改动前一模一样。 */
    uint8_t pct = config_get_jog_speed_pct();

    set_state(AMS_STATE_JOG, material_index);
    ams_log("手动点动：料盘位%d %s %dms @%u%%（duty %u/255，网页可调）",
            material_index + 1, dir > 0 ? "进料" : "退料", ms,
            (unsigned)pct, (unsigned)motor_pct_to_duty(pct));

    /* 点动不做微动提前停止 —— 用户在手动调试，就是要它转满设定的时长 */
    drive_channel_speed(material_index, dir, pct, (uint32_t)ms, false, NULL);

    /* ★ 点动也打执行回执（含硬件回读）。
     *   现场就是靠这个找临界值的：把速度调到 50% 按下点动，日志里能看到
     *   "duty 128/255、回读 IN1=128/255(1)" —— 信号确实出去了；再对照
     *   "电机转不转"，就能把临界点一档一档夹出来。只有百分比没有 duty，
     *   现场没法跟别的动作（辅助送料 / 送料）横向比。 */
    const drive_receipt_t *rc = ams_last_drive_receipt();
    if (!rc->clutch_ok) {
        ams_log_err("手动点动未执行：离合没吸合上 —— 本次**没有**驱动电机"
                    "（原因见上一条错误）");
    } else {
        ams_log("  点动回执：离合已吸合 —— 料盘位%d @%u%% → duty=%u/255；"
                "**硬件回读** IN1=%u/255(引脚%d) IN2=%u/255(引脚%d)，通电 %ums",
                material_index + 1, (unsigned)rc->speed_pct,
                (unsigned)rc->duty_raw,
                (unsigned)rc->duty_in1, (int)rc->level_in1,
                (unsigned)rc->duty_in2, (int)rc->level_in2,
                (unsigned)rc->ran_ms);
    }
    s_diag.jog_count++;
    set_state(AMS_STATE_IDLE, -1);
}

static void execute_cmd(const ams_cmd_t *cmd)
{
    /* ★ 任何一条明确指令都意味着"要正式动电机了" —— 先把辅助送料的
     *   **保持吸合**收掉。不这么做有两个后果：
     *     · 下一条指令的 clutch_engage() 会白等一次 CLUTCH_RELEASE_MS +
     *       CLUTCH_SETTLE_MS（60+20ms）；
     *     · 状态显示会打架（"保持吸合中"和"换料中"同时成立）。
     *   收掉之后是一次性代价：下一拍报文来了如果仍在辅助阶段，
     *   assist_hold_pulse() 会自动重新吸上（clutch_engage 幂等）。
     *   没在保持状态时这是空操作。 */
    assist_hold_release();

    switch (cmd->id) {
    case AMS_CMD_JOG:
        do_jog(cmd->channel, cmd->arg, cmd->ms);
        break;

    case AMS_CMD_ASSIST_TEST:
        do_assist_test(cmd->channel);
        break;

    case AMS_CMD_LOAD:
        /* 手动「送料」：stop_on_buf_switch = true —— 这条路是给你**调试
         * 缓冲区**用的（在网页上点一下，看料能不能推、微动能不能停住），
         * 所以保持"停止送料微动触发即停"的老行为。换料不走这里。 */
        if (do_load(cmd->channel, true)) {
            set_state(AMS_STATE_IDLE, -1);
        } else {
            set_state(AMS_STATE_ERROR, -1);
        }
        break;

    case AMS_CMD_RETRACT:
        if (do_retract(cmd->channel)) {
            set_state(AMS_STATE_IDLE, -1);
        } else {
            set_state(AMS_STATE_ERROR, -1);
        }
        break;

    case AMS_CMD_485_DRIVE:
        /* 485 运动指令直接驱动电机（非阻塞）。arg = 方向。 */
        do_485_drive(cmd->channel, cmd->arg);
        break;

    case AMS_CMD_EXCHANGE: {
        if (do_exchange(cmd->channel)) {
            s_exchange_attempts = 0;
            /* s_change_active 必须清零 —— 不清的话后续换料会被"正在处理"
             * 一直挡住。换料完成后打印机还会重复上报旧状态，那种情况由
             * 下面 s_change_fp 的指纹去重接管，不靠这个标志。 */
            s_change_active = false;
            /* ★ 把这次触发"盖章完成"：打印机在 1~3 秒内还会用旧状态
             *   继续上报 change_needed=true，指纹相同就说明是残留，忽略掉。
             *   以前这里只是清掉 s_change_active，于是下一帧立刻又排一次
             *   换料命令 —— 表现为"同一个颜色换两次"。 */
            if (s_pending_fp != 0) {
                s_change_fp = s_pending_fp;
                s_change_fp_us = esp_timer_get_time();
            }
            s_pending_fp = 0;
            /* do_exchange 内部已发 resume，这里不重复发 */
        } else {
            s_exchange_attempts++;
            if (s_exchange_attempts >= AMS_EXCHANGE_MAX_RETRY) {
                ams_log_err("换料连续失败 %d 次，AMS 已停止重试，"
                            "请检查机构后手动处理",
                            s_exchange_attempts);
                set_state(AMS_STATE_ERROR, -1);
                s_change_active = true;   /* 不再重试 */
                /* 即使失败也发 resume，别让打印机一直卡在暂停 */
                bambu_mqtt_send_resume();
            } else {
                ams_log_warn("换料失败（第 %d 次），等打印机的下一次请求再试",
                             s_exchange_attempts);
                s_change_active = false;
            }
        }
        break;
    }

    case AMS_CMD_STOP:
        ams_log("收到停止指令");
        motor_stop();                 /* ★ 485 非阻塞驱动也要停电机 */
        clutch_release_all();
        set_state(AMS_STATE_IDLE, -1);
        break;

    default:
        break;
    }
}

/* ==========================================================================
 * 十、对外接口
 * ========================================================================== */

bool ams_post_cmd(const ams_cmd_t *cmd)
{
    if (!cmd || s_cmd_queue == NULL) {
        return false;
    }
    /* 不等待：队列满说明设备正忙，直接拒绝比让网页请求挂在那里好 */
    return xQueueSend(s_cmd_queue, cmd, 0) == pdTRUE;
}

bool ams_post_jog(int material_index, int direction, int ms)
{
    ams_cmd_t cmd = { .id = AMS_CMD_JOG, .channel = material_index,
                      .arg = direction, .ms = ms };
    return ams_post_cmd(&cmd);
}

bool ams_post_assist_test(int material_index)
{
    ams_cmd_t cmd = { .id = AMS_CMD_ASSIST_TEST, .channel = material_index };
    return ams_post_cmd(&cmd);
}

bool ams_post_stop(void)
{
    ams_cmd_t cmd = { .id = AMS_CMD_STOP };
    /* 停止指令优先级最高：先把队列里排着的命令清掉，再塞进去。
     * 不能用 xQueueReset() —— 它要求调用方必须是队列的 owner 任务，
     * 这里是从 httpd 任务调过来的，跨任务 reset 是 FreeRTOS 明确
     * 禁止的行为（会破坏队列内部结构）。安全的清法是循环收空。 */
    if (s_cmd_queue) {
        ams_cmd_t drain;
        while (xQueueReceive(s_cmd_queue, &drain, 0) == pdTRUE) {
            /* 丢弃排队中的命令 */
        }
    }
    return ams_post_cmd(&cmd);
}

bool ams_post_load(int material_index)
{
    ams_cmd_t cmd = { .id = AMS_CMD_LOAD, .channel = material_index };
    return ams_post_cmd(&cmd);
}

bool ams_post_retract(int material_index)
{
    ams_cmd_t cmd = { .id = AMS_CMD_RETRACT, .channel = material_index };
    return ams_post_cmd(&cmd);
}

bool ams_post_485_drive(int material_index, int direction)
{
    ams_cmd_t cmd = { .id = AMS_CMD_485_DRIVE, .channel = material_index,
                      .arg = (direction < 0) ? -1 : 1 };
    return ams_post_cmd(&cmd);
}

bool ams_post_exchange(int printer_channel)
{
    ams_cmd_t cmd = { .id = AMS_CMD_EXCHANGE, .channel = printer_channel };
    return ams_post_cmd(&cmd);
}

void ams_get_diag(ams_diag_t *out)
{
    if (!out) {
        return;
    }
    *out = s_diag;
    if (s_last_error_us != UINT32_MAX) {
        int64_t delta = esp_timer_get_time() - (int64_t)s_last_error_us;
        out->last_error_ms = delta > 0 ? (uint32_t)(delta / 1000) : 0;
    } else {
        out->last_error_ms = UINT32_MAX;
    }
}

void ams_reset_diag(void)
{
    memset(&s_diag, 0, sizeof(s_diag));
    s_last_error_us = UINT32_MAX;
    clutch_reset_conflict_count();
    ams_log("诊断统计已清零");
}

int ams_describe_hardware_json(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return 0;
    }

    /* 先让 clutch 写主体，再补微动和业务状态 */
    int off = clutch_describe_json(buf, buflen, (int)motor_get_dir(),
                                   ams_is_busy(), clutch_get_owner());
    /* ★ 余量从 8 提到 320（原先 200 偏小）：
     *   下面要追加的不只是 limits/state/motor_speed，还多了 assist_hold_ch
     *   和一整块 drive 回执。逐段实测的最长尾巴：
     *       limits 18 + state 12 + state_text 31 + active_material 21
     *       + motor_speed 21 + assist_hold_ch 19 + drive 149 ≈ 271 字节
     *   余量取 200 时，若 clutch 主体长到 600 字节就会被截掉尾巴 ——
     *   而截断的后果不是"少显示几个字段"，是生成**不完整**的 JSON，
     *   web_server 那边 cJSON_Parse 直接失败 → 整个 hardware 块从网页消失。
     *   这里宁可提前 return（buffer 里仍是 clutch 自己那份完整 JSON）。
     *   当前 hwbuf=800、clutch 主体约 140 字节，正常远碰不到这条线。 */
    if (off <= 0 || (size_t)off >= buflen - 320) {
        return off;
    }

    /* 去掉结尾的 '}'，追加字段后再补回来 */
    if (buf[off - 1] == '}') {
        off--;
    }

    off += snprintf(buf + off, buflen - off, ",\"limits\":[");
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        /* 1 = 这一路走「微动模式」（缓冲送料开着），
         * 0 = 开关关着，走按时间 / 闭环降级模式。
         * 前端只拿它显示"有微动 / 无微动"这个标签。 */
        off += snprintf(buf + off, buflen - off, "%s%d", i ? "," : "",
                        micro_feedback_available(i) ? 1 : 0);
    }
    off += snprintf(buf + off, buflen - off,
                    "],\"state\":%d,\"state_text\":\"%s\","
                    "\"active_material\":%d,\"motor_speed\":%d,"
                    "\"assist_hold_ch\":%d,"
                    "\"drive\":{\"clutch_ok\":%s,\"skipped\":%s,\"dir\":%d,"
                    "\"speed_pct\":%u,\"duty_raw\":%u,\"ran_ms\":%u,"
                    "\"rb_in1\":%u,\"rb_in2\":%u,\"lv_in1\":%d,\"lv_in2\":%d}}",
                    (int)s_state, ams_state_text(), s_active_material,
                    motor_get_speed_pct(),
                    ams_assist_hold_channel(),
                    s_last_drive.clutch_ok ? "true" : "false",
                    s_last_drive.skipped   ? "true" : "false",
                    (int)s_last_drive.dir,
                    (unsigned)s_last_drive.speed_pct,
                    (unsigned)s_last_drive.duty_raw,
                    (unsigned)s_last_drive.ran_ms,
                    (unsigned)s_last_drive.duty_in1,
                    (unsigned)s_last_drive.duty_in2,
                    (int)s_last_drive.level_in1,
                    (int)s_last_drive.level_in2);
    return off;
}

/* ==========================================================================
 * 十一、任务
 * ========================================================================== */

static void ams_task(void *arg)
{
    (void)arg;
    ams_cmd_t cmd;
    int boot_count = 0;

    ams_log("AMS 主任务已启动");

    while (1) {
        /* 每 100 轮打印一次心跳，用于诊断是否卡住 */
        if (++boot_count % 100 == 0) {
            if (boot_count == 100) ams_log("系统运行正常");
        }

        /* ---- ① 离合体检 ---- */
        clutch_assert_single(true);

        /* ---- ② 处理打印机上报 ---- */
        if (s_report_pending) {
            bambu_report_t copy;
            bool got = false;
            if (xSemaphoreTake(s_report_lock, pdMS_TO_TICKS(20)) == pdTRUE) {
                copy = s_last_report;
                s_report_pending = false;
                got = true;
                xSemaphoreGive(s_report_lock);
            }
            if (got) {
                handle_report(&copy);
            }
        }

        /* ---- ③ 执行命令（会阻塞，没问题 —— 只有这个任务被占住）---- */
        if (xQueueReceive(s_cmd_queue, &cmd, 0) == pdTRUE) {
            execute_cmd(&cmd);
            continue;
        }

        /* ---- ④ 微动触发的自动动作 ---- */
        poll_trigger_switches();

        /* ---- ④' 485 进料到位后的双微动决策（续送 / 断料） ---- */
        poll_485_decide();

        /* ---- ⑤ 状态灯 ---- */
        led_update();

        /* ---- ⑥ 空闲时把时钟让出去 ---- */
        vTaskDelay(pdMS_TO_TICKS(AMS_TASK_POLL_MS));
    }
}

esp_err_t ams_init(void)
{
    if (s_cmd_queue == NULL) {
        s_cmd_queue = xQueueCreate(AMS_CMD_QUEUE_LEN, sizeof(ams_cmd_t));
        if (!s_cmd_queue) {
            ams_log_err("创建命令队列失败（内存不足？）");
            return ESP_FAIL;
        }
    }
    if (s_report_lock == NULL) {
        s_report_lock = xSemaphoreCreateMutex();
    }
    if (s_state_lock == NULL) {
        s_state_lock = xSemaphoreCreateMutex();
    }

    memset(&s_diag, 0, sizeof(s_diag));
    memset(&s_last_report, 0, sizeof(s_last_report));
    s_last_report.extruder_inplace_hint = -1;  /* -1 = 还没收到上报 */
    s_last_error_us = UINT32_MAX;
    s_has_report = false;
    s_report_pending = false;
    s_exchange_attempts = 0;
    s_change_active = false;
    s_change_fp = -1;          /* -1 = 还没有"已完成"的触发，不会误挡 */
    s_change_fp_us = 0;
    s_pending_fp = 0;
    s_bed_target_max = 0.0f;
    s_last_assist_time_us = 0;   /* 0 = 开机后允许立刻做第一次辅助送料 */
    s_assist_hold_mat     = -1;  /* 开机没有保持吸合 */
    s_assist_hold_pulses  = 0;
    s_assist_hold_start_us     = 0;   /* 本轮保持计时未开始 */
    s_assist_hold_cool_until_us = 0;  /* 没有待走的散热窗口 */
    s_state = AMS_STATE_IDLE;
    s_active_material = -1;

    /* 微动序号先取当前值，避免开机瞬间把"已经触发着"当成一次新触发 */
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        s_last_start_seq[i] = sensor_trigger_seq(i, SENSOR_START);
    }

    led_init();

    ams_log("AMS 控制器已初始化（MQTT 将在 WiFi 起来后再连）");
    return ESP_OK;
}

/**
 * 在 WiFi 连接完成后调用，启动 MQTT 客户端。
 * 这样避免 esp_mqtt_client_start() 在 WiFi 未就绪时阻塞。
 */
esp_err_t ams_connect_printer(void)
{
    esp_err_t err = bambu_mqtt_init(on_mqtt_report, NULL);
    if (err == ESP_OK) {
        ams_log("打印机 MQTT 客户端已就绪");
    }
    return err;
}

esp_err_t ams_start(void)
{
    if (s_task) {
        return ESP_OK;
    }
    BaseType_t ok = xTaskCreate(ams_task, "ams_task", AMS_TASK_STACK, NULL,
                                AMS_TASK_PRIO, &s_task);
    if (ok != pdPASS) {
        ams_log_err("创建 AMS 任务失败（栈 %d 字节不够？）", AMS_TASK_STACK);
        return ESP_FAIL;
    }
    return ESP_OK;
}
