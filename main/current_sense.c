/**
 * current_sense.c —— 电机电流采样（AT8236 ISEN → GPIO3）+ 堵转保护
 * ============================================================================
 *
 * 详细背景见 current_sense.h 的文件头。这里只强调三条实现上的取舍：
 *
 * 1) **为什么用 ADC_ATTEN_DB_0 而不是 DB_12**
 *    ISEN 满打满算只有 500mV（2.5A × 200mΩ）。DB_12 的量程是 3.1V，
 *    0.8mV/LSB → 折算 4mA/LSB；DB_0 量程 0.95V，0.23mV/LSB → 1.2mA/LSB。
 *    在这么小的信号上，**分辨率就是精度**，所以选小量程。
 *
 * 2) **为什么一串连采再平均**
 *    20kHz PWM 下 ISEN 是锯齿波：导通期上升、关断期衰减。单次采样取到
 *    波峰还是波谷全看运气，抖动能有 ±40%。每 10ms 连采 8 个点跨过多个
 *    PWM 周期取平均，再做一阶低通（7/8 旧 + 1/8 新），读数才稳得住。
 *
 * 3) **为什么保护动作要同时断离合**
 *    堵转时离合还吸着 = 电机被卡着、线圈还在通电，热量全堆在驱动和线圈上。
 *    所以动作必须是"停电机 + 断全部离合"，不能只停电机。
 */

#include "current_sense.h"

#include <string.h>

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs.h"

#include "adc_shared.h"
#include "board_pins.h"
#include "clutch.h"
#include "log_buffer.h"
#include "motor.h"

#if !BOARD_CURRENT_SENSE_ENABLED

/* ==========================================================================
 * 本板型没有 ISEN 采样（C3）—— 全部退化成空实现，接口照旧可用
 * ========================================================================== */
static current_cfg_t s_cfg_off = {
    .enable = 0, .r_mohm = CURRENT_R_MOHM_DEF, .mv_per_a = 0,
    .stall_ma = 0, .stall_ms = CURRENT_STALL_MS_DEF,
    .blank_ms = CURRENT_BLANK_MS_DEF,
};

esp_err_t current_sense_init(void) { return ESP_ERR_NOT_SUPPORTED; }
const current_cfg_t *current_cfg_get(void) { return &s_cfg_off; }
current_cfg_t current_cfg_set(const current_cfg_t *in)
{
    (void)in;
    return s_cfg_off;
}
current_cfg_t current_cfg_reset(void) { return s_cfg_off; }
int current_effective_mv_per_a(void) { return 0; }
int current_mv(void) { return -1; }
int current_peak_mv(void) { return -1; }
int current_ma(void) { return -1; }
int current_peak_ma(void) { return -1; }
bool current_stall_tripped(void) { return false; }
uint32_t current_stall_count(void) { return 0; }
void current_clear_fault(void) {}
void current_reset_peak(void) {}
bool current_sense_ready(void) { return false; }
int current_describe_json(char *buf, size_t buflen)
{
    if (buf == NULL || buflen == 0) {
        return 0;
    }
    return snprintf(buf, buflen,
                    "{\"enabled\":0,\"ready\":false,\"board\":false}");
}

#else /* BOARD_CURRENT_SENSE_ENABLED */

/* ==========================================================================
 * NVS
 * ========================================================================== */
#define CS_NVS_NS    "ams_cs"
#define CS_NVS_KEY   "cfg"
#define CS_MAGIC     0x43534E53u   /* "CSNS" */
#define CS_VER       1u

typedef struct {
    uint32_t       magic;
    uint16_t       ver;
    uint16_t       rsv;
    current_cfg_t  cfg;
} cs_blob_t;

/* ==========================================================================
 * 运行状态
 * ========================================================================== */
#define CS_TASK_STACK   3072
#define CS_TASK_PRIO    3
#define CS_PERIOD_MS    10       /* 10ms 一拍：既够快，又不至于把 CPU 吃满 */
#define CS_BURST        8        /* 每拍连采 8 次（跨多个 PWM 周期） */
#define CS_PEAK_WINDOW_US (1000000LL)   /* 峰值按 1 秒窗口滚动 */
#define CS_AUTO_CLEAR_MS 2000    /* 电机停稳 2 秒后自动复归故障位 */

static adc_oneshot_unit_handle_t s_adc;
static adc_cali_handle_t         s_cali;
static adc_channel_t             s_ch;
static bool                      s_ready;
static bool                      s_cali_ok;
static TaskHandle_t              s_task;

static current_cfg_t s_cfg;

static int      s_filt_mv;        /* 低通后的电压 */
static int      s_peak_mv;        /* 1 秒窗口内的峰值 */
static int64_t  s_peak_win_us;    /* 窗口起点 */
static int64_t  s_over_since_us;  /* 连续超阈值的起点，0 = 没超 */
static int64_t  s_stop_since_us;  /* 电机停稳的起点 */
static bool     s_tripped;
static uint32_t s_trip_count;

/* ==========================================================================
 * 参数夹取
 * ========================================================================== */
static uint16_t clamp_u16(int v, int lo, int hi)
{
    if (v < lo) {
        v = lo;
    }
    if (v > hi) {
        v = hi;
    }
    return (uint16_t)v;
}

static current_cfg_t cfg_sanitize(const current_cfg_t *in)
{
    current_cfg_t c;

    c.enable   = in->enable ? 1 : 0;
    c.r_mohm   = clamp_u16((int)in->r_mohm, CURRENT_R_MOHM_MIN,
                           CURRENT_R_MOHM_MAX);
    /* mv_per_a 允许 0 = "跟着采样电阻走"，否则夹到合法区间 */
    c.mv_per_a = (in->mv_per_a == 0)
                     ? 0
                     : clamp_u16((int)in->mv_per_a, CURRENT_MV_PER_A_MIN,
                                 CURRENT_MV_PER_A_MAX);
    /* 阈值允许 0 = 不保护 */
    c.stall_ma = (in->stall_ma == 0)
                     ? 0
                     : clamp_u16((int)in->stall_ma, CURRENT_STALL_MA_MIN,
                                 CURRENT_STALL_MA_MAX);
    c.stall_ms = clamp_u16((int)in->stall_ms, CURRENT_STALL_MS_MIN,
                           CURRENT_STALL_MS_MAX);
    c.blank_ms = clamp_u16((int)in->blank_ms, CURRENT_BLANK_MS_MIN,
                           CURRENT_BLANK_MS_MAX);
    return c;
}

static void cfg_defaults(current_cfg_t *c)
{
    c->enable   = 1;                 /* 采样默认开着（只是显示，不保护） */
    c->r_mohm   = BOARD_CURRENT_SENSE_R_MOHM;
    c->mv_per_a = 0;                 /* 0 = 直接用 r_mohm */
    c->stall_ma = CURRENT_STALL_MA_DEF;   /* 0 = 不保护，等现场测出来再填 */
    c->stall_ms = CURRENT_STALL_MS_DEF;
    c->blank_ms = CURRENT_BLANK_MS_DEF;
}

static void cfg_load(void)
{
    nvs_handle_t h;
    cs_blob_t b;

    if (nvs_open(CS_NVS_NS, NVS_READONLY, &h) != ESP_OK) {
        cfg_defaults(&s_cfg);
        return;
    }
    size_t len = sizeof(b);
    esp_err_t err = nvs_get_blob(h, CS_NVS_KEY, &b, &len);
    nvs_close(h);

    if (err != ESP_OK || len != sizeof(b) || b.magic != CS_MAGIC ||
        b.ver != CS_VER) {
        cfg_defaults(&s_cfg);
        return;
    }
    s_cfg = cfg_sanitize(&b.cfg);
}

static void cfg_save(void)
{
    nvs_handle_t h;
    cs_blob_t b = {
        .magic = CS_MAGIC,
        .ver   = CS_VER,
        .rsv   = 0,
        .cfg   = s_cfg,
    };

    if (nvs_open(CS_NVS_NS, NVS_READWRITE, &h) != ESP_OK) {
        ams_log_err("电流采样参数保存失败：打不开 NVS 命名空间（本次运行仍生效）");
        return;
    }
    esp_err_t err = nvs_set_blob(h, CS_NVS_KEY, &b, sizeof(b));
    if (err == ESP_OK) {
        err = nvs_commit(h);
    }
    nvs_close(h);
    if (err != ESP_OK) {
        ams_log_err("电流采样参数写 NVS 失败: %s（本次运行仍生效）",
                    esp_err_to_name(err));
    }
}

/* ==========================================================================
 * 采样
 * ========================================================================== */

/**
 * 连采 CS_BURST 次取平均电压。
 * 返回 mV；采样不可用时返回 -1。
 */
static int read_avg_mv(void)
{
    if (!s_ready || s_adc == NULL) {
        return -1;
    }

    int32_t sum_mv = 0;
    int     n      = 0;
    for (int i = 0; i < CS_BURST; i++) {
        int raw = 0;
        if (adc_oneshot_read(s_adc, s_ch, &raw) != ESP_OK) {
            continue;
        }
        int mv = 0;
        if (s_cali_ok && s_cali != NULL) {
            if (adc_cali_raw_to_voltage(s_cali, raw, &mv) != ESP_OK) {
                mv = 0;
            }
        } else {
            /* 没校准曲线就按 0.95V 满量程线性折算（DB_0） */
            mv = (raw * 950) / 4095;
        }
        sum_mv += mv;
        n++;
    }
    if (n == 0) {
        return -1;
    }
    return (int)(sum_mv / n);
}

/* ==========================================================================
 * 堵转保护动作
 * ========================================================================== */
static void stall_trip(int ma)
{
    motor_stop();
    clutch_release_all();   /* 堵转时离合必须一起断，否则热量全堆在线圈上 */

    s_tripped = true;
    s_trip_count++;
    s_over_since_us = 0;

    /* ★ 这行必须短：超出 AMS_LOG_LINE_MAX 会被静默截断 */
    ams_log_err("★ 堵转保护：%dmA≥%dmA 持续%ums → 停电机+断全部离合",
                ma, (int)s_cfg.stall_ma, (int)s_cfg.stall_ms);
}

/* ==========================================================================
 * 采样任务
 * ========================================================================== */
static void cs_task(void *arg)
{
    (void)arg;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(CS_PERIOD_MS));

        if (!s_ready) {
            continue;
        }

        int mv = read_avg_mv();
        if (mv < 0) {
            continue;
        }

        /* 一阶低通：旧值占 7/8。首次直接把读数当起点，别从 0 爬 */
        if (s_filt_mv < 0) {
            s_filt_mv = mv;
        } else {
            s_filt_mv = (s_filt_mv * 7 + mv) / 8;
        }

        /* 峰值：1 秒滚动窗口，网页上那个"峰值"才有意义 */
        int64_t now = esp_timer_get_time();
        if (s_peak_mv < 0 || now - s_peak_win_us > CS_PEAK_WINDOW_US) {
            s_peak_mv     = s_filt_mv;
            s_peak_win_us = now;
        } else if (s_filt_mv > s_peak_mv) {
            s_peak_mv = s_filt_mv;
        }

        /* ---- 堵转判定 ---- */
        bool running = (motor_get_dir() != MOTOR_DIR_STOP);

        if (s_tripped) {
            /* 故障位：等电机停稳 2 秒自动复归，方便现场接着试 */
            if (!running) {
                if (s_stop_since_us == 0) {
                    s_stop_since_us = now;
                } else if ((now - s_stop_since_us) / 1000 >= CS_AUTO_CLEAR_MS) {
                    s_tripped       = false;
                    s_stop_since_us = 0;
                    ams_log("堵转保护已自动复归（电机停稳 %d 秒）",
                            CS_AUTO_CLEAR_MS / 1000);
                }
            } else {
                s_stop_since_us = 0;
            }
            continue;
        }

        if (!running) {
            s_stop_since_us = 0;
        }

        if (!s_cfg.enable || s_cfg.stall_ma == 0 || !running) {
            s_over_since_us = 0;
            continue;
        }

        /* 起步屏蔽：直流电机起动电流是稳态的 3~8 倍，不屏蔽会次次误跳 */
        uint32_t run_ms = motor_elapsed_ms();
        if (run_ms < (uint32_t)s_cfg.blank_ms) {
            s_over_since_us = 0;
            continue;
        }

        int ma = current_ma();
        if (ma < 0) {
            s_over_since_us = 0;
            continue;
        }

        if (ma >= (int)s_cfg.stall_ma) {
            if (s_over_since_us == 0) {
                s_over_since_us = now;
            } else if ((now - s_over_since_us) / 1000 >=
                       (int64_t)s_cfg.stall_ms) {
                stall_trip(ma);
            }
        } else {
            s_over_since_us = 0;
        }
    }
}

/* ==========================================================================
 * 对外接口
 * ========================================================================== */
esp_err_t current_sense_init(void)
{
    s_filt_mv       = -1;
    s_peak_mv       = -1;
    s_over_since_us = 0;
    s_stop_since_us = 0;
    s_tripped       = false;
    s_trip_count    = 0;
    s_ready         = false;
    s_cali_ok       = false;

    cfg_load();

    /* ---- ADC 单元 / 通道 ---- */
    adc_unit_t    unit = ADC_UNIT_1;
    adc_channel_t ch   = ADC_CHANNEL_0;
    esp_err_t err = adc_oneshot_io_to_channel(BOARD_PIN_CURRENT_SENSE, &unit, &ch);
    if (err != ESP_OK) {
        ams_log_err("电流采样脚 GPIO%d 不是 ADC 引脚 —— 电流显示不可用",
                    BOARD_PIN_CURRENT_SENSE);
        return err;
    }

    /* ★ 走 adc_shared 而不是直接 new_unit。
     *
     *   现场故障（2026-10-22）：
     *       电流采样 ADC1 初始化失败: ESP_ERR_NOT_FOUND
     *       ★ 电流采样初始化失败 —— 电流不显示、堵转保护不可用
     *
     *   ESP-IDF 的 adc_oneshot_new_unit() 对同一个 unit 只能成功一次，
     *   第二次返回的就是 ESP_ERR_NOT_FOUND（内部文案 "adc1 is already in
     *   use"）。而微动模块在 sensor_init() 里已经把 ADC1 建好了。
     *   → 统一从 adc_shared 取句柄，谁先调谁负责创建。 */
    err = adc_shared_unit(unit, &s_adc);
    if (err != ESP_OK) {
        ams_log_err("电流采样 ADC%d 不可用: %s", (int)unit + 1,
                    esp_err_to_name(err));
        s_adc = NULL;
        return err;
    }

    adc_oneshot_chan_cfg_t ccfg = {
        .atten    = ADC_ATTEN_DB_0,   /* 0~950mV：ISEN 最大 500mV，量程刚好 */
        .bitwidth = ADC_BITWIDTH_12,
    };
    if (adc_oneshot_config_channel(s_adc, ch, &ccfg) != ESP_OK) {
        ams_log_err("电流采样通道配置失败（GPIO%d）", BOARD_PIN_CURRENT_SENSE);
        /* ★ 这里**不能** adc_oneshot_del_unit —— s_adc 是微动模块在用的
         *   共享句柄，删掉会连带把 8 路微动一起废掉，比本功能不可用严重得多。 */
        s_adc = NULL;
        return ESP_FAIL;
    }
    s_ch = ch;

    /* ---- 校准（有 eFuse 曲线就用，没有就线性折算） ---- */
    {
        adc_cali_curve_fitting_config_t cal = {
            .unit_id  = unit,
            .chan     = ch,
            .atten    = ADC_ATTEN_DB_0,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cal, &s_cali) == ESP_OK) {
            s_cali_ok = true;
        } else {
            s_cali     = NULL;
            s_cali_ok  = false;
        }
    }

    /* ISEN 经 200Ω 到地，内部上下拉一律关掉，否则会给读数加偏置 */
    gpio_pullup_dis((gpio_num_t)BOARD_PIN_CURRENT_SENSE);
    gpio_pulldown_dis((gpio_num_t)BOARD_PIN_CURRENT_SENSE);

    s_ready = true;

    if (xTaskCreate(cs_task, "ams_current", CS_TASK_STACK, NULL,
                    CS_TASK_PRIO, &s_task) != pdPASS) {
        ams_log_err("电流采样任务创建失败 —— 只显示不保护");
        s_task  = NULL;
        s_ready = false;
        return ESP_FAIL;
    }

    ams_log("电流采样已启动: ISEN=GPIO%d 采样电阻 %dmΩ → %dmV/A%s",
            BOARD_PIN_CURRENT_SENSE, (int)s_cfg.r_mohm,
            current_effective_mv_per_a(),
            s_cali_ok ? "（已校准）" : "（线性折算）");
    if (s_cfg.enable && s_cfg.stall_ma > 0) {
        ams_log("堵转保护: ≥%dmA 持续 %ums 动作，起步屏蔽 %ums",
                (int)s_cfg.stall_ma, (int)s_cfg.stall_ms,
                (int)s_cfg.blank_ms);
    } else {
        ams_log("堵转保护: 未启用（阈值 0）—— 网页「硬件调试」里测完再设");
    }
    return ESP_OK;
}

const current_cfg_t *current_cfg_get(void)
{
    return &s_cfg;
}

current_cfg_t current_cfg_set(const current_cfg_t *in)
{
    if (in == NULL) {
        return s_cfg;
    }
    s_cfg = cfg_sanitize(in);
    cfg_save();
    s_over_since_us = 0;   /* 改了参数就重新计时，别拿旧窗口去跳闸 */
    return s_cfg;
}

current_cfg_t current_cfg_reset(void)
{
    cfg_defaults(&s_cfg);
    cfg_save();
    s_over_since_us = 0;
    return s_cfg;
}

int current_effective_mv_per_a(void)
{
    if (s_cfg.mv_per_a != 0) {
        return (int)s_cfg.mv_per_a;
    }
    /* AT8236：V_ISEN = I × R，所以 mV/A 的数值就等于毫欧数 */
    return (int)s_cfg.r_mohm;
}

int current_mv(void)
{
    if (!s_ready || s_filt_mv < 0) {
        return -1;
    }
    return s_filt_mv;
}

int current_peak_mv(void)
{
    if (!s_ready || s_peak_mv < 0) {
        return -1;
    }
    return s_peak_mv;
}

int current_ma(void)
{
    int mv = current_mv();
    int sens = current_effective_mv_per_a();
    if (mv < 0 || sens <= 0) {
        return -1;
    }
    return (mv * 1000) / sens;
}

int current_peak_ma(void)
{
    int mv = current_peak_mv();
    int sens = current_effective_mv_per_a();
    if (mv < 0 || sens <= 0) {
        return -1;
    }
    return (mv * 1000) / sens;
}

bool current_stall_tripped(void)
{
    return s_tripped;
}

uint32_t current_stall_count(void)
{
    return s_trip_count;
}

void current_clear_fault(void)
{
    s_tripped       = false;
    s_over_since_us = 0;
    s_stop_since_us = 0;
}

void current_reset_peak(void)
{
    s_peak_mv     = s_filt_mv;
    s_peak_win_us = esp_timer_get_time();
}

bool current_sense_ready(void)
{
    return s_ready;
}

int current_describe_json(char *buf, size_t buflen)
{
    if (buf == NULL || buflen == 0) {
        return 0;
    }
    return snprintf(buf, buflen,
                    "{\"enabled\":%d,\"ready\":%s,\"board\":true,"
                    "\"mv\":%d,\"peak_mv\":%d,\"ma\":%d,\"peak_ma\":%d,"
                    "\"r_mohm\":%d,\"mv_per_a\":%d,\"stall_ma\":%d,"
                    "\"stall_ms\":%d,\"blank_ms\":%d,"
                    "\"tripped\":%s,\"trips\":%u}",
                    s_cfg.enable ? 1 : 0,
                    s_ready ? "true" : "false",
                    current_mv(), current_peak_mv(),
                    current_ma(), current_peak_ma(),
                    (int)s_cfg.r_mohm, current_effective_mv_per_a(),
                    (int)s_cfg.stall_ma, (int)s_cfg.stall_ms,
                    (int)s_cfg.blank_ms,
                    s_tripped ? "true" : "false", (unsigned)s_trip_count);
}

#endif /* BOARD_CURRENT_SENSE_ENABLED */
