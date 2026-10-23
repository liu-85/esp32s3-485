/**
 * filament_sensor.c —— 微动开关的实现（5ms 轮询去抖 + 边沿序号）
 * ============================================================================
 *
 * 数据结构很直白：每个 (通道, 类型) 一格，记四样东西
 *
 *     raw         最近一次读到的电平（0/1）
 *     stable      去抖之后的稳定电平（0/1）
 *     debounce    连续读到"与 stable 不同"的次数，达到 SENSOR_DEBOUNCE_N 就翻转
 *     trig_seq    累计触发次数
 *     rel_seq     累计松开次数
 *     last_trig_us 最近一次触发的时刻（esp_timer 微秒），用于算"多久没动了"
 *
 * 去抖定时器是 esp_timer 的周期回调，跑在 esp_timer 任务里（独立于
 * ams_controller 那个任务），所以即使主循环正忙在一次长动作里，微动状态也在
 * 被持续更新 —— 等会儿一查序号就知道期间发生过什么，不会漏事件。
 */

#include "filament_sensor.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#if BOARD_SENSOR_DUAL_ANALOG
#include "esp_adc/adc_oneshot.h"
#endif

#include "config.h"
#include "adc_shared.h"
#include "log_buffer.h"

/* ==========================================================================
 * 引脚表
 * ==========================================================================
 * 顺序必须和 sensor_kind_t 一致：[停止, 开始]，每行是 4 个料盘位。
 * 两个宏来自 board_pins.h，各自展开成 4 项的初始化列表。
 *
 * ★ 2026-10-22：S3 换成「每路 2 只微动合占 1 个 ADC 脚」之后，下面这张
 *   数字引脚表在 S3 上根本不存在（那些宏全是 -1），真正生效的是
 *   s_combo[].pin。C3 没有模拟方案，继续用这张表。 */

/* ==========================================================================
 * 状态
 * ========================================================================== */
typedef struct {
    int      pin;             /* -1 表示未配置 */
    int      raw;
    int      stable;
    int      debounce;
    uint32_t trig_seq;
    uint32_t rel_seq;
    int64_t  last_trig_us;
    int64_t  last_rel_us;
} sensor_slot_t;

static sensor_slot_t s_slot[SENSOR_KIND_COUNT][BOARD_CHANNEL_COUNT];

/* 挤出机到位信号 */
static int      s_extruder_pin = -1;
static int      s_extruder_raw;
static int      s_extruder_stable;
static int      s_extruder_debounce;
static uint32_t s_extruder_seq;
static int64_t  s_extruder_last_us;

/* 补充机满足信号（外部输入，只上报不参与流程） */
static int      s_in_ext_pin = -1;
static int      s_in_ext_level;

static esp_timer_handle_t s_timer;

/** 有效电平：BOARD_SENSOR_ACTIVE_LEVEL 默认 0（低电平触发） */
#define ACTIVE_LEVEL BOARD_SENSOR_ACTIVE_LEVEL

static inline int level_is_active(int level)
{
    return level == ACTIVE_LEVEL ? 1 : 0;
}

/* ==========================================================================
 * 单 IO 双微动（电阻编码 + ADC）—— 仅 BOARD_SENSOR_DUAL_ANALOG = 1 的板型
 * ==========================================================================
 * 硬件：每路两只微动串不同电阻后并在同一个 GPIO 上，靠**读电压**分三档。
 *
 *     开始位 微动(NO) ──4.7K──┐
 *                             ├── GPIO ── 内部上拉 45K ── 3V3
 *     停止位 微动(NC) ──10K ──┘
 *
 *   · 开始位压下           → 4.7K∥10K = 3.2K → 电压最低（≈220mV）→ 缓冲器空，该补料
 *   · 都不压（只有 10K）   → ≈600mV                                  → 中间，保持上一拍
 *   · 停止位顶开（NC 断开）→ 只剩上拉                                 → ≈3300mV → 满，停
 *
 * ★ 为什么必须 ADC，不能用 digitalRead：
 *   三档都"对地有连接"时数字口读出来全是 0 —— 分不出空/中间/满，等于没用。
 *   这也是为什么这四个脚必须是 ADC1（GPIO1~GPIO10），ADC2 在 WiFi 下不可用。
 *
 * ★ 为什么不接外部上拉：分压比只跟**并联后的电阻**和上拉比值有关，
 *   内部上拉虽然离散（31K~58K），但 R 越大电压越高的**单调性**不变，
 *   所以只要阈值落在两档中间，上拉怎么飘都不会判反（推导见 board_pins_s3.h）。
 */
#if BOARD_SENSOR_DUAL_ANALOG

/** 三档分类（数值即 s_combo[].cls） */
enum { COMBO_START = 0, COMBO_MID = 1, COMBO_STOP = 2 };

/** 滞回门限：进档用标称阈值，出档放宽 25%，免得在门槛上来回抖 */
#define COMBO_MV_START_IN  (BOARD_SENSOR_MV_START)
#define COMBO_MV_START_OUT ((BOARD_SENSOR_MV_START) * 13 / 10)
#define COMBO_MV_STOP_IN   (BOARD_SENSOR_MV_STOP)
#define COMBO_MV_STOP_OUT  ((BOARD_SENSOR_MV_STOP) * 85 / 100)

typedef struct {
    int                 pin;    /**< -1 = 本路没有微动 */
    adc_channel_t       ch;
    adc_unit_t          unit;   /**< ADC_UNIT_1 / ADC_UNIT_2（PCB 固定的那 3 路） */
    int                 mv;     /**< 最近一次采样电压（mV） */
    int                 cls;    /**< 滞回后的分类：COMBO_START/MID/STOP */
    uint16_t            fails;  /**< 连续读失败次数（只有 ADC2 会涨） */
    bool                degraded;/**< true = ADC2 在 WiFi 下读不到，本路已降级 */
} combo_t;

static combo_t s_combo[BOARD_CHANNEL_COUNT];
static adc_oneshot_unit_handle_t s_adc;    /* ADC1 */
static adc_oneshot_unit_handle_t s_adc2;   /* ADC2：PCB 上通道 6/7/8 落在这里 */
static bool s_adc_ok;
static bool s_adc2_ok;
static bool s_adc2_tried;   /* ADC2 单元只尝试建立一次 */
/** 连续失败多少次才认定"这一路降级"：5ms 一次 × 100 = 0.5 秒 */
#define COMBO_ADC2_FAIL_LIMIT 100

/** 按滞回把 mV 归到三档。cls 传上一拍的结果，返回新结果。 */
static int combo_classify(int mv, int cls)
{
    switch (cls) {
    case COMBO_START:
        if (mv > COMBO_MV_START_OUT) {
            cls = (mv > COMBO_MV_STOP_IN) ? COMBO_STOP : COMBO_MID;
        }
        break;
    case COMBO_STOP:
        if (mv < COMBO_MV_STOP_OUT) {
            cls = (mv < COMBO_MV_START_IN) ? COMBO_START : COMBO_MID;
        }
        break;
    default: /* COMBO_MID */
        if (mv < COMBO_MV_START_IN) {
            cls = COMBO_START;
        } else if (mv > COMBO_MV_STOP_IN) {
            cls = COMBO_STOP;
        }
        break;
    }
    return cls;
}

/** 采一次样并更新分类（由 5ms 定时器调用） */
static void combo_poll(int c)
{
    combo_t *cb = &s_combo[c];
    if (cb->pin < 0) {
        return;
    }

    adc_oneshot_unit_handle_t h =
        (cb->unit == ADC_UNIT_2) ? s_adc2 : s_adc;
    bool ok_unit = (cb->unit == ADC_UNIT_2) ? s_adc2_ok : s_adc_ok;
    if (!ok_unit || h == NULL) {
        return;
    }

    int raw = 0;
    esp_err_t err = adc_oneshot_read(h, cb->ch, &raw);

    /* ★★ ADC2 在 WiFi 下会稳定失败（ESP_ERR_TIMEOUT）★★
     *   这一路就不能再按"读到的旧值"继续判断了 —— 否则一个卡在
     *   "开始位"分类上的通道会**一直要求送料**，比不工作更危险。
     *   所以连续失败到阈值就把它标成 degraded，并且**把分类压回 MID**
     *   （既不说空也不说满），上层的"缺料/满料"判断因此都不会误触发。
     *   日志只打一次，不能 5ms 一条把日志刷爆。 */
    if (err != ESP_OK) {
        if (cb->unit != ADC_UNIT_2) {
            return;
        }
        if (cb->fails < UINT16_MAX) {
            cb->fails++;
        }
        if (!cb->degraded && cb->fails >= COMBO_ADC2_FAIL_LIMIT) {
            cb->degraded = true;
            cb->cls = COMBO_MID;
            ams_log_err("料盘位%d 的微动(GPIO%d) 在 ADC2 上，WiFi 开启期间读不到"
                        "（%s）→ 本路降级：微动不再参与判断。"
                        "改用 485 直连（WiFi 关闭）即可恢复",
                        c + 1, cb->pin, esp_err_to_name(err));
        }
        return;
    }
    cb->fails = 0;
    if (cb->degraded) {
        /* WiFi 关了（比如切到 485 模式）之后自己恢复，不需要重启 */
        cb->degraded = false;
        ams_log("料盘位%d 的微动(GPIO%d) 已恢复可读", c + 1, cb->pin);
    }

    /* 12 位满量程 → mV。不用校准曲线：这里只做**三档比较**，不做测量，
     * 线性近似带来的几个百分点误差远小于两档之间的间距（220/600/3300）。 */
    cb->mv  = (raw * BOARD_SENSOR_VDD_MV) / 4095;
    cb->cls = combo_classify(cb->mv, cb->cls);
}

#endif /* BOARD_SENSOR_DUAL_ANALOG */

/* ==========================================================================
 * 去抖核心
 * ========================================================================== */

/**
 * 处理一格：给定"当前是否触发" → 去抖 → 翻转时更新序号。
 *
 * ★ 拆成"给 active"而不是"自己读引脚"，是因为模拟通道下 START 和 STOP
 *   来自同一次 ADC 采样的同一个分类结果，不能让它们各读一次 ADC
 *   （那会采到两个时刻的值，边界上能出现"开始和停止同时触发"的假象）。
 */
static void debounce_slot_active(sensor_slot_t *s, int active)
{
    if (s->pin < 0) {
        return;
    }
    s->raw = active;
    if (active == s->stable) {
        s->debounce = 0;
        return;
    }

    /* 读数与稳定值不同 → 累计；连着 N 次都不同才认可这次翻转 */
    if (++s->debounce < SENSOR_DEBOUNCE_N) {
        return;
    }
    s->debounce = 0;
    s->stable = active;

    int64_t now = esp_timer_get_time();
    if (active) {
        s->trig_seq++;
        s->last_trig_us = now;
    } else {
        s->rel_seq++;
        s->last_rel_us = now;
    }
}

/** 数字通道：读引脚 → 去抖（模拟方案下用不到，避免 unused 告警） */
#if !BOARD_SENSOR_DUAL_ANALOG
static void debounce_slot(sensor_slot_t *s)
{
    if (s->pin < 0) {
        return;
    }
    debounce_slot_active(s, level_is_active(gpio_get_level((gpio_num_t)s->pin)));
}
#endif

static void sensor_timer_cb(void *arg)
{
    (void)arg;
#if BOARD_SENSOR_DUAL_ANALOG
    /* 模拟通道：每路只采一次 ADC，然后喂给两个"逻辑微动"（开始 / 停止）。
     * ★ 一次采样必须同时决定两只的状态 —— 分别读会把两个时刻的值混在一起，
     *   在阈值边界上能读出"开始和停止同时触发"这种物理上不存在的组合。 */
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        combo_poll(c);
        if (s_combo[c].pin < 0) {
            continue;
        }
        debounce_slot_active(&s_slot[SENSOR_START][c],
                             s_combo[c].cls == COMBO_START ? 1 : 0);
        debounce_slot_active(&s_slot[SENSOR_STOP][c],
                             s_combo[c].cls == COMBO_STOP ? 1 : 0);
    }
#else
    for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
        for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
            debounce_slot(&s_slot[k][c]);
        }
    }
#endif

    /* 补充机满足信号：只读数上报，不参与任何流程 */
    if (s_in_ext_pin >= 0) {
        s_in_ext_level = gpio_get_level((gpio_num_t)s_in_ext_pin);
    }

    /* 挤出机到位信号：GPIO 模式才需要读引脚；MQTT 模式由
     * extruder_inplace_notify_from_mqtt() 直接推进序号 */
    if (s_extruder_pin >= 0) {
        int level = gpio_get_level((gpio_num_t)s_extruder_pin);
        s_extruder_raw = level;
        int active = level_is_active(level);
        if (active != s_extruder_stable) {
            if (++s_extruder_debounce >= SENSOR_DEBOUNCE_N) {
                s_extruder_debounce = 0;
                s_extruder_stable = active;
                if (active) {
                    s_extruder_seq++;
                    s_extruder_last_us = esp_timer_get_time();
                }
            }
        } else {
            s_extruder_debounce = 0;
        }
    }
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

/**
 * 引脚号的显示文本。负值一律返回「无」——
 * 直接 printf("%d") 会打成 "GPIO-1"，C3 上（只有 3 只微动脚）日志和网页
 * 接线表里会一整排 GPIO-1，看着像引脚丢了 / 固件坏了，其实那是设计如此：
 * C3 的 10 个 IO 被电机(2)+离合(4) 占掉 6 个，只剩 4/5/8/9/10 五只，
 * 而 4 通道 × 3 只微动要 12 只 —— 给不出的角色就是 -1，走按时间推进。
 */
static const char *pin_text(char *buf, size_t len, int pin)
{
    if (pin < 0) {
        snprintf(buf, len, "无");
    } else {
        snprintf(buf, len, "GPIO%d", pin);
    }
    return buf;
}

esp_err_t sensor_init(void)
{
    memset(s_slot, 0, sizeof(s_slot));
    s_in_ext_pin   = -1;
    s_in_ext_level = 0;
    for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
        for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
            s_slot[k][c].pin = -1;
            s_slot[k][c].last_trig_us = -1;
            s_slot[k][c].last_rel_us = -1;
        }
    }

    int configured = 0;

#if BOARD_SENSOR_DUAL_ANALOG
    /* ======================================================================
     * 模拟方案：每路 1 个 ADC 脚 → 还原出「开始位 / 停止位」两只逻辑微动
     * ====================================================================== */
    static const int combo_pin[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_COMBO;

    /* ★ 走 adc_shared：ADC1 可能已经被别的模块（电流采样）建过了，
     *   直接 new_unit 会撞 "adc1 is already in use" → ESP_ERR_NOT_FOUND。 */
    esp_err_t u1err = adc_shared_unit(ADC_UNIT_1, &s_adc);
    if (u1err == ESP_OK) {
        s_adc_ok = true;
    } else {
        ams_log_err("ADC1 初始化失败(%s) —— 微动无法采样",
                    esp_err_to_name(u1err));
        ams_log_err("各路将按无微动处理（降级为按时间推进送料）");
        s_adc_ok = false;
    }
    s_adc2_ok = false;

    adc_oneshot_chan_cfg_t ccfg = {
        .atten    = ADC_ATTEN_DB_12,   /* 量程约 0~3.1V，够覆盖 0/0.6/3.3V 三档 */
        .bitwidth = ADC_BITWIDTH_12,
    };

    int adc2_count = 0;
    int adc2_failed = 0;

    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        s_combo[c].pin      = -1;
        s_combo[c].mv       = 0;
        s_combo[c].cls      = COMBO_MID;
        s_combo[c].unit     = ADC_UNIT_1;
        s_combo[c].fails    = 0;
        s_combo[c].degraded = false;

        int pin = combo_pin[c];
        if (pin < 0) {
            continue;
        }

        adc_unit_t    unit = ADC_UNIT_1;
        adc_channel_t ch   = ADC_CHANNEL_0;
        esp_err_t err = adc_oneshot_io_to_channel(pin, &unit, &ch);
        if (err != ESP_OK) {
            ams_log_err("料盘位%d 的微动脚 GPIO%d 不是 ADC 引脚", c + 1, pin);
            continue;
        }

        /* ★ 2026-10-22：PCB 把通道 6/7/8 的微动接在 11/12/13（ADC2）上，
         *   改不了线，所以这里**同时支持 ADC2**，读不到时再降级。
         *   ADC1 的单元建不起来就整块都不采；ADC2 建不起来只影响那几路。 */
        if (unit == ADC_UNIT_2) {
            adc2_count++;
            if (!s_adc2_tried) {
                s_adc2_tried = true;          /* 只建一次，失败也不再重试 */
                esp_err_t e2 = adc_shared_unit(ADC_UNIT_2, &s_adc2);
                if (e2 == ESP_OK) {
                    s_adc2_ok = true;
                } else {
                    s_adc2 = NULL;
                    ams_log_err("ADC2 单元建立失败(%s) —— ADC2 上的微动读不到",
                                esp_err_to_name(e2));
                }
            }
            if (!s_adc2_ok || s_adc2 == NULL) {
                adc2_failed++;
                continue;
            }
        } else if (!s_adc_ok) {
            continue;
        }

        adc_oneshot_unit_handle_t h = (unit == ADC_UNIT_2) ? s_adc2 : s_adc;
        if (adc_oneshot_config_channel(h, ch, &ccfg) != ESP_OK) {
            ams_log_err("料盘位%d 微动脚 GPIO%d（ADC 通道）配置失败",
                        c + 1, pin);
            if (unit == ADC_UNIT_2) {
                adc2_failed++;
            }
            continue;
        }

        /* ★ 分压靠**内部上拉**，必须在 ADC 把脚切到模拟功能之后再打开：
         *   顺序反了会被 adc_oneshot_config_channel 清掉，现象是读数永远是 0
         *   （三档都判成"开始位"，一路狂送料）。 */
        gpio_pulldown_dis((gpio_num_t)pin);
        gpio_pullup_en((gpio_num_t)pin);

        s_combo[c].pin  = pin;
        s_combo[c].ch   = ch;
        s_combo[c].unit = unit;

        /* 两只逻辑微动共用这一个脚（pin_present 为真，上层逻辑不用改） */
        s_slot[SENSOR_STOP][c].pin     = pin;
        s_slot[SENSOR_START][c].pin    = pin;
        configured += 2;
    }
#else
    /* ======================================================================
     * 数字方案（C3）：每只微动一个脚
     * ====================================================================== */
    {
        static const int kind_pins[SENSOR_KIND_COUNT][BOARD_CHANNEL_COUNT] = {
            BOARD_PIN_SENSOR_STOP,
            BOARD_PIN_SENSOR_START,
        };
        uint64_t mask = 0;
        for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
            for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
                int pin = kind_pins[k][c];
                s_slot[k][c].pin = pin;
                if (pin < 0) {
                    continue;
                }
                mask |= (1ULL << pin);
                configured++;
            }
        }
        if (configured > 0) {
            gpio_config_t cfg = {
                .pin_bit_mask = mask,
                .mode         = GPIO_MODE_INPUT,
                .pull_up_en   = BOARD_SENSOR_PULL_UP ? GPIO_PULLUP_ENABLE
                                                     : GPIO_PULLUP_DISABLE,
                .pull_down_en = BOARD_SENSOR_PULL_UP ? GPIO_PULLDOWN_DISABLE
                                                     : GPIO_PULLDOWN_ENABLE,
                .intr_type    = GPIO_INTR_DISABLE,
            };
            esp_err_t err = gpio_config(&cfg);
            if (err != ESP_OK) {
                ams_log_err("微动引脚配置失败: %s", esp_err_to_name(err));
                return err;
            }
        }
    }
#endif

    /* ---- 挤出机到位信号 ---- */
    if (!BOARD_EXTRUDER_INPLACE_UNUSED && BOARD_PIN_EXTRUDER_INPLACE >= 0) {
        s_extruder_pin = BOARD_PIN_EXTRUDER_INPLACE;
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << s_extruder_pin),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = BOARD_SENSOR_PULL_UP ? GPIO_PULLUP_ENABLE
                                                 : GPIO_PULLUP_DISABLE,
            .pull_down_en = BOARD_SENSOR_PULL_UP ? GPIO_PULLDOWN_DISABLE
                                                 : GPIO_PULLDOWN_ENABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK) {
            ams_log_err("挤出机到位信号(GPIO%d)配置失败: %s",
                        s_extruder_pin, esp_err_to_name(err));
            s_extruder_pin = -1;
        }
    } else {
        s_extruder_pin = -1;
    }

    /* ---- 补充机满足信号（只上报，不参与流程） ---- */
    if (!BOARD_IN_EXT_UNUSED && BOARD_PIN_IN_EXT >= 0) {
        s_in_ext_pin = BOARD_PIN_IN_EXT;
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << s_in_ext_pin),
            .mode         = GPIO_MODE_INPUT,
            .pull_up_en   = GPIO_PULLUP_ENABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        if (gpio_config(&cfg) != ESP_OK) {
            s_in_ext_pin = -1;
        }
    }

#if BOARD_SENSOR_DUAL_ANALOG
    /* ---- 先采一次，把初值定下来 ----
     * 不这么做的话，stable 从 0 起算，真实的"停止位顶开"会在上电 15ms 后
     * 产生一次假的"触发"边沿（虽然会被"没打印任务"那道门挡掉，但日志里
     * 会多一行莫名其妙的触发记录，排查时特别容易被带偏）。 */
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        if (s_combo[c].pin < 0) {
            continue;
        }
        combo_poll(c);
        s_combo[c].cls = combo_classify(s_combo[c].mv, COMBO_MID);
        int st = (s_combo[c].cls == COMBO_START);
        int sp = (s_combo[c].cls == COMBO_STOP);
        s_slot[SENSOR_START][c].raw    = st;
        s_slot[SENSOR_START][c].stable = st;
        s_slot[SENSOR_STOP][c].raw     = sp;
        s_slot[SENSOR_STOP][c].stable  = sp;
    }
#else
    /* ---- 读一遍初值，让 stable 反映真实状态，而不是停留在 0 ---- */
    for (int k = 0; k < SENSOR_KIND_COUNT; k++) {
        for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
            if (s_slot[k][c].pin >= 0) {
                int level = gpio_get_level((gpio_num_t)s_slot[k][c].pin);
                s_slot[k][c].raw = level;
                s_slot[k][c].stable = level_is_active(level);
            }
        }
    }
#endif
    if (s_extruder_pin >= 0) {
        int level = gpio_get_level((gpio_num_t)s_extruder_pin);
        s_extruder_raw = level;
        s_extruder_stable = level_is_active(level);
    }
    if (s_in_ext_pin >= 0) {
        s_in_ext_level = gpio_get_level((gpio_num_t)s_in_ext_pin);
    }

    /* ---- 启动去抖定时器 ---- */
    if (s_timer == NULL) {
        esp_timer_create_args_t args = {
            .callback        = sensor_timer_cb,
            .arg             = NULL,
            .dispatch_method = ESP_TIMER_TASK,
            .name            = "ams_sensor",
        };
        esp_err_t err = esp_timer_create(&args, &s_timer);
        if (err != ESP_OK) {
            ams_log_err("创建微动去抖定时器失败: %s", esp_err_to_name(err));
            return err;
        }
    }
    esp_timer_start_periodic(s_timer, SENSOR_POLL_MS * 1000);

    /* ---- 打一份接线摘要 ---- */
#if BOARD_SENSOR_DUAL_ANALOG
    ams_log("%d 只微动（%d 路 × 开始/停止各一，单 IO 电阻编码）已初始化，"
            "去抖 %d×%dms",
            configured, BOARD_CHANNEL_COUNT, SENSOR_DEBOUNCE_N, SENSOR_POLL_MS);
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        char ps[12];
        ams_log("  料盘位%d: 微动=%s%s  %d mV  缓冲送料=%s",
                c + 1,
                pin_text(ps, sizeof(ps), s_combo[c].pin),
                (s_combo[c].pin < 0) ? ""
                    : (s_combo[c].unit == ADC_UNIT_2 ? "(ADC2)" : "(ADC1)"),
                s_combo[c].mv,
                config_sensor_enabled(c) ? "开" : "关");
    }
    ams_log("  判定门限：< %dmV = 开始位(空)  ·  %d~%dmV = 中间  ·  > %dmV = 停止位(满)",
            BOARD_SENSOR_MV_START, BOARD_SENSOR_MV_START,
            BOARD_SENSOR_MV_STOP, BOARD_SENSOR_MV_STOP);
    if (adc2_count > 0) {
        /* ★ 这句必须打出来：现场最容易误判成"微动坏了"的就是这 3 路 */
        ams_log_warn("%d 路微动落在 ADC2（GPIO11~GPIO20，本板是 %d 路）："
                     "WiFi 开启期间 ADC2 读不到，这几路会自动降级成「不参与判断」；"
                     "切到 485 直连（WiFi 关闭）后自动恢复",
                     adc2_count - adc2_failed, adc2_count);
    }
    ams_log("  新硬件每路只有「停止位 / 开始位」两只微动（自吸功能已删除）");
#else
    ams_log("%d 个微动已初始化（%s触发，去抖 %d×%dms）",
            configured, ACTIVE_LEVEL ? "高电平" : "低电平",
            SENSOR_DEBOUNCE_N, SENSOR_POLL_MS);
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        char ps[12], pst[12];
        ams_log("  料盘位%d: 停止=%s 开始=%s  缓冲送料=%s",
                c + 1,
                pin_text(ps,  sizeof(ps),  s_slot[SENSOR_STOP][c].pin),
                pin_text(pst, sizeof(pst), s_slot[SENSOR_START][c].pin),
                config_sensor_enabled(c) ? "开" : "关");
    }
#endif
    if (s_extruder_pin >= 0) {
        ams_log("  挤出机到位信号: GPIO%d", s_extruder_pin);
    } else {
        ams_log("  挤出机到位信号: 不接 GPIO，等打印机 MQTT 事件");
    }

    return ESP_OK;
}

void sensor_deinit(void)
{
    if (s_timer) {
        esp_timer_stop(s_timer);
    }
}

/* ==========================================================================
 * 读数
 * ========================================================================== */

static inline const sensor_slot_t *slot_of(int channel, sensor_kind_t kind)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) {
        return NULL;
    }
    if (kind < 0 || kind >= SENSOR_KIND_COUNT) {
        return NULL;
    }
    return &s_slot[kind][channel];
}

bool sensor_triggered(int channel, sensor_kind_t kind)
{
    const sensor_slot_t *s = slot_of(channel, kind);
    if (s == NULL || s->pin < 0) {
        return false;
    }
    return s->stable != 0;
}

int sensor_raw_level(int channel, sensor_kind_t kind)
{
    const sensor_slot_t *s = slot_of(channel, kind);
    if (s == NULL || s->pin < 0) {
        return -1;
    }
    return s->raw;
}

bool sensor_pin_present(int channel, sensor_kind_t kind)
{
    const sensor_slot_t *s = slot_of(channel, kind);
    return (s != NULL) && (s->pin >= 0);
}

bool sensor_dual_analog_mode(void)
{
    return BOARD_SENSOR_DUAL_ANALOG ? true : false;
}

int sensor_combo_mv(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) {
        return -1;
    }
#if BOARD_SENSOR_DUAL_ANALOG
    return (s_combo[channel].pin >= 0) ? s_combo[channel].mv : -1;
#else
    /* 数字方案没有"电压"这个概念，返回 -1 让前端知道该显示"—"而不是 0mV */
    return -1;
#endif
}

int sensor_combo_adc_unit(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) {
        return 0;
    }
#if BOARD_SENSOR_DUAL_ANALOG
    if (s_combo[channel].pin < 0) {
        return 0;
    }
    return (s_combo[channel].unit == ADC_UNIT_2) ? 2 : 1;
#else
    return 0;
#endif
}

bool sensor_combo_degraded(int channel)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) {
        return false;
    }
#if BOARD_SENSOR_DUAL_ANALOG
    return s_combo[channel].degraded;
#else
    return false;
#endif
}

int sensor_in_ext_level(void)
{
    return (s_in_ext_pin >= 0) ? s_in_ext_level : -1;
}

bool sensor_channel_enabled(int channel)
{
    return config_sensor_enabled(channel);
}

uint8_t sensor_enabled_mask(void)
{
    return config_get()->sensor_enabled_mask;
}

esp_err_t sensor_set_channel_enabled(int channel, bool enabled)
{
    if (channel < 0 || channel >= BOARD_CHANNEL_COUNT) {
        return ESP_ERR_INVALID_ARG;
    }
    ams_config_t *cfg = config_get();
    uint8_t before = cfg->sensor_enabled_mask;
    if (enabled) {
        cfg->sensor_enabled_mask |= (uint8_t)(1u << channel);
    } else {
        cfg->sensor_enabled_mask &= (uint8_t)~(1u << channel);
    }
    if (cfg->sensor_enabled_mask != before) {
        ams_log("料盘位%d 缓冲送料%s%s", channel + 1,
                enabled ? "已开启" : "已关闭",
                enabled ? "：开始送料微动触发送料，停止送料微动触发即停"
                        : "（送/退料改走降级模式：按时间推进）");
        return config_save();
    }
    return ESP_OK;
}

/* ==========================================================================
 * 边沿序号与等待
 * ========================================================================== */

uint32_t sensor_trigger_seq(int channel, sensor_kind_t kind)
{
    const sensor_slot_t *s = slot_of(channel, kind);
    return s ? s->trig_seq : 0;
}

uint32_t sensor_release_seq(int channel, sensor_kind_t kind)
{
    const sensor_slot_t *s = slot_of(channel, kind);
    return s ? s->rel_seq : 0;
}

uint32_t sensor_ms_since_trigger(int channel, sensor_kind_t kind)
{
    const sensor_slot_t *s = slot_of(channel, kind);
    if (s == NULL || s->last_trig_us < 0) {
        return UINT32_MAX;
    }
    int64_t delta = esp_timer_get_time() - s->last_trig_us;
    if (delta < 0) {
        return 0;
    }
    return (uint32_t)(delta / 1000);
}

bool sensor_wait_trigger(int channel, sensor_kind_t kind, uint32_t timeout_ms)
{
    if (!sensor_pin_present(channel, kind)) {
        return false;
    }
    /* 已经触发着就直接算成功 —— 料本来就压着微动时，"开始送料"应该马上动作，
     * 而不是死等一次新的边沿（那会一直等到超时） */
    if (sensor_triggered(channel, kind)) {
        return true;
    }
    uint32_t base = sensor_trigger_seq(channel, kind);
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (sensor_trigger_seq(channel, kind) != base) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
    return false;
}

bool sensor_wait_release(int channel, sensor_kind_t kind, uint32_t timeout_ms)
{
    if (!sensor_pin_present(channel, kind)) {
        return false;
    }
    if (!sensor_triggered(channel, kind)) {
        return true;
    }
    uint32_t base = sensor_release_seq(channel, kind);
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (sensor_release_seq(channel, kind) != base) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
    return false;
}

/* ==========================================================================
 * 挤出机到位信号
 * ========================================================================== */

bool extruder_inplace_triggered(void)
{
    return s_extruder_stable != 0;
}

uint32_t extruder_inplace_seq(void)
{
    return s_extruder_seq;
}

int extruder_inplace_raw_level(void)
{
    return (s_extruder_pin >= 0) ? s_extruder_raw : -1;
}

uint32_t extruder_inplace_ms_since(void)
{
    if (s_extruder_last_us < 0) {
        return UINT32_MAX;
    }
    int64_t delta = esp_timer_get_time() - s_extruder_last_us;
    if (delta < 0) {
        return 0;
    }
    return (uint32_t)(delta / 1000);
}

void extruder_inplace_clear(void)
{
    s_extruder_stable = 0;
    s_extruder_debounce = 0;
    if (s_extruder_pin >= 0) {
        /* 清状态时也把软件值拉到当前引脚状态，避免"清完立刻又变回到位" */
        s_extruder_stable =
            level_is_active(gpio_get_level((gpio_num_t)s_extruder_pin));
    }
}

void extruder_inplace_notify_from_mqtt(void)
{
    /* 走 MQTT 模式时，把上报直接记成一次边沿 —— 效果和 GPIO 触发完全一样，
     * 换料流程不需要区分信号从哪来 */
    s_extruder_stable = 1;
    s_extruder_seq++;
    s_extruder_last_us = esp_timer_get_time();
}

bool extruder_inplace_wait(uint32_t timeout_ms)
{
    if (extruder_inplace_triggered()) {
        return true;
    }
    uint32_t base = s_extruder_seq;
    int64_t deadline = esp_timer_get_time() + (int64_t)timeout_ms * 1000;
    while (esp_timer_get_time() < deadline) {
        if (s_extruder_seq != base) {
            return true;
        }
        vTaskDelay(pdMS_TO_TICKS(SENSOR_POLL_MS));
    }
    return false;
}

/* ==========================================================================
 * 状态描述（给网页）
 * ========================================================================== */

int sensor_describe_json(char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return 0;
    }
    /* enabled = 「缓冲送料」掩码。
     * ch[c] = [停止送料, 开始送料] 两只开关的实时状态。
     * dual  = 1 表示本板型走「单 IO 双微动电阻编码」，此时 mv[] 有读数。 */
    int off = snprintf(buf, buflen,
                       "{\"enabled\":%u,\"dual\":%d,\"extruder\":%d,"
                       "\"inext\":%d,\"ch\":[",
                       (unsigned)sensor_enabled_mask(),
                       BOARD_SENSOR_DUAL_ANALOG ? 1 : 0,
                       extruder_inplace_triggered() ? 1 : 0,
                       sensor_in_ext_level() > 0 ? 1 : 0);

    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        off += snprintf(buf + off, buflen - off, "%s[%d,%d]",
                        c ? "," : "",
                        sensor_triggered(c, SENSOR_STOP) ? 1 : 0,
                        sensor_triggered(c, SENSOR_START) ? 1 : 0);
    }

    /* pins[c] = [停止, 开始] 两只微动**接在哪个 GPIO**（-1 = 没分到脚）。
     * 前端拿它把"这只微动到底接哪只脚"直接写在界面上 ——
     * C3 上大部分角色都是 -1，用户光看徽标分辨不出是"没接"还是"没脚"。 */
    off += snprintf(buf + off, buflen - off, "],\"pins\":[");
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        off += snprintf(buf + off, buflen - off, "%s[%d,%d]",
                        c ? "," : "",
                        s_slot[SENSOR_STOP][c].pin,
                        s_slot[SENSOR_START][c].pin);
    }

    /* mv[c] = 该路微动脚的实测电压（mV）；-1 = 本路没有微动 / 不是模拟方案。
     * ★ 这个字段是给**接线调试**用的：三档的典型值是 220 / 600 / 3300mV，
     *   接错电阻、忘记接 GND、微动卡住都会立刻在里面露出来 —— 比只看
     *   0/1 状态好定位得多。 */
    off += snprintf(buf + off, buflen - off, "],\"mv\":[");
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        off += snprintf(buf + off, buflen - off, "%s%d",
                        c ? "," : "", sensor_combo_mv(c));
    }

    /* adc[c] = 这一路挂在哪个 ADC 上（1 = ADC1 / 2 = ADC2 / 0 = 没接微动）
     * deg[c] = 1 表示"在 ADC2 上且当前读不到"（WiFi 占着射频）→ 已降级
     *
     * ★ 这两个数组只为了**让现场看懂状态**：PCB 固定有 3 路落在 ADC2，
     *   在 MQTT 模式下必然读不到。不标出来的话，用户会以为微动坏了、
     *   或者以为固件有 bug —— 其实切到 485 模式就好了。 */
    off += snprintf(buf + off, buflen - off, "],\"adc\":[");
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        off += snprintf(buf + off, buflen - off, "%s%d",
                        c ? "," : "", sensor_combo_adc_unit(c));
    }
    off += snprintf(buf + off, buflen - off, "],\"deg\":[");
    for (int c = 0; c < BOARD_CHANNEL_COUNT; c++) {
        off += snprintf(buf + off, buflen - off, "%s%d",
                        c ? "," : "", sensor_combo_degraded(c) ? 1 : 0);
    }

    off += snprintf(buf + off, buflen - off, "]}");
    return off;
}
