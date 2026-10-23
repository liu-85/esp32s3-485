/**
 * motor.c —— 共享直流电机驱动（LEDC PWM 两路输出）
 * ============================================================================
 *
 * 用两个 LEDC 通道分别驱动 H 桥的 IN1 / IN2：
 *
 *     方向       IN1 占空比        IN2 占空比
 *     进料       speed            0
 *     退料       0                speed
 *     停止       0                0
 *
 * LEDC 定时器选 20kHz / 8 位分辨率：
 *   · 20kHz 在人耳听阈（20Hz~20kHz）之外，电机不会啸叫
 *   · 8 位（0~255）分辨率对这种"只要快慢两档"的场合足够了
 *   · S3 的 LEDC 时钟 80MHz，80MHz / 20kHz / 256 ≈ 15.6，分频器能精确表达，
 *     LEDC_AUTO_CLK 让驱动自己算，不会出现频率偏差
 *
 * ⚠️ 换向死区：H 桥上下管直通会瞬间烧驱动芯片，所以换向前必须先两路都写 0、
 *    等 dead_time_ms 再反向。这个值和 Python 版一样默认 30ms。
 */

#include "motor.h"

#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "board_pins.h"
#include "log_buffer.h"

/* ==========================================================================
 * 硬件参数
 * ========================================================================== */
#define MOTOR_LEDC_MODE       LEDC_LOW_SPEED_MODE
#define MOTOR_LEDC_TIMER      LEDC_TIMER_0
#define MOTOR_LEDC_CH_IN1     LEDC_CHANNEL_0
#define MOTOR_LEDC_CH_IN2     LEDC_CHANNEL_1
#define MOTOR_LEDC_RES        LEDC_TIMER_8_BIT   /* 占空比 0~255 */
#define MOTOR_LEDC_FREQ_HZ    20000              /* 20kHz，超出人耳听阈 */

/** 占空比满量程（8 位） */
#define MOTOR_DUTY_MAX        255
#define MOTOR_DEFAULT_DEAD_MS 30

/* ==========================================================================
 * 运行状态
 * ========================================================================== */
static motor_dir_t s_dir;
static int         s_speed_pct;
static uint32_t    s_dead_time_ms = MOTOR_DEFAULT_DEAD_MS;
static int64_t     s_dir_started_us;   /* 当前方向是什么时候开始的 */

/** 把百分比换算成 LEDC 的 duty 原值。公开版本见 motor.h（诊断日志要用） */
uint32_t motor_pct_to_duty(int pct)
{
    if (pct <= 0) {
        return 0;
    }
    if (pct > 100) {
        pct = 100;
    }
    return (uint32_t)((pct * MOTOR_DUTY_MAX + 50) / 100);
}

/**
 * 直接写两路占空比。
 *
 * ★ 这是唯一碰硬件的地方。上层无论怎么调，最终都收敛到这里，所以"两路绝不
 *   同时为大"这条硬件红线必须在本函数里落实 —— 不能指望调用方守规矩。
 *
 * ----------------------------------------------------------------------------
 * ★★ 写引脚顺序是硬件级红线，不要"优化"成先写目标通道 ★★
 * ----------------------------------------------------------------------------
 * 错法（看起来只省两次寄存器写，实际会烧驱动芯片）：
 *      写 IN1 = 新值   ← 此刻 IN2 还是上一刻的旧值
 *      写 IN2 = 新值
 *   从「退料」切到「进料」时，IN2 本来有占空比，先写 IN1 的那一瞬间
 *   **IN1 和 IN2 同时为高** → H 桥上下管直通 → 米级电流穿过两个 MOS，
 *   驱动芯片当场烧掉。
 *
 * 对法（本函数的做法）：
 *      先写 IN1 = 0
 *      再写 IN2 = 0        ← 到这里两路一定都是 0，不可能直通
 *      最后给目标通道赋值
 *   代价是常态多两个寄存器写（us 级），换的是"任何调用序列都不会直通"。
 *   本函数还额外兜了一道：万一调用方传进来两路都非 0，直接当刹车处理 ——
 *   宁可电机不动，也不能直通。
 */
static void motor_apply(int in1_pct, int in2_pct)
{
    /* 兜底：两路都非 0 属于逻辑错误，一律按"刹车"处理 */
    if (in1_pct != 0 && in2_pct != 0) {
        in1_pct = 0;
        in2_pct = 0;
    }

    const uint32_t d1 = motor_pct_to_duty(in1_pct);
    const uint32_t d2 = motor_pct_to_duty(in2_pct);

    /* ---- 第一步：双路清零。到这一步结束，H 桥一定不会直通 ---- */
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, 0);
    ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);

    /* ---- 第二步：给目标通道赋值（两路都 0 时就是停车，什么都不用写） ---- */
    if (d1) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1, d1);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    } else if (d2) {
        ledc_set_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2, d2);
        ledc_update_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);
    }
}

esp_err_t motor_init(void)
{
    /* ---- 1) 定时器 ---- */
    ledc_timer_config_t timer = {
        .speed_mode      = MOTOR_LEDC_MODE,
        .timer_num       = MOTOR_LEDC_TIMER,
        .duty_resolution = MOTOR_LEDC_RES,
        .freq_hz         = MOTOR_LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer);
    if (err != ESP_OK) {
        ams_log_err("电机 PWM 定时器配置失败: %s", esp_err_to_name(err));
        return err;
    }

    /* ---- 2) 两个通道 ---- */
    ledc_channel_config_t ch1 = {
        .gpio_num   = BOARD_PIN_MOTOR_IN1,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEDC_CH_IN1,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch1);
    if (err != ESP_OK) {
        ams_log_err("电机 IN1(GPIO%d) PWM 通道配置失败: %s",
                    BOARD_PIN_MOTOR_IN1, esp_err_to_name(err));
        return err;
    }

    ledc_channel_config_t ch2 = {
        .gpio_num   = BOARD_PIN_MOTOR_IN2,
        .speed_mode = MOTOR_LEDC_MODE,
        .channel    = MOTOR_LEDC_CH_IN2,
        .timer_sel  = MOTOR_LEDC_TIMER,
        .duty       = 0,
        .hpoint     = 0,
    };
    err = ledc_channel_config(&ch2);
    if (err != ESP_OK) {
        ams_log_err("电机 IN2(GPIO%d) PWM 通道配置失败: %s",
                    BOARD_PIN_MOTOR_IN2, esp_err_to_name(err));
        return err;
    }

    /* ---- 3) 上电必须静止 ---- */
    s_dir = MOTOR_DIR_STOP;
    s_speed_pct = MOTOR_SPEED_FULL;
    s_dir_started_us = 0;
    motor_apply(0, 0);

    ams_log("共享电机已初始化: IN1=GPIO%d IN2=GPIO%d  PWM %dHz/%d位  换向死区=%ums",
            BOARD_PIN_MOTOR_IN1, BOARD_PIN_MOTOR_IN2,
            MOTOR_LEDC_FREQ_HZ, 8, (unsigned)s_dead_time_ms);
    return ESP_OK;
}

void motor_set_dead_time_ms(uint32_t ms)
{
    s_dead_time_ms = ms;
}

void motor_stop(void)
{
    if (s_dir == MOTOR_DIR_STOP) {
        /* 已经是停止状态，仍然写一次硬件 —— 万一是外部改过引脚，
         * 这一次能把状态纠正回来，代价只是两次 LEDC 寄存器写。 */
        motor_apply(0, 0);
        return;
    }
    motor_apply(0, 0);
    s_dir = MOTOR_DIR_STOP;
    s_dir_started_us = 0;
}

esp_err_t motor_set_dir_speed(motor_dir_t dir, int speed_pct)
{
    if (dir != MOTOR_DIR_FEED && dir != MOTOR_DIR_RETRACT &&
        dir != MOTOR_DIR_STOP) {
        return ESP_ERR_INVALID_ARG;
    }
    if (speed_pct < 0) {
        speed_pct = 0;
    }
    if (speed_pct > 100) {
        speed_pct = 100;
    }

    if (dir == MOTOR_DIR_STOP) {
        motor_stop();
        return ESP_OK;
    }

    /* 速度为 0 等同于停止：绝不能出现"方向设为进料但占空比 0"这种中间态，
     * 那会让上层以为电机在转、实际没动 */
    if (speed_pct == 0) {
        motor_stop();
        return ESP_OK;
    }

    /* 同方向同速度重复设置：幂等，不打断已运行的动作 */
    bool same_dir = (s_dir == dir);
    if (same_dir && s_speed_pct == speed_pct && s_dir_started_us != 0) {
        return ESP_OK;
    }

    if (!same_dir && s_dir != MOTOR_DIR_STOP) {
        /* ---- 换向：先刹车，留出死区时间 ---- */
        motor_apply(0, 0);
        if (s_dead_time_ms) {
            vTaskDelay(pdMS_TO_TICKS(s_dead_time_ms));
        }
    }

    if (dir == MOTOR_DIR_FEED) {
        motor_apply(speed_pct, 0);
    } else {
        motor_apply(0, speed_pct);
    }

    /* 只有"重新起步"（原来是停的，或这次是换向）才重置计时；
     * 同方向只改速度时保留原起点，elapsed 反映整段动作的时长。 */
    if (!same_dir || s_dir_started_us == 0) {
        s_dir_started_us = esp_timer_get_time();
    }
    s_dir = dir;
    s_speed_pct = speed_pct;
    return ESP_OK;
}

esp_err_t motor_set_dir(motor_dir_t dir)
{
    return motor_set_dir_speed(dir, MOTOR_SPEED_FULL);
}

esp_err_t motor_run_speed(motor_dir_t dir, uint32_t ms, int speed_pct)
{
    esp_err_t err = motor_set_dir_speed(dir, speed_pct);
    if (err != ESP_OK) {
        return err;
    }
    if (ms > 0) {
        vTaskDelay(pdMS_TO_TICKS(ms));
    }
    return ESP_OK;
}

esp_err_t motor_run(motor_dir_t dir, uint32_t ms)
{
    return motor_run_speed(dir, ms, MOTOR_SPEED_FULL);
}

esp_err_t motor_run_then_stop(motor_dir_t dir, uint32_t ms)
{
    esp_err_t err = motor_run(dir, ms);
    motor_stop();
    return err;
}

motor_dir_t motor_get_dir(void)
{
    return s_dir;
}

int motor_get_speed_pct(void)
{
    return s_speed_pct;
}

uint32_t motor_elapsed_ms(void)
{
    if (s_dir == MOTOR_DIR_STOP || s_dir_started_us == 0) {
        return 0;
    }
    int64_t delta_us = esp_timer_get_time() - s_dir_started_us;
    if (delta_us < 0) {
        return 0;
    }
    return (uint32_t)(delta_us / 1000);
}

/**
 * 回读硬件：LEDC 寄存器里的 duty + IN1/IN2 引脚电平。
 *
 * ⚠️ 注意调用时机：motor_stop() 会把两路清零，所以必须在**通电期间**调用
 *    （drive_channel_speed 里就是在 motor_run_speed() 返回后、motor_stop()
 *    之前取的，那时电机还在跑）。
 *
 * gpio_get_level() 读的是 pad 上的实际电平。输出脚上它等于输出值；如果这
 * 颗芯片/这个引脚恰好读不回来（某些脚有特殊功能），也不要报错，直接给 -1，
 * 让上层知道"这一位没有证据"，而不是拿个假的 0 去误导现场。
 */
void motor_readback(motor_readback_t *out)
{
    if (out == NULL) {
        return;
    }
    out->duty_in1  = ledc_get_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN1);
    out->duty_in2  = ledc_get_duty(MOTOR_LEDC_MODE, MOTOR_LEDC_CH_IN2);

    out->level_in1 = -1;
    out->level_in2 = -1;
    if (BOARD_PIN_MOTOR_IN1 >= 0 && BOARD_PIN_MOTOR_IN1 < 64) {
        out->level_in1 = gpio_get_level((gpio_num_t)BOARD_PIN_MOTOR_IN1);
    }
    if (BOARD_PIN_MOTOR_IN2 >= 0 && BOARD_PIN_MOTOR_IN2 < 64) {
        out->level_in2 = gpio_get_level((gpio_num_t)BOARD_PIN_MOTOR_IN2);
    }
}
