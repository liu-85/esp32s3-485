/**
 * current_sense.h —— 电机电流采样（AT8236 的 ISEN）+ 堵转保护
 * ============================================================================
 *
 * 硬件（见 PCB 原理图）：
 *
 *     AT8236 ISEN(pin7) ──┬── R11 200mΩ ── GND
 *                         └── G3（GPIO3，ADC1_CH2）
 *
 * 为什么 ISEN 是"电压"而不是"比例电流"：
 *   AT8236 数据手册给的关系是   V_ISEN = I_motor × R_ISEN
 *   内部的 10 倍增益（A_V = 10）用在**限流比较器**上：
 *       I_TRIP = V_REF / (A_V × R_ISEN)
 *   所以采样电阻 200mΩ 时，**1A 对应 200mV**，也就是 200 mV/A。
 *   本板 VREF 接 5V（原理图），芯片内部限流点约 5/(10×0.2) = **2.5A**。
 *
 *   ⚠ 因此固件保护值应设在 2.5A 以下。超过之后芯片自己斩波，ISEN 会出现
 *     一个平顶，固件看到的是"电流不再上升"，分不出真堵转还是被限流。
 *
 * 采样上的两个硬问题：
 *   1. **PWM 锯齿**：20kHz 下 ISEN 在导通期上升、关断期衰减，单次采样没意义。
 *      做法是每 10ms 连采一串取平均（跨好几个 PWM 周期），再做一阶低通。
 *   2. **盲区时间**：芯片在每次导通后 2.2µs 内忽略 ISEN（blanking），
 *      这期间采到的值偏低。取平均天然会把它稀释掉，不单独处理。
 *
 * 堵转保护的边界（很重要）：
 *   · 只在电机**正在转**的时候判（停着时 ISEN 恒 0，判了也没意义）
 *   · 起步后 blank_ms 内不判 —— 直流电机起动电流是稳态的 3~8 倍，
 *     不屏蔽的话每次起步都会误跳闸
 *   · 超过阈值要**持续** stall_ms 才动作，避免单次毛刺
 *   · 动作 = 停电机 + 断全部离合 + 置故障位；电机停稳 2 秒后自动复归，
 *     不需要断电重启（现场还要能继续试）
 *
 * 配置存放在**独立的 NVS 键**里（命名空间 ams_cs），不动 ams_config_t 的
 * 布局 —— 那边一旦改长度就要升 CONFIG_VERSION + 写迁移，代价远大于收益。
 */

#ifndef CURRENT_SENSE_H
#define CURRENT_SENSE_H

#include <stdbool.h>
#include <stdint.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 采样电阻默认值（毫欧），与 BOARD_CURRENT_SENSE_R_MOHM 一致 */
#define CURRENT_R_MOHM_DEF   200
/** 堵转阈值默认 0 = **不保护**，必须现场测出来再填（见网页上的说明） */
#define CURRENT_STALL_MA_DEF 0
/** 超过阈值持续多久算堵转 */
#define CURRENT_STALL_MS_DEF 300
/** 起步屏蔽（避开起动电流） */
#define CURRENT_BLANK_MS_DEF 800

/** 各项夹取范围（网页/接口都按这个夹，别信调用方） */
#define CURRENT_R_MOHM_MIN    10
#define CURRENT_R_MOHM_MAX  2000
#define CURRENT_MV_PER_A_MIN  10
#define CURRENT_MV_PER_A_MAX 5000
#define CURRENT_STALL_MA_MIN  50
#define CURRENT_STALL_MA_MAX 20000
#define CURRENT_STALL_MS_MIN  20
#define CURRENT_STALL_MS_MAX 5000
#define CURRENT_BLANK_MS_MIN   0
#define CURRENT_BLANK_MS_MAX 5000

/** 电流采样与堵转保护的参数（网页「硬件调试」可改，存 NVS） */
typedef struct {
    uint16_t enable;      /**< 1 = 采样 + 保护都开；0 = 全关 */
    uint16_t r_mohm;      /**< 采样电阻（毫欧） */
    uint16_t mv_per_a;    /**< 灵敏度 mV/A；0 = 直接用 r_mohm（AT8236 就是这样） */
    uint16_t stall_ma;    /**< 堵转阈值 mA；0 = 只显示不保护 */
    uint16_t stall_ms;    /**< 超过阈值持续多久才算堵转 */
    uint16_t blank_ms;    /**< 起步屏蔽时间，避开起动电流 */
} current_cfg_t;

/** 初始化 ADC + 读 NVS 参数 + 起采样任务。失败也不拖垮整机（返回错误即可） */
esp_err_t current_sense_init(void);

/** 当前生效参数（只读指针，改请用 current_cfg_set） */
const current_cfg_t *current_cfg_get(void);

/**
 * 写入新参数：先逐项夹到安全区间，再存 NVS，最后立刻生效。
 * 返回实际存进去的那份（夹取后的值），调用方据此判断"填的值被改过没有"。
 */
current_cfg_t current_cfg_set(const current_cfg_t *in);

/** 恢复默认（200mΩ / 不保护 / 300ms / 800ms 屏蔽） */
current_cfg_t current_cfg_reset(void);

/** 实际使用的灵敏度（mV/A）：mv_per_a 为 0 时返回 r_mohm */
int current_effective_mv_per_a(void);

/** 最近一次滤波后的电压（mV）；-1 = 采样不可用 */
int current_mv(void);
/** 最近约 1 秒内的峰值电压（mV）；-1 = 采样不可用 */
int current_peak_mv(void);
/** 换算出的电流（mA）；采样不可用时返回 -1 */
int current_ma(void);
/** 换算出的峰值电流（mA）；采样不可用时返回 -1 */
int current_peak_ma(void);

/** true = 上一次动作是被堵转保护掐停的 */
bool current_stall_tripped(void);
/** 累计跳闸次数 */
uint32_t current_stall_count(void);
/** 人工清除故障位（网页上那个「清除」按钮） */
void current_clear_fault(void);
/** 清掉峰值（重新开始观察） */
void current_reset_peak(void);

/** ADC 是否可用（初始化成没成） */
bool current_sense_ready(void);

/** 打包成 JSON 给 /status 用；返回写入长度 */
int current_describe_json(char *buf, size_t buflen);

#ifdef __cplusplus
}
#endif

#endif /* CURRENT_SENSE_H */
