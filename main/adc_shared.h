/**
 * adc_shared.h —— ADC oneshot 单元的**唯一归属点**
 * ============================================================================
 *
 * ★★ 为什么需要这个模块（2026-10-22 现场故障的直接原因）★★
 *
 * ESP-IDF 的 `adc_oneshot_new_unit()` 对**同一个 ADC unit 只能成功一次**。
 * 源码（components/esp_adc/adc_oneshot.c）里是一句：
 *
 *     bool success_claim = s_adc_unit_claim(init_config->unit_id);
 *     ESP_GOTO_ON_FALSE(success_claim, ESP_ERR_NOT_FOUND, err, TAG,
 *                       "adc%d is already in use", init_config->unit_id + 1);
 *
 * 也就是说第二次调用返回的是 **ESP_ERR_NOT_FOUND**（不是 INVALID_STATE，
 * 这个错误码极具误导性 —— 看起来像"脚不存在"，实际是"单元已被占用"）。
 *
 * 现场现象：
 *     电流采样 ADC1 初始化失败: ESP_ERR_NOT_FOUND
 *     ★ 电流采样初始化失败 —— 电流不显示、堵转保护不可用，送料功能不受影响
 *
 * 原因是 `filament_sensor.c` 先调 `sensor_init()` 建了 ADC1，之后
 * `current_sense_init()` 又去建一次 → 撞车。两个模块各建各的，谁先谁后
 * 就看 main 里的调用顺序，属于**隐式耦合**，加一个用 ADC 的功能就炸一次。
 *
 * 解法：不管有几个模块要用 ADC，都只通过本模块拿句柄。
 *   · 第一次调用 → 真的 new_unit
 *   · 之后再调用 → 直接返回同一个句柄
 *   · 建失败 → 把错误码记住，后续调用立刻返回同一个错误（不重复打日志）
 *
 * ★ 注意：**不提供 del**。句柄是整个固件共享的，任何模块都不该销毁它
 *   （原来 current_sense 的失败路径里有 adc_oneshot_del_unit，会把微动
 *   正在用的 ADC1 一起删掉 —— 这比它自己失败严重得多，已去掉）。
 *
 * ★ atten 可以按通道不同：`adc_oneshot_read()` 内部每次转换前都会
 *   `adc_oneshot_ll_set_atten(unit, chan, chan_configs[chan].atten)`，
 *   所以在同一个 unit 上，微动用 DB_12(3.1V)、电流采样用 DB_0(0.95V)
 *   互不干扰。已核对 hal/adc_oneshot_hal.c 的 adc_oneshot_hal_setup()。
 */

#ifndef ADC_SHARED_H
#define ADC_SHARED_H

#include <stdbool.h>

#include "esp_adc/adc_oneshot.h"
#include "esp_err.h"

/**
 * 取（必要时创建）指定 ADC unit 的 oneshot 句柄。
 *
 * @param unit  ADC_UNIT_1 / ADC_UNIT_2
 * @param out   成功时写入句柄；失败写 NULL。不可为 NULL。
 * @return ESP_OK 成功；否则为 new_unit 的原始错误码（ESP_ERR_NOT_FOUND /
 *         ESP_ERR_NO_MEM / ESP_ERR_NOT_SUPPORTED 等）。
 *
 * 线程安全：内部加锁，可在任意任务里调用；但**不保证**第一次创建是懒加载
 * 完成的 —— 谁先调用谁触发创建。
 */
esp_err_t adc_shared_unit(adc_unit_t unit, adc_oneshot_unit_handle_t *out);

/** 该 unit 是否已经成功建好（用于降级判断，不触发创建） */
bool adc_shared_ok(adc_unit_t unit);

#endif /* ADC_SHARED_H */
