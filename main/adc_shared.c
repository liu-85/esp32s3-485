/**
 * adc_shared.c —— ADC oneshot 单元的唯一归属点（见 .h 里的背景说明）
 */

#include "adc_shared.h"

#include <stdbool.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "soc/soc_caps.h"

/** S3 / C3 都是 2（ADC1 + ADC2）；个别老芯片不同，按 SOC 宏走 */
#define ADC_SHARED_UNITS (SOC_ADC_PERIPH_NUM)

static portMUX_TYPE s_mux = portMUX_INITIALIZER_UNLOCKED;

static adc_oneshot_unit_handle_t s_h[ADC_SHARED_UNITS];
static esp_err_t                 s_err[ADC_SHARED_UNITS];
static bool                      s_tried[ADC_SHARED_UNITS];
static bool                      s_creating;   /* 防止两个任务同时 new_unit */

esp_err_t adc_shared_unit(adc_unit_t unit, adc_oneshot_unit_handle_t *out)
{
    if (out == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    *out = NULL;

    if ((int)unit < 0 || (int)unit >= ADC_SHARED_UNITS) {
        return ESP_ERR_INVALID_ARG;
    }

    /* ---- 快路径：已经建过（无论成功失败），直接返回结论 ---- */
    portENTER_CRITICAL(&s_mux);
    if (s_tried[unit]) {
        *out = s_h[unit];
        esp_err_t err = s_err[unit];
        portEXIT_CRITICAL(&s_mux);
        return err;
    }
    if (s_creating) {
        portEXIT_CRITICAL(&s_mux);
        return ESP_ERR_INVALID_STATE;   /* 另一个任务正在建，稍后再来 */
    }
    s_creating = true;
    portEXIT_CRITICAL(&s_mux);

    /* ---- 慢路径：真正创建。**必须在临界区之外**：
     *      new_unit 内部要分配内存、拿它自己的 lock，关中断跑这些不安全。 */
    adc_oneshot_unit_init_cfg_t cfg = {
        .unit_id  = unit,
        /* ★ clk_src 传 0 = 让 IDF 按芯片自己选默认时钟源。
         *
         *   **不能写死 ADC_RTC_CLK_SRC_DEFAULT**：C3 的 ADC 走 DIGI 控制器
         *   （SOC_ADC_DIG_CTRL_SUPPORTED && !SOC_ADC_RTC_CTRL_SUPPORTED），
         *   那个枚举在 C3 上根本不存在 → 编译直接失败。而 adc_oneshot.c 里是
         *       adc_oneshot_clk_src_t clk_src = ADC_DIGI_CLK_SRC_DEFAULT;
         *       if (init_config->clk_src) { clk_src = init_config->clk_src; }
         *   传 0 就保持它挑好的默认值，两个目标都对。 */
        .clk_src  = 0,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    adc_oneshot_unit_handle_t h   = NULL;
    esp_err_t                 err = adc_oneshot_new_unit(&cfg, &h);

    portENTER_CRITICAL(&s_mux);
    s_h[unit]     = (err == ESP_OK) ? h : NULL;
    s_err[unit]   = err;
    s_tried[unit] = true;
    s_creating    = false;
    *out          = s_h[unit];
    portEXIT_CRITICAL(&s_mux);

    return err;
}

bool adc_shared_ok(adc_unit_t unit)
{
    if ((int)unit < 0 || (int)unit >= ADC_SHARED_UNITS) {
        return false;
    }

    bool ok = false;
    portENTER_CRITICAL(&s_mux);
    ok = (s_tried[unit] && s_h[unit] != NULL);
    portEXIT_CRITICAL(&s_mux);
    return ok;
}
