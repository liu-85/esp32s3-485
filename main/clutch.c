/**
 * clutch.c —— 电磁离合的实现（通道数由 BOARD_CHANNEL_COUNT 决定）
 * ============================================================================
 *
 * 核心是 clutch_engage()，它把"吸合前先全部断开"这件事做成了**无条件**的：
 *
 *     ┌─ 拿锁（防止两个任务同时切换） ──────────────────────────┐
 *     │  1. 停电机（绝不带着转速切换离合）                       │
 *     │  2. 把**全部**通道写成断开电平                           │
 *     │  3. 如果刚才确实有某路在吸合，等 断开+静默 的时间         │
 *     │  4. 回读全部通道，只要有 1 路还吸着 → 全部断开 + 报冲突  │
 *     │  5. 吸合目标通道，等咬合时间                              │
 *     └─────────────────────────────────────────────────────────┘
 *
 * 第 4 步是这个模块的灵魂。没有它，任何一次"驱动板没跟上"或"线圈粘连"都会
 * 被无声吞掉，直到某天两卷料互相拉扯才发现。
 */

#include "clutch.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"

#include "log_buffer.h"
#include "motor.h"

/* ==========================================================================
 * 内部状态
 * ========================================================================== */
static const int s_pins[BOARD_CHANNEL_COUNT] = BOARD_PIN_CLUTCH;

static const int s_active_level   = BOARD_CLUTCH_ACTIVE_LEVEL;
static const int s_inactive_level = 1 - BOARD_CLUTCH_ACTIVE_LEVEL;

static SemaphoreHandle_t s_lock;

/** 软件记账：总线认为当前吸合的是哪一路（0 = 全断开） */
static int      s_active;
/** 当前占用总线的动作用户名，打日志用（指向常量字符串，不需要释放） */
static const char *s_owner;
/** 是否正处在一次复合动作中 */
static bool     s_busy;
/** 累计违规次数，健康状态永远是 0 */
static uint32_t s_conflicts;

/* ==========================================================================
 * 底层：唯一碰引脚的地方（static，不对外暴露）
 * ========================================================================== */

static inline void clutch_apply_one(int index, bool on, bool settle)
{
    gpio_set_level((gpio_num_t)s_pins[index],
                   on ? s_active_level : s_inactive_level);
    if (settle) {
        vTaskDelay(pdMS_TO_TICKS(on ? CLUTCH_ENGAGE_MS : CLUTCH_RELEASE_MS));
    }
}

/**
 * 无条件把 4 路全写成断开（可选择性等待机械脱开）。
 * 这是所有纠正动作的公共出口。
 */
static void clutch_apply_all_off(bool settle)
{
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        clutch_apply_one(i, false, false);
    }
    if (settle) {
        vTaskDelay(pdMS_TO_TICKS(CLUTCH_RELEASE_MS));
    }
    s_active = 0;
    s_owner = NULL;
}

/* ==========================================================================
 * 查询（读引脚，不是读软件记账）
 * ========================================================================== */

uint8_t clutch_engaged_mask(void)
{
    uint8_t mask = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (gpio_get_level((gpio_num_t)s_pins[i]) == s_active_level) {
            mask |= (uint8_t)(1u << i);
        }
    }
    return mask;
}

bool clutch_is_engaged(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) {
        return false;
    }
    return (clutch_engaged_mask() & (1u << (channel - 1))) != 0;
}

int clutch_active_channel(void)
{
    return s_active;
}

void clutch_set_owner(const char *owner)
{
    s_owner = owner;
}

const char *clutch_get_owner(void)
{
    return s_owner ? s_owner : "";
}

void clutch_set_busy(bool busy)
{
    s_busy = busy;
}

bool clutch_is_busy(void)
{
    return s_busy;
}

uint32_t clutch_conflict_count(void)
{
    return s_conflicts;
}

void clutch_reset_conflict_count(void)
{
    s_conflicts = 0;
}

/* ==========================================================================
 * 初始化
 * ========================================================================== */

esp_err_t clutch_init(void)
{
    if (s_lock == NULL) {
        s_lock = xSemaphoreCreateMutex();
    }

    int bad = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << s_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        esp_err_t err = gpio_config(&cfg);
        if (err != ESP_OK) {
            /* ★ 2026-10-22 改：原来这里直接 return err —— 8 通道里只要有
             *   一只脚配不上（比如某只脚在模组上被占），**整块离合都不再
             *   初始化**，现场看到的就是"电机没反应"（因为离合没吸合，
             *   drive_channel_speed 会拒绝驱动电机）。
             *   现在改成：记下来、跳过这一路、其余照常工作。 */
            ams_log_err("电磁离合%d(GPIO%d) 配置失败: %s —— 该路跳过，其余照常",
                        i + 1, s_pins[i], esp_err_to_name(err));
            bad++;
            continue;
        }
        /* ★ 先写断开电平，再交给下面的初始化流程 —— 顺序反了的话，
         *   中间那一瞬间引脚是"浮空/上一个状态"，继电器或驱动板可能误动作 */
        gpio_set_level((gpio_num_t)s_pins[i], s_inactive_level);
    }

    s_active = 0;
    s_owner = NULL;
    s_busy = false;
    s_conflicts = 0;
    clutch_apply_all_off(false);

    /* ---- 上电回读自检（★ "点进料、电机一点反应都没有"最快的判据）----
     * 写完"全部断开"之后立刻回读。只要有一路仍读到**吸合**电平，
     * clutch_engage() 第 4 步的复核就会判"冲突"并拒绝吸合；而离合没吸合时
     * drive_channel_speed() 是**不会驱动电机**的 —— 现场表现就是
     * "按了进料，电机完全没反应"，日志里只有一条容易被忽略的"离合没吸合"。
     * 所以这一条必须在开机日志里明确露出来，带上具体脚号。 */
    {
        uint8_t stuck = clutch_engaged_mask();
        if (stuck) {
            char det[64] = {0};
            int o = 0;
            for (int i = 0; i < BOARD_CHANNEL_COUNT && o < 52; i++) {
                if (stuck & (1u << i)) {
                    o += snprintf(det + o, sizeof(det) - o, "%s%d(GPIO%d)",
                                  o ? "," : "", i + 1, s_pins[i]);
                }
            }
            ams_log_err("★ 离合上电自检异常：通道 %s 写入「断开」后仍读到「吸合」"
                        "电平 —— 检查该路驱动/接线。此状态下所有送料都会被拒绝"
                        "（现象就是电机不转）", det);
        }
    }

    char list[64] = {0};
    int off = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        off += snprintf(list + off, sizeof(list) - off, "%s%d",
                        i ? "," : "", s_pins[i]);
        if (off >= (int)sizeof(list) - 1) {
            break;
        }
    }
    ams_log("%d 路电磁离合已初始化: GPIO%s（%s吸合，吸合等待%ums）%s",
            BOARD_CHANNEL_COUNT, list,
            s_active_level ? "高电平" : "低电平", (unsigned)CLUTCH_ENGAGE_MS,
            bad ? "  ★ 有脚配置失败（见上面的错误），该路不可用" : "");
    /* 只有"一路都没配上"才算整体失败；部分失败仍然可用，别把整机拖死 */
    return (bad < BOARD_CHANNEL_COUNT) ? ESP_OK : ESP_FAIL;
}

/* ==========================================================================
 * 核心：互斥吸合
 * ========================================================================== */

esp_err_t clutch_engage(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) {
        ams_log_err("离合通道 %d 不存在，可用 1~%d",
                    channel, BOARD_CHANNEL_COUNT);
        return ESP_ERR_CLUTCH_CHANNEL;
    }
    if (s_lock == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        ams_log_err("切换离合超时：有别的动作正占着总线");
        return ESP_ERR_TIMEOUT;
    }

    const int index = channel - 1;

    /* ---- 同一通道重复吸合是幂等的：不重复动作，避免机构抖动 ---- */
    if (s_active == channel &&
        gpio_get_level((gpio_num_t)s_pins[index]) == s_active_level) {
        xSemaphoreGive(s_lock);
        return ESP_OK;
    }

    /* ---- 1) 电机必须先停，绝不能带着转速切换离合 ---- */
    motor_stop();

    /* ---- 2) 物理层：先全部断开 ---- */
    bool had_engaged = (clutch_engaged_mask() != 0);
    clutch_apply_all_off(false);

    /* ---- 3) 等上一路离合彻底脱开，否则会短暂出现"两路都咬合"的机械重叠 ---- */
    if (had_engaged) {
        vTaskDelay(pdMS_TO_TICKS(CLUTCH_RELEASE_MS + CLUTCH_SETTLE_MS));
    }

    /* ---- 4) 复核：必须全部处于断开状态 ---- */
    uint8_t still = clutch_engaged_mask();
    if (still != 0) {
        s_conflicts++;
        clutch_apply_all_off(false);
        xSemaphoreGive(s_lock);

        char detail[48] = {0};
        int off = 0;
        for (int i = 0; i < BOARD_CHANNEL_COUNT && off < 40; i++) {
            if (still & (1u << i)) {
                off += snprintf(detail + off, sizeof(detail) - off,
                                "%s%d", off ? "," : "", i + 1);
            }
        }
        ams_log_err("吸合通道 %d 前检测到通道 %s 仍处于吸合状态，"
                    "已强制全部断开并取消本次动作（累计第 %u 次）",
                    channel, detail, (unsigned)s_conflicts);
        return ESP_ERR_CLUTCH_CONFLICT;
    }

    /* ---- 5) 吸合目标 ---- */
    clutch_apply_one(index, true, true);
    s_active = channel;

    xSemaphoreGive(s_lock);
    return ESP_OK;
}

esp_err_t clutch_release(int channel)
{
    if (channel < 1 || channel > BOARD_CHANNEL_COUNT) {
        return ESP_ERR_CLUTCH_CHANNEL;
    }
    if (!clutch_is_engaged(channel)) {
        return ESP_OK;
    }
    if (s_lock == NULL ||
        xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    esp_err_t ret = ESP_OK;
    if (s_active != channel) {
        /* 引脚吸着但软件记账说不是它 —— 状态不一致，按最安全的做法处理：
         * 全部断开，并记一次违规 */
        s_conflicts++;
        ams_log_err("通道 %d 处于吸合状态但未被总线记录，已强制全部断开",
                    channel);
        ret = ESP_ERR_CLUTCH_CONFLICT;
    }
    motor_stop();
    clutch_apply_all_off(true);

    xSemaphoreGive(s_lock);
    return ret;
}

void clutch_release_all(void)
{
    motor_stop();
    if (s_lock != NULL &&
        xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) == pdTRUE) {
        clutch_apply_all_off(false);
        s_busy = false;
        xSemaphoreGive(s_lock);
    } else {
        /* 拿不到锁也必须保证安全 —— 直接在锁外写，宁可短暂竞争也不要
         * 留一路离合吸着 */
        clutch_apply_all_off(false);
        s_busy = false;
    }
}

void clutch_release_all_settled(void)
{
    motor_stop();
    if (s_lock != NULL &&
        xSemaphoreTake(s_lock, pdMS_TO_TICKS(3000)) == pdTRUE) {
        clutch_apply_all_off(true);
        s_busy = false;
        xSemaphoreGive(s_lock);
    } else {
        clutch_apply_all_off(true);
        s_busy = false;
    }
}

/* ==========================================================================
 * 运行期体检
 * ========================================================================== */

bool clutch_assert_single(bool force_release)
{
    uint8_t mask = clutch_engaged_mask();
    int count = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (mask & (1u << i)) {
            count++;
        }
    }

    if (count > 1) {
        s_conflicts++;
        char detail[48] = {0};
        int off = 0;
        for (int i = 0; i < BOARD_CHANNEL_COUNT && off < 40; i++) {
            if (mask & (1u << i)) {
                off += snprintf(detail + off, sizeof(detail) - off,
                                "%s%d", off ? "," : "", i + 1);
            }
        }
        ams_log_err("体检发现 %d 路电磁离合同时吸合（通道 %s），已处置",
                    count, detail);
        if (force_release) {
            clutch_release_all();
        }
        return false;
    }

    /* 让软件记账与实际引脚保持一致（引脚可能被外部改过） */
    int actual = 0;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (mask & (1u << i)) {
            actual = i + 1;
            break;
        }
    }
    if (s_active != actual) {
        s_active = actual;
        if (actual == 0) {
            s_owner = NULL;
        }
    }
    return true;
}

/* ==========================================================================
 * 状态描述（给网页）
 * ========================================================================== */

int clutch_describe_json(char *buf, size_t buflen, int motor_direction,
                         bool busy, const char *owner)
{
    if (!buf || buflen == 0) {
        return 0;
    }
    uint8_t mask = clutch_engaged_mask();

    int off = snprintf(buf, buflen,
                       "{\"active\":%d,\"engaged\":[", s_active);

    bool first = true;
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        if (mask & (1u << i)) {
            off += snprintf(buf + off, buflen - off, "%s%d",
                            first ? "" : ",", i + 1);
            first = false;
        }
    }

    off += snprintf(buf + off, buflen - off,
                    "],\"conflicts\":%u,\"motor_direction\":%d,"
                    "\"channels\":[",
                    (unsigned)s_conflicts, motor_direction);

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        off += snprintf(buf + off, buflen - off, "%s%d",
                        i ? "," : "", i + 1);
    }

    /* owner 只可能是我们自己的常量名（不含引号 / 反斜杠），直接嵌入 JSON 是安全的 */
    const char *who = owner ? owner : (s_owner ? s_owner : "");
    off += snprintf(buf + off, buflen - off, "],\"busy\":%s,\"owner\":\"%s\"}",
                    busy ? "true" : "false", who);

    return off;
}
