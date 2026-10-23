/**
 * clutch.h —— 4 路电磁离合（带互斥仲裁）
 * ============================================================================
 *
 * 这个模块是整套代码里**安全级别最高**的一块，因为它守着一条硬约束：
 *
 *     ★ 任何时刻最多只能有 1 路电磁离合吸合 ★
 *
 * 违反它的后果不是"不好用"，是"会坏东西"：
 *   · 两路同时咬合 → 两卷料被同一个电机往相反方向拉 → 料线绷断 / 打滑
 *   · 两路线圈浪涌叠一起 → 5V 被拉塌 → ESP32 欠压复位
 *   · 带着转速切换离合 → 齿轮打齿
 *
 * 所以这里照搬了 MicroPython 版 FilamentMotorBus 的**四重保障**：
 *
 *   1. 物理层前置断开
 *      engage() 在吸合目标通道之前，**无条件**先把所有离合写成"断开"电平，
 *      再等一段时间让上一路彻底脱开，然后才吸合目标。
 *      也就是说，任何一次吸合动作都是从"全断开"状态出发的。
 *
 *   2. 吸合前状态复核
 *      写完全部断开后立刻回读每一路的状态。如果发现还有残留吸合
 *      （引脚被外部代码改写、驱动板没跟上、线圈粘连），立即再次全部断开，
 *      并返回失败取消本次动作，绝不带病吸合。
 *
 *   3. 直接调用拦截
 *      写引脚只有一个内部函数 `clutch_apply()`，**不对外暴露**。
 *      上层想吸合只能走 clutch_engage()，永远没有绕过仲裁的路径。
 *
 *   4. 运行期体检
 *      clutch_assert_single() 可以随时调用，发现多路吸合就立即全部断开。
 *      ams_controller 的主循环每个周期都调一次（只是读 4 个 GPIO，开销极小）。
 *
 * 另外所有"吸合 → 动作 → 断开"的复合动作都被 ams_controller 用统一出的
 * `ams_bus_guard_begin/end` 包住，异常路径也一定会释放离合、停电机
 * （对应 Python 版 try/finally 的作用，这里用 FreeRTOS 的清理函数实现）。
 */

#ifndef CLUTCH_H
#define CLUTCH_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 时序参数（改这里就够，不用翻 .c）
 * ========================================================================== */

/** 吸合后的机械稳定等待，单位 ms —— 离合咬合需要时间 */
#define CLUTCH_ENGAGE_MS   80
/** 断开后的机械稳定等待 */
#define CLUTCH_RELEASE_MS  60
/** 从一路切到另一路时的额外静默时间，确保上一路彻底脱开 */
#define CLUTCH_SETTLE_MS   20

/* ==========================================================================
 * 错误码（借用 esp_err_t 的自定义区间）
 * ========================================================================== */
/** 通道号越界（有效范围 1 ~ BOARD_CHANNEL_COUNT） */
#define ESP_ERR_CLUTCH_CHANNEL   0x7001
/** 检测到多路同时吸合 —— 严重违反硬件约束 */
#define ESP_ERR_CLUTCH_CONFLICT  0x7002

/* ==========================================================================
 * 生命周期
 * ========================================================================== */

/**
 * 初始化 4 路离合：
 *   · 把 4 个引脚配成输出，并**立刻全部写成断开**
 *   · 建互斥锁（防止两个任务同时吸合不同通道）
 *
 * 上电第一件事就是"确保全断开"，绝不能让板子带着上次的残留状态起跑。
 */
esp_err_t clutch_init(void);

/* ==========================================================================
 * 操作
 * ========================================================================== */

/**
 * 吸合指定通道（1 起），并**强制断开其它所有通道**。
 *
 * @param channel 1 ~ BOARD_CHANNEL_COUNT
 * @return
 *     ESP_OK                    吸合成功
 *     ESP_ERR_CLUTCH_CHANNEL    通道号越界
 *     ESP_ERR_CLUTCH_CONFLICT   吸合前复核发现残留吸合（已强制全断开，本次取消）
 *     ESP_ERR_TIMEOUT           拿不到互斥锁（有别的任务正在切换通道）
 */
esp_err_t clutch_engage(int channel);

/** 断开指定通道（如果它正好是当前吸合的那一路） */
esp_err_t clutch_release(int channel);

/**
 * 断开全部离合。
 *
 * ⚠️ 这是**总是安全**的操作，任何异常路径、任何清理逻辑都应该调它。
 *    不会失败、不等机械动作以外的事情。
 */
void clutch_release_all(void);

/**
 * 断开全部离合并等待机械彻底脱开（用于换料收尾）。
 * 比 clutch_release_all() 多等一次 CLUTCH_RELEASE_MS。
 */
void clutch_release_all_settled(void);

/* ==========================================================================
 * 查询
 * ========================================================================== */

/** 当前吸合的通道号；0 表示全部断开 */
int clutch_active_channel(void);

/**
 * 直接读引脚得到的"实际吸合掩码"（bit0 = 通道1）。
 *
 * ★ 这个值来自硬件回读，不是软件记账 —— 体检就是靠它发现"软件以为断开了、
 *   实际还吸着"这种不一致。
 */
uint8_t clutch_engaged_mask(void);

/** 某一通道是否吸合（读引脚，非软件记账） */
bool clutch_is_engaged(int channel);

/** 累计检测到几次违规（正常应该永远是 0） */
uint32_t clutch_conflict_count(void);

/** 清零违规计数（网页"重置诊断"用） */
void clutch_reset_conflict_count(void);

/**
 * 记录当前占用总线的动作名（只用于日志与网页显示，不参与仲裁）。
 * 例：clutch_set_owner("换料-通道2")
 */
void clutch_set_owner(const char *owner);

/** 当前占用者名字；没有动作在跑时返回 "" */
const char *clutch_get_owner(void);

/**
 * 标记总线是否正处在一次两段式动作中。
 *
 * 网页靠它判断"现在能不能再按一下点动"：忙的时候直接拒绝并说明，
 * 而不是让用户排长队等（那正是"按一下转圈半天"的来源）。
 */
void clutch_set_busy(bool busy);

/** 总线是否忙 */
bool clutch_is_busy(void);

/**
 * 运行期体检：确认同时最多只有 1 路吸合。
 *
 * @param force_release 发现异常时是否立即全部断开（默认传 true）
 * @return true 表示状态正常
 */
bool clutch_assert_single(bool force_release);

/**
 * 生成一份可 JSON 序列化的状态描述（给网页用）。
 * 写入 buf，返回写入字符数。形如：
 *   {"active":2,"engaged":[2],"conflicts":0,"channels":[1,2,3,4]}
 */
int clutch_describe_json(char *buf, size_t buflen, int motor_direction,
                         bool busy, const char *owner);

#ifdef __cplusplus
}
#endif

#endif /* CLUTCH_H */
