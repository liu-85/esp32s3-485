/**
 * board_pins_s3.h —— ESP32-S3（42 针）引脚表  ★ 改接线只改这一个文件 ★
 * ============================================================================
 *
 * 这个文件对应 MicroPython 版的 board_s3.py + hardware_config.py 里的引脚部分。
 * 单独抽出来是因为它最常被改：换一块模组、改一根线，都只动这里，业务代码
 * （motor / clutch / ams_controller / web_server）一行都不用碰。
 *
 * 由 board_pins.h 自动选择 include（S3 → 本文件，C3 → board_pins_c3.h）。
 *
 * ----------------------------------------------------------------------------
 * ★★ 2026-10-22 第三次改：**按已打样的 PCB 原理图逐针核对**（本表即真相）★★
 * ----------------------------------------------------------------------------
 *
 *   上一版为了把 8 路模拟微动全塞进 ADC1，把离合 C1/C2 从 6/7 挪到了 11/12。
 *   但**板子已经打样了**，实际走线是固定的（原理图核对结果）：
 *
 *     电机 AT8236   IN1=G5  IN2=G4  VREF=5V  ISEN→G3（200mΩ 采样电阻 R11）
 *     离合 ULN2803  In1..In8 = G47,G21,G18,G17,G16,G15,G7,G6
 *                   Out18..Out11 = O8..O1 → 电磁8..电磁1
 *                   → 反推：**电磁1=GPIO6, 2=GPIO7, 3=15, 4=16,
 *                            5=17, 6=18, 7=21, 8=47**
 *     微动            G1,G2,G9,G8,G10,G11,G12,G13（= 1,2,9,8,10,11,12,13）
 *     485           G14(TX) / G48(RX) / G39(DE)
 *     灯            G42
 *
 *   所以本版**离合回到 6/7/15/16/17/18/21/47**（= 现场原始表，一根线都不用改）。
 *
 *   ★★ 代价：GPIO3 被 ISEN 占了，微动只剩 5 个 ADC1 脚 ★★
 *
 *     ADC1 一共 10 个（GPIO1~10）：
 *          4、5   → 电机
 *          6、7   → 离合 1、2（PCB 固定）
 *          3      → ISEN 电流采样（PCB 固定）
 *          1、2、8、9、10 → **只剩这 5 个给微动**
 *     而这里有 8 路微动 → 必然有 3 路落在 ADC2（11/12/13）。
 *
 *     ADC2 的限制（ESP-IDF 官方文档原话）：adc2 的读取在 wifi 启动期间
 *     **可能失败**（返回 ESP_ERR_TIMEOUT），S3 上 ADC2 由 WiFi PHY 仲裁并让路。
 *     所以：
 *       · **MQTT 模式（WiFi 开）** → 通道 6/7/8 的微动读不到，自动降级
 *       · **485 模式（WiFi 关）**  → 8 路全部正常 ← 这是最终形态
 *     固件会把这件事**在开机日志和网页上明确标出来**，不静默失败
 *     （见 filament_sensor.c 的 s_combo[].adc2 / adc2_ok）。
 *
 *   三档电压（VDD=3.3V，内部上拉典型 45K，实测值见 filament_sensor.c 的日志）：
 *
 *     状态                                串联电阻        估算电压    判定
 *     开始位压下（缓冲器空，要补料）      4.7K∥10K=3.2K   ≈0.22V    START
 *     中间（只有停止位常闭那只 10K）     10K             ≈0.60V    MID
 *     停止位顶开（缓冲器满，NC 断开）     ∞（只有上拉）   ≈3.30V    STOP
 *
 *     判定阈值：< 433mV = 开始位，> 1050mV = 停止位，其余 = 中间。
 *     这两个数按"内部上拉 31K~58K 全范围都不误判"选的，见 .c 里的推导。
 *
 *   ⚠️ GPIO3 同时是 strapping（JTAG 源选择）。ISEN 经 200Ω 到地，上电时
 *      没有电流 → ISEN≈0V → GPIO3 上电被拉低 → ROM 选"外部 JTAG"
 *      （GPIO39~42 先按 JTAG 功能）。固件随后把 39/42 重新配成普通 GPIO，
 *      功能不受影响；若发现 485 方向脚或状态灯异常，优先怀疑这里。
 */

#ifndef BOARD_PINS_S3_H
#define BOARD_PINS_S3_H

#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 一、通道数
 * ========================================================================== */
/**
 * 料盘位（电磁离合 + 微动组）数量。整套代码都按这个宏循环，加通道只改这里。
 *
 * ★ 4 → 8 时结构体长度会变（688 → 716），必须同时升 CONFIG_VERSION 并写迁移，
 *   否则老设备刷完固件会丢掉 WiFi / MQTT 访问码 / 颜色映射。见 config.h。
 */
#define BOARD_CHANNEL_COUNT 8

/* ==========================================================================
 * 二、执行机构（输出）
 * ========================================================================== */
#define BOARD_PIN_MOTOR_IN1 4      /* H 桥 IN1：方向 1 = 进料（正转）
                                    * ⚠ 净表是 G5→AT8236 的 IN1、G4→IN2。
                                    *   本文件沿用历史的 4/5 命名，所以固件里的
                                    *   "IN1"实际驱动 G4（=芯片 IN2）。
                                    *   进/退料方向反了就把这两行对调即可。 */
#define BOARD_PIN_MOTOR_IN2 5      /* H 桥 IN2：方向 -1 = 退料（反转） */
/* 电磁离合 1~8，高电平吸合。
 *
 * ★ 2026-10-22 第三次改：按 PCB 实际走线（ULN2803A 反推）填回原始表。
 *      ULN2803 的 In1..In8 = G47,G21,G18,G17,G16,G15,G7,G6
 *      Out18..Out11 = O8..O1，而 O1..O8 接电磁1..8
 *      → 电磁1=G6 电磁2=G7 电磁3=G15 电磁4=G16
 *        电磁5=G17 电磁6=G18 电磁7=G21 电磁8=G47
 *   上一版把 C1/C2 挪到 11/12，但 **11/12 在 PCB 上根本没接到 ULN2803**
 *   → 通道 1~6 点亮了错误的线圈、通道 7~8 完全没反应。这就是现场
 *   "电磁不吸合"的根因，本版已修正。
 *
 * ★ ULN2803 是**反相**达林顿阵列：输入高 → 输出拉低 → 线圈（另一端接 24V）
 *   得电吸合。所以 BOARD_CLUTCH_ACTIVE_LEVEL 仍然是 1（高电平吸合）。
 *   ⚠ 前提：ULN2803 的 COM(pin10) 必须接 24V（续流二极管公共端），见原理图。 */
#define BOARD_PIN_CLUTCH {6, 7, 15, 16, 17, 18, 21, 47}
#define BOARD_PIN_LED 42           /* 板载状态灯；不需要就改成 -1 */

/* ==========================================================================
 * 三-1、★ 电机电流采样（ISEN / AT8236）★ 新增
 * ==========================================================================
 * 原理图：AT8236 的 ISEN(pin7) ──┬── R11 200mΩ ── GND
 *                                └── G3（GPIO3）
 *
 * 电流关系（AT8236 数据手册：ISEN 增益 A_V = 10，目标电流
 *   I_TRIP = V_REF /(A_V × R_ISEN)）：
 *
 *     V_ISEN = I_motor × R_ISEN
 *
 *   这是**电压输出**，不是比例电流 —— 200mΩ 时正好 **200 mV/A**。
 *     · 0.50A → 100mV     · 1.00A → 200mV
 *     · 1.65A → 330mV（VREF=3.3V 时的芯片内部限流点）
 *     · 2.50A → 500mV（本板 VREF 接 5V，芯片硬件限流约 2.5A）
 *
 * ⚠ 所以**固件的堵转保护要设在 2.5A 以下**才有意义 —— 超过 2.5A 芯片自己
 *   就斩波了，固件看到的是一个"卡在 500mV"的平顶，分不出真假堵转。
 *
 * ★ 采样点必须在**下管导通期**才准：20kHz PWM 下 ISEN 是锯齿波，固件
 *   用"一串连采取平均 + 取峰值"来抹平（见 current_sense.c）。
 */
#define BOARD_PIN_CURRENT_SENSE 3
/** 1 = 本板型有 ISEN 采样（C3 没有，见 board_pins_c3.h） */
#define BOARD_CURRENT_SENSE_ENABLED 1
/** 采样电阻（毫欧）。默认值只用于"第一次上电就能大概显示"，实际值网页可改 */
#define BOARD_CURRENT_SENSE_R_MOHM 200

/* ==========================================================================
 * 三、★ 微动开关（输入）：每路 2 只，合占 1 个 ADC 脚 ★
 * ==========================================================================
 * 数组下标 0~7 对应料盘位 1~8，**每路一个脚**。
 *
 * 接线（一路的样子）：
 *
 *     3V3 ──[内部上拉 45K]──┐
 *                           ├── GPIO1（ADC1_CH0）── 固件读电压
 *           开始位 微动(NO) ──┤
 *                    │        │
 *                  4.7K       │
 *                    │        │
 *                   GND       │
 *           停止位 微动(NC) ──┤
 *                    │        │
 *                   10K       │
 *                    │        │
 *                   GND       │
 *
 * 用 1/2/9/8/10/11/12/13 的理由：**这就是 PCB 上 G1/G2/G9/G8/G10/G11/G12/G13
 * 这八个网络标号的顺序**，一根线都不用改（与现场原始 IO 表一致）。
 * 其中 GPIO11/12/13 是 ADC2 → 见文件头"代价"那段，WiFi 开启时这 3 路会降级。
 *
 * ⚠ 如果实测发现"料盘位 N"对应的不是 G 标号 N（比如厂家把 W6 接到了 G13），
 *   **只改这一行**的顺序即可，别去动别的。
 */
#define BOARD_PIN_SENSOR_COMBO {1, 2, 9, 8, 10, 11, 12, 13}

/** 1 = 走「单 IO 双微动电阻编码 + ADC」；0 = 走旧的「每只微动一个数字脚」 */
#define BOARD_SENSOR_DUAL_ANALOG 1

/* ---- 电阻编码参数（换了电阻就改这里，阈值跟着自动算） ---- */
#define BOARD_SENSOR_R_START_OHM 4700    /* 开始位微动串联电阻（常开那只） */
#define BOARD_SENSOR_R_STOP_OHM 10000    /* 停止位微动串联电阻（常闭那只） */
#define BOARD_SENSOR_RPU_OHM    45000    /* 芯片内部上拉典型值，只用来折算阈值 */
#define BOARD_SENSOR_VDD_MV     3300     /* 供电电压，折算阈值用 */

/**
 * 三档判定阈值（mV），按上面的 Rpu 折算出来的。
 *
 *   < MV_START             → 开始位压下（有 4.7K 参与了分压）
 *   MV_START ~ MV_STOP     → 中间（只有 10K 那只常闭的接着）
 *   > MV_STOP              → 停止位顶开（回路断开，只剩上拉）
 *
 * 为什么这两个数字能容忍内部上拉 31K~58K 的离散：
 *   电压 = VDD × R/(R+Rpu)，R 只有 3.2K / 10K / ∞ 三档，且**单调**
 *   （R 越大电压越高），所以只要阈值落在 4.7K 和 10K 之间、10K 和 ∞ 之间，
 *   上拉怎么飘都不会把两档搞反。433mV ≈ 6.8K，1050mV ≈ 21K，
 *   分别落在 (4.7K,10K) 和 (10K,∞) 的中间。
 */
#define BOARD_SENSOR_MV_START 433
#define BOARD_SENSOR_MV_STOP  1050

/**
 * 旧方案的微动脚 —— 本表**不用**，定义成 -1 只是为了让 filament_sensor.c
 * 里那段数字引脚代码（C3 还在用）在 S3 上也能编译通过。
 * 真正生效的是上面的 BOARD_PIN_SENSOR_COMBO。
 */
#define BOARD_PIN_SENSOR_STOP     {-1, -1, -1, -1, -1, -1, -1, -1}
#define BOARD_PIN_SENSOR_START    {-1, -1, -1, -1, -1, -1, -1, -1}

/**
 * 挤出机到位信号（全局）。
 *
 * 新接线表里没有这一根线（脚都排给 485 / 微动 / 电流采样了），所以走打印机
 * MQTT 上报，由 ams_controller.c 的 extruder_inplace_notify_from_mqtt() 驱动。
 * ⚠ 不要再想"拿 GPIO10 顶一下" —— 10 现在是第 5 路微动，占了就少一路。
 */
#define BOARD_PIN_EXTRUDER_INPLACE -1
#define BOARD_EXTRUDER_INPLACE_UNUSED true

/* ==========================================================================
 * 四、RS-485（BambuBus 直连打印机）—— 新增
 * ==========================================================================
 * 和 MQTT 是**两条互斥的通路**，用哪个由 config 里的 comm_mode 决定
 * （网页「硬件调试」可切，默认 MQTT）。
 *
 * 电气要点（照 BMCU 的实现核实过）：
 *   · 1 250 000 bps、8 数据位 + **偶校验** + 1 停止位（= 9 位字长）
 *   · 半双工：DE 高 = 发，DE 低 = 收，**空闲必须处于收**
 *   · ESP32-S3 用 APB 80MHz 分频，80M/1.25M = 64 整除 → 波特率误差 0
 *
 * 引脚选择：TX=14 / RX=48 都在 GPIO1~10 之外（不抢 ADC1），
 *          DE=39 是 JTAG 的 MTDO，不外接调试器时当普通输出没问题。
 */
#define BOARD_PIN_485_TX 14
#define BOARD_PIN_485_RX 48
#define BOARD_PIN_485_DE 39
/** 1 = 本板型有 485 电路；0 = 没有（C3 就是这样，脚不够） */
#define BOARD_485_ENABLED 1

/**
 * 补充机满足信号（外部输入，低有效，内部上拉）。
 *
 * ⚠️ GPIO0 是 strapping 脚（BOOT 键），**上电瞬间不能被拽低**，否则芯片进
 *    下载模式。所以外部那台设备必须是"开漏/干接点 + 上电时高阻"的接法；
 *    如果它上电就输出低电平，这一路必须换脚（GPIO38/40/41 是本表剩下的
 *    干净脚，见 BOARD_SPARE_PINS）。
 *
 * 目前只做「读电平 + 上报给网页」，不参与任何自动流程。
 */
#define BOARD_PIN_IN_EXT 0
/** 1 = 不接这根线 */
#define BOARD_IN_EXT_UNUSED false

/* ==========================================================================
 * 五、电平约定
 * ========================================================================== */
/** 微动有效电平：0 = 低电平触发。★ 模拟通道下不用它，只是给数字通道（C3）留的 */
#define BOARD_SENSOR_ACTIVE_LEVEL 0
/** 微动是否启用内部上拉。★ 模拟通道**必须**开着 —— 分压就靠这个上拉 */
#define BOARD_SENSOR_PULL_UP true
/** 电磁离合有效电平：1 = 高电平吸合（驱动板若为低电平吸合则改 0） */
#define BOARD_CLUTCH_ACTIVE_LEVEL 1
/** 状态灯有效电平：1 表示高电平点亮（多数开发板如此） */
#define BOARD_LED_ACTIVE_LEVEL 1

/* ==========================================================================
 * 六、引脚分类（上电自检用）
 * ========================================================================== */
#define BOARD_GPIO_MAX 48

/** strapping 脚：作**输入**安全，作**输出**危险（会改上电瞬间的电平） */
#define BOARD_STRAPPING_PINS {0, 3, 45, 46}

/** 完全不能用的脚（模组内部占用） */
typedef struct {
    int gpio;
    const char *why;
} board_reserved_pin_t;

#define BOARD_RESERVED_PINS                                                    \
    {                                                                          \
        {19, "USB D-（占用后没有 USB 串口与内置 JTAG）"},                      \
        {20, "USB D+（占用后没有 USB 串口与内置 JTAG）"},                      \
        {26, "模组内置 SPI Flash (SPI0/1)"}, {27, "模组内置 SPI Flash (SPI0/1)"}, \
        {28, "模组内置 SPI Flash (SPI0/1)"}, {29, "模组内置 SPI Flash (SPI0/1)"}, \
        {30, "模组内置 SPI Flash (SPI0/1)"}, {31, "模组内置 SPI Flash (SPI0/1)"}, \
        {32, "模组内置 SPI Flash (SPI0/1)"},                                   \
        {33, "八线 PSRAM 占用（无 PSRAM 的模组上可用）"},                      \
        {34, "八线 PSRAM 占用（无 PSRAM 的模组上可用）"},                      \
        {35, "八线 PSRAM 占用（无 PSRAM 的模组上可用）"},                      \
        {36, "八线 PSRAM 占用（无 PSRAM 的模组上可用）"},                      \
        {37, "八线 PSRAM 占用（无 PSRAM 的模组上可用）"},                      \
        {43, "UART0 TX（日志 / 串口命令口）"},                                 \
        {44, "UART0 RX（日志 / 串口命令口）"},                                 \
    }

/* ==========================================================================
 * 七、剩余可用 IO —— 本方案没占用、也不是保留脚的全部空闲脚
 * ==========================================================================
 * 8 路方案占完之后：38、40、41（13 已被微动占用）
 * strapping（仅建议作输入）：45、46
 */
typedef struct {
    int pin;
    const char *note;
} board_spare_pin_t;

#define BOARD_SPARE_PINS                                                       \
    {                                                                          \
        {38, "干净脚，可作输入或输出"},                                        \
        {40, "外部 JTAG(MTDI)；不接调试器时可作普通 IO（GPIO3 低电平时会被 ROM 先占）"}, \
        {41, "外部 JTAG(MTDO)；同上"},                                         \
        {45, "strapping(VDD_SPI 电压选择)，仅建议作输入"},                     \
        {46, "strapping(ROM 打印)，仅建议作输入"},                             \
    }

/* ==========================================================================
 * 八、工具函数（header-only，避免再多一个 .c 文件）
 * ========================================================================== */

/** 本方案实际用到的全部引脚（用于重复占用检查） */
typedef struct {
    int  pin;
    char role[40];
} board_used_pin_t;

/**
 * 把"本方案用到的所有引脚 + 角色名"填进 out（最多 max 项），返回实际数量。
 *
 * 上电时拿它跑两件事：
 *   1. 查有没有同一只脚被两个角色占用（接线表改错时最常见的错误）
 *   2. 查有没有踩到保留脚 / 把 strapping 脚当输出用
 */
static inline int board_collect_used(board_used_pin_t *out, int max)
{
    static const int clutch[BOARD_CHANNEL_COUNT] = BOARD_PIN_CLUTCH;
    static const int s_combo[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_COMBO;
    int n = 0;

#define ADD(_pin, _role_fmt, ...)                                              \
    do {                                                                       \
        if (n < max) {                                                         \
            out[n].pin = (_pin);                                               \
            snprintf(out[n].role, sizeof(out[n].role), _role_fmt, __VA_ARGS__);\
            n++;                                                               \
        }                                                                      \
    } while (0)

    ADD(BOARD_PIN_MOTOR_IN1, "%s", "电机 IN1");
    ADD(BOARD_PIN_MOTOR_IN2, "%s", "电机 IN2");
#if BOARD_CURRENT_SENSE_ENABLED
    /* ★ 角色名里**不能出现"电机"两个字**：board_pins_validate() 判断"是不是
     *   输出脚"时靠 strstr(role, "电机")，而 ISEN 挂在 GPIO3 —— 那正好是
     *   strapping 脚（JTAG 源选择）。名字带"电机"会让它被当成输出脚，
     *   开机直接报"GPIO3 是 strapping 脚，不能当输出用"，纯属误报。
     *   它实际是**模拟输入**，作输入是安全的。 */
    ADD(BOARD_PIN_CURRENT_SENSE, "%s", "ISEN 电流采样");
#endif
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        ADD(clutch[i], "电磁离合%d", i + 1);
    }
    if (BOARD_PIN_LED >= 0) {
        ADD(BOARD_PIN_LED, "%s", "状态 LED");
    }
    /* ★ 模拟方案下每路**只有一只脚**：如果按"停止/开始"各登记一次，
     *   重复占用检查会立刻报错（它俩本来就该是同一条腿）。
     *   所以这里合成一条角色名写清楚。 */
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        ADD(s_combo[i], "通道%d微动(开始4K7+停止10K)", i + 1);
    }
    if (BOARD_PIN_EXTRUDER_INPLACE >= 0 && !BOARD_EXTRUDER_INPLACE_UNUSED) {
        ADD(BOARD_PIN_EXTRUDER_INPLACE, "%s", "挤出机到位信号");
    }
    if (BOARD_485_ENABLED) {
        ADD(BOARD_PIN_485_TX, "%s", "RS-485 TX");
        ADD(BOARD_PIN_485_RX, "%s", "RS-485 RX");
        ADD(BOARD_PIN_485_DE, "%s", "RS-485 DE(方向)");
    }
    if (BOARD_PIN_IN_EXT >= 0 && !BOARD_IN_EXT_UNUSED) {
        ADD(BOARD_PIN_IN_EXT, "%s", "补充机满足信号");
    }
#undef ADD
    return n;
}

/**
 * 引脚表自检。发现错误返回 false，并把人类可读的说明写进 err（可以为 NULL）。
 *
 * 查四件事：
 *   1. 重复占用 —— 同一只脚被两个角色用了
 *   2. 越界 / 踩保留脚 —— 用了 Flash / USB / UART 这些不能碰的脚
 *   3. strapping 脚被当输出用 —— 其中最严重的是电机 H 桥（AT8236 的 IN 脚
 *      内置下拉，上电瞬间会把 strapping 拉低，芯片进不了启动模式）
 *   4. 模拟微动脚不在 ADC1 范围 —— 落到 ADC2（GPIO11~20）上，WiFi 一开
 *      就采不到值，现象是"微动永远不动"，而且完全不报错
 */
static inline bool board_pins_validate(char *err, size_t errlen)
{
    board_used_pin_t used[32];
    int n = board_collect_used(used, (int)(sizeof(used) / sizeof(used[0])));
    const board_reserved_pin_t reserved[] = BOARD_RESERVED_PINS;
    const int strapping[] = BOARD_STRAPPING_PINS;
    static const int s_combo[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_COMBO;

    bool ok = true;
#define FAIL(_fmt, ...)                                                        \
    do {                                                                       \
        ok = false;                                                            \
        if (err && errlen) {                                                   \
            snprintf(err, errlen, _fmt, ##__VA_ARGS__);                        \
            return false; /* 一次只报第一个错，避免刷屏 */                     \
        }                                                                      \
    } while (0)

    /* 1) 重复占用 */
    for (int i = 0; i < n; i++) {
        for (int j = i + 1; j < n; j++) {
            if (used[i].pin == used[j].pin) {
                FAIL("引脚 GPIO%d 被「%s」和「%s」同时占用",
                     used[i].pin, used[i].role, used[j].role);
            }
        }
    }

    /* 2) 越界 / 保留脚 */
    for (int i = 0; i < n; i++) {
        int p = used[i].pin;
        if (p < 0 || p > BOARD_GPIO_MAX) {
            FAIL("GPIO%d（%s）超出 ESP32-S3 的 GPIO0~GPIO%d 范围",
                 p, used[i].role, BOARD_GPIO_MAX);
        }
        for (size_t r = 0; r < sizeof(reserved) / sizeof(reserved[0]); r++) {
            if (reserved[r].gpio == p) {
                FAIL("GPIO%d（%s）被 %s 占用，不能用",
                     p, used[i].role, reserved[r].why);
            }
        }
    }

    /* 3) strapping 脚被当输出用 */
    for (int i = 0; i < n; i++) {
        int p = used[i].pin;
        bool is_strap = false;
        for (size_t s = 0; s < sizeof(strapping) / sizeof(strapping[0]); s++) {
            if (strapping[s] == p) {
                is_strap = true;
                break;
            }
        }
        if (!is_strap) {
            continue;
        }
        /* 输入角色允许（内部上拉 + 开关对地，空闲就是高电平）；输出角色不允许 */
        bool is_output = (strstr(used[i].role, "电机") != NULL) ||
                         (strstr(used[i].role, "电磁离合") != NULL) ||
                         (strstr(used[i].role, "LED") != NULL) ||
                         (strstr(used[i].role, "RS-485") != NULL);
        if (is_output) {
            FAIL("GPIO%d（%s）是 strapping 启动模式脚，不能当输出用！"
                 "上电瞬间会改掉启动电平，芯片可能进不了正常模式"
                 "（现象：反复重启、网页打不开、电机一直叫）",
                 p, used[i].role);
        }
    }

    /* 4) 模拟微动脚必须是**能当 ADC 用的脚**（ADC1 / ADC2 都算）
     *
     * ★ 2026-10-22 第三次改：这一项从"必须在 ADC1"降级为"必须在 ADC1 或 ADC2"。
     *   原因：PCB 已经打样，微动固定接在 1/2/9/8/10/11/12/13 上，其中
     *   11/12/13 是 ADC2，接不了也改不了。硬拦下来的结果是**开机直接报
     *   引脚自检失败**，等于把一个"已知、可降级"的限制变成了致命错误。
     *   现在改成：只拦"根本不是 ADC 脚"的接线错误，ADC2 那几路另走
     *   board_pins_adc2_note() 打印一条明确提示（不阻断启动）。 */
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int p = s_combo[i];
        if (p < 0) {
            continue;
        }
        if (p < 1 || p > 20) {
            FAIL("通道%d 的微动脚 GPIO%d 不是 ADC 引脚（S3 的 ADC 只到 "
                 "GPIO1~GPIO20）—— 电阻编码微动必须接在 ADC 上",
                 i + 1, p);
        }
    }
#undef FAIL

    if (ok && err && errlen) {
        err[0] = '\0';
    }
    return ok;
}

/**
 * 把"哪些微动脚落在 ADC2 上"写成一句给现场看的中文提示。
 *
 * ADC2（GPIO11~GPIO20）由 WiFi 射频仲裁，**WiFi 一开就读不到值**
 * （ESP-IDF 文档：adc2 读取在 wifi 启动期间可能返回 ESP_ERR_TIMEOUT）。
 * 本项目 PCB 固定有 3 路落在 ADC2，所以这不是错误，是**必须让人知道**的
 * 工作条件：MQTT（WiFi 开）时那几路降级，485 模式（WiFi 关）时全好。
 *
 * 返回写入字符数；没占到 ADC2 时写空串并返回 0。
 */
static inline int board_pins_adc2_note(char *buf, size_t buflen)
{
    static const int s_combo[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_COMBO;
    int off = 0;

    if (buf == NULL || buflen == 0) {
        return 0;
    }
    buf[0] = '\0';

    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        int p = s_combo[i];
        if (p < 11 || p > 20) {
            continue;
        }
        if (off < (int)buflen - 1) {
            int w = snprintf(buf + off, buflen - off, "%s%d(GPIO%d)",
                             off ? "、" : "", i + 1, p);
            if (w > 0) {
                off += w;
            }
        }
    }
    return off;
}

/**
 * 把完整接线表写进 buf（多行文本），用于上电打印到日志。
 * 返回写入的字符数。
 */
static inline int board_pins_describe(char *buf, size_t buflen)
{
    static const int clutch[BOARD_CHANNEL_COUNT] = BOARD_PIN_CLUTCH;
    static const int s_combo[BOARD_CHANNEL_COUNT] = BOARD_PIN_SENSOR_COMBO;
    const board_spare_pin_t spare[] = BOARD_SPARE_PINS;
    int off = 0;

#define P(...)                                                                 \
    do {                                                                       \
        if (off < (int)buflen - 1) {                                           \
            int _w = snprintf(buf + off, buflen - off, __VA_ARGS__);           \
            if (_w > 0) { off += _w; }                                         \
        }                                                                      \
    } while (0)

    /* -1 = 该角色没有分配引脚。直接按 "GPIO%d" 打印会变成 "GPIO-1"，
     *   看着像固件坏了；统一打成「无」。 */
#define P_PIN(_p)                                                              \
    do {                                                                       \
        if ((_p) < 0) {                                                        \
            P("无");                                                           \
        } else {                                                               \
            P("GPIO%d", (_p));                                                 \
        }                                                                      \
    } while (0)

    P("---- 硬件接线表（ESP32-S3 / 42 针，按 PCB 原理图核对）----\n");
    P("共享电机 : IN1=GPIO%d  IN2=GPIO%d\n", BOARD_PIN_MOTOR_IN1,
      BOARD_PIN_MOTOR_IN2);
#if BOARD_CURRENT_SENSE_ENABLED
    P("电流采样 : ISEN=GPIO%d（采样电阻 %dmΩ，AT8236 输出 %dmV/A）\n",
      BOARD_PIN_CURRENT_SENSE, BOARD_CURRENT_SENSE_R_MOHM,
      BOARD_CURRENT_SENSE_R_MOHM);
#endif
    for (int i = 0; i < BOARD_CHANNEL_COUNT; i++) {
        P("料盘位%d  : 离合=GPIO%d | 微动=", i + 1, clutch[i]);
        P_PIN(s_combo[i]);
        P("（开始4K7 + 停止10K 并在同一脚，ADC 读）\n");
    }
    if (BOARD_EXTRUDER_INPLACE_UNUSED || BOARD_PIN_EXTRUDER_INPLACE < 0) {
        P("挤出机到位: 不接线（改用打印机 MQTT 事件）\n");
    } else {
        P("挤出机到位: GPIO%d\n", BOARD_PIN_EXTRUDER_INPLACE);
    }
    if (BOARD_PIN_LED >= 0) {
        P("状态 LED : GPIO%d\n", BOARD_PIN_LED);
    } else {
        P("状态 LED : 未使用\n");
    }
    if (BOARD_485_ENABLED) {
        P("RS-485   : TX=GPIO%d  RX=GPIO%d  DE=GPIO%d（1.25M 8E1）\n",
          BOARD_PIN_485_TX, BOARD_PIN_485_RX, BOARD_PIN_485_DE);
    } else {
        P("RS-485   : 未启用\n");
    }
    if (BOARD_IN_EXT_UNUSED || BOARD_PIN_IN_EXT < 0) {
        P("补充满足 : 不接线\n");
    } else {
        P("补充满足 : GPIO%d（输入，仅上报状态）\n", BOARD_PIN_IN_EXT);
    }
    P("剩余可用 : ");
    for (size_t i = 0; i < sizeof(spare) / sizeof(spare[0]); i++) {
        P("GPIO%d%s", spare[i].pin,
          i + 1 < sizeof(spare) / sizeof(spare[0]) ? ", " : "\n");
    }
    {
        char adc2[96];
        if (board_pins_adc2_note(adc2, sizeof(adc2)) > 0) {
            P("⚠ ADC2 微动 : 通道 %s 在 ADC2 上 —— WiFi 开启时读不到，"
              "485 模式（WiFi 关）下正常\n", adc2);
        }
    }
#undef P_PIN
#undef P
    return off;
}

#ifdef __cplusplus
}
#endif

#endif /* BOARD_PINS_S3_H */
