/**
 * config.h —— 持久化配置（NVS）
 * ============================================================================
 *
 * 对应 MicroPython 版的 config.json + wifi.dat，换成 ESP-IDF 的 NVS 存储。
 *
 * 为什么换：
 *   · NVS 有掉电保护 —— 写一半掉电不会像 config.json 那样留下半个坏 JSON
 *     （Python 版为此专门加了 ValueError 兜底，这里从根上不需要）
 *   · 没有文件系统，就没有"挂载失败"这一类启动失败
 *   · 自带磨损均衡，频繁改 jog_ms / 当前料盘的场景更安全
 *
 * 存的东西（一个结构体整块存，带 magic + version 校验）：
 *     WiFi 账号密码（最多 3 组，最近用的排前面）
 *     配置热点自身的 SSID / 密码
 *     打印机 MQTT 参数（IP / 序列号 / 访问码 / 用户名 / 端口 / 客户端名）
 *     通道映射 access_list（打印机通道号 → 物理料盘位）
 *     通道颜色 color_list
 *     当前料盘 filament_current
 *     手动点动时长 jog_ms
 *     换料收尾的蠕动参数（次数 / 单次脉冲时长）
 *     微动安装掩码 sensor_enabled_mask（没装微动的通道自动走降级模式）
 *
 * ⚠️ 结构体改字段时**必须**把 CONFIG_VERSION 加一，否则旧 NVS 里的老结构
 *    会被按新结构解析，字段错位。加了版本号之后旧数据会被自动丢弃并回到默认值。
 */

#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "board_pins.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 常量
 * ========================================================================== */
#define CONFIG_NAMESPACE       "ams"
#define CONFIG_KEY             "cfg"

/**
 * ★★ 2026-10-22：通道数 4 → 8，CONFIG_VERSION 从 2 提到 3 ★★
 *
 * 结构体里有三个数组跟着通道数走（access_list / color_list / temper），
 * 8 通道时 sizeof(ams_config_t) 从 688 变成 716 —— **长度变了**，所以
 * config.c 里那道 `len != sizeof(s_cfg)` 校验会拒绝所有老配置。
 *
 * 为了不让现场设备"刷个固件，WiFi 密码 / MQTT 访问码 / 通道映射全没了"，
 * config.c 里配了**迁移逻辑**：读到 688 字节的老 blob 时，按老布局
 * （ams_cfg_v2_t）解读，把 WiFi / MQTT / 前 4 路映射 / 全部动作参数搬进
 * 新结构体，第 5~8 路用默认值，然后再存成新布局。
 *
 * → 老配置**不会丢**，只是多了一次自动转换。见 config.c 的
 *   config_migrate_from_v2()。
 */
#define CONFIG_VERSION         3u
#define CONFIG_MAGIC           0x414D5331u  /* "AMS1" */

/** 老版本（4 通道）配置的字节数，迁移时用来识别 blob */
#define CONFIG_LEGACY_V2_SIZE  688

/* 一次性配置补丁的水位线（见 config_apply_patch_once）。
 *
 * 为什么需要它：本次新增的几个字段（蠕动时长 / 间隔 / 轮数 / 辅助送料时长）
 * 是从 reserved 里抠出来的，老固件写下的那几字节恒为 0。光靠"读到 0 就用
 * 默认值"分不清"老配置没设置过"和"用户就是选了 0" —— 而 0 轮蠕动是合法
 * 设置。所以单独用一个 NVS 键当标记：没有这个键 = 打补丁前的配置，
 * 补一次并写下水位线；有键就完全信任存下来的值。 */
#define CONFIG_KEY_PATCH       "cfgpatch"
/*   3：2026-10-23 把「485 运动执行」从关改为开（运动语义已对齐 Tianshu） */
#define CONFIG_PATCH_LEVEL     3

/**
 * 通信方式（2026-10-22 新增）：MQTT 还是 RS-485。
 *
 * ★ 为什么**单独存一个 NVS 键**，而不是塞进 ams_config_t：
 *   ams_config_t 的 688 字节已经被 reserved 吃干（见本文件末尾的 _Static_assert），
 *   再加字段就必须升 CONFIG_VERSION —— 而升版本的代价是
 *   **用户设备上的 WiFi 密码 / MQTT 访问码 / 通道映射整份被重置**。
 *   为了一个 1 字节的枚举付这个代价值不值？不值。所以和 cfgpatch 一样，
 *   单独占一个键：老固件没有这个键 → 读不到 → 用默认值 MQTT，
 *   升级过来的人不会被改到任何东西。
 */
#define CONFIG_KEY_COMM        "comm"

/**
 * "通信方式迁移已完成" 的标记键（1 字节）。
 *
 * 用途：老固件里存的是 comm=1（手动 RS-485），这版要把它升级成 2（自动）。
 * 迁移只做一次，之后用户手动改回 1 不会被再次覆盖 —— 就靠这个标记。
 */
#define CONFIG_KEY_COMM_MIG    "comm_mig"

/**
 * "WiFi 上电是否启动" 的独立键（1 字节，**默认 1 = 开**）。
 *
 * ★ 为什么单独一个键、而且默认是"开"：
 *   AUTO 模式为了保住 ADC2（微动 6/7/8 = GPIO11/12/13）开机故意不开 WiFi，
 *   可打印机的总线枚举（0x05 广播 → 0x01 点名 → 0x402/0x103 → 0x211）
 *   **只在上电后那一小会儿**做完。等 2 分钟超时 WiFi 起来，握手早结束了，
 *   现场等于看不到任何 485 通信 —— "刷了固件有没有效"根本无法验证。
 *
 *   所以默认改成"上电就开 WiFi"，先把链路调通；调试完在网页上关掉它，
 *   就恢复原来的上电静默逻辑（微动 6/7/8 也随之恢复可读）。
 *
 * ★ 关掉 WiFi 后那三个微动能不能用：**能**。ADC2 与射频共用前端，
 *   `esp_wifi_stop()` 之后 `adc_oneshot_read()` 立即恢复正常（这是
 *   ESP32 的既有行为，不是我们的补丁）。WiFi 开着就一定读不到，无解。
 */
#define CONFIG_KEY_WIFI_ON     "wifi_on"

/**
 * "485 运动指令是否真的驱动电机" 的独立键（1 字节，**默认 0 = 关**）。
 *
 * 关：收到打印机的进料/退料指令只回"空闲、未动作"，电机不转 —— 安全，
 *     但打印机点了没反应（会等到超时）。
 * 开：按 statu_flags/motion 映射到本机 8 通道机构真进退料。
 *
 * ★ 默认关的理由：485 路径上的通道映射、电机方向还没在真机上完整验证；
 *   一旦映射错了，打印机会以为在进料而机构在退料，可能把料顶断。
 *   确认通道号对得上之后再在网页上打开。
 */
#define CONFIG_KEY_485_ACT     "a485act"


#define CONFIG_WIFI_MAX_PROFILES 3
#define CONFIG_SSID_MAX          33   /* 含结尾 '\0' */
#define CONFIG_PASS_MAX          65   /* 含结尾 '\0' */
#define CONFIG_HOST_MAX          64
#define CONFIG_SERIAL_MAX        32
#define CONFIG_CLIENTID_MAX      32

/* 默认配置热点 —— 和 MicroPython 版保持一致，老用户不用重新记 */
#define CONFIG_DEFAULT_AP_SSID     "AMS_WIFI"
#define CONFIG_DEFAULT_AP_PASS     "A12345678"
#define CONFIG_DEFAULT_AP_CHANNEL  6

/* 手动点动时长的安全区间（毫秒）。网页填超范围会被夹住。
 *   下限：再短离合还没咬合就停了，没意义
 *   上限：防止手滑把关在机构里的料顶坏
 */
#define CONFIG_JOG_MIN_MS 200
#define CONFIG_JOG_MAX_MS 60000
#define CONFIG_JOG_DEF_MS 1000

/* 手动点动的 PWM 占空比（%）。
 *
 * ★ 2026-09-23 现场要求原话："在这里加一个 PWM 值的设置，我来看看目测一下
 *   多少值可以转动电机和电磁。"
 *
 * 为什么值得单开一个参数：手动点动原来写死全速（duty 255），于是它只能
 * 回答"通不通"这一个问题。现场真正想知道的是"**能带动的临界占空比是多少**"
 * —— 这个数只能一档一档试出来：从 5% 往上加，看电机什么时候能可靠转起来、
 * 离合咬合时什么时候不丢步。知道临界值之后，辅助送料/蠕动才有依据在
 * 临界值之上留余量（默认值 100% 就是这么来的：宁可有力，不要没力）。
 *
 * 这个值**只作用于手动点动**，不影响自动换料（换料走的是全速）。
 * 0 = 未设置（老配置的字节就是 0）→ 按 CONFIG_JOG_SPEED_DEF 处理。
 */
#define CONFIG_JOG_SPEED_DEF 100
#define CONFIG_JOG_SPEED_MIN 5
#define CONFIG_JOG_SPEED_MAX 100

/* 换料收尾的"蠕动送料"参数
 *   挤出机到位信号来了之后，慢速短脉冲推 CREEP_TIMES 次，确保耗材被挤出机
 *   齿轮真正咬住，而不是虚顶在入口。
 */
#define CONFIG_CREEP_TIMES_DEF     3     /* 你要求的是三次 */
#define CONFIG_CREEP_TIMES_MIN     1
#define CONFIG_CREEP_TIMES_MAX     10
#define CONFIG_CREEP_PULSE_MS_DEF  400   /* 单次脉冲时长 */
#define CONFIG_CREEP_PULSE_MIN     100
#define CONFIG_CREEP_PULSE_MAX     3000
/* 蠕动速度（PWM 占空比百分比）。
 *
 * ★ 2026-09-22 真机定论：45% 在这个机构上**完全带不动电机** —— 实机表现为
 *   "听不到电机响、料一动不动"。证据：同一轮换色里 12 轮 × 2000ms 的 45%
 *   蠕动（合计 24 秒）一根料都没拉动，紧接着 100% 的连续退料 5 秒就把料
 *   拉出挤出机了（打印机 hw_switch_state 同一秒从 1 变 0）。
 *   所以默认值从 45 提到 70。低于 CONFIG_PWM_FUTILE_PCT 的值基本等于不转。 */
#define CONFIG_CREEP_SPEED_PCT_DEF 70    /* 蠕动速度（PWM 占空比百分比） */

/* 占空比"带不动电机"的经验阈值：低于它，电机只会堵转（听不到声、不转）。
 * 只用于**一次性迁移**老配置，不参与运行期夹取 —— 运行期用户填多少就是多少，
 * 免得把用户故意调低的值悄悄改掉。 */
#define CONFIG_PWM_FUTILE_PCT      60

/* 辅助送料参数（换色时同步推料帮打印机咬住新料 / 打印中辅助送料）
 *
 * ★ 2026-09-23 真机定论：默认值 60% **仍然带不动本机构的电机**。
 *   run7 整轮打印的日志里，辅助送料的回执每条都是
 *       "离合已吸合 —— 通道N PWM duty=153/255（60%），通电 1500ms"
 *   也就是说：离合真的吸合了、占空比真的写到 153 了，**但料就是不往前走**。
 *   同一轮里同一颗电机跑 100%（duty 255）的连续退料/闭环送料都正常。
 *   60% 正好压在 CONFIG_PWM_FUTILE_PCT 这条"堵转线"上 —— 阈值写 60、
 *   默认值也写 60，等于默认值就坐在悬崖边上。
 *   所以默认值提到 100：辅助送料是"帮一把"，宁可有力，不要没力。 */
#define CONFIG_ASSIST_PCT_DEF      100  /* 辅助送料 PWM 占空比百分比 */
#define CONFIG_ASSIST_PCT_MIN      5
#define CONFIG_ASSIST_PCT_MAX      100
#define CONFIG_ASSIST_MS_DEF       1500 /* 每次辅助送料的持续时长 */
#define CONFIG_ASSIST_MS_MIN       200
#define CONFIG_ASSIST_MS_MAX       10000

/* 退料参数（C3 无微动降级模式）—— 全是网页「硬件调试」里可改的现场参数 */
#define CONFIG_RETRACT_WAIT_MS_DEF  5000  /* 蠕动退料后等待挤出机 MQTT 信号的最长时间 */
#define CONFIG_RETRACT_WAIT_MIN     1000
#define CONFIG_RETRACT_WAIT_MAX     15000
/* 连续退料时长。★ 现在它是**第一动作**（不再是蠕动之后的收尾）：
 * 打印机一报 ams_status=260（挤出机已跑回冲刷区），我们立刻全速拉料，
 * 所以这个值同时是"全速拉料的上限时间"。拉到打印机报"无料"就提前停。 */
#define CONFIG_RETRACT_CONT_MS_DEF  6000
#define CONFIG_RETRACT_CONT_MIN     1000
#define CONFIG_RETRACT_CONT_MAX     30000
/* 蠕动退料：连续退料没把料拉出来时，再用慢速短脉冲拱几次试试 */
#define CONFIG_RETRACT_CREEP_MS_DEF  2000  /* 单次蠕动退料时长 */
#define CONFIG_RETRACT_CREEP_MS_MIN   100
#define CONFIG_RETRACT_CREEP_MS_MAX 10000
#define CONFIG_RETRACT_GAP_MS_DEF    1000  /* 两次蠕动之间等"挤出机已空"的间隔 */
#define CONFIG_RETRACT_GAP_MS_MIN     100
#define CONFIG_RETRACT_GAP_MS_MAX    5000
#define CONFIG_RETRACT_CREEP_MAX_DEF    3  /* 蠕动最多几轮；0 = 不蠕动 */
#define CONFIG_RETRACT_CREEP_MAX_MAX   20

/* 换料温度（每个料盘位一个，单位 ℃）
 *   退料前把热端升到这个温度：太低，热端里的料是硬的，退不出来；
 *   太高，PLA 之类会烤糊、拉丝。
 *   按材料给：PLA 220 / PETG 250 / ABS 260 / TPU 230 左右。
 *   没配过（NVS 里是 0）时用默认值 —— 这样旧固件升上来直接可用。
 */
#define CONFIG_TEMPER_DEF           250
#define CONFIG_TEMPER_MIN           150   /* 低于这个温度没有材料能退出来 */
#define CONFIG_TEMPER_MAX           300   /* 再高就超过 A1 热端上限了 */

/* ==========================================================================
 * 数据类型
 * ========================================================================== */

/** 一组保存过的 WiFi */
typedef struct {
    char ssid[CONFIG_SSID_MAX];
    char pass[CONFIG_PASS_MAX];
} config_wifi_profile_t;

/** 「挤出机到位信号」从哪里来 */
typedef enum {
    EXTRUDER_SRC_GPIO = 0,  /**< 走 GPIO1 那根线（默认） */
    EXTRUDER_SRC_MQTT = 1,  /**< 走打印机上报的 MQTT 事件 */
} config_extruder_src_t;

/**
 * 和打印机通信的方式。任何时刻只跑一条通路（两条同时开 = 两份指令抢同一个
 * 状态机，一条说送料、另一条说退料，电机听谁的？）。
 *
 *   MQTT  —— 我们主动连打印机的 MQTT Broker，解析 G-code 信令
 *   485   —— 我们扮演真 AMS，挂在 BambuBus 上；**WiFi 保持开启**，
 *            所以网页随时能访问（调试用）。代价：ADC2 被射频占用，
 *            微动 6/7/8（GPIO11/12/13）读不到 → 那三路降级。
 *   AUTO  —— **默认**。开机先试 485 且**不开 WiFi**（这样 8 路微动全可读、
 *            也省电）；2 分钟内收到打印机任何一帧就保持 485，否则自动
 *            开 WiFi 转 MQTT。**只在开机时判定一次**，运行期不来回切。
 *
 * 切换方式：网页「硬件调试」里点一下，或者调用 config_set_comm_mode()。
 * 切换后**必须重启**才生效。
 */
typedef enum {
    COMM_MODE_MQTT = 0,
    COMM_MODE_485  = 1,   /**< 手动 485：保留 WiFi，网页可访问 */
    COMM_MODE_AUTO = 2,   /**< 默认：485 优先（关 WiFi）→ 超时切 MQTT */
    /**
     * ★ 2026-10-22 深夜新增：485 + WiFi **同时**，且**不起 MQTT**。
     *
     * 存在的唯一理由：AUTO 模式下开机 2 分钟没有 WiFi，而打印机的总线
     * 枚举恰好只在上电那一会儿做 —— 现场根本看不到握手过程。
     * 这个模式让 WiFi 开机就起来（网页/日志随时可看），485 照常跑，
     * 并且**不启动 MQTT**（避免两条通路同时下指令打架）。
     *
     * 代价是硬的：ADC2（GPIO11~20）与 WiFi 共用射频，微动 6/7/8
     * （GPIO11/12/13）读不到 → 那三路降级。所以它只用于**调试**，
     * 生产形态仍是 AUTO（485 通了就不开 WiFi，8 路全可读）。
     */
    COMM_MODE_485_DBG = 3,
} config_comm_mode_t;

/** 完整配置。整块存进 NVS，所以字段顺序不能随便调。 */
typedef struct {
    uint32_t magic;
    uint32_t version;

    /* ---- WiFi ---- */
    config_wifi_profile_t profiles[CONFIG_WIFI_MAX_PROFILES];
    uint8_t  profile_count;                 /* 有效项个数，0 表示没配过 */
    char     ap_ssid[CONFIG_SSID_MAX];
    char     ap_pass[CONFIG_PASS_MAX];
    uint8_t  ap_channel;

    /* ---- 打印机 MQTT ---- */
    char     mqtt_host[CONFIG_HOST_MAX];
    char     mqtt_serial[CONFIG_SERIAL_MAX];
    char     mqtt_user[CONFIG_SERIAL_MAX];
    char     mqtt_pass[CONFIG_PASS_MAX];
    char     mqtt_client_id[CONFIG_CLIENTID_MAX];
    uint16_t mqtt_port;

    /* ---- 业务 ---- */
    uint8_t  access_list[BOARD_CHANNEL_COUNT];   /* 物理料盘位 -> 打印机通道号 */
    uint32_t color_list[BOARD_CHANNEL_COUNT];    /* 0xRRGGBB */
    int8_t   filament_current;                   /* 打印机通道号，0 = 未知 */

    /* ---- 动作参数（网页可改） ---- */
    uint16_t jog_ms;
    uint8_t  creep_times;
    uint16_t creep_pulse_ms;
    uint8_t  creep_speed_pct;
    uint8_t  extruder_src;                       /* config_extruder_src_t */

    /* ---- 辅助送料（换色时同步推料帮打印机咬住新料） ---- */
    uint8_t  assist_enabled;                     /* 0=关 1=开（默认开） */
    uint8_t  assist_speed_pct;                   /* PWM 占空比百分比 */

    /* ---- 退料参数（C3 无微动降级模式） ---- */
    uint16_t retract_wait_ms;                    /* 蠕动退料后等 MQTT 信号的最长时间 */
    uint16_t retract_cont_ms;                    /* 收到"没料"信号后连续退料时长 */

    /* ---- 微动安装情况：bit i = 1 表示料盘位 i+1 装了微动组 ---- */
    uint8_t  sensor_enabled_mask;

    /* ---- 换料温度：每个料盘位一个（℃），0 = 用 CONFIG_TEMPER_DEF ----
     * 放在原来 reserved 的位置，**结构体总长不变**，所以 NVS 里存的老
     * 数据读进来不会错位 —— 这是特意不升 CONFIG_VERSION 的原因：
     * 升版本号会把用户的 WiFi / MQTT / 通道映射全部清掉，而这里完全
     * 没必要。老固件里 reserved 恒为 0（只有 memset 0，没有别的写入），
     * 因此旧配置读出来 temper 全是 0，自动落回默认值，行为不变。
     * ⚠️ 以后动这个结构体，如果**改变了总长度**，那就必须升版本号。 */
    uint16_t temper[BOARD_CHANNEL_COUNT];

    /* ---- 退料 / 辅助送料时长（2026-09-22 新增，网页「硬件调试」可改）----
     *
     * ★ 这几个字段是**从原来的 reserved 里抠出来的**，不是插在结构体中间 ——
     *   插中间会让 sensor_enabled_mask / temper 全部后移，NVS 里的老数据就
     *   按错位的偏移读出来（temper 会读成乱值）。放在这里：所有老字段的
     *   偏移不变、结构体总长也不变，所以**不需要升 CONFIG_VERSION**
     *   （升版本号会把用户的 WiFi / MQTT / 通道映射全清掉）。
     *
     * ⚠️ 老固件写下的这几个字节是 0（reserved 当初 memset 0，从没被写过）。
     *   所以读到 0 一律当"没设置过"，落回 CONFIG_*_DEF 默认值。
     */
    uint16_t retract_creep_ms;     /* 蠕动退料：单次脉冲时长 */
    uint16_t retract_gap_ms;       /* 两次蠕动之间等"挤出机已空"的间隔 */
    uint16_t assist_ms;            /* 每次辅助送料的持续时长 */
    uint8_t  retract_creep_max;    /* 蠕动最多几轮；0 = 不蠕动（只做连续退料） */

    /* ---- 2026-09-23 新增（网页「手动点动」/「辅助送料」可改）----
     *
     * ★★ 这两个字段是**吃掉结构体尾部的对齐填充**塞进来的，不是把整体加长 ★★
     *
     *   本次改动前，结构体的尾部是（偏移由 tools/check_config_layout.py 实测）：
     *       ... retract_creep_max[684] + reserved[685] + 对齐填充[686,687]
     *   而这两个新字段放的正是"reserved + 那 2 字节填充"里的前两个字节：
     *       ... retract_creep_max[684] + jog_speed_pct[685] + assist_hold[686]
     *       + reserved[687]
     *   结构体总长因而**保持 688 字节不变** —— 这一点必须牢牢守住，
     *   因为 config.c 读 NVS 时有 `len != sizeof(s_cfg)` 的硬校验：
     *   长度一变，用户设备上存的 WiFi 密码 / MQTT 访问码 / 通道映射会被
     *   **整份丢弃**（现场表现就是"刷个固件，配置全没了"）。
     *
     *   验证方式（本机没编译器，只能这么验）：
     *       python tools/check_config_layout.py
     *   它按 C ABI 把新旧两个布局都建出来，断言 sizeof 不变、新字段落在旧
     *   填充上、老字段偏移一个都没动。CI 编译时下面那条 _Static_assert
     *   还会再兜一次。
     *
     * ⚠️ 老固件写下的这两字节是 0（config_load_defaults 里 memset 0，
     *   reserved 从没被写过、填充字节从没被赋过值），所以：
     *     · jog_speed_pct = 0 → config_get_jog_speed_pct() 按"未设置"处理，
     *       返回默认全速 100%，行为不变；
     *     · assist_hold   = 0 → 会被当成"关闭保持吸合"，但需求是开启，
     *       所以由配置补丁 2（见 config.c 的 config_apply_patch_once）
     *       显式置成 1。这也是 CONFIG_PATCH_LEVEL 从 1 提到 2 的原因。 */
    uint8_t  jog_speed_pct;        /* 手动点动 PWM 占空比%；0 = 未设置（默认 100） */
    uint8_t  assist_hold;          /* 1 = 辅助送料阶段内保持离合吸合、只脉冲电机 */

    /* ---- 原「自吸」使能掩码的占位字节（功能已删除，2026-10-23）----------
     *
     * 这一字节是 ams_config_t 的**最后一字节**（4 通道时偏移 687、8 通道 715）。
     * 「自吸上料」功能已按现场要求整套删除，但**字段不能删**：
     *   · 删了 sizeof(ams_config_t) 就变短 → config_load() 的
     *     `len != sizeof(s_cfg)` 会把老 NVS 配置整份丢掉；
     *   · 想保住数据就得升 CONFIG_VERSION 并写迁移，代价是把用户设备上的
     *     WiFi / MQTT 访问码 / 通道映射全清一遍 —— 明显不划算。
     * 所以改名保留占位，长度一个字节都不动，`_Static_assert` 照样过。
     * 以后要加一个"不用升版本"的小字段，直接用它。 */
    uint8_t  reserved_al;
} ams_config_t;

/* ★★ 结构体长度必须和板型的通道数对上 ★★
 *
 * 这条断言是最后一道闸门：长度一旦变了，config.c 的 `len != sizeof(s_cfg)`
 * 就会把老 NVS 配置整份丢掉。所以**改通道数必须同时做两件事**：
 *   1. 升 CONFIG_VERSION（本次 2 → 3）
 *   2. 在 config.c 里写一份迁移（本次 = config_migrate_from_v2）
 *
 * 实测长度（32 位 ABI、uint8/16/32 自然对齐）：
 *     4 通道 → 688    （C3，脚不够扩）
 *     8 通道 → 716    （S3，按现场 IO 表）
 * 多出来的 28 字节 = access_list +4、color_list +16、temper +8。
 */
#if BOARD_CHANNEL_COUNT == 4
_Static_assert(sizeof(ams_config_t) == 688,
               "4 通道的 ams_config_t 应为 688 字节；改了字段就必须升 "
               "CONFIG_VERSION 并写迁移代码");
#elif BOARD_CHANNEL_COUNT == 8
_Static_assert(sizeof(ams_config_t) == 716,
               "8 通道的 ams_config_t 应为 716 字节；改了字段就必须升 "
               "CONFIG_VERSION 并写迁移代码");
#else
#error "换了新的通道数：先核算 ams_config_t 长度，再补一条 _Static_assert 和对应迁移"
#endif

/* ==========================================================================
 * 生命周期
 * ========================================================================== */

/**
 * 初始化配置模块：挂载 NVS，读出配置；读不到或版本不符就写入一份默认值。
 *
 * 必须在所有其它模块之前调用（wifi_manager / mqtt_client / web_server 都要读它）。
 * 返回 ESP_OK 表示至少拿到了一份可用的默认配置；NVS 彻底坏掉时返回错误，
 * 但**配置对象依然可用**（内容是默认值），调用方可以选择继续跑。
 */
esp_err_t config_init(void);

/** 取配置对象（只读指针，全局唯一，永不为 NULL） */
ams_config_t *config_get(void);

/** 把当前配置写回 NVS。改完任何字段都要调一次。 */
esp_err_t config_save(void);

/** 恢复出厂：字段全部回到默认值并写回 NVS（不动 WiFi 记录，避免被困在配网模式） */
esp_err_t config_reset_business(void);

/* ==========================================================================
 * 便捷访问器 —— 业务代码用这些，不要直接摸结构体字段
 * ========================================================================== */

/** 打印机参数是否齐全（IP / 序列号 / 访问码三项都有） */
bool config_mqtt_ready(void);

/** 当前料盘（打印机通道号，0 = 未知） */
int config_get_filament_current(void);
void config_set_filament_current(int printer_channel);

/** 手动点动时长（毫秒，已夹到安全区间） */
uint16_t config_get_jog_ms(void);

/**
 * 设置手动点动时长。value 单位毫秒，会被夹到 [CONFIG_JOG_MIN_MS, CONFIG_JOG_MAX_MS]。
 * 返回**实际生效**的值（可能和传入值不同），网页据此显示准确提示。
 */
uint16_t config_set_jog_ms(int value);

/**
 * 手动点动的 PWM 占空比（%）。
 *
 * 用途只有一个：**现场目测"能带动电机和离合的临界占空比"**。从 5% 一档档
 * 往上加，看电机什么时候能可靠转起来 —— 找到临界值后再给辅助送料/蠕动
 * 留余量（它们默认都是 100%）。
 *
 * 0 是"未设置"（老配置那个字节就是 0）→ 返回 CONFIG_JOG_SPEED_DEF（全速）。
 */
uint8_t config_get_jog_speed_pct(void);

/** 设置手动点动 PWM 占空比（%），返回**实际存进去**的值（已夹到区间） */
uint8_t config_set_jog_speed_pct(int pct);

/**
 * 辅助送料是否"阶段内保持离合吸合"（1 = 是，默认）。
 *
 * 开了之后，在校准 / 打印中这些需要辅助送料的阶段里，离合**全程吸着**，
 * 只有电机在按周期脉冲（转 assist_ms → 停 → 再转），不再每送一次就
 * "吸合—断开"折腾一遍机械。离开这些阶段立刻断开。
 */
uint8_t config_get_assist_hold(void);
void config_set_assist_hold(uint8_t on);

/* ==========================================================================
 * 通信方式（MQTT / RS-485）
 * ========================================================================== */

/** 当前选择的通信方式（COMM_MODE_MQTT / COMM_MODE_485），启动时读一次 */
uint8_t config_get_comm_mode(void);

/**
 * 改通信方式并落盘。**重启后生效**。
 *
 * @return 实际存进去的值（非法入参夹到 MQTT）
 *
 * ★ 为什么不允许"立刻生效"：MQTT 客户端是 esp-mqtt 的任务，485 是 UART
 *   接收任务，两者同时存在时"谁在解读打印机的意图"就有两份答案 ——
 *   送料逻辑只有一个状态机，两条通路同时投命令会互相踩。
 *   所以切换只写配置，由重启把另一条彻底关掉。
 */
uint8_t config_set_comm_mode(int mode);

/** 该通信方式的中文名（给网页/日志用），不认识的值返回"未知" */
const char *config_comm_mode_text(uint8_t mode);

/**
 * WiFi 上电是否启动（默认 true）。
 *
 * 关掉它 → 上电不启射频 → ADC2（微动 6/7/8，GPIO11/12/13）可读，
 * 但网页也打不开，只能靠串口/OLED 看状态。
 */
bool config_get_wifi_on(void);

/**
 * 改「WiFi 上电开关」并落盘，**重启后生效**。
 *
 * ★ 为什么要重启生效：射频已经起来的情况下 `esp_wifi_stop()` 再
 *   `esp_wifi_start()`，httpd 的监听 socket 不会自动恢复，网页会变成
 *   "能 ping 通但打不开"。重启是最干净的路径。
 */
bool config_set_wifi_on(bool on);

/**
 * 485 收到运动指令时是否真的驱动电机（**默认 true**，见 CONFIG_KEY_485_ACT）。
 *
 * ★ 2026-10-23 从默认关闭改成默认开启：运动语义（flags=0x03/motion=0x00 是
 *   send_out 正转进料）已经和 Tianshu 逐条对齐，"点进料电机不转"的直接原因
 *   就是这个开关关着。关掉时仍会正常回状态帧，只是不驱动机 —— 单独验证总线时用。
 */
bool config_get_485_actuate(void);

/** 改「485 运动执行」并落盘。**立即生效**（不涉及射频，不用重启） */
bool config_set_485_actuate(bool on);

/**
 * 某一路的「缓冲送料」是否开启（原「微动已启用」）。
 * 开启后：开始送料微动触发 → 送料；送 / 退料途中停止送料微动触发 → 立刻停。
 *
 * ★ 2026-10-23：「自吸」功能已整套删除，现在这一路只有**这一个**微动开关。
 */
bool config_sensor_enabled(int channel_index);

/** 物理料盘位（0 起）对应哪个打印机通道号（1 起）；未映射时返回 -1 */
int config_printer_channel_of(int material_index);

/** 打印机通道号（1 起）对应哪个物理料盘位（0 起）；未映射时返回 -1 */
int config_material_index_of(int printer_channel);

/** 写一组 WiFi 并把它排到最前面 */
esp_err_t config_set_wifi(const char *ssid, const char *pass);

/** 改配置热点的 SSID / 密码（password 传 NULL 或 "" 表示保持原值） */
esp_err_t config_set_ap(const char *ssid, const char *pass);

/** 改打印机 MQTT 参数（任何一项传 NULL 表示保持原值） */
esp_err_t config_set_mqtt(const char *host, const char *serial,
                          const char *user, const char *pass,
                          int port, const char *client_id);

/** 改通道映射与颜色（两个数组都是 BOARD_CHANNEL_COUNT 长） */
esp_err_t config_set_access(const uint8_t *access_list,
                            const uint32_t *color_list);

/**
 * 取某个物理料盘位（0 起）的颜色（0xRRGGBB）。
 * 用于 4 通道自动颜色匹配：拿切片器的目标颜色和所有通道做距离比较。
 */
uint32_t config_get_color(int material_index);

/**
 * 自动颜色匹配：从 4 个通道里挑出和 target_rgb 最接近的料盘位。
 *
 * 距离度量用感知加权欧氏距离（R×0.30 + G×0.59 + B×0.11 的加权平方和），
 * 比朴素 RGB 距离更贴近人眼对色差的主观感受。
 *
 * @param target_rgb  目标颜色 0xRRGGBB
 * @param out_material 输出：最接近通道的物理料盘位下标（0 起）
 * @return 距离值（越小越接近），UINT32_MAX 表示所有通道都是 0（未配颜色）
 */
uint32_t config_color_match(uint32_t target_rgb, int *out_material);

/** 把配置整理成一段人类可读的文本，打日志用 */
int config_describe(char *buf, size_t buflen);

/** 辅助送料 PWM 占空比百分比（已夹到安全区间） */
uint8_t config_get_assist_speed_pct(void);
void config_set_assist_speed_pct(uint8_t pct);

/** 辅助送料开关（0=关 1=开） */
uint8_t config_get_assist_enabled(void);
void config_set_assist_enabled(uint8_t on);

/** 每次辅助送料的持续时长（毫秒，已夹到安全区间） */
uint16_t config_get_assist_ms(void);
uint16_t config_set_assist_ms(int value);

/** 退料参数获取/设置（已夹到安全区间） */
uint16_t config_get_retract_wait_ms(void);
uint16_t config_set_retract_wait_ms(int value);
uint16_t config_get_retract_cont_ms(void);
uint16_t config_set_retract_cont_ms(int value);

/** 蠕动退料：单次脉冲时长（毫秒） */
uint16_t config_get_retract_creep_ms(void);
uint16_t config_set_retract_creep_ms(int value);

/** 两次蠕动之间等"挤出机已空"信号的间隔（毫秒） */
uint16_t config_get_retract_gap_ms(void);
uint16_t config_set_retract_gap_ms(int value);

/** 蠕动最多几轮（0 = 不蠕动，只做连续退料） */
uint8_t config_get_retract_creep_max(void);
uint8_t config_set_retract_creep_max(int value);

/** 某料盘位的换料温度（℃）。参数越界或没配过 → CONFIG_TEMPER_DEF */
int config_get_temper(int material_index);
/** 设置某料盘位的换料温度（℃），自动夹到 [MIN, MAX] 并写回 NVS */
void config_set_temper(int material_index, int value);

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
