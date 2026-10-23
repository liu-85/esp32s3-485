/**
 * bambu_proto.h —— 拓竹打印机 MQTT 报文协议
 * ============================================================================
 *
 * 对应 MicroPython 版的 bambu/get_event_info.py + bambu/bambu_const.py
 * + bambu/bambu_commands.py，三块合成一个模块。
 *
 * 单独抽出来放的原因：mqtt_client.c 只管"连接 / 收发 / 重连"，报文怎么解析、
 * 命令长什么样，全都归这里。换打印机固件版本时只需要动这一个文件。
 *
 * ----------------------------------------------------------------------------
 * 一、换料是怎么被触发的（这是本项目最核心的一个判定）
 * ----------------------------------------------------------------------------
 *   一共有三条触发通路。**前提是打印机已暂停**（gcode_state = "PAUSE"），
 *   在这个前提下三条任意一条成立即触发（见 parse 末尾的判定）：
 *
 *   ① 热床温度信道（★ 本项目实际使用的主力通路）
 *      G-code 换料宏里写 `M140 S{next_extruder + 1};EXT` —— 把"热床目标
 *      温度"直接改写成通道号。于是打印机的状态上报里：
 *
 *          print.bed_target_temper = 1 ~ 16      ← 这是通道号，不是温度
 *
 *      现实中没有哪个打印任务会把床温设成 4℃，所以只要读到 (0,17) 范围内
 *      的 bed_target_temper，就是切片在告诉我们"换到第 N 通道"。
 *      这个约定的好处：不依赖 mc_percent / ams.stage 这些不同固件版本
 *      会变的字段，A1 / P1 系列都能用（Top-AMS 就是这么做的）。
 *
 *      ⚠️ 但**不能只看床温就动手**：M140 写在换料宏很靠前的位置（A1 的宏
 *         里是第 6 行），真正让打印机停下来的 M400 U1 在第 25 行，中间还
 *         隔着抬 Z、移动喷头等十几行。三条通路统一要求 is_paused 就是为了
 *         这个 —— 否则会在喷头还在移动时去切刀、退料，时序全乱。
 *         好在床温信令会持续上报到我们发 M190 改回去为止，所以等 PAUSE
 *         那一帧不会漏。
 *
 *      ⚠️ 借了热床的温度**必须还** —— 否则热床真的会按 M140 的设定降到
 *         1~4℃，首层直接粘不住。恢复逻辑在 ams_controller.c 的
 *         bed_target_max 那一段（用 M190 发回去）。
 *
 *   ② 官方宏：`M73 P101 R[next_extruder]`
 *      打印机会把 mc_percent 置成 101 并暂停（gcode_state = "PAUSE"）。
 *      判定条件 `is_paused && mc_percent == 101`。
 *
 *   ③ 第三方宏：`M400 U1`（暂停等 AMS）
 *      部分固件会把 ams.stage 置成 1。判定 `is_paused && ams_stage == 1`。
 *
 *   目标通道号（**0 起**，用的时候要注意 +1）优先取通路 ① 的
 *   bed_channel - 1；没有信道时才退回探测 M73 P101 系列回传的字段
 *   （filament_next / mc_next_tray 等），最后才用 mc_remaining_time 兜底。
 *   ⚠️ mc_remaining_time 名义上是"剩余打印时间（分钟）"，不是通道号，
 *   旧版曾拿它当通道号用，4 通道自动换料几乎永远不触发。
 *
 * ----------------------------------------------------------------------------
 * 二、命令（发出去的那些 JSON）
 * ----------------------------------------------------------------------------
 *   全部在文件末尾的 bambu_cmd_* 函数里，返回值是要发送的字节数。
 *   和 Python 版的 bambu_commands.py 一一对应，连 sequence_id 都保持一致。
 */

#ifndef BAMBU_PROTO_H
#define BAMBU_PROTO_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/* ==========================================================================
 * 报文解析结果
 * ========================================================================== */

/** HMS 错误条目（最多同时留这么多条，够显示又不会撑爆栈） */
#define BAMBU_HMS_MAX      4
#define BAMBU_HMS_CODE_LEN 24
#define BAMBU_HMS_TEXT_LEN 72

typedef struct {
    char code[BAMBU_HMS_CODE_LEN];   /**< 形如 "0300_4000_0002_0001" */
    char text[BAMBU_HMS_TEXT_LEN];   /**< 中文说明；查不到表时是 "未知错误" */
} bambu_hms_t;

/** 一次推送解析出来的全部信息 */
typedef struct {
    /* ---- 换料判定 ---- */
    bool    change_needed;      /**< 需要 AMS 执行换料 */
    int     filament_next;      /**< 目标通道号，**0 起**；不需要换料时 -1 */
    int     target_color;       /**< 目标颜色 0xRRGGBB；-1 = 报文里没有 */

    /* ---- 打印机状态 ---- */
    int     gcode_state;        /**< 0=空闲 1=准备 2=打印中 3=暂停 ... */
    int     stg_cur;            /**< 当前阶段号，查表得文字 */
    const char *stage_text;     /**< 阶段中文说明（指向静态表，不要 free） */
    int     mc_percent;         /**< 打印进度百分比 */
    int     mc_remaining_time;  /**< 剩余时间（分钟） */
    /**
     * 当前层号。★ 用来判"这个任务是不是真的推进过"（见 ams_controller.c
     * 辅助送料里 stg0_is_really_printing 的说明）：刚下发打印任务、打印机
     * 还在自检时 layer_num 是 0，stg_cur 可能还残留着上一次的 0（"打印中"）。
     */
    int     layer_num;
    int     print_error;        /**< 打印错误码，0 = 无 */
    int     ams_stage;          /**< ams.stage，1 = 等待 AMS 完成换料；-1 = 无此字段 */
    bool    is_paused;
    bool    is_printing;

    /* ---- 温度 ---- */
    float   nozzle_temper;
    float   nozzle_target;
    float   bed_temper;
    float   bed_target;
    /**
     * ★ 打印机发的是**增量报文**：某个字段只有"值变了"的那一帧才带，
     *   其余帧里这个键**根本不存在**。
     *
     *   "没带"和"值为 0"是两回事：前者必须保持上一次记忆不动，
     *   后者才代表热床真的关了（打印结束）。
     *
     *   2026-09-22 真机事故：解析层把缺失的 bed_target_temper 当 0 返回，
     *   上层"< 17 就清记忆"的逻辑于是被每一帧增量报文触发 ——
     *   M140 S76 刚记住就被抹掉，换色时报告"没有热床温度的历史记录"，
     *   M190 发不出去，热床被 M140 S{next+1} 留在 3℃ 凉了一整晚。
     */
    bool    has_bed_target;

    /* ---- 热床温度信道（见文件头第一节的通路 ①）----
     * bed_target 落在 (0, 17) 时，它不是床温而是**通道号**：
     *   bed_channel = 1..16 → 该值就是目标通道号，bed_target 不可信
     *   bed_channel = 0    → 正常床温，bed_target 是真实温度
     * 上层（ams_controller）据此做两件事：① 拿它当换料触发源
     * ② 赶在它被改写之前把真实床温记下来，换完用 M190 恢复。 */
    int     bed_channel;

    /* ---- 错误 ---- */
    int     hms_count;
    bambu_hms_t hms[BAMBU_HMS_MAX];

    /* ---- 挤出机到位提示 ----
     * 字段名：print.hw_switch_state（ha-bambulab 确认）
     * 值域 0/1/2/3：0 = 无耗材，非零 = 有耗材
     * 1 = 打印机说耗材已到挤出机
     * 0 = 打印机说没到
     * -1 = 报文里没有这个信息（字段名对不上或该固件不上报）
     * 详见 bambu_proto.c probe_extruder_inplace() 的说明。 */
    int     extruder_inplace_hint;

    /* ---- ★ AMS 握手状态（print.ams_status）----
     * 这是打印机对 AMS 说的"话"，也是换料流程里**唯一**能用来对齐时序的
     * 信号源。取值（Top-AMS 的 bambu.hpp 那份常量表，A1 实测完全一致）：
     *
     *      0    = 空闲 / 退料完成
     *      259  = 退料（切刀 + 吐料）进行中
     *      260  = ★ 退料完成，需要 AMS 退线 —— 轮到我们动手了
     *      1280 = 载入打印材料中
     *
     * 2026-09-22 真机实测（A1，固件 7116b3c 触发
     * `M109 S250\nM620 S255\nT255\nM621 S255\n`）的变化史：
     *
     *      10:00:01  指令发出
     *      10:00:43  ams_status → 259（喷嘴升到 245℃，开始切刀）
     *      10:00:56  ams_status → 260（切完、吐完，轮到 AMS 退线）
     *      10:02:43  ams_status → 0（退料完成）
     *
     * ★ 259 出现在指令之后 **42 秒** —— 那 42 秒全花在等喷嘴升温上
     *   （M109 S250 要求从 140℃ 拉到 250℃）。所以**绝不能拿固定延时去猜**
     *   这一步做完没有：旧版只盲等 3000ms 就开始退料蠕动，那时料还没被
     *   切断、还被挤出机齿轮咬着，电机拽的是完整的一整根料。
     *
     * -1 = 报文里从来没有过这个字段（该固件不上报）。 */
    int     ams_status;
} bambu_report_t;

/* ==========================================================================
 * 解析
 * ========================================================================== */

/**
 * 解析一条 report 报文（完整 JSON 文本，可以带结尾 '\0' 也可以不带）。
 *
 * @return ESP_OK 解析成功；ESP_ERR_INVALID_ARG 不是合法 JSON 或没有 "print" 段
 *
 * 注意：解析失败**不是异常情况** —— 打印机偶尔会发心跳类报文，里面没有
 * "print"，这时返回错误是正常的，调用方忽略即可。
 */
esp_err_t bambu_proto_parse(const char *json, size_t len, bambu_report_t *out);

/** 把 stg_cur 翻成中文（查内置表，返回静态字符串，永远不返回 NULL） */
const char *bambu_stage_text(int stg_cur);

/** 把 HMS 码翻成中文（查内置表，查不到返回 "未知错误"） */
const char *bambu_hms_text(const char *code);

/** 把 attr/code 两个整数拼成 HMS 码字符串，形如 "0300_4000_0002_0001" */
int bambu_hms_format(int attr, int code, char *buf, size_t buflen);

/* ==========================================================================
 * 命令构造（对应 bambu_commands.py）
 * ========================================================================== */
/* 全部返回"写进 buf 的字节数"（不含结尾 '\0'），buf 不够大会返回 -1。 */

/** 让打印机继续打印（换料完成后发） */
int bambu_cmd_resume(char *buf, size_t buflen);

/** 查询全部状态 —— 老代码里的 banbu_start（pushall） */
int bambu_cmd_pushall(char *buf, size_t buflen);

/** 开始定时推送 —— 老代码里的 START_PUSH */
int bambu_cmd_push_start(char *buf, size_t buflen);

/** 发一条 G-code（外层套 gcode_line 命令） */
int bambu_cmd_gcode(char *buf, size_t buflen, const char *gcode);

#ifdef __cplusplus
}
#endif

#endif /* BAMBU_PROTO_H */
