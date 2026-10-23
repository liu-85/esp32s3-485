/**
 * bambu_proto.c —— 拓竹 MQTT 报文的解析与命令构造
 * ============================================================================
 *
 * 用 ESP-IDF 自带的 cJSON（组件名 json）解析。相比 MicroPython 版的 ujson：
 *   · 不用把整条报文先 decode 成 str 再 load —— cJSON 直接吃 char*，
 *     少一次内存拷贝（报文里有中文时那次 decode 尤其亏）
 *   · 解析失败时 cJSON_Parse 返回 NULL，不会抛异常打断任务
 *
 * ★ 内存约定：cJSON_Parse 会 malloc 一整棵树，用完必须 cJSON_Delete。
 *   bambu_proto_parse() 内部保证这点 —— 它只把需要的字段**拷贝**到调用方
 *   的结构体里（stage_text 例外，那是静态表里的常量指针），
 *   函数返回后调用方持有的数据与 JSON 树再无关系。
 */

#include "bambu_proto.h"

#include <stdio.h>
#include <stdlib.h>
#include <ctype.h>
#include <string.h>

#include "cJSON.h"
#include "log_buffer.h"

/* ==========================================================================
 * 一、阶段码表（对应 bambu_const.CURRENT_STAGE_IDS）
 * ==========================================================================
 * 打印机把自己正在做的事用一个整数 stg_cur 报出来，这张表把它翻成中文。
 * 表里没有的码会显示成 "阶段码 N"，所以打印机固件升级加了新阶段也不会
 * 让界面变空白。
 */
typedef struct {
    int         id;
    const char *text;
} stage_entry_t;

static const stage_entry_t s_stages[] = {
    {0,   "打印中"},
    {1,   "自动床调平"},
    {2,   "热床预热"},
    {3,   "XY 机械模式扫描"},
    {4,   "更换打印材料"},
    {5,   "M400 暂停"},
    {6,   "暂停 - 打印材料耗尽"},
    {7,   "加热挤出头"},
    {8,   "校准挤出"},
    {9,   "扫描打印床表面"},
    {10,  "检查首层"},
    {11,  "识别打印板类型"},
    {12,  "校准微型激光雷达"},
    {13,  "归位工具头"},
    {14,  "清洁喷嘴尖端"},
    {15,  "检查挤出头温度"},
    {16,  "用户暂停"},
    {17,  "暂停 - 前盖掉落"},
    {18,  "校准微型激光雷达"},
    /* ★ 2026-09-22 真机实测：**流量校准报的是 8（校准挤出），不是 19**。
     *   把 run5 的全部报文按顺序扫过一遍，stg_cur 只出现过
     *   {2,13,14,0,8,22,24,255}，19 一次都没有。
     *   所以跟"校准"有关的逻辑必须同时认 8 和 19，只认 19 等于没做
     *   （辅助送料就踩过这个坑，见 ams_controller.c 的辅助送料注释）。 */
    {19,  "校准挤出流量"},
    {20,  "暂停 - 喷嘴温度故障"},
    {21,  "暂停 - 热床温度故障"},
    {22,  "卸载打印材料"},
    {23,  "暂停 - 跳过步骤"},
    {24,  "载入打印材料"},
    {25,  "校准电机噪音"},
    {26,  "暂停 - AMS 丢失"},
    {27,  "暂停 - 风扇转速过低"},
    {28,  "暂停 - 腔温控制错误"},
    {29,  "冷却腔体"},
    {30,  "用户 G 代码暂停"},
    {31,  "电机噪音展示"},
    {32,  "暂停 - 检测到喷嘴被料覆盖"},
    {33,  "暂停 - 切刀错误"},
    {34,  "暂停 - 首层错误"},
    {35,  "暂停 - 喷嘴堵塞"},
    {-1,  "空闲"},
    {255, "空闲"},
};

/* ==========================================================================
 * 二、HMS 错误表（对应 bambu_const.HMS_ERRORS_ENGLISH 的中文部分）
 * ==========================================================================
 * HMS = Health Management System，拓竹自己的故障码体系。报文里给的是两个
 * 整数 attr / code，拼成 "0300_4000_0002_0001" 这样的字符串，再查这张表。
 *
 * 表里没有的码不是错误 —— 只是我们还没收录，界面会显示"未知错误"并给出码，
 * 用户可以直接拿去搜。想补就直接往下面加一行。
 */
typedef struct {
    const char *code;
    const char *text;
} hms_entry_t;

static const hms_entry_t s_hms_table[] = {
    {"0300_1000_0002_0001", "X 轴的第一阶机械谐振模式较低。"},
    {"0300_1000_0002_0002", "X 轴的第一阶机械谐振模式差异较大。"},
    {"0300_0F00_0001_0001", "加速度计数据不可用。"},
    {"0300_0D00_0001_000B", "Z 轴电机向上移动时似乎卡住了。"},
    {"0300_0D00_0001_0002", "热床归位失败：环境振动过大。"},
    {"0300_0D00_0001_0003", "打印板未正确放置。"},
    {"0300_0D00_0002_0001", "热床归位异常：可能有凸起。"},
    {"0300_0A00_0001_0005", "力传感器 1/2/3 的静态电压不为 0。"},
    {"0300_0A00_0001_0004", "测试力传感器时检测到外部干扰。"},
    {"0300_0A00_0001_0003", "热床力传感器 1/2/3 的灵敏度过低。"},
    {"0300_0A00_0001_0002", "热床力传感器 1/2/3 的灵敏度较低。"},
    {"0300_0A00_0001_0001", "热床力传感器 1/2/3 的灵敏度过高。"},
    {"0300_0400_0002_0001", "部件冷却风扇转速过慢或已停止。"},
    {"0300_0300_0002_0002", "喷嘴风扇转速较慢。"},
    {"0300_0300_0001_0001", "喷嘴风扇转速过慢或已停止。"},
    {"0300_0600_0001_0001", "电机 A 开路：可能连接松动，或电机已损坏。"},
    {"0300_0600_0001_0002", "电机 A 短路：可能已损坏。"},
    {"0300_0600_0001_0003", "电机 A 电阻异常：可能已损坏。"},
    {"0300_0100_0001_0001", "热床温度异常：加热器可能短路。"},
    {"0300_0100_0001_0002", "热床温度异常：加热器可能断路，或热敏开关断开。"},
    {"0300_0100_0001_0003", "热床温度异常：加热器超温。"},
    {"0300_0100_0001_0006", "热床温度异常：传感器可能短路。"},
    {"0300_0100_0001_0007", "热床温度异常：传感器可能断路。"},
    {"0300_1300_0001_0001", "电机 A 的电流传感器异常：可能是硬件采样电路故障。"},
    {"0300_4000_0002_0001", "串口数据传输异常：软件系统可能存在故障。"},
    {"0300_4100_0001_0001", "系统电压不稳定，已触发断电保护。"},
    {"0300_0200_0001_0001", "喷嘴温度异常：加热器可能短路。"},
    {"0300_0200_0001_0002", "喷嘴温度异常：加热器可能断路。"},
    {"0300_0200_0001_0003", "喷嘴温度异常：加热器超温。"},
    {"0300_0200_0001_0006", "喷嘴温度异常：传感器可能短路。"},
    {"0300_0200_0001_0007", "喷嘴温度异常：传感器可能断路。"},
    {"0300_1200_0002_0001", "工具头前盖已脱落。"},
    {"0C00_0100_0001_0001", "微型激光雷达摄像头离线。"},
    {"0C00_0100_0002_0002", "微型激光雷达摄像头故障。"},
    {"0C00_0100_0001_0003", "微型激光雷达与 MCU 同步异常。"},
    {"0C00_0100_0001_0004", "微型激光雷达镜头似乎脏了。"},
    {"0C00_0100_0001_0005", "微型激光雷达 OTP 参数异常。"},
    {"0C00_0100_0002_0006", "微型激光雷达外参参数异常。"},
    {"0C00_0100_0002_0007", "微型激光雷达激光参数漂移。"},
    {"0C00_0100_0002_0008", "无法从腔体摄像头获取图像。"},
    {"0C00_0100_0001_0009", "腔体摄像头脏了。"},
    {"0C00_0100_0001_000A", "微型激光雷达的 LED 可能损坏。"},
    {"0C00_0100_0001_000B", "微型激光雷达校准失败。"},
    {"0C00_0200_0001_0001", "水平激光未点亮。"},
    {"0C00_0200_0002_0002", "水平激光过粗。"},
    {"0C00_0200_0002_0003", "水平激光亮度不足。"},
    {"0C00_0200_0002_0004", "喷嘴高度似乎太低。"},
    {"0C00_0200_0001_0005", "检测到新的微型激光雷达。"},
    {"0C00_0200_0002_0006", "喷嘴高度似乎太高。"},
    {"0C00_0300_0002_0001", "曝光计量失败。"},
    {"0C00_0300_0002_0002", "激光雷达数据异常，首层检测已终止。"},
    {"0C00_0300_0002_0004", "当前打印不支持首层检测。"},
    {"0C00_0300_0002_0005", "首层检测超时。"},
    {"0C00_0300_0003_0006", "排出的料可能堆积在一起。"},
    {"0C00_0300_0003_0007", "可能存在首层缺陷。"},
    {"0C00_0300_0003_0008", "可能检测到「意面」缺陷（打印乱丝）。"},
    {"0C00_0300_0001_0009", "首层检测模块异常重启。"},
    {"0C00_0300_0003_000B", "正在检测首层。"},
    {"0C00_0300_0002_000C", "未检测到打印板定位标记。"},
    {"0500_0100_0002_0001", "媒体管道故障。"},
    {"0500_0100_0002_0002", "USB 摄像头未连接。"},
    {"0500_0100_0002_0003", "USB 摄像头故障。"},
    {"0500_0100_0003_0004", "SD 卡空间不足。"},
    {"0500_0100_0003_0005", "SD 卡错误。"},
    {"0500_0100_0003_0006", "SD 卡未格式化。"},
    {"0500_0200_0002_0001", "无法连接互联网，请检查网络。"},
    {"0500_0200_0002_0002", "设备登录失败。"},
    {"0500_0200_0002_0004", "未经授权的用户。"},
    {"0500_0200_0002_0006", "实时查看服务故障。"},
    {"0500_0300_0001_0001", "MC 模块故障，请重启设备。"},
    {"0500_0300_0001_0002", "工具头故障，请重启设备。"},
    {"0500_0300_0001_0003", "AMS 模块故障，请重启设备。"},
    {"0500_0300_0001_000A", "系统状态异常，请恢复出厂设置。"},
    {"0500_0300_0001_000B", "屏幕故障。"},
    {"0500_0300_0002_000C", "无线硬件错误，请重启 WiFi 或设备。"},
    {"0500_0400_0001_0001", "下载打印任务失败，请检查网络。"},
    {"0500_0400_0001_0002", "上报打印状态失败，请检查网络。"},
    {"0500_0400_0001_0003", "打印文件内容无法读取，请重新发送。"},
    {"0500_0400_0001_0004", "打印文件未授权。"},
    {"0500_0400_0001_0006", "无法恢复之前的打印。"},
    {"0500_0400_0002_0007", "热床温度超过耗材玻璃化温度，可能导致喷嘴堵塞。"},
    {"0700_4000_0002_0001", "耗材缓冲信号丢失：线缆或位置传感器可能故障。"},
    {"0700_4000_0002_0002", "耗材缓冲位置信号错误：位置传感器可能故障。"},
    {"0700_4000_0002_0003", "AMS Hub 通信异常：线缆可能接触不良。"},
    {"0700_4000_0002_0004", "耗材缓冲信号异常：弹簧可能卡住。"},
    {"0700_4500_0002_0001", "切料传感器故障：可能断开或损坏。"},
    {"0700_4500_0002_0002", "切料距离过大：XY 电机可能失步。"},
    {"0700_4500_0002_0003", "切料手柄未释放：手柄或刀片可能卡住。"},
    {"0700_5100_0003_0001", "AMS 已被禁用，请从卷轴支架上料。"},
    {"12FF_2000_0002_0001", "外挂料已用完，请装载新料。"},
    {"12FF_2000_0002_0002", "外挂料缺失，请装载新料。"},
    {"12FF_2000_0002_0004", "请从挤出机上把卷轴支架的料拉出来。"},
};

/* ==========================================================================
 * 三、小工具
 * ========================================================================== */

/** 取整数，缺字段返回 def */
static int json_int(const cJSON *obj, const char *key, int def)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item)) {
        return (int)item->valuedouble;
    }
    if (cJSON_IsString(item) && item->valuestring[0]) {
        return atoi(item->valuestring);
    }
    return def;
}

/** 取浮点，缺字段返回 def */
static float json_float(const cJSON *obj, const char *key, float def)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsNumber(item)) {
        return (float)item->valuedouble;
    }
    return def;
}

/** 取字符串，缺字段返回 NULL */
static const char *json_str(const cJSON *obj, const char *key)
{
    const cJSON *item = cJSON_GetObjectItemCaseSensitive(obj, key);
    if (cJSON_IsString(item)) {
        return item->valuestring;
    }
    return NULL;
}

const char *bambu_stage_text(int stg_cur)
{
    for (size_t i = 0; i < sizeof(s_stages) / sizeof(s_stages[0]); i++) {
        if (s_stages[i].id == stg_cur) {
            return s_stages[i].text;
        }
    }
    return "未知阶段";
}

const char *bambu_hms_text(const char *code)
{
    if (!code) {
        return "未知错误";
    }
    for (size_t i = 0; i < sizeof(s_hms_table) / sizeof(s_hms_table[0]); i++) {
        if (strcmp(s_hms_table[i].code, code) == 0) {
            return s_hms_table[i].text;
        }
    }
    return "未知错误（可在网上按码搜索）";
}

int bambu_hms_format(int attr, int code, char *buf, size_t buflen)
{
    if (!buf || buflen == 0) {
        return -1;
    }
    /* 拓竹的约定：attr 和 code 各是 32 位，各自拆成两个 16 位段，
     * 拼成 4 段十六进制，用下划线连接 */
    return snprintf(buf, buflen, "%04X_%04X_%04X_%04X",
                    (unsigned)((attr >> 16) & 0xFFFF),
                    (unsigned)(attr & 0xFFFF),
                    (unsigned)((code >> 16) & 0xFFFF),
                    (unsigned)(code & 0xFFFF));
}

/* ==========================================================================
 * 四、主解析
 * ========================================================================== */

/**
 * 挤出机到位提示：在 print 段里找几个候选字段。
 *
 * 确认的字段名（来自 ha-bambulab 的 models.py）：
 *   print.hw_switch_state —— 值域 0/1/2/3，0 = 无耗材，非零 = 有耗材
 *
 * 其余字段是历史猜测，保留兜底。
 */
static const char *const s_extruder_keys[] = {
    "hw_switch_state",          /* ha-bambulab 确认：print.hw_switch_state */
    "s_filament",
    "filament_in_extruder",
    "extruder_filament",
    "s_extruder_filament",
};

static int probe_extruder_inplace(const cJSON *print)
{
    for (size_t i = 0;
         i < sizeof(s_extruder_keys) / sizeof(s_extruder_keys[0]); i++) {
        const cJSON *item =
            cJSON_GetObjectItemCaseSensitive(print, s_extruder_keys[i]);
        if (cJSON_IsNumber(item)) {
            /* hw_switch_state 值域 0/1/2/3：0 = 无耗材，非零 = 有耗材 */
            return item->valueint ? 1 : 0;
        }
        if (cJSON_IsBool(item)) {
            return cJSON_IsTrue(item) ? 1 : 0;
        }
    }
    return -1;
}

/**
 * 从字符串提取 "R128G128B0" 格式的颜色值（兼容 "PLA_R128G128B0" 前缀）。
 * 也支持纯 "128,128,0" 或 "128 128 0" 格式。
 * 成功时 *out_rgb 被设为 0xRRGGBB 并返回 1；失败返回 0。
 */
/** 从 "128G128B0" 提取 R/G/B，兜底手动扫描 */
static int parse_rgb_from_str(const char *p, int *out_rgb)
{
    int r, g, b;
    if (sscanf(p, "%dG%dB%d", &r, &g, &b) == 3 &&
        r >= 0 && r <= 255 &&
        g >= 0 && g <= 255 &&
        b >= 0 && b <= 255) {
        *out_rgb = (r << 16) | (g << 8) | b;
        return 1;
    }
    /* 兜底：逐个扫描第一个、第二个、第三个数字组 */
    const char *s = p;
    int vals[3] = {0, 0, 0};
    int idx = 0;
    while (idx < 3 && *s) {
        if (*s >= '0' && *s <= '9') {
            vals[idx] = 0;
            while (*s >= '0' && *s <= '9') {
                vals[idx] = vals[idx] * 10 + (*s - '0');
                s++;
            }
            idx++;
        } else {
            s++;
        }
    }
    if (idx == 3 &&
        vals[0] >= 0 && vals[0] <= 255 &&
        vals[1] >= 0 && vals[1] <= 255 &&
        vals[2] >= 0 && vals[2] <= 255) {
        *out_rgb = (vals[0] << 16) | (vals[1] << 8) | vals[2];
        return 1;
    }
    return 0;
}

static int parse_color_str(const char *s, int *out_rgb)
{
    if (!s) return 0;
    /* 先找 "R数字G数字B数字" 格式（兼容 "PLA_R128G128B0" 前缀） */
    const char *p = s;
    while (*p) {
        if ((*p == 'R' || *p == 'r') &&
            isdigit((unsigned char)p[1]) && isdigit((unsigned char)p[2])) {
            return parse_rgb_from_str(p + 1, out_rgb);
        }
        p++;
    }
    /* 也试纯 "128,128,0" 或 "128 128 0" 格式 */
    int r = -1, g = -1, b = -1;
    if (sscanf(s, "%d%*[, ]%d%*[, ]%d", &r, &g, &b) == 3 &&
        r >= 0 && r <= 255 &&
        g >= 0 && g <= 255 &&
        b >= 0 && b <= 255) {
        *out_rgb = (r << 16) | (g << 8) | b;
        return 1;
    }
    return 0;
}

/**
 * ★ 把这一帧的 print 段合并进"累积状态"，返回累积对象。
 *
 * 为什么必须这么做（2026-09-22 真机事故后加的）——
 *
 *   拓竹打印机上报的是**增量**状态：某个字段只有"值变了"的那一帧才带，
 *   其余帧里这个键**根本不出现**（有时会显式给 null）。
 *   而本文件的取值一律是"缺字段 → 默认值"的写法，于是下游会把
 *   "打印机这帧没提"读成"值就是 0"。已经确认的两个后果：
 *
 *     · bed_target_temper 被读成 0
 *         → 热床温度记忆被每一帧增量报文清空
 *         → 换色时"没有热床温度的历史记录"，M190 发不出去
 *         → 热床被 M140 S{next+1} 留在 3℃ 一晚上，打印卡死
 *     · nozzle_target_temper 被读成 0
 *         → "等热端到温"永远等不到（喷嘴明明一直在 250℃），白等 30 秒
 *
 *   修法：维护一份跨帧累积的 print 对象，把这一帧带的字段覆盖进去，
 *   后面所有字段都从这个累积对象里读。"缺字段"于是真的等价于
 *   "沿用上一次的值"，和打印机上报的语义一致。
 *
 * 两点注意：
 *   ① 显式 null 表示"当前没有这个值"，**不能**拿它覆盖已有值 —— 跳过。
 *   ② 累积对象是常驻的（键集合有限，不会无限增长），由 cJSON 自己管内存；
 *      每帧只 duplicate 变了的那个键，开销和原来一次解析相当。
 */
static const cJSON *merge_incremental_print(const cJSON *root)
{
    const cJSON *incoming = cJSON_GetObjectItemCaseSensitive(root, "print");
    if (!cJSON_IsObject(incoming)) {
        return NULL;
    }

    static cJSON *s_merged = NULL;
    if (s_merged == NULL) {
        s_merged = cJSON_CreateObject();
        if (s_merged == NULL) {
            /* 内存不够时退回"只看这一帧"的老行为，总比整条链路挂掉强 */
            return incoming;
        }
    }

    const cJSON *c = NULL;
    cJSON_ArrayForEach(c, incoming) {
        if (c->string == NULL || cJSON_IsNull(c)) {
            continue;
        }
        cJSON *dup = cJSON_Duplicate(c, true);
        if (dup == NULL) {
            continue;
        }
        /* ★ 必须先删再加，不能用 cJSON_ReplaceItemInObjectCaseSensitive ——
         *   它内部走 ReplaceItemViaPointer，键**还不存在**时直接返回 false
         *   什么都不做；累积对象就会永远是个空壳，一个字段都攒不下来。 */
        cJSON_DeleteItemFromObjectCaseSensitive(s_merged, c->string);
        cJSON_AddItemToObject(s_merged, c->string, dup);
    }
    return s_merged;
}

esp_err_t bambu_proto_parse(const char *json, size_t len, bambu_report_t *out)
{
    if (!json || !out) {
        return ESP_ERR_INVALID_ARG;
    }
    memset(out, 0, sizeof(*out));
    out->filament_next = -1;
    out->extruder_inplace_hint = -1;
    out->ams_status = -1;
    out->stage_text = "未知";

    cJSON *root = cJSON_ParseWithLength(json, len);
    if (root == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    /* print 指向**跨帧累积**的对象（打印机发的是增量报文，见函数说明） */
    const cJSON *print = merge_incremental_print(root);
    if (!cJSON_IsObject(print)) {
        /* 心跳 / 空报文，属于正常现象，不是错误 */
        cJSON_Delete(root);
        return ESP_ERR_INVALID_ARG;
    }

    /* ---- 打印机状态 ---- */
    const char *gcode_state = json_str(print, "gcode_state");
    if (gcode_state) {
        if (strcmp(gcode_state, "IDLE") == 0) {
            out->gcode_state = 0;
        } else if (strcmp(gcode_state, "PREPARE") == 0) {
            out->gcode_state = 1;
        } else if (strcmp(gcode_state, "RUNNING") == 0) {
            out->gcode_state = 2;
            out->is_printing = true;
        } else if (strcmp(gcode_state, "PAUSE") == 0) {
            out->gcode_state = 3;
            out->is_paused = true;
        } else if (strcmp(gcode_state, "FINISH") == 0) {
            out->gcode_state = 4;
        } else if (strcmp(gcode_state, "FAILED") == 0) {
            out->gcode_state = 5;
        }
    }

    /*
     * ★ 这里原本有一段「gcode_state=RUNNING 但 ams.stage==1 → 视同暂停」的
     *   补丁，已删除。删的理由是它治标不治本，而且把「正在打印」误判成
     *   「已暂停」的风险比收益大：
     *     · M400 U1 触发后打印机一定会进 PAUSE，本来就等得到 is_paused；
     *     · 真正的问题是触发通路没接通（热床信道没解析），
     *       补丁是在给一个不存在的字段打补丁，反而掩盖了根因；
     *     · RUNNING + ams.stage==1 在别的固件版本里可能表示完全不同的意思。
     *   现在换料触发走文件头列的三条通路，不再猜 gcode_state。
     */

    out->mc_percent         = json_int(print, "mc_percent", 0);
    out->mc_remaining_time  = json_int(print, "mc_remaining_time", 0);
    out->layer_num          = json_int(print, "layer_num", 0);
    out->print_error        = json_int(print, "print_error", 0);
    out->stg_cur            = json_int(print, "stg_cur", 255);

    /* 空闲时打印机会把 stg_cur 报 0，但那和"正在打印中"的 0 冲突，
     * 所以按 print_type 校正一下 —— 这是上游逻辑，照搬 */
    const char *print_type = json_str(print, "print_type");
    if (print_type && strcmp(print_type, "idle") == 0 &&
        out->stg_cur == 0) {
        out->stg_cur = 255;
    }
    out->stage_text = bambu_stage_text(out->stg_cur);

    /* ---- 温度 ---- */
    out->nozzle_temper = json_float(print, "nozzle_temper", 0);
    out->nozzle_target = json_float(print, "nozzle_target_temper", 0);
    out->bed_temper    = json_float(print, "bed_temper", 0);

    /* ★ 这里不能图省事用 json_float —— 缺字段时它返回默认值 0，
     *   而"这帧没带床温"和"床温真的是 0"在换料逻辑里含义完全相反。
     *   参见 bambu_proto.h 的 has_bed_target。 */
    {
        const cJSON *bed_tgt =
            cJSON_GetObjectItemCaseSensitive(print, "bed_target_temper");
        out->has_bed_target = cJSON_IsNumber(bed_tgt);
        out->bed_target = out->has_bed_target
                              ? (float)bed_tgt->valuedouble : 0.0f;
    }

    /* ---- 热床温度信道（文件头通路 ①）----
     * 切片用 `M140 S{next_extruder + 1};EXT` 把通道号写进"目标床温"，
     * 所以读到 (0, 17) 范围的值就是通道号，不是温度。
     * 上界 16 沿用 Top-AMS 的做法（它支持 16 通道 AMS）；真实打印任务的
     * 床温不可能低于 17℃（PLA 都要 50+），所以两个区间不会混淆。 */
    out->bed_channel = 0;
    if (out->bed_target > 0.0f && out->bed_target < 17.0f) {
        out->bed_channel = (int)out->bed_target;
    }

    /* ---- ★ 换料判定（本项目最核心的一条）----
     * 三条通路，任意一条成立即触发（见文件头第一节）：
     *  ① 热床温度信道：print.bed_target_temper 落在 (0,17) 就是通道号
     *  ② 官方宏 M73 P101 → mc_percent == 101
     *  ③ 第三方宏 M400 U1 → ams.stage == 1
     *
     * ★ 三条都**必须**以 is_paused 为前提，包括通路 ①。
     *   这一点很关键：换料宏里 M140 S{next_extruder+1} 写在很前面
     *   （A1 的宏在第 6 行），而真正让打印机停下来的 M400 U1 在第 25 行，
     *   中间还有抬 Z、移动喷头等十几行。如果一看到床温变成通道号就动手，
     *   我们会在喷头**还在移动**的时候去切刀、退料 —— 时序全乱。
     *   床温信令会被打印机持续上报（一直到我们发 M190 改回去），
     *   所以等到 PAUSE 那一帧再触发，一帧都不会漏，最多晚 1~2 秒。 */
    /* 先读 ams.stage */
    const cJSON *ams_obj_top =
        cJSON_GetObjectItemCaseSensitive(root, "ams");
    out->ams_stage = -1;
    if (cJSON_IsObject(ams_obj_top)) {
        out->ams_stage = json_int(ams_obj_top, "stage", -1);
    }

    if (out->is_paused &&
        (out->mc_percent == 101 || out->ams_stage == 1 ||
         out->bed_channel > 0)) {
        out->change_needed = true;

        /* ---- 目标通道号（**0 起**）----
         * ① 热床信道优先：切片写 M140 S{next_extruder + 1} 的目的就是
         *    告诉 AMS 换到第几通道，语义没有歧义。
         * ② 没有信道才去探测 M73 P101 系列回传的字段。这些字段名各固件
         *    版本不一样，而且 print.filament_current 在 M400 U1 场景下
         *    报的可能还是**旧**通道 —— 只能当兜底，不能当主力。
         * ③ 都没有才用 mc_remaining_time（旧逻辑，基本不适用）。 */
        out->filament_next = -1;
        if (out->bed_channel > 0) {
            out->filament_next = out->bed_channel - 1;   /* 1 起 → 0 起 */
        } else {
            static const char *const s_filament_keys[] = {
                "filament_next",
                "mc_next_tray",
                "mc_tray_idx",
                "next_tray",
                "filament_current",  /* M400 U1 场景：暂停时当前耗材可能是目标 */
            };
            for (size_t i = 0;
                 i < sizeof(s_filament_keys) / sizeof(s_filament_keys[0]); i++) {
                const cJSON *item =
                    cJSON_GetObjectItemCaseSensitive(print, s_filament_keys[i]);
                if (cJSON_IsNumber(item)) {
                    out->filament_next = (int)item->valueint;
                    break;
                }
            }
            /* 一个都没找到，退回 mc_remaining_time 兜底（值大概率不在 0~3，
             * 上层 ams_controller 会因越界拒绝并记错误日志，不会静默出错） */
            if (out->filament_next < 0) {
                out->filament_next = out->mc_remaining_time;
            }
        }

        /* ---- 目标颜色：自动匹配用 ----
         * M73 P101 R[next_extruder] 只传了通道号，没传颜色。
         * 切片器的 filament_type[next_extruder] 在 G-code 里是字符串
         * （如 "PLA_R0G100B0"），打印机可能把它上报到 ams 段或 print 段。
         *
         * 支持多种来源（按优先级）：
         *  1. ams.filament_color / ams.color：字符串 "R128G128B0"
         *  2. print.filament_type：字符串 "PLA_R0G0B0"（拓竹实际报法）
         *  3. ams.color_arr：数组 [R, G, B]（第三方脚本注入）
         *  4. print.tray_color / print.color：数组 [R, G, B]
         * 抠不出来就保持 -1，上层退回按通道号换料。 */
        out->target_color = -1;

        /* 来源 1：ams.filament_color / ams.color */
        const cJSON *ams_obj =
            cJSON_GetObjectItemCaseSensitive(root, "ams");
        if (cJSON_IsObject(ams_obj)) {
            const cJSON *fcolor =
                cJSON_GetObjectItemCaseSensitive(ams_obj, "filament_color");
            if (cJSON_IsString(fcolor) && fcolor->valuestring) {
                parse_color_str(fcolor->valuestring, &out->target_color);
            }
            if (out->target_color < 0) {
                const cJSON *color =
                    cJSON_GetObjectItemCaseSensitive(ams_obj, "color");
                if (cJSON_IsString(color) && color->valuestring) {
                    parse_color_str(color->valuestring, &out->target_color);
                }
            }
            /* 来源 3：ams.color_arr */
            if (out->target_color < 0) {
                const cJSON *color_arr =
                    cJSON_GetObjectItemCaseSensitive(ams_obj, "color_arr");
                if (cJSON_IsArray(color_arr) &&
                    cJSON_GetArraySize(color_arr) >= 3) {
                    int r = cJSON_GetArrayItem(color_arr, 0)->valueint;
                    int g = cJSON_GetArrayItem(color_arr, 1)->valueint;
                    int b = cJSON_GetArrayItem(color_arr, 2)->valueint;
                    if (r >= 0 && r <= 255 &&
                        g >= 0 && g <= 255 &&
                        b >= 0 && b <= 255) {
                        out->target_color = (r << 16) | (g << 8) | b;
                    }
                }
            }
        }

        /* 来源 2：print.filament_type（拓竹 A1 实际报法） */
        if (out->target_color < 0) {
            const cJSON *ftype =
                cJSON_GetObjectItemCaseSensitive(print, "filament_type");
            if (cJSON_IsString(ftype) && ftype->valuestring) {
                parse_color_str(ftype->valuestring, &out->target_color);
            }
        }

        /* 来源 4：print.tray_color / print.color */
        if (out->target_color < 0) {
            const cJSON *tray_c =
                cJSON_GetObjectItemCaseSensitive(print, "tray_color");
            if (cJSON_IsArray(tray_c) &&
                cJSON_GetArraySize(tray_c) >= 3) {
                int r = cJSON_GetArrayItem(tray_c, 0)->valueint;
                int g = cJSON_GetArrayItem(tray_c, 1)->valueint;
                int b = cJSON_GetArrayItem(tray_c, 2)->valueint;
                if (r >= 0 && r <= 255 &&
                    g >= 0 && g <= 255 &&
                    b >= 0 && b <= 255) {
                    out->target_color = (r << 16) | (g << 8) | b;
                }
            }
        }
        if (out->target_color < 0) {
            const cJSON *pc =
                cJSON_GetObjectItemCaseSensitive(print, "color");
            if (cJSON_IsArray(pc) &&
                cJSON_GetArraySize(pc) >= 3) {
                int r = cJSON_GetArrayItem(pc, 0)->valueint;
                int g = cJSON_GetArrayItem(pc, 1)->valueint;
                int b = cJSON_GetArrayItem(pc, 2)->valueint;
                if (r >= 0 && r <= 255 &&
                    g >= 0 && g <= 255 &&
                    b >= 0 && b <= 255) {
                    out->target_color = (r << 16) | (g << 8) | b;
                }
            }
        }
    }

    /* ---- HMS 错误 ---- */
    const cJSON *hms = cJSON_GetObjectItemCaseSensitive(print, "hms");
    if (cJSON_IsArray(hms)) {
        const cJSON *one = NULL;
        cJSON_ArrayForEach(one, hms) {
            if (out->hms_count >= BAMBU_HMS_MAX) {
                break;
            }
            int attr = json_int(one, "attr", 0);
            int code = json_int(one, "code", 0);
            if (attr <= 0 || code <= 0) {
                continue;
            }
            bambu_hms_t *slot = &out->hms[out->hms_count];
            bambu_hms_format(attr, code, slot->code, sizeof(slot->code));
            strncpy(slot->text, bambu_hms_text(slot->code),
                    sizeof(slot->text) - 1);
            out->hms_count++;
        }
    }

    /* ---- 挤出机到位提示（见函数上方的说明）---- */
    out->extruder_inplace_hint = probe_extruder_inplace(print);

    /* ---- ★ AMS 握手状态（见 bambu_proto.h 的 ams_status）----
     * 从**累积对象**读，所以拿到的是"最新已知值"而不是"这帧带没带"。
     * 这正是握手要的语义：等它变成 260（退料完成需要退线）。
     * 没见过就返回 -1，调用方据此判断"这台机器不给这个字段"。 */
    out->ams_status = json_int(print, "ams_status", -1);

    cJSON_Delete(root);
    return ESP_OK;
}

/* ==========================================================================
 * 五、命令构造（对应 bambu_commands.py）
 * ========================================================================== */

/**
 * 这些 JSON 里的 sequence_id 全都写死成 "1111111"，和 Python 版一致。
 * 打印机并不校验它是否递增 —— 上游项目跑了很久也没问题。
 * 如果你想改成递增计数，只要保证同一分钟内不重复即可。
 */
#define BAMBU_SEQ_ID "1111111"

static int cmd_write(char *buf, size_t buflen, const char *text)
{
    if (!buf || buflen == 0) {
        return -1;
    }
    size_t n = strlen(text);
    if (n + 1 > buflen) {
        return -1;
    }
    memcpy(buf, text, n + 1);
    return (int)n;
}

int bambu_cmd_resume(char *buf, size_t buflen)
{
    return cmd_write(buf, buflen,
        "{\"print\":{\"command\":\"resume\",\"sequence_id\":\"" BAMBU_SEQ_ID "\"},"
        "\"user_id\":\"1\"}");
}

int bambu_cmd_pushall(char *buf, size_t buflen)
{
    return cmd_write(buf, buflen,
        "{\"pushing\":{\"sequence_id\":\"" BAMBU_SEQ_ID "\","
        "\"command\":\"pushall\"}}");
}

int bambu_cmd_push_start(char *buf, size_t buflen)
{
    return cmd_write(buf, buflen,
        "{\"pushing\":{\"sequence_id\":\"" BAMBU_SEQ_ID "\","
        "\"command\":\"start\"}}");
}

int bambu_cmd_gcode(char *buf, size_t buflen, const char *gcode)
{
    if (!buf || !gcode) {
        return -1;
    }
    /*
     * G-code 里带引号、反斜杠、换行会把 JSON 搞坏（换料宏里常有这些字符）。
     * 这里做最小转义：只处理 "  \  和 \n 三个字符，其余原样。
     * 注意："\n G1 E-50 F200;" 这类字符串里带真实换行，旧版没转义会导致
     * JSON 非法，打印机直接丢弃。
     */
    char escaped[512];
    size_t e = 0;
    /* 循环留 5 字节余量：下面还要保证结尾那个 \n，再加 '\0' */
    for (const char *p = gcode; *p && e + 5 < sizeof(escaped); p++) {
        switch (*p) {
        case '"':
        case '\\':
            escaped[e++] = '\\';
            escaped[e++] = *p;
            break;
        case '\n':
            escaped[e++] = '\\';
            escaped[e++] = 'n';
            break;
        default:
            escaped[e++] = *p;
            break;
        }
    }

    /* ★★ 结尾的 \n 在这里统一兜住 —— 这是本协议里最容易踩、且最难发现的坑。
     *
     *   打印机的 gcode_line 要求 param **以换行结尾**，少了它整条指令会被
     *   **静默丢弃**：不报错、不应答，只表现成"指令发出去了但一点反应都没有"。
     *   Top-AMS 是在它自己的 runGcode() 里统一追加的：
     *       bambu.hpp:12  ... + code + R"(\n" ...   //Gcode结尾一定要\n
     *
     *   我们原来是让每个调用点自己写 \n，结果 exchange_heat_nozzle() 里的
     *   "M109 S%d" 就漏了 —— 那条"进料前把热端升到新料温度"的指令一直没生效，
     *   只是因为 4 个通道温度都配成 250℃ 才没暴露出来。等哪天通道温度不一样了，
     *   进料就会撞在偏冷的热端上（料头顶弯、送不进去）。
     *
     *   所以改成同一处保证：结尾一定有 \n，且**只保证一个** ——
     *   调用方写没写都不会变成两个，也不会一个都没有。 */
    if (!(e >= 2 && escaped[e - 2] == '\\' && escaped[e - 1] == 'n')) {
        escaped[e++] = '\\';
        escaped[e++] = 'n';
    }
    escaped[e] = '\0';

    int n = snprintf(buf, buflen,
        "{\"print\":{\"sequence_id\":\"" BAMBU_SEQ_ID "\","
        "\"command\":\"gcode_line\",\"param\":\"%s\"}}", escaped);
    if (n < 0 || (size_t)n >= buflen) {
        return -1;
    }
    return n;
}
