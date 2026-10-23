# -*- coding: utf-8 -*-
"""
ESP-AMS 换色配置工具
====================

功能：
  1. 选择通道数（4 / 8 / 16）
  2. 配置每个通道的颜色（左键调色，右键启禁用）
  3. 首次换料开关
  4. 配置文件管理（新建 / 打开 / 目录）
  5. 导出 filament_settings.json（供 Bambu Studio 后处理脚本使用）
  6. G-code 匹配预览弹窗（显示每个 T 指令匹配到哪个通道）
  7. 可独立运行（GUI），也可嵌入 Bambu Studio 作为后处理脚本

独立运行（GUI）：
  python esp_ams_postprocess.py
  或编译成 EXE 后直接运行。

Bambu Studio 后处理脚本：
  本文件放在 {BambuStudio}/scripts/filament/ 目录下，
  在偏好设置 → G-code 文件编辑 → 后处理脚本中启用。
  导出后的 filament_settings.json 放在 G-code 输出目录同位置，
  后处理脚本会自动查找并加载，匹配日志写入 <gcode>.ams.log。
"""

import json
import os
import re
import sys
from pathlib import Path
from typing import Dict, List, Optional, Tuple

# ============================== 常量 ==============================

# 默认 8 通道颜色（白、灰、黑、红、绿、蓝、橙、黄）
DEFAULT_COLORS_8CH: List[Tuple[int, int, int]] = [
    (255, 255, 255),  # 白
    (128, 128, 128),  # 灰
    (30, 30, 30),     # 黑
    (220, 50, 50),    # 红
    (50, 180, 80),    # 绿
    (50, 80, 220),    # 蓝
    (230, 140, 40),   # 橙
    (220, 210, 40),   # 黄
]

# 16 通道额外颜色（紫、粉、棕、浅绿、深红、浅蓝、灰蓝、金黄）
DEFAULT_COLORS_16CH_EXTRA: List[Tuple[int, int, int]] = [
    (160, 32, 240),   # 紫
    (255, 105, 180),  # 粉
    (139, 69, 19),    # 棕
    (144, 238, 144),  # 浅绿
    (139, 0, 0),      # 深红
    (135, 206, 250),  # 浅蓝
    (100, 149, 237),  # 灰蓝
    (255, 215, 0),    # 金黄
]

SETTINGS_FILENAME = "filament_settings.json"

# 项目里还可能存在这两种叫法的配置（AMS_colors.json 是网页导出的），
# 一并认，免得"明明配了却说没找到"。
LEGACY_SETTINGS_NAMES = ("filament_settings.json", "AMS_colors.json",
                         "esp_ams_map.json")

# ★★ 换料握手信令：`M140 S<n>;EXT`（n = 切片器的耗材序号 + 1）
#    这是**主握手通路** —— 固件读打印机的 bed_target_temper，落在 (0,17)
#    就当成通道号。起始 G-code 写的是 {initial_no_support_extruder+1}，
#    换料 G-code 写的是 {next_extruder + 1}，切片后都变成这一行的字面量。
#
#    ⚠️ 2026-09-24 才发现：本脚本以前**一个字都没改过 M140**，只改了
#    M73 P101（而那份换料 G-code 里 M73 根本没进最终文件）。于是
#    "颜色匹配"整条链路是断的：切片槽号被原样当成物理通道号用了。
#    现场表现就是用户报的"切片软件通道颜色要跟项目一致才能换对颜色"。
#
#    注释标记容错：TOP AMS 的 EXE 产出的是被截断的 `;ET`（少一个 X），
#    两种都认；改写时一律规范成 `;EXT`。
EXT_MARK_RE = re.compile(r'^(\s*M140\s+S)(\d+)(\s*;\s*E[XT]+\s*)$',
                         re.IGNORECASE)

COLOR_DISTANCE_THRESHOLD = 100.0

# G-code 头部映射块的标识行。**同时是幂等守卫**：文件里已经有它 =
# 已经映射过，绝不能再按"值-1=切片槽号"映射第二遍（会把白映射成橄榄）。
MAP_HEADER = ";=========== ESP-AMS 颜色映射（切片槽 → 物理通道）==========="

# 颜色名兜底表：`filament_colour` 缺失时，从 filament_settings_id 的
# 名字里（"Generic PETG 240 230红" / "PLA 白色"）猜颜色。
# ⚠️ 顺序有讲究：长名在前，否则"浅蓝"会被"蓝"先吃掉。
COLOR_NAME_TABLE: List[Tuple[str, Tuple[int, int, int]]] = [
    ("浅蓝", (135, 206, 250)), ("灰蓝", (100, 149, 237)),
    ("深红", (139, 0, 0)), ("浅绿", (144, 238, 144)),
    ("金黄", (255, 215, 0)), ("橙色", (230, 140, 40)),
    ("黄色", (220, 210, 40)), ("灰色", (128, 128, 128)),
    ("白色", (255, 255, 255)), ("黑色", (30, 30, 30)),
    ("红色", (220, 50, 50)), ("绿色", (50, 180, 80)),
    ("蓝色", (50, 80, 220)), ("紫色", (160, 32, 240)),
    ("粉色", (255, 105, 180)), ("棕色", (139, 69, 19)),
    ("白", (255, 255, 255)), ("黑", (30, 30, 30)),
    ("红", (220, 50, 50)), ("绿", (50, 180, 80)),
    ("蓝", (50, 80, 220)), ("黄", (220, 210, 40)),
    ("橙", (230, 140, 40)), ("紫", (160, 32, 240)),
    ("粉", (255, 105, 180)), ("棕", (139, 69, 19)),
    ("灰", (128, 128, 128)),
]

# 弹窗策略（配置里的 "popup" 键）
#   always（默认）—— 每次切片都弹映射确认窗。★ 这是**现场明确要求**的：
#                    TOP AMS 的 EXE 就是每次切片都弹一个"耗材丝 ↔ 通道"确认窗，
#                    用户能一眼看到"哪个切片槽 → 哪个料盘位"，不对劲当场就能改。
#   auto        —— 只在**需要人做决定**时才弹：有颜色匹配不上、有歧义、
#                  或者切片颜色比通道还多。一切正常就静默完成，不打断切片。
#   never       —— 从不弹，只写日志和 G-code 注释
POPUP_AUTO, POPUP_ALWAYS, POPUP_NEVER = "auto", "always", "never"
POPUP_DEFAULT = POPUP_ALWAYS

# 弹窗无人操作时的自动确认秒数。
# ★ 为什么要它：切片器是在「运行后处理脚本」这一步等我们退出的
#   （进度条卡在 95%）。人要是走开了，窗口挂着 = 切片永远不结束。
#   超时后按"窗口里当前显示的结论"自动确认，并在 `.ams.log` 里记一笔。
POPUP_AUTO_DONE_SEC = 180

# 配置文件名（写回 popup 偏好时用）
SETTINGS_KEY_POPUP = "popup"

# 各通道数默认颜色
def _default_colors_for_channel_count(n: int) -> List[Tuple[int, int, int]]:
    """生成 n 个通道的默认颜色"""
    if n <= 4:
        # 4 通道：白、灰、黑、蓝
        return [
            (255, 255, 255),
            (128, 128, 128),
            (30, 30, 30),
            (50, 80, 220),
        ]
    elif n <= 6:
        # 6 通道：白、灰、黑、蓝、红、绿
        return [
            (255, 255, 255),
            (128, 128, 128),
            (30, 30, 30),
            (50, 80, 220),
            (220, 50, 50),
            (50, 180, 80),
        ]
    elif n <= 8:
        return list(DEFAULT_COLORS_8CH)
    else:
        colors = list(DEFAULT_COLORS_8CH) + list(DEFAULT_COLORS_16CH_EXTRA)
        if len(colors) < n:
            while len(colors) < n:
                colors.extend(colors[:8])
        return colors[:n]


# ============================== AMS 起始 G-code 占位符 ==============================

def find_ams_initial_tray(
    gcode_content: str,
    ams_colors: Dict[int, Tuple[int, int, int]],
    slice_colors: Dict[int, Tuple[int, int, int]],
) -> int:
    """
    根据 G-code 内容查找首层目标 AMS 通道号。

    优先级：
      1. G-code 里第一个出现的 T 指令（切片器对首层使用的颜色），用
         slice_colors[T] 在 ams_colors 里匹配最接近的颜色
      2. 没有切片颜色信息 → 取 ams_colors 里编号最小的启用通道
      3. 全都没匹配上 → 返回 0（由调用方回退到 1）
    """
    if not ams_colors:
        return 0

    # 1. 找第一个 T 指令（通常是首层用的耗材）
    first_t = None
    m = re.search(r'^T(\d+)', gcode_content, re.MULTILINE)
    if m:
        first_t = int(m.group(1))

    if first_t is not None:
        target = slice_colors.get(first_t)
        if target is not None:
            matched, dist = find_best_channel(target, ams_colors)
            if matched > 0 and dist <= COLOR_DISTANCE_THRESHOLD:
                return matched

    # 2. 回退：启用通道中编号最小的那个
    return min(ams_colors.keys())


def substitute_ams_initial_tray(
    gcode_content: str,
    ams_colors: Dict[int, Tuple[int, int, int]],
    slice_colors: Dict[int, Tuple[int, int, int]],
) -> Tuple[str, List[str]]:
    """
    把 G-code 里的 {ams_initial_tray} 占位符替换为具体的 AMS 通道号。
    返回 (新内容, 日志行列表)。
    """
    if '{ams_initial_tray}' not in gcode_content:
        return gcode_content, []

    target = find_ams_initial_tray(gcode_content, ams_colors, slice_colors)
    if target == 0:
        target = 1
        log = f"警告：无法匹配首层颜色，{{ams_initial_tray}} 回退为通道 1"
    else:
        log = f"已替换 {{ams_initial_tray}} → 通道 {target}"

    return gcode_content.replace('{ams_initial_tray}', str(target)), [log]


# ============================== 颜色工具 ==============================

def rgb_to_hex(r: int, g: int, b: int) -> str:
    return "#{:02X}{:02X}{:02X}".format(r, g, b)


def hex_to_rgb(h: str) -> Optional[Tuple[int, int, int]]:
    h = h.strip().lstrip('#')
    if len(h) != 6:
        return None
    try:
        return (int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16))
    except (ValueError, IndexError):
        return None


def _is_dark(c: Tuple[int, int, int]) -> bool:
    r, g, b = c
    return 0.299 * r + 0.587 * g + 0.114 * b < 128


def color_distance(c1: Tuple[int, int, int], c2: Tuple[int, int, int]) -> float:
    """RGB 欧氏距离"""
    return ((c1[0] - c2[0]) ** 2 +
            (c1[1] - c2[1]) ** 2 +
            (c1[2] - c2[2]) ** 2) ** 0.5


def find_best_channel(
    target: Tuple[int, int, int],
    ams_colors: Dict[int, Tuple[int, int, int]],
) -> Tuple[int, float]:
    """在 AMS 通道里找最接近 target 的通道号（1 起），返回 (ch, dist)"""
    best_ch = -1
    best_dist = float('inf')
    for ch, c in ams_colors.items():
        d = color_distance(target, c)
        if d < best_dist:
            best_dist = d
            best_ch = ch
    return best_ch, best_dist


# ============================== 配置文件读写 ==============================

def default_settings(channel_count: int = 8) -> dict:
    colors = _default_colors_for_channel_count(channel_count)
    materials = []
    for i, c in enumerate(colors, start=1):
        materials.append({
            "channel": i,
            "color": list(c),
            "enabled": True,
        })
    return {
        "channel_count": channel_count,
        "selected_channel": 1,
        "first_filament": True,  # 首次换料开关
        "popup": POPUP_DEFAULT,  # 每次切片都弹映射确认窗（见 POPUP_* 注释）
        "materials": materials,
    }


def update_settings_key(path: Optional[str], key: str, value) -> bool:
    """只改配置里的**一个键**，其余键原样保留。

    ★ 为什么不能直接 `save_settings(default_settings(), path)`：那会把用户
      配的通道颜色全部冲掉。确认窗里的"以后别再问了"就是走这条。
    """
    if not path:
        return False
    data = load_settings(path)
    if not data:
        return False
    data[key] = value
    return save_settings(data, path)


def load_settings(path: str) -> Optional[dict]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = json.load(f)
        # 兼容 AMS_colors.json 的 AMS[] 格式
        if "AMS" in data and "materials" not in data:
            materials = []
            for item in data.get("AMS", []):
                ch = int(item.get("channel", 0))
                c = item.get("color", [128, 128, 128])
                if isinstance(c, str):
                    parsed = hex_to_rgb(c)
                    c = list(parsed) if parsed else [128, 128, 128]
                materials.append({"channel": ch, "color": c, "enabled": True})
            data["materials"] = materials
            data.setdefault("channel_count", len(materials))
            data.setdefault("selected_channel", 1)
            data.setdefault("first_filament", True)
        if "materials" not in data:
            return None
        return data
    except (json.JSONDecodeError, OSError):
        return None


def save_settings(data: dict, path: str) -> bool:
    try:
        d = os.path.dirname(path)
        if d:
            os.makedirs(d, exist_ok=True)
        with open(path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        return True
    except OSError:
        return False


def get_ams_colors(data: dict) -> Dict[int, Tuple[int, int, int]]:
    """从配置中提取 {channel: (r,g,b)}，只包含启用的通道"""
    result = {}
    for m in data.get("materials", []):
        ch = int(m.get("channel", 0))
        c = m.get("color", [])
        if ch > 0 and len(c) >= 3 and m.get("enabled", True):
            result[ch] = (int(c[0]), int(c[1]), int(c[2]))
    return result


# ============================== G-code 改写 ==============================

def name_to_rgb(text: str) -> Optional[Tuple[int, int, int]]:
    """从一段文字（耗材预设名）里猜颜色。猜不到返回 None。"""
    if not text:
        return None
    for name, rgb in COLOR_NAME_TABLE:
        if name in text:
            return rgb
    return None


def parse_slice_colors(gcode_content: str) -> Dict[int, Tuple[int, int, int]]:
    """
    从 G-code 头部注释里提取**切片器实际用的**耗材颜色。

    返回 {耗材序号(0 起): (r,g,b)}。

    ★★ 真实格式（Bambu Studio 02.08 实测，2026-09-24）——

        ; extruder_colour = #018001
        ; filament_colour = #808000;#FFFFFF;#101010;#0000FF
        ; filament_settings_id = "Generic PETG 240 230红";"…白色";"…黑色";"…蓝色"

    ⚠️ 本函数原来只认 `;T0 ;color=#FFFFFF` / `; extruder 0: #FFFFFF` 这类
    **第三方工具自己写的**格式，Bambu Studio 一个都不产出 → 永远返回空 →
    颜色匹配整条链路静默失效（不报错、只是不匹配）。这是"配了颜色也不生效"
    的根因之一。

    三级兜底，越靠后置信度越低（调用方按 confidence 决定要不要提醒用户）：
      ① `; filament_colour = ...`（最权威，就是切片器里的色块）
      ② `; extruder_colour = ...`（单色打印时只有这一个）
      ③ `; filament_settings_id` 里的名字（"…红"/"…白色"）—— 仅在前两级
         完全拿不到时才用
    """
    colors: Dict[int, Tuple[int, int, int]] = {}

    def _split_colors(val: str) -> List[Tuple[int, int, int]]:
        out: List[Tuple[int, int, int]] = []
        for part in val.split(';'):
            p = part.strip().strip('"').strip()
            if not p:
                out.append((0, 0, 0))
                continue
            rgb = hex_to_rgb(p)
            out.append(rgb if rgb else (0, 0, 0))
        return out

    head = gcode_content[:262144]

    # ① filament_colour：形如 `; filament_colour = #808000;#FFFFFF;...`
    m = re.search(r'^;\s*filament_colour\s*=\s*(.+)$', head, re.MULTILINE)
    if m:
        vals = _split_colors(m.group(1))
        for i, c in enumerate(vals):
            # 0x000000 有两种可能：真的是黑色，或者那一格是空的。
            # 用 filament_settings_id 里同序号的名字来分辨。
            colors[i] = c

    # ② extruder_colour：单色/多挤出机机型才有
    if not colors:
        m = re.search(r'^;\s*extruder_colour\s*=\s*(.+)$', head, re.MULTILINE)
        if m:
            vals = _split_colors(m.group(1))
            for i, c in enumerate(vals):
                colors[i] = c

    # ③ 名字兜底：把拿到 (0,0,0) 或压根没有的槽位，用预设名里带的中文补齐
    m = re.search(r'^;\s*filament_settings_id\s*=\s*(.+)$', head, re.MULTILINE)
    if m:
        names = [p.strip().strip('"') for p in m.group(1).split(';')]
        for i, nm in enumerate(names):
            if colors.get(i) in (None, (0, 0, 0)):
                guess = name_to_rgb(nm)
                if guess:
                    colors[i] = guess

    return colors


def build_channel_map(
    ams_colors: Dict[int, Tuple[int, int, int]],
    slice_colors: Dict[int, Tuple[int, int, int]],
    threshold: float = COLOR_DISTANCE_THRESHOLD,
) -> Tuple[Dict[int, int], List[str], List[Dict]]:
    """
    算出「切片耗材序号(0 起) → 物理通道号(1 起)」的映射表。

    返回 (映射表, 日志行, 明细行列表)。
    明细行是给弹窗/注释用的结构化数据，每项：
        {"idx":0, "color":(r,g,b), "ch":1, "dist":0.0, "status":"ok"}

    ★★ 这就是"切片软件通道颜色随便设，开发板自动换对颜色"的实现点。
       切片器的槽位号只在这张表里活一次，写进 G-code 的永远是物理通道号 ——
       固件那边一个字都不用改（它本来就只认通道号）。
    """
    mapping: Dict[int, int] = {}
    logs: List[str] = []
    detail: List[Dict] = []

    if not ams_colors:
        for idx in sorted(slice_colors):
            mapping[idx] = idx + 1
            detail.append({"idx": idx, "color": slice_colors[idx],
                           "ch": idx + 1, "dist": -1.0, "status": "no_cfg"})
        logs.append("警告：项目通道颜色为空（GUI 里没配 / 找不到配置文件）"
                    "→ 只能按通道号换料，颜色对不对全看运气")
        return mapping, logs, detail

    if not slice_colors:
        logs.append("警告：切片文件里没有颜色信息 → 只能按通道号换料")
        for idx in range(len(ams_colors)):
            mapping[idx] = idx + 1
            detail.append({"idx": idx, "color": None, "ch": idx + 1,
                           "dist": -1.0, "status": "no_slice"})
        return mapping, logs, detail

    used: Dict[int, List[int]] = {}      # 通道 → 哪些切片槽占了它
    for idx in sorted(slice_colors):
        c = slice_colors[idx]
        best_ch, dist = find_best_channel(c, ams_colors)
        if best_ch > 0 and dist <= threshold:
            mapping[idx] = best_ch
            status = "ok"
            logs.append("切片槽%d（%s）→ 通道%d（色差 %.1f）✓"
                        % (idx + 1, rgb_to_hex(*c), best_ch, dist))
        else:
            mapping[idx] = idx + 1
            status = "far" if best_ch > 0 else "none"
            logs.append("切片槽%d（%s）→ **匹配不上**（最近是通道%d，色差 %.1f "
                        "> 阈值 %.0f），保留原通道号 %d"
                        % (idx + 1, rgb_to_hex(*c), best_ch, dist,
                           threshold, idx + 1))
        used.setdefault(mapping[idx], []).append(idx)
        detail.append({"idx": idx, "color": c, "ch": mapping[idx],
                       "dist": dist, "status": status})

    # 两个不同的切片槽落到同一个物理通道 → 值得提一句（可能是同料，也可能是
    # 两个白料盘漏配了颜色；后者打出来颜色就会串）
    for ch, idxs in sorted(used.items()):
        if len(idxs) > 1:
            logs.append("注意：切片槽 %s 都映射到通道 %d —— 若它们真是同一种料"
                        "就正常；否则请在 GUI 里把两个通道的颜色区分开"
                        % ("、".join(str(i + 1) for i in idxs), ch))

    return mapping, logs, detail


def map_line_for(idx: int, ch: int, color, dist: float, status: str) -> str:
    """给 G-code 里插的注释行：现场 `grep ESP-AMS` 就能核验映射。"""
    if color is None:
        return ";ESP-AMS 切片槽%d → 通道%d（切片没给颜色）" % (idx + 1, ch)
    if status == "ok":
        return (";ESP-AMS 切片槽%d = %s → **通道%d**（色差 %.1f）"
                % (idx + 1, rgb_to_hex(*color), ch, dist))
    return (";ESP-AMS 切片槽%d = %s → 通道%d（⚠ 色差 %.1f，没匹配上，原样保留）"
            % (idx + 1, rgb_to_hex(*color), ch, max(dist, 0.0)))


def rewrite_gcode(
    gcode_content: str,
    ams_colors: Dict[int, Tuple[int, int, int]],
    slice_colors: Dict[int, Tuple[int, int, int]],
    first_filament: bool = True,
    threshold: float = COLOR_DISTANCE_THRESHOLD,
) -> Tuple[str, List[str], Dict, List[Dict]]:
    """
    改写 G-code 里的换料信令，把**切片槽号**换成**物理通道号**。

    返回 (改写后的内容, 日志行, 汇总 dict, 映射明细)

    ★ 原则：**只改已有的信令，不新增任何信令。**
      （老版本会顺手给每个 `T<n>` 插一条 `M73 P101 R<n>`，见下面 ② 的注释 ——
        那是个能凭空多触发几次换料的 bug。）

    改两处：
      ① ★ `M140 S<n>;EXT` —— 主握手通路（值 = 切片槽号 + 1）。
         **以前完全没改过这一条**，所以颜色匹配整条链路是断的。
      ② `M73 P101 R<n>`   —— 备用通路（宏里写了才会出现；n = 切片槽号）。

    `first_filament` 保留只为兼容旧调用（首次换料的冲刷量归宏管，本脚本不碰）。
    """
    del first_filament

    log_lines: List[str] = []

    # ★ 幂等守卫：头部已经有映射块 → 这个文件已经被处理过了。
    #
    #   为什么必须有：改写后的 `M140 S<n>` 里 n 是**物理通道号**，
    #   再按"值 - 1 = 切片槽号"去映射一次就会变成别的通道（实测：白 → 橄榄）。
    #   而"重复执行"在这条链路上很容易发生 —— 手动再跑一次、切片器把已导出
    #   的文件又交给后处理一次、或者拿旧文件做实验。
    mapping, map_logs, detail = build_channel_map(ams_colors, slice_colors,
                                                  threshold)
    if MAP_HEADER in gcode_content:
        log_lines.append("这个 G-code 已经映射过了（头部有 " + MAP_HEADER
                         + "）→ 跳过，不重复改写")
        summary = {"ams_channels": len(ams_colors),
                   "slice_colors": len(slice_colors),
                   "m140_ok": 0, "m140_miss": 0, "m73_ok": 0,
                   "change_points": 0, "needs_human": False,
                   "already_done": True}
        return gcode_content, log_lines, summary, detail

    log_lines.extend(map_logs)

    summary = {
        "ams_channels": len(ams_colors),
        "slice_colors": len(slice_colors),
        "m140_ok": 0, "m140_miss": 0,
        "m73_ok": 0, "change_points": 0,
        "needs_human": False,
    }
    # 需要人来拍板的情形：颜色匹配不上、或者压根没有颜色可比
    if any(d["status"] in ("far", "none", "no_slice", "no_cfg") for d in detail):
        summary["needs_human"] = True

    t_real = sum(1 for l in gcode_content.split('\n')
                 if re.match(r'^T(\d+)\s*(?:;.*)?$', l.strip(), re.IGNORECASE)
                 and int(re.match(r'^T(\d+)', l.strip(), re.IGNORECASE)
                         .group(1)) < 100)
    m73_count = len(re.findall(r'M73\s+P101\s+R\d+', gcode_content))
    m140_count = sum(1 for l in gcode_content.split('\n')
                     if EXT_MARK_RE.match(l))
    log_lines.insert(0, "G-code 统计：M140 换料信令 %d 处，真换料点 T<n> %d 处，"
                        "M73 P101 %d 处" % (m140_count, t_real, m73_count))

    # ★ 改写 {ams_initial_tray} 占位符（起始 G-code 用自定义占位符时才有）
    if '{ams_initial_tray}' in gcode_content:
        gcode_content, ams_tray_logs = substitute_ams_initial_tray(
            gcode_content, ams_colors, slice_colors)
        log_lines.extend(ams_tray_logs)

    lines = gcode_content.split('\n')
    out_lines: List[str] = []
    ptr: Dict[int, int] = {d["idx"]: i for i, d in enumerate(detail)}

    for line in lines:

        # ---------- ① M140 S<n>;EXT：主握手通路 ----------
        m = EXT_MARK_RE.match(line)
        if m:
            slice_ch = int(m.group(2))
            idx = slice_ch - 1                      # 切片槽号 = 值 - 1
            new_ch = mapping.get(idx, slice_ch)
            d = detail[ptr[idx]] if idx in ptr else None
            color = d["color"] if d else slice_colors.get(idx)
            dist = d["dist"] if d else -1.0
            status = d["status"] if d else "no_slice"

            out_lines.append(map_line_for(idx, new_ch, color, dist, status))
            out_lines.append("%s%d%s" % (m.group(1), new_ch, ";EXT"))
            if new_ch == slice_ch:
                summary["m140_miss"] += 1
            else:
                summary["m140_ok"] += 1
            continue

        # ---------- ② 裸露的 T 指令：**只加注释、绝不新增信令** ----------
        #
        # ★★ 这里删掉了一段"把 T<n> 就地展开成 M73 P101 R<n>"的老逻辑。
        #    它是**危险的**，触发条件还特别常见：
        #      · Bambu 的起始/结束宏里本来就写着 `T1000`（"没有 AMS 工具"的
        #        内部标记）和 `T255`（卸载），它们会命中 `^T(\d+)`；
        #      · 老逻辑于是给每个 T 都插一条 `M73 P101 R1001`。
        #    而固件把 `mc_percent == 101` 当作**换料请求**（ams_controller.c
        #    的 r->mc_percent == 101 分支）→ 凭空多出 3 次换料请求，
        #    通道号还是 1001 这种垃圾值。
        #    两份真实切片里这类 T 各有 3 处，也就是说"一挂脚本就多触发 3 次"。
        #
        #    规则：**只改已有的信令，不加新的。** 时序是宏作者的，改时序要
        #    有实测支撑，不能靠后处理脚本顺手"加固"。
        m_t = re.match(r'^T(\d+)\s*(?:;.*)?$', line.strip(), re.IGNORECASE)
        if m_t:
            n = int(m_t.group(1))
            if n < 100 and n in ptr:      # <100 才是真的换料点
                d = detail[ptr[n]]
                out_lines.append(
                    ";ESP-AMS 换料点：切片槽%d = %s → 通道%d"
                    % (n + 1, rgb_to_hex(*d["color"]) if d["color"] else "（无）",
                       d["ch"]))
                summary["change_points"] = summary.get("change_points", 0) + 1
            out_lines.append(line)
            continue

        # ---------- ③ 已有的 M73 P101 R<n>（部分宏会写）----------
        #   注意：这里的 n 是**切片器的 0 起序号**（宏里写 R[next_extruder]），
        #   而 M140 S<n> 里的 n 是"序号 + 1"。两者口径不同，别抄错。
        m73 = re.match(r'^(M73\s+P101\s+R)(\d+)(\s*;.*)?$',
                       line.strip(), re.IGNORECASE)
        if m73:
            idx = int(m73.group(2))
            new_ch = mapping.get(idx, idx + 1)
            out_lines.append("%s%d" % (m73.group(1), new_ch)
                             + (m73.group(3) or ""))
            if new_ch != idx + 1:
                summary["m73_ok"] = summary.get("m73_ok", 0) + 1
            continue

        out_lines.append(line)

    result = '\n'.join(out_lines)

    # ---------- ★ 把映射摘要写进文件头注释 ----------
    # 为什么要写进文件：`.ams.log` 放在输出目录里，用户不一定会去看；
    # 而切片完的 G-code 本身就是要发给打印机的东西，"打开看一眼映射对不对"
    # 是最顺手的一次核验。现场 `grep ESP-AMS xxx.gcode` 即可。
    # ★ 幂等守卫见函数开头（MAP_HEADER 检查）。

    if mapping:
        stamp = [MAP_HEADER]
        for d in detail:
            stamp.append(map_line_for(d["idx"], d["ch"], d["color"],
                                      d["dist"], d["status"]))
        stamp.append("; 只改两处已有信令：M140 S<n>;EXT 和 M73 P101 R<n>。"
                     "不新增任何信令。")
        stamp.append(";======================================================")
        result = '\n'.join(stamp) + '\n' + result

    return result, log_lines, summary, detail


def find_settings_file(gcode_path: str) -> Optional[str]:
    """在 G-code 所在目录及上级目录查找配置文件"""
    base = Path(gcode_path).parent
    for d in (base, base.parent):
        for name in LEGACY_SETTINGS_NAMES:
            c = d / name
            if c.is_file():
                return str(c)
    return None


def _app_base() -> Path:
    """脚本/EXE 所在目录（PyInstaller 打包后就是 EXE 所在目录）"""
    if getattr(sys, "frozen", False):
        return Path(sys.executable).parent
    return Path(__file__).parent


def _bambu_config_dir() -> Optional[str]:
    """查找 GUI 导出的配置文件目录。

    ★★ 这里踩过一个"配了也不生效"的坑（2026-09-24）：GUI（esp_ams_tool.exe）
       把配置写在 **<EXE同目录>/ams_profiles/filament_settings.json**，
       而本函数的候选目录里**没有 ams_profiles** —— 于是 EXE 作为后处理脚本
       跑的时候永远找不到配置，只能退回"按通道号换料"，且只在控制台里
       （windowed 打包后连控制台都没有）打一句，用户完全看不到。
       现在把 ams_profiles 排在最前，并且加一个"上次打开过的目录"记忆文件。
    """
    base = _app_base()
    candidates: List[Path] = []
    for root in (base, base.parent, Path.home()):
        for sub in ("ams_profiles", "ESP_AMS", "filament_settings", ""):
            candidates.append(root / sub if sub else root)
    candidates.append(Path.home() / "ESP_AMS")
    candidates.append(Path(gcode_last_dir_hint() or "."))

    for c in candidates:
        for name in LEGACY_SETTINGS_NAMES:
            target = c / name
            if target.is_file():
                return str(c)
    return None


def gcode_last_dir_hint() -> Optional[str]:
    """读 GUI 记下的"上次使用的配置目录"（没有就返回 None）。"""
    try:
        p = _app_base() / ".esp_ams_last_dir"
        if p.is_file():
            v = p.read_text(encoding="utf-8").strip()
            return v or None
    except OSError:
        pass
    return None


def find_settings_extended() -> Optional[str]:
    """比 find_settings_file 更宽的搜索：脚本目录 + ams_profiles + 用户目录"""
    config_dir = _bambu_config_dir()
    if config_dir:
        for name in LEGACY_SETTINGS_NAMES:
            target = Path(config_dir) / name
            if target.is_file():
                return str(target)
    return None


# ============================== 后处理脚本入口（Bambu Studio 用） ==============================

# 「当前进程有没有可用的控制台」。
# ★ main() 会把 None 的 stdout 换成黑洞（防 AttributeError），那就再也分不出
#   "没控制台"了 —— 所以在换之前把真值记在这里。
#   PyInstaller --windowed 的 EXE 从切片器/资源管理器启动时是 False，
#   这时任何"打印说明给人看"的地方都必须改用窗口，否则用户什么都看不到。
STDOUT_OK = True


def safe_print(*args) -> None:
    """打印到控制台。

    ★ PyInstaller 用 `--windowed` 打包后 sys.stdout 是 None，直接 print()
      会抛 AttributeError；而 Bambu Studio 又把后处理进程的 stdout 收走，
      所以任何一次裸 print 都可能把整个后处理搞崩 —— 而崩了就是"窗口一闪
      然后什么都没发生"，正是现场看到的现象。这里统一兜住。
    """
    try:
        if sys.stdout is None:
            return
        print(*args)
    except Exception:
        pass


def load_config_for(gcode_path: str, explicit: Optional[str] = None):
    """统一的配置加载：找文件 → 读出通道颜色 / 首次换料 / 弹窗策略。

    返回 (settings_path, ams_colors, first_filament, popup_mode)
    """
    settings_path = explicit or find_settings_file(gcode_path) \
        or find_settings_extended()
    ams_colors: Dict[int, Tuple[int, int, int]] = {}
    first_filament = True
    popup_mode = POPUP_DEFAULT

    if settings_path:
        data = load_settings(settings_path)
        if data:
            ams_colors = get_ams_colors(data)
            first_filament = bool(data.get("first_filament", True))
            popup_mode = str(data.get(SETTINGS_KEY_POPUP, POPUP_DEFAULT)).lower()
            if popup_mode not in (POPUP_AUTO, POPUP_ALWAYS, POPUP_NEVER):
                popup_mode = POPUP_DEFAULT
        else:
            safe_print("[ESP-AMS] 警告：配置文件读不出来（不是有效 JSON？）：%s"
                       % settings_path)
            settings_path = None
    else:
        safe_print("[ESP-AMS] 没找到配置文件 —— 只能按通道号换料，"
                   "颜色对不对全看切片槽和物理通道是不是同一个顺序。"
                   "请双击 esp_ams_tool 配一次通道颜色。")
    return settings_path, ams_colors, first_filament, popup_mode


def format_match_log(gcode_path: str, settings_path: Optional[str],
                     ams_colors, slice_colors, first_filament: bool,
                     log_lines: List[str]) -> str:
    """`.ams.log` 的正文。"""
    out = []
    out.append("[ESP-AMS] 匹配日志  %s" % gcode_path)
    out.append("[ESP-AMS] 配置文件：%s" % (settings_path or "**没找到**"))
    out.append("[ESP-AMS] 项目通道：%d 个   切片颜色：%d 种"
               % (len(ams_colors), len(slice_colors)))
    for ch, c in sorted(ams_colors.items()):
        out.append("[ESP-AMS]   通道%d = %s" % (ch, rgb_to_hex(*c)))
    out.append("[ESP-AMS] 首次换料：%s" % ("开" if first_filament else "关"))
    out.append("-" * 56)
    for l in log_lines:
        out.append("  " + l)
    out.append("-" * 56)
    bad = [l for l in log_lines if "匹配不上" in l]
    if bad:
        out.append("")
        out.append("⚠ 有 %d 处颜色没匹配上（已保留原通道号）。"
                   "要么切片里用的颜色在项目通道里没有，要么需要把阈值放宽。"
                   % len(bad))
    return "\n".join(out) + "\n"


def show_map_window(detail: List[Dict], ams_colors,
                    gcode_path: str, settings_path: Optional[str] = None,
                    mode: str = POPUP_DEFAULT,
                    auto_close_sec: int = POPUP_AUTO_DONE_SEC) -> None:
    """弹一个"切片槽 → 料盘位"的确认窗（复刻 TOP AMS EXE 的交互）。

    ★ 现场要求（2026-09-24）："TOP AMS 用的是 EXE，切片后会弹出一个窗口
      自动映射外部换料通道的颜色，可以拉起一个弹窗让确认" —— 所以默认
      策略是 `always`（每次切片都弹），`auto` 只在需要拍板时弹。

    ★ 窗口**会挡住切片**：切片器在"运行后处理脚本"这一步等本进程退出
      （进度条 95%）。所以
        ① 确认流程本身要一眼能看完（颜色方块 + 通道 + 结论）；
        ② 人走开时必须能自己收场 —— `auto_close_sec` 秒后按当前结论
           自动确认，否则切片会永远卡在 95%。
    """
    try:
        import tkinter as tk
    except Exception:
        return

    try:
        root = tk.Tk()
    except Exception:
        # 无显示环境（或已经从别的 Tk 实例里起来）→ 静默跳过
        return

    p = ui_build_theme(root, ui_resolve_dark(ui_load_pref()))
    ui_apply(root, p)
    root.title("ESP-AMS 耗材丝 ↔ 料盘位 映射确认")
    root.resizable(False, False)
    root.configure(bg=p["bg"])

    # ---------------- 品牌条 ----------------
    brand = tk.Frame(root, bg=p["brand_bg"])
    brand.pack(fill=tk.X, side=tk.TOP)
    bi = tk.Frame(brand, bg=p["brand_bg"])
    bi.pack(fill=tk.X, padx=ui_px(p, 16), pady=ui_px(p, 11))
    lw, lh = ui_px(p, 32), ui_px(p, 28)
    logo = tk.Canvas(bi, width=lw, height=lh, bg=p["brand_bg"],
                     highlightthickness=0, bd=0)
    logo.pack(side=tk.LEFT)
    ui_round_rect(logo, 0, 0, lw, lh, ui_px(p, 9), fill=p["accent"], outline="")
    logo.create_text(lw / 2, lh / 2, text="AMS", fill=p["accent_fg"],
                     font=ui_font(p, 7, True))
    tk.Label(bi, text="耗材丝 ↔ 料盘位 映射确认", bg=p["brand_bg"],
             fg=p["brand_fg"], font=ui_font(p, 13, True)).pack(side=tk.LEFT,
                                                               padx=(10, 0))
    tk.Label(bi, text="按颜色自动对通道", bg=p["brand_bg"], fg=p["brand_sub"],
             font=ui_font(p, 9)).pack(side=tk.RIGHT)

    # ---------------- 底部按钮条（先 pack 占住底） ----------------
    left = tk.IntVar(value=auto_close_sec)
    btnbar = tk.Frame(root, bg=p["bg"])
    btnbar.pack(fill=tk.X, side=tk.BOTTOM, padx=ui_px(p, 16),
                pady=(ui_px(p, 4), ui_px(p, 14)))
    hint_lbl = tk.Label(btnbar, text="", bg=p["bg"], fg=p["faint"],
                        font=ui_font(p, 9))

    # ---------------- 主体 ----------------
    wrap = tk.Frame(root, bg=p["bg"])
    wrap.pack(fill=tk.BOTH, expand=True, padx=ui_px(p, 16), pady=ui_px(p, 14))

    info = tk.Frame(wrap, bg=p["bg"])
    info.pack(fill=tk.X)
    tk.Label(info, text="切片文件", bg=p["bg"], fg=p["faint"],
             font=ui_font(p, 9), width=9, anchor="w") \
        .grid(row=0, column=0, sticky="w")
    tk.Label(info, text=Path(gcode_path).name, bg=p["bg"], fg=p["text"],
             font=ui_font(p, 9)).grid(row=0, column=1, sticky="w")
    tk.Label(info, text="配置文件", bg=p["bg"], fg=p["faint"],
             font=ui_font(p, 9), width=9, anchor="w") \
        .grid(row=1, column=0, sticky="w", pady=(3, 0))
    tk.Label(info,
             text=(Path(settings_path).name if settings_path
                   else "没找到 —— 只能按通道号换料"),
             bg=p["bg"], fg=(p["text"] if settings_path else p["danger"]),
             font=ui_font(p, 9)).grid(row=1, column=1, sticky="w", pady=(3, 0))

    # ---- 映射结果表 ----
    c1, box = ui_card(wrap, p, title="映射结果")
    c1.pack(fill=tk.X, pady=(ui_px(p, 12), 0))

    grid = tk.Frame(box, bg=p["panel"])
    grid.pack(fill=tk.X)

    for c_, htxt in enumerate(("切片槽", "切片里设的颜色",
                               "→ 实际料盘位", "结论")):
        tk.Label(grid, text=htxt, bg=p["panel"], fg=p["faint"],
                 font=ui_font(p, 9)).grid(row=0, column=c_, sticky="w",
                                          padx=(0, ui_px(p, 16)),
                                          pady=(0, ui_px(p, 7)))

    status_text = {
        "ok":       ("✓ 已按颜色对上", "accent"),
        "far":      ("⚠ 色差太大，原样保留", "danger"),
        "none":     ("⚠ 项目里没这个颜色", "danger"),
        "no_slice": ("⚠ 切片没给颜色", "muted"),
        "no_cfg":   ("⚠ 项目未配置颜色", "danger"),
    }
    colmap = {"accent": p["accent"], "danger": p["danger"], "muted": p["muted"]}

    for r, d in enumerate(detail, start=1):
        tk.Label(grid, text="第 %d 个" % (d["idx"] + 1), bg=p["panel"],
                 fg=p["text"], font=ui_font(p, 9, True)) \
            .grid(row=r, column=0, sticky="w", padx=(0, ui_px(p, 16)),
                  pady=ui_px(p, 3))

        cell = tk.Frame(grid, bg=p["panel"])
        cell.grid(row=r, column=1, sticky="w", padx=(0, ui_px(p, 16)),
                  pady=ui_px(p, 3))
        sws = ui_px(p, 22)
        sw = tk.Canvas(cell, width=sws, height=sws, bg=p["panel"],
                       highlightthickness=0, bd=0)
        sw.pack(side=tk.LEFT)
        if d["color"]:
            hexs = rgb_to_hex(*d["color"])
            ui_round_rect(sw, 0, 0, sws, sws, ui_px(p, 6), fill=hexs,
                          outline=ui_shade(hexs, 34 if p["dark"] else -34))
        else:
            ui_round_rect(sw, 0, 0, sws, sws, ui_px(p, 6), fill=p["chip"],
                          outline=p["border"])
        tk.Label(cell,
                 text=(rgb_to_hex(*d["color"]) if d["color"] else "（无）"),
                 bg=p["panel"], fg=p["text"], font=ui_font(p, 9, mono=True)) \
            .pack(side=tk.LEFT, padx=(ui_px(p, 7), 0))

        if d.get("ch"):
            ch_txt = "通道 %d" % d["ch"]
            ch_c = ams_colors.get(d["ch"])
            if ch_c:
                ch_txt += "    %s" % rgb_to_hex(*ch_c)
        else:
            ch_txt = "—"
        tk.Label(grid, text=ch_txt, bg=p["panel"], fg=p["text"],
                 font=ui_font(p, 9)).grid(row=r, column=2, sticky="w",
                                          padx=(0, ui_px(p, 16)),
                                          pady=ui_px(p, 3))

        stxt, skind = status_text.get(d["status"], (d["status"], "muted"))
        tk.Label(grid, text=stxt, bg=p["panel"],
                 fg=colmap.get(skind, p["muted"]), font=ui_font(p, 9, True)) \
            .grid(row=r, column=3, sticky="w", pady=ui_px(p, 3))

    # ---- 结论条 ----
    bad = [d for d in detail if d["status"] != "ok"]
    if bad:
        tip = ("有 %d 个切片槽没匹配上（见上面的 ⚠）。映射还是写进了 G-code，"
               "但它们会按切片槽号原样发 —— 想改就点左下角"
               "「打开配置工具改颜色」，把料盘颜色配成和切片里一致，再重新切片。"
               % len(bad))
        tbg, tfg = p["danger_soft"], p["danger"]
    else:
        tip = "全部按颜色对上，映射已写进 G-code（在文件里搜 ESP-AMS 就能看到）。"
        tbg, tfg = p["accent_soft"], p["accent"]
    tipbox = tk.Frame(wrap, bg=tbg)
    tipbox.pack(fill=tk.X, pady=(ui_px(p, 12), 0))
    tk.Label(tipbox, text=tip, bg=tbg, fg=tfg, font=ui_font(p, 9),
             justify="left", wraplength=ui_px(p, 540)).pack(
                 anchor="w", padx=ui_px(p, 12), pady=ui_px(p, 9))

    # ---- 下次还弹不弹（只改配置文件里的 popup 一个键）----
    pref_var = tk.StringVar(value=mode)
    c2, pbox = ui_card(wrap, p, title="下次切片")
    c2.pack(fill=tk.X, pady=(ui_px(p, 12), 0))

    redraws = []

    def _pref_row(text, val):
        rowf = tk.Frame(pbox, bg=p["panel"], cursor="hand2")
        rowf.pack(fill=tk.X, pady=ui_px(p, 2))
        ds = ui_px(p, 18)
        dot = tk.Canvas(rowf, width=ds, height=ds, bg=p["panel"],
                        highlightthickness=0, bd=0)
        dot.pack(side=tk.LEFT)
        lbl = tk.Label(rowf, text=text, bg=p["panel"], fg=p["muted"],
                       font=ui_font(p, 9))
        lbl.pack(side=tk.LEFT, padx=(ui_px(p, 7), 0))

        def redraw():
            sel = (pref_var.get() == val)
            dot.delete("all")
            if sel:
                dot.create_oval(ui_px(p, 2), ui_px(p, 2), ds - ui_px(p, 2),
                                ds - ui_px(p, 2), fill=p["accent"], outline="")
                dot.create_oval(ds / 2 - ui_px(p, 2.5), ds / 2 - ui_px(p, 2.5),
                                ds / 2 + ui_px(p, 2.5), ds / 2 + ui_px(p, 2.5),
                                fill=p["accent_fg"], outline="")
            else:
                dot.create_oval(ui_px(p, 2), ui_px(p, 2), ds - ui_px(p, 2),
                                ds - ui_px(p, 2), outline=p["border_strong"],
                                width=ui_px(p, 2), fill="")
            lbl.config(fg=(p["text"] if sel else p["muted"]),
                       font=ui_font(p, 9, sel))

        def pick(_e=None):
            if not settings_path:
                return
            pref_var.set(val)
            for fn in redraws:
                fn()

        for w in (rowf, dot, lbl):
            w.bind("<Button-1>", pick)
        redraws.append(redraw)
        redraw()

    _pref_row("还是每次都弹（和 TOP AMS 一样）", POPUP_ALWAYS)
    _pref_row("只在需要拍板时弹（颜色匹配不上 / 有歧义）", POPUP_AUTO)
    if not settings_path:
        tk.Label(pbox, text="（没找到配置文件，这一项改不了）", bg=p["panel"],
                 fg=p["faint"], font=ui_font(p, 9)).pack(anchor="w", pady=(5, 0))

    # ---- 按钮 ----
    def _open_tool():
        """另开一个进程启动配置 GUI —— 本进程不能自己开 Tk 主循环，
        它正处在"给切片器写文件"的关键路径上，卡住就会把切片顶住。"""
        try:
            import subprocess
            if getattr(sys, "frozen", False):
                exe = sys.executable
                subprocess.Popen([exe, "gui"], close_fds=True)
            else:
                subprocess.Popen([sys.executable, os.path.abspath(__file__),
                                  "gui"], close_fds=True)
        except Exception:
            pass
        root.destroy()

    def _confirm():
        # 只在用户真的改了这个选择时才写文件（免得每次切片都动配置文件）
        if settings_path and pref_var.get() != mode:
            update_settings_key(settings_path, SETTINGS_KEY_POPUP, pref_var.get())
        root.destroy()

    ui_button(btnbar, p, "确定", _confirm, kind="accent", bold=True,
              pad=(26, 8)).pack(side=tk.RIGHT)
    ui_button(btnbar, p, "打开配置工具改颜色", _open_tool,
              kind="normal").pack(side=tk.LEFT)
    hint_lbl.pack(side=tk.RIGHT, padx=(0, 12))

    def _tick():
        n = left.get() - 1
        left.set(n)
        if n <= 0:
            safe_print("[ESP-AMS] 确认窗无人操作，已按当前结论自动确认"
                       "（%d 秒）" % auto_close_sec)
            root.destroy()
            return
        hint_lbl.config(text="%d 秒后自动确认" % n)
        root.after(1000, _tick)

    if auto_close_sec > 0:
        hint_lbl.config(text="%d 秒后自动确认" % left.get())
        root.after(1000, _tick)

    # ---- 定宽自适应高，摆到屏幕中偏上（别挡住切片器的进度条） ----
    try:
        root.update_idletasks()
        ui_refresh_scale(root, p)      # 映射后重采，锁出来的宽度才不飘
        w = ui_px(p, 640)
        h = max(root.winfo_reqheight(), ui_px(p, 240))
        # ★ 光调 geometry() 是不够的：窗口还没映射时 Tk 会用**内容的请求尺寸**
        #   覆盖它（实测宽度会随文件名长短在 735~795 之间飘）。必须同时钉住
        #   min/max，宽度才是真的固定。
        root.minsize(w, h)
        root.maxsize(w, h)
        root.geometry("%dx%d" % (w, h))
        root.update_idletasks()
        ui_center(root, w, h)
    except Exception:
        pass

    try:
        root.attributes("-topmost", True)
    except Exception:
        pass
    root.mainloop()


def process_file(in_path: str, out_path: Optional[str] = None,
                 explicit_config: Optional[str] = None,
                 write_back: bool = True,
                 popup_mode: Optional[str] = None,
                 do_popup: bool = True,
                 verbose: bool = True) -> Dict:
    """后处理的**唯一**实现，CLI 与 Bambu Studio 钩子都走这里。

    返回汇总 dict（含 log_lines / detail / summary），方便调用方决定退出码。
    """
    settings_path, ams_colors, first_filament, cfg_popup = \
        load_config_for(in_path, explicit_config)
    mode = (popup_mode or cfg_popup or POPUP_DEFAULT).lower()

    try:
        with open(in_path, 'r', encoding='utf-8', errors='replace') as f:
            content = f.read()
    except OSError as e:
        safe_print("[ESP-AMS] 读不到 G-code：%s（%s）" % (in_path, e))
        return {"ok": False, "error": "read_failed", "log_lines": []}

    slice_colors = parse_slice_colors(content)
    rewritten, log_lines, summary, detail = rewrite_gcode(
        content, ams_colors, slice_colors, first_filament)

    out = out_path or in_path
    if write_back:
        try:
            with open(out, 'w', encoding='utf-8', newline='\n') as f:
                f.write(rewritten)
        except OSError as e:
            safe_print("[ESP-AMS] 写不回 G-code：%s（%s）" % (out, e))
            return {"ok": False, "error": "write_failed", "log_lines": log_lines}

    log_file = ""
    if write_back:
        log_file = out + '.ams.log'
        try:
            with open(log_file, 'w', encoding='utf-8') as lf:
                lf.write(format_match_log(in_path, settings_path, ams_colors,
                                          slice_colors, first_filament, log_lines))
        except OSError:
            log_file = ""

    if verbose:
        safe_print("[ESP-AMS] 配置文件：%s" % (settings_path or "**没找到**"))
        safe_print("[ESP-AMS] 项目通道 %d 个 / 切片颜色 %d 种"
                   % (len(ams_colors), len(slice_colors)))
        for line in log_lines:
            safe_print("[ESP-AMS] " + line)
        safe_print("[ESP-AMS] M140 信令改写了 %d 处（保持原样 %d 处）"
                   % (summary.get("m140_ok", 0), summary.get("m140_miss", 0)))
        if log_file:
            safe_print("[ESP-AMS] 明细日志：%s" % log_file)

    # ---- 弹窗判定 ----
    #   always：每切一次都弹（现场要求的默认值，对齐 TOP AMS 的 EXE）
    #   auto  ：只在需要人拍板时弹（有颜色匹配不上 / 有歧义 / 切片色多于通道）
    need_popup = (mode == POPUP_ALWAYS) or \
                 (mode == POPUP_AUTO and summary.get("needs_human"))
    if do_popup and need_popup and detail:
        if verbose:
            safe_print("[ESP-AMS] 弹映射确认窗（策略=%s）" % mode)
        show_map_window(detail, ams_colors, in_path, settings_path, mode=mode)
    elif do_popup and need_popup and not detail:
        # 切片里连颜色都没解析到 → 弹一个空表没意义，写进日志即可
        safe_print("[ESP-AMS] 没有可展示的映射（切片里没解析到颜色），跳过确认窗")

    return {"ok": True, "settings_path": settings_path, "log_lines": log_lines,
            "summary": summary, "detail": detail, "log_file": log_file,
            "slice_colors": slice_colors, "ams_colors": ams_colors}


def onBeforeWriteGCode(input_file, gcode_path, project, filename, output):
    """Bambu Studio 后处理钩子。

    ⚠️ 这个钩子只在把本文件当 Python 模块挂进切片器时才会被调用；
    通过"偏好设置 → 后处理脚本"配 .bat/.exe 的方式走的是命令行入口。
    两条路共用同一套映射逻辑（rewrite_gcode），结果完全一致。
    """
    settings_path, ams_colors, first_filament, popup_mode = \
        load_config_for(input_file)
    try:
        with open(input_file, 'r', encoding='utf-8', errors='replace') as f:
            content = f.read()
    except OSError:
        output.write("ESP-AMS: 无法读取 G-code 文件\n", encoding='utf-8')
        return

    slice_colors = parse_slice_colors(content)
    rewritten, log_lines, summary, detail = rewrite_gcode(
        content, ams_colors, slice_colors, first_filament)

    for line in log_lines:
        safe_print("[ESP-AMS] " + line)

    try:
        with open(gcode_path + '.ams.log', 'w', encoding='utf-8') as lf:
            lf.write(format_match_log(input_file, settings_path, ams_colors,
                                      slice_colors, first_filament, log_lines))
    except OSError:
        pass

    output.write(rewritten, encoding='utf-8')


# ============================== UI 皮肤（Tkinter 主题） ==============================
#
# 为什么要自己搭一套：
#   ttk 的 `vista` / `xpnative` 主题是**系统原生绘制** —— `style.configure(bg=...)`
#   会被整个忽略。想让按钮 / 单选 / 下拉框都吃我们的配色，只有两条路：
#   ① 强制 `theme_use("clam")` 再把每个控件颜色显式钉死；② 干脆自绘。
#   这里两条都用：ttk 只留 Combobox，其余（按钮、分段选择器、色块）自绘。
#   自绘 = 只用 Canvas 画圆角矩形，**不带任何图片资源**，PyInstaller 不用多带文件。
#
# 明暗：默认跟随系统（winreg 读 AppsUseLightTheme），界面上可手动切，
#       偏好写在安装目录的 `ui_theme.txt`（auto / light / dark）。

UI_THEME_FILENAME = "ui_theme.txt"
UI_PREF_AUTO, UI_PREF_LIGHT, UI_PREF_DARK = "auto", "light", "dark"

# 浅色：冷灰底 + 白卡片 + 拓竹绿强调（#00AE42 是 Bambu Lab 的品牌绿）
_UI_LIGHT = {
    "bg":           "#EDF0F4",   # 窗口底（比卡片深一档，用来撑出层次）
    "panel":        "#FFFFFF",   # 卡片面
    "panel_alt":    "#F5F7FA",   # 卡片内的次级面（色块区底）
    "border":       "#DFE4EA",
    "border_strong": "#C6CDD5",
    "text":         "#1A1F26",
    "muted":        "#66707D",
    "faint":        "#98A2B3",
    "accent":       "#00AE42",
    "accent_hov":   "#00953A",
    "accent_fg":    "#FFFFFF",
    "accent_soft":  "#E4F6EB",
    "chip":         "#E6EAEF",
    "chip_hov":     "#DBE0E6",
    "danger":       "#D93025",
    "danger_soft":  "#FCE9E7",
    "canvas":       "#FFFFFF",
    "brand_bg":     "#161B22",
    "brand_fg":     "#FFFFFF",
    "brand_sub":    "#8E9AAB",
    "disabled_fg":  "#96A0AC",
}

# 深色：近黑底 + 面板抬高一级；强调色略微提亮，保证暗底上的对比度
_UI_DARK = {
    "bg":           "#131619",
    "panel":        "#1C2025",
    "panel_alt":    "#171B1F",
    "border":       "#303843",   # 暗底上边框要亮一档才看得见卡片边界
    "border_strong": "#414A56",
    "text":         "#E7EBF0",
    "muted":        "#96A1AF",
    "faint":        "#6B7683",
    "accent":       "#00C24B",
    "accent_hov":   "#2AD763",
    "accent_fg":    "#04150A",
    "accent_soft":  "#123120",
    "chip":         "#262C33",
    "chip_hov":     "#303843",
    "danger":       "#F0776C",
    "danger_soft":  "#3A211F",
    "canvas":       "#1C2025",
    "brand_bg":     "#0C0F12",
    "brand_fg":     "#FFFFFF",
    "brand_sub":    "#8E9AAB",
    "disabled_fg":  "#5A636E",
}

_UI_FONTS: Dict[str, str] = {}


def ui_pref_path() -> Path:
    """主题偏好的存放位置（跟 EXE / 脚本同目录，一并与安装目录进退）。"""
    try:
        return _app_base() / UI_THEME_FILENAME
    except Exception:
        return Path.home() / UI_THEME_FILENAME


def ui_system_dark() -> bool:
    """系统是不是深色主题。读不到一律当**浅色** —— 浅色在任何环境下都不会看不清。"""
    if os.name != "nt":
        return False
    try:
        import winreg
        key = winreg.OpenKey(
            winreg.HKEY_CURRENT_USER,
            r"Software\Microsoft\Windows\CurrentVersion\Themes\Personalize")
        try:
            val, _ = winreg.QueryValueEx(key, "AppsUseLightTheme")
            return int(val) == 0
        finally:
            winreg.CloseKey(key)
    except Exception:
        return False


def ui_load_pref() -> str:
    try:
        t = ui_pref_path().read_text(encoding="utf-8").strip().lower()
        if t in (UI_PREF_AUTO, UI_PREF_LIGHT, UI_PREF_DARK):
            return t
    except OSError:
        pass
    return UI_PREF_AUTO


def ui_save_pref(mode: str) -> None:
    try:
        ui_pref_path().write_text(mode, encoding="utf-8")
    except OSError:
        pass


def ui_resolve_dark(pref: str) -> bool:
    if pref == UI_PREF_DARK:
        return True
    if pref == UI_PREF_LIGHT:
        return False
    return ui_system_dark()


def ui_fonts(root) -> Dict[str, str]:
    """挑本机真实存在的字体。

    不能直接写死 \"Microsoft YaHei UI\"：字体族不存在时 Tk 会**静默**回退到
    一个很难看的默认字体（不是报错），界面就毁了。所以先枚举再挑。
    """
    if _UI_FONTS:
        return _UI_FONTS
    fams = set()
    try:
        import tkinter.font as tkfont
        fams = set(tkfont.families(root))
    except Exception:
        pass

    def pick(cands, fallback):
        for c in cands:
            if c in fams:
                return c
        return fallback

    _UI_FONTS["ui"] = pick(
        ["Microsoft YaHei UI", "Microsoft YaHei", "微软雅黑",
         "Segoe UI", "PingFang SC", "Noto Sans CJK SC"], "Arial")
    _UI_FONTS["mono"] = pick(
        ["Cascadia Mono", "Consolas", "JetBrains Mono",
         "DejaVu Sans Mono", "Courier New"], "Courier")
    return _UI_FONTS


def _hex_to_rgb_t(h: str) -> Tuple[int, int, int]:
    h = h.lstrip("#")
    return int(h[0:2], 16), int(h[2:4], 16), int(h[4:6], 16)


def ui_shade(h: str, amount: int) -> str:
    """把颜色整体调亮/调暗 amount（-255..255），用来算 hover 色。"""
    r, g, b = _hex_to_rgb_t(h)
    f = lambda v: max(0, min(255, v + amount))
    return "#%02X%02X%02X" % (f(r), f(g), f(b))


def ui_mix(h1: str, h2: str, t: float) -> str:
    """在两色之间线性插值（t=0 取 h1，t=1 取 h2）。用于把色块"洗淡"成禁用态。"""
    a, b = _hex_to_rgb_t(h1), _hex_to_rgb_t(h2)
    return "#%02X%02X%02X" % tuple(
        int(round(a[i] + (b[i] - a[i]) * t)) for i in range(3))


def ui_disp_w(text: str) -> int:
    """字符串在界面里占多少"格"（CJK 全角算 2 格，ASCII 算 1 格）。

    ★ 为什么不能直接用 `len()`：一个中文字符的显示宽度约等于两个西文字符。
      按 `len()` 截断时，"58 个字符的中文路径"实际占 90+ 格宽，
      窗口照样被顶宽 —— 第一版就是这么翻车的。
    """
    n = 0
    for ch in text:
        o = ord(ch)
        n += 2 if (0x1100 <= o <= 0x115F or 0x2E80 <= o <= 0xA4CF
                   or 0xAC00 <= o <= 0xD7A3 or 0xF900 <= o <= 0xFAFF
                   or 0xFE30 <= o <= 0xFE6F or 0xFF00 <= o <= 0xFF60
                   or 0xFFE0 <= o <= 0xFFE6) else 1
    return n


def ui_shorten(text: str, limit: int = 56) -> str:
    """按显示宽度中间省略（limit 单位是"格"）。

    ★ 为什么必须做：Tk 的 Label 宽度是**内容决定**的，一行完整长路径
      （`D:\\kx\\AI提示词\\...\\tools\\filament_settings.json`）会把整个窗口
      顶宽 —— 实测能顶到 870px，而且随目录深浅变化，窗口宽度飘忽不定。
      完整路径仍然靠"点一下复制"给用户，不必挤在界面上。
    """
    if ui_disp_w(text) <= limit:
        return text
    left, w = [], 0
    for ch in text:
        cw = ui_disp_w(ch)
        if w + cw > limit // 2:
            break
        left.append(ch)
        w += cw
    right, w2 = [], 0
    for ch in reversed(text):
        cw = ui_disp_w(ch)
        if w2 + cw > max(6, limit - w - 1):
            break
        right.append(ch)
        w2 += cw
    return "".join(left) + "…" + "".join(reversed(right))


def ui_contrast_fg(c: Tuple[int, int, int]) -> str:
    """给一个底色，返回在上面看得清的文字色（按感知亮度分档，不是简单均值）。"""
    r, g, b = c
    lum = 0.299 * r + 0.587 * g + 0.114 * b
    return "#FFFFFF" if lum < 140 else "#1A1F26"


def ui_build_theme(root, dark: bool) -> Dict[str, str]:
    """组装一套可用的主题字典（配色 + 字体 + DPI 比例）。切换主题时重新调用即可。"""
    p = dict(_UI_DARK if dark else _UI_LIGHT)
    p["dark"] = dark          # type: ignore[assignment]
    fonts = ui_fonts(root)
    p["fui"] = fonts["ui"]
    p["fmono"] = fonts["mono"]
    # ★ DPI：Tk 的**字体**是按 point 算的，会自动跟着屏幕 DPI 放大；
    #   **像素**尺寸（色块、圆角、内边距）不会。不补这一课，在 125% / 150%
    #   缩放的屏上就是"字大格子小"——色块里的通道号和 hex 会挤成一团。
    #   PyInstaller 打出来的 EXE 默认带 dpiAware，所以这不是小概率情况。
    try:
        p["scale"] = max(1.0, root.winfo_fpixels("1i") / 96.0)
    except Exception:
        p["scale"] = 1.0
    return p


def ui_refresh_scale(root, p) -> float:
    """重新采一次屏幕缩放。

    ★ 为什么需要"再采一次"：窗口**还没映射**时 Tk 报的 DPI 可能是默认 96
      （或者上一次查询的缓存值），按它算出来的锁定尺寸会随机飘 ——
      实测同一个确认窗在两次运行里能差 55px。凡是**要按缩放锁死尺寸**的
      地方，都要在 `update_idletasks()` 之后再刷一次。
    """
    try:
        p["scale"] = max(1.0, root.winfo_fpixels("1i") / 96.0)
    except Exception:
        pass
    return float(p.get("scale", 1.0))


def ui_px(p, n: float) -> int:
    """把"设计像素"（按 96 DPI 画的稿）换算成本机物理像素。"""
    try:
        return int(round(n * float(p.get("scale", 1.0))))
    except Exception:
        return int(n)


def ui_apply(root, p: Dict[str, str]) -> None:
    """把主题套到 root 上（含 ttk 那一小块）。

    注意：每个 ttk 控件的颜色都必须显式写死 —— clam 的继承是**逐元素**的，
    `.` 上配了不保证子样式吃到。
    """
    from tkinter import ttk

    root.configure(bg=p["bg"])

    st = ttk.Style(root)
    try:
        st.theme_use("clam")
    except Exception:
        pass

    st.configure(".", background=p["bg"], foreground=p["text"],
                 fieldbackground=p["panel"], font=(p["fui"], 10),
                 borderwidth=0, focuscolor=p["bg"])

    # ---- Combobox（唯一保留的 ttk 控件；它的下拉列表是 tk Listbox，要另外配）----
    st.configure("TCombobox",
                 fieldbackground=p["panel"], background=p["chip"],
                 foreground=p["text"], arrowcolor=p["muted"],
                 bordercolor=p["border_strong"], lightcolor=p["panel"],
                 darkcolor=p["panel"], selectbackground=p["panel"],
                 selectforeground=p["text"], padding=(8, 5),
                 relief="flat")
    st.map("TCombobox",
           fieldbackground=[("readonly", p["panel"]), ("disabled", p["panel_alt"])],
           foreground=[("disabled", p["faint"])],
           bordercolor=[("focus", p["accent"])],
           lightcolor=[("focus", p["accent"])],
           darkcolor=[("focus", p["accent"])],
           arrowcolor=[("active", p["text"])])
    # 下拉列表（原生 Listbox，ttk 管不到，只能走 option 数据库）
    root.option_add("*TCombobox*Listbox.background", p["panel"])
    root.option_add("*TCombobox*Listbox.foreground", p["text"])
    root.option_add("*TCombobox*Listbox.selectBackground", p["accent"])
    root.option_add("*TCombobox*Listbox.selectForeground", p["accent_fg"])
    root.option_add("*TCombobox*Listbox.borderWidth", 0)
    root.option_add("*TCombobox*Listbox.font", (p["fui"], 10))

    # ---- 滚动条（G-code 预览窗里用）----
    st.configure("Vertical.TScrollbar",
                 background=p["chip"], troughcolor=p["panel_alt"],
                 bordercolor=p["panel_alt"], arrowcolor=p["muted"],
                 lightcolor=p["chip"], darkcolor=p["chip"],
                 relief="flat", arrowsize=12)
    st.map("Vertical.TScrollbar", background=[("active", p["chip_hov"])])


def ui_font(p, size=10, bold=False, mono=False):
    """取字体元组：ui_font(p, 10, True) / ui_font(p, 9, mono=True)。"""
    fam = p["fmono"] if mono else p["fui"]
    return (fam, size, "bold") if bold else (fam, size)


def ui_round_rect(cv, x1, y1, x2, y2, r, **kw):
    """在 Canvas 上画圆角矩形（用 smooth 多边形的标准做法，无需图片）。"""
    r = max(0, min(r, (x2 - x1) / 2, (y2 - y1) / 2))
    pts = [
        x1 + r, y1, x2 - r, y1, x2, y1, x2, y1 + r,
        x2, y2 - r, x2, y2, x2 - r, y2, x1 + r, y2,
        x1, y2, x1, y2 - r, x1, y1 + r, x1, y1,
    ]
    return cv.create_polygon(pts, smooth=True, **kw)


def ui_center(win, width=None, height=None) -> None:
    """把窗口摆到屏幕中偏上（不垂直居中，弹窗更贴近视觉重心）。"""
    win.update_idletasks()
    w = width or win.winfo_width()
    h = height or win.winfo_height()
    x = max(0, (win.winfo_screenwidth() - w) // 2)
    y = max(0, int((win.winfo_screenheight() - h) * 0.32))
    win.geometry("+%d+%d" % (x, y))


def ui_card(parent, p, title=None, subtitle=None, pad=14):
    """一张卡片。返回 (外框, 内容区)。

    1px 边框是靠**外层 Frame 的底色**透出来的（内层四周留 1px），
    比 tk 的 highlightthickness 在缩放时更稳定。
    """
    import tkinter as tk

    pad = ui_px(p, pad)
    gap8 = ui_px(p, 8)

    outer = tk.Frame(parent, bg=p["border"])
    inner = tk.Frame(outer, bg=p["panel"])
    inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

    if title:
        head = tk.Frame(inner, bg=p["panel"])
        head.pack(fill=tk.X, padx=pad, pady=(pad, 0))
        tk.Label(head, text=title, bg=p["panel"], fg=p["text"],
                 font=ui_font(p, 10, True)).pack(side=tk.LEFT)
        if subtitle:
            tk.Label(head, text=subtitle, bg=p["panel"], fg=p["faint"],
                     font=ui_font(p, 9)).pack(side=tk.LEFT, padx=(gap8, 0))

    body = tk.Frame(inner, bg=p["panel"])
    body.pack(fill=tk.BOTH, expand=True, padx=pad,
              pady=(gap8 if title else pad, pad))
    return outer, body


def ui_button(parent, p, text, command, kind="normal", pad=(14, 7),
              size=10, bold=False, width=None):
    """扁平按钮（tk.Button + 手工 hover）。

    为什么不用 ttk.Button：clam 的按钮有硬编码的内阴影和 focus 虚框，
    想彻底压平要动 layout，还不如直接用 tk.Button 把颜色全握在手里。
    """
    import tkinter as tk

    if kind == "accent":
        bg, fg, hov = p["accent"], p["accent_fg"], p["accent_hov"]
    elif kind == "danger":
        bg, fg, hov = p["panel"], p["danger"], p["danger_soft"]
    elif kind == "ghost":
        bg, fg, hov = p["panel"], p["muted"], p["chip"]
    else:
        bg, fg, hov = p["chip"], p["text"], p["chip_hov"]

    btn = tk.Button(parent, text=text, command=command, bg=bg, fg=fg,
                    activebackground=hov, activeforeground=fg,
                    relief="flat", bd=0, highlightthickness=0,
                    padx=ui_px(p, pad[0]), pady=ui_px(p, pad[1]), cursor="hand2",
                    font=ui_font(p, size, bold), **({"width": width} if width else {}))
    btn.bind("<Enter>", lambda e: btn.config(bg=hov))
    btn.bind("<Leave>", lambda e: btn.config(bg=bg))
    return btn


def ui_segment(parent, p, options, value, on_pick,
               item_w=64, height=32, gap=4):
    """胶囊式分段选择器（自绘）。

    返回 (canvas, set_value)。options 是 [(label, value), ...]。
    选中的那格填强调色 + 深色字，其余透明底 + 灰字，hover 时描边。
    """
    import tkinter as tk

    item_w = ui_px(p, item_w)
    height = ui_px(p, height)
    gap = ui_px(p, gap)
    radius = ui_px(p, 8)

    total_w = len(options) * item_w + (len(options) - 1) * gap
    cv = tk.Canvas(parent, width=total_w, height=height,
                   bg=p["panel"], highlightthickness=0, bd=0, cursor="hand2")
    state = {"value": value, "items": []}

    def draw():
        cv.delete("all")
        state["items"] = []
        for i, (label, val) in enumerate(options):
            x = i * (item_w + gap)
            sel = (state["value"] == val)
            if sel:
                fill, fg, outline, ow = p["accent"], p["accent_fg"], p["accent"], 0
            else:
                fill, fg, outline, ow = p["panel_alt"], p["muted"], p["border"], 1
            rid = ui_round_rect(cv, x + 0.5, 0.5, x + item_w - 0.5, height - 0.5,
                                radius, fill=fill, outline=outline, width=ow)
            tid = cv.create_text(x + item_w / 2, height / 2, text=label,
                                 fill=fg, font=ui_font(p, 10, True))
            state["items"].append((x, x + item_w, val, rid, tid, sel))

    def hit(event):
        for x1, x2, val, _r, _t, _s in state["items"]:
            if x1 <= event.x <= x2:
                return val
        return None

    def on_click(event):
        val = hit(event)
        if val is not None and val != state["value"]:
            state["value"] = val
            draw()
            on_pick(val)

    def on_move(event):
        val = hit(event)
        cv.configure(cursor="hand2" if val is not None else "arrow")

    cv.bind("<Button-1>", on_click)
    cv.bind("<Motion>", on_move)

    def set_value(val):
        state["value"] = val
        draw()

    draw()
    return cv, set_value


def ui_badge(parent, p, text="", kind="ok"):
    """小圆角状态徽章。返回 (canvas, set_badge(text, kind))。

    实现要点：文字**只建一次**，宽度靠 bbox 量出来再改 canvas 宽度；
    底色用 `tags="bg"` 画在文字下面（`tag_lower`）。这样重设文字不用重建画布，
    也不会依赖 `find_all()` 的返回顺序（那个顺序很脆）。
    """
    import tkinter as tk

    colors = {
        "ok":   (p["accent_soft"], p["accent"]),
        "warn": (p["danger_soft"], p["danger"]),
        "info": (p["chip"], p["muted"]),
    }
    bh = ui_px(p, 22)
    bx = ui_px(p, 10)
    bpad = ui_px(p, 20)
    cv = tk.Canvas(parent, bg=p["panel"], highlightthickness=0, bd=0, height=bh)
    tid = cv.create_text(bx, bh // 2, text=text, anchor="w",
                         font=ui_font(p, 9, True))

    def set_badge(new_text, new_kind="info"):
        bg, fg = colors.get(new_kind, colors["info"])
        cv.itemconfig(tid, text=new_text, fill=fg)
        box = cv.bbox(tid) or (0, 0, 40, bh)
        w = (box[2] - box[0]) + bpad
        cv.config(width=w)
        cv.delete("bg")
        ui_round_rect(cv, 0, 0, w, bh, bh // 2, fill=bg, outline=bg, tags="bg")
        cv.tag_lower("bg")

    set_badge(text, kind)
    return cv, set_badge


def ui_scroll_text(parent, p, height=16):
    """带滚动条的只读文本框（日志 / 预览用），配色跟着主题走。"""
    import tkinter as tk
    from tkinter import ttk

    wrap = tk.Frame(parent, bg=p["border"])
    inner = tk.Frame(wrap, bg=p["panel"])
    inner.pack(fill=tk.BOTH, expand=True, padx=1, pady=1)

    txt = tk.Text(inner, wrap=tk.WORD, font=ui_font(p, 9, mono=True),
                  bg=p["panel"], fg=p["text"], insertbackground=p["text"],
                  relief="flat", bd=0, highlightthickness=0,
                  height=height, padx=ui_px(p, 10), pady=ui_px(p, 8),
                  selectbackground=p["accent"], selectforeground=p["accent_fg"])
    sb = ttk.Scrollbar(inner, orient="vertical", command=txt.yview,
                       style="Vertical.TScrollbar")
    txt.configure(yscrollcommand=sb.set)
    sb.pack(side=tk.RIGHT, fill=tk.Y)
    txt.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)

    # 行号式的左侧留白感：给首行加一点点内边距
    txt.tag_configure("dim", foreground=p["faint"])
    txt.tag_configure("ok", foreground=p["accent"])
    txt.tag_configure("warn", foreground=p["danger"])
    return wrap, txt


# ============================== GUI（Tkinter） ==============================

def _run_gui():
    """配置界面（浅色/深色双主题，默认跟随系统）。

    皮肤策略：**切换主题 = 整体重建控件**（`build()` 再跑一遍）。
    比"遍历所有控件挨个 set 颜色"可靠得多 —— 数据都放在 `state` 里，
    重建不丢，也不会漏掉某个控件。
    """
    import tkinter as tk
    from tkinter import ttk, filedialog, messagebox, colorchooser

    # ---------------- 状态（跨主题重建保持） ----------------
    state = {
        "data": default_settings(8),
        "ch_count": 8,
        "dir": str(Path.home() / "ESP_AMS"),
        "pref": ui_load_pref(),
        "config_name": SETTINGS_FILENAME,
        "dirty": False,
        "msg": "",
    }
    TITLE_BASE = "ESP-AMS 换色配置工具"

    root = tk.Tk()
    root.title(TITLE_BASE)
    root.resizable(False, False)
    root.withdraw()            # 布局算完再显示，免得"半成品"闪一下

    th = {"p": None}
    holder = {"win": None, "status": None}

    def _mark_dirty(flag=True, msg=""):
        state["dirty"] = flag
        state["msg"] = msg
        root.title(TITLE_BASE + ("  •" if flag else ""))
        if holder["status"]:
            holder["status"]()

    def _set_theme(pref):
        if pref == state["pref"]:
            return
        state["pref"] = pref
        ui_save_pref(pref)
        build()

    # ================= 界面构建 =================
    def build():
        p = ui_build_theme(root, ui_resolve_dark(state["pref"]))
        th["p"] = p
        ui_apply(root, p)
        ui_refresh_scale(root, p)

        if holder["win"] is not None:
            holder["win"].destroy()
        win = tk.Frame(root, bg=p["bg"])
        win.pack(fill=tk.BOTH, expand=True)
        holder["win"] = win

        first_var = tk.BooleanVar(value=bool(state["data"].get("first_filament", True)))

        # ---------- 品牌栏 ----------
        brand = tk.Frame(win, bg=p["brand_bg"])
        brand.pack(fill=tk.X, side=tk.TOP)
        bi = tk.Frame(brand, bg=p["brand_bg"])
        bi.pack(fill=tk.X, padx=ui_px(p, 16), pady=ui_px(p, 13))

        lw, lh = ui_px(p, 36), ui_px(p, 32)
        logo = tk.Canvas(bi, width=lw, height=lh, bg=p["brand_bg"],
                         highlightthickness=0, bd=0)
        logo.pack(side=tk.LEFT)
        ui_round_rect(logo, 0, 0, lw, lh, ui_px(p, 10),
                      fill=p["accent"], outline="")
        logo.create_text(lw / 2, lh / 2, text="AMS", fill=p["accent_fg"],
                         font=ui_font(p, 8, True))

        tb = tk.Frame(bi, bg=p["brand_bg"])
        tb.pack(side=tk.LEFT, padx=(11, 0))
        tk.Label(tb, text="ESP-AMS 换色配置", bg=p["brand_bg"],
                 fg=p["brand_fg"], font=ui_font(p, 14, True)).pack(anchor="w")
        tk.Label(tb, text="切片里颜色随便设 · 自动匹配到正确料盘通道",
                 bg=p["brand_bg"], fg=p["brand_sub"],
                 font=ui_font(p, 9)).pack(anchor="w", pady=(2, 0))

        seg_wrap = tk.Frame(bi, bg=p["brand_bg"])
        seg_wrap.pack(side=tk.RIGHT)
        pb = dict(p)
        pb.update({"panel": p["brand_bg"],
                   "panel_alt": "#1B222B" if p["dark"] else "#242C36",
                   "border": "#39424E", "muted": p["brand_sub"]})
        cv_th, _set_th_seg = ui_segment(
            seg_wrap, pb,
            [("跟随系统", UI_PREF_AUTO), ("浅色", UI_PREF_LIGHT),
             ("深色", UI_PREF_DARK)],
            state["pref"], _set_theme, item_w=64, height=28)
        cv_th.pack()

        # ---------- 状态栏（先 pack，占住最底） ----------
        sbar = tk.Frame(win, bg=p["panel_alt"])
        sbar.pack(fill=tk.X, side=tk.BOTTOM)
        tk.Frame(win, bg=p["border"], height=1).pack(fill=tk.X, side=tk.BOTTOM)

        st_var = tk.StringVar(value="就绪")
        dir_var = tk.StringVar()
        path_var = tk.StringVar()

        def _full_path():
            return os.path.join(state["dir"], state["config_name"])

        def _sync_status():
            if state["msg"]:
                st_var.set(ui_shorten(state["msg"], 62))
            elif state["dirty"]:
                st_var.set("● 有未保存的改动 —— 点右下角「保存配置」")
            else:
                st_var.set("就绪")
            # ★ 一律截断：Label 的宽度是由内容决定的，把完整长路径贴上去
            #   会把整个窗口顶宽（实测 573 → 760），而且随目录变浅变深。
            dir_var.set(ui_shorten(state["dir"], 44))
            path_var.set("存放位置（点一下复制完整路径）：%s"
                         % ui_shorten(_full_path(), 58))

        holder["status"] = _sync_status

        tk.Label(sbar, textvariable=st_var, bg=p["panel_alt"], fg=p["muted"],
                 font=ui_font(p, 9)).pack(side=tk.LEFT, padx=ui_px(p, 16),
                                         pady=ui_px(p, 7))
        tk.Label(sbar, textvariable=dir_var, bg=p["panel_alt"], fg=p["faint"],
                 font=ui_font(p, 9)).pack(side=tk.RIGHT, padx=ui_px(p, 16))

        # ---------- 底部操作条（BOTTOM，落在状态栏上面） ----------
        foot = tk.Frame(win, bg=p["bg"])
        foot.pack(fill=tk.X, side=tk.BOTTOM, padx=ui_px(p, 16),
                  pady=(ui_px(p, 10), ui_px(p, 12)))

        # ---------- 主体 ----------
        body = tk.Frame(win, bg=p["bg"])
        body.pack(fill=tk.BOTH, expand=True, side=tk.TOP, padx=ui_px(p, 16),
                  pady=(ui_px(p, 12), 0))

        # ===== 卡片 1：料盘颜色 =====
        c1, box1 = ui_card(body, p, title="料盘颜色",
                           subtitle="左键改色 · 右键启用/禁用")
        c1.pack(fill=tk.X)

        def _apply_channel_count(n):
            old = state["data"]
            nd = default_settings(n)
            # ★ 保留与通道数无关的键（first_filament / popup / …）——
            #   否则切一下通道数就把「每次切片弹窗」这类偏好悄悄抹掉了
            for k, v in old.items():
                if k not in ("channel_count", "materials"):
                    nd[k] = v
            old_mats = old.get("materials", [])
            for i in range(min(n, len(old_mats))):
                nd["materials"][i]["color"] = old_mats[i].get(
                    "color", nd["materials"][i]["color"])
                nd["materials"][i]["enabled"] = bool(
                    old_mats[i].get("enabled", True))
            state["data"] = nd
            state["ch_count"] = n
            _draw_colors()
            _mark_dirty(msg="已切到 %d 通道（还没保存）" % n)

        row = tk.Frame(box1, bg=p["panel"])
        row.pack(fill=tk.X, pady=(0, 11))
        cv_ch, set_ch = ui_segment(
            row, p, [("4 通道", 4), ("8 通道", 8), ("16 通道", 16)],
            state["ch_count"], _apply_channel_count)
        cv_ch.pack(side=tk.LEFT)
        badge, set_badge = ui_badge(row, p, "", "info")
        badge.pack(side=tk.RIGHT, pady=ui_px(p, 3))

        # ★ 像素尺寸一律过 ui_px：Tk 的**字体**按 point 走、会自己跟着屏幕
        #   DPI 放大，**像素**尺寸不会。不补这一课，高 DPI 屏上就是
        #   "字大格子小"，色块里的通道号和 hex 会挤成一团。
        GAP = ui_px(p, 7)
        CELL_W = ui_px(p, 58)
        CELL_H = ui_px(p, 54)
        CELL_R = ui_px(p, 11)
        COLS = 8
        CANVAS_W = COLS * (CELL_W + GAP) - GAP
        cv = tk.Canvas(box1, width=CANVAS_W, height=CELL_H, bg=p["panel"],
                       highlightthickness=0, bd=0)
        cv.pack(anchor="w")

        tk.Label(box1, text="块上的数字 = 料盘位（通道号），下面一行是它当前的颜色值",
                 bg=p["panel"], fg=p["faint"],
                 font=ui_font(p, 9)).pack(anchor="w", pady=(ui_px(p, 9), 0))

        def _hover(rec, on):
            cv.itemconfig(rec["poly"],
                          outline=(p["accent"] if on else rec["edge"]),
                          width=(2 if on else 1))
            cv.configure(cursor="hand2" if on else "arrow")

        def _pick_color(rec):
            res = colorchooser.askcolor(initialcolor=rgb_to_hex(*rec["color"]),
                                        parent=root,
                                        title="通道 %d 的颜色" % rec["ch"])
            if res and res[1]:
                rgb = hex_to_rgb(res[1])
                if rgb:
                    for m in state["data"]["materials"]:
                        if int(m.get("channel", -1)) == rec["ch"]:
                            m["color"] = list(rgb)
                            break
                    _draw_colors()
                    _mark_dirty(msg="已改通道 %d 的颜色（还没保存）" % rec["ch"])

        def _toggle_enabled(rec):
            for m in state["data"]["materials"]:
                if int(m.get("channel", -1)) == rec["ch"]:
                    m["enabled"] = not bool(m.get("enabled", True))
                    break
            _draw_colors()
            _mark_dirty(msg="通道 %d 已%s（还没保存）"
                            % (rec["ch"], "禁用" if rec["enabled"] else "启用"))

        def _draw_colors():
            cv.delete("all")
            n = state["ch_count"]
            mats = state["data"].setdefault("materials", [])
            while len(mats) < n:
                mats.append({"channel": len(mats) + 1,
                             "color": [128, 128, 128], "enabled": True})
            rows = (n + COLS - 1) // COLS
            cv.config(height=rows * (CELL_H + GAP) - GAP)
            n_en = 0

            for i in range(n):
                m = mats[i]
                ch = int(m.get("channel", i + 1))
                col = tuple(m.get("color", [128, 128, 128]))
                en = bool(m.get("enabled", True))
                n_en += 1 if en else 0
                rr, cc = divmod(i, COLS)
                x, y = cc * (CELL_W + GAP), rr * (CELL_H + GAP)
                hexs = rgb_to_hex(*col)

                if en:
                    fill = hexs
                    fg_main = ui_contrast_fg(col)
                    fg_sub = ui_mix(hexs, fg_main, 0.45)
                    edge = ui_shade(hexs, 34 if p["dark"] else -34)
                else:
                    fill = ui_mix(hexs, p["panel"], 0.80)
                    fg_main = fg_sub = p["disabled_fg"]
                    edge = p["border"]

                poly = ui_round_rect(cv, x, y, x + CELL_W, y + CELL_H, CELL_R,
                                     fill=fill, outline=edge, width=1)
                t1 = cv.create_text(x + CELL_W / 2,
                                    y + CELL_H / 2 - ui_px(p, 7),
                                    text=str(ch), fill=fg_main,
                                    font=ui_font(p, 15, True))
                t2 = cv.create_text(x + CELL_W / 2,
                                    y + CELL_H - ui_px(p, 12),
                                    text=(hexs if en else "已禁用"),
                                    fill=fg_sub, font=ui_font(p, 7))
                if not en:
                    pad = ui_px(p, 11)
                    # 斜线只走色块**上半部**：压到下面那行"已禁用"就成了乱码
                    cv.create_line(x + pad, y + CELL_H * 0.56,
                                   x + CELL_W - pad, y + pad,
                                   fill=ui_mix(fill, p["text"], 0.46),
                                   width=ui_px(p, 2), capstyle="round")

                rec = {"poly": poly, "edge": edge, "ch": ch,
                       "color": col, "enabled": en}
                for item in (poly, t1, t2):
                    cv.tag_bind(item, "<Button-1>",
                                lambda e, r=rec: _pick_color(r))
                    cv.tag_bind(item, "<Button-3>",
                                lambda e, r=rec: _toggle_enabled(r))
                for item in (poly, t1):
                    cv.tag_bind(item, "<Enter>",
                                lambda e, r=rec: _hover(r, True))
                    cv.tag_bind(item, "<Leave>",
                                lambda e, r=rec: _hover(r, False))

            set_badge("%d / %d 启用" % (n_en, n),
                      "ok" if n_en == n else "warn")
            _sync_status()

        # ===== 卡片 2：配置文件 =====
        c2, box2 = ui_card(body, p, title="配置文件",
                           subtitle="后处理脚本按名字读它")
        c2.pack(fill=tk.X, pady=(ui_px(p, 12), 0))

        r2 = tk.Frame(box2, bg=p["panel"])
        r2.pack(fill=tk.X)

        cfg_var = tk.StringVar(value=state["config_name"])
        combo = ttk.Combobox(r2, textvariable=cfg_var, width=23,
                             state="readonly", font=ui_font(p, 10),
                             values=[SETTINGS_FILENAME])
        combo.pack(side=tk.LEFT)
        combo.bind("<<ComboboxSelected>>", lambda e: _load_config(cfg_var.get()))

        def _refresh_config_names():
            names = []
            try:
                names = sorted(f for f in os.listdir(state["dir"])
                               if f.lower().endswith(".json"))
            except OSError:
                pass
            if not names:
                names = [SETTINGS_FILENAME]
            combo["values"] = names
            if state["config_name"] not in names:
                state["config_name"] = names[0]
            cfg_var.set(state["config_name"])

        def _load_config(name):
            if not name:
                return
            data = load_settings(os.path.join(state["dir"], name))
            if not data:
                messagebox.showerror("无法打开",
                                     "文件不是有效的 AMS 配置文件", parent=root)
                return
            state["data"] = data
            state["ch_count"] = int(data.get("channel_count", 8))
            state["config_name"] = name
            set_ch(state["ch_count"])
            first_var.set(bool(data.get("first_filament", True)))
            _sync_first()
            _draw_colors()
            _mark_dirty(False, msg="已载入 %s" % name)

        def _new_config():
            name = "default_%dch.json" % state["ch_count"]
            state["data"] = default_settings(state["ch_count"])
            state["config_name"] = name
            _refresh_config_names()
            _draw_colors()
            _mark_dirty(True, msg="已新建 %s（点「保存配置」写盘）" % name)

        def _open_config():
            path = filedialog.askopenfilename(
                parent=root, title="打开配置文件",
                filetypes=[("JSON", "*.json"), ("所有文件", "*.*")])
            if not path:
                return
            state["dir"] = str(Path(path).parent)
            state["config_name"] = os.path.basename(path)
            _refresh_config_names()
            _load_config(state["config_name"])

        def _open_dir():
            d = filedialog.askdirectory(parent=root, title="选择配置目录")
            if not d:
                return
            state["dir"] = d
            _refresh_config_names()
            _draw_colors()
            _sync_status()

        ui_button(r2, p, "打开…", _open_config).pack(side=tk.LEFT,
                                                     padx=(ui_px(p, 8), 0))
        ui_button(r2, p, "新建", _new_config).pack(side=tk.LEFT,
                                                   padx=(ui_px(p, 6), 0))
        ui_button(r2, p, "换目录", _open_dir).pack(side=tk.LEFT,
                                                     padx=(ui_px(p, 6), 0))

        path_lbl = tk.Label(box2, textvariable=path_var, bg=p["panel"],
                            fg=p["faint"], font=ui_font(p, 9), cursor="hand2")
        path_lbl.pack(anchor="w", pady=(ui_px(p, 9), 0))

        def _copy_path(_e=None):
            try:
                root.clipboard_clear()
                root.clipboard_append(_full_path())
                _mark_dirty(state["dirty"],
                            msg="已复制完整路径：%s" % _full_path())
            except Exception:
                pass

        path_lbl.bind("<Button-1>", _copy_path)
        path_lbl.bind("<Enter>", lambda e: path_lbl.config(fg=p["muted"]))
        path_lbl.bind("<Leave>", lambda e: path_lbl.config(fg=p["faint"]))

        # ===== 对话框 =====
        def _show_export_dialog(path):
            p2 = th["p"]
            dlg = tk.Toplevel(root)
            dlg.title("配置已保存")
            dlg.configure(bg=p2["bg"])
            dlg.resizable(False, False)
            dlg.transient(root)

            wrap = tk.Frame(dlg, bg=p2["bg"])
            wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)

            top = tk.Frame(wrap, bg=p2["bg"])
            top.pack(fill=tk.X)
            tick = tk.Canvas(top, width=36, height=36, bg=p2["bg"],
                             highlightthickness=0, bd=0)
            tick.pack(side=tk.LEFT)
            tick.create_oval(1, 1, 35, 35, fill=p2["accent_soft"], outline="")
            tick.create_text(18, 18, text="✓", fill=p2["accent"],
                             font=ui_font(p2, 15, True))
            tt = tk.Frame(top, bg=p2["bg"])
            tt.pack(side=tk.LEFT, padx=(10, 0))
            tk.Label(tt, text="配置已保存", bg=p2["bg"], fg=p2["text"],
                     font=ui_font(p2, 12, True)).pack(anchor="w")
            tk.Label(tt, text=Path(path).name, bg=p2["bg"], fg=p2["faint"],
                     font=ui_font(p2, 9)).pack(anchor="w")

            ams = get_ams_colors(state["data"])
            en_map = {int(m.get("channel", -1)): bool(m.get("enabled", True))
                      for m in state["data"].get("materials", [])}
            keys = sorted(ams.keys())
            if keys:
                c3, box3 = ui_card(wrap, p2, title="已写入的通道颜色")
                c3.pack(fill=tk.X, pady=(ui_px(p2, 14), 0))
                SW, SG = ui_px(p2, 40), ui_px(p2, 6)
                cvv = tk.Canvas(box3, width=len(keys) * (SW + SG) - SG,
                                height=SW, bg=p2["panel"],
                                highlightthickness=0, bd=0)
                cvv.pack(anchor="w")
                for i, ch in enumerate(keys):
                    col = ams[ch]
                    en = en_map.get(ch, True)
                    x = i * (SW + SG)
                    hexs = rgb_to_hex(*col)
                    fill = hexs if en else ui_mix(hexs, p2["panel"], 0.80)
                    ui_round_rect(cvv, x, 0, x + SW, SW, ui_px(p2, 9), fill=fill,
                                  outline=(ui_shade(fill, 34 if p2["dark"] else -34)
                                           if en else p2["border"]), width=1)
                    cvv.create_text(x + SW / 2, SW / 2, text=str(ch),
                                    fill=(ui_contrast_fg(col) if en
                                          else p2["disabled_fg"]),
                                    font=ui_font(p2, 11, True))
                    if not en:
                        p2s = ui_px(p2, 7)
                        cvv.create_line(x + p2s, SW - p2s, x + SW - p2s, p2s,
                                        fill=ui_mix(fill, p2["text"], 0.46),
                                        width=ui_px(p2, 2), capstyle="round")
                n_en = sum(1 for k in keys if en_map.get(k, True))
                tk.Label(box3,
                         text="共 %d 个通道，%d 个启用（禁用的不会被自动匹配选中）"
                              % (len(keys), n_en),
                         bg=p2["panel"], fg=p2["muted"],
                         font=ui_font(p2, 9)).pack(anchor="w", pady=(9, 0))

            tk.Label(wrap, text="把这个文件和切片 G-code 放在同一目录，"
                                "后处理脚本会自动读它。",
                     bg=p2["bg"], fg=p2["muted"], font=ui_font(p2, 9),
                     wraplength=CANVAS_W, justify="left").pack(
                         anchor="w", pady=(ui_px(p2, 12), 0))

            bar = tk.Frame(wrap, bg=p2["bg"])
            bar.pack(fill=tk.X, pady=(ui_px(p2, 14), 0))
            ui_button(bar, p2, "知道了", dlg.destroy, kind="accent",
                      bold=True, pad=(20, 7)).pack(side=tk.RIGHT)

            dlg.update_idletasks()
            ui_center(dlg)
            try:
                dlg.attributes("-topmost", True)
            except Exception:
                pass
            dlg.grab_set()

        def _show_match_result(gcode_path):
            p2 = th["p"]
            ams_colors = get_ams_colors(state["data"])
            first_fil = bool(state["data"].get("first_filament", True))
            try:
                with open(gcode_path, "r", encoding="utf-8", errors="replace") as f:
                    content = f.read()
            except OSError as e:
                messagebox.showerror("读取失败", "无法读取：%s" % e, parent=root)
                return
            slice_colors = parse_slice_colors(content)
            _, log_lines, _s, detail = rewrite_gcode(
                content, ams_colors, slice_colors, first_fil)

            dlg = tk.Toplevel(root)
            dlg.title("匹配预览 - %s" % Path(gcode_path).name)
            dlg.configure(bg=p2["bg"])
            dlg.transient(root)
            wrap = tk.Frame(dlg, bg=p2["bg"])
            wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)

            tk.Label(wrap, text="切片槽 → 物理通道", bg=p2["bg"], fg=p2["text"],
                     font=ui_font(p2, 12, True)).pack(anchor="w")
            tk.Label(wrap, text="以下结果不会写进文件，只是预览",
                     bg=p2["bg"], fg=p2["faint"],
                     font=ui_font(p2, 9)).pack(anchor="w", pady=(2, 10))

            if detail:
                SW, SG = ui_px(p2, 46), ui_px(p2, 8)
                cols = min(len(detail), 8)
                rows = (len(detail) + cols - 1) // cols
                cvv = tk.Canvas(wrap, width=cols * (SW + SG) - SG,
                                height=rows * (SW + ui_px(p2, 20) + SG) - SG,
                                bg=p2["bg"], highlightthickness=0, bd=0)
                cvv.pack(anchor="w")
                st_col = {"ok": p2["accent"], "far": p2["danger"],
                          "none": p2["danger"], "no_slice": p2["muted"],
                          "no_cfg": p2["danger"]}
                for i, d in enumerate(detail):
                    rr, cc = divmod(i, cols)
                    x = rr * 0 + cc * (SW + SG)
                    y = rr * (SW + ui_px(p2, 20) + SG)
                    col = d["color"] or (128, 128, 128)
                    hexs = rgb_to_hex(*col)
                    ui_round_rect(cvv, x, y, x + SW, y + SW, ui_px(p2, 10), fill=hexs,
                                  outline=ui_shade(hexs, 34 if p2["dark"] else -34),
                                  width=1)
                    cvv.create_text(x + SW / 2, y + SW / 2,
                                    text=str(d["idx"] + 1),
                                    fill=ui_contrast_fg(col),
                                    font=ui_font(p2, 12, True))
                    cvv.create_text(x + SW / 2, y + SW + ui_px(p2, 9),
                                    text=("→ %d" % d["ch"]) if d["status"] == "ok"
                                         else "—",
                                    fill=st_col.get(d["status"], p2["muted"]),
                                    font=ui_font(p2, 9, True))

            scroll, txt = ui_scroll_text(wrap, p2, height=14)
            scroll.pack(fill=tk.BOTH, expand=True, pady=(ui_px(p2, 12), 0))
            txt.insert("1.0", "\n".join(log_lines))
            txt.config(state=tk.DISABLED)
            dlg.geometry("640x540")
            ui_center(dlg)
            dlg.grab_set()

        def _do_match_preview():
            path = filedialog.askopenfilename(
                parent=root, title="选择 G-code 文件",
                filetypes=[("G-code", "*.gcode"), ("TXT", "*.txt"),
                           ("所有文件", "*.*")])
            if path:
                _show_match_result(path)

        def _do_setup_help():
            text, _entry = _setup_text()
            out = None
            try:
                out = _setup_file_path()
                out.write_text(text + "\n", encoding="utf-8")
            except OSError:
                out = None
            if out:
                text += "\n\n（同样内容已存成文件：%s）" % out
            _show_text_window("ESP-AMS：后处理脚本该填什么", text, parent=root)

        def _do_confirm():
            state["data"]["channel_count"] = state["ch_count"]
            state["data"]["first_filament"] = bool(first_var.get())
            path = os.path.join(state["dir"], state["config_name"])
            if save_settings(state["data"], path):
                _mark_dirty(False, msg="已保存 %s" % state["config_name"])
                _show_export_dialog(path)
            else:
                messagebox.showerror("保存失败", "无法写入：%s" % path, parent=root)

        # ===== 底部操作条内容 =====
        first_btn = ui_button(foot, p, "", lambda: _toggle_first())

        def _sync_first():
            on = bool(first_var.get())
            if on:
                bg, fg = p["accent"], p["accent_fg"]
                hov, fgh = p["accent_hov"], p["accent_fg"]
            else:
                bg, fg = p["chip"], p["muted"]
                hov, fgh = p["chip_hov"], p["text"]
            first_btn.config(text="首次换料：开" if on else "首次换料：关",
                             bg=bg, fg=fg, activebackground=hov,
                             activeforeground=fgh)
            first_btn.bind("<Enter>", lambda e: first_btn.config(bg=hov, fg=fgh))
            first_btn.bind("<Leave>", lambda e: first_btn.config(bg=bg, fg=fg))

        def _toggle_first():
            first_var.set(not first_var.get())
            state["data"]["first_filament"] = bool(first_var.get())
            _sync_first()
            _mark_dirty(msg="首次换料已%s（还没保存）"
                            % ("开" if first_var.get() else "关"))

        first_btn.pack(side=tk.LEFT)
        _sync_first()

        ui_button(foot, p, "匹配预览", _do_match_preview,
                  kind="ghost").pack(side=tk.LEFT, padx=(ui_px(p, 9), 0))
        ui_button(foot, p, "切片器该填什么", _do_setup_help,
                  kind="ghost").pack(side=tk.LEFT, padx=(ui_px(p, 4), 0))
        ui_button(foot, p, "保存配置", _do_confirm, kind="accent",
                  bold=True, pad=(22, 7)).pack(side=tk.RIGHT)

        # ===== 首次绘制 =====
        _refresh_config_names()
        _draw_colors()

    # ---------------- 启动时自动找一份已有配置 ----------------
    if getattr(sys, "frozen", False):
        base_dir = Path(sys.executable).parent
    else:
        base_dir = Path(__file__).parent

    auto = None
    for cand in (base_dir / SETTINGS_FILENAME,                    # EXE/脚本同目录
                 base_dir / "ams_profiles" / SETTINGS_FILENAME,   # 本工具自己的配置目录
                 Path(state["dir"]) / SETTINGS_FILENAME):         # ~/ESP_AMS
        if cand.is_file():
            auto = cand
            break
    if auto is None:
        ext = find_settings_extended()                            # 系统 Bambu 配置目录
        if ext:
            auto = Path(ext)

    if auto is not None:
        loaded = load_settings(str(auto))
        if loaded:
            state["data"] = loaded
            state["ch_count"] = int(loaded.get("channel_count", 8))
            state["dir"] = str(auto.parent)
            state["config_name"] = auto.name

    build()

    root.deiconify()
    root.update_idletasks()
    ui_center(root)
    try:
        root.attributes("-topmost", True)
        root.after(260, lambda: root.attributes("-topmost", False))
    except Exception:
        pass
    root.mainloop()


# ============================== 命令行模式 ==============================

def _find_pythonw() -> Optional[str]:
    """找一个 pythonw.exe（GUI 子系统 → 起进程时**不会创建控制台窗口**）。

    这就是"切片后弹个黑窗一闪而过"的解法：把后处理命令从 .bat 换成
    `pythonw.exe esp_ams_postprocess.py`，或者用同样无窗口的 esp_ams_tool.exe。
    """
    import shutil

    cands: List[str] = []

    # ★ 必须自己扫 PATH 的**每一个**目录，不能只用 shutil.which()——
    #   它返回第一个命中项，而本机 PATH 里排第一的恰好是沙箱内的临时
    #   Python（.workbuddy\binaries\...）。那条路径对用户是"过几天就没了"，
    #   填进切片器后表现为"某天开始后处理从来不生效"。
    for d in os.environ.get("PATH", "").split(os.pathsep):
        d = d.strip().strip('"')
        if d:
            cands.append(os.path.join(d, "pythonw.exe"))
    la = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Python")
    if os.path.isdir(la):
        for sub in sorted(os.listdir(la)):
            cands.append(os.path.join(la, sub, "pythonw.exe"))
    cands.append(str(_app_base() / "pythonw.exe"))
    for exe in ("pythonw", "pythonw.exe"):
        p = shutil.which(exe)
        if p:
            cands.append(p)

    def rank(p: str) -> int:
        low = p.lower()
        if not os.path.isfile(p):
            return 99
        # 优先用户自己装的 Python；沙箱/商店别名这类"临时"解释器排后面
        if ".workbuddy" in low or "windowsapps" in low:
            return 20
        if os.sep + "programs" + os.sep + "python" + os.sep in low:
            return 0
        return 5

    cands.sort(key=rank)
    for p in cands:
        if os.path.isfile(p):
            return p
    return None


def rv_escape(cmd: str) -> str:
    """把**整条命令行**转成 Bambu Studio 配置里那种转义写法。

    正确形式就是 JSON 字符串（内部 `"` → `\\"`，反斜杠双写），例如：

        "\\"C:\\\\x\\\\pythonw.exe\\" \\"D:\\\\y\\\\pp.py\\""

    ★ 踩过（2026-09-24）：把整条命令当成**一个路径**去转义，结果内部的引号
      没被转义 → 切片器读到的是半条路径 + 半条参数。必须按"整条命令行"转。
      对照组：TOP AMS 在文件头留下的是 `; post_process = "\\"D:\\\\kx\\\\a1duose\\\\topasm.exe\\""`。
    """
    return json.dumps(cmd, ensure_ascii=False)


def _pick_entry() -> Tuple[str, str, str]:
    """挑出「后处理脚本」框里该填的那**一条**命令。

    ★ 选优顺序是有讲究的（2026-09-24 踩过）：
      1. **EXE**：单一路径、无参数、无需引号（只要路径没空格），
         和 TOP AMS 的 `"D:\\kx\\a1duose\\topasm.exe"` 完全同型。
         用户只要复制一行就行，最不容易填错。
      2. **pythonw.exe + 脚本路径**：两段、必须带引号，而且 `python` 不在
         PATH 上时切片器会直接报 "The configured post-processing script
         does not exist: python" —— 这正是现场那次报错。
      3. **.bat**：会闪黑窗，但输出落盘，排错方便。
    返回 (填框用的那一行, 说明, 备选行列表)
    """
    base = _app_base()
    exe = base / "esp_ams_tool.exe"
    script = os.path.abspath(__file__)
    pyw = _find_pythonw()

    if exe.is_file():
        return str(exe), "不需要 Python、不闪窗、单一路径无参数", []
    if pyw:
        return ('"%s" "%s"' % (pyw, script),
                "用系统 Python 的 pythonw.exe（无控制台窗口）", [])
    # 兜底
    bat = base / "esp_ams_postprocess.bat"
    return ('"%s"' % bat, "找不到 Python/EXE，退回 .bat（会闪一下黑窗）", [])


def _setup_text() -> Tuple[str, str]:
    """算出「该往切片器里填什么」，返回 (整段说明, 该填的那一行)。

    ★★ 这一版是**照现场报错重写的**（2026-09-24）：
       用户把上一版的说明文字连同 `<路径>` 占位符一起粘进了框里、还带上
       了 `setup` 子命令，切片器直接提示
       "The configured post-processing script does not exist: python"。
       教训：给用户的**必须是能整行复制的一条命令**，说明文字要放到它下面，
       并且要显式写清"不要写 python / 不要写 setup / 不要带尖括号"。
    """
    entry, why, _ = _pick_entry()
    pyw = _find_pythonw()
    script = os.path.abspath(__file__)
    base = _app_base()
    exe = base / "esp_ams_tool.exe"
    cfg = find_settings_extended()
    mode_now = POPUP_DEFAULT
    if cfg:
        d = load_settings(cfg)
        if d:
            v = str(d.get(SETTINGS_KEY_POPUP, POPUP_DEFAULT)).lower()
            mode_now = v if v in (POPUP_AUTO, POPUP_ALWAYS, POPUP_NEVER) \
                else POPUP_DEFAULT
    mode_cn = {POPUP_ALWAYS: "每次都弹（默认）", POPUP_AUTO: "只在需要时弹",
               POPUP_NEVER: "从不弹"}.get(mode_now, mode_now)

    L = []
    L.append("=" * 68)
    L.append("★ 就填这一行 ★")
    L.append("  位置：Bambu Studio → 偏好设置 → 其他 → 「后处理脚本」")
    L.append("")
    L.append("  " + entry)
    L.append("")
    L.append("  （%s）" % why)
    L.append("=" * 68)
    L.append("")
    L.append("★ 那个框里【只能有这一行】。下面这些是现场踩过的坑，逐条对照：")
    L.append("   ✗ 不要写 `python` 三个字母开头 —— 切片器不认 PATH，"
             "会报")
    L.append("     \"The configured post-processing script does not exist: python\"")
    L.append("   ✗ 不要写 `setup` / `--dry-run` 之类的参数（那是给我自己看的）")
    L.append("   ✗ 不要保留说明书里的 `<路径>` 尖括号")
    L.append("   ✗ 不要粘两行（框里放两条命令 = 第二条被当成第一条的参数）")
    if " " in entry.strip('"') and not entry.startswith('"'):
        L.append("   ⚠ 这条路径里有空格，整条要用英文双引号包起来")
    L.append("")
    L.append("-" * 68)
    L.append("自检（现在是这台机器上的真实情况）")
    L.append("  配置文件    ：%s" % (cfg or "**没有** —— 先双击 esp_ams_tool 配一次通道颜色"))
    L.append("  esp_ams_tool.exe：%s" % (exe if exe.is_file() else "**没有**"))
    L.append("  pythonw.exe ：%s" % (pyw or "**没找到**"))
    L.append("  弹窗策略    ：%s（改法见下）" % mode_cn)
    L.append("")
    L.append("-" * 68)
    L.append("备选（一般用第一条就够了）")
    if pyw:
        L.append('  pythonw 版 ："%s" "%s"' % (pyw, script))
    L.append('  bat 版      ："%s"（会闪黑窗，输出落盘好排错）'
             % (base / "esp_ams_postprocess.bat"))
    L.append('  vbs 版      ："wscript.exe" "%s"（不闪窗）'
             % (base / "esp_ams_hidden.vbs"))
    L.append("")
    L.append("要粘到 .3mf / 打印机配置里的**转义**写法（整体加 \\\"，反斜杠双写）：")
    L.append("  " + rv_escape(entry))
    L.append("")
    L.append("-" * 68)
    L.append("弹窗：默认「每次切片都弹」一个映射确认窗（和 TOP AMS 一样），")
    L.append("      在窗口里选一次就会记住。想直接改，编辑上面那个配置文件里的")
    L.append('      %s 键："%s" 每次弹 / "%s" 只在需要时弹 / "%s" 从不弹。'
             % (SETTINGS_KEY_POPUP, POPUP_ALWAYS, POPUP_AUTO, POPUP_NEVER))
    L.append("      也可以临时用参数：--popup always|auto|never 或 --no-popup。")
    L.append("")
    L.append("填完**必须重新切片** —— 已切好的 .gcode 里存的是旧通道号。")
    return "\n".join(L), entry


def _setup_file_path() -> Path:
    return _app_base() / "后处理脚本该填什么.txt"


def _show_text_window(title: str, text: str, parent=None) -> None:
    """把一个长文本框弹出来给人看（带"复制全部"按钮）。

    ★ 为什么需要：`esp_ams_tool.exe` 是 PyInstaller `--windowed` 打的包，
      从切片器/资源管理器启动时 **没有控制台**，`sys.stdout is None`，
      于是 `setup` 打印的东西全被吞掉 —— 用户看到的就是"跑了，什么都没出来"。
      所以没有控制台时必须改用窗口呈现。

    `parent` 给了就开 Toplevel（配置 GUI 里调用），没给就自己开 Tk + mainloop。
    """
    try:
        import tkinter as tk
    except Exception:
        return
    try:
        win = tk.Toplevel(parent) if parent is not None else tk.Tk()
    except Exception:
        return

    p = ui_build_theme(win, ui_resolve_dark(ui_load_pref()))
    ui_apply(win, p)
    win.title(title)
    win.configure(bg=p["bg"])
    win.resizable(False, False)

    wrap = tk.Frame(win, bg=p["bg"])
    wrap.pack(fill=tk.BOTH, expand=True, padx=16, pady=16)

    tk.Label(wrap, text="把「就填这一行」那一段整行复制到："
                        "Bambu Studio → 偏好设置 → 其他 → 后处理脚本",
             bg=p["bg"], fg=p["text"], font=ui_font(p, 10, True),
             justify="left").pack(anchor="w")
    tk.Label(wrap, text="框里只能有这一行；不要带 <尖括号>、不要写 setup、不要粘说明文字。",
             bg=p["bg"], fg=p["muted"], font=ui_font(p, 9),
             justify="left").pack(anchor="w", pady=(4, 10))

    scroll, txt = ui_scroll_text(wrap, p, height=20)
    scroll.pack(fill=tk.BOTH, expand=True)
    txt.insert("1.0", text)
    txt.config(state=tk.DISABLED)

    bar = tk.Frame(wrap, bg=p["bg"])
    bar.pack(fill=tk.X, pady=(ui_px(p, 12), 0))
    tip = tk.Label(bar, text="", bg=p["bg"], fg=p["accent"],
                   font=ui_font(p, 9))
    tip.pack(side=tk.LEFT)

    def _copy():
        try:
            win.clipboard_clear()
            win.clipboard_append(text)
            tip.config(text="已复制到剪贴板 ✓")
        except Exception:
            tip.config(text="复制失败，请手动选中复制", fg=p["danger"])

    ui_button(bar, p, "复制全部", _copy, kind="accent", bold=True,
              pad=(18, 7)).pack(side=tk.RIGHT)
    ui_button(bar, p, "关闭", win.destroy,
              kind="normal").pack(side=tk.RIGHT, padx=(0, 8))

    win.update_idletasks()
    ui_refresh_scale(win, p)
    TW, TH = ui_px(p, 780), ui_px(p, 620)
    win.minsize(TW, TH)          # 同 show_map_window：不钉住 min/max，
    win.maxsize(TW, TH)          # 窗口宽度会被内容顶开
    win.geometry("%dx%d" % (TW, TH))
    win.update_idletasks()
    ui_center(win, TW, TH)
    try:
        win.attributes("-topmost", True)
    except Exception:
        pass
    if parent is None:
        win.mainloop()


def _cmd_setup():
    """命令行 `setup`：打印 + 落盘 + （没有控制台时）弹窗。"""
    text, _entry = _setup_text()
    out = None
    try:
        out = _setup_file_path()
        out.write_text(text + "\n", encoding="utf-8")
    except OSError:
        out = None
    safe_print(text)
    if out:
        safe_print("")
        safe_print("（同样内容已存成文件：%s）" % out)
    if not STDOUT_OK:
        # windowed EXE 没有控制台 → 必须用窗口，否则用户什么都看不到
        extra = ("\n\n（同样内容已存成文件：%s）" % out) if out else ""
        _show_text_window("ESP-AMS：后处理脚本该填什么", text + extra)


def _cmd_export_config():
    """命令行导出配置 / 匹配"""
    import argparse
    parser = argparse.ArgumentParser(description="ESP-AMS 配置导出工具")
    sub = parser.add_subparsers(dest="cmd")

    exp = sub.add_parser("export", help="导出默认配置到指定路径")
    exp.add_argument("--ch", type=int, default=8, choices=[4, 8, 16], help="通道数")
    exp.add_argument("-o", "--output", required=True, help="输出文件路径")

    match = sub.add_parser("match", help="对 G-code 执行匹配预览（**不写文件**）")
    match.add_argument("gcode", help="G-code 文件路径")
    match.add_argument("--config", default=None, help="配置文件路径")
    match.add_argument("-l", "--log", action="store_true", help="打印匹配日志")

    sub.add_parser("setup", help="打印该往切片器「后处理脚本」里填的那一行")

    args = parser.parse_args()

    if args.cmd == "setup":
        _cmd_setup()
        return
    if args.cmd == "export":
        data = default_settings(args.ch)
        save_settings(data, args.output)
        safe_print("配置已导出：%s" % args.output)
    elif args.cmd == "match":
        # 预览模式：dry_run，只把映射表打出来，一个字节都不改
        res = process_file(args.gcode, explicit_config=args.config,
                           write_back=False, popup_mode=POPUP_NEVER,
                           do_popup=False, verbose=False)
        if not res.get("ok"):
            return
        out = []
        out.append("配置文件：%s" % (res["settings_path"] or "**没找到**"))
        out.append("切片颜色：%s"
                   % (", ".join("%d=%s" % (i + 1, rgb_to_hex(*c))
                                for i, c in sorted(res["slice_colors"].items()))
                      or "（切片里没有）"))
        out.append("项目通道：%s"
                   % (", ".join("通道%d=%s" % (ch, rgb_to_hex(*c))
                                for ch, c in sorted(res["ams_colors"].items()))
                      or "（没配）"))
        out.append("-" * 48)
        for line in res["log_lines"]:
            out.append("  " + line)
        out.append("-" * 48)
        s = res["summary"]
        out.append("M140 信令：改写了 %d 处，保持原样 %d 处"
                   % (s.get("m140_ok", 0), s.get("m140_miss", 0)))
        safe_print("\n".join(out))
    else:
        parser.print_help()


def main():
    """
    入口函数：
      - 在 Bambu Studio 中导入时：onBeforeWriteGCode 由框架调用
      - 独立运行时（无参数）：启动 GUI
      - 带参数运行时：执行命令行子命令
    """
    # ★★ 用 pythonw.exe / PyInstaller --windowed 运行时，sys.stdout 和
    #    sys.stderr 都是 **None**。这时候连 argparse 自己报个错都会
    #    AttributeError（它内部直接 file.write），于是现象就是"窗口一闪、
    #    什么都没发生"。这里先把它们换成黑洞，保证任何路径都不会因此崩。
    global STDOUT_OK
    STDOUT_OK = sys.stdout is not None
    if sys.stdout is None:
        sys.stdout = open(os.devnull, "w", encoding="utf-8")
    if sys.stderr is None:
        sys.stderr = open(os.devnull, "w", encoding="utf-8")

    # 检测是否在 Bambu Studio 环境中（环境变量或模块被 import 时）
    in_bambu = (
        "BAMBU_STUDIO" in os.environ
        or "BAMBU" in os.environ
        or "sk1app" in sys.modules          # Bambu Studio 内置 Python 环境
        or any(k.startswith("sk") for k in sys.modules)  # sk1app/sklabs 模块
    )
    if in_bambu:
        return

    if len(sys.argv) > 1:
        if sys.argv[1] in ("export", "match", "setup"):
            _cmd_export_config()
            return
        elif sys.argv[1] == "gui":
            _run_gui()
            return
        elif sys.argv[1] == "--no-gui":
            pass
        else:
            # 第一个参数是文件路径 → Bambu Studio「后处理脚本」走的就是这条
            import argparse
            parser = argparse.ArgumentParser(description="ESP-AMS G-code 后处理")
            parser.add_argument('input', help='输入 G-code 文件')
            parser.add_argument('output', nargs='?',
                                help='输出文件（默认就地覆盖输入）')
            parser.add_argument('--config', help='手动指定配置文件路径')
            parser.add_argument('--log', action='store_true', help='输出匹配日志')
            parser.add_argument('--dry-run', action='store_true',
                                help='只算映射，不写文件（用来先看一眼对不对）')
            parser.add_argument('--popup', default=None,
                                choices=[POPUP_AUTO, POPUP_ALWAYS, POPUP_NEVER],
                                help='弹窗策略：auto（默认，只在需要拍板时弹）/ '
                                     'always / never')
            parser.add_argument('--no-popup', action='store_true',
                                help='等价于 --popup never')
            args = parser.parse_args()

            res = process_file(
                args.input, args.output,
                explicit_config=args.config,
                write_back=not args.dry_run,
                popup_mode=POPUP_NEVER if args.no_popup else args.popup,
                do_popup=not args.dry_run,
                verbose=not args.dry_run,
            )
            if args.dry_run:
                safe_print("（--dry-run：一个字节都没改）")
                for line in res.get("log_lines", []):
                    safe_print("  " + line)
                s = res.get("summary", {})
                safe_print("  M140 信令改写了 %d 处（保持原样 %d 处）"
                           % (s.get("m140_ok", 0), s.get("m140_miss", 0)))
            elif res.get("ok"):
                s = res["summary"]
                safe_print("完成：M140 改写了 %d 处（保持原样 %d 处）"
                           % (s.get("m140_ok", 0), s.get("m140_miss", 0)))
            sys.exit(0 if res.get("ok") else 2)

    # 无参数 → 启动 GUI
    _run_gui()


if __name__ == '__main__':
    main()
