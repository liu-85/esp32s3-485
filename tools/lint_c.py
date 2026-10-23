#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
C 源码体检 —— 在没有交叉编译器的情况下，把"一眼看不出来"的低级错误先挑出来。

主要抓五类：

  0. ★ 文件带 UTF-8 BOM（开头的 EF BB BF）
     这条是**真实踩过的坑**，而且后果比想象中重：ESP-IDF 生成分区表用的
     gen_esp32part.py 以二进制读入后 `data.decode()`（默认 utf-8，**不是**
     utf-8-sig），再用 `line.strip().startswith('#')` 跳注释。带 BOM 时首字符
     是 U+FEFF 而不是 '#'，跳过逻辑失效 —— 首行注释被当成一条分区定义解析，
     报 "Field 'type' can't be left empty."，构建直接失败。
     报错信息完全指不到"BOM"上，只能靠这条判据提前拦住。
     编辑器（尤其 Windows 上的记事本 / 某些 VS Code 配置）会偷偷加 BOM，
     所以这不是一次性问题，必须常驻检查。

  1. ★ 代码位置出现中文字符（最可靠的一条，优先看它）
     典型来源：中文字符串里手写了英文双引号，例如
         desc = "可能检测到"意面"缺陷（打印乱丝）。";
     编译器只会报"expected ';'"，真正的原因藏在中文里，极难定位。

     ★ 为什么不用"引号数量为奇数"来判：上面这行里的引号是**成对**的
       （"…" 意面 "…" 一共 6 个，是偶数），奇偶判据完全看不出来。
       而 C 语言里标识符只能是 ASCII，**中文不可能合法地出现在字符串和注释
       之外** —— 所以只要代码位置扫到中文，就一定是某个字符串被提前截断了。
       这条判据零误报，比数引号可靠得多。

  2. 字符串引号数量为奇数（引号没配上的另一种形态，作为补充）

  3. 括号/花括号/方括号不配对（同样先在剔掉字符串和注释之后数）

  4. 中文全角标点混进代码区（引号、逗号、分号、括号）
     这些在字符串里是正常的，但在代码位置出现就是错误。

用法：
    python tools/lint_c.py [目录]
    python tools/lint_c.py --selftest      # 先验证判据本身没坏

默认扫 esp-ams-s3/ 下的 .c / .h（跑全部判据）和 .csv（只查 BOM ——
分区表那种文件跑括号/引号判据没有意义，但 BOM 判据恰恰最需要）。
"""

import os
import re
import sys

# 全角标点 → 半角，用来提示"这个可能应该是代码里的符号"
FULLWIDTH = {
    '“': '"', '”': '"', '‘': "'", '’': "'",
    '，': ',', '；': ';', '：': ':', '（': '(', '）': ')',
    '【': '[', '】': ']', '＋': '+', '－': '-', '＝': '=',
    '＜': '<', '＞': '>', '／': '/', '＼': '\\',
}
# 允许出现在注释/字符串里的全角标点（中文说明里这些太常见，不算问题）
CODE_ALLOWED_FW = set('（）')

CJK = re.compile(r'[\u4e00-\u9fff]')


def strip_code(line):
    """把一行里的字符串内容和注释去掉，只留下"结构字符"，用于配对检查。

    返回 (结构串, 字符串内是否出现过 CJK, 是否有未闭合的字符串)
    这一步是刻意写得保守的：C 的字符串拼接、行连续符都会让精确解析变复杂，
    而我们只需要一个"够用就好"的判据，宁可漏报也不要误报。
    """
    out = []
    i = 0
    n = len(line)
    in_str = False
    in_char = False
    str_has_cjk = False
    while i < n:
        c = line[i]
        if in_str:
            if c == '\\':
                i += 2
                continue
            if c == '"':
                in_str = False
                out.append('"')
            else:
                if CJK.match(c):
                    str_has_cjk = True
            i += 1
            continue
        if in_char:
            if c == '\\':
                i += 2
                continue
            if c == "'":
                in_char = False
            i += 1
            continue
        # 不在字符串里
        if c == '/' and i + 1 < n and line[i + 1] == '/':
            break                     # 行注释，后面都不要了
        if c == '/' and i + 1 < n and line[i + 1] == '*':
            # 块注释：只处理同一行内闭合的情况，跨行的交给下面单独判断
            end = line.find('*/', i + 2)
            if end < 0:
                break
            i = end + 2
            continue
        if c == '"':
            in_str = True
            out.append('"')
            i += 1
            continue
        if c == "'":
            in_char = True
            i += 1
            continue
        out.append(c)
        i += 1

    return ''.join(out), str_has_cjk, in_str


BOM_BYTES = b'\xef\xbb\xbf'

# 会跑"代码判据"（括号/引号/中文越界）的扩展名。
#   其它扩展名（如 .csv）只跑 BOM 判据 —— 对分区表跑括号配对没有意义，
#   但它恰恰是最怕 BOM 的文件。
CODE_EXTS = ('.c', '.h')
SCAN_EXTS = CODE_EXTS + ('.csv',)


def check_bytes(data, is_code=True):
    """对一份"文件原始字节"做全部检查，返回 [(行号, 说明, 原文), ...]。

    以字节为入口（而不是直接读文本）是为了能看见 BOM —— 一旦用
    `open(..., encoding='utf-8')` 读进来，BOM 会被 Python 当成普通字符塞在
    第一个字符里，很难意识到它是"文件头签名"而不是内容。
    """
    problems = []

    has_bom = data.startswith(BOM_BYTES)
    if has_bom:
        problems.append(
            (1,
             '★ 文件带 UTF-8 BOM（开头的 EF BB BF）—— 必须去掉。'
             'ESP-IDF 的 gen_esp32part.py 不做 BOM 剥离，'
             '会导致分区表首行注释被当成分区定义、构建失败',
             ''))

    # 有 BOM 就用 utf-8-sig 解码，把签名吃掉，免得影响后面的行内容判据
    text = data.decode('utf-8-sig' if has_bom else 'utf-8', errors='replace')

    if is_code:
        problems.extend(check_lines(text.splitlines(True)))

    return problems


def check_file(path):
    with open(path, 'rb') as fh:
        data = fh.read()
    is_code = path.lower().endswith(CODE_EXTS)
    return check_bytes(data, is_code=is_code)


def check_lines(lines):
    """对一份"行列表"做全部检查，返回 [(行号, 说明, 原文), ...]。

    单独抽出来是为了让 --selftest 能直接喂内存里的样例，而不用落地成临时文件。
    """
    problems = []

    # 括号必须按**整个文件累计**配平，不能按行判 —— `}` 本来就经常单独占一行。
    depth = {'(': 0, '[': 0, '{': 0}
    pair_of = {')': '(', ']': '[', '}': '{'}
    depth_at_eof = None

    in_block_comment = False
    for idx, raw in enumerate(lines, 1):
        line = raw.rstrip('\n')

        # 处理跨行块注释
        if in_block_comment:
            end = line.find('*/')
            if end < 0:
                continue
            line = line[end + 2:]
            in_block_comment = False
        # 简单处理：一行里出现 /* 且没闭合 → 后面整段忽略
        while True:
            s = line.find('/*')
            if s < 0:
                break
            e = line.find('*/', s + 2)
            if e < 0:
                line = line[:s]
                in_block_comment = True
                break
            line = line[:s] + line[e + 2:]

        struct, str_has_cjk, unclosed = strip_code(line)

        # ---- 检查 1：代码位置出现中文（★ 最可靠的一条） ----
        # C 的标识符只能是 ASCII，中文合法出现的位置只有字符串和注释。
        # 所以 `struct`（已经剔掉字符串内容和注释）里还能扫到中文，就说明
        # 有个字符串被裸引号提前截断了，后面的中文掉出来变成了"代码"。
        #
        # 为什么这条能补上"引号奇偶"的漏：
        #     "可能检测到"意面"缺陷（打印乱丝）。"
        # 这行引号是成对的（6 个，偶数），奇偶判据看不见；
        # 但掉到字符串外面的 `意面` 一定会暴露在 struct 里。
        m = CJK.search(struct)
        if m:
            problems.append(
                (idx,
                 '★ 代码位置出现中文「%s」—— 字符串里混进了英文双引号，'
                 '把后面的中文挤出了字符串（编译器会报 expected 分号/右括号）'
                 % m.group(0),
                 line.strip()))

        # ---- 检查 2：字符串引号数量为奇数（补充判据） ----
        if struct.count('"') % 2 == 1:
            problems.append(
                (idx, '字符串引号数量为奇数（很可能字符串里混进了英文双引号）',
                 line.strip()))

        # ---- 检查 3：括号按文件累计配平 ----
        for i, c in enumerate(struct):
            if c in depth:
                depth[c] += 1
            elif c in pair_of:
                depth[pair_of[c]] -= 1
                if depth[pair_of[c]] < 0:
                    problems.append(
                        (idx, '多余的「%s」—— 到这里就配不上了' % c,
                         line.strip()))
                    depth[pair_of[c]] = 0     # 复位，避免后面刷屏

        # ---- 检查 4：代码位置出现全角标点 ----
        for fw, hw in FULLWIDTH.items():
            if fw in struct:
                problems.append(
                    (idx, '代码位置出现全角字符「%s」，应为「%s」' % (fw, hw),
                     line.strip()))

    depth_at_eof = dict(depth)
    leftover = {k: v for k, v in depth_at_eof.items() if v != 0}
    if leftover:
        problems.append(
            (len(lines),
             '★ 文件结束时括号仍未配平（剩余 %s）—— 多半是漏了一个 ）或 }'
             % ', '.join('%s:%+d' % (k, v) for k, v in sorted(leftover.items())),
             ''))

    return problems


# ============================================================================
# 自测 —— 保证"该抓的能抓到、不该报的不报"
# ============================================================================
# 每条是 (说明, 源码, 期望发现的问题条数)。
#
# ★ 第 1 条就是这次真实踩到的坑：中文字符串里嵌了英文双引号，而这一行里的
#   引号总数是**偶数**，旧版"数引号奇偶"的判据完全看不见它 —— 正是靠这条
#   用例把判据换成"代码位置出现中文"的。改判据之前先跑一遍 --selftest。
SELFTEST_CASES = [
    (
        '中文字符串里嵌了裸双引号（引号成对，奇偶判据抓不到）——必须抓到',
        'static const char *s = "可能检测到"意面"缺陷（打印乱丝）。";\n',
        1,
    ),
    (
        '字符串引号没闭合（引号落单）——必须抓到',
        'static const char *s = "没有闭合;\n',
        1,
    ),
    (
        '正常写法：中文在字符串内、嵌套用全角引号——不该报',
        'static const char *s = "可能检测到「意面」缺陷（打印乱丝）。";\n',
        0,
    ),
    (
        '正常写法：中文只在行注释里，注释里还有引号——不该报',
        '// 这行是注释，随便写中文"引号"也不该被报\nint a = 1;\n',
        0,
    ),
    (
        '正常写法：JSON 转义引号 \\" ——不该报',
        'cJSON_AddStringToObject(o, "k", "值\\"带引号\\"");\n',
        0,
    ),
    (
        '多余一个右括号——必须抓到',
        'void f(void) {\n    if (a)) {\n    }\n}\n',
        1,
    ),
    (
        '花括号少一个（文件结束时没配平）——必须抓到',
        'void f(void) {\n    int a = 1;\n',
        1,
    ),
    (
        '正常写法：括号跨多行且全部配平——不该报',
        'void f(void)\n{\n    if (a) {\n        g();\n    }\n}\n',
        0,
    ),
]

# 字节级用例（用来验证 BOM 判据）。每条是 (说明, 原始字节, 期望发现的问题条数)。
BOM_SELFTEST_CASES = [
    (
        '带 UTF-8 BOM 的分区表（首行是 # 注释）——必须抓到',
        BOM_BYTES + b'# comment\nnvs, data, nvs, 0x9000, 0x6000,\n',
        1,
    ),
    (
        '带 UTF-8 BOM 的源文件——必须抓到',
        BOM_BYTES + b'int a = 1;\n',
        1,
    ),
    (
        '正常：同样内容但不带 BOM——不该报',
        b'# comment\nnvs, data, nvs, 0x9000, 0x6000,\n',
        0,
    ),
    (
        '正常：Linux 换行的源码、含中文注释——不该报',
        '#include <stdio.h>\n// 中文注释\nint a = 1;\n'.encode('utf-8'),
        0,
    ),
]


def run_selftest():
    bad = 0
    total = 0
    for title, src, want in SELFTEST_CASES:
        got = len(check_lines(src.splitlines(True)))
        ok = (got == want)
        total += 1
        if not ok:
            bad += 1
        print('  [%s] %s' % ('ok  ' if ok else 'FAIL', title))
        if not ok:
            print('        期望 %d 条，实际 %d 条' % (want, got))
    for title, data, want in BOM_SELFTEST_CASES:
        # 分区表那种扩展名只跑 BOM 判据，这里 is_code=False 就是模拟 csv
        got = len(check_bytes(data, is_code=False))
        ok = (got == want)
        total += 1
        if not ok:
            bad += 1
        print('  [%s] %s' % ('ok  ' if ok else 'FAIL', title))
        if not ok:
            print('        期望 %d 条，实际 %d 条' % (want, got))
    print('')
    print('自测 %d 条，%s'
          % (total,
             '全部通过' if bad == 0 else '有 %d 条不符合预期 ★ 判据坏了' % bad))
    return 1 if bad else 0


def main():
    argv = sys.argv[1:]

    if '--selftest' in argv:
        return run_selftest()

    root = argv[0] if argv else 'esp-ams-s3'
    targets = []
    if os.path.isfile(root):
        targets = [root]
    else:
        for dirpath, dirnames, filenames in os.walk(root):
            # 跳过构建产物，别去扫 build/ 里自动生成的东西
            dirnames[:] = [d for d in dirnames
                           if d not in ('build', 'managed_components', '.git')]
            for name in sorted(filenames):
                if name.lower().endswith(SCAN_EXTS):
                    targets.append(os.path.join(dirpath, name))

    total = 0
    for path in targets:
        probs = check_file(path)
        if probs:
            print('=' * 72)
            print(path)
            print('=' * 72)
            for line_no, msg, text in probs:
                print('  L%-5d %s' % (line_no, msg))
                print('         | %s' % text[:110])
            total += len(probs)

    print('')
    print('扫描 %d 个文件，发现 %d 处可疑点' % (len(targets), total))
    return 1 if total else 0


if __name__ == '__main__':
    sys.exit(main())
