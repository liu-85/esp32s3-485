# -*- coding: utf-8 -*-
"""
PyInstaller 编译脚本
将 esp_ams_postprocess.py 打包为单文件 Windows EXE

用法（在本目录运行）：
  python build_exe.py

产物：
  dist/esp_ams_tool/esp_ams_tool.exe

GitHub Actions 中也会调用此脚本。
"""

import os
import subprocess
import sys
from pathlib import Path

# 强制 stdout 使用 UTF-8（GitHub Actions Windows runner 默认 cp1252 无法输出中文）
if sys.platform == "win32":
    import io
    sys.stdout = io.TextIOWrapper(sys.stdout.buffer, encoding="utf-8", errors="replace")
    sys.stderr = io.TextIOWrapper(sys.stderr.buffer, encoding="utf-8", errors="replace")

HERE = Path(__file__).parent
SCRIPT = HERE / "esp_ams_postprocess.py"
DIST_DIR = HERE / "dist"
WORK_DIR = HERE / "build"


def main():
    # 确保 PyInstaller 已安装
    try:
        import PyInstaller  # noqa: F401
    except ImportError:
        print("正在安装 PyInstaller...")
        subprocess.check_call([
            sys.executable, "-m", "pip",
            "install", "pyinstaller",
        ])

    # 清理旧的构建目录
    for d in [DIST_DIR, WORK_DIR, HERE / "esp_ams_tool.spec"]:
        if d.is_dir():
            import shutil
            shutil.rmtree(d)
        elif d.is_file():
            d.unlink()

    print(f"编译目标：{SCRIPT}")
    print("输出目录：dist/esp_ams_tool/")

    cmd = [
        sys.executable, "-m", "PyInstaller",
        "--onedir",          # 多文件目录模式（启动更快，EXE 体积小）
        "--windowed",        # 不弹出控制台窗口
        "--name", "esp_ams_tool",
        "--icon", "NONE",    # 无图标（可替换为 .ico）
        "--add-data", f"esp_ams_postprocess.py;.",
    ]

    # ★ DPI manifest：PyInstaller 默认**不带 dpiAware**，进程是 DPI-unaware 的，
    #   在 125% / 150% 缩放的屏幕上整窗被 Windows 当位图拉伸（界面发虚，
    #   且 GetDpiForWindow 只会回 96）。声明 PerMonitorV2 之后 Tk 才知道真实
    #   DPI，字体按真实 DPI 渲染，代码里的 ui_px() 缩放也才对得上。
    manifest = HERE / "esp_ams_tool.manifest"
    if manifest.is_file():
        cmd += ["--manifest", str(manifest)]
    else:
        print("警告：找不到 esp_ams_tool.manifest，"
              "高 DPI 屏上界面会被拉伸得发虚")

    cmd.append(SCRIPT)

    # 把当前目录作为工作目录，让 PyInstaller 找到脚本
    result = subprocess.run(cmd, cwd=str(HERE))
    if result.returncode != 0:
        print("编译失败！")
        sys.exit(1)

    exe = DIST_DIR / "esp_ams_tool" / "esp_ams_tool.exe"
    if not exe.is_file():
        # 尝试查找实际输出位置
        candidates = list((DIST_DIR / "esp_ams_tool").rglob("*.exe")) if (DIST_DIR / "esp_ams_tool").is_dir() else []
        if candidates:
            exe = candidates[0]
        else:
            print(f"找不到 EXE，dist/ 目录内容：{list(DIST_DIR.glob('*'))}")
            sys.exit(1)

    size_mb = exe.stat().st_size / (1024 * 1024)
    print(f"\n编译成功：{exe}  ({size_mb:.1f} MB)")
    print("运行方式：双击 esp_ams_tool.exe 或命令行 esp_ams_tool.exe")
    print("Bambu Studio 后处理脚本：填这个 EXE 的绝对路径（一行、无参数）。")
    print("  注意：框里不要写 python / 不要写 setup / 不要带尖括号。")
    print("  跑一次 esp_ams_tool.exe setup 会给出「★ 就填这一行 ★」并落盘 txt。")


if __name__ == "__main__":
    main()
