@echo off
rem ============================================================================
rem  ESP-AMS 后处理启动器（Bambu Studio 用）—— **看得见输出** 的那一版
rem ============================================================================
rem
rem  在切片器里填：
rem      "C:\...\esp_ams_postprocess.bat"
rem  Bambu Studio 会把导出的 .gcode 路径追加在后面。
rem
rem  ⚠️ 这个 .bat 会**闪一个控制台窗口**（cmd.exe 是控制台程序，无法避免）。
rem     不想要闪窗就用同目录的 `esp_ams_hidden.vbs`（见文件末尾说明），
rem     或者直接用打包好的 esp_ams_tool.exe。
rem
rem  历史教训（2026-09-24）：原来的版本用的是 `pause` 收尾 + 输出全丢在控制台，
rem  结果是"窗口一闪，什么都没看到"。现在改成：
rem    ① 输出落盘到 <gcode>.ams.log.console，随时能翻；
rem    ② 失败时**必须留痕**：写 .err.log + 弹一个消息框（G-code 里的通道号错了
rem       必须让人知道，静默打出去就是印错颜色）。
rem ============================================================================

setlocal
set "SCRIPT_DIR=%~dp0"
set "PY_SCRIPT=%SCRIPT_DIR%esp_ams_postprocess.py"

if not exist "%PY_SCRIPT%" (
    call :fail "找不到 esp_ams_postprocess.py（应当在 %SCRIPT_DIR%）"
    exit /b 1
)

rem ---- 找一个 pythonw（无控制台）优先，退回 python ----
set "PYW="
where pythonw >nul 2>nul && set "PYW=pythonw"
if not defined PYW if exist "%SCRIPT_DIR%pythonw.exe" set "PYW=%SCRIPT_DIR%pythonw.exe"

set "PY="
where python >nul 2>nul && set "PY=python"
if not defined PY where py >nul 2>nul && set "PY=py -3"
if not defined PY if exist "%SCRIPT_DIR%python.exe" set "PY=%SCRIPT_DIR%python.exe"

if not defined PYW if not defined PY (
    call :fail "这台机器上没有找到 Python^（需要 Python 3.8+^）。可以用打包好的 esp_ams_tool.exe 代替这个 .bat。"
    exit /b 1
)

rem ---- 输出重定向到一个日志，免得"一闪而过什么都没留下" ----
rem  用第一个参数（Bambu Studio 传来的 gcode 路径）派生日志名。
set "GCODE=%~1"
set "OUTLOG=%SCRIPT_DIR%esp_ams_last_run.log"
if not "%GCODE%"=="" set "OUTLOG=%GCODE%.ams.log.console"

if defined PYW (
    "%PYW%" "%PY_SCRIPT%" %* > "%OUTLOG%" 2>&1
) else (
    %PY% "%PY_SCRIPT%" %* > "%OUTLOG%" 2>&1
)
set "RC=%ERRORLEVEL%"

if not "%RC%"=="0" (
    call :fail "后处理失败（返回码 %RC%）。详情见：%OUTLOG%"
    exit /b %RC%
)

exit /b 0

:fail
echo [ESP-AMS] %~1
echo [ESP-AMS] %~1 >> "%OUTLOG%"
rem 弹窗告知（G-code 通道号没改对就发出去 = 印错颜色，必须让人知道）
mshta "vbscript:Execute(""msgbox """"ESP-AMS 后处理失败："""" & """"%~1"""" ,16,""""ESP-AMS"""":close"")" >nul 2>nul
exit /b 1

rem ============================================================================
rem  不想看到闪窗？把切片器里的后处理命令换成：
rem
rem      "wscript.exe" "C:\...\esp_ams_hidden.vbs"
rem
rem  同目录的 esp_ams_hidden.vbs 会以隐藏窗口启动同一个脚本，
rem  结果完全一样（映射写进 G-code + 写 .ams.log），只是一个窗口都不闪。
rem ============================================================================
