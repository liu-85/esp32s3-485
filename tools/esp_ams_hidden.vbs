' ============================================================================
'  ESP-AMS 后处理启动器（**完全无窗口**版）
' ============================================================================
'
'  为什么需要它：Bambu Studio 的「后处理脚本」是用 CreateProcess 起一个子进程。
'  .bat 由 cmd.exe 执行，而 cmd.exe 是**控制台程序** → 必然闪一个黑窗，
'  脚本一跑完窗口就没了，现场看到的就是"弹出个窗口，什么都没看到就没有了"。
'  wscript.exe 是 **GUI 子系统**程序 → 不会创建控制台窗口。
'
'  用法：在切片器的「后处理脚本」里填——
'
'      "wscript.exe" "C:\...\esp_ams_hidden.vbs"
'
'  Bambu Studio 会把导出的 .gcode 路径追加在后面，本脚本取最后一个参数。
'
'  行为与跑 .py 完全一致：颜色映射写进 G-code + 生成 <gcode>.ams.log。
'  失败时会在同目录写 esp_ams_last_error.txt，并弹一个说明框 —— 绝不静默。
' ============================================================================
Option Explicit

Dim fso, sh, base, pyScript, gcode, cmd, rc, pythonExe, errFile, msg

Set fso = CreateObject("Scripting.FileSystemObject")
Set sh  = CreateObject("WScript.Shell")

base     = fso.GetParentFolderName(WScript.ScriptFullName)
pyScript = base & "\esp_ams_postprocess.py"
errFile  = base & "\esp_ams_last_error.txt"

If Not fso.FileExists(pyScript) Then
    Fail "找不到 " & pyScript & vbCrLf & _
         "请把 esp_ams_hidden.vbs 和 esp_ams_postprocess.py 放在同一个目录。"
End If

If WScript.Arguments.Count < 1 Then
    Fail "没有收到 G-code 路径 —— 这个脚本是给切片器当后处理用的，" & vbCrLf & _
         "不要在资源管理器里双击它。想手动测试请跑：" & vbCrLf & _
         "  python esp_ams_postprocess.py <你的.gcode>"
End If

' 取最后一个参数 = Bambu Studio 传来要处理的 .gcode
gcode = WScript.Arguments(WScript.Arguments.Count - 1)

pythonExe = FindPython()
If pythonExe = "" Then
    Fail "没有找到 Python（需要 Python 3.8+）。" & vbCrLf & vbCrLf & _
         "两个选择：" & vbCrLf & _
         "  1) 装一个 Python 并勾选 Add to PATH；" & vbCrLf & _
         "  2) 改用打包好的 esp_ams_tool.exe（不需要 Python）。"
End If

cmd = """" & pythonExe & """ """ & pyScript & """ """ & gcode & """"

' 0 = 隐藏窗口；True = 等它跑完（切片器紧接着就要读这个文件，必须等）
On Error Resume Next
rc = sh.Run(cmd, 0, True)
If Err.Number <> 0 Then
    Fail "启动失败：" & Err.Description & vbCrLf & vbCrLf & "命令：" & cmd
End If
On Error GoTo 0

If rc <> 0 Then
    ' 脚本自己也会写 .ams.log，这里再兜一层，顺便把退出码说清楚
    Fail "后处理返回码 " & rc & "（非 0）。" & vbCrLf & vbCrLf & _
         "常见原因：G-code 被别的程序占着、磁盘满、配置文件是坏的 JSON。" & vbCrLf & _
         "明细见 " & gcode & ".ams.log.console（如果有） 或 " & gcode & ".ams.log"
End If

WScript.Quit 0


' ---------------------------------------------------------------------------
' 找 pythonw.exe（GUI 子系统，没有控制台）；找不到就退回 python.exe
' ---------------------------------------------------------------------------
Function FindPython()
    Dim paths, i, p, cand, shell

    ' ① 同目录自带
    If fso.FileExists(base & "\pythonw.exe") Then
        FindPython = base & "\pythonw.exe" : Exit Function
    End If
    If fso.FileExists(base & "\python.exe") Then
        FindPython = base & "\python.exe" : Exit Function
    End If

    ' ② PATH 里逐个目录找 pythonw.exe → python.exe
    Set shell = CreateObject("WScript.Shell")
    paths = shell.ExpandEnvironmentStrings("%PATH%")
    For Each p In Split(paths, ";")
        p = Trim(p)
        If Len(p) > 0 Then
            cand = p
            If Right(cand, 1) <> "\" Then cand = cand & "\"
            If fso.FileExists(cand & "pythonw.exe") Then
                FindPython = cand & "pythonw.exe" : Exit Function
            End If
        End If
    Next
    For Each p In Split(paths, ";")
        p = Trim(p)
        If Len(p) > 0 Then
            cand = p
            If Right(cand, 1) <> "\" Then cand = cand & "\"
            If fso.FileExists(cand & "python.exe") Then
                FindPython = cand & "python.exe" : Exit Function
            End If
        End If
    Next

    ' ③ 常见的按用户安装位置（勾了"只为我安装"时不会进 PATH）
    Dim la
    la = shell.ExpandEnvironmentStrings("%LOCALAPPDATA%") & "\Programs\Python"
    If fso.FolderExists(la) Then
        Dim d, sub_
        Set d = fso.GetFolder(la).SubFolders
        For Each sub_ In d
            If fso.FileExists(sub_.Path & "\pythonw.exe") Then
                FindPython = sub_.Path & "\pythonw.exe" : Exit Function
            End If
        Next
    End If

    FindPython = ""
End Function


' ---------------------------------------------------------------------------
' 失败留痕：写文件 + 弹框。绝不静默 —— G-code 通道号没改对就发出去 = 印错颜色
' ---------------------------------------------------------------------------
Sub Fail(text)
    Dim f, tip
    On Error Resume Next
    Set f = fso.CreateTextFile(errFile, True)
    f.WriteLine "ESP-AMS 后处理失败  " & Now
    f.WriteLine String(50, "-")
    f.WriteLine text
    f.Close
    On Error GoTo 0

    tip = "ESP-AMS 后处理失败：" & vbCrLf & vbCrLf & text & vbCrLf & vbCrLf & _
          "（这份说明也存到了 " & errFile & "）"
    ' 4096 = 置顶 + 系统模态，16 = 错误图标
    MsgBox tip, 16 + 4096, "ESP-AMS 后处理"
    WScript.Quit 1
End Sub
