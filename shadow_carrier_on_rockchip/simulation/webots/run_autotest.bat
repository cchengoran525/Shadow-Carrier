@echo off
setlocal
set "SIM_AUTOTEST=1"
set "SRC=%~dp0worlds\shadow_carrier.wbt"
set "WORLD=%~dp0worlds\_autotest.wbt"
set "WEBOTS=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if not exist "%WEBOTS%" set "WEBOTS=webots.exe"

rem 跑临时副本: 即使 Webots 提示保存, 也只写临时文件, 原始世界永不被改
copy /Y "%SRC%" "%WORLD%" >nul

echo [autotest] world  = %WORLD% (temp copy)
echo [autotest] webots = %WEBOTS%
echo [autotest] A Webots window opens and runs ~90s by itself (keyboard ignored).
echo [autotest] If asked to save changes, choose "Don't Save" (or "No").
"%WEBOTS%" --mode=realtime --stdout --stderr "%WORLD%"
set RC=%errorlevel%
del "%WORLD%" >nul 2>&1
echo [autotest] exit code %RC%
echo [autotest] log file: %~dp0controllers\shadow_carrier\sim_session.jsonl
pause
