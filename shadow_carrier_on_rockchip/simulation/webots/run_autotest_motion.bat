@echo off
setlocal
set "SIM_AUTOTEST=1"
set "SIM_AUTOTEST_MOTION=1"
set "SRC=%~dp0worlds\shadow_carrier.wbt"
set "WORLD=%~dp0worlds\_autotest.wbt"
set "WEBOTS=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if not exist "%WEBOTS%" set "WEBOTS=webots.exe"

rem 跑临时副本: 即使 Webots 提示保存, 也只写临时文件, 原始世界永不被改
copy /Y "%SRC%" "%WORLD%" >nul

echo [autotest-motion] world  = %WORLD% (temp copy)
echo [autotest-motion] HRI actions DO drive the chassis (APPROACH/BACK_OFF/GOTO_SAFE)
echo [autotest-motion] A Webots window opens and runs ~90s by itself.
echo [autotest-motion] If asked to save changes, choose "Don't Save".
"%WEBOTS%" --mode=realtime --stdout --stderr "%WORLD%"
set RC=%errorlevel%
del "%WORLD%" >nul 2>&1
echo [autotest-motion] exit code %RC%
echo [autotest-motion] log file: %~dp0controllers\shadow_carrier\sim_session.jsonl
pause
