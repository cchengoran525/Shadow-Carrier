@echo off
setlocal
set "SIM_AUTOTEST=1"
set "WORLD=%~dp0worlds\shadow_carrier.wbt"
set "WEBOTS=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if not exist "%WEBOTS%" set "WEBOTS=webots.exe"
echo [autotest] world  = %WORLD%
echo [autotest] webots = %WEBOTS%
echo [autotest] A Webots window will open and run ~90s by itself (keyboard ignored).
echo [autotest] Please do not close it.
"%WEBOTS%" --mode=realtime --stdout --stderr "%WORLD%"
echo [autotest] exit code %errorlevel%
echo [autotest] log file: %~dp0controllers\shadow_carrier\sim_session.jsonl
pause
