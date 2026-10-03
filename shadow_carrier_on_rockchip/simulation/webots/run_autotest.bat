@echo off
setlocal
set "SIM_AUTOTEST=1"
set "WORLD=%~dp0worlds\shadow_carrier.wbt"
set "WEBOTS=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if not exist "%WEBOTS%" set "WEBOTS=webots.exe"
echo [autotest] world  = %WORLD%
echo [autotest] webots = %WEBOTS%
echo [autotest] running ~90s, please wait...
"%WEBOTS%" --batch --mode=realtime --stdout --stderr "%WORLD%"
echo [autotest] exit code %errorlevel%
echo [autotest] log file: %~dp0controllers\shadow_carrier\sim_session.jsonl
pause
