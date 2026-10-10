@echo off
set SIM_AUTOTEST=1
set SIM_AUTOTEST_MOTION=1
set "WEBOTS=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if not exist "%WEBOTS%" set "WEBOTS=webots.exe"
"%WEBOTS%" --mode=realtime --stdout --stderr "%~dp0worlds\_autotest.wbt"
exit /b %errorlevel%
