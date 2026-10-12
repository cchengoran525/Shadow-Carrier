@echo off
REM 运行指定鲁棒性场景: _run_scenario.bat <scenario> [phantom_seed] [phantom_rate]
setlocal
set SIM_AUTOTEST=1
set SIM_AUTOTEST_MOTION=1
set "SIM_SCENARIO=%~1"
if not "%~2"=="" set "SIM_PHANTOM_SEED=%~2"
if not "%~3"=="" set "SIM_PHANTOM_RATE=%~3"
set "WEBOTS=C:\Program Files\Webots\msys64\mingw64\bin\webots.exe"
if not exist "%WEBOTS%" set "WEBOTS=webots.exe"
echo [rob] scenario=%SIM_SCENARIO% seed=%SIM_PHANTOM_SEED% rate=%SIM_PHANTOM_RATE%
"%WEBOTS%" --mode=realtime --batch --minimize --stdout --stderr "%~dp0worlds\_autotest.wbt"
exit /b %errorlevel%
