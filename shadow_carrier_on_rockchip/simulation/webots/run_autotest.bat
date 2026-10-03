@echo off
REM 一键跑 Webots 自动测试(无需按键): 脚本驱动主人按时间轴演一遍, 写 sim_session.jsonl, 跑完自动退出.
REM 用法: 双击本文件; 跑完把 controllers\shadow_carrier\sim_session.jsonl 留着即可(在仓库里, WSL 侧可直接读).
setlocal
set SIM_AUTOTEST=1
set "WORLD=%~dp0worlds\shadow_carrier.wbt"

set "WEBOTS=webots.exe"
where webots.exe >nul 2>&1
if errorlevel 1 (
  if exist "%ProgramFiles%\Webots\msys64\mingw64\bin\webots.exe" (
    set "WEBOTS=%ProgramFiles%\Webots\msys64\mingw64\bin\webots.exe"
  ) else if exist "%ProgramFiles%\Webots\webots.exe" (
    set "WEBOTS=%ProgramFiles%\Webots\webots.exe"
  ) else (
    echo [错误] 找不到 webots.exe, 请手动把 WEBOTS 改成你的安装路径.
    pause
    exit /b 1
  )
)

echo [autotest] world = %WORLD%
echo [autotest] 运行约 90 秒, 请勿关闭窗口...
"%WEBOTS%" --batch --mode=realtime --stdout --stderr "%WORLD%"
echo [autotest] 结束. 日志: %~dp0controllers\shadow_carrier\sim_session.jsonl
pause
