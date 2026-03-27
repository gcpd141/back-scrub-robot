@echo off
chcp 65001 >nul
echo ========================================
echo   搓背机器人 - 联动版本 (推荐)
echo ========================================
echo.
echo 启动视觉 + 仿真联动版本...
echo.

cd /d "%~dp0"
python vision_driven_sim_fixed.py

pause
