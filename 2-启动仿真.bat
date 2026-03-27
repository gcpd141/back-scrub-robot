@echo off
chcp 65001 >nul
echo ========================================
echo   搓背机器人 - 仿真模块
echo ========================================
echo.
echo 启动 MuJoCo 仿真...
echo 将读取 face_coord.json 控制机械臂
echo.
echo 按任意键停止
echo.

cd /d "%~dp0"
python simulation.py

pause
