@echo off
chcp 65001 >nul
echo ========================================
echo   搓背机器人 - 视觉模块
echo ========================================
echo.
echo 启动摄像头进行人脸检测...
echo 坐标将保存至：face_coord.json
echo.
echo 按任意键停止
echo.

cd /d "%~dp0"
python vision.py

pause
