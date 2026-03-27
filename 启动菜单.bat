@echo off
chcp 65001 >nul
echo ========================================
echo   搓背机器人系统 - 快速开始
echo ========================================
echo.
echo 请选择要执行的操作：
echo.
echo   [1] 启动视觉模块 (检测人脸)
echo   [2] 启动仿真模块 (机械臂控制)
echo   [3] 联动运行 (推荐)
echo   [0] 退出
echo.
set /p choice=请输入选项 (0-3)：

if "%choice%"=="1" start "" "1-启动视觉.bat"
if "%choice%"=="2" start "" "2-启动仿真.bat"
if "%choice%"=="3" start "" "3-联动运行.bat"
if "%choice%"=="0" exit

echo.
pause
