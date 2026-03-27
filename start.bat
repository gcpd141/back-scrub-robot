@echo off
chcp 65001 >nul
echo ========================================
echo   Back Scrub Robot - Menu
echo ========================================
echo.
echo Select an option:
echo.
echo   [1] Start Vision (face detection)
echo   [2] Start Simulation (robot arm)
echo   [3] Run Combined (recommended)
echo   [0] Exit
echo.
set /p choice=Enter choice (0-3): 

if "%choice%"=="1" start "" "1-vision.bat"
if "%choice%"=="2" start "" "2-simulation.bat"
if "%choice%"=="3" start "" "3-combined.bat"
if "%choice%"=="0" exit

echo.
pause
