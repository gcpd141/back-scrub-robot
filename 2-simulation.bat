@echo off
chcp 65001 >nul
echo ========================================
echo   Simulation Module (Robot Arm)
echo ========================================
echo.
echo Starting MuJoCo simulation...
echo Reads face_coord.json to control arm
echo.
echo Press any key to stop
echo.

cd /d "%~dp0"
python simulation.py

pause
