@echo off
chcp 65001 >nul
echo ========================================
echo   Combined Mode (Recommended)
echo ========================================
echo.
echo Starting vision + simulation combined...
echo.

cd /d "%~dp0"
python vision_driven_sim_fixed.py

pause
