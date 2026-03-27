@echo off
chcp 65001 >nul
echo ========================================
echo   Vision Module (Face Detection)
echo ========================================
echo.
echo Starting camera for face detection...
echo Coordinates saved to: face_coord.json
echo.
echo Press any key to stop
echo.

cd /d "%~dp0"
python vision.py

pause
