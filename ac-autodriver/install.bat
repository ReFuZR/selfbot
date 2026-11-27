@echo off
rem Double-click to install everything ac-autodriver needs. Safe to re-run.
cd /d "%~dp0"
powershell -NoProfile -ExecutionPolicy Bypass -File "%~dp0install.ps1" %*
echo.
pause
