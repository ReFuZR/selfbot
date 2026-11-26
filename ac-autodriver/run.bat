@echo off
rem ac-autodriver launcher menu
cd /d "%~dp0"
set "PY=%~dp0.venv\Scripts\python.exe"
if not exist "%PY%" (
    echo Not installed yet - running install.bat first.
    call "%~dp0install.bat"
    if not exist "%PY%" exit /b 1
)

:menu
cls
echo.
echo   ac-autodriver
echo   -------------------------------------------------------------
echo   1  DRIVE                  (F8 engage/pause, F9 mode, F12 kill)
echo   2  Wheel setup            (learn your Logitech's axes - once)
echo   3  Bind test              (bind the virtual wheel in Content Manager)
echo   4  Telemetry monitor      (sit in an AC session)
echo   5  Vision: calibrate      (screenshot with ROI -^> calibration.png)
echo   6  Vision: live view      (debug window, lane offset, distances)
echo   7  Simulator              (compare cruise / simple / madmax)
echo   8  Self-tests
echo   9  Re-run installer
echo   Q  Quit
echo.
choice /c 123456789Q /n /m "  Pick: "
set "PICK=%errorlevel%"
echo.
if "%PICK%"=="10" exit /b 0
if "%PICK%"=="1" goto drive
if "%PICK%"=="2" "%PY%" controller.py --wheel-setup
if "%PICK%"=="3" "%PY%" controller.py --bind-test
if "%PICK%"=="4" "%PY%" telemetry.py
if "%PICK%"=="5" "%PY%" vision.py --calibrate
if "%PICK%"=="6" "%PY%" vision.py --live
if "%PICK%"=="7" "%PY%" sim.py
if "%PICK%"=="8" for %%m in (telemetry controller vision planner main sim) do (
    echo --- %%m
    "%PY%" %%m.py --selftest
)
if "%PICK%"=="9" call "%~dp0install.bat"
echo.
pause
goto menu

:drive
choice /c 0123 /n /m "  Mode - 0: config.py  1: cruise  2: simple  3: madmax : "
set "M=%errorlevel%"
set "MODEARG="
if "%M%"=="2" set "MODEARG=--mode cruise"
if "%M%"=="3" set "MODEARG=--mode simple"
if "%M%"=="4" set "MODEARG=--mode madmax"
if not exist logs mkdir logs
for /f %%t in ('powershell -NoProfile -Command "Get-Date -Format yyyyMMdd-HHmmss"') do set "STAMP=%%t"
"%PY%" main.py %MODEARG% --log "logs\run-%STAMP%.csv"
echo.
echo Log saved to logs\run-%STAMP%.csv
pause
goto menu
