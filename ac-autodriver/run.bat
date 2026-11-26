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
echo   1  Telemetry monitor      (sit in an AC practice session)
echo   2  Controller bind test   (bind the axes in Content Manager)
echo   3  Vision: calibrate      (screenshot with ROI -^> calibration.png)
echo   4  Vision: live view      (debug window, lane offset, distances)
echo   5  Simulator              (compare cruise / simple / madmax)
echo   6  Self-tests
echo   7  Re-run installer
echo   Q  Quit
echo.
choice /c 1234567Q /n /m "  Pick: "
set "PICK=%errorlevel%"
echo.
if "%PICK%"=="8" exit /b 0
if "%PICK%"=="1" "%PY%" telemetry.py
if "%PICK%"=="2" "%PY%" controller.py --bind-test
if "%PICK%"=="3" "%PY%" vision.py --calibrate
if "%PICK%"=="4" "%PY%" vision.py --live
if "%PICK%"=="5" "%PY%" sim.py
if "%PICK%"=="6" for %%m in (telemetry controller vision planner sim) do (
    echo --- %%m
    "%PY%" %%m.py --selftest
)
if "%PICK%"=="7" call "%~dp0install.bat"
echo.
pause
goto menu
