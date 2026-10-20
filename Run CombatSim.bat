@echo off
setlocal
set "COMBATSIM_EXE=%~dp0Standalone\build\Release\CombatSimStandalone.exe"
set "COMBATSIM_MODEL=%~dp0Assets\CombatSim\Models\CombatRobot500g.xml"

if not exist "%COMBATSIM_EXE%" (
    echo CombatSim has not been built yet.
    echo Run this once from the project folder:
    echo powershell -ExecutionPolicy Bypass -File .\Standalone\setup.ps1
    pause
    exit /b 1
)

cd /d "%~dp0"
"%COMBATSIM_EXE%" "%COMBATSIM_MODEL%"
if errorlevel 1 (
    echo.
    echo CombatSim stopped with an error. See the message above.
    pause
)
