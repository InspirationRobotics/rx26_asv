@echo off
rem ==================================================================
rem  TASK1_PANEL.cmd -- double-click: the Task 1 operator panel, in the
rem  browser. Place the ten buoys, launch the sim, then play the UAV.
rem  GZ_SIM_DOWN.cmd stops the sim AND the panel.
rem
rem  Runs in: Windows. The panel is a plain python3 process on the WSL
rem  host (task1_panel_up.sh -> crusader_sim.task1_panel on :8095),
rem  started hidden, and it runs gz_sim_up.sh itself. This window only
rem  starts it, finds an address Windows can reach it on, and opens the
rem  browser.
rem
rem  Running it again replaces a running panel; a running sim is left alone.
rem ==================================================================
setlocal
set "DISTRO=Ubuntu-22.04"
set "PORT=8095"
title Crusader sim - Task 1 panel

set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"
for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

rem The hidden start below cannot carry a path with spaces in it:
rem PowerShell 5.1 joins -ArgumentList with spaces and quotes nothing. So
rem the script is run from a copy in /tmp (CR stripped, as always), and the
rem checkout's location crosses as an environment variable instead, which
rem WSLENV's /p flag turns from a Windows path into the WSL one.
for %%i in ("%~dp0..\..") do set "RX26_WIN_SRC=%%~fi"
if defined WSLENV (set "WSLENV=%WSLENV%:RX26_WIN_SRC/p") else set "WSLENV=RX26_WIN_SRC/p"
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/task1_panel_up.sh' > /tmp/task1_panel_up.sh"
if errorlevel 1 goto :nowsl

rem Hidden, and detached from this window: the panel outlives it, and
rem while it runs it is the wsl.exe client that keeps the WSL VM up.
echo Starting the Task 1 panel (it syncs the Windows checkout into WSL first).
powershell -NoProfile -Command "Start-Process -WindowStyle Hidden -FilePath wsl.exe -ArgumentList '-d','%DISTRO%','--cd','~','-e','bash','/tmp/task1_panel_up.sh'"

rem WHICH HOST ANSWERS FROM WINDOWS (tools/sitl/SIM_UP.cmd has the story):
rem WSL's localhost proxy sometimes refuses Windows connections to a port
rem that serves fine inside WSL; the VM's own address always works. Both
rem are tried until one answers or about a minute is up.
rem The probe is curl.exe, NOT PowerShell's Invoke-WebRequest: this file
rem with a hidden Start-Process AND an Invoke-WebRequest loop in it was
rem deleted by this PC's malware protection the moment it was opened,
rem before its first line ran (2026-09-30; either line alone is left alone).
set "VMIP="
for /f "usebackq tokens=1" %%i in (`wsl.exe -d %DISTRO% -- hostname -I`) do set "VMIP=%%i"
echo Waiting for it to answer on port %PORT% (up to a minute)...
set "HOST="
for /l %%n in (1,1,15) do if not defined HOST call :probe
if not defined HOST goto :noanswer
if /i "%HOST%"=="localhost" goto :hostok
echo   localhost is not reaching WSL; using %HOST% instead.
echo   (WSL's localhost proxy has dropped. "wsl --shutdown" restores it, but
echo   also stops the panel and any running sim.)
:hostok

start "" http://%HOST%:%PORT%
echo.
echo Task 1 panel:  http://%HOST%:%PORT%
echo It keeps running after this window closes; GZ_SIM_DOWN.cmd stops it,
echo and the sim with it. Its log is /tmp/task1_panel.log in WSL.
timeout /t 8 >nul
exit /b 0

:nowsl
echo Could not reach WSL distro %DISTRO%.
pause
exit /b 2

:noanswer
echo.
echo *** The panel did not answer within a minute. The end of its log
echo     (/tmp/task1_panel.log in WSL):
echo.
wsl.exe -d %DISTRO% -- tail -n 20 /tmp/task1_panel.log
echo.
pause
exit /b 2

rem one round of the wait: each host once, then 2 s if neither answered
:probe
for %%h in (localhost %VMIP%) do if not defined HOST curl.exe -sf -o nul -m 2 "http://%%h:%PORT%/api/state" && set "HOST=%%h"
if not defined HOST timeout /t 2 >nul
exit /b 0
