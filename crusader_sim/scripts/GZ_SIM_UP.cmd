@echo off
rem ==================================================================
rem  GZ_SIM_UP.cmd -- double-click: the whole Gazebo sim of Crusader,
rem  then Task 1.  GZ_SIM_DOWN.cmd stops all of it.
rem
rem  Runs in: Windows. Everything real happens in WSL (gz_sim_up.sh);
rem  this file only crosses the boundary, opens the pages, and runs the
rem  mission in this window so its verdict stays on screen.
rem
rem  Optional argument: a course name (default task1_core), e.g. from a
rem  terminal:   GZ_SIM_UP.cmd task3      (no mission is started for a
rem  course that isn't a Task 1 course)
rem
rem  Running it while the sim is up restarts everything from scratch.
rem ==================================================================
setlocal
set "DISTRO=Ubuntu-22.04"
set "COURSE=%~1"
if "%COURSE%"=="" set "COURSE=task1_core"
title Crusader sim - %COURSE%

set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"
for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

echo Bringing up the Gazebo sim (course %COURSE%). About a minute; the first run
echo ever also builds the workspace, which takes a few minutes more.
echo.
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/gz_sim_up.sh' > /tmp/gz_sim_up.sh && RX26_WIN_SRC='%HERE%/../..' bash /tmp/gz_sim_up.sh %COURSE%"
if errorlevel 2 goto :failed

rem WSL2 stops its VM about a minute after the last wsl.exe client exits,
rem taking the sim with it. gz_keepalive.sh is that client, hidden;
rem GZ_SIM_DOWN.cmd releases it.
powershell -NoProfile -Command "Start-Process -WindowStyle Hidden -FilePath wsl.exe -ArgumentList '-d','%DISTRO%','--cd','~','-e','bash','robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_keepalive.sh'"

start "" http://localhost:8090
start "" http://localhost:8085

if /i "%COURSE:~0,5%"=="task3" goto :task3
if /i not "%COURSE:~0,5%"=="task1" goto :idle
echo.
echo Gazebo is on the desktop; the ground station and the tree are in the browser.
choice /c YN /t 20 /d Y /m "Run Task 1 now? It starts by itself in 20 s. N = skip it and drive yourself"
if "%errorlevel%"=="2" goto :idle
echo.
wsl.exe -d %DISTRO% --cd ~ -e bash robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_task1.sh %COURSE%
echo.
echo Task 1 run over. The log is ~/.cache/crusader_sim/task1_last.log in WSL.
goto :idle

:task3
echo.
echo Gazebo is on the desktop; the ground station and the tree are in the browser.
choice /c YN /t 20 /d Y /m "Run Task 3 now? It starts by itself in 20 s. N = skip it and drive yourself"
if "%errorlevel%"=="2" goto :idle
echo.
wsl.exe -d %DISTRO% --cd ~ -e bash robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_task3.sh %COURSE%
echo.
echo Task 3 run over. The log is ~/.cache/crusader_sim/task3_last.log in WSL.

:idle
echo.
echo The sim keeps running until you double-click GZ_SIM_DOWN.cmd.
echo To run Task 1 again from the same start, run GZ_SIM_UP.cmd again: it
echo restarts everything, and the boat goes back to the start.
pause
exit /b 0

:nowsl
echo Could not reach WSL distro %DISTRO%.
pause
exit /b 2

:failed
echo.
echo *** The sim did not come up. Logs are in WSL: /tmp/gz_server.log /tmp/sitl.log /tmp/start_sitl.out
pause
exit /b 2
