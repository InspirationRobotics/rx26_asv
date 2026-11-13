@echo off
rem ==================================================================
rem  GZ_SIM_UP.cmd -- double-click: the whole Gazebo sim of Crusader,
rem  then Task 1.  GZ_SIM_DOWN.cmd stops all of it.
rem
rem  Runs in: Windows. Everything real happens in WSL (gz_sim_up.sh);
rem  this file only crosses the boundary, opens the pages, and runs the
rem  mission in this window so its verdict stays on screen.
rem
rem  Optional argument: a course name, e.g. from a terminal:
rem  GZ_SIM_UP.cmd task3 (no mission is started for a course that isn't a
rem  Task 1 course). Double-clicked, it lists the courses and asks.
rem
rem  Then, for a Task 1 course: G = arm + GUIDED only (the pilot's SB and
rem  mode switch), and you run the mission from the ground station as on
rem  the water: Mission tab, tree, Run panel (field centre), Start, then
rem  "Release without RoboCommand". T = the scripted Task 1 run instead.
rem
rem  Running it while the sim is up restarts everything from scratch.
rem ==================================================================
setlocal EnableDelayedExpansion
rem Chase and Chris have Ubuntu-22.04; a plain "Ubuntu" (also 22.04) works too
set "DISTRO=Ubuntu-22.04"
wsl.exe -d %DISTRO% -- true >nul 2>&1 || set "DISTRO=Ubuntu"
set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"

set "COURSE=%~1"
if not "%COURSE%"=="" goto :have_course
echo Courses (crusader_sim\courses):
set /a N=0
for %%f in ("%HEREWIN%\..\courses\*.yaml") do (
  set /a N+=1
  set "C_!N!=%%~nf"
  echo   !N!  %%~nf
)
set "PICK="
set /p "PICK=Course number (Enter = task1_core): "
if defined PICK set "COURSE=!C_%PICK%!"
if "%COURSE%"=="" set "COURSE=task1_core"
:have_course
title Crusader sim - %COURSE%

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

rem the ground station, connected to the sim's gcs_bridge
start "Crusader GCS" cmd /c "%HEREWIN%\..\..\tools\scripts\BOAT_GUI.cmd" --boat 127.0.0.1

if /i not "%COURSE:~0,5%"=="task1" goto :idle
echo.
echo Gazebo is on the desktop; the ground station is in the browser.
echo   G = arm + GUIDED only, then run the mission from the ground station (default, 20 s)
echo   T = the scripted Task 1 run (task1_advanced)
echo   N = nothing: drive it yourself
choice /c GTN /t 20 /d G /m "Choose"
set "ACT=%errorlevel%"
if "%ACT%"=="3" goto :idle
echo.
if "%ACT%"=="1" (
  wsl.exe -d %DISTRO% --cd ~ -e bash robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_task1.sh %COURSE% --arm-only
  echo.
  echo Now in the ground station: Mission tab - pick the tree, set the Run panel's
  echo field centre with Pick on map, Start, then Release without RoboCommand.
  goto :idle
)
wsl.exe -d %DISTRO% --cd ~ -e bash robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_task1.sh %COURSE%
echo.
echo Task 1 run over. The log is ~/.cache/crusader_sim/task1_last.log in WSL.

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
