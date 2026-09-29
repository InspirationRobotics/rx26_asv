@echo off
rem ==================================================================
rem  GZ_SIM_UP.cmd -- double-click to bring up the Gazebo sim of Crusader.
rem
rem  Runs in: Windows. Everything real happens in WSL (gz_sim_up.sh);
rem  this file only crosses the boundary and opens the pages.
rem
rem  Optional argument: a course name (default task1_core), e.g. from a
rem  terminal:   GZ_SIM_UP.cmd task3
rem
rem  Same WSL keep-alive as tools\sitl\SIM_UP.cmd, for the same reason:
rem  WSL2 stops the VM about a minute after the last client leaves, and
rem  takes Gazebo, SITL and the crsd-sim container with it, silently.
rem  GZ_SIM_DOWN.cmd kills the keep-alive.
rem ==================================================================
setlocal
set "DISTRO=Ubuntu-22.04"
set "SYS=%SystemRoot%\System32"
set "COURSE=%~1"
if "%COURSE%"=="" set "COURSE=task1_core"

set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"
for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

start "rx26-gz-keepalive" /min wsl.exe -d %DISTRO% -- sleep infinity

echo Bringing up the Gazebo sim (course %COURSE%). First run builds the workspace: a few minutes.
echo.
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/gz_sim_up.sh' > /tmp/gz_sim_up.sh && RX26_WIN_SRC='%HERE%/../..' bash /tmp/gz_sim_up.sh %COURSE%"
if errorlevel 2 goto :failed

start "" http://localhost:8090
start "" http://localhost:8085
echo.
echo Up. Gazebo's window is on the desktop; the ground station and the tree are in the browser.
echo Start Task 1 from a WSL terminal:
echo   docker exec -it crsd-sim bash -lc "python3 -m crusader_sim.task1_goal --course %COURSE%"
echo.
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
