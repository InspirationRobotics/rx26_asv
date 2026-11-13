@echo off
rem ==================================================================
rem  GZ_SIM_DOWN.cmd -- double-click: stop everything GZ_SIM_UP.cmd started.
rem
rem  Runs in: Windows. Stops the ROS rig and the crsd-sim container, SITL
rem  and MAVProxy, the transmitter and Gazebo (server and window), then
rem  releases the WSL keep-alive. It does NOT shut WSL down: the Ubuntu-22.04 distro is
rem  shared with other work, so the VM is left to idle out by itself
rem  about a minute later.
rem ==================================================================
setlocal
rem Chase and Chris have Ubuntu-22.04; a plain "Ubuntu" (also 22.04) works too
set "DISTRO=Ubuntu-22.04"
wsl.exe -d %DISTRO% -- true >nul 2>&1 || set "DISTRO=Ubuntu"
title Crusader sim - stopping

set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"
for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

echo Stopping the ROS rig, SITL, the transmitter and Gazebo...
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/gz_sim_down.sh' > /tmp/gz_sim_down.sh && bash /tmp/gz_sim_down.sh"
rem the hidden keep-alive exits cleanly within 2 s of this file appearing
wsl.exe -d %DISTRO% -e touch /tmp/rx26_gz_keepalive.stop
echo Keep-alive released.
echo.
echo Stopped. This window closes by itself.
timeout /t 8 >nul
exit /b 0

:nowsl
echo Could not reach WSL distro %DISTRO%.
pause
exit /b 2
