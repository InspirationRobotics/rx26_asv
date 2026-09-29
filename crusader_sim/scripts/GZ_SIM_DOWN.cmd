@echo off
rem GZ_SIM_DOWN.cmd -- stop the Gazebo sim and release the WSL keep-alive.
rem Runs in: Windows. Leaves the WSL VM itself to idle out on its own.
setlocal
set "DISTRO=Ubuntu-22.04"
set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"
for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/gz_sim_down.sh' > /tmp/gz_sim_down.sh && bash /tmp/gz_sim_down.sh"
taskkill /fi "WINDOWTITLE eq rx26-gz-keepalive*" /f >nul 2>&1
echo Stopped.
pause
