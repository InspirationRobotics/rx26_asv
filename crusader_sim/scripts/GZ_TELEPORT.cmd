@echo off
rem ==================================================================
rem  GZ_TELEPORT.cmd -- double-click: move the SIM boat instantly
rem  (Gazebo only; never the real boat). Enter = back to the course's
rem  start. Or type a GPS point:  lat lon [heading]   (commas are fine;
rem  heading in compass degrees, 0 = north; blank keeps the bow).
rem  Not while a mission runs. gz_teleport.sh does the work in WSL.
rem ==================================================================
setlocal
rem Chase and Chris have Ubuntu-22.04; a plain "Ubuntu" (also 22.04) works too
set "DISTRO=Ubuntu-22.04"
wsl.exe -d %DISTRO% -- true >nul 2>&1 || set "DISTRO=Ubuntu"
set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"
for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

echo Move the sim boat.
echo   Enter          = back to the course's start
echo   lat lon        = to a GPS point, e.g.  1.2806344, 103.8556542
echo   lat lon hdg    = and face hdg degrees (0 = north)
set "WHERE="
set /p "WHERE=Where: "
if not defined WHERE set "WHERE=start"
set "WHERE=%WHERE:,= %"
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/gz_teleport.sh' > /tmp/gz_teleport.sh && bash /tmp/gz_teleport.sh %WHERE%"
pause
exit /b 0

:nowsl
echo Could not reach WSL distro %DISTRO%.
pause
exit /b 2
