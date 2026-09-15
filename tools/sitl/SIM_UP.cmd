@echo off
setlocal
title RobotX 2026  --  Task 1 sim  [UP]

rem ==================================================================
rem  SIM_UP.cmd -- double-click to bring up the Task 1 simulator.
rem
rem  Every decision lives in sim_up.sh, inside WSL. This file only
rem  crosses the Windows/WSL boundary and then opens what a person
rem  needs to look at. Logic put HERE instead would be logic that
rem  cannot be run or tested from the WSL side, which is where the
rem  whole simulator actually lives.
rem
rem  Still running when this window closes: ArduRover SITL and MAVProxy
rem  in WSL, the ROS nodes in the crsd-sim container, three browser
rem  tabs and QGroundControl. SIM_DOWN.cmd takes all of it back down.
rem
rem  IF YOU PIPE THIS SCRIPT it will look like it hangs at the end, and
rem  it has not: `start` hands the browser and QGroundControl a copy of
rem  this window's stdout handle, so the pipe never reaches EOF while
rem  they are open. Double-click it, or redirect only the WSL line.
rem
rem  EVERY WINDOWS TOOL IS CALLED BY FULL PATH. Run from a Git Bash
rem  shell -- which is how this was tested -- the inherited PATH puts
rem  /usr/bin first, so a bare `find` is GNU find and `timeout` is GNU
rem  timeout. Both then fail on the /I and /t switches, silently enough
rem  that the QGroundControl check simply reported the wrong answer.
rem ==================================================================

set "DISTRO=Ubuntu-22.04"
set "QGC=C:\Program Files\QGroundControl\bin\QGroundControl.exe"
set "SYS=%SystemRoot%\System32"

rem %~dp0 keeps a trailing backslash, which escapes the closing quote
rem handed to wslpath and turns the path into nonsense. Strip it.
set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"

for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

rem HOLD THE VM OPEN. WSL2 shuts the VM down after about a minute with no
rem client attached, and that takes dockerd and every ROS node in crsd-sim with
rem it: `docker start` brings the container back but PID 1 is `tail -f
rem /dev/null`, so the rig is gone and the ports simply stop answering. Nothing
rem logs a fault because nothing faulted.
rem
rem One detached `sleep infinity` is a client, and that is enough. SIM_DOWN
rem kills it. Killing it by hand is `taskkill /f /im wsl.exe`, which also ends
rem every other WSL session, so prefer SIM_DOWN.
start "rx26-wsl-keepalive" /min wsl.exe -d %DISTRO% -- sleep infinity

echo Bringing up the Task 1 simulator. Give it about a minute.
echo.
wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/sim_up.sh' > /tmp/rx26_sim_up.sh && bash /tmp/rx26_sim_up.sh '%HERE%/../..'"
if errorlevel 2 goto :failed
if errorlevel 1 echo   WARNING: a page did not answer. Opening the rest anyway.

rem WHICH HOST ACTUALLY ANSWERS FROM WINDOWS.
rem
rem WSL2 forwards localhost into the VM through a proxy, and that proxy stops
rem working after the VM restarts often enough to be a real nuisance: the pages
rem serve perfectly inside WSL, `ss -ltn` shows them bound, and
rem http://localhost:8086 is refused from Windows with nothing logged anywhere.
rem Seen 2026-09-14 and again 2026-09-15.
rem
rem The VM's own address always works, because that route is direct rather than
rem proxied. So try localhost, and fall back rather than hand over three tabs
rem that will not load. The address changes when WSL restarts, which is why
rem localhost is tried first and why this is not simply hardcoded.
set "HOST=localhost"
for /f %%i in ('%SYS%\WindowsPowerShell\v1.0\powershell.exe -NoProfile -Command "try{(Invoke-WebRequest http://localhost:8086 -UseBasicParsing -TimeoutSec 6).StatusCode}catch{0}"') do set "RC=%%i"
if "%RC%"=="200" goto :hostok
for /f "usebackq delims= " %%i in (`wsl.exe -d %DISTRO% -- hostname -I`) do set "HOST=%%i"
echo   localhost is not reaching WSL; using %HOST% instead.
echo   (WSL's localhost proxy has dropped. "wsl --shutdown" restores it.)
:hostok

start "" http://%HOST%:8086
start "" http://%HOST%:8085
start "" http://%HOST%:8090

%SYS%\tasklist.exe /NH /FI "IMAGENAME eq QGroundControl.exe" 2>nul | %SYS%\find.exe /I "QGroundControl.exe" >nul
if not errorlevel 1 goto :qgcrunning
if not exist "%QGC%" goto :noqgc
start "" "%QGC%"
echo   QGroundControl started. It binds UDP 14550 and connects itself.
goto :done

:qgcrunning
echo   QGroundControl already running.
goto :done

:noqgc
echo   QGroundControl not found at %QGC% -- skipped.
goto :done

:failed
echo.
echo   BRING-UP FAILED, nothing was opened. The reason is above.
goto :end

:nowsl
echo.
echo   Could not reach WSL distro %DISTRO%.
echo   Check the name with:  wsl.exe -l -v
goto :end

:done
echo.
echo   http://%HOST%:8086  aircraft   place buoys, Transmit, Send goal, Confirm
echo   http://%HOST%:8085  tree       which leaf is RUNNING, what just failed
echo   http://%HOST%:8090  boat       map, pose, fused targets
echo.
echo   Shut everything down with SIM_DOWN.cmd, in this same folder.

:end
echo.
pause
