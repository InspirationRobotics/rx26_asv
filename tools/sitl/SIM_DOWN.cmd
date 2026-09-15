@echo off
setlocal
title RobotX 2026  --  Task 1 sim  [DOWN]

rem ==================================================================
rem  SIM_DOWN.cmd -- double-click to stop everything SIM_UP.cmd started.
rem
rem  Order is ROS nodes, then SITL, then the container, and the reason
rem  is spelled out in sim_down.sh. QGroundControl is closed here
rem  because it is the only piece that is a Windows process.
rem
rem  The WSL VM is deliberately LEFT RUNNING -- docker and whatever else
rem  you keep in there is not this script's to kill. For the memory back
rem  as well, run:  wsl --shutdown
rem
rem  Windows tools are called by full path; see the note in SIM_UP.cmd.
rem ==================================================================

set "DISTRO=Ubuntu-22.04"
set "SYS=%SystemRoot%\System32"

set "HEREWIN=%~dp0"
set "HEREWIN=%HEREWIN:~0,-1%"

for /f "usebackq delims=" %%i in (`wsl.exe -d %DISTRO% -- wslpath -a "%HEREWIN%"`) do set "HERE=%%i"
if not defined HERE goto :nowsl

wsl.exe -d %DISTRO% -- bash -lc "tr -d '\r' < '%HERE%/sim_down.sh' > /tmp/rx26_sim_down.sh && bash /tmp/rx26_sim_down.sh '%HERE%/../..'"

echo.
echo === the WSL keepalive ===
rem Started by SIM_UP to stop WSL idling the VM out from under the rig. Matched
rem on its window title so this cannot take out another WSL session.
%SYS%\taskkill.exe /f /fi "WINDOWTITLE eq rx26-wsl-keepalive*" >nul 2>&1
echo   released

echo.
echo === QGroundControl ===
%SYS%\tasklist.exe /NH /FI "IMAGENAME eq QGroundControl.exe" 2>nul | %SYS%\find.exe /I "QGroundControl.exe" >nul
if errorlevel 1 goto :noqgc

rem Ask before forcing. A hard kill skips QGC's own shutdown, which is
rem what leaves a half-written .tlog behind in its telemetry folder.
%SYS%\taskkill.exe /IM QGroundControl.exe >nul 2>&1
rem ping, not timeout.exe: timeout REFUSES to run whenever stdin is
rem redirected ("Input redirection is not supported, exiting the process
rem immediately"), so from any shell that pipes this script the wait
rem vanished and QGC got force-killed a millisecond after being asked
rem politely. ping -n 4 against loopback is three seconds and does not care.
%SYS%\ping.exe -n 4 127.0.0.1 >nul
%SYS%\tasklist.exe /NH /FI "IMAGENAME eq QGroundControl.exe" 2>nul | %SYS%\find.exe /I "QGroundControl.exe" >nul
if errorlevel 1 goto :qgcclosed
%SYS%\taskkill.exe /F /IM QGroundControl.exe >nul 2>&1
echo   QGroundControl would not close and was forced.
goto :done

:qgcclosed
echo   QGroundControl closed.
goto :done

:noqgc
echo   QGroundControl was not running.
goto :done

:nowsl
echo.
echo   Could not reach WSL distro %DISTRO%.
echo   Check the name with:  wsl.exe -l -v
goto :end

:done
echo.
echo   Browser tabs on :8086 / :8085 / :8090 are yours to close.

:end
echo.
pause
