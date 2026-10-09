@echo off
rem ==================================================================
rem  BOAT_GUI.cmd -- double-click: Crusader's ground station page,
rem  served from THIS laptop, opened in the default browser.
rem
rem  Runs in: Windows. gcs_laptop.py (system python, no ROS) serves the
rem  boat's own page on http://localhost:8150, takes the numbers from
rem  the Jetson over the network, and reads the autopilot's MAVLink
rem  listen-only. It runs in its own window, titled "Boat GUI (laptop)";
rem  close that window (or Ctrl+C in it) to stop it. If the server is
rem  already answering, this just opens the browser - no second one.
rem
rem  Extra arguments go to gcs_laptop.py, e.g.
rem    BOAT_GUI.cmd --mav udpin:0.0.0.0:14550 --jetson 192.168.8.109
rem  Set BOAT_GUI_NO_BROWSER=1 to skip opening the browser.
rem
rem  The probe is curl.exe, NOT PowerShell's Invoke-WebRequest, and the
rem  server window is an ordinary visible one: a launcher that used a
rem  hidden Start-Process with an Invoke-WebRequest loop was deleted by
rem  this PC's malware protection within 2 s of being opened
rem  (2026-09-30; see crusader_sim/scripts/TASK1_PANEL.cmd).
rem ==================================================================
setlocal EnableDelayedExpansion
set "PORT=8150"
set "PREV="
rem --port N or --port=N (the for loop splits on = as well)
for %%a in (%*) do (
  if "!PREV!"=="--port" set "PORT=%%~a"
  set "PREV=%%~a"
)
set "PROBE=http://127.0.0.1:%PORT%/"

curl.exe -sf -o nul -m 1 "%PROBE%"
if not errorlevel 1 (
  echo The Boat GUI is already running on port %PORT%.
  goto :open
)

python -c "import sys" >nul 2>&1
if errorlevel 1 goto :nopython

echo Starting the Boat GUI server in its own window...
rem cmd /k keeps that window open if the server exits, so a startup
rem error (bad config, port in use) can be read instead of vanishing.
start "Boat GUI (laptop)" cmd /k python "%~dp0gcs_laptop.py" %*

set "UP="
echo Waiting for it to answer on port %PORT% (up to 15 s)...
for /l %%n in (1,1,15) do if not defined UP call :probe
if not defined UP goto :noanswer

:open
if defined BOAT_GUI_NO_BROWSER goto :done
start "" "http://localhost:%PORT%/"
:done
exit /b 0

:nopython
echo.
echo *** "python" was not found, or is the Microsoft Store stub.
echo     Install Python 3 for Windows with pymavlink, pyserial and PyYAML:
echo         python -m pip install pymavlink pyserial PyYAML
echo.
pause
exit /b 2

:noanswer
echo.
echo *** Nothing answered on port %PORT% within 15 s. Read the window
echo     titled "Boat GUI (laptop)" for the reason.
echo.
pause
exit /b 2

rem one round of the wait. ping is the one-second sleep: timeout.exe
rem refuses to run when stdin is not a console and would end the wait
rem instantly.
:probe
curl.exe -sf -o nul -m 1 "%PROBE%" && set "UP=1"
if not defined UP ping -n 2 127.0.0.1 >nul
exit /b 0
