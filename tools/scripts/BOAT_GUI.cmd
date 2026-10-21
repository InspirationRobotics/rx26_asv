@echo off
rem BOAT_GUI.cmd: double-click to start the ground station on this laptop.
rem Runs crusader_groundstation\crusader_groundstation\laptop\gcs_app.py (standard library
rem only) with the repo's .venv python if there is one, else "python". It connects to the
rem boat's gcs_bridge (default 192.168.100.109; --boat 127.0.0.1 for the sim) and opens
rem http://localhost:8090. Extra arguments go to gcs_app.py. Close this window to stop it.
setlocal
set "ROOT=%~dp0..\.."
set "PY=python"
if exist "%ROOT%\.venv\Scripts\python.exe" set "PY=%ROOT%\.venv\Scripts\python.exe"
curl.exe -sf -o nul -m 1 http://127.0.0.1:8090/ && (start "" http://localhost:8090/ & exit /b 0)
title Crusader GCS
"%PY%" "%ROOT%\crusader_groundstation\crusader_groundstation\laptop\gcs_app.py" %*
if errorlevel 1 pause
