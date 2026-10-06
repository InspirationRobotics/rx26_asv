@echo off
rem ==================================================================
rem  TASK3_SIM.cmd -- double-click: the Gazebo sim on the Task 3 dock,
rem  then the whole Task 3 run with the pan/tilt cannon (approach in
rem  GUIDED, dock on the LiDAR in MANUAL, aim and spray, the reports),
rem  and the referee's verdict. GZ_SIM_DOWN.cmd stops all of it.
rem
rem  It is GZ_SIM_UP.cmd on course task3. Another dock course from a
rem  terminal:   TASK3_SIM.cmd task3_ul
rem ==================================================================
setlocal
set "C=%~1"
if "%C%"=="" set "C=task3"
call "%~dp0GZ_SIM_UP.cmd" %C%
