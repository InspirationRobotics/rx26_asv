#!/usr/bin/env bash
# gz_keepalive.sh — hold the WSL VM up while the sim runs. Runs in: WSL2
# Ubuntu-22.04, started hidden by GZ_SIM_UP.cmd.
#
# WSL2 stops the VM about a minute after the last wsl.exe client exits, and
# takes Gazebo, SITL and the crsd-sim container with it, silently. This loop is
# that client. GZ_SIM_DOWN.cmd ends it by creating the stop file, so it exits 0
# (a killed client would leave an error behind in a terminal tab).
STOP=/tmp/rx26_gz_keepalive.stop
rm -f "$STOP"
while [ ! -e "$STOP" ]; do sleep 2; done
