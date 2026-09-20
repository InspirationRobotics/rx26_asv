#!/usr/bin/env bash
# sim_down.sh -- stop the Task 1 simulator in the order that leaves nothing behind.
#
# Runs INSIDE WSL. On Windows double-click SIM_DOWN.cmd, which is this plus
# closing QGroundControl.
#
#   bash sim_down.sh [/mnt/c/... path to the Windows checkout]
#
# ORDER MATTERS, AND IT IS ROS NODES FIRST, WHILE SITL STILL ANSWERS. A
# telemetry_bridge that loses its autopilot mid-shutdown spends its last seconds
# retrying a dead socket and writes a page of connection errors into
# /tmp/tb.log. The next run's first `tail` reads that as a fault on startup,
# which it is not, and the hunt starts at the wrong end of the stack.
#
# IT STAGES ITS OWN TOOLS FIRST. Shutting down must work on a machine that has
# never brought the rig up -- a fresh clone, or a WSL workspace that predates
# these scripts -- so the three files this needs are copied in rather than
# assumed. The first run of this hit exactly that: nodes_down.sh did not exist
# in the container yet and every ROS node survived a "clean" shutdown.
#
# THE WSL VM IS LEFT RUNNING on purpose -- docker and anything else you keep in
# there is not this script's to kill. To get the memory back as well, from
# Windows:  wsl --shutdown
set -uo pipefail

DEFAULT_WIN=/mnt/c/Users/chase/OneDrive/Documents/GitHub/RobotX_2026/Boat/rx26_asv
WIN_SRC="$(cd "${1:-$DEFAULT_WIN}" 2>/dev/null && pwd)" || WIN_SRC=""
WSL_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
CTR_SRC=/root/robotx_ws/src/rx26_asv

step() { printf '\n=== %s ===\n' "$1"; }

# Copy one file Windows -> WSL, CR stripped. The container sees the result
# through the ~/robotx_ws bind mount, so this is also how it reaches the
# container -- there is no docker cp anywhere in this rig.
stage() {
  [ -n "$WIN_SRC" ] || return 1
  [ -f "$WIN_SRC/tools/sitl/$1" ] || return 1
  mkdir -p "$WSL_SRC/tools/sitl" || return 1
  tr -d '\r' < "$WIN_SRC/tools/sitl/$1" > "$WSL_SRC/tools/sitl/$1"
}
for f in nodes_down.sh rig_processes.txt start_sitl.sh; do stage "$f"; done

step "1/3  ROS nodes"
if [ -z "$(docker ps -q -f "name=^${CONTAINER}$" 2>/dev/null)" ]; then
  echo "  $CONTAINER was not running"
elif [ ! -f "$WSL_SRC/tools/sitl/nodes_down.sh" ]; then
  echo "  no nodes_down.sh at $WSL_SRC -- cannot stop the nodes by name."
  echo "  stopping the container below will take them with it."
else
  docker exec "$CONTAINER" bash "$CTR_SRC/tools/sitl/nodes_down.sh" 2>&1 | sed 's/^/  /'
fi

step "2/3  ArduRover SITL"
if [ -f "$WSL_SRC/tools/sitl/start_sitl.sh" ]; then
  bash "$WSL_SRC/tools/sitl/start_sitl.sh" --stop 2>&1 | sed 's/^/  /'
else
  pkill -f sim_vehicle 2>/dev/null
  pkill -f ardurover   2>/dev/null
  pkill -f mavproxy    2>/dev/null
  sleep 1
  echo "  stopped (no start_sitl.sh at $WSL_SRC)"
fi

step "3/3  the container"
# Safe to stop and restart: crsd-sim is AutoRemove=false with the workspace bind
# mounted, so `docker start` brings back the same container and the same build.
if docker stop -t 5 "$CONTAINER" > /dev/null 2>&1; then
  echo "  $CONTAINER stopped"
else
  echo "  $CONTAINER was already down"
fi

echo
echo "=== what is left ==="
# `pgrep -c` PRINTS 0 and RETURNS 1 when nothing matches, so the obvious
# `|| echo 0` prints the count twice.
for p in sim_vehicle ardurover mavproxy; do
  c="$(pgrep -c -f "$p" 2>/dev/null)"
  printf '  %-12s %s process(es)\n' "$p" "${c:-0}"
done
c="$(docker ps --format '{{.Names}}' 2>/dev/null | tr '\n' ' ')"
printf '  %-12s %s\n' containers "${c:-none}"
echo
echo "  all down. SIM_UP.cmd brings it back."
