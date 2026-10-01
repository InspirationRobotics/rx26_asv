#!/usr/bin/env bash
# gz_task1.sh — run Task 1 on the sim gz_sim_up.sh brought up. Runs in: WSL2
# Ubuntu-22.04 (GZ_SIM_UP.cmd calls it after the countdown).
#
#     bash crusader_sim/scripts/gz_task1.sh [course] [task1_goal flags...]
#
# task1_goal arms, confirms GUIDED, sends the SafePassage goal and prints the
# tree's phases, then task1_judge's verdict. The run is also written to
# ~/.cache/crusader_sim/task1_last.log, so the verdict survives the window being
# closed (not /tmp: WSL wipes /tmp every time the distro starts).
# Ctrl-C stops the operator script; the tree itself keeps its goal until
# gz_sim_down.sh.
COURSE="${1:-task1_core}"
shift || true
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
LOG="$HOME/.cache/crusader_sim/task1_last.log"
mkdir -p "$(dirname "$LOG")"
docker exec -it "$CONTAINER" bash -c \
  "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash &&
   python3 -u -m crusader_sim.task1_goal --course $COURSE $*" 2>&1 \
  | tee "$LOG"
