#!/usr/bin/env bash
# gz_task3.sh — run Task 3 on the sim gz_sim_up.sh brought up. Runs in: WSL2
# Ubuntu-22.04 (TASK3_SIM.cmd / GZ_SIM_UP.cmd task3 call it after the countdown).
#
#     bash crusader_sim/scripts/gz_task3.sh [course] [task3_goal flags...]
#
# task3_goal arms, sets WP_RADIUS 0.3, goes GUIDED, sends the goal, flips to
# MANUAL when the tree asks for it (the pilot's SC, in the sim), and prints the
# referee's verdict. The run is also written to ~/.cache/crusader_sim/task3_last.log
# (not /tmp: WSL wipes /tmp every time the distro starts). The world's own log
# (lights, water, every report as RoboCommand saw it) is /tmp/task3_world.log in
# the container. Ctrl-C cancels the goal and waits for the tree to stop.
COURSE="${1:-task3}"
shift || true
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
LOG="$HOME/.cache/crusader_sim/task3_last.log"
mkdir -p "$(dirname "$LOG")"
docker exec -it "$CONTAINER" bash -c \
  "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash &&
   python3 -u -m crusader_sim.task3_goal --course $COURSE $*" 2>&1 \
  | tee "$LOG"
