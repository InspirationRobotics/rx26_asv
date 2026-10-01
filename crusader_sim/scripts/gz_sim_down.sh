#!/usr/bin/env bash
# gz_sim_down.sh — stop the Gazebo sim. Runs in: WSL2 Ubuntu-22.04.
#
#     bash crusader_sim/scripts/gz_sim_down.sh [--keep-container] [--all]
#
# ROS rig (in the container) -> SITL + MAVProxy -> transmitter -> Gazebo.
# The container itself is stopped too unless --keep-container.
# --all also stops the Task 1 panel. Without it the panel survives: the panel
# itself runs this script (its "Stop sim" button, and gz_sim_up.sh's step 0).
KEEP=0; ALL=0
for a in "$@"; do
  case "$a" in
    --keep-container) KEEP=1 ;;
    --all) ALL=1 ;;
    *) echo "unknown flag $a" >&2; exit 2 ;;
  esac
done
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
# the panel first: it is what launches sims, so it must not start another
# while this one is being taken down
# only the real panel (:8095): a test instance on another port is left alone
[ "$ALL" = 1 ] && pkill -f 'crusader_sim\.task1_panel --port 8095' 2>/dev/null
if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  docker exec "$CONTAINER" bash -lc \
    "bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_rig_down.sh" 2>/dev/null
  [ "$KEEP" = 1 ] || docker stop "$CONTAINER" >/dev/null
fi
bash "$WS_SRC/tools/sitl/start_sitl.sh" --stop >/dev/null 2>&1
pkill -f "crusader_sim.sim_transmitter" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
# the server takes seconds to exit (still up 7 s after STOP SIM, 2026-09-30);
# a LAUNCH straight after must not find the old one still on the partition
for _ in $(seq 1 20); do pgrep -f "gz sim" >/dev/null || break; sleep 0.5; done
pkill -9 -f "gz sim" 2>/dev/null
if [ "$ALL" = 1 ]; then echo "gz sim and the Task 1 panel stopped."; else echo "gz sim stopped."; fi
