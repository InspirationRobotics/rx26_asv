#!/usr/bin/env bash
# gz_sim_down.sh — stop the Gazebo sim. Runs in: WSL2 Ubuntu-22.04.
#
#     bash crusader_sim/scripts/gz_sim_down.sh [--keep-container]
#
# ROS rig (in the container) -> SITL + MAVProxy -> transmitter -> Gazebo.
# The container itself is stopped too unless --keep-container.
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
if docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true; then
  docker exec "$CONTAINER" bash -lc \
    "bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/gz_rig_down.sh" 2>/dev/null
  [ "${1:-}" = "--keep-container" ] || docker stop "$CONTAINER" >/dev/null
fi
bash "$WS_SRC/tools/sitl/start_sitl.sh" --stop >/dev/null 2>&1
pkill -f "crusader_sim.sim_transmitter" 2>/dev/null
pkill -f "gz sim" 2>/dev/null
sleep 1
echo "gz sim stopped."
