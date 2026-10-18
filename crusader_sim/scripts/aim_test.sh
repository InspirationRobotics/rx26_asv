#!/usr/bin/env bash
# aim_test.sh lr|ul [--quick] [--gui] [--tol M] - the cannon's aim test, on its own.
# Runs in: WSL2 Ubuntu-22.04; `sim aimtest lr` is the same.
#
#   1. brings up task3_aim_lr / task3_aim_ul WHOLE (gz_sim_up.sh without --boat: the
#      cannon's node, the LiDAR's, the referee; no goal is sent, so no tree drives)
#   2. starts crusader_sim.aim_pin here (Gazebo's Python is in WSL, not the container),
#      which holds the boat wherever the test asks
#   3. runs crusader_sim.aim_test in the container: every pose of the grid, a shot
#      at the window, the referee's miss from its middle; the table, PASS/MISS
#
# Exit 0 only when every pose is within --tol (0.05 m) of the window's middle.
# sim.sh runs a CR-stripped copy from /tmp, and says where the real one is
HERE="${RX26_SIM_SCRIPTS:-$(cd "$(dirname "$0")" && pwd)}"
WIN_SRC="$(cd "$HERE/../.." && pwd)"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
W="${1:-lr}"
[ $# -gt 0 ] && shift
case "$W" in lr|ul) ;; *) echo "aim_test.sh lr|ul [--quick] [--gui] [--tol M]" >&2; exit 2 ;; esac
gui=(--no-gui)
extra=()
for a in "$@"; do
  case "$a" in --gui) gui=() ;; *) extra+=("$a") ;; esac
done
export GZ_PARTITION=crusader_sim
export PYTHONPATH="$WIN_SRC/crusader_sim${PYTHONPATH:+:$PYTHONPATH}"

tr -d '\r' < "$HERE/gz_sim_down.sh" > /tmp/gz_sim_down.sh
bash /tmp/gz_sim_down.sh > /dev/null 2>&1
tr -d '\r' < "$HERE/gz_sim_up.sh" > /tmp/gz_sim_up.sh
echo "== bringing up task3_aim_$W (about 2 minutes)"
RX26_WIN_SRC="$WIN_SRC" bash /tmp/gz_sim_up.sh "task3_aim_$W" "${gui[@]}" > /tmp/aim_up.log 2>&1 \
  || { tail -20 /tmp/aim_up.log; echo "*** bring-up failed (/tmp/aim_up.log)"; exit 1; }

rm -f "$HOME/robotx_ws/.aimtest/pose.json" "$HOME/robotx_ws/.aimtest/pinned.json"
python3 -u -m crusader_sim.aim_pin > /tmp/aim_pin.log 2>&1 &
PIN=$!
trap 'kill "$PIN" 2>/dev/null' EXIT

docker exec -i "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && \
  source /root/robotx_ws/install/setup.bash && \
  python3 -u -m crusader_sim.aim_test --window $W ${extra[*]}"
