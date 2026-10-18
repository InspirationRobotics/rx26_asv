#!/usr/bin/env bash
# slip.sh - what the LiDAR makes of the slip, one line a second, for checking it
# against a tape measure (docs/T3_slip_test_plan.md, step 2). Starts
# dock_slot_node if none is running and stops it again on Ctrl-C. Moves nothing.
# Runs INSIDE the asv container:
#
#   docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/slip.sh
#
# On the Jetson, once:  alias slip='docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/slip.sh'
# then:                 slip
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

NODE_PID=""
if pgrep -f "[c]rusader_perception/dock_slot_node" >/dev/null; then
  echo "dock_slot_node is already running (the tree's launch?): reading that one"
else
  ros2 run crusader_perception dock_slot_node --ros-args --params-file "$PARAMS" \
    >"$LOGDIR/slip_node.log" 2>&1 &
  NODE_PID=$!
  echo "started dock_slot_node (log: ~/robotx_ws/t3tools/slip_node.log)"
fi
# TERM, not INT: a job started with & in a script ignores SIGINT
stop() {
  [ -n "$NODE_PID" ] || return 0
  pkill -TERM -P "$NODE_PID" 2>/dev/null
  kill -TERM "$NODE_PID" 2>/dev/null
  echo "stopped dock_slot_node"
}
trap stop EXIT
python3 "$SCRIPT_DIR/slip_watch.py"
