#!/usr/bin/env bash
# tree.sh SETPOINTS PUMP [TREE] [-p name:=value ...] - (re)start bt_runner_node
# on a Task 3 tree, in the background, logging to $LOG. Runs INSIDE the asv
# container:
#
#   docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/tree.sh false false
#   docker exec asv .../tree.sh true false task3_fire_manual_1p6.xml \
#       -p strafe.kp_lat:=100.0 -p strafe.kd_lat:=200.0
#   docker exec asv .../tree.sh true false task3_approach_test.xml -p dock_bays:=1
#
# SETPOINTS (publish_setpoints, Gate G1: may the tree move the boat) and PUMP
# (fire_pump, Gate G7: may it squirt) are REQUIRED, true|false, so turning
# either on is always typed, never a default. TREE is a file in the installed
# behavior_trees/ (or an absolute path); default task3_fire_manual.xml.
# Anything after it goes to the node as ROS arguments - the live strafe.* gains,
# dock_bays, autonomous_modes - and becomes the node's starting value.
#
# Starting the tree moves nothing: a run starts only with go.sh.
# Restarting it ends any run in progress (the sticks are released).
set -u
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
set -u
SP="${1:?usage: tree.sh SETPOINTS PUMP [TREE] [-p name:=value ...]}"
PUMP="${2:?usage: tree.sh SETPOINTS PUMP [TREE] [-p name:=value ...]}"
shift 2
for v in "$SP" "$PUMP"; do
  case "$v" in true|false) ;; *) echo "tree.sh: SETPOINTS and PUMP are true|false, got '$v'" >&2; exit 1;; esac
done
TREE=task3_fire_manual.xml
if [ $# -gt 0 ] && [ "${1#-}" = "$1" ]; then TREE="$1"; shift; fi
case "$TREE" in /*) ;; *) TREE="$TREES/$TREE";; esac
[ -f "$TREE" ] || { echo "tree.sh: no tree file $TREE" >&2; exit 1; }

# [c]: the pattern must not match this script's own command line.
pkill -f "[c]rusader_bt/bt_runner_node" 2>/dev/null; sleep 1
: > "$LOG"
nohup ros2 run crusader_bt bt_runner_node --ros-args --params-file "$PARAMS" \
  -p tree_file:="$TREE" -p publish_setpoints:="$SP" -p fire_pump:="$PUMP" "$@" \
  >> "$LOG" 2>&1 < /dev/null &
sleep 5

if ! pgrep -f "[c]rusader_bt/bt_runner_node" >/dev/null; then
  echo "tree.sh: bt_runner_node did not stay up. Last lines of $LOG:" >&2
  tail -n 15 "$LOG" >&2
  exit 1
fi
get() { ros2 param get /bt_runner_node "$1" 2>/dev/null | awk '{print $NF}'; }
echo "tree: $(basename "$TREE")   publish_setpoints=$(get publish_setpoints)   fire_pump=$(get fire_pump)"
live=""
for p in kp_fwd kd_fwd ki_fwd kp_lat kd_lat ki_lat kp_yaw kd_yaw i_max_us window_median_s rate_window_s; do
  v=$(get "strafe.$p"); [ -n "$v" ] && [ "$v" != "-1.0" ] && live="$live $p=$v"
done
[ -n "$live" ] && echo "live strafe gains:$live"
grep -m3 -E "ready|CANNOT|TEST setting|ERROR" "$LOG" | sed 's/\x1b\[[0-9;]*m//g' | cut -c1-200
