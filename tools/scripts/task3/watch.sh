#!/usr/bin/env bash
# watch.sh [SECONDS] [N] - wait SECONDS (default 0), then show the last N
# (default 20) lines of the tree log that say what it is doing: strafe ticks,
# line-up reasons, shots, survey legs, results. Changes nothing.
#
#   docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/watch.sh 10
# Live instead:
#   docker exec -it asv bash -c 'tail -f /root/robotx_ws/t3tools/bt.log | grep --line-buffered -E "strafe:|FIRE|RESULT"'
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
sleep "${1:-0}"
sed 's/\x1b\[[0-9;]*m//g' "$LOG" \
  | grep -E "strafe:|lining up|FIRE|HIT|RESULT|abort|FAIL|no firing|solution|dock:|survey|NavigateTo|REPORTED|decoded|strafe\.|GOAL" \
  | sed -E 's/^\[[A-Z]+\] \[[0-9.]+\] \[bt_runner_node\]: //' \
  | tail -n "${2:-20}"
