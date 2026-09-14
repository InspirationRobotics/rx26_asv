#!/bin/bash
# nodes_down.sh -- stop every ROS node of the Task 1 sim rig.
#
# Runs INSIDE the crsd-sim container, and is the only safe place to do this:
# the kill patterns live in rig_processes.txt precisely so they never reach a
# command line, where pkill -f would match the shell running them.
#
#   (container)  bash src/rx26_asv/tools/sitl/nodes_down.sh
#   (WSL)        docker exec crsd-sim bash /root/robotx_ws/src/rx26_asv/tools/sitl/nodes_down.sh
#
# It does NOT touch SITL. SITL is a WSL-side process in a different PID
# namespace and this cannot see it, let alone kill it -- sim_down.sh does that.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LIST="$HERE/rig_processes.txt"

killed=0
while read -r pat || [ -n "$pat" ]; do
  case "$pat" in ''|'#'*) continue ;; esac
  if pgrep -f "$pat" > /dev/null 2>&1; then
    killed=$((killed + 1))
    printf '  stopping %s\n' "$pat"
  fi
  pkill -9 -f "$pat" 2>/dev/null
done < "$LIST"

sleep 1

# Report what did NOT die. A pattern still matching after SIGKILL means the
# process is in uninterruptible sleep on something -- worth seeing, because the
# next bring-up will collide with it rather than replace it.
survivors=0
while read -r pat || [ -n "$pat" ]; do
  case "$pat" in ''|'#'*) continue ;; esac
  pgrep -f "$pat" > /dev/null 2>&1 && { printf '  STILL UP: %s\n' "$pat"; survivors=$((survivors + 1)); }
done < "$LIST"

if [ "$survivors" -gt 0 ]; then
  echo "  $killed stopped, $survivors would not die"
  exit 1
fi
echo "  $killed process group(s) stopped, none left"
