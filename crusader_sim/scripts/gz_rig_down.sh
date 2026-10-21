#!/usr/bin/env bash
# gz_rig_down.sh — stop gz_rig_up.sh's processes. Runs in: the crsd-sim container.
#
# The team's nodes_down.sh (tools/sitl/rig_processes.txt) stops the boat nodes;
# this adds the sim-only ones. Patterns live in a FILE for the reason
# rig_processes.txt gives: a pkill -f pattern typed on a command line matches
# the shell running it and kills that shell, exit 143, silently.
HERE="$(cd "$(dirname "$0")" && pwd)"
SRC="${RX26_SRC:-/root/robotx_ws/src/rx26_asv}"
bash "$SRC/tools/sitl/nodes_down.sh" >/dev/null 2>&1
kill_listed() {
  while read -r pat; do
    [ -z "$pat" ] || [ "${pat:0:1}" = "#" ] && continue
    pkill -f -- "$pat" 2>/dev/null
  done < "$HERE/gz_rig_processes.txt"
}
kill_listed
sleep 1
# second pass: nav.launch.py respawns planner_server, so anything that came back
# between the first pass and the launch going down is caught here
kill_listed
sleep 1
echo "gz rig nodes stopped."
