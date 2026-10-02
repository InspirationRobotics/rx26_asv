#!/usr/bin/env bash
# lake_rig_down.sh — stop what lake_rig_up.sh started, and ONLY that.
#
#     Runs in: INSIDE the `asv` container on the Jetson (the same place as lake_rig_up.sh).
#
#     bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_down.sh [--quiet]
#
# It reads the pids lake_rig_up.sh recorded in $LAKE_LOGDIR/pids (default ~/.cache/crusader_lake/pids), one
# "<name> <pid>" per line, each the leader of its own process group (setsid), and stops each group: SIGINT
# first (ros2 launch and rclpy nodes shut down cleanly on it), SIGTERM after 4 s, SIGKILL after 8 s. It does NOT
# pkill by name, so it can never touch core.launch.py's nodes (telemetry_bridge, rc_watchdog, the LiDAR chain,
# the ground station that core runs) or an oak_detector you started from the GCS Nodes tab.
#
# Stopping the lake rig stops the task1 panel, so the UAV goes silent: a boat that is running a goal aborts
# ~15 s later on its own plan-freshness guard. The RC (SC to HOLD, SB) is still the stop; this script is not.
#
# After it, core.launch.py's own rxl_link_node (the respawning serial one) is whatever it was before: if it
# was stopped by lake_rig_up.sh it has respawned by now (5 s).
LAKE_LOGDIR="${LAKE_LOGDIR:-$HOME/.cache/crusader_lake}"
PIDS="$LAKE_LOGDIR/pids"
QUIET=0; [ "${1:-}" = "--quiet" ] && QUIET=1
say() { [ "$QUIET" = 1 ] || echo "$@"; }

[ -s "$PIDS" ] || { say "lake rig: nothing recorded in $PIDS"; exit 0; }
stopped=0
while read -r name pid; do
  [ -n "${pid:-}" ] || continue
  if ! kill -0 "$pid" 2>/dev/null; then say "  $name ($pid): already gone"; continue; fi
  # only a process group this rig made: its leader's pid == its pgid. A recycled pid is not stopped blindly.
  pgid="$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')"
  if [ "$pgid" != "$pid" ]; then say "  $name ($pid): pid reused by something else (pgid $pgid), left alone"; continue; fi
  kill -INT -- "-$pid" 2>/dev/null
  stopped=$((stopped + 1))
  say "  $name ($pid): SIGINT"
done < "$PIDS"
# wait up to 4 s, then TERM, then up to 4 s more, then KILL
for step in 1 2; do
  for _ in 1 2 3 4 5 6 7 8; do
    alive=0
    while read -r name pid; do
      [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null && [ "$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')" = "$pid" ] && alive=1
    done < "$PIDS"
    [ "$alive" = 0 ] && break
    sleep 0.5
  done
  [ "$alive" = 0 ] && break
  while read -r name pid; do
    [ -n "${pid:-}" ] && kill -0 "$pid" 2>/dev/null && [ "$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')" = "$pid" ] \
      && { [ "$step" = 1 ] && kill -TERM -- "-$pid" || kill -KILL -- "-$pid"; say "  $name ($pid): $([ "$step" = 1 ] && echo SIGTERM || echo SIGKILL)"; }
  done < "$PIDS"
done
: > "$PIDS"
say "lake rig stopped ($stopped process group(s))."
