#!/usr/bin/env bash
# run.sh shadow|dry|live - Task 3 with the pan/tilt cannon, on the boat, in one
# word. Runs INSIDE the asv container:
#
#   docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/run.sh dry
#
#   shadow  the tree runs and logs what it would do; it cannot move the boat, and
#           the cannon never asks for water (publish_setpoints false, fire_pump false)
#   dry     the tree takes the boat (GUIDED approach, MANUAL dock on the LiDAR) and
#           the cannon AIMS, but every burst is logged DRY (fire_pump false)    G1
#   live    ...and the cannon fires the pump                                  G1 + G7
#
# What it does, in order:
#   1. dock_slot_node (the slip from the LiDAR) and cannon_aim_node (the aim and
#      the bursts), (re)started with this posture. Both log to $LOGDIR.
#   2. checks dock_view (the dock detector, which owns the OAK-D) is publishing
#      /dock/observations: start it from the ground station's Nodes tab first.
#   3. tree.sh on task3_cannon.xml with this posture.
#   4. go.sh with the approach point HERE (start with the boat facing the dock,
#      in GUIDED on SC), tier ${TIER:-2}. When the tree has lined up on the GREEN
#      bay it says "waiting for the pilot to select MANUAL (SC)": flip SC to
#      MANUAL - the tree never changes the mode itself.
#
# Taking the boat back is as for every Task 3 tree: SC out of the mode the tree
# is in ends the run; SD up trips the drop latch; SB is the e-stop.
# Environment: TIER=0|1|2 (default 2, Disruptive), TIMEOUT_S (default 900).
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
set -u

case "${1:-}" in
  shadow) SP=false; PUMP=false ;;
  dry)    SP=true;  PUMP=false ;;
  live)   SP=true;  PUMP=true ;;
  *) echo "usage: run.sh shadow|dry|live" >&2; exit 1 ;;
esac
say() { echo "[run] $*"; }

# 1. the LiDAR slip and the cannon. [d]/[c]: the pattern must not match this script.
pkill -f "[c]rusader_perception/dock_slot_node" 2>/dev/null
pkill -f "[c]rusader_fcu/cannon_aim_node" 2>/dev/null
sleep 1
nohup ros2 run crusader_perception dock_slot_node --ros-args --params-file "$PARAMS" \
  > "$LOGDIR/dock_slot.log" 2>&1 < /dev/null &
nohup ros2 run crusader_fcu cannon_aim_node --ros-args --params-file "$PARAMS" \
  -p fire_pump:="$PUMP" > "$LOGDIR/cannon_aim.log" 2>&1 < /dev/null &
sleep 4
for n in dock_slot_node cannon_aim_node; do
  pgrep -f "[/]$n" >/dev/null || { echo "[run] $n did not stay up:" >&2; tail -n 10 "$LOGDIR/${n%%_node}.log" >&2; exit 1; }
done
say "dock_slot_node and cannon_aim_node up (fire_pump=$PUMP); logs in $LOGDIR"

# 2. the dock detector
timeout 4 ros2 topic echo --once /dock/observations >/dev/null 2>&1 \
  || { echo "[run] REFUSED: nothing on /dock/observations - start dock_view (ground station, Nodes tab)" >&2; exit 1; }
say "dock_view is publishing"

# 3. the tree, 4. the run
"$SCRIPT_DIR/tree.sh" "$SP" "$PUMP" task3_cannon.xml || exit 1
say "when the tree says 'waiting for the pilot to select MANUAL': SC to MANUAL"
exec "$SCRIPT_DIR/go.sh" "${TIMEOUT_S:-900}" --here "tier=${TIER:-2}"
