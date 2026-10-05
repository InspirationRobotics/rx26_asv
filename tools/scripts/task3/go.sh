#!/usr/bin/env bash
# go.sh [TIMEOUT_S] [--here] [tier=N] [-d] - the operator's deliberate step: check,
# reset the autonomy-drop latch, start the run (the SafePassage goal). Runs
# INSIDE the asv container, after tree.sh (or task3.launch.py) is up:
#
#   docker exec -it asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/go.sh
#   docker exec -it asv .../go.sh 86400              # no practical time limit
#   docker exec -it asv .../go.sh 600 --here         # approach point = the boat
#   docker exec asv .../go.sh 600 --here -d          # detached: feedback to $GOLOG
#   docker exec -it asv .../go.sh 600 tier=2         # Disruptive: the request too
#
# TIMEOUT_S: the goal's timeout (default 600). Inside the tree, the fire trees'
#   AwaitStrafeSolution has its own per-attempt timeout (60 s x 5 in
#   task3_fire_manual.xml; none in task3_fire_manual_1p6.xml).
# --here: put the boat's CURRENT position in the goal as the approach point.
#   The Task 3 mission trees need one: with no bay seen yet and no approach
#   point the survey has nowhere to look from and fails at once. Start with the
#   boat facing the dock - with nothing seen, the first look is from here and
#   the next ones are a 4 m ring search around it.
# tier=N: the goal's tier, 0 Core (default), 1 Advanced, 2 Disruptive. The Task 3
#   trees read the resource request (TierAtLeast) only at 1 or 2. --tier=N works too.
# -d: detach (the goal keeps running after this returns); otherwise this stays
#   attached and prints feedback. Ctrl+C here does NOT stop the tree.
#
# WHY THIS IS NOT IN THE LAUNCH FILE. The drop latch trips on SD (ch9) up, or on
# RC data going stale for over a second, and stays tripped until a person resets
# it - the reset is that person saying "I have the sticks". A launch that reset
# it on its own would defeat the latch. So the launch starts the nodes, and
# this script is the act.
#
# It refuses (exit 1) when: bt_runner_node is not up; the flight mode is wrong
# for the tree (the MANUAL fire trees need MANUAL; the GUIDED trees need one of
# the node's autonomous_modes); or the bridge refuses the reset (SD up, RC
# stale). It never changes publish_setpoints or fire_pump - tree.sh does.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
set -u

TIMEOUT_S=600; HERE=0; DETACH=0; TIER=0
for a in "$@"; do
  case "$a" in
    --here) HERE=1 ;;
    -d) DETACH=1 ;;
    tier=[012]|--tier=[012]) TIER="${a##*=}" ;;
    tier=*|--tier=*) echo "go.sh: tier must be 0, 1 or 2 (got '${a##*=}')" >&2; exit 1 ;;
    ''|*[!0-9.]*) echo "go.sh: unknown argument '$a'" >&2; exit 1 ;;
    *) TIMEOUT_S="$a" ;;
  esac
done

say()  { echo "[go] $*"; }
fail() { echo "[go] REFUSED: $*" >&2; exit 1; }

# 1. the tree is up, and what it may do
pgrep -f "[c]rusader_bt/bt_runner_node" >/dev/null \
  || fail "bt_runner_node is not running - start it with tree.sh (or task3.launch.py)"
param() { ros2 param get /bt_runner_node "$1" 2>/dev/null | awk '{print $NF}'; }
SP=$(param publish_setpoints); PUMP=$(param fire_pump)
TREE=$(basename "$(param tree_file)")
case "$SP/$PUMP" in
  False/False) MODE="SHADOW - logs only, cannot move the boat or fire" ;;
  True/False)  MODE="TAKES THE BOAT (sticks or GUIDED setpoints); pump DRY" ;;
  True/True)   MODE="TAKES THE BOAT AND FIRES THE PUMP" ;;
  *)           MODE="publish_setpoints=$SP fire_pump=$PUMP" ;;
esac
say "tree:  $TREE"
say "mode:  $MODE"

# 2. the flight mode this tree runs in
FCU_MODE=$(timeout 5 ros2 topic echo --once /crsd/fcu_status 2>/dev/null | awk '/^mode:/{print $2}')
case "$TREE" in
  task3_fire_manual*|task3_part2_*)
    [ "$FCU_MODE" = "MANUAL" ] || fail "flight mode is '${FCU_MODE:-unknown}'; this tree strafes on the sticks and needs MANUAL (SC)"
    ;;
  *)
    ALLOWED=$(ros2 param get /bt_runner_node autonomous_modes 2>/dev/null | tr -d "[],'" | sed 's/.*://')
    echo " $ALLOWED " | grep -q " ${FCU_MODE:-none} " \
      || fail "flight mode is '${FCU_MODE:-unknown}'; this tree runs in one of:$ALLOWED (SC to GUIDED)"
    ;;
esac
say "flight mode: $FCU_MODE"

# 3. the drop latch: show it, reset it if tripped
DROP=$(timeout 5 ros2 topic echo --once --qos-durability transient_local \
         --qos-reliability reliable /crsd/autonomy_drop 2>/dev/null | awk '/^data:/{print $2}')
if [ "$DROP" = "true" ]; then
  say "autonomy-drop latch is TRIPPED - resetting (SD must be DOWN, RC transmitter on)"
  OUT=$(ros2 service call /crsd/autonomy_drop_reset std_srvs/srv/Trigger 2>&1)
  echo "$OUT" | grep -q "success=True" \
    || fail "reset refused by telemetry_bridge: $(echo "$OUT" | grep -o "message='[^']*'")"
  say "latch reset: $(echo "$OUT" | grep -o "message='[^']*'")"
elif [ "$DROP" = "false" ]; then
  say "autonomy-drop latch is clear"
else
  fail "could not read /crsd/autonomy_drop - is telemetry_bridge running?"
fi

# 4. the goal
GOAL="tier: ${TIER}, timeout_s: ${TIMEOUT_S}"
if [ "$HERE" = 1 ]; then
  P=$(timeout 4 ros2 topic echo --once /crsd/pose 2>/dev/null)
  LAT=$(echo "$P" | awk '/^latitude:/{print $2}'); LON=$(echo "$P" | awk '/^longitude:/{print $2}')
  [ -n "$LAT" ] && [ -n "$LON" ] || fail "--here: no /crsd/pose to take the approach point from"
  GOAL="$GOAL, approach_latitude: $LAT, approach_longitude: $LON"
  say "approach point: here ($LAT, $LON)"
fi
say "sending the SafePassage goal (tier ${TIER}, timeout ${TIMEOUT_S}s)."
say "to take the boat back: SC to MANUAL (GUIDED trees) or out of MANUAL (fire trees)"
say "ends the run; SD up (drop latch); SB (e-stop). Ctrl+C here does NOT stop the tree."
if [ "$DETACH" = 1 ]; then
  nohup ros2 action send_goal --feedback /crsd/safe_passage crusader_msgs/action/SafePassage \
    "{$GOAL}" > "$GOLOG" 2>&1 < /dev/null &
  sleep 3
  grep -E "Goal (accepted|rejected)|Waiting" "$GOLOG"
  say "detached: feedback in $GOLOG, tree log in $LOG (watch.sh)"
else
  ros2 action send_goal --feedback /crsd/safe_passage crusader_msgs/action/SafePassage "{$GOAL}"
fi
