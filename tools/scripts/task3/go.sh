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
# shadow: run a SHADOW tree (publish_setpoints false) on purpose. Without it a
#   shadow runner is refused: it cannot move the boat, and that was never what
#   the person at the transmitter meant (2026-10-09: a runner left by
#   task3.launch.py's defaults answered instead of tree.sh's).
#
# WHY THIS IS NOT IN THE LAUNCH FILE. The drop latch trips on SD (ch9) up, or on
# RC data going stale for over a second, and stays tripped until a person resets
# it - the reset is that person saying "I have the sticks". A launch that reset
# it on its own would defeat the latch. So the launch starts the nodes, and
# this script is the act.
#
# It refuses (exit 1) when: bt_runner_node is not up, or more than one is;
# it is a shadow runner (see `shadow`); the flight mode is wrong for the tree
# (the MANUAL fire trees need MANUAL; the GUIDED trees need one of the node's
# autonomous_modes); a MANUAL fire tree's inputs are missing (wall_range_node,
# dock_view, the LiDAR seeing a wall; cannon_aim_node for the cannon trees, and
# no `cal window` aiming it too); or the bridge refuses the reset (SD up, RC
# stale). Each says what to do. It never changes publish_setpoints or fire_pump
# - tree.sh (or task3.launch.py) does.
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"
set -u

TIMEOUT_S=600; HERE=0; DETACH=0; TIER=0; SHADOW=0
for a in "$@"; do
  case "$a" in
    --here) HERE=1 ;;
    shadow) SHADOW=1 ;;
    -d) DETACH=1 ;;
    tier=[012]|--tier=[012]) TIER="${a##*=}" ;;
    tier=*|--tier=*) echo "go.sh: tier must be 0, 1 or 2 (got '${a##*=}')" >&2; exit 1 ;;
    ''|*[!0-9.]*) echo "go.sh: unknown argument '$a'" >&2; exit 1 ;;
    *) TIMEOUT_S="$a" ;;
  esac
done

say()  { echo "[go] $*"; }
fail() { echo "[go] REFUSED: $*" >&2; exit 1; }

# how many processes match (the binaries; `ros2 run`'s own wrapper does not)
count() { pgrep -fc "$1" 2>/dev/null || true; }

# 1. the tree is up - ONE of it - and what it may do
N=$(count "[c]rusader_bt/bt_runner_node")
[ "$N" -ge 1 ] || fail "bt_runner_node is not running - start it with tree.sh (or task3.launch.py)"
[ "$N" -eq 1 ] || fail "$N tree runners are running, and the goal would go to either one. Stop them all
       (pkill -f crusader_bt/bt_runner_node, and Ctrl-C any task3.launch.py), then start ONE"
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
if [ "$SP" != "True" ] && [ "$SHADOW" != 1 ]; then
  fail "this runner is SHADOW (publish_setpoints $SP): it would not move the boat.
       Load the tree with the sticks:  tree.sh true false $TREE
       (or task3.launch.py ... publish_setpoints:=true); or 'go.sh ... shadow' on purpose"
fi

# 2. the flight mode this tree runs in
FCU_MODE=$(timeout 5 ros2 topic echo --once /crsd/fcu_status 2>/dev/null | awk '/^mode:/{print $2}')
case "$TREE" in
  task3_fire_manual*|task3_part2_*|task3_cannon_hold*|task3_cannon_fire*)
    [ "$FCU_MODE" = "MANUAL" ] || fail "flight mode is '${FCU_MODE:-unknown}'; this tree strafes on the sticks and needs MANUAL (SC)"
    ;;
  *)
    ALLOWED=$(ros2 param get /bt_runner_node autonomous_modes 2>/dev/null | tr -d "[],'" | sed 's/.*://')
    echo " $ALLOWED " | grep -q " ${FCU_MODE:-none} " \
      || fail "flight mode is '${FCU_MODE:-unknown}'; this tree runs in one of:$ALLOWED (SC to GUIDED)"
    ;;
esac
say "flight mode: $FCU_MODE"

# 2b. what a MANUAL fire tree reads, so it does not abort on its first tick
case "$TREE" in
  task3_fire_manual*|task3_part2_*|task3_cannon_hold*|task3_cannon_fire*)
    n=$(count "[c]rusader_perception/wall_range_node")
    [ "$n" -ge 1 ] || fail "wall_range_node is not running (the LiDAR distance to the wall):
       task3.launch.py starts it; by hand, docs/T3_running.md"
    [ "$n" -eq 1 ] || fail "$n wall_range_nodes are running: pkill -f crusader_perception/wall_range_node, then start ONE"
    n=$(count "[d]ock_view.py")
    [ "$n" -ge 1 ] || fail "dock_view is not running (the camera): ground station Nodes tab, or task3.launch.py"
    [ "$n" -eq 1 ] || fail "$n dock_views are running and only one can have the camera: stop the extra"
    W=$(timeout 3 ros2 topic echo --once /crsd/wall_range 2>/dev/null | awk '/^valid:/{print $2}')
    if [ "$W" = "true" ]; then
      say "inputs: one wall_range_node (wall seen), one dock_view"
    else
      case "$TREE" in
        task3_cannon_hold*|task3_cannon_fire*)
          # these range on the camera's face until the LiDAR has the wall
          B=$(timeout 3 ros2 topic echo --once /dock/observations 2>/dev/null | grep -c "bay_index")
          [ "${B:-0}" -gt 0 ] || fail "neither the LiDAR (no wall) nor the camera (no bay) sees the dock:
       point the bow at the bay"
          say "inputs: the LiDAR has no wall yet - the camera brings the boat in until it does"
          ;;
        *)
          fail "the LiDAR sees no wall (/crsd/wall_range valid: ${W:-no message}):
       start about 2 m out from the dock, roughly square to it"
          ;;
      esac
    fi
    ;;
esac
case "$TREE" in
  task3_cannon_hold*|task3_cannon_fire*)
    [ "$(count "[c]rusader_fcu/cannon_aim_node")" -ge 1 ] \
      || fail "cannon_aim_node is not running: cal node (or start with task3.launch.py)"
    ;;
esac
case "$TREE" in
  task3_cannon_fire*)
    [ "$(count "[c]annon_cal.py window")" -eq 0 ] \
      || fail "cal window is running: Ctrl-C it first - it and this tree would both aim the cannon"
    ;;
esac

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
