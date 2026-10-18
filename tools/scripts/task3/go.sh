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
# wp: put the autopilot's MISSION WAYPOINT in the goal as the approach point (the
#   first waypoint uploaded from QGC's Plan view; mission_wp.py reads it back).
#   task3_cannon_slip.xml started in GUIDED drives there, then switches to
#   MANUAL itself (RequestMode). With wp the slip tree runs from GUIDED, and the
#   dock need not be in view yet.
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

TIMEOUT_S=600; HERE=0; DETACH=0; TIER=0; SHADOW=0; WP=0
for a in "$@"; do
  case "$a" in
    --here) HERE=1 ;;
    wp|--wp) WP=1 ;;
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
# A parameter read can fail for a few seconds after a node starts or the boat
# boots ("Node not found" while discovery settles: 2026-10-11). Retry, and treat
# a value never read as UNKNOWN - never as false.
nparam() {
  local v="" i
  for i in 1 2 3 4; do
    v=$(timeout 6 ros2 param get "$1" "$2" 2>/dev/null | awk '/value is:/{print $NF}')
    [ -n "$v" ] && break
    sleep 1
  done
  echo "$v"
}
param() { nparam /bt_runner_node "$1"; }
SP=$(param publish_setpoints); PUMP=$(param fire_pump)
TF=$(param tree_file)
[ -n "$SP" ] && [ -n "$PUMP" ] && [ -n "$TF" ] || fail "could not read bt_runner_node's parameters
       (publish_setpoints '${SP}', fire_pump '${PUMP}', tree_file '${TF}'): it may still be
       starting, or ROS discovery is slow after a boot. Run go.sh again in a few seconds"
TREE=$(basename "$TF")
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
       (or bt_runner_node publish_setpoints: true in crusader_params.yaml and restart
       task3.launch.py); or 'go.sh ... shadow' on purpose"
fi

# 2. the flight mode this tree runs in
FCU_MODE=""
for i in 1 2 3; do
  FCU_MODE=$(timeout 5 ros2 topic echo --once /crsd/fcu_status 2>/dev/null | awk '/^mode:/{print $2}')
  [ -n "$FCU_MODE" ] && break
done
case "$TREE" in
  task3_cannon_slip*)
    if [ "$WP" = 1 ]; then
      [ "$FCU_MODE" = "GUIDED" ] || fail "flight mode is '${FCU_MODE:-unknown}'; with wp this tree
       drives to the waypoint in GUIDED first (GUIDED from QGC or the ground station; tx guided in the sim)"
    else
      [ "$FCU_MODE" = "MANUAL" ] || fail "flight mode is '${FCU_MODE:-unknown}'; this tree needs MANUAL (SC),
       or GUIDED with a waypoint: t3 go wp"
    fi
    ;;
  task3_fire_manual*|task3_part2_*|task3_cannon_hold*|task3_cannon_fire*)
    [ "$FCU_MODE" = "MANUAL" ] || fail "flight mode is '${FCU_MODE:-unknown}'; this tree strafes on the sticks and needs MANUAL (SC)"
    ;;
  *)
    ALLOWED=""
    for i in 1 2 3; do
      ALLOWED=$(timeout 6 ros2 param get /bt_runner_node autonomous_modes 2>/dev/null | tr -d "[],'" | sed 's/.*://')
      [ -n "$ALLOWED" ] && break
      sleep 1
    done
    echo " $ALLOWED " | grep -q " ${FCU_MODE:-none} " \
      || fail "flight mode is '${FCU_MODE:-unknown}'; this tree runs in one of:$ALLOWED (SC to GUIDED)"
    ;;
esac
say "flight mode: $FCU_MODE"

# 2b. what a MANUAL fire tree reads, so it does not abort on its first tick
case "$TREE" in
  task3_fire_manual*|task3_part2_*|task3_cannon_hold*|task3_cannon_fire*|task3_cannon_slip*)
    n=$(count "[c]rusader_perception/wall_range_node")
    [ "$n" -ge 1 ] || fail "wall_range_node is not running (the LiDAR distance to the wall):
       task3.launch.py starts it; by hand, docs/T3_running.md"
    [ "$n" -eq 1 ] || fail "$n wall_range_nodes are running: pkill -f crusader_perception/wall_range_node, then start ONE"
    n=$(count "[d]ock_view.py")
    # the Gazebo sim's camera is task3_world (CRSD_SIM=1: crusader_sim/scripts/sim_aliases.sh)
    [ "${CRSD_SIM:-0}" = 1 ] && n=$(count "[c]rusader_sim/task3_world")
    [ "$n" -ge 1 ] || fail "dock_view is not running (the camera): ground station Nodes tab, or task3.launch.py"
    [ "$n" -eq 1 ] || fail "$n dock_views are running and only one can have the camera: stop the extra"
    W=""; for i in 1 2 3; do W=$(timeout 4 ros2 topic echo --once /crsd/wall_range 2>/dev/null | awk '/^valid:/{print $2}'); [ "$W" = "true" ] && break; done
    if [ "$W" = "true" ]; then
      say "inputs: one wall_range_node (wall seen), one dock_view"
    else
      case "$TREE" in
        task3_cannon_hold*|task3_cannon_fire*|task3_cannon_slip*)
          # these range on the camera's face until the LiDAR has the wall
          B=0; for i in 1 2 3; do B=$(timeout 4 ros2 topic echo --once /dock/observations 2>/dev/null | grep -c "bay_index"); [ "${B:-0}" -gt 0 ] && break; done
          if [ "${B:-0}" -gt 0 ]; then
            say "inputs: the LiDAR has no wall yet - the camera brings the boat in until it does"
          elif [ "$WP" = 1 ]; then
            say "inputs: no dock in view yet - fine: the waypoint leg comes first"
          else
            fail "neither the LiDAR (no wall) nor the camera (no bay) sees the dock:
       point the bow at the bay"
          fi
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
  task3_cannon_slip*)
    # the dock is on the LiDAR's slip: it must be running (it need not see the
    # slip yet: the camera brings the boat to its mouth first)
    n=$(count "[c]rusader_perception/dock_slot_node")
    [ "$n" -ge 1 ] || fail "dock_slot_node is not running (the LiDAR's slip): start the tree with
       task3.launch.py tree:=cannon_slip"
    [ "$n" -eq 1 ] || fail "$n dock_slot_nodes are running: pkill -f crusader_perception/dock_slot_node, then start ONE"
    S=$(timeout 4 ros2 topic echo --once /crsd/dock_slot 2>/dev/null | awk '/^valid:/{v=$2} /^why:/{$1=""; w=$0} END{print v "|" w}')
    say "slip: dock_slot_node running; the LiDAR sees the slip now: ${S%%|*} ${S#*|}"
    ;;
esac
case "$TREE" in
  task3_cannon_hold*|task3_cannon_fire*|task3_cannon_slip*)
    n=$(count "[c]rusader_fcu/cannon_aim_node")
    [ "$n" -ge 1 ] || fail "cannon_aim_node is not running: cal node (or start with task3.launch.py)"
    [ "$n" -eq 1 ] || fail "$n cannon_aim_nodes are running (cal node AND task3.launch.py?): they fight
       over the nozzle, and either may be the dry one. pkill -f crusader_fcu/cannon_aim_node,
       then start ONE: the launch (its fire_pump is crusader_params.yaml's), or cal node (always dry)"
    # the WATER is cannon_aim_node's own fire_pump, not the runner's
    CP=$(nparam /cannon_aim_node fire_pump)
    if [ "$CP" = "True" ]; then
      say "water: cannon_aim_node FIRES THE PUMP (SE off; SB e-stop and SE still stop it)"
    else
      say "water: cannon_aim_node is DRY (fire_pump ${CP:-unknown}): no water from the tree.
     For water: cannon_aim_node fire_pump: true in crusader_params.yaml, then Ctrl-C the
     launch and start it again (cal node is always dry)"
    fi
    ;;
esac
case "$TREE" in
  task3_cannon_fire*|task3_cannon_slip*)
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
if [ "$WP" = 1 ]; then
  [ "$HERE" = 0 ] || fail "wp and --here both set the approach point: pick one"
  W=$(python3 "$SCRIPT_DIR/mission_wp.py" 2>/tmp/mission_wp.err)     || fail "wp: $(cat /tmp/mission_wp.err)"
  LAT=${W% *}; LON=${W#* }
  GOAL="$GOAL, approach_latitude: $LAT, approach_longitude: $LON"
  say "approach point: the mission waypoint ($LAT, $LON)"
fi
say "sending the SafePassage goal (tier ${TIER}, timeout ${TIMEOUT_S}s)."
if [ "$WP" = 1 ]; then
  say "at the waypoint the TREE switches to MANUAL itself (leave the transmitter as it is)."
  say "to take the boat back: move SC (any position change takes the mode back),"
else
  say "to take the boat back: SC to MANUAL (GUIDED trees) or out of MANUAL (fire trees)"
fi
say "ends the run; SD up (drop latch); SB (e-stop). Ctrl+C here does NOT stop the tree."
# The runner's reply to a goal can be LOST on a loaded machine ("Failed to send
# goal response ... (timeout)" in its log). The runner then never had the goal,
# and send_goal would wait for that reply for ever. So: no reply in 15 s, send
# it again, up to 3 times. (The sim, 2026-10-17.)
answered() { grep -q -E "Goal (accepted|was rejected)" "$GOLOG"; }
for try in 1 2 3; do
  nohup ros2 action send_goal --feedback /crsd/safe_passage crusader_msgs/action/SafePassage \
    "{$GOAL}" > "$GOLOG" 2>&1 < /dev/null &
  PID=$!
  for _ in $(seq 1 30); do answered && break; sleep 0.5; done
  answered && break
  kill "$PID" 2>/dev/null
  [ "$try" -lt 3 ] && say "no reply to the goal in 15 s (lost on the way back?): sending it again"
done
answered || fail "the runner never replied to the goal (3 tries): is it overloaded? t3 log"
grep -E "Goal (accepted|was rejected)" "$GOLOG"
grep -q "Goal was rejected" "$GOLOG" && fail "the runner REJECTED the goal: a run is already going (t3 steps), or see t3 log"
if [ "$DETACH" = 1 ]; then
  say "detached: feedback in $GOLOG, tree log in $LOG (watch.sh)"
else
  tail -n +1 -f --pid="$PID" "$GOLOG"
fi
