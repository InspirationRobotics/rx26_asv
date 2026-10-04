#!/usr/bin/env bash
# lake_rig_up.sh — everything LAKE MODE needs that core.launch.py does not already run.
#
#     Runs in: INSIDE the `asv` container on the Jetson (ROS 2 Humble, host network).
#     NOT on the laptop, NOT on the Jetson host, NOT in crsd-sim.
#
#   from the laptop:   plink ... crusader@192.168.100.109            (the Jetson host, over SSH)
#   on the host:       docker exec -it asv bash
#   in the container:  LAKE_DATUM=1.3000000,103.8500000 bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh
#   then, on the laptop, a browser at  http://192.168.100.109:8095
#
#   Add --check to see whether this container is READY (packages, imports, ports, core's nodes) and exit without
#   starting anything: `LAKE_DATUM=1.3,103.85 bash .../lake_rig_up.sh --check`. Do it at home, not on the dock.
#
# Environment (all optional but LAKE_DATUM):
#   LAKE_DATUM   REQUIRED "lat,lon": the map origin. nav_frames_node's datum, panel_feed's origin and the
#                panel's origin are all this one value; the field, SAVE AS COURSE and every boat layer are
#                metres east/north of it. Pick a point on the lake bank you can name again (a pier corner).
#   TREE         default task1_global.xml: the WHOLE-FIELD planner (Advanced and Disruptive tier). It drives GUIDED
#                setpoints itself and never calls Nav2 (crusader_bt/src/global_leaves.cpp:19), so it behaves the same
#                in every nav_mode. TREE=task1_disruptive.xml is the per-gate tree (needs NAV_MODE=shadow/on to avoid
#                anything). A bare name is crusader_bt's behavior_trees/; or a path.
#   NAV_MODE     off | shadow | on. NOT GIVEN: `off` with the global tree (the rig then needs NO Nav2 in this
#                container), `shadow` with a per-gate tree (the planner runs and is DRAWN, the tree drives the legacy
#                straight legs). `on` = the per-gate tree drives the planner's paths. With the global tree shadow/on
#                only start the nav stack and draw its planner; they do not change how the boat is driven.
#   PUBLISH      1|true | anything else   default false = STAND TEST: bt_runner plans and ticks but sends NO
#                setpoints (publish_setpoints:=false), so the boat cannot move on the tree's account.
#                PUBLISH=1 is what lets the tree steer, and the pilot must still arm + choose GUIDED.
#   LAKE_TUNING  a planner-tuning file (the sim's tuning-profile format: nested ROS params, only the keys that
#                change; see config/tuning_profiles/). bt_runner gets it as a 2nd --params-file, Nav2 as nav2_overlay:=
#                (only when the nav stack is started). The banner lists every key; a missing file, a key the boat's
#                own params file does not have, or a wrong type stops the rig BEFORE it starts anything.
#   POOL         1|true: the Saturday pool test. Camera only: NAV_MODE forced off (no Nav2, no STVL), target_tracker
#                started with use_lidar:=false, LAKE_TUNING defaults to config/tuning_profiles/tight_3to5m.yaml.
#                (A pool profile belongs beside it as pool_<name>.yaml once the pool is measured: not guessed.)
#   PANEL_PORT   default 8095            LAKE_LOGDIR  default ~/.cache/crusader_lake
#   LAKE_FEED_PORT  default 14556 (udp, loopback): panel_feed -> the panel. Change it ONLY to run beside another
#                panel that holds 14556 (the sim's Task 1 panel does); keep it in the boat's 1455x range.
#   LAKE_RXL     replace (default) | keep.  See "rxl_link_node" below.
#   LAKE_SRC     the checkout inside the container, default /root/robotx_ws/src/rx26_asv
#
# WHAT IT STARTS (only what core.launch.py does not run — it never starts a duplicate of a node that is up):
#   rxl_link_node   with `-p rxl_endpoint:=udpin:127.0.0.1:14555` ON THE COMMAND LINE (the YAML's
#                   /dev/crsd-rfd is never edited): the panel plays the UAV over LOOPBACK, no radio.
#   target_tracker  camera detections + pose -> /crsd/world_targets (POOL=1: with use_lidar:=false stated explicitly)
#   nav             ros2 launch crusader_nav nav.launch.py datum_source:=param (NOT with nav_mode off: the global
#                   tree's default, and POOL)
#   bt_view         the tree, http://<jetson>:8085
#   ground_station  :8090 ONLY if core is not already running one
#   panel_feed      the boat's layers (pose, FCU, tracks, hazards, costmap, path) -> the panel, udp 127.0.0.1:14556
#   bt_runner       tree_file $TREE, nav_mode, publish_setpoints, default_timeout_s 600, the LAKE_TUNING overlay
#   task1_panel --lake   the page, http://<jetson>:8095 (START picks the tier: Advanced or Disruptive)
# WHAT IT NEVER STARTS: telemetry_bridge, lidar_cluster_node, proximity_bridge, rc_watchdog, led nodes (core's),
# and the OAK-D owner. START oak_detector FROM THE GCS NODES TAB (http://<jetson>:8090): it owns the OAK-D, and
# the device admits one client; without it there are no camera tracks to click.
#
# rxl_link_node: core.launch.py runs one WITH RESPAWN on /dev/crsd-rfd (the RFD900). The panel needs it on
# loopback instead, so with LAKE_RXL=replace (default) this script stops the running one ONCE and starts its own.
# core.launch.py then respawns its serial one after 5 s: without the RFD900 on USB it exits at startup (harmless
# noise in `journalctl -u crsd-ros`); with the radio plugged in it runs beside ours and, hearing no UAV, publishes
# nothing. For a cleaner lake day unplug the RFD900. To put the radio back to normal, run lake_rig_down.sh: it
# stops only what this script started (pids in $LAKE_LOGDIR/pids) and never touches core's nodes.
# LAKE_RXL=keep leaves core's node alone and starts none (then the panel has no loopback peer: bench use only).
#
# Logs: $LAKE_LOGDIR/<name>.log (default ~/.cache/crusader_lake/; inside asv ~ is /root).
# Stop: bash lake_rig_down.sh
#
# SAFETY: nothing here arms, disarms, selects a mode or publishes an RC override. The RC SB switch is the only
# e-stop. PUBLISH defaults to false.
# ROS's setup.bash reads unset variables, so strict mode goes on AFTER it
source "${LAKE_ROS_SETUP:-/opt/ros/humble/setup.bash}"          # LAKE_WS / LAKE_ROS_SETUP: test hooks (test/test_lake_rig.py), never needed on the boat
source "${LAKE_WS:-/root/robotx_ws}/install/setup.bash"
set -uo pipefail

WS="${LAKE_WS:-/root/robotx_ws}"
SRC="${LAKE_SRC:-$WS/src/rx26_asv}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CFG=$WS/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
BT=$WS/install/crusader_bt/share/crusader_bt/behavior_trees
LAKE_LOGDIR="${LAKE_LOGDIR:-$HOME/.cache/crusader_lake}"
PANEL_PORT="${PANEL_PORT:-8095}"
LAKE_RXL="${LAKE_RXL:-replace}"
RXL_ENDPOINT="udpin:127.0.0.1:14555"
FEED_PORT="${LAKE_FEED_PORT:-14556}"
case "$FEED_PORT" in ''|*[!0-9]*) echo "*** LAKE_FEED_PORT must be a udp port number (got '$FEED_PORT')" >&2; exit 2 ;; esac
mkdir -p "$LAKE_LOGDIR"
PIDS="$LAKE_LOGDIR/pids"

# ---- inputs, checked before anything starts
case "${PUBLISH:-}" in 1|true|TRUE|True) PUB=true ;; *) PUB=false ;; esac
CHECK=0; [ "${1:-}" = "--check" ] && CHECK=1          # --check: say whether this container is READY, start nothing
case "$LAKE_RXL" in replace|keep) ;; *) echo "*** LAKE_RXL must be replace or keep" >&2; exit 2 ;; esac
if [ -z "${LAKE_DATUM:-}" ]; then
  echo "*** LAKE_DATUM is required: 'lat,lon' of the map origin, e.g. LAKE_DATUM=1.3000000,103.8500000" >&2; exit 2
fi
export PYTHONPATH="$SRC/crusader_sim${PYTHONPATH:+:$PYTHONPATH}"
export RX26_SRC="$SRC"
if ! python3 -c "import sys; from crusader_sim.lake_panel import parse_datum; parse_datum(sys.argv[1])" "$LAKE_DATUM" 2>/tmp/lake_datum.err; then
  echo "*** LAKE_DATUM '$LAKE_DATUM' is not a usable 'lat,lon' (or crusader_sim is not under $SRC):" >&2
  sed 's/^/    /' /tmp/lake_datum.err >&2; exit 2
fi
DLAT="${LAKE_DATUM%%,*}"; DLON="${LAKE_DATUM##*,}"
# which tree, which nav_mode and why, POOL, the LAKE_TUNING file: decided in python (lake_rig_plan.py, offline-tested).
# A bad input (NAV_MODE, a missing tuning file) is refused here, before anything starts; it says why on stderr.
PLAN="$(python3 -m crusader_sim.lake_rig_plan decide)" || exit 2
eval "$PLAN"
TREE="$PLAN_TREE"; NAV_MODE="$PLAN_NAV_MODE"; POOL="$PLAN_POOL"; TUNING="$PLAN_TUNING"; GLOBAL="$PLAN_GLOBAL"
case "$TREE" in */*) ;; *) TREE="$BT/$TREE" ;; esac
TREE_NAME="$(basename "$TREE")"
NAV2_CFG=$WS/install/crusader_nav/share/crusader_nav/config/nav2_params.yaml
echo "=== plan ==="
printf '%s\n' "$PLAN_BANNER" | sed 's/^/  /'
[ -f "$CFG" ] || { echo "*** no $CFG: is this the asv container, with the workspace built?" >&2; exit 2; }
[ -f "$TREE" ] || { echo "*** no tree file $TREE (crusader_bt built in this workspace?)" >&2; exit 2; }
for t in pgrep setsid; do command -v $t >/dev/null || { echo "*** $t is not in this container" >&2; exit 2; }; done

# ---- the boat's own stack must be up: this script adds to it, it does not start it
echo "=== preflight ==="
# Every ros2 introspection here is --no-daemon: a wedged ros2 daemon ("!rclpy.ok()") makes `ros2 node list` fail
# and would read as "nothing is running". A process check does not depend on it.
if pgrep -f 'lib/crusader_fcu/telemetry_bridge|crusader_fcu.*telemetry_bridge' >/dev/null; then echo "  telemetry_bridge up (core.launch.py)"
else echo "*** telemetry_bridge is not running: core.launch.py (systemd crsd-ros) must be up first. On the HOST: systemctl status crsd-ros" >&2; exit 2; fi
fcu="$(timeout 8 ros2 topic echo /crsd/fcu_status --once --no-daemon 2>/dev/null | grep -E '^mode|^armed' | tr '
' ' ')"
if [ -n "$fcu" ]; then echo "  autopilot: $fcu"
else echo "  *** no /crsd/fcu_status within 8 s: telemetry_bridge is up but silent (MAVProxy, the Pixhawk?). START will stay disabled"; fi
need=""
for p in crusader_link crusader_bt crusader_world_model crusader_nav crusader_nav_layers nav2_planner crusader_groundstation; do
  ros2 pkg prefix $p >/dev/null 2>&1 || need="$need $p"
done
if [ -n "$need" ]; then
  if echo "$need" | grep -qE 'crusader_nav|nav2_planner'; then
    if [ "$NAV_MODE" = off ]; then
      echo "  nav packages not installed here:$(echo "$need" | grep -oE 'crusader_nav_layers|crusader_nav|nav2_planner' | tr '\n' ' ') -- not needed with nav_mode off"
    else
      echo "  *** AVOIDANCE OFF: not built / not installed here:$need  (rebuild on the HOST: tools/scripts/rebuild.sh). NAV_MODE=$NAV_MODE becomes off"
      NAV_MODE=off
    fi
    need="$(echo "$need" | sed -E 's/ (crusader_nav_layers|crusader_nav|nav2_planner)//g')"
  fi
  [ -z "${need// /}" ] || { echo "*** missing packages:$need  (rebuild on the HOST: tools/scripts/rebuild.sh)" >&2; exit 2; }
fi
if [ "$POOL" = 1 ] && pgrep -f 'crusader_world_model.*target_tracker|target_tracker' >/dev/null; then
  # POOL promises camera only: a tracker somebody else started with use_lidar true would put LiDAR tracks into the plan
  ul="$(timeout 10 ros2 param get /target_tracker use_lidar --no-daemon 2>/dev/null | tr -d '\n')"
  case "$ul" in
    *alse*) echo "  POOL: the running target_tracker already has use_lidar false" ;;
    *) echo "*** POOL=1: a target_tracker is already running and its use_lidar is not false (${ul:-no answer}): stop it (GCS Nodes tab) and run this again" >&2
       [ "$CHECK" = 1 ] || exit 3 ;;
  esac
fi
if pgrep -f 'crusader_perception.*oak_detector|oak_detector' >/dev/null; then echo "  oak_detector running"
else echo "  *** oak_detector is NOT running: no camera tracks until you start it from the GCS Nodes tab (http://<jetson>:8090)"; fi
if python3 - <<'PY' 2>/tmp/lake_imports.err
import yaml, pymavlink                                              # the panel's radio and its course files
from crusader_sim import lake_panel, lake_goal, panel_feed          # PYTHONPATH = $LAKE_SRC/crusader_sim
from crusader_msgs.action import SafePassage
from crusader_msgs.msg import FcuStatus, HazardArray, LatLonHead, TrackedTargetArray
PY
then echo "  python imports ok (yaml, pymavlink, crusader_sim, crusader_msgs)"
else echo "*** python imports failed:" >&2; sed 's/^/    /' /tmp/lake_imports.err >&2
  [ "$CHECK" = 1 ] || exit 2; fi
# the planner-tuning overlay, checked against the boat's own params files for the nav_mode that will REALLY run (the
# preflight above may have turned nav off); writes $LAKE_LOGDIR/tuning_applied.yaml + rig.json (the page's rig line)
FIN=(python3 -m crusader_sim.lake_rig_plan finish --nav-mode "$NAV_MODE" --cfg "$CFG" --nav2-cfg "$NAV2_CFG" --out-dir "$LAKE_LOGDIR" --publish "$PUB")
if [ "$CHECK" = 1 ]; then "${FIN[@]}" --dry || exit 2; else "${FIN[@]}" || exit 2; fi
if [ "$CHECK" = 1 ]; then
  echo "  would start: tree $TREE_NAME, nav_mode $NAV_MODE, publish_setpoints $PUB$([ "$POOL" = 1 ] && echo ', POOL (camera only)')"
  for pr in 14555 $FEED_PORT; do ss -lun 2>/dev/null | grep -q ":$pr " && echo "  note: udp $pr is in use (a lake rig already up? lake_rig_up.sh restarts it)"; done
  echo "check done: nothing was started."; exit 0
fi

# ---- ours only: stop the previous lake rig (by recorded pid), never anything else
bash "$HERE/lake_rig_down.sh" --quiet
# core.launch.py's rxl_link_node (serial, respawning) is stopped ONCE, here, so the loopback one can have udp 14555
if [ "$LAKE_RXL" = replace ]; then
  for pid in $(pgrep -f 'rxl_link_node'); do
    if tr '\0' ' ' < /proc/$pid/cmdline 2>/dev/null | grep -q 'rxl_link_node'; then
      echo "  stopping the running rxl_link_node (pid $pid); core.launch.py respawns its serial one in 5 s (see this script's header)"
      kill "$pid" 2>/dev/null
    fi
  done
  sleep 1
fi
if python3 -c "
import socket, sys
for port in (14555, int(sys.argv[1])):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        s.bind(('127.0.0.1', port))
    except OSError as e:
        sys.exit('udp %d is taken (%s): something else owns it. 14555 is rxl_link_node, %s is panel_feed -> the panel (LAKE_FEED_PORT moves that one)' % (port, e, sys.argv[1]))
    finally:
        s.close()
" "$FEED_PORT" 2>&1; then :; else
  echo "  (an old process of ours may have escaped lake_rig_down: ps -ef | grep -E 'rxl_link|panel_feed')" >&2
  exit 2
fi

up() {   # up <name> <logfile> <command...>: its own process group; its pid (= pgid) recorded for lake_rig_down.sh
  local name=$1 log=$2; shift 2
  local pf="$LAKE_LOGDIR/$name.pid"
  rm -f "$pf"
  # the inner shell writes its OWN pid inside the new session, then becomes the command: right even if setsid forks
  setsid nohup bash -c 'echo $$ > "$0"; exec "$@"' "$pf" "$@" > "$log" 2>&1 &
  for _ in 1 2 3 4 5 6 7 8 9 10; do [ -s "$pf" ] && break; sleep 0.1; done
  echo "$name $(cat "$pf" 2>/dev/null)" >> "$PIDS"
  printf '  %-16s -> %s\n' "$name" "$log"
}
# a live process matching a pattern. pgrep -f falls back to the process NAME for a zombie (empty cmdline), and the
# container's init does not reap: a dead nav_frames_node read as "already running" on 2026-10-02. Zombies are skipped.
# The CONTENT of cmdline, not `[ -s ]`: /proc files always stat as size 0, so `-s` was never true
# and every check below read "not running" (2026-10-03: a second ground_station on the boat, which
# failed on :8090). An empty cmdline is a zombie or a kernel thread, which is not "running".
running() { local p; for p in $(pgrep -f -- "$1"); do
              if [ -n "$(tr -d '\0' < /proc/$p/cmdline 2>/dev/null)" ]; then echo "    (pid $p: $(tr '\0' ' ' < /proc/$p/cmdline | cut -c1-90))" >&2; return 0; fi
            done; return 1; }

# the overlay finish() kept for this run (none: the boat's own params). bt_runner reads it as a 2nd --params-file,
# Nav2 as nav2_overlay:= (the same file: each node reads only its own section)
APPLIED="$LAKE_LOGDIR/tuning_applied.yaml"
BT_TUNE=(); NAV_TUNE=()
if [ -n "$TUNING" ] && [ -s "$APPLIED" ]; then BT_TUNE=(--params-file "$APPLIED"); NAV_TUNE=("nav2_overlay:=$APPLIED"); fi
TT_ARGS=(); [ "$POOL" = 1 ] && TT_ARGS=(-p use_lidar:=false)
echo "=== lake rig: datum $LAKE_DATUM  tree $TREE_NAME  nav_mode $NAV_MODE  publish_setpoints $PUB$([ "$POOL" = 1 ] && echo '  POOL: camera-only') ==="
# rxl_link_node: loopback, by command-line override (the running one was stopped above, once)
if [ "$LAKE_RXL" = replace ]; then
  up rxl_link_node "$LAKE_LOGDIR/rxl.log" \
    ros2 run crusader_link rxl_link_node --ros-args --params-file "$CFG" -p rxl_endpoint:=$RXL_ENDPOINT
else
  echo "  rxl_link_node        -- not started (LAKE_RXL=keep): the panel has no loopback peer"
fi
if running 'crusader_world_model.*target_tracker|target_tracker'; then echo "  target_tracker       -- already running, left alone"
else up target_tracker "$LAKE_LOGDIR/tt.log" ros2 run crusader_world_model target_tracker --ros-args --params-file "$CFG" "${TT_ARGS[@]}"; fi
if [ "$NAV_MODE" != off ]; then
  if running 'nav_frames_node'; then echo "  nav                  -- already running (its datum was NOT set by this rig: check /crsd/datum)"
  else up nav "$LAKE_LOGDIR/nav.log" ros2 launch crusader_nav nav.launch.py datum_source:=param datum_lat:=$DLAT datum_lon:=$DLON "${NAV_TUNE[@]}"; fi
else
  echo "  nav                  -- not started (nav_mode off$([ "$GLOBAL" = 1 ] || echo ': the legacy straight legs'))"
fi
if running 'tools/bt_view.py'; then echo "  bt_view              -- already running, left alone"
else up bt_view "$LAKE_LOGDIR/btview.log" python3 -u "$SRC/tools/bt_view.py"; fi
if running 'groundstation.*ground_station|ground_station'; then echo "  ground_station       -- running (core.launch.py)"
else up ground_station "$LAKE_LOGDIR/gcs.log" ros2 run crusader_groundstation ground_station --ros-args --params-file "$CFG"; fi
# the boat's layers for the panel. crusader_sim is not colcon-built on the boat: python3 -m from source
up panel_feed "$LAKE_LOGDIR/panel_feed.log" \
  python3 -u -m crusader_sim.panel_feed --ros-args -p origin:="$LAKE_DATUM" -p course:=lake -p panel_port:=$FEED_PORT
sleep 6
if running 'bt_runner_node'; then
  echo "*** a bt_runner_node is already running that this rig did not start (GCS Nodes tab?). Stop it first: its publish_setpoints / nav_mode are not this rig's." >&2
  exit 3
fi
up bt_runner "$LAKE_LOGDIR/bt.log" \
  ros2 run crusader_bt bt_runner_node --ros-args --params-file "$CFG" "${BT_TUNE[@]}" \
  -p tree_file:="$TREE" -p publish_setpoints:=$PUB -p default_timeout_s:=600.0 -p nav_mode:="$NAV_MODE"
sleep 6
up task1_panel "$LAKE_LOGDIR/panel.log" python3 -u -m crusader_sim.task1_panel --lake --datum "$LAKE_DATUM" --port "$PANEL_PORT" \
  --feed-port "$FEED_PORT" --rig-file "$LAKE_LOGDIR/rig.json"
# the overlay is only a claim until the node says so: read it back from the running bt_runner
if [ "${#BT_TUNE[@]}" -gt 0 ]; then python3 -m crusader_sim.lake_rig_plan verify "$APPLIED"; fi

# planner_server leaves 'activating' only once TF map -> base_footprint exists: a pose WITH A FINITE HEADING
# (GPS yaw from the moving-baseline RTK pair; the compass is disabled by design). Say so now, not 15 s into a leg.
if [ "$NAV_MODE" != off ]; then
  t0=$SECONDS; state=""
  while [ $((SECONDS - t0)) -lt 40 ]; do
    state="$(timeout 8 ros2 lifecycle get /planner_server --no-daemon 2>/dev/null | head -1)"
    case "$state" in active*) break ;; esac
    sleep 2
  done
  case "$state" in
    active*) echo "  planner_server: $state" ;;
    *) echo "  *** planner_server is not active after 40 s (${state:-no such node}): NAV_MODE=$NAV_MODE legs will hold and FAIL at 15 s. Does the autopilot report a valid heading (RTK GPS yaw)? See $LAKE_LOGDIR/nav.log" ;;
  esac
fi

echo
echo "=== up ==="
timeout 10 ros2 node list --no-daemon 2>/dev/null | sort | sed 's/^/  /'
echo "  rxl_link_node processes: $(pgrep -fc rxl_link_node) (ours has $RXL_ENDPOINT; core's serial one respawns, see the header)"
timeout 5 ros2 topic echo /crsd/fcu_status --once --no-daemon 2>/dev/null | grep -E '^mode|^armed' | tr '\n' ' ' | sed 's/^/  autopilot: /'; echo
echo
echo "  panel:        http://<jetson>:$PANEL_PORT      ground station: http://<jetson>:8090      tree: http://<jetson>:8085"
if [ "$PUB" = true ]; then
  echo "  PUBLISH=true: the tree WILL send setpoints once the pilot has armed and chosen GUIDED. The RC SB switch is the only e-stop."
else
  if [ "$GLOBAL" = 1 ]; then
    echo "  STAND TEST (PUBLISH=false): the tree plans and ticks but sends NO setpoints. For the water: PUBLISH=1 (the global tree drives the same in every nav_mode)"
  else
    echo "  STAND TEST (PUBLISH=false): the tree plans and ticks but sends NO setpoints. For the water: PUBLISH=1 (and NAV_MODE=on for avoidance)"
  fi
fi
echo "  tree $TREE_NAME, nav_mode $NAV_MODE <- $PLAN_NAV_WHY$([ "$POOL" = 1 ] && echo '. POOL: camera-only, LiDAR not used for planning')"
echo "  stop everything this started: bash $HERE/lake_rig_down.sh"
