#!/usr/bin/env bash
# gz_nav_extra.sh — the avoidance sim tests that are not a plain Task 1 run, each scored on ONE line.
# Runs in: WSL2 Ubuntu-22.04. Spec: docs/nav2_avoidance_spec.md 10.1 (N2, N3) and 10.2 (S6, S9).
#
#     bash gz_nav_extra.sh s6 [--rmax 10|30] [--timeout-s S] [--tag NAME]
#     bash gz_nav_extra.sh s9 [--at-s N] [--hold-s 20] [--timeout-s S] [--tag NAME]
#     bash gz_nav_extra.sh n2 [--seconds 10] [--no-up] [--tag NAME]
#     bash gz_nav_extra.sh n3 [--in entry] [--out g1_grn] [--no-up] [--tag NAME]
#
#   s6  open_water_platform, TREE=nav_test_line.xml, NAV_MODE=on. First the spec's precondition (is
#       the platform in the LiDAR nav cloud? if not, water_margin 0.08 on /lidar_cluster_node, recorded),
#       then crusader_sim.nav_goal 40 m east with --watch plat (arrives, centre-to-surface >= 0.73 m).
#       Without --rmax it runs twice, lidar_cluster_node r_max 10 then 30.
#   s9  task1_core, NAV_MODE=on, the default tree. Into the first TRANSIT phase (--at-s N: N s after the
#       goal instead) SITL's SIM_GPS_HDG goes to 0 for --hold-s, then back to 1. Pass: DEGRADED within a
#       tick of the heading going NaN, no leg FAILED, the leg resumes, the mission completes.
#   n2  TF map -> base_footprint against frames_core.to_local(/crsd/pose) over --seconds: max <= 0.05 m.
#   n3  STVL decay: a buoy marked in view, then REMOVED from Gazebo, leaves the costmap within 5 s;
#       a buoy beyond the clearing frustum's 20 m persists >= 20 s and is gone by 35 s. Scored on a
#       STVL-ONLY twin of the planner (nav_checks twin-params, namespace /n3, the boat's own STVL values):
#       the real costmap also holds the BT's hazards, which bt_runner keeps publishing for the tracks it last
#       held when the tracker goes quiet, so a removed buoy stays lethal there whatever STVL does.
#
# Every run: gz_sim_up.sh (fresh world) -> the test -> logs kept -> gz_sim_down.sh --keep-container
# (never --all). Logs and summary.txt: ~/.cache/crusader_sim/runs/<tag>/. --no-up (n2, n3) uses a rig
# that is already up; it is still taken down at the end.
#
# Run it from a /tmp copy, as gz_nav_test.sh: its first step syncs the checkout, which rewrites changed
# scripts in place, and bash reads a script as it runs.
#     tr -d '\r' < /mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv/crusader_sim/scripts/gz_nav_extra.sh > /tmp/gz_nav_extra.sh && bash /tmp/gz_nav_extra.sh s6
set -uo pipefail

usage() { sed -n '5,8p' "$0" >&2; exit 2; }
[ $# -ge 1 ] || usage
CMD="$1"; shift
case "$CMD" in s6|s9|n2|n3) ;; *) usage ;; esac
RMAXES="10 30"; TMO=""; TAG=""; AT_S=""; HOLD_S=20; SECS=10; IN_EL=entry; OUT_EL=g1_grn; UP=1
while [ $# -gt 0 ]; do
  case "$1" in
    --rmax)      RMAXES="${2:?}"; shift ;;
    --timeout-s) TMO="${2:?}"; shift ;;
    --tag)       TAG="${2:?}"; shift ;;
    --at-s)      AT_S="${2:?}"; shift ;;
    --hold-s)    HOLD_S="${2:?}"; shift ;;
    --seconds)   SECS="${2:?}"; shift ;;
    --in)        IN_EL="${2:?}"; shift ;;
    --out)       OUT_EL="${2:?}"; shift ;;
    --no-up)     UP=0 ;;
    *)           echo "unknown option: $1" >&2; usage ;;
  esac
  shift
done

# ---- arguments, checked before anything is touched (a mistake here must not cost a running sim)
is_num() { [[ "$1" =~ ^[0-9]+(\.[0-9]+)?$ ]]; }
is_name() { [[ "$1" =~ ^[A-Za-z0-9_.-]+$ ]]; }
case "$CMD" in s6) COURSE=open_water_platform; TMO="${TMO:-300}" ;; *) COURSE=task1_core; TMO="${TMO:-400}" ;; esac
for v in $RMAXES; do is_num "$v" || { echo "--rmax must be a number (10 or 30), got '$v'" >&2; exit 2; }; done
for v in "$TMO" "$HOLD_S" "$SECS" ${AT_S:+"$AT_S"}; do is_num "$v" || { echo "not a number: '$v'" >&2; exit 2; }; done
is_name "$IN_EL" && is_name "$OUT_EL" && [ "$IN_EL" != "$OUT_EL" ] \
  || { echo "--in and --out must be two different element names" >&2; exit 2; }
[ -z "$TAG" ] || is_name "$TAG" || { echo "bad tag: $TAG" >&2; exit 2; }
[ "$UP" = 1 ] || case "$CMD" in n2|n3) ;; *) echo "--no-up is for n2 and n3" >&2; exit 2 ;; esac

WIN_SRC="${RX26_WIN_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv}"
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
SIM="$WS_SRC/crusader_sim"
CSRC="${RX26_CSRC:-/root/robotx_ws/src/rx26_asv/crusader_sim}"      # $SIM as the container sees it
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
GEN="${CRUSADER_SIM_GEN:-$HOME/.cache/crusader_sim}"
SITL_EP="${RX26_SITL_EP:-tcp:127.0.0.1:5762}"       # SITL's spare SERIAL1: see sitl_param_set.py
OUT="$GEN/runs/${TAG:-$(date +%Y%m%d_%H%M%S)_$CMD}"
mkdir -p "$OUT"

# Sync ONCE, before anything runs from the WSL copy (see gz_nav_test.sh). Its own temp name: the
# sync that gz_sim_up.sh and gz_nav_test.sh write to /tmp/gz_sync.sh may be running from it.
tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sync.sh" > /tmp/gz_nav_extra_sync.sh \
  && RX26_WIN_SRC="$WIN_SRC" RX26_WSL_SRC="$WS_SRC" bash /tmp/gz_nav_extra_sync.sh > "$OUT/sync.log" 2>&1 \
  || { echo "*** Windows -> WSL sync failed (see $OUT/sync.log)" >&2; exit 2; }
[ -f "$SIM/courses/$COURSE.yaml" ] || { echo "no course $SIM/courses/$COURSE.yaml" >&2; exit 2; }

# ---- helpers. WSL -> container unless a comment says otherwise.
ENVSRC="source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash"
say() { echo "$*" | tee -a "$OUT/summary.txt"; }
epoch() { date +%s.%N; }
# the crsd-sim command line for crusader_sim.<module> under `timeout <s>`: run from the synced source tree, so a
# module the last colcon build has not seen is found, and without writing .pyc into the shared workspace
pyline() { local t="$1"; shift; echo "$ENVSRC && PYTHONDONTWRITEBYTECODE=1 PYTHONPATH=$CSRC\${PYTHONPATH:+:\$PYTHONPATH} timeout $t python3 -u -m crusader_sim.$*"; }
# cpy <timeout s> <module> <args>: that, in the container, in the foreground
cpy() { docker exec "$CONTAINER" bash -c "$(pyline "$@")"; }
cx() { docker exec "$CONTAINER" bash -c "$ENVSRC && $*"; }
# hpy <module> <args>: a crusader_sim module on THIS host (WSL python3, no ROS): scorers and the SITL parameter
hpy() { PYTHONDONTWRITEBYTECODE=1 PYTHONPATH="$SIM${PYTHONPATH:+:$PYTHONPATH}" python3 -m "crusader_sim.$1" "${@:2}"; }
# setp <param> <value>: ros2 param set on /lidar_cluster_node; 0 only when the node confirms it
setp() { timeout 30 docker exec "$CONTAINER" bash -c "$ENVSRC && ros2 param set /lidar_cluster_node $1 $2" 2>&1 \
         | tee -a "$OUT/params.log" | grep 'Set parameter successful' >/dev/null; }
# first line of $2 matching regex $1, without the seconds prefix
pick() { grep -m1 -E "$1" "$2" 2>/dev/null | sed -E 's/^ *[0-9]+ //'; }
# the same, without its [tag] too
field() { pick "$1" "$2" | sed -E 's/^\[[a-z0-9]+\] //'; }
# the seconds prefix of that line
stamp() { grep -m1 -E "$1" "$2" 2>/dev/null | awk '{print $1}'; }
stamp_lines() { while IFS= read -r line; do printf '%4d %s\n' "$SECONDS" "$line"; done; }
# the container's logs next to the run's own
grab_logs() {
  local f
  for f in bt nav lidar_cluster; do
    docker cp "$CONTAINER:/tmp/$f.log" "$1.$f.log" >/dev/null 2>&1 || : > "$1.$f.log"
  done
}
# a stale crusader_bt build ignores nav_mode and runs legacy legs: that must not be scored as the mode asked for
mode_note() {
  local ran
  ran="$(grep -m1 -oE 'obstacle avoidance: nav_mode [a-z]+' "$1.bt.log" 2>/dev/null | awk '{print $NF}')"
  [ "$ran" = on ] || echo " | *** INVALID: bt_runner ran nav_mode '${ran:-none logged}', not 'on'"
}
# stop a probe in the container (the pattern has a [x] so the pkill's own shell does not match it)
stop_probe() { cx "pkill -f '$1'; true" >/dev/null 2>&1; }
sim_down() { bash "$SIM/scripts/gz_sim_down.sh" --keep-container > "$OUT/down.log" 2>&1; }
# fail_run <scored line>: say it and leave the sim down, for a run that cannot go on
fail_run() { say "$*"; sim_down; }

# bring_up <label> <log prefix> <course> <tree> [gz_sim_up flags]: a fresh sim with NAV_MODE=on and the nav
# stack active, else the failed line under <label> and 1. An empty tree is the default tree.
bring_up() {
  local label="$1" pre="$2" course="$3" tree="$4" why=""; shift 4
  if ! NAV_MODE=on TREE="$tree" bash "$SIM/scripts/gz_sim_up.sh" "$course" "$@" > "$pre.up.log" 2>&1; then
    why="gz_sim_up.sh failed (see $pre.up.log)"; tail -5 "$pre.up.log" | sed 's/^/    /'
  else
    grep -E "AVOIDANCE OFF|planner_server|nav_mode" "$pre.up.log" | sed 's/^/    /'
    grep -q 'planner_server: active' "$pre.up.log" || why="planner_server is not active after bring-up (see $pre.up.log)"
  fi
  [ -z "$why" ] || { fail_run "$label: FAIL | bring-up: $why"; return 1; }
}

# ---------------------------------------------------------------- S6
# One run at lidar_cluster_node r_max $1.
s6_run() {
  local rmax="$1" L="$OUT/s6_rmax$1" wm=0.15 pre rc rv score blk
  echo "== s6  rmax $rmax  course $COURSE  tree nav_test_line.xml"
  bring_up "s6 rmax $rmax" "$L" "$COURSE" nav_test_line.xml --no-gui --no-uav || return

  # THE PRECONDITION (spec 10.2 S6). The platform is 19 m out, so it is looked for at r_max 30 whatever this run
  # uses; r_max is set to the run's value afterwards.
  setp r_max 30.0 || { fail_run "s6 rmax $rmax: FAIL | could not set r_max 30 for the precondition"; return; }
  cpy 60 nav_checks s6-precond --course "$COURSE" --element plat > "$L.precond.log" 2>&1; rc=$?
  case $rc in
    0) pre="platform clustered at water_margin 0.15" ;;
    1) # the spec's remedy: water_margin 0.08, recorded
       if setp water_margin 0.08; then
         wm=0.08; sleep 3
         cpy 60 nav_checks s6-precond --course "$COURSE" --element plat >> "$L.precond.log" 2>&1
         case $? in
           0) pre="NOT clustered at water_margin 0.15, clustered after water_margin 0.08 (set, as the spec says)" ;;
           1) pre="NOT clustered even at water_margin 0.08 (19 m out; it may only show closer)" ;;
           *) pre="water_margin 0.08 set, precondition unreadable afterwards" ;;
         esac
       else pre="NOT clustered at 0.15 and water_margin 0.08 could not be set"; fi ;;
    *) pre="precondition unreadable (no nav cloud or pose: see $L.precond.log)" ;;
  esac
  echo "    precondition: $pre"

  rv="$(awk -v v="$rmax" 'BEGIN{printf "%.1f", v}')"
  setp r_max "$rv" || { fail_run "s6 rmax $rmax: FAIL | could not set r_max $rv"; return; }
  # what the r_max 30 look left in STVL decays by itself (voxel_decay 30 s); a shorter r_max must not start with it
  awk -v v="$rmax" 'BEGIN{exit !(v < 30)}' && { echo "    waiting 35 s for STVL to forget the r_max 30 look"; sleep 35; }

  SECONDS=0
  timeout $(( ${TMO%.*} + 90 )) docker exec "$CONTAINER" bash -c \
    "$(pyline $(( ${TMO%.*} + 60 )) nav_goal --course "$COURSE" --east-m 40 --north-m 0 --timeout-s "$TMO" --min-clear-m 0.73 --watch plat)" 2>&1 \
    | stamp_lines > "$L.nav_goal.log"
  grab_logs "$L"
  score="$(field '\[score\]' "$L.nav_goal.log")"
  blk="$(grep -c 'BLOCKED' "$L.bt.log" 2>/dev/null)"
  say "s6 rmax $rmax: ${score:-NO SCORE (see $L.nav_goal.log)} | $(field '\[clear\] plat' "$L.nav_goal.log") | $(field '\[arrive\]' "$L.nav_goal.log") | $(field '\[result\]' "$L.nav_goal.log") | precond: $pre | water_margin $wm | BLOCKED x${blk:-0}$(mode_note "$L")"
  sim_down
}

# ---------------------------------------------------------------- S9
# s9_abort <log prefix> <scored line>: the run could not be carried out
s9_abort() { stop_probe '[n]av_checks s9-watch'; grab_logs "$1"; fail_run "$2"; }

s9_run() {
  local L="$OUT/s9" tmo="${TMO%.*}" t_goal t_off="" t_on="" try out tpid wpid verdict deg
  echo "== s9  course $COURSE  default tree  SIM_GPS_HDG 0 for ${HOLD_S}s via $SITL_EP"
  bring_up "s9" "$L" "$COURSE" "" --no-gui || return

  # the recorder of heading validity and leg states first, so its history covers the whole mission
  docker exec "$CONTAINER" bash -c "$(pyline $((tmo + 90)) nav_checks s9-watch --seconds $((tmo + 60)))" > "$L.watch.log" 2>&1 &
  wpid=$!
  sleep 2
  SECONDS=0
  { timeout $((tmo + 90)) docker exec "$CONTAINER" bash -c \
      "$(pyline $((tmo + 60)) task1_goal --course "$COURSE" --timeout-s "$tmo")" 2>&1 | stamp_lines > "$L.task1.log"; } &
  tpid=$!

  # when: the first TRANSIT phase +5 s (the spec says mid-transit), or --at-s N after the goal was sent
  while ! grep -q '\[operator\] goal:' "$L.task1.log" 2>/dev/null; do
    kill -0 "$tpid" 2>/dev/null && [ "$SECONDS" -lt 150 ] || break; sleep 1
  done
  t_goal="$(stamp '\[operator\] goal:' "$L.task1.log")"
  if [ -n "$t_goal" ]; then
    if [ -n "$AT_S" ]; then
      while [ $(( SECONDS - t_goal )) -lt "${AT_S%.*}" ] && kill -0 "$tpid" 2>/dev/null; do sleep 1; done
    else
      while ! grep -q '\[tree\] TRANSIT' "$L.task1.log" && kill -0 "$tpid" 2>/dev/null \
            && [ $(( SECONDS - t_goal )) -lt "$tmo" ]; do sleep 1; done
      sleep 5
    fi
  fi
  if [ -z "$t_goal" ] || ! kill -0 "$tpid" 2>/dev/null; then
    s9_abort "$L" "s9: INVALID | the mission was not running at the injection point (see $L.task1.log)"; return
  fi

  out="$(hpy sitl_param_set SIM_GPS_HDG 0 --endpoint "$SITL_EP" 2>&1)"; echo "$out" | tee -a "$L.sitl.log"
  t_off="$(sed -nE 's/.* epoch=([0-9.]+).*/\1/p' <<< "$out" | head -1)"
  [ -n "$t_off" ] || { s9_abort "$L" "s9: INVALID | SIM_GPS_HDG 0 was not acknowledged by SITL on $SITL_EP: $out"; return; }
  sleep "${HOLD_S%.*}"
  for try in 1 2 3; do
    out="$(hpy sitl_param_set SIM_GPS_HDG 1 --endpoint "$SITL_EP" 2>&1)"; echo "$out" | tee -a "$L.sitl.log"
    t_on="$(sed -nE 's/.* epoch=([0-9.]+).*/\1/p' <<< "$out" | head -1)"
    [ -n "$t_on" ] && break; sleep 2
  done
  [ -n "$t_on" ] || { s9_abort "$L" "s9: FAIL | SIM_GPS_HDG was set to 0 and could NOT be set back to 1 on $SITL_EP: $out"; return; }

  wait "$tpid"                          # the tree's own timeout ends the mission at the latest
  grab_logs "$L"
  stop_probe '[n]av_checks s9-watch'; wait "$wpid" 2>/dev/null
  verdict="$(hpy nav_checks score-s9 --watch "$L.watch.log" --t-off "$t_off" --t-on "$t_on" \
               --bt "$L.bt.log" --task1 "$L.task1.log" 2>&1)"
  deg="$(grep -c 'leg DEGRADED' "$L.bt.log" 2>/dev/null)"
  say "s9: $verdict | bt.log 'leg DEGRADED' x${deg:-0} | $(pick '\[result\]' "$L.task1.log" | cut -c1-110)$(mode_note "$L")"
  sim_down
}

# ---------------------------------------------------------------- N2
n2_run() {
  local L="$OUT/n2" line
  echo "== n2  course $COURSE  TF against /crsd/pose for ${SECS}s"
  [ "$UP" = 0 ] || bring_up "n2" "$L" "$COURSE" "" --no-gui --no-uav || return
  cpy $(( ${SECS%.*} + 40 )) nav_checks n2 --seconds "$SECS" > "$L.n2.log" 2>&1
  line="$(field '^\[n2\] (PASS|FAIL)' "$L.n2.log")"
  grab_logs "$L"
  say "n2: ${line:-FAIL | no result (see $L.n2.log)}"
  sim_down
}

# ---------------------------------------------------------------- N3
# gz_remove <model>: Gazebo Harmonic's remove-entity service of the generated world. The gz CLI lives on this
# host, not in the container. Gazebo's reply goes to the log; 0 only when it says data: true.
gz_remove() {
  GZ_PARTITION=crusader_sim gz service -s "/world/$WORLD/remove" --reqtype gz.msgs.Entity \
    --reptype gz.msgs.Boolean --timeout 3000 --req "name: \"$1\" type: MODEL" 2>&1 \
    | tee -a "$OUT/gz_remove.log" | grep 'data: true' >/dev/null
}

n3_run() {
  local L="$OUT/n3" removed=() n wpid verdict ready
  echo "== n3  course $COURSE  in-view $IN_EL, out-of-view $OUT_EL"
  [ "$UP" = 0 ] || bring_up "n3" "$L" "$COURSE" "" --no-gui --no-uav || return
  WORLD="$(grep -m1 -oE '<world name="[^"]+"' "$GEN/worlds/$COURSE.sdf" 2>/dev/null | sed -E 's/.*name="([^"]+)"/\1/')"
  WORLD="${WORLD:-crusader_sim}"

  # The sim's params file keeps r_max 10 (the pool profile): the ENTRY buoy is 12.4 m out and the beyond-frustum
  # buoys 22 m.
  setp r_max 30.0 || { fail_run "n3: FAIL | could not set r_max 30"; return; }
  # The STVL-only twin planner (see the header): same cloud, same TF, the boat's STVL values, no HazardLayer.
  cpy 30 nav_checks twin-params --ns n3 --out /tmp/n3_twin.yaml > "$L.twin.log" 2>&1 \
    || { fail_run "n3: FAIL | could not write the twin's params (see $L.twin.log)"; return; }
  docker exec -d "$CONTAINER" bash -c "$ENVSRC && timeout 400 ros2 run nav2_planner planner_server --ros-args \
      -r __ns:=/n3 --params-file /tmp/n3_twin.yaml > /tmp/n3_twin.log 2>&1"
  sleep 4
  cx "ros2 lifecycle set /n3/planner_server configure && ros2 lifecycle set /n3/planner_server activate" \
    >> "$L.twin.log" 2>&1 || { fail_run "n3: FAIL | the twin planner would not configure and activate (see $L.twin.log)"; return; }

  docker exec "$CONTAINER" bash -c "$(pyline 200 nav_checks n3-watch --course "$COURSE" --ns n3 --in "$IN_EL" --out "$OUT_EL" \
      --mark-wait-s 30 --after-ready-s 45)" > "$L.n3.log" 2>&1 &
  wpid=$!
  while ! grep -q '^\[n3\] ready' "$L.n3.log" 2>/dev/null; do
    kill -0 "$wpid" 2>/dev/null || break; sleep 1
  done
  grep -q '^\[n3\] ready' "$L.n3.log" || { fail_run "n3: FAIL | the watcher never got ready (see $L.n3.log)"; return; }
  ready="$(grep -m1 '^\[n3\] ready' "$L.n3.log")"
  for n in "$IN_EL" "$OUT_EL"; do
    case "$ready " in *" $n=marked "*) ;; *) continue ;; esac
    if gz_remove "$n"; then removed+=("$n=$(epoch)"); else echo "    gz remove $n FAILED (see $OUT/gz_remove.log)"; fi
  done
  wait "$wpid"
  grab_logs "$L"
  docker cp "$CONTAINER:/tmp/n3_twin.log" "$L.twin_planner.log" >/dev/null 2>&1 || :
  verdict="$(hpy nav_checks score-n3 --log "$L.n3.log" --in "$IN_EL" --out "$OUT_EL" --removed ${removed[@]+"${removed[@]}"} 2>&1)"
  say "n3: $verdict$( [[ "$verdict" == FAIL* ]] && echo ' | spec: switch to the ObstacleLayer fallback (3.5); NOT switched here')"
  sim_down
}

trap sim_down EXIT      # from here on, however this ends, the sim is left down (never --all)
echo "== $CMD -> $OUT" | tee -a "$OUT/summary.txt"
case "$CMD" in
  s6) for r in $RMAXES; do s6_run "$r"; done ;;
  s9) s9_run ;;
  n2) n2_run ;;
  n3) n3_run ;;
esac
echo "logs: $OUT"
