#!/usr/bin/env bash
# gz_nav_test.sh — one avoidance test case, N fresh runs, each scored on one line.
# Runs in: WSL2 Ubuntu-22.04. The test plan it serves: docs/nav2_avoidance_spec.md §10.2.
#
#     bash crusader_sim/scripts/gz_nav_test.sh <course> [--mode off|shadow|on] [--tree X.xml]
#          [--no-uav] [--runs N] [--gui] [--tag NAME] [--timeout-s S] [--detector truth|yolo]
#          [-- <task1_goal flags>]
#
# Each run: gz_sim_up.sh (fresh world, boat back at the start) -> task1_goal, with no
# TTY (gz_task1.sh needs one) -> logs kept -> gz_sim_down.sh --keep-container.
# Kept per run under ~/.cache/crusader_sim/runs/<tag>/ (not /tmp: WSL wipes it):
#     runN.up.log    bring-up            runN.task1.log  operator + tree + judge, each line
#     runN.bt.log    the container's     prefixed with seconds since the run started
#     runN.nav.log   /tmp/bt.log, nav.log
# and summary.txt, one line per run:
#     run N: <[result] line> | <judge VERDICT> | <min clearance> | mission <s> | BLOCKED xN
# "mission" is goal sent -> result, so arming and the GUIDED switch are not in it.
set -uo pipefail

usage() { sed -n '5,6p' "$0" >&2; exit 2; }
[ $# -ge 1 ] || usage
COURSE="$1"; shift
MODE=on; TREE_ARG=""; UAV=""; RUNS=1; GUI="--no-gui"; TAG=""; TMO=400; GOAL_ARGS=""; DETECTOR=truth
while [ $# -gt 0 ]; do
  case "$1" in
    --mode)      MODE="${2:?}"; shift ;;
    --tree)      TREE_ARG="${2:?}"; shift ;;
    --no-uav)    UAV="--no-uav" ;;
    --runs)      RUNS="${2:?}"; shift ;;
    --gui)       GUI="" ;;
    --tag)       TAG="${2:?}"; shift ;;
    --timeout-s) TMO="${2:?}"; shift ;;
    --detector)  DETECTOR="${2:?}"; shift ;;
    --)          shift; GOAL_ARGS="$*"; break ;;
    *)           echo "unknown option: $1" >&2; usage ;;
  esac
  shift
done
case "$MODE" in off|shadow|on) ;; *) echo "--mode must be off, shadow or on" >&2; exit 2 ;; esac
case "$DETECTOR" in truth|yolo) ;; *) echo "--detector must be truth or yolo" >&2; exit 2 ;; esac
[[ "$RUNS" =~ ^[1-9][0-9]*$ ]] || { echo "--runs must be a positive integer" >&2; exit 2; }
[[ "$COURSE" =~ ^[A-Za-z0-9_.-]+$ ]] || { echo "bad course name: $COURSE" >&2; exit 2; }

WIN_SRC="${RX26_WIN_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv}"
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
SIM="$WS_SRC/crusader_sim"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
OUT="$HOME/.cache/crusader_sim/runs/${TAG:-$(date +%Y%m%d_%H%M%S)_${COURSE}_${MODE}}"
mkdir -p "$OUT"

# Sync ONCE, before anything runs from the WSL copy: bash reads a script as it goes,
# so the copy must not change under a running gz_sim_up.sh.
tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sync.sh" > /tmp/gz_sync.sh \
  && RX26_WIN_SRC="$WIN_SRC" RX26_WSL_SRC="$WS_SRC" bash /tmp/gz_sync.sh > "$OUT/sync.log" 2>&1 \
  || { echo "*** Windows -> WSL sync failed (see $OUT/sync.log)" >&2; exit 2; }
[ -f "$SIM/courses/$COURSE.yaml" ] || { echo "no course $SIM/courses/$COURSE.yaml" >&2; exit 2; }

echo "== $COURSE  mode $MODE  tree ${TREE_ARG:-default}  ${UAV:-with UAV}  detector $DETECTOR  runs $RUNS  -> $OUT" | tee -a "$OUT/summary.txt"

# first line of $2 matching regex $1, without the seconds prefix (blank when absent)
pick() { grep -m1 -E "$1" "$2" 2>/dev/null | sed -E 's/^ *[0-9]+ //'; }
# the seconds prefix of the first line of $2 matching regex $1 (blank when absent)
stamp() { grep -m1 -E "$1" "$2" 2>/dev/null | awk '{print $1}'; }

for i in $(seq 1 "$RUNS"); do
  L="$OUT/run$i"
  # gz_sim_up.sh's own step 1 re-syncs identical content, which is harmless
  if ! NAV_MODE="$MODE" TREE="$TREE_ARG" SIM_DETECTOR="$DETECTOR" bash "$SIM/scripts/gz_sim_up.sh" "$COURSE" $GUI $UAV > "$L.up.log" 2>&1; then
    echo "run $i: BRING-UP FAILED (see $L.up.log)" | tee -a "$OUT/summary.txt"
    tail -5 "$L.up.log" | sed 's/^/    /'
    bash "$SIM/scripts/gz_sim_down.sh" --keep-container > "$L.down.log" 2>&1
    continue
  fi
  grep -E "AVOIDANCE OFF|planner_server|nav_mode|SIM_DETECTOR" "$L.up.log" | sed 's/^/    /'

  SECONDS=0
  timeout $(( ${TMO%.*} + 90 )) docker exec "$CONTAINER" bash -c \
    "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash &&
     python3 -u -m crusader_sim.task1_goal --course $COURSE --timeout-s $TMO $GOAL_ARGS" 2>&1 \
    | while IFS= read -r line; do printf '%4d %s\n' "$SECONDS" "$line"; done > "$L.task1.log"
  docker cp "$CONTAINER:/tmp/bt.log" "$L.bt.log" >/dev/null 2>&1 || : > "$L.bt.log"
  docker cp "$CONTAINER:/tmp/nav.log" "$L.nav.log" >/dev/null 2>&1 || : > "$L.nav.log"
  docker cp "$CONTAINER:/tmp/sim_camera.log" "$L.cam.log" >/dev/null 2>&1 || : > "$L.cam.log"
  cp "$L.task1.log" "$HOME/.cache/crusader_sim/task1_last.log"

  RES="$(pick '\[result\]' "$L.task1.log")"
  VER="$(pick 'VERDICT' "$L.task1.log")"
  CLR="$(pick 'min clearance' "$L.task1.log" | sed 's/^\[judge\] *//')"
  TG="$(stamp '\[operator\] goal:' "$L.task1.log")"; TR="$(stamp '\[result\]' "$L.task1.log")"
  MIS="n/a"; [ -n "$TG" ] && [ -n "$TR" ] && MIS="$(( TR - TG ))"
  BLK="$(grep -c 'BLOCKED' "$L.bt.log" 2>/dev/null)"
  # the mode bt_runner itself announced: a stale crusader_bt build ignores nav_mode
  # and runs legacy, which must not be scored as the mode asked for
  RAN="$(grep -m1 -oE 'obstacle avoidance: nav_mode [a-z]+' "$L.bt.log" 2>/dev/null | awk '{print $NF}')"
  BAD=""; [ "$RAN" = "$MODE" ] || BAD=" | *** INVALID: bt_runner ran nav_mode '${RAN:-none logged}', not '$MODE'"
  # with --detector yolo: the last score line sim_camera logged (the run's, not the window's), and
  # INVALID when it fell back to the oracle (a truth run must not be filed as a YOLO one)
  YOLO=""
  if [ "$DETECTOR" = yolo ]; then
    if grep -q "detector:=yolo is UNAVAILABLE\|SIM_DETECTOR=yolo UNAVAILABLE" "$L.cam.log" "$L.up.log" 2>/dev/null; then
      BAD="$BAD | *** INVALID: yolo was unavailable, the truth detector ran"
    else
      YOLO=" | $(grep 'yolo vs truth' "$L.cam.log" | tail -1 | sed -E 's/.*\|\| run/yolo run/')"
    fi
  fi
  echo "run $i: ${RES:-NO RESULT} | ${VER:-no verdict} | ${CLR:-no clearance} | mission ${MIS} s | BLOCKED x${BLK:-0}${YOLO}${BAD}" \
    | tee -a "$OUT/summary.txt"

  bash "$SIM/scripts/gz_sim_down.sh" --keep-container > "$L.down.log" 2>&1
done
echo "logs: $OUT"
