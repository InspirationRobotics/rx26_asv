#!/bin/bash
# One lake-mode REHEARSAL run against the Gazebo sim, in WSL. Only when nobody else has the sim: it brings it up
# and down itself. The real boat is not involved; the sim stands in for core.launch.py + the OAK-D + the boat.
#
#   bash crusader_sim/test/lake_rehearsal/reh_run.sh <tag> <pass|recolour|abort|deadman|bringup> [course]
#
# Results: ~/.cache/lake_rehearsal/<tag>/ (summary.txt is the one-screen answer; driver.log has the story,
# verdict.txt the referee, rig_up.log the rig, bringup_check.txt the bringup scenario's checks).
#
# Scenarios (reh_driver.py has the detail): pass = load the course, COMMIT, the pilot's arm + GUIDED, START, ACK every
# checkpoint; recolour = the pass with a recolour + SEND CHANGES + ACK at checkpoint 1, the run COMPLETES; abort = the
# recolour, then ABORT GOAL in transit; deadman = the browser stops polling in transit and the boat must abort itself
# (~15 s); bringup = the rig only: bring it up, read back what it says it did (params, processes, the page's rig line),
# bring it down (no driver, no START).
#
# Environment (all optional):
#   RX26_WIN_SRC  the checkout to rehearse, as a WSL path. DEFAULT: the checkout THIS script lives in, so running it from
#                 a git worktree rehearses that worktree. It is synced into ~/robotx_ws/src/rx26_asv by gz_sync.sh,
#                 exactly as the sim scripts do (they read the same variable).
#   REH_TIER      disruptive (default) | advanced : the tier the page's START sends
#   REH_TREE, REH_NAV_MODE, REH_POOL, REH_TUNING   the rig's TREE, NAV_MODE, POOL, LAKE_TUNING, passed only when set.
#                 Unset = the rig's own defaults: the global tree, nav_mode off. (REH_TUNING is a path INSIDE crsd-sim,
#                 e.g. /root/robotx_ws/src/rx26_asv/crusader_sim/config/tuning_profiles/tight_3to5m.yaml)
#   REH_PUBLISH   1 (default) : the rig's PUBLISH.     REH_FEED_PORT  14557 : LAKE_FEED_PORT, so the rehearsal's panel
#                 does not collide with the sim's own Task 1 panel (which holds 14556).
#
# 0 sync the checkout (once, up front: a later sync of identical files is a no-op, so no script is rewritten under a
#   running bash)  1 sim up (--no-uav --no-gui)  2 core stand-in in crsd-sim (what core.launch + the OAK-D give the boat)
# 3 lake_rig_up.sh --check, then the rig itself (UNMODIFIED, as on the boat)  4 a standalone judge  5 the HTTP driver
# 6 verdict + logs into ~/.cache/lake_rehearsal/<tag>/  7 lake_rig_down.sh, sim down.
set -u
TAG=${1:?tag}; SCN=${2:?scenario}; COURSE=${3:-task1_core}
case "$SCN" in pass|recolour|abort|deadman|bringup) ;; *) echo "scenario must be pass|recolour|abort|deadman|bringup" >&2; exit 2 ;; esac
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WIN_SRC="${RX26_WIN_SRC:-$(cd "$HERE/../../.." && pwd)}"
export RX26_WIN_SRC="$WIN_SRC"
SP="$WIN_SRC/crusader_sim/test/lake_rehearsal"          # the helper files, from the checkout being rehearsed
SIM=$HOME/robotx_ws/src/rx26_asv/crusader_sim           # the synced copy the container sees
OUT=$HOME/.cache/lake_rehearsal/$TAG
SCR=$HOME/robotx_ws/lake_scratch
TIER=${REH_TIER:-disruptive}
PORT=8097
mkdir -p "$OUT" "$SCR/reh"
say() { echo "[reh $(date +%T)] $*" | tee -a "$OUT/steps.log"; }

# the rig's environment, only what was asked for; the rest is the rig's own defaults
RIGENV=(-e LAKE_DATUM=1.2806,103.8557 -e "PUBLISH=${REH_PUBLISH:-1}" -e PANEL_PORT=$PORT -e "LAKE_FEED_PORT=${REH_FEED_PORT:-14557}")
for pair in TREE:REH_TREE NAV_MODE:REH_NAV_MODE POOL:REH_POOL LAKE_TUNING:REH_TUNING; do
  var=${pair%%:*}; src=${pair##*:}
  [ -n "${!src:-}" ] && RIGENV+=(-e "$var=${!src}")
done
# the sim's own rig starts Nav2 only when the lake rig will use it; core_standin.sh stops the sim's rig either way
SIMNAV=off
case "${REH_NAV_MODE:-}" in shadow|on) SIMNAV=on ;; esac
if [ -z "${REH_NAV_MODE:-}" ] && [ -n "${REH_TREE:-}" ] && [ "$(basename "$REH_TREE")" != task1_global.xml ] && [ "${REH_POOL:-}" != 1 ]; then SIMNAV=on; fi

say "checkout $WIN_SRC  scenario $SCN  tier $TIER  rig env: ${RIGENV[*]}"
say "sync (once, up front)"
tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sync.sh" > /tmp/gz_sync_reh.sh
bash /tmp/gz_sync_reh.sh 2>&1 | tail -3
for f in core_standin.sh pilot_standin.py judge_verdict.py bringup_container.sh; do tr -d '\r' < "$SP/$f" > "$SCR/reh/$f"; done
ENVC='source /opt/ros/humble/setup.bash; source /root/robotx_ws/install/setup.bash'
RIGEXEC=(docker exec "${RIGENV[@]}" -e "LAKE_LOGDIR=/root/robotx_ws/lake_scratch/logs_$TAG" crsd-sim)
RIGUP=/root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh

say "sim up ($COURSE, --no-uav --no-gui, NAV_MODE=$SIMNAV)"
NAV_MODE=$SIMNAV bash $SIM/scripts/gz_sim_up.sh $COURSE --no-gui --no-uav > "$OUT/sim_up.log" 2>&1
tail -4 "$OUT/sim_up.log"
say "core stand-in (gz bridge, livox, camera, telemetry_bridge, lidar, gcs)"
docker exec crsd-sim bash /root/robotx_ws/lake_scratch/reh/core_standin.sh $COURSE | tee "$OUT/core_standin.log"
say "lake_rig_up.sh --check"
"${RIGEXEC[@]}" bash $RIGUP --check 2>&1 | tee "$OUT/rig_check.log"
say "lake_rig_up.sh"
"${RIGEXEC[@]}" bash $RIGUP 2>&1 | tee "$OUT/rig_up.log"

if [ "$SCN" = bringup ]; then
  say "bringup checks (the rig is up; no START)"
  { docker exec crsd-sim bash /root/robotx_ws/lake_scratch/reh/bringup_container.sh
    python3 "$SP/bringup_check.py" --port $PORT; } 2>&1 | tee "$OUT/bringup_check.txt"
else
  say "judge (standalone, background)"
  docker exec -d crsd-sim bash -lc "$ENVC; cd /root/robotx_ws; python3 -u -m crusader_sim.task1_judge --course $COURSE > /tmp/judge_$TAG.log 2>&1"
  sleep 3
  PILOT="docker exec crsd-sim bash -lc '$ENVC; python3 /root/robotx_ws/lake_scratch/reh/pilot_standin.py arm-guided'"
  HOLD="docker exec crsd-sim bash -lc '$ENVC; python3 /root/robotx_ws/lake_scratch/reh/pilot_standin.py hold'"
  say "driver: $SCN ($TIER)"
  python3 -u "$SP/reh_driver.py" --port $PORT --scenario $SCN --tier $TIER --course $COURSE --pilot "$PILOT" --hold "$HOLD" \
    --result "$OUT/result.json" 2>&1 | tee "$OUT/driver.log"
  say "verdict"
  docker exec crsd-sim bash -lc "$ENVC; cd /root/robotx_ws; python3 /root/robotx_ws/lake_scratch/reh/judge_verdict.py" | tee "$OUT/verdict.txt"
fi
say "logs"
cp -r $SCR/logs_$TAG "$OUT/lake_logs" 2>/dev/null
tail -25 $SCR/logs_$TAG/bt.log > "$OUT/bt_tail.txt" 2>/dev/null
say "rig down, sim down"
docker exec -e LAKE_LOGDIR=/root/robotx_ws/lake_scratch/logs_$TAG crsd-sim bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_down.sh | tee "$OUT/rig_down.log"
docker exec crsd-sim bash -lc 'for p in $(pgrep -f "crusader_sim.task1_judge"); do kill $p; done' 2>/dev/null
bash $SIM/scripts/gz_sim_down.sh --keep-container > "$OUT/sim_down.log" 2>&1
python3 "$SP/reh_summary.py" "$OUT" --tag "$TAG" --scenario "$SCN" --tier "$TIER" | tee "$OUT/summary.txt"
say "done -> $OUT"
