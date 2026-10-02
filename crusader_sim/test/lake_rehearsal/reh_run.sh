#!/bin/bash
# One lake-mode REHEARSAL run against the Gazebo sim, in WSL. Only when nobody else has the sim: it brings it up
# and down itself. The real boat is not involved; the sim stands in for core.launch.py + the OAK-D + the boat.
#   bash crusader_sim/test/lake_rehearsal/reh_run.sh <tag> <pass|abort|deadman> [course]
# Results: ~/.cache/lake_rehearsal/<tag>/ (driver.log has the story, verdict.txt the referee, rig_up.log the rig).
# 1 sim up (--no-uav --no-gui, NAV_MODE=on)   2 core stand-in in crsd-sim (what core.launch + the OAK-D give the boat)
# 3 lake_rig_up.sh (UNMODIFIED) in crsd-sim, PUBLISH=1 NAV_MODE=on   4 a standalone judge   5 the HTTP driver
# 6 verdict + logs into ~/.cache/lake_rehearsal/<tag>/   7 lake_rig_down.sh, sim down.
set -u
TAG=${1:?tag}; SCN=${2:?scenario}; COURSE=${3:-task1_core}
SP="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"      # this directory (core_standin.sh, pilot_standin.py, reh_driver.py ...)
SIM=$HOME/robotx_ws/src/rx26_asv/crusader_sim
OUT=$HOME/.cache/lake_rehearsal/$TAG
SCR=$HOME/robotx_ws/lake_scratch
mkdir -p "$OUT" "$SCR/reh"
for f in core_standin.sh pilot_standin.py judge_verdict.py; do tr -d '\r' < $SP/$f > $SCR/reh/$f; done
ENVC='source /opt/ros/humble/setup.bash; source /root/robotx_ws/install/setup.bash'
say() { echo "[reh $(date +%T)] $*" | tee -a "$OUT/steps.log"; }

say "sim up ($COURSE, --no-uav --no-gui, NAV_MODE=on)"
NAV_MODE=on bash $SIM/scripts/gz_sim_up.sh $COURSE --no-gui --no-uav > "$OUT/sim_up.log" 2>&1
tail -4 "$OUT/sim_up.log"
say "core stand-in (gz bridge, livox, camera, telemetry_bridge, lidar, gcs)"
docker exec crsd-sim bash /root/robotx_ws/lake_scratch/reh/core_standin.sh $COURSE | tee "$OUT/core_standin.log"
say "lake_rig_up.sh --check"
docker exec -e LAKE_DATUM=1.2806,103.8557 crsd-sim bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh --check | tee "$OUT/rig_check.log"
say "lake_rig_up.sh (PUBLISH=1 NAV_MODE=on)"
docker exec -e LAKE_DATUM=1.2806,103.8557 -e PUBLISH=1 -e NAV_MODE=on -e PANEL_PORT=8097 \
  -e LAKE_LOGDIR=/root/robotx_ws/lake_scratch/logs_$TAG crsd-sim \
  bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_up.sh | tee "$OUT/rig_up.log"
say "judge (standalone, background)"
docker exec -d crsd-sim bash -lc "$ENVC; cd /root/robotx_ws; python3 -u -m crusader_sim.task1_judge --course $COURSE > /tmp/judge_$TAG.log 2>&1"
sleep 3
PILOT="docker exec crsd-sim bash -lc '$ENVC; python3 /root/robotx_ws/lake_scratch/reh/pilot_standin.py arm-guided'"
HOLD="docker exec crsd-sim bash -lc '$ENVC; python3 /root/robotx_ws/lake_scratch/reh/pilot_standin.py hold'"
say "driver: $SCN"
python3 -u $SP/reh_driver.py --port 8097 --scenario $SCN --course $COURSE --pilot "$PILOT" --hold "$HOLD" 2>&1 | tee "$OUT/driver.log"
say "verdict"
docker exec crsd-sim bash -lc "$ENVC; cd /root/robotx_ws; python3 /root/robotx_ws/lake_scratch/reh/judge_verdict.py" | tee "$OUT/verdict.txt"
say "logs"
cp -r $SCR/logs_$TAG "$OUT/lake_logs" 2>/dev/null
tail -25 $SCR/logs_$TAG/bt.log > "$OUT/bt_tail.txt" 2>/dev/null
say "rig down, sim down"
docker exec -e LAKE_LOGDIR=/root/robotx_ws/lake_scratch/logs_$TAG crsd-sim bash /root/robotx_ws/src/rx26_asv/crusader_sim/scripts/lake_rig_down.sh | tee "$OUT/rig_down.log"
docker exec crsd-sim bash -lc 'for p in $(pgrep -f "crusader_sim.task1_judge"); do kill $p; done' 2>/dev/null
bash $SIM/scripts/gz_sim_down.sh --keep-container > "$OUT/sim_down.log" 2>&1
say "done -> $OUT"
