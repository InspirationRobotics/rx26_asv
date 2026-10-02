#!/bin/bash
# SIM-mode Task 1 panel flow, in WSL: the sim panel on :8095 launches the sim itself (NAV_MODE=on) and sim_flow.py drives it
# over HTTP (LAUNCH, START, ACK every checkpoint, read the boat's-map layers + the panel's judge).
#   bash crusader_sim/test/lake_rehearsal/sim_flow_run.sh [course]       the sim must be FREE; it is left down
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COURSE=${1:-task1_avoid}
WIN=/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv
WS=$HOME/robotx_ws/src/rx26_asv
OUT=$HOME/.cache/lake_rehearsal/sim_flow_$COURSE
mkdir -p "$OUT"
tr -d '\r' < $WIN/crusader_sim/scripts/gz_sync.sh > /tmp/gz_sync.sh && bash /tmp/gz_sync.sh | tail -1
export PYTHONPATH="$WS/crusader_sim" GZ_PARTITION=crusader_sim NAV_MODE=on GALLIUM_DRIVER=d3d12 MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
cd $HOME
python3 -u -m crusader_sim.task1_panel --port 8095 > "$OUT/panel.log" 2>&1 &
PANEL=$!
sleep 4
python3 -u $HERE/sim_flow.py --port 8095 --course $COURSE 2>&1 | tee "$OUT/flow.log"
kill $PANEL 2>/dev/null
bash $WS/crusader_sim/scripts/gz_sim_down.sh --keep-container > "$OUT/sim_down.log" 2>&1
echo "sim down; logs in $OUT"
