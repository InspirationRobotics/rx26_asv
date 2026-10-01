#!/usr/bin/env bash
# task1_panel_up.sh — sync, then run the Task 1 panel on :8095. Runs in: WSL2
# Ubuntu-22.04, on the host (not in the container), started hidden by
# TASK1_PANEL.cmd from a CR-stripped copy in /tmp.
#
# This process IS the panel's WSL keep-alive: WSL2 stops the VM about a minute
# after the last wsl.exe client exits, and this one becomes the panel (exec),
# so the VM lives exactly as long as the panel does. GZ_SIM_DOWN.cmd
# (gz_sim_down.sh --all) ends it.
#
# The panel launches gz_sim_up.sh and gz_sim_down.sh itself, so everything they
# need from the environment is set here once and inherited. RX26_WIN_SRC comes
# in from TASK1_PANEL.cmd through WSLENV.
#
# Log: /tmp/task1_panel.log (this script's output too — nobody sees its window).
exec >> /tmp/task1_panel.log 2>&1
echo "=== $(date '+%F %T') task1_panel_up ==="

WIN_SRC="${RX26_WIN_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv}"
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
SIM="$WS_SRC/crusader_sim"
export RX26_WIN_SRC="$WIN_SRC" RX26_WSL_SRC="$WS_SRC"

# one panel at a time: a second double-click replaces the first rather than
# failing to bind :8095
PANEL='crusader_sim\.task1_panel --port 8095'   # not test instances on other ports
pkill -f "$PANEL" 2>/dev/null
for _ in $(seq 1 10); do pgrep -f "$PANEL" >/dev/null || break; sleep 0.5; done

# the WINDOWS copy, as gz_sim_up.sh does: the workspace may not have it yet
tr -d '\r' < "$WIN_SRC/crusader_sim/scripts/gz_sync.sh" > /tmp/gz_sync.sh \
  || { echo "*** no gz_sync.sh under $WIN_SRC (set RX26_WIN_SRC)"; exit 2; }
bash /tmp/gz_sync.sh || { echo "*** Windows -> WSL sync failed"; exit 2; }

export PATH="$HOME/.local/bin:$PATH"
export PYTHONPATH="$SIM${PYTHONPATH:+:$PYTHONPATH}"
export GZ_PARTITION=crusader_sim
# GPU as in gz_sim_up.sh (see the reason there), for anything Gazebo the panel
# starts itself; GZ_GPU=cpu for software rendering
GZ_GPU="${GZ_GPU:-NVIDIA}"
if [ "$GZ_GPU" = cpu ]; then
  export LIBGL_ALWAYS_SOFTWARE=1
else
  export GALLIUM_DRIVER=d3d12
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="$GZ_GPU"
fi

cd "$HOME"
exec python3 -u -m crusader_sim.task1_panel --port 8095
