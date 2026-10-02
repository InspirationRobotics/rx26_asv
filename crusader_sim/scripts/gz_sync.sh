#!/usr/bin/env bash
# gz_sync.sh — Windows checkout -> ~/robotx_ws/src. Runs in: WSL2 Ubuntu-22.04.
#
#     bash crusader_sim/scripts/gz_sync.sh
#
# Step 1 of gz_sim_up.sh, and the first thing task1_panel_up.sh does. Both run
# the WINDOWS copy of this file (CR stripped into /tmp): before the first sync
# the WSL workspace may not have it yet.
#
#     RX26_WIN_SRC   the Windows checkout, as a WSL path (default below)
#     RX26_WSL_SRC   the WSL workspace copy (default ~/robotx_ws/src/rx26_asv)
set -uo pipefail

WIN_SRC="${RX26_WIN_SRC:-/mnt/c/Users/Chaser/Documents/dev/RobotX_2026/Boat/rx26_asv}"
WS_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
SIM="$WS_SRC/crusader_sim"

[ -d "$WIN_SRC" ] || { echo "no Windows checkout at $WIN_SRC (set RX26_WIN_SRC)" >&2; exit 2; }
mkdir -p "$WS_SRC"
tr -d '\r' < "$WIN_SRC/tools/sitl/sync_to_wsl.sh" > /tmp/rx26_sync.sh
RX26_WIN_SRC="$WIN_SRC" RX26_WSL_SRC="$WS_SRC" bash /tmp/rx26_sync.sh sync | tail -1 | sed 's/^/  /'
# meshes are BINARY: the sync above strips \r from every file it copies, which
# would corrupt a .glb/.stl, so they are copied byte-for-byte here instead
if [ -d "$WIN_SRC/crusader_sim/meshes" ]; then
  mkdir -p "$SIM/meshes" && cp -u "$WIN_SRC/crusader_sim/meshes/"* "$SIM/meshes/" 2>/dev/null
  echo "  meshes: $(ls "$SIM/meshes" | wc -l) file(s)"
fi
# the Task 1 panel's pages: the team's extension list has no *.html, *.js or *.css, and the
# panel serves them from this source tree (task1_panel.py make_handler), not install/
for f in "$WIN_SRC"/crusader_sim/crusader_sim/*.html "$WIN_SRC"/crusader_sim/crusader_sim/*.js \
         "$WIN_SRC"/crusader_sim/crusader_sim/*.css; do
  if [ -f "$f" ]; then tr -d '\r' < "$f" > "$SIM/crusader_sim/$(basename "$f")"; fi
done
