#!/usr/bin/env bash
# sim_up.sh -- the whole Task 1 simulator, from cold, in one call.
#
# Runs INSIDE WSL. On Windows do not run this by hand: double-click SIM_UP.cmd,
# which is this script plus the three browser tabs and QGroundControl.
#
#   Windows laptop  ->  WSL2 Ubuntu-22.04  ->  docker crsd-sim
#                       ArduRover SITL         every ROS node
#
# WHY SITL IS NOT IN THE CONTAINER. ardupilot is built in the WSL home
# (~/ardupilot) and the container runs NET=host, so udp:127.0.0.1:14551 is the
# same socket seen from either side. Moving SITL inside would cost a rebuild and
# buy nothing.
#
# CLICKING THIS TWICE IS A CLEAN RESTART, deliberately: start_sitl.sh stops what
# is running before it starts, so the boat goes back to the home position and
# the rig comes up fresh. Between two attempts at the same mission the GUI's
# "Return to start" button is the cheaper move -- it drives home under GUIDED
# without dropping the link. This one is for between sessions.
#
# IT DOES NOT BUILD, because a colcon build is minutes and a bring-up should be
# seconds. A C++ change needs, from Windows:
#     bash tools/sitl/sim.sh 'colcon build --packages-select crusader_bt'
# The Python tools -- bench_world_model.py, bt_view.py -- run straight out of
# src, so the sync below is all they need.
set -uo pipefail

DEFAULT_WIN=/mnt/c/Users/chase/OneDrive/Documents/GitHub/RobotX_2026/Boat/rx26_asv
WSL_SRC="${RX26_WSL_SRC:-$HOME/robotx_ws/src/rx26_asv}"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
CTR_SRC=/root/robotx_ws/src/rx26_asv

step() { printf '\n=== %s ===\n' "$1"; }
die()  { printf '\n*** FAILED: %s\n' "$1" >&2; exit 2; }
# exit 2 = nothing is up, do not bother opening tabs. exit 1 = the rig started
# but a page is not answering. SIM_UP.cmd tells those apart.

# Normalised with cd+pwd so SIM_UP.cmd can hand over a path with ../.. in it and
# every later message still names somewhere a human can go and look.
WIN_SRC="$(cd "${1:-$DEFAULT_WIN}" 2>/dev/null && pwd)" \
  || die "no Windows checkout visible at ${1:-$DEFAULT_WIN}"
[ -d "$WSL_SRC" ] || die "no WSL workspace at $WSL_SRC"

step "1/4  Windows -> WSL"
tr -d '\r' < "$WIN_SRC/tools/sitl/sync_to_wsl.sh" > /tmp/rx26_sync.sh \
  || die "cannot read $WIN_SRC/tools/sitl/sync_to_wsl.sh"
RX26_WIN_SRC="$WIN_SRC" bash /tmp/rx26_sync.sh sync | tail -3 | sed 's/^/  /'

step "2/4  ArduRover SITL"
bash "$WSL_SRC/tools/sitl/start_sitl.sh" 2>&1 | sed 's/^/  /'
# PIPESTATUS, not $?, which belongs to sed.
[ "${PIPESTATUS[0]}" -eq 0 ] || die "SITL did not start -- see /tmp/sitl.log"

step "3/4  the ROS rig, in $CONTAINER"
docker start "$CONTAINER" > /dev/null 2>&1 || die "cannot start container $CONTAINER"
docker exec "$CONTAINER" bash -lc "bash $CTR_SRC/tools/sitl/task1_sim_up.sh" 2>&1 | sed 's/^/  /'

# The rig printing "=== up ===" is not the same claim as a page answering on a
# socket: bench_world_model can be alive and still have died before it bound.
# That exact gap is what "the UI is dead" looked like the first time.
step "4/4  are the pages serving?"
probe() {
  local port=$1 name=$2 i
  for i in $(seq 1 20); do
    if (exec 3<>"/dev/tcp/127.0.0.1/$port") 2>/dev/null; then
      printf '  :%s  %-9s listening\n' "$port" "$name"
      return 0
    fi
    sleep 0.5
  done
  printf '  :%s  %-9s NOT LISTENING -- check the log\n' "$port" "$name"
  return 1
}
bad=0
probe 8086 aircraft || bad=1
probe 8085 tree     || bad=1
probe 8090 boat     || bad=1

echo
if [ "$bad" -eq 0 ]; then
  echo "  ready. Place a passage on :8086, hit Transmit, then Send goal."
else
  echo "  something did not come up. Logs, inside the container:"
  echo "    /tmp/gui.log  /tmp/btview.log  /tmp/gcs.log  /tmp/bt.log  /tmp/tb.log"
  echo "    bash tools/sitl/sim.sh --no-sync 'tail -40 /tmp/gui.log'"
fi
exit "$bad"
