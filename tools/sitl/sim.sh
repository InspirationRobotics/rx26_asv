#!/bin/bash
# Run a command inside the crsd-sim container, restoring whatever WSL dropped.
#
# Runs FROM WINDOWS (Git Bash). Three execution contexts are in play and this
# script exists so you only have to think about the innermost one:
#
#   Windows laptop  ->  WSL2 Ubuntu-22.04  ->  docker crsd-sim  ->  your command
#
# WSL2 SHUTS ITSELF DOWN WHEN IDLE and takes dockerd and /tmp with it. The
# container then shows `exit=255`, which is NOT a container fault and not OOM --
# do not go looking for a crash. Every call here therefore re-establishes the
# whole chain before running anything.
#
#   bash tools/sitl/sim.sh 'colcon build --packages-select crusader_link'
#   bash tools/sitl/sim.sh --no-sync 'ros2 topic list'
#
# TWO QUOTING TRAPS, both of which cost real time:
#
#   * A `$VAR` in your command expands in the OUTER shell, where it is empty.
#     Write literal paths, not variables.
#   * `pkill -f <pattern>` matches this script's own argv and kills the shell
#     running it -- exit 143. Put pkill patterns in a FILE, never on a command
#     line. (`ros2 run pkg node` also execs the install-path binary, so a
#     pattern of "ros2 run crusader_fcu" matches only the short-lived wrapper.)
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
W="wsl.exe -d Ubuntu-22.04 --"

SYNC=1
if [ "${1:-}" = "--no-sync" ]; then SYNC=0; shift; fi
CMD="${1:?usage: sim.sh [--no-sync] '<command to run in the container>'}"

if [ "$SYNC" = 1 ]; then
  # sync_to_wsl.sh lives on the Windows side; copy it in each time because
  # /tmp does not survive a WSL shutdown either.
  MSYS_NO_PATHCONV=1 $W bash -c \
    "tr -d '\r' < /mnt/c/Users/chase/OneDrive/Documents/GitHub/RobotX_2026/Boat/rx26_asv/tools/sitl/sync_to_wsl.sh > /tmp/sync_to_wsl.sh; bash /tmp/sync_to_wsl.sh sync" \
    | tail -2
fi

MSYS_NO_PATHCONV=1 $W docker start crsd-sim > /dev/null 2>&1
MSYS_NO_PATHCONV=1 $W docker exec crsd-sim bash -lc \
  "source /opt/ros/humble/setup.bash; source /root/robotx_ws/install/setup.bash 2>/dev/null; cd /root/robotx_ws; $CMD"
