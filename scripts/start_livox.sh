#!/bin/bash
# Start the MID360 driver inside the livox container. Called by
# crsd-livox.service; safe to run by hand for a bench check.
#
# The MID360 is the one device this repo does NOT own. It lives in its own
# container (see docs/OPERATIONS.md §13) and publishes /livox/lidar, which
# crusader_perception's lidar_cluster_node consumes. This script is the single
# blessed way to start it, so "how the LiDAR comes up" has one answer.
#
# ---------------------------------------------------------------------------
# THE rviz TRAP — read this before changing CRSD_LIVOX_LAUNCH.
#
# livox_ros_driver2's `rviz_MID360_launch.py` starts rviz2 alongside the driver.
# Where the STOCK launch file registers an OnProcessExit handler that SHUTS DOWN
# THE WHOLE LAUNCH when rviz exits, a headless boot kills rviz2 immediately and
# takes the driver with it — systemd restarts, rviz dies again, and you get a
# crash loop whose journal looks like an orderly shutdown rather than an error.
#
# MEASURED ON THIS BOAT 2026-09-03: it does NOT happen here. rviz2 died at boot
# (exit code -6) and the driver stayed up, publishing /livox/lidar at 10.0 Hz
# hours later. The launch file inside crusader_legacy evidently lacks that
# handler. So the rviz2 ERROR in the journal is not evidence the LiDAR is down —
# check `ros2 topic hz /livox/lidar` from inside asv before chasing it. The
# headless copy is still worth doing, as housekeeping rather than as a repair.
#
# It is still the default here because it is what publishes PointCloud2
# (xfer_format 0); `msg_MID360_launch.py` publishes CustomMsg, which the ROS 2
# type system will not connect to a PointCloud2 subscriber at all. The right fix
# is a headless launch — see the WARN this script prints, and §13.
# ---------------------------------------------------------------------------
#
# Usage: ./start_livox.sh              (env: CRSD_LIVOX_CONTAINER, CRSD_LIVOX_LAUNCH)
set -euo pipefail

CONTAINER="${CRSD_LIVOX_CONTAINER:-crusader_legacy}"
LAUNCH="${CRSD_LIVOX_LAUNCH:-ros2 launch livox_ros_driver2 rviz_MID360_launch.py}"
# ROS environment INSIDE the container, sourced explicitly. `docker exec bash -lc`
# is not enough: a login shell reads /etc/profile and ~/.bash_profile, while ROS
# setup conventionally lands in ~/.bashrc, which a NON-INTERACTIVE login shell
# never reads. The symptom is `ros2: command not found` from a container where
# `docker exec -it ... bash` then `ros2` works perfectly by hand.
# Space-separated; missing entries are skipped, so one list covers several
# layouts. Overlays must come AFTER the base distro.
#
# /root/livox_ws is THIS boat's (confirmed 2026-08-14). ws_livox is Livox's own
# convention and the two are easy to transpose — which is exactly the guess that
# cost a round trip, so both are listed and the autodiscovery below is the real
# safety net.
SETUPS="${CRSD_LIVOX_SETUP:-/opt/ros/humble/setup.bash /root/livox_ws/install/setup.bash /root/ws_livox/install/setup.bash /root/robotx_ws/install/setup.bash}"
# The MID360 is an ETHERNET device: the driver binds the host address in
# MID360_config.json (192.168.1.5) and talks to the sensor at 192.168.1.166.
HOST_IP="${CRSD_LIVOX_HOST_IP:-192.168.1.5}"
WAIT_S="${CRSD_LIVOX_WAIT_S:-30}"

# Name, never the 12-hex ID. An ID changes every time the container is recreated
# — `docker rm` + `docker run` and the unit silently points at nothing. Same
# reason the udev rules match VID/PID instead of chasing ttyACM numbering.
if ! docker inspect "$CONTAINER" >/dev/null 2>&1; then
  echo "ERROR: no container named '$CONTAINER'." >&2
  echo "  docker ps -a --format '{{.Names}}\t{{.Image}}\t{{.Status}}'" >&2
  echo "  Set CRSD_LIVOX_CONTAINER if it is named something else." >&2
  exit 1
fi

# network-online.target does NOT mean "this interface has this address". At boot
# the wired link often comes up seconds after the target fires, and the driver
# fails to bind with an error that reads like a LiDAR fault rather than a
# timing one. Wait for the address, bounded, and say which it is.
deadline=$(( SECONDS + WAIT_S ))
until ip -4 addr show | grep -q "inet ${HOST_IP}/"; do
  if (( SECONDS >= deadline )); then
    echo "ERROR: $HOST_IP is not on any interface after ${WAIT_S}s." >&2
    echo "  The driver binds that address (MID360_config.json host_net_info)." >&2
    echo "  Check the wired link:  ip -4 addr" >&2
    echo "  Bring it up, e.g.:     sudo ip addr add $HOST_IP/24 dev eth0" >&2
    exit 1
  fi
  sleep 1
done

# Idempotent: already-running is the normal case on a restart, and `docker start`
# on a running container succeeds without disturbing it.
docker start "$CONTAINER" >/dev/null

case "$LAUNCH" in
  *rviz*)
    # ADVISORY, not an error — it prints on every start. If the service is
    # failing, read the LAST line of the journal, not this.
    echo "NOTE: '$LAUNCH' starts rviz2, which cannot run headless. If the stock" >&2
    echo "      launch registers OnProcessExit->Shutdown, rviz dying takes the" >&2
    echo "      driver with it. See docs/OPERATIONS.md §13 for the headless copy." >&2
    ;;
esac

echo "starting MID360 driver in '$CONTAINER': $LAUNCH"

# Source explicitly, then check. `ros2: command not found` from inside a
# container where it plainly works by hand is confusing enough to be worth a
# real diagnostic rather than one line from bash.
read -r -d '' INNER <<INNER_EOF || true
ROOTS="/opt /root /home /ws_livox /workspace"
have_pkg() { ros2 pkg prefix livox_ros_driver2 >/dev/null 2>&1; }

# 1. whatever CRSD_LIVOX_SETUP names, if it exists
for f in ${SETUPS}; do [ -f "\$f" ] && . "\$f" 2>/dev/null; done

# 2. any ROS distro under /opt/ros, if that did not produce a ros2
if ! command -v ros2 >/dev/null 2>&1; then
  for d in /opt/ros/*/setup.bash; do
    [ -f "\$d" ] && . "\$d" 2>/dev/null && break
  done
fi

# 3. overlays, until one actually provides livox_ros_driver2. Guessing a
#    workspace path is what cost a round trip here: the layout differs per
#    container and the only reliable test is asking ros2 whether the package
#    resolves.
if command -v ros2 >/dev/null 2>&1 && ! have_pkg; then
  for s in \$(find \$ROOTS -maxdepth 5 -name setup.bash -path "*install*" 2>/dev/null); do
    . "\$s" 2>/dev/null || true
    have_pkg && { echo "livox overlay: \$s"; break; }
  done
fi

if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 is not on PATH in this container, even after autodiscovery." >&2
  echo "Tried CRSD_LIVOX_SETUP:" >&2
  for f in ${SETUPS}; do
    [ -f "\$f" ] && echo "  [found]   \$f" >&2 || echo "  [missing] \$f" >&2
  done
  echo "ROS installs present:" >&2
  ls -d /opt/ros/*/ 2>/dev/null >&2 || echo "  (none under /opt/ros)" >&2
  echo "What the interactive shell sources (this is what works by hand):" >&2
  grep -nE "^\\s*(source|\\.)\\s" /root/.bashrc "\$HOME/.bashrc" 2>/dev/null \\
    | grep setup >&2 || echo "  (nothing in .bashrc)" >&2
  exit 127
fi
if ! have_pkg; then
  echo "ERROR: ros2 works, but package 'livox_ros_driver2' does not resolve." >&2
  echo "Overlay setup.bash files searched under: \$ROOTS" >&2
  find \$ROOTS -maxdepth 5 -name setup.bash -path "*install*" 2>/dev/null | head >&2
  echo "Is the driver actually built in THIS container? Check by hand:" >&2
  echo "  docker exec -it ${CONTAINER} bash -lc 'ros2 pkg prefix livox_ros_driver2'" >&2
  exit 127
fi
echo "livox_ros_driver2 at: \$(ros2 pkg prefix livox_ros_driver2)"
exec ${LAUNCH}
INNER_EOF

# -i (not -it): no TTY under systemd, but keep stdin so SIGTERM propagates to
# the launch instead of orphaning it inside the container on `systemctl stop`.
# Plain `bash -c`, not `-lc`: the environment is built above, explicitly, rather
# than depending on whatever this container's profile happens to do.
exec docker exec -i "$CONTAINER" bash -c "$INNER"
