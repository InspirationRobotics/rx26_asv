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
# livox_ros_driver2's `rviz_MID360_launch.py` starts rviz2 alongside the driver,
# and the stock launch file registers an OnProcessExit handler that SHUTS DOWN
# THE WHOLE LAUNCH when rviz exits. At boot there is no display, so rviz2 dies
# immediately and takes the driver with it — systemd restarts, rviz dies again,
# and you get a crash loop whose journal looks like an orderly shutdown rather
# than an error. The LiDAR is simply never there.
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
SETUPS="${CRSD_LIVOX_SETUP:-/opt/ros/humble/setup.bash /opt/livox_ws/install/setup.bash /root/ws_livox/install/setup.bash /root/robotx_ws/install/setup.bash}"
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
for f in ${SETUPS}; do [ -f "\$f" ] && . "\$f"; done
if ! command -v ros2 >/dev/null 2>&1; then
  echo "ERROR: ros2 not on PATH inside this container after sourcing:" >&2
  for f in ${SETUPS}; do
    [ -f "\$f" ] && echo "  [found]   \$f" >&2 || echo "  [missing] \$f" >&2
  done
  echo "ROS installs present:" >&2
  ls -d /opt/ros/*/ 2>/dev/null >&2 || echo "  (none under /opt/ros)" >&2
  echo "Overlay setup.bash candidates:" >&2
  # Bounded to the roots workspaces actually live under. \`find /\` here would
  # walk every bind mount the container happens to have, turning a one-line
  # diagnostic into a minute of silence before the error appears.
  find /opt /root /home /ws_livox /workspace -maxdepth 4 -name setup.bash \\
       -path "*install*" 2>/dev/null | head >&2
  echo "Set CRSD_LIVOX_SETUP in /etc/default/crusader to the right ones," >&2
  echo "base distro first, e.g.:" >&2
  echo '  CRSD_LIVOX_SETUP="/opt/ros/humble/setup.bash /opt/livox_ws/install/setup.bash"' >&2
  exit 127
fi
exec ${LAUNCH}
INNER_EOF

# -i (not -it): no TTY under systemd, but keep stdin so SIGTERM propagates to
# the launch instead of orphaning it inside the container on `systemctl stop`.
# Plain `bash -c`, not `-lc`: the environment is built above, explicitly, rather
# than depending on whatever this container's profile happens to do.
exec docker exec -i "$CONTAINER" bash -c "$INNER"
