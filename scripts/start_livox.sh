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
    echo "WARN: launching '$LAUNCH', which starts rviz2." >&2
    echo "      Headless (at boot) rviz2 exits immediately, and the stock livox" >&2
    echo "      launch shuts the whole launch down with it -> restart loop." >&2
    echo "      Make a headless copy INSIDE the container, once:" >&2
    echo "        cd \$(ros2 pkg prefix livox_ros_driver2)/share/livox_ros_driver2/launch_ROS2" >&2
    echo "        cp rviz_MID360_launch.py MID360_headless_launch.py" >&2
    echo "        # delete the rviz2 Node and the OnProcessExit/Shutdown handler" >&2
    echo "      then set in /etc/default/crusader:" >&2
    echo "        CRSD_LIVOX_LAUNCH=\"ros2 launch livox_ros_driver2 MID360_headless_launch.py\"" >&2
    ;;
esac

echo "starting MID360 driver in '$CONTAINER': $LAUNCH"
# -i (not -it): no TTY under systemd, but keep stdin so SIGTERM propagates to
# the launch instead of orphaning it inside the container on `systemctl stop`.
exec docker exec -i "$CONTAINER" bash -lc "$LAUNCH"
