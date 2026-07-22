#!/bin/bash
# MAVProxy is the SOLE owner of the Pixhawk serial link (only ONE process may
# open it). It rebroadcasts MAVLink over UDP to everything else:
#   127.0.0.1:14551 -> telemetry_bridge (this repo's single ROS-side consumer)
#   127.0.0.1:14550 -> spare local consumer (preflight, ad-hoc mavproxy/QGC on host)
#   <LAPTOP_IP>:14550 -> Mission Planner / QGroundControl on the laptop
#
# Uses the stable udev symlink /dev/crsd-pixhawk (tools/udev/99-crusader.rules)
# instead of a per-boat /dev/serial/by-id path.
#
# Usage: ./start_mavproxy.sh [LAPTOP_IP]
set -euo pipefail

LAPTOP_IP="${1:-192.168.8.137}"          # override per field session
MASTER="${CRSD_PIXHAWK_DEV:-/dev/crsd-pixhawk}"

if [ ! -e "$MASTER" ]; then
  echo "ERROR: Pixhawk device $MASTER not found." >&2
  echo "  Check the udev symlink (tools/udev) or set CRSD_PIXHAWK_DEV, e.g.:" >&2
  echo "  ls -l /dev/serial/by-id/" >&2
  exit 1
fi

exec mavproxy.py \
  --master="$MASTER" \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14550 \
  --out=udp:"${LAPTOP_IP}":14550
