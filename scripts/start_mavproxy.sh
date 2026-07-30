#!/bin/bash
# MAVProxy is the SOLE owner of the Pixhawk serial link (only ONE process may
# open it). It rebroadcasts MAVLink over UDP to everything else:
#   127.0.0.1:14551      -> telemetry_bridge (this repo's single ROS-side consumer)
#   127.0.0.1:14550      -> spare local consumer (preflight, ad-hoc mavproxy/QGC on host)
#   <BCAST_ADDR>:14550   -> Mission Planner / QGroundControl on ANY laptop on the
#                           field WiFi (broadcast, not unicast) — QGC/Mission
#                           Planner both listen on 14550 for traffic from any
#                           sender, so nothing needs to be typed in on the laptop
#                           side and no per-laptop/per-day IP has to be passed here.
#
# CAVEAT: broadcast is subnet-scoped, not laptop-scoped. If two teams share the
# same field network, every laptop on it sees every broadcasting boat's telemetry
# — pick the right vehicle in QGC/Mission Planner's connection list. And if the
# field network ever hands out a different subnet than 192.168.100.0/24, update
# BCAST_ADDR to match (it must be that subnet's broadcast address, i.e. network
# address with the host bits set to 1 — .255 for a /24).
#
# Uses the stable udev symlink /dev/crsd-pixhawk (tools/udev/99-crusader.rules)
# instead of a per-boat /dev/serial/by-id path.
#
# Usage: ./start_mavproxy.sh [BCAST_ADDR]
set -euo pipefail

BCAST_ADDR="${1:-192.168.100.255}"         # override if the field subnet changes
MASTER="${CRSD_PIXHAWK_DEV:-/dev/crsd-pixhawk}"

if [ ! -e "$MASTER" ]; then
  echo "ERROR: Pixhawk device $MASTER not found." >&2
  echo "  Check the udev symlink (tools/udev) or set CRSD_PIXHAWK_DEV, e.g.:" >&2
  echo "  ls -l /dev/serial/by-id/" >&2
  exit 1
fi

# --daemon is REQUIRED under systemd, not a preference. MAVProxy runs an
# interactive console by default; with stdin on /dev/null it prints the "MAV> "
# prompt, immediately reads EOF, treats that as "quit", and unloads every module
# and exits 1. systemd restarts it, and you get a clean-looking crash loop whose
# log ends in an orderly shutdown rather than an error. Confirmed on the boat:
# identical invocation stays up in a terminal and dies under systemd.
# Run it by hand (a TTY) and you get the console; drop --daemon here and the
# service will loop forever.
exec mavproxy.py \
  --master="$MASTER" \
  --daemon \
  --out=udp:127.0.0.1:14551 \
  --out=udp:127.0.0.1:14550 \
  --out=udpbcast:"${BCAST_ADDR}":14550
