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
#   <each GCS_IPS>:14550 -> explicit unicast, for the laptops broadcast does not
#                           reach. See "when broadcast is not enough" below.
#
# CAVEAT: broadcast is subnet-scoped, not laptop-scoped. If two teams share the
# same field network, every laptop on it sees every broadcasting boat's telemetry
# — pick the right vehicle in QGC/Mission Planner's connection list. And if the
# field network ever hands out a different subnet than 192.168.100.0/24, update
# BCAST_ADDR to match (it must be that subnet's broadcast address, i.e. network
# address with the host bits set to 1 — .255 for a /24).
#
# WHEN BROADCAST IS NOT ENOUGH. A subnet broadcast only reaches hosts ON that
# subnet, and only if nothing between drops it — an AP with client isolation, a
# laptop on the other side of the Bullet AC bridge, or a machine on the wired
# 192.168.1.0/24 all see nothing while the boat looks perfectly healthy from the
# Jetson. GCS_IPS adds an explicit unicast --out per address for exactly those:
#
#     GCS_IPS="192.168.100.50 192.168.1.20" ./start_mavproxy.sh
#
# Space-separated. Under systemd put it in /etc/default/crusader (the unit's
# EnvironmentFile), quoted, and it reaches this script through the environment
# with no unit edit:
#
#     GCS_IPS="192.168.100.50 192.168.1.20"
#
# Unicast is additive, not a replacement — broadcast stays on, so a laptop that
# was already working keeps working whether or not its address is listed here.
#
# Uses the stable udev symlink /dev/crsd-pixhawk (tools/udev/99-crusader.rules)
# instead of a per-boat /dev/serial/by-id path.
#
# Usage: ./start_mavproxy.sh [BCAST_ADDR]        (env: GCS_IPS, CRSD_PIXHAWK_DEV)
set -euo pipefail

BCAST_ADDR="${1:-192.168.100.255}"         # override if the field subnet changes
MASTER="${CRSD_PIXHAWK_DEV:-/dev/crsd-pixhawk}"

if [ ! -e "$MASTER" ]; then
  echo "ERROR: Pixhawk device $MASTER not found." >&2
  echo "  Check the udev symlink (tools/udev) or set CRSD_PIXHAWK_DEV, e.g.:" >&2
  echo "  ls -l /dev/serial/by-id/" >&2
  exit 1
fi

# Built as an array so each --out is one argv element. 14551 stays FIRST because
# it is the one output the boat cannot run without: telemetry_bridge is the sole
# ROS-side consumer, and everything downstream of it (LEDs, RC watchdog, pose,
# attitude) goes dark if it is missing.
OUTS=(--out=udp:127.0.0.1:14551
      --out=udp:127.0.0.1:14550
      --out=udpbcast:"${BCAST_ADDR}":14550)

# UNQUOTED on purpose: GCS_IPS is a space-separated list and this relies on word
# splitting to turn it into one --out per address. Quoting "${GCS_IPS}" would
# produce a single bogus --out containing spaces, and MAVProxy would fail to
# parse the address rather than obviously ignoring it. The :- keeps `set -u`
# happy when the variable is unset, which is the normal case.
for ip in ${GCS_IPS:-}; do
  OUTS+=(--out=udp:"${ip}":14550)
done

# --daemon is REQUIRED under systemd, not a preference. MAVProxy runs an
# interactive console by default; with stdin on /dev/null it prints the "MAV> "
# prompt, immediately reads EOF, treats that as "quit", and unloads every module
# and exits 1. systemd restarts it, and you get a clean-looking crash loop whose
# log ends in an orderly shutdown rather than an error. Confirmed on the boat:
# identical invocation stays up in a terminal and dies under systemd.
# Run it by hand (a TTY) and you get the console; drop --daemon here and the
# service will loop forever.
#
# --streamrate=-1 means "request NOTHING; leave the vehicle's own SRx_* rates
# alone". It is not a tuning choice — without it MAVProxy sends
# REQUEST_DATA_STREAM(MAV_DATA_STREAM_ALL, 4Hz) on every connect and after every
# reconnect, which overwrites SR0_* in the autopilot's RAM. A rate set in
# QGroundControl is saved to the Pixhawk's EEPROM, looks correct in QGC forever,
# and is silently stomped back to 4 Hz the moment this service restarts. That is
# how /crsd/attitude ends up at 4 Hz while the params say 30.
#
# So per-message rates are now set ONCE, in QGC, on SR0_* (USB = SERIAL0, which
# is the port this --master opens). Raise only the stream you need: SR0_EXTRA1
# carries ATTITUDE and is the one mission-element mapping cares about. Doing it
# that way instead of --streamrate=30 also keeps RAW_SENS/PARAMS/etc at their
# low rates rather than lifting every stream at once.
#
# If SR0_* is ever left at 0 the vehicle streams nothing and telemetry_bridge
# sits at "still no heartbeat" — check the params before suspecting the link.
exec mavproxy.py \
  --master="$MASTER" \
  --daemon \
  --streamrate=-1 \
  "${OUTS[@]}"
