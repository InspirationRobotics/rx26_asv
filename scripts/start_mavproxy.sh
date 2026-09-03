#!/bin/bash
# MAVProxy is the SOLE owner of the Pixhawk serial link (only ONE process may
# open it). It rebroadcasts MAVLink over UDP to everything else:
#   127.0.0.1:14551      -> telemetry_bridge (this repo's single ROS-side consumer)
#   127.0.0.1:14552      -> batt_watchdog (crsd-battwatch, low-voltage poweroff)
#   127.0.0.1:14550      -> ad-hoc tooling ONLY (preflight, param_guard --live,
#                           dump_params, a scratch mavproxy/QGC on the host).
#                           KEEP IT FREE: a udpin bind STEALS datagrams, so a
#                           long-lived service squatting here would make those
#                           tools sit in silence rather than fail with an error
#                           that names the cause. Anything permanent gets its
#                           own port, which is why the watchdog has 14552.
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
# — pick the right vehicle in QGC/Mission Planner's connection list. And when the
# network hands out a subnet other than this script's default, update BCAST_ADDR
# to match (it must be that subnet's broadcast address, i.e. the network address
# with the host bits set to 1 — .255 for a /24).
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

# DEFAULT IS THE BULLET'S WIRED SUBNET, NOT THE FIELD WIFI. 192.168.8.255 is the
# broadcast address of the 192.168.8.0/24 side of the Bullet AC bridge, which is
# the link that exists at the venue. The Jetson's WiFi is a different subnet
# (192.168.100.0/24 in the lab as of 2026-09-03), and a broadcast to one does
# NOT reach the other. Pass the argument, or set CRSD_BCAST_ADDR in
# /etc/default/crusader, to broadcast on the subnet you are actually on; use
# GCS_IPS for a laptop on the far side of either.
BCAST_ADDR="${1:-192.168.8.255}"
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
      --out=udp:127.0.0.1:14552
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
# HEARTBEAT AT 5 Hz, and why it cannot be done with a parameter.
#
# ArduPilot sends HEARTBEAT on a FIXED 1 Hz timer. It is in no SRx_* stream
# group — there is no SR0_HEARTBEAT to raise — so the only way to change it is
# MAV_CMD_SET_MESSAGE_INTERVAL at runtime. Confirmed on Crusader 2026-09-03:
# ArduRover 4.6.3 returns MAV_RESULT_ACCEPTED and the rate goes 1.00 -> 5.33 Hz.
#
# It matters because /crsd/fcu_status (mode + armed) is fed by HEARTBEAT ALONE,
# and the boat has to report state to RoboCommand at 2 Hz. A 1 Hz source cannot
# honestly feed a 2 Hz report — and at 1 Hz the source period exactly equalled
# telemetry_bridge's 1.0s staleness threshold, so ordinary jitter tripped it and
# the log filled with "stream 'fcu_status' stale" on a perfectly healthy boat.
# At 5 Hz there is 5x margin and that error means something again.
#
# 200000 us = 5 Hz. Interval 0 would mean "default rate"; interval -1 means
# DISABLE, not restore — sending -1 here would switch heartbeat OFF and take
# fcu_status, the LED stack and the RC watchdog's bridge_ok with it.
#
# THE CAVEAT, because it will bite someone: SET_MESSAGE_INTERVAL is RUNTIME
# state, not EEPROM. It survives nothing. This --cmd re-requests it every time
# MAVProxy starts, which covers a host reboot and a `systemctl restart`. It does
# NOT cover the autopilot rebooting on its own — after that the rate silently
# falls back to 1 Hz and the stale-stream errors return. That error storm IS the
# signal: if you see it, the request was lost, and
#     sudo systemctl restart crsd-mavproxy
# puts it back. Check with:
#     python3 -c "from pymavlink import mavutil; ..."   (or tools/scripts/preflight.py)
HEARTBEAT_US="${CRSD_HEARTBEAT_US:-200000}"

exec mavproxy.py \
  --master="$MASTER" \
  --daemon \
  --streamrate=-1 \
  --cmd="long SET_MESSAGE_INTERVAL 0 ${HEARTBEAT_US}" \
  "${OUTS[@]}"
