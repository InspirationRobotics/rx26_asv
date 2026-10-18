#!/bin/bash
# MAVProxy is the SOLE owner of the Pixhawk serial link (only ONE process may open it). It
# rebroadcasts MAVLink over UDP to everything else:
#   127.0.0.1:14551      -> telemetry_bridge (this repo's single ROS-side consumer)
#   127.0.0.1:14552      -> RESERVED for batt_watchdog --dry-run (the automatic low-voltage
#                           poweroff, crsd-battwatch, was REMOVED on 2026-09-05; the port stays
#                           reserved so a monitor run never has to take 14550)
#   127.0.0.1:14550      -> ad-hoc tooling ONLY (preflight, param_guard --live, dump_params, a
#                           scratch mavproxy/QGC on the host). KEEP IT FREE: a udpin bind STEALS
#                           datagrams, so a long-lived service squatting here makes those tools
#                           sit in silence rather than fail with an error naming the cause.
#                           Anything permanent gets its own port, like the watchdog's 14552.
#   <BCAST_ADDR>:14550   -> Mission Planner / QGroundControl on ANY laptop on the field WiFi
#                           (broadcast, not unicast: both listen on 14550 for any sender, so no
#                           per-laptop IP has to be typed in or passed here)
#   <each GCS_IPS>:14550 -> explicit unicast, for the laptops broadcast does not reach (below)
#
# CAVEAT: broadcast is subnet-scoped, not laptop-scoped. If two teams share the field network,
# every laptop on it sees every broadcasting boat's telemetry: pick the right vehicle in the
# connection list. When the network hands out a different subnet, update BCAST_ADDR to that
# subnet's broadcast address (.255 for a /24).
#
# WHEN BROADCAST IS NOT ENOUGH. A subnet broadcast only reaches hosts ON that subnet, and only if
# nothing drops it (an AP with client isolation, a laptop across the Bullet AC bridge, a machine on
# the wired 192.168.1.0/24: all see nothing while the boat looks healthy from the Jetson). GCS_IPS
# adds an explicit unicast --out per address for exactly those:
#
#     GCS_IPS="192.168.100.50 192.168.1.20" ./start_mavproxy.sh
#
# Space-separated. Under systemd put it, quoted, in /etc/default/crusader (the unit's
# EnvironmentFile); it reaches this script through the environment with no unit edit. Unicast is
# additive: broadcast stays on.
#
# Uses the stable udev symlink /dev/crsd-pixhawk (tools/udev/99-crusader.rules)
# instead of a per-boat /dev/serial/by-id path.
#
# Usage: ./start_mavproxy.sh [BCAST_ADDR]        (env: GCS_IPS, CRSD_PIXHAWK_DEV)
set -euo pipefail

# DEFAULT IS THE BULLET'S WIRED SUBNET, NOT THE FIELD WIFI. 192.168.8.255 is the broadcast address
# of the 192.168.8.0/24 side of the Bullet AC bridge, the link that exists at the venue; the Jetson's
# WiFi is a different subnet (192.168.100.0/24 in the lab) and a broadcast to one does NOT reach the
# other. Pass the argument, or set CRSD_BCAST_ADDR in /etc/default/crusader, to broadcast on the
# subnet you are on; use GCS_IPS for a laptop on the far side of either.
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

# UNQUOTED on purpose: GCS_IPS is a space-separated list and this relies on word splitting to make one
# --out per address; quoting would produce one bogus --out containing spaces that MAVProxy fails to
# parse. The :- keeps `set -u` happy when the variable is unset (the normal case).
for ip in ${GCS_IPS:-}; do
  OUTS+=(--out=udp:"${ip}":14550)
done

# --daemon is REQUIRED under systemd, not a preference. MAVProxy runs an interactive console by
# default; with stdin on /dev/null it prints the "MAV> " prompt, reads EOF, treats that as "quit",
# unloads every module and exits 1, so systemd restarts it in a clean-looking crash loop whose log
# ends in an orderly shutdown. Run by hand (a TTY) you get the console; drop --daemon here and the
# service loops forever.
#
# --streamrate=-1 means "request NOTHING; leave the vehicle's own SRx_* rates alone". It is not a
# tuning choice: without it MAVProxy sends REQUEST_DATA_STREAM(ALL, 4Hz) on every connect and
# reconnect, overwriting SR0_* in the autopilot's RAM. A rate set in QGC is saved to the Pixhawk's
# EEPROM, looks correct there forever, and is silently stomped to 4 Hz when this service restarts
# (how /crsd/attitude ends up at 4 Hz while the params say 30).
#
# So per-message rates are set ONCE, in QGC, on SR0_* (USB = SERIAL0, the port this --master opens).
# Raise only the stream you need: SR0_EXTRA1 carries ATTITUDE, the one mission-element mapping cares
# about; --streamrate=30 would lift every stream at once.
#
# If SR0_* is ever left at 0 the vehicle streams nothing and telemetry_bridge sits at "still no
# heartbeat": check the params before suspecting the link.
# HEARTBEAT AT 5 Hz, and why it cannot be done with a parameter.
#
# ArduPilot sends HEARTBEAT on a FIXED 1 Hz timer, in no SRx_* stream group, so the only way to
# change it is MAV_CMD_SET_MESSAGE_INTERVAL at runtime (ArduRover 4.6.3 accepts it: 1.00 -> 5.33 Hz).
#
# It matters because /crsd/fcu_status (mode + armed) is fed by HEARTBEAT ALONE and the boat reports
# state to RoboCommand at 2 Hz. At 1 Hz the source period equalled telemetry_bridge's 1.0 s staleness
# threshold, so ordinary jitter filled the log with "stream 'fcu_status' stale" on a healthy boat; at
# 5 Hz there is 5x margin and that error means something again.
#
# 200000 us = 5 Hz. Interval 0 would mean "default rate"; -1 means DISABLE, not restore (it would
# switch heartbeat OFF and take fcu_status, the LED stack and the RC watchdog's bridge_ok with it).
#
# THE CAVEAT: SET_MESSAGE_INTERVAL is RUNTIME state, not EEPROM. This --cmd re-requests it every time
# MAVProxy starts (host reboot, `systemctl restart`) but NOT when the autopilot reboots on its own:
# the rate then falls back to 1 Hz and the stale-stream errors return. That error storm IS the
# signal; `sudo systemctl restart crsd-mavproxy` puts it back. Check with:
#     python3 -c "from pymavlink import mavutil; ..."   (or tools/scripts/preflight.py)
HEARTBEAT_US="${CRSD_HEARTBEAT_US:-200000}"

exec mavproxy.py \
  --master="$MASTER" \
  --daemon \
  --streamrate=-1 \
  --cmd="long SET_MESSAGE_INTERVAL 0 ${HEARTBEAT_US}" \
  "${OUTS[@]}"
