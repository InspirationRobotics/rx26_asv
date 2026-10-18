#!/usr/bin/env bash
# start_sitl.sh — ArduRover SITL that looks EXACTLY like the boat from ROS.
#
#     bash tools/sitl/start_sitl.sh              # start
#     bash tools/sitl/start_sitl.sh --stop       # stop
#     bash tools/sitl/start_sitl.sh --status     # what is running
#
# Runs on a laptop, headless, no GPU, no Gazebo. Afterwards `udp:127.0.0.1:14551` is an ArduRover
# 4.6.3 that telemetry_bridge cannot tell from the Pixhawk, so the guided-setpoint path, the mode
# latch and OBSTACLE_DISTANCE avoidance are testable without the water.
#
# The port map mirrors scripts/start_mavproxy.sh exactly, so a script written against SITL runs
# against the real vehicle unchanged:
#
#     14551   telemetry_bridge      NEVER take this port
#     14550   ad-hoc tooling        preflight, param_guard --live, a scratch QGC
#
# TWO FLAGS THAT ARE NOT STYLE:
#
#   --streamrate=-1   without it MAVProxy stomps SR0_* back to 4 Hz on every reconnect.
#   SET_MESSAGE_INTERVAL 0 200000
#                     HEARTBEAT is on a fixed 1 Hz timer no SRx_* parameter controls; 200000 us =
#                     5 Hz, which the OCS link needs to sample at 2 Hz honestly. RUNTIME state: it
#                     does not survive an autopilot reboot, same as the boat.
#
# WHAT SITL IS NOT. The frame is `motorboat`, a skid-steer hull, NOT Crusader's 4xT200 OmniX;
# FRAME_CLASS=2/FRAME_TYPE=2 here would mix for four thrusters the physics model lacks and the boat
# would sit still. So SITL proves the PLUMBING (modes, GUIDED acceptance, the type mask, avoidance,
# the behaviour tree), not thruster mixing or lateral movement. The yaw source differs too: SITL uses
# its compass, the boat COMPASS_USE=0 with GPS yaw; simulating a moving-baseline pair buys nothing
# for mission logic and is deliberately not attempted.
set -euo pipefail

AP="${ARDUPILOT_DIR:-$HOME/ardupilot}"
# Marina Bay, near enough to the 2026 venue that logs read sensibly.
HOME_LL="${SITL_HOME:-1.28060,103.85570,0,0}"
FRAME="${SITL_FRAME:-motorboat}"

stop() {
  pkill -f "sim_vehicle" 2>/dev/null || true
  pkill -f "ardurover" 2>/dev/null || true
  pkill -f "mavproxy" 2>/dev/null || true
  sleep 1
  echo "stopped."
}

status() {
  echo "ardurover : $(pgrep -c -f 'bin/ardurover' 2>/dev/null || echo 0) process(es)"
  echo "mavproxy  : $(pgrep -c -f 'mavproxy' 2>/dev/null || echo 0) process(es)"
  echo "endpoints : udp:127.0.0.1:14551 (ROS)   udp:127.0.0.1:14550 (tooling)"
  local w; w="$(ip route 2>/dev/null | awk '/^default/ {print $3; exit}')"
  [[ -n "$w" ]] && echo "            udp:$w:14550 (QGC on the Windows host)"
}

case "${1:-start}" in
  --stop) stop; exit 0 ;;
  --status) status; exit 0 ;;
esac

[[ -x "$AP/build/sitl/bin/ardurover" ]] || {
  echo "ERROR: no ardurover binary at $AP/build/sitl/bin/ardurover" >&2
  echo "       cd $AP && ./waf configure --board sitl && ./waf rover" >&2
  exit 1
}

# Version check, loud: a SITL on a different release than the boat answers questions about a vehicle
# nobody is flying. Read from the binary's strings, NOT --help, which prints usage only and never the
# version (the check would "fail" on every healthy build).
ver="$(strings "$AP/build/sitl/bin/ardurover" 2>/dev/null | grep -oE 'ArduRover V[0-9.]+' | sort -u | head -1 || true)"
echo "== $ver  (the boat runs ArduRover V4.6.3) =="
[[ "$ver" == "ArduRover V4.6.3" ]] || echo "   WARNING: version differs from the boat."

stop
cd "$AP"
echo "== starting SITL: frame $FRAME, home $HOME_LL =="
# SITL_EXTRA_ARGS: extra sim_vehicle.py flags, word-split on purpose. Empty for
# the headless rig. crusader_sim/scripts/gz_sim_up.sh uses it to put this SAME
# autopilot and port map behind Gazebo:
#   --model JSON --use-dir <dir> -w --add-param-file=<crusader_gz.parm>
# shellcheck disable=SC2086
nohup python3 Tools/autotest/sim_vehicle.py -v Rover -f "$FRAME" \
  --no-rebuild --no-mavproxy -l "$HOME_LL" ${SITL_EXTRA_ARGS:-} \
  > /tmp/sitl.log 2>&1 &
sleep 10

# QGC on the Windows side. WSL2 is NAT'd, so the Windows host is WSL's default
# gateway — resolved dynamically because that address changes when WSL restarts.
# QGC listens on UDP 14550 by DEFAULT and auto-connects, so pushing telemetry to
# it here means nobody has to add a comm link by hand.
WIN_HOST="$(ip route 2>/dev/null | awk '/^default/ {print $3; exit}')"
QGC_OUT=""
if [[ -n "$WIN_HOST" ]]; then
  QGC_OUT="--out udp:${WIN_HOST}:14550"
  echo "== QGC output -> ${WIN_HOST}:14550 (Windows host) =="
fi

echo "== starting MAVProxy on the boat's port map =="
# shellcheck disable=SC2086  # QGC_OUT must word-split, or be absent entirely
nohup mavproxy.py --master tcp:127.0.0.1:5760 \
  --out udp:127.0.0.1:14551 \
  --out udp:127.0.0.1:14550 \
  $QGC_OUT \
  --streamrate=-1 \
  --cmd="long SET_MESSAGE_INTERVAL 0 200000" \
  --daemon --non-interactive \
  > /tmp/mavproxy.log 2>&1 &
sleep 8

status
echo
echo "logs: /tmp/sitl.log  /tmp/mavproxy.log"
echo "next: python3 tools/sitl/check_sitl.py"
