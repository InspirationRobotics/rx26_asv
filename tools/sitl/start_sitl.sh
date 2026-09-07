#!/usr/bin/env bash
# start_sitl.sh — ArduRover SITL that looks EXACTLY like the boat from ROS.
#
#     bash tools/sitl/start_sitl.sh              # start
#     bash tools/sitl/start_sitl.sh --stop       # stop
#     bash tools/sitl/start_sitl.sh --status     # what is running
#
# Runs on a laptop, headless, no GPU, no Gazebo. The point is that after this
# script, `udp:127.0.0.1:14551` is an ArduRover 4.6.3 that telemetry_bridge
# cannot tell from the Pixhawk — so the whole guided-setpoint path, the mode
# latch and OBSTACLE_DISTANCE avoidance become testable without the water.
#
# WHY THE PORT MAP IS COPIED RATHER THAN SIMPLIFIED. It mirrors
# scripts/start_mavproxy.sh exactly:
#
#     14551   telemetry_bridge      NEVER take this port
#     14550   ad-hoc tooling        preflight, param_guard --live, a scratch QGC
#
# A script written against SITL therefore runs against the real vehicle
# unchanged, and vice versa. Two ports that differ by one digit between the sim
# and the boat is a debugging session nobody needs.
#
# TWO FLAGS THAT ARE NOT STYLE, both learned on the boat:
#
#   --streamrate=-1   without it MAVProxy stomps SR0_* back to 4 Hz on every
#                     reconnect, and the ROS-side rates quietly halve.
#   SET_MESSAGE_INTERVAL 0 200000
#                     HEARTBEAT is on a fixed 1 Hz timer that no SRx_* parameter
#                     controls. 200000 us = 5 Hz, which is what the OCS link
#                     needs to sample at 2 Hz honestly. It is RUNTIME state and
#                     does not survive an autopilot reboot -- same as the boat.
#
# WHAT SITL IS NOT. The frame is `motorboat`, a skid-steer hull, NOT Crusader's
# 4xT200 OmniX. Setting FRAME_CLASS=2/FRAME_TYPE=2 here would make ArduRover mix
# for four thrusters the physics model does not have, and the boat would sit
# still. So SITL proves the PLUMBING -- modes, GUIDED acceptance, the type mask,
# avoidance, the behaviour tree -- and says nothing about thruster mixing or
# lateral movement. Those need the real hull.
#
# The yaw source differs too: SITL uses its compass, while the boat runs
# COMPASS_USE=0 with GPS yaw. Simulating a moving-baseline pair is a rabbit hole
# and buys nothing for mission logic, so it is deliberately not attempted.
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

# Version check, loud. A SITL on a different release than the boat is worse than
# no SITL: it answers questions about a vehicle nobody is flying.
ver="$("$AP/build/sitl/bin/ardurover" --help 2>&1 | grep -oE 'ArduRover V[0-9.]+' | head -1 || true)"
echo "== $ver  (the boat runs ArduRover V4.6.3) =="
[[ "$ver" == "ArduRover V4.6.3" ]] || echo "   WARNING: version differs from the boat."

stop
cd "$AP"
echo "== starting SITL: frame $FRAME, home $HOME_LL =="
nohup python3 Tools/autotest/sim_vehicle.py -v Rover -f "$FRAME" \
  --no-rebuild --no-mavproxy -l "$HOME_LL" \
  > /tmp/sitl.log 2>&1 &
sleep 10

echo "== starting MAVProxy on the boat's port map =="
nohup mavproxy.py --master tcp:127.0.0.1:5760 \
  --out udp:127.0.0.1:14551 \
  --out udp:127.0.0.1:14550 \
  --streamrate=-1 \
  --cmd="long SET_MESSAGE_INTERVAL 0 200000" \
  --daemon --non-interactive \
  > /tmp/mavproxy.log 2>&1 &
sleep 8

status
echo
echo "logs: /tmp/sitl.log  /tmp/mavproxy.log"
echo "next: python3 tools/sitl/check_sitl.py"
