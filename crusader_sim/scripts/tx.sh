#!/usr/bin/env bash
# tx.sh - the pilot's transmitter in the Gazebo sim (there is no RadioMaster in a
# sim). Runs in: WSL2 Ubuntu-22.04; with the aliases (sim_aliases.sh) it is `tx ...`.
#
#   tx keys                  ALL OF IT ON THE KEYBOARD: SC, SD, SB, SE, arm, the sticks
#   tx arm | disarm          arm / disarm (on the water: the pilot, with SB)
#   tx manual | hold | auto  the SC mode switch (the Task 3 slip and fire trees run in MANUAL)
#   tx guided                GUIDED (on the water: QGC or the ground station, not a switch)
#   tx estop on | off        SB: the motors' e-stop (it does NOT stop the pump)
#   tx drop on | off         SD: the autonomy-drop latch (on = trip it: the tree lets go)
#   tx show                  the switches now, and the flight mode
#
# The switches are real RC channels into the autopilot (crusader_sim.sim_transmitter),
# so the mode switch, the e-stop and the drop latch behave as they do on the boat.
HERE="$(cd "$(dirname "$0")" && pwd)"
WIN_SRC="$(cd "$HERE/../.." && pwd)"
CONTAINER="${RX26_CONTAINER:-crsd-sim}"
export PYTHONPATH="$WIN_SRC/crusader_sim${PYTHONPATH:+:$PYTHONPATH}"
TX="python3 -m crusader_sim.sim_transmitter"

in_rig() {    # run python in the container, with ROS
  docker exec -i "$CONTAINER" bash -lc "source /opt/ros/humble/setup.bash && source /root/robotx_ws/install/setup.bash && $1"
}
fcu_mode() {
  in_rig "timeout 4 ros2 topic echo --once /crsd/fcu_status 2>/dev/null | grep -E '^(mode|armed):' | tr '\n' ' '"
  echo
}

cmd="${1:-help}"
[ $# -gt 0 ] && shift
case "$cmd" in
  arm)
    in_rig "python3 -c 'from crusader_sim.operator_tools import arm; ok, why = arm(); print(why)'"
    ;;
  disarm)
    in_rig "python3 - <<'PY'
from pymavlink import mavutil
m = mavutil.mavlink_connection('udpin:127.0.0.1:14550', source_system=255)
m.wait_heartbeat(timeout=10)
m.arducopter_disarm()
ack = m.recv_match(type='COMMAND_ACK', blocking=True, timeout=3)
print('disarmed' if ack is not None and ack.result == 0 else 'disarm: no ACK')
PY"
    ;;
  keys)
    exec python3 -m crusader_sim.tx_keys
    ;;
  guided)
    in_rig "ros2 topic pub --once /crsd/set_mode std_msgs/msg/String '{data: GUIDED}' >/dev/null"
    sleep 1.5; echo -n "GUIDED | "; fcu_mode
    ;;
  manual|hold|auto)
    # a mode switch only acts on a CHANGE: flick it through another position first,
    # as a pilot would if the switch were already there
    other=hold; [ "$cmd" = hold ] && other=manual
    $TX set mode "$other" >/dev/null; sleep 0.6
    $TX set mode "$cmd" >/dev/null; sleep 1.5
    echo -n "SC -> $cmd | "; fcu_mode
    ;;
  estop|drop)
    case "${1:-}" in on|off) ;; *) echo "tx $cmd on|off" >&2; exit 2 ;; esac
    $TX set "$cmd" "$1"
    ;;
  show)
    $TX show
    echo -n "flight controller: "; fcu_mode
    ;;
  *)
    sed -n '2,16p' "$0" | sed 's/^# \{0,1\}//'
    ;;
esac
