#!/usr/bin/env bash
# status.sh - one look at everything a Task 3 run depends on. Changes nothing.
#
#   docker exec asv /root/robotx_ws/src/rx26_asv/tools/scripts/task3/status.sh
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/env.sh"

get() { ros2 param get /bt_runner_node "$1" 2>/dev/null | awk '{print $NF}'; }
if pgrep -f "[c]rusader_bt/bt_runner_node" >/dev/null; then
  echo "bt_runner: RUNNING  publish_setpoints=$(get publish_setpoints)  fire_pump=$(get fire_pump)  tree=$(basename "$(get tree_file)")"
else
  echo "bt_runner: not running (tree.sh starts it)"
fi

F=$(timeout 4 ros2 topic echo --once /crsd/fcu_status 2>/dev/null)
echo "mode: $(echo "$F" | awk '/^mode:/{print $2}')   armed: $(echo "$F" | awk '/^armed:/{print $2}')"

D=$(timeout 4 ros2 topic echo --once --qos-durability transient_local --qos-reliability reliable \
      /crsd/autonomy_drop 2>/dev/null | awk '/^data:/{print $2}')
echo "drop latch tripped: ${D:-unknown (is telemetry_bridge up?)}"

CH9=$(timeout 3 ros2 topic echo --once /crsd/rc_channels 2>/dev/null | tr '\n' ' ' \
        | grep -oE 'channels:( - [0-9]+){9}' | grep -oE '[0-9]+$')
if [ -z "$CH9" ]; then
  echo "SD (ch9): no RC data"
elif [ "$CH9" -lt 1700 ]; then
  echo "SD (ch9): $CH9 - down"
else
  echo "SD (ch9): $CH9 - UP: the latch is held, and go.sh's reset will be refused"
fi

W=$(timeout 3 ros2 topic echo --once /crsd/wall_range 2>/dev/null | tr '\n' ' ' \
      | grep -oE '(valid|range_m|angle_deg|lat_m): [^ ]+' | tr '\n' ' ')
echo "wall: ${W:-none - wall_range_node is not running or sees no wall}"

H=$(timeout 3 ros2 topic echo --once /crsd/dock_view_health std_msgs/msg/String 2>/dev/null \
      | grep -oE '"(fps|det_ms|total_ms)": [0-9.]+' | tr -d '"' | tr '\n' ' ')
echo "dock_view: ${H:-no health report - is it running?}"

for n in "[w]all_range_node" "[d]ock_view.py"; do
  pgrep -f "$n" >/dev/null && echo "${n//[\[\]]/}: up" || echo "${n//[\[\]]/}: DOWN"
done
