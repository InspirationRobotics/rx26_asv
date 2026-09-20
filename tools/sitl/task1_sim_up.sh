#!/bin/bash
# Bring up the whole Task 1 Disruptive rig against a running SITL.
#
# Runs INSIDE the crsd-sim container. Start SITL first:
#
#   (WSL)      bash tools/sitl/start_sitl.sh
#   (Windows)  bash tools/sitl/sim.sh 'bash src/rx26_asv/tools/sitl/task1_sim_up.sh'
#
# Then three browser tabs on the machine running it:
#
#   :8086   the aircraft   place buoys, author gates, Transmit, recolour
#   :8085   the tree       which leaf is RUNNING, what just failed
#   :8090   the boat       map, pose, fused targets
#
# It does NOT send a goal. Place a passage and hit Transmit first -- the guard
# band refuses to transit before the aircraft has reported, which is the
# handbook rule and not a bug. Then:
#
#   ros2 action send_goal /crsd/safe_passage crusader_msgs/action/SafePassage \
#     '{tier: 2, timeout_s: 400.0}' -f
#
# publish_setpoints is TRUE here and that is safe by construction: the only
# vehicle listening is the simulator.
source /opt/ros/humble/setup.bash
source /root/robotx_ws/install/setup.bash

WS=/root/robotx_ws
CFG=$WS/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
BT=$WS/install/crusader_bt/share/crusader_bt/behavior_trees
SRC=$WS/src/rx26_asv
TREE="${TREE:-$BT/task1_disruptive.xml}"

# Clear whatever the last run left up. The patterns live in rig_processes.txt
# and nodes_down.sh is the same script sim_down.sh runs, so a node added to this
# file cannot be forgotten on the shutdown path.
bash "$SRC/tools/sitl/nodes_down.sh"
sleep 1

# IS ANYTHING RUNNING FROM A STALE install/? Nodes import from install/, never
# from src/, and this script deliberately does NOT build - a colcon build is
# minutes and a bring-up should be seconds. The cost of that choice is that
# edited code silently does not run, and it is impossible to see from the
# outside: the node starts, logs normally, and serves the old behaviour.
#
# 2026-09-14: the ground station on :8090 was serving a page less than half the
# size of the one in src, and the param its newer code needed was sitting
# unused in crusader_bringup. Nothing was wrong with either file. Neither
# package had been rebuilt in this container for a day.
#
# The sync only rewrites files whose CONTENT differs, so an unchanged file
# keeps its mtime and this does not cry wolf after every bring-up.
newest() { find "$1" -type f -printf '%T@
' 2>/dev/null | sort -rn | head -1; }
stale=""
for d in "$SRC"/*/; do
  pkg=$(basename "$d")
  [ -d "$WS/install/$pkg" ] || continue
  src_t=$(newest "$d"); ins_t=$(newest "$WS/install/$pkg")
  [ -n "$src_t" ] && [ -n "$ins_t" ] || continue
  awk -v a="$src_t" -v b="$ins_t" 'BEGIN{exit !(a > b + 1)}' && stale="$stale $pkg"
done
if [ -n "$stale" ]; then
  echo
  echo "  *** SOURCE IS NEWER THAN install/ FOR:$stale"
  echo "  *** those nodes will run the OLD code. To fix, from Windows:"
  echo "  ***   bash tools/sitl/sim.sh 'colcon build --packages-select$stale'"
  echo
fi

up() { local name=$1 log=$2; shift 2; nohup "$@" > "$log" 2>&1 &
       printf '  %-18s -> %s\n' "$name" "$log"; }

echo "=== bringing up ==="
up telemetry_bridge /tmp/tb.log \
  ros2 run crusader_fcu telemetry_bridge --ros-args --params-file "$CFG" \
  -p mav_endpoint:=udp:127.0.0.1:14551
up rxl_link_node /tmp/rxl.log \
  ros2 run crusader_link rxl_link_node --ros-args --params-file "$CFG"
sleep 10

# GUIDED, and ARMED. IsAutonomous gates the tree on the mode; ArduRover ignores
# a setpoint unless armed. Without both the tree ticks happily and the boat
# never moves, which reads as a stuck leg. SITL is not armable the instant it
# boots, so this retries the way check_sitl.py does.
ros2 topic pub --once /crsd/set_mode std_msgs/msg/String '{data: GUIDED}' >/dev/null 2>&1
python3 -u - <<'ARM'
import time
from pymavlink import mavutil
c = mavutil.mavlink_connection("udpin:127.0.0.1:14550")
c.wait_heartbeat(timeout=20)
t0, armed = time.time(), False
while time.time() - t0 < 60 and not armed:
    c.mav.command_long_send(c.target_system, c.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0, 0, 0, 0, 0)
    end = time.time() + 3
    while time.time() < end:
        m = c.recv_match(type="COMMAND_ACK", blocking=False)
        if m and m.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
            armed = (m.result == 0)
            break
        time.sleep(0.05)
print("  armed after %.0fs" % (time.time() - t0) if armed
      else "  ARM FAILED -- the tree will tick but the boat will not move")
ARM

up target_tracker /tmp/tt.log \
  ros2 run crusader_world_model target_tracker --ros-args --params-file "$CFG"
up bench+aircraft /tmp/gui.log \
  python3 -u "$SRC/tools/bench/bench_world_model.py" --gui --uav
up bt_view /tmp/btview.log python3 -u "$SRC/tools/bt_view.py"
up ground_station /tmp/gcs.log \
  ros2 run crusader_groundstation ground_station --ros-args --params-file "$CFG"
sleep 6
up bt_runner /tmp/bt.log \
  ros2 run crusader_bt bt_runner_node --ros-args --params-file "$CFG" \
  -p tree_file:="$TREE" -p publish_setpoints:=true -p default_timeout_s:=400.0
sleep 6

echo
echo "=== up ==="
ros2 node list 2>/dev/null | sort | sed 's/^/  /'
echo "  tree: $(basename "$TREE")"
timeout 5 ros2 topic echo /crsd/fcu_status --once 2>/dev/null \
  | grep -E '^mode|^armed' | tr '\n' ' ' | sed 's/^/  /'; echo
echo
echo "  :8086 aircraft   :8085 tree   :8090 boat"
echo "  place a passage, Transmit, then send the goal (see the header)"
