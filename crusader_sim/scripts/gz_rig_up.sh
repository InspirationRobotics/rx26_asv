#!/usr/bin/env bash
# gz_rig_up.sh — the ROS half of the Gazebo sim. Runs in: the crsd-sim container.
#
#     bash crusader_sim/scripts/gz_rig_up.sh [course] [--no-uav]
#
# Started by gz_sim_up.sh (WSL) after Gazebo and SITL are up. It is
# tools/sitl/task1_sim_up.sh with ONE substitution: where that rig runs
# bench_world_model (an invented buoy field and invented detections), this one
# runs the simulator's sensors through the boat's own perception:
#
#     gz  --ros_gz_bridge-->  /sim/*  --livox_shim-->  /livox/lidar
#                                         --> lidar_cluster_node (the boat's) --> crsd/lidar_clusters
#                             /sim/*  --sim_camera-->  oak/rgb, oak/depth, crsd/oak/detections
#     sim_uav  --RXL udp 14555-->  rxl_link_node (the boat's)
#
# Everything downstream — telemetry_bridge, rxl_link_node, target_tracker,
# ground_station, bt_runner — is the boat's code, unmodified, same params file.
# ROS's setup.bash reads unset variables, so strict mode goes on AFTER it
source /opt/ros/humble/setup.bash
source /root/robotx_ws/install/setup.bash
set -uo pipefail

COURSE="${1:-task1_core}"
UAV=1
[ "${2:-}" = "--no-uav" ] && UAV=0
WS=/root/robotx_ws
SRC=$WS/src/rx26_asv
CFG=$WS/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
SIMSHARE=$WS/install/crusader_sim/share/crusader_sim
BT=$WS/install/crusader_bt/share/crusader_bt/behavior_trees
TREE="${TREE:-$BT/task1_disruptive.xml}"
export GZ_PARTITION=crusader_sim
export RX26_SRC=$SRC

bash "$SRC/crusader_sim/scripts/gz_rig_down.sh" >/dev/null 2>&1
sleep 1

up() { local name=$1 log=$2; shift 2; nohup "$@" > "$log" 2>&1 &
       printf '  %-20s -> %s\n' "$name" "$log"; }

echo "=== sensors (Gazebo -> the boat's topics) ==="
up gz_bridge /tmp/gzb.log \
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SIMSHARE/config/gz_bridge.yaml"
up livox_shim /tmp/livox_shim.log ros2 run crusader_sim livox_shim
up sim_camera /tmp/sim_camera.log \
  ros2 run crusader_sim sim_camera --ros-args -p course:="$COURSE"

echo "=== the boat's stack ==="
up telemetry_bridge /tmp/tb.log \
  ros2 run crusader_fcu telemetry_bridge --ros-args --params-file "$CFG" \
  -p mav_endpoint:=udp:127.0.0.1:14551
up lidar_cluster_node /tmp/lidar_cluster.log \
  ros2 run crusader_perception lidar_cluster_node --ros-args --params-file "$CFG"
# rxl_endpoint on this branch is the RFD900 (/dev/crsd-rfd); the params file's
# own comment gives the UDP form for a bench/sim, which is what sim_uav talks to
up rxl_link_node /tmp/rxl.log \
  ros2 run crusader_link rxl_link_node --ros-args --params-file "$CFG" \
  -p rxl_endpoint:=udpin:127.0.0.1:14555
if [ "$UAV" = 1 ]; then
  up sim_uav /tmp/sim_uav.log python3 -u -m crusader_sim.sim_uav --course "$COURSE"
else
  echo "  sim_uav              -- off (--no-uav): the boat is on its own camera"
fi
up target_tracker /tmp/tt.log \
  ros2 run crusader_world_model target_tracker --ros-args --params-file "$CFG"
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
echo "  tree: $(basename "$TREE")   course: $COURSE"
timeout 5 ros2 topic echo /crsd/fcu_status --once 2>/dev/null \
  | grep -E '^mode|^armed' | tr '\n' ' ' | sed 's/^/  /'; echo
