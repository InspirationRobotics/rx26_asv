#!/usr/bin/env bash
# core_standin.sh — in crsd-sim: what core.launch.py + the OAK-D give the boat, so lake_rig_up.sh has something to add to.
#   gz bridge, livox_shim, sim_camera (= the detector), telemetry_bridge, lidar_cluster_node, ground_station.
# NOT started (lake_rig_up.sh's job): rxl_link_node, target_tracker, nav, bt_view, panel_feed, bt_runner, the panel.
source /opt/ros/humble/setup.bash
source /root/robotx_ws/install/setup.bash
set -uo pipefail
COURSE="${1:-task1_core}"
WS=/root/robotx_ws
SRC=$WS/src/rx26_asv
CFG=$WS/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml
SIMSHARE=$WS/install/crusader_sim/share/crusader_sim
export GZ_PARTITION=crusader_sim
export RX26_SRC=$SRC
# the sim rig's nodes down first (gz and SITL stay up)
bash "$SRC/crusader_sim/scripts/gz_rig_down.sh" >/dev/null 2>&1
sleep 1
up() { local name=$1 log=$2; shift 2; nohup "$@" > "$log" 2>&1 &
       printf '  %-20s -> %s\n' "$name" "$log"; }
up gz_bridge /tmp/gzb.log \
  ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$SIMSHARE/config/gz_bridge.yaml"
up livox_shim /tmp/livox_shim.log ros2 run crusader_sim livox_shim
up sim_camera /tmp/sim_camera.log ros2 run crusader_sim sim_camera --ros-args -p course:="$COURSE"
up telemetry_bridge /tmp/tb.log \
  ros2 run crusader_fcu telemetry_bridge --ros-args --params-file "$CFG" -p mav_endpoint:=udp:127.0.0.1:14551
up lidar_cluster_node /tmp/lidar_cluster.log \
  ros2 run crusader_perception lidar_cluster_node --ros-args --params-file "$CFG"
up ground_station /tmp/gcs.log \
  ros2 run crusader_groundstation ground_station --ros-args --params-file "$CFG"
sleep 8
ros2 node list 2>/dev/null | sort | sed 's/^/  /'
