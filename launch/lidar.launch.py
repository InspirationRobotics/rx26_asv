"""lidar.launch.py — Livox MID360 driver only (no camera, no fusion).

Brings up livox_ros_driver2_node, which owns the MID360 (Ethernet/UDP) and
publishes /livox/lidar as PointCloud2. Use this to bench-test the LiDAR on its
own — inspect the raw cloud (e.g. `ros2 topic hz /livox/lidar`, rviz2) before
wiring it into fusion.

The driver's ROS params come from the `livox_lidar` section of
config/crusader_params.yaml (single source of truth). The network/IP layer stays
in config/MID360_config.json — the driver requires its own JSON via
`user_config_path`, which this launch sets to the package's installed share copy
(NOT the copy baked into /opt/livox_ws at image build). The MID360 must be on its
own NIC — set the host/LiDAR IPs in that JSON to the interface's subnet.

Requires livox_ros_driver2 on the ROS path (baked into the crusader image).

  ros2 launch robotx_2026 lidar.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robotx_2026")
    params = os.path.join(share, "config", "crusader_params.yaml")
    mid360_cfg = os.path.join(share, "config", "MID360_config.json")

    return LaunchDescription([
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar",
            output="screen",
            # driver ROS params from crusader_params.yaml (livox_lidar section);
            # user_config_path is computed here (dynamic share path) and overrides.
            parameters=[params, {"user_config_path": mid360_cfg}],
        ),
    ])
