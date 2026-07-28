"""camera.launch.py — OAK-D LR perception only (no LiDAR, no fusion).

Brings up perception_node: OAK-D LR capture -> TensorRT detect -> depth
associate -> /crsd/detections_body. Use this to bench-test the camera path on
its own; downstream consumers (frame_transform, gate_navigator, dp_hold) read
/crsd/detections_body directly when no fusion node is running.

  ros2 launch rx26_asv camera.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("rx26_asv"),
        "config", "crusader_params.yaml")

    return LaunchDescription([
        Node(package="rx26_asv", executable="perception_node",
             output="screen", parameters=[params]),
    ])
