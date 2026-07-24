"""lidar_fusion.launch.py — full fused perception stack: camera + LiDAR + fusion.

Composes the two single-sensor launches (so the sensors stay independently
runnable — see camera.launch.py / lidar.launch.py) and adds the fusion node:

  camera.launch.py  -> perception_node        -> /crsd/detections_body
  lidar.launch.py   -> livox_ros_driver2_node -> /livox/lidar
  lidar_fusion_node -> fuses both             -> /crsd/detections_fused

Fusion is additive and optional: if you instead run only camera.launch.py, the
downstream consumers (frame_transform, ...) fall back to /crsd/detections_body
via DetectionInput. This launch is the "everything on" convenience form.

  ros2 launch robotx_2026 lidar_fusion.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("robotx_2026")
    params = os.path.join(share, "config", "crusader_params.yaml")
    launch_dir = os.path.join(share, "launch")

    def include(name):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, name)))

    return LaunchDescription([
        include("camera.launch.py"),
        include("lidar.launch.py"),
        Node(package="robotx_2026", executable="lidar_fusion_node",
             output="screen", parameters=[params]),
    ])
