"""core.launch.py — the always-on Crusader status/telemetry stack.

Starts (all reading config/crusader_params.yaml from the package share):
  * telemetry_bridge      — the single ROS-side gateway to MAVProxy's rebroadcast
                            (/crsd/pose, /crsd/fcu_status, /crsd/rc_channels,
                             /crsd/autonomy_drop; sanctioned override/setpoint TX)
  * led_node              — /crsd/led_state -> LED Arduino serial
  * pixhawk_led_status    — autopilot + RC state -> /crsd/led_state

This differs from the boat repo's core launch (led + pixhawk_led only): here the
LED-status node consumes telemetry_bridge topics instead of opening its own
MAVLink connection, so the bridge must be up for the LEDs to reflect real state.
MAVProxy itself (the sole Pixhawk owner) is started outside ROS — see
scripts/start_mavproxy.sh — before this launch.

  ros2 launch robotx_2026 core.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory("robotx_2026"),
        "config", "crusader_params.yaml")

    return LaunchDescription([
        Node(package="robotx_2026", executable="telemetry_bridge",
             output="screen", parameters=[params]),
        Node(package="robotx_2026", executable="led_node",
             output="screen", parameters=[params]),
        Node(package="robotx_2026", executable="pixhawk_led_status_node",
             output="screen", parameters=[params]),
    ])
