"""core.launch.py — the always-on Crusader status/telemetry stack.

Starts (all reading config/crusader_params.yaml from THIS package's share dir):
  * telemetry_bridge      — crusader_fcu; the single ROS-side gateway to
                            MAVProxy's rebroadcast (/crsd/pose, /crsd/fcu_status,
                            /crsd/rc_channels, /crsd/autonomy_drop; the sanctioned
                            force-disarm and RC-override TX paths)
  * pixhawk_led_status    — crusader_behavior; autopilot + RC state -> /crsd/led_state
  * led_node              — crusader_behavior; /crsd/led_state -> LED Arduino serial
  * rc_watchdog           — crusader_behavior; force-disarm on RC-transmitter link
                            loss; consumes telemetry_bridge topics and routes the
                            disarm back through the bridge (/crsd/force_disarm)

Perception and world-model nodes are NOT here: those packages are scaffolded and
empty (see their READMEs). Add a node to this launch once it has run on the boat,
not when it compiles.

`crusader_sensors/oakd_publisher` is not here either, for a different reason: it
owns the OAK-D and runs in the SENSOR container. This launch runs in `asv`, which
has no depthai by design, so starting it from here could only fail. bringup still
exec_depends on that package so it is built and checked with the rest.

MAVProxy itself (the sole Pixhawk owner) is started outside ROS by systemd — see
scripts/start_mavproxy.sh — before this launch.

  ros2 launch crusader_bringup core.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # The same file crusader_common/config.py resolves for declaration defaults.
    # Both paths must point at one file, or a node's declared default and its
    # launched value can disagree — which is invisible until behaviour differs.
    params = os.path.join(
        get_package_share_directory("crusader_bringup"),
        "config", "crusader_params.yaml")

    return LaunchDescription([
        Node(package="crusader_fcu", executable="telemetry_bridge",
             output="screen", parameters=[params]),
        Node(package="crusader_behavior", executable="pixhawk_led_status_node",
             output="screen", parameters=[params]),
        Node(package="crusader_behavior", executable="led_node",
             output="screen", parameters=[params]),
        Node(package="crusader_behavior", executable="rc_watchdog",
             output="screen", parameters=[params]),
    ])
