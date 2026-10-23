"""core.launch.py: the always-on Crusader stack, every node reading config/crusader_params.yaml.

  telemetry_bridge  crusader_behavior: the one MAVLink gateway (MAVProxy's rebroadcast)
  led_status_node   crusader_behavior: autopilot + RC state -> /crsd/led_state
  led_node          crusader_behavior: /crsd/led_state -> the LED Arduino
  rc_watchdog       crusader_behavior: force-disarm on RC link loss
  gcs_bridge        crusader_groundstation: the laptop GCS connects here (tcp 14559)
  lidar_node        crusader_perception: clusters, the Nav2 cloud, OBSTACLE_DISTANCE
  camera_node       crusader_perception: the OAK-D's one owner, with the model the YAML names.
                    No respawn, so the Nodes tab can stop it to hand the camera to something
                    else; with no OAK-D at boot it exits and the Nodes tab starts it later
  rfd_link_node     crusader_link: the RFD900 mesh to Ekko. Respawns, so a radio plugged in
                    after boot is picked up; never also start it by hand (two readers on one
                    serial port each get half the bytes)

lidar_node is here because with PRX1_TYPE=2 the autopilot refuses to arm without a proximity
source ("PreArm: PRX1: No Data"): on 2026-09-05 a power cycle left a hand-started LiDAR chain
behind. The escape hatch is PRX1_TYPE=0, never an automatic one.

Not here: target_tracker and the nav stack (nav.launch.py). A node joins this launch once it
has run on the boat.
MAVProxy, the sole Pixhawk owner, is started by systemd before this (scripts/start_mavproxy.sh).

  ros2 launch crusader_bringup core.launch.py
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(get_package_share_directory("crusader_bringup"),
                          "config", "crusader_params.yaml")

    def node(package, executable, **kw):
        return Node(package=package, executable=executable, output="screen",
                    parameters=[params], **kw)

    return LaunchDescription([
        node("crusader_behavior", "telemetry_bridge"),
        node("crusader_behavior", "led_status_node"),
        node("crusader_behavior", "led_node"),
        node("crusader_behavior", "rc_watchdog"),
        node("crusader_groundstation", "gcs_bridge"),
        node("crusader_perception", "lidar_node"),
        node("crusader_perception", "camera_node"),
        node("crusader_link", "rfd_link_node", respawn=True, respawn_delay=5.0),
    ])
