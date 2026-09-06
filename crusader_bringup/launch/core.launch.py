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
  * lidar_cluster_node    — crusader_perception; MID360 cloud -> crsd/lidar_clusters
  * proximity_bridge      — crusader_perception; clusters -> OBSTACLE_DISTANCE,
                            which is what feeds the autopilot's own avoidance
  * lidar_view            — tools/lidar_view.py; plan + elevation MJPEG on :8081,
                            the ground station's LiDAR tab

A NOTE ON lidar_view, because the dependency runs the OTHER WAY round from how
it reads. lidar_cluster_node does NOT need it: the cluster node subscribes to
/livox/lidar, /crsd/pose and /crsd/attitude and would run identically with the
viewer absent. What is true is a CODE dependency in the opposite direction —
tools/lidar_view.py imports the transform and the filters from
lidar_cluster_core, so the picture on :8081 is drawn through the same maths the
clusters are, which is exactly what makes it worth having at boot: it is the
only way to SEE what the cluster node is deciding.

It is launched because it is nearly free when nobody is looking. Every panel is
gated on FrameBuffer.has_clients, so with no browser attached it decodes nothing
and encodes no JPEG. That is the same optimisation buoy_detector uses, and the
same reason its health line reads ~0 fps with viewers=0.

THE LIDAR NOW GATES ARMING, and that is worth knowing before it surprises anyone
on a dock. With PRX1_TYPE=2 the autopilot expects a MAVLink proximity source and
refuses to arm without one:

    PreArm: PRX1: No Data

Those last two nodes are the source. They are in this launch precisely so that
message does not appear on a healthy boat — it was exactly what happened on
2026-09-05, when a power cycle left them behind because they were hand-started.
The flip side is the coupling: if the MID360 is unpowered, or crsd-livox is down,
or clusters stop, the boat cannot arm. That is the correct behaviour for a
vehicle configured to rely on avoidance, and the escape hatch is deliberate and
explicit rather than automatic:

    PRX1_TYPE = 0      # no proximity source; arming stops depending on it

Check the LiDAR before reaching for it — `ros2 topic hz /livox/lidar` should read
10 Hz, and `ros2 topic echo /crsd/proximity_health --once` says what the bridge
thinks it is seeing.

World-model nodes are NOT here: that package is scaffolded and empty (see its
README). Add a node to this launch once it has run on the boat, not when it
compiles.

`crusader_perception`'s CAMERA nodes (`oakd_publisher`, `buoy_detector`,
`oak_detector`) run in `asv` like everything else here, but are still NOT in this
launch: they contend for the same OAK-D and the device admits exactly one client.
Which one runs is an operator choice per session — frames for a human, or
detections for the stack — so it cannot be a constant in a launch file. Start the
one you want by hand. Its LiDAR nodes are different: nothing else competes for
the MID360, so they can be constants, and the paragraph above is why they must
be.

MAVProxy itself (the sole Pixhawk owner) is started outside ROS by systemd — see
scripts/start_mavproxy.sh — before this launch.

  ros2 launch crusader_bringup core.launch.py
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node


def generate_launch_description():
    # The same file crusader_common/config.py resolves for declaration defaults.
    # Both paths must point at one file, or a node's declared default and its
    # launched value can disagree — which is invisible until behaviour differs.
    params = os.path.join(
        get_package_share_directory("crusader_bringup"),
        "config", "crusader_params.yaml")

    # tools/ is NOT installed into share — the viewers are plain scripts run from
    # the source tree. Read the path OUT OF THE PARAMS FILE rather than deriving
    # it: `ground_station.tools_dir` is the same value process_manager already
    # uses to start these scripts from the web UI, so the two ways of launching
    # lidar_view cannot point at different files.
    #
    # Explicitly NOT computed from the share dir. In the installed layout this
    # lives in install/crusader_bringup/share/crusader_bringup while tools/ is in
    # src/rx26_asv/tools, and no fixed number of ".." reaches it from here — the
    # same path arithmetic crusader_common/config.py warns about at length, and
    # which I got wrong once writing this file.
    with open(params, encoding="utf-8") as f:
        tools_dir = yaml.safe_load(f)["ground_station"]["ros__parameters"][
            "tools_dir"]

    return LaunchDescription([
        Node(package="crusader_fcu", executable="telemetry_bridge",
             output="screen", parameters=[params]),
        Node(package="crusader_behavior", executable="pixhawk_led_status_node",
             output="screen", parameters=[params]),
        Node(package="crusader_behavior", executable="led_node",
             output="screen", parameters=[params]),
        Node(package="crusader_behavior", executable="rc_watchdog",
             output="screen", parameters=[params]),
        Node(package="crusader_groundstation", executable="ground_station",
             output="screen", parameters=[params]),
        # The proximity chain, in dependency order. Both take the params file
        # for the reason at the top of generate_launch_description: a node's
        # declared default and its launched value disagreeing is invisible until
        # behaviour differs.
        Node(package="crusader_perception", executable="lidar_cluster_node",
             output="screen", parameters=[params]),
        Node(package="crusader_perception", executable="proximity_bridge",
             output="screen", parameters=[params]),
        # A plain script, not a ROS entry point — hence ExecuteProcess rather
        # than Node. It reads the LiDAR extrinsic out of the params file itself
        # (see tools/lidar_view.py), so it takes no `parameters=`.
        ExecuteProcess(
            cmd=["python3", os.path.join(tools_dir, "lidar_view.py")],
            output="screen"),
    ])
