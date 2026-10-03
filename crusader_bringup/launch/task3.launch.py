"""task3.launch.py — what Task 3 (coordinated logistics / the fixed-nozzle shot)
needs ON TOP OF core.launch.py.

    ros2 launch crusader_bringup task3.launch.py                       # shadow
    ros2 launch crusader_bringup task3.launch.py publish_setpoints:=true   # hold, dry
    ros2 launch crusader_bringup task3.launch.py publish_setpoints:=true fire_pump:=true

then start the run, SD down and the transmitter on (MANUAL for the fire trees,
GUIDED for the mission trees):

    tools/scripts/task3/go.sh        # checks, resets the drop latch, sends the goal

tools/scripts/task3/ also has tree.sh (restart just the tree, with live-gain
presets), status.sh and watch.sh; docs/T3_running.md is the whole procedure.

It is a separate step on purpose. The autonomy-drop latch (ch9 = SD) also
trips when RC data goes stale for over a second, and stays tripped until a
person resets it; a tripped latch makes the fire tree abort on its first tick.
Resetting it is the operator saying "I have the sticks", so this launch never
does it on its own. By hand, go.sh is:

    ros2 service call /crsd/autonomy_drop_reset std_srvs/srv/Trigger
    ros2 action send_goal --feedback /crsd/safe_passage crusader_msgs/action/SafePassage "{tier: 0, timeout_s: 600}"

WHAT IT STARTS, AND WHY EACH:

  wall_range_node   the LiDAR range to the dock face (/crsd/wall_range): the
                    fire tree's forward/back axis, and its guard band ends the
                    run if the wall is lost for a second.
  dock_view         the dock detector: the CV team's model + colour rule on the
                    OAK-D, publishing DockObservation (each window's x, y, z) on
                    /dock/observations - the fire tree's left/right axis. A plain
                    script, hence ExecuteProcess (see core.launch.py).
  bt_runner_node    the behaviour tree, on the Task 3 tree (default the MANUAL
                    fixed-nozzle shot, task3_fire_manual.xml).

WHAT IT DOES NOT START, ON PURPOSE:

  * core.launch.py's nodes (telemetry_bridge, ground_station, the LiDAR
    proximity chain, rxl_link_node). systemd runs core (crsd-ros); a second copy
    of any of them would fight the first for its topic or device.
  * any other OAK-D client. dock_view OWNS the camera: stop oak_detector,
    buoy_detector, oakd_publisher and oak_view first (or it fails to open the
    device), and do NOT also start dock_view from the Nodes tab while this runs.

THE TWO SWITCHES DEFAULT OFF. publish_setpoints (may the tree take the sticks,
Gate G1) and fire_pump (may it squirt, Gate G7) are false unless given here, so a
bare `ros2 launch` is a shadow run: the tree logs what it would do and touches
nothing. Turning either on is a deliberate act on the command line, never a
default in a file. See docs/T3_coordinated_logistics.md.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TREES = {
    "fire_manual": "task3_fire_manual.xml",   # MANUAL, strafes onto the window (the shot)
    "fire_test": "task3_fire_test.xml",       # GUIDED, aims by turning
    "disruptive": "task3_disruptive.xml",     # the full Task 3 mission tree
    # TEST variants (headers say how they differ):
    "fire_1p6": "task3_fire_manual_1p6.xml",  # pool tuning: 1.6 m, no attempt limit
    "approach_test": "task3_approach_test.xml",  # GUIDED approach to line-up only; dock_bays:=1
}


def _truthy(text):
    v = str(text).strip().lower()
    if v in ("true", "1", "yes", "on"):
        return True
    if v in ("false", "0", "no", "off"):
        return False
    raise ValueError(f"expected true/false, got {text!r}")


def _nodes(context):
    params = os.path.join(get_package_share_directory("crusader_bringup"),
                          "config", "crusader_params.yaml")
    # tools/ is not installed; read its path out of the params file, exactly as
    # core.launch.py does, so the page and this launch start the same script.
    with open(params, encoding="utf-8") as f:
        tools_dir = yaml.safe_load(f)["ground_station"]["ros__parameters"]["tools_dir"]

    tree = LaunchConfiguration("tree").perform(context)
    if tree not in TREES:
        raise ValueError(f"tree:={tree!r}: one of {sorted(TREES)}")
    tree_file = os.path.join(get_package_share_directory("crusader_bt"),
                             "behavior_trees", TREES[tree])
    setpoints = _truthy(LaunchConfiguration("publish_setpoints").perform(context))
    pump = _truthy(LaunchConfiguration("fire_pump").perform(context))
    bays = int(LaunchConfiguration("dock_bays").perform(context))
    if bays not in (1, 2, 3):
        raise ValueError(f"dock_bays:={bays}: 1..3 (3 on the course)")
    mode = ("FIRE (sticks + pump)" if setpoints and pump else
            "HOLD (sticks, pump DRY)" if setpoints else
            "SHADOW (logs only, touches nothing)" if not pump else
            "PUMP WITHOUT STICKS")

    return [
        LogInfo(msg=f"task3: tree {TREES[tree]} | publish_setpoints {setpoints} | "
                    f"fire_pump {pump} -> {mode}"),
        Node(package="crusader_perception", executable="wall_range_node",
             output="screen", parameters=[params]),
        ExecuteProcess(cmd=["python3", os.path.join(tools_dir, "dock_view.py")],
                       output="screen"),
        # The params file first, then the three overrides: a later entry wins, so
        # the file stays the source of truth for everything else on the node.
        Node(package="crusader_bt", executable="bt_runner_node",
             output="screen",
             parameters=[params, {"tree_file": tree_file,
                                  "publish_setpoints": setpoints,
                                  "fire_pump": pump,
                                  "dock_bays": bays}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tree", default_value="fire_manual",
                              description="fire_manual | fire_test | disruptive"),
        DeclareLaunchArgument("publish_setpoints", default_value="false",
                              description="may the tree take the sticks (G1)"),
        DeclareLaunchArgument("fire_pump", default_value="false",
                              description="may the tree fire the pump (G7)"),
        DeclareLaunchArgument("dock_bays", default_value="3",
                              description="bays in the dock; 1 only to test against one practice bay"),
        OpaqueFunction(function=_nodes),
    ])
