"""task3.launch.py — what Task 3 (coordinated logistics / the fixed-nozzle shot)
needs ON TOP OF core.launch.py.

    ros2 launch crusader_bringup task3.launch.py tree:=fire_manual

The pan/tilt cannon in front of one bay (after a reboot, this is everything):

    ros2 launch crusader_bringup task3.launch.py tree:=cannon_fire

What the run may do is crusader_params.yaml's, read at launch (2026-10-11: no
switches on the command line): bt_runner_node publish_setpoints (the sticks,
G1) and fire_pump, and cannon_aim_node fire_pump (the WATER, G7). The first line
the launch prints says which.

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

THE SWITCHES ARE THE FILE'S. publish_setpoints (may the tree take the sticks,
Gate G1) and fire_pump (may it squirt, Gate G7) come from crusader_params.yaml,
so check them there before a run: false/false is a shadow run that logs what it
would do and touches nothing. tree.sh still takes them as arguments, for a run
that differs from the file on purpose. See docs/T3_coordinated_logistics.md.
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
    # the mission in two halves, SC GUIDED -> MANUAL between them (part 1's header)
    "part1": "task3_part1_approach.xml",      # GUIDED: find the GREEN bay, line up 3 m out
    "part2": "task3_part2_dock_fire.xml",     # MANUAL: dock, report, fire, request
    # the pan/tilt cannon, ONE run: GUIDED approach, SC to MANUAL, dock on the
    # LiDAR's slip (dock_slot_node), aim + spray (cannon_aim_node), the reports
    "cannon": "task3_cannon.xml",
    # the pan/tilt cannon in front of ONE bay, MANUAL (2026-10-09, the practice
    # dock: no slip walls): hold 1.2 m out on the bay's face (cal window aims),
    # or that hold + wait for a RED window and spray it until GREEN
    "cannon_hold": "task3_cannon_hold.xml",
    "cannon_fire": "task3_cannon_fire.xml",
    # the same in a SLIP (2026-10-16, the practice slip): the camera brings the
    # bow to the slip's mouth, then the LiDAR's slip (dock_slot_node) centres,
    # squares and sets the distance
    "cannon_slip": "task3_cannon_slip.xml",
    # TEST variants (headers say how they differ):
    "fire_1p6": "task3_fire_manual_1p6.xml",  # pool tuning: 1.6 m, no attempt limit
    "approach_test": "task3_approach_test.xml",  # GUIDED approach to line-up only; dock_bays:=1
}



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
    # the switches, from the file (see the header)
    with open(params, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    bt = cfg["bt_runner_node"]["ros__parameters"]
    setpoints = bool(bt.get("publish_setpoints", False))
    pump = bool(bt.get("fire_pump", False))
    water = bool(cfg.get("cannon_aim_node", {}).get("ros__parameters", {}).get("fire_pump", False))
    bays = int(LaunchConfiguration("dock_bays").perform(context))
    if bays not in (1, 2, 3):
        raise ValueError(f"dock_bays:={bays}: 1..3 (3 on the course)")
    mode = ("FIRE (sticks + pump)" if setpoints and pump else
            "HOLD (sticks, pump DRY)" if setpoints else
            "SHADOW (logs only, touches nothing)" if not pump else
            "PUMP WITHOUT STICKS")
    if tree in ("cannon", "cannon_hold", "cannon_fire", "cannon_slip"):
        mode += " | cannon " + ("FIRES THE PUMP" if water else "DRY")

    # The cannon tree docks on the slip, not the wall range, and its water goes
    # through cannon_aim_node (whose own fire_pump is this launch's).
    cannon = Node(package="crusader_fcu", executable="cannon_aim_node",
                  output="screen", parameters=[params])
    wall = Node(package="crusader_perception", executable="wall_range_node",
                output="screen", parameters=[params])
    slot = Node(package="crusader_perception", executable="dock_slot_node",
                output="screen", parameters=[params])
    if tree == "cannon":
        lidar = [slot, cannon]
    elif tree == "cannon_slip":
        # the approach is on the wall range and the face, the dock on the slip
        lidar = [wall, slot, cannon]
    elif tree in ("cannon_hold", "cannon_fire"):
        # the hold is on the wall range and the face; the cannon aims (cal
        # window during a hold, the tree in cannon_fire)
        lidar = [wall, cannon]
    else:
        lidar = [wall]
    return [
        LogInfo(msg=f"task3: tree {TREES[tree]} | from crusader_params.yaml: publish_setpoints "
                    f"{setpoints}, fire_pump {pump} -> {mode}"),
        *lidar,
        ExecuteProcess(cmd=["python3", os.path.join(tools_dir, "dock_view.py")],
                       output="screen"),
        # The params file first, then the tree and the bays: a later entry wins,
        # so the file is the source of truth for everything else, the switches too.
        Node(package="crusader_bt", executable="bt_runner_node",
             output="screen",
             parameters=[params, {"tree_file": tree_file,
                                  "dock_bays": bays}]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("tree", default_value="fire_manual",
                              description="fire_manual | fire_test | disruptive | part1 | part2 | "
                                          "cannon | cannon_hold | cannon_fire | cannon_slip | "
                                          "fire_1p6 | "
                                          "approach_test"),
        DeclareLaunchArgument("dock_bays", default_value="3",
                              description="bays in the dock; 1 only to test against one practice bay"),
        OpaqueFunction(function=_nodes),
    ])
