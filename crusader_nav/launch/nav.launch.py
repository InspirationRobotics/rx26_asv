"""nav.launch.py - the Nav2 planner stack: nav_frames_node, planner_server and
nav_lifecycle, the node that configures and activates it.
docs/nav2_avoidance_spec.md section 3.6.

    ros2 launch crusader_nav nav.launch.py datum_source:=param datum_lat:=<lat> datum_lon:=<lon>

NOT in core.launch.py, on purpose: core.launch.py is for what has run on the
water. Start this BY HAND, inside the asv container (or crsd-sim), BEFORE
bt_runner_node. Source /opt/ros/humble/setup.bash and the workspace first.

Arguments
  nav2_params   Nav2 parameter file          (default: this package's config/nav2_params.yaml)
  crsd_params   crusader_params.yaml         (default: crusader_bringup's)
  datum_source  param | first_fix            ('' = whatever crsd_params says)
  datum_lat, datum_lon                       used with datum_source:=param

planner_server stays in `activating` until TF map -> base_footprint exists,
which needs a pose WITH A FINITE HEADING. Check with
`ros2 lifecycle get /planner_server` in the same container.

nav_lifecycle replaces nav2_lifecycle_manager (Humble 1.1.20), whose
change_state call has no timeout and waited forever for a reply Fast DDS had
dropped, leaving planner_server `inactive`. nav_lifecycle times out, asks again,
and re-activates a respawned planner_server; there is no bond to wait for.
"""
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

RESPAWN_DELAY_S = 2.0


def _yaml_datum_source(crsd_params):
    """The datum_source crsd_params gives nav_frames_node ('first_fix' if it
    says nothing), so the respawn decision uses the value the node will see."""
    try:
        with open(crsd_params, encoding="utf-8") as f:
            section = yaml.safe_load(f)["nav_frames_node"]["ros__parameters"]
        return str(section.get("datum_source", "first_fix"))
    except (OSError, KeyError, TypeError, yaml.YAMLError):
        return "first_fix"


def _nodes(context):
    arg = {k: LaunchConfiguration(k).perform(context)
           for k in ("nav2_params", "crsd_params", "datum_source", "datum_lat", "datum_lon")}
    frames_params = [arg["crsd_params"]]
    source = _yaml_datum_source(arg["crsd_params"])
    if arg["datum_source"]:
        source = arg["datum_source"]
        frames_params.append({"datum_source": source,
                              "datum_lat": float(arg["datum_lat"]),
                              "datum_lon": float(arg["datum_lon"])})
    # A respawned nav_frames_node with first_fix would pin a NEW datum and move
    # the map under a running bt_runner, so only a parameter datum may respawn.
    respawn_policy = dict(respawn=source == "param", respawn_delay=RESPAWN_DELAY_S)
    return [
        Node(package="crusader_nav", executable="nav_frames_node", name="nav_frames_node",
             output="screen", parameters=frames_params, **respawn_policy),
        # nav_lifecycle configures and activates a respawned planner_server again
        # within a few check periods of it coming back (nav_lifecycle in nav2_params.yaml).
        Node(package="nav2_planner", executable="planner_server", name="planner_server",
             output="screen", parameters=[arg["nav2_params"]],
             respawn=True, respawn_delay=RESPAWN_DELAY_S),
        # Stateless: a respawned one just looks at the planner again.
        Node(package="crusader_nav", executable="nav_lifecycle", name="nav_lifecycle",
             output="screen", parameters=[arg["nav2_params"]],
             respawn=True, respawn_delay=RESPAWN_DELAY_S),
    ]


def generate_launch_description():
    nav_share = get_package_share_directory("crusader_nav")

    def default_crsd_params():
        # crusader_bringup is looked up at launch time, not declared as a build
        # dependency: bringup depends on this package, so the reverse would be
        # a cycle.
        return os.path.join(get_package_share_directory("crusader_bringup"),
                            "config", "crusader_params.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("nav2_params", default_value=os.path.join(
            nav_share, "config", "nav2_params.yaml")),
        DeclareLaunchArgument("crsd_params", default_value=default_crsd_params()),
        DeclareLaunchArgument("datum_source", default_value="",
                              description="param | first_fix; empty = crsd_params"),
        DeclareLaunchArgument("datum_lat", default_value="0.0"),
        DeclareLaunchArgument("datum_lon", default_value="0.0"),
        OpaqueFunction(function=_nodes),
    ])
