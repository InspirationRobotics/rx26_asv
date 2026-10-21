"""node_registry — what the boat can run, and what the ground station may do about it.

No ROS, no subprocess, no I/O: the catalogue and the rules. Starting things is
process_manager's job; deciding what is startable is this file's.

THE PROTECTION RULE. The RC e-stop is the only safety path and WiFi is a convenience,
never a safety mechanism, so the core stack is PROTECTED: the ground station shows
whether it is up and may start it, but nothing reachable over the network can stop it
(a stop button next to rc_watchdog would let anyone on the WiFi switch off the RC-loss
force-disarm). Starting a safety node can only move the boat toward safe.

THE EXCLUSION RULE. The OAK-D and the RFD900 each admit one owner; nodes contending for
one device share an `exclusive` tag, and the ground station offers to stop the incumbent.
"""
from dataclasses import dataclass

# In the order the Nodes tab lists them: the way the stack is brought up.
GROUPS = (
    ("core", "Core", "Safety and telemetry. Protected: startable here, never stoppable."),
    ("perception", "Perception", "The OAK-D and the MID360."),
    ("world", "World model", "Fusion and tracking. Needs perception and pose."),
    ("mission", "Mission", "The behaviour trees."),
    ("link", "Links", "The RFD900 mesh to Ekko, and the OCS."),
)


@dataclass(frozen=True)
class NodeSpec:
    """One ROS node: `ros2 run <package> <executable>`, which names itself `name`."""
    name: str
    package: str
    executable: str
    group: str
    protected: bool = False        # startable here, never stoppable here
    exclusive: str = ""            # tag; two nodes sharing one cannot co-run
    note: str = ""


REGISTRY = (
    NodeSpec("telemetry_bridge", "crusader_behavior", "telemetry_bridge", "core",
             protected=True, note="the only thing that speaks MAVLink"),
    NodeSpec("rc_watchdog", "crusader_behavior", "rc_watchdog", "core", protected=True,
             note="force-disarm on RC link loss"),
    NodeSpec("led_status_node", "crusader_behavior", "led_status_node", "core",
             protected=True),
    NodeSpec("led_node", "crusader_behavior", "led_node", "core", protected=True),
    NodeSpec("camera_node", "crusader_perception", "camera_node", "perception",
             exclusive="oakd", note="the OAK-D; model none | buoy | dock"),
    NodeSpec("lidar_node", "crusader_perception", "lidar_node", "perception",
             note="clusters, the Nav2 cloud, OBSTACLE_DISTANCE for the autopilot"),
    NodeSpec("target_tracker", "crusader_world_model", "target_tracker", "world",
             note="camera -> earth-anchored targets; use_lidar adds the LiDAR"),
    NodeSpec("bt_runner_node", "crusader_bt", "bt_runner_node", "mission",
             note="runs the built-in trees /crsd/run_mission names"),
    NodeSpec("rfd_link_node", "crusader_link", "rfd_link_node", "link",
             exclusive="rfd_radio", note="the RFD900 mesh to Ekko; never the Pixhawk"),
    NodeSpec("ocs_client", "crusader_groundstation", "ocs_client", "link",
             note="heartbeat and task reports to the OCS"),
)

BY_NAME = {n.name: n for n in REGISTRY}

# One-click profiles. A profile never resolves a device conflict (see start_profile).
PROFILES = {
    "field": ("Field", ("telemetry_bridge", "rc_watchdog", "led_status_node", "led_node",
                        "camera_node", "lidar_node", "target_tracker", "bt_runner_node",
                        "rfd_link_node", "ocs_client")),
    "bench": ("Bench", ("telemetry_bridge", "lidar_node", "target_tracker")),
}


def conflicts(name: str, running) -> list:
    """Running nodes that hold the device `name` needs, which must stop first."""
    spec = BY_NAME.get(name)
    if spec is None or not spec.exclusive:
        return []
    return [other for other in running
            if other != name and BY_NAME.get(other)
            and BY_NAME[other].exclusive == spec.exclusive]


def may_stop(name: str) -> tuple:
    """(allowed, reason): the single gate every stop or restart passes through."""
    spec = BY_NAME.get(name)
    if spec is None:
        return False, f"unknown node {name!r}"
    if spec.protected:
        return False, (
            f"{name} is part of the safety stack and cannot be stopped from the ground "
            "station: the RC e-stop is the only safety path, and a network-reachable off "
            "switch for it would undo that. Stop it from a terminal if you really mean to.")
    return True, ""
