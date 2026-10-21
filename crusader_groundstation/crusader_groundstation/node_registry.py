"""node_registry — what the boat can run, and what the page may do about it.

No ROS, no subprocess, no I/O: the catalogue and the rules, exercised on a
laptop. Starting things is process_manager's job; deciding what is startable is
this file's.

THE PROTECTION RULE. The RC e-stop is the only safety path and WiFi is a
convenience, never a safety mechanism, so the core stack is PROTECTED: the page
shows whether it is up and may start it, but nothing served over HTTP can take
it down (a "stop" button next to rc_watchdog would let anyone on the WiFi switch
off the RC-loss force-disarm). Starting a safety node can only move the boat
toward safe, so it needs no such guard.

THE EXCLUSION RULE. The OAK-D admits exactly one client, so oakd_publisher,
buoy_detector and oak_detector cannot co-run (the second fails with an obscure
depthai error). Nodes contending for one device share an `exclusive` tag, and
the page offers to stop the incumbent.
"""
from dataclasses import dataclass, field

# Groups, in the order the page lists them: top-down, the way the stack is
# brought up.
GROUPS = (
    ("core", "Core", "Safety and telemetry. Protected: startable here, not stoppable."),
    ("perception", "Perception", "Sensors. The three OAK-D nodes contend for one camera."),
    ("world", "World model", "Fusion and tracking. Needs perception and pose."),
    ("link", "UAV link", "The RFD900 mesh to the aircraft. One owner of the radio."),
    ("viewers", "Viewers", "Bench views served on their own ports."),
)


@dataclass(frozen=True)
class NodeSpec:
    """One launchable thing.

    kind:
      "ros"    -> ros2 run <package> <executable>. Presence is detected from the
                  ROS graph, so a node started by core.launch.py or by hand in
                  another terminal shows as running here too.
      "script" -> python3 <tools_dir>/<executable>. A bench tool, not a ROS
                  node we can name in the graph, so it is only visible while
                  THIS process is its parent.
    """
    name: str                      # ROS node name (kind="ros") or a unique id
    label: str
    package: str
    executable: str
    group: str
    kind: str = "ros"
    protected: bool = False        # startable here, never stoppable here
    exclusive: str = ""            # tag; two nodes sharing one cannot co-run
    port: int = 0                  # serves a browser view on this port, if any
    # Path of the raw MJPEG stream on that port, for the RECORDER only (the page
    # embeds `/`). The two servers disagree: buoy_detector streams from ANY
    # non-root path, mjpeg_server routes /stream/<view>, and lidar_view's views
    # are plan/elev/both.
    stream_path: str = ""
    # Views this producer offers, as (name, label). Empty means one unnamed
    # view, which is every viewer that is not the detector.
    views: tuple = ()
    # What a RECORDING pulls, when that is not what the tab shows: the tab shows
    # the annotated frame, a recording wants the raw one (a training set of
    # frames with boxes burned in is a training set of boxes).
    record_stream_path: str = ""
    note: str = ""


REGISTRY = (
    # ---- core: the stack that has been on the water. Protected. ----
    NodeSpec("telemetry_bridge", "telemetry_bridge", "crusader_behavior",
             "telemetry_bridge", "core", protected=True,
             note="the only thing that speaks MAVLink"),
    NodeSpec("rc_watchdog", "rc_watchdog", "crusader_behavior",
             "rc_watchdog", "core", protected=True,
             note="force-disarm on RC link loss"),
    NodeSpec("led_status_node", "led_status_node", "crusader_behavior",
             "led_status_node", "core", protected=True),
    NodeSpec("led_node", "led_node", "crusader_behavior", "led_node", "core",
             protected=True),

    # ---- perception ----
    NodeSpec("buoy_detector", "buoy_detector", "crusader_perception",
             "buoy_detector", "perception", exclusive="oakd", port=8080,
             stream_path="/stream",
             note="detections + annotated view; owns the OAK-D"),
    NodeSpec("oak_detector", "oak_detector", "crusader_perception",
             "oak_detector", "perception", exclusive="oakd", port=8080,
             stream_path="/stream/annotated",
             views=(("annotated", "Annotated"), ("raw", "Raw")),
             record_stream_path="/stream/raw",
             note="shape + LED colour, two engines; owns the OAK-D"),
    NodeSpec("oakd_publisher", "oakd_publisher", "crusader_perception",
             "oakd_publisher", "perception", exclusive="oakd",
             note="raw frames; owns the OAK-D. 38 MB/s on the wire"),
    NodeSpec("lidar_cluster_node", "lidar_cluster_node", "crusader_perception",
             "lidar_cluster_node", "perception",
             note="consumes the livox container's cloud"),
    # Listed so the Nodes tab can say whether the autopilot is being fed
    # obstacles: the map's proximity layer is empty either because this producer
    # is down or because the water is clear.
    NodeSpec("proximity_bridge", "proximity_bridge", "crusader_perception",
             "proximity_bridge", "perception",
             note="clusters -> OBSTACLE_DISTANCE for the autopilot (PRX1)"),

    # ---- world model ----
    NodeSpec("target_tracker", "target_tracker", "crusader_world_model",
             "target_tracker", "world",
             note="camera -> earth-anchored targets; use_lidar adds the LiDAR"),

    # ---- viewers: tools/ scripts, not ROS entry points ----
    # ---- link: the RFD900 mesh. Not protected: it has not been on the water. ----
    NodeSpec("rxl_link_node", "rxl_link_node", "crusader_link",
             "rxl_link_node", "link", exclusive="rfd_radio",
             note="owns the RFD900 and the Radio tab's record; talks to the "
                  "aircraft, never to the Pixhawk"),

    NodeSpec("oak_view", "oak_view", "tools", "oak_view.py", "viewers",
             kind="script", exclusive="port8080", port=8080,
             stream_path="/stream/view",      # bare FrameBuffer -> named "view"
             note="re-serves a camera topic; does NOT open the device"),
    NodeSpec("dock_view", "dock_view", "tools", "dock_view.py", "viewers",
             kind="script", exclusive="oakd", port=8080,
             stream_path="/stream/annotated",
             views=(("annotated", "Annotated"), ("raw", "Raw")),
             record_stream_path="/stream/raw",
             note="the Task 3 dock detector: the CV team's model (INTERIM, "
                  "mock-up bay) + colour rule; owns the OAK-D; publishes "
                  "DockObservation (window x,y,z) on /dock/observations"),
    NodeSpec("lidar_view", "lidar_view", "tools", "lidar_view.py", "viewers",
             kind="script", port=8081,
             stream_path="/stream/plan",      # views: plan / elev / both
             note="MID360 cloud, plan and elevation"),
)

BY_NAME = {n.name: n for n in REGISTRY}

# One-click profiles. A profile is a claim about what a session needs, and
# naming them here rather than in the page keeps that claim reviewable.
PROFILES = {
    "field": ("Field profile",
              ("telemetry_bridge", "rc_watchdog",
               "led_status_node", "led_node",
               "buoy_detector", "lidar_cluster_node", "target_tracker")),
    "bench": ("Bench profile",
              ("telemetry_bridge", "lidar_cluster_node", "target_tracker")),
    "avoidance": ("Avoidance check",
                  ("telemetry_bridge", "lidar_cluster_node",
                   "proximity_bridge", "lidar_view")),
}

# All serve the camera tab on ONE port (8080), so none can start while another
# holds the socket. The first one found running wins the tab.
CAMERA_TAB_SOURCES = ("buoy_detector", "oak_detector", "dock_view", "oak_view")
LIDAR_TAB_SOURCES = ("lidar_view",)


def conflicts(name: str, running) -> list:
    """Names that must stop before `name` may start.

    `running` is any iterable of running node names. Returns those sharing an
    `exclusive` tag with the requested node, so the page can offer "stop
    buoy_detector and start oakd_publisher?".
    """
    spec = BY_NAME.get(name)
    if spec is None or not spec.exclusive:
        return []
    return [other for other in running
            if other != name
            and BY_NAME.get(other)
            and BY_NAME[other].exclusive == spec.exclusive]


def may_stop(name: str) -> tuple:
    """(allowed, reason). The single gate every stop request passes through.

    A reason, not a bare False, so the page can say why a button is absent.
    """
    spec = BY_NAME.get(name)
    if spec is None:
        return False, f"unknown node {name!r}"
    if spec.protected:
        return False, (
            f"{name} is part of the safety stack and cannot be stopped from a "
            "web page. RC e-stop is the only safety path; a network-reachable "
            "off switch for the RC-loss watchdog would undo that. Stop it from "
            "a terminal if you really mean to.")
    return True, ""


def tab_source(sources, running, serving=None):
    """Which candidate is filling a viewer tab: (source, starting).

    `running` is the set of node names that exist as processes. `serving` is
    the subset whose HTTP port is actually accepting connections — pass None to
    skip that distinction.

    THE DISTINCTION IS THE WHOLE POINT. buoy_detector appears in the process
    table within a second but spends 10-30 s loading a TensorRT engine and
    opening the OAK-D before its stream server binds. A tab that trusted
    "running" alone pointed an iframe at a port nothing was listening on, got
    connection-refused, and (the page does not reload a stream it thinks is
    correct) stayed on that error page.

    Returns:
      (name, False)  a viewer is up and serving; show it.
      (name, True)   the process exists but its port is not open YET; say
                     "starting" and check again.
      (None, False)  nothing is running; offer to start it.
    """
    for name in sources:
        if name not in running:
            continue
        if serving is None or name in serving:
            return name, False
        return name, True
    return None, False