"""node_registry — what the boat can run, and what the page may do about it.

No ROS, no subprocess, no I/O: this is the catalogue and the rules, so both can
be read in one place and exercised on a laptop. Starting things is
process_manager's job; deciding what is startable is this file's.

THE PROTECTION RULE, which is the reason this file exists at all. The repo's
second standing safety constraint says the RC e-stop is the only safety path
and that WiFi is a convenience, never a safety mechanism. A web page with a
"stop" button next to rc_watchdog inverts that: anyone who can reach the Jetson
over WiFi could switch off the RC-loss force-disarm, from a laptop, silently.

So the core stack is PROTECTED. The page shows whether it is up, and may start
it if it is down, but nothing served over HTTP can take it down. Stopping those
is a decision for someone at a terminal who has thought about it, which is
exactly the friction that should exist. Note the asymmetry is deliberate:
starting a safety node can only ever move the boat toward safe, so it needs no
such guard.

THE EXCLUSION RULE. The OAK-D admits exactly one client, so oakd_publisher,
buoy_detector and oak_detector cannot co-run — the second to start fails on a
device already open, which surfaces as an obscure depthai error rather than
"the camera is busy". Nodes that contend for one device share an `exclusive`
tag, and the page offers to stop the incumbent instead of letting the operator
discover the conflict from a traceback.
"""
from dataclasses import dataclass, field

# Groups, in the order the page lists them. Ordering is not cosmetic: the stack
# reads top-down the way it is brought up, so a group above another is one you
# want running first.
GROUPS = (
    ("core", "Core", "Safety and telemetry. Protected: startable here, not stoppable."),
    ("perception", "Perception", "Sensors. The three OAK-D nodes contend for one camera."),
    ("world", "World model", "Fusion and tracking. Needs perception and pose."),
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
    # Path of the raw MJPEG stream on that port, for the RECORDER only. The
    # page embeds `/` and never needs this, but a recorder pulling frames has
    # to name the stream itself — and the two servers disagree. buoy_detector
    # streams from ANY non-root path; mjpeg_server routes /stream/<view> and
    # 404s on a name it does not have. lidar_view's views are plan/elev/both,
    # so the obvious guess of /stream/view records nothing from it, silently.
    stream_path: str = ""
    note: str = ""


REGISTRY = (
    # ---- core: the stack that has been on the water. Protected. ----
    NodeSpec("telemetry_bridge", "telemetry_bridge", "crusader_fcu",
             "telemetry_bridge", "core", protected=True,
             note="the only thing that speaks MAVLink"),
    NodeSpec("rc_heartbeat_watchdog", "rc_watchdog", "crusader_behavior",
             "rc_watchdog", "core", protected=True,
             note="force-disarm on RC link loss"),
    NodeSpec("pixhawk_led_status_node", "pixhawk_led_status", "crusader_behavior",
             "pixhawk_led_status_node", "core", protected=True),
    NodeSpec("led_node", "led_node", "crusader_behavior", "led_node", "core",
             protected=True),

    # ---- perception ----
    NodeSpec("buoy_detector", "buoy_detector", "crusader_perception",
             "buoy_detector", "perception", exclusive="oakd", port=8080,
             stream_path="/stream",
             note="detections + annotated view; owns the OAK-D"),
    NodeSpec("oak_detector", "oak_detector", "crusader_perception",
             "oak_detector", "perception", exclusive="oakd", port=8080,
             stream_path="/stream",
             note="shape + LED colour, two engines; owns the OAK-D"),
    NodeSpec("oakd_publisher", "oakd_publisher", "crusader_perception",
             "oakd_publisher", "perception", exclusive="oakd",
             note="raw frames; owns the OAK-D. 38 MB/s on the wire"),
    NodeSpec("lidar_cluster_node", "lidar_cluster_node", "crusader_perception",
             "lidar_cluster_node", "perception",
             note="consumes the livox container's cloud"),

    # ---- world model ----
    NodeSpec("target_tracker", "target_tracker", "crusader_world_model",
             "target_tracker", "world",
             note="camera + LiDAR -> earth-anchored targets"),

    # ---- viewers: tools/ scripts, not ROS entry points ----
    NodeSpec("oak_view", "oak_view", "tools", "oak_view.py", "viewers",
             kind="script", exclusive="port8080", port=8080,
             stream_path="/stream/view",      # bare FrameBuffer -> named "view"
             note="re-serves a camera topic; does NOT open the device"),
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
              ("telemetry_bridge", "rc_heartbeat_watchdog",
               "pixhawk_led_status_node", "led_node",
               "buoy_detector", "lidar_cluster_node", "target_tracker")),
    "bench": ("Bench profile",
              ("telemetry_bridge", "lidar_cluster_node", "target_tracker")),
}

# All three serve the camera tab on ONE port (8080). buoy_detector and
# oak_detector are alternative detectors that each own the device; oak_view
# re-serves a topic without touching it. Any of them can fill the tab, and none
# can start while another holds the socket — which is fine, because the two
# detectors already cannot co-run for the device itself. Order matters: the
# first one found running wins the tab.
CAMERA_TAB_SOURCES = ("buoy_detector", "oak_detector", "oak_view")
LIDAR_TAB_SOURCES = ("lidar_view",)


def conflicts(name: str, running) -> list:
    """Names that must stop before `name` may start.

    `running` is any iterable of currently-running node names. Returns the
    subset that shares an `exclusive` tag with the requested node — the page
    turns that into "stop buoy_detector and start oakd_publisher?" instead of
    letting the operator meet a device-busy traceback.
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

    Returning a reason rather than a bare False is what lets the page say why a
    button is absent instead of just not having one — an unexplained missing
    control reads as a bug and invites someone to go around it.
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
    table within a second of being started and then spends ten to thirty more
    loading a TensorRT engine and opening the OAK-D before its stream server
    binds. A tab that trusted "running" alone pointed an iframe at a port
    nothing was listening on yet, got connection-refused, and — because the
    page will not reload a stream it thinks is already correct — stayed on that
    error page permanently, while opening the same URL by hand worked fine
    because that load happened after the port came up.

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