"""ground_station — the whole boat in one browser tab.

    ros2 run crusader_groundstation ground_station
    # then, on the laptop:  http://<JETSON_IP>:8090

Publishes no topics. Its outward effects are processes, parameter sets, the
mission goal and power; spawning processes and the power verbs are gated and
re-checked server-side.

Port 8090, clear of the viewers it embeds: 8080 is buoy_detector /
tools/oak_view.py, 8081 is tools/lidar_view.py. The camera and LiDAR tabs are
iframes onto those, and mjpeg_server skips rendering with no client attached,
so an unopened tab costs the Jetson nothing.

The map lives here, not in crusader_world_model: a display of world state
belongs with the operator's controls, and that leaves the world model pure
geometry with no HTTP server bolted to it.

`/params/set` is confined to what the TARGET node declared dynamic — a
read-only parameter is refused by the node itself, not by a rule kept here.
Nothing is persisted: crusader_params.yaml stays the source of truth, the page
shows live-versus-YAML drift, and writing the file back from a browser would
destroy the comments that are most of its value.

TWO RULES, re-checked on every request whatever the page rendered:

  PROTECTED NODES CANNOT BE STOPPED FROM HERE. The RC e-stop is the only
  safety path and WiFi is never a safety mechanism; a network-reachable off
  switch for rc_watchdog would undo that. Starting them can only move the boat
  toward safe, so it is allowed.

  POWER IS LOCKED WHILE ARMED, and then needs the hostname typed. A reboot
  button one stray click away from a boat under way is not a button.
"""
import json
import math
import os
import socket
import threading
import time
from collections import deque

from rclpy.action import ActionClient
from rclpy.node import Node

from rcl_interfaces.msg import Log
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crusader_msgs.action import SafePassage
from crusader_msgs.msg import (Attitude, Cluster3DArray, FcuStatus,
                               LatLonHead, ObstacleDistance, RadioFrame,
                               TrackedTargetArray)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.node_main import run_node
from crusader_common.param_utils import (declare, declare_from_config,
                                        make_set_callback)
from crusader_common.stream_cache import StreamCache

from crusader_groundstation import node_registry as reg
from crusader_groundstation import bag_recorder, camera_profiles, lake_rig, param_client
from crusader_groundstation import mission_api
from crusader_groundstation import planner_profiles
from crusader_groundstation import power_client, proc_scan, radio_core
from crusader_groundstation import system_info
from crusader_groundstation.log_buffer import LogBuffer
from crusader_groundstation.recorder import Recorder
from crusader_groundstation.recorder import free_gb as recorder_free_gb
from crusader_groundstation.recorder import frame_rates
from crusader_groundstation.gcs_page import render as render_page
from crusader_groundstation.gcs_server import GcsServer
from crusader_groundstation.process_manager import ProcessManager

#: How long an HTTP action waits for rxl_link_node to answer send_test: long
#: enough for a busy executor, short enough to give an answer, not a spinner.
RADIO_TEST_TIMEOUT_S = 5.0

PARAM_SPEC = {
    "port": dict(read_only=True, lo=1024, hi=65535,
                 description="HTTP port; 8080/8081 are the viewers"),
    "bind_host": dict(read_only=True,
                      description="0.0.0.0 so the laptop can reach it"),
    "targets_topic": dict(read_only=True, description="TrackedTargetArray in"),
    "clusters_topic": dict(read_only=True,
                           description="Cluster3DArray in, for the map layer"),
    "obstacle_topic": dict(read_only=True,
                           description="ObstacleDistance in, as the autopilot "
                                       "receives it"),
    "clusters_timeout_s": dict(read_only=True, lo=0.2, hi=30.0),
    "obstacle_timeout_s": dict(read_only=True, lo=0.2, hi=30.0),
    "nav_leg_topic": dict(read_only=True,
                          description="bt_runner's planned-leg status (String "
                                      "JSON) in, for the path on the map"),
    "nav_leg_timeout_s": dict(read_only=False, lo=0.5, hi=30.0,
                              description="a leg status older than this is "
                                          "blank, never the last path"),
    "tools_dir": dict(read_only=True,
                      description="where tools/*.py live, for the viewers"),
    "power_socket": dict(read_only=True,
                         description="host power helper socket; see "
                                     "tools/scripts/crsd_power_helper.py"),
    "disk_path": dict(read_only=True, description="filesystem to report free"),
    "pose_timeout_s": dict(read_only=True, lo=0.2, hi=10.0,
                           description="= shared.pose_timeout_s"),
    "attitude_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "status_timeout_s": dict(read_only=True, lo=0.2, hi=10.0),
    "targets_timeout_s": dict(read_only=True, lo=0.2, hi=30.0),
    "poll_period_s": dict(read_only=True, lo=0.05, hi=5.0,
                          description="how often the browser asks for /state"),
    "graph_period_s": dict(read_only=False, lo=0.2, hi=10.0,
                           description="how often the ROS node graph is scanned"),
    "trail_length": dict(read_only=False, lo=0, hi=20000),
    "trail_min_move_m": dict(read_only=False, lo=0.0, hi=50.0),
    "allow_power": dict(read_only=True,
                        description="master switch for the power tab"),
    "log_capacity": dict(read_only=True, lo=100, hi=20000,
                         description="/rosout lines kept in the ring"),
    "record_dir": dict(read_only=True, description="where sessions are written"),
    "record_telemetry_hz": dict(read_only=False, lo=0.1, hi=20.0,
                                description="telemetry.jsonl sample rate"),
    # 60: the pipeline runs at ~30 fps and this ceiling must reach it; 60, not
    # 30, because pinning the PARAMETER to today's camera would silently
    # under-record a faster one. The page reads this range out of the snapshot,
    # so its boxes and this check can never disagree.
    "record_frame_hz": dict(read_only=False, lo=0.05, hi=60.0,
                            description="default frames per second per viewer; "
                                        "the Record tab overrides it per "
                                        "session"),
    "record_min_free_gb": dict(read_only=False, lo=0.5, hi=500.0,
                               description="stop recording below this"),
}


# Defaults that live HERE, not in crusader_params.yaml (the team's file, not edited for
# tooling); a ground_station section that names them wins, because declare() takes the override.
CODE_DEFAULTS = {
    "planner_profiles_dir": planner_profiles.DEFAULT_SHIPPED_DIR,
    "planner_profiles_save_dir": planner_profiles.DEFAULT_SAVE_DIR,
    "mission_runner_node": "/bt_runner_node",
    "mission_action": "/crsd/safe_passage",
    "mission_bt_dir": "",
    "mission_saved_dir": "~/.ros/crusader_missions/saved",
    "mission_run_dir": "~/.ros/crusader_missions/run",
}
PARAM_SPEC.update({
    "mission_runner_node": dict(
        read_only=True,
        description="the BT runner node the Mission tab points at: it sets this node's tree_file "
                    "parameter, then sends the goal"),
    "mission_action": dict(
        read_only=True, description="the SafePassage action the Mission tab's Start sends"),
    "mission_bt_dir": dict(
        read_only=True,
        description="behavior_trees directory the Mission tab reads subtrees and pre-built "
                    "missions from; blank = the installed crusader_bt share"),
    "mission_saved_dir": dict(
        read_only=True, description="where the Mission tab saves missions (~ is the user's home)"),
    "mission_run_dir": dict(
        read_only=True,
        description="where Start writes the XML that the runner loads (~ is the user's home)"),
    "planner_profiles_dir": dict(
        read_only=True,
        description="*.yaml planner-tuning profiles the Tuning tab can load "
                    "(crusader_sim/config/tuning_profiles); blank or missing -> an empty "
                    "list, with the reason"),
    "planner_profiles_save_dir": dict(
        read_only=True,
        description="where 'Save as profile' writes; also listed in the picker. "
                    "~ is the user's home (in asv: /root)"),
})


# Module level and published in the snapshot because the Record tab lays out
# one fps control per KEY — a second list in the page is how a third viewer
# would get added everywhere except the control that sets its rate.
FRAME_SOURCES = (("camera", reg.CAMERA_TAB_SOURCES),
                 ("lidar", reg.LIDAR_TAB_SOURCES))

FRAME_KEYS = [k for k, _ in FRAME_SOURCES]


class SafePassagePort:
    """The SafePassage action as mission_api.MissionRun's goal port.

    Called from an HTTP thread while the node spins on the main one, so every wait is a
    done-callback plus an Event with a short timeout, never a second executor.
    """

    def __init__(self, node, action_name, wait_s=2.0):
        self._name, self._wait_s = action_name, wait_s
        self._client = ActionClient(node, SafePassage, action_name)
        self._handle = None

    def _wait(self, future):
        done = threading.Event()
        future.add_done_callback(lambda _f: done.set())
        return done.wait(self._wait_s)

    def send(self, tier, timeout_s, on_feedback, on_result):
        if not self._client.wait_for_server(timeout_sec=1.0):
            return False, f"nothing serves {self._name}: is the BT runner up?"
        goal = SafePassage.Goal()
        goal.tier = int(tier)
        goal.timeout_s = float(timeout_s)
        sent = self._client.send_goal_async(
            goal, feedback_callback=lambda m: on_feedback(_feedback_dict(m.feedback)))
        if not self._wait(sent):
            sent.cancel()
            return False, f"{self._name} did not answer the goal within {self._wait_s:.0f} s"
        handle = sent.result()
        if not handle.accepted:
            return False, (f"{self._name} rejected the goal (a mission may already be "
                           "running there)")
        self._handle = handle
        handle.get_result_async().add_done_callback(
            lambda f: on_result(_result_dict(f.result().result)))
        return True, ""

    def cancel(self):
        if self._handle is None:
            return False, "no goal to cancel"
        fut = self._handle.cancel_goal_async()
        if not self._wait(fut):
            return False, f"{self._name} did not answer the cancel within {self._wait_s:.0f} s"
        return True, ""


def _feedback_dict(fb):
    return {"phase": fb.phase, "progress": _round(float(fb.progress), 3),
            "buoys_known": int(fb.buoys_known), "buoys_resolved": int(fb.buoys_resolved),
            "plan_version": int(fb.plan_version), "warning": fb.warning}


def _result_dict(res):
    return {"code": int(res.outcome), "detail": res.detail,
            "elapsed_s": _round(float(res.elapsed_s), 1)}


class GroundStation(Node):
    """Subscriptions and a process table in, one JSON snapshot out."""

    def __init__(self):
        super().__init__("ground_station")
        p = declare_from_config(self, crsd_config.node_params("ground_station"),
                                PARAM_SPEC)
        for name, default in CODE_DEFAULTS.items():
            if name not in p:
                p[name] = declare(self, name, default, **PARAM_SPEC[name])
        self.p = p

        ranges = {n: (s["lo"], s["hi"]) for n, s in PARAM_SPEC.items()
                  if not s.get("read_only") and "lo" in s}
        self.add_on_set_parameters_callback(
            make_set_callback(self, ranges, self._apply))

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._targets = StreamCache(p["targets_timeout_s"])
        # Map layers: subscribed always, sent only when a page asks (_snapshot).
        # A callback per message is cheap; sixty clusters in every /state is not.
        self._clusters = StreamCache(p["clusters_timeout_s"])
        self._obstacles = StreamCache(p["obstacle_timeout_s"])
        # bt_runner's planned leg (docs/nav2_avoidance_spec.md 4.3), parsed once on
        # arrival. It is not a layer: a few hundred bytes, in every /state.
        self._nav_leg = StreamCache(p["nav_leg_timeout_s"])

        self._origin = None
        self._trail = deque(maxlen=int(p["trail_length"]) or 1)
        self._cpu = system_info.CpuMeter()
        self._hostname = socket.gethostname()

        self.procs = ProcessManager(
            tools_dir=p["tools_dir"],
            logger=lambda m: self.get_logger().info(m))

        # Task 1 tab's START/STOP RIG: crusader_sim's lake_rig_up.sh, run here (inside asv)
        # with the datum from the boat. The scripts sit beside tools/, so no parameter.
        repo = os.path.dirname(os.path.normpath(p["tools_dir"]))
        self.rig = lake_rig.LakeRig(os.path.join(repo, "crusader_sim", "scripts"),
                                    os.path.expanduser("~/.cache/crusader_lake"),
                                    logger=lambda m: self.get_logger().info(m))

        # Parameter clients are created on first use, so a session that never
        # opens the Tuning tab adds no graph entities.
        self.tuning = param_client.ParamBridge(self)

        # Mission tab: Start writes the run file, points the BT runner at it and sends the
        # SafePassage goal; mission_api.MissionRun owns the rules.
        self.mission = mission_api.MissionLib(
            self._mission_bt_dir(), os.path.expanduser(p["mission_saved_dir"]))
        self.mission_run = mission_api.MissionRun(
            self.mission, os.path.expanduser(p["mission_run_dir"]),
            self._mission_set_tree_file, SafePassagePort(self, p["mission_action"]),
            logger=lambda m: self.get_logger().info(m))

        # /rosout rather than journalctl: the host journal is outside this
        # container, while /rosout crosses the DDS domain with no privilege.
        self.logs = LogBuffer(int(p["log_capacity"]))
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)

        # Every RFD900 mesh frame rxl_link_node sent or heard. Kept here, not in
        # the page, so a closed Radio tab misses nothing: the counts and rate
        # estimates cannot be rebuilt from a scroll-back.
        self._radio = radio_core.RadioLog()
        self.create_subscription(RadioFrame, "/crsd/radio/traffic",
                                 self._on_radio, 50)
        self._radio_test = self.create_client(Trigger, "/crsd/radio/send_test")

        self.recorder = Recorder(p["record_dir"], p["record_min_free_gb"],
                                 logger=lambda m: self.get_logger().warn(m))
        # The bag is a SEPARATE child process in the same session directory: it
        # must be stopped with SIGINT to be readable, and /livox/lidar must never
        # go through this node's executor (bag_recorder's header).
        self.bags = bag_recorder.BagRecorder(
            logger=lambda m: self.get_logger().info(m))
        self._persist = self._check_persistence(p["record_dir"])
        self.create_timer(1.0 / p["record_telemetry_hz"], self._record_sample)

        # One /proc pass per graph tick: the two see different worlds and we
        # need both (proc_scan's header).
        self._proc = {}
        self._serving = set()

        # Scanned on a timer, not per request: get_node_names() is a discovery
        # call, and per poll per client it would put graph traffic on the wire
        # in proportion to how many people have the page open.
        self._graph = set()
        self._graph_full = []
        self.create_timer(p["graph_period_s"], self._scan_graph)

        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                 self._on_status, 10)
        self.create_subscription(TrackedTargetArray, p["targets_topic"],
                                 self._on_targets, 10)
        self.create_subscription(Cluster3DArray, p["clusters_topic"],
                                 self._on_clusters, 10)
        self.create_subscription(ObstacleDistance, p["obstacle_topic"],
                                 self._on_obstacles, 10)
        self.create_subscription(String, p["nav_leg_topic"],
                                 self._on_nav_leg, 10)

        self.server = GcsServer(render_page(p["poll_period_s"] * 1000.0),
                                self._snapshot, self._action,
                                download_fn=self.archive)
        self.server.start(int(p["port"]), p["bind_host"])
        self.get_logger().info(
            f"ground station on http://<JETSON_IP>:{int(p['port'])} — "
            f"nodes, telemetry, viewers, map, system. "
            f"power {'ENABLED' if p['allow_power'] else 'disabled by param'}.")

    def _check_persistence(self, record_dir):
        """Is record_dir on a bind mount, and say so once at startup.

        Mounts do not change under a running container, and the operator needs
        the answer BEFORE a session rather than after losing one.
        """
        mp, src, persists = system_info.mount_for(record_dir)
        if persists:
            self.get_logger().info(
                f"recordings persist: {record_dir} is on {mp} (from {src}) — "
                "they survive the container being recreated")
        else:
            self.get_logger().warn(
                f"RECORDINGS WILL NOT SURVIVE: {record_dir} is on the "
                "container's own filesystem, so `docker rm` discards every "
                "session. Point record_dir at a bind-mounted path — on this "
                "fleet /root/robotx_ws is the host's ~/robotx_ws — or add a "
                "mount (see crusader_groundstation/README.md).")
        return {"persists": bool(persists), "mount": mp, "source": src}

    def _apply(self, changes):
        if "trail_length" in changes:
            self._trail = deque(self._trail,
                                maxlen=int(changes["trail_length"]) or 1)
        if "nav_leg_timeout_s" in changes:
            self._nav_leg.timeout_s = float(changes["nav_leg_timeout_s"])
        self.p.update(changes)

    # ---------- inputs ----------

    def _on_pose(self, msg: LatLonHead):
        if math.isnan(msg.heading):
            return                     # GPS yaw unresolved; not a usable fix
        if self._origin is None:
            self._origin = (msg.latitude, msg.longitude)
        x, y = geo.latlon_to_xy(msg.latitude, msg.longitude, self._origin)
        self._pose.set((msg.latitude, msg.longitude, msg.heading,
                        msg.ground_speed, x, y), time.monotonic())
        gate = self.p["trail_min_move_m"]
        if not self._trail or math.hypot(x - self._trail[-1][0],
                                         y - self._trail[-1][1]) >= gate:
            self._trail.append((x, y))

    def _on_att(self, msg: Attitude):
        self._att.set((msg.roll, msg.pitch, msg.yaw), time.monotonic())

    def _on_status(self, msg: FcuStatus):
        self._status.set((msg.mode, msg.armed), time.monotonic())

    def _on_targets(self, msg: TrackedTargetArray):
        self._targets.set(msg, time.monotonic())

    def _on_clusters(self, msg: Cluster3DArray):
        self._clusters.set(msg, time.monotonic())

    def _on_obstacles(self, msg: ObstacleDistance):
        self._obstacles.set(msg, time.monotonic())

    def _on_nav_leg(self, msg: String):
        try:
            leg = json.loads(msg.data)
        except ValueError:
            leg = None
        if not isinstance(leg, dict):
            # a line that is not a JSON object is dropped; the cache then ages
            # out to a blank rather than keep showing the previous leg
            self.get_logger().warn("nav leg status is not a JSON object; ignored",
                                   throttle_duration_sec=10.0)
            return
        self._nav_leg.set(leg, time.monotonic())

    def _scan_graph(self):
        """Which registry nodes are present in the ROS graph right now, so a
        node started by core.launch.py, systemd or by hand shows as running.
        """
        try:
            pairs = self.get_node_names_and_namespaces()
            self._graph = {n for n, _ns in pairs}
            # Fully qualified too, for the parameter services: a namespaced node
            # answers /ns/node/set_parameters, and the bare name fails as a
            # timeout — i.e. as "that node is down", which it is not.
            self._graph_full = param_client.visible_nodes(
                ns.rstrip("/") + "/" + n for n, ns in pairs)
        except Exception:
            pass                       # discovery hiccup; keep the last answer
        # Catches what the graph cannot: the tools/ viewer scripts (not ROS
        # nodes) and anything started from a terminal.
        self._proc = proc_scan.scan([s.executable for s in reg.REGISTRY])
        # Ports actually ACCEPTING connections. A process in the table may not
        # have bound yet, and saying otherwise left an iframe stuck on
        # connection-refused.
        self._serving = {s.name for s in reg.REGISTRY
                         if s.port and _port_open(s.port)}

    def _on_rosout(self, msg: Log):
        """Every node's logger output, from anywhere in the DDS domain.

        Not filtered to our own nodes: the livox driver complaining is exactly
        the kind of thing that otherwise needs an SSH session.
        """
        self.logs.add(msg.name, msg.level, msg.msg,
                      stamp=msg.stamp.sec + msg.stamp.nanosec * 1e-9)

    def _record_sample(self):
        """One telemetry line, if a session is live. Also runs the disk guard."""
        self.recorder.tick()
        self.bags.tick()
        # The disk guard stops the SESSION at the free-space floor. A bag left
        # running would keep filling that disk, invisibly, because the session
        # that reported its size has closed.
        if self.bags.recording and not (
                self.recorder.session
                and self.recorder.session.stopped is None):
            self.get_logger().warn(
                "session ended with a bag still recording — closing the bag")
            self.bags.stop()
        if not (self.recorder.session
                and self.recorder.session.stopped is None):
            return
        now = time.monotonic()
        pose = self._pose.get(now)
        att = self._att.get(now)
        status = self._status.get(now)
        targets = self._targets.get(now)
        # Freshness is a FIELD, not a reason to skip the sample: a gap in the
        # file looks like a dead recorder; "pose_ok": false is worth having.
        self.recorder.sample({
            "t": time.time(),
            "pose_ok": pose is not None,
            "pose": None if pose is None else {
                "lat": pose[0], "lon": pose[1], "heading": pose[2],
                "speed": pose[3]},
            "att_ok": att is not None,
            "att": None if att is None else {
                "roll": att[0], "pitch": att[1], "yaw": att[2]},
            "fcu_ok": status is not None,
            "fcu": None if status is None else {
                "mode": status[0], "armed": bool(status[1])},
            "targets_ok": targets is not None,
            "targets": [] if targets is None else [
                {"id": int(x.id), "label": x.label, "lat": x.latitude,
                 "lon": x.longitude, "range": x.range, "bearing": x.bearing,
                 "hits": int(x.hits), "confirmed": bool(x.confirmed),
                 "unseen": x.time_since_seen}
                for x in targets.targets],
        })

    def _record_sources(self):
        """{key: mjpeg url} for whichever viewers are up right now.

        Recording pulls from the viewers rather than re-encoding, so a viewer
        that is not running contributes nothing.
        """
        _items, running = self._node_items()
        sources = {}
        for key, names in FRAME_SOURCES:
            src, starting = reg.tab_source(names, running, self._serving)
            if src and not starting:
                spec = reg.BY_NAME[src]
                # The RAW path when offered: frames are training material too,
                # and burned-in boxes ruin that; the boxes are in the bag anyway.
                path = spec.record_stream_path or spec.stream_path
                if spec.port and path:
                    sources[key] = f"http://127.0.0.1:{spec.port}{path}"
        return sources

    # ---------- the snapshot ----------

    def _armed(self):
        """(known, armed). Unknown is NOT the same as disarmed, and the power
        interlock treats it as unsafe — a missing FcuStatus most often means
        the bridge is down, which is not evidence the boat is safe to reboot."""
        s = self._status.get(time.monotonic())
        return (s is not None), (s[1] if s else False)

    def _node_items(self):
        owned = self.procs.running()
        running_names = set()
        items = []
        for spec in reg.REGISTRY:
            state, detail = self.procs.status(spec.name)
            in_graph = spec.kind == "ros" and spec.name in self._graph
            pids = self._proc.get(spec.executable, [])
            running = in_graph or spec.name in owned or bool(pids)
            if running:
                running_names.add(spec.name)
            allowed, why = reg.may_stop(spec.name)
            items.append({
                "name": spec.name, "label": spec.label, "package": spec.package,
                "group": spec.group, "protected": spec.protected,
                "note": spec.note, "port": spec.port,
                "running": running,
                "state": "running" if running else state,
                "detail": detail if not running else
                          ("in graph" if in_graph and spec.name not in owned
                           else detail),
                # Stoppable if the rule allows it and we can reach the process:
                # we own it, or /proc gave a real PID (never one guessed from a name).
                "stoppable": bool(allowed and (spec.name in owned or pids)),
                "pids": pids,
                "stop_reason": why or "",
                "tail": self.procs.tail(spec.name)[-8:],
            })
        for item in items:
            item["conflicts"] = reg.conflicts(item["name"], running_names)
        return items, running_names

    def _viewer_tab(self, sources, title, hint, running):
        """Which node is filling a viewer tab. Takes `running` rather than
        recomputing it: _node_items() walks the whole registry.
        """
        src, starting = reg.tab_source(sources, running, self._serving)
        return {
            # Set only once the port answers, so the page never points an
            # iframe at a socket that is not listening yet.
            "source": None if starting else src,
            "starting": starting,
            "starting_name": src if starting else None,
            "port": reg.BY_NAME[src].port if src else 0,
            # Empty for every viewer but the detector; an empty list renders
            # no picker.
            "views": [{"name": n, "label": label}
                      for n, label in (reg.BY_NAME[src].views if src else ())],
            "title": title, "hint": hint,
            "candidates": [{"name": n, "label": reg.BY_NAME[n].label}
                           for n in sources],
        }

    def _snapshot(self, layers=()):
        now = time.monotonic()
        pose = self._pose.get(now)
        att = self._att.get(now)
        status = self._status.get(now)
        targets = self._targets.get(now)
        items, running = self._node_items()

        boat = {"ok": pose is not None, "age": _round(self._pose.age(now)),
                "lat": None, "lon": None, "heading": None, "speed": None,
                "x": 0.0, "y": 0.0, "att_ok": att is not None,
                "att_age": _round(self._att.age(now)),
                "roll": None, "pitch": None, "yaw": None}
        if pose is not None:
            lat, lon, hdg, spd, x, y = pose
            boat.update(lat=lat, lon=lon, heading=hdg, speed=spd, x=x, y=y)
        if att is not None:
            boat.update(roll=math.degrees(att[0]), pitch=math.degrees(att[1]),
                        yaw=math.degrees(att[2]) % 360.0)

        sysinfo = system_info.snapshot(self._cpu, self.p["disk_path"])
        sysinfo["uptime_text"] = system_info.format_uptime(sysinfo["uptime_s"])

        snap = {
            "boat": boat,
            "fcu": {"ok": status is not None,
                    "age": _round(self._status.age(now)),
                    "mode": status[0] if status else None,
                    "armed": bool(status[1]) if status else False},
            "targets": {"ok": targets is not None,
                        "age": _round(self._targets.age(now)),
                        "items": self._target_items(targets)},
            "trail": [[round(x, 2), round(y, 2)] for x, y in self._trail],
            "nav": self._nav_state(now),
            "nodes": {
                "groups": [{"id": g, "label": lbl, "hint": hint}
                           for g, lbl, hint in reg.GROUPS],
                "items": items,
                "up": len(running),
            },
            "profiles": [{"id": k, "label": v[0]}
                         for k, v in reg.PROFILES.items()],
            # NAMES ONLY. Values come from /params/list on demand: parameter
            # round trips per poll per client would scale graph traffic with
            # viewers, like the graph scan. Node names fit the poll budget.
            "tuning": {"nodes": self._graph_full},
            "tabs": {
                "camera": self._viewer_tab(
                    reg.CAMERA_TAB_SOURCES, "Camera viewer not running",
                    "buoy_detector serves an annotated view on :8080; oak_view "
                    "re-serves a raw topic without touching the device.",
                    running),
                "lidar": self._viewer_tab(
                    reg.LIDAR_TAB_SOURCES, "LiDAR viewer not running",
                    "lidar_view serves plan and elevation on :8081.", running),
            },
            "system": sysinfo,
            "power": self._power_state(),
            "lake": self._lake_state(pose),
            "logs": {"counts": self.logs.counts(),
                     "nodes": self.logs.nodes()},
            "layers": {"clusters": "clusters" in layers,
                       "prox": "prox" in layers},
            "record": {
                "sessions": self.recorder.sessions(),
                "live": (self.recorder.session.status()
                         if self.recorder.session else None),
                "sources": sorted(self._record_sources()),
                "frame_hz": self.p["record_frame_hz"],
                # Same PARAM_SPEC range the node clamps to, so a value the page
                # accepts is never one the node quietly rewrites.
                "frame_hz_range": [PARAM_SPEC["record_frame_hz"]["lo"],
                                   PARAM_SPEC["record_frame_hz"]["hi"]],
                "frame_keys": FRAME_KEYS,
                "telemetry_hz": self.p["record_telemetry_hz"],
                "min_free_gb": self.p["record_min_free_gb"],
                "dir": self.p["record_dir"],
                # Whether a recording survives the container being recreated
                # is not guessable from the path (system_info.mount_for).
                "persist": self._persist,
                # Status only, for the growth rate; the topic table is fetched
                # once by /record/topics when the tab opens.
                "bag": self.bags.status(
                    _free_bytes(self.p["record_dir"])),
            },
        }
        # Optional map layers, built ONLY when a page asked. With a shoreline
        # in view the cluster layer is a few KB — ten times the rest of /state.
        # They ride the map's own poll, not a separate request, because they
        # must be drawn against the SAME boat position as the targets.
        if "clusters" in layers:
            snap["clusters"] = self._cluster_layer(now, pose, att)
        if "prox" in layers:
            snap["prox"] = self._prox_layer(now)
        return snap

    def _cluster_layer(self, now, pose, att):
        """Raw LiDAR clusters, in BOTH frames, or an empty stale marker.

        BODY frame is what the sensor said, and the frame QGC's PRX1 view is
        drawn in; WORLD frame lays it over the tracks built from it. Rotated
        here, not in the page: geo.body_to_world_ypr is the repo's one
        implementation of that transform, and a JavaScript copy would drift —
        a viewer that models the geometry differently from the node under test
        passes and fails for the wrong reasons.

        `placed` is false when pose or attitude is stale. Body-frame numbers
        are still sent (they do not depend on where the boat is), and the page
        falls back to bow-up rather than drawing the world layer where the boat
        used to be.
        """
        msg = self._clusters.get(now)
        placed = pose is not None and att is not None
        out = {"ok": msg is not None, "age": _round(self._clusters.age(now)),
               "placed": bool(placed and msg is not None), "items": []}
        if msg is None:
            return out
        for c in msg.clusters:
            item = {"x": round(c.x, 2), "y": round(c.y, 2), "z": round(c.z, 2),
                    "ex": round(c.extent_x, 2), "ey": round(c.extent_y, 2),
                    "n": int(c.n_points), "r": round(c.range, 2)}
            if placed:
                wx, wy, _wz = geo.body_to_world_ypr(
                    c.x, c.y, c.z, att[0], att[1], att[2], pose[4], pose[5])
                item["wx"], item["wy"] = round(wx, 2), round(wy, 2)
            out["items"].append(item)
        return out

    def _prox_layer(self, now):
        """The 72 sectors exactly as the autopilot receives them, unprocessed.

        It is what telemetry_bridge forwards as MAVLink OBSTACLE_DISTANCE, so
        beside QGC's PRX1 view: if the two disagree, the fault is between the
        ROS graph and the flight controller; if they agree with each other but
        not with the clusters, it is proximity_bridge's sector maths.

        UINT16_MAX survives as UINT16_MAX: ObstacleDistance.msg defines it as
        NO READING (0 is touching the hull), so the page draws nothing rather
        than a max-range arc — "not seen" and "seen and clear" differ.
        """
        msg = self._obstacles.get(now)
        out = {"ok": msg is not None, "age": _round(self._obstacles.age(now))}
        if msg is None:
            return out
        out.update(d=[int(v) for v in msg.distances],
                   increment=round(float(msg.increment_deg), 3),
                   offset=round(float(msg.angle_offset_deg), 3),
                   min_cm=int(msg.min_distance_cm),
                   max_cm=int(msg.max_distance_cm))
        return out

    def _power_state(self):
        if not self.p["allow_power"]:
            return {"available": False, "locked": True,
                    "reason": "power controls are disabled by the allow_power "
                              "parameter", "lock_reason": "", "hostname": ""}
        ok, why = power_client.available(self.p["power_socket"])
        known, armed = self._armed()
        locked, lock_reason = False, ""
        if armed:
            locked, lock_reason = True, "vehicle is ARMED"
        elif not known:                # unsafe on purpose; see _armed
            locked, lock_reason = True, ("autopilot status unknown — cannot "
                                         "confirm the vehicle is disarmed")
        return {"available": ok, "reason": why, "locked": locked,
                "lock_reason": lock_reason, "hostname": self._hostname}

    def _xy(self, latlon):
        """A [lat, lon] pair as [x, y] in the trail's frame, 2 dp. None when there
        is no origin yet, or the pair is missing or not two finite numbers."""
        try:
            lat, lon = float(latlon[0]), float(latlon[1])
        except (TypeError, ValueError, IndexError):
            return None
        if self._origin is None or not (math.isfinite(lat) and math.isfinite(lon)):
            return None
        x, y = geo.latlon_to_xy(lat, lon, self._origin)
        return [round(x, 2), round(y, 2)]

    def _nav_state(self, now):
        """The planned leg for the map: path, carrot ("target") and goal in the
        trail's own frame, so they sit where the trail does. A leg status that
        has gone stale is a BLANK, never the last path: bt_runner publishes at
        2 Hz while a leg runs, so silence means no leg or no bt_runner."""
        leg = self._nav_leg.get(now)
        out = {"ok": leg is not None, "age": _round(self._nav_leg.age(now)),
               "state": None, "why": "", "blocked_s": None, "mode": None,
               "path": [], "target": None, "goal": None}
        if leg is None:
            return out
        try:
            blocked = _round(float(leg["blocked_s"]), 1)
        except (KeyError, TypeError, ValueError):
            blocked = None
        path = leg.get("path")
        points = [self._xy(pt) for pt in (path if isinstance(path, list) else [])]
        out.update(state=leg.get("state"), why=str(leg.get("why") or ""),
                   blocked_s=blocked, mode=leg.get("mode"),
                   path=[pt for pt in points if pt is not None],
                   target=self._xy(leg.get("target")), goal=self._xy(leg.get("goal")))
        return out

    def _target_items(self, msg):
        if msg is None or self._origin is None:
            return []
        out = []
        for t in msg.targets:
            x, y = geo.latlon_to_xy(t.latitude, t.longitude, self._origin)
            out.append({
                "id": int(t.id), "label": t.label,
                "conf": round(float(t.confidence), 2),
                "lat": t.latitude, "lon": t.longitude,
                "x": round(x, 2), "y": round(y, 2),
                "range": round(t.range, 2), "bearing": round(t.bearing, 1),
                "stddev": round(t.position_stddev, 2),
                "hits": int(t.hits), "unseen": round(t.time_since_seen, 1),
                "confirmed": bool(t.confirmed), "sources": int(t.sources),
            })
        return out

    # ---------- actions ----------

    def _action(self, path, payload):
        """Route one POST. Every rule is re-checked here, on arrival."""
        if path == "/node/start":
            return self._start(payload)
        if path == "/node/stop":
            return self._stop(payload)
        if path == "/profile/start":
            return self._start_profile(payload)
        if path == "/power":
            return self._power(payload)
        if path == "/lake/start":
            return self._lake_start(payload)
        if path == "/lake/stop":
            return self._lake_stop(payload)
        if path in mission_api.LIB_PATHS:
            return self.mission.handle(path, payload)
        if path in mission_api.RUN_PATHS:
            return self.mission_run.handle(path, payload)
        if path == "/trail/clear":
            self._trail.clear()
            return {"ok": True, "message": "trail cleared"}
        if path == "/params/list":
            return self._params_list(payload)
        if path == "/params/set":
            return self._params_set(payload)
        if path == "/planner/profile/list":
            return self._planner_profiles()
        if path == "/planner/profile/save":
            return self._planner_profile_save(payload)
        if path == "/logs":
            return self._logs(payload)
        if path == "/logs/clear":
            self.logs.clear()
            return {"ok": True, "message": "log buffer cleared"}
        if path == "/radio":
            return self._radio_read(payload)
        if path == "/radio/clear":
            self._radio.clear()
            return {"ok": True, "message": "radio log cleared"}
        if path == "/radio/send_test":
            return self._radio_send_test()
        if path == "/camera/profile/list":
            return self._camera_profiles()
        if path == "/camera/profile/save":
            return self._camera_profile_save(payload)
        if path == "/camera/profile/delete":
            return self._camera_profile_delete(payload)
        if path == "/record/topics":
            return self._record_topics()
        if path == "/record/start":
            return self._record_start(payload)
        if path == "/record/stop":
            return self._record_stop()
        if path == "/record/delete":
            ok, message = self.recorder.delete(payload.get("name", ""))
            return {"ok": ok, "message": message}
        return {"ok": False, "message": f"unknown action {path}"}

    def _logs(self, payload):
        """Incremental log read. The page sends the newest seq it already has,
        so a Logs tab left open costs one line per new line rather than
        resending the whole ring five times a second."""
        try:
            since = int(payload.get("since", 0))
            level = int(payload.get("level", 10))
        except (TypeError, ValueError):
            return {"ok": False, "message": "since/level must be integers"}
        node = payload.get("node") or None
        records, newest, dropped = self.logs.read(since, level, node)
        return {"ok": True, "message": "", "records": records,
                "newest": newest, "dropped": dropped}

    def _on_radio(self, msg: RadioFrame):
        """One frame off the mesh. The stamp is the LINK NODE's clock, not this
        one's: both are this boat, so they share a time base."""
        self._radio.add(msg.direction, msg.src_system, msg.src_component,
                        msg.dst_system, msg.msg_name, msg.payload_type,
                        msg.summary, msg.frame_bytes,
                        stamp=msg.header.stamp.sec
                        + msg.header.stamp.nanosec * 1e-9)

    def _radio_read(self, payload):
        """Incremental read of the radio record, plus the two summaries. Same
        cursor contract as _logs, for the same reason."""
        try:
            since = int(payload.get("since", 0))
            limit = min(int(payload.get("limit", 300)), 1000)
        except (TypeError, ValueError):
            return {"ok": False, "message": "since/limit must be integers"}
        rows, newest, dropped = self._radio.read(since, limit)
        return {"ok": True, "message": "", "rows": rows, "newest": newest,
                "dropped": dropped, "systems": self._radio.systems(),
                "streams": self._radio.streams()}

    def _radio_send_test(self):
        """Ask rxl_link_node to put one test frame on the air.

        WAITING ON THE FUTURE IS SAFE ONLY BECAUSE this runs on the HTTP thread,
        never in a ROS callback, so the main thread's executor completes the
        call. Polling done(), not spin_until_future_complete: spinning here
        would fight the executor that already owns this node.
        """
        if not self._radio_test.service_is_ready():
            return {"ok": False,
                    "message": "rxl_link_node is not offering "
                               "/crsd/radio/send_test — is it running?"}
        fut = self._radio_test.call_async(Trigger.Request())
        deadline = time.monotonic() + RADIO_TEST_TIMEOUT_S
        while not fut.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not fut.done():
            return {"ok": False,
                    "message": "rxl_link_node did not answer within %.0fs; "
                               "nothing has changed as far as this page knows"
                               % RADIO_TEST_TIMEOUT_S}
        res = fut.result()
        return {"ok": bool(res.success), "message": res.message}

    def _param_call(self, payload, call):
        """(node, result, error) for one parameter endpoint. Two of three are set.

        The target must be in the ROS graph, checked before any service call so
        an unreachable name fails at once instead of after a full service
        timeout. ParamBridge raises rather than returning a sentinel; turning
        that into an operator's sentence is this HTTP layer's job.

        Args:
          payload: the POST body; its "node" key names the target.
          call: one-arg callable, given the node name, returning the result.
        """
        name = payload.get("node") or ""
        if name not in self._graph_full:
            return name, None, {"ok": False,
                                "message": f"{name or 'no node'} is not in "
                                           "the ROS graph"}
        try:
            return name, call(name), None
        except (TimeoutError, RuntimeError) as exc:
            return name, None, {"ok": False, "message": str(exc)}

    def _params_list(self, payload):
        """One node's parameters, described, valued, and set against the YAML."""
        name, params, err = self._param_call(
            payload, lambda n: self.tuning.list(n, _yaml_defaults(n)))
        return err or {"ok": True, "message": "", "node": name,
                       "params": params}

    def _params_set(self, payload):
        """Apply {name: value} to one node and report what it said.

        Every result comes back, successes included, and the node's reason
        unedited: accepted-then-clamped differs from taken verbatim, and the
        operator is about to act on which happened.
        """
        values = payload.get("values")
        if not isinstance(values, dict) or not values:
            return {"ok": False, "message": "no values to set"}
        name, results, err = self._param_call(
            payload, lambda n: self.tuning.set(n, values))
        if err:
            return err

        bad = [r for r in results if not r["ok"]]
        # Logged as well as returned, so a change made from a browser can be
        # found afterwards in /rosout (Logs tab, recordings).
        # TWO CALL SITES, DELIBERATELY, not `(log.info if ok else log.warn)(line)`:
        # rclpy caches logger configuration per SOURCE LINE, so one line at two
        # severities raises `Logger severity cannot be changed between calls`,
        # which replaced the node's real refusal in the HTTP reply. A mock
        # cannot reproduce it; only real rclpy does.
        for r in results:
            line = (f"{name} {r['name']} <- {values.get(r['name'])!r}: "
                    + ("applied" if r["ok"] else f"REFUSED — {r['reason']}"))
            if r["ok"]:
                self.get_logger().info(line)
            else:
                self.get_logger().warn(line)
        message = ("; ".join(f"{r['name']}: {r['reason'] or 'refused'}"
                             for r in bad) if bad
                   else f"applied {', '.join(sorted(values))} on "
                        f"{name.lstrip('/')}")
        return {"ok": not bad, "message": message, "results": results}

    def _planner_save_dir(self):
        return os.path.expanduser(self.p["planner_profiles_save_dir"])

    def _planner_listing(self):
        """The planner profiles on disk, from both directories, ~ expanded. Every
        profile endpoint answers with this, so each refreshes the page the same way."""
        return planner_profiles.listing([
            (planner_profiles.SHIPPED,
             os.path.expanduser(self.p["planner_profiles_dir"])),
            (planner_profiles.SAVED, self._planner_save_dir())])

    def _planner_profiles(self):
        """The planner profiles on disk, and why a directory is empty if it is.

        THERE IS NO /planner/profile/load, for the camera tab's reason: loading one is
        /params/set on bt_runner_node with the keys listed here, so the node's own
        set-callback judges every value and the operator reads one kind of refusal.
        """
        return {"ok": True, "message": "", **self._planner_listing()}

    def _planner_profile_save(self, payload):
        """Save bt_runner_node's LIVE drifted planner keys under a name.

        The browser sends a name, never values, for the camera tab's reason: what the
        page is showing may be a poll old or another browser's set, and a profile is a
        claim about what the planner was actually using. The node is read here, at save
        time, and "drifted" means differing from crusader_params.yaml -- the same marker
        the Tuning tab draws -- so the file holds exactly the lines a person would
        otherwise write back by hand, and loads with LAKE_TUNING= as it is.
        """
        node, params, err = self._param_call(
            {"node": planner_profiles.NODE_PATH},
            lambda n: self.tuning.list(n, _yaml_defaults(n)))
        if err:
            return err
        values, note = planner_profiles.drifted_planner_values(params)
        if not values:
            return {"ok": False, "message": note or "no planner key differs from "
                    "crusader_params.yaml - nothing to save"}
        ok, message, _path = planner_profiles.save(
            self._planner_save_dir(), payload.get("name"), values)
        if ok:
            self.get_logger().info(f"planner profile {payload.get('name')!r}: {message}")
        return {"ok": ok, "message": message, **self._planner_listing()}

    def _camera_profiles(self):
        """The stored profiles, plus which parameters count as camera controls.

        THERE IS NO /camera/profile/load, on purpose. Recalling a profile is
        `/params/set` with the stored mapping — same refusals, same log lines. A
        second apply path would be a second place for the rules to drift.

        `controls` is the ORDER AND GROUPING ONLY. Ranges, choices, editability
        and the YAML comparison come from /params/list, which describes the
        running node; sending them twice is how the page shows a bound the node
        does not enforce.
        """
        names, err = _camera_control_groups()
        return {"ok": True, "message": "", "profiles": camera_profiles.listing(),
                "store": camera_profiles.status(), "controls": names,
                "controls_error": err}

    def _camera_profile_save(self, payload):
        """Save the camera's LIVE values under a name.

        The browser sends a name and a node, never values: a profile is a claim
        about what the camera was doing, and what the page displays may miss
        another browser's set or a clamp the node applied. Read from the node,
        at save time.
        """
        name = payload.get("name")
        control_groups, err = _camera_control_groups()
        if err:
            return {"ok": False, "message": err}
        node, params, call_err = self._param_call(
            payload, lambda n: self.tuning.list(n, _yaml_defaults(n)))
        if call_err:
            return call_err

        wanted = {c for group in control_groups for c in group["controls"]}
        live = {p["name"]: p["value"] for p in params if p["name"] in wanted}
        missing = sorted(wanted - set(live))
        if missing:
            # Never a partial block: a profile with holes silently inherits
            # whatever the previous one left in those knobs.
            return {"ok": False,
                    "message": f"{node.lstrip('/')} has no "
                               f"{', '.join(missing[:3])}"
                               + (f" (+{len(missing) - 3} more)"
                                  if len(missing) > 3 else "")
                               + " — is it oak_detector?"}

        ok, message = camera_profiles.save(name, live)
        if ok:
            self.get_logger().info(
                f"camera profile {name!r} <- {node.lstrip('/')}: {message}")
        return {"ok": ok, "message": message,
                "profiles": camera_profiles.listing(),
                "store": camera_profiles.status()}

    def _camera_profile_delete(self, payload):
        ok, message = camera_profiles.delete(payload.get("name"))
        if ok:
            self.get_logger().info(f"camera profile: {message}")
        return {"ok": ok, "message": message,
                "profiles": camera_profiles.listing(),
                "store": camera_profiles.status()}

    def _record_topics(self):
        """Every topic in the graph, typed, for the Record tab's checkboxes.

        On demand, not in /state: a couple of KB resent five times a second to
        be looked at once per session.

        The LIVE GRAPH, not a curated set: a recording is worth making because
        something unexpected happened, which is when a list chosen weeks
        earlier turns out wrong.
        """
        try:
            pairs = self.get_topic_names_and_types()
        except Exception as e:
            return {"ok": False, "message": f"could not read the graph: {e}"}
        return {"ok": True, "message": "",
                "topics": bag_recorder.describe_topics(pairs)}

    def _record_start(self, payload):
        """Begin a session, and a bag inside it if any topics were ticked.

        Order matters: the bag goes INSIDE the session directory the Recorder
        creates, so `ros2 bag record --output` gets a path whose parent exists
        and whose own name does not, and the bag is in the Download tarball.
        """
        topics = payload.get("topics") or []
        if not isinstance(topics, list) or any(not isinstance(t, str)
                                               for t in topics):
            return {"ok": False, "message": "topics must be a list of names"}

        # PER SESSION, deliberately NOT a parameter write. A sticky 30 fps is
        # 1 MB/s (~3.6 GB an hour) on every later session too, and the first
        # feedback would be the disk guard stopping a run you needed.
        sources = self._record_sources()
        spec = PARAM_SPEC["record_frame_hz"]
        rates, notes = frame_rates(payload.get("frame_hz"), sources.keys(),
                                   self.p["record_frame_hz"],
                                   spec["lo"], spec["hi"])

        ok, message = self.recorder.start(sources, rates)
        if not ok:
            return {"ok": False, "message": message}
        # Clamp notes only once the session is real; on a refusal they would
        # bury the reason.
        if notes:
            message += " (" + "; ".join(notes) + ")"
        if not topics:
            return {"ok": True, "message": message}

        bag_dir = os.path.join(self.recorder.session.dir, "bag")
        bag_ok, bag_message = self.bags.start(bag_dir, topics)
        # A failed bag does NOT fail the session (telemetry and frames are
        # already being written), but the operator must be told, or they sail
        # on believing the topics are captured.
        return {"ok": True,
                "message": message + ("; " + bag_message if not bag_ok
                                      else f"; {bag_message}")}

    def _record_stop(self):
        """Stop the bag FIRST, then the session.

        Otherwise meta.json records the size from before rosbag2 flushed.
        """
        bag_message = ""
        if self.bags.recording:
            _ok, bag_message = self.bags.stop()
        ok, message = self.recorder.stop()
        return {"ok": ok,
                "message": message + (f"; {bag_message}" if bag_message else "")}

    def archive(self, name, fileobj):
        """Stream one recording session out as .tar.gz. Used by the GET path."""
        return self.recorder.archive_into(name, fileobj)

    def _start(self, payload):
        name = payload.get("name")
        spec = reg.BY_NAME.get(name)
        if spec is None:
            return {"ok": False, "message": f"unknown node {name!r}"}
        _items, running = self._node_items()
        if name in running:
            return {"ok": True, "message": f"{name} is already running"}

        clash = reg.conflicts(name, running)
        if clash and not payload.get("stop_conflicts"):
            return {"ok": False,
                    "message": f"{', '.join(clash)} holds the same device — "
                               "confirm to stop it first"}
        for other in clash:
            allowed, why = reg.may_stop(other)
            if not allowed:
                return {"ok": False, "message": why}
            self.procs.stop(other)

        ok, message = self.procs.start(spec)
        return {"ok": ok, "message": message}

    def _stop(self, payload):
        name = payload.get("name")
        allowed, why = reg.may_stop(name)
        if not allowed:
            self.get_logger().warn(f"refused stop of {name!r}: {why}")
            return {"ok": False, "message": why}
        if name in self.procs.running():
            ok, message = self.procs.stop(name)      # ours: signal the group
        else:
            spec = reg.BY_NAME.get(name)
            if spec is None:
                return {"ok": False, "message": f"unknown node {name!r}"}
            pids = self._proc.get(spec.executable, [])
            # Foreign process: signal the PID only, never the group. A node
            # started by core.launch.py shares its group with the whole launch.
            ok, message = self.procs.stop_external(name, pids)
        return {"ok": ok, "message": message}

    def _start_profile(self, payload):
        profile = reg.PROFILES.get(payload.get("name"))
        if profile is None:
            return {"ok": False, "message": f"unknown profile {payload!r}"}
        _label, names = profile
        _items, running = self._node_items()
        started, skipped = [], []
        for name in names:
            if name in running:
                skipped.append(name)
                continue
            # A profile never resolves a device conflict: silently stopping a
            # camera node someone deliberately chose is how a run gets lost.
            if reg.conflicts(name, running):
                skipped.append(name)
                continue
            ok, _msg = self.procs.start(reg.BY_NAME[name])
            (started if ok else skipped).append(name)
            running.add(name)
        return {"ok": True,
                "message": f"started {len(started)}"
                           + (f", skipped {', '.join(skipped)}" if skipped else "")}

    def _power(self, payload):
        """Shut down or reboot the HOST, through the helper. All gates re-run."""
        if not self.p["allow_power"]:
            return {"ok": False,
                    "message": "power controls are disabled (allow_power)"}
        verb = payload.get("verb")
        if verb not in power_client.VERBS:
            return {"ok": False, "message": f"unknown power verb {verb!r}"}

        state = self._power_state()
        if state["locked"]:
            return {"ok": False, "message": f"refused: {state['lock_reason']}"}
        if (payload.get("confirm") or "").strip() != self._hostname:
            return {"ok": False,
                    "message": f"type the hostname ({self._hostname}) to confirm"}

        self.get_logger().warn(
            f"{verb.upper()} requested from the ground station and accepted "
            "(disarmed, hostname confirmed)")
        try:
            reply = power_client.request(
                verb, self.p["power_socket"],
                reason=f"ground_station on {self._hostname}")
        except (power_client.PowerUnavailable, ValueError) as e:
            return {"ok": False, "message": str(e)}
        return {"ok": True, "message": reply}

    # ---------- the lake rig (Task 1 tab) ----------

    def _rig_blocked(self):
        """Why the rig must not be started or stopped right now, or None. Restarting it
        kills its bt_runner, so never under a mission: armed in GUIDED or AUTO is the
        pilot's to end first. An unknown FCU status is not a block; the rig's own
        preflight says so, and START in the panel needs armed + GUIDED anyway."""
        status = self._status.get(time.monotonic())
        if status is not None and status[1] and str(status[0]).upper() in ("GUIDED", "AUTO"):
            return (f"the autopilot is ARMED in {status[0]}: restarting the rig would cut a "
                    "running mission. The pilot takes it out of GUIDED first.")
        return None

    def _lake_start(self, payload):
        """START RIG: lake_rig_up.sh with the datum from the boat (lake_rig.choose_datum)."""
        why = self._rig_blocked()
        if why:
            return {"ok": False, "message": why}
        pose = self._pose.get(time.monotonic())
        if pose is None:
            return {"ok": False,
                    "message": "no fresh boat pose (GPS yaw resolved?): the datum comes "
                               "from the boat, so the rig cannot start without one"}
        lat, lon, chosen = lake_rig.choose_datum(
            pose[:2], self.rig.field(), new_here=bool(payload.get("new_datum")))
        ok, message = self.rig.start((lat, lon), chosen, bool(payload.get("pool")),
                                     bool(payload.get("publish")))
        return {"ok": ok, "message": message}

    def _lake_stop(self, _payload):
        """STOP RIG: lake_rig_down.sh, which stops only what the rig started."""
        why = self._rig_blocked()
        if why:
            return {"ok": False, "message": why}
        ok, message = self.rig.stop()
        return {"ok": ok, "message": message}

    def _lake_state(self, pose):
        """The rig as the Task 1 tab shows it, plus the datum a START RIG would use now
        (None without a fresh pose: the button would be refused, and the tab says why)."""
        state = self.rig.view()
        state["next"] = None
        if pose is not None:
            lat, lon, chosen = lake_rig.choose_datum(pose[:2], self.rig.field())
            state["next"] = {"lat": lat, "lon": lon, "why": chosen}
        return state

    # ---------- the Mission tab ----------

    def _mission_bt_dir(self):
        """The behavior_trees directory: the param if set, else crusader_bt's installed share."""
        if self.p["mission_bt_dir"]:
            return os.path.expanduser(self.p["mission_bt_dir"])
        from ament_index_python.packages import (PackageNotFoundError,
                                                 get_package_share_directory)
        try:
            return os.path.join(get_package_share_directory("crusader_bt"), "behavior_trees")
        except PackageNotFoundError:
            # The ground station must not fail to start because one tab's package is absent.
            self.get_logger().warn("crusader_bt is not installed: the Mission tab has no "
                                   "subtrees or pre-built missions (set mission_bt_dir)")
            return ""

    def _mission_set_tree_file(self, path):
        """(ok, reason): the BT runner's tree_file parameter <- path. The runner reads it when
        it accepts the next goal, so it is set before the goal is sent."""
        name = self.p["mission_runner_node"]
        if name not in self._graph_full:
            return False, f"{name} is not in the ROS graph (is the BT runner running?)"
        try:
            results = self.tuning.set(name, {"tree_file": path})
        except (TimeoutError, RuntimeError) as exc:
            return False, str(exc)
        bad = [r for r in results if not r["ok"]]
        if bad:
            return False, "; ".join(f"{r['name']}: {r['reason'] or 'refused'}" for r in bad)
        return True, ""

    # ---------- teardown ----------

    def destroy_node(self):
        """Stop the server, but LEAVE THE NODES RUNNING.

        Closing the dashboard must not take the boat's stack down; the
        processes run in their own session precisely so they survive this.
        """
        self.server.stop()
        # So an interrupted recording is still readable.
        if self.recorder.session and self.recorder.session.stopped is None:
            self.recorder.stop()
        super().destroy_node()


def _port_open(port, host="127.0.0.1", timeout=0.25):
    """Is something listening there right now? A loopback connect: sub-ms on
    success, immediate RST when nothing is bound. Run per graph tick, not per
    browser poll.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            return s.connect_ex((host, port)) == 0
    except OSError:
        return False


def _free_bytes(path):
    """Free space where recordings are written, or None off a real filesystem.

    None rather than 0 when statvfs is unavailable, so the hours-remaining
    figure goes blank rather than reading "no time left".
    """
    gb = recorder_free_gb(path)
    return None if gb is None else gb * 1073741824.0


YAML_SECTION_ALIASES = {"dock_view": "oak_detector"}


def _yaml_defaults(node_name):
    """crusader_params.yaml's section for a node, or {} when it has none.

    Not an error: nodes started by hand have real parameters and no entry, and
    the tab must open for them, saying it has nothing to compare against
    rather than implying agreement.
    """
    name = node_name.rstrip("/").split("/")[-1]
    # tools/dock_view.py has no section of its own: it opens the same camera
    # with oak_detector's camera values, so that is what it is compared against.
    name = YAML_SECTION_ALIASES.get(name, name)
    try:
        return crsd_config.node_params(name)
    except Exception:
        return {}


def _camera_control_groups():
    """oak_controls' table as [{group, controls:[name]}], or (None, why).

    IMPORTED FROM crusader_perception RATHER THAN RESTATED: oak_controls exists
    so a knob's name lives in one place, and a copied list fails silently — add
    a control and the Camera tab quietly omits it. Group and control order are
    table order, so the page has no ordering knowledge of its own.

    This knowingly breaks package.xml's rule that the ground station imports no
    package it launches, so it is as narrow as possible: SOFT, so a missing
    crusader_perception is a tab with nothing to tune, not a ground station that
    will not start; and NOT in package.xml, so depthai and TensorRT do not
    become build dependencies. The proper fix is for oak_detector to advertise
    the set as a read-only parameter, read through /params/list.
    """
    try:
        from crusader_perception import oak_controls
    except Exception as e:
        return None, f"camera controls unavailable: {e}"
    groups, order = {}, []
    for c in oak_controls.ALL:
        if c.group not in groups:
            groups[c.group] = []
            order.append(c.group)
        groups[c.group].append(c.name)
    return [{"group": g, "controls": groups[g]} for g in order], ""


def _round(v, n=2):
    return None if v is None else round(v, n)


def main(args=None):
    run_node(GroundStation, args=args)


if __name__ == "__main__":
    main()
