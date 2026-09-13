"""ground_station — the whole boat in one browser tab.

NOTE: unverified on the boat. It starts and stops real nodes and can power the
Jetson off, and none of that has been exercised on the water. Deliberately NOT
in core.launch.py.

    ros2 run crusader_groundstation ground_station
    # then, on the laptop:  http://<JETSON_IP>:8090

Subscribes (read-only): /crsd/pose, /crsd/attitude, /crsd/fcu_status,
crsd/world_targets. Publishes nothing. Its only outward effects are the
processes it spawns and the two power verbs it can hand to the host helper —
both gated, both re-checked server-side.

Port 8090, clear of the viewers it embeds: 8080 is buoy_detector's annotated
view and tools/oak_view.py, 8081 is tools/lidar_view.py. The camera and LiDAR
tabs are iframes onto those, not a second implementation of streaming — and
because mjpeg_server counts clients and skips rendering when nobody is
attached, a tab the operator has not opened costs the Jetson nothing.

WHAT IT REPLACES. crusader_world_model's map_server moved here and became tab
5. The map is a display, and a display of world state belongs with the rest of
the operator's controls rather than inside the package that computes it — which
also puts the testable half of the world model back to pure geometry, with no
HTTP server bolted to it.

THE TUNING TAB WRITES TO OTHER NODES, and it is the only thing here that
does. Everything else in this file reads: four subscriptions, a process table,
a disk. `/params/set` reaches into a running node and changes a value, which is
a real outward effect and is why it is confined to what the TARGET node already
declared dynamic — a read-only parameter is refused by the node itself, not by
a rule kept here. It cannot persist anything: crusader_params.yaml stays the
source of truth, the page shows live-versus-YAML drift, and writing the file
back from a browser would destroy the comments that are most of its value.

THE TWO RULES THIS NODE HOLDS, and holds again on every request no matter what
the page rendered:

  PROTECTED NODES CANNOT BE STOPPED FROM HERE. The repo's second safety
  constraint says the RC e-stop is the only safety path and WiFi is never a
  safety mechanism; a network-reachable off switch for rc_watchdog would undo
  that. Starting them is allowed — that can only move the boat toward safe.

  POWER IS LOCKED WHILE ARMED, and then needs the hostname typed. A reboot
  button one stray click away from a boat under way is not a button.
"""
import json
import math
import os
import socket
import time
from collections import deque

from rclpy.node import Node

from rcl_interfaces.msg import Log

from crusader_msgs.msg import (Attitude, Cluster3DArray, FcuStatus,
                               LatLonHead, ObstacleDistance,
                               TrackedTargetArray)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config, make_set_callback
from crusader_common.stream_cache import StreamCache

from crusader_groundstation import node_registry as reg
from crusader_groundstation import bag_recorder, param_client, power_client
from crusader_groundstation import proc_scan, system_info
from crusader_groundstation.log_buffer import LogBuffer
from crusader_groundstation.recorder import Recorder
from crusader_groundstation.recorder import free_gb as recorder_free_gb
from crusader_groundstation.gcs_page import render as render_page
from crusader_groundstation.gcs_server import GcsServer
from crusader_groundstation.process_manager import ProcessManager

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
    "record_frame_hz": dict(read_only=False, lo=0.05, hi=10.0,
                            description="frames saved per second per viewer"),
    "record_min_free_gb": dict(read_only=False, lo=0.5, hi=500.0,
                               description="stop recording below this"),
}


class GroundStation(Node):
    """Subscriptions and a process table in, one JSON snapshot out."""

    def __init__(self):
        super().__init__("ground_station")
        p = declare_from_config(self, crsd_config.node_params("ground_station"),
                                PARAM_SPEC)
        self.p = p

        ranges = {n: (s["lo"], s["hi"]) for n, s in PARAM_SPEC.items()
                  if not s.get("read_only") and "lo" in s}
        self.add_on_set_parameters_callback(
            make_set_callback(self, ranges, self._apply))

        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["attitude_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._targets = StreamCache(p["targets_timeout_s"])
        # Map layers. Subscribed always, sent only when a page asks — see
        # _snapshot. A subscription costs one callback per message; putting
        # sixty clusters into every /state for every client is what costs.
        self._clusters = StreamCache(p["clusters_timeout_s"])
        self._obstacles = StreamCache(p["obstacle_timeout_s"])

        self._origin = None
        self._trail = deque(maxlen=int(p["trail_length"]) or 1)
        self._cpu = system_info.CpuMeter()
        self._hostname = socket.gethostname()

        self.procs = ProcessManager(
            tools_dir=p["tools_dir"],
            logger=lambda m: self.get_logger().info(m))

        # Parameter services against every OTHER node. Clients are created on
        # first use, so a session that never opens the Tuning tab adds no graph
        # entities at all.
        self.tuning = param_client.ParamBridge(self)

        # /rosout rather than journalctl: we are inside a container and the
        # host journal is on the other side of that boundary, while /rosout
        # crosses the DDS domain and needs no privilege. See log_buffer.
        self.logs = LogBuffer(int(p["log_capacity"]))
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)

        self.recorder = Recorder(p["record_dir"], p["record_min_free_gb"],
                                 logger=lambda m: self.get_logger().warn(m))
        # The bag is a SEPARATE child process writing into the same session
        # directory. Separate because it must be stopped with SIGINT to be
        # readable at all, and because /livox/lidar must never go through this
        # node's executor — see bag_recorder's header for both.
        self.bags = bag_recorder.BagRecorder(
            logger=lambda m: self.get_logger().info(m))
        self._persist = self._check_persistence(p["record_dir"])
        self.create_timer(1.0 / p["record_telemetry_hz"], self._record_sample)

        # One /proc pass per graph tick, alongside the graph scan. The two see
        # different worlds and we need both — see proc_scan's header.
        self._proc = {}
        self._serving = set()

        # The ROS graph is scanned on a timer rather than per request:
        # get_node_names() is a discovery call, and running it once per browser
        # poll per client would put graph traffic on the wire in proportion to
        # how many people have the page open.
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

        Checked at construction rather than per poll: mounts do not change
        under a running container, and the operator needs the answer BEFORE a
        session rather than after losing one.
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

    def _scan_graph(self):
        """Which registry nodes are present in the ROS graph right now.

        This is how a node started by core.launch.py, by systemd, or by hand in
        another terminal shows as running. Nobody wants a dashboard reporting
        the telemetry bridge is down because it did not personally start it.
        """
        try:
            pairs = self.get_node_names_and_namespaces()
            self._graph = {n for n, _ns in pairs}
            # Fully qualified as well, for the parameter services. A node in a
            # namespace answers /ns/node/set_parameters, and the bare name
            # addresses a node that does not exist — which fails as a timeout,
            # i.e. as "that node is down", which it is not.
            self._graph_full = param_client.visible_nodes(
                ns.rstrip("/") + "/" + n for n, ns in pairs)
        except Exception:
            pass                       # discovery hiccup; keep the last answer
        # One /proc pass for every registry entry at once. Catches what the
        # graph cannot: the tools/ viewer scripts, which are not ROS nodes, and
        # anything started from a terminal. See proc_scan.
        self._proc = proc_scan.scan([s.executable for s in reg.REGISTRY])
        # Which viewer ports are actually ACCEPTING connections. A process in
        # the table is not a server that has bound its socket yet, and telling
        # the page otherwise is what left an iframe stuck on connection-refused.
        self._serving = {s.name for s in reg.REGISTRY
                         if s.port and _port_open(s.port)}

    def _on_rosout(self, msg: Log):
        """Every node's logger output, from anywhere in the DDS domain.

        Filtered to our own nodes' names would be wrong: the point of the tab
        is to see what is actually happening, and the livox driver complaining
        is exactly the kind of thing that otherwise needs an SSH session.
        """
        self.logs.add(msg.name, msg.level, msg.msg,
                      stamp=msg.stamp.sec + msg.stamp.nanosec * 1e-9)

    def _record_sample(self):
        """One telemetry line, if a session is live. Also runs the disk guard."""
        self.recorder.tick()
        self.bags.tick()
        # The Recorder's disk guard stops the SESSION when free space crosses
        # the floor. A bag left running past that point would keep filling the
        # disk the guard just fired to protect, and would do it invisibly,
        # because the session that was reporting the size has closed.
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
        # Freshness is recorded as a FIELD, not as a reason to skip the sample.
        # A gap in the file is indistinguishable from the recorder being dead;
        # an explicit "pose_ok": false is the thing worth having afterwards.
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
        that is not running contributes nothing — which is honest: there is no
        camera footage to record from a camera nobody turned on.
        """
        _items, running = self._node_items()
        sources = {}
        for key, names in (("camera", reg.CAMERA_TAB_SOURCES),
                           ("lidar", reg.LIDAR_TAB_SOURCES)):
            src, starting = reg.tab_source(names, running, self._serving)
            if src and not starting:
                spec = reg.BY_NAME[src]
                if spec.port and spec.stream_path:
                    sources[key] = (f"http://127.0.0.1:{spec.port}"
                                    f"{spec.stream_path}")
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
                # Stoppable if the rule allows it and we can actually reach
                # the process — either we own it, or /proc gave us a real PID.
                # The old objection was that stopping a foreign node meant
                # guessing a PID from a name; proc_scan removes the guess, so
                # the objection goes with it.
                "stoppable": bool(allowed and (spec.name in owned or pids)),
                "pids": pids,
                "stop_reason": why or "",
                "tail": self.procs.tail(spec.name)[-8:],
            })
        for item in items:
            item["conflicts"] = reg.conflicts(item["name"], running_names)
        return items, running_names

    def _viewer_tab(self, sources, title, hint, running):
        """Which node is filling a viewer tab, given the already-computed set.

        Takes `running` rather than recomputing it: _node_items() walks the
        registry and asks the process table about every entry, and calling it
        once per tab meant doing that three times for one browser poll.
        """
        src, starting = reg.tab_source(sources, running, self._serving)
        return {
            # `source` is only set once the port answers, so the page never
            # points an iframe at a socket that is not listening yet.
            "source": None if starting else src,
            "starting": starting,
            "starting_name": src if starting else None,
            "port": reg.BY_NAME[src].port if src else 0,
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
            "nodes": {
                "groups": [{"id": g, "label": lbl, "hint": hint}
                           for g, lbl, hint in reg.GROUPS],
                "items": items,
                "up": len(running),
            },
            "profiles": [{"id": k, "label": v[0]}
                         for k, v in reg.PROFILES.items()],
            # NAMES ONLY. The Tuning tab's values come from /params/list, on
            # demand, because three parameter service round trips per browser
            # poll per client would put graph traffic on the wire in proportion
            # to how many people have the page open — the same reason the ROS
            # graph is scanned on a timer. A couple of hundred bytes of node
            # names is inside the poll budget; a parameter table is not.
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
                "telemetry_hz": self.p["record_telemetry_hz"],
                "min_free_gb": self.p["record_min_free_gb"],
                "dir": self.p["record_dir"],
                # Whether a recording survives the container being recreated is
                # the one thing about it worth stating on screen, and it is not
                # guessable from the path — see system_info.mount_for.
                "persist": self._persist,
                # Status only — small, and the operator needs the growth rate
                # in front of them while it runs. The topic table is fetched
                # once, by /record/topics, when the tab is opened.
                "bag": self.bags.status(
                    _free_bytes(self.p["record_dir"])),
            },
        }
        # Optional map layers, built ONLY when a page asked for them. With a
        # shoreline in view the cluster layer is a few KB — an order of
        # magnitude more than everything else in /state put together, which is
        # cheap while you are verifying avoidance and pure waste five times a
        # second while you are not. Same reasoning as the Tuning tab fetching
        # on demand; the difference is that these belong in the map's own poll
        # rather than a separate request, because they have to be drawn
        # against the SAME boat position as the targets beside them.
        if "clusters" in layers:
            snap["clusters"] = self._cluster_layer(now, pose, att)
        if "prox" in layers:
            snap["prox"] = self._prox_layer(now)
        return snap

    def _cluster_layer(self, now, pose, att):
        """Raw LiDAR clusters, in BOTH frames, or an empty stale marker.

        Each cluster carries its BODY-frame position — what the sensor
        actually said, and the frame QGC's PRX1 view is drawn in — and its
        WORLD-frame position, so it can be laid over the tracks built from it.

        Both are computed here rather than rotating in the page, because
        geo.body_to_world_ypr is this repo's one implementation of that
        transform and a JavaScript copy would be a second one that drifts.
        It is the same argument tools/lidar_view.py makes for reading the
        LiDAR extrinsic out of the params file instead of restating it: a
        viewer that models the geometry differently from the node under test
        fails and passes for reasons that have nothing to do with the thing
        being checked.

        `placed` is false when pose or attitude is stale. The body-frame
        numbers are still true then — they do not depend on knowing where the
        boat is — so they are still sent, and the page falls back to the
        bow-up view rather than drawing the world layer at a position the
        boat has already left.
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
        """The 72 sectors exactly as the autopilot receives them.

        Not summarised, not rescaled, not cleaned up. The entire value of this
        layer is that it is what telemetry_bridge forwards as MAVLink
        OBSTACLE_DISTANCE, so laying it beside QGC's PRX1 view answers a
        question nothing else here can: if the two disagree, the fault is
        between this boat's ROS graph and the flight controller; if they agree
        with each other and disagree with the clusters, it is
        proximity_bridge's sector maths.

        UINT16_MAX survives as UINT16_MAX. ObstacleDistance.msg is explicit
        that it means NO READING and that 0 means touching the hull, so the
        page draws nothing for it rather than a maximum-range arc — "not seen"
        and "seen and clear" are different facts, and this is the repo that
        keeps insisting they must not render the same.
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
        elif not known:
            # Unknown is treated as unsafe on purpose: no FcuStatus usually
            # means the bridge is down, which is not evidence the boat is safe.
            locked, lock_reason = True, ("autopilot status unknown — cannot "
                                         "confirm the vehicle is disarmed")
        return {"available": ok, "reason": why, "locked": locked,
                "lock_reason": lock_reason, "hostname": self._hostname}

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
        if path == "/trail/clear":
            self._trail.clear()
            return {"ok": True, "message": "trail cleared"}
        if path == "/params/list":
            return self._params_list(payload)
        if path == "/params/set":
            return self._params_set(payload)
        if path == "/logs":
            return self._logs(payload)
        if path == "/logs/clear":
            self.logs.clear()
            return {"ok": True, "message": "log buffer cleared"}
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

    def _param_call(self, payload, call):
        """(node, result, error) for one parameter endpoint. Two of three are set.

        Both endpoints need the same preamble, so it lives here once: the
        target must be in the ROS graph — checked before any service call, so
        an unreachable name fails immediately instead of costing the browser a
        full service timeout to learn the same thing — and ParamBridge raises
        rather than returning a sentinel, because a caller that wanted the
        value has no use for one. Turning that into a sentence an operator can
        read is the HTTP layer's job, and there is one HTTP layer.

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

        Every result comes back, successes included, because a set that was
        accepted and then clamped by the node's own callback is a different
        outcome from one that took the value verbatim, and the operator is
        about to act on which of the two happened. The node's reason is passed
        through unedited for the same reason.
        """
        values = payload.get("values")
        if not isinstance(values, dict) or not values:
            return {"ok": False, "message": "no values to set"}
        name, results, err = self._param_call(
            payload, lambda n: self.tuning.set(n, values))
        if err:
            return err

        bad = [r for r in results if not r["ok"]]
        # Logged as well as returned: a parameter changed from a browser and
        # nowhere else is a change nobody can find afterwards, and /rosout is
        # what the Logs tab and any recording already capture.
        log = self.get_logger()
        for r in results:
            line = (f"{name} {r['name']} <- {values.get(r['name'])!r}: "
                    + ("applied" if r["ok"] else f"REFUSED — {r['reason']}"))
            (log.info if r["ok"] else log.warn)(line)
        message = ("; ".join(f"{r['name']}: {r['reason'] or 'refused'}"
                             for r in bad) if bad
                   else f"applied {', '.join(sorted(values))} on "
                        f"{name.lstrip('/')}")
        return {"ok": not bad, "message": message, "results": results}

    def _record_topics(self):
        """Every topic in the graph, typed, for the Record tab's checkboxes.

        On demand rather than in /state, for the Tuning tab's reason: a topic
        table is a couple of KB and would be resent to every client five times
        a second to be looked at once, at the start of a session.

        The list is the LIVE GRAPH, not a curated set. A recording is worth
        making because something unexpected happened, and a curated list is a
        judgement made weeks earlier about what would matter — which is exactly
        the judgement that turns out to be wrong on the day.
        """
        try:
            pairs = self.get_topic_names_and_types()
        except Exception as e:
            return {"ok": False, "message": f"could not read the graph: {e}"}
        return {"ok": True, "message": "",
                "topics": bag_recorder.describe_topics(pairs)}

    def _record_start(self, payload):
        """Begin a session, and a bag inside it if any topics were ticked.

        Order matters: the Recorder creates the session directory, and the bag
        goes INSIDE it, so `ros2 bag record --output` gets a path whose parent
        exists and whose own name does not. A bag started first would have
        nowhere to live, and one started outside the session would not be in
        the tarball the Download button produces.
        """
        topics = payload.get("topics") or []
        if not isinstance(topics, list) or any(not isinstance(t, str)
                                               for t in topics):
            return {"ok": False, "message": "topics must be a list of names"}

        ok, message = self.recorder.start(self._record_sources(),
                                          self.p["record_frame_hz"])
        if not ok or not topics:
            return {"ok": ok, "message": message}

        bag_dir = os.path.join(self.recorder.session.dir, "bag")
        bag_ok, bag_message = self.bags.start(bag_dir, topics)
        # A failed bag does NOT fail the session. Telemetry and frames are
        # already being written, and throwing those away because rosbag2 would
        # not start is the wrong trade — but the operator has to be told, or
        # they will sail on believing the topics are being captured.
        return {"ok": True,
                "message": message + ("; " + bag_message if not bag_ok
                                      else f"; {bag_message}")}

    def _record_stop(self):
        """Stop the bag FIRST, then the session.

        The bag has to finish writing into the session directory before the
        session writes its meta.json, or the size the session records is the
        size before rosbag2 flushed.
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
            # A profile never resolves a device conflict on its own. Silently
            # stopping the camera node someone deliberately chose, because a
            # profile lists the other one, is the kind of helpfulness that
            # loses a run.
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

    # ---------- teardown ----------

    def destroy_node(self):
        """Stop the server, but LEAVE THE NODES RUNNING.

        Closing the dashboard must not take the boat's stack down with it. The
        processes were put in their own session precisely so they survive this,
        and a ground station that kills the telemetry bridge on exit is one
        nobody dares restart mid-session.
        """
        self.server.stop()
        # Close the session file rather than letting the process exit with it
        # open: an interrupted recording should still be readable.
        if self.recorder.session and self.recorder.session.stopped is None:
            self.recorder.stop()
        super().destroy_node()


def _port_open(port, host="127.0.0.1", timeout=0.25):
    """Is something listening there right now?

    A plain TCP connect to loopback, which on the same host is sub-millisecond
    when it succeeds and immediate (RST) when nothing is bound. Run once per
    graph tick rather than per browser poll, so the cost does not scale with
    how many people have the page open.
    """
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(timeout)
            return s.connect_ex((host, port)) == 0
    except OSError:
        return False


def _free_bytes(path):
    """Free space where recordings are written, or None off a real filesystem.

    None rather than 0 when statvfs is unavailable: the hours-remaining figure
    it feeds must go blank rather than read as "no time left", which would be
    an alarm nobody could act on.
    """
    gb = recorder_free_gb(path)
    return None if gb is None else gb * 1073741824.0


def _yaml_defaults(node_name):
    """crusader_params.yaml's section for a node, or {} when it has none.

    Not having a section is not an error here. bt_runner_node and anything
    started by hand are real nodes with real parameters and no entry in the
    file, and the tab has to open for them — it just has nothing to compare
    their values against, which it says rather than implying agreement.
    """
    try:
        return crsd_config.node_params(node_name.rstrip("/").split("/")[-1])
    except Exception:
        return {}


def _round(v, n=2):
    return None if v is None else round(v, n)


def main(args=None):
    run_node(GroundStation, args=args)


if __name__ == "__main__":
    main()
