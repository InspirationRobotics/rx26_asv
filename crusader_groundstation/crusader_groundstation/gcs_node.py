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
import socket
import time
from collections import deque

from rclpy.node import Node

from rcl_interfaces.msg import Log

from crusader_msgs.msg import (Attitude, FcuStatus, LatLonHead,
                               TrackedTargetArray)

from crusader_common import config as crsd_config
from crusader_common import geo
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config, make_set_callback
from crusader_common.stream_cache import StreamCache

from crusader_groundstation import node_registry as reg
from crusader_groundstation import power_client, proc_scan, system_info
from crusader_groundstation.log_buffer import LogBuffer
from crusader_groundstation.recorder import Recorder
from crusader_groundstation.gcs_page import render as render_page
from crusader_groundstation.gcs_server import GcsServer
from crusader_groundstation.process_manager import ProcessManager

PARAM_SPEC = {
    "port": dict(read_only=True, lo=1024, hi=65535,
                 description="HTTP port; 8080/8081 are the viewers"),
    "bind_host": dict(read_only=True,
                      description="0.0.0.0 so the laptop can reach it"),
    "targets_topic": dict(read_only=True, description="TrackedTargetArray in"),
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

        self._origin = None
        self._trail = deque(maxlen=int(p["trail_length"]) or 1)
        self._cpu = system_info.CpuMeter()
        self._hostname = socket.gethostname()

        self.procs = ProcessManager(
            tools_dir=p["tools_dir"],
            logger=lambda m: self.get_logger().info(m))

        # /rosout rather than journalctl: we are inside a container and the
        # host journal is on the other side of that boundary, while /rosout
        # crosses the DDS domain and needs no privilege. See log_buffer.
        self.logs = LogBuffer(int(p["log_capacity"]))
        self.create_subscription(Log, "/rosout", self._on_rosout, 50)

        self.recorder = Recorder(p["record_dir"], p["record_min_free_gb"],
                                 logger=lambda m: self.get_logger().warn(m))
        self._persist = self._check_persistence(p["record_dir"])
        self.create_timer(1.0 / p["record_telemetry_hz"], self._record_sample)

        # One /proc pass per graph tick, alongside the graph scan. The two see
        # different worlds and we need both — see proc_scan's header.
        self._proc = {}

        # The ROS graph is scanned on a timer rather than per request:
        # get_node_names() is a discovery call, and running it once per browser
        # poll per client would put graph traffic on the wire in proportion to
        # how many people have the page open.
        self._graph = set()
        self.create_timer(p["graph_period_s"], self._scan_graph)

        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                 self._on_status, 10)
        self.create_subscription(TrackedTargetArray, p["targets_topic"],
                                 self._on_targets, 10)

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

    def _scan_graph(self):
        """Which registry nodes are present in the ROS graph right now.

        This is how a node started by core.launch.py, by systemd, or by hand in
        another terminal shows as running. Nobody wants a dashboard reporting
        the telemetry bridge is down because it did not personally start it.
        """
        try:
            names = {n for n, _ns in self.get_node_names_and_namespaces()}
            self._graph = names
        except Exception:
            pass                       # discovery hiccup; keep the last answer
        # One /proc pass for every registry entry at once. Catches what the
        # graph cannot: the tools/ viewer scripts, which are not ROS nodes, and
        # anything started from a terminal. See proc_scan.
        self._proc = proc_scan.scan([s.executable for s in reg.REGISTRY])

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
            src = reg.tab_source(names, running)
            if src:
                port = reg.BY_NAME[src].port
                sources[key] = f"http://127.0.0.1:{port}/stream/view"
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
        src = reg.tab_source(sources, running)
        return {
            "source": src,
            "port": reg.BY_NAME[src].port if src else 0,
            "title": title, "hint": hint,
            "candidates": [{"name": n, "label": reg.BY_NAME[n].label}
                           for n in sources],
        }

    def _snapshot(self):
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

        return {
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
            },
        }

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
        if path == "/logs":
            return self._logs(payload)
        if path == "/logs/clear":
            self.logs.clear()
            return {"ok": True, "message": "log buffer cleared"}
        if path == "/record/start":
            ok, message = self.recorder.start(self._record_sources(),
                                              self.p["record_frame_hz"])
            return {"ok": ok, "message": message}
        if path == "/record/stop":
            ok, message = self.recorder.stop()
            return {"ok": ok, "message": message}
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


def _round(v, n=2):
    return None if v is None else round(v, n)


def main(args=None):
    run_node(GroundStation, args=args)


if __name__ == "__main__":
    main()
