"""gcs_bridge — the boat's end of the ground station: one TCP server, no web page.

    ros2 run crusader_groundstation gcs_bridge      # then, on the laptop: BOAT_GUI.cmd

Speaks gcs_wire on 0.0.0.0:14559, one persistent connection per laptop (laptop/gcs_app.py).
It publishes only /crsd/passage_plan (the operator's MANUAL field); its other effects are
processes, parameter sets, the mission goal, recordings and power, each re-checked here on
arrival whatever the laptop showed:

  PROTECTED NODES CANNOT BE STOPPED from here (node_registry).
  POWER IS LOCKED while armed or while the arm state is unknown, and needs the hostname.

Each laptop gets a bounded send queue: a slow WiFi link loses frames, never stalls the boat.
The pictures and the LiDAR preview are subscribed only while a laptop keeps asking for them,
so a closed tab costs camera_node and lidar_node nothing.
"""
import json
import math
import os
import queue
import socket
import threading
import time
from collections import deque
from concurrent.futures import ThreadPoolExecutor

import yaml
from action_msgs.srv import CancelGoal
from rcl_interfaces.msg import Log
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, PointCloud2
from std_msgs.msg import String
from std_srvs.srv import Trigger

from crusader_common import geo
from crusader_common.node_main import declare_params, run_node
from crusader_common.stream_cache import StreamCache
from crusader_msgs.action import RunMission
from crusader_msgs.msg import (Attitude, FcuHealth, FcuStatus, HazardArray, LatLonHead,
                               PassageBuoy, PassagePlan, RadioFrame, TrackedTargetArray)

from crusader_groundstation import (bag_recorder, camera_profiles, gcs_wire, param_client,
                                    planner_profiles, power_client, proc_scan, system_info)
from crusader_groundstation import node_registry as reg
from crusader_groundstation.log_buffer import LogBuffer
from crusader_groundstation.process_manager import ProcessManager
from crusader_groundstation.radio_core import RadioLog
from crusader_groundstation.recorder import Recorder, frame_rates, free_gb

PARAMS = {
    "port": 14559,                      # = gcs_wire.PORT
    "bind_host": "0.0.0.0",
    "params_file": "",                  # "" = crusader_bringup's installed crusader_params.yaml
    "power_socket": "/run/crsd-power.sock",
    "allow_power": True,
    "disk_path": "/",
    "pose_timeout_s": 1.0,
    "status_timeout_s": 1.0,
    "health_timeout_s": 3.0,            # /crsd/fcu_health is 1 Hz
    "targets_timeout_s": 3.0,
    "leg_timeout_s": 2.0,               # bt_runner's leg status is 2 Hz while a leg runs
    "bt_timeout_s": 3.0,
    "sensor_health_timeout_s": 12.0,    # camera_node and lidar_node say so every 5 s
    "graph_period_s": 1.0,
    "log_capacity": 1500,
    "record_dir": "/root/robotx_ws/logs/sessions",
    "record_telemetry_hz": 2.0,
    "record_frame_hz": 1.0,             # default per session; the Record tab overrides it
    "record_min_free_gb": 2.0,
    "planner_profiles_dir": "/root/robotx_ws/src/rx26_asv/crusader_sim/config/tuning_profiles",
    "planner_profiles_save_dir": "~/.cache/crusader/planner_profiles",
}

LEG_TOPIC = "/crsd/nav/leg_status"      # = bt_runner_node.nav_leg_topic (check_config)
IMAGE_TOPIC = "/crsd/camera/image/compressed"
RAW_TOPIC = "/crsd/camera/raw/compressed"
PREVIEW_TOPIC = "/crsd/lidar/preview"
MISSION_ACTION = "/crsd/run_mission"
STATE_PERIOD_S = 0.5
EVENT_PERIOD_S = 0.25
LEASE_S = 3.0                           # a picture nobody asked for in this long is dropped
FRAME_MAX_AGE_S = 3.0
MANUAL_PERIOD_S = 1.0                   # the MANUAL field is resent, as Ekko's plan is
MANUAL_VERSION_BASE = 1_000_000         # clear of rfd_link_node's plan versions
LIDAR_SCALE = 0.01                      # metres per int16 count
RECORD_FRAME_HZ_RANGE = (0.05, 30.0)
REPLAY_EVENTS = 300                     # log/radio rows a laptop gets when it connects
FILE_CHUNK = 256 * 1024
OPS_WORKERS = 4


def clean(obj):
    """JSON-safe: non-finite floats become None, tuples lists."""
    if isinstance(obj, float):
        return obj if math.isfinite(obj) else None
    if isinstance(obj, dict):
        return {k: clean(v) for k, v in obj.items()}
    if isinstance(obj, (list, tuple)):
        return [clean(v) for v in obj]
    return obj


def signature(frame):
    """What a frame says, without the ages that change on every tick."""
    return json.dumps(frame, sort_keys=True, default=str,
                      separators=(",", ":")).replace('"age"', "")


def _round(v, n=2):
    return gcs_wire.finite(v, n)


class Connection:
    """One laptop: a reader thread, a sender thread, and a bounded queue between them."""

    QUEUE = 400

    def __init__(self, sock, addr, handle, log):
        self.sock, self.addr = sock, addr
        self.min_level = 20                 # INFO: the Logs tab's default
        self.dropped = 0
        self._q = queue.Queue(self.QUEUE)
        self._alive = True
        self._handle, self._log = handle, log
        threading.Thread(target=self._send_loop, daemon=True).start()
        threading.Thread(target=self._read_loop, daemon=True).start()

    @property
    def alive(self):
        return self._alive

    def send(self, header, blob=b"", wait_s=0.0):
        """Queue one frame -> sent? Never blocks unless wait_s (a download's backpressure)."""
        if not self._alive:
            return False
        try:
            frame = gcs_wire.encode(clean(header), blob)
        except ValueError as e:
            self._log.error(f"frame {header.get('t')!r} not sent: {e}")
            return False
        try:
            if wait_s:
                self._q.put(frame, timeout=wait_s)
            else:
                self._q.put_nowait(frame)
            return True
        except queue.Full:
            self.dropped += 1
            return False

    def _send_loop(self):
        try:
            while self._alive:
                frame = self._q.get()
                if frame is None:
                    break
                self.sock.sendall(frame)
        except OSError as e:
            self._log.info(f"laptop {self.addr[0]}: send failed ({e})")
        finally:
            self.close()

    def _read_loop(self):
        try:
            while self._alive:
                header, blob = gcs_wire.read_frame(self.sock)
                self._handle(self, header, blob)
        except (ConnectionError, OSError) as e:
            self._log.info(f"laptop {self.addr[0]}: {e}")
        finally:
            self.close()

    def close(self):
        if not self._alive:
            return
        self._alive = False
        try:
            self._q.put_nowait(None)
        except queue.Full:
            pass
        for fn in (lambda: self.sock.shutdown(socket.SHUT_RDWR), self.sock.close):
            try:
                fn()
            except OSError:
                pass


class _ChunkWriter:
    """A file-like object that sends what is written as file frames, with backpressure."""

    def __init__(self, conn, rid):
        self.conn, self.rid, self.seq, self.sent = conn, rid, 0, 0
        self._buf = bytearray()

    def write(self, data):
        self._buf += data
        while len(self._buf) >= FILE_CHUNK:
            self._send(bytes(self._buf[:FILE_CHUNK]), last=False)
            del self._buf[:FILE_CHUNK]
        return len(data)

    def flush(self):
        pass

    def finish(self):
        self._send(bytes(self._buf), last=True)

    def _send(self, chunk, last):
        if not self.conn.send({"t": "file", "id": self.rid, "seq": self.seq, "last": last},
                              chunk, wait_s=30.0):
            raise OSError("the laptop stopped reading the download")
        self.seq += 1
        self.sent += len(chunk)


class GcsBridge(Node):

    def __init__(self):
        super().__init__("gcs_bridge")
        p = self.p = declare_params(self, PARAMS)
        if not p["params_file"]:
            from ament_index_python.packages import get_package_share_directory
            p["params_file"] = os.path.join(get_package_share_directory("crusader_bringup"),
                                            "config", "crusader_params.yaml")
        self._hostname = socket.gethostname()
        self._cpu = system_info.CpuMeter()
        self.procs = ProcessManager(p["params_file"], logger=self.get_logger().info)
        self.params_bridge = param_client.ParamBridge(self)
        self.logs = LogBuffer(int(p["log_capacity"]))
        self.radio = RadioLog()
        self._ocs = deque(maxlen=REPLAY_EVENTS)
        self._ocs_seq = 0
        self.recorder = Recorder(p["record_dir"], p["record_min_free_gb"],
                                 logger=self.get_logger().warn)
        self.bags = bag_recorder.BagRecorder(logger=self.get_logger().info)
        self._persist = self._check_persistence(p["record_dir"])

        # Freshness caches, all written by ROS callbacks.
        self._pose = StreamCache(p["pose_timeout_s"])
        self._att = StreamCache(p["pose_timeout_s"])
        self._status = StreamCache(p["status_timeout_s"])
        self._health = StreamCache(p["health_timeout_s"])
        self._targets = StreamCache(p["targets_timeout_s"])
        self._leg = StreamCache(p["leg_timeout_s"])
        self._bt = StreamCache(p["bt_timeout_s"])
        self._camera = StreamCache(p["sensor_health_timeout_s"])
        self._lidar = StreamCache(p["sensor_health_timeout_s"])
        self._plan = None                   # (PassagePlan, monotonic time)
        self._hazards = None
        self._datum = None
        self._latest = {}                   # "image" | "preview" -> (stamp, payload, mono)
        self._latest_cv = threading.Condition()
        self._leases = {}                   # "image" | "preview" -> expiry (monotonic)
        self._subs = {}
        self._graph, self._graph_full, self._pids = set(), [], {}
        self._last = {}                     # frame kind -> the newest frame of it
        self._sigs = {}
        self._log_seq = self._radio_seq = self._ocs_sent = 0
        self._manual = None                 # the MANUAL PassagePlan while it stands
        self._manual_version = MANUAL_VERSION_BASE
        self._ekko_heard = None             # monotonic time of the last plan not ours

        self.create_subscription(LatLonHead, "/crsd/pose", self._on_pose, 10)
        self.create_subscription(Attitude, "/crsd/attitude",
                                 lambda m: self._att.set((m.roll, m.pitch, m.yaw),
                                                         time.monotonic()), 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status",
                                 lambda m: self._status.set((m.mode, m.armed),
                                                            time.monotonic()), 10)
        self.create_subscription(FcuHealth, "/crsd/fcu_health",
                                 lambda m: self._health.set(m, time.monotonic()), 10)
        self.create_subscription(TrackedTargetArray, "/crsd/world_targets",
                                 lambda m: self._targets.set(m, time.monotonic()), 10)
        self.create_subscription(String, LEG_TOPIC, self._on_json(self._leg), 10)
        self.create_subscription(String, "/crsd/bt_status", self._on_json(self._bt), 10)
        self.create_subscription(String, "/crsd/camera/health", self._on_json(self._camera), 10)
        self.create_subscription(String, "/crsd/lidar/health", self._on_json(self._lidar), 10)
        self.create_subscription(PassagePlan, "/crsd/passage_plan", self._on_plan, 10)
        self.create_subscription(HazardArray, "/crsd/nav/hazards",
                                 lambda m: setattr(self, "_hazards", m), 10)
        self.create_subscription(
            LatLonHead, "/crsd/datum", lambda m: setattr(self, "_datum",
                                                         (m.latitude, m.longitude)),
            QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                       durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Log, "/rosout", self._on_rosout, 100)
        self.create_subscription(RadioFrame, "/crsd/radio/traffic", self._on_radio, 50)
        self.create_subscription(String, "/crsd/ocs_traffic", self._on_ocs, 50)

        self.plan_pub = self.create_publisher(PassagePlan, "/crsd/passage_plan", 10)
        self.mission = ActionClient(self, RunMission, MISSION_ACTION)
        self._cancel = self.create_client(CancelGoal, MISSION_ACTION + "/_action/cancel_goal")
        self._radio_test = self.create_client(Trigger, "/crsd/radio/send_test")
        self._goal = None

        self.create_timer(STATE_PERIOD_S, self._tick)
        self.create_timer(EVENT_PERIOD_S, self._flush_events)
        self.create_timer(p["graph_period_s"], self._scan_graph)
        self.create_timer(0.25, self._reconcile_subs)
        self.create_timer(MANUAL_PERIOD_S, self._resend_manual)
        self.create_timer(1.0 / p["record_telemetry_hz"], self._record_sample)

        self._ops = {name[4:]: getattr(self, name) for name in dir(self)
                     if name.startswith("_op_")}
        self._pool = ThreadPoolExecutor(OPS_WORKERS)
        self._conns, self._conns_lock = [], threading.Lock()
        self._server = socket.create_server((p["bind_host"], int(p["port"])), reuse_port=False)
        self._stop = threading.Event()
        threading.Thread(target=self._accept_loop, daemon=True).start()
        self.get_logger().info(
            f"gcs_bridge on tcp://{p['bind_host']}:{int(p['port'])}, nodes started with "
            f"{p['params_file']}; power {'ENABLED' if p['allow_power'] else 'disabled'}")

    # ------------------------------------------------------------------ connections

    def _accept_loop(self):
        while not self._stop.is_set():
            try:
                sock, addr = self._server.accept()
            except OSError:
                break
            gcs_wire.keepalive(sock)
            conn = Connection(sock, addr, self._on_frame, self.get_logger())
            self.get_logger().info(f"laptop connected from {addr[0]}")
            with self._conns_lock:
                self._conns = [c for c in self._conns if c.alive] + [conn]
            self._welcome(conn)

    def _welcome(self, conn):
        """Everything a fresh laptop needs to draw every tab at once."""
        conn.send({"t": "hello", "hostname": self._hostname, "ops": sorted(self._ops)})
        for frame in list(self._last.values()):
            conn.send(frame)
        records, _, _ = self.logs.read(0, conn.min_level, limit=REPLAY_EVENTS)
        conn.send({"t": "log", "items": records, "replay": True})
        rows, _, _ = self.radio.read(0, REPLAY_EVENTS)
        conn.send({"t": "radio", "items": rows, "systems": self.radio.systems(),
                   "streams": self.radio.streams(), "replay": True})
        conn.send({"t": "ocs", "items": list(self._ocs), "replay": True})

    def _connections(self):
        with self._conns_lock:
            self._conns = [c for c in self._conns if c.alive]
            return list(self._conns)

    def _broadcast(self, frame, blob=b""):
        for conn in self._connections():
            conn.send(frame, blob)

    def _send_if_changed(self, kind, frame):
        self._last[kind] = frame
        sig = signature(frame)
        if self._sigs.get(kind) != sig:
            self._sigs[kind] = sig
            self._broadcast(frame)

    def _on_frame(self, conn, header, blob):
        if header.get("t") == "req":
            self._pool.submit(self._run_op, conn, header)

    def _run_op(self, conn, req):
        rid, op = req.get("id"), str(req.get("op"))
        fn = self._ops.get(op)
        if fn is None:
            result = (False, f"unknown op {op!r}", None)
        else:
            try:
                result = fn(conn, req)
            except Exception as e:  # noqa: BLE001 - an op must answer, whatever went wrong
                self.get_logger().error(f"op {op} failed: {type(e).__name__}: {e}")
                result = (False, f"{op} failed: {type(e).__name__}: {e}", None)
        if result is not None:
            ok, msg, data = result
            conn.send({"t": "result", "id": rid, "op": op, "ok": bool(ok), "msg": msg,
                       "data": data})

    # ------------------------------------------------------------------ inputs

    def _on_pose(self, msg):
        if math.isfinite(msg.latitude) and math.isfinite(msg.longitude) and \
                (msg.latitude, msg.longitude) != (0.0, 0.0):
            self._pose.set((msg.latitude, msg.longitude, msg.heading, msg.ground_speed),
                           time.monotonic())

    def _on_json(self, cache):
        def callback(msg):
            try:
                value = json.loads(msg.data)
            except ValueError:
                return              # the cache ages out to a blank, never the last value
            if isinstance(value, dict):
                cache.set(value, time.monotonic())
        return callback

    def _on_plan(self, msg):
        if self._manual is None or msg.plan_version != self._manual.plan_version:
            self._ekko_heard = time.monotonic()
        self._plan = (msg, time.monotonic())

    def _on_rosout(self, msg):
        self.logs.add(msg.name, msg.level, msg.msg,
                      stamp=msg.stamp.sec + msg.stamp.nanosec * 1e-9)

    def _on_radio(self, msg):
        self.radio.add(msg.direction, msg.src_system, msg.src_component, msg.dst_system,
                       msg.msg_name, msg.payload_type, msg.summary, msg.frame_bytes,
                       stamp=msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)

    def _on_ocs(self, msg):
        try:
            item = json.loads(msg.data)
        except ValueError:
            return
        self._ocs_seq += 1
        item["seq"] = self._ocs_seq
        self._ocs.append(item)

    def _on_image(self, key):
        def callback(msg):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            payload = bytes(msg.data) if key != "preview" else msg
            with self._latest_cv:
                self._latest[key] = (stamp, payload, time.monotonic())
                self._latest_cv.notify_all()
        return callback

    def _on_raw(self, msg):
        self.recorder.put_frame("camera", bytes(msg.data))

    # ------------------------------------------------------------------ on-demand topics

    def _lease(self, key):
        self._leases[key] = time.monotonic() + LEASE_S

    def _reconcile_subs(self):
        """Subscriptions made and dropped here, on the executor's thread, never from an op."""
        now = time.monotonic()
        wanted = {k for k, until in self._leases.items() if until > now}
        if self.recorder.recording and "camera" in self.recorder.session.frames:
            wanted.add("raw")
        topics = {"image": (CompressedImage, IMAGE_TOPIC, self._on_image("image"), 2),
                  "preview": (PointCloud2, PREVIEW_TOPIC, self._on_image("preview"),
                              qos_profile_sensor_data),
                  "raw": (CompressedImage, RAW_TOPIC, self._on_raw, 5)}
        for key, (kind, topic, cb, qos) in topics.items():
            if key in wanted and key not in self._subs:
                self._subs[key] = self.create_subscription(kind, topic, cb, qos)
            elif key not in wanted and key in self._subs:
                self.destroy_subscription(self._subs.pop(key))
                with self._latest_cv:
                    self._latest.pop(key, None)

    def _wait_latest(self, key, timeout_s=1.5):
        deadline = time.monotonic() + timeout_s
        with self._latest_cv:
            while key not in self._latest and time.monotonic() < deadline:
                self._latest_cv.wait(0.1)
            return self._latest.get(key)

    # ------------------------------------------------------------------ the 2 Hz frames

    def _tick(self):
        now = time.monotonic()
        state = self._state(now)
        self._last["state"] = state
        for conn in self._connections():
            conn.send(dict(state, link={"dropped": conn.dropped}))
        self._send_if_changed("targets", self._targets_frame(now))
        self._send_if_changed("path", self._path_frame(now))
        self._send_if_changed("plan", self._plan_frame(now))
        self._send_if_changed("hazards", self._hazards_frame())
        bt = self._bt.get(now)
        self._send_if_changed("tree", dict(bt or {}, t="tree", ok=bt is not None,
                                           age=_round(self._bt.age(now), 1)))

    def _state(self, now):
        pose, att = self._pose.get(now), self._att.get(now)
        status, health = self._status.get(now), self._health.get(now)
        cam, lidar, bt = self._camera.get(now), self._lidar.get(now), self._bt.get(now)
        boat = {"lat": None, "lon": None, "hdg": None, "spd": None,
                "age": _round(self._pose.age(now), 1)}
        if pose is not None:
            boat.update(lat=pose[0], lon=pose[1], hdg=_round(pose[2], 1), spd=_round(pose[3]))
        sysinfo = system_info.snapshot(self._cpu, self.p["disk_path"])
        session = self.recorder.session if self.recorder.recording else None
        return {
            "t": "state", "stamp": time.time(), "boat": boat,
            "att": {k: (_round(math.degrees(v), 1) if att else None)
                    for k, v in zip(("roll", "pitch", "yaw"), att or (0, 0, 0))},
            "fcu": {"mode": status[0] if status else None,
                    "armed": bool(status[1]) if status else None,
                    "age": _round(self._status.age(now), 1)},
            "health": None if health is None else {
                "batt_v": _round(health.battery_v), "batt_pct": _round(health.battery_pct, 0),
                "fix": health.gps_fix_type, "sats": health.gps_sats,
                "hdop": _round(health.gps_hdop), "ekf": health.ekf_ok,
                "prearm": health.prearm_ok, "estop": health.estop_active,
                "text": health.last_statustext, "sev": health.last_statustext_severity,
                "age": _round(self._health.age(now), 1)},
            "mission": None if bt is None else {
                k: bt.get(k) for k in ("state", "name", "elapsed_s", "running_node",
                                       "current_task", "warning", "last")},
            "camera": None if cam is None else {
                k: cam.get(k) for k in ("source", "model", "model_state", "fps", "set")},
            "lidar": None if lidar is None else {
                "clusters": lidar.get("n_clusters"), "points": lidar.get("n_in"),
                "levelled": lidar.get("levelled"), "nav_cloud": lidar.get("nav_cloud"),
                "prox": (lidar.get("proximity") or {}).get("enabled")},
            "sys": {"cpu": _round(sysinfo["cpu_percent"], 0), "temp": _round(sysinfo["temp_c"], 0),
                    "mem_gb": _round(sysinfo["mem_used_gb"], 1),
                    "mem_total_gb": _round(sysinfo["mem_total_gb"], 1),
                    "disk_gb": _round(sysinfo["disk_free_gb"], 1),
                    "uptime": system_info.format_uptime(sysinfo["uptime_s"])},
            "record": None if session is None else {
                "name": session.name, "elapsed_s": round(time.time() - session.started, 1),
                "frames": session.frame_stats()[0], "bag": self.bags.recording},
            "power": self._power_state(),
        }

    def _targets_frame(self, now):
        msg = self._targets.get(now)
        items = [] if msg is None else [{
            "id": int(t.id), "label": t.label, "conf": _round(t.confidence),
            "lat": t.latitude, "lon": t.longitude, "stddev": _round(t.position_stddev),
            "hits": int(t.hits), "unseen": _round(t.time_since_seen, 1),
            "confirmed": bool(t.confirmed), "sources": int(t.sources)}
            for t in msg.targets]
        return {"t": "targets", "ok": msg is not None, "items": items}

    def _path_frame(self, now):
        leg = self._leg.get(now)
        if leg is None:
            return {"t": "path", "ok": False}

        def pts(v):
            try:
                return [[float(a), float(b)] for a, b in v]
            except (TypeError, ValueError):
                return []
        target, goal = pts([leg.get("target")] if leg.get("target") else []), \
            pts([leg.get("goal")] if leg.get("goal") else [])
        return {"t": "path", "ok": True, "state": leg.get("state"), "why": leg.get("why"),
                "mode": leg.get("mode"), "blocked_s": leg.get("blocked_s"),
                "path": pts(leg.get("path") or []), "target": target[0] if target else None,
                "goal": goal[0] if goal else None}

    def _plan_frame(self, now):
        if self._plan is None:
            return {"t": "plan", "ok": False}
        msg, when = self._plan
        manual = self._manual is not None and msg.plan_version == self._manual.plan_version

        def at(lat, lon):
            return None if (lat, lon) == (0.0, 0.0) else [lat, lon]
        return {"t": "plan", "ok": True, "source": "manual" if manual else "ekko",
                "version": int(msg.plan_version), "age": round(now - when, 1),
                "entry": at(msg.entry_latitude, msg.entry_longitude),
                "exit": at(msg.exit_latitude, msg.exit_longitude),
                "buoys": [{"id": int(b.id), "lat": b.latitude, "lon": b.longitude,
                           "beacon": int(b.beacon)} for b in msg.buoys],
                "ekko_too": bool(manual and self._ekko_heard is not None
                                 and now - self._ekko_heard < 5.0)}

    def _hazards_frame(self):
        msg, datum = self._hazards, self._datum
        if msg is None or datum is None:
            return {"t": "hazards", "ok": False, "items": []}
        items = []
        for h in msg.hazards:
            item = {"src": int(h.source), "id": int(h.id), "keep": _round(h.keepout_m)}
            if h.kind == h.CIRCLE:
                item["c"] = list(geo.xy_to_latlon(h.x, h.y, datum))
                item["r"] = _round(h.radius_m)
            else:
                item["poly"] = [list(geo.xy_to_latlon(x, y, datum))
                                for x, y in zip(h.polygon_x, h.polygon_y)]
            items.append(item)
        return {"t": "hazards", "ok": True, "items": items}

    def _flush_events(self):
        records, newest, _ = self.logs.read(self._log_seq, 0, limit=500)
        self._log_seq = newest
        rows, rnewest, _ = self.radio.read(self._radio_seq, 500)
        self._radio_seq = rnewest
        ocs = [i for i in self._ocs if i["seq"] > self._ocs_sent]
        self._ocs_sent = self._ocs_seq
        if not (records or rows or ocs):
            return
        radio = {"t": "radio", "items": rows, "systems": self.radio.systems(),
                 "streams": self.radio.streams()} if rows else None
        for conn in self._connections():
            mine = [r for r in records if r["level"] >= conn.min_level]
            if mine:
                conn.send({"t": "log", "items": mine})
            if radio:
                conn.send(radio)
            if ocs:
                conn.send({"t": "ocs", "items": ocs})

    # ------------------------------------------------------------------ nodes

    def _scan_graph(self):
        try:
            pairs = self.get_node_names_and_namespaces()
            self._graph = {n for n, _ns in pairs}
            self._graph_full = param_client.visible_nodes(ns.rstrip("/") + "/" + n
                                                          for n, ns in pairs)
        except Exception:  # noqa: BLE001 - a discovery hiccup keeps the last answer
            pass
        self._pids = proc_scan.scan([s.executable for s in reg.REGISTRY])
        items, running = self._node_items()
        self._send_if_changed("nodes", {
            "t": "nodes", "items": items, "up": len(running), "graph": self._graph_full,
            "groups": [{"id": g, "label": label, "hint": hint} for g, label, hint in reg.GROUPS],
            "profiles": [{"id": k, "label": v[0]} for k, v in reg.PROFILES.items()]})

    def _node_items(self):
        owned = self.procs.running()
        items, running = [], set()
        for spec in reg.REGISTRY:
            state, detail = self.procs.status(spec.name)
            pids = self._pids.get(spec.executable, [])
            up = spec.name in self._graph or spec.name in owned or bool(pids)
            if up:
                running.add(spec.name)
            allowed, why = reg.may_stop(spec.name)
            items.append({
                "name": spec.name, "group": spec.group, "protected": spec.protected,
                "note": spec.note, "running": up,
                "state": "running" if up else state,
                "detail": detail if spec.name in owned else ("in graph" if up else detail),
                "stoppable": bool(allowed and (spec.name in owned or pids)),
                "stop_reason": why, "tail": self.procs.tail(spec.name)[-8:]})
        for item in items:
            item["conflicts"] = reg.conflicts(item["name"], running)
        return items, running

    def _start_node(self, name, stop_conflicts):
        spec = reg.BY_NAME.get(name)
        if spec is None:
            return False, f"unknown node {name!r}"
        _items, running = self._node_items()
        if name in running:
            return True, f"{name} is already running"
        clash = reg.conflicts(name, running)
        if clash and not stop_conflicts:
            return False, f"{', '.join(clash)} holds the same device: confirm to stop it first"
        for other in clash:
            ok, why = self._stop_node(other)
            if not ok:
                return False, why
        return self.procs.start(spec)

    def _stop_node(self, name):
        allowed, why = reg.may_stop(name)
        if not allowed:
            self.get_logger().warn(f"refused stop of {name!r}: {why}")
            return False, why
        if name in self.procs.running():
            return self.procs.stop(name)
        # Not ours: the PID only, never its group, which may be the whole launch.
        return self.procs.stop_external(name, self._pids.get(reg.BY_NAME[name].executable, []))

    def _op_node(self, conn, req):
        action, name = req.get("action"), req.get("name")
        if action == "start":
            ok, msg = self._start_node(name, bool(req.get("stop_conflicts")))
        elif action == "stop":
            ok, msg = self._stop_node(name)
        elif action == "restart":
            allowed, why = reg.may_stop(name)
            if not allowed:
                return False, why, None
            ok, msg = self._stop_node(name)
            if ok:
                ok, msg = self._start_node(name, False)
        elif action == "profile":
            ok, msg = self._start_profile(name)
        else:
            return False, f"unknown node action {action!r}", None
        self._scan_graph()
        return ok, msg, None

    def _start_profile(self, name):
        """Start what is not running. A profile never resolves a device conflict: silently
        stopping a node someone chose is how a run gets lost."""
        profile = reg.PROFILES.get(name)
        if profile is None:
            return False, f"unknown profile {name!r}"
        _items, running = self._node_items()
        started, skipped = [], []
        for node in profile[1]:
            if node in running or reg.conflicts(node, running):
                skipped.append(node)
                continue
            ok, _msg = self.procs.start(reg.BY_NAME[node])
            (started if ok else skipped).append(node)
            running.add(node)
        return True, f"started {len(started)}" + (f", skipped {', '.join(skipped)}"
                                                  if skipped else "")

    # ------------------------------------------------------------------ mission

    def _op_missions_list(self, conn, req):
        from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
        try:
            d = os.path.join(get_package_share_directory("crusader_bt"), "behavior_trees")
        except PackageNotFoundError:
            return False, "crusader_bt is not installed", None
        names = sorted(f[:-4] for f in os.listdir(d) if f.endswith(".xml"))
        return True, "", {"names": names}

    def _op_run_mission(self, conn, req):
        if not self.mission.wait_for_server(timeout_sec=1.0):
            return False, f"nothing serves {MISSION_ACTION}: is bt_runner_node up?", None
        goal = RunMission.Goal()
        goal.name = str(req.get("name") or "")
        goal.tier = int(req.get("tier") or 0)
        goal.timeout_s = float(req.get("timeout_s") or 0.0)
        approach = req.get("approach")
        if approach:
            goal.approach_latitude, goal.approach_longitude = map(float, approach)
        done = threading.Event()
        future = self.mission.send_goal_async(goal)
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(3.0):
            return False, f"{MISSION_ACTION} did not answer within 3 s", None
        handle = future.result()
        if not handle.accepted:
            return False, (f"bt_runner_node refused {goal.name!r} (one mission at a time; see "
                           "the Logs tab for why)"), None
        self._goal = handle
        handle.get_result_async().add_done_callback(self._on_result)
        self.get_logger().info(f"mission {goal.name} tier {goal.tier} started from the GCS")
        return True, f"{goal.name} accepted", None

    def _on_result(self, future):
        r = future.result().result
        self.get_logger().info(f"mission ended: outcome {r.outcome}, {r.detail} "
                               f"({r.elapsed_s:.0f} s)")

    def _op_cancel_mission(self, conn, req):
        """Cancels every goal, whoever sent it: Stop means stop."""
        if not self._cancel.wait_for_service(timeout_sec=1.0):
            return False, "bt_runner_node does not answer cancel: is it up?", None
        done = threading.Event()
        future = self._cancel.call_async(CancelGoal.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(3.0):
            return False, "cancel was not answered within 3 s", None
        n = len(future.result().goals_canceling)
        return n > 0, (f"cancelling {n} goal(s)" if n else "no mission was running"), None

    def _op_manual_field(self, conn, req):
        """The operator's field, published as Ekko's would be. Clear = an empty plan, which
        bt_runner reads as no plan (core tier)."""
        if req.get("clear"):
            self._manual = None
            self.plan_pub.publish(self._plan_msg({}))
            self.get_logger().warn("MANUAL field cleared from the ground station")
            return True, "manual field cleared", None
        plan = req.get("plan") or {}
        try:
            msg = self._plan_msg(plan)
        except (TypeError, ValueError, KeyError) as e:
            return False, f"bad field: {e}", None
        if not msg.buoys:
            return False, "a field needs at least one buoy (or send clear)", None
        self._manual = msg
        self.plan_pub.publish(msg)
        self.get_logger().warn(f"MANUAL field v{msg.plan_version}: {len(msg.buoys)} buoys "
                               "from the ground station, resent every second until cleared")
        return True, f"manual field v{msg.plan_version} published", None

    def _plan_msg(self, plan):
        def ll(v):
            if not v:
                return 0.0, 0.0
            lat, lon = float(v[0]), float(v[1])
            if not (math.isfinite(lat) and math.isfinite(lon) and abs(lat) <= 90
                    and abs(lon) <= 180):
                raise ValueError(f"{v!r} is not a position")
            return lat, lon
        m = PassagePlan()
        m.header.stamp = self.get_clock().now().to_msg()
        self._manual_version += 1
        m.plan_version = self._manual_version
        m.entry_latitude, m.entry_longitude = ll(plan.get("entry"))
        m.exit_latitude, m.exit_longitude = ll(plan.get("exit"))
        for i, b in enumerate(plan.get("buoys") or []):
            pb = PassageBuoy()
            pb.id = int(b.get("id", i + 1))
            pb.latitude, pb.longitude = ll((b["lat"], b["lon"]))
            pb.beacon = int(b.get("beacon", PassageBuoy.BEACON_UNKNOWN))
            if not 0 <= pb.beacon <= PassageBuoy.BEACON_STEADY_BLUE:
                raise ValueError(f"beacon {pb.beacon} is not a BeaconState")
            m.buoys.append(pb)
        return m

    def _resend_manual(self):
        if self._manual is not None:
            self._manual.header.stamp = self.get_clock().now().to_msg()
            self.plan_pub.publish(self._manual)

    # ------------------------------------------------------------------ pictures

    def _op_cam(self, conn, req):
        w = max(80, min(1920, int(req.get("w") or 320)))
        q = max(10, min(95, int(req.get("q") or 50)))
        self._lease("image")
        latest = self._wait_latest("image")
        if latest is None or time.monotonic() - latest[2] > FRAME_MAX_AGE_S:
            return False, "no picture: is camera_node running?", None
        stamp, jpeg, mono = latest
        jpeg, width, height = rescale_jpeg(jpeg, w, q)
        conn.send({"t": "cam", "id": req.get("id"), "w": width, "h": height, "stamp": stamp,
                   "age": round(time.monotonic() - mono, 2)}, jpeg)
        return None

    def _op_lidar(self, conn, req):
        n_max = max(100, min(3000, int(req.get("max_points") or 3000)))
        self._lease("preview")
        latest = self._wait_latest("preview")
        if latest is None or time.monotonic() - latest[2] > FRAME_MAX_AGE_S:
            return False, "no LiDAR preview: is lidar_node running?", None
        stamp, msg, mono = latest
        blob, n = pack_xy(msg, n_max)
        conn.send({"t": "lidar", "id": req.get("id"), "n": n, "scale": LIDAR_SCALE,
                   "frame": msg.header.frame_id, "stamp": stamp,
                   "age": round(time.monotonic() - mono, 2)}, blob)
        return None

    # ------------------------------------------------------------------ parameters

    def _yaml_section(self, node_name):
        name = node_name.rstrip("/").split("/")[-1]
        try:
            with open(self.p["params_file"], encoding="utf-8") as f:
                doc = yaml.safe_load(f) or {}
            return dict((doc.get(name) or {}).get("ros__parameters") or {})
        except (OSError, yaml.YAMLError):
            return {}

    def _param_list(self, node):
        if node not in self._graph_full:
            raise RuntimeError(f"{node or 'no node'} is not in the ROS graph")
        rows = self.params_bridge.list(node, self._yaml_section(node))
        choices = camera_choices() if node.endswith("/camera_node") else {}
        for row in rows:
            row["choices"] = choices.get(row["name"], [])
        return rows

    def _param_set(self, node, values):
        if node not in self._graph_full:
            raise RuntimeError(f"{node or 'no node'} is not in the ROS graph")
        results = self.params_bridge.set(node, values)
        for r in results:
            line = f"{node} {r['name']} <- {values.get(r['name'])!r}: "
            if r["ok"]:                 # two call sites: rclpy fixes a line's severity
                self.get_logger().info(line + "applied")
            else:
                self.get_logger().warn(line + f"REFUSED: {r['reason']}")
        bad = [r for r in results if not r["ok"]]
        msg = ("; ".join(f"{r['name']}: {r['reason'] or 'refused'}" for r in bad) if bad
               else f"applied {', '.join(sorted(values))} on {node.lstrip('/')}")
        return not bad, msg, {"results": results}

    def _op_params_list(self, conn, req):
        node = req.get("node") or ""
        return True, "", {"node": node, "params": self._param_list(node)}

    def _op_params_set(self, conn, req):
        values = req.get("values")
        if not isinstance(values, dict) or not values:
            return False, "no values to set", None
        return self._param_set(req.get("node") or "", values)

    def _camera_node(self):
        return next((n for n in self._graph_full if n.endswith("/camera_node")), None)

    def _op_cam_profiles_list(self, conn, req):
        return True, "", {"profiles": camera_profiles.listing(),
                          "store": camera_profiles.status(), "controls": camera_groups()}

    def _op_cam_profile_apply(self, conn, req):
        row = camera_profiles.get(req.get("name"))
        node = self._camera_node()
        if row is None:
            return False, f"no camera profile {req.get('name')!r}", None
        if node is None:
            return False, "camera_node is not running", None
        return self._param_set(node, row["values"])

    def _op_cam_profile_save(self, conn, req):
        """Saves the camera's LIVE values, read now: what the laptop shows may be stale."""
        node = self._camera_node()
        if node is None:
            return False, "camera_node is not running", None
        wanted = {c for g in camera_groups() for c in g["controls"]}
        live = {r["name"]: r["value"] for r in self._param_list(node) if r["name"] in wanted}
        if set(live) != wanted:     # never a partial block: the holes would inherit
            return False, f"camera_node lacks {sorted(wanted - set(live))}", None
        ok, msg = camera_profiles.save(req.get("name"), live)
        return ok, msg, {"profiles": camera_profiles.listing()}

    def _op_cam_profile_delete(self, conn, req):
        ok, msg = camera_profiles.delete(req.get("name"))
        return ok, msg, {"profiles": camera_profiles.listing()}

    def _planner_listing(self):
        return planner_profiles.listing([
            (planner_profiles.SHIPPED, os.path.expanduser(self.p["planner_profiles_dir"])),
            (planner_profiles.SAVED, os.path.expanduser(self.p["planner_profiles_save_dir"]))])

    def _op_planner_profiles_list(self, conn, req):
        return True, "", self._planner_listing()

    def _op_planner_profile_apply(self, conn, req):
        rows = [r for r in self._planner_listing()["profiles"] if r["name"] == req.get("name")]
        row = next((r for r in rows if r["origin"] == planner_profiles.SAVED),
                   rows[0] if rows else None)
        if row is None or row["error"] or not row["values"]:
            return False, (row and (row["error"] or "no bt_runner_node planner keys")
                           or f"no planner profile {req.get('name')!r}"), None
        ok, msg, data = self._param_set(planner_profiles.NODE_PATH, row["values"])
        return ok, msg + (f" (skipped: {', '.join(row['skipped'])})" if row["skipped"] else ""), \
            data

    def _op_planner_profile_save(self, conn, req):
        rows = self._param_list(planner_profiles.NODE_PATH)
        values, note = planner_profiles.drifted_planner_values(rows)
        if not values:
            return False, note or "no planner key differs from crusader_params.yaml", None
        ok, msg, _path = planner_profiles.save(
            os.path.expanduser(self.p["planner_profiles_save_dir"]), req.get("name"), values)
        return ok, msg, self._planner_listing()

    # ------------------------------------------------------------------ recording

    def _check_persistence(self, record_dir):
        mp, src, persists = system_info.mount_for(record_dir)
        if not persists:
            self.get_logger().warn(
                f"RECORDINGS WILL NOT SURVIVE: {record_dir} is on the container's own "
                "filesystem. Point record_dir at a bind mount (/root/robotx_ws is the host's "
                "~/robotx_ws on this fleet).")
        return {"persists": bool(persists), "mount": mp, "source": src}

    def _free_bytes(self):
        gb = free_gb(self.p["record_dir"])
        return None if gb is None else gb * 1073741824.0

    def _op_record(self, conn, req):
        kind, action = req.get("kind"), req.get("action")
        if action == "stop":
            msg = ""
            if self.bags.recording:
                _ok, msg = self.bags.stop()       # the bag first, or meta.json is short
            if kind == "bag":
                return bool(msg), msg or "no bag recording", None
            ok, smsg = self.recorder.stop()
            return ok, smsg + (f"; {msg}" if msg else ""), None
        if action != "start" or kind not in ("session", "bag"):
            return False, "record needs kind session|bag and action start|stop", None
        topics = req.get("topics") or []
        if not isinstance(topics, list) or not all(isinstance(t, str) for t in topics):
            return False, "topics must be a list of names", None
        msg = ""
        if not self.recorder.recording:
            sources = ["camera"] if self.count_publishers(RAW_TOPIC) else []
            rates, notes = frame_rates(req.get("frame_hz"), sources, self.p["record_frame_hz"],
                                       *RECORD_FRAME_HZ_RANGE)
            ok, msg = self.recorder.start(sources, rates)
            if not ok:
                return False, msg, None
            msg += f" ({'; '.join(notes)})" if notes else ""
        if topics:
            ok, bag_msg = self.bags.start(os.path.join(self.recorder.session.dir, "bag"), topics)
            msg += f"; {bag_msg}"         # a failed bag does not fail the session, but says so
        return True, msg.lstrip("; "), None

    def _op_record_list(self, conn, req):
        live = self.recorder.session
        return True, "", {
            "sessions": self.recorder.sessions(),
            "live": live.status() if live and live.stopped is None else None,
            "bag": self.bags.status(self._free_bytes()), "persist": self._persist,
            "dir": self.p["record_dir"], "frame_hz": self.p["record_frame_hz"],
            "frame_hz_range": RECORD_FRAME_HZ_RANGE, "min_free_gb": self.p["record_min_free_gb"]}

    def _op_record_topics(self, conn, req):
        return True, "", {"topics": bag_recorder.describe_topics(
            self.get_topic_names_and_types())}

    def _op_record_delete(self, conn, req):
        ok, msg = self.recorder.delete(req.get("name", ""))
        return ok, msg, None

    def _op_record_download(self, conn, req):
        """The session as a .tar.gz, streamed in file frames: never whole in memory."""
        writer = _ChunkWriter(conn, req.get("id"))
        if not self.recorder.archive_into(req.get("name", ""), writer):
            return False, f"no such session {req.get('name')!r}", None
        writer.finish()
        return True, f"{writer.sent / 1048576.0:.1f} MB sent", {"name": req.get("name")}

    def _record_sample(self):
        self.recorder.tick()
        self.bags.tick()
        if self.bags.recording and not self.recorder.recording:
            self.get_logger().warn("the session ended with a bag still recording: closing it")
            self.bags.stop()
        if not self.recorder.recording:
            return
        state = self._last.get("state") or self._state(time.monotonic())
        self.recorder.sample({k: state.get(k) for k in ("stamp", "boat", "att", "fcu",
                                                        "health", "mission")}
                             | {"targets": (self._last.get("targets") or {}).get("items", [])})

    # ------------------------------------------------------------------ the rest

    def _power_state(self):
        if not self.p["allow_power"]:
            return {"available": False, "locked": True, "reason": "disabled by allow_power",
                    "hostname": self._hostname}
        ok, why = power_client.available(self.p["power_socket"])
        s = self._status.get(time.monotonic())
        reason = ("vehicle is ARMED" if s and s[1] else
                  "autopilot status unknown: cannot confirm the vehicle is disarmed"
                  if s is None else "")
        return {"available": ok, "reason": why, "locked": bool(reason), "lock_reason": reason,
                "hostname": self._hostname}

    def _op_power(self, conn, req):
        verb = req.get("verb")
        if verb not in power_client.VERBS:
            return False, f"unknown power verb {verb!r}", None
        state = self._power_state()
        if not self.p["allow_power"] or state["locked"]:
            return False, f"refused: {state.get('lock_reason') or state['reason']}", None
        if str(req.get("hostname") or "").strip() != self._hostname:
            return False, f"type the hostname ({self._hostname}) to confirm", None
        self.get_logger().warn(f"{verb.upper()} requested from the ground station and accepted")
        try:
            return True, power_client.request(verb, self.p["power_socket"],
                                              reason=f"gcs_bridge on {self._hostname}"), None
        except (power_client.PowerUnavailable, ValueError) as e:
            return False, str(e), None

    def _op_radio_test(self, conn, req):
        if not self._radio_test.wait_for_service(timeout_sec=1.0):
            return False, "rfd_link_node does not offer /crsd/radio/send_test: is it up?", None
        done = threading.Event()
        future = self._radio_test.call_async(Trigger.Request())
        future.add_done_callback(lambda _f: done.set())
        if not done.wait(5.0):
            return False, "rfd_link_node did not answer within 5 s", None
        res = future.result()
        return bool(res.success), res.message, None

    def _op_log_level(self, conn, req):
        conn.min_level = max(0, min(50, int(req.get("level", 20))))
        records, _, _ = self.logs.read(0, conn.min_level, limit=REPLAY_EVENTS)
        conn.send({"t": "log", "items": records, "replay": True})
        return True, f"log level {conn.min_level}", None

    def destroy_node(self):
        """Stops the server but LEAVES THE NODES RUNNING: closing the ground station must not
        take the boat's stack down."""
        self._stop.set()
        try:
            self._server.close()
        except OSError:
            pass
        for conn in self._connections():
            conn.close()
        if self.recorder.recording:
            if self.bags.recording:
                self.bags.stop()
            self.recorder.stop()
        self._pool.shutdown(wait=False)
        super().destroy_node()


def rescale_jpeg(jpeg, width, quality):
    """(jpeg, w, h) no wider than `width`; the original bytes when cv2 is missing."""
    try:
        import cv2
        import numpy as np
    except ImportError:
        return jpeg, 0, 0
    img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        return jpeg, 0, 0
    h, w = img.shape[:2]
    if w > width:
        img = cv2.resize(img, (width, max(1, round(h * width / w))), interpolation=cv2.INTER_AREA)
        h, w = img.shape[:2]
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), int(quality)])
    return (buf.tobytes() if ok else jpeg), w, h


def pack_xy(msg, n_max):
    """lidar_node's preview (packed float32 x, y, z) -> (int16 x, y pairs, count)."""
    import numpy as np
    if msg.point_step != 12 or not msg.width:
        return b"", 0
    pts = np.frombuffer(bytes(msg.data), "<f4").reshape(-1, 3)[:, :2]
    if len(pts) > n_max:
        pts = pts[np.linspace(0, len(pts) - 1, n_max).astype(int)]
    xy = np.clip(np.round(pts / LIDAR_SCALE), -32767, 32767).astype("<i2")
    return xy.tobytes(), len(xy)


def _oak_controls():
    """camera_node's control table. Soft: a ground station without crusader_perception
    still starts, with nothing to tune."""
    try:
        from crusader_perception import oak_controls
        return oak_controls
    except Exception:  # noqa: BLE001
        return None


def camera_groups():
    oc = _oak_controls()
    if oc is None:
        return []
    groups = {}
    for c in oc.ALL:
        groups.setdefault(c.group, []).append(c.name)
    return [{"group": g, "controls": names} for g, names in groups.items()]


def camera_choices():
    oc = _oak_controls()
    return {} if oc is None else {c.name: list(c.choices) for c in oc.ALL if c.choices}


def main(args=None):
    run_node(GcsBridge, args=args)


if __name__ == "__main__":
    main()
