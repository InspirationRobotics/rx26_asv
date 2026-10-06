"""task3_world — Task 3 in Gazebo: RoboCommand, the dock detector's eye, the water, and the referee.

    ros2 run crusader_sim task3_world --ros-args -p course:=task3

Runs in: the crsd-sim container, started by gz_rig_up.sh for a course with a dock.
Its partner on the WSL host, crusader_sim.task3_gz_agent, does the two things
only gz-transport can do - recolour the dock's windows (visual_config) and draw
the water (markers) - on this node's word, over udp 127.0.0.1:14558.

    /sim/crusader/odometry      the boat's TRUE pose (Gazebo)
    /sim/cannon/joint_states    the cannon's TRUE pan/tilt (the joints SERVO11/12 drive)
    /crsd/pump_state            the pump output as the autopilot reports it (SERVO10)
    /crsd/docking_report, /crsd/firefighting_report,
    /crsd/resource_delivery_request, /crsd/uav_resource_request   the boat's reports
  ->
    /dock/observations          crusader_msgs/DockObservation, 15 Hz: what the dock
                                detector would publish, from GEOMETRY (below)
    /crsd/ocs_command           RoboCommand's ReadinessConfirm for the docking report
    /sim/task3/status           JSON, 2 Hz: lights, water, truth, the judge so far
    /sim/task3/verdict          JSON, latched: the referee's verdict

WHAT IT REUSES, NOT REWRITES. tools/task3_sim/world.py is the team's Task 3
world (the no-ROS sim the trees were written against): the dock from the build
guide, RoboCommand's Lights (dark until the boat reports docking in the GREEN
bay, RED until `extinguish_s` of water on the window, GREEN 5 s, off 1 s, then
the tier's code for 60 s), the Judge, and the Camera that turns a pose into the
CV team's DockObservation field for field - with the CV report's error rates,
the 188-row hull band and the camera's pitch, and target_pattern from the CV
team's own timing stage (vendor/dock_sequence_core.py). Here those run on
GAZEBO's truth instead of world.py's kinematic boat. The dock's geometry is
gen_world.dock()'s (the two were written to match): the course's dock x, y is
the deck's back edge; the faces stand FACE_U out from it.

THE WATER: a drag-free parabola from the cannon's TRUE pivot along its TRUE
pan/tilt at the exit speed cannon_aim_core uses (cannon_aim_node's params: the
measured 3.0 m at 45 deg), crossed with the GREEN bay's face. Gazebo's gravity
acts on bodies; a jet would need thousands of spawned droplets, and each one
would only follow this same curve. The deck's edge (3.5 cm in front of the
face, 0.3 m high) can stop the water short.

THE REFEREE says PASS when: the docking report names the GREEN bay AND the boat
really is in it, the fire went out (water on the lit window for extinguish_s),
the firefighting report names that window, and (Advanced/Disruptive) the
request and the UAV relay carry the code that was flashed - with no hull
contact. "Docked" here is the BOW inside the slip (every hull corner between
the fingers, the bow no further out than their tips). The cannon tree docks
with the LiDAR 1.1 m from the deck edge, which puts the stern ~8 cm inside the
2 m fingers' tips (at 1.2 m one run reported with it 8 cm out), and watches the
code from 1.6 m, which leaves the stern ~0.4 m out. The handbook does not say which counts; the
docking report is judged where the boat is when it is sent. `docked_rule`
whole_hull is the strict one (world.py's).
"""
import json
import math
import os
import random
import socket
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from crusader_msgs.msg import DockBay, DockObservation, DockWindow, PumpState

from crusader_common import config as crsd_config
from crusader_fcu import cannon_aim_core as ca
from crusader_sim import course as C


def _repo():
    """The rx26_asv checkout (tools/ is not installed)."""
    for p in (os.environ.get("RX26_SRC", ""), "/root/robotx_ws/src/rx26_asv",
              os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))):
        if p and os.path.isdir(os.path.join(p, "tools", "task3_sim")):
            return p
    raise RuntimeError("cannot find tools/task3_sim (set RX26_SRC to the rx26_asv checkout)")


sys.path.insert(0, os.path.join(_repo(), "tools", "task3_sim"))
import world as T3   # noqa: E402  (tools/task3_sim/world.py: Lights, Camera, Judge, Dock)

AGENT_PORT = 14558          # task3_gz_agent on the WSL host (1455x: 14550-14556 are taken)
FACE_U = 0.965              # gen_world.dock(): the face's front, out from the deck's back edge
DECK_EDGE_U = 1.0           # ...and the deck's front edge (the LiDAR's back wall)
DECK_Z = 0.3                # gen_world.dock() dz: the deck's top above the water
HULL_L, HULL_B = 1.0, 0.6   # crusader_hull.yaml
TIER_OF = {"core": 0, "advanced": 1, "disruptive": 2}
COLOUR_NAME = {T3.OFF: "off", T3.RED: "red", T3.GREEN: "green", T3.BLUE: "blue"}


def quat_to_R(q):
    w, x, y, z = q
    return ((1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)),
            (2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)),
            (2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)))


def mat_vec(R, v):
    return tuple(R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2] for i in range(3))


class TruthBoat:
    """Gazebo's pose of the model (origin: hull-bottom datum, REP-103 body), in
    the shapes world.py's Camera and Dock ask a boat for."""

    def __init__(self, cp: ca.CannonParams, lidar_xy):
        self.cp = cp
        self.lidar_xy = lidar_xy
        self.t = (0.0, 0.0, 0.0)
        self.R = quat_to_R((1.0, 0.0, 0.0, 0.0))
        self.yaw = 0.0                     # ENU, radians
        self.have = False

    def update(self, m: Odometry):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        self.t = (p.x, p.y, p.z)
        self.R = quat_to_R((q.w, q.x, q.y, q.z))
        self.yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.have = True

    def to_world(self, p):
        r = mat_vec(self.R, p)
        return (r[0] + self.t[0], r[1] + self.t[1], r[2] + self.t[2])

    def heading(self):
        """Compass degrees."""
        return (90.0 - math.degrees(self.yaw)) % 360.0

    # --- world.py's Boat interface ---
    def camera(self):
        c = self.to_world((self.cp.cam_x, self.cp.cam_y, self.cp.cam_z))
        return (c[0], c[1], c[2]), (self.heading() - self.cp.cam_yaw_deg) % 360.0

    def corners(self):
        """Hull corners (e, n): bow left, bow right, stern right, stern left."""
        out = []
        for x, y in ((HULL_L / 2, HULL_B / 2), (HULL_L / 2, -HULL_B / 2),
                     (-HULL_L / 2, -HULL_B / 2), (-HULL_L / 2, HULL_B / 2)):
            w = self.to_world((x, y, 0.0))
            out.append((w[0], w[1]))
        return out


def scenario(course, cp: ca.CannonParams, seed):
    """world.py's Scenario for this course, in Gazebo's world frame."""
    d = next(e for e in course["elements"] if e.get("type") == "dock")
    psi = math.radians(float(d.get("facing_deg", 180.0)))         # ENU yaw, faces OUT
    out = (math.cos(psi), math.sin(psi))
    lit = d.get("lit_window", {"slot": 0})
    sc = T3.Scenario(
        seed=seed, origin=(course["origin"]["lat"], course["origin"]["lon"]),
        dock_e=float(d["x"]) + FACE_U * out[0], dock_n=float(d["y"]) + FACE_U * out[1],
        facing_deg=(90.0 - math.degrees(psi)) % 360.0, deck_z=DECK_Z,
        green_bay=int(d.get("green_bay", 2)),
        tier=TIER_OF.get(str(course.get("tier", "disruptive")), 2),
        target_window=int(lit.get("slot", 0)),
        code=tuple(course.get("code", ("red", "blue"))),
        extinguish_s=float(course.get("extinguish_s", 2.0)),
        cam_pitch_deg=cp.cam_pitch_deg)
    return sc


def _nan(v):
    return float("nan") if v is None else float(v)


def obs_to_msg(obs, stamp):
    """world.py's DockObservation dict -> crusader_msgs/DockObservation."""
    m = DockObservation()
    m.header.stamp = stamp
    m.header.frame_id = "camera_link"
    for b in obs["bays"]:
        bm = DockBay()
        bm.bay_index = int(b["bay_index"])
        bm.detector_confidence = float(b["detector_confidence"])
        bm.bbox = [0, 0, 0, 0]
        bm.truncated = bool(b["truncated"])
        bm.indicator_present = bool(b["indicator_present"])
        bm.indicator_colour = int(b["indicator_colour"])
        bm.indicator_confidence = float(b["indicator_confidence"])
        bm.indicator_bbox = [0, 0, 0, 0]
        for w in b["windows"]:
            wm = DockWindow()
            wm.index = int(w["index"])
            wm.slot = str(w["slot"])
            wm.identity_confidence = float(w["identity_confidence"])
            wm.state = int(w["state"])
            wm.state_confidence = float(w["state_confidence"])
            wm.lit_score = float(w["lit_score"])
            wm.detector_confidence = float(w["detector_confidence"])
            wm.bbox = [0, 0, 0, 0]
            wm.has_position = bool(w["has_position"])
            wm.x, wm.y, wm.z = float(w["x"]), float(w["y"]), float(w["z"])
            bm.windows.append(wm)
        bm.lit_window_index = int(b["lit_window_index"])
        bm.lit_state = int(b["lit_state"])
        bm.has_plane = bool(b["has_plane"])
        bm.plane_normal = [_nan(v) for v in b["plane_normal"]]
        bm.plane_offset = _nan(b["plane_offset"])
        bm.plane_rms_m = _nan(b["plane_rms_m"])
        bm.range_from_size_m = _nan(b["range_from_size_m"])
        bm.bearing_deg = float(b["bearing_deg"])
        m.bays.append(bm)
    m.target_pattern = str(obs["target_pattern"])
    m.target_colours = [str(c) for c in obs["target_colours"]]
    m.target_window_index = int(obs["target_window_index"])
    m.last_event = str(obs["last_event"])
    m.observed_fps = float(obs["observed_fps"])
    return m


class Task3World(Node):

    def __init__(self):
        super().__init__("task3_world")
        self.course_name = self.declare_parameter("course", "task3").value
        seed = int(self.declare_parameter("seed", 1).value)
        self.docked_rule = str(self.declare_parameter("docked_rule", "bow_in").value)
        self.agent_port = int(self.declare_parameter("agent_port", AGENT_PORT).value)
        course = C.load(self.course_name)
        # the cannon's and the camera's numbers: the aim's own (one source)
        d = dict(crsd_config.node_params("cannon_aim_node"))
        cam = crsd_config.node_params("target_tracker")
        for k in ("cam_x", "cam_y", "cam_z", "cam_yaw_deg", "cam_pitch_deg"):
            d[k] = cam.get(k, 0.0)
        self.cp = ca.CannonParams.from_dict(d)
        lid = crsd_config.node_params("lidar_cluster_node")
        self.lidar = (float(lid["lidar_x"]), float(lid["lidar_y"]), float(lid["lidar_z"]))
        self.sc = scenario(course, self.cp, seed)
        self.dock = T3.Dock(self.sc)
        self.lights = T3.Lights(self.sc)
        self.rnd = random.Random(seed)
        self.camera = T3.Camera(self.sc, self.rnd)
        self.judge = T3.Judge(self.sc)
        self.boat = TruthBoat(self.cp, self.lidar[:2])
        self.pan_deg = 0.0
        self.tilt_deg = 0.0
        self.pump = 0.0
        self.t0 = time.monotonic()
        self.t_last = 0.0
        self.next_cam = 0.0
        self.in_contact = False
        self.stream = None                 # the latest stream, for status and the agent
        self.hits_s = 0.0                  # water on the TARGET window, all told
        self.water_s = 0.0                 # water in the air, all told
        self.shots = []                    # (t_on, t_off, where) per stretch of water
        self._shot = None
        self.dock_truth = None             # the truth when the docking report came
        self.lit_t = None
        self.events_done = 0
        self.verdict = None
        self._sent_lights = None
        self._lights_t = -1.0
        self._stream_sent = False
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.obs_pub = self.create_publisher(DockObservation, "dock/observations", 10)
        self.ocs_pub = self.create_publisher(String, "/crsd/ocs_command", 10)
        self.status_pub = self.create_publisher(String, "/sim/task3/status", 10)
        self.verdict_pub = self.create_publisher(String, "/sim/task3/verdict", latched)
        self.create_subscription(Odometry, "/sim/crusader/odometry", self.boat.update, 10)
        self.create_subscription(JointState, "/sim/cannon/joint_states", self._on_joints, 10)
        self.create_subscription(PumpState, "/crsd/pump_state", self._on_pump, 10)
        for topic, kind in (("/crsd/docking_report", "docking"),
                            ("/crsd/firefighting_report", "firefighting"),
                            ("/crsd/resource_delivery_request", "request"),
                            ("/crsd/uav_resource_request", "uav")):
            self.create_subscription(String, topic,
                                     lambda m, k=kind: self._on_report(k, m), 10)
        self.create_timer(0.05, self._step)
        self.create_timer(0.5, self._status)
        sc = self.sc
        self.get_logger().info(
            f"task3 world: course {self.course_name}, GREEN bay {sc.green_bay}, the fire in window "
            f"{sc.target_window} ({'UL' if sc.target_window == 0 else 'LR'}), tier {sc.tier}, "
            f"code {sc.code[0]}->{sc.code[1]}, {sc.extinguish_s:.1f} s of water puts it out | "
            f"water {self.cp.exit_speed_mps:.2f} m/s from ({self.cp.nozzle_x:.2f}, "
            f"{self.cp.nozzle_y:.2f}, {self.cp.nozzle_z:.2f}) | camera pitch "
            f"{sc.cam_pitch_deg:+.1f} deg | docked = {self.docked_rule}")

    # ---------------------------------------------------------------- inputs

    def _on_joints(self, m: JointState):
        for name, pos in zip(m.name, m.position):
            if name.endswith("cannon_pan_joint"):
                self.pan_deg = math.degrees(pos)
            elif name.endswith("cannon_tilt_joint"):
                self.tilt_deg = math.degrees(pos)

    def _on_pump(self, m: PumpState):
        self.pump = 1.0 if (m.output_fresh and m.on) else 0.0

    def now(self):
        return time.monotonic() - self.t0

    def _on_report(self, kind, m: String):
        t = self.now()
        try:
            j = json.loads(m.data)
        except ValueError:
            self.get_logger().error(f"{kind} report is not JSON: {m.data!r}")
            return
        n0 = len(self.judge.events)
        if kind == "docking":
            truth = self.truth_bay()
            self.judge.on_docking(t, int(j.get("bay_id", 0)), truth, self.lights)
            if self.dock_truth is None:
                self.dock_truth = self.berth_truth()
        elif kind == "firefighting":
            self.judge.on_firefighting(t, int(j.get("window_id", 0)), self.lights)
        elif kind == "request":
            self.judge.on_request(t, j)
        elif kind == "uav":
            self.judge.on_uav(t, j)
        self._say_events(n0)
        self._judge_verdict()

    # ---------------------------------------------------------------- truth

    def truth_bay(self):
        """The bay the boat is docked in (docked_rule), or 0."""
        corners = self.boat.corners()
        uv = [self.dock.uv(c) for c in corners]
        for i in (1, 2, 3):
            lo, hi = self.dock.slip(i)
            if not all(lo <= u <= hi and v >= 0.0 for u, v in uv):
                continue
            reach = [v for u, v in (uv if self.docked_rule == "whole_hull" else uv[:2])]
            if max(reach) <= self.sc.finger_len:
                return i
        return 0

    def berth_truth(self):
        """Where the boat really is against the GREEN bay: the LiDAR's standoff from
        the deck edge, the body origin's offset from the slip centreline (+ left as
        seen from the boat, facing in) and the bow's angle off the slip's axis."""
        b = self.sc.green_bay
        lo, hi = self.dock.slip(b)
        lid = self.boat.to_world(self.lidar)
        u_l, v_l = self.dock.uv((lid[0], lid[1]))
        u_o, v_o = self.dock.uv((self.boat.t[0], self.boat.t[1]))
        edge_v = DECK_EDGE_U - FACE_U
        square = (self.sc.facing_deg + 180.0) % 360.0           # bow straight in
        return {"bay": self.truth_bay(),
                "lidar_to_deck_edge_m": round(v_l - edge_v, 3),
                # dock u runs to the boat's RIGHT when it faces in: a boat right of
                # the centre has the centreline on its left (DockSlot.lateral_m's sign)
                "centreline_left_m": round(u_o - (lo + hi) / 2.0, 3),
                "yaw_off_square_deg": round(T3.wrap180(self.boat.heading() - square), 2)}

    # ---------------------------------------------------------------- the water

    def _water(self):
        """The stream now, or None with the pump off: its points, where it meets the
        GREEN bay's face, and which window (or not) it lands in."""
        if self.pump < 0.5 or not self.boat.have:
            return None
        o = self.boat.to_world((self.cp.nozzle_x, self.cp.nozzle_y, self.cp.nozzle_z))
        dvec = mat_vec(self.boat.R, ca.direction(self.pan_deg, self.tilt_deg))
        v = self.cp.exit_speed_mps
        bay = self.sc.green_bay
        face = self.dock.faces[bay]
        n = (self.dock.out[0], self.dock.out[1], 0.0)
        hit = ca.crossing(o, dvec, v, (face[0], face[1], self.dock.face_z), n)
        where, window, miss = "nowhere near the face", None, None
        t_end = 1.2
        if hit is not None:
            t_end, p = hit
            edge = self.dock.at(0.0, DECK_EDGE_U - FACE_U)
            e = ca.crossing(o, dvec, v, (edge[0], edge[1], 0.0), n)
            if e is not None and e[0] < t_end and e[1][2] < DECK_Z:
                t_end, p = e
                where = "short: on the deck edge"
            else:
                r = T3.dot(T3.sub((p[0], p[1]), (face[0], face[1])), self.dock.right)
                where = "on the face, missed both windows"
                for idx, _s, (we, wn, wz), (hw, hh) in self.dock.windows(bay):
                    rr = T3.dot(T3.sub((p[0], p[1]), (we, wn)), self.dock.right)
                    if idx == self.sc.target_window:
                        miss = (round(rr, 3), round(p[2] - wz, 3))   # + right, + high
                    if abs(rr) <= hw and abs(p[2] - wz) <= hh:
                        window = idx
                        where = f"IN window {idx}"
                if window is None and (abs(r) > T3.FACE_W / 2 or p[2] > self.dock.face_z + 0.5):
                    where = "off the face"
        pts = []
        for k in range(13):
            q = ca.trajectory(o, dvec, v, t_end * k / 12.0)
            pts.append([round(q[0], 3), round(q[1], 3), round(q[2], 3)])
            if q[2] < 0.0:
                break
        return {"points": pts, "end": pts[-1], "window": window, "where": where,
                "miss_m": miss, "pan_deg": round(self.pan_deg, 1), "tilt_deg": round(self.tilt_deg, 1)}

    # ---------------------------------------------------------------- the loop

    def _step(self):
        t = self.now()
        dt = max(0.0, t - self.t_last)
        self.t_last = t
        if not self.boat.have:
            return
        w = self._water()
        self.stream = w
        on_target = w is not None and w["window"] == self.sc.target_window
        if w is not None:
            self.water_s += dt
            if on_target and self.lights.state == "FIRE":
                self.hits_s += dt
            if self._shot is None:
                self._shot = {"t_on": round(t, 2), "where": w["where"], "miss_m": w["miss_m"]}
            else:
                self._shot["where"], self._shot["miss_m"] = w["where"], w["miss_m"]
        elif self._shot is not None:
            self._shot["t_off"] = round(t, 2)
            self.shots.append(self._shot)
            self.get_logger().info(f"[referee] water {self._shot['t_off'] - self._shot['t_on']:.1f} s: "
                                   f"{self._shot['where']} (miss {self._shot['miss_m']} m right/high)")
            self._shot = None
        before = self.lights.state
        self.lights.step(t, on_target, dt)
        if self.lights.state != before:
            self.get_logger().info(f"[referee] lights: {before} -> {self.lights.state}")
            if self.lights.state == "FIRE":
                self.lit_t = t
            self._judge_verdict()
        corners = self.boat.corners()
        touching = self.dock.contact(corners)
        if touching and not self.in_contact:
            self.judge.contacts += 1
            self.judge.log(t, "course", "hull contact with the dock", False)
            self.get_logger().warn("[referee] hull contact with the dock")
        self.in_contact = touching
        if self.judge.confirm_at is not None and t >= self.judge.confirm_at:
            self.judge.confirm_at = None
            self.judge.log(t, "RoboCommand", "ReadinessConfirm sent", True)
            self.ocs_pub.publish(String(data=json.dumps(
                {"readiness_confirm": {"report_seq": 1, "vehicle_id": "USV1"}})))
        if t >= self.next_cam:
            self.next_cam = t + 1.0 / T3.Camera.FPS
            obs = self.camera.frame(t, self.boat, self.dock, self.lights)
            self.obs_pub.publish(obs_to_msg(obs, self.get_clock().now().to_msg()))
        self._agent(t, w)

    def _agent(self, t, w):
        """Tell the WSL-side agent what Gazebo should show."""
        lit_w, lit_c = self.lights.window_colour(t)
        lights = {}
        for b in (1, 2, 3):
            for idx in (0, 1):
                c = lit_c if (b == self.sc.green_bay and idx == lit_w) else T3.OFF
                lights[f"bay{b}_win{idx}"] = COLOUR_NAME.get(c, "off")
        if lights != self._sent_lights or t - self._lights_t > 2.0:
            self._send({"lights": lights})
            self._sent_lights, self._lights_t = lights, t
        if w is not None:
            self._send({"stream": {"points": w["points"], "hit": w["window"] is not None}})
            self._stream_sent = True
        elif self._stream_sent:
            self._send({"stream": None})
            self._stream_sent = False

    def _send(self, d):
        try:
            self._udp.sendto(json.dumps(d).encode(), ("127.0.0.1", self.agent_port))
        except OSError:
            pass

    # ---------------------------------------------------------------- the referee

    def _say_events(self, n0):
        for e in self.judge.events[n0:]:
            mark = {True: "OK", False: "WRONG", None: "--"}[e["ok"]]
            self.get_logger().info(f"[referee] {e['who']}: {e['text']}  [{mark}]")

    def _judge_verdict(self):
        j, sc = self.judge, self.sc
        s = j.summary()
        need_req = sc.tier >= 1
        lines = []
        dt = self.dock_truth or {}
        if j.docking is not None:
            lines.append(f"docking: bay {j.docking['bay_id']} = the GREEN bay, and docked in it "
                         f"(LiDAR {dt['lidar_to_deck_edge_m']:.2f} m from the deck edge, centreline "
                         f"{dt['centreline_left_m']:+.2f} m left, {dt['yaw_off_square_deg']:+.1f} deg "
                         "off square)" if dt else f"docking: bay {j.docking['bay_id']} accepted")
        elif j.docking_heard:
            lines.append("docking: reported, NOT accepted (wrong bay, or not really in it)")
        else:
            lines.append("docking: no report")
        out = self.lights.hit_t is not None
        if out:
            lines.append(f"fire: window {sc.target_window} lit, out after {self.hits_s:.1f} s of water "
                         f"on it ({self.water_s:.1f} s of water in all, {len(self.shots) + (self._shot is not None)} stretch(es))")
        elif self.lights.state == "FIRE":
            lines.append(f"fire: window {sc.target_window} STILL BURNING ({self.hits_s:.1f} of "
                         f"{sc.extinguish_s:.1f} s of water on it)")
        else:
            lines.append(f"fire: never lit (lights {self.lights.state})")
        if j.fire is not None:
            lines.append(f"firefighting report: window_id {j.fire['window_id']} "
                         f"({'right' if j.fire['ok'] else 'WRONG'}; lit = {sc.target_window + 1})")
        else:
            lines.append("firefighting report: none")
        if need_req:
            want = j.expected_request()
            lines.append("request: " + (f"{j.request['got'][0]} -> {j.request['got'][1]} "
                                        f"({'right' if j.request['ok'] else 'WRONG'}; flashed "
                                        f"{want[0]} -> {want[1]})" if j.request else "none"))
            lines.append("UAV: " + (f"tasked {j.uav['got'][0]} -> {j.uav['got'][1]} "
                                    f"({'right' if j.uav['ok'] else 'WRONG'})" if j.uav else "not tasked"))
        lines.append(f"contacts: {j.contacts}")
        ok = (s["docking"] and out and s["firefighting"] and j.contacts == 0
              and (not need_req or (s["request"] and s["uav"])))
        complete = (j.fire is not None) and (not need_req or (j.request is not None and j.uav is not None))
        v = {"pass": bool(ok), "complete": bool(complete), "lines": lines, "summary": s,
             "dock_truth": self.dock_truth, "shots": self.shots[-10:]}
        if v != self.verdict:
            self.verdict = v
            self.verdict_pub.publish(String(data=json.dumps(v)))
            if complete:
                self.get_logger().info("[referee] VERDICT: " + ("PASS" if ok else "FAIL") + " | "
                                       + " | ".join(lines))

    def _status(self):
        t = self.now()
        lw, lc = self.lights.window_colour(t)
        w = self.stream
        st = {"t": round(t, 1), "lights": self.lights.state, "window": lw,
              "colour": COLOUR_NAME.get(lc, "?"), "sprayed_s": round(self.lights.sprayed, 2),
              "water_on": w is not None, "water": None if w is None else
              {k: w[k] for k in ("where", "miss_m", "pan_deg", "tilt_deg")},
              "truth_bay": self.truth_bay() if self.boat.have else None,
              "berth": self.berth_truth() if self.boat.have else None,
              "contact": self.in_contact, "judge": self.judge.summary(),
              "verdict": None if self.verdict is None else
              {"pass": self.verdict["pass"], "complete": self.verdict["complete"]}}
        self.status_pub.publish(String(data=json.dumps(st)))
        if self.verdict is None:
            self._judge_verdict()


def main(args=None):
    rclpy.init(args=args)
    node = Task3World()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.verdict is not None:
            print("[referee] at exit: " + ("PASS" if node.verdict["pass"] else "FAIL") + " | "
                  + " | ".join(node.verdict["lines"]), flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
