"""cannon_aim_node — the pan/tilt water cannon: aim at the window the tree names, and fire once on it.

Subscribes:
  /crsd/water_cannon  std_msgs/String (JSON)    the tree (SprayUntilHit), every tick
                      it wants water: {"fire": true, "frame_id": "camera_link",
                      "x", "y", "z"} = the burning window's aim point from the dock
                      detector, and "window": "UL" / "LR" (optional) for that
                      window's own pan/tilt trim; {"fire": false} when it stops
  /crsd/attitude      crusader_msgs/Attitude    roll/pitch, so the stream droops
                      in the world and not in the hull
  /crsd/cannon_state  crusader_msgs/CannonState the two servos' real outputs
  /crsd/pump_state    crusader_msgs/PumpState   for the log
  /crsd/dock_slot     crusader_msgs/DockSlot    the slip's back wall, and
  /crsd/wall_range    crusader_msgs/WallRange   the wall in front: the face's plane,
                      for the distance (aim_point_source geometry, the default:
                      the camera's bearing, the LiDAR's distance, the window's
                      known height - cannon_aim_core.geometric_target)
Publishes:
  /crsd/cannon_cmd    crusader_msgs/CannonCommand  pan/tilt PWM -> telemetry_bridge
  /crsd/pump_cmd      crusader_msgs/PumpCommand    water, ONLY with fire_pump (G7)
  /crsd/cannon_status std_msgs/String (JSON)       what it is doing, and why not

THE LOOP (Bumblebee's lesson from 2024: a slow aim loop lags the boat's motion,
so this re-solves every aim_rate_hz on the newest aim point and attitude, and
keeps doing so WHILE the water runs): the aim point -> cannon_aim_core.solve ->
the servos; when SERVO_OUTPUT_RAW says both servos are at the command and have
been for settle_s, the water goes ON (FireGate) and STAYS on: a burst_s burst
asked every resend_s, each extending the last at the bridge, while the servos
keep tracking, until the tree says stop (the window reads GREEN). No penalty
for too much water. The bridge's pump gates still apply to every ask.

STOPPING. The tree says fire false, or goes quiet for cannon_timeout_s, or the
aim point goes stale: the pump is sent OFF (once) and the cannon holds where it
is. A burst already out ends on the autopilot's clock (DO_REPEAT_SERVO).

fire_pump false (the default, as bt_runner_node's): the whole loop runs and the
bursts are logged as DRY, nothing goes to the pump. The servos move either way.
"""
import json
import math
import statistics
import time
from collections import deque

from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from std_msgs.msg import String

from crusader_msgs.msg import (Attitude, CannonCommand, CannonState, DockSlot, PumpCommand,
                               PumpState, WallRange)

from crusader_common import config as crsd_config
from crusader_common.node_main import run_node
from crusader_common.param_utils import declare_from_config

from crusader_fcu import cannon_aim_core as ca

RO = dict(read_only=True)
PARAM_SPEC = {
    # THE MODEL, live (2026-10-11: tuned on the water rather than trimmed): where
    # the nozzle is, and how fast the water leaves it - from one measured throw
    # unless exit_speed_mps is given. A change rebuilds the aim at once.
    "nozzle_x": dict(read_only=False, lo=-1.0, hi=2.0,
                     description="nozzle pivot, m forward of the body origin"),
    "nozzle_y": dict(read_only=False, lo=-1.0, hi=1.0,
                     description="nozzle pivot, m to PORT of the centreline (the camera's is cam_y)"),
    "nozzle_z": dict(read_only=False, lo=0.0, hi=2.0,
                     description="nozzle pivot, m above the hull-bottom datum"),
    "exit_speed_mps": dict(read_only=False, lo=0.0, hi=50.0,
                           description="the water's speed, m/s; 0 = from the measured throw (throw_*)"),
    "throw_range_m": dict(read_only=False, lo=0.1, hi=50.0,
                          description="measured throw: how far out it landed, m (horizontal)"),
    "throw_elev_deg": dict(read_only=False, lo=1.0, hi=89.0,
                           description="measured throw: the nozzle's angle above level, deg"),
    "throw_drop_m": dict(read_only=False, lo=-1.0, hi=3.0,
                         description="measured throw: it landed this far BELOW the nozzle, m "
                                     "(on the water ~0.51; 0 = level with it)"),
    "pan_center_us": RO, "pan_us_per_deg": RO, "pan_sign": RO,
    "tilt_center_us": RO, "tilt_us_per_deg": RO, "tilt_sign": RO,
    "pwm_min": dict(read_only=True, lo=800, hi=2200),
    "pwm_max": dict(read_only=True, lo=800, hi=2200),
    "pan_min_deg": RO, "pan_max_deg": RO, "tilt_min_deg": RO, "tilt_max_deg": RO,
    # each window's own trim, added to its solution (none for an aim that names
    # no window: cal point / deg)
    "pan_trim_ul_deg": dict(read_only=False, lo=-45.0, hi=45.0,
                            description="UPPER-LEFT window: added to the pan, deg (+ = left)"),
    "tilt_trim_ul_deg": dict(read_only=False, lo=-45.0, hi=45.0,
                             description="UPPER-LEFT window: added to the tilt, deg (+ = up)"),
    "pan_trim_lr_deg": dict(read_only=False, lo=-45.0, hi=45.0,
                            description="LOWER-RIGHT window: added to the pan, deg (+ = left)"),
    "tilt_trim_lr_deg": dict(read_only=False, lo=-45.0, hi=45.0,
                             description="LOWER-RIGHT window: added to the tilt, deg (+ = up)"),
    # THE AIM POINT (2026-10-18, no trims): "geometry" = the camera's bearing, the
    # LiDAR's face plane for the distance, the window's known height for the
    # height (cannon_aim_core.geometric_target); "camera" = the stereo point as is
    "aim_point_source": dict(read_only=False,
                             description="geometry | camera: where the aim point comes from"),
    "window_height_ul_m": dict(read_only=False, lo=0.0, hi=3.0,
                               description="UPPER-LEFT window's centre above the water, m "
                                           "(0 = unknown: the camera's height)"),
    "window_height_lr_m": dict(read_only=False, lo=0.0, hi=3.0,
                               description="LOWER-RIGHT window's centre above the water, m "
                                           "(0 = unknown: the camera's height)"),
    "waterline_z_m": dict(read_only=True, lo=0.0, hi=1.0,
                          description="the water above the hull-bottom datum, m (the draft)"),
    "face_setback_m": dict(read_only=False, lo=-0.5, hi=0.5,
                           description="the windows' face stands this far beyond the wall "
                                       "the LiDAR measures, m"),
    "lidar_max_age_s": dict(read_only=True, lo=0.05, hi=5.0,
                            description="an older LiDAR plane is not used"),
    "aim_smooth_s": dict(read_only=False, lo=0.0, hi=2.0,
                         description="the aim point: the median of the camera frames this "
                                     "recent, s (0 = the newest alone)"),
    "aim_rate_hz": dict(read_only=True, lo=1.0, hi=100.0),
    "settle_s": dict(read_only=True, lo=0.0, hi=5.0),
    "servo_tol_us": dict(read_only=True, lo=1.0, hi=200.0),
    "aim_max_age_s": dict(read_only=True, lo=0.05, hi=10.0),
    "burst_s": dict(read_only=True, lo=0.05, hi=2.0,
                    description="each ask: water for this long from now"),
    "resend_s": dict(read_only=True, lo=0.05, hi=2.0,
                     description="ask again this often while spraying (< burst_s)"),
    "cannon_timeout_s": dict(read_only=True, lo=0.1, hi=10.0),
    "fire_pump": dict(read_only=True, description="G7: may it ask for water"),
    "status_period_s": dict(read_only=True, lo=0.05, hi=10.0),
}


class CannonAimNode(Node):

    def __init__(self):
        super().__init__("cannon_aim_node")
        p = declare_from_config(self, crsd_config.node_params("cannon_aim_node"), PARAM_SPEC)
        cam = crsd_config.node_params("target_tracker")    # one camera, one mount
        d = dict(p)
        for k in ("cam_x", "cam_y", "cam_z", "cam_yaw_deg", "cam_pitch_deg"):
            d[k] = cam.get(k, 0.0)
        self.cp = ca.CannonParams.from_dict(d)
        self._d = d                  # what cp is built from: the model's live keys change it
        # each window's own (pan, tilt) trim (0 if the params file predates
        # them: then they are not declared either)
        self.win_trim = {s: [float(p.get(f"pan_trim_{s.lower()}_deg", 0.0)),
                             float(p.get(f"tilt_trim_{s.lower()}_deg", 0.0))]
                         for s in ca.WINDOW_SLOTS}
        self.gate = ca.FireGate(ca.FireParams(
            settle_s=p["settle_s"], servo_tol_us=p["servo_tol_us"],
            aim_max_age_s=p["aim_max_age_s"], burst_s=p["burst_s"],
            resend_s=p["resend_s"]))
        self.fire_pump = bool(p["fire_pump"])
        self.timeout_s = float(p["cannon_timeout_s"])
        # the aim point's sources (geometric_target)
        self.aim_source = str(p.get("aim_point_source", "geometry")).strip().lower()
        self.win_height = {"UL": float(p.get("window_height_ul_m", 0.0)),
                           "LR": float(p.get("window_height_lr_m", 0.0))}
        self.waterline_z = float(p.get("waterline_z_m", 0.24))
        self.setback = float(p.get("face_setback_m", 0.0))
        self.lidar_max_age = float(p.get("lidar_max_age_s", 0.5))
        lid = crsd_config.node_params("lidar_cluster_node")
        self.lidar_xy = (float(lid.get("lidar_x", 0.32)), float(lid.get("lidar_y", 0.0)))
        self._slot = None            # (plane, t) from /crsd/dock_slot
        self._wall = None            # (plane, t) from /crsd/wall_range
        self._aim_from = "camera"
        self._target = None          # the body-frame aim point last solved for
        # the camera's ~2 cm of noise a frame made the pan chase it (the sim, 2026-10-18):
        # docked, the window does not move, so a short median of the frames
        self.smooth_s = float(p.get("aim_smooth_s", 0.4))
        self._hist = deque(maxlen=64)   # (t, target_body), one per camera frame
        self._hist_key = None

        self._want = False
        self._aim = None             # (x, y, z) camera_link
        self._aim_win = None         # "UL" / "LR": which window, for its own trim
        self._aim_t = -1e18          # receipt, monotonic
        self._cmd_t = -1e18          # the last /crsd/water_cannon of any kind
        self._att = (0.0, 0.0)
        self._att_t = -1e18
        self._servo = None           # CannonState
        self._pump = None            # PumpState
        self._sol = ca.AimSolution()
        self._firing = False         # bursts asked (or DRY) since the last stop
        self._log_t = -1e18          # the spray's log line, every 5 s
        self._seq = 200000           # our PumpCommand numbering (the tree's starts at 100000)
        self._bursts = 0

        self.cmd_pub = self.create_publisher(CannonCommand, "/crsd/cannon_cmd", 10)
        self.pump_pub = self.create_publisher(PumpCommand, "/crsd/pump_cmd", 10)
        self.status_pub = self.create_publisher(String, "/crsd/cannon_status", 10)
        self.create_subscription(String, "/crsd/water_cannon", self._on_cannon, 10)
        self.create_subscription(Attitude, "/crsd/attitude", self._on_att, 10)
        self.create_subscription(CannonState, "/crsd/cannon_state", self._on_servo, 10)
        self.create_subscription(PumpState, "/crsd/pump_state", self._on_pump, 10)
        self.create_subscription(DockSlot, "/crsd/dock_slot", self._on_slot, 10)
        self.create_subscription(WallRange, "/crsd/wall_range", self._on_wall, 10)
        self.add_on_set_parameters_callback(self._on_set)
        self.create_timer(1.0 / p["aim_rate_hz"], self._tick)
        self.create_timer(p["status_period_s"], self._status)
        c = self.cp
        self.get_logger().info(
            f"cannon aim: pivot ({c.nozzle_x:.2f}, {c.nozzle_y:.2f}, {c.nozzle_z:.2f}) m, "
            f"camera ({c.cam_x:.2f}, {c.cam_y:.2f}, {c.cam_z:.2f}) pitch {c.cam_pitch_deg:+.1f} deg, "
            f"exit {c.exit_speed_mps:.2f} m/s, pan {c.pan.min_us:.0f}-{c.pan.max_us:.0f} us = "
            f"{c.pan.deg_limits()[0]:+.0f}..{c.pan.deg_limits()[1]:+.0f} deg | "
            + ("FIRES THE PUMP" if self.fire_pump else "pump DRY (fire_pump false)"))

    # ---------------------------------------------------------------- inputs

    MODEL_KEYS = ("nozzle_x", "nozzle_y", "nozzle_z", "exit_speed_mps",
                  "throw_range_m", "throw_elev_deg", "throw_drop_m")

    def _on_set(self, params):
        d = dict(self._d)
        for prm in params:
            if prm.name in self.MODEL_KEYS:
                d[prm.name] = float(prm.value)
        if d != self._d:
            try:
                cp = ca.CannonParams.from_dict(d)
            except ValueError as e:      # a throw no stream could make: refused
                return SetParametersResult(successful=False, reason=str(e))
            self._d, self.cp = d, cp
            self.get_logger().info(
                f"cannon model: exit {cp.exit_speed_mps:.2f} m/s, pivot ({cp.nozzle_x:.2f}, "
                f"{cp.nozzle_y:.2f}, {cp.nozzle_z:.2f}) m")
        for prm in params:
            for s in ca.WINDOW_SLOTS:
                for i, axis in enumerate(("pan", "tilt")):
                    if prm.name == f"{axis}_trim_{s.lower()}_deg":
                        self.win_trim[s][i] = float(prm.value)
                if prm.name == f"window_height_{s.lower()}_m":
                    self.win_height[s] = float(prm.value)
            if prm.name == "aim_point_source":
                v = str(prm.value).strip().lower()
                if v not in ("geometry", "camera"):
                    return SetParametersResult(successful=False,
                                               reason="aim_point_source: geometry | camera")
                self.aim_source = v
            elif prm.name == "face_setback_m":
                self.setback = float(prm.value)
            elif prm.name == "aim_smooth_s":
                self.smooth_s = float(prm.value)
        return SetParametersResult(successful=True)

    def _on_cannon(self, msg: String):
        now = time.monotonic()
        try:
            j = json.loads(msg.data)
        except ValueError:
            self.get_logger().warn(f"bad /crsd/water_cannon: {msg.data!r}",
                                   throttle_duration_sec=5.0)
            return
        self._cmd_t = now
        want = bool(j.get("fire"))
        if want and str(j.get("frame_id", "camera_link")) != "camera_link":
            self.get_logger().error(f"aim point in {j.get('frame_id')!r}, not camera_link: ignored",
                                    throttle_duration_sec=5.0)
            want = False
        if want:
            try:
                p = (float(j["x"]), float(j["y"]), float(j["z"]))
            except (KeyError, TypeError, ValueError):
                p = None
            if p is not None and all(math.isfinite(v) for v in p):
                self._aim, self._aim_t = p, now
                self._aim_win = ca.window_slot(j.get("window"))
        if self._want and not want:
            self._stop("the tree stopped asking")
        self._want = want

    def _on_att(self, msg: Attitude):
        self._att = (float(msg.roll), float(msg.pitch))
        self._att_t = time.monotonic()

    def _on_slot(self, msg: DockSlot):
        if msg.valid and msg.has_back and math.isfinite(msg.back_range_m) \
                and math.isfinite(msg.angle_deg):
            self._slot = (ca.plane_from_slot(msg.back_range_m, msg.angle_deg, self.lidar_xy),
                          time.monotonic())

    def _on_wall(self, msg: WallRange):
        if msg.valid and math.isfinite(msg.range_m) and math.isfinite(msg.angle_deg):
            self._wall = (ca.plane_from_wall(msg.range_m, msg.angle_deg), time.monotonic())

    def _face_plane(self, now):
        """The face's plane from the LiDAR: the slip's back wall when the slip is
        seen (docked), else the wall in front; None when neither is fresh."""
        for got in (self._slot, self._wall):
            if got is not None and now - got[1] <= self.lidar_max_age:
                return got[0]
        return None

    def _on_servo(self, msg: CannonState):
        self._servo = msg

    def _on_pump(self, msg: PumpState):
        self._pump = msg

    # ---------------------------------------------------------------- the loop

    def _stop(self, why):
        if self._firing:
            if self.fire_pump:
                m = PumpCommand()
                m.header.stamp = self.get_clock().now().to_msg()
                m.duration_s = 0.0
                self._seq += 1
                m.seq = self._seq
                m.source = "cannon_aim"
                self.pump_pub.publish(m)
            self.get_logger().info(f"cannon: stop ({why}) after {self._bursts} burst(s)")
        self._firing = False
        self._bursts = 0
        self.gate.reset()
        self._hist.clear()

    def _tick(self):
        now = time.monotonic()
        if self._want and now - self._cmd_t > self.timeout_s:
            self._want = False
            self._stop(f"no /crsd/water_cannon for {self.timeout_s:.1f} s")
        if not self._want or self._aim is None:
            return
        roll, pitch = self._att if now - self._att_t <= 0.5 else (0.0, 0.0)
        trim = self.win_trim.get(self._aim_win, (0.0, 0.0))
        if self.aim_source == "geometry":
            h = self.win_height.get(self._aim_win, 0.0)
            target, self._aim_from = ca.geometric_target(
                self._aim, self.cp, roll, pitch, plane=self._face_plane(now),
                height_above_water=h if h > 0.0 else None,
                waterline_z=self.waterline_z, setback_m=self.setback)
        else:
            target, self._aim_from = ca.cam_to_body(self._aim, self.cp), "camera"
        if self.smooth_s > 0.0:
            key = (self._aim_t, self._aim_win)
            if key != self._hist_key:            # a new camera frame
                if self._hist_key is not None and key[1] != self._hist_key[1]:
                    self._hist.clear()           # another window: start again
                self._hist_key = key
                self._hist.append((now, target))
            recent = [tg for (t, tg) in self._hist if now - t <= self.smooth_s]
            if recent:
                target = tuple(statistics.median(v[k] for v in recent) for k in range(3))
        self._target = target
        sol = ca.solve(target, ca.with_trims(self.cp, *trim), roll, pitch)
        self._sol = sol
        m = CannonCommand()
        m.header.stamp = self.get_clock().now().to_msg()
        m.pan_deg, m.tilt_deg = float(sol.pan_deg), float(sol.tilt_deg)
        m.pan_pwm, m.tilt_pwm = int(sol.pan_pwm), int(sol.tilt_pwm)
        m.source = "cannon_aim"
        self.cmd_pub.publish(m)
        err = None
        s = self._servo
        if s is not None and s.output_fresh and s.pan_pwm and s.tilt_pwm:
            err = max(abs(int(s.pan_pwm) - sol.pan_pwm), abs(int(s.tilt_pwm) - sol.tilt_pwm))
        burst = self.gate.update(now, True, now - self._aim_t, sol, err)
        if burst <= 0.0:
            return
        self._firing = True
        self._bursts += 1
        if self.fire_pump:
            pm = PumpCommand()
            pm.header.stamp = self.get_clock().now().to_msg()
            pm.duration_s = float(burst)
            self._seq += 1
            pm.seq = self._seq
            pm.source = "cannon_aim"
            self.pump_pub.publish(pm)
        if self._bursts > 1 and now - self._log_t < 5.0:
            return
        self._log_t = now
        what = "water ON" if self._bursts == 1 else f"spraying ({self._bursts} asks)"
        self.get_logger().info(
            f"cannon: {what}{'' if self.fire_pump else ' (DRY: fire_pump is off)'}"
            f" | {self._aim_win or 'point'}: pan {sol.pan_deg:+.1f} tilt {sol.tilt_deg:+.1f} deg "
            f"(trims {trim[0]:+.1f} {trim[1]:+.1f}), target {sol.dist_m:.2f} m "
            f"away {sol.height_m:+.2f} m up ({self._aim_from}), throw {sol.elev_deg:.1f} deg"
            + (" CLIPPED" if sol.clipped else ""))

    def _status(self):
        s, sol = self._servo, self._sol
        st = dict(
            want=self._want, firing=self._firing, bursts=self._bursts, fire_pump=self.fire_pump,
            why=self.gate.why,
            aim_cam=None if self._aim is None else [round(v, 3) for v in self._aim],
            aim_age_s=None if self._aim is None else round(time.monotonic() - self._aim_t, 2),
            window=self._aim_win,
            aim_from=self._aim_from,
            target_body=None if self._target is None else [round(v, 3) for v in self._target],
            trims=list(self.win_trim.get(self._aim_win, (0.0, 0.0))),
            pan_deg=round(sol.pan_deg, 2), tilt_deg=round(sol.tilt_deg, 2),
            pan_pwm=sol.pan_pwm, tilt_pwm=sol.tilt_pwm,
            servo_out=None if s is None or not s.output_fresh else [int(s.pan_pwm), int(s.tilt_pwm)],
            reachable=sol.reachable, clipped=sol.clipped,
            dist_m=None if math.isnan(sol.dist_m) else round(sol.dist_m, 3),
            height_m=None if math.isnan(sol.height_m) else round(sol.height_m, 3),
            elev_deg=None if math.isnan(sol.elev_deg) else round(sol.elev_deg, 1),
            pump_on=None if self._pump is None else bool(self._pump.on),
            pump_reason=None if self._pump is None else self._pump.last_reason)
        self.status_pub.publish(String(data=json.dumps(st)))


def main(args=None):
    run_node(CannonAimNode, args=args)


if __name__ == "__main__":
    main()
