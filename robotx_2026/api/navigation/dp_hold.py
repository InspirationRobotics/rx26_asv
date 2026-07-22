"""dp_hold — dynamic-position hold onto a visual target (Mission 3 docking).

REWIRED port. The operational dp_hold was monolithic: it opened its own MAVLink
connection (a second Pixhawk consumer), ran its own OAK-D + YOLO + depth pipeline,
served its own MJPEG stream, and sent RC overrides straight to the autopilot with
no autonomy-drop integration. This version keeps the SAME control law but plugs
into this repo's architecture:

  in:  /crsd/detections_body (DetectionArray, frame="body") — from perception_node
       /crsd/pose            (LatLonHead)   — heading, from telemetry_bridge
       /crsd/fcu_status      (FcuStatus)    — flight mode
  out: /crsd/rc_override     via OverrideGuard (the ONLY sanctioned override path,
                             latch-gated) — steer/throttle/lateral on ch 1/3/4
       /crsd/autonomy_active (Bool)         — GREEN-LED heartbeat while holding

Control law (unchanged): yaw PD (KP_YAW·err − KD_YAW·yaw_rate) + forward P + lateral
P, all clamped to ±PWM_LIMIT about the neutral trim. Why MANUAL RC override and not
GUIDED: ArduRover GUIDED cannot strafe on the OmniX frame — RC override in MANUAL is
the only true lateral-hold path (CLAUDE.md).

FIDELITY NOTE: the original took yaw_rate from ATTITUDE.yawspeed on its own link;
telemetry_bridge does not republish ATTITUDE, so here yaw_rate is derived from
/crsd/pose heading deltas (noisier — revisit if the damper needs the cleaner source).

SAFETY: RC-override, so blocked from beyond-WiFi field testing until the
autonomy-drop switch is signed off (G1). OverrideGuard already forces this node to
stop computing overrides within one cycle of a drop.
"""
import math
import time

from rclpy.node import Node
from std_msgs.msg import Bool

from interfaces.msg import DetectionArray, LatLonHead, FcuStatus

from ..common import config as crsd_config
from ..common.node_main import run_node
from ..common.override_guard import OverrideGuard
from ..common.param_utils import declare_from_config

# Empirical neutral trim from the operational tune (not exactly 1500).
STEER_NEUTRAL = 1489
THROTTLE_NEUTRAL = 1495
LATERAL_NEUTRAL = 1495
STEER_CH, THROTTLE_CH, LATERAL_CH = 1, 3, 4     # RC channels (1-indexed)

PARAM_SPEC = {
    "target_classes": dict(read_only=True,
                           description="detection labels to lock onto"),
    "target_yaw_deg": dict(read_only=True, lo=0.0, hi=360.0,
                           description="hold heading [deg]"),
    "target_forward_m": dict(read_only=True, lo=0.3, hi=20.0,
                             description="hold target this far ahead [m]"),
    "target_lateral_m": dict(read_only=True, lo=-10.0, hi=10.0,
                             description="target lateral offset (0 = centered)"),
    "conf_min": dict(read_only=True, lo=0.05, hi=0.95),
    "lost_timeout_s": dict(read_only=True, lo=0.1, hi=10.0,
                           description="s without target -> hold yaw only"),
    "grace_s": dict(read_only=True, lo=1.0, hi=120.0,
                    description="s after last sight before disengaging"),
    "kp_yaw": dict(read_only=True, lo=0.0, hi=50.0),
    "kd_yaw": dict(read_only=True, lo=0.0, hi=50.0),
    "kp_fwd": dict(read_only=True, lo=0.0, hi=500.0),
    "kp_lat": dict(read_only=True, lo=0.0, hi=500.0),
    "pwm_limit": dict(read_only=True, lo=10, hi=400,
                      description="max PWM deflection from neutral"),
    "dead_yaw_deg": dict(read_only=True, lo=0.0, hi=45.0),
    "dead_pos_m": dict(read_only=True, lo=0.0, hi=2.0),
    "rate_hz": dict(read_only=True, lo=1.0, hi=50.0),
}


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


class DPHold(Node):
    def __init__(self):
        super().__init__("dp_hold")
        p = declare_from_config(self, crsd_config.node_params("dp_hold"),
                                PARAM_SPEC)
        self.p = p
        self.target_classes = set(p["target_classes"])
        self.pwm_limit = p["pwm_limit"]

        self.guard = OverrideGuard(self)
        self.auto_pub = self.create_publisher(Bool, "/crsd/autonomy_active", 10)
        self.create_subscription(DetectionArray, "/crsd/detections_body",
                                 self._det_cb, 10)
        self.create_subscription(LatLonHead, "/crsd/pose", self._pose_cb, 10)
        self.create_subscription(FcuStatus, "/crsd/fcu_status", self._fcu_cb, 10)

        self.heading = None
        self.mode = ""
        self.yaw_rate = 0.0
        self._last_hdg = None
        self._last_hdg_t = None
        self.target = None                 # (t, forward_m, lateral_m)
        self.engaged = False
        self.last_seen = 0.0

        self.create_timer(1.0 / p["rate_hz"], self._tick)
        self.get_logger().info(
            f"dp_hold up: hold yaw {p['target_yaw_deg']:.0f}deg, "
            f"target {p['target_forward_m']:.1f}m ahead — MANUAL RC override")

    # ---------- inputs ----------

    def _pose_cb(self, msg: LatLonHead):
        h = msg.heading
        if math.isnan(h):
            return
        now = time.monotonic()
        if self._last_hdg is not None and self._last_hdg_t is not None:
            dt = now - self._last_hdg_t
            if dt > 1e-3:
                rate = wrap180(h - self._last_hdg) / dt
                # light smoothing — derived rate is noisier than ATTITUDE.yawspeed
                self.yaw_rate = 0.7 * self.yaw_rate + 0.3 * rate
        self._last_hdg, self._last_hdg_t = h, now
        self.heading = h

    def _fcu_cb(self, msg: FcuStatus):
        self.mode = msg.mode

    def _det_cb(self, msg: DetectionArray):
        if msg.frame != "body":
            return
        best = None                        # nearest target-class detection
        for d in msg.detections:
            if d.label not in self.target_classes or d.confidence < self.p["conf_min"]:
                continue
            forward = d.y                  # BODY: y = forward+, x = starboard+
            lateral = d.x
            if forward <= 0.3:
                continue
            if best is None or forward < best[0]:
                best = (forward, lateral)
        if best is not None:
            self.target = (time.monotonic(), best[0], best[1])

    # ---------- control ----------

    def _clamp(self, v):
        return int(max(-self.pwm_limit, min(self.pwm_limit, v)))

    def _tick(self):
        now = time.monotonic()
        fresh = (self.target is not None
                 and now - self.target[0] < self.p["lost_timeout_s"])
        if fresh:
            self.engaged = True
            self.last_seen = now

        if (self.mode != "MANUAL" or self.heading is None or not self.engaged
                or now - self.last_seen > self.p["grace_s"]):
            self.engaged = False
            self._release()
            self.auto_pub.publish(Bool(data=False))
            return
        if not self.guard.allowed:         # autonomy dropped: stop overriding NOW
            self.engaged = False
            self.auto_pub.publish(Bool(data=False))
            return

        self.auto_pub.publish(Bool(data=True))

        yaw_err = wrap180(self.p["target_yaw_deg"] - self.heading)
        if abs(yaw_err) < self.p["dead_yaw_deg"]:
            yaw_err = 0.0
        steer = self._clamp(self.p["kp_yaw"] * yaw_err
                            - self.p["kd_yaw"] * self.yaw_rate)

        if fresh:
            _, forward, lateral = self.target
            fwd_err = forward - self.p["target_forward_m"]
            lat_err = lateral - self.p["target_lateral_m"]
            if abs(fwd_err) < self.p["dead_pos_m"]:
                fwd_err = 0.0
            if abs(lat_err) < self.p["dead_pos_m"]:
                lat_err = 0.0
            self._override(steer, self._clamp(self.p["kp_fwd"] * fwd_err),
                           self._clamp(self.p["kp_lat"] * lat_err))
        else:
            self._override(steer, 0, 0)    # lost sight: hold yaw, wait
        self.get_logger().info(
            f"yaw_err {yaw_err:+.0f} lock {'Y' if fresh else 'lost'}",
            throttle_duration_sec=1.0)

    def _override(self, steer, throttle, lateral):
        ch = [0] * 18
        ch[STEER_CH - 1] = STEER_NEUTRAL + steer
        ch[THROTTLE_CH - 1] = THROTTLE_NEUTRAL + throttle
        ch[LATERAL_CH - 1] = LATERAL_NEUTRAL + lateral
        self.guard.publish_override(ch)

    def _release(self):
        # publish all-zero: telemetry_bridge releases every channel to the pilot
        self.guard.publish_override([0] * 8)


def main(args=None):
    run_node(DPHold, args=args)


if __name__ == "__main__":
    main()
