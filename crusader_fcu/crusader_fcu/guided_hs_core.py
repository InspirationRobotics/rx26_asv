"""guided_hs_core — the rules for GUIDED heading+speed, with no ROS and no MAVLink.

The fixed-nozzle shot has to stand a set distance off a wall and point at a
window, which needs astern as well as ahead and a chosen heading. Position
setpoints can do neither on this boat, so telemetry_bridge grows one narrow
extra TX path: /crsd/guided_heading_speed -> MAVLink SET_ATTITUDE_TARGET.

ARDUROVER, AS REMEMBERED (verify in SITL before trusting; docs/G1 records it):
  * SET_ATTITUDE_TARGET is acted on only in GUIDED, and only if the
    THROTTLE_IGNORE bit (64) is CLEAR;
  * with BODY_YAW_RATE_IGNORE (4) set, the heading comes from the quaternion,
    and speed = thrust * the mode's default speed (WP_SPEED), thrust clamped
    to [-1, 1] -- negative = astern;
  * after 3 s with no new target it stops by itself.

WHAT THIS ADDS ON TOP, because 3 s at 0.3 m/s is 0.9 m next to a dock:
  * TWO interlocks, both independent of us: the flight MODE (the pilot's SC
    switch; ArduPilot also ignores this message outside GUIDED) and the
    autonomy-drop LATCH (the pilot's SE switch on ch9);
  * a speed clamp here, whatever the tree asks for;
  * a dead-man: commands stop for deadman_s -> ONE zero-speed command, sent by
    the bridge itself. So does a latch trip.

`now` is monotonic seconds from the caller; tests drive it.
"""
import math
from dataclasses import dataclass

TYPE_MASK = 1 | 2 | 4          # ignore body roll/pitch/yaw RATES: use the attitude + thrust
THROTTLE_IGNORE = 64


@dataclass
class HsParams:
    max_speed: float           # m/s, either direction
    wp_speed: float            # ArduRover's WP_SPEED: thrust 1.0 means this
    deadman_s: float

    @classmethod
    def from_dict(cls, d):
        """From the bridge's resolved ROS params. KeyError on a missing key."""
        return cls(max_speed=float(d["hs_max_speed_mps"]),
                   wp_speed=float(d["hs_wp_speed_mps"]),
                   deadman_s=float(d["hs_deadman_s"]))


def encode(heading_deg, speed_mps, wp_speed):
    """(type_mask, q[w,x,y,z], thrust) for SET_ATTITUDE_TARGET.

    q is a pure yaw about NED z (down), so yaw = the compass heading in
    radians: q = (cos(psi/2), 0, 0, sin(psi/2)). thrust = speed / WP_SPEED,
    clamped to [-1, 1]."""
    psi = math.radians(float(heading_deg))
    q = [math.cos(psi / 2.0), 0.0, 0.0, math.sin(psi / 2.0)]
    thrust = max(-1.0, min(1.0, float(speed_mps) / float(wp_speed)))
    return TYPE_MASK, q, thrust


class HsGate:
    """The path's state: whether we are driving, the last heading, the dead-man."""

    def __init__(self, p: HsParams):
        self.p = p
        self.active = False            # we have commanded motion and not stopped it
        self.last_cmd_t = None
        self.last_heading = None

    def on_command(self, now, heading_deg, speed_mps, mode, latch_allowed):
        """(ok, reason, fields). fields = (type_mask, q, thrust) to send, or None."""
        if heading_deg is None or not math.isfinite(heading_deg):
            return False, "no heading", None
        if speed_mps is None or not math.isfinite(speed_mps):
            return False, "no speed", None
        if str(mode).upper() != "GUIDED":
            self.active = False        # the pilot has it; nothing of ours is live
            return False, f"mode {mode} is not GUIDED", None
        if not latch_allowed:
            self.active = False
            return False, "autonomy-drop latch tripped", None
        speed = max(-self.p.max_speed, min(self.p.max_speed, float(speed_mps)))
        self.active = True
        self.last_cmd_t = now
        self.last_heading = float(heading_deg) % 360.0
        reason = "ok" if speed == speed_mps else f"speed clamped to {speed:+.2f}"
        return True, reason, encode(self.last_heading, speed, self.p.wp_speed)

    def tick(self, now):
        """fields for ONE stop command when the dead-man expires, else None."""
        if self.active and now - self.last_cmd_t > self.p.deadman_s:
            self.active = False
            return encode(self.last_heading, 0.0, self.p.wp_speed)
        return None

    def on_trip(self):
        """The drop latch tripped: a stop if we were driving, else None."""
        if not self.active:
            return None
        self.active = False
        return encode(self.last_heading, 0.0, self.p.wp_speed)
