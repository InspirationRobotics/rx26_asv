"""guided_hs_core: the rules for GUIDED commands from the tree. No ROS.

Heading + signed speed (SET_ATTITUDE_TARGET) goes out only in GUIDED and while the drop
latch allows it, speed clamped, with one zero-speed command when commands stop for deadman_s
or the latch trips: ArduRover itself waits 3 s, 0.9 m at 0.3 m/s next to a dock. ArduRover's
handling is unverified until SITL (docs/G1).
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


def setpoint_error(lat, lon):
    """Why a GUIDED position setpoint must not be sent, or '' when it is a real position."""
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return "non-finite latitude/longitude"
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return "latitude/longitude out of range"
    if lat == 0.0 and lon == 0.0:
        return "(0, 0) is an unset message, not a place"
    return ""


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
