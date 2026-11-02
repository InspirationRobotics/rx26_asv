"""cannon_core: aim the pan/tilt water cannon at a point the camera sees. No ROS.

The Task 3 tree (SprayUntilHit) sends the burning window's aim point in camera_link on
/crsd/water_cannon; telemetry_bridge turns it into the two servos' PWM with solve() and asks
for water through pump_core once FireGate says the servos are on it.

One frame: camera -> body (the camera's mount) -> minus the nozzle's pivot -> levelled with
roll/pitch so the stream droops in the world -> the LOW drag-free arc through the target at
the measured exit speed -> back into the hull -> pan = bearing, tilt = elevation -> PWM.
Body frame REP-103 (x forward, y LEFT, z up) on the hull-bottom datum. Pan + = LEFT, tilt + = UP.
"""
import math
from dataclasses import dataclass, field

G = 9.81


def exit_speed_from_range(range_m, elev_deg, drop_m=0.0):
    """Exit speed of a drag-free stream that leaves at elev_deg and lands range_m out and
    drop_m BELOW the nozzle (9 ft at 30 deg onto the water, 0.51 m down: 4.85 m/s)."""
    t = math.radians(elev_deg)
    den = 2.0 * math.cos(t) ** 2 * (range_m * math.tan(t) + drop_m)
    if range_m <= 0 or not 0.0 < elev_deg < 90.0 or den <= 0:
        raise ValueError("need a positive range, 0 < elev < 90 deg, and a reachable landing")
    return math.sqrt(G * range_m * range_m / den)


@dataclass
class ServoMap:
    """One servo axis, degrees <-> PWM. sign +1: more PWM = more + angle."""
    center_us: float = 1500.0
    us_per_deg: float = 500.0 / 90.0
    sign: float = 1.0
    min_us: float = 1100.0
    max_us: float = 1900.0

    def to_pwm(self, deg):
        us = self.center_us + self.sign * deg * self.us_per_deg
        return int(round(min(self.max_us, max(self.min_us, us))))

    def deg_limits(self):
        a, b = ((us - self.center_us) / (self.sign * self.us_per_deg)
                for us in (self.min_us, self.max_us))
        return min(a, b), max(a, b)


@dataclass
class CannonParams:
    nozzle: tuple = (0.37, 0.279, 0.75)      # the pan/tilt pivot, body frame [m]
    cam: tuple = (0.37, 0.0, 0.65)           # the camera's mount, = bt_runner_node cam_*
    cam_yaw_deg: float = 0.0                 # + = aimed LEFT
    cam_pitch_deg: float = 0.0               # + = aimed DOWN
    exit_speed_mps: float = 4.85
    pan: ServoMap = field(default_factory=ServoMap)
    tilt: ServoMap = field(default_factory=ServoMap)
    pan_lim: tuple = (-60.0, 60.0)           # software limits inside the servos' own [deg]
    tilt_lim: tuple = (-10.0, 60.0)

    @classmethod
    def from_dict(cls, d):
        """From telemetry_bridge's cannon_* params."""
        def servo(axis):
            return ServoMap(float(d[f"cannon_{axis}_center_us"]), float(d[f"cannon_{axis}_us_per_deg"]),
                            float(d[f"cannon_{axis}_sign"]), float(d["cannon_pwm_min"]),
                            float(d["cannon_pwm_max"]))
        v = float(d["cannon_exit_speed_mps"])
        if v <= 0:                               # 0 = from the measured throw
            v = exit_speed_from_range(float(d["cannon_throw_range_m"]),
                                      float(d["cannon_throw_elev_deg"]),
                                      float(d["cannon_throw_drop_m"]))
        return cls(nozzle=tuple(float(x) for x in d["cannon_nozzle_xyz"]),
                   cam=tuple(float(x) for x in d["cannon_cam_xyz"]),
                   cam_yaw_deg=float(d["cannon_cam_yaw_deg"]),
                   cam_pitch_deg=float(d["cannon_cam_pitch_deg"]),
                   exit_speed_mps=v, pan=servo("pan"), tilt=servo("tilt"),
                   pan_lim=tuple(float(x) for x in d["cannon_pan_limits_deg"]),
                   tilt_lim=tuple(float(x) for x in d["cannon_tilt_limits_deg"]))


def cam_to_body(p, cp):
    """A camera_link point -> body frame (crusader_bt fire::camToBody, plus the camera's height)."""
    x, y, z = p
    pr, yr = math.radians(cp.cam_pitch_deg), math.radians(cp.cam_yaw_deg)
    xl, zl = x * math.cos(pr) + z * math.sin(pr), -x * math.sin(pr) + z * math.cos(pr)
    return (xl * math.cos(yr) - y * math.sin(yr) + cp.cam[0],
            xl * math.sin(yr) + y * math.cos(yr) + cp.cam[1], zl + cp.cam[2])


def level(v, roll, pitch):
    """A body vector into the gravity-levelled frame (yaw kept); NED roll/pitch, radians."""
    x, y, z = v
    cr, sr, cp_, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return (cp_ * x - sp * sr * y - sp * cr * z, cr * y - sr * z, sp * x + cp_ * sr * y + cp_ * cr * z)


def unlevel(v, roll, pitch):
    """level()'s inverse."""
    x, y, z = v
    cr, sr, cp_, sp = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return (cp_ * x + sp * z, -sp * sr * x + cr * y + cp_ * sr * z, -sp * cr * x - sr * y + cp_ * cr * z)


def low_arc_elevation(d, h, v):
    """(elevation deg, reachable) of the low arc through a point d out and h up. Unreachable: 45."""
    if d < 1e-6:
        return (90.0 if h >= 0 else -90.0), True
    disc = v ** 4 - G * (G * d * d + 2.0 * h * v * v)
    if disc < 0:
        return 45.0, False
    return math.degrees(math.atan((v * v - math.sqrt(disc)) / (G * d))), True


@dataclass
class Aim:
    reachable: bool = False
    clipped: bool = False                    # a limit cut the angle: it will miss
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    pan_pwm: int = 0
    tilt_pwm: int = 0
    dist_m: float = float("nan")             # horizontal, pivot -> target
    why: str = ""


def solve(p_cam, cp, roll=0.0, pitch=0.0):
    """Pan and tilt that put the stream through a camera_link point, the hull at roll/pitch."""
    b = cam_to_body(p_cam, cp)
    lx, ly, lz = level((b[0] - cp.nozzle[0], b[1] - cp.nozzle[1], b[2] - cp.nozzle[2]), roll, pitch)
    out = Aim(dist_m=math.hypot(lx, ly))
    el, out.reachable = low_arc_elevation(out.dist_m, lz, cp.exit_speed_mps)
    if not out.reachable:
        out.why = f"out of reach: {out.dist_m:.2f} m out, {lz:+.2f} m up"
    az, elr = math.atan2(ly, lx), math.radians(el)
    u = unlevel((math.cos(elr) * math.cos(az), math.cos(elr) * math.sin(az), math.sin(elr)), roll, pitch)
    pan = math.degrees(math.atan2(u[1], u[0]))
    tilt = math.degrees(math.atan2(u[2], math.hypot(u[0], u[1])))
    (plo, phi), (tlo, thi) = cp.pan.deg_limits(), cp.tilt.deg_limits()
    pc = min(min(phi, cp.pan_lim[1]), max(max(plo, cp.pan_lim[0]), pan))
    tc = min(min(thi, cp.tilt_lim[1]), max(max(tlo, cp.tilt_lim[0]), tilt))
    if abs(pc - pan) > 1e-6 or abs(tc - tilt) > 1e-6:
        out.clipped = True
        out.why = f"limit: wanted pan {pan:+.1f} tilt {tilt:+.1f} deg"
    out.pan_deg, out.tilt_deg = pc, tc
    out.pan_pwm, out.tilt_pwm = cp.pan.to_pwm(pc), cp.tilt.to_pwm(tc)
    return out


class FireGate:
    """When to ask for water: asked, a fresh reachable unclipped aim, and the servos AT the
    command for settle_s. Then it stays on: another burst_s ask every resend_s (each extends
    the last in pump_core) while the servos keep tracking, until a hard reason."""

    def __init__(self, settle_s=0.4, servo_tol_us=12.0, aim_max_age_s=3.0, burst_s=1.0, resend_s=0.3):
        if not 0.05 <= resend_s < burst_s:
            raise ValueError("cannon resend_s must be under burst_s: the water would stop between asks")
        self.settle_s, self.servo_tol_us, self.aim_max_age_s = settle_s, servo_tol_us, aim_max_age_s
        self.burst_s, self.resend_s = burst_s, resend_s
        self.reset()

    def reset(self):
        self.ok_since, self.next_t, self.firing, self.why = None, -1e18, False, "idle"

    def update(self, now, aim_age_s, aim, servo_err_us):
        """The burst to ask for now [s], or 0.0."""
        hard = ("aim point %.1f s old" % aim_age_s if aim_age_s > self.aim_max_age_s else
                aim.why if not aim.reachable or aim.clipped else
                "servo position unknown (SERVO_OUTPUT_RAW)" if servo_err_us is None else "")
        if hard:
            self.ok_since, self.firing, self.why = None, False, hard
            return 0.0
        if not self.firing:
            if servo_err_us > self.servo_tol_us:
                self.ok_since, self.why = None, f"servos moving ({servo_err_us:.0f} us off)"
                return 0.0
            self.ok_since = now if self.ok_since is None else self.ok_since
            if now - self.ok_since < self.settle_s:
                self.why = "settling"
                return 0.0
            self.firing = True
        elif now < self.next_t:
            self.why = "spraying"
            return 0.0
        self.next_t, self.why = now + self.resend_s, "FIRE"
        return self.burst_s
