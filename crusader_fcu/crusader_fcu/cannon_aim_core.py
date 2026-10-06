"""cannon_aim_core — point the pan/tilt water cannon at a window the camera sees.

Task 3 with the 2-DOF cannon (Team Bumblebee's idea, RobotX 2026): the boat
docks on the LiDAR (dock_slot_node, SlotKeep) and holds still; the CAMERA'S aim
point for the burning window (DockWindow x, y, z in camera_link, from the dock
detector) is turned into a pan and a tilt for the nozzle, every frame, while the
water runs. No ROS; stdlib only; unit-tested (test/test_cannon_aim_core.py).
cannon_aim_node is the I/O; Gazebo's water stream (crusader_sim.task3_world)
uses trajectory()/crossing() below, so the sim's water obeys the same physics
the aim assumes.

THE CHAIN, one frame:

  1. camera -> body   the aim point through the camera's mount (cam_x/y/z,
                      cam_yaw_deg, cam_pitch_deg: target_tracker's numbers, the
                      same ones bt_runner places bays with). Same rotation as
                      crusader_bt fire::camToBody.
  2. body -> nozzle   minus the pan/tilt PIVOT's position in the body frame.
                      The camera is fixed to the hull and so is the pivot, so
                      this is one constant offset (ASSUMED until measured: the
                      nozzle ~10 cm above, 30 cm left of and 10 cm ahead of the
                      camera).
  3. level it         with the autopilot's roll and pitch (NED), so gravity
                      points straight down: the stream droops in the WORLD, not
                      in the hull.
  4. ballistics       a drag-free parabola from the pivot at exit speed v, the
                      LOW arc through the target: horizontal distance d, height
                      h above the nozzle,
                          tan(el) = (v^2 - sqrt(v^4 - g (g d^2 + 2 h v^2))) / (g d)
                      v from one measurement: 3.0 m of range at 45 deg on the
                      level, v = sqrt(g R / sin 2el) = 5.42 m/s. Out of reach
                      (the root negative): the 45 deg throw, marked unreachable.
  5. back to the hull the levelled direction un-levelled, then pan = its bearing
                      about the hull's up axis, tilt = its elevation above the
                      hull's level plane - the two angles the servos turn.
  6. degrees -> PWM   ServoMap per axis: centre, us per degree, sign, clamp.
                      ASSUMED until benched: 1000-2000 us = -90..+90 deg, so
                      1100-1900 (the outputs' SERVOn_MIN/MAX) = +-72 deg.

Frames: body REP-103 (x forward, y LEFT, z up), z on the hull-bottom datum (the
datum lidar_z and cam_z are quoted against). Pan + = LEFT, tilt + = UP.
"""
import math
from dataclasses import dataclass, field

G = 9.81


def exit_speed_from_range(range_m, elev_deg):
    """The exit speed that throws `range_m` on the level at `elev_deg`
    (landing at the nozzle's own height): R = v^2 sin(2 el) / g."""
    s = math.sin(math.radians(2.0 * elev_deg))
    if range_m <= 0 or s <= 0:
        raise ValueError("need a positive range and 0 < elev < 90 deg")
    return math.sqrt(G * range_m / s)


@dataclass
class ServoMap:
    """One servo axis: degrees <-> PWM. sign +1: more PWM = more + angle."""
    center_us: float = 1500.0
    us_per_deg: float = 500.0 / 90.0
    sign: float = 1.0
    min_us: float = 1100.0
    max_us: float = 1900.0

    def deg_limits(self):
        a = self.to_deg(self.min_us)
        b = self.to_deg(self.max_us)
        return min(a, b), max(a, b)

    def to_pwm(self, deg):
        us = self.center_us + self.sign * deg * self.us_per_deg
        return int(round(min(self.max_us, max(self.min_us, us))))

    def to_deg(self, us):
        return (float(us) - self.center_us) / (self.sign * self.us_per_deg)


@dataclass
class CannonParams:
    # the pan/tilt PIVOT in the body frame (hull-bottom datum) [m]
    nozzle_x: float = 0.47
    nozzle_y: float = 0.30
    nozzle_z: float = 0.75
    # the camera's mount = target_tracker's / bt_runner_node's cam_*
    cam_x: float = 0.37
    cam_y: float = 0.0
    cam_z: float = 0.65
    cam_yaw_deg: float = 0.0           # + = aimed LEFT
    cam_pitch_deg: float = 0.0         # + = aimed DOWN
    exit_speed_mps: float = exit_speed_from_range(3.0, 45.0)
    pan: ServoMap = field(default_factory=ServoMap)
    tilt: ServoMap = field(default_factory=ServoMap)
    # software limits inside the servos' own (keep the stream off the hull)
    pan_min_deg: float = -60.0
    pan_max_deg: float = 60.0
    tilt_min_deg: float = -10.0
    tilt_max_deg: float = 60.0
    # trim, added to every solution (a calibration shot's correction)
    pan_trim_deg: float = 0.0
    tilt_trim_deg: float = 0.0

    @classmethod
    def from_dict(cls, d):
        """From cannon_aim_node's ROS params (KeyError on a missing key) - the
        cam_* keys are passed in from target_tracker's section by the node."""
        def servo(prefix):
            return ServoMap(center_us=float(d[prefix + "_center_us"]),
                            us_per_deg=float(d[prefix + "_us_per_deg"]),
                            sign=float(d[prefix + "_sign"]),
                            min_us=float(d["pwm_min"]), max_us=float(d["pwm_max"]))
        v = float(d["exit_speed_mps"])
        if v <= 0:                     # 0 = derive it from the measured throw
            v = exit_speed_from_range(float(d["throw_range_m"]), float(d["throw_elev_deg"]))
        return cls(nozzle_x=float(d["nozzle_x"]), nozzle_y=float(d["nozzle_y"]),
                   nozzle_z=float(d["nozzle_z"]),
                   cam_x=float(d["cam_x"]), cam_y=float(d["cam_y"]), cam_z=float(d["cam_z"]),
                   cam_yaw_deg=float(d["cam_yaw_deg"]), cam_pitch_deg=float(d["cam_pitch_deg"]),
                   exit_speed_mps=v, pan=servo("pan"), tilt=servo("tilt"),
                   pan_min_deg=float(d["pan_min_deg"]), pan_max_deg=float(d["pan_max_deg"]),
                   tilt_min_deg=float(d["tilt_min_deg"]), tilt_max_deg=float(d["tilt_max_deg"]),
                   pan_trim_deg=float(d["pan_trim_deg"]), tilt_trim_deg=float(d["tilt_trim_deg"]))


# ------------------------------------------------------------------ frames

def cam_to_body(p, cp: CannonParams):
    """A camera_link point -> body (absolute z on the hull datum).
    crusader_bt fire::camToBody, plus the camera's height."""
    x, y, z = p
    pr, yr = math.radians(cp.cam_pitch_deg), math.radians(cp.cam_yaw_deg)
    xl = x * math.cos(pr) + z * math.sin(pr)              # undo the pitch
    zl = -x * math.sin(pr) + z * math.cos(pr)
    return (xl * math.cos(yr) - y * math.sin(yr) + cp.cam_x,
            xl * math.sin(yr) + y * math.cos(yr) + cp.cam_y,
            zl + cp.cam_z)


def level(v, roll, pitch):
    """A body vector into the gravity-levelled frame (yaw kept), with the
    autopilot's NED roll (+ starboard down) and pitch (+ bow up), radians.
    lidar_cluster_core.level, for one vector."""
    x, y, z = v
    cr, sr, cpt, spt = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return (cpt * x - spt * sr * y - spt * cr * z,
            cr * y - sr * z,
            spt * x + cpt * sr * y + cpt * cr * z)


def unlevel(v, roll, pitch):
    """level()'s inverse (its rotation transposed)."""
    x, y, z = v
    cr, sr, cpt, spt = math.cos(roll), math.sin(roll), math.cos(pitch), math.sin(pitch)
    return (cpt * x + spt * z,
            -spt * sr * x + cr * y + cpt * sr * z,
            -spt * cr * x - sr * y + cpt * cr * z)


def direction(pan_deg, tilt_deg):
    """The nozzle's unit vector in the BODY frame for a pan and a tilt."""
    p, t = math.radians(pan_deg), math.radians(tilt_deg)
    return (math.cos(t) * math.cos(p), math.cos(t) * math.sin(p), math.sin(t))


# ------------------------------------------------------------------ ballistics

def low_arc_elevation(d, h, v):
    """(elevation deg, reachable) of the low arc from the origin through a point
    `d` away horizontally and `h` above, at exit speed v. Unreachable: 45 deg."""
    if d < 1e-6:
        return (90.0 if h >= 0 else -90.0), True
    disc = v ** 4 - G * (G * d * d + 2.0 * h * v * v)
    if disc < 0:
        return 45.0, False
    return math.degrees(math.atan((v * v - math.sqrt(disc)) / (G * d))), True


@dataclass
class AimSolution:
    ok: bool = False                   # a target was given
    reachable: bool = False            # the low arc reaches it
    clipped: bool = False              # a limit cut the angle: it will miss
    pan_deg: float = 0.0
    tilt_deg: float = 0.0
    pan_pwm: int = 0
    tilt_pwm: int = 0
    dist_m: float = float("nan")       # horizontal, pivot -> target
    height_m: float = float("nan")     # target above the pivot
    elev_deg: float = float("nan")     # the throw's elevation, WORLD level
    flight_s: float = float("nan")
    why: str = ""


def solve(target_body, cp: CannonParams, roll=0.0, pitch=0.0):
    """Pan and tilt that put the stream through `target_body` (a body-frame
    point, hull datum), with the hull at `roll`/`pitch` (NED, radians)."""
    out = AimSolution(ok=True)
    r = (target_body[0] - cp.nozzle_x, target_body[1] - cp.nozzle_y,
         target_body[2] - cp.nozzle_z)
    lx, ly, lz = level(r, roll, pitch)
    d = math.hypot(lx, ly)
    out.dist_m, out.height_m = d, lz
    el, out.reachable = low_arc_elevation(d, lz, cp.exit_speed_mps)
    out.elev_deg = el
    if not out.reachable:
        out.why = f"out of reach: {d:.2f} m away, {lz:+.2f} m up at {cp.exit_speed_mps:.2f} m/s"
    az = math.atan2(ly, lx)
    elr = math.radians(el)
    out.flight_s = d / max(1e-6, cp.exit_speed_mps * math.cos(elr))
    u = unlevel((math.cos(elr) * math.cos(az), math.cos(elr) * math.sin(az), math.sin(elr)),
                roll, pitch)
    pan = math.degrees(math.atan2(u[1], u[0])) + cp.pan_trim_deg
    tilt = math.degrees(math.atan2(u[2], math.hypot(u[0], u[1]))) + cp.tilt_trim_deg
    lo_p, hi_p = max(cp.pan_min_deg, cp.pan.deg_limits()[0]), min(cp.pan_max_deg, cp.pan.deg_limits()[1])
    lo_t, hi_t = max(cp.tilt_min_deg, cp.tilt.deg_limits()[0]), min(cp.tilt_max_deg, cp.tilt.deg_limits()[1])
    pc, tc = min(hi_p, max(lo_p, pan)), min(hi_t, max(lo_t, tilt))
    if abs(pc - pan) > 1e-6 or abs(tc - tilt) > 1e-6:
        out.clipped = True
        out.why = (out.why + "; " if out.why else "") + \
            f"limit: wanted pan {pan:+.1f} tilt {tilt:+.1f} deg"
    out.pan_deg, out.tilt_deg = pc, tc
    out.pan_pwm, out.tilt_pwm = cp.pan.to_pwm(pc), cp.tilt.to_pwm(tc)
    return out


def solve_from_camera(p_cam, cp: CannonParams, roll=0.0, pitch=0.0):
    """solve() for a camera_link aim point (the dock detector's DockWindow x, y, z)."""
    return solve(cam_to_body(p_cam, cp), cp, roll, pitch)


# ------------------------------------------------------------------ the stream

def trajectory(origin, dir_unit, v, t):
    """Where water that left `origin` along `dir_unit` (WORLD frame, z up) at
    speed v is after t seconds, drag-free."""
    return (origin[0] + dir_unit[0] * v * t,
            origin[1] + dir_unit[1] * v * t,
            origin[2] + dir_unit[2] * v * t - 0.5 * G * t * t)


def crossing(origin, dir_unit, v, plane_point, plane_normal, t_max=3.0):
    """(t, point) where the stream first crosses the plane n.(p - q) = 0 going
    THROUGH it from the side it started on, or None. WORLD frame, z up."""
    n, q = plane_normal, plane_point
    a = -0.5 * G * n[2]
    b = v * (n[0] * dir_unit[0] + n[1] * dir_unit[1] + n[2] * dir_unit[2])
    c = n[0] * (origin[0] - q[0]) + n[1] * (origin[1] - q[1]) + n[2] * (origin[2] - q[2])
    if abs(a) < 1e-12:
        if abs(b) < 1e-12:
            return None
        roots = [-c / b]
    else:
        disc = b * b - 4 * a * c
        if disc < 0:
            return None
        s = math.sqrt(disc)
        roots = sorted(((-b - s) / (2 * a), (-b + s) / (2 * a)))
    for t in roots:
        if 1e-6 < t <= t_max:
            return t, trajectory(origin, dir_unit, v, t)
    return None


# ------------------------------------------------------------------ firing

@dataclass
class FireParams:
    settle_s: float = 0.4          # servos at the command and the target still, this long
    servo_tol_us: float = 12.0     # SERVO_OUTPUT_RAW within this of the command
    aim_max_age_s: float = 1.0     # an aim point older than this is not a target
    burst_s: float = 1.0           # one pump burst (the bridge caps it: pump_max_burst_s)
    gap_s: float = 1.1             # from a burst's start + burst_s to the next ask (> pump_min_gap_s)


class FireGate:
    """When to ask the bridge for water: the tree wants it (fire true), the aim
    point is fresh, the solution reaches and is not clipped, and the servos have
    been AT the command for settle_s. Then a burst every burst_s + gap_s while
    that holds. Pure: the node feeds it and sends what it returns."""

    def __init__(self, p: FireParams = None):
        self.p = p or FireParams()
        self.reset()

    def reset(self):
        self.ok_since = None
        self.next_burst_t = -1e18
        self.why = "idle"

    def update(self, now, want, aim_age_s, sol: AimSolution, servo_err_us):
        """Returns the burst length to ask for now (s), or 0.0 for nothing."""
        p = self.p
        if not want:
            why = "not asked"
        elif aim_age_s > p.aim_max_age_s:
            why = f"aim point {aim_age_s:.1f} s old"
        elif not sol.ok or not sol.reachable:
            why = sol.why or "no solution"
        elif sol.clipped:
            why = sol.why
        elif servo_err_us is None:
            why = "servo position unknown (no SERVO_OUTPUT_RAW)"
        elif servo_err_us > p.servo_tol_us:
            why = f"servos moving ({servo_err_us:.0f} us off)"
        else:
            why = ""
        if why:
            self.ok_since = None
            self.why = why
            return 0.0
        if self.ok_since is None:
            self.ok_since = now
        if now - self.ok_since < p.settle_s:
            self.why = "settling"
            return 0.0
        if now < self.next_burst_t:
            self.why = "between bursts"
            return 0.0
        self.next_burst_t = now + p.burst_s + p.gap_s
        self.why = "FIRE"
        return p.burst_s


# ------------------------------------------------------------------ the bridge's side

class ServoLimiter:
    """telemetry_bridge's rule for /crsd/cannon_cmd, per axis: clamp to
    [pwm_min, pwm_max], send only on a change of at least min_step_us, and not
    more often than min_period_s. Returns the PWM to send now, or None."""

    def __init__(self, pwm_min, pwm_max, min_period_s=0.05, min_step_us=2):
        self.lo, self.hi = int(pwm_min), int(pwm_max)
        self.min_period_s = float(min_period_s)
        self.min_step_us = int(min_step_us)
        self.last_us = None
        self.last_t = -1e18
        self.pending = None

    def request(self, pwm, now):
        if not pwm:
            return None                       # 0 = leave this axis alone
        us = int(min(self.hi, max(self.lo, int(pwm))))
        self.pending = us
        return self.due(now)

    def due(self, now):
        """The pending PWM if it may go out now (also called from the tick)."""
        us = self.pending
        if us is None:
            return None
        if self.last_us is not None and abs(us - self.last_us) < self.min_step_us:
            self.pending = None
            return None
        if now - self.last_t < self.min_period_s:
            return None
        self.last_us, self.last_t, self.pending = us, now, None
        return us
