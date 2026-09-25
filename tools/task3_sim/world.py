"""world.py — the Task 3 course, the boat, the camera and RoboCommand, in plain Python.

No ROS, no numpy, nothing to install. sim.py steps this in real time and trades
JSON lines with the off-ROS runner (crusader_bt/offros), which runs the real
behaviour tree. test_world.py exercises it on its own.

WHAT IS MODELLED, and how honestly:

  the course     the RobotX 2026 build guide ("Docking Bay Structure"):
                 0.5 m dock cubes, three 1.5 m slips between 0.5 m fingers
                 2.0 m long, a 1 m deep main deck, and on it a 1 m square face
                 per bay with the two windows and the indicator where the
                 front-panel drawing puts them. Numbered left to right FACING
                 them (handbook 3.3.4); one GREEN indicator. The deck's height
                 above the water (0.3 m) is ASSUMED.
  the boat       ~1.0 x 0.6 m, driven like ArduRover GUIDED on this hull:
                 position setpoints only, it TURNS rather than strafes,
                 WP_SPEED 1.0, ATC_ACCEL_MAX 1.0, and on arrival it loiters
                 with LOIT_RADIUS 2.0 / LOIT_TYPE 0 - drift under 2 m is not
                 corrected, and over it the boat drives forward OR in reverse
                 back to the point. params/working_crusader.params.
  the camera     an OAK-D LR at the bow (cam_x 0.37, 0.41 m above water -
                 Resources.md), fx 1173 px at 1920x1200, with the CV config's
                 188-row hull band masked off the top, and a PITCH. It reports
                 what the dock detector's DRAFT DockObservation carries, field
                 for field, in the tilted camera frame, with the CV report's
                 error rates: faces to ~9 m, indicators weakening past 6 m
                 (92/125 at 6-10 m), colours that abstain but are never wrong
                 (unless you turn on `miscolour`). A window it cannot see whole
                 is not reported. target_pattern comes from the CV team's own
                 timing stage (vendor/dock_sequence_core.py), on these frames.
  RoboCommand    lights the fire when the boat reports docking IN the green
                 bay, and only then (strict judge); checks every report.
  the lights     handbook: RED until hit, GREEN 5 s, off 1 s, then the tier's
                 signal for 60 s. The cannon puts the fire out after
                 `extinguish_s` of spray on the right window.

WHAT IS NOT: hydrodynamics, wind, waves, the fingers stopping the hull (contact
is recorded, not simulated), the water stream's flight, any image processing.

COORDINATES. World is ENU metres about Scenario.origin: e EAST, n NORTH.
Headings are compass degrees. The camera frame is REP-103 (x forward, y left,
z up) - so a bearing in it is POSITIVE TO PORT.

THE PROJECTION matches crusader_bt's nav_math (spherical, R = 6371 km), not
crusader_common.geo's M_PER_DEG. This file stands in for the GPS that feeds a
C++ consumer using nav_math, and matching it makes "the boat is 1 cm off" mean
the same thing on both sides.
"""
import math
import random
from dataclasses import dataclass, field, asdict

from vendor.dock_sequence_core import DockSequence

EARTH_R = 6371000.0
DEG = math.pi / 180.0

# CV numbering: DockBay.COLOUR_* and DockWindow.STATE_* (the draft messages).
UNKNOWN, OFF, RED, GREEN, BLUE = 0, 1, 2, 3, 4
NAME = {UNKNOWN: "unknown", OFF: "off", RED: "red", GREEN: "green", BLUE: "blue"}
BY_NAME = {v: k for k, v in NAME.items()}
LIT = (RED, GREEN, BLUE)
# RoboCommand's Color: no OFF, so RED is 1. The judge reads reports in this.
WIRE = {"COLOR_RED": RED, "COLOR_GREEN": GREEN, "COLOR_BLUE": BLUE}


# ------------------------------------------------------------------ geometry

def to_local(lat, lon, origin):
    """lat/lon -> (east, north) m. nav_math::toLocal, exactly."""
    return ((lon - origin[1]) * DEG * EARTH_R * math.cos(origin[0] * DEG),
            (lat - origin[0]) * DEG * EARTH_R)


def to_latlon(e, n, origin):
    """(east, north) m -> lat/lon. nav_math::toLatLon, exactly."""
    return (origin[0] + (n / EARTH_R) / DEG,
            origin[1] + (e / (EARTH_R * math.cos(origin[0] * DEG))) / DEG)


def hvec(h_deg):
    """Unit vector for a compass heading."""
    return (math.sin(h_deg * DEG), math.cos(h_deg * DEG))


def port(v):
    return (-v[1], v[0])


def starboard(v):
    return (v[1], -v[0])


def add(a, b):
    return (a[0] + b[0], a[1] + b[1])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1])


def mul(a, s):
    return (a[0] * s, a[1] * s)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def norm(a):
    return math.hypot(a[0], a[1])


def wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def bearing_deg(frm, to):
    """Compass bearing from one point to another."""
    return math.degrees(math.atan2(to[0] - frm[0], to[1] - frm[1])) % 360.0


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


def seg_intersect(p1, p2, q1, q2):
    """Do segments p1-p2 and q1-q2 cross?"""
    def orient(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])
    d1, d2 = orient(q1, q2, p1), orient(q1, q2, p2)
    d3, d4 = orient(p1, p2, q1), orient(p1, p2, q2)
    return (d1 * d2 < 0) and (d3 * d4 < 0)


# ------------------------------------------------------------------ scenario

@dataclass
class Scenario:
    """Every knob, in one place. The page edits this; tests build their own."""
    seed: int = 1
    origin: tuple = (27.7745, -82.6320)        # St Petersburg, FL
    # the boat
    start_e: float = 0.0
    start_n: float = 0.0
    start_heading: float = 0.0
    # the dock: centre of the back wall, and the compass direction its faces
    # point OUT (180 = the bays open to the south)
    dock_e: float = 0.0
    dock_n: float = 20.0
    facing_deg: float = 180.0
    # the dock, from the build guide: 0.5 m cubes; slips 3 cubes wide between
    # fingers 1 cube wide and 4 long; a main deck 2 cubes deep behind the faces
    slip_width: float = 1.5
    finger_width: float = 0.5
    finger_len: float = 2.0
    deck_depth: float = 1.0
    deck_z: float = 0.3                        # ASSUMED: dock-cube freeboard
    green_bay: int = 2                         # 1..3, left to right facing them
    # the goal's approach point, metres out in front of the dock centre. The
    # operator's rough "the dock is over there", on the water side and clear
    # of the fingers.
    approach_m: float = 7.0
    # RoboCommand
    tier: int = 2                              # 0 Core, 1 Advanced, 2 Disruptive
    target_window: int = 1                     # DockWindow.index that lights
    code: tuple = ("red", "blue")              # (resource tin, delivery circle)
    strict_judge: bool = True                  # light only if really docked there
    lose_docking_reports: int = 0              # RoboCommand never hears the first N
    activation_delay_s: float = 1.5
    extinguish_s: float = 2.0                  # spray-on-target to put it out
    # sensing
    camera_ok: bool = True
    # + = aimed DOWN (cam_pitch_deg). THE BOAT'S CAMERA IS LEVEL TODAY (0), and
    # level it sees NEITHER window whole from a berth inside the slip; -25 is
    # what that takes with the hull band. The default shows the tree working
    # with the mount it needs; test_e2e level_camera pins today's failing.
    cam_pitch_deg: float = -25.0
    hull_band_rows: int = 188                  # CV config occluded_top_rows
    miscolour: float = 0.0                     # P(a lit colour read as another)
    unknown_rate: float = 0.03                 # P(the colour rule abstains)
    gps_sd_m: float = 0.02                     # RTK
    heading_sd_deg: float = 0.3
    # the autopilot
    wp_radius: float = 0.2                     # the BOAT's WP_RADIUS is 2.0 - see Boat
    loit_radius: float = 2.0                   # LOIT_RADIUS
    # disturbance
    current_mps: float = 0.0
    current_to_deg: float = 90.0

    def randomise(self, rnd):
        """A fresh course: bay, window, code. The geometry is left alone."""
        self.green_bay = rnd.randint(1, 3)
        self.target_window = rnd.randint(0, 1)
        cols = ["red", "green", "blue"]
        self.code = (rnd.choice(cols), rnd.choice(cols))


# ------------------------------------------------------------------ the dock

# The structure on each bay, from the build guide's front-panel drawing (mm):
# a 1000 x 1000 panel standing on the deck; the upper-left window opening
# 210 x 290 with its bottom 605 up and its left 175 in; the lower-right one
# ~230 x 310, bottom 355 up, left ~615 in; the indicator cut-out 160 mm square
# at the bottom centre. As (slot, right, up, half-width, half-height) from the
# face centre, m. Index 0 is the LEFT slot, as DockWindow.index defines it.
FACE_W = 1.0
FACE_H = 1.0
WINDOW_SLOTS = (("UL", -0.220, +0.250, 0.105, 0.145),
                ("LR", +0.230, +0.010, 0.115, 0.155))
INDICATOR_UP = -0.420
WINDOW_M = 0.25                # the handbook's nominal size, for range-from-size


def convex_overlap(a, b):
    """Do two convex polygons overlap? (Separating axis test.)"""
    for poly in (a, b):
        for i in range(len(poly)):
            p1, p2 = poly[i], poly[(i + 1) % len(poly)]
            axis = (p1[1] - p2[1], p2[0] - p1[0])
            pa = [dot(axis, q) for q in a]
            pb = [dot(axis, q) for q in b]
            if max(pa) < min(pb) or max(pb) < min(pa):
                return False
    return True


class Dock:
    """Three slips between four solid fingers, a deck behind, faces on it."""

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.centre = (sc.dock_e, sc.dock_n)      # the deck edge, mid-dock
        self.out = hvec(sc.facing_deg)
        # LEFT -> RIGHT for someone facing the bays, i.e. looking along -out.
        self.right = starboard(mul(self.out, -1.0))
        self.pitch = sc.slip_width + sc.finger_width
        self.face_z = sc.deck_z + FACE_H / 2.0
        self.faces = {i: self.at((i - 2) * self.pitch, 0.0) for i in (1, 2, 3)}
        fw = sc.finger_width
        self.finger_rects = [((k - 1.5) * self.pitch - fw / 2.0, (k - 1.5) * self.pitch + fw / 2.0,
                              0.0, sc.finger_len) for k in range(4)]
        self.deck_rect = (-1.5 * self.pitch - fw / 2.0, 1.5 * self.pitch + fw / 2.0,
                          -sc.deck_depth, 0.0)

    def at(self, u, v):
        """World point of dock coordinates (u right, v out)."""
        return add(add(self.centre, mul(self.right, u)), mul(self.out, v))

    def uv(self, p):
        """(u right, v out) of a world point, about the dock centre."""
        d = sub(p, self.centre)
        return dot(d, self.right), dot(d, self.out)

    def poly(self, rect):
        u0, u1, v0, v1 = rect
        return [self.at(u0, v0), self.at(u1, v0), self.at(u1, v1), self.at(u0, v1)]

    def slip(self, bay):
        """(u0, u1) of a bay's slip: between its two fingers."""
        c = (bay - 2) * self.pitch
        return c - self.sc.slip_width / 2.0, c + self.sc.slip_width / 2.0

    def windows(self, bay):
        """[(index, slot, (e, n, z), (half_w, half_h))] for one bay's windows."""
        f = self.faces[bay]
        out = []
        for idx, (slot, r, u, hw, hh) in enumerate(WINDOW_SLOTS):
            q = add(f, mul(self.right, r))
            out.append((idx, slot, (q[0], q[1], self.face_z + u), (hw, hh)))
        return out

    def indicator(self, bay):
        f = self.faces[bay]
        return (f[0], f[1], self.face_z + INDICATOR_UP)

    def indicator_colour(self, bay):
        return GREEN if bay == self.sc.green_bay else RED

    def bay_of(self, corners):
        """The bay a hull is FULLY inside, or 0: every corner between that
        slip's fingers, in front of its face, and no further out than the
        fingers reach."""
        for i in (1, 2, 3):
            lo, hi = self.slip(i)
            if all(lo <= u <= hi and 0.0 <= v <= self.sc.finger_len
                   for u, v in (self.uv(c) for c in corners)):
                return i
        return 0

    def contact(self, corners):
        """Does the hull overlap a finger or the deck?"""
        return any(convex_overlap(corners, self.poly(r))
                   for r in self.finger_rects + [self.deck_rect])


# ------------------------------------------------------------------ the boat

class Boat:
    """The boat (~1.0 x 0.6 m) under ArduRover GUIDED, as it is configured.

    Position setpoints only (GuidedSetpoint.yaw is discarded upstream). It
    turns toward the point and drives forward; it does not strafe.

    ARRIVAL IS WP_RADIUS, NOT THE POINT. ArduRover calls a GUIDED destination
    reached once inside WP_RADIUS and, on a boat, starts loitering - at its
    STOPPING POINT, not at the target (mode_loiter.enter). This boat's
    WP_RADIUS is 2.0 m, so it parks up to 2 m short of wherever it was sent.
    `wp_radius` is a knob for exactly that reason.

    LOITERING: nothing is corrected inside LOIT_RADIUS (2.0 m). Outside it the
    boat returns to the loiter point forward OR ASTERN, whichever needs less
    turning (LOIT_TYPE 0).
    """
    LENGTH = 1.0               # rough, from the team
    BEAM = 0.6
    CAM_X = 0.37               # target_tracker.cam_x - near the bow
    CAM_Z = 0.41               # above the waterline (0.65 datum - 0.24 draft)
    WP_SPEED = 1.0             # WP_SPEED, m/s
    ACCEL = 1.0                # ATC_ACCEL_MAX, m/s^2
    DECEL = 0.5                # the braking the speed profile plans with
    RATE_MAX = 60.0            # deg/s actually achieved (ATC_STR_RAT_MAX is 90)
    YAW_ACCEL = 90.0           # deg/s^2

    def __init__(self, e, n, heading, wp_radius=0.2, loit_radius=2.0):
        self.e, self.n, self.yaw = e, n, heading % 360.0
        self.v = 0.0           # forward speed, m/s (negative = astern)
        self.r = 0.0           # yaw rate, deg/s
        self.sp = None         # (e, n): the GUIDED destination
        self.wp_radius = wp_radius
        self.loit_radius = loit_radius
        self.mode = "GUIDED"
        self.armed = True
        self.loiter_pt = None  # set once the destination is reached
        self.origin = None     # where the current leg started
        self.returning = False
        self.odometer = 0.0

    @property
    def p(self):
        return (self.e, self.n)

    @property
    def loitering(self):
        return self.loiter_pt is not None

    def set_point(self, e, n):
        """A GUIDED position target. The same point again changes nothing.

        The leg starts where the last one was reached, or here if it was not
        (AR_WPNav::set_desired_location), and the boat follows THAT LINE.
        """
        if self.sp is not None and norm(sub((e, n), self.sp)) < 0.05:
            return
        self.origin = self.loiter_pt if self.loiter_pt is not None else self.p
        self.sp = (e, n)
        self.loiter_pt = None
        self.returning = False
        if norm(sub(self.sp, self.p)) <= self.wp_radius:
            self._arrive()

    def _carrot(self):
        """Where to steer on the current leg: `look` metres ahead of the boat's
        projection onto the origin->destination line. Following the line, not
        the point, is what lets it hold a centreline across a current - it
        crabs into it - which pure pursuit of the point cannot."""
        o, dst = self.origin or self.p, self.sp
        seg = sub(dst, o)
        length = norm(seg)
        if length < 0.5:
            return dst
        u = mul(seg, 1.0 / length)
        s = clamp(dot(sub(self.p, o), u), 0.0, length)
        look = max(1.0, 2.0 * abs(self.v))
        return add(o, mul(u, min(length, s + look)))

    def _arrive(self):
        """Reached: loiter at the STOPPING point, v^2/2a along the heading."""
        stop = self.v * abs(self.v) / (2.0 * self.ACCEL)
        self.loiter_pt = add(self.p, mul(hvec(self.yaw), stop))
        self.returning = False

    def _drive(self, target, allow_astern, steer_to=None):
        """(v_des, r_des) toward `target`, steering at `steer_to` if given."""
        d = norm(sub(target, self.p))
        aim = steer_to if steer_to is not None else target
        err = wrap180(bearing_deg(self.p, aim) - self.yaw)
        astern = allow_astern and abs(err) > 100.0
        if astern:
            err = wrap180(err + 180.0)
        r_des = clamp(2.0 * err, -self.RATE_MAX, self.RATE_MAX)
        cap = min(self.WP_SPEED, math.sqrt(2.0 * self.DECEL * d))
        if abs(err) < 75.0:
            v_des = cap * max(0.0, math.cos(err * DEG)) ** 3
        else:
            v_des = 0.15                          # a tight arc, not a pivot
        if astern:
            v_des = -min(v_des, 0.5)
        return v_des, r_des

    def corners(self):
        f = hvec(self.yaw)
        l = port(f)
        hl, hb = self.LENGTH / 2.0, self.BEAM / 2.0
        c = self.p
        return [add(add(c, mul(f, hl)), mul(l, hb)), add(add(c, mul(f, hl)), mul(l, -hb)),
                add(add(c, mul(f, -hl)), mul(l, -hb)), add(add(c, mul(f, -hl)), mul(l, hb))]

    def camera(self):
        """(e, n, z) of the camera and the compass direction it looks."""
        c = add(self.p, mul(hvec(self.yaw), self.CAM_X))
        return (c[0], c[1], self.CAM_Z), self.yaw

    def step(self, dt, current=(0.0, 0.0)):
        v_des, r_des = 0.0, 0.0
        if self.armed and self.mode == "GUIDED" and self.sp is not None:
            if self.loiter_pt is None and norm(sub(self.sp, self.p)) <= self.wp_radius:
                self._arrive()
            if self.loiter_pt is None:
                v_des, r_des = self._drive(self.sp, allow_astern=False, steer_to=self._carrot())
            else:
                dl = norm(sub(self.loiter_pt, self.p))
                if dl > self.loit_radius:
                    self.returning = True         # drifted out: go back
                elif dl < 0.3:
                    self.returning = False
                if self.returning:
                    v_des, r_des = self._drive(self.loiter_pt, allow_astern=True)
        dv = clamp(v_des - self.v, -self.ACCEL * dt, self.ACCEL * dt)
        self.v += dv
        dr = clamp(r_des - self.r, -self.YAW_ACCEL * dt, self.YAW_ACCEL * dt)
        self.r += dr
        self.yaw = (self.yaw + self.r * dt) % 360.0
        f = hvec(self.yaw)
        de = f[0] * self.v * dt + current[0] * dt
        dn = f[1] * self.v * dt + current[1] * dt
        self.e += de
        self.n += dn
        self.odometer += math.hypot(de, dn)


# ------------------------------------------------------------------ the lights

class Lights:
    """RoboCommand's side of the bay: the fire, the hit, the code."""

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.state = "IDLE"
        self.t0 = 0.0
        self.sprayed = 0.0
        self.hit_t = None

    def activate(self, t):
        if self.state == "IDLE":
            self.state = "ARMING"
            self.t0 = t

    def window_colour(self, t):
        """(index, colour) of the lit window right now, colour OFF if dark."""
        w = self.sc.target_window
        s = self.state
        if s in ("IDLE", "ARMING", "DONE"):
            return w, OFF
        if s == "FIRE":
            return w, RED
        dt = t - self.t0
        if s == "GREEN":
            return w, GREEN
        if s == "GAP":
            return w, OFF
        if s == "FLASH":                           # Advanced: c on 1 s, off 1 s
            return w, (BY_NAME[self.sc.code[1]] if (dt % 2.0) < 1.0 else OFF)
        if s == "CODE":                            # c1 1, off 1, c2 1, off 2
            x = dt % 5.0
            if x < 1.0:
                return w, BY_NAME[self.sc.code[0]]
            if 2.0 <= x < 3.0:
                return w, BY_NAME[self.sc.code[1]]
            return w, OFF
        return w, OFF

    def step(self, t, spraying_target, dt):
        s = self.state
        if s == "ARMING" and t - self.t0 >= self.sc.activation_delay_s:
            self.state, self.t0 = "FIRE", t
        elif s == "FIRE":
            if spraying_target:
                self.sprayed += dt
            if self.sprayed >= self.sc.extinguish_s:
                self.state, self.t0, self.hit_t = "GREEN", t, t
        elif s == "GREEN" and t - self.t0 >= 5.0:
            self.state, self.t0 = "GAP", t
        elif s == "GAP" and t - self.t0 >= 1.0:
            nxt = {0: "DONE", 1: "FLASH", 2: "CODE"}[self.sc.tier]
            self.state, self.t0 = nxt, t
        elif s in ("FLASH", "CODE") and t - self.t0 >= 60.0:
            self.state, self.t0 = "DONE", t


# ------------------------------------------------------------------ the camera

class Camera:
    """What the dock detector would publish, one DockObservation per frame."""
    FPS = 15.0
    FX = 1173.0                # px at 1920 wide (CV report)
    W_PX, H_PX = 1920, 1200
    HFOV = 2.0 * math.degrees(math.atan(960.0 / 1173.0))      # ~78.6
    VFOV = 2.0 * math.degrees(math.atan(600.0 / 1173.0))      # ~54.2
    STEREO_MIN_M = 0.31        # 640x400 stereo, extended disparity (CV spec)
    PLANE_MAX_M = 12.0
    DOWN = math.degrees(math.atan(600.0 / 1173.0))   # below the axis

    def __init__(self, sc: Scenario, rnd: random.Random):
        self.sc = sc
        self.rnd = rnd
        self.seq = DockSequence()
        self._stamps = []          # recent frame times, for observed_fps

    # detection rates from the CV report's held-out numbers
    @staticmethod
    def p_face(r):
        return 0.99 if r <= 9.0 else max(0.0, 0.99 - (r - 9.0) * 0.25)

    @staticmethod
    def p_indicator(r):
        if r <= 6.0:
            return 0.98
        if r <= 10.0:
            return 0.74
        if r <= 13.0:
            return 0.30
        return 0.0

    def _read(self, true_colour):
        """The white-referenced colour rule: abstains sometimes, never wrong
        unless `miscolour` says so."""
        u = self.rnd.random()
        if u < self.sc.unknown_rate:
            return UNKNOWN, 0.3
        if true_colour in LIT and u < self.sc.unknown_rate + self.sc.miscolour:
            return self.rnd.choice([c for c in LIT if c != true_colour]), 0.7
        return true_colour, self.rnd.uniform(0.75, 0.97)

    def up_deg(self):
        """How far above its axis the camera sees, with the hull band masked."""
        return math.degrees(math.atan((600.0 - self.sc.hull_band_rows) / self.FX))

    def to_cam(self, cam, f, lft, pt):
        """A world point (e, n, z) in the camera frame (x fwd, y left, z up),
        pitch included: + pitch aims DOWN, so a point level with the camera
        sits ABOVE its axis."""
        d = (pt[0] - cam[0], pt[1] - cam[1])
        xb, yb, zb = dot(d, f), dot(d, lft), pt[2] - cam[2]
        pr = self.sc.cam_pitch_deg * DEG
        return (xb * math.cos(pr) - zb * math.sin(pr), yb, xb * math.sin(pr) + zb * math.cos(pr))

    def in_view(self, c):
        """Is a camera-frame point inside the image (and not the hull band)?"""
        if c[0] <= 0.05:
            return False
        az = math.degrees(math.atan2(c[1], c[0]))
        el = math.degrees(math.atan2(c[2], c[0]))
        return abs(az) <= self.HFOV / 2.0 and -self.DOWN <= el <= self.up_deg()

    def window_visible(self, cam, f, lft, dock, centre, half):
        """A window counts only if the whole opening is in the image."""
        rgt = dock.right
        for su in (-1, 1):
            for sz in (-1, 1):
                q = add((centre[0], centre[1]), mul(rgt, su * half[0]))
                if not self.in_view(self.to_cam(cam, f, lft, (q[0], q[1], centre[2] + sz * half[1]))):
                    return False
        return True

    def frame(self, t, boat: Boat, dock: Dock, lights: Lights):
        cam, look = boat.camera()
        f = hvec(look)
        lft = port(f)
        rnd = self.rnd
        seen = []
        for bay in (1, 2, 3):
            face = dock.faces[bay]
            d = sub(face, (cam[0], cam[1]))
            r = norm(d)
            if dot(d, f) <= 0.3:
                continue
            c = self.to_cam(cam, f, lft, (face[0], face[1], dock.face_z))
            if abs(math.degrees(math.atan2(c[1], c[0]))) > self.HFOV / 2.0:
                continue
            # some of the face must be in the image vertically
            top = self.to_cam(cam, f, lft, (face[0], face[1], dock.face_z + FACE_H / 2.0))
            bot = self.to_cam(cam, f, lft, (face[0], face[1], dock.face_z - FACE_H / 2.0))
            if math.degrees(math.atan2(bot[2], bot[0])) > self.up_deg() or \
                    math.degrees(math.atan2(top[2], top[0])) < -self.DOWN:
                continue
            # the face must face the camera, and not too obliquely
            if dot(dock.out, mul(d, -1.0 / max(r, 1e-6))) < math.cos(75.0 * DEG):
                continue
            if rnd.random() > self.p_face(r):
                continue
            corners = [(face[0] + dock.right[0] * su * FACE_W / 2.0,
                        face[1] + dock.right[1] * su * FACE_W / 2.0,
                        dock.face_z + sz * FACE_H / 2.0) for su in (-1, 1) for sz in (-1, 1)]
            truncated = not all(self.in_view(self.to_cam(cam, f, lft, q)) for q in corners)
            seen.append((bay, face, r, c, truncated))

        # left to right in THIS frame: + bearing is left
        seen.sort(key=lambda s: -math.atan2(s[3][1], s[3][0]))
        bays = []
        tracked_states = None
        pr = self.sc.cam_pitch_deg * DEG
        for idx, (bay, face, r, c, truncated) in enumerate(seen):
            # the face centre, with range error along the line of sight
            rc = math.sqrt(c[0] ** 2 + c[1] ** 2 + c[2] ** 2)
            k = 1.0 + rnd.gauss(0.0, 0.01 + 0.004 * r * r) / rc
            cn = (c[0] * k, c[1] * k, c[2] * k)
            b_n = math.degrees(math.atan2(cn[1], cn[0])) + rnd.gauss(0.0, 0.2)
            has_plane = self.STEREO_MIN_M <= r <= self.PLANE_MAX_M
            # the face's normal, noisy in the horizontal, then into the tilted frame
            ang = rnd.gauss(0.0, 3.0) * DEG
            ob = (dot(dock.out, f), dot(dock.out, lft))
            nbx = ob[0] * math.cos(ang) - ob[1] * math.sin(ang)
            nby = ob[0] * math.sin(ang) + ob[1] * math.cos(ang)
            nc = (nbx * math.cos(pr), nby, nbx * math.sin(pr))
            offset = -(nc[0] * cn[0] + nc[1] * cn[1] + nc[2] * cn[2])
            ip = dock.indicator(bay)
            ind_vis = self.in_view(self.to_cam(cam, f, lft, ip))
            ind_present = ind_vis and rnd.random() < self.p_indicator(r)
            ind_col, ind_conf = self._read(dock.indicator_colour(bay)) if ind_present else (UNKNOWN, 0.0)

            windows = []
            states = {}
            lit_w, lit_c = lights.window_colour(t) if bay == self.sc.green_bay else (-1, OFF)
            for w_idx, slot, wc, half in dock.windows(bay):
                if not self.window_visible(cam, f, lft, dock, wc, half):
                    continue                  # not seen whole: not reported
                wcam = self.to_cam(cam, f, lft, wc)
                true_c = lit_c if w_idx == lit_w else OFF
                st, conf = self._read(true_c)
                states[w_idx] = NAME[st]
                windows.append({
                    "index": w_idx, "slot": slot, "identity_confidence": 0.95,
                    "state": st, "state_confidence": conf,
                    "lit_score": 0.4 if st in LIT else 0.02,
                    "detector_confidence": 0.9, "bbox": [0, 0, 0, 0],
                    "has_position": has_plane,
                    "x": wcam[0] + rnd.gauss(0.0, 0.02), "y": wcam[1] + rnd.gauss(0.0, 0.02),
                    "z": wcam[2] + rnd.gauss(0.0, 0.02)})
            lit = [w["index"] for w in windows if w["state"] in LIT and w["state_confidence"] >= 0.5]
            h_px = self.FX * WINDOW_M / r
            bays.append({
                "bay_index": idx, "detector_confidence": 0.9, "bbox": [0, 0, 0, 0],
                "truncated": truncated,
                "indicator_present": ind_present, "indicator_colour": ind_col,
                "indicator_confidence": ind_conf, "indicator_bbox": [0, 0, 0, 0],
                "windows": windows,
                "lit_window_index": lit[0] if len(lit) == 1 else -1,
                "lit_state": next(w["state"] for w in windows if w["index"] == lit[0])
                if len(lit) == 1 else UNKNOWN,
                "has_plane": has_plane,
                "plane_normal": list(nc) if has_plane else [None, None, None],
                "plane_offset": offset if has_plane else None,
                "plane_rms_m": 0.01 if has_plane else None,
                "range_from_size_m": (rc * (1.0 + rnd.gauss(0.0, 1.0 / h_px))) if windows else None,
                "bearing_deg": b_n,
                "_truth_bay": bay})
            # The timing stage tracks the bay whose indicator reads GREEN, or
            # the only bay in view (dock_detector_node.md section 4 step 6).
            if ind_col == GREEN or len(seen) == 1:
                tracked_states = states

        if tracked_states is not None:
            new = self.seq.update(t, tracked_states)
        else:
            new = self.seq.update(t, {})
        ev = ""
        for e in new:
            if e["type"] in ("hit", "hit_done", "lost"):
                ev = e["type"]
        target = self.seq.target
        pat, cols = ("", ())
        if target is not None and target in self.seq.patterns:
            pat, cols = self.seq.patterns[target]
        # The MEAN rate over the last 30 frames. Not the median gap: frames land
        # on 20 Hz world steps, so gaps alternate 0.05 / 0.10 s and the median
        # says 20 fps for a 15 fps camera.
        self._stamps = (self._stamps + [t])[-31:]
        span = self._stamps[-1] - self._stamps[0]
        fps = (len(self._stamps) - 1) / span if span > 0 else self.FPS
        return {
            "stamp": t, "frame_id": "camera_link", "bays": bays,
            "target_pattern": pat, "target_colours": list(cols),
            "target_window_index": target if target is not None else -1,
            "last_event": ev,
            "observed_fps": fps}


# ------------------------------------------------------------------ the judges

class Judge:
    """RoboCommand, and the UAV's end of the radio: records every report and
    marks it against the truth."""

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.events = []                   # (t, who, text, ok)
        self.docking = None                # the ACCEPTED docking report
        self.fire = None
        self.request = None
        self.uav = None
        self.confirm_at = None
        self.contacts = 0
        self.first_docked_t = None
        self.docking_heard = 0             # docking reports that reached us

    def log(self, t, who, text, ok=None):
        self.events.append({"t": round(t, 2), "who": who, "text": text, "ok": ok})

    def on_docking(self, t, bay_id, truth_bay, lights):
        want = self.sc.green_bay
        self.docking_heard += 1
        if self.docking_heard <= self.sc.lose_docking_reports:
            self.log(t, "RoboCommand", "DockingReport bay_id=%d LOST on the way (simulated)"
                     % bay_id, None)
            return
        if not self.sc.strict_judge:
            ok = True
        else:
            ok = bay_id == want and truth_bay == want
        why = ("docked in bay %d" % truth_bay) if truth_bay else "not docked in any bay"
        self.log(t, "RoboCommand", "DockingReport bay_id=%d (%s; the GREEN bay is %d)"
                 % (bay_id, why, want), ok)
        if ok and self.docking is None:
            self.docking = {"bay_id": bay_id, "t": t}
            self.confirm_at = t + 0.5
            lights.activate(t)

    def on_firefighting(self, t, window_id, lights):
        want = self.sc.target_window + 1
        ok = window_id == want and lights.hit_t is not None
        self.log(t, "RoboCommand", "FirefightingReport window_id=%d (lit: %d, fire %s)"
                 % (window_id, want, "out" if lights.hit_t is not None else "STILL BURNING"), ok)
        if self.fire is None:
            self.fire = {"window_id": window_id, "ok": ok, "t": t}

    def expected_request(self):
        if self.sc.tier == 2:
            return (self.sc.code[0].upper(), self.sc.code[1].upper())
        if self.sc.tier == 1:
            return ("ANY", self.sc.code[1].upper())
        return None

    def on_request(self, t, rq):
        got = (rq.get("resource_color", "").replace("COLOR_", ""),
               rq.get("delivery_circle_color", "").replace("COLOR_", ""))
        want = self.expected_request()
        ok = want is not None and got == want
        self.log(t, "RoboCommand", "ResourceDeliveryRequest %s -> %s (flashed: %s)"
                 % (got[0], got[1], "%s -> %s" % want if want else "nothing"), ok)
        if self.request is None:
            self.request = {"got": got, "ok": ok, "t": t}

    def on_uav(self, t, rq):
        names = {1: "RED", 2: "GREEN", 3: "BLUE", 4: "ANY"}
        got = (names.get(rq.get("resource_color"), "?"), names.get(rq.get("delivery_color"), "?"))
        want = self.expected_request()
        ok = want is not None and got == want
        self.log(t, "UAV", "tasked: fetch %s, deliver to the %s circle" % got, ok)
        if self.uav is None:
            self.uav = {"got": got, "ok": ok, "t": t}

    def summary(self):
        return {
            "docking": self.docking is not None,
            "firefighting": bool(self.fire and self.fire["ok"]),
            "request": bool(self.request and self.request["ok"]),
            "uav": bool(self.uav and self.uav["ok"]),
            "contacts": self.contacts,
        }


# ------------------------------------------------------------------ the world

class World:
    """Everything, stepped together. sim.py owns the clock and the runner."""
    DT = 0.05

    def __init__(self, sc: Scenario):
        self.sc = sc
        self.rnd = random.Random(sc.seed)
        self.dock = Dock(sc)
        self.boat = Boat(sc.start_e, sc.start_n, sc.start_heading, sc.wp_radius, sc.loit_radius)
        self.lights = Lights(sc)
        self.camera = Camera(sc, self.rnd)
        self.judge = Judge(sc)
        self.t = 0.0
        self._next_cam = 0.0
        self._next_pose = 0.0
        self._next_status = 0.0
        self.cannon = {"fire": False, "x": 0.0, "y": 0.0, "z": 0.0}
        self.spray_on = None               # window index being hit, or None
        self.in_contact = False
        self.trail = []
        self.last_obs = None               # the latest DockObservation, for the page
        self._pending_confirm = False
        self._mount_sent = False

    # -------------------------------------------------------------- inputs

    def approach_latlon(self):
        p = add(self.dock.centre, mul(self.dock.out, self.sc.approach_m))
        return to_latlon(p[0], p[1], self.sc.origin)

    def on_setpoint(self, lat, lon):
        e, n = to_local(lat, lon, self.sc.origin)
        self.boat.set_point(e, n)

    def on_cannon(self, c):
        self.cannon = c

    def truth_bay(self):
        return self.dock.bay_of(self.boat.corners())

    # -------------------------------------------------------------- step

    def step(self):
        """Advance DT. Returns the messages due to the runner this step."""
        sc, dt = self.sc, self.DT
        cur = mul(hvec(sc.current_to_deg), sc.current_mps)
        self.boat.step(dt, cur)
        self.t += dt
        t = self.t

        corners = self.boat.corners()
        touching = self.dock.contact(corners)
        if touching and not self.in_contact:
            self.judge.contacts += 1
            self.judge.log(t, "course", "hull contact with the dock", False)
        self.in_contact = touching
        if self.judge.first_docked_t is None and self.dock.bay_of(corners):
            self.judge.first_docked_t = t

        self.spray_on = self._spray_hits()
        self.lights.step(t, self.spray_on == sc.target_window, dt)
        if self.judge.confirm_at is not None and t >= self.judge.confirm_at:
            self.judge.confirm_at = None
            self.judge.log(t, "RoboCommand", "ReadinessConfirm sent", True)
            self._pending_confirm = True

        out = []
        if not self._mount_sent:
            # bt_runner_node's cam_* and dock_face_dz_m parameters, which the
            # runner cannot know: the scenario can change the camera's pitch.
            self._mount_sent = True
            out.append({"type": "mount", "x": Boat.CAM_X, "y": 0.0, "yaw_deg": 0.0,
                        "pitch_deg": sc.cam_pitch_deg,
                        "face_dz": self.dock.face_z - Boat.CAM_Z})
        if t >= self._next_pose:
            self._next_pose += 0.1
            lat, lon = to_latlon(self.boat.e + self.rnd.gauss(0, sc.gps_sd_m),
                                 self.boat.n + self.rnd.gauss(0, sc.gps_sd_m), sc.origin)
            out.append({"type": "pose", "lat": lat, "lon": lon,
                        "heading": (self.boat.yaw + self.rnd.gauss(0, sc.heading_sd_deg)) % 360.0})
        if t >= self._next_status:
            self._next_status += 0.5
            out.append({"type": "status", "mode": self.boat.mode, "armed": self.boat.armed})
        if t >= self._next_cam:
            self._next_cam += 1.0 / Camera.FPS
            if sc.camera_ok:
                obs = self.camera.frame(t, self.boat, self.dock, self.lights)
                self.last_obs = obs
                out.append(dict(obs, type="dock_obs"))
        if self._pending_confirm:
            self._pending_confirm = False
            out.append({"type": "ocs_command",
                        "data": {"readiness_confirm": {"report_seq": 1, "vehicle_id": "USV1"}}})
        if not self.trail or norm(sub(self.trail[-1], self.boat.p)) > 0.3:
            self.trail.append(self.boat.p)
            self.trail = self.trail[-2000:]
        return out

    def _spray_hits(self):
        """Which window the stream is landing on, or None.

        The command's aim point is in camera_link (tilted by the pitch); put it
        in the world with the boat's TRUE pose, and it is a hit when it lands
        inside a window's opening on the face. The stream reaches 6 m.
        """
        c = self.cannon
        if not c.get("fire"):
            return None
        cam, look = self.boat.camera()
        f = hvec(look)
        pr = self.sc.cam_pitch_deg * DEG
        xb = c["x"] * math.cos(pr) + c["z"] * math.sin(pr)
        zb = -c["x"] * math.sin(pr) + c["z"] * math.cos(pr)
        aim = add((cam[0], cam[1]), add(mul(f, xb), mul(port(f), c["y"])))
        aim_z = cam[2] + zb
        if norm(sub(aim, (cam[0], cam[1]))) > 6.0:
            return None
        bay = self.sc.green_bay
        for idx, _slot, (we, wn, wz), (hw, hh) in self.dock.windows(bay):
            d = sub(aim, (we, wn))
            if abs(dot(d, self.dock.right)) <= hw and abs(aim_z - wz) <= hh and \
                    abs(dot(d, self.dock.out)) <= 0.3:
                return idx
        return None

    def target_visible_from_berth(self, berth_m=1.25):
        """Could a boat berthed in the green bay (body origin berth_m out, bow
        in) see the fire window whole? The tree cannot put out what it cannot
        see, and this is the first thing to know about a scenario."""
        b = self.dock
        p = b.at((self.sc.green_bay - 2) * b.pitch, berth_m)
        heading = (self.sc.facing_deg + 180.0) % 360.0
        f = hvec(heading)
        c = add(p, mul(f, Boat.CAM_X))
        cam = (c[0], c[1], Boat.CAM_Z)
        for idx, _s, wc, half in b.windows(self.sc.green_bay):
            if idx == self.sc.target_window:
                return self.camera.window_visible(cam, f, port(f), b, wc, half)
        return False

    # -------------------------------------------------------------- outputs

    def on_report(self, kind, payload):
        """A report from the boat. `payload` is the JSON the runner published."""
        t = self.t
        if kind == "docking_report":
            self.judge.on_docking(t, int(payload["bay_id"]), self.truth_bay(), self.lights)
        elif kind == "firefighting_report":
            self.judge.on_firefighting(t, int(payload["window_id"]), self.lights)
        elif kind == "resource_request":
            self.judge.on_request(t, payload)
        elif kind == "uav_request":
            self.judge.on_uav(t, payload)

    def snapshot(self):
        """Truth, for the page."""
        sc = self.sc
        d = self.dock
        lw, lc = self.lights.window_colour(self.t)
        return {
            "t": round(self.t, 2),
            "boat": {"e": self.boat.e, "n": self.boat.n, "heading": self.boat.yaw,
                     "v": self.boat.v, "corners": self.boat.corners(),
                     "sp": self.boat.sp, "loitering": self.boat.loitering},
            "dock": {"faces": {str(k): v for k, v in d.faces.items()},
                     "fingers": [d.poly(r) for r in d.finger_rects], "deck": d.poly(d.deck_rect),
                     "out": d.out, "right": d.right, "green_bay": sc.green_bay,
                     "finger_len": sc.finger_len, "face_w": FACE_W,
                     "windows": {str(b): [(i, s, p) for i, s, p, _h in d.windows(b)]
                                 for b in (1, 2, 3)}},
            "target_visible_from_berth": self.target_visible_from_berth(),
            "lights": {"state": self.lights.state, "window": lw, "colour": NAME[lc],
                       "sprayed": round(self.lights.sprayed, 2)},
            "cannon": self.cannon, "spray_on": self.spray_on,
            "truth_bay": self.truth_bay(), "contact": self.in_contact,
            "trail": self.trail[-600:],
            "judge": {"events": self.judge.events[-40:], "summary": self.judge.summary()},
            "scenario": asdict(sc),
        }
