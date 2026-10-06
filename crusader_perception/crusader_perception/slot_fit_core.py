"""slot_fit_core — the docking slip from one LiDAR sweep: its two side walls and its back wall.

Task 3 with the pan/tilt cannon docks by the LiDAR alone. The rule is a FIXED
standoff: 1.1 m from the LiDAR to the back wall, centred between the slip's two
side walls (the fingers' inner faces), square to the slip. The camera aims the
cannon; it no longer positions the boat, so nothing here depends on seeing the
windows. (Bumblebee's 2026 design splits it the same way: the LiDAR resolves the
bay's side walls during entry, where the camera's forward view no longer covers
them.)

WHAT IS FITTED, in the levelled body frame (x forward, y LEFT, z up):

  side walls  per side, the line NEAREST the boat that runs roughly along the
              bow (within max_yaw_deg), through the points beside it in the
              height band. The slip's axis is the mean of the two directions,
              its centreline their midpoint. From in front of the slip the
              neighbouring slips' fingers are further out than ours, and the
              finger TIPS run across the bow, so "nearest line along the bow"
              picks our slip's inner faces from outside it as well as inside.
              With only one wall the other is put slip_width_m across, and
              `why` says so.
  back wall   across the slip, AHEAD, between the side walls: the deck edge seen
              between the fingers. Found in the slip's own frame as the NEAREST
              band of points square to the axis that spans enough of the slip,
              then refined as a line. Its distance from the LIDAR (not the body
              origin), along the axis, is the standoff - the number the tree
              holds at 1.1 m to dock (1.6 m to watch).

THE MID360 IS LOW (0.28 m above the water) and upside down: it sees from ~7 deg
above level downwards, so on the course's 0.3 m dock it ranges the deck edge
(the panel above it stands ~3 cm further back and merges into the same band).

No ROS; numpy only; unit-tested on ray-cast clouds (test/test_slot_fit_core.py).
"""
import math
from dataclasses import dataclass

import numpy as np

from crusader_perception.lidar_cluster_core import level

STRONG_SHARE = 0.3     # a side line with this share of the best one's inliers may win on distance
BACK_BIN_M, BACK_COVER_MIN = 0.05, 0.5   # the back wall must have points along half the slip's width
BACK_MIN_HEIGHT_M = 0.10                 # ...and up a wall's height: the deck edge is 0.3 m


@dataclass
class SlotParams:
    r_min: float = 0.3                # horizontal range from the body origin [m]
    r_max: float = 6.0
    z_min_above_water: float = 0.03   # the height band that is not water [m]
    z_max_above_water: float = 1.0
    side_min_off_m: float = 0.30      # a side wall is this far beside the body origin...
    side_max_off_m: float = 1.40      # ...and at most this
    side_x_min: float = -0.6          # ...and seen between these x (body) [m]
    side_x_max: float = 4.5
    max_yaw_deg: float = 35.0         # a side wall runs within this of the bow
    parallel_deg: float = 10.0        # the two walls agree to within this
    slip_width_m: float = 1.5         # the build guide's slip; used with one wall
    width_tol_m: float = 0.45         # two walls further from slip_width than this: not a slip
    tol_m: float = 0.03               # RANSAC inlier distance
    iters: int = 150
    max_points: int = 3000            # RANSAC scores a subsample of this size
    min_inliers: int = 25
    min_span_m: float = 0.4           # this much wall, along it
    max_lines: int = 3                # lines tried per side before choosing the nearest
    back_margin_m: float = 0.15       # the back wall is looked for this far inside the walls
    back_min_ahead_m: float = 0.2     # ...and at least this far ahead of the LiDAR
    back_band_m: float = 0.06         # the 1-D search band, along the axis
    back_min_inliers: int = 20
    back_min_span_m: float = 0.3      # across the slip
    lidar_x: float = 0.32             # the LiDAR in the body frame (lidar_cluster_node's)
    lidar_y: float = 0.05


@dataclass
class SlotFit:
    valid: bool = False
    why: str = ""
    angle_deg: float = float("nan")     # slip axis, + = points LEFT of the bow
    lateral_m: float = float("nan")     # centreline at the body origin, + = LEFT
    width_m: float = float("nan")
    left_m: float = float("nan")        # perpendicular distance, body origin -> wall
    right_m: float = float("nan")
    n_left: int = 0
    n_right: int = 0
    rms_m: float = float("nan")
    back_range_m: float = float("nan")  # LiDAR -> back wall, along the axis
    n_back: int = 0
    n_band: int = 0                     # points that survived the gates

    @property
    def has_left(self):
        return not math.isnan(self.left_m)

    @property
    def has_right(self):
        return not math.isnan(self.right_m)

    @property
    def has_back(self):
        return not math.isnan(self.back_range_m)


def select(pts_body, water_z, sp: SlotParams, roll=0.0, pitch=0.0):
    """Levelled (x, y, z) of the points in the range and height band."""
    if pts_body.shape[0] == 0:
        return np.zeros((0, 3))
    lev = level(pts_body, roll, pitch)
    x, y, z = lev[:, 0], lev[:, 1], lev[:, 2]
    r = np.hypot(x, y)
    keep = ((r >= sp.r_min) & (r <= sp.r_max)
            & (z >= water_z + sp.z_min_above_water)
            & (z <= water_z + sp.z_max_above_water))
    return lev[keep]


def _wall_angle(n):
    """Direction of a line with unit normal n, as an angle from +x in (-90, 90]."""
    if n[1] < 0:
        n = -n
    return math.degrees(math.atan2(-n[0], n[1]))


def _ransac_side(pts, sp: SlotParams, rng):
    """(n, d) of the along-the-bow line with the most points within tol, or None.
    n points AWAY from the boat (d > 0)."""
    if pts.shape[0] < 2:
        return None
    if pts.shape[0] > sp.max_points:
        pts = pts[rng.choice(pts.shape[0], sp.max_points, replace=False)]
    best, best_n = None, 0
    for i, j in rng.integers(0, pts.shape[0], size=(sp.iters, 2)):
        t = pts[j] - pts[i]
        L = math.hypot(t[0], t[1])
        if L < 0.05:
            continue
        n = np.array([-t[1], t[0]]) / L
        d = float(n @ pts[i])
        if d < 0:
            n, d = -n, -d
        if abs(_wall_angle(n)) > sp.max_yaw_deg:
            continue
        cnt = int((np.abs(pts @ n - d) <= sp.tol_m).sum())
        if cnt > best_n:
            best, best_n = (n, d), cnt
    return best


def _refine(pts, line, tol):
    """Total least squares on the inliers, twice. Keeps n pointing away (d > 0)."""
    n, d = line
    for _ in range(2):
        inl = pts[np.abs(pts @ n - d) <= tol]
        if inl.shape[0] < 3:
            break
        c = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - c, full_matrices=False)
        n = vt[1] / np.linalg.norm(vt[1])
        d = float(n @ c)
        if d < 0:
            n, d = -n, -d
    return n, d


def _span(pts, n, d, tol):
    inl = pts[np.abs(pts @ n - d) <= tol]
    if inl.shape[0] < 2:
        return 0.0, inl
    u = inl @ np.array([-n[1], n[0]])
    return float(u.max() - u.min()), inl


def fit_side(xy, side, sp: SlotParams, rng):
    """The nearest solid along-the-bow line on one side (+1 left, -1 right):
    (n, d, n_inliers, rms) or None."""
    off = side * xy[:, 1]
    pool = xy[(off >= sp.side_min_off_m) & (off <= sp.side_max_off_m)
              & (xy[:, 0] >= sp.side_x_min) & (xy[:, 0] <= sp.side_x_max)]
    found = []
    for _ in range(sp.max_lines):
        if pool.shape[0] < sp.min_inliers:
            break
        line = _ransac_side(pool, sp, rng)
        if line is None:
            break
        n, d = _refine(pool, line, sp.tol_m)
        span, inl = _span(pool, n, d, sp.tol_m)
        if inl.shape[0] >= sp.min_inliers and span >= sp.min_span_m \
                and abs(_wall_angle(n)) <= sp.max_yaw_deg:
            res = inl @ n - d
            found.append((n, d, int(inl.shape[0]), float(np.sqrt(np.mean(res ** 2)))))
        pool = pool[np.abs(pool @ n - d) > sp.tol_m]
    if not found:
        return None
    # The nearest STRONG line: our slip's inner face. A weak line (a few dozen
    # points strung between the deck edge and a finger) can sit nearer than the
    # face it crosses; it does not get to win on distance alone.
    most = max(f[2] for f in found)
    strong = [f for f in found if f[2] >= STRONG_SHARE * most]
    return min(strong, key=lambda f: f[1])


def _rot(xy, ang_deg):
    """Points in a frame rotated by ang_deg (+ = left): x' along that direction."""
    a = math.radians(ang_deg)
    c, s = math.cos(a), math.sin(a)
    return np.column_stack([c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]])


def fit_back(q, y_left, y_right, lidar_q, sp: SlotParams, z=None):
    """The back wall in the slip frame q (x' along the axis, y' left): the
    nearest band across the slip, between y_right and y_left, ahead of the
    LiDAR, that fills the slip's width and stands up (z, the points' heights).
    Returns (range from the LiDAR along the axis, n_inliers) or None."""
    lo, hi = y_right + sp.back_margin_m, y_left - sp.back_margin_m
    if hi - lo < sp.back_min_span_m:
        return None
    sel = (q[:, 1] > lo) & (q[:, 1] < hi) & (q[:, 0] > lidar_q[0] + sp.back_min_ahead_m)
    pts = q[sel]
    zs_all = z[sel] if z is not None else np.zeros(pts.shape[0])
    if pts.shape[0] < sp.back_min_inliers:
        return None
    order = np.argsort(pts[:, 0])
    xs, ys, zs = pts[order, 0], pts[order, 1], zs_all[order]
    nbins = max(1, int((hi - lo) / BACK_BIN_M))

    def covered(y):
        """The share of the slip's width (BACK_BIN_M bins) that has points: a
        wall fills it, a few stray returns (water, spray) that happen to line
        up do not."""
        b = np.floor((y - lo) / (hi - lo) * nbins).astype(int)
        return np.unique(np.clip(b, 0, nbins - 1)).size / float(nbins)
    j = 0
    for i in range(xs.size):                        # sliding band, nearest first
        while xs[i] - xs[j] > sp.back_band_m:
            j += 1
        if i - j + 1 >= sp.back_min_inliers and ys[j:i + 1].max() - ys[j:i + 1].min() >= sp.back_min_span_m \
                and covered(ys[j:i + 1]) >= BACK_COVER_MIN \
                and (z is None or np.ptp(zs[j:i + 1]) >= BACK_MIN_HEIGHT_M):
            # The first window that qualifies starts at the NEAREST points, noise
            # included, so its median sits a centimetre or two short; re-centre
            # it on the wall's own points before fitting.
            x0 = float(np.median(xs[j:i + 1]))
            half = max(sp.tol_m, sp.back_band_m / 2.0)
            for _ in range(3):
                band = pts[np.abs(pts[:, 0] - x0) <= half]
                if band.shape[0] < 3:
                    break
                x0 = float(np.median(band[:, 0]))
            band = pts[np.abs(pts[:, 0] - x0) <= half]
            # x' = m y' + b: the small yaw the axis estimate still has
            if band.shape[0] >= 3 and np.ptp(band[:, 1]) > 1e-3:
                m, b = np.polyfit(band[:, 1], band[:, 0], 1)
            else:
                m, b = 0.0, x0
            x_at = m * lidar_q[1] + b
            rng_ = (x_at - lidar_q[0]) / math.sqrt(1.0 + m * m)
            return float(rng_), int(band.shape[0])
    return None


def fit(pts_body, water_z, sp: SlotParams = None, roll=0.0, pitch=0.0, seed=0):
    """SlotFit for one sweep of BODY-frame points (already to_body'd)."""
    sp = sp or SlotParams()
    out = SlotFit()
    xyz = select(pts_body, water_z, sp, roll, pitch)
    xy = xyz[:, :2]
    out.n_band = int(xy.shape[0])
    if out.n_band < sp.min_inliers:
        out.why = f"only {out.n_band} points in the band"
        return out
    rng = np.random.default_rng(seed)
    left = fit_side(xy, +1, sp, rng)
    right = fit_side(xy, -1, sp, rng)
    if left is None and right is None:
        out.why = "no side wall along the bow"
        return out
    notes = []
    if left is not None and right is not None:
        al, ar = _wall_angle(left[0]), _wall_angle(right[0])
        width = left[1] + right[1]
        if abs(al - ar) > sp.parallel_deg:
            notes.append(f"walls {al:+.0f}/{ar:+.0f} deg not parallel: kept the "
                         f"{'left' if left[2] >= right[2] else 'right'}")
            if left[2] >= right[2]:
                right = None
            else:
                left = None
        elif abs(width - sp.slip_width_m) > sp.width_tol_m:
            notes.append(f"walls {width:.2f} m apart, not a {sp.slip_width_m:.1f} m slip: kept the nearer")
            if left[1] <= right[1]:
                right = None
            else:
                left = None
    if left is not None and right is not None:
        out.angle_deg = (_wall_angle(left[0]) + _wall_angle(right[0])) / 2.0
        out.left_m, out.right_m = left[1], right[1]
        out.width_m = left[1] + right[1]
        out.n_left, out.n_right = left[2], right[2]
        out.rms_m = max(left[3], right[3])
    elif left is not None:
        out.angle_deg = _wall_angle(left[0])
        out.left_m, out.n_left, out.rms_m = left[1], left[2], left[3]
        notes.append(f"left wall only: right put {sp.slip_width_m:.2f} m across")
    else:
        out.angle_deg = _wall_angle(right[0])
        out.right_m, out.n_right, out.rms_m = right[1], right[2], right[3]
        notes.append(f"right wall only: left put {sp.slip_width_m:.2f} m across")
    # the walls in the slip's frame: y' = +left_m and y' = -right_m (one assumed)
    yl = out.left_m if out.has_left else sp.slip_width_m - out.right_m
    yr = -out.right_m if out.has_right else out.left_m - sp.slip_width_m
    out.lateral_m = (yl + yr) / 2.0
    q = _rot(xy, out.angle_deg)
    lq = _rot(np.array([[sp.lidar_x, sp.lidar_y]]), out.angle_deg)[0]
    back = fit_back(q, yl, yr, lq, sp, z=xyz[:, 2])
    if back is None:
        notes.append("no back wall between the side walls")
        out.why = "; ".join(notes)
        return out
    out.back_range_m, out.n_back = back
    out.valid = True
    out.why = "; ".join(notes)
    return out


def params_from(d: dict, lidar_x=0.32, lidar_y=0.05) -> SlotParams:
    """From dock_slot_node's ROS params (KeyError on a missing key), with the
    LiDAR's position from lidar_cluster_node's: one sensor, one mounting."""
    return SlotParams(
        r_min=d["r_min"], r_max=d["r_max"],
        z_min_above_water=d["z_min_above_water"], z_max_above_water=d["z_max_above_water"],
        side_min_off_m=d["side_min_off_m"], side_max_off_m=d["side_max_off_m"],
        side_x_min=d["side_x_min"], side_x_max=d["side_x_max"],
        max_yaw_deg=d["max_yaw_deg"], parallel_deg=d["parallel_deg"],
        slip_width_m=d["slip_width_m"], width_tol_m=d["width_tol_m"],
        tol_m=d["tol_m"], iters=int(d["iters"]), max_points=int(d["max_points"]),
        min_inliers=int(d["min_inliers"]), min_span_m=d["min_span_m"],
        back_margin_m=d["back_margin_m"], back_min_ahead_m=d["back_min_ahead_m"],
        back_band_m=d["back_band_m"], back_min_inliers=int(d["back_min_inliers"]),
        back_min_span_m=d["back_min_span_m"],
        lidar_x=float(lidar_x), lidar_y=float(lidar_y))
