"""wall_fit_core — the vertical wall ahead, and the slip fingers beside the boat.

For Task 3's fixed nozzle the boat's position IS the aim, so the boat needs its
distance to the dock face to a couple of centimetres, relative, right now. GPS
cannot do that; the LiDAR can, from a single sweep: at 1-3 m there are hundreds
of returns on a wall.

THE WALL IS FITTED AS A VERTICAL PLANE after levelling, i.e. a line in the
horizontal plane. Its perpendicular distance from the boat does not depend on
roll or pitch (rocking moves the points, not the plane), which is the property
squirt_cal needs from a range it calibrates against. Attitude only matters for
picking the height band, and a stale one costs a little band, not the range.

WHAT IT RANGES. The MID360 sits ~0.28 m above the water, upside down, seeing from
~7 deg above horizontal downwards. On a raised dock that is the dock's EDGE below
the target panel, not the panel. Calibration against this number absorbs the
offset, as long as the tree later positions by the same number (face_setback in
squirt_cal is only for the starting guess).

Frames: body REP-103 (x forward, y LEFT, z up; z datum = the hull bottom, so the
waterline is ClusterParams.water_z). No ROS; numpy only; unit-tested.
"""
import math
from dataclasses import dataclass

import numpy as np

from crusader_perception.lidar_cluster_core import level


@dataclass
class WallParams:
    sector_deg: float = 40.0         # half-angle ahead to look for the wall
    finger_sector_deg: float = 90.0  # fingers are BESIDE the boat: look wider
    r_min: float = 0.3               # horizontal range from the body origin [m]
    r_max: float = 4.0
    z_min_above_water: float = 0.03  # height band, above the waterline [m]
    z_max_above_water: float = 1.5
    tol_m: float = 0.03              # RANSAC inlier distance
    iters: int = 200
    max_points: int = 4000           # RANSAC scores a subsample of this size
    min_inliers: int = 40
    min_span_m: float = 0.4          # this much wall must be seen, along it
    max_angle_deg: float = 35.0      # a wall more oblique than this is not "ahead"
    finger_depth_m: float = 2.5      # look for fingers this far out from the wall
    finger_min_off_m: float = 0.25   # a finger's inner edge is at least this far
    finger_max_off_m: float = 1.5    # ... and at most this far to the side
    finger_min_points: int = 15
    finger_quantile: float = 0.10    # inner edge = this quantile of |along-wall|


@dataclass
class WallFit:
    valid: bool = False
    why: str = ""
    range_m: float = float("nan")
    angle_deg: float = float("nan")  # bearing of the wall's nearest point, + = left
    n_inliers: int = 0
    rms_m: float = float("nan")
    span_m: float = 0.0
    left_m: float = float("nan")     # left finger's inner edge, along the wall
    right_m: float = float("nan")    # right finger's, as a positive distance
    n_band: int = 0                  # points that survived the gates

    @property
    def lat_m(self):
        """Offset from the slip centre, + = LEFT; NaN without both fingers."""
        if math.isnan(self.left_m) or math.isnan(self.right_m):
            return float("nan")
        return (self.right_m - self.left_m) / 2.0


def select(pts_body, water_z, wp: WallParams, roll=0.0, pitch=0.0):
    """Levelled points in the range and height band within finger_sector_deg,
    and a mask of those also within sector_deg (the wall candidates)."""
    if pts_body.shape[0] == 0:
        return pts_body.reshape(0, 3), np.zeros(0, bool)
    lev = level(pts_body, roll, pitch)
    x, y, z = lev[:, 0], lev[:, 1], lev[:, 2]
    r = np.hypot(x, y)
    bearing = np.abs(np.arctan2(y, x))
    keep = ((r >= wp.r_min) & (r <= wp.r_max)
            & (bearing <= math.radians(max(wp.sector_deg, wp.finger_sector_deg)))
            & (z >= water_z + wp.z_min_above_water)
            & (z <= water_z + wp.z_max_above_water))
    ahead = bearing[keep] <= math.radians(wp.sector_deg)
    return lev[keep], ahead


def _line_through(p1, p2):
    """Unit normal n and offset d of the line through two 2D points (n.p = d)."""
    t = p2 - p1
    L = math.hypot(t[0], t[1])
    if L < 1e-6:
        return None
    n = np.array([-t[1], t[0]]) / L
    return n, float(n @ p1)


def fit(pts_body, water_z, wp: WallParams = None, roll=0.0, pitch=0.0, seed=0):
    """WallFit for one sweep of BODY-frame points (already to_body'd)."""
    wp = wp or WallParams()
    out = WallFit()
    band, ahead = select(pts_body, water_z, wp, roll, pitch)
    out.n_band = int(ahead.sum())
    if out.n_band < wp.min_inliers:
        out.why = f"only {out.n_band} points in the band ahead"
        return out
    wide = band[:, :2]
    xy = wide[ahead]
    rng = np.random.default_rng(seed)
    score_set = xy
    if xy.shape[0] > wp.max_points:
        score_set = xy[rng.choice(xy.shape[0], wp.max_points, replace=False)]

    best, best_n = None, 0
    idx = rng.integers(0, score_set.shape[0], size=(wp.iters, 2))
    for i, j in idx:
        line = _line_through(score_set[i], score_set[j])
        if line is None:
            continue
        n, d = line
        if d < 0:
            n, d = -n, -d                       # normal points from the boat to the wall
        if abs(math.degrees(math.atan2(n[1], n[0]))) > wp.max_angle_deg:
            continue                            # a finger, or a wall off to the side
        cnt = int((np.abs(score_set @ n - d) <= wp.tol_m).sum())
        if cnt > best_n:
            best, best_n = (n, d), cnt
    if best is None:
        out.why = "no line faces the boat"
        return out

    # refine on ALL band points: total least squares on the inliers
    n, d = best
    for _ in range(2):
        inl = xy[np.abs(xy @ n - d) <= wp.tol_m]
        if inl.shape[0] < 2:
            break
        c = inl.mean(axis=0)
        _, _, vt = np.linalg.svd(inl - c)
        n = vt[1] / np.linalg.norm(vt[1])
        d = float(n @ c)
        if d < 0:
            n, d = -n, -d
    resid = xy @ n - d
    mask = np.abs(resid) <= wp.tol_m
    inl = xy[mask]
    t = np.array([-n[1], n[0]])                 # along the wall, + = LEFT
    u = inl @ t
    out.n_inliers = int(mask.sum())
    out.span_m = float(u.max() - u.min()) if inl.shape[0] else 0.0
    out.rms_m = float(np.sqrt(np.mean(resid[mask] ** 2))) if mask.any() else float("nan")
    angle = math.degrees(math.atan2(n[1], n[0]))
    if out.n_inliers < wp.min_inliers:
        out.why = f"{out.n_inliers} inliers (< {wp.min_inliers})"
        return out
    if out.span_m < wp.min_span_m:
        out.why = f"wall seen over only {out.span_m:.2f} m"
        return out
    if abs(angle) > wp.max_angle_deg:
        out.why = f"wall at {angle:.0f} deg is not ahead"
        return out
    out.valid, out.range_m, out.angle_deg = True, d, angle

    # fingers: everything in the wide sector that is not wall, in front of
    # the wall, off to each side
    rest = wide[np.abs(wide @ n - d) > wp.tol_m]
    if rest.shape[0]:
        v = d - rest @ n                        # distance out from the wall
        ur = rest @ t
        near = (v > 0.05) & (v < wp.finger_depth_m)
        left = ur[near & (ur >= wp.finger_min_off_m) & (ur <= wp.finger_max_off_m)]
        right = -ur[near & (ur <= -wp.finger_min_off_m) & (ur >= -wp.finger_max_off_m)]
        if left.size >= wp.finger_min_points:
            out.left_m = float(np.quantile(left, wp.finger_quantile))
        if right.size >= wp.finger_min_points:
            out.right_m = float(np.quantile(right, wp.finger_quantile))
    return out


def params_from(d: dict) -> WallParams:
    """From wall_range_node's ROS params. KeyError on a missing key."""
    return WallParams(
        sector_deg=d["sector_deg"], finger_sector_deg=d["finger_sector_deg"],
        r_min=d["r_min"], r_max=d["r_max"],
        z_min_above_water=d["z_min_above_water"],
        z_max_above_water=d["z_max_above_water"],
        tol_m=d["tol_m"], iters=int(d["iters"]), max_points=int(d["max_points"]),
        min_inliers=int(d["min_inliers"]), min_span_m=d["min_span_m"],
        max_angle_deg=d["max_angle_deg"], finger_depth_m=d["finger_depth_m"],
        finger_min_off_m=d["finger_min_off_m"], finger_max_off_m=d["finger_max_off_m"],
        finger_min_points=int(d["finger_min_points"]),
        finger_quantile=d["finger_quantile"])
