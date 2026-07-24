"""LiDAR↔camera detection fusion core — refine a camera detection's RANGE with
Livox MID360 returns that fall in its bearing sector.

This is the LiDAR counterpart to depth_association.py, and it slots in behind the
same seam that module's docstring anticipated ("a full voxel/ICP path can slot in
behind the same associate() signature later"). The split is identical: this file
is pure numpy/math (no rclpy, no ROS msgs) so the geometry is unit-tested on any
machine; lidar_fusion_node.py is the thin ROS wrapper.

Why fuse at the detection level (not a dense LiDAR depth image): the camera gives
a *reliable bearing* but stereo range degrades with distance; the MID360 gives an
*accurate range* but no class. So for each camera detection we keep its bearing
and replace its range with the robust median range of LiDAR points inside a narrow
bearing gate (and a height band that rejects water/sky returns).

Safety rule (objective 1): a detection with no LiDAR support is NEVER dropped — it
is returned with its camera range intact and fused=False. Losing an obstacle is
strictly worse than carrying a coarse range for it.

Frame convention (matches interfaces/msg/Detection.msg + depth_association.py):
  BODY: x = starboard+ [m], y = forward+ [m], z = up+ [m].
  Livox MID360 sensor frame: x = forward, y = left, z = up.
"""
import math
from dataclasses import dataclass

import numpy as np

# Nominal Livox(sensor) -> BODY axis remap (before any calibrated mount rotation):
#   body_x(starboard) = -lidar_y(left)
#   body_y(forward)   =  lidar_x(forward)
#   body_z(up)        =  lidar_z(up)
_R0 = np.array([[0.0, -1.0, 0.0],
                [1.0,  0.0, 0.0],
                [0.0,  0.0, 1.0]])


@dataclass
class LidarExtrinsics:
    """Calibrated LiDAR pose in the BODY frame, applied ON TOP of the nominal
    Livox->BODY axis remap. All-zero = LiDAR perfectly aligned with BODY axes at
    the origin. roll/pitch/yaw in radians (about BODY x/y/z); x/y/z in meters."""
    roll: float = 0.0
    pitch: float = 0.0
    yaw: float = 0.0
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0

    def rotation(self) -> np.ndarray:
        cr, sr = math.cos(self.roll), math.sin(self.roll)
        cp, sp = math.cos(self.pitch), math.sin(self.pitch)
        cy, sy = math.cos(self.yaw), math.sin(self.yaw)
        rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return rz @ ry @ rx

    @classmethod
    def from_degrees(cls, roll_deg, pitch_deg, yaw_deg, x, y, z):
        return cls(math.radians(roll_deg), math.radians(pitch_deg),
                   math.radians(yaw_deg), float(x), float(y), float(z))

    def is_identity(self, tol: float = 1e-9) -> bool:
        return all(abs(v) <= tol for v in
                   (self.roll, self.pitch, self.yaw, self.x, self.y, self.z))


def config_extrinsic_nonidentity(config: dict, tol: float = 1e-9) -> dict:
    """Scan a parsed Livox MID360 config for a non-identity driver-side extrinsic.

    Returns {lidar_config_index: {component: value}} for every `lidar_configs`
    entry whose `extrinsic_parameter` is not identity; {} if all are identity or
    absent. Used to catch the double-transform trap: the driver AND
    lidar_fusion_node both applying an extrinsic would move points twice.
    """
    out = {}
    for i, lc in enumerate(config.get("lidar_configs", []) or []):
        ext = lc.get("extrinsic_parameter") or {}
        vals = {k: float(ext.get(k, 0.0))
                for k in ("roll", "pitch", "yaw", "x", "y", "z")}
        nz = {k: v for k, v in vals.items() if abs(v) > tol}
        if nz:
            out[i] = nz
    return out


@dataclass
class FusionParams:
    bearing_gate_rad: float = math.radians(4.0)   # +/- window around detection bearing
    z_min: float = -0.5                            # BODY-z band (reject water/ground)
    z_max: float = 3.0                             # reject sky/superstructure
    r_min: float = 0.5                             # ignore returns closer than this [m]
    r_max: float = 60.0                            # MID360 usable range ceiling [m]
    min_points: int = 3                            # returns needed to accept a fix


@dataclass
class FusedDetection:
    """Same shape as depth_association.BodyDetection, plus fusion provenance."""
    label: str
    confidence: float
    x: float                # starboard+ [m]
    y: float                # forward+ [m]
    radius: float           # estimated object radius [m]
    fused: bool             # True = range came from LiDAR; False = camera passthrough
    n_support: int          # LiDAR returns in the bearing gate


def to_body(points, extr: LidarExtrinsics) -> np.ndarray:
    """points: (N,3) in the Livox sensor frame -> (N,3) in the BODY frame."""
    pts = np.asarray(points, dtype=float).reshape(-1, 3)
    if pts.shape[0] == 0:
        return pts.reshape(0, 3)
    remapped = pts @ _R0.T
    rotated = remapped @ extr.rotation().T
    return rotated + np.array([extr.x, extr.y, extr.z])


def _wrap(a):
    """Wrap angle(s) to (-pi, pi]; works on scalars and arrays."""
    return (np.asarray(a) + math.pi) % (2 * math.pi) - math.pi


def fuse(detections, points_body, params: FusionParams):
    """detections: iterable with .label/.confidence/.x/.y/.radius (BODY frame).
    points_body: (N,3) LiDAR points already transformed into BODY (see to_body).

    Returns (list[FusedDetection], n_fused). Detections without >= min_points of
    LiDAR support in their bearing gate are returned UNCHANGED (fused=False), never
    dropped.
    """
    pts = (np.asarray(points_body, dtype=float).reshape(-1, 3)
           if points_body is not None else np.empty((0, 3)))
    if pts.shape[0]:
        rng = np.hypot(pts[:, 0], pts[:, 1])                 # horizontal range
        brg = np.arctan2(pts[:, 0], pts[:, 1])               # bearing: starboard/forward
        keep = ((pts[:, 2] >= params.z_min) & (pts[:, 2] <= params.z_max)
                & (rng >= params.r_min) & (rng <= params.r_max))
        rng_k, brg_k = rng[keep], brg[keep]
    else:
        rng_k = brg_k = np.empty(0)

    out, n_fused = [], 0
    for d in detections:
        det_r = math.hypot(d.x, d.y)
        det_b = math.atan2(d.x, d.y)
        if rng_k.size:
            sel = np.abs(_wrap(brg_k - det_b)) <= params.bearing_gate_rad
            n = int(np.count_nonzero(sel))
        else:
            n = 0
        if n >= params.min_points:
            fr = float(np.median(rng_k[sel]))
            nx, ny = math.sin(det_b) * fr, math.cos(det_b) * fr
            scale = fr / det_r if det_r > 1e-6 else 1.0
            radius = max(0.05, d.radius * scale)
            out.append(FusedDetection(d.label, float(d.confidence),
                                      nx, ny, radius, True, n))
            n_fused += 1
        else:
            out.append(FusedDetection(d.label, float(d.confidence),
                                      float(d.x), float(d.y), float(d.radius),
                                      False, n))
    return out, n_fused
