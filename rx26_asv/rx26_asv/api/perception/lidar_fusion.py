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

A bearing gate ALONE is not enough. The MID360 sees 360 deg out to tens of metres,
so the wedge behind a buoy routinely contains a shoreline, a dock wall, or another
vessel — and those large surfaces return far more points than a 0.3 m buoy. Taking
the median over the whole wedge then reports the buoy at the shoreline's range,
flagged fused=True. That is worse than no fusion at all: a dropped detection is at
least absent, but a *relocated* one moves a real obstacle outside the avoidance
horizon while looking confident. So association is gated three ways, in order:

  1. bearing gate     — points must share the detection's bearing (camera is trusted here)
  2. range-consistency gate — points must agree with the CAMERA's own range to
     within max(range_gate_abs, range_gate_frac * camera_range). Stereo range is
     coarse, not useless; it is a perfectly good bracket for "which object in this
     wedge is the one the camera saw".
  3. nearest coherent cluster — among survivors, split on radial gaps of
     cluster_gap_m and take the NEAREST cluster with >= min_points. Ties inside the
     bracket resolve toward the closer obstacle, which is the objective-1 answer.

Fusion REFINES a range; it must never relocate a detection to a different object.

Safety rule (objective 1): a detection with no LiDAR support is NEVER dropped — it
is returned with its camera range intact and fused=False. Losing an obstacle is
strictly worse than carrying a coarse range for it. The same holds when the gates
above reject every point: the detection passes through on its camera range with
`reason` set, so `/crsd/fusion_health` can show *why* fusion is not contributing.
A miscalibrated extrinsic shows up as mass `range_disagree` rather than as silence.

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


# Why a detection was not fused (FusedDetection.reason; "" when it WAS fused).
# These are the health topic's diagnostic vocabulary — see fuse().
NO_POINTS = "no_points"            # nothing survived the bearing/height/range-band gates
RANGE_DISAGREE = "range_disagree"  # points on-bearing, but none near the camera range
TOO_FEW_POINTS = "too_few_points"  # bracketed points exist, no cluster reaches min_points


@dataclass
class FusionParams:
    bearing_gate_rad: float = math.radians(4.0)   # +/- window around detection bearing
    z_min: float = -0.5                            # BODY-z band (reject water/ground)
    z_max: float = 3.0                             # reject sky/superstructure
    r_min: float = 0.5                             # ignore returns closer than this [m]
    r_max: float = 60.0                            # MID360 usable range ceiling [m]
    min_points: int = 3                            # returns needed to accept a fix
    # --- range-consistency gate (see module docstring) ---
    # Half-width of the window around the CAMERA range that a LiDAR return must
    # fall in: max(range_gate_abs, range_gate_frac * camera_range). Stereo range
    # error grows roughly linearly with range (z^2 / (f*baseline)), hence the
    # fractional term; the absolute floor keeps close-in detections workable.
    # Default 0.5 is deliberately generous — on water, glare and low texture make
    # stereo worse than the textbook figure, and the nearest-cluster rule below
    # resolves what is left inside the bracket. Tune at bench (both are [DYN]).
    range_gate_frac: float = 0.5
    range_gate_abs: float = 2.0                    # [m]
    cluster_gap_m: float = 2.0                     # radial gap that splits objects [m]


@dataclass
class FusedDetection:
    """Same shape as depth_association.BodyDetection, plus fusion provenance."""
    label: str
    confidence: float
    x: float                # starboard+ [m]
    y: float                # forward+ [m]
    radius: float           # estimated object radius [m]
    fused: bool             # True = range came from LiDAR; False = camera passthrough
    n_support: int          # returns in the ACCEPTED cluster (0 when not fused)
    n_gate: int = 0         # returns that passed the bearing gate (before range gate)
    reason: str = ""        # "" when fused, else NO_POINTS/RANGE_DISAGREE/TOO_FEW_POINTS


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


def _nearest_cluster(ranges: np.ndarray, gap: float, min_points: int):
    """Split sorted ranges on radial gaps > `gap`; return the NEAREST cluster with
    at least `min_points` members, or None.

    Scanning outward (rather than taking the single nearest cluster and giving up)
    means a couple of stray near returns — spray, a railing, a bird — cannot veto a
    good fix behind them, while a real nearer object still wins over a farther one.
    """
    if ranges.size == 0:
        return None
    r = np.sort(ranges)
    splits = np.flatnonzero(np.diff(r) > gap) + 1
    for chunk in np.split(r, splits):
        if chunk.size >= min_points:
            return chunk
    return None


def passthrough(d, reason: str, n_gate: int = 0) -> FusedDetection:
    """Return a camera detection UNCHANGED, tagged with why fusion did not apply.

    The single place the objective-1 invariant is expressed: a detection fusion
    cannot improve is neither dropped nor moved — it keeps its camera position and
    radius verbatim. Used by fuse() for every gate rejection and by
    lidar_fusion_node for the no-cloud/stale-cloud cases, so the two paths cannot
    drift apart.
    """
    return FusedDetection(d.label, float(d.confidence),
                          float(d.x), float(d.y), float(d.radius),
                          False, 0, n_gate, reason)


def fuse(detections, points_body, params: FusionParams):
    """detections: iterable with .label/.confidence/.x/.y/.radius (BODY frame).
    points_body: (N,3) LiDAR points already transformed into BODY (see to_body).

    Returns (list[FusedDetection], n_fused). A detection is fused only when LiDAR
    returns pass all three gates (bearing, range-consistency vs the camera, nearest
    coherent cluster >= min_points) — see the module docstring for why bearing
    alone is unsafe. Detections that fail ANY gate are returned UNCHANGED on their
    camera range (fused=False) with `reason` set, never dropped.
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

        # 1. bearing gate — the camera's bearing is the trusted quantity
        if rng_k.size:
            sel = np.abs(_wrap(brg_k - det_b)) <= params.bearing_gate_rad
            on_bearing = rng_k[sel]
        else:
            on_bearing = np.empty(0)
        n_gate = int(on_bearing.size)
        if n_gate == 0:
            out.append(passthrough(d, NO_POINTS))
            continue

        # 2. range-consistency gate — reject objects the camera did not see here
        tol = max(params.range_gate_abs, params.range_gate_frac * det_r)
        bracketed = on_bearing[np.abs(on_bearing - det_r) <= tol]
        if bracketed.size == 0:
            out.append(passthrough(d, RANGE_DISAGREE, n_gate))
            continue

        # 3. nearest coherent cluster inside the bracket
        cluster = _nearest_cluster(bracketed, params.cluster_gap_m,
                                   params.min_points)
        if cluster is None:
            out.append(passthrough(d, TOO_FEW_POINTS, n_gate))
            continue

        fr = float(np.median(cluster))
        nx, ny = math.sin(det_b) * fr, math.cos(det_b) * fr
        # bbox angular size is fixed; the implied physical radius scales with range
        scale = fr / det_r if det_r > 1e-6 else 1.0
        radius = max(0.05, d.radius * scale)
        out.append(FusedDetection(d.label, float(d.confidence), nx, ny, radius,
                                  True, int(cluster.size), n_gate, ""))
        n_fused += 1
    return out, n_fused


def params_from(p: dict) -> FusionParams:
    """Build FusionParams from a flat {param_name: value} dict — the node's live
    ROS params, or a crusader_params.yaml section.

    Owns the one unit conversion at the config boundary (bearing gate is degrees
    in YAML, radians in the core). Raises KeyError on a missing key: a core param
    that config forgot must fail loudly, not silently fall back to a code default
    (Phase 3.5 single-source rule). tests/test_config_shared.py pins both
    directions of that mapping.
    """
    return FusionParams(
        bearing_gate_rad=math.radians(p["bearing_gate_deg"]),
        z_min=p["z_min"], z_max=p["z_max"],
        r_min=p["r_min"], r_max=p["r_max"],
        min_points=int(p["min_points"]),
        range_gate_frac=p["range_gate_frac"],
        range_gate_abs=p["range_gate_abs"],
        cluster_gap_m=p["cluster_gap_m"],
    )
