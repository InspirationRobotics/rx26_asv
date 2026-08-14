"""lidar_cluster_core — MID360 point cloud to 3D object clusters.

Pure numpy: no rclpy, no ROS messages, no device. Every decision in here can be
exercised with an invented cloud on a laptop, which is the only reason the
geometry below is trustworthy — none of it is verifiable by staring at a live
sensor. `lidar_cluster_node.py` is the thin ROS wrapper.

`tools/lidar_view.py` imports the transform and the filters FROM HERE. The bench
view and the node must apply identical geometry or the orientation check
(docs/G2) proves nothing about what the node actually does.

THE PIPELINE, in order, and the order matters:

  1. sensor-frame filters (near_mask, fov_mask) — the origin here IS the sensor,
     which is what "0.5 m around the LiDAR" means, and both describe what the
     sensor can physically see.
  2. to_body — sign flips then translation, into REP-103 (x fwd, y left, z up).
  3. level — de-rotate roll and pitch out, keeping yaw. Now z is true vertical.
  4. height/range gates — water and sky. These MUST come after levelling: under
     power the boat pitches bow-up, and a z gate applied in the tilted body
     frame slices the water surface at a different height on every wave.
  5. cluster — voxel-grid DBSCAN, in the levelled frame so the distance metric
     is isotropic and "0.4 m apart" means the same thing at any attitude.

Cluster centroids are reported back in BODY frame, not levelled: the camera is
bolted to the same hull, so `oak/detections` are body-frame too, and fusion
compares the two directly. Whoever projects into the world applies attitude
once, at that point (crusader_common.geo.body_to_world_ypr). Publishing levelled
positions would make it apply twice.

FRAME, bench-confirmed 2026-08-14 (docs/G2_lidar_orientation.md): the raw Livox
frame is x forward, y STARBOARD, z down — the plain 180-degree roll about the
forward axis, which is what mounting it upside down does. Resources.md first
described it as "y left, z down", which is left-handed and cannot describe a
rigid sensor. Both y and z therefore negate into REP-103, and both signs are
parameters rather than constants so a remount is a config change.
"""
import math
from dataclasses import dataclass, field

import numpy as np

# Voxel keys are packed into one int64 so neighbour lookup is a searchsorted
# rather than a dict. STRIDE must exceed the index span and OFFSET must keep
# indices non-negative, or two different voxels collide onto one key and points
# silently merge across the map. 2^16 span at 0.05 m leaf covers +/-1638 m.
_VOX_OFFSET = 1 << 15
_VOX_STRIDE = 1 << 16


@dataclass
class ClusterParams:
    # --- extrinsic (sensor -> body) ---
    sign_y: float = -1.0            # bench-confirmed: raw +y is starboard
    sign_z: float = -1.0            # mounted upside down
    tx: float = 0.32                # sensor position in BODY [m]
    ty: float = 0.05
    tz: float = 0.52
    # --- sensor-frame filters ---
    r_min: float = 0.5              # clear sphere around the sensor [m]
    fov_deg: float = 180.0          # forward sector; hull blocks the rest
    # --- levelled-frame gates ---
    water_z: float = 0.10           # waterline height above the hull datum [m]
    water_margin: float = 0.15      # keep points this far above the water
    z_ceiling: float = 4.0          # reject sky/rain returns above this [m]
    r_max: float = 40.0             # horizontal range ceiling [m]
    # --- voxel-grid DBSCAN ---
    # eps stays SMALL and grows only gently with range. The instinct is to open
    # it up for distant objects, but that is backwards: a buoy is 0.3 m wide at
    # every range, so its returns are always within ~0.3 m of each other, while
    # the neighbourhood VOLUME grows as eps^3 and sweeps in proportionally more
    # scattered noise. Widening eps therefore helps the noise more than the
    # buoy. What actually changes with range is the SPACING between returns on
    # one object, which is what the gentle growth covers.
    leaf_size: float = 0.15         # voxel edge [m]
    eps_0: float = 0.30             # neighbourhood radius at eps_r_ref [m]
    eps_r_ref: float = 15.0         # range at which eps_0 applies [m]
    max_rings: int = 2              # cap on adaptive eps, in voxels
    min_density_points: int = 6     # raw points in the neighbourhood -> core
    min_points: int = 5             # raw points needed to emit a cluster
    max_extent_m: float = 8.0       # bigger than this is shoreline, not an object


@dataclass
class Cluster:
    """One object. Centroid and extent are BODY frame (x fwd, y left, z up)."""
    x: float
    y: float
    z: float
    ex: float                       # AABB extent [m]
    ey: float
    ez: float
    n_points: int
    range_m: float                  # horizontal range from the body origin


@dataclass
class ClusterStats:
    """Where the points went. Every stage reports, so a filter that is silently
    eating the whole cloud is visible instead of looking like a dead sensor."""
    n_in: int = 0
    n_near: int = 0                 # dropped by the clear sphere
    n_fov: int = 0                  # dropped as behind the beam
    n_water: int = 0                # dropped as water/below the margin
    n_sky: int = 0
    n_far: int = 0
    n_clustered: int = 0            # survived into a published cluster
    n_noise: int = 0                # DBSCAN noise, or a cluster below min_points
    n_clusters: int = 0
    levelled: bool = True           # False when attitude was stale
    dropped: dict = field(default_factory=dict)

    def as_dict(self):
        d = self.__dict__.copy()
        d.pop("dropped")
        return d


# ---------------------------------------------------------------- transforms

def near_mask(pts: np.ndarray, r_min: float) -> np.ndarray:
    """Drop returns inside a sphere of `r_min` around the SENSOR.

    Run in the RAW sensor frame, where the origin is the sensor itself — the
    mount, the cabling and the deck directly beneath it return on every sweep.
    They are not the world, and left in they form a permanent cluster at arm's
    length that no downstream gate removes.

    3D range, not horizontal: the strongest self-returns from an upside-down
    sensor point straight down, and a horizontal-only test keeps every one.
    """
    if pts.shape[0] == 0 or r_min <= 0.0:
        return np.ones(pts.shape[0], dtype=bool)
    return (pts ** 2).sum(axis=1) >= r_min * r_min


def fov_mask(pts: np.ndarray, fov_deg: float) -> np.ndarray:
    """Keep points within +/- fov/2 of dead ahead.

    The MID360 sees 360 degrees, but it is mounted at the bow and the hull
    blocks most of the aft view, so returns behind the beam are the boat's own
    structure rather than the world.

    Symmetric about the x axis, so it does not care which way y points: valid in
    the raw frame and the body frame alike.
    """
    if pts.shape[0] == 0 or fov_deg >= 360.0:
        return np.ones(pts.shape[0], dtype=bool)
    half = math.radians(fov_deg) / 2.0
    return np.abs(np.arctan2(pts[:, 1], pts[:, 0])) <= half


def to_body(pts: np.ndarray, p: ClusterParams) -> np.ndarray:
    """Sensor frame -> REP-103 body (x fwd, y left, z up).

    Sign flips FIRST, then translation. The offsets are measured in body
    directions, so translating before the flip would move the sensor the wrong
    way along every flipped axis — a 10 cm error that looks like calibration
    drift rather than an ordering bug.
    """
    if pts.shape[0] == 0:
        return pts.reshape(0, 3)
    out = np.empty_like(pts, dtype=np.float64)
    out[:, 0] = pts[:, 0] + p.tx
    out[:, 1] = pts[:, 1] * p.sign_y + p.ty
    out[:, 2] = pts[:, 2] * p.sign_z + p.tz
    return out


def level(pts: np.ndarray, roll: float, pitch: float) -> np.ndarray:
    """Rotate roll and pitch out of a REP-103 body cloud, keeping yaw.

    The result is gravity-aligned and still boat-centred and boat-headed, which
    is what the water gate and the clustering metric want.

    This is `crusader_common.geo.body_to_world_ypr` with yaw=0, vectorised and
    re-expressed in REP-103 axes (that function returns ENU). Written out rather
    than imported because crusader_common carries no numpy dependency and a
    per-point Python call over 20k points a sweep is not affordable. The
    equivalence is pinned by a test rather than asserted here.
    """
    if pts.shape[0] == 0:
        return pts.reshape(0, 3)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    x, y, z = pts[:, 0], pts[:, 1], pts[:, 2]
    out = np.empty_like(pts, dtype=np.float64)
    out[:, 0] = cp * x - sp * sr * y - sp * cr * z
    out[:, 1] = cr * y - sr * z
    out[:, 2] = sp * x + cp * sr * y + cp * cr * z
    return out


def compensate(pts_body: np.ndarray, pose_then, pose_now) -> np.ndarray:
    """Rotate/translate an older sweep's BODY points into the CURRENT body frame.

    pose_*: (world_east, world_north, heading_rad), heading clockwise from north.

    This is what makes multi-sweep accumulation legal. Without it a 2 m/s drift
    smears every cluster over half a metre across a 0.5 s window, and the error
    grows with speed — which reads as a calibration problem rather than a
    missing transform.

    Yaw and translation only. Roll and pitch are removed separately by `level`,
    and across a half-second window their change moves a 20 m return far less
    than the metre-scale translation corrected here.
    """
    if pts_body.shape[0] == 0:
        return pts_body.reshape(0, 3)
    e0, n0, psi0 = pose_then
    e1, n1, psi1 = pose_now
    s, c = math.sin(psi0), math.cos(psi0)
    # old body -> world offset from the OLD origin
    oe = s * pts_body[:, 0] - c * pts_body[:, 1]
    on = c * pts_body[:, 0] + s * pts_body[:, 1]
    # -> offset from the CURRENT origin
    we, wn = (e0 - e1) + oe, (n0 - n1) + on
    s1, c1 = math.sin(psi1), math.cos(psi1)
    out = np.empty_like(pts_body, dtype=np.float64)
    out[:, 0] = c1 * wn + s1 * we
    out[:, 1] = s1 * wn - c1 * we
    out[:, 2] = pts_body[:, 2]
    return out


# ------------------------------------------------------------------ clustering

class _UnionFind:
    __slots__ = ("parent",)

    def __init__(self, n):
        self.parent = list(range(n))

    def find(self, a):
        p = self.parent
        while p[a] != a:
            p[a] = p[p[a]]              # path halving
            a = p[a]
        return a

    def union(self, a, b):
        ra, rb = self.find(a), self.find(b)
        if ra != rb:
            self.parent[ra] = rb


def _offsets(max_rings: int):
    """Neighbourhood offsets as (delta_key, chebyshev_distance), nearest first."""
    out = []
    for dx in range(-max_rings, max_rings + 1):
        for dy in range(-max_rings, max_rings + 1):
            for dz in range(-max_rings, max_rings + 1):
                if dx == dy == dz == 0:
                    continue
                delta = (dx * _VOX_STRIDE + dy) * _VOX_STRIDE + dz
                out.append((delta, max(abs(dx), abs(dy), abs(dz))))
    out.sort(key=lambda t: t[1])
    return out


def dbscan_voxel(pts: np.ndarray, p: ClusterParams):
    """Voxel-grid DBSCAN. Returns an int label per point; -1 is noise.

    Why DBSCAN rather than plain Euclidean cluster extraction: connectivity
    alone lets a thin trail of spray or rain CHAIN a buoy to the shoreline
    behind it, producing one cluster centred on neither, and it turns every
    isolated glint return into its own tiny cluster. Requiring a density before
    a point may seed a cluster kills both, and labels the leftovers noise.

    Why the voxel grid rather than a KD-tree: it makes the neighbour query O(1)
    integer arithmetic with no scipy dependency, and it caps the work per sweep
    regardless of how many points land on one surface.

    Why eps grows with range: LiDAR density falls as 1/r^2 (a 0.3 m buoy returns
    ~37 points at 5 m and ~2 at 20 m). A single global eps tuned to reject water
    noise up close silently becomes a range cutoff further out, and the failure
    looks like "the LiDAR cannot see that buoy" rather than "the parameter is
    wrong". eps(r) = eps_0 * max(1, r / eps_r_ref) holds the density test
    roughly range-invariant instead.

    Density is measured in RAW POINTS, not occupied voxels: voxelising throws
    away exactly the quantity the core test needs, so each voxel carries its
    point count as a weight.
    """
    n = pts.shape[0]
    if n == 0:
        return np.empty(0, dtype=np.int64)

    idx = np.floor(pts / p.leaf_size).astype(np.int64)
    inside = (np.abs(idx) < _VOX_OFFSET).all(axis=1)
    labels = np.full(n, -1, dtype=np.int64)
    if not inside.any():
        return labels                       # everything beyond the key space
    idx, sub = idx[inside], np.flatnonzero(inside)

    keys = ((idx[:, 0] + _VOX_OFFSET) * _VOX_STRIDE
            + (idx[:, 1] + _VOX_OFFSET)) * _VOX_STRIDE + (idx[:, 2] + _VOX_OFFSET)
    ukeys, inverse, counts = np.unique(keys, return_inverse=True,
                                       return_counts=True)
    nv = ukeys.size

    # adaptive eps -> per-voxel ring budget, from each voxel's own mean range
    rng = np.hypot(pts[sub, 0], pts[sub, 1])
    vrange = np.bincount(inverse, weights=rng, minlength=nv) / counts
    eps = p.eps_0 * np.maximum(1.0, vrange / max(p.eps_r_ref, 1e-6))
    rings = np.clip(np.ceil(eps / p.leaf_size), 1, p.max_rings).astype(np.int64)

    offs = _offsets(p.max_rings)

    # --- pass 1: density -> core voxels ---
    dens = counts.astype(np.int64).copy()
    neigh = []                              # cache the lookups; pass 2 reuses them
    for delta, cheb in offs:
        pos = np.searchsorted(ukeys, ukeys + delta)
        np.clip(pos, 0, nv - 1, out=pos)
        hit = ukeys[pos] == (ukeys + delta)
        neigh.append((pos, hit, cheb))
        use = hit & (rings >= cheb)
        dens[use] += counts[pos[use]]
    core = dens >= p.min_density_points
    if not core.any():
        return labels

    # --- pass 2: connect core voxels, then attach borders ---
    uf = _UnionFind(nv)
    for pos, hit, cheb in neigh:
        both = hit & core & core[pos] & (rings >= cheb)
        for i in np.flatnonzero(both):
            uf.union(int(i), int(pos[i]))

    vlabel = np.full(nv, -1, dtype=np.int64)
    for pos, hit, cheb in neigh:
        # a non-core voxel joins a cluster when a CORE voxel's own eps reaches
        # it — DBSCAN's border rule, so the core's ring budget is the one that
        # applies, not the border's
        border = hit & (~core) & core[pos] & (rings[pos] >= cheb) & (vlabel < 0)
        for i in np.flatnonzero(border):
            vlabel[i] = uf.find(int(pos[i]))
    vlabel[core] = [uf.find(int(i)) for i in np.flatnonzero(core)]

    labels[sub] = vlabel[inverse]
    return labels


def clusters_from_labels(pts_body: np.ndarray, labels: np.ndarray,
                         p: ClusterParams):
    """Group body-frame points by label into Clusters, applying size gates."""
    out, n_noise = [], int((labels < 0).sum())
    for lab in np.unique(labels[labels >= 0]):
        sel = pts_body[labels == lab]
        if sel.shape[0] < p.min_points:
            n_noise += sel.shape[0]
            continue
        lo, hi = sel.min(axis=0), sel.max(axis=0)
        ext = hi - lo
        if max(ext[0], ext[1]) > p.max_extent_m:
            # A shoreline, a dock wall or a moored hull. Dropped as an OBJECT
            # because a centroid is meaningless for it — but that makes this the
            # one place the "never lose an obstacle" rule is not free, so the
            # count is reported and a future occupancy grid should take these
            # as extended geometry rather than a point.
            n_noise += sel.shape[0]
            continue
        c = sel.mean(axis=0)
        out.append(Cluster(float(c[0]), float(c[1]), float(c[2]),
                           float(ext[0]), float(ext[1]), float(ext[2]),
                           int(sel.shape[0]),
                           float(math.hypot(c[0], c[1]))))
    out.sort(key=lambda c: c.range_m)       # nearest first: closest obstacle wins
    return out, n_noise


# --------------------------------------------------------------------- driver

def process_body(pts_body: np.ndarray, p: ClusterParams, roll: float = 0.0,
                 pitch: float = 0.0, levelled: bool = True, st=None):
    """BODY-frame cloud -> (clusters, ClusterStats): level, gate, cluster.

    Split out from `process` because the node applies the sensor-frame filters
    and the body transform ONCE PER SWEEP, then accumulates several sweeps and
    clusters the merged window. Re-running those per window would redo the same
    work on the same points N times, and — worse — the sensor-frame filters are
    only meaningful on a sweep still in its own sensor frame, before motion
    compensation moved it.

    `levelled=False` skips the roll/pitch de-rotation, for when attitude is
    stale. Clustering still runs — a stale attitude must not blind the boat —
    but the water gate is then only as good as the boat is flat, and the stat
    says so rather than leaving it to be inferred.
    """
    st = st or ClusterStats(n_in=int(pts_body.shape[0]))
    st.levelled = levelled
    body = np.asarray(pts_body, dtype=np.float64).reshape(-1, 3)
    lvl = level(body, roll, pitch) if levelled else body

    z, r = lvl[:, 2], np.hypot(lvl[:, 0], lvl[:, 1])
    below = z < (p.water_z + p.water_margin)
    above = z > p.z_ceiling
    far = r > p.r_max
    st.n_water = int(below.sum())
    st.n_sky = int((above & ~below).sum())
    st.n_far = int((far & ~below & ~above).sum())
    keep = ~(below | above | far)
    lvl, body = lvl[keep], body[keep]

    labels = dbscan_voxel(lvl, p)
    clusters, n_noise = clusters_from_labels(body, labels, p)
    st.n_noise = n_noise
    st.n_clustered = int(sum(c.n_points for c in clusters))
    st.n_clusters = len(clusters)
    return clusters, st


def process(pts_sensor: np.ndarray, p: ClusterParams, roll: float = 0.0,
            pitch: float = 0.0, levelled: bool = True):
    """Raw sensor cloud -> (clusters, ClusterStats). The whole pipeline.

    Used by the tests and by anything processing one sweep on its own; the node
    takes the split path (sensor filters per sweep, process_body per window).
    """
    st = ClusterStats(n_in=int(pts_sensor.shape[0]), levelled=levelled)
    pts = np.asarray(pts_sensor, dtype=np.float64).reshape(-1, 3)

    m = near_mask(pts, p.r_min)
    st.n_near = int((~m).sum())
    pts = pts[m]

    m = fov_mask(pts, p.fov_deg)
    st.n_fov = int((~m).sum())
    pts = pts[m]

    return process_body(to_body(pts, p), p, roll, pitch, levelled, st)


def params_from(d: dict) -> ClusterParams:
    """Build ClusterParams from a flat {name: value} dict — the node's live ROS
    params, or a crusader_params.yaml section.

    Raises KeyError on a missing key rather than falling back to a code default:
    a parameter the config forgot must fail at startup, loudly, not drift
    silently away from the file that is supposed to be the single source of
    truth.
    """
    return ClusterParams(
        sign_y=d["lidar_sign_y"], sign_z=d["lidar_sign_z"],
        tx=d["lidar_x"], ty=d["lidar_y"], tz=d["lidar_z"],
        r_min=d["r_min"], fov_deg=d["fov_deg"],
        water_z=d["water_z"], water_margin=d["water_margin"],
        z_ceiling=d["z_ceiling"], r_max=d["r_max"],
        leaf_size=d["leaf_size"], eps_0=d["eps_0"], eps_r_ref=d["eps_r_ref"],
        max_rings=int(d["max_rings"]),
        min_density_points=int(d["min_density_points"]),
        min_points=int(d["min_points"]), max_extent_m=d["max_extent_m"])
