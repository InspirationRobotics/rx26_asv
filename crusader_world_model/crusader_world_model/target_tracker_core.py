"""target_tracker_core — sensor detections in, earth-anchored target tracks out.

NOTE: unverified on the boat. Written off-boat against invented detections
(tools/bench/bench_world_model.py drives the whole chain); no part of this has
seen real water. It is deliberately NOT in core.launch.py.

No ROS imports at all — this is the half of the world model that can be run on
a laptop with no camera, no LiDAR, no GPU and no boat, which is the entire
reason crusader_world_model is a package separate from crusader_perception.
crusader_common.geo is the only import with any geometry in it, and it is
imported rather than re-derived.

THE THREE STAGES, and why they are separate.

1. FUSE (body frame). A camera detection and a LiDAR cluster of the same object
   arrive in different messages, in different frames, at different rates. The
   camera knows WHERE TO LOOK — its bearing is an angle read off a rectified
   image and is good to a fraction of a degree — and knows WHAT the thing is.
   The LiDAR knows HOW FAR — a time-of-flight range, centimetres, against the
   camera's stereo disparity which is metres out past 15 m. Fusion is therefore
   not an average: it keeps the camera's DIRECTION and the LiDAR's DISTANCE,
   and throws away each sensor's weak number.

   Both inputs are body-frame already (Cluster3DArray is base_link;
   Detection3DArray is camera_link, which is the same hull with a fixed offset
   applied here), so this stage is pure trigonometry with no attitude in it.

   The package's standing safety invariant applies HERE: a camera detection
   with no LiDAR support is passed through with its stereo range, NEVER
   dropped. A coarse range for a real buoy beats no buoy. The same goes the
   other way — an unlabelled LiDAR cluster is a thing that is THERE, and it is
   published with an empty label rather than discarded for being anonymous.

2. PROJECT (body -> world). One call to geo.body_to_world_ypr per observation,
   with the boat's roll, pitch, yaw and position. This is the ONLY place
   attitude enters. On a surface vessel in chop this is not a refinement: at
   20 m, 5 degrees of uncompensated roll moves a target 1.7 m, which is most of
   a buoy gate.

3. TRACK (world frame). Associate each projected observation with an existing
   track by nearest neighbour, update it, and let unseen tracks decay. This is
   what turns a stream of sightings into an object that persists while the boat
   looks away — the thing a mission actually needs.

WHY NEAREST-NEIGHBOUR AND AN EMA, NOT A KALMAN FILTER. The targets are mostly
STATIC objects being observed from a moving platform whose own position is RTK
GPS. The dominant error is not process noise to be modelled, it is
misassociation — feeding one track sightings of two different buoys — and no
amount of filter sophistication fixes a wrong association. So the effort goes
into the gate (a hard radius plus label compatibility) and the residual
(position_stddev, which grows loudly when the gate is wrong) rather than into
covariance propagation. An EMA whose gain decays as 1/hits IS the running mean
for the first few sightings and a tracking filter after that, which is the
right behaviour for a buoy that is not going anywhere.
"""
import math
from dataclasses import dataclass, field

from crusader_common import geo

SOURCE_CAMERA = 1
SOURCE_LIDAR = 2

# Smoothing constant for the association-residual estimate that becomes
# TrackedTarget.position_stddev. Not a tunable: it is a diagnostic, and a
# diagnostic whose responsiveness moves with a tuning knob cannot be compared
# between two runs. 0.2 settles in roughly five sightings, which is the same
# scale as confirm_hits — a track reports a meaningful spread at about the
# moment it becomes CONFIRMED.
_VAR_ALPHA = 0.2


# --------------------------------------------------------------- parameters

@dataclass
class TrackerParams:
    """Everything tunable, in one object the node fills from the YAML.

    Grouped the way the three stages are: mount geometry, then fusion gates,
    then tracking gates. A default here is a STARTING POINT for the bench, not
    a blessed value — crusader_bringup/config/crusader_params.yaml is the
    single source of truth, and the node passes those in.
    """

    # -- camera extrinsic: camera_link -> base_link (REP-103: x fwd, y left, z up)
    cam_x: float = 0.0           # m forward of the body origin
    cam_y: float = 0.0           # m to PORT of the centreline
    cam_z: float = 0.0           # m above the hull-bottom datum
    cam_yaw_deg: float = 0.0     # mount yaw, + = aimed to PORT
    cam_pitch_deg: float = 0.0   # mount tilt, + = aimed DOWN

    # -- fusion gates (stage 1)
    fuse_bearing_deg: float = 6.0    # max camera/LiDAR bearing disagreement
    fuse_range_m: float = 4.0        # absolute range-agreement gate
    fuse_range_frac: float = 0.25    # ... or this fraction of the camera range,
                                     # whichever is larger. See _range_gate.
    min_confidence: float = 0.50     # re-threshold above buoy_detector's own
    track_unlabeled: bool = True     # keep LiDAR-only clusters as tracks

    # -- tracking gates (stage 3)
    assoc_radius_m: float = 3.0      # world-frame association radius
    pos_alpha: float = 0.30          # floor on the position EMA gain
    vel_alpha: float = 0.20          # velocity EMA gain
    confirm_hits: int = 3            # sightings before a track is CONFIRMED
    tentative_timeout_s: float = 3.0  # unconfirmed tracks expire fast
    track_timeout_s: float = 30.0    # confirmed tracks are held much longer
    max_tracks: int = 64             # hard cap; see _prune


# ------------------------------------------------------------------ inputs

@dataclass
class CameraDetection:
    """One crusader_msgs/Detection3D, in camera_link (REP-103 body axes)."""
    x: float
    y: float
    z: float
    label: str
    confidence: float


@dataclass
class LidarCluster:
    """One crusader_msgs/Cluster3D, in base_link (REP-103 body axes)."""
    x: float
    y: float
    z: float
    extent: tuple = (0.0, 0.0, 0.0)
    n_points: int = 0


@dataclass
class BoatState:
    """Where the boat is and how it is oriented, at one instant.

    east/north are metres in the world frame (geo.latlon_to_xy against the
    tracker's origin). roll/pitch/yaw are radians straight off /crsd/attitude,
    in the autopilot's own NED body axes — passed through unconverted, because
    geo.body_to_world_ypr is documented to take them that way and converting
    them here would put a second frame convention in a second place.
    """
    east: float
    north: float
    roll: float
    pitch: float
    yaw: float


@dataclass
class Observation:
    """A sighting after fusion: body frame (base_link), ready to project."""
    x: float
    y: float
    z: float
    label: str
    confidence: float
    sources: int
    extent: tuple = (0.0, 0.0, 0.0)

    @property
    def range_h(self) -> float:
        """Horizontal range [m] — the number both gates and sorting use."""
        return math.hypot(self.x, self.y)


# ----------------------------------------------------------- stage 1: fuse

def camera_to_body(x_fwd: float, y_left: float, z_up: float,
                   p: TrackerParams) -> tuple:
    """Rotate and translate a camera_link point into base_link.

    Both frames are REP-103 (x forward, y left, z up); what separates them is
    where the OAK-D is bolted and which way it is aimed. Order is tilt first,
    then yaw, then the translation — the mount rotates the camera about its own
    axes before the offset places it on the hull, and doing the translation
    first would rotate the offset too.

    Args:
      x_fwd, y_left, z_up: the Detection3D position [m], in camera_link.
      p: TrackerParams carrying cam_x/y/z and the two mount angles.

    Returns:
      (x, y, z) in base_link [m].

    The sign conventions are spelled out because they are the kind of thing
    that is wrong for a whole field season: cam_pitch_deg is POSITIVE DOWN
    (cameras get tilted down at the water, so the common case is the positive
    one) and cam_yaw_deg is POSITIVE TO PORT (matching y_left being positive).
    """
    t = math.radians(p.cam_pitch_deg)
    ct, st = math.cos(t), math.sin(t)
    # Tilt about the +y (left) axis. Check it on the two easy rays: a ray
    # straight ahead in the camera (1, 0, 0) comes out (cos t, 0, -sin t) —
    # pointing down for a positive (downward) tilt, as it must.
    x1 = x_fwd * ct + z_up * st
    z1 = -x_fwd * st + z_up * ct

    a = math.radians(p.cam_yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    # Yaw about the +z (up) axis: (1, 0) -> (cos a, sin a), i.e. swung to port
    # for a positive angle.
    x2 = x1 * ca - y_left * sa
    y2 = x1 * sa + y_left * ca

    return x2 + p.cam_x, y2 + p.cam_y, z1 + p.cam_z


def body_to_camera(x_fwd: float, y_left: float, z_up: float,
                   p: TrackerParams) -> tuple:
    """The exact inverse of camera_to_body — base_link point to camera_link.

    The tracker never calls this; the bench harness does, to work out what the
    camera WOULD have reported for a buoy it has placed in the world. It lives
    here beside its forward twin rather than in the bench for the reason
    tools/lidar_view.py reads the LiDAR extrinsic out of the params file: a
    checking tool that models the geometry differently from the node under test
    is worse than no check, because it fails and passes for reasons that have
    nothing to do with the thing being checked.

    Args:
      x_fwd, y_left, z_up: the point in base_link [m].
      p: TrackerParams carrying cam_x/y/z and the two mount angles.

    Returns:
      (x, y, z) in camera_link [m].

    Undoes the forward operations in reverse order — translate, then yaw, then
    tilt — because that is what an inverse is. Composing them in the same order
    as the forward pass is wrong the moment either angle is non-zero, which is
    exactly when the bench stops being able to find the error.
    """
    x2, y2, z1 = x_fwd - p.cam_x, y_left - p.cam_y, z_up - p.cam_z

    a = math.radians(p.cam_yaw_deg)
    ca, sa = math.cos(a), math.sin(a)
    x1 = x2 * ca + y2 * sa
    y = -x2 * sa + y2 * ca

    t = math.radians(p.cam_pitch_deg)
    ct, st = math.cos(t), math.sin(t)
    return x1 * ct - z1 * st, y, x1 * st + z1 * ct


def bearing_deg(x: float, y: float) -> float:
    """Body-frame bearing of a point: 0 = dead ahead, + = to PORT [deg]."""
    return math.degrees(math.atan2(y, x))


def angle_diff_deg(a: float, b: float) -> float:
    """Absolute difference between two bearings, wrapped to [0, 180]."""
    return abs(math.degrees(geo.wrap_pi(math.radians(a - b))))


def _range_gate(cam_range: float, p: TrackerParams) -> float:
    """How far apart the two ranges may be before the pair is rejected [m].

    A single absolute tolerance cannot work across this sensor's useful span.
    Close in, stereo depth is good to centimetres and a 4 m tolerance would
    happily pair a buoy at 3 m with a dock at 7 m. Far out, stereo error grows
    with the square of range, and the same 4 m would reject the correct pairing
    at 22 m. So the gate is the LARGER of a fixed floor and a fraction of the
    camera's own range: tight where the camera is trustworthy, loose where it
    is not, with one line of arithmetic instead of an error model.
    """
    return max(p.fuse_range_m, p.fuse_range_frac * cam_range)


def fuse(cameras, lidars, p: TrackerParams):
    """Pair camera detections with LiDAR clusters and emit body-frame sightings.

    Args:
      cameras: list[CameraDetection], in camera_link.
      lidars: list[LidarCluster], in base_link.
      p: TrackerParams.

    Returns:
      (observations, stats) — observations is list[Observation] in base_link;
      stats is a dict of counts for the health topic.

    Association is GREEDY, nearest camera detection first. Greedy is the right
    trade here: the alternative (a global assignment) buys accuracy only when
    the gates are wide enough for one cluster to be plausible for several
    detections at once, and if that is happening the gates are wrong. Taking
    the near targets first means that when the gates ARE too wide, the object
    that gets the good range is the close one — which is the one about to be
    hit.

    Nothing is discarded for failing to pair. Every camera detection above
    min_confidence and (if track_unlabeled) every cluster comes out the other
    side; the only thing pairing changes is which range the sighting carries.
    """
    stats = {"cam_in": len(cameras), "lidar_in": len(lidars),
             "cam_low_conf": 0, "fused": 0, "cam_only": 0, "lidar_only": 0}

    # Move the camera onto the hull first, so every bearing below is measured
    # about the same origin the LiDAR's already is. Skipping this puts the
    # whole fusion stage out by the mount offset — a constant, invisible error
    # that looks exactly like a miscalibrated extrinsic.
    cam_obs = []
    for d in cameras:
        if d.confidence < p.min_confidence:
            stats["cam_low_conf"] += 1
            continue
        x, y, z = camera_to_body(d.x, d.y, d.z, p)
        cam_obs.append(Observation(x, y, z, d.label, d.confidence,
                                   SOURCE_CAMERA))
    cam_obs.sort(key=lambda o: o.range_h)

    taken = set()
    out = []
    for obs in cam_obs:
        cam_b = bearing_deg(obs.x, obs.y)
        cam_r = obs.range_h
        gate_r = _range_gate(cam_r, p)

        best, best_da = None, p.fuse_bearing_deg
        for i, c in enumerate(lidars):
            if i in taken:
                continue
            da = angle_diff_deg(cam_b, bearing_deg(c.x, c.y))
            if da >= best_da:
                continue
            if abs(math.hypot(c.x, c.y) - cam_r) > gate_r:
                continue          # same bearing, different object
            best, best_da = i, da

        if best is None:
            out.append(obs)                     # safety invariant: never dropped
            stats["cam_only"] += 1
            continue

        taken.add(best)
        out.append(_fused(obs, lidars[best]))
        stats["fused"] += 1

    if p.track_unlabeled:
        for i, c in enumerate(lidars):
            if i in taken:
                continue
            out.append(Observation(c.x, c.y, c.z, "", 0.0, SOURCE_LIDAR,
                                   tuple(c.extent)))
            stats["lidar_only"] += 1

    return out, stats


def _fused(cam: Observation, lid: LidarCluster) -> Observation:
    """Camera DIRECTION at LiDAR RANGE — the whole point of fusing the two.

    The camera's (x, y, z) is treated as a unit direction and rescaled so its
    horizontal component equals the cluster's horizontal range. Bearing and
    elevation survive untouched; only the distance along the ray changes.

    Scaling the WHOLE vector rather than replacing x and y is what keeps z
    consistent with the new range: a target re-ranged from 8 m to 14 m without
    its z scaled would appear to sink, and z is the sanity check this package
    relies on to notice a bad attitude.

    The cluster's own centroid is not used as the position. It is the centroid
    of whatever surface faced the LiDAR, biased toward the near side and
    wandering as the boat circles; the camera's bearing to the object does not
    have that bias.
    """
    cam_r = cam.range_h
    lid_r = math.hypot(lid.x, lid.y)
    # A detection dead ahead at zero horizontal range is degenerate — the ray
    # has no horizontal direction to preserve, so there is nothing to scale.
    # Fall back to the cluster, which at least has a position.
    if cam_r < 1e-6:
        return Observation(lid.x, lid.y, lid.z, cam.label, cam.confidence,
                           SOURCE_CAMERA | SOURCE_LIDAR, tuple(lid.extent))
    k = lid_r / cam_r
    return Observation(cam.x * k, cam.y * k, cam.z * k,
                       cam.label, cam.confidence,
                       SOURCE_CAMERA | SOURCE_LIDAR, tuple(lid.extent))


# -------------------------------------------------------------- the tracks

@dataclass
class Track:
    """One earth-anchored object and everything known about it.

    Position is world ENU metres against the tracker's origin; the node turns
    it into lat/lon on the way out. `label_votes` is kept rather than a single
    label so the reported name is a majority over the track's whole life —
    see TrackedTarget.msg on why the most recent label is the wrong answer.
    """
    id: int
    x: float
    y: float
    z: float
    label_votes: dict = field(default_factory=dict)
    confidence: float = 0.0
    sources: int = 0
    extent: tuple = (0.0, 0.0, 0.0)
    hits: int = 0
    first_seen: float = 0.0
    last_seen: float = 0.0
    vel_e: float = 0.0
    vel_n: float = 0.0
    var: float = 0.0                 # EMA of squared association residual [m^2]
    # Snapshot of the boat-relative geometry at the last update. Recomputed on
    # every publish against the current pose, so these are only a fallback for
    # a track that was not updated this cycle.
    range_h: float = 0.0
    bearing_true: float = 0.0

    @property
    def label(self) -> str:
        """Majority label, or "" if only the LiDAR has ever seen this."""
        if not self.label_votes:
            return ""
        return max(self.label_votes.items(), key=lambda kv: kv[1])[0]

    def confirmed(self, p: TrackerParams) -> bool:
        return self.hits >= p.confirm_hits

    @property
    def stddev(self) -> float:
        return math.sqrt(self.var)


def _label_ok(track: Track, obs: Observation) -> bool:
    """May this sighting be associated with this track?

    Compatible means: same label, or one of the two has no label at all. The
    empty case is not a loophole, it is the point — it is how a LiDAR-only
    track picks up a name the first time the camera sees it, and how a
    camera-only track picks up a footprint the first time the LiDAR does.
    Requiring an exact match would keep the two halves of one object apart
    forever, which is exactly the outcome fusion exists to prevent.

    Two DIFFERENT labels never merge, however close: a red buoy and a green
    buoy 2 m apart are a gate, and a tracker that averaged them into one
    object at the midpoint would put a waypoint through the middle of nothing.
    """
    tl, ol = track.label, obs.label
    return not tl or not ol or tl == ol


class TargetTracker:
    """Stage 3: the track set, and the only mutable state in this module.

    Not a ROS node and not thread-safe — the node calls update() from one timer
    callback and reads the result on the same thread. Time is a caller-supplied
    monotonic float for the same reason StreamCache takes one: tests drive it
    deterministically and nothing here reads a clock behind your back.
    """

    def __init__(self, params: TrackerParams):
        self.p = params
        self.tracks = []
        self._next_id = 1
        self.stats = {}

    # ---- the cycle ----

    def update(self, cameras, lidars, boat: BoatState, now: float):
        """One fuse -> project -> associate -> decay cycle.

        Args:
          cameras: list[CameraDetection] in camera_link (may be empty).
          lidars: list[LidarCluster] in base_link (may be empty).
          boat: BoatState — position AND attitude for this instant.
          now: monotonic seconds.

        Returns:
          list[Track], sorted nearest-first by horizontal range from the boat.

        An empty sensor cycle is a legitimate call and does real work: tracks
        still age, and tracks past their timeout are still dropped. Skipping
        the call when nothing was detected is how a stale track set outlives
        the boat leaving the area.
        """
        obs, stats = fuse(cameras, lidars, self.p)

        projected = []
        for o in obs:
            wx, wy, wz = geo.body_to_world_ypr(
                o.x, o.y, o.z, boat.roll, boat.pitch, boat.yaw,
                boat.east, boat.north)
            projected.append((o, wx, wy, wz))

        stats["associated"] = 0
        stats["new_tracks"] = 0
        self._associate(projected, now, stats)

        stats["expired"] = self._prune(now)
        stats["tracks"] = len(self.tracks)
        stats["confirmed"] = sum(1 for t in self.tracks if t.confirmed(self.p))
        self.stats = stats
        return self.snapshot(boat)

    def snapshot(self, boat: BoatState):
        """Tracks with range/bearing refreshed against `boat`, nearest first.

        Split out from update() so the node can re-range the same track set
        against a newer pose without inventing sensor observations to do it.
        """
        for t in self.tracks:
            de, dn = t.x - boat.east, t.y - boat.north
            t.range_h = math.hypot(de, dn)
            # Absolute bearing in the LatLonHead convention: 0 = true north,
            # clockwise positive. atan2(east, north) — the arguments are in
            # that order on purpose, and swapping them mirrors the map.
            t.bearing_true = math.degrees(math.atan2(de, dn)) % 360.0
        return sorted(self.tracks, key=lambda t: t.range_h)

    # ---- association ----

    def _associate(self, projected, now, stats):
        """Greedy nearest-neighbour, one observation to at most one track.

        Candidate pairs are scored by world-frame distance, gated by
        assoc_radius_m and label compatibility, then consumed best-first. This
        is O(obs x tracks) with both bounded in the tens, so the simple version
        is also the fast one.

        Best-first rather than in-arrival-order matters when two targets sit
        inside one radius of each other: taking the closest pairing first gives
        each observation to the track it actually fits, where a first-come
        sweep would hand the first observation to whichever track it happened
        to reach and strand the second.
        """
        pairs = []
        for oi, (o, wx, wy, wz) in enumerate(projected):
            for ti, t in enumerate(self.tracks):
                if not _label_ok(t, o):
                    continue
                d = math.hypot(wx - t.x, wy - t.y)
                if d <= self.p.assoc_radius_m:
                    pairs.append((d, oi, ti))
        pairs.sort()

        used_obs, used_tracks = set(), set()
        for d, oi, ti in pairs:
            if oi in used_obs or ti in used_tracks:
                continue
            used_obs.add(oi)
            used_tracks.add(ti)
            o, wx, wy, wz = projected[oi]
            self._update_track(self.tracks[ti], o, wx, wy, wz, d, now)
            stats["associated"] += 1

        for oi, (o, wx, wy, wz) in enumerate(projected):
            if oi in used_obs:
                continue
            self._new_track(o, wx, wy, wz, now)
            stats["new_tracks"] += 1

    def _update_track(self, t: Track, o: Observation,
                      wx: float, wy: float, wz: float, resid: float, now: float):
        """Blend one sighting into an existing track.

        The position gain is max(pos_alpha, 1/hits): for the first few
        sightings that IS the running mean, so a new track converges on its
        true position immediately instead of crawling there at 30% per frame;
        after about 1/pos_alpha sightings it flattens into a fixed-gain EMA
        that keeps responding to a target that moves. One expression covers
        both regimes and there is no mode to get stuck in the wrong one.
        """
        dt = max(now - t.last_seen, 1e-3)
        t.hits += 1
        a = max(self.p.pos_alpha, 1.0 / t.hits)

        nx = t.x + a * (wx - t.x)
        ny = t.y + a * (wy - t.y)
        # Velocity from the SMOOTHED positions, not the raw sighting: a finite
        # difference of raw observations is mostly detector noise divided by a
        # small dt, which at 10 Hz reads as several m/s for a moored buoy.
        self._blend_velocity(t, (nx - t.x) / dt, (ny - t.y) / dt)
        t.x, t.y = nx, ny
        t.z += a * (wz - t.z)

        # Residual is measured against the estimate BEFORE this update — it is
        # the innovation, "how wrong were we", not the post-fit leftover, which
        # would shrink toward zero whatever the data did.
        t.var += _VAR_ALPHA * (resid * resid - t.var)

        t.last_seen = now
        t.sources |= o.sources
        if o.label:
            t.label_votes[o.label] = t.label_votes.get(o.label, 0) + 1
            t.confidence = o.confidence
        if o.sources & SOURCE_LIDAR and any(o.extent):
            t.extent = o.extent

    def _blend_velocity(self, t: Track, ve: float, vn: float):
        """EMA the world velocity. See TrackedTarget.msg on what it is for."""
        b = self.p.vel_alpha
        t.vel_e += b * (ve - t.vel_e)
        t.vel_n += b * (vn - t.vel_n)

    def _new_track(self, o: Observation, wx: float, wy: float, wz: float,
                   now: float):
        """Birth. Tentative until confirm_hits sightings agree it is real."""
        t = Track(id=self._next_id, x=wx, y=wy, z=wz,
                  first_seen=now, last_seen=now, hits=1,
                  sources=o.sources, confidence=o.confidence,
                  extent=tuple(o.extent))
        if o.label:
            t.label_votes[o.label] = 1
        self._next_id += 1           # never reused, even after the track dies
        self.tracks.append(t)

    # ---- decay ----

    def _prune(self, now: float) -> int:
        """Drop expired tracks and enforce the cap. Returns how many went.

        Two timeouts, because the two kinds of track are wrong in different
        ways. A TENTATIVE track is very often one frame of detector noise, and
        holding that on the map for half a minute puts phantom obstacles in
        front of a mission. A CONFIRMED track has been seen repeatedly and is
        almost certainly a real object that the boat has simply turned away
        from — dropping it because it left the camera's 80 degree field is how
        the boat forgets the buoy it just rounded.

        The cap is a last-resort guard against a runaway (a shoreline
        clustering into hundreds of anonymous objects), not a normal path: the
        tracks kept are the confirmed ones, then the most recently seen. It is
        logged by the caller through the health topic because silently
        forgetting an obstacle is exactly the failure this package's README
        forbids.
        """
        before = len(self.tracks)
        keep = []
        for t in self.tracks:
            timeout = (self.p.track_timeout_s if t.confirmed(self.p)
                       else self.p.tentative_timeout_s)
            if now - t.last_seen <= timeout:
                keep.append(t)
        self.tracks = keep

        if len(self.tracks) > self.p.max_tracks:
            self.tracks.sort(key=lambda t: (t.confirmed(self.p), t.last_seen),
                             reverse=True)
            self.tracks = self.tracks[:self.p.max_tracks]

        return before - len(self.tracks)
