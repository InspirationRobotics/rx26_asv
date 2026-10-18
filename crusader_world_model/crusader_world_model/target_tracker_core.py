"""target_tracker_core — sensor detections in, earth-anchored target tracks out.

NOTE: unverified on the boat. Written off-boat against invented detections
(tools/bench/bench_world_model.py drives the whole chain); no part of this has
seen real water. It is deliberately NOT in core.launch.py.

No ROS imports at all: this half of the world model runs on a laptop with no
camera, LiDAR, GPU or boat, which is why crusader_world_model is separate from
crusader_perception. crusader_common.geo is the only import with geometry in it.

THE THREE STAGES, and why they are separate.

1. FUSE (body frame). The camera knows WHERE TO LOOK (a bearing good to a fraction
   of a degree) and WHAT the thing is; the LiDAR knows HOW FAR (centimetres, against
   stereo disparity that is metres out past 15 m). Fusion keeps the camera's
   DIRECTION and the LiDAR's DISTANCE. Both inputs are body-frame already
   (Cluster3DArray is base_link; Detection3DArray is camera_link with a fixed offset
   applied here), so this stage is pure trigonometry with no attitude in it.

   USE_LIDAR GATES THIS ENTIRE STAGE, and it defaults to FALSE: near walls every
   hard surface clusters unlabelled and becomes a PERMANENT map object a mission
   cannot tell from a buoy (the pool map filled with 100+ permanent tracks at r_max
   40 m; see crusader_params.yaml). With it False only the camera can create a
   track, with stereo ranges; turn it on for open water.

   The package's standing safety invariant applies HERE: a camera detection with no
   LiDAR support is passed through with its stereo range, NEVER dropped, and an
   unlabelled LiDAR cluster is published with an empty label rather than discarded.

2. PROJECT (body -> world). One call to geo.body_to_world_ypr per observation. The
   ONLY place attitude enters: at 20 m, 5 degrees of uncompensated roll moves a
   target 1.7 m, most of a buoy gate.

3. TRACK (world frame). Associate each observation with an existing track by
   nearest neighbour, update it, and let unseen tracks decay, so a target persists
   while the boat looks away.

WHY NEAREST-NEIGHBOUR AND AN EMA, NOT A KALMAN FILTER. The targets are mostly
STATIC and the platform has RTK GPS, so the dominant error is misassociation, which
no filter fixes: the effort goes into the gate (hard radius plus label
compatibility) and the residual (position_stddev, which grows loudly when the gate
is wrong). An EMA whose gain decays as 1/hits IS the running mean for the first
sightings and a tracking filter after that.

ONE BUOY, MANY COLOURS. A RoboBuoy's beacon changes state (lit red/green, flashing
or steady blue, dark) and the detector names the STATE, so one buoy arrives as
red_buoy then black_buoy; keyed on the label that is two tracks a metre apart. So
the labels in `colour_family_labels` are compatible at association, and the track's
reported colour is a VOTE (resolve_label). A buoy of unknown colour is STILL a track
(an obstacle is an obstacle; the UAV's field supplies the colour).
"""
import math
from dataclasses import dataclass, field

from crusader_common import geo

SOURCE_CAMERA = 1
SOURCE_LIDAR = 2

# Smoothing constant for the association-residual estimate behind
# TrackedTarget.position_stddev. Not a tunable: a diagnostic whose responsiveness
# moves with a knob cannot be compared between runs. 0.2 settles in ~five
# sightings, about when a track becomes CONFIRMED.
_VAR_ALPHA = 0.2


# --------------------------------------------------------------- parameters

@dataclass
class TrackerParams:
    """Everything tunable, in one object the node fills from the YAML. A default
    here is a STARTING POINT: crusader_bringup/config/crusader_params.yaml is the
    single source of truth.
    """

    # -- camera extrinsic: camera_link -> base_link (REP-103: x fwd, y left, z up)
    cam_x: float = 0.0           # m forward of the body origin
    cam_y: float = 0.0           # m to PORT of the centreline
    cam_z: float = 0.0           # m above the hull-bottom datum
    cam_yaw_deg: float = 0.0     # mount yaw, + = aimed to PORT
    cam_pitch_deg: float = 0.0   # mount tilt, + = aimed DOWN

    # -- fusion gates (stage 1)
    use_lidar: bool = False          # False: clusters are counted, then dropped
                                     # before stage 1 runs. See fuse().
    fuse_bearing_deg: float = 6.0    # max camera/LiDAR bearing disagreement
    fuse_range_m: float = 4.0        # absolute range-agreement gate
    fuse_range_frac: float = 0.25    # ... or this fraction of the camera range,
                                     # whichever is larger. See _range_gate.
    min_confidence: float = 0.50     # re-threshold above buoy_detector's own
    track_unlabeled: bool = True     # keep LiDAR-only clusters as tracks.
                                     # Inert while use_lidar is False: there
                                     # are no LiDAR-only sightings to keep.

    # -- tracking gates (stage 3)
    assoc_radius_m: float = 3.0      # world-frame association radius
    pos_alpha: float = 0.30          # floor on the position EMA gain
    vel_alpha: float = 0.20          # velocity EMA gain
    confirm_hits: int = 3            # sightings before a track is CONFIRMED
    tentative_timeout_s: float = 3.0  # unconfirmed tracks expire fast; ALWAYS
    track_timeout_s: float = 0.0     # confirmed: 0 = never expire. See _prune
    max_tracks: int = 64             # hard cap; see _prune

    # -- colour voting (stage 3). See the module docstring, "ONE BUOY, MANY
    # COLOURS". Tuples, because the node hands in whatever list rclpy gives it
    # and membership is the only thing ever asked of them.
    colour_vote_enable: bool = True  # False = the pre-voting behaviour: same
                                     # label or one side empty to associate,
                                     # plain majority to report.
    colour_family_labels: tuple = ("red_buoy", "green_buoy", "black_buoy",
                                   "flashing_blue_buoy", "steady_blue_buoy",
                                   "blue_buoy")
                                     # ONE object type in different beacon
                                     # states: associate with each other
    colour_unlit_labels: tuple = ("black_buoy",)
                                     # family votes that carry NO colour
    colour_unknown_label: str = "unknown_buoy"
                                     # reported when no lit colour wins. Must
                                     # contain no colour word: bt_runner's
                                     # beaconFromLabel() matches substrings
    colour_family_shapes: tuple = ("diamond",)
                                     # oak_detector's vocabulary is
                                     # [flash_|off_]<colour>_<shape> or a bare <shape>:
                                     # every label ending in _<shape>, and <shape> itself,
                                     # is one family per shape; bare and off_ labels are
                                     # its unlit votes, and its unknown label is unknown_<shape>.
    colour_min_votes: int = 5        # a lit colour needs at least this many
    colour_min_ratio: float = 0.6    # ... and this share of the LIT votes


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

    east/north: metres in the world frame (geo.latlon_to_xy against the tracker's
    origin). roll/pitch/yaw: radians straight off /crsd/attitude in the autopilot's
    NED body axes, unconverted because geo.body_to_world_ypr takes them that way.
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

    Both frames are REP-103 (x forward, y left, z up). Order is tilt, then yaw, then
    translation (the mount rotates the camera before the offset places it on the hull).

    Args:
      x_fwd, y_left, z_up: the Detection3D position [m], in camera_link.
      p: TrackerParams carrying cam_x/y/z and the two mount angles.

    Returns:
      (x, y, z) in base_link [m].

    Signs: cam_pitch_deg is POSITIVE DOWN and cam_yaw_deg is POSITIVE TO PORT
    (matching y_left).
    """
    t = math.radians(p.cam_pitch_deg)
    ct, st = math.cos(t), math.sin(t)
    # Tilt about the +y (left) axis: a ray straight ahead (1, 0, 0) comes out
    # (cos t, 0, -sin t), pointing down for a positive tilt.
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

    The tracker never calls this; the bench harness does, to work out what the camera
    WOULD have reported for a buoy placed in the world, so the checking tool shares the
    node's geometry.

    Args:
      x_fwd, y_left, z_up: the point in base_link [m].
      p: TrackerParams carrying cam_x/y/z and the two mount angles.

    Returns:
      (x, y, z) in camera_link [m].

    Undoes the forward operations in reverse order: translate, yaw, then tilt.
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

    A single absolute tolerance cannot span the sensor's range (4 m would pair a buoy
    at 3 m with a dock at 7 m, yet reject the right pairing at 22 m where stereo error
    grows with range squared), so the gate is the LARGER of a fixed floor and a
    fraction of the camera's own range.
    """
    return max(p.fuse_range_m, p.fuse_range_frac * cam_range)


def fuse(cameras, lidars, p: TrackerParams):
    """Pair camera detections with LiDAR clusters and emit body-frame sightings.

    Args:
      cameras: list[CameraDetection], in camera_link.
      lidars: list[LidarCluster], in base_link. Counted and then DISCARDED
        when p.use_lidar is False, which is the default — see the module
        docstring for why, and stats['lidar_ignored'] for how to see it.
      p: TrackerParams.

    Returns:
      (observations, stats) — observations is list[Observation] in base_link;
      stats is a dict of counts for the health topic.

    Association is GREEDY, nearest camera detection first: a global assignment buys
    accuracy only if the gates are wide enough for one cluster to suit several
    detections, and then the gates are wrong; nearest-first gives the good range to
    the close object, the one about to be hit.

    Nothing is discarded for FAILING TO PAIR: every camera detection above
    min_confidence and (if use_lidar and track_unlabeled) every cluster comes out;
    pairing only changes which range a sighting carries. Discarding the LiDAR on
    use_lidar is a separate decision taken once at the top and stated in the stats.
    """
    stats = {"cam_in": len(cameras), "lidar_in": len(lidars),
             "cam_low_conf": 0, "fused": 0, "cam_only": 0, "lidar_only": 0,
             "lidar_ignored": 0}

    if not p.use_lidar:
        # Counted FIRST, then dropped: "lidar_in 10, lidar_ignored 10" is a
        # configuration and "lidar_in 0" is a dead sensor, and one health topic
        # must tell them apart. Zeroing the list keeps ONE place where the switch
        # is read.
        stats["lidar_ignored"] = len(lidars)
        lidars = []

    # Move the camera onto the hull first, so every bearing is measured about the
    # same origin as the LiDAR's; skipping it puts fusion out by the mount offset,
    # which looks like a miscalibrated extrinsic.
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

    The camera's (x, y, z) is rescaled so its horizontal component equals the
    cluster's horizontal range: bearing and elevation survive, only the distance along
    the ray changes (scaling z too keeps it consistent, and z is the sanity check for
    a bad attitude). The cluster's centroid is not used: it is biased to the near side
    of whatever surface faced the LiDAR and wanders as the boat circles.
    """
    cam_r = cam.range_h
    lid_r = math.hypot(lid.x, lid.y)
    # Dead ahead at zero horizontal range has no direction to scale; fall back to
    # the cluster, which at least has a position.
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

    Position is world ENU metres against the tracker's origin (the node turns it
    into lat/lon). `label_votes` holds EVERY label ever seen, dark ones included,
    so the reported name is a vote over the track's whole life (TrackedTarget.msg:
    the most recent label is the wrong answer) and a changed colour_* parameter
    re-resolves the history.
    """
    id: int
    x: float
    y: float
    z: float
    label_votes: dict = field(default_factory=dict)
    # The REPORTED label: resolve_label() over label_votes, refreshed by
    # TargetTracker.snapshot() since it depends on runtime parameters. "" =
    # LiDAR-only.
    label: str = ""
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

    def confirmed(self, p: TrackerParams) -> bool:
        return self.hits >= p.confirm_hits

    @property
    def stddev(self) -> float:
        return math.sqrt(self.var)


def _majority(votes: dict) -> str:
    """The label with the most votes ("" for none). Ties go to the first seen."""
    if not votes:
        return ""
    return max(votes.items(), key=lambda kv: kv[1])[0]


_LIST = ""   # family key of colour_family_labels; a shape family's key is the shape


def _family_key(label: str, p: TrackerParams):
    """Which colour family `label` belongs to: _LIST, a shape, or None."""
    if not p.colour_vote_enable or not label:
        return None
    if label in p.colour_family_labels:
        return _LIST
    for shape in p.colour_family_shapes:
        if label == shape or label.endswith("_" + shape):
            return shape
    return None


def _family_of(votes: dict, p: TrackerParams):
    """The family of a track's votes (the first family label found), or None."""
    for label in votes:
        key = _family_key(label, p)
        if key is not None:
            return key
    return None


def _unlit(label: str, key: str, p: TrackerParams) -> bool:
    """A vote that carries no colour: a dark side beacon or the off half of a flash."""
    if label in p.colour_unlit_labels:
        return True
    return key != _LIST and (label == key or label.startswith("off_"))


def _colour(label: str, key: str) -> str:
    """The colour a lit vote is FOR. flash_red_diamond and red_diamond are both
    red: whether the light flashed is the detector's call, the colour is the
    vote. A colour_family_labels entry is its own colour."""
    if key == _LIST:
        return label
    c = label[:-len(key)].rstrip("_")
    return c[len("flash_"):] if c.startswith("flash_") else c


def _unknown_label(key: str, p: TrackerParams) -> str:
    return p.colour_unknown_label if key == _LIST else "unknown_" + key


def resolve_label(votes: dict, p: TrackerParams) -> str:
    """The label a track reports, from its complete vote tally.

    A track with no family votes (a dock sign, a target boat, an unlabelled LiDAR
    return) keeps the plain majority. A COLOUR-FAMILY track reports the lit colour
    with BOTH at least colour_min_votes votes AND at least colour_min_ratio of the LIT
    family votes, else colour_unknown_label. The ratio leaves out unlit labels (a
    flashing red buoy is dark half the time), so a track with only dark votes is
    unknown, not "black". A dead heat between the two leading colours is unknown:
    picking one would be a coin toss reported as a fact.
    """
    if not votes:
        return ""
    key = _family_of(votes, p)
    if key is None:
        return _majority(votes)
    tally, best = {}, {}      # colour -> lit votes; colour -> (votes, label)
    for label, n in votes.items():
        if _family_key(label, p) != key or _unlit(label, key, p):
            continue
        c = _colour(label, key)
        tally[c] = tally.get(c, 0) + n
        if n > best.get(c, (0, ""))[0]:
            best[c] = (n, label)
    ranked = sorted(tally.items(), key=lambda kv: kv[1], reverse=True)
    if not ranked or (len(ranked) > 1 and ranked[1][1] == ranked[0][1]):
        return _unknown_label(key, p)
    top, n_top = ranked[0]
    if n_top >= p.colour_min_votes and n_top / sum(tally.values()) >= p.colour_min_ratio:
        return best[top][1]
    return _unknown_label(key, p)


def _label_ok(track: Track, obs: Observation, p: TrackerParams) -> bool:
    """May this sighting be associated with this track?

    Compatible means: same label, both in the colour family, or one of the two has no
    label. The empty case is how a LiDAR-only track picks up a name the first time the
    camera sees it (an exact match would keep the two halves of one object apart
    forever). red_buoy and black_buoy are one buoy in two beacon states; the track
    side is read from its RAW votes, so an unknown_buoy track still takes any family
    colour and can stop being unknown.

    Two DIFFERENT labels outside the family never merge, however close: averaging a
    dock marker with a target boat would put a waypoint through the middle of
    nothing. (Two family buoys of different colours DO merge inside assoc_radius_m, so
    that radius must stay below the narrowest gate.)
    """
    if not track.label_votes or not obs.label:
        return True
    key = _family_key(obs.label, p)
    if key is not None:
        return _family_of(track.label_votes, p) == key
    return _majority(track.label_votes) == obs.label


class TargetTracker:
    """Stage 3: the track set, and the only mutable state in this module.

    Not a ROS node and not thread-safe: the node calls update() from one timer
    callback and reads the result on the same thread. Time is a caller-supplied
    monotonic float so tests drive it deterministically.
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

        An empty sensor cycle is a legitimate call and does real work: tracks still
        age and expire (skipping it lets a stale track set outlive the boat leaving
        the area).
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
        tracks = self.snapshot(boat)
        stats["tracks"] = len(tracks)
        stats["confirmed"] = sum(1 for t in tracks if t.confirmed(self.p))
        stats["colour_unknown"] = sum(
            1 for t in tracks if t.label == self.p.colour_unknown_label
            or t.label.startswith("unknown_"))
        self.stats = stats
        return tracks

    def snapshot(self, boat: BoatState):
        """Tracks with range/bearing and label refreshed, nearest first.

        Split out from update() so the node can re-range the same track set
        against a newer pose without inventing observations.
        """
        for t in self.tracks:
            t.label = resolve_label(t.label_votes, self.p)
            de, dn = t.x - boat.east, t.y - boat.north
            t.range_h = math.hypot(de, dn)
            # Absolute bearing in the LatLonHead convention (0 = true north,
            # clockwise): atan2(east, north), in that order; swapping them
            # mirrors the map.
            t.bearing_true = math.degrees(math.atan2(de, dn)) % 360.0
        return sorted(self.tracks, key=lambda t: t.range_h)

    # ---- association ----

    def _associate(self, projected, now, stats):
        """Greedy nearest-neighbour, one observation to at most one track.

        Candidate pairs are scored by world-frame distance, gated by assoc_radius_m
        and label compatibility, then consumed best-first (O(obs x tracks), both in
        the tens): first-come would hand an observation to whichever track it reached
        when two targets sit inside one radius, and strand the second.
        """
        pairs = []
        for oi, (o, wx, wy, wz) in enumerate(projected):
            for ti, t in enumerate(self.tracks):
                if not _label_ok(t, o, self.p):
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

        The position gain is max(pos_alpha, 1/hits): for the first few sightings
        that IS the running mean, so a new track converges at once instead of
        crawling at 30% per frame; after ~1/pos_alpha sightings it is a fixed-gain
        EMA that keeps responding to a moving target.
        """
        dt = max(now - t.last_seen, 1e-3)
        t.hits += 1
        a = max(self.p.pos_alpha, 1.0 / t.hits)

        nx = t.x + a * (wx - t.x)
        ny = t.y + a * (wy - t.y)
        # Velocity from the SMOOTHED positions: a finite difference of raw
        # sightings is detector noise over a small dt (several m/s for a moored
        # buoy at 10 Hz).
        self._blend_velocity(t, (nx - t.x) / dt, (ny - t.y) / dt)
        t.x, t.y = nx, ny
        t.z += a * (wz - t.z)

        # Residual against the estimate BEFORE this update: the innovation, not
        # the post-fit leftover, which would shrink toward zero whatever the data
        # did.
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

        Two timeouts: a TENTATIVE track is often one frame of detector noise (a
        phantom obstacle in front of a mission), while a CONFIRMED track is almost
        certainly a real object the boat has turned away from (dropping it is how the
        boat forgets the buoy it just rounded). track_timeout_s <= 0 means a CONFIRMED
        track NEVER expires: right for a buoy field the boat leaves and returns to, and
        a re-sighting then updates the ORIGINAL track instead of creating a second one.

        Tentative tracks always expire on tentative_timeout_s; that guard cannot be
        switched off. With immortal tracks, confirm_hits is the only thing between a
        reflection and a permanent phantom obstacle: raise it if phantoms accumulate.

        The cap is a last-resort guard against a runaway (a shoreline clustering into
        hundreds of anonymous objects): confirmed tracks are kept first, then the most
        recently seen. The caller logs it through the health topic, because silently
        forgetting an obstacle is what this package's README forbids. It applies to
        immortal tracks too.
        """
        before = len(self.tracks)
        keep = []
        for t in self.tracks:
            if t.confirmed(self.p):
                if self.p.track_timeout_s <= 0:
                    keep.append(t)                # remembered for the whole run
                    continue
                timeout = self.p.track_timeout_s
            else:
                timeout = self.p.tentative_timeout_s
            if now - t.last_seen <= timeout:
                keep.append(t)
        self.tracks = keep

        if len(self.tracks) > self.p.max_tracks:
            self.tracks.sort(key=lambda t: (t.confirmed(self.p), t.last_seen),
                             reverse=True)
            self.tracks = self.tracks[:self.p.max_tracks]

        return before - len(self.tracks)
