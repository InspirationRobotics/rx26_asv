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
    min_confidence: float = 0.50     # re-threshold above the detector's own
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
    # A camera frame and the pose it is projected with are not from one instant
    # (detector latency vs the pose stream): turning at w rad/s, a sighting at
    # range R lands about R * w * turn_skew_s sideways of its buoy. Past
    # assoc_radius_m that is a new track that never expires (a ghost copy of
    # the buoy, often family-merged with its gate partner); inside it, it drags
    # the real track. Camera-only sightings whose expected smear exceeds
    # turn_smear_max_m are dropped (0 = off). LiDAR-positioned ones are kept.
    turn_smear_max_m: float = 1.0
    turn_skew_s: float = 0.2

    # -- RANGE-AWARE TRACKING (stage 3). OFF while range_sigma_k is 0: the fixed
    # assoc_radius_m and pos_alpha above, exactly as before.
    #
    # Why. Stereo range error grows with range squared (sigma ~ k * r^2), so ONE
    # association radius cannot serve a buoy field whose buoys are 1.9-3.5 m apart
    # (the 2026-11-10 Singapore field): 3.0 m merges neighbours seen up close
    # (their positions drag toward each other and their colours mix), 1.5 m splits
    # every buoy seen from 15-20 m into duplicates (23 tracks for 11 buoys in the
    # sim, 23-25 for ten on the water on 2026-11-13). On, each camera sighting has
    # sigma(r) = max(range_sigma_floor_m, range_sigma_k * r^2), and:
    #   * its association radius with a track is range_gate_sigmas times their
    #     combined sigma (the track's is 1/sqrt of its accumulated weight), clamped to
    #     [assoc_min_m, assoc_radius_m]: tight up close on a well-placed buoy, wide
    #     far out or on a buoy so far only seen from far;
    #   * it moves the track by INVERSE-VARIANCE weight, w = 1 / sigma^2, against
    #     the track's accumulated weight (capped at 4 / floor^2 so the track still
    #     follows): a far sighting barely moves a buoy fixed from close in, a close
    #     one corrects a buoy first placed from far away;
    #   * a sighting whose nearest OTHER compatible track is within
    #     vote_ambiguity_m of its own casts NO colour vote (it may be the
    #     neighbour's light); its position still counts, at its weight.
    # A LiDAR-positioned sighting has sigma = the floor. Sim calibration
    # (sim_camera's depth model, f 368 px, B 0.15 m, 0.2 px): k ~ 0.0036; a real
    # OAK-D (B 0.075 m) is about twice that.
    range_sigma_k: float = 0.0       # m per m^2; 0 = off
    range_sigma_floor_m: float = 0.10
    range_gate_sigmas: float = 2.5
    assoc_min_m: float = 0.9         # the gate never below this
    vote_ambiguity_m: float = 0.0    # 0 = every sighting votes

    # -- MAPPED FIELD PRIOR (stage 2b, between projection and association). OFF
    # until TargetTracker.set_prior() is given the mapped buoy positions (the node:
    # the prior_field parameter). A moored field does not move between runs, only
    # its lights change, and the 2026-11-10 LiDAR map has it to ~0.12 m. With a map:
    #   1. ALIGN. The map's offset from the boat's GPS frame (3 m between two runs
    #      on 2026-11-10) is found by a coarse-to-fine grid search, every
    #      prior_align_period_s, over the last sightings within prior_snap_range_m:
    #      the shift (up to prior_align_max_m) that puts the most of them on a
    #      mapped buoy. Until it is found (prior_align_min_obs sightings, half of
    #      them explained) sightings near the field are DROPPED, never tracked, so
    #      no stray track is born there before the map is placed.
    #   2. SNAP. A sighting within prior_snap_m of an aligned mapped buoy, seen from
    #      within prior_snap_range_m, whose next-nearest mapped buoy is at least
    #      prior_margin_m farther, is placed EXACTLY on it: one track per buoy, at
    #      the map's position, every run.
    #   3. DROP. Any other sighting within prior_zone_m of a mapped buoy is ambiguous
    #      (which buoy's light is it?) and is dropped.
    #   4. PASS. A sighting farther than prior_zone_m from every mapped buoy is not
    #      the field (a boat, a dock, a buoy moved): tracked as before.
    prior_snap_m: float = 1.0
    prior_zone_m: float = 2.0
    prior_margin_m: float = 0.5
    prior_snap_range_m: float = 15.0
    prior_align_max_m: float = 4.0
    prior_align_period_s: float = 2.0
    prior_align_min_obs: int = 15

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
                                     # buoy_model's vocabulary is
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
    yaw_rate: float = 0.0   # rad/s, Attitude.yawspeed; 0 = steady or unknown


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
    info: float = 0.0                # range-aware tracking: accumulated 1/sigma^2
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


def _placeholder(key: str, p: TrackerParams):
    """One UNLIT vote of family `key`: keeps a track in its family with no colour (so it still
    takes any family colour, _label_ok), or None when the configuration has no such label."""
    if key == _LIST:
        return next((lb for lb in p.colour_unlit_labels if lb in p.colour_family_labels), None)
    return key


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
        self.set_prior(())

    # ---- the mapped field prior (TrackerParams, "MAPPED FIELD PRIOR") ----

    def set_prior(self, points):
        """The mapped buoys as world (x, y); empty = no prior. Resets the alignment, and a
        map REPLACES what was tracked in the field: every track within prior_zone_m +
        prior_align_max_m of a mapped buoy is dropped (it was placed without the map, and
        could stay as a duplicate of the buoy the map now places)."""
        self.prior = tuple((float(x), float(y)) for x, y in points)
        if self.prior:
            reach = self.p.prior_zone_m + self.p.prior_align_max_m
            self.tracks = [t for t in self.tracks
                           if min(math.hypot(t.x - px, t.y - py) for px, py in self.prior)
                           > reach]
        self.prior_offset = (0.0, 0.0)
        self.prior_aligned = False
        self.prior_explained = 0.0
        self._align_buf = []
        self._last_align = -math.inf

    def _prior_dists(self, x, y, off):
        """Sorted distances from (x, y) to every mapped buoy shifted by off, with indexes."""
        return sorted((math.hypot(x - px - off[0], y - py - off[1]), i)
                      for i, (px, py) in enumerate(self.prior))

    def _align_score(self, pts, off):
        """How well a shift explains the sightings: each scores 1 on a mapped buoy, 0 at
        prior_snap_m or farther."""
        s = 0.0
        r = self.p.prior_snap_m
        for x, y in pts:
            d = min(math.hypot(x - px - off[0], y - py - off[1]) for px, py in self.prior)
            if d < r:
                s += 1.0 - d / r
        return s

    def _align(self):
        """Grid search for the map's offset: 0.5 m steps over +-prior_align_max_m, then
        0.1 m steps round the best. Ties go to the smaller shift."""
        pts = self._align_buf[-60:]
        lim = self.p.prior_align_max_m
        best = None
        n = int(lim / 0.5)
        for i in range(-n, n + 1):
            for j in range(-n, n + 1):
                off = (0.5 * i, 0.5 * j)
                key = (self._align_score(pts, off), -math.hypot(*off))
                if best is None or key > best[0]:
                    best = (key, off)
        cx, cy = best[1]
        for i in range(-5, 6):
            for j in range(-5, 6):
                off = (cx + 0.1 * i, cy + 0.1 * j)
                if math.hypot(*off) > lim:
                    continue
                key = (self._align_score(pts, off), -math.hypot(*off))
                if key > best[0]:
                    best = (key, off)
        explained = best[0][0] / len(pts)
        self.prior_explained = explained
        if explained >= 0.5:
            self.prior_offset = best[1]
            self.prior_aligned = True

    def _apply_prior(self, projected, now, stats):
        """Stage 2b: align, then snap, drop or pass every projected sighting."""
        p = self.p
        for o, wx, wy, _ in projected:
            if o.range_h <= p.prior_snap_range_m:
                self._align_buf.append((wx, wy))
        del self._align_buf[:-200]
        if (now - self._last_align >= p.prior_align_period_s
                and len(self._align_buf) >= p.prior_align_min_obs):
            self._last_align = now
            self._align()
        kept, snapped, dropped = [], 0, 0
        for o, wx, wy, wz in projected:
            if not self.prior_aligned:
                # not placed yet: nothing near the field is tracked
                d0 = self._prior_dists(wx, wy, (0.0, 0.0))[0][0]
                if d0 <= p.prior_zone_m + p.prior_align_max_m:
                    dropped += 1
                    continue
                kept.append((o, wx, wy, wz))
                continue
            ds = self._prior_dists(wx, wy, self.prior_offset)
            d1, i = ds[0]
            d2 = ds[1][0] if len(ds) > 1 else math.inf
            if (d1 <= p.prior_snap_m and d2 - d1 >= p.prior_margin_m
                    and o.range_h <= p.prior_snap_range_m):
                px, py = self.prior[i]
                kept.append((o, px + self.prior_offset[0], py + self.prior_offset[1], wz))
                snapped += 1
            elif d1 <= p.prior_zone_m:
                dropped += 1
            else:
                kept.append((o, wx, wy, wz))
        stats["prior_snapped"] = snapped
        stats["prior_dropped"] = dropped
        stats["prior_offset"] = (round(self.prior_offset[0], 2), round(self.prior_offset[1], 2))
        stats["prior_aligned"] = self.prior_aligned
        return kept

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
        obs, stats["turn_dropped"] = self._drop_turn_smear(obs, boat)

        projected = []
        for o in obs:
            wx, wy, wz = geo.body_to_world_ypr(
                o.x, o.y, o.z, boat.roll, boat.pitch, boat.yaw,
                boat.east, boat.north)
            projected.append((o, wx, wy, wz))
        if self.prior:
            projected = self._apply_prior(projected, now, stats)

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

    def _drop_turn_smear(self, obs, boat: BoatState):
        """(kept, dropped count): camera-only sightings too far out to place while
        turning this fast (see TrackerParams.turn_smear_max_m)."""
        lim = self.p.turn_smear_max_m
        rate = abs(boat.yaw_rate)
        if lim <= 0.0 or not math.isfinite(rate) or rate == 0.0:
            return obs, 0
        per_m = rate * self.p.turn_skew_s
        kept = [o for o in obs
                if o.sources != SOURCE_CAMERA or o.range_h * per_m <= lim]
        return kept, len(obs) - len(kept)

    def forget_colours(self) -> int:
        """A new run: the competition re-deals red and green between runs, and votes from the
        last run would outvote this run's for a long time (resolve_label counts a track's
        whole life). Every colour-family track keeps its id, position and hits and loses its
        colour: its votes become one unlit vote of its family, so it reports unknown until
        the camera reads its light again. Other tracks (dock signs, boats, LiDAR-only) keep
        theirs. Returns how many tracks lost a colour."""
        n = 0
        for t in self.tracks:
            key = _family_of(t.label_votes, self.p)
            ph = None if key is None else _placeholder(key, self.p)
            if ph is None:
                continue
            t.label_votes = {ph: 1}
            t.label = resolve_label(t.label_votes, self.p)
            n += 1
        return n

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

    def _ranged(self) -> bool:
        return self.p.range_sigma_k > 0.0

    def _sigma(self, o: Observation) -> float:
        """Range-aware tracking: the sighting's position sigma [m] (TrackerParams)."""
        floor = max(self.p.range_sigma_floor_m, 1e-3)
        if o.sources & SOURCE_LIDAR:
            return floor
        return max(floor, self.p.range_sigma_k * o.range_h * o.range_h)

    def _gate(self, o: Observation, t: Track) -> float:
        """The association radius for this sighting and this track. Range-aware, it is
        range_gate_sigmas times their COMBINED sigma (the track's is 1/sqrt(info)), so a
        buoy first placed from far away still takes the close sighting that corrects it."""
        if not self._ranged():
            return self.p.assoc_radius_m
        var_t = 1.0 / t.info if t.info > 0.0 else math.inf
        g = self.p.range_gate_sigmas * math.sqrt(self._sigma(o) ** 2 + var_t)
        return min(self.p.assoc_radius_m, max(self.p.assoc_min_m, g))

    def _associate(self, projected, now, stats):
        """Greedy nearest-neighbour, one observation to at most one track.

        Candidate pairs are scored by world-frame distance, gated by assoc_radius_m
        (range-aware: by _gate) and label compatibility, then consumed best-first
        (O(obs x tracks), both in the tens): first-come would hand an observation to
        whichever track it reached when two targets sit inside one radius, and
        strand the second.
        """
        pairs = []
        nearest = []                      # per observation: distances to every compatible track
        for oi, (o, wx, wy, wz) in enumerate(projected):
            ds = []
            for ti, t in enumerate(self.tracks):
                if not _label_ok(t, o, self.p):
                    continue
                d = math.hypot(wx - t.x, wy - t.y)
                ds.append((d, ti))
                if d <= self._gate(o, t):
                    pairs.append((d, oi, ti))
            nearest.append(ds)
        pairs.sort()

        used_obs, used_tracks = set(), set()
        stats.setdefault("votes_withheld", 0)
        for d, oi, ti in pairs:
            if oi in used_obs or ti in used_tracks:
                continue
            used_obs.add(oi)
            used_tracks.add(ti)
            o, wx, wy, wz = projected[oi]
            vote = self._unambiguous(d, ti, nearest[oi])
            if not vote and o.label:
                stats["votes_withheld"] += 1
            self._update_track(self.tracks[ti], o, wx, wy, wz, d, now, vote=vote)
            stats["associated"] += 1

        for oi, (o, wx, wy, wz) in enumerate(projected):
            if oi in used_obs:
                continue
            self._new_track(o, wx, wy, wz, now)
            stats["new_tracks"] += 1

    def _unambiguous(self, d, ti, ds) -> bool:
        """May this sighting vote on its track's colour? Not when another compatible
        track is within vote_ambiguity_m of the one it joined (TrackerParams)."""
        margin = self.p.vote_ambiguity_m
        if margin <= 0.0:
            return True
        others = [dd for dd, tj in ds if tj != ti]
        return not others or min(others) - d >= margin

    def _update_track(self, t: Track, o: Observation,
                      wx: float, wy: float, wz: float, resid: float, now: float,
                      vote: bool = True):
        """Blend one sighting into an existing track.

        The position gain is max(pos_alpha, 1/hits): for the first few sightings
        that IS the running mean, so a new track converges at once instead of
        crawling at 30% per frame; after ~1/pos_alpha sightings it is a fixed-gain
        EMA that keeps responding to a moving target. Range-aware (range_sigma_k > 0),
        it is the sighting's inverse-variance share of the track's weight instead
        (TrackerParams). `vote` False = the position counts, the label does not.
        """
        dt = max(now - t.last_seen, 1e-3)
        t.hits += 1
        if self._ranged():
            w = 1.0 / self._sigma(o) ** 2
            a = w / (t.info + w)
            cap = 4.0 / max(self.p.range_sigma_floor_m, 1e-3) ** 2
            t.info = min(t.info + w, cap)
        else:
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
        if o.label and vote:
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
                  extent=tuple(o.extent),
                  info=1.0 / self._sigma(o) ** 2 if self._ranged() else 0.0)
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
