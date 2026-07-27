"""Pure-core tests for LiDAR<->camera fusion geometry (no ROS, numpy only) —
mirrors tests/test_depth_association.py in style and frame conventions.

The "association gates" section below is the objective-1 safety surface: fusion
must REFINE a detection's range, never RELOCATE the detection onto a different
object. Read api/perception/lidar_fusion.py's docstring before relaxing any of it.
"""
import math

import numpy as np
import pytest

from rx26_asv.api.perception.depth_association import BodyDetection
from rx26_asv.api.perception.lidar_fusion import (
    NO_POINTS, RANGE_DISAGREE, TOO_FEW_POINTS, FusionParams, LidarExtrinsics,
    _nearest_cluster, config_extrinsic_nonidentity, fuse, params_from,
    passthrough, to_body)


# ---- extrinsics / axis remap --------------------------------------------------

def test_nominal_axis_remap_livox_forward_is_body_forward():
    # Livox x=forward -> BODY y=forward; identity extrinsic
    pts = to_body([[1.0, 0.0, 0.0]], LidarExtrinsics())
    assert np.allclose(pts[0], [0.0, 1.0, 0.0], atol=1e-9)


def test_nominal_axis_remap_livox_left_is_body_port():
    # Livox y=left -> BODY x=-1 (port = negative starboard)
    pts = to_body([[0.0, 1.0, 0.0]], LidarExtrinsics())
    assert np.allclose(pts[0], [-1.0, 0.0, 0.0], atol=1e-9)


def test_extrinsic_translation_applied():
    pts = to_body([[1.0, 0.0, 0.0]], LidarExtrinsics.from_degrees(0, 0, 0, 0.2, 0.5, 0.1))
    assert np.allclose(pts[0], [0.2, 1.5, 0.1], atol=1e-9)


def test_empty_cloud_returns_empty():
    assert to_body([], LidarExtrinsics()).shape == (0, 3)


# ---- fusion behavior ----------------------------------------------------------

def _forward_points(n, y, x=0.0, z=0.0, spread=0.01):
    """n points clustered dead ahead at forward range y (BODY frame)."""
    rng = np.random.default_rng(0)
    pts = np.tile([x, y, z], (n, 1)).astype(float)
    pts[:, 0] += rng.normal(0, spread, n)
    return pts


def test_lidar_refines_camera_range():
    # camera says 10 m dead ahead; LiDAR cluster sits at 8 m -> range corrected
    det = BodyDetection("buoy_flash_red", 0.9, 0.0, 10.0, 0.3)
    pts = _forward_points(20, y=8.0)
    fused, n_fused = fuse([det], pts, FusionParams())
    assert n_fused == 1
    assert fused[0].fused is True
    assert abs(fused[0].y - 8.0) < 0.1
    assert abs(fused[0].x) < 0.1


def test_no_support_keeps_camera_detection_not_dropped():
    # LiDAR points are far off-bearing (to port); detection must survive unchanged
    det = BodyDetection("buoy_off", 0.9, 0.0, 10.0, 0.3)
    pts = _forward_points(20, y=8.0, x=-8.0)     # ~45 deg to port
    fused, n_fused = fuse([det], pts, FusionParams())
    assert n_fused == 0
    assert len(fused) == 1
    assert fused[0].fused is False
    assert fused[0].y == 10.0                     # camera range preserved


def test_empty_cloud_passes_all_through():
    dets = [BodyDetection("a", 0.9, 0.0, 10.0, 0.3),
            BodyDetection("b", 0.8, 3.0, 5.0, 0.2)]
    fused, n_fused = fuse(dets, np.empty((0, 3)), FusionParams())
    assert n_fused == 0
    assert [f.fused for f in fused] == [False, False]


def test_bearing_gate_rejects_off_bearing_cluster():
    det = BodyDetection("x", 0.9, 0.0, 10.0, 0.3)
    pts = _forward_points(20, y=10.0, x=2.0)      # ~11 deg off; gate default 4 deg
    _, n_fused = fuse([det], pts, FusionParams(bearing_gate_rad=math.radians(4.0)))
    assert n_fused == 0


def test_height_band_rejects_water_and_sky():
    det = BodyDetection("x", 0.9, 0.0, 10.0, 0.3)
    low = _forward_points(20, y=8.0, z=-2.0)      # below z_min (water)
    high = _forward_points(20, y=8.0, z=10.0)     # above z_max (sky)
    _, n_low = fuse([det], low, FusionParams())
    _, n_high = fuse([det], high, FusionParams())
    assert n_low == 0 and n_high == 0


def test_min_points_threshold():
    det = BodyDetection("x", 0.9, 0.0, 10.0, 0.3)
    pts = _forward_points(2, y=8.0)               # only 2 returns
    _, n_fused = fuse([det], pts, FusionParams(min_points=3))
    assert n_fused == 0


def test_range_ceiling_excludes_distant_returns():
    det = BodyDetection("x", 0.9, 0.0, 80.0, 0.3)
    pts = _forward_points(20, y=80.0)             # beyond default r_max=60
    _, n_fused = fuse([det], pts, FusionParams(r_max=60.0))
    assert n_fused == 0


# ---- association gates (objective 1: refine, never relocate) --------------------

def _cluster(n, y, x=0.0, z=0.5, spread=0.01):
    """n returns at forward range y, spread laterally (BODY frame)."""
    rng = np.random.default_rng(0)
    pts = np.tile([x, y, z], (n, 1)).astype(float)
    pts[:, 0] += rng.normal(0, spread, n)
    return pts


def test_background_cannot_relocate_detection():
    """REGRESSION. A bearing gate alone let the shoreline behind a buoy win the
    median and report the buoy at the shoreline's range with fused=True — moving
    a real obstacle outside the avoidance horizon while looking confident.

    Scene: buoy at 12 m (8 returns), shoreline at 45 m (400 returns) on the same
    bearing. The shoreline out-votes the buoy 50:1, so any vote/median over the
    whole wedge picks it. The range-consistency gate must exclude it outright.
    """
    det = BodyDetection("buoy_flash_red", 0.9, 0.0, 12.0, 0.3)
    shore = _cluster(400, y=45.0, z=1.0, spread=0.3)
    pts = np.vstack([_cluster(8, y=12.0), shore])

    fused, n_fused = fuse([det], pts, FusionParams())

    assert n_fused == 1
    assert fused[0].fused is True
    assert abs(fused[0].y - 12.0) < 0.2, "fused range must stay on the buoy"
    assert fused[0].y < 20.0, "must never land on the 45 m background"
    assert fused[0].n_support == 8, "only the buoy's returns support the fix"
    assert fused[0].n_gate == 408, "both objects were on-bearing"
    # the radius must not inflate with a background range either
    assert fused[0].radius == pytest.approx(0.3, abs=0.05)


def test_foreground_clutter_cannot_pull_detection_nearer():
    """The mirror case: nearest-cluster alone would snap a far detection onto
    near clutter. The range gate brackets it to what the camera actually saw."""
    det = BodyDetection("buoy_solid_blue", 0.9, 0.0, 30.0, 0.5)
    pts = np.vstack([_cluster(30, y=5.0),          # mooring ball / spray, not the buoy
                     _cluster(20, y=30.0)])
    fused, n_fused = fuse([det], pts, FusionParams())
    assert n_fused == 1
    assert abs(fused[0].y - 30.0) < 0.3
    assert fused[0].n_support == 20


def test_nearest_cluster_wins_inside_the_bracket():
    """Two plausible objects both inside the range window: prefer the closer one
    — the conservative answer for collision avoidance."""
    det = BodyDetection("x", 0.9, 0.0, 12.0, 0.3)
    pts = np.vstack([_cluster(20, y=10.0), _cluster(20, y=16.0)])
    fused, n_fused = fuse([det], pts, FusionParams())
    assert n_fused == 1
    assert abs(fused[0].y - 10.0) < 0.2


def test_stray_near_returns_do_not_veto_a_good_fix():
    """A couple of sub-min_points returns in front (spray, a railing) must not
    block the real cluster behind them — scan outward past them."""
    det = BodyDetection("x", 0.9, 0.0, 12.0, 0.3)
    pts = np.vstack([_cluster(2, y=8.0), _cluster(20, y=12.0)])
    fused, n_fused = fuse([det], pts, FusionParams())
    assert n_fused == 1
    assert abs(fused[0].y - 12.0) < 0.2
    assert fused[0].n_support == 20


def test_range_disagree_passes_camera_range_through_with_reason():
    """On-bearing returns that all disagree with the camera -> passthrough, NOT a
    drop and NOT a relocation. `reason` is what makes a bad extrinsic visible."""
    det = BodyDetection("buoy_flash_green", 0.9, 0.0, 12.0, 0.3)
    fused, n_fused = fuse([det], _cluster(400, y=45.0, z=1.0, spread=0.3),
                          FusionParams())
    assert n_fused == 0
    assert len(fused) == 1                       # never dropped
    assert fused[0].fused is False
    assert (fused[0].x, fused[0].y) == (0.0, 12.0)   # camera range untouched
    assert fused[0].radius == 0.3
    assert fused[0].reason == RANGE_DISAGREE
    assert fused[0].n_gate == 400                # returns existed, they just disagreed
    assert fused[0].n_support == 0


def test_reasons_distinguish_the_three_rejection_modes():
    det = BodyDetection("x", 0.9, 0.0, 12.0, 0.3)
    off_bearing, = fuse([det], _cluster(20, y=12.0, x=-12.0), FusionParams())[0]
    disagree, = fuse([det], _cluster(20, y=45.0), FusionParams())[0]
    sparse, = fuse([det], _cluster(2, y=12.0), FusionParams())[0]
    assert off_bearing.reason == NO_POINTS
    assert disagree.reason == RANGE_DISAGREE
    assert sparse.reason == TOO_FEW_POINTS
    assert all(f.fused is False and f.y == 12.0
               for f in (off_bearing, disagree, sparse))


def test_fused_detection_carries_no_reason():
    det = BodyDetection("x", 0.9, 0.0, 12.0, 0.3)
    fused, _ = fuse([det], _cluster(20, y=11.0), FusionParams())
    assert fused[0].fused is True and fused[0].reason == ""


def test_range_gate_scales_with_camera_range():
    """Window = max(abs, frac * camera_range): a 3 m error is inside the gate at
    30 m (frac term) and outside it at 4 m (abs floor)."""
    p = FusionParams(range_gate_frac=0.5, range_gate_abs=2.0)
    far = BodyDetection("x", 0.9, 0.0, 30.0, 0.3)
    near = BodyDetection("x", 0.9, 0.0, 4.0, 0.3)
    assert fuse([far], _cluster(20, y=27.0), p)[1] == 1     # |3| <= 15
    assert fuse([near], _cluster(20, y=7.0), p)[1] == 0     # |3| >  2


def test_passthrough_never_moves_or_drops_a_detection():
    """The invariant itself, tested directly — lidar_fusion_node's no-cloud and
    stale-cloud paths call this same helper, so it must not mutate anything."""
    d = BodyDetection("buoy_off", 0.77, -1.5, 9.0, 0.42)
    f = passthrough(d, "stale_cloud", n_gate=5)
    assert (f.label, f.x, f.y, f.radius) == ("buoy_off", -1.5, 9.0, 0.42)
    assert f.confidence == pytest.approx(0.77)
    assert f.fused is False and f.n_support == 0
    assert f.reason == "stale_cloud" and f.n_gate == 5


# ---- _nearest_cluster ------------------------------------------------------------

def test_nearest_cluster_splits_on_gaps():
    r = np.array([10.0, 10.1, 10.2, 30.0, 30.1, 30.2])
    got = _nearest_cluster(r, gap=2.0, min_points=3)
    assert got is not None and np.allclose(got, [10.0, 10.1, 10.2])


def test_nearest_cluster_skips_undersized_and_returns_none_when_all_are():
    r = np.array([5.0, 20.0, 20.1, 20.2])
    assert np.allclose(_nearest_cluster(r, gap=2.0, min_points=3), [20.0, 20.1, 20.2])
    assert _nearest_cluster(r, gap=2.0, min_points=5) is None
    assert _nearest_cluster(np.empty(0), gap=2.0, min_points=1) is None


def test_nearest_cluster_merges_when_gap_is_large():
    r = np.array([10.0, 12.0, 14.0])
    assert _nearest_cluster(r, gap=5.0, min_points=3).size == 3


# ---- params_from: config <-> core boundary --------------------------------------

_PARAMS = dict(bearing_gate_deg=4.0, z_min=-0.5, z_max=3.0, r_min=0.5, r_max=60.0,
               min_points=3, range_gate_frac=0.5, range_gate_abs=2.0,
               cluster_gap_m=2.0)


def test_params_from_converts_degrees_to_radians():
    p = params_from(dict(_PARAMS, bearing_gate_deg=90.0))
    assert p.bearing_gate_rad == pytest.approx(math.pi / 2)
    assert p.min_points == 3 and isinstance(p.min_points, int)


def test_params_from_raises_on_a_missing_key():
    """A core param config forgot must fail loudly, not fall back to a default."""
    with pytest.raises(KeyError):
        params_from({k: v for k, v in _PARAMS.items() if k != "range_gate_frac"})


# ---- extrinsic identity / double-transform guard --------------------------------

def test_extrinsic_is_identity():
    assert LidarExtrinsics().is_identity()
    assert not LidarExtrinsics.from_degrees(0, 0, 5.0, 0, 0, 0).is_identity()
    assert not LidarExtrinsics.from_degrees(0, 0, 0, 0.1, 0, 0).is_identity()


def _cfg(ext):
    return {"lidar_configs": [{"ip": "x", "extrinsic_parameter": ext}]}


def test_driver_extrinsic_identity_is_clean():
    assert config_extrinsic_nonidentity(_cfg(
        {"roll": 0, "pitch": 0, "yaw": 0, "x": 0, "y": 0, "z": 0})) == {}
    assert config_extrinsic_nonidentity({}) == {}          # no lidar_configs key
    assert config_extrinsic_nonidentity(_cfg({})) == {}    # missing extrinsic -> treated identity


def test_driver_extrinsic_nonidentity_is_flagged():
    nz = config_extrinsic_nonidentity(_cfg(
        {"roll": 0, "pitch": 0, "yaw": 90.0, "x": 0, "y": 0, "z": 0.5}))
    assert nz == {0: {"yaw": 90.0, "z": 0.5}}
