"""Pure-core tests for LiDAR<->camera fusion geometry (no ROS, numpy only) —
mirrors tests/test_depth_association.py in style and frame conventions."""
import math

import numpy as np

from robotx_2026.api.perception.depth_association import BodyDetection
from robotx_2026.api.perception.lidar_fusion import (
    FusionParams, LidarExtrinsics, config_extrinsic_nonidentity, fuse, to_body)


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
