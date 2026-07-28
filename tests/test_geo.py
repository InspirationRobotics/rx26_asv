import math

import pytest

from rx26_asv.api.common import geo


ORIGIN = (32.7020, -117.2510)


def test_latlon_xy_roundtrip():
    lat, lon = geo.xy_to_latlon(120.0, -45.0, ORIGIN)
    x, y = geo.latlon_to_xy(lat, lon, ORIGIN)
    assert abs(x - 120.0) < 1e-6 and abs(y - (-45.0)) < 1e-6


def test_north_is_plus_y():
    x, y = geo.latlon_to_xy(ORIGIN[0] + 0.001, ORIGIN[1], ORIGIN)
    assert abs(x) < 1e-9 and y > 100


def test_body_to_world_identity_at_north():
    # heading 0 (north): forward (+by) -> +y, starboard (+bx) -> +x
    assert geo.body_to_world(0, 10, 0, 0, 0.0) == (0, 10)
    assert geo.body_to_world(3, 0, 0, 0, 0.0) == (3, 0)


def test_body_to_world_heading_east():
    # heading 90 deg (east): forward -> +x, starboard -> -y
    wx, wy = geo.body_to_world(0, 10, 0, 0, math.pi / 2)
    assert abs(wx - 10) < 1e-9 and abs(wy) < 1e-9
    wx, wy = geo.body_to_world(3, 0, 0, 0, math.pi / 2)
    assert abs(wx) < 1e-9 and abs(wy + 3) < 1e-9


def test_translation_applied():
    wx, wy = geo.body_to_world(0, 5, 100, 200, 0.0)
    assert (wx, wy) == (100, 205)


def test_wrap_pi():
    assert abs(geo.wrap_pi(3 * math.pi) - math.pi) < 1e-9
    assert abs(geo.wrap_pi(-3 * math.pi) + math.pi) < 1e-9


# --- ground speed: the cm/s -> m/s seam between MAVLink and objective 2 ---

def test_ground_speed_converts_cm_s_to_m_s():
    assert geo.ground_speed_mps(200.0, 0.0) == pytest.approx(2.0)   # CRUISE_SPEED
    assert geo.ground_speed_mps(0.0, -150.0) == pytest.approx(1.5)


def test_ground_speed_combines_both_axes():
    assert geo.ground_speed_mps(300.0, 400.0) == pytest.approx(5.0)


def test_ground_speed_is_zero_when_stopped():
    assert geo.ground_speed_mps(0.0, 0.0) == 0.0


def test_ground_speed_straddles_the_objective2_floor():
    """The 0.3 m/s speed_floor must sit where the config says it does."""
    assert geo.ground_speed_mps(20.0, 0.0) < 0.3       # 0.2 m/s -> below floor
    assert geo.ground_speed_mps(40.0, 0.0) > 0.3       # 0.4 m/s -> above floor
