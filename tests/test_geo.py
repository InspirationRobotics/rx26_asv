import math

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
