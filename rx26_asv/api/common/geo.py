"""Shared frame/geodesy helpers (no ROS imports — used by nodes AND unit tests).

Conventions:
  WORLD: x = east+ [m], y = north+ [m], anchored at `origin` (lat, lon).
  BODY:  x = starboard+ [m], y = forward+ [m].
  heading: radians, 0 = true north, clockwise positive (compass convention).

Equirectangular approximation — fine at course scale (<5 km), matches the legacy
RX24 GIS math (111_139 m/deg).
"""
import math

M_PER_DEG = 111_139.0


def ground_speed_mps(vx_cms: float, vy_cms: float) -> float:
    """Horizontal ground speed [m/s] from GLOBAL_POSITION_INT's vx/vy.

    MAVLink reports those as int16 cm/s in the NED frame; the unit conversion
    lives here rather than inline in telemetry_bridge so it is unit-tested and
    cannot be silently re-derived (wrongly) by a second consumer later.
    Vertical velocity is deliberately excluded — this is a surface vessel and
    objective-2's speed floor is a horizontal-progress threshold.
    """
    return math.hypot(vx_cms, vy_cms) / 100.0


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2 * math.pi
    while a < -math.pi:
        a += 2 * math.pi
    return a


def latlon_to_xy(lat: float, lon: float, origin) -> tuple:
    lat0, lon0 = origin
    y = (lat - lat0) * M_PER_DEG
    x = (lon - lon0) * M_PER_DEG * math.cos(math.radians(lat0))
    return x, y


def xy_to_latlon(x: float, y: float, origin) -> tuple:
    lat0, lon0 = origin
    lat = lat0 + y / M_PER_DEG
    lon = lon0 + x / (M_PER_DEG * math.cos(math.radians(lat0)))
    return lat, lon


def body_to_world(bx: float, by: float, boat_x: float, boat_y: float,
                  heading_rad: float) -> tuple:
    """Rotate a BODY-frame offset into WORLD and translate by boat position.
    (The diagram's 'Coordinate Transform Node (rotation matrix)'.)"""
    ch, sh = math.cos(heading_rad), math.sin(heading_rad)
    wx = bx * ch + by * sh
    wy = -bx * sh + by * ch
    return boat_x + wx, boat_y + wy
