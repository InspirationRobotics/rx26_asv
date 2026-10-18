"""Shared frame/geodesy helpers (no ROS imports — used by nodes AND unit tests).

Conventions:
  WORLD: x = east+ [m], y = north+ [m], z = up+ [m], anchored at `origin`.
  BODY:  x = starboard+ [m], y = forward+ [m]   (legacy RX24; body_to_world).
  REP-103 BODY: x = forward+, y = left+, z = up+ (Detection3D; body_to_world_ypr).
  heading/yaw: radians, 0 = true north, clockwise positive (compass convention).

The two BODY conventions coexist because the RX24 math and ROS disagree, and
renaming either one silently breaks whichever caller was right. Each function
states which it takes; do not mix them.

Equirectangular approximation — fine at course scale (<5 km), matches the legacy
RX24 GIS math (111_139 m/deg).
"""
import math

M_PER_DEG = 111_139.0


def ground_speed_mps(vx_cms: float, vy_cms: float) -> float:
    """Horizontal ground speed [m/s] from GLOBAL_POSITION_INT's vx/vy.

    MAVLink reports those as int16 cm/s in NED; the conversion lives here so it
    is unit-tested and cannot be re-derived (wrongly) by a second consumer.
    Vertical velocity is excluded: this is a surface vessel and the speed floor is
    a horizontal-progress threshold.
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
    (The diagram's 'Coordinate Transform Node (rotation matrix)'.)

    Yaw only: assumes the boat is level. For a camera or LiDAR detection use
    body_to_world_ypr: in chop, roll and pitch move a 20 m bearing by metres.

    NOTE the axis convention here is the legacy RX24 one (bx = starboard,
    by = forward), NOT the REP-103 one Detection3D uses; body_to_world_ypr takes
    REP-103 and converts internally so a mapping node never has to remember which
    is which.
    """
    ch, sh = math.cos(heading_rad), math.sin(heading_rad)
    wx = bx * ch + by * sh
    wy = -bx * sh + by * ch
    return boat_x + wx, boat_y + wy


def body_to_world_ypr(x_fwd: float, y_left: float, z_up: float,
                      roll: float, pitch: float, yaw: float,
                      boat_x: float = 0.0, boat_y: float = 0.0) -> tuple:
    """Place a REP-103 body-frame detection in the WORLD frame using full YPR.

    The transform mission-element mapping runs on every detection: a Detection3D
    arrives in body axes relative to a boat that is rolling and pitching, and the
    map needs fixed world axes.

    Args:
      x_fwd, y_left, z_up: offset from the boat in REP-103 BODY axes [m] —
        exactly the (x, y, z) triple carried by crusader_msgs/Detection3D.
      roll, pitch, yaw: attitude [rad] straight off /crsd/attitude, in the
        autopilot's NED body axes and compass yaw. Passed through unconverted
        so a value read in Mission Planner can be typed in here unchanged.
      boat_x, boat_y: boat position in WORLD [m] (latlon_to_xy of /crsd/pose).
        Default 0 gives the world-frame OFFSET rather than an absolute point.

    Returns:
      (world_x, world_y, world_z) — east+ [m], north+ [m], up+ [m]. z is
      returned because it is the cheapest sanity check: a buoy that maps to +4 m
      of altitude means the attitude and the detection disagree.

    Rotation is the standard aerospace 3-2-1 (yaw, then pitch, then roll) applied
    to the FRD form of the input, matching how ArduPilot's EKF defines the
    angles; any other order is wrong by degrees once the boat does two of the
    three at once.
    """
    # REP-103 (x fwd, y left, z up) -> NED body / FRD (x fwd, y right, z down),
    # which is the frame the EKF's roll/pitch/yaw are defined against.
    xf, yr, zd = x_fwd, -y_left, -z_up

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    north = cy * cp * xf + (cy * sp * sr - sy * cr) * yr + (cy * sp * cr + sy * sr) * zd
    east = sy * cp * xf + (sy * sp * sr + cy * cr) * yr + (sy * sp * cr - cy * sr) * zd
    down = -sp * xf + cp * sr * yr + cp * cr * zd

    # WORLD is x = east, y = north, z = up (geo.py convention, top of file).
    return boat_x + east, boat_y + north, -down


def world_to_body_ypr(world_x: float, world_y: float, world_z: float,
                      roll: float, pitch: float, yaw: float,
                      boat_x: float = 0.0, boat_y: float = 0.0) -> tuple:
    """The exact inverse of body_to_world_ypr — world point to REP-103 BODY.

    Answers "where is that, from here": a known world position (waypoint, stored
    target, simulated buoy) as the offset a sensor on this boat would report.

    Args:
      world_x, world_y, world_z: the point in WORLD axes — east+ [m], north+
        [m], up+ [m], against the same origin latlon_to_xy uses.
      roll, pitch, yaw: attitude [rad], autopilot NED body axes and compass
        yaw, exactly as body_to_world_ypr takes them.
      boat_x, boat_y: boat position in WORLD [m]. Default 0 treats the inputs
        as an offset from the boat rather than an absolute point.

    Returns:
      (x_fwd, y_left, z_up) [m] — REP-103 body axes, the triple Detection3D and
      Cluster3D carry.

    Implemented as the TRANSPOSE of the forward rotation: re-deriving it by
    negating the angles and reversing the order gives the same matrix and one more
    place to get a sign wrong.

    NOTE round-tripping through both proves the pair is CONSISTENT, not that the
    convention is right (a shared sign error cancels). Proving the convention is a
    bench exercise against a real sensor, as docs/G2 did for the LiDAR.
    """
    # WORLD (east, north, up) -> NED (north, east, down), relative to the boat.
    north = world_y - boat_y
    east = world_x - boat_x
    down = -world_z

    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    xf = cy * cp * north + sy * cp * east - sp * down
    yr = ((cy * sp * sr - sy * cr) * north + (sy * sp * sr + cy * cr) * east
          + cp * sr * down)
    zd = ((cy * sp * cr + sy * sr) * north + (sy * sp * cr - cy * sr) * east
          + cp * cr * down)

    # FRD (x fwd, y right, z down) -> REP-103 (x fwd, y LEFT, z UP).
    return xf, -yr, -zd
