"""frames_core - the one local frame, as arithmetic. No rclpy, no numpy.

docs/nav2_avoidance_spec.md section 2. The TF frame `map`, the BT's east/north
frame (bt_runner's ctx_->origin) and every HazardArray coordinate are the same
equirectangular ENU plane, centred on one datum. `to_local` is that plane,
written the way crusader_bt/nav_math.hpp:92-104 writes it (same R, same operand
order). A SECOND projection anywhere in the chain is a silent offset:
crusader_common.geo (111139 m/deg) and SITL (111318.845 m/deg) differ from this
by 0.05 % and 0.11 %, so neither may be used to make map coordinates.

PARITY LITERAL. WP1's C++ test and this package's test_frames_core both pin
    datum (1.2806, 103.8557), point (+0.0009, +0.0009)  ->  x = 100.050439,
    y = 100.075434   (+-1e-6)
If you change a constant or an operand order here, the C++ side must change in
the same commit, or the BT and the costmap disagree by metres at 100 m.
"""
import math

EARTH_R_M = 6371000.0           # nav_math.hpp kEarthR
DEG = math.pi / 180.0           # nav_math.hpp kDeg


def to_local(lat, lon, dlat, dlon):
    """(lat, lon) -> (x east, y north) metres from the datum (dlat, dlon)."""
    return ((lon - dlon) * DEG * EARTH_R_M * math.cos(dlat * DEG),
            (lat - dlat) * DEG * EARTH_R_M)


def to_latlon(x, y, dlat, dlon):
    """The exact inverse of to_local: (x east, y north) -> (lat, lon)."""
    return (dlat + (y / EARTH_R_M) / DEG,
            dlon + (x / (EARTH_R_M * math.cos(dlat * DEG))) / DEG)


def yaw_from_heading(heading_deg):
    """Compass degrees (0 = north, clockwise) -> ENU yaw [rad] (0 = east,
    counter-clockwise). NaN in, NaN out: the caller decides what no heading
    means, and for TF it means no transform."""
    return math.radians(90.0 - heading_deg)


def quat_from_yaw(yaw):
    """Yaw-only rotation about +z as (x, y, z, w)."""
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


def bearing_deg(dx, dy):
    """Compass bearing [0, 360) of an east/north offset."""
    return math.degrees(math.atan2(dx, dy)) % 360.0


def is_fix(lat, lon):
    """A usable position: finite, on the globe, and not exactly (0, 0).

    GLOBAL_POSITION_INT reports lat = lon = 0 before the autopilot has a
    position. (0, 0) is a real place, but it is 8000 km from anywhere this boat
    floats, so a first fix of exactly that is the absence of a fix, and a datum
    pinned there is wrong for the life of the process.
    """
    if not (math.isfinite(lat) and math.isfinite(lon)):
        return False
    if abs(lat) > 90.0 or abs(lon) > 180.0:
        return False
    return not (lat == 0.0 and lon == 0.0)


class StampGate:
    """Strictly increasing stamps only.

    telemetry_bridge republishes /crsd/pose at 20 Hz but stamps each message at
    MAVLink RECEIPT, so a stamp repeats until the next sample arrives (and a
    stale pose repeats it forever). tf2 rejects a repeated stamp with a
    TF_REPEATED_DATA warning on every message, so only a new stamp is a new
    transform.
    """

    def __init__(self):
        self._last_ns = None

    def accept(self, stamp_ns):
        if self._last_ns is not None and stamp_ns <= self._last_ns:
            return False
        self._last_ns = stamp_ns
        return True


class TfStats:
    """What the health line reports about the transform stream.

    Times are seconds on ONE monotonic clock, supplied by the caller (so this
    stays pure). A number is only reported when there is something to measure:
    before the first transform `tf_hz` is 0.0 and `last_tf_age_s` is None,
    never a made-up age (blanks over guesses).
    """

    def __init__(self, t0):
        self.sent = 0
        self.nan_heading = 0
        self._last_sent_t = None
        self._mark_t = t0           # the rate window opens when the node does
        self._mark_sent = 0

    def note_sent(self, t):
        self.sent += 1
        self._last_sent_t = t

    def note_nan_heading(self):
        self.nan_heading += 1

    def snapshot(self, now):
        """(tf_hz over the interval since the last snapshot, age of the last
        transform in seconds or None, cumulative NaN-heading count)."""
        if now <= self._mark_t:
            hz = 0.0
        else:
            hz = (self.sent - self._mark_sent) / (now - self._mark_t)
        self._mark_t, self._mark_sent = now, self.sent
        age = None if self._last_sent_t is None else now - self._last_sent_t
        return hz, age, self.nan_heading
