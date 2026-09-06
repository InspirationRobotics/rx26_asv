"""proximity_core — turn body-frame clusters into an OBSTACLE_DISTANCE sector map.

Pure Python, no ROS, no numpy requirement beyond what the caller passes in. The
whole point of the split is that the sign conventions below can be tested at a
desk, because getting one of them wrong mirrors the world and is invisible on a
symmetric test object.

THE CONVENTIONS, in the order they bite:

  INPUT is REP-103 body: x forward, y LEFT, z up. That is what Cluster3D carries.
  OUTPUT is MAV_FRAME_BODY_FRD: index 0 at the nose, increasing CLOCKWISE.

  So bearing_cw = -atan2(y, x). A cluster off the port bow (y > 0) has a positive
  REP-103 bearing and lands in a HIGH sector index (near 360), not a low one.

  Distances are CENTIMETRES, and the two sentinels are not interchangeable:
    UINT16_MAX -> "no reading here"   (a direction we cannot see)
    0          -> "touching the sensor"
  Filling unseen sectors with 0 tells the autopilot it is wedged against
  something on every bearing. Rover always STOPS on simple avoidance
  (AVOID_BEHAVE is Copter-only), so that halts the boat.

WHY CLUSTERS GET AN ANGULAR WIDTH. A buoy 0.3 m across at 5 m subtends 3.4
degrees — most of one 5-degree sector. At 20 m it subtends 0.9 degrees and would
land in a single sector as a one-cell needle. ArduPilot then consolidates 72
sectors into 8, keeping the closest in each, so a needle survives; but a needle
placed one sector off by rounding can straddle a 45-degree boundary and land in
the wrong consolidated sector. Painting the cluster's true angular extent makes
that rounding harmless.
"""
import math

# MAVLink's "no reading" sentinel. Not 0 — see the module docstring.
NO_READING = 65535
SECTOR_COUNT = 72          # the length of OBSTACLE_DISTANCE.distances


def bearing_cw_deg(x, y):
    """Clockwise-from-nose bearing in [0, 360) for a REP-103 body point.

    REP-103 has +y to PORT and measures angles counter-clockwise; BODY_FRD
    measures clockwise. The negation is the whole conversion, and it is the one
    line in this file most likely to be "fixed" into a bug by someone who reads
    atan2(y, x) and assumes the usual convention.
    """
    return (-math.degrees(math.atan2(y, x))) % 360.0


def half_width_deg(extent_m, range_m, min_half_deg=1.0):
    """Half the angular width an object of `extent_m` subtends at `range_m`.

    Floored at min_half_deg so a distant object still paints at least its own
    sector rather than falling between two.
    """
    if range_m <= 0.01:
        return 90.0                      # on top of us: paint a wide arc
    half = math.degrees(math.atan2(max(extent_m, 0.0) * 0.5, range_m))
    return max(half, min_half_deg)


def build_sectors(clusters, *, fov_deg=180.0, increment_deg=5.0,
                  min_range_m=0.5, max_range_m=40.0, min_half_deg=1.0):
    """Sector distances in cm, nearest-wins, unseen sectors = NO_READING.

    Args:
      clusters: iterable of (x, y, extent) in REP-103 body metres. `extent` is
        the object's horizontal size; pass 0 if unknown and it gets min_half_deg.
      fov_deg: total forward field of view actually believed. The hull occludes
        aft of the beam on this boat, so the default keeps only +/-90 degrees and
        reports everything else as NO_READING. Sending hull returns as obstacles
        would make the boat think it is permanently boxed in astern.
      min_range_m / max_range_m: readings outside are discarded rather than
        clamped. A clamped bad reading is indistinguishable from a real one.

    Returns:
      list[int] of length SECTOR_COUNT.
    """
    n = SECTOR_COUNT
    out = [NO_READING] * n
    half_fov = max(0.0, min(360.0, fov_deg)) * 0.5

    for x, y, extent in clusters:
        r = math.hypot(x, y)
        if not (min_range_m <= r <= max_range_m):
            continue
        centre = bearing_cw_deg(x, y)
        # Reject anything outside the believed FOV, measured as the shorter way
        # round from the nose so 350 degrees counts as 10 degrees off the bow.
        off_nose = min(centre, 360.0 - centre)
        if off_nose > half_fov:
            continue

        hw = half_width_deg(extent, r, min_half_deg)
        cm = int(round(r * 100.0))
        # Paint outward from the sector NEAREST the object's centre, rather than
        # walking floor(edge)..ceil(edge). Two reasons, both found by test:
        #   * flooring the low edge paints a sector whose bearing lies outside
        #     the object — a buoy at 45.0 deg was reporting an obstacle at 40.0;
        #   * a narrow object sitting between two sector bearings can produce
        #     first > last and paint NOTHING, which is the dangerous direction.
        # distances[k] is a sample AT bearing k*increment, not a bin, so the
        # centre sector is the honest one and the rest are its skirt.
        centre_idx = int(round(centre / increment_deg))
        n_extra = int(round(hw / increment_deg))
        for k in range(centre_idx - n_extra, centre_idx + n_extra + 1):
            idx = k % n
            # Do not paint outside the FOV even if the object's width reaches
            # there: those sectors are "not seen", not "seen and clear".
            edge = (idx * increment_deg) % 360.0
            if min(edge, 360.0 - edge) > half_fov:
                continue
            if cm < out[idx]:
                out[idx] = cm

    return out


def summarise(sectors, increment_deg=5.0):
    """(n_filled, nearest_cm, nearest_bearing_deg) — for logs and health output.

    Returns (0, None, None) when nothing is seen, rather than inventing a
    nearest reading, so a quiet log line means quiet and not "0 cm".
    """
    filled = [(i, d) for i, d in enumerate(sectors) if d != NO_READING]
    if not filled:
        return 0, None, None
    i, d = min(filled, key=lambda t: t[1])
    return len(filled), d, (i * increment_deg) % 360.0
