"""frames_core: the projection, the heading convention, the stamp gate.

stdlib only, no ROS:   python crusader_nav/test/test_frames_core.py

The expected numbers are worked out from the definitions, not from the code
under test, and the parity literal is the one WP1's C++ test pins too.
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_nav import frames_core as fc   # noqa: E402

DATUM = (1.2806, 103.8557)                   # Singapore, the sim course origin


class ProjectionTest(unittest.TestCase):

    def test_parity_literal(self):
        """Pinned in crusader_bt's C++ test as well: a change here without a
        change there is a metres-sized offset between the BT and the costmap."""
        x, y = fc.to_local(DATUM[0] + 0.0009, DATUM[1] + 0.0009, *DATUM)
        self.assertAlmostEqual(x, 100.050439, delta=1e-6)
        self.assertAlmostEqual(y, 100.075434, delta=1e-6)

    def test_datum_is_the_origin(self):
        self.assertEqual(fc.to_local(*DATUM, *DATUM), (0.0, 0.0))

    def test_axes(self):
        """+lon is east (x), +lat is north (y); a degree of latitude is R*pi/180."""
        x, y = fc.to_local(DATUM[0] + 1e-4, DATUM[1], *DATUM)
        self.assertEqual(x, 0.0)
        self.assertAlmostEqual(y, 1e-4 * math.pi / 180.0 * 6371000.0, places=9)
        x, y = fc.to_local(DATUM[0], DATUM[1] + 1e-4, *DATUM)
        self.assertEqual(y, 0.0)
        self.assertGreater(x, 0.0)

    def test_longitude_scaled_by_cos_datum_latitude(self):
        x, _ = fc.to_local(DATUM[0], DATUM[1] + 1e-4, *DATUM)
        self.assertAlmostEqual(x, 1e-4 * math.pi / 180 * 6371000.0 * math.cos(
            math.radians(DATUM[0])), places=9)

    def test_round_trip(self):
        for x, y in ((0.0, 0.0), (35.0, -12.5), (-80.25, 99.9), (1000.0, 1000.0)):
            lat, lon = fc.to_latlon(x, y, *DATUM)
            bx, by = fc.to_local(lat, lon, *DATUM)
            self.assertAlmostEqual(bx, x, places=6)
            self.assertAlmostEqual(by, y, places=6)


class HeadingTest(unittest.TestCase):
    """Compass degrees (north, clockwise) -> ENU yaw (east, counter-clockwise)."""

    def test_cardinals(self):
        for deg, yaw in ((0.0, math.pi / 2), (90.0, 0.0), (180.0, -math.pi / 2),
                         (270.0, -math.pi)):
            self.assertAlmostEqual(fc.yaw_from_heading(deg), yaw, places=12)

    def test_nan_in_nan_out(self):
        self.assertTrue(math.isnan(fc.yaw_from_heading(math.nan)))

    def test_quaternion_rotates_x_onto_the_heading(self):
        """Rotating the body +x axis by the quaternion must give the unit vector
        of the compass heading: east = sin(h), north = cos(h)."""
        for deg in (0.0, 37.0, 90.0, 135.0, 225.0, 315.0):
            qx, qy, qz, qw = fc.quat_from_yaw(fc.yaw_from_heading(deg))
            self.assertEqual((qx, qy), (0.0, 0.0))
            self.assertAlmostEqual(qx * qx + qy * qy + qz * qz + qw * qw, 1.0, places=12)
            east = 1 - 2 * (qy * qy + qz * qz)         # x-axis, ENU components
            north = 2 * (qx * qy + qw * qz)
            self.assertAlmostEqual(east, math.sin(math.radians(deg)), places=12)
            self.assertAlmostEqual(north, math.cos(math.radians(deg)), places=12)

    def test_bearing(self):
        self.assertAlmostEqual(fc.bearing_deg(0.0, 1.0), 0.0)
        self.assertAlmostEqual(fc.bearing_deg(1.0, 0.0), 90.0)
        self.assertAlmostEqual(fc.bearing_deg(0.0, -1.0), 180.0)
        self.assertAlmostEqual(fc.bearing_deg(-1.0, 0.0), 270.0)


class FixTest(unittest.TestCase):

    def test_usable(self):
        self.assertTrue(fc.is_fix(*DATUM))
        self.assertTrue(fc.is_fix(-33.9, 151.2))

    def test_unusable(self):
        for lat, lon in ((0.0, 0.0), (math.nan, 103.0), (1.0, math.inf), (91.0, 0.0),
                         (0.0, 181.0)):
            self.assertFalse(fc.is_fix(lat, lon), (lat, lon))

    def test_one_zero_coordinate_is_a_real_place(self):
        self.assertTrue(fc.is_fix(0.0, 103.8))      # on the equator
        self.assertTrue(fc.is_fix(1.28, 0.0))       # on the prime meridian


class StampGateTest(unittest.TestCase):

    def test_strictly_increasing_only(self):
        g = fc.StampGate()
        self.assertTrue(g.accept(100))
        self.assertFalse(g.accept(100))              # repeated stamp: tf2 would reject it
        self.assertFalse(g.accept(99))               # backwards
        self.assertTrue(g.accept(101))
        self.assertFalse(g.accept(101))

    def test_zero_is_a_stamp(self):
        g = fc.StampGate()
        self.assertTrue(g.accept(0))
        self.assertFalse(g.accept(0))


class TfStatsTest(unittest.TestCase):

    def test_no_transform_yet_is_blank_not_zero(self):
        s = fc.TfStats(t0=10.0)
        hz, age, nan_n = s.snapshot(11.0)
        self.assertEqual(hz, 0.0)
        self.assertIsNone(age)                       # a blank, not a made-up age
        self.assertEqual(nan_n, 0)

    def test_rate_age_and_nan_count(self):
        s = fc.TfStats(t0=0.0)
        for k in range(10):
            s.note_sent(0.1 * (k + 1))
        s.note_nan_heading()
        s.note_nan_heading()
        hz, age, nan_n = s.snapshot(2.0)
        self.assertAlmostEqual(hz, 5.0)              # 10 transforms in 2 s
        self.assertAlmostEqual(age, 1.0)             # last one at t = 1.0
        self.assertEqual(nan_n, 2)
        hz, age, nan_n = s.snapshot(3.0)             # a quiet second
        self.assertEqual(hz, 0.0)
        self.assertAlmostEqual(age, 2.0)             # and it is getting older


if __name__ == "__main__":
    unittest.main()
