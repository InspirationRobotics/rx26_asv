"""guided_hs_core: the gates, the encoding, the dead-man.

No ROS, no pymavlink:   python crusader_fcu/test/test_guided_hs_core.py
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_fcu import guided_hs_core as hs   # noqa: E402

P = hs.HsParams(max_speed=0.4, wp_speed=1.0, deadman_s=0.5)


def yaw_of(q):
    """Compass yaw back out of a pure-yaw quaternion (ZYX euler yaw)."""
    w, x, y, z = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))) % 360.0


class TestEncode(unittest.TestCase):

    def test_type_mask_uses_attitude_and_thrust(self):
        mask, _, _ = hs.encode(0, 0, 1.0)
        self.assertEqual(mask & 4, 4)                       # yaw RATE ignored -> quaternion
        self.assertEqual(mask & hs.THROTTLE_IGNORE, 0)      # thrust NOT ignored

    def test_heading_round_trips(self):
        for h in (0.0, 45.0, 90.0, 179.0, 181.0, 270.0, 356.1):
            _, q, _ = hs.encode(h, 0.1, 1.0)
            self.assertAlmostEqual(yaw_of(q), h % 360.0, places=6)
            self.assertAlmostEqual(sum(v * v for v in q), 1.0, places=12)

    def test_thrust_is_speed_over_wp_speed(self):
        self.assertAlmostEqual(hs.encode(0, 0.2, 1.0)[2], 0.2)
        self.assertAlmostEqual(hs.encode(0, -0.2, 1.0)[2], -0.2)    # astern
        self.assertAlmostEqual(hs.encode(0, 0.2, 2.0)[2], 0.1)
        self.assertEqual(hs.encode(0, 5.0, 1.0)[2], 1.0)            # clamped


class TestGate(unittest.TestCase):

    def test_guided_and_latch_allowed_sends(self):
        g = hs.HsGate(P)
        ok, why, f = g.on_command(0.0, 90.0, 0.2, "GUIDED", True)
        self.assertTrue(ok)
        self.assertAlmostEqual(f[2], 0.2)
        self.assertTrue(g.active)

    def test_not_guided_refuses(self):
        g = hs.HsGate(P)
        for mode in ("MANUAL", "HOLD", "AUTO", "LOITER"):
            ok, why, f = g.on_command(0.0, 90.0, 0.2, mode, True)
            self.assertFalse(ok)
            self.assertIsNone(f)
            self.assertIn("not GUIDED", why)

    def test_lower_case_mode_still_guided(self):
        self.assertTrue(hs.HsGate(P).on_command(0.0, 90.0, 0.2, "guided", True)[0])

    def test_latch_refuses(self):
        ok, why, f = hs.HsGate(P).on_command(0.0, 90.0, 0.2, "GUIDED", False)
        self.assertFalse(ok)
        self.assertIn("latch", why)

    def test_speed_clamped_both_ways(self):
        g = hs.HsGate(P)
        ok, why, f = g.on_command(0.0, 0.0, 3.0, "GUIDED", True)
        self.assertAlmostEqual(f[2], 0.4)
        self.assertIn("clamped", why)
        ok, why, f = g.on_command(0.1, 0.0, -3.0, "GUIDED", True)
        self.assertAlmostEqual(f[2], -0.4)

    def test_garbage_refused(self):
        g = hs.HsGate(P)
        self.assertFalse(g.on_command(0.0, float("nan"), 0.1, "GUIDED", True)[0])
        self.assertFalse(g.on_command(0.0, 10.0, float("inf"), "GUIDED", True)[0])
        self.assertFalse(g.on_command(0.0, None, 0.1, "GUIDED", True)[0])

    def test_deadman_sends_one_stop_holding_heading(self):
        g = hs.HsGate(P)
        g.on_command(0.0, 123.0, 0.2, "GUIDED", True)
        self.assertIsNone(g.tick(0.4))
        stop = g.tick(0.6)
        self.assertIsNotNone(stop)
        self.assertEqual(stop[2], 0.0)
        self.assertAlmostEqual(yaw_of(stop[1]), 123.0, places=6)
        self.assertIsNone(g.tick(0.7))            # once, not every tick
        self.assertFalse(g.active)

    def test_commands_keep_the_deadman_quiet(self):
        g = hs.HsGate(P)
        for i in range(20):
            g.on_command(i * 0.1, 10.0, 0.1, "GUIDED", True)
            self.assertIsNone(g.tick(i * 0.1 + 0.05))

    def test_trip_stops_only_if_driving(self):
        g = hs.HsGate(P)
        self.assertIsNone(g.on_trip())
        g.on_command(0.0, 45.0, -0.2, "GUIDED", True)
        stop = g.on_trip()
        self.assertEqual(stop[2], 0.0)
        self.assertIsNone(g.on_trip())

    def test_leaving_guided_disarms_the_deadman(self):
        g = hs.HsGate(P)
        g.on_command(0.0, 45.0, 0.2, "GUIDED", True)
        g.on_command(0.1, 45.0, 0.2, "MANUAL", True)
        self.assertIsNone(g.tick(5.0))   # the pilot has it: we send nothing

    def test_from_dict_needs_every_key(self):
        d = dict(hs_max_speed_mps=0.4, hs_wp_speed_mps=1.0, hs_deadman_s=0.5)
        self.assertEqual(hs.HsParams.from_dict(d).max_speed, 0.4)
        del d["hs_deadman_s"]
        with self.assertRaises(KeyError):
            hs.HsParams.from_dict(d)


if __name__ == "__main__":
    unittest.main(verbosity=2)
