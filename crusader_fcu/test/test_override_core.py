"""override_core: the bridge's rules for software on the sticks. No ROS.

    python crusader_fcu/test/test_override_core.py
"""
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_fcu import override_core as oc   # noqa: E402


def sticks(steer=1500, thr=1500, lat=1500, **extra):
    ch = [0] * 18
    ch[0], ch[2], ch[3] = steer, thr, lat
    for k, v in extra.items():
        ch[int(k[2:]) - 1] = v
    return ch


class TestOverride(unittest.TestCase):

    def setUp(self):
        self.g = oc.OverrideGate(oc.OverrideParams())

    def test_forwarded_in_manual(self):
        ok, why, out = self.g.on_command(0.0, sticks(1520, 1560, 1440), "MANUAL", True)
        self.assertTrue(ok)
        self.assertEqual(out, [1520, 0, 1560, 1440, 0, 0, 0, 0])

    def test_never_sb_sc_or_pump(self):
        # ch7 SB (e-stop), ch8 SC (mode), ch10 the pump: the pilot's, always
        ok, _, out = self.g.on_command(0.0, sticks(ch7=1000, ch8=1900, ch10=2000), "MANUAL", True)
        self.assertTrue(ok)
        self.assertEqual(out[6], 0)
        self.assertEqual(out[7], 0)
        self.assertEqual(len(out), 8)            # ch10 cannot even be expressed

    def test_clamped(self):
        ok, why, out = self.g.on_command(0.0, sticks(2000, 1000, 1500), "MANUAL", True)
        self.assertEqual(out[0], 1650)
        self.assertEqual(out[2], 1350)
        self.assertIn("clamped", why)

    def test_refused_outside_manual_and_releases(self):
        self.g.on_command(0.0, sticks(1560), "MANUAL", True)
        ok, why, out = self.g.on_command(0.1, sticks(1560), "HOLD", True)
        self.assertFalse(ok)
        self.assertIn("HOLD", why)
        self.assertEqual(out, oc.release())      # held the sticks: hand them back
        ok, _, out = self.g.on_command(0.2, sticks(1560), "HOLD", True)
        self.assertIsNone(out)                   # and then nothing

    def test_refused_when_dropped(self):
        ok, why, out = self.g.on_command(0.0, sticks(1560), "MANUAL", False)
        self.assertFalse(ok)
        self.assertIsNone(out)

    def test_deadman_releases_once(self):
        self.g.on_command(0.0, sticks(1560), "MANUAL", True)
        self.assertIsNone(self.g.tick(0.4, "MANUAL"))
        self.assertEqual(self.g.tick(0.6, "MANUAL"), oc.release())
        self.assertIsNone(self.g.tick(0.7, "MANUAL"))

    def test_mode_change_releases_on_tick(self):
        self.g.on_command(0.0, sticks(1560), "MANUAL", True)
        self.assertEqual(self.g.tick(0.1, "HOLD"), oc.release())

    def test_trip(self):
        self.assertIsNone(self.g.on_trip())
        self.g.on_command(0.0, sticks(1560), "MANUAL", True)
        self.assertEqual(self.g.on_trip(), oc.release())

    def test_explicit_release(self):
        self.g.on_command(0.0, sticks(1560), "MANUAL", True)
        ok, why, out = self.g.on_command(0.1, [0] * 18, "MANUAL", True)
        self.assertEqual((ok, why, out), (True, "release", oc.release()))
        self.assertIsNone(self.g.on_command(0.2, [0] * 18, "MANUAL", True)[2])

    def test_from_dict(self):
        p = oc.OverrideParams.from_dict(dict(override_channels=[1, 3, 4], override_max_us=120,
                                             override_deadman_s=0.4, override_modes=["manual"]))
        self.assertEqual(p.modes, ("MANUAL",))
        with self.assertRaises(KeyError):
            oc.OverrideParams.from_dict({})


if __name__ == "__main__":
    unittest.main(verbosity=2)
