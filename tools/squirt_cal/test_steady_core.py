"""steady_core on invented attitude and sticks.   python test_steady_core.py"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from steady_core import SteadyMonitor   # noqa: E402

NEUTRAL = [1500, 1500, 1500, 1500]


def feed(m, t0, t1, hz=30.0, roll=lambda t: 0.0, pitch=lambda t: 0.0,
         sticks=lambda t: NEUTRAL):
    """Feed attitude at hz and sticks at 10 Hz over [t0, t1)."""
    n = int(round((t1 - t0) * hz))
    for i in range(n):
        t = t0 + i / hz
        h = 1e-3
        r, p = roll(t), pitch(t)
        rr = (roll(t + h) - roll(t - h)) / (2 * h)
        pr = (pitch(t + h) - pitch(t - h)) / (2 * h)
        m.feed_att(t, r, p, rr, pr)
        if i % max(1, int(hz / 10)) == 0:
            m.feed_sticks(t, sticks(t))
    return t0 + (n - 1) / hz


class TestSteady(unittest.TestCase):

    def test_flat_and_hands_off_is_steady(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 3.0)
        st = m.status(t)
        self.assertTrue(st["steady"], st["reasons"])
        self.assertEqual(st["reasons"], [])

    def test_a_steady_list_is_fine(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 6.0, roll=lambda t: 4.0, pitch=lambda t: -2.0)
        self.assertTrue(m.status(t)["steady"])

    def test_rocking_is_not(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 6.0, pitch=lambda t: 3.0 * math.sin(2 * math.pi * t / 2.0))
        st = m.status(t)
        self.assertFalse(st["steady"])
        self.assertTrue(any("rocking" in r or "pitch" in r for r in st["reasons"]),
                        st["reasons"])

    def test_sticks_must_be_quiet(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 3.0, sticks=lambda t: [1500, 1500, 1600 if t > 2.5 else 1500, 1500])
        st = m.status(t)
        self.assertFalse(st["steady"])
        self.assertTrue(any("sticks" in r for r in st["reasons"]), st["reasons"])
        t = feed(m, 3.0, 5.0, sticks=lambda t: [1500, 1500, 1600, 1500])
        self.assertTrue(m.status(t)["steady"], m.status(t)["reasons"])

    def test_small_stick_noise_is_ignored(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 3.0, sticks=lambda t: [1500 + (int(t * 10) % 2) * 10] + NEUTRAL[1:])
        self.assertTrue(m.status(t)["steady"])

    def test_slow_attitude_is_refused(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 4.0, hz=4.0)
        st = m.status(t)
        self.assertFalse(st["steady"])
        self.assertTrue(any("Hz" in r for r in st["reasons"]), st["reasons"])

    def test_stale_attitude_is_refused(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 3.0)
        self.assertIn("no attitude", m.status(t + 1.0)["reasons"])

    def test_no_rc_is_refused(self):
        m = SteadyMonitor()
        for i in range(90):
            m.feed_att(i / 30.0, 0, 0, 0, 0)
        self.assertIn("no RC", m.status(89 / 30.0)["reasons"])

    def test_steady_for_counts_up(self):
        m = SteadyMonitor()
        t = feed(m, 0.0, 3.0)
        m.status(t)
        t2 = feed(m, 3.0, 4.0)
        self.assertGreater(m.status(t2)["steady_for_s"], 0.8)

    def test_peak_to_peak(self):
        m = SteadyMonitor()
        feed(m, 0.0, 2.0, roll=lambda t: 2.0 * math.sin(2 * math.pi * t))
        rpp, ppp = m.peak_to_peak(0.0, 2.0)
        self.assertAlmostEqual(rpp, 4.0, delta=0.1)
        self.assertAlmostEqual(ppp, 0.0, places=6)
        self.assertIsNone(m.peak_to_peak(10.0, 11.0))

    def test_unknown_setting_is_an_error(self):
        with self.assertRaises(KeyError):
            SteadyMonitor(rate_max=1.0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
