"""drop_latch: channel 0 is no latch; a real channel still latches as before. No ROS.

python3 -m pytest crusader_behavior/test/test_drop_latch.py
"""
import unittest

from crusader_behavior.drop_latch import DropLatch


def rc(sc=1995):
    """One RC_CHANNELS reading, SC (ch8) at `sc` us."""
    ch = [1500] * 18
    ch[7] = sc
    return ch


class TestNoLatch(unittest.TestCase):

    def setUp(self):
        self.latch = DropLatch(channel=0, threshold=1230, invert=True, stale_timeout=1.0)

    def test_allowed_before_any_rc(self):
        self.assertTrue(self.latch.off)
        self.assertTrue(self.latch.allowed)
        self.assertFalse(self.latch.tick(10.0))          # no RC at all: not "stale"

    def test_nothing_trips_it(self):
        self.assertFalse(self.latch.rc_sample([0] * 18, 1.0))   # RC lost
        self.assertFalse(self.latch.rc_sample(rc(994), 2.0))    # SC down
        self.assertFalse(self.latch.tick(100.0))                # RC stale
        self.assertTrue(self.latch.allowed)
        self.assertFalse(self.latch.dropped)

    def test_reset_is_a_no_op(self):
        self.assertEqual(self.latch.reset(5.0), (True, "not dropped"))


class TestScLatch(unittest.TestCase):
    """drop_channel 8, as on the boat until 2026-11-11: unchanged."""

    def setUp(self):
        self.latch = DropLatch(channel=8, threshold=1230, invert=True, stale_timeout=1.0)

    def test_blocked_until_a_safe_sample(self):
        self.assertFalse(self.latch.off)
        self.assertFalse(self.latch.allowed)
        self.latch.rc_sample(rc(1995), 0.0)
        self.assertTrue(self.latch.allowed)

    def test_sc_down_trips_and_rc_loss_trips(self):
        self.latch.rc_sample(rc(1995), 0.0)
        self.assertTrue(self.latch.rc_sample(rc(994), 0.1))
        self.assertTrue(self.latch.dropped)
        self.latch.rc_sample(rc(1995), 0.2)                     # SC back up clears it
        self.assertTrue(self.latch.allowed)
        self.assertTrue(self.latch.rc_sample([0] * 18, 0.3))
        self.assertTrue(self.latch.dropped)


class TestChannelRange(unittest.TestCase):

    def test_out_of_range(self):
        for ch in (-1, 19):
            with self.assertRaises(ValueError):
                DropLatch(channel=ch, threshold=1230, invert=True, stale_timeout=1.0)


if __name__ == "__main__":
    unittest.main()
