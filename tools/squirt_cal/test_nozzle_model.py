"""nozzle_model against hand-worked numbers.   python test_nozzle_model.py"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from nozzle_model import NozzleModel, target_height   # noqa: E402


class TestNozzle(unittest.TestCase):

    def setUp(self):
        # 3 m on the level at 45 deg: v^2 = g R, height above the nozzle is
        # x - x^2 / 3, which tops out at 0.75 m, 1.5 m out.
        self.m = NozzleModel.from_range(3.0, 45.0, 0.40)

    def test_level_range_and_speed(self):
        self.assertAlmostEqual(self.m.v, math.sqrt(9.81 * 3.0), places=6)
        self.assertAlmostEqual(self.m.height_at(3.0), 0.40, places=6)

    def test_hand_worked_heights(self):
        for x, rise in ((0.5, 0.5 - 0.25 / 3), (1.0, 1 - 1 / 3), (1.5, 0.75)):
            self.assertAlmostEqual(self.m.height_at(x) - 0.40, rise, places=6)

    def test_apex(self):
        x, z = self.m.apex()
        self.assertAlmostEqual(x, 1.5, places=6)
        self.assertAlmostEqual(z, 1.15, places=6)

    def test_solve_both_branches(self):
        z = 0.40 + 1 - 1 / 3                       # the height at x = 1 and x = 2
        self.assertAlmostEqual(self.m.solve_x(z, "near"), 1.0, places=6)
        self.assertAlmostEqual(self.m.solve_x(z, "far"), 2.0, places=6)
        self.assertGreater(self.m.slope_at(1.0), 0)
        self.assertLess(self.m.slope_at(2.0), 0)

    def test_out_of_reach_is_none(self):
        self.assertIsNone(self.m.solve_x(1.20))    # apex is 1.15

    def test_behind_the_nozzle_is_none(self):
        self.assertIsNone(self.m.solve_x(0.10, "near"))

    def test_bow_up_raises_the_stream(self):
        self.assertGreater(self.m.height_at(1.0, pitch_deg=2.0), self.m.height_at(1.0))
        # about 1.3 m per radian at 1 m out: ~2.3 cm per degree
        d = self.m.height_at(1.0, 1.0) - self.m.height_at(1.0)
        self.assertAlmostEqual(d, 0.023, delta=0.003)

    def test_target_height_is_deck_plus_edge(self):
        self.assertAlmostEqual(target_height(0.30, 895.0), 1.195, places=9)

    def test_bad_inputs(self):
        with self.assertRaises(ValueError):
            NozzleModel.from_range(0.0, 45.0, 0.4)
        with self.assertRaises(ValueError):
            NozzleModel(45.0, 0.0, 0.4)


if __name__ == "__main__":
    unittest.main(verbosity=2)
