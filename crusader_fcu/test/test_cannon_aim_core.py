"""cannon_aim_core: the aim, checked by flying the water.

stdlib only:   python crusader_fcu/test/test_cannon_aim_core.py

The central test is a round trip: solve for a target, then fly a drag-free
stream along the solved pan/tilt (through the same roll and pitch) and see where
it crosses the target's plane. The solver and the flight are separate code
paths (the solver works in angles, the flight in vectors), so a sign or frame
error in either shows up as a miss.
"""
import math
import os
import random
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_fcu import cannon_aim_core as ca   # noqa: E402


def fly_to_plane_x(cp, sol, target, roll, pitch):
    """Fly the solved stream (world = levelled body, origin the pivot) to the
    vertical plane through `target` facing the boat; the miss vector there."""
    tw = ca.level((target[0] - cp.nozzle_x, target[1] - cp.nozzle_y, target[2] - cp.nozzle_z),
                  roll, pitch)
    dw = ca.level(ca.direction(sol.pan_deg, sol.tilt_deg), roll, pitch)
    # the plane through the target, square to the horizontal line of sight
    h = math.hypot(tw[0], tw[1])
    n = (tw[0] / h, tw[1] / h, 0.0)
    hit = ca.crossing((0.0, 0.0, 0.0), dw, cp.exit_speed_mps, tw, n)
    assert hit is not None
    p = hit[1]
    return math.dist(p, tw)


class TestExitSpeed(unittest.TestCase):

    def test_three_metres_at_45(self):
        v = ca.exit_speed_from_range(3.0, 45.0)
        self.assertAlmostEqual(v, math.sqrt(9.81 * 3.0), places=6)       # 5.42 m/s
        # and it does throw 3.0 m back at the same height
        hit = ca.crossing((0, 0, 0), ca.direction(0, 45), v, (0, 0, 0), (0, 0, 1))
        self.assertAlmostEqual(hit[1][0], 3.0, places=6)


class TestServoMap(unittest.TestCase):

    def test_round_trip_and_limits(self):
        s = ca.ServoMap()
        for deg in (-60, -10, 0, 7.5, 40):
            self.assertAlmostEqual(s.to_deg(s.to_pwm(deg)), deg, delta=0.2)
        lo, hi = s.deg_limits()
        self.assertAlmostEqual(lo, -72.0, delta=0.01)       # 1100 us
        self.assertAlmostEqual(hi, 72.0, delta=0.01)        # 1900 us
        self.assertEqual(s.to_pwm(200), 1900)
        self.assertEqual(s.to_pwm(-200), 1100)

    def test_reversed_servo(self):
        s = ca.ServoMap(sign=-1.0)
        self.assertLess(s.to_pwm(20.0), 1500)


class TestSolve(unittest.TestCase):

    def test_signs(self):
        cp = ca.CannonParams()
        left = ca.solve((1.9, cp.nozzle_y + 0.4, cp.nozzle_z), cp)
        self.assertGreater(left.pan_deg, 5.0)              # + pan = LEFT
        high = ca.solve((1.9, cp.nozzle_y, cp.nozzle_z + 0.6), cp)
        low = ca.solve((1.9, cp.nozzle_y, cp.nozzle_z), cp)
        self.assertGreater(high.tilt_deg, low.tilt_deg)
        self.assertGreater(low.tilt_deg, 0.0)              # level target: aim UP for the droop

    def test_round_trip_level(self):
        cp = ca.CannonParams()
        for x, y, z in ((1.94, 0.22, 1.29), (1.94, -0.23, 1.05), (2.5, 0.0, 1.0), (1.2, 0.5, 0.9)):
            with self.subTest(target=(x, y, z)):
                sol = ca.solve((x, y, z), cp)
                self.assertTrue(sol.reachable and not sol.clipped, sol.why)
                self.assertLess(fly_to_plane_x(cp, sol, (x, y, z), 0.0, 0.0), 0.01)

    def test_round_trip_rocking(self):
        """The boat rolled and pitched: the solution still lands it."""
        cp = ca.CannonParams()
        rnd = random.Random(4)
        for _ in range(200):
            tgt = (rnd.uniform(1.3, 2.6), rnd.uniform(-0.6, 0.6), rnd.uniform(0.8, 1.4))
            roll, pitch = math.radians(rnd.uniform(-6, 6)), math.radians(rnd.uniform(-6, 6))
            sol = ca.solve(tgt, cp, roll, pitch)
            if not sol.reachable or sol.clipped:
                continue
            self.assertLess(fly_to_plane_x(cp, sol, tgt, roll, pitch), 0.01)

    def test_what_ignoring_attitude_costs(self):
        """The camera measures the window in the HULL frame, so a pitched hull
        carries the target with it and only gravity's direction changes. At
        dock range that is small - ~1.5 cm at 8 deg - but it is not zero, and
        the levelled solution removes it."""
        cp = ca.CannonParams()
        tgt = (1.94, 0.0, 1.2)
        pitch = math.radians(8.0)
        naive = ca.solve(tgt, cp)
        self.assertGreater(fly_to_plane_x(cp, naive, tgt, 0.0, pitch), 0.012)
        right = ca.solve(tgt, cp, 0.0, pitch)
        self.assertLess(fly_to_plane_x(cp, right, tgt, 0.0, pitch), 0.002)

    def test_out_of_reach(self):
        cp = ca.CannonParams()
        sol = ca.solve((8.0, 0.0, 1.0), cp)
        self.assertFalse(sol.reachable)
        self.assertIn("out of reach", sol.why)

    def test_limits_clip(self):
        cp = ca.CannonParams(pan_max_deg=20.0)
        sol = ca.solve((1.0, 1.5, 0.9), cp)                # far to the left
        self.assertTrue(sol.clipped)
        self.assertAlmostEqual(sol.pan_deg, 20.0)


class TestCamera(unittest.TestCase):

    def test_cam_to_body(self):
        cp = ca.CannonParams(cam_x=0.37, cam_y=0.0, cam_z=0.65)
        b = ca.cam_to_body((1.5, 0.2, 0.3), cp)
        self.assertAlmostEqual(b[0], 1.87)
        self.assertAlmostEqual(b[1], 0.2)
        self.assertAlmostEqual(b[2], 0.95)
        up = ca.CannonParams(cam_pitch_deg=-10.0)          # aimed UP 10 deg
        b = ca.cam_to_body((2.0, 0.0, 0.0), up)            # on the optical axis
        self.assertAlmostEqual(b[2] - up.cam_z, 2.0 * math.sin(math.radians(10.0)), places=6)

    def test_level_unlevel_inverse(self):
        v = (0.3, -1.2, 0.7)
        w = ca.unlevel(ca.level(v, 0.1, -0.07), 0.1, -0.07)
        for a, b in zip(v, w):
            self.assertAlmostEqual(a, b, places=9)


class TestFireGate(unittest.TestCase):

    def test_settle_then_bursts(self):
        g = ca.FireGate(ca.FireParams(settle_s=0.4, burst_s=1.0, gap_s=1.1))
        sol = ca.AimSolution(ok=True, reachable=True)
        self.assertEqual(g.update(0.0, True, 0.1, sol, 3.0), 0.0)     # settling
        self.assertEqual(g.update(0.3, True, 0.1, sol, 3.0), 0.0)
        self.assertEqual(g.update(0.45, True, 0.1, sol, 3.0), 1.0)    # FIRE
        self.assertEqual(g.update(1.0, True, 0.1, sol, 3.0), 0.0)     # between
        self.assertEqual(g.update(2.6, True, 0.1, sol, 3.0), 1.0)     # next

    def test_refusals(self):
        g = ca.FireGate()
        ok = ca.AimSolution(ok=True, reachable=True)
        self.assertEqual(g.update(0, False, 0.1, ok, 0), 0.0)
        self.assertEqual(g.update(1, True, 5.0, ok, 0), 0.0)
        self.assertIn("old", g.why)
        self.assertEqual(g.update(2, True, 0.1, ok, 50.0), 0.0)
        self.assertIn("moving", g.why)
        self.assertEqual(g.update(3, True, 0.1, ok, None), 0.0)
        self.assertIn("unknown", g.why)
        far = ca.AimSolution(ok=True, reachable=False, why="out of reach")
        self.assertEqual(g.update(4, True, 0.1, far, 0), 0.0)


class TestServoLimiter(unittest.TestCase):

    def test_clamp_rate_and_step(self):
        s = ca.ServoLimiter(1100, 1900, min_period_s=0.05, min_step_us=2)
        self.assertEqual(s.request(2500, 0.0), 1900)
        self.assertIsNone(s.request(1500, 0.01))           # too soon...
        self.assertEqual(s.due(0.06), 1500)                # ...sent from the tick
        self.assertIsNone(s.request(1501, 0.2))            # a 1 us change: not worth it
        self.assertIsNone(s.request(0, 0.3))               # 0 = leave it alone


if __name__ == "__main__":
    unittest.main()
