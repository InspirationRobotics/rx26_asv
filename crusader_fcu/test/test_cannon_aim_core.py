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

    def test_landing_below_the_nozzle(self):
        # 9 ft at 30 deg onto the water 0.51 m below the nozzle (2026-10-11)
        v = ca.exit_speed_from_range(2.743, 30.0, 0.51)
        self.assertAlmostEqual(v, 4.85, delta=0.01)
        self.assertAlmostEqual(ca.exit_speed_from_range(2.743, 30.0, 0.0),
                               math.sqrt(9.81 * 2.743 / math.sin(math.radians(60))), places=9)
        # ...and that stream does come down 0.51 m below, 2.743 m out
        hit = ca.crossing((0, 0, 0), ca.direction(0, 30), v, (0, 0, -0.51), (0, 0, 1))
        self.assertAlmostEqual(hit[1][0], 2.743, places=3)


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


class TestWindowTrim(unittest.TestCase):

    def test_window_names(self):
        self.assertEqual(ca.window_slot("UL"), "UL")
        self.assertEqual(ca.window_slot("lr"), "LR")
        self.assertEqual(ca.window_slot(0), "UL")
        self.assertEqual(ca.window_slot(1), "LR")
        for v in (None, "", "XX", 2, -1, True, 1.5):
            self.assertIsNone(ca.window_slot(v), v)

    def test_a_windows_trims(self):
        cp = ca.CannonParams()
        target = (1.6, -0.2, 1.0)
        base = ca.solve(target, cp)
        trimmed = ca.solve(target, ca.with_trims(cp, 3.0, 5.0))
        self.assertAlmostEqual(trimmed.tilt_deg - base.tilt_deg, 5.0, places=6)
        self.assertAlmostEqual(trimmed.pan_deg - base.pan_deg, 3.0, places=6)
        self.assertEqual((cp.pan_trim_deg, cp.tilt_trim_deg), (0.0, 0.0))   # not in place

    def test_params_without_the_old_common_trims(self):
        d = dict(nozzle_x=0.47, nozzle_y=0.3, nozzle_z=0.75, cam_x=0.37, cam_y=0.0, cam_z=0.65,
                 cam_yaw_deg=0.0, cam_pitch_deg=0.0, exit_speed_mps=4.2, throw_range_m=3.0,
                 throw_elev_deg=45.0, pan_center_us=1500, pan_us_per_deg=5.556, pan_sign=1,
                 tilt_center_us=1680, tilt_us_per_deg=5.556, tilt_sign=-1, pwm_min=1100,
                 pwm_max=1900, pan_min_deg=-60, pan_max_deg=60, tilt_min_deg=-10, tilt_max_deg=60)
        cp = ca.CannonParams.from_dict(d)
        self.assertEqual((cp.pan_trim_deg, cp.tilt_trim_deg), (0.0, 0.0))


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

    def test_settle_then_the_water_stays_on(self):
        g = ca.FireGate(ca.FireParams(settle_s=0.4, burst_s=1.0, resend_s=0.3))
        sol = ca.AimSolution(ok=True, reachable=True)
        self.assertEqual(g.update(0.0, True, 0.1, sol, 3.0), 0.0)     # settling
        self.assertEqual(g.update(0.3, True, 0.1, sol, 3.0), 0.0)
        self.assertEqual(g.update(0.45, True, 0.1, sol, 3.0), 1.0)    # water ON
        # the servos track the target while it sprays: no stop, no re-settle
        self.assertEqual(g.update(0.6, True, 0.1, sol, 40.0), 0.0)
        self.assertEqual(g.why, "spraying")
        asks = [0.45]
        t = 0.6
        while t < 10.0:                                              # 50 Hz ticks
            if g.update(t, True, 0.1, sol, 40.0 if int(t) % 2 else 3.0) > 0.0:
                asks.append(t)
            t = round(t + 0.02, 2)
        # it never left a burst to run out (and two asks per burst: one lost
        # on the way to the autopilot does not stop the water)
        self.assertGreater(len(asks), 25)
        for a, b in zip(asks, asks[1:]):
            self.assertLessEqual(b - a, 0.5 + 1e-9)

    def test_a_hard_reason_stops_it(self):
        g = ca.FireGate(ca.FireParams(settle_s=0.4, burst_s=1.0, resend_s=0.3))
        sol = ca.AimSolution(ok=True, reachable=True)
        g.update(0.0, True, 0.1, sol, 3.0)
        self.assertEqual(g.update(0.45, True, 0.1, sol, 3.0), 1.0)
        far = ca.AimSolution(ok=True, reachable=False, why="out of reach")
        self.assertEqual(g.update(0.6, True, 0.1, far, 3.0), 0.0)
        self.assertFalse(g.firing)
        # back in reach: settles again before the water goes back on
        self.assertEqual(g.update(0.8, True, 0.1, sol, 3.0), 0.0)
        self.assertEqual(g.why, "settling")
        self.assertEqual(g.update(1.25, True, 0.1, sol, 3.0), 1.0)
        # the servos far off the command before it is on: it waits
        g.reset()
        self.assertEqual(g.update(2.0, True, 0.1, sol, 40.0), 0.0)
        self.assertIn("moving", g.why)
        # the tree says stop (GREEN)
        g.update(2.0, True, 0.1, sol, 3.0)
        self.assertEqual(g.update(2.45, True, 0.1, sol, 3.0), 1.0)
        self.assertEqual(g.update(2.5, False, 0.1, sol, 3.0), 0.0)
        self.assertFalse(g.firing)

    def test_resend_must_beat_the_burst(self):
        with self.assertRaises(ValueError):
            ca.FireGate(ca.FireParams(burst_s=1.0, resend_s=1.0))


class TestGeometricTarget(unittest.TestCase):
    """The no-trim aim point (2026-10-18): the camera's bearing, the LiDAR's face
    plane, the window's known height."""

    def setUp(self):
        self.cp = ca.CannonParams(nozzle_x=0.37, nozzle_y=0.279, nozzle_z=0.75,
                                  cam_x=0.37, cam_y=0.0, cam_z=0.465,
                                  exit_speed_mps=ca.exit_speed_from_range(2.74, 30.0, 0.51))

    def cam_of(self, body, scale=1.0, dz=0.0):
        """What the camera would report for a body point: the exact point
        (camera_link), its range scaled (stereo error) and its height moved
        (the visible part of a cut-off window)."""
        x, y, z = (body[0] - self.cp.cam_x, body[1] - self.cp.cam_y, body[2] - self.cp.cam_z)
        return (x * scale, y * scale, z * scale + dz)

    def assertNear(self, a, b, tol=1e-6):
        for u, v in zip(a, b):
            self.assertAlmostEqual(u, v, delta=tol)

    def test_square_to_the_face(self):
        # the face 1.6 m ahead of the body origin, the LR window 0.3 m left, 0.81 up
        truth = (1.6, 0.3, 0.24 + 0.81)
        cam = self.cam_of(truth, scale=1.10, dz=-0.06)   # 10 % long, 6 cm low
        got, used = ca.geometric_target(cam, self.cp, plane=ca.plane_from_wall(1.6, 0.0),
                                        height_above_water=0.81, waterline_z=0.24)
        self.assertEqual(used, "lidar+height")
        self.assertNear(got, truth, 0.005)               # the low height moves the bearing a hair

    def test_turned_off_the_face(self):
        a = math.radians(-8.0)                           # the wall's perpendicular 8 deg right
        n = (math.cos(a), math.sin(a))
        d = 1.5
        foot = (d * n[0], d * n[1])
        along = (-n[1], n[0])                            # along the wall, + left
        truth = (foot[0] + 0.25 * along[0], foot[1] + 0.25 * along[1], 0.24 + 1.05)
        cam = self.cam_of(truth, scale=0.93)
        got, used = ca.geometric_target(cam, self.cp, plane=ca.plane_from_wall(d, -8.0),
                                        height_above_water=1.05)
        self.assertEqual(used, "lidar+height")
        self.assertNear(got, truth)

    def test_rolled_and_pitched(self):
        roll, pitch = math.radians(4.0), math.radians(-3.0)
        lev = (1.4, -0.2, 0.24 + 0.81)                   # the window in the levelled frame
        body = ca.unlevel(lev, roll, pitch)              # where the hull-fixed camera sees it
        cam = self.cam_of(body, scale=1.08)
        got, used = ca.geometric_target(cam, self.cp, roll, pitch,
                                        plane=ca.plane_from_wall(1.4, 0.0),
                                        height_above_water=0.81)
        self.assertEqual(used, "lidar+height")
        self.assertNear(got, body)

    def test_the_setback(self):
        truth = (1.635, 0.0, 0.24 + 0.81)
        got, _ = ca.geometric_target(self.cam_of(truth, scale=1.05), self.cp,
                                     plane=ca.plane_from_wall(1.6, 0.0),
                                     height_above_water=0.81, setback_m=0.035)
        self.assertNear(got, truth)

    def test_the_slip_plane_is_the_lidars(self):
        n, d = ca.plane_from_slot(1.25, 0.0, (0.32, 0.05))
        self.assertEqual(n, (1.0, 0.0))
        self.assertAlmostEqual(d, 1.57)

    def test_a_plane_on_the_finger_tips_is_refused(self):
        truth = (2.2, 0.1, 0.24 + 0.81)
        cam = self.cam_of(truth)
        got, used = ca.geometric_target(cam, self.cp, plane=ca.plane_from_wall(0.4, 0.0),
                                        height_above_water=0.81)
        self.assertEqual(used, "height")                 # the camera's range kept
        self.assertNear(got, truth)

    def test_nothing_known_is_the_camera(self):
        truth = (1.6, 0.3, 1.0)
        got, used = ca.geometric_target(self.cam_of(truth), self.cp)
        self.assertEqual(used, "camera")
        self.assertNear(got, truth)

    def test_the_water_goes_through_the_window_centre(self):
        """The whole chain: a wrong stereo range and a low height in, the stream
        through the window's centre out (within 1 cm), from several places."""
        for d, lat, h in ((1.57, 0.0, 0.81), (1.9, 0.3, 0.81), (1.6, -0.3, 1.05)):
            truth = (d, lat, 0.24 + h)
            cam = self.cam_of(truth, scale=1.12, dz=-0.05)
            tgt, _ = ca.geometric_target(cam, self.cp, plane=ca.plane_from_wall(d, 0.0),
                                         height_above_water=h)
            sol = ca.solve(tgt, self.cp)
            self.assertTrue(sol.reachable and not sol.clipped, (d, lat, h, sol.why))
            self.assertLess(fly_to_plane_x(self.cp, sol, truth, 0.0, 0.0), 0.01, (d, lat, h))

    def test_the_upper_left_from_2_3_m_is_out_of_reach(self):
        """At the measured 4.85 m/s the upper-left window (1.05 m up) is beyond the
        stream from 2.3 m: it must be marked so (and FireGate refuses), not fired at."""
        truth = (2.3, -0.3, 0.24 + 1.05)
        tgt, _ = ca.geometric_target(self.cam_of(truth), self.cp,
                                     plane=ca.plane_from_wall(2.3, 0.0), height_above_water=1.05)
        sol = ca.solve(tgt, self.cp)
        self.assertFalse(sol.reachable)
        self.assertIn("out of reach", sol.why)


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
