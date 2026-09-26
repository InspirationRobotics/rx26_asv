"""wall_fit_core on synthetic clouds: a dock edge, slip fingers, water, spray.

numpy only, no ROS:   python crusader_perception/test/test_wall_fit_core.py

The clouds are built in the LEVELLED frame the answer is defined in, then
rotated by the boat's roll and pitch into the body frame the fit receives, so
"the range does not change when the boat rocks" is tested, not assumed.
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_perception import lidar_cluster_core as lcc   # noqa: E402
from crusader_perception import wall_fit_core as wf         # noqa: E402

WATER = 0.24          # the waterline above the hull-bottom datum (the YAML's water_z)


def unlevel(lev, roll, pitch):
    """Inverse of lidar_cluster_core.level: levelled -> body."""
    R = lcc.level(np.eye(3), roll, pitch).T        # level(p) = R p, row-wise
    return lev @ R                                  # R^T p, row-wise


def dock(range_m=1.2, angle_deg=0.0, lat=0.0, fingers=True, water=True,
         spray=False, noise=0.008, seed=0, edge_top=0.35):
    """Levelled points. The wall's nearest point is `range_m` away at bearing
    `angle_deg`; the slip centre is `lat` to the RIGHT of the boat (so the boat
    sits `lat` LEFT of centre), fingers 0.75 either side of it, 2 m long."""
    rng = np.random.default_rng(seed)
    a = math.radians(angle_deg)
    n = np.array([math.cos(a), math.sin(a)])
    t = np.array([-n[1], n[0]])                      # along the wall, + left
    foot = range_m * n
    pts = []
    # the dock edge: the MID360 sees from the water up to ~edge_top above it
    for u in np.arange(-2.5, 2.5, 0.015):
        for z in np.arange(0.02, edge_top, 0.03):
            p = foot + u * t
            pts.append((p[0], p[1], WATER + z))
    if fingers:
        for side in (+1, -1):
            u_edge = -lat + side * 0.75
            for v in np.arange(0.05, 2.0, 0.02):
                for z in np.arange(0.02, 0.25, 0.04):
                    for w in (0.0, 0.1, 0.2, 0.3, 0.4, 0.5):     # finger 0.5 wide
                        p = foot - v * n + (u_edge + side * w) * t
                        pts.append((p[0], p[1], WATER + z))
    pts = np.array(pts)
    pts += rng.normal(0, noise, pts.shape)
    extra = []
    if water:                                         # the water surface itself
        k = 3000
        xy = rng.uniform([0.3, -2.0], [range_m, 2.0], (k, 2))
        extra.append(np.column_stack([xy, WATER + rng.normal(0, 0.01, k)]))
    if spray:                                         # the stream, in the air
        k = 800
        x = rng.uniform(0.4, range_m - 0.05, k)
        extra.append(np.column_stack([x, rng.normal(0, 0.08, k),
                                      WATER + rng.uniform(0.1, 0.9, k)]))
    if extra:
        pts = np.vstack([pts] + extra)
    return pts


def raycast_dock(range_m, lat=0.0, setback=0.0, h=0.28, step_deg=0.4, noise=0.008, seed=0):
    """A sweep ray-cast against the build guide's dock from OUTSIDE the slip.

    Unlike dock(), which lays points on a grid in space, this fires rays on an
    ANGULAR grid, as the MID360 does, so near things get more points than far
    ones - which is the whole question from 3.2 m out: the four slip fingers'
    TIP faces, 2 m nearer and collinear, against the dock edge behind them.

    Levelled frame: the sensor `h` above the water, the deck edge `range_m`
    ahead, the boat `lat` LEFT of its slip's centre. Fingers 0.5 m wide and 2 m
    long, slips 1.5 m, all 0.3 m above the water; 1 m panels on the deck,
    `setback` behind its edge. The sensor sees from 7 deg above horizontal
    down (mounted upside down)."""
    rng = np.random.default_rng(seed)
    az = np.radians(np.arange(-90.0, 90.0, step_deg))
    el = np.radians(np.arange(-52.0, 7.0, step_deg))
    A, E = np.meshgrid(az, el)
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    o = np.array([0.0, 0.0, h])
    boxes = []                                     # (lo, hi) in (x fwd, y left, z up above water)
    for c in (-3.0, -1.0, 1.0, 3.0):               # finger centres from the slip centre
        yc = c - lat                               # the slip centre is `lat` to the boat's RIGHT
        boxes.append(((range_m - 2.0, yc - 0.25, 0.0), (range_m, yc + 0.25, 0.3)))
    boxes.append(((range_m, -5.0, 0.0), (range_m + 1.0, 5.0, 0.3)))           # the deck
    for c in (-2.0, 0.0, 2.0):                                                # the panels
        yc = c - lat
        boxes.append(((range_m + setback, yc - 0.5, 0.3), (range_m + setback + 0.02, yc + 0.5, 1.3)))
    t_best = np.full(d.shape[0], np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for lo, hi in boxes:
            lo, hi = np.array(lo), np.array(hi)
            t1, t2 = (lo - o) / d, (hi - o) / d
            tn = np.nanmax(np.minimum(t1, t2), axis=1)
            tf = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tn <= tf) & (tf > 0)
            t_best = np.where(hit & (tn > 0) & (tn < t_best), tn, t_best)
        tw = np.where(d[:, 2] < 0, -h / d[:, 2], np.inf)                      # the water
    water = tw < t_best
    t_best = np.minimum(t_best, tw)
    keep = np.isfinite(t_best) & (t_best < 8.0)
    pts = o + d[keep] * t_best[keep, None]
    pts = pts[~water[keep] | (rng.random(keep.sum()) < 0.05)]   # water returns little
    pts[:, 2] += WATER                                          # hull-bottom datum
    return pts + rng.normal(0, noise, pts.shape)


class TestOutsideTheSlip(unittest.TestCase):
    """The fixed nozzle fires from ~3.2 m: 1.2 m OUTSIDE the finger tips. The
    fit must range the dock edge there, not the tips (which read 2 m short and
    would put the shot 2 m out). task3_fire_test.xml also cross-checks the
    camera's range (range_check_m) in case this ever goes wrong on the water."""

    def check_edge(self, fit, range_m):
        self.assertTrue(fit.valid, fit.why)
        self.assertAlmostEqual(fit.range_m, range_m, delta=0.03,
                               msg="fitted %.2f m: %s" % (fit.range_m,
                                   "the finger TIPS" if abs(fit.range_m - (range_m - 2.0)) < 0.1
                                   else "something else"))

    def test_panel_on_the_edge(self):
        self.check_edge(wf.fit(raycast_dock(3.2), WATER), 3.2)

    def test_panel_set_back(self):
        self.check_edge(wf.fit(raycast_dock(3.2, setback=0.5), WATER), 3.2)

    def test_off_the_centreline(self):
        self.check_edge(wf.fit(raycast_dock(3.2, lat=0.3), WATER), 3.2)

    def test_one_tip_in_the_sector(self):
        # 0.3 m off the centreline at 3.0 m only ONE finger tip is inside the
        # +-40 deg sector: no gap to see through, but it is too short to be
        # the wall, and the dock edge is behind it
        for sb in (0.0, 0.8):
            self.check_edge(wf.fit(raycast_dock(3.0, lat=0.3, setback=sb), WATER), 3.0)

    def test_every_range_ranges_the_edge(self):
        # the whole approach, both panel positions: the SAME surface throughout,
        # which is what makes a calibrated range mean anything
        for sb in (0.0, 0.8):
            for r in (1.0, 1.8, 2.4, 2.8, 3.4, 3.8):
                self.check_edge(wf.fit(raycast_dock(r, lat=0.15, setback=sb, step_deg=0.5,
                                                    seed=int(r * 10)), WATER), r)

    def test_the_fingers_still_give_the_offset(self):
        fit = wf.fit(raycast_dock(3.2, lat=0.2), WATER)
        self.assertAlmostEqual(fit.lat_m, 0.2, delta=0.05)

    def test_inside_the_slip_unchanged(self):
        fit = wf.fit(raycast_dock(1.2), WATER)
        self.check_edge(fit, 1.2)
        self.assertTrue(math.isnan(fit.skipped_m))

    def test_the_skip_is_reported(self):
        fit = wf.fit(raycast_dock(3.2, setback=0.5), WATER)
        if not math.isnan(fit.skipped_m):            # only if the tips won RANSAC
            self.assertAlmostEqual(fit.skipped_m, 1.2, delta=0.05)

    def test_a_panel_seen_over_the_edge_is_not_a_gap(self):
        # inside the slip with the panel set back: the edge, not the panel
        self.check_edge(wf.fit(raycast_dock(1.2, setback=0.5), WATER), 1.2)
        self.check_edge(wf.fit(raycast_dock(2.5, setback=0.8), WATER), 2.5)


class TestWall(unittest.TestCase):

    def check(self, fit, range_m, angle_deg=0.0, tol=0.01):
        self.assertTrue(fit.valid, fit.why)
        self.assertAlmostEqual(fit.range_m, range_m, delta=tol)
        self.assertAlmostEqual(fit.angle_deg, angle_deg, delta=1.0)

    def test_square_on(self):
        self.check(wf.fit(dock(1.2), WATER), 1.2)

    def test_ranges_across_the_useful_band(self):
        for r in (0.6, 0.9, 1.5, 2.5):
            self.check(wf.fit(dock(r, seed=int(r * 10)), WATER), r)

    def test_angled_wall(self):
        for a in (-20.0, -8.0, 8.0, 20.0):
            fit = wf.fit(dock(1.2, angle_deg=a), WATER)
            self.check(fit, 1.2, a)

    def test_angle_sign_means_turn_left(self):
        # the boat yawed RIGHT of square: the wall's nearest point is to the LEFT
        fit = wf.fit(dock(1.2, angle_deg=10.0), WATER)
        self.assertGreater(fit.angle_deg, 0)

    def test_rocking_does_not_move_the_range(self):
        lev = dock(1.1)
        for roll, pitch in ((0.0, 0.0), (0.06, 0.0), (0.0, 0.06), (-0.05, 0.04)):
            body = unlevel(lev, roll, pitch)
            self.check(wf.fit(body, WATER, roll=roll, pitch=pitch), 1.1, tol=0.012)

    def test_unlevel_round_trips(self):
        lev = dock(1.0, fingers=False, water=False)[:50]
        body = unlevel(lev, 0.05, -0.03)
        back = lcc.level(body, 0.05, -0.03)
        self.assertLess(np.abs(back - lev).max(), 1e-9)

    def test_spray_does_not_pull_the_range_in(self):
        self.check(wf.fit(dock(1.2, spray=True), WATER), 1.2, tol=0.012)

    def test_water_returns_are_gated_out(self):
        fit = wf.fit(dock(1.2), WATER)
        self.assertTrue(fit.valid)
        self.assertLess(fit.rms_m, 0.02)

    def test_no_wall(self):
        rng = np.random.default_rng(1)
        k = 2000
        water = np.column_stack([rng.uniform(0.3, 3, k), rng.uniform(-2, 2, k),
                                 WATER + rng.normal(0, 0.01, k)])
        fit = wf.fit(water, WATER)
        self.assertFalse(fit.valid)
        self.assertTrue(fit.why)

    def test_a_short_stub_is_not_a_wall(self):
        pts = dock(1.2, fingers=False, water=False)
        stub = pts[np.abs(pts[:, 1]) < 0.1]       # 0.2 m of wall
        fit = wf.fit(stub, WATER)
        self.assertFalse(fit.valid)


class TestFingers(unittest.TestCase):

    def test_centred(self):
        fit = wf.fit(dock(1.2, lat=0.0), WATER)
        self.assertAlmostEqual(fit.left_m, 0.75, delta=0.03)
        self.assertAlmostEqual(fit.right_m, 0.75, delta=0.03)
        self.assertAlmostEqual(fit.lat_m, 0.0, delta=0.03)

    def test_boat_left_of_centre_is_positive(self):
        fit = wf.fit(dock(1.2, lat=0.2), WATER)
        self.assertAlmostEqual(fit.left_m, 0.55, delta=0.03)
        self.assertAlmostEqual(fit.right_m, 0.95, delta=0.03)
        self.assertAlmostEqual(fit.lat_m, 0.2, delta=0.03)

    def test_boat_right_of_centre_is_negative(self):
        self.assertAlmostEqual(wf.fit(dock(1.2, lat=-0.15), WATER).lat_m, -0.15,
                               delta=0.03)

    def test_no_fingers_no_offset(self):
        fit = wf.fit(dock(1.2, fingers=False), WATER)
        self.assertTrue(fit.valid)
        self.assertTrue(math.isnan(fit.lat_m))

    def test_fingers_with_the_wall_at_an_angle(self):
        fit = wf.fit(dock(1.2, angle_deg=10.0, lat=0.1), WATER)
        self.assertAlmostEqual(fit.lat_m, 0.1, delta=0.04)


class TestSensorFrame(unittest.TestCase):

    def test_through_the_upside_down_mount(self):
        """The same wall, as the MID360 reports it: sign flips and offsets
        undone by lidar_cluster_core.to_body before the fit."""
        cp = lcc.ClusterParams(sign_y=-1.0, sign_z=-1.0, tx=0.32, ty=0.05, tz=0.52)
        body = dock(1.3)
        sensor = np.column_stack([body[:, 0] - cp.tx,
                                  (body[:, 1] - cp.ty) * cp.sign_y,
                                  (body[:, 2] - cp.tz) * cp.sign_z])
        back = lcc.to_body(sensor, cp)
        self.assertLess(np.abs(back - body).max(), 1e-9)
        fit = wf.fit(back, WATER)
        self.assertTrue(fit.valid)
        self.assertAlmostEqual(fit.range_m, 1.3, delta=0.01)


class TestParams(unittest.TestCase):

    def test_params_from_needs_every_key(self):
        d = {k: v for k, v in wf.WallParams().__dict__.items()}
        self.assertEqual(wf.params_from(d), wf.WallParams())
        del d["tol_m"]
        with self.assertRaises(KeyError):
            wf.params_from(d)


if __name__ == "__main__":
    unittest.main(verbosity=2)
