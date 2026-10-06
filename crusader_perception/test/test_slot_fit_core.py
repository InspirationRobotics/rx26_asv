"""slot_fit_core on ray-cast sweeps of the build guide's dock.

numpy only, no ROS:   python crusader_perception/test/test_slot_fit_core.py

The sweep is fired on the MID360's angular grid (front 180 deg, 7 deg up to 52
deg down, upside down at 0.28 m above the water) against boxes laid out in the
SLIP's frame, then turned into the body frame by the truth the test sets: the
slip's angle off the bow, the centreline's offset, and the LiDAR-to-back-wall
standoff. So every number the fit returns is checked against a pose, not
against the generator's own arithmetic.
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_perception import lidar_cluster_core as lcc   # noqa: E402
from crusader_perception import slot_fit_core as sf         # noqa: E402

WATER = 0.24          # the waterline above the hull-bottom datum (the YAML's water_z)
LX, LY, LH = 0.32, 0.05, 0.28     # the LiDAR in the body frame, and above the water


def rot(xy, deg):
    a = math.radians(deg)
    c, s = math.cos(a), math.sin(a)
    return np.column_stack([c * xy[:, 0] - s * xy[:, 1], s * xy[:, 0] + c * xy[:, 1]])


def raycast_slip(back_m=1.6, lateral_m=0.0, angle_deg=0.0, deck=0.3, step_deg=0.5,
                 noise=0.008, seed=0, hide_left=False, roll=0.0, pitch=0.0):
    """Body-frame points of one sweep. Truth: the slip's axis is `angle_deg`
    LEFT of the bow, its centreline `lateral_m` LEFT of the body origin, and the
    deck edge `back_m` from the LiDAR along the axis. Fingers 0.5 x 2.0 m either
    side of 1.5 m slips (four of them), a 1 m deck, panels on it; all `deck` high.
    `roll`/`pitch` tilt the boat: the cloud is returned UNlevelled, as the
    sensor would see it."""
    rng = np.random.default_rng(seed)
    az = np.radians(np.arange(-90.0, 90.0, step_deg))
    el = np.radians(np.arange(-52.0, 7.0, step_deg))
    A, E = np.meshgrid(az, el)
    d = np.stack([np.cos(E) * np.cos(A), np.cos(E) * np.sin(A), np.sin(E)], -1).reshape(-1, 3)
    # into the slip frame S (x' along the axis, y' left, origin at the body origin)
    o_xy = rot(np.array([[LX, LY]]), -angle_deg)[0]
    d_xy = rot(d[:, :2], -angle_deg)
    o = np.array([o_xy[0], o_xy[1], LH])
    dS = np.column_stack([d_xy, d[:, 2]])
    xw = o[0] + back_m                                   # the deck edge in S
    boxes = []
    for k, c in enumerate((-3.0, -1.0, 1.0, 3.0)):       # finger centres about the slip centre
        if hide_left and k == 2:
            continue
        yc = lateral_m + c * 1.0                         # 2.0 m pitch: +-1.0 is our slip's
        boxes.append(((xw - 2.0, yc - 0.25, 0.0), (xw, yc + 0.25, deck)))
    boxes.append(((xw, lateral_m - 5.0, 0.0), (xw + 1.0, lateral_m + 5.0, deck)))
    for c in (-2.0, 0.0, 2.0):
        yc = lateral_m + c
        boxes.append(((xw + 0.035, yc - 0.5, deck), (xw + 0.065, yc + 0.5, deck + 1.0)))
    t_best = np.full(dS.shape[0], np.inf)
    with np.errstate(divide="ignore", invalid="ignore"):
        for lo, hi in boxes:
            lo, hi = np.array(lo), np.array(hi)
            t1, t2 = (lo - o) / dS, (hi - o) / dS
            tn = np.nanmax(np.minimum(t1, t2), axis=1)
            tf = np.nanmin(np.maximum(t1, t2), axis=1)
            hit = (tn <= tf) & (tf > 0)
            t_best = np.where(hit & (tn > 0) & (tn < t_best), tn, t_best)
        tw = np.where(dS[:, 2] < 0, -LH / dS[:, 2], np.inf)
    water = tw < t_best
    t_best = np.minimum(t_best, tw)
    keep = np.isfinite(t_best) & (t_best < 8.0)
    pS = o + dS[keep] * t_best[keep, None]
    pS = pS[~water[keep] | (rng.random(keep.sum()) < 0.05)]
    body = np.column_stack([rot(pS[:, :2], angle_deg), pS[:, 2] + WATER])
    body += rng.normal(0, noise, body.shape)
    if roll or pitch:
        R = lcc.level(np.eye(3), roll, pitch).T          # level(p) = R p, row-wise
        body = body @ R                                  # R^T p: levelled -> body
    return body


def fit(**kw):
    roll, pitch = kw.get("roll", 0.0), kw.get("pitch", 0.0)
    return sf.fit(raycast_slip(**kw), WATER, sf.SlotParams(lidar_x=LX, lidar_y=LY),
                  roll=roll, pitch=pitch)


class TestInTheSlip(unittest.TestCase):
    """The cannon tree's berth: 1.6 m from the LiDAR to the back wall."""

    def test_centred_square(self):
        f = fit()
        self.assertTrue(f.valid, f.why)
        self.assertAlmostEqual(f.back_range_m, 1.6, delta=0.03)
        self.assertAlmostEqual(f.lateral_m, 0.0, delta=0.03)
        self.assertAlmostEqual(f.angle_deg, 0.0, delta=1.5)
        self.assertAlmostEqual(f.width_m, 1.5, delta=0.06)
        self.assertTrue(f.has_left and f.has_right and f.has_back)

    def test_offsets_and_yaw(self):
        for lat in (-0.25, 0.25):
            for ang in (-8.0, 8.0):
                with self.subTest(lat=lat, ang=ang):
                    f = fit(lateral_m=lat, angle_deg=ang, seed=3)
                    self.assertTrue(f.valid, f.why)
                    self.assertAlmostEqual(f.lateral_m, lat, delta=0.04)
                    self.assertAlmostEqual(f.angle_deg, ang, delta=1.5)
                    self.assertAlmostEqual(f.back_range_m, 1.6, delta=0.04)

    def test_signs(self):
        """+ lateral = the slip is LEFT (strafe left); + angle = it points LEFT."""
        f = fit(lateral_m=0.2, angle_deg=5.0)
        self.assertGreater(f.lateral_m, 0.1)
        self.assertGreater(f.angle_deg, 2.0)
        f = fit(lateral_m=-0.2, angle_deg=-5.0)
        self.assertLess(f.lateral_m, -0.1)
        self.assertLess(f.angle_deg, -2.0)

    def test_range_is_from_the_lidar(self):
        """Not the body origin: the two differ by lidar_x (0.32 m)."""
        for r in (1.2, 1.6, 2.0):
            with self.subTest(r=r):
                self.assertAlmostEqual(fit(back_m=r).back_range_m, r, delta=0.03)

    def test_rocking(self):
        f = fit(roll=math.radians(4.0), pitch=math.radians(-3.0), lateral_m=0.1)
        self.assertTrue(f.valid, f.why)
        self.assertAlmostEqual(f.back_range_m, 1.6, delta=0.04)
        self.assertAlmostEqual(f.lateral_m, 0.1, delta=0.04)


class TestOutsideTheSlip(unittest.TestCase):
    """Where part 1 hands over: 3.5 m from the face to the body origin."""

    def test_ranges_the_deck_edge_not_the_tips(self):
        f = fit(back_m=3.18, lateral_m=-0.15, angle_deg=4.0)
        self.assertTrue(f.valid, f.why)
        self.assertAlmostEqual(f.back_range_m, 3.18, delta=0.05)    # tips are 2 m nearer
        self.assertAlmostEqual(f.lateral_m, -0.15, delta=0.05)
        self.assertAlmostEqual(f.angle_deg, 4.0, delta=2.0)


class TestOneWall(unittest.TestCase):

    def test_left_finger_missing(self):
        f = fit(hide_left=True, lateral_m=0.1)
        self.assertTrue(f.valid, f.why)
        self.assertFalse(f.has_left)
        self.assertTrue(f.has_right)
        self.assertIn("right wall only", f.why)
        self.assertAlmostEqual(f.lateral_m, 0.1, delta=0.05)


class TestNoisyLidar(unittest.TestCase):
    """Gazebo's MID-360 has 2 cm of range noise (crusader_hull.yaml), and a few
    water returns get through. Strays that line up across the slip must not
    become a back wall a quarter of a metre ahead (they did, before the
    coverage test)."""

    def test_random_poses_two_cm_noise(self):
        import random
        rnd = random.Random(5)
        for i in range(25):
            b, lat, ang = rnd.uniform(1.2, 3.3), rnd.uniform(-0.3, 0.3), rnd.uniform(-12, 12)
            with self.subTest(back=round(b, 2), lat=round(lat, 2), ang=round(ang, 1)):
                f = sf.fit(raycast_slip(back_m=b, lateral_m=lat, angle_deg=ang, seed=i, noise=0.02),
                           WATER, sf.SlotParams(lidar_x=LX, lidar_y=LY), seed=i)
                self.assertTrue(f.valid, f.why)
                self.assertAlmostEqual(f.back_range_m, b, delta=0.04)
                self.assertAlmostEqual(f.lateral_m, lat, delta=0.05)
                self.assertAlmostEqual(f.angle_deg, ang, delta=2.0)


class TestNoSlip(unittest.TestCase):

    def test_open_water(self):
        rng = np.random.default_rng(1)
        pts = np.column_stack([rng.uniform(0.5, 5, 500), rng.uniform(-3, 3, 500),
                               WATER + rng.normal(0, 0.01, 500)])
        f = sf.fit(pts, WATER)
        self.assertFalse(f.valid)

    def test_a_lone_wall_ahead_is_not_a_slip(self):
        pts = []
        for y in np.arange(-2.0, 2.0, 0.02):
            for z in np.arange(0.05, 0.3, 0.03):
                pts.append((2.0, y, WATER + z))
        f = sf.fit(np.array(pts), WATER)
        self.assertFalse(f.valid)
        self.assertIn("side wall", f.why)


if __name__ == "__main__":
    unittest.main()
