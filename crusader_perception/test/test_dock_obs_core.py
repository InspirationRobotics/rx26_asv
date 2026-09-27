"""dock_obs_core on synthetic depth: a face plane, window holes, the aim points.

numpy only, no ROS, no camera:   python crusader_perception/test/test_dock_obs_core.py

The depth image is RENDERED from a known face (a 1 m panel at a known range
and angle) through the same pinhole the code inverts, so "the window is at
x, y, z" is checked against geometry, not against the code's own output.
"""
import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_perception import dock_obs_core as do   # noqa: E402

W, H = 1920, 1200
INTR = (1173.0, 1173.0, 960.0, 600.0)          # the OAK-D LR at 1920x1200 (CV report)
DW, DH = 640, 400                              # depth at the stereo size


def project(p):
    """REP-103 camera point -> RGB pixel."""
    fx, fy, cx, cy = INTR
    return cx - fx * p[1] / p[0], cy - fy * p[2] / p[0]


def face(range_m=3.0, yaw_deg=0.0, noise=0.0, seed=0):
    """Depth [mm] of a vertical face `range_m` ahead, turned yaw_deg (+ = its
    normal swung toward +y), with its two windows as HOLES (zero depth), and
    the true normal/offset and window centres."""
    a = math.radians(yaw_deg)
    n = np.array([-math.cos(a), -math.sin(a), 0.0])       # toward the camera
    c0 = np.array([range_m, 0.0, 0.2])                     # the face centre
    d = -float(n @ c0)
    t_right = np.array([-math.sin(a), math.cos(a), 0.0]) * -1.0   # along the face, to the RIGHT
    wins = {0: c0 - 0.22 * t_right + np.array([0, 0, 0.25]),      # UL (left)
            1: c0 + 0.23 * t_right + np.array([0, 0, 0.01])}      # LR (right)
    half = {0: (0.105, 0.145), 1: (0.115, 0.155)}
    rng = np.random.default_rng(seed)
    depth = np.zeros((DH, DW), np.uint16)
    fx, fy, cx, cy = INTR
    sx, sy = DW / W, DH / H
    vv, uu = np.mgrid[0:DH, 0:DW]
    ur, vr = (uu + 0.5) / sx - 0.5, (vv + 0.5) / sy - 0.5
    ray = np.stack([np.ones_like(ur), -(ur - cx) / fx, -(vr - cy) / fy], -1)
    t = -d / (ray @ n)
    p = ray * t[..., None]
    q = p - c0
    along, up = q @ t_right, q[..., 2]
    on_face = (np.abs(along) <= 0.5) & (np.abs(up) <= 0.5) & (t > 0)
    for k, wc in wins.items():
        qa, qz = (p - wc) @ t_right, (p - wc)[..., 2]
        on_face &= ~((np.abs(qa) <= half[k][0]) & (np.abs(qz) <= half[k][1]))
    z = p[..., 0] * (1.0 + rng.normal(0, noise, p[..., 0].shape))
    depth[on_face] = np.clip(z[on_face] * 1000.0, 0, 65535).astype(np.uint16)
    boxes = {}
    for k, wc in wins.items():
        corners = [wc + sa * half[k][0] * t_right + np.array([0, 0, sz * half[k][1]])
                   for sa in (-1, 1) for sz in (-1, 1)]
        px = [project(cn) for cn in corners]
        boxes[k] = (min(u for u, _ in px), min(v for _, v in px),
                    max(u for u, _ in px), max(v for _, v in px))
    fc = [c0 + sa * 0.5 * t_right + np.array([0, 0, sz * 0.5]) for sa in (-1, 1) for sz in (-1, 1)]
    fpx = [project(cn) for cn in fc]
    fbox = (min(u for u, _ in fpx), min(v for _, v in fpx), max(u for u, _ in fpx), max(v for _, v in fpx))
    return depth, n, d, wins, boxes, fbox


def bay_dict(boxes, fbox, states=("red", "off"), ind_colour="green"):
    return dict(face=fbox, face_conf=0.9, truncated=False,
                indicator=dict(box=(fbox[0] + 10, fbox[3] - 60, fbox[0] + 60, fbox[3] - 10),
                               conf=0.8, colour=ind_colour, colour_conf=0.85),
                windows=[dict(index=0, slot="UL", box=boxes[0], conf=0.9, state=states[0], state_conf=0.8),
                         dict(index=1, slot="LR", box=boxes[1], conf=0.9, state=states[1], state_conf=0.7)],
                lit=0 if states[0] in ("red", "green", "blue") else None,
                range_m=3.0, bearing_deg=0.0)


class TestPlane(unittest.TestCase):

    def test_square_on(self):
        depth, n, d, wins, boxes, fbox = face(3.0)
        pts = do.face_points(depth, fbox, list(boxes.values()), (W, H), INTR)
        nrm, off, rms, cnt = do.fit_plane(pts)
        self.assertGreater(off, 0)                                  # faces the camera
        self.assertAlmostEqual(float(nrm @ n), 1.0, places=4)
        self.assertAlmostEqual(off, d, delta=0.005)

    def test_turned_and_noisy(self):
        depth, n, d, wins, boxes, fbox = face(3.2, yaw_deg=8.0, noise=0.01, seed=2)
        pts = do.face_points(depth, fbox, list(boxes.values()), (W, H), INTR)
        nrm, off, rms, cnt = do.fit_plane(pts)
        self.assertLess(math.degrees(math.acos(min(1.0, float(nrm @ n)))), 1.0)
        self.assertAlmostEqual(off, d, delta=0.03)

    def test_too_little_depth(self):
        pts = np.zeros((20, 3))
        self.assertIsNone(do.fit_plane(pts))


class TestAimPoints(unittest.TestCase):

    def test_windows_land_on_their_centres(self):
        for yaw in (0.0, 6.0, -6.0):
            depth, n, d, wins, boxes, fbox = face(3.0, yaw_deg=yaw, noise=0.005, seed=4)
            obs = do.observation([bay_dict(boxes, fbox)], depth, (W, H), INTR)
            b = obs["bays"][0]
            self.assertTrue(b["has_plane"])
            for w in b["windows"]:
                true = wins[w["index"]]
                got = np.array([w["x"], w["y"], w["z"]])
                self.assertTrue(w["has_position"])
                # the box centre of a turned window is not quite its centre:
                # a centimetre or two of perspective, well inside a 21 cm window
                self.assertLess(np.linalg.norm(got - true), 0.03, (yaw, w["index"], got, true))

    def test_no_depth_no_positions(self):
        depth, n, d, wins, boxes, fbox = face(3.0)
        obs = do.observation([bay_dict(boxes, fbox)], None, (W, H), INTR)
        b = obs["bays"][0]
        self.assertFalse(b["has_plane"])
        self.assertFalse(any(w["has_position"] for w in b["windows"]))
        self.assertTrue(math.isnan(b["plane_rms_m"]))


class TestMessageShape(unittest.TestCase):

    def test_fields_and_numbering(self):
        depth, n, d, wins, boxes, fbox = face(3.0)
        obs = do.observation([bay_dict(boxes, fbox, states=("red", "off"))], depth, (W, H), INTR,
                             fps=14.2)
        b = obs["bays"][0]
        self.assertEqual(b["indicator_colour"], 3)                # GREEN = 3 (CV numbering)
        self.assertEqual([w["state"] for w in b["windows"]], [2, 1])   # RED, OFF
        self.assertEqual(b["lit_window_index"], 0)
        self.assertEqual(b["lit_state"], 2)
        self.assertEqual(len(b["bbox"]), 4)
        self.assertEqual(obs["target_window_index"], -1)          # no timing layer given
        self.assertAlmostEqual(obs["observed_fps"], 14.2)

    def test_windows_without_an_index_are_dropped(self):
        depth, n, d, wins, boxes, fbox = face(3.0)
        bay = bay_dict(boxes, fbox)
        bay["windows"][1]["index"] = None
        obs = do.observation([bay], depth, (W, H), INTR)
        self.assertEqual([w["index"] for w in obs["bays"][0]["windows"]], [0])

    def test_timing_layer(self):
        class Seq:
            target = 0
            patterns = {0: ("code", ("red", "blue"))}
        obs = do.observation([], None, (W, H), INTR, seq=Seq(), seq_events=[{"type": "hit"}])
        self.assertEqual((obs["target_pattern"], obs["target_colours"], obs["target_window_index"],
                          obs["last_event"]), ("code", ["red", "blue"], 0, "hit"))

    def test_tracked_bay(self):
        g = dict(indicator=dict(colour="green"))
        r = dict(indicator=dict(colour="red"))
        self.assertEqual(do.tracked_bay([r, g, r]), 1)
        self.assertEqual(do.tracked_bay([r]), 0)                  # the only one in view
        self.assertIsNone(do.tracked_bay([r, r]))
        self.assertIsNone(do.tracked_bay([]))


if __name__ == "__main__":
    unittest.main(verbosity=2)
