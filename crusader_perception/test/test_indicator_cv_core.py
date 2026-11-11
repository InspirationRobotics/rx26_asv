"""indicator_cv_core on synthetic frames: a white face, a coloured block at its base.

numpy only, no ROS, no camera:   python crusader_perception/test/test_indicator_cv_core.py
"""
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_perception import indicator_cv_core as ic   # noqa: E402
from crusader_perception import indicator_cv_settings as cvs   # noqa: E402

FACE = (200, 50, 400, 250)              # a 200 px face; the search box is x 230..370, y 180..300
RED, GREEN = (200, 25, 30), (40, 160, 50)           # fire-engine red, grass green (RGB)
WHITE, DECK = (235, 235, 230), (90, 90, 95)


def frame(blocks=(), h=400, w=600):
    """A grey deck, the white face on it, then each (x0, y0, x1, y1, rgb) block painted."""
    img = np.empty((h, w, 3), np.uint8)
    img[:] = DECK
    img[FACE[1]:FACE[3], FACE[0]:FACE[2]] = WHITE
    for x0, y0, x1, y1, rgb in blocks:
        img[y0:y1, x0:x1] = rgb
    return img


IND = (285, 200, 315, 240)              # the indicator: 30 x 40 px at the base, centred


class Hsv(unittest.TestCase):

    def test_primaries_and_grey(self):
        px = np.array([[[255, 0, 0], [0, 255, 0], [0, 0, 255], [128, 128, 128], [255, 0, 40]]],
                      np.uint8)
        h, s, v = ic.hsv(px)
        np.testing.assert_allclose(h[0, :3], [0.0, 120.0, 240.0], atol=0.01)
        np.testing.assert_allclose(s[0, :3], 1.0)
        self.assertEqual(s[0, 3], 0.0)                  # grey: no saturation, hue 0
        self.assertAlmostEqual(float(v[0, 3]), 128 / 255.0, places=5)
        self.assertGreater(float(h[0, 4]), 350.0)       # red leaning to magenta: just under 360

    def test_hue_band_wraps_through_zero(self):
        h = np.array([350.0, 5.0, 30.0, 120.0])
        self.assertEqual(list(ic.hue_in(h, (340.0, 20.0))), [True, True, False, False])
        self.assertEqual(list(ic.hue_in(h, (75.0, 160.0))), [False, False, False, True])


class ReadIndicator(unittest.TestCase):

    def read(self, blocks=(), holes=(), cfg=None, face=FACE, **kw):
        return ic.read_indicator(frame(blocks, **kw), face, holes, cfg)

    def test_red_and_green(self):
        r = self.read([(*IND, RED)])
        self.assertTrue(r["present"])
        self.assertEqual(r["colour"], "red")
        self.assertEqual(r["confidence"], 1.0)
        self.assertAlmostEqual(r["red"], 1200 / 40000.0)     # its share of the face
        np.testing.assert_allclose(r["box"], IND, atol=1.5)   # where it is: its bearing
        g = self.read([(*IND, GREEN)])
        self.assertEqual((g["present"], g["colour"]), (True, "green"))

    def test_nothing_coloured_is_no_indicator(self):
        r = self.read()
        self.assertFalse(r["present"])
        self.assertIsNone(r["box"])
        self.assertEqual(r["roi"], (230, 180, 370, 300))

    def test_both_colours_is_unknown_not_a_vote(self):
        r = self.read([(285, 200, 300, 240, RED), (300, 200, 315, 240, GREEN)])
        self.assertTrue(r["present"])
        self.assertEqual(r["colour"], "unknown")
        self.assertAlmostEqual(r["confidence"], 0.5)
        r = self.read([(285, 200, 312, 240, RED), (312, 200, 315, 240, GREEN)])
        self.assertEqual(r["colour"], "red")           # 9 to 1: red, confidence 0.9
        self.assertAlmostEqual(r["confidence"], 0.9)

    def test_too_small_is_noise(self):
        self.assertFalse(self.read([(300, 220, 308, 228, RED)])["present"])   # 64 px < 80
        self.assertTrue(self.read([(300, 220, 310, 229, RED)])["present"])    # 90 px

    def test_outside_the_search_box_is_ignored(self):
        self.assertFalse(self.read([(205, 200, 225, 240, RED)])["present"])   # left of it
        self.assertFalse(self.read([(285, 100, 315, 140, RED)])["present"])   # up the face

    def test_a_lit_window_is_cut_out(self):
        window = (330, 120, 370, 185)                   # its bottom pokes into the search box
        lit = [(*window, RED)]
        self.assertTrue(self.read(lit)["present"])      # read as the indicator without the cut
        r = self.read(lit, holes=[window])
        self.assertFalse(r["present"])
        r = self.read(lit + [(*IND, GREEN)], holes=[window])
        self.assertEqual(r["colour"], "green")          # the indicator beside a RED window

    def test_dark_and_pale_do_not_count(self):
        self.assertFalse(self.read([(*IND, (40, 5, 6))])["present"])          # v 0.16
        self.assertFalse(self.read([(*IND, (235, 180, 180))])["present"])     # s 0.23, pink
        self.assertFalse(self.read([(*IND, (230, 130, 20))])["present"])      # orange, 29 deg

    def test_face_at_the_frame_edge(self):
        face = (200, 250, 400, 450)                     # its base below the frame
        r = ic.read_indicator(frame([(285, 380, 315, 400, RED)]), face)
        self.assertEqual(r["roi"], (230, 380, 370, 400))
        self.assertEqual((r["present"], r["colour"]), (True, "red"))
        self.assertFalse(ic.read_indicator(frame(), (10, 10, 15, 15))["present"])   # tiny
        self.assertIsNone(ic.read_indicator(frame(), (10, 10, 15, 15))["roi"])

    def test_settings_are_checked(self):
        self.assertEqual(cvs.check({})["roi"], cvs.DEFAULTS["roi"])
        self.assertEqual(cvs.check({"sat_min": 1})["sat_min"], 1.0)
        for bad in ({"roi": [0.8, 0.2, 0.6, 1.2]}, {"roi": [0.2, 0.8]}, {"red_hue": [340.0]},
                    {"green_hue": [75.0, 400.0]}, {"sat_min": 1.5}, {"min_share": 0.0},
                    {"ratio": 0.5}, {"roi": "low"}):
            with self.assertRaises(ValueError, msg=str(bad)):
                cvs.check(bad)
        r = self.read([(*IND, RED)], cfg={"red_hue": [100.0, 110.0]})
        self.assertFalse(r["present"])                  # the settings are the ones used


if __name__ == "__main__":
    unittest.main()
