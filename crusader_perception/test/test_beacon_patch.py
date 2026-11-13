"""beacon_patch: the colour classifier's crop, the band above a BODY-only box.

numpy + cv2, no ROS, no camera:   python crusader_perception/test/test_beacon_patch.py

The training notebook cuts every classifier crop with vision/tools/beacon_crop.py's
classifier_patch; the boat must cut the same picture. Where that file is present (the team
laptop's checkout, not the Jetson) the copy is checked against it pixel for pixel.
"""
import importlib.util
import os
import sys
import unittest

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from crusader_perception import oak_detector_core as core   # noqa: E402

try:
    import cv2                                                # noqa: F401
    HAVE_CV2 = True
except ImportError:
    HAVE_CV2 = False

# Boat/rx26_asv*/crusader_perception/test -> Boat/vision/tools/beacon_crop.py
BEACON_CROP = os.path.join(HERE, "..", "..", "..", "vision", "tools", "beacon_crop.py")

RED, WHITE, GREY = (40, 40, 230), (245, 245, 245), (90, 90, 90)      # BGR


def scene(body, beacon_colour=RED, size=(1200, 1920)):
    """A grey frame with a white body box and a beacon of the measured size sitting on it."""
    img = np.full(size + (3,), GREY, np.uint8)
    x1, y1, x2, y2 = body
    img[y1:y2, x1:x2] = WHITE
    bh = y2 - y1
    h, w = int(round(core.BEACON_H_FRAC * bh)), int(round(core.BEACON_W_FRAC * bh))
    cx = (x1 + x2) // 2
    img[y1 - h:y1, cx - w // 2:cx + w // 2] = beacon_colour
    return img


def share(patch, colour, tol=30):
    """Fraction of the patch's pixels within `tol` of `colour` on every channel."""
    return float(np.mean(np.all(np.abs(patch.astype(int) - colour) <= tol, axis=-1)))


class Geometry(unittest.TestCase):
    def test_measured_proportions(self):
        # body 100 px tall: beacon 32 tall, 31 wide; crop 30% above, 15% below, 40% each side
        cx, cy, w, h = core.beacon_geom((100, 200, 200, 300))
        self.assertAlmostEqual(cx, 150.0)
        self.assertAlmostEqual(cy, (200 - 32 * 1.3 + 200 + 32 * 0.15) / 2)
        self.assertAlmostEqual(w, 31 * 1.8)
        self.assertAlmostEqual(h, 32 * 1.45)

    def test_scales_off_height_not_width(self):
        # a yawed pyramid is wider, not taller: the crop must not change
        face_on = core.beacon_geom((100, 200, 160, 300))
        corner_on = core.beacon_geom((80, 200, 180, 300))
        self.assertAlmostEqual(face_on[2], corner_on[2])
        self.assertAlmostEqual(face_on[3], corner_on[3])

    def test_degenerate_box(self):
        self.assertIsNone(core.beacon_geom((10, 10, 10, 50)))
        self.assertIsNone(core.beacon_geom((10, 50, 40, 50)))


@unittest.skipUnless(HAVE_CV2, "needs cv2")
class Patch(unittest.TestCase):
    def test_frames_the_beacon_not_the_body(self):
        body = (800, 700, 1060, 830)
        patch = core.beacon_patch(scene(body), body, size=64)
        self.assertEqual(patch.shape, (64, 64, 3))
        # the beacon is the largest thing in the crop; the body is a thin strip at the bottom
        self.assertGreater(share(patch, RED), 0.25)
        self.assertLess(share(patch, WHITE), 0.15)

    def test_box_is_in_frame_pixels_and_contains_the_beacon(self):
        body = (800, 700, 1060, 830)
        _, (x0, y0, x1, y1) = core.beacon_patch(scene(body), body, return_box=True)
        bh = body[3] - body[1]
        self.assertLess(y0, body[1] - core.BEACON_H_FRAC * bh)      # above the beacon's top
        self.assertGreater(y1, body[1])                               # a little of the body
        self.assertLess(x0, 930 - core.BEACON_W_FRAC * bh / 2)
        self.assertGreater(x1, 930 + core.BEACON_W_FRAC * bh / 2)

    def test_too_small_to_classify(self):
        img = np.zeros((100, 100, 3), np.uint8)
        # body 16 px tall: the crop is ~7 px tall, under BEACON_MIN_PX
        self.assertIsNone(core.beacon_patch(img, (40, 60, 60, 76)))
        self.assertEqual(core.beacon_patch(img, (40, 60, 60, 76), return_box=True),
                         (None, None))
        self.assertIsNotNone(core.beacon_patch(img, (40, 50, 60, 70)))

    def test_top_of_frame_replicates_the_edge(self):
        body = (100, 5, 200, 105)                 # the crop starts ~37 px above the frame
        img = scene((100, 40, 200, 140))[35:]     # same picture, shifted up off the frame
        patch = core.beacon_patch(img, body, size=64)
        self.assertEqual(patch.shape, (64, 64, 3))
        self.assertGreater(patch[0].min(), 0)     # no black bar for "off" to read

    def test_min_beacon_px_gate(self):
        img = np.zeros((300, 300, 3), np.uint8)
        # beacon height = 0.32 x body height: 24 px body -> 7.68 px, 25 px -> 8.0 px
        self.assertIsNone(core.beacon_patch(img, (100, 100, 140, 124), min_beacon_px=8.0))
        self.assertIsNotNone(core.beacon_patch(img, (100, 100, 140, 125), min_beacon_px=8.0))
        self.assertEqual(core.beacon_patch(img, (100, 100, 140, 124), min_beacon_px=8.0,
                                           return_box=True), (None, None))
        # the default is the training contract: no size gate beyond the 8 px crop floor
        self.assertIsNotNone(core.beacon_patch(img, (100, 100, 140, 124)))
        self.assertAlmostEqual(core.beacon_height((0, 10, 5, 35)), 8.0)

    def test_no_crop_top_frac_left_behind(self):
        self.assertNotIn("crop_top_frac", core.PARAMS)
        self.assertFalse(hasattr(core, "led_patch"))


def state(word, colour):
    return core.FlashState(word, 0.5, 0.9, 30, 6.0, colour, 1.0)


class MustFlash(unittest.TestCase):
    """rg_must_flash: Task 1 shows red and green only flashing, so a red or green light that
    is solid, or not yet decided, publishes with no colour."""
    MF = core.MUST_FLASH

    def test_the_defaults(self):
        self.assertEqual(self.MF, ("red", "green"))
        self.assertTrue(core.PARAMS["rg_must_flash"])
        self.assertEqual(core.PARAMS["cls_min_beacon_px"], 8.0)

    def test_red_and_green_need_a_flash(self):
        for c in ("red", "green"):
            self.assertEqual(state("flashing", c).label("diamond", self.MF), f"flash_{c}_diamond")
            self.assertEqual(state("solid", c).label("diamond", self.MF), "diamond")
            self.assertEqual(state("unknown", c).label("diamond", self.MF), "diamond")
            self.assertTrue(state("solid", c).withheld(self.MF))
            self.assertFalse(state("flashing", c).withheld(self.MF))

    def test_blue_and_off_are_untouched(self):
        self.assertEqual(state("solid", "blue").label("diamond", self.MF), "blue_diamond")
        self.assertEqual(state("flashing", "blue").label("diamond", self.MF),
                         "flash_blue_diamond")
        self.assertEqual(state("off", "red").label("diamond", self.MF), "off_diamond")
        self.assertFalse(state("off", "red").withheld(self.MF))

    def test_guard_off_is_the_old_label(self):
        self.assertEqual(state("solid", "red").label("diamond"), "red_diamond")
        # undecided is never a colour: an undecided flashing-blue ENTRY must not read as the EXIT
        self.assertEqual(state("unknown", "green").label("diamond", ()), "diamond")
        self.assertEqual(state("unknown", "blue").label("diamond", self.MF), "diamond")


@unittest.skipUnless(HAVE_CV2 and os.path.exists(BEACON_CROP),
                     "needs cv2 and Boat/vision/tools/beacon_crop.py (team laptop only)")
class SameAsTraining(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        spec = importlib.util.spec_from_file_location("beacon_crop", BEACON_CROP)
        cls.bc = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.bc)

    def test_constants(self):
        bc = self.bc
        self.assertEqual((core.BEACON_H_FRAC, core.BEACON_W_FRAC, core.BEACON_MARGIN_ABOVE,
                          core.BEACON_MARGIN_BELOW, core.BEACON_MARGIN_SIDE),
                         (bc.BEACON_H_FRAC, bc.BEACON_W_FRAC, bc.MARGIN_ABOVE,
                          bc.MARGIN_BELOW, bc.MARGIN_SIDE))

    def test_identical_pixels(self):
        rng = np.random.default_rng(7)
        img = rng.integers(0, 256, (1200, 1920, 3), dtype=np.uint8)
        for body in [(800, 700, 1060, 830), (5, 3, 60, 40), (1850, 1100, 1919, 1199),
                     (1078.4, 681.2, 1158.9, 726.6)]:
            mine = core.beacon_patch(img, body, size=64)
            theirs = self.bc.classifier_patch(img, body, size=64)
            np.testing.assert_array_equal(mine, theirs, err_msg=str(body))


if __name__ == "__main__":
    unittest.main()
