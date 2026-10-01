"""costmap_probe's blob finder and line format. Stdlib only, no ROS:

    python crusader_nav/test/test_costmap_probe.py
"""
import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

from crusader_nav import costmap_probe as cp   # noqa: E402
from crusader_nav import frames_core as fc     # noqa: E402

W, H, RES = 20, 10, 0.1


def grid(*cells):
    data = [0] * (W * H)
    for i, j in cells:
        data[j * W + i] = cp.LETHAL
    return data


class FindBlobsTest(unittest.TestCase):

    def test_empty(self):
        self.assertEqual(cp.find_blobs(grid(), W, H, RES, 0.0, 0.0), [])

    def test_one_blob_centroid_size_and_count(self):
        # a 3 x 2 block at cells i 4..6, j 3..4, origin (-1, 2)
        cells = [(i, j) for i in (4, 5, 6) for j in (3, 4)]
        (cx, cy, size, n), = cp.find_blobs(grid(*cells), W, H, RES, -1.0, 2.0)
        self.assertAlmostEqual(cx, -1.0 + 5.5 * RES)       # centre of cells 4..6 = 5.0 + 0.5
        self.assertAlmostEqual(cy, 2.0 + 4.0 * RES)        # centre of cells 3..4 = 3.5 + 0.5
        self.assertAlmostEqual(size, 3 * RES)              # the longer side
        self.assertEqual(n, 6)

    def test_diagonal_neighbours_are_one_blob_and_gaps_are_two(self):
        self.assertEqual(len(cp.find_blobs(grid((2, 2), (3, 3)), W, H, RES, 0, 0)), 1)
        self.assertEqual(len(cp.find_blobs(grid((2, 2), (4, 2)), W, H, RES, 0, 0)), 2)

    def test_below_threshold_is_not_lethal(self):
        data = grid()
        data[5] = 99                                       # INSCRIBED, not LETHAL
        self.assertEqual(cp.find_blobs(data, W, H, RES, 0, 0), [])

    def test_edges_do_not_wrap(self):
        # last cell of one row and first cell of the next are NOT neighbours
        self.assertEqual(len(cp.find_blobs(grid((W - 1, 2), (0, 4)), W, H, RES, 0, 0)), 2)
        self.assertEqual(len(cp.find_blobs(grid((W - 1, 2), (0, 3)), W, H, RES, 0, 0)), 2)


class BlobLineTest(unittest.TestCase):
    DATUM = (1.2806, 103.8557)

    def test_range_and_bearing(self):
        blob = (30.0, 40.0, 0.4, 9)                        # 3-4-5: 50 m, bearing atan2(30, 40)
        line = cp.blob_line(blob, self.DATUM, (0.0, 0.0))
        self.assertIn("range 50.0 m", line)
        self.assertIn(f"bearing {math.degrees(math.atan2(30, 40)):03.0f} deg", line)
        self.assertIn("size 0.4 m", line)
        lat, lon = fc.to_latlon(30.0, 40.0, *self.DATUM)
        self.assertIn(f"lat {lat:.7f}", line)
        self.assertIn(f"lon {lon:.7f}", line)

    def test_no_fresh_pose_is_blank_not_zero(self):
        line = cp.blob_line((30.0, 40.0, 0.4, 9), self.DATUM, None)
        self.assertIn("range - m", line)
        self.assertIn("bearing - deg", line)


if __name__ == "__main__":
    unittest.main()
