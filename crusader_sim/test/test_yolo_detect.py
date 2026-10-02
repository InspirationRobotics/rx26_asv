"""Offline tests for yolo_detect (the real YOLO + LED classifier on sim frames), without
torch, ultralytics, ROS or a running sim. The two models are stubbed; what is under test is
everything AROUND them: the labelling (oak_detector_core's FlashTracker, fed the sim clock),
the worker thread's hand-off, and the Scoreboard's recall / precision / colour arithmetic.

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest discover -s test -v

Needs numpy and PyYAML. (The models themselves are exercised by
`python -m crusader_sim.yolo_detect frame.png`, in the venv from setup_yolo_venv.sh.)
"""
import os
import sys
import tempfile
import time
import unittest

import numpy as np
import yaml

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
REPO = os.path.dirname(PKG)
sys.path.insert(0, PKG)
sys.path.insert(0, os.path.join(REPO, "crusader_perception"))     # oak_detector_core, pure

from crusader_sim import yolo_detect as Y                           # noqa: E402

with open(os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml"),
          encoding="utf-8") as f:
    OD = yaml.safe_load(f)["oak_detector"]["ros__parameters"]

BOX = (100, 200, 140, 260)
RED = [0.0, 0.0, 0.05, 0.95]        # cls_labels order: blue, green, off, red
OFF = [0.02, 0.02, 0.94, 0.02]
BLUE = [0.95, 0.02, 0.02, 0.01]


def stub_pipeline(frames):
    """A YoloPipeline whose models are replaced: `frames` is a list of probability
    vectors, one per process() call, for a single buoy at BOX."""
    p = Y.YoloPipeline.__new__(Y.YoloPipeline)
    p.core = Y._oak_core()
    p.shapes, p.colours = list(OD["det_labels"]), list(OD["cls_labels"])
    p.off_index = p.colours.index("off")
    p.flash_enable = True
    p.od = OD
    p.reset()
    it = iter(frames)
    p._detect = lambda bgr: ([BOX], ["diamond"], [0.9])
    p._classify = lambda bgr, boxes: {0: np.asarray(next(it), np.float32)}
    return p


def run(probs_at, seconds=12.0, hz=5.0):
    """Drive the stub at hz on the sim clock; probs_at(t) -> the classifier's output."""
    ts = [i / hz for i in range(int(seconds * hz))]
    p = stub_pipeline([probs_at(t) for t in ts])
    img = np.zeros((1200, 1920, 3), np.uint8)
    return [p.process(img, 100.0 + t)[0] for t in ts]


class LabelTest(unittest.TestCase):
    def test_flashing_red_is_called_flash_red(self):
        out = run(lambda t: RED if int(t) % 2 == 0 else OFF)           # 1 s on / 1 s off
        self.assertEqual(out[-1].label, "flash_red_diamond")
        self.assertEqual(out[-1].state, "flashing")
        self.assertEqual(out[-1].colour, "red")

    def test_steady_blue_is_not_called_flashing(self):
        out = run(lambda t: BLUE)
        self.assertEqual(out[-1].label, "blue_diamond")
        self.assertEqual(out[-1].state, "solid")

    def test_unlit_buoy_is_off(self):
        out = run(lambda t: OFF)
        self.assertEqual(out[-1].label, "off_diamond")
        self.assertEqual(out[-1].state, "off")

    def test_a_young_track_publishes_the_colour_not_a_flash_state(self):
        # FlashTracker needs min_span_s of history: before it, the label is the bare colour
        out = run(lambda t: RED)
        self.assertEqual(out[0].state, "unknown")
        self.assertEqual(out[0].label, "diamond")                      # no lit colour voted yet

    def test_the_sim_clock_not_the_wall_clock_drives_the_flash_test(self):
        # the same blink, with all 12 s of frames processed in a few ms of wall time
        t0 = time.monotonic()
        out = run(lambda t: RED if int(t) % 2 == 0 else OFF)
        self.assertLess(time.monotonic() - t0, 2.0)
        self.assertEqual(out[-1].state, "flashing")

    def test_box_is_clamped_to_the_frame(self):
        p = stub_pipeline([RED])
        p._detect = lambda bgr: ([(-5, -3, 2000, 1300)], ["diamond"], [0.9])
        d = p.process(np.zeros((1200, 1920, 3), np.uint8), 0.0)[0]
        self.assertEqual(d.box, (0, 0, 1919, 1199))


class RunnerTest(unittest.TestCase):
    def test_one_frame_at_a_time_and_rate_limited(self):
        class Slow:
            def process(self, bgr, t):
                time.sleep(0.15)
                return [t]
        r = Y.AsyncRunner(Slow(), rate_hz=5.0, log=lambda m: None)
        self.assertTrue(r.wants(10.0))
        r.submit(None, 10.0)
        self.assertFalse(r.wants(10.5))             # busy: the frame is not even converted
        deadline = time.monotonic() + 3.0
        got = []
        while not got and time.monotonic() < deadline:
            got = r.take()
            time.sleep(0.02)
        self.assertEqual(got, [(10.0, [10.0])])
        self.assertFalse(r.wants(10.1))             # idle, but under 1/rate_hz of sim time
        self.assertTrue(r.wants(10.2))
        self.assertTrue(r.wants(3.0))               # sim time went backwards (a reset)

    def test_an_exception_does_not_end_the_worker(self):
        class Flaky:
            n = 0

            def process(self, bgr, t):
                self.n += 1
                if self.n == 1:
                    raise ValueError("bad frame")
                return ["ok"]
        msgs = []
        r = Y.AsyncRunner(Flaky(), rate_hz=100.0, log=msgs.append)
        r.submit(None, 0.0)
        deadline = time.monotonic() + 3.0
        while r.busy and time.monotonic() < deadline:
            time.sleep(0.01)
        r.submit(None, 1.0)
        got = []
        while not got and time.monotonic() < deadline:
            got = r.take()
            time.sleep(0.01)
        self.assertEqual(r.errors, 1)
        self.assertTrue(msgs and "bad frame" in msgs[0])
        self.assertEqual(got, [(1.0, ["ok"])])


def det(box, colour="red", state="flashing", conf=0.9):
    return Y.Det(box, f"{state}_{colour}", conf, "diamond", colour, state, 1)


def truth(name, label, box, rng=10.0, scored=True, occluded=False):
    return Y.Truth(name, label, rng, 40.0, box, scored, occluded)


class ScoreboardTest(unittest.TestCase):
    def test_match_is_greedy_by_iou_and_one_to_one(self):
        hit = Y.match_boxes([(0, 0, 10, 10), (50, 50, 60, 60)],
                            [(1, 1, 11, 11), (0, 0, 10, 10), (200, 200, 210, 210)])
        self.assertEqual(hit[0][0], 1)               # the exact box wins the truth it overlaps
        self.assertNotIn(1, hit)                     # nothing near the second truth

    def test_recall_precision_colour(self):
        s = Y.Scoreboard()
        truths = [truth("a", "red_buoy", (0, 0, 10, 10)),
                  truth("b", "green_buoy", (100, 0, 110, 10)),
                  truth("c", "black_buoy", (200, 0, 210, 10)),
                  truth("hidden", "red_buoy", (300, 0, 310, 10), occluded=True),
                  truth("far", "red_buoy", (400, 0, 404, 4), scored=False)]
        dets = [det((0, 0, 10, 10), "red"),                      # right
                det((100, 0, 110, 10), "red"),                   # found, colour wrong
                det((400, 0, 404, 4), "red"),                    # a buoy too small to be a recall target
                det((600, 0, 610, 10), "red")]                   # no buoy there
        s.update(0.0, truths, dets)
        n = s.run.n
        self.assertEqual((n["truth"], n["tp_scored"]), (3, 2))   # c missed; hidden and far not targets
        self.assertEqual((n["dets"], n["tp"]), (4, 3))           # precision counts the small one
        self.assertEqual((n["colour_ok"], n["colour_bad"], n["colour_none"]), (1, 1, 0))
        self.assertEqual(n["occluded"], 1)
        self.assertIn("recall 0.67 (2/3) precision 0.75 (3/4)", s.report())

    def test_entry_exit_calls(self):
        s = Y.Scoreboard()
        truths = [truth("entry", "flashing_blue_buoy", (0, 0, 10, 10)),
                  truth("exit", "steady_blue_buoy", (100, 0, 110, 10)),
                  truth("e2", "flashing_blue_buoy", (200, 0, 210, 10))]
        dets = [det((0, 0, 10, 10), "blue", "flashing"),
                det((100, 0, 110, 10), "blue", "flashing"),         # EXIT called ENTRY: wrong
                det((200, 0, 210, 10), "blue", "unknown")]          # undecided: not a call
        s.update(0.0, truths, dets)
        n = s.run.n
        self.assertEqual((n["blue_n"], n["blue_ok"], n["blue_undecided"]), (3, 1, 1))

    def test_an_off_buoy_read_as_off_is_a_colour_hit(self):
        s = Y.Scoreboard()
        s.update(0.0, [truth("k", "black_buoy", (0, 0, 10, 10))],
                 [Y.Det((0, 0, 10, 10), "off_diamond", 0.9, "diamond", None, "off", 1)])
        self.assertEqual((s.run.n["colour_ok"], s.run.n["colour_bad"], s.run.n["colour_none"]), (1, 0, 0))

    def test_a_colourless_label_is_unresolved_not_wrong(self):
        s = Y.Scoreboard()
        s.update(0.0, [truth("k", "red_buoy", (0, 0, 10, 10))],
                 [Y.Det((0, 0, 10, 10), "diamond", 0.9, "diamond", None, "unknown", 1)])
        n = s.run.n
        self.assertEqual((n["colour_ok"], n["colour_bad"], n["colour_none"]), (0, 0, 1))
        self.assertIn("colour 0 right / 0 wrong / 1 unresolved", s.report())

    def test_blank_over_a_guess_with_no_truth(self):
        s = Y.Scoreboard()
        s.update(0.0, [], [])
        self.assertIn("recall n/a (0) precision n/a (0)", s.report())

    def test_range_bands_and_window_reset(self):
        s = Y.Scoreboard()
        s.update(0.0, [truth("n", "red_buoy", (0, 0, 10, 10), rng=5.0),
                       truth("m", "red_buoy", (100, 0, 110, 10), rng=15.0)],
                 [det((0, 0, 10, 10))])
        text = s.report()
        self.assertIn("0-10 m 1/1, 10-20 m 0/1, >20 m 0/0", text)
        self.assertEqual(s.window.n["frames"], 0)                    # the window restarted
        self.assertEqual(s.run.n["frames"], 1)                       # the run did not

    def test_csv_rows(self):
        with tempfile.TemporaryDirectory() as d:
            path = os.path.join(d, "y.csv")
            s = Y.Scoreboard(path)
            s.update(1.5, [truth("a", "red_buoy", (0, 0, 10, 10))],
                     [det((0, 0, 10, 10)), det((500, 0, 510, 10))])
            s.close()
            with open(path, encoding="utf-8") as f:
                rows = f.read().splitlines()
        self.assertEqual(rows[0], Y.CSV_HEAD.strip())
        self.assertEqual(len(rows), 3)                               # head, the buoy, one false positive
        self.assertTrue(rows[1].startswith("1.500,a,red_buoy,10.00,40,1,0,1,1.000,0.90,flashing_red"))
        self.assertTrue(rows[2].startswith("1.500,,,,,,,0,0,0.90"))


if __name__ == "__main__":
    unittest.main()
