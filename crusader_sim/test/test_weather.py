"""Offline tests for crusader_sim.weather: the varying wind and current (no Gazebo).

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest test.test_weather -v
"""
import io
import json
import os
import random
import sys
import tempfile
import unittest
from contextlib import redirect_stdout
from unittest import mock

PKG = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, PKG)

from crusader_sim import weather as W                                          # noqa: E402


class OUTest(unittest.TestCase):
    def test_unit_variance_and_clipped(self):
        o = W.OU(4.0, random.Random(7))
        xs = [o.step(0.2) for _ in range(20000)]
        mean = sum(xs) / len(xs)
        var = sum((x - mean) ** 2 for x in xs) / len(xs)
        self.assertLess(abs(mean), 0.15)
        self.assertGreater(var, 0.6)          # unit variance, less what the clip takes off
        self.assertLess(var, 1.2)
        self.assertLessEqual(max(abs(x) for x in xs), 2.0)

    def test_it_moves_slowly(self):
        o = W.OU(20.0, random.Random(3))
        xs = [o.step(0.2) for _ in range(500)]
        steps = [abs(b - a) for a, b in zip(xs, xs[1:])]
        self.assertLess(max(steps), 0.5)       # no jumps: a current does not flick round


class ActualTest(unittest.TestCase):
    def test_constant_settings_are_constant(self):
        d = json.loads(json.dumps(W.CALM))
        d["wind"].update(mps=6.0, toward_deg=90.0, gust=0.0)
        d["current"].update(mps=0.15, toward_deg=0.0)
        self.assertEqual(W.actual(d, 1.3, -0.7, 1.9, 0.4), (6.0, 90.0, 0.15, 0.0))

    def test_swing_and_vary(self):
        d = json.loads(json.dumps(W.PRESETS["rough"]))
        ws, wd, cs, cd = W.actual(d, 0.5, 1.0, -1.0, -0.5)
        self.assertAlmostEqual(ws, 6.0 * (1 + 0.4 * 0.5))
        self.assertAlmostEqual(wd, 120.0)                 # 90 + 30 x 1
        self.assertAlmostEqual(cs, 0.15 * (1 - 0.5))      # -1 sigma of 50 %
        self.assertAlmostEqual(cd, 340.0)                 # 0 - 40 x 0.5, wrapped
        _, _, cs2, _ = W.actual(d, 0, 0, -2.0, 0)
        self.assertGreaterEqual(cs2, 0.0)                 # never a negative speed


class CommandsTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        for name, val in (("SETTINGS", "w.json"), ("NOW_FILE", "now.txt")):
            p = mock.patch.object(W, name, os.path.join(self.tmp.name, val))
            p.start()
            self.addCleanup(p.stop)

    def run_cmd(self, *argv):
        out = io.StringIO()
        with redirect_stdout(out):
            self.assertEqual(W.main(list(argv)), 0)
        return out.getvalue()

    def test_wind_with_swing(self):
        self.assertIn("direction +-25 deg", self.run_cmd("wind", "6", "90", "--swing", "25"))
        d = W.load()
        self.assertEqual((d["wind"]["mps"], d["wind"]["toward_deg"], d["wind"]["swing"]),
                         (6.0, 90.0, 25.0))

    def test_current_with_vary_and_swing(self):
        out = self.run_cmd("current", "0.15", "0", "--vary", "0.5", "--swing", "40")
        self.assertIn("speed +-50%", out)
        self.assertIn("direction +-40 deg", out)
        self.assertEqual(W.load()["current"]["vary"], 0.5)

    def test_presets_and_calm(self):
        self.run_cmd("rough")
        self.assertEqual(W.load(), W.PRESETS["rough"])
        self.run_cmd("light")
        self.assertEqual(W.load()["current"]["mps"], 0.05)
        self.assertIn("calm", self.run_cmd("calm"))
        self.assertEqual(W.load(), W.CALM)

    def test_an_old_settings_file_still_loads(self):
        with open(W.SETTINGS, "w", encoding="utf-8") as f:     # no swing / vary keys
            json.dump({"wind": {"mps": 4.0, "toward_deg": 10.0, "gust": 0.3},
                       "current": {"mps": 0.1, "toward_deg": 0.0}}, f)
        d = W.load()
        self.assertEqual(d["wind"]["swing"], 0.0)
        self.assertEqual(d["current"]["vary"], 0.0)
        self.assertEqual(W.actual(d, 0, 2, 2, 2)[1:], (10.0, 0.1, 0.0))

    def test_show_prints_what_it_is_now(self):
        with open(W.NOW_FILE, "w", encoding="utf-8") as f:
            f.write("now: wind 5.1 m/s toward 97 deg (~8 N), current 0.12 m/s toward 352 deg\n")
        self.assertIn("now: wind 5.1", self.run_cmd("show"))


if __name__ == "__main__":
    unittest.main()
