"""squirt_core: the bracket, the state machine, the log.   python test_squirt_core.py"""
import json
import os
import shutil
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from squirt_core import (DEFAULTS, FA, LAT, App, AxisEstimator,   # noqa: E402
                         load_config)


class TestAxisEstimator(unittest.TestCase):

    def test_no_data(self):
        e = AxisEstimator().estimate()
        self.assertIsNone(e["centre"])
        self.assertIsNone(AxisEstimator().next_value(0.1))

    def test_one_sided_steps_out(self):
        a = AxisEstimator([(0.90, +1)])          # too far forward at 0.90
        self.assertAlmostEqual(a.next_value(0.1), 1.00)
        b = AxisEstimator([(0.90, -1)])          # too far back at 0.90
        self.assertAlmostEqual(b.next_value(0.1), 0.80)

    def test_bracket_midpoint(self):
        a = AxisEstimator([(0.80, +1), (1.00, -1), (0.85, +1), (0.95, -1)])
        e = a.estimate()
        self.assertTrue(e["consistent"])
        self.assertAlmostEqual(e["lo"], 0.85)
        self.assertAlmostEqual(e["hi"], 0.95)
        self.assertAlmostEqual(e["centre"], 0.90)

    def test_hits_aim_for_the_middle_of_the_band(self):
        # misses at 0.80 (too close) and 1.00 (too far) around hits 0.86-0.88:
        # the band's edges are halfway out, 0.83 and 0.94, so aim at 0.885
        a = AxisEstimator([(0.80, +1), (1.00, -1), (0.86, 0), (0.88, 0), (0.87, 0)])
        e = a.estimate(min_hits_for_band=3)
        self.assertAlmostEqual(e["edge_lo"], 0.83)
        self.assertAlmostEqual(e["edge_hi"], 0.94)
        self.assertAlmostEqual(e["centre"], 0.885)
        self.assertEqual(e["band"], (0.86, 0.88))

    def test_after_a_hit_it_probes_both_sides(self):
        a = AxisEstimator([(0.80, +1), (0.90, 0)])     # came from below, then hit
        self.assertAlmostEqual(a.next_value(0.1), 0.95)  # the far side first
        a.add(0.95, -1)                                 # too far there
        self.assertAlmostEqual(a.next_value(0.1), (0.85 + 0.925) / 2)
        b = AxisEstimator([(0.90, 0), (0.95, 0), (1.00, -1)])
        self.assertAlmostEqual(b.next_value(0.1), 0.85)  # far edge known: probe near

    def test_contradicting_misses_inside_the_hits_are_ignored_for_edges(self):
        a = AxisEstimator([(0.86, 0), (0.90, 0), (0.88, +1), (0.80, +1), (1.00, -1)])
        e = a.estimate()
        self.assertAlmostEqual(e["edge_lo"], 0.83)
        self.assertAlmostEqual(e["edge_hi"], 0.95)

    def test_contradictions_pick_the_least_contradicted(self):
        # one wrong "too far back" at 0.82 inside a clear answer near 0.90
        obs = [(0.80, +1), (0.84, +1), (0.86, +1), (0.82, -1),
               (0.94, -1), (0.96, -1), (0.98, -1)]
        e = AxisEstimator(obs).estimate()
        self.assertFalse(e["consistent"])
        self.assertGreater(e["centre"], 0.86)
        self.assertLess(e["centre"], 0.94)

    def test_verdict_signs(self):
        # "too far forward" = the boat is too close = the range must GROW
        self.assertEqual(FA["fwd"], +1)
        self.assertEqual(FA["back"], -1)
        # offset is + LEFT, so "too far left" = the offset must SHRINK
        self.assertEqual(LAT["left"], -1)
        self.assertEqual(LAT["right"], +1)


class TestConfig(unittest.TestCase):

    def test_defaults(self):
        self.assertEqual(load_config(), DEFAULTS)

    def test_unknown_and_missing_keys_refused(self):
        bad = dict(DEFAULTS, nozle_x_m=0.4)
        with self.assertRaises(KeyError):
            load_config(bad)
        bad = dict(DEFAULTS)
        del bad["deck_height_m"]
        with self.assertRaises(KeyError):
            load_config(bad)

    def test_target_lists_must_line_up(self):
        bad = dict(DEFAULTS, target_edge_mm=[895.0])
        with self.assertRaises(KeyError):
            load_config(bad)


class StubAdapter:
    """Scripted inputs: everything steady, a fixed range, firing allowed."""

    def __init__(self, can=True):
        self.can = can
        self.fired = []

    def info(self):
        return dict(kind="stub")

    def can_fire(self):
        return (True, "ok") if self.can else (False, "no pump path")

    def fire(self, burst_s, seq):
        self.fired.append((burst_s, seq))
        return True, "sent"

    def snapshot(self, path):
        return False

    def poll(self, now):
        pass

    def fake_state(self, target):
        return {}

    def fake_action(self, path, payload, app):
        return dict(ok=False, message="stub")


class Rig:

    def __init__(self, can=True, log_root=""):
        self.t = 0.0
        self.ad = StubAdapter(can)
        self.app = App(load_config(), self.ad, clock=lambda: self.t,
                       wall=lambda: 1.8e9 + self.t, log_root=log_root)

    def run(self, seconds, rng=1.00, lat=0.05, roll=0.0, sticks=None):
        end = self.t + seconds
        while self.t < end:
            self.t = round(self.t + 1 / 30.0, 6)
            self.app.on_att(self.t, roll, 0.0, 0.0, 0.0)
            self.app.on_sticks(self.t, sticks or [1500] * 4)
            self.app.on_range(self.t, True, rng, 0.5, lat)
            self.app.tick(self.t)


class TestStateMachine(unittest.TestCase):

    def test_arm_waits_for_steady_then_fires(self):
        r = Rig()
        r.run(0.3)                                   # not long enough to be steady
        self.assertTrue(r.app.action("/arm", {"mode": "steady"})["ok"])
        self.assertEqual(r.app.state, "armed")
        self.assertEqual(r.ad.fired, [])
        r.run(3.0)
        self.assertEqual(len(r.ad.fired), 1)
        self.assertEqual(r.app.state, "verdict")
        p = r.app.pending
        self.assertAlmostEqual(p["range_m"], 1.00)
        self.assertEqual(p["range_src"], "lidar")
        self.assertTrue(p["steady"])

    def test_fire_now_does_not_wait(self):
        r = Rig()
        r.run(0.2)
        r.app.action("/arm", {"mode": "now"})
        self.assertEqual(len(r.ad.fired), 1)
        self.assertFalse(r.app.pending["steady"])      # and says so in the log

    def test_never_steady_times_out(self):
        r = Rig()
        r.run(1.0)
        r.app.action("/arm", {"mode": "steady"})
        # sticks never still
        for i in range(int(r.app.cfg["arm_timeout_s"] * 30) + 30):
            r.run(1 / 30.0, sticks=[1500 + (i % 2) * 100, 1500, 1500, 1500])
        self.assertEqual(r.app.state, "idle")
        self.assertIn("never got steady", r.app.message)
        self.assertEqual(r.ad.fired, [])

    def test_refused_when_the_path_is_down(self):
        r = Rig(can=False)
        r.run(2.0)
        res = r.app.action("/arm", {"mode": "now"})
        self.assertFalse(res["ok"])
        self.assertIn("no pump path", res["message"])

    def test_refused_while_a_mission_runs(self):
        r = Rig()
        r.run(2.0)
        r.app.on_mission(True)
        self.assertFalse(r.app.action("/arm", {"mode": "now"})["ok"])

    def test_min_gap(self):
        r = Rig()
        r.run(2.0)
        r.app.action("/arm", {"mode": "now"})
        r.run(1.5)                                   # < min_gap_s (2.0) after firing
        self.assertTrue(r.app.action("/verdict", {"fa": "fwd", "lat": "ok"})["ok"])
        res = r.app.action("/arm", {"mode": "now"})
        self.assertFalse(res["ok"])
        self.assertIn("too soon", res["message"])
        r.run(0.6)
        self.assertTrue(r.app.action("/arm", {"mode": "now"})["ok"])

    def test_verdict_updates_advice(self):
        r = Rig()
        r.run(2.0)
        r.app.action("/arm", {"mode": "now"})
        r.run(2.0)
        r.app.action("/verdict", {"fa": "fwd", "lat": "ok"})   # too close at 1.00
        adv = r.app.advice(r.t)
        self.assertAlmostEqual(adv["goal_range_m"], 1.10)
        self.assertEqual(adv["fa_dir"], "back")
        self.assertIn("10 cm BACK", adv["fa_text"])

    def test_bad_verdicts_refused(self):
        r = Rig()
        self.assertFalse(r.app.action("/verdict", {"fa": "ok", "lat": "ok"})["ok"])
        r.run(2.0)
        r.app.action("/arm", {"mode": "now"})
        self.assertFalse(r.app.action("/verdict", {"fa": "up", "lat": "ok"})["ok"])

    def test_undo_puts_the_shot_back(self):
        r = Rig()
        r.run(2.0)
        r.app.action("/arm", {"mode": "now"})
        r.run(2.0)
        r.app.action("/verdict", {"fa": "fwd", "lat": "left"})
        self.assertEqual(len(r.app.shots), 1)
        self.assertTrue(r.app.action("/undo", {})["ok"])
        self.assertEqual(r.app.state, "verdict")
        self.assertEqual(r.app.shots, [])
        r.app.action("/verdict", {"fa": "back", "lat": "ok"})
        self.assertEqual(r.app.shots[-1]["fa"], "back")

    def test_discard_is_not_evidence(self):
        r = Rig()
        r.run(2.0)
        r.app.action("/arm", {"mode": "now"})
        r.run(2.0)
        r.app.action("/discard", {"why": "didn't see"})
        (_, fa), _ = r.app.estimates(r.app.target)
        self.assertEqual(fa["n"], 0)

    def test_pilot_edge_while_armed_becomes_a_pilot_shot(self):
        r = Rig()
        r.run(0.2)
        r.app.action("/arm", {"mode": "steady"})
        r.app.on_pump_edge(r.t, True)
        self.assertEqual(r.app.pending["source"], "pilot")
        r.app.on_pump_edge(r.t + 0.3, False)
        self.assertAlmostEqual(r.app.pending["burst_s"], 0.3, places=3)
        self.assertEqual(r.ad.fired, [])

    def test_tape_range_when_no_lidar(self):
        r = Rig()
        r.app.action("/tape", {"range_m": 0.95})
        for _ in range(60):
            r.t += 1 / 30.0
            r.app.on_att(r.t, 0, 0, 0, 0)
            r.app.on_sticks(r.t, [1500] * 4)
            r.app.tick(r.t)
        r.app.action("/arm", {"mode": "now"})
        self.assertEqual(r.app.pending["range_src"], "tape")
        self.assertAlmostEqual(r.app.pending["range_m"], 0.95)

    def test_snapshot_is_json(self):
        r = Rig()
        r.run(1.0)
        json.dumps(r.app.snapshot())


class TestLog(unittest.TestCase):

    def test_jsonl_with_retraction(self):
        root = tempfile.mkdtemp()
        try:
            r = Rig(log_root=root)
            r.run(2.0)
            r.app.action("/arm", {"mode": "now"})
            r.run(2.0)
            r.app.action("/verdict", {"fa": "fwd", "lat": "ok"})
            r.app.action("/undo", {})
            r.app.action("/verdict", {"fa": "ok", "lat": "ok"})
            path = os.path.join(root, r.app.log.name, "shots.jsonl")
            lines = [json.loads(x) for x in open(path)]
            self.assertEqual([x.get("fa", "retract") for x in lines],
                             ["fwd", "retract", "ok"])
            self.assertEqual(lines[1]["retract"], 1)
        finally:
            shutil.rmtree(root)


if __name__ == "__main__":
    unittest.main(verbosity=2)
