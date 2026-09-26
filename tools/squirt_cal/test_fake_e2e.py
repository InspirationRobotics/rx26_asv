"""The whole session against the fake boat, headless, in simulated time.

The fake pilot drives to wherever the tool says, the fake operator judges each
shot from where the stream really hit, and the fake nozzle is not the one in the
config. What must come out: a range per target that HITS on a level boat, in a
handful of shots, with rocking and with an operator who is sometimes wrong.

    python tools/squirt_cal/test_fake_e2e.py
"""
import os
import statistics
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from fake_boat import FakeAdapter, FakeBoat          # noqa: E402
from squirt_core import App, load_config             # noqa: E402

DT = 0.05


class Session:
    """App + FakeBoat on a virtual clock."""

    def __init__(self, seed, sea=1.0, p_wrong=0.0):
        self.t = 0.0
        self.cfg = load_config()
        self.boat = FakeBoat(self.cfg, seed=seed)
        self.boat.sea, self.boat.p_wrong = sea, p_wrong
        self.boat.follow_advice = True
        self.adapter = FakeAdapter(self.boat)
        self.app = App(self.cfg, self.adapter, clock=lambda: self.t,
                       wall=lambda: 1.8e9 + self.t, log_root="")
        self.adapter.attach(self.app)
        self.run(2.0)

    def run(self, seconds, until=None):
        end = self.t + seconds
        while self.t < end:
            self.t += DT
            self.app.tick(self.t)
            if until and until():
                return True
        return False

    def settle(self):
        """Let the pilot reach the advised spot (the tool's own advice)."""
        b = self.boat
        self.run(20.0, until=lambda: self.t > b.moving_until + 1.0
                 and abs(b.range - b.goal[0]) < 0.01)

    def shoot(self, target_id, mode="steady"):
        """One shot, judged by the oracle. Returns the verdict or None."""
        self.app.action("/target", {"id": target_id})
        self.settle()
        self.run(self.cfg["min_gap_s"] + 0.1)
        r = self.app.action("/arm", {"mode": mode})
        if not r["ok"]:
            raise AssertionError(r["message"])
        if not self.run(30.0, until=lambda: self.app.state in ("verdict", "idle")):
            raise AssertionError(f"shot never completed ({self.app.state})")
        if self.app.state == "idle":
            return None                                   # refused, e.g. never steady
        tgt = next(t for t in self.app.targets if t[0] == target_id)
        v = self.boat.oracle_verdict(tgt[2], tgt[3])
        ok, msg = self.app.verdict(*v)
        assert ok, msg
        return v

    def level_hit_error(self, target_id):
        """|stream height - target| in metres if the boat sat LEVEL at the
        estimated range: the thing that matters on the day."""
        tgt = next(t for t in self.app.targets if t[0] == target_id)
        (_, fa), _ = self.app.estimates(target_id)
        if fa["centre"] is None:
            return None
        b = self.boat
        x = fa["centre"] - b.nozzle_x - b.setback
        zt, _ = b.target(tgt[2], tgt[3])
        return abs(b.truth.height_at(x) - zt)


def converge(seed, target_id, shots, sea=1.0, p_wrong=0.0):
    s = Session(seed, sea=sea, p_wrong=p_wrong)
    verdicts = [s.shoot(target_id) for _ in range(shots)]
    return s, verdicts


class TestConvergence(unittest.TestCase):

    def _check(self, seeds, target_id, shots, tol_z, sea=1.0, p_wrong=0.0,
               need=1.0):
        good, errs = 0, []
        for seed in seeds:
            s, _ = converge(seed, target_id, shots, sea, p_wrong)
            if s.boat.truth_range(next(t for t in s.app.targets
                                       if t[0] == target_id)[2]) is None:
                continue                          # this draw cannot reach it at all
            err = s.level_hit_error(target_id)
            errs.append(err)
            good += err is not None and err <= tol_z
        frac = good / max(1, len(errs))
        msg = (f"{target_id}: {good}/{len(errs)} sessions hit within "
               f"{tol_z * 100:.1f} cm after {shots} shots; errors cm: "
               + ", ".join("-" if e is None else f"{e * 100:.1f}" for e in errs))
        print(msg)
        self.assertGreaterEqual(frac, need, msg)

    def test_lower_window_calm_honest(self):
        self._check(range(10), "lr_top", 12, 0.02, sea=0.5)

    # Rocking of +-1.5 deg is +-3.6 cm at the face by itself, so in rough water
    # the bar is looser: the operator judges a 3 cm hit on a boat that is never
    # level, and the band the verdicts describe is blurred by it.
    def test_lower_window_rocking(self):
        self._check(range(10), "lr_top", 12, 0.035, sea=1.5)

    def test_lower_window_noisy_operator(self):
        self._check(range(10), "lr_top", 15, 0.025, sea=1.0, p_wrong=0.1, need=0.9)

    def test_upper_window(self):
        # Most random nozzles cannot reach this edge at all (see the next test);
        # these are the ones that can, and they sit near the top of the arc.
        self._check(range(30), "ul_top", 12, 0.02, sea=1.0)


class TestOutOfReach(unittest.TestCase):

    def test_says_so_when_the_stream_cannot_reach_the_edge(self):
        warned = 0
        # Flat water: rocking lifts the odd shot onto an edge the level arc
        # misses, which is real, but this test is about the warning's logic.
        for seed in range(30):
            s = Session(seed, sea=0.0)
            edge = next(t for t in s.app.targets if t[0] == "ul_top")[2]
            zt, _ = s.boat.target(edge, 0.0)
            if s.boat.truth.apex()[1] >= zt - 0.03:
                continue          # reaches it, or gets within the 3 cm a hit allows
            for _ in range(10):
                s.shoot("ul_top")
            (_, fa), _ = s.app.estimates("ul_top")
            self.assertEqual(fa["n_hits"], 0)
            warn = s.app.advice(s.t)["warn"]
            self.assertIn("cannot reach", warn)
            warned += 1
        self.assertGreater(warned, 5)


class TestPilotFired(unittest.TestCase):

    def test_pilot_squirt_is_a_shot_with_pre_fire_range(self):
        s = Session(3)
        s.boat.pump_path = False                     # log only: the tool cannot fire
        self.assertFalse(s.app.action("/arm", {"mode": "now"})["ok"])
        s.run(3.0)
        r = s.app.action("/fake/pilot_squirt", {"burst_s": 0.4})
        self.assertTrue(r["ok"])
        self.assertTrue(s.run(10.0, until=lambda: s.app.state == "verdict"))
        p = s.app.pending
        self.assertEqual(p["source"], "pilot")
        self.assertAlmostEqual(p["burst_s"], 0.4, places=2)
        # the range logged is the pre-fire one, not a spray return
        self.assertAlmostEqual(p["range_m"], s.boat.range, delta=0.05)
        self.assertEqual(p["range_src"], "lidar")
        self.assertTrue(s.app.verdict("fwd", "ok")[0])
        self.assertEqual(s.app.shots[-1]["fa"], "fwd")


class TestSprayDoesNotFoolTheRange(unittest.TestCase):

    def test_logged_range_ignores_spray_returns(self):
        errs = []
        for seed in range(8):
            s = Session(seed)
            s.shoot("lr_top", mode="now")
            shot = s.app.shots[-1]
            errs.append(abs(shot["range_m"] - s.boat.last_hit["x"]
                            - s.boat.nozzle_x - s.boat.setback))
        self.assertLess(statistics.median(errs), 0.03, errs)


if __name__ == "__main__":
    unittest.main(verbosity=2)
