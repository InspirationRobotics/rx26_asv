"""Proof of Readiness (USV) scorer — accepts good runs, rejects bad ones.

A scorer that only ever passes is not a scorer. These tests prove BOTH
directions, because this one gates the 31 August submission: a scorer that
passes a run RoboNation would fail is worse than having no scorer, since it
converts a recoverable early resubmission into a late surprise.

Stdlib + pytest only; no ROS, no SITL, no Gazebo.
"""
import json
import math
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO))
sys.path.insert(0, str(REPO / "orchestrator"))

from episodes.runner import EpisodeResult, Sample          # noqa: E402
from episodes.scenario import Scenario                     # noqa: E402
from evaluator.por_usv import PorUsvScorer, score_por_usv  # noqa: E402
from robotx_2026.api.common import config as crsd_config   # noqa: E402

SCENARIO = REPO / "orchestrator" / "scenarios" / "por_usv_gate.json"


@pytest.fixture
def scenario():
    return Scenario.load(str(SCENARIO))


@pytest.fixture
def scorer():
    return PorUsvScorer(**crsd_config.por_usv_kwargs())


def _trace(points, dt=0.2, mode=None, t0=0.0):
    """Build an EpisodeResult from (x, y) points."""
    res = EpisodeResult(scenario_name="por_usv_gate", scenario_version="0.1",
                        seed=0)
    for i, (x, y) in enumerate(points):
        s = Sample(t0 + i * dt, x, y, 0.0, 1.5)
        if mode is not None:
            s.mode = mode
        res.trace.append(s)
    return res


def _leg(p, q, step=0.25):
    d = math.hypot(q[0] - p[0], q[1] - p[1])
    n = max(2, int(d / step))
    return [(p[0] + (q[0] - p[0]) * i / n, p[1] + (q[1] - p[1]) * i / n)
            for i in range(n + 1)]


def _good_track():
    """Start 3 m behind gate 1, straight through both gate centres."""
    return _leg((0.0, -3.0), (0.0, 18.0))


# ------------------------------------------------------------------ accepts #

def test_accepts_a_clean_run(scorer, scenario):
    r = scorer.score(_trace(_good_track()), scenario)
    failed = [k for k, v in r["criteria"].items() if not v["passed"]]
    assert r["passed"], f"failed: {failed}"


def test_convenience_wrapper_uses_config(scenario):
    """score_por_usv() must read thresholds from crusader_params.yaml."""
    r = score_por_usv(_trace(_good_track()), scenario)
    assert r["passed"]


def test_every_criterion_is_reported(scorer, scenario):
    r = scorer.score(_trace(_good_track()), scenario)
    for expected in ("start_behind_gate", "pass_through_gate_1",
                     "pass_through_gate_2", "gates_in_order",
                     "no_buoy_contact", "fully_autonomous_throughout",
                     "run_fits_video_limit"):
        assert expected in r["criteria"], expected


# ------------------------------------------------------------------ rejects #

def test_rejects_wrong_start_distance(scorer, scenario):
    r = scorer.score(_trace(_leg((0.0, -12.0), (0.0, 18.0))), scenario)
    assert not r["criteria"]["start_behind_gate"]["passed"]
    assert not r["passed"]


def test_rejects_passing_outside_a_gate_post(scorer, scenario):
    """
    The case a naive line-side test gets wrong: rounding the OUTSIDE of a post
    crosses the infinite gate line but not the segment between the posts.
    """
    r = scorer.score(_trace(_leg((-4.0, -3.0), (-4.0, 18.0))), scenario)
    assert not r["criteria"]["pass_through_gate_1"]["passed"]
    assert not r["criteria"]["pass_through_gate_2"]["passed"]


def test_rejects_gates_out_of_order(scorer, scenario):
    """Approaching from beyond gate 2 does not demonstrate the course."""
    track = _leg((0.0, 18.0), (0.0, -3.0))
    r = scorer.score(_trace(track), scenario)
    assert not r["criteria"]["gates_in_order"]["passed"]


def test_rejects_buoy_contact(scorer, scenario):
    track = _good_track() + [(-1.5, 0.0)]        # drive onto gate1_red_port
    r = scorer.score(_trace(track), scenario)
    assert not r["criteria"]["no_buoy_contact"]["passed"]
    assert "gate1_red_port" in r["criteria"]["no_buoy_contact"]["detail"]


def test_rejects_non_autonomous_run(scorer, scenario):
    r = scorer.score(_trace(_good_track(), mode="MANUAL"), scenario)
    assert not r["criteria"]["fully_autonomous_throughout"]["passed"]


def test_accepts_auto_mode_when_backend_reports_it(scorer, scenario):
    r = scorer.score(_trace(_good_track(), mode="AUTO"), scenario)
    c = r["criteria"]["fully_autonomous_throughout"]
    assert c["passed"] and c["verified"]


def test_unreported_mode_is_flagged_unverified_not_silently_passed(
        scorer, scenario):
    """A criterion that always passes is not a criterion."""
    c = scorer.score(_trace(_good_track()), scenario)["criteria"][
        "fully_autonomous_throughout"]
    assert c["passed"] and c["verified"] is False
    assert "NOT VERIFIED" in c["detail"]


def test_rejects_run_over_five_minutes(scorer, scenario):
    r = scorer.score(_trace(_good_track(), dt=5.0), scenario)
    assert not r["criteria"]["run_fits_video_limit"]["passed"]


def test_warns_when_run_is_tight_against_the_limit(scorer, scenario):
    track = _good_track()
    dt = 250.0 / max(1, len(track) - 1)          # ~250 s: under 300, over 240
    r = scorer.score(_trace(track, dt=dt), scenario)
    assert r["criteria"]["run_fits_video_limit"]["passed"]
    assert any("tight" in n for n in r["notes"])


def test_empty_trace_fails_cleanly(scorer, scenario):
    r = scorer.score(EpisodeResult("por_usv_gate", "0.1", 0), scenario)
    assert not r["passed"]


def test_grazing_a_post_is_reported_but_not_failed(scorer, scenario):
    """Diagnostic, not pass/fail — the handbook requires passing between the
    posts, not passing centred. Still worth surfacing before submission."""
    r = scorer.score(_trace(_leg((1.2, -3.0), (1.2, 18.0))), scenario)
    assert r["criteria"]["pass_through_gate_1"]["passed"]
    assert "GRAZING" in r["criteria"]["pass_through_gate_1"]["detail"]


# ----------------------------------------------------------------- scenario #

def test_scenario_matches_handbook_geometry():
    """Handbook 3.1.2: two gates; ~0.23 m radius Sur-Mark markers."""
    sc = json.loads(SCENARIO.read_text())
    assert len(sc["obstacles"]) == 4
    ys = sorted({o["y"] for o in sc["obstacles"]})
    assert len(ys) == 2, "PoR course is TWO gates"
    for o in sc["obstacles"]:
        assert o["radius"] == pytest.approx(0.23, abs=0.05)
    assert sc["timeout_s"] <= 300, "5-minute single-take limit"


def test_gate_pairing_is_ordered_by_y(scenario):
    gates = PorUsvScorer._gates(scenario)
    assert len(gates) == 2
    assert gates[0][0].y < gates[1][0].y
    for port, stbd in gates:
        assert port.x < stbd.x
