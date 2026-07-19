"""Closed-loop APF advisory tests — the boat's real apf_core driving the runner."""
from pathlib import Path

from episodes.apf_advisor import ApfAdvisor
from episodes.backends.kinematic import KinematicBackend
from episodes.runner import EpisodeRunner
from episodes.scenario import Scenario
from evaluator import metrics

SCENARIOS = Path(__file__).parent.parent / "scenarios"


def run(advised, seed=0, noise=0.02):
    sc = Scenario.load(str(SCENARIOS / "mission1_obstacle_field.json"))
    advisor = ApfAdvisor(sc) if advised else None
    res = EpisodeRunner(sc, KinematicBackend(noise_std=noise), seed=seed,
                        advisor=advisor).run()
    return metrics.assemble(res, sc), advisor


def test_naive_run_violates_clearance():
    m, _ = run(advised=False)
    o1 = m["objective1"]
    assert o1["clearance_violations"] > 0 or o1["hard_collisions"] > 0


def test_advised_run_clean_across_seeds():
    for seed in (0, 1, 2):
        m, advisor = run(advised=True, seed=seed)
        o1 = m["objective1"]
        assert o1["clearance_violations"] == 0, (seed, o1)
        assert o1["hard_collisions"] == 0
        assert m["objective3"]["completed"]
        assert m["objective2"]["stalls"] == 0
        assert advisor.max_repulsion > 0          # the advisor actually acted


def test_advisory_never_moves_the_mission():
    # waypoint acceptance is against the ORIGINAL waypoints — completion with
    # the advisor on proves shaping never redefined the mission goals
    m, _ = run(advised=True)
    assert set(m["objective3"]["elements"]) == {"waypoint_0", "waypoint_1"}
    assert m["objective3"]["partial_credit"] == 1.0
