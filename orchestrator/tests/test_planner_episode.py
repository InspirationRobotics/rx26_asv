"""Planner-driven episode tests, including the negative case that proves the
resume-fidelity check catches the 'resume-state drift' known failure mode
(a resume that silently restarts still finishes and passes naive checks)."""
import sys
from pathlib import Path

from episodes.apf_advisor import ApfAdvisor
from episodes.backends.kinematic import KinematicBackend
from episodes.planner_episode import PlannerEpisodeRunner
from episodes.scenario import Scenario
from evaluator import metrics

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))
from robotx_2026.api.mission.tasks.waypoint_mission import WaypointMission  # noqa: E402

SCENARIOS = Path(__file__).parent.parent / "scenarios"


def run(seed=0, noise=0.02, runner_cls=PlannerEpisodeRunner, **kw):
    sc = Scenario.load(str(SCENARIOS / "mission4_core.json"))
    result, planner = runner_cls(sc, KinematicBackend(noise_std=noise),
                                 seed=seed, advisor=ApfAdvisor(sc), **kw).run()
    return metrics.assemble(result, sc, planner=planner), result, planner


def test_mission4_core_full_cycle():
    m, result, planner = run()
    m4 = m["objective3"]["mission4"]
    assert m["objective3"]["completed"]
    assert m4["comms_compliance"] == 1.0
    assert m4["ack_correct"] is True
    assert m4["resume_fidelity"] is True
    # keep-out was injected via comms, acked, cleared — and never entered
    assert m["objective1"]["keepout_violations"] == 0
    assert result.keepouts and result.keepouts[0].zone_id == "K1"
    assert result.keepouts[0].active_until < float("inf")     # All Clear applied
    # commanded loiter must not read as a stall (the G4 evaluator fix)
    assert m["objective2"]["stalls"] == 0
    # exactly one push (assistance), keep-outs stayed off the stack
    assert len(planner.stack.push_log) == 1


def test_boat_visits_assist_point():
    _, result, _ = run()
    import math
    d_min = min(math.hypot(25.0 - s.x, 25.0 - s.y) for s in result.trace)
    assert d_min <= 3.0            # loiter radius


class RestartingMission(WaypointMission):
    """Deliberately broken: resume() silently restarts from waypoint 0 —
    the exact drift the plan says a naive 'it finished' check would miss."""

    def resume(self, ctx):
        self.wp_index = 0
        self.reached = [False] * len(self.waypoints)


def test_silent_restart_is_caught_by_fidelity_check(monkeypatch):
    from episodes import planner_episode as pe
    monkeypatch.setattr(pe, "WaypointMission", RestartingMission)
    m, result, planner = run()
    m4 = m["objective3"]["mission4"]
    # the broken mission still eventually finishes (naive check would pass)...
    assert m["objective3"]["completed"]
    # ...but the explicit fidelity check fails it
    assert m4["resume_fidelity"] is False
