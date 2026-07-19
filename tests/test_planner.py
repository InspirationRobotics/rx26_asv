import queue

import pytest

from robotx_2026.api.mission.events import (AllClear, AssistanceRequest,
                                            Clearance, KeepOutZone,
                                            MovingObjectReport, StatusKind)
from robotx_2026.api.mission.planner import MissionPlanner, PlannerConfig
from robotx_2026.api.mission.task_stack import TaskContext, TaskStack
from robotx_2026.api.mission.tasks.waypoint_mission import (LoiterAssist,
                                                            WaypointMission)


class FakeComms:
    def __init__(self):
        self.sent = []

    def send_status(self, kind, ref_id, t, position=None):
        self.sent.append((kind, ref_id, t))


class FakeSink:
    def __init__(self):
        self.keepouts, self.cleared, self.movings = [], [], []

    def keepout(self, zone_id, x, y, r, t):
        self.keepouts.append((zone_id, x, y, r, t))

    def clear(self, ref, t):
        self.cleared.append((ref, t))

    def moving(self, ev, t):
        self.movings.append((ev.object_id, t))


def make_planner(waypoints=((0, 10), (0, 30)), sink=None, strict=True,
                 **cfg_kw):
    q = queue.Queue()
    comms = FakeComms()
    cfg = PlannerConfig(loiter_radius=3.0, loiter_min_s=2.0, **cfg_kw)
    planner = MissionPlanner(WaypointMission(list(waypoints)), comms, q,
                             avoidance_sink=sink, config=cfg, strict=strict)
    return planner, q, comms


# ---------- task-layer units ----------

def test_waypoint_mission_suspend_resume_roundtrip():
    m = WaypointMission([(0, 10), (0, 30), (0, 50)])
    assert m.tick(0, 0, 0) == (0, 10)
    m.tick(1, 0, 10)                       # reach wp0 -> advances
    assert m.tick(2, 0, 12) == (0, 30)
    ctx = m.suspend()
    m2 = WaypointMission([(0, 10), (0, 30), (0, 50)])
    m2.resume(ctx)
    assert m2.wp_index == 1 and m2.reached[0]
    assert m2.tick(3, 0, 12) == (0, 30)    # continues, does NOT restart


def test_loiter_dwell_resets_on_leaving():
    l = LoiterAssist((10, 10), loiter_radius=3.0, loiter_min_s=5.0)
    l.tick(0, 10, 10)
    assert not l.on_station(4)             # not enough dwell yet
    l.tick(4, 20, 20)                      # drifted out -> dwell resets
    l.tick(5, 10, 10)
    assert not l.on_station(9)
    assert l.on_station(10.1)


def test_task_stack_pop_empty_fails_loudly():
    with pytest.raises(IndexError):
        TaskStack().pop()


# ---------- planner: Core-tier interrupt cycle ----------

def test_full_interrupt_cycle_with_acks_and_resume_fidelity():
    planner, q, comms = make_planner(waypoints=[(0, 10), (0, 30)])
    assert planner.tick(0, 0, 0) == (0, 10)
    planner.tick(1, 0, 10)                 # wp0 reached; heading to wp1

    q.put(AssistanceRequest("A1", x=20.0, y=15.0))
    goal = planner.tick(2, 0, 12)
    assert goal == (20.0, 15.0)            # diverted to assist point
    assert planner.state == planner.INTERRUPT
    kinds = [k for k, _, _ in comms.sent]
    assert kinds[:2] == [StatusKind.ACK_RECEIPT, StatusKind.ACK_INTENT]

    # arrive and dwell -> READINESS exactly once
    planner.tick(10, 20, 15)
    planner.tick(13, 20, 15)
    planner.tick(14, 20, 15)
    assert [k for k, _, _ in comms.sent].count(StatusKind.READINESS) == 1

    # clearance BEFORE readiness would not resume; here readiness is sent
    q.put(Clearance("A1"))
    goal = planner.tick(15, 20, 15)
    assert planner.state == planner.MISSION
    assert goal == (0, 30)                 # resumed toward wp1, NOT wp0
    assert [k for k, _, _ in comms.sent][-1] == StatusKind.RESUMPTION

    # resume fidelity: recorded pre/post contexts identical
    assert planner.resume_record["pre_suspend"] == \
        planner.resume_record["post_resume"]

    # finish the mission
    planner.tick(16, 0, 30)
    assert planner.tick(17, 0, 30) is None
    assert planner.state == planner.DONE
    assert planner.mission.reached == [True, True]


def test_clearance_for_wrong_request_does_not_resume():
    planner, q, comms = make_planner()
    q.put(AssistanceRequest("A1", x=20.0, y=15.0))
    planner.tick(0, 0, 0)
    planner.tick(10, 20, 15)
    planner.tick(13, 20, 15)               # readiness sent
    q.put(Clearance("WRONG"))
    planner.tick(14, 20, 15)
    assert planner.state == planner.INTERRUPT


def test_no_resume_before_readiness():
    planner, q, comms = make_planner()
    q.put(AssistanceRequest("A1", x=20.0, y=15.0))
    q.put(Clearance("A1"))                 # clearance arrives instantly
    planner.tick(0, 0, 0)
    planner.tick(1, 0, 0)                  # still transiting — not on station
    assert planner.state == planner.INTERRUPT
    planner.tick(10, 20, 15)
    planner.tick(13, 20, 15)               # on station + clearance held -> resume
    planner.tick(13.5, 20, 15)
    assert planner.state == planner.MISSION


def test_keepouts_never_touch_the_stack():
    sink = FakeSink()
    planner, q, comms = make_planner(sink=sink)
    q.put(KeepOutZone("K1", x=5.0, y=5.0, radius=4.0))
    q.put(MovingObjectReport("M1", x=30.0, y=0.0, heading_deg=90, speed_mps=1))
    goal = planner.tick(0, 0, 0)
    assert goal == (0, 10)                 # mission uninterrupted
    assert planner.state == planner.MISSION
    assert planner.stack.depth == 0        # NOT a stack entry
    assert sink.keepouts == [("K1", 5.0, 5.0, 4.0, 0)]
    assert sink.movings == [("M1", 0)]
    assert (0, "KEEPOUT_ACK", "K1") in planner.comms_log

    q.put(AllClear("K1"))
    planner.tick(1, 0, 0)
    assert sink.cleared == [("K1", 1)]
    assert (1, "ALLCLEAR_ACK", "K1") in planner.comms_log


def test_unknown_event_raises_in_strict_mode():
    planner, q, _ = make_planner(strict=True)
    q.put({"not": "an event"})
    with pytest.raises(ValueError):
        planner.tick(0, 0, 0)


def test_bad_event_contained_at_runtime():
    # audit finding #4: at runtime a malformed event degrades to a dropped
    # event + counter — never a dead planner
    planner, q, _ = make_planner(strict=False)
    q.put({"not": "an event"})
    q.put(AssistanceRequest("A1", x=20.0, y=15.0))
    goal = planner.tick(0, 0, 0)
    assert planner.event_errors == 1 and planner.errors
    assert goal == (20.0, 15.0)            # stream kept working after the bad one
    assert planner.state == planner.INTERRUPT


def test_context_json_roundtrip():
    ctx = TaskContext("waypoint_mission", {"wp_index": 2, "reached": [True, True, False]})
    assert TaskContext.from_json(ctx.to_json()) == ctx


# ---------- audit-fix regression tests ----------

def test_second_request_deferred_intent_not_lied(monkeypatch=None):
    # audit finding #1: a second request during INTERRUPT gets ACK_RECEIPT but
    # NOT ACK_INTENT until its service actually begins — no promised-and-
    # dropped requests
    planner, q, comms = make_planner(waypoints=[(0, 10)])
    q.put(AssistanceRequest("A1", x=20.0, y=15.0))
    planner.tick(0, 0, 0)
    q.put(AssistanceRequest("A2", x=-10.0, y=8.0))
    planner.tick(1, 5, 5)
    a2_kinds = [k for k, r, _ in comms.sent if r == "A2"]
    assert a2_kinds == [StatusKind.ACK_RECEIPT]     # receipt yes, intent NOT yet
    assert len(planner.pending_requests) == 1

    # service A1 to completion
    planner.tick(10, 20, 15)
    planner.tick(13, 20, 15)                        # readiness A1
    q.put(Clearance("A1"))
    goal = planner.tick(14, 20, 15)
    # A2 now committed: intent sent, diverted to A2's point
    a2_kinds = [k for k, r, _ in comms.sent if r == "A2"]
    assert a2_kinds == [StatusKind.ACK_RECEIPT, StatusKind.ACK_INTENT]
    assert goal == (-10.0, 8.0)
    assert planner.state == planner.INTERRUPT
    assert planner.active_request == "A2"

    # and A2 completes normally too
    planner.tick(20, -10, 8)
    planner.tick(23, -10, 8)
    q.put(Clearance("A2"))
    planner.tick(24, -10, 8)
    assert planner.state == planner.MISSION
    kinds_a2 = [k for k, r, _ in comms.sent if r == "A2"]
    assert kinds_a2[-1] == StatusKind.RESUMPTION
    assert len(planner.stack.push_log) == 2         # both were real services


def test_loiter_hysteresis_survives_boundary_jitter():
    # audit finding #2: jitter straddling loiter_radius must not reset dwell
    l = LoiterAssist((0, 0), loiter_radius=3.0, loiter_min_s=5.0,
                     hysteresis=1.15)
    l.tick(0.0, 0.0, 2.9)                  # inside -> dwell starts
    for i in range(1, 50):                 # oscillate 2.9..3.3 (< exit 3.45)
        r = 2.9 if i % 2 else 3.3
        l.tick(i * 0.1, 0.0, r)
    assert l.on_station(5.1)               # dwell accumulated despite jitter
    # but a genuine departure past the exit radius still resets
    l.tick(6.0, 0.0, 3.6)
    assert not l.on_station(6.1)


def test_interrupt_stalled_warning_no_auto_resume():
    # audit finding #3: no Clearance -> loud warning, but NEVER auto-resume
    planner, q, _ = make_planner(interrupt_warn_s=30.0)
    q.put(AssistanceRequest("A1", x=20.0, y=15.0))
    planner.tick(0, 0, 0)
    planner.tick(10, 20, 15)
    planner.tick(45, 20, 15)               # > warn threshold, still no clearance
    assert planner.warnings and "stalled" in planner.warnings[0][1]
    assert len(planner.warnings) == 1      # warned once, not spammed
    planner.tick(60, 20, 15)
    assert len(planner.warnings) == 1
    assert planner.state == planner.INTERRUPT   # holding, per the rules


def test_event_drain_cap_bounds_a_storm():
    planner, q, _ = make_planner(max_events_per_tick=50)
    for i in range(200):
        q.put(Clearance(f"X{i}"))
    planner.tick(0, 0, 0)
    assert q.qsize() == 150                # capped; remainder deferred
    planner.tick(1, 0, 0)
    assert q.qsize() == 100


def test_resume_rejects_mismatched_context():
    # audit finding #7
    m = WaypointMission([(0, 10), (0, 30)])
    with pytest.raises(ValueError, match="wp_index"):
        m.resume(TaskContext("waypoint_mission",
                             {"wp_index": 7, "reached": [False, False]}))
    with pytest.raises(ValueError, match="reached"):
        m.resume(TaskContext("waypoint_mission",
                             {"wp_index": 1, "reached": [False, False, False]}))


def test_from_json_malformed_raises_clear_error():
    # audit finding #8
    with pytest.raises(ValueError, match="malformed"):
        TaskContext.from_json("{truncated")
    with pytest.raises(ValueError, match="missing"):
        TaskContext.from_json('{"task_name": "x"}')
    with pytest.raises(ValueError, match="types"):
        TaskContext.from_json('{"task_name": "x", "state": []}')


def test_to_json_handles_numpy_scalars():
    # audit finding #6
    import numpy as np
    ctx = TaskContext("t", {"eta": np.float64(3.5), "idx": np.int64(2),
                            "vec": np.array([1.0, 2.0])})
    d = TaskContext.from_json(ctx.to_json())
    assert d.state == {"eta": 3.5, "idx": 2, "vec": [1.0, 2.0]}
