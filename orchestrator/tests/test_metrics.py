import math

from episodes.runner import EpisodeResult, Sample
from episodes.scenario import Scenario, Obstacle, KeepOut, MovingObject
from evaluator import metrics


def make_scenario(**kw):
    base = dict(name="t", version="0", origin=(0.0, 0.0), waypoints=[(0, 50)],
                avoid_margin=2.0)
    base.update(kw)
    return Scenario(**base)


def straight_trace(y_from=0, y_to=50, x=0.0, dt=0.5, speed=2.0):
    trace, t, y = [], 0.0, float(y_from)
    while y <= y_to:
        trace.append(Sample(t, x, y, 0.0, speed))
        y += speed * dt
        t += dt
    return trace


def result_with(trace, keepouts=(), moving=()):
    r = EpisodeResult("t", "0", 0)
    r.trace = trace
    r.keepouts = list(keepouts)
    r.moving_objects = list(moving)
    r.waypoints_reached = [True]
    return r


def test_clearance_violation_counted_once_per_entry():
    # obstacle 1.0 m off the path (radius 0.3): clearance dips to 0.7 < margin 2.0
    sc = make_scenario(obstacles=[Obstacle(x=1.0, y=25.0, radius=0.3)])
    o1 = metrics.objective1(result_with(straight_trace()), sc)
    assert o1["clearance_violations"] == 1        # edge-triggered, not per-sample
    assert o1["hard_collisions"] == 0
    assert not o1["auto_fail"]
    assert abs(o1["min_clearance_m"] - 0.7) < 0.15


def test_hard_collision_is_auto_fail():
    sc = make_scenario(obstacles=[Obstacle(x=0.0, y=25.0, radius=0.5)])
    o1 = metrics.objective1(result_with(straight_trace()), sc)
    assert o1["hard_collisions"] > 0
    assert o1["auto_fail"]


def test_moving_object_10m_rule_is_auto_fail():
    m = MovingObject(x0=5.0, y0=25.0, heading_deg=90.0, speed_mps=0.0)
    sc = make_scenario()
    o1 = metrics.objective1(result_with(straight_trace(), moving=[m]), sc)
    assert o1["moving_10m_violations"] >= 1
    assert o1["auto_fail"]


def test_keepout_active_window_respected():
    # keep-out ON the path, but only active after the boat has passed it
    k = KeepOut(x=0.0, y=10.0, radius=2.0, zone_id="K1", active_from=999.0)
    sc = make_scenario()
    o1 = metrics.objective1(result_with(straight_trace(), keepouts=[k]), sc)
    assert o1["keepout_violations"] == 0
    # same zone active from t=0 -> violation
    k2 = KeepOut(x=0.0, y=10.0, radius=2.0, zone_id="K1", active_from=0.0)
    o1b = metrics.objective1(result_with(straight_trace(), keepouts=[k2]), sc)
    assert o1b["keepout_violations"] == 1


def test_objective2_flags_a_stall():
    # boat oscillates in place far from goal -> at-risk flag, escalates to stall
    trace = []
    for i in range(400):                      # 40 s at dt=0.1
        t = i * 0.1
        trace.append(Sample(t, 0.0 + 0.05 * math.sin(t * 3), 5.0,
                            0.4 * math.sin(t * 2), 0.05))
    sc = make_scenario()
    o2 = metrics.objective2(result_with(trace), sc)
    assert o2["at_risk_flags"] >= 1
    assert o2["stalls"] >= 1


def test_objective2_clean_run_has_no_flags():
    sc = make_scenario()
    o2 = metrics.objective2(result_with(straight_trace()), sc)
    assert o2["at_risk_flags"] == 0
    assert o2["stalls"] == 0


def test_assemble_schema_and_nulls():
    sc = make_scenario()
    m = metrics.assemble(result_with(straight_trace()), sc)
    assert m["schema"] == "rx26-episode-metrics/1"
    assert m["objective1"]["trusted"] is False            # pre-G2 default
    m4 = m["objective3"]["mission4"]
    assert set(m4) == {"ack_correct", "ack_latency_s",
                       "resume_fidelity", "comms_compliance"}
    assert all(v is None for v in m4.values())            # explicit nulls until Phase 4
