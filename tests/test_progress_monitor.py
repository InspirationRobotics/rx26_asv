import math

from rx26_asv.api.navigation.progress_monitor import ProgressMonitor


def run(mon, samples):
    last = None
    for s in samples:
        last = mon.update(*s)
    return last


def moving_north(t0=0.0, n=200, dt=0.1, speed=2.0):
    return [(t0 + i * dt, 0.0, i * dt * speed, 0.0, speed, 0.0, 100.0)
            for i in range(n)]


def oscillating_stuck(t0=0.0, n=300, dt=0.1):
    return [(t0 + i * dt, 0.05 * math.sin(i * 0.3), 20.0,
             0.4 * math.sin(i * 0.2), 0.05, 0.0, 100.0)
            for i in range(n)]


def test_healthy_transit_stays_ok():
    m = ProgressMonitor()
    assert run(m, moving_north()) == m.OK
    assert m.flags_raised == 0


def test_stuck_flags_before_stall_then_stalls():
    m = ProgressMonitor(window_s=10.0, stall_after_s=15.0)
    states = []
    for s in oscillating_stuck():
        states.append(m.update(*s))
    assert m.AT_RISK in states               # preventative flag raised first
    assert states.index(m.AT_RISK) < len(states) - 1
    assert states[-1] == m.STALLED           # persisted -> stall
    assert m.flags_raised == 1 and m.stalls == 1
    # flag must precede the stall by the configured margin (preventative!)
    t_flag = states.index(m.AT_RISK) * 0.1
    t_stall = states.index(m.STALLED) * 0.1
    assert t_stall - t_flag >= 15.0 - 1e-9


def test_recovery_counts_escape():
    m = ProgressMonitor(window_s=10.0)
    for s in oscillating_stuck(n=150):
        m.update(*s)
    assert m.state == m.AT_RISK
    for s in moving_north(t0=15.0):
        m.update(*s)
    assert m.state == m.OK
    assert m.escapes == 1


def test_at_goal_never_flags():
    m = ProgressMonitor(goal_radius=2.0)
    # sitting still ON the goal (e.g. dp_hold holding position) is not a stall
    samples = [(i * 0.1, 0.0, 0.0, 0.01 * i, 0.0, 0.5, 0.5) for i in range(300)]
    assert run(m, samples) == m.OK
    assert m.flags_raised == 0
