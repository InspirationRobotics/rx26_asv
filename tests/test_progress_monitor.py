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


# --- regression: the at-risk signal must not depend on which way north is ---
# roa_apf_node feeds math.radians(compass_deg), i.e. 0..2*pi, so a plain pstdev
# over raw headings hits its discontinuity at due NORTH: half a degree of jitter
# there read as ~3.13 rad of "oscillation" against a 0.05 rad floor, and any
# slow, non-progressing boat pointing north was auto-flagged.

def stuck_at_heading(heading_deg, jitter_deg=0.5, n=200, dt=0.1):
    """Stopped, no goal progress, and only jitter_deg of heading wobble.

    Headings are built the way roa_apf_node builds them: compass degrees
    wrapped into 0..360, then math.radians -> 0..2*pi. Modelling that domain
    is the whole point — a helper that lets the angle go negative around north
    never crosses the discontinuity and cannot see the bug.
    """
    return [(i * dt, 0.0, 20.0,
             math.radians((heading_deg + (jitter_deg if i % 2 else -jitter_deg))
                          % 360.0),
             0.0, 0.0, 100.0)
            for i in range(n)]


def test_at_risk_is_invariant_to_compass_direction():
    states = {}
    for name, hdg in (("north", 0.0), ("east", 90.0),
                      ("south", 180.0), ("west", 270.0)):
        m = ProgressMonitor()
        states[name] = (run(m, stuck_at_heading(hdg)), m.flags_raised)
    assert len(set(states.values())) == 1, \
        f"identical motion scored differently by heading: {states}"


def test_steady_heading_across_the_wrap_is_not_oscillation():
    m = ProgressMonitor()
    assert run(m, stuck_at_heading(0.0)) == m.OK      # jitter across 0/360
    assert m.flags_raised == 0


def test_genuine_oscillation_still_flags_at_every_heading():
    for hdg in (0.0, 90.0, 180.0, 270.0):
        m = ProgressMonitor()
        assert run(m, stuck_at_heading(hdg, jitter_deg=25.0)) != m.OK, \
            f"real oscillation at {hdg} deg was missed"


def test_heading_spread_matches_pstdev_away_from_the_wrap():
    """Off the wrap the new metric must be the old one, so shared
    config/crusader_params.yaml thresholds need no retuning."""
    from statistics import pstdev
    from rx26_asv.api.navigation.progress_monitor import heading_spread
    angles = [math.radians(90 + 25 * math.sin(i / 3)) for i in range(50)]
    assert abs(heading_spread(angles) - pstdev(angles)) < 1e-9
