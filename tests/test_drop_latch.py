"""DropLatch state machine — every rule in the G1 design gets a test."""
from robotx_2026.api.common.drop_latch import DropLatch, DropState


def sample(latch, value, t, ch=7):
    channels = [1500] * 18
    channels[ch - 1] = value
    return latch.rc_sample(channels, t)


def test_startup_blocks_until_safe_sample():
    l = DropLatch(channel=7, threshold=1700)
    assert not l.allowed                       # no data = no override
    sample(l, 1000, t=0.0)                     # safe position seen
    assert l.allowed


def test_startup_with_switch_in_drop_position_stays_blocked_not_latched():
    l = DropLatch()
    newly = sample(l, 1900, t=0.0)
    assert not newly                           # no trip event spam at boot
    assert not l.allowed
    assert l.state == DropState.STARTUP
    sample(l, 1000, t=0.5)                     # pilot moves switch to safe
    assert l.allowed


def test_pilot_commanded_drop_trips_and_latches():
    l = DropLatch()
    sample(l, 1000, t=0.0)
    assert sample(l, 1900, t=1.0) is True      # newly tripped
    assert l.dropped and "pilot commanded" in l.trip_reason
    assert sample(l, 1900, t=1.1) is False     # already latched, no re-trip
    # switch back to safe does NOT auto-clear — latched until explicit reset
    sample(l, 1000, t=2.0)
    assert l.dropped


def test_rc_link_lost_trips():
    l = DropLatch()
    sample(l, 1000, t=0.0)
    assert sample(l, 0, t=1.0) is True
    assert "link lost" in l.trip_reason


def test_stale_rc_trips_via_tick():
    l = DropLatch(stale_timeout=1.0)
    sample(l, 1000, t=0.0)
    assert l.tick(0.5) is False
    assert l.tick(2.0) is True
    assert "stale" in l.trip_reason


def test_reset_rules():
    l = DropLatch(stale_timeout=1.0)
    sample(l, 1000, t=0.0)
    sample(l, 1900, t=1.0)                     # trip
    ok, reason = l.reset(t=1.1)
    assert not ok and "drop position" in reason    # switch still high
    sample(l, 1000, t=2.0)                     # switch back to safe
    ok, _ = l.reset(t=10.0)
    assert not ok                              # data stale at reset time
    sample(l, 1000, t=10.5)
    ok, _ = l.reset(t=10.6)
    assert ok and l.allowed


def test_inverted_polarity():
    l = DropLatch(threshold=1300, invert=True)
    sample(l, 1800, t=0.0)                     # high = safe on inverted switch
    assert l.allowed
    assert sample(l, 1000, t=1.0) is True      # low trips


def test_short_channel_list_counts_as_lost():
    l = DropLatch(channel=7)
    assert l.rc_sample([1500] * 4, 0.0) in (True, False)
    assert not l.allowed                       # missing channel -> value 0 path
