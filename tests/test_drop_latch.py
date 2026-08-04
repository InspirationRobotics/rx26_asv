"""DropLatch state machine — every rule in the G1 design gets a test."""
from rx26_asv.api.common.drop_latch import DropLatch, DropState


def sample(latch, value, t, ch=None):
    ch = latch.channel if ch is None else ch
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


def test_default_channel_is_outside_the_override_writable_range():
    """_send_override truncates to 8 channels, so a default drop channel <= 8
    would be one an override mechanism could drive."""
    assert DropLatch().channel >= 9


# --- SYS_STATUS RC-receiver health verdict (the 2026-08-02 blind spot) ---
#
# The scenario every test below encodes: ELRS receiver failsafe set to "Last
# Position", transmitter powered OFF. RC_CHANNELS keeps arriving at 20 Hz with
# the held stick values, so neither the zero check nor the staleness check can
# fire. Only ArduPilot's own verdict sees it.


def test_held_pwm_through_dead_tx_does_not_trip_without_the_health_bit():
    """Documents the defect itself: with PWM alone, a dead transmitter whose
    receiver holds last position leaves the latch ACTIVE. If this ever starts
    failing, the PWM path gained a detection and this test should be revisited —
    it is not asserting desirable behaviour, it is pinning the gap the health
    verdict exists to close."""
    l = DropLatch(threshold=1700, stale_timeout=1.0)
    sample(l, 1000, t=0.0)
    assert l.allowed
    for i in range(1, 40):                     # 2 s of held, valid PWM
        sample(l, 1000, t=i * 0.05)
        assert l.tick(i * 0.05) is False
    assert l.allowed                           # <-- the boat has no pilot


def test_unhealthy_verdict_trips_the_same_scenario():
    l = DropLatch(threshold=1700, stale_timeout=1.0, health_timeout=3.0)
    sample(l, 1000, t=0.0)
    assert l.allowed
    assert l.note_rc_health(False, t=0.5) is True
    assert l.dropped and "UNHEALTHY" in l.trip_reason
    sample(l, 1000, t=0.6)                     # held PWM keeps arriving
    assert not l.allowed                       # latched, as designed


def test_unknown_verdict_is_never_treated_as_healthy_or_unhealthy():
    """None = the autopilot never advertised the bit. Strictly additive: it must
    neither trip nor promote, leaving the PWM checks the sole decider."""
    l = DropLatch(threshold=1700)
    assert l.note_rc_health(None, t=0.0) is False
    sample(l, 1000, t=0.1)
    assert l.allowed                           # PWM path still governs
    assert l.tick(0.2) is False


def test_stale_verdict_falls_back_to_the_pwm_checks():
    """A verdict older than health_timeout must stop trip-ing on its own, or a
    single unhealthy report would latch the boat forever after SYS_STATUS
    recovers or stops."""
    l = DropLatch(threshold=1700, stale_timeout=1.0, health_timeout=3.0)
    sample(l, 1000, t=0.0)
    l.note_rc_health(False, t=0.1)             # trips
    ok, reason = l.reset(t=0.2)
    assert not ok and "unhealthy" in reason    # fresh verdict blocks reset
    sample(l, 1000, t=5.0)                     # verdict now stale (>3 s)
    ok, _ = l.reset(t=5.1)
    assert ok and l.allowed                    # falls back to PWM, reset allowed


def test_unhealthy_at_boot_blocks_without_latching():
    """A benign boot order (nodes up before the transmitter) must not require an
    operator service call — stay STARTUP, promote once the link is real."""
    l = DropLatch(threshold=1700, health_timeout=3.0)
    assert l.note_rc_health(False, t=0.0) is False
    sample(l, 1000, t=0.1)                     # valid-looking held PWM
    assert not l.allowed                       # must NOT promote on a dead link
    assert l.state == DropState.STARTUP        # blocked, not latched
    l.note_rc_health(True, t=0.2)              # transmitter comes up
    sample(l, 1000, t=0.3)
    assert l.allowed


def test_tick_enforces_health_regardless_of_arrival_order():
    l = DropLatch(threshold=1700, stale_timeout=1.0, health_timeout=3.0)
    sample(l, 1000, t=0.0)
    assert l.allowed
    l._health, l._health_t = False, 0.1        # verdict landed without a trip
    assert l.tick(0.2) is True                 # tick must still catch it
    assert "UNHEALTHY" in l.trip_reason


def test_health_verdict_can_only_add_trips_never_suppress_one():
    """A HEALTHY verdict must not rescue a latch the PWM path already tripped."""
    l = DropLatch(threshold=1700)
    sample(l, 1000, t=0.0)
    sample(l, 1900, t=1.0)                     # pilot commanded drop
    assert l.dropped
    l.note_rc_health(True, t=1.1)
    assert l.dropped and not l.allowed
