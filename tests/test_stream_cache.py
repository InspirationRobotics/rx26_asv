"""StreamCache + the staleness-laundering scenario it exists to prevent.

telemetry_bridge cached each MAVLink stream and republished it at 20 Hz with
`header.stamp = now`, with no check that MAVProxy was still delivering. A dead
gateway therefore looked exactly like a healthy one, which disabled the two
consumers built to notice silence:

  * rc_heartbeat_watchdog judges the RC link by the PWM on /crsd/rc_channels and
    the gateway by the arrival of ANY bridge topic. Both timers were refreshed
    by the republished cache, so its force-disarm and its explicit
    `gateway_down_in_loss` branch became unreachable.
  * frame_transform / occupancy_grid_node kept placing obstacles against a
    frozen pose while the boat drove on.
"""
import pytest

from rx26_asv.api.common.stream_cache import StreamCache
from rx26_asv.api.safety.rc_heartbeat_core import RcHeartbeatCore, WatchdogConfig


# ---------------------------------------------------------------- StreamCache

def test_rejects_a_nonpositive_timeout():
    with pytest.raises(ValueError):
        StreamCache(0.0)


def test_nothing_received_reads_as_absent():
    c = StreamCache(1.0)
    assert c.get(0.0) is None
    assert not c.fresh(0.0)
    assert c.age(0.0) is None


def test_fresh_value_is_returned():
    c = StreamCache(1.0)
    c.set("v", 10.0)
    assert c.get(10.5) == "v"
    assert c.age(10.5) == pytest.approx(0.5)


def test_value_expires_at_the_timeout():
    c = StreamCache(1.0)
    c.set("v", 10.0)
    assert c.get(10.99) == "v"
    assert c.get(11.0) is None            # boundary is exclusive
    assert c.get(60.0) is None


def test_a_new_value_refreshes():
    c = StreamCache(1.0)
    c.set("old", 10.0)
    assert c.get(11.5) is None
    c.set("new", 11.5)
    assert c.get(11.6) == "new"


def test_stamp_is_captured_at_receipt():
    """The stamp must travel with the value, so a republished frame carries the
    age it actually has instead of being restamped 'now' at publish time."""
    c = StreamCache(1.0)
    c.set("v", 10.0, stamp="t=10")
    assert c.stamp == "t=10"


def test_went_stale_is_a_one_shot_edge():
    c = StreamCache(1.0)
    c.set("v", 10.0)
    assert not c.went_stale(10.5)         # still fresh
    assert c.went_stale(11.5)             # fires once
    assert not c.went_stale(12.0)         # ...and only once
    assert not c.went_stale(99.0)


def test_went_stale_rearms_after_recovery():
    c = StreamCache(1.0)
    c.set("v", 10.0)
    assert c.went_stale(11.5)
    c.set("v", 12.0)
    assert not c.went_stale(12.1)
    assert c.went_stale(13.5)             # a second dropout is reported again


def test_startup_silence_is_not_a_dropout():
    """Never having received is different from having gone quiet — the same
    distinction rc_heartbeat_core draws by seeding last_bridge_msg 'down'."""
    c = StreamCache(1.0)
    assert not c.went_stale(0.0)
    assert not c.went_stale(1000.0)


# ------------------------------------------- the watchdog scenario, end to end

WD = WatchdogConfig(heartbeat_timeout=1.0, min_valid_pwm=900, require_armed=True,
                    link_timeout=3.0, enable_latch=False, reset_low=1300,
                    reset_high=1700)


def run_watchdog(publish_while_stale, seconds=60.0, hz=20.0):
    """Drive RcHeartbeatCore the way telemetry_bridge feeds it, from t=0 when
    MAVProxy dies. Returns (first_disarm_t, first_gateway_down_t).

    publish_while_stale=True reproduces the old bridge: keep republishing the
    last cached frame forever. False is the fixed bridge: a stale stream simply
    stops being published, so silence stays silent.
    """
    core = RcHeartbeatCore(WD, now=0.0)
    core.note_fcu(0.0, armed=True)
    core.note_rc(0.0, monitored_valid=True, reset_pwm=1500)
    cache = StreamCache(1.0)
    cache.set(True, 0.0)                  # last frame from a now-dead MAVProxy

    disarm_t = gateway_t = None
    for i in range(1, int(seconds * hz) + 1):
        t = i / hz
        if publish_while_stale or cache.get(t) is not None:
            core.note_rc(t, monitored_valid=True, reset_pwm=1500)
            core.note_fcu(t, armed=True)
        d = core.tick(t)
        if d.disarm_request and disarm_t is None:
            disarm_t = t
        if d.gateway_down_in_loss and gateway_t is None:
            gateway_t = t
    return disarm_t, gateway_t


def test_dead_gateway_reaches_the_watchdog_when_stale_streams_stop():
    disarm_t, gateway_t = run_watchdog(publish_while_stale=False)

    # Detection latency COMPOSES: the bridge withholds the stream after its own
    # stream_timeout (1.0s), and only then does the watchdog's heartbeat_timeout
    # (1.0s) start running. Pinned because it is a safety-relevant number --
    # lengthening stream_timeout silently delays every force-disarm downstream.
    assert disarm_t is not None, "watchdog never requested a force-disarm"
    assert disarm_t == pytest.approx(2.0, abs=0.15)

    # Later the bridge's own link_timeout (3.0s) elapses too: the watchdog can no
    # longer route a disarm through the gateway and must SAY so rather than go
    # quiet, so ArduPilot's onboard failsafe is known to be the last line.
    assert gateway_t is not None, "watchdog never reported the gateway down"
    assert gateway_t == pytest.approx(3.95, abs=0.15)
    assert gateway_t > disarm_t


def test_republishing_a_stale_cache_hides_the_dead_gateway():
    """Characterises the defect: this is exactly what the old bridge did.

    Neither safety path ever fires, because every republished frame refreshes
    both of the watchdog's timers.
    """
    disarm_t, gateway_t = run_watchdog(publish_while_stale=True)
    assert disarm_t is None and gateway_t is None, (
        "this scenario is meant to demonstrate the FAILURE; if the watchdog now "
        "reacts, this characterisation test is stale and should be removed")
