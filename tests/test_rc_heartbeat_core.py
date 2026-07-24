"""Unit tests for the RC-heartbeat force-disarm state machine (rc_heartbeat_core).

Safety-critical: this machine decides when the boat is force-disarmed on RC-link
loss. Every transition below is a behaviour the field procedure depends on, so
they are asserted here rather than discovered on the water. ROS-free by design —
these run under plain pytest (Format best-practice #2)."""
from robotx_2026.api.safety.rc_heartbeat_core import (
    RcHeartbeatCore, WatchdogConfig)


def cfg(**over):
    base = dict(heartbeat_timeout=1.0, min_valid_pwm=900, require_armed=True,
                link_timeout=3.0, enable_latch=False, reset_low=1300,
                reset_high=1700)
    base.update(over)
    return WatchdogConfig(**base)


def make(now=100.0, **over):
    return RcHeartbeatCore(cfg(**over), now=now)


# ---------------- healthy / basic loss ----------------

def test_healthy_link_never_disarms():
    c = make(now=100.0)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    d = c.tick(100.5)                       # well within heartbeat_timeout
    assert not d.disarm_request and not d.kill_active


def test_disarm_on_link_loss_then_auto_recovers_nonlatch():
    c = make(now=100.0)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    # 1.5 s of stale RC, but the bridge is still fresh (last topic at t=100)
    d = c.tick(101.5)
    assert d.disarm_request and d.kill_active and not d.engaged
    # link returns -> non-latch machine recovers on its own
    c.note_rc(102.0, monitored_valid=True, reset_pwm=1500)
    d = c.tick(102.1)
    assert not d.disarm_request and not d.kill_active


def test_require_armed_gates_disarm_when_disarmed():
    c = make(now=100.0)
    c.note_fcu(103.5, armed=False)          # keeps the bridge fresh, vehicle safe
    d = c.tick(103.6)                        # RC long stale, but not armed
    assert not d.disarm_request and not d.kill_active


def test_disarms_when_armed_even_if_require_armed():
    c = make(now=100.0)
    c.note_fcu(103.5, armed=True)
    d = c.tick(103.6)
    assert d.disarm_request and d.kill_active


# ---------------- gateway-down safety case ----------------

def test_bridge_down_during_loss_defers_to_ardupilot():
    # RC lost AND telemetry_bridge silent: we cannot route a disarm through the
    # gateway, so the watchdog must NOT claim to and must flag the case.
    c = make(now=100.0)
    c.note_fcu(100.0, armed=True)           # last bridge topic at t=100
    d = c.tick(104.0)                        # bridge age 4 s > link_timeout 3 s
    assert d.gateway_down_in_loss
    assert not d.disarm_request and not d.kill_active


def test_startup_silence_does_not_disarm():
    # Fresh boot, no topics yet: bridge is "down" and RC is optimistic — nothing
    # should ever fire on startup silence.
    c = make(now=100.0)
    for t in (100.001, 150.0, 500.0):
        d = c.tick(t)
        assert not d.disarm_request


# ---------------- latch behaviour ----------------

def test_latch_holds_after_link_returns():
    c = make(now=100.0, enable_latch=True)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    d = c.tick(101.5)                        # trip
    assert d.engaged and c.killed and d.disarm_request
    # RC link recovers, but the latch must hold and keep re-enforcing the kill
    c.note_rc(102.0, monitored_valid=True, reset_pwm=1500)
    d = c.tick(102.1)
    assert c.killed and d.kill_active and d.disarm_request and not d.latch_cleared


def test_latch_cleared_by_low_high_toggle():
    c = make(now=100.0, enable_latch=True)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    c.tick(101.5)                            # tripped + latched
    # operator drives the reset switch LOW (link healthy again)
    c.note_rc(102.0, monitored_valid=True, reset_pwm=1200)
    assert not c.tick(102.1).latch_cleared   # LOW alone does not clear
    # then HIGH -> clears
    c.note_rc(102.5, monitored_valid=True, reset_pwm=1800)
    d = c.tick(102.6)
    assert d.latch_cleared and not c.killed and not d.kill_active


def test_latch_not_cleared_by_high_without_prior_low():
    c = make(now=100.0, enable_latch=True)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    c.tick(101.5)
    # reconnect with the switch already HIGH — must NOT auto-clear
    c.note_rc(102.0, monitored_valid=True, reset_pwm=1800)
    d = c.tick(102.1)
    assert not d.latch_cleared and c.killed


def test_zero_pwm_during_loss_never_clears_latch():
    c = make(now=100.0, enable_latch=True)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    c.tick(101.5)
    # link still down: reset channel reads 0 (RC loss). Not a valid toggle.
    c.note_rc(102.0, monitored_valid=False, reset_pwm=0)
    d = c.tick(102.1)
    assert not d.latch_cleared and c.killed and d.kill_active


def test_latch_only_engages_once():
    c = make(now=100.0, enable_latch=True)
    c.note_fcu(100.0, armed=True)
    c.note_rc(100.0, monitored_valid=True, reset_pwm=1500)
    assert c.tick(101.5).engaged            # first trip fires the one-shot
    c.note_fcu(101.6, armed=True)
    assert not c.tick(101.7).engaged        # subsequent ticks do not re-fire it
