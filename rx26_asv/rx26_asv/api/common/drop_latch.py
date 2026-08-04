"""Autonomy-drop switch latch — the state machine behind the G1 safety gate.

Context: `dp_hold` overrides RC sticks, and
flipping SC to manual does NOT regain control while an override is active. The
pilot's only recovery paths were Ctrl+C (needs WiFi — not a safety tool) or the
hardware e-stop. This latch closes that gap: a dedicated RC channel, read via the
Pixhawk (so it works at ELRS range, far beyond WiFi), software-latches autonomy
OFF and releases all RC overrides.

Design rules (deliberate, do not weaken):
  * FAIL-SAFE START: overrides are NOT allowed until the latch has seen a fresh,
    valid RC sample with the switch in the SAFE position. No data = no override.
  * TRIP conditions (any -> latched DROPPED):
      - switch channel crosses the drop threshold (pilot commanded drop),
      - channel value 0 (RC link lost / failsafe no-pulses),
      - RC data stale for > stale_timeout (can't verify the pilot has a path in),
      - ArduPilot's own SYS_STATUS RC-receiver health bit reports UNHEALTHY.
  * LATCHED: once dropped, stays dropped. reset() succeeds only when the switch is
    back in SAFE position AND data is fresh — and reset must be an explicit
    operator action (service call), never automatic.
  * This class contains NO ROS/MAVLink code so it is unit-testable everywhere;
    enforcement wiring lives in telemetry_bridge (the sole RC-override sender).

WHY THE HEALTH BIT IS NOT OPTIONAL (measured on Crusader, 2026-08-02): the
"channel value 0" trip above rests on ArduPilot reporting 0 PWM when the
transmitter dies. That is FALSE on this airframe. With the ELRS receiver's
failsafe set to "Last Position", ch7 held 1995 through a full transmitter
power-down — RC_CHANNELS kept arriving at 20 Hz, so neither the zero check nor
the staleness check could fire, and this latch would have kept overrides ENABLED
on a boat with no pilot. Whether a receiver zeroes its outputs on link loss is a
configuration choice on a separate device and can never be the sole basis for a
safety interlock. MAV_SYS_STATUS_SENSOR_RC_RECEIVER is ArduPilot's own failsafe
verdict and does not care what the receiver puts on the wire.

rc_heartbeat_watchdog and pixhawk_led_status_node were fixed for this in
commit 217cf36; this latch was not, and inherited the disproven premise.

The health verdict is STRICTLY ADDITIVE, exactly as in rc_heartbeat_watchdog:
unknown (the autopilot never advertised the bit) and stale both fall back to the
PWM/zero checks, so it can only ever ADD a trip, never suppress one.

This is a software layer ABOVE the hardware e-stop (SB switch), never a
replacement for it.
"""
from enum import Enum


class DropState(Enum):
    STARTUP = "startup"      # no valid safe sample seen yet — overrides blocked
    ACTIVE = "active"        # overrides allowed
    DROPPED = "dropped"      # latched — overrides blocked until explicit reset


class DropLatch:
    def __init__(self, channel: int = 9, threshold: int = 1700,
                 invert: bool = False, stale_timeout: float = 1.0,
                 health_timeout: float = 3.0):
        """channel is 1-indexed (RC convention). threshold in us.
        invert=False: value >= threshold trips. invert=True: value <= threshold trips.

        health_timeout is the window in which a SYS_STATUS RC-receiver verdict is
        still trusted; it should track telemetry_bridge's status_timeout_s, since
        SYS_STATUS is a SLOW stream (SR*_EXT_STAT, 2 Hz on Crusader) and judging
        it against the 1 s RC timeout would discard a usable safety signal on
        jitter alone.
        """
        if not 1 <= channel <= 18:
            raise ValueError("channel must be 1..18")
        self.channel = channel
        self.threshold = threshold
        self.invert = invert
        self.stale_timeout = stale_timeout
        self.health_timeout = health_timeout
        self.state = DropState.STARTUP
        self.trip_reason = None
        self._last_sample_t = None
        self._last_value = None
        self._health = None          # None = autopilot never advertised the bit
        self._health_t = None

    # ---- inputs ----

    def rc_sample(self, channels, t: float) -> bool:
        """Feed one RC_CHANNELS reading (list of us values, index 0 = channel 1).
        Returns True if this sample newly tripped the latch."""
        value = channels[self.channel - 1] if len(channels) >= self.channel else 0
        self._last_sample_t = t
        self._last_value = value

        if value == 0:
            return self._trip("RC link lost (channel value 0)")
        if self._is_drop_position(value):
            if self.state == DropState.ACTIVE:
                return self._trip(f"pilot commanded drop (ch{self.channel}={value})")
            if self.state == DropState.STARTUP:
                # switch already in drop position at boot: stay blocked, don't latch
                return False
            return False
        # Safe position, fresh data. Promotion out of STARTUP additionally
        # requires that the autopilot is not currently calling the receiver dead
        # — otherwise a receiver holding last position would enable overrides on
        # a link with no pilot behind it, which is the exact 2026-08-02 case.
        if self.state == DropState.STARTUP and not self._health_lost(t):
            self.state = DropState.ACTIVE
        return False

    def note_rc_health(self, healthy, t: float) -> bool:
        """Feed ArduPilot's SYS_STATUS RC-receiver health verdict.

        `healthy` is None when the autopilot does not advertise the bit at all —
        recorded as "no verdict", never as True. Returns True if this sample
        newly tripped the latch.
        """
        if healthy is None:
            return False              # absence is not a verdict; PWM path decides
        self._health = bool(healthy)
        self._health_t = t
        if not self._health and self.state == DropState.ACTIVE:
            return self._trip(
                "ArduPilot reports RC receiver UNHEALTHY (link lost)")
        # STARTUP: stay blocked without latching, so simply powering the
        # transmitter on clears it — no operator reset for a benign boot order.
        return False

    def tick(self, t: float) -> bool:
        """Call periodically. Returns True if this tick newly tripped the latch."""
        if self.state != DropState.ACTIVE:
            return False
        if self._last_sample_t is None or t - self._last_sample_t > self.stale_timeout:
            return self._trip("RC data stale — cannot verify pilot control path")
        # Continuously enforced, not just checked at transitions: ACTIVE must
        # never coexist with a fresh unhealthy verdict, whatever the arrival
        # order of RC_CHANNELS and SYS_STATUS.
        if self._health_lost(t):
            return self._trip(
                "ArduPilot reports RC receiver UNHEALTHY (link lost)")
        return False

    def reset(self, t: float):
        """Explicit operator reset. Returns (ok: bool, reason: str)."""
        if self.state != DropState.DROPPED:
            return True, "not dropped"
        if self._last_sample_t is None or t - self._last_sample_t > self.stale_timeout:
            return False, "refused: RC data stale"
        if self._last_value == 0:
            return False, "refused: RC link still lost"
        if self._is_drop_position(self._last_value):
            return False, f"refused: switch still in drop position (ch{self.channel}={self._last_value})"
        if self._health_lost(t):
            return False, "refused: ArduPilot reports RC receiver unhealthy"
        self.state = DropState.ACTIVE
        self.trip_reason = None
        return True, "reset — overrides re-enabled"

    # ---- queries ----

    @property
    def allowed(self) -> bool:
        """May RC overrides be forwarded right now?"""
        return self.state == DropState.ACTIVE

    @property
    def dropped(self) -> bool:
        return self.state == DropState.DROPPED

    # ---- internals ----

    def _is_drop_position(self, value: int) -> bool:
        return value <= self.threshold if self.invert else value >= self.threshold

    def _health_lost(self, t: float) -> bool:
        """True ONLY on a fresh, explicit unhealthy verdict.

        Unknown (bit never advertised) and stale both return False, so the
        PWM/zero/staleness checks remain the decider. This is what makes the
        signal strictly additive — it may only ever ADD a trip, never suppress
        one, and can never mask a detection the older checks would have made.
        """
        if self._health is None or self._health_t is None:
            return False
        if (t - self._health_t) > self.health_timeout:
            return False
        return not self._health

    def _trip(self, reason: str) -> bool:
        newly = self.state != DropState.DROPPED
        self.state = DropState.DROPPED
        if newly:
            self.trip_reason = reason
        return newly
