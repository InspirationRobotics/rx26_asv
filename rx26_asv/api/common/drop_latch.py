"""Autonomy-drop switch latch — the state machine behind the G1 safety gate.

Context (CLAUDE.md standing safety constraint): `dp_hold` overrides RC sticks, and
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
      - RC data stale for > stale_timeout (can't verify the pilot has a path in).
  * LATCHED: once dropped, stays dropped. reset() succeeds only when the switch is
    back in SAFE position AND data is fresh — and reset must be an explicit
    operator action (service call), never automatic.
  * This class contains NO ROS/MAVLink code so it is unit-testable everywhere;
    enforcement wiring lives in telemetry_bridge (the sole RC-override sender).

This is a software layer ABOVE the hardware e-stop (SB switch), never a
replacement for it.
"""
from enum import Enum


class DropState(Enum):
    STARTUP = "startup"      # no valid safe sample seen yet — overrides blocked
    ACTIVE = "active"        # overrides allowed
    DROPPED = "dropped"      # latched — overrides blocked until explicit reset


class DropLatch:
    def __init__(self, channel: int = 7, threshold: int = 1700,
                 invert: bool = False, stale_timeout: float = 1.0):
        """channel is 1-indexed (RC convention). threshold in us.
        invert=False: value >= threshold trips. invert=True: value <= threshold trips.
        """
        if not 1 <= channel <= 18:
            raise ValueError("channel must be 1..18")
        self.channel = channel
        self.threshold = threshold
        self.invert = invert
        self.stale_timeout = stale_timeout
        self.state = DropState.STARTUP
        self.trip_reason = None
        self._last_sample_t = None
        self._last_value = None

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
        # safe position, fresh data
        if self.state == DropState.STARTUP:
            self.state = DropState.ACTIVE
        return False

    def tick(self, t: float) -> bool:
        """Call periodically. Returns True if staleness newly tripped the latch."""
        if self.state != DropState.ACTIVE:
            return False
        if self._last_sample_t is None or t - self._last_sample_t > self.stale_timeout:
            return self._trip("RC data stale — cannot verify pilot control path")
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

    def _trip(self, reason: str) -> bool:
        newly = self.state != DropState.DROPPED
        self.state = DropState.DROPPED
        if newly:
            self.trip_reason = reason
        return newly
