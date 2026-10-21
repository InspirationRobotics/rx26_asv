"""rc_heartbeat_core: the force-disarm-on-RC-loss state machine behind rc_watchdog. No ROS.

The latch clears only on a deliberate LOW->HIGH toggle of the reset channel seen with valid
PWM, so reconnecting with the switch already HIGH (or the 0 PWM of a lost link) never clears it.
"""
from dataclasses import dataclass


@dataclass
class WatchdogConfig:
    """Static configuration for the watchdog machine (all node params are [RO])."""
    heartbeat_timeout: float   # s of stale monitored-channel PWM before RC link declared lost
    min_valid_pwm: int         # us; below this = no pulses (RC loss reads 0)
    require_armed: bool        # only force-disarm when the vehicle is armed
    link_timeout: float        # s without any bridge topic before the gateway is deemed down
    enable_latch: bool         # hold the kill until an RC reset toggle clears it
    reset_low: int             # us; reset switch "low" at/below this
    reset_high: int            # us; reset switch "high" at/above this


@dataclass
class Decision:
    """What the node should do after a tick. Pure data — no ROS, no logging."""
    disarm_request: bool = False       # request a force-disarm this tick
    kill_active: bool = False          # watchdog kill state after this tick
    latch_cleared: bool = False        # the RC reset toggle cleared the latch this tick
    engaged: bool = False              # the latched kill newly engaged this tick (one-shot log)
    gateway_down_in_loss: bool = False # RC lost AND bridge down: cannot disarm via the gateway


class RcHeartbeatCore:
    """Force-disarm-on-RC-loss state machine.

    Drive it by feeding observations (`note_rc`, `note_fcu`) as topics arrive and
    calling `tick(now)` at the control rate. `now` is a monotonic seconds value
    supplied by the caller so tests can drive time deterministically.
    """

    def __init__(self, cfg: WatchdogConfig, now: float):
        self.cfg = cfg
        self.killed = False
        self.reset_armed = False   # saw a valid LOW since the latch engaged
        self.reset_pwm = 0         # latest reset-channel raw PWM
        self.armed = False
        self.last_rc_ok = now
        self.last_bridge_msg = 0.0

    # ---------------- observations ----------------
    def note_rc(self, now: float, monitored_valid: bool, reset_pwm: int):
        """A republished RcChannels arrived. `monitored_valid` = the heartbeat
        channel carries a valid PWM this message; `reset_pwm` = raw reset-channel PWM."""
        self.last_bridge_msg = now
        if monitored_valid:
            self.last_rc_ok = now
        self.reset_pwm = reset_pwm

    def note_fcu(self, now: float, armed: bool):
        """A republished FcuStatus arrived."""
        self.last_bridge_msg = now
        self.armed = armed

    # ---------------- helpers ----------------
    def _bridge_ok(self, now: float) -> bool:
        return (now - self.last_bridge_msg) < self.cfg.link_timeout

    def _link_lost(self, now: float) -> bool:
        return (now - self.last_rc_ok) > self.cfg.heartbeat_timeout

    def _poll_latch_reset(self) -> bool:
        """Return True iff a deliberate LOW->HIGH reset toggle cleared the latch.

        A valid LOW must be seen first (`reset_armed`) before a HIGH clears the
        latch, so reconnecting with the switch already HIGH -- or the 0 PWM that
        appears during link loss -- can never auto-clear the kill.
        """
        pwm = self.reset_pwm
        if pwm < self.cfg.min_valid_pwm:
            return False  # no valid RC signal on this channel; not a real toggle
        if pwm <= self.cfg.reset_low:
            self.reset_armed = True
        elif pwm >= self.cfg.reset_high and self.reset_armed:
            self.reset_armed = False
            self.killed = False
            return True
        return False

    # ---------------- the machine ----------------
    def tick(self, now: float) -> Decision:
        d = Decision()
        bridge_ok = self._bridge_ok(now)
        link_lost = self._link_lost(now)

        if self.killed:
            # Latched: look for the operator's RC reset toggle, otherwise keep
            # enforcing the kill.
            if self._poll_latch_reset():
                d.latch_cleared = True
                # latch just cleared; fall through and evaluate normally this tick
            else:
                if self.armed and bridge_ok:
                    d.disarm_request = True
                d.kill_active = True
                return d

        if not link_lost:
            d.kill_active = False
            return d

        # RC link is down.
        if self.cfg.require_armed and not self.armed:
            # Nothing to stop, and we do not want to latch a kill on a boat that
            # is already safe on the bench.
            d.kill_active = False
            return d

        if not bridge_ok:
            # Lost telemetry_bridge too: cannot route a disarm through the gateway;
            # ArduPilot's onboard failsafe must handle this case.
            d.gateway_down_in_loss = True
            return d

        if self.cfg.enable_latch:
            if not self.killed:
                self.reset_armed = False   # require a fresh LOW->HIGH after kill
                self.killed = True
                d.engaged = True
        d.disarm_request = True
        d.kill_active = True
        return d
