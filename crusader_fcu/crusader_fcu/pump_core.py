"""pump_core — every rule about the water pump, with no ROS and no MAVLink.

telemetry_bridge is the only thing that talks to the Pixhawk, so the pump's
rules live where the command leaves: anything upstream (squirt_cal, the Task 3
tree, `ros2 topic pub`) can only ASK on /crsd/pump_cmd.

HOW A BURST IS SENT. The pump is on a Pixhawk output that passes the pilot's
pump channel through (SERVOn_FUNCTION = RCINx). MAV_CMD_DO_REPEAT_SERVO with ONE
cycle drives that output to ON for half the cycle time and then back to the
output's TRIM, on the autopilot's own clock. So a burst ends even if the bridge,
the laptop, the WiFi or MAVProxy dies halfway through it — which only works if
SERVOn_TRIM IS THE PUMP'S OFF VALUE. check_config and the watchdog below both
hold that line. (From ArduPilot's AP_ServoRelayEvents as remembered, not read:
docs/G7_pump_bench.md verifies it on the bench before this is trusted.)

WHY THE BRIDGE REFUSES SO MUCH. The e-stop (SB, RC7_OPTION 165) stops the
MOTORS; a pass-through output is not a motor, and disarming does not stop it
either. So nothing but these checks stands between a software request and water
coming out while someone is standing at the bow.

`now` is monotonic seconds from the caller; tests drive it.
"""
from dataclasses import dataclass

MAV_CMD_DO_SET_SERVO = 183
MAV_CMD_DO_REPEAT_SERVO = 211

# PumpState.RESULT_* (kept equal by test_pump_core)
RESULT_NONE, RESULT_SENT, RESULT_ACCEPTED, RESULT_REJECTED, RESULT_REFUSED = range(5)


@dataclass
class PumpParams:
    servo_channel: int        # Pixhawk output 1..16 driving the pump; 0 = no pump path
    rc_channel: int           # the pilot's pump switch (1-based)
    on_pwm: int               # PWM that runs the pump
    off_pwm: int              # PWM that stops it; MUST equal SERVOn_TRIM
    max_burst_s: float
    min_gap_s: float          # from the end of one burst to the start of the next
    allow_disarmed: bool      # bench only; the YAML keeps it false
    estop_channel: int        # SB
    estop_threshold: int      # below = e-stop (RC loss reads 0, which is below)
    return_timeout_s: float = 0.3   # output must be OFF this long after a burst

    @classmethod
    def from_dict(cls, d):
        """From the bridge's resolved ROS params. KeyError on a missing key."""
        return cls(servo_channel=int(d["pump_servo_channel"]),
                   rc_channel=int(d["pump_rc_channel"]),
                   on_pwm=int(d["pump_on_pwm"]), off_pwm=int(d["pump_off_pwm"]),
                   max_burst_s=float(d["pump_max_burst_s"]),
                   min_gap_s=float(d["pump_min_gap_s"]),
                   allow_disarmed=bool(d["pump_allow_disarmed"]),
                   estop_channel=int(d["estop_channel"]),
                   estop_threshold=int(d["estop_threshold"]))

    def is_on(self, pwm):
        """Nearer ON than OFF. Works whichever way round the pump's PWM runs."""
        if not pwm:
            return False
        return abs(pwm - self.on_pwm) < abs(pwm - self.off_pwm)


@dataclass
class PumpInputs:
    now: float
    rc: list = None           # the 18 RC channels, or None when stale
    armed: bool = None        # None when HEARTBEAT is stale
    output_pwm: int = None    # the pump output from SERVO_OUTPUT_RAW, None when stale


class PumpGate:
    """The pump path's state: the last burst, and whether the watchdog has
    latched the whole path off (only a bridge restart clears that, on purpose:
    a pump that did not stop is a thing to look at, not to retry)."""

    def __init__(self, p: PumpParams):
        self.p = p
        self.burst_until = None       # when the current/last burst should have ended
        self.latched = ""             # non-empty = path disabled, and why
        self._last_off_sent = None

    @property
    def enabled(self):
        return self.p.servo_channel > 0 and not self.latched

    # ---- requests ----

    def check_burst(self, duration, inp: PumpInputs):
        """(ok, reason). `duration` > 0; OFF requests never come here."""
        p = self.p
        if p.servo_channel <= 0:
            return False, "no pump path (pump_servo_channel is 0)"
        if self.latched:
            return False, f"pump path latched off: {self.latched} (restart the bridge)"
        if not 0.0 < duration <= p.max_burst_s:
            return False, f"burst {duration:.2f} s outside (0, {p.max_burst_s:.2f}] s"
        if inp.rc is None:
            return False, "RC not fresh"
        estop = rc_value(inp.rc, p.estop_channel)
        if estop < p.estop_threshold:
            return False, f"SB e-stop engaged or RC lost (ch{p.estop_channel}={estop})"
        pilot = rc_value(inp.rc, p.rc_channel)
        if p.is_on(pilot):
            return False, f"the pilot's pump switch is ON (ch{p.rc_channel}={pilot})"
        if inp.armed is None:
            return False, "armed state unknown (no HEARTBEAT)"
        if not inp.armed and not p.allow_disarmed:
            return False, "disarmed"
        if inp.output_pwm is None:
            return False, "pump output not reported (SERVO_OUTPUT_RAW stale)"
        if p.is_on(inp.output_pwm):
            return False, f"pump output already ON ({inp.output_pwm})"
        if self.burst_until is not None:
            if inp.now < self.burst_until:
                return False, "a burst is still running"
            if inp.now - self.burst_until < p.min_gap_s:
                return False, f"too soon ({p.min_gap_s:.1f} s between bursts)"
        return True, "ok"

    def start_burst(self, duration, now):
        self.burst_until = now + duration

    # ---- the watchdog, every tick ----

    def watchdog(self, inp: PumpInputs):
        """Reason to send OFF now, or "". Rate-limited to one OFF a second.

        * the output is ON while SB is in e-stop, or RC is lost: the e-stop
          does not stop a pass-through output, so this does;
        * the output is still ON return_timeout_s after one of OUR bursts should
          have ended while the pilot's switch is OFF: the autopilot did not
          return it (TRIM is not OFF?). This one also LATCHES the path off.
        """
        p = self.p
        if p.servo_channel <= 0 or inp.output_pwm is None or not p.is_on(inp.output_pwm):
            return ""
        reason = ""
        if inp.rc is None:
            reason = "pump ON with RC lost"
        elif rc_value(inp.rc, p.estop_channel) < p.estop_threshold:
            reason = "pump ON with SB e-stop engaged"
        elif (self.burst_until is not None
              and inp.now > self.burst_until + p.return_timeout_s
              and inp.now < self.burst_until + p.return_timeout_s + 5.0
              and not p.is_on(rc_value(inp.rc, p.rc_channel))):
            reason = (f"pump still ON {inp.now - self.burst_until:.1f} s after the "
                      f"burst ended: is SERVO{p.servo_channel}_TRIM the OFF value "
                      f"({p.off_pwm})?")
            self.latched = reason
        if not reason:
            return ""
        if self._last_off_sent is not None and inp.now - self._last_off_sent < 1.0:
            return ""
        self._last_off_sent = inp.now
        return reason


def rc_value(rc, ch):
    """1-based channel from an RC list; 0 if absent."""
    i = ch - 1
    return int(rc[i]) if 0 <= i < len(rc) and rc[i] else 0


def repeat_servo_params(channel, pwm, duration):
    """COMMAND_LONG params 1-7 for one burst: ON for `duration`, then TRIM.
    param3 (cycles) is pinned to 1: a negative count repeats forever."""
    return (float(channel), float(pwm), 1.0, 2.0 * float(duration), 0.0, 0.0, 0.0)


def set_servo_params(channel, pwm):
    return (float(channel), float(pwm), 0.0, 0.0, 0.0, 0.0, 0.0)


def servo_raw(msg, channel):
    """The channel's PWM out of a SERVO_OUTPUT_RAW message (port 0 = outputs
    1-16; 9-16 are MAVLink 2 extension fields, absent = 0)."""
    if getattr(msg, "port", 0) != 0 or not 1 <= channel <= 16:
        return None
    return int(getattr(msg, f"servo{channel}_raw", 0) or 0)
