"""override_core — the rules for RC overrides (software on the sticks), no ROS, no MAVLink.

The fixed-nozzle shot needs to STRAFE, and on this OmniX hull only MANUAL mode
strafes: GUIDED turns and drives, it does not slide. So in MANUAL the tree moves
the boat the way the team's dp_hold did - RC_CHANNELS_OVERRIDE on the steering,
throttle and lateral sticks - and telemetry_bridge is still the only thing that
sends it. These are its rules for /crsd/rc_override:

  * MODE: forwarded only in one of `modes` (MANUAL). The pilot's SC switch is
    the take-back: flip it and nothing of ours reaches the motors. ArduRover
    applies overrides in any mode, so this check is ours to make.
  * LATCH: the autonomy-drop latch must be clear (as before).
  * CHANNELS: only `channels` (1 steer, 3 throttle, 4 lateral) may be
    overridden; every other channel is sent as 0 = "the pilot's own". SB (the
    e-stop, ch7), SC (the mode, ch8) and the pump can never be taken over, so
    the pilot keeps all three whatever the software asks.
  * CLAMP: each overridden channel is held within +-max_us of 1500.
  * DEAD-MAN: commands stop for deadman_s -> ONE release (all zeros), which
    hands the sticks straight back. ArduPilot's own RC_OVERRIDE_TIME (3 s on
    the boat) is the backstop; 3 s at 0.3 m/s is 0.9 m next to a dock.
  * The mode leaving `modes`, or the latch tripping, releases at once.

A value of 0 in RC_CHANNELS_OVERRIDE means "release this channel to RC";
that is what "release" means everywhere here.

`now` is monotonic seconds from the caller; tests drive it.
"""
from dataclasses import dataclass, field

NEUTRAL_US = 1500
N_SENT = 8                 # RC_CHANNELS_OVERRIDE as the bridge sends it: ch1..8


@dataclass
class OverrideParams:
    channels: tuple = (1, 3, 4)          # the only channels software may take
    max_us: int = 150                    # deflection cap about 1500
    deadman_s: float = 0.5
    modes: tuple = ("MANUAL",)

    @classmethod
    def from_dict(cls, d):
        """From the bridge's resolved ROS params. KeyError on a missing key."""
        return cls(channels=tuple(int(c) for c in d["override_channels"]),
                   max_us=int(d["override_max_us"]),
                   deadman_s=float(d["override_deadman_s"]),
                   modes=tuple(str(m).upper() for m in d["override_modes"]))


def release():
    return [0] * N_SENT


class OverrideGate:
    """Whether software holds the sticks, and when it last said so."""

    def __init__(self, p: OverrideParams):
        self.p = p
        self.active = False      # we have sent a non-release override not yet released
        self.last_cmd_t = None

    def filter(self, channels):
        """The 8 channels to send: ours clamped, everyone else's released."""
        out = release()
        lo, hi = NEUTRAL_US - self.p.max_us, NEUTRAL_US + self.p.max_us
        for ch in self.p.channels:
            i = ch - 1
            if 0 <= i < N_SENT and i < len(channels) and channels[i]:
                out[i] = max(lo, min(hi, int(channels[i])))
        return out

    def on_command(self, now, channels, mode, latch_allowed):
        """(ok, reason, ch8). ch8 = what to send, or None to send nothing."""
        mode = str(mode).upper()
        if not latch_allowed:
            return self._refuse("autonomy-drop latch tripped")
        if mode not in self.p.modes:
            return self._refuse(f"mode {mode} is not one of {'/'.join(self.p.modes)}")
        out = self.filter(channels)
        if not any(out):                 # an explicit release
            was = self.active
            self.active = False
            return True, "release", out if was else None
        self.active = True
        self.last_cmd_t = now
        clamped = [c for c in self.p.channels
                   if c - 1 < len(channels) and channels[c - 1] and out[c - 1] != channels[c - 1]]
        return True, ("ok" if not clamped else f"clamped ch{clamped} to +-{self.p.max_us} us"), out

    def _refuse(self, why):
        # Refused while we held the sticks: release them now rather than leave
        # the last override standing until RC_OVERRIDE_TIME.
        if self.active:
            self.active = False
            return False, why, release()
        return False, why, None

    def tick(self, now, mode):
        """ONE release when the dead-man expires or the mode leaves, else None."""
        if not self.active:
            return None
        if str(mode).upper() not in self.p.modes or now - self.last_cmd_t > self.p.deadman_s:
            self.active = False
            return release()
        return None

    def on_trip(self):
        """The drop latch tripped: a release if we held the sticks, else None."""
        if not self.active:
            return None
        self.active = False
        return release()
