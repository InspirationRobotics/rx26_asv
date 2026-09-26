"""steady_core — is the boat still enough to shoot?

The nozzle has no pitch control, so the boat IS the aim, and this boat rocks.
At ~1 m from the face, one degree of pitch moves the hit about 2.4 cm and one
degree of roll about 1 cm sideways; a window is 21-29 cm. A shot taken mid-roll
tells you about the roll, not about where the boat should sit.

Three things have to hold for STEADY:
  * RATES   |roll rate| and |pitch rate| under rate_max over the last hold_s;
  * BAND    roll and pitch within band of their mean over the last mean_s,
            for the whole of the last hold_s (a slow list is fine; a swing is not);
  * STICKS  no stick has moved more than stick_tol for sticks_quiet_s. The pilot
            holding a steady bit of throttle against a current is fine; changing
            it is what sets the hull rocking.
and the attitude has to be arriving fast enough to judge any of it (min_hz):
at 4 Hz a 1 s roll period is barely sampled at all.

No ROS. Angles in DEGREES here (the page shows degrees and so do the logs);
the ROS adapter converts from the autopilot's radians. Time is a caller-supplied
monotonic seconds value, so tests drive it.
"""
from collections import deque

DEFAULTS = dict(
    rate_max_dps=4.0,        # deg/s, roll and pitch
    band_deg=1.5,            # deg either side of the running mean
    hold_s=0.5,              # all of the above held for this long
    mean_s=5.0,              # running-mean window
    sticks_quiet_s=1.5,      # s since the last stick change
    stick_tol_us=25,         # us; smaller changes are noise, not the pilot
    min_hz=10.0,             # attitude samples per second needed to judge it
    att_timeout_s=0.5,       # no attitude for this long = not steady
)


class SteadyMonitor:

    def __init__(self, **kw):
        unknown = set(kw) - set(DEFAULTS)
        if unknown:
            raise KeyError(f"unknown steadiness setting(s): {sorted(unknown)}")
        self.p = dict(DEFAULTS, **kw)
        self._att = deque()          # (t, roll, pitch, roll_rate, pitch_rate)
        self._stick_ref = None       # the stick values the last 'move' settled at
        self._last_move = None       # t of the last stick move; None = never seen
        self._steady_since = None

    # ---- inputs ----

    def feed_att(self, t, roll, pitch, roll_rate, pitch_rate):
        self._att.append((t, roll, pitch, roll_rate, pitch_rate))
        keep = max(self.p["mean_s"], 10.0)
        while self._att and self._att[0][0] < t - keep:
            self._att.popleft()

    def feed_sticks(self, t, sticks):
        """sticks: the pilot's channels 1-4 in microseconds (0 = no signal)."""
        sticks = [int(v) for v in sticks]
        if self._stick_ref is None:
            self._stick_ref, self._last_move = sticks, t
            return
        tol = self.p["stick_tol_us"]
        if any(abs(a - b) > tol for a, b in zip(sticks, self._stick_ref)):
            self._stick_ref, self._last_move = sticks, t

    # ---- queries ----

    def sticks_quiet_s(self, t):
        return None if self._last_move is None else t - self._last_move

    def att_hz(self, t, window=2.0):
        n = sum(1 for s in self._att if s[0] >= t - window)
        return n / window

    def latest(self):
        return self._att[-1] if self._att else None

    def peak_to_peak(self, t0, t1):
        """(roll_pp, pitch_pp) in degrees over [t0, t1], or None with no samples.
        Logged per shot: how much the boat moved while the water was in the air."""
        s = [a for a in self._att if t0 <= a[0] <= t1]
        if not s:
            return None
        rolls, pitches = [a[1] for a in s], [a[2] for a in s]
        return max(rolls) - min(rolls), max(pitches) - min(pitches)

    def status(self, t):
        """{steady, steady_for_s, reasons[], rate_max_dps, roll_dev, pitch_dev,
        sticks_quiet_s, att_hz}. `reasons` is empty exactly when steady."""
        p = self.p
        reasons = []
        hz = self.att_hz(t)
        last = self.latest()
        out = dict(steady=False, steady_for_s=0.0, reasons=reasons,
                   rate_max_dps=None, roll_dev=None, pitch_dev=None,
                   sticks_quiet_s=self.sticks_quiet_s(t), att_hz=round(hz, 1))
        if last is None or t - last[0] > p["att_timeout_s"]:
            reasons.append("no attitude")
        else:
            if hz < p["min_hz"]:
                reasons.append(f"attitude only {hz:.0f} Hz (need {p['min_hz']:.0f})")
            recent = [a for a in self._att if a[0] >= t - p["hold_s"]]
            window = [a for a in self._att if a[0] >= t - p["mean_s"]]
            mr = sum(a[1] for a in window) / len(window)
            mp = sum(a[2] for a in window) / len(window)
            rate = max(max(abs(a[3]), abs(a[4])) for a in recent)
            rdev = max(abs(a[1] - mr) for a in recent)
            pdev = max(abs(a[2] - mp) for a in recent)
            out.update(rate_max_dps=round(rate, 2), roll_dev=round(rdev, 2),
                       pitch_dev=round(pdev, 2))
            if rate > p["rate_max_dps"]:
                reasons.append(f"rocking {rate:.1f} deg/s")
            if pdev > p["band_deg"]:
                reasons.append(f"pitch swinging +-{pdev:.1f} deg")
            if rdev > p["band_deg"]:
                reasons.append(f"roll swinging +-{rdev:.1f} deg")
        quiet = out["sticks_quiet_s"]
        if quiet is None:
            reasons.append("no RC")
        elif quiet < p["sticks_quiet_s"]:
            reasons.append(f"hands off the sticks ({quiet:.1f} s)")
        if reasons:
            self._steady_since = None
        else:
            if self._steady_since is None:
                self._steady_since = t
            out["steady_for_s"] = round(t - self._steady_since, 2)
            out["steady"] = True
        return out
