"""phase_sequencer — the part of a mission that decides WHEN to stop.

Pure Python. No rclpy, no ROS types, no clock of its own: every method takes
`now` in monotonic seconds. That is what lets the mission's termination logic —
the half that decides whether the boat is still allowed to be driving — be
tested at a desk against a fake clock, in milliseconds, with no boat.

WHY THIS IS A SEPARATE THING FROM THE MISSION
---------------------------------------------
Every mission on this vehicle has the same three ways to end badly and one way
to end well, and they are all more dangerous than the mission logic itself:

  * the pilot takes the mode away          -> stop, do not fight for it
  * the operator cancels                   -> stop promptly, from ANY phase
  * the clock runs out                     -> stop, report what we got

Written per-mission, one of those gets forgotten and the boat sits there with
the behaviour tree blocked behind it. Written once, "every action terminates" is
a property of this file, and a new mission gets it by construction.

THE ONE RULE THIS FILE ENFORCES: there is no path through poll() that returns
"still running" forever. Either a phase advances, or a deadline fires. A goal
with no timeout is not representable — timeout_s has no sentinel meaning
"never".
"""

# Outcome codes. These MUST match crusader_msgs/action/SafePassage.action; the
# sequencer deliberately does not import the generated message, so it can run
# with no ROS installed at all. bench_safe_passage.py asserts the two agree.
OUTCOME_SUCCESS = 0
OUTCOME_TIMEOUT = 1
OUTCOME_NO_ENTRY = 2
OUTCOME_CANCELLED = 3
OUTCOME_NOT_AUTONOMOUS = 4
OUTCOME_FAULT = 5

OUTCOME_NAMES = {
    OUTCOME_SUCCESS: "SUCCESS",
    OUTCOME_TIMEOUT: "TIMEOUT",
    OUTCOME_NO_ENTRY: "NO_ENTRY",
    OUTCOME_CANCELLED: "CANCELLED",
    OUTCOME_NOT_AUTONOMOUS: "NOT_AUTONOMOUS",
    OUTCOME_FAULT: "FAULT",
}

ADVANCE = "advance"      # a phase deadline that just moves on, not a failure


class PhaseSpec:
    """One phase: what it is called, how long it may take, how it ends.

    deadline_s is a MAXIMUM, never a target. A phase with a `done` predicate
    ends the moment the predicate is true; deadline_s is only what happens when
    it never becomes true.

    on_deadline is either ADVANCE (the phase was a dwell, or its timeout is not
    fatal — carry on) or an outcome code (running out of time HERE ends the
    mission, e.g. OBSERVE -> OUTCOME_NO_ENTRY). Making that an explicit field is
    the point: a silent "advance anyway" on a phase that genuinely failed is how
    a mission comes to report SUCCESS for a run that did nothing.

    done(world, phase_elapsed_s) returns True when the phase's real work is
    finished. None means "this phase is a dwell" — which is exactly the empty
    mission, and is why the empty mission exercises the same code path the real
    one will.
    """

    __slots__ = ("name", "weight", "deadline_s", "on_deadline", "done")

    def __init__(self, name, *, weight=1.0, deadline_s=5.0,
                 on_deadline=ADVANCE, done=None):
        if deadline_s <= 0:
            raise ValueError(
                name + ": deadline_s must be positive (no phase runs forever)")
        if weight < 0:
            raise ValueError(name + ": weight must not be negative")
        self.name = name
        self.weight = float(weight)
        self.deadline_s = float(deadline_s)
        self.on_deadline = on_deadline
        self.done = done

    def __repr__(self):
        return "PhaseSpec({!r}, deadline_s={})".format(self.name,
                                                       self.deadline_s)


class Verdict:
    """What poll() decided. `outcome is None` means the mission continues."""

    __slots__ = ("outcome", "detail", "phase", "progress", "phase_changed")

    def __init__(self, outcome, detail, phase, progress, phase_changed=False):
        self.outcome = outcome
        self.detail = detail
        self.phase = phase
        self.progress = progress
        self.phase_changed = phase_changed

    @property
    def running(self):
        return self.outcome is None

    def __repr__(self):
        o = "RUNNING" if self.running else OUTCOME_NAMES.get(self.outcome,
                                                             self.outcome)
        return "<Verdict {} phase={} p={:.2f}>".format(o, self.phase,
                                                       self.progress)


class PhaseSequencer:
    """Walks a list of PhaseSpec, and terminates. Always terminates.

    Usage, once per goal:

        seq = PhaseSequencer(phases, timeout_s=120.0)
        seq.start(now)
        while True:
            v = seq.poll(now, autonomous=..., cancel_requested=..., world=...)
            if not v.running:
                break

    `autonomous` is tri-state on purpose: True in an autonomous mode, False in a
    manual one, and **None when we do not know** — no HEARTBEAT, or none yet.
    None is not False: at goal start it means the precondition has not arrived
    yet (FAULT after a grace period), and mid-mission it means the link died,
    which is also not a reason to claim the pilot took over. A boolean here
    would collapse "the pilot has the boat" and "we cannot see the boat" into
    one report, and those need different words in the log.
    """

    def __init__(self, phases, *, timeout_s, mode_grace_s=3.0):
        if not phases:
            raise ValueError("a mission needs at least one phase")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive; a goal that cannot "
                             "time out can leave the boat sitting there")
        self.phases = list(phases)
        self.timeout_s = float(timeout_s)
        self.mode_grace_s = float(mode_grace_s)
        self._total_weight = sum(p.weight for p in self.phases) or 1.0
        self._i = 0
        self._t0 = None
        self._phase_t0 = None
        self._max_progress = 0.0

    # ---- lifecycle ----

    def start(self, now):
        self._i = 0
        self._t0 = now
        self._phase_t0 = now
        self._max_progress = 0.0

    @property
    def phase(self):
        return self.phases[min(self._i, len(self.phases) - 1)].name

    def elapsed(self, now):
        return 0.0 if self._t0 is None else now - self._t0

    # ---- the decision ----

    def poll(self, now, *, autonomous, cancel_requested=False, world=None):
        if self._t0 is None:
            raise RuntimeError("poll() before start()")

        # Order matters, and it is the safety posture. Cancel outranks
        # everything, because a cancel that waits for a phase boundary is a
        # cancel that does not work. The mode check outranks the clock because
        # "the pilot has the boat" is the more useful sentence in the log.
        if cancel_requested:
            return self._stop(now, OUTCOME_CANCELLED,
                              "cancelled during " + self.phase)

        if autonomous is False:
            return self._stop(now, OUTCOME_NOT_AUTONOMOUS,
                              "flight mode left the autonomous set — the pilot "
                              "took control; not fighting for it")
        if autonomous is None:
            # Unknown, not manual. Tolerated briefly at the start of a goal
            # (HEARTBEAT may not have landed yet) and never for long.
            if self.elapsed(now) > self.mode_grace_s:
                return self._stop(
                    now, OUTCOME_FAULT,
                    "flight mode unknown for {:.1f}s — no HEARTBEAT, so we "
                    "cannot say whether we are allowed to drive".format(
                        self.elapsed(now)))
            return self._running(now, "waiting for HEARTBEAT")

        if self.elapsed(now) >= self.timeout_s:
            return self._stop(now, OUTCOME_TIMEOUT,
                              "mission timeout {:.0f}s expired in {}".format(
                                  self.timeout_s, self.phase))

        spec = self.phases[self._i]
        phase_elapsed = now - self._phase_t0
        overdue = phase_elapsed >= spec.deadline_s

        # A dwell phase has no predicate, so its deadline is what paces it. A
        # working phase ends on its predicate. Both funnel through _advance.
        if spec.done is None:
            return self._advance(now) if overdue else self._running(now)

        if spec.done(world, phase_elapsed):
            return self._advance(now)
        if overdue:
            if spec.on_deadline is ADVANCE:
                return self._advance(now, warning=(
                    "{} hit its {:.0f}s deadline without finishing; "
                    "continuing anyway".format(spec.name, spec.deadline_s)))
            return self._stop(now, spec.on_deadline,
                              "{} did not complete within {:.0f}s".format(
                                  spec.name, spec.deadline_s))
        return self._running(now)

    # ---- internals ----

    def _advance(self, now, warning=""):
        self._i += 1
        self._phase_t0 = now
        if self._i >= len(self.phases):
            self._i = len(self.phases) - 1
            return self._stop(now, OUTCOME_SUCCESS, "all phases complete",
                              progress=1.0)
        v = self._running(now, warning)
        v.phase_changed = True
        return v

    def _progress(self, now):
        """Weighted, and MONOTONIC — it never goes backwards.

        A progress bar that retreats reads as a bug to whoever is watching, and
        the one place it would retreat (a phase advancing before its deadline)
        is exactly when they most need to trust the display.
        """
        done_w = sum(p.weight for p in self.phases[:self._i])
        spec = self.phases[self._i]
        frac = 0.0
        if spec.deadline_s > 0 and self._phase_t0 is not None:
            frac = min(1.0, (now - self._phase_t0) / spec.deadline_s)
        p = (done_w + spec.weight * frac) / self._total_weight
        self._max_progress = max(self._max_progress, min(1.0, p))
        return self._max_progress

    def _running(self, now, detail=""):
        return Verdict(None, detail, self.phase, self._progress(now))

    def _stop(self, now, outcome, detail, progress=None):
        p = self._progress(now) if progress is None else progress
        return Verdict(outcome, detail, self.phase, p)
