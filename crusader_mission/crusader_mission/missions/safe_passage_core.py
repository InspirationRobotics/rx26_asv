"""safe_passage_core — Task 1's phase list. Pure Python, no rclpy.

Right now every phase is a DWELL: the mission walks
IDLE -> APPROACH -> OBSERVE -> ORBIT_ENTRY -> TRANSIT -> ORBIT_EXIT -> COMPLETE
on a timer and does nothing else. That is deliberate and it is not a stub for
its own sake — an empty mission still exercises the whole workflow that has to
work before any of the real logic is worth writing:

  goal accepted -> feedback streams -> cancel returns promptly -> result lands
  -> the tree moves on

and it exercises the terminating behaviour (mode loss, timeout, cancel) through
exactly the same code path the real mission will use, because the decision to
stop lives in phase_sequencer, not here.

FILLING IT IN. Each phase becomes real by giving its PhaseSpec a `done`
predicate. `done(world, phase_elapsed_s)` receives whatever the node passes as
`world` — a snapshot object, never live handles, so the predicate stays
testable. Nothing else about this file changes: the phase order, the weights,
the deadlines and the outcome-on-deadline are already the mission's shape.

  APPROACH     done when within approach_radius_m of the goal point OR the
               entry gate resolves — whichever comes first. The team-provided
               waypoint is a hint about where to look, not a place to touch.
  OBSERVE      done when ENTRY resolves. on_deadline is OUTCOME_NO_ENTRY, which
               is why running out of time here is a reported failure and not a
               quiet advance into a transit with nothing to transit.
  ORBIT_*      done at the last orbit waypoint.
  TRANSIT      done when the last gate is cleared.
"""
from .phase_sequencer import ADVANCE, OUTCOME_NO_ENTRY, PhaseSpec

# Phase names are part of the action's FEEDBACK contract (the `phase` string),
# so a watcher, the ground station and the logs all read the same words. Rename
# one and you have renamed a wire value.
IDLE = "IDLE"
APPROACH = "APPROACH"
OBSERVE = "OBSERVE"
ORBIT_ENTRY = "ORBIT_ENTRY"
TRANSIT = "TRANSIT"
ORBIT_EXIT = "ORBIT_EXIT"
COMPLETE = "COMPLETE"

# Weights are how much of the progress bar each phase owns. They are guesses
# about DURATION, not importance, and they only affect what an operator sees.
# TRANSIT is the long one; IDLE and COMPLETE are bookends worth almost nothing.
_WEIGHTS = {IDLE: 0.0, APPROACH: 3.0, OBSERVE: 2.0, ORBIT_ENTRY: 2.0,
            TRANSIT: 5.0, ORBIT_EXIT: 2.0, COMPLETE: 0.0}

# What running out of time in each phase MEANS. Only OBSERVE is fatal: every
# other phase can be cut short and still leave a partly-scored run, while an
# OBSERVE that never resolves means there is no passage to attempt.
_ON_DEADLINE = {OBSERVE: OUTCOME_NO_ENTRY}

ORDER = (IDLE, APPROACH, OBSERVE, ORBIT_ENTRY, TRANSIT, ORBIT_EXIT, COMPLETE)


def build_phases(dwell_s, *, predicates=None):
    """The Task 1 phase list.

    dwell_s: {phase_name: seconds}. For the empty mission this is the whole
      behaviour — how long to sit in each phase. For the real mission it becomes
      the per-phase DEADLINE and the predicates end the phases early.
    predicates: {phase_name: done_fn} for phases that have real work yet. A
      phase with no predicate stays a dwell, so the mission can be filled in one
      phase at a time and still run end to end at every step.

    Raises on an unknown phase name rather than ignoring it — a typo'd key in a
    params file would otherwise silently leave that phase at its default and
    look like the parameter had no effect.
    """
    predicates = predicates or {}
    unknown = (set(dwell_s) | set(predicates)) - set(ORDER)
    if unknown:
        raise ValueError("unknown phase name(s): {} — valid: {}".format(
            sorted(unknown), list(ORDER)))
    return [PhaseSpec(name,
                      weight=_WEIGHTS[name],
                      deadline_s=max(float(dwell_s.get(name, 1.0)), 1e-3),
                      on_deadline=_ON_DEADLINE.get(name, ADVANCE),
                      done=predicates.get(name))
            for name in ORDER]
