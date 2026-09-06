#!/usr/bin/env python3
"""bench_mission — prove the mission's TERMINATION logic without a boat.

    python3 tools/bench/bench_mission.py --selftest     # exits nonzero on failure
    python3 tools/bench/bench_mission.py --trace        # walk one nominal run

Stdlib only, no ROS, no rclpy — so it runs in CI beside check_config.py and on a
laptop with nothing installed.

WHY THIS EXISTS. The mission logic decides where the boat goes; the termination
logic decides whether it is allowed to be going anywhere at all, and that is the
half that strands a boat. Every case here is one that is expensive or impossible
to stage on the water:

  * the pilot takes the mode away MID-MISSION
  * a cancel arrives during each phase in turn
  * HEARTBEAT never arrives, so the mode is UNKNOWN rather than manual
  * the clock expires in a phase that was not going to finish anyway

A fake clock makes all of them a few microseconds each, and deterministic —
`time.sleep` in a test is a test that is slow AND flaky.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "crusader_mission"))
from crusader_mission.missions import phase_sequencer as ps      # noqa: E402
from crusader_mission.missions import safe_passage_core as spc   # noqa: E402

DWELL = {n: 2.0 for n in spc.ORDER}


def drive(seq, *, autonomous=True, cancel_at=None, dt=0.1, mode_at=None,
          max_ticks=100000, collect=None):
    """Tick a started sequencer to termination. Returns (verdict, t, collected).

    The loop, not the sequencer, is what would hang if poll() ever failed to
    terminate — so max_ticks raises rather than spinning, and the message says
    so. Every test in this file goes through here; a second hand-written loop
    would be a second place for that guard to be missing.
    """
    t = 0.0
    got = []
    for _ in range(max_ticks):
        auto = autonomous
        if mode_at is not None and t >= mode_at[0]:
            auto = mode_at[1]
        v = seq.poll(t, autonomous=auto,
                     cancel_requested=cancel_at is not None and t >= cancel_at)
        if collect is not None:
            got.append(collect(v))
        if not v.running:
            return v, t, got
        t += dt
    raise AssertionError("sequencer never terminated in {} ticks — that is THE "
                         "bug this file exists to catch".format(max_ticks))


def run(*, autonomous=True, cancel_at=None, timeout_s=1000.0, dt=0.1,
        max_ticks=100000, mode_at=None, dwell=None):
    """Drive a sequencer on a fake clock. Returns (verdict, t, phases_seen).

    autonomous: the value poll() sees, unless mode_at overrides it later.
    cancel_at:  fake-clock time at which the operator cancels.
    mode_at:    (t, value) — the mode changes to `value` at time t.
    """
    seq = ps.PhaseSequencer(spc.build_phases(dwell or DWELL),
                            timeout_s=timeout_s)
    seq.start(0.0)
    v, t, seen = drive(seq, autonomous=autonomous, cancel_at=cancel_at, dt=dt,
                       mode_at=mode_at, max_ticks=max_ticks,
                       collect=lambda x: x.phase)
    phases = [p for i, p in enumerate(seen) if i == 0 or p != seen[i - 1]]
    return v, t, phases


def with_predicate(predicates, collect=None):
    """Run the mission with real `done` predicates on some phases.

    This is the seam the real mission fills in — a phase stops being a dwell the
    moment it gets one — so the tests that matter most exercise it rather than
    the dwell path.
    """
    seq = ps.PhaseSequencer(spc.build_phases(DWELL, predicates=predicates),
                            timeout_s=1000.0)
    seq.start(0.0)
    return drive(seq, collect=collect)


def selftest():
    fails = []

    def chk(name, got, want):
        ok = got == want
        if not ok:
            fails.append("{}: got {!r} want {!r}".format(name, got, want))
        print("  [{}] {}".format("ok" if ok else "FAIL", name))

    # --- the outcome codes must match the .action file, or the result the
    # --- caller decodes means something other than what we set.
    action = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                          "..", "..", "crusader_msgs", "action",
                          "SafePassage.action")
    with open(action, encoding="utf-8") as f:
        text = f.read()
    for const, value in (("OUTCOME_SUCCESS", ps.OUTCOME_SUCCESS),
                         ("OUTCOME_TIMEOUT", ps.OUTCOME_TIMEOUT),
                         ("OUTCOME_NO_ENTRY", ps.OUTCOME_NO_ENTRY),
                         ("OUTCOME_CANCELLED", ps.OUTCOME_CANCELLED),
                         ("OUTCOME_NOT_AUTONOMOUS", ps.OUTCOME_NOT_AUTONOMOUS),
                         ("OUTCOME_FAULT", ps.OUTCOME_FAULT)):
        chk("{} == {} in the .action".format(const, value),
            "uint8 {}={}".format(const, value) in text, True)

    # --- nominal ---
    v, t, phases = run()
    chk("a clean run succeeds", v.outcome, ps.OUTCOME_SUCCESS)
    chk("...visiting every phase in order", phases, list(spc.ORDER))
    chk("...ending at progress 1.0", v.progress, 1.0)

    # --- the pilot takes the boat, mid-mission ---
    v, t, _ = run(mode_at=(5.0, False))
    chk("mode loss ends the goal", v.outcome, ps.OUTCOME_NOT_AUTONOMOUS)
    chk("...promptly (< 1 tick after)", t <= 5.1, True)

    # --- cancel, from EVERY phase in turn ---
    for i, name in enumerate(spc.ORDER):
        at = i * 2.0 + 0.5                       # mid-phase, dwell is 2.0
        v, t, _ = run(cancel_at=at)
        chk("cancel during {} returns CANCELLED".format(name),
            v.outcome, ps.OUTCOME_CANCELLED)
        chk("cancel during {} returns within a tick".format(name),
            t - at <= 0.1 + 1e-9, True)

    # --- unknown mode: not manual, and not tolerated forever ---
    v, t, _ = run(autonomous=None)
    chk("unknown mode is a FAULT, not NOT_AUTONOMOUS", v.outcome,
        ps.OUTCOME_FAULT)
    chk("...tolerated for the grace period first", t > 3.0, True)
    v, _, _ = run(autonomous=None, mode_at=(1.0, True))
    chk("a late HEARTBEAT rescues it", v.outcome, ps.OUTCOME_SUCCESS)

    # --- the clock ---
    v, t, _ = run(timeout_s=5.0)
    chk("timeout fires", v.outcome, ps.OUTCOME_TIMEOUT)
    chk("...at the timeout, not later", t <= 5.1, True)

    # --- OBSERVE is the one fatal deadline (a phase with a predicate that
    # --- never becomes true must NOT quietly advance into a transit with
    # --- nothing to transit) ---
    v, _, _ = with_predicate({spc.OBSERVE: lambda w, e: False})
    chk("OBSERVE that never resolves reports NO_ENTRY", v.outcome,
        ps.OUTCOME_NO_ENTRY)

    # ...while a non-fatal phase deadline just carries on
    v, _, _ = with_predicate({spc.TRANSIT: lambda w, e: False})
    chk("TRANSIT that never resolves still completes the run", v.outcome,
        ps.OUTCOME_SUCCESS)

    # --- progress never goes backwards, even when a phase ends early ---
    _, _, seen = with_predicate({spc.TRANSIT: lambda w, e: e > 0.05},
                                collect=lambda x: x.progress)
    chk("progress is monotonic",
        all(b >= a for a, b in zip(seen, seen[1:])), True)

    # --- a goal that cannot time out must not be constructible ---
    for bad, why in ((0.0, "zero"), (-1.0, "negative")):
        try:
            ps.PhaseSequencer(spc.build_phases(DWELL), timeout_s=bad)
            chk("timeout_s={} is refused".format(why), "accepted", "refused")
        except ValueError:
            chk("timeout_s={} is refused".format(why), "refused", "refused")

    # --- a typo'd phase name is refused rather than silently ignored ---
    try:
        spc.build_phases({"APPROCH": 3.0})
        chk("a typo'd phase name is refused", "accepted", "refused")
    except ValueError:
        chk("a typo'd phase name is refused", "refused", "refused")

    # --- the tree, structurally ---
    from crusader_mission.tree import mission_spec as tree
    chk("the shipped tree passes its own check", tree.check(tree.build()), [])
    unprotected = tree.Sequence("run", tree.Mission("m", "/a", "T"))
    chk("...and catches a mission that could end the run",
        len(tree.check(unprotected)) >= 1, True)

    # THE property the tree exists to have: one mission failing must not cost
    # the ones after it, because scoring is per task. Asserted by SIMULATING a
    # tick rather than by reading the structure, so a decorator that is present
    # but wired to the wrong child still fails this.
    reached = []
    result = tree.simulate(tree.build(), {"safe_passage": tree.FAILURE},
                           trace=reached)
    chk("a failing mission does NOT end the run", result, tree.SUCCESS)
    chk("...and it was retried before giving up",
        sum(1 for _d, n, _r in reached if n == "safe_passage"), 2)
    chk("a failing guard DOES stop the run",
        tree.simulate(tree.build(), {"is_autonomous": tree.FAILURE}),
        tree.FAILURE)
    # ...and the same simulation on a tree with no protection shows the
    # difference, so the check above is not passing for the wrong reason.
    two = tree.Sequence("run", tree.Mission("a", "/a", "T"),
                        tree.Mission("b", "/b", "T"))
    seen = []
    tree.simulate(two, {"a": tree.FAILURE}, trace=seen)
    chk("an unprotected tree loses the mission after the failure",
        any(n == "b" for _d, n, _r in seen), False)

    print()
    if fails:
        print("FAIL — {} check(s):".format(len(fails)))
        for f in fails:
            print("  " + f)
        return 1
    print("PASS")
    return 0


def trace():
    seq = ps.PhaseSequencer(spc.build_phases(DWELL), timeout_s=1000.0)
    t = 0.0
    seq.start(t)
    while True:
        v = seq.poll(t, autonomous=True)
        if v.phase_changed or not v.running:
            print("  t={:6.2f}s  {:12s} progress={:5.1%}  {}".format(
                t, v.phase, v.progress,
                "" if v.running else ps.OUTCOME_NAMES[v.outcome]))
        if not v.running:
            return 0
        t += 0.1


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--selftest", action="store_true")
    ap.add_argument("--trace", action="store_true")
    a = ap.parse_args()
    return trace() if a.trace else selftest()


if __name__ == "__main__":
    sys.exit(main())
