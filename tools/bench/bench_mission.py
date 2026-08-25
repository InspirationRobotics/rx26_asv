#!/usr/bin/env python3
"""bench_mission — the boat's autonomy rule, exercised with no ROS present.

    python3 tools/bench/bench_mission.py

mission_core decides whether the boat is driving itself, and that answer is what
the OCS relays to RoboCommand as our claim. It is deliberately importable with
no rclpy, so this runs on any laptop with a Python on it.

The case worth reading twice is the last one: bench_auto fabricates STATE_AUTO
for producing rc-test logs off the water, and it still must not be able to
override an RC kill.
"""
import os
import sys

sys.path.insert(0, os.path.join(
    os.path.dirname(os.path.abspath(__file__)), "..", "..", "crusader_behavior"))

from crusader_behavior.mission import mission_core as mc  # noqa: E402

AUTO, MANUAL, KILLED = mc.AUTO, mc.MANUAL, mc.KILLED


def main() -> int:
    r = []

    def check(name, got, want):
        r.append((name, got == want, "%s (wanted %s)" % (got, want)))

    print("state, from the autopilot mode")
    check("armed GUIDED -> AUTO", mc.robot_state("GUIDED", True)[0], AUTO)
    check("armed AUTO -> AUTO", mc.robot_state("AUTO", True)[0], AUTO)
    check("lowercase guided still AUTO", mc.robot_state("guided", True)[0], AUTO)
    check("armed MANUAL -> MANUAL", mc.robot_state("MANUAL", True)[0], MANUAL)
    check("armed HOLD -> MANUAL", mc.robot_state("HOLD", True)[0], MANUAL)
    check("disarmed in GUIDED -> MANUAL", mc.robot_state("GUIDED", False)[0], MANUAL)
    check("stale fcu_status -> no state", mc.robot_state(None, None)[0], None)
    check("never STATE_UNKNOWN",
          mc.robot_state("", False)[0] != "STATE_UNKNOWN", True)

    print("\nthe RC kill outranks everything")
    check("kill while GUIDED -> KILLED",
          mc.robot_state("GUIDED", True, killed=True)[0], KILLED)
    check("kill while stale -> KILLED",
          mc.robot_state(None, None, killed=True)[0], KILLED)

    print("\ntask")
    check("default task is NONE", mc.decide("GUIDED", True)[1], mc.TASK_NONE)
    check("a real task passes through",
          mc.decide("GUIDED", True, task="TASK_SAFE_PASSAGE")[1],
          "TASK_SAFE_PASSAGE")
    try:
        mc.decide("GUIDED", True, task="TASK_MADE_UP")
        r.append(("an invented task is refused", False, "no exception raised"))
    except ValueError:
        r.append(("an invented task is refused", True, "ValueError"))
    check("TASK_UNKNOWN is not declarable",
          "TASK_UNKNOWN" in mc.TASKS, False)

    print("\nbench_auto — the switch that fabricates a state")
    check("bench_auto forces AUTO from MANUAL",
          mc.decide("MANUAL", True, bench_auto=True)[0], AUTO)
    check("bench_auto says so in the reason",
          "BENCH_AUTO" in mc.decide("MANUAL", True, bench_auto=True)[2], True)
    check("bench_auto CANNOT override a kill",
          mc.decide("MANUAL", True, killed=True, bench_auto=True)[0], KILLED)

    for name, ok, detail in r:
        print("  %-42s %s  %s" % (name, "PASS" if ok else "FAIL",
                                  "" if ok else detail))
    passed = sum(1 for _, ok, _ in r if ok)
    print("\n%d/%d" % (passed, len(r)))
    return 0 if passed == len(r) else 1


if __name__ == "__main__":
    raise SystemExit(main())
