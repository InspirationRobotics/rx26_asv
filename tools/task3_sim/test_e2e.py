#!/usr/bin/env python3
"""test_e2e.py — the real tree against the simulated course, scenario by scenario.

    python tools/task3_sim/test_e2e.py              # everything, ~5 min
    python tools/task3_sim/test_e2e.py bay1 core    # just these

Each scenario is one `sim.py --headless` run in its own process, several at
once. Real time, because the tree's timers are wall-clock.

Every scenario states what SHOULD happen, including the ones that should fail.
A failure case that passes is as much a bug as a pass case that fails: the
WP_RADIUS=2.0 run, for instance, exists to prove the sim reproduces the
precondition in task3_disruptive.xml rather than papering over it.
"""
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor

HERE = os.path.dirname(os.path.abspath(__file__))
SIM = os.path.join(HERE, "sim.py")

# name: (scenario knobs, expectation, timeout_s)
#   "pass"          every report right, SUCCESS, no hull contact
#   "pass_contact"  every report right, SUCCESS, AND the hull touched the dock:
#                   a known limit (see task3_disruptive.xml note 5), pinned so a
#                   fix shows up as this scenario changing
#   "resent"        as "pass", and the docking report was sent more than once
#   "no_dock"       the tree must NOT report docking, and must not succeed
#   "fail_fast"     the tree must fail, reporting nothing, within 10 s
#   "no_fire"       docked and reported, but no fire report and no success
SCENARIOS = {
    "bay2":        ({"green_bay": 2, "tier": 2, "code": ["red", "blue"]}, "pass", 240),
    "bay1":        ({"green_bay": 1, "tier": 2, "code": ["green", "green"], "target_window": 1}, "pass", 240),
    "bay3":        ({"green_bay": 3, "tier": 2, "code": ["blue", "red"]}, "pass", 240),
    "east_facing": ({"green_bay": 1, "tier": 2, "facing_deg": 90.0, "dock_e": -30.0, "dock_n": 20.0,
                     "start_heading": 270.0, "code": ["blue", "green"]}, "pass", 260),
    "north_facing": ({"green_bay": 3, "tier": 2, "facing_deg": 0.0, "dock_n": -40.0,
                      "start_heading": 180.0, "code": ["green", "red"]}, "pass", 260),
    "core":        ({"green_bay": 3, "tier": 0}, "pass", 200),
    "advanced":    ({"green_bay": 2, "tier": 1, "code": ["red", "green"]}, "pass", 240),
    "noisy":       ({"green_bay": 1, "tier": 2, "miscolour": 0.05, "unknown_rate": 0.15,
                     "code": ["blue", "blue"], "seed": 7}, "pass", 280),
    # A 5 cm/s cross-current: the mission still works, but the berth cannot be
    # held - LOIT_RADIUS 2 m, no strafing, 0.8 m of room. The known limit.
    "current":     ({"green_bay": 3, "tier": 2, "current_mps": 0.05, "current_to_deg": 90.0,
                     "code": ["red", "red"]}, "pass_contact", 260),
    # RoboCommand never hears the first docking report: the tree must re-send.
    "lost_report": ({"green_bay": 1, "tier": 2, "lose_docking_reports": 1}, "resent", 260),
    # THE PRECONDITION: this boat's real WP_RADIUS parks it 2 m short.
    "wp_radius_2": ({"green_bay": 2, "tier": 2, "wp_radius": 2.0}, "no_dock", 150),
    # A dead dock detector: the guard band must stop the run, not drive blind.
    "no_camera":   ({"green_bay": 2, "tier": 2, "camera_ok": False}, "fail_fast", 30),
    # Confirmed, but the fire never lights: fail, and never claim a fire out.
    "no_fire":     ({"green_bay": 2, "tier": 2, "activation_delay_s": 1e6}, "no_fire", 200),
}


def run(name):
    knobs, expect, timeout = SCENARIOS[name]
    t0 = time.monotonic()
    p = subprocess.run([sys.executable, SIM, "--headless", "--timeout", str(timeout),
                        "--scenario", json.dumps(knobs)],
                       capture_output=True, text=True, timeout=timeout + 90)
    try:
        out = json.loads(p.stdout)
    except ValueError:
        return name, False, "no verdict: " + (p.stderr or p.stdout)[-400:], time.monotonic() - t0
    m, j = out["mission"], out["judge"]
    reported = [r["kind"] for r in out["reports"]]
    reports_right = (m.get("outcome") == 0 and j["docking"] and j["firefighting"]
                     and (knobs.get("tier", 2) == 0 or (j["request"] and j["uav"])))
    if expect == "pass":
        ok = out["ok"]
        why = "%s in %.0fs, judge %s" % (m.get("outcome_name"), m.get("elapsed_s", 0), j)
    elif expect == "pass_contact":
        ok = reports_right and j["contacts"] >= 1
        why = "%s in %.0fs, %d contact(s), judge %s" % (
            m.get("outcome_name"), m.get("elapsed_s", 0), j["contacts"], j)
    elif expect == "resent":
        ok = out["ok"] and reported.count("docking_report") >= 2
        why = "%s in %.0fs, %d docking reports" % (
            m.get("outcome_name"), m.get("elapsed_s", 0), reported.count("docking_report"))
    elif expect == "no_dock":
        ok = m.get("outcome") != 0 and "docking_report" not in reported
        why = "%s, reports %s" % (m.get("outcome_name"), reported)
    elif expect == "fail_fast":
        ok = m.get("outcome") not in (0, None) and not reported and m.get("elapsed_s", 99) < 10
        why = "%s after %.1fs: %s" % (m.get("outcome_name"), m.get("elapsed_s", 0), m.get("detail"))
    elif expect == "no_fire":
        ok = (m.get("outcome") != 0 and "docking_report" in reported
              and "firefighting_report" not in reported)
        why = "%s, reports %s" % (m.get("outcome_name"), reported)
    else:
        ok, why = False, "unknown expectation " + expect
    return name, ok, why, time.monotonic() - t0


def main():
    names = sys.argv[1:] or list(SCENARIOS)
    for n in names:
        if n not in SCENARIOS:
            sys.exit("no scenario %r; have: %s" % (n, ", ".join(SCENARIOS)))
    print("running %d scenarios, 6 at a time, in real time ..." % len(names), flush=True)
    results = []
    with ThreadPoolExecutor(6) as pool:
        for name, ok, why, took in pool.map(run, names):
            results.append(ok)
            print("  [%s] %-13s %-8s %5.0fs  %s" % ("ok" if ok else "FAIL", name,
                  SCENARIOS[name][1], took, why), flush=True)
    print("\n%d/%d as expected" % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
