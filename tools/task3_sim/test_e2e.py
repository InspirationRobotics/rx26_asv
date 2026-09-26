#!/usr/bin/env python3
"""test_e2e.py — the real tree against the simulated course, scenario by scenario.

    python tools/task3_sim/test_e2e.py              # everything, ~8 min
    python tools/task3_sim/test_e2e.py bay1 core    # just these
    python tools/task3_sim/test_e2e.py fire         # every fire_* scenario

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

# name: (scenario knobs, expectation, timeout_s[, options])
#
# options: {"fire": True} runs task3_fire_test.xml on sim.py's --fire course
# (the knobs go on top of it); "pump": True adds --fire-pump; "set": a list of
# NODE.port=value overrides for a copy of the tree.
#
# Task 3 (task3_disruptive.xml):
#   "pass"          every report right, SUCCESS, no hull contact
#   "pass_contact"  every report right, SUCCESS, AND the hull touched the dock:
#                   a known limit (see task3_disruptive.xml note 5), pinned so a
#                   fix shows up as this scenario changing
#   "resent"        as "pass", and the docking report was sent more than once
#   "no_dock"       the tree must NOT report docking, and must not succeed
#   "fail_fast"     the tree must fail, reporting nothing, within 10 s
#   "no_fire"       docked and reported, but no fire report and no success
# The fixed-nozzle shot (task3_fire_test.xml):
#   "fire_out"      SUCCESS, the window put out by a burst of water, no hull
#                   contact, never inside the keep's 1.5 m floor
#   "no_burst"      not a drop of water, not even a pump command, and no SUCCESS
#   "stops"         the pilot takes it back mid-approach (over 10 cm/s): below
#                   5 cm/s 1 s later, no water after, and no SUCCESS
#   "fail_loud"     the pump path is down: FAILURE, and no water
#   "misses"        bursts go out and every one misses: the window stays lit
#   "dry"           fire_pump off: the tree lines up and "fires" DRY - no pump
#                   command at all - and the window stays lit
FIRE = {"fire": True, "pump": True}
FINGERS = dict(FIRE, set=["StationKeep.lateral=fingers"])
SCENARIOS = {
    # The default course: the build guide's dock 20 m ahead, bays opening south,
    # the camera pitched up 25 deg (see world.Scenario.cam_pitch_deg).
    "bay2":        ({"green_bay": 2, "tier": 2, "code": ["red", "blue"]}, "pass", 200),
    "bay1":        ({"green_bay": 1, "tier": 2, "code": ["green", "green"], "target_window": 0}, "pass", 200),
    "bay3":        ({"green_bay": 3, "tier": 2, "code": ["blue", "red"]}, "pass", 200),
    "east_facing": ({"green_bay": 1, "tier": 2, "facing_deg": 90.0, "dock_e": -15.0, "dock_n": 10.0,
                     "start_heading": 270.0, "code": ["blue", "green"]}, "pass", 240),
    "north_facing": ({"green_bay": 3, "tier": 2, "facing_deg": 0.0, "dock_n": -20.0,
                      "start_heading": 180.0, "code": ["green", "red"], "target_window": 0}, "pass", 220),
    "core":        ({"green_bay": 3, "tier": 0}, "pass", 160),
    "advanced":    ({"green_bay": 2, "tier": 1, "code": ["red", "green"]}, "pass", 200),
    "noisy":       ({"green_bay": 1, "tier": 2, "miscolour": 0.05, "unknown_rate": 0.15,
                     "code": ["blue", "blue"], "seed": 7}, "pass", 240),
    # A 5 cm/s cross-current: the mission still works, but the berth cannot be
    # held - LOIT_RADIUS 2 m, no strafing, 0.45 m of room. The known limit.
    "current":     ({"green_bay": 3, "tier": 2, "current_mps": 0.05, "current_to_deg": 90.0,
                     "code": ["red", "red"]}, "pass_contact", 240),
    # RoboCommand never hears the first docking report: the tree must re-send.
    "lost_report": ({"green_bay": 1, "tier": 2, "lose_docking_reports": 1}, "resent", 240),
    # THE CAMERA AS MOUNTED TODAY: level. It docks, reports, and never sees the
    # fire - neither window is in view from the berth. Pinned, so a better
    # mount shows up as this scenario changing.
    "level_camera": ({"green_bay": 2, "tier": 2, "cam_pitch_deg": 0.0}, "no_fire", 200),
    # THE PRECONDITION: this boat's real WP_RADIUS parks it 2 m short.
    "wp_radius_2": ({"green_bay": 2, "tier": 2, "wp_radius": 2.0}, "no_dock", 130),
    # A dead dock detector: the guard band must stop the run, not drive blind.
    "no_camera":   ({"green_bay": 2, "tier": 2, "camera_ok": False}, "fail_fast", 30),
    # Confirmed, but the fire never lights: fail, and never claim a fire out.
    "no_fire":     ({"green_bay": 2, "tier": 2, "activation_delay_s": 1e6}, "no_fire", 180),

    # ---- the fixed-nozzle shot. The boat starts 3.8 m off bay 2's slip.
    "fire_calm":        ({}, "fire_out", 60, FIRE),
    # Rocking: it must wait for a calm spell, and still hit.
    "fire_rocking":     ({"sea": 2.0, "seed": 3}, "fire_out", 120, FIRE),
    # Too rough to ever call steady: it never fires.
    "fire_rough":       ({"sea": 3.0, "seed": 3}, "no_burst", 60, FIRE),
    # 3 cm/s across the slip: it drifts sideways (heading+speed cannot strafe);
    # the fingers' offset re-aims the heading.
    "fire_current":     ({"current_mps": 0.03, "current_to_deg": 90.0}, "fire_out", 90, FINGERS),
    # 0.3 m left of the slip centre: aimed from the fingers' offset.
    "fire_offset":      ({"start_e": -0.3}, "fire_out", 60, FINGERS),
    # Starts 2.0 m out, between the fingers: backs out to the firing range.
    "fire_close_start": ({"start_n": 18.0}, "fire_out", 60, FIRE),
    # 5 m out the LiDAR cannot see the dock (wall_range_node.r_max 4 m): the
    # guard band stops the run at once. Hand over inside 4 m...
    "fire_beyond_lidar": ({"start_n": 15.0}, "no_burst", 30, FIRE),
    # ...or raise r_max: from 8 m it approaches square, then aims.
    "fire_far_start":   ({"start_n": 12.0, "wall_r_max": 9.0}, "fire_out", 90, FIRE),
    # No wall from the LiDAR: nothing moves, nothing fires.
    "fire_no_wall":     ({"wall_ok": False}, "no_burst", 30, FIRE),
    # Attitude at 4 Hz is too slow to call the hull steady: never fires.
    "fire_slow_att":    ({"att_hz": 4.0}, "no_burst", 45, FIRE),
    # The wall fit locks onto the finger tips, 2 m short: the camera's range
    # disagrees, so no aim and no shot.
    "fire_finger_lock": ({"wall_on_fingers": True}, "no_burst", 45, FIRE),
    # The pilot takes it back mid-approach, both ways. Autopilot avoidance
    # off, so the stop is the bridge's (SE) or the mode's (SC), not a lucky
    # re-enabled avoidance next to the fingers.
    "fire_drop":        ({"drop_at_s": 2.5, "autopilot_avoidance": False}, "stops", 40, FIRE),
    "fire_mode_manual": ({"manual_at_s": 2.5, "autopilot_avoidance": False}, "stops", 40, FIRE),
    # The bridge has no pump output (G7 not done): FireBurst fails, loudly.
    "fire_pump_off":    ({"pump_path": False}, "fail_loud", 90, FIRE),
    # The calibration is wrong (the truth is 3.6 m, the tree believes 3.22):
    # it fires, misses high, and says so shot by shot.
    "fire_cal_error":   ({"nozzle_hit_range_m": 3.6}, "misses", 120, FIRE),
    # fire_pump off: the shadow posture. Lines up, fires DRY, no water.
    "fire_dry":         ({}, "dry", 90, {"fire": True}),
}


def run(name):
    knobs, expect, timeout = SCENARIOS[name][:3]
    opts = SCENARIOS[name][3] if len(SCENARIOS[name]) > 3 else {}
    args = [sys.executable, SIM, "--headless", "--timeout", str(timeout),
            "--scenario", json.dumps(knobs)]
    if opts.get("fire"):
        args.append("--fire")
    if opts.get("pump"):
        args.append("--fire-pump")
    for item in opts.get("set", []):
        args += ["--set", item]
    t0 = time.monotonic()
    p = subprocess.run(args, capture_output=True, text=True, timeout=timeout + 90)
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
    elif expect.startswith("fire_") or expect in ("no_burst", "stops", "fail_loud", "misses", "dry"):
        ok, why = fire_verdict(expect, out)
    else:
        ok, why = False, "unknown expectation " + expect
    return name, ok, why, time.monotonic() - t0


def fire_verdict(expect, out):
    """(ok, why) for a task3_fire_test.xml run."""
    m, j, f, shots = out["mission"], out["judge"], out["fire"], out["shots"]
    b = f["bridge"]
    outcome = m.get("outcome")
    desc = "%s in %.0fs, %d shot(s) %d in, window %s, pump cmds %d, closest %.2f m" % (
        m.get("outcome_name"), m.get("elapsed_s", 0) or 0, len(shots), f["n_hits"],
        "OUT" if f["window_out"] else "lit", b["pump_cmds"], f["min_range"])
    if expect == "fire_out":
        ok = (outcome == 0 and f["window_out"] and f["n_hits"] >= 1 and j["contacts"] == 0
              and f["min_range"] >= 1.4)
    elif expect == "no_burst":
        ok = outcome != 0 and b["pump_cmds"] == 0 and not shots
        desc += ": " + (m.get("detail") or "")
    elif expect == "stops":
        ev = f["event"] or {}
        v1 = ev.get("v_1s")
        ok = (v1 is not None and v1 < 0.05 and (ev.get("v_at") or 0) > 0.1 and outcome != 0
              and all(s["t"] < ev["t"] for s in shots))
        desc += ", %s at %.1fs at %s m/s -> %s m/s 1 s later" % (ev.get("what"), ev.get("t", 0),
                                                                 ev.get("v_at"), v1)
    elif expect == "fail_loud":
        ok = outcome not in (0, None) and b["bursts"] == 0 and not shots
        desc += ": " + (m.get("detail") or "")
    elif expect == "misses":
        ok = len(shots) >= 1 and f["n_hits"] == 0 and not f["window_out"]
        desc += ", misses dz %s" % [s["dz"] for s in shots]
    elif expect == "dry":
        ok = outcome == 0 and b["pump_cmds"] == 0 and not shots and not f["window_out"]
    else:
        ok, desc = False, "unknown expectation " + expect
    return ok, desc


def main():
    names = sys.argv[1:] or list(SCENARIOS)
    if names == ["fire"]:
        names = [n for n in SCENARIOS if n.startswith("fire_")]
    for n in names:
        if n not in SCENARIOS:
            sys.exit("no scenario %r; have: %s" % (n, ", ".join(SCENARIOS)))
    print("running %d scenarios, 6 at a time, in real time ..." % len(names), flush=True)
    results = []
    with ThreadPoolExecutor(6) as pool:
        for name, ok, why, took in pool.map(run, names):
            results.append(ok)
            print("  [%s] %-16s %-9s %5.0fs  %s" % ("ok" if ok else "FAIL", name,
                  SCENARIOS[name][1], took, why), flush=True)
    print("\n%d/%d as expected" % (sum(results), len(results)))
    sys.exit(0 if all(results) else 1)


if __name__ == "__main__":
    main()
