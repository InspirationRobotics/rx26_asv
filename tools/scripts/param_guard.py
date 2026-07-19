#!/usr/bin/env python3
"""param_guard — protect Crusader's hard-won ArduRover config.

Two uses:
  1. CLI: diff a candidate param file (or the live vehicle via MAVProxy's
     rebroadcast) against the known-good file. Nonzero exit if any PROTECTED
     param differs. Used by preflight and by CI on any PR touching params.
  2. Library: the Level-1 autoresearch evaluator imports is_tunable() to reject
     any proposed change that touches a protected param.

PROTECTED params encode the "critical engineering lessons" in CLAUDE.md —
steering inversion lives in SERVOx_*, compass stays off, arming checks stay on.
Autoresearch may only ever touch TUNABLE params.

Usage:
    param_guard.py known_good.params candidate.params
    param_guard.py known_good.params --live udp:127.0.0.1:14550
"""
import fnmatch
import sys

PROTECTED = [
    "ARMING_REQUIRE", "ARMING_CHECK", "BRD_SAFETYOPTION",
    "RC*_REVERSED",                       # never "fix" inversion here
    "SERVO1_REVERSED", "SERVO4_REVERSED", # rear thrusters — physical, correct
    "PILOT_STEER_TYPE",                   # =3, required for autonomous reverse
    "COMPASS_USE", "COMPASS_USE2", "COMPASS_USE3",  # compass off, GPS yaw
    "EK3_SRC1_YAW", "AHRS_EKF_TYPE",
    "FRAME_TYPE",                         # =2 OmniX
    "AVOID_ENABLE",                       # avoidance on; margin is tunable, enable is not
]

TUNABLE = [
    "AVOID_MARGIN", "AVOID_BEHAVE", "AVOID_BACKUP_SPD", "AVOID_ACCEL_MAX",
    "WP_RADIUS", "CRUISE_SPEED", "CRUISE_THROTTLE", "TURN_RADIUS", "WP_PIVOT_RATE",
    "MOT_THST_ASYM", "MOT_THST_EXPO",
    "DOCK_*",   # flagged open question — tunable only inside a documented experiment
]

def _match(name, patterns):
    return any(fnmatch.fnmatch(name, p) for p in patterns)

def is_protected(name): return _match(name, PROTECTED)
def is_tunable(name):   return _match(name, TUNABLE) and not is_protected(name)

def load_param_file(path):
    params = {}
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            # Mission Planner format: NAME,VALUE  — QGC format: NAME\tVALUE (+extra cols)
            for sep in (",", "\t", " "):
                if sep in line:
                    parts = [p for p in line.split(sep) if p]
                    try:
                        params[parts[0]] = float(parts[1])
                    except (IndexError, ValueError):
                        pass
                    break
    return params

def fetch_live(endpoint, timeout=60):
    """Read-only fetch via MAVProxy REBROADCAST. Never the Pixhawk serial port."""
    from pymavlink import mavutil
    m = mavutil.mavlink_connection(endpoint)
    m.wait_heartbeat(timeout=timeout)
    m.mav.param_request_list_send(m.target_system, m.target_component)
    params, expected = {}, None
    import time
    deadline = time.time() + timeout
    while time.time() < deadline:
        msg = m.recv_match(type="PARAM_VALUE", blocking=True, timeout=5)
        if msg is None:
            break
        params[msg.param_id] = msg.param_value
        expected = msg.param_count
        if expected and len(params) >= expected:
            break
    return params

def diff(known, other):
    """Returns (protected_violations, tunable_diffs) as lists of strings."""
    bad, warn = [], []
    for name, val in known.items():
        if name not in other:
            continue  # absent on the other side; not a drift signal by itself
        if abs(other[name] - val) > 1e-6:
            entry = f"{name}: known={val:g} other={other[name]:g}"
            (bad if is_protected(name) else warn).append(entry)
    return bad, warn

def diff_live(endpoint, known_path):
    return diff(load_param_file(known_path), fetch_live(endpoint))

def main():
    if len(sys.argv) < 3:
        print(__doc__)
        sys.exit(2)
    known = load_param_file(sys.argv[1])
    if sys.argv[2] == "--live":
        other = fetch_live(sys.argv[3] if len(sys.argv) > 3 else "udp:127.0.0.1:14550")
    else:
        other = load_param_file(sys.argv[2])
    bad, warn = diff(known, other)
    for b in bad:
        print(f"PROTECTED DRIFT: {b}")
    for w in warn:
        print(f"tunable diff:    {w}")
    if bad:
        print(f"\nFAIL — {len(bad)} protected param(s) differ. Do not fly this config.")
        sys.exit(1)
    print(f"\nOK — no protected drift ({len(warn)} tunable diffs).")

if __name__ == "__main__":
    main()
