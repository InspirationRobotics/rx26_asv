#!/usr/bin/env python3
"""Crusader preflight — run on the Jetson HOST before every arm (bench or water).

Exit code 0 = safe to proceed to arming. Nonzero = DO NOT ARM; fix and re-run.
Checks degrade gracefully: a check that can't run here is reported as SKIP,
never silently passed — the operator decides if a SKIP is ok.

Run it from the HOST, not from inside the container: the container checks shell
out to `docker`, which does not exist in the container. Running it inside used
to FAIL the container check and then SKIP everything behind it, which read as
"mostly fine" when it was actually "nothing was checked". It now detects that
case and says so.

Usage:
    python3 preflight.py [--params-file working_crusader_params.params]
                         [--mav udp:127.0.0.1:14550] [--skip-ros]
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

# Container name. Overridable so a renamed/parallel container (e.g. an
# "asv-next" built from a new image) can be preflighted without editing code.
CONTAINER = os.environ.get("CRSD_CONTAINER", "asv")

# The committed known-good ArduRover config. Resolved from THIS file's location,
# not the cwd: preflight is run from wherever the operator happens to be stood,
# and a relative default that misses just prints SKIP — which reads as "no param
# baseline exists" rather than "you are in the wrong directory".
DEFAULT_PARAMS = (Path(__file__).resolve().parents[2]
                  / "params" / "working_crusader.params")

RESULTS = []  # (name, status, detail)   status in {PASS, FAIL, WARN, SKIP}


def record(name, status, detail=""):
    RESULTS.append((name, status, detail))
    print(f"[{status:4}] {name}" + (f" — {detail}" if detail else ""))


def on_host():
    """False when we are running INSIDE the asv container.

    /.dockerenv is created by the docker runtime in every container; `docker`
    being absent from PATH is the corroborating signal. Either one alone is
    weak, so require both to declare "not the host".
    """
    return not (os.path.exists("/.dockerenv") and shutil.which("docker") is None)


def check_symlinks():
    # Only the two devices actually aboard (OPERATIONS.md, confirmed hardware).
    # A symlink for anything else means someone rewired the boat without
    # updating tools/udev/99-crusader.rules.
    for dev in ("/dev/crsd-pixhawk", "/dev/crsd-led"):
        ok = os.path.exists(dev)
        record(f"udev {dev}", "PASS" if ok else "FAIL",
               "" if ok else "symlink missing — udev rules / cabling")


def check_disk():
    free_gb = shutil.disk_usage("/").free / 1e9
    record("disk space", "PASS" if free_gb > 5 else "FAIL",
           f"{free_gb:.1f} GB free (need >5 for logs)")


def check_container():
    if not on_host():
        record(f"{CONTAINER} container", "SKIP",
               "running inside the container — re-run from the Jetson host")
        return False
    try:
        out = subprocess.run(["docker", "inspect", "-f", "{{.State.Running}}", CONTAINER],
                             capture_output=True, text=True, timeout=10)
        running = out.stdout.strip() == "true"
        record(f"{CONTAINER} container", "PASS" if running else "FAIL",
               "" if running else "container not running")
        return running
    except Exception as e:
        record(f"{CONTAINER} container", "FAIL", str(e))
        return False


def check_mavproxy():
    """MAVProxy is the sole Pixhawk owner — nothing else works without it."""
    try:
        out = subprocess.run(["pgrep", "-f", "mavproxy"], capture_output=True, text=True)
        alive = out.returncode == 0
        record("MAVProxy process", "PASS" if alive else "FAIL",
               "" if alive else "systemctl status crsd-mavproxy — sole Pixhawk owner")
    except Exception as e:
        record("MAVProxy process", "FAIL", str(e))


def check_params(mav_endpoint, params_file):
    """Read-only param diff vs known-good file, via MAVProxy REBROADCAST only.
    Never opens the Pixhawk serial device directly."""
    if not params_file or not os.path.exists(params_file):
        record("param diff", "SKIP",
               f"no param baseline at {params_file} — export the boat's params "
               "from QGC and commit them (see tools/scripts/param_guard.py)")
        return
    try:
        sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
        from param_guard import diff_live      # same directory
        bad, warn = diff_live(mav_endpoint, params_file)
        if bad:
            record("param diff (protected)", "FAIL", "; ".join(bad[:5]))
        else:
            record("param diff (protected)", "PASS")
        if warn:
            record("param diff (tunables)", "WARN",
                   f"{len(warn)} tunables differ (see param_guard.py output)")
    except ImportError:
        record("param diff", "SKIP", "pymavlink not available on this host")
    except Exception as e:
        record("param diff", "FAIL", str(e))


def check_ros(container_ok):
    if not container_ok:
        record("ROS topics", "SKIP", "container check did not pass")
        return
    # Every topic core.launch.py must be producing. All /crsd-namespaced.
    must_exist = ["/crsd/led_state", "/crsd/pose", "/crsd/fcu_status",
                  "/crsd/rc_channels"]
    try:
        out = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-lc",
             "source /root/robotx_ws/install/setup.bash && ros2 topic list"],
            capture_output=True, text=True, timeout=30)
        topics = out.stdout.split()
        for t in must_exist:
            record(f"topic {t}", "PASS" if t in topics else "FAIL")
    except Exception as e:
        record("ROS topics", "FAIL", str(e))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params-file", default=str(DEFAULT_PARAMS))
    ap.add_argument("--mav", default="udp:127.0.0.1:14550",
                    help="MAVProxy rebroadcast endpoint (never the serial device)")
    ap.add_argument("--skip-ros", action="store_true")
    args = ap.parse_args()

    if not on_host():
        print("WARNING: this looks like the inside of the container. The "
              "container and ROS-topic checks need the host's docker — "
              "re-run from the Jetson host.\n")

    check_symlinks()
    check_disk()
    container_ok = check_container()
    check_mavproxy()
    check_params(args.mav, args.params_file)
    if not args.skip_ros:
        check_ros(container_ok)

    fails = [r for r in RESULTS if r[1] == "FAIL"]
    skips = [r for r in RESULTS if r[1] == "SKIP"]
    print("\n==== PREFLIGHT:", "FAIL — DO NOT ARM" if fails else "PASS", "====")
    if skips:
        print(f"{len(skips)} check(s) SKIPPED — a SKIP is not a PASS. "
              "Decide each one before arming.")
    # Reminder items no script can check:
    print("Manual items: GPS yaw resolved (open sky, 2-3 min)? ELRS e-stop "
          "range-tested today? LED strip showing the state you expect?")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
