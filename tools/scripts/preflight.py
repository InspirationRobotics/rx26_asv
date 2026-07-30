#!/usr/bin/env python3
"""Crusader preflight — run on the Jetson HOST before every arm (bench or water).

Exit code 0 = safe to proceed to arming. Nonzero = DO NOT ARM; fix and re-run.
Checks degrade gracefully: a check that can't run (missing dep on this host) is
reported as SKIP, never silently passed — the operator decides if a SKIP is ok.

Usage:
    python3 preflight.py [--params-file working_crusader_params.params]
                         [--mav udp:127.0.0.1:14550] [--skip-ros]
"""
import argparse
import os
import shutil
import subprocess
import sys

# Container name. Overridable so a renamed/parallel container (e.g. a
# "asv-next" built from a new image) can be preflighted without editing code.
CONTAINER = os.environ.get("CRSD_CONTAINER", "asv")

RESULTS = []  # (name, status, detail)   status in {PASS, FAIL, WARN, SKIP}

def record(name, status, detail=""):
    RESULTS.append((name, status, detail))
    print(f"[{status:4}] {name}" + (f" — {detail}" if detail else ""))

def check_symlinks():
    required = ["/dev/crsd-pixhawk", "/dev/crsd-led"]
    # crsd-gps/-teensy/-ball-launcher come from the generated per-boat rules
    # (tools/udev/gen_udev_rules.py + config/crusader_devices.json)
    optional = ["/dev/crsd-gps", "/dev/crsd-gps1", "/dev/crsd-gps2",
                "/dev/crsd-telem", "/dev/crsd-teensy", "/dev/crsd-ball-launcher"]
    for dev in required:
        record(f"udev {dev}", "PASS" if os.path.exists(dev) else "FAIL",
               "" if os.path.exists(dev) else "symlink missing — udev rules / cabling")
    for dev in optional:
        if not os.path.exists(dev):
            record(f"udev {dev}", "WARN", "optional symlink missing")

def check_disk():
    free_gb = shutil.disk_usage("/").free / 1e9
    record("disk space", "PASS" if free_gb > 5 else "FAIL", f"{free_gb:.1f} GB free (need >5 for bags)")

def check_oakd_usb():
    # Must report SUPER (USB3). HIGH (USB2) tanks stereo throughput.
    try:
        import depthai as dai  # noqa
        speed = str(dai.Device().getUsbSpeed())
        ok = "SUPER" in speed
        record("OAK-D USB speed", "PASS" if ok else "FAIL", speed)
    except ImportError:
        record("OAK-D USB speed", "SKIP", "depthai not on host — run inside container")
    except Exception as e:
        record("OAK-D USB speed", "FAIL", f"device error: {e}")

def check_container():
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
    try:
        out = subprocess.run(["pgrep", "-f", "mavproxy"], capture_output=True, text=True)
        alive = out.returncode == 0
        record("MAVProxy process", "PASS" if alive else "FAIL",
               "" if alive else "start scripts/start_mavproxy.sh — sole Pixhawk owner")
    except Exception as e:
        record("MAVProxy process", "FAIL", str(e))

def check_params(mav_endpoint, params_file):
    """Read-only param diff vs known-good file, via MAVProxy REBROADCAST only.
    Never opens the Pixhawk serial device directly."""
    if not params_file or not os.path.exists(params_file):
        record("param diff", "SKIP", "no --params-file given/found")
        return
    try:
        from param_guard import diff_live  # same directory
        bad, warn = diff_live(mav_endpoint, params_file)
        if bad:
            record("param diff (protected)", "FAIL", "; ".join(bad[:5]))
        else:
            record("param diff (protected)", "PASS")
        if warn:
            record("param diff (tunables)", "WARN", f"{len(warn)} tunables differ (see param_guard.py output)")
    except ImportError:
        record("param diff", "SKIP", "pymavlink not available on host")
    except Exception as e:
        record("param diff", "FAIL", str(e))

def check_ros(container_ok):
    if not container_ok:
        record("ROS topics", "SKIP", "container down")
        return
    # core.launch.py topics. All /crsd-namespaced — the bare /led_state this
    # checked for was the boat repo's name and always reported FAIL.
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

def check_engine(container_ok):
    if not container_ok:
        record("TensorRT engine", "SKIP", "container down")
        return
    try:
        out = subprocess.run(
            ["docker", "exec", CONTAINER, "bash", "-lc",
             "test -f /root/robotx_ws/models/buoy_v16.engine && echo ok"],
            capture_output=True, text=True, timeout=15)
        ok = "ok" in out.stdout
        record("TensorRT engine present", "PASS" if ok else "FAIL",
               "" if ok else "buoy_v16.engine missing — regenerate per-Jetson (yolo export ... device=0)")
    except Exception as e:
        record("TensorRT engine present", "FAIL", str(e))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--params-file", default="working_crusader_params.params")
    ap.add_argument("--mav", default="udp:127.0.0.1:14550",
                    help="MAVProxy rebroadcast endpoint (never the serial device)")
    ap.add_argument("--skip-ros", action="store_true")
    args = ap.parse_args()

    check_symlinks()
    check_disk()
    check_oakd_usb()
    container_ok = check_container()
    check_mavproxy()
    check_params(args.mav, args.params_file)
    if not args.skip_ros:
        check_ros(container_ok)
        check_engine(container_ok)

    fails = [r for r in RESULTS if r[1] == "FAIL"]
    print("\n==== PREFLIGHT:", "FAIL — DO NOT ARM" if fails else "PASS", "====")
    # Reminder items no script can check:
    print("Manual items: GPS yaw resolved (open sky, 2-3 min)? ELRS e-stop range-tested today?"
          " Autonomy-drop switch verified if any RC-override task is planned?")
    sys.exit(1 if fails else 0)

if __name__ == "__main__":
    main()
