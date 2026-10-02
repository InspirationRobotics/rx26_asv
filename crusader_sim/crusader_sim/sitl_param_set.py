"""sitl_param_set — set one parameter on the sim's ArduRover SITL, over SITL's own spare TCP port.

    python3 -m crusader_sim.sitl_param_set SIM_GPS_HDG 0 [--endpoint tcp:127.0.0.1:5762]

Runs in: WSL (host python3 + pymavlink, no ROS). SITL lives in WSL, not in crsd-sim, and the
S9 test (docs/nav2_avoidance_spec.md 10.2) changes SIM_GPS_HDG mid-mission from a script.

WHY TCP 5762 AND NOT ANY UDP PORT. SITL listens on tcp 5760 (SERIAL0: MAVProxy's master),
5762 (SERIAL1) and 5763 (SERIAL2). Nothing in the rig uses 5762 or 5763. A TCP client is its
own connection and takes nothing from anyone, whereas a `udpin` bind STEALS datagrams: bound
on 14551 it would silence telemetry_bridge, and 14550 is the arming tool's and a scratch QGC's
(crusader-net skill, UDP port discipline). The boat's port map is untouched.

Prints one line, `[sitl] NAME = VALUE acked epoch=<seconds> (<endpoint>)` on success, with the
wall-clock time of the autopilot's PARAM_VALUE echo (the moment it took effect, as far as a
script can tell), and `[sitl] NAME NOT acked: <why>` with exit status 1 otherwise. SITL's own
parameters persist only until its next `-w` start, which every gz_sim_up.sh does.
"""
import argparse
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")

SITL_ENDPOINT = "tcp:127.0.0.1:5762"


def acks(msg, name, value):
    """Is this PARAM_VALUE the autopilot's echo of `name` set to `value`? (float32 on the wire)"""
    pid = msg.param_id.decode("ascii", "ignore") if isinstance(msg.param_id, bytes) else msg.param_id
    return pid.strip("\x00") == name and abs(msg.param_value - value) <= 1e-4 * max(1.0, abs(value))


def set_param(name, value, endpoint=SITL_ENDPOINT, wait_s=5.0, tries=3):
    """Set `name` to `value`, resending up to `tries` times, wait_s for each echo.
    Returns (acked, detail, epoch of the echo or None)."""
    from pymavlink import mavutil
    try:
        link = mavutil.mavlink_connection(endpoint, source_system=255)
    except OSError as e:
        return False, f"cannot connect to {endpoint}: {e}", None
    try:
        if link.wait_heartbeat(timeout=wait_s * 2) is None:
            return False, f"no HEARTBEAT on {endpoint}", None
        for _ in range(tries):
            link.mav.param_set_send(link.target_system, link.target_component, name.encode("ascii"),
                                    float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            t_end = time.time() + wait_s
            while time.time() < t_end:
                msg = link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if msg is not None and acks(msg, name, float(value)):
                    return True, "", time.time()
        return False, f"no PARAM_VALUE echo after {tries} tries (is {name} a parameter of this SITL?)", None
    finally:
        link.close()


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("name")
    ap.add_argument("value", type=float)
    ap.add_argument("--endpoint", default=SITL_ENDPOINT)
    ap.add_argument("--wait-s", type=float, default=5.0, help="per try")
    a = ap.parse_args(argv)
    ok, why, t = set_param(a.name, a.value, a.endpoint, a.wait_s)
    if ok:
        print(f"[sitl] {a.name} = {a.value:g} acked epoch={t:.3f} ({a.endpoint})", flush=True)
        return 0
    print(f"[sitl] {a.name} NOT acked: {why}", flush=True)
    return 1


if __name__ == "__main__":
    sys.exit(main())
