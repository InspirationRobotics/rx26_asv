#!/usr/bin/env python3
"""check_sitl — prove the guided path against a real ArduRover, with no boat.

    python3 tools/sitl/check_sitl.py            # full check
    python3 tools/sitl/check_sitl.py --quick    # skip the drive

Needs only pymavlink, and `bash tools/sitl/start_sitl.sh` running first.

WHY THIS EXISTS. Until 2026-09-06 nothing had ever sent a real
SET_POSITION_TARGET_GLOBAL_INT from this codebase — publish_setpoints has been
false everywhere, so telemetry_bridge._guided_cb's body had never run. The mask,
the frame and the whole assumption that ArduRover would accept and act on them
were unverified. This script sends the EXACT message telemetry_bridge sends and
watches the vehicle move.

It runs against SITL and against the boat unchanged, because start_sitl.sh
copies the boat's port map. That is the point of copying it.
"""
import argparse
import math
import sys
import time

from pymavlink import mavutil

# Must equal POSITION_ONLY_TYPE_MASK in crusader_fcu/telemetry_bridge.py.
# Bits: use x/y, ignore z, ignore velocity, acceleration, yaw and yaw rate.
POSITION_ONLY_TYPE_MASK = 0b110111111100      # 3580

fails = []


def chk(name, ok, detail=""):
    print("  [{}] {}{}".format("ok" if ok else "FAIL", name,
                               "  " + detail if detail else ""))
    if not ok:
        fails.append(name)
    return ok


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550",
                    help="14550 is the ad-hoc port; 14551 belongs to "
                         "telemetry_bridge and taking it steals its datagrams")
    ap.add_argument("--distance", type=float, default=60.0)
    ap.add_argument("--quick", action="store_true", help="skip the drive")
    a = ap.parse_args()

    print("connecting to {} ...".format(a.endpoint))
    m = mavutil.mavlink_connection(a.endpoint)
    if not m.wait_heartbeat(timeout=20):
        print("no HEARTBEAT. Is start_sitl.sh running?")
        return 1
    print("connected: system {}\n".format(m.target_system))

    # ---- HEARTBEAT rate. The boat sets 5 Hz at runtime because the OCS link
    # ---- samples at 2 Hz and 1 Hz cannot honestly feed that.
    t0, n = time.time(), 0
    while time.time() - t0 < 6.0:
        if m.recv_match(type="HEARTBEAT", blocking=True, timeout=2):
            n += 1
    hz = n / (time.time() - t0)
    chk("HEARTBEAT >= 4 Hz", hz >= 4.0, "{:.2f} Hz".format(hz))

    # ---- a 3D fix, or nothing below means anything
    fix = 0
    for _ in range(40):
        g = m.recv_match(type="GPS_RAW_INT", blocking=True, timeout=2)
        if g and g.fix_type >= 3:
            fix = g.fix_type
            break
    chk("GPS fix", fix >= 3, "fix_type={}".format(fix))

    # ---- the mode latch, from both sides
    modes = m.mode_mapping()
    chk("GUIDED exists in the mode map", "GUIDED" in modes)

    def set_mode(name):
        m.set_mode(modes[name])
        for _ in range(40):
            m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if m.flightmode == name:
                return True
        return False

    chk("can enter HOLD", set_mode("HOLD"))
    chk("can enter GUIDED", set_mode("GUIDED"))

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                            0, 1, 0, 0, 0, 0, 0, 0)
    ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=6)
    chk("arms", ack is not None and ack.result == 0,
        "result={}".format(ack.result if ack else "no ack"))

    if a.quick:
        return report()

    # ---- THE ONE THAT MATTERS: the exact message telemetry_bridge sends
    p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=6)
    if p is None:
        chk("has a position", False)
        return report()
    lat0, lon0 = p.lat / 1e7, p.lon / 1e7
    tgt_lat = lat0 + a.distance / 111320.0
    tgt_lon = lon0
    print("\n  driving {:.0f} m north from {:.7f}, {:.7f}".format(
        a.distance, lat0, lon0))

    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_ONLY_TYPE_MASK,
        int(tgt_lat * 1e7), int(tgt_lon * 1e7), 0,
        0, 0, 0, 0, 0, 0, 0, 0)

    t0, best, moved = time.time(), 1e9, False
    while time.time() - t0 < 60.0:
        p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=2)
        if p is None:
            continue
        d = math.hypot(
            (p.lat / 1e7 - tgt_lat) * 111320.0,
            (p.lon / 1e7 - tgt_lon) * 111320.0 * math.cos(math.radians(lat0)))
        best = min(best, d)
        if math.hypot(p.vx, p.vy) / 100.0 > 0.3:
            moved = True
        if d < 5.0:
            break

    chk("the vehicle actually moved", moved)
    chk("reached the setpoint (< 5 m)", best < 5.0,
        "closest {:.1f} m after {:.0f}s".format(best, time.time() - t0))

    # ---- and the latch the other way: a setpoint in HOLD must do nothing.
    # ---- This is ArduPilot's half of the interlock -- telemetry_bridge refuses
    # ---- to send outside the autonomous modes, and the autopilot refuses to
    # ---- act on it if something does. Neither alone is the safety story.
    set_mode("HOLD")
    p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    hold_lat, hold_lon = p.lat / 1e7, p.lon / 1e7
    m.mav.set_position_target_global_int_send(
        0, m.target_system, m.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        POSITION_ONLY_TYPE_MASK,
        int((hold_lat + 0.0009) * 1e7), int(hold_lon * 1e7), 0,
        0, 0, 0, 0, 0, 0, 0, 0)
    time.sleep(8)
    p = m.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=5)
    drift = math.hypot((p.lat / 1e7 - hold_lat) * 111320.0,
                       (p.lon / 1e7 - hold_lon) * 111320.0)
    chk("a setpoint in HOLD is IGNORED", drift < 10.0,
        "moved {:.1f} m".format(drift))

    return report()


def report():
    print()
    if fails:
        print("==== SITL CHECK: FAIL ({}) ====".format(", ".join(fails)))
        return 1
    print("==== SITL CHECK: PASS ====")
    return 0


if __name__ == "__main__":
    sys.exit(main())
