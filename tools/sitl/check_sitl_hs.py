#!/usr/bin/env python3
"""check_sitl_hs — prove GUIDED heading+speed against a real ArduRover, with no boat.

    python3 tools/sitl/check_sitl_hs.py

Needs pymavlink, and `bash tools/sitl/start_sitl.sh` running first. Same port
rules as check_sitl.py: 14550 is the ad-hoc port.

WHY THIS EXISTS. The fixed-nozzle shot (task3_fire_test.xml) drives the boat
with SET_ATTITUDE_TARGET - heading from a yaw quaternion, speed from thrust -
which nothing in this codebase had ever sent. guided_hs_core.py says how
ArduRover treats it AS REMEMBERED, and the Task 3 sim models exactly that. This
script sends the EXACT fields telemetry_bridge sends (guided_hs_core.encode)
and measures what the vehicle does. Whatever it finds, the sim's Boat and
guided_hs_core's docstring are corrected to match BEFORE the tree drives the
real boat (docs/T3_coordinated_logistics.md, the water order).

  1  turns to a heading at zero speed      (the keep turns before it drives)
  2  ahead at 0.25 m/s = thrust 0.25       (speed = thrust x WP_SPEED)
  3  astern at -0.25 m/s, bow held          (the keep backs off the dock)
  4  0.12 m/s moves it at all               (the keep's v_min)
  5  3 s without a target: it stops         (the autopilot's own backstop)
  6  in HOLD the same message does nothing  (the mode interlock, ArduPilot's half)

SITL's motorboat is a skid-steer, not this hull, so the numbers are a first
answer, not the boat's: the speed loop below 0.2 m/s in particular wants the
real hull (the range-hold step on the water).
"""
import argparse
import math
import os
import sys
import time

from pymavlink import mavutil

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                "crusader_fcu"))
from crusader_fcu.guided_hs_core import encode   # noqa: E402  the bridge's own encoding

WP_SPEED = 1.0          # = telemetry_bridge.hs_wp_speed_mps = the boat's WP_SPEED
RATE_HZ = 5.0           # the tree ticks at 10 Hz; 5 is the slowest it would send

fails = []


def chk(name, ok, detail=""):
    print("  [{}] {}{}".format("ok" if ok else "FAIL", name, "  " + detail if detail else ""))
    if not ok:
        fails.append(name)
    return ok


def info(name, detail):
    print("  [info] {}  {}".format(name, detail))


class Vehicle:
    def __init__(self, m):
        self.m = m
        self.modes = m.mode_mapping()
        self.yaw = None           # compass degrees
        self.vn = self.ve = 0.0   # m/s

    def pump(self, until):
        """Read messages until `until`, keeping yaw and velocity current."""
        while time.time() < until:
            msg = self.m.recv_match(type=["ATTITUDE", "GLOBAL_POSITION_INT", "HEARTBEAT"],
                                    blocking=True, timeout=0.05)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "ATTITUDE":
                self.yaw = math.degrees(msg.yaw) % 360.0
            elif t == "GLOBAL_POSITION_INT":
                self.vn, self.ve = msg.vx / 100.0, msg.vy / 100.0

    def along(self):
        """Velocity along the bow (+ ahead), m/s."""
        h = math.radians(self.yaw or 0.0)
        return self.vn * math.cos(h) + self.ve * math.sin(h)

    def send(self, heading, speed):
        mask, q, thrust = encode(heading, speed, WP_SPEED)
        self.m.mav.set_attitude_target_send(
            int(time.monotonic() * 1000.0) & 0xFFFFFFFF, self.m.target_system,
            self.m.target_component, mask, q, 0.0, 0.0, 0.0, thrust)

    def drive(self, heading, speed, seconds):
        """Send at RATE_HZ for `seconds`; return (along-speed samples, final yaw)."""
        samples, end = [], time.time() + seconds
        while time.time() < end:
            self.send(heading, speed)
            self.pump(time.time() + 1.0 / RATE_HZ)
            samples.append((time.time(), self.along()))
        return samples, self.yaw

    def set_mode(self, name):
        self.m.set_mode(self.modes[name])
        for _ in range(40):
            self.m.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
            if self.m.flightmode == name:
                return True
        return False


def herr(a, b):
    return abs((a - b + 180.0) % 360.0 - 180.0)


def mean_after(samples, t_from):
    v = [s for t, s in samples if t >= t_from]
    return sum(v) / len(v) if v else float("nan")


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    a = ap.parse_args()

    print("connecting to {} ...".format(a.endpoint))
    m = mavutil.mavlink_connection(a.endpoint)
    if not m.wait_heartbeat(timeout=20):
        print("no HEARTBEAT. Is start_sitl.sh running?")
        return 1
    print("connected: system {}\n".format(m.target_system))
    v = Vehicle(m)

    # 10 Hz attitude and position, as the boat runs (SR0_EXTRA1, SR0_POSITION)
    for msg_id in (mavutil.mavlink.MAVLINK_MSG_ID_ATTITUDE,
                   mavutil.mavlink.MAVLINK_MSG_ID_GLOBAL_POSITION_INT):
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, 0,
                                msg_id, 100000, 0, 0, 0, 0, 0)

    if not chk("can enter GUIDED", v.set_mode("GUIDED")):
        return report()
    armed, t0 = False, time.time()
    while time.time() - t0 < 60.0 and not armed:      # the EKF needs 30-60 s after launch
        m.mav.command_long_send(m.target_system, m.target_component,
                                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                                0, 1, 0, 0, 0, 0, 0, 0)
        ack = m.recv_match(type="COMMAND_ACK", blocking=True, timeout=3)
        armed = bool(ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM
                     and ack.result == 0)
    if not chk("arms", armed):
        return report()
    v.pump(time.time() + 2.0)
    h0 = v.yaw
    print("\n  start heading {:.1f}\n".format(h0))

    # 1: turn at zero speed
    target = (h0 + 90.0) % 360.0
    samples, yaw = v.drive(target, 0.0, 12.0)
    chk("1 turns to a heading at zero speed", herr(yaw, target) < 10.0,
        "wanted {:.0f}, got {:.0f}".format(target, yaw))
    info("1 drift while turning", "{:+.2f} m/s along the bow".format(mean_after(samples, 0)))

    # 2: ahead
    t_start = time.time()
    samples, yaw = v.drive(target, 0.25, 12.0)
    sp = mean_after(samples, t_start + 6.0)
    chk("2 ahead: 0.25 m/s asked, speed = thrust x WP_SPEED", abs(sp - 0.25) < 0.08,
        "{:+.2f} m/s, heading error {:.1f}".format(sp, herr(yaw, target)))

    # 3: astern, bow held
    t_start = time.time()
    samples, yaw = v.drive(target, -0.25, 12.0)
    sp = mean_after(samples, t_start + 6.0)
    chk("3 astern: -0.25 m/s asked", sp < -0.15, "{:+.2f} m/s".format(sp))
    chk("3 astern: the bow holds its heading", herr(yaw, target) < 10.0,
        "heading error {:.1f}".format(herr(yaw, target)))

    # 4: the keep's v_min
    t_start = time.time()
    samples, yaw = v.drive(target, 0.12, 12.0)
    sp = mean_after(samples, t_start + 6.0)
    chk("4 0.12 m/s (v_min) moves it", sp > 0.05, "{:+.2f} m/s".format(sp))

    # 5: silence. ArduRover should stop (and a boat loiter) 3 s after the last target
    v.drive(target, 0.25, 8.0)
    t_quiet = time.time()
    stopped_at = None
    while time.time() - t_quiet < 12.0:
        v.pump(time.time() + 0.2)
        if stopped_at is None and abs(v.along()) < 0.05:
            stopped_at = time.time() - t_quiet
    chk("5 stops by itself after the targets stop", stopped_at is not None,
        "below 5 cm/s {:.1f} s after the last target".format(stopped_at)
        if stopped_at is not None else "still moving after 12 s")
    info("5 mode after the timeout", m.flightmode)

    # 6: the mode interlock
    chk("can enter HOLD", v.set_mode("HOLD"))
    v.pump(time.time() + 3.0)
    samples, yaw = v.drive((yaw + 90.0) % 360.0, 0.3, 8.0)
    moved = max(abs(s) for _t, s in samples)
    chk("6 in HOLD the message is IGNORED", moved < 0.1, "peak {:.2f} m/s".format(moved))

    m.mav.command_long_send(m.target_system, m.target_component,
                            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 0, 0, 0, 0, 0, 0, 0)
    return report()


def report():
    print()
    if fails:
        print("==== SITL HEADING+SPEED: FAIL ({}) ====".format(", ".join(fails)))
        print("Fix guided_hs_core's docstring and tools/task3_sim/world.py Boat to match")
        print("what was measured BEFORE the tree drives the boat.")
        return 1
    print("==== SITL HEADING+SPEED: PASS ====")
    return 0


if __name__ == "__main__":
    sys.exit(main())
