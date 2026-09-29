"""check_motion — does the simulated Crusader move the way the boat's params say?

    python3 -m crusader_sim.check_motion            # against udp 14550 (tooling port)

Runs in: WSL, with gz_sim_up.sh already up. Talks MAVLink on the TOOLING port
(14550), never 14551 — that one is telemetry_bridge's.

Three MANUAL pulses on exactly the channels telemetry_bridge's /crsd/rc_override
path uses (override_channels [1, 3, 4] = steer, throttle, lateral), then one
GUIDED leg with the exact SET_POSITION_TARGET_GLOBAL_INT telemetry_bridge sends.
Each result is measured in the BODY frame at the start of the pulse, so the
verdict is about the mixer + thruster layout, not about where the boat pointed:

    throttle  +  ->  moves FORWARD
    lateral   +  ->  moves to STARBOARD         (only an omni frame can)
    steer     +  ->  yaws to STARBOARD (clockwise from above)
    GUIDED 20 m  ->  arrives within WP_RADIUS

A wrong sign here is the sim's version of "perfect in manual, spins in AUTO":
fix it in crusader_hull.yaml (pwm_sense / yaw_deg), never in the params.
"""
import argparse
import math
import os
import sys
import time

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

OVERRIDE_US = 150            # = telemetry_bridge override_max_us
PULSE_S = 6.0


def wait_msg(m, typ, timeout=5.0, cond=None):
    t_end = time.time() + timeout
    while time.time() < t_end:
        msg = m.recv_match(type=typ, blocking=True, timeout=0.5)
        if msg is not None and (cond is None or cond(msg)):
            return msg
    return None


def state(m):
    """The NEWEST position and attitude. recv_match hands back the OLDEST queued
    message, so after a few seconds of not reading, a plain recv returns the
    state from when we stopped reading — which is how the first run measured
    every MANUAL pulse as 0.00 m. Drain first, keep the last of each."""
    gp = att = None
    while True:
        msg = m.recv_match(type=["GLOBAL_POSITION_INT", "ATTITUDE"], blocking=False)
        if msg is None:
            break
        if msg.get_type() == "GLOBAL_POSITION_INT":
            gp = msg
        else:
            att = msg
    gp = gp or wait_msg(m, "GLOBAL_POSITION_INT", 3.0)
    att = att or wait_msg(m, "ATTITUDE", 3.0)
    if gp is None or att is None:
        return None
    return {"lat": gp.lat / 1e7, "lon": gp.lon / 1e7, "yaw": att.yaw,
            "hdg_gp": gp.hdg / 100.0 if gp.hdg != 65535 else None}


def ne_between(a, b):
    """metres north, east from a to b (flat earth — fine over tens of metres)"""
    dn = (b["lat"] - a["lat"]) * 111320.0
    de = (b["lon"] - a["lon"]) * 111320.0 * math.cos(math.radians(a["lat"]))
    return dn, de


def body(dn, de, yaw):
    """NE displacement -> (forward, starboard) at heading yaw (rad, from north)"""
    fwd = dn * math.cos(yaw) + de * math.sin(yaw)
    stb = -dn * math.sin(yaw) + de * math.cos(yaw)
    return fwd, stb


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def set_mode(m, name):
    m.set_mode(name)
    ok = wait_msg(m, "HEARTBEAT", 5.0,
                  lambda h: mavutil.mode_string_v10(h) == name) is not None
    return ok


def override(m, ch1=0, ch3=0, ch4=0):
    """0 = release that channel (MAVLink semantics); 65535 = ignore."""
    m.mav.rc_channels_override_send(m.target_system, m.target_component,
                                    ch1, 65535, ch3, ch4, 65535, 65535, 65535, 65535)


def settle(m, seconds=15.0, speed=0.05):
    """Wait until the boat has (nearly) stopped, so one pulse's coast does not
    land in the next pulse's measurement."""
    t_end = time.time() + seconds
    while time.time() < t_end:
        vfr = wait_msg(m, "VFR_HUD", 1.0)
        if vfr is not None and abs(vfr.groundspeed) < speed:
            return True
    return False


def pulse(m, label, **ch):
    settle(m)
    s0 = state(m)
    modes, texts, pwm_dev = set(), [], [0, 0, 0, 0]
    # yaw is INTEGRATED from successive ATTITUDE samples: at ~40 deg/s a 6 s
    # spin is ~250 deg, and an end-minus-start difference wraps that to -110
    # and calls a correct right turn a left one (it did, on the first run).
    last_yaw, turned = s0["yaw"], 0.0
    t_end = time.time() + PULSE_S
    while time.time() < t_end:
        override(m, **ch)
        t_tick = time.time() + 0.1
        while time.time() < t_tick:      # drain while we drive: see state()
            msg = m.recv_match(blocking=True, timeout=0.02)
            if msg is None:
                continue
            t = msg.get_type()
            if t == "HEARTBEAT" and msg.get_srcSystem() == m.target_system:
                modes.add(mavutil.mode_string_v10(msg))
            elif t == "SERVO_OUTPUT_RAW":
                for i in range(4):
                    d = getattr(msg, f"servo{i + 1}_raw") - 1500
                    if abs(d) > abs(pwm_dev[i]):
                        pwm_dev[i] = d
            elif t == "STATUSTEXT":
                texts.append(msg.text)
            elif t == "ATTITUDE":
                turned += wrap(msg.yaw - last_yaw)
                last_yaw = msg.yaw
    for _ in range(5):
        override(m)                      # release
        time.sleep(0.05)
    s1 = state(m)
    turned += wrap(s1["yaw"] - last_yaw)
    dn, de = ne_between(s0, s1)
    fwd, stb = body(dn, de, s0["yaw"])
    dyaw = math.degrees(turned)
    return {"label": label, "fwd": fwd, "stb": stb, "dyaw": dyaw,
            "modes": sorted(modes), "pwm_dev": pwm_dev, "texts": texts}


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--endpoint", default="udpin:127.0.0.1:14550")
    ap.add_argument("--guided-m", type=float, default=20.0)
    a = ap.parse_args()

    # source_system 255 = SYSID_MYGCS: ArduPilot drops RC_CHANNELS_OVERRIDE from
    # any other system id, silently. telemetry_bridge gets 255 as pymavlink's
    # default; a test on another id would "prove" the override path is dead.
    m = mavutil.mavlink_connection(a.endpoint, source_system=255)
    hb = m.wait_heartbeat(timeout=30)
    if hb is None:
        print("FAIL no HEARTBEAT on", a.endpoint)
        return 2
    print(f"connected: sys {m.target_system}, mode {mavutil.mode_string_v10(hb)}")
    m.mav.request_data_stream_send(m.target_system, m.target_component,
                                   mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)

    # EKF + GPS yaw: wait until the vehicle has a position and a heading
    t_end = time.time() + 90
    while time.time() < t_end:
        s = state(m)
        if s is not None and s["lat"] != 0:
            break
    else:
        print("FAIL no position after 90 s")
        return 2

    if not set_mode(m, "MANUAL"):
        print("FAIL could not enter MANUAL")
        return 2
    # Retry for up to a minute: right after boot the pre-arm checks refuse for
    # ~10-20 s ("Gyros inconsistent" while the consistency window fills, EKF
    # still converging) — the same wait a pilot sees on the dock.
    armed = None
    t_end = time.time() + 60
    while armed is None and time.time() < t_end:
        m.arducopter_arm()
        armed = wait_msg(m, "HEARTBEAT", 3.0,
                         lambda h: h.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
    if armed is None:
        print("FAIL did not arm. Recent STATUSTEXT:")
        for _ in range(10):
            st = wait_msg(m, "STATUSTEXT", 1.0)
            if st:
                print("   ", st.text)
        return 2
    print("armed in MANUAL")

    res = [pulse(m, "throttle +", ch3=1500 + OVERRIDE_US),
           pulse(m, "lateral  +", ch4=1500 + OVERRIDE_US),
           pulse(m, "steer    +", ch1=1500 + OVERRIDE_US)]
    print(f"{'pulse':12s} {'fwd m':>7s} {'stbd m':>7s} {'dyaw deg':>9s}  verdict")
    verdicts = []
    for r in res:
        if r["label"].startswith("throttle"):
            ok = r["fwd"] > 1.0 and abs(r["stb"]) < 0.5 * r["fwd"]
        elif r["label"].startswith("lateral"):
            ok = r["stb"] > 0.5 and abs(r["fwd"]) < r["stb"]
        else:
            ok = r["dyaw"] > 20.0
        verdicts.append(ok)
        print(f"{r['label']:12s} {r['fwd']:7.2f} {r['stb']:7.2f} {r['dyaw']:9.1f}  "
              f"{'PASS' if ok else 'FAIL'}   modes {r['modes']}  "
              f"SERVO1-4 peak dev {r['pwm_dev']} us")
        for t in r["texts"]:
            print(f"{'':14s}AP: {t}")

    # GUIDED: the exact message shape telemetry_bridge sends (position-only mask)
    s0 = state(m)
    tgt_lat = s0["lat"] + a.guided_m / 111320.0
    if not set_mode(m, "GUIDED"):
        print("FAIL could not enter GUIDED")
        return 2
    mask = 0b0000111111111000
    t_end = time.time() + 60
    best = None
    while time.time() < t_end:
        m.mav.set_position_target_global_int_send(
            0, m.target_system, m.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, mask,
            int(tgt_lat * 1e7), int(s0["lon"] * 1e7), 0, 0, 0, 0, 0, 0, 0, 0, 0)
        s = state(m)
        if s:
            dn, de = ne_between(s, {"lat": tgt_lat, "lon": s0["lon"]})
            d = math.hypot(dn, de)
            best = d if best is None else min(best, d)
            if d < 2.0:
                break
        time.sleep(0.2)
    g_ok = best is not None and best < 2.0
    verdicts.append(g_ok)
    print(f"GUIDED {a.guided_m:.0f} m north: closest {best:.2f} m  {'PASS' if g_ok else 'FAIL'}")
    set_mode(m, "HOLD")
    m.arducopter_disarm()
    print("ALL PASS" if all(verdicts) else "SOME FAILED")
    return 0 if all(verdicts) else 1


if __name__ == "__main__":
    sys.exit(main())
