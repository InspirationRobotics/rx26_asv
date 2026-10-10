#!/usr/bin/env python3
"""cannon_cal.py - calibrate the pan/tilt water cannon's aim on the boat.

Runs INSIDE the asv container (or crsd-sim), next to telemetry_bridge. The
procedure is docs/T3_cannon_cal.md; this is the tool it uses.

    cannon_cal.py node                 (re)start cannon_aim_node on crusader_params.yaml, pump DRY
    cannon_cal.py pwm PAN_US TILT_US   hold the two servos at these PWMs
    cannon_cal.py deg PAN TILT         ...at these angles, through the file's servo maps (no trims)
    cannon_cal.py point X Y Z          aim at a tape-measured point until Ctrl-C (m from the
                                       camera's lens along the HULL: X forward, Y left, Z up)
    cannon_cal.py window lr|ul [--bay N]   aim at the window dock_view sees until Ctrl-C
    cannon_cal.py set KEY VALUE        write a value into crusader_params.yaml (trims also go live)
    cannon_cal.py speed D H TILT       exit speed from a hit on a vertical board
    cannon_cal.py show                 the aim numbers in crusader_params.yaml

ONE FILE. `set` writes straight into crusader_params.yaml (the installed one is
a link to the source tree's), the same file the ground station's Tuning tab
saves into, so a value kept from either place is the one the next start uses.
`git diff` on the Jetson shows what a session changed; commit what is right.
(Until 2026-10-09 this worked on a copy, t3tools/cannon_cal.yaml, and nothing
reached the boat's file until copied by hand.)

THE TRIMS are per window (pan_trim_ul_deg ... tilt_trim_lr_deg): `window ul|lr`
names its window, so cannon_aim_node adds that window's trims. `point` and
`deg` name none and are untrimmed.

NO WATER FROM HERE. cannon_aim_node always runs with fire_pump false: its
bursts are logged DRY. The water is the pilot's pump switch (ch10), which works
whatever G7 says. The cannon MOVES whenever `pwm`, `deg`, `point` or `window`
runs: keep hands and faces clear of the nozzle.
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

LOGDIR = "/root/robotx_ws/t3tools"
INSTALLED = "/root/robotx_ws/install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml"
NODE_LOG = os.path.join(LOGDIR, "cannon_cal_node.log")
G = 9.81

# the keys the aim uses, and the sections they live in
AIM_KEYS = ("nozzle_x", "nozzle_y", "nozzle_z", "exit_speed_mps", "throw_range_m",
            "throw_elev_deg", "pan_center_us", "pan_us_per_deg", "pan_sign",
            "tilt_center_us", "tilt_us_per_deg", "tilt_sign", "pan_min_deg", "pan_max_deg",
            "tilt_min_deg", "tilt_max_deg",
            "pan_trim_ul_deg", "tilt_trim_ul_deg", "pan_trim_lr_deg", "tilt_trim_lr_deg")
CAM_KEYS = ("cam_x", "cam_y", "cam_z", "cam_yaw_deg", "cam_pitch_deg")
LIVE_KEYS = ("pan_trim_ul_deg", "tilt_trim_ul_deg", "pan_trim_lr_deg", "tilt_trim_lr_deg")


# ------------------------------------------------------------------ pure maths

def exit_speed(d, h, tilt_deg):
    """Exit speed [m/s] of a drag-free stream that, leaving at tilt_deg above
    level, hits a vertical board d metres away horizontally at h metres above
    the nozzle. None if no speed can (the board is above the tilt line)."""
    t = math.radians(tilt_deg)
    den = 2.0 * math.cos(t) ** 2 * (d * math.tan(t) - h)
    if d <= 0 or den <= 0:
        return None
    return math.sqrt(G * d * d / den)


def hull_to_camera(dx, dy, dz, cam_pitch_deg, cam_yaw_deg):
    """An offset from the camera measured along the HULL's axes (x forward,
    y left, z up) -> camera_link (x along the optical axis). The inverse of
    cannon_aim_core.cam_to_body's rotation."""
    pr, yr = math.radians(cam_pitch_deg), math.radians(cam_yaw_deg)
    xl = dx * math.cos(yr) + dy * math.sin(yr)          # undo the yaw
    y = -dx * math.sin(yr) + dy * math.cos(yr)
    return (xl * math.cos(pr) - dz * math.sin(pr), y, xl * math.sin(pr) + dz * math.cos(pr))


# ------------------------------------------------------------------ the working copy

def ensure_copy():
    """The params file every value is read from and written to (the name is
    from when this was a working copy)."""
    os.makedirs(LOGDIR, exist_ok=True)
    return INSTALLED


def section_params(path, section):
    from crusader_common import config as crsd_config
    return dict(crsd_config.node_params(section, path=path))


def aim_params(path):
    """cannon_aim_node's numbers with the camera's mount, as the node builds them."""
    d = section_params(path, "cannon_aim_node")
    cam = section_params(path, "target_tracker")
    for k in CAM_KEYS:
        d[k] = cam.get(k, 0.0)
    return d


def set_in_yaml(path, section, key, value):
    """Replace `key: <value>` inside the top-level `section:` of a params YAML,
    keeping the line's comment. False if the key is not in that section."""
    lines = open(path, encoding="utf-8").read().split("\n")
    inside, done = False, False
    pat = re.compile(r"^(\s+" + re.escape(key) + r":\s*)([^#]*?)(\s*#.*)?$")
    for i, ln in enumerate(lines):
        if re.match(r"^\S", ln):
            inside = ln.startswith(section + ":")
            continue
        if inside:
            m = pat.match(ln)
            if m:
                old = m.group(2)
                new = str(value)
                comment = m.group(3) or ""
                pad = max(1, len(old) + (len(comment) - len(comment.lstrip())) - len(new))
                lines[i] = m.group(1) + new + (" " * pad + comment.lstrip() if comment else "")
                done = True
                break
    if done:
        open(path, "w", encoding="utf-8").write("\n".join(lines))
    return done


# ------------------------------------------------------------------ ROS helpers

def ros_node(name):
    import rclpy
    try:                                       # Ctrl-C is ours: aim_loop sends "fire false"
        from rclpy.signals import SignalHandlerOptions
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    except ImportError:
        rclpy.init()
    return rclpy.create_node(name)


def spin_for(node, seconds):
    import rclpy
    t0 = time.monotonic()
    while time.monotonic() - t0 < seconds:
        rclpy.spin_once(node, timeout_sec=0.05)


def publish_pwm(pan_us, tilt_us, source, pan_deg=0.0, tilt_deg=0.0):
    """Send one pan/tilt PWM to telemetry_bridge and report what the Pixhawk
    then puts on the two outputs."""
    from crusader_msgs.msg import CannonCommand, CannonState
    n = ros_node("cannon_cal")
    pub = n.create_publisher(CannonCommand, "/crsd/cannon_cmd", 10)
    state = {}
    n.create_subscription(CannonState, "/crsd/cannon_state", lambda m: state.update(s=m), 10)
    t0 = time.monotonic()                         # discovery: wait for the bridge
    while time.monotonic() - t0 < 5.0 and (pub.get_subscription_count() == 0 or "s" not in state):
        spin_for(n, 0.1)
    if pub.get_subscription_count() == 0:
        print("[cal] nobody listens on /crsd/cannon_cmd: is telemetry_bridge running (this branch)?")
        return
    for _ in range(3):
        m = CannonCommand()
        m.header.stamp = n.get_clock().now().to_msg()
        m.pan_pwm, m.tilt_pwm, m.source = int(pan_us), int(tilt_us), source
        m.pan_deg, m.tilt_deg = float(pan_deg), float(tilt_deg)
        pub.publish(m)
        spin_for(n, 0.2)
    t0 = time.monotonic()                         # until the outputs say so, or 3 s
    while time.monotonic() - t0 < 3.0:
        spin_for(n, 0.1)
        s = state.get("s")
        if s is not None and s.output_fresh and abs(int(s.pan_pwm) - pan_us) <= 12                 and abs(int(s.tilt_pwm) - tilt_us) <= 12:
            break
    s = state.get("s")
    if s is None:
        print("[cal] no /crsd/cannon_state: is telemetry_bridge running (this branch)?")
    elif not s.enabled:
        print("[cal] the bridge has no cannon path (cannon_pan/tilt_channel 0)")
    else:
        print(f"[cal] asked pan {pan_us} tilt {tilt_us} us | the Pixhawk outputs "
              f"pan {s.pan_pwm} tilt {s.tilt_pwm} us ({'fresh' if s.output_fresh else 'STALE'})"
              f"{' | ' + s.last_reason if s.last_reason else ''}")


def aim_loop(get_point, label, window=None):
    """Publish {"fire": true, x, y, z} at 10 Hz while get_point() has one, and
    print what cannon_aim_node does with it, until Ctrl-C. `window` ("UL"/"LR")
    names the window, so the node adds that window's own tilt trim."""
    from std_msgs.msg import String
    n = ros_node("cannon_cal")
    pub = n.create_publisher(String, "/crsd/water_cannon", 10)
    st = {}
    n.create_subscription(String, "/crsd/cannon_status",
                          lambda m: st.update(j=json.loads(m.data), t=time.monotonic()), 10)
    stop = {"now": False}
    signal.signal(signal.SIGINT, lambda *a: stop.update(now=True))
    signal.signal(signal.SIGTERM, lambda *a: stop.update(now=True))
    print(f"[cal] aiming at {label}. Ctrl-C to stop. Water = the pilot's pump switch, "
          "only when it says ON TARGET.")
    last_print = 0.0
    while not stop["now"]:
        p = get_point(n)
        msg = {"fire": True, "frame_id": "camera_link"}
        if window:
            msg["window"] = window
        if p is not None:
            msg.update(x=p[0], y=p[1], z=p[2])
        pub.publish(String(data=json.dumps(msg)))
        spin_for(n, 0.1)
        now = time.monotonic()
        if now - last_print >= 1.0:
            last_print = now
            j = st.get("j")
            if j is None or now - st.get("t", 0) > 1.0:
                print("[cal] no /crsd/cannon_status: run `cal node` first")
                continue
            out = j.get("servo_out")
            on = (out is not None and abs(out[0] - j["pan_pwm"]) <= 12
                  and abs(out[1] - j["tilt_pwm"]) <= 12)
            aim = j.get("aim_cam")
            print(f"[cal] {'ON TARGET' if on and j.get('reachable') else 'moving   '} | "
                  f"pan {j['pan_deg']:+6.1f} tilt {j['tilt_deg']:+6.1f} deg = "
                  f"{j['pan_pwm']}/{j['tilt_pwm']} us, servos {out} | "
                  f"target {j.get('dist_m')} m out {j.get('height_m')} m up, throw "
                  f"{j.get('elev_deg')} deg"
                  + ("" if j.get("reachable", True) else " OUT OF REACH")
                  + (" CLIPPED" if j.get("clipped") else "")
                  + ("" if p is not None else " | camera: window NOT SEEN")
                  + (f" | aim {aim}" if aim else ""))
    for _ in range(5):
        pub.publish(String(data=json.dumps({"fire": False})))
        spin_for(n, 0.05)
    print("[cal] stopped (fire false sent). The cannon holds where it is.")


# ------------------------------------------------------------------ commands

def cmd_node(a):
    path = ensure_copy()
    subprocess.call(["pkill", "-f", "[c]rusader_fcu/cannon_aim_node"])
    time.sleep(1.0)
    env = dict(os.environ, CRUSADER_PARAMS=path)
    with open(NODE_LOG, "w") as log:
        subprocess.Popen(["ros2", "run", "crusader_fcu", "cannon_aim_node", "--ros-args",
                          "-p", "fire_pump:=false"],
                         env=env, stdout=log, stderr=subprocess.STDOUT, stdin=subprocess.DEVNULL,
                         start_new_session=True)
    time.sleep(4.0)
    if subprocess.call(["pgrep", "-f", "[/]cannon_aim_node"], stdout=subprocess.DEVNULL) != 0:
        print(f"[cal] cannon_aim_node did not stay up; {NODE_LOG}:")
        print(open(NODE_LOG).read()[-1500:])
        return 1
    banner = [ln for ln in open(NODE_LOG).read().splitlines() if "cannon aim:" in ln]
    print(f"[cal] cannon_aim_node up on {path} (pump DRY)")
    if banner:
        print("[cal] " + banner[-1].split("]: ", 1)[-1])
    return 0


def cmd_pwm(a):
    publish_pwm(a.pan_us, a.tilt_us, "cannon_cal pwm")
    return 0


def cmd_deg(a):
    from crusader_fcu import cannon_aim_core as ca
    cp = ca.CannonParams.from_dict(aim_params(ensure_copy()))
    pan_us, tilt_us = cp.pan.to_pwm(a.pan), cp.tilt.to_pwm(a.tilt)
    print(f"[cal] pan {a.pan:+.1f} deg -> {pan_us} us, tilt {a.tilt:+.1f} deg -> {tilt_us} us "
          f"(the working copy's maps; trims NOT added)")
    publish_pwm(pan_us, tilt_us, "cannon_cal deg", a.pan, a.tilt)
    return 0


def cmd_point(a):
    d = aim_params(ensure_copy())
    c = hull_to_camera(a.x, a.y, a.z, float(d["cam_pitch_deg"]), float(d["cam_yaw_deg"]))
    print(f"[cal] {a.x:.3f} fwd {a.y:+.3f} left {a.z:+.3f} up from the lens = camera_link "
          f"({c[0]:.3f}, {c[1]:+.3f}, {c[2]:+.3f}) with the camera at pitch "
          f"{float(d['cam_pitch_deg']):+.1f} deg")
    aim_loop(lambda n: c, f"the point {a.x:.2f} m ahead, {a.y:+.2f} m left, {a.z:+.2f} m up")
    return 0


def cmd_window(a):
    from crusader_msgs.msg import DockObservation
    want = a.which.upper()
    seen = {}

    def on_obs(m):
        bays = list(m.bays)
        if a.bay is not None:
            bays = [b for b in bays if b.bay_index == a.bay]
        best = None
        for b in bays:
            for w in b.windows:
                if w.slot.upper() == want and w.has_position:
                    if best is None or abs(w.y) < abs(best[1]):
                        best = (w.x, w.y, w.z)
        if best is not None:
            seen.update(p=best, t=time.monotonic())

    subscribed = {}

    def get_point(n):
        if not subscribed:
            n.create_subscription(DockObservation, a.topic, on_obs, 10)
            subscribed["yes"] = True
        if "p" in seen and time.monotonic() - seen["t"] <= 0.5:
            return seen["p"]
        return None

    aim_loop(get_point, f"the {want} window on {a.topic}", window=want)
    return 0


def cmd_set(a):
    path = ensure_copy()
    key, val = a.key, a.value
    try:
        float(val)
    except ValueError:
        print(f"[cal] {key}: a number, please (got {val!r})")
        return 1
    if key in CAM_KEYS:
        ok = [s for s in ("target_tracker", "bt_runner_node") if set_in_yaml(path, s, key, val)]
        if not ok:
            print(f"[cal] {key} not found in target_tracker / bt_runner_node")
            return 1
        print(f"[cal] {key} = {val} in {', '.join(ok)} ({path})")
    elif key in AIM_KEYS:
        if not set_in_yaml(path, "cannon_aim_node", key, val):
            print(f"[cal] {key} not found in cannon_aim_node")
            return 1
        print(f"[cal] {key} = {val} in cannon_aim_node ({path})")
    else:
        print(f"[cal] not an aim key: {key}. Keys: {', '.join(AIM_KEYS + CAM_KEYS)}")
        return 1
    if key in LIVE_KEYS:
        r = subprocess.call(["ros2", "param", "set", "/cannon_aim_node", key, str(float(val))],
                            stdout=subprocess.DEVNULL)
        print("[cal] ...and live on cannon_aim_node" if r == 0 else
              "[cal] (not live: is cannon_aim_node running? `cal node`)")
    else:
        print("[cal] restart the node to use it: cal node")
    return 0


def cmd_speed(a):
    v = exit_speed(a.d, a.h, a.tilt)
    if v is None:
        print("[cal] no drag-free stream does that: the hit is above the line the nozzle points "
              "along. Check the numbers (H is the hit's height ABOVE THE NOZZLE).")
        return 1
    print(f"[cal] exit speed {v:.2f} m/s (= a {v * v / G:.2f} m throw at 45 deg on the level)")
    print(f"[cal] to keep:  cal set exit_speed_mps {v:.2f}   (then: cal node)")
    return 0


def cmd_show(a):
    path = ensure_copy()
    mine = aim_params(path)
    print(f"[cal] the aim numbers in {path} (git diff on the Jetson: what changed):")
    for k in CAM_KEYS + AIM_KEYS:
        print(f"  {k:18s} {str(mine.get(k)):>10s}")
    return 0


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    sp = ap.add_subparsers(dest="cmd", required=True)
    sp.add_parser("node").set_defaults(f=cmd_node)
    p = sp.add_parser("pwm"); p.add_argument("pan_us", type=int); p.add_argument("tilt_us", type=int)
    p.set_defaults(f=cmd_pwm)
    p = sp.add_parser("deg"); p.add_argument("pan", type=float); p.add_argument("tilt", type=float)
    p.set_defaults(f=cmd_deg)
    p = sp.add_parser("point")
    for k in ("x", "y", "z"):
        p.add_argument(k, type=float)
    p.set_defaults(f=cmd_point)
    p = sp.add_parser("window"); p.add_argument("which", choices=["lr", "ul", "LR", "UL"])
    p.add_argument("--bay", type=int, default=None, help="bay_index (left to right in the frame)")
    p.add_argument("--topic", default="/dock/observations")
    p.set_defaults(f=cmd_window)
    p = sp.add_parser("set"); p.add_argument("key"); p.add_argument("value")
    p.set_defaults(f=cmd_set)
    p = sp.add_parser("speed")
    p.add_argument("d", type=float, help="board's horizontal distance from the nozzle pivot [m]")
    p.add_argument("h", type=float, help="the hit's height above the nozzle pivot [m] (- = below)")
    p.add_argument("tilt", type=float, help="the nozzle's MEASURED tilt above level [deg]")
    p.set_defaults(f=cmd_speed)
    sp.add_parser("show").set_defaults(f=cmd_show)
    a = ap.parse_args(argv)
    return a.f(a)


if __name__ == "__main__":
    sys.exit(main())
