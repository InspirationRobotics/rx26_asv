"""aim_pin — hold the simulated boat where the aim test asks: its place and heading.

    python3 -m crusader_sim.aim_pin          # crusader_sim/scripts/aim_test.sh starts it

Runs in: WSL, next to Gazebo (gz-transport's Python bindings, no ROS: the crsd-sim
container has no gz Python), like crusader_sim.weather. It reads POSE_FILE, which
crusader_sim.aim_test writes from inside the container (~/robotx_ws is the
container's /root/robotx_ws), and puts the model there through /world/<w>/set_pose
PIN_HZ times a second: the place (x, y) and the heading it asks for, but the HEIGHT,
ROLL and PITCH the hull has of itself (/world/<w>/dynamic_pose/info). Forcing it
level fought the physics - the cannon swinging rocks the hull - so the autopilot's
attitude was a blur of the two, and the aim corrected for a tilt the water never
saw (2026-10-18). {"release": true} (or no file) lets go.

POSE_FILE: {"seq": N, "x": east m, "y": north m, "yaw_deg": ENU deg} (Gazebo's
world frame, the model's origin = the hull-bottom datum). It answers in ACK_FILE
with the seq it is holding, so the test knows the boat is there.
"""
import json
import math
import os
import signal
import sys
import threading
import time

WORLD = "crusader_sim"
MODEL = "crusader"
DIR = os.path.expanduser("~/robotx_ws/.aimtest")
POSE_FILE = os.path.join(DIR, "pose.json")
ACK_FILE = os.path.join(DIR, "pinned.json")
PIN_HZ = 20.0
DRAFT_M = 0.24          # crusader_hull.yaml draft_m: until Gazebo says where it floats


def read_pose():
    try:
        with open(POSE_FILE, encoding="utf-8") as f:
            d = json.load(f)
    except (OSError, ValueError):
        return None
    if d.get("release"):
        return None
    return d


def roll_pitch(q):
    """(roll, pitch) of a quaternion (w, x, y, z), radians, ZYX."""
    w, x, y, z = q
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    return roll, pitch


def quat(roll, pitch, yaw):
    """(w, x, y, z) from ZYX roll, pitch, yaw (radians)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def main():
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_pb2 import Pose
    from gz.msgs10.pose_v_pb2 import Pose_V
    from gz.transport13 import Node

    os.makedirs(DIR, exist_ok=True)
    node = Node()
    lock = threading.Lock()
    now_pose = {}                          # the hull's own: z, roll, pitch

    def on_poses(msg):
        for p in msg.pose:
            if p.name == MODEL:
                r, pt = roll_pitch((p.orientation.w, p.orientation.x, p.orientation.y,
                                    p.orientation.z))
                with lock:
                    now_pose.update(z=p.position.z, roll=r, pitch=pt)
                return

    node.subscribe(Pose_V, f"/world/{WORLD}/dynamic_pose/info", on_poses)
    stop = []
    signal.signal(signal.SIGTERM, lambda *_: stop.append(1))
    signal.signal(signal.SIGINT, lambda *_: stop.append(1))
    print(f"[aim_pin] holding the boat where {POSE_FILE} says", flush=True)
    last_seq = None
    while not stop:
        d = read_pose()
        if d is not None:
            with lock:
                z = now_pose.get("z", -DRAFT_M)
                roll, pitch = now_pose.get("roll", 0.0), now_pose.get("pitch", 0.0)
            p = Pose()
            p.name = MODEL
            p.position.x, p.position.y, p.position.z = float(d["x"]), float(d["y"]), z
            q = quat(roll, pitch, math.radians(float(d["yaw_deg"])))
            p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = q
            ok, _rep = node.request(f"/world/{WORLD}/set_pose", p, Pose, Boolean, 500)
            if ok and d.get("seq") != last_seq:
                last_seq = d.get("seq")
                with open(ACK_FILE + ".tmp", "w", encoding="utf-8") as f:
                    json.dump({"seq": last_seq}, f)
                os.replace(ACK_FILE + ".tmp", ACK_FILE)
                print(f"[aim_pin] pose {last_seq}: ({p.position.x:+.2f}, {p.position.y:+.2f}) "
                      f"yaw {float(d['yaw_deg']):.1f}", flush=True)
        time.sleep(1.0 / PIN_HZ)
    return 0


if __name__ == "__main__":
    sys.exit(main())
