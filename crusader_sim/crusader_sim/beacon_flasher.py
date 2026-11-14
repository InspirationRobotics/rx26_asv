"""beacon_flasher — makes the world's FLASHING beacons flash in the render. Runs in: WSL, beside gz sim.

    python3 -m crusader_sim.beacon_flasher WORLD.sdf [--period 2.0]

gen_world draws a flashing beacon as a black band plus a lit band in its own model
"<buoy>_led". This moves every *_led model to its place for the ON half of each period and
LED_OFF_Z under water for the OFF half (3.3.2: 1 s on, 1 s off, oak_detector_core's
flash_period_s). Each buoy gets its own phase, from its name, so they do not blink in step.
A pose change is what reaches the camera sensors; a material change only reaches the GUI.
"""
import argparse
import math
import os
import re
import sys
import time
import zlib

LED_OFF_Z = -3.0
MODEL_RE = re.compile(r'<model name="([^"]+_led)"><static>true</static>'
                      r'<pose>([-\d.]+) ([-\d.]+) ([-\d.]+) [-\d.]+ [-\d.]+ ([-\d.]+)</pose>')


def leds(world_text):
    """[(name, x, y, z, yaw)] of every *_led model in the world SDF."""
    return [(m[0], *map(float, m[1:])) for m in MODEL_RE.findall(world_text)]


def is_on(name, t, period):
    """ON for the first half of each period, with a per-buoy phase."""
    phase = (zlib.crc32(name.encode()) % 1000) / 1000.0 * period
    return ((t + phase) % period) < period / 2.0


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("world")
    ap.add_argument("--world-name", default="crusader_sim")
    ap.add_argument("--period", type=float, default=2.0)
    a = ap.parse_args()
    with open(a.world, encoding="utf-8") as f:
        models = leds(f.read())
    if not models:
        print("beacon_flasher: no flashing beacons in", a.world)
        return 0
    os.environ.setdefault("GZ_PARTITION", "crusader_sim")
    from gz.transport13 import Node
    from gz.msgs10.boolean_pb2 import Boolean
    from gz.msgs10.pose_v_pb2 import Pose_V

    node = Node()
    svc = f"/world/{a.world_name}/set_pose_vector"
    print(f"beacon_flasher: {len(models)} beacons, period {a.period} s", flush=True)
    state = {}
    while True:
        t = time.monotonic()
        req = Pose_V()
        for name, x, y, z, yaw in models:
            on = is_on(name, t, a.period)
            if state.get(name) == on:
                continue
            p = req.pose.add()
            p.name = name
            p.position.x, p.position.y, p.position.z = x, y, (z if on else LED_OFF_Z)
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            state[name] = on
        if req.pose:
            ok, rep = node.request(svc, req, Pose_V, Boolean, 1000)
            if not (ok and rep.data):
                for p in req.pose:          # retry next tick
                    state.pop(p.name, None)
        time.sleep(0.05)


if __name__ == "__main__":
    sys.exit(main())
