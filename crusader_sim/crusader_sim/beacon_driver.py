"""beacon_driver — the course's FLASHING beacons blink in Gazebo, on sim time.

    python3 -m crusader_sim.beacon_driver COURSE [--world crusader_sim]

gen_world draws every beacon lit. This switches each flash_red / flash_green / flash_blue buoy's
LED band (and its up glow, where that one shows) between its colour and dark as
course.beacon_lit() says: FLASH_ON_S lit, FLASH_OFF_S dark, every buoy at its own phase. It is the
same function sim_camera labels its boxes by, so the Gazebo window, the rendered camera images
(SIM_DETECTOR=yolo) and the camera stand-in all see the same light at the same sim time.
steady_blue stays lit and off stays dark, as drawn.

gz-transport: /world/<w>/scene/info once for the visuals' ids, then /world/<w>/visual_config on
each switch (UserCommands). Runs on the WSL host beside `gz sim -s` (gz_sim_up.sh starts it; the
container has no gz Python bindings). The courses' "the beacon_driver node does the flashing"
comment was written before this existed (2026-11-13).
"""
import argparse
import sys
import threading
import time

from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.visual_pb2 import Visual
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.transport13 import Node

from crusader_sim import course as C
from crusader_sim import sdf_util as S
from crusader_sim.gen_world import BEACON_COLOUR

DARK = S.COLOURS["black"]


class Driver:
    def __init__(self, course_path, world):
        self.world = world
        self.node = Node()          # requests
        self.sub = Node()           # the stats subscription, apart from the requests
        self.stats = None           # (sim s, wall s, real-time factor) of the last /stats
        self.lock = threading.Lock()
        # the flashing buoys and which of their two lights gen_world drew lit
        self.flashing = {}
        for name, _x, _y, state, side_on, up_on in C.buoys(C.load(course_path)):
            if str(state).startswith("flash_") and BEACON_COLOUR.get(state):
                visuals = [v for v, on in (("beacon_side", side_on), ("beacon_up", up_on)) if on]
                if visuals:
                    self.flashing[name] = (state, S.COLOURS[BEACON_COLOUR[state]], visuals)
        # sim time from the world's STATISTICS (~5 Hz), carried forward between them at the real-time
        # factor. Not /world/<w>/clock: it comes every physics step, and that many Python callbacks
        # starved the visual_config requests (every one timed out, 2026-11-13)
        self.sub.subscribe(WorldStatistics, f"/world/{world}/stats", self._on_stats)

    def _on_stats(self, msg):
        with self.lock:
            self.stats = (msg.sim_time.sec + msg.sim_time.nsec * 1e-9, time.monotonic(),
                          max(0.0, min(2.0, msg.real_time_factor or 1.0)) if not msg.paused else 0.0)

    def now(self):
        """Sim time now (s), or None before the first /stats."""
        with self.lock:
            s = self.stats
        return None if s is None else s[0] + (time.monotonic() - s[1]) * s[2]

    def visual_ids(self, wait_s=90.0):
        """{(buoy, visual name): id} from the scene: once the buoy models are in it."""
        t0, got = time.time(), {}
        while time.time() - t0 < wait_s:
            ok, scene = self.node.request(f"/world/{self.world}/scene/info", Empty(), Empty, Scene, 3000)
            if ok:
                got = {}
                for m in scene.model:
                    if m.name not in self.flashing:
                        continue
                    for link in m.link:
                        for v in link.visual:
                            if v.name in self.flashing[m.name][2]:
                                got[(m.name, v.name)] = v.id
                if got:
                    missing = sorted(set(self.flashing) - {k[0] for k in got})
                    if missing:
                        print(f"beacon_driver: not in the scene, left lit: {missing}", flush=True)
                    return got
            time.sleep(1.0)
        return got

    def paint(self, vid, rgb):
        v = Visual()
        v.id = vid
        for c in (v.material.ambient, v.material.diffuse, v.material.emissive):
            c.r, c.g, c.b, c.a = rgb[0], rgb[1], rgb[2], 1.0
        v.material.specular.r = v.material.specular.g = v.material.specular.b = 0.1
        v.material.specular.a = 1.0
        ok, _rep = self.node.request(f"/world/{self.world}/visual_config", v, Visual, Boolean, 1000)
        return ok

    def run(self):
        if not self.flashing:
            print("beacon_driver: no flashing beacons on this course", flush=True)
            return 0
        print(f"beacon_driver: {len(self.flashing)} flashing beacons on the course; waiting for "
              f"/world/{self.world}/scene/info", flush=True)
        ids = self.visual_ids()
        if not ids:
            print(f"beacon_driver: no beacon visuals found in /world/{self.world}/scene/info", flush=True)
            return 1
        print(f"beacon_driver: {len(self.flashing)} flashing buoys, {len(ids)} lights, "
              f"{C.FLASH_ON_S:g} s on / {C.FLASH_OFF_S:g} s off on sim time", flush=True)
        shown, fails = {}, 0
        while True:
            t = self.now()
            if t is not None:
                for name, (state, rgb, visuals) in self.flashing.items():
                    lit = C.beacon_lit(state, name, t)
                    if shown.get(name) == lit:
                        continue
                    ok = all([self.paint(ids[(name, v)], rgb if lit else DARK)
                              for v in visuals if (name, v) in ids])
                    if ok:
                        shown[name] = lit
                    else:
                        fails += 1
                        if fails in (10, 100, 1000):      # the first can time out on discovery
                            print(f"beacon_driver: visual_config failed ({fails} so far)", flush=True)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("course", help="a courses/*.yaml name or path")
    ap.add_argument("--world", default="crusader_sim")
    a = ap.parse_args()
    sys.exit(Driver(C.resolve(a.course), a.world).run())


if __name__ == "__main__":
    main()
