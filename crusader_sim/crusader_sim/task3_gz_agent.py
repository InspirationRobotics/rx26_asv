"""task3_gz_agent — the Gazebo-side hands of task3_world: the dock's lights and the water.

    python3 -m crusader_sim.task3_gz_agent [--dock dock] [--port 14558]

Runs in: WSL (host python3 + gz-transport's Python bindings, no ROS), started by
gz_sim_up.sh for a course with a dock and stopped by gz_sim_down.sh.

task3_world (ROS, in crsd-sim) decides what RoboCommand's windows show and where
the water goes; the container has no gz-transport bindings, so it says so in
JSON datagrams to udp 127.0.0.1:<port> and this does it:

  {"lights": {"bay2_win1": "red", ...}}   -> /world/<w>/visual_config on that visual
                                             (diffuse + emissive): the GUI AND the
                                             rendered OAK-D see it
  {"stream": {"points": [[x,y,z]...], "hit": bool}}  -> droplets flowing along the
                                             arc in the GUI, and a splash at its end
                                             (green on a hit)
  {"stream": null}                         -> the water is removed

THE WATER is a train of small glowing spheres, re-spaced along the arc and moved
along it every frame, so it reads as a jet from any angle. They are real (static,
visual-only) models, made once and parked under the water, and moved with
/world/<w>/set_pose_vector: one request a frame. Not GUI markers: the GUI's
/marker service is ONE-WAY (it answers nothing), which gz-transport's Python
cannot call - every request timed out, and no water was drawn until 2026-10-12.
The droplets carry the LiDAR's hidden visibility bit (gen_crusader.LIDAR_HIDDEN),
have no collision and do not move anything; the rendered cameras see them, as a
real camera sees the jet. An arc older than STALE_S is parked again, so the water
stops in the GUI when the pump does even if a "null" is lost.
"""
import argparse
import json
import math
import socket
import threading
import time

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.entity_factory_v_pb2 import EntityFactory_V
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.visual_pb2 import Visual

WORLD = "crusader_sim"
RGB = {"off": (0.03, 0.03, 0.03, 0.0), "red": (0.91, 0.16, 0.17, 1.0),
       "green": (0.05, 0.85, 0.05, 1.0), "blue": (0.0, 0.45, 0.9, 1.0)}
DROP_M = 0.09        # droplet spacing along the arc [m]
DROP_R = 0.022       # droplet radius [m]
N_DROPS = 45         # enough for a 4 m arc
FLOW_MPS = 2.0       # how fast the droplets move along the arc in the GUI (looks, not physics)
FRAME_S = 0.066      # redraw period
STALE_S = 0.4        # an arc older than this is parked
PARK_Z = -10.0       # under the water: out of sight
LIDAR_HIDDEN = 2     # = gen_crusader.LIDAR_HIDDEN (the LiDAR's visibility mask skips it)
DROP = "crusader_water_drop_{}"
SPLASH, SPLASH_HIT = "crusader_water_splash", "crusader_water_splash_hit"


def droplet_sdf(name, radius, rgb, glow):
    r, g, b = rgb
    return (f"<sdf version='1.9'><model name='{name}'><static>true</static>"
            f"<pose>0 0 {PARK_Z} 0 0 0</pose><link name='l'><visual name='v'>"
            f"<geometry><sphere><radius>{radius}</radius></sphere></geometry>"
            f"<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>"
            f"<emissive>{r * glow} {g * glow} {b * glow} 1</emissive></material>"
            f"<cast_shadows>false</cast_shadows><visibility_flags>{LIDAR_HIDDEN}</visibility_flags>"
            f"</visual></link></model></sdf>")


def resample(points, spacing, offset):
    """Points every `spacing` metres along a polyline, starting `offset` in."""
    out = []
    if len(points) < 2:
        return out
    want = offset
    walked = 0.0
    for a, b in zip(points, points[1:]):
        seg = math.dist(a, b)
        while seg > 1e-9 and want <= walked + seg:
            f = (want - walked) / seg
            out.append(tuple(a[i] + f * (b[i] - a[i]) for i in range(3)))
            want += spacing
        walked += seg
    return out


class Agent:

    def __init__(self, dock, port):
        self.node = Node()
        self.dock = dock
        self.ids = {}
        self.shown = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.5)
        self._marker = None             # (the newest arc, when it came); older ones are dropped
        self._mlock = threading.Lock()
        threading.Thread(target=self._marker_loop, daemon=True).start()

    def find_visuals(self):
        """The dock's window visuals' entity ids, from the scene (retried until the world is up)."""
        while not self.ids:
            ok, sc = self.node.request(f"/world/{WORLD}/scene/info", Empty(), Empty, Scene, 3000)
            if ok:
                for m in sc.model:
                    if m.name == self.dock:
                        for ln in m.link:
                            for v in ln.visual:
                                if "_win" in v.name and not v.name.endswith("_border"):
                                    self.ids[v.name] = v.id
            if not self.ids:
                print(f"[gz_agent] waiting for the {self.dock!r} model in /world/{WORLD}", flush=True)
                time.sleep(2.0)
        print(f"[gz_agent] {len(self.ids)} window visuals: {sorted(self.ids)}", flush=True)

    def light(self, name, colour):
        vid = self.ids.get(name)
        if vid is None or self.shown.get(name) == colour:
            return
        r, g, b, e = RGB.get(colour, RGB["off"])
        v = Visual()
        v.id = vid
        v.name = name
        for c in (v.material.diffuse, v.material.ambient):
            c.r, c.g, c.b, c.a = r, g, b, 1.0
        v.material.emissive.r, v.material.emissive.g, v.material.emissive.b = r * e, g * e, b * e
        v.material.emissive.a = 1.0
        ok, rep = self.node.request(f"/world/{WORLD}/visual_config", v, Visual, Boolean, 2000)
        if ok:
            self.shown[name] = colour
            print(f"[gz_agent] {name} -> {colour}", flush=True)
        else:
            print(f"[gz_agent] visual_config {name} -> {colour}: no answer", flush=True)

    # --- the water, in the GUI ---
    def stream(self, s):
        with self._mlock:
            self._marker = None if s is None else (s, time.monotonic())

    def _make_droplets(self):
        """The droplet models, parked; True once they exist (made here or earlier)."""
        fv = EntityFactory_V()
        for k in range(N_DROPS):
            fv.data.add().sdf = droplet_sdf(DROP.format(k), DROP_R, (0.55, 0.82, 1.0), 0.6)
        fv.data.add().sdf = droplet_sdf(SPLASH, 0.07, (1.0, 1.0, 1.0), 0.8)
        fv.data.add().sdf = droplet_sdf(SPLASH_HIT, 0.07, (0.1, 1.0, 0.1), 0.8)
        ok, _ = self.node.request(f"/world/{WORLD}/create_multiple", fv, EntityFactory_V, Boolean, 5000)
        return ok

    def _place(self, where):
        """Move droplets: {model name: (x, y, z)}; every other one is parked."""
        pv = Pose_V()
        for name in [DROP.format(k) for k in range(N_DROPS)] + [SPLASH, SPLASH_HIT]:
            x, y, z = where.get(name, (0.0, 0.0, PARK_Z))
            p = pv.pose.add()
            p.name = name
            p.position.x, p.position.y, p.position.z = x, y, z
            p.orientation.w = 1.0
        ok, _ = self.node.request(f"/world/{WORLD}/set_pose_vector", pv, Pose_V, Boolean, 300)
        return ok

    def _marker_loop(self):
        while not self.ids:                    # the world first (find_visuals)
            time.sleep(0.5)
        while not self._make_droplets():
            print("[gz_agent] could not make the water droplets yet; retrying", flush=True)
            time.sleep(3.0)
        print(f"[gz_agent] water: {N_DROPS} droplets ready", flush=True)
        shown = False
        while True:
            t0 = time.monotonic()
            with self._mlock:
                job = self._marker
            try:
                pts = [] if job is None or t0 - job[1] > STALE_S else                     [tuple(p) for p in (job[0].get("points") or [])]
                if len(pts) < 2:
                    if shown:
                        self._place({})
                        shown = False
                    time.sleep(0.05)
                    continue
                drops = resample(pts, DROP_M, (t0 * FLOW_MPS) % DROP_M)[:N_DROPS]
                where = {DROP.format(k): q for k, q in enumerate(drops)}
                where[SPLASH_HIT if job[0].get("hit") else SPLASH] = pts[-1]
                shown = self._place(where)
                time.sleep(max(0.0, FRAME_S - (time.monotonic() - t0)))
            except Exception as e:           # the water must never stop the lights
                print(f"[gz_agent] water: {e}", flush=True)
                time.sleep(1.0)

    def run(self):
        self.find_visuals()
        for name in self.ids:                 # dark until RoboCommand says otherwise
            self.light(name, "off")
        print("[gz_agent] ready", flush=True)
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except socket.timeout:
                continue
            try:
                msg = json.loads(data.decode())
            except ValueError:
                continue
            for name, colour in (msg.get("lights") or {}).items():
                self.light(name, colour)
            if "stream" in msg:
                self.stream(msg["stream"])


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--dock", default="dock", help="the dock model's name in the world")
    ap.add_argument("--port", type=int, default=14558)
    a = ap.parse_args()
    Agent(a.dock, a.port).run()


if __name__ == "__main__":
    main()
