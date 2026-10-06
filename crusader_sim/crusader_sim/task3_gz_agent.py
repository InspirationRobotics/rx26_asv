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
  {"stream": {"points": [[x,y,z]...], "hit": bool}}  -> a blue line in the GUI and
                                             a splash at its end (green on a hit)
  {"stream": null}                         -> the line is removed

Markers live in the GUI only (gz's MarkerManager): with --no-gui there is no
/marker service and the requests just time out, one at a time, off the main
loop. The sensors never see the water, as the real camera barely does.
"""
import argparse
import json
import socket
import threading
import time

from gz.transport13 import Node
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.empty_pb2 import Empty
from gz.msgs10.marker_pb2 import Marker
from gz.msgs10.scene_pb2 import Scene
from gz.msgs10.visual_pb2 import Visual

WORLD = "crusader_sim"
RGB = {"off": (0.03, 0.03, 0.03, 0.0), "red": (0.91, 0.16, 0.17, 1.0),
       "green": (0.05, 0.85, 0.05, 1.0), "blue": (0.0, 0.45, 0.9, 1.0)}
NS = "crusader_water"


class Agent:

    def __init__(self, dock, port):
        self.node = Node()
        self.dock = dock
        self.ids = {}
        self.shown = {}
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(("127.0.0.1", port))
        self.sock.settimeout(0.5)
        self._marker = None             # the newest marker job; older ones are dropped
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
            self._marker = ("del", None) if s is None else ("add", s)

    def _marker_loop(self):
        while True:
            with self._mlock:
                job, self._marker = self._marker, None
            if job is None:
                time.sleep(0.05)
                continue
            kind, s = job
            try:
                if kind == "del":
                    for mid in (1, 2):
                        m = Marker()
                        m.ns, m.id, m.action = NS, mid, Marker.DELETE_MARKER
                        self.node.request("/marker", m, Marker, Boolean, 300)
                    continue
                pts = s.get("points") or []
                if len(pts) < 2:
                    continue
                line = Marker()
                line.ns, line.id, line.action, line.type = NS, 1, Marker.ADD_MODIFY, Marker.LINE_STRIP
                line.lifetime.sec = 1
                for c in (line.material.ambient, line.material.diffuse):
                    c.r, c.g, c.b, c.a = 0.2, 0.55, 1.0, 1.0
                for x, y, z in pts:
                    p = line.point.add()
                    p.x, p.y, p.z = x, y, z
                self.node.request("/marker", line, Marker, Boolean, 300)
                dot = Marker()
                dot.ns, dot.id, dot.action, dot.type = NS, 2, Marker.ADD_MODIFY, Marker.SPHERE
                dot.lifetime.sec = 1
                dot.pose.position.x, dot.pose.position.y, dot.pose.position.z = pts[-1]
                dot.scale.x = dot.scale.y = dot.scale.z = 0.08
                hit = bool(s.get("hit"))
                for c in (dot.material.ambient, dot.material.diffuse):
                    c.r, c.g, c.b, c.a = (0.1, 1.0, 0.1, 1.0) if hit else (1.0, 1.0, 1.0, 1.0)
                self.node.request("/marker", dot, Marker, Boolean, 300)
            except Exception as e:           # a GUI that is not there must not stop the lights
                print(f"[gz_agent] marker: {e}", flush=True)
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
