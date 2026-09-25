#!/usr/bin/env python3
"""sim.py — the Task 3 simulator: the world, the REAL tree, and its own page.

    python tools/task3_sim/build.py         # once: BT.CPP, the tests, the runner
    python tools/task3_sim/sim.py           # then open http://localhost:8088

No ROS, no WSL, no Docker. Plain Python 3.10+ and the runner build.py made.

    world.py  (this process)                offros_runner (child process)
    course, boat, camera, lights,   JSON    bt_runner_node's loop, the SAME
    RoboCommand, the UAV        <-------->  leaves.cpp + task3_leaves.cpp +
                                  lines     behavior_trees/task3_disruptive.xml

The page is its own server on :8088, deliberately NOT a tab of the ground
station (:8090). The ground station is the boat's operator page and runs on
the Jetson; this is a desk tool that invents a world, and the two should never
be one click apart on competition day. 8088 is clear of 8090 (ground station),
8086 (Task 1's aircraft page), 8085 (bt_view), 8080/8081 (camera/LiDAR) - and
of 8087, which rx26_uav's sim_ekko.py already takes, and 8091-8093 (the UAV's).

REAL TIME, NOT FASTER. The tree's timers - Sleep, Timeout, every "resend
every 10 s" - read the wall clock, so the world must too. A Disruptive run
takes about two minutes.

    --headless   no server: run one mission, print the judge's verdict as
                 JSON, exit 0 only if every report was right. test_e2e.py
                 drives it this way.
"""
import argparse
import http.server
import json
import os
import queue
import random
import subprocess
import sys
import threading
import time
from collections import deque
from dataclasses import asdict, fields

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, "..", ".."))
sys.path.insert(0, HERE)

import world as W                                         # noqa: E402

RUNNER = os.path.join(HERE, ".build", "offros_runner" + (".exe" if os.name == "nt" else ""))
TREE = os.path.join(REPO, "crusader_bt", "behavior_trees", "task3_disruptive.xml")
PAGE = os.path.join(HERE, "page.html")
PORT = 8088

OUTCOMES = {0: "SUCCESS", 1: "TIMEOUT", 2: "TREE FAILED", 3: "CANCELLED",
            4: "NOT AUTONOMOUS", 5: "FAULT", -1: "REJECTED"}


class RunnerProc:
    """The off-ROS runner as a child: JSON lines in, JSON lines and logs out."""

    def __init__(self, exe, tree):
        if not os.path.isfile(exe):
            sys.exit("no runner at %s - run: python tools/task3_sim/build.py" % exe)
        self.p = subprocess.Popen(
            [exe, "--tree", tree, "--publish-setpoints"],
            stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1, encoding="utf-8", errors="replace")
        self.out = queue.Queue()
        self.logs = queue.Queue()
        self._wlock = threading.Lock()
        threading.Thread(target=self._pump, args=(self.p.stdout, self.out, True), daemon=True).start()
        threading.Thread(target=self._pump, args=(self.p.stderr, self.logs, False), daemon=True).start()

    @staticmethod
    def _pump(stream, q, as_json):
        for line in stream:
            line = line.rstrip("\n")
            if not line:
                continue
            if as_json:
                try:
                    q.put(json.loads(line))
                except ValueError:
                    q.put({"type": "garbled", "line": line})
            else:
                q.put(line)

    def send(self, msg):
        s = json.dumps(msg, allow_nan=False)
        with self._wlock:
            try:
                self.p.stdin.write(s + "\n")
                self.p.stdin.flush()
            except (BrokenPipeError, OSError):
                pass

    def alive(self):
        return self.p.poll() is None

    def close(self):
        self.send({"type": "quit"})
        try:
            self.p.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.p.kill()


def _clean(v):
    """NaN is not JSON: the runner reads null as NaN. Also drops the world's
    private _truth fields, which a real camera would not know."""
    if isinstance(v, float):
        return None if v != v or v in (float("inf"), float("-inf")) else v
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items() if not k.startswith("_")}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    return v


class Sim:
    """The world and the runner, stepped together in real time."""

    def __init__(self, sc, exe=RUNNER, tree=TREE):
        self.lock = threading.RLock()
        self.sc = sc
        self.world = W.World(sc)
        self.runner = RunnerProc(exe, tree)
        self.tree_name = os.path.basename(tree)
        self.bt = None
        self.book = None
        self.feedback = {}
        self.mission = {"state": "idle", "outcome": None, "detail": "", "elapsed_s": 0.0}
        self.task = "TASK_NONE"
        self.log = deque(maxlen=400)
        self.reports = deque(maxlen=60)
        self.stop = False
        self._thread = threading.Thread(target=self._loop, daemon=True)

    # -------------------------------------------------------------- control

    def start(self):
        self._thread.start()

    def close(self):
        self.stop = True
        self.runner.close()

    def send_goal(self, tier=None, timeout_s=400.0):
        with self.lock:
            if self.mission["state"] == "active":
                raise RuntimeError("a mission is already running - cancel it first")
            tier = self.sc.tier if tier is None else int(tier)
            self.sc.tier = tier
            lat, lon = self.world.approach_latlon()
            self.mission = {"state": "active", "outcome": None, "detail": "",
                            "elapsed_s": 0.0, "tier": tier}
            self.bt, self.book = None, None
        self.runner.send({"type": "goal", "tier": tier, "timeout_s": float(timeout_s),
                          "approach_latitude": lat, "approach_longitude": lon})
        self._note("sim", "goal sent: tier %d, approach %.7f, %.7f" % (tier, lat, lon))

    def cancel(self):
        self.runner.send({"type": "cancel"})
        self._note("sim", "cancel sent")

    def reset(self, updates=None, randomise=False):
        """A fresh course. Refused while a mission runs: the boat would be
        teleported under a tree that thinks it knows where the bays are."""
        with self.lock:
            if self.mission["state"] == "active":
                raise RuntimeError("cancel the mission first")
            sc = W.Scenario(**{f.name: getattr(self.sc, f.name) for f in fields(W.Scenario)})
            for k, v in (updates or {}).items():
                if not hasattr(sc, k):
                    raise ValueError("no such knob: %s" % k)
                cur = getattr(sc, k)
                if isinstance(cur, bool):
                    v = bool(v)
                elif isinstance(cur, int):
                    v = int(v)
                elif isinstance(cur, float):
                    v = float(v)
                elif isinstance(cur, tuple):
                    v = tuple(v)
                setattr(sc, k, v)
            if randomise:
                sc.seed = random.randint(1, 10 ** 6)
                sc.randomise(random.Random(sc.seed))
            self.sc = sc
            self.world = W.World(sc)
            self.bt, self.book = None, None
            self.mission = {"state": "idle", "outcome": None, "detail": "", "elapsed_s": 0.0}
        self._note("sim", "course reset: green bay %d, window %d, code %s, tier %d"
                   % (sc.green_bay, sc.target_window, "/".join(sc.code), sc.tier))

    def knob(self, name, value):
        """Live knobs: things that can change mid-run without lying to the tree."""
        live = {"camera_ok", "miscolour", "unknown_rate", "current_mps", "current_to_deg"}
        with self.lock:
            if name not in live:
                raise ValueError("%s is not a live knob; use reset" % name)
            cur = getattr(self.sc, name)
            setattr(self.sc, name, bool(value) if isinstance(cur, bool) else float(value))
        self._note("sim", "%s -> %s" % (name, value))

    def set_mode(self, mode):
        with self.lock:
            self.world.boat.mode = mode.upper()
        self._note("sim", "autopilot mode -> %s" % mode.upper())

    # -------------------------------------------------------------- the loop

    def _note(self, who, text):
        with self.lock:
            self.log.append({"t": round(self.world.t, 2), "who": who, "text": text})

    def _loop(self):
        dt = W.World.DT
        next_t = time.monotonic()
        while not self.stop:
            with self.lock:
                msgs = self.world.step()
            for m in msgs:
                self.runner.send(_clean(m))
            self._drain()
            next_t += dt
            lag = next_t - time.monotonic()
            if lag > 0:
                time.sleep(lag)
            elif lag < -1.0:
                next_t = time.monotonic()          # fell badly behind: do not sprint

    def _drain(self):
        while True:
            try:
                m = self.runner.out.get_nowait()
            except queue.Empty:
                break
            self._handle(m)
        while True:
            try:
                line = self.runner.logs.get_nowait()
            except queue.Empty:
                break
            self._note("tree", line)

    def _handle(self, m):
        kind = m.get("type")
        with self.lock:
            w = self.world
            if kind == "setpoint":
                w.on_setpoint(m["lat"], m["lon"])
            elif kind == "cannon":
                w.on_cannon(json.loads(m["json"]))
            elif kind in ("docking_report", "firefighting_report", "resource_request", "uav_request"):
                payload = json.loads(m["json"])
                self.reports.append({"t": round(w.t, 2), "kind": kind, "json": m["json"]})
                w.on_report(kind, payload)
            elif kind == "bt":
                self.bt = m["status"]
            elif kind == "book":
                self.book = m
            elif kind == "feedback":
                self.feedback = m
                self.mission["elapsed_s"] = m.get("elapsed_s", 0.0)
            elif kind == "task":
                self.task = m["token"]
            elif kind == "result":
                self.mission.update(state="done", outcome=m.get("outcome"),
                                    detail=m.get("detail", ""),
                                    elapsed_s=m.get("elapsed_s", self.mission["elapsed_s"]))
                self.log.append({"t": round(w.t, 2), "who": "sim",
                                 "text": "RESULT %s: %s" % (OUTCOMES.get(m.get("outcome"), "?"),
                                                            m.get("detail", ""))})

    # -------------------------------------------------------------- state

    def state(self):
        with self.lock:
            snap = self.world.snapshot()
            obs = self.world.last_obs
            return _clean({
                "world": snap,
                "obs": obs,
                "camera": {"fx": W.Camera.FX, "w": W.Camera.W_PX, "h": W.Camera.H_PX,
                           "face_w": W.FACE_W, "face_h": W.FACE_H, "face_z": W.FACE_Z,
                           "cam_z": W.Boat.CAM_Z, "window_m": W.WINDOW_M,
                           "slots": W.WINDOW_SLOTS},
                "origin": list(self.sc.origin),
                "bt": self.bt, "book": self.book, "feedback": self.feedback,
                "mission": dict(self.mission, outcome_name=OUTCOMES.get(self.mission["outcome"])),
                "task": self.task, "tree": self.tree_name,
                "runner_alive": self.runner.alive(),
                "log": list(self.log)[-120:], "reports": list(self.reports),
            })


# ------------------------------------------------------------------ the page

def serve(sim, port):
    class H(http.server.BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _send(self, code, body, ctype="application/json"):
            data = body if isinstance(body, bytes) else body.encode("utf-8")
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Cache-Control", "no-store")
            self.send_header("Content-Length", str(len(data)))
            self.end_headers()
            self.wfile.write(data)

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                with open(PAGE, "rb") as f:
                    self._send(200, f.read(), "text/html; charset=utf-8")
            elif self.path == "/state":
                self._send(200, json.dumps(sim.state(), allow_nan=False))
            else:
                self._send(404, '{"error":"not found"}')

        def do_POST(self):
            n = int(self.headers.get("Content-Length") or 0)
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except ValueError:
                return self._send(400, '{"error":"bad json"}')
            try:
                if self.path == "/goal":
                    sim.send_goal(body.get("tier"), body.get("timeout_s", 400.0))
                elif self.path == "/cancel":
                    sim.cancel()
                elif self.path == "/reset":
                    sim.reset(body.get("scenario"), bool(body.get("randomise")))
                elif self.path == "/knob":
                    sim.knob(body["name"], body["value"])
                elif self.path == "/mode":
                    sim.set_mode(body["mode"])
                else:
                    return self._send(404, '{"error":"not found"}')
            except (RuntimeError, ValueError, KeyError) as e:
                return self._send(409, json.dumps({"error": str(e)}))
            self._send(200, '{"ok":true}')

    srv = http.server.ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    return srv


def headless(sim, tier, timeout_s):
    """One mission, then the verdict. Exit 0 only if every report was right."""
    sim.start()
    time.sleep(1.0)
    sim.send_goal(tier, timeout_s)
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout_s + 20:
        time.sleep(0.5)
        with sim.lock:
            if sim.mission["state"] == "done":
                break
    time.sleep(1.0)
    st = sim.state()
    summ = st["world"]["judge"]["summary"]
    need = ["docking", "firefighting"] + (["request", "uav"] if tier >= 1 else [])
    ok = (st["mission"]["outcome"] == 0 and all(summ[k] for k in need)
          and summ["contacts"] == 0)
    with sim.lock:
        log = [e for e in sim.log if e["who"] != "tree" or "tree @" not in e["text"]]
    out = {"ok": ok, "mission": st["mission"], "judge": summ,
           "events": st["world"]["judge"]["events"],
           "scenario": st["world"]["scenario"], "reports": st["reports"],
           "log": log[-150:]}
    sim.close()
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--port", type=int, default=PORT)
    ap.add_argument("--tree", default=TREE)
    ap.add_argument("--runner", default=RUNNER)
    ap.add_argument("--headless", action="store_true", help="run one mission, print the verdict")
    ap.add_argument("--timeout", type=float, default=400.0)
    ap.add_argument("--scenario", default="{}",
                    help='JSON of Scenario knobs, e.g. \'{"green_bay": 3, "tier": 1}\'')
    ap.add_argument("--random", action="store_true", help="randomise bay, window and code")
    a = ap.parse_args()

    sc = W.Scenario()
    for k, v in json.loads(a.scenario).items():
        if not hasattr(sc, k):
            sys.exit("no such knob: %s" % k)
        setattr(sc, k, tuple(v) if isinstance(getattr(sc, k), tuple) else v)
    if a.random:
        sc.seed = random.randint(1, 10 ** 6)
        sc.randomise(random.Random(sc.seed))
    sim = Sim(sc, a.runner, a.tree)

    if a.headless:
        out = headless(sim, sc.tier, a.timeout)
        print(json.dumps(out, indent=1))
        sys.exit(0 if out["ok"] else 1)

    srv = serve(sim, a.port)
    sim.start()
    print("Task 3 sim on http://localhost:%d  (tree: %s)" % (a.port, os.path.relpath(a.tree, REPO)),
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        sim.close()


if __name__ == "__main__":
    main()
