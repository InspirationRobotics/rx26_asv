"""fake_gcs — the REAL ground-station page served against invented state, for a browser.

    python3 test/fake_gcs.py [--port 8190] [--panel-port 8195] [--stale] [--no-shipped-dir]

No ROS and no boat: gcs_server.GcsServer is handed a canned snapshot and an action function, which
is exactly the seam it was written with ("that is what lets the whole page be exercised on a laptop
against invented state"). Then open http://localhost:8190 and read the DOM -- no screenshots needed:

  * the Tuning tab on /bt_runner_node shows the "Planner (applies at the next START)" group, the
    profile picker, Load profile and Save as profile. --stale makes the node look like an OLD build
    (its descriptors lack the "applies at the next START" mark) so the red warning can be checked;
    --no-shipped-dir points the shipped directory at one that does not exist (blank list + reason).
  * the Task 1 tab frames http://<host>:8095/ by default. A stand-in lake panel is served on
    --panel-port (8195): type http://localhost:8195/ into the tab's URL field. Its page polls
    /api/state four times a second like the real one, and GET /polls on it returns how many polls it
    has had, so "leaving the tab stops the heartbeat" is a number that stops moving, not a claim.

Profile listing and saving use the real planner_profiles module when PyYAML is importable (WSL, the
container); otherwise a canned listing stands in and says so. Saved profiles go to a temp directory.
"""
import argparse
import json
import os
import sys
import tempfile
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))

from crusader_groundstation.gcs_page import render  # noqa: E402
from crusader_groundstation.gcs_server import GcsServer  # noqa: E402

try:
    from crusader_groundstation import planner_profiles as pp
except ImportError:                      # no PyYAML on this machine
    pp = None

MARK = "planner knob: applies at the next START (read when a goal is accepted, never mid-mission). Range: "


def knob(name, value, default, bound="> 0", **kw):
    row = {"name": name, "type": "double", "value": value, "default": default, "in_yaml": True,
           "read_only": False, "editable": True, "reason": "", "lo": None, "hi": None, "choices": [],
           "description": MARK + bound}
    row.update(kw)
    return row


def fixed(name, value, kind="double"):
    return {"name": name, "type": kind, "value": value, "default": value, "in_yaml": True,
            "read_only": True, "editable": False, "reason": "[RO] structural", "lo": None, "hi": None,
            "choices": [], "description": "fixed at startup (a timer, topic or the planner port is built "
                                          "from it once): change it in crusader_params.yaml and restart"}


class Fake:
    def __init__(self, stale, shipped_dir, save_dir):
        self.rows = {
            "nav_hard_m": knob("nav_hard_m", 0.8, 0.8),
            "nav_soft_m": knob("nav_soft_m", 2.0, 2.0),
            "nav_gate_step_m": knob("nav_gate_step_m", 0.25, 0.25),
            "nav_orbit_radius_m": knob("nav_orbit_radius_m", 6.0, 6.0),
            "nav_orbit_points": knob("nav_orbit_points", 8, 8, ">= 1", type="integer"),
            "nav_fence_len_m": knob("nav_fence_len_m", 10.0, 10.0, "any finite number"),
            "nav_hazard_rate_hz": fixed("nav_hazard_rate_hz", 2.0),
            "nav_mode": fixed("nav_mode", "off", "string"),
            "tick_hz": {"name": "tick_hz", "type": "double", "value": 10.0, "default": 10.0, "in_yaml": True,
                        "read_only": False, "editable": True, "reason": "", "lo": 1.0, "hi": 50.0,
                        "choices": [], "description": "tick rate"},
        }
        if stale:
            for r in self.rows.values():
                if r["name"].startswith("nav_") and r["editable"]:
                    r["description"] = ""
        self.shipped_dir, self.save_dir = shipped_dir, save_dir
        self.log = []

    def sources(self):
        return [("shipped", self.shipped_dir), ("saved", self.save_dir)]

    def listing(self):
        if pp is not None:
            return pp.listing(self.sources())
        return {"sources": [{"origin": "shipped", "dir": self.shipped_dir, "ok": True, "reason": ""},
                            {"origin": "saved", "dir": self.save_dir, "ok": False,
                             "reason": "fake_gcs has no PyYAML: canned listing"}],
                "profiles": [{"origin": "shipped", "name": "tight_3to5m", "title": "Tight field (canned)",
                              "about": "canned", "error": "",
                              "values": {"nav_orbit_radius_m": 3.0, "nav_orbit_points": 12},
                              "skipped": [{"key": "planner_server.GridBased.tolerance",
                                           "why": "Nav2 / another node: applied only when the rig "
                                                  "launches, needs a rig restart"}]}]}

    def action(self, path, payload):
        self.log.append((path, payload))
        if path == "/params/list":
            node = payload.get("node")
            rows = list(self.rows.values()) if node == "/bt_runner_node" else [
                {"name": "assoc_radius_m", "type": "double", "value": 5.0, "default": 5.0, "in_yaml": True,
                 "read_only": False, "editable": True, "reason": "", "lo": 0.2, "hi": 20.0, "choices": [],
                 "description": "association gate"}]
            return {"ok": True, "message": "", "node": node, "params": rows}
        if path == "/params/set":
            results = []
            for k, v in (payload.get("values") or {}).items():
                row = self.rows.get(k)
                if row is None:
                    results.append({"name": k, "ok": False, "reason": "no parameter %r" % k})
                elif row["read_only"]:
                    results.append({"name": k, "ok": False,
                                    "reason": "parameter '%s' cannot be set because it is read-only" % k})
                elif row["name"] == "nav_gate_step_m" and not v > 0:
                    results.append({"name": k, "ok": False,
                                    "reason": "nav_gate_step_m=%g is refused: it must be > 0 (a zero or "
                                              "negative size or period breaks the planner)" % v})
                else:
                    row["value"] = int(v) if row["type"] == "integer" else float(v)
                    results.append({"name": k, "ok": True, "reason": ""})
            bad = [r for r in results if not r["ok"]]
            return {"ok": not bad, "results": results,
                    "message": "; ".join("%s: %s" % (r["name"], r["reason"]) for r in bad) if bad else
                    "applied %s on bt_runner_node" % ", ".join(sorted(payload.get("values") or {}))}
        if path == "/planner/profile/list":
            return {"ok": True, "message": "", **self.listing()}
        if path == "/planner/profile/save":
            if pp is None:
                return {"ok": True, "message": "canned: saved", **self.listing()}
            rows = [dict(r) for r in self.rows.values()]
            values, note = pp.drifted_planner_values(rows)
            if not values:
                return {"ok": False, "message": note or "no planner key differs from crusader_params.yaml "
                                                        "- nothing to save"}
            ok, message, _ = pp.save(self.save_dir, payload.get("name"), values)
            return {"ok": ok, "message": message, **self.listing()}
        return {"ok": False, "message": "fake_gcs: unknown action %s" % path}

    def snapshot(self, layers):
        return {
            "boat": {"ok": True, "att_ok": True, "lat": 1.3, "lon": 103.85, "x": 0.0, "y": 0.0,
                     "speed": 0.0, "heading": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.0, "age": 0.1},
            "fcu": {"ok": True, "age": 0.1, "mode": "HOLD", "armed": False},
            "targets": {"ok": True, "age": 0.1, "items": []},
            "trail": [], "nav": {"ok": False},
            "nodes": {"groups": [], "items": [], "up": 0}, "profiles": [],
            "tuning": {"nodes": ["/bt_runner_node", "/target_tracker"]},
            "tabs": {"camera": {}, "lidar": {}}, "system": {}, "power": {},
            "logs": {"counts": {}, "nodes": []}, "layers": {"clusters": False, "prox": False},
            "record": {"sessions": [], "live": None, "sources": [], "frame_hz": 10, "frame_hz_range": [0.05, 60],
                       "frame_keys": [], "telemetry_hz": 5, "min_free_gb": 1, "dir": "", "persist": False,
                       "bag": {}},
        }


PANEL_PAGE = b"""<!doctype html><title>fake lake panel</title><body>fake lake panel
<script>setInterval(function(){fetch('/api/state').catch(function(){})},250)</script>"""


def panel_server(port, counter):
    """A stand-in for the lake panel: a page that polls /api/state, a counter, and NO framing headers.
    HEAD gets BaseHTTPRequestHandler's own 501, as it does on the real panel."""
    class H(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def do_GET(self):
            if self.path == "/polls":
                body, ctype = json.dumps({"polls": counter["n"], "t": time.time()}).encode(), "application/json"
            elif self.path.startswith("/api/state"):
                counter["n"] += 1
                body, ctype = b"{}", "application/json"
            else:
                body, ctype = PANEL_PAGE, "text/html"
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def log_message(self, *_):
            pass
    srv = ThreadingHTTPServer(("0.0.0.0", port), H)
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    return srv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=8190)
    ap.add_argument("--panel-port", type=int, default=8195)
    ap.add_argument("--stale", action="store_true", help="bt_runner_node looks like an old build")
    ap.add_argument("--no-shipped-dir", action="store_true")
    ap.add_argument("--shipped-dir", default=os.path.join(HERE, "..", "..", "crusader_sim", "config",
                                                          "tuning_profiles"))
    a = ap.parse_args()
    save_dir = os.path.join(tempfile.mkdtemp(prefix="fake_gcs_"), "tuning")
    shipped = os.path.join(save_dir, "..", "no_such_dir") if a.no_shipped_dir else os.path.abspath(a.shipped_dir)
    fake = Fake(a.stale, shipped, save_dir)
    GcsServer(render(200), fake.snapshot, fake.action).start(a.port)
    panel_server(a.panel_port, {"n": 0})
    print("page  http://localhost:%d   fake lake panel http://localhost:%d/   saved profiles -> %s"
          % (a.port, a.panel_port, save_dir), flush=True)
    while True:
        time.sleep(3600)


if __name__ == "__main__":
    main()
