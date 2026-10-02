"""SIM-mode panel flow over the sim panel's HTTP API: LAUNCH, START, ACK every checkpoint (EXIT included), then read
the boat's-map layers and the judge from /api/state.   python3 sim_flow.py --port 8095"""
import argparse
import json
import time
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8095)
ap.add_argument("--max-s", type=float, default=420.0)
ap.add_argument("--course", default="task1_avoid")
a = ap.parse_args()
B = "http://127.0.0.1:%d" % a.port
T0 = time.time()


def say(m):
    print("[%6.1f] %s" % (time.time() - T0, m), flush=True)


def st():
    with urllib.request.urlopen(B + "/api/state?log=sim:0,mission:0", timeout=8) as r:
        return json.loads(r.read())


def post(p, body=None):
    rq = urllib.request.Request(B + p, data=json.dumps(body or {}).encode(), method="POST", headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(rq, timeout=30) as r:
        return json.loads(r.read())


say("template %s -> %s" % (a.course, post("/api/template", {"name": a.course})))
time.sleep(0.5)
S = st()
say("panel: layout %d buoys, note %r, errors %s" % (len(S["layout"]["buoys"]), S["layout"]["note"], S["layout"]["errors"]))
say("LAUNCH -> %s" % post("/api/launch"))
t = time.time()
while time.time() - t < 480:
    S = st()
    if S["sim"]["state"] in ("up", "failed"):
        break
    time.sleep(3)
say("sim %s %s" % (S["sim"]["state"], S["sim"]["detail"]))
if S["sim"]["state"] != "up":
    raise SystemExit(2)
time.sleep(8)
say("START -> %s" % post("/api/start"))
t_start = time.time()
peak = {"path": 0, "tracks": 0, "fused": 0, "cells": 0, "lidar": 0, "n_gates": None, "nav": set()}
answered, phases = set(), []
while time.time() - t_start < a.max_s:
    S = st()
    m, f = S["mission"], S["feed"]
    if m["phase"] and (not phases or phases[-1][0] != m["phase"]):
        phases.append((m["phase"], round(time.time() - t_start)))
        say("phase %s plan %s buoys %s" % (m["phase"], m["plan"], m["buoys"]))
    lg, tr, ps, cm = f["leg"], f["tracks"], f["passage"], f["costmap"]
    if lg["status"] == "fresh":
        peak["path"] = max(peak["path"], len(lg["data"]["path"]))
        peak["nav"].add(lg["data"]["state"])
    if tr["status"] == "fresh":
        peak["tracks"] = max(peak["tracks"], len(tr["data"]["items"]))
    if ps["status"] == "fresh":
        peak["fused"] = max(peak["fused"], len(ps["data"]["buoys"]))
        if ps["data"].get("n_gates") is not None:
            peak["n_gates"] = ps["data"]["n_gates"]
    if cm["status"] == "fresh":
        peak["cells"] = max(peak["cells"], len(cm["data"]["cells"]))
        peak["lidar"] = max(peak["lidar"], len(cm["data"]["lidar"] or []))
    c = S["checkpoint"]
    if c and c["seq"] not in answered and time.time() - c["asked"] > 1.0:
        say("BOAT ASKS %d: %s" % (c["seq"], c["label"]))
        say("ACK -> %s" % post("/api/ack"))
        answered.add(c["seq"])
    if not m["running"] and m["exit_code"] is not None:
        break
    time.sleep(0.5)
time.sleep(2)
S = st()
m = S["mission"]
say("mission exit %s, %.0f s since START" % (m["exit_code"], time.time() - t_start))
say("result: %s" % (m["result"] or "").replace("\n", " | "))
say("phases: %s" % phases)
say("checkpoints: %s" % [(c["seq"], c["label"], c["reply"]) for c in S["checkpoints"]])
say("boat's-map layers (peak while running): path pts %d, tracks %d, fused buoys %d, costmap cells %d, lidar cells %d, n_gates %s, nav states %s" % (
    peak["path"], peak["tracks"], peak["fused"], peak["cells"], peak["lidar"], peak["n_gates"], sorted(peak["nav"])))
say("VERDICT:\n%s" % S["verdict"])
say("STOP SIM -> %s" % post("/api/stop_sim"))
t = time.time()
while time.time() - t < 90 and st()["sim"]["state"] != "down":
    time.sleep(3)
say("sim %s" % st()["sim"]["state"])
