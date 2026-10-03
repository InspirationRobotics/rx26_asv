"""The rehearsal's browser stand-in: drives the lake panel over its HTTP API only (no screenshots, no UI).

    python3 reh_driver.py --port 8097 --scenario pass|recolour|abort|deadman --pilot "<cmd to arm+GUIDED>"
                          [--hold "<cmd>"] [--tier disruptive|advanced] [--result FILE]

  pass      load the course as the field, COMMIT, pilot arms + GUIDED, START (with --tier), ACK every checkpoint (EXIT
            included), wait for the goal to finish. Prints the checkpoints with their labels and the goal's result.
            With --tier advanced the boat must ask NO checkpoint, and the driver says so.
  recolour  the pass, plus the pretend-UAV colour flow at checkpoint 1: the last buoy is recoloured (BLACK -> RED, on the
            starboard side of the course, so the passage stays valid), SEND CHANGES + ACK, the boat re-plans and the run
            COMPLETES. (abort's recolour, without the abort.)
  abort     the recolour, and ABORT in TRANSIT: the goal is cancelled, the boat must hold, the banner state is read back.
  deadman   the pass, but in TRANSIT the driver STOPS POLLING /api/state (the browser goes away) and touches nothing
            until the boat's own guard has aborted the mission; polling resumes afterwards to read what happened.

--result FILE gets one JSON object: the scenario, tier, START-to-finish seconds, the goal's exit code and result line,
the checkpoints asked with their labels. reh_run.sh folds it into summary.txt.
"""
import argparse
import json
import subprocess
import sys
import threading
import time
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8097)
ap.add_argument("--scenario", default="pass", choices=("pass", "recolour", "abort", "deadman"))
ap.add_argument("--tier", default="disruptive", choices=("disruptive", "advanced"))
ap.add_argument("--result", default="")
ap.add_argument("--course", default="task1_core")
ap.add_argument("--pilot", required=True)
ap.add_argument("--hold", default="")
ap.add_argument("--max-s", type=float, default=480.0)
a = ap.parse_args()
BASE = "http://127.0.0.1:%d" % a.port
T0 = time.time()
polling = threading.Event()
polling.set()
S = {"state": None, "err": None, "last_poll": 0.0}
LOG = []


def say(msg):
    line = "[%6.1f] %s" % (time.time() - T0, msg)
    LOG.append(line)
    print(line, flush=True)


def get_state():
    with urllib.request.urlopen(BASE + "/api/state?log=radio:0,mission:0", timeout=5) as r:
        return json.loads(r.read())


def post(path, body=None):
    req = urllib.request.Request(BASE + path, data=json.dumps(body or {}).encode(), method="POST",
                                 headers={"Content-Type": "application/json"})
    with urllib.request.urlopen(req, timeout=10) as r:
        return json.loads(r.read())


def poller():
    while True:
        if polling.is_set():
            try:
                S["state"], S["last_poll"] = get_state(), time.time()
            except Exception as e:                                  # noqa: BLE001
                S["err"] = str(e)
        time.sleep(0.25)


threading.Thread(target=poller, daemon=True).start()


def wait(cond, timeout, what):
    t = time.time()
    while time.time() - t < timeout:
        st = S["state"]
        if st is not None and cond(st):
            return st
        time.sleep(0.2)
    say("TIMEOUT waiting for %s" % what)
    return S["state"]


def run_cmd(cmd):
    r = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=90)
    say("cmd: %s -> %s" % (cmd.split("pilot_standin.py")[-1].strip() or cmd[:40], (r.stdout + r.stderr).strip().replace("\n", " | ")[:200]))
    return r.returncode


# ---------------------------------------------------------------- 1. the panel and the feed
st = wait(lambda s: s["feed"]["up"] and s["fcu"]["status"] == "fresh", 60, "the feed and a fresh FCU status")
say("panel up; feed up=%s fcu=%s boat=%s datum=%s" % (st["feed"]["up"], st["fcu"], st["boat"] and (round(st["boat"]["x"], 1), round(st["boat"]["y"], 1)), st["datum"]))
say("start_block now: %s" % st["start_block"])

# ---------------------------------------------------------------- 2. the field: pin / typed lat/lon, then the course
r = post("/api/pin", {"state": "off"})
say("PIN AT BOAT -> %s" % r)
b = S["state"]["layout"]["buoys"]
say("field after pin: %s" % [(round(x["x"], 1), round(x["y"], 1), x["state"]) for x in S["state"]["layout"]["buoys"]])
post("/api/clear")
r = post("/api/template", {"name": a.course})
time.sleep(0.6)
n = len(S["state"]["layout"]["buoys"])
say("loaded course %s -> %s, %d buoys, errors %s" % (a.course, r, n, S["state"]["layout"]["errors"]))
layout0 = S["state"]["layout"]["buoys"]

# ---------------------------------------------------------------- 3. COMMIT
r = post("/api/commit")
say("COMMIT FIELD -> %s" % r)
st = wait(lambda s: s["mode"] == "live" and s["radio"]["sent"] >= 1, 10, "the field on the air")
say("live; radio sent=%s resends=%s" % (st["radio"]["sent"], st["radio"]["resends"]))
time.sleep(4)
st = S["state"]
fused = st["feed"]["passage"]
say("boat's fused passage before START: status %s" % fused["status"])

# ---------------------------------------------------------------- 4. the pilot, then START
say("start_block before the pilot: %s" % S["state"]["start_block"])
rc = post("/api/start", {"tier": a.tier})
say("START before arm/GUIDED (must be refused) -> %s" % rc)
run_cmd(a.pilot)
st = wait(lambda s: s["start_block"] is None, 30, "START to enable (armed + GUIDED)")
say("fcu now %s; start_block %s" % (st["fcu"]["data"], st["start_block"]))
r = post("/api/start", {"tier": a.tier})
say("START (tier %s) -> %s" % (a.tier, r))
t_start = time.time()
t_done = None
time.sleep(0.6)
say("the page reports mission tier %r (rig %s)" % (S["state"]["mission"].get("tier"), (S["state"].get("rig") or {}).get("tree")))

# ---------------------------------------------------------------- 5. the run
answered = set()
recoloured = False
abort_sent = None
deadman_t = None
t_end = time.time() + a.max_s
phase_seen = []
while time.time() < t_end:
    st = S["state"]
    if a.scenario == "deadman" and deadman_t is not None:
        # nothing is polled: look at the world only through the container's own logs, via the orchestrator
        if time.time() - deadman_t > 45:
            break
        time.sleep(1)
        continue
    if st is None:
        time.sleep(0.2)
        continue
    m = st["mission"]
    if m["phase"] and (not phase_seen or phase_seen[-1] != m["phase"]):
        phase_seen.append(m["phase"])
        say("phase %s %.0f%% plan %s buoys %s" % (m["phase"], m["pct"], m["plan"], m["buoys"]))
    c = st["checkpoint"]
    if c and c["seq"] not in answered and time.time() - c["asked"] > 1.0:
        say("BOAT ASKS %d: %s   [%s]" % (c["seq"], c["label"], c["what"][:70]))
        if a.scenario in ("abort", "recolour") and c["seq"] == 1 and not recoloured:
            b9 = len(st["run"]["buoys"]) - 1
            say("stage b%d (black) -> RED: %s" % (b9, post("/api/stage", {"id": b9, "state": "flash_red"})))
            time.sleep(0.6)
            say("SEND CHANGES + ACK -> %s" % post("/api/send_ack"))
            recoloured = True
        else:
            say("ACK -> %s" % post("/api/ack"))
        answered.add(c["seq"])
    if a.scenario == "abort" and abort_sent is None and m["phase"] in ("TRANSIT", "transit") and len(answered) >= 2:
        abort_sent = time.time()
        say("ABORT in TRANSIT (phase %s, %d checkpoints answered) -> %s" % (m["phase"], len(answered), post("/api/abort")))
    if a.scenario == "deadman" and deadman_t is None and m["phase"] in ("TRANSIT", "transit") and len(answered) >= 1:
        deadman_t = time.time()
        polling.clear()
        say("STOP POLLING (the browser goes away) in phase %s; last poll %.2f s ago" % (m["phase"], time.time() - S["last_poll"]))
        continue
    if not m["running"] and m["exit_code"] is not None:
        t_done = time.time()
        say("goal finished: exit %s after %.0f s" % (m["exit_code"], t_done - t_start))
        break
    time.sleep(0.25)

# ---------------------------------------------------------------- 6. what happened
if a.scenario == "deadman":
    polling.set()
    time.sleep(2)
st = S["state"]
m = st["mission"]
say("RESULT mission: running=%s exit=%s aborted_at=%s" % (m["running"], m["exit_code"], m["aborted_at"]))
say("RESULT text: %s" % (m["result"] or "").replace("\n", " | "))
say("checkpoints: %s" % [(c["seq"], c["label"], c["reply"]) for c in st["checkpoints"]])
logs = get_state()
full = json.loads(urllib.request.urlopen(BASE + "/api/state?log=radio:0,mission:0", timeout=5).read())
say("--- mission log tail ---")
for l in full["logs"]["mission"]["lines"][-14:]:
    say("  " + l)
say("--- radio log tail ---")
for l in full["logs"]["radio"]["lines"][-14:]:
    say("  " + l)
say("radio: %s  deadman: %s" % (st["radio"], st["deadman"]))
if a.tier == "advanced":
    say("ADVANCED: checkpoints the boat asked: %d (must be 0)" % len(st["checkpoints"]))
if a.result:
    with open(a.result, "w", encoding="utf-8") as f:
        json.dump({"scenario": a.scenario, "tier": a.tier, "mission_tier": m.get("tier"), "exit_code": m["exit_code"],
                   "seconds": None if t_done is None else round(t_done - t_start, 1),
                   "result": (m["result"] or "").split("\n")[0], "aborted_at": m["aborted_at"],
                   "checkpoints": [(c["seq"], c["label"], c["reply"]) for c in st["checkpoints"]]}, f, indent=1)
if a.scenario == "abort":
    time.sleep(6)
    st = get_state()
    sp = st["feed"]["pose"]["data"]
    say("after the abort the boat's ground speed is %s m/s (pose layer %s); banner aborted_at=%s" % (sp and sp.get("speed"), st["feed"]["pose"]["status"], st["mission"]["aborted_at"]))
    if a.hold:
        run_cmd(a.hold)
if a.scenario != "pass":
    pass
print("DRIVER-DONE", flush=True)
