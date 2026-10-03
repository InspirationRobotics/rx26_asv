"""reh_summary.py — one screen for a rehearsal run, from what reh_run.sh left in ~/.cache/lake_rehearsal/<tag>/.

    python3 reh_summary.py <run dir> --tag T --scenario S --tier TIER

Reads result.json (the driver), verdict.txt (the referee), rig_up.log (what the rig decided and read back) and
lake_logs/bt.log (the boat's own side of a dead-man). Prints; changes nothing.
"""
import argparse
import json
import os
import re

ap = argparse.ArgumentParser()
ap.add_argument("run")
ap.add_argument("--tag", default="")
ap.add_argument("--scenario", default="")
ap.add_argument("--tier", default="")
a = ap.parse_args()


def read(name):
    try:
        with open(os.path.join(a.run, name), encoding="utf-8", errors="replace") as f:
            return f.read()
    except OSError:
        return ""


rig = read("rig_up.log")


def grab(pattern, text=rig):
    m = re.search(pattern, text, re.M)
    return m.group(0).strip() if m else "?"


print("tag %s   scenario %s   tier %s" % (a.tag, a.scenario, a.tier))
print("rig:   %s | %s" % (grab(r"^\s*tree\s+\S+.*$"), grab(r"^\s*nav_mode\s+\S+.*$")[:150]))
for pat in (r"^\s*POOL: .*$", r"^\s*planner overlay.*$", r".*overlay read back.*", r".*overlay read-back.*"):
    line = grab(pat)
    if line != "?":
        print("       " + line)
try:
    res = json.loads(read("result.json"))
except ValueError:
    res = None
if res:
    print("run:   exit %s after %s s   %s" % (res["exit_code"], res["seconds"], res["result"]))
    print("       page's mission tier: %s; aborted_at %s" % (res["mission_tier"], res["aborted_at"]))
    print("       checkpoints (%d): %s" % (len(res["checkpoints"]), "; ".join("%s %s [%s]" % (s, l, r) for s, l, r in res["checkpoints"]) or "none"))
else:
    print("run:   no result.json (a bringup run, or the driver did not finish)")
verdict = [l for l in read("verdict.txt").splitlines() if l.strip()]
if verdict:
    print("referee: %s" % verdict[0])
    print("         %s" % (verdict[-1] if len(verdict) > 1 else ""))
bt = read(os.path.join("lake_logs", "bt.log"))
if a.scenario == "deadman":
    stale = [l for l in bt.splitlines() if "stale" in l or "FAILURE" in l or "failed" in l.lower()]
    print("boat's side: %s" % ("\n             ".join(stale[-3:]) or "no stale / FAILURE line in bt.log"))
print("logs:  %s" % a.run)
