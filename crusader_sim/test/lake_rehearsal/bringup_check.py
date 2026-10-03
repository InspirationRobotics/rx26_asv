"""bringup_check.py — the rehearsal's `bringup` scenario, the page's half: ask the lake panel what it says about the rig.

    python3 bringup_check.py --port 8097

Plain HTTP GET /api/state (stdlib only, runs on the WSL host). Prints the `rig` block the page shows (tree, nav_mode and
why, POOL, the overlay's keys and what was ignored) and exits 1 when there is none. Reads only.
"""
import argparse
import json
import sys
import urllib.request

ap = argparse.ArgumentParser()
ap.add_argument("--port", type=int, default=8097)
a = ap.parse_args()
try:
    with urllib.request.urlopen("http://127.0.0.1:%d/api/state?log=radio:0" % a.port, timeout=8) as r:
        st = json.loads(r.read())
except Exception as e:  # noqa: BLE001
    print("--- the page: panel unreachable on :%d (%s)" % (a.port, e))
    sys.exit(1)
rig = st.get("rig")
print("--- the page (/api/state): rig")
if not rig:
    print("  NONE: the page has no rig line")
    sys.exit(1)
if "error" in rig:
    print("  rig file problem: %s" % rig["error"])
    sys.exit(1)
print("  tree %s (global %s)  nav_mode %s  pool %s  publish %s" % (
    rig["tree"], rig["global_tree"], rig["nav_mode"], rig["pool"], rig["publish"]))
print("  nav_mode because: %s" % rig["nav_why"])
for n in rig.get("notes") or []:
    print("  note: %s" % n)
n = sum(len(k) for k in (rig.get("applied") or {}).values())
print("  overlay %s: %d key(s) applied %s" % (rig.get("tuning"), n, {k: len(v) for k, v in (rig.get("applied") or {}).items()}))
print("  ignored (nav not started): %s" % (rig.get("ignored") or "nothing"))
print("  feed up %s, FCU %s, tiers offered %s" % (st["feed"]["up"], (st["fcu"].get("data") or {}).get("mode"), st.get("tiers")))
