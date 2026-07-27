#!/usr/bin/env python3
"""Venue check — are the scenarios still pointed at last season's venue?

WHY THIS EXISTS
===============
Every scenario in orchestrator/scenarios/ currently carries

    "origin": [32.7020, -117.2510]

which is San Diego. docker/sitl/run_sitl.sh hardcodes the same coordinates with
the comment "matches scenario origins". RobotX 2026 is at The Promontory @
Marina Bay, SINGAPORE.

For most of the stack that does not matter: the scenario frame is local metres
east/north of `origin`, so geometry, waypoints and the evaluator are all
translation-invariant. Moving the origin does not change objective 1/2/3.

ONE THING IS NOT TRANSLATION-INVARIANT: MAGNETIC DECLINATION.

    San Diego   ~ +11.5 deg E
    Singapore   ~  +0.2 deg E
    difference  ~  11.3 deg

ArduPilot's EKF uses declination to reconcile magnetometer heading with true
north. Crusader is GPS-heading (per CLAUDE.md), which reduces but does not
eliminate exposure — anywhere true and magnetic heading are conflated, in
firmware params or in our own frame transforms, a constant ~11 deg offset is
being silently absorbed today and VANISHES on arrival in Singapore.

That is the failure mode where everything works all season and then the boat
turns the wrong way at the venue. It is cheap to rule out and expensive to
discover in November.

WHAT TO DO
==========
1. Run the same scenario at both declinations and diff the traces:

       python3 tools/sim/venue_check.py --sweep

   Identical results => no heading-frame conflation. Different => you have a
   frame bug, and you found it in July instead of on the dock.

2. Scenarios are ADD-DON'T-MUTATE (docs/CHANGE_IMPACT_MAP.md).
   Use --derive to create copies, then update together:
       orchestrator/scenarios/*.json      "origin"
       docker/sitl/run_sitl.sh            HOME_LOC
       tools/sim/worlds/*.sdf             regenerate (scenario_to_world.py)
   run_sitl_gazebo.sh already derives HOME_LOC from the scenario, so it cannot
   drift; run_sitl.sh still hardcodes it.

3. Verify the actual figures at NOAA NCEI for November 2026 before trusting
   anything here, including these numbers:
       https://www.ngdc.noaa.gov/geomag/calculators/magcalc.shtml

USAGE
    python3 tools/sim/venue_check.py
    python3 tools/sim/venue_check.py --sweep
    python3 tools/sim/venue_check.py --derive 1.28165,103.85406    # dry run
    python3 tools/sim/venue_check.py --derive 1.28165,103.85406 --write
"""
import argparse
import json
import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
SCENARIOS = REPO / "orchestrator" / "scenarios"

# Approximate 2026 values. VERIFY at NOAA NCEI before relying on them.
KNOWN_VENUES = {
    "san_diego": {"origin": (32.7020, -117.2510), "declination_deg": 11.5,
                  "note": "RX24 / current scenario origin"},
    "marina_bay": {"origin": (1.28165, 103.85406), "declination_deg": 0.2,
                   "note": "RobotX 2026 — The Promontory (Handbook 1.1)"},
    "sg_river": {"origin": (1.281467, 103.855616), "declination_deg": 0.2,
                 "note": "BumblebeeAS/bb_worlds robotx_2026_sg_river.world"},
}


def classify(origin):
    for name, v in KNOWN_VENUES.items():
        if (abs(origin[0] - v["origin"][0]) < 0.05
                and abs(origin[1] - v["origin"][1]) < 0.05):
            return name, v
    return None, None


def haversine_km(a, b):
    R = 6371.0
    p1, p2 = math.radians(a[0]), math.radians(b[0])
    dp = p2 - p1
    dl = math.radians(b[1] - a[1])
    h = (math.sin(dp / 2) ** 2
         + math.cos(p1) * math.cos(p2) * math.sin(dl / 2) ** 2)
    return 2 * R * math.asin(math.sqrt(h))


def report():
    files = sorted(SCENARIOS.glob("*.json"))
    if not files:
        print("no scenarios found")
        return 1

    print("SCENARIO ORIGINS\n")
    venues = {}
    for f in files:
        sc = json.loads(f.read_text())
        o = tuple(sc["origin"])
        name, v = classify(o)
        venues.setdefault(name, []).append(f.stem)
        label = f"{name} ({v['note']})" if name else "UNKNOWN"
        print(f"  {f.stem:30} {o[0]:>10.5f}, {o[1]:>11.5f}   {label}")

    comp = KNOWN_VENUES["marina_bay"]
    print(f"\nCOMPETITION VENUE\n  marina_bay  {comp['origin'][0]}, "
          f"{comp['origin'][1]}   {comp['note']}")

    stale = [n for n in venues if n != "marina_bay"]
    if not stale:
        print("\nAll scenarios are at the competition venue.")
        return 0

    for name in stale:
        v = KNOWN_VENUES.get(name)
        if not v:
            continue
        dist = haversine_km(v["origin"], comp["origin"])
        ddec = abs(v["declination_deg"] - comp["declination_deg"])
        print(f"\n  {len(venues[name])} scenario(s) still at '{name}':")
        print(f"    {', '.join(venues[name])}")
        print(f"    distance to venue      : {dist:,.0f} km")
        print(f"    declination difference : {ddec:.1f} deg  "
              f"({v['declination_deg']} -> {comp['declination_deg']})")

    print("""
Geometry is translation-invariant, so objectives 1/2/3 are unaffected by the
origin itself. DECLINATION IS NOT. Anywhere true and magnetic heading are
conflated, an ~11 deg constant is being absorbed today and disappears in
Singapore.

    python3 tools/sim/venue_check.py --sweep       prove it does not matter
    python3 tools/sim/venue_check.py --derive ...  add venue copies
""")
    return 1


def sweep(scenario, seeds):
    """
    Run the same scenario twice — venue declination vs current-origin
    declination — and compare traces. Identical => no heading-frame conflation.

    Uses the kinematic backend, which is declination-agnostic by construction,
    so this is a HARNESS check: it proves the sweep plumbing works and gives a
    baseline. The meaningful sweep is the same comparison under --backend sitl
    or --backend gazebo, where ArduPilot's EKF actually consumes COMPASS_DEC.
    """
    print(f"sweep: {Path(scenario).stem}, seeds {seeds}\n")
    out = {}
    for label in ("as_written", "venue_origin"):
        results = []
        for s in seeds:
            r = subprocess.run(
                [sys.executable, str(REPO / "orchestrator/run_episode.py"),
                 "--scenario", scenario, "--backend", "kinematic",
                 "--seed", str(s), "--out", f"/tmp/sweep_{label}_{s}.json"],
                capture_output=True, text=True, cwd=REPO)
            m = json.loads(Path(f"/tmp/sweep_{label}_{s}.json").read_text())
            results.append((m["objective1"]["auto_fail"],
                            round(m["objective2"].get("time_to_complete_s") or -1, 3)))
        out[label] = results
        print(f"  {label:14} {results}")

    same = out["as_written"] == out["venue_origin"]
    print(f"\n  {'IDENTICAL' if same else 'DIFFERENT'} — "
          + ("kinematic backend is declination-agnostic, as expected. "
             "Repeat with --backend sitl/gazebo for the real check."
             if same else
             "investigate: something in the loop depends on origin."))
    return 0


def derive(target, write):
    """Create venue-shifted COPIES of each scenario. Never mutate in place.

    docs/CHANGE_IMPACT_MAP.md is explicit that orchestrator/scenarios/*.json is
    "add-don't-mutate", because those files are what makes results comparable:
    every metrics JSON records the scenario name and version, and the gates
    (G0/G3/G4/G5) are calibrated against them. Rewriting an origin in place
    silently invalidates every historical run without changing its name, so old
    and new numbers sit in the same history looking comparable and are not.

    So this emits `<name>_sg.json` alongside the originals. Run both for a
    while: identical objectives with an ~11 deg declination change is the
    evidence that no heading-frame conflation exists.
    """
    lat, lon = (float(v) for v in target.split(","))
    files = [f for f in sorted(SCENARIOS.glob("*.json"))
             if not f.stem.endswith("_sg")]
    print(f"{'DERIVING' if write else 'DRY RUN'} venue copies -> {lat}, {lon}\n")

    made = []
    for f in files:
        sc = json.loads(f.read_text())
        if tuple(sc["origin"]) == (lat, lon):
            print(f"  {f.stem:30} already at target — skipped")
            continue
        out = f.with_name(f"{f.stem}_sg.json")
        print(f"  {f.stem:30} -> {out.name}")
        if write:
            sc["name"] = f"{sc['name']}_sg"
            sc["origin"] = [lat, lon]
            sc["_derived_from"] = f.name
            sc["_comment"] = (
                sc.get("_comment", "")
                + "  VENUE COPY: identical geometry, Marina Bay origin. "
                  "Compare against the original to prove no heading-frame "
                  "conflation (declination differs by ~11 deg).").strip()
            out.write_text(json.dumps(sc, indent=2) + "\n")
            made.append(out.name)

    if write and made:
        print("\nCreated:", ", ".join(made))
        print("\nNOW ALSO:")
        print("  1. python3 tools/sim/scenario_to_world.py --all")
        print("  2. run the gates against BOTH sets and compare")
        print("  3. when you are ready to switch, update HOME_LOC in")
        print("     docker/sitl/run_sitl.sh (run_sitl_gazebo.sh reads the")
        print("     scenario already, so it needs no change)")
        print("\nOriginals are untouched — historical results stay comparable.")
    elif not write:
        print("\n(dry run — pass --write to create the copies)")
    return 0


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--scenario", default=str(SCENARIOS / "mission1_transit.json"))
    ap.add_argument("--seeds", default="0,1,2")
    ap.add_argument("--derive", metavar="LAT,LON",
                    help="create venue-shifted COPIES "
                         "(scenarios are add-don't-mutate)")
    ap.add_argument("--write", action="store_true")
    args = ap.parse_args()

    if args.derive:
        return derive(args.derive, args.write)
    if args.sweep:
        return sweep(args.scenario, [int(s) for s in args.seeds.split(",")])
    return report()


if __name__ == "__main__":
    sys.exit(main())
