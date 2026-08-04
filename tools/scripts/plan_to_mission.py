#!/usr/bin/env python3
"""plan_to_mission — QGroundControl/Mission Planner .plan -> mission_file JSON.

`gate_navigator` and `mission_planner_node` both read the same shape:

    {"waypoints": [[lat, lon], [lat, lon], ...]}

QGC saves a `.plan` instead: a mission item list where each waypoint's
coordinate lives in `params[4]` (lat) and `params[5]` (lon) — MAVLink param5/6,
zero-indexed. This converts one to the other so the QGC planning workflow still
produces something the ROS nodes can load.

    python3 plan_to_mission.py buoy_field.plan -o ~/robotx_ws/missions/mission_1.json

WHERE THE OUTPUT GOES: the workspace root, beside `models/` — NOT `~/missions/`.
`~/robotx_ws` is the only host directory bind-mounted into the `asv` container, so
it is the only place a mission file is both persistent across `docker rm` and
visible from host and container alike. Set the node's `mission_file` param to the
CONTAINER path, `/root/robotx_ws/missions/<name>.json`: neither gate_navigator nor
mission_planner_node expands `~`, so a literal tilde in that param is a directory
named `~` and fails.

The printed index list is what `gate_wp_indices` refers to (zero-based), so read
it before setting that param.

WHY NOT DOWNLOAD FROM THE AUTOPILOT: the pre-integration gate_navigator pulled
waypoints over MAVLink (the MISSION_REQUEST_LIST dialog). The rewired node has no
MAVLink connection of its own — telemetry_bridge is the single gateway — and the
bridge's mission queue is currently owned by the fence uploader, which does not
demultiplex on `mission_type`. Restoring the download means demuxing that queue
and serialising the two dialogs; until then, this file is the path.

Skips are reported on stderr and never silent: a mission item that vanishes
without comment shifts every later index, and `gate_wp_indices` is positional —
a dropped waypoint makes the boat hunt for a gate at the wrong place.
"""
import argparse
import json
import sys
from pathlib import Path

# Mission item types that carry a transit coordinate. Everything else
# (DO_* commands, loiter, speed changes) is deliberately dropped: this file
# feeds a waypoint list, not a mission program.
POSITION_CMDS = {
    16: "NAV_WAYPOINT",
    82: "NAV_SPLINE_WAYPOINT",
}


class PlanError(ValueError):
    """The .plan is not something we can convert. Always fatal — never fall
    back to a partial waypoint list."""


def extract(plan):
    """plan: parsed .plan JSON. Returns (waypoints, skipped).

    waypoints: [[lat, lon], ...] in mission order.
    skipped:   [(item_index, reason)] for everything not converted.
    """
    if not isinstance(plan, dict):
        raise PlanError("top level of a .plan must be a JSON object")
    ftype = plan.get("fileType")
    if ftype != "Plan":
        raise PlanError(f"not a QGC plan file (fileType={ftype!r}, expected 'Plan')")

    mission = plan.get("mission")
    if not isinstance(mission, dict):
        raise PlanError("plan has no 'mission' object")
    items = mission.get("items")
    if not isinstance(items, list) or not items:
        raise PlanError("plan mission has no items")

    waypoints, skipped = [], []
    for i, item in enumerate(items):
        if not isinstance(item, dict):
            skipped.append((i, "not an object"))
            continue
        # ComplexItem = survey/corridor/structure-scan pattern. Those expand to
        # many waypoints inside QGC and are NOT represented in `items`, so a plan
        # built from one cannot be converted — say so rather than emit a short
        # list that looks complete.
        itype = item.get("type")
        if itype != "SimpleItem":
            skipped.append((i, f"type {itype!r} (not a simple waypoint)"))
            continue

        cmd = item.get("command")
        if cmd not in POSITION_CMDS:
            skipped.append((i, f"command {cmd} (carries no transit position)"))
            continue

        params = item.get("params")
        if not isinstance(params, list) or len(params) < 6:
            skipped.append((i, "params too short for a coordinate"))
            continue

        lat, lon = params[4], params[5]
        # None = "unchanged / use previous" in the MAVLink param encoding;
        # (0, 0) is Null Island. Neither is a place the boat was told to go, and
        # emitting either would drive it somewhere nobody planned.
        if lat is None or lon is None:
            skipped.append((i, "null coordinate"))
            continue
        if not isinstance(lat, (int, float)) or not isinstance(lon, (int, float)):
            skipped.append((i, "non-numeric coordinate"))
            continue
        if lat == 0 and lon == 0:
            skipped.append((i, "coordinate (0, 0)"))
            continue
        # Catches the obvious transposition only — a swap where both values
        # happen to stay in range still parses. Cheap, and it catches the case
        # that actually happens (a longitude past +/-90 in the lat slot).
        if not (-90.0 <= lat <= 90.0) or not (-180.0 <= lon <= 180.0):
            raise PlanError(
                f"item {i}: coordinate ({lat}, {lon}) out of range — "
                "lat/lon transposed in the source plan?")

        waypoints.append([float(lat), float(lon)])

    if not waypoints:
        raise PlanError(
            f"no convertible waypoints in {len(items)} mission item(s); "
            "a survey/corridor pattern cannot be converted — lay the course "
            "out as plain waypoints in QGC")
    return waypoints, skipped


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="Convert a QGC .plan into a gate_navigator mission_file.")
    ap.add_argument("plan", type=Path, help="input .plan from QGC/Mission Planner")
    ap.add_argument("-o", "--out", type=Path, required=True,
                    help="output mission JSON")
    args = ap.parse_args(argv)

    try:
        raw = json.loads(args.plan.read_text())
    except FileNotFoundError:
        return f"ERROR: no such plan file: {args.plan}"
    except json.JSONDecodeError as e:
        return f"ERROR: {args.plan} is not valid JSON: {e}"

    try:
        waypoints, skipped = extract(raw)
    except PlanError as e:
        return f"ERROR: {e}"

    for i, why in skipped:
        print(f"  skipped item {i}: {why}", file=sys.stderr)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({"waypoints": waypoints}, indent=2) + "\n")

    print(f"{len(waypoints)} waypoint(s) -> {args.out}")
    print("indices below are what gate_wp_indices refers to:")
    for i, (lat, lon) in enumerate(waypoints):
        print(f"  [{i}] {lat:.7f}, {lon:.7f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())