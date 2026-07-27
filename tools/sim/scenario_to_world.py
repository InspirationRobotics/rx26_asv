#!/usr/bin/env python3
"""Generate a Gazebo world FROM an orchestrator scenario JSON.

    orchestrator/scenarios/mission1_transit.json
                    |
                    v
    tools/sim/worlds/mission1_transit.sdf      (generated, gitignored)

WHY GENERATE RATHER THAN HAND-AUTHOR
------------------------------------
The scenario JSON is already the single source of truth for course geometry —
`Scenario.load()` feeds the kinematic backend, the SITL backend, the evaluator
and the metrics record. A hand-written Gazebo world would be a SECOND copy of
that geometry, and the moment someone nudges a buoy in the Gazebo GUI the two
disagree.

Worse, the disagreement is invisible: the episode runs, the evaluator scores it,
and objective-1 collision checks are computed against `scenario.obstacles` while
the vehicle is physically hitting buoys that live somewhere else. Every number
is plausible. All of them are wrong.

So the world is a BUILD ARTIFACT. Regenerate it; never edit it.

    obstacles  -> buoys, at scenario radius, with collision
    origin     -> <spherical_coordinates>, matching SITL's -l HOME_LOC
    keep-outs  -> NOT rendered (they are virtual RoboCommand zones, not physical)
    waypoints  -> optional translucent markers, visual only, --markers

ORIGIN IS THE WHOLE POINT
-------------------------
`<spherical_coordinates>` here, `-l HOME_LOC` in run_sitl_gazebo.sh and
`scenario.origin` in the JSON must all be the same numbers. All three are
derived from the scenario file by this script and that launcher, so they cannot
drift. GazeboBackend re-checks at reset() and refuses to run on a mismatch.

USAGE
    python3 tools/sim/scenario_to_world.py \
        --scenario orchestrator/scenarios/mission1_transit.json
    python3 tools/sim/scenario_to_world.py --all --markers
"""
import argparse
import json
import math
import sys
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

REPO = Path(__file__).resolve().parents[2]
SCENARIOS = REPO / "orchestrator" / "scenarios"
OUT = Path(__file__).resolve().parent / "worlds"

# Buoy visuals. Colour is inferred from the scenario label so the generated world
# is readable at a glance and matches what the perception stack expects to see.
LABEL_COLORS = {
    "red":   "0.910 0.157 0.169 1",     # RoboNation #E8282B
    "green": "0.031 0.580 0.016 1",     # #089404
    "blue":  "0.000 0.447 0.808 1",     # #0072CE
    "black": "0.05 0.05 0.05 1",
    "white": "0.95 0.95 0.95 1",
}
DEFAULT_COLOR = "0.85 0.75 0.20 1"      # yellow: unlabelled / stray


def sub(parent, tag, text=None, **attrs):
    el = ET.SubElement(parent, tag, {k: str(v) for k, v in attrs.items()})
    if text is not None:
        el.text = str(text)
    return el


def color_for(label: str) -> str:
    low = (label or "").lower()
    for key, rgba in LABEL_COLORS.items():
        if key in low:
            return rgba
    return DEFAULT_COLOR


def add_buoy(world, obs, idx):
    """
    One scenario obstacle -> one physical buoy.

    Radius comes STRAIGHT from the scenario, so what the evaluator scores as a
    collision is what physically collides. Height is cosmetic; the collision
    cylinder is what matters and it matches `Obstacle.radius`.
    """
    name = obs.get("label") or f"obstacle_{idx}"
    r = float(obs.get("radius", 0.3))
    x, y = float(obs["x"]), float(obs["y"])

    model = sub(world, "model", name=name)
    sub(model, "static", "true")
    sub(model, "pose", f"{x:.4f} {y:.4f} 0 0 0 0")
    link = sub(model, "link", name="link")

    col = sub(link, "collision", name="collision")
    cg = sub(sub(col, "geometry"), "cylinder")
    sub(cg, "radius", f"{r:.4f}")
    sub(cg, "length", "1.2")

    vis = sub(link, "visual", name="visual")
    vg = sub(sub(vis, "geometry"), "cylinder")
    sub(vg, "radius", f"{r:.4f}")
    sub(vg, "length", "1.2")
    mat = sub(vis, "material")
    rgba = color_for(name)
    sub(mat, "ambient", rgba)
    sub(mat, "diffuse", rgba)


def add_waypoint_marker(world, wp, idx):
    """Visual only, no collision — a debugging aid, never a physical object."""
    model = sub(world, "model", name=f"wp_{idx}")
    sub(model, "static", "true")
    sub(model, "pose", f"{wp[0]:.4f} {wp[1]:.4f} 0.05 0 0 0")
    link = sub(model, "link", name="link")
    vis = sub(link, "visual", name="visual")
    g = sub(sub(vis, "geometry"), "cylinder")
    sub(g, "radius", "0.35")
    sub(g, "length", "0.02")
    mat = sub(vis, "material")
    sub(mat, "ambient", "0.2 0.6 1.0 0.35")
    sub(mat, "diffuse", "0.2 0.6 1.0 0.35")


def build(scenario: dict, markers: bool, speedup: float) -> ET.Element:
    name = scenario.get("name", "scenario")
    origin = scenario["origin"]

    sdf = ET.Element("sdf", version="1.10")
    world = sub(sdf, "world", name=name)

    world.insert(0, ET.Comment(
        f"\n  GENERATED FROM orchestrator/scenarios/{name}.json — DO NOT EDIT.\n"
        f"    python3 tools/sim/scenario_to_world.py ––scenario <file>\n"
        f"  scenario version : {scenario.get('version', '?')}\n"
        f"  origin           : {origin[0]}, {origin[1]}\n"
        f"  obstacles        : {len(scenario.get('obstacles', []))}\n"
        f"  Course geometry lives in the scenario JSON. Edit it there.\n"))

    # --- physics ----------------------------------------------------------- #
    # real_time_factor must be kept consistent with SITL's SIM_SPEEDUP. Changing
    # one and not the other desynchronises the FDM link and the resulting motion
    # is physically meaningless (it still looks like motion).
    ph = sub(world, "physics", name="default", type="dartsim")
    sub(ph, "max_step_size", "0.001")
    sub(ph, "real_time_factor", f"{speedup:.1f}")
    sub(world, "gravity", "0 0 -9.80665")

    for fn, nm in (("gz-sim-physics-system", "gz::sim::systems::Physics"),
                   ("gz-sim-user-commands-system",
                    "gz::sim::systems::UserCommands"),
                   ("gz-sim-scene-broadcaster-system",
                    "gz::sim::systems::SceneBroadcaster")):
        sub(world, "plugin", filename=fn, name=nm)
    s = sub(world, "plugin", filename="gz-sim-sensors-system",
            name="gz::sim::systems::Sensors")
    sub(s, "render_engine", "ogre2")

    # --- geodetic origin: MUST equal scenario.origin and SITL's HOME_LOC ---- #
    sc = sub(world, "spherical_coordinates")
    sub(sc, "surface_model", "EARTH_WGS84")
    sub(sc, "world_frame_orientation", "ENU")
    sub(sc, "latitude_deg", origin[0])
    sub(sc, "longitude_deg", origin[1])
    sub(sc, "elevation", "0.0")
    sub(sc, "heading_deg", "0.0")

    # --- environment -------------------------------------------------------- #
    sun = sub(world, "light", name="sun", type="directional")
    sub(sun, "cast_shadows", "false")
    sub(sun, "pose", "0 0 100 0 0 0")
    sub(sun, "diffuse", "1 1 1 1")
    sub(sun, "specular", "0.3 0.3 0.3 1")
    sub(sun, "direction", "-0.3 0.2 -0.9")

    # Freshwater. Marina Bay has been a non-tidal freshwater reservoir since the
    # Marina Barrage closed it off; BumblebeeAS/bb_worlds independently uses
    # 1000 too. Seawater's 1025 would mis-trim anything buoyancy-dependent.
    buoy = sub(world, "plugin", filename="gz-sim-buoyancy-system",
               name="gz::sim::systems::Buoyancy")
    sub(buoy, "uniform_fluid_density", "1000")

    water = sub(world, "model", name="water_plane")
    sub(water, "static", "true")
    wl = sub(water, "link", name="link")
    wv = sub(wl, "visual", name="visual")
    wg = sub(sub(wv, "geometry"), "plane")
    sub(wg, "normal", "0 0 1")
    sub(wg, "size", "2000 2000")
    wm = sub(wv, "material")
    sub(wm, "ambient", "0.12 0.20 0.24 1")
    sub(wm, "diffuse", "0.16 0.28 0.32 1")

    # --- course ------------------------------------------------------------- #
    for i, obs in enumerate(scenario.get("obstacles", [])):
        add_buoy(world, obs, i)

    if markers:
        for i, wp in enumerate(scenario.get("waypoints", [])):
            add_waypoint_marker(world, wp, i)

    # Keep-outs and moving objects are deliberately NOT rendered: they are
    # virtual RoboCommand constructs with no physical presence on the course.
    # Rendering them would tempt a perception stack into "seeing" something the
    # real course does not contain.

    return sdf


def write(sdf, path: Path):
    raw = ET.tostring(sdf, encoding="unicode")
    pretty = minidom.parseString(raw).toprettyxml(indent="  ")
    pretty = "\n".join(ln for ln in pretty.splitlines() if ln.strip())
    path.parent.mkdir(parents=True, exist_ok=True)
    # encoding is explicit: the header comment carries non-ASCII, and the SDF
    # declares itself UTF-8. Without this, write_text uses the platform ANSI
    # codepage on Windows and every XML reader rejects the result.
    path.write_text(pretty + "\n", encoding="utf-8")


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--scenario", help="path to a scenario JSON")
    ap.add_argument("--all", action="store_true",
                    help="every scenario in orchestrator/scenarios/")
    ap.add_argument("--markers", action="store_true",
                    help="render translucent waypoint discs (visual only)")
    ap.add_argument("--speedup", type=float, default=1.0,
                    help="Gazebo real_time_factor; keep equal to SITL SIM_SPEEDUP")
    args = ap.parse_args()

    if args.all:
        files = sorted(SCENARIOS.glob("*.json"))
    elif args.scenario:
        files = [Path(args.scenario)]
    else:
        ap.error("need --scenario or --all")

    for f in files:
        scenario = json.loads(f.read_text())
        sdf = build(scenario, args.markers, args.speedup)
        out = OUT / f"{scenario.get('name', f.stem)}.sdf"
        write(sdf, out)
        n_obs = len(scenario.get("obstacles", []))
        lat, lon = scenario["origin"]
        print(f"  {out.relative_to(REPO)}  ({n_obs} obstacles, origin {lat}, {lon})")

    print("\nGenerated. Launch with:")
    print("  RX26_GZ_WORLD=tools/sim/worlds/<name>.sdf docker/sitl/run_sitl_gazebo.sh")
    return 0


if __name__ == "__main__":
    sys.exit(main())
