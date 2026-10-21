"""ekko_fake_map -- the sim course as the map Ekko would publish, for a radio test.

    python3 -m crusader_sim.ekko_fake_map task1_core                 # BuoyMap as YAML
    python3 -m crusader_sim.ekko_fake_map task1_core --command       # a ros2 topic pub line
    python3 -m crusader_sim.ekko_fake_map task1_core --noise-m 0.5   # each buoy off by < 0.5 m

Runs on any machine with Python and PyYAML (no ROS, no pymavlink), so the line it prints
can be made on the sim PC and pasted on Ekko.

WHAT IT IS FOR. Closing the radio loop with the REAL aircraft before a field test: the sim boat
listens on its RFD radio for Ekko's v3 TUNNEL traffic, and Ekko's telemetry_bridge puts a map
on the air from `/uav/perception/buoy_map`. Publish this map there, on Ekko, and Ekko
transmits the sim course as if its camera had found it, in the SIM's Gazebo world where the
sim boat is. No camera, no flight, no buoys.

    ros2 topic pub -r 1 /uav/perception/buoy_map uav_msgs/msg/BuoyMap '<yaml>'

(Ekko's `ros2` is in the uav_ekko container; `--command` prints exactly that line,
single-quoted so bash cannot touch it.)

POSITIONS ARE THE WORLD'S: every buoy goes through course.enu_to_latlon, the function
gen_world.py places the Gazebo buoys with and sim_uav transmits them with.

WHAT THE UAV SEES. A course keeps each buoy's TRUE beacon (`beacon:`) and separately which
lights are lit (`side_beacon:`, `up_beacon:`, both default on). Ekko's camera looks down, so
it reads the UP beacon in advanced or disruptive tier (only the aircraft can see the route)
and the side beacon at core tier. A buoy whose visible light is off reads OFF, whatever it
means.

HONEST EVIDENCE. The BuoyMap carries the mapper's evidence (samples, lit fraction, spread); a
fake map has none, so those fields stay at zero rather than plausible-looking measurements.
`locked` is true: the state is decided, which is what the search and the radio act on.

Labels are the strings buoy_mapper produces and telemetry_bridge sends: FLASHING_RED,
FLASHING_GREEN, FLASHING_BLUE, SOLID_BLUE, OFF (state + "_" + colour; boat_link.STATES maps
them to the wire codes).
"""
import argparse
import json
import math
import random
import sys

import yaml

try:
    from crusader_sim import course as C
except ImportError:                       # run as a script from a checkout
    import os
    sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
    from crusader_sim import course as C

TOPIC = "/uav/perception/buoy_map"
MSG_TYPE = "uav_msgs/msg/BuoyMap"
#: Only the tiers where the course is visible from above alone.
UP_TIERS = ("advanced", "disruptive")
SEED = 1                                  # fixed, so --noise-m gives the same map every run

#: course beacon -> (Buoy.state, Buoy.colour). The label is state_colour, or the
#: state alone when there is no colour (uav_perception.buoy_tracker.label()).
UAV_STATE = {
    "flash_red": ("FLASHING", "RED"),
    "flash_green": ("FLASHING", "GREEN"),
    "flash_blue": ("FLASHING", "BLUE"),
    "steady_blue": ("SOLID", "BLUE"),
    "off": ("OFF", ""),
}
assert set(UAV_STATE) == set(C.BEACON_STATES)


def uav_view(course):
    """'up' or 'side': which of a buoy's two lights the aircraft reads."""
    return "up" if course.get("tier", "core") in UP_TIERS else "side"


def uav_beacon(beacon, side_on, up_on, view):
    """What the aircraft sees on a buoy: its true beacon if the light it looks at
    is on, else a dark buoy."""
    lit = up_on if view == "up" else side_on
    return beacon if lit else "off"


def noise_offsets(n, radius_m, seed=SEED):
    """n (east, north) offsets in metres, each strictly inside radius_m, the same
    every call. Uniform over the disc."""
    rng = random.Random(seed)
    out = []
    for _ in range(n):
        r = radius_m * math.sqrt(rng.random())
        a = rng.random() * 2.0 * math.pi
        out.append((r * math.cos(a), r * math.sin(a)))
    return out


def buoy_map(course, noise_m=0.0):
    """The BuoyMap for a course dict, as a plain dict in the message's field
    names (ids 1..n in course order)."""
    view = uav_view(course)
    robobuoys = C.buoys(course)
    offsets = noise_offsets(len(robobuoys), noise_m) if noise_m > 0 else [(0.0, 0.0)] * len(robobuoys)
    buoys = []
    for i, ((_name, x, y, beacon, side_on, up_on), (de, dn)) in enumerate(zip(robobuoys, offsets), 1):
        state, colour = UAV_STATE[uav_beacon(beacon, side_on, up_on, view)]
        lat, lon = C.enu_to_latlon(x + de, y + dn, course["origin"])
        buoys.append({
            "id": i, "latitude": lat, "longitude": lon,
            "state": state, "colour": colour,
            "label": "%s_%s" % (state, colour) if colour else state,
            "locked": True,
            "lit_fraction": 0.0, "colour_agreement": 0.0, "samples": 0,
            "flash_transitions": 0, "observed_s": 0.0, "sightings": 0,
            "spread_m": 0.0, "last_seen_age_s": 0.0,
        })
    return {
        "header": {"frame_id": "fake"},
        "buoys": buoys,
        "detections_used": 0, "detections_partial": 0, "detections_low_conf": 0,
        "frames_rejected": 0, "last_reject_reason": "",
        # Ekko's map number is keyed on this: a different course is a new map
        "export_stem": "fake-%s" % course.get("name", "course"),
    }


def as_yaml(msg):
    return yaml.safe_dump(msg, sort_keys=False, default_flow_style=False)


def as_command(msg):
    """`ros2 topic pub -r 1 ...` with the message as ONE single-quoted argument.
    JSON is valid YAML and has no single quote in it to end the quoting early."""
    flow = json.dumps(msg, separators=(", ", ": "))
    assert "'" not in flow, "a single quote in the message would break the shell quoting"
    return "ros2 topic pub -r 1 %s %s '%s'" % (TOPIC, MSG_TYPE, flow)


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("course", nargs="?", default="task1_core",
                    help="a name in crusader_sim/courses/, or a path (default task1_core)")
    ap.add_argument("--command", action="store_true",
                    help="print a complete `ros2 topic pub -r 1` line instead of the YAML")
    ap.add_argument("--noise-m", type=float, default=0.0, metavar="R",
                    help="move each buoy by a fixed-seed random offset under R metres")
    a = ap.parse_args(argv)
    if a.noise_m < 0:
        ap.error("--noise-m must be >= 0")
    course = C.load(a.course)
    msg = buoy_map(course, a.noise_m)
    print(as_command(msg) if a.command else as_yaml(msg).rstrip("\n"))
    print("ekko_fake_map: course %s (%s tier, the aircraft reads the %s beacon), %d buoys%s"
          % (course.get("name", a.course), course.get("tier", "core"), uav_view(course),
             len(msg["buoys"]), ", noise < %g m" % a.noise_m if a.noise_m else ""),
          file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
