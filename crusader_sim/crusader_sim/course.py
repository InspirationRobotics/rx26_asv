"""course — one reading of a courses/*.yaml, shared by every sim node.

The world generator, the camera oracle and the UAV stand-in must agree on where
each buoy is and what it shows; they agree because they all read it through
this module.
"""
import math
import os

import yaml

from crusader_sim.paths import courses_dir

# ArduPilot's LOCATION_SCALING_FACTOR is metres per 1e-7 degree:
# 0.011131884502145034 * 1e7 = 111318.845 m/deg
EARTH_M_PER_DEG = 0.011131884502145034 * 1e7

# course beacon state -> the detector label the boat's stack expects
# (bench_world_model.BEACON_OF / bt_runner beaconFromLabel(): the substrings
# red, green, blue, off/black; flashing_blue = ENTRY, steady_blue = EXIT)
LABEL_OF = {
    "flash_red": "red_buoy",
    "flash_green": "green_buoy",
    "flash_blue": "flashing_blue_buoy",
    "steady_blue": "steady_blue_buoy",
    "off": "black_buoy",
}
# course beacon state -> rxl_codec beacon enum on the radio
RXL_BEACON_OF = {"off": 1, "flash_red": 2, "flash_green": 3,
                 "flash_blue": 4, "steady_blue": 5}


def resolve(course):
    """A name ("task1_core") or a path."""
    if os.path.isfile(course):
        return course
    p = os.path.join(courses_dir(), course if course.endswith(".yaml") else course + ".yaml")
    if not os.path.isfile(p):
        raise FileNotFoundError(f"no course {course!r} (looked for {p})")
    return p


def load(course):
    with open(resolve(course), encoding="utf-8") as f:
        c = yaml.safe_load(f)
    c.setdefault("elements", [])
    return c


def enu_to_latlon(east_m, north_m, origin):
    """The TRUE lat/lon of a point in the sim world.

    Deliberately NOT crusader_common.geo (M_PER_DEG = 111139). In the sim the
    ground truth is whatever SITL's GPS reports, and SITL turns Gazebo metres
    into lat/lon with ArduPilot's Location::offset — 1/LOCATION_SCALING_FACTOR
    = 111318.845 m/deg. Placing buoys with geo's constant would put every one
    0.16 % (~8 cm at 50 m) away from where the boat's GPS says the same water
    is: an error the sim manufactured, not one the boat has. Whatever geo's
    own constant costs the boat's mapping still shows up, as it would for
    real."""
    lat0, lon0 = origin["lat"], origin["lon"]
    lat = lat0 + north_m / EARTH_M_PER_DEG
    lon = lon0 + east_m / (EARTH_M_PER_DEG * math.cos(math.radians(lat0)))
    return lat, lon


def buoys(course):
    """[(name, x, y, beacon_state, side_on, up_on)] for every RoboBuoy."""
    out = []
    for e in course["elements"]:
        if e.get("type") == "robobuoy":
            out.append((e["name"], float(e["x"]), float(e["y"]),
                        e.get("beacon", "off"),
                        bool(e.get("side_beacon", True)),
                        bool(e.get("up_beacon", True))))
    return out
