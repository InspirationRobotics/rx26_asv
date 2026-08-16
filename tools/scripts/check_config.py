#!/usr/bin/env python3
"""check_config — static guards on the two config files that can ground the boat.

Exit 0 = both files are well-formed and internally consistent. Nonzero = fix
before building. Needs nothing but the standard library plus pyyaml; no ROS, no
test framework, no hardware. Run it from CI, from a dev laptop, or on the Jetson.

    python3 tools/scripts/check_config.py

What it guards, and why each one is here:

1. config/crusader_params.yaml must parse under RCL's rules, which are stricter
   than PyYAML's and fail CLOSED. An illegal file kills every node in the launch
   at rclpy.init(), before a single line of node code runs. This happened on the
   boat on 2026-07-29 and grounded the whole stack, LEDs included — including the
   LEDs, which meant the boat was dark rather than obviously broken.

2. params/working_crusader.params must parse into real parameter names. The
   parameter dumps the team exports differ in WHERE the name sits (Mission
   Planner puts it first, QGroundControl puts it third), and a positional parser
   reads a QGC dump as ~900 copies of a parameter called "1". param_guard would
   then diff that empty-in-practice map against the live vehicle and report a
   clean PASS — a preflight gate that cannot fail is worse than no gate.
"""
import importlib.util
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PARAMS_YAML = REPO / "crusader_bringup" / "config" / "crusader_params.yaml"
PARAMS_BASELINE = REPO / "params" / "working_crusader.params"

# Every node that reads crusader_params.yaml. A section for a node that no longer
# exists is a parameter set nobody reviews, and it reads as a capability the boat
# still has; a node with no section fails at startup instead.
CONFIG_DRIVEN_NODES = {"telemetry_bridge", "led_node",
                       "pixhawk_led_status_node", "rc_heartbeat_watchdog",
                       "oakd_publisher", "buoy_detector", "lidar_cluster_node",
                       "target_tracker", "map_server"}

# Topic names that two sections must agree on, as (producer, param) ->
# (consumer, param). A producer and a consumer that disagree about a topic name
# do not fail: both start, both look healthy, `ros2 topic list` shows two
# plausible names, and the consumer simply never receives anything. That is the
# most expensive kind of green.
TOPIC_PAIRS = (
    (("buoy_detector", "detections_topic"),
     ("target_tracker", "detections_topic")),
    (("lidar_cluster_node", "clusters_topic"),
     ("target_tracker", "clusters_topic")),
    (("target_tracker", "targets_topic"), ("map_server", "targets_topic")),
)

# oakd_publisher and buoy_detector each build the SAME OAK-D pipeline (only one
# runs at a time — the camera admits one client). These params decide the image
# geometry, and depth is aligned to the RGB camera at exactly this size, so the
# RGB intrinsics are only valid on the depth image while both agree. Let them
# drift and nothing fails: ranges measured under one node just quietly stop
# meaning what they meant under the other.
CAMERA_PARAMS = ("fps", "isp_denominator", "sync_threshold_ms",
                 "subpixel", "lr_check")

failures = []


def fail(check, detail):
    failures.append((check, detail))
    print(f"[FAIL] {check}: {detail}")


def ok(check, detail=""):
    print(f"[ ok ] {check}" + (f": {detail}" if detail else ""))


# ---------------------------------------------------------------- params YAML

def check_params_yaml():
    import yaml
    try:
        text = PARAMS_YAML.read_text(encoding="utf-8")
        cfg = yaml.safe_load(text)
    except FileNotFoundError:
        fail("params yaml present", f"{PARAMS_YAML} not found")
        return
    except yaml.YAMLError as e:
        fail("params yaml parses", str(e))
        return

    # Rule 1: every top-level key is a node whose only child is ros__parameters.
    bad = [name for name, section in cfg.items()
           if not isinstance(section, dict) or list(section) != ["ros__parameters"]]
    if bad:
        fail("rcl rule 1: ros__parameters",
             f"sections with a wrong shape: {bad} — a bare top-level value gives "
             '"Cannot have a value before ros__parameters"')
    else:
        ok("rcl rule 1: every section is <node>/ros__parameters")

    # Rule 2: no YAML anchors or aliases anywhere (rcl parses tokens, not docs).
    anchored = [n for n, line in enumerate(text.splitlines(), 1)
                if "&" in line.split("#", 1)[0] or "*" in line.split("#", 1)[0]]
    if anchored:
        fail("rcl rule 2: no anchors/aliases",
             f"lines {anchored} — write the value out literally instead")
    else:
        ok("rcl rule 2: no YAML anchors/aliases")

    # Section set matches the shipped nodes, in both directions.
    sections = set(cfg) - {"shared"}
    if sections != CONFIG_DRIVEN_NODES:
        missing = CONFIG_DRIVEN_NODES - sections
        extra = sections - CONFIG_DRIVEN_NODES
        fail("config sections match the shipped nodes",
             f"missing={sorted(missing)} stale={sorted(extra)} "
             "(update CONFIG_DRIVEN_NODES here if a node was added or removed)")
    else:
        ok("config sections match the shipped nodes")

    # Shared values that must stay equal across sections (rcl forbids anchors,
    # so they are written out literally and pinned here instead).
    #
    # Checked in BOTH directions: the bridge's own republish budget, and every
    # consumer that decides when to stop acting on the last pose it saw. The
    # YAML says "any future consumer must use the same number" — a comment
    # cannot enforce that, and a consumer trusting a pose for longer than the
    # bridge vouches for it is the frozen-pose failure the value exists to
    # prevent. This finds the drift while it is still a diff.
    try:
        want = cfg["shared"]["ros__parameters"]["pose_timeout_s"]
        pinned = [("telemetry_bridge", "stream_timeout_s")]
        pinned += [(name, "pose_timeout_s") for name in sorted(cfg)
                   if name != "shared"
                   and "pose_timeout_s" in cfg[name]["ros__parameters"]]
        differing = {f"{n}.{k}": cfg[n]["ros__parameters"][k]
                     for n, k in pinned if cfg[n]["ros__parameters"][k] != want}
        if differing:
            fail("shared pose_timeout_s",
                 f"shared={want} but {differing}; a consumer trusting a pose "
                 "longer than the bridge vouches for it is the frozen-pose "
                 "failure this value exists to prevent")
        else:
            ok("shared pose_timeout_s consistent",
               f"{len(pinned)} section(s) pinned at {want}")
    except KeyError as e:
        fail("shared pose_timeout_s", f"missing key {e}")

    # Producer/consumer topic names (see TOPIC_PAIRS).
    for (pn, pk), (cn, ck) in TOPIC_PAIRS:
        try:
            a, b = cfg[pn]["ros__parameters"][pk], cfg[cn]["ros__parameters"][ck]
        except KeyError as e:
            fail(f"topic {pn}.{pk} -> {cn}.{ck}", f"missing key {e}")
            continue
        if a != b:
            fail(f"topic {pn}.{pk} -> {cn}.{ck}",
                 f"{a!r} != {b!r} — both nodes start, neither complains, and "
                 "the consumer receives nothing")
        else:
            ok(f"topic {pn}.{pk} -> {cn}.{ck}", a)

    # The two OAK-D nodes must describe the same camera (see CAMERA_PARAMS).
    try:
        publisher = cfg["oakd_publisher"]["ros__parameters"]
        detector = cfg["buoy_detector"]["ros__parameters"]
        differing = {name: (publisher[name], detector[name])
                     for name in CAMERA_PARAMS
                     if publisher[name] != detector[name]}
        if differing:
            fail("oak camera params consistent",
                 f"oakd_publisher vs buoy_detector differ: {differing} — depth "
                 "is aligned at this geometry, so a range means different "
                 "things under each node")
        else:
            ok("oak camera params consistent across both OAK-D nodes")
    except KeyError as e:
        fail("oak camera params consistent", f"missing key {e}")


# ------------------------------------------------------------ param baseline

def check_param_baseline():
    if not PARAMS_BASELINE.exists():
        fail("param baseline present",
             f"{PARAMS_BASELINE} not found — export the boat's params from QGC "
             "and commit them; preflight's param diff cannot run without it")
        return

    spec = importlib.util.spec_from_file_location(
        "param_guard", Path(__file__).parent / "param_guard.py")
    pg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pg)

    params = pg.load_param_file(PARAMS_BASELINE)
    if "1" in params or len(params) < 100:
        fail("param baseline parses",
             f"{len(params)} params parsed"
             + (" and one of them is named '1'" if "1" in params else "")
             + " — positional mis-parse; see load_param_file")
        return
    ok("param baseline parses", f"{len(params)} params")

    # The frame/heading config the boat cannot fly without. These are not tuning
    # values: FRAME_TYPE=2 is the OmniX mixer, and COMPASS_USE=0 with
    # EK3_SRC1_YAW=2 is the dual-antenna GPS-yaw setup (there is no compass).
    expected = {"FRAME_TYPE": 2.0, "COMPASS_USE": 0.0, "EK3_SRC1_YAW": 2.0,
                "ARMING_CHECK": 1.0, "ARMING_REQUIRE": 1.0}
    for name, want in expected.items():
        got = params.get(name)
        if got is None:
            fail(f"baseline {name}", "absent from the dump")
        elif abs(got - want) > 1e-6:
            fail(f"baseline {name}", f"expected {want:g}, dump has {got:g}")
    if not any(f[0].startswith("baseline ") for f in failures):
        ok("baseline holds the documented frame/heading/arming config")


def main():
    print(f"repo: {REPO}")
    check_params_yaml()
    check_param_baseline()
    print()
    if failures:
        print(f"==== CONFIG CHECK: FAIL ({len(failures)} problem(s)) ====")
        sys.exit(1)
    print("==== CONFIG CHECK: PASS ====")


if __name__ == "__main__":
    main()
