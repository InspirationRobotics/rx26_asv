#!/usr/bin/env python3
"""check_config — static guards on the files that can ground the boat.

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

2. Every package.xml must be well-formed XML, and every package must be in
   crusader_bringup's exec_depends. Both are build-time failures that only
   happen ON THE JETSON, after a copy: rebuild.sh dies with a CMake wall of
   text (a stray "--" inside an XML comment is illegal and did exactly this on
   2026-09-05), or worse, the package is silently NOT BUILT by
   "colcon build (packages-up-to crusader_bringup)" and the failure surfaces
   much later as `ros2 run` not finding a node. Both are catchable on a laptop
   in milliseconds.

3. params/working_crusader.params must parse into real parameter names. The
   parameter dumps the team exports differ in WHERE the name sits (Mission
   Planner puts it first, QGroundControl puts it third), and a positional parser
   reads a QGC dump as ~900 copies of a parameter called "1". param_guard would
   then diff that empty-in-practice map against the live vehicle and report a
   clean PASS — a preflight gate that cannot fail is worse than no gate.
"""
import importlib.util
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PARAMS_YAML = REPO / "crusader_bringup" / "config" / "crusader_params.yaml"
PARAMS_BASELINE = REPO / "params" / "working_crusader.params"

# Every node that reads crusader_params.yaml. A section for a node that no longer
# exists is a parameter set nobody reviews, and it reads as a capability the boat
# still has; a node with no section fails at startup instead.
#
# WRITTEN AS ONE STRING, SPLIT ON WHITESPACE, and that is deliberate. As a set
# literal of quoted names, a single missing comma is not an error: Python joins
# the two adjacent strings, so {"a" "b", "c"} silently becomes {"ab", "c"}. The
# check then reports the wrong node as stale and the reader goes looking in the
# YAML for a bug that is in this file. Splitting a string cannot do that.
CONFIG_DRIVEN_NODES = set("""
    telemetry_bridge
    led_node
    pixhawk_led_status_node
    rc_heartbeat_watchdog
    ocs_client
    oakd_publisher
    buoy_detector
    oak_detector
    lidar_cluster_node
    proximity_bridge
    target_tracker
    ground_station
    safe_passage_server
    bt_runner_node
""".split())

# Topic names that two sections must agree on, as (producer, param) ->
# (consumer, param). A producer and a consumer that disagree about a topic name
# do not fail: both start, both look healthy, `ros2 topic list` shows two
# plausible names, and the consumer simply never receives anything. That is the
# most expensive kind of green.
TOPIC_PAIRS = (
    (("buoy_detector", "detections_topic"),
     ("target_tracker", "detections_topic")),
    (("oak_detector", "detections_topic"),
     ("target_tracker", "detections_topic")),
    (("lidar_cluster_node", "clusters_topic"),
     ("target_tracker", "clusters_topic")),
    # proximity_bridge is the second consumer of the LiDAR clusters. If it
    # disagrees with the producer the boat drives with avoidance silently blind
    # while every node reports healthy — see the paragraph above.
    (("lidar_cluster_node", "clusters_topic"),
     ("proximity_bridge", "clusters_topic")),
    (("target_tracker", "targets_topic"), ("ground_station", "targets_topic")),
)

# Ports that must stay distinct: two servers cannot bind one socket, and the
# loser fails at startup with an address-in-use that reads like a crash. The
# ground station EMBEDS the viewers rather than proxying them, so it has to
# avoid their ports rather than share them.
PORT_OWNERS = (("buoy_detector", "stream_port"), ("ground_station", "port"))

# The three nodes that each build the SAME OAK-D pipeline (only one runs at a
# time — the camera admits one client). Depth is aligned to the RGB camera at
# exactly this geometry, so the RGB intrinsics are only valid on the depth image
# while all three agree. Let them drift and nothing fails: ranges measured under
# one node just quietly stop meaning what they meant under another.
CAMERA_NODES = ("oakd_publisher", "buoy_detector", "oak_detector")

# sync_threshold_ms, subpixel and lr_check used to be here. They are now module
# constants in crusader_perception/oak_pipeline.py, shared by all three nodes by
# construction, so there is nothing left for this check to compare. What remains
# is the frame geometry, which is still per-node YAML.
CAMERA_PARAMS = ("fps", "isp_denominator")

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
             "(missing = named here but no YAML section; stale = a YAML section "
             "this list does not know about. Check `ros2 pkg executables` before "
             "deleting either — the node may still ship and just not be running)")
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

    # Distinct HTTP ports (see PORT_OWNERS).
    seen = {}
    clash = False
    for node, key in PORT_OWNERS:
        try:
            port = cfg[node]["ros__parameters"][key]
        except KeyError as e:
            fail("http ports distinct", f"missing key {e}")
            clash = True
            continue
        if port in seen:
            fail("http ports distinct",
                 f"{node}.{key} and {seen[port]} both bind {port} — the second "
                 "to start dies with address-in-use, which reads like a crash")
            clash = True
        seen[port] = f"{node}.{key}"
    if not clash:
        ok("http ports distinct", ", ".join(f"{p}={n}" for p, n in
                                            sorted(seen.items())))

    # The mission's task token must be a name RoboCommand's RxTask enum has.
    # ParseDict REJECTS THE WHOLE FRAME on an unknown enum name, so a typo here
    # does not degrade one field -- it stops every heartbeat reaching
    # RoboCommand for as long as the mission runs, which is exactly the window
    # that is being scored.
    try:
        spec2 = importlib.util.spec_from_file_location(
            "ocs_link", REPO / "crusader_groundstation" /
            "crusader_groundstation" / "ocs_link.py")
        link = importlib.util.module_from_spec(spec2)
        spec2.loader.exec_module(link)
        bad = {}
        for who in ("safe_passage_server", "bt_runner_node"):
            token = cfg[who]["ros__parameters"]["task_token"]
            if token not in link.RX_TASKS:
                bad[who] = token
        if bad:
            fail("task_token is a real RxTask",
                 f"{bad} not in {sorted(link.RX_TASKS)} — protobuf's ParseDict "
                 "rejects the whole frame, so every heartbeat sent during the "
                 "mission is lost, not just this field")
        else:
            ok("task_token is a real RxTask",
               cfg["bt_runner_node"]["ros__parameters"]["task_token"])
    except KeyError as e:
        fail("task_token is a real RxTask", f"missing key {e}")

    # All three OAK-D nodes must describe the same camera (see CAMERA_NODES).
    # Compared against the first node in the list rather than pairwise, so the
    # message names one reference and the odd ones out.
    reference = CAMERA_NODES[0]
    try:
        differing = {}
        for name in CAMERA_PARAMS:
            want = cfg[reference]["ros__parameters"][name]
            for node in CAMERA_NODES[1:]:
                got = cfg[node]["ros__parameters"][name]
                if got != want:
                    differing[f"{node}.{name}"] = (got, f"{reference}={want}")
        if differing:
            fail("oak camera params consistent",
                 f"{differing} — depth is aligned at this geometry, so a range "
                 "means different things under each node")
        else:
            ok("oak camera params consistent",
               f"{len(CAMERA_NODES)} nodes agree on {', '.join(CAMERA_PARAMS)}")
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


# --------------------------------------------------------------- package.xml

def check_packages():
    import xml.etree.ElementTree as ET

    manifests = sorted(REPO.glob("*/package.xml"))
    if not manifests:
        fail("package manifests found", f"no */package.xml under {REPO}")
        return

    names = {}
    for m in manifests:
        try:
            root = ET.parse(m).getroot()
        except ET.ParseError as e:
            fail(f"{m.parent.name}/package.xml parses",
                 f"{e} — colcon dies on this with a CMake wall of text. A "
                 'double hyphen inside an XML comment is the usual cause; XML '
                 "forbids it, so a colcon flag written out in a comment breaks "
                 "the manifest that mentions it")
            continue
        node = root.find("name")
        if node is None or not (node.text or "").strip():
            fail(f"{m.parent.name}/package.xml has a name", "no <name> element")
            continue
        pkg = node.text.strip()
        if pkg != m.parent.name:
            fail(f"{m.parent.name}/package.xml name matches its directory",
                 f"declares {pkg!r}")
        names[pkg] = root
    if not any(f[0].endswith("parses") for f in failures):
        ok("every package.xml is well-formed", f"{len(names)} manifests")

    if "crusader_bringup" not in names:
        fail("crusader_bringup manifest", "not found or did not parse")
        return

    # REACHABILITY, not direct declaration. `colcon build (packages-up-to
    # crusader_bringup)` builds the transitive closure, so crusader_common and
    # crusader_msgs are built via the packages that depend on them and do NOT
    # need to be listed in bringup. Checking for direct listing instead would
    # report those two as broken forever, and a check that cries wolf is a check
    # people stop reading.
    DEP_TAGS = ("depend", "exec_depend", "build_depend", "buildtool_depend",
                "build_export_depend", "test_depend")
    deps = {pkg: {e.text.strip() for e in root.iter()
                  if e.tag in DEP_TAGS and e.text}
            for pkg, root in names.items()}

    reachable, stack = set(), ["crusader_bringup"]
    while stack:
        pkg = stack.pop()
        if pkg in reachable or pkg not in deps:
            continue
        reachable.add(pkg)
        stack.extend(deps[pkg])

    orphans = sorted(set(names) - reachable)
    if orphans:
        fail("every package is reachable from crusader_bringup",
             f"{orphans} — nothing depends on it, so it is silently NOT BUILT "
             "and the failure surfaces later as `ros2 run` not finding a node. "
             "Add it to crusader_bringup's exec_depends")
    else:
        ok("every package is reachable from crusader_bringup",
           f"{len(reachable)} packages in the closure")


# ------------------------------------------------------- msg/action hygiene

def check_interfaces():
    """Behaviour trees that will not load, and messages rosidl chokes on.

    Behaviour-tree XML is checked here too: a tree that does not parse fails at
    RUNTIME, when a goal arrives, which on the boat means at the dock.

    rosidl feeds every .msg/.action through an empy IDL template. A trailing
    backslash in a COMMENT makes that template fail to decode and the build dies
    with

        UnicodeDecodeError processing template 'struct.idl.em'

    which names neither the file nor the line. Cost a full build round trip on
    2026-09-05, on a shell command wrapped across two lines the way anyone would
    write it.
    """
    # Behaviour trees are XML and get the XML rules, including the one that
    # keeps biting: a double hyphen is ILLEGAL inside an <!-- comment -->. It
    # has now broken a package.xml, a Dockerfile and a behaviour tree in this
    # repo, each time with an error that names a line and not a cause.
    for tree in sorted(REPO.glob("*/behavior_trees/*.xml")):
        try:
            ET.parse(tree)
        except ET.ParseError as e:
            fail(f"{tree.parent.parent.name}/{tree.name} parses",
                 f"{e} — if this is inside a comment, a DOUBLE HYPHEN is "
                 "illegal there; XML forbids it")
        else:
            ok(f"{tree.parent.parent.name}/{tree.name} parses")

    files = [f for pattern in ("*/msg/*.msg", "*/action/*.action", "*/srv/*.srv")
             for f in sorted(REPO.glob(pattern))]
    if not files:
        fail("interface files found", "no .msg/.action/.srv under the repo")
        return
    bad = []
    for f in files:
        for n, line in enumerate(f.read_text(encoding="utf-8").splitlines(), 1):
            if line.endswith("\\"):
                bad.append(f"{f.parent.parent.name}/{f.parent.name}/{f.name}:{n}")
    if bad:
        fail("no trailing backslash in an interface file",
             f"{bad} — rosidl's IDL template fails to decode these, and the "
             "build error names neither the file nor the line")
    else:
        ok("no trailing backslash in an interface file", f"{len(files)} files")


def main():
    print(f"repo: {REPO}")
    check_params_yaml()
    check_packages()
    check_interfaces()
    check_param_baseline()
    print()
    if failures:
        print(f"==== CONFIG CHECK: FAIL ({len(failures)} problem(s)) ====")
        sys.exit(1)
    print("==== CONFIG CHECK: PASS ====")


if __name__ == "__main__":
    main()