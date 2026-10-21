#!/usr/bin/env python3
"""check_config — static guards on the files that can ground the boat.

Exit 0 = both files are well-formed and internally consistent. Nonzero = fix
before building. Needs nothing but the standard library plus pyyaml; no ROS, no
test framework, no hardware. Run it from CI, from a dev laptop, or on the Jetson.

    python3 tools/scripts/check_config.py

What it guards, and why each one is here:

1. config/crusader_params.yaml must parse under RCL's rules, which are stricter
   than PyYAML's and fail CLOSED: an illegal file kills every node in the launch
   at rclpy.init(), LEDs included, so the boat is dark rather than obviously
   broken. Every key in it must be a parameter its node declares, of the declared
   type: ROS ignores a misspelt key without a word and refuses a wrong type (1 for
   a double) at startup.

2. Every package.xml must be well-formed XML, and every package must be in
   crusader_bringup's exec_depends. Both fail only ON THE JETSON, after a copy:
   rebuild.sh dies with a CMake wall of text (a stray "--" inside an XML comment
   is illegal), or worse, "colcon build (packages-up-to crusader_bringup)"
   silently does NOT BUILD the package and `ros2 run` later cannot find a node.
   Both are catchable on a laptop in milliseconds.

3. params/working_crusader.params must parse into real parameter names. Mission
   Planner dumps put the name first, QGroundControl third, and a positional
   parser reads a QGC dump as ~900 copies of a parameter called "1". param_guard
   would then report a clean PASS — a gate that cannot fail is worse than none.

4. The Nav2 avoidance numbers must agree across the three files that each hold
   part of them (crusader_params.yaml, crusader_nav's nav2_params.yaml, the
   autopilot baseline). Drift fails silently: the BT plans a 0.8 m clearance the
   costmap does not enforce, or publishes hazards on a topic the layer is not
   listening to, and the boat looks healthy throughout. See check_nav.
"""
import ast
import functools
import importlib
import importlib.util
import math
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PARAMS_YAML = REPO / "crusader_bringup" / "config" / "crusader_params.yaml"
PARAMS_BASELINE = REPO / "params" / "working_crusader.params"
NAV2_PARAMS = REPO / "crusader_nav" / "config" / "nav2_params.yaml"

# Every node that reads crusader_params.yaml, and the source that declares its parameters
# (PARAMS in a Python node; declare_parameter and NavParams in bt_runner_node). A section
# for a node not listed here is a parameter set nobody reviews. squirt_cal is a bench
# tool whose loader insists on every key itself (squirt_core.load_config).
NODE_SOURCES = {
    "telemetry_bridge": "crusader_behavior/crusader_behavior/telemetry_bridge.py",
    "led_node": "crusader_behavior/crusader_behavior/led_node.py",
    "led_status_node": "crusader_behavior/crusader_behavior/led_status_node.py",
    "rc_watchdog": "crusader_behavior/crusader_behavior/rc_watchdog.py",
    "ocs_client": "crusader_groundstation/crusader_groundstation/ocs_client_node.py",
    "gcs_bridge": "crusader_groundstation/crusader_groundstation/gcs_bridge.py",
    "camera_node": "crusader_perception/crusader_perception/camera_node.py",
    "lidar_node": "crusader_perception/crusader_perception/lidar_node.py",
    "target_tracker": "crusader_world_model/crusader_world_model/target_tracker_node.py",
    "nav_frames_node": "crusader_nav/crusader_nav/nav_frames_node.py",
    "rfd_link_node": "crusader_link/crusader_link/rfd_link_node.py",
    "bt_runner_node": "crusader_bt/src/bt_runner_node.cpp",
    "squirt_cal": None,
}
SHARED = "/**"          # rcl's every-node section: one value for each node that declares it

# Packages deliberately NOT in crusader_bringup's closure: they run on the laptop
# (in WSL) and must never be built on the boat. Without this the reachability
# check below reports them as orphans ("silently not built" — wanted here).
SIM_ONLY_PACKAGES = {"crusader_sim"}

# Topic names two sections must agree on, as (producer, param) -> (consumer,
# param). A mismatch does not fail: both start healthy, `ros2 topic list` shows
# two plausible names, and the consumer never receives anything — the most
# expensive kind of green.
# Ports that must stay distinct: the loser of a bind fails at startup with an
# address-in-use that reads like a crash.
PORT_OWNERS = (("gcs_bridge", "port"), ("squirt_cal", "port"))
GCS_BRIDGE = REPO / "crusader_groundstation" / "crusader_groundstation" / "gcs_bridge.py"

OCS_LINK = (REPO / "crusader_groundstation" / "crusader_groundstation"
            / "ocs_link.py")


def load_ocs_link():
    """ocs_link.py as a module, straight off the source tree. It imports nothing
    but the standard library, which is what lets this run before any build."""
    spec = importlib.util.spec_from_file_location("ocs_link", OCS_LINK)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


failures = []


def fail(check, detail):
    failures.append((check, detail))
    print(f"[FAIL] {check}: {detail}")


def ok(check, detail=""):
    print(f"[ ok ] {check}" + (f": {detail}" if detail else ""))


# ------------------------------------------------------------- ocs_client

def check_ocs_client(ocs):
    """Where the OCS link may be aimed (docs: OCS/docs/VEHICLE_LINK.md, Discovery).

    `ocs` is ocs_client's effective parameters. The boat must never be pointed at
    RoboCommand's course networks, and a bad fallback list fails quietly: ocs_client
    drops the entry with one log line and waits for a beacon that may never come. The
    rules are ocs_link's own (check_fallback_hosts, on_course_network), so file and
    code cannot disagree.
    """
    try:
        host, port = ocs["ocs_host"], ocs["ocs_port"]
    except KeyError as e:
        fail("ocs_client target", f"missing key {e}")
        return
    try:
        link = load_ocs_link()
    except Exception as e:
        fail("ocs_client target", f"could not import ocs_link: {e}")
        return

    problems = []
    auto = str(host).strip().lower() == link.AUTO_HOST
    if not auto:
        kept, bad = link.check_fallback_hosts([host])
        problems += [f"ocs_host: {b}" for b in bad]
    else:
        disc = ocs.get("ocs_discovery_port")
        fallback = ocs.get("ocs_fallback_hosts")
        if isinstance(disc, bool) or not isinstance(disc, int) or not (
                disc == 0 or 1024 <= disc <= 65535):
            problems.append(f"ocs_discovery_port {disc!r} must be 0 (off) or "
                            "1024-65535")
        elif disc == port:
            problems.append(f"ocs_discovery_port equals ocs_port ({port})")
        elif 14540 <= disc <= 14559:
            problems.append(f"ocs_discovery_port {disc} is inside the MAVLink "
                            "UDP block (1454x aircraft, 1455x boat)")
        if not isinstance(fallback, list):
            problems.append("ocs_fallback_hosts must be a list of addresses")
            fallback = []
        kept, bad = link.check_fallback_hosts(fallback)
        problems += [f"ocs_fallback_hosts: {b}" for b in bad]
        if len(kept) != len(fallback) and not bad:
            problems.append("ocs_fallback_hosts repeats an address")
        if not kept and disc == 0:
            problems.append("ocs_discovery_port is 0 and ocs_fallback_hosts is "
                            "empty -- the link has nowhere to look and never "
                            "connects")
        elif not kept:
            problems.append("ocs_fallback_hosts is empty -- if the beacon is "
                            "not heard (an access point that drops broadcasts) "
                            "the link never connects")
    if problems:
        fail("ocs_client target", "; ".join(problems))
    else:
        ok("ocs_client target",
           "auto: beacon UDP %s, fallback %s" % (disc, ", ".join(kept)) if auto
           else f"fixed {host}:{port}")


# ---------------------------------------------------------------- params YAML

def load_params_yaml():
    """crusader_params.yaml parsed, or None after reporting why not."""
    import yaml
    try:
        return yaml.safe_load(PARAMS_YAML.read_text(encoding="utf-8"))
    except FileNotFoundError:
        fail("params yaml present", f"{PARAMS_YAML} not found")
    except yaml.YAMLError as e:
        fail("params yaml parses", str(e))
    return None


def check_params_yaml():
    cfg = load_params_yaml()
    if cfg is None:
        return
    text = PARAMS_YAML.read_text(encoding="utf-8")

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
    # `/**:` is rcl's wildcard, not an alias.
    anchored = [n for n, line in enumerate(text.splitlines(), 1)
                if re.search(r"(^|[\s\[{,:])[&*]\w", line.split("#", 1)[0])]
    if anchored:
        fail("rcl rule 2: no anchors/aliases",
             f"lines {anchored} — write the value out literally instead")
    else:
        ok("rcl rule 2: no YAML anchors/aliases")

    sections = set(cfg) - {SHARED}
    if sections - set(NODE_SOURCES):
        fail("config sections are shipped nodes",
             f"{sorted(sections - set(NODE_SOURCES))}: a section NODE_SOURCES does not "
             "know. Check `ros2 pkg executables` before deleting it — the node may still "
             "ship and just not be running")
    else:
        ok("config sections are shipped nodes", f"{len(sections)} sections")

    check_declared(cfg)

    # The pose age, one value in /** for every node that declares pose_timeout_s; the two
    # stream_timeout_s are the bridge's republish budget and the runner's stop-acting age.
    # A consumer trusting a pose longer than the bridge vouches for it is the frozen-pose
    # failure.
    want = cfg.get(SHARED, {}).get("ros__parameters", {}).get("pose_timeout_s")
    pinned = {f"{n}.stream_timeout_s": _effective(cfg, n)["stream_timeout_s"]
              for n in ("telemetry_bridge", "bt_runner_node")}
    _ensure("pose timeout consistent",
            (want is None and f"{SHARED} has no pose_timeout_s")
            or ({k: v for k, v in pinned.items() if v != want}
                and f"{SHARED}.pose_timeout_s = {want} but {pinned}"),
            f"{want} s everywhere")

    # One camera, one mounting: bt_runner_node places the Task 3 bays and
    # target_tracker the buoys, each with its own copy of the extrinsic. If they
    # differ the two worlds disagree and neither looks wrong on its own.
    keys = ("cam_x", "cam_y", "cam_yaw_deg", "cam_pitch_deg")
    tt, bt = _effective(cfg, "target_tracker"), _effective(cfg, "bt_runner_node")
    differing = {k: (tt[k], bt[k]) for k in keys if tt[k] != bt[k]}
    _ensure("camera extrinsic consistent",
            differing and f"target_tracker vs bt_runner_node {differing}",
            "target_tracker and bt_runner_node agree on " + ", ".join(keys))

    # A leg-topic mismatch is a blank path on the map, not an error.
    _same("leg topic bt_runner -> gcs_bridge",
          ("bt_runner_node.nav_leg_topic", bt["nav_leg_topic"]),
          ("gcs_bridge.LEG_TOPIC", _module_literals(GCS_BRIDGE)["LEG_TOPIC"]),
          "the map shows no planned path")

    # UDP port discipline on the MAVLink links. The boat owns 1455x so a --out
    # line names its vehicle. The flight stack's port is never taken by anything
    # else: a udpin bind STEALS datagrams, and the displaced consumer sees
    # silence, not an error -- /crsd/pose goes stale and the mission stops.
    def _udp_port(endpoint):
        """Port out of 'udpin:host:port' / 'udp:host:port', or None."""
        m = re.search(r":(\d+)\s*$", str(endpoint))
        return int(m.group(1)) if m else None

    mav_ep = _effective(cfg, "telemetry_bridge")["mav_endpoint"]
    rxl_ep = _effective(cfg, "rfd_link_node")["rfd_endpoint"]
    # A NON-UDP endpoint is the RFD900 on its FTDI cable, owned directly by
    # the link node as the mesh needs. No port to check then -- only that it
    # names a device at all.
    net = str(rxl_ep).startswith(("udp:", "udpin:", "udpout:", "tcp:"))
    mav_p, rxl_p = _udp_port(mav_ep), _udp_port(rxl_ep)
    if not net and str(rxl_ep).startswith("/dev/"):
        ok("mavlink udp ports", f"rfd_link on the radio at {rxl_ep}")
    elif not net:
        fail("mavlink udp ports",
             f"rfd_endpoint {rxl_ep!r} is neither a udp endpoint nor a "
             "/dev device; pymavlink would read it as a file name and the "
             "link would be silent rather than refused")
    elif mav_p is None or rxl_p is None:
        fail("mavlink udp ports",
             f"could not read a port from {mav_ep!r} / {rxl_ep!r}")
    elif mav_p == rxl_p:
        fail("mavlink udp ports",
             f"rfd_link_node and telemetry_bridge both on {mav_p} — the "
             "second to bind steals the first's datagrams and the boat "
             "loses pose with nothing logging an error")
    elif not (14550 <= rxl_p <= 14559):
        fail("mavlink udp ports",
             f"rfd_endpoint port {rxl_p} is outside the boat's 1455x range "
             "(the aircraft owns 1454x)")
    elif not str(rxl_ep).startswith("udpin:"):
        fail("mavlink udp ports",
             f"rfd_endpoint {rxl_ep!r} must be udpin: to BIND and listen; "
             "'udp:' is an outbound client that receives nothing, silently")
    else:
        ok("mavlink udp ports", f"telemetry_bridge {mav_p}, rfd_link {rxl_p}")

    # Distinct HTTP ports (see PORT_OWNERS).
    seen = {}
    for node, key in PORT_OWNERS:
        port = _effective(cfg, node)[key]
        if port in seen:
            fail("http ports distinct",
                 f"{node}.{key} and {seen[port]} both bind {port} — the second "
                 "to start dies with address-in-use, which reads like a crash")
            break
        seen[port] = f"{node}.{key}"
    else:
        ok("http ports distinct", ", ".join(f"{p}={n}" for p, n in sorted(seen.items())))

    # Every TaskAttempt token in a built-in tree must be in RoboCommand's RxTask enum: ParseDict
    # rejects the whole heartbeat frame on an unknown enum name.
    try:
        link = load_ocs_link()
        trees = sorted((REPO / "crusader_bt" / "behavior_trees").rglob("*.xml"))
        tokens = {t for f in trees for t in re.findall(r'TaskAttempt token="([A-Z_]+)"', f.read_text())}
        bad = sorted(tokens - set(link.RX_TASKS))
        if bad:
            fail("TaskAttempt tokens are real RxTasks", f"{bad} not in {sorted(link.RX_TASKS)}")
        else:
            ok("TaskAttempt tokens are real RxTasks", ", ".join(sorted(tokens)))
    except Exception as e:  # noqa: BLE001
        fail("TaskAttempt tokens are real RxTasks", f"{type(e).__name__}: {e}")

    check_ocs_client(_effective(cfg, "ocs_client"))

    # camera_node's camera controls against oak_controls' ranges and choices: one outside
    # them stops the node at startup (ROS refuses it at declaration).
    oak_controls = _import("crusader_perception", "oak_controls")
    cam = _effective(cfg, "camera_node")
    problems = []
    for control in oak_controls.ALL:
        value = cam[control.name]
        if control.choices and str(value).lower() not in control.choices:
            problems.append(f"{control.name}={value!r} is not one of " + "/".join(control.choices))
        if control.lo is not None and not (control.lo <= value <= control.hi):
            problems.append(f"{control.name}={value} is outside [{control.lo}, {control.hi}]")
    _ensure("oak camera profile", "; ".join(problems),
            f"{len(oak_controls.ALL)} controls in range")


def check_declared(cfg):
    """Every YAML key is a parameter its node declares, of the declared type.

    ROS ignores an override nobody declares, so a misspelt key is a default silently in
    force; it refuses one of the wrong type at startup (InvalidParameterTypeException), so
    the node never comes up. A /** value set again in a node section is two values for
    what must be one.
    """
    problems = []
    shared = cfg.get(SHARED, {}).get("ros__parameters", {})
    nodes = [n for n, src in NODE_SOURCES.items() if src]
    for node in nodes:
        if f'"{node}"' not in (REPO / NODE_SOURCES[node]).read_text():
            problems.append(f"{NODE_SOURCES[node]} no longer names the node {node!r}")

    def type_problem(label, value, default):
        have, want = _ros_type(value), _ros_type(default)
        if have and want and have != want:
            return f"{label} is {have}, declared {want}"
        return None

    for key, value in shared.items():
        users = [n for n in nodes if key in _declared(n)]
        if not users:
            problems.append(f"{SHARED}.{key}: no node declares it")
        problems += [p for n in users
                     if (p := type_problem(f"{SHARED}.{key} (for {n})", value, _declared(n)[key]))]
    for node in nodes:
        declared = _declared(node)
        for key, value in cfg.get(node, {}).get("ros__parameters", {}).items():
            if key not in declared:
                problems.append(f"{node}.{key}: not declared")
            elif key in shared:
                problems.append(f"{node}.{key} sets the {SHARED} value again")
            elif p := type_problem(f"{node}.{key}", value, declared[key]):
                problems.append(p)
    _ensure("every key is a declared parameter of its type", "; ".join(problems),
            f"{len(nodes)} nodes")


# ------------------------------------------------------------------ the pump

# ArduPilot output functions that drive the boat: a pump path on one would let a
# "burst" run a thruster. ArduPilot refuses DO_SET_SERVO on them too; this check
# fires before anyone arms.
MOTOR_FUNCTIONS = {26, 70, 73, 74} | set(range(33, 41))


def check_pump():
    """The water pump's path through telemetry_bridge (docs/G7_pump_bench.md).

    * the pump's RC channel is none of: the sticks, the mode switch, the SB
      e-stop, or the autonomy-drop channel (every squirt would trip the latch);
    * squirt_cal and the bridge agree on it, and the bridge's e-stop matches the
      LED node's;
    * with a pump output configured, the saved baseline says that output passes
      the pump channel through (SERVOn_FUNCTION = 50 + ch) and rests at the OFF
      value (SERVOn_TRIM = pump_off_pwm): the autopilot ENDS every burst by
      returning the output to TRIM, so a wrong TRIM is a pump that never stops.
    """
    cfg = load_params_yaml()
    if cfg is None:
        return
    tb = _effective(cfg, "telemetry_bridge")
    sq = _effective(cfg, "squirt_cal")
    led = _effective(cfg, "led_status_node")
    try:
        ch, servo = int(tb["pump_rc_channel"]), int(tb["pump_servo_channel"])
        on, off = int(tb["pump_on_pwm"]), int(tb["pump_off_pwm"])
        drop, estop = int(tb["drop_channel"]), int(tb["estop_channel"])
    except KeyError as e:
        fail("pump path", f"cannot read the pump config: {e}")
        return
    problems = []
    taken = {1: "steering stick", 2: "stick", 3: "throttle stick", 4: "stick",
             8: "the mode switch", estop: "the SB e-stop", drop: "autonomy-drop"}
    if ch in taken:
        problems.append(f"pump_rc_channel {ch} is {taken[ch]}")
    if int(sq["pump_rc_channel"]) != ch:
        problems.append(f"squirt_cal.pump_rc_channel {sq['pump_rc_channel']} != "
                        f"telemetry_bridge's {ch}")
    if (estop, int(tb["estop_threshold"])) != (int(led["estop_channel"]),
                                                int(led["estop_threshold"])):
        problems.append("telemetry_bridge's estop_channel/threshold differ from "
                        "led_status_node's")
    if abs(on - off) < 200:
        problems.append(f"pump_on_pwm {on} and pump_off_pwm {off} are too close to "
                        "tell apart")
    if tb["pump_allow_disarmed"]:
        problems.append("pump_allow_disarmed is true (bench only, by a -p override)")
    if servo:
        spec = importlib.util.spec_from_file_location(
            "param_guard", Path(__file__).parent / "param_guard.py")
        pg = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pg)
        base = pg.load_param_file(PARAMS_BASELINE) if PARAMS_BASELINE.exists() else {}
        fn = base.get(f"SERVO{servo}_FUNCTION")
        trim = base.get(f"SERVO{servo}_TRIM")
        if fn is None or int(fn) in MOTOR_FUNCTIONS:
            problems.append(f"SERVO{servo}_FUNCTION is {fn} in the baseline: "
                            "missing, or a motor output")
        elif int(fn) != 50 + ch:
            problems.append(f"SERVO{servo}_FUNCTION is {int(fn)}, not RCIN{ch} "
                            f"({50 + ch})")
        if trim is None or int(trim) != off:
            problems.append(f"SERVO{servo}_TRIM is {trim}, not pump_off_pwm {off}: "
                            "a burst would END at TRIM")
    # GUIDED heading+speed: thrust 1.0 means WP_SPEED to the autopilot, so the
    # bridge must agree or every commanded speed is off by the ratio.
    try:
        hs_wp = float(tb["hs_wp_speed_mps"])
    except KeyError as e:
        problems.append(f"telemetry_bridge missing {e}")
    else:
        spec = importlib.util.spec_from_file_location(
            "param_guard", Path(__file__).parent / "param_guard.py")
        pg = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(pg)
        base = pg.load_param_file(PARAMS_BASELINE) if PARAMS_BASELINE.exists() else {}
        wp = base.get("WP_SPEED")
        if wp is None:
            problems.append("WP_SPEED missing from the baseline")
        elif abs(wp - hs_wp) > 1e-6:
            problems.append(f"telemetry_bridge.hs_wp_speed_mps {hs_wp} != baseline "
                            f"WP_SPEED {wp}: every heading+speed command would be "
                            f"scaled by {wp / hs_wp:.2f}")
    # RC overrides: software may take the sticks, NEVER the pilot's e-stop, mode
    # switch, pump or drop switch - those are how the pilot takes the boat back.
    try:
        ov = [int(c) for c in tb["override_channels"]]
    except KeyError as e:
        problems.append(f"telemetry_bridge missing {e}")
    else:
        mode_ch = 8
        try:
            base = pg.load_param_file(PARAMS_BASELINE) if PARAMS_BASELINE.exists() else {}
            mode_ch = int(base.get("MODE_CH", 8))
        except NameError:
            pass
        guarded = {estop: "the SB e-stop", mode_ch: "the mode switch (MODE_CH)",
                   ch: "the pump", drop: "autonomy-drop"}
        bad = [f"ch{c} ({guarded[c]})" for c in ov if c in guarded]
        if bad:
            problems.append("override_channels includes " + ", ".join(bad))
        if [str(m).upper() for m in tb["override_modes"]] != ["MANUAL"]:
            problems.append(f"override_modes {tb['override_modes']} is not [MANUAL]: "
                            "overrides in an autopilot mode fight the autopilot")
    if problems:
        fail("pump path", "; ".join(problems))
    else:
        ok("pump path", f"pump on ch{ch}, " + (f"SERVO{servo} passes it through, "
                                               f"TRIM = OFF {off}" if servo
                                               else "no pump output yet (G7)"))


# --------------------------------------------------------- nav2 avoidance

def _load_module(name, path):
    """Import one source file by path, for a module with no ROS in it."""
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _module_literals(path):
    """{name: value} for every top-level `NAME = <literal>` in a source file, without
    importing it (the nodes import rclpy): a node's PARAMS defaults and fixed topics."""
    out = {}
    for stmt in ast.parse(Path(path).read_text()).body:
        if isinstance(stmt, ast.Assign) and len(stmt.targets) == 1 \
                and isinstance(stmt.targets[0], ast.Name):
            try:
                out[stmt.targets[0].id] = ast.literal_eval(stmt.value)
            except ValueError:
                pass
    return out


def _effective(cfg, node):
    """The values a node runs with: its declared defaults, then /**, then its own section."""
    declared = _declared(node) if NODE_SOURCES.get(node) else {}
    shared = {k: v for k, v in cfg.get(SHARED, {}).get("ros__parameters", {}).items()
              if k in declared}
    out = dict(declared)
    out.update(shared)
    out.update(cfg.get(node, {}).get("ros__parameters", {}))
    return out


@functools.lru_cache(maxsize=None)
def _declared(node):
    """{name: default} for every parameter a node declares, read from its source."""
    path = REPO / NODE_SOURCES[node]
    return _cpp_declared(path) if path.suffix == ".cpp" else _py_declared(path)


def _py_declared(path):
    """A Python node's PARAMS without importing the node (it imports rclpy). A
    `**module.PARAMS` entry is read from that sibling module the same way; any other
    non-literal (ocs_link's constants, oak_controls.defaults()) is evaluated against the
    pure-Python module the node imports it from."""
    tree = ast.parse(path.read_text())
    imports = {a.asname or a.name: (s.module, a.name) for s in tree.body
               if isinstance(s, ast.ImportFrom) and s.module for a in s.names}
    params = next(s.value for s in tree.body if isinstance(s, ast.Assign)
                  and getattr(s.targets[0], "id", None) == "PARAMS")
    out = {}
    for key, value in zip(params.keys, params.values):
        if key is None and isinstance(value, ast.Attribute) and value.attr == "PARAMS":
            out.update(_py_declared(path.with_name(value.value.id + ".py")))
            continue
        try:
            v = ast.literal_eval(value)
        except ValueError:
            names = {n.id for n in ast.walk(value) if isinstance(n, ast.Name)} & set(imports)
            scope = {n: _import(*imports[n]) for n in names}
            v = eval(compile(ast.Expression(value), str(path), "eval"), scope)
        if key is None:
            out.update(v)
        else:
            out[ast.literal_eval(key)] = v
    return out


def _import(module, name):
    """`from module import name`, off the source tree (each package dir on sys.path)."""
    for pkg in REPO.glob("crusader_*/"):
        if str(pkg) not in sys.path:
            sys.path.insert(0, str(pkg))
    try:
        return importlib.import_module(f"{module}.{name}")
    except ImportError:
        return getattr(importlib.import_module(module), name)


CPP_TYPES = {"double": float, "int64_t": int, "int": int, "bool": bool, "std::string": str}


def _cpp_declared(path):
    """bt_runner_node's parameters: every declare_parameter<T>("name", default), nav_mode
    (dynamically typed, so None), and the nav_* planner knobs at NavParams' defaults."""
    def strip(text):
        return re.sub(r"//[^\n]*", "", text)

    src = strip(path.read_text())
    out = {}
    for t, name, default in re.findall(
            r'declare_parameter<([\w:<> ]+)>\(\s*"(\w+)",\s*(.+?)(?:,\s*\w+)?\);', src, re.S):
        element = CPP_TYPES[re.sub(r"^std::vector<(.+)>$", r"\1", t.strip())]
        text = re.sub(r"^std::vector<.+?>+(?=\{)", "", default.strip())
        text = re.sub(r"\btrue\b", "True", re.sub(r"\bfalse\b", "False", text))
        v = ast.literal_eval(text.replace("{", "[").replace("}", "]"))
        out[name] = [element(x) for x in v] if isinstance(v, list) else element(v)
    out.update((name, None) for name in re.findall(r'declare_parameter\(\s*"(\w+)"', src))
    include = path.parents[1] / "include" / "crusader_bt"
    struct = re.search(r"struct NavParams\s*\{(.*?)\n\};",
                       strip((include / "path_math.hpp").read_text()), re.S).group(1)
    members = {}
    for kind, decl in re.findall(r"\b(double|int)\s+([^;(]+);", struct):
        for part in decl.split(","):
            member, value = (s.strip() for s in part.split("="))
            members[member] = CPP_TYPES[kind](float(value))
    for name, member in re.findall(r'(?:real|whole)Knob\("(\w+)",\s*&NavParams::(\w+)',
                                   (include / "nav_params.hpp").read_text()):
        out[name] = members[member]
    return out


def _ros_type(v):
    """The ROS parameter type a value declares or sets; None where it cannot be told."""
    if isinstance(v, bool):
        return "bool"
    if isinstance(v, (int, float, str)):
        return {int: "integer", float: "double", str: "string"}[type(v)]
    if isinstance(v, (list, tuple)) and v:
        kinds = {_ros_type(x) for x in v}
        return f"{kinds.pop()}[]" if len(kinds) == 1 else "mixed[]"
    return None


def _baseline_params():
    """The saved autopilot baseline as {name: value}, or {} when it is absent
    (check_param_baseline reports that on its own)."""
    if not PARAMS_BASELINE.exists():
        return {}
    spec = importlib.util.spec_from_file_location(
        "param_guard", Path(__file__).parent / "param_guard.py")
    pg = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(pg)
    return pg.load_param_file(PARAMS_BASELINE)


def _ensure(check, problem, detail=""):
    """Report one check: `problem` is the failure text, or empty when it holds."""
    if problem:
        fail(check, problem)
    else:
        ok(check, detail)


def _same(check, left, right, why):
    """One equality cross-check. `left` and `right` are (label, value) pairs."""
    (ln, lv), (rn, rv) = left, right
    _ensure(check, lv != rv and f"{ln} = {lv!r} but {rn} = {rv!r} — {why}",
            f"{ln} = {rn} = {lv!r}")


def check_nav():
    """Nav2 avoidance cross-checks (docs/nav2_avoidance_spec.md sections 3.5, 5.8, 6, 10).

    Reads crusader_params.yaml, crusader_nav/config/nav2_params.yaml and the
    autopilot baseline. A key one of them lacks is a FAILURE, like every other
    cross-check here: a producer and a consumer that cannot be compared are not
    known to agree.
    """
    import yaml
    cfg = load_params_yaml()
    if cfg is None:
        return
    try:
        nav2_text = NAV2_PARAMS.read_text(encoding="utf-8")
        nav2 = yaml.safe_load(nav2_text)
    except (OSError, yaml.YAMLError) as e:
        fail("nav2 params readable", f"{NAV2_PARAMS}: {e}")
        return

    # rcl parses this file too (planner_server), so: no anchors. And no
    # use_sim_time: the sim runs on wall clock, and a stray true stops every plan
    # waiting for a /clock nothing publishes.
    anchored = [n for n, line in enumerate(nav2_text.splitlines(), 1)
                if "&" in line.split("#", 1)[0] or "*" in line.split("#", 1)[0]]
    uses_sim_time = any("use_sim_time" in line.split("#", 1)[0]
                        for line in nav2_text.splitlines())
    _ensure("nav2 params: no anchors/aliases, no use_sim_time",
            (anchored and f"anchors or aliases on lines {anchored}")
            or (uses_sim_time and "use_sim_time is set: the sim runs on wall clock"))

    try:
        cm = nav2["global_costmap"]["global_costmap"]["ros__parameters"]
        planner = nav2["planner_server"]["ros__parameters"]
        lifecycle = nav2["nav_lifecycle"]["ros__parameters"]
        hazard_topic = cm["hazard_layer"]["topic"]
        bt = _effective(cfg, "bt_runner_node")
        frames = _effective(cfg, "nav_frames_node")
        datum_topic = _module_literals(REPO / NODE_SOURCES["nav_frames_node"])["DATUM_TOPIC"]
        lidar = _effective(cfg, "lidar_node")
        lidar["nav_cloud_topic"] = _module_literals(REPO / NODE_SOURCES["lidar_node"])["NAV_CLOUD_TOPIC"]
        inscribed = cm["robot_radius"] * math.cos(math.pi / 16)
        half_window = min(cm["width"], cm["height"]) / 2.0
        hard, soft, clip = bt["nav_hard_m"], bt["nav_soft_m"], bt["nav_clip_radius_m"]
    except (KeyError, TypeError) as e:
        fail("nav2 cross-checks", f"missing key {e}")
        return

    # Clearance. Nav2 builds a radius footprint as a 16-gon, so the circle it
    # really enforces has radius r*cos(pi/16), not r.
    _ensure("nav_hard_m within the inscribed radius",
            hard > inscribed + 1e-9 and
            f"bt_runner_node.nav_hard_m {hard} > robot_radius {cm['robot_radius']} * "
            f"cos(pi/16) = {inscribed:.4f}: the BT promises a clearance the costmap "
            "does not enforce", f"{hard} <= {inscribed:.4f}")
    _same("nav_soft_m is the inflation radius",
          ("bt_runner_node.nav_soft_m", soft),
          ("inflation_layer.inflation_radius", cm["inflation_layer"]["inflation_radius"]),
          "the BT soft band and the planner soft band disagree")
    # Smac is not bounds-checked for a goal off the window, so the BT clips
    # goals inside it, with 5 m kept for the inflation band and the carrot.
    _ensure("nav_clip_radius_m inside the window",
            clip > half_window - 5.0 and
            f"{clip} > half the costmap ({half_window}) - 5: a goal can land off the "
            "window, where SmacPlanner2D behaviour is undefined",
            f"{clip} <= {half_window} - 5")

    # Topics and frames: each pair silently does nothing when it disagrees.
    _same("hazard topic bt_runner -> HazardLayer",
          ("bt_runner_node.nav_hazards_topic", bt["nav_hazards_topic"]),
          ("hazard_layer.topic", hazard_topic),
          "the layer subscribes to a topic nobody publishes, the costmap goes "
          "non-current and every plan hangs")
    for src in ("lidar_mark", "lidar_clear"):
        _same(f"obstacle cloud lidar_node -> stvl {src}",
              ("lidar_node.NAV_CLOUD_TOPIC", lidar["nav_cloud_topic"]),
              (f"{src}.topic", cm["stvl_layer"][src]["topic"]),
              "STVL listens to a topic nobody publishes")
    _same("nav_cloud_frame is the TF child frame",
          ("lidar_node.nav_cloud_frame", lidar["nav_cloud_frame"]),
          ("nav_frames_node.base_frame", frames["base_frame"]),
          "the cloud is stamped in a frame TF does not publish, so STVL drops it")
    _same("costmap robot_base_frame is the TF child frame",
          ("global_costmap.robot_base_frame", cm["robot_base_frame"]),
          ("nav_frames_node.base_frame", frames["base_frame"]),
          "planner_server never gets a robot pose and stays in configuring")
    _same("costmap global_frame is the TF parent frame",
          ("global_costmap.global_frame", cm["global_frame"]),
          ("nav_frames_node.map_frame", frames["map_frame"]),
          "the plan and the hazards are in different frames")
    _same("hazard frame is the costmap frame",
          ("bt_runner_node.nav_map_frame", bt["nav_map_frame"]),
          ("global_costmap.global_frame", cm["global_frame"]),
          "HazardLayer refuses a HazardArray in any other frame")
    _same("STVL voxel size is the costmap resolution",
          ("stvl_layer.voxel_size", cm["stvl_layer"]["voxel_size"]),
          ("global_costmap.resolution", cm["resolution"]), "a voxel is a cell")

    # bt_runner calls Nav2's server names (from nav.launch.py's node names); a
    # different one is a planner that "is not available" forever.
    nav2_names = {"nav_planner_action": "/compute_path_to_pose",
                  "nav_valid_service": "/is_path_valid",
                  "nav_clear_service": "/global_costmap/clear_entirely_global_costmap"}
    wrong = {k: (bt.get(k), v) for k, v in nav2_names.items() if bt.get(k) != v}
    _ensure("bt_runner_node planner endpoints", wrong and f"(have, want) {wrong}",
            "match planner_server")
    _ensure("nav_planner_id is a configured plugin",
            bt["nav_planner_id"] not in planner["planner_plugins"] and
            f"{bt['nav_planner_id']!r} not in planner_plugins {planner['planner_plugins']}",
            bt["nav_planner_id"])
    _ensure("nav_lifecycle drives planner_server",
            "planner_server" not in lifecycle["node_names"] and
            f"node_names {lifecycle['node_names']}: it would never leave unconfigured")
    # The node ignores a key it does not declare and dies on a wrong type (rcl takes
    # no int for a double), so ask the node's own validator, which is pure.
    core = _load_module("lifecycle_core", REPO / "crusader_nav" / "crusader_nav" /
                        "lifecycle_core.py")
    _ensure("nav_lifecycle parameters", core.params_error(lifecycle) or "",
            f"{sorted(lifecycle)}")

    # A dead bt_runner must make the costmap non-current: the layer max age has
    # to cover at least two publishes, or a healthy one flaps it.
    period = 1.0 / bt["nav_hazard_rate_hz"]
    max_age = cm["hazard_layer"]["max_age_s"]
    _ensure("hazard max_age_s covers two publishes",
            max_age < 2.0 * period and
            f"max_age_s {max_age} < 2 x {period:.2f} s (1 / nav_hazard_rate_hz)",
            f"{max_age} s vs {period:.2f} s period")

    # The autopilot idea of "reached": the carrot logic is built around it.
    wp_radius = _baseline_params().get("WP_RADIUS")
    if wp_radius is None:
        fail("nav_wp_radius_m", "WP_RADIUS missing from the baseline")
    else:
        _same("nav_wp_radius_m is the autopilot WP_RADIUS",
              ("bt_runner_node.nav_wp_radius_m", float(bt["nav_wp_radius_m"])),
              ("baseline WP_RADIUS", float(wp_radius)),
              "the carrot arrival test disagrees with the autopilot")

    # A datum-topic mismatch leaves bt_runner waiting for a datum nobody sends, holding
    # every planned leg.
    _same("datum topic nav_frames_node -> bt_runner",
          ("nav_frames_node.DATUM_TOPIC", datum_topic),
          ("bt_runner_node.nav_datum_topic", bt["nav_datum_topic"]),
          "bt_runner never adopts a datum")

    # The datum: a 'param' datum of (0, 0) is the default, not a datum.
    source, dlat, dlon = frames["datum_source"], frames["datum_lat"], frames["datum_lon"]
    _ensure("nav_frames_node datum",
            (source not in ("param", "first_fix") and
             f"datum_source {source!r} is not param or first_fix")
            or (source == "param" and (dlat, dlon) == (0.0, 0.0) and
                "datum_source is 'param' but the datum is (0, 0)"),
            f"source {source}")


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

    # The frame/heading config the boat cannot drive without: FRAME_TYPE=2 is the
    # OmniX mixer; COMPASS_USE=0 with EK3_SRC1_YAW=2 is dual-antenna GPS yaw.
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

    # REACHABILITY, not direct declaration: packages-up-to builds the transitive
    # closure, so crusader_common and crusader_msgs need not be listed in bringup.
    # A direct-listing check would cry wolf forever, and people stop reading it.
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

    orphans = sorted(set(names) - reachable - SIM_ONLY_PACKAGES)
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
    backslash in a COMMENT (a shell command wrapped across two lines) makes it
    fail to decode, and the build dies with

        UnicodeDecodeError processing template 'struct.idl.em'

    which names neither the file nor the line.
    """
    # Behaviour trees get the XML rules, including the one that keeps biting: a
    # double hyphen is ILLEGAL inside an <!-- comment -->, and the error names a
    # line, not the cause. behavior_trees/lib/ holds the included task subtrees.
    trees = sorted(REPO.glob("*/behavior_trees/*.xml")) + sorted(REPO.glob("*/behavior_trees/lib/*.xml"))
    for tree in trees:
        base = next(p for p in tree.parents if p.name == "behavior_trees")
        label = f"{base.parent.name}/{tree.relative_to(base).as_posix()}"
        try:
            ET.parse(tree)
        except ET.ParseError as e:
            fail(f"{label} parses",
                 f"{e} — if this is inside a comment, a DOUBLE HYPHEN is "
                 "illegal there; XML forbids it")
        else:
            ok(f"{label} parses")

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
    check_pump()
    check_nav()
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