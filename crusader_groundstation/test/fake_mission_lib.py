"""fake_mission_lib — a stand-in for crusader_groundstation.mission_lib, same five functions.

Used by fake_gcs.py and the tests so the Mission tab and mission_api can be exercised without
the real library and without a boat. It is NOT the contract's authority: mission_lib is. It
validates just enough (well-formed XML, a main tree that exists, SubTree references that
resolve, literal waypoints with a lat and lon) to give the page real errors to render.
"""
import os
import re
import xml.etree.ElementTree as ET

CATEGORIES = [
    {"id": "flow", "label": "Flow", "color": "#FFAB19"},
    {"id": "decorators", "label": "Decorators", "color": "#9966FF"},
    {"id": "robocommand", "label": "RoboCommand", "color": "#FF6680"},
    {"id": "navigation", "label": "Navigation", "color": "#4C97FF"},
    {"id": "tasks", "label": "Tasks", "color": "#0FBD8C"},
    {"id": "conditions", "label": "Conditions", "color": "#59C0D9"},
    {"id": "other", "label": "Other", "color": "#8A8F99"},
]


def port(name, type_="string", default=None, desc="", direction="in", required=False, choices=None):
    return {"name": name, "type": type_, "default": default, "desc": desc, "choices": choices or [],
            "direction": direction, "required": required}


def node(id_, kind, cat, label, desc, children="none", ports=None, advanced=False):
    return {"id": id_, "kind": kind, "category": cat, "label": label, "desc": desc,
            "children": children, "advanced": advanced, "ports": ports or []}


NAV = [port("target", "enum", "fix", "where to go", choices=["fix", "home", "approach"]),
       port("lat", "double", None, "latitude, degrees"), port("lon", "double", None, "longitude, degrees"),
       port("tolerance", "double", 4.0, "arrival radius, metres"),
       port("avoid", "bool", True, "plan around known objects")]
CIRCLE = [port("anchor", "enum", "fix", "what to circle", choices=["fix", "entry", "exit"]),
          port("lat", "double"), port("lon", "double"), port("radius", "double", 6.0, "metres"),
          port("points", "int", 8, "waypoints per circle"),
          port("direction", "enum", "cw", choices=["cw", "ccw"])]

NODES = [
    node("Sequence", "control", "flow", "Sequence", "Runs children in order; stops at the first failure.", "many"),
    node("ReactiveSequence", "control", "flow", "Reactive sequence",
         "Re-checks every child each tick; a failing guard halts the rest.", "many"),
    node("Fallback", "control", "flow", "Fallback", "Tries children until one succeeds.", "many"),
    node("ForceSuccess", "decorator", "decorators", "Force success", "Succeeds whatever the child does.", "one"),
    node("Inverter", "decorator", "decorators", "Inverter", "Swaps success and failure.", "one"),
    node("Timeout", "decorator", "decorators", "Timeout", "Fails the child after a time.", "one",
         [port("msec", "int", 10000, "milliseconds", required=True)]),
    node("RetryUntilSuccessful", "decorator", "decorators", "Retry", "Re-runs a failing child.", "one",
         [port("num_attempts", "int", 3)]),
    node("WaitForRunStart", "action", "robocommand", "Wait for RunStart",
         "Holds position until RoboCommand's RunStart arrives.", "none",
         [port("max_age_s", "double", 900), port("timeout_s", "double", 0), port("hold", "bool", True),
          port("run_id", "bb", None, "out: the run id", direction="out")]),
    node("TaskAttempt", "decorator", "robocommand", "Task attempt",
         "Reports the task to RoboCommand while the child runs.", "one",
         [port("token", "enum", None, "RxTask", required=True,
               choices=["TASK_SAFE_PASSAGE", "TASK_COORDINATED_LOGISTICS"])]),
    node("NavigateTo", "action", "navigation", "Navigate to", "Drives to a target.", "none", NAV),
    node("CircleBuoy", "action", "navigation", "Circle", "Circles a point.", "none", CIRCLE),
    node("HoldStation", "action", "navigation", "Hold station", "Holds position.", "none"),
    node("IsAutonomous", "condition", "conditions", "Is autonomous", "Succeeds while the pilot allows autonomy."),
    node("SetTask", "action", "other", "Set task", "Sets the heartbeat task.", "none",
         [port("token", "string")], advanced=True),
    node("SubTree", "subtree", "tasks", "Subtree", "Calls another tree.", "none"),
]
PRESETS = [
    {"id": "GpsWaypoint", "preset_of": "NavigateTo", "fixed": {"target": "fix"}, "label": "GPS waypoint",
     "category": "navigation", "desc": "Drive to a latitude and longitude.",
     "show_ports": ["lat", "lon", "tolerance", "avoid"]},
    {"id": "ReturnHome", "preset_of": "NavigateTo", "fixed": {"target": "home"}, "label": "Return home",
     "category": "navigation", "desc": "Drive back to the launch point.", "show_ports": ["tolerance"]},
    {"id": "OrbitGpsPoint", "preset_of": "CircleBuoy", "fixed": {"anchor": "fix"},
     "label": "Orbit a GPS point", "category": "navigation", "desc": "Circle a latitude and longitude.",
     "show_ports": ["lat", "lon", "radius", "points", "direction"]},
    {"id": "HoldPosition", "preset_of": "HoldStation", "fixed": {}, "label": "Hold position",
     "category": "navigation", "desc": "Stay where we are.", "show_ports": []},
]
TASK1 = ('<BehaviorTree ID="Task1Global"><Sequence name="task1"><HoldStation/>'
         '<CircleBuoy anchor="entry" radius="{orbit_radius}" points="{orbit_points}"/></Sequence></BehaviorTree>')
SUBTREES = [{"id": "Task1Global", "label": "Task 1: Safe Passage", "desc": "Find the gate and pass through it.",
             "file": "lib/task1_global.xml",
             "ports": [port("orbit_points", "int", 8, "waypoints per orbit"),
                       port("orbit_radius", "double", 6.0, "orbit radius, metres")],
             "xml": TASK1}]


def catalogue(bt_dir):
    return {"version": 1, "categories": CATEGORIES, "nodes": NODES, "presets": PRESETS, "subtrees": SUBTREES}


def _description(path):
    try:
        with open(path, encoding="utf-8") as f:
            m = re.search(r"<!--\s*MISSION:\s*(.*?)\s*-->", f.read(), re.S)
    except OSError:
        return ""
    return m.group(1) if m else ""


def list_missions(bt_dir, missions_dir):
    out = []
    for source, d in (("prebuilt", bt_dir), ("saved", missions_dir)):
        if not os.path.isdir(d):
            continue
        for f in sorted(os.listdir(d)):
            if f.endswith(".xml"):
                path = os.path.join(d, f)
                out.append({"name": f[:-4], "source": source, "file": path, "main_tree": "",
                            "description": _description(path)})
    return out


def load_mission(bt_dir, missions_dir, name, source):
    d = bt_dir if source == "prebuilt" else missions_dir
    path = os.path.join(d, os.path.basename(name) + ".xml")
    if not os.path.isfile(path):
        return {"ok": False, "name": name, "source": source, "xml": "", "description": "",
                "message": f"no {source} mission called {name}"}
    with open(path, encoding="utf-8") as f:
        xml = f.read()
    return {"ok": True, "name": name, "source": source, "xml": xml, "description": _description(path),
            "message": ""}


def validate(xml_text, cat):
    errors, warnings = [], []
    try:
        root = ET.fromstring(xml_text)
    except ET.ParseError as e:
        return {"ok": False, "errors": [f"not well-formed XML: {e}"], "warnings": [], "main_tree": "", "trees": []}
    trees = {t.get("ID"): t for t in root.findall("BehaviorTree")}
    main = root.get("main_tree_to_execute") or ""
    if main not in trees:
        errors.append(f"main_tree_to_execute '{main}' is not a BehaviorTree in this file")
    known = {n["id"] for n in cat["nodes"]}

    def walk(el, path):
        # The real library's error shape: "Main > guard > mission > NavigateTo: what is wrong".
        for kid in el:
            here = path + [kid.get("name") or (f"SubTree {kid.get('ID')}" if kid.tag == "SubTree" else kid.tag)]
            where = " > ".join(here)
            if kid.tag == "SubTree":
                if kid.get("ID") not in trees:
                    errors.append(f"{where}: SubTree '{kid.get('ID')}' is not defined in this mission")
            elif kid.tag not in known:
                warnings.append(f"{where}: {kid.tag} is not in the catalogue")
            if kid.tag == "NavigateTo" and kid.get("target") == "fix":
                for port in ("lat", "lon"):
                    if kid.get(port) in (None, ""):
                        errors.append(f"{where}: a GPS point needs {port}")
            walk(kid, here)
    for tid, tree in trees.items():
        walk(tree, [tid])
    return {"ok": not errors, "errors": errors, "warnings": warnings, "main_tree": main, "trees": list(trees)}


def save_mission(missions_dir, name, xml_text, cat, overwrite=False):
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", name or ""):
        return {"ok": False, "message": f"bad mission name {name!r}", "file": ""}
    path = os.path.join(missions_dir, name + ".xml")
    if os.path.exists(path) and not overwrite:
        return {"ok": False, "message": f"{name} already exists", "file": path}
    os.makedirs(missions_dir, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(xml_text)
    return {"ok": True, "message": f"saved {name}", "file": path}
