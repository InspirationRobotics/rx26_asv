"""lake_rig_plan — what lake_rig_up.sh decides BEFORE it starts anything: which tree, which nav_mode and why,
POOL, and the planner-tuning overlay (LAKE_TUNING).

Runs in: the `asv` container on the Jetson (python3 + PyYAML; ROS only for `verify`), called by lake_rig_up.sh,
and by test/test_lake_rig.py, which is why the decisions live here and not in bash: they can be tested offline.

    python3 -m crusader_sim.lake_rig_plan decide
        reads TREE, NAV_MODE, POOL, LAKE_TUNING from the environment. stdout: shell assignments for `eval`
        (PLAN_TREE, PLAN_NAV_MODE, PLAN_POOL, PLAN_TUNING, PLAN_BANNER); a bad input: `*** why` on stderr, exit 2.
    python3 -m crusader_sim.lake_rig_plan finish --nav-mode MODE --cfg F --nav2-cfg F --out-dir D [--dry] [--publish true|false]
        after the package preflight (which may have turned nav off): checks the overlay file against the boat's own
        params files, writes D/tuning_applied.yaml (what bt_runner and Nav2 really get) and D/rig.json (what the
        page shows), prints the keys. --dry (lake_rig_up.sh --check) writes nothing. A bad file: exit 2.
    python3 -m crusader_sim.lake_rig_plan verify D/tuning_applied.yaml
        reads /bt_runner_node's parameters back from the running node (ros2 param dump) and compares them.

THE RULES (each is a test in test/test_lake_rig.py):
  TREE        default task1_global.xml, the whole-field planner. It drives GUIDED setpoints itself and never
              calls Nav2 (crusader_bt/src/global_leaves.cpp:19-20), so it behaves the same in every nav_mode.
  NAV_MODE    given: as given. Not given: `off` for the global tree (the rig then needs no Nav2 in the container),
              `shadow` for the per-gate trees (task1_disruptive.xml, ...), as before. POOL=1 forces `off`.
  POOL=1      a small outdoor pool, camera only: NAV_MODE off (no Nav2, no STVL: the LiDAR feeds nothing that
              plans), target_tracker started with use_lidar:=false (the shell does that), and LAKE_TUNING
              defaults to the tight_3to5m profile.
  LAKE_TUNING a params file in the sim's tuning-profile format (nested ROS params, only the keys that change):
              bt_runner_node always, planner_server / global_costmap / nav_lifecycle only when the nav stack is
              started. A key that is not in the boat's own params file (a typo, which ROS would silently ignore)
              or has another type (which would stop the node at start) is an error here, not at the lake.

WHERE A POOL PROFILE WOULD GO: crusader_sim/config/tuning_profiles/pool_<name>.yaml, in tight_3to5m.yaml's format,
then POOL=1 LAKE_TUNING=<that file>. It is NOT written yet: the pool's dimensions are not known, and a profile
made of guessed numbers would look like a measurement. POOL_PROFILE below is the stand-in until it exists.
"""
import argparse
import json
import math
import os
import shlex
import subprocess
import sys

import yaml

GLOBAL_TREE = "task1_global.xml"
PER_GATE_TREE = "task1_disruptive.xml"
NAV_MODES = ("off", "shadow", "on")
POOL_PROFILE = "tight_3to5m"             # crusader_sim/config/tuning_profiles/<name>.yaml; see "WHERE A POOL PROFILE WOULD GO"
BT_NODE = "bt_runner_node"                # always gets the overlay (a 2nd --params-file)
NAV_NODES = ("planner_server", "global_costmap/global_costmap", "nav_lifecycle")    # only with the nav stack (nav2_overlay:=)
APPLIED_FILE = "tuning_applied.yaml"
RIG_FILE = "rig.json"
DUMP_TOL = 1e-6

_HERE = os.path.dirname(os.path.abspath(__file__))
PROFILES_DIR = os.path.join(os.path.dirname(_HERE), "config", "tuning_profiles")


class PlanError(Exception):
    """An input the rig must refuse before it starts anything. str(e) is the operator's message."""


def truthy(v):
    """The shell script's own PUBLISH rule: 1 / true / TRUE / True."""
    return str(v or "").strip() in ("1", "true", "TRUE", "True")


def is_global_tree(tree):
    return os.path.basename(str(tree)) == GLOBAL_TREE


def profile_path(name=POOL_PROFILE):
    return os.path.join(PROFILES_DIR, name + ".yaml")


# ------------------------------------------------------------------ the decision

def decide(env):
    """The tree, the nav_mode and the overlay file this rig run uses, with the reason for each.

    -> dict: tree (name or path as given), global_tree, nav_mode, nav_given (what NAV_MODE said, or None),
    nav_why, pool, tuning (path or ""), tuning_why, lines (the plan banner). PlanError for a bad input."""
    tree = (env.get("TREE") or "").strip() or GLOBAL_TREE
    given = (env.get("NAV_MODE") or "").strip() or None
    if given is not None and given not in NAV_MODES:
        raise PlanError("NAV_MODE must be off, shadow or on (got '%s')" % given)
    pool = truthy(env.get("POOL"))
    glob = is_global_tree(tree)
    notes = []
    if pool:
        nav_mode, nav_why = "off", "POOL=1: camera only, so no Nav2 and no STVL (the LiDAR feeds nothing that plans)"
        if given not in (None, "off"):
            notes.append("NAV_MODE=%s was given and is IGNORED: POOL=1 forces off" % given)
    elif given is not None:
        nav_mode, nav_why = given, "NAV_MODE=%s was given" % given
        if glob and given != "off":
            notes.append("%s drives its own plan with GUIDED setpoints and never uses Nav2: NAV_MODE=%s starts the "
                         "nav stack and DRAWS its planner on the page, but that planner does not drive the boat"
                         % (GLOBAL_TREE, given))
    elif glob:
        nav_mode = "off"
        nav_why = ("%s drives GUIDED setpoints itself and never calls Nav2 (crusader_bt/src/global_leaves.cpp:19), "
                   "so no Nav2 is needed in this container" % GLOBAL_TREE)
    else:
        nav_mode = "shadow"
        nav_why = "the default for a per-gate tree: the planner runs and is drawn, the tree drives the legacy legs"
    if nav_mode == "off" and not glob:
        notes.append("nav_mode off with %s: the legacy straight legs, no avoidance planner" % os.path.basename(tree))
    tuning = (env.get("LAKE_TUNING") or "").strip()
    if tuning:
        tuning, tuning_why = os.path.abspath(tuning), "LAKE_TUNING"
    elif pool:
        tuning = profile_path()
        tuning_why = ("POOL default: the small-field profile %s (replace it with a pool profile once the pool is "
                      "measured: LAKE_TUNING=<file>)" % POOL_PROFILE)
    else:
        tuning_why = ""
    if tuning and not os.path.isfile(tuning):
        raise PlanError("%s=%s: no such file%s" % ("LAKE_TUNING" if tuning_why == "LAKE_TUNING" else "POOL's default profile",
                                                  tuning, "" if tuning_why == "LAKE_TUNING" else " (restore it, or give LAKE_TUNING=<file>)"))
    lines = ["tree        %s%s" % (tree, "  (the whole-field planner)" if glob else "  (per-gate)"),
             "nav_mode    %s  <- %s" % (nav_mode, nav_why)]
    lines += ["            %s" % n for n in notes]
    if pool:
        lines.append("POOL: camera-only, LiDAR not used for planning  (nav_mode off, target_tracker use_lidar:=false)")
    lines.append("tuning      %s" % ("%s  <- %s" % (tuning, tuning_why) if tuning else "none: the boat's own params, unchanged"))
    return {"tree": tree, "global_tree": glob, "nav_mode": nav_mode, "nav_given": given, "nav_why": nav_why,
            "pool": pool, "tuning": tuning, "tuning_why": tuning_why, "notes": notes, "lines": lines}


# ------------------------------------------------------------------ the overlay

def _load_yaml(path, what):
    try:
        with open(path, encoding="utf-8") as f:
            doc = yaml.safe_load(f)
    except OSError as e:
        raise PlanError("%s: cannot read %s (%s)" % (what, path, e.strerror or e))
    except yaml.YAMLError as e:
        raise PlanError("%s: %s is not valid YAML: %s" % (what, path, (str(e).splitlines() or ["?"])[0]))
    if doc is None:
        return {}
    if not isinstance(doc, dict):
        raise PlanError("%s: %s is not a YAML mapping of node -> ros__parameters" % (what, path))
    return doc


def _flat(d, prefix=()):
    """{dotted key: leaf}: nested maps become dotted keys; a scalar, a list or an empty map is a leaf."""
    out = {}
    for k, v in d.items():
        if isinstance(v, dict) and v:
            out.update(_flat(v, prefix + (str(k),)))
        else:
            out[".".join(prefix + (str(k),))] = v
    return out


def sections(doc):
    """{node path: {dotted key: value}}: every ros__parameters block in a params document, however it is
    nested (bt_runner_node.ros__parameters, global_costmap.global_costmap.ros__parameters). A top-level entry
    with no ros__parameters under it is a PlanError: nothing would read it."""
    def walk(d, path, out):
        for k, v in d.items():
            if k == "ros__parameters":
                if not isinstance(v, dict):
                    raise PlanError("%s: ros__parameters is not a mapping" % "/".join(path))
                out.setdefault("/".join(path), {}).update(_flat(v))
            elif isinstance(v, dict):
                walk(v, path + [str(k)], out)
    secs = {}
    for top, v in doc.items():
        if not isinstance(v, dict):
            raise PlanError("top-level '%s' is not a node section (nest it under <node>: ros__parameters:)" % top)
        found = {}
        walk(v, [str(top)], found)
        if not found:
            raise PlanError("top-level '%s' has no ros__parameters under it: nothing would read it" % top)
        for node, keys in found.items():
            secs.setdefault(node, {}).update(keys)
    return secs


def _base_value(base_doc, node, key):
    """(found, value) of `key` (dotted) in the boat's own params document, under `node`'s ros__parameters."""
    d = base_doc
    for k in node.split("/") + ["ros__parameters"] + key.split("."):
        if not isinstance(d, dict) or k not in d:
            return False, None
        d = d[k]
    return True, d


def _same_type(a, b):
    """ROS takes a YAML 3 for an integer parameter and 3.0 for a double, and refuses the other way round
    (the node would stop at start): the overlay's type must be the base file's."""
    return type(a) is type(b)


def overlay_plan(doc, nav_started, bt_base=None, nav_base=None):
    """Split an overlay document into what is applied and what is ignored, and collect every problem.

    -> dict: applied {node: {key: value}}, ignored {node: {key: value}} (a Nav2 node when nav is not started),
    errors [str], unchecked [str] (a base file that could not be read). Does not raise."""
    errors, unchecked = [], []
    try:
        secs = sections(doc)
    except PlanError as e:
        return {"applied": {}, "ignored": {}, "errors": [str(e)], "unchecked": []}
    applied, ignored = {}, {}
    for node, keys in secs.items():
        if node == BT_NODE:
            base = bt_base
        elif node in NAV_NODES:
            if not nav_started:
                ignored[node] = dict(keys)
                continue
            base = nav_base
        else:
            errors.append("'%s' is not a node this overlay reaches: bt_runner_node always; %s only when the nav stack "
                          "is started" % (node.replace("/", "."), ", ".join(NAV_NODES)))
            continue
        if base is None:
            unchecked.append(node)
        for key, v in keys.items():
            if base is not None:
                found, bv = _base_value(base, node, key)
                if not found:
                    errors.append("%s: %s is not a parameter in the boat's params file: ROS would silently ignore it"
                                  % (node, key))
                    continue
                if bv is not None and not _same_type(v, bv):
                    errors.append("%s: %s is %s but the boat's file has %s (%r): the node would refuse to start"
                                  % (node, key, type(v).__name__, type(bv).__name__, bv))
                    continue
            applied.setdefault(node, {})[key] = v
    return {"applied": applied, "ignored": ignored, "errors": errors, "unchecked": unchecked}


def _nest(applied):
    """{node: {key: v}} -> the params-file document (dotted keys become nested maps)."""
    doc = {}
    for node, keys in applied.items():
        d = doc
        for k in node.split("/") + ["ros__parameters"]:
            d = d.setdefault(k, {})
        for key, v in keys.items():
            *parents, leaf = key.split(".")
            dd = d
            for p in parents:
                dd = dd.setdefault(p, {})
            dd[leaf] = v
    return doc


def finish(decision, nav_mode, cfg, nav2_cfg):
    """The overlay, checked, for the nav_mode the rig really runs. -> (plan, applied_doc or None, lines).
    PlanError when the file is missing or unusable (all problems in ONE message)."""
    path = decision["tuning"]
    if not path:
        return {"applied": {}, "ignored": {}, "errors": [], "unchecked": []}, None, []
    doc = _load_yaml(path, "LAKE_TUNING")
    nav = nav_mode != "off"
    bt_base = _load_yaml(cfg, "the boat's params") if cfg and os.path.isfile(cfg) else None
    nav_base = _load_yaml(nav2_cfg, "Nav2's params") if nav and nav2_cfg and os.path.isfile(nav2_cfg) else None
    plan = overlay_plan(doc, nav, bt_base, nav_base)
    if plan["errors"]:
        raise PlanError("%s:\n    %s" % (path, "\n    ".join(plan["errors"])))
    lines = []
    n = sum(len(k) for k in plan["applied"].values())
    lines.append("  planner overlay, checked against the boat's params files: %d key(s) applied" % n)
    for node, keys in plan["applied"].items():
        target = "bt_runner (2nd --params-file)" if node == BT_NODE else "Nav2 (nav2_overlay:=)"
        lines.append("    %s -> %s" % (node, target))
        lines += ["      %s = %s" % (k, v) for k, v in keys.items()]
    for node, keys in plan["ignored"].items():
        lines.append("    IGNORED %s (%d key(s): %s): nav is not started (nav_mode off), so they would do nothing"
                     % (node, len(keys), ", ".join(keys)))
    if plan["unchecked"]:
        lines.append("    (not checked against the boat's own file for: %s: it could not be read here)"
                     % ", ".join(plan["unchecked"]))
    return plan, _nest(plan["applied"]), lines


def rig_info(decision, nav_mode, nav_why, plan, publish):
    """What the page shows about the rig (rig.json)."""
    return {"tree": os.path.basename(decision["tree"]), "global_tree": decision["global_tree"],
            "nav_mode": nav_mode, "nav_why": nav_why, "notes": decision["notes"], "pool": decision["pool"],
            "publish": bool(publish), "tuning": decision["tuning"] or None,
            "tuning_why": decision["tuning_why"] or None,
            "applied": {n: dict(k) for n, k in plan["applied"].items()},
            "ignored": {n: sorted(k) for n, k in plan["ignored"].items()}}


# ------------------------------------------------------------------ reading the overlay back from the running node

def _num(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def verify_dump(applied, dump_doc):
    """Compare bt_runner's overlay keys with `ros2 param dump /bt_runner_node`. -> (matching, [problem]).
    `applied` is the plan's {node: {key: v}}; only bt_runner_node is read back."""
    want = applied.get(BT_NODE, {})
    try:
        have = dump_doc["/" + BT_NODE]["ros__parameters"]
    except (KeyError, TypeError):
        return 0, ["the dump has no /%s ros__parameters" % BT_NODE]
    ok, bad = 0, []
    for key, v in want.items():
        d = have
        for k in key.split("."):
            d = d.get(k) if isinstance(d, dict) else None
        if d is None:
            bad.append("%s: not on the node (expected %r)" % (key, v))
        elif _num(v) and _num(d) and math.isclose(float(v), float(d), rel_tol=DUMP_TOL, abs_tol=DUMP_TOL):
            ok += 1
        elif v == d:
            ok += 1
        else:
            bad.append("%s: the node has %r, the overlay says %r" % (key, d, v))
    return ok, bad


def read_back(retries=4):
    """`ros2 param dump /bt_runner_node --print` parsed, or (None, why)."""
    why = "no attempt"
    for i in range(retries):
        try:
            r = subprocess.run(["ros2", "param", "dump", "/" + BT_NODE, "--no-daemon", "--print"],
                               capture_output=True, text=True, timeout=25)
        except (OSError, subprocess.TimeoutExpired) as e:
            why = str(e)
        else:
            if r.returncode == 0 and r.stdout.strip():
                try:
                    return yaml.safe_load(r.stdout), None
                except yaml.YAMLError as e:
                    why = "unparseable dump: %s" % e
            else:
                lines = (r.stderr or r.stdout or "").strip().splitlines()
                why = lines[-1] if lines else "exit %d" % r.returncode
    return None, why


# ------------------------------------------------------------------ the command line

def sh_assignments(decision, banner):
    pairs = (("PLAN_TREE", decision["tree"]), ("PLAN_NAV_MODE", decision["nav_mode"]),
             ("PLAN_NAV_WHY", decision["nav_why"]), ("PLAN_POOL", "1" if decision["pool"] else "0"),
             ("PLAN_GLOBAL", "1" if decision["global_tree"] else "0"),
             ("PLAN_TUNING", decision["tuning"]), ("PLAN_BANNER", banner))
    return "\n".join("%s=%s" % (k, shlex.quote(str(v))) for k, v in pairs) + "\n"


def main(argv=None, env=None):
    env = os.environ if env is None else env
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("decide")
    f = sub.add_parser("finish")
    f.add_argument("--nav-mode", required=True, choices=NAV_MODES)
    f.add_argument("--cfg", default="")
    f.add_argument("--nav2-cfg", default="")
    f.add_argument("--out-dir", default="")
    f.add_argument("--publish", default="false")
    f.add_argument("--dry", action="store_true")
    v = sub.add_parser("verify")
    v.add_argument("applied")
    a = ap.parse_args(argv)
    try:
        if a.cmd == "verify":
            return _verify(a.applied)
        d = decide(env)
        if a.cmd == "decide":
            sys.stdout.write(sh_assignments(d, "\n".join(d["lines"])))
            return 0
        nav_why = d["nav_why"] if a.nav_mode == d["nav_mode"] else (
            "%s, then forced to %s: the nav packages are not installed here" % (d["nav_why"], a.nav_mode))
        plan, applied, lines = finish(d, a.nav_mode, a.cfg, a.nav2_cfg)
        if lines:
            print("\n".join(lines))
        if not a.dry and a.out_dir:
            os.makedirs(a.out_dir, exist_ok=True)
            _remove(os.path.join(a.out_dir, APPLIED_FILE))          # never a previous run's overlay
            if applied:
                with open(os.path.join(a.out_dir, APPLIED_FILE), "w", encoding="utf-8") as fh:
                    fh.write("# Written by lake_rig_up.sh (crusader_sim.lake_rig_plan): the keys of %s that this run\n"
                             "# really applies. bt_runner reads it as its 2nd --params-file; Nav2 as nav2_overlay.\n" % d["tuning"])
                    yaml.safe_dump(applied, fh, default_flow_style=False, sort_keys=False)
            with open(os.path.join(a.out_dir, RIG_FILE), "w", encoding="utf-8") as fh:
                json.dump(rig_info(d, a.nav_mode, nav_why, plan, truthy(a.publish)), fh, indent=1)
        return 0
    except PlanError as e:
        sys.stderr.write("*** %s\n" % e)
        return 2


def _remove(path):
    try:
        os.remove(path)
    except OSError:
        pass


def _verify(path):
    try:
        with open(path, encoding="utf-8") as fh:
            applied_doc = yaml.safe_load(fh) or {}
    except OSError as e:
        print("  *** overlay read-back skipped: cannot read %s (%s)" % (path, e.strerror))
        return 1
    applied = sections(applied_doc)
    want = applied.get(BT_NODE, {})
    if not want:
        print("  overlay read-back: no bt_runner_node keys to check")
        return 0
    dump, why = read_back()
    if dump is None:
        print("  *** overlay read-back FAILED (%s): check by hand: ros2 param get /bt_runner_node %s"
              % (why, next(iter(want))))
        return 1
    ok, bad = verify_dump(applied, dump)
    if bad:
        print("  *** overlay read-back: %d of %d bt_runner key(s) did NOT take effect:" % (len(bad), len(want)))
        for b in bad:
            print("      " + b)
        return 1
    print("  overlay read back from /bt_runner_node: %d/%d key(s) match" % (ok, len(want)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
