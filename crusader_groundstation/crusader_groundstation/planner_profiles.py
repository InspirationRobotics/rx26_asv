"""planner_profiles — named planner-tuning sets for bt_runner_node, as files.

WHAT A PROFILE IS. A ROS 2 params file with ONLY the keys that differ from
crusader_params.yaml / nav2_params.yaml, nested like those files. It is the format the
sim's Task 1 panel writes (crusader_sim/config/tuning_profiles/*.yaml, `gz_sim_up.sh
--tuning`) and the lake rig reads (`LAKE_TUNING=`), so one file serves all three:

    # title: Tight field (3-5 m between buoys)
    # about: 3 m orbits ...
    bt_runner_node:
      ros__parameters:
        nav_orbit_radius_m: 3.0
    planner_server:                 # Nav2: only at rig LAUNCH, never from here
      ros__parameters:
        GridBased: {tolerance: 0.3}

WHAT THE GROUND STATION DOES WITH ONE. Loading applies ONLY the bt_runner_node nav_* keys
(the planner knobs bt_runner re-reads at the next START), through the Tuning tab's ordinary
/params/set path, so the node's own set-callback judges every value. Everything else in
the file is LISTED as skipped and why, never applied silently: Nav2's planner_server and
global_costmap are configured when the rig launches and cannot be changed live, and a
bt_runner_node key that is not a nav_* planner knob (publish_setpoints!) has no business
arriving from a file picker.

WHERE THEY LIVE. Two directories, listed together:
  shipped  the repo's crusader_sim/config/tuning_profiles (a parameter; read-only here)
  saved    ~/.cache/crusader_lake/tuning, where "Save as profile" writes -- the lake rig's
           own log directory, so `LAKE_TUNING=<that file>` finds it on the same disk.

BLANKS OVER GUESSES. A directory that is missing is a listing with no profiles and the
reason, not an empty list that looks like "none were ever made"; a file that does not
parse is listed with its error, not dropped.

Pure apart from the filesystem: no ROS, so the whole of it is tested off-boat.
"""
import os
import re
import time

import yaml

NODE = "bt_runner_node"
NODE_PATH = "/" + NODE
PLANNER_PREFIX = "nav_"

DEFAULT_SHIPPED_DIR = "/root/robotx_ws/src/rx26_asv/crusader_sim/config/tuning_profiles"
DEFAULT_SAVE_DIR = "~/.cache/crusader_lake/tuning"

# The sim panel's rule (crusader_sim.task1_panel.NAME_RE), so a name that works there works here.
NAME_RE = re.compile(r"^[A-Za-z0-9_-]{1,40}$")

SHIPPED = "shipped"
SAVED = "saved"

_META_RE = re.compile(r"^#\s*(title|about)\s*:\s*(.*)$", re.IGNORECASE)


def valid_name(name):
    return isinstance(name, str) and bool(NAME_RE.match(name))


def _is_number(v):
    return isinstance(v, (int, float)) and not isinstance(v, bool)


def _leaves(mapping, prefix=()):
    """(dotted-path tuple, value) for every leaf of a nested mapping."""
    for k, v in mapping.items():
        if isinstance(v, dict):
            yield from _leaves(v, prefix + (str(k),))
        else:
            yield prefix + (str(k),), v


def split_profile(doc):
    """(values, skipped) of a parsed profile.

    values   {nav_key: number} -- bt_runner_node.ros__parameters, nav_* keys with a numeric value.
    skipped  [{"key": dotted, "why": sentence}] -- every other leaf, with the reason it is not applied.
    """
    values, skipped = {}, []
    for path, v in _leaves(doc if isinstance(doc, dict) else {}):
        shown = ".".join(p for p in path if p != "ros__parameters")
        if path[:2] == (NODE, "ros__parameters") and len(path) == 3:
            key = path[2]
            if not key.startswith(PLANNER_PREFIX):
                skipped.append({"key": shown, "why": "not a nav_* planner knob"})
            elif not _is_number(v):
                skipped.append({"key": shown, "why": "not a number"})
            else:
                values[key] = v
        elif path[0] == NODE:
            skipped.append({"key": shown, "why": "not a nav_* planner knob"})
        else:
            skipped.append({"key": shown, "why": "Nav2 / another node: applied only when the rig "
                                                 "launches, needs a rig restart"})
    return values, skipped


def _header(text):
    """title / about from the leading `# title: ...` / `# about: ...` comment lines."""
    meta = {}
    for line in text.splitlines():
        line = line.strip()
        if line and not line.startswith("#"):
            break                          # the header is the comments before the first key
        m = _META_RE.match(line)
        if m and m.group(2).strip():
            meta[m.group(1).lower()] = m.group(2).strip()
    return meta


def read_profile(path, origin):
    """One profile file as a listing row. A file that does not parse is a row with `error`."""
    name = os.path.splitext(os.path.basename(path))[0]
    row = {"origin": origin, "name": name, "title": name, "about": "",
           "values": {}, "skipped": [], "error": ""}
    try:
        with open(path, encoding="utf-8") as f:
            text = f.read()
        row.update({k: v for k, v in _header(text).items() if k in ("title", "about")})
        doc = yaml.safe_load(text)
        if doc is not None and not isinstance(doc, dict):
            raise ValueError("not a YAML mapping")
        row["values"], row["skipped"] = split_profile(doc)
    except (OSError, ValueError, yaml.YAMLError) as e:
        row["error"] = (str(e).splitlines() or ["unreadable"])[0]
    return row


def listing(sources):
    """Every profile in every directory, and the state of each directory.

    Args:
      sources: [(origin, directory)], shipped first. A blank directory is "not configured".

    Returns:
      {"sources": [{origin, dir, ok, reason}], "profiles": [row, ...]} -- profiles sorted by
      (name, origin). A directory that is absent or unreadable has ok False and the reason, and
      contributes no profiles: the reason is the answer to "why is the list empty".
    """
    states, rows = [], []
    for origin, directory in sources:
        state = {"origin": origin, "dir": directory, "ok": True, "reason": ""}
        states.append(state)
        if not directory:
            state.update(ok=False, reason="no directory configured")
            continue
        try:
            files = sorted(f for f in os.listdir(directory)
                           if f.endswith(".yaml") and valid_name(f[:-5]))
        except FileNotFoundError:
            state.update(ok=False, reason=f"{directory} does not exist")
            continue
        except OSError as e:
            state.update(ok=False, reason=f"{directory}: {e.strerror or e}")
            continue
        rows += [read_profile(os.path.join(directory, f), origin) for f in files]
    rows.sort(key=lambda r: (r["name"], r["origin"]))
    return {"sources": states, "profiles": rows}


def is_planner_row(row):
    """A /params/list row that is a planner knob: nav_*, numeric, and editable.

    The structural nav_* keys (rates, frames, topics, nav_mode) are read-only on a current
    bt_runner_node and strings or rates on an old one; neither belongs in a profile."""
    return (row["name"].startswith(PLANNER_PREFIX) and row.get("editable")
            and row.get("type") in ("double", "integer") and _is_number(row.get("value")))


def _differs(row):
    return abs(row["value"] - row["default"]) > 1e-9 if _is_number(row.get("default")) else True


def drifted_planner_values(rows):
    """({name: value}, note) -- the planner knobs whose live value differs from the YAML's.

    `rows` is ParamBridge.list's output for bt_runner_node. The comparison is against
    crusader_params.yaml (the same drift the Tuning tab marks), so what is saved is the set of
    lines a person would otherwise have to write back by hand. A knob with no YAML default has
    nothing to differ FROM and is not saved; if none of them has one, say so rather than
    returning an empty set that reads as "nothing changed".
    """
    planner = [r for r in rows if is_planner_row(r)]
    if not planner:
        return {}, f"{NODE} declares no editable {PLANNER_PREFIX}* planner knobs"
    known = [r for r in planner if r.get("in_yaml") and _is_number(r.get("default"))]
    if not known:
        return {}, (f"crusader_params.yaml has no {NODE} {PLANNER_PREFIX}* defaults here, so "
                    "there is nothing to compare with and no way to tell what changed")
    return {r["name"]: _clean(r["value"]) for r in known if _differs(r)}, ""


def _clean(v):
    """A float the way a person would write it: 0.30000000000000004 -> 0.3."""
    return float(f"{v:.10g}") if isinstance(v, float) else v


def profile_text(name, values, about=None):
    """The file a save writes: the header, then only these keys, nested like crusader_params.yaml."""
    about = about or ("Saved from the ground station's Tuning tab "
                      + time.strftime("%Y-%m-%d %H:%M") + ": the nav_* planner keys that differed "
                      "from crusader_params.yaml. Load profile in the Tuning tab applies them at the "
                      "next START; LAKE_TUNING=<this file> applies them when the rig launches.")
    doc = {NODE: {"ros__parameters": {k: values[k] for k in sorted(values)}}}
    return (f"# title: {name}\n# about: {about}\n"
            + yaml.safe_dump(doc, default_flow_style=False, sort_keys=False))


def save(save_dir, name, values):
    """Write the profile atomically. Returns (ok, message, path).

    Overwrites a file of the same name: the page asks first. The name is checked against NAME_RE
    and never joined to the directory otherwise, so a name cannot name a path.
    """
    if not valid_name(name):
        return False, "profile name: letters, digits, - and _ only (max 40)", ""
    if not values:
        return False, "nothing to save", ""
    path = os.path.join(save_dir, name + ".yaml")
    tmp = path + ".tmp"
    try:
        os.makedirs(save_dir, exist_ok=True)
        with open(tmp, "w", encoding="utf-8") as f:
            f.write(profile_text(name, values))
        os.replace(tmp, path)
    except OSError as e:
        return False, f"could not write {path}: {e.strerror or e}", ""
    return True, f"saved {len(values)} planner keys to {path}", path
