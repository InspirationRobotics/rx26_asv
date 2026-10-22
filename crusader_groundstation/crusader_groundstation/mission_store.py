"""mission_store — the boat's missions as files, and the catalogue they are built from.

A mission is a BehaviorTree.CPP file run BY NAME (bt_runner_node, crusader_bt/mission_loader.hpp):
a built-in from crusader_bt's share/behavior_trees/, or one the laptop uploaded into
user_trees_dir, which gcs_bridge and bt_runner_node both take as the same parameter. Standard
library only: gcs_bridge hands this module the two things that need the install (the text of
`bt_tree_check --models` and the directories), so everything here runs and tests without ROS.

THE CATALOGUE (what mission_xml and the laptop's Mission tab build blocks from):
  nodes     every registered leaf and BT.CPP's own controls and decorators, from the TreeNodesModel
            bt_tree_check prints, with a `required` on each port (see port_required)
  subtrees  the per-tier task blocks, one per lib/task<N>_<tier>.xml: {id, task, tier, ports}. The
            ports are the attributes of the SubTree in the built-in wrapper <stem>.xml, which
            writes out every tunable with its default; the type of each is that of the leaf port
            the tunable feeds. The shared bodies in lib/common/ are listed WITHOUT task or tier:
            known IDs (so a mission naming one validates), not blocks to offer
  builtins  the names a user mission may not take
  user      {name, bytes, mtime} of each uploaded mission

THE FILES: <user_trees_dir>/<name>.xml. A file replaced or deleted is MOVED, never removed, into
<user_trees_dir>/.archive/<name>.<YYYYmmdd-HHMMSS>.xml, and a new one is written beside the old and
os.replace()d into place, so the runner reads the old mission or the new one, never half of one.
"""
import os
import re
import shutil
import tempfile
import time
import xml.etree.ElementTree as ET

ARCHIVE = ".archive"
_KINDS = ("Action", "Condition", "Control", "Decorator")
_DIRS = {"input_port": "in", "output_port": "out", "inout_port": "inout"}
_SUBTREE_FILE = re.compile(r"task(\d+)_([a-z0-9]+)")
_ABSENT = re.compile(r"\(absent\b", re.IGNORECASE)         # the leaves' own mark of an optional port
_BLACKBOARD = re.compile(r"\{([^{}]+)\}")
_REAL, _INT = re.compile(r"[+-]?\d+\.\d+"), re.compile(r"[+-]?\d+")
_NOT_A_TUNABLE = ("ID", "name", "_autoremap")

# Input ports with no default that the leaf still runs without, and the doc does not say "absent".
# NavigateTo reads `goal` only for target="port" (mission_xml checks that pairing itself);
# AwaitFireTarget uses `bay` only to re-send the docking report, and waits without it.
OPTIONAL_PORTS = frozenset({("NavigateTo", "goal"), ("AwaitFireTarget", "bay")})


# ------------------------------------------------------------------------------ the catalogue

def port_required(node_id, port):
    """True when the node cannot do its job without this input. BT.CPP cannot say "optional", so
    the Crusader leaves do: a default in the port, or "(absent: ...)" in its doc, as in
    CircleBuoy's `radius`. Every other input without a default is read with a plain getInput (or
    fails the leaf when it is missing), and the BT.CPP built-ins (Sleep.msec, Script.code,
    SetBlackboard.value ...) throw without theirs. OPTIONAL_PORTS holds the exceptions."""
    return (port["dir"] != "out" and port["default"] is None
            and not _ABSENT.search(port["doc"]) and (node_id, port["name"]) not in OPTIONAL_PORTS)


def _tidy_default(type_name, default):
    """BT.CPP prints a double as 0.500000: the catalogue says 0.5, as the wrappers write it."""
    if default is not None and type_name in ("double", "float") and _REAL.fullmatch(default):
        return default.rstrip("0") + ("0" if default.rstrip("0").endswith(".") else "")
    return default


def parse_models(text):
    """The catalogue's `nodes` from the TreeNodesModel XML of `bt_tree_check --models`."""
    nodes = []
    for el in ET.fromstring(text).iter():
        if el.tag not in _KINDS:
            continue
        ports = []
        for p in el:
            if p.tag not in _DIRS:
                continue
            type_name = p.get("type", "")
            ports.append({"name": p.get("name"), "dir": _DIRS[p.tag], "type": type_name,
                          "default": _tidy_default(type_name, p.get("default")),
                          "doc": (p.text or "").strip()})
        for port in ports:
            port["required"] = port_required(el.get("ID"), port)
        nodes.append({"id": el.get("ID"), "kind": el.tag.lower(), "ports": ports})
    return nodes


def _trees(directory):
    """The root element of each *.xml directly in `directory`, by file stem."""
    return {stem: ET.parse(os.path.join(directory, stem + ".xml")).getroot()
            for stem in stems(directory)}


def _tree_ids(root):
    return [t.get("ID") for t in root.iter("BehaviorTree") if t.get("ID")]


def _infer_type(default):
    if _INT.fullmatch(default):
        return "int"
    if _REAL.fullmatch(default):
        return "double"
    return "bool" if default in ("true", "false") else "std::string"


def _fed_types(roots, nodes):
    """{blackboard name: the type of the leaf port it is wired to} over the given trees: a
    tunable `lookahead_m` is a double because DrivePlanPhase reads it as `{lookahead_m}`."""
    by_id = {n["id"]: {p["name"]: p["type"] for p in n["ports"]} for n in nodes}
    fed = {}
    for root in roots:
        for el in root.iter():
            ports = by_id.get(el.get("ID") if el.tag in _KINDS else el.tag, {})
            for attr, val in el.attrib.items():
                m = _BLACKBOARD.fullmatch(val)
                if m and attr in ports:
                    fed.setdefault(m.group(1), ports[attr])
    return fed


def _subtree_ports(tier_root, wrapper_root, sid, common, nodes):
    """A task block's tunables: names and defaults from its wrapper, docs from its own
    TreeNodesModel, types from the leaves they feed (or, failing that, from the default)."""
    model = {p.get("name"): p for p in tier_root.findall(f"TreeNodesModel/SubTree[@ID='{sid}']/input_port")}
    use = None if wrapper_root is None else next(
        (el for el in wrapper_root.iter("SubTree") if el.get("ID") == sid), None)
    wanted = ([(k, v) for k, v in use.attrib.items() if k not in _NOT_A_TUNABLE] if use is not None
              else [(k, p.get("default", "")) for k, p in model.items()])
    fed = _fed_types([tier_root, *common], nodes)
    return [{"name": name, "dir": "in", "type": fed.get(name) or _infer_type(default),
             "default": default, "doc": (model[name].text or "").strip() if name in model else "",
             "required": False} for name, default in wanted]


def lib_subtrees(trees_dir, nodes):
    """The catalogue's `subtrees`: every tier file of lib/, then the shared bodies of lib/common/."""
    lib, common = _trees(os.path.join(trees_dir, "lib")), _trees(os.path.join(trees_dir, "lib", "common"))
    wrappers, out = _trees(trees_dir), []
    for stem, root in lib.items():
        m = _SUBTREE_FILE.fullmatch(stem)
        ids = _tree_ids(root)
        if m and ids:
            out.append({"id": ids[0], "task": int(m.group(1)), "tier": m.group(2),
                        "ports": _subtree_ports(root, wrappers.get(stem), ids[0], common.values(), nodes)})
    out.extend({"id": sid, "ports": []} for root in common.values() for sid in _tree_ids(root))
    return out


def catalogue(trees_dir, nodes, store):
    """{nodes, subtrees, builtins, user}: what mission_catalogue answers. `nodes` is parse_models'."""
    return {"nodes": nodes, "subtrees": lib_subtrees(trees_dir, nodes),
            "builtins": store.builtins(), "user": store.user_entries()}


# ------------------------------------------------------------------------------------ the files

def stems(directory):
    """The names of the .xml files directly in `directory`, sorted; none when it does not exist."""
    try:
        return sorted(f[:-4] for f in os.listdir(directory)
                      if f.endswith(".xml") and os.path.isfile(os.path.join(directory, f)))
    except OSError:
        return []


class MissionStore:
    """The built-in directory (read only) and the user's (read and written here)."""

    def __init__(self, builtin_dir, user_dir):
        self.builtin_dir, self.user_dir = builtin_dir, user_dir

    def builtins(self):
        return stems(self.builtin_dir)

    def user(self):
        """The user's mission names. One a built-in also has is left out: the runner runs the
        built-in under that name, so listing it would offer a mission that is not what it says."""
        builtin = set(self.builtins())
        return [n for n in stems(self.user_dir) if n not in builtin]

    def runnable(self):
        return self.builtins() + self.user()

    def user_entries(self):
        out = []
        for name in self.user():
            st = os.stat(self.path(name))
            out.append({"name": name, "bytes": st.st_size, "mtime": round(st.st_mtime, 1)})
        return out

    def path(self, name):
        return os.path.join(self.user_dir, name + ".xml")

    def read(self, name):
        """(xml, builtin) of a runnable mission; KeyError if `name` is neither. `name` is matched
        against the listings, so it can never be a path."""
        if name in self.builtins():
            return _read(os.path.join(self.builtin_dir, name + ".xml")), True
        if name in self.user():
            return _read(self.path(name)), False
        raise KeyError(name)

    def stage(self, name, xml):
        """Writes `xml` beside where `name` will live and returns that path: complete and flushed
        to disk, but not yet a mission. `commit` makes it one; `discard` throws it away."""
        os.makedirs(self.user_dir, exist_ok=True)
        fd, staged = tempfile.mkstemp(dir=self.user_dir, prefix=f".{name}.", suffix=".tmp")
        with os.fdopen(fd, "wb") as f:
            f.write(xml.encode("utf-8"))
            f.flush()
            os.fsync(f.fileno())
        os.chmod(staged, 0o644)         # mkstemp's 0600 would hide it from the host's own user
        return staged

    def commit(self, name, staged):
        """The staged file becomes `name`; the file it replaces goes to the archive first.
        Returns the archive path, or None when the name was new."""
        archived = self._archive(name) if os.path.isfile(self.path(name)) else None
        os.replace(staged, self.path(name))
        return archived

    @staticmethod
    def discard(staged):
        try:
            os.remove(staged)
        except OSError:
            pass

    def delete(self, name):
        """Moves the user's `name` into the archive and returns where it went."""
        return self._archive(name, move=True)

    def _archive(self, name, move=False):
        os.makedirs(os.path.join(self.user_dir, ARCHIVE), exist_ok=True)
        stamp, again = time.strftime("%Y%m%d-%H%M%S"), 1
        dest = os.path.join(self.user_dir, ARCHIVE, f"{name}.{stamp}.xml")
        while os.path.exists(dest):     # two changes inside one second must not overwrite each other
            again += 1
            dest = os.path.join(self.user_dir, ARCHIVE, f"{name}.{stamp}-{again}.xml")
        (os.replace if move else shutil.copy2)(self.path(name), dest)
        return dest


def _read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()
