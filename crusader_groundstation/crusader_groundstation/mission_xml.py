"""mission_xml — what a mission file is, for the laptop's gcs_app and the boat's gcs_bridge alike.

The Mission tab edits a mission as nested blocks; the boat keeps it as BehaviorTree.CPP XML and
runs it by name. Everything that decides whether a mission is acceptable lives here, once:
gcs_app builds, reads back and checks with it before it uploads, and gcs_bridge imports the same
file for its upload refusals, so the two cannot disagree. Standard library only, no ROS.

THE CATALOGUE COMES FROM THE BOAT (op mission_catalogue), never from this file. Its `nodes` are
every registered leaf plus BT.CPP's own controls and decorators; its `subtrees` are the per-tier
task blocks (Task1Advanced ...), each with its `task`, its `tier` and its tunable `ports`, plus
the shared bodies they call (Task1PlanDrive ...), which carry no task or tier: they are known IDs
but not blocks to offer, and they take no part in the one-tier rule; its `builtins` are the
mission names a user may not take. A port is {name, dir, type, default, doc}
(dir in | out | inout). `default` is None when the port has none and "" when its default is the
empty string. A port is REQUIRED when it is an input with no default and a scalar type, unless
the entry says `"required": false` (BT.CPP cannot say "optional"; ports documented "absent: nav_*"
need that key).

A BLOCK is {"id", "name"?, "ports": {name: text}, "children": [block]}, plus "subtree": true for a
task block. `build` turns the blocks the Mission tab holds into the one shape the boat accepts:

    <root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">
      <ReactiveSequence><IsAutonomous/> BODY </ReactiveSequence></BehaviorTree></root>

The guard is always emitted, never edited. BODY is exactly one block, a Sequence: a
ReactiveSequence re-ticks its finished children, so blocks side by side under it would run
again. There is no <include>: a task block is <SubTree ID="Task1Advanced" _autoremap="true" .../>
and the boat supplies the tree. `read_blocks` goes the other way, and also opens the boat's
built-in missions (which carry includes and their own guard) by leaving those out.
"""
import copy
import re
import xml.etree.ElementTree as ET

MAX_BYTES = 256 * 1024
MAX_BLOCKS = 2000
MAX_DEPTH = 40
NAME = re.compile(r"[a-z][a-z0-9_]{0,39}")
MAIN, GUARD, AUTONOMOUS, BODY = "Main", "ReactiveSequence", "IsAutonomous", "mission"
EXPECTED = ('<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">'
            "<ReactiveSequence><IsAutonomous/> BODY </ReactiveSequence></BehaviorTree></root>")

_GENERIC_TAGS = ("Action", "Condition", "Control", "Decorator")      # <Action ID="Foo"/> is <Foo/>
_ARITY = {"IfThenElse": (2, 3), "WhileDoElse": (2, 3)}               # they throw at tick otherwise
_FIX_SELECTOR = {"NavigateTo": "target", "CircleBuoy": "anchor"}     # which port says "a GPS point"
_XML_NAME = re.compile(r"[A-Za-z_][A-Za-z0-9_.-]*")           # a block id, a port name
_BLACKBOARD = re.compile(r"\{[^{}]+\}")
_INT = re.compile(r"[+-]?\d+")
_REAL = re.compile(r"[+-]?\d+\.\d+")
_NUMBER = re.compile(r"[+-]?(\d+\.?\d*|\.\d+)([eE][+-]?\d+)?")
_INT_TYPE = re.compile(r"(unsigned )?(int|long|short)|u?int\d+_t|size_t")
_BOOLS = ("true", "false", "1", "0")
_NO_CONTROL = re.compile(r"[\x00-\x1f\x7f]")


class BlockError(ValueError):
    """Blocks that cannot be turned into XML at all (a bug in the sender, not a refusal)."""


class Catalogue:
    """The boat's mission_catalogue, indexed for lookup."""

    def __init__(self, raw):
        self.raw = raw
        try:
            self.nodes = {n["id"]: n for n in raw["nodes"]}
            self.subtrees = {s["id"]: s for s in raw.get("subtrees") or []}
            self.builtins = frozenset(raw.get("builtins") or [])
        except (KeyError, TypeError, AttributeError) as e:
            raise ValueError(f"not a mission catalogue: {type(e).__name__} {e}") from e


def _index(catalogue):
    return catalogue if isinstance(catalogue, Catalogue) else Catalogue(catalogue)


def _ports(entry):
    return [p for p in entry.get("ports") or [] if isinstance(p, dict) and p.get("name")]


def _is_input(port):
    return port.get("dir", "in") in ("in", "inout")


def _kind(type_name):
    """int, real, bool, text or other, for a port type as BT.CPP names it."""
    t = str(type_name or "string").lower().replace("std::", "")
    if _INT_TYPE.fullmatch(t):
        return "int"
    if t in ("double", "float"):
        return "real"
    if t == "bool":
        return "bool"
    return "text" if "string" in t or t == "enum" else "other"


def _required(port):
    if not _is_input(port):
        return False
    if "required" in port:
        return bool(port["required"])
    return port.get("default") is None and _kind(port.get("type")) != "other"


def _max_children(nid, kind):
    """How many blocks a block can hold: None for any number."""
    if nid in _ARITY:
        return _ARITY[nid][1]
    return 0 if kind in ("action", "condition") else 1 if kind == "decorator" else None


def annotate(raw):
    """The catalogue for the Mission tab: a copy with each port's `required` and `kind` filled in,
    each block's `max_children`, and `fix_selector` (which port says "a GPS point"), so the page
    never re-derives a rule this file owns. Validation uses the boat's catalogue as it came."""
    out = copy.deepcopy(_index(raw).raw)
    for entry in out["nodes"]:
        entry["max_children"] = _max_children(entry["id"], entry.get("kind"))
    for entry in out.get("subtrees") or []:
        entry["max_children"] = 0
    for entry in out["nodes"] + (out.get("subtrees") or []):
        for port in _ports(entry):
            port["required"], port["kind"] = _required(port), _kind(port.get("type"))
    out["fix_selector"] = dict(_FIX_SELECTOR)
    return out


def _number(text):
    """The finite float `text` spells, else None (float() alone also takes nan, inf and 1_0)."""
    return float(text) if _NUMBER.fullmatch(text.strip()) else None


def _node_id(el):
    return (el.get("ID") or el.tag) if el.tag in _GENERIC_TAGS else el.tag


def check_name(name, builtins=()):
    """Why `name` cannot be a user mission's name: a list of reasons, empty when it can."""
    if not isinstance(name, str) or not NAME.fullmatch(name):
        return [f"mission name {name!r} must be lower case letters, digits and _, starting with a "
                "letter, at most 40 characters"]
    if name in builtins:
        return [f"{name!r} is a built-in mission: choose another name"]
    return []


# ----------------------------------------------------------------------------- blocks -> XML

def build_xml(blocks, catalogue):
    """The guard-wrapped mission XML for a list of blocks. BlockError for blocks that are not
    blocks; whether the mission is acceptable is `validate`'s question, not this one's."""
    cat = _index(catalogue)
    if not isinstance(blocks, list):
        raise BlockError("the mission's blocks must be a list")
    body, count = ET.Element("Sequence", name=BODY), [0]
    body.extend([_element(b, cat, 1, count) for b in blocks])
    root = ET.Element("root", BTCPP_format="4", main_tree_to_execute=MAIN)
    guard = ET.SubElement(ET.SubElement(root, "BehaviorTree", ID=MAIN), GUARD)
    ET.SubElement(guard, AUTONOMOUS)
    guard.append(body)
    ET.indent(root, space="  ")
    return '<?xml version="1.0" encoding="UTF-8"?>\n' + ET.tostring(root, encoding="unicode") + "\n"


def _text(what, value):
    if isinstance(value, bool):
        value = "true" if value else "false"
    elif isinstance(value, (int, float)):
        value = repr(value)
    if not isinstance(value, str) or _NO_CONTROL.search(value):
        raise BlockError(f"{what} must be plain text")
    return value


def _literal(kind, value):
    """BT.CPP types a SubTree literal by its text (6 is an int, 6.0 a real) and refuses the load
    when that differs from the port's type, so a number is written the way its port wants."""
    if kind == "real" and re.fullmatch(r"[+-]?\d+\.?", value):
        return value.rstrip(".") + ".0"
    if kind == "int" and re.fullmatch(r"[+-]?\d+\.0*", value):
        return value.split(".")[0]
    return value


def _element(block, cat, depth, count):
    count[0] += 1
    if not isinstance(block, dict) or depth > MAX_DEPTH or count[0] > MAX_BLOCKS:
        raise BlockError("a block must be an object, within %d levels and %d blocks"
                         % (MAX_DEPTH, MAX_BLOCKS))
    bid, ports = block.get("id"), block.get("ports") or {}
    kids = block.get("children") or []
    if not isinstance(bid, str) or not _XML_NAME.fullmatch(bid):
        raise BlockError(f"{bid!r} is not a block id")
    if not isinstance(ports, dict) or not isinstance(kids, list):
        raise BlockError(f"block {bid}: ports must be an object and children a list")
    values = {}
    for key, value in ports.items():
        if not isinstance(key, str) or not _XML_NAME.fullmatch(key):
            raise BlockError(f"block {bid}: {key!r} is not a port name")
        values[key] = _text(f"port {key} of {bid}", value)
    attrs = {}
    if block.get("name"):
        attrs["name"] = _text(f"the name of {bid}", block["name"])
    if block.get("subtree"):
        attrs.update(ID=bid, _autoremap="true")
        for port in _ports(cat.subtrees.get(bid) or {}):
            if not _is_input(port):
                continue
            # BT.CPP does not apply a SubTree port's model default, so every tunable is written
            v = values.pop(port["name"], "")
            if v == "" and port.get("default") is not None:
                v = str(port["default"])
            if v != "" or port.get("default") == "":
                attrs[port["name"]] = _literal(_kind(port.get("type")), v)
    attrs.update((k, v) for k, v in values.items() if v != "" or block.get("subtree"))
    el = ET.Element("SubTree" if block.get("subtree") else bid, attrs)
    el.extend([_element(k, cat, depth + 1, count) for k in kids])
    return el


# ----------------------------------------------------------------------------- XML -> blocks

def _parse(xml):
    """The root element of mission text. ValueError for what is not well-formed or not safe."""
    if not isinstance(xml, str):
        raise ValueError("the mission is not text")
    if "<!DOCTYPE" in xml or "<!ENTITY" in xml:
        raise ValueError("a DOCTYPE or ENTITY declaration is not allowed")
    try:
        return ET.fromstring(xml.encode("utf-8", "replace"))
    except ET.ParseError as e:
        raise ValueError(f"not well-formed XML: {e}") from e


def _block(el, depth, count):
    count[0] += 1
    if depth > MAX_DEPTH or count[0] > MAX_BLOCKS:
        raise ValueError("the mission is nested more than %d levels or holds more than %d blocks"
                         % (MAX_DEPTH, MAX_BLOCKS))
    sub = el.tag == "SubTree"
    ports = {k: v for k, v in el.attrib.items()
             if k != "name" and not (k == "ID" and (sub or el.tag in _GENERIC_TAGS))
             and not (sub and k == "_autoremap")}
    if sub and not el.get("ID"):
        raise ValueError("a <SubTree> has no ID")
    block = {"id": el.get("ID") if sub else _node_id(el), "ports": ports,
             "children": [_block(k, depth + 1, count) for k in el]}
    if el.get("name"):
        block["name"] = el.get("name")
    if sub:
        block["subtree"] = True
    return block


def read_blocks(xml, group=None):
    """(blocks, notes) from mission XML: the editor's own files, and the boat's built-ins.

    Left out, and said so in `notes`: <include>s, the trees other than the main one, and the
    autonomy guard (the editor always adds it). A body that is one plain Sequence is unwrapped,
    so build(read_blocks(x)) gives x back for a file the editor made. With `group`, the blocks
    come back as one Sequence of that name: how a mission is inserted into another.
    ValueError if the text is not a tree file."""
    root = _parse(xml)
    if root.tag != "root":
        raise ValueError("not a BehaviorTree.CPP file: the top element must be <root>")
    trees = [t for t in root if t.tag == "BehaviorTree"]
    wanted = root.get("main_tree_to_execute")
    main = next((t for t in trees if t.get("ID") == wanted), trees[0] if len(trees) == 1 and not wanted
                else None)
    if main is None:
        raise ValueError("the file has no main tree (main_tree_to_execute)")
    notes = [f"left out {n} {what}" for n, what in (
        (sum(c.tag == "include" for c in root), "<include>(s): the boat supplies task blocks"),
        (len(trees) - 1, "other tree(s): only the main tree is read")) if n]
    top = list(main)
    if len(top) == 1 and top[0].tag == GUARD and len(top[0]) and top[0][0].tag == AUTONOMOUS:
        top = list(top[0])[1:]
    count = [0]
    blocks = [_block(el, 1, count) for el in top]
    if (len(blocks) == 1 and blocks[0]["id"] == "Sequence" and not blocks[0]["ports"]
            and not blocks[0].get("subtree") and blocks[0].get("name") in (None, BODY)):
        blocks = blocks[0]["children"]
    if group is not None:
        blocks = [{"id": "Sequence", "name": group, "ports": {}, "children": blocks}]
    return blocks, notes


# ----------------------------------------------------------------------------------- validate

class _Check:
    """One validate() run: the findings so far, as (path into the body or None, text)."""

    def __init__(self, cat):
        self.cat = cat
        self.found = []
        self.tiers = {}         # task -> {tier: {task block ids}}

    def add(self, path, text):
        self.found.append((path, text))

    def top(self, root):
        """The body elements of a mission whose top level is exactly the guard, else None and why."""
        trees = [c for c in root if c.tag != "include"]       # an include is reported once, by issues()
        if root.tag != "root" or root.attrib != {"BTCPP_format": "4", "main_tree_to_execute": MAIN}:
            why = 'the root must be <root BTCPP_format="4" main_tree_to_execute="Main">'
        elif len(trees) != 1 or trees[0].tag != "BehaviorTree" or trees[0].attrib != {"ID": MAIN}:
            why = 'the root must hold just <BehaviorTree ID="Main"> (no TreeNodesModel, no other tree)'
        elif len(trees[0]) != 1 or trees[0][0].tag != GUARD or set(trees[0][0].attrib) - {"name"}:
            why = f"Main must hold just one <{GUARD}>"
        elif (not len(trees[0][0]) or trees[0][0][0].tag != AUTONOMOUS or len(trees[0][0][0])
              or set(trees[0][0][0].attrib) - {"name"}):
            why = f"the <{GUARD}> must start with <{AUTONOMOUS}/>"
        else:
            return list(trees[0][0])[1:]
        self.add(None, f"the top level is not the autonomy guard the boat runs missions under: "
                       f"{why}. It is always {EXPECTED}")
        return None

    def body(self, body):
        if not body:
            self.add(None, "the mission is empty: add blocks")
        if len(body) > 1:
            self.add(None, "the mission's blocks must be inside one Sequence: a ReactiveSequence "
                           "re-ticks its finished children, so blocks side by side would run again")
        for i, el in enumerate(body):
            self.node(el, (i,), "", 1)
        if len(body) == 1 and not len(body[0]) and \
                self.cat.nodes.get(_node_id(body[0]), {}).get("kind") == "control":
            self.add(None, "the mission is empty: add blocks")

    def node(self, el, path, where, depth):
        label = el.get("name") or (f"SubTree {el.get('ID')}" if el.tag == "SubTree" else _node_id(el))
        here = label if len(path) == 1 else f"{where} > {label}[{path[-1] + 1}]"
        if el.tag == "include":
            return                                    # reported once, with the rest of the text
        if depth > MAX_DEPTH:
            return self.add(path, f"{here}: nested more than {MAX_DEPTH} levels deep")
        if el.tag == "SubTree":
            self.subtree(el, here, path)
        else:
            nid = _node_id(el)
            spec = self.cat.nodes.get(nid)
            if spec is None:
                self.add(path, f"{here}: unknown block '{nid}' (not in the boat's catalogue)")
            else:
                self.ports(el, spec, here, path)
                self.arity(nid, spec.get("kind"), len(el), here, path)
        for i, kid in enumerate(el):
            self.node(kid, path + (i,), here, depth + 1)

    def arity(self, nid, kind, n, here, path):
        if nid in _ARITY:
            lo, hi = _ARITY[nid]
            if not lo <= n <= hi:
                self.add(path, f"{here}: needs {lo} to {hi} children, has {n}")
        elif kind in ("action", "condition") and n:
            self.add(path, f"{here}: is a leaf and cannot hold blocks (has {n})")
        elif kind == "decorator" and n != 1:
            self.add(path, f"{here}: a decorator needs exactly one child, has {n}")

    def subtree(self, el, here, path):
        sid = el.get("ID") or ""
        spec = self.cat.subtrees.get(sid)
        if spec is None:
            return self.add(path, f"{here}: '{sid}' is not a task block the boat offers")
        if el.get("_autoremap") != "true":
            self.add(path, f'{here}: a task block needs _autoremap="true" (every block reads its '
                           "context from the blackboard, which a SubTree gets its own of)")
        if len(el):
            self.add(path, f"{here}: a task block cannot hold other blocks")
        if spec.get("task") is not None and spec.get("tier"):       # a shared body has neither
            self.tiers.setdefault(spec["task"], {}).setdefault(spec["tier"], set()).add(sid)
        self.ports(el, spec, here, path, subtree=True)

    def ports(self, el, spec, here, path, subtree=False):
        by_name = {p["name"]: p for p in _ports(spec)}
        for attr, val in el.attrib.items():
            if attr == "name" or (attr == "ID" and (subtree or el.tag in _GENERIC_TAGS)) \
                    or (subtree and attr == "_autoremap"):
                continue
            port = by_name.get(attr)
            if port is None:
                self.add(path, f"{here}: unknown port '{attr}'")
            elif val.strip() or not _required(port):      # an empty required port is said once, below
                why = _bad_value(port, val, subtree)
                if why:
                    self.add(path, f"{here}: port '{attr}' {why}")
        for port in by_name.values():
            if _required(port) and not el.get(port["name"], "").strip():
                self.add(path, f"{here}: required port '{port['name']}' is empty")
        if el.tag != "SubTree":
            self.rules(el, here, path)

    def rules(self, el, here, path):
        """What no port type can say: where a GPS block points, what target=port needs."""
        sel = _FIX_SELECTOR.get(_node_id(el))
        if sel and el.get(sel) == "fix":
            found = {}
            for key, limit in (("lat", 90.0), ("lon", 180.0)):
                raw = el.get(key)
                value = _number(raw) if raw is not None else None
                if raw is None or not raw.strip():
                    self.add(path, f"{here}: a GPS point needs a {key}")
                elif value is None:
                    self.add(path, f"{here}: {key} {raw!r} is not a number")
                elif abs(value) > limit:
                    self.add(path, f"{here}: {key} {raw} is outside -{limit:g}..{limit:g}")
                else:
                    found[key] = value
            if found.get("lat") == 0 and found.get("lon") == 0:
                self.add(path, f"{here}: lat and lon are both 0: that is the default, not a place")
        if _node_id(el) == "NavigateTo" and el.get("target") == "port" and not el.get("goal"):
            self.add(path, f'{here}: target="port" needs goal="{{name}}" naming the point to drive to')

    def one_tier_per_task(self):
        for task, tiers in self.tiers.items():
            if len(tiers) > 1:
                used = ", ".join(sorted(i for ids in tiers.values() for i in ids))
                self.add(None, f"Task {task} is used at {len(tiers)} tiers ({used}): a mission "
                               "may use one tier of a task")


def _bad_value(port, val, subtree):
    """Why `val` cannot be this port's value; empty when it can."""
    if _BLACKBOARD.fullmatch(val):
        return ""
    if not _is_input(port):
        return "is an output: it takes a blackboard entry like {goal}"
    kind = _kind(port.get("type"))
    if val == "":
        ok = kind in ("text", "other") or port.get("default") == ""
        return "" if ok else "is empty: leave it out to use its default"
    if kind == "int" and not _INT.fullmatch(val):
        return f"must be a whole number, not {val!r}"
    if kind == "real" and subtree and not _REAL.fullmatch(val):
        return (f"must be written with a decimal point (a SubTree reads a literal by its text, "
                f"and {val!r} would load as a whole number)" if _INT.fullmatch(val)
                else f"must be a number with a decimal point, not {val!r}")
    if kind == "real" and _number(val) is None:
        return f"must be a number, not {val!r}"
    if kind == "bool" and val.lower() not in _BOOLS:
        return f"must be true or false, not {val!r}"
    if port.get("choices") and val not in port["choices"]:
        return "must be one of " + ", ".join(port["choices"])
    return ""


def issues(xml, catalogue, name=None):
    """Every reason this mission cannot be accepted, as (path, text). The path indexes into the
    body (the first number picks BODY's block, then children), or is None for the whole mission."""
    check = _Check(_index(catalogue))
    if name is not None:
        for why in check_name(name, check.cat.builtins):
            check.add(None, why)
    if not isinstance(xml, str):
        check.add(None, "the mission is not text")
        return check.found
    size = len(xml.encode("utf-8", "replace"))
    if size > MAX_BYTES:
        check.add(None, f"the mission is {size // 1024} KiB: more than {MAX_BYTES // 1024} KiB")
        return check.found
    if "<include" in xml:
        check.add(None, "<include> is not allowed: a mission is one file, and a task block is a "
                        "<SubTree> the boat supplies")
    try:
        root = _parse(xml)
    except ValueError as e:
        check.add(None, str(e))
        return check.found
    body = check.top(root)
    if body is not None:
        check.body(body)
    check.one_tier_per_task()
    return check.found


def validate(xml, catalogue, name=None):
    """Human-readable reasons to refuse this mission; empty when it is acceptable. With `name`,
    the name is checked too (lower case, not a built-in)."""
    return [text for _path, text in issues(xml, catalogue, name)]


def problems(xml, catalogue, name=None):
    """`issues` as the Mission tab shows them: {"path": [block indexes into the editor's blocks]
    or None for the mission as a whole, "msg": text}."""
    return [{"path": list(path[1:]) if path and len(path) > 1 else None, "msg": text}
            for path, text in issues(xml, catalogue, name)]


def build(blocks, catalogue, name=None):
    """(xml, problems) for the blocks the Mission tab holds."""
    xml = build_xml(blocks, catalogue)
    return xml, problems(xml, catalogue, name)
