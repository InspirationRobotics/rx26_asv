"""params_yaml: write parameter values into the boat's params YAML, keeping its comments and
layout - the GCS Params tab's "Set + save" (gcs_bridge's params_save op).

The file is crusader_bringup/config/crusader_params.yaml, which every node reads at start (the
installed copy is a symlink to it): a value saved here is the one the node starts with next
time. Only the `<node>:` / `ros__parameters:` / `<name>: <value>` lines are touched. A value is
replaced in place, the line's comment kept at its column; a parameter the node's section does
not list yet is added at the end of that section. Before the file is replaced, the new text
must parse to the same document with only the saved values changed, the old file is copied to
backup_dir, and the replacement is atomic with the owner and mode kept (gcs_bridge runs as root
in the container; the checkout belongs to crusader).

Pure Python and PyYAML: tested without ROS.
"""
import json
import math
import os
import re
import shutil
import threading
import time

import yaml

_LOCK = threading.Lock()
_NAME = re.compile(r"^[A-Za-z_][A-Za-z0-9_]*$")


def node_key(node):
    """The YAML section of a ROS node name: '/ns/target_tracker' -> 'target_tracker'."""
    return node.rstrip("/").split("/")[-1]


def coerce(value, rtype=None):
    """`value` as the ROS type `rtype` says ('double', 'integer', 'bool', 'string', or one of
    those + '_array'): a double stays a double when the GCS sends 70 for 70.0."""
    if rtype is None:
        return value
    if rtype.endswith("_array"):
        if not isinstance(value, (list, tuple)) or not value:
            raise ValueError(f"{rtype} needs a non-empty list, not {value!r}")
        return [coerce(v, rtype[:-len("_array")]) for v in value]
    if rtype == "double":
        if isinstance(value, bool):
            raise ValueError(f"a double, not {value!r}")
        return float(value)
    if rtype == "integer":
        if isinstance(value, bool) or float(value) != int(float(value)):
            raise ValueError(f"an integer, not {value!r}")
        return int(float(value))
    if rtype == "bool":
        if not isinstance(value, bool):
            raise ValueError(f"true or false, not {value!r}")
        return value
    if rtype == "string":
        return str(value)
    raise ValueError(f"{rtype} parameters cannot be saved")


def fmt(value):
    """YAML text for one parameter value: a double always has a decimal point (1e-05 would be a
    string to YAML 1.1), a string is double-quoted, a list is a flow list."""
    if isinstance(value, bool):
        return "true" if value else "false"
    if isinstance(value, int):
        return str(value)
    if isinstance(value, float):
        if not math.isfinite(value):
            raise ValueError(f"{value!r} cannot be saved")
        r = repr(value)
        if "e" in r and "." not in r.split("e")[0]:
            m, e = r.split("e")
            r = f"{m}.0e{e}"
        return r
    if isinstance(value, str):
        return json.dumps(value)
    if isinstance(value, (list, tuple)):
        if not value:
            raise ValueError("an empty list has no type: it cannot be saved")
        return "[" + ", ".join(fmt(v) for v in value) + "]"
    raise ValueError(f"{value!r} cannot be saved")


def _split_comment(rest):
    """(value text with its trailing spaces, '# comment' or '') for what follows 'name:'."""
    quote = None
    for i, ch in enumerate(rest):
        if quote:
            if ch == quote:
                quote = None
        elif ch in "\"'":
            quote = ch
        elif ch == "#" and (i == 0 or rest[i - 1] in " \t"):
            return rest[:i], rest[i:]
    return rest, ""


def _indent(line):
    return len(line) - len(line.lstrip(" "))


def _typed(v):
    """`v` with its type attached, so 1 is not 1.0 and true is not 1."""
    if isinstance(v, (list, tuple)):
        return ("list", tuple(_typed(x) for x in v))
    return (type(v).__name__, v)


def _same(text, value):
    """The YAML value `text` already reads as `value`: its line is left as it is written."""
    try:
        return _typed(yaml.safe_load(text)) == _typed(value)
    except yaml.YAMLError:
        return False


def set_values(text, node, values):
    """`text` with `values` ({name: value}) written into section `node` (its YAML key). Returns
    (new text, [(name, 'changed' | 'added' | 'unchanged')]). ValueError when the section or its
    ros__parameters is missing, a name is not a plain one, or a value is written over several
    lines (block style: edit it by hand)."""
    lines = text.split("\n")
    head = re.compile(rf"^{re.escape(node)}:\s*(#.*)?$")
    start = next((i for i, ln in enumerate(lines) if head.match(ln)), None)
    if start is None:
        raise ValueError(f"the YAML has no {node}: section")
    end = next((i for i in range(start + 1, len(lines))
                if lines[i].strip() and not lines[i][0].isspace() and not lines[i].startswith("#")),
               len(lines))
    rp = next((i for i in range(start + 1, end)
               if re.match(r"^\s+ros__parameters:\s*(#.*)?$", lines[i])), None)
    if rp is None:
        raise ValueError(f"{node}: has no ros__parameters:")
    depth = None
    found, last = {}, rp
    for i in range(rp + 1, end):
        s = lines[i].strip()
        if not s or s.startswith("#"):
            continue
        if _indent(lines[i]) <= _indent(lines[rp]):
            break
        depth = depth if depth is not None else _indent(lines[i])
        if _indent(lines[i]) == depth:
            m = re.match(r"^\s+([^:#\s]+):", lines[i])
            if m:
                found.setdefault(m.group(1), i)
        last = i
    depth = depth if depth is not None else _indent(lines[rp]) + 2
    done = []
    for name, value in values.items():
        if not _NAME.match(name):
            raise ValueError(f"{name}: not a plain parameter name (nested: edit the YAML by hand)")
        text_v = fmt(value)
        i = found.get(name)
        if i is None:
            cr = "\r" if lines[last].endswith("\r") else ""      # keep a CRLF file CRLF
            lines.insert(last + 1, " " * depth + f"{name}: {text_v}" + cr)
            last += 1
            found = {k: (v + 1 if v >= last else v) for k, v in found.items()}
            found[name] = last
            done.append((name, "added"))
            continue
        cr = "\r" if lines[i].endswith("\r") else ""
        m = re.match(r"^(\s+[^:#\s]+:)(\s*)(.*)$", lines[i].rstrip("\r"))
        lead, gap, rest = m.groups()
        val, comment = _split_comment(rest)
        if not val.strip():
            raise ValueError(f"{name}: its value is written over several lines: edit the YAML by hand")
        if _same(val, value):
            done.append((name, "unchanged"))
            continue
        new_val = text_v.ljust(max(len(text_v) + 1, len(val))) if comment else text_v
        lines[i] = lead + (gap or " ") + new_val + comment + cr
        done.append((name, "changed"))
    return "\n".join(lines), done


def _check(old_text, new_text, node, values):
    """The new text parses, holds `values` for `node`, and differs from the old in nothing else."""
    old, new = yaml.safe_load(old_text) or {}, yaml.safe_load(new_text) or {}
    got = ((new.get(node) or {}).get("ros__parameters") or {})
    for name, value in values.items():
        if got.get(name) != value:
            raise ValueError(f"{name}: the written YAML reads {got.get(name)!r}, not {value!r}")
    want = dict(old)
    sec = dict(want.get(node) or {})
    sec["ros__parameters"] = dict(sec.get("ros__parameters") or {}, **values)
    want[node] = sec
    if want != new:
        raise ValueError("the write would change more than the saved values: not saved")


def backup_dir_for(path):
    """Where the old copies of `path` go: <workspace>/backups/params for a YAML under
    <workspace>/src/ (on the boat ~/robotx_ws/backups/params, outside the git checkout)."""
    head, sep, _ = path.partition(f"{os.sep}src{os.sep}")
    return (os.path.join(head, "backups", "params") if sep
            else os.path.join(os.path.dirname(path), "params_backups"))


def _own(path, st):
    """Give `path` the owner of the file `st` describes: gcs_bridge runs as root, the
    checkout belongs to crusader, and a root-owned file there blocks his next git pull."""
    try:
        os.chown(path, st.st_uid, st.st_gid)
    except (AttributeError, OSError):          # Windows; or not root, which writes as itself
        pass


def _backup(path, backup_dir, st, now):
    made, d = [], backup_dir
    while d and not os.path.isdir(d):
        made.append(d)
        d = os.path.dirname(d)
    for d in reversed(made):
        os.mkdir(d)
        _own(d, st)
    stamp = time.strftime("%Y%m%d-%H%M%S", time.localtime(now))
    dst = os.path.join(backup_dir, f"{os.path.basename(path)}.{stamp}")
    shutil.copy2(path, dst)
    _own(dst, st)
    return dst


def save(params_file, node, values, types=None, backup_dir=None, now=None):
    """Write `values` ({name: value}) for ROS node `node` into the YAML behind `params_file` (a
    symlink is followed to the file it names). `types` ({name: ROS type name}) keeps each value
    its type. The old file is copied to `backup_dir` first (None: backup_dir_for the file;
    False: no copy). Returns (path written, [(name, 'changed' | 'added' | 'unchanged')],
    the old copy's path or None when nothing was written)."""
    path = os.path.realpath(params_file)
    if any(f"{os.sep}{d}{os.sep}" in path for d in ("install", "build")):
        raise ValueError(f"{params_file} is a built copy, not a link to the source YAML: "
                         "a save there would be lost at the next rebuild")
    key = node_key(node)
    vals = {k: coerce(v, (types or {}).get(k)) for k, v in values.items()}
    with _LOCK:
        with open(path, encoding="utf-8", newline="") as f:
            old = f.read()
        new, done = set_values(old, key, vals)
        if new == old:
            return path, done, None
        _check(old, new, key, vals)
        st = os.stat(path)
        backup = None
        if backup_dir is not False:
            backup = _backup(path, backup_dir or backup_dir_for(path), st, now)
        tmp = f"{path}.gcs-save"
        try:
            with open(tmp, "w", encoding="utf-8", newline="") as f:
                f.write(new)
            os.chmod(tmp, st.st_mode & 0o7777)
            _own(tmp, st)
            os.replace(tmp, path)
        finally:
            if os.path.exists(tmp):
                os.remove(tmp)
    return path, done, backup
