"""param_save — the Tuning tab's Save: live values back into crusader_params.yaml.

A value tuned on the water used to die with the node, and the drift marker was
a list of lines for somebody to type into the file later. Save types them.

THE FILE IS EDITED, NOT REWRITTEN. Its comments are most of its value, and a
yaml.dump of the parsed file would drop every one of them. So a save touches
only the value on that parameter's own line, keeping the line's comment where
it was, and a parameter the file does not have yet (bt_runner_node's strafe.*,
declared in code) gets one new line at the top of its node's section. The
result is parsed again and must read back the values just written, or nothing
is written at all.

IN PLACE, NOT RENAMED. The installed file is a link into the boat's git
checkout and is owned by the host's user; writing a temporary and renaming it
over the original would leave a root-owned file that git on the host cannot
touch. The previous contents are kept as a timestamped backup first.

Pure text in, text out, apart from save(): testable off-boat.
"""
import json
import math
import os
import re
import time

import yaml


def format_value(v):
    """A parameter value as YAML. Floats keep a decimal point, because the
    nodes declare doubles and `150` would be read back as an integer that a
    double parameter refuses."""
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    if isinstance(v, float):
        if not math.isfinite(v):
            raise ValueError(f"cannot save {v!r}")
        s = repr(v)
        return s if any(c in s for c in ".eE") else s + ".0"
    if isinstance(v, str):
        return json.dumps(v)
    raise ValueError(f"cannot save a {type(v).__name__}: {v!r}")


def set_in_section(text, section, key, value):
    """`text` with `key: value` set inside `section`'s ros__parameters.

    Returns (new_text, "changed" | "added" | "same"). Raises KeyError when the
    file has no such section. Only lines at the section's parameter indent
    count, so a commented-out line or a deeper nested key is never edited."""
    lines = text.split("\n")
    start = next((i for i, ln in enumerate(lines) if re.match(re.escape(section) + r":\s*(#.*)?$", ln)),
                 None)
    if start is None:
        raise KeyError(f"no {section!r} section in the file")
    end = next((i for i in range(start + 1, len(lines)) if re.match(r"\S", lines[i])
                and not lines[i].startswith("#")), len(lines))
    rp = next((i for i in range(start + 1, end) if re.match(r"\s+ros__parameters:\s*(#.*)?$", lines[i])),
              None)
    if rp is None:
        raise KeyError(f"no ros__parameters under {section!r}")
    rp_indent = len(lines[rp]) - len(lines[rp].lstrip())
    indent = next((len(ln) - len(ln.lstrip()) for ln in lines[rp + 1:end]
                   if ln.strip() and not ln.strip().startswith("#")
                   and len(ln) - len(ln.lstrip()) > rp_indent), rp_indent + 2)
    new = format_value(value)
    pat = re.compile(r"^( {%d}" % indent + re.escape(key) + r":\s*)([^#]*?)(\s*#.*)?$")
    for i in range(rp + 1, end):
        m = pat.match(lines[i])
        if not m:
            continue
        old = m.group(2)
        if old.strip() == new:
            return text, "same"
        comment = m.group(3) or ""
        # keep the comment's column where the new value is no longer than the old
        pad = max(1, len(old) + (len(comment) - len(comment.lstrip())) - len(new))
        lines[i] = m.group(1) + new + (" " * pad + comment.lstrip() if comment else "")
        return "\n".join(lines), "changed"
    line = " " * indent + f"{key}: {new}"
    line += " " * max(1, 43 - len(line)) + f"# saved from the Tuning tab {time.strftime('%Y-%m-%d')}"
    lines.insert(rp + 1, line)
    return "\n".join(lines), "added"


def _same(a, b):
    if isinstance(a, float) or isinstance(b, float):
        try:
            return abs(float(a) - float(b)) <= 1e-9 * max(1.0, abs(float(b)))
        except (TypeError, ValueError):
            return False
    return a == b


def apply(text, section, values):
    """All of `values` into `section`, checked: the result must parse and read
    back every value. Returns (new_text, {key: "changed"|"added"|"same"})."""
    how = {}
    for k, v in values.items():
        text, how[k] = set_in_section(text, section, k, v)
    back = (yaml.safe_load(text) or {}).get(section, {}).get("ros__parameters", {})
    bad = [k for k, v in values.items() if k not in back or not _same(back[k], v)]
    if bad:
        raise ValueError(f"the edited file would not read back {', '.join(bad)}: not saved")
    return text, how


def save(path, section, values, backup_dir):
    """Write `values` into `section` of the file at `path`, after a backup of
    its current contents in `backup_dir`. Returns {key: how}."""
    with open(path, encoding="utf-8", newline="") as f:
        old = f.read()
    crlf = "\r\n" in old
    text, how = apply(old.replace("\r\n", "\n"), section, values)
    if all(h == "same" for h in how.values()):
        return how
    os.makedirs(backup_dir, exist_ok=True)
    stamp = time.strftime("%Y%m%d-%H%M%S")
    with open(os.path.join(backup_dir, f"{os.path.basename(path)}.{stamp}"), "w",
              encoding="utf-8", newline="") as f:
        f.write(old)
    with open(path, "w", encoding="utf-8", newline="") as f:
        f.write(text.replace("\n", "\r\n") if crlf else text)
    return how
