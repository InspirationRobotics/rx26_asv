"""camera_profiles — named OAK-D camera settings, in two layers.

WHY TWO FILES. camera_profiles.yaml is hand-written and carries the reasoning
for every number in it. A machine write destroys that: PyYAML's dumper emits
values, not comments, so the first Save from the browser would silently replace
a documented file with a bare mapping. So the stock file is READ ONLY here, and
everything the operator saves goes to camera_profiles.local.yaml beside it,
which no human is expected to read. Documentation and state stop fighting over
one file, and a profile saved on the water can never eat the explanation of why
the shipped one looks the way it does.

HOW THEY LAYER. Local wins by name. Saving over a stock name makes an override
and leaves the stock block untouched underneath, so `delete` on an override is
"revert to stock" rather than a loss — which is the operation you actually want
at 3pm when the light has moved and the profile you just saved is worse than the
one you started from. Deleting a stock profile is refused, because the file it
lives in is in git and the UI would be promising something it cannot do.

WHERE THE LOCAL FILE GOES. Beside the SOURCE config, not the installed one.
crusader_params.yaml is symlinked into the install space, but a new file is
COPIED there by the build — so a profile written to the install copy survives
until the next colcon build and then vanishes, which is the worst failure shape
available: it works all afternoon and is gone tomorrow. Resolution prefers the
source tree for exactly that reason, and `status()` reports the path it settled
on so the tab can show it rather than leaving the operator to guess.

THIS MODULE DOES NOT VALIDATE CAMERA VALUES, deliberately. oak_detector owns
every range and refuses in its own words through /params/set, and a second copy
of the bounds here is a copy that goes stale the first time oak_controls grows a
knob. A profile is stored as whatever it was given; what is unacceptable is
found out by trying to apply it, and the node's own sentence is what the
operator reads.
"""
import io
import os
from pathlib import Path

import yaml

STOCK_FILENAME = "camera_profiles.yaml"
LOCAL_FILENAME = "camera_profiles.local.yaml"
BRINGUP_PACKAGE = "crusader_bringup"
ENV_OVERRIDE = "CRUSADER_CAMERA_PROFILES"

# <repo>/crusader_groundstation/crusader_groundstation/camera_profiles.py -> <repo>
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SOURCE_DIR = _REPO_ROOT / BRINGUP_PACKAGE / "config"

STOCK = "stock"          # shipped in camera_profiles.yaml, in git
SAVED = "saved"          # only in the local file
OVERRIDE = "override"    # a local block shadowing a stock name of the same name


def _config_dir():
    """The directory both files live in.

    ENV_OVERRIDE names a DIRECTORY, not a file, because the two layers have to
    stay together — pointing at one of them and inferring the other is how you
    get stock profiles from the repo and saved ones from a test fixture.
    """
    override = os.environ.get(ENV_OVERRIDE)
    if override:
        return Path(override)
    if _SOURCE_DIR.is_dir():
        return _SOURCE_DIR
    # No source tree: an installed-only deployment. Reading works; the first
    # save will report the directory as unwritable, which is the truth.
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(BRINGUP_PACKAGE)) / "config"
    except Exception:
        return _SOURCE_DIR


def stock_path():
    return _config_dir() / STOCK_FILENAME


def local_path():
    return _config_dir() / LOCAL_FILENAME


def _read(path):
    """One file's `profiles:` mapping, or {} when it is absent or unusable.

    A missing local file is the normal state on a fresh checkout, not an error.
    A malformed one is an error, but not one worth taking the tab down for: the
    stock layer still loads, and `status()` carries the reason so the operator
    is told which file to look at instead of being shown a short list that looks
    complete.
    """
    try:
        raw = yaml.safe_load(io.open(path, encoding="utf-8")) or {}
    except FileNotFoundError:
        return {}, ""
    except Exception as e:
        return {}, f"{path.name}: {e}"
    profiles = raw.get("profiles")
    if not isinstance(profiles, dict):
        return {}, f"{path.name}: no 'profiles:' mapping"
    return {k: v for k, v in profiles.items() if isinstance(v, dict)}, ""


def listing():
    """Every profile, merged, as [{name, values, origin}] sorted by name."""
    stock, _ = _read(stock_path())
    local, _ = _read(local_path())
    rows = []
    for name in sorted(set(stock) | set(local)):
        if name in local:
            origin = OVERRIDE if name in stock else SAVED
            values = local[name]
        else:
            origin, values = STOCK, stock[name]
        rows.append({"name": name, "values": values, "origin": origin})
    return rows


def get(name):
    for row in listing():
        if row["name"] == name:
            return row
    return None


def _write_local(profiles):
    """Replace the local file atomically.

    Atomic because the alternative is a truncated YAML file: a crash or a full
    disk partway through an in-place write leaves a file that parses as nothing,
    and every saved profile is gone at once. Written to a temp name in the SAME
    directory so os.replace is a rename rather than a cross-device copy.

    safe_dump is what quotes 'off' — verified, not assumed, because YAML 1.1
    resolves a bare `off` back as the boolean False, and awb_mode: off is a real
    value here. Do not replace this with a dumper that emits unquoted scalars.
    """
    path = local_path()
    body = yaml.safe_dump({"profiles": profiles}, default_flow_style=False,
                          sort_keys=True, allow_unicode=True)
    header = ("# camera_profiles.local.yaml — WRITTEN BY THE GROUND STATION.\n"
              "# Operator-saved camera profiles. Edit camera_profiles.yaml for\n"
              "# anything that wants a comment; this file is regenerated whole\n"
              "# on every save and will not keep yours.\n")
    tmp = path.with_suffix(path.suffix + ".tmp")
    with io.open(tmp, "w", encoding="utf-8", newline="\n") as fh:
        fh.write(header)
        fh.write(body)
    os.replace(tmp, path)


def save(name, values):
    """Store `values` under `name` in the local layer. Returns (ok, message)."""
    name = (name or "").strip()
    if not name:
        return False, "a profile needs a name"
    # The name becomes a YAML key and a button label. Restricting it here keeps
    # both honest rather than discovering on reload that "a: b" was parsed as
    # something else entirely.
    if not all(c.isalnum() or c in "_-" for c in name):
        return False, f"{name!r}: use letters, digits, underscore or hyphen"
    if not isinstance(values, dict) or not values:
        return False, "no values to save"

    local, err = _read(local_path())
    if err:
        return False, f"refusing to overwrite an unreadable file — {err}"
    existed = name in local
    local[name] = values
    try:
        _write_local(local)
    except OSError as e:
        return False, f"cannot write {local_path()}: {e}"
    stock, _ = _read(stock_path())
    if name in stock:
        return True, f"saved {name} — overrides the stock profile"
    return True, f"{'updated' if existed else 'saved'} {name}"


def delete(name):
    """Remove a local profile. Returns (ok, message)."""
    local, err = _read(local_path())
    if err:
        return False, f"refusing to rewrite an unreadable file — {err}"
    if name not in local:
        stock, _ = _read(stock_path())
        if name in stock:
            return False, (f"{name} is a stock profile in {STOCK_FILENAME} — "
                           "edit that file in git, it is not deletable here")
        return False, f"no profile named {name}"
    del local[name]
    try:
        _write_local(local)
    except OSError as e:
        return False, f"cannot write {local_path()}: {e}"
    stock, _ = _read(stock_path())
    if name in stock:
        return True, f"reverted {name} to the stock profile"
    return True, f"deleted {name}"


def status():
    """Where the two layers are, whether saving can work, and any read error."""
    stock, stock_err = _read(stock_path())
    local, local_err = _read(local_path())
    directory = _config_dir()
    # Probed rather than inferred from the path: an installed share dir owned by
    # root and a source tree on a read-only mount look identical from here, and
    # the difference only shows up as a failed save after a tuning session.
    writable = os.access(directory, os.W_OK) and (
        not local_path().exists() or os.access(local_path(), os.W_OK))
    return {
        "stock_path": str(stock_path()),
        "local_path": str(local_path()),
        "writable": bool(writable),
        "n_stock": len(stock),
        "n_local": len(local),
        "error": "; ".join(e for e in (stock_err, local_err) if e),
    }
