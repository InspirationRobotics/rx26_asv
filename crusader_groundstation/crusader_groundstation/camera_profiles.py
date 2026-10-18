"""camera_profiles — named OAK-D camera settings, in two layers.

WHY TWO FILES. camera_profiles.yaml is hand-written and carries the reasoning
for every number in it, and PyYAML's dumper emits values, not comments, so a
browser Save would replace a documented file with a bare mapping. The stock file
is READ ONLY here; everything the operator saves goes to
camera_profiles.local.yaml beside it.

HOW THEY LAYER. Local wins by name. Saving over a stock name makes an override
and leaves the stock block untouched, so `delete` on an override is "revert to
stock". Deleting a stock profile is refused: the file is in git and the UI would
promise something it cannot do.

WHERE THE LOCAL FILE GOES. Beside the SOURCE config, not the installed one:
crusader_params.yaml is symlinked into the install space but a new file is
COPIED there by the build, so a profile written to the install copy vanishes at
the next colcon build. `status()` reports the path it settled on.

THIS MODULE DOES NOT VALIDATE CAMERA VALUES, deliberately. oak_detector owns
every range and refuses in its own words through /params/set; a second copy of
the bounds here would go stale. A profile is stored as given and found
unacceptable by trying to apply it.
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

    ENV_OVERRIDE names a DIRECTORY, not a file, so the two layers stay together
    (pointing at one and inferring the other mixes repo stock with test-fixture
    saved profiles).
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

    A missing local file is the normal state on a fresh checkout. A malformed one
    is not worth taking the tab down for: the stock layer still loads and
    `status()` carries the reason.
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

    A crash or full disk partway through an in-place write would leave a file
    that parses as nothing and lose every saved profile. Written to a temp name
    in the SAME directory so os.replace is a rename, not a cross-device copy.

    safe_dump is what quotes 'off' (verified): YAML 1.1 resolves a bare `off` as
    the boolean False, and awb_mode: off is a real value here. Do not replace it
    with a dumper that emits unquoted scalars.
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
    # The name becomes a YAML key and a button label; restrict it so "a: b" is
    # not parsed as something else on reload.
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
    # Probed, not inferred from the path: a root-owned share dir and a read-only
    # source mount look identical from here.
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
