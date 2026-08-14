"""Shared config loader — crusader_params.yaml is the single source of truth for
ROS-side parameters.

Consumed by node code for declaration defaults, so a code default can't drift
from the file the launch system loads. Nodes still receive their live values
through normal ROS parameter machinery — this loader only supplies defaults.

WHERE THE FILE LIVES, AND WHY THIS IS NOT PATH ARITHMETIC
---------------------------------------------------------
The params file ships in `crusader_bringup`'s share dir, next to the launch files
that pass it to nodes. Resolution order, first hit wins:

  1. $CRUSADER_PARAMS                     — explicit override (bench, CI, a
                                            second boat, a one-off experiment)
  2. <crusader_bringup>/share/config/     — the installed location, via ament
  3. <repo>/crusader_bringup/config/      — the source tree, for off-boat tooling
                                            that has no ROS workspace sourced

Do NOT "simplify" this into `Path(__file__).parents[N]`. In the INSTALLED layout
this module lives in `install/crusader_common/lib/python3.10/site-packages/` while
the config lives in `install/crusader_bringup/share/` — different packages, and
lib/ and share/ are siblings. No fixed number of `parents` reaches it from here,
which is exactly the bug that killed every node at rclpy.init() on 2026-07-29.
Ask ament where the package is; keep the relative path only as a source-tree
fallback.

Note the direction: crusader_common looks up crusader_bringup by NAME at runtime,
and has no build dependency on it. That is deliberate — bringup depends on the
code packages, never the reverse. The cost is that a workspace with the code but
no bringup installed has no params, which `load()` reports loudly rather than
falling back to invented defaults.
"""
import hashlib
import os
from pathlib import Path

PARAMS_FILENAME = "crusader_params.yaml"
BRINGUP_PACKAGE = "crusader_bringup"
ENV_OVERRIDE = "CRUSADER_PARAMS"

# <repo>/crusader_common/crusader_common/config.py -> <repo>
_REPO_ROOT = Path(__file__).resolve().parents[2]
_SOURCE_CONFIG_PATH = _REPO_ROOT / BRINGUP_PACKAGE / "config" / PARAMS_FILENAME


def _resolve_config_path(get_share_dir=None) -> Path:
    """Env override, else the installed share dir, else the source tree.

    `get_share_dir` is injectable so the resolution order is testable off-boat,
    where ament_index_python is not installed.
    """
    override = os.environ.get(ENV_OVERRIDE)
    if override:
        return Path(override)

    if get_share_dir is None:
        try:
            from ament_index_python.packages import (
                get_package_share_directory as get_share_dir)
        except ImportError:
            return _SOURCE_CONFIG_PATH        # no ROS here: source tree it is
    try:
        # PackageNotFoundError when the workspace is not sourced; fall through
        # rather than fail, so off-boat tooling still works.
        p = Path(get_share_dir(BRINGUP_PACKAGE)) / "config" / PARAMS_FILENAME
    except Exception:
        return _SOURCE_CONFIG_PATH
    return p if p.is_file() else _SOURCE_CONFIG_PATH


DEFAULT_CONFIG_PATH = _resolve_config_path()

_cache = {}


def load(path=None) -> dict:
    """Parse the config file (cached per path). Raises on missing file or bad
    YAML — a node silently running on code defaults is config drift."""
    p = Path(path or DEFAULT_CONFIG_PATH)
    key = str(p)
    if key not in _cache:
        import yaml
        try:
            with open(p, encoding="utf-8") as f:
                _cache[key] = yaml.safe_load(f)
        except FileNotFoundError as e:
            # The bare errno message names one path and gives no hint which
            # layout was assumed — that cost a boat-side debugging session.
            raise FileNotFoundError(
                f"{PARAMS_FILENAME} not found at {p}.\n"
                f"  override:         ${ENV_OVERRIDE} "
                f"(currently {os.environ.get(ENV_OVERRIDE) or 'unset'})\n"
                f"  installed layout: <install>/share/{BRINGUP_PACKAGE}/config/ "
                f"(via ament; is the workspace sourced, and is "
                f"{BRINGUP_PACKAGE} built?)\n"
                f"  source layout:    {_SOURCE_CONFIG_PATH}\n"
                f"If running from the install space, check that "
                f"{BRINGUP_PACKAGE}'s CMakeLists still installs config/ into "
                f"share/ and that colcon build succeeded."
            ) from e
    return _cache[key]


def config_hash(path=None):
    p = Path(path or DEFAULT_CONFIG_PATH)
    try:
        return hashlib.sha256(p.read_bytes()).hexdigest()[:12]
    except OSError:
        return None


def shared_params(path=None) -> dict:
    """The `shared` section: canonical values for anything duplicated across
    node sections.

    Not a real node — rcl forbids YAML aliases in a params file, so values that
    must stay equal are written out literally per-node and pinned to this
    section by tools/scripts/check_config.py. The section carries a
    `ros__parameters` level only because rcl rejects a top-level scalar; no node
    is named `shared`, so nothing ever loads it.
    """
    return node_params("shared", path)


def node_params(node_name: str, path=None) -> dict:
    """Flat {param_name: default} for one node's ros__parameters section."""
    cfg = load(path)
    try:
        return dict(cfg[node_name]["ros__parameters"])
    except KeyError:
        raise KeyError(f"node {node_name!r} missing from {path or DEFAULT_CONFIG_PATH}")
