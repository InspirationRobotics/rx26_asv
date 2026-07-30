"""Shared config loader — config/crusader_params.yaml is the single source of
truth for ROS-side parameters (Phase 3.5).

Consumed by:
  * node code, for declaration defaults (so code defaults can't drift from the
    file the launch system loads);
  * the episode evaluator, for objective-2 thresholds (so scoring uses the same
    numbers as the onboard at-risk detector);
  * metrics.assemble, which records the file's sha256 as ros_config_hash.

Nodes still receive their live values through normal ROS parameter machinery —
this loader only supplies defaults and non-ROS consumers.
"""
import hashlib
from pathlib import Path

# Where this file sits relative to the config differs between the two layouts,
# and NO fixed number of `parents` satisfies both:
#
#   source tree   <repo>/rx26_asv/rx26_asv/api/common/config.py
#                 <repo>/rx26_asv/config/crusader_params.yaml        -> parents[3]
#
#   installed     install/rx26_asv/lib/python3.10/site-packages/rx26_asv/api/common/config.py
#                 install/rx26_asv/share/rx26_asv/config/crusader_params.yaml
#
# lib/ and share/ are siblings, so path arithmetic from the module cannot reach
# the installed config at all — parents[3] lands in site-packages/ and every
# node dies at __init__ with FileNotFoundError. Ask ament where the package's
# share dir is, and keep the relative path only for the source tree (unit tests,
# and any non-ROS consumer like the episode evaluator).
_SOURCE_CONFIG_PATH = (Path(__file__).resolve().parents[3]
                       / "config" / "crusader_params.yaml")


def _resolve_config_path(get_share_dir=None) -> Path:
    """Installed share dir if available, else the source-tree path.

    `get_share_dir` is injectable so the resolution order is testable off-boat,
    where ament_index_python is not installed.
    """
    if get_share_dir is None:
        try:
            from ament_index_python.packages import (
                get_package_share_directory as get_share_dir)
        except ImportError:
            return _SOURCE_CONFIG_PATH        # no ROS here: source tree it is
    try:
        # PackageNotFoundError when the workspace is not sourced; fall through
        # rather than fail, so `python -m pytest` in a container still works.
        p = Path(get_share_dir("rx26_asv")) / "config" / "crusader_params.yaml"
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
            with open(p) as f:
                _cache[key] = yaml.safe_load(f)
        except FileNotFoundError as e:
            # The bare errno message names one path and gives no hint which
            # layout was assumed — that cost a boat-side debugging session.
            raise FileNotFoundError(
                f"crusader_params.yaml not found at {p}.\n"
                f"  installed layout: <install>/share/rx26_asv/config/ "
                f"(via ament; is the workspace sourced?)\n"
                f"  source layout:    {_SOURCE_CONFIG_PATH}\n"
                f"If running from the install space, check setup.py still "
                f"installs config/ into share/ and that colcon build succeeded."
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
    section by tests/test_config_shared.py. The section carries a
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


def monitor_kwargs(path=None) -> dict:
    """ProgressMonitor constructor kwargs — the shared objective-2 thresholds."""
    p = node_params("roa_apf_node", path)
    return {
        "window_s": p["monitor_window_s"],
        "speed_floor": p["monitor_speed_floor"],
        "progress_floor": p["monitor_progress_floor"],
        "heading_osc_floor": p["monitor_heading_osc_floor"],
        "stall_after_s": p["monitor_stall_after_s"],
    }


def apf_kwargs(path=None) -> dict:
    """ApfParams constructor kwargs from the roa_apf_node section."""
    p = node_params("roa_apf_node", path)
    return {k[len("apf_"):]: v for k, v in p.items() if k.startswith("apf_")}
