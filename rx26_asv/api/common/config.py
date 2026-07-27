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

DEFAULT_CONFIG_PATH = (Path(__file__).resolve().parents[3]
                       / "config" / "crusader_params.yaml")

_cache = {}


def load(path=None) -> dict:
    """Parse the config file (cached per path). Raises on missing file or bad
    YAML — a node silently running on code defaults is config drift."""
    p = Path(path or DEFAULT_CONFIG_PATH)
    key = str(p)
    if key not in _cache:
        import yaml
        with open(p) as f:
            _cache[key] = yaml.safe_load(f)
    return _cache[key]


def config_hash(path=None):
    p = Path(path or DEFAULT_CONFIG_PATH)
    try:
        return hashlib.sha256(p.read_bytes()).hexdigest()[:12]
    except OSError:
        return None


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


def por_usv_kwargs(path=None) -> dict:
    """PorUsvScorer constructor kwargs from the por_usv section.

    Proof of Readiness (Handbook 3.1.2) acceptance thresholds. Scored
    off-board by orchestrator/evaluator/por_usv.py; they live in the config
    file so the criteria cannot drift from the run the boat is flown against
    (Phase 3.5 single-source rule), and so the config sha256 recorded in every
    episode metrics JSON covers them too.

    Args:
        path: optional config path override (tests).

    Returns:
        dict: kwargs accepted by evaluator.por_usv.PorUsvScorer.
    """
    p = node_params("por_usv", path)
    return {
        "start_distance_m": p["start_distance_m"],
        "start_tolerance_m": p["start_tolerance_m"],
        "max_video_s": p["max_video_s"],
        "video_warn_s": p["video_warn_s"],
        "gate_centre_tolerance": p["gate_centre_tolerance"],
        "contact_margin_m": p["contact_margin_m"],
    }


def apf_kwargs(path=None) -> dict:
    """ApfParams constructor kwargs from the roa_apf_node section."""
    p = node_params("roa_apf_node", path)
    return {k[len("apf_"):]: v for k, v in p.items() if k.startswith("apf_")}
