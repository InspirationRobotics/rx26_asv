"""Phase 3.5 single-source-of-truth guards: if any of these fail, two parts of
the system have drifted apart on values that MUST be identical."""
import dataclasses

from robotx_2026.api.common import config as crsd_config
from robotx_2026.api.common.param_utils import check_range
from robotx_2026.api.navigation.apf_core import ApfParams
from robotx_2026.api.navigation.progress_monitor import ProgressMonitor


def test_all_nodes_have_config_sections():
    for node in ("telemetry_bridge", "perception_node",
                 "occupancy_grid_node", "roa_apf_node", "mission_planner_node"):
        params = crsd_config.node_params(node)
        assert params, node


def test_occupied_threshold_anchor_holds():
    # grid publisher and APF consumer must agree on what "occupied" means
    grid = crsd_config.node_params("occupancy_grid_node")
    apf = crsd_config.node_params("roa_apf_node")
    assert grid["occupied_threshold"] == apf["occupied_threshold"]


def test_monitor_thresholds_match_shared_section():
    cfg = crsd_config.load()
    shared = cfg["shared"]
    mk = crsd_config.monitor_kwargs()
    assert mk["window_s"] == shared["progress_window_s"]
    assert mk["speed_floor"] == shared["progress_speed_floor"]
    assert mk["progress_floor"] == shared["progress_floor"]
    assert mk["heading_osc_floor"] == shared["progress_heading_osc_floor"]
    assert mk["stall_after_s"] == shared["progress_stall_after_s"]


def test_monitor_kwargs_construct_progress_monitor():
    ProgressMonitor(**crsd_config.monitor_kwargs())     # raises on rename drift


def test_apf_kwargs_construct_apf_params():
    kwargs = crsd_config.apf_kwargs()
    fields = {f.name for f in dataclasses.fields(ApfParams)}
    assert set(kwargs) == fields, (
        "config apf_* keys must exactly match ApfParams fields")
    ApfParams(**kwargs)


def test_evaluator_reads_same_thresholds_as_node():
    import sys
    from pathlib import Path
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "orchestrator"))
    from evaluator import metrics
    assert metrics._shared_thresholds() == crsd_config.monitor_kwargs()


def test_config_hash_stable_and_present():
    h1, h2 = crsd_config.config_hash(), crsd_config.config_hash()
    assert h1 and h1 == h2 and len(h1) == 12


def test_check_range_pure_validator():
    ranges = {"k_rep": (0.1, 50.0), "ch": (1, 18)}
    assert check_range("k_rep", 5.0, ranges) is None
    assert "outside" in check_range("k_rep", 500.0, ranges)
    assert "outside" in check_range("ch", 0, ranges)
    assert check_range("unknown_param", 1e9, ranges) is None   # unranged: allowed
    assert "numeric" in check_range("k_rep", "high", ranges)
    assert "numeric" in check_range("k_rep", True, ranges)     # bool is not a number
