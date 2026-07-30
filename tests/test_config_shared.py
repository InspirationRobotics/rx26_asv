"""Phase 3.5 single-source-of-truth guards: if any of these fail, two parts of
the system have drifted apart on values that MUST be identical."""
import dataclasses

from rx26_asv.api.common import config as crsd_config
from rx26_asv.api.common.param_utils import check_range
from rx26_asv.api.navigation.apf_core import ApfParams
from rx26_asv.api.navigation.progress_monitor import ProgressMonitor
from rx26_asv.api.perception.lidar_fusion import FusionParams, params_from


def test_all_nodes_have_config_sections():
    for node in ("telemetry_bridge", "perception_node",
                 "occupancy_grid_node", "roa_apf_node", "mission_planner_node",
                 # operational stack + LiDAR fusion (must not silently drift out
                 # of the config again — every config-driven node belongs here)
                 "rc_heartbeat_watchdog", "actuator_node", "ivc_node",
                 "lidar_fusion_node"):
        params = crsd_config.node_params(node)
        assert params, node


def test_params_file_parses_under_rcl_rules():
    """rcl_yaml_param_parser is stricter than PyYAML, and it fails CLOSED: an
    illegal params file kills every node in the launch at rclpy.init(), before
    any node code runs. Both rules below were violated on 2026-07-29 and
    grounded the whole stack (LEDs included) with "Couldn't parse params file".

    Rule 1: every top-level key is a node name whose only child is
            `ros__parameters` (a bare top-level scalar -> "Cannot have a value
            before ros__parameters").
    Rule 2: no YAML anchors/aliases anywhere — rcl parses tokens, not documents,
            and rejects an alias outright.

    PyYAML accepts both mistakes happily, which is exactly why this test exists:
    every other test in this file loads the YAML through PyYAML and would stay
    green while the boat could not launch a single node.
    """
    cfg = crsd_config.load()
    for name, section in cfg.items():
        assert isinstance(section, dict), f"{name}: top-level value is not a map"
        assert list(section) == ["ros__parameters"], (
            f"{name}: sole child must be `ros__parameters`, got {list(section)}")

    # anchors/aliases: check the raw text, since PyYAML resolves them away
    for lineno, line in enumerate(
            crsd_config.DEFAULT_CONFIG_PATH.read_text().splitlines(), 1):
        code = line.split("#", 1)[0]
        assert "&" not in code and "*" not in code, (
            f"line {lineno}: YAML anchor/alias — rcl rejects these, write the "
            f"value out literally and pin it in this file's shared-value tests")


def test_occupied_threshold_matches_shared():
    # grid publisher and APF consumer must agree on what "occupied" means
    want = crsd_config.shared_params()["occupied_threshold"]
    for node in ("occupancy_grid_node", "roa_apf_node"):
        assert crsd_config.node_params(node)["occupied_threshold"] == want, node


def test_pose_timeout_consumers_match_shared():
    """A consumer that trusts a pose longer than telemetry_bridge vouches for it
    is the frozen-pose failure this value exists to prevent."""
    want = crsd_config.shared_params()["pose_timeout_s"]
    for node in ("occupancy_grid_node", "roa_apf_node", "dp_hold",
                 "gate_navigator"):
        assert crsd_config.node_params(node)["pose_timeout_s"] == want, node
    bridge = crsd_config.node_params("telemetry_bridge")
    assert bridge["stream_timeout_s"] == want


def test_monitor_thresholds_match_shared_section():
    shared = crsd_config.shared_params()
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


def test_fusion_params_construct_from_config():
    """lidar_fusion_node's gates must come from the YAML, not from code defaults.

    params_from() raises KeyError on anything missing, so this pins config ->
    core. `declare_from_config` already rejects a YAML key with no declared
    posture at node start, but that only fires on the boat — this fires in CI.
    """
    p = params_from(crsd_config.node_params("lidar_fusion_node"))
    assert p.bearing_gate_rad > 0 and p.min_points >= 1


def test_fusion_config_keys_exactly_cover_the_core():
    """Bidirectional drift guard: every FusionParams field is configurable, and
    the section carries no gate key the core silently ignores. The one rename is
    the deg/rad conversion params_from() owns.

    This is the objective-1 surface — a range_gate_* that exists in the core but
    not in the YAML would quietly run on a code default nobody reviewed.
    """
    core = {f.name for f in dataclasses.fields(FusionParams)}
    core = {k[:-4] + "_deg" if k.endswith("_rad") else k for k in core}

    section = set(crsd_config.node_params("lidar_fusion_node"))
    extrinsic = {k for k in section if k.startswith("lidar_")}   # LiDAR pose in BODY
    link = {"cloud_timeout_s"}                                   # node-level, not geometry

    assert section - extrinsic - link == core


# NOTE: test_evaluator_reads_same_thresholds_as_node lives on the
# `sim/orchestrator` branch with the evaluator it guards (docs/PARKED_SIM.md).
# The YAML side of that contract is still pinned by the monitor tests above.


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
