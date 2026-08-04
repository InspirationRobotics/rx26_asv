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


def test_slow_status_streams_get_a_looser_timeout_than_pose():
    """HEARTBEAT is a FIXED 1 Hz — no SR* parameter raises it — and SYS_STATUS
    runs at SR*_EXT_STAT (2 Hz on Crusader). Judging either against the 1.0 s
    pose/RC timeout marks it stale on jitter alone, which flaps /crsd/fcu_status
    in and out of republication and leaves consumers frozen on a cached value.
    Two clear HEARTBEAT periods is the floor."""
    bridge = crsd_config.node_params("telemetry_bridge")
    assert bridge["status_timeout_s"] >= 2.0, \
        "status_timeout_s must clear at least two 1 Hz HEARTBEAT periods"
    assert bridge["status_timeout_s"] > bridge["stream_timeout_s"], \
        "the slow status streams need a looser timeout than pose/RC, not tighter"


def test_led_input_timeout_outlives_the_bridge_status_timeout():
    """The LED must not call an input stale before telemetry_bridge would have
    stopped vouching for it, or it fails RED on streams the bridge still
    considers healthy — a red strip on a working boat teaches the crew to
    ignore the strip."""
    led = crsd_config.node_params("pixhawk_led_status_node")
    bridge = crsd_config.node_params("telemetry_bridge")
    assert led["input_timeout_s"] >= bridge["stream_timeout_s"], \
        "LED input_timeout_s is tighter than the bridge's fast-stream timeout"
    assert led["input_timeout_s"] > 1.0, \
        "input_timeout_s must clear normal 20 Hz republish jitter"


def test_drop_channel_is_not_the_estop_channel():
    """ch7 carries RC7_OPTION=165 (arm/e-stop). Its NORMAL ARMED position (1995)
    is above drop_threshold, so pointing the autonomy-drop latch at ch7 leaves it
    permanently tripped: every RC override and every GUIDED setpoint is dropped
    at the bridge exactly when the boat is armed and expected to move. dp_hold
    and gate_navigator could not actuate at all, and nothing in the logs says
    "your drop channel is your arm channel" — it just looks like the nodes do
    nothing. Found in the 2026-08-02 session review."""
    bridge = crsd_config.node_params("telemetry_bridge")
    led = crsd_config.node_params("pixhawk_led_status_node")
    assert bridge["drop_channel"] != led["estop_channel"], (
        f"drop_channel and estop_channel are both ch{bridge['drop_channel']} — "
        "the switch position that clears the e-stop also trips the drop latch")


def test_drop_channel_cannot_be_written_by_an_override():
    """telemetry_bridge's _send_override truncates to 8 channels, so a drop
    channel >= 9 is unreachable by ANY override the graph can emit. Below 9, a
    mechanism could drive the latch's own input — a feedback path where the
    thing being stopped controls the stop signal."""
    bridge = crsd_config.node_params("telemetry_bridge")
    assert bridge["drop_channel"] >= 9, (
        "drop_channel must sit outside the 1-8 window RC_CHANNELS_OVERRIDE can "
        "write, so no override mechanism can drive the latch's input")


def test_latch_health_window_matches_the_bridge_status_timeout():
    """The latch trusts a SYS_STATUS RC-receiver verdict for health_timeout, and
    telemetry_bridge constructs it from status_timeout_s. SYS_STATUS is a SLOW
    stream (SR*_EXT_STAT, 2 Hz); judging it against the 1 s RC window would
    discard a usable safety signal on jitter alone."""
    bridge = crsd_config.node_params("telemetry_bridge")
    assert bridge["status_timeout_s"] > bridge["rc_stale_timeout"], (
        "the SYS_STATUS health window must be looser than the RC window, or the "
        "verdict expires before the next SYS_STATUS arrives")


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
