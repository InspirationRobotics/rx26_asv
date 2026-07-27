"""Level-1 parameter space — what the inner loop may touch, and nothing else.

Kinds:
  apf       -> ApfParams fields (ROS-side Level-1 tunables; kinematic + boat)
  backend   -> kinematic-backend dynamics (sim proxies for CRUISE_SPEED /
               WP_PIVOT_RATE / accel; on SITL these become the ArduPilot params)
  scenario  -> per-episode overrides (wp_radius mirrors WP_RADIUS)
  ardupilot -> real ArduRover params, SITL/boat only — validated against
               param_guard's TUNABLE list (single source of the safety fence)

Safety: validate() consults param_guard for anything ArduPilot-shaped; a
protected name (ARMING_*, SERVO*_REVERSED, ...) is REJECTED here, before any
proposal is even evaluated. The proposing LLM never gets to argue with this.
"""
import importlib.util
import sys
from dataclasses import dataclass
from pathlib import Path

_REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_REPO))

from rx26_asv.api.navigation.apf_core import ApfParams  # noqa: E402


def _load_param_guard():
    spec = importlib.util.spec_from_file_location(
        "param_guard", _REPO / "tools" / "scripts" / "param_guard.py")
    mod = importlib.util.module_from_spec(spec)
    sys.modules.setdefault("param_guard", mod)
    spec.loader.exec_module(mod)
    return mod


param_guard = _load_param_guard()


@dataclass
class ParamSpec:
    lo: float
    hi: float
    step: float          # heuristic-proposer perturbation size
    kind: str            # apf | backend | scenario | ardupilot
    default: float


SPACE = {
    # --- APF gains (mirror config/crusader_params.yaml [DYN] entries) ---
    "apf_k_rep":          ParamSpec(0.5, 20.0, 1.0, "apf", 5.0),
    "apf_influence_m":    ParamSpec(2.0, 15.0, 1.0, "apf", 6.0),
    "apf_lookahead_m":    ParamSpec(3.0, 20.0, 1.5, "apf", 8.0),
    "apf_slow_radius_m":  ParamSpec(1.0, 10.0, 1.0, "apf", 4.0),
    "apf_tangent_gain":   ParamSpec(0.1, 3.0, 0.2, "apf", 0.8),
    "apf_project_s":      ParamSpec(1.0, 15.0, 1.0, "apf", 5.0),
    # --- backend dynamics (kinematic proxies for ArduRover tunables) ---
    "cruise_speed":       ParamSpec(0.5, 4.0, 0.3, "backend", 2.0),
    "turn_rate_dps":      ParamSpec(20.0, 120.0, 10.0, "backend", 60.0),
    "accel":              ParamSpec(0.3, 3.0, 0.3, "backend", 1.0),
    # --- scenario-level ---
    "wp_radius":          ParamSpec(1.0, 4.0, 0.5, "scenario", 2.0),
}

_APF_FIELDS = {f for f in ApfParams.__dataclass_fields__}


def validate(name: str, value) -> "str | None":
    """Returns an error string, or None if the change is acceptable."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return f"{name}: value must be numeric"
    if name in SPACE:
        spec = SPACE[name]
        if not spec.lo <= value <= spec.hi:
            return f"{name}={value} outside [{spec.lo}, {spec.hi}]"
        if spec.kind == "apf" and name[len("apf_"):] not in _APF_FIELDS:
            return f"{name}: no matching ApfParams field (space drift)"
        return None
    # ArduPilot-shaped names: the param_guard fence is the authority
    if param_guard.is_protected(name):
        return f"{name}: PROTECTED ArduRover param — never tunable"
    if param_guard.is_tunable(name):
        return (f"{name}: ArduRover tunable — valid on SITL/boat episodes only "
                "(not in the kinematic space)")
    return f"{name}: unknown parameter"


def defaults() -> dict:
    return {name: spec.default for name, spec in SPACE.items()}


def apf_kwargs(config: dict) -> dict:
    return {k[len("apf_"):]: v for k, v in config.items() if k.startswith("apf_")}


def backend_kwargs(config: dict) -> dict:
    return {k: v for k, v in config.items()
            if k in SPACE and SPACE[k].kind == "backend"}
