"""Fixed scenario-suite runner for autoresearch (Level 1 evaluates against THIS,
never a cherry-picked subset — CLAUDE.md).

Per-episode bookkeeping the gates require:
  * active-mechanism assertion: the advisor the mechanism built must report the
    mechanism's name; a mismatch raises (silent fallback = safety issue).
  * thread audit: threading.active_count() before/after every episode — any
    delta is recorded and fails the run (deterministic-teardown rule).
"""
import threading
from pathlib import Path

from episodes.apf_advisor import ApfAdvisor
from episodes.backends.kinematic import KinematicBackend
from episodes.planner_episode import PlannerEpisodeRunner
from episodes.runner import EpisodeRunner
from episodes.scenario import Scenario
from evaluator import metrics

from rx26_asv.api.navigation.apf_core import ApfParams

from level1 import param_space

SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"
DEFAULT_SUITE = ["mission1_transit.json", "mission1_obstacle_field.json",
                 "mission4_core.json"]


class MechanismAssertionError(RuntimeError):
    """The advisor running is not the mechanism we asked for — never count the
    episode as evaluated (known failure mode: silent fallback)."""


class BaselineMechanism:
    """Known-good default: the stock APF advisor."""
    name = "baseline_apf"

    def make_advisor(self, scenario, apf_params: ApfParams):
        advisor = ApfAdvisor(scenario, params=apf_params)
        advisor.mechanism_name = self.name
        return advisor


def _needs_planner(scenario: Scenario) -> bool:
    return any(e.type in ("assistance_request", "clearance")
               for e in scenario.events)


def run_episode(scenario, config: dict, seed: int, mechanism, noise=0.02):
    backend = KinematicBackend(noise_std=noise,
                               **param_space.backend_kwargs(config))
    apf = ApfParams(**param_space.apf_kwargs(config))
    advisor = mechanism.make_advisor(scenario, apf)
    if getattr(advisor, "mechanism_name", None) != mechanism.name:
        raise MechanismAssertionError(
            f"requested mechanism {mechanism.name!r} but advisor reports "
            f"{getattr(advisor, 'mechanism_name', None)!r}")
    if _needs_planner(scenario):
        result, planner = PlannerEpisodeRunner(
            scenario, backend, seed=seed, advisor=advisor).run()
        return metrics.assemble(result, scenario, planner=planner)
    result = EpisodeRunner(scenario, backend, seed=seed, advisor=advisor).run()
    return metrics.assemble(result, scenario)


def run_suite(config: dict, seeds=(0, 1), mechanism=None, suite=None,
              noise=0.02):
    """Returns (metrics_list, episode_records). Records carry the audit trail
    the G5 gate checks: mechanism name + thread-count delta per episode."""
    mechanism = mechanism or BaselineMechanism()
    records, all_metrics = [], []
    for name in (suite or DEFAULT_SUITE):
        scenario = Scenario.load(str(SCENARIO_DIR / name))
        # apply scenario-kind overrides
        if "wp_radius" in config:
            scenario.wp_radius = config["wp_radius"]
        for seed in seeds:
            threads_before = threading.active_count()
            m = run_episode(scenario, config, seed, mechanism, noise)
            threads_after = threading.active_count()
            all_metrics.append(m)
            records.append({
                "scenario": scenario.name,
                "seed": seed,
                "mechanism": mechanism.name,
                "mechanism_asserted": True,     # run_episode raised otherwise
                "thread_delta": threads_after - threads_before,
            })
    return all_metrics, records
