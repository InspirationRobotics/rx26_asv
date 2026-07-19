"""Level-2 validate-and-revert — the non-negotiable safety pattern (CLAUDE.md).

Every candidate mechanism goes through, in order:
  1. compile   — py_compile in a SUBPROCESS (an import-time crash can't take
                 the harness down)   [on the boat this stage is rebuild.sh]
  2. load      — contract-checked import (contract.load_mechanism)
  3. startup + smoke — one dry-run episode; the per-episode active-mechanism
                 assertion in suite.run_episode fires here if the mechanism
                 lies about what's running (silent-fallback guard)
  4. thread audit — threading.active_count() must return to its pre-startup
                 value after shutdown(); an orphaned thread is a REVERT, not a
                 warning (deterministic-teardown rule)
  5. full suite + keep rule — must not regress vs the active baseline
                 (keep_rule.evaluate; single-episode wins never promote)

ANY stage failure -> automatic revert: active.json is only written after every
stage passes, so "revert" is structural — the known-good mechanism was never
deactivated. Every stage outcome is logged; failures are LOUD.
"""
import shutil
import subprocess
import sys
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

from evaluator.keep_rule import Thresholds, aggregate, evaluate
from level1 import suite as l1_suite

from . import contract


@dataclass
class InjectionReport:
    candidate: str
    stages: dict = field(default_factory=dict)   # stage -> "ok" | error string
    promoted: bool = False
    reverted_reason: str = None
    keep_reasons: list = field(default_factory=list)
    wall_clock_s: float = 0.0

    def fail(self, stage, reason):
        self.stages[stage] = str(reason)
        self.reverted_reason = f"{stage}: {reason}"
        print(f"[LEVEL2] REVERT — {self.reverted_reason}", file=sys.stderr)
        return self


def validate_and_revert(candidate_path, baseline_agg, seeds=(0, 1),
                        thresholds: Thresholds = None,
                        promote: bool = True) -> InjectionReport:
    """baseline_agg: aggregate() of the ACTIVE mechanism on the same suite/seeds
    (caller computes it so candidate and baseline are scored identically)."""
    candidate_path = Path(candidate_path)
    report = InjectionReport(candidate=candidate_path.name)
    t0 = time.time()
    thresholds = thresholds or Thresholds(min_episodes=1)

    try:
        # 1. compile in isolation
        proc = subprocess.run(
            [sys.executable, "-m", "py_compile", str(candidate_path)],
            capture_output=True, text=True, timeout=60)
        if proc.returncode != 0:
            return report.fail("compile", proc.stderr.strip()[-400:])
        report.stages["compile"] = "ok"

        # 2. load + contract
        try:
            mech = contract.load_mechanism(candidate_path)
        except Exception as e:
            return report.fail("load", repr(e))
        report.stages["load"] = "ok"

        # 3+4. startup, smoke episode, shutdown, thread audit
        threads_before = threading.active_count()
        started = False
        try:
            if callable(getattr(mech, "startup", None)):
                mech.startup()
                started = True
            from episodes.scenario import Scenario
            scenario = Scenario.load(
                str(l1_suite.SCENARIO_DIR / "mission1_obstacle_field.json"))
            from level1 import param_space
            l1_suite.run_episode(scenario, param_space.defaults(), seed=0,
                                 mechanism=mech)
            report.stages["smoke"] = "ok"
        except Exception as e:
            return report.fail("smoke", repr(e))
        finally:
            if started and callable(getattr(mech, "shutdown", None)):
                try:
                    mech.shutdown()
                except Exception as e:
                    return report.fail("shutdown", repr(e))
        # settle briefly, then audit (a still-running thread = orphan)
        time.sleep(0.05)
        delta = threading.active_count() - threads_before
        if delta != 0:
            return report.fail(
                "thread_audit",
                f"{delta} orphaned thread(s) after shutdown — Event-based "
                "teardown missing (deterministic-teardown rule)")
        report.stages["thread_audit"] = "ok"

        # 5. full suite + keep rule vs the active baseline
        try:
            from level1 import param_space
            metrics_list, _ = l1_suite.run_suite(param_space.defaults(),
                                                 seeds, mech)
        except Exception as e:
            return report.fail("suite", repr(e))
        agg = aggregate(metrics_list)
        decision = evaluate(agg, baseline_agg, thresholds)
        report.keep_reasons = decision.reasons
        report.stages["suite"] = "ok"
        if not decision.keep:
            return report.fail("keep_rule", "; ".join(decision.reasons))
        report.stages["keep_rule"] = "ok"

        # promote: copy into mechanisms/ then flip active.json (atomic order:
        # file first, pointer second — a crash in between leaves baseline live)
        if promote:
            dest = contract.MECHANISMS_DIR / candidate_path.name
            if candidate_path.resolve() != dest.resolve():
                shutil.copy2(candidate_path, dest)
            contract.set_active(candidate_path.name)
        report.promoted = True
        print(f"[LEVEL2] PROMOTED {candidate_path.name}: "
              f"{'; '.join(decision.reasons)}", file=sys.stderr)
        return report
    finally:
        report.wall_clock_s = round(time.time() - t0, 2)
