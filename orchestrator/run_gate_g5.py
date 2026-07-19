#!/usr/bin/env python3
"""Gate G5 — unattended autoresearch run (plan Phase 5).

Pass criteria:
  * >= 50 episodes executed across Level-1 iterations (fixed suite x seeds),
  * ZERO orphaned threads (global count identical before/after the whole run,
    and every per-episode thread_delta == 0),
  * every episode carries a logged active-mechanism assertion,
  * >= 1 revert exercised END-TO-END via deliberately broken mechanisms
    (import-crash, thread-leaker, name-liar — each must be caught at its
    intended stage and NOT promoted),
  * a valid candidate runs every validation stage cleanly. On an all-green
    suite the keep rule rightly rejects a no-improvement variant, so the gate
    accepts promoted OR keep-rule-rejected here; the promotion/activation path
    itself is proven in tests/test_level2.py against a degraded baseline.

Deterministic (HeuristicProposer) so CI can run it; the LLM-driven flavor is
run_autoresearch.py.

Usage: python3 run_gate_g5.py [--episodes 50] [--out g5_report.json]
"""
import argparse
import json
import shutil
import sys
import tempfile
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root

from evaluator.keep_rule import Thresholds, aggregate
from level1 import param_space, suite
from level1.loop import Level1Loop
from level1.proposers import HeuristicProposer
from level1_5.strategy import FreezeRedirect
from level2 import contract, validate

FIXTURES = Path(__file__).parent / "level2" / "fixtures"

GOOD_CANDIDATE = '''"""G5 promotion-drill candidate: APF with a stronger tangent
gain - a valid, contract-complete variant."""
import sys
from pathlib import Path
_ORCH = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(_ORCH))
sys.path.insert(0, str(_ORCH.parent))
from episodes.apf_advisor import ApfAdvisor


class StrongTangent:
    name = "g5_strong_tangent"

    def make_advisor(self, scenario, apf_params):
        import dataclasses
        p = dataclasses.replace(apf_params, tangent_gain=1.2)
        advisor = ApfAdvisor(scenario, params=p)
        advisor.mechanism_name = self.name
        return advisor


MECHANISM = StrongTangent()
'''


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--out", default="g5_report.json")
    args = ap.parse_args()

    failures = []
    t0 = time.time()
    threads_at_start = threading.active_count()
    workdir = Path(tempfile.mkdtemp(prefix="g5_"))

    # ---------- Phase A: unattended Level-1 run (with Level 1.5) ----------
    seeds = (0, 1)
    episodes_per_iter = len(suite.DEFAULT_SUITE) * len(seeds)
    iterations = max(1, -(-args.episodes // episodes_per_iter))  # ceil
    loop = Level1Loop(HeuristicProposer(seed=42), out_dir=workdir,
                      seeds=seeds, strategy=FreezeRedirect(k_inner=3),
                      thresholds=Thresholds(min_episodes=1))
    summary = loop.run(iterations)

    total_episodes = summary["episodes"]
    if total_episodes < args.episodes:
        failures.append(f"only {total_episodes} episodes (< {args.episodes})")
    bad_assert = [r for r in loop.episode_records
                  if not r["mechanism_asserted"]]
    if bad_assert:
        failures.append(f"{len(bad_assert)} episodes missing mechanism assertion")
    leaky = [r for r in loop.episode_records if r["thread_delta"] != 0]
    if leaky:
        failures.append(f"{len(leaky)} episodes with nonzero thread delta")
    if not (workdir / "level1_history.jsonl").exists():
        failures.append("level1 history JSONL missing")

    # ---------- Phase B: revert drills (broken mechanisms) ----------
    baseline_agg = aggregate(
        suite.run_suite(param_space.defaults(), seeds,
                        contract.load_active())[0])
    drills = {}
    expected = {"broken_import.py": "load",
                "thread_leaker.py": "thread_audit",
                "name_liar.py": "smoke"}
    active_before = contract.ACTIVE_FILE.read_text()
    for fixture, want_stage in expected.items():
        report = validate.validate_and_revert(FIXTURES / fixture, baseline_agg,
                                              seeds=seeds)
        drills[fixture] = {"promoted": report.promoted,
                           "reverted_reason": report.reverted_reason}
        if report.promoted:
            failures.append(f"{fixture}: was PROMOTED — revert failed")
        if not report.reverted_reason or \
                not report.reverted_reason.startswith(want_stage):
            failures.append(f"{fixture}: expected revert at {want_stage!r}, "
                            f"got {report.reverted_reason!r}")
    if contract.ACTIVE_FILE.read_text() != active_before:
        failures.append("active.json changed during failed injections")

    # ---------- Phase C: promotion drill (valid candidate) + restore ----------
    good_path = workdir / "g5_strong_tangent.py"
    good_path.write_text(GOOD_CANDIDATE, encoding="utf-8")
    # Candidate and baseline are scored on IDENTICAL suite/seeds — never
    # handicap the baseline to force a promotion. Stages must all run clean;
    # keep-rule may correctly reject a no-improvement variant.
    report = validate.validate_and_revert(good_path, baseline_agg, seeds=seeds)
    promotion_stages_ok = all(report.stages.get(s) == "ok" for s in
                              ("compile", "load", "smoke", "thread_audit",
                               "suite"))
    if not promotion_stages_ok:
        failures.append(f"good candidate failed mechanics: {report.stages}")
    if report.promoted:
        # restore baseline (G5 cleanup: never leave a drill mechanism active)
        contract.set_active("baseline_apf.py")
        (contract.MECHANISMS_DIR / good_path.name).unlink(missing_ok=True)
    drills["g5_strong_tangent.py"] = {"promoted": report.promoted,
                                      "stages": report.stages,
                                      "keep": report.keep_reasons}

    # ---------- global thread audit ----------
    time.sleep(0.6)                    # let the leaker fixture's thread expire
    threads_at_end = threading.active_count()
    if threads_at_end != threads_at_start:
        failures.append(f"global thread count {threads_at_start} -> "
                        f"{threads_at_end}: orphaned threads")

    shutil.rmtree(workdir, ignore_errors=True)
    report_out = {
        "gate": "G5",
        "episodes": total_episodes,
        "level1": {k: v for k, v in summary.items() if k != "best_aggregate"},
        "level1_5_events": loop.strategy.events,
        "frozen_at_end": sorted(loop.strategy.frozen),
        "revert_drills": drills,
        "reverts_exercised": sum(1 for d in drills.values()
                                 if d.get("reverted_reason")),
        "threads": {"start": threads_at_start, "end": threads_at_end},
        "wall_clock_s": round(time.time() - t0, 1),
        "pass": not failures,
        "failures": failures,
    }
    Path(args.out).write_text(json.dumps(report_out, indent=2, default=str))
    print(json.dumps(report_out, indent=2, default=str))
    print(f"\nGate G5: {'PASS' if not failures else 'FAIL'}  ({args.out})",
          file=sys.stderr)
    sys.exit(0 if not failures else 1)


if __name__ == "__main__":
    main()
