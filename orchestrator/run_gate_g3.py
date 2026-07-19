#!/usr/bin/env python3
"""Gate G3 — Mission-1 obstacle scenario, APF advisory on, N seeded runs.

Pass criteria (plan Phase 3):
  * zero clearance violations and zero hard collisions across ALL seeds,
  * every run completes (no timeouts),
  * zero stalls — at-risk flags are allowed only if they recover (a logged
    escape), never escalate.

Also runs one CONTROL episode (advisor off) and requires that it DOES violate
clearance — proving the scenario actually stresses avoidance; a gate that
passes on an empty test is not a gate.

Usage:
    python3 run_gate_g3.py [--seeds 20] [--noise 0.02] \
        [--scenario scenarios/mission1_obstacle_field.json] [--out g3_report.json]
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root

from episodes.apf_advisor import ApfAdvisor
from episodes.backends.kinematic import KinematicBackend
from episodes.runner import EpisodeRunner
from episodes.scenario import Scenario
from evaluator import metrics
from evaluator.keep_rule import aggregate


def run_one(scenario, seed, noise, advised=True):
    backend = KinematicBackend(noise_std=noise)
    advisor = ApfAdvisor(scenario) if advised else None
    result = EpisodeRunner(scenario, backend, seed=seed, advisor=advisor).run()
    return metrics.assemble(result, scenario), advisor


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario",
                    default=str(Path(__file__).parent / "scenarios"
                                / "mission1_obstacle_field.json"))
    ap.add_argument("--seeds", type=int, default=20)
    ap.add_argument("--noise", type=float, default=0.02)
    ap.add_argument("--out", default="g3_report.json")
    args = ap.parse_args()

    scenario = Scenario.load(args.scenario)
    failures = []

    # control: advisor OFF must violate — otherwise the scenario tests nothing
    control, _ = run_one(scenario, seed=0, noise=args.noise, advised=False)
    c1 = control["objective1"]
    control_stressed = (c1["clearance_violations"] > 0 or c1["hard_collisions"] > 0)
    if not control_stressed:
        failures.append("control run (advisor off) had no violations — "
                        "scenario does not stress avoidance")

    per_seed, trap_total = [], 0
    for seed in range(args.seeds):
        m, advisor = run_one(scenario, seed, args.noise, advised=True)
        per_seed.append(m)
        trap_total += advisor.trap_events
        o1, o2, o3 = m["objective1"], m["objective2"], m["objective3"]
        tag = f"seed {seed}"
        if o1["clearance_violations"] or o1["hard_collisions"]:
            failures.append(f"{tag}: clearance_violations="
                            f"{o1['clearance_violations']} "
                            f"hard={o1['hard_collisions']}")
        if not o3["completed"]:
            failures.append(f"{tag}: did not complete (timeout={o3['timed_out']})")
        if o2["stalls"]:
            failures.append(f"{tag}: {o2['stalls']} stall(s) — flags escalated")

    agg = aggregate(per_seed)
    report = {
        "gate": "G3",
        "scenario": scenario.name,
        "seeds": args.seeds,
        "noise": args.noise,
        "control_run_stressed": control_stressed,
        "control_objective1": c1,
        "aggregate": agg,
        "trap_events_total": trap_total,
        "pass": not failures,
        "failures": failures,
    }
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print(f"\nGate G3: {'PASS' if not failures else 'FAIL'}  ({args.out})",
          file=sys.stderr)
    sys.exit(0 if not failures else 1)


if __name__ == "__main__":
    main()
