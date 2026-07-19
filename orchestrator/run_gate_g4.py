#!/usr/bin/env python3
"""Gate G4 — Mission-4 Core: interrupt mid-transit -> suspend -> transit ->
loiter -> READINESS -> Clearance -> resume WITH PROGRESS PRESERVED.

Pass criteria (plan Phase 4), every seed:
  * mission completes; zero clearance violations / hard collisions; zero stalls
  * comms compliance == 1.0 and ack ordering correct (receipt<=intent<=
    readiness<=resumption), immediate-ack latency <= max_ack_latency
  * resume fidelity TRUE — the explicit check, not "eventually finishes":
    the post-resume context equals the pre-suspend context (waypoint index +
    reached list), so a silent restart-from-scratch FAILS this gate
  * the concurrent Advanced-tier keep-out (K1) is acked and never entered,
    and the mission never pauses for it (no extra stack pushes)

The APF advisor runs throughout — G4 exercises planner + avoidance together.

Usage: python3 run_gate_g4.py [--seeds 10] [--noise 0.02] [--out g4_report.json]
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root

from episodes.apf_advisor import ApfAdvisor
from episodes.backends.kinematic import KinematicBackend
from episodes.planner_episode import PlannerEpisodeRunner
from episodes.scenario import Scenario
from evaluator import metrics

MAX_ACK_LATENCY_S = 1.0


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario",
                    default=str(Path(__file__).parent / "scenarios"
                                / "mission4_core.json"))
    ap.add_argument("--seeds", type=int, default=10)
    ap.add_argument("--noise", type=float, default=0.02)
    ap.add_argument("--out", default="g4_report.json")
    args = ap.parse_args()

    scenario = Scenario.load(args.scenario)
    failures, per_seed = [], []

    for seed in range(args.seeds):
        backend = KinematicBackend(noise_std=args.noise)
        advisor = ApfAdvisor(scenario)
        result, planner = PlannerEpisodeRunner(
            scenario, backend, seed=seed, advisor=advisor).run()
        m = metrics.assemble(result, scenario, planner=planner)
        per_seed.append(m)
        tag = f"seed {seed}"
        o1, o2, o3 = m["objective1"], m["objective2"], m["objective3"]
        m4 = o3["mission4"]

        if not o3["completed"]:
            failures.append(f"{tag}: mission incomplete "
                            f"(reached={o3['elements']})")
        if o1["clearance_violations"] or o1["hard_collisions"]:
            failures.append(f"{tag}: violations={o1['clearance_violations']} "
                            f"hard={o1['hard_collisions']}")
        if o1["keepout_violations"]:
            failures.append(f"{tag}: entered keep-out "
                            f"({o1['keepout_violations']}x)")
        if o2["stalls"]:
            failures.append(f"{tag}: {o2['stalls']} stall(s)")
        if m4["comms_compliance"] != 1.0:
            failures.append(f"{tag}: comms compliance {m4['comms_compliance']}")
        if not m4["ack_correct"]:
            failures.append(f"{tag}: ack ordering/completeness wrong")
        if m4["ack_latency_s"] is None or m4["ack_latency_s"] > MAX_ACK_LATENCY_S:
            failures.append(f"{tag}: ack latency {m4['ack_latency_s']}s "
                            f"> {MAX_ACK_LATENCY_S}s")
        if m4["resume_fidelity"] is not True:
            failures.append(f"{tag}: RESUME FIDELITY FAILED — "
                            f"{planner.resume_record}")
        if planner.stack.depth != 0:
            failures.append(f"{tag}: task stack not empty at episode end")
        if len(planner.stack.push_log) != 1:
            failures.append(f"{tag}: expected exactly 1 stack push "
                            f"(keep-outs must not push), got "
                            f"{len(planner.stack.push_log)}")

    report = {
        "gate": "G4",
        "scenario": scenario.name,
        "seeds": args.seeds,
        "noise": args.noise,
        "example_mission4": per_seed[0]["objective3"]["mission4"],
        "pass": not failures,
        "failures": failures,
    }
    Path(args.out).write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))
    print(f"\nGate G4: {'PASS' if not failures else 'FAIL'}  ({args.out})",
          file=sys.stderr)
    sys.exit(0 if not failures else 1)


if __name__ == "__main__":
    main()
