#!/usr/bin/env python3
"""Gate G0 entry point: run one scripted episode headless, emit 3-objective metrics JSON.

Examples:
    # dev laptop / CI (no ROS, no SITL):
    python3 run_episode.py --scenario scenarios/mission1_transit.json \
        --backend kinematic --seed 0 --out ep.json

    # in the crusader container with SITL up (docker/sitl/run_sitl.sh):
    RX26_SITL_OK=1 python3 run_episode.py --scenario scenarios/mission1_transit.json \
        --backend sitl --mav udp:127.0.0.1:14550 --out ep.json
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root

from episodes.scenario import Scenario
from episodes.runner import EpisodeRunner
from evaluator import metrics


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--scenario", required=True)
    ap.add_argument("--backend", choices=["kinematic", "sitl"], default="kinematic")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--dt", type=float, default=0.1)
    ap.add_argument("--mav", default="udp:127.0.0.1:14550")
    ap.add_argument("--fidelity", default=None,
                    help="metrics tag; defaults to 'sim' for both backends")
    ap.add_argument("--params-file", default=None, help="hashed into the metrics record")
    ap.add_argument("--perception-trusted", action="store_true",
                    help="set ONLY after Gate G2 sign-off (docs/G2_bench_procedure.md)")
    ap.add_argument("--noise", type=float, default=0.0,
                    help="kinematic backend heading-noise std (rad/s)")
    ap.add_argument("--out", default="episode_metrics.json")
    args = ap.parse_args()

    scenario = Scenario.load(args.scenario)

    if args.backend == "kinematic":
        from episodes.backends.kinematic import KinematicBackend
        backend = KinematicBackend(noise_std=args.noise)
    else:
        from episodes.backends.sitl import SitlBackend
        backend = SitlBackend(endpoint=args.mav)

    result = EpisodeRunner(scenario, backend, seed=args.seed, dt=args.dt).run()
    m = metrics.assemble(result, scenario,
                         fidelity=args.fidelity or "sim",
                         params_file=args.params_file,
                         perception_trusted=args.perception_trusted)
    metrics.write_json(m, args.out)

    print(json.dumps(m, indent=2))
    print(f"\nwrote {args.out}", file=sys.stderr)
    # exit nonzero on auto-fail so shell scripts / CI can gate on it
    sys.exit(1 if m["objective1"]["auto_fail"] else 0)


if __name__ == "__main__":
    main()
