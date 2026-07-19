#!/usr/bin/env python3
"""Autoresearch entry point — Level 1 (+1.5) loop, optionally Level 2 every M
outer cycles. The LLM-driven flavor of what run_gate_g5.py does deterministically.

Examples:
    # deterministic (no API key needed):
    python3 run_autoresearch.py --proposer heuristic --iterations 20

    # LLM-driven Level 1 + a Level-2 research round every 4 iterations
    # (needs ANTHROPIC_API_KEY or an `ant auth login` profile):
    python3 run_autoresearch.py --proposer llm --iterations 20 --level2-every 4
"""
import argparse
import json
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))   # repo root

from evaluator.keep_rule import Thresholds, aggregate
from level1 import param_space, suite
from level1.loop import Level1Loop
from level1.proposers import HeuristicProposer, LlmProposer
from level1_5.strategy import FreezeRedirect
from level2 import contract, pipeline, validate


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--proposer", choices=["heuristic", "llm"],
                    default="heuristic")
    ap.add_argument("--model", default="claude-opus-4-8")
    ap.add_argument("--iterations", type=int, default=10)
    ap.add_argument("--seeds", type=int, nargs="+", default=[0, 1])
    ap.add_argument("--k-inner", type=int, default=5,
                    help="Level 1.5 cadence (every K inner iterations)")
    ap.add_argument("--level2-every", type=int, default=0,
                    help="run a Level-2 research round every M iterations "
                         "(0 = never; requires --proposer llm)")
    ap.add_argument("--out-dir", default="autoresearch_runs/latest")
    args = ap.parse_args()

    out_dir = Path(args.out_dir)
    llm = None
    if args.proposer == "llm" or args.level2_every:
        from llm import AnthropicLLM
        llm = AnthropicLLM(model=args.model)
    proposer = (LlmProposer(llm) if args.proposer == "llm"
                else HeuristicProposer(seed=0))

    mechanism = contract.load_active()
    print(f"active mechanism: {mechanism.name}")
    loop = Level1Loop(proposer, out_dir=out_dir, seeds=tuple(args.seeds),
                      mechanism=mechanism,
                      strategy=FreezeRedirect(k_inner=args.k_inner),
                      thresholds=Thresholds(min_episodes=1))

    if not args.level2_every:
        summary = loop.run(args.iterations)
    else:
        summary = None
        done = 0
        while done < args.iterations:
            chunk = min(args.level2_every, args.iterations - done)
            summary = loop.run(chunk)
            done += chunk
            if done >= args.iterations:
                break
            print(f"\n[LEVEL2] research round after {done} iterations ...")
            candidate = pipeline.run_research(llm, loop.history,
                                              out_dir / "candidates")
            baseline_agg = aggregate(suite.run_suite(
                loop.best_config, tuple(args.seeds), mechanism)[0])
            report = validate.validate_and_revert(candidate, baseline_agg,
                                                  seeds=tuple(args.seeds))
            (out_dir / "level2_reports.jsonl").open("a").write(
                json.dumps(report.__dict__, default=str) + "\n")
            if report.promoted:
                mechanism = contract.load_active()
                loop.mechanism = mechanism
                print(f"[LEVEL2] now running mechanism: {mechanism.name}")

    print(json.dumps({k: v for k, v in summary.items()
                      if k != "best_aggregate"}, indent=2))
    print(f"history: {out_dir / 'level1_history.jsonl'}")


if __name__ == "__main__":
    main()
