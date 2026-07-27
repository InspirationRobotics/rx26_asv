"""Level1Loop — the inner autoresearch loop (parameter-level task optimization).

Per iteration: proposer suggests one change + hypothesis -> param_space
validates (bounds + param_guard protection fence) -> full fixed suite x seeds
-> keep_rule.evaluate vs the current best (evaluator-enforced, never
LLM-enforced) -> keep/discard -> JSONL history (Level 2's Explore round is
bottlenecked by the quality of this trace — log everything).
"""
import json
import time
from pathlib import Path

from evaluator.keep_rule import Thresholds, aggregate, evaluate

from level1 import param_space, suite


class Level1Loop:
    def __init__(self, proposer, out_dir, seeds=(0, 1), mechanism=None,
                 strategy=None, thresholds: Thresholds = None):
        self.proposer = proposer
        self.out_dir = Path(out_dir)
        self.out_dir.mkdir(parents=True, exist_ok=True)
        self.seeds = tuple(seeds)
        self.mechanism = mechanism
        self.strategy = strategy               # Level 1.5 hook (may be None)
        self.thresholds = thresholds or Thresholds(min_episodes=1)
        self.history = []                      # per-iteration records
        self.episode_records = []              # per-episode audit records
        self.best_config = param_space.defaults()
        self.best_agg = None

    def _evaluate_config(self, config):
        m, records = suite.run_suite(config, self.seeds, self.mechanism)
        self.episode_records.extend(records)
        return aggregate(m)

    def run(self, iterations: int):
        history_path = self.out_dir / "level1_history.jsonl"
        self.best_agg = self._evaluate_config(self.best_config)

        with open(history_path, "a") as log:
            for i in range(iterations):
                frozen = self.strategy.frozen if self.strategy else set()
                guidance = self.strategy.guidance if self.strategy else ""
                prop = self.proposer.propose(self.best_config, frozen,
                                             guidance, self.history)

                record = {"iteration": i, "t": time.time(),
                          "param": prop.param, "value": prop.value,
                          "hypothesis": prop.hypothesis,
                          "objective": prop.objective,
                          "frozen": sorted(frozen), "guidance": guidance}

                err = param_space.validate(prop.param, prop.value)
                if err is None and prop.param in frozen:
                    err = f"{prop.param} is frozen (Level 1.5)"
                if err:
                    record.update(kept=False, reason=f"rejected: {err}",
                                  aggregate=None)
                else:
                    candidate = dict(self.best_config)
                    candidate[prop.param] = prop.value
                    agg = self._evaluate_config(candidate)
                    decision = evaluate(agg, self.best_agg, self.thresholds)
                    record.update(kept=decision.keep,
                                  reason="; ".join(decision.reasons),
                                  aggregate=agg)
                    if decision.keep:
                        self.best_config = candidate
                        self.best_agg = agg

                self.history.append(record)
                log.write(json.dumps(record) + "\n")
                log.flush()

                if self.strategy:
                    self.strategy.update(self.history, self.best_agg)

        return {
            "iterations": iterations,
            "kept": sum(1 for r in self.history if r["kept"]),
            "rejected": sum(1 for r in self.history
                            if r["reason"].startswith("rejected")),
            "best_config": self.best_config,
            "best_aggregate": self.best_agg,
            "episodes": len(self.episode_records),
        }
