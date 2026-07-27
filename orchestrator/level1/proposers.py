"""Level-1 proposers. A proposer suggests ONE parameter change + a hypothesis
tied to an objective. It never applies changes, never sees the keep rule, and
never touches loop structure — that separation is the whole design (CLAUDE.md:
keep-rule enforced by the evaluator, never by the proposing LLM).
"""
import random
from dataclasses import dataclass

from level1 import param_space

PROPOSAL_SCHEMA = {
    "type": "object",
    "properties": {
        "param": {"type": "string"},
        "value": {"type": "number"},
        "hypothesis": {"type": "string"},
        "objective": {"type": "integer", "enum": [1, 2, 3]},
    },
    "required": ["param", "value", "hypothesis", "objective"],
    "additionalProperties": False,
}

SYSTEM = """You are the Level-1 inner loop of an autoresearch harness tuning \
Crusader, a RobotX USV, in simulation. Propose exactly ONE parameter change per \
iteration with a one-sentence hypothesis tied to one of three objectives: \
1=minimize collision probability, 2=reduce local-minima/stall incidence, \
3=maximize mission completion. Respect the given bounds. Do not re-propose \
frozen parameters. Known failure mode of loops like you: near-deterministic \
repetition — avoid re-proposing the same 1-2 fixes for the same failure."""


@dataclass
class Proposal:
    param: str
    value: float
    hypothesis: str
    objective: int


class HeuristicProposer:
    """Deterministic pseudo-random hill-climber for CI/G5 — perturbs one
    unfrozen parameter per iteration by +/- its step."""

    def __init__(self, seed: int = 0):
        self.rng = random.Random(seed)

    def propose(self, best_config, frozen, guidance, history) -> Proposal:
        candidates = [n for n in param_space.SPACE if n not in frozen]
        if not candidates:
            candidates = list(param_space.SPACE)     # everything frozen: retry
        name = self.rng.choice(candidates)
        spec = param_space.SPACE[name]
        direction = self.rng.choice((-1, 1))
        value = min(spec.hi, max(spec.lo,
                                 best_config[name] + direction * spec.step))
        objective = 2 if "obj2" in (guidance or "") else \
            1 if "obj1" in (guidance or "") else self.rng.choice((1, 2, 3))
        return Proposal(name, round(value, 4),
                        f"perturb {name} {'+' if direction > 0 else '-'}"
                        f"{spec.step} to probe objective {objective}",
                        objective)


class LlmProposer:
    """The real Level-1 proposer: structured-output JSON from the LLM."""

    def __init__(self, llm):
        self.llm = llm

    def propose(self, best_config, frozen, guidance, history) -> Proposal:
        recent = history[-8:]
        prompt = (
            f"Current best config: {best_config}\n"
            f"Frozen parameters (do NOT propose): {sorted(frozen)}\n"
            f"Bounds: {{ {', '.join(f'{n}: [{s.lo}, {s.hi}]' for n, s in param_space.SPACE.items())} }}\n"
            f"Strategy guidance: {guidance or 'none'}\n"
            f"Recent iterations (param, kept, decision reason):\n" +
            "\n".join(f"  {r['param']}={r['value']} kept={r['kept']} "
                      f"({r['reason'][:100]})" for r in recent) +
            "\nPropose the next single change."
        )
        d = self.llm.complete_json(SYSTEM, prompt, PROPOSAL_SCHEMA)
        return Proposal(d["param"], float(d["value"]), d["hypothesis"],
                        int(d["objective"]))
