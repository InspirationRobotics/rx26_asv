"""Level 1 (+ proposers, param space, suite audit trail)."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))   # repo root

from evaluator.keep_rule import Thresholds
from level1 import param_space, suite
from level1.loop import Level1Loop
from level1.proposers import (HeuristicProposer, LlmProposer, Proposal,
                              PROPOSAL_SCHEMA)
from llm import ScriptedLLM


def test_param_space_bounds_and_kinds():
    assert param_space.validate("apf_k_rep", 5.0) is None
    assert "outside" in param_space.validate("apf_k_rep", 999.0)
    assert "numeric" in param_space.validate("apf_k_rep", "high")


def test_param_space_ardupilot_fence():
    # protected ArduRover params are rejected at the space, before evaluation
    assert "PROTECTED" in param_space.validate("SERVO1_REVERSED", 0)
    assert "PROTECTED" in param_space.validate("ARMING_CHECK", 0)
    # tunables are recognized but SITL-only (not in the kinematic space)
    assert "SITL" in param_space.validate("AVOID_MARGIN", 3.0)
    assert "unknown" in param_space.validate("NOT_A_PARAM", 1.0)


def test_suite_records_carry_audit_fields():
    metrics, records = suite.run_suite(param_space.defaults(), seeds=(0,),
                                       suite=["mission1_transit.json"])
    assert len(metrics) == len(records) == 1
    r = records[0]
    assert r["mechanism"] == "baseline_apf"
    assert r["mechanism_asserted"] is True
    assert r["thread_delta"] == 0


def test_mechanism_assertion_fails_loudly():
    class Liar:
        name = "want_this"

        def make_advisor(self, scenario, apf_params):
            from episodes.apf_advisor import ApfAdvisor
            a = ApfAdvisor(scenario, params=apf_params)
            a.mechanism_name = "something_else"
            return a

    import pytest
    with pytest.raises(suite.MechanismAssertionError):
        suite.run_suite(param_space.defaults(), seeds=(0,), mechanism=Liar(),
                        suite=["mission1_transit.json"])


def test_loop_keeps_history_and_enforces_rejections(tmp_path):
    class FixedProposer:
        """Proposes an out-of-bounds value then a frozen-set violation then a
        valid change — the loop must record all three verdicts."""
        def __init__(self):
            self.seq = [Proposal("apf_k_rep", 999.0, "oob", 1),
                        Proposal("apf_k_rep", 6.0, "ok-but-frozen", 2),
                        Proposal("cruise_speed", 2.3, "valid", 3)]

        def propose(self, best, frozen, guidance, history):
            return self.seq.pop(0)

    class FreezeKRep:
        frozen = {"apf_k_rep"}
        guidance = ""

        def update(self, history, best_agg):
            pass

    loop = Level1Loop(FixedProposer(), out_dir=tmp_path, seeds=(0,),
                      strategy=FreezeKRep(),
                      thresholds=Thresholds(min_episodes=1))
    # frozen check applies to proposal 2; proposal 1 rejected on bounds
    loop.strategy.frozen = set()          # proposal 1: only bounds
    summary = loop.run(1)
    loop.strategy.frozen = {"apf_k_rep"}  # proposal 2: frozen
    loop.run(2)
    assert loop.history[0]["reason"].startswith("rejected")
    assert "frozen" in loop.history[1]["reason"]
    assert loop.history[2]["reason"] and not \
        loop.history[2]["reason"].startswith("rejected")
    assert (tmp_path / "level1_history.jsonl").exists()
    assert summary["episodes"] > 0


def test_llm_proposer_parses_structured_response():
    llm = ScriptedLLM(json_responses=[{
        "param": "apf_influence_m", "value": 7.0,
        "hypothesis": "wider influence reduces late repulsion (obj 2)",
        "objective": 2}])
    p = LlmProposer(llm).propose(param_space.defaults(), set(), "", [])
    assert p.param == "apf_influence_m" and p.value == 7.0 and p.objective == 2
    assert llm.calls[0][0] == "json"
    assert PROPOSAL_SCHEMA["required"] == ["param", "value", "hypothesis",
                                           "objective"]


def test_heuristic_proposer_deterministic_and_respects_frozen():
    a = HeuristicProposer(seed=7).propose(param_space.defaults(), set(), "", [])
    b = HeuristicProposer(seed=7).propose(param_space.defaults(), set(), "", [])
    assert (a.param, a.value) == (b.param, b.value)
    frozen = set(param_space.SPACE) - {"cruise_speed"}
    for _ in range(10):
        p = HeuristicProposer(seed=1).propose(param_space.defaults(), frozen,
                                              "", [])
        assert p.param == "cruise_speed"
