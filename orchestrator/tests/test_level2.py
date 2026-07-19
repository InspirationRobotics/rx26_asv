"""Level 2 — contract, validate-and-revert (every stage), promotion, pipeline."""
import json
import threading
import time
from pathlib import Path

import pytest

from evaluator.keep_rule import aggregate
from level1 import param_space, suite
from level2 import contract, pipeline, validate
from llm import ScriptedLLM

FIXTURES = Path(__file__).resolve().parents[1] / "level2" / "fixtures"


@pytest.fixture
def baseline_agg():
    metrics, _ = suite.run_suite(param_space.defaults(), seeds=(0,),
                                 suite=["mission1_obstacle_field.json"])
    return aggregate(metrics)


@pytest.fixture
def restore_active():
    before = contract.ACTIVE_FILE.read_text()
    yield
    contract.ACTIVE_FILE.write_text(before)


def test_load_active_baseline():
    mech = contract.load_active()
    assert mech.name == "baseline_apf"
    assert callable(mech.make_advisor)


def test_contract_rejects_missing_mechanism(tmp_path):
    p = tmp_path / "empty.py"
    p.write_text("x = 1\n")
    with pytest.raises(contract.ContractError, match="MECHANISM"):
        contract.load_mechanism(p)


def test_broken_import_reverts_at_load(baseline_agg, restore_active):
    r = validate.validate_and_revert(FIXTURES / "broken_import.py",
                                     baseline_agg, seeds=(0,))
    assert not r.promoted
    assert r.reverted_reason.startswith("load")
    assert r.stages["compile"] == "ok"       # syntactically fine; dies at import


def test_thread_leaker_reverts_at_thread_audit(baseline_agg, restore_active):
    before = threading.active_count()
    r = validate.validate_and_revert(FIXTURES / "thread_leaker.py",
                                     baseline_agg, seeds=(0,))
    assert not r.promoted
    assert r.reverted_reason.startswith("thread_audit")
    assert "orphaned" in r.reverted_reason
    time.sleep(0.6)                          # fixture thread self-expires
    assert threading.active_count() == before


def test_name_liar_reverts_at_smoke(baseline_agg, restore_active):
    r = validate.validate_and_revert(FIXTURES / "name_liar.py",
                                     baseline_agg, seeds=(0,))
    assert not r.promoted
    assert r.reverted_reason.startswith("smoke")
    assert "name_liar" in r.reverted_reason or "baseline_apf" in r.reverted_reason


def test_failed_injection_never_touches_active(baseline_agg, restore_active):
    before = contract.ACTIVE_FILE.read_text()
    validate.validate_and_revert(FIXTURES / "broken_import.py",
                                 baseline_agg, seeds=(0,))
    assert contract.ACTIVE_FILE.read_text() == before


def test_promotion_end_to_end_with_degraded_baseline(tmp_path, restore_active):
    """Prove the promote path: a valid candidate vs a fabricated worse baseline
    (identical schema, lower completion) -> keep -> active.json flips."""
    degraded = {
        "n_episodes": 1,
        "objective1": {"min_clearance_m": 2.5, "min_comms_clearance_m": None,
                       "clearance_violations": 0, "keepout_violations": 0,
                       "moving_10m_violations": 0, "hard_collisions": 0,
                       "auto_fail": False},
        "objective2": {"at_risk_flags": 0, "stalls": 0, "flagged_s": 0.0},
        "objective3": {"partial_credit": 0.5, "completed_all": False},
    }
    candidate = tmp_path / "promoted_variant.py"
    candidate.write_text((FIXTURES.parent / "mechanisms" /
                          "baseline_apf.py").read_text().replace(
        'name = "baseline_apf"', 'name = "promoted_variant"'))
    r = validate.validate_and_revert(candidate, degraded, seeds=(0,))
    assert r.promoted, r.reverted_reason
    assert r.stages["keep_rule"] == "ok"
    active = json.loads(contract.ACTIVE_FILE.read_text())
    assert active["module"] == "promoted_variant.py"
    # cleanup the promoted copy (restore_active resets the pointer)
    (contract.MECHANISMS_DIR / "promoted_variant.py").unlink()


def test_pipeline_four_rounds_and_extraction(tmp_path):
    code = ("class M:\n    name = 'gen'\n"
            "    def make_advisor(self, s, p):\n        return None\n"
            "MECHANISM = M()\n")
    llm = ScriptedLLM(text_responses=[
        "explore ideas", "critique: pick idea 2", "spec: ...",
        f"here is the module:\n```python\n{code}```"])
    path = pipeline.run_research(llm, history=[], out_dir=tmp_path,
                                 mechanism_name="gen_test")
    assert path.read_text() == code
    assert len(llm.calls) == 4
    assert all(kind == "text" for kind, _, _ in llm.calls)
    # Explore round saw the real sources and the failure trace prompt
    assert "apf_core.py" in llm.calls[0][2]
    assert (tmp_path / "gen_test.dialogue.json").exists()


def test_pipeline_no_code_block_fails_loudly(tmp_path):
    llm = ScriptedLLM(text_responses=["a", "b", "c", "no code here"])
    with pytest.raises(ValueError, match="python block"):
        pipeline.run_research(llm, history=[], out_dir=tmp_path)
