import copy

from evaluator.keep_rule import Thresholds, aggregate, evaluate


def episode(min_clear=3.0, violations=0, hard=0, moving=0, flags=0, stalls=0,
            flagged=0.0, credit=1.0, completed=True):
    return {
        "objective1": {"min_clearance_m": min_clear, "min_comms_clearance_m": None,
                       "clearance_violations": violations, "keepout_violations": 0,
                       "moving_10m_violations": moving, "hard_collisions": hard,
                       "auto_fail": hard > 0 or moving > 0, "trusted": False},
        "objective2": {"at_risk_flags": flags, "stalls": stalls, "escapes": 0,
                       "mean_recovery_s": None, "flagged_s": flagged, "causes": {}},
        "objective3": {"elements": {}, "partial_credit": credit,
                       "completed": completed, "timed_out": False,
                       "mission4": {}},
    }


BASELINE = aggregate([episode(credit=0.8, flags=2, flagged=10.0) for _ in range(5)])


def test_small_sample_guard():
    cand = aggregate([episode(credit=1.0)])
    d = evaluate(cand, BASELINE)
    assert not d.keep
    assert "small-sample" in d.reasons[0]


def test_hard_collision_rejected():
    cand = aggregate([episode(hard=1)] + [episode() for _ in range(4)])
    assert not evaluate(cand, BASELINE).keep


def test_completion_improvement_kept():
    cand = aggregate([episode(credit=1.0, flags=2, flagged=10.0) for _ in range(5)])
    d = evaluate(cand, BASELINE)
    assert d.keep, d.reasons


def test_conservatism_tradeoff_rejected():
    # fewer flags but completion dropped: the blended-score-gaming guard
    cand = aggregate([episode(credit=0.5, flags=0, flagged=0.0) for _ in range(5)])
    d = evaluate(cand, BASELINE)
    assert not d.keep
    assert any("completion dropped" in r for r in d.reasons)


def test_clearance_regression_rejected():
    cand = aggregate([episode(min_clear=1.0, credit=1.0) for _ in range(5)])
    d = evaluate(cand, BASELINE)   # baseline min_clear 3.0, regress > 0.25 -> reject
    assert not d.keep


def test_no_improvement_rejected():
    cand = copy.deepcopy(BASELINE)
    d = evaluate(cand, BASELINE)
    assert not d.keep
    assert "no improvement" in d.reasons[0]


def test_safety_floor_absolute():
    # even vs a bad baseline, clearance below the absolute floor is rejected
    bad_base = aggregate([episode(min_clear=0.4, credit=0.5) for _ in range(5)])
    cand = aggregate([episode(min_clear=0.4, credit=1.0) for _ in range(5)])
    d = evaluate(cand, bad_base, Thresholds())
    assert not d.keep
    assert any("safety floor" in r for r in d.reasons)
