"""Level 1.5 — freeze/redirect bookkeeping (and nothing else)."""
from level1_5.strategy import FreezeRedirect


def hist(param, kept, n):
    return [{"param": param, "kept": kept} for _ in range(n)]


def agg(hard=0, viol=0, stalls=0, flagged=0.0, credit=1.0):
    return {"objective1": {"hard_collisions": hard,
                           "clearance_violations": viol,
                           "keepout_violations": 0,
                           "moving_10m_violations": 0},
            "objective2": {"stalls": stalls, "flagged_s": flagged},
            "objective3": {"partial_credit": credit}}


def test_freezes_stalled_param_after_k_proposals():
    s = FreezeRedirect(k_inner=3, freeze_after=3)
    s.update(hist("apf_k_rep", False, 3), agg())
    assert "apf_k_rep" in s.frozen
    assert any(a == "freeze" for _, a, _ in s.events)


def test_param_with_a_keep_is_never_frozen():
    s = FreezeRedirect(k_inner=3, freeze_after=3)
    h = hist("apf_k_rep", False, 2) + hist("apf_k_rep", True, 1)
    s.update(h, agg())
    assert "apf_k_rep" not in s.frozen


def test_only_updates_on_k_inner_boundary():
    s = FreezeRedirect(k_inner=5, freeze_after=3)
    s.update(hist("apf_k_rep", False, 4), agg())     # n=4, not a boundary
    assert not s.frozen


def test_guidance_points_at_dominant_failure():
    s = FreezeRedirect(k_inner=1)
    s.update(hist("x", False, 1), agg(viol=2))
    assert "obj1" in s.guidance
    s.update(hist("x", False, 2), agg(stalls=1))
    assert "obj2" in s.guidance
    s.update(hist("x", False, 3), agg(credit=0.5))
    assert "obj3" in s.guidance
    s.update(hist("x", False, 4), agg())
    assert "green" in s.guidance


def test_unfreezes_when_failure_mode_changes():
    s = FreezeRedirect(k_inner=3, freeze_after=3)
    s.update(hist("apf_k_rep", False, 3), agg(viol=2))     # frozen under obj1
    assert "apf_k_rep" in s.frozen
    s.update(hist("apf_k_rep", False, 6), agg(stalls=1))   # failure mode -> obj2
    assert "apf_k_rep" not in s.frozen
    assert any(a == "unfreeze" for _, a, _ in s.events)
