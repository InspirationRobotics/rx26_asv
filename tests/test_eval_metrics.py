"""Tests for the model-regression scoring logic (tools/training/eval_regression.py)."""
import importlib.util
import sys
from pathlib import Path

spec = importlib.util.spec_from_file_location(
    "eval_regression",
    Path(__file__).parent.parent / "tools" / "training" / "eval_regression.py")
ev = importlib.util.module_from_spec(spec)
sys.modules["eval_regression"] = ev
spec.loader.exec_module(ev)


B = (100, 100, 200, 200)          # a reference bbox
B_NEAR = (110, 110, 210, 210)     # IoU ~0.68 with B
B_FAR = (400, 400, 500, 500)


def test_iou_basics():
    assert ev.iou(B, B) == 1.0
    assert ev.iou(B, B_FAR) == 0.0
    assert 0.6 < ev.iou(B, B_NEAR) < 0.75


def test_greedy_match_prefers_high_conf():
    preds = [("a", 0.5, B_NEAR), ("a", 0.9, B)]
    gts = [("a", B)]
    matches, un_p, un_g = ev.greedy_match(preds, gts)
    assert matches == [(1, 0)]        # the 0.9 pred wins the only gt
    assert un_p == [0] and un_g == []


def test_score_correct_and_confused():
    frames = [
        # correct match
        ([("buoy_flash_red", 0.9, B)], [("buoy_flash_red", B)]),
        # localization right, class wrong -> confusion + fp/fn
        ([("buoy_flash_green", 0.9, B)], [("buoy_flash_red", B)]),
        # missed gt
        ([], [("buoy_flash_red", B)]),
        # spurious pred
        ([("buoy_flash_red", 0.9, B_FAR)], []),
    ]
    r = ev.score(frames)
    red = r["per_class"]["buoy_flash_red"]
    assert red["tp"] == 1 and red["fn"] == 2 and red["fp"] == 1
    assert r["confusion"]["buoy_flash_red->buoy_flash_green"] == 1


def test_gate_fails_on_missing_class_coverage():
    r = ev.score([([("buoy_flash_red", 0.9, B)], [("buoy_flash_red", B)])])
    ok, failures = ev.gate(r)
    assert not ok
    assert any("no ground truth" in f for f in failures)


def test_gate_passes_with_full_coverage():
    frames = []
    for c in ev.GATED_CLASSES:
        frames += [([(c, 0.9, B)], [(c, B)])] * 10
    ok, failures = ev.gate(ev.score(frames))
    assert ok, failures


def test_gate_fails_on_low_recall():
    frames = []
    for c in ev.GATED_CLASSES:
        frames += [([(c, 0.9, B)], [(c, B)])] * 7 + [([], [(c, B)])] * 3  # recall 0.7
    ok, failures = ev.gate(ev.score(frames))
    assert not ok
    assert any("recall" in f for f in failures)
