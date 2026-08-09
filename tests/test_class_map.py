"""load_class_map — the detector's fail-loud gate on class naming.

detector.py imports ultralytics at module scope (it lives in the container, not
in the dev venv), so stub it before importing. Only load_class_map is exercised.
"""
import json
import sys
import types

import pytest

if "ultralytics" not in sys.modules:            # dev host has no ultralytics
    stub = types.ModuleType("ultralytics")
    stub.YOLO = object
    sys.modules["ultralytics"] = stub

from rx26_asv.api.perception.detector import (      # noqa: E402
    CANONICAL_LABELS, DEFAULT_CLASS_MAP, load_class_map)


def write(tmp_path, obj):
    p = tmp_path / "class_map.json"
    p.write_text(json.dumps(obj))
    return p


def test_comment_keys_are_not_class_entries(tmp_path):
    """The shipped file carries a "_comment"; it must not be read as a mapping."""
    cmap = load_class_map(write(tmp_path, {
        "_comment": "free text that is not a canonical label",
        "red_buoy": "buoy_flash_red",
    }))
    assert cmap == {"red_buoy": "buoy_flash_red"}


def test_unknown_canonical_label_still_fails_loudly(tmp_path):
    with pytest.raises(ValueError, match="unknown canonical labels"):
        load_class_map(write(tmp_path, {"red_buoy": "buoy_flashing_red"}))


def test_null_maps_are_allowed(tmp_path):
    cmap = load_class_map(write(tmp_path, {"duck_image": None}))
    assert cmap == {"duck_image": None}


def test_shipped_class_map_loads():
    cmap = load_class_map(DEFAULT_CLASS_MAP)
    assert cmap, "shipped class_map.json has no class entries"
    assert not {v for v in cmap.values() if v is not None} - CANONICAL_LABELS


# Verified on the Jetson 2026-08-05:
#   python3 -c "from ultralytics import YOLO; \
#       print(YOLO('/root/robotx_ws/models/buoy_v16.engine').names)"
# Detector.__init__ makes an unmapped class fatal at startup; this makes it fatal
# in CI instead, so a retrain that renames classes is caught before the boat.
BUOY_V16_NAMES = [
    "black_buoy", "black_cross", "black_target_boat", "black_triangle",
    "blue_buoy", "green_buoy", "green_light_buoy", "green_pole_buoy",
    "red_buoy", "red_light_buoy", "red_pole_buoy", "yellow_buoy",
    "yellow_target_boat",
]


def test_every_model_class_is_mapped():
    cmap = load_class_map(DEFAULT_CLASS_MAP)
    assert set(BUOY_V16_NAMES) - set(cmap) == set(), "model class with no mapping"
    assert set(cmap) - set(BUOY_V16_NAMES) == set(), "mapping for a nonexistent class"


def test_gate_classes_reach_gate_navigator():
    """The six red/green model classes must fold into the two labels
    gate_navigator matches on, or Mission 1 sees no gates at all."""
    cmap = load_class_map(DEFAULT_CLASS_MAP)
    red = {"red_buoy", "red_pole_buoy", "red_light_buoy"}
    green = {"green_buoy", "green_pole_buoy", "green_light_buoy"}
    assert {cmap[n] for n in red} == {"buoy_flash_red"}
    assert {cmap[n] for n in green} == {"buoy_flash_green"}


def test_no_physical_object_is_dropped():
    """objective 1: losing an obstacle is worse than a coarse label. Every class
    this model emits is a real object, so none may map to null."""
    cmap = load_class_map(DEFAULT_CLASS_MAP)
    assert [k for k, v in cmap.items() if v is None] == []
