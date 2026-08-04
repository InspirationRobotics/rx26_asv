"""Tests for the QGC .plan -> mission_file converter.

Fixtures mirror the real shape QGC writes: a SimpleItem carries its coordinate
in params[4]/params[5] (MAVLink param5/param6), and home lives in
`plannedHomePosition` rather than the item list — so unlike the MAVLink download
path, there is no seq-0 home entry to filter out here.
"""
import importlib.util
import json
import sys
from pathlib import Path

import pytest

spec = importlib.util.spec_from_file_location(
    "plan_to_mission",
    Path(__file__).parent.parent / "tools" / "scripts" / "plan_to_mission.py")
p2m = importlib.util.module_from_spec(spec)
sys.modules["plan_to_mission"] = p2m
spec.loader.exec_module(p2m)


def item(lat, lon, command=16):
    return {"type": "SimpleItem", "command": command, "autoContinue": True,
            "frame": 3, "doJumpId": 1,
            "params": [0, 0, 0, None, lat, lon, 0]}


def plan(items):
    return {"fileType": "Plan", "version": 1,
            "mission": {"items": items,
                        "plannedHomePosition": [37.0, -122.0, 0],
                        "vehicleType": 10, "firmwareType": 3},
            "geoFence": {"circles": [], "polygons": []},
            "rallyPoints": {"points": []}}


def test_extracts_in_mission_order():
    wps, skipped = p2m.extract(plan([
        item(37.1234567, -122.1234567),
        item(37.1234800, -122.1234900),
        item(37.1235100, -122.1235200),
    ]))
    assert wps == [[37.1234567, -122.1234567],
                   [37.1234800, -122.1234900],
                   [37.1235100, -122.1235200]]
    assert skipped == []


def test_planned_home_is_not_a_waypoint():
    # home lives outside `items`; converting it would silently prepend a
    # waypoint and shift every gate_wp_indices entry by one
    wps, _ = p2m.extract(plan([item(37.5, -122.5)]))
    assert wps == [[37.5, -122.5]]


def test_spline_waypoints_accepted():
    wps, _ = p2m.extract(plan([item(37.5, -122.5, command=82)]))
    assert wps == [[37.5, -122.5]]


def test_non_position_commands_skipped_and_reported():
    wps, skipped = p2m.extract(plan([
        item(37.5, -122.5),
        item(0, 0, command=178),          # DO_CHANGE_SPEED
        item(37.6, -122.6),
    ]))
    assert wps == [[37.5, -122.5], [37.6, -122.6]]
    # the skip must be REPORTED — a silent drop renumbers gate_wp_indices
    assert [i for i, _ in skipped] == [1]
    assert "178" in skipped[0][1]


def test_complex_item_skipped_and_reported():
    survey = {"type": "ComplexItem", "complexItemType": "survey"}
    wps, skipped = p2m.extract(plan([item(37.5, -122.5), survey]))
    assert wps == [[37.5, -122.5]]
    assert "ComplexItem" in skipped[0][1]


def test_null_and_null_island_coordinates_skipped():
    wps, skipped = p2m.extract(plan([
        item(None, None),
        item(0, 0),
        item(37.5, -122.5),
    ]))
    assert wps == [[37.5, -122.5]]
    assert [i for i, _ in skipped] == [0, 1]


def test_transposed_coordinate_raises():
    # a longitude in the lat slot is out of [-90, 90]; fail rather than convert
    with pytest.raises(p2m.PlanError, match="transposed"):
        p2m.extract(plan([item(-122.5, 37.5)]))


def test_rejects_non_plan_file():
    with pytest.raises(p2m.PlanError, match="fileType"):
        p2m.extract({"fileType": "Fence", "mission": {"items": [item(37.5, -122.5)]}})


def test_rejects_empty_mission():
    with pytest.raises(p2m.PlanError, match="no items"):
        p2m.extract(plan([]))


def test_all_items_unconvertible_raises_rather_than_writing_empty():
    # a survey-only plan must fail loudly, not produce {"waypoints": []}
    survey = {"type": "ComplexItem", "complexItemType": "survey"}
    with pytest.raises(p2m.PlanError, match="no convertible waypoints"):
        p2m.extract(plan([survey]))


def test_cli_round_trip(tmp_path):
    src = tmp_path / "course.plan"
    src.write_text(json.dumps(plan([item(37.5, -122.5), item(37.6, -122.6)])))
    out = tmp_path / "nested" / "mission_1.json"

    assert p2m.main([str(src), "-o", str(out)]) == 0
    assert json.loads(out.read_text()) == {
        "waypoints": [[37.5, -122.5], [37.6, -122.6]]}


def test_cli_reports_missing_file_without_traceback(tmp_path):
    result = p2m.main([str(tmp_path / "nope.plan"), "-o", str(tmp_path / "o.json")])
    assert isinstance(result, str) and "no such plan file" in result