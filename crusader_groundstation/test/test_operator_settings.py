"""operator_settings off-boat: what is saved, what is set back, and what is never touched.

    python3 -m pytest crusader_groundstation/test/test_operator_settings.py
    python3 -m unittest discover -s crusader_groundstation/test        (no pytest needed)

Needs nothing outside the standard library.
"""
import json
import os
import re
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from crusader_groundstation import operator_settings as ops  # noqa: E402

UNSET = {"dock_heading_deg": -1.0, "search_first": ""}


class OperatorSettingsTest(unittest.TestCase):

    def setUp(self):
        self.dir = tempfile.TemporaryDirectory()
        self.path = os.path.join(self.dir.name, "sub", "operator_settings.json")

    def tearDown(self):
        self.dir.cleanup()

    def test_unset_is_what_bt_runner_starts_with(self):
        self.assertTrue(ops.is_unset("dock_heading_deg", -1.0))
        self.assertTrue(ops.is_unset("dock_heading_deg", None))
        self.assertTrue(ops.is_unset("dock_heading_deg", "junk"))
        self.assertFalse(ops.is_unset("dock_heading_deg", 0.0))      # north is a heading
        self.assertFalse(ops.is_unset("dock_heading_deg", "60"))
        self.assertTrue(ops.is_unset("search_first", ""))
        self.assertFalse(ops.is_unset("search_first", "left"))

    def test_saved_values_come_back_after_a_restart(self):
        ops.Store(self.path).remember({"dock_heading_deg": 60.0, "search_first": "left"}, now=1e9)
        store = ops.Store(self.path)                     # a new gcs_bridge reads the file
        self.assertEqual(store.to_restore(UNSET), {"dock_heading_deg": 60.0, "search_first": "left"})
        self.assertTrue(store.set_at("dock_heading_deg"))

    def test_a_set_value_is_never_overwritten(self):
        store = ops.Store(self.path)
        store.remember({"dock_heading_deg": 60.0, "search_first": "left"})
        live = {"dock_heading_deg": 90.0, "search_first": ""}
        self.assertEqual(store.to_restore(live), {"search_first": "left"})

    def test_only_its_keys_are_kept(self):
        store = ops.Store(self.path)
        kept = store.remember({"nav_orbit_radius_m": 3.0, "dock_heading_deg": 45.0})
        self.assertEqual(kept, ["dock_heading_deg"])
        with open(self.path, encoding="utf-8") as f:
            self.assertEqual(set(json.load(f)), {"dock_heading_deg"})
        self.assertEqual(store.remember({"nav_orbit_radius_m": 3.0}), [])

    def test_the_operator_clearing_a_value_stops_its_restore(self):
        store = ops.Store(self.path)
        store.remember({"dock_heading_deg": 60.0, "search_first": "left"})
        store.remember({"dock_heading_deg": -1.0})
        self.assertEqual(ops.Store(self.path).to_restore(UNSET), {"search_first": "left"})

    def test_a_key_the_node_does_not_report_is_left(self):
        store = ops.Store(self.path)
        store.remember({"dock_heading_deg": 60.0, "search_first": "left"})
        self.assertEqual(store.to_restore({"search_first": ""}), {"search_first": "left"})

    def test_a_missing_or_broken_file_is_empty(self):
        self.assertEqual(ops.Store(self.path).to_restore(UNSET), {})
        os.makedirs(os.path.dirname(self.path))
        with open(self.path, "w", encoding="utf-8") as f:
            f.write("{not json")
        self.assertEqual(ops.Store(self.path).to_restore(UNSET), {})
        with open(self.path, "w", encoding="utf-8") as f:
            json.dump({"dock_heading_deg": 60.0, "x": {"value": 1}}, f)   # not {value, set_t}
        self.assertEqual(ops.Store(self.path).to_restore(UNSET), {})


    def test_mission_inputs_unset_at_their_defaults(self):
        self.assertTrue(ops.is_unset("t3_backoff_m", 4.0))
        self.assertTrue(ops.is_unset("t3_backoff_m", 4))              # a browser's 4 is a double's 4.0
        self.assertFalse(ops.is_unset("t3_backoff_m", 5.0))
        self.assertTrue(ops.is_unset("t1_field_lat", 0.0))
        self.assertFalse(ops.is_unset("t1_field_lat", 1.2805))
        self.assertFalse(ops.is_unset("t1_pause_every_m", 0.0))       # 0 = no pauses, a choice
        self.assertTrue(ops.is_unset("t1_pause_every_m", 5.0))

    def test_mission_inputs_come_back_after_a_restart(self):
        ops.Store(self.path).remember({"t3_backoff_m": 5.0, "t1_field_lat": 1.2805,
                                       "t1_field_lon": 103.8557, "t1_pause_every_m": 0.0})
        live = {"t3_backoff_m": 4.0, "t1_field_lat": 0.0, "t1_field_lon": 0.0,
                "t1_pause_every_m": 5.0, "t3_via_lat": 0.0}
        self.assertEqual(ops.Store(self.path).to_restore(live),
                         {"t3_backoff_m": 5.0, "t1_field_lat": 1.2805, "t1_field_lon": 103.8557,
                          "t1_pause_every_m": 0.0})

    def test_task1_approach_and_unknown_range(self):
        self.assertTrue(ops.is_unset("t1_approach_lat", 0.0))
        self.assertFalse(ops.is_unset("t1_approach_lon", 103.8557))
        self.assertTrue(ops.is_unset("t1_unknown_range_m", 12))       # a browser's 12 is a double's 12.0
        self.assertFalse(ops.is_unset("t1_unknown_range_m", 0.0))     # 0 = never look, a choice
        ops.Store(self.path).remember({"t1_approach_lat": 1.2805, "t1_approach_lon": 103.8557,
                                       "t1_unknown_range_m": 0.0})
        live = {"t1_approach_lat": 0.0, "t1_approach_lon": 0.0, "t1_unknown_range_m": 12.0}
        self.assertEqual(ops.Store(self.path).to_restore(live),
                         {"t1_approach_lat": 1.2805, "t1_approach_lon": 103.8557,
                          "t1_unknown_range_m": 0.0})

    def test_task1_approach_and_unknown_range_never_overwrite_a_set_value(self):
        ops.Store(self.path).remember({"t1_approach_lat": 1.2805, "t1_unknown_range_m": 20.0})
        live = {"t1_approach_lat": 1.3, "t1_unknown_range_m": 5.0}
        self.assertEqual(ops.Store(self.path).to_restore(live), {})

    def test_task4_settings_are_never_kept(self):
        store = ops.Store(self.path)
        self.assertEqual(store.remember({"t4_enabled": False, "t4_margin_m": 6.0}), [])
        self.assertEqual(store.to_restore({"t4_enabled": True, "t4_margin_m": 3.0}), {})

    def test_describe(self):
        self.assertEqual(ops.describe("t3_backoff_m", 5), "back off 5 m")
        self.assertEqual(ops.describe("dock_heading_deg", 60.0), "dock heading 60°")
        self.assertEqual(ops.describe("search_first", "left"), "search first left")
        self.assertEqual(ops.describe("t1_field_lat", 1.28), "field centre lat 1.2800000")
        self.assertEqual(ops.describe("t1_approach_lon", 103.8), "Task 1 approach lon 103.8000000")
        self.assertEqual(ops.describe("t1_unknown_range_m", 12.0), "look at unnamed buoys within 12 m")

    # ---- Task 3 control: one list in three places (t3_tuning.hpp, here, the page's rows)

    def test_task3_control_is_kept_and_comes_back(self):
        store = ops.Store(self.path)
        store.remember({"t3_app_kp_lat": 300.0, "t3_lat_offset_m": 0.2})
        live = dict(ops.T3_CONTROL)
        self.assertEqual(ops.Store(self.path).to_restore(live),
                         {"t3_app_kp_lat": 300.0, "t3_lat_offset_m": 0.2})
        store.remember({"t3_app_kp_lat": ops.T3_CONTROL["t3_app_kp_lat"]})   # back to the default
        self.assertEqual(ops.Store(self.path).to_restore(live), {"t3_lat_offset_m": 0.2})

    def test_task3_control_matches_bt_runner(self):
        hpp = os.path.join(os.path.dirname(__file__), "..", "..", "crusader_bt", "include",
                           "crusader_bt", "t3_tuning.hpp")
        if not os.path.exists(hpp):
            self.skipTest("crusader_bt is not beside this package")
        with open(hpp, encoding="utf-8") as f:
            knobs = {k: float(v) for k, v in re.findall(r'\{"(t3_\w+)", ([-0-9.]+),', f.read())}
        self.assertEqual(knobs, ops.T3_CONTROL)

    def test_task3_control_matches_the_page(self):
        page = os.path.join(os.path.dirname(__file__), "..", "crusader_groundstation", "laptop",
                            "page.html")
        with open(page, encoding="utf-8") as f:
            rows = {k: float(v) for k, v in
                    re.findall(r't3k\("(t3_\w+)", "\w+", "[^"]*", "[^"]*", ([-0-9.]+),', f.read())}
        self.assertEqual(rows, ops.T3_CONTROL)


if __name__ == "__main__":
    unittest.main()
