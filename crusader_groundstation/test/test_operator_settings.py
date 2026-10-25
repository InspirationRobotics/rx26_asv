"""operator_settings off-boat: what is saved, what is set back, and what is never touched.

    python3 -m pytest crusader_groundstation/test/test_operator_settings.py
    python3 -m unittest discover -s crusader_groundstation/test        (no pytest needed)

Needs nothing outside the standard library.
"""
import json
import os
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

    def test_only_its_two_keys_are_kept(self):
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


if __name__ == "__main__":
    unittest.main()
