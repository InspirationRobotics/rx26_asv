"""planner_profiles off-boat: parsing, listing, drift and save, against temp directories.

    python3 -m pytest crusader_groundstation/test/test_planner_profiles.py
    python3 -m unittest discover -s crusader_groundstation/test        (no pytest needed)

Needs PyYAML, nothing else: the module is the pure half of the Tuning tab's profile picker.
"""
import os
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

try:
    import yaml  # noqa: F401
except ImportError:
    raise unittest.SkipTest("PyYAML not installed (it is in asv, crsd-sim and WSL)")

from crusader_groundstation import planner_profiles as pp  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SHIPPED_DIR = os.path.join(REPO, "crusader_sim", "config", "tuning_profiles")

MIXED = """\
# title: Mixed
# about: planner keys, a Nav2 key, a stray bt_runner key
bt_runner_node:
  ros__parameters:
    nav_orbit_radius_m: 3.0
    nav_orbit_points: 12
    publish_setpoints: true
    nav_mode: "on"
planner_server:
  ros__parameters:
    GridBased:
      tolerance: 0.3
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 1.2
"""


def row(name, value, default, **kw):
    r = {"name": name, "type": "double", "value": value, "default": default,
         "in_yaml": default is not None, "editable": True}
    r.update(kw)
    return r


class SplitProfile(unittest.TestCase):
    def test_planner_keys_applied_everything_else_skipped_with_a_reason(self):
        import yaml
        values, skipped = pp.split_profile(yaml.safe_load(MIXED))
        self.assertEqual(values, {"nav_orbit_radius_m": 3.0, "nav_orbit_points": 12})
        why = {s["key"]: s["why"] for s in skipped}
        self.assertEqual(set(why), {"bt_runner_node.publish_setpoints", "bt_runner_node.nav_mode",
                                    "planner_server.GridBased.tolerance",
                                    "global_costmap.global_costmap.inflation_layer.inflation_radius"})
        self.assertIn("not a nav_* planner knob", why["bt_runner_node.publish_setpoints"])
        self.assertIn("not a number", why["bt_runner_node.nav_mode"])
        self.assertIn("rig restart", why["planner_server.GridBased.tolerance"])

    def test_empty_and_non_mapping_documents(self):
        self.assertEqual(pp.split_profile(None), ({}, []))
        self.assertEqual(pp.split_profile(["a"]), ({}, []))


class ShippedProfile(unittest.TestCase):
    @unittest.skipUnless(os.path.isdir(SHIPPED_DIR), "crusader_sim not beside this package")
    def test_tight_profile_reads_in_full(self):
        r = pp.read_profile(os.path.join(SHIPPED_DIR, "tight_3to5m.yaml"), pp.SHIPPED)
        self.assertEqual(r["error"], "")
        self.assertEqual(r["title"], "Tight field (3-5 m between buoys)")
        self.assertEqual(len(r["values"]), 19)
        self.assertEqual(r["values"]["nav_orbit_radius_m"], 3.0)
        self.assertEqual(r["values"]["nav_orbit_points"], 12)
        keys = {s["key"] for s in r["skipped"]}
        self.assertIn("planner_server.GridBased.tolerance", keys)
        self.assertIn("global_costmap.global_costmap.inflation_layer.inflation_radius", keys)
        self.assertEqual(len(r["skipped"]), 4)


class Listing(unittest.TestCase):
    def test_missing_directory_is_a_blank_list_with_the_reason(self):
        with tempfile.TemporaryDirectory() as d:
            gone = os.path.join(d, "nope")
            out = pp.listing([(pp.SHIPPED, gone)])
        self.assertEqual(out["profiles"], [])
        self.assertFalse(out["sources"][0]["ok"])
        self.assertIn("does not exist", out["sources"][0]["reason"])
        self.assertIn(gone, out["sources"][0]["reason"])

    def test_blank_directory_says_not_configured(self):
        out = pp.listing([(pp.SAVED, "")])
        self.assertEqual(out["sources"][0]["reason"], "no directory configured")

    def test_two_directories_sorted_and_a_bad_file_listed_with_its_error(self):
        with tempfile.TemporaryDirectory() as a, tempfile.TemporaryDirectory() as b:
            for d, name, text in ((a, "zeta", MIXED), (a, "alpha", MIXED), (b, "alpha", MIXED),
                                  (a, "broken", "bt_runner_node: [unclosed"),
                                  (a, "not a name", MIXED), (a, "notes.txt", "x")):
                with open(os.path.join(d, name + (".yaml" if "." not in name else "")), "w") as f:
                    f.write(text)
            out = pp.listing([(pp.SHIPPED, a), (pp.SAVED, b)])
        self.assertEqual([(r["name"], r["origin"]) for r in out["profiles"]],
                         [("alpha", "saved"), ("alpha", "shipped"), ("broken", "shipped"),
                          ("zeta", "shipped")])
        broken = [r for r in out["profiles"] if r["name"] == "broken"][0]
        self.assertNotEqual(broken["error"], "")
        self.assertEqual(broken["values"], {})
        self.assertTrue(all(s["ok"] for s in out["sources"]))


class Drift(unittest.TestCase):
    def test_only_changed_numeric_editable_planner_keys(self):
        rows = [row("nav_hard_m", 1.0, 0.8),
                row("nav_soft_m", 2.0, 2.0),
                row("nav_orbit_points", 12, 8, type="integer"),
                row("nav_hazard_rate_hz", 5.0, 2.0, editable=False),       # structural: read-only
                row("nav_mode", "on", "off", type="string"),
                row("tick_hz", 20.0, 10.0),                                # not a planner key
                row("nav_new_knob", 1.0, None)]                            # no YAML default
        values, note = pp.drifted_planner_values(rows)
        self.assertEqual(values, {"nav_hard_m": 1.0, "nav_orbit_points": 12})
        self.assertEqual(note, "")

    def test_float_noise_is_not_drift_and_values_are_tidied(self):
        values, _ = pp.drifted_planner_values([row("nav_soft_m", 2.0 + 1e-12, 2.0),
                                               row("nav_hard_m", 0.30000000000000004, 0.8)])
        self.assertEqual(values, {"nav_hard_m": 0.3})

    def test_nothing_to_compare_against_says_so_not_nothing_changed(self):
        values, note = pp.drifted_planner_values([row("nav_hard_m", 1.0, None)])
        self.assertEqual(values, {})
        self.assertIn("nothing to compare", note)

    def test_no_planner_knobs_says_so(self):
        values, note = pp.drifted_planner_values([row("tick_hz", 20.0, 10.0)])
        self.assertEqual(values, {})
        self.assertIn("no editable nav_*", note)


class Save(unittest.TestCase):
    def test_round_trip_in_the_shipped_format(self):
        with tempfile.TemporaryDirectory() as d:
            save_dir = os.path.join(d, "a", "tuning")                # created on demand
            ok, message, path = pp.save(save_dir, "dock_day", {"nav_soft_m": 1.2, "nav_orbit_points": 12})
            self.assertTrue(ok, message)
            self.assertEqual(path, os.path.join(save_dir, "dock_day.yaml"))
            with open(path, encoding="utf-8") as f:
                text = f.read()
            self.assertTrue(text.startswith("# title: dock_day\n# about: Saved from the ground station"))
            self.assertIn("bt_runner_node:\n  ros__parameters:\n    nav_orbit_points: 12\n    nav_soft_m: 1.2\n", text)
            r = pp.read_profile(path, pp.SAVED)
            self.assertEqual((r["error"], r["title"], r["values"], r["skipped"]),
                             ("", "dock_day", {"nav_soft_m": 1.2, "nav_orbit_points": 12}, []))
            self.assertEqual(os.listdir(save_dir), ["dock_day.yaml"])    # no .tmp left behind

    def test_overwrite_replaces(self):
        with tempfile.TemporaryDirectory() as d:
            pp.save(d, "x", {"nav_soft_m": 1.0})
            ok, _, path = pp.save(d, "x", {"nav_hard_m": 0.9})
            self.assertTrue(ok)
            self.assertEqual(pp.read_profile(path, pp.SAVED)["values"], {"nav_hard_m": 0.9})

    def test_a_name_can_never_be_a_path(self):
        with tempfile.TemporaryDirectory() as d:
            for bad in ("../x", "a/b", "a\\b", "", " ", "x" * 41, "x.yaml", None, 7):
                ok, message, path = pp.save(d, bad, {"nav_soft_m": 1.0})
                self.assertFalse(ok, bad)
                self.assertEqual(path, "")
            self.assertEqual(os.listdir(d), [])

    def test_nothing_to_save_and_unwritable_directory_are_refused_in_words(self):
        with tempfile.TemporaryDirectory() as d:
            self.assertEqual(pp.save(d, "x", {})[:2], (False, "nothing to save"))
            blocker = os.path.join(d, "file")
            with open(blocker, "w") as f:
                f.write("x")
            ok, message, _ = pp.save(os.path.join(blocker, "sub"), "x", {"nav_soft_m": 1.0})
            self.assertFalse(ok)
            self.assertIn("could not write", message)


if __name__ == "__main__":
    unittest.main()
