"""Offline tests for the Task 1 panel's Planner tuning: the catalogue read from the two YAML files, the
override validation, tuning.yaml (nested, atomic, deleted when empty), the launch argv, and the routes.

No ROS, no sim, no gz. Needs python3 with PyYAML (the WSL host has it):

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest discover -s test -p "test_tuning.py" -v

The panel is built on a private state dir and a private copy of the two YAML files; Proc is replaced by a
stand-in that only records its argv, so nothing is launched and no radio is opened.
"""
import json
import os
import shutil
import subprocess
import sys
import tempfile
import threading
import unittest
import urllib.error
import urllib.request
from http.server import ThreadingHTTPServer
from types import SimpleNamespace as NS
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lake_testkit as K                                                      # noqa: E402  (also sets sys.path)

import yaml                                                                   # noqa: E402

from crusader_sim import task1_panel as T                                     # noqa: E402

BT_YAML = """\
# a comment block at the top
shared:
  ros__parameters:
    nav_hard_m: 5.0                         # another node's key of the same name: never in the catalogue
bt_runner_node:
  ros__parameters:
    tree_file: "/x/y.xml"                   # [RO] a string
    tick_hz: 10.0                           # [RO] a number, but not nav_*
    nav_mode: "off"                         # [RO] a string: excluded
                                            #      (and its continuation)
    nav_enabled: true                       # [RO] a bool: excluded
    nav_zones: [1, 2, 3]                    # a list: excluded
    nav_hard_m: 0.8                         # [RO] "0.8 m hard": never closer than
                                            #      this to a hazard's surface. At
                                            #      most robot_radius (check_config)
    nav_soft_m: 2                           # [DYN] "2 m soft", written as an int
    # ---- a section comment at the keys' own indent belongs to no key above ----
    nav_gate_clear_m: 1.5                   # [RO] the gate rule
                                            #      second line

                                            #      after a blank line: not part of it
    nav_goal_max_move_m: 3.0
    nav_orbit_clear_m: 1.4                  # [RO]
    nav_buoy_radius_m: 0.30                 # a quote " in a comment, and a # too
    nav_clip_radius_m: 35.0
    nav_dock_finger_len_m: 2.0              # [RO] build guide: dock fingers
    nav_fence_len_m: 10.0
    nav_hysteresis_frac: 0.2
    nav_invalid_confirm: 2                  # [RO] needs this many
    nav_offset_m: -0.5                      # a negative default
    nav_period_s: 0.5
    nav_rate_hz: 2.0
    nav_turn_deg: 45.0
    nav_lookahead_m: 5.0
rxl_link_node:
  ros__parameters:
    nav_other_m: 1.0
"""

NAV_YAML = """\
# Crusader Nav2
planner_server:
  ros__parameters:
    expected_planner_frequency: 2.0          # warn if one plan takes >0.5 s
    planner_plugins: ["GridBased"]
    GridBased:
      plugin: "nav2_smac_planner/SmacPlanner2D"
      tolerance: 0.5                         # m; only used when the exact goal is unreachable
      downsample_costmap: false
      downsampling_factor: 1
      allow_unknown: true
      max_iterations: 1000000
      smoother:
        max_iterations: 1000
        w_smooth: 0.3
        tolerance: 1.0e-10
global_costmap:
  global_costmap:
    ros__parameters:
      global_frame: map
      rolling_window: true
      width: 80                              # m (int)
      # 0.82 * cos(pi/16) = 0.804 m INSCRIBED radius.
      # This IS the hard clearance.
      robot_radius: 0.82
      plugins: ["hazard_layer", "stvl_layer", "inflation_layer"]
      footprint: [[0.5, 0.3], [0.5, -0.3]]
      stvl_layer:
        plugin: "spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer"
        enabled: true
        voxel_decay: 30.0                    # s, linear
        observation_sources: lidar_mark lidar_clear
        lidar_mark:
          marking: true
          # obstacle_range is PER SOURCE
          # (second line)
          obstacle_range: 40.0
          min_obstacle_height: 0.0           # the inline comment wins
      inflation_layer:
        inflation_radius: 2.0                # "2 m soft"
nav_lifecycle:
  ros__parameters:
    check_period_s: 1.0
"""

LAYOUT = [{"x": 10.0, "y": 3.0, "state": "flash_blue"}, {"x": 20.0, "y": -3.0, "state": "flash_red"},
          {"x": 20.0, "y": 3.0, "state": "flash_green"}, {"x": 40.0, "y": 0.0, "state": "steady_blue"}]

HARD = "bt_runner_node/nav_hard_m"
SOFT = "bt_runner_node/nav_soft_m"
CONFIRM = "bt_runner_node/nav_invalid_confirm"
OFFSET = "bt_runner_node/nav_offset_m"
TOL = "planner_server/GridBased.tolerance"
SMOOTH = "planner_server/GridBased.smoother.w_smooth"
INFL = "global_costmap/inflation_layer.inflation_radius"
LIDAR = "global_costmap/stvl_layer.lidar_mark.min_obstacle_height"


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def make_repo(root, bt=BT_YAML, nav=NAV_YAML):
    """A workspace's rx26_asv with just the two YAML files the catalogue reads."""
    if bt is not None:
        write(os.path.join(root, *T.BT_PARAMS), bt)
    if nav is not None:
        write(os.path.join(root, *T.NAV_PARAMS), nav)
    return root


class FakeProc:
    """Stands in for task1_panel.Proc: records the argv, runs nothing. gz_sim_down 'finishes' at once."""
    started = []

    def __init__(self, name, argv, log, on_line=None, on_exit=None):
        self.name, self.argv, self.on_exit = name, list(argv), on_exit

    def start(self):
        FakeProc.started.append(self)
        if self.name == "gz_sim_down.sh" and self.on_exit:
            self.on_exit(0)
        return self

    def running(self):
        return False

    def signal(self, _sig):
        pass


class QuietPanel(T.Panel):
    """The sim's panel with no gz-transport and no sensor hub, on a private state dir."""

    def _panel_dir(self):
        return self.a.panel_dir

    def _init_gz(self):
        pass

    def _init_sensors(self):
        pass


class PanelCase(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="tuning_test_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.repo = make_repo(os.path.join(self.tmp, "repo"))
        a = NS(dry_run=True, dry_run_mission_s=1.0, rxl_endpoint="udpout:127.0.0.1:%d" % K.free_udp_port(),
               feed_port=K.free_udp_port(), container="none", panel_dir=os.path.join(self.tmp, "panel"),
               src_repo=self.repo)
        self.panel = QuietPanel(a)
        self.addCleanup(self.panel.shutdown)
        self.path = os.path.join(self.panel.dir, "tuning.yaml")
        FakeProc.started = []
        patcher = mock.patch.object(T, "Proc", FakeProc)
        patcher.start()
        self.addCleanup(patcher.stop)

    def set_(self, sets=None, reset=None):
        """POST /api/tuning's body, called directly: {set: {id: n}} and/or {reset: [id] | "all"}."""
        body = {}
        if sets is not None:
            body["set"] = sets
        if reset is not None:
            body["reset"] = reset
        return self.panel.act_tuning(body)

    def written(self):
        with open(self.path, encoding="utf-8") as f:
            return yaml.safe_load(f.read())

    def view(self):
        return self.panel.tuning_view()

    def entry(self, id_):
        return next(e for e in self.view()["catalogue"] if e["id"] == id_)


# ------------------------------------------------------------------ the catalogue

class CommentScanTest(unittest.TestCase):
    def test_inline_comment_ignores_hashes_in_quotes_and_in_words(self):
        self.assertEqual(T._inline_comment('"/a#b"  # real'), "real")
        self.assertEqual(T._inline_comment("http://x/#frag  # c"), "c")
        self.assertEqual(T._inline_comment("nav2's value"), "")
        self.assertEqual(T._inline_comment("1.0"), "")
        self.assertEqual(T._inline_comment("# only a comment"), "only a comment")

    def test_own_is_the_inline_comment_plus_the_deeper_comment_lines_under_it(self):
        n = T.scan_comments(BT_YAML)
        own = lambda k: n[("bt_runner_node", "ros__parameters", k)][0]            # noqa: E731
        self.assertEqual(own("nav_hard_m"), "[RO] \"0.8 m hard\": never closer than this to a hazard's "
                                            "surface. At most robot_radius (check_config)")
        self.assertEqual(own("nav_gate_clear_m"), "[RO] the gate rule second line")   # the blank line ends it
        self.assertEqual(own("nav_goal_max_move_m"), "")
        self.assertEqual(own("nav_soft_m"), '[DYN] "2 m soft", written as an int')    # the section comment is not its

    def test_before_is_the_comment_block_right_above_at_the_keys_indent(self):
        n = T.scan_comments(NAV_YAML)
        base = ("global_costmap", "global_costmap", "ros__parameters")
        self.assertEqual(n[base + ("robot_radius",)][1],
                         "0.82 * cos(pi/16) = 0.804 m INSCRIBED radius. This IS the hard clearance.")
        self.assertEqual(n[base + ("robot_radius",)][0], "")
        self.assertEqual(n[base + ("stvl_layer", "lidar_mark", "obstacle_range")][1],
                         "obstacle_range is PER SOURCE (second line)")
        self.assertEqual(n[base + ("width",)], ("m (int)", ""))

    def test_the_path_follows_the_indentation(self):
        n = T.scan_comments(NAV_YAML)
        self.assertIn(("planner_server", "ros__parameters", "GridBased", "smoother", "w_smooth"), n)
        self.assertIn(("nav_lifecycle", "ros__parameters", "check_period_s"), n)
        self.assertNotIn(("planner_server", "ros__parameters", "w_smooth"), n)


class CatalogueTest(unittest.TestCase):
    def setUp(self):
        self.root = tempfile.mkdtemp(prefix="tuning_cat_")
        self.addCleanup(shutil.rmtree, self.root, True)
        make_repo(self.root)
        self.entries, self.errors = T.tuning_catalogue(self.root)
        self.by_id = {e["id"]: e for e in self.entries}

    def test_only_numeric_nav_keys_of_bt_runner_node_and_numeric_leaves_of_nav2(self):
        self.assertEqual(self.errors, [])
        ids = sorted(self.by_id)
        self.assertEqual([i for i in ids if i.startswith("bt_runner_node/")], sorted(
            "bt_runner_node/" + k for k in (
                "nav_hard_m", "nav_soft_m", "nav_gate_clear_m", "nav_goal_max_move_m", "nav_orbit_clear_m",
                "nav_buoy_radius_m", "nav_clip_radius_m", "nav_dock_finger_len_m", "nav_fence_len_m",
                "nav_hysteresis_frac", "nav_invalid_confirm", "nav_offset_m", "nav_period_s", "nav_rate_hz",
                "nav_turn_deg", "nav_lookahead_m")))
        self.assertEqual([i for i in ids if i.startswith("planner_server/")], sorted(
            "planner_server/" + k for k in (
                "expected_planner_frequency", "GridBased.tolerance", "GridBased.downsampling_factor",
                "GridBased.max_iterations", "GridBased.smoother.max_iterations", "GridBased.smoother.w_smooth",
                "GridBased.smoother.tolerance")))
        self.assertEqual([i for i in ids if i.startswith("global_costmap/")], sorted(
            "global_costmap/" + k for k in (
                "width", "robot_radius", "stvl_layer.voxel_decay", "stvl_layer.lidar_mark.obstacle_range",
                "stvl_layer.lidar_mark.min_obstacle_height", "inflation_layer.inflation_radius")))

    def test_bools_strings_lists_other_nodes_and_non_nav_keys_are_left_out(self):
        for gone in ("bt_runner_node/nav_mode", "bt_runner_node/nav_enabled", "bt_runner_node/nav_zones",
                     "bt_runner_node/tick_hz", "bt_runner_node/tree_file", "shared/nav_hard_m",
                     "rxl_link_node/nav_other_m", "planner_server/GridBased.allow_unknown",
                     "planner_server/GridBased.plugin", "planner_server/planner_plugins",
                     "global_costmap/rolling_window", "global_costmap/footprint", "global_costmap/plugins",
                     "global_costmap/stvl_layer.enabled", "global_costmap/stvl_layer.observation_sources",
                     "global_costmap/stvl_layer.lidar_mark.marking", "nav_lifecycle/check_period_s"):
            self.assertNotIn(gone, self.by_id)

    def test_entry_fields(self):
        e = self.by_id[HARD]
        self.assertEqual({k: e[k] for k in ("id", "node", "key", "default", "type", "unit", "group")},
                         {"id": HARD, "node": "bt_runner_node", "key": "nav_hard_m", "default": 0.8,
                          "type": "float", "unit": "m", "group": "Hazards"})
        self.assertEqual(e["desc"], "\"0.8 m hard\": never closer than this to a hazard's surface. "
                                    "At most robot_radius (check_config)")                  # [RO] stripped
        self.assertEqual(self.by_id[SOFT]["desc"], '"2 m soft", written as an int')          # [DYN] stripped
        self.assertEqual((self.by_id[SOFT]["type"], self.by_id[SOFT]["default"]), ("int", 2))
        self.assertEqual(self.by_id[CONFIRM]["type"], "int")
        self.assertEqual(self.by_id[CONFIRM]["unit"], "")
        self.assertEqual(self.by_id["bt_runner_node/nav_orbit_clear_m"]["desc"], "")          # a bare [RO]
        self.assertEqual(self.by_id["bt_runner_node/nav_buoy_radius_m"]["desc"], 'a quote " in a comment, and a # too')

    def test_units_come_from_the_suffix(self):
        unit = lambda k: self.by_id["bt_runner_node/" + k]["unit"]                    # noqa: E731
        self.assertEqual([unit(k) for k in ("nav_period_s", "nav_rate_hz", "nav_turn_deg", "nav_hard_m",
                                            "nav_hysteresis_frac")], ["s", "Hz", "deg", "m", ""])
        self.assertEqual(self.by_id[INFL]["unit"], "")                                # inflation_radius: no suffix

    def test_nested_nav2_keys_are_dotted_and_keep_their_comments(self):
        t = self.by_id[TOL]
        self.assertEqual((t["node"], t["key"], t["default"], t["group"]),
                         ("planner_server", "GridBased.tolerance", 0.5, "Nav2 planner"))
        self.assertEqual(t["desc"], "m; only used when the exact goal is unreachable")
        s = self.by_id["planner_server/GridBased.smoother.tolerance"]
        self.assertEqual((s["default"], s["type"], s["desc"]), (1e-10, "float", ""))
        c = self.by_id[LIDAR]
        self.assertEqual((c["node"], c["group"], c["desc"]), ("global_costmap", "Nav2 costmap", "the inline comment wins"))
        self.assertEqual(self.by_id["global_costmap/stvl_layer.lidar_mark.obstacle_range"]["desc"],
                         "obstacle_range is PER SOURCE (second line)")                # the comment above, no inline one
        self.assertEqual(self.by_id["global_costmap/robot_radius"]["desc"],
                         "0.82 * cos(pi/16) = 0.804 m INSCRIBED radius. This IS the hard clearance.")
        self.assertEqual(self.by_id["global_costmap/width"]["type"], "int")
        self.assertEqual(self.by_id["global_costmap/stvl_layer.voxel_decay"]["desc"], "s, linear")

    def test_grouping(self):
        g = {k.split("/", 1)[1]: e["group"] for k, e in self.by_id.items() if k.startswith("bt_runner_node/")}
        self.assertEqual(g["nav_gate_clear_m"], "Goals")
        self.assertEqual(g["nav_goal_max_move_m"], "Goals")
        self.assertEqual(g["nav_orbit_clear_m"], "Goals")
        for k in ("nav_hard_m", "nav_soft_m", "nav_buoy_radius_m", "nav_clip_radius_m"):
            self.assertEqual(g[k], "Hazards", k)
        self.assertEqual(g["nav_fence_len_m"], "Fences")
        self.assertEqual(g["nav_dock_finger_len_m"], "Task 3 dock")
        for k in ("nav_hysteresis_frac", "nav_invalid_confirm", "nav_offset_m", "nav_period_s", "nav_lookahead_m"):
            self.assertEqual(g[k], "Leg following", k)
        self.assertEqual(self.by_id[TOL]["group"], "Nav2 planner")
        self.assertEqual(self.by_id[INFL]["group"], "Nav2 costmap")

    def test_the_hazard_keys_in_the_spec_are_hazards_and_nothing_else_is(self):
        self.assertEqual(sorted(T.BT_HAZARD_KEYS), sorted((
            "nav_hard_m", "nav_soft_m", "nav_buoy_radius_m", "nav_track_radius_m", "nav_exempt_radius_m",
            "nav_local_check_tol_m", "nav_escape_margin_m", "nav_clip_radius_m")))
        self.assertEqual(T.bt_group("nav_goal_replan_m"), "Goals")       # nav_goal_* is Goals even when it is a replan gap
        self.assertEqual(T.bt_group("nav_track_radius_m"), "Hazards")
        self.assertEqual(T.bt_group("nav_dock_x_m"), "Task 3 dock")

    def test_entries_come_in_group_order(self):
        order = [T.TUNING_GROUPS.index(e["group"]) for e in self.entries]
        self.assertEqual(order, sorted(order))
        self.assertEqual(T.TUNING_GROUPS, ("Goals", "Hazards", "Fences", "Leg following", "Task 3 dock",
                                           "Nav2 planner", "Nav2 costmap"))

    def test_a_missing_or_broken_file_is_an_error_and_not_a_crash(self):
        root = tempfile.mkdtemp(prefix="tuning_cat_")
        self.addCleanup(shutil.rmtree, root, True)
        make_repo(root, bt=None, nav="planner_server: [unclosed\n")
        entries, errors = T.tuning_catalogue(root)
        self.assertEqual(entries, [])
        self.assertEqual(len(errors), 2)
        self.assertTrue(errors[0].startswith("crusader_params.yaml:"), errors)
        self.assertTrue(errors[1].startswith("nav2_params.yaml:"), errors)

    def test_it_is_read_from_disk_every_time(self):
        edited = BT_YAML.replace("nav_hard_m: 0.8 ", "nav_hard_m: 0.9 ").replace(
            "    nav_lookahead_m: 5.0\n", "    nav_lookahead_m: 5.0\n    nav_brand_new_m: 1.5     # [RO] added by a colleague\n")
        make_repo(self.root, bt=edited)
        by_id = {e["id"]: e for e in T.tuning_catalogue(self.root)[0]}
        self.assertEqual(by_id[HARD]["default"], 0.9)
        self.assertEqual(by_id["bt_runner_node/nav_brand_new_m"]["desc"], "added by a colleague")
        self.assertEqual(by_id["bt_runner_node/nav_brand_new_m"]["group"], "Leg following")

    def test_the_real_files_give_a_sane_catalogue(self):
        entries, errors = T.tuning_catalogue(T._SRC_REPO)
        self.assertEqual(errors, [])
        ids = [e["id"] for e in entries]
        self.assertEqual(len(ids), len(set(ids)))
        self.assertGreater(len(ids), 40)
        self.assertIn(HARD, ids)
        for e in entries:
            self.assertIn(e["group"], T.TUNING_GROUPS)
            self.assertIn(e["type"], ("int", "float"))
            self.assertIs(type(e["default"]), int if e["type"] == "int" else float, e["id"])
            self.assertNotIsInstance(e["default"], bool)
        self.assertTrue(all(e["key"].startswith("nav_") for e in entries if e["node"] == "bt_runner_node"))


# ------------------------------------------------------------------ validation

class ValidationTest(PanelCase):
    def bad(self, id_, v, text):
        r = self.set_({id_: v})
        self.assertFalse(r["ok"], (id_, v))
        self.assertIn(text, r["error"])
        self.assertFalse(os.path.exists(self.path), "nothing is written on an error")

    def test_not_a_real_number(self):
        for v in (True, False, None, "0.5", [1], {"a": 1}, float("nan"), float("inf"), float("-inf"), 10 ** 400):
            self.bad(HARD, v, "finite number")

    def test_non_integral_for_an_int_entry(self):
        self.bad(CONFIRM, 2.5, "whole number")
        self.bad(SOFT, 0.5, "whole number")

    def test_negative_where_the_default_is_not_negative(self):
        self.bad(HARD, -0.1, "must not be negative")
        self.bad(TOL, -1, "must not be negative")
        self.bad("global_costmap/width", -80, "must not be negative")

    def test_zero_is_fine_and_a_negative_default_allows_a_negative_value(self):
        self.assertTrue(self.set_({HARD: 0})["ok"])
        self.assertTrue(self.set_({OFFSET: -2.5})["ok"])
        self.assertEqual(self.written()["bt_runner_node"]["ros__parameters"], {"nav_hard_m": 0.0, "nav_offset_m": -2.5})

    def test_unknown_id_and_malformed_bodies(self):
        self.bad("bt_runner_node/nav_nope_m", 1.0, "unknown parameter")
        self.bad("bt_runner_node/nav_mode", 1.0, "unknown parameter")            # a string key is not in the catalogue
        for body, text in (({"setz": {}}, "unknown field"), ({"set": [1]}, "object"), ({"reset": "most"}, "list of ids"),
                           ({"reset": [3]}, "list of ids"), ({"reset": {"a": 1}}, "list of ids")):
            r = self.panel.act_tuning(body)
            self.assertFalse(r["ok"], body)
            self.assertIn(text, r["error"])
        self.assertFalse(os.path.exists(self.path))

    def test_one_bad_value_writes_nothing_at_all_and_leaves_an_old_file_alone(self):
        self.assertTrue(self.set_({HARD: 0.9})["ok"])
        before = read(self.path)
        r = self.set_({SOFT: 3, TOL: float("nan")})
        self.assertFalse(r["ok"])
        self.assertEqual(read(self.path), before)
        r = self.set_({SOFT: 3, CONFIRM: 1.5})
        self.assertFalse(r["ok"])
        self.assertEqual(read(self.path), before)

    def test_values_are_stored_in_the_entrys_own_type(self):
        self.assertTrue(self.set_({CONFIRM: 3.0, HARD: 1, TOL: 2})["ok"])
        w = self.written()
        self.assertIs(type(w["bt_runner_node"]["ros__parameters"]["nav_invalid_confirm"]), int)
        self.assertIs(type(w["bt_runner_node"]["ros__parameters"]["nav_hard_m"]), float)   # rcl will not take 1 for a double
        self.assertIs(type(w["planner_server"]["ros__parameters"]["GridBased"]["tolerance"]), float)
        self.assertIn("nav_hard_m: 1.0", read(self.path))


# ------------------------------------------------------------------ tuning.yaml

class FileTest(PanelCase):
    def test_nested_exactly_like_the_source_files(self):
        r = self.set_({HARD: 0.9, TOL: 0.7, INFL: 2.5, LIDAR: 0.2, SMOOTH: 0.4})
        self.assertEqual(r, {"ok": True, "saved": 5})
        self.assertEqual(self.written(), {
            "bt_runner_node": {"ros__parameters": {"nav_hard_m": 0.9}},
            "planner_server": {"ros__parameters": {"GridBased": {"tolerance": 0.7, "smoother": {"w_smooth": 0.4}}}},
            "global_costmap": {"global_costmap": {"ros__parameters": {
                "inflation_layer": {"inflation_radius": 2.5},
                "stvl_layer": {"lidar_mark": {"min_obstacle_height": 0.2}}}}}})

    def test_the_file_says_who_wrote_it_and_where_it_applies(self):
        self.set_({HARD: 0.9})
        header = [ln for ln in read(self.path).splitlines() if ln.startswith("#")]
        self.assertTrue(header[0].startswith("# Written by the Task 1 panel"))
        for word in ("SIM ONLY", "LAUNCH", "crusader_params.yaml", "nav2_params.yaml", "not changed"):
            self.assertIn(word, "\n".join(header))

    def test_it_round_trips_through_yaml_and_the_panels_own_reader(self):
        saved = {HARD: 0.9, SOFT: 3, CONFIRM: 4, TOL: 0.7, SMOOTH: 1e-5, INFL: 2.5, LIDAR: 0.2, OFFSET: -1.5}
        text = T.tuning_text(saved)
        nested = yaml.safe_load(text)
        flat = {}
        for node, where in T.TUNING_PATHS.items():
            for kt, v in T._numeric_leaves(T._section(nested, where)):
                flat["%s/%s" % (node, ".".join(kt))] = v
        self.assertEqual(flat, saved)
        write(self.path, text)
        self.assertEqual(T.read_tuning(self.path)[0], saved)
        self.assertEqual(self.view()["saved"], saved)
        self.assertEqual(self.view()["yaml"], text)

    def test_floats_stay_floats_so_the_params_loader_takes_them(self):
        text = T.tuning_text({HARD: 1.0, TOL: 2.0, "planner_server/GridBased.smoother.tolerance": 1e-10})
        self.assertIn("nav_hard_m: 1.0", text)
        self.assertIn("tolerance: 2.0", text)
        self.assertRegex(text, r"tolerance: 1\.0e-10")
        doc = yaml.safe_load(text)
        self.assertIs(type(doc["bt_runner_node"]["ros__parameters"]["nav_hard_m"]), float)
        self.assertIs(type(doc["planner_server"]["ros__parameters"]["GridBased"]["smoother"]["tolerance"]), float)

    def test_an_override_equal_to_the_default_is_dropped(self):
        self.assertEqual(self.set_({HARD: 0.8}), {"ok": True, "saved": 0})
        self.assertFalse(os.path.exists(self.path))
        self.assertEqual(self.set_({HARD: 0.9, SOFT: 2}), {"ok": True, "saved": 1})        # int default 2 == 2
        self.assertEqual(list(self.written()["bt_runner_node"]["ros__parameters"]), ["nav_hard_m"])
        self.assertEqual(self.set_({HARD: 0.8}), {"ok": True, "saved": 0})                  # back to the default: gone
        self.assertFalse(os.path.exists(self.path))

    def test_an_int_valued_float_equal_to_the_default_is_dropped_too(self):
        self.assertEqual(self.set_({SOFT: 2.0}), {"ok": True, "saved": 0})
        self.assertEqual(self.set_({CONFIRM: 2.0}), {"ok": True, "saved": 0})

    def test_no_overrides_means_no_file(self):
        self.set_({HARD: 0.9, TOL: 0.7})
        self.assertTrue(os.path.isfile(self.path))
        r = self.set_(reset=[HARD, TOL])
        self.assertEqual(r, {"ok": True, "saved": 0})
        self.assertFalse(os.path.exists(self.path))
        self.assertEqual(self.view()["yaml"], "")
        self.assertEqual(self.view()["saved"], {})

    def test_reset_all_and_reset_one(self):
        self.set_({HARD: 0.9, TOL: 0.7, INFL: 2.5})
        self.assertEqual(self.set_(reset=[TOL]), {"ok": True, "saved": 2})
        self.assertEqual(sorted(self.view()["saved"]), sorted((HARD, INFL)))
        self.assertEqual(self.set_(reset="all"), {"ok": True, "saved": 0})
        self.assertFalse(os.path.exists(self.path))
        self.assertEqual(self.set_(reset="all"), {"ok": True, "saved": 0})                  # again: still fine

    def test_edits_merge_with_what_is_saved_and_reset_applies_before_set(self):
        self.set_({HARD: 0.9})
        self.set_({TOL: 0.7})
        self.assertEqual(sorted(self.view()["saved"]), sorted((HARD, TOL)))
        self.set_({HARD: 1.1}, reset=[HARD, TOL])                                           # reset both, then set HARD
        self.assertEqual(self.view()["saved"], {HARD: 1.1})

    def test_the_write_is_atomic_no_temp_file_is_left(self):
        self.set_({HARD: 0.9})
        self.assertEqual(sorted(os.listdir(self.panel.dir)), ["tuning.yaml"])

    def test_unknown_overrides_are_kept_and_reported_then_dropped_by_the_next_save(self):
        write(self.path, T.tuning_text({HARD: 0.9, "bt_runner_node/nav_removed_m": 2.0,
                                        "planner_server/GridBased.gone": 1.0}))
        v = self.view()
        self.assertEqual(v["unknown"], ["bt_runner_node/nav_removed_m", "planner_server/GridBased.gone"])
        self.assertEqual(len(v["saved"]), 3)                                                # kept: still in the file
        self.assertNotIn("bt_runner_node/nav_removed_m", [e["id"] for e in v["catalogue"]])
        self.assertEqual(self.panel.state({})["tuning"]["saved"], 3)
        self.assertEqual(self.set_({TOL: 0.7}), {"ok": True, "saved": 2})                   # dropped on the save
        self.assertEqual(sorted(self.view()["saved"]), sorted((HARD, TOL)))
        self.assertEqual(self.view()["unknown"], [])

    def test_with_a_source_file_unreadable_nothing_is_called_unknown_and_values_are_refused(self):
        self.set_({HARD: 0.9, TOL: 0.7})
        os.remove(os.path.join(self.repo, *T.NAV_PARAMS))
        v = self.view()
        self.assertEqual(v["unknown"], [])
        self.assertEqual(len(v["errors"]), 1)
        self.assertIn("nav2_params.yaml", v["errors"][0])
        r = self.set_({HARD: 1.0})
        self.assertFalse(r["ok"])
        self.assertIn("cannot check", r["error"])
        self.assertEqual(self.set_(reset=[HARD]), {"ok": True, "saved": 1})                 # a reset needs no catalogue ...
        self.assertEqual(list(self.view()["saved"]), [TOL])                                 # ... and the other node's key stays

    def test_a_hand_edited_file_that_does_not_parse_reads_as_none_with_the_reason(self):
        write(self.path, "bt_runner_node: [unclosed\n")
        v = self.view()
        self.assertEqual(v["saved"], {})
        self.assertTrue(any(e.startswith("tuning.yaml:") for e in v["errors"]), v["errors"])
        self.assertEqual(self.panel.state({})["tuning"]["saved"], 0)

    def test_non_numbers_and_foreign_nodes_in_the_file_are_ignored(self):
        write(self.path, "bt_runner_node:\n  ros__parameters:\n    nav_hard_m: 0.9\n    nav_mode: on\n    nav_x: [1]\n"
                         "some_other_node:\n  ros__parameters:\n    nav_hard_m: 7.0\n")
        self.assertEqual(self.view()["saved"], {HARD: 0.9})


# ------------------------------------------------------------------ the catalogue as the page gets it

class ViewTest(PanelCase):
    def test_every_entry_carries_its_current_value_and_whether_it_is_overridden(self):
        self.set_({HARD: 0.9})
        v = self.view()
        h = self.entry(HARD)
        self.assertEqual((h["value"], h["default"], h["overridden"]), (0.9, 0.8, True))
        s = self.entry(SOFT)
        self.assertEqual((s["value"], s["default"], s["overridden"]), (2, 2, False))
        self.assertEqual(v["saved"], {HARD: 0.9})
        self.assertEqual(v["path"], self.path)
        self.assertTrue(os.path.isabs(v["path"]))
        self.assertEqual(v["errors"], [])
        for k in ("catalogue", "groups", "saved", "launched", "pending", "yaml", "unknown", "path"):
            self.assertIn(k, v)

    def test_groups_are_in_display_order_and_only_the_ones_with_entries(self):
        self.assertEqual(self.view()["groups"], list(T.TUNING_GROUPS))
        make_repo(self.repo, bt=BT_YAML.replace("nav_fence_len_m", "nav_gone_a_m").replace("nav_dock_finger_len_m", "nav_gone_b_m"))
        g = self.view()["groups"]
        self.assertEqual(g, ["Goals", "Hazards", "Leg following", "Nav2 planner", "Nav2 costmap"])

    def test_the_view_follows_the_files_on_disk_with_no_caching(self):
        self.assertEqual(self.entry(HARD)["default"], 0.8)
        make_repo(self.repo, bt=BT_YAML.replace("nav_hard_m: 0.8 ", "nav_hard_m: 0.85"))
        self.assertEqual(self.entry(HARD)["default"], 0.85)

    def test_the_view_is_json(self):
        self.set_({HARD: 0.9})
        json.dumps(self.view())


# ------------------------------------------------------------------ launch

class LaunchTest(PanelCase):
    def setUp(self):
        super().setUp()
        self.panel.act_layout({"buoys": LAYOUT})
        self.script = os.path.join(T._SRC_PKG, "scripts", "gz_sim_up.sh")
        self.course_path = os.path.join(self.panel.dir, "panel.yaml")
        self.panel.a.dry_run = False                        # only to see the real command line; Proc is a stand-in

    def launch(self):
        r = self.panel.act_launch({})
        self.assertTrue(r["ok"], r)
        return FakeProc.started[-1].argv

    def test_without_overrides_the_argv_is_what_it_always_was(self):
        self.assertEqual(self.launch(), ["bash", self.script, "--course-file", self.course_path, "--no-uav"])
        self.assertEqual(self.panel.tuning_launched, {})
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 0, "launched": 0, "pending": False})

    def test_with_overrides_the_argv_ends_in_tuning_and_the_absolute_path(self):
        self.set_({HARD: 0.9, TOL: 0.7})
        argv = self.launch()
        self.assertEqual(argv, ["bash", self.script, "--course-file", self.course_path, "--no-uav",
                                "--tuning", self.path])
        self.assertTrue(os.path.isabs(argv[-1]))
        self.assertTrue(os.path.isfile(argv[-1]))
        self.assertEqual(self.panel.tuning_launched, {HARD: 0.9, TOL: 0.7})
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 2, "launched": 2, "pending": False})
        self.assertIn("--tuning " + self.path, "\n".join(self.panel.logs["sim"].since(0)["lines"]))

    def test_only_a_non_empty_tuning_file_is_passed(self):
        self.set_({HARD: 0.9})
        self.set_(reset="all")
        self.assertNotIn("--tuning", self.launch())

    def test_the_dry_run_stub_takes_the_extra_arguments(self):
        self.panel.a.dry_run = True
        self.set_({HARD: 0.9})
        argv = self.panel._script("gz_sim_up.sh", "--course-file", self.course_path, "--no-uav", "--tuning", self.path)
        self.assertEqual(argv[:4], [sys.executable, "-u", "-c", T.STUB_UP])
        out = subprocess.run(argv, capture_output=True, text=True, timeout=60)
        self.assertEqual(out.returncode, 0, out.stderr)
        self.assertIn("=== ready ===", out.stdout)

    def test_a_save_after_launch_is_pending_until_the_next_launch(self):
        self.set_({HARD: 0.9})
        self.launch()
        self.assertFalse(self.panel.state({})["tuning"]["pending"])
        self.set_({TOL: 0.7})
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 2, "launched": 1, "pending": True})
        v = self.view()
        self.assertEqual(v["launched"], {HARD: 0.9})
        self.assertTrue(v["pending"])
        self.set_(reset=[TOL])                                  # back to what the run has: nothing to relaunch for
        self.assertFalse(self.panel.state({})["tuning"]["pending"])
        self.set_({HARD: 1.0})                                  # same key, another value: pending
        self.assertTrue(self.view()["pending"])

    def test_launched_with_none_is_an_empty_dict_and_a_save_makes_it_pending(self):
        self.launch()
        self.assertEqual(self.view()["launched"], {})
        self.assertFalse(self.view()["pending"])
        self.set_({HARD: 0.9})
        self.assertTrue(self.view()["pending"])
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 1, "launched": 0, "pending": True})

    def test_no_sim_means_launched_is_null_and_nothing_is_pending(self):
        self.set_({HARD: 0.9})
        v = self.view()
        self.assertIsNone(v["launched"])
        self.assertFalse(v["pending"])
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 1, "launched": None, "pending": False})

    def test_stop_sim_and_a_failed_launch_forget_it(self):
        self.set_({HARD: 0.9})
        self.launch()
        self.panel.sim = "up"
        self.assertTrue(self.panel.act_stop_sim({})["ok"])      # the stand-in finishes gz_sim_down.sh at once
        self.assertEqual(self.panel.sim, "down")
        self.assertIsNone(self.panel.tuning_launched)
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 1, "launched": None, "pending": False})
        self.launch()
        self.assertEqual(self.panel.tuning_launched, {HARD: 0.9})
        self.panel._on_sim_exit(3)                              # gz_sim_up.sh died
        self.assertEqual(self.panel.sim, "failed")
        self.assertIsNone(self.panel.tuning_launched)
        self.assertEqual(self.launch()[-2:], ["--tuning", self.path])                        # failed is relaunchable

    def test_a_launch_that_could_not_start_forgets_it(self):
        self.set_({HARD: 0.9})

        def boom(self_):
            raise OSError("no bash")
        with mock.patch.object(FakeProc, "start", boom):
            r = self.panel.act_launch({})
        self.assertFalse(r["ok"])
        self.assertEqual(self.panel.sim, "failed")
        self.assertIsNone(self.panel.tuning_launched)

    def test_attach_did_not_launch_so_what_the_run_used_is_unknown(self):
        self.set_({HARD: 0.9})
        self.panel.act_layout({"buoys": LAYOUT})
        T.write_atomic(self.course_path, T.course_yaml(T.course_of("panel", LAYOUT)))
        self.panel.tuning_launched = {TOL: 0.1}                 # whatever an earlier run left behind must not survive
        with mock.patch.object(self.panel, "_radio_up", lambda: None):
            self.assertTrue(self.panel.act_attach({})["ok"])
        self.assertEqual(self.panel.sim, "up")
        self.assertIsNone(self.panel.tuning_launched)
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 1, "launched": None, "pending": False})

    def test_a_refused_launch_changes_nothing(self):
        self.set_({HARD: 0.9})
        self.panel.act_layout({"buoys": LAYOUT[:2]})            # no EXIT buoy
        r = self.panel.act_launch({})
        self.assertFalse(r["ok"])
        self.assertEqual(FakeProc.started, [])
        self.assertIsNone(self.panel.tuning_launched)


# ------------------------------------------------------------------ routes and the page

PROFILE_TIGHT = """\
# title: Tight 3 to 5 m
# about: Gates and orbits squeezed for a narrow course.
bt_runner_node:
  ros__parameters:
    nav_hard_m: 0.7
    nav_soft_m: 3
planner_server:
  ros__parameters:
    GridBased:
      tolerance: 0.7
global_costmap:
  global_costmap:
    ros__parameters:
      inflation_layer:
        inflation_radius: 2.5
"""
TIGHT = {HARD: 0.7, SOFT: 3, TOL: 0.7, INFL: 2.5}


class ProfileTest(PanelCase):
    def profile(self, name, text):
        write(os.path.join(self.repo, *T.PROFILES, name + ".yaml"), text)

    def profiles(self):
        return self.view()["profiles"]

    def test_listing_gives_name_title_and_about_sorted_by_name(self):
        self.profile("zeta", "# about: only an about line\nbt_runner_node:\n  ros__parameters:\n    nav_hard_m: 0.9\n")
        self.profile("tight", PROFILE_TIGHT)
        self.profile("plain", "bt_runner_node:\n  ros__parameters:\n    nav_hard_m: 0.9\n")
        self.profile("late", "bt_runner_node:\n  ros__parameters:\n    nav_hard_m: 0.9\n# title: after the first key\n")
        write(os.path.join(self.repo, *T.PROFILES, "notes.txt"), "# title: not a profile\n")
        write(os.path.join(self.repo, *T.PROFILES, "bad name.yaml"), "# title: not offered\n")
        self.assertEqual(self.profiles(), [
            {"name": "late", "title": "late", "about": ""},
            {"name": "plain", "title": "plain", "about": ""},
            {"name": "tight", "title": "Tight 3 to 5 m", "about": "Gates and orbits squeezed for a narrow course."},
            {"name": "zeta", "title": "zeta", "about": "only an about line"}])

    def test_a_missing_directory_is_an_empty_list_and_a_load_says_there_is_no_profile(self):
        self.assertFalse(os.path.isdir(os.path.join(self.repo, *T.PROFILES)))
        self.assertEqual(self.profiles(), [])
        self.assertEqual(T.list_profiles(os.path.join(self.tmp, "nowhere")), [])
        r = self.panel.act_tuning({"profile": "tight"})
        self.assertFalse(r["ok"])
        self.assertIn("no profile tight", r["error"])

    def test_load_replaces_the_saved_overrides_and_does_not_merge(self):
        self.profile("tight", PROFILE_TIGHT)
        self.set_({HARD: 0.9, CONFIRM: 4, LIDAR: 0.2})
        r = self.panel.act_tuning({"profile": "tight"})
        self.assertEqual(r, {"ok": True, "saved": 4})
        self.assertEqual(self.view()["saved"], TIGHT)                        # CONFIRM and LIDAR are gone
        self.assertEqual(self.written(), {
            "bt_runner_node": {"ros__parameters": {"nav_hard_m": 0.7, "nav_soft_m": 3}},
            "planner_server": {"ros__parameters": {"GridBased": {"tolerance": 0.7}}},
            "global_costmap": {"global_costmap": {"ros__parameters": {"inflation_layer": {"inflation_radius": 2.5}}}}})
        self.assertEqual(self.entry(HARD)["value"], 0.7)
        self.assertEqual(self.entry(CONFIRM)["value"], self.entry(CONFIRM)["default"])

    def test_load_with_nothing_saved_and_loading_twice(self):
        self.profile("tight", PROFILE_TIGHT)
        self.assertEqual(self.panel.act_tuning({"profile": "tight"}), {"ok": True, "saved": 4})
        self.assertEqual(self.panel.act_tuning({"profile": "tight"}), {"ok": True, "saved": 4})
        self.assertEqual(self.view()["saved"], TIGHT)

    def test_leaves_equal_to_the_default_are_dropped_and_an_empty_profile_clears_everything(self):
        self.profile("same", "# title: same\nbt_runner_node:\n  ros__parameters:\n    nav_hard_m: 0.8\n    nav_soft_m: 3\n")
        self.profile("empty", "# title: defaults\n")
        self.set_({TOL: 0.7})
        self.assertEqual(self.panel.act_tuning({"profile": "same"}), {"ok": True, "saved": 1})
        self.assertEqual(self.view()["saved"], {SOFT: 3})
        self.assertEqual(self.panel.act_tuning({"profile": "empty"}), {"ok": True, "saved": 0})
        self.assertFalse(os.path.exists(self.path))

    def test_a_profile_is_checked_like_a_set_and_one_bad_leaf_refuses_it_all(self):
        self.set_({CONFIRM: 4})
        before = read(self.path)
        ok_part = "bt_runner_node:\n  ros__parameters:\n    nav_hard_m: 0.7\n"
        cases = (
            (ok_part + "    nav_nope_m: 1.0\n", "unknown parameter bt_runner_node/nav_nope_m"),
            (ok_part + "    nav_soft_m: -1\n", "must not be negative"),
            (ok_part + "    nav_invalid_confirm: 2.5\n", "whole number"),
            (ok_part + "    nav_period_s: true\n", "finite number"),
            (ok_part + "    nav_period_s: fast\n", "finite number"),
            (ok_part + "    nav_zones: [1, 2]\n", "unknown parameter"),
            (ok_part + "    nav_mode: off\n", "unknown parameter"),
            (ok_part + "planner_server:\n  ros__parameters:\n    GridBased:\n      plugin: x\n", "unknown parameter"),
            (ok_part + "foo: 1\n", "unexpected key foo"),
            ("bt_runner_node:\n  nav_hard_m: 0.7\n", "unexpected key bt_runner_node.nav_hard_m"),
            ("global_costmap:\n  ros__parameters:\n    robot_radius: 0.9\n", "unexpected key"),
            (ok_part + "planner_server:\n  ros__parameters:\n    GridBased:\n      tolerance: .nan\n", "finite number"),
            ("bt_runner_node: [unclosed\n", "profile bad:"),
            ("- a\n- b\n", "not a YAML mapping"))
        for text, want in cases:
            self.profile("bad", "# title: bad\n" + text)
            r = self.panel.act_tuning({"profile": "bad"})
            self.assertFalse(r["ok"], text)
            self.assertIn(want, r["error"], text)
            self.assertEqual(read(self.path), before, "nothing is written: " + text)

    def test_a_bad_name_is_refused(self):
        self.profile("tight", PROFILE_TIGHT)
        for name in ("", "../tight", "a b", "x" * 41, "tight.yaml", "ti/ght", "ti\\ght", 5, None, ["tight"], {"a": 1}):
            r = self.panel.act_tuning({"profile": name})
            self.assertFalse(r["ok"], repr(name))
            self.assertIn("profile name", r["error"], repr(name))
        self.assertFalse(os.path.exists(self.path))
        self.assertTrue(T.NAME_RE.match("a-B_9"))
        self.assertEqual(self.panel.act_tuning({"profile": "x" * 40}), {"ok": False, "error": "no profile " + "x" * 40})

    def test_a_profile_takes_no_other_field(self):
        self.profile("tight", PROFILE_TIGHT)
        for body in ({"profile": "tight", "set": {HARD: 0.9}}, {"profile": "tight", "reset": "all"}):
            r = self.panel.act_tuning(body)
            self.assertFalse(r["ok"])
            self.assertIn("cannot be combined", r["error"])
        self.assertFalse(os.path.exists(self.path))

    def test_with_a_source_file_unreadable_a_profile_is_refused(self):
        self.profile("tight", PROFILE_TIGHT)
        self.set_({HARD: 0.9})
        before = read(self.path)
        os.remove(os.path.join(self.repo, *T.NAV_PARAMS))
        r = self.panel.act_tuning({"profile": "tight"})
        self.assertFalse(r["ok"])
        self.assertIn("cannot check", r["error"])
        self.assertEqual(read(self.path), before)

    def test_a_profile_loaded_after_launch_is_pending(self):
        self.profile("tight", PROFILE_TIGHT)
        self.panel.act_layout({"buoys": LAYOUT})
        self.assertTrue(self.panel.act_launch({})["ok"])                    # Proc is the stand-in
        self.assertFalse(self.view()["pending"])
        self.panel.act_tuning({"profile": "tight"})
        self.assertTrue(self.view()["pending"])
        self.assertEqual(self.panel.state({})["tuning"], {"saved": 4, "launched": 0, "pending": True})

    def test_the_shipped_profiles_load_against_the_real_catalogue(self):
        entries, errors = T.tuning_catalogue(T._SRC_REPO)
        self.assertEqual(errors, [])
        cat = {e["id"]: e for e in entries}
        pdir = os.path.join(T._SRC_REPO, *T.PROFILES)
        for p in T.list_profiles(pdir):
            sets, err = T.load_profile(pdir, p["name"], cat)
            self.assertIsNone(err, p["name"])
            self.assertGreater(len(sets), 0, p["name"])


class HttpTest(PanelCase):
    def setUp(self):
        super().setUp()
        self.srv = ThreadingHTTPServer(("127.0.0.1", 0), T.make_handler(self.panel))
        self.srv.daemon_threads = True
        threading.Thread(target=self.srv.serve_forever, daemon=True).start()
        self.addCleanup(self.srv.server_close)
        self.addCleanup(self.srv.shutdown)
        self.base = "http://127.0.0.1:%d" % self.srv.server_address[1]

    def get(self, path, base=None):
        with urllib.request.urlopen((base or self.base) + path, timeout=5) as r:
            return json.loads(r.read())

    def post(self, path, body=None, base=None):
        req = urllib.request.Request((base or self.base) + path, data=json.dumps(body or {}).encode(),
                                     method="POST", headers={"Content-Type": "application/json"})
        with urllib.request.urlopen(req, timeout=5) as r:
            return json.loads(r.read())

    def test_get_and_post_tuning(self):
        d = self.get("/api/tuning")
        self.assertEqual(d["saved"], {})
        self.assertEqual(d["catalogue"][0]["group"], "Goals")
        self.assertEqual(self.post("/api/tuning", {"set": {HARD: 0.9}}), {"ok": True, "saved": 1})
        self.assertEqual(self.get("/api/tuning")["saved"], {HARD: 0.9})
        self.assertIn("nav_hard_m: 0.9", self.get("/api/tuning")["yaml"])
        r = self.post("/api/tuning", {"set": {HARD: "nope"}})
        self.assertFalse(r["ok"])
        self.assertIn("finite number", r["error"])
        self.assertEqual(self.post("/api/tuning", {"reset": "all"}), {"ok": True, "saved": 0})
        self.assertEqual(self.get("/api/tuning")["yaml"], "")

    def test_profiles_are_listed_and_loaded_over_http(self):
        write(os.path.join(self.repo, *T.PROFILES, "tight.yaml"), PROFILE_TIGHT)
        self.post("/api/tuning", {"set": {CONFIRM: 4}})
        self.assertEqual(self.get("/api/tuning")["profiles"], [
            {"name": "tight", "title": "Tight 3 to 5 m", "about": "Gates and orbits squeezed for a narrow course."}])
        self.assertEqual(self.post("/api/tuning", {"profile": "tight"}), {"ok": True, "saved": 4})
        self.assertEqual(self.get("/api/tuning")["saved"], TIGHT)
        r = self.post("/api/tuning", {"profile": "../etc"})
        self.assertFalse(r["ok"])
        self.assertEqual(self.get("/api/tuning")["saved"], TIGHT)

    def test_state_carries_the_small_summary_and_not_the_catalogue(self):
        self.post("/api/tuning", {"set": {HARD: 0.9}})
        st = self.get("/api/state")
        self.assertEqual(st["tuning"], {"saved": 1, "launched": None, "pending": False})
        self.assertNotIn("catalogue", json.dumps(st))

    def test_the_lake_panel_has_no_tuning_routes(self):
        tmp = tempfile.mkdtemp(prefix="tuning_lake_")
        self.addCleanup(shutil.rmtree, tmp, True)
        lake = K.make_panel(NS(port=K.free_udp_port()), tmp)
        self.addCleanup(lake.shutdown)
        srv = ThreadingHTTPServer(("127.0.0.1", 0), T.make_handler(lake))
        srv.daemon_threads = True
        threading.Thread(target=srv.serve_forever, daemon=True).start()
        self.addCleanup(srv.server_close)
        self.addCleanup(srv.shutdown)
        base = "http://127.0.0.1:%d" % srv.server_address[1]
        with self.assertRaises(urllib.error.HTTPError) as cm:
            self.get("/api/tuning", base)
        self.assertEqual(cm.exception.code, 404)
        with self.assertRaises(urllib.error.HTTPError) as cm:
            self.post("/api/tuning", {"set": {HARD: 0.9}}, base)
        self.assertEqual(cm.exception.code, 404)
        self.assertNotIn("tuning", lake.state({}))


class PageTest(unittest.TestCase):
    def setUp(self):
        src = os.path.join(K.PKG, "crusader_sim")
        self.html, self.js, self.css = (read(os.path.join(src, n)) for n in (
            "task1_panel.html", "panel_common.js", "panel_common.css"))

    def test_the_card_follows_the_uav_error_card_and_says_what_it_does(self):
        h = self.html
        self.assertLess(h.index('id="uav"'), h.index('id="tuning"'))
        self.assertLess(h.index('id="tuning"'), h.index('id="uav-anchor-setup"'))
        self.assertIn("Planner tuning", h)
        self.assertIn("Applies at the next LAUNCH (STOP SIM, then LAUNCH). Sim only: the team's YAML files are not changed.", h)
        for word in ("SAVE", "DISCARD EDITS", "RESET ALL", "relaunch needed",
                     "overrides as YAML (paste into crusader_params.yaml / nav2_params.yaml when you are happy)"):
            self.assertIn(word, h)
        self.assertIn("initTuning();", h)
        self.assertIn("renderTuning();", h)

    def test_the_profile_row_and_its_confirm(self):
        self.assertIn("Profile:", self.html)
        self.assertIn('id="tuneProf"', self.html)
        self.assertIn('id="tuneProfAbout"', self.html)
        self.assertIn(">LOAD<", self.html)
        self.assertIn("Replace the saved overrides with profile '", self.js)
        self.assertIn("post('/api/tuning',{profile:name})", self.js)
        self.assertLess(self.html.index('id="tuneProf"'), self.html.index('id="tuneGroups"'))

    def test_the_card_travels_with_the_uav_card_between_setup_and_run(self):
        self.assertIn("home.insertBefore($('tuning'),before)", self.html)

    def test_the_js_fetches_the_catalogue_on_open_and_after_a_save_not_on_the_poll(self):
        self.assertEqual(self.js.count("fetch('/api/tuning'"), 1)
        for fn in ("function initTuning", "function renderTuning", "function tuneFetch", "function tuneSave"):
            self.assertIn(fn, self.js)
        self.assertNotIn("/api/tuning", self.js.split("function poll1")[1].split("function loop")[0])

    def test_the_lake_page_is_untouched_by_the_tuning_code(self):
        lake = read(os.path.join(K.PKG, "crusader_sim", "lake_panel.html"))
        self.assertNotIn('id="tuning"', lake)
        self.assertNotIn("initTuning", lake)
        self.assertNotIn("renderTuning", lake)

    def test_the_css_marks_an_overridden_row_with_amber(self):
        self.assertRegex(self.css, r"\.trow\.ovr\{border-left-color:var\(--warn\)")


class DocstringTest(unittest.TestCase):
    def test_the_routes_and_the_how_it_works_paragraph_are_documented(self):
        doc = T.__doc__
        self.assertIn("GET  /api/tuning", doc)
        self.assertIn("POST /api/tuning", doc)
        self.assertIn("PLANNER TUNING", doc)
        self.assertIn("{profile: name}", doc)
        self.assertIn("tuning_profiles", doc)
        for word in ("tuning.yaml", "--tuning", "LAUNCH", "sim only", "nested"):
            self.assertIn(word, doc.replace("Sim only", "sim only"))


if __name__ == "__main__":
    unittest.main()
