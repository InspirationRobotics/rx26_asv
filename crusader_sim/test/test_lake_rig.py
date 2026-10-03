"""Offline tests for what lake_rig_up.sh decides before it starts anything (lake_rig_plan.py) and for the script itself:
the default tree and nav_mode, POOL, the LAKE_TUNING overlay.

No ROS, no sim, no boat. Needs python3 with PyYAML and bash (the WSL host has both):

    cd ~/robotx_ws/src/rx26_asv/crusader_sim && python3 -m unittest discover -s test -p "test_lake_rig.py" -v

The script tests run `lake_rig_up.sh --check` (nothing is started) in a throwaway workspace (LAKE_WS, LAKE_ROS_SETUP: the
script's two test hooks) with a fake `ros2` and `pgrep` first on PATH, so what the preflight sees is under the test's control.
"""
import io
import json
import os
import shutil
import stat
import subprocess
import sys
import tempfile
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import lake_testkit as K                                                      # noqa: E402  (also sets sys.path)

import yaml                                                                   # noqa: E402

from crusader_sim import lake_rig_plan as RP                                  # noqa: E402

PKG = K.PKG                                                                   # .../crusader_sim
REPO = K.REPO                                                                 # .../rx26_asv
SCRIPT = os.path.join(PKG, "scripts", "lake_rig_up.sh")
DOWN = os.path.join(PKG, "scripts", "lake_rig_down.sh")
BASH = shutil.which("bash")
REAL_CFG = os.path.join(REPO, "crusader_bringup", "config", "crusader_params.yaml")
REAL_NAV = os.path.join(REPO, "crusader_nav", "config", "nav2_params.yaml")
TIGHT = os.path.join(PKG, "config", "tuning_profiles", "tight_3to5m.yaml")
BT_KEYS_IN_TIGHT = 19


def read_text(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def write_text(path, text):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)


def read_yaml(path):
    return yaml.safe_load(read_text(path))


class DecideTest(unittest.TestCase):
    def test_the_default_is_the_global_tree_with_nav_off_and_says_why(self):
        d = RP.decide({})
        self.assertEqual((d["tree"], d["nav_mode"], d["pool"], d["tuning"]), ("task1_global.xml", "off", False, ""))
        text = "\n".join(d["lines"])
        self.assertIn("task1_global.xml", text)
        self.assertIn("nav_mode    off", text)
        self.assertIn("never calls Nav2", text)
        self.assertIn("global_leaves.cpp", text)

    def test_the_per_gate_tree_keeps_its_old_default_and_its_explicit_modes(self):
        self.assertEqual(RP.decide({"TREE": "task1_disruptive.xml"})["nav_mode"], "shadow")
        for mode in ("off", "shadow", "on"):
            d = RP.decide({"TREE": "task1_disruptive.xml", "NAV_MODE": mode})
            self.assertEqual((d["tree"], d["nav_mode"]), ("task1_disruptive.xml", mode))
            self.assertEqual(d["nav_why"], "NAV_MODE=%s was given" % mode)

    def test_an_explicit_nav_mode_with_the_global_tree_is_honoured_and_the_page_is_told_the_planner_does_not_drive(self):
        d = RP.decide({"NAV_MODE": "on"})
        self.assertEqual((d["tree"], d["nav_mode"]), ("task1_global.xml", "on"))
        self.assertIn("does not drive the boat", "\n".join(d["notes"]))
        self.assertEqual(RP.decide({"NAV_MODE": "off"})["notes"], [])

    def test_a_path_to_the_global_tree_is_the_global_tree_and_an_empty_nav_mode_is_not_given(self):
        d = RP.decide({"TREE": "/x/behavior_trees/task1_global.xml", "NAV_MODE": ""})
        self.assertTrue(d["global_tree"])
        self.assertEqual(d["nav_mode"], "off")

    def test_a_bad_nav_mode_is_refused(self):
        with self.assertRaises(RP.PlanError) as cm:
            RP.decide({"NAV_MODE": "maybe"})
        self.assertIn("must be off, shadow or on", str(cm.exception))

    def test_pool_forces_nav_off_and_says_camera_only(self):
        for v in ("1", "true", "TRUE", "True"):
            d = RP.decide({"POOL": v, "NAV_MODE": "on", "TREE": "task1_disruptive.xml"})
            self.assertTrue(d["pool"], v)
            self.assertEqual(d["nav_mode"], "off")
            self.assertIn("IGNORED", "\n".join(d["notes"]))
            self.assertIn("POOL: camera-only, LiDAR not used for planning", "\n".join(d["lines"]))
        self.assertFalse(RP.decide({"POOL": "0"})["pool"])
        self.assertFalse(RP.decide({"POOL": "yes"})["pool"])      # the shell's PUBLISH rule: 1/true/TRUE/True only

    def test_pool_defaults_to_the_tight_profile_unless_a_tuning_file_is_given(self):
        d = RP.decide({"POOL": "1"})
        self.assertEqual(d["tuning"], TIGHT)
        self.assertTrue(os.path.isfile(d["tuning"]))
        self.assertIn("POOL default", d["tuning_why"])
        mine = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False)
        mine.close()
        try:
            d = RP.decide({"POOL": "1", "LAKE_TUNING": mine.name})
            self.assertEqual((d["tuning"], d["tuning_why"]), (os.path.abspath(mine.name), "LAKE_TUNING"))
        finally:
            os.remove(mine.name)

    def test_no_pool_profile_is_invented_and_the_module_says_where_one_goes(self):
        src = read_text(RP.__file__)
        self.assertIn("pool_<name>.yaml", src)
        self.assertIn("NOT written yet", src)
        self.assertEqual(RP.POOL_PROFILE, "tight_3to5m")

    def test_an_absent_tuning_file_is_a_clear_error_not_a_silent_skip(self):
        with self.assertRaises(RP.PlanError) as cm:
            RP.decide({"LAKE_TUNING": "/no/such/tuning.yaml"})
        self.assertIn("LAKE_TUNING=/no/such/tuning.yaml: no such file", str(cm.exception))

    def test_nothing_overrides_when_nothing_is_asked(self):
        d = RP.decide({})
        self.assertEqual(d["tuning"], "")
        self.assertIn("none: the boat's own params, unchanged", "\n".join(d["lines"]))


class OverlayTest(unittest.TestCase):
    cfg = read_yaml(REAL_CFG)
    nav = read_yaml(REAL_NAV)

    def plan(self, doc, nav_started, **kw):
        return RP.overlay_plan(doc, nav_started, kw.get("bt", self.cfg), kw.get("nav", self.nav))

    def test_the_tight_profile_is_valid_against_the_boats_own_files(self):
        p = self.plan(read_yaml(TIGHT), True)
        self.assertEqual(p["errors"], [])
        self.assertEqual(len(p["applied"]["bt_runner_node"]), BT_KEYS_IN_TIGHT)
        self.assertEqual(p["applied"]["bt_runner_node"]["nav_orbit_radius_m"], 3.0)
        self.assertEqual(p["applied"]["planner_server"], {"GridBased.tolerance": 0.3, "GridBased.cost_travel_multiplier": 1.5})
        self.assertEqual(set(p["applied"]["global_costmap/global_costmap"]),
                         {"inflation_layer.inflation_radius", "inflation_layer.cost_scaling_factor"})
        self.assertEqual(p["ignored"], {})

    def test_nav_keys_are_ignored_and_named_when_the_nav_stack_is_not_started(self):
        p = self.plan(read_yaml(TIGHT), False)
        self.assertEqual(p["errors"], [])
        self.assertEqual(list(p["applied"]), ["bt_runner_node"])
        self.assertEqual(sorted(p["ignored"]), ["global_costmap/global_costmap", "planner_server"])

    def test_a_key_the_boat_does_not_have_is_an_error_ros_would_have_ignored_silently(self):
        p = self.plan({"bt_runner_node": {"ros__parameters": {"nav_orbit_radius_mm": 3.0}}}, False)
        self.assertEqual(len(p["errors"]), 1)
        self.assertIn("nav_orbit_radius_mm", p["errors"][0])
        self.assertIn("silently ignore", p["errors"][0])

    def test_a_wrong_type_is_an_error_ros_would_stop_the_node_for(self):
        p = self.plan({"bt_runner_node": {"ros__parameters": {"nav_orbit_radius_m": 3, "nav_orbit_points": 12.0}}}, False)
        self.assertEqual(len(p["errors"]), 2)
        self.assertTrue(all("refuse to start" in e for e in p["errors"]))

    def test_a_node_the_overlay_cannot_reach_is_an_error(self):
        p = self.plan({"rxl_link_node": {"ros__parameters": {"x": 1}}}, True)
        self.assertIn("is not a node this overlay reaches", p["errors"][0])
        p = self.plan({"bt_runner_node": {"x": 1}}, True)
        self.assertIn("no ros__parameters", p["errors"][0])
        p = self.plan({"bt_runner_node": 3}, True)
        self.assertIn("not a node section", p["errors"][0])

    def test_a_base_file_that_cannot_be_read_is_said_not_assumed_fine(self):
        p = RP.overlay_plan(read_yaml(TIGHT), False, None, None)
        self.assertEqual(p["errors"], [])
        self.assertEqual(p["unchecked"], ["bt_runner_node"])

    def test_the_applied_document_round_trips(self):
        p = self.plan(read_yaml(TIGHT), True)
        doc = RP._nest(p["applied"])
        self.assertEqual(RP.sections(doc), p["applied"])
        self.assertEqual(doc["bt_runner_node"]["ros__parameters"]["nav_orbit_points"], 12)
        self.assertEqual(doc["global_costmap"]["global_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"], 1.2)

    def test_read_back_matches_mismatches_and_misses(self):
        applied = {"bt_runner_node": {"nav_orbit_radius_m": 3.0, "nav_orbit_points": 12, "nav_soft_m": 1.2}}
        dump = {"/bt_runner_node": {"ros__parameters": {"nav_orbit_radius_m": 3.0, "nav_orbit_points": 12, "nav_soft_m": 2.0}}}
        ok, bad = RP.verify_dump(applied, dump)
        self.assertEqual((ok, len(bad)), (2, 1))
        self.assertIn("nav_soft_m", bad[0])
        dump["/bt_runner_node"]["ros__parameters"]["nav_soft_m"] = 1.2000000001
        self.assertEqual(RP.verify_dump(applied, dump), (3, []))
        del dump["/bt_runner_node"]["ros__parameters"]["nav_orbit_points"]
        ok, bad = RP.verify_dump(applied, dump)
        self.assertEqual(ok, 2)
        self.assertIn("not on the node", bad[0])
        self.assertEqual(RP.verify_dump(applied, {})[0], 0)


class FinishTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lake_rig_")
        self.addCleanup(shutil.rmtree, self.tmp, True)

    def run_main(self, argv, env):
        out, err = io.StringIO(), io.StringIO()
        old = sys.stdout, sys.stderr
        sys.stdout, sys.stderr = out, err
        try:
            rc = RP.main(argv, env)
        finally:
            sys.stdout, sys.stderr = old
        return rc, out.getvalue(), err.getvalue()

    def finish(self, env, nav="off", dry=False):
        argv = ["finish", "--nav-mode", nav, "--cfg", REAL_CFG, "--nav2-cfg", REAL_NAV, "--out-dir", self.tmp,
                "--publish", "true"] + (["--dry"] if dry else [])
        return self.run_main(argv, env)

    def test_pool_writes_the_applied_overlay_and_the_rig_file(self):
        rc, out, err = self.finish({"POOL": "1"})
        self.assertEqual((rc, err), (0, ""))
        self.assertIn("19 key(s) applied", out)
        self.assertIn("nav_orbit_radius_m = 3.0", out)
        self.assertIn("IGNORED planner_server", out)
        applied = read_yaml(os.path.join(self.tmp, "tuning_applied.yaml"))
        self.assertEqual(list(applied), ["bt_runner_node"])
        self.assertEqual(applied["bt_runner_node"]["ros__parameters"]["nav_orbit_radius_m"], 3.0)
        rig = json.loads(read_text(os.path.join(self.tmp, "rig.json")))
        self.assertEqual((rig["tree"], rig["global_tree"], rig["nav_mode"], rig["pool"], rig["publish"]),
                         ("task1_global.xml", True, "off", True, True))
        self.assertEqual(rig["ignored"], {"planner_server": ["GridBased.cost_travel_multiplier", "GridBased.tolerance"],
                                          "global_costmap/global_costmap": ["inflation_layer.cost_scaling_factor",
                                                                            "inflation_layer.inflation_radius"]})

    def test_with_nav_started_the_nav_sections_are_kept(self):
        rc, out, _ = self.finish({"TREE": "task1_disruptive.xml", "LAKE_TUNING": TIGHT}, nav="on")
        self.assertEqual(rc, 0)
        applied = read_yaml(os.path.join(self.tmp, "tuning_applied.yaml"))
        self.assertEqual(sorted(applied), ["bt_runner_node", "global_costmap", "planner_server"])
        self.assertNotIn("IGNORED", out)

    def test_dry_writes_nothing(self):
        rc, out, _ = self.finish({"POOL": "1"}, dry=True)
        self.assertEqual(rc, 0)
        self.assertIn("19 key(s) applied", out)
        self.assertEqual(os.listdir(self.tmp), [])

    def test_no_overlay_removes_a_previous_runs_applied_file_and_still_writes_the_rig_file(self):
        stale = os.path.join(self.tmp, "tuning_applied.yaml")
        write_text(stale, "bt_runner_node: {ros__parameters: {nav_orbit_radius_m: 99.0}}\n")
        rc, out, _ = self.finish({})
        self.assertEqual((rc, out), (0, ""))
        self.assertFalse(os.path.exists(stale), "a previous run's overlay must never be applied to this one")
        self.assertIsNone(json.loads(read_text(os.path.join(self.tmp, "rig.json")))["tuning"])

    def test_a_bad_file_says_every_problem_at_once_and_exits_2(self):
        bad = os.path.join(self.tmp, "bad.yaml")
        write_text(bad, "bt_runner_node:\n  ros__parameters:\n    nav_orbit_radius_mm: 3.0\n    nav_orbit_points: 12.5\n")
        rc, out, err = self.finish({"LAKE_TUNING": bad})
        self.assertEqual(rc, 2)
        self.assertIn("nav_orbit_radius_mm", err)
        self.assertIn("nav_orbit_points", err)
        self.assertNotIn("rig.json", os.listdir(self.tmp))

    def test_a_missing_file_and_an_unparseable_one_exit_2_with_the_path(self):
        rc, _, err = self.run_main(["decide"], {"LAKE_TUNING": os.path.join(self.tmp, "nope.yaml")})
        self.assertEqual(rc, 2)
        self.assertIn("nope.yaml: no such file", err)
        junk = os.path.join(self.tmp, "junk.yaml")
        write_text(junk, "a: [unclosed\n")
        rc, _, err = self.finish({"LAKE_TUNING": junk})
        self.assertEqual(rc, 2)
        self.assertIn("junk.yaml", err)

    def test_the_decide_assignments_survive_the_shells_eval(self):
        if not BASH:
            self.skipTest("no bash")
        rc, out, _ = self.run_main(["decide"], {"POOL": "1", "TREE": "task1_global.xml"})
        self.assertEqual(rc, 0)
        r = subprocess.run([BASH, "-c", 'eval "$1"; printf "%s|%s|%s|%s" "$PLAN_TREE" "$PLAN_NAV_MODE" "$PLAN_POOL" "$PLAN_NAV_WHY"; '
                            'printf "\\n%s" "$PLAN_BANNER"', "sh", out], capture_output=True, text=True)
        self.assertEqual(r.returncode, 0, r.stderr)
        first, banner = r.stdout.split("\n", 1)
        self.assertTrue(first.startswith("task1_global.xml|off|1|POOL=1: camera only"), first)
        self.assertIn("POOL: camera-only, LiDAR not used for planning", banner)
        self.assertIn("tuning      " + TIGHT, banner)


# ------------------------------------------------------------------ the script, in a fake workspace

FAKE_ROS2 = """#!/bin/bash
# a stand-in for ros2: which packages exist, what the autopilot says, what use_lidar is
case "$1 $2" in
  "pkg prefix") for m in $FAKE_MISSING; do [ "$3" = "$m" ] && exit 1; done; echo "/fake/$3"; exit 0 ;;
  "topic echo") printf 'mode: GUIDED\\narmed: true\\n'; exit 0 ;;
  "param get") printf 'Boolean value is: %s\\n' "${FAKE_USE_LIDAR:-False}"; exit 0 ;;
esac
exit 0
"""
FAKE_PGREP = """#!/bin/bash
# a stand-in for pgrep: only what FAKE_RUNNING names is "running"
for pat in $FAKE_RUNNING; do case "$*" in *"$pat"*) echo 4242; exit 0 ;; esac; done
exit 1
"""


@unittest.skipUnless(BASH, "needs bash")
class RigScriptTest(unittest.TestCase):
    """lake_rig_up.sh --check (and the refusals that come before anything starts) in a throwaway workspace."""

    def setUp(self):
        self.tmp = tempfile.mkdtemp(prefix="lake_ws_")
        self.addCleanup(shutil.rmtree, self.tmp, True)
        ws = os.path.join(self.tmp, "ws")
        for rel, src in (("install/crusader_bringup/share/crusader_bringup/config/crusader_params.yaml", REAL_CFG),
                         ("install/crusader_nav/share/crusader_nav/config/nav2_params.yaml", REAL_NAV),
                         ("install/crusader_bt/share/crusader_bt/behavior_trees/task1_global.xml",
                          os.path.join(REPO, "crusader_bt", "behavior_trees", "task1_global.xml")),
                         ("install/crusader_bt/share/crusader_bt/behavior_trees/task1_disruptive.xml",
                          os.path.join(REPO, "crusader_bt", "behavior_trees", "task1_disruptive.xml"))):
            os.makedirs(os.path.dirname(os.path.join(ws, rel)), exist_ok=True)
            shutil.copy(src, os.path.join(ws, rel))
        write_text(os.path.join(ws, "install", "setup.bash"), "")
        write_text(os.path.join(self.tmp, "ros_setup.bash"), "")
        msgs = os.path.join(self.tmp, "pylib", "crusader_msgs")          # what the script's python-imports preflight imports
        os.makedirs(msgs)
        write_text(os.path.join(msgs, "__init__.py"), "")
        write_text(os.path.join(msgs, "action.py"), "class SafePassage: pass\n")
        write_text(os.path.join(msgs, "msg.py"), "".join("class %s: pass\n" % n for n in (
            "FcuStatus", "HazardArray", "LatLonHead", "TrackedTargetArray")))
        bindir = os.path.join(self.tmp, "bin")
        os.makedirs(bindir)
        for name, body in (("ros2", FAKE_ROS2), ("pgrep", FAKE_PGREP)):
            path = os.path.join(bindir, name)
            write_text(path, body)
            os.chmod(path, os.stat(path).st_mode | stat.S_IEXEC)
        self.logdir = os.path.join(self.tmp, "logs")
        self.env = dict(os.environ, LAKE_WS=ws, LAKE_ROS_SETUP=os.path.join(self.tmp, "ros_setup.bash"),
                        LAKE_SRC=REPO, LAKE_LOGDIR=self.logdir, LAKE_DATUM="1.3000000,103.8500000",
                        PATH=bindir + os.pathsep + os.environ["PATH"], PYTHONPATH=os.path.join(self.tmp, "pylib"),
                        FAKE_MISSING="", FAKE_RUNNING="telemetry_bridge",
                        PANEL_PORT="38095")
        for k in ("TREE", "NAV_MODE", "POOL", "LAKE_TUNING", "PUBLISH"):
            self.env.pop(k, None)
        self.started = False

    def tearDown(self):
        if self.started:                                  # a run that was not supposed to start anything did
            subprocess.run([BASH, DOWN, "--quiet"], env=self.env, capture_output=True, timeout=60)

    def run_rig(self, check=True, timeout=120, **env):
        e = dict(self.env, **{k: v for k, v in env.items() if v is not None})
        r = subprocess.run([BASH, SCRIPT] + (["--check"] if check else []), env=e, capture_output=True, text=True,
                           timeout=timeout)
        self.started = not check
        return r

    def test_default_check_says_the_global_tree_nav_off_and_why(self):
        r = self.run_rig()
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("tree        task1_global.xml", r.stdout)
        self.assertIn("nav_mode    off  <- task1_global.xml drives GUIDED setpoints itself and never calls Nav2", r.stdout)
        self.assertIn("would start: tree task1_global.xml, nav_mode off, publish_setpoints false", r.stdout)
        self.assertIn("check done: nothing was started", r.stdout)
        self.assertNotIn("POOL", r.stdout)

    def test_nav_off_does_not_need_the_nav_packages(self):
        r = self.run_rig(FAKE_MISSING="crusader_nav crusader_nav_layers nav2_planner")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("not needed with nav_mode off", r.stdout)
        self.assertNotIn("missing packages", r.stderr)
        self.assertNotIn("AVOIDANCE OFF", r.stdout)

    def test_a_per_gate_tree_without_nav2_still_falls_back_to_off_with_the_banner(self):
        r = self.run_rig(TREE="task1_disruptive.xml", FAKE_MISSING="nav2_planner")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("AVOIDANCE OFF", r.stdout)
        self.assertIn("would start: tree task1_disruptive.xml, nav_mode off", r.stdout)

    def test_a_missing_package_that_is_not_nav_still_stops_the_rig(self):
        r = self.run_rig(FAKE_MISSING="crusader_bt")
        self.assertEqual(r.returncode, 2)
        self.assertIn("missing packages: crusader_bt", r.stderr)

    def test_the_per_gate_tree_with_nav_on_is_exactly_as_before(self):
        r = self.run_rig(TREE="task1_disruptive.xml", NAV_MODE="on")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("nav_mode    on  <- NAV_MODE=on was given", r.stdout)
        self.assertIn("would start: tree task1_disruptive.xml, nav_mode on", r.stdout)
        self.assertNotIn("overlay", r.stdout)
        r = self.run_rig(TREE="task1_disruptive.xml")
        self.assertIn("nav_mode    shadow", r.stdout)

    def test_a_bad_nav_mode_is_refused_before_anything(self):
        r = self.run_rig(NAV_MODE="auto")
        self.assertEqual(r.returncode, 2)
        self.assertIn("NAV_MODE must be off, shadow or on (got 'auto')", r.stderr)

    def test_pool_check_says_camera_only_forces_off_and_lists_the_overlay(self):
        r = self.run_rig(POOL="1", NAV_MODE="on")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("POOL: camera-only, LiDAR not used for planning", r.stdout)
        self.assertIn("nav_mode    off  <- POOL=1", r.stdout)
        self.assertIn("NAV_MODE=on was given and is IGNORED", r.stdout)
        self.assertIn("tight_3to5m.yaml  <- POOL default", r.stdout)
        self.assertIn("19 key(s) applied", r.stdout)
        self.assertIn("nav_orbit_radius_m = 3.0", r.stdout)
        self.assertIn("IGNORED planner_server", r.stdout)
        self.assertIn("would start: tree task1_global.xml, nav_mode off, publish_setpoints false, POOL (camera only)", r.stdout)
        self.assertEqual(os.listdir(self.logdir), [])      # --check writes nothing

    def test_a_given_tuning_file_beats_the_pool_default(self):
        mine = os.path.join(self.tmp, "mine.yaml")
        write_text(mine, "bt_runner_node:\n  ros__parameters:\n    nav_orbit_radius_m: 4.5\n")
        r = self.run_rig(POOL="1", LAKE_TUNING=mine)
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("1 key(s) applied", r.stdout)
        self.assertIn("nav_orbit_radius_m = 4.5", r.stdout)
        self.assertNotIn("tight_3to5m", r.stdout)

    def test_a_missing_or_wrong_tuning_file_stops_the_check_and_the_run(self):
        for check in (True, False):
            r = self.run_rig(check=check, LAKE_TUNING=os.path.join(self.tmp, "absent.yaml"))
            self.assertEqual(r.returncode, 2, (check, r.stdout))
            self.assertIn("absent.yaml: no such file", r.stderr)
        typo = os.path.join(self.tmp, "typo.yaml")
        write_text(typo, "bt_runner_node:\n  ros__parameters:\n    nav_orbit_radis_m: 3.0\n")
        for check in (True, False):
            r = self.run_rig(check=check, LAKE_TUNING=typo)
            self.assertEqual(r.returncode, 2, (check, r.stdout))
            self.assertIn("nav_orbit_radis_m is not a parameter in the boat's params file", r.stderr)
        self.assertFalse(os.path.exists(os.path.join(self.logdir, "pids")), "nothing may have been started")

    def test_pool_refuses_a_running_tracker_that_fuses_the_lidar_before_it_starts_anything(self):
        r = self.run_rig(check=False, POOL="1", FAKE_RUNNING="telemetry_bridge target_tracker", FAKE_USE_LIDAR="True")
        self.assertEqual(r.returncode, 3, r.stdout + r.stderr)
        self.assertIn("a target_tracker is already running and its use_lidar is not false", r.stderr)
        self.assertFalse(os.path.exists(os.path.join(self.logdir, "pids")))
        r = self.run_rig(POOL="1", FAKE_RUNNING="telemetry_bridge target_tracker", FAKE_USE_LIDAR="False")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("already has use_lidar false", r.stdout)

    def test_the_script_wires_the_plan_into_the_commands_it_starts(self):
        text = read_text(SCRIPT)
        self.assertNotIn("\r", text, "shell scripts must stay LF")
        self.assertNotIn('TREE="${TREE:-task1_disruptive.xml}"', text)
        # bt_runner gets the overlay as a 2nd --params-file, right after the boat's own
        self.assertIn('bt_runner_node --ros-args --params-file "$CFG" "${BT_TUNE[@]}"', text)
        self.assertIn('BT_TUNE=(--params-file "$APPLIED")', text)
        self.assertIn('NAV_TUNE=("nav2_overlay:=$APPLIED")', text)
        self.assertIn('"${NAV_TUNE[@]}"; fi', text)
        # target_tracker: use_lidar:=false explicitly, only under POOL
        self.assertIn('target_tracker --ros-args --params-file "$CFG" "${TT_ARGS[@]}"', text)
        self.assertIn('[ "$POOL" = 1 ] && TT_ARGS=(-p use_lidar:=false)', text)
        self.assertEqual(text.count("use_lidar:=false"), 3)     # the header, the assignment, the --check message
        self.assertIn("--rig-file", text)

    def test_the_feed_port_can_move_beside_another_panel_and_a_bad_one_is_refused(self):
        self.assertEqual(self.run_rig(LAKE_FEED_PORT="14557").returncode, 0)
        r = self.run_rig(LAKE_FEED_PORT="14x57")
        self.assertEqual(r.returncode, 2)
        self.assertIn("LAKE_FEED_PORT must be a udp port number", r.stderr)
        text = read_text(SCRIPT)
        self.assertIn("-p panel_port:=$FEED_PORT", text)           # panel_feed sends there ...
        self.assertIn('--feed-port "$FEED_PORT"', text)            # ... and the panel listens there
        self.assertIn('FEED_PORT="${LAKE_FEED_PORT:-14556}"', text)    # the default is the one the sim and the docs use

    def test_both_scripts_parse_and_stay_lf(self):
        for path in (SCRIPT, DOWN):
            r = subprocess.run([BASH, "-n", path], capture_output=True, text=True)
            self.assertEqual(r.returncode, 0, r.stderr)
            with open(path, "rb") as f:
                self.assertNotIn(b"\r", f.read(), path)


class PoolTrackerContractTest(unittest.TestCase):
    """POOL's `-p use_lidar:=false` restates what the boat's own params already say: if that ever changes, POOL must
    keep working and the banner must not lie."""

    def test_the_boats_params_and_the_trackers_default_are_camera_only(self):
        self.assertIs(read_yaml(REAL_CFG)["target_tracker"]["ros__parameters"]["use_lidar"], False)
        core = read_text(os.path.join(REPO, "crusader_world_model", "crusader_world_model", "target_tracker_core.py"))
        self.assertIn("use_lidar: bool = False", core)


if __name__ == "__main__":
    unittest.main()
