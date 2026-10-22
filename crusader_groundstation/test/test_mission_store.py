"""mission_store: the catalogue the boat serves and the mission files it keeps. Standard library
only; the library trees are a synthetic directory, and the real ones are checked for shape.
"""
import os
import shutil
import sys
import tempfile
import unittest
from unittest import mock

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, ".."))
from crusader_groundstation import mission_store  # noqa: E402

REAL_TREES = os.path.normpath(os.path.join(HERE, "..", "..", "crusader_bt", "behavior_trees"))

# What `bt_tree_check --models` prints, cut down: BT.CPP's own nodes, two of ours, one SubTree.
MODELS = """<?xml version="1.0"?>
<root BTCPP_format="4">
  <TreeNodesModel>
    <Control ID="Sequence"/>
    <Decorator ID="RetryUntilSuccessful">
      <input_port name="num_attempts" type="int"/>
    </Decorator>
    <Action ID="Sleep"><input_port name="msec" type="unsigned int"/></Action>
    <Action ID="Wait">
      <input_port name="seconds" type="double" default="0.500000">how long, seconds</input_port>
      <input_port name="count" type="int"/>
      <input_port name="radius" type="double">metres (absent: the parameter)</input_port>
      <output_port name="done" type="bool"/>
    </Action>
    <Action ID="NavigateTo">
      <input_port name="target" type="std::string"/>
      <input_port name="goal" type="Waypoint"/>
    </Action>
    <SubTree ID="Task1Advanced"><input_port name="ignored" type="int"/></SubTree>
  </TreeNodesModel>
</root>
"""

WRAPPER = """<root BTCPP_format="4" main_tree_to_execute="Main">
  <include path="lib/common/task1_plan_drive.xml"/>
  <include path="lib/task1_advanced.xml"/>
  <BehaviorTree ID="Main"><ReactiveSequence><IsAutonomous/>
    <SubTree ID="Task1Advanced" _autoremap="true" lookahead_m="2.5" plan_attempts="4"
             orbit_min_m="1.5" orbit_radius_m="" flag="true"/>
  </ReactiveSequence></BehaviorTree>
</root>
"""
TIER = """<root BTCPP_format="4">
  <BehaviorTree ID="Task1Advanced"><Sequence><Wait seconds="{lookahead_m}"/>
    <SubTree ID="Task1PlanDrive" _autoremap="true"/></Sequence></BehaviorTree>
  <TreeNodesModel><SubTree ID="Task1Advanced">
    <input_port name="lookahead_m" default="2.5">pure-pursuit lookahead, metres</input_port>
  </SubTree></TreeNodesModel>
</root>
"""
COMMON = """<root BTCPP_format="4">
  <BehaviorTree ID="Task1PlanDrive"><RetryUntilSuccessful num_attempts="{plan_attempts}">
    <Wait count="3"/></RetryUntilSuccessful></BehaviorTree>
</root>
"""


def write(path, text):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8") as f:
        f.write(text)


def read(path):
    with open(path, encoding="utf-8") as f:
        return f.read()


def by_name(items):
    return {i["name"]: i for i in items}


class Tmp(unittest.TestCase):

    def setUp(self):
        self.tmp = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, self.tmp, True)
        self.trees, self.mine = os.path.join(self.tmp, "trees"), os.path.join(self.tmp, "missions")
        write(os.path.join(self.trees, "task1_advanced.xml"), WRAPPER)
        write(os.path.join(self.trees, "other.xml"), "<root/>")
        write(os.path.join(self.trees, "lib", "task1_advanced.xml"), TIER)
        write(os.path.join(self.trees, "lib", "common", "task1_plan_drive.xml"), COMMON)
        self.store = mission_store.MissionStore(self.trees, self.mine)


class Models(unittest.TestCase):

    def setUp(self):
        self.nodes = {n["id"]: n for n in mission_store.parse_models(MODELS)}

    def test_kinds_and_ports(self):
        self.assertEqual(self.nodes["Sequence"], {"id": "Sequence", "kind": "control", "ports": []})
        self.assertEqual(self.nodes["RetryUntilSuccessful"]["kind"], "decorator")
        self.assertNotIn("Task1Advanced", self.nodes)           # the SubTree entry is not a leaf
        wait = by_name(self.nodes["Wait"]["ports"])
        self.assertEqual((wait["seconds"]["dir"], wait["seconds"]["type"]), ("in", "double"))
        self.assertEqual(wait["seconds"]["doc"], "how long, seconds")
        self.assertEqual(wait["done"]["dir"], "out")

    def test_a_double_default_is_written_as_the_wrappers_write_it(self):
        self.assertEqual(by_name(self.nodes["Wait"]["ports"])["seconds"]["default"], "0.5")
        self.assertEqual(mission_store._tidy_default("double", "45.000000"), "45.0")
        self.assertEqual(mission_store._tidy_default("int", "4"), "4")
        self.assertIsNone(mission_store._tidy_default("double", None))
        self.assertEqual(mission_store._tidy_default("std::string", "0.500000"), "0.500000")

    def test_required_means_the_leaf_cannot_run_without_it(self):
        wait, nav = by_name(self.nodes["Wait"]["ports"]), by_name(self.nodes["NavigateTo"]["ports"])
        self.assertFalse(wait["seconds"]["required"])           # has a default
        self.assertTrue(wait["count"]["required"])              # none, and nothing says optional
        self.assertFalse(wait["radius"]["required"])            # "(absent: ...)" in its doc
        self.assertFalse(wait["done"]["required"])              # an output
        self.assertTrue(nav["target"]["required"])
        self.assertFalse(nav["goal"]["required"])               # only for target="port"
        self.assertTrue(by_name(self.nodes["Sleep"]["ports"])["msec"]["required"])


class Subtrees(Tmp):

    def setUp(self):
        super().setUp()
        self.nodes = mission_store.parse_models(MODELS)

    def test_tier_files_are_blocks_and_common_bodies_are_known_ids_only(self):
        subtrees = mission_store.lib_subtrees(self.trees, self.nodes)
        self.assertEqual([s["id"] for s in subtrees], ["Task1Advanced", "Task1PlanDrive"])
        tier, body = subtrees
        self.assertEqual((tier["task"], tier["tier"]), (1, "advanced"))
        self.assertEqual(body, {"id": "Task1PlanDrive", "ports": []})

    def test_ports_are_the_wrappers_tunables_typed_by_what_they_feed(self):
        ports = by_name(mission_store.lib_subtrees(self.trees, self.nodes)[0]["ports"])
        self.assertEqual(list(ports), ["lookahead_m", "plan_attempts", "orbit_min_m",
                                       "orbit_radius_m", "flag"])
        self.assertEqual((ports["lookahead_m"]["type"], ports["lookahead_m"]["default"]),
                         ("double", "2.5"))                     # {lookahead_m} feeds Wait.seconds
        self.assertEqual(ports["lookahead_m"]["doc"], "pure-pursuit lookahead, metres")
        self.assertEqual(ports["plan_attempts"]["type"], "int")  # fed in the common body
        self.assertEqual(ports["orbit_min_m"]["type"], "double")  # unfed: from the default's look
        self.assertEqual((ports["orbit_radius_m"]["type"], ports["orbit_radius_m"]["default"]),
                         ("std::string", ""))                   # empty default: no type to guess
        self.assertEqual(ports["flag"]["type"], "bool")
        self.assertTrue(all(p["dir"] == "in" and p["required"] is False for p in ports.values()))

    def test_a_tree_directory_without_lib_has_no_subtrees(self):
        self.assertEqual(mission_store.lib_subtrees(os.path.join(self.tmp, "nowhere"), self.nodes), [])

    def test_the_catalogue_adds_builtins_and_the_users_missions(self):
        write(os.path.join(self.mine, "mine.xml"), "<root/>")
        cat = mission_store.catalogue(self.trees, self.nodes, self.store)
        self.assertEqual(sorted(cat), ["builtins", "nodes", "subtrees", "user"])
        self.assertEqual(cat["builtins"], ["other", "task1_advanced"])
        self.assertEqual([(u["name"], u["bytes"]) for u in cat["user"]], [("mine", 7)])
        self.assertIsInstance(cat["user"][0]["mtime"], float)

    def test_the_real_library_has_every_tier_and_both_shared_bodies(self):
        if not os.path.isdir(REAL_TREES):
            self.skipTest("crusader_bt is not beside this package")
        subtrees = mission_store.lib_subtrees(REAL_TREES, [])
        tiers = {s["id"]: s for s in subtrees if "tier" in s}
        self.assertEqual(sorted(tiers), ["Task1Advanced", "Task1Disruptive", "Task3Advanced",
                                         "Task3Core", "Task3Disruptive"])
        self.assertEqual({(s["task"], s["tier"]) for s in tiers.values()},
                         {(1, "advanced"), (1, "disruptive"), (3, "advanced"), (3, "core"),
                          (3, "disruptive")})
        self.assertEqual([s["id"] for s in subtrees if "tier" not in s],
                         ["Task1PlanDrive", "Task3DockFire"])
        self.assertEqual([p["name"] for p in tiers["Task1Advanced"]["ports"]],
                         ["orbit_radius_m", "orbit_min_m", "overshoot_deg", "lookahead_m",
                          "blocked_timeout_s", "plan_attempts"])
        self.assertEqual(by_name(tiers["Task1Advanced"]["ports"])["lookahead_m"]["default"], "2.5")


class Files(Tmp):

    def put(self, name, text):
        write(os.path.join(self.mine, name + ".xml"), text)

    def test_listings(self):
        self.assertEqual((self.store.builtins(), self.store.user()), (["other", "task1_advanced"], []))
        self.put("b_two", "<root/>")
        self.put("a_one", "<root/>")
        self.put("task1_advanced", "<root>mine</root>")         # a clash: the built-in wins
        write(os.path.join(self.mine, "notes.txt"), "no")
        write(os.path.join(self.mine, ".archive", "a_one.20260101-000000.xml"), "<root/>")
        write(os.path.join(self.mine, ".a_one.abc.tmp"), "<root/>")      # a staged upload
        self.assertEqual(self.store.user(), ["a_one", "b_two"])
        self.assertEqual(self.store.runnable(),
                         ["other", "task1_advanced", "a_one", "b_two"])
        self.assertEqual([e["name"] for e in self.store.user_entries()], ["a_one", "b_two"])

    def test_read_answers_by_listing_never_by_path(self):
        self.put("mine", "<root>mine</root>")
        self.assertEqual(self.store.read("mine"), ("<root>mine</root>", False))
        self.assertEqual(self.store.read("other"), ("<root/>", True))
        write(os.path.join(self.tmp, "secret.xml"), "<root>secret</root>")
        for name in ("nope", "../secret", "..\\secret", "", "mine.xml", os.path.join(self.tmp, "secret")):
            with self.assertRaises(KeyError, msg=name):
                self.store.read(name)

    def test_a_new_mission_is_staged_beside_its_place_then_committed(self):
        staged = self.store.stage("fresh", "<root>1</root>")
        self.assertEqual(os.path.dirname(staged), self.mine)    # same directory: os.replace is atomic
        self.assertEqual(self.store.user(), [])                 # not a mission yet
        if os.name == "posix":
            self.assertEqual(os.stat(staged).st_mode & 0o777, 0o644)
        self.assertIsNone(self.store.commit("fresh", staged))
        self.assertEqual(read(self.store.path("fresh")), "<root>1</root>")
        self.assertFalse(os.path.exists(staged))
        self.assertFalse(os.path.exists(os.path.join(self.mine, mission_store.ARCHIVE)))

    def test_replacing_a_mission_archives_the_old_one_first(self):
        self.put("mine", "<root>old</root>")
        archived = self.store.commit("mine", self.store.stage("mine", "<root>new</root>"))
        self.assertEqual(read(self.store.path("mine")), "<root>new</root>")
        self.assertEqual(os.path.dirname(archived), os.path.join(self.mine, mission_store.ARCHIVE))
        self.assertRegex(os.path.basename(archived), r"^mine\.\d{8}-\d{6}\.xml$")
        self.assertEqual(read(archived), "<root>old</root>")
        self.assertEqual(self.store.user(), ["mine"])           # the archive is not a mission

    def test_changes_in_one_second_do_not_overwrite_each_other(self):
        self.put("mine", "<root>0</root>")
        with mock.patch("time.strftime", return_value="20261022-120000"):
            kept = [self.store.commit("mine", self.store.stage("mine", f"<root>{i}</root>"))
                    for i in (1, 2, 3)]
            kept.append(self.store.delete("mine"))
        self.assertEqual(len(set(kept)), 4)
        self.assertEqual([read(p) for p in kept],
                         ["<root>0</root>", "<root>1</root>", "<root>2</root>", "<root>3</root>"])

    def test_a_replace_that_fails_leaves_the_old_mission_whole(self):
        self.put("mine", "<root>old</root>")
        staged = self.store.stage("mine", "<root>new</root>")
        with mock.patch("os.replace", side_effect=OSError("disk")):
            with self.assertRaises(OSError):
                self.store.commit("mine", staged)
        self.assertEqual(read(self.store.path("mine")), "<root>old</root>")
        self.store.discard(staged)
        self.assertFalse(os.path.exists(staged))
        self.store.discard(staged)                              # twice is fine

    def test_delete_moves_to_the_archive_and_removes_nothing(self):
        self.put("mine", "<root>keep me</root>")
        archived = self.store.delete("mine")
        self.assertEqual(self.store.user(), [])
        self.assertFalse(os.path.exists(self.store.path("mine")))
        self.assertEqual(read(archived), "<root>keep me</root>")
        self.assertRegex(os.path.basename(archived), r"^mine\.\d{8}-\d{6}\.xml$")


if __name__ == "__main__":
    unittest.main()
