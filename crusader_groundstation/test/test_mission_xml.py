"""mission_xml: blocks <-> XML and every reason to refuse a mission. Standard library only."""
import copy
import os
import re
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
from crusader_groundstation import mission_xml  # noqa: E402


def port(name, type_, default=None, dir_="in", doc="", **extra):
    return dict({"name": name, "dir": dir_, "type": type_, "default": default, "doc": doc}, **extra)


def node(id_, kind, *ports):
    return {"id": id_, "kind": kind, "ports": list(ports)}


def task(id_, task_no, tier, *ports):
    return {"id": id_, "task": task_no, "tier": tier, "ports": list(ports)}


# The tunables of the task trees, each with the default their wrapper in crusader_bt writes out.
PLAN_DRIVE = [port("orbit_radius_m", "double", ""), port("orbit_min_m", "double", "1.5"),
              port("overshoot_deg", "double", "45.0"), port("lookahead_m", "double", "2.5"),
              port("blocked_timeout_s", "double", "15.0"), port("plan_attempts", "int", "4")]
DOCK_FIRE = [port("vantage_standoff_m", "double", "5.0"), port("predock_m", "double", "3.0"),
             port("berth_m", "double", "1.25"), port("lead_m", "double", "2.0"),
             port("berth_tolerance_m", "double", "0.25"), port("spray_timeout_s", "double", "60.0"),
             port("camera_max_age_s", "double", "3.0")]

# What the boat's mission_catalogue answers, cut down to what these tests need. Shared with
# test_gcs_app, whose fake boat serves it.
CATALOGUE = {
    "nodes": [
        node("Sequence", "control"), node("ReactiveSequence", "control"),
        node("Fallback", "control"), node("IfThenElse", "control"),
        node("Inverter", "decorator"), node("ForceSuccess", "decorator"),
        node("RetryUntilSuccessful", "decorator", port("num_attempts", "int")),
        node("Timeout", "decorator", port("msec", "int")),
        node("Sleep", "action", port("msec", "int")),
        node("IsAutonomous", "condition"),
        node("HoldStation", "action"),
        node("StopBoat", "action"),
        node("WaitForRunStart", "action", port("timeout_s", "double", "0"),
             port("run_id", "std::string", None, "out")),
        node("TargetAhead", "action", port("range", "double", "50.0"),
             port("out", "Waypoint", None, "out")),
        node("NavigateTo", "action",
             port("target", "std::string", "waypoint", doc="port | waypoint | fix"),
             port("goal", "Waypoint"),
             port("lat", "double", "0.0"), port("lon", "double", "0.0"),
             port("tolerance", "double", "2.0"), port("avoid", "bool", "true")),
        node("CircleBuoy", "action", port("anchor", "std::string", "entry"),
             port("lat", "double", "0.0"), port("lon", "double", "0.0"),
             port("radius", "double", required=False)),
        # `choices` is optional: BT.CPP cannot say "one of", so a port only has it if the boat adds it
        node("ModeIs", "condition", port("mode", "enum", None, choices=["GUIDED", "MANUAL", "AUTO"])),
    ],
    "subtrees": [
        task("Task1Advanced", 1, "advanced", *PLAN_DRIVE),
        task("Task1Disruptive", 1, "disruptive", *PLAN_DRIVE, port("plan_max_age_s", "double", "15.0")),
        task("Task3Core", 3, "core", *DOCK_FIRE),
        task("Task3Advanced", 3, "advanced", *DOCK_FIRE, port("decode_timeout_s", "double", "80.0")),
        task("Task3Disruptive", 3, "disruptive", *DOCK_FIRE, port("decode_timeout_s", "double", "80.0")),
        # the shared bodies the task trees call: known to the boat, but no task and no tier
        {"id": "Task1PlanDrive", "ports": []}, {"id": "Task3DockFire", "ports": []},
    ],
    "builtins": ["nav_test_line", "task1_advanced", "task1_disruptive", "task3_core", "task3_advanced",
                 "task3_disruptive", "task3_fire_manual", "task3_fire_test"],
    "user": [],
}

# crusader_bt/behavior_trees/task1_advanced.xml as the boat ships it: includes, comments, the guard,
# one task block with every tunable written out
TASK1_ADVANCED_XML = """<?xml version="1.0" encoding="UTF-8"?>
<!-- MISSION: Task 1 Safe Passage, Advanced: wait for Ekko's field, plan the whole field, drive it. -->
<!--
  The tree is Task1Advanced, in lib/task1_advanced.xml. Its shared body, lib/common/task1_plan_drive.xml,
  is included once, here. This adds the autonomy guard; the task heartbeat is inside the tree.
  Every tunable is written out with its default.
-->
<root BTCPP_format="4" main_tree_to_execute="Main">
  <include path="lib/common/task1_plan_drive.xml"/>
  <include path="lib/task1_advanced.xml"/>

  <BehaviorTree ID="Main">
    <ReactiveSequence>
      <IsAutonomous/>
      <SubTree ID="Task1Advanced" _autoremap="true"
               orbit_radius_m=""
               orbit_min_m="1.5"
               overshoot_deg="45.0"
               lookahead_m="2.5"
               blocked_timeout_s="15.0"
               plan_attempts="4"/>
    </ReactiveSequence>
  </BehaviorTree>
</root>
"""

# crusader_bt/behavior_trees/nav_test_line.xml: a main tree that is not called Main, a named guard
# band, a leaf that points at a named place instead of a fix
NAV_TEST_LINE_XML = """<?xml version="1.0"?>
<root BTCPP_format="4" main_tree_to_execute="NavTestLine">
  <BehaviorTree ID="NavTestLine">
    <ReactiveSequence name="guard_band">
      <IsAutonomous/>
      <NavigateTo target="approach" tolerance="2.0"/>
    </ReactiveSequence>
  </BehaviorTree>
</root>
"""

GUARD_HEAD = ('<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">'
              "<ReactiveSequence><IsAutonomous/>")
GUARD_TAIL = "</ReactiveSequence></BehaviorTree></root>"


def gps(lat="1.3", lon="103.8", **ports):
    return {"id": "NavigateTo", "ports": dict(target="fix", lat=lat, lon=lon, **ports),
            "children": []}


def leaf(id_, **ports):
    return {"id": id_, "ports": ports, "children": []}


def sub(id_, **ports):
    return {"id": id_, "subtree": True, "ports": ports, "children": []}


def wrapped(body):
    return GUARD_HEAD + body + GUARD_TAIL


class Mission(unittest.TestCase):

    def cat(self, **changes):
        return dict(copy.deepcopy(CATALOGUE), **changes)

    def reasons(self, xml, name=None, catalogue=None):
        return mission_xml.validate(xml, catalogue or CATALOGUE, name)

    def refused(self, xml, needle, **kw):
        found = self.reasons(xml, **kw)
        self.assertTrue(any(needle in r for r in found), f"{needle!r} not in {found}")
        return found

    def built(self, blocks):
        return mission_xml.build_xml(blocks, CATALOGUE)


class Building(Mission):

    def test_the_guard_is_always_there_and_the_blocks_sit_in_one_sequence(self):
        xml = self.built([gps(), leaf("StopBoat")])
        self.assertIn("<IsAutonomous />", xml)
        self.assertEqual(self.reasons(xml), [])
        empty = self.built([])
        self.assertIn('<Sequence name="mission" />', empty)

    def test_no_include_and_no_model_in_what_it_builds(self):
        xml = self.built([sub("Task1Advanced")])
        self.assertNotIn("<include", xml)
        self.assertNotIn("TreeNodesModel", xml)

    def test_a_task_block_gets_autoremap_and_every_tunable_with_its_default(self):
        xml = self.built([sub("Task1Advanced", lookahead_m="3")])
        self.assertIn('<SubTree ID="Task1Advanced" _autoremap="true"', xml)
        self.assertIn('orbit_radius_m=""', xml)         # an empty default is written, not dropped
        self.assertIn('lookahead_m="3.0"', xml)          # 3 would load as a whole number
        self.assertIn('plan_attempts="4"', xml)
        self.assertEqual(self.reasons(xml), [])

    def test_a_whole_number_port_loses_a_trailing_point(self):
        xml = self.built([sub("Task1Advanced", plan_attempts="6.0")])
        self.assertIn('plan_attempts="6"', xml)

    def test_empty_leaf_ports_are_left_out_so_their_defaults_apply(self):
        xml = self.built([gps(tolerance="", avoid="false")])
        self.assertNotIn("tolerance", xml)
        self.assertIn('avoid="false"', xml)

    def test_blocks_that_are_not_blocks_are_a_build_error(self):
        for bad in ([{"id": "a b"}], [{"id": "Sequence", "ports": {"x y": "1"}}],
                    [{"id": "Sequence", "ports": {"x": "line\nbreak"}}], ["Sequence"],
                    [{"id": "Sequence", "children": "no"}], {"id": "Sequence"}):
            with self.subTest(bad=bad):
                with self.assertRaises(mission_xml.BlockError):
                    self.built(bad)

    def test_too_deep_or_too_many_blocks_are_a_build_error(self):
        deep = leaf("StopBoat")
        for _ in range(mission_xml.MAX_DEPTH + 1):
            deep = {"id": "Sequence", "ports": {}, "children": [deep]}
        with self.assertRaises(mission_xml.BlockError):
            self.built([deep])
        with self.assertRaises(mission_xml.BlockError):
            self.built([leaf("StopBoat")] * (mission_xml.MAX_BLOCKS + 1))

    def test_numbers_from_json_become_text(self):
        xml = self.built([{"id": "Sleep", "ports": {"msec": 500}, "children": []}])
        self.assertIn('msec="500"', xml)


class RoundTrip(Mission):

    def blocks(self):
        return [gps(), {"id": "Fallback", "ports": {}, "name": "try", "children": [
                    leaf("HoldStation"),
                    {"id": "Timeout", "ports": {"msec": "2000"}, "children": [leaf("StopBoat")]}]},
                sub("Task3Core", **{p["name"]: p["default"] for p in DOCK_FIRE})]     # as built writes it

    def test_build_then_read_gives_the_blocks_back(self):
        blocks, notes = mission_xml.read_blocks(self.built(self.blocks()))
        self.assertEqual(blocks, self.blocks())
        self.assertEqual(notes, [])

    def test_read_then_build_gives_the_file_back(self):
        xml = self.built(self.blocks())
        blocks, _ = mission_xml.read_blocks(xml)
        self.assertEqual(self.built(blocks), xml)

    def test_a_body_that_is_not_the_editors_sequence_is_kept_whole(self):
        xml = wrapped('<SubTree ID="Task3Core" _autoremap="true"/>')
        blocks, _ = mission_xml.read_blocks(xml)
        self.assertEqual(blocks, [sub("Task3Core")])

    def test_generic_tags_read_as_their_id(self):
        xml = wrapped('<Sequence><Action ID="StopBoat"/></Sequence>')
        self.assertEqual(mission_xml.read_blocks(xml)[0], [leaf("StopBoat")])

    def test_a_built_in_opens_without_its_guard_and_its_includes(self):
        builtin = ('<?xml version="1.0"?><!-- MISSION: x --><root BTCPP_format="4" '
                   'main_tree_to_execute="Main"><include path="lib/a.xml"/><include path="lib/b.xml"/>'
                   '<BehaviorTree ID="Main"><ReactiveSequence name="guard"><IsAutonomous/>'
                   '<Sequence name="mission"><WaitForRunStart timeout_s="0.0"/>'
                   '<SubTree ID="Task3Core" _autoremap="true" berth_m="1.25"/></Sequence>'
                   '</ReactiveSequence></BehaviorTree>'
                   '<BehaviorTree ID="Other"><StopBoat/></BehaviorTree></root>')
        blocks, notes = mission_xml.read_blocks(builtin)
        self.assertEqual(blocks, [leaf("WaitForRunStart", timeout_s="0.0"),
                                  sub("Task3Core", berth_m="1.25")])
        self.assertEqual(len(notes), 2)
        self.assertIn("2 <include>", notes[0])
        self.assertIn("1 other tree", notes[1])

    def test_the_real_task_wrapper_opens_as_one_task_block_with_its_tunables(self):
        blocks, notes = mission_xml.read_blocks(TASK1_ADVANCED_XML)
        self.assertEqual([b["id"] for b in blocks], ["Task1Advanced"])
        self.assertEqual(blocks[0]["ports"], {p["name"]: p["default"] for p in PLAN_DRIVE})
        self.assertIn("2 <include>", notes[0])
        self.assertEqual(self.reasons(self.built(blocks)), [])           # and builds back into a mission
        group, _ = mission_xml.read_blocks(TASK1_ADVANCED_XML, group="task1_advanced")
        self.assertEqual(self.reasons(self.built(group)), [])

    def test_a_main_tree_with_another_name_and_a_named_guard_opens_too(self):
        blocks, notes = mission_xml.read_blocks(NAV_TEST_LINE_XML)
        self.assertEqual(blocks, [leaf("NavigateTo", target="approach", tolerance="2.0")])
        self.assertEqual(notes, [])
        self.assertEqual(self.reasons(self.built(blocks)), [])

    def test_a_tree_without_the_autonomy_guard_keeps_its_whole_body(self):
        manual = ('<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M">'
                  '<ReactiveSequence name="guard_band"><ModeIs mode="MANUAL"/><StopBoat/>'
                  "</ReactiveSequence></BehaviorTree></root>")
        blocks, _ = mission_xml.read_blocks(manual)
        self.assertEqual([b["id"] for b in blocks], ["ReactiveSequence"])
        self.assertEqual([c["id"] for c in blocks[0]["children"]], ["ModeIs", "StopBoat"])

    def test_inserting_wraps_the_blocks_in_a_named_group(self):
        blocks, _ = mission_xml.read_blocks(self.built(self.blocks()), group="tour")
        self.assertEqual(len(blocks), 1)
        self.assertEqual((blocks[0]["id"], blocks[0]["name"]), ("Sequence", "tour"))
        self.assertEqual(blocks[0]["children"], self.blocks())
        self.assertEqual(self.reasons(self.built(blocks)), [])

    def test_an_unreadable_file_is_a_value_error(self):
        for bad in ("<root", "<foo/>", "<root><BehaviorTree ID='a'/><BehaviorTree ID='b'/></root>",
                    '<!DOCTYPE x [<!ENTITY a "b">]><root/>', 5,
                    '<root main_tree_to_execute="X"><BehaviorTree ID="Y"/></root>',
                    wrapped('<SubTree _autoremap="true"/>')):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    mission_xml.read_blocks(bad)


class Refusals(Mission):

    def ok_xml(self):
        return self.built([gps(), leaf("StopBoat")])

    def test_a_good_mission_is_accepted(self):
        self.assertEqual(self.reasons(self.ok_xml(), "my_tour"), [])

    def test_names(self):
        for bad in ("", "Tour", "1tour", "my-tour", "my tour", "a" * 41, "tour\n", "_x", 7):
            with self.subTest(name=bad):
                self.assertEqual(len(self.reasons(self.ok_xml(), bad)), 1)
        for good in ("t", "a" * 40, "my_tour_2"):
            with self.subTest(name=good):
                self.assertEqual(self.reasons(self.ok_xml(), good), [])

    def test_a_name_may_not_be_a_built_in(self):
        for name in CATALOGUE["builtins"]:
            with self.subTest(name=name):
                self.refused(self.ok_xml(), "built-in", name=name)

    def test_more_than_256_kib(self):
        pad = "<!-- " + "x" * (mission_xml.MAX_BYTES) + " -->"
        found = self.reasons(self.ok_xml() + pad)
        self.assertEqual(len(found), 1)
        self.assertIn("256 KiB", found[0])
        just_under = self.ok_xml() + "<!-- " + "x" * 1000 + " -->"
        self.assertEqual(self.reasons(just_under), [])

    def test_any_include(self):
        self.refused('<root BTCPP_format="4"><include path="lib/x.xml"/></root>', "<include>")
        self.refused(wrapped('<Sequence><include path="x"/><StopBoat/></Sequence>'), "<include>")
        self.refused(self.ok_xml() + "<!-- <include path='x'/> -->", "<include>")

    def test_top_level_must_be_exactly_the_guard(self):
        tail = "<Sequence><StopBoat/></Sequence>"
        for xml in (
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\">"
                "<Sequence><StopBoat/></Sequence></BehaviorTree></root>",
                wrapped(tail).replace("ReactiveSequence", "Sequence"),
                wrapped(tail).replace("<IsAutonomous/>", "<StopBoat/>"),
                wrapped(tail).replace("<IsAutonomous/>", ""),
                wrapped(tail).replace("<IsAutonomous/>", '<IsAutonomous x="1"/>'),
                wrapped(tail).replace('"Main">', '"Other">'),
                wrapped(tail).replace('main_tree_to_execute="Main"', 'main_tree_to_execute="Other"'),
                wrapped(tail).replace('BTCPP_format="4"', 'BTCPP_format="3"'),
                wrapped(tail).replace("</BehaviorTree>", "</BehaviorTree><BehaviorTree ID=\"B\"/>"),
                wrapped(tail).replace("<BehaviorTree", "<TreeNodesModel/><BehaviorTree"),
                "<root BTCPP_format=\"4\" main_tree_to_execute=\"Main\"><BehaviorTree ID=\"Main\"/></root>",
                "<tree/>"):
            with self.subTest(xml=xml[:90]):
                self.refused(xml, "autonomy guard")
        self.assertEqual(self.reasons(wrapped(tail)), [])

    def test_the_guard_may_carry_a_label(self):
        xml = wrapped("<Sequence><StopBoat/></Sequence>").replace(
            "<ReactiveSequence>", '<ReactiveSequence name="guard">')
        self.assertEqual(self.reasons(xml), [])

    def test_blocks_must_share_one_sequence(self):
        self.refused(wrapped("<StopBoat/><StopBoat/>"), "one Sequence")

    def test_an_empty_mission(self):
        self.refused(self.built([]), "empty")
        self.refused(GUARD_HEAD + GUARD_TAIL, "empty")

    def test_malformed_xml_and_entities(self):
        self.refused("<root><unclosed></root>", "not well-formed")
        self.refused('<!DOCTYPE r [<!ENTITY a "b">]>' + self.ok_xml(), "DOCTYPE")
        self.refused(None, "not text")

    def test_a_gps_waypoint_needs_a_real_place(self):
        cases = [
            (gps(lat="0", lon="0"), "both 0"),
            (gps(lat="0.0", lon="0.00"), "both 0"),
            ({"id": "NavigateTo", "ports": {"target": "fix"}, "children": []}, "needs a lat"),
            ({"id": "NavigateTo", "ports": {"target": "fix", "lat": "1.3"}, "children": []},
             "needs a lon"),
            (gps(lat="abc"), "not a number"),
            (gps(lon="{lon}"), "not a number"),
            (gps(lat="nan"), "not a number"),
            (gps(lat="1_0"), "not a number"),
            (gps(lat="91"), "outside"),
            (gps(lon="-181"), "outside"),
            ({"id": "CircleBuoy", "ports": {"anchor": "fix", "lat": "0", "lon": "0"},
              "children": []}, "both 0"),
        ]
        for block, needle in cases:
            with self.subTest(needle=needle, block=block["ports"]):
                self.refused(self.built([block]), needle)

    def test_a_waypoint_on_the_equator_or_the_meridian_is_a_place(self):
        for block in (gps(lat="0", lon="103.8"), gps(lat="1.3", lon="0")):
            self.assertEqual(self.reasons(self.built([block])), [])

    def test_a_nonzero_point_is_only_demanded_of_a_fix(self):
        plain = {"id": "NavigateTo", "ports": {"target": "home"}, "children": []}
        self.assertEqual(self.reasons(self.built([plain])), [])

    def test_target_port_needs_a_goal(self):
        block = {"id": "NavigateTo", "ports": {"target": "port"}, "children": []}
        self.refused(self.built([block]), "needs goal")
        block["ports"]["goal"] = "{goal}"
        self.assertEqual(self.reasons(self.built([block])), [])

    def test_two_tiers_of_one_task(self):
        both = [sub(i)
                for i in ("Task1Advanced", "Task1Disruptive")]
        self.refused(self.built(both), "Task 1 is used at 2 tiers")
        self.refused(self.built(both + [leaf("StopBoat")]), "one tier of a task")

    def test_the_same_block_twice_and_other_tasks_are_fine(self):
        twice = [sub("Task1Advanced")] * 2
        self.assertEqual(self.reasons(self.built(twice)), [])
        mixed = [sub("Task1Advanced"),
                 sub("Task3Core")]
        self.assertEqual(self.reasons(self.built(mixed)), [])
        self.refused(self.built(mixed + [sub("Task3Advanced")]), "Task 3 is used at 2 tiers")

    def test_a_shared_body_is_a_known_block_with_no_tier(self):
        body = sub("Task1PlanDrive")
        advanced = sub("Task1Advanced")
        disruptive = sub("Task1Disruptive")
        self.assertEqual(self.reasons(self.built([body, advanced])), [])
        self.assertEqual(self.reasons(self.built([body, body])), [])
        self.refused(self.built([body, advanced, disruptive]), "Task 1 is used at 2 tiers")

    def test_a_block_that_is_not_in_the_catalogue(self):
        self.refused(self.built([leaf("Teleport")]), "unknown block 'Teleport'")
        self.refused(wrapped('<Sequence><Action ID="Teleport"/></Sequence>'), "'Teleport'")
        self.refused(self.built([sub("Task9Core")]), "not a task block the boat offers")

    def test_a_catalogue_without_the_block_refuses_it(self):
        thin = self.cat(nodes=[n for n in CATALOGUE["nodes"] if n["id"] != "StopBoat"])
        self.refused(self.ok_xml(), "'StopBoat'", catalogue=thin)

    def test_a_required_port_left_empty(self):
        for block in ({"id": "Timeout", "ports": {}, "children": [leaf("StopBoat")]},
                      {"id": "Timeout", "ports": {"msec": ""}, "children": [leaf("StopBoat")]},
                      {"id": "Timeout", "ports": {"msec": "  "}, "children": [leaf("StopBoat")]},
                      leaf("Sleep")):
            with self.subTest(block=block["id"], ports=block["ports"]):
                found = self.refused(self.built([block]), "required port 'msec' is empty")
                self.assertEqual(len(found), 1)

    def test_a_required_port_filled_by_a_blackboard_entry_is_filled(self):
        block = {"id": "Sleep", "ports": {"msec": "{wait}"}, "children": []}
        self.assertEqual(self.reasons(self.built([block])), [])

    def test_optional_ports_are_not_required(self):
        # goal has an unwired type, radius says required: false, lat/lon have defaults
        circle = {"id": "CircleBuoy", "ports": {"anchor": "entry"}, "children": []}
        self.assertEqual(self.reasons(self.built([circle])), [])
        wp = {"id": "NavigateTo", "ports": {"target": "home"}, "children": []}
        self.assertEqual(self.reasons(self.built([wp])), [])

    def test_a_required_scalar_without_a_flag_is_required_but_not_an_unwired_type(self):
        bare = self.cat(nodes=[node("Sequence", "control"), node("IsAutonomous", "condition"),
                               node("Radius", "action", port("radius", "double"),
                                    port("goal", "Waypoint"))])
        found = mission_xml.validate(mission_xml.build_xml([leaf("Radius")], bare), bare)
        self.assertEqual(len(found), 1)
        self.assertIn("required port 'radius'", found[0])
        flagged = copy.deepcopy(bare)
        flagged["nodes"][2]["ports"][0]["required"] = False
        self.assertEqual(mission_xml.validate(mission_xml.build_xml([leaf("Radius")], flagged),
                                              flagged), [])

    def test_a_required_task_port_left_empty(self):
        needs = self.cat(subtrees=[task("Task3Core", 3, "core", port("bay", "int"))])
        xml = mission_xml.build_xml([sub("Task3Core")], needs)
        self.refused(xml, "required port 'bay'", catalogue=needs)

    def test_port_values_must_fit_their_type(self):
        for block, needle in (
                (leaf("Sleep", msec="soon"), "whole number"),
                (leaf("Sleep", msec="1.5"), "whole number"),
                (gps(tolerance="wide"), "must be a number"),
                (gps(avoid="maybe"), "true or false"),
                (leaf("WaitForRunStart", run_id="plain"), "is an output"),
                (leaf("ModeIs", mode="SPACE"), "one of GUIDED, MANUAL, AUTO"),
                (leaf("StopBoat", colour="red"), "unknown port 'colour'"),
                (leaf("StopBoat", **{"_skipIf": "x"}), "unknown port '_skipIf'")):
            with self.subTest(needle=needle):
                self.refused(self.built([block]), needle)
        self.assertEqual(self.reasons(self.built([leaf("WaitForRunStart", run_id="{run}")])), [])

    def test_a_task_block_literal_is_typed_by_its_text(self):
        def with_(**ports):
            xml = self.built([sub("Task1Advanced")])
            for name, value in ports.items():          # as someone writing the XML by hand would
                xml = xml.replace(f'{name}="2.5"', f'{name}="{value}"') \
                         .replace(f'{name}="4"', f'{name}="{value}"')
            return xml
        self.refused(with_(lookahead_m="3"), "decimal point")
        self.refused(with_(lookahead_m="far"), "decimal point")
        self.refused(with_(plan_attempts="4.0"), "whole number")
        self.assertEqual(self.reasons(with_(lookahead_m="3.0", plan_attempts="5")), [])

    def test_a_task_block_without_autoremap_and_with_children(self):
        xml = self.built([sub("Task3Core")])
        self.refused(xml.replace(' _autoremap="true"', ""), "_autoremap")
        self.refused(re.sub(r"(<SubTree [^>]*?)\s*/>", r"\1><StopBoat/></SubTree>", xml), "cannot hold")

    def test_how_many_children_a_block_takes(self):
        self.refused(self.built([{"id": "Inverter", "ports": {}, "children": []}]),
                     "exactly one child")
        self.refused(self.built([{"id": "Inverter", "ports": {},
                                  "children": [leaf("StopBoat")] * 2}]), "exactly one child")
        self.refused(self.built([{"id": "StopBoat", "ports": {}, "children": [leaf("StopBoat")]}]),
                     "leaf")
        self.refused(self.built([{"id": "IfThenElse", "ports": {}, "children": [leaf("StopBoat")]}]),
                     "2 to 3 children")
        self.assertEqual(self.reasons(self.built([{"id": "IfThenElse", "ports": {}, "children": [
            leaf("HoldStation"), leaf("StopBoat")]}])), [])

    def test_a_reason_says_where_it_is(self):
        xml = self.built([leaf("StopBoat"), {"id": "Fallback", "name": "tries", "ports": {},
                                             "children": [gps(lat="0", lon="0")]}])
        self.assertEqual(self.reasons(xml)[0].split(": ")[0], "mission > tries[2] > NavigateTo[1]")

    def test_the_catalogue_must_be_one(self):
        for bad in (None, {}, {"nodes": 3}, {"nodes": [1]}, []):
            with self.subTest(bad=bad):
                with self.assertRaises(ValueError):
                    mission_xml.validate(self.ok_xml(), bad)

    def test_too_deeply_nested_xml_is_refused_not_crashed_on(self):
        xml = wrapped("<Sequence>" + "<Sequence>" * 100 + "<StopBoat/>" + "</Sequence>" * 101)
        self.refused(xml, "nested more than")


class ForThePage(Mission):

    def ports(self, entry_id, notes):
        entries = notes["nodes"] + notes["subtrees"]
        return {p["name"]: p for e in entries if e["id"] == entry_id for p in e["ports"]}

    def test_annotate_marks_each_port_required_or_not_and_its_kind(self):
        notes = mission_xml.annotate(CATALOGUE)
        nav = self.ports("NavigateTo", notes)
        self.assertEqual([(n, nav[n]["required"], nav[n]["kind"]) for n in ("goal", "lat", "avoid")],
                         [("goal", False, "other"), ("lat", False, "real"), ("avoid", False, "bool")])
        self.assertEqual(self.ports("Sleep", notes)["msec"]["required"], True)
        self.assertEqual(self.ports("CircleBuoy", notes)["radius"]["required"], False)   # said so
        self.assertEqual(self.ports("WaitForRunStart", notes)["run_id"]["required"], False)   # an output
        self.assertEqual(self.ports("ModeIs", notes)["mode"]["kind"], "text")

    def test_annotate_says_how_many_blocks_each_can_hold(self):
        notes = mission_xml.annotate(CATALOGUE)
        held = {n["id"]: n["max_children"] for n in notes["nodes"]}
        self.assertEqual((held["Sequence"], held["Inverter"], held["StopBoat"], held["IsAutonomous"],
                          held["IfThenElse"]), (None, 1, 0, 0, 3))
        self.assertEqual({s["max_children"] for s in notes["subtrees"]}, {0})

    def test_annotate_names_the_gps_selectors_and_leaves_the_original_alone(self):
        before = copy.deepcopy(CATALOGUE)
        notes = mission_xml.annotate(CATALOGUE)
        self.assertEqual(notes["fix_selector"], {"NavigateTo": "target", "CircleBuoy": "anchor"})
        self.assertEqual(CATALOGUE, before)
        self.assertEqual(notes["builtins"], CATALOGUE["builtins"])

    def test_what_annotate_says_is_what_validate_enforces(self):
        notes = mission_xml.annotate(CATALOGUE)
        for entry in notes["nodes"]:
            wants = [p["name"] for p in entry["ports"] if p["required"]]
            if not wants or entry["id"] in ("Timeout", "RetryUntilSuccessful"):
                continue
            xml = self.built([leaf(entry["id"])])
            found = " ".join(self.reasons(xml))
            for name in wants:
                self.assertIn(f"required port '{name}' is empty", found)


class Problems(Mission):

    def test_paths_point_at_the_editors_blocks(self):
        blocks = [leaf("StopBoat"),
                  {"id": "Fallback", "ports": {}, "children": [leaf("HoldStation"),
                                                               gps(lat="0", lon="0")]},
                  {"id": "Timeout", "ports": {}, "children": [leaf("StopBoat")]}]
        xml, problems = mission_xml.build(blocks, CATALOGUE)
        self.assertEqual(xml, self.built(blocks))
        self.assertEqual([p["path"] for p in problems], [[1, 1], [2]])
        self.assertTrue(all(isinstance(p["msg"], str) for p in problems))

    def test_whole_mission_problems_have_no_path(self):
        _xml, problems = mission_xml.build([], CATALOGUE, "Bad Name")
        self.assertEqual(sorted(str(p["path"]) for p in problems), ["None", "None"])
        self.assertTrue(any("must be lower case" in p["msg"] for p in problems))
        self.assertTrue(any("empty" in p["msg"] for p in problems))

    def test_a_good_draft_has_no_problems_and_survives_json(self):
        import json
        blocks = [gps(), leaf("StopBoat")]
        _xml, problems = mission_xml.build(json.loads(json.dumps(blocks)), CATALOGUE, "tour")
        self.assertEqual(problems, [])


if __name__ == "__main__":
    unittest.main()
