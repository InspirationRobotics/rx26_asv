"""mission_lib off-boat: the catalogue against the C++, the shipped trees, and every validate rule.

    python -m pytest crusader_groundstation/test/test_mission_lib.py -q        (from rx26_asv)
    python -m unittest discover -s crusader_groundstation/test                 (no pytest needed)

Stdlib only, like the module. The C++ is read as text: a leaf registered or a port added there
and not in the catalogue table fails here, not on the water.
"""
import glob
import json
import os
import re
import sys
import tempfile
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from crusader_groundstation import mission_lib as ml  # noqa: E402

REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
BT_DIR = os.path.join(REPO, "crusader_bt", "behavior_trees")
SRC_DIR = os.path.join(REPO, "crusader_bt", "src")
OCS_LINK = os.path.join(REPO, "crusader_groundstation", "crusader_groundstation", "ocs_link.py")

CAT = ml.catalogue(BT_DIR)
NODES = {n["id"]: n for n in CAT["nodes"]}
# Blocks the catalogue lists before their C++ lands (crusader_bt is edited in parallel).
ANNOUNCED = {"WaitForRunStart", "TaskAttempt", "SetTier"}
BASE_KIND = {"CrusaderCondition": "condition", "CrusaderAction": "action",
             "CrusaderSyncAction": "action"}
CPP_TYPE = {"double": {"double"}, "int": {"int"}, "bool": {"bool"}, "std::string": {"string", "enum"},
            "Waypoint": {"bb"}}


def mission(body, tree_id="Main", extra=""):
    """A one-tree mission around `body`, with `extra` as further root children."""
    return ('<root BTCPP_format="4" main_tree_to_execute="%s">'
            '<BehaviorTree ID="%s">%s</BehaviorTree>%s</root>' % (tree_id, tree_id, body, extra))


def check(xml):
    return ml.validate(xml, CAT)


def has(messages, fragment):
    return any(fragment in m for m in messages)


# ------------------------------------------------------------------- the C++, read as text

def _cpp_text():
    texts = {}
    for path in sorted(glob.glob(os.path.join(SRC_DIR, "*.cpp"))):
        with open(path, encoding="utf-8", errors="replace") as f:
            raw = f.read()
        raw = re.sub(r"/\*.*?\*/", "", raw, flags=re.S)
        texts[path] = re.sub(r"//[^\n]*", "", raw)
    return texts


def cpp_registry():
    """{node id: (class name, file text)} for every factory.registerNodeType<Class>("Id")."""
    found = {}
    for text in _cpp_text().values():
        for cls, nid in re.findall(r'registerNodeType<(\w+)>\(\s*"(\w+)"\s*\)', text):
            found[nid] = (cls, text)
    return found


def cpp_class(cls, text):
    """(base class, [(direction, C++ type, port name)]) for `class cls : public Base`."""
    m = re.search(r"\bclass\s+%s\s*(?:final\s*)?:\s*public\s+([\w:]+)" % cls, text)
    if not m:
        return None, []
    nxt = re.search(r"\n(?:class|struct)\s+\w+", text[m.end():])
    body = text[m.end(): m.end() + nxt.start()] if nxt else text[m.end():]
    ports = []
    start = body.find("providedPorts")
    if start >= 0:
        fn = body[start:]
        ports = [(d, t.strip(), n) for d, t, n in re.findall(
            r'BT::(Input|Output|Bidirectional)Port<([^>]+)>\(\s*"(\w+)"', fn)]
    return m.group(1), ports


class CatalogueVsCpp(unittest.TestCase):
    def test_every_registered_leaf_is_in_the_catalogue(self):
        registry = cpp_registry()
        self.assertGreater(len(registry), 50, "found too few registerNodeType calls")
        self.assertEqual(sorted(set(registry) - set(NODES)), [])

    def test_only_announced_blocks_lack_a_registration(self):
        listed = {n["id"] for n in CAT["nodes"] if n["category"] not in ("flow", "decorator")
                  and n["id"] != "SubTree"}
        self.assertEqual(sorted(listed - set(cpp_registry()) - ANNOUNCED), [])

    def test_ports_match_provided_ports(self):
        for nid, (cls, text) in cpp_registry().items():
            base, ports = cpp_class(cls, text)
            self.assertIsNotNone(base, "no class %s found for %s" % (cls, nid))
            listed = {(p["direction"], p["name"]): p for p in NODES[nid]["ports"]}
            source = {("out" if d == "Output" else "in", n): t for d, t, n in ports}
            self.assertEqual(sorted(listed), sorted(source), "ports of %s" % nid)
            for key, cpp_type in source.items():
                if key[0] == "in":
                    self.assertIn(listed[key]["type"], CPP_TYPE[cpp_type], "%s.%s" % (nid, key[1]))

    def test_kind_follows_the_base_class(self):
        for nid, (cls, text) in cpp_registry().items():
            base, _ = cpp_class(cls, text)
            if base in BASE_KIND:
                self.assertEqual(NODES[nid]["kind"], BASE_KIND[base], nid)

    def test_user_facing_blocks_are_not_advanced(self):
        shown = {"IsAutonomous", "WaitForRunStart", "TaskAttempt", "SetTier", "NavigateTo",
                 "CircleBuoy", "HoldStation", "StopBoat", "TierIs", "TierAtLeast", "ModeIs"}
        for n in CAT["nodes"]:
            if n["category"] in ("task1", "task3", "fire"):
                self.assertTrue(n["advanced"], n["id"])
            if n["id"] in shown:
                self.assertFalse(n["advanced"], n["id"])

    def test_rx_tasks_match_ocs_link(self):
        with open(OCS_LINK, encoding="utf-8") as f:
            block = re.search(r"RX_TASKS = frozenset\(\((.*?)\)\)", f.read(), re.S).group(1)
        self.assertEqual(set(re.findall(r'"(TASK_\w+)"', block)),
                         set(NODES["TaskAttempt"]["ports"][0]["choices"]))
        self.assertEqual(NODES["SetTask"]["ports"][0]["choices"],
                         NODES["TaskAttempt"]["ports"][0]["choices"])


class CatalogueShape(unittest.TestCase):
    def test_json_serialisable_and_versioned(self):
        again = json.loads(json.dumps(CAT))
        self.assertEqual(again["version"], ml.CATALOGUE_VERSION)
        for key in ("categories", "nodes", "presets", "subtrees"):
            self.assertIsInstance(again[key], list)

    def test_categories_and_node_fields(self):
        cats = {c["id"]: c["color"] for c in CAT["categories"]}
        self.assertEqual(cats["flow"], "#e6a23c")
        self.assertEqual(cats["task1"], "#cf63cf")
        self.assertEqual(len(cats), 10)
        for n in CAT["nodes"]:
            self.assertIn(n["category"], cats, n["id"])
            self.assertIn(n["kind"], ("control", "decorator", "action", "condition", "subtree"))
            self.assertIn(n["children"], ("none", "one", "many"))
            self.assertTrue(n["desc"], n["id"])
            for p in n["ports"]:
                self.assertIn(p["type"], ("string", "int", "double", "bool", "enum", "bb"))
                self.assertEqual("choices" in p, p["type"] == "enum")

    def test_builtin_ports_use_the_real_names(self):
        def names(nid):
            return [p["name"] for p in NODES[nid]["ports"]]
        self.assertEqual(names("Parallel"), ["success_count", "failure_count"])
        self.assertEqual(names("ParallelAll"), ["max_failures"])
        self.assertEqual(names("RetryUntilSuccessful"), ["num_attempts"])
        self.assertEqual(names("Repeat"), ["num_cycles"])
        self.assertEqual(names("Timeout"), ["msec"])
        self.assertEqual(names("Delay"), ["delay_msec"])
        self.assertEqual(names("RunOnce"), ["then_skip"])
        self.assertEqual(names("Sleep"), ["msec"])
        self.assertEqual(NODES["ReactiveSequence"]["children"], "many")
        self.assertEqual(NODES["Inverter"]["children"], "one")
        self.assertIn("guard", NODES["ReactiveSequence"]["desc"])

    def test_new_robocommand_blocks(self):
        wait = {p["name"]: p for p in NODES["WaitForRunStart"]["ports"]}
        self.assertEqual((wait["max_age_s"]["default"], wait["timeout_s"]["default"],
                          wait["hold"]["default"]), ("30", "0", "true"))
        self.assertEqual(wait["run_id"]["direction"], "out")
        self.assertEqual(NODES["TaskAttempt"]["kind"], "decorator")
        self.assertEqual(NODES["SetTier"]["ports"][0]["choices"], ["core", "advanced", "disruptive"])
        for nid in ANNOUNCED:
            self.assertEqual(NODES[nid]["category"], "robocommand")

    def test_presets_point_at_real_ports(self):
        presets = {p["id"]: p for p in CAT["presets"]}
        self.assertEqual(set(presets), {"GpsWaypoint", "ReturnHome", "OrbitGpsPoint", "HoldPosition"})
        self.assertEqual(presets["GpsWaypoint"]["fixed"], {"target": "fix"})
        self.assertEqual(presets["OrbitGpsPoint"]["fixed"], {"anchor": "fix"})
        for p in CAT["presets"]:
            ports = {q["name"]: q for q in NODES[p["preset_of"]]["ports"]}
            self.assertTrue(set(p["show_ports"]) <= set(ports), p["id"])
            for attr, value in p["fixed"].items():
                self.assertIn(value, ports[attr]["choices"], p["id"])

    def test_a_missing_bt_dir_gives_an_empty_subtree_list(self):
        cat = ml.catalogue(os.path.join(tempfile.gettempdir(), "no_such_bt_dir_xyz"))
        self.assertEqual((cat["subtrees"], cat["problems"]), ([], []))


# -------------------------------------------------------------------------- shipped trees

class ShippedTrees(unittest.TestCase):
    def test_every_tree_validates_flattened(self):
        files = sorted(glob.glob(os.path.join(BT_DIR, "*.xml")))
        self.assertTrue(files)
        for path in files:
            verdict = check(ml.flatten(path))
            self.assertEqual(verdict["errors"], [], os.path.basename(path))

    def test_every_lib_subtree_drops_into_a_valid_mission(self):
        self.assertEqual(CAT["problems"], [])
        for sub in CAT["subtrees"]:
            call = '<SubTree ID="%s" _autoremap="true"%s/>' % (sub["id"], "".join(
                ' %s="%s"' % (p["name"], p["default"]) for p in sub["ports"]
                if p["default"] is not None))
            xml = ('<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">'
                   '%s</BehaviorTree>%s</root>' % (call, sub["xml"]))
            self.assertEqual(check(xml)["errors"], [], sub["id"])
            self.assertEqual({p["type"] for p in sub["ports"]} - {"string", "int", "double", "bool"},
                             set())

    def test_list_missions_reads_the_shipped_folder(self):
        rows = ml.list_missions(BT_DIR, os.path.join(tempfile.gettempdir(), "no_missions_here"))
        names = [r["name"] for r in rows]
        self.assertEqual(names, sorted(names))
        self.assertTrue(all(r["source"] == "prebuilt" and r["main_tree"] for r in rows))
        self.assertNotIn("lib", names)


# -------------------------------------------------------------------------------- flatten

LIB = ('<?xml version="1.0"?>\n<!-- Lib: a subtree -->\n<root BTCPP_format="4">\n'
       '  <BehaviorTree ID="Lib">\n    <Sequence>\n      <StopBoat/>\n    </Sequence>\n'
       '  </BehaviorTree>\n  <TreeNodesModel>\n    <SubTree ID="Lib">\n'
       '      <input_port name="n" default="8">how many</input_port>\n    </SubTree>\n'
       '  </TreeNodesModel>\n</root>\n')
MAIN = ('<?xml version="1.0"?>\n<!-- MISSION: runs lib -->\n'
        '<root BTCPP_format="4" main_tree_to_execute="Main">\n  <include path="lib/lib.xml"/>\n'
        '  <BehaviorTree ID="Main">\n    <SubTree ID="Lib" _autoremap="true" n="8"/>\n'
        '  </BehaviorTree>\n</root>\n')


def write(directory, rel, text):
    path = os.path.join(directory, *rel.split("/"))
    os.makedirs(os.path.dirname(path), exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        f.write(text)
    return path


class Flatten(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = self._tmp.name
        self.addCleanup(self._tmp.cleanup)

    def test_inlines_an_include_relative_to_the_including_file(self):
        write(self.dir, "lib/lib.xml", LIB)
        out = ml.flatten(write(self.dir, "main.xml", MAIN))
        self.assertNotIn("<include", out)
        self.assertIn('main_tree_to_execute="Main"', out)
        self.assertIn("MISSION: runs lib", out)
        verdict = check(out)
        self.assertEqual((verdict["errors"], verdict["trees"]), ([], ["Lib", "Main"]))
        self.assertEqual(out.count("<TreeNodesModel"), 1)

    def test_nested_include_is_relative_to_its_own_file(self):
        write(self.dir, "lib/inner/leaf.xml",
              '<root BTCPP_format="4"><BehaviorTree ID="Leaf"><StopBoat/></BehaviorTree></root>')
        write(self.dir, "lib/mid.xml", '<root BTCPP_format="4"><include path="inner/leaf.xml"/>'
              '<BehaviorTree ID="Mid"><SubTree ID="Leaf" _autoremap="true"/></BehaviorTree></root>')
        out = ml.flatten(write(self.dir, "main.xml", '<root BTCPP_format="4" '
                               'main_tree_to_execute="Main"><include path="lib/mid.xml"/>'
                               '<BehaviorTree ID="Main"><SubTree ID="Mid" _autoremap="true"/>'
                               '</BehaviorTree></root>'))
        self.assertEqual(check(out)["trees"], ["Leaf", "Mid", "Main"])

    def test_the_same_include_twice_is_one_copy(self):
        write(self.dir, "lib/lib.xml", LIB)
        twice = MAIN.replace('<include path="lib/lib.xml"/>',
                             '<include path="lib/lib.xml"/><include path="lib/lib.xml"/>')
        out = ml.flatten(write(self.dir, "main.xml", twice))
        self.assertEqual(out.count('<BehaviorTree ID="Lib"'), 1)
        self.assertEqual(out.count("<TreeNodesModel"), 1)
        self.assertEqual(out.count('<input_port name="n"'), 1)

    def test_conflicting_definitions_are_refused(self):
        write(self.dir, "lib/lib.xml", LIB)
        clash = MAIN.replace("</root>", '<BehaviorTree ID="Lib"><StopBoat/></BehaviorTree></root>')
        with self.assertRaisesRegex(ValueError, "defined twice"):
            ml.flatten(write(self.dir, "main.xml", clash))

    def test_include_cycle_is_refused(self):
        write(self.dir, "a.xml", '<root BTCPP_format="4"><include path="b.xml"/></root>')
        write(self.dir, "b.xml", '<root BTCPP_format="4"><include path="a.xml"/></root>')
        with self.assertRaisesRegex(ValueError, r"include cycle: a\.xml -> b\.xml -> a\.xml"):
            ml.flatten(os.path.join(self.dir, "a.xml"))

    def test_self_include_is_a_cycle(self):
        write(self.dir, "a.xml", '<root BTCPP_format="4"><include path="a.xml"/></root>')
        with self.assertRaisesRegex(ValueError, "include cycle"):
            ml.flatten(os.path.join(self.dir, "a.xml"))

    def test_missing_and_malformed_files_raise_valueerror(self):
        with self.assertRaisesRegex(ValueError, "cannot read"):
            ml.flatten(os.path.join(self.dir, "nope.xml"))
        write(self.dir, "bad.xml", "<root><oops></root>")
        with self.assertRaisesRegex(ValueError, "not well-formed"):
            ml.flatten(os.path.join(self.dir, "bad.xml"))
        write(self.dir, "inc.xml", '<root BTCPP_format="4"><include path="gone.xml"/></root>')
        with self.assertRaisesRegex(ValueError, "cannot read"):
            ml.flatten(os.path.join(self.dir, "inc.xml"))

    def test_subtree_entry_comes_from_a_lib_file(self):
        write(self.dir, "lib/lib.xml", LIB)
        cat = ml.catalogue(self.dir)
        (sub,) = cat["subtrees"]
        self.assertEqual((sub["id"], sub["label"], sub["desc"], sub["file"]),
                         ("Lib", "Lib", "a subtree", "lib.xml"))
        self.assertEqual(sub["ports"], [{"name": "n", "default": "8", "desc": "how many",
                                         "type": "int"}])
        self.assertTrue(sub["xml"].startswith('<BehaviorTree ID="Lib">'))
        self.assertNotIn("<!--", sub["xml"])

    def test_subtree_xml_carries_the_trees_it_depends_on(self):
        write(self.dir, "lib/two.xml",
              '<root BTCPP_format="4"><BehaviorTree ID="Top"><SubTree ID="Helper" '
              '_autoremap="true"/></BehaviorTree><BehaviorTree ID="Helper"><StopBoat/>'
              '</BehaviorTree><BehaviorTree ID="Unused"><StopBoat/></BehaviorTree></root>')
        (sub,) = ml.catalogue(self.dir)["subtrees"]
        self.assertEqual(sub["id"], "Top")
        self.assertIn('ID="Helper"', sub["xml"])
        self.assertNotIn('ID="Unused"', sub["xml"])

    def test_a_broken_lib_file_is_reported_not_listed(self):
        write(self.dir, "lib/bad.xml", "<root>")
        cat = ml.catalogue(self.dir)
        self.assertEqual(cat["subtrees"], [])
        self.assertTrue(has(cat["problems"], "lib/bad.xml"))


# ----------------------------------------------------------------------- save, list, load

GOOD = mission('<ReactiveSequence name="guard"><IsAutonomous/><Sequence name="go">'
               '<NavigateTo target="fix" lat="1.3" lon="103.8" tolerance="2"/>'
               '</Sequence></ReactiveSequence>')


class SaveListLoad(unittest.TestCase):
    def setUp(self):
        self._tmp = tempfile.TemporaryDirectory()
        self.dir = self._tmp.name
        self.saved = os.path.join(self.dir, "missions")
        self.addCleanup(self._tmp.cleanup)

    def test_round_trip(self):
        res = ml.save_mission(self.saved, "my_run-1", GOOD, CAT)
        self.assertTrue(res["ok"], res["message"])
        self.assertTrue(os.path.isfile(res["file"]))
        rows = ml.list_missions(self.dir, self.saved)
        self.assertEqual([(r["name"], r["source"], r["main_tree"]) for r in rows],
                         [("my_run-1", "saved", "Main")])
        loaded = ml.load_mission(self.dir, self.saved, "my_run-1", "saved")
        self.assertTrue(loaded["ok"], loaded["message"])
        self.assertEqual(check(loaded["xml"])["trees"], ["Main"])
        self.assertIn('target="fix"', loaded["xml"])

    def test_description_comes_from_the_first_mission_comment(self):
        text = "<!-- MISSION: drive\n   to the dock -->\n" + GOOD
        ml.save_mission(self.saved, "dock", text, CAT)
        self.assertEqual(ml.list_missions(self.dir, self.saved)[0]["description"],
                         "drive to the dock")
        ml.save_mission(self.saved, "plain", "<!-- just a note -->\n" + GOOD, CAT)
        rows = {r["name"]: r for r in ml.list_missions(self.dir, self.saved)}
        self.assertEqual(rows["plain"]["description"], "")
        self.assertEqual(ml.load_mission(self.dir, self.saved, "dock", "saved")["description"],
                         "drive to the dock")

    def test_prebuilt_listed_first_and_loaded_flattened(self):
        write(self.dir, "lib/lib.xml", LIB)
        write(self.dir, "zeta.xml", MAIN)
        ml.save_mission(self.saved, "alpha", GOOD, CAT)
        rows = ml.list_missions(self.dir, self.saved)
        self.assertEqual([(r["name"], r["source"]) for r in rows],
                         [("zeta", "prebuilt"), ("alpha", "saved")])
        self.assertEqual(rows[0]["description"], "runs lib")
        loaded = ml.load_mission(self.dir, self.saved, "zeta", "prebuilt")
        self.assertTrue(loaded["ok"])
        self.assertNotIn("<include", loaded["xml"])
        self.assertEqual(loaded["description"], "runs lib")

    def test_refuses_an_existing_file_unless_overwrite(self):
        self.assertTrue(ml.save_mission(self.saved, "a", GOOD, CAT)["ok"])
        again = ml.save_mission(self.saved, "a", GOOD, CAT)
        self.assertFalse(again["ok"])
        self.assertIn("already exists", again["message"])
        self.assertTrue(ml.save_mission(self.saved, "a", GOOD, CAT, overwrite=True)["ok"])
        self.assertEqual(os.listdir(self.saved), ["a.xml"])

    def test_refuses_unsafe_names_and_invalid_xml(self):
        for bad in ("../x", "a/b", "a.b", "", "x" * 65, None, "a b"):
            res = ml.save_mission(self.saved, bad, GOOD, CAT)
            self.assertFalse(res["ok"], repr(bad))
        res = ml.save_mission(self.saved, "broken", mission("<Nope/>"), CAT)
        self.assertFalse(res["ok"])
        self.assertIn("unknown node", res["message"])
        self.assertFalse(os.path.exists(self.saved))

    def test_load_refuses_bad_requests(self):
        for name, source in (("../x", "saved"), ("missing", "saved"), ("a", "elsewhere")):
            res = ml.load_mission(self.dir, self.saved, name, source)
            self.assertFalse(res["ok"])
            self.assertTrue(res["message"])

    def test_unreadable_files_are_listed_with_the_reason(self):
        write(self.saved, "bad.xml", "<root>")
        write(self.saved, "not safe.xml", GOOD)
        rows = ml.list_missions(self.dir, self.saved)
        self.assertEqual([r["name"] for r in rows], ["bad"])
        self.assertTrue(rows[0]["description"].startswith("unreadable"))

    def test_save_reports_warnings_in_the_message(self):
        text = mission('<Sequence><TaskAttempt token="TASK_NONE"><StopBoat/></TaskAttempt>'
                       '</Sequence>')
        res = ml.save_mission(self.saved, "warned", text, CAT)
        self.assertTrue(res["ok"])
        self.assertIn("1 warning", res["message"])


class SafeName(unittest.TestCase):
    def test_accepts_and_rejects(self):
        for ok in ("a", "Task1_Global-2", "x" * 64):
            self.assertEqual(ml.safe_name(ok), ok)
        for bad in ("", "x" * 65, "a.b", "a b", "a/b", "..", "a\n", "é", None, 3, "CON", "nul",
                    "Com1"):
            self.assertIsNone(ml.safe_name(bad), repr(bad))


# ---------------------------------------------------------------------------------- validate

def errs(body, extra=""):
    return check(mission(body, extra=extra))["errors"]


def warns(body, extra=""):
    return check(mission(body, extra=extra))["warnings"]


def valid(body, extra=""):
    return check(mission(body, extra=extra))["ok"]


SUB = '<BehaviorTree ID="Sub"><StopBoat/></BehaviorTree>'
ATTEMPT = '<TaskAttempt token="TASK_SAFE_PASSAGE">%s</TaskAttempt>'


class Validate(unittest.TestCase):
    def test_a_good_mission_is_clean(self):
        verdict = check(GOOD)
        self.assertEqual((verdict["ok"], verdict["errors"], verdict["warnings"]),
                         (True, [], []))
        self.assertEqual((verdict["main_tree"], verdict["trees"]), ("Main", ["Main"]))

    def test_malformed_xml(self):
        verdict = check("<root><BehaviorTree>")
        self.assertFalse(verdict["ok"])
        self.assertTrue(has(verdict["errors"], "not well-formed"))
        self.assertIsNone(verdict["main_tree"])

    def test_root_must_be_btcpp_4(self):
        for text in (GOOD.replace('"4"', '"3"'), GOOD.replace('BTCPP_format="4" ', "")):
            self.assertTrue(has(check(text)["errors"], "BTCPP_format"))
        self.assertTrue(has(check("<tree/>")["errors"], "must be <root>"))

    def test_main_tree_must_exist(self):
        self.assertTrue(has(check(GOOD.replace('main_tree_to_execute="Main"', ""))["errors"],
                            "no main_tree_to_execute"))
        gone = GOOD.replace('main_tree_to_execute="Main"', 'main_tree_to_execute="Gone"')
        self.assertTrue(has(check(gone)["errors"], "'Gone'"))

    def test_unknown_node(self):
        found = errs("<Sequence><Teleport/></Sequence>")
        self.assertTrue(has(found, "unknown node 'Teleport'"))
        self.assertTrue(has(found, "Main > Sequence > Teleport"))

    def test_generic_tag_form_is_resolved(self):
        self.assertTrue(valid('<Control ID="Sequence"><Action ID="StopBoat"/></Control>'))
        self.assertFalse(valid('<Action ID="Nope"/>'))

    def test_include_is_not_valid_in_a_saved_mission(self):
        self.assertTrue(has(errs("<StopBoat/>", extra='<include path="lib/x.xml"/>'), "<include>"))

    def test_subtree_to_an_undefined_tree(self):
        self.assertTrue(has(errs('<SubTree ID="Ghost" _autoremap="true"/>'), "'Ghost' is not defined"))
        self.assertTrue(has(errs('<SubTree _autoremap="true"/>'), "no ID"))

    def test_subtree_needs_autoremap(self):
        self.assertTrue(has(errs('<SubTree ID="Sub"/>', SUB), "_autoremap"))
        self.assertTrue(has(errs('<SubTree ID="Sub" _autoremap="false"/>', SUB), "_autoremap"))
        self.assertTrue(valid('<SubTree ID="Sub" _autoremap="true"/>', SUB))

    def test_subtree_tunables_are_typed_by_their_text(self):
        tuned = ('<BehaviorTree ID="Orbit"><StopBoat/></BehaviorTree><TreeNodesModel>'
                 '<SubTree ID="Orbit">'
                 '<input_port name="radius" default="6.0">a real: write it with a decimal point</input_port>'
                 '<input_port name="points" default="8">a whole number</input_port>'
                 '<input_port name="dir" default="cw">cw or ccw</input_port>'
                 '<input_port name="n_pts" default="">a whole number; empty = the nav_orbit_points '
                 'parameter</input_port>'
                 '<input_port name="n_rad" default="">a real: write it with a decimal point; empty '
                 '= the nav_orbit_radius_m parameter</input_port>'
                 '<input_port name="free" default="">no hint at all</input_port>'
                 '</SubTree></TreeNodesModel>')

        def call(attrs):
            return '<SubTree ID="Orbit" _autoremap="true" %s/>' % attrs

        def errors(attrs):
            return errs(call(attrs), tuned)

        self.assertEqual(errors('radius="6.0" points="8" dir="ccw" n_pts="" n_rad="" free=""'), [])
        self.assertEqual(errors('n_pts="12" n_rad="7.5" free="anything"'), [])
        self.assertTrue(has(errors('radius="6"'), "write 6.0"))
        self.assertTrue(has(errors('radius="abc"'), "decimal point"))
        self.assertTrue(has(errors('points="8.0"'), "write 8"))
        self.assertTrue(has(errors('points="eight"'), "whole number"))
        self.assertTrue(has(errors('n_rad="7"'), "write 7.0"))
        self.assertTrue(has(errors('n_pts="8.0"'), "write 8"))
        # empty is for the "use the nav_* parameter" tunables only; never for a direction
        for attrs in ('radius=""', 'points=""', 'dir=""'):
            self.assertTrue(has(errors(attrs), "must not be empty"), attrs)
        # a blackboard pointer is not a literal
        self.assertEqual(errors('radius="{r}" points="{p}"'), [])

    def test_shipped_tunables_follow_the_typing_rule(self):
        core = next(s for s in CAT["subtrees"] if s["id"] == "Task1Core")
        defaults = {p["name"]: p["default"] for p in core["ports"]}

        def verdict(**override):
            attrs = dict(defaults, **override)
            call = '<SubTree ID="Task1Core" _autoremap="true"%s/>' % "".join(
                ' %s="%s"' % kv for kv in sorted(attrs.items()))
            return check('<root BTCPP_format="4" main_tree_to_execute="Main"><BehaviorTree ID="Main">'
                         '%s</BehaviorTree>%s</root>' % (call, core["xml"]))["errors"]

        self.assertEqual(verdict(), [])
        self.assertTrue(has(verdict(orbit_radius_m="6"), "write 6.0"))
        self.assertTrue(has(verdict(orbit_points="8.0"), "write 8"))
        self.assertTrue(has(verdict(entry_direction=""), "must not be empty"))

    def test_tree_named_as_a_tag_is_a_subtree_reference(self):
        found = errs("<Sub/>", SUB)
        self.assertTrue(has(found, "_autoremap"))
        self.assertFalse(has(found, "unknown node"))

    def test_subtree_recursion(self):
        trees = ('<BehaviorTree ID="A"><SubTree ID="B" _autoremap="true"/></BehaviorTree>'
                 '<BehaviorTree ID="B"><SubTree ID="A" _autoremap="true"/></BehaviorTree>')
        self.assertTrue(has(errs('<SubTree ID="A" _autoremap="true"/>', trees),
                            "SubTree recursion: A -> B -> A"))
        self.assertTrue(has(errs('<SubTree ID="Main" _autoremap="true"/>'),
                            "SubTree recursion: Main -> Main"))

    def test_child_counts(self):
        self.assertTrue(has(errs("<Inverter/>"), "exactly one child"))
        self.assertTrue(has(errs("<Inverter><StopBoat/><StopBoat/></Inverter>"), "exactly one child"))
        self.assertTrue(has(errs("<StopBoat><StopBoat/></StopBoat>"), "leaf"))
        self.assertTrue(has(errs("<IfThenElse><StopBoat/></IfThenElse>"), "2 to 3 children"))
        self.assertTrue(has(warns("<Sequence><Sequence/></Sequence>"), "no children"))
        empty = '<root BTCPP_format="4" main_tree_to_execute="M"><BehaviorTree ID="M"/></root>'
        self.assertTrue(has(check(empty)["errors"], "exactly one root"))

    def test_unknown_port_is_a_warning_but_name_and_underscore_attrs_are_not(self):
        verdict = check(mission('<StopBoat flavour="x"/>'))
        self.assertTrue(verdict["ok"])
        self.assertTrue(has(verdict["warnings"], "unknown port 'flavour'"))
        self.assertEqual(warns('<StopBoat name="halt" _skipIf="false"/>'), [])

    def test_required_port_missing(self):
        self.assertTrue(has(errs("<Timeout><StopBoat/></Timeout>"), "required port 'msec'"))
        self.assertTrue(has(errs("<TaskAttempt><StopBoat/></TaskAttempt>"), "required port 'token'"))
        self.assertTrue(has(errs("<SetTier/>"), "required port 'tier'"))
        self.assertTrue(has(errs("<RetryUntilSuccessful><StopBoat/></RetryUntilSuccessful>"),
                            "'num_attempts'"))

    def test_optional_ports_are_not_required(self):
        self.assertEqual(errs('<CircleBuoy anchor="entry"/>'), [])
        self.assertEqual(errs("<NavigateTo/>"), [])

    def test_enum_value_must_be_a_choice(self):
        self.assertTrue(has(errs('<TaskAttempt token="TASK_FLY"><StopBoat/></TaskAttempt>'),
                            "'TASK_FLY'"))
        self.assertTrue(has(errs('<SetTier tier="hard"/>'), "core, advanced"))
        self.assertTrue(has(errs('<CircleBuoy direction="left"/>'), "cw, ccw"))
        self.assertTrue(valid('<SetTier tier="{t}"/>'))

    def test_numeric_ports_must_parse_unless_blackboard(self):
        self.assertTrue(has(errs('<NavigateTo tolerance="two"/>'), "a number"))
        self.assertTrue(has(errs('<NavigateTo tolerance="nan"/>'), "a number"))
        self.assertTrue(has(errs('<CircleBuoy points="8.5"/>'), "a whole number"))
        self.assertTrue(has(errs('<NavigateTo avoid="maybe"/>'), "true or false"))
        self.assertTrue(valid('<NavigateTo tolerance="{tol}" avoid="{a}"/>'))
        self.assertTrue(valid('<NavigateTo tolerance="2" avoid="TRUE"/>'))

    def test_blackboard_ports_need_a_blackboard_entry(self):
        self.assertTrue(has(errs('<NavigateTo target="port" goal="5"/>'), "blackboard entry"))
        self.assertTrue(has(errs('<GateWaypoint out="x"/>'), "blackboard entry"))
        self.assertTrue(valid('<Sequence><GateWaypoint out="{goal}"/>'
                              '<NavigateTo target="port" goal="{goal}"/></Sequence>'))

    def test_target_port_needs_a_goal(self):
        self.assertTrue(has(errs('<NavigateTo target="port"/>'), "needs goal"))

    def test_gps_point_needs_a_real_position(self):
        def nav(**kw):
            return errs('<NavigateTo target="fix"%s/>' % "".join(' %s="%s"' % kv for kv in kw.items()))
        self.assertTrue(has(nav(), "needs lat"))
        self.assertTrue(has(nav(lat="1.3"), "needs lon"))
        self.assertTrue(has(nav(lat="91", lon="10"), "outside"))
        self.assertTrue(has(nav(lat="10", lon="-181"), "outside"))
        self.assertTrue(has(nav(lat="0", lon="0"), "both 0"))
        self.assertEqual(nav(lat="0", lon="103.8"), [])
        self.assertEqual(nav(lat="{la}", lon="{lo}"), [])
        self.assertTrue(has(errs('<CircleBuoy anchor="fix" lat="0" lon="0" radius="5"/>'), "both 0"))
        self.assertFalse(has(errs('<NavigateTo target="home"/>'), "lat"))

    def test_task_attempt_without_wait_for_run_start_warns(self):
        verdict = check(mission(ATTEMPT % "<StopBoat/>"))
        self.assertTrue(verdict["ok"])
        self.assertTrue(has(verdict["warnings"], "starts without RoboCommand"))
        self.assertEqual(warns("<Sequence><WaitForRunStart/>%s</Sequence>" % (ATTEMPT % "<StopBoat/>")),
                         [])
        self.assertEqual(warns("<StopBoat/>"), [])

    def test_wait_for_run_start_in_a_subtree_counts(self):
        start = '<BehaviorTree ID="Start"><WaitForRunStart/></BehaviorTree>'
        body = '<Sequence><SubTree ID="Start" _autoremap="true"/>%s</Sequence>' % (ATTEMPT % "<StopBoat/>")
        self.assertEqual(warns(body, start), [])

    def test_nested_task_attempt_warns(self):
        inner = '<TaskAttempt token="TASK_COORDINATED_LOGISTICS"><StopBoat/></TaskAttempt>'
        body = "<Sequence><WaitForRunStart/>%s</Sequence>" % (ATTEMPT % inner)
        self.assertTrue(valid(body))
        self.assertTrue(has(warns(body), "inside another TaskAttempt"))

    def test_nested_task_attempt_through_a_subtree_warns(self):
        sub = ('<BehaviorTree ID="T3"><TaskAttempt token="TASK_COORDINATED_LOGISTICS">'
               '<StopBoat/></TaskAttempt></BehaviorTree>')
        body = ("<Sequence><WaitForRunStart/>%s</Sequence>"
                % (ATTEMPT % '<SubTree ID="T3" _autoremap="true"/>'))
        self.assertTrue(has(warns(body, sub), "inside another"))

    def test_subtree_attributes_are_checked_against_the_declared_model(self):
        model = ('<TreeNodesModel><SubTree ID="Sub"><input_port name="n" default="1"/>'
                 '</SubTree></TreeNodesModel>')
        self.assertEqual(warns('<SubTree ID="Sub" _autoremap="true" n="3"/>', SUB + model), [])
        self.assertTrue(has(warns('<SubTree ID="Sub" _autoremap="true" m="3"/>', SUB + model),
                            "'m' is not a tunable"))

    def test_duplicate_tree_ids_and_missing_ids(self):
        self.assertTrue(has(errs("<StopBoat/>", '<BehaviorTree ID="Main"><StopBoat/></BehaviorTree>'),
                            "more than once"))
        self.assertTrue(has(errs("<StopBoat/>", "<BehaviorTree><StopBoat/></BehaviorTree>"),
                            "has no ID"))

    def test_leading_whitespace_and_bom_are_tolerated(self):
        self.assertTrue(check(chr(0xFEFF) + " \n" + GOOD)["ok"])


if __name__ == "__main__":
    unittest.main()
