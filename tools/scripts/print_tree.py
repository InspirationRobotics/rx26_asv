#!/usr/bin/env python3
"""print_tree — render and check the mission behaviour tree. No boat, no ROS.

    python3 tools/scripts/print_tree.py                  # ascii, for review
    python3 tools/scripts/print_tree.py --xml            # BehaviorTree.CPP 4
    python3 tools/scripts/print_tree.py --dot | dot -Tpng -o tree.png
    python3 tools/scripts/print_tree.py --check          # exit 1 on a defect
    python3 tools/scripts/print_tree.py --check --live   # ...also check the boat
    python3 tools/scripts/print_tree.py --fail safe_passage   # what if it fails?

Stdlib only in the default paths, so this runs on the Windows laptop, in CI, and
on the Jetson with nothing sourced. `--live` is the exception: it asks the ROS
graph whether each mission's action server is actually there, so it needs ROS
and only makes sense on the boat.

WHAT --check IS FOR. Every problem it reports is one that otherwise shows up as
a mission that silently never runs, on the water, once — a typo'd action name
(the leaf waits forever for a server that will never appear, which looks exactly
like a slow mission), or a mission whose failure ends the whole run. Scoring is
per task, so the second one is the expensive one.
"""
import argparse
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                "..", "..", "crusader_mission"))
from crusader_mission.tree import mission_spec, render      # noqa: E402


def check_live(root):
    """Do the servers this tree names actually exist? Returns problem strings.

    Deliberately reports "could not look" as a PROBLEM rather than silently
    passing: a --live check that quietly degrades into a structural one is worse
    than no --live check, because it reads as proof the servers are up.
    """
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError as e:
        return ["--live needs ROS: {}. Source the workspace, or drop --live "
                "for the structural check only.".format(e)]

    wanted = {n.attrs["action"] for n, _d, _p in mission_spec.walk(root)
              if n.kind == mission_spec.MISSION and n.attrs.get("action")}
    problems = []
    rclpy.init()
    try:
        probe = Node("print_tree_probe")
        # Spin briefly: discovery is asynchronous, and asking the graph the
        # instant after init reliably reports an empty world. A zero-wait check
        # here would fail every server every time.
        end = probe.get_clock().now().nanoseconds + 2_000_000_000
        found = set()
        while probe.get_clock().now().nanoseconds < end:
            rclpy.spin_once(probe, timeout_sec=0.1)
            names = {n for n, _t in probe.get_topic_names_and_types()}
            found = {a for a in wanted
                     if any(n.startswith(a + "/_action/") for n in names)}
            if found == wanted:
                break
        for a in sorted(wanted - found):
            problems.append("no action server on {} — the leaf naming it would "
                            "wait forever".format(a))
        probe.destroy_node()
    finally:
        rclpy.try_shutdown()
    return problems


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--ascii", action="store_true", help="tree as text (default)")
    g.add_argument("--xml", action="store_true", help="BehaviorTree.CPP 4 XML")
    g.add_argument("--dot", action="store_true", help="graphviz")
    ap.add_argument("--check", action="store_true",
                    help="report structural defects; exit 1 if any")
    ap.add_argument("--live", action="store_true",
                    help="with --check: also verify the servers exist (needs ROS)")
    ap.add_argument("-o", "--out", help="write to a file instead of stdout")
    ap.add_argument("--missions", nargs="*", default=None,
                    help="only these missions (default: all)")
    ap.add_argument("--fail", nargs="*", metavar="LEAF", default=None,
                    help="simulate a tick with these leaves returning FAILURE, "
                         "and show what the run does. The question this answers "
                         "is 'if this mission fails, does the rest of the run "
                         "still happen?' — scoring is per task, so it must.")
    a = ap.parse_args()

    root = mission_spec.build(a.missions)

    if a.fail is not None:
        names = {n.name for n, _d, _p in mission_spec.walk(root)
                 if n.kind in mission_spec.LEAVES}
        unknown = sorted(set(a.fail) - names)
        if unknown:
            print("no such leaf: {} — leaves are {}".format(unknown,
                                                            sorted(names)))
            return 1
        trace = []
        result = mission_spec.simulate(
            root, {n: mission_spec.FAILURE for n in a.fail}, trace=trace)
        print("failing: {}".format(sorted(a.fail) or "(nothing)"))
        for depth, name, r in trace:
            print("  {}{:<18} {}".format("  " * depth, name, r))
        print()
        print("run -> {}".format(result))
        by_kind = {n.name: n.kind for n, _d, _p in mission_spec.walk(root)}
        missions = [n for n, k in by_kind.items() if k == mission_spec.MISSION]
        stopped = [m for m in missions if not any(t[1] == m for t in trace)]
        if not stopped:
            print("every mission was still attempted.")
            return 0

        # A failing GUARD stopping the run is the design, not a defect: if the
        # boat is not autonomous, no mission should be ticking. Only a failing
        # MISSION that costs another mission is the expensive kind, because
        # scoring is per task.
        guards = [n for n in a.fail if by_kind.get(n) == mission_spec.CONDITION]
        if guards and not [n for n in a.fail
                           if by_kind.get(n) == mission_spec.MISSION]:
            print("not reached: {} — expected: the guard {} is what decides "
                  "whether any mission runs at all".format(stopped,
                                                           sorted(guards)))
            return 0
        print("NOT REACHED: {} — a failing MISSION ended the run early, and "
              "scoring is per task, so this costs every task after "
              "it".format(stopped))
        return 1

    if a.check:
        problems = mission_spec.check(root)
        if a.live:
            problems += check_live(root)
        if problems:
            print("==== TREE CHECK: FAIL ({} problem(s)) ====".format(
                len(problems)))
            for p in problems:
                print("  [FAIL] " + p)
            return 1
        n = sum(1 for x, _d, _p in mission_spec.walk(root)
                if x.kind == mission_spec.MISSION)
        print("==== TREE CHECK: PASS ({} mission(s){}) ====".format(
            n, ", servers live" if a.live else ""))
        return 0

    if a.xml:
        text = render.xml(root)
    elif a.dot:
        text = render.dot(root)
    else:
        # Ask the real stream what it can encode. A Windows console at cp1252
        # turns the box-drawing characters into mojibake, and a review artefact
        # nobody can read is a review that does not happen.
        enc = (getattr(sys.stdout, "encoding", None) or "").lower()
        unicode_ok = bool(a.out) or enc.startswith("utf")
        text = render.ascii(root, unicode_ok=unicode_ok)

    if a.out:
        with open(a.out, "w", encoding="utf-8", newline="\n") as f:
            f.write(text + "\n")
        print("wrote {}".format(a.out))
    else:
        print(text)
    return 0


if __name__ == "__main__":
    sys.exit(main())
