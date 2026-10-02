"""nav_goal — the operator's "Send goal" for a NON-Task-1 target: arm, GUIDED, go to a point.

    python3 -m crusader_sim.nav_goal --course open_water_platform --east-m 40 [--north-m 0]
        [--timeout-s 300] [--watch plat ...] [--arrive-m 3.0] [--min-clear-m 0.73]

Runs in: the crsd-sim container, after gz_rig_up.sh — the same place as task1_goal, and
it does what task1_goal does before a mission (operator_tools.arm over MAVLink 14550,
operator_tools.set_mode GUIDED through /crsd/set_mode, then the SafePassage goal to
bt_runner), with two differences: the goal is tier 0 at an approach point you give as
metres EAST/NORTH of the course origin (course.enu_to_latlon, the SITL scale), and the
referee is task1_judge's clearance arithmetic only, on GROUND TRUTH
(/sim/crusader/odometry, never anything the boat's stack computes).

Meant for a tree that is one planned leg to the approach point (nav_test_line.xml), so
the boat's path is whatever the planner made of the hazards. Prints, one line each:

    [result] outcome 0 (SUCCESS)  <detail>  87 s
    [clear] plat centre-to-surface 1.23 m (hull 0.95 m)      one per --watch element
    [contact] none                                           elements the hull touched
    [arrive] final distance to goal 1.4 m
    [score] PASS | FAIL: <why>

--watch takes any element task1_judge measures: a buoy, a platform, a dock or a launch pad.
[score] is PASS when the tree reports SUCCESS, the boat ended within --arrive-m of the
goal and touched nothing, and (only when --min-clear-m is given) every watched element
kept that centre-to-surface distance. Exit status 0 = PASS, 1 = anything else, 2 = could
not start (no element, no arming, no GUIDED, no server).
"""
import argparse
import sys

from crusader_sim import course as C
from crusader_sim.operator_tools import arm, set_mode
from crusader_sim.task1_judge import Task1Judge

# rclpy and the message types are imported inside main(): everything above and the pure
# helpers below also run on a plain python3 (the unit test, the WSL host)

DEFAULT_ARRIVE_M = 3.0     # nav_test_line.xml's NavigateTo tolerance is 2.0 m


# ------------------------------------------------------------------ pure helpers

def goal_latlon(course, east_m, north_m):
    """(lat, lon) of the point east_m / north_m from the course origin, at SITL scale."""
    return C.enu_to_latlon(east_m, north_m, course["origin"])


def unknown_watch(judge, names):
    """The --watch names task1_judge has no shape for (it knows every buoy, platform,
    dock and launch pad of the course)."""
    return [n for n in names if n not in judge.shapes]


def _num(v, decimals=2):
    return "n/a" if v is None else f"{v:.{decimals}f}"


def watched_clearance(verdict, names):
    """{name: (centre_m, hull_m)} for each watched element, from a task1_judge verdict.
    (None, None) when no ground-truth pose ever arrived, and hull None without a yaw."""
    by = verdict["min_clearance"]["by_object"]
    return {n: (by.get(n, {}).get("centre_m"), by.get(n, {}).get("hull_m")) for n in names}


def clear_line(name, centre_m, hull_m):
    return f"[clear] {name} centre-to-surface {_num(centre_m)} m (hull {_num(hull_m)} m)"


def touching(verdict, clearance):
    """What the hull touched: task1_judge's buoy contacts, plus any watched element whose
    measured hull gap (or, without a yaw, centre distance) reached zero."""
    hit = set(verdict["contacts"])
    for name, (centre, hull) in clearance.items():
        if (hull if hull is not None else centre) == 0.0:
            hit.add(name)
    return sorted(hit)


def final_distance(judge, east_m, north_m):
    """Distance from the boat's last ground-truth position to the goal, in the world's ENU
    frame (the course origin is the world origin); None before any pose arrived."""
    if judge.prev is None:
        return None
    return ((judge.prev[0] - east_m) ** 2 + (judge.prev[1] - north_m) ** 2) ** 0.5


def score(outcome_ok, dist_m, arrive_m, clearance, min_clear_m, hit):
    """(passed, [why not]). Only --min-clear-m makes clearance a criterion."""
    why = []
    if not outcome_ok:
        why.append("tree outcome is not SUCCESS")
    if dist_m is None:
        why.append("no ground-truth pose, so no arrival distance")
    elif dist_m > arrive_m:
        why.append(f"final distance {dist_m:.1f} m is beyond {arrive_m:g} m")
    if hit:
        why.append("touched " + ", ".join(hit))
    if min_clear_m is not None:
        for name, (centre, _hull) in clearance.items():
            if centre is None or centre < min_clear_m:
                why.append(f"{name} clearance {_num(centre)} m is below {min_clear_m:g} m")
    return not why, why


def feedback_printer():
    """One line per phase, per 5 % of progress or per new warning: the tree sends
    feedback every tick."""
    last = [None]

    def fb(msg):
        f = msg.feedback
        key = (f.phase, int(f.progress * 20), f.warning)
        if key != last[0]:
            print(f"[tree] {f.phase:10s} {f.progress * 100:5.1f}%  {f.warning}".rstrip(), flush=True)
            last[0] = key
    return fb


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--course", required=True)
    ap.add_argument("--east-m", type=float, required=True,
                    help="goal, metres east of the course origin")
    ap.add_argument("--north-m", type=float, default=0.0,
                    help="goal, metres north of the course origin (default 0)")
    ap.add_argument("--timeout-s", type=float, default=300.0)
    ap.add_argument("--watch", nargs="*", default=[], metavar="ELEMENT",
                    help="course elements to report the minimum clearance of")
    ap.add_argument("--arrive-m", type=float, default=DEFAULT_ARRIVE_M,
                    help="[score] needs the boat this close to the goal at the end")
    ap.add_argument("--min-clear-m", type=float, default=None,
                    help="[score] also needs every watched centre-to-surface at least this")
    return ap.parse_args(argv)


# ------------------------------------------------------------------ the operator

def main(argv=None):
    a = parse_args(argv)
    course = C.load(a.course)
    judge = Task1Judge(course, echo=False)
    bad = unknown_watch(judge, a.watch)
    if bad:
        print(f"[nav_goal] --watch: no such element {bad}; the judge measures {sorted(judge.shapes)}")
        return 2
    lat, lon = goal_latlon(course, a.east_m, a.north_m)

    ok, why = arm()
    print(f"[operator] {why}", flush=True)
    if not ok:
        return 2

    import rclpy
    from crusader_msgs.action import SafePassage
    from crusader_sim.task1_goal import Operator

    rclpy.init()
    op = Operator(judge)
    if not set_mode(op, op.mode_pub, "GUIDED"):
        print("[operator] GUIDED never confirmed on /crsd/fcu_status — is telemetry_bridge up?")
        return 2
    print("[operator] mode GUIDED (confirmed by the autopilot)", flush=True)
    if not op.client.wait_for_server(timeout_sec=20.0):
        print("[operator] no /crsd/safe_passage server (is bt_runner up?)")
        return 2
    goal = SafePassage.Goal(tier=SafePassage.Goal.TIER_CORE, approach_latitude=lat,
                            approach_longitude=lon, orbit_radius_m=0.0,
                            timeout_s=float(a.timeout_s))
    print(f"[operator] goal: tier 0, approach {lat:.7f} {lon:.7f} "
          f"({a.east_m:g} m east, {a.north_m:g} m north of the origin)", flush=True)

    fut = op.client.send_goal_async(goal, feedback_callback=feedback_printer())
    rclpy.spin_until_future_complete(op, fut)
    handle = fut.result()
    if not handle.accepted:
        print("[operator] goal REJECTED")
        return 1
    res_fut = handle.get_result_async()
    rclpy.spin_until_future_complete(op, res_fut)
    r = res_fut.result().result
    outcomes = {getattr(SafePassage.Result, k): k[len("OUTCOME_"):]
                for k in dir(SafePassage.Result) if k.startswith("OUTCOME_")}
    print(f"[result] outcome {r.outcome} ({outcomes.get(r.outcome, '?')})  {r.detail}  "
          f"{r.elapsed_s:.0f} s", flush=True)

    verdict = judge.verdict()
    clearance = watched_clearance(verdict, a.watch)
    for name, (centre, hull) in clearance.items():
        print(clear_line(name, centre, hull), flush=True)
    hit = touching(verdict, clearance)
    print(f"[contact] {', '.join(hit) if hit else 'none'}", flush=True)
    dist = final_distance(judge, a.east_m, a.north_m)
    print(f"[arrive] final distance to goal {_num(dist, 1)} m", flush=True)
    passed, reasons = score(r.outcome == SafePassage.Result.OUTCOME_SUCCESS, dist, a.arrive_m,
                            clearance, a.min_clear_m, hit)
    print("[score] PASS" if passed else "[score] FAIL: " + "; ".join(reasons), flush=True)
    op.destroy_node()
    rclpy.shutdown()
    return 0 if passed else 1


if __name__ == "__main__":
    sys.exit(main())
